#!/usr/bin/env python3
"""
Guide-glasses PC Agent.

Core loop:
- Pull snapshot from ESP32 every ~1 s.
- Run YOLO and track obstacles across frames to avoid flickering.
- Poll /imu to detect completed body turns.
- Offload TTS + audio POST to a single background worker so detection stays fast.
"""

import argparse
import asyncio
import io
import json
import os
import queue
import socket
import subprocess
import sys
import tempfile
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

# Load a local .env file (if present) so secrets like DASHSCOPE_API_KEY
# survive across shell restarts without being hard-coded.
# Check the project root first, then fall back to the script directory.
_SCRIPT_DIR = Path(__file__).parent
_PROJECT_ROOT = _SCRIPT_DIR.parent
_ENV_PATH = _PROJECT_ROOT / ".env"
if not _ENV_PATH.exists():
    _ENV_PATH = _SCRIPT_DIR / ".env"
if _ENV_PATH.exists():
    with _ENV_PATH.open("r", encoding="utf-8") as _env_f:
        for _env_line in _env_f:
            _env_line = _env_line.strip()
            if not _env_line or _env_line.startswith("#"):
                continue
            if "=" in _env_line:
                _k, _v = _env_line.split("=", 1)
                if _k not in os.environ:
                    os.environ[_k] = _v

import edge_tts
import miniaudio
import requests
from PIL import Image
from ultralytics import YOLO
from zeroconf import Zeroconf

from dashboard import start_dashboard

# OpenCV color fallback for traffic lights (only imported when needed).
import cv2  # noqa: E402
import numpy as np  # noqa: E402

# Offline TTS fallback (Windows SAPI5).
try:
    import pyttsx3
    _HAS_PYTTSX3 = True
except Exception:
    _HAS_PYTTSX3 = False

# Force offline TTS for stable demos without network.
_OFFLINE_TTS_ONLY = False

ESP32_HOST = "192.168.198.181"


def resolve_host(host: str, timeout: float = 2.0) -> str:
    if not host.endswith(".local"):
        try:
            return socket.gethostbyname(host)
        except Exception:
            return host
    try:
        zc = Zeroconf()
        info = zc.get_service_info("_http._tcp.local.", f"{host}.", timeout=int(timeout * 1000))
        zc.close()
        if info and info.parsed_addresses():
            return info.parsed_addresses()[0]
    except Exception as exc:
        print(f"[mdns] failed: {exc}")
    return host


def make_urls(host: str) -> tuple[str, str, str, str]:
    base = f"http://{host}"
    return f"{base}/snapshot.jpg", f"{base}/status", f"{base}/play", f"{base}/imu"


SNAPSHOT_URL = ""
STATUS_URL = ""
PLAY_URL = ""
IMU_URL = ""

MODEL_PATH = r"D:\YOLO_CUSTOM\runs\guide_glasses\weights\best.pt"
COCO_MODEL_PATH = "yolov8n.pt"  # Ultralytics COCO pretrained model
COCO_CHAIR_CLASS_ID = 56          # COCO class index for chair
COCO_CHAIR_CONF = 0.40

# Dedicated traffic-light model found in external project.
TRAFFIC_MODEL_PATH = r"D:\李承城大学\bro fan glass\trafficlight.pt"
TRAFFIC_CLASS_MAP = {
    "stop": "red_light",
    "countdown_stop": "red_light",
    "go": "green_light",
    "countdown_go": "green_light",
    # "crossing", "blank", "countdown_blank" are ignored.
}
VOICE = "zh-CN-XiaoxiaoNeural"
TTS_RATE = "+0%"
TTS_VOLUME = "+0%"

SAMPLE_RATE = 16000
MAX_PCM_BYTES = 160 * 1024

_EDGE_TTS_PROXY = (
    os.getenv("EDGE_TTS_PROXY") or os.getenv("HTTPS_PROXY") or os.getenv("HTTP_PROXY")
)

# DashScope (Qwen / CosyVoice) TTS configuration.
_DASHSCOPE_API_KEY = os.getenv("DASHSCOPE_API_KEY")
_DASHSCOPE_TTS_ENABLED = False
_DASHSCOPE_MODEL = os.getenv("DASHSCOPE_TTS_MODEL") or "cosyvoice-v3-flash"
_DASHSCOPE_VOICE = os.getenv("DASHSCOPE_TTS_VOICE") or "longanhuan"

ZONE_LEFT = 0.33
ZONE_RIGHT = 0.67
CONF_THRESHOLD = 0.45
DIST_NEAR_Y = 0.65
DIST_MID_Y = 0.35

# Filter out obvious bogus boxes (full-frame glitches / tiny noise).
MIN_BOX_AREA_RATIO = 0.005
MAX_BOX_AREA_RATIO = 0.60

# Traffic-light color fallback is disabled by default (too noisy indoors).
TRAFFIC_FALLBACK_ENABLED = False

# Debug: save snapshots/masks when traffic fallback runs.
_DEBUG_SAVE = False
_DEBUG_DIR = "debug_imgs"

# Class-specific confidence thresholds to suppress common false positives.
CLASS_CONF_THRESHOLDS = {
    "ashcan": 0.75,
    "tree": 0.60,
    "dog": 0.60,
    "person": 0.30,  # lower so partial/edge people are still detected before camera is re-oriented
    "car": 0.75,
    "bus": 0.75,
    "truck": 0.75,
    "motorcycle": 0.60,
    "bicycle": 0.60,
    "tricycle": 0.60,
    "sign": 0.75,
    # Lower thresholds for traffic lights: they are small/distant and critical.
    "red_light": 0.25,
    "green_light": 0.25,
}

# Tracking parameters.
TRACK_MIN_SEEN = 2          # frames before an obstacle is considered stable
TRACK_MAX_MISSING = 3       # frames after which a tracked obstacle is forgotten
TRACK_COOLDOWN_S = 6.0      # min seconds before re-announcing the same obstacle

# Posture alerting thresholds.
POSTURE_PITCH_THRESHOLD = 30.0   # degrees
POSTURE_ROLL_THRESHOLD = 20.0    # degrees
POSTURE_COOLDOWN_S = 10.0        # min seconds between posture reminders
POSTURE_CALIBRATE_SAMPLES = 5    # frames used for upright zero calibration
POSTURE_HOLD_S = 1.0             # seconds the posture must stay bad before alerting
POSTURE_DISABLE = False
POSTURE_AUTO_CALIBRATE = False   # wait for manual trigger by default
POSTURE_TRIGGER_FILE = "posture_calibrate.txt"
_last_posture_time = 0.0
_posture_pitch_offset = 0.0
_posture_roll_offset = 0.0
_posture_calibrated = False
_posture_history: List[Tuple[float, float, float]] = []  # (timestamp, pitch, roll)

# TTS worker queue.
tts_queue: queue.Queue = queue.Queue(maxsize=10)
_tts_pending: set = set()
_TTS_PENDING_LOCK = threading.Lock()

# Traffic-light state machine for low-latency red/green announcements.
_TRAFFIC_CLASSES = {"red_light", "green_light"}
_TRAFFIC_COOLDOWN_S = 2.0
_TRAFFIC_DEBOUNCE_FRAMES = 1
_last_traffic_state: Optional[str] = None
_last_traffic_time: float = 0.0
_traffic_state_counts: Dict[str, int] = {}

# Digital zoom shared with the web dashboard.
_zoom_factor = 1.0
_ZOOM_LOCK = threading.Lock()

# Dashboard shared state.
_dashboard_state = {
    "frame": None,          # PIL RGB image (zoomed, as YOLO sees it)
    "detections": [],       # list[Detection]
    "detections_json": [],
    "imu": {},
    "status_text": "",
    "tts_last": "",
    "zoom": 1.0,
    "esp32_host": "",
    "last_update": 0.0,
}
_DASHBOARD_LOCK = threading.Lock()



@dataclass
class Detection:
    cls: str
    conf: float
    zone: str
    distance: str
    priority: int = 0
    # Normalized bbox [x1, y1, x2, y2] in the image passed to the model.
    bbox: Optional[Tuple[float, float, float, float]] = None


@dataclass
class TrackedObstacle:
    cls: str
    zone: str
    distance: str
    conf: float
    seen: int = 0
    missing: int = 0
    last_announced: float = field(default_factory=lambda: -1e9)


CLASS_NAMES_ZH = {
    "car": "汽车",
    "dog": "狗",
    "person": "行人",
    "bus": "公交车",
    "truck": "卡车",
    "green_light": "绿灯",
    "pole": "电线杆",
    "sign": "交通标志",
    "warning_column": "警示柱",
    "tree": "树木",
    "red_light": "红灯",
    "fire_hydrant": "消防栓",
    "motorcycle": "摩托车",
    "ashcan": "垃圾桶",
    "bicycle": "自行车",
    "reflective_cone": "反光锥",
    "blind_road": "盲道",
    "crosswalk": "斑马线",
    "tricycle": "三轮车",
    "roadblock": "路障",
    "chair": "椅子",
}


def fetch_url(url: str, timeout: float = 8.0) -> requests.Response:
    return requests.get(url, timeout=timeout)


def fetch_snapshot(timeout: float = 12.0, retries: int = 3) -> Image.Image:
    last_exc: Optional[Exception] = None
    for attempt in range(1, retries + 1):
        try:
            resp = requests.get(SNAPSHOT_URL, timeout=timeout)
            resp.raise_for_status()
            return Image.open(io.BytesIO(resp.content)).convert("RGB")
        except Exception as exc:
            last_exc = exc
            print(f"[snapshot] attempt {attempt}/{retries} failed: {exc}")
            if attempt < retries:
                time.sleep(0.5)
    raise last_exc or RuntimeError("fetch_snapshot failed")


def fetch_imu(timeout: float = 5.0) -> dict:
    try:
        resp = requests.get(IMU_URL, timeout=timeout)
        resp.raise_for_status()
        return resp.json()
    except Exception as exc:
        print(f"[imu] skipped: {exc}")
        return {}


def get_zoom_factor() -> float:
    with _ZOOM_LOCK:
        return max(1.0, min(_zoom_factor, 4.0))


def set_zoom_factor(factor: float) -> float:
    global _zoom_factor
    factor = max(1.0, min(float(factor), 4.0))
    with _ZOOM_LOCK:
        _zoom_factor = factor
    print(f"[zoom] set to {factor:.2f}x")
    return factor


def apply_zoom(image: Image.Image, factor: Optional[float] = None) -> Image.Image:
    """Center-crop and upscale to simulate phone-camera pinch zoom."""
    if factor is None:
        factor = get_zoom_factor()
    if factor <= 1.0:
        return image
    w, h = image.size
    crop_w = int(w / factor)
    crop_h = int(h / factor)
    crop_w = max(1, min(crop_w, w))
    crop_h = max(1, min(crop_h, h))
    left = (w - crop_w) // 2
    top = (h - crop_h) // 2
    return image.crop((left, top, left + crop_w, top + crop_h)).resize((w, h), Image.Resampling.LANCZOS)


def _detection_to_dict(det: Detection) -> dict:
    return {
        "cls": det.cls,
        "conf": round(det.conf, 2),
        "zone": det.zone,
        "distance": det.distance,
        "bbox": det.bbox,
    }


def update_dashboard_state(frame: Image.Image, detections: List[Detection], imu: dict, status_text: str = "", esp32_host: str = "") -> None:
    with _DASHBOARD_LOCK:
        _dashboard_state["frame"] = frame.copy()
        _dashboard_state["detections"] = list(detections)
        _dashboard_state["detections_json"] = [_detection_to_dict(d) for d in detections]
        _dashboard_state["imu"] = dict(imu)
        _dashboard_state["status_text"] = status_text
        _dashboard_state["zoom"] = get_zoom_factor()
        if esp32_host:
            _dashboard_state["esp32_host"] = esp32_host
        _dashboard_state["last_update"] = time.time()


def zone_name(x_center_norm: float) -> str:
    if x_center_norm < ZONE_LEFT:
        return "left"
    if x_center_norm > ZONE_RIGHT:
        return "right"
    return "center"


def zh_zone_name(zone: str) -> str:
    return {"left": "左侧", "center": "正前方", "right": "右侧"}.get(zone, "前方")


def zh_distance_name(distance: str) -> str:
    return {"near": "近处", "mid": "中距离", "far": "远处"}.get(distance, "前方")


def distance_name(y_bottom_norm: float) -> Tuple[str, int]:
    if y_bottom_norm >= DIST_NEAR_Y:
        return "近处", 0
    if y_bottom_norm >= DIST_MID_Y:
        return "中距离", 1
    return "远处", 2


def zone_priority(zone: str) -> int:
    return {"center": 0, "left": 1, "right": 1}.get(zone, 1)


def zh_class_name(name: str) -> str:
    return CLASS_NAMES_ZH.get(name, name)


def detour_direction(zone: str) -> str:
    if zone == "right":
        return "向左"
    return "向右"


# Simple obstacle tracker keyed by (cls, zone, distance).
_obstacle_tracker: Dict[Tuple[str, str, str], TrackedObstacle] = {}


def update_tracker(detections: List[Detection]) -> List[TrackedObstacle]:
    """Update multi-frame tracker and return newly stable obstacles that should be announced."""
    now = time.time()
    seen_keys = set()
    for det in detections:
        key = (det.cls, det.zone, det.distance)
        seen_keys.add(key)
        if key in _obstacle_tracker:
            tr = _obstacle_tracker[key]
            tr.seen += 1
            tr.missing = 0
            tr.conf = max(tr.conf, det.conf)
        else:
            _obstacle_tracker[key] = TrackedObstacle(
                cls=det.cls, zone=det.zone, distance=det.distance, conf=det.conf, seen=1
            )

    # Mark missing tracks.
    new_obstacles: List[TrackedObstacle] = []
    for key, tr in list(_obstacle_tracker.items()):
        if key not in seen_keys:
            tr.missing += 1
        if tr.missing > TRACK_MAX_MISSING:
            del _obstacle_tracker[key]
            continue
        if tr.seen >= TRACK_MIN_SEEN and (now - tr.last_announced) >= TRACK_COOLDOWN_S:
            tr.last_announced = now
            new_obstacles.append(tr)

    return new_obstacles


def build_obstacle_text(tr: TrackedObstacle) -> str:
    cls = tr.cls
    zone_text = zh_zone_name(tr.zone)
    dist_text = zh_distance_name(tr.distance)

    # Traffic lights -> direct stop/go instructions.
    if cls == "red_light":
        return f"{zone_text}红灯，请等待。"
    if cls == "green_light":
        return f"{zone_text}绿灯，可通行。"

    # Vulnerable road users / vehicles / chair in front -> stop or detour.
    if cls in ("person", "bicycle", "motorcycle", "tricycle", "dog", "chair"):
        if tr.zone == "center" and tr.distance == "near":
            return f"近处正前方有{zh_class_name(cls)}，请停下或向右绕行。"
        return f"{dist_text}{zone_text}有{zh_class_name(cls)}，请{detour_direction(tr.zone)}绕行。"

    # Static guidance features -> no detour needed.
    if cls in ("blind_road", "crosswalk"):
        return f"{dist_text}{zone_text}有{zh_class_name(cls)}。"

    # Generic obstacle -> detour.
    return f"{dist_text}{zone_text}有{zh_class_name(cls)}，请{detour_direction(tr.zone)}绕行。"


def _flush_tts_queue() -> None:
    """Drop pending TTS jobs to avoid stale obstacle announcements."""
    global _tts_pending
    while not tts_queue.empty():
        try:
            tts_queue.get_nowait()
        except queue.Empty:
            break
    with _TTS_PENDING_LOCK:
        _tts_pending.clear()


def _handle_traffic_lights(detections: List[Detection]) -> Optional[str]:
    """Traffic-light announcer.

    Announces the current red/green state as long as it persists (with a short
    cooldown), so the user gets continuous reminders while standing in front of
    a traffic light.  Red outranks green for safety.
    """
    global _last_traffic_state, _last_traffic_time, _traffic_state_counts
    now = time.time()

    red = [d for d in detections if d.cls == "red_light"]
    green = [d for d in detections if d.cls == "green_light"]

    # Red outranks green for safety.
    if red:
        current = "red_light"
    elif green:
        current = "green_light"
    else:
        current = None

    # Count consecutive frames for each candidate state.
    for key in ("red_light", "green_light", None):
        if key == current:
            _traffic_state_counts[key] = _traffic_state_counts.get(key, 0) + 1
        else:
            _traffic_state_counts[key] = 0

    # Light disappeared: drop stale pending speech and stay silent.
    if current is None:
        if _traffic_state_counts.get(None, 0) >= _TRAFFIC_DEBOUNCE_FRAMES:
            _last_traffic_state = None
            _flush_tts_queue()
        return None

    # Require a few consecutive frames before trusting the detection.
    if _traffic_state_counts.get(current, 0) < _TRAFFIC_DEBOUNCE_FRAMES:
        return None

    # Cooldown between announcements, even for the same state.
    if (now - _last_traffic_time) < _TRAFFIC_COOLDOWN_S:
        return None

    # Announce (first detection or repeated while light persists).
    _last_traffic_state = current
    _last_traffic_time = now

    # Prefer the highest-confidence detection for zone/distance.
    det = max(red or green, key=lambda d: d.conf)
    return build_obstacle_text(TrackedObstacle(
        cls=det.cls, zone=det.zone, distance=det.distance, conf=det.conf
    ))


def _dashscope_tts_to_pcm(text: str, model: str, voice: str) -> bytes:
    """Synthesize PCM using DashScope CosyVoice / Qwen-TTS.

    Falls back to offline TTS if the API key is missing or the call fails.
    """
    try:
        from dashscope.audio.tts_v2 import SpeechSynthesizer
    except Exception as exc:
        raise RuntimeError(f"DashScope SDK unavailable: {exc}")

    synthesizer = SpeechSynthesizer(model=model, voice=voice)
    audio = synthesizer.call(text)
    if audio is None:
        raise RuntimeError("DashScope returned empty audio")
    decoded = miniaudio.decode(
        bytes(audio),
        output_format=miniaudio.SampleFormat.SIGNED16,
        nchannels=1,
        sample_rate=SAMPLE_RATE,
    )
    return bytes(decoded.samples)


async def _edge_tts_to_pcm(text: str) -> bytes:
    communicate = edge_tts.Communicate(
        text, VOICE, rate=TTS_RATE, volume=TTS_VOLUME, proxy=_EDGE_TTS_PROXY
    )
    mp3_buf = b""
    async for chunk in communicate.stream():
        if chunk["type"] == "audio":
            mp3_buf += chunk["data"]
    decoded = miniaudio.decode(
        mp3_buf,
        output_format=miniaudio.SampleFormat.SIGNED16,
        nchannels=1,
        sample_rate=SAMPLE_RATE,
    )
    return bytes(decoded.samples)


def _offline_tts_to_pcm(text: str) -> bytes:
    """Synthesize PCM with pyttsx3 in a subprocess to avoid COM/event-loop hangs."""
    helper = os.path.join(os.path.dirname(__file__), "tts_helper.py")
    if not os.path.isfile(helper):
        raise RuntimeError(f"offline TTS helper not found: {helper}")
    with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
        wav_path = tmp.name
    try:
        proc = subprocess.run(
            [sys.executable, helper, text, wav_path],
            capture_output=True,
            text=True,
            timeout=10.0,
        )
        if proc.returncode != 0:
            raise RuntimeError(f"offline TTS helper failed: {proc.stderr}")
        with open(wav_path, "rb") as f:
            wav_bytes = f.read()
        decoded = miniaudio.decode(
            wav_bytes,
            output_format=miniaudio.SampleFormat.SIGNED16,
            nchannels=1,
            sample_rate=SAMPLE_RATE,
        )
        return bytes(decoded.samples)
    finally:
        try:
            os.remove(wav_path)
        except OSError:
            pass


def post_pcm(session: requests.Session, pcm: bytes, timeout: float = 15.0) -> None:
    if len(pcm) > MAX_PCM_BYTES:
        print(f"[play] truncating PCM from {len(pcm)} to {MAX_PCM_BYTES} bytes")
        pcm = pcm[:MAX_PCM_BYTES]
    headers = {"Content-Length": str(len(pcm))}
    resp = session.post(PLAY_URL, data=pcm, headers=headers, timeout=timeout)
    resp.raise_for_status()
    print(f"[play] {resp.status_code} OK")


def tts_worker() -> None:
    """Single background thread for all TTS + audio POST."""
    if not _HAS_PYTTSX3:
        print("[tts-worker] WARNING: pyttsx3 unavailable, offline TTS disabled")

    session = requests.Session()
    cache: Dict[str, bytes] = {}

    while True:
        text = tts_queue.get()
        with _TTS_PENDING_LOCK:
            _tts_pending.discard(text)
        if text is None:
            break
        if not text:
            continue

        t0 = time.time()
        try:
            if text in cache:
                pcm = cache[text]
                print(f"[tts-worker] cached: '{text}'")
            else:
                pcm: Optional[bytes] = None
                source = "offline"
                if _DASHSCOPE_TTS_ENABLED:
                    try:
                        pcm = _dashscope_tts_to_pcm(text, _DASHSCOPE_MODEL, _DASHSCOPE_VOICE)
                        source = "dashscope"
                    except Exception as exc:
                        print(f"[tts-worker] dashscope failed ({exc}), trying next source")
                        pcm = None
                if pcm is None and not _OFFLINE_TTS_ONLY:
                    try:
                        pcm = asyncio.run(_edge_tts_to_pcm(text))
                        source = "edge-tts"
                    except Exception as exc:
                        print(f"[tts-worker] edge-tts failed ({exc}), offline fallback")
                        pcm = None
                if pcm is None:
                    if not _HAS_PYTTSX3:
                        raise RuntimeError("offline TTS unavailable")
                    pcm = _offline_tts_to_pcm(text)
                    source = "offline"
                print(f"[tts-worker] {source}: '{text}'")
                cache[text] = pcm
                if len(cache) > 80:
                    cache.pop(next(iter(cache)))

            dur = len(pcm) / (SAMPLE_RATE * 2)
            print(f"[tts-worker] {len(pcm)} bytes, {dur:.2f}s, took {time.time() - t0:.2f}s")
            post_pcm(session, pcm)
        except Exception as exc:
            print(f"[tts-worker] failed for '{text}': {exc}")

    print("[tts-worker] exit")


def detect_traffic_light_by_color(image: Image.Image) -> Optional[Detection]:
    """Fallback red/green traffic-light detector using HSV color thresholds.

    Returns a Detection only when a reasonably traffic-light-shaped red or green
    blob is found, so the demo can still announce traffic lights even if the
    YOLO model misses small or back-lit lights.
    """
    try:
        img = np.array(image)
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        h, w = hsv.shape[:2]

        # Red appears at both ends of the HSV hue axis.
        # Higher saturation/value to avoid skin/orange false positives.
        lower_red1 = np.array([0, 90, 60])
        upper_red1 = np.array([12, 255, 255])
        lower_red2 = np.array([168, 90, 60])
        upper_red2 = np.array([180, 255, 255])
        red_mask = cv2.bitwise_or(
            cv2.inRange(hsv, lower_red1, upper_red1),
            cv2.inRange(hsv, lower_red2, upper_red2),
        )

        # Tight pure-green range to avoid white-balance cast and yellow/cyan.
        lower_green = np.array([40, 70, 60])
        upper_green = np.array([85, 255, 255])
        green_mask = cv2.inRange(hsv, lower_green, upper_green)

        def best_blob(mask: np.ndarray, color_label: str) -> Optional[Tuple[int, int, int, int, float]]:
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            best = None
            best_score = 0.0
            for cnt in contours:
                area = cv2.contourArea(cnt)
                # Phone screens / traffic lights held up in front of the camera.
                if area < 300 or area > h * w * 0.40:
                    continue
                x, y, bw, bh = cv2.boundingRect(cnt)
                if bw == 0 or bh == 0:
                    continue
                y_bottom_norm = (y + bh) / h
                y_center_norm = (y + bh / 2) / h
                # Traffic lights are typically in the upper part of the frame;
                # this suppresses false positives from tables/floor indoors.
                if y_bottom_norm > 0.85 or y_center_norm > 0.55:
                    continue
                aspect = bh / bw
                if aspect < 0.2 or aspect > 6.0:
                    continue
                fill_ratio = area / (bw * bh)
                if fill_ratio < 0.25:
                    continue
                score = area * fill_ratio
                if score < 400:
                    continue
                if score > best_score:
                    best_score = score
                    best = (x, y, x + bw, y + bh, float(score))
            return best

        red_blob = best_blob(red_mask, "red")
        green_blob = best_blob(green_mask, "green")

        chosen = None
        if red_blob and (not green_blob or red_blob[4] >= green_blob[4] * 0.6):
            chosen = ("red_light", red_blob)
        elif green_blob:
            chosen = ("green_light", green_blob)

        if chosen:
            cls_name, blob = chosen
            if _DEBUG_SAVE:
                try:
                    os.makedirs(_DEBUG_DIR, exist_ok=True)
                    ts = time.strftime("%m%d_%H%M%S")
                    Image.fromarray(img).save(os.path.join(_DEBUG_DIR, f"{ts}_{cls_name}_src.jpg"))
                    Image.fromarray(red_mask).save(os.path.join(_DEBUG_DIR, f"{ts}_{cls_name}_red.jpg"))
                    Image.fromarray(green_mask).save(os.path.join(_DEBUG_DIR, f"{ts}_{cls_name}_green.jpg"))
                    print(f"[debug] saved {ts}_{cls_name}_*.jpg")
                except Exception as exc:
                    print(f"[debug] save failed: {exc}")
            x1, y1, x2, y2, _ = blob
            x_center_norm = ((x1 + x2) / 2.0) / w
            y_bottom_norm = max(y1, y2) / h
            return Detection(
                cls=cls_name,
                conf=0.75,
                zone=zone_name(x_center_norm),
                distance="near" if y_bottom_norm >= DIST_NEAR_Y else ("mid" if y_bottom_norm >= DIST_MID_Y else "far"),
                priority=0 if y_bottom_norm >= DIST_NEAR_Y else (1 if y_bottom_norm >= DIST_MID_Y else 2),
                bbox=(x1 / w, y1 / h, x2 / w, y2 / h),
            )
    except Exception as exc:
        print(f"[traffic-color] fallback failed: {exc}")
    return None


def _boxes_to_detections(result, default_conf: float, class_override: Optional[str] = None, class_map: Optional[dict] = None) -> List[Detection]:
    """Convert a YOLO result into our Detection list.

    Args:
        class_map: optional mapping from model class name to our class name.
                   Only classes present in the map (or already known traffic classes)
                   are emitted when the map is provided.
    """
    detections: List[Detection] = []
    if result.boxes is None:
        return detections
    names = result.names
    img_width = result.orig_shape[1]
    img_height = result.orig_shape[0]
    img_area = img_width * img_height
    for box in result.boxes:
        conf = float(box.conf[0])
        cls_id = int(box.cls[0])
        raw_name = class_override or names.get(cls_id, str(cls_id))
        if class_map is not None:
            if raw_name not in class_map:
                continue
            cls_name = class_map[raw_name]
        else:
            cls_name = raw_name
        x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]
        box_area = (x2 - x1) * (y2 - y1)
        area_ratio = box_area / img_area
        if area_ratio < MIN_BOX_AREA_RATIO or area_ratio > MAX_BOX_AREA_RATIO:
            continue
        threshold = CLASS_CONF_THRESHOLDS.get(cls_name, default_conf)
        if conf < threshold:
            continue
        x_center_norm = ((x1 + x2) / 2.0) / img_width
        y_bottom_norm = max(y1, y2) / img_height
        zone = zone_name(x_center_norm)
        dist_word, dist_priority = distance_name(y_bottom_norm)
        detections.append(Detection(
            cls=cls_name,
            conf=conf,
            zone=zone,
            distance="near" if dist_priority == 0 else ("mid" if dist_priority == 1 else "far"),
            priority=dist_priority,
            bbox=(x1 / img_width, y1 / img_height, x2 / img_width, y2 / img_height),
        ))
    return detections


def _detect_zoomed_traffic_lights(model: YOLO, image: Image.Image, class_map: Optional[dict] = None) -> List[Detection]:
    """Digital-zoom inference on the upper-center region where traffic lights live.

    If the full-frame model missed the lights, this crops the top-center part of
    the image (about 2x magnification) and re-runs YOLO there.
    """
    w, h = image.size
    # Upper-center crop: 60% width, top 50% height.  This is where traffic lights
    # normally appear when the camera is pointed slightly upward.
    left = int(w * 0.20)
    top = 0
    right = int(w * 0.80)
    bottom = int(h * 0.50)
    cropped = image.crop((left, top, right, bottom))

    crop_results = model.predict(
        source=cropped,
        imgsz=640,
        conf=0.25,
        verbose=False,
        device="cuda",
    )
    if not crop_results or not crop_results[0].boxes:
        return []

    names = crop_results[0].names
    dets: List[Detection] = []
    img_width, img_height = image.size
    img_area = img_width * img_height
    for box in crop_results[0].boxes:
        cls_id = int(box.cls[0])
        raw_name = names.get(cls_id, str(cls_id))
        if class_map is not None:
            if raw_name not in class_map:
                continue
            cls_name = class_map[raw_name]
        else:
            cls_name = raw_name
            if cls_name not in _TRAFFIC_CLASSES:
                continue
        conf = float(box.conf[0])
        threshold = CLASS_CONF_THRESHOLDS.get(cls_name, CONF_THRESHOLD)
        if conf < threshold:
            continue
        x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]
        # Map coordinates back to the original image.
        x1 += left
        x2 += left
        y1 += top
        y2 += top
        box_area = (x2 - x1) * (y2 - y1)
        area_ratio = box_area / img_area
        if area_ratio < MIN_BOX_AREA_RATIO or area_ratio > MAX_BOX_AREA_RATIO:
            continue
        x_center_norm = ((x1 + x2) / 2.0) / img_width
        y_bottom_norm = max(y1, y2) / img_height
        dets.append(Detection(
            cls=cls_name,
            conf=conf,
            zone=zone_name(x_center_norm),
            distance="near" if y_bottom_norm >= DIST_NEAR_Y else ("mid" if y_bottom_norm >= DIST_MID_Y else "far"),
            priority=0 if y_bottom_norm >= DIST_NEAR_Y else (1 if y_bottom_norm >= DIST_MID_Y else 2),
            bbox=(x1 / img_width, y1 / img_height, x2 / img_width, y2 / img_height),
        ))
    return dets


def detect_objects(model: YOLO, image: Image.Image, coco_model: Optional[YOLO] = None, traffic_model: Optional[YOLO] = None) -> List[Detection]:
    results = model.predict(
        source=image,
        imgsz=640,
        conf=0.25,  # let YOLO emit low-confidence candidates; per-class thresholds filter below
        verbose=False,
        device="cuda",
    )
    detections: List[Detection] = []
    if results:
        detections.extend(_boxes_to_detections(results[0], CONF_THRESHOLD))

    # Digital zoom on the upper-center region if full-frame missed the lights.
    if not any(d.cls in _TRAFFIC_CLASSES for d in detections):
        try:
            zoomed = _detect_zoomed_traffic_lights(model, image)
            if zoomed:
                print(f"[zoom-traffic] found {len(zoomed)} light(s)")
                detections.extend(zoomed)
        except Exception as exc:
            print(f"[zoom-traffic] failed: {exc}")

    # Dedicated traffic-light model as a second opinion.
    if traffic_model is not None and not any(d.cls in _TRAFFIC_CLASSES for d in detections):
        try:
            tl_results = traffic_model.predict(
                source=image,
                imgsz=640,
                conf=0.25,
                verbose=False,
                device="cuda",
            )
            if tl_results:
                tl_dets = _boxes_to_detections(tl_results[0], CONF_THRESHOLD, class_map=TRAFFIC_CLASS_MAP)
                if tl_dets:
                    print(f"[traffic-model] found {len(tl_dets)} light(s): {[d.cls for d in tl_dets]}")
                    detections.extend(tl_dets)
        except Exception as exc:
            print(f"[traffic-model] failed: {exc}")

    if coco_model is not None:
        coco_results = coco_model.predict(
            source=image,
            imgsz=640,
            conf=COCO_CHAIR_CONF,
            classes=[COCO_CHAIR_CLASS_ID],
            verbose=False,
            device="cuda",
        )
        if coco_results:
            detections.extend(_boxes_to_detections(coco_results[0], COCO_CHAIR_CONF, class_override="chair"))
    return detections


# IMU turn state.
_last_turn_completed_count = 0


def check_turn(imu_data: dict) -> Optional[str]:
    """Return a turn announcement if a new turn was completed since last check."""
    global _last_turn_completed_count
    turn = imu_data.get("turn", {})
    count = turn.get("completed_count", 0)
    if count <= _last_turn_completed_count:
        return None
    _last_turn_completed_count = count
    deg = turn.get("last_completed_deg", 0.0)
    direction = turn.get("dir", "")
    if direction == "left":
        return f"向左转约{int(deg)}度。"
    if direction == "right":
        return f"向右转约{int(deg)}度。"
    return f"转身约{int(deg)}度。"


def _calibrate_posture(imu_data: dict) -> None:
    """Capture the upright neutral pose during the first N valid IMU samples."""
    global _posture_pitch_offset, _posture_roll_offset, _posture_calibrated, _posture_history
    attitude = imu_data.get("attitude", {})
    pitch = float(attitude.get("pitch", 0.0))
    roll = float(attitude.get("roll", 0.0))
    _posture_history.append((time.time(), pitch, roll))
    if len(_posture_history) >= POSTURE_CALIBRATE_SAMPLES:
        _posture_pitch_offset = sum(s[1] for s in _posture_history) / len(_posture_history)
        _posture_roll_offset = sum(s[2] for s in _posture_history) / len(_posture_history)
        _posture_calibrated = True
        print(f"[posture] calibrated offset pitch={_posture_pitch_offset:.1f} roll={_posture_roll_offset:.1f}")


def _calibrate_posture_now(imu_data: dict) -> None:
    """One-shot upright zero calibration from a single IMU sample."""
    global _posture_pitch_offset, _posture_roll_offset, _posture_calibrated, _posture_history
    attitude = imu_data.get("attitude", {})
    pitch = float(attitude.get("pitch", 0.0))
    roll = float(attitude.get("roll", 0.0))
    _posture_pitch_offset = pitch
    _posture_roll_offset = roll
    _posture_calibrated = True
    _posture_history = []
    print(f"[posture] manual calibrated offset pitch={_posture_pitch_offset:.1f} roll={_posture_roll_offset:.1f}")


def check_posture(imu_data: dict) -> Optional[str]:
    """Alert if the user's posture or device tilt is abnormal.

    Uses auto-calibrated offsets so that the IMU mounting angle does not
    trigger false alerts when the user is standing straight.
    """
    global _last_posture_time, _posture_history
    if POSTURE_DISABLE:
        return None
    attitude = imu_data.get("attitude", {})
    pitch_raw = float(attitude.get("pitch", 0.0))
    roll_raw = float(attitude.get("roll", 0.0))

    if not _posture_calibrated and len(_posture_history) < POSTURE_CALIBRATE_SAMPLES:
        return None

    pitch = pitch_raw - _posture_pitch_offset
    roll = roll_raw - _posture_roll_offset
    now = time.time()

    # Require the bad posture to persist for a short hold time.
    recent = [s for s in _posture_history if now - s[0] <= POSTURE_HOLD_S]
    _posture_history = recent
    _posture_history.append((now, pitch_raw, roll_raw))

    if (now - _last_posture_time) < POSTURE_COOLDOWN_S:
        return None

    bad_pitch = all(abs(s[1] - _posture_pitch_offset) > POSTURE_PITCH_THRESHOLD for s in recent)
    bad_roll = all(abs(s[2] - _posture_roll_offset) > POSTURE_ROLL_THRESHOLD for s in recent)

    if bad_pitch:
        _last_posture_time = now
        if pitch > 0:
            return "请注意，身体前倾过大。"
        return "请注意，身体后仰过大。"
    if bad_roll:
        _last_posture_time = now
        if roll > 0:
            return "设备向右倾斜，请保持水平。"
        return "设备向左倾斜，请保持水平。"
    return None


def enqueue_speech(text: str) -> None:
    if not text:
        return
    with _TTS_PENDING_LOCK:
        if text in _tts_pending:
            return
        try:
            tts_queue.put_nowait(text)
            _tts_pending.add(text)
        except queue.Full:
            print(f"[tts] queue full, dropping '{text}'")


def run_once(model: YOLO, coco_model: Optional[YOLO] = None, traffic_model: Optional[YOLO] = None, status: bool = False) -> None:
    print(f"\n--- iter @ {time.strftime('%H:%M:%S')} ---")

    if status:
        try:
            stat = fetch_url(STATUS_URL, timeout=5.0).text.strip()
            print(f"[status] {stat.replace(chr(10), '; ')}")
        except Exception as exc:
            print(f"[status] {exc}")

    # Poll IMU for turns and posture (lightweight, independent of snapshot).
    imu_data = fetch_imu(timeout=3.0)
    turn_text = check_turn(imu_data)
    if turn_text:
        print(f"[turn] {turn_text}")
        enqueue_speech(turn_text)

    # Manual upright-zero trigger via a flag file.
    if not POSTURE_DISABLE and imu_data.get("attitude"):
        if os.path.exists(POSTURE_TRIGGER_FILE):
            try:
                os.remove(POSTURE_TRIGGER_FILE)
            except OSError:
                pass
            _calibrate_posture_now(imu_data)
        elif POSTURE_AUTO_CALIBRATE and not _posture_calibrated:
            _calibrate_posture(imu_data)

    posture_text = check_posture(imu_data)
    if posture_text:
        print(f"[posture] {posture_text}")
        enqueue_speech(posture_text)

    image = fetch_snapshot()
    print(f"[snapshot] {image.size}")

    # Apply user-controlled digital zoom (phone-camera style).
    zoomed = apply_zoom(image)
    if zoomed is not image:
        print(f"[zoom] {get_zoom_factor():.2f}x")

    detections = detect_objects(model, zoomed, coco_model=coco_model, traffic_model=traffic_model)
    for det in detections:
        print(f"[yolo] {det.cls} conf={det.conf:.2f} zone={det.zone} distance={det.distance}")

    # Optional color-based fallback for traffic lights (useful if YOLO misses them).
    if TRAFFIC_FALLBACK_ENABLED and not any(det.cls in _TRAFFIC_CLASSES for det in detections):
        color_det = detect_traffic_light_by_color(zoomed)
        if color_det:
            print(f"[traffic-color] fallback {color_det.cls} zone={color_det.zone}")
            detections.append(color_det)

    # Handle traffic lights with a fast state machine instead of the obstacle tracker.
    traffic_text = _handle_traffic_lights(detections)
    if traffic_text:
        print(f"[traffic] {traffic_text}")
        # Traffic-light announcements are safety-critical: drop stale pending speech
        # so the red/green instruction is spoken immediately.
        _flush_tts_queue()
        enqueue_speech(traffic_text)

    # Track and announce everything else (people, vehicles, obstacles).
    # When a traffic light is present, skip obstacle announcements so the
    # safety-critical red/green instruction is not delayed by the TTS queue.
    status_text = ""
    if traffic_text:
        status_text = traffic_text
    else:
        non_traffic = [d for d in detections if d.cls not in _TRAFFIC_CLASSES]
        new_obstacles = update_tracker(non_traffic)
        if new_obstacles:
            tr = new_obstacles[0]
            text = build_obstacle_text(tr)
            print(f"[obstacle] {text}")
            enqueue_speech(text)
            status_text = text
        elif not detections:
            print("[obstacle] none")

    # Publish to dashboard without blocking TTS/audio.
    update_dashboard_state(zoomed, detections, imu_data, status_text=status_text)


def main() -> None:
    parser = argparse.ArgumentParser(description="Guide-glasses PC agent")
    parser.add_argument("--once", action="store_true")
    parser.add_argument("--interval", type=float, default=1.5)
    parser.add_argument("--status", action="store_true")
    parser.add_argument("--proxy", type=str, default=None)
    parser.add_argument("--host", type=str, default=ESP32_HOST)
    parser.add_argument("--traffic-fallback", action="store_true", help="enable color-based traffic-light fallback")
    parser.add_argument("--offline-tts", action="store_true", help="force offline TTS (avoids edge-tts network issues)")
    parser.add_argument("--dashscope-tts", action="store_true", help="use DashScope Qwen/CosyVoice TTS")
    parser.add_argument("--dashscope-model", type=str, default=None, help="DashScope TTS model (default cosyvoice-v3-flash)")
    parser.add_argument("--dashscope-voice", type=str, default=None, help="DashScope TTS voice (default longanhuan)")
    parser.add_argument("--posture-auto-calibrate", action="store_true", help="auto calibrate upright zero at startup (default: wait for manual trigger)")
    parser.add_argument("--posture-disable", action="store_true", help="disable posture alerts entirely")
    parser.add_argument("--posture-hold", type=float, default=None, help="seconds bad posture must persist (default 1.0)")
    parser.add_argument("--save-debug", action="store_true", help="save snapshots and HSV masks for traffic fallback debugging")
    parser.add_argument("--coco-chairs", action="store_true", help="use a COCO pretrained model to detect chairs (no retraining)")
    parser.add_argument("--model", type=str, default=None, help="path to custom YOLO model (default: best.pt)")
    parser.add_argument("--traffic-model", nargs="?", const=TRAFFIC_MODEL_PATH, default=None, metavar="PATH",
                        help="load a dedicated traffic-light YOLO model as a second opinion (default path used if flag is bare)")
    parser.add_argument("--dashboard", action="store_true", default=True,
                        help="start the web dashboard (default: true)")
    parser.add_argument("--no-dashboard", action="store_true",
                        help="disable the web dashboard")
    parser.add_argument("--dashboard-port", type=int, default=8080,
                        help="dashboard HTTP port (default: 8080)")
    args = parser.parse_args()

    global _EDGE_TTS_PROXY
    global TRAFFIC_FALLBACK_ENABLED, _OFFLINE_TTS_ONLY
    global _DASHSCOPE_TTS_ENABLED, _DASHSCOPE_MODEL, _DASHSCOPE_VOICE
    global POSTURE_DISABLE, POSTURE_HOLD_S, POSTURE_AUTO_CALIBRATE, _DEBUG_SAVE
    global _posture_calibrated
    TRAFFIC_FALLBACK_ENABLED = args.traffic_fallback
    _OFFLINE_TTS_ONLY = args.offline_tts
    _DASHSCOPE_TTS_ENABLED = args.dashscope_tts
    if args.dashscope_model:
        _DASHSCOPE_MODEL = args.dashscope_model
    if args.dashscope_voice:
        _DASHSCOPE_VOICE = args.dashscope_voice
    POSTURE_DISABLE = args.posture_disable
    POSTURE_AUTO_CALIBRATE = args.posture_auto_calibrate
    if args.posture_hold is not None:
        POSTURE_HOLD_S = args.posture_hold
    _DEBUG_SAVE = args.save_debug
    if args.proxy:
        _EDGE_TTS_PROXY = args.proxy
    if _EDGE_TTS_PROXY:
        print(f"[init] proxy: {_EDGE_TTS_PROXY}")

    resolved = resolve_host(args.host)
    print(f"[init] ESP32 host: {args.host} -> {resolved}")
    _dashboard_state["esp32_host"] = resolved
    global SNAPSHOT_URL, STATUS_URL, PLAY_URL, IMU_URL
    SNAPSHOT_URL, STATUS_URL, PLAY_URL, IMU_URL = make_urls(resolved)

    model_path = args.model if args.model else MODEL_PATH
    print(f"[init] loading model: {model_path}")
    model = YOLO(model_path)
    print("[init] model ready")

    coco_model: Optional[YOLO] = None
    if args.coco_chairs:
        print(f"[init] loading COCO chair model: {COCO_MODEL_PATH}")
        coco_model = YOLO(COCO_MODEL_PATH)
        print("[init] COCO chair model ready")

    traffic_model: Optional[YOLO] = None
    if args.traffic_model is not None:
        traffic_model_path = args.traffic_model if args.traffic_model else TRAFFIC_MODEL_PATH
        if os.path.isfile(traffic_model_path):
            print(f"[init] loading traffic-light model: {traffic_model_path}")
            traffic_model = YOLO(traffic_model_path)
            print("[init] traffic-light model ready")
        else:
            print(f"[init] traffic-light model not found at {traffic_model_path}, skipping")

    if args.dashboard and not args.no_dashboard:
        try:
            start_dashboard(
                snapshot_url=SNAPSHOT_URL,
                imu_url=IMU_URL,
                state_dict=_dashboard_state,
                state_lock=_DASHBOARD_LOCK,
                get_zoom=get_zoom_factor,
                set_zoom=set_zoom_factor,
                class_names_zh=CLASS_NAMES_ZH,
                port=args.dashboard_port,
            )
        except Exception as exc:
            print(f"[dashboard] failed to start: {exc}")

    if args.once:
        run_once(model, coco_model=coco_model, traffic_model=traffic_model, status=args.status)
        return

    worker = threading.Thread(target=tts_worker, daemon=True)
    worker.start()

    print(f"[init] loop interval={args.interval}s")
    try:
        while True:
            t0 = time.time()
            try:
                run_once(model, coco_model=coco_model, traffic_model=traffic_model, status=args.status)
            except Exception as exc:
                print(f"[loop-error] {exc}")
            elapsed = time.time() - t0
            sleep_time = max(0.0, args.interval - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)
    except KeyboardInterrupt:
        print("\n[exit] interrupted")
    finally:
        try:
            tts_queue.put_nowait(None)
        except queue.Full:
            pass
        worker.join(timeout=2.0)
        sys.exit(0)


if __name__ == "__main__":
    main()
