#!/usr/bin/env python3
"""
Qwen-VL Agent for guide-glasses prototype.

Does NOT need local GPU / YOLO / edge-tts proxy.
Workflow:
1. Pull /snapshot.jpg from ESP32-P4.
2. Send image to DashScope qwen-vl-plus with a guide-assistant prompt.
3. Get a concise Chinese scene description.
4. Offline TTS (pyttsx3) -> 16 kHz 16-bit mono PCM.
5. POST PCM to ESP32 /play endpoint.

Requires DASHSCOPE_API_KEY environment variable.
"""

import argparse
import base64
import io
import os
import re
import socket
import sys
import tempfile
import time
from typing import Callable, Optional, TypeVar

# Force UTF-8 stdout/stderr so Chinese logs are captured correctly on Windows.
try:
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", line_buffering=True)
    sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding="utf-8", line_buffering=True)
except Exception:
    pass

import dashscope
import miniaudio
import requests
from PIL import Image, ImageStat
from zeroconf import Zeroconf

ESP32_HOST = "192.168.198.181"  # fallback to static IP when mDNS (.local) fails
RESOLVED_HOST = ""


def set_volume(host: str, volume: int) -> None:
    """Set ESP32 speaker volume via /volume endpoint."""
    try:
        url = f"http://{host}/volume?vol={volume}"
        resp = _HTTP_SESSION.get(url, timeout=5)
        resp.raise_for_status()
        print(f"[volume] {resp.text.strip()}")
    except Exception as exc:
        print(f"[volume] failed: {exc}")


# Offline TTS fallback (Windows SAPI5).
try:
    import pyttsx3
    _HAS_PYTTSX3 = True
except Exception:
    _HAS_PYTTSX3 = False

def resolve_host(host: str, timeout: float = 2.0) -> str:
    """Resolve a hostname to an IPv4 address.

    For .local mDNS names, use Zeroconf; otherwise use system DNS.
    Returns the original host on failure so callers can still try.
    """
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
        print(f"[mdns] resolution failed: {exc}")

    # Fallback: let requests/urllib try system mDNS/Bonjour if available.
    return host


def make_urls(host: str) -> tuple[str, str, str]:
    base = f"http://{host}"
    return f"{base}/snapshot.jpg", f"{base}/status", f"{base}/play"

SAMPLE_RATE = 16000
MAX_PCM_BYTES = 256 * 1024

# Reuse TCP connections to reduce ESP32 socket churn.
_HTTP_SESSION = requests.Session()
_HTTP_SESSION.headers.update({"Connection": "keep-alive"})

# Image-quality thresholds used before sending frames to Qwen.
_DARK_MEAN_THRESHOLD = 20.0
_BRIGHT_MEAN_THRESHOLD = 250.0
_LOW_DETAIL_RMS_THRESHOLD = 8.0

QWEN_MODEL = "qwen-vl-plus"

DEFAULT_PROMPT = (
    "你是一位导盲助手的视觉分析专家。用户佩戴了一副胸前朝前下方的摄像头，正在行走。"
    "请用一句极简短的中文描述前方路况，必须优先报告障碍物和行人、车辆。"
    "如果看到障碍物或行人/车辆，必须说明它相对于佩戴者的方向：左侧、正前方还是右侧，"
    "并给出简短的绕行建议：向左走、向右走或停下。"
    "例如：'正前方有椅子，请向左走'、'右侧有行人，请向左绕行'、'左侧有墙，请向右走'。"
    "必须优先且绝对不能忽略：行人、车辆（汽车/公交车/卡车/电动车/摩托车/自行车）、"
    "红绿灯（红灯要停）、坑洞、台阶、电线杆、椅子、桌子、门、墙等障碍物。"
    "如果前方确实空旷无障碍物，请回答：前方畅通。"
    "只输出一句话，不要解释。"
)

T = TypeVar("T")


def get_api_key() -> str:
    key = os.getenv("DASHSCOPE_API_KEY")
    if key:
        return key

    # Load from project .env file if env var is not set.
    script_dir = os.path.dirname(os.path.abspath(__file__))
    env_path = os.path.join(os.path.dirname(script_dir), ".env")
    if not os.path.exists(env_path):
        env_path = os.path.join(script_dir, ".env")
    if os.path.exists(env_path):
        try:
            with open(env_path, "r", encoding="utf-8") as f:
                for line in f:
                    line = line.strip()
                    if line.startswith("DASHSCOPE_API_KEY="):
                        key = line.split("=", 1)[1].strip().strip('"\'')
                        break
        except Exception as exc:
            print(f"[key] failed to read .env: {exc}")

    if not key:
        raise RuntimeError(
            "DASHSCOPE_API_KEY not set. Get one from https://dashscope.aliyun.com/"
        )
    return key


def retry_call(
    fn: Callable[[], T],
    retries: int = 3,
    delay: float = 1.0,
    name: str = "operation",
) -> T:
    """Call fn(), retrying on any exception up to retries times."""
    last_exc: Optional[Exception] = None
    for attempt in range(1, retries + 1):
        try:
            return fn()
        except Exception as exc:
            last_exc = exc
            print(f"[retry] {name} failed (attempt {attempt}/{retries}): {exc}")
            if attempt < retries:
                time.sleep(delay)
    raise last_exc or RuntimeError(f"{name} failed after {retries} attempts")


def fetch_snapshot(timeout: float = 15.0, retries: int = 3) -> Image.Image:
    def _get():
        resp = _HTTP_SESSION.get(SNAPSHOT_URL, timeout=timeout)
        resp.raise_for_status()
        return Image.open(io.BytesIO(resp.content)).convert("RGB")

    return retry_call(_get, retries=retries, name="fetch_snapshot")


def fetch_status(timeout: float = 10.0, retries: int = 2) -> str:
    def _get():
        resp = _HTTP_SESSION.get(STATUS_URL, timeout=timeout)
        resp.raise_for_status()
        return resp.text.strip()

    try:
        return retry_call(_get, retries=retries, name="fetch_status")
    except Exception as exc:
        print(f"[status] skipped: {exc}")
        return ""


def image_to_base64(image: Image.Image, fmt: str = "JPEG") -> str:
    buf = io.BytesIO()
    image.save(buf, format=fmt)
    b64 = base64.b64encode(buf.getvalue()).decode("utf-8")
    return f"data:image/{fmt.lower()};base64,{b64}"


def ask_qwen(image_b64: str, prompt: str, retries: int = 3) -> str:
    def _call():
        messages = [
            {
                "role": "user",
                "content": [
                    {"type": "image", "image": image_b64},
                    {"type": "text", "text": prompt},
                ],
            }
        ]
        response = dashscope.MultiModalConversation.call(
            model=QWEN_MODEL,
            messages=messages,
        )
        if response.status_code != 200:
            raise RuntimeError(
                f"DashScope error: {response.status_code} {response.message}"
            )

        content = response.output.choices[0].message.content
        if isinstance(content, list):
            text_parts = [
                item.get("text", "") for item in content if isinstance(item, dict)
            ]
            return "".join(text_parts).strip()
        return str(content).strip()

    return retry_call(_call, retries=retries, name="ask_qwen")


def normalize_description(text: str) -> str:
    """Strip punctuation/spaces for deduplication comparison."""
    return re.sub(r"[\s。，、,.!?！？]", "", text).lower()


def text_to_pcm(text: str) -> bytes:
    if not _HAS_PYTTSX3:
        raise RuntimeError("pyttsx3 not installed, cannot synthesize speech")
    engine = pyttsx3.init()
    engine.setProperty("rate", 220)
    with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
        wav_path = tmp.name
    try:
        engine.save_to_file(text, wav_path)
        engine.runAndWait()
        with open(wav_path, "rb") as f:
            wav_bytes = f.read()
        decoded = miniaudio.decode(
            wav_bytes,
            output_format=miniaudio.SampleFormat.SIGNED16,
            nchannels=1,
            sample_rate=SAMPLE_RATE,
        )
        pcm = bytes(decoded.samples)
        if len(pcm) > MAX_PCM_BYTES:
            pcm = pcm[:MAX_PCM_BYTES]
        return pcm
    finally:
        try:
            os.remove(wav_path)
        except OSError:
            pass


def post_pcm(pcm: bytes, timeout: float = 20.0, retries: int = 3) -> None:
    def _post():
        headers = {"Content-Length": str(len(pcm))}
        resp = _HTTP_SESSION.post(PLAY_URL, data=pcm, headers=headers, timeout=timeout)
        resp.raise_for_status()
        return resp

    resp = retry_call(_post, retries=retries, name="post_pcm")
    print(f"[play] {resp.status_code} {resp.text.strip()}")


def image_quality_check(image: Image.Image) -> tuple[bool, str]:
    """Return (ok, reason).  reason is empty when ok."""
    gray = image.convert("L")
    stat = ImageStat.Stat(gray)
    mean = stat.mean[0]
    rms = stat.rms[0]
    if mean < _DARK_MEAN_THRESHOLD:
        return False, f"画面过暗，亮度{mean:.0f}"
    if mean > _BRIGHT_MEAN_THRESHOLD:
        return False, f"画面过亮，亮度{mean:.0f}"
    if rms < _LOW_DETAIL_RMS_THRESHOLD:
        return False, f"画面无细节，方差{rms:.0f}"
    return True, ""


def _speak_description(description: str) -> None:
    try:
        pcm = text_to_pcm(description)
        print(f"[pcm] {len(pcm)} bytes, {len(pcm) / (SAMPLE_RATE * 2):.2f}s")
        post_pcm(pcm)
    except Exception as exc:
        print(f"[error] failed to play audio: {exc}")


def run_once(
    prompt: str,
    status: bool = False,
    last_description: str = "",
    dedup: bool = True,
    save_dir: Optional[str] = None,
) -> str:
    """Single perception -> speak iteration.

    Returns the current description (for deduplication tracking).
    """
    print(f"\n--- iter @ {time.strftime('%H:%M:%S')} ---")

    if status:
        stat = fetch_status()
        if stat:
            print(f"[status] {stat.replace(chr(10), '; ')}")

    try:
        image = fetch_snapshot()
        print(f"[snapshot] {image.size}")
        if save_dir:
            try:
                os.makedirs(save_dir, exist_ok=True)
                ts = time.strftime("%Y%m%d_%H%M%S") + f"_{int(time.time()*1000)%1000:03d}"
                path = os.path.join(save_dir, f"snapshot_{ts}.jpg")
                image.save(path, "JPEG")
                print(f"[save] {path}")
            except Exception as exc:
                print(f"[save] failed: {exc}")
    except Exception as exc:
        print(f"[error] failed to fetch snapshot: {exc}")
        description = "设备连接异常，请检查网络或摄像头"
        if normalize_description(description) != normalize_description(last_description):
            _speak_description(description)
        return description

    quality_ok, quality_msg = image_quality_check(image)
    if not quality_ok:
        print(f"[quality] {quality_msg}")
        description = f"摄像头画面异常，{quality_msg}，请检查镜头或排线"
        if dedup and normalize_description(description) == normalize_description(last_description):
            print("[dedup] same as last, skip TTS")
            return description
        _speak_description(description)
        return description

    try:
        image_b64 = image_to_base64(image)
        description = ask_qwen(image_b64, prompt)
        print(f"[qwen] {description}")
    except Exception as exc:
        print(f"[error] failed to call DashScope: {exc}")
        return last_description

    if dedup and normalize_description(description) == normalize_description(last_description):
        print("[dedup] same as last, skip TTS")
        return last_description

    _speak_description(description)
    return description


def main() -> None:
    parser = argparse.ArgumentParser(description="Qwen-VL guide-glasses agent")
    parser.add_argument("--once", action="store_true", help="Run one iteration")
    parser.add_argument("--interval", type=float, default=3.0, help="Loop interval (s)")
    parser.add_argument("--status", action="store_true", help="Fetch /status")
    parser.add_argument(
        "--prompt",
        type=str,
        default=DEFAULT_PROMPT,
        help="Custom prompt for qwen-vl-plus",
    )
    parser.add_argument(
        "--no-dedup",
        action="store_true",
        help="Disable description deduplication (always announce)",
    )
    parser.add_argument(
        "--retries",
        type=int,
        default=3,
        help="Max retries for network/DashScope calls",
    )
    parser.add_argument(
        "--max-iters",
        type=int,
        default=0,
        help="Stop after N iterations (0 = infinite)",
    )
    parser.add_argument(
        "--host",
        type=str,
        default=ESP32_HOST,
        help="ESP32 hostname or IP (default: guide-glasses.local)",
    )
    parser.add_argument(
        "--save-dir",
        type=str,
        default=None,
        help="Directory to save snapshots for later labeling",
    )
    parser.add_argument(
        "--volume",
        type=int,
        default=None,
        help="Set ESP32 speaker volume at startup (0-100)",
    )
    args = parser.parse_args()

    dashscope.api_key = get_api_key()
    print(f"[init] DashScope key loaded, model={QWEN_MODEL}")

    resolved = resolve_host(args.host)
    print(f"[init] ESP32 host: {args.host} -> {resolved}")
    global SNAPSHOT_URL, STATUS_URL, PLAY_URL, RESOLVED_HOST
    SNAPSHOT_URL, STATUS_URL, PLAY_URL = make_urls(resolved)
    RESOLVED_HOST = resolved

    if args.volume is not None:
        set_volume(resolved, max(0, min(100, args.volume)))

    if args.once:
        run_once(
            args.prompt,
            status=args.status,
            dedup=not args.no_dedup,
            save_dir=args.save_dir,
        )
        return

    print(f"[init] Starting loop, interval={args.interval}s, dedup={not args.no_dedup}")
    last_description = ""
    iter_count = 0
    try:
        while args.max_iters == 0 or iter_count < args.max_iters:
            iter_count += 1
            t0 = time.time()
            last_description = run_once(
                args.prompt,
                status=args.status,
                last_description=last_description,
                dedup=not args.no_dedup,
                save_dir=args.save_dir,
            )
            elapsed = time.time() - t0
            sleep_time = max(0.0, args.interval - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)
        print("\n[exit] max-iters reached")
    except KeyboardInterrupt:
        print("\n[exit] Interrupted")
        sys.exit(0)


if __name__ == "__main__":
    main()
