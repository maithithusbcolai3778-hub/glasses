"""MJPEG stream reader for the ESP32 camera.

Opens a single long-running /stream connection to the ESP32, decodes frames,
and shares the latest frame with both the perception loop and the dashboard.
This avoids the firmware's warning about concurrently polling /snapshot.jpg
and /stream.
"""

import io
import re
import threading
import time
from typing import Optional

import requests
from PIL import Image


class FrameGrabber:
    def __init__(self, stream_url: str, snapshot_url: str, target_fps: int = 20):
        self.stream_url = stream_url
        self.snapshot_url = snapshot_url
        self.target_fps = target_fps
        self._frame_lock = threading.Lock()
        self._latest_frame: Optional[Image.Image] = None
        self._latest_jpeg: bytes = b""
        self._frame_id = 0
        self._last_frame_time = 0.0
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._error: Optional[str] = None

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)

    def _run(self) -> None:
        while self._running:
            try:
                self._read_stream()
            except Exception as exc:
                self._error = str(exc)
                print(f"[frame-grabber] stream error: {exc}; retrying in 1s")
                time.sleep(1.0)

    def _read_stream(self) -> None:
        resp = requests.get(self.stream_url, stream=True, timeout=15.0)
        resp.raise_for_status()
        ctype = resp.headers.get("Content-Type", "")
        boundary = "--frame"
        m = re.search(r"boundary=([^;\s]+)", ctype)
        if m:
            boundary = m.group(1).strip('"')
        if not boundary.startswith("--"):
            boundary = "--" + boundary
        boundary_b = boundary.encode("ascii")

        buffer = b""
        for chunk in resp.iter_content(chunk_size=8192):
            if not self._running:
                break
            buffer += chunk
            while True:
                # Find next boundary
                idx = buffer.find(boundary_b)
                if idx == -1:
                    break
                part = buffer[:idx]
                buffer = buffer[idx + len(boundary_b):]
                if not part.startswith(b"\r\n"):
                    # discard preamble before first boundary
                    continue
                # strip leading CRLF and trailing CRLF
                part = part[2:]
                if part.endswith(b"\r\n"):
                    part = part[:-2]
                # split headers and jpeg body
                sep = part.find(b"\r\n\r\n")
                if sep == -1:
                    continue
                headers = part[:sep].decode("latin1")
                jpeg = part[sep + 4:]
                cl = 0
                for line in headers.split("\r\n"):
                    if line.lower().startswith("content-length:"):
                        try:
                            cl = int(line.split(":", 1)[1].strip())
                        except Exception:
                            pass
                        break
                if cl > 0 and len(jpeg) >= cl:
                    jpeg = jpeg[:cl]
                if not jpeg:
                    continue
                try:
                    img = Image.open(io.BytesIO(jpeg)).convert("RGB")
                except Exception:
                    continue
                with self._frame_lock:
                    self._latest_frame = img
                    self._latest_jpeg = jpeg
                    self._frame_id += 1
                    self._last_frame_time = time.time()
                # Throttle to roughly target_fps by sleeping a bit if frames
                # are arriving faster than we want to process/store.
                time.sleep(1.0 / self.target_fps)
        resp.close()

    def get_frame(self, timeout: float = 5.0) -> Optional[Image.Image]:
        """Return the latest decoded frame, or None if not available."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._frame_lock:
                frame = self._latest_frame
                if frame is not None:
                    return frame
            time.sleep(0.05)
        return None

    def get_jpeg(self) -> bytes:
        with self._frame_lock:
            return self._latest_jpeg

    @property
    def last_frame_time(self) -> float:
        with self._frame_lock:
            return self._last_frame_time

    @property
    def error(self) -> Optional[str]:
        return self._error
