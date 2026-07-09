"""Web dashboard for the guide-glasses PC agent.

Serves a lightweight HTML UI that embeds the ESP32's raw MJPEG stream
(http://<esp32>/stream) and overlays the latest YOLO boxes / IMU data using
HTML/CSS.  Because the video never passes through the PC, it stays smooth and
does not compete with the agent's /snapshot, /play and /imu traffic.
"""

import json
import os
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
from typing import Any, Callable, Dict, Optional

_HTML_PATH = os.path.join(os.path.dirname(__file__), "dashboard.html")


def _load_html() -> bytes:
    try:
        with open(_HTML_PATH, "rb") as f:
            return f.read()
    except Exception as exc:
        return f"<!-- dashboard.html not found: {exc} -->".encode("utf-8")


class DashboardState:
    def __init__(
        self,
        state_dict: Dict[str, Any],
        state_lock: threading.Lock,
        get_zoom: Callable[[], float],
        set_zoom: Callable[[float], float],
        class_names_zh: Dict[str, str],
    ):
        self.state_dict = state_dict
        self.lock = state_lock
        self.get_zoom = get_zoom
        self.set_zoom = set_zoom
        self.class_names_zh = class_names_zh


class DashboardServer:
    def __init__(
        self,
        state: DashboardState,
        port: int = 8080,
    ):
        self.state = state
        self.port = port
        self._server: Optional[HTTPServer] = None

    def start(self) -> None:
        handler = self._make_handler()
        self._server = ThreadedHTTPServer(("0.0.0.0", self.port), handler)
        print(f"[dashboard] http://0.0.0.0:{self.port}")
        server_thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        server_thread.start()

    def _make_handler(self):
        server = self
        html_body = _load_html()

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):
                pass

            def do_GET(self):
                path = self.path.split("?", 1)[0]
                if path == "/":
                    self._serve_html()
                elif path == "/state":
                    self._serve_state()
                elif path == "/set_zoom":
                    self._set_zoom()
                else:
                    self.send_error(404)

            def _serve_html(self):
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(html_body)))
                self.end_headers()
                self.wfile.write(html_body)

            def _serve_state(self):
                with server.state.lock:
                    state = dict(server.state.state_dict)
                state.pop("frame", None)
                state.pop("detections", None)
                state["detections"] = state.get("detections_json", [])
                for d in state["detections"]:
                    d["cls_zh"] = server.state.class_names_zh.get(d.get("cls", ""), d.get("cls", ""))
                state["zoom"] = server.state.get_zoom()
                body = json.dumps(state, ensure_ascii=False, default=str).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def _set_zoom(self):
                factor = 1.0
                try:
                    if "?" in self.path:
                        qs = self.path.split("?", 1)[1]
                        for kv in qs.split("&"):
                            if "=" in kv:
                                k, v = kv.split("=", 1)
                                if k == "factor":
                                    factor = float(v)
                except Exception:
                    pass
                new_zoom = server.state.set_zoom(factor)
                body = json.dumps({"zoom": new_zoom}).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

        return Handler


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def start_dashboard(
    snapshot_url: str,
    imu_url: str,
    state_dict: Dict[str, Any],
    state_lock: threading.Lock,
    get_zoom: Callable[[], float],
    set_zoom: Callable[[float], float],
    class_names_zh: Dict[str, str],
    port: int = 8080,
) -> DashboardServer:
    state = DashboardState(state_dict, state_lock, get_zoom, set_zoom, class_names_zh)
    srv = DashboardServer(state, port=port)
    srv.start()
    return srv
