import io
import os
import time
from datetime import datetime
from pathlib import Path

import requests
from PIL import Image
from ultralytics import YOLO

SNAPSHOT_URL = "http://192.168.198.181/snapshot.jpg"
MODEL_PATH = r"D:\YOLO_CUSTOM\runs\guide_glasses\weights\best.pt"
CONF = 0.45
MIN_BOX_AREA_RATIO = 0.005
MAX_BOX_AREA_RATIO = 0.60
CLASS_CONF_THRESHOLDS = {
    "ashcan": 0.70,
    "tree": 0.60,
    "dog": 0.60,
}
OUT_DIR = Path(r"D:\YOLO_CUSTOM\live_test")


def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    model = YOLO(MODEL_PATH)
    print(f"[live-test] conf={CONF}, saving to {OUT_DIR}")
    session = requests.Session()
    for i in range(20):
        t0 = time.time()
        try:
            resp = session.get(SNAPSHOT_URL, timeout=5)
            resp.raise_for_status()
            img = Image.open(io.BytesIO(resp.content)).convert("RGB")
        except Exception as exc:
            print(f"[{i}] snapshot failed: {exc}")
            time.sleep(1)
            continue

        results = model.predict(source=img, imgsz=640, conf=CONF, verbose=False, device="cuda")
        r = results[0]
        dets = []
        if r.boxes is not None:
            img_w, img_h = img.size
            img_area = img_w * img_h
            for box in r.boxes:
                x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]
                area_ratio = (x2 - x1) * (y2 - y1) / img_area
                if area_ratio < MIN_BOX_AREA_RATIO or area_ratio > MAX_BOX_AREA_RATIO:
                    continue
                cls = r.names[int(box.cls)]
                conf = float(box.conf)
                if conf < CLASS_CONF_THRESHOLDS.get(cls, CONF):
                    continue
                dets.append((cls, conf))
        ts = datetime.now().strftime("%H%M%S")
        print(f"[{i} @ {ts}] dets={dets}")
        if dets:
            out_path = OUT_DIR / f"frame_{ts}_{i}.jpg"
            r.save(str(out_path))
        elapsed = time.time() - t0
        time.sleep(max(0.0, 1.0 - elapsed))


if __name__ == "__main__":
    main()
