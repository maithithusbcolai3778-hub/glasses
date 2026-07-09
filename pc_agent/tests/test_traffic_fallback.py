#!/usr/bin/env python3
"""Offline sanity check for traffic-light color fallback."""
import sys
import io
from PIL import Image
import numpy as np

# Import the detector from pc_agent (heavy import, but okay for a quick test).
from pc_agent import detect_traffic_light_by_color, zone_name

def make_image(color, size=(800, 480), rect=None):
    """Create an RGB image with a filled rectangle."""
    img = Image.new("RGB", size, (64, 64, 64))
    if rect is None:
        # Full-frame-ish but under the 50% area limit? Use 45% of frame.
        w, h = size
        rw, rh = int(w * 0.45), int(h * 0.45)
        rect = ((w - rw) // 2, (h - rh) // 2, (w + rw) // 2, (h + rh) // 2)
    overlay = Image.new("RGB", size, color)
    mask = Image.new("L", size, 0)
    draw = Image.new("L", size, 255)
    mask.paste(draw, (0, 0), mask=draw)
    # Actually just crop/paste
    img.paste(color, rect)
    return img

def test_case(name, color, rect):
    img = make_image(color, rect=rect)
    det = detect_traffic_light_by_color(img)
    if det is None:
        print(f"[FAIL] {name}: no detection")
    else:
        print(f"[PASS] {name}: {det.cls} zone={det.zone} dist={det.distance}")

if __name__ == "__main__":
    W, H = 800, 480
    cases = [
        ("red-center-large", (255, 0, 0), (W*0.25, H*0.2, W*0.75, H*0.7)),
        ("red-center-small", (200, 0, 0), (W*0.4, H*0.3, W*0.6, H*0.5)),
        ("red-left", (220, 0, 0), (W*0.05, H*0.1, W*0.25, H*0.6)),
        ("red-right", (220, 0, 0), (W*0.75, H*0.1, W*0.95, H*0.6)),
        ("green-center", (0, 200, 0), (W*0.3, H*0.2, W*0.7, H*0.7)),
        ("green-left", (0, 180, 0), (W*0.05, H*0.1, W*0.25, H*0.6)),
        ("dark-red", (130, 0, 0), (W*0.3, H*0.2, W*0.7, H*0.6)),
    ]
    for name, color, rect in cases:
        test_case(name, color, tuple(int(v) for v in rect))
