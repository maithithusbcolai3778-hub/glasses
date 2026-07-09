#!/usr/bin/env python3
"""Trigger upright-zero calibration in the running pc_agent."""
import os
import time

TRIGGER = "posture_calibrate.txt"

if __name__ == "__main__":
    with open(TRIGGER, "w", encoding="utf-8") as f:
        f.write(f"calibrate@{time.time()}\n")
    print(f"[calibrate] trigger written to {TRIGGER}")
    print("[calibrate] pc_agent will calibrate on its next IMU poll.")
