#!/usr/bin/env python3
import math
import urllib.request
import struct

URL = "http://192.168.198.181/play"
SR = 16000
DURATION = 1.0
FREQ = 440.0

samples = int(SR * DURATION)
pcm = bytearray()
for i in range(samples):
    v = int(32767 * 0.3 * math.sin(2 * math.pi * FREQ * i / SR))
    pcm.extend(struct.pack("<h", v))

req = urllib.request.Request(
    URL,
    data=bytes(pcm),
    method="POST",
    headers={"Content-Length": str(len(pcm))},
)
try:
    with urllib.request.urlopen(req, timeout=10) as resp:
        print(resp.status, resp.read().decode())
except Exception as e:
    print("ERROR:", e)
