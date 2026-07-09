#!/usr/bin/env python3
import requests
from qwen_agent import resolve_host, make_urls

host = resolve_host("guide-glasses.local", timeout=3.0)
print("resolved:", host)
urls = make_urls(host)
print("urls:", urls)
try:
    r = requests.get(urls[1], timeout=5)
    print("status:", r.status_code, r.text.strip()[:200])
except Exception as e:
    print("status error:", e)
try:
    r = requests.get(urls[0], timeout=10)
    print("snapshot:", r.status_code, len(r.content))
except Exception as e:
    print("snapshot error:", e)
