#!/usr/bin/env python3
"""Quick test for DashScope / Qwen TTS before using it in pc_agent."""
import os
import sys

import miniaudio

api_key = os.getenv("DASHSCOPE_API_KEY")
if not api_key:
    print("[error] DASHSCOPE_API_KEY is not set")
    sys.exit(1)

model = os.getenv("DASHSCOPE_TTS_MODEL") or "cosyvoice-v3-flash"
voice = os.getenv("DASHSCOPE_TTS_VOICE") or "longanhuan"
text = "前方有行人，请向左绕行。"

print(f"[test] model={model} voice={voice}")
print(f"[test] text={text}")

try:
    from dashscope.audio.tts_v2 import SpeechSynthesizer
    synthesizer = SpeechSynthesizer(model=model, voice=voice)
    audio = synthesizer.call(text)
    if not audio:
        print("[error] no audio returned")
        sys.exit(1)
    decoded = miniaudio.decode(
        bytes(audio),
        output_format=miniaudio.SampleFormat.SIGNED16,
        nchannels=1,
        sample_rate=16000,
    )
    pcm = bytes(decoded.samples)
    dur = len(pcm) / (16000 * 2)
    print(f"[ok] {len(pcm)} bytes PCM, {dur:.2f}s")
except Exception as exc:
    print(f"[error] {exc}")
    sys.exit(1)
