import tempfile
import os
import pyttsx3
import miniaudio
import requests

TEXT = "近处右侧有行人，请向左绕行。"
IP = "192.168.198.181"

engine = pyttsx3.init()
engine.setProperty("rate", 160)

with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
    wav_path = tmp.name
engine.save_to_file(TEXT, wav_path)
engine.runAndWait()

try:
    with open(wav_path, "rb") as f:
        wav = f.read()
    decoded = miniaudio.decode(
        wav,
        output_format=miniaudio.SampleFormat.SIGNED16,
        nchannels=1,
        sample_rate=16000,
    )
    pcm = bytes(decoded.samples)
    print(f"[tts] generated {len(pcm)} bytes PCM")

    resp = requests.post(
        f"http://{IP}/play",
        data=pcm,
        headers={"Content-Type": "audio/pcm"},
        timeout=10,
    )
    print(f"[play] status={resp.status_code} body={resp.text!r}")
finally:
    try:
        os.remove(wav_path)
    except OSError:
        pass
