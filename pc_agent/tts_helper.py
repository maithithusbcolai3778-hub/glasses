import sys, pyttsx3, tempfile, os, time
if len(sys.argv)<3:
    print("usage: tts_helper.py <text> <out_wav>")
    sys.exit(1)
text=sys.argv[1]
out=sys.argv[2]
engine=pyttsx3.init()
engine.setProperty('rate',160)
engine.save_to_file(text,out)
engine.runAndWait()
print("ok", out)
