import sys
sys.path.insert(0, r'D:\esp-idf\WIFI+OV5647+SPEAKER_CAMERA_MJPEG_AUDIO_DEV_IMU')
from pc_agent import Detection, build_announcement

# Simulate: far car on left, near chair center, mid person right
dets = [
    Detection('car', 0.9, 'left', 'far', 2),
    Detection('chair', 0.7, 'center', 'near', 0),
    Detection('person', 0.8, 'right', 'mid', 1),
]
print(build_announcement(dets))

# Single close center obstacle
dets = [Detection('chair', 0.85, 'center', 'near', 0)]
print(build_announcement(dets))

# No detection
print(build_announcement([]))
