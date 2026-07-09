import time, requests, re

ip = '192.168.198.181'

def parse_status(text):
    d = {}
    for line in text.splitlines():
        if '=' in line:
            k, v = line.split('=', 1)
            d[k.strip()] = v.strip()
    return d

def get_status():
    return parse_status(requests.get(f'http://{ip}/status', timeout=10).text)

print('Measuring camera FPS from /status deltas...')
s1 = get_status()
t1 = time.time()
time.sleep(10)
s2 = get_status()
t2 = time.time()
dt = t2 - t1
raw1 = int(s1.get('raw_frames', 0))
raw2 = int(s2.get('raw_frames', 0))
jpeg1 = int(s1.get('jpeg_frames', 0))
jpeg2 = int(s2.get('jpeg_frames', 0))
print(f'raw FPS: {(raw2-raw1)/dt:.2f}')
print(f'jpeg FPS: {(jpeg2-jpeg1)/dt:.2f}')
print(f'snapshot latency test ({ip}/snapshot.jpg):')
latencies = []
for i in range(10):
    t0 = time.time()
    r = requests.get(f'http://{ip}/snapshot.jpg', timeout=10)
    dt = time.time() - t0
    latencies.append(dt)
    print(f'  {i+1}: status={r.status_code} len={len(r.content)} time={dt:.3f}s')
print(f'avg latency: {sum(latencies)/len(latencies):.3f}s')
