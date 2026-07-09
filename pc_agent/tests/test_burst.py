import time, requests
ip = '192.168.198.181'
for i in range(5):
    try:
        t0 = time.time()
        r = requests.get(f'http://{ip}/snapshot.jpg', timeout=10)
        dt = time.time() - t0
        print(f'[{i+1}/5] status={r.status_code} len={len(r.content)} time={dt:.2f}s')
    except Exception as e:
        print(f'[{i+1}/5] error: {e}')
    time.sleep(5)
print('burst done')
