import time, requests

ip = '192.168.198.181'
duration_sec = 30
interval_sec = 0.2

print(f'[fast preview] fetching /snapshot.jpg every {interval_sec}s for {duration_sec}s')
t0 = time.time()
ok = fail = 0
while time.time() - t0 < duration_sec:
    try:
        r = requests.get(f'http://{ip}/snapshot.jpg', timeout=5)
        if r.status_code == 200:
            ok += 1
        else:
            fail += 1
    except Exception as e:
        fail += 1
        print(f'error: {e}')
    elapsed = time.time() - t0
    if int(elapsed) % 5 == 0 and elapsed % 1 < 0.3:
        print(f'... {elapsed:.0f}s ok={ok} fail={fail}')
    sleep = interval_sec - (time.time() - (t0 + ok*interval_sec))
    if sleep > 0:
        time.sleep(sleep)
print(f'[done] ok={ok} fail={fail}')
