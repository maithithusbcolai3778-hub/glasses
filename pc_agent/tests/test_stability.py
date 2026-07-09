import time, requests, os, sys

ip = '192.168.198.181'
duration_sec = 300  # 5 minutes
interval_sec = 1.0

print(f'[stability] testing http://{ip} for {duration_sec}s')
t0 = time.time()
ok_ping = ok_snap = fail = 0
last_ok = time.time()

while time.time() - t0 < duration_sec:
    loop_start = time.time()
    try:
        r = requests.get(f'http://{ip}/ping', timeout=5)
        if r.status_code == 200 and r.text == 'pong':
            ok_ping += 1
            last_ok = time.time()
        else:
            print(f'[ping] bad {r.status_code} {r.text!r}')
            fail += 1
    except Exception as e:
        print(f'[ping] error: {e}')
        fail += 1

    try:
        r = requests.get(f'http://{ip}/snapshot.jpg', timeout=10)
        if r.status_code == 200:
            ok_snap += 1
            last_ok = time.time()
        else:
            print(f'[snapshot] bad {r.status_code}')
            fail += 1
    except Exception as e:
        print(f'[snapshot] error: {e}')
        fail += 1

    elapsed = time.time() - t0
    if int(elapsed) % 30 == 0:
        print(f'... {elapsed:.0f}s ping={ok_ping} snap={ok_snap} fail={fail}')

    sleep = interval_sec - (time.time() - loop_start)
    if sleep > 0:
        time.sleep(sleep)

print(f'[done] duration={time.time()-t0:.0f}s ping={ok_ping} snap={ok_snap} fail={fail}')
