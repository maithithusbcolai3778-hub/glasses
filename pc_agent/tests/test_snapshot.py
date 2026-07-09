import os, requests

ip = '192.168.198.181'
print(f'[snapshot] fetching from {ip}')
try:
    r = requests.get(f'http://{ip}/snapshot.jpg', timeout=10)
    print(f'[snapshot] status={r.status_code} len={len(r.content)}')
    if r.status_code == 200:
        path = 'test_bottom_crop.jpg'
        with open(path, 'wb') as f:
            f.write(r.content)
        print(f'[save] {path} size={os.path.getsize(path)}')
except Exception as e:
    print(f'[snapshot] error: {e}')
