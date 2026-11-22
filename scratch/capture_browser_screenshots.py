import subprocess
import time
import os

views = [
    ('dual', 'browser_wheel_dual_sync.png'),
    ('side', 'browser_wheel_side_view.png'),
    ('iso', 'browser_wheel_iso_34.png'),
    ('front_closeup', 'browser_wheel_front_closeup.png'),
    ('rear_closeup', 'browser_wheel_rear_closeup.png'),
    ('front_right_closeup', 'browser_wheel_front_right_closeup.png'),
    ('rear_right_closeup', 'browser_wheel_rear_right_closeup.png'),
]

out_dir = os.path.abspath('web/assets/renders')
os.makedirs(out_dir, exist_ok=True)

chrome_bin = '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome'

for view_name, filename in views:
    out_path = os.path.join(out_dir, filename)
    url = f'http://localhost:8000/?view={view_name}'
    cmd = [
        chrome_bin,
        '--headless=new',
        '--use-gl=angle',
        '--use-angle=swiftshader',
        '--enable-webgl',
        '--enable-unsafe-swiftshader',
        '--window-size=1600,1000',
        '--virtual-time-budget=2000',
        f'--screenshot={out_path}',
        url
    ]
    print(f"Capturing {view_name} -> {filename}...")
    res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    if os.path.exists(out_path):
        size = os.path.getsize(out_path)
        print(f"  ✓ Saved {filename} ({size} bytes)")
    else:
        print(f"  ✗ Failed {filename}: {res.stderr[:200]}")

print("Browser WebGL screenshot capture complete!")
