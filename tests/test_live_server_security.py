"""
Comprehensive Live HTTP Security Verification Script
Starts the server on port 8099 with a known test admin key,
runs automated penetration & compliance checks, and asserts all security defenses.
"""

import os
import sys
import time
import json
import urllib.request
import urllib.error
import subprocess

PORT = 8099
ADMIN_KEY = "test_admin_secret_key_abcdef123456"
BASE_URL = f"http://127.0.0.1:{PORT}"

def main():
    print("=" * 70)
    print("STARTING AUTOMATED LIVE HTTP SECURITY VERIFICATION")
    print("=" * 70)

    # 1. Start test server
    env = os.environ.copy()
    env['ADMIN_API_KEY'] = ADMIN_KEY
    server_proc = subprocess.Popen(
        [sys.executable, "main.py", "--port", str(PORT), "--no-browser"],
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )

    time.sleep(1.2)

    try:
        # TEST 1: Security Headers on Static / Root
        print("\n[CHECK 1] HTTP Security Headers on Root...")
        req = urllib.request.Request(f"{BASE_URL}/")
        with urllib.request.urlopen(req) as resp:
            headers = dict(resp.info())
            assert resp.status == 200
            assert "Content-Security-Policy" in headers, "Missing CSP header"
            assert headers.get("X-Content-Type-Options") == "nosniff", "Missing X-Content-Type-Options"
            assert headers.get("X-Frame-Options") == "DENY", "Missing X-Frame-Options"
            assert "strict-origin-when-cross-origin" in headers.get("Referrer-Policy", ""), "Missing Referrer-Policy"
            print("  ✓ Pass: All security headers present and strictly configured.")

        # TEST 2: Public Status API
        print("\n[CHECK 2] Public /api/status Endpoint...")
        with urllib.request.urlopen(f"{BASE_URL}/api/status") as resp:
            assert resp.status == 200
            data = json.loads(resp.read().decode('utf-8'))
            assert data['status'] == 'online'
            assert data['security']['headers_hardened'] is True
            print("  ✓ Pass: Public status endpoint online with verified security metadata.")

        # TEST 3: Input Sanitization & Bounds Checking on /api/quarter_car
        print("\n[CHECK 3] Input Sanitization & Range Validation on /api/quarter_car...")
        # Valid query
        with urllib.request.urlopen(f"{BASE_URL}/api/quarter_car?k=25000&c=1500&m=1400") as resp:
            assert resp.status == 200
            qc = json.loads(resp.read().decode('utf-8'))
            assert "natural_freq_hz" in qc
            print("  ✓ Pass: Valid parameter query returned 200 OK.")

        # Malicious / Out of bounds query
        try:
            urllib.request.urlopen(f"{BASE_URL}/api/quarter_car?k=999999999")
            assert False, "Should have returned 400 for out-of-bounds input"
        except urllib.error.HTTPError as e:
            assert e.code == 400
            body = json.loads(e.read().decode('utf-8'))
            assert body['error'] == 'Bad Request'
            print("  ✓ Pass: Out-of-bounds input rejected with 400 Bad Request.")

        # XSS / Non-numeric query injection
        try:
            urllib.request.urlopen(f"{BASE_URL}/api/quarter_car?k=<script>alert(1)</script>")
            assert False, "Should have returned 400 for non-numeric input"
        except urllib.error.HTTPError as e:
            assert e.code == 400
            print("  ✓ Pass: Malicious non-numeric injection rejected with 400 Bad Request.")

        # TEST 4: Protected Admin Route Authentication
        print("\n[CHECK 4] Protected Admin Route /api/admin/status...")
        # Unauthenticated request -> 401
        try:
            urllib.request.urlopen(f"{BASE_URL}/api/admin/status")
            assert False, "Should have returned 401 Unauthorized"
        except urllib.error.HTTPError as e:
            assert e.code == 401
            assert 'WWW-Authenticate' in dict(e.headers)
            print("  ✓ Pass: Unauthenticated access blocked with 401 Unauthorized & WWW-Authenticate.")

        # Authenticated with Bearer token -> 200
        req_auth = urllib.request.Request(
            f"{BASE_URL}/api/admin/status",
            headers={"Authorization": f"Bearer {ADMIN_KEY}"}
        )
        with urllib.request.urlopen(req_auth) as resp:
            assert resp.status == 200
            admin_data = json.loads(resp.read().decode('utf-8'))
            assert admin_data['authenticated'] is True
            print("  ✓ Pass: Authenticated admin request successfully validated (200 OK).")

        # TEST 5: Protected /api/export_plots Authentication
        print("\n[CHECK 5] Disk-Exhaustion Protection on /api/export_plots...")
        try:
            urllib.request.urlopen(f"{BASE_URL}/api/export_plots")
            assert False, "Should have returned 401 Unauthorized"
        except urllib.error.HTTPError as e:
            assert e.code == 401
            print("  ✓ Pass: Unauthenticated plot export blocked with 401 Unauthorized.")

        # TEST 6: Path Traversal & Forbidden File Probing
        print("\n[CHECK 6] Path Traversal & Sensitive File Leak Prevention...")
        probed_paths = [
            "/../main.py",
            "/.env",
            "/.git/config",
            "/server.py",
            "/deploy.sh",
        ]
        for probe in probed_paths:
            try:
                urllib.request.urlopen(f"{BASE_URL}{probe}")
                assert False, f"Should have blocked forbidden path: {probe}"
            except urllib.error.HTTPError as e:
                assert e.code in (403, 404), f"Path {probe} returned unexpected code {e.code}"
                print(f"  ✓ Pass: Path traversal/probe '{probe}' safely blocked with HTTP {e.code}.")

        # TEST 7: Rate Limiting Enforcement
        print("\n[CHECK 7] In-Memory Rate Limiting Enforcement...")
        rate_req = urllib.request.Request(
            f"{BASE_URL}/api/export_plots",
            headers={"Authorization": f"Bearer {ADMIN_KEY}"}
        )
        # Limit on /api/export_plots is 5 requests/minute
        rate_limited = False
        for i in range(8):
            try:
                with urllib.request.urlopen(rate_req) as resp:
                    pass
            except urllib.error.HTTPError as e:
                if e.code == 429:
                    rate_limited = True
                    retry_header = e.headers.get("Retry-After")
                    assert retry_header is not None, "Missing Retry-After header"
                    print(f"  ✓ Pass: 429 Too Many Requests triggered at iteration {i+1} (Retry-After: {retry_header}s).")
                    break

        assert rate_limited, "Rate limiting was not triggered as expected!"

        # TEST 8: OPTIONS Preflight CORS Handling
        print("\n[CHECK 8] CORS Preflight OPTIONS Handling...")
        opts_req = urllib.request.Request(f"{BASE_URL}/api/status", method="OPTIONS")
        opts_req.add_header("Origin", "http://localhost:8000")
        with urllib.request.urlopen(opts_req) as resp:
            assert resp.status == 204
            assert resp.headers.get("Access-Control-Allow-Origin") == "http://localhost:8000"
            assert "GET" in resp.headers.get("Access-Control-Allow-Methods", "")
            print("  ✓ Pass: Preflight OPTIONS request returned 204 No Content with CORS headers.")

        print("\n" + "=" * 70)
        print("ALL 8 LIVE HTTP SECURITY CHECKS PASSED PERFECTLY!")
        print("=" * 70)
        return True

    finally:
        server_proc.terminate()
        server_proc.wait()

if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1)
