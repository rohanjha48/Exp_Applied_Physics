"""
Unit tests for security.py
"""
import unittest
import time
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from security import (
    hash_password, verify_password,
    AdminAuth, RateLimiter,
    sanitize_quarter_car_inputs, is_path_safe
)

class TestSecurityEngine(unittest.TestCase):

    def test_password_hashing(self):
        pw = "SuperSecretAutomotiveKey123!$"
        hashed = hash_password(pw)
        self.assertTrue(hashed.startswith("pbkdf2_sha256$600000$"))
        self.assertTrue(verify_password(pw, hashed))
        self.assertFalse(verify_password("WrongPassword", hashed))
        self.assertFalse(verify_password("", hashed))

    def test_admin_auth(self):
        os.environ['ADMIN_API_KEY'] = 'AutomotiveAdminSecret_2026'
        auth = AdminAuth()
        self.assertFalse(auth.is_ephemeral)

        # Valid Bearer
        self.assertTrue(auth.verify_request_auth({'Authorization': 'Bearer AutomotiveAdminSecret_2026'}))
        # Valid X-Admin-Key
        self.assertTrue(auth.verify_request_auth({'X-Admin-Key': 'AutomotiveAdminSecret_2026'}))
        # Invalid Bearer
        self.assertFalse(auth.verify_request_auth({'Authorization': 'Bearer WrongKey'}))
        # Missing header
        self.assertFalse(auth.verify_request_auth({}))

    def test_rate_limiter(self):
        limiter = RateLimiter()
        ip = "192.168.1.100"

        # Allow 5 requests in a 10s window
        for _ in range(5):
            allowed, _ = limiter.is_allowed(ip, endpoint="api_test", max_requests=5, window_seconds=10)
            self.assertTrue(allowed)

        # 6th request should be blocked
        blocked, retry = limiter.is_allowed(ip, endpoint="api_test", max_requests=5, window_seconds=10)
        self.assertFalse(blocked)
        self.assertGreater(retry, 0)

        # Another IP should still be allowed
        allowed_other, _ = limiter.is_allowed("192.168.1.101", endpoint="api_test", max_requests=5, window_seconds=10)
        self.assertTrue(allowed_other)

    def test_input_sanitizer(self):
        # Valid input
        valid_query = {'k': ['25000'], 'c': ['1500'], 'm': ['1400']}
        cleaned, err = sanitize_quarter_car_inputs(valid_query)
        self.assertIsNone(err)
        self.assertEqual(cleaned['front_spring_stiffness'], 25000.0)
        self.assertEqual(cleaned['front_damping_coeff'], 1500.0)
        self.assertEqual(cleaned['sprung_mass'], 1400.0)

        # Non-numeric input
        bad_query = {'k': ['<script>alert(1)</script>']}
        cleaned, err = sanitize_quarter_car_inputs(bad_query)
        self.assertIsNone(cleaned)
        self.assertIn("must be a valid finite number", err)

        # Out-of-bounds input
        too_high = {'k': ['999999999']}
        cleaned, err = sanitize_quarter_car_inputs(too_high)
        self.assertIsNone(cleaned)
        self.assertIn("out of bounds", err)

    def test_path_safety(self):
        root = "/var/www/web"
        self.assertTrue(is_path_safe("/index.html", root))
        self.assertTrue(is_path_safe("/assets/wheel_assembly.glb", root))
        self.assertTrue(is_path_safe("/lib/three.min.js", root))

        # Path traversal
        self.assertFalse(is_path_safe("/../etc/passwd", root))
        self.assertFalse(is_path_safe("/../../main.py", root))
        self.assertFalse(is_path_safe("/assets/../../main.py", root))

        # Hidden dotfiles
        self.assertFalse(is_path_safe("/.env", root))
        self.assertFalse(is_path_safe("/.git/config", root))

        # Forbidden extensions
        self.assertFalse(is_path_safe("/server.py", root))
        self.assertFalse(is_path_safe("/deploy.sh", root))

if __name__ == '__main__':
    unittest.main()
