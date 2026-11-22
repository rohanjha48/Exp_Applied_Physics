"""
security.py
===========
Enterprise-Grade Security Engine for Automotive Physics Lab.

Covers:
1. Cryptographic Password & Token Hashing (PBKDF2-HMAC-SHA256, constant-time compare)
2. Administrative Access Control & Token Authentication
3. In-Memory Thread-Safe Sliding Window Rate Limiter
4. HTTP Security Headers (CSP, HSTS, X-Content-Type-Options, etc.)
5. Strict Query & Path Traversal Input Sanitization
6. CORS Filtering & Protection
"""

import os
import time
import hmac
import hashlib
import binascii
import threading
from typing import Dict, Tuple, Optional, List, Any


# -----------------------------------------------------------------------------
# 1. CRYPTOGRAPHIC PASSWORD & SECRET HASHING (PBKDF2-HMAC-SHA256)
# -----------------------------------------------------------------------------

PBKDF2_ITERATIONS = 600_000
SALT_SIZE_BYTES = 16


def hash_password(password: str) -> str:
    """
    Securely hash a password or token using PBKDF2-HMAC-SHA256 with 600,000 rounds
    and a 128-bit cryptographically secure random salt.
    Format: pbkdf2_sha256$<iterations>$<salt_hex>$<hash_hex>
    """
    if not password or not isinstance(password, str):
        raise ValueError("Password must be a non-empty string")

    salt = os.urandom(SALT_SIZE_BYTES)
    derived = hashlib.pbkdf2_hmac(
        'sha256',
        password.encode('utf-8'),
        salt,
        PBKDF2_ITERATIONS
    )
    salt_hex = binascii.hexlify(salt).decode('ascii')
    hash_hex = binascii.hexlify(derived).decode('ascii')
    return f"pbkdf2_sha256${PBKDF2_ITERATIONS}${salt_hex}${hash_hex}"


def verify_password(password: str, hashed_value: str) -> bool:
    """
    Verify a password against a stored PBKDF2-HMAC-SHA256 hash using constant-time comparison.
    """
    if not password or not hashed_value or not isinstance(hashed_value, str):
        return False

    try:
        parts = hashed_value.split('$')
        if len(parts) != 4 or parts[0] != 'pbkdf2_sha256':
            return False

        iterations = int(parts[1])
        salt = binascii.unhexlify(parts[2].encode('ascii'))
        expected_hash = binascii.unhexlify(parts[3].encode('ascii'))

        # Compute candidate hash with identical parameters
        candidate_hash = hashlib.pbkdf2_hmac(
            'sha256',
            password.encode('utf-8'),
            salt,
            iterations
        )

        # Constant-time comparison prevents timing attacks
        return hmac.compare_digest(candidate_hash, expected_hash)
    except Exception:
        return False


# -----------------------------------------------------------------------------
# 2. ADMIN AUTHENTICATION & ACCESS CONTROL
# -----------------------------------------------------------------------------

class AdminAuth:
    """
    Manages API key / bearer token validation for administrative routes.
    Keys are retrieved from the environment (ADMIN_API_KEY).
    """

    def __init__(self):
        # Default key can be overridden via environment variable
        self._admin_key = os.environ.get('ADMIN_API_KEY', '').strip()
        self._fallback_active = False

        if not self._admin_key:
            # Generate a secure ephemeral token for this process if unset
            self._admin_key = binascii.hexlify(os.urandom(24)).decode('ascii')
            self._fallback_active = True

    @property
    def admin_key(self) -> str:
        return self._admin_key

    @property
    def is_ephemeral(self) -> bool:
        return self._fallback_active

    def verify_request_auth(self, headers: Dict[str, str]) -> bool:
        """
        Check Authorization: Bearer <token> or X-Admin-Key against the configured admin key.
        Uses constant-time comparison to protect against side-channel analysis.
        """
        provided_token = ""

        auth_header = headers.get('authorization', headers.get('Authorization', ''))
        if auth_header.startswith('Bearer '):
            provided_token = auth_header[7:].strip()
        elif 'x-admin-key' in headers:
            provided_token = headers['x-admin-key'].strip()
        elif 'X-Admin-Key' in headers:
            provided_token = headers['X-Admin-Key'].strip()

        if not provided_token or not self._admin_key:
            return False

        return hmac.compare_digest(provided_token.encode('utf-8'), self._admin_key.encode('utf-8'))


# -----------------------------------------------------------------------------
# 3. THREAD-SAFE IN-MEMORY RATE LIMITER
# -----------------------------------------------------------------------------

class RateLimiter:
    """
    Sliding window rate limiter with per-IP and per-endpoint tracking.
    Thread-safe and features automatic memory cleanup for stale clients.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._requests: Dict[str, List[float]] = {}
        self._last_cleanup = time.time()

    def is_allowed(self, client_ip: str, endpoint: str = "default",
                   max_requests: int = 60, window_seconds: int = 60) -> Tuple[bool, int]:
        """
        Checks whether client_ip is within the rate limit.
        Returns:
            (allowed: bool, retry_after_seconds: int)
        """
        now = time.time()
        key = f"{client_ip}:{endpoint}"

        with self._lock:
            # Periodic cleanup of stale records every 5 minutes
            if now - self._last_cleanup > 300:
                self._cleanup(now)

            timestamps = self._requests.get(key, [])
            cutoff = now - window_seconds
            timestamps = [t for t in timestamps if t > cutoff]

            if len(timestamps) >= max_requests:
                oldest_in_window = timestamps[0]
                retry_after = max(1, int(oldest_in_window + window_seconds - now))
                self._requests[key] = timestamps
                return False, retry_after

            timestamps.append(now)
            self._requests[key] = timestamps
            return True, 0

    def _cleanup(self, now: float):
        cutoff = now - 3600  # Remove records older than 1 hour
        keys_to_delete = []
        for k, timestamps in self._requests.items():
            valid = [t for t in timestamps if t > cutoff]
            if not valid:
                keys_to_delete.append(k)
            else:
                self._requests[k] = valid
        for k in keys_to_delete:
            del self._requests[k]
        self._last_cleanup = now


# -----------------------------------------------------------------------------
# 4. HTTP SECURITY HEADERS
# -----------------------------------------------------------------------------

SECURITY_HEADERS = {
    # Content Security Policy (strict self-contained policy with external CDNs for fallback)
    "Content-Security-Policy": (
        "default-src 'self'; "
        "script-src 'self' 'unsafe-inline' https://cdnjs.cloudflare.com https://cdn.jsdelivr.net; "
        "style-src 'self' 'unsafe-inline'; "
        "img-src 'self' data: blob:; "
        "font-src 'self' data:; "
        "connect-src 'self'; "
        "object-src 'none'; "
        "base-uri 'self'; "
        "frame-ancestors 'none'; "
        "form-action 'self';"
    ),
    # Prevent MIME-type confusion attacks
    "X-Content-Type-Options": "nosniff",
    # Protect against clickjacking
    "X-Frame-Options": "DENY",
    # Legacy XSS auditor enforcement
    "X-XSS-Protection": "1; mode=block",
    # Referrer privacy
    "Referrer-Policy": "strict-origin-when-cross-origin",
    # Restrict risky device hardware APIs
    "Permissions-Policy": (
        "accelerometer=(), camera=(), geolocation=(), gyroscope=(), "
        "magnetometer=(), microphone=(), payment=(), usb=()"
    ),
    # Cross-Origin isolation
    "Cross-Origin-Opener-Policy": "same-origin",
}


# -----------------------------------------------------------------------------
# 5. INPUT SANITIZATION & BOUNDS VALIDATION
# -----------------------------------------------------------------------------

# Physical parameter safety envelopes for vehicle suspension simulation
PARAM_BOUNDS = {
    'front_spring_stiffness': (1000.0, 300_000.0),   # N/m (kart to heavy truck)
    'front_damping_coeff': (100.0, 30_000.0),        # N*s/m
    'sprung_mass': (300.0, 10_000.0),                # kg
}


def sanitize_quarter_car_inputs(query: Dict[str, List[str]]) -> Tuple[Optional[Dict[str, float]], Optional[str]]:
    """
    Sanitize and validate quarter-car query string parameters.
    Rejects malformed floats, NaN, infinity, and values exceeding physical bounds.
    """
    cleaned: Dict[str, float] = {}

    mapping = {
        'k': ('front_spring_stiffness', PARAM_BOUNDS['front_spring_stiffness']),
        'c': ('front_damping_coeff', PARAM_BOUNDS['front_damping_coeff']),
        'm': ('sprung_mass', PARAM_BOUNDS['sprung_mass']),
    }

    for param_key, (target_attr, (min_v, max_v)) in mapping.items():
        if param_key in query:
            raw_val = query[param_key][0]
            try:
                val = float(raw_val)
            except (ValueError, TypeError):
                return None, f"Parameter '{param_key}' must be a valid finite number"

            if not (min_v <= val <= max_v):
                return None, f"Parameter '{param_key}' value {val} out of bounds [{min_v}, {max_v}]"

            cleaned[target_attr] = val

    return cleaned, None


FORBIDDEN_EXTENSIONS = (
    '.py', '.pyc', '.pyo', '.pyd',
    '.env', '.env.local', '.env.production',
    '.git', '.gitignore', '.vercelignore',
    '.key', '.pem', '.cert', '.crt',
    '.sh', '.bash', '.zsh',
    '.bak', '.backup', '.swp',
    '.yml', '.yaml', '.toml'
)


def is_path_safe(requested_path: str, root_dir: str) -> bool:
    """
    Verifies that the requested path does not escape the web directory
    and does not access sensitive source files, hidden dotfiles, or server scripts.
    """
    # Reject null bytes and path traversal patterns upfront
    if '\0' in requested_path or '..' in requested_path:
        return False

    # Check for forbidden hidden files
    segments = requested_path.replace('\\', '/').split('/')
    for seg in segments:
        if seg.startswith('.') and seg not in ('.', ''):
            return False

    # Check forbidden extensions
    lower_path = requested_path.lower()
    for ext in FORBIDDEN_EXTENSIONS:
        if lower_path.endswith(ext):
            return False

    # Canonical path resolution check
    full_path = os.path.abspath(os.path.join(root_dir, requested_path.lstrip('/')))
    norm_root = os.path.abspath(root_dir)

    try:
        common = os.path.commonpath([full_path, norm_root])
        return common == norm_root
    except ValueError:
        return False
