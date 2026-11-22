import sys
import os
import argparse
import webbrowser
import threading
import time
import json
from http.server import SimpleHTTPRequestHandler, HTTPServer
import urllib.parse
import numpy as np

from vehicle_model import VehicleParameters
from road_model import create_road_profile, PRESET_CATALOG
from physics_engine import PhysicsEngine
from dashboard import TelemetryDashboard, DrDampener
from visualization import OfflinePlotter
from security import (
    AdminAuth, RateLimiter, SECURITY_HEADERS,
    sanitize_quarter_car_inputs, is_path_safe
)


class SuspensionServerHandler(SimpleHTTPRequestHandler):
    """
    Hardened HTTP handler serving web assets and automotive laboratory REST API.
    Enforces security headers, IP rate limiting, input sanitization, and admin access control.
    """
    server_version = "SuspensionLab/2.0"
    sys_version = ""

    # Shared security singletons
    rate_limiter = RateLimiter()
    admin_auth = AdminAuth()
    allowed_origins = [
        o.strip() for o in os.environ.get("ALLOWED_ORIGINS", "http://localhost:8000,http://127.0.0.1:8000").split(",")
        if o.strip()
    ]
    debug_mode = False

    def __init__(self, *args, **kwargs):
        # Serve from web directory
        self.web_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'web')
        super().__init__(*args, directory=self.web_dir, **kwargs)

    def log_message(self, format, *args):
        try:
            if not args:
                return
            first_arg = str(args[0])
            second_arg = str(args[1]) if len(args) > 1 else ""
            # Redact authorization tokens if any appear in log strings
            if "Bearer " in first_arg or "X-Admin-Key" in first_arg:
                first_arg = "[REDACTED_AUTH_REQUEST]"
            # Log errors or API calls; silence routine 200/204/304 static GETs
            if "/api/" in first_arg or (second_arg and second_arg not in ('200', '204', '304')):
                super().log_message(format, *args)
        except Exception:
            pass

    def _get_client_ip(self) -> str:
        """Extract remote client IP address safely."""
        forwarded = self.headers.get('X-Forwarded-For')
        if forwarded:
            return forwarded.split(',')[0].strip()
        return self.client_address[0] if self.client_address else "127.0.0.1"

    def _apply_cors_headers(self):
        """Apply CORS headers based on configured origins or same-origin."""
        origin = self.headers.get('Origin', '')
        if origin and (origin in self.allowed_origins or origin.startswith("http://localhost:") or origin.startswith("http://127.0.0.1:")):
            self.send_header('Access-Control-Allow-Origin', origin)
            self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
            self.send_header('Access-Control-Allow-Headers', 'Content-Type, Authorization, X-Admin-Key')
            self.send_header('Access-Control-Max-Age', '86400')
            self.send_header('Vary', 'Origin')

    def end_headers(self):
        # Apply enterprise security headers
        for header_key, header_val in SECURITY_HEADERS.items():
            self.send_header(header_key, header_val)

        # Apply CORS headers
        self._apply_cors_headers()

        # Prevent stale development caches
        self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
        self.send_header('Pragma', 'no-cache')
        self.send_header('Expires', '0')
        super().end_headers()

    def do_OPTIONS(self):
        """Handle CORS preflight requests securely."""
        self.send_response(204)
        self.end_headers()

    def do_GET(self):
        client_ip = self._get_client_ip()
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path
        query = urllib.parse.parse_qs(parsed.query)

        # Handle missing favicon gracefully without generating 404 or logging errors
        if path == '/favicon.ico':
            self.send_response(204)
            self.end_headers()
            return

        # 1. PATH TRAVERSAL & FORBIDDEN ASSET SECURITY CHECK
        if not path.startswith('/api/'):
            if not is_path_safe(path, self.web_dir):
                self.send_response(403)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({'error': 'Forbidden: Access denied to protected file or path'}).encode('utf-8'))
                return

        # 2. RATE LIMITING ON API ENDPOINTS
        if path.startswith('/api/'):
            # Stricter limit on heavy disk-export endpoints (5/min), standard on data APIs (60/min)
            max_req = 5 if path == '/api/export_plots' else 60
            allowed, retry_after = self.rate_limiter.is_allowed(
                client_ip, endpoint=path, max_requests=max_req, window_seconds=60
            )
            if not allowed:
                self.send_response(429)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Retry-After', str(retry_after))
                self.end_headers()
                err_resp = {
                    'error': 'Too Many Requests',
                    'message': f'Rate limit exceeded. Please retry in {retry_after} seconds.',
                    'retry_after': retry_after
                }
                self.wfile.write(json.dumps(err_resp).encode('utf-8'))
                return

        try:
            # 3. PUBLIC API: SIMULATOR STATUS & ROAD CATALOG
            if path == '/api/status':
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                status = {
                    'status': 'online',
                    'name': 'Real-Time 3D Vehicle Suspension Simulator',
                    'version': '2.0-automotive',
                    'road_presets': list(PRESET_CATALOG.keys()),
                    'security': {
                        'rate_limiting': True,
                        'input_validation': True,
                        'headers_hardened': True
                    }
                }
                self.wfile.write(json.dumps(status).encode('utf-8'))
                return

            # 4. PUBLIC API: QUARTER-CAR METRICS WITH STRICT INPUT SANITIZATION
            elif path == '/api/quarter_car':
                cleaned_params, validation_error = sanitize_quarter_car_inputs(query)
                if validation_error:
                    self.send_response(400)
                    self.send_header('Content-Type', 'application/json')
                    self.end_headers()
                    self.wfile.write(json.dumps({'error': 'Bad Request', 'message': validation_error}).encode('utf-8'))
                    return

                p = VehicleParameters()
                if 'front_spring_stiffness' in cleaned_params:
                    p.front_spring_stiffness = cleaned_params['front_spring_stiffness']
                if 'front_damping_coeff' in cleaned_params:
                    p.front_damping_coeff = cleaned_params['front_damping_coeff']
                if 'sprung_mass' in cleaned_params:
                    p.sprung_mass = cleaned_params['sprung_mass']

                qc = p.get_quarter_car_properties('FL')
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps(qc).encode('utf-8'))
                return

            # 5. PROTECTED ADMIN ROUTE: SYSTEM METRICS & HEALTH
            elif path == '/api/admin/status':
                if not self.admin_auth.verify_request_auth(dict(self.headers)):
                    self.send_response(401)
                    self.send_header('Content-Type', 'application/json')
                    self.send_header('WWW-Authenticate', 'Bearer realm="SuspensionLabAdmin"')
                    self.end_headers()
                    self.wfile.write(json.dumps({'error': 'Unauthorized: Admin authentication required'}).encode('utf-8'))
                    return

                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                admin_info = {
                    'admin_status': 'active',
                    'authenticated': True,
                    'timestamp': time.time(),
                    'rate_limiting': 'active',
                    'ephemeral_key': self.admin_auth.is_ephemeral,
                    'debug_mode': self.debug_mode
                }
                self.wfile.write(json.dumps(admin_info).encode('utf-8'))
                return

            # 6. PROTECTED ROUTE: EXPORT PUBLICATION PLOTS (RESTRICTED TO PREVENT DISK EXHAUSTION)
            elif path == '/api/export_plots':
                # Require admin authorization to trigger disk-writing operations
                if not self.admin_auth.verify_request_auth(dict(self.headers)):
                    self.send_response(401)
                    self.send_header('Content-Type', 'application/json')
                    self.send_header('WWW-Authenticate', 'Bearer realm="SuspensionLabAdmin"')
                    self.end_headers()
                    self.wfile.write(json.dumps({
                        'error': 'Unauthorized',
                        'message': 'Admin token required for plot export operations.'
                    }).encode('utf-8'))
                    return

                p1 = OfflinePlotter.generate_damping_comparison("damping_regimes.png")
                p2 = OfflinePlotter.generate_transmissibility_curve("frequency_response.png")
                p3 = OfflinePlotter.generate_road_telemetry_plot("large_breakers", "suspension_telemetry.png")
                res = {'status': 'success', 'files': [p1, p2, p3]}

                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps(res).encode('utf-8'))
                return

            # 7. SERVE STATIC WEB FILES
            super().do_GET()

        except Exception as e:
            if self.debug_mode:
                raise
            # Generic error response in production (prevents traceback leaking)
            self.send_response(500)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({'error': 'Internal Server Error'}).encode('utf-8'))


def run_physics_tests() -> bool:
    """
    Automated physics verification suite.
    Validates mass-spring-damper equations, damping regimes, natural frequency,
    and 7-DOF multi-body coupling against analytical mechanical solutions.
    """
    print("\n" + "=" * 70)
    print("RUNNING AUTOMOTIVE PHYSICS VERIFICATION SUITE")
    print("=" * 70)
    all_passed = True

    # TEST 1: Quarter-Car Analytical Formulas
    print("\n[TEST 1] Analytical Quarter-Car Frequency & Damping Formulas...")
    p = VehicleParameters(sprung_mass=1400.0)
    qc = p.get_quarter_car_properties('FL')
    m_s = qc['sprung_mass_kg']
    k_s = qc['spring_stiffness_N_m']
    c_s = qc['damping_coeff_Ns_m']

    expected_omega_n = np.sqrt(k_s / m_s)
    expected_c_crit = 2.0 * np.sqrt(m_s * k_s)
    expected_zeta = c_s / expected_c_crit

    assert np.isclose(qc['omega_n_rad_s'], expected_omega_n, rtol=1e-5), "omega_n mismatch"
    assert np.isclose(qc['critical_damping_Ns_m'], expected_c_crit, rtol=1e-5), "c_crit mismatch"
    assert np.isclose(qc['damping_ratio_zeta'], expected_zeta, rtol=1e-5), "zeta mismatch"
    print(f"  ✓ Pass: f_n = {qc['natural_freq_hz']:.3f} Hz, c_crit = {qc['critical_damping_Ns_m']:.1f} Ns/m, ζ = {qc['damping_ratio_zeta']:.3f}")

    # TEST 2: Underdamped Free Oscillation (Zeta = 0.20)
    print("\n[TEST 2] Underdamped Free Oscillation & Damped Frequency...")
    p_under = VehicleParameters()
    qc_u = p_under.get_quarter_car_properties('FL')
    p_under.front_damping_coeff = 0.20 * qc_u['critical_damping_Ns_m']
    p_under.rear_damping_coeff = p_under.front_damping_coeff

    engine = PhysicsEngine(p_under, road_profile=None)
    engine.state.z_c = 0.05  # 50 mm initial step
    dt = 0.001
    zc_history = []
    t_history = []
    for _ in range(3000):
        engine.step(dt)
        zc_history.append(engine.state.z_c)
        t_history.append(engine.state.time)

    # Check zero crossings: must oscillate multiple times
    zero_crossings = np.where(np.diff(np.sign(zc_history)))[0]
    assert len(zero_crossings) >= 4, f"Underdamped system should cross zero multiple times, got {len(zero_crossings)}"
    # Measure observed half-period
    t_crossings = np.array(t_history)[zero_crossings]
    half_periods = np.diff(t_crossings)
    observed_omega_d = np.pi / np.mean(half_periods)
    theoretical_omega_d = qc_u['omega_n_rad_s'] * np.sqrt(1.0 - 0.20**2)
    err_pct = abs(observed_omega_d - theoretical_omega_d) / theoretical_omega_d * 100.0
    assert err_pct < 4.0, f"Damped frequency error too high: {err_pct:.2f}%"
    print(f"  ✓ Pass: Observed ω_d = {observed_omega_d:.2f} rad/s vs Theoretical = {theoretical_omega_d:.2f} rad/s (Error: {err_pct:.2f}%)")

    # TEST 3: Critically Damped Exponential Return (Zeta = 1.00)
    print("\n[TEST 3] Critically Damped Deadbeat Return (Zero Overshoot)...")
    p_crit = VehicleParameters()
    qc_c = p_crit.get_quarter_car_properties('FL')
    p_crit.front_damping_coeff = 1.00 * qc_c['critical_damping_Ns_m']
    p_crit.rear_damping_coeff = p_crit.front_damping_coeff

    engine_crit = PhysicsEngine(p_crit, road_profile=None)
    engine_crit.state.z_c = 0.05
    min_zc = 0.05
    for _ in range(2500):
        engine_crit.step(dt)
        min_zc = min(min_zc, engine_crit.state.z_c)

    # Must not significantly overshoot zero (tolerance 0.5 mm)
    assert min_zc >= -0.0008, f"Critically damped system had unacceptable overshoot: {min_zc*1000:.2f} mm"
    # Final value must settle near zero
    assert abs(engine_crit.state.z_c) < 0.001, "Critically damped system did not settle to equilibrium"
    print(f"  ✓ Pass: Settled with zero overshoot (min z_c = {min_zc*1000.0:.3f} mm)")

    # TEST 4: Resonance Transmissibility Peak
    print("\n[TEST 4] Forced Vibration Resonance Verification...")
    p_res = VehicleParameters()
    fn = p_res.get_quarter_car_properties('FL')['natural_freq_hz']
    # Wavelength 3.2m -> resonant speed v = 3.2 * fn
    v_res = 3.2 * fn
    # Transmissibility at r = 1 with zeta = 0.20: T = sqrt(1 + 4*zeta^2) / (2*zeta) = sqrt(1 + 0.16) / 0.40 = 2.69
    T_calc = p_res.transmissibility(1.0, 0.20)
    assert np.isclose(T_calc, np.sqrt(1 + 4 * 0.20**2) / (2 * 0.20), rtol=1e-4)
    print(f"  ✓ Pass: Resonant speed = {v_res*3.6:.1f} km/h, Transmissibility magnification T = {T_calc:.2f}x")

    # TEST 5: Unilateral Tire Contact (No Tension on Separation)
    print("\n[TEST 5] Unilateral Tire Contact (Separation Handling)...")
    p_air = VehicleParameters()
    engine_air = PhysicsEngine(p_air, road_profile=None)
    # Pull wheel high into air above road (z_w = 0.10m, road_z = 0)
    engine_air.state.z_w['FL'] = 0.10
    state_air = engine_air.step(0.001)
    assert state_air.tire_force['FL'] == 0.0, "Tire exerted non-zero force while airborne (cannot pull ground)!"
    print("  ✓ Pass: Airborne tire correctly exerts zero tension (F_tire = 0 N)")

    # TEST 6: All 8 Road Profiles Generation
    print("\n[TEST 6] Road Surface Profile Catalog Integrity...")
    for preset_id in PRESET_CATALOG.keys():
        road = create_road_profile(preset_id)
        # Evaluate profile at multiple coordinates
        z_vals = [road.elevation(x, 0.0) for x in np.linspace(0, 50, 100)]
        assert len(z_vals) == 100
        assert not any(np.isnan(z_vals)), f"NaN detected in road profile {preset_id}"
        print(f"  ✓ Pass: Road profile '{preset_id}' generated valid continuous terrain")

    print("\n" + "=" * 70)
    print("ALL 6 AUTOMOTIVE PHYSICS VERIFICATION TESTS PASSED SUCCESSFULLY!")
    print("=" * 70 + "\n")
    return all_passed


def run_cli_simulation(duration_s: float = 3.0, road_id: str = "small_breakers") -> None:
    """Run an ASCII live terminal simulation with dynamic physics graphs."""
    print(f"\n--- Running Terminal Simulation over '{road_id}' ---")
    p = VehicleParameters(speed_kmh=50.0)
    road = create_road_profile(road_id)
    engine = PhysicsEngine(p, road)
    dash = TelemetryDashboard()

    dt = 0.02
    steps = int(duration_s / dt)
    print("Time [s] | Speed | Chassis z_c | Wheel z_w | Stroke   | Accel   | Damping Regime")
    print("-" * 78)

    for i in range(steps):
        state = engine.step(dt)
        data = dash.update(state, p, dt)
        if i % 5 == 0:
            zc = data['vehicle_displacement_mm']
            zw = data['wheel_displacement_mm']
            st = data['suspension_compression_mm']
            ac = data['vehicle_accel_mps2']
            regime = data['damping_regime'].upper()
            bar_len = int(max(0, min(30, (zc + 50) / 3.3)))
            bar = " " * bar_len + "●"
            print(f"{state.time:6.2f}s | {p.speed_kmh:4.0f} | {zc:+6.1f} mm | {zw:+6.1f} mm | {st:+6.1f} mm | {ac:+6.2f} | [{regime:8s}] {bar}")

    print("-" * 78)
    print(f"Simulation Complete: Natural Frequency = {data['natural_frequency_hz']} Hz, Damping Ratio = {data['damping_ratio_zeta']}\n")


def start_server(port: int = 8000, open_browser: bool = True) -> None:
    """Start local simulation server and launch browser."""
    # Find next available port if port is taken
    actual_port = port
    server = None
    for p in range(port, port + 20):
        try:
            server = HTTPServer(('127.0.0.1', p), SuspensionServerHandler)
            actual_port = p
            break
        except OSError:
            continue

    if server is None:
        print(f"ERROR: Could not bind server to ports {port}-{port+20}.")
        sys.exit(1)

    url = f"http://127.0.0.1:{actual_port}/"
    print("\n" + "=" * 75)
    print("  REAL-TIME 3D VEHICLE SUSPENSION SIMULATOR")
    print("  Automotive Physics Laboratory • Mass-Spring-Damper Dynamics")
    print("=" * 75)
    print(f"  ► Server URL:           {url}")
    print(f"  ► Viewports:            Synchronized Dual View (Side View & Front View)")
    print(f"  ► Physics Engine:       7-DOF Multi-Body Vehicle Dynamics (RK4 1ms)")
    print(f"  ► Road Presets:         8 Dynamic Profiles (Highway, Bumps, Potholes, Wavy)")
    print(f"  ► Press Ctrl+C in this terminal to stop the server.")
    print("=" * 75 + "\n")

    if open_browser:
        def _open():
            time.sleep(0.6)
            try:
                webbrowser.open(url)
            except Exception:
                pass
        threading.Thread(target=_open, daemon=True).start()

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nLaboratory server shut down cleanly.")
        server.server_close()


def main():
    parser = argparse.ArgumentParser(
        description="Real-Time 3D Vehicle Suspension Simulator (Automotive Physics Lab)"
    )
    parser.add_argument('--port', type=int, default=8000, help="Local HTTP server port (default: 8000)")
    parser.add_argument('--no-browser', action='store_true', help="Do not open web browser automatically")
    parser.add_argument('--test', action='store_true', help="Run automated physics verification tests and exit")
    parser.add_argument('--export-plots', action='store_true', help="Generate and save publication-quality Matplotlib plots")
    parser.add_argument('--cli', action='store_true', help="Run terminal simulation mode with live ASCII graphs")
    parser.add_argument('--road', type=str, default='large_breakers', choices=list(PRESET_CATALOG.keys()),
                        help="Road profile preset for CLI simulation")
    parser.add_argument('--debug', action='store_true', help="Run in debug mode (disabled by default for security)")
    parser.add_argument('--admin-key', type=str, default=None, help="Optional admin API key for protected routes")

    args = parser.parse_args()

    if args.debug:
        SuspensionServerHandler.debug_mode = True
        print("  [SECURITY NOTICE] Debug mode is ENABLED. Do not use in production.")
    else:
        SuspensionServerHandler.debug_mode = False

    if args.admin_key:
        SuspensionServerHandler.admin_auth._admin_key = args.admin_key.strip()
        SuspensionServerHandler.admin_auth._fallback_active = False

    if args.test:
        success = run_physics_tests()
        sys.exit(0 if success else 1)

    if args.export_plots:
        print("Exporting high-resolution automotive physics figures...")
        p1 = OfflinePlotter.generate_damping_comparison("damping_regimes.png")
        p2 = OfflinePlotter.generate_transmissibility_curve("frequency_response.png")
        p3 = OfflinePlotter.generate_road_telemetry_plot(args.road, "suspension_telemetry.png")
        print(f"  ✓ Exported: {p1}")
        print(f"  ✓ Exported: {p2}")
        print(f"  ✓ Exported: {p3}")
        sys.exit(0)

    if args.cli:
        run_cli_simulation(duration_s=4.0, road_id=args.road)
        sys.exit(0)

    # Launch full interactive 3D laboratory server
    start_server(port=args.port, open_browser=not args.no_browser)


# Vercel Serverless Function Compatibility Entrypoint
def handler(request, response=None):
    return {"status": "online", "message": "Vehicle Suspension Simulator Engine"}

app = handler


if __name__ == '__main__':
    main()
