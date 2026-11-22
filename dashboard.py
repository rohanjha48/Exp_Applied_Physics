"""
dashboard.py
============
Telemetry data manager, live graph time-series buffers,
"Dr. Dampener" contextual humorous commentary engine,
and educational physics explanation generator.
"""

from collections import deque
from typing import Dict, Any, List, Optional
import numpy as np

from vehicle_model import VehicleParameters
from physics_engine import PhysicsState


class DrDampener:
    """
    Contextual humorous engineering assistant.
    Analyzes live vehicle telemetry and outputs entertaining yet
    scientifically sound status messages based on genuine physics conditions.
    """

    def __init__(self):
        self.last_message: str = "Welcome to the Suspension Lab! Ready for test run."
        self.last_mood: str = "ready"  # ready, happy, bouncing, shocked, dancing, sleepy, approved
        self.message_cooldown: float = 0.0
        self.zero_crossings: int = 0
        self.last_zc_sign: float = 0.0
        self.oscillation_timer: float = 0.0

    def evaluate(self, state: PhysicsState, params: VehicleParameters, dt: float) -> Dict[str, str]:
        """
        Evaluate physics state and determine Dr. Dampener's commentary.
        """
        self.message_cooldown -= dt
        self.oscillation_timer += dt

        # Track oscillations / zero crossings of chassis velocity or displacement
        current_sign = np.sign(state.z_c)
        if current_sign != self.last_zc_sign and abs(state.z_c) > 0.003:
            self.zero_crossings += 1
            self.last_zc_sign = current_sign

        if self.oscillation_timer > 2.5:
            # Reset window
            self.zero_crossings = 0
            self.oscillation_timer = 0.0

        # Don't switch messages too frenetically (minimum 1.2s between updates unless severe event)
        can_update = self.message_cooldown <= 0.0

        qc = params.get_quarter_car_properties('FL')
        zeta = qc['damping_ratio_zeta']
        fn = qc['natural_freq_hz']

        # Check for severe events first (pothole hit, wheel liftoff, bump stop)
        min_tire_force = min(state.tire_force.values())
        max_stroke = max(abs(s) for s in state.susp_stroke.values())
        min_road_z = min(state.road_z.values())
        max_accel = abs(state.ddz_c)

        # 1. Wheel liftoff
        if min_tire_force <= 0.01 and abs(state.z_c) > 0.02:
            self.last_message = "Airborne wheel! Contact lost with reality (and tarmac)!"
            self.last_mood = "shocked"
            self.message_cooldown = 1.8
            return {'message': self.last_message, 'mood': self.last_mood}

        # 2. Bump stop bottoming out
        if max_stroke >= params.bump_stop_clearance * 0.95:
            self.last_message = "Ouch! Bump stop hit. The rubber cushion just saved your chassis."
            self.last_mood = "shocked"
            self.message_cooldown = 1.8
            return {'message': self.last_message, 'mood': self.last_mood}

        # 3. Pothole drop
        if min_road_z < -0.025 and state.ddz_w['FL'] < -15.0:
            self.last_message = "Pothole detected! Your suspension has questions."
            self.last_mood = "shocked"
            self.message_cooldown = 1.8
            return {'message': self.last_message, 'mood': self.last_mood}

        if not can_update:
            return {'message': self.last_message, 'mood': self.last_mood}

        # 4. Resonance check
        # If speed produces excitation frequency close to natural frequency
        # For wavy road with 3.2m wavelength:
        v_ms = params.speed_ms
        if v_ms > 1.0:
            f_road = v_ms / 3.2
            freq_ratio = f_road / fn if fn > 0 else 1.0
            if abs(freq_ratio - 1.0) < 0.12 and abs(state.z_c) > 0.025:
                self.last_message = "Resonance alert! The car is having a dance party at its natural frequency."
                self.last_mood = "dancing"
                self.message_cooldown = 2.2
                return {'message': self.last_message, 'mood': self.last_mood}

        # 5. Underdamped bounce
        if (zeta < 0.35 and self.zero_crossings >= 3) or (zeta < 0.20 and abs(state.z_c) > 0.015):
            self.last_message = "Too much bouncing! Is this a car or a trampoline?"
            self.last_mood = "bouncing"
            self.message_cooldown = 2.0
            return {'message': self.last_message, 'mood': self.last_mood}

        if zeta < 0.22:
            self.last_message = "Underdamped: The car just cannot calm down. Endless oscillations!"
            self.last_mood = "bouncing"
            self.message_cooldown = 2.0
            return {'message': self.last_message, 'mood': self.last_mood}

        # 6. Overdamped
        if zeta > 1.8:
            self.last_message = "Overdamped: The suspension is moving like it has Monday morning energy. Rock hard!"
            self.last_mood = "sleepy"
            self.message_cooldown = 2.0
            return {'message': self.last_message, 'mood': self.last_mood}

        # 7. Critically damped / optimal damping
        if 0.62 <= zeta <= 0.78 and max_accel < 3.0 and abs(state.z_c) < 0.03:
            self.last_message = "Perfect damping (~0.7 critical). The physics professor approves."
            self.last_mood = "approved"
            self.message_cooldown = 2.5
            return {'message': self.last_message, 'mood': self.last_mood}

        # 8. Smooth road
        if abs(min_road_z) < 0.003 and max_accel < 0.35:
            self.last_message = "Smooth road. The suspension is enjoying its vacation."
            self.last_mood = "happy"
            self.message_cooldown = 2.5
            return {'message': self.last_message, 'mood': self.last_mood}

        # Default active state
        self.last_message = f"Damping ratio ζ = {zeta:.2f}. Monitoring dynamic tire contact and chassis heave."
        self.last_mood = "ready"
        self.message_cooldown = 2.0
        return {'message': self.last_message, 'mood': self.last_mood}


class TelemetryDashboard:
    """
    Manages live numerical metrics, rolling graph buffers,
    and physics explanations for the user interface.
    """

    def __init__(self, buffer_size: int = 300):
        self.buffer_size = buffer_size
        self.dampener = DrDampener()

        # Rolling buffers for graphs
        self.time_buffer = deque(maxlen=buffer_size)
        self.zc_buffer = deque(maxlen=buffer_size)
        self.zw_buffer = deque(maxlen=buffer_size)
        self.dzc_buffer = deque(maxlen=buffer_size)
        self.ddzc_buffer = deque(maxlen=buffer_size)
        self.road_buffer = deque(maxlen=buffer_size)
        self.spring_force_buffer = deque(maxlen=buffer_size)
        self.damper_force_buffer = deque(maxlen=buffer_size)

    def update(self, state: PhysicsState, params: VehicleParameters, dt: float) -> Dict[str, Any]:
        """
        Record a telemetry sample and return consolidated dashboard data.
        """
        t = state.time
        zc = state.z_c
        zw = state.z_w['FL']
        dzc = state.dz_c
        ddzc = state.ddz_c
        road_z = state.road_z['FL']
        sp_f = state.spring_force['FL']
        da_f = state.damper_force['FL']

        self.time_buffer.append(t)
        self.zc_buffer.append(zc)
        self.zw_buffer.append(zw)
        self.dzc_buffer.append(dzc)
        self.ddzc_buffer.append(ddzc)
        self.road_buffer.append(road_z)
        self.spring_force_buffer.append(sp_f)
        self.damper_force_buffer.append(da_f)

        qc = params.get_quarter_car_properties('FL')
        assistant = self.dampener.evaluate(state, params, dt)

        # Dynamic parameter effect description
        explanation = self.generate_parameter_insight(params, qc)

        return {
            'time_s': t,
            'speed_kmh': params.speed_kmh,
            'vehicle_displacement_mm': zc * 1000.0,
            'wheel_displacement_mm': zw * 1000.0,
            'suspension_compression_mm': state.susp_stroke['FL'] * 1000.0,
            'spring_force_N': sp_f,
            'damping_force_N': da_f,
            'total_susp_force_N': state.total_susp_force['FL'],
            'tire_force_N': state.tire_force['FL'],
            'natural_frequency_hz': qc['natural_freq_hz'],
            'damping_ratio_zeta': qc['damping_ratio_zeta'],
            'damping_regime': qc['damping_regime'],
            'vehicle_accel_mps2': ddzc,
            'vehicle_accel_g': ddzc / 9.80665,
            'pitch_deg': np.degrees(state.theta),
            'roll_deg': np.degrees(state.phi),
            'assistant_message': assistant['message'],
            'assistant_mood': assistant['mood'],
            'parameter_insight': explanation,
        }

    def get_graph_data(self) -> Dict[str, List[float]]:
        """Return history lists for chart rendering."""
        return {
            'time': list(self.time_buffer),
            'zc': [z * 1000.0 for z in self.zc_buffer],         # mm
            'zw': [z * 1000.0 for z in self.zw_buffer],         # mm
            'dzc': list(self.dzc_buffer),                       # m/s
            'ddzc': list(self.ddzc_buffer),                     # m/s^2
            'road': [r * 1000.0 for r in self.road_buffer],     # mm
            'spring_force': list(self.spring_force_buffer),     # N
            'damper_force': list(self.damper_force_buffer),     # N
        }

    @staticmethod
    def generate_parameter_insight(params: VehicleParameters, qc: Dict[str, Any]) -> str:
        """
        Dynamically explains the engineering impact of the current stiffness and damping settings.
        """
        k = qc['spring_stiffness_N_m']
        c = qc['damping_coeff_Ns_m']
        m = qc['sprung_mass_kg']
        fn = qc['natural_freq_hz']
        zeta = qc['damping_ratio_zeta']
        sag = qc['static_sag_mm']

        insight = (
            f"**Corner Sprung Mass:** {m:.1f} kg | **Stiffness:** {k:.0f} N/m | **Damping:** {c:.0f} N·s/m\n\n"
            f"• **Natural Frequency f_n = {fn:.2f} Hz:** "
        )

        if fn < 1.0:
            insight += "Low natural frequency (luxury/floaty feel). Long oscillation period; sensitive to low-speed body sway.\n"
        elif fn <= 1.5:
            insight += "Standard passenger car range (1.1 - 1.4 Hz). Balances ride comfort with adequate chassis motion control.\n"
        else:
            insight += "High natural frequency (sports car / track setup). Very sharp transient response, but transmits harsher road vibration.\n"

        insight += f"• **Damping Ratio ζ = {zeta:.2f} ({qc['damping_regime']}):** "
        if zeta < 0.4:
            insight += "Underdamped. Rapid initial compression absorption, but the chassis will bounce repeatedly before stabilizing.\n"
        elif 0.6 <= zeta <= 0.8:
            insight += "Automotive optimum (~0.7). Settles within 1–1.5 oscillation cycles without feeling excessively rigid.\n"
        elif zeta < 1.0:
            insight += "Firm damping. Highly controlled body motions, minimal rebound overshoot, but firmer ride over sharp bumps.\n"
        elif np.isclose(zeta, 1.0, atol=0.05):
            insight += "Critically damped (ζ = 1.0). Returns to static equilibrium in the shortest possible time without any overshoot.\n"
        else:
            insight += "Overdamped (ζ > 1.0). Sluggish suspension stroke; forces are transmitted directly into the passenger cabin.\n"

        insight += f"• **Static Suspension Sag:** {sag:.1f} mm under gravitational preload."
        return insight

    @staticmethod
    def get_educational_curriculum() -> Dict[str, Any]:
        """
        Structured educational physics lessons for the dedicated explanation panel.
        """
        return {
            "title": "Automotive Suspension Physics: Mass-Spring-Damper Systems",
            "sections": [
                {
                    "title": "1. What is Simple Harmonic Motion (SHM)?",
                    "content": (
                        "When a vehicle wheel encounters a disturbance, the suspension spring is compressed or extended. "
                        "According to Hooke's Law, the spring exerts a restoring force directly proportional to displacement:\n\n"
                        "$$F_s = -k \\cdot x$$\n\n"
                        "Without damping, this restoring force accelerates the vehicle mass back toward equilibrium, creating "
                        "perpetual oscillation known as Simple Harmonic Motion (SHM) governed by $m x'' + k x = 0$."
                    )
                },
                {
                    "title": "2. Damping: Dissipating Kinetic Energy",
                    "content": (
                        "A spring stores energy, but does not dissipate it. Shock absorbers (hydraulic dampers) introduce a "
                        "viscous drag force proportional to relative velocity:\n\n"
                        "$$F_d = -c \\cdot \\dot{x}$$\n\n"
                        "Hydraulic fluid forced through calibrated valving orifices converts kinetic bounce energy into thermal "
                        "energy (heat), damping out oscillations and keeping tires glued to the road surface."
                    )
                },
                {
                    "title": "3. Natural Frequency and Critical Damping",
                    "content": (
                        "The undamped natural frequency determines how quickly the vehicle naturally bounces:\n\n"
                        "$$\\omega_n = \\sqrt{\\frac{k}{m}} \\quad [\\text{rad/s}], \\qquad f_n = \\frac{\\omega_n}{2\\pi} \\quad [\\text{Hz}]$$\n\n"
                        "Critical damping $c_{\\text{crit}} = 2\\sqrt{m k}$ is the threshold separating oscillatory from non-oscillatory decay. "
                        "The non-dimensional damping ratio is:\n\n"
                        "$$\\zeta = \\frac{c}{c_{\\text{crit}}} = \\frac{c}{2\\sqrt{m k}}$$"
                    )
                },
                {
                    "title": "4. The Three Damping Regimes",
                    "content": (
                        "• **Underdamped ($\\zeta < 1$):** System oscillates with exponentially decaying envelope $e^{-\\zeta \\omega_n t}$. "
                        "Damped frequency is $\\omega_d = \\omega_n \\sqrt{1 - \\zeta^2}$. Most cars use $\\zeta \\approx 0.6 - 0.7$ for optimal comfort.\n"
                        "• **Critically Damped ($\\zeta = 1$):** Fastest possible return to equilibrium without overshoot.\n"
                        "• **Overdamped ($\\zeta > 1$):** Sluggish non-oscillatory return. High damper resistance makes the ride harsh."
                    )
                },
                {
                    "title": "5. Forced Oscillation and Resonance",
                    "content": (
                        "When driving over periodic road waves (wavelength $\\lambda$) at speed $v$, the excitation frequency is:\n\n"
                        "$$f_{\\text{road}} = \\frac{v}{\\lambda}$$\n\n"
                        "When $f_{\\text{road}} \\to f_n$ (frequency ratio $r = f_{\\text{road}} / f_n \\approx 1$), **RESONANCE** occurs! "
                        "Transmissibility spikes dramatically, magnifying chassis bounce. Proper damping prevents dangerous resonance amplification."
                    )
                },
                {
                    "title": "6. The Fundamental Automotive Trade-Off",
                    "content": (
                        "Automotive engineers must balance two competing objectives:\n\n"
                        "1. **Ride Comfort (Isolation):** Requires softer springs and lower damping to minimize vertical acceleration transmitted to passengers.\n"
                        "2. **Road Holding (Safety & Handling):** Requires stiffer springs and higher damping to prevent dynamic tire force variations ($F_{\\text{tire}} \\to 0$), "
                        "ensuring constant contact for steering and braking."
                    )
                }
            ]
        }
