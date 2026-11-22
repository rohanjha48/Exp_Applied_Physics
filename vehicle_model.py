

from dataclasses import dataclass, field
import numpy as np


@dataclass
class VehicleParameters:
    """Comprehensive parameters for passenger car multi-body suspension dynamics."""

    # Sprung mass (chassis + body + occupants) [kg]
    sprung_mass: float = 1400.0

    # Moments of inertia [kg * m^2]
    # Approximated from radius of gyration or specified directly
    pitch_inertia: float = 2100.0  # I_yy (around pitch axis)
    roll_inertia: float = 650.0    # I_xx (around roll axis)
    yaw_inertia: float = 2400.0    # I_zz (around yaw axis)

    # Geometry [m]
    wheelbase: float = 2.70        # L = a + b
    cg_to_front: float = 1.25      # a: distance from CG to front axle
    cg_to_rear: float = 1.45       # b: distance from CG to rear axle (L - a)
    front_track: float = 1.55      # t_f: lateral distance between front wheels
    rear_track: float = 1.55       # t_r: lateral distance between rear wheels
    cg_height: float = 0.52        # Height of center of gravity above ground [m]

    # Suspension stiffness [N/m] (Nominal per corner)
    front_spring_stiffness: float = 28000.0
    rear_spring_stiffness: float = 26000.0

    # Suspension damping [N*s/m] (Nominal per corner)
    front_damping_coeff: float = 2200.0
    rear_damping_coeff: float = 2100.0

    # Non-linear bump stops
    bump_stop_clearance: float = 0.08    # [m] free travel before bump stop engagement
    bump_stop_stiffness: float = 180000.0 # [N/m] progressive stiffness

    # Unsprung mass (wheel, tire, hub, upright, brake, part of control arm) [kg]
    front_unsprung_mass: float = 45.0
    rear_unsprung_mass: float = 42.0

    # Tire compliance [N/m] and tire damping [N*s/m]
    tire_stiffness: float = 200000.0     # Vertical spring stiffness of tire carcass
    tire_damping: float = 150.0          # Vertical damping of pneumatic tire

    # Dimensions for 3D visualization [m]
    body_length: float = 4.45
    body_width: float = 1.82
    body_height: float = 1.38
    wheel_radius: float = 0.33
    rim_radius: float = 0.22
    tire_width: float = 0.225
    static_ground_clearance: float = 0.16

    # Operational parameters
    speed_kmh: float = 50.0  # Vehicle forward speed [km/h]

    @property
    def speed_ms(self) -> float:
        """Vehicle forward speed in m/s."""
        return self.speed_kmh / 3.6

    @speed_ms.setter
    def speed_ms(self, value_ms: float) -> None:
        self.speed_kmh = value_ms * 3.6

    def get_corner_positions(self) -> dict:
        """
        Coordinates (x_longitudinal, y_lateral) of the 4 wheel contact patches relative to CG.
        Sign convention: +x forward, +y left (ISO vehicle dynamics coordinate system).
        """
        return {
            'FL': (self.cg_to_front, self.front_track / 2.0),
            'FR': (self.cg_to_front, -self.front_track / 2.0),
            'RL': (-self.cg_to_rear, self.rear_track / 2.0),
            'RR': (-self.cg_to_rear, -self.rear_track / 2.0),
        }

    def get_corner_sprung_masses(self) -> dict:
        """
        Static weight distribution per corner based on longitudinal and lateral CG position.
        Assumes symmetric lateral weight distribution.
        """
        # Static front-rear load transfer:
        # M_front = M_s * (b / L), M_rear = M_s * (a / L)
        m_front_total = self.sprung_mass * (self.cg_to_rear / self.wheelbase)
        m_rear_total = self.sprung_mass * (self.cg_to_front / self.wheelbase)
        return {
            'FL': m_front_total / 2.0,
            'FR': m_front_total / 2.0,
            'RL': m_rear_total / 2.0,
            'RR': m_rear_total / 2.0,
        }

    def get_quarter_car_properties(self, corner: str = 'FL') -> dict:
        """
        Calculate analytical 1-DOF / 2-DOF single corner metrics:
        natural frequency, critical damping, damping ratio, damped frequency.
        """
        m_s = self.get_corner_sprung_masses()[corner]
        k_s = self.front_spring_stiffness if 'F' in corner else self.rear_spring_stiffness
        c_s = self.front_damping_coeff if 'F' in corner else self.rear_damping_coeff

        # Natural angular frequency: omega_n = sqrt(k / m)
        omega_n = np.sqrt(k_s / m_s)
        fn_hz = omega_n / (2.0 * np.pi)

        # Critical damping: c_crit = 2 * sqrt(m * k)
        c_crit = 2.0 * np.sqrt(m_s * k_s)

        # Damping ratio: zeta = c / c_crit
        zeta = c_s / c_crit

        # Damped natural frequency
        if zeta < 1.0:
            omega_d = omega_n * np.sqrt(1.0 - zeta**2)
            regime = "Underdamped"
        elif np.isclose(zeta, 1.0, atol=0.01):
            omega_d = 0.0
            regime = "Critically Damped"
        else:
            omega_d = 0.0
            regime = "Overdamped"

        fn_damped_hz = omega_d / (2.0 * np.pi)

        # Static suspension sag under gravity (g = 9.81 m/s^2)
        static_deflection = (m_s * 9.80665) / k_s

        return {
            'corner': corner,
            'sprung_mass_kg': m_s,
            'spring_stiffness_N_m': k_s,
            'damping_coeff_Ns_m': c_s,
            'omega_n_rad_s': omega_n,
            'natural_freq_hz': fn_hz,
            'critical_damping_Ns_m': c_crit,
            'damping_ratio_zeta': zeta,
            'damped_natural_freq_hz': fn_damped_hz,
            'damping_regime': regime,
            'static_sag_mm': static_deflection * 1000.0,
        }

    def transmissibility(self, freq_ratio: float, zeta: float) -> float:
        """
        Calculate displacement transmissibility ratio T = |X / Y|
        for harmonic road excitation at frequency ratio r = omega_road / omega_n.
        T = sqrt( (1 + (2*zeta*r)^2) / ((1 - r^2)^2 + (2*zeta*r)^2) )
        """
        r = freq_ratio
        numerator = 1.0 + (2.0 * zeta * r) ** 2
        denominator = (1.0 - r**2) ** 2 + (2.0 * zeta * r) ** 2
        return float(np.sqrt(numerator / denominator))
