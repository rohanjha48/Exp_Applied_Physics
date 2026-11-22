"""
physics_engine.py
=================
Full 7-Degree-of-Freedom (7-DOF) vehicle suspension dynamics engine
with 4th-order Runge-Kutta (RK4) numerical integration.

Equations of Motion:
  Sprung mass heave (z_c):   M_s * z_c''  = sum(F_susp,i)
  Sprung mass pitch (theta): I_yy * theta'' = a * (F_FL + F_FR) - b * (F_RL + F_RR)
  Sprung mass roll (phi):    I_xx * phi''   = (t_f/2)*(F_FL - F_FR) + (t_r/2)*(F_RL - F_RR)
  Unsprung masses (z_w,i):   m_u,i * z_w,i'' = F_tire,i - F_susp,i

Tire unilateral contact law: F_tire = max(0, k_t*(z_road - z_w) + c_t*(z_road' - z_w'))
"""

from typing import Dict, Any, Tuple, List, Optional
import numpy as np

from vehicle_model import VehicleParameters
from road_model import RoadProfile


class PhysicsState:
    """State vector container for 7-DOF multi-body vehicle system."""

    def __init__(self):
        # Time [s] and vehicle longitudinal position along road [m]
        self.time: float = 0.0
        self.x_position: float = 0.0

        # Sprung mass (chassis CG) degrees of freedom
        self.z_c: float = 0.0        # Heave displacement [m]
        self.dz_c: float = 0.0       # Heave velocity [m/s]
        self.ddz_c: float = 0.0      # Heave acceleration [m/s^2]

        self.theta: float = 0.0      # Pitch angle [rad] (+nose up)
        self.dtheta: float = 0.0     # Pitch rate [rad/s]
        self.ddtheta: float = 0.0    # Pitch angular acceleration [rad/s^2]

        self.phi: float = 0.0        # Roll angle [rad] (+roll left)
        self.dphi: float = 0.0       # Roll rate [rad/s]
        self.ddphi: float = 0.0      # Roll angular acceleration [rad/s^2]

        # 4 Unsprung wheel masses: displacement [m] and velocity [m/s]
        self.z_w: Dict[str, float] = {'FL': 0.0, 'FR': 0.0, 'RL': 0.0, 'RR': 0.0}
        self.dz_w: Dict[str, float] = {'FL': 0.0, 'FR': 0.0, 'RL': 0.0, 'RR': 0.0}
        self.ddz_w: Dict[str, float] = {'FL': 0.0, 'FR': 0.0, 'RL': 0.0, 'RR': 0.0}

        # Corner suspension forces and displacements
        self.susp_stroke: Dict[str, float] = {'FL': 0.0, 'FR': 0.0, 'RL': 0.0, 'RR': 0.0}
        self.spring_force: Dict[str, float] = {'FL': 0.0, 'FR': 0.0, 'RL': 0.0, 'RR': 0.0}
        self.damper_force: Dict[str, float] = {'FL': 0.0, 'FR': 0.0, 'RL': 0.0, 'RR': 0.0}
        self.total_susp_force: Dict[str, float] = {'FL': 0.0, 'FR': 0.0, 'RL': 0.0, 'RR': 0.0}
        self.tire_force: Dict[str, float] = {'FL': 0.0, 'FR': 0.0, 'RL': 0.0, 'RR': 0.0}

        # Road surface input at each wheel contact patch
        self.road_z: Dict[str, float] = {'FL': 0.0, 'FR': 0.0, 'RL': 0.0, 'RR': 0.0}
        self.road_dz: Dict[str, float] = {'FL': 0.0, 'FR': 0.0, 'RL': 0.0, 'RR': 0.0}

        # Wheel rotation angle [rad]
        self.wheel_angle: float = 0.0

    def to_array(self) -> np.ndarray:
        """Pack state into 14-element numpy array for ODE integration."""
        return np.array([
            self.z_c, self.dz_c,
            self.theta, self.dtheta,
            self.phi, self.dphi,
            self.z_w['FL'], self.dz_w['FL'],
            self.z_w['FR'], self.dz_w['FR'],
            self.z_w['RL'], self.dz_w['RL'],
            self.z_w['RR'], self.dz_w['RR'],
        ], dtype=np.float64)

    def from_array(self, arr: np.ndarray) -> None:
        """Unpack state from 14-element numpy array."""
        self.z_c, self.dz_c = arr[0], arr[1]
        self.theta, self.dtheta = arr[2], arr[3]
        self.phi, self.dphi = arr[4], arr[5]
        self.z_w['FL'], self.dz_w['FL'] = arr[6], arr[7]
        self.z_w['FR'], self.dz_w['FR'] = arr[8], arr[9]
        self.z_w['RL'], self.dz_w['RL'] = arr[10], arr[11]
        self.z_w['RR'], self.dz_w['RR'] = arr[12], arr[13]

    def copy(self) -> 'PhysicsState':
        cp = PhysicsState()
        cp.time = self.time
        cp.x_position = self.x_position
        cp.z_c = self.z_c
        cp.dz_c = self.dz_c
        cp.ddz_c = self.ddz_c
        cp.theta = self.theta
        cp.dtheta = self.dtheta
        cp.ddtheta = self.ddtheta
        cp.phi = self.phi
        cp.dphi = self.dphi
        cp.ddphi = self.ddphi
        cp.z_w = dict(self.z_w)
        cp.dz_w = dict(self.dz_w)
        cp.ddz_w = dict(self.ddz_w)
        cp.susp_stroke = dict(self.susp_stroke)
        cp.spring_force = dict(self.spring_force)
        cp.damper_force = dict(self.damper_force)
        cp.total_susp_force = dict(self.total_susp_force)
        cp.tire_force = dict(self.tire_force)
        cp.road_z = dict(self.road_z)
        cp.road_dz = dict(self.road_dz)
        cp.wheel_angle = self.wheel_angle
        return cp


class PhysicsEngine:
    """
    High-fidelity multi-body 7-DOF vehicle dynamics engine.
    Uses 4th-order Runge-Kutta with subcycling for rock-solid stability.
    """

    def __init__(self, vehicle_params: Optional[VehicleParameters] = None, road_profile: Optional[RoadProfile] = None):
        self.params = vehicle_params if vehicle_params is not None else VehicleParameters()
        self.road = road_profile
        self.state = PhysicsState()
        self.is_paused: bool = False
        self.time_scale: float = 1.0  # 1.0 = real time, 0.5 = slow motion

        # Anti-roll bar stiffness [N*m / rad]
        self.arb_stiffness_front: float = 8500.0
        self.arb_stiffness_rear: float = 4500.0

    def reset(self) -> None:
        """Reset state to static equilibrium at x = 0."""
        self.state = PhysicsState()

    def set_road(self, road: RoadProfile) -> None:
        self.road = road

    def compute_chassis_corner_kinematics(self, z_c: float, dz_c: float,
                                           theta: float, dtheta: float,
                                           phi: float, dphi: float) -> Tuple[Dict[str, float], Dict[str, float]]:
        """
        Calculate vertical position z_c,i and velocity dz_c,i of chassis mounting points.
        Sign conventions:
          FL: (+a, +tf/2) -> z_c + a*theta + (tf/2)*phi
          FR: (+a, -tf/2) -> z_c + a*theta - (tf/2)*phi
          RL: (-b, +tr/2) -> z_c - b*theta + (tr/2)*phi
          RR: (-b, -tr/2) -> z_c - b*theta - (tr/2)*phi
        """
        a, b = self.params.cg_to_front, self.params.cg_to_rear
        tf2, tr2 = self.params.front_track / 2.0, self.params.rear_track / 2.0

        z_corners = {
            'FL': z_c + a * theta + tf2 * phi,
            'FR': z_c + a * theta - tf2 * phi,
            'RL': z_c - b * theta + tr2 * phi,
            'RR': z_c - b * theta - tr2 * phi,
        }
        dz_corners = {
            'FL': dz_c + a * dtheta + tf2 * dphi,
            'FR': dz_c + a * dtheta - tf2 * dphi,
            'RL': dz_c - b * dtheta + tr2 * dphi,
            'RR': dz_c - b * dtheta - tr2 * dphi,
        }
        return z_corners, dz_corners

    def compute_derivatives(self, state_arr: np.ndarray, t: float, x_pos: float) -> Tuple[np.ndarray, Dict[str, Any]]:
        """
        Evaluates state derivatives dX/dt for 7-DOF equations of motion.
        Returns:
          dX_dt: 14-element derivative vector
          diagnostics: forces and telemetry dictionary
        """
        p = self.params
        v = p.speed_ms

        z_c, dz_c = state_arr[0], state_arr[1]
        theta, dtheta = state_arr[2], state_arr[3]
        phi, dphi = state_arr[4], state_arr[5]

        z_w = {'FL': state_arr[6], 'FR': state_arr[8], 'RL': state_arr[10], 'RR': state_arr[12]}
        dz_w = {'FL': state_arr[7], 'FR': state_arr[9], 'RL': state_arr[11], 'RR': state_arr[13]}

        # Chassis corner positions and velocities
        z_c_corner, dz_c_corner = self.compute_chassis_corner_kinematics(z_c, dz_c, theta, dtheta, phi, dphi)

        # Query road profile at each tire contact patch
        # Longitudinal position of each axle:
        x_fl = x_pos + p.cg_to_front
        x_fr = x_pos + p.cg_to_front
        x_rl = x_pos - p.cg_to_rear
        x_rr = x_pos - p.cg_to_rear

        y_fl = p.front_track / 2.0
        y_fr = -p.front_track / 2.0
        y_rl = p.rear_track / 2.0
        y_rr = -p.rear_track / 2.0

        if self.road is not None:
            road_data = self.road.get_track_elevations(
                x_fl, y_fl, x_fr, y_fr, x_rl, y_rl, x_rr, y_rr, v
            )
        else:
            road_data = {
                'FL': (0.0, 0.0), 'FR': (0.0, 0.0), 'RL': (0.0, 0.0), 'RR': (0.0, 0.0)
            }

        # Calculate suspension and tire forces
        F_spring = {}
        F_damper = {}
        F_bumpstop = {}
        F_susp = {}
        F_tire = {}
        stroke = {}

        for corner in ['FL', 'FR', 'RL', 'RR']:
            is_front = 'F' in corner
            k_s = p.front_spring_stiffness if is_front else p.rear_spring_stiffness
            c_s = p.front_damping_coeff if is_front else p.rear_damping_coeff

            # Relative suspension deflection (positive = compression, wheel moves towards chassis)
            delta_z = z_w[corner] - z_c_corner[corner]
            delta_dz = dz_w[corner] - dz_c_corner[corner]
            stroke[corner] = delta_z

            # Linear spring force
            F_sp = k_s * delta_z
            F_spring[corner] = F_sp

            # Linear viscous damper force
            F_d = c_s * delta_dz
            F_damper[corner] = F_d

            # Non-linear bump stop engagement
            F_bs = 0.0
            if delta_z > p.bump_stop_clearance:
                excess = delta_z - p.bump_stop_clearance
                F_bs = p.bump_stop_stiffness * excess + (p.bump_stop_stiffness * 2.0) * (excess ** 2)
            elif delta_z < -p.bump_stop_clearance:
                excess = -delta_z - p.bump_stop_clearance
                F_bs = -(p.bump_stop_stiffness * excess + (p.bump_stop_stiffness * 2.0) * (excess ** 2))
            F_bumpstop[corner] = F_bs

            # Total upward suspension force on chassis
            F_susp[corner] = F_sp + F_d + F_bs

        # Anti-roll bar torque / forces
        # Front ARB: resists difference between left and right wheel displacement
        arb_f = (self.arb_stiffness_front / (p.front_track ** 2)) * (z_w['FL'] - z_w['FR'])
        F_susp['FL'] += arb_f
        F_susp['FR'] -= arb_f

        # Rear ARB
        arb_r = (self.arb_stiffness_rear / (p.rear_track ** 2)) * (z_w['RL'] - z_w['RR'])
        F_susp['RL'] += arb_r
        F_susp['RR'] -= arb_r

        # Equations of motion for sprung mass
        # 1. Heave: M_s * ddz_c = sum(F_susp)
        sum_F_susp = F_susp['FL'] + F_susp['FR'] + F_susp['RL'] + F_susp['RR']
        ddz_c = sum_F_susp / p.sprung_mass

        # 2. Pitch: I_yy * ddtheta = a * (F_FL + F_FR) - b * (F_RL + F_RR)
        pitch_moment = p.cg_to_front * (F_susp['FL'] + F_susp['FR']) - p.cg_to_rear * (F_susp['RL'] + F_susp['RR'])
        ddtheta = pitch_moment / p.pitch_inertia

        # 3. Roll: I_xx * ddphi = (tf/2)*(F_FL - F_FR) + (tr/2)*(F_RL - F_RR)
        roll_moment = (p.front_track / 2.0) * (F_susp['FL'] - F_susp['FR']) + (p.rear_track / 2.0) * (F_susp['RL'] - F_susp['RR'])
        ddphi = roll_moment / p.roll_inertia

        # Corner static preloads (weight supported by each tire at rest)
        corner_ms = p.get_corner_sprung_masses()
        g = 9.80665
        ddz_w_dict = {}

        # Calculate tire forces (unilateral contact with ground)
        for corner in ['FL', 'FR', 'RL', 'RR']:
            z_r, dz_r = road_data[corner]
            delta_tire = z_r - z_w[corner]
            delta_dtire = dz_r - dz_w[corner]

            m_u = p.front_unsprung_mass if 'F' in corner else p.rear_unsprung_mass
            F0 = (corner_ms[corner] + m_u) * g

            # Dynamic tire perturbation force
            delta_Ft = p.tire_stiffness * delta_tire + p.tire_damping * delta_dtire
            # Total normal contact force
            F_tot = F0 + delta_Ft

            if F_tot <= 0.0:
                # Wheel is airborne / separated from ground
                delta_Ft = -F0
                F_tire[corner] = 0.0
            else:
                F_tire[corner] = F_tot

            # Net dynamic upward force on unsprung mass from tire
            F_tire_net = delta_Ft
            ddz_w_dict[corner] = (F_tire_net - F_susp[corner]) / m_u

        # Assemble derivative vector: [dz_c, ddz_c, dtheta, ddtheta, dphi, ddphi, dz_w1, ddz_w1, ...]
        dX_dt = np.array([
            dz_c, ddz_c,
            dtheta, ddtheta,
            dphi, ddphi,
            dz_w['FL'], ddz_w_dict['FL'],
            dz_w['FR'], ddz_w_dict['FR'],
            dz_w['RL'], ddz_w_dict['RL'],
            dz_w['RR'], ddz_w_dict['RR'],
        ], dtype=np.float64)

        diagnostics = {
            'ddz_c': ddz_c,
            'ddtheta': ddtheta,
            'ddphi': ddphi,
            'ddz_w': ddz_w_dict,
            'F_spring': F_spring,
            'F_damper': F_damper,
            'F_susp': F_susp,
            'F_tire': F_tire,
            'stroke': stroke,
            'road_data': road_data,
        }
        return dX_dt, diagnostics

    def step(self, dt: float) -> PhysicsState:
        """
        Advance simulation by dt seconds using 4th-Order Runge-Kutta.
        Splits dt into subcycles (substep <= 0.001 s) to ensure numerical stability.
        """
        if self.is_paused:
            return self.state

        effective_dt = dt * self.time_scale
        max_substep = 0.001  # 1 ms substepping
        n_substeps = max(1, int(np.ceil(effective_dt / max_substep)))
        h = effective_dt / n_substeps

        v = self.params.speed_ms
        current_state = self.state.to_array()
        t = self.state.time
        x_pos = self.state.x_position

        diag = {}
        for _ in range(n_substeps):
            # RK4 stages
            k1, d1 = self.compute_derivatives(current_state, t, x_pos)
            k2, _ = self.compute_derivatives(current_state + 0.5 * h * k1, t + 0.5 * h, x_pos + 0.5 * h * v)
            k3, _ = self.compute_derivatives(current_state + 0.5 * h * k2, t + 0.5 * h, x_pos + 0.5 * h * v)
            k4, _ = self.compute_derivatives(current_state + h * k3, t + h, x_pos + h * v)

            current_state = current_state + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
            t += h
            x_pos += h * v
            diag = d1

        # Update physical state object
        self.state.from_array(current_state)
        self.state.time = t
        self.state.x_position = x_pos

        if diag:
            self.state.ddz_c = float(diag['ddz_c'])
            self.state.ddtheta = float(diag['ddtheta'])
            self.state.ddphi = float(diag['ddphi'])
            self.state.ddz_w = diag['ddz_w']
            self.state.spring_force = diag['F_spring']
            self.state.damper_force = diag['F_damper']
            self.state.total_susp_force = diag['F_susp']
            self.state.tire_force = diag['F_tire']
            self.state.susp_stroke = diag['stroke']

            # Extract road elevation and velocity
            for corner in ['FL', 'FR', 'RL', 'RR']:
                self.state.road_z[corner] = diag['road_data'][corner][0]
                self.state.road_dz[corner] = diag['road_data'][corner][1]

        # Wheel rotation based on distance travelled: theta = x / R
        self.state.wheel_angle = (x_pos / self.params.wheel_radius) % (2.0 * np.pi)

        return self.state

    def get_resonant_speed_kmh(self, wavelength_m: float = 3.2) -> float:
        """
        Calculate vehicle forward speed (km/h) that triggers primary bounce resonance
        over a periodic road profile of given wavelength:
        f_road = v / lambda == f_n  ==>  v = lambda * f_n
        """
        qc = self.params.get_quarter_car_properties('FL')
        fn = qc['natural_freq_hz']
        v_ms = wavelength_m * fn
        return float(v_ms * 3.6)
