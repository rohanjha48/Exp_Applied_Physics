"""
road_model.py
=============
Provides 8 distinct road elevation profiles and spatial excitation models
for the 3D Vehicle Suspension Simulator.

Supports ISO 8608 road roughness PSD, speed bumps, potholes, sinusoidal
washboards, and asymmetric (unilateral) road hazards.
"""

from dataclasses import dataclass
from typing import Dict, Any, Tuple
import numpy as np


@dataclass
class RoadPresetInfo:
    id: str
    name: str
    description: str
    educational_tip: str
    typical_amplitude_m: float
    wavelength_m: float = 0.0


class RoadProfile:
    """Base class for all road surfaces."""

    def __init__(self, name: str, preset_id: str, bump_height_scale: float = 1.0, roughness_scale: float = 1.0):
        self.name = name
        self.preset_id = preset_id
        self.bump_height_scale = bump_height_scale
        self.roughness_scale = roughness_scale

    def elevation(self, x: float, y: float) -> float:
        """Return road surface elevation z [m] at longitudinal coordinate x [m] and lateral y [m]."""
        raise NotImplementedError

    def elevation_derivative(self, x: float, y: float, v_ms: float) -> float:
        """
        Return vertical velocity of road input dz/dt [m/s]
        dz/dt = v * (dz/dx)
        Approximated via centered finite difference for arbitrary profiles.
        """
        dx = 0.005  # 5 mm step
        z_plus = self.elevation(x + dx, y)
        z_minus = self.elevation(x - dx, y)
        dz_dx = (z_plus - z_minus) / (2.0 * dx)
        return float(v_ms * dz_dx)

    def get_track_elevations(self, x_fl: float, y_fl: float,
                              x_fr: float, y_fr: float,
                              x_rl: float, y_rl: float,
                              x_rr: float, y_rr: float,
                              v_ms: float) -> Dict[str, Tuple[float, float]]:
        """
        Returns elevation and vertical velocity for all 4 tire contact patches:
        {'FL': (z, dz_dt), 'FR': (z, dz_dt), 'RL': (z, dz_dt), 'RR': (z, dz_dt)}
        """
        return {
            'FL': (self.elevation(x_fl, y_fl), self.elevation_derivative(x_fl, y_fl, v_ms)),
            'FR': (self.elevation(x_fr, y_fr), self.elevation_derivative(x_fr, y_fr, v_ms)),
            'RL': (self.elevation(x_rl, y_rl), self.elevation_derivative(x_rl, y_rl, v_ms)),
            'RR': (self.elevation(x_rr, y_rr), self.elevation_derivative(x_rr, y_rr, v_ms)),
        }


class SmoothHighwayRoad(RoadProfile):
    """ISO 8608 Class A very smooth highway surface."""

    def __init__(self, bump_height_scale: float = 1.0, roughness_scale: float = 1.0):
        super().__init__("Smooth Highway", "smooth_highway", bump_height_scale, roughness_scale)

    def elevation(self, x: float, y: float) -> float:
        # Subtle realistic millimeter road texture
        base = 0.0012 * np.sin(2.0 * np.pi * x / 0.8) + 0.0008 * np.cos(2.0 * np.pi * x / 0.25)
        return float(base * self.roughness_scale)


class SmallSpeedBreakersRoad(RoadProfile):
    """Periodic small speed humps (height ~4 cm, width 1.2 m, spaced every 20 m)."""

    def __init__(self, bump_height_scale: float = 1.0, roughness_scale: float = 1.0):
        super().__init__("Small Speed Breakers", "small_breakers", bump_height_scale, roughness_scale)
        self.spacing = 20.0
        self.width = 1.2
        self.height = 0.045 * bump_height_scale

    def elevation(self, x: float, y: float) -> float:
        x_mod = x % self.spacing
        h = 0.0
        if 2.0 <= x_mod <= (2.0 + self.width):
            u = (x_mod - 2.0) / self.width
            h = self.height * np.sin(np.pi * u) ** 2
        # Add slight texture
        texture = 0.001 * np.sin(x * 12.0) * self.roughness_scale
        return float(h + texture)


class LargeSpeedBreakersRoad(RoadProfile):
    """Substantial speed table / raised crossing (height ~8.5 cm, width 2.5 m, spaced every 28 m)."""

    def __init__(self, bump_height_scale: float = 1.0, roughness_scale: float = 1.0):
        super().__init__("Large Speed Breakers", "large_breakers", bump_height_scale, roughness_scale)
        self.spacing = 28.0
        self.width = 2.4
        self.height = 0.085 * bump_height_scale

    def elevation(self, x: float, y: float) -> float:
        x_mod = x % self.spacing
        h = 0.0
        start_x = 3.0
        if start_x <= x_mod <= (start_x + self.width):
            u = (x_mod - start_x) / self.width
            h = self.height * np.sin(np.pi * u) ** 2
        texture = 0.001 * np.sin(x * 15.0) * self.roughness_scale
        return float(h + texture)


class PotholeRoad(RoadProfile):
    """Pothole road with sharp negative dips (depth ~6 cm, width 0.75 m)."""

    def __init__(self, bump_height_scale: float = 1.0, roughness_scale: float = 1.0):
        super().__init__("Pothole Road", "potholes", bump_height_scale, roughness_scale)
        self.spacing = 18.0
        self.depth = 0.060 * bump_height_scale
        self.width = 0.85

    def elevation(self, x: float, y: float) -> float:
        x_mod = x % self.spacing
        h = 0.0
        # Pothole 1 on center-left
        if 2.5 <= x_mod <= (2.5 + self.width) and y > -0.2:
            u = (x_mod - 2.5) / self.width
            h = -self.depth * np.sin(np.pi * u) ** 2
        # Pothole 2 on right track at offset distance
        elif 9.0 <= x_mod <= (9.0 + self.width * 0.9) and y < 0.2:
            u = (x_mod - 9.0) / (self.width * 0.9)
            h = -self.depth * 0.85 * np.sin(np.pi * u) ** 2

        texture = 0.002 * (np.sin(x * 7.0) + 0.5 * np.cos(x * 19.0)) * self.roughness_scale
        return float(h + texture)


class WavyRoad(RoadProfile):
    """
    Continuous sinusoidal washboard road profile.
    Used for resonance demonstration!
    z(x) = h0 * sin(2 * pi * x / lambda)
    """

    def __init__(self, bump_height_scale: float = 1.0, roughness_scale: float = 1.0, wavelength: float = 3.2):
        super().__init__("Wavy Road (Resonance)", "wavy_road", bump_height_scale, roughness_scale)
        self.wavelength = wavelength
        self.amplitude = 0.038 * bump_height_scale

    def elevation(self, x: float, y: float) -> float:
        return float(self.amplitude * np.sin(2.0 * np.pi * x / self.wavelength))


class RandomUnevenRoad(RoadProfile):
    """
    ISO 8608 Class C / D random road surface synthesized via multi-harmonic superposition.
    Recreates real-world asphalt degradation and rutting.
    """

    def __init__(self, bump_height_scale: float = 1.0, roughness_scale: float = 1.0):
        super().__init__("Random Uneven Road", "random_uneven", bump_height_scale, roughness_scale)
        # Precompute static random harmonic components for deterministic, smooth continuous profile
        np.random.seed(42)
        self.n_harmonics = 32
        # Spatial frequencies between 0.05 cycle/m (20m wave) and 5.0 cycle/m (0.2m wave)
        self.spatial_freqs = np.geomspace(0.05, 4.0, self.n_harmonics)
        # ISO PSD power law: amplitude ~ 1 / n
        amplitudes = 0.018 * (self.spatial_freqs / 0.1) ** (-1.1)
        # Add random phases
        self.phases_left = np.random.uniform(0, 2.0 * np.pi, self.n_harmonics)
        self.phases_right = np.random.uniform(0, 2.0 * np.pi, self.n_harmonics)
        self.amplitudes = amplitudes * bump_height_scale * roughness_scale

    def elevation(self, x: float, y: float) -> float:
        # Interpolate between left and right track phases based on lateral position y
        # y > 0 is left, y < 0 is right
        weight_left = np.clip(0.5 + y / 1.6, 0.0, 1.0)
        z_left = np.sum(self.amplitudes * np.sin(2.0 * np.pi * self.spatial_freqs * x + self.phases_left))
        z_right = np.sum(self.amplitudes * np.sin(2.0 * np.pi * self.spatial_freqs * x + self.phases_right))
        z = weight_left * z_left + (1.0 - weight_left) * z_right
        return float(np.clip(z * 0.35, -0.07, 0.07))


class LeftSideBumpRoad(RoadProfile):
    """
    Asymmetric single-sided bump on LEFT wheel track only.
    Demonstrates chassis roll and independent suspension isolation.
    """

    def __init__(self, bump_height_scale: float = 1.0, roughness_scale: float = 1.0):
        super().__init__("Left-Side Bump", "left_bump", bump_height_scale, roughness_scale)
        self.spacing = 24.0
        self.width = 1.8
        self.height = 0.075 * bump_height_scale

    def elevation(self, x: float, y: float) -> float:
        # Left track only: y > 0.1 m
        if y < 0.1:
            return 0.0
        x_mod = x % self.spacing
        if 3.0 <= x_mod <= (3.0 + self.width):
            u = (x_mod - 3.0) / self.width
            return float(self.height * np.sin(np.pi * u) ** 2)
        return 0.0


class RightSideBumpRoad(RoadProfile):
    """
    Asymmetric single-sided bump on RIGHT wheel track only.
    Demonstrates chassis roll and independent suspension isolation.
    """

    def __init__(self, bump_height_scale: float = 1.0, roughness_scale: float = 1.0):
        super().__init__("Right-Side Bump", "right_bump", bump_height_scale, roughness_scale)
        self.spacing = 24.0
        self.width = 1.8
        self.height = 0.075 * bump_height_scale

    def elevation(self, x: float, y: float) -> float:
        # Right track only: y < -0.1 m
        if y > -0.1:
            return 0.0
        x_mod = x % self.spacing
        if 3.0 <= x_mod <= (3.0 + self.width):
            u = (x_mod - 3.0) / self.width
            return float(self.height * np.sin(np.pi * u) ** 2)
        return 0.0


PRESET_CATALOG: Dict[str, RoadPresetInfo] = {
    'smooth_highway': RoadPresetInfo(
        id='smooth_highway',
        name='1. Smooth Highway',
        description='Standard high-speed asphalt with minimal ISO Class A micro-texture.',
        educational_tip='With minimal excitation, the suspension remains near static equilibrium. Ideal baseline for observing static sag.',
        typical_amplitude_m=0.002
    ),
    'small_breakers': RoadPresetInfo(
        id='small_breakers',
        name='2. Small Speed Breakers',
        description='Gentle residential speed bumps (4.5 cm height, 1.2 m width).',
        educational_tip='Demonstrates impulse response. Notice how the front suspension compresses first, followed by the rear with a time delay L / v.',
        typical_amplitude_m=0.045
    ),
    'large_breakers': RoadPresetInfo(
        id='large_breakers',
        name='3. Large Speed Breakers',
        description='Severe traffic calming hump (8.5 cm height, 2.4 m width).',
        educational_tip='High vertical impulse tests damper dissipation and may engage non-linear bump stops if underdamped.',
        typical_amplitude_m=0.085
    ),
    'potholes': RoadPresetInfo(
        id='potholes',
        name='4. Pothole Road',
        description='Sharp pavement depressions (6 cm depth). Staggered left and right dips.',
        educational_tip='Suspension extends into the pothole (rebound stroke), then experiences high compressive shock on the lip.',
        typical_amplitude_m=0.060
    ),
    'wavy_road': RoadPresetInfo(
        id='wavy_road',
        name='5. Wavy Road (Resonance)',
        description='Continuous sinusoidal washboard road (wavelength 3.2 m).',
        educational_tip='When driving frequency f = v / lambda matches natural frequency fn, RESONANCE occurs and oscillation amplitude spikes violently!',
        typical_amplitude_m=0.038,
        wavelength_m=3.2
    ),
    'random_uneven': RoadPresetInfo(
        id='random_uneven',
        name='6. Random Uneven Road',
        description='Broadband ISO 8608 Class C/D road surface roughness.',
        educational_tip='Real-world roads contain multi-frequency noise. The suspension acts as a mechanical low-pass filter, isolating the chassis.',
        typical_amplitude_m=0.035
    ),
    'left_bump': RoadPresetInfo(
        id='left_bump',
        name='7. Left-Side Bump',
        description='Unilateral bump exciting only the left wheels.',
        educational_tip='Observe chassis roll (phi) and independent left suspension compression. Watch the Front View to see roll tilt.',
        typical_amplitude_m=0.075
    ),
    'right_bump': RoadPresetInfo(
        id='right_bump',
        name='8. Right-Side Bump',
        description='Unilateral bump exciting only the right wheels.',
        educational_tip='Complementary roll excitation on the right side. Highlights the role of anti-roll stiffness in vehicle handling.',
        typical_amplitude_m=0.075
    ),
}


def create_road_profile(preset_id: str, bump_height_scale: float = 1.0, roughness_scale: float = 1.0) -> RoadProfile:
    """Factory function to instantiate road profiles with given parameter scaling."""
    if preset_id == 'smooth_highway':
        return SmoothHighwayRoad(bump_height_scale, roughness_scale)
    elif preset_id == 'small_breakers':
        return SmallSpeedBreakersRoad(bump_height_scale, roughness_scale)
    elif preset_id == 'large_breakers':
        return LargeSpeedBreakersRoad(bump_height_scale, roughness_scale)
    elif preset_id == 'potholes':
        return PotholeRoad(bump_height_scale, roughness_scale)
    elif preset_id == 'wavy_road':
        return WavyRoad(bump_height_scale, roughness_scale)
    elif preset_id == 'random_uneven':
        return RandomUnevenRoad(bump_height_scale, roughness_scale)
    elif preset_id == 'left_bump':
        return LeftSideBumpRoad(bump_height_scale, roughness_scale)
    elif preset_id == 'right_bump':
        return RightSideBumpRoad(bump_height_scale, roughness_scale)
    else:
        return SmoothHighwayRoad(bump_height_scale, roughness_scale)
