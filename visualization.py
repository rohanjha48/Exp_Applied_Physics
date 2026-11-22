"""
visualization.py
================
Visualization manager for the 3D Vehicle Suspension Simulator.

Provides:
1. Publication-quality Matplotlib engineering plots and offline diagnostics:
   - Dynamic road response (Displacement, Velocity, Acceleration, Forces)
   - Damping regime step-response comparisons (Underdamped, Critical, Overdamped)
   - Transmissibility frequency-response Bode/resonance curves
2. Server configuration and assets for the real-time interactive 3D WebGL laboratory
   featuring dual synchronized viewpoints (Side View and Front View).
"""

import os
from typing import Dict, Any, List, Optional
import numpy as np

# Configure matplotlib for headless server / sandbox environments
os.environ.setdefault('MPLCONFIGDIR', '/tmp/matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from vehicle_model import VehicleParameters
from road_model import create_road_profile
from physics_engine import PhysicsEngine


class OfflinePlotter:
    """Generates high-resolution engineering figures for simulation analysis."""

    @staticmethod
    def generate_damping_comparison(output_path: str = "damping_regimes.png") -> str:
        """
        Simulate and plot the impulse/step response of the 3 fundamental damping regimes:
        Underdamped (zeta = 0.20), Critically Damped (zeta = 1.00), Overdamped (zeta = 2.20).
        """
        t_span = 3.5  # seconds
        dt = 0.001
        n_steps = int(t_span / dt)
        t_arr = np.linspace(0, t_span, n_steps)

        regimes = [
            ("Underdamped (ζ = 0.20)", 0.20, '#d97736'),
            ("Critically Damped (ζ = 1.00)", 1.00, '#2b8a3e'),
            ("Overdamped (ζ = 2.20)", 2.20, '#495057'),
        ]

        plt.style.use('default')
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True, dpi=180)
        fig.patch.set_facecolor('#faf8f5')
        for ax in (ax1, ax2):
            ax.set_facecolor('#ffffff')
            ax.grid(True, linestyle='--', alpha=0.5, color='#ced4da')

        initial_displacement = 0.05  # 50 mm initial disturbance

        for label, target_zeta, color in regimes:
            p = VehicleParameters()
            # Calculate c required to achieve target zeta for front corner
            qc = p.get_quarter_car_properties('FL')
            c_crit = qc['critical_damping_Ns_m']
            p.front_damping_coeff = target_zeta * c_crit
            p.rear_damping_coeff = target_zeta * c_crit

            engine = PhysicsEngine(p, road_profile=None)
            # Give initial chassis displacement (drop test)
            engine.state.z_c = initial_displacement

            zc_history = []
            dzc_history = []

            for _ in range(n_steps):
                engine.step(dt)
                zc_history.append(engine.state.z_c * 1000.0)  # mm
                dzc_history.append(engine.state.dz_c)

            ax1.plot(t_arr, zc_history, label=label, color=color, linewidth=2.2)
            ax2.plot(t_arr, dzc_history, label=label, color=color, linewidth=2.0)

        ax1.set_ylabel("Chassis Displacement [mm]", fontsize=11, fontweight='bold', color='#1f242c')
        ax1.set_title("Automotive Suspension Step Response: Damping Regimes Comparison",
                      fontsize=13, fontweight='bold', pad=12, color='#1f242c')
        ax1.legend(loc='upper right', framealpha=0.9, facecolor='#faf8f5', edgecolor='#ced4da')
        ax1.axhline(0, color='#868e96', linestyle='-', linewidth=0.8)

        ax2.set_xlabel("Time [seconds]", fontsize=11, fontweight='bold', color='#1f242c')
        ax2.set_ylabel("Chassis Velocity [m/s]", fontsize=11, fontweight='bold', color='#1f242c')
        ax2.axhline(0, color='#868e96', linestyle='-', linewidth=0.8)

        plt.tight_layout()
        plt.savefig(output_path, facecolor=fig.get_facecolor(), edgecolor='none')
        plt.close(fig)
        return output_path

    @staticmethod
    def generate_transmissibility_curve(output_path: str = "frequency_response.png") -> str:
        """
        Plot displacement transmissibility ratio T(r) = |X/Y| vs frequency ratio r = omega / omega_n
        for various damping ratios zeta, demonstrating resonance.
        """
        r = np.linspace(0.01, 3.0, 500)
        zetas = [0.1, 0.25, 0.5, 0.707, 1.0, 1.5]
        colors = ['#e03131', '#d97736', '#f59f00', '#2f9e44', '#1971c2', '#7048e8']

        fig, ax = plt.subplots(figsize=(9, 6), dpi=180)
        fig.patch.set_facecolor('#faf8f5')
        ax.set_facecolor('#ffffff')
        ax.grid(True, linestyle='--', alpha=0.5, color='#ced4da')

        p = VehicleParameters()
        for zeta, color in zip(zetas, colors):
            T = [p.transmissibility(ri, zeta) for ri in r]
            lbl = f"ζ = {zeta:.3f}" if zeta == 0.707 else f"ζ = {zeta:.2f}"
            if zeta == 0.707:
                lbl += " (Optimal Automotive)"
            elif zeta == 0.1:
                lbl += " (Severe Resonance)"
            ax.plot(r, T, label=lbl, color=color, linewidth=2.0)

        ax.axvline(1.0, color='#adb5bd', linestyle=':', linewidth=1.5, label='Resonance Peak (r = 1.0)')
        ax.axhline(1.0, color='#adb5bd', linestyle='--', linewidth=1.0)
        ax.set_xlim(0, 3.0)
        ax.set_ylim(0, 5.5)
        ax.set_xlabel("Frequency Ratio r = ω_road / ω_n", fontsize=11, fontweight='bold', color='#1f242c')
        ax.set_ylabel("Displacement Transmissibility T = |X / Y|", fontsize=11, fontweight='bold', color='#1f242c')
        ax.set_title("Suspension Transmissibility & Resonance (Forced Road Vibration)",
                     fontsize=13, fontweight='bold', pad=12, color='#1f242c')
        ax.legend(loc='upper right', framealpha=0.9, facecolor='#faf8f5', edgecolor='#ced4da')

        # Annotation of isolation zone
        ax.annotate('Vibration Isolation Zone (r > √2)', xy=(2.0, 0.8), xytext=(1.8, 2.5),
                    arrowprops=dict(facecolor='#1971c2', shrink=0.05, width=1.2, headwidth=6),
                    fontsize=10, fontweight='bold', color='#1971c2')

        plt.tight_layout()
        plt.savefig(output_path, facecolor=fig.get_facecolor(), edgecolor='none')
        plt.close(fig)
        return output_path

    @staticmethod
    def generate_road_telemetry_plot(road_id: str = "large_breakers", output_path: str = "suspension_telemetry.png") -> str:
        """
        Run multi-second simulation over selected road and plot synchronized 4-channel telemetry:
        1. Road Input vs Chassis & Wheel Displacement
        2. Suspension Stroke / Compression
        3. Vertical Acceleration
        4. Spring Force & Damper Force
        """
        p = VehicleParameters()
        road = create_road_profile(road_id)
        engine = PhysicsEngine(p, road)

        t_span = 4.0
        dt = 0.002
        n_steps = int(t_span / dt)

        time_hist = []
        zc_hist = []
        zw_hist = []
        road_hist = []
        stroke_hist = []
        accel_hist = []
        sp_f_hist = []
        da_f_hist = []

        for _ in range(n_steps):
            state = engine.step(dt)
            time_hist.append(state.time)
            zc_hist.append(state.z_c * 1000.0)
            zw_hist.append(state.z_w['FL'] * 1000.0)
            road_hist.append(state.road_z['FL'] * 1000.0)
            stroke_hist.append(state.susp_stroke['FL'] * 1000.0)
            accel_hist.append(state.ddz_c)
            sp_f_hist.append(state.spring_force['FL'])
            da_f_hist.append(state.damper_force['FL'])

        fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True, dpi=180)
        fig.patch.set_facecolor('#faf8f5')

        for ax in axes:
            ax.set_facecolor('#ffffff')
            ax.grid(True, linestyle='--', alpha=0.5, color='#ced4da')

        # 1. Road vs Vehicle Displacements
        axes[0].plot(time_hist, road_hist, label='Road Elevation z_road', color='#868e96', linestyle='--', linewidth=1.5)
        axes[0].plot(time_hist, zw_hist, label='Wheel Displacement z_w (FL)', color='#2b8a3e', linewidth=1.8)
        axes[0].plot(time_hist, zc_hist, label='Chassis Heave z_c', color='#d97736', linewidth=2.2)
        axes[0].set_ylabel("Displacement [mm]", fontweight='bold', color='#1f242c')
        axes[0].set_title(f"Dynamic Telemetry over {road.name} at {p.speed_kmh} km/h",
                          fontsize=13, fontweight='bold', pad=10, color='#1f242c')
        axes[0].legend(loc='upper right', framealpha=0.9, facecolor='#faf8f5')

        # 2. Suspension Compression
        axes[1].plot(time_hist, stroke_hist, label='Suspension Stroke (z_w - z_c)', color='#1971c2', linewidth=2.0)
        axes[1].axhline(p.bump_stop_clearance * 1000.0, color='#e03131', linestyle=':', label='+Bump Stop Limit')
        axes[1].axhline(-p.bump_stop_clearance * 1000.0, color='#e03131', linestyle=':', label='-Rebound Stop Limit')
        axes[1].set_ylabel("Stroke [mm]", fontweight='bold', color='#1f242c')
        axes[1].legend(loc='upper right', framealpha=0.9, facecolor='#faf8f5')

        # 3. Acceleration
        axes[2].plot(time_hist, accel_hist, label="Chassis Vertical Acceleration z_c''", color='#e03131', linewidth=1.8)
        axes[2].set_ylabel("Accel [m/s²]", fontweight='bold', color='#1f242c')
        axes[2].legend(loc='upper right', framealpha=0.9, facecolor='#faf8f5')

        # 4. Forces
        axes[3].plot(time_hist, sp_f_hist, label='Spring Force F_s = k·Δx', color='#f59f00', linewidth=1.8)
        axes[3].plot(time_hist, da_f_hist, label='Damper Force F_d = c·Δẋ', color='#7048e8', linewidth=1.8)
        axes[3].set_xlabel("Time [seconds]", fontweight='bold', color='#1f242c')
        axes[3].set_ylabel("Force [N]", fontweight='bold', color='#1f242c')
        axes[3].legend(loc='upper right', framealpha=0.9, facecolor='#faf8f5')

        plt.tight_layout()
        plt.savefig(output_path, facecolor=fig.get_facecolor(), edgecolor='none')
        plt.close(fig)
        return output_path
