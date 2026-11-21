# REAL-TIME 3D VEHICLE SUSPENSION SIMULATOR
### Automotive Physics Experiential Learning Laboratory — Mass–Spring–Damper System

A comprehensive, physically authentic, real-time 3D vehicle suspension simulation built with Python, modern WebGL/Three.js, NumPy, SciPy, and Matplotlib. 

This platform models the multi-body dynamics of a complete passenger vehicle traversing varied road surfaces, providing students and engineers with deep intuition into harmonic motion, viscous damping, natural frequencies, resonance amplification, and suspension kinematics.

---

## Key Features

### 1. Realistic Automotive CAD Rolling Chassis & Independent Suspensions
- **CAD-Level Independent Suspension Assemblies**:
  - **Front Suspension — Double-Wishbone Architecture**:
    - Forged lower A-arm wishbone with I-beam cross-section, dual subframe pivot bushing eyes, and hollow lightening core.
    - Tubular arched upper control arm with twin ball joint mounts.
    - Cast steering knuckle / upright with spindle axle, steering arm boss, and caliper mounting brackets.
    - Steering rack and tie rods with inner/outer spherical joints and threaded hex adjustment sleeves.
    - Tubular front anti-roll bar (sway bar) with pillow-block bushing brackets and vertical drop links.
    - Coilover assembly featuring motorsport blue anodized camber plate / top hat with 3 chassis mounting studs and nuts, blue threaded preload collar lock rings, yellow polyurethane microcellular bump stop, dark titanium hydraulic damper cylinder, chrome piston rod, accordion dust boot, and high-visibility metallic red 8-coil helical spring with flat-ground end seats.
    - Brembo copper-red 6-piston monobloc brake caliper and floating ventilated brake rotor.
  - **Rear Suspension — Dedicated Multi-Link / SLA Architecture**:
    - Distinctly modeled independent multi-link geometry (NOT a duplicate of the front suspension).
    - Heavy forged lower camber control wishbone with subframe eccentric alignment cam bolt.
    - Tubular upper camber link with spherical rod ends (heim joints) and locking jam nuts.
    - Longitudinal trailing / integral link stabilizing brake torque and forward wheel travel.
    - Tubular toe control link with central hex adjustment sleeve for dynamic toe angle stabilization.
    - Cast rear hub carrier / knuckle upright with multi-point pivot bosses.
    - Heavy-duty steel CV drive half-shafts with convoluted rubber boots connecting the rear differential to the wheel hubs, articulating through dynamic suspension stroke.
    - Rear tubular anti-roll bar and drop links.
    - Rear coilover assembly with blue anodized top hat, preload collar, yellow bump stop, and metallic red helical spring.
    - Rear 4-piston copper-red brake caliper and ventilated rotor.
- **Parametric Wheels & High-Performance Tires**:
  - 13-point lathe-profiled $225/45\text{ R}18$ radial tires with procedural canvas PBR tread texture (continuous circumferential aquachannels and lateral shoulder sipes).
  - 10-spoke (5-split-spoke) flow-formed alloy wheels with recessed lug nuts and center cap.
  - 2-piece floating ventilated brake discs with procedural radial brushed sweep marks and spiral cross-drilled cooling holes.
  - Real-time wheel spin and half-shaft rotation driven directly by vehicle travel velocity.
- **Structural Frame Architecture**:
  - Heavy-duty boxed longitudinal perimeter frame rails with authentic upswept kick-ups over front and rear axle centers.
  - Front boxed suspension cradle subframe with lower wishbone pivot bushings and engine mounts.
  - Front crash bumper beam and radiator core support crossmember with electric cooling fans.
  - Sturdy cast aluminum suspension shock towers with front strut tower reinforcement brace across the engine bay.
  - Transmission crossmember with rubber vibration isolator.
  - Mid-chassis torsional crossmembers and structural diagonal X-braces.
  - Rear boxed suspension cradle subframe carrying the rear differential.
  - Rear bumper crash beam.
- **Powertrain & Drivetrain Components**:
  - Longitudinal engine block with aluminum cylinder head, valve covers, intake manifold with runner tubes, front crankshaft pulley, alternator, water pump, and serpentine belt drive.
  - Longitudinal ribbed transmission casing bolted to engine block.
  - Polished central steel propeller shaft (driveshaft) with universal joints (U-joints), spinning with vehicle velocity.
  - Rear differential carrier casing with cooling ribs and inspection cover.
  - Front and rear drive half-shafts (CV axles with ribbed black rubber CV boots) articulating with wheel stroke.
  - Dual stainless steel exhaust pipes and mufflers.
- **Chassis Display Mode Toggle**: One-click button (`🏎 Mode: Exposed Chassis` / `🚗 Mode: Complete Vehicle`) allows toggling between the pure exposed mechanical rolling chassis and a complete aerodynamic passenger car body.

### 2. Synchronized 3D Viewpoints with Multi-Angle Layouts & 3/4 Inspection
- **VIEW A — SIDE ELEVATION**:
  - Displays the full rolling chassis profile.
  - Highlights chassis heave ($z_c$), pitch rotation ($\theta$), front/rear axle phase delays, coil compression, and terrain hazards scrolling underneath the wheels.
  - Interactive `OrbitControls`: Click and drag to orbit, right-click to pan, scroll to zoom.
- **VIEW B — FRONT ELEVATION**:
  - Displays the rolling chassis head-on.
  - Highlights radiator, front suspension A-arms, steering rack, chassis roll tilt ($\phi$), track width, and asymmetric wheel deflection.
  - Interactive `OrbitControls`: Full independent 3D inspection of front suspension kinematics.
- **3/4 INSPECTION VIEW**:
  - Elevated isometric camera perspective (Front-3/4 on View A, Rear-3/4 on View B) specifically angled for simultaneous, close-up CAD inspection of both front and rear suspension corners under dynamic road excitation.
- **Single Physics State**: All viewports render the exact same physics simulation at 60 FPS without desynchronization.
- **Camera Controls**: Includes quick layout pills (Dual Sync View, 3/4 Inspection View, Side View Focus, Front View Focus) and a dedicated `📷 Reset Cameras` button to restore standard orthographic-style engineering elevations.

### 3. Rigorous 7-DOF Automotive Physics Engine
- **Governing Equations**:
  $$\text{Sprung Mass Heave: } M_s \ddot{z}_c = \sum_{i=1}^4 F_{\text{susp},i}$$
  $$\text{Sprung Mass Pitch: } I_{yy} \ddot{\theta} = a (F_{\text{susp,FL}} + F_{\text{susp,FR}}) - b (F_{\text{susp,RL}} + F_{\text{susp,RR}})$$
  $$\text{Sprung Mass Roll: } I_{xx} \ddot{\phi} = \frac{t_f}{2} (F_{\text{susp,FL}} - F_{\text{susp,FR}}) + \frac{t_r}{2} (F_{\text{susp,RL}} - F_{\text{susp,RR}})$$
  $$\text{Unsprung Wheel Masses: } m_{u,i} \ddot{z}_{w,i} = \Delta F_{\text{tire},i} - F_{\text{susp},i}$$
- **Numerical Integration**: 4th-Order Runge-Kutta (RK4) with subcycling ($dt_{\text{sub}} \le 1.0\text{ ms}$) for absolute numerical stability across sharp potholes and curbs.
- **Unilateral Ground Contact Law**: Tires maintain normal compressive load but exert zero tensile force when separated from the ground ($F_{\text{tire}} = 0$ upon liftoff).
- **Progressive Elastomeric Bump Stops**: Non-linear hardening spring engagement prevents chassis-hub metal-on-metal bottoming out.

### 4. Interactive Engineering Controls
- **Parameters**: Vehicle Speed (0–120 km/h), Spring Stiffness (10,000–75,000 N/m), Damping Coefficient (400–6,000 N·s/m), Vehicle Mass (800–2,500 kg), Bump Height Scale, Road Roughness, Simulation Speed (0.25x to 2.0x).
- **8 Road Surface Presets**:
  1. *Smooth Highway*: ISO 8608 Class A pavement micro-texture.
  2. *Small Speed Breakers*: 4.5 cm sinusoidal speed humps spaced every 20 m.
  3. *Large Speed Breakers*: Severe 8.5 cm traffic-calming humps.
  4. *Pothole Road*: Sharp 6.0 cm pavement depressions with staggered wheel tracks.
  5. *Wavy Road (Resonance)*: Continuous sinusoidal washboard road ($\lambda = 3.2\text{ m}$).
  6. *Random Uneven Road*: ISO 8608 multi-harmonic broadband road roughness.
  7. *Left-Side Bump*: Asymmetric bump on left wheel track only (triggers body roll).
  8. *Right-Side Bump*: Asymmetric bump on right wheel track only.
- **Quick Damping Regimes**: Underdamped ($\zeta = 0.20$), Optimal ($\zeta = 0.70$), Critically Damped ($\zeta = 1.00$), Overdamped ($\zeta = 2.20$).
- **⚡ Auto-Tune Resonance Button**: Automatically calculates and applies vehicle speed matching road wave excitation frequency to the vehicle's natural frequency ($f_{\text{road}} = f_n$).

### 5. Live Physics Dashboard & Oscilloscopes
- **10 Real-Time Metrics**: Vehicle displacement ($z_c$), wheel displacement ($z_w$), suspension stroke ($\Delta x$), spring force ($F_s$), damping force ($F_d$), natural frequency ($f_n$), damping ratio ($\zeta$), vertical acceleration ($\ddot{z}_c$ in $\text{m/s}^2$ and $g$), speed, pitch & roll angles.
- **4 Real-Time Oscilloscope Canvas Charts**:
  1. *Displacement vs Time*: Chassis heave ($z_c$) & wheel bounce ($z_w$).
  2. *Velocity vs Time*: Chassis heave velocity ($\dot{z}_c$).
  3. *Acceleration vs Time*: Chassis vertical acceleration ($\ddot{z}_c$).
  4. *Road Input vs Response*: Road surface elevation ($z_r$) vs suspension response ($z_c$).

### 6. Industrial Automotive Laboratory Visual Design
- **Color Palette**: Warm ivory / light grey background (`#faf8f5`), dark graphite panels (`#1c212d`), burnt copper / muted orange accents (`#d97736`), high-visibility red coil springs (`#d90429`), brushed aluminum components (`#d0d5dd`), rubber-black tires (`#181a1c`).
- **Typography & Layout**: Clean engineering monospace data tags, subtle millimeter technical grid floor, multi-point studio lighting with underbody suspension illumination.

---

## Architecture & Code Structure

The project is structured into modular Python and web components:

```
.
├── vehicle_model.py       # Vehicle mass, geometry, moments of inertia, and quarter-car metrics
├── road_model.py          # 8 Road elevation profiles and ISO 8608 terrain synthesis
├── physics_engine.py      # 7-DOF vehicle dynamics engine with RK4 numerical integration
├── dashboard.py           # Telemetry buffers, metric computations, and educational content
├── visualization.py      # Matplotlib publication plots and offline diagnostic exporters
├── main.py                # CLI runner, physics test suite, and HTTP simulation server
├── web/
│   ├── assets/            # Target directory for imported automotive CAD/GLTF assets (.glb)
│   ├── index.html         # Modern laboratory layout with dual viewports, CAD modal & HUD
│   ├── style.css          # Industrial automotive aesthetic styling
│   ├── physics_client.js  # Client-side 7-DOF physics engine (RK4 1ms)
│   ├── vehicle_3d.js      # CAD asset pipeline, node mapper, coilovers, wishbones & cameras
│   └── app.js             # Application orchestrator, live charts, CAD modal & event bindings
├── damping_regimes.png    # Exported step-response comparison plot
├── frequency_response.png # Exported transmissibility resonance curve
└── suspension_telemetry.png # Exported 4-channel dynamic road telemetry plot
```

---

## Quick Start & Usage

### 1. Requirements
- Python 3.8+
- NumPy
- SciPy
- Matplotlib

Install dependencies:
```bash
pip install numpy scipy matplotlib
```

### 2. Launch the Interactive 3D Simulation
Start the application and automatically open the laboratory in your default browser:
```bash
python3 main.py
```
Open **`http://localhost:8000/`** to view the interactive 3D laboratory.

Custom port:
```bash
python3 main.py --port 8080
```

Headless server mode (without auto-opening browser):
```bash
python3 main.py --no-browser
```

### 3. Run Automated Physics Verification Tests
Validate the 7-DOF mass-spring-damper mathematical engine against analytical solutions:
```bash
python3 main.py --test
```
This runs 6 automated test cases verifying:
1. Analytical natural frequency $\omega_n$, critical damping $c_{\text{crit}}$, and damping ratio $\zeta$.
2. Underdamped free oscillation period and damped frequency $\omega_d = \omega_n \sqrt{1 - \zeta^2}$.
3. Critically damped deadbeat response (zero overshoot).
4. Forced vibration resonance transmissibility peak.
5. Unilateral tire contact with liftoff handling ($F_{\text{tire}} = 0$).
6. Road elevation profile continuity and safety across all 8 presets.

### 4. Export Publication-Quality Matplotlib Figures
Generate high-resolution engineering plots for reports and presentations:
```bash
python3 main.py --export-plots
```
Generates:
- `damping_regimes.png`: Step response comparison of Underdamped, Critically Damped, and Overdamped systems.
- `frequency_response.png`: Transmissibility Bode curve $T(r) = |X/Y|$ showing resonance peaks and isolation zones ($r > \sqrt{2}$).
- `suspension_telemetry.png`: 4-channel time-series telemetry over speed breakers (Displacement, Stroke, Acceleration, Forces).

### 5. Terminal ASCII Live Simulation Mode
Run a quick terminal-based live telemetry stream:
```bash
python3 main.py --cli --road large_breakers
```

---

## Physics Summary & Equations

| Quantity | Formula | Typical Value / Units |
| :--- | :--- | :--- |
| **Sprung Mass per Corner** | $m_s = M_s \cdot (b / L) / 2$ | $350 - 450\text{ kg}$ |
| **Spring Restoring Force** | $F_s = k_s \cdot (z_w - z_c)$ | $20,000 - 40,000\text{ N/m}$ |
| **Viscous Damping Force** | $F_d = c_s \cdot (\dot{z}_w - \dot{z}_c)$ | $1,500 - 3,500\text{ N}\cdot\text{s/m}$ |
| **Natural Frequency** | $\omega_n = \sqrt{k_s / m_s}, \quad f_n = \omega_n / 2\pi$ | $1.1 - 1.4\text{ Hz}$ |
| **Critical Damping** | $c_{\text{crit}} = 2\sqrt{m_s k_s}$ | $5,000 - 7,000\text{ N}\cdot\text{s/m}$ |
| **Damping Ratio** | $\zeta = c_s / c_{\text{crit}}$ | $0.6 - 0.7$ (Automotive Optimum) |
| **Road Wave Frequency** | $f_{\text{road}} = v / \lambda$ | Excitation frequency (Hz) |
| **Displacement Transmissibility** | $T(r) = \sqrt{\frac{1 + (2\zeta r)^2}{(1 - r^2)^2 + (2\zeta r)^2}}$ | Magnification at $r = f_{\text{road}} / f_n$ |

---

## License
MIT License. Created for Automotive Physics Experiential Learning.
# Exp_Applied_Physics
