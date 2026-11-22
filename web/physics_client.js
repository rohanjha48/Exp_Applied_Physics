/**
 * physics_client.js
 * =================
 * Full 7-DOF Multi-Body Vehicle Suspension Physics Engine in JavaScript.
 * Implements 4th-Order Runge-Kutta (RK4) integration with 1ms subcycling.
 *
 * Equations of Motion:
 *   Sprung Mass Heave:   M_s * z_c''  = sum(F_susp,i)
 *   Sprung Mass Pitch:   I_yy * theta'' = a * (F_FL + F_FR) - b * (F_RL + F_RR)
 *   Sprung Mass Roll:    I_xx * phi''   = (tf/2)*(F_FL - F_FR) + (tr/2)*(F_RL - F_RR)
 *   Unsprung Masses:     m_u,i * z_w,i'' = F_tire,i - F_susp,i
 *   Tire Ground Contact: F_tire = max(0, k_t*(z_road - z_w) + c_t*(z_road' - z_w'))
 */

class VehicleParametersJS {
  constructor() {
    this.sprungMass = 1400.0; // [kg]
    this.pitchInertia = 2100.0; // [kg*m^2]
    this.rollInertia = 650.0;   // [kg*m^2]

    this.wheelbase = 2.70;      // [m] L = a + b
    this.cgToFront = 1.25;      // [m] a
    this.cgToRear = 1.45;       // [m] b
    this.frontTrack = 1.55;     // [m] tf
    this.rearTrack = 1.55;      // [m] tr

    this.frontSpringStiffness = 28000.0; // [N/m]
    this.rearSpringStiffness = 26000.0;  // [N/m]
    this.frontDampingCoeff = 2200.0;     // [N*s/m]
    this.rearDampingCoeff = 2100.0;      // [N*s/m]

    this.bumpStopClearance = 0.08;      // [m]
    this.bumpStopStiffness = 180000.0;  // [N/m]

    this.frontUnsprungMass = 45.0;      // [kg]
    this.rearUnsprungMass = 42.0;       // [kg]

    this.tireStiffness = 200000.0;      // [N/m]
    this.tireDamping = 150.0;           // [N*s/m]

    this.wheelRadius = 0.33;            // [m]
    this.speedKmh = 50.0;               // [km/h]

    this.arbStiffnessFront = 8500.0;    // [N*m/rad]
    this.arbStiffnessRear = 4500.0;
  }

  get speedMs() {
    return this.speedKmh / 3.6;
  }

  set speedMs(v) {
    this.speedKmh = v * 3.6;
  }

  getQuarterCarMetrics(corner = 'FL') {
    const isFront = corner.includes('F');
    const m_total = isFront
      ? this.sprungMass * (this.cgToRear / this.wheelbase)
      : this.sprungMass * (this.cgToFront / this.wheelbase);
    const m_s = m_total / 2.0;
    const k_s = isFront ? this.frontSpringStiffness : this.rearSpringStiffness;
    const c_s = isFront ? this.frontDampingCoeff : this.rearDampingCoeff;

    const omega_n = Math.sqrt(k_s / m_s);
    const fn_hz = omega_n / (2.0 * Math.PI);
    const c_crit = 2.0 * Math.sqrt(m_s * k_s);
    const zeta = c_s / c_crit;
    const omega_d = zeta < 1.0 ? omega_n * Math.sqrt(1.0 - zeta * zeta) : 0.0;
    const fn_damped_hz = omega_d / (2.0 * Math.PI);
    const static_sag_mm = ((m_s * 9.80665) / k_s) * 1000.0;

    let regime = "Underdamped";
    if (Math.abs(zeta - 1.0) < 0.03) regime = "Critically Damped";
    else if (zeta > 1.0) regime = "Overdamped";

    return {
      corner,
      sprungMassKg: m_s,
      springStiffness: k_s,
      dampingCoeff: c_s,
      omegaN: omega_n,
      naturalFreqHz: fn_hz,
      criticalDamping: c_crit,
      dampingRatio: zeta,
      dampedNaturalFreqHz: fn_damped_hz,
      regime,
      staticSagMm: static_sag_mm,
    };
  }
}

class RoadModelJS {
  constructor(presetId = 'small_breakers', bumpScale = 1.0, roughnessScale = 1.0) {
    this.presetId = presetId;
    this.bumpScale = bumpScale;
    this.roughnessScale = roughnessScale;

    // Pre-generate pseudo-random phases for ISO 8608 roughness
    this.harmonics = [];
    const nH = 24;
    for (let i = 0; i < nH; i++) {
      const f = 0.05 * Math.pow(4.0 / 0.05, i / (nH - 1));
      const amp = 0.016 * Math.pow(f / 0.1, -1.1);
      const phaseL = (i * 1.6180339887) % (2.0 * Math.PI);
      const phaseR = ((i + 7) * 2.718281828) % (2.0 * Math.PI);
      this.harmonics.push({ f, amp, phaseL, phaseR });
    }
  }

  elevation(x, y) {
    const sB = this.bumpScale;
    const sR = this.roughnessScale;

    switch (this.presetId) {
      case 'smooth_highway': {
        const tex = 0.0012 * Math.sin(2.0 * Math.PI * x / 0.8) + 0.0008 * Math.cos(2.0 * Math.PI * x / 0.25);
        return tex * sR;
      }

      case 'small_breakers': {
        const spacing = 20.0;
        const width = 1.2;
        const height = 0.045 * sB;
        const xMod = ((x % spacing) + spacing) % spacing;
        let h = 0.0;
        if (xMod >= 2.0 && xMod <= 2.0 + width) {
          const u = (xMod - 2.0) / width;
          h = height * Math.pow(Math.sin(Math.PI * u), 2);
        }
        return h + 0.001 * Math.sin(x * 12.0) * sR;
      }

      case 'large_breakers': {
        const spacing = 28.0;
        const width = 2.4;
        const height = 0.085 * sB;
        const xMod = ((x % spacing) + spacing) % spacing;
        let h = 0.0;
        if (xMod >= 3.0 && xMod <= 3.0 + width) {
          const u = (xMod - 3.0) / width;
          h = height * Math.pow(Math.sin(Math.PI * u), 2);
        }
        return h + 0.001 * Math.sin(x * 15.0) * sR;
      }

      case 'potholes': {
        const spacing = 18.0;
        const width = 0.85;
        const depth = 0.060 * sB;
        const xMod = ((x % spacing) + spacing) % spacing;
        let h = 0.0;
        // Pothole 1 on left track
        if (xMod >= 2.5 && xMod <= 2.5 + width && y > -0.2) {
          const u = (xMod - 2.5) / width;
          h = -depth * Math.pow(Math.sin(Math.PI * u), 2);
        } else if (xMod >= 9.0 && xMod <= 9.0 + width * 0.9 && y < 0.2) {
          const u = (xMod - 9.0) / (width * 0.9);
          h = -depth * 0.85 * Math.pow(Math.sin(Math.PI * u), 2);
        }
        return h + 0.0018 * Math.sin(x * 9.0) * sR;
      }

      case 'wavy_road': {
        const wavelength = 3.2;
        const amplitude = 0.038 * sB;
        return amplitude * Math.sin(2.0 * Math.PI * x / wavelength);
      }

      case 'random_uneven': {
        const wL = Math.max(0.0, Math.min(1.0, 0.5 + y / 1.6));
        let zL = 0.0;
        let zR = 0.0;
        for (const h of this.harmonics) {
          zL += h.amp * Math.sin(2.0 * Math.PI * h.f * x + h.phaseL);
          zR += h.amp * Math.sin(2.0 * Math.PI * h.f * x + h.phaseR);
        }
        const z = (wL * zL + (1.0 - wL) * zR) * 0.35 * sB * sR;
        return Math.max(-0.07, Math.min(0.07, z));
      }

      case 'left_bump': {
        if (y < 0.1) return 0.0;
        const spacing = 24.0;
        const width = 1.8;
        const height = 0.075 * sB;
        const xMod = ((x % spacing) + spacing) % spacing;
        if (xMod >= 3.0 && xMod <= 3.0 + width) {
          const u = (xMod - 3.0) / width;
          return height * Math.pow(Math.sin(Math.PI * u), 2);
        }
        return 0.0;
      }

      case 'right_bump': {
        if (y > -0.1) return 0.0;
        const spacing = 24.0;
        const width = 1.8;
        const height = 0.075 * sB;
        const xMod = ((x % spacing) + spacing) % spacing;
        if (xMod >= 3.0 && xMod <= 3.0 + width) {
          const u = (xMod - 3.0) / width;
          return height * Math.pow(Math.sin(Math.PI * u), 2);
        }
        return 0.0;
      }

      default:
        return 0.0;
    }
  }

  elevationDerivative(x, y, vMs) {
    const dx = 0.005;
    const zp = this.elevation(x + dx, y);
    const zm = this.elevation(x - dx, y);
    return vMs * ((zp - zm) / (2.0 * dx));
  }
}

class PhysicsEngineJS {
  constructor(params = new VehicleParametersJS(), road = new RoadModelJS()) {
    this.params = params;
    this.road = road;

    this.time = 0.0;
    this.xPosition = 0.0;
    this.isPaused = false;
    this.timeScale = 1.0;

    // State vector: 14 elements
    // [z_c, dz_c, theta, dtheta, phi, dphi, z_FL, dz_FL, z_FR, dz_FR, z_RL, dz_RL, z_RR, dz_RR]
    this.state = new Float64Array(14);

    // Provide safe named properties directly on the state array
    Object.defineProperties(this.state, {
      z_c: { get: () => this.state[0], set: v => { this.state[0] = v; }, configurable: true },
      dz_c: { get: () => this.state[1], set: v => { this.state[1] = v; }, configurable: true },
      theta: { get: () => this.state[2], set: v => { this.state[2] = v; }, configurable: true },
      dtheta: { get: () => this.state[3], set: v => { this.state[3] = v; }, configurable: true },
      phi: { get: () => this.state[4], set: v => { this.state[4] = v; }, configurable: true },
      dphi: { get: () => this.state[5], set: v => { this.state[5] = v; }, configurable: true },
      z_w: {
        get: () => ({
          FL: this.state[6],
          FR: this.state[8],
          RL: this.state[10],
          RR: this.state[12],
        }),
        configurable: true,
      },
      dz_w: {
        get: () => ({
          FL: this.state[7],
          FR: this.state[9],
          RL: this.state[11],
          RR: this.state[13],
        }),
        configurable: true,
      },
      wheelAngle: { get: () => this.wheelAngle, configurable: true },
      wheel_angle: { get: () => this.wheelAngle, configurable: true },
      x_position: { get: () => this.xPosition, configurable: true },
      xPosition: { get: () => this.xPosition, configurable: true },
    });

    // Diagnostics / telemetry outputs
    this.ddz_c = 0.0;
    this.ddtheta = 0.0;
    this.ddphi = 0.0;
    this.ddz_w = { FL: 0.0, FR: 0.0, RL: 0.0, RR: 0.0 };

    this.suspStroke = { FL: 0.0, FR: 0.0, RL: 0.0, RR: 0.0 };
    this.springForce = { FL: 0.0, FR: 0.0, RL: 0.0, RR: 0.0 };
    this.damperForce = { FL: 0.0, FR: 0.0, RL: 0.0, RR: 0.0 };
    this.totalSuspForce = { FL: 0.0, FR: 0.0, RL: 0.0, RR: 0.0 };
    this.tireForce = { FL: 0.0, FR: 0.0, RL: 0.0, RR: 0.0 };

    this.roadZ = { FL: 0.0, FR: 0.0, RL: 0.0, RR: 0.0 };
    this.roadDz = { FL: 0.0, FR: 0.0, RL: 0.0, RR: 0.0 };

    this.wheelAngle = 0.0;
  }

  reset() {
    this.time = 0.0;
    this.xPosition = 0.0;
    this.state.fill(0.0);
    this.ddz_c = 0.0;
    this.ddtheta = 0.0;
    this.ddphi = 0.0;
    for (const c of ['FL', 'FR', 'RL', 'RR']) {
      this.ddz_w[c] = 0.0;
      this.suspStroke[c] = 0.0;
      this.springForce[c] = 0.0;
      this.damperForce[c] = 0.0;
      this.totalSuspForce[c] = 0.0;
      this.tireForce[c] = 0.0;
      this.roadZ[c] = 0.0;
      this.roadDz[c] = 0.0;
    }
    this.wheelAngle = 0.0;
  }

  computeDerivatives(S, t, xPos, dS, diagOut = false) {
    const p = this.params;
    const v = p.speedMs;

    const z_c = S[0], dz_c = S[1];
    const theta = S[2], dtheta = S[3];
    const phi = S[4], dphi = S[5];

    const z_w = { FL: S[6], FR: S[8], RL: S[10], RR: S[12] };
    const dz_w = { FL: S[7], FR: S[9], RL: S[11], RR: S[13] };

    const a = p.cgToFront, b = p.cgToRear;
    const tf2 = p.frontTrack / 2.0, tr2 = p.rearTrack / 2.0;

    // Corner kinematics of chassis mounting points
    const z_c_corner = {
      FL: z_c + a * theta + tf2 * phi,
      FR: z_c + a * theta - tf2 * phi,
      RL: z_c - b * theta + tr2 * phi,
      RR: z_c - b * theta - tr2 * phi,
    };
    const dz_c_corner = {
      FL: dz_c + a * dtheta + tf2 * dphi,
      FR: dz_c + a * dtheta - tf2 * dphi,
      RL: dz_c - b * dtheta + tr2 * dphi,
      RR: dz_c - b * dtheta - tr2 * dphi,
    };

    // Wheel road contacts
    const xCoord = {
      FL: xPos + a, FR: xPos + a,
      RL: xPos - b, RR: xPos - b,
    };
    const yCoord = {
      FL: tf2, FR: -tf2,
      RL: tr2, RR: -tr2,
    };

    const F_sp = {}, F_d = {}, F_susp = {}, F_tire = {}, stroke = {}, rZ = {}, rDz = {};

    for (const c of ['FL', 'FR', 'RL', 'RR']) {
      const isF = c.includes('F');
      const ks = isF ? p.frontSpringStiffness : p.rearSpringStiffness;
      const cs = isF ? p.frontDampingCoeff : p.rearDampingCoeff;

      const deltaZ = z_w[c] - z_c_corner[c];
      const deltaDz = dz_w[c] - dz_c_corner[c];
      stroke[c] = deltaZ;

      const fSpring = ks * deltaZ;
      const fDamper = cs * deltaDz;
      F_sp[c] = fSpring;
      F_d[c] = fDamper;

      let fBump = 0.0;
      if (deltaZ > p.bumpStopClearance) {
        const ex = deltaZ - p.bumpStopClearance;
        fBump = p.bumpStopStiffness * ex + (p.bumpStopStiffness * 2.0) * ex * ex;
      } else if (deltaZ < -p.bumpStopClearance) {
        const ex = -deltaZ - p.bumpStopClearance;
        fBump = -(p.bumpStopStiffness * ex + (p.bumpStopStiffness * 2.0) * ex * ex);
      }

      F_susp[c] = fSpring + fDamper + fBump;

      // Tire force & normal load
      const rz = this.road.elevation(xCoord[c], yCoord[c]);
      const rdz = this.road.elevationDerivative(xCoord[c], yCoord[c], v);
      rZ[c] = rz;
      rDz[c] = rdz;

      const dZ_tire = rz - z_w[c];
      const dDz_tire = rdz - dz_w[c];

      const m_total = isF
        ? p.sprungMass * (p.cgToRear / p.wheelbase)
        : p.sprungMass * (p.cgToFront / p.wheelbase);
      const m_s = m_total / 2.0;
      const m_u = isF ? p.frontUnsprungMass : p.rearUnsprungMass;
      const F0 = (m_s + m_u) * 9.80665;

      let deltaFt = p.tireStiffness * dZ_tire + p.tireDamping * dDz_tire;
      const F_tot = F0 + deltaFt;

      if (F_tot <= 0.0) {
        // Airborne tire
        deltaFt = -F0;
        F_tire[c] = 0.0;
      } else {
        F_tire[c] = F_tot;
      }

      F_tire[c + '_delta'] = deltaFt;
    }

    // Anti-roll bar torque
    const arbF = (p.arbStiffnessFront / (p.frontTrack * p.frontTrack)) * (z_w.FL - z_w.FR);
    F_susp.FL += arbF;
    F_susp.FR -= arbF;

    const arbR = (p.arbStiffnessRear / (p.rearTrack * p.rearTrack)) * (z_w.RL - z_w.RR);
    F_susp.RL += arbR;
    F_susp.RR -= arbR;

    // Chassis accelerations
    const sumF_susp = F_susp.FL + F_susp.FR + F_susp.RL + F_susp.RR;
    const ddz_c = sumF_susp / p.sprungMass;

    const pitchMom = a * (F_susp.FL + F_susp.FR) - b * (F_susp.RL + F_susp.RR);
    const ddtheta = pitchMom / p.pitchInertia;

    const rollMom = tf2 * (F_susp.FL - F_susp.FR) + tr2 * (F_susp.RL - F_susp.RR);
    const ddphi = rollMom / p.rollInertia;

    // Wheel accelerations
    const ddz_w = {};
    for (const c of ['FL', 'FR', 'RL', 'RR']) {
      const mu = c.includes('F') ? p.frontUnsprungMass : p.rearUnsprungMass;
      ddz_w[c] = (F_tire[c + '_delta'] - F_susp[c]) / mu;
    }

    // Assemble dS
    dS[0] = dz_c;
    dS[1] = ddz_c;
    dS[2] = dtheta;
    dS[3] = ddtheta;
    dS[4] = dphi;
    dS[5] = ddphi;

    dS[6] = dz_w.FL;
    dS[7] = ddz_w.FL;
    dS[8] = dz_w.FR;
    dS[9] = ddz_w.FR;
    dS[10] = dz_w.RL;
    dS[11] = ddz_w.RL;
    dS[12] = dz_w.RR;
    dS[13] = ddz_w.RR;

    if (diagOut) {
      this.ddz_c = ddz_c;
      this.ddtheta = ddtheta;
      this.ddphi = ddphi;
      this.ddz_w = ddz_w;
      this.suspStroke = stroke;
      this.springForce = F_sp;
      this.damperForce = F_d;
      this.totalSuspForce = F_susp;
      this.tireForce = F_tire;
      this.roadZ = rZ;
      this.roadDz = rDz;
    }
  }

  step(dt) {
    if (this.isPaused) return;

    const effectiveDt = dt * this.timeScale;
    const maxSubstep = 0.001; // 1 ms substepping
    const nSub = Math.max(1, Math.ceil(effectiveDt / maxSubstep));
    const h = effectiveDt / nSub;

    const v = this.params.speedMs;
    const S = this.state;
    const n = 14;

    const k1 = new Float64Array(n);
    const k2 = new Float64Array(n);
    const k3 = new Float64Array(n);
    const k4 = new Float64Array(n);
    const sTemp = new Float64Array(n);

    for (let step = 0; step < nSub; step++) {
      const isLast = step === nSub - 1;

      // k1
      this.computeDerivatives(S, this.time, this.xPosition, k1, isLast);

      // k2
      for (let i = 0; i < n; i++) sTemp[i] = S[i] + 0.5 * h * k1[i];
      this.computeDerivatives(sTemp, this.time + 0.5 * h, this.xPosition + 0.5 * h * v, k2, false);

      // k3
      for (let i = 0; i < n; i++) sTemp[i] = S[i] + 0.5 * h * k2[i];
      this.computeDerivatives(sTemp, this.time + 0.5 * h, this.xPosition + 0.5 * h * v, k3, false);

      // k4
      for (let i = 0; i < n; i++) sTemp[i] = S[i] + h * k3[i];
      this.computeDerivatives(sTemp, this.time + h, this.xPosition + h * v, k4, false);

      // Update state
      for (let i = 0; i < n; i++) {
        S[i] += (h / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
      }

      this.time += h;
      this.xPosition += h * v;
    }

    this.wheelAngle = this.xPosition / this.params.wheelRadius;
  }
}
