/**
 * app.js
 * ======
 * Main Web Application Orchestrator for 3D Vehicle Suspension Simulator.
 * Connects 7-DOF physics engine, dual 3D viewports, live telemetry charts,
 * CAD rolling chassis visualization, and interactive parameter controls.
 */

document.addEventListener('DOMContentLoaded', () => {
  // 1. INITIALIZE SIMULATION CORE
  const params = new VehicleParametersJS();
  const road = new RoadModelJS('small_breakers', 1.0, 1.0);
  const physics = new PhysicsEngineJS(params, road);

  // 2. INITIALIZE 3D RENDERER WITH SYNCHRONIZED VIEWPORTS
  const canvasA = document.getElementById('canvasSide');
  const canvasB = document.getElementById('canvasFront');

  const renderer = new Vehicle3DRenderer(canvasA, canvasB);

  // 3. TELEMETRY TIME-SERIES BUFFERS & OSCILLOSCOPE CHARTS
  const maxHistory = 200;
  const history = {
    time: [],
    zc: [],
    zw: [],
    dzc: [],
    ddzc: [],
    road: [],
    spForce: [],
    daForce: [],
  };

  const chartCanvases = {
    disp: document.getElementById('chartDisp'),
    vel: document.getElementById('chartVel'),
    accel: document.getElementById('chartAccel'),
    road: document.getElementById('chartRoad'),
  };

  // 4. DOM ELEMENTS FOR NUMERICAL DASHBOARD
  const dom = {
    valZc: document.getElementById('valZc'),
    valZw: document.getElementById('valZw'),
    valStroke: document.getElementById('valStroke'),
    valSpForce: document.getElementById('valSpForce'),
    valDaForce: document.getElementById('valDaForce'),
    valFn: document.getElementById('valFn'),
    valZeta: document.getElementById('valZeta'),
    valRegime: document.getElementById('valRegime'),
    valAccel: document.getElementById('valAccel'),
    valAccelG: document.getElementById('valAccelG'),
    valSpeed: document.getElementById('valSpeed'),
    valPitchRoll: document.getElementById('valPitchRoll'),
    hudSpeed: document.getElementById('hudSpeed'),
    hudTime: document.getElementById('hudTime'),
    parameterInsight: document.getElementById('parameterInsight'),
    btnPlayPause: document.getElementById('btnPlayPause'),
    btnReset: document.getElementById('btnReset'),
    btnRestart: document.getElementById('btnRestart'),
    btnChassisMode: document.getElementById('btnChassisMode'),
    btnResetCamera: document.getElementById('btnResetCamera'),
  };

  // 6. CONTROLS BINDING
  const sliders = {
    speed: { el: document.getElementById('sliderSpeed'), val: document.getElementById('valSliderSpeed') },
    stiffness: { el: document.getElementById('sliderStiffness'), val: document.getElementById('valSliderStiffness') },
    damping: { el: document.getElementById('sliderDamping'), val: document.getElementById('valSliderDamping') },
    mass: { el: document.getElementById('sliderMass'), val: document.getElementById('valSliderMass') },
    bumpHeight: { el: document.getElementById('sliderBumpHeight'), val: document.getElementById('valSliderBumpHeight') },
    roughness: { el: document.getElementById('sliderRoughness'), val: document.getElementById('valSliderRoughness') },
    simSpeed: { el: document.getElementById('sliderSimSpeed'), val: document.getElementById('valSliderSimSpeed') },
  };

  function updateParameterInsight() {
    const qc = params.getQuarterCarMetrics('FL');
    const k = qc.springStiffness;
    const c = qc.dampingCoeff;
    const m = qc.sprungMassKg;
    const fn = qc.naturalFreqHz;
    const zeta = qc.dampingRatio;
    const sag = qc.staticSagMm;

    let text = `<strong>Corner Sprung Mass:</strong> ${m.toFixed(1)} kg | <strong>Stiffness:</strong> ${k.toFixed(0)} N/m | <strong>Damping:</strong> ${c.toFixed(0)} N·s/m<br>`;
    text += `• <strong>Natural Frequency f_n = ${fn.toFixed(2)} Hz:</strong> `;
    if (fn < 1.0) text += "Low natural frequency (luxury/soft). Slow oscillation cycle.<br>";
    else if (fn <= 1.5) text += "Standard passenger car range (1.1 - 1.4 Hz). Balances comfort and stability.<br>";
    else text += "Sports car / track suspension. High vertical stiffness; transmits more road vibration.<br>";

    text += `• <strong>Damping Ratio ζ = ${zeta.toFixed(2)} (${qc.regime}):</strong> `;
    if (zeta < 0.35) text += "Underdamped. Rapid initial compliance, but car bounces for several cycles.<br>";
    else if (zeta >= 0.6 && zeta <= 0.8) text += "Automotive optimum (~0.7). Suppresses bounce within 1 cycle without jarring stiffness.<br>";
    else if (zeta < 1.0) text += "Firm damping. Tight body control, fast settling.<br>";
    else if (Math.abs(zeta - 1.0) < 0.05) text += "Critically damped (ζ = 1.0). Returns to equilibrium in minimum time without overshoot.<br>";
    else text += "Overdamped (ζ > 1.0). Sluggish suspension stroke; impacts transfer harshly to passengers.<br>";

    text += `• <strong>Static Suspension Sag:</strong> ${sag.toFixed(1)} mm under gravity.`;
    dom.parameterInsight.innerHTML = text;
  }

  // Sliders input events
  sliders.speed.el.addEventListener('input', (e) => {
    const v = parseFloat(e.target.value);
    params.speedKmh = v;
    sliders.speed.val.textContent = v < 0 ? `${v.toFixed(0)} km/h (Rev)` : `${v.toFixed(0)} km/h`;
  });

  sliders.stiffness.el.addEventListener('input', (e) => {
    const k = parseFloat(e.target.value);
    params.frontSpringStiffness = k;
    params.rearSpringStiffness = k * 0.92;
    sliders.stiffness.val.textContent = `${k.toLocaleString()} N/m`;
    updateParameterInsight();
  });

  sliders.damping.el.addEventListener('input', (e) => {
    const c = parseFloat(e.target.value);
    params.frontDampingCoeff = c;
    params.rearDampingCoeff = c * 0.95;
    sliders.damping.val.textContent = `${c.toLocaleString()} N·s/m`;
    updateParameterInsight();
  });

  sliders.mass.el.addEventListener('input', (e) => {
    const m = parseFloat(e.target.value);
    params.sprungMass = m;
    params.pitchInertia = m * 1.5;
    params.rollInertia = m * 0.46;
    sliders.mass.val.textContent = `${m.toLocaleString()} kg`;
    updateParameterInsight();
  });

  sliders.bumpHeight.el.addEventListener('input', (e) => {
    const h = parseFloat(e.target.value);
    road.bumpScale = h;
    sliders.bumpHeight.val.textContent = `${h.toFixed(1)}x`;
  });

  sliders.roughness.el.addEventListener('input', (e) => {
    const r = parseFloat(e.target.value);
    road.roughnessScale = r;
    sliders.roughness.val.textContent = `${r.toFixed(1)}x`;
  });

  sliders.simSpeed.el.addEventListener('input', (e) => {
    const s = parseFloat(e.target.value);
    physics.timeScale = s;
    sliders.simSpeed.val.textContent = `${s.toFixed(2)}x`;
  });

  // Road preset buttons
  document.querySelectorAll('.preset-btn').forEach(btn => {
    btn.addEventListener('click', () => {
      document.querySelectorAll('.preset-btn').forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
      const preset = btn.dataset.preset;
      road.presetId = preset;
    });
  });

  // Quick Damping Regime Buttons
  document.querySelectorAll('.regime-btn').forEach(btn => {
    btn.addEventListener('click', () => {
      document.querySelectorAll('.regime-btn').forEach(b => b.classList.remove('active'));
      btn.classList.add('active');

      const targetZeta = parseFloat(btn.dataset.zeta);
      const qc = params.getQuarterCarMetrics('FL');
      const cCrit = qc.criticalDamping;
      const targetC = targetZeta * cCrit;

      params.frontDampingCoeff = targetC;
      params.rearDampingCoeff = targetC * 0.95;
      sliders.damping.el.value = Math.min(6000, Math.max(400, targetC));
      sliders.damping.val.textContent = `${Math.round(targetC).toLocaleString()} N·s/m`;
      updateParameterInsight();
    });
  });

  // Auto-Tune Resonance Button
  const btnResonance = document.getElementById('btnResonance');
  if (btnResonance) {
    btnResonance.addEventListener('click', () => {
      // Set to wavy road
      document.querySelectorAll('.preset-btn').forEach(b => b.classList.remove('active'));
      const wavyBtn = document.querySelector('[data-preset="wavy_road"]');
      if (wavyBtn) wavyBtn.classList.add('active');
      road.presetId = 'wavy_road';

      // Low damping to highlight resonance
      const qc = params.getQuarterCarMetrics('FL');
      const targetC = 0.18 * qc.criticalDamping;
      params.frontDampingCoeff = targetC;
      params.rearDampingCoeff = targetC * 0.95;
      sliders.damping.el.value = targetC;
      sliders.damping.val.textContent = `${Math.round(targetC)} N·s/m`;

      // Set speed to match road wavelength (3.2m) * natural frequency
      const resonantSpeedMs = 3.2 * qc.naturalFreqHz;
      const resonantSpeedKmh = Math.min(120, resonantSpeedMs * 3.6);
      params.speedKmh = resonantSpeedKmh;
      sliders.speed.el.value = resonantSpeedKmh;
      sliders.speed.val.textContent = `${resonantSpeedKmh.toFixed(1)} km/h`;

      updateParameterInsight();
    });
  }

  // Play / Pause / Reset / Restart
  dom.btnPlayPause.addEventListener('click', () => {
    physics.isPaused = !physics.isPaused;
    dom.btnPlayPause.innerHTML = physics.isPaused ? '▶ Play' : '⏸ Pause';
  });

  dom.btnReset.addEventListener('click', () => {
    physics.reset();
    history.time.length = 0;
    history.zc.length = 0;
    history.zw.length = 0;
    history.dzc.length = 0;
    history.ddzc.length = 0;
    history.road.length = 0;
    history.spForce.length = 0;
    history.daForce.length = 0;
  });

  dom.btnRestart.addEventListener('click', () => {
    physics.reset();
  });

  // Chassis Display Mode (Exposed Rolling Chassis vs Complete Vehicle)
  if (dom.btnChassisMode) {
    dom.btnChassisMode.addEventListener('click', () => {
      const isExposed = renderer.toggleChassisMode();
      if (isExposed) {
        dom.btnChassisMode.classList.add('active');
        dom.btnChassisMode.innerHTML = '🏎 Mode: Exposed Chassis';
      } else {
        dom.btnChassisMode.classList.remove('active');
        dom.btnChassisMode.innerHTML = '🚗 Mode: Complete Vehicle';
      }
    });
  }

  // Reset 3D Viewport Orbit Cameras to True Orthogonal Elevation Angles
  if (dom.btnResetCamera) {
    dom.btnResetCamera.addEventListener('click', () => {
      renderer.resetCameras();
      document.querySelectorAll('.view-pill').forEach(p => p.classList.remove('active'));
      document.querySelector('.view-pill[data-view="dual"]')?.classList.add('active');
      viewportsWrapper.classList.remove('side-focus', 'front-focus');
      if (titleViewA) titleViewA.textContent = 'VIEW A — SIDE ELEVATION';
      if (titleViewB) titleViewB.textContent = 'VIEW B — FRONT ELEVATION';
      setTimeout(() => renderer.handleResize(), 50);
    });
  }

  // View Layout Switcher
  const viewportsWrapper = document.getElementById('viewportsWrapper');
  const titleViewA = document.getElementById('titleViewA');
  const titleViewB = document.getElementById('titleViewB');

  document.querySelectorAll('.view-pill').forEach(pill => {
    pill.addEventListener('click', () => {
      document.querySelectorAll('.view-pill').forEach(p => p.classList.remove('active'));
      pill.classList.add('active');
      const mode = pill.dataset.view;

      // Update 3D Camera Angles & Target
      if (renderer && renderer.setCameraMode) {
        renderer.setCameraMode(mode);
      }

      viewportsWrapper.classList.remove('side-focus', 'front-focus');
      if (mode === 'side') {
        viewportsWrapper.classList.add('side-focus');
        if (titleViewA) titleViewA.textContent = 'VIEW A — SIDE ELEVATION (FULL)';
      } else if (mode === 'front') {
        viewportsWrapper.classList.add('front-focus');
        if (titleViewB) titleViewB.textContent = 'VIEW B — FRONT ELEVATION (FULL)';
      } else if (mode === 'iso') {
        if (titleViewA) titleViewA.textContent = 'VIEW A — FRONT 3/4 SUSPENSION INSPECTION';
        if (titleViewB) titleViewB.textContent = 'VIEW B — REAR 3/4 SUSPENSION INSPECTION';
      } else if (mode === 'front_closeup') {
        viewportsWrapper.classList.add('side-focus');
        if (titleViewA) titleViewA.textContent = 'VIEW A — FRONT-LEFT (FL) SUSPENSION CLOSE-UP';
        if (titleViewB) titleViewB.textContent = 'VIEW B — FRONT ELEVATION';
      } else if (mode === 'front_right_closeup') {
        viewportsWrapper.classList.add('side-focus');
        if (titleViewA) titleViewA.textContent = 'VIEW A — FRONT-RIGHT (FR) SUSPENSION CLOSE-UP';
        if (titleViewB) titleViewB.textContent = 'VIEW B — FRONT ELEVATION';
      } else if (mode === 'rear_closeup') {
        viewportsWrapper.classList.add('front-focus');
        if (titleViewA) titleViewA.textContent = 'VIEW A — SIDE ELEVATION';
        if (titleViewB) titleViewB.textContent = 'VIEW B — REAR-LEFT (RL) SUSPENSION CLOSE-UP';
      } else if (mode === 'rear_right_closeup') {
        viewportsWrapper.classList.add('front-focus');
        if (titleViewA) titleViewA.textContent = 'VIEW A — SIDE ELEVATION';
        if (titleViewB) titleViewB.textContent = 'VIEW B — REAR-RIGHT (RR) SUSPENSION CLOSE-UP';
      } else if (mode === 'compression' || mode === 'rebound') {
        viewportsWrapper.classList.add('side-focus');
        if (titleViewA) titleViewA.textContent = mode === 'compression' ? 'VIEW A — SUSPENSION COMPRESSION TEST' : 'VIEW A — SUSPENSION REBOUND TEST';
      } else if (mode === 'chassis_34') {
        viewportsWrapper.classList.add('side-focus');
        if (titleViewA) titleViewA.textContent = 'VIEW A — 3/4 ROLLING CHASSIS INSPECTION';
      } else {
        if (titleViewA) titleViewA.textContent = 'VIEW A — SIDE ELEVATION';
        if (titleViewB) titleViewB.textContent = 'VIEW B — FRONT ELEVATION';
      }
      setTimeout(() => renderer.handleResize(), 100);
    });
  });

  // Helper: Secure HTML Entity Escaping (Prevents DOM XSS / Injection)
  function escapeHtml(str) {
    if (str === null || str === undefined) return '';
    return String(str)
      .replace(/&/g, '&amp;')
      .replace(/</g, '&lt;')
      .replace(/>/g, '&gt;')
      .replace(/"/g, '&quot;')
      .replace(/'/g, '&#039;');
  }

  // Handle URL query parameter for automated view inspection with strict whitelisting
  const ALLOWED_VIEWS = [
    'dual', 'iso', 'side', 'front', 'front_closeup', 'front_right_closeup',
    'rear_closeup', 'rear_right_closeup', 'compression', 'rebound', 'chassis_34'
  ];
  const ALLOWED_ROADS = [
    'highway_smooth', 'small_breakers', 'large_breakers', 'pothole_asymmetric',
    'pothole_single', 'wavy_road', 'washboard_corrugations', 'curb_strike'
  ];
  const ALLOWED_CHASSIS = ['complete', 'exposed'];

  const urlParams = new URLSearchParams(window.location.search);
  const rawView = urlParams.get('view');
  const initialView = (rawView && ALLOWED_VIEWS.includes(rawView)) ? rawView : null;
  if (initialView) {
    setTimeout(() => {
      if (renderer && renderer.setCameraMode) {
        renderer.setCameraMode(initialView);
      }
      const targetPill = document.querySelector(`.view-pill[data-view="${CSS.escape(initialView)}"]`);
      if (targetPill) {
        document.querySelectorAll('.view-pill').forEach(p => p.classList.remove('active'));
        targetPill.classList.add('active');
      }
      viewportsWrapper.classList.remove('side-focus', 'front-focus');
      if (initialView === 'side') {
        viewportsWrapper.classList.add('side-focus');
        if (titleViewA) titleViewA.textContent = 'VIEW A — SIDE ELEVATION (FULL)';
      } else if (initialView === 'front') {
        viewportsWrapper.classList.add('front-focus');
        if (titleViewB) titleViewB.textContent = 'VIEW B — FRONT ELEVATION (FULL)';
      } else if (initialView === 'iso') {
        if (titleViewA) titleViewA.textContent = 'VIEW A — FRONT 3/4 SUSPENSION INSPECTION';
        if (titleViewB) titleViewB.textContent = 'VIEW B — REAR 3/4 SUSPENSION INSPECTION';
      } else if (initialView === 'front_closeup') {
        viewportsWrapper.classList.add('side-focus');
        if (titleViewA) titleViewA.textContent = 'VIEW A — FRONT-LEFT (FL) SUSPENSION CLOSE-UP';
        if (titleViewB) titleViewB.textContent = 'VIEW B — FRONT ELEVATION';
      } else if (initialView === 'front_right_closeup') {
        viewportsWrapper.classList.add('side-focus');
        if (titleViewA) titleViewA.textContent = 'VIEW A — FRONT-RIGHT (FR) SUSPENSION CLOSE-UP';
        if (titleViewB) titleViewB.textContent = 'VIEW B — FRONT ELEVATION';
      } else if (initialView === 'rear_closeup') {
        viewportsWrapper.classList.add('front-focus');
        if (titleViewA) titleViewA.textContent = 'VIEW A — SIDE ELEVATION';
        if (titleViewB) titleViewB.textContent = 'VIEW B — REAR-LEFT (RL) SUSPENSION CLOSE-UP';
      } else if (initialView === 'rear_right_closeup') {
        viewportsWrapper.classList.add('front-focus');
        if (titleViewA) titleViewA.textContent = 'VIEW A — SIDE ELEVATION';
        if (titleViewB) titleViewB.textContent = 'VIEW B — REAR-RIGHT (RR) SUSPENSION CLOSE-UP';
      } else if (initialView === 'compression' || initialView === 'rebound') {
        viewportsWrapper.classList.add('side-focus');
        if (titleViewA) titleViewA.textContent = initialView === 'compression' ? 'VIEW A — SUSPENSION COMPRESSION TEST' : 'VIEW A — SUSPENSION REBOUND TEST';
      } else if (initialView === 'chassis_34') {
        viewportsWrapper.classList.add('side-focus');
        if (titleViewA) titleViewA.textContent = 'VIEW A — 3/4 ROLLING CHASSIS INSPECTION';
      }
      renderer.handleResize();
    }, 150);
  }

  // Handle URL query parameters for road profile and vehicle speed (?road=large_breakers, speed=0, etc.)
  const rawRoad = urlParams.get('road');
  const initialRoad = (rawRoad && ALLOWED_ROADS.includes(rawRoad)) ? rawRoad : null;
  if (initialRoad) {
    setTimeout(() => {
      const roadBtn = document.querySelector(`.preset-btn[data-preset="${CSS.escape(initialRoad)}"]`);
      if (roadBtn) roadBtn.click();
    }, 200);
  }

  const initialSpeed = urlParams.get('speed');
  if (initialSpeed !== null) {
    const spd = parseFloat(initialSpeed);
    if (!isNaN(spd) && isFinite(spd) && spd >= -50.0 && spd <= 200.0) {
      params.speedKmh = spd;
      if (sliders.speed && sliders.speed.el) sliders.speed.el.value = spd;
      if (sliders.speed && sliders.speed.val) {
        sliders.speed.val.textContent = spd < 0 ? `${spd.toFixed(0)} km/h (Rev)` : `${spd.toFixed(0)} km/h`;
      }
    }
  }

  const rawChassis = urlParams.get('chassis');
  const chassisParam = (rawChassis && ALLOWED_CHASSIS.includes(rawChassis)) ? rawChassis : null;
  if (chassisParam) {
    setTimeout(() => {
      if (renderer) {
        if (chassisParam === 'complete' && renderer.isExposedChassis) {
          if (dom.btnChassisMode) dom.btnChassisMode.click();
        } else if (chassisParam === 'exposed' && !renderer.isExposedChassis) {
          if (dom.btnChassisMode) dom.btnChassisMode.click();
        }
      }
    }, 250);
  }

  const modalParam = urlParams.get('modal');
  if (modalParam === 'cad') {
    setTimeout(() => {
      renderAssetReportUI();
      if (assetModal) assetModal.classList.add('open');
    }, 350);
  }

  // Educational Modal
  const eduModal = document.getElementById('eduModal');
  const btnEdu = document.getElementById('btnEdu');
  const btnCloseEdu = document.getElementById('btnCloseEdu');

  btnEdu.addEventListener('click', () => eduModal.classList.add('open'));
  btnCloseEdu.addEventListener('click', () => eduModal.classList.remove('open'));
  eduModal.addEventListener('click', (e) => {
    if (e.target === eduModal) eduModal.classList.remove('open');
  });

  // CAD Asset Pipeline Modal
  const assetModal = document.getElementById('assetReportModal');
  const btnAssetReport = document.getElementById('btnAssetReport');
  const btnCloseAssetReport = document.getElementById('btnCloseAssetReport');
  const assetModalBody = document.getElementById('assetReportModalBody');

  function renderAssetReportUI() {
    if (!renderer || !renderer.getAssetReport || !assetModalBody) return;
    const report = renderer.getAssetReport();
    if (!report) return;

    let html = `
      <div class="cad-report-section">
        <h3>
          <span>PIPELINE STATUS</span>
          <span class="status-badge-inline ${report.pipelineStatus === 'ACTIVE_CAD_LOADED' ? 'matched' : 'pending'}">
            ${report.pipelineStatus === 'ACTIVE_CAD_LOADED' ? '✓ CAD ASSETS LOADED & MOUNTED' : 'AWAITING CAD ASSETS (BASELINE FALLBACK ACTIVE)'}
          </span>
        </h3>
        <p style="font-size: 0.82rem; margin: 4px 0 10px 0; color: #94a3b8;">
          Three.js Loader: <strong>${report.loaderAvailable ? 'THREE.GLTFLoader Active' : 'Offline / Mock Mode'}</strong> •
          Target Directory: <code>web/assets/</code> • Coordinate Scale: <code>1 Unit = 1.0 Metre</code>
        </p>
      </div>

      <div class="cad-report-section">
        <h3>REQUIRED AUTOMOTIVE CAD ASSETS</h3>
        <table class="cad-table">
          <thead>
            <tr>
              <th>Target Asset</th>
              <th>Path</th>
              <th>Status</th>
              <th>Measured Dimensions</th>
            </tr>
          </thead>
          <tbody>
    `;

    for (const [id, a] of Object.entries(report.manifest)) {
      const isLoaded = a.status === 'loaded';
      html += `
        <tr>
          <td><strong>${escapeHtml(a.label)}</strong></td>
          <td><code>web/${escapeHtml(a.file)}</code></td>
          <td>
            <span class="status-badge-inline ${isLoaded ? 'matched' : 'missing'}">
              ${isLoaded ? '✓ LOADED' : 'MISSING (404)'}
            </span>
          </td>
          <td>${escapeHtml(a.dimensions) || '—'}</td>
        </tr>
      `;
    }

    html += `
          </tbody>
        </table>
      </div>

      <div class="cad-report-section">
        <h3>CANONICAL MECHANICAL NODE HIERARCHY & MAPPING</h3>
        <p style="font-size: 0.82rem; color: #94a3b8; margin-bottom: 8px;">
          The simulation connects 7-DOF kinematics to the following 24 mechanical components. If a node is missing from an imported model, it is flagged below:
        </p>
        <table class="cad-table">
          <thead>
            <tr>
              <th>Canonical Role</th>
              <th>Subsystem</th>
              <th>Mapping Status</th>
              <th>Accepted Node Aliases</th>
            </tr>
          </thead>
          <tbody>
    `;

    const rolesInfo = [
      { role: 'Chassis_Frame', sub: 'Sprung Mass', aliases: 'frame, chassis, main_frame, rails' },
      { role: 'Front_Subframe', sub: 'Sprung Mass', aliases: 'front_subframe, f_subframe, cradle_front' },
      { role: 'Rear_Subframe', sub: 'Sprung Mass', aliases: 'rear_subframe, r_subframe, cradle_rear' },
      { role: 'Engine_Transmission', sub: 'Sprung Mass', aliases: 'engine, transmission, powertrain, motor' },
      { role: 'Differential', sub: 'Sprung Mass', aliases: 'diff, differential, rear_diff, final_drive' },
      { role: 'Steering_Rack', sub: 'Sprung Mass', aliases: 'steering_rack, rack_and_pinion, rack' },

      { role: 'FL_UpperWishbone', sub: 'Front Left', aliases: 'fl_upperwishbone, fl_uca, fl_upper_arm' },
      { role: 'FL_LowerWishbone', sub: 'Front Left', aliases: 'fl_lowerwishbone, fl_lca, fl_lower_arm' },
      { role: 'FL_Knuckle', sub: 'Front Left', aliases: 'fl_knuckle, fl_upright, fl_spindle' },
      { role: 'FL_DamperBody', sub: 'Front Left', aliases: 'fl_damperbody, fl_shock_body, damper_outer' },
      { role: 'FL_PistonRod', sub: 'Front Left', aliases: 'fl_pistonrod, fl_damper_inner, shock_shaft' },
      { role: 'FL_CoilSpring', sub: 'Front Left', aliases: 'fl_coilspring, fl_spring, helical_spring' },
      { role: 'FL_BrakeCaliper', sub: 'Front Left', aliases: 'fl_brakecaliper, fl_caliper' },

      { role: 'FR_UpperWishbone', sub: 'Front Right', aliases: 'fr_upperwishbone, fr_uca, fr_upper_arm' },
      { role: 'FR_LowerWishbone', sub: 'Front Right', aliases: 'fr_lowerwishbone, fr_lca, fr_lower_arm' },
      { role: 'FR_Knuckle', sub: 'Front Right', aliases: 'fr_knuckle, fr_upright, fr_spindle' },
      { role: 'FR_DamperBody', sub: 'Front Right', aliases: 'fr_damperbody, fr_shock_body, damper_outer' },
      { role: 'FR_PistonRod', sub: 'Front Right', aliases: 'fr_pistonrod, fr_damper_inner, shock_shaft' },
      { role: 'FR_CoilSpring', sub: 'Front Right', aliases: 'fr_coilspring, fr_spring, helical_spring' },
      { role: 'FR_BrakeCaliper', sub: 'Front Right', aliases: 'fr_brakecaliper, fr_caliper' },

      { role: 'RL_UpperControlArm', sub: 'Rear Left', aliases: 'rl_uppercontrolarm, rl_uca, rl_camber_link' },
      { role: 'RL_LowerControlArm', sub: 'Rear Left', aliases: 'rl_lowercontrolarm, rl_lca, rl_camber_arm' },
      { role: 'RL_Knuckle', sub: 'Rear Left', aliases: 'rl_knuckle, rl_upright, rl_hub_carrier' },
      { role: 'RL_DamperBody', sub: 'Rear Left', aliases: 'rl_damperbody, rl_shock_body, damper_outer' },
      { role: 'RL_PistonRod', sub: 'Rear Left', aliases: 'rl_pistonrod, rl_damper_inner, shock_shaft' },
      { role: 'RL_CoilSpring', sub: 'Rear Left', aliases: 'rl_coilspring, rl_spring, helical_spring' },
      { role: 'RL_BrakeCaliper', sub: 'Rear Left', aliases: 'rl_brakecaliper, rl_caliper' },

      { role: 'RR_UpperControlArm', sub: 'Rear Right', aliases: 'rr_uppercontrolarm, rr_uca, rr_camber_link' },
      { role: 'RR_LowerControlArm', sub: 'Rear Right', aliases: 'rr_lowercontrolarm, rr_lca, rr_camber_arm' },
      { role: 'RR_Knuckle', sub: 'Rear Right', aliases: 'rr_knuckle, rr_upright, rr_hub_carrier' },
      { role: 'RR_DamperBody', sub: 'Rear Right', aliases: 'rr_damperbody, rr_shock_body, damper_outer' },
      { role: 'RR_PistonRod', sub: 'Rear Right', aliases: 'rr_pistonrod, rr_damper_inner, shock_shaft' },
      { role: 'RR_CoilSpring', sub: 'Rear Right', aliases: 'rr_coilspring, rr_spring, helical_spring' },
      { role: 'RR_BrakeCaliper', sub: 'Rear Right', aliases: 'rr_brakecaliper, rr_caliper' },

      { role: 'WheelAssembly (Tire, Rim, Rotor)', sub: 'All 4 Corners', aliases: 'tire, rim, brake_rotor, hub, lug_nuts, center_cap' }
    ];

    rolesInfo.forEach(item => {
      let isMatched = false;
      let matchedNode = '';
      if (report.reports) {
        for (const rep of Object.values(report.reports)) {
          if (rep && rep.mapped && rep.mapped[item.role]) {
            isMatched = true;
            matchedNode = rep.mapped[item.role].nodeName;
            break;
          }
        }
      }

      html += `
        <tr>
          <td><code>${escapeHtml(item.role)}</code></td>
          <td>${escapeHtml(item.sub)}</td>
          <td>
            <span class="status-badge-inline ${isMatched ? 'matched' : 'missing'}">
              ${isMatched ? `✓ MATCHED: "${escapeHtml(matchedNode)}"` : 'MISSING (Awaiting CAD Model)'}
            </span>
          </td>
          <td><span style="color:#64748b;">${escapeHtml(item.aliases)}</span></td>
        </tr>
      `;
    });

    html += `
          </tbody>
        </table>
      </div>

      <div class="cad-report-section">
        <h3>VEHICLE DIMENSIONAL & COORDINATE AUDIT</h3>
        <p style="font-size: 0.82rem; color: #94a3b8;">
          All CAD models are verified against the 7-DOF multi-body physics coordinate framework:
        </p>
        <div class="code-snippet-box">
• Coordinate Axes:      +X = Vehicle Forward, +Y = Vehicle Up, +Z = Vehicle Left
• Unit Scale:           1 Three.js Unit = 1.0 Metre (1:1 Engineering Scale)
• Wheelbase (L):        2.70 m (a = 1.25 m forward of CG, b = 1.45 m rearward of CG)
• Track Width (t_f/t_r): 1.55 m (Left = +0.775 m, Right = -0.775 m)
• Tire Radius (R_w):    0.33 m (Overall Wheel Diameter = 0.66 m)
• Static Positions:     FL: (+1.25, 0.33, +0.775)   FR: (+1.25, 0.33, -0.775)
                        RL: (-1.45, 0.33, +0.775)   RR: (-1.45, 0.33, -0.775)
        </div>
      </div>

      <div class="cad-report-section">
        <h3>HOW TO PLACE NEW CAD ASSETS</h3>
        <ol style="margin-left: 18px; font-size: 0.82rem; color: #cbd5e1; line-height: 1.6;">
          <li>Export your automotive CAD assembly from SolidWorks / CATIA / Blender into binary glTF (<code>.glb</code>).</li>
          <li>Place the file(s) into <code>web/assets/</code> (e.g. <code>web/assets/rolling_chassis.glb</code>).</li>
          <li>Refresh or launch the simulator (<code>python3 main.py</code>).</li>
          <li>The pipeline will automatically validate bounds, map the 24 mechanical joints, and connect them to the 7-DOF physics engine!</li>
        </ol>
      </div>
    `;

    assetModalBody.innerHTML = html;
  }

  if (btnAssetReport && assetModal) {
    btnAssetReport.addEventListener('click', () => {
      renderAssetReportUI();
      assetModal.classList.add('open');
    });
  }
  if (btnCloseAssetReport && assetModal) {
    btnCloseAssetReport.addEventListener('click', () => assetModal.classList.remove('open'));
  }
  if (assetModal) {
    assetModal.addEventListener('click', (e) => {
      if (e.target === assetModal) assetModal.classList.remove('open');
    });
  }

  window.addEventListener('resize', () => renderer.handleResize());

  // 7. CHART DRAWING ROUTINES (HIGH-PERFORMANCE CANVAS PLOTTING)
  function drawChart(canvas, traces, yRange = null, zeroLine = true) {
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    const W = canvas.width = canvas.clientWidth * window.devicePixelRatio;
    const H = canvas.height = canvas.clientHeight * window.devicePixelRatio;

    ctx.clearRect(0, 0, W, H);

    // Background grid
    ctx.strokeStyle = '#222836';
    ctx.lineWidth = 1;
    for (let x = 0; x <= W; x += W / 8) {
      ctx.beginPath();
      ctx.moveTo(x, 0);
      ctx.lineTo(x, H);
      ctx.stroke();
    }
    for (let y = 0; y <= H; y += H / 4) {
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(W, y);
      ctx.stroke();
    }

    if (traces.length === 0 || traces[0].data.length < 2) return;

    // Find y-limits if not provided
    let minY = Infinity;
    let maxY = -Infinity;
    if (yRange) {
      minY = yRange[0];
      maxY = yRange[1];
    } else {
      traces.forEach(t => {
        for (let i = 0; i < t.data.length; i++) {
          const v = t.data[i];
          if (v < minY) minY = v;
          if (v > maxY) maxY = v;
        }
      });
      if (minY === maxY) {
        minY -= 1;
        maxY += 1;
      }
      const pad = (maxY - minY) * 0.15;
      minY -= pad;
      maxY += pad;
    }

    const nPts = traces[0].data.length;

    // Zero line
    if (zeroLine && minY < 0 && maxY > 0) {
      const yZero = H - ((0 - minY) / (maxY - minY)) * H;
      ctx.strokeStyle = '#414b62';
      ctx.lineWidth = 1.5;
      ctx.setLineDash([4, 4]);
      ctx.beginPath();
      ctx.moveTo(0, yZero);
      ctx.lineTo(W, yZero);
      ctx.stroke();
      ctx.setLineDash([]);
    }

    // Draw traces
    traces.forEach(t => {
      ctx.strokeStyle = t.color;
      ctx.lineWidth = t.lineWidth || 2.0;
      ctx.beginPath();
      for (let i = 0; i < nPts; i++) {
        const x = (i / (nPts - 1)) * W;
        const y = H - ((t.data[i] - minY) / (maxY - minY)) * H;
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      }
      ctx.stroke();
    });
  }

  // 8. ANIMATION LOOP (60 FPS)
  let lastFrameTime = performance.now();
  updateParameterInsight();

  function animate(now) {
    requestAnimationFrame(animate);

    try {
      const deltaMs = now - lastFrameTime;
      lastFrameTime = now;
      const dt = Math.min(0.05, Math.max(0.001, deltaMs / 1000.0));

      // Step physics engine
      physics.step(dt);

      // Update 3D scene & viewports
      renderer.update(physics);

      // Record telemetry buffer
      const state = physics.state;
      const zc = state[0];
      const dzc = state[1];
      const ddzc = physics.ddz_c;
      const zw = state[6]; // FL wheel
      const roadZ = physics.roadZ.FL;
      const spF = physics.springForce.FL;
      const daF = physics.damperForce.FL;

      history.time.push(physics.time);
      history.zc.push(zc * 1000.0); // mm
      history.zw.push(zw * 1000.0); // mm
      history.dzc.push(dzc);        // m/s
      history.ddzc.push(ddzc);      // m/s^2
      history.road.push(roadZ * 1000.0); // mm
      history.spForce.push(spF);
      history.daForce.push(daF);

      if (history.time.length > maxHistory) {
        history.time.shift();
        history.zc.shift();
        history.zw.shift();
        history.dzc.shift();
        history.ddzc.shift();
        history.road.shift();
        history.spForce.shift();
        history.daForce.shift();
      }

      // Update numerical readouts
      const qc = params.getQuarterCarMetrics('FL');
      dom.valZc.textContent = (zc * 1000.0).toFixed(1);
      dom.valZw.textContent = (zw * 1000.0).toFixed(1);
      dom.valStroke.textContent = (physics.suspStroke.FL * 1000.0).toFixed(1);
      dom.valSpForce.textContent = Math.round(spF).toLocaleString();
      dom.valDaForce.textContent = Math.round(daF).toLocaleString();
      dom.valFn.textContent = qc.naturalFreqHz.toFixed(2);
      dom.valZeta.textContent = qc.dampingRatio.toFixed(2);
      dom.valRegime.textContent = qc.regime;
      dom.valAccel.textContent = ddzc.toFixed(2);
      dom.valAccelG.textContent = (ddzc / 9.80665).toFixed(2);
      dom.valSpeed.textContent = params.speedKmh.toFixed(0);

      const pitchDeg = (state[2] * 180 / Math.PI).toFixed(1);
      const rollDeg = (state[4] * 180 / Math.PI).toFixed(1);
      dom.valPitchRoll.textContent = `P: ${pitchDeg}° | R: ${rollDeg}°`;

      dom.hudSpeed.textContent = `V: ${params.speedKmh.toFixed(0)} km/h`;
      dom.hudTime.textContent = `T: ${physics.time.toFixed(1)}s`;

      // Render Charts
      drawChart(chartCanvases.disp, [
        { data: history.zc, color: '#d97736', lineWidth: 2.2 },
        { data: history.zw, color: '#2ecc71', lineWidth: 1.8 },
      ]);

      drawChart(chartCanvases.vel, [
        { data: history.dzc, color: '#3498db', lineWidth: 2.0 },
      ]);

      drawChart(chartCanvases.accel, [
        { data: history.ddzc, color: '#e74c3c', lineWidth: 2.0 },
      ]);

      drawChart(chartCanvases.road, [
        { data: history.road, color: '#9aa5b8', lineWidth: 1.5 },
        { data: history.zc, color: '#d97736', lineWidth: 2.2 },
      ]);
    } catch (loopErr) {
      console.error("Simulation loop error:", loopErr);
    }
  }

  requestAnimationFrame(animate);
});
