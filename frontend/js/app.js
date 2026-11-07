// Spider Mech control app.
//
// Wires the 3D view and every panel control to the robot. One rule
// throughout: the backend owns the truth. The UI sends intent and
// renders telemetry; it never assumes a command took effect.

import { Robot } from './net.js';
import { HexapodView } from './hexapod.js';
import { $, $$, chip, slider, pad, setActive, banner, fmt } from './ui.js';

const robot = new Robot();
const view = new HexapodView($('#scene'));

const JOINTS = ['coxa', 'femur', 'tibia'];
let config = null;
let status = null;
let selectedLeg = 'L1';
let legMode = 'joints';       // whether the Legs tab drives joints or the foot
const sliders = {};           // id -> slider handle
const calibration = { table: {}, leg: 'L1', joint: 'coxa' };

// The femur is shown and typed in servo degrees (0–180, 90 = thigh
// horizontal with the default calibration), so the numbers on screen
// are the numbers the servo gets. Everything sent to the robot stays in
// kinematic degrees; these two convert at the edge of the UI.
const SERVO_UNIT_JOINTS = new Set(['femur']);
function jointCal(leg, joint) {
  return calibration.table[leg]?.[joint] || { offset: 90, direction: -1, min: 0, max: 180 };
}
function toUi(leg, joint, kinematic) {
  if (!SERVO_UNIT_JOINTS.has(joint)) return kinematic;
  const c = jointCal(leg, joint);
  return c.direction * kinematic + c.offset;
}
function fromUi(leg, joint, value) {
  if (!SERVO_UNIT_JOINTS.has(joint)) return value;
  const c = jointCal(leg, joint);
  const [lo, hi] = config.joint_limits[joint];
  return Math.max(lo, Math.min(hi, (value - c.offset) / c.direction));
}
const uiRange = (joint) => (SERVO_UNIT_JOINTS.has(joint) ? [0, 180] : config.joint_limits[joint]);

// ============================================================== boot

robot.addEventListener('link', (e) => {
  const live = e.detail.connected;
  $('#stat-link').classList.toggle('live', live);
  $('#stat-link').querySelector('span').textContent = live ? 'live' : 'offline';
  if (!live) banner('Lost the link to the robot — retrying…', 'error');
  else banner('');
});

robot.addEventListener('config', (e) => applyConfig(e.detail));
robot.addEventListener('telemetry', (e) => applyTelemetry(e.detail));

robot.connect();
setInterval(() => robot.ping(), 700);

// The renderer only needs to redraw when something changed, but
// telemetry arrives at 20Hz and drags at pointer rate, so a rAF loop
// is both simpler and smoother than trying to be clever about it.
function frame() { view.draw(); requestAnimationFrame(frame); }
requestAnimationFrame(frame);

window.addEventListener('resize', () => view.resize());

// ============================================================ config

function applyConfig(cfg) {
  const first = config === null;
  config = cfg;
  view.configure(cfg);
  $('#sim-badge').hidden = !cfg.hardware.all_simulated;

  if (!first) return;   // the panels only need building once

  buildGaits();
  buildMoves();
  buildStances();
  buildAxisModes();
  buildBodySliders();
  buildLegGrid();
  buildLegSliders();
  buildInspector();
  buildCalibration();
  buildHardwareReadout();
  refreshPoses();

  const simulatedImu = (cfg.hardware.backends.imu || '').startsWith('simulated');
  $('#sim-tilt').hidden = !simulatedImu;
  if (simulatedImu) {
    robot.get('/api/sim/tilt').then((tilt) => {
      $('#in-tilt-roll').value = tilt.roll;
      $('#out-tilt-roll').textContent = `${tilt.roll}\u00B0`;
      $('#in-tilt-pitch').value = tilt.pitch;
      $('#out-tilt-pitch').textContent = `${tilt.pitch}\u00B0`;
    }).catch(() => {});
  }
}

// ------------------------------------------------------------ gaits

function buildGaits() {
  const host = $('#gait-chips');
  host.innerHTML = '';
  config.gaits.forEach((g, i) => {
    host.appendChild(chip(g.label.toUpperCase(), {
      title: `${g.description}\n${g.min_support} feet down at minimum`,
      dataset: { gait: g.name, index: String(i + 1) },
      onClick: () => selectGait(g.name),
    }));
  });
  selectGait(config.gaits[0].name, false);
}

function selectGait(name, send = true) {
  if (send) robot.gait(name);
  const g = config.gaits.find((x) => x.name === name);
  setActive($('#gait-chips'), '.chip', (el) => el.dataset.gait === name);
  if (g) {
    $('#gait-note').textContent =
      `${g.description} Minimum ${g.min_support} feet on the ground; duty ${(g.duty * 100).toFixed(0)}%.`;
  }
}

// ------------------------------------------------------------ moves

function buildMoves() {
  const host = $('#move-chips');
  host.innerHTML = '';
  config.moves.forEach((m) => {
    host.appendChild(chip(m.label.toUpperCase(), {
      title: m.description,
      dataset: { move: m.name },
      onClick: () => { robot.move(m.name); $('#move-note').textContent = m.description; },
    }));
  });
  host.appendChild(chip('STOP MOVE', {
    title: 'Cancel the running move',
    dataset: { move: 'stop' },
    onClick: () => robot.move('stop'),
  }));
}

// ----------------------------------------------------------- stances

function buildStances() {
  const host = $('#stance-chips');
  host.innerHTML = '';
  for (const name of Object.keys(config.stances)) {
    host.appendChild(chip(name.toUpperCase(), {
      dataset: { stance: name },
      onClick: () => robot.stance(name),
    }));
  }
}

// -------------------------------------------------------- axis modes

let axisPad = null;

function buildAxisModes() {
  const host = $('#axis-chips');
  host.innerHTML = '';
  config.axis_modes.forEach((mode) => {
    host.appendChild(chip(mode.label, {
      title: mode.description,
      dataset: { axis: mode.name },
      onClick: () => selectAxisMode(mode.name),
    }));
  });

  axisPad = pad($('#axis-pad'), {
    sticky: true,
    onChange: (h, v) => {
      const mode = currentAxisMode();
      robot.axis(mode.name, h, v);
      renderAxisReadout(mode, h, v);
    },
  });
  selectAxisMode(config.axis_modes.find((m) => m.name === 'uv')?.name || config.axis_modes[0].name);
}

const currentAxisMode = () =>
  config.axis_modes.find((m) => m.name === (status?.body.axis_mode)) ||
  config.axis_modes.find((m) => m.name === axisModeName) ||
  config.axis_modes[0];

let axisModeName = 'uv';

function selectAxisMode(name) {
  axisModeName = name;
  robot.send({ type: 'axis', mode: name, h: 0, v: 0 });
  axisPad?.set(0, 0);
  const mode = config.axis_modes.find((m) => m.name === name);
  setActive($('#axis-chips'), '.chip', (el) => el.dataset.axis === name);
  $('#axis-note').textContent = mode.description;
  const padEl = $('#axis-pad');
  padEl.dataset.labelUp = `\u2195 ${describeAxis(mode.v)}`;
  padEl.dataset.labelDown = `\u2194 ${describeAxis(mode.h)}`;
  renderAxisReadout(mode, 0, 0);
}

const FIELD_LABELS = {
  body_x: 'X', body_y: 'Y', body_z: 'Z',
  body_roll: 'ROLL', body_pitch: 'PITCH', body_yaw: 'YAW',
};

const describeAxis = (entries) => entries.map((e) => FIELD_LABELS[e.field]).join(' + ');

function renderAxisReadout(mode, h, v) {
  const line = (axisName, reading, entries) =>
    `<div><span class="k">${axisName}</span> ` +
    entries.map((e) => {
      const value = reading * e.scale;
      return `<span class="v">${FIELD_LABELS[e.field]} ${value >= 0 ? '+' : ''}${value.toFixed(1)}${e.unit === 'deg' ? '°' : 'mm'}</span>`;
    }).join(' · ') + '</div>';
  $('#axis-readout').innerHTML = line('horiz', h, mode.h) + line('vert&nbsp;', v, mode.v);
}

// -------------------------------------------------------- body pose

function buildBodySliders() {
  const host = $('#body-sliders');
  host.innerHTML = '';
  const L = config.limits;
  const defs = [
    ['x', 'Surge X', L.body_translate, 'mm'],
    ['y', 'Sway Y', L.body_translate, 'mm'],
    ['z', 'Heave Z', L.body_z, 'mm'],
    ['roll', 'Roll', L.body_angle, '°'],
    ['pitch', 'Pitch', L.body_angle, '°'],
    ['yaw', 'Yaw', L.body_angle, '°'],
  ];
  for (const [field, label, limit, unit] of defs) {
    sliders[`body_${field}`] = slider(host, {
      label, min: -limit, max: limit, value: 0, step: 0.5,
      format: (v) => `${v > 0 ? '+' : ''}${v.toFixed(1)}${unit}`,
      onInput: (v) => robot.body({ [field]: v }),
    });
  }
}

$('#btn-center').addEventListener('click', async () => {
  await robot.post('/api/body/center').catch(() => {});
  axisPad?.reset();
  for (const field of ['x', 'y', 'z', 'roll', 'pitch', 'yaw']) sliders[`body_${field}`]?.set(0);
});

// ============================================================== drive

const drivePad = pad($('#drive-pad'), {
  onChange: (h, v) => robot.drive({ walk_x: v, walk_y: h }),
});

slidersFromDom();
function slidersFromDom() {
  const bind = (inputId, outputId, format, apply) => {
    const input = $(inputId), output = $(outputId);
    const update = () => {
      const v = Number(input.value);
      output.textContent = format(v);
      apply(v);
    };
    input.addEventListener('input', update);
    return { input, output, format };
  };
  bind('#in-turn', '#out-turn', (v) => String(v), (v) => robot.drive({ turn: v / 100 }));
  bind('#in-speed', '#out-speed', (v) => `${v}%`, (v) => robot.drive({ speed: v / 100 }));
  bind('#in-height', '#out-height', (v) => `${v} mm`, (v) => robot.drive({ step_height: v }));
  bind('#in-stride', '#out-stride', (v) => `${v}%`, (v) => robot.drive({ stride: v / 100 }));
  bind('#in-tilt-roll', '#out-tilt-roll', (v) => `${v}°`, () => sendTilt());
  bind('#in-tilt-pitch', '#out-tilt-pitch', (v) => `${v}°`, () => sendTilt());
}

let tiltTimer = null;
function sendTilt() {
  clearTimeout(tiltTimer);
  tiltTimer = setTimeout(() => {
    robot.post('/api/sim/tilt', {
      roll: Number($('#in-tilt-roll').value),
      pitch: Number($('#in-tilt-pitch').value),
    }).catch(() => {});
  }, 60);
}

// =============================================================== legs

function buildLegGrid() {
  const host = $('#leg-grid');
  host.innerHTML = '';
  for (const leg of config.legs) {
    const el = document.createElement('button');
    el.className = 'leg-btn';
    el.dataset.leg = leg.id;
    el.innerHTML = `${leg.id}<span class="ov"></span>`;
    el.addEventListener('click', () => selectLeg(leg.id));
    host.appendChild(el);
  }
  selectLeg('L1', false);
}

// `highlight` is false only at boot: the Legs tab needs a leg chosen
// so its sliders have something to drive, but nothing should look
// selected in the 3D view until the operator picks one.
function selectLeg(legId, highlight = true) {
  selectedLeg = legId;
  setActive($('#leg-grid'), '.leg-btn', (el) => el.dataset.leg === legId);
  if (highlight) view.select(legId, view.selected?.joint || 'coxa');
  syncLegSliders();
  updateInspector();
}

function buildLegSliders() {
  const jointHost = $('#leg-joint-sliders');
  jointHost.innerHTML = '';
  for (const joint of JOINTS) {
    const [min, max] = uiRange(joint);
    const locked = joint === 'femur' && config.femur_lock != null;
    const name = joint[0].toUpperCase() + joint.slice(1);
    sliders[`joint_${joint}`] = slider(jointHost, {
      label: locked ? `Femur (locked at ${config.femur_lock}°)`
        : SERVO_UNIT_JOINTS.has(joint) ? `${name} (servo °, 90 = horizontal)` : `${name} (°)`,
      min, max, value: toUi(selectedLeg, joint, locked ? config.femur_lock : 0), step: 0.5, numeric: true,
      format: (v) => `${v.toFixed(1)}°`,
      onInput: () => sendLegJoints(),
    });
    if (locked) {
      for (const el of sliders.joint_femur.el.querySelectorAll('input')) el.disabled = true;
      sliders.joint_femur.el.title = 'Every femur is held at this angle (FEMUR_LOCK_DEG in config.py)';
    }
  }

  const footHost = $('#leg-foot-sliders');
  footHost.innerHTML = '';
  const reach = config.geometry.reach_max + config.geometry.coxa;
  const defs = [
    ['x', 'X out', 40, reach],
    ['y', 'Y fwd/back', -reach * 0.6, reach * 0.6],
    ['z', 'Z up/down', -reach, 60],
  ];
  for (const [axis, label, min, max] of defs) {
    sliders[`foot_${axis}`] = slider(footHost, {
      label: `${label} (mm)`, min: Math.round(min), max: Math.round(max), value: 0, step: 1,
      numeric: true,
      format: (v) => `${v.toFixed(0)} mm`,
      onInput: () => sendLegFoot(),
    });
  }
}

function sendLegJoints() {
  legMode = 'joints';
  const angles = {};
  for (const j of JOINTS) angles[j] = fromUi(selectedLeg, j, sliders[`joint_${j}`].get());
  view.angles[selectedLeg] = angles;
  robot.legJoints(selectedLeg, angles);
}

function sendLegFoot() {
  legMode = 'foot';
  const foot = {};
  for (const a of ['x', 'y', 'z']) foot[a] = sliders[`foot_${a}`].get();
  robot.legFoot(selectedLeg, foot);
}

function syncLegSliders() {
  if (!status) return;
  const angles = status.joints[selectedLeg];
  const foot = status.feet[selectedLeg];
  if (angles) for (const j of JOINTS) sliders[`joint_${j}`]?.set(toUi(selectedLeg, j, angles[j]));
  if (foot) for (const a of ['x', 'y', 'z']) sliders[`foot_${a}`]?.set(foot[a]);
  const unreachable = (status.unreachable || []).includes(selectedLeg);
  $('#foot-status').textContent = unreachable
    ? 'Target is outside this leg’s workspace — it has been clamped to the nearest reachable point.'
    : '';
  $('#foot-status').style.color = unreachable ? 'var(--amber)' : '';
}

$('#btn-release-leg').addEventListener('click', () => robot.clearOverride(selectedLeg));

// Put one joint at the same kinematic angle on all six legs, e.g. every
// femur at 0° (thigh horizontal). Hips and knees stay where they are.
async function setJointOnAllLegs(joint, value) {
  try {
    await robot.post('/api/legs/joint', { joint, value });
    banner(joint === 'femur' && value === 0
      ? 'All femurs horizontal (servo 90°) — Release all hands the legs back'
      : `All ${joint}s set to ${value}° — Release all hands the legs back`);
    setTimeout(() => banner(''), 2500);
  } catch (err) {
    banner(`Could not set all ${joint}s: ${err.message}`);
  }
}

$('#btn-leg-zero').addEventListener('click', async () => {
  legMode = 'joints';
  const zero = { coxa: 0, femur: 0, tibia: 0 };
  for (const j of JOINTS) sliders[`joint_${j}`]?.set(toUi(selectedLeg, j, 0));
  view.angles[selectedLeg] = zero;
  try { await robot.post(`/api/leg/${selectedLeg}/joints`, zero); }
  catch (err) { banner(`Could not zero ${selectedLeg}: ${err.message}`); }
});
for (const id of ['#btn-femurs-zero', '#btn-cal-femurs-zero']) {
  $(id).addEventListener('click', () => setJointOnAllLegs('femur', 0));
}
$('#btn-release-all').addEventListener('click', () => robot.clearOverride(null));

// ========================================================= inspector

function buildInspector() {
  const host = $('#joint-rows');
  host.innerHTML = '';
  for (const joint of JOINTS) {
    const [min, max] = uiRange(joint);
    const row = document.createElement('div');
    row.className = 'joint-row';
    row.dataset.joint = joint;
    row.innerHTML = `<span class="jname">${joint}</span>
      <input type="range" min="${min}" max="${max}" step="0.5" value="0">
      <span class="jval">0.0°</span>`;
    const input = row.querySelector('input');
    input.addEventListener('input', () => {
      const leg = view.selected?.leg || selectedLeg;
      view.setJoint(leg, joint, fromUi(leg, joint, Number(input.value)));
      row.querySelector('.jval').textContent = `${Number(input.value).toFixed(1)}°`;
    });
    host.appendChild(row);
  }
}

view.onSelect = (selection) => {
  if (selection?.leg) {
    selectedLeg = selection.leg;
    setActive($('#leg-grid'), '.leg-btn', (el) => el.dataset.leg === selection.leg);
    syncLegSliders();
  }
  updateInspector();
};

view.onJointChange = (legId, angles) => {
  robot.legJoints(legId, angles);
  if (legId === selectedLeg) for (const j of JOINTS) sliders[`joint_${j}`]?.set(toUi(legId, j, angles[j]));
  updateInspector();
};

view.onFootChange = (legId, foot) => {
  robot.legFoot(legId, foot);
  if (legId === selectedLeg) for (const a of ['x', 'y', 'z']) sliders[`foot_${a}`]?.set(foot[a]);
};

function updateInspector() {
  const selection = view.selected;
  const panel = $('#inspector');
  if (!selection || view.mode === 'view') { panel.hidden = true; return; }
  panel.hidden = false;

  const legId = selection.leg;
  const angles = view.angles[legId] || {};
  $('#inspector-title').innerHTML = `${legId} &middot; ${selection.joint}`;

  for (const row of $$('.joint-row', panel)) {
    const joint = row.dataset.joint;
    row.classList.toggle('active', joint === selection.joint);
    const input = row.querySelector('input');
    const shown = toUi(legId, joint, angles[joint] ?? 0);
    if (document.activeElement !== input) input.value = shown;
    row.querySelector('.jval').textContent = `${shown.toFixed(1)}°`;
  }

  const foot = status?.feet?.[legId];
  $('#inspector-foot-pos').textContent = foot
    ? `foot ${foot.x.toFixed(0)}, ${foot.y.toFixed(0)}, ${foot.z.toFixed(0)}`
    : 'foot —';
  const channels = config.legs.find((l) => l.id === legId)?.channels;
  $('#inspector-channel').textContent = channels
    ? `ch ${channels.coxa}/${channels.femur}/${channels.tibia}`
    : 'ch —';
}

$('#inspector-release').addEventListener('click', () => {
  robot.clearOverride(view.selected?.leg || selectedLeg);
});

// ====================================================== view controls

$('#view-presets').addEventListener('click', (e) => {
  const button = e.target.closest('button');
  if (!button) return;
  view.setViewPreset(button.dataset.view);
  setActive($('#view-presets'), 'button', (el) => el === button);
});

$('#view-toggles').addEventListener('click', (e) => {
  const button = e.target.closest('button');
  if (!button) return;
  const map = { grid: 'showGrid', labels: 'showLabels', tripods: 'showTripods' };
  const key = map[button.dataset.toggle];
  view[key] = !view[key];
  button.classList.toggle('on', view[key]);
});

const MODE_HINTS = {
  view: 'Drag to orbit · pinch or scroll to zoom',
  servo: 'Drag a joint handle to turn that servo — the robot follows',
  foot: 'Drag a foot to place it; IK solves the three joints',
};

$('#interact-mode').addEventListener('click', (e) => {
  const button = e.target.closest('button');
  if (!button) return;
  const mode = button.dataset.mode;
  view.setMode(mode);
  setActive($('#interact-mode'), 'button', (el) => el === button);
  $('#mode-hint').textContent = MODE_HINTS[mode];
  updateInspector();
});

// ========================================================== behaviour

$('#behaviour-toggles').addEventListener('click', (e) => {
  const button = e.target.closest('button[data-flag]');
  if (!button) return;
  const flag = button.dataset.flag;
  // Flip the button now, so a quick second tap toggles back instead of
  // repeating the same command before telemetry catches up.
  const next = !button.classList.contains('on');
  button.classList.toggle('on', next);
  robot.behaviour({ [flag]: next });
});

$('#btn-beep').addEventListener('click', () => robot.beep('beep'));
$('#btn-level-now').addEventListener('click', () => robot.post('/api/imu/level').catch(() => {}));

$('#estop').addEventListener('click', () => {
  if (status?.behaviour.estop) robot.clearEstop();
  else robot.estop();
});

// ======================================================== calibration
//
// While the Setup tab is open, the selected leg is held with all three
// joints at kinematic 0, so servo = direction * 0 + offset = offset and
// the Offset slider sweeps the real servo across its full 0–180°.
//
// Slider drags are smoothed: the local value is the truth while the
// operator is dragging, changes go to the robot at most every 60 ms
// (always ending with the final value), and replies or telemetry never
// move the slider that is being dragged.

const FIELD_KEY = { offset: 'offset', min_deg: 'min', max_deg: 'max' };
const CAL_SLIDERS = [
  ['#in-cal-offset', '#out-cal-offset', 'offset'],
  ['#in-cal-min',    '#out-cal-min',    'min_deg'],
  ['#in-cal-max',    '#out-cal-max',    'max_deg'],
];
const calEdit = { field: null, timer: null, pending: null, inflight: false, releaseTimer: null };
let lastHoldSent = 0;

// Only a page the operator is actually looking at may pin a leg: a
// forgotten background tab (or another device) left on Setup would
// otherwise keep forcing the calibration leg back to neutral forever.
const setupVisible = () => !$('.tab[data-tab="setup"]').hidden
  && document.visibilityState === 'visible';

function buildCalibration() {
  const legSelect = $('#cal-leg'), jointSelect = $('#cal-joint');
  legSelect.innerHTML = config.legs.map((l) => `<option>${l.id}</option>`).join('');
  jointSelect.innerHTML = JOINTS.map((j) => `<option>${j}</option>`).join('');
  legSelect.addEventListener('change', () => {
    const previousLeg = calibration.leg;
    calibration.leg = legSelect.value;
    if (previousLeg && previousLeg !== calibration.leg) {
      robot.del(`/api/leg/${previousLeg}/override`).catch(() => {});
    }
    holdLegAtZero(calibration.leg);
    syncCalibration();
    refreshCalibration();
  });
  jointSelect.addEventListener('change', () => {
    calibration.joint = jointSelect.value;
    syncCalibration();
    refreshCalibration();
  });

  for (const [sliderId, outId, field] of CAL_SLIDERS) {
    const el = $(sliderId);
    const begin = () => {
      calEdit.field = field;
      clearTimeout(calEdit.releaseTimer);
      holdLegAtZero(calibration.leg);
    };
    const finish = () => {
      flushCalibration();
      clearTimeout(calEdit.releaseTimer);
      // Let the final reply land before telemetry may move this slider again.
      calEdit.releaseTimer = setTimeout(() => { calEdit.field = null; syncCalibration(); }, 500);
    };
    el.addEventListener('pointerdown', begin);
    el.addEventListener('keydown', begin);
    el.addEventListener('pointerup', finish);
    el.addEventListener('pointercancel', finish);
    el.addEventListener('keyup', finish);
    el.addEventListener('change', finish);
    el.addEventListener('input', () => {
      if (calEdit.field !== field) begin();
      const value = Number(el.value);
      $(outId).textContent = `${value}°`;
      const entry = calibration.table[calibration.leg]?.[calibration.joint];
      if (entry) entry[FIELD_KEY[field]] = value;
      queueCalibration({ [field]: value });
      renderCalibrationReadout();
    });
  }

  $('#btn-cal-flip').addEventListener('click', async () => {
    const leg = calibration.leg, joint = calibration.joint;
    const entry = calibration.table[leg]?.[joint];
    const before = entry ? { ...entry } : null;
    await sendCalibration({ direction: (entry?.direction ?? 1) * -1 });
    const after = calibration.table[leg]?.[joint];
    if (!after) { banner(`Flip failed for ${leg} ${joint}`); return; }
    banner(`${leg} ${joint}: direction ${before?.direction > 0 ? '+1' : '−1'} → ${after.direction > 0 ? '+1' : '−1'}, `
      + `offset ${before?.offset ?? '?'}° → ${after.offset}° — click Test direction to check, Save to keep`);
    setTimeout(() => banner(''), 4000);
  });

  // Move the selected joint +30° (kinematic) for a moment, then back to
  // neutral, so the operator can compare the real joint with the 3D model.
  let testing = false;
  $('#btn-cal-test').addEventListener('click', async () => {
    if (testing) return;
    testing = true;
    const leg = calibration.leg, joint = calibration.joint;
    const [lo, hi] = config.joint_limits[joint];
    const step = Math.max(lo, Math.min(hi, 30));
    const angles = { coxa: 0, femur: 0, tibia: 0, [joint]: step };
    banner(`${leg} ${joint} moving ${step}° — the real joint should move like the 3D model. Opposite way? Click Flip direction.`);
    try {
      await robot.post(`/api/leg/${leg}/joints`, angles);
      lastHoldSent = performance.now() + 1500;        // keep the self-healing hold from undoing the test
      await new Promise((r) => setTimeout(r, 1500));
    } catch (err) {
      banner(`Test failed for ${leg} ${joint}: ${err.message}`);
    } finally {
      await holdLegAtZero(leg);
      testing = false;
      setTimeout(() => banner(''), 2500);
    }
  });
  $('#btn-cal-widen').addEventListener('click', () => {
    sendCalibration({ min_deg: 0, max_deg: 180 });
  });
  $('#btn-cal-save').addEventListener('click', async () => {
    await robot.post('/api/calibration/save').catch(() => {});
    banner('Calibration saved to calibration.json');
    setTimeout(() => banner(''), 2200);
  });
  $('#btn-cal-fit-all').addEventListener('click', async () => {
    if (!window.confirm('Replace the offset of all 18 motors with the one centred in its servo min–max? '
      + 'Any offset you trimmed by hand is overwritten (nothing is saved until you click Save).')) return;
    try {
      const data = await robot.post('/api/calibration/fit', {});
      calibration.table = data.calibration;
      syncCalibration();
      banner('Offsets fitted to min–max for all 18 motors — click Save to keep them');
      setTimeout(() => banner(''), 3000);
    } catch (err) {
      banner(`Could not fit offsets: ${err.message}`);
    }
  });
  $('#btn-cal-reset').addEventListener('click', async () => {
    if (!window.confirm('Reset all 18 motors to the default calibration from config.py? '
      + 'Your tuned values stay in calibration.json until you click Save; restart the server without saving to get them back.')) return;
    const data = await robot.post('/api/calibration/reset').catch(() => null);
    if (data) { calibration.table = data.calibration; syncCalibration(); }
  });

  loadCalibration();
}

// Release the calibration hold as soon as this page is hidden or
// closed, and take it again when the operator comes back to Setup.
function releaseCalibrationHold() {
  if (!calibration.leg) return;
  fetch(`${robot.origin}/api/leg/${calibration.leg}/override`,
        { method: 'DELETE', keepalive: true }).catch(() => {});
}
document.addEventListener('visibilitychange', () => {
  if (document.visibilityState === 'hidden') {
    if (!$('.tab[data-tab="setup"]').hidden) releaseCalibrationHold();
  } else if (setupVisible()) {
    holdLegAtZero(calibration.leg);
  }
});
window.addEventListener('pagehide', () => {
  if (!$('.tab[data-tab="setup"]').hidden) releaseCalibrationHold();
});

async function holdLegAtZero(legId) {
  lastHoldSent = performance.now();
  try {
    await robot.post(`/api/leg/${legId}/joints`, { coxa: 0, femur: 0, tibia: 0 });
  } catch (err) {
    banner(`Hold at neutral failed for ${legId}: ${err.message}`);
  }
}

// Self-healing hold: if anything else (the Legs tab, a 3D drag, a
// stance change) moved the calibrated leg, put it back at neutral.
function ensureCalibrationHold() {
  if (!setupVisible() || !status) return;
  const held = status.override?.manual_joints?.[calibration.leg];
  const atZero = held && JOINTS.every((j) => Math.abs(held[j] ?? 1) < 1e-6);
  if (!atZero && performance.now() - lastHoldSent > 1000) holdLegAtZero(calibration.leg);
}

// Re-read one joint's calibration from the robot, so the panel never
// shows values that were changed elsewhere (another browser, a fit).
async function refreshCalibration() {
  const leg = calibration.leg, joint = calibration.joint;
  const data = await robot.get('/api/calibration').catch(() => null);
  if (!data || calEdit.field || calEdit.pending) return;
  if (leg !== calibration.leg || joint !== calibration.joint) return;
  calibration.table = data.calibration;
  syncCalibration();
}

async function loadCalibration() {
  const data = await robot.get('/api/calibration').catch(() => null);
  if (!data) return;
  calibration.table = data.calibration;
  if (setupVisible()) holdLegAtZero(calibration.leg);
  syncCalibration();
}

// Throttle: at most one request in flight, the newest value always wins.
function queueCalibration(changes) {
  calEdit.pending = { ...(calEdit.pending || {}), ...changes };
  if (!calEdit.timer) calEdit.timer = setTimeout(flushCalibration, 60);
}

async function flushCalibration() {
  clearTimeout(calEdit.timer);
  calEdit.timer = null;
  if (!calEdit.pending) return;
  if (calEdit.inflight) { calEdit.timer = setTimeout(flushCalibration, 30); return; }
  const changes = calEdit.pending;
  calEdit.pending = null;
  calEdit.inflight = true;
  await sendCalibration(changes);
  calEdit.inflight = false;
}

async function sendCalibration(changes) {
  const leg = calibration.leg, joint = calibration.joint;
  const data = await robot.post('/api/calibration', { leg, joint, ...changes }).catch(() => null);
  if (!data || !calibration.table[leg]) return;
  const entry = { ...data.calibration };
  // Never let a reply overwrite the value under the operator's finger,
  // or a value that has been changed again since this request left.
  const local = calibration.table[leg][joint] || {};
  for (const [field, key] of Object.entries(FIELD_KEY)) {
    if (calEdit.field === field || (calEdit.pending && field in calEdit.pending)) entry[key] = local[key];
  }
  calibration.table[leg][joint] = entry;
  syncCalibration();
}

function syncCalibration() {
  const entry = calibration.table[calibration.leg]?.[calibration.joint];
  if (!entry) return;
  for (const [sliderId, outId, field] of CAL_SLIDERS) {
    if (calEdit.field === field) continue;           // never fight the operator
    const value = entry[FIELD_KEY[field]];
    $(sliderId).value = value;
    $(outId).textContent = `${value}°`;
  }
  renderCalibrationReadout();
}

function renderCalibrationReadout() {
  const entry = calibration.table[calibration.leg]?.[calibration.joint];
  if (!entry) return;
  const channel = config.legs.find((l) => l.id === calibration.leg)?.channels[calibration.joint];
  const live = status?.joints?.[calibration.leg]?.[calibration.joint];
  const servo = live === undefined ? null
    : Math.max(entry.min, Math.min(entry.max, entry.direction * live + entry.offset));
  $('#cal-readout').innerHTML =
    `<div><span class="k">channel</span> <span class="v">${channel}</span> ` +
    `<span class="k">direction</span> <span class="v">${entry.direction > 0 ? '+1' : '−1'}</span> ` +
    `<span class="k">range</span> <span class="v">${entry.min.toFixed(0)}°&ndash;${entry.max.toFixed(0)}°</span></div>` +
    (servo === null ? '' :
      `<div><span class="k">kinematic</span> <span class="v">${live.toFixed(1)}°</span> ` +
      `<span class="k">→ servo</span> <span class="v">${servo.toFixed(1)}°</span></div>`) +
    calibrationWarnings();
}

// Anything that stops the controller applying the hold freezes the
// kinematic reading, so say which one it is instead of failing silently.
function calibrationWarnings() {
  if (!status) return '';
  const b = status.behaviour || {};
  const held = status.override?.manual_joints?.[calibration.leg];
  const problems = [];
  if (b.estop) problems.push('STOP is latched — press STOP again to clear');
  if (b.torque === false) problems.push('TORQUE is off — turn it on in Behaviour');
  if (b.hold_pos) problems.push('HOLD POS is on — turn it off, it freezes every joint');
  const atZero = held && JOINTS.every((j) => Math.abs(held[j] ?? 1) < 1e-6);
  if (!atZero) problems.push(`${calibration.leg} is not held at neutral yet — re-holding it`);
  const fit = rangeFit();
  if (fit) problems.push(fit);
  return problems.map((p) => `<div class="warn">${p}</div>`).join('');
}

// Checks whether the joint's whole kinematic range (what the Legs tab
// slider and the 3D model sweep) lands inside servo min..max. If not,
// part of the slider is clamped and the real joint moves less than the model.
function rangeFit() {
  const entry = calibration.table[calibration.leg]?.[calibration.joint];
  const limits = config.joint_limits?.[calibration.joint];
  if (!entry || !limits) return '';
  const [lo, hi] = limits;
  const a = entry.direction * lo, b = entry.direction * hi;
  const mappedLow = Math.min(a, b) + entry.offset;
  const mappedHigh = Math.max(a, b) + entry.offset;
  if (mappedLow >= entry.min - 1e-6 && mappedHigh <= entry.max + 1e-6) return '';
  const usable = Math.max(0, Math.min(mappedHigh, entry.max) - Math.max(mappedLow, entry.min));
  const okLow = entry.min - Math.min(a, b);
  const okHigh = entry.max - Math.max(a, b);
  const advice = okLow <= okHigh + 1e-6
    ? (Math.abs(okHigh - okLow) < 1e-6 ? `set offset to ${okLow.toFixed(0)}°`
      : `set offset between ${okLow.toFixed(0)}° and ${okHigh.toFixed(0)}°`)
    : `the ${(hi - lo).toFixed(0)}° kinematic range is wider than servo min–max; widen min/max or narrow JOINT_LIMITS_DEG`;
  return `Only ${usable.toFixed(0)}° of this joint's ${(hi - lo).toFixed(0)}° range reaches the servo — ${advice}.`;
}

function buildHardwareReadout() {
  const hw = config.hardware;
  const rows = Object.entries(hw.backends)
    .map(([k, v]) => `<div><span class="k">${k}</span> <span class="${v.startsWith('simulated') ? 'v' : 'warn'}">${v}</span></div>`)
    .join('');
  const warnings = hw.warnings.map((w) => `<div class="warn">${w}</div>`).join('');
  $('#hardware-readout').innerHTML = rows + warnings +
    `<div><span class="k">tick</span> <span class="v">${config.tick_rate} Hz</span></div>`;
}

// ============================================================== poses

async function refreshPoses() {
  const data = await robot.get('/api/pose').catch(() => null);
  const host = $('#pose-chips');
  host.innerHTML = '';
  for (const name of data?.poses || []) {
    const el = chip(name, { onClick: () => robot.post(`/api/pose/load/${encodeURIComponent(name)}`) });
    el.addEventListener('contextmenu', async (e) => {
      e.preventDefault();
      await robot.del(`/api/pose/${encodeURIComponent(name)}`).catch(() => {});
      refreshPoses();
    });
    el.title = 'Click to load · right-click to delete';
    host.appendChild(el);
  }
}

$('#btn-save-pose').addEventListener('click', async () => {
  const input = $('#pose-name');
  const name = input.value.trim();
  if (!name) return;
  await robot.post('/api/pose/save', { name }).catch(() => {});
  input.value = '';
  refreshPoses();
});

// =========================================================== telemetry

// The Drive sliders start at the HTML defaults; on the first telemetry
// (and after every reconnect) show what the robot is really using, so a
// slider never claims e.g. 40 mm while the robot steps 30 mm.
let driveSlidersSynced = false;
robot.addEventListener('link', (e) => { if (!e.detail.connected) driveSlidersSynced = false; });

function syncDriveSliders(gait) {
  for (const [id, out, value, text] of [
    ['#in-turn', '#out-turn', Math.round(gait.turn * 100), (v) => String(v)],
    ['#in-speed', '#out-speed', Math.round(gait.speed * 100), (v) => `${v}%`],
    ['#in-height', '#out-height', Math.round(gait.step_height), (v) => `${v} mm`],
    ['#in-stride', '#out-stride', Math.round(gait.stride * 100), (v) => `${v}%`],
  ]) {
    if (document.activeElement === $(id)) continue;
    $(id).value = value;
    $(out).textContent = text(Number($(id).value));
  }
}

function applyTelemetry(snapshot) {
  status = snapshot;
  if (!driveSlidersSynced) { syncDriveSliders(snapshot.gait); driveSlidersSynced = true; }
  view.applyTelemetry(snapshot);

  const power = snapshot.power;
  $('#stat-volts').innerHTML =
    `${power.voltage?.toFixed(1) ?? '--'}<em>V</em>`;
  $('#stat-amps').innerHTML =
    `${power.current?.toFixed(1) ?? '--'}<em>A</em>`;
  $('#stat-hz').innerHTML = `${Math.round(snapshot.health.loop_hz)}<em>Hz</em>`;
  $('#stat-gait').textContent = snapshot.gait.walking
    ? `${snapshot.gait.name} ${Math.round(snapshot.gait.phase * 100)}%`
    : snapshot.gait.name;

  setActive($('#gait-chips'), '.chip', (el) => el.dataset.gait === snapshot.gait.name);
  setActive($('#stance-chips'), '.chip', (el) => el.dataset.stance === snapshot.stance.name);
  setActive($('#axis-chips'), '.chip', (el) => el.dataset.axis === snapshot.body.axis_mode);
  setActive($('#move-chips'), '.chip', (el) => el.dataset.move === snapshot.move.active);

  for (const button of $$('#behaviour-toggles button[data-flag]')) {
    button.classList.toggle('on', !!snapshot.behaviour[button.dataset.flag]);
  }
  $('#estop').classList.toggle('engaged', snapshot.behaviour.estop);
  $('#estop').textContent = snapshot.behaviour.estop ? 'RESET' : 'STOP';

  const notes = [];
  if (snapshot.behaviour.estop) notes.push('E-stop engaged — press RESET to re-arm.');
  if (!snapshot.behaviour.torque) notes.push('Torque released — the servos are limp.');
  if (snapshot.behaviour.hold_pos) notes.push('Holding position — motion commands are ignored.');
  $('#behaviour-note').textContent = notes.join(' ');

  const imu = snapshot.imu;
  $('#imu-readout').innerHTML =
    `<div><span class="k">roll</span> <span class="v">${fmtNum(imu.roll)}°</span> ` +
    `<span class="k">pitch</span> <span class="v">${fmtNum(imu.pitch)}°</span></div>` +
    `<div><span class="k">correction</span> <span class="v">${fmtNum(imu.level_roll)}° / ${fmtNum(imu.level_pitch)}°</span></div>`;

  for (const button of $$('#leg-grid .leg-btn')) {
    const legId = button.dataset.leg;
    const override = snapshot.override.manual_joints[legId] ? 'servo'
      : snapshot.override.manual_feet[legId] ? 'foot' : '';
    button.querySelector('.ov').textContent = override;
  }

  if (!$('.tab[data-tab="legs"]').hidden) syncLegSliders();
  if (setupVisible()) { ensureCalibrationHold(); syncCalibration(); }
  updateInspector();

  const body = snapshot.body;
  for (const [field, value] of Object.entries({
    x: body.x, y: body.y, z: body.z, roll: body.roll, pitch: body.pitch, yaw: body.yaw,
  })) sliders[`body_${field}`]?.set(value);
}

const fmtNum = (v) => (v === null || v === undefined ? '--' : v.toFixed(1));

// ================================================================ tabs

$('#tabs').addEventListener('click', (e) => {
  const button = e.target.closest('button');
  if (!button) return;
  const tab = button.dataset.tab;
  setActive($('#tabs'), 'button', (el) => el === button);
  for (const section of $$('.tab')) section.hidden = section.dataset.tab !== tab;
  if (tab === 'legs') view.select(selectedLeg, view.selected?.joint || 'coxa');
  // Opening Setup auto-pins the selected leg at kinematic 0 so the
  // Offset slider directly commands the servo. Leaving Setup releases
  // the override so the leg returns to walking / stance.
  if (tab === 'setup') {
    holdLegAtZero(calibration.leg);
  } else if (tab !== 'setup' && calibration.leg) {
    robot.del(`/api/leg/${calibration.leg}/override`).catch(() => {});
  }
});

// ============================================================ keyboard

const held = new Set();

window.addEventListener('keydown', (e) => {
  if (e.target.matches('input, select, textarea')) return;

  if (e.code === 'Space') {
    e.preventDefault();
    robot.estop();
    return;
  }
  const gaitIndex = Number(e.key);
  if (gaitIndex >= 1 && gaitIndex <= (config?.gaits.length || 0)) {
    selectGait(config.gaits[gaitIndex - 1].name);
    return;
  }
  if ('wasdqe'.includes(e.key.toLowerCase())) {
    held.add(e.key.toLowerCase());
    sendKeyboardDrive();
  }
});

window.addEventListener('keyup', (e) => {
  const key = e.key.toLowerCase();
  if (held.delete(key)) sendKeyboardDrive();
});
window.addEventListener('blur', () => { held.clear(); sendKeyboardDrive(); });

// A deliberate debug hook. Open the browser console on the robot and
// you can drive it directly - `spiderMech.robot.gait('wave')`, or read
// `spiderMech.view.angles` to see exactly what the 3D model believes.
// Useful on a machine with no keyboard attached, and it is what the
// end-to-end tests drive.
window.spiderMech = { robot, view, get status() { return status; }, get config() { return config; } };

function sendKeyboardDrive() {
  const walk_x = (held.has('w') ? 1 : 0) - (held.has('s') ? 1 : 0);
  const walk_y = (held.has('a') ? 1 : 0) - (held.has('d') ? 1 : 0);
  const turn = (held.has('q') ? 1 : 0) - (held.has('e') ? 1 : 0);
  robot.drive({ walk_x, walk_y, turn });
  drivePad.set(walk_y, walk_x);
}
