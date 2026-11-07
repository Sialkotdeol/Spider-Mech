// The hexapod scene: builds the 3D model from live joint angles, and
// turns pointer gestures into servo commands.
//
// The world frame here is the "feet planted" frame. Foot targets live
// in it directly, and the body plate is drawn at the commanded body
// pose - so tilting the body makes the body move while the feet stay
// put on screen, exactly as the robot behaves.

import { Renderer } from './render3d.js';
import { v3, mat3, rad, deg, clamp, rotateAbout } from './math3d.js';
import * as kin from './kinematics.js';

const COLORS = {
  background: '#12151a',
  grid: '#232932',
  gridStrong: '#2d3540',
  body: '#3d4756',
  bodyTop: '#4a5567',
  legA: '#59647a',
  legB: '#4d5768',
  selected: '#e0a03d',
  dimmed: '#39414e',
  foot: '#4fb6a8',
  footAir: '#5d6675',
  handle: '#8d99ac',
  handleHot: '#e0a03d',
  text: '#8b93a1',
  textBright: '#e8eaee',
  warn: '#d64545',
};

const JOINT_ORDER = ['coxa', 'femur', 'tibia'];

export class HexapodView {
  constructor(canvas) {
    this.renderer = new Renderer(canvas);
    this.canvas = canvas;

    this.mode = 'view';           // 'view' | 'servo' | 'foot'
    this.selected = null;         // { leg, joint }
    this.hover = null;
    this.showTripods = false;
    this.showLabels = true;
    this.showGrid = true;

    // Live model, updated from telemetry or by dragging.
    this.angles = {};             // legId -> {coxa, femur, tibia}
    this.body = { x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0 };
    this.grounded = new Set();
    this.unreachable = new Set();
    this.overrides = {};          // legId -> 'joints' | 'foot' | undefined

    this._handles = [];           // screen positions, rebuilt each frame
    this._drag = null;
    this._groundZ = -120;
    this._ready = false;

    this.onJointChange = null;    // (legId, angles) => void
    this.onFootChange = null;     // (legId, {x,y,z}) => void
    this.onSelect = null;         // ({leg, joint}) => void

    this._bindPointer();
  }

  configure(cfg) {
    kin.configure(cfg);
    for (const leg of cfg.legs) {
      if (!this.angles[leg.id]) this.angles[leg.id] = { coxa: 0, femur: 0, tibia: 90 };
    }
    this._ready = true;
    this.resize();
  }

  resize() { this.renderer.resize(); }

  // Adopt a telemetry frame, unless the operator is mid-drag on that leg.
  applyTelemetry(snapshot) {
    if (!this._ready) return;
    const draggingLeg = this._drag?.leg;
    for (const [legId, a] of Object.entries(snapshot.joints || {})) {
      if (legId === draggingLeg) continue;
      this.angles[legId] = { coxa: a.coxa, femur: a.femur, tibia: a.tibia };
    }
    if (!this._drag) {
      this.body = {
        x: snapshot.body.x, y: snapshot.body.y, z: snapshot.body.z,
        roll: snapshot.body.roll + (snapshot.imu?.level_roll || 0),
        pitch: snapshot.body.pitch + (snapshot.imu?.level_pitch || 0),
        yaw: snapshot.body.yaw,
      };
    }
    this.grounded = new Set(snapshot.gait?.grounded || []);
    this.unreachable = new Set(snapshot.unreachable || []);
    this.overrides = {};
    for (const id of Object.keys(snapshot.override?.manual_joints || {})) this.overrides[id] = 'joints';
    for (const id of Object.keys(snapshot.override?.manual_feet || {})) this.overrides[id] = 'foot';
  }

  // ------------------------------------------------------ scene maths

  bodyMatrix() {
    return mat3.fromEulerDeg(this.body.roll, this.body.pitch, this.body.yaw);
  }

  bodyTranslation() {
    return [this.body.x, this.body.y, this.body.z];
  }

  // Body-frame point -> world.
  toWorld(p) {
    return v3.add(mat3.apply(this.bodyMatrix(), p), this.bodyTranslation());
  }

  // Every joint of one leg, in world space.
  legWorldChain(legId) {
    const leg = kin.legById(legId);
    const local = kin.legChain(this.angles[legId] || { coxa: 0, femur: 0, tibia: 90 });
    const legYaw = mat3.rotZ(rad(leg.mount_angle));
    return local.map((p) => this.toWorld(v3.add(mat3.apply(legYaw, p), leg.mount_point)));
  }

  // The world-space axis a given servo rotates about.
  jointAxis(legId, joint) {
    const leg = kin.legById(legId);
    const R = this.bodyMatrix();
    if (joint === 'coxa') return v3.normalize(mat3.apply(R, [0, 0, 1]));
    // Femur and knee both pivot about the horizontal axis perpendicular
    // to the leg's own vertical plane.
    const a = rad(leg.mount_angle + (this.angles[legId]?.coxa || 0));
    return v3.normalize(mat3.apply(R, [-Math.sin(a), Math.cos(a), 0]));
  }

  // ---------------------------------------------------------- drawing

  draw() {
    if (!this._ready) return;
    const r = this.renderer;
    r.beginFrame();
    this._handles.length = 0;

    const chains = {};
    let lowest = Infinity;
    for (const leg of kin.legs()) {
      chains[leg.id] = this.legWorldChain(leg.id);
      lowest = Math.min(lowest, chains[leg.id][3][2]);
    }
    // Ease the ground plane so it does not jitter while walking.
    this._groundZ += (lowest - this._groundZ) * 0.25;

    if (this.showGrid) this._drawGround();
    for (const leg of kin.legs()) this._drawFootShadow(chains[leg.id][3]);
    this._drawBody();
    for (const leg of kin.legs()) this._drawLeg(leg, chains[leg.id]);

    r.render(COLORS.background);
  }

  _drawGround() {
    const r = this.renderer;
    const z = this._groundZ;
    const extent = 420, step = 60;
    for (let i = -extent; i <= extent; i += step) {
      const strong = i === 0;
      const color = strong ? COLORS.gridStrong : COLORS.grid;
      r.addLine([i, -extent, z], [i, extent, z], color, strong ? 1.4 : 1, 0.75);
      r.addLine([-extent, i, z], [extent, i, z], color, strong ? 1.4 : 1, 0.75);
    }
  }

  _drawFootShadow(foot) {
    this.renderer.addDisc([foot[0], foot[1], this._groundZ + 0.6], 22, '#0c0e12', 0.5, 14);
  }

  _drawBody() {
    const r = this.renderer;
    const R = this.bodyMatrix();
    const t = this.bodyTranslation();
    const radius = kin.geometry().body_radius;
    const half = kin.geometry().body_thickness / 2;

    // A hexagonal plate, built in the body frame then transformed, so
    // it tilts with the commanded pose.
    const top = [], bottom = [];
    for (let i = 0; i < 6; i++) {
      const a = (i / 6) * Math.PI * 2;
      const p = [Math.cos(a) * radius * 1.05, Math.sin(a) * radius * 1.05, 0];
      top.push(v3.add(mat3.apply(R, [p[0], p[1], half]), t));
      bottom.push(v3.add(mat3.apply(R, [p[0], p[1], -half]), t));
    }
    for (let i = 0; i < 6; i++) {
      const j = (i + 1) % 6;
      r.addFace([bottom[i], bottom[j], top[j], top[i]], COLORS.body, { stroke: '#20252d' });
    }
    r.addFace(top, COLORS.bodyTop, { stroke: '#20252d' });
    r.addFace(bottom.slice().reverse(), COLORS.body);

    // Heading arrow, so "forward" is never ambiguous when orbiting.
    const nose = v3.add(mat3.apply(R, [radius * 1.28, 0, half + 1]), t);
    const left = v3.add(mat3.apply(R, [radius * 0.72, 26, half + 1]), t);
    const right = v3.add(mat3.apply(R, [radius * 0.72, -26, half + 1]), t);
    r.addFace([nose, left, right], COLORS.selected, { flat: true, alpha: 0.9 });
  }

  _drawLeg(leg, chain) {
    const r = this.renderer;
    const legId = leg.id;
    const isSelected = this.selected?.leg === legId;
    // Only fade the other legs while a joint is actually grabbable -
    // in VIEW mode the whole robot should read at full contrast.
    const dim = this.mode !== 'view' && this.selected && !isSelected;

    let base = this.showTripods
      ? (leg.tripod === 'A' ? COLORS.legA : COLORS.legB)
      : COLORS.legA;
    if (dim) base = COLORS.dimmed;
    if (isSelected) base = COLORS.selected;
    if (this.unreachable.has(legId)) base = COLORS.warn;

    const radii = [11, 9.5, 7.5];
    for (let i = 0; i < 3; i++) {
      r.addSegment(chain[i], chain[i + 1], radii[i], base, 6, {
        taper: i === 2 ? 0.45 : 0.9,
        stroke: isSelected ? '#f0c47a' : undefined,
        alpha: dim ? 0.85 : 1,
      });
    }

    // Foot contact marker.
    const foot = chain[3];
    const down = this.grounded.has(legId);
    r.addHandle(foot, down ? 6 : 4.5, down ? COLORS.foot : COLORS.footAir, {
      alpha: dim ? 0.6 : 1,
    });

    // Draggable joint handles.
    for (let i = 0; i < 3; i++) {
      const joint = JOINT_ORDER[i];
      const active = this.selected?.leg === legId && this.selected?.joint === joint;
      const hovered = this.hover?.leg === legId && this.hover?.joint === joint;
      const interactive = this.mode === 'servo';
      const screen = r.addHandle(chain[i], active ? 7.5 : hovered ? 6.5 : 5,
        active || hovered ? COLORS.handleHot : COLORS.handle, {
          ring: active ? '#f0c47a' : undefined,
          onTop: true,
          alpha: interactive ? (dim ? 0.5 : 1) : 0.35,
        });
      if (screen && interactive) {
        this._handles.push({ leg: legId, joint, x: screen.x, y: screen.y });
      }
    }

    if (this.mode === 'foot') {
      const screen = r.addHandle(foot, 8, COLORS.handleHot, { onTop: true, alpha: 0.9 });
      if (screen) this._handles.push({ leg: legId, joint: 'foot', x: screen.x, y: screen.y });
    }

    // Show which way the selected servo turns. Without this the drag
    // direction has to be guessed, and dragging across the rotation
    // plane does almost nothing - correctly, but confusingly.
    if (isSelected && this.mode === 'servo' && this.selected.joint !== 'foot') {
      this._drawRotationArc(legId, this.selected.joint, chain);
    }

    if (this.showLabels) {
      const label = this.overrides[legId] ? `${legId}*` : legId;
      r.addText(chain[1], label, isSelected ? COLORS.textBright : COLORS.text,
        { onTop: true, offsetY: -18 });
    }
  }

  _drawRotationArc(legId, joint, chain) {
    const r = this.renderer;
    const index = JOINT_ORDER.indexOf(joint);
    const pivot = chain[index];
    const axis = this.jointAxis(legId, joint);

    // Point the arc along the limb, in the plane the joint sweeps.
    const arm = v3.sub(chain[3], pivot);
    let ref = v3.sub(arm, v3.scale(axis, v3.dot(arm, axis)));
    if (v3.length(ref) < 1e-6) return;
    ref = v3.scale(v3.normalize(ref), 52);

    const span = rad(52);
    const steps = 16;
    let previous = null;
    for (let i = 0; i <= steps; i++) {
      const angle = -span + (2 * span * i) / steps;
      const point = v3.add(pivot, rotateAbout(ref, axis, angle));
      if (previous) r.addLine(previous, point, COLORS.handleHot, 2, 0.85);
      previous = point;
    }
    // Arrowheads, so the sweep reads as a direction and not a bracket.
    for (const end of [-span, span]) {
      const tip = v3.add(pivot, rotateAbout(ref, axis, end));
      const inner = v3.add(pivot, rotateAbout(v3.scale(ref, 0.82), axis, end * 0.82));
      r.addLine(tip, inner, COLORS.handleHot, 2, 0.85);
    }
  }

  // ------------------------------------------------------ interaction

  _bindPointer() {
    const c = this.canvas;
    const pointers = new Map();
    let pinchDistance = 0;

    const local = (e) => {
      const rect = c.getBoundingClientRect();
      return { x: e.clientX - rect.left, y: e.clientY - rect.top };
    };

    c.addEventListener('pointerdown', (e) => {
      c.setPointerCapture(e.pointerId);
      const p = local(e);
      pointers.set(e.pointerId, p);

      if (pointers.size === 2) {
        const [a, b] = [...pointers.values()];
        pinchDistance = Math.hypot(a.x - b.x, a.y - b.y);
        this._drag = null;
        return;
      }

      const hit = this._pick(p.x, p.y);
      if (hit && this.mode !== 'view') {
        this._beginJointDrag(hit, p);
        this.selected = { leg: hit.leg, joint: hit.joint };
        this.onSelect?.(this.selected);
      } else {
        this._drag = { kind: 'orbit', last: p };
      }
    });

    c.addEventListener('pointermove', (e) => {
      const p = local(e);
      if (pointers.has(e.pointerId)) pointers.set(e.pointerId, p);

      if (pointers.size === 2) {
        const [a, b] = [...pointers.values()];
        const d = Math.hypot(a.x - b.x, a.y - b.y);
        if (pinchDistance > 0 && d > 0) this.renderer.camera.zoom(pinchDistance / d);
        pinchDistance = d;
        return;
      }

      if (!this._drag) {
        const hit = this.mode === 'view' ? null : this._pick(p.x, p.y);
        const changed = (hit?.leg !== this.hover?.leg) || (hit?.joint !== this.hover?.joint);
        this.hover = hit;
        c.style.cursor = hit ? 'grab' : 'default';
        if (changed) this.draw();
        return;
      }

      if (this._drag.kind === 'orbit') {
        const dx = p.x - this._drag.last.x;
        const dy = p.y - this._drag.last.y;
        this.renderer.camera.orbit(-dx * 0.008, dy * 0.008);
        this._drag.last = p;
      } else if (this._drag.kind === 'joint') {
        this._updateJointDrag(p);
      } else if (this._drag.kind === 'foot') {
        this._updateFootDrag(p);
      }
    });

    const end = (e) => {
      pointers.delete(e.pointerId);
      if (pointers.size < 2) pinchDistance = 0;
      if (pointers.size === 0) this._drag = null;
    };
    c.addEventListener('pointerup', end);
    c.addEventListener('pointercancel', end);

    c.addEventListener('wheel', (e) => {
      e.preventDefault();
      this.renderer.camera.zoom(e.deltaY > 0 ? 1.1 : 0.9);
    }, { passive: false });
  }

  _pick(x, y) {
    let best = null, bestDistance = 26;
    for (const h of this._handles) {
      const d = Math.hypot(h.x - x, h.y - y);
      if (d < bestDistance) { bestDistance = d; best = h; }
    }
    return best ? { leg: best.leg, joint: best.joint } : null;
  }

  _beginJointDrag(hit, p) {
    if (hit.joint === 'foot') {
      const chain = this.legWorldChain(hit.leg);
      const screen = this.renderer.project(chain[3]);
      this._drag = { kind: 'foot', leg: hit.leg, last: p, depth: screen?.depth || 600 };
      return;
    }

    const chain = this.legWorldChain(hit.leg);
    const index = JOINT_ORDER.indexOf(hit.joint);
    const pivot = chain[index];
    const tip = chain[3];                       // rotate about the pivot, follow the foot
    const axis = this.jointAxis(hit.leg, hit.joint);

    // Screen-space tangent: how far, in pixels, the foot travels for
    // one radian of this joint. Dragging along it rotates the servo by
    // a matching amount, which is what makes the grab feel direct.
    const arm = v3.sub(tip, pivot);
    const radial = v3.sub(arm, v3.scale(axis, v3.dot(arm, axis)));
    const tangent = v3.cross(axis, radial);

    const a = this.renderer.project(tip);
    const b = this.renderer.project(v3.add(tip, tangent));
    if (!a || !b) { this._drag = { kind: 'orbit', last: p }; return; }

    const dx = b.x - a.x, dy = b.y - a.y;
    const arcPixels = Math.hypot(dx, dy);

    // The tangent gives the drag DIRECTION, which is what makes the
    // grab feel attached to the joint. Its raw length would give the
    // gain too, but following the foot exactly is far too stiff - a
    // long leg seen head-on needs most of a screen-width of drag for
    // a useful angle. So the direction is physical and the gain is
    // clamped to a fraction of the viewport, tuned so a comfortable
    // ~100px drag sweeps roughly 30 degrees at any zoom or screen size.
    const height = this.renderer.height;
    const gain = clamp(arcPixels, height * 0.12, height * 0.28);

    this._drag = {
      kind: 'joint', leg: hit.leg, joint: hit.joint, last: p,
      dir: arcPixels > 1 ? [dx / arcPixels, dy / arcPixels] : [0, 1],
      pixelsPerRadian: gain,
      start: this.angles[hit.leg][hit.joint],
      accumulated: 0,
    };
  }

  _updateJointDrag(p) {
    const d = this._drag;
    const dx = p.x - d.last.x, dy = p.y - d.last.y;
    d.last = p;
    d.accumulated += (dx * d.dir[0] + dy * d.dir[1]) / d.pixelsPerRadian;

    const [min, max] = kin.jointLimits()[d.joint];
    const next = clamp(d.start + deg(d.accumulated), min, max);
    this.angles[d.leg] = { ...this.angles[d.leg], [d.joint]: next };
    this.onJointChange?.(d.leg, this.angles[d.leg]);
  }

  _updateFootDrag(p) {
    const d = this._drag;
    const dx = p.x - d.last.x, dy = p.y - d.last.y;
    d.last = p;

    // Convert a screen-plane drag into world millimetres at the foot's depth.
    const { right, up } = this.renderer._basis;
    const scale = d.depth / this.renderer._focal;
    const world = v3.add(v3.scale(right, dx * scale), v3.scale(up, -dy * scale));

    const chain = this.legWorldChain(d.leg);
    const target = v3.add(chain[3], world);

    // World -> body -> leg-local, then solve.
    const R = this.bodyMatrix();
    const inBody = mat3.apply(
      [R[0], R[3], R[6], R[1], R[4], R[7], R[2], R[5], R[8]],   // transpose
      v3.sub(target, this.bodyTranslation())
    );
    const local = kin.bodyToLegLocal(d.leg, inBody);
    const solved = kin.legIk(local[0], local[1], local[2]);
    this.angles[d.leg] = { coxa: solved.coxa, femur: solved.femur, tibia: solved.tibia };
    this.onFootChange?.(d.leg, { x: local[0], y: local[1], z: local[2] });
  }

  // --------------------------------------------------------- commands

  setMode(mode) {
    this.mode = mode;
    this.hover = null;
    if (mode === 'view') this.selected = null;
  }

  select(legId, joint) {
    this.selected = legId ? { leg: legId, joint: joint || 'coxa' } : null;
    this.onSelect?.(this.selected);
  }

  setJoint(legId, joint, value) {
    const [min, max] = kin.jointLimits()[joint];
    this.angles[legId] = { ...this.angles[legId], [joint]: clamp(value, min, max) };
    this.onJointChange?.(legId, this.angles[legId]);
  }

  resetView() {
    const c = this.renderer.camera;
    c.azimuth = -0.9; c.elevation = 0.42; c.distance = 620; c.target = [0, 0, -40];
  }

  setViewPreset(name) {
    const c = this.renderer.camera;
    const presets = {
      iso:   { azimuth: -0.9,             elevation: 0.42, distance: 620 },
      front: { azimuth: Math.PI,          elevation: 0.06, distance: 620 },
      side:  { azimuth: -Math.PI / 2,     elevation: 0.06, distance: 620 },
      top:   { azimuth: -Math.PI / 2,     elevation: 1.44, distance: 640 },
    };
    Object.assign(c, presets[name] || presets.iso);
    c.target = [0, 0, -40];
  }
}
