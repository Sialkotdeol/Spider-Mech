// A mirror of backend/kinematics.py.
//
// The 3D view runs its own copy of the forward kinematics so it can
// redraw at 60fps and respond to a drag with zero latency, instead of
// waiting for a telemetry frame. It is the same formula as the
// backend's, so what you see is what the servos are being told - if
// you change the geometry in config.py, the shape on screen changes
// with it, because everything here reads from /api/config.

import { mat3, v3, rad, clamp } from './math3d.js';

let G = {
  coxa: 55, femur: 90, tibia: 130,
  body_radius: 90, body_thickness: 22,
  reach_min: 40, reach_max: 220,
};
let LEGS = [];
let JOINT_LIMITS = { coxa: [-60, 60], femur: [-90, 90], tibia: [0, 160] };
// Mirror of config.FEMUR_LOCK_DEG: when set, every femur stays at this
// angle and the knee alone sets the foot height.
let FEMUR_LOCK = null;
const LOCKED_REACH_TOLERANCE_MM = 50;

export function configure(cfg) {
  G = cfg.geometry;
  LEGS = cfg.legs;
  JOINT_LIMITS = cfg.joint_limits;
  FEMUR_LOCK = cfg.femur_lock ?? null;
}

export const femurLock = () => FEMUR_LOCK;

export const geometry = () => G;
export const legs = () => LEGS;
// Limits for commanding a joint: a locked femur can only sit at its lock.
export const jointLimits = () => (FEMUR_LOCK === null ? JOINT_LIMITS
  : { ...JOINT_LIMITS, femur: [FEMUR_LOCK, FEMUR_LOCK] });
export const legById = (id) => LEGS.find((l) => l.id === id);

// Joint positions along one leg, in the leg's own frame:
// [coxa pivot, femur pivot, knee, foot].
export function legChain(angles) {
  const c = rad(angles.coxa), f = rad(angles.femur), t = rad(angles.tibia);
  const cc = Math.cos(c), sc = Math.sin(c);

  const p0 = [0, 0, 0];
  const p1 = [G.coxa * cc, G.coxa * sc, 0];

  const fh = G.femur * Math.cos(f), fz = G.femur * Math.sin(f);
  const p2 = [p1[0] + fh * cc, p1[1] + fh * sc, fz];

  const th = G.tibia * Math.cos(f - t), tz = G.tibia * Math.sin(f - t);
  const p3 = [p2[0] + th * cc, p2[1] + th * sc, p2[2] + tz];

  return [p0, p1, p2, p3];
}

export const legFoot = (angles) => legChain(angles)[3];

// IK with the femur held at its lock, identical to the backend's
// _leg_ik_femur_locked(): the coxa aims, the tibia sets the height.
function legIkFemurLocked(x, y, z, femurDeg) {
  const coxa = (Math.atan2(y, x) * 180) / Math.PI;
  const f = rad(femurDeg);
  const drop = (z - G.femur * Math.sin(f)) / G.tibia;
  let reachable = drop >= -1 && drop <= 1;
  const shin = Math.asin(clamp(drop, -1, 1));
  const out = { coxa, femur: femurDeg, tibia: ((f - shin) * 180) / Math.PI, reachable };
  for (const j of ['coxa', 'tibia']) {
    const [min, max] = JOINT_LIMITS[j];
    out[j] = clamp(out[j], min, max);
  }
  const foot = legFoot(out);
  if (Math.hypot(foot[0] - x, foot[1] - y) > LOCKED_REACH_TOLERANCE_MM) out.reachable = false;
  return out;
}

// Analytic 3-DOF IK, identical to backend leg_ik().
export function legIk(x, y, z) {
  if (FEMUR_LOCK !== null) return legIkFemurLocked(x, y, z, FEMUR_LOCK);
  const coxa = Math.atan2(y, x);
  const horiz = Math.hypot(x, y) - G.coxa;
  let dist = Math.hypot(horiz, z);

  let reachable = true;
  const lo = G.reach_min + 1e-6, hi = G.reach_max - 1e-6;
  if (dist < lo || dist > hi) { reachable = false; dist = clamp(dist, lo, hi); }

  const angleToFoot = Math.atan2(z, horiz);
  const femurOffset = Math.acos(
    clamp((G.femur ** 2 + dist ** 2 - G.tibia ** 2) / (2 * G.femur * dist), -1, 1)
  );
  const knee = Math.acos(
    clamp((G.femur ** 2 + G.tibia ** 2 - dist ** 2) / (2 * G.femur * G.tibia), -1, 1)
  );

  const out = {
    coxa: (coxa * 180) / Math.PI,
    femur: ((angleToFoot + femurOffset) * 180) / Math.PI,
    tibia: ((Math.PI - knee) * 180) / Math.PI,
    reachable,
  };
  for (const j of ['coxa', 'femur', 'tibia']) {
    const [min, max] = JOINT_LIMITS[j];
    out[j] = clamp(out[j], min, max);
  }
  return out;
}

// Leg-local point -> body frame.
export function legLocalToBody(legId, p) {
  const leg = legById(legId);
  const R = mat3.rotZ(rad(leg.mount_angle));
  return v3.add(mat3.apply(R, p), leg.mount_point);
}

// Body frame -> leg-local.
export function bodyToLegLocal(legId, p) {
  const leg = legById(legId);
  const d = Math.PI / 180;
  const R = mat3.rotZ(-leg.mount_angle * d);
  return mat3.apply(R, v3.sub(p, leg.mount_point));
}
