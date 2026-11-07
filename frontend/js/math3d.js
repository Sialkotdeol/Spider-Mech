// Minimal 3D maths for the hexapod viewer.
// Vectors are plain [x, y, z] arrays; matrices are row-major [9].
// Deliberately dependency-free: the Pi serves this page on a network
// that may have no internet, so nothing here may come from a CDN.

export const v3 = {
  add: (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]],
  sub: (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]],
  scale: (a, s) => [a[0] * s, a[1] * s, a[2] * s],
  dot: (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2],
  cross: (a, b) => [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ],
  length: (a) => Math.hypot(a[0], a[1], a[2]),
  normalize(a) {
    const l = Math.hypot(a[0], a[1], a[2]);
    return l < 1e-9 ? [0, 0, 0] : [a[0] / l, a[1] / l, a[2] / l];
  },
  lerp: (a, b, t) => [
    a[0] + (b[0] - a[0]) * t,
    a[1] + (b[1] - a[1]) * t,
    a[2] + (b[2] - a[2]) * t,
  ],
};

export const mat3 = {
  identity: () => [1, 0, 0, 0, 1, 0, 0, 0, 1],

  mul(a, b) {
    const out = new Array(9);
    for (let r = 0; r < 3; r++) {
      for (let c = 0; c < 3; c++) {
        out[r * 3 + c] =
          a[r * 3] * b[c] + a[r * 3 + 1] * b[3 + c] + a[r * 3 + 2] * b[6 + c];
      }
    }
    return out;
  },

  apply: (m, v) => [
    m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
    m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
    m[6] * v[0] + m[7] * v[1] + m[8] * v[2],
  ],

  rotX(rad) {
    const c = Math.cos(rad), s = Math.sin(rad);
    return [1, 0, 0, 0, c, -s, 0, s, c];
  },
  rotY(rad) {
    const c = Math.cos(rad), s = Math.sin(rad);
    return [c, 0, s, 0, 1, 0, -s, 0, c];
  },
  rotZ(rad) {
    const c = Math.cos(rad), s = Math.sin(rad);
    return [c, -s, 0, s, c, 0, 0, 0, 1];
  },

  // Matches kinematics.rotation_matrix on the backend exactly:
  // R = Rz(yaw) @ Ry(pitch) @ Rx(roll). The 3D view must agree with
  // the maths the servos are actually being driven from.
  fromEulerDeg(rollDeg, pitchDeg, yawDeg) {
    const d = Math.PI / 180;
    return mat3.mul(
      mat3.mul(mat3.rotZ(yawDeg * d), mat3.rotY(pitchDeg * d)),
      mat3.rotX(rollDeg * d)
    );
  },
};

// Rotate a vector about an arbitrary axis (Rodrigues' formula).
// Used to draw the arc showing which way a servo turns.
export function rotateAbout(vec, axis, angleRad) {
  const k = v3.normalize(axis);
  const c = Math.cos(angleRad), s = Math.sin(angleRad);
  return v3.add(
    v3.add(v3.scale(vec, c), v3.scale(v3.cross(k, vec), s)),
    v3.scale(k, v3.dot(k, vec) * (1 - c))
  );
}

export const deg = (rad) => (rad * 180) / Math.PI;
export const rad = (degrees) => (degrees * Math.PI) / 180;
export const clamp = (v, lo, hi) => (v < lo ? lo : v > hi ? hi : v);
