// A small software 3D renderer on a 2D canvas.
//
// Why not a 3D library: this page is served off the robot itself,
// often with no route to the internet, and the scene is ~500 polygons.
// A painter's-algorithm renderer handles that comfortably at 60fps,
// costs nothing to load, and gives us exact control over picking -
// which is what makes dragging an individual servo feel direct.
//
// Pipeline: build world-space primitives -> transform to camera space
// -> reject anything behind the near plane -> sort back-to-front ->
// draw. Flat Lambert shading, one directional light.

import { v3, mat3, clamp } from './math3d.js';

export class Camera {
  constructor() {
    this.target = [0, 0, -40];
    this.azimuth = -0.9;     // radians around world Z
    this.elevation = 0.42;   // radians above the horizon
    this.distance = 620;
    this.fov = Math.PI / 4;
    this.near = 40;
  }

  eye() {
    const ce = Math.cos(this.elevation);
    return v3.add(this.target, [
      ce * Math.cos(this.azimuth) * this.distance,
      ce * Math.sin(this.azimuth) * this.distance,
      Math.sin(this.elevation) * this.distance,
    ]);
  }

  // Orthonormal basis: right, up, forward.
  basis() {
    const eye = this.eye();
    const forward = v3.normalize(v3.sub(this.target, eye));
    let right = v3.cross(forward, [0, 0, 1]);
    if (v3.length(right) < 1e-6) right = [1, 0, 0];
    right = v3.normalize(right);
    const up = v3.cross(right, forward);
    return { eye, right, up, forward };
  }

  orbit(dAzimuth, dElevation) {
    this.azimuth += dAzimuth;
    this.elevation = clamp(this.elevation + dElevation, -1.45, 1.45);
  }

  zoom(factor) {
    this.distance = clamp(this.distance * factor, 220, 1800);
  }
}

export class Renderer {
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.camera = new Camera();
    this.primitives = [];
    this.width = 1;
    this.height = 1;
    this.dpr = 1;
    this.light = v3.normalize([-0.45, -0.6, 0.85]);
    this._basis = null;
    this._focal = 1;
  }

  resize() {
    const rect = this.canvas.getBoundingClientRect();
    this.dpr = Math.min(window.devicePixelRatio || 1, 2);
    this.width = Math.max(1, Math.round(rect.width));
    this.height = Math.max(1, Math.round(rect.height));
    this.canvas.width = Math.round(this.width * this.dpr);
    this.canvas.height = Math.round(this.height * this.dpr);
    this.ctx.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
  }

  // ------------------------------------------------------- projection

  beginFrame() {
    this.primitives.length = 0;
    this._basis = this.camera.basis();
    this._focal = this.height / 2 / Math.tan(this.camera.fov / 2);
  }

  toCamera(p) {
    const { eye, right, up, forward } = this._basis;
    const d = v3.sub(p, eye);
    return [v3.dot(d, right), v3.dot(d, up), v3.dot(d, forward)];
  }

  // World point -> screen. Returns null when behind the near plane.
  project(p) {
    const c = this.toCamera(p);
    if (c[2] < this.camera.near) return null;
    return {
      x: this.width / 2 + (this._focal * c[0]) / c[2],
      y: this.height / 2 - (this._focal * c[1]) / c[2],
      depth: c[2],
    };
  }

  // --------------------------------------------------- scene building

  addFace(points, color, options = {}) {
    const projected = [];
    let depth = 0;
    for (const p of points) {
      const s = this.project(p);
      if (!s) return;                 // clip the whole face; fine at this scale
      projected.push(s);
      depth += s.depth;
    }
    depth /= points.length;

    let shade = 1;
    if (!options.flat) {
      const normal = v3.normalize(
        v3.cross(v3.sub(points[1], points[0]), v3.sub(points[2], points[0]))
      );
      // Two-sided lighting: back faces of an open shape still read as solid.
      shade = 0.34 + 0.66 * Math.abs(v3.dot(normal, this.light));
    }

    this.primitives.push({
      kind: 'face', points: projected, color, shade, depth,
      stroke: options.stroke, alpha: options.alpha ?? 1,
    });
  }

  addLine(a, b, color, width = 1, alpha = 1) {
    const pa = this.project(a), pb = this.project(b);
    if (!pa || !pb) return;
    this.primitives.push({
      kind: 'line', a: pa, b: pb, color, width, alpha,
      depth: (pa.depth + pb.depth) / 2,
    });
  }

  addDisc(center, radius, color, alpha = 1, segments = 20) {
    const points = [];
    for (let i = 0; i < segments; i++) {
      const a = (i / segments) * Math.PI * 2;
      points.push([center[0] + Math.cos(a) * radius, center[1] + Math.sin(a) * radius, center[2]]);
    }
    this.addFace(points, color, { flat: true, alpha });
  }

  // A prism along a segment: the basic building block of every limb.
  addSegment(a, b, radius, color, sides = 6, options = {}) {
    const axis = v3.normalize(v3.sub(b, a));
    if (v3.length(axis) < 1e-9) return;
    let u = Math.abs(axis[2]) > 0.9 ? [1, 0, 0] : [0, 0, 1];
    u = v3.normalize(v3.cross(axis, u));
    const w = v3.cross(axis, u);

    const radiusB = options.taper ? radius * options.taper : radius;
    const ringA = [], ringB = [];
    for (let i = 0; i < sides; i++) {
      const t = (i / sides) * Math.PI * 2;
      const dir = v3.add(v3.scale(u, Math.cos(t)), v3.scale(w, Math.sin(t)));
      ringA.push(v3.add(a, v3.scale(dir, radius)));
      ringB.push(v3.add(b, v3.scale(dir, radiusB)));
    }
    for (let i = 0; i < sides; i++) {
      const j = (i + 1) % sides;
      this.addFace([ringA[i], ringA[j], ringB[j], ringB[i]], color, options);
    }
    this.addFace(ringA.slice().reverse(), color, options);
    this.addFace(ringB, color, options);
  }

  addPrism(centre, radius, height, sides, color, options = {}) {
    const top = [], bottom = [];
    for (let i = 0; i < sides; i++) {
      const a = (i / sides) * Math.PI * 2 + (options.rotate || 0);
      const x = centre[0] + Math.cos(a) * radius;
      const y = centre[1] + Math.sin(a) * radius;
      top.push([x, y, centre[2] + height / 2]);
      bottom.push([x, y, centre[2] - height / 2]);
    }
    for (let i = 0; i < sides; i++) {
      const j = (i + 1) % sides;
      this.addFace([bottom[i], bottom[j], top[j], top[i]], color, options);
    }
    this.addFace(top, color, options);
    this.addFace(bottom.slice().reverse(), color, options);
    return { top, bottom };
  }

  // A screen-space marker that always faces the viewer - used for the
  // draggable joint handles, so they stay the same size at any zoom.
  addHandle(p, radius, color, options = {}) {
    const s = this.project(p);
    if (!s) return null;
    this.primitives.push({
      kind: 'handle', x: s.x, y: s.y, radius, color,
      depth: s.depth - (options.onTop ? 1e6 : 0),
      ring: options.ring, label: options.label, alpha: options.alpha ?? 1,
    });
    return s;
  }

  addText(p, text, color, options = {}) {
    const s = this.project(p);
    if (!s) return;
    this.primitives.push({
      kind: 'text', x: s.x, y: s.y, text, color,
      depth: s.depth - (options.onTop ? 1e6 : 0),
      font: options.font || '600 11px ui-monospace, monospace',
      align: options.align || 'center',
      offsetY: options.offsetY || 0,
    });
  }

  // -------------------------------------------------------- rasterise

  render(background) {
    const ctx = this.ctx;
    ctx.clearRect(0, 0, this.width, this.height);
    if (background) {
      ctx.fillStyle = background;
      ctx.fillRect(0, 0, this.width, this.height);
    }

    // Painter's algorithm: furthest first.
    this.primitives.sort((a, b) => b.depth - a.depth);

    for (const p of this.primitives) {
      ctx.globalAlpha = p.alpha ?? 1;
      if (p.kind === 'face') {
        ctx.beginPath();
        ctx.moveTo(p.points[0].x, p.points[0].y);
        for (let i = 1; i < p.points.length; i++) ctx.lineTo(p.points[i].x, p.points[i].y);
        ctx.closePath();
        ctx.fillStyle = shadeColor(p.color, p.shade);
        ctx.fill();
        if (p.stroke) {
          ctx.strokeStyle = p.stroke;
          ctx.lineWidth = 1;
          ctx.stroke();
        }
      } else if (p.kind === 'line') {
        ctx.beginPath();
        ctx.moveTo(p.a.x, p.a.y);
        ctx.lineTo(p.b.x, p.b.y);
        ctx.strokeStyle = p.color;
        ctx.lineWidth = p.width;
        ctx.stroke();
      } else if (p.kind === 'handle') {
        ctx.beginPath();
        ctx.arc(p.x, p.y, p.radius, 0, Math.PI * 2);
        ctx.fillStyle = p.color;
        ctx.fill();
        if (p.ring) {
          ctx.beginPath();
          ctx.arc(p.x, p.y, p.radius + 3.5, 0, Math.PI * 2);
          ctx.strokeStyle = p.ring;
          ctx.lineWidth = 2;
          ctx.stroke();
        }
      } else if (p.kind === 'text') {
        ctx.font = p.font;
        ctx.textAlign = p.align;
        ctx.textBaseline = 'middle';
        ctx.fillStyle = p.color;
        ctx.fillText(p.text, p.x, p.y + p.offsetY);
      }
    }
    ctx.globalAlpha = 1;
  }
}

// Multiply a #rrggbb colour by a shading factor.
const shadeCache = new Map();
export function shadeColor(hex, factor) {
  const key = hex + '|' + factor.toFixed(3);
  const hit = shadeCache.get(key);
  if (hit) return hit;
  const n = parseInt(hex.slice(1), 16);
  const r = Math.min(255, Math.round(((n >> 16) & 255) * factor));
  const g = Math.min(255, Math.round(((n >> 8) & 255) * factor));
  const b = Math.min(255, Math.round((n & 255) * factor));
  const out = `rgb(${r},${g},${b})`;
  if (shadeCache.size < 4096) shadeCache.set(key, out);
  return out;
}
