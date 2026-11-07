// Small DOM helpers, shared by the panels.

export const $ = (sel, root = document) => root.querySelector(sel);
export const $$ = (sel, root = document) => [...root.querySelectorAll(sel)];

export function chip(label, { title, onClick, dataset = {} } = {}) {
  const el = document.createElement('button');
  el.className = 'chip';
  el.textContent = label;
  if (title) el.title = title;
  Object.assign(el.dataset, dataset);
  if (onClick) el.addEventListener('click', () => onClick(el));
  return el;
}

// One labelled slider with a live readout. Returns handles so callers
// can push values in without re-reading the DOM.
export function slider(parent, { label, min, max, value, step = 1, unit = '', format, onInput, numeric = false }) {
  const wrap = document.createElement('div');
  wrap.className = 'slider';
  const text = format || ((v) => `${Math.round(v)}${unit}`);

  // `numeric` adds a box to type an exact value; Enter or leaving the
  // box applies it, clamped to the slider's range.
  wrap.innerHTML = `<label><span></span>${numeric
      ? `<input type="number" class="num" min="${min}" max="${max}" step="${step}" value="${value}">`
      : '<output></output>'}</label>
    <input type="range" min="${min}" max="${max}" step="${step}" value="${value}">`;
  wrap.querySelector('span').textContent = label;
  const input = wrap.querySelector('input[type=range]');
  const output = wrap.querySelector('output');
  const box = wrap.querySelector('input.num');
  const show = (v) => {
    if (output) output.textContent = text(v);
    if (box && document.activeElement !== box) box.value = Number(v).toFixed(step < 1 ? 1 : 0);
  };
  show(value);

  input.addEventListener('input', () => {
    const v = Number(input.value);
    show(v);
    onInput?.(v);
  });

  if (box) {
    const apply = () => {
      const typed = Number(box.value);
      if (box.value === '' || !Number.isFinite(typed)) { show(input.value); return; }
      const v = Math.max(Number(min), Math.min(Number(max), typed));
      input.value = v;
      box.value = v;
      onInput?.(v);
    };
    box.addEventListener('change', apply);
    box.addEventListener('keydown', (e) => { if (e.key === 'Enter') { apply(); box.blur(); } });
  }
  parent.appendChild(wrap);

  return {
    el: wrap,
    input,
    set(v) {
      if (document.activeElement === input || (box && document.activeElement === box)) return;   // never fight the operator
      input.value = v;
      show(Number(v));
    },
    get: () => Number(input.value),
  };
}

// A circular two-axis pad. Reports -1..1 on each axis; springs back to
// centre on release unless `sticky` is set.
export function pad(el, { onChange, sticky = false } = {}) {
  const knob = el.querySelector('.pad-knob');
  let dragging = false;
  let value = { h: 0, v: 0 };

  const place = (h, v) => {
    const r = el.clientWidth / 2;
    const travel = r - knob.offsetWidth / 2 - 2;
    knob.style.left = `${r + h * travel}px`;
    knob.style.top = `${r - v * travel}px`;
  };

  const update = (clientX, clientY) => {
    const rect = el.getBoundingClientRect();
    const r = rect.width / 2;
    let dx = (clientX - rect.left - r) / (r - knob.offsetWidth / 2 - 2);
    let dy = -(clientY - rect.top - r) / (r - knob.offsetHeight / 2 - 2);
    const mag = Math.hypot(dx, dy);
    if (mag > 1) { dx /= mag; dy /= mag; }
    value = { h: dx, v: dy };
    place(dx, dy);
    onChange?.(dx, dy);
  };

  el.addEventListener('pointerdown', (e) => {
    dragging = true;
    el.setPointerCapture(e.pointerId);
    update(e.clientX, e.clientY);
  });
  el.addEventListener('pointermove', (e) => { if (dragging) update(e.clientX, e.clientY); });
  const release = () => {
    if (!dragging) return;
    dragging = false;
    if (!sticky) { value = { h: 0, v: 0 }; place(0, 0); onChange?.(0, 0); }
  };
  el.addEventListener('pointerup', release);
  el.addEventListener('pointercancel', release);

  place(0, 0);
  return {
    get: () => value,
    set(h, v) { value = { h, v }; place(h, v); },
    reset() { value = { h: 0, v: 0 }; place(0, 0); },
  };
}

export function setActive(container, selector, matcher) {
  $$(selector, container).forEach((el) => el.classList.toggle('on', matcher(el)));
}

export function banner(message, kind = 'info') {
  const el = $('#banner');
  if (!message) { el.hidden = true; return; }
  el.textContent = message;
  el.className = `banner ${kind === 'error' ? 'error' : ''}`;
  el.hidden = false;
}

export const fmt = {
  deg: (v) => `${v >= 0 ? '' : ''}${v.toFixed(1)}°`,
  mm: (v) => `${v.toFixed(0)} mm`,
  pct: (v) => `${Math.round(v)}%`,
};
