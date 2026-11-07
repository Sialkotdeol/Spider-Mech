// Connection to the robot.
//
// The WebSocket carries everything continuous - drive commands, servo
// drags, telemetry - because a dropped frame there is harmless and
// latency is not. REST is for one-shot commands, where knowing the
// request actually landed matters more than speed.
//
// Reconnects on its own with a backoff, and reports link state so the
// UI can grey itself out rather than silently pretending to work.

export class Robot extends EventTarget {
  constructor() {
    super();
    this.ws = null;
    this.connected = false;
    this.config = null;
    this.status = null;
    this._retry = 0;
    this._reconnectTimer = null;
    this._lastSend = new Map();
  }

  get origin() { return window.location.origin; }

  connect() {
    if (this.ws && (this.ws.readyState === WebSocket.OPEN ||
                    this.ws.readyState === WebSocket.CONNECTING)) return;

    const proto = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    try {
      this.ws = new WebSocket(`${proto}//${window.location.host}/ws/control`);
    } catch {
      this._scheduleReconnect();
      return;
    }

    this.ws.onopen = () => {
      this.connected = true;
      this._retry = 0;
      this.dispatchEvent(new CustomEvent('link', { detail: { connected: true } }));
    };

    this.ws.onclose = () => {
      this.connected = false;
      this.dispatchEvent(new CustomEvent('link', { detail: { connected: false } }));
      this._scheduleReconnect();
    };

    this.ws.onerror = () => this.ws?.close();

    this.ws.onmessage = (event) => {
      let message;
      try { message = JSON.parse(event.data); } catch { return; }
      if (message.type === 'hello') {
        this.config = message.config;
        this.dispatchEvent(new CustomEvent('config', { detail: message.config }));
      } else if (message.type === 'telemetry') {
        this.status = message;
        this.dispatchEvent(new CustomEvent('telemetry', { detail: message }));
      }
    };
  }

  _scheduleReconnect() {
    if (this._reconnectTimer) return;
    const delay = Math.min(500 * 2 ** this._retry++, 5000);
    this._reconnectTimer = setTimeout(() => {
      this._reconnectTimer = null;
      this.connect();
    }, delay);
  }

  send(message) {
    if (this.ws?.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(message));
      return true;
    }
    return false;
  }

  // Rate-limit a stream of updates of the same kind (a dragged slider,
  // a joystick) without ever dropping the final value: the last frame
  // is always flushed once the window closes.
  sendThrottled(key, message, intervalMs = 50) {
    const now = performance.now();
    const entry = this._lastSend.get(key);
    if (entry && now - entry.at < intervalMs) {
      clearTimeout(entry.timer);
      entry.timer = setTimeout(() => {
        this._lastSend.set(key, { at: performance.now(), timer: null });
        this.send(message);
      }, intervalMs - (now - entry.at));
      entry.pending = message;
      return;
    }
    if (entry?.timer) clearTimeout(entry.timer);
    this._lastSend.set(key, { at: now, timer: null });
    this.send(message);
  }

  async get(path) {
    const response = await fetch(this.origin + path);
    if (!response.ok) throw new Error(`${path}: ${response.status}`);
    return response.json();
  }

  async post(path, body) {
    const response = await fetch(this.origin + path, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: body === undefined ? undefined : JSON.stringify(body),
    });
    if (!response.ok) {
      let detail = `${response.status}`;
      try { detail = (await response.json()).detail ?? detail; } catch { /* not JSON */ }
      throw new Error(detail);
    }
    return response.json();
  }

  async del(path) {
    const response = await fetch(this.origin + path, { method: 'DELETE' });
    if (!response.ok) throw new Error(`${path}: ${response.status}`);
    return response.json();
  }

  // ------------------------------------------------------- shorthands

  drive(fields) { this.sendThrottled('drive', { type: 'drive', ...fields }, 45); }
  body(fields) { this.sendThrottled('body', { type: 'body', ...fields }, 45); }
  axis(mode, h, v) { this.sendThrottled('axis', { type: 'axis', mode, h, v }, 45); }
  legJoints(leg, angles) {
    this.sendThrottled(`leg:${leg}`, { type: 'leg_joints', leg, ...angles }, 40);
  }
  legFoot(leg, foot) {
    this.sendThrottled(`foot:${leg}`, { type: 'leg_foot', leg, ...foot }, 40);
  }
  clearOverride(leg) { this.send({ type: 'clear_override', leg: leg || null }); }
  gait(name) { this.send({ type: 'gait', name }); }
  stance(name) { this.send({ type: 'stance', name }); }
  behaviour(fields) { this.send({ type: 'behaviour', ...fields }); }
  move(name) { this.send({ type: 'move', name }); }
  beep(tone = 'beep') { this.send({ type: 'beep', tone }); }
  estop() { this.send({ type: 'estop' }); this.post('/api/estop').catch(() => {}); }
  clearEstop() { this.send({ type: 'estop_clear' }); }
  ping() { this.send({ type: 'ping' }); }
}
