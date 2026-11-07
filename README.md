# Spider Mech

Control software for an 18-DOF hexapod: a Python control server and a
browser UI with a live 3D model you can drive, pose and hand-articulate
one servo at a time.

**It runs with no hardware attached.** Every peripheral defaults to a
simulated backend that models the real thing — servos with slew rates,
an IMU reading a virtual ground plane you can tilt, a battery that sags
under load. So the gaits, the balance maths, the calibration workflow
and the whole UI can be built and tested now, and the electronics drop
in later as a one-word change in `config.py`.

```bash
pip install -r backend/requirements.txt
cd backend && uvicorn main:app --host 0.0.0.0 --port 8000
```

Then open `http://<host>:8000/` from a phone or laptop on the same network.

---

## The 3D view

The viewport is a live model built from the robot's own geometry and
the joint angles the servos are actually being commanded to. It is not
an illustration: the JavaScript kinematics are a line-for-line mirror
of `backend/kinematics.py`, and the two are tested against each other,
so what you see is what the machine is doing.

| Mode | What dragging does |
|---|---|
| **VIEW** | Orbits the camera. Scroll or pinch to zoom. |
| **SERVO** | Grabs a joint handle and turns *that servo*, on screen and on the robot together. |
| **FOOT** | Drags a foot; IK solves the three joints to put it there. |

Grab a joint and an arc appears showing which way that servo sweeps —
drag along it. The panel underneath gives the same three joints as
numeric sliders, plus the servo channel numbers, for when you want a
precise angle rather than a gesture.

Taking hold of a leg stops the robot walking and marks that leg as
overridden (`L2*` in the view, `servo` on the leg button). **Release
leg** hands it back to the stance and gait engine.

`ISO / FRONT / SIDE / TOP` snap the camera. `TRIPODS` colours the two
alternating tripod groups, which is the clearest way to see what a gait
is actually doing.

---

## What the controls do

**Gaits.** All seven are pure data in `config.GAITS` — a phase offset
per leg, a duty factor, and scale factors. One generic cycle generator
runs all of them.

| Gait | Feet down (min) | Character |
|---|---|---|
| `TRIPOD` | 3 | Two alternating sets of three. Fastest, least stable. |
| `TRIPLE` | 4 | Three groups of two. |
| `TRIPLE 2.5` | 4 | Overlapping stance, longer stride. Smoother, slower. |
| `RIPPLE` | 4 | Two travelling waves, one per side, half a cycle apart. |
| `RIPPLE 1.5` | 4 | Short stride, long stance. The most stable. |
| `WAVE` | 5 | One leg at a time around the body. Slowest. |
| `CRAB WALK` | 3 | Tripod timing, travel rotated 90° to the heading. |

> `TRIPOD`, `RIPPLE` and `WAVE` are standard gaits with standard
> timings. `TRIPLE`, `TRIPLE 2.5` and `RIPPLE 1.5` are names from the
> hexapod controller apps rather than from the literature — the
> parameters are a considered interpretation, not a reproduction of
> some other firmware. Tune the numbers in `config.GAITS`; nothing else
> needs to change.

**Stances.** `STAND` `HOME` `WIDE` `CROUCH` `SIT` — resting foot
positions the feet slide toward rather than snapping to.

**Axis modes.** The pose pad's two axes are remapped by the selected
mode, so one pad drives different degrees of freedom:

| Mode | Horizontal | Vertical |
|---|---|---|
| `X-Y` | body slides sideways | body slides fore/aft |
| `Z-W` | yaw / twist | body height |
| `U-V` | roll | pitch |
| `XY-UV` | slide + bank together | slide + pitch together |

The feet stay planted throughout — this is body IK, not walking.

**Moves.** `JUMP` (crouch, extend hard, tuck, absorb) · `BOUNCE` ·
`FLEX` (roll/pitch/yaw figure over planted feet — a good warm-up to
check every joint moves freely; loops until stopped) · `DIVE`.

Whether `JUMP` actually leaves the ground depends on your servos and
the machine's mass — many hobby hexapods will not. The trajectory is
correct and safe either way; it just looks like a sharp hop.

**Behaviour.** `TORQUE` energises or releases the servos ·
`HOLD POS` freezes every joint and ignores motion commands ·
`AUTO LEVEL` feeds IMU roll/pitch back into body pose to stay
horizontal on a slope · `AUTO SIT` sits down after a long idle to take
the load off the servos · `BEEP`.

**STOP** is always in the header, and on <kbd>Space</kbd>. It zeroes
motion, cancels any move and freezes the joints where they are — it
does *not* jump to a default pose, since a sudden move can be worse
than holding still.

---

## Architecture

```
backend/
  config.py        one description of YOUR robot: geometry, wiring,
                   gaits, stances, limits, which drivers to use
  kinematics.py    leg IK/FK, full joint chain, body IK
  gaits.py         one generic cycle generator, driven by config data
  moves.py         scripted trajectories (jump/bounce/flex/dive)
  controller.py    the 50 Hz control loop
  calibration.py   kinematic angles -> servo units, persisted
  poses.py         named pose storage
  main.py          REST + WebSocket API, serves the UI
  hardware/
    base.py        the four interfaces everything above talks to
    simulated.py   working models of all of them
    servo_serial.py    USB 32-channel controller
    servo_pca9685.py   PCA9685 x2 over I2C
    sensors.py     MPU6050, INA219, ADS1115, GPIO buzzer
frontend/
  js/math3d.js     vectors, matrices
  js/kinematics.js mirror of the backend kinematics
  js/render3d.js   software 3D renderer on a 2D canvas
  js/hexapod.js    the scene, picking, and servo dragging
  js/net.js        WebSocket + REST client
  js/app.js        panel wiring
tests/             175 tests, all runnable with no hardware
```

The control loop, each tick: read sensors → compute the auto-level
correction → slew the stance → advance the gait → build foot targets →
overlay the active move → resolve body pose → body IK then leg IK per
leg → rate-limit every joint → calibrate → write to the servo bus.

Ordering is deliberate. Overrides beat gaits, safety interlocks beat
everything, and the rate limiter sits last so nothing upstream can
command a joint faster than a servo can follow.

**No CDN, no build step.** The page is served off the robot, often on a
network with no route to the internet. The 3D renderer is ~300 lines of
canvas code rather than a 3D library, and the whole frontend is plain
ES modules.

---

## Moving to real hardware

Everything defaults to `"simulated"`. Switching a peripheral is one
word in `config.HARDWARE`:

```python
HARDWARE = {
    "servos": "serial32",   # or "pca9685", "simulated"
    "imu":    "mpu6050",
    "power":  "ina219",     # or "ads1115"
    "buzzer": "gpio",
}
```

If a backend can't start — library missing, nothing plugged in, wrong
address — it logs the reason, shows it in the UI's Setup tab, and falls
back to simulation rather than taking the robot down. Set
`HARDWARE_FALLBACK_TO_SIMULATED = False` once you'd rather a missing
servo bus be fatal.

Install only the libraries for what you actually fitted; they're listed
and commented in `backend/requirements.txt`.

### ⚠ The servo controller protocol is not confirmed

The datasheet for the "Generic USB 32Ch Servo Motor Controller Board"
(order code 2223800.1) confirms USB serial at 115200 baud but **does
not document the command byte format**. The board's feature list
matches the common Lobot/Hiwonder-style family, which uses a `0x55
0x55` binary protocol — that is an educated guess for this board, not a
fact. A second common format (SSC-32 style ASCII) is also implemented.

**Bring-up, in order — do not skip steps:**

1. Leave everything simulated and confirm the full stack works.
2. Power the servo rail from its own supply, **not** from the Pi.
3. Disconnect all servos but **one**, on a joint that can swing freely
   without hitting the frame.
4. Sweep that one channel under each protocol:
   ```bash
   cd backend && python -m hardware.servo_serial --port /dev/ttyUSB0 --probe 0
   ```
   It moves the servo gently and asks which protocol worked.
5. Set the working protocol, reconnect the rest, and calibrate one
   joint at a time.
6. If neither protocol moves it, find documentation matching your
   board's chip markings, or ask the seller — don't assume.

The PCA9685 backend has no such ambiguity; it's a good fallback if the
serial board proves undocumented.

### Calibration

`servo_angle = direction × kinematic_angle + offset`, clamped.

Do it from the UI's **Setup → Calibration** panel, which shows the live
`kinematic → servo` conversion as you drag. Pick a leg and joint, adjust
the offset until the real limb matches the 3D model, flip direction for
any servo mounted mirrored, then **Save** — it writes `calibration.json`
and survives restarts. You should never need to hand-edit `config.py`
once the robot is physically built.

The defaults are a neutral starting point, not a working calibration.
Note that the tibia's offset is `0` while coxa and femur are `90`: the
tibia's kinematic range is positive-only (0–160°), so an offset of 90
would push it to 90–250° and jam it against its stop. Check that first
if a joint parks hard against one end of its travel.

### Sanity checks before power-up

- `MIN/MAX_FEMUR_TO_FOOT` and `JOINT_LIMITS_DEG` are what stop IK
  commanding something the leg can't do. Measure your real segments and
  update `config.py` if your build differs from 55/90/130 mm.
- `SERVO_MAX_SLEW_DEG_PER_S` caps how fast any joint may move. Lower it
  for the first powered test.
- Run `pytest tests/` after any change to `config.py` — the suite
  checks every stance is reachable, every gait keeps at least three
  feet down, and no gait drives a leg out of its workspace.

---

## API

The WebSocket at `/ws/control` carries everything continuous — driving,
servo drags, and 20 Hz telemetry. It greets with the full robot
description, so a client needs no prior knowledge of the geometry.
REST is for one-shot commands and for scripting from a shell.

```
GET  /api/config                     geometry, gaits, moves, stances, axis modes, hardware
GET  /api/status                     full state snapshot
POST /api/drive                      walk_x, walk_y, turn, speed, step_height, stride
POST /api/gait                       {"name": "ripple"}
POST /api/stance                     {"name": "wide"}
POST /api/body  /api/axis  /api/body/center
POST /api/behaviour                  torque, hold_pos, auto_level, auto_sit
POST /api/move/{name}                jump | bounce | flex | dive | stop
POST /api/estop  /api/estop/clear  /api/beep
POST /api/leg/{id}/joints            direct servo control for one leg
POST /api/leg/{id}/foot              foot target, solved through IK
DEL  /api/leg/{id}/override  /api/legs/override
GET/POST /api/calibration  + /save /reset
GET/POST/DEL /api/pose...
POST /api/imu/level                  treat the current orientation as level
GET/POST /api/sim/tilt               tilt the virtual ground (simulated IMU only)
GET  /api/servos                     raw channel angles
```

The browser console has `spiderMech.robot` and `spiderMech.view`, so
you can drive the machine directly: `spiderMech.robot.gait('wave')`.

---

## Tests

```bash
pip install pytest httpx
pytest tests/ -q          # 175 tests, no hardware needed
```

They cover the things that are expensive to get wrong: IK/FK round-trip
to floating-point exactness, every stance reachable, every gait
statically stable with continuous foot paths, the slew limiter, the
safety interlocks, auto-level convergence and bounds, per-leg
overrides, servo frame encoding, and hardware fallback.

---

## Not built yet

- **Stability-margin gating.** Nothing computes the support polygon and
  refuses a body pose that would put the centre of mass outside it.
  `gaits.grounded_legs()` gives the support set; that's the hook.
- **Servo position feedback.** The `ServoBus.read_angle` contract
  exists and the simulator implements it, but neither real servo
  backend reports actual position, so nothing detects a stalled or
  fighting joint.
- **Battery cut-off.** Voltage and current are read and displayed, but
  nothing acts on them — no low-voltage sit-down or over-current trip.
- **Terrain adaptation.** Auto-level uses the IMU only. There is no
  ground contact sensing, so the robot can't feel for a foothold.
