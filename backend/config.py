"""Spider Mech — hardware & kinematics configuration.

Single source of truth for geometry, wiring, calibration, limits, gaits,
stances, axis modes and which hardware backends to use. The 3D viewer
reads geometry from here via /api/config, so changing a number here
changes the on-screen hexapod with no front-end edits.

Calibration mapping (see calibration.py):
    servo_angle = direction * kinematic_angle + offset,   clamped

Kinematic conventions:
    coxa  0° → leg points straight out from body
    femur 0° → thigh horizontal (positive rotates up)
    tibia 0° → shin in line with femur (positive bends the knee)
"""

from __future__ import annotations

import os


# ============================================================ leg identity
LEG_IDS = ["L1", "L2", "L3", "R1", "R2", "R3"]
LEFT_LEGS = ["L1", "L2", "L3"]
RIGHT_LEGS = ["R1", "R2", "R3"]
JOINTS = ["coxa", "femur", "tibia"]

TRIPOD_A = ["L1", "R2", "L3"]
TRIPOD_B = ["R1", "L2", "R3"]


# =================================================== physical geometry (mm)
COXA_LENGTH_MM = 55.0
FEMUR_LENGTH_MM = 90.0
TIBIA_LENGTH_MM = 130.0

# Reachable annulus for the foot, measured from the femur joint.
MIN_FEMUR_TO_FOOT_MM = 60.0
MAX_FEMUR_TO_FOOT_MM = 215.0

BODY_RADIUS_MM = 90.0
BODY_PLATE_THICKNESS_MM = 6.0

# The battery hangs under the centre of the body. Heights are measured
# down from the hip plane (z = 0, the height of the femur servo shafts).
# The controller lifts the body whenever a stance, body pose, tilt or
# move would bring any bottom corner of the battery closer to the
# ground than BATTERY_CLEARANCE_MM. x is forward, y is left.
BATTERY_DROP_MM = 60.0            # hip plane down to the battery's bottom face
BATTERY_HALF_LENGTH_MM = 70.0     # half the battery's fore-aft length
BATTERY_HALF_WIDTH_MM = 25.0      # half its side-to-side width
BATTERY_CLEARANCE_MM = 15.0       # minimum air gap under the battery

# Body pose limits.
BODY_ANGLE_LIMIT_DEG = 25.0
BODY_TRANSLATE_LIMIT_MM = 40.0
BODY_Z_LIMIT_MM = 50.0

# Each coxa pivot sits on a circle of radius BODY_RADIUS_MM, at this angle.
LEG_MOUNT_ANGLES_DEG = {
    "L1":  45.0, "L2":  90.0, "L3":  135.0,
    "R1": -45.0, "R2": -90.0, "R3": -135.0,
}


# ======================================================== servo channel map
# Left legs on PCA9685 board #1 (I2C 0x40), channels 0–8 = its pins 0–8.
# Right legs on board #2 (0x41), channels 16–24 = its pins 0–8.
# Same channel numbers are used by the serial 32-channel controller —
# only the physical wire changes.
SERVO_CHANNELS = {
    "L1": {"coxa": 0,  "femur": 1,  "tibia": 2},    # board #1 pins 0, 1, 2
    "L2": {"coxa": 3,  "femur": 4,  "tibia": 5},    # board #1 pins 3, 4, 5
    "L3": {"coxa": 6,  "femur": 7,  "tibia": 8},    # board #1 pins 6, 7, 8
    "R1": {"coxa": 16, "femur": 17, "tibia": 18},   # board #2 pins 0, 1, 2
    "R2": {"coxa": 19, "femur": 20, "tibia": 21},   # board #2 pins 3, 4, 5
    "R3": {"coxa": 22, "femur": 23, "tibia": 24},   # board #2 pins 6, 7, 8
}


# ========================================= joint calibration (per-leg defaults)
# These are seed values. The UI's Calibrate panel writes to
# calibration.json, which overrides the values here at startup.
#
# direction = -1 for every femur: the physical femur servos are mounted
# such that increasing servo angle rotates the thigh in the opposite
# sense to the kinematic convention. Flipping the sign here aligns the
# real robot's femur motion with the 3D model.
_DEFAULT_JOINT_CAL = {
    "coxa":  {"offset": 90.0, "direction":  1, "min": 0.0, "max": 180.0},
    "femur": {"offset": 90.0, "direction": -1, "min": 0.0, "max": 180.0},
    "tibia": {"offset":  0.0, "direction":  1, "min": 0.0, "max": 180.0},
}
JOINT_CALIBRATION = {
    leg: {j: dict(cal) for j, cal in _DEFAULT_JOINT_CAL.items()}
    for leg in LEG_IDS
}

# Right-side tibias are usually mounted mirrored, so flipping direction
# must be paired with an offset at the top of travel so the mapped servo
# angle stays inside 0..180 across the kinematic range.
for _leg in RIGHT_LEGS:
    JOINT_CALIBRATION[_leg]["tibia"]["direction"] = -1
    JOINT_CALIBRATION[_leg]["tibia"]["offset"] = 180.0


# =================================== kinematic joint limits (degrees, [lo, hi])
# Used by leg_ik() to clamp solutions, and sent to the frontend so its
# sliders match. Keep as 2-element lists so `lo, hi = ...` unpacking works.
JOINT_LIMITS_DEG = {
    "coxa":  [-60.0,  60.0],
    "femur": [-90.0,  90.0],
    "tibia": [  0.0, 160.0],
}

# Femur lock. Every femur (L1..R3) is held at this kinematic angle
# everywhere: stances, walking, body poses, the Legs tab and the 3D
# view. 0 = thigh horizontal = servo 90 with the default calibration.
# The knee (tibia) alone then sets the foot height, so steps are lower
# and body height has less range. Set to None to let the femur move
# freely again.
FEMUR_LOCK_DEG = None


# ========================================================= hardware backends
def _on_raspberry_pi() -> bool:
    try:
        with open("/proc/device-tree/model") as f:
            return "raspberry pi" in f.read().lower()
    except OSError:
        return False


# The same config.py runs on the Mac and on the Pi. On the Pi it drives
# the real boards; anywhere else it simulates them, so the UI and tests
# run without hardware. Set SPIDER_SIMULATE=1 to force simulation on
# the Pi, or SPIDER_SIMULATE=0 to force real hardware elsewhere.
_sim_env = os.environ.get("SPIDER_SIMULATE")
USE_REAL_HARDWARE = (_sim_env == "0") if _sim_env is not None else _on_raspberry_pi()

if USE_REAL_HARDWARE:
    HARDWARE = {
        "servos": "pca9685",      # "pca9685" | "serial32" | "simulated"
        "imu":    "simulated",    # "mpu6050" | "simulated"
        "power":  "ads1115",      # "ina219" | "ads1115" | "simulated"
        "buzzer": "simulated",    # "gpio" | "simulated"
    }
else:
    HARDWARE = {
        "servos": "simulated",
        "imu":    "simulated",
        "power":  "simulated",
        "buzzer": "simulated",
    }

# If a real backend fails to initialise, fall back to simulation instead
# of taking the whole robot down. Set False once wiring is proven.
HARDWARE_FALLBACK_TO_SIMULATED = True


# ---- PCA9685 ---------------------------------------------------------------
PCA9685_ADDRESSES = (0x40, 0x41)
PCA9685_FREQ_HZ = 50

# Servo pulse-width range (microseconds). Standard hobby servos accept
# 1000–2000 µs over 0–180°; 500–2500 is a *wide* range that will push
# many servos past their mechanical stops. Narrow this if a joint grinds
# or clicks at an extreme.
SERVO_MIN_PULSE_US = 500
SERVO_MAX_PULSE_US = 2500


# ---- serial 32-channel controller -----------------------------------------
SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 115200


# ============================================================ control timing
TICK_RATE_HZ = 50                 # control loop
TELEMETRY_RATE_HZ = 10            # WS frames to the UI
CONTROL_TIMEOUT_SECONDS = 5.0     # drop drive if no command for this long
AUTO_SIT_IDLE_SECONDS = 30.0

# PS5 / Bluetooth gamepad (backend/gamepad.py). Needs python-evdev on the
# Pi; on the Mac it switches itself off.
GAMEPAD_ENABLED = True
GAMEPAD_DEADZONE = 0.12           # ignore stick drift below this (0..1)
GAMEPAD_RATE_HZ = 20              # how often a held stick is re-sent

# Rate limits applied last in the control loop.
# 360 °/s is about the top speed of a 35 kg servo at 7.4 V, so the
# limiter no longer slows the gait; lower it (e.g. 90 °/s) for careful
# bring-up of new hardware.
SERVO_MAX_SLEW_DEG_PER_S = 360.0
STANCE_SLEW_MM_PER_S = 60.0


# =========================================================== auto-level
AUTO_LEVEL_GAIN = 0.6                  # how aggressively to correct roll/pitch
AUTO_LEVEL_MAX_CORRECTION_DEG = 15.0   # ceiling on the correction applied
AUTO_LEVEL_SLEW_DEG_PER_S = 30.0       # how fast the correction can change


# ============================================================ sensors
# IMU
IMU_I2C_ADDRESS = 0x68
IMU_MOUNT_ROLL_OFFSET_DEG = 0.0
IMU_MOUNT_PITCH_OFFSET_DEG = 0.0

# INA219 (power monitor option A)
INA219_I2C_ADDRESS = 0x40

# ADS1115 + ACS758 (power monitor option B)
ADS1115_I2C_ADDRESS = 0x48
ADS1115_VOLTAGE_CHANNEL = 1
ADS1115_CURRENT_CHANNEL = 0
BATTERY_DIVIDER_RATIO = 5.0        # measured_V / actual_V
ACS758_ZERO_VOLTS = 2.5            # output at zero current (bidirectional, 5V VCC)
ACS758_VOLTS_PER_AMP = 0.040       # 40 mV/A for the ±50A variant

# Buzzer
BUZZER_GPIO_PIN = 18
BEEP_TONES = {
    "ok":      [(1200, 80)],
    "warn":    [(800, 120), (0, 60), (800, 120)],
    "error":   [(400, 300)],
    "startup": [(600, 80), (900, 80), (1200, 120)],
    "estop":   [(300, 200), (0, 60), (300, 200), (0, 60), (300, 200)],
    "beep":    [(1000, 100)],
}


# ============================================================== gaits
DEFAULT_GAIT = "tripod"

# Phase offsets expressed as fractions of one cycle (0.0 … 1.0). The
# generic cycle generator in gaits.py drives all of them.
def _offsets_from_groups(groups):
    n = len(groups)
    return {leg: round(i / n, 6) for i, group in enumerate(groups) for leg in group}


_WAVE_GROUPS = [["L1"], ["L2"], ["L3"], ["R3"], ["R2"], ["R1"]]
_TRIPLE_GROUPS = [["L1", "R2"], ["L2", "R3"], ["L3", "R1"]]
_RIPPLE_GROUPS = [["L1"], ["R2"], ["L3"], ["R1"], ["L2"], ["R3"]]

GAITS = {
    "tripod": {
        "label": "Tripod",
        "description": "Two alternating sets of three. Fastest, least stable.",
        "offsets": _offsets_from_groups([TRIPOD_A, TRIPOD_B]),
        "duty": 0.5,
    },
    "triple": {
        "label": "Triple",
        "description": "Three groups of two.",
        "offsets": _offsets_from_groups(_TRIPLE_GROUPS),
        "duty": 0.667,
    },
    "triple_2_5": {
        "label": "Triple 2.5",
        "description": "Overlapping stance, longer stride. Smoother, slower.",
        "offsets": _offsets_from_groups(_TRIPLE_GROUPS),
        "duty": 0.75,
    },
    "ripple": {
        "label": "Ripple",
        "description": "Two travelling waves, one per side, half a cycle apart.",
        "offsets": _offsets_from_groups(_RIPPLE_GROUPS),
        "duty": 0.75,
    },
    "ripple_1_5": {
        "label": "Ripple 1.5",
        "description": "Short stride, long stance. The most stable.",
        "offsets": _offsets_from_groups(_RIPPLE_GROUPS),
        "duty": 0.833,
    },
    "wave": {
        "label": "Wave",
        "description": "One leg at a time around the body. Slowest.",
        "offsets": _offsets_from_groups(_WAVE_GROUPS),
        "duty": 0.833,
    },
    "crab": {
        "label": "Crab walk",
        "description": "Tripod timing, travel rotated 90° to the heading.",
        "offsets": _offsets_from_groups([TRIPOD_A, TRIPOD_B]),
        "duty": 0.5,
        "crab": 90.0,
    },
}

GAIT_CYCLE_SECONDS = 0.8          # one tripod cycle at 100% speed
GAIT_STEP_HEIGHT_MM = 30.0
GAIT_STANCE_REACH_MM = 50.0

# Holding a drive input accelerates exponentially: the gait starts at
# GAIT_RAMP_START of the Speed slider and grows by e every GAIT_RAMP_TAU_S
# seconds until it reaches the slider value. Letting go resets the ramp.
GAIT_RAMP_START = 0.2
GAIT_RAMP_TAU_S = 1.0             # 0.2 -> 1.0 takes tau * ln(5), about 1.6 s


# ============================================================= stances
DEFAULT_STANCE = "stand"
# z is how far the feet are below the hip plane. Every stance keeps the
# hips at least BATTERY_DROP_MM + BATTERY_CLEARANCE_MM (75 mm) up, so the
# battery under the body never touches the ground; sit is that minimum.
STANCE_POSES = {
    "stand":  {"name": "Stand",  "x": 150.0, "y": 0.0, "z": -110.0},
    "home":   {"name": "Home",   "x": 150.0, "y": 0.0, "z": -100.0},
    "wide":   {"name": "Wide",   "x": 180.0, "y": 0.0, "z":  -95.0},
    "crouch": {"name": "Crouch", "x": 150.0, "y": 0.0, "z":  -85.0},
    "tall":   {"name": "Tall",   "x": 130.0, "y": 0.0, "z": -140.0},
    "sit":    {"name": "Sit",    "x": 160.0, "y": 0.0, "z":  -75.0},
}

# With the femur locked, the knee alone sets the foot height, so the
# foot's distance out from the body follows from the height. Recompute
# each stance's x so the targets match what the leg will really do.
if FEMUR_LOCK_DEG is not None:
    import math as _math
    _knee_x = COXA_LENGTH_MM + FEMUR_LENGTH_MM * _math.cos(_math.radians(FEMUR_LOCK_DEG))
    _knee_z = FEMUR_LENGTH_MM * _math.sin(_math.radians(FEMUR_LOCK_DEG))
    for _pose in STANCE_POSES.values():
        _drop = max(-TIBIA_LENGTH_MM, min(0.0, _pose["z"] - _knee_z))
        _pose["z"] = round(_knee_z + _drop, 1)
        _pose["x"] = round(_knee_x + _math.sqrt(TIBIA_LENGTH_MM ** 2 - _drop ** 2), 1)


# ============================================================ axis modes
# Each pad reading (horizontal, vertical) is routed into one or more
# state fields via `scale`. The field names match attributes on `state`.
DEFAULT_AXIS_MODE = "xy"
AXIS_MODES = {
    "xy": {
        "label": "X-Y",
        "description": "body slides sideways / fore-aft",
        "h": {"field": "body_x", "scale": BODY_TRANSLATE_LIMIT_MM, "unit": "mm"},
        "v": {"field": "body_y", "scale": BODY_TRANSLATE_LIMIT_MM, "unit": "mm"},
    },
    "zw": {
        "label": "Z-W",
        "description": "yaw / height",
        "h": {"field": "body_yaw", "scale": BODY_ANGLE_LIMIT_DEG, "unit": "°"},
        "v": {"field": "body_z",   "scale": BODY_Z_LIMIT_MM,      "unit": "mm"},
    },
    "uv": {
        "label": "U-V",
        "description": "roll / pitch",
        "h": {"field": "body_roll",  "scale": BODY_ANGLE_LIMIT_DEG, "unit": "°"},
        "v": {"field": "body_pitch", "scale": BODY_ANGLE_LIMIT_DEG, "unit": "°"},
    },
    "xy_uv": {
        "label": "XY-UV",
        "description": "slide + bank / slide + pitch",
        "h": [
            {"field": "body_x",    "scale": BODY_TRANSLATE_LIMIT_MM, "unit": "mm"},
            {"field": "body_roll", "scale": BODY_ANGLE_LIMIT_DEG,    "unit": "°"},
        ],
        "v": [
            {"field": "body_y",     "scale": BODY_TRANSLATE_LIMIT_MM, "unit": "mm"},
            {"field": "body_pitch", "scale": BODY_ANGLE_LIMIT_DEG,    "unit": "°"},
        ],
    },
}
