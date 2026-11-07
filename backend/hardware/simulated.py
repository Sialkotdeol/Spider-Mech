"""
Simulated hardware backends.

These are not stubs - they are a working model of the robot's
electronics, so the entire system (kinematics, gaits, moves, API, 3D
UI) can be developed and exercised with nothing plugged in.

The simulated servos even model slew rate, so the UI shows the same
lag a real servo has, and the simulated IMU derives roll/pitch from a
virtual ground plane you can tilt from the UI - which means auto-level
can be tested for real before the robot exists.
"""

import math
import random
import threading
import time
from typing import Dict, List, Optional, Tuple

from .base import Buzzer, Imu, ImuSample, PowerMonitor, PowerSample, ServoBus


class SimulatedServoBus(ServoBus):
    """Tracks commanded and (rate-limited) actual angles in memory."""

    name = "simulated-servos"
    supports_torque_control = True
    supports_feedback = True

    def __init__(self, channel_count: int = 32, slew_deg_per_s: float = 320.0):
        self.channel_count = channel_count
        self.slew_deg_per_s = slew_deg_per_s
        self._commanded: Dict[int, float] = {}
        self._actual: Dict[int, float] = {}
        self._pending: Dict[int, float] = {}
        self._torque = True
        self._last_flush = time.time()
        self._lock = threading.Lock()
        self.flush_count = 0

    def set_angle(self, channel: int, angle_deg: float) -> None:
        angle_deg = max(0.0, min(180.0, float(angle_deg)))
        with self._lock:
            self._pending[channel] = angle_deg

    def flush(self, move_time_ms: int = 20) -> None:
        now = time.time()
        with self._lock:
            dt = max(0.0, now - self._last_flush)
            self._last_flush = now
            self.flush_count += 1
            self._commanded.update(self._pending)
            self._pending.clear()
            if not self._torque:
                return
            max_step = self.slew_deg_per_s * dt if dt > 0 else 180.0
            for channel, target in self._commanded.items():
                current = self._actual.get(channel, target)
                delta = target - current
                if abs(delta) > max_step:
                    delta = math.copysign(max_step, delta)
                self._actual[channel] = current + delta

    def set_torque(self, enabled: bool) -> None:
        with self._lock:
            self._torque = bool(enabled)

    def read_angle(self, channel: int) -> Optional[float]:
        with self._lock:
            value = self._actual.get(channel)
        return None if value is None else round(value, 2)

    def snapshot(self) -> Dict[int, float]:
        with self._lock:
            return {ch: round(v, 2) for ch, v in self._actual.items()}


class SimulatedImu(Imu):
    """
    Reports the tilt of a virtual ground plane. Set that tilt from the
    UI (or /api/sim/tilt) to watch auto-level respond with no robot.
    """

    name = "simulated-imu"

    def __init__(self, noise_deg: float = 0.15):
        self.noise_deg = noise_deg
        self.ground_roll_deg = 0.0
        self.ground_pitch_deg = 0.0
        self._offset = (0.0, 0.0)
        self._yaw = 0.0
        self._last = time.time()

    def set_ground_tilt(self, roll_deg: float, pitch_deg: float) -> None:
        self.ground_roll_deg = float(roll_deg)
        self.ground_pitch_deg = float(pitch_deg)

    def read(self) -> Optional[ImuSample]:
        now = time.time()
        self._last = now
        n = self.noise_deg
        roll = self.ground_roll_deg - self._offset[0] + random.uniform(-n, n)
        pitch = self.ground_pitch_deg - self._offset[1] + random.uniform(-n, n)
        r, p = math.radians(roll), math.radians(pitch)
        return ImuSample(
            roll_deg=roll,
            pitch_deg=pitch,
            yaw_deg=self._yaw,
            accel_g=(-math.sin(p), math.sin(r) * math.cos(p), math.cos(r) * math.cos(p)),
            gyro_dps=(0.0, 0.0, 0.0),
            temperature_c=31.0,
        )

    def calibrate_level(self) -> None:
        self._offset = (self.ground_roll_deg, self.ground_pitch_deg)


class SimulatedPowerMonitor(PowerMonitor):
    """A 3S LiPo draining slowly, with current that responds to motion."""

    name = "simulated-power"

    def __init__(self, nominal_volts: float = 11.6):
        self.nominal_volts = nominal_volts
        self._start = time.time()
        self.activity = 0.0   # 0..1, set by the controller each tick

    def read(self) -> PowerSample:
        elapsed = time.time() - self._start
        sag = 0.35 * self.activity
        drift = 0.00008 * elapsed
        voltage = self.nominal_volts - sag - drift + random.uniform(-0.02, 0.02)
        current = 1.1 + 4.2 * self.activity + random.uniform(-0.05, 0.05)
        return PowerSample(voltage=round(voltage, 2), current=round(current, 2))


class SimulatedBuzzer(Buzzer):
    """Records what would have been played, so the UI can confirm it."""

    name = "simulated-buzzer"

    def __init__(self):
        self.history: List[Tuple[float, List[Tuple[int, int]]]] = []

    def play(self, tones: List[Tuple[int, int]]) -> None:
        self.history.append((time.time(), list(tones)))
        del self.history[:-20]
