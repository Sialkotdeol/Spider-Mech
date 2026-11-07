"""
Hardware interfaces for Spider Mech.

Everything above this package talks to the robot through these four
abstract classes and never imports a device library directly. That is
the whole point: when the real electronics are assembled, you write
(or enable) a concrete subclass here and change one word in
config.HARDWARE - no controller, gait, kinematics or UI code changes.

Contract notes for anyone writing a new backend:
  * set_angle() takes a SERVO angle in degrees (0..180), already
    calibrated. Kinematic-to-servo conversion happens in
    backend/calibration.py, above this layer.
  * set_angle() must be cheap and non-blocking - it is called ~900
    times a second (18 joints x 50 Hz). Buffer, and do the actual I/O
    in flush().
  * flush() is called once per control tick with the whole batch.
  * read()/read_angle() may return None when a value isn't available;
    callers must handle that.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple


@dataclass
class ImuSample:
    roll_deg: float
    pitch_deg: float
    yaw_deg: float = 0.0
    accel_g: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    gyro_dps: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    temperature_c: Optional[float] = None


@dataclass
class PowerSample:
    voltage: Optional[float] = None
    current: Optional[float] = None

    @property
    def watts(self) -> Optional[float]:
        if self.voltage is None or self.current is None:
            return None
        return self.voltage * self.current


class ServoBus(ABC):
    """A bank of position-controlled servos addressed by channel index."""

    name = "servo-bus"
    channel_count = 32
    supports_torque_control = False
    supports_feedback = False

    @abstractmethod
    def set_angle(self, channel: int, angle_deg: float) -> None:
        """Queue one servo angle (0..180 servo degrees). Non-blocking."""

    def set_angles(self, angles: Dict[int, float]) -> None:
        for channel, angle in angles.items():
            self.set_angle(channel, angle)

    def flush(self, move_time_ms: int = 20) -> None:
        """Commit the queued batch to the hardware."""

    def set_torque(self, enabled: bool) -> None:
        """Energise or release the servos. No-op where unsupported."""

    def read_angle(self, channel: int) -> Optional[float]:
        """Read a servo's actual position, if the bus can report it."""
        return None

    def close(self) -> None:
        pass


class Imu(ABC):
    name = "imu"

    @abstractmethod
    def read(self) -> Optional[ImuSample]:
        """Latest orientation, or None if unavailable."""

    def calibrate_level(self) -> None:
        """Treat the current orientation as level (zero the offsets)."""

    def close(self) -> None:
        pass


class PowerMonitor(ABC):
    name = "power"

    @abstractmethod
    def read(self) -> PowerSample:
        """Latest battery voltage and current draw."""

    def close(self) -> None:
        pass


class Buzzer(ABC):
    name = "buzzer"

    @abstractmethod
    def play(self, tones: List[Tuple[int, int]]) -> None:
        """Play a sequence of (frequency_hz, duration_ms). 0 Hz = rest."""

    def close(self) -> None:
        pass


@dataclass
class Hardware:
    """Everything the controller needs, bundled."""
    servos: ServoBus
    imu: Imu
    power: PowerMonitor
    buzzer: Buzzer
    backends: Dict[str, str] = field(default_factory=dict)
    warnings: List[str] = field(default_factory=list)

    @property
    def all_simulated(self) -> bool:
        return all(v.startswith("simulated") for v in self.backends.values())

    def close(self) -> None:
        for device in (self.servos, self.imu, self.power, self.buzzer):
            try:
                device.close()
            except Exception:
                pass
