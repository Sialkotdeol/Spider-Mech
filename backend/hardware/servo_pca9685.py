"""
PCA9685 servo backend.

An 18-DOF hexapod needs more than one 16-channel PCA9685, so this
driver chains two boards at different I2C addresses: channels 0-15 go
to the first, 16-31 to the second. Wire the second board's A0 jumper
to make it 0x41.

Requires `adafruit-circuitpython-pca9685` (or `Adafruit_PCA9685`).
Unlike the serial controller, this protocol is completely standard -
there is no guesswork here. The tradeoff is that PCA9685 has no
position feedback and no per-servo torque command; torque is released
by simply stopping the PWM pulses, which every hobby servo respects.
"""

import logging
import threading
import time
from typing import Dict, List, Optional

from .base import ServoBus

logger = logging.getLogger("hardware.servo_pca9685")

_MODE1_SLEEP = 0x10
# How often flush() re-reads each board's PRESCALE and MODE1 registers.
# A board that browns out resets to ~200 Hz (prescale 0x1E) or to sleep,
# and an I2C error during start-up can leave the 50 Hz write undone;
# either way its servos stop following commands until it is fixed.
_HEALTH_CHECK_S = 2.0
# A failing bus fails every write in every tick; log it at most this often.
_ERROR_LOG_S = 5.0


class Pca9685ServoBus(ServoBus):
    name = "pca9685"
    supports_torque_control = True

    def __init__(self, addresses=(0x40, 0x41), freq_hz: int = 50,
                 min_pulse: int = 500, max_pulse: int = 2500):
        import board
        import busio
        from adafruit_pca9685 import PCA9685

        self.min_pulse = min_pulse
        self.max_pulse = max_pulse
        self.freq_hz = freq_hz
        i2c = busio.I2C(board.SCL, board.SDA)
        self._boards: List = []
        for address in addresses:
            pca = PCA9685(i2c, address=address)
            self._boards.append(pca)
            self._configure(len(self._boards) - 1)
        self.channel_count = 16 * len(self._boards)
        self._pending: Dict[int, float] = {}
        self._last: Dict[int, float] = {}
        # Duty last written to each channel. The control loop sets every
        # servo every tick; only a changed duty goes out on the slow I2C
        # bus, which is what lets the loop keep up at TICK_RATE_HZ.
        self._duty: Dict[int, int] = {}
        self._failures = 0
        self._next_error_log = 0.0
        self._next_check = time.monotonic() + _HEALTH_CHECK_S
        self._lock = threading.Lock()
        logger.info("PCA9685 bus ready: %d channels across %d boards",
                    self.channel_count, len(self._boards))

    def _expected_prescale(self, pca) -> int:
        clock = getattr(pca, "reference_clock_speed", 25_000_000)
        return int(clock / 4096.0 / self.freq_hz + 0.5) - 1

    def _board_ok(self, index: int) -> bool:
        pca = self._boards[index]
        return (pca.prescale_reg == self._expected_prescale(pca)
                and not pca.mode1_reg & _MODE1_SLEEP)

    def _configure(self, index: int, attempts: int = 3) -> bool:
        """Set the board to freq_hz and read it back, retrying I2C errors."""
        pca = self._boards[index]
        for attempt in range(1, attempts + 1):
            try:
                # A board that browned out restarts asleep (MODE1 0x11) with
                # its outputs off. The library's frequency setter restores
                # the old MODE1, sleep bit included, so wake it first.
                pca.mode1_reg = 0x00
                time.sleep(0.001)
                pca.frequency = self.freq_hz
                if self._board_ok(index):
                    return True
                logger.warning("PCA9685 board %d: prescale %d, MODE1 0x%02X after setting %d Hz "
                               "(want prescale %d, awake), retrying", index, pca.prescale_reg,
                               pca.mode1_reg, self.freq_hz, self._expected_prescale(pca))
            except OSError as exc:
                logger.warning("PCA9685 board %d: I2C error setting %d Hz (attempt %d/%d): %s",
                               index, self.freq_hz, attempt, attempts, exc)
            time.sleep(0.01)
        logger.error("PCA9685 board %d is not running at %d Hz - its servos will not follow "
                     "commands. Check its power, GND and SDA/SCL wiring.", index, self.freq_hz)
        return False

    def _check_boards(self) -> None:
        """Re-apply 50 Hz to any board that lost it, then resend its servos."""
        for index in range(len(self._boards)):
            try:
                if self._board_ok(index):
                    continue
            except OSError as exc:
                logger.warning("PCA9685 board %d: health check read failed: %s", index, exc)
                continue
            logger.warning("PCA9685 board %d lost its %d Hz setting (reset or failed write) - "
                           "reconfiguring", index, self.freq_hz)
            if self._configure(index):
                with self._lock:
                    for channel, angle in self._last.items():
                        if channel // 16 == index:
                            self._duty.pop(channel, None)   # the reset cleared it
                            self._pending.setdefault(channel, angle)

    def _duty_for(self, angle_deg: float) -> int:
        angle_deg = max(0.0, min(180.0, float(angle_deg)))
        pulse_us = self.min_pulse + (angle_deg / 180.0) * (self.max_pulse - self.min_pulse)
        period_us = 1_000_000.0 / self.freq_hz
        return int(max(0, min(0xFFFF, round(pulse_us / period_us * 0xFFFF))))

    def set_angle(self, channel: int, angle_deg: float) -> None:
        with self._lock:
            self._pending[channel] = angle_deg

    def flush(self, move_time_ms: int = 20) -> None:
        now = time.monotonic()
        if now >= self._next_check:
            self._next_check = now + _HEALTH_CHECK_S
            self._check_boards()
        with self._lock:
            batch = dict(self._pending)
            self._pending.clear()
        for channel, angle in batch.items():
            board_index, local = divmod(int(channel), 16)
            if board_index >= len(self._boards):
                continue
            duty = self._duty_for(angle)
            self._last[channel] = angle
            if self._duty.get(channel) == duty:
                continue
            try:
                self._boards[board_index].channels[local].duty_cycle = duty
                self._duty[channel] = duty
            except OSError as exc:
                self._duty.pop(channel, None)
                self._log_write_failure(channel, exc)

    def _log_write_failure(self, channel: int, exc: OSError) -> None:
        self._failures += 1
        now = time.monotonic()
        if now < self._next_error_log:
            return
        self._next_error_log = now + _ERROR_LOG_S
        logger.error("PCA9685 writes failing (%d in the last %.0f s, latest channel %d: %s) - "
                     "check board VCC (Pi 3.3 V), GND and SDA/SCL wiring",
                     self._failures, _ERROR_LOG_S, channel, exc)
        self._failures = 0

    def set_torque(self, enabled: bool) -> None:
        if enabled:
            return   # the next position frame re-energises the servos
        with self._lock:
            self._last.clear()   # a board repair must not re-energise limp servos
            self._duty.clear()
        for pca in self._boards:
            for channel in pca.channels:
                try:
                    channel.duty_cycle = 0
                except Exception:
                    pass

    def close(self) -> None:
        for pca in self._boards:
            try:
                pca.deinit()
            except Exception:
                pass
