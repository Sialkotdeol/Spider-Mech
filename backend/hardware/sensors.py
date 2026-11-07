"""
Real sensor and buzzer backends: MPU6050 IMU, INA219 / ADS1115 power
monitoring, GPIO buzzer.

Each class imports its device library lazily inside __init__, so the
simulated build never needs any of them installed. If a board is
missing at runtime the factory in __init__.py catches the failure and
falls back to simulation rather than taking the robot down.
"""

import logging
import math
import threading
import time
from typing import List, Optional, Tuple

from .base import Buzzer, Imu, ImuSample, PowerMonitor, PowerSample

logger = logging.getLogger("hardware.sensors")


class Mpu6050Imu(Imu):
    """
    MPU6050 over I2C, with a complementary filter fusing the
    accelerometer (stable but noisy) and the gyro (smooth but drifts).

    Roll/pitch from this are what feeds auto-level, so the filter
    constant matters: TAU is the time over which the accelerometer
    pulls the estimate back. 0.35s is a reasonable start for a walking
    robot - lower it if levelling feels sluggish, raise it if leg
    impacts make the body twitch.
    """

    name = "mpu6050"
    TAU = 0.35

    _PWR_MGMT_1 = 0x6B
    _ACCEL_XOUT_H = 0x3B
    _CONFIG = 0x1A
    _GYRO_CONFIG = 0x1B
    _ACCEL_CONFIG = 0x1C

    def __init__(self, address: int = 0x68, bus_number: int = 1,
                 roll_offset: float = 0.0, pitch_offset: float = 0.0):
        import smbus2

        self.address = address
        self._bus = smbus2.SMBus(bus_number)
        self._bus.write_byte_data(address, self._PWR_MGMT_1, 0x00)   # wake
        time.sleep(0.05)
        self._bus.write_byte_data(address, self._CONFIG, 0x03)        # 44Hz DLPF
        self._bus.write_byte_data(address, self._GYRO_CONFIG, 0x00)   # +/-250 dps
        self._bus.write_byte_data(address, self._ACCEL_CONFIG, 0x00)  # +/-2 g

        self._offset = [roll_offset, pitch_offset]
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0
        self._last = time.time()
        self._seeded = False
        self._lock = threading.Lock()
        logger.info("MPU6050 ready at 0x%02X", address)

    def _read_word(self, reg: int) -> int:
        high, low = self._bus.read_i2c_block_data(self.address, reg, 2)
        value = (high << 8) | low
        return value - 65536 if value >= 0x8000 else value

    def read(self) -> Optional[ImuSample]:
        try:
            with self._lock:
                raw = self._bus.read_i2c_block_data(self.address, self._ACCEL_XOUT_H, 14)
        except Exception:
            logger.exception("MPU6050 read failed")
            return None

        def word(index: int) -> int:
            value = (raw[index] << 8) | raw[index + 1]
            return value - 65536 if value >= 0x8000 else value

        ax, ay, az = (word(0) / 16384.0, word(2) / 16384.0, word(4) / 16384.0)
        temperature = word(6) / 340.0 + 36.53
        gx, gy, gz = (word(8) / 131.0, word(10) / 131.0, word(12) / 131.0)

        accel_roll = math.degrees(math.atan2(ay, math.hypot(ax, az)))
        accel_pitch = math.degrees(math.atan2(-ax, math.hypot(ay, az)))

        now = time.time()
        dt = min(max(now - self._last, 0.0), 0.2)
        self._last = now

        if not self._seeded:
            self._roll, self._pitch, self._seeded = accel_roll, accel_pitch, True
        else:
            alpha = self.TAU / (self.TAU + dt) if dt > 0 else 1.0
            self._roll = alpha * (self._roll + gx * dt) + (1 - alpha) * accel_roll
            self._pitch = alpha * (self._pitch + gy * dt) + (1 - alpha) * accel_pitch
        self._yaw += gz * dt   # drifts without a magnetometer; indicative only

        return ImuSample(
            roll_deg=self._roll - self._offset[0],
            pitch_deg=self._pitch - self._offset[1],
            yaw_deg=self._yaw,
            accel_g=(ax, ay, az),
            gyro_dps=(gx, gy, gz),
            temperature_c=round(temperature, 1),
        )

    def calibrate_level(self) -> None:
        sample = self.read()
        if sample is not None:
            self._offset[0] += sample.roll_deg
            self._offset[1] += sample.pitch_deg

    def close(self) -> None:
        try:
            self._bus.close()
        except Exception:
            pass


class Ina219PowerMonitor(PowerMonitor):
    """INA219 high-side current/voltage monitor. Simple and accurate."""

    name = "ina219"

    def __init__(self, address: int = 0x40):
        import board
        import busio
        from adafruit_ina219 import INA219

        self._sensor = INA219(busio.I2C(board.SCL, board.SDA), addr=address)
        logger.info("INA219 ready at 0x%02X", address)

    def read(self) -> PowerSample:
        try:
            voltage = self._sensor.bus_voltage + self._sensor.shunt_voltage
            return PowerSample(voltage=round(voltage, 2),
                               current=round(self._sensor.current / 1000.0, 2))
        except Exception:
            logger.exception("INA219 read failed")
            return PowerSample()


class Ads1115PowerMonitor(PowerMonitor):
    """
    ADS1115 ADC reading a resistor divider (battery voltage) and an
    ACS758 hall-effect current sensor.

    The ACS758 outputs `zero_volts` at no current and swings by
    `volts_per_amp`; both are part numbers off the sensor's datasheet,
    so set them to match the variant you actually fitted.
    """

    name = "ads1115"

    def __init__(self, address: int = 0x48,
                 voltage_channel: int = 0, current_channel: int = 1,
                 divider_ratio: float = 5.7,
                 acs_zero_volts: float = 2.5, acs_volts_per_amp: float = 0.040):
        import adafruit_ads1x15.ads1115 as ADS
        import board
        import busio
        from adafruit_ads1x15.analog_in import AnalogIn

        self._ads = ADS.ADS1115(busio.I2C(board.SCL, board.SDA), address=address)
        # Older adafruit-circuitpython-ads1x15 exposed ADS.P0..P3; newer
        # versions dropped them. AnalogIn accepts the integer channel.
        self._voltage_in = AnalogIn(self._ads, voltage_channel)
        self._current_in = AnalogIn(self._ads, current_channel)
        self.divider_ratio = divider_ratio
        self.acs_zero_volts = acs_zero_volts
        self.acs_volts_per_amp = acs_volts_per_amp
        self._next_error_log = 0.0
        logger.info("ADS1115 ready at 0x%02X", address)

    def read(self) -> PowerSample:
        try:
            voltage = self._voltage_in.voltage * self.divider_ratio
            current = (self._current_in.voltage - self.acs_zero_volts) / self.acs_volts_per_amp
            return PowerSample(voltage=round(voltage, 2), current=round(current, 2))
        except OSError as exc:
            now = time.monotonic()
            if now >= self._next_error_log:
                self._next_error_log = now + 5.0
                logger.error("ADS1115 read failed: %s", exc)
            return PowerSample()


class GpioBuzzer(Buzzer):
    """
    Passive piezo buzzer on a GPIO pin, driven with software PWM so it
    can play distinct tones. Plays in a background thread so a beep
    never stalls the control loop.
    """

    name = "gpio-buzzer"

    def __init__(self, pin: int = 18):
        import RPi.GPIO as GPIO

        self._gpio = GPIO
        self.pin = pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin, GPIO.OUT)
        self._pwm = GPIO.PWM(pin, 1000)
        self._lock = threading.Lock()
        logger.info("GPIO buzzer ready on BCM pin %d", pin)

    def play(self, tones: List[Tuple[int, int]]) -> None:
        threading.Thread(target=self._play_blocking, args=(list(tones),), daemon=True).start()

    def _play_blocking(self, tones: List[Tuple[int, int]]) -> None:
        with self._lock:
            for frequency, duration_ms in tones:
                try:
                    if frequency > 0:
                        self._pwm.ChangeFrequency(frequency)
                        self._pwm.start(50)
                    else:
                        self._pwm.stop()
                    time.sleep(duration_ms / 1000.0)
                finally:
                    self._pwm.stop()

    def close(self) -> None:
        try:
            self._pwm.stop()
            self._gpio.cleanup(self.pin)
        except Exception:
            pass
