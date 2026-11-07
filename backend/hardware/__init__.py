"""
Hardware factory.

`create_hardware()` reads config.HARDWARE and builds one concrete
backend per peripheral. If a real backend cannot be created - library
not installed, device not plugged in, wrong address - it logs the
reason, records a warning the UI displays, and substitutes the
simulated backend, so a missing sensor never stops the robot from
walking.

Adding a new backend is two lines in the relevant _BUILDERS table.
"""

import logging
from typing import Callable, Dict

import config

from .base import Buzzer, Hardware, Imu, ImuSample, PowerMonitor, PowerSample, ServoBus
from .simulated import (SimulatedBuzzer, SimulatedImu, SimulatedPowerMonitor,
                        SimulatedServoBus)

logger = logging.getLogger("hardware")

__all__ = ["Hardware", "ServoBus", "Imu", "PowerMonitor", "Buzzer",
           "ImuSample", "PowerSample", "create_hardware",
           "SimulatedServoBus", "SimulatedImu", "SimulatedPowerMonitor",
           "SimulatedBuzzer"]


# ---------------------------------------------------------------- builders

def _build_simulated_servos() -> ServoBus:
    return SimulatedServoBus(slew_deg_per_s=config.SERVO_MAX_SLEW_DEG_PER_S)


def _build_serial_servos() -> ServoBus:
    from .servo_serial import SerialServoBus
    return SerialServoBus(config.SERIAL_PORT, config.SERIAL_BAUD,
                          config.SERVO_MIN_PULSE_US, config.SERVO_MAX_PULSE_US,
                                                     protocol="ssc32")


def _build_pca9685_servos() -> ServoBus:
    from .servo_pca9685 import Pca9685ServoBus
    return Pca9685ServoBus(config.PCA9685_ADDRESSES, config.PCA9685_FREQ_HZ,
                           config.SERVO_MIN_PULSE_US, config.SERVO_MAX_PULSE_US)


def _build_mpu6050() -> Imu:
    from .sensors import Mpu6050Imu
    return Mpu6050Imu(config.IMU_I2C_ADDRESS,
                      roll_offset=config.IMU_MOUNT_ROLL_OFFSET_DEG,
                      pitch_offset=config.IMU_MOUNT_PITCH_OFFSET_DEG)


def _build_ina219() -> PowerMonitor:
    from .sensors import Ina219PowerMonitor
    return Ina219PowerMonitor(config.INA219_I2C_ADDRESS)


def _build_ads1115() -> PowerMonitor:
    from .sensors import Ads1115PowerMonitor
    return Ads1115PowerMonitor(config.ADS1115_I2C_ADDRESS,
                               config.ADS1115_VOLTAGE_CHANNEL,
                               config.ADS1115_CURRENT_CHANNEL,
                               config.BATTERY_DIVIDER_RATIO,
                               config.ACS758_ZERO_VOLTS,
                               config.ACS758_VOLTS_PER_AMP)


def _build_gpio_buzzer() -> Buzzer:
    from .sensors import GpioBuzzer
    return GpioBuzzer(config.BUZZER_GPIO_PIN)


_BUILDERS: Dict[str, Dict[str, Callable]] = {
    "servos": {
        "simulated": _build_simulated_servos,
        "serial32": _build_serial_servos,
        "pca9685": _build_pca9685_servos,
    },
    "imu": {
        "simulated": SimulatedImu,
        "mpu6050": _build_mpu6050,
    },
    "power": {
        "simulated": SimulatedPowerMonitor,
        "ina219": _build_ina219,
        "ads1115": _build_ads1115,
    },
    "buzzer": {
        "simulated": SimulatedBuzzer,
        "gpio": _build_gpio_buzzer,
    },
}


def create_hardware() -> Hardware:
    devices: Dict[str, object] = {}
    backends: Dict[str, str] = {}
    warnings = []

    for peripheral, builders in _BUILDERS.items():
        requested = config.HARDWARE.get(peripheral, "simulated")
        builder = builders.get(requested)

        if builder is None:
            message = (f"{peripheral}: unknown backend '{requested}' "
                       f"(known: {', '.join(sorted(builders))}) - using simulation")
            logger.error(message)
            warnings.append(message)
            builder, requested = builders["simulated"], "simulated"

        try:
            devices[peripheral] = builder()
            backends[peripheral] = requested
        except Exception as exc:
            if not config.HARDWARE_FALLBACK_TO_SIMULATED:
                raise
            message = (f"{peripheral}: backend '{requested}' failed to start "
                       f"({type(exc).__name__}: {exc}) - using simulation")
            logger.warning(message)
            warnings.append(message)
            devices[peripheral] = builders["simulated"]()
            backends[peripheral] = f"simulated (wanted {requested})"

    hardware = Hardware(
        servos=devices["servos"], imu=devices["imu"],
        power=devices["power"], buzzer=devices["buzzer"],
        backends=backends, warnings=warnings,
    )
    logger.info("Hardware ready: %s", backends)
    return hardware
