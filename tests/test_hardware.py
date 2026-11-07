"""
Hardware-layer tests.

These guard the promise that makes this project buildable before the
electronics exist: swapping a backend is a config change, a missing
device degrades to simulation instead of taking the robot down, and
every backend honours the same contract.
"""

import importlib

import pytest

import config
import hardware
from hardware.base import Buzzer, Hardware, Imu, PowerMonitor, ServoBus
from hardware.servo_serial import PROTOCOL_LOBOT, PROTOCOL_SSC32, SerialServoBus


@pytest.fixture(autouse=True)
def restore_config():
    original = dict(config.HARDWARE)
    yield
    config.HARDWARE.update(original)


def test_default_build_is_fully_simulated():
    hw = hardware.create_hardware()
    assert hw.all_simulated
    assert not hw.warnings
    assert isinstance(hw.servos, ServoBus) and isinstance(hw.imu, Imu)
    assert isinstance(hw.power, PowerMonitor) and isinstance(hw.buzzer, Buzzer)
    hw.close()


@pytest.mark.parametrize("peripheral,backend", [
    ("servos", "serial32"),
    ("servos", "pca9685"),
    ("imu", "mpu6050"),
    ("power", "ina219"),
    ("power", "ads1115"),
    ("buzzer", "gpio"),
])
def test_missing_hardware_falls_back_to_simulation(peripheral, backend):
    """
    None of these libraries or devices exist on a dev machine. The
    robot must still come up, with the reason recorded for the UI.
    """
    config.HARDWARE[peripheral] = backend
    hw = hardware.create_hardware()
    assert hw.backends[peripheral].startswith("simulated")
    assert any(peripheral in w for w in hw.warnings)
    hw.close()


def test_unknown_backend_name_falls_back_and_warns():
    config.HARDWARE["servos"] = "not-a-real-driver"
    hw = hardware.create_hardware()
    assert hw.backends["servos"] == "simulated"
    assert any("unknown backend" in w for w in hw.warnings)
    hw.close()


def test_fallback_can_be_disabled():
    """On the real robot you may want a missing servo bus to be fatal."""
    config.HARDWARE["servos"] = "serial32"
    original = config.HARDWARE_FALLBACK_TO_SIMULATED
    config.HARDWARE_FALLBACK_TO_SIMULATED = False
    try:
        with pytest.raises(Exception):
            hardware.create_hardware()
    finally:
        config.HARDWARE_FALLBACK_TO_SIMULATED = original


def test_one_failure_does_not_take_out_the_others():
    config.HARDWARE["imu"] = "mpu6050"
    hw = hardware.create_hardware()
    assert hw.backends["imu"].startswith("simulated")
    assert hw.backends["servos"] == "simulated"      # unaffected
    assert hw.power.read().voltage is not None       # still working
    hw.close()


# ----------------------------------------------------- servo contract

def test_simulated_servos_clamp_and_report():
    bus = hardware.SimulatedServoBus()
    bus.set_angle(0, 500)
    bus.set_angle(1, -500)
    bus.flush()
    bus.flush()
    assert bus.read_angle(0) == 180.0
    assert bus.read_angle(1) == 0.0


def test_simulated_servos_respect_the_slew_limit():
    bus = hardware.SimulatedServoBus(slew_deg_per_s=90.0)
    bus.set_angle(0, 0)
    bus.flush()
    bus._last_flush -= 0.5              # pretend half a second passed
    bus.set_angle(0, 180)
    bus.flush()
    assert bus.read_angle(0) == pytest.approx(45.0, abs=1.0)


def test_torque_off_freezes_the_simulated_servos():
    bus = hardware.SimulatedServoBus()
    bus.set_angle(0, 90)
    bus.flush()
    bus.flush()
    bus.set_torque(False)
    bus.set_angle(0, 20)
    bus.flush()
    assert bus.read_angle(0) == 90.0


def test_simulated_imu_tracks_the_virtual_ground():
    imu = hardware.SimulatedImu(noise_deg=0.0)
    imu.set_ground_tilt(12.0, -6.0)
    sample = imu.read()
    assert sample.roll_deg == pytest.approx(12.0)
    assert sample.pitch_deg == pytest.approx(-6.0)
    imu.calibrate_level()
    levelled = imu.read()
    assert levelled.roll_deg == pytest.approx(0.0)
    assert levelled.pitch_deg == pytest.approx(0.0)


def test_simulated_power_responds_to_load():
    power = hardware.SimulatedPowerMonitor()
    power.activity = 0.0
    idle = power.read()
    power.activity = 1.0
    busy = power.read()
    assert busy.current > idle.current
    assert busy.voltage < idle.voltage      # the pack sags under load
    assert busy.watts > 0


def test_buzzer_records_what_it_played():
    buzzer = hardware.SimulatedBuzzer()
    buzzer.play(config.BEEP_TONES["estop"])
    assert buzzer.history[-1][1] == config.BEEP_TONES["estop"]


# --------------------------------------------- serial frame encoding

def _bus_without_a_port():
    """Build a SerialServoBus with the serial port stubbed out."""
    bus = SerialServoBus.__new__(SerialServoBus)
    bus.min_pulse, bus.max_pulse = 500, 2500
    bus.channel_count = 32
    bus.protocol = PROTOCOL_LOBOT
    return bus


def test_angle_to_pulse_spans_the_configured_range():
    bus = _bus_without_a_port()
    assert bus._angle_to_pulse(0) == 500
    assert bus._angle_to_pulse(180) == 2500
    assert bus._angle_to_pulse(90) == 1500
    assert bus._angle_to_pulse(400) == 2500     # clamped, not extrapolated


def test_lobot_frame_structure():
    bus = _bus_without_a_port()
    frame = bus.build_frame({0: 1500, 1: 1600}, 200)
    assert frame[0:2] == b"\x55\x55"
    assert frame[2] == len(frame) - 3 + 2       # length byte counts itself + checksum slot
    assert frame[3] == 0x03                     # move command
    assert frame[4] == 2                        # servo count
    assert frame[5] | (frame[6] << 8) == 200    # move time, little endian
    assert frame[7] == 0 and frame[8] | (frame[9] << 8) == 1500


def test_ssc32_frame_structure():
    bus = _bus_without_a_port()
    bus.protocol = PROTOCOL_SSC32
    frame = bus.build_frame({0: 1500, 3: 1200}, 250)
    assert frame == b"#0P1500#3P1200T250\r\n"


def test_channel_map_covers_every_joint_uniquely():
    channels = [config.SERVO_CHANNELS[leg][joint]
                for leg in config.LEG_IDS for joint in config.JOINTS]
    assert len(channels) == 18
    assert len(set(channels)) == 18, "two joints share a servo channel"
    assert max(channels) < hardware.SimulatedServoBus().channel_count


class _FakePca:
    """Stand-in for adafruit_pca9685.PCA9685 with real register behaviour."""
    fail_frequency_writes = 0

    def __init__(self, i2c, address=0x40):
        self.address = address
        self.reference_clock_speed = 25_000_000
        self.prescale_reg = 0x1E          # power-on default, about 200 Hz
        self.mode1_reg = 0x00             # the library's constructor resets MODE1
        self.channels = [type("Ch", (), {"duty_cycle": 0})() for _ in range(16)]

    @property
    def frequency(self):
        return self.reference_clock_speed / 4096 / (self.prescale_reg + 1)

    @frequency.setter
    def frequency(self, freq):
        if _FakePca.fail_frequency_writes:
            _FakePca.fail_frequency_writes -= 1
            raise OSError(5, "Input/output error")
        # Same as adafruit_pca9685: the old MODE1 (sleep bit included)
        # is restored, plus RESTART and auto-increment.
        old_mode = self.mode1_reg
        self.prescale_reg = int(self.reference_clock_speed / 4096.0 / freq + 0.5) - 1
        self.mode1_reg = old_mode | 0xA0

    def deinit(self):
        pass


@pytest.fixture
def fake_pca9685(monkeypatch):
    import sys
    import types
    monkeypatch.setitem(sys.modules, "board", types.SimpleNamespace(SCL=3, SDA=2))
    monkeypatch.setitem(sys.modules, "busio", types.SimpleNamespace(I2C=lambda scl, sda: object()))
    monkeypatch.setitem(sys.modules, "adafruit_pca9685", types.SimpleNamespace(PCA9685=_FakePca))
    import hardware.servo_pca9685 as driver
    monkeypatch.setattr(driver.time, "sleep", lambda s: None)
    _FakePca.fail_frequency_writes = 0
    return driver


def test_pca9685_retries_a_failed_50hz_write_at_startup(fake_pca9685):
    _FakePca.fail_frequency_writes = 2          # two I2C errors, then success
    bus = fake_pca9685.Pca9685ServoBus((0x40, 0x41))
    assert all(b.prescale_reg == 121 for b in bus._boards)


def test_pca9685_repairs_a_board_that_lost_50hz_and_resends_its_servos(fake_pca9685, monkeypatch):
    bus = fake_pca9685.Pca9685ServoBus((0x40, 0x41))
    bus.set_angle(18, 90.0)                     # board 2, pin 2 (L1 femur)
    bus.set_angle(0, 45.0)
    bus.flush()
    good = bus._boards[1].channels[2].duty_cycle

    second = bus._boards[1]
    second.prescale_reg, second.mode1_reg = 0x1E, 0x11     # brown-out reset
    second.channels[2].duty_cycle = 0
    bus._next_check = 0                                    # health check is due
    bus.flush()
    bus.flush()                                            # resent servos go out
    assert second.prescale_reg == 121 and not second.mode1_reg & 0x10
    assert second.channels[2].duty_cycle == good


def test_pca9685_repair_does_not_re_energise_after_torque_off(fake_pca9685):
    bus = fake_pca9685.Pca9685ServoBus((0x40, 0x41))
    bus.set_angle(18, 90.0)
    bus.flush()
    bus.set_torque(False)
    bus._boards[1].prescale_reg = 0x1E
    bus._next_check = 0
    bus.flush()
    bus.flush()
    assert bus._boards[1].channels[2].duty_cycle == 0


class _CountingChannel:
    def __init__(self):
        self.writes = 0
        self._duty = 0

    @property
    def duty_cycle(self):
        return self._duty

    @duty_cycle.setter
    def duty_cycle(self, value):
        self.writes += 1
        self._duty = value


def test_pca9685_skips_writes_that_would_not_change_the_pulse(fake_pca9685):
    bus = fake_pca9685.Pca9685ServoBus((0x40, 0x41))
    ch = bus._boards[0].channels[1] = _CountingChannel()
    for _ in range(10):                  # robot standing still for 10 ticks
        bus.set_angle(1, 90.0)
        bus.flush()
    assert ch.writes == 1
    bus.set_angle(1, 100.0)
    bus.flush()
    assert ch.writes == 2


def test_pca9685_torque_on_after_off_rewrites_the_pulse(fake_pca9685):
    bus = fake_pca9685.Pca9685ServoBus((0x40, 0x41))
    bus.set_angle(1, 90.0)
    bus.flush()
    good = bus._boards[0].channels[1].duty_cycle
    bus.set_torque(False)
    bus.set_angle(1, 90.0)               # same angle as before torque-off
    bus.flush()
    assert bus._boards[0].channels[1].duty_cycle == good


def test_pca9685_failed_write_is_retried_next_tick(fake_pca9685):
    class Flaky(_CountingChannel):
        fail = True

        @_CountingChannel.duty_cycle.setter
        def duty_cycle(self, value):
            if Flaky.fail:
                Flaky.fail = False
                raise OSError(5, "Input/output error")
            _CountingChannel.duty_cycle.fset(self, value)

    bus = fake_pca9685.Pca9685ServoBus((0x40, 0x41))
    ch = bus._boards[0].channels[1] = Flaky()
    bus.set_angle(1, 90.0)
    bus.flush()                          # I2C error
    bus.set_angle(1, 90.0)
    bus.flush()                          # same angle must still go out
    assert ch.writes == 1 and ch.duty_cycle == bus._duty_for(90.0)


def test_pca9685_repair_wakes_a_board_that_restarted_asleep(fake_pca9685):
    bus = fake_pca9685.Pca9685ServoBus((0x40, 0x41))
    bus.set_angle(1, 90.0)
    bus.flush()
    first = bus._boards[0]
    first.prescale_reg, first.mode1_reg = 0x1E, 0x11       # brown-out: 200 Hz, asleep
    bus._next_check = 0
    bus.flush()
    assert first.prescale_reg == 121
    assert not first.mode1_reg & 0x10, "board left asleep - its servos stay limp"
