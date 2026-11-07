"""
Serial servo bus driver.

Two protocols are implemented:

* PROTOCOL_LOBOT - the 0x55 0x55 binary format used by Lobot/Hiwonder
  LSC-series boards. Educated guess for the "Generic USB 32Ch Servo
  Motor Controller Board" sold under order code 2223800.1; its
  datasheet does not document the command byte format, so the real
  board may need the other one.
* PROTOCOL_SSC32 - the ASCII `#<ch>P<pulse>T<time>\\r\\n` format
  popularised by Lynxmotion's SSC-32. Also widely cloned.

The driver buffers set_angle() calls and ships a single frame per
flush(), so the 50 Hz control loop never does serial I/O inline.
"""

import logging
import threading
import time
from typing import Dict, Optional

from .base import ServoBus

logger = logging.getLogger(__name__)

PROTOCOL_LOBOT = "lobot"
PROTOCOL_SSC32 = "ssc32"
SUPPORTED_PROTOCOLS = (PROTOCOL_LOBOT, PROTOCOL_SSC32)

_LOBOT_HEADER = b"\x55\x55"
_LOBOT_CMD_MOVE = 0x03


class SerialServoBus(ServoBus):
    name = "serial32"

    def __init__(self, port: str, baud: int,
                 min_pulse_us: int, max_pulse_us: int,
                 protocol: str = PROTOCOL_LOBOT,
                 channel_count: int = 32) -> None:
        if protocol not in SUPPORTED_PROTOCOLS:
            raise ValueError(f"unknown servo protocol: {protocol!r}")
        import serial  # noqa: F401 - imported lazily so tests can stub the bus
        self._serial = serial.Serial(port, baud, timeout=0.5)
        self._lock = threading.Lock()
        self._pending: Dict[int, int] = {}
        self._last_sent: Dict[int, int] = {}
        self._closed = False
        self.min_pulse = min_pulse_us
        self.max_pulse = max_pulse_us
        self.protocol = protocol
        self.channel_count = channel_count
        logger.warning(
            "SerialServoBus open on %s @ %d (protocol=%s).",
            port, baud, protocol,
        )

    # ----------------------------------------------------- frame building

    def _angle_to_pulse(self, angle_deg: float) -> int:
        angle = max(0.0, min(180.0, float(angle_deg)))
        span = self.max_pulse - self.min_pulse
        return int(round(self.min_pulse + (angle / 180.0) * span))

    def build_frame(self, pulses: Dict[int, int], move_time_ms: int) -> bytes:
        if self.protocol == PROTOCOL_SSC32:
            body = "".join(f"#{ch}P{pulse}"
                           for ch, pulse in sorted(pulses.items()))
            return f"{body}T{move_time_ms}\r\n".encode("ascii")

        # Lobot binary move frame.
        items = sorted(pulses.items())
        payload = bytearray()
        payload.append(_LOBOT_CMD_MOVE)
        payload.append(len(items))
        payload.append(move_time_ms & 0xFF)
        payload.append((move_time_ms >> 8) & 0xFF)
        for ch, pulse in items:
            payload.append(ch & 0xFF)
            payload.append(pulse & 0xFF)
            payload.append((pulse >> 8) & 0xFF)
        # length byte spans itself through the checksum slot
        length_byte = len(payload) + 3
        header_and_len = bytes(_LOBOT_HEADER) + bytes([length_byte])
        checksum = (~sum(header_and_len[2:] + bytes(payload))) & 0xFF
        return header_and_len + bytes(payload) + bytes([checksum])

    # ----------------------------------------------------- ServoBus API

    def set_angle(self, channel: int, angle_deg: float) -> None:
        pulse = self._angle_to_pulse(angle_deg)
        with self._lock:
            self._pending[channel] = pulse

    def flush(self, move_time_ms: int = 20) -> None:
        if self._closed:
            return
        with self._lock:
            if not self._pending:
                return
            batch = dict(self._pending)
            self._pending.clear()

        changed = {ch: p for ch, p in batch.items()
                   if self._last_sent.get(ch) != p}
        if not changed:
            return

        try:
            frame = self.build_frame(changed, move_time_ms)
            self._serial.write(frame)
            self._last_sent.update(changed)
        except Exception:
            logger.exception("serial write failed")

    def close(self) -> None:
        self._closed = True
        try:
            self._serial.close()
        except Exception as exc:
            logger.warning("Serial close failed: %s", exc)


# --------------------------------------------------------- bring-up helper

def _probe(port: str, channel: int, min_pulse: int, max_pulse: int) -> None:
    """
    Gently sweep one channel under each protocol and ask which worked.
    Called from `python -m hardware.servo_serial --port ... --probe N`.
    """
    import argparse  # pragma: no cover - CLI only
    for proto in SUPPORTED_PROTOCOLS:
        print(f"\n--- trying protocol={proto} on channel {channel} ---")
        try:
            bus = SerialServoBus(port, 115200, min_pulse, max_pulse,
                                 protocol=proto)
        except Exception as exc:
            print(f"  open failed: {exc}")
            continue
        try:
            for angle in (60, 120, 90):
                bus.set_angle(channel, angle)
                bus.flush(move_time_ms=400)
                time.sleep(0.6)
        finally:
            bus.close()
        answer = input("  did the servo move? [y/N] ").strip().lower()
        if answer == "y":
            print(f"OK - set HARDWARE['servos']='serial32' and protocol={proto!r}.")
            return
    print("No protocol moved the servo. Check wiring, power and board docs.")


def main(argv: Optional[list] = None) -> None:  # pragma: no cover - CLI only
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--probe", type=int, metavar="CHANNEL",
                        help="sweep one channel under each supported protocol")
    parser.add_argument("--min-pulse", type=int, default=500)
    parser.add_argument("--max-pulse", type=int, default=2500)
    args = parser.parse_args(argv)
    if args.probe is None:
        parser.error("--probe CHANNEL is required")
    _probe(args.port, args.probe, args.min_pulse, args.max_pulse)


if __name__ == "__main__":  # pragma: no cover
    main()
