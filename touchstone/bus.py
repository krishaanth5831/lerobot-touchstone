"""The small slice of a servo bus that touchstone uses, for the real arm and the simulator.

Both buses expose the same raw interface, so the homing code that runs in the tests is
the code that runs on the arm:

    read(reg, motor) -> int          raw register value (sign already decoded)
    write(reg, motor, value)         raw register value
    torque_on(motor) / torque_off(motor)
    now() / sleep(dt)                the clock the homing loop paces itself on
    writes                           every write made, in order, for the run record
"""
from __future__ import annotations

import json
import time
from pathlib import Path

from . import config


class FeetechBus:
    """The real follower, through lerobot's FeetechMotorsBus with normalisation OFF.

    We build the bus directly instead of going through SOFollower.connect(): connect() can
    prompt for or rerun calibration, and configure() writes EEPROM (gripper limits, PID).
    The handshake only pings the motors and reads their firmware version.
    """

    def __init__(self, port: str = config.FOLLOWER_PORT, calibration_json: Path = config.CALIBRATION_JSON):
        from lerobot.motors import Motor, MotorCalibration, MotorNormMode
        from lerobot.motors.feetech import FeetechMotorsBus

        cal = json.loads(Path(calibration_json).read_text())
        self.bus = FeetechMotorsBus(
            port=port,
            motors={j: Motor(config.MOTOR_IDS[j], "sts3215", MotorNormMode.RANGE_M100_100) for j in config.JOINTS},
            calibration={j: MotorCalibration(**cal[j]) for j in config.JOINTS},
        )
        self.bus.connect(handshake=True)
        self.writes: list[tuple[float, str, str, int]] = []
        self._t0 = time.monotonic()

    def read(self, reg: str, motor: str) -> int:
        return int(self.bus.read(reg, motor, normalize=False, num_retry=2))

    def write(self, reg: str, motor: str, value: int) -> None:
        self.writes.append((round(self.now(), 3), motor, reg, int(value)))
        self.bus.write(reg, motor, int(value), normalize=False, num_retry=2)

    def torque_on(self, motor: str) -> None:
        # lerobot's enable_torque also sets Lock=1 (EEPROM write-protected)
        self.writes.append((round(self.now(), 3), motor, "Torque_Enable", 1))
        self.bus.enable_torque(motor, num_retry=2)

    def torque_off(self, motor: str) -> None:
        # ...and disable_torque sets Lock=0, which is what makes EEPROM writable
        self.writes.append((round(self.now(), 3), motor, "Torque_Enable", 0))
        self.bus.disable_torque(motor, num_retry=2)

    def now(self) -> float:
        return time.monotonic() - self._t0

    def sleep(self, dt: float) -> None:
        if dt > 0:
            time.sleep(dt)

    def close(self) -> None:
        # disable_torque=False: lerobot's default would drop every joint. Torque is left
        # exactly as touchstone's cleanup restored it.
        self.bus.disconnect(disable_torque=False)


def load_calibration(path: Path = config.CALIBRATION_JSON) -> dict[str, dict]:
    cal = json.loads(Path(path).read_text())
    missing = [j for j in config.JOINTS if j not in cal]
    if missing:
        raise ValueError(f"{path} has no entry for {missing}")
    return cal
