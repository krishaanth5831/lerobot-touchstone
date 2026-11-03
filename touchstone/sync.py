"""Write one joint's calibration from the lerobot JSON back into its motor, torque OFF.

The motor's EEPROM holds a copy of the calibration: Homing_Offset (where its zero is) and
Min/Max_Position_Limit (the firmware clamp). When that copy drifts from the JSON, homing
refuses the joint. lerobot can rewrite it ("connect and press ENTER"), but its connect()
switches torque on for the whole arm afterwards. This writes three registers on one motor
and never enables torque anywhere, so nothing can move.

Changing Homing_Offset changes what Present_Position reads for the same physical pose.
That is the point: the JSON's offset is the one every recording was made with.
"""
from __future__ import annotations

from typing import Callable

from . import config

FIELDS = (("homing_offset", "Homing_Offset"), ("range_min", "Min_Position_Limit"),
          ("range_max", "Max_Position_Limit"))


class SyncRefused(Exception):
    """A check failed before anything was written."""


def motor_values(bus, joint: str) -> dict[str, int]:
    return {k: bus.read(r, joint) for k, r in FIELDS}


def sync_joint(bus, cal: dict, joint: str, confirm: Callable[[str], bool] = lambda msg: True,
               on_event: Callable[[str], None] = print) -> dict:
    """Returns {"before", "after", "pos_before", "pos_after", "changed"}. Raises SyncRefused."""
    if joint not in config.JOINTS:
        raise SyncRefused(f"unknown joint {joint!r}")
    want = {k: cal[joint][k] for k, _ in FIELDS}
    volt = bus.read("Present_Voltage", joint) / 10
    if not config.VOLTAGE_MIN_V <= volt <= config.VOLTAGE_MAX_V:
        raise SyncRefused(f"{joint}: {volt:.1f} V, this is not the follower on its 12 V supply")
    if bus.read("Torque_Enable", joint):
        raise SyncRefused(f"{joint}: torque is ON. Rest the arm with torque off first")
    before = motor_values(bus, joint)
    pos_before = bus.read("Present_Position", joint)
    out = {"before": before, "pos_before": pos_before, "after": before, "pos_after": pos_before, "changed": False}
    if before == want:
        on_event(f"{joint}: motor already matches the JSON {want}. Nothing written.")
        return out
    lines = "\n".join(f"  {k:<14} motor {before[k]:>6}  ->  json {want[k]:>6}" for k in want)
    if not confirm(f"{joint}: write these from the JSON into the motor (torque stays off, nothing moves):\n{lines}"):
        raise SyncRefused("not confirmed, nothing written")
    bus.torque_off(joint)            # already off; this also sets Lock=0, which EEPROM writes need
    for k, reg in FIELDS:
        if before[k] != want[k]:
            bus.write(reg, joint, want[k])
    after = motor_values(bus, joint)
    pos_after = bus.read("Present_Position", joint)
    out.update(after=after, pos_after=pos_after, changed=True)
    if after != want:
        raise RuntimeError(f"{joint}: read back {after}, expected {want}. Power-cycle and try again")
    on_event(f"{joint}: written and read back OK. Present_Position {pos_before} -> {pos_after} "
             f"(same pose, new zero); json range {want['range_min']}..{want['range_max']}")
    if not want["range_min"] <= pos_after <= want["range_max"]:
        on_event(f"  !! {joint} now reads {pos_after}, OUTSIDE its range. Do not run anything; tell Claude.")
    return out
