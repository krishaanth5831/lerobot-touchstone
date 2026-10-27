"""A simulated follower bus with real hard stops, for the tests and for `--sim`.

Per joint, 1 DOF, raw ticks, physics stepped at 1 kHz on a virtual clock (sleep() advances
it, so the tests run in milliseconds and `--sim --realtime` runs at wall-clock speed):

    goal     = Goal_Position clamped to [Min_Position_Limit, Max_Position_Limit] (firmware)
    drive    = K * (goal - pos), 0 inside the 1-tick dead zone, clipped at Torque_Limit
               K = 9 load/tick * 2.94/1000 N.m * P/16   (measured on the real arm)
    external = gravity (constant N.m per joint) + hard stops (stiff springs at the true stops)
               + an optional obstacle (a stop somewhere it should not be: the table)
    friction = Coulomb 0.05 N.m: the joint does not move while |drive + external| <= it
    motion   = overdamped, speed capped by Goal_Velocity
    reading  = round(pos + 0.08 tick rms jitter)

It also enforces the rules the real servo does, so a bug that would fail on the arm fails
here: EEPROM registers only accept writes with torque off and Lock=0, read-only registers
reject writes, and a joint pushed at >95 % stall torque for 1 s trips the overload bit.
The friction, jitter and K values come from torque-tactile-sensing's arm recordings.
"""
from __future__ import annotations

import random

from . import config

NM_PER_LOAD = 2.94 / 1000
NM_PER_TICK_P16 = 9 * NM_PER_LOAD
FRICTION_NM = 0.05
ENCODER_JITTER = 0.08
DEAD_ZONE = 1
TIME_CONSTANT = 0.03
STOP_STIFFNESS_NM = 0.3       # N.m per tick of penetration (stable at the 1 ms step)
PHYSICS_DT = 0.001

EEPROM = {"Min_Position_Limit", "Max_Position_Limit", "Homing_Offset", "Max_Torque_Limit", "P_Coefficient",
          "Operating_Mode", "Protection_Current", "Overload_Torque"}
READ_ONLY = {"Present_Position", "Present_Load", "Present_Voltage", "Present_Temperature", "Status",
             "Present_Current", "Moving"}


class SimJoint:
    def __init__(self, name: str, cal: dict, stops: tuple[float, float], pos: float, gravity_nm=0.0):
        """gravity_nm: N.m in the +tick direction, a constant or a function of position."""
        self.name = name
        self.stops = stops
        self.pos = float(pos)
        self.gravity_nm = gravity_nm
        self.obstacle: tuple[str, float] | None = None   # ("max", tick): blocks motion past tick
        self.temp = 30.0
        self.overload_s = 0.0
        self.drive_nm = 0.0
        self.reg = {
            "Torque_Enable": 0, "Lock": 0, "Torque_Limit": 1000, "Max_Torque_Limit": 1000,
            "Acceleration": 254, "Goal_Velocity": 0, "Goal_Position": int(round(pos)),
            "Min_Position_Limit": cal["range_min"], "Max_Position_Limit": cal["range_max"],
            "Homing_Offset": cal["homing_offset"], "Operating_Mode": 0, "P_Coefficient": 16,
            "Protection_Current": 250, "Overload_Torque": 25, "Protective_Torque": 20, "Protection_Time": 200,
            "Status": 0, "Present_Voltage": 120,
        }

    @property
    def K(self) -> float:
        return NM_PER_TICK_P16 * self.reg["P_Coefficient"] / 16

    def step(self, dt: float) -> None:
        r = self.reg
        if not r["Torque_Enable"]:
            self.drive_nm = 0.0
            return
        goal = min(max(r["Goal_Position"], r["Min_Position_Limit"]), r["Max_Position_Limit"])
        err = goal - self.pos
        limit = min(r["Torque_Limit"], r["Max_Torque_Limit"]) * NM_PER_LOAD
        drive = 0.0 if abs(err) <= DEAD_ZONE else max(-limit, min(limit, self.K * err))
        self.drive_nm = drive

        lo, hi = self.stops
        if self.obstacle is not None:
            side, tick = self.obstacle
            lo, hi = (max(lo, tick), hi) if side == "min" else (lo, min(hi, tick))
        ext = self.gravity_nm(self.pos) if callable(self.gravity_nm) else self.gravity_nm
        if self.pos < lo:
            ext += STOP_STIFFNESS_NM * (lo - self.pos)
        if self.pos > hi:
            ext -= STOP_STIFFNESS_NM * (self.pos - hi)
        net = drive + ext
        excess = (1 if net > 0 else -1) * max(abs(net) - FRICTION_NM, 0.0)
        v = excess / (self.K * TIME_CONSTANT)
        if r["Goal_Velocity"] > 0:
            v = max(-r["Goal_Velocity"], min(r["Goal_Velocity"], v))
        self.pos += v * dt

        # heat: grows with the square of the torque, cools toward 30 C
        self.temp += ((drive / 2.94) ** 2 * 2.0 - (self.temp - 30.0) * 0.01) * dt
        # firmware overload protection: sustained near-stall torque -> status bit 5, torque off
        self.overload_s = self.overload_s + dt if abs(drive) > 0.95 * 2.94 else 0.0
        if self.overload_s > 1.0:
            r["Status"] |= 1 << 5
            r["Torque_Enable"] = 0


class SimBus:
    """Same interface as FeetechBus. `stops` are the TRUE hard stops per joint (ticks)."""

    def __init__(self, calibration: dict[str, dict], stops: dict[str, tuple[float, float]] | None = None,
                 pose: dict[str, float] | None = None, gravity_nm: dict[str, float] | None = None,
                 seed: int = 0, realtime: bool = False):
        self.rng = random.Random(seed)
        self.calibration = calibration
        self.joints: dict[str, SimJoint] = {}
        for j in config.JOINTS:
            c = calibration[j]
            if stops and j in stops:
                st = stops[j]
            else:  # hand calibration records the stops a few ticks short of where they really are
                st = (c["range_min"] - self.rng.uniform(2, 15), c["range_max"] + self.rng.uniform(2, 15))
            p = (pose or {}).get(j, (c["range_min"] + c["range_max"]) / 2)
            self.joints[j] = SimJoint(j, c, st, p, (gravity_nm or {}).get(j, 0.0))
        self.t = 0.0
        self.realtime = realtime
        self.writes: list[tuple[float, str, str, int]] = []
        self.faults: list[tuple[float, str, str, int]] = []      # (t, motor, kind, value)
        self.interrupt_at: float | None = None

    # ------------------------------------------------------------------ bus interface
    def read(self, reg: str, motor: str) -> int:
        J = self.joints[motor]
        if reg == "Present_Position":
            return int(round(J.pos + self.rng.gauss(0, ENCODER_JITTER)))
        if reg == "Present_Load":
            return int(round(J.drive_nm / NM_PER_LOAD))
        if reg == "Present_Temperature":
            return int(J.temp)
        if reg == "Present_Current":
            return int(abs(J.drive_nm) / NM_PER_LOAD // 150)
        if reg == "Moving":
            return 0
        return int(J.reg[reg])

    def write(self, reg: str, motor: str, value: int) -> None:
        J = self.joints[motor]
        if reg in READ_ONLY:
            raise RuntimeError(f"sim: {reg} is read-only")
        if reg in EEPROM and (J.reg["Torque_Enable"] or J.reg["Lock"]):
            raise RuntimeError(f"sim: EEPROM write {reg} on {motor} with torque on or Lock=1")
        self.writes.append((round(self.t, 3), motor, reg, int(value)))
        J.reg[reg] = int(value)

    def torque_on(self, motor: str) -> None:
        self.writes.append((round(self.t, 3), motor, "Torque_Enable", 1))
        self.joints[motor].reg.update(Torque_Enable=1, Lock=1)

    def torque_off(self, motor: str) -> None:
        self.writes.append((round(self.t, 3), motor, "Torque_Enable", 0))
        self.joints[motor].reg.update(Torque_Enable=0, Lock=0)

    def now(self) -> float:
        return self.t

    def sleep(self, dt: float) -> None:
        end = self.t + max(dt, 0.0)
        while self.t < end:
            for J in self.joints.values():
                J.step(PHYSICS_DT)
            self.t += PHYSICS_DT
            for f in [f for f in self.faults if f[0] <= self.t]:
                self.faults.remove(f)
                _, motor, kind, value = f
                if kind == "status":
                    self.joints[motor].reg["Status"] |= value
                elif kind == "temp":
                    self.joints[motor].temp = value
                elif kind == "voltage":
                    self.joints[motor].reg["Present_Voltage"] = value
            if self.interrupt_at is not None and self.t >= self.interrupt_at:
                self.interrupt_at = None
                raise KeyboardInterrupt
        if self.realtime:
            import time
            time.sleep(dt)

    def close(self) -> None:
        pass

    # ------------------------------------------------------------------ test hooks
    def inject(self, t: float, motor: str, kind: str, value: int) -> None:
        """At sim time t: kind 'status' ORs bits into Status, 'temp' sets C, 'voltage' sets 0.1 V."""
        self.faults.append((t, motor, kind, value))
