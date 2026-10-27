"""Find a joint's hard stops by gently pushing into them.

One joint per run, both ends (or one). For each end:

    touch 1  creep toward the stop at ~13 deg/s until the joint stalls   -> back off 40 ticks
    touch 2  creep in again at ~4 deg/s: this one is the measurement      -> back off 40 ticks
    verdict  ok           both touches agree and the stop is near the calibrated range end
             suspect      it stalled somewhere else (table, own body, gravity winning)
             no_stop      it reached the search limit without stalling
             unrepeatable the two touches disagree

then the joint returns to where it started and every register goes back to how it was.

A stall: the goal keeps advancing, the joint has not moved for STALL_WINDOW_S, and the goal
is at least STALL_MIN_LEAD_TICKS ahead. The goal is never more than LEAD_MAX_TICKS ahead,
and Torque_Limit is the joint's low homing limit, so the push is capped twice (config.py).

Nothing here touches homing_offset: the candidate only changes range_min/range_max.
"""
from __future__ import annotations

import signal
import statistics
import traceback
from collections import deque
from dataclasses import asdict, dataclass, field
from typing import Callable

from . import config


class Refused(Exception):
    """Preflight said no. Nothing has been written to the arm."""


class Aborted(Exception):
    """Stopped mid-run. The cleanup still restores every register."""


@dataclass
class Touch:
    kind: str                 # "stop" | "soft_limit"
    position: int
    t: float
    peak_load: int            # largest |Present_Load| seen during the approach (0..1000)


@dataclass
class EndResult:
    end: str                  # "min" | "max"
    expected: int             # the current calibration's range end
    touches: list[Touch] = field(default_factory=list)
    stop: int | None = None
    verdict: str = ""
    note: str = ""


@dataclass
class JointResult:
    joint: str
    torque_limit: int
    widen: int
    tolerance: int
    start_pos: int | None = None
    ends: list[EndResult] = field(default_factory=list)
    aborted: str | None = None
    restore_errors: list[str] = field(default_factory=list)

    def to_dict(self) -> dict:
        return asdict(self)


@dataclass
class Sample:
    t: float
    pos: int
    load: int
    status: int


class Homer:
    def __init__(self, bus, calibration: dict[str, dict], joint: str, torque_limit: int | None = None,
                 widen: int = config.WIDEN_TICKS, tolerance: int = config.STOP_TOLERANCE_TICKS,
                 hold_others: bool = True, on_event: Callable[[str], None] = print,
                 on_sample: Callable[[dict], None] | None = None,
                 confirm_release: Callable[[], None] | None = None):
        if joint not in config.JOINTS:
            raise Refused(f"unknown joint {joint!r}; choose from {config.JOINTS}")
        self.bus = bus
        self.cal = calibration
        self.joint = joint
        self.torque_limit = config.HOMING_TORQUE_LIMIT[joint] if torque_limit is None else int(torque_limit)
        self.widen = int(widen)
        self.tolerance = int(tolerance)
        self.hold_others = hold_others
        self.on_event = on_event
        self.on_sample = on_sample
        self.confirm_release = confirm_release
        self.others = [j for j in config.JOINTS if j != joint]
        self.rows: list[dict] = []
        self.phase = "preflight"
        self.goal: int | None = None
        self.limits = (calibration[joint]["range_min"], calibration[joint]["range_max"])
        self._last_pos: int | None = None
        self._cycle = 0
        self._next_tick = 0.0

    # ================================================================== public
    def run(self, ends: tuple[str, ...] = ("min", "max")) -> JointResult:
        res = JointResult(self.joint, self.torque_limit, self.widen, self.tolerance)
        self.preflight()                                   # raises Refused, writes nothing
        snapshot = self._snapshot()
        try:
            self._setup()
            res.start_pos = self._read_pos()
            for end in ends:
                res.ends.append(self._home_end(end))
            self._event(f"returning to the start pose ({res.start_pos})")
            self._move_to(res.start_pos, config.STEP_MOVE_TICKS, "return")
        except BaseException as e:  # noqa: BLE001 - Ctrl-C included: always clean up, then report
            if isinstance(e, Aborted):
                res.aborted = str(e)
            elif isinstance(e, KeyboardInterrupt):
                res.aborted = "interrupted (Ctrl-C)"
            else:
                res.aborted = f"error: {e!r}\n{traceback.format_exc()}"
            self._event(f"ABORT: {res.aborted.splitlines()[0]}")
        finally:
            res.restore_errors = self._restore(snapshot, aborted=res.aborted is not None)
        return res

    def preflight(self) -> None:
        """Read-only checks. Any failure raises Refused before a single write."""
        j, cal = self.joint, self.cal
        if not 0 < self.torque_limit <= config.TORQUE_LIMIT_CEILING:
            raise Refused(f"torque limit {self.torque_limit} outside 1..{config.TORQUE_LIMIT_CEILING}")
        if not 0 <= self.widen <= config.WIDEN_CEILING:
            raise Refused(f"widen {self.widen} outside 0..{config.WIDEN_CEILING}")
        if not 0 < self.tolerance <= config.STOP_TOLERANCE_CEILING:
            raise Refused(f"tolerance {self.tolerance} outside 1..{config.STOP_TOLERANCE_CEILING}")

        problems = []
        for m in config.JOINTS:
            volt = self.bus.read("Present_Voltage", m) / 10
            temp = self.bus.read("Present_Temperature", m)
            status = self.bus.read("Status", m)
            if not config.VOLTAGE_MIN_V <= volt <= config.VOLTAGE_MAX_V:
                problems.append(f"{m}: {volt:.1f} V (want {config.VOLTAGE_MIN_V}-{config.VOLTAGE_MAX_V} V; "
                                "~4.6 V means this is the LEADER or the PSU is off)")
            if temp >= config.TEMP_REFUSE_C:
                problems.append(f"{m}: {temp} C, let it cool below {config.TEMP_REFUSE_C} C")
            if status:
                problems.append(f"{m}: Status {config.describe_status(status)}")
            if self.bus.read("Torque_Enable", m):
                problems.append(f"{m}: torque is ON. Start with the arm resting and torque off on every joint")
        if self.bus.read("Operating_Mode", j) != 0:
            problems.append(f"{j}: not in position mode")
        eeprom = {k: self.bus.read(r, j) for k, r in
                  (("homing_offset", "Homing_Offset"), ("range_min", "Min_Position_Limit"),
                   ("range_max", "Max_Position_Limit"))}
        mismatch = {k: (v, cal[j][k]) for k, v in eeprom.items() if v != cal[j][k]}
        if mismatch:
            problems.append(f"{j}: motor EEPROM {mismatch} (motor, json) differs from the calibration JSON. "
                            "Connect once with lerobot and press ENTER to write the JSON to the motors")
        pos = self.bus.read("Present_Position", j)
        lo, hi = cal[j]["range_min"], cal[j]["range_max"]
        if not lo - config.START_MARGIN_TICKS <= pos <= hi + config.START_MARGIN_TICKS:
            problems.append(f"{j}: at {pos}, outside its calibrated range {lo}..{hi}")
        if problems:
            raise Refused("preflight failed, nothing was written:\n  " + "\n  ".join(problems))
        self._event(f"preflight ok: {j} at {pos}, range {lo}..{hi}, torque limit {self.torque_limit}/1000, "
                    f"widen {self.widen}, tolerance {self.tolerance}")

    # ================================================================== setup / restore
    def _snapshot(self) -> dict:
        j = self.joint
        snap = {"homed": {r: self.bus.read(r, j) for r in
                          ("Torque_Enable", "Torque_Limit", "Acceleration", "Goal_Velocity",
                           "Min_Position_Limit", "Max_Position_Limit")},
                "others": {m: self.bus.read("Torque_Enable", m) for m in self.others}}
        self.snapshot = snap
        return snap

    def _setup(self) -> None:
        j, b = self.joint, self.bus
        self.phase = "setup"
        if self.hold_others:
            for m in self.others:      # hold the rest of the arm exactly where it lies
                b.write("Goal_Position", m, b.read("Present_Position", m))
                b.torque_on(m)
        lo, hi = self.cal[j]["range_min"], self.cal[j]["range_max"]
        if self.widen:
            b.torque_off(j)            # Lock=0: EEPROM writable. The homed joint is resting.
            lo, hi = max(0, lo - self.widen), min(4095, hi + self.widen)
            b.write("Min_Position_Limit", j, lo)
            b.write("Max_Position_Limit", j, hi)
            got = (b.read("Min_Position_Limit", j), b.read("Max_Position_Limit", j))
            if got != (lo, hi):
                raise Aborted(f"widened limits read back as {got}, expected {(lo, hi)}")
            self._event(f"firmware limits widened to {lo}..{hi} for this run")
        self.limits = (lo, hi)
        b.write("Torque_Limit", j, self.torque_limit)
        b.write("Acceleration", j, config.ACCELERATION_CAP)
        b.write("Goal_Velocity", j, config.GOAL_VELOCITY_CAP)
        self.goal = b.read("Present_Position", j)
        b.write("Goal_Position", j, self.goal)    # no jump when torque comes on
        b.torque_on(j)
        self._next_tick = b.now()
        self._hold_check()

    def _hold_check(self) -> None:
        """At the homing torque limit, can the joint even hold itself where it lies?"""
        self.phase = "hold-check"
        s0 = self._sample()
        s, recent = s0, deque([(s0.t, s0.pos)])
        while True:
            self._wait_cycle()
            s = self._sample()
            recent.append((s.t, s.pos))
            while s.t - recent[0][0] > 0.2:
                recent.popleft()
            sagging = abs(s.pos - s0.pos) > config.HOLD_SAG_MAX_TICKS
            done = s.t - s0.t >= config.HOLD_CHECK_S
            if sagging or (done and max(p for _, p in recent) - min(p for _, p in recent) > 3):
                raise Aborted(f"{self.joint} cannot hold itself at torque limit {self.torque_limit}/1000 "
                              f"(sagged {s0.pos} -> {s.pos}): gravity wins. Raise --torque-limit or rest the "
                              "arm in a pose where this joint carries less weight")
            if done:
                return

    def _restore(self, snap: dict, aborted: bool) -> list[str]:
        """Put every register back, step by step; one failure never skips the rest."""
        errors: list[str] = []
        j, b, h = self.joint, self.bus, snap["homed"]
        old = signal.signal(signal.SIGINT, signal.SIG_IGN) if _main_thread() else None

        def step(what, fn):
            try:
                fn()
            except Exception as e:  # noqa: BLE001
                errors.append(f"{what}: {e!r}")

        try:
            self.phase = "restore"
            step("hold homed joint", lambda: b.write("Goal_Position", j, b.read("Present_Position", j)))
            for r in ("Torque_Limit", "Acceleration", "Goal_Velocity"):
                step(f"restore {r}", lambda r=r: b.write(r, j, h[r]))
            if aborted and self.confirm_release is not None:
                step("confirm release", self.confirm_release)
            want = (h["Min_Position_Limit"], h["Max_Position_Limit"])

            def limits_now():
                return b.read("Min_Position_Limit", j), b.read("Max_Position_Limit", j)

            try:
                widened = limits_now() != want
            except Exception as e:  # noqa: BLE001 - can't tell, so restore them anyway
                errors.append(f"read limits: {e!r}")
                widened = True
            if widened:
                step("torque off for EEPROM", lambda: b.torque_off(j))
                step("restore Min_Position_Limit", lambda: b.write("Min_Position_Limit", j, want[0]))
                step("restore Max_Position_Limit", lambda: b.write("Max_Position_Limit", j, want[1]))
                try:
                    got = limits_now()
                    if got != want:
                        errors.append(f"limits read back as {got}, expected {want}")
                except Exception as e:  # noqa: BLE001
                    errors.append(f"verify limits: {e!r}")
            step("restore torque", lambda: b.torque_on(j) if h["Torque_Enable"] else b.torque_off(j))
            for m, was_on in snap["others"].items():
                if self.hold_others:
                    step(f"restore torque {m}", lambda m=m, on=was_on: b.torque_on(m) if on else b.torque_off(m))
        finally:
            if old is not None:
                signal.signal(signal.SIGINT, old)
        self._event("registers restored" if not errors else "RESTORE INCOMPLETE:\n  " + "\n  ".join(errors))
        return errors

    # ================================================================== one end
    def _home_end(self, end: str) -> EndResult:
        d = -1 if end == "min" else 1
        expected = self.cal[self.joint]["range_min" if end == "min" else "range_max"]
        r = EndResult(end, expected)
        # never search further than a stop we would accept could be
        fw = self.limits[0] if d < 0 else self.limits[1]
        search = max(fw, expected - self.tolerance) if d < 0 else min(fw, expected + self.tolerance)

        self._event(f"{end}: touch 1 (fast) toward {expected}, search limit {search}")
        t1 = self._approach(d, config.STEP_FAST_TICKS, search, f"{end}-touch1")
        r.touches.append(t1)
        self._backoff(t1.position, d, end)
        if t1.kind == "soft_limit":
            r.verdict, r.note = "no_stop", f"reached the search limit {search} without stalling"
            return r
        if abs(t1.position - expected) > self.tolerance:
            r.verdict, r.stop = "suspect", t1.position
            r.note = f"stalled at {t1.position}, {t1.position - expected:+d} from the calibrated end"
            return r

        self._event(f"{end}: touch 2 (slow)")
        t2 = self._approach(d, config.STEP_SLOW_TICKS, search, f"{end}-touch2")
        r.touches.append(t2)
        self._backoff(t2.position, d, end)
        if t2.kind != "stop" or abs(t2.position - t1.position) > config.REPEAT_TOLERANCE_TICKS:
            r.verdict = "unrepeatable"
            r.note = f"touch 1 {t1.kind}@{t1.position}, touch 2 {t2.kind}@{t2.position}"
            return r
        r.stop = t2.position
        if abs(r.stop - expected) > self.tolerance:
            r.verdict, r.note = "suspect", f"{r.stop - expected:+d} from the calibrated end"
        else:
            r.verdict, r.note = "ok", f"stop {r.stop} ({r.stop - expected:+d} vs calibration), touches agree " \
                                      f"to {abs(t2.position - t1.position)} ticks"
        self._event(f"{end}: {r.verdict}: {r.note}")
        return r

    def _approach(self, d: int, step: int, search: int, phase: str) -> Touch:
        self.phase = phase
        s = self._sample()
        # keep the current goal: resetting it to pos would drop a gravity-loaded joint's preload
        t0, hist, peak, best = s.t, deque([(s.t, s.pos)]), 0, s.pos
        timeout = abs(search - s.pos) / (step * config.CYCLE_HZ) + config.APPROACH_TIMEOUT_EXTRA_S
        while True:
            self.goal = self._next_goal(s.pos, d, step, search)
            self.bus.write("Goal_Position", self.joint, self.goal)
            self._wait_cycle()
            s = self._sample()
            peak = max(peak, abs(s.load))
            best = self._check_sag(s.pos, d, best, phase)
            hist.append((s.t, s.pos))
            while len(hist) > 1 and s.t - hist[1][0] >= config.STALL_WINDOW_S:
                hist.popleft()
            if s.t - hist[0][0] >= config.STALL_WINDOW_S and d * (s.pos - hist[0][1]) < config.STALL_PROGRESS_TICKS:
                lead = d * (self.goal - s.pos)
                if lead >= config.STALL_MIN_LEAD_TICKS:
                    return Touch("stop", int(statistics.median(p for _, p in list(hist)[-5:])), s.t, peak)
                if self.goal == search and abs(search - s.pos) <= config.SOFT_LIMIT_TOL_TICKS:
                    return Touch("soft_limit", s.pos, s.t, peak)
            if s.t - t0 > timeout:
                raise Aborted(f"{phase}: no stall and no limit after {timeout:.1f} s (at {s.pos}, goal {self.goal})")

    def _backoff(self, pos: int, d: int, end: str) -> None:
        self._move_to(pos - d * config.BACKOFF_TICKS, config.STEP_MOVE_TICKS, f"{end}-backoff")

    def _move_to(self, target: int, step: int, phase: str) -> None:
        """Step the goal toward target with the same lead cap; a stall on the way aborts."""
        self.phase = phase
        lo, hi = self.limits
        target = min(max(target, lo), hi)
        s = self._sample()
        d = 1 if target >= s.pos else -1
        t0, hist, best = s.t, deque([(s.t, s.pos)]), s.pos
        timeout = abs(target - s.pos) / (step * config.CYCLE_HZ) + config.APPROACH_TIMEOUT_EXTRA_S
        while True:
            self.goal = self._next_goal(s.pos, d, step, target)
            self.bus.write("Goal_Position", self.joint, self.goal)
            self._wait_cycle()
            s = self._sample()
            best = self._check_sag(s.pos, d, best, phase)
            hist.append((s.t, s.pos))
            while len(hist) > 1 and s.t - hist[1][0] >= config.STALL_WINDOW_S:
                hist.popleft()
            still = s.t - hist[0][0] >= config.STALL_WINDOW_S and abs(s.pos - hist[0][1]) < config.STALL_PROGRESS_TICKS
            if self.goal == target:
                # arrived once it is still; under gravity a P servo rests a few ticks off its goal
                if abs(target - s.pos) <= 3 or (still and abs(target - s.pos) <= config.HOLD_SAG_MAX_TICKS):
                    return
            elif still and d * (self.goal - s.pos) >= config.STALL_MIN_LEAD_TICKS:
                raise Aborted(f"{phase}: blocked at {s.pos} while moving to {target}")
            if s.t - t0 > timeout:
                raise Aborted(f"{phase}: did not reach {target} in {timeout:.1f} s (at {s.pos})")

    def _check_sag(self, pos: int, d: int, best: int, phase: str) -> int:
        """Furthest progress so far; abort when the joint slides back from it (gravity)."""
        best = max(best, pos) if d > 0 else min(best, pos)
        if d * (pos - best) < -config.SAG_TICKS:
            raise Aborted(f"{phase}: {self.joint} moving against the command ({best} -> {pos}): gravity is "
                          f"stronger than torque limit {self.torque_limit}/1000")
        return best

    def _next_goal(self, pos: int, d: int, step: int, limit: int) -> int:
        if d > 0:
            return min(self.goal + step, pos + config.LEAD_MAX_TICKS, limit)
        return max(self.goal - step, pos - config.LEAD_MAX_TICKS, limit)

    # ================================================================== sensing
    def _wait_cycle(self) -> None:
        self._next_tick += 1 / config.CYCLE_HZ
        self.bus.sleep(self._next_tick - self.bus.now())
        if self.bus.now() - self._next_tick > 0.1:     # fell behind (slow bus): don't burst to catch up
            self._next_tick = self.bus.now()

    def _read_pos(self) -> int:
        return self.bus.read("Present_Position", self.joint)

    def _sample(self) -> Sample:
        b, j = self.bus, self.joint
        s = Sample(b.now(), b.read("Present_Position", j), b.read("Present_Load", j), b.read("Status", j))
        row = {"t": round(s.t, 3), "phase": self.phase, "goal": self.goal, "pos": s.pos, "load": s.load,
               "status": s.status, "temp": "", "volt": ""}
        if s.status:
            raise Aborted(f"{j} Status {config.describe_status(s.status)} at {s.pos}")
        if self._last_pos is not None and abs(s.pos - self._last_pos) > config.JUMP_ABORT_TICKS:
            raise Aborted(f"{j} jumped {self._last_pos} -> {s.pos} in one cycle (encoder wrap or glitch)")
        self._last_pos = s.pos
        lo, hi = self.limits
        if not lo - 20 <= s.pos <= hi + 20:
            raise Aborted(f"{j} at {s.pos}, outside the limits {lo}..{hi}")
        if self._cycle % config.TEMP_VOLT_EVERY == 0:
            temp, volt = b.read("Present_Temperature", j), b.read("Present_Voltage", j) / 10
            row.update(temp=temp, volt=volt)
            if temp >= config.TEMP_ABORT_C:
                raise Aborted(f"{j} at {temp} C")
            if not config.VOLTAGE_MIN_V <= volt <= config.VOLTAGE_MAX_V:
                raise Aborted(f"supply at {volt:.1f} V")
            if self.hold_others:
                for m in self.others:
                    st = b.read("Status", m)
                    if st:
                        raise Aborted(f"{m} (holding) Status {config.describe_status(st)}")
        self._cycle += 1
        self.rows.append(row)
        if self.on_sample is not None:
            self.on_sample(row)
        return s

    def _event(self, msg: str) -> None:
        self.on_event(f"[{self.joint}] {msg}")


def candidate_calibration(calibration: dict[str, dict], results: list[JointResult],
                          inset: int = config.STOP_INSET_TICKS) -> dict[str, dict]:
    """A copy of the calibration with range_min/max moved to the found stops (minus the
    inset) for every end whose verdict is ok. homing_offset is never changed."""
    cand = {j: dict(v) for j, v in calibration.items()}
    for res in results:
        if res.aborted:
            continue
        for e in res.ends:
            if e.verdict != "ok":
                continue
            key = "range_min" if e.end == "min" else "range_max"
            cand[res.joint][key] = e.stop + inset if e.end == "min" else e.stop - inset
    return cand


def _main_thread() -> bool:
    import threading
    return threading.current_thread() is threading.main_thread()
