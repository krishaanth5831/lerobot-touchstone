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

import numpy as np
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
    tests: list = field(default_factory=list)   # test moves: {"joint", "to", "stall_after", "outcome"}
    untested: list = field(default_factory=list)  # link pairs no test move could shift


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
    staged: dict = field(default_factory=dict)          # other joint -> staging tick it was moved to
    span_note: str = ""                                 # measured travel vs the design travel

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
                 confirm_release: Callable[[], None] | None = None,
                 stage: dict[str, int] | None = None, kin=None,
                 gravity_fn: Callable[[dict], dict] | None = None, verify_all: bool = False):
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
        self.lead_max = config.LEAD_MAX_TICKS     # scaled by 16/P in preflight
        self._last_pos: int | None = None
        self._cycle = 0
        self._next_tick = 0.0
        self._dir = 0                             # direction of the current motion, 0 = holding
        # staging: other joints moved to a hand-taught pose so this joint can sweep freely
        self.stage = {m: int(t) for m, t in (stage or {}).items() if m != joint}
        self._staged: dict[str, int] = {}         # joint -> its rest tick, while it is away from rest
        self.limp: list[str] = []                 # other joints left limp (resting outside their limits)
        # blocked-or-real-stop tests (touchstone.kinematics.Kinematics); None = old behaviour
        self.kin = kin
        self.gravity_fn = gravity_fn or (kin.gravity_load if kin is not None else None)
        self._history: list[tuple[str, int, int, int]] = []  # (other joint, its tick before, homed tick, cap)
        self.verify_all = verify_all          # test every stall, even one where the stop should be
        self._force = False                   # test this end even if near (a short span asked for it)
        self._search = None                   # the current end's search limit
        self._d = 0
        self._too_heavy: list[str] = []       # test moves dropped: they'd leave the homed joint too weak
        self._hot = 0                             # temperature readings in a row over the abort line
        self._bad_volt = 0

    # ================================================================== public
    def run(self, ends: tuple[str, ...] = ("min", "max")) -> JointResult:
        res = JointResult(self.joint, self.torque_limit, self.widen, self.tolerance)
        self.preflight()                                   # raises Refused, writes nothing
        snapshot = self._snapshot()
        try:
            self._setup()
            res.start_pos = self._read_pos()
            self._stage_in()
            res.staged = dict(self.stage)
            if self.stage:
                self._set_homed_limit(self.torque_limit)   # now the homing limit
            try:
                self._hold_check()                         # in the pose it will sweep from
            except Aborted:
                # Too weak to hold, before any sweep: the homed joint is still where it started.
                # Give it its full torque back and unstage normally, so home-all can retry one
                # rung up its ladder from rest. Any other abort while staged holds everything.
                if self.stage:
                    self._hold_homed_firm(snapshot)
                    self._stage_out()
                raise
            for end in ends:
                res.ends.append(self._home_end(end))
            res.span_note = self._span_note(res)
            if "short" in res.span_note and self.kin is not None:
                for i, e in enumerate(res.ends):
                    if not e.tests:
                        self._event(f"{res.span_note}: testing the {e.end} end, which was not tested")
                        self._force = True
                        res.ends[i] = self._home_end(e.end)
                        self._force = False
                res.span_note = self._span_note(res)
            self._event(f"returning to the start pose ({res.start_pos})")
            self._return_to(res.start_pos)
            if self.stage:
                self._hold_homed_firm(snapshot)
            self._stage_out()                              # back to rest BEFORE anything is let go
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

    # ================================================================== staging
    def _return_to(self, start: int) -> None:
        """Back to the start along the path it just swept, so known clear. If gravity helped it out
        and the homing limit cannot bring it back (a barely strong enough joint), retry once at up
        to TORQUE_LIMIT_CEILING: the goal lead cap still bounds the push."""
        try:
            self._move_to(start, config.STEP_MOVE_TICKS, "return")
        except Aborted as e:
            if "blocked" not in str(e) and "did not reach" not in str(e):
                raise
            lim = min(self.snapshot["homed"]["Torque_Limit"], config.TORQUE_LIMIT_CEILING)
            if lim <= self.torque_limit:
                raise
            self._event(f"return: too weak at {self.torque_limit}/1000 ({e}); retrying at {lim}/1000")
            self._set_homed_limit(lim)
            self._move_to(start, config.STEP_MOVE_TICKS, "return")

    def _set_homed_limit(self, lim: int) -> None:
        self.bus.write("Torque_Limit", self.joint, lim)
        got = self.bus.read("Torque_Limit", self.joint)
        if got != lim:
            raise Aborted(f"{self.joint}: Torque_Limit reads {got}, expected {lim}")

    def _hold_homed_firm(self, snapshot: dict) -> None:
        """Hold the homed joint where it is at its old (full) limit, for unstaging around it."""
        self.goal = self._read_pos()
        self.bus.write("Goal_Position", self.joint, self.goal)
        self._set_homed_limit(snapshot["homed"]["Torque_Limit"])

    def _stage_in(self) -> None:
        for m in config.STAGE_ORDER:
            if m in self.stage and m in self.limp:
                raise Aborted(f"staging {m}: it rests outside its firmware limits and is limp; home it first")
            if m in self.stage:
                rest = self.bus.read("Present_Position", m)
                self._staged[m] = rest                      # marked BEFORE it moves
                self._event(f"staging {m}: {rest} -> {self.stage[m]}")
                self._move_other(m, self.stage[m])
        if self.stage:
            self._next_tick = self.bus.now()

    def _stage_out(self) -> None:
        for m in reversed(list(self._staged)):
            self._event(f"unstaging {m}: back to {self._staged[m]}")
            self._move_other(m, self._staged[m])
            del self._staged[m]

    def _move_other(self, m: str, target: int, lim: int | None = None, what: str = "staging") -> None:
        """Move one held joint to target at a capped torque, watching it and the homed joint."""
        b = self.bus
        hold_limit = b.read("Torque_Limit", m)
        lim = config.STAGE_TORQUE_LIMIT[m] if lim is None else int(lim)
        b.write("Torque_Limit", m, lim)
        if b.read("Torque_Limit", m) != lim:
            b.write("Torque_Limit", m, hold_limit)
            raise Aborted(f"{what} {m}: Torque_Limit did not read back as {lim}")
        stage_lead = config.lead_for(lim, max(b.read("P_Coefficient", m), config.P_REFERENCE))
        self.phase, self._dir = f"{'stage' if what == 'staging' else 'test'} {m}", 0   # the homed joint holds still
        try:
            pos = b.read("Present_Position", m)
            goal = pos
            d = 1 if target >= pos else -1
            hist = deque([(b.now(), pos)])
            t0 = b.now()
            timeout = abs(target - pos) / (config.STAGE_STEP_TICKS * config.CYCLE_HZ) + config.APPROACH_TIMEOUT_EXTRA_S
            self._next_tick = b.now()
            while True:
                lead_cap = pos + d * stage_lead
                goal = min(goal + config.STAGE_STEP_TICKS, lead_cap, target) if d > 0 else \
                    max(goal - config.STAGE_STEP_TICKS, lead_cap, target)
                b.write("Goal_Position", m, goal)
                self._wait_cycle()
                self._sample()                              # the homed joint must stay put meanwhile
                pos, st = b.read("Present_Position", m), b.read("Status", m)
                t = b.now()
                if st:
                    raise Aborted(f"{what} {m}: Status {config.describe_status(st)} at {pos}")
                if abs(pos - goal) > config.OVERRUN_TICKS + stage_lead:
                    raise Aborted(f"{what} {m}: at {pos}, {pos - goal:+d} from its goal {goal}: not under control")
                hist.append((t, pos))
                while len(hist) > 1 and t - hist[1][0] >= config.STALL_WINDOW_S:
                    hist.popleft()
                still = t - hist[0][0] >= config.STALL_WINDOW_S and abs(pos - hist[0][1]) < config.STALL_PROGRESS_TICKS
                if goal == target and (abs(target - pos) <= 3 or (still and abs(target - pos) <= config.HOLD_SAG_MAX_TICKS)):
                    return
                if goal != target and still and d * (goal - pos) >= config.STALL_MIN_LEAD_TICKS:
                    raise Aborted(f"{what} {m}: blocked at {pos} on the way to {target} (pushing at most "
                                  f"{lim}/1000). Something is in the way")
                if t - t0 > timeout:
                    raise Aborted(f"{what} {m}: did not reach {target} in {timeout:.1f} s (at {pos})")
        finally:
            # hold exactly where it is, at the limit it held with before (no jump toward a goal)
            try:
                b.write("Goal_Position", m, b.read("Present_Position", m))
            finally:
                b.write("Torque_Limit", m, hold_limit)

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
        p_gain = self.bus.read("P_Coefficient", j)
        if not 0 < p_gain <= config.P_MAX:
            problems.append(f"{j}: P_Coefficient {p_gain}, expected 1..{config.P_MAX} (lerobot sets 16)")
        else:
            # sized so the push can reach the torque limit and no further (a stiffer P = fewer ticks)
            self.lead_max = config.lead_for(self.torque_limit, max(p_gain, config.P_REFERENCE))
        eeprom = {k: self.bus.read(r, j) for k, r in
                  (("homing_offset", "Homing_Offset"), ("range_min", "Min_Position_Limit"),
                   ("range_max", "Max_Position_Limit"))}
        mismatch = {k: (v, cal[j][k]) for k, v in eeprom.items() if v != cal[j][k]}
        if mismatch:
            problems.append(f"{j}: motor EEPROM {mismatch} (motor, json) differs from the calibration JSON. "
                            "Connect once with lerobot and press ENTER to write the JSON to the motors")
        pos = self.bus.read("Present_Position", j)
        lo, hi = cal[j]["range_min"], cal[j]["range_max"]
        # The widened firmware limits must cover where it rests: shoulder_lift rests on its real
        # stop at 748, 37 below the hand-recorded 785 (real arm 2026-11-04).
        margin = max(config.START_MARGIN_TICKS, self.widen - 10)
        if not lo - margin <= pos <= hi + margin:
            problems.append(f"{j}: at {pos}, outside its calibrated range {lo}..{hi}")
        if self.stage and not self.hold_others:
            problems.append("staging needs the other joints held (hold_others)")
        for m, t in self.stage.items():
            mlo, mhi = cal[m]["range_min"], cal[m]["range_max"]
            if not mlo <= t <= mhi:
                problems.append(f"staging pose: {m} {t} is outside its calibrated range {mlo}..{mhi}")
        if problems:
            raise Refused("preflight failed, nothing was written:\n  " + "\n  ".join(problems))
        self._event(f"preflight ok: {j} at {pos}, range {lo}..{hi}, torque limit {self.torque_limit}/1000, "
                    f"widen {self.widen}, tolerance {self.tolerance}, lead cap {self.lead_max}")

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
                p = b.read("Present_Position", m)
                lo_m, hi_m = b.read("Min_Position_Limit", m), b.read("Max_Position_Limit", m)
                if not lo_m <= p <= hi_m:
                    # Resting past its firmware limit (on its real stop, by gravity): the servo would
                    # clamp the goal and jump to the limit at full torque. Leave it limp instead.
                    self.limp.append(m)
                    self._event(f"{m} rests at {p}, outside its firmware limits {lo_m}..{hi_m}: left limp")
                    continue
                b.write("Goal_Position", m, p)
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
        # While other joints stage around it, the homed joint holds at its old limit (full), not
        # the homing limit: the arm's weight on it changes as they move. Lowered after staging.
        first = self.snapshot["homed"]["Torque_Limit"] if self.stage else self.torque_limit
        b.write("Torque_Limit", j, first)
        b.write("Acceleration", j, config.ACCELERATION_CAP)
        b.write("Goal_Velocity", j, config.GOAL_VELOCITY_CAP)
        self.goal = b.read("Present_Position", j)
        b.write("Goal_Position", j, self.goal)    # no jump when torque comes on
        b.torque_on(j)
        # Trust nothing: the force cap is only real if the servo still holds these after the
        # torque-enable edge. Checked before the joint is asked to do anything.
        want = {"Torque_Limit": first, "Acceleration": config.ACCELERATION_CAP,
                "Goal_Velocity": config.GOAL_VELOCITY_CAP}
        got = {r: b.read(r, j) for r in want}
        if got != want:
            raise Aborted(f"after torque on, {j} reads {got}, expected {want}: the caps are not in effect")
        self._next_tick = b.now()

    def _hold_check(self) -> None:
        """At the homing torque limit, can the joint even hold itself where it lies?"""
        self.phase, self._dir = "hold-check", 0
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
            if self._staged:
                # Still staged (an abort or Ctrl-C before unstaging finished): releasing torque
                # would drop the arm. Everything holds; only a person supporting it may let go.
                self._event("the arm is still in its staging pose (" + ", ".join(self._staged) + ") and holding")
                if self.confirm_release is None:
                    errors.append("arm left staged and holding: no one to confirm a release")
                    self._event("RESTORE INCOMPLETE:\n  " + "\n  ".join(errors))
                    return errors                     # holds everything, EEPROM left widened
                step("confirm release", self.confirm_release)
            elif aborted and self.confirm_release is not None:
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
        fw = self.limits[0] if d < 0 else self.limits[1]
        # Without the blocked-or-real test the plausibility window bounds the search; with it, the
        # whole widened window is searched and a far stall is tested instead of trusted or rejected.
        search = fw if self.kin is not None else (max(fw, expected - self.tolerance) if d < 0
                                                  else min(fw, expected + self.tolerance))
        self._history = []
        self._find_stop(r, d, expected, search)
        if self._history:
            self._event(f"{end}: retracing {len(self._history)} test move(s) back to the start pose")
            self._retrace()
        self._event(f"{end}: {r.verdict}: {r.note}")
        return r

    def _find_stop(self, r: EndResult, d: int, expected: int, search: int) -> None:
        end = r.end
        p0 = self._read_pos()
        self._event(f"{end}: touch 1 (fast) toward {expected}, search limit {search}")
        anchor, uncovered, tried, n_same = None, None, set(), 0
        self._search, self._d, self._too_heavy = search, d, []
        retouch = True
        while True:
            if not retouch:                                 # a worse test move was just undone:
                retouch, p = True, anchor                   # back at the anchor pose, pick the next test
                cand = self._choose_test(p, uncovered, tried)
                if cand is None or len(r.tests) >= config.MAX_PERTURBATIONS:
                    r.verdict, r.stop = "blocked", anchor
                    r.note = f"stalled at {anchor}; every test move made it worse or none was left"
                    return
                self._run_test(r, cand, tried, p, d, end)
                continue
            t1 = self._approach(d, config.STEP_FAST_TICKS, search, f"{end}-touch1")
            r.touches.append(t1)
            if t1.kind == "soft_limit":
                self._backoff(t1.position, d, end)
                r.verdict, r.note = "no_stop", f"reached the search limit {search} without stalling"
                return
            p = t1.position
            near = abs(p - expected) <= self.tolerance and not (self.verify_all or self._force)
            # Gravity can only explain a stall away from the stop: AT a real stop it stalls anyway.
            weak = None if near else self._weakness(p)
            if weak:
                self._backoff(p, d, end)
                r.verdict, r.stop, r.note = "weak", p, weak
                if abs(p - p0) < config.STUCK_AT_START_TICKS:
                    r.note += f"; it could not move off its start pose ({p0})"
                return
            if anchor is not None:
                last = r.tests[-1]
                last["stall_after"] = p
                if abs(p - anchor) <= config.CONFIRM_TICKS:
                    last["outcome"] = "same stop"
                    n_same += 1
                    uncovered -= set(map(tuple, last["moves_pairs"]))
                elif d * (p - anchor) > config.CONFIRM_TICKS:
                    last["outcome"] = f"got further: {anchor} -> {p}, it was blocked"
                    self._event(f"{end}: moving {last['joint']} cleared it: {anchor} -> {p}")
                    anchor, uncovered, n_same = None, None, 0
                else:
                    last["outcome"] = f"stopped shorter ({p} vs {anchor}): that pose is worse, undone"
                    self._backoff_far(p, d)
                    self._undo_last()
                    retouch = False
                    continue
            if anchor is None:
                if near:
                    break                                   # where the stop should be: measure it
                if self.kin is None:                        # no way to test it: never push it twice
                    self._backoff(p, d, end)
                    r.verdict, r.stop = "suspect", p
                    r.note = f"stalled at {p}, {p - expected:+d} from the calibrated end"
                    if abs(p - p0) < config.STUCK_AT_START_TICKS:
                        r.note += (f"; it could not move off its start pose ({p0}): torque limit "
                                   f"{self.torque_limit}/1000 is too weak for this joint here, or something blocks it")
                    return
                anchor, uncovered = p, self._coverable(p)
                self._event(f"{end}: stalled at {p} ({p - expected:+d} vs calibration): testing whether "
                            "it is the real stop or something in the way")
            if n_same and not uncovered:
                break                                       # every pair that can move, moved: real stop
            cand = None if len(r.tests) >= config.MAX_PERTURBATIONS else self._choose_test(p, uncovered, tried)
            if cand is None:
                if n_same:                                  # out of moves, and every test said "same"
                    r.untested = sorted("-".join(x) for x in uncovered)
                    break
                self._backoff(p, d, end)
                r.stop = anchor
                if self._too_heavy:
                    r.verdict = "weak"
                    r.note = (f"stalled at {anchor}; the moves that might clear it ({', '.join(sorted(set(self._too_heavy)))}) "
                              f"would leave it too heavy for {self.torque_limit}/1000: too weak, not blocked")
                else:
                    r.verdict = "blocked"
                    r.note = (f"stalled at {anchor} ({anchor - expected:+d} vs calibration) and could not tell or "
                              f"clear it in {len(r.tests)} test moves")
                return
            self._run_test(r, cand, tried, p, d, end)

        # the measurement: slow touches at this pose
        self._backoff(t1.position, d, end)
        self._event(f"{end}: touch 2 (slow)")
        t2 = self._approach(d, config.STEP_SLOW_TICKS, search, f"{end}-touch2")
        r.touches.append(t2)
        self._backoff(t2.position, d, end)
        agree = t2.kind == "stop" and abs(t2.position - t1.position) <= config.REPEAT_TOLERANCE_TICKS
        stop = t2.position
        if not agree and t2.kind == "stop":                 # a third touch breaks the tie
            self._event(f"{end}: touches disagree ({t1.position} / {t2.position}): touch 3 (slow)")
            t3 = self._approach(d, config.STEP_SLOW_TICKS, search, f"{end}-touch3")
            r.touches.append(t3)
            self._backoff(t3.position, d, end)
            pairs = [(a, b) for a, b in ((t1, t3), (t2, t3)) if b.kind == "stop"
                     and abs(a.position - b.position) <= config.REPEAT_TOLERANCE_TICKS]
            if pairs:
                agree, stop = True, pairs[-1][1].position
        if not agree:
            r.verdict = "unrepeatable"
            r.note = "touches " + ", ".join(f"{t.kind}@{t.position}" for t in r.touches[-3:])
            return
        r.stop = stop
        if r.tests:
            same = sum(1 for t in r.tests if t.get("outcome") == "same stop")
            cleared = [t["joint"] for t in r.tests if str(t.get("outcome", "")).startswith("got further")]
            r.verdict = "ok"
            r.note = f"stop {stop} ({stop - expected:+d} vs calibration)"
            if cleared:
                r.note += f"; it was blocked until {', '.join(cleared)} moved out of the way"
            if same:
                r.note += f"; the same stop under {1 + same} different poses"
            if r.untested:
                r.note += f" (could not test: {', '.join(r.untested)})"
        elif abs(stop - expected) > self.tolerance:
            r.verdict, r.note = "suspect", f"{stop - expected:+d} from the calibrated end"
        else:
            r.verdict, r.note = "ok", f"stop {stop} ({stop - expected:+d} vs calibration), touches agree"

    # ------------------------------------------------------------------ blocked or real: helpers
    def _span_note(self, res: JointResult) -> str:
        stops = {e.end: e.stop for e in res.ends if e.verdict == "ok"}
        if set(stops) != {"min", "max"}:
            return ""
        deg = (stops["max"] - stops["min"]) * 360 / 4095
        design = config.DESIGN_SPAN_DEG[self.joint]
        if deg < design - config.SPAN_SHORT_DEG:
            return (f"travel {deg:.0f} deg is {design - deg:.0f} deg short of the design {design}: one end may "
                    "still be something in the way")
        return f"travel {deg:.0f} deg (design {design})"

    def _pose(self) -> dict[str, int]:
        return {m: self.bus.read("Present_Position", m) for m in config.JOINTS}

    def _weakness(self, p: int) -> str | None:
        """A plain reason if gravity alone explains a stall at p, else None."""
        if self.gravity_fn is None:
            return None
        g = self.gravity_fn({**self._pose(), self.joint: p})[self.joint]
        need = config.WEAK_FACTOR * g + config.WEAK_FRICTION
        if need > self.torque_limit:
            return (f"stalled at {p}, where gravity alone needs ~{need:.0f}/1000 (1.5 x {g:.0f} + 40) and the "
                    f"limit is {self.torque_limit}/1000: too weak, not blocked")
        return None

    def _pairs(self) -> list[tuple[str, str]]:
        """Pairs that could be touching when this joint stalls: a link it moves against one it does not
        (or the base, or the table), minus neighbours (their contact IS a joint's own stop)."""
        from .kinematics import CHAIN
        order = config.JOINTS
        k = order.index(self.joint)
        moved = [CHAIN[j] for j in order[k:]]
        fixed = [CHAIN[j] for j in order[:k]] + ["base", "table"]
        chain = ["base"] + [CHAIN[j] for j in order]
        adjacent = {(chain[i], chain[i + 1]) for i in range(len(chain) - 1)}
        return [(a, b) for a in moved for b in fixed if (b, a) not in adjacent and (a, b) not in adjacent]

    def _moved_pairs(self, before: dict, after: dict) -> set[tuple[str, str]]:
        """Pairs whose relative position a pose change shifts by more than PAIR_MOVED_MM."""
        out = set()
        for a, b in self._pairs():
            A0, A1 = before[a], after[a]
            if b == "table":
                shift = float(np.mean(np.abs(A1[:, 2] - A0[:, 2])))        # only height matters
            elif b == "base":
                # the base may be round about the pan axis: turning about it changes nothing there
                ax = self.kin.pan_axis_xy()
                r0, r1 = np.linalg.norm(A0[:, :2] - ax, axis=1), np.linalg.norm(A1[:, :2] - ax, axis=1)
                shift = float(np.mean(np.abs(A1[:, 2] - A0[:, 2]) + np.abs(r1 - r0)))
            else:
                c0, c1 = before[b].mean(0), after[b].mean(0)
                shift = float(np.mean(np.linalg.norm((A1 - c1) - (A0 - c0), axis=1)))
            if shift * 1000 >= config.PAIR_MOVED_MM:
                out.add((a, b))
        return out

    def _test_moves(self, p: int):
        """(other joint, target tick, pairs it moves, torque cap) for every allowed test move."""
        pose = {**self._pose(), self.joint: p}
        before = self.kin.corners(pose)
        out = []
        for m in self.others:
            if m in self.limp:
                continue
            lo = self.cal[m]["range_min"] + config.PERTURB_MARGIN_TICKS
            hi = self.cal[m]["range_max"] - config.PERTURB_MARGIN_TICKS
            for sgn in (1, -1):
                target = int(min(max(pose[m] + sgn * config.PERTURB_TICKS, lo), hi))
                if abs(target - pose[m]) < config.PERTURB_TICKS // 2:
                    continue
                after = self.kin.corners({**pose, m: target})
                g = max(self.gravity_fn({**pose, m: pose[m] + f * (target - pose[m])})[m] for f in (0, .25, .5, .75, 1))
                cap = int(min(config.TORQUE_LIMIT_CEILING, max(80, round(config.WEAK_FACTOR * g + config.WEAK_FRICTION))))
                if config.WEAK_FACTOR * g + config.WEAK_FRICTION > config.TORQUE_LIMIT_CEILING:
                    continue
                # the homed joint must still carry itself over the rest of its sweep in the new pose
                rest = [p + f * (self._search - p) for f in (0, .2, .4, .6, .8, 1)] if self._search is not None else [p]
                gj = max(self.gravity_fn({**pose, m: target, self.joint: t})[self.joint] for t in rest)
                if config.WEAK_FACTOR * gj + config.WEAK_FRICTION > self.torque_limit:
                    self._too_heavy.append(m)
                    continue
                out.append((m, target, self._moved_pairs(before, after), cap))
        return out

    def _coverable(self, p: int) -> set[tuple[str, str]]:
        cov = set()
        for _, _, pairs, _ in self._test_moves(p):
            cov |= pairs
        return cov

    def _choose_test(self, p: int, uncovered: set, tried: set):
        """The untried test move that shifts the most still-untested pairs; low cap breaks ties.
        Lifting the link tips off the table is what clears most blocks, so it wins ties too."""
        best, best_key = None, None
        for m, target, pairs, cap in self._test_moves(p):
            if (m, target) in tried:
                continue
            key = (len(pairs & uncovered), len(pairs), -cap)
            if best_key is None or key > best_key:
                best, best_key = (m, target, pairs, cap), key
        return best

    def _run_test(self, r: EndResult, cand, tried: set, p: int, d: int, end: str) -> None:
        m, target, pairs, cap = cand
        tried.add((m, target))
        self._backoff_far(p, d)
        now = self._read_pos()
        if d * (p - now) < config.BACKOFF_TICKS:          # still pressed against it: moving another
            raise Aborted(f"{self.joint} could not back off from {p} (at {now}) before a test move: "
                          "held where it is")              # joint now could push the two together
        self._event(f"{end}: test move: {m} {self.bus.read('Present_Position', m)} -> {target} "
                    f"(cap {cap}/1000), then touch again")
        r.tests.append({"joint": m, "to": target, "moves_pairs": sorted(pairs), "cap": cap})
        self._perturb(m, target, cap)

    def _backoff_far(self, p: int, d: int) -> None:
        self._move_to(p - d * config.PERTURB_BACKOFF_TICKS, config.STEP_MOVE_TICKS, "test-backoff", blocked_ok=True)

    def _perturb(self, m: str, target: int, cap: int) -> None:
        """Move another joint for a test, the homed joint holding firm. Recorded for the retrace."""
        frm = self.bus.read("Present_Position", m)
        if m not in self._staged:
            self._staged[m] = frm                           # away from rest: never released unattended
        self._history.append((m, frm, self._read_pos(), cap))
        self._hold_homed_firm(self.snapshot)
        self._move_other(m, target, lim=cap, what="test move")
        self._set_homed_limit(self.torque_limit)

    def _undo_last(self) -> None:
        m, frm, jpos, cap = self._history[-1]
        self._move_to(jpos, config.STEP_MOVE_TICKS, "retrace", blocked_ok=True)
        now = self._read_pos()
        if abs(now - jpos) > config.HOLD_SAG_MAX_TICKS:   # off the known-clear path back: stop
            raise Aborted(f"retrace: {self.joint} could not get back to {jpos} (at {now}) before moving "
                          f"{m} back: held where it is")
        self._history.pop()
        self._hold_homed_firm(self.snapshot)
        self._move_other(m, frm, lim=cap, what="undoing test move")
        if m in self._staged and abs(self.bus.read("Present_Position", m) - self._staged[m]) <= config.HOLD_SAG_MAX_TICKS \
                and not any(h[0] == m for h in self._history):
            del self._staged[m]
        self._set_homed_limit(self.torque_limit)

    def _retrace(self) -> None:
        """Undo every test move in reverse: the exact path back, so known clear."""
        while self._history:
            self._undo_last()

    def _approach(self, d: int, step: int, search: int, phase: str) -> Touch:
        self.phase = phase
        self._release_push(d)
        s = self._sample()
        # keep the current goal (same direction): resetting it would drop a gravity-loaded joint's preload
        t0, hist, peak, best = s.t, deque([(s.t, s.pos)]), 0, s.pos
        timeout = abs(search - s.pos) / (step * config.CYCLE_HZ) + config.APPROACH_TIMEOUT_EXTRA_S
        push_since = None          # when the goal reached the full lead cap ahead of the joint
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
            lead = d * (self.goal - s.pos)
            # A stop means: pushed with the whole lead cap (the most force allowed) for a full
            # STALL_WINDOW_S and still no progress. "Lead >= 8 at this instant" was not enough: on
            # the real arm a slow touch on shoulder_pan, still ramping up from a backoff (load 48
            # of 80), was called a stop 33 ticks before the real one.
            # "Pushing with everything allowed": the goal a full lead cap ahead, or (when the goal is
            # pinned on the search limit and cannot get that far) the servo's own load at the cap.
            # A joint parked on friction short of the limit reads far less (40 of 80 on the base).
            full = lead >= self.lead_max - step or \
                (lead >= config.STALL_MIN_LEAD_TICKS and abs(s.load) >= config.FULL_PUSH_LOAD_FRAC * self.torque_limit)
            if full:
                push_since = s.t if push_since is None else push_since
            else:
                push_since = None
            if s.t - hist[0][0] >= config.STALL_WINDOW_S and d * (s.pos - hist[0][1]) < config.STALL_PROGRESS_TICKS:
                if push_since is not None and s.t - push_since >= config.STALL_WINDOW_S:
                    return Touch("stop", int(statistics.median(p for _, p in list(hist)[-5:])), s.t, peak)
                # Goal parked on the search limit and the joint stopped short of it with too
                # little lead to call a stall: friction, not a stop. A stiff joint (shoulder_pan
                # on the real arm) parks up to ~STALL_MIN_LEAD_TICKS short, so no distance test.
                if self.goal == search and not full:      # pushing at the cap there = let the window decide
                    return Touch("soft_limit", s.pos, s.t, peak)
            if s.t - t0 > timeout:
                raise Aborted(f"{phase}: no stall and no limit after {timeout:.1f} s (at {s.pos}, goal {self.goal})")

    def _backoff(self, pos: int, d: int, end: str) -> None:
        # blocked_ok: backing off can run into the joint's OTHER stop when it never got far from
        # it (real arm: elbow_flex starts folded on its max stop and could not unfold at 80/1000).
        self._move_to(pos - d * config.BACKOFF_TICKS, config.STEP_MOVE_TICKS, f"{end}-backoff", blocked_ok=True)

    def _move_to(self, target: int, step: int, phase: str, blocked_ok: bool = False) -> None:
        """Step the goal toward target with the same lead cap; a stall on the way aborts."""
        self.phase = phase
        lo, hi = self.limits
        target = min(max(target, lo), hi)
        d = 1 if target >= self._read_pos() else -1
        self._release_push(d)
        s = self._sample()
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
                if blocked_ok:
                    self.goal = s.pos                      # stop pressing, hold here
                    self.bus.write("Goal_Position", self.joint, self.goal)
                    self._event(f"{phase}: blocked at {s.pos} on the way to {target}, holding there")
                    return
                raise Aborted(f"{phase}: blocked at {s.pos} while moving to {target}")
            if s.t - t0 > timeout:
                if blocked_ok:                         # a backoff that cannot get clear: hold there
                    self.goal = s.pos
                    self.bus.write("Goal_Position", self.joint, self.goal)
                    self._event(f"{phase}: could not get to {target} (at {s.pos}), holding there")
                    return
                raise Aborted(f"{phase}: did not reach {target} in {timeout:.1f} s (at {s.pos})")

    def _check_sag(self, pos: int, d: int, best: int, phase: str) -> int:
        """Furthest progress so far; abort when the joint slides back from it (gravity)."""
        best = max(best, pos) if d > 0 else min(best, pos)
        if d * (pos - best) < -config.SAG_TICKS:
            raise Aborted(f"{phase}: {self.joint} moving against the command ({best} -> {pos}): gravity is "
                          f"stronger than torque limit {self.torque_limit}/1000")
        return best

    def _release_push(self, d: int) -> None:
        """Before moving in direction d: if the goal still sits on the other side (pressed into a
        stop the other way, up to a full lead cap), start it from where the joint really is.
        Otherwise the first sample sees the joint a whole lead 'ahead' of its goal."""
        pos = self._read_pos()
        if self.goal is not None and d * (self.goal - pos) < 0:
            self.goal = pos
            self.bus.write("Goal_Position", self.joint, self.goal)
        self._dir = d

    def _next_goal(self, pos: int, d: int, step: int, limit: int) -> int:
        if d > 0:
            return min(self.goal + step, pos + self.lead_max, limit)
        return max(self.goal - step, pos - self.lead_max, limit)

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
        if self.goal is not None and abs(s.pos - self.goal) > config.OVERRUN_TICKS + self.lead_max \
                and self.phase not in ("setup", "restore"):
            raise Aborted(f"{j} at {s.pos}, {s.pos - self.goal:+d} from its goal {self.goal}: not under control")
        if self.goal is not None and self.phase not in ("setup", "restore") and self._ahead(s.pos):
            raise Aborted(f"{j} running ahead of its goal ({self.goal} -> {s.pos}): gravity is pulling it "
                          f"faster than torque limit {self.torque_limit}/1000 can brake")
        lo, hi = self.limits
        if not lo - 20 <= s.pos <= hi + 20:
            raise Aborted(f"{j} at {s.pos}, outside the limits {lo}..{hi}")
        if self._cycle % config.TEMP_VOLT_EVERY == 0:
            temp, volt = b.read("Present_Temperature", j), b.read("Present_Voltage", j) / 10
            row.update(temp=temp, volt=volt)
            # One bad sample is a bus glitch, not heat: on the real arm a steady 33 C wrist_roll
            # read 41, 43 and then 58 in single samples. Real heating takes tens of seconds, so
            # only GLITCH_SAMPLES readings in a row (0.6 s) count.
            self._hot = self._hot + 1 if temp >= config.TEMP_ABORT_C else 0
            self._bad_volt = self._bad_volt + 1 if not config.VOLTAGE_MIN_V <= volt <= config.VOLTAGE_MAX_V else 0
            if self._hot >= config.GLITCH_SAMPLES:
                raise Aborted(f"{j} at {temp} C ({self._hot} readings in a row)")
            if self._bad_volt >= config.GLITCH_SAMPLES:
                raise Aborted(f"supply at {volt:.1f} V ({self._bad_volt} readings in a row)")
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

    def _ahead(self, pos: int) -> bool:
        """Past the goal in the direction of travel by more than a held joint ever is."""
        d = self._dir
        return d != 0 and d * (pos - self.goal) > config.OVERRUN_TICKS

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
