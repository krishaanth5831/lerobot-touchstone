"""Home the whole follower, joint by joint, in one command.

Each joint is an ordinary single-joint run (same Homer, same guards, its own runs/ folder),
in a safe order: light joints first, the two that carry the arm's weight last. The per-joint
settings are what worked on the real arm on 2026-11-03.

A joint that is simply too weak at its torque limit (cannot hold itself, slides back, or
could not move off its start pose) is retried at the next step of its ladder, never above
200/1000. Its failed ends are retried, not the ok ones. Anything else that goes wrong (a
real abort, a restore error, Ctrl-C) stops the whole sequence. A stall somewhere unexpected
(`suspect`) is NOT retried harder: pushing harder into an obstacle is what we must not do.

The result is one merged candidate calibration, never applied.
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

from . import config, report
from .homing import Homer, JointResult, Refused, candidate_calibration


@dataclass
class Step:
    joint: str
    torques: list[int]            # the ladder: first value, then retries if too weak
    widen: int = config.WIDEN_TICKS
    tolerance: int = config.STOP_TOLERANCE_TICKS
    advice: str = ""


PLAN = [
    Step("gripper", [80], advice="nothing between the jaws"),
    Step("wrist_roll", [80], widen=120, tolerance=120, advice="watch the cable winding round the wrist"),
    Step("shoulder_pan", [80], widen=150, tolerance=150, advice="the base sweeps ~220 deg: clear the path"),
    Step("wrist_flex", [120, 150], advice="jaw clear of the table and the forearm"),
    Step("elbow_flex", [120, 150, 200], advice="hand under the forearm"),
    Step("shoulder_lift", [120, 150, 200],
         advice="hand under the upper arm the whole time; Ctrl-C if it falls forward past vertical"),
]

# Signs that the torque limit was too weak, so the next rung of the ladder is worth a try.
# "no stall and no limit": it crawled so slowly the approach timed out, i.e. barely strong enough.
WEAK_ABORTS = ("cannot hold itself", "moving against the command", "no stall and no limit")
WEAK_NOTE = "could not move off its start pose"


def too_weak(res: JointResult) -> bool:
    if res.aborted:
        return any(k in res.aborted for k in WEAK_ABORTS)
    return any(e.verdict == "suspect" and WEAK_NOTE in e.note for e in res.ends)


@dataclass
class SequenceResult:
    runs: list[tuple[JointResult, Path]] = field(default_factory=list)
    final: dict[str, JointResult] = field(default_factory=dict)   # last run per joint
    verdicts: dict[str, dict[str, str]] = field(default_factory=dict)   # joint -> end -> verdict
    stopped: str | None = None                                    # why the sequence stopped early
    skipped: list[str] = field(default_factory=list)
    candidate: dict | None = None
    out_dir: Path | None = None


def run_sequence(bus, cal: dict, runs_dir: Path, plan: list[Step] = PLAN, mode: str = "real",
                 pause: Callable[[Step, int], str] = lambda step, tq: "",
                 on_event: Callable[[str], None] = print, on_sample=None,
                 confirm_release=None) -> SequenceResult:
    """pause(step, torque) -> "" to go, "s" to skip this joint, "q" to stop."""
    seq = SequenceResult()
    for i, step in enumerate(plan):
        todo = ("min", "max")
        for rung, tq in enumerate(step.torques):
            try:
                answer = pause(step, tq).strip().lower()
            except (KeyboardInterrupt, EOFError):
                answer = "q"
            if answer == "q":
                seq.stopped = "stopped by you"
                break
            if answer == "s":
                seq.skipped.append(step.joint)
                break
            on_event(f"\n=== {i + 1}/{len(plan)} {step.joint}: torque limit {tq}/1000, ends {'+'.join(todo)}")
            run_dir = report.new_run_dir(step.joint, runs_dir)
            report.write_backup(run_dir, bus, cal)
            first_write = len(bus.writes)
            homer = Homer(bus, cal, step.joint, torque_limit=tq, widen=step.widen, tolerance=step.tolerance,
                          on_event=on_event, on_sample=on_sample, confirm_release=confirm_release)
            try:
                res = homer.run(todo)
            except Refused as e:
                seq.stopped = f"{step.joint} refused: {e}"
                break
            report.write_run(run_dir, res, homer.rows, bus.writes[first_write:], cal, mode)
            seq.runs.append((res, run_dir))
            seq.final[step.joint] = res
            for e in res.ends:
                seq.verdicts.setdefault(step.joint, {})[e.end] = e.verdict
            on_event(report.summary(res, homer.rows))
            if res.restore_errors:
                seq.stopped = f"{step.joint}: RESTORE INCOMPLETE"
                break
            weak = too_weak(res)
            if res.aborted and not weak:
                seq.stopped = f"{step.joint} aborted: {res.aborted.splitlines()[0]}"
                break
            if not weak:
                break
            if rung + 1 == len(step.torques):
                on_event(f"{step.joint}: still too weak at {tq}/1000, the top of its ladder. Moving on.")
                break
            # retry only what did not come out ok (all of it if it aborted before any end)
            done_ok = {e.end for e in res.ends if e.verdict == "ok"}
            todo = tuple(e for e in todo if e not in done_ok)
            on_event(f"{step.joint}: too weak at {tq}/1000, retrying {'+'.join(todo)} at {step.torques[rung + 1]}/1000")
        if seq.stopped:
            break

    # one merged candidate from every ok end, in run order (a later ok end of a joint wins)
    seq.candidate = candidate_calibration(cal, [r for r, _ in seq.runs])
    seq.out_dir = report.new_run_dir("all", runs_dir)
    (seq.out_dir / "candidate_calibration.json").write_text(json.dumps(seq.candidate, indent=4))
    (seq.out_dir / "backup_calibration.json").write_text(json.dumps(cal, indent=4))
    (seq.out_dir / "sequence.json").write_text(json.dumps({
        "mode": mode, "stopped": seq.stopped, "skipped": seq.skipped, "verdicts": seq.verdicts,
        "runs": [{"dir": str(d), "joint": r.joint, "torque_limit": r.torque_limit, "aborted": r.aborted,
                  "ends": {e.end: {"verdict": e.verdict, "stop": e.stop, "note": e.note} for e in r.ends}}
                 for r, d in seq.runs]}, indent=2))
    return seq


def table(seq: SequenceResult, plan: list[Step] = PLAN) -> str:
    lines = [f"{'joint':<14}{'min':>14}{'max':>14}   torque"]
    for step in plan:
        v = seq.verdicts.get(step.joint)
        if step.joint in seq.skipped:
            lines.append(f"{step.joint:<14}{'skipped':>14}")
            continue
        if v is None:
            lines.append(f"{step.joint:<14}{'not run':>14}")
            continue
        last = seq.final[step.joint]
        lines.append(f"{step.joint:<14}{v.get('min', '-'):>14}{v.get('max', '-'):>14}   {last.torque_limit}/1000"
                     + (f"   ABORTED: {last.aborted.splitlines()[0]}" if last.aborted else ""))
    return "\n".join(lines)
