"""Everything a run leaves behind, all under runs/<stamp>_<joint>/ (never overwritten):

    backup_calibration.json   the lerobot JSON as it was before the run
    backup_eeprom.json        what the motors themselves held (offset, limits) before the run
    trace.csv                 every 20 ms cycle: phase, goal, position, load, status, temp, volt
    result.json               the verdict per end, every touch, every register write
    candidate_calibration.json the JSON with range_min/max moved to the stops that passed

Nothing is ever written into the lerobot calibration folder. Copying a candidate over is
your decision, after `touchstone diff`.
"""
from __future__ import annotations

import csv
import json
import time
from pathlib import Path

from . import config
from .homing import JointResult, candidate_calibration


def new_run_dir(joint: str, root: Path = config.RUNS_DIR) -> Path:
    stamp = time.strftime("%Y%m%d_%H%M%S")
    d = Path(root) / f"{stamp}_{joint}"
    n = 1
    while d.exists():
        n += 1
        d = Path(root) / f"{stamp}_{joint}_{n}"
    d.mkdir(parents=True)
    return d


def write_backup(run_dir: Path, bus, calibration: dict) -> None:
    (run_dir / "backup_calibration.json").write_text(json.dumps(calibration, indent=4))
    eeprom = {m: {"homing_offset": bus.read("Homing_Offset", m),
                  "range_min": bus.read("Min_Position_Limit", m),
                  "range_max": bus.read("Max_Position_Limit", m)} for m in config.JOINTS}
    (run_dir / "backup_eeprom.json").write_text(json.dumps(eeprom, indent=4))


def write_run(run_dir: Path, result: JointResult, rows: list[dict], writes: list, calibration: dict,
              mode: str) -> Path:
    with open(run_dir / "trace.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["t", "phase", "goal", "pos", "load", "status", "temp", "volt"])
        w.writeheader()
        w.writerows(rows)
    out = {"mode": mode, "result": result.to_dict(),
           "register_writes": [{"t": t, "motor": m, "reg": r, "value": v} for t, m, r, v in writes]}
    (run_dir / "result.json").write_text(json.dumps(out, indent=2))
    cand = candidate_calibration(calibration, [result])
    path = run_dir / "candidate_calibration.json"
    path.write_text(json.dumps(cand, indent=4))
    return path


def summary(result: JointResult, rows: list[dict] | None = None) -> str:
    lines = [f"{result.joint}: torque limit {result.torque_limit}/1000, widen {result.widen}, "
             f"tolerance {result.tolerance}"]
    if rows and len(rows) > 2:
        dts = [b["t"] - a["t"] for a, b in zip(rows, rows[1:])]
        lines.append(f"  cycle: mean {1000 * sum(dts) / len(dts):.1f} ms, max {1000 * max(dts):.1f} ms "
                     f"(target {1000 / config.CYCLE_HZ:.0f} ms)")
    for e in result.ends:
        touches = ", ".join(f"{t.kind}@{t.position} (peak load {t.peak_load})" for t in e.touches)
        lines.append(f"  {e.end:>3}: {e.verdict:<12} calibrated {e.expected:>5}  found {e.stop!s:>5}  "
                     f"[{touches}]  {e.note}")
    if result.aborted:
        lines.append(f"  ABORTED: {result.aborted.splitlines()[0]}")
    if result.restore_errors:
        lines.append("  !! RESTORE INCOMPLETE: " + "; ".join(result.restore_errors))
        lines.append("  !! Power-cycle the arm (RAM registers reset), then connect once with lerobot and press"
                     " ENTER to write the calibration JSON back to the motors.")
    return "\n".join(lines)


def diff(old: dict, new: dict) -> str:
    """What changes, in ticks and in what lerobot would report for the same physical pose."""
    lines = [f"{'joint':<14}{'old range':>14}{'new range':>14}{'d_min':>7}{'d_max':>7}"
             f"{'deg-mode shift':>16}{'m100 shift @mid':>17}"]
    for j in config.JOINTS:
        a, b = old[j], new[j]
        if (a["range_min"], a["range_max"]) == (b["range_min"], b["range_max"]):
            lines.append(f"{j:<14}{a['range_min']:>6}..{a['range_max']:<6}{'(unchanged)':>14}")
            continue
        # DEGREES mode reads (pos - mid) * 360/4095: the zero moves with the range midpoint
        mid_a = (a["range_min"] + a["range_max"]) / 2
        mid_b = (b["range_min"] + b["range_max"]) / 2
        deg = (mid_a - mid_b) * 360 / 4095
        # RANGE_M100_100 maps the range onto -100..100: the old midpoint now reads this
        m100 = ((mid_a - b["range_min"]) / (b["range_max"] - b["range_min"])) * 200 - 100
        lines.append(f"{j:<14}{a['range_min']:>6}..{a['range_max']:<6}{b['range_min']:>6}..{b['range_max']:<6}"
                     f"{b['range_min'] - a['range_min']:>+7}{b['range_max'] - a['range_max']:>+7}"
                     f"{deg:>+15.2f}°{m100:>+17.2f}")
    lines.append("\nShifts are how differently lerobot would read the SAME physical pose with the new file."
                 "\nDatasets recorded with the old file and policies trained on them see exactly this offset.")
    return "\n".join(lines)
