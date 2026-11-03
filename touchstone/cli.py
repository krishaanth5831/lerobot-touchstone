"""touchstone: find the SO-101 follower's hard stops instead of calibrating by hand.

    touchstone probe                         read-only: voltage, temperature, status, limits
    touchstone home --joint gripper --sim    the whole sequence on the simulated arm
    touchstone home --joint gripper          the real follower (asks you to confirm first)
    touchstone plot runs/<run>               a window: position, goal, load over the run
    touchstone diff runs/<run>               what the candidate would change, and for whom
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from . import config, report
from .bus import load_calibration
from .homing import Homer, Refused

PROBE_REGS = ["Present_Position", "Min_Position_Limit", "Max_Position_Limit", "Homing_Offset", "Torque_Enable",
              "Torque_Limit", "Max_Torque_Limit", "P_Coefficient", "Protective_Torque", "Protection_Time",
              "Overload_Torque", "Present_Voltage", "Present_Temperature", "Status"]


def make_bus(args, calibration):
    if args.sim:
        from .sim import SimBus
        return SimBus(calibration, seed=args.seed, realtime=args.realtime)
    from .bus import FeetechBus
    return FeetechBus(args.port, args.calibration)


def cmd_probe(args) -> int:
    cal = load_calibration(args.calibration)
    bus = make_bus(args, cal)
    try:
        short = {"Present_Position": "pos", "Min_Position_Limit": "min_lim", "Max_Position_Limit": "max_lim",
                 "Homing_Offset": "offset", "Torque_Enable": "torque", "Torque_Limit": "tq_lim",
                 "Max_Torque_Limit": "max_tq", "P_Coefficient": "P", "Protective_Torque": "prot_tq",
                 "Protection_Time": "prot_t", "Overload_Torque": "ovl_tq", "Present_Voltage": "dV",
                 "Present_Temperature": "C", "Status": "status"}
        print(f"{'joint':<14}" + "".join(f"{short[r]:>9}" for r in PROBE_REGS) + "   json range / offset")
        for j in config.JOINTS:
            vals = [bus.read(r, j) for r in PROBE_REGS]
            c = cal[j]
            flag = "" if (vals[1], vals[2], vals[3]) == (c["range_min"], c["range_max"], c["homing_offset"]) \
                else "   <-- motor differs from json"
            print(f"{j:<14}" + "".join(f"{v:>9}" for v in vals)
                  + f"   {c['range_min']}..{c['range_max']} / {c['homing_offset']}{flag}")
        print("\n(dV = supply in 0.1 V: ~120 is the follower, ~46 the leader. Nothing was written.)")
    finally:
        bus.close()
    return 0


def cmd_home(args) -> int:
    cal = load_calibration(args.calibration)
    ends = {"both": ("min", "max"), "none": ()}.get(args.end, (args.end,))
    if not args.sim:
        what = f"move the REAL follower's {args.joint} into its {' and '.join(ends)} stop(s)" if ends else \
            f"torque the REAL follower's {args.joint} at the homing limit and hold it (no motion)"
        print(f"About to {what}, "
              f"torque limit {args.torque_limit or config.HOMING_TORQUE_LIMIT[args.joint]}/1000.\n"
              "Clear the area around the arm, keep a hand near the power switch. Ctrl-C stops it at any time.")
        if input(f"Type '{args.joint}' to start: ").strip() != args.joint:
            print("Not confirmed, nothing done.")
            return 1
    bus = make_bus(args, cal)
    try:
        run_dir = report.new_run_dir(args.joint, args.runs_dir)
        report.write_backup(run_dir, bus, cal)

        last = {"t": -1.0}

        def live(row):
            if sys.stdout.isatty() and row["t"] - last["t"] >= 0.2:
                last["t"] = row["t"]
                print(f"\r  {row['phase']:<16} pos {row['pos']:>5}  goal {row['goal']!s:>5}  load {row['load']:>5}",
                      end="", flush=True)

        def event(msg):
            print(("\r" + " " * 70 + "\r" if sys.stdout.isatty() else "") + msg, flush=True)

        def release():
            input("\nThe arm is holding still. Support it with your hand, then press ENTER to release torque.")

        homer = Homer(bus, cal, args.joint, torque_limit=args.torque_limit, widen=args.widen,
                      tolerance=args.tolerance, on_event=event, on_sample=live,
                      confirm_release=None if args.sim else release)
        try:
            result = homer.run(ends)
        except Refused as e:
            print(f"REFUSED: {e}")
            return 2
        cand = report.write_run(run_dir, result, homer.rows, bus.writes, cal, "sim" if args.sim else "real")
    finally:
        bus.close()
    print("\n" + report.summary(result, homer.rows))
    print(f"\nrun saved to {run_dir}\n  watch it:   touchstone plot {run_dir}\n  compare:    touchstone diff {run_dir}")
    if args.sim:
        truth = bus.joints[args.joint].stops
        print(f"  (sim truth: stops at {truth[0]:.1f} / {truth[1]:.1f})")
    print(f"  candidate:  {cand}  (NOT applied; the lerobot JSON is untouched)")
    return 0 if not result.aborted and not result.restore_errors else 3


def cmd_home_all(args) -> int:
    from .sequence import PLAN, run_sequence, table
    cal = load_calibration(args.calibration)
    plan = [s for s in PLAN if not args.only or s.joint in args.only]
    if not args.sim:
        print("About to home the REAL follower joint by joint:\n")
        for s in plan:
            print(f"  {s.joint:<14} torque {'/'.join(map(str, s.torques))}/1000   {s.advice}")
        print("\nRest the arm folded, torque off, area clear, hand near the power switch.\n"
              "Before each joint it waits for you: ENTER = go, s = skip that joint, q = stop.\n"
              "Ctrl-C stops the joint that is moving and ends the sequence; everything is restored.")
        if input("Type 'all' to start: ").strip() != "all":
            print("Not confirmed, nothing done.")
            return 1
    bus = make_bus(args, cal)
    last = {"t": -1.0}

    def live(row):
        if sys.stdout.isatty() and row["t"] - last["t"] >= 0.2:
            last["t"] = row["t"]
            print(f"\r  {row['phase']:<16} pos {row['pos']:>5}  goal {row['goal']!s:>5}  load {row['load']:>5}",
                  end="", flush=True)
        if row["t"] < last["t"]:      # a new run's clock started again
            last["t"] = -1.0

    def event(msg):
        print(("\r" + " " * 70 + "\r" if sys.stdout.isatty() else "") + msg, flush=True)

    def pause(step, tq):
        if args.sim:
            return ""
        return input(f"\nNext: {step.joint} at {tq}/1000 ({step.advice}). ENTER = go, s = skip, q = stop: ")

    def release():
        input("\nThe arm is holding still. Support it with your hand, then press ENTER to release torque.")

    try:
        seq = run_sequence(bus, cal, args.runs_dir, plan=plan, mode="sim" if args.sim else "real",
                           pause=pause, on_event=event, on_sample=live,
                           confirm_release=None if args.sim else release)
    finally:
        bus.close()
    print("\n" + "=" * 60 + "\n" + table(seq, plan))
    if seq.stopped:
        print(f"\nSTOPPED EARLY: {seq.stopped}")
    print(f"\nmerged candidate: {seq.out_dir / 'candidate_calibration.json'}  (NOT applied)")
    print(report.diff(cal, seq.candidate))
    bad = seq.stopped or any(r.restore_errors for r, _ in seq.runs)
    return 3 if bad else 0


def cmd_plot(args) -> int:
    import csv

    import matplotlib.pyplot as plt

    run = Path(args.run)
    rows = list(csv.DictReader(open(run / "trace.csv")))
    res = json.loads((run / "result.json").read_text())["result"]
    t = [float(r["t"]) for r in rows]
    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(11, 7), height_ratios=[3, 1])
    ax1.plot(t, [int(r["pos"]) for r in rows], label="position", lw=1.4)
    ax1.plot(t, [float(r["goal"]) if r["goal"] not in ("", "None") else float("nan") for r in rows],
             label="goal", lw=0.9, alpha=0.7)
    for e in res["ends"]:
        ax1.axhline(e["expected"], ls=":", c="grey")
        ax1.annotate(f"calibrated {e['end']}", (t[0], e["expected"]), fontsize=8, color="grey")
        for touch in e["touches"]:
            ax1.plot(touch["t"], touch["position"], "o", c="red" if touch["kind"] == "stop" else "orange")
        if e["stop"] is not None:
            ax1.axhline(e["stop"], c="green" if e["verdict"] == "ok" else "red", lw=0.8,
                        label=f"{e['end']} stop: {e['verdict']}")
    ax1.set_ylabel("ticks")
    ax1.legend(loc="best", fontsize=8)
    ax1.set_title(f"{res['joint']}  (torque limit {res['torque_limit']}/1000)"
                  + (f"  ABORTED: {res['aborted'].splitlines()[0]}" if res["aborted"] else ""))
    ax2.plot(t, [int(r["load"]) for r in rows], c="purple", lw=1)
    ax2.axhline(res["torque_limit"], ls="--", c="red", lw=0.8)
    ax2.axhline(-res["torque_limit"], ls="--", c="red", lw=0.8)
    ax2.set_ylabel("Present_Load (/1000)")
    ax2.set_xlabel("s")
    # shade the phases
    start = 0
    for i in range(1, len(rows) + 1):
        if i == len(rows) or rows[i]["phase"] != rows[start]["phase"]:
            if "touch" in rows[start]["phase"]:
                for ax in (ax1, ax2):
                    ax.axvspan(t[start], t[i - 1], color="orange", alpha=0.08)
            start = i
    plt.tight_layout()
    plt.show()
    return 0


def cmd_diff(args) -> int:
    run = Path(args.run)
    old = load_calibration(args.calibration) if args.against_current else \
        json.loads((run / "backup_calibration.json").read_text())
    new = json.loads((run / "candidate_calibration.json").read_text())
    print(report.diff(old, new))
    return 0


def cmd_sync(args) -> int:
    from .sync import SyncRefused, sync_joint
    cal = load_calibration(args.calibration)
    bus = make_bus(args, cal)

    def confirm(msg):
        print(msg)
        return args.sim or input(f"Type '{args.joint}' to write: ").strip() == args.joint

    try:
        sync_joint(bus, cal, args.joint, confirm=confirm)
    except SyncRefused as e:
        print(f"REFUSED: {e}")
        return 2
    finally:
        bus.close()
    print("Now run `touchstone probe`: this row must no longer say 'motor differs from json'.")
    return 0


def main(argv=None) -> int:
    p = argparse.ArgumentParser(prog="touchstone", description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=config.FOLLOWER_PORT)
    p.add_argument("--calibration", type=Path, default=config.CALIBRATION_JSON)
    sub = p.add_subparsers(dest="cmd", required=True)

    def sim_flags(sp):
        sp.add_argument("--sim", action="store_true", help="simulated arm, no hardware")
        sp.add_argument("--realtime", action="store_true", help="with --sim: run at wall-clock speed")
        sp.add_argument("--seed", type=int, default=0)

    sp = sub.add_parser("probe", help="read-only register dump")
    sim_flags(sp)
    sp.set_defaults(fn=cmd_probe)

    sp = sub.add_parser("home", help="find one joint's stops")
    sim_flags(sp)
    sp.add_argument("--joint", required=True, choices=config.JOINTS)
    sp.add_argument("--end", choices=["min", "max", "both", "none"], default="both",
                    help="none = rehearsal: setup, hold check, restore, no motion")
    sp.add_argument("--torque-limit", type=int, default=None,
                    help=f"0..{config.TORQUE_LIMIT_CEILING} per mille (default per joint, see config.py)")
    sp.add_argument("--widen", type=int, default=config.WIDEN_TICKS,
                    help=f"ticks to open the firmware limits for this run (0..{config.WIDEN_CEILING})")
    sp.add_argument("--tolerance", type=int, default=config.STOP_TOLERANCE_TICKS,
                    help="how far from the calibrated end a stop may be and still count")
    sp.add_argument("--runs-dir", type=Path, default=config.RUNS_DIR)
    sp.set_defaults(fn=cmd_home)

    sp = sub.add_parser("home-all", help="home every joint in turn, light joints first, one merged candidate")
    sim_flags(sp)
    sp.add_argument("--only", nargs="+", choices=config.JOINTS, help="just these joints, in the safe order")
    sp.add_argument("--runs-dir", type=Path, default=config.RUNS_DIR)
    sp.set_defaults(fn=cmd_home_all)

    sp = sub.add_parser("sync", help="write one joint's offset + limits from the JSON into its motor (torque off)")
    sim_flags(sp)
    sp.add_argument("--joint", required=True, choices=config.JOINTS)
    sp.set_defaults(fn=cmd_sync)

    sp = sub.add_parser("plot", help="open a window with a run's trace")
    sp.add_argument("run")
    sp.set_defaults(fn=cmd_plot)

    sp = sub.add_parser("diff", help="what a run's candidate changes")
    sp.add_argument("run")
    sp.add_argument("--against-current", action="store_true", help="compare with the live JSON, not the backup")
    sp.set_defaults(fn=cmd_diff)

    args = p.parse_args(argv)
    return args.fn(args)


if __name__ == "__main__":
    sys.exit(main())
