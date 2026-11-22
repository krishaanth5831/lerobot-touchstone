# lerobot-touchstone

Your SO-101 follower checks its own range. Each joint creeps gently into its two hard stops,
feels where they really are, and touchstone writes a **candidate** calibration with
`range_min` / `range_max` moved to those stops. Nothing in your lerobot calibration changes
until you copy the candidate over yourself.

A *touchstone* is the reference you test against. Here the arm's own end stops are the
reference, found by touching them.

> **Status: experimental.** 74 simulator tests pass. It has run on one real follower (the
> author's), and several parts have never run on hardware. Read
> [What has been tested on hardware](#what-has-been-tested-on-hardware) before you point it at
> your arm.

## Why not just `lerobot-calibrate`?

`lerobot-calibrate` asks you to move every joint by hand through its full range. Where you
stop is where the range ends, so the result depends on how far you pushed that day. touchstone
finds the ends with the motors themselves, at a low torque limit, touching each stop twice and
checking that the touches agree. On the author's arm, repeated runs found the gripper,
wrist_roll and elbow_flex stops to within a few ticks (shoulder_pan not yet, see below), and
`touchstone diff` shows how far your hand calibration was off.

What it is **not**, yet:

- **Not a first-time calibration.** It starts from an existing lerobot calibration file (it
  needs the offsets and a rough range to search around) and never changes `homing_offset`.
  Run `lerobot-calibrate` once first.
- **Not automatic on connect.** You run it when you want a check.

## Requirements

- An **SO-101 follower** with STS3215 servos on a **12 V** supply, motor IDs 1–6 in lerobot's
  order (`shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll`, `gripper`).
  The preflight refuses anything outside 10–13.5 V. That keeps the ~5 V leader out, but it also
  refuses a 7.4 V follower build.
- An existing lerobot calibration JSON for that arm (from `lerobot-calibrate`).
- Python 3.10+ in an environment with **lerobot and its Feetech extra** (`lerobot[feetech]`).
- Optional:
  - **mujoco**: turns on the "blocked or the real stop?" test moves (see
    [How it works](#how-it-works)). Without it, touchstone says so at the start and reports far
    stalls as `suspect`. Whether mujoco imports decides which behaviour you get on the real arm,
    so check it (`python -c "import mujoco"`). Pass `--no-tests` to switch the test moves off
    on purpose. Tests pass with mujoco 3.8.1 and 4.0.1.
  - **matplotlib**: for `touchstone plot`.

The simulator needs neither an arm nor lerobot.

## Install

```bash
git clone https://github.com/krishaanth5831/lerobot-touchstone.git
cd lerobot-touchstone
conda activate lerobot           # or whichever environment has lerobot[feetech]
pip install -e ".[plot]"         # installs the `touchstone` command (or use `python -m touchstone`)
pip install mujoco               # optional, see above
```

## Point it at your arm

The defaults in [`touchstone/config.py`](touchstone/config.py) are the author's: a serial port
by-id and the calibration file `follower_white.json`. On your machine, pass your own. Both are
**global options**, so they go before the subcommand:

```bash
ls /dev/serial/by-id/            # find your follower's adapter (or run lerobot-find-port)
touchstone --port /dev/serial/by-id/usb-1a86_USB_Single_Serial_XXXXXXXX-if00 \
           --calibration ~/.cache/huggingface/lerobot/calibration/robots/so_follower/<your_id>.json \
           probe
```

Use the by-id path, not `/dev/ttyACM0`: the ACM numbers swap with plug order, so the leader
and follower can trade places. The calibration folder name (`so_follower` here) depends on
your lerobot version; use whichever file `lerobot-calibrate` wrote for the follower.

The examples below leave out `--port` / `--calibration` to stay short. Add them, or edit the
two constants in `config.py` once.

The per-joint torque limits in `config.py` were worked out for the author's arm (gravity from
the SO-101 model, checked on the arm). A standard SO-101 should be similar, but watch your
first runs.

## Quick start: watch it in the simulator

Even the simulator reads a calibration JSON (it builds the simulated arm around your ranges).
Use your own, or save this example as `example_cal.json`:

```json
{
  "shoulder_pan":  {"id": 1, "drive_mode": 0, "homing_offset": -1462, "range_min": 792,  "range_max": 3288},
  "shoulder_lift": {"id": 2, "drive_mode": 0, "homing_offset": -1382, "range_min": 785,  "range_max": 3110},
  "elbow_flex":    {"id": 3, "drive_mode": 0, "homing_offset": 1748,  "range_min": 927,  "range_max": 3135},
  "wrist_flex":    {"id": 4, "drive_mode": 0, "homing_offset": -1388, "range_min": 910,  "range_max": 3001},
  "wrist_roll":    {"id": 5, "drive_mode": 0, "homing_offset": -1433, "range_min": 174,  "range_max": 3920},
  "gripper":       {"id": 6, "drive_mode": 0, "homing_offset": 1317,  "range_min": 2030, "range_max": 3481}
}
```

```bash
touchstone --calibration example_cal.json home --joint gripper --sim --realtime   # live readout at real speed
touchstone plot runs/<the run it printed>      # opens a window: position, goal, load, the touches
touchstone diff runs/<the run it printed>      # what the candidate would change
touchstone --calibration example_cal.json home-all --sim                          # the whole arm
```

The simulated run also prints the simulator's true stops, so you can see how close it got.

## On the real follower: the first session

> **Use `--no-tests` on the real arm for now.** Every completed hardware run so far was made
> without the blocked-or-real-stop test moves. With mujoco installed and no `--no-tests`, a far
> stall makes touchstone move *other* joints by itself, a path that has run on the arm only
> once (and aborted). The commands below include the flag.

```bash
touchstone probe                              # 1. READ-ONLY: voltage, temps, limits, json vs motor
touchstone home --joint gripper --end none --no-tests   # 2. rehearsal: torque on at the homing limit, hold, restore. NO motion
touchstone probe                              # 3. must show no "motor differs from json"
touchstone home --joint gripper --no-tests    # 4. the real thing (asks you to type 'gripper' first)
touchstone plot runs/<run>
```

If `probe` says `motor differs from json` for a joint, `touchstone sync --joint <joint>` writes
that joint's offset and limits from the JSON back into the motor, with torque off, so nothing moves.

Step 2 is the first time anything talks to real servos. It checks the register writes, the
firmware-limit widen-and-restore round trip, that the low torque limit really is in effect
after torque-on, and the real cycle time (printed as `cycle: mean ... ms`). Don't skip it.

Before you run anything:

- Rest the arm folded with **torque off on every joint** (it refuses otherwise).
- Clear the area, nothing between the jaws, and keep a hand near the power switch.
- Without `--no-tests` (or with staging poses taught), joints other than the one being homed
  may move on their own.
- Ctrl-C stops it at any time and still restores everything.

Run the gripper 3 times and compare the stops: that is your repeatability number. Then try the
other joints one at a time: `wrist_roll` → `shoulder_pan` → `wrist_flex` → `elbow_flex` →
`shoulder_lift`. The last two carry the arm's weight, so keep a hand under the arm. Use
`--end min` / `--end max` to home one end only.

Useful options for `home`: `--torque-limit` (per mille, up to 400), `--widen` (how far the
firmware limits open, up to 200 ticks), `--tolerance` (how far from the calibrated end a stop
may be and still count, up to 150), `--no-tests`, `--verify-all` (test every stall, even near
ones). `touchstone home --help` lists them all.

## The whole arm in one command

```bash
touchstone home-all --no-tests                                   # gripper, wrist_roll, shoulder_pan, wrist_flex, elbow_flex, shoulder_lift
touchstone home-all --no-tests --only elbow_flex shoulder_lift   # just these, still in the safe order
```

It waits before each joint (ENTER = go, s = skip, q = stop). A joint that is too weak at its
torque limit is retried one step up its ladder, and only its failed ends are retried. The
ladders are in [`touchstone/sequence.py`](touchstone/sequence.py): wrist_flex 100 → 120,
elbow_flex 230 → 280, shoulder_lift 100 → 150 → 200, the rest stay at 80. A stall somewhere
unexpected is never pushed harder. It ends with one merged candidate in `runs/<stamp>_all/`.

## What has been tested on hardware

Everything below ran on **one** SO-101 follower (12 V, stock jaws) on 2026-11-03 and
2026-11-04. No other arm has run it.

| Part | On the real arm? |
|---|---|
| `probe` (read-only) | yes, used throughout |
| `home --end none` rehearsal | yes: gripper, wrist_roll, shoulder_pan, wrist_flex, elbow_flex |
| `home` (one joint) and `home-all` (three whole-arm runs, two of them complete) | yes, but **without** the test moves: these runs used the code from before they were added (commits 9e63a90 and f91ed02), so far stalls were reported as `suspect` |
| Blocked-or-real-stop test moves (on by default when mujoco imports) | **once**: shoulder_lift, 2026-11-04. The first test move (elbow_flex) was itself blocked, so the run aborted safely and restored every register. A full test sequence has never completed on the arm |
| Cleanup after a fault | the abort-and-restore path ran on several aborts. Once, during a Ctrl-C, no motor answered any more (no status packets), and the restore errors were reported as designed (fix: power-cycle, then `sync`) |
| `sync` | written during the first session for a wrist_roll motor whose EEPROM no longer matched the JSON. It leaves no run folder, so the logs don't record its use |
| Staging poses (`teach`, staged `home`) | **never** run on the arm |
| Applying a candidate to the lerobot calibration | **never** done |
| Leader arm, 7.4 V followers, other SO-101s, fin-ray fingers | never |

Stops found on the real arm (raw ticks, `ok` unless noted; torque limit in /1000):

| Joint | min end | max end | Notes |
|---|---|---|---|
| gripper | 2014–2017 (4 runs) | 3470 (4 runs) | at 80. At 150 it read 2002 / 3491: the stock jaws flex, so home every joint at the same limit before you compare |
| wrist_roll | 142–146 (3 runs) | 3952–3955 (3 runs) | at 80. One run from the folded pose stalled at 2699 (`suspect`: the arm was in its own way) |
| shoulder_pan | 676, 754, and 2× `unrepeatable` | 3255, 3316, 3322 | at 80. The runs that passed disagree by up to 78 ticks. **Not trustworthy yet**, cause unknown |
| wrist_flex | 915–921 (3 runs) | 2936–2942 in the last 4 runs (`ok` at 2942, just inside the 60-tick tolerance; `suspect` at 2936–2937). Earlier stalls at 2876 / 2882 | the max end sits ~60 ticks short of the hand calibration (3001): a real stop or the jaw touching the arm, unresolved |
| elbow_flex | 915–927 | 3136–3141 | on 11-03 the folded elbow often could not unfold (`suspect` near its start, see the note below). 230, the default since the fix, gave `ok` at both ends twice |
| shoulder_lift | `no_stop` (to 725), twice | `suspect` at 992 and 1088 | with the arm folded, it blocks itself when the shoulder lifts. **Open problem** |

The 2026-11-03 runs came before a fix (commit 0485a73): a fixed 30-tick goal lead capped every
push near 140/1000 whatever the torque limit said, so the limits above ~140 on that day (gripper
at 150, elbow at 150–300) are nominal. The 2026-11-04 runs have the fix.

## How it works

For one joint, each end:

1. **Touch 1:** creep toward the stop at ~13°/s until the joint stalls, then back off 40 ticks.
2. **Touch 2:** creep in again at ~4°/s. This one is the measurement. Back off. If touches 1 and
   2 disagree by more than 6 ticks, a third touch breaks the tie.
3. **Verdict:**
   - `ok`: the touches agree and the stop is within the tolerance of the calibrated end (60
     ticks by default), or test moves confirmed it.
   - `suspect`: it stalled somewhere else (the table, the arm's own body) and could not be
     tested. It does not push a second time.
   - `no_stop`: it reached the search limit without stalling.
   - `unrepeatable`: the touches disagree.
   - `weak`: gravity alone explains the stall at this torque limit, so the joint is too weak, not
     blocked. `home-all` retries these one step up the ladder.
   - `blocked`: a far stall that test moves could neither confirm nor clear.

A stall means: the goal is a full lead cap ahead of the joint, so it is pushing with everything
allowed, and the joint has not moved for 0.3 s.

**Blocked or the real stop?** A joint's own hard stop does not care where the other joints
are; an obstruction does. So when mujoco is available, a stall far from the calibrated end is
tested. The joint backs off 80 ticks, *another* joint makes a ~20° test move, and the joint
touches again:

- the same tick under poses that move every pair of links that could be touching = the real stop
- further = it was blocked, and the test move cleared it
- shorter = that pose is worse, so it is undone

Which joint moves, and which way, comes from a kinematics model of the SO-101
([`touchstone/kinematics.py`](touchstone/kinematics.py), link positions and gravity only). The
model's collision shapes are 10–30 mm off near the folded pose, so they are not used. Every
test move is retraced in reverse afterwards. If both ends come out `ok` but the travel is
clearly short of the design travel, the untested end is tested as well.

Then the joint returns to where it started and every register goes back to its old value.
Only `ok` ends change the candidate, which is placed 5 ticks inside each stop.

## Safety: how it protects the motors and gears

Every number lives in [`touchstone/config.py`](touchstone/config.py), with the reasoning next to it.

| Guard | Value | Why |
|---|---|---|
| `Torque_Limit` (RAM) | per joint: 80/1000 for gripper, wrist_roll and shoulder_pan, 100 for wrist_flex and shoulder_lift, 230 for elbow_flex. Hard ceiling 400 | A firmware cap on the push. It lives in RAM, so a power cycle undoes it and it never wears the EEPROM. Sized as 1.5 × gravity along the sweep + 40 for friction |
| Goal lead cap | just enough lead to reach 1.25 × the torque limit (≈23 ticks at 80, ≈64 at 230), between 12 and 100 ticks | A second, independent force cap: the servo is a P controller, so the lead bounds the push |
| P gain | lead cap scaled by 16/P; P above 32 refused | The lead cap is only a force cap through P |
| Speed | 150 ticks/s (~13°/s) first touch, 50 ticks/s (~4°/s) measurement; servo `Goal_Velocity` 400 and `Acceleration` 20 capped too | Slow contact, small impact |
| Time on the stop | 0.3 s of no progress, then back off | Never left pressed into a stop |
| Readback after torque-on | `Torque_Limit`, `Goal_Velocity`, `Acceleration` must read back as set | The caps are only real if the servo still holds them. Aborts before any motion |
| Hold check | must settle within 0.6 s (≤ 30 ticks of give) at the homing limit before any motion | If gravity beats the low limit, it aborts before moving |
| Sag / wrong way | sliding 10 ticks back from its best progress aborts | Gravity winning mid-push |
| Runaway | more than 40 ticks past its goal in the direction of travel aborts, and the cleanup brings full torque back | Gravity pulling it faster than the low limit can brake |
| Search window | with test moves: the calibrated range widened by `--widen` (100 by default). Without: calibrated end ± `--tolerance` (60) | It can't wander |
| Plausibility | `ok` only within the tolerance of the calibrated end, unless test moves confirm it | The table or a self-collision is not a joint limit |
| Status register | any alarm bit (voltage, overheat, overcurrent, overload) aborts. The homed joint is checked every cycle, the held joints 5× a second | The servo's own alarms |
| Temperature | refuses to start at ≥ 45 °C, aborts at ≥ 55 °C (3 readings in a row) | The STS3215 cuts out at 70 °C. One-sample bus glitches are ignored |
| Voltage | 10–13.5 V, at start and during the run (3 readings in a row) | A swapped port can't home the leader |
| Encoder jump | > 150 ticks in one 20 ms cycle aborts | Wrap or glitch |
| Preflight | the motor's EEPROM equals the JSON, the joint is inside its range, position mode, torque off everywhere | Nothing is written unless all of these pass |
| Joints resting past their limits | left limp, not torqued | The servo would otherwise jump to its limit at full torque |
| Test moves | ≤ ~20° (230 ticks), 30 ticks inside the moving joint's range, torque 1.5 × gravity + 40 (80–400), the homed joint backed off first, at most 6 per end, retraced in reverse | A test move must not push two links together |
| Cleanup | `finally`, Ctrl-C ignored during cleanup, each step independent and read back | One failed write never skips the rest. Failures are printed loudly |
| After an abort | holds still and asks you to support the arm before releasing torque | No sudden drop |

**The one EEPROM write.** The servo clamps every goal to `Min/Max_Position_Limit`, which
lerobot sets to your hand-recorded range. That range ends at (or short of) the stops, so the
stop could never be pushed against. For the homed joint only, touchstone widens those two
registers by 100 ticks for the run. It snapshots them first, restores them afterwards and reads
them back. If power dies mid-run, the next preflight sees the mismatch and refuses; run
`touchstone sync --joint <joint>` (or connect once with lerobot and press ENTER) to rewrite them
from the JSON. `--widen 0` skips the write, but then a stop just past the old limit reads as
`no_stop`.

## What it writes

Every run goes to `runs/<stamp>_<joint>/` (git-ignored, never overwritten; `--runs-dir`
changes the folder):

```
backup_calibration.json     the lerobot JSON before the run
backup_eeprom.json          what the motors held before the run
trace.csv                   every 20 ms: phase, goal, position, load, status, temp, volt
result.json                 verdicts, touches, test moves, and every register write made
candidate_calibration.json  range_min/max moved to the ok stops (5 ticks inside them)
```

`home-all` also writes `runs/<stamp>_all/` with the merged `candidate_calibration.json`, the
backup, and `sequence.json` (every joint's verdicts).

**It never writes into `~/.cache/huggingface/lerobot/calibration/`.** Applying a candidate is
a manual copy, after reading `touchstone diff`.

## Before you apply a candidate

- **`homing_offset` never changes**, only `range_min` / `range_max`, so datasets recorded with
  your current zero stay valid.
- **Changing the range still shifts readings.** lerobot normalises with the range:
  `RANGE_M100_100` maps min..max onto −100..100, and `DEGREES` measures from the range
  *midpoint*. A few ticks of new range therefore shift what the same pose reads, and policies
  trained on old data see that shift. `touchstone diff` prints it per joint.
- **wrist_roll** gets real stops. lerobot's own calibration treats it as a full 0–4095 turn,
  which is wrong for an arm with physical stops (the author's has ~334° of travel).
- **Gripper fingers.** With stock jaws, "closed" is a hard stop and the gripper homes both
  ends. Soft fingers (fin-ray style) bend, so the closed reading depends on the torque limit:
  home those with `--end max`.

## Staging poses (manual fallback, never run on the arm)

When the arm is folded, its own links can block a sweep. `touchstone teach --joint X` records a
pose you set by hand (torque off, read-only) in `staging.json` at the repo root. After that,
`home` and `home-all` first move the other joints to that pose, one at a time at a capped
torque, and move them back before anything is released. `--no-stage` ignores taught poses,
and `teach --show` / `teach --forget --joint X` list or remove them. This works in the
simulator but has **never run on the real arm**, and the test moves above are the intended
replacement.

## Tests

```bash
pip install pytest mujoco
env -u PYTHONPATH python -m pytest -q
```

`env -u PYTHONPATH` keeps stray pytest plugins (ROS installs add some) out of the run.
`tests/test_blocked.py` imports the kinematics model, so it needs mujoco. Expected: **74 passed**
in about 10 s.

The simulator ([`touchstone/sim.py`](touchstone/sim.py)) models P control, the torque-limit
clip, 0.05 N·m gear friction and encoder jitter (values measured on a real SO-101). It also
models hard stops, the firmware limit clamp, gravity, obstacles that depend on another joint's
pose, and the overload trip. It enforces the servo's own rules too: EEPROM writes need torque off
and Lock=0, and read-only registers reject writes. The tests cover:

- the happy path, with each stop found to within 3 ticks
- every refusal, checking that no write happened
- obstacles, gravity stalls, gravity sag and runaway
- stops beyond the search window
- faults and Ctrl-C mid-push, checking that every register is restored
- a bus failure during cleanup, checking it is reported
- encoder jumps and one-sample temperature glitches
- blocked-or-real-stop test moves: a block found, cleared and retraced; a real stop confirmed
  under other poses
- staging order and release rules, `home-all` ladders and stopping, and `sync`

## Project layout

```
touchstone/
  cli.py                 the `touchstone` command: probe, home, home-all, teach, sync, plot, diff
  homing.py              one joint's run: preflight, setup, touches, test moves, restore
  sequence.py            home-all: the safe order, torque ladders, merged candidate
  config.py              every threshold and limit, with the reasoning
  bus.py                 the real follower through lerobot's FeetechMotorsBus (raw ticks)
  sim.py                 the simulated follower used by --sim and the tests
  kinematics.py          link positions and gravity from model_kinematics.xml (needs mujoco)
  model_kinematics.xml   SO-101 bodies, joints and inertia, no meshes
  report.py              run folders, summaries, diff
  sync.py                write one joint's offset + limits from the JSON into its motor
  collide.py             experimental self-collision model; not used while homing
scripts/
  gravity_torque.py      gravity along each sweep; the numbers behind the torque limits
  decompose_so101.py     builds the (git-ignored) convex-part model that collide.py reads
tests/                   74 simulator tests
runs/                    run logs (git-ignored)
```

The two scripts are the author's tools. They default to the author's paths (an SO-101 MJCF and
calibration file), so check the top of each before you run it.

## Roadmap

1. Prove the blocked-or-real-stop test moves on the arm, starting with shoulder_lift, and
   explain the wrist_flex max end and the shoulder_pan spread.
2. A cheap read-only startup *check* that warns when the arm no longer matches its calibration.
3. Optional first-time calibration (`homing_offset` from the stops), so a brand-new arm needs no
   manual step. Not built.
4. Contribute upstream. lerobot already has open work on automatic calibration
   ([huggingface/lerobot#4826](https://github.com/huggingface/lerobot/pull/4826),
   `lerobot-calibrate --auto`) and on sweeping wrist_roll
   ([#4583](https://github.com/huggingface/lerobot/pull/4583)), so that is the natural place.

## License

There is **no licence file yet**. Until one is added, the code is visible but not licensed for
reuse; open an issue if you want to use it. `touchstone/model_kinematics.xml` is derived from
[TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100)
(`Simulation/SO101/so101_new_calib.xml`, Apache-2.0).
