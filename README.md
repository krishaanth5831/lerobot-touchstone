# lerobot-touchstone

The SO-101 follower calibrates itself: each joint creeps gently into its hard stops, feels
where they are, and writes a calibration candidate. You no longer move it by hand.

A *touchstone* is the reference you test against. Here the arm's own end stops are the
reference, found by touching them.

Status: **simulation-tested only (37 tests). Never run on the real arm yet.** It sits in
its own repo so it can be tested in isolation before it goes anywhere near lerobot, B.L.U.S.H
or torque-tactile-sensing.

## Start here: watch it in the simulator

```bash
conda activate lerobot
cd ~/Desktop/personal_projects/github/vinea-so101/lerobot-touchstone
pip install -e .                                     # gives you the `touchstone` command
                                                     # (or skip it and use `python -m touchstone`)

touchstone home --joint gripper --sim --realtime     # live readout at real speed
touchstone plot runs/<the run it printed>            # a window: position, goal, load, the touches
touchstone diff runs/<the run it printed>            # what the candidate would change
```

## On the real follower: the first session

```bash
touchstone probe                              # 1. READ-ONLY: voltage, temps, limits, json vs motor
touchstone home --joint gripper --end none    # 2. rehearsal: torque on at 15 %, hold, restore. NO motion
touchstone probe                              # 3. must show no "motor differs from json"
touchstone home --joint gripper               # 4. the real thing (asks you to type 'gripper' first)
touchstone plot runs/<run>
```

Step 2 is the first time any of this talks to real servos. It checks the register writes,
the EEPROM widen-and-restore round trip, that the low torque limit really is in effect after
torque-on, and the real cycle time (printed as `cycle: mean ... ms`). Don't skip it.

Before you run it: rest the arm folded with **torque off on every joint** (it refuses
otherwise), clear the area, keep a hand near the power switch. Ctrl-C stops it at any
time and still restores everything. Run the gripper 3 times and compare the stops: that
is the repeatability number we need before trusting any of it.

Suggested order after that: `wrist_roll` → `shoulder_pan` → `wrist_flex` → `elbow_flex`
→ `shoulder_lift`. The last two carry the arm's weight, so watch the first runs closely.
Use `--end min` / `--end max` to home one end only.

## What one run does

For one joint, each end:

1. **touch 1**: creep toward the stop at ~13°/s until the joint stalls, then back off 40 ticks
2. **touch 2**: creep in again at ~4°/s. This one is the measurement. Back off.
3. **verdict**:
   - `ok`: both touches agree to within 6 ticks and the stop is within 60 ticks of the calibrated end
   - `suspect`: it stalled somewhere else (the table, the arm's own body, gravity winning). It doesn't push a second time.
   - `no_stop`: it reached the search limit without stalling
   - `unrepeatable`: the two touches disagree

Then the joint returns to where it started and every register goes back to its old value.
Only `ok` ends change the candidate.

## How it protects the motors and gears

Every number lives in [`touchstone/config.py`](touchstone/config.py), with the reasoning.

| Guard | Value | Why |
|---|---|---|
| `Torque_Limit` (RAM) | 15–30 % per joint, hard ceiling 40 % | A firmware cap on the push. RAM, so a power cycle undoes it and it never wears the EEPROM |
| Goal lead cap | ≤ 30 ticks ahead of the joint | A second, independent cap. At P=16 that is ≤ ~27 % torque even if the first cap failed |
| Speed | 13°/s, 4°/s for the measurement; servo `Goal_Velocity` / `Acceleration` capped too | Slow contact, small impact |
| Time on the stop | 0.3 s per touch, then back off | Never left pressed into a stop |
| Readback after torque-on | `Torque_Limit`, `Goal_Velocity`, `Acceleration` must read back as set | The caps are only real if the servo still holds them; aborts before any motion |
| P gain | lead cap scaled by 16/P, P > 32 refused | The lead cap is a force cap only through P (factory default is 32) |
| Hold check | must settle within 0.6 s at the homing torque, before any motion | If gravity beats the low torque limit, it aborts before moving |
| Sag / wrong-way | sliding 10 ticks back from its best progress aborts | Gravity winning mid-push |
| Runaway | running > 40 ticks ahead of its goal aborts, and full torque comes back | Gravity pulling it faster than the low limit can brake (arm swinging out) |
| Plausibility | stop must be within 60 ticks of the calibrated end | The table or a self-collision is not a joint limit |
| Search window | never commands past calibrated end ± 60 ticks | It can't wander |
| Status register | any bit (overload, overcurrent, overheat, voltage) aborts | The servo's own alarms |
| Temperature | refuses at ≥ 45 °C, aborts at ≥ 55 °C | STS3215 cuts out at 70 °C |
| Voltage | 10–13.5 V only | The leader (~4.6 V) is refused, so a swapped port can't home the wrong arm |
| Encoder jump | > 150 ticks in one cycle aborts | Wrap or glitch |
| Preflight | motor EEPROM must equal the JSON; joint inside its range; torque off | Nothing is written unless all pass |
| Cleanup | `finally` + Ctrl-C ignored during cleanup, each step independent, read back | One failed write never skips the rest, and failures are printed loudly |
| After an abort | holds still and asks you to support the arm before releasing torque | No sudden drop |

**The one EEPROM write.** The servo clamps every goal to `Min/Max_Position_Limit`, which
lerobot sets to your hand-recorded range. That range ends at the stops, so the stop could
never be pushed against. For the homed joint only, touchstone widens those two registers by
100 ticks for the run. It snapshots them first, restores them afterwards and reads them
back. If power dies mid-run, the next preflight sees the mismatch and refuses. Connect once
with lerobot and press ENTER to rewrite them from the JSON. `--widen 0` skips the write,
but then a stop just past the old limit reads as `no_stop`.

## What it writes

Everything goes to `runs/<stamp>_<joint>/`, which is git-ignored and never overwritten:

```
backup_calibration.json     the lerobot JSON before the run
backup_eeprom.json          what the motors held before the run
trace.csv                   every 20 ms: phase, goal, position, load, status, temp, volt
result.json                 verdicts, touches, and every single register write made
candidate_calibration.json  range_min/max moved to the ok stops (5 ticks inside them)
```

**It never touches `~/.cache/huggingface/lerobot/calibration/`.** Applying a candidate is a
manual copy after reading `touchstone diff`.

## Open question: which fingers are on the follower?

With the stock jaws, "closed" is a hard stop. With the fin-ray fingers it is soft: they
bend, so the closed reading depends on the torque limit. If those are fitted, the gripper's
`min` end measures the fingers, not the joint. Use `--end max` for the gripper, or home
it with the stock jaws.

## Decisions taken for v1 (change them if you disagree)

- **`homing_offset` is never changed**, only `range_min` / `range_max`. Your datasets
  (e.g. `tissues_into_box`) were recorded with the current zero.
- **Changing the ranges still shifts readings.** lerobot normalises with the range:
  `RANGE_M100_100` maps min..max onto −100..100, and `DEGREES` measures from the range
  *midpoint*. A few ticks of new range therefore shift what the same pose reads.
  `touchstone diff` prints that shift per joint. This is why it writes a candidate instead
  of overwriting your calibration on every startup.
- **Follower only, on demand.** No auto-run on connect yet.
- **wrist_roll** gets real stops. lerobot's `calibrate()` hardcodes it to 0–4095, which
  is wrong for your arm (334° of travel with physical stops).

## Tests

```bash
env -u PYTHONPATH python -m pytest -q        # (env -u: the ROS pytest plugins break collection)
```

The simulator ([`touchstone/sim.py`](touchstone/sim.py)) models P-control, the
torque-limit clip, 0.05 N·m gear friction and encoder jitter (measured on this arm in
torque-tactile-sensing). It also models hard stops, the firmware limit clamp, gravity, the
overload trip, and the servo's own rules: EEPROM writes need torque off with Lock=0, and
read-only registers reject writes. The tests cover:

- the happy path, with the stop found to within 3 ticks
- every refusal, checking that no write happened
- obstacles, gravity stalls and gravity sag
- stops beyond the search window
- faults and Ctrl-C mid-push, checking that every register is restored
- a bus failure during cleanup, checking it is reported
- encoder jumps

## Roadmap

1. Real arm: gripper ×3 for repeatability, then the other joints one at a time
2. All joints in one sequence, in a safe order, with a startup *check* (cheap, read-only)
   that warns when the arm no longer matches its calibration
3. Optional `homing_offset` from the stops (stop + fixed per-joint constant), so a brand-new
   arm needs no manual step at all
4. Wire into B.L.U.S.H / torque-tactile-sensing, then an issue + PR to huggingface/lerobot
   (e.g. `lerobot-calibrate --auto`)
