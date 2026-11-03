"""Every number that decides how hard and how fast touchstone pushes the arm.

Everything is in RAW servo units (ticks, 0..1000 torque per mille). We never go through
lerobot's normalisation.

The force on a stop is capped twice, independently:
  1. Torque_Limit (RAM, addr 48) is set to the joint's HOMING torque limit below. This is
     a hard ceiling inside the servo firmware. It is RAM, so a crash followed by a power
     cycle puts the old value back by itself, and writing it never wears the EEPROM.
  2. The goal never runs more than LEAD_MAX_TICKS ahead of where the joint really is.
     The servo is a P controller, so its push is about 9 load units per tick of error at
     P=16: 30 ticks of lead is at most ~27 % torque even if (1) somehow failed.
"""
from pathlib import Path

# The 12 V arm is the follower. Address it by-id, never /dev/ttyACM*: those swap with
# plug order (2026-09-20). Only the follower is homed; the leader runs at ~5 V and its
# preflight voltage check refuses it anyway.
FOLLOWER_PORT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B7B014343-if00"
CALIBRATION_JSON = (
    Path.home() / ".cache/huggingface/lerobot/calibration/robots/so_follower/follower_white.json"
)

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
MOTOR_IDS = {name: i + 1 for i, name in enumerate(JOINTS)}

# Everything touchstone writes goes here (timestamped, never overwritten). It NEVER writes
# into the lerobot calibration folder: a candidate JSON is for you to compare and copy.
RUNS_DIR = Path(__file__).resolve().parents[1] / "runs"

# ------------------------------------------------------------------ torque
# Homing torque limit per joint, 0..1000 = 0..100 % of stall torque (~2.94 N.m at 12 V).
#
#   limit >= 1.5 x (peak gravity torque along the homing sweep) + 40 friction, rounded up
#
# Gravity from scripts/gravity_torque.py: the MJCF link masses, each joint swept over its
# window with the rest of the arm folded as it rests. Friction 40: the base parked 7 ticks
# short of its goal at load 40 without moving (real arm, 2026-11-03), the stickiest joint
# measured; gear friction alone is ~0.05 N.m = 17 (torque-tactile-sensing). x1.5 covers the
# model's masses and the servo's real torque at 11.9 V. Checked against the arm: the folded
# elbow needs 120 for gravity alone and could not unfold at 80; the light joints homed ok at 80.
#
#   joint          gravity peak   at rest   -> limit   on the arm 2026-11-03
#   gripper              1           0         80      ok at 80
#   wrist_roll           0           0         80      ok at 80
#   shoulder_pan         0           0         80      stops found at 80 (unrepeatable: a since-fixed bug)
#   wrist_flex          40           6        100      ok at 120; at 80 stalled at 2876 where gravity
#                                                       is only ~8, so that was an obstacle, not weakness
#   elbow_flex         123         120        230      stuck at its start at 80
#   shoulder_lift       32          20         90      not run (with the elbow folded the arm's weight
#                                                       sits close to the shoulder; extended it is 294)
#
# If a joint stalls before its stop the run says "suspect"; raise it with --torque-limit,
# never past TORQUE_LIMIT_CEILING.
HOMING_TORQUE_LIMIT = {
    "shoulder_pan": 80,
    "shoulder_lift": 100,
    "elbow_flex": 230,
    "wrist_flex": 100,
    "wrist_roll": 80,
    "gripper": 80,
}
TORQUE_LIMIT_CEILING = 400

# ------------------------------------------------------------------ staging
# Folded at rest, the arm's own links block some sweeps (the wrist cannot roll fully with the
# arm folded, 2026-11-03). For such a joint the other joints are first moved, one at a time, to
# a staging pose YOU taught by hand (`touchstone teach --joint X`: pose the rest of the arm so X
# sweeps end to end touching nothing). The mesh model cannot pick these poses: its convex hulls
# already "collide" in the folded rest pose. Unstaging runs in reverse before anything is let go.
STAGING_JSON = Path(__file__).resolve().parents[1] / "staging.json"
STAGE_ORDER = ["shoulder_lift", "elbow_flex", "wrist_flex", "shoulder_pan", "wrist_roll", "gripper"]
# Torque cap while a joint MOVES to or from its staging pose (it holds at its old limit
# otherwise). 1.5 x worst gravity anywhere (scripts/gravity_torque.py) + 40, capped at the
# ceiling: a collision on the way pushes with at most this, never with the 1000 it holds at.
STAGE_TORQUE_LIMIT = {
    "shoulder_lift": 400,   # 1.5 x 294 + 40 = 481, capped: keep the elbow folded while it lifts
    "elbow_flex": 280,      # 1.5 x 154 + 40
    "wrist_flex": 100,      # 1.5 x 40 + 40
    "shoulder_pan": 80,
    "wrist_roll": 80,
    "gripper": 80,
}
STAGE_STEP_TICKS = 2          # 100 ticks/s ~ 9 deg/s while staging
STAGE_LEAD_MAX_TICKS = 40     # goal never more than this ahead of a staging joint

# ------------------------------------------------------------------ motion
CYCLE_HZ = 50                 # one read-decide-write cycle every 20 ms
STEP_FAST_TICKS = 3           # first touch:  150 ticks/s ~ 13 deg/s
STEP_SLOW_TICKS = 1           # second touch:  50 ticks/s ~  4 deg/s (the measurement)
STEP_MOVE_TICKS = 4           # backing off and returning to the start pose
LEAD_MAX_TICKS = 30           # goal never more than this ahead of the joint (see top), at P=16
# The lead cap is a force cap only through the P gain: at P=32 (Feetech's factory default)
# the same lead pushes twice as hard, so the cap is scaled by 16/P and P above this refused.
P_REFERENCE = 16
P_MAX = 32
BACKOFF_TICKS = 40            # after a touch, pull this far away from the stop
# Speed and acceleration caps inside the servo (RAM, restored after the run). Goal_Velocity
# in position mode is the max speed in steps/s (0 = unlimited); Acceleration is in
# 100 steps/s^2. Our own stepping is far slower; these only bound a surprise jump.
GOAL_VELOCITY_CAP = 400
ACCELERATION_CAP = 20

# ------------------------------------------------------------------ stop detection
# A stop = we keep asking for motion, the joint has not moved in STALL_WINDOW_S, and the
# goal is at least STALL_MIN_LEAD_TICKS ahead of it (we are really pushing, not parked).
STALL_WINDOW_S = 0.3
STALL_PROGRESS_TICKS = 2
STALL_MIN_LEAD_TICKS = 8
# The two touches of one end must agree to within this, or the end is not accepted.
REPEAT_TOLERANCE_TICKS = 6
STUCK_AT_START_TICKS = 20     # a first touch this close to where it started = "could not move at all"
# A found stop must lie this close to the end of the current calibrated range. Anything
# further is probably the table, the arm's own body, or gravity winning: "suspect".
STOP_TOLERANCE_TICKS = 60
STOP_TOLERANCE_CEILING = 150
# The candidate range sits this far inside the stops, so a full-range command does not
# keep the motor pressed into the stop. The raw stops are reported as well.
STOP_INSET_TICKS = 5

# ------------------------------------------------------------------ firmware position limits
# The servo clamps every goal to Min/Max_Position_Limit (EEPROM), which lerobot sets to the
# calibrated range. Hand calibration records the stops as the range ends, so the stop sits
# right at that limit and could never be pushed against. We widen the HOMED joint's limits
# by this much for the run (snapshot first, restored and read back after), so a stop up to
# WIDEN_TICKS outside the old range can be reached. Two EEPROM registers, written twice.
WIDEN_TICKS = 100
WIDEN_CEILING = 200

# ------------------------------------------------------------------ abort conditions
VOLTAGE_MIN_V = 10.0          # follower PSU reads ~12.0 V; the leader ~4.6 V (refused)
VOLTAGE_MAX_V = 13.5
TEMP_REFUSE_C = 45            # won't start above this
TEMP_ABORT_C = 55             # stops mid-run above this (STS3215 cuts out at 70)
JUMP_ABORT_TICKS = 150        # one cycle never moves more than ~10 ticks; more = encoder wrap / glitch
START_MARGIN_TICKS = 30
# Gravity stronger than the homing torque limit makes a joint sag instead of push. Before any
# motion the joint must SETTLE within HOLD_CHECK_S at the homing limit: a P servo always gives
# a little under load (~0.3 N.m -> ~11 ticks at P=16), but it must stop within HOLD_SAG_MAX
# and be still for the last 0.2 s. At any time, sliding SAG_TICKS back from its furthest
# progress aborts. The cleanup then puts the normal Torque_Limit back, which catches the joint.
HOLD_CHECK_S = 0.6
HOLD_SAG_MAX_TICKS = 30
SAG_TICKS = 10
# The opposite failure: gravity pulling the joint the way it is commanded, faster than the
# low torque limit can brake. A held joint never runs more than ~torque_limit/9 ticks past its
# goal (P=16), so further than this ahead of the goal = runaway: abort, and the cleanup puts
# the full Torque_Limit back to catch it.
OVERRUN_TICKS = 40       # the joint must start within the calibrated range +- this
TEMP_VOLT_EVERY = 10          # read temperature and voltage every N cycles (5 Hz)
GLITCH_SAMPLES = 3            # temperature/voltage must be out of range this many reads in a row
APPROACH_TIMEOUT_EXTRA_S = 5.0

STATUS_BITS = {0: "voltage", 1: "encoder", 2: "overheat", 3: "overcurrent", 4: "bit4", 5: "overload"}


def describe_status(status: int) -> str:
    names = [name for bit, name in STATUS_BITS.items() if status & (1 << bit)]
    return f"0x{status:02x} ({', '.join(names) or 'unknown bit'})"
