"""Blocked or the real stop? A joint's own stop ignores the other joints; an obstruction doesn't.
The sim's coupled obstacles depend on another joint's pose, like the folded arm hitting itself."""
import pytest

from touchstone import config
from touchstone.homing import Homer
from touchstone.kinematics import Kinematics
from test_homing import CAL, STOPS, sim

KIN = Kinematics(CAL)
NO_GRAVITY = lambda pose: {j: 0.0 for j in config.JOINTS}     # the sim's own gravity does the physics


def homer(bus, joint, **kw):
    kw.setdefault("gravity_fn", NO_GRAVITY)
    return Homer(bus, CAL, joint, on_event=lambda m: None, kin=KIN, **kw)


def lift_blocked_while_elbow_folded(bus):
    # like the real arm: with the elbow folded the shoulder cannot lift past 1000
    folded = bus.joints["elbow_flex"].pos > 2900
    bus.joints["shoulder_lift"].obstacle = ("max", 1000.0) if folded else None


def test_a_block_that_depends_on_another_joint_is_found_cleared_and_retraced():
    bus = sim(stops={**STOPS, "shoulder_lift": (780.0, 3115.0)}, pose={"shoulder_lift": 800, "elbow_flex": 3100})
    bus.coupled.append(lift_blocked_while_elbow_folded)
    rest = {m: bus.joints[m].pos for m in config.JOINTS}
    h = homer(bus, "shoulder_lift", torque_limit=150)
    res = h.run(("max",))
    e = res.ends[0]
    assert res.aborted is None and not res.restore_errors, res.aborted
    assert e.verdict == "ok" and abs(e.stop - 3115) <= 3, (e.verdict, e.stop, e.note)
    assert "elbow_flex" in e.note and "out of the way" in e.note
    for m in config.JOINTS:                                     # everything back where it started
        assert abs(bus.joints[m].pos - rest[m]) <= 15, (m, bus.joints[m].pos, rest[m])
    assert not h._staged


def test_a_real_stop_far_from_the_calibration_is_confirmed_by_other_poses():
    # the hand calibration says 3001, the joint really stops at 2930 whatever the rest does (real: 2936)
    bus = sim(stops={**STOPS, "wrist_flex": (905.0, 2930.0)})
    h = homer(bus, "wrist_flex", torque_limit=100)
    res = h.run(("max",))
    e = res.ends[0]
    assert res.aborted is None and e.verdict == "ok" and abs(e.stop - 2930) <= 3, (e.verdict, e.note)
    assert len(e.tests) >= 1 and all(t["outcome"] == "same stop" for t in e.tests)
    assert "different poses" in e.note and not h._staged


def test_too_weak_is_called_weak_and_nothing_else_moves():
    bus = sim(stops={**STOPS, "shoulder_lift": (780.0, 3115.0)}, pose={"shoulder_lift": 800})
    bus.joints["shoulder_lift"].obstacle = ("max", 1200.0)
    heavy = lambda pose: {**NO_GRAVITY(pose), "shoulder_lift": 200.0}   # needs 340 > 150
    res = homer(bus, "shoulder_lift", torque_limit=150, gravity_fn=heavy).run(("max",))
    e = res.ends[0]
    assert e.verdict == "weak" and e.tests == [] and "too weak" in e.note


def test_a_blocked_test_move_aborts_and_holds_until_someone_confirms():
    bus = sim(stops={**STOPS, "shoulder_lift": (780.0, 3115.0)}, pose={"shoulder_lift": 800, "elbow_flex": 3100})
    bus.coupled.append(lift_blocked_while_elbow_folded)
    for m in ("elbow_flex", "wrist_flex", "wrist_roll", "gripper", "shoulder_pan"):
        p = bus.joints[m].pos
        bus.joints[m].stops = (p - 30, p + 30)                 # every test move runs into something
    res = homer(bus, "shoulder_lift", torque_limit=150).run(("max",))
    assert res.aborted and "test move" in res.aborted
    assert "arm left staged" in " ".join(res.restore_errors)                # nobody to confirm: holds
    moved = [m for m in config.JOINTS if bus.joints[m].reg["Torque_Enable"] == 1]
    assert moved                                                            # still holding


def test_whole_sequence_with_tests_on_and_a_coupled_block():
    from touchstone.sequence import run_sequence
    bus = sim(stops={**STOPS, "shoulder_lift": (780.0, 3115.0)}, pose={"shoulder_lift": 800, "elbow_flex": 3100})
    bus.coupled.append(lift_blocked_while_elbow_folded)
    import tempfile, pathlib
    seq = run_sequence(bus, CAL, pathlib.Path(tempfile.mkdtemp()), mode="sim", on_event=lambda m: None,
                       kin=KIN, gravity_fn=NO_GRAVITY)
    assert seq.stopped is None, seq.stopped
    assert seq.verdicts["shoulder_lift"]["max"] == "ok", seq.final["shoulder_lift"].ends


def test_a_stall_at_the_expected_stop_is_never_called_weak():
    bus = sim()
    heavy = lambda pose: {**NO_GRAVITY(pose), "gripper": 500.0}
    res = homer(bus, "gripper", torque_limit=80, gravity_fn=heavy).run(("min", "max"))
    assert [e.verdict for e in res.ends] == ["ok", "ok"]
