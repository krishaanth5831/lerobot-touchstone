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


def test_a_trapped_backoff_never_lets_another_joint_move():
    bus = sim(stops={**STOPS, "shoulder_lift": (780.0, 3115.0)}, pose={"shoulder_lift": 800, "elbow_flex": 3100})
    bus.coupled.append(lift_blocked_while_elbow_folded)

    def trap(bus):                       # once it reaches the block, something closes in behind it
        J = bus.joints["shoulder_lift"]
        if J.pos > 990:
            J.stops = (985.0, J.stops[1])
    bus.coupled.append(trap)
    elbow0 = bus.joints["elbow_flex"].pos
    res = homer(bus, "shoulder_lift", torque_limit=150).run(("max",))
    assert res.aborted and "could not back off" in res.aborted
    assert abs(bus.joints["elbow_flex"].pos - elbow0) < 3          # no test move happened


def test_a_blocked_way_back_holds_instead_of_moving_the_other_joint_back():
    bus = sim(stops={**STOPS, "shoulder_lift": (780.0, 3115.0)}, pose={"shoulder_lift": 800, "elbow_flex": 3100})
    bus.coupled.append(lift_blocked_while_elbow_folded)

    def trap(bus):                       # after the elbow moved, the way back for the shoulder closes
        J, E = bus.joints["shoulder_lift"], bus.joints["elbow_flex"]
        if E.pos < 2950 and J.pos > 1600:
            J.stops = (1500.0, J.stops[1])
    bus.coupled.append(trap)
    res = homer(bus, "shoulder_lift", torque_limit=150).run(("max",))
    assert res.aborted and "retrace" in res.aborted
    assert bus.joints["elbow_flex"].pos < 2950                      # not moved back blind
    assert "arm left staged" in " ".join(res.restore_errors)        # and nothing released


def test_folded_shoulder_block_clears_with_real_gravity_from_the_model():
    bus = sim(stops={**STOPS, "shoulder_lift": (780.0, 3115.0)}, pose={"shoulder_lift": 800, "elbow_flex": 3100})
    bus.coupled.append(lift_blocked_while_elbow_folded)
    h = Homer(bus, CAL, "shoulder_lift", on_event=lambda m: None, kin=KIN, torque_limit=150)   # KIN gravity
    res = h.run(("max",))
    e = res.ends[0]
    assert res.aborted is None and e.verdict == "ok" and abs(e.stop - 3115) <= 3, (res.aborted, e.verdict, e.note)
    assert all(t["cap"] > 80 for t in e.tests if t["joint"] == "elbow_flex")   # gravity-sized, not the floor


def test_a_short_travel_tests_the_end_that_was_not_tested():
    # both ends land within 60 of the calibration, but the travel is 30 deg short of the design
    cal = {**CAL, "wrist_flex": {**CAL["wrist_flex"], "range_min": 1100, "range_max": 2850}}
    bus = SimBusFor(cal, {"wrist_flex": (1095.0, 2855.0)})
    h = Homer(bus, cal, "wrist_flex", on_event=lambda m: None, kin=Kinematics(cal), gravity_fn=NO_GRAVITY,
              torque_limit=100)
    res = h.run(("min", "max"))
    assert res.aborted is None
    assert all(e.tests for e in res.ends)                           # both ends were then tested
    assert "short" in res.span_note


def SimBusFor(cal, stops):
    from touchstone.sim import SimBus
    return SimBus(cal, stops={**STOPS, **stops})
