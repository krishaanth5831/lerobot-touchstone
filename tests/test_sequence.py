"""`touchstone home-all`: every joint in turn, retry only when too weak, stop on anything else."""
from touchstone.sequence import PLAN, Step, run_sequence
from test_homing import CAL, STOPS, sim


def go(bus, tmp_path, plan=PLAN, **kw):
    return run_sequence(bus, CAL, tmp_path, plan=plan, mode="sim", on_event=lambda m: None, **kw)


def test_whole_arm_homes_and_merges_one_candidate(tmp_path):
    seq = go(sim(), tmp_path)
    assert seq.stopped is None
    assert all(v == {"min": "ok", "max": "ok"} for v in seq.verdicts.values()) and len(seq.verdicts) == 6
    assert (seq.out_dir / "candidate_calibration.json").exists()
    for j in CAL:   # homing_offset never touched; every range moved to its found stops
        assert seq.candidate[j]["homing_offset"] == CAL[j]["homing_offset"]


def test_too_weak_joint_climbs_its_ladder(tmp_path):
    # elbow rests folded on its max stop and gravity holds it there harder than 120/1000 can lift
    bus = sim(stops={**STOPS, "elbow_flex": (900.0, 3135.0)}, pose={"elbow_flex": 3133},
              gravity_nm={"elbow_flex": 0.38})
    seq = go(bus, tmp_path, plan=[Step("elbow_flex", [120, 150, 200])])
    tqs = [r.torque_limit for r, _ in seq.runs]
    assert tqs[0] == 120 and len(tqs) >= 2 and seq.stopped is None
    assert seq.verdicts["elbow_flex"]["min"] == "ok"


def test_obstacle_is_not_pushed_harder(tmp_path):
    bus = sim()
    bus.joints["wrist_flex"].obstacle = ("max", 2800.0)
    seq = go(bus, tmp_path, plan=[Step("wrist_flex", [120, 150])])
    assert [r.torque_limit for r, _ in seq.runs] == [120]          # no retry at 150
    assert seq.verdicts["wrist_flex"]["max"] == "suspect"


def test_a_real_abort_stops_the_sequence(tmp_path):
    bus = sim()
    bus.inject(1.0, "gripper", "status", 1 << 3)
    seq = go(bus, tmp_path)
    assert seq.stopped and "gripper aborted" in seq.stopped
    assert list(seq.verdicts) == [] or list(seq.final) == ["gripper"]


def test_skip_and_quit(tmp_path):
    answers = iter(["s", "", "q"])
    seq = go(sim(), tmp_path, pause=lambda step, tq: next(answers))
    assert seq.skipped == ["gripper"] and list(seq.final) == ["wrist_roll"] and seq.stopped == "stopped by you"
