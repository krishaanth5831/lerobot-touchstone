"""The homing sequence against the simulated follower: the happy path, and every way it
must refuse, stop, or decline to trust what it found. Run: env -u PYTHONPATH pytest"""
import json

import pytest

from touchstone import config, report
from touchstone.homing import Homer, Refused, candidate_calibration
from touchstone.sim import NM_PER_LOAD, SimBus

CAL = {
    "shoulder_pan": {"id": 1, "drive_mode": 0, "homing_offset": -1462, "range_min": 792, "range_max": 3288},
    "shoulder_lift": {"id": 2, "drive_mode": 0, "homing_offset": -1382, "range_min": 785, "range_max": 3110},
    "elbow_flex": {"id": 3, "drive_mode": 0, "homing_offset": 1748, "range_min": 927, "range_max": 3135},
    "wrist_flex": {"id": 4, "drive_mode": 0, "homing_offset": -1388, "range_min": 910, "range_max": 3001},
    "wrist_roll": {"id": 5, "drive_mode": 0, "homing_offset": -1433, "range_min": 174, "range_max": 3920},
    "gripper": {"id": 6, "drive_mode": 0, "homing_offset": 1317, "range_min": 2030, "range_max": 3481},
}
STOPS = {"gripper": (2021.0, 3488.0), "wrist_roll": (168.0, 3926.0), "shoulder_lift": (780.0, 3115.0)}


def sim(**kw):
    kw.setdefault("stops", STOPS)
    return SimBus(CAL, **kw)


def home(bus, joint="gripper", ends=("min", "max"), **kw):
    h = Homer(bus, CAL, joint, on_event=lambda m: None, **kw)
    return h, h.run(ends)


def registers(bus, joint):
    return {k: bus.joints[joint].reg[k] for k in
            ("Torque_Enable", "Torque_Limit", "Acceleration", "Goal_Velocity",
             "Min_Position_Limit", "Max_Position_Limit", "Homing_Offset")}


def assert_restored(bus, before):
    for j, regs in before.items():
        assert registers(bus, j) == regs, j


# ---------------------------------------------------------------- happy path
@pytest.mark.parametrize("joint", ["gripper", "wrist_roll"])
def test_finds_both_stops_and_restores_everything(joint):
    bus = sim()
    before = {j: registers(bus, j) for j in config.JOINTS}
    start = bus.joints[joint].pos
    h, res = home(bus, joint)
    assert res.aborted is None and not res.restore_errors
    for e, true in zip(res.ends, STOPS[joint]):
        assert e.verdict == "ok", e
        assert abs(e.stop - true) <= 3          # within ~0.25 deg of the real stop
        assert len(e.touches) == 2
    assert abs(bus.joints[joint].pos - start) <= 4   # back where it started
    assert_restored(bus, before)


def test_push_is_capped_by_torque_limit_and_lead():
    bus = sim()
    h, res = home(bus, "gripper")
    limit = config.HOMING_TORQUE_LIMIT["gripper"]
    assert max(abs(r["load"]) for r in h.rows) <= limit
    assert all(abs(r["goal"] - r["pos"]) <= config.LEAD_MAX_TICKS + config.STEP_MOVE_TICKS + 2
               for r in h.rows if r["goal"] is not None)
    # the servo's own Torque_Limit really was the homing value while it moved
    tl = [v for _, m, r, v in bus.writes if m == "gripper" and r == "Torque_Limit"]
    assert tl[0] == limit and tl[-1] == 1000


def test_candidate_moves_only_ranges_of_ok_ends():
    bus = sim()
    _, res = home(bus, "gripper")
    cand = candidate_calibration(CAL, [res])
    assert cand["gripper"]["range_min"] == res.ends[0].stop + config.STOP_INSET_TICKS
    assert cand["gripper"]["range_max"] == res.ends[1].stop - config.STOP_INSET_TICKS
    assert cand["gripper"]["homing_offset"] == CAL["gripper"]["homing_offset"]
    assert {j: cand[j] for j in config.JOINTS if j != "gripper"} == {j: CAL[j] for j in config.JOINTS if j != "gripper"}


def test_already_resting_on_a_stop():
    # shoulder_lift lies on its stop in the folded rest pose: that stop must still be found
    bus = sim(pose={"shoulder_lift": 781})
    _, res = home(bus, "shoulder_lift", ends=("min",))
    assert res.ends[0].verdict == "ok" and abs(res.ends[0].stop - 780) <= 3


# ---------------------------------------------------------------- refusals: nothing written
def refuse(bus, joint="gripper", **kw):
    with pytest.raises(Refused) as e:
        home(bus, joint, **kw)
    assert bus.writes == []
    return str(e.value)


def test_refuses_the_leader_arm():
    bus = sim()
    for J in bus.joints.values():
        J.reg["Present_Voltage"] = 46
    assert "LEADER" in refuse(bus)


def test_refuses_hot_motor():
    bus = sim()
    bus.joints["elbow_flex"].temp = 50
    assert "cool" in refuse(bus)


def test_refuses_with_torque_on():
    bus = sim()
    bus.joints["shoulder_pan"].reg["Torque_Enable"] = 1
    assert "torque is ON" in refuse(bus)


def test_refuses_when_motor_and_json_disagree():
    # e.g. a previous run lost power while the limits were widened
    bus = sim()
    bus.joints["gripper"].reg["Min_Position_Limit"] = 1930
    assert "differs from the calibration JSON" in refuse(bus)


def test_refuses_status_error_and_out_of_range_start():
    bus = sim()
    bus.joints["wrist_flex"].reg["Status"] = 1 << 5
    assert "overload" in refuse(bus)
    bus = sim(pose={"gripper": 1900})
    assert "outside its calibrated range" in refuse(bus)


@pytest.mark.parametrize("kw", [{"torque_limit": config.TORQUE_LIMIT_CEILING + 1}, {"torque_limit": 0},
                                {"widen": config.WIDEN_CEILING + 1}, {"tolerance": 1000}])
def test_refuses_unsafe_settings(kw):
    refuse(sim(), **kw)


# ---------------------------------------------------------------- not trusting what it found
def test_obstacle_is_suspect_not_a_stop():
    # something in the way 300 ticks before the real stop: the table, the arm's own body
    bus = sim()
    bus.joints["gripper"].obstacle = ("max", 3180)
    _, res = home(bus, "gripper", ends=("max",))
    e = res.ends[0]
    assert e.verdict == "suspect" and len(e.touches) == 1      # no second push into it
    assert candidate_calibration(CAL, [res])["gripper"] == CAL["gripper"]


def test_gravity_winning_is_suspect():
    # enough torque to hold, too little to lift: it stalls at once and must not be believed
    bus = sim(gravity_nm={"shoulder_lift": -0.42})
    _, res = home(bus, "shoulder_lift", ends=("max",), torque_limit=150)
    assert res.ends[0].verdict == "suspect" and len(res.ends[0].touches) == 1
    assert candidate_calibration(CAL, [res])["shoulder_lift"] == CAL["shoulder_lift"]
    assert bus.joints["shoulder_lift"].reg["Torque_Limit"] == 1000


def test_gravity_loaded_joint_with_enough_torque_homes_fine():
    bus = sim(gravity_nm={"shoulder_lift": -0.3})
    _, res = home(bus, "shoulder_lift", torque_limit=300)
    assert res.aborted is None, res.aborted
    assert [e.verdict for e in res.ends] == ["ok", "ok"]
    assert abs(res.ends[0].stop - 780) <= 4 and abs(res.ends[1].stop - 3115) <= 4


def test_joint_that_cannot_hold_itself_aborts_before_moving():
    bus = sim(gravity_nm={"shoulder_lift": -0.6})
    start = bus.joints["shoulder_lift"].pos
    h, res = home(bus, "shoulder_lift", ends=("max",), torque_limit=150)
    assert res.aborted and "cannot hold itself" in res.aborted
    assert not any(r["phase"].startswith("max") for r in h.rows)
    assert abs(bus.joints["shoulder_lift"].pos - start) < 40     # caught after a small sag
    assert bus.joints["shoulder_lift"].reg["Torque_Limit"] == 1000


def test_sliding_back_mid_push_aborts():
    # gravity appears mid-run (as when a joint swings past horizontal): abort, don't chase it
    bus = sim()
    real_sleep = bus.sleep

    def sleep(dt):
        real_sleep(dt)
        if bus.t > 1.5:
            bus.joints["gripper"].gravity_nm = -0.8

    bus.sleep = sleep
    _, res = home(bus, "gripper", ends=("max",))
    assert res.aborted and "against the command" in res.aborted


def test_stop_beyond_the_search_window_is_no_stop():
    bus = sim(stops={**STOPS, "gripper": (2021.0, 3700.0)})
    _, res = home(bus, "gripper", ends=("max",))
    assert res.ends[0].verdict == "no_stop" and res.ends[0].stop is None


def test_without_widening_a_stop_past_the_limit_is_not_reached():
    bus = sim()
    before = registers(bus, "gripper")
    _, res = home(bus, "gripper", ends=("max",), widen=0)
    assert res.ends[0].verdict == "no_stop"
    assert not any(r in ("Min_Position_Limit", "Max_Position_Limit") for _, _, r, _ in bus.writes)
    assert registers(bus, "gripper") == before


# ---------------------------------------------------------------- aborts restore everything
@pytest.mark.parametrize("fault", [("status", 1 << 3), ("temp", 60), ("voltage", 95)])
def test_fault_mid_push_aborts_and_restores(fault):
    bus = sim()
    before = {j: registers(bus, j) for j in config.JOINTS}
    bus.inject(1.0, "gripper", *fault)
    _, res = home(bus, "gripper")
    assert res.aborted and not res.restore_errors
    assert_restored(bus, before)


def test_holding_joint_fault_aborts():
    bus = sim()
    bus.inject(1.0, "elbow_flex", "status", 1 << 2)
    _, res = home(bus, "gripper")
    assert res.aborted and "elbow_flex" in res.aborted


def test_ctrl_c_while_pushing_restores_widened_limits():
    bus = sim()
    before = {j: registers(bus, j) for j in config.JOINTS}
    bus.interrupt_at = 1.2
    released = []
    h = Homer(bus, CAL, "gripper", on_event=lambda m: None, confirm_release=lambda: released.append(1))
    res = h.run()
    assert res.aborted == "interrupted (Ctrl-C)" and released == [1]
    assert_restored(bus, before)


def test_bus_failure_during_cleanup_is_reported_not_hidden():
    bus = sim()
    bus.interrupt_at = 1.0
    real_write = bus.write

    def flaky(reg, motor, value):
        if reg == "Acceleration" and bus.t > 0.5:
            raise ConnectionError("no status packet")
        real_write(reg, motor, value)

    bus.write = flaky
    _, res = home(bus, "gripper")
    assert any("Acceleration" in e for e in res.restore_errors)
    # ...and the rest of the cleanup still ran
    assert bus.joints["gripper"].reg["Min_Position_Limit"] == CAL["gripper"]["range_min"]
    assert "RESTORE INCOMPLETE" in report.summary(res)


def test_encoder_jump_aborts():
    bus = sim()
    real_read = bus.read

    def wrapped(reg, motor):
        v = real_read(reg, motor)
        return v + 2000 if reg == "Present_Position" and motor == "gripper" and bus.t > 1.0 else v

    bus.read = wrapped
    _, res = home(bus, "gripper")
    assert res.aborted and "jumped" in res.aborted


def test_sim_enforces_eeprom_rules():
    bus = sim()
    bus.torque_on("gripper")
    with pytest.raises(RuntimeError):
        bus.write("Min_Position_Limit", "gripper", 0)
    with pytest.raises(RuntimeError):
        bus.write("Present_Position", "gripper", 0)


# ---------------------------------------------------------------- outputs
def test_run_outputs(tmp_path):
    bus = sim()
    run = report.new_run_dir("gripper", tmp_path)
    report.write_backup(run, bus, CAL)
    h, res = home(bus, "gripper")
    report.write_run(run, res, h.rows, bus.writes, CAL, "sim")
    assert {p.name for p in run.iterdir()} == {"backup_calibration.json", "backup_eeprom.json", "trace.csv",
                                              "result.json", "candidate_calibration.json"}
    assert json.loads((run / "backup_eeprom.json").read_text())["gripper"]["range_min"] == 2030
    assert report.new_run_dir("gripper", tmp_path) != run      # never reuses a folder
    text = report.diff(CAL, json.loads((run / "candidate_calibration.json").read_text()))
    assert "gripper" in text and "(unchanged)" in text
