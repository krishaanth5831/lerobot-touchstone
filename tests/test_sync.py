"""`touchstone sync`: rewrite one motor's calibration copy from the JSON, never with torque on."""
import pytest

from touchstone.sync import SyncRefused, sync_joint
from test_homing import CAL, sim

PRE_PATCH = {"Homing_Offset": 1589, "Min_Position_Limit": 0, "Max_Position_Limit": 4095}


def reverted_wrist():
    bus = sim()
    bus.joints["wrist_roll"].reg.update(PRE_PATCH)
    return bus


def test_writes_json_values_and_reads_back():
    bus = reverted_wrist()
    out = sync_joint(bus, CAL, "wrist_roll", on_event=lambda m: None)
    r = bus.joints["wrist_roll"].reg
    assert (r["Homing_Offset"], r["Min_Position_Limit"], r["Max_Position_Limit"]) == (-1433, 174, 3920)
    assert out["changed"] and r["Torque_Enable"] == 0
    touched = {m for _, m, _, _ in bus.writes}
    assert touched == {"wrist_roll"}                       # no other motor written
    assert not any(reg == "Torque_Enable" and v == 1 for _, _, reg, v in bus.writes)


def test_refuses_with_torque_on_and_writes_nothing():
    bus = reverted_wrist()
    bus.joints["wrist_roll"].reg["Torque_Enable"] = 1
    with pytest.raises(SyncRefused, match="torque is ON"):
        sync_joint(bus, CAL, "wrist_roll", on_event=lambda m: None)
    assert bus.writes == []


def test_refuses_leader_voltage():
    bus = reverted_wrist()
    bus.joints["wrist_roll"].reg["Present_Voltage"] = 46
    with pytest.raises(SyncRefused, match="not the follower"):
        sync_joint(bus, CAL, "wrist_roll", on_event=lambda m: None)
    assert bus.writes == []


def test_not_confirmed_writes_nothing():
    bus = reverted_wrist()
    with pytest.raises(SyncRefused, match="not confirmed"):
        sync_joint(bus, CAL, "wrist_roll", confirm=lambda m: False, on_event=lambda m: None)
    assert bus.writes == []


def test_already_matching_writes_nothing():
    bus = sim()
    out = sync_joint(bus, CAL, "wrist_roll", on_event=lambda m: None)
    assert not out["changed"] and bus.writes == []
