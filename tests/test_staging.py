"""Staging: other joints move to a taught pose before the sweep and back to rest after it.
The sim has no self-collision, so these test the sequencing and the release rules."""
import pytest

from touchstone import config
from touchstone.homing import Homer, Refused
from test_homing import CAL, sim

STAGE = {"shoulder_lift": 2300, "elbow_flex": 1800, "wrist_flex": 2100}   # arm opened up


def watched(bus):
    """Record each joint's position whenever its torque is switched off."""
    offs, orig = [], bus.torque_off

    def torque_off(m):
        offs.append((m, bus.joints[m].pos))
        orig(m)
    bus.torque_off = torque_off
    return offs


def homer(bus, joint="wrist_roll", **kw):
    kw.setdefault("stage", STAGE)
    return Homer(bus, CAL, joint, on_event=lambda m: None, **kw)


def rest_of(bus):
    return {m: bus.joints[m].pos for m in config.JOINTS}


def test_stages_sweeps_and_returns_to_rest(tmp_path):
    bus = sim()
    rest = rest_of(bus)
    offs = watched(bus)
    h = homer(bus)
    seen = {}
    h.on_sample = lambda row: row["phase"].endswith("touch1") and seen.update(
        {m: bus.joints[m].pos for m in STAGE})
    res = h.run(("min", "max"))
    assert res.aborted is None and not res.restore_errors
    assert [e.verdict for e in res.ends] == ["ok", "ok"]
    assert all(abs(seen[m] - t) <= 5 for m, t in STAGE.items())          # swept while staged
    for m in STAGE:                                                       # and came back
        assert abs(bus.joints[m].pos - rest[m]) <= 5
    for m, pos in offs:                                                   # never let go away from rest
        if m in STAGE:
            assert abs(pos - rest[m]) <= 5, (m, pos, rest[m])
    assert res.staged == STAGE


def test_rehearsal_stages_and_unstages_without_sweeping():
    bus = sim()
    rest = rest_of(bus)
    h = homer(bus)
    res = h.run(())
    assert res.aborted is None and res.ends == []
    assert {r["phase"] for r in h.rows} >= {"stage shoulder_lift", "stage elbow_flex", "hold-check"}
    assert all(abs(bus.joints[m].pos - rest[m]) <= 5 for m in STAGE)


def test_abort_while_staged_never_releases_without_a_person():
    bus = sim()
    offs = watched(bus)
    bus.inject(12.0, "wrist_roll", "status", 1 << 3)                     # mid-sweep fault
    res = homer(bus).run(("min", "max"))
    assert res.aborted and "arm left staged" in " ".join(res.restore_errors)
    assert not [m for m, _ in offs if m in STAGE]                          # staged joints never let go
    assert all(bus.joints[m].reg["Torque_Enable"] == 1 for m in STAGE)


def test_abort_while_staged_releases_only_after_the_person_confirms():
    bus = sim()
    events = []
    orig = bus.torque_off
    bus.torque_off = lambda m: (events.append(("off", m)), orig(m))
    bus.inject(12.0, "wrist_roll", "status", 1 << 3)
    res = homer(bus, confirm_release=lambda: events.append(("confirm", None))).run(("min", "max"))
    assert res.aborted
    first_confirm = events.index(("confirm", None))
    assert all(i > first_confirm for i, e in enumerate(events) if e[0] == "off" and e[1] in STAGE)


def test_ctrl_c_while_staging_holds():
    bus = sim()
    bus.interrupt_at = 1.0                                                  # during the shoulder_lift move
    res = homer(bus).run(("min", "max"))
    assert res.aborted == "interrupted (Ctrl-C)"
    assert bus.joints["shoulder_lift"].reg["Torque_Enable"] == 1


def test_blocked_staging_move_aborts_at_its_capped_torque():
    bus = sim()
    bus.joints["elbow_flex"].obstacle = ("min", 1950.0)                    # something in the way
    res = homer(bus, confirm_release=lambda: None).run(("min", "max"))
    assert res.aborted and "staging elbow_flex: blocked" in res.aborted
    assert bus.joints["elbow_flex"].reg["Torque_Limit"] == 1000            # back to its holding limit


def test_refuses_a_staging_pose_outside_the_range_and_writes_nothing():
    bus = sim()
    with pytest.raises(Refused, match="staging pose"):
        homer(bus, stage={"elbow_flex": 5000}).run(("min",))
    assert bus.writes == []


def test_teach_then_home_uses_the_taught_pose(tmp_path, monkeypatch, capsys):
    from touchstone import cli
    monkeypatch.setattr(config, "STAGING_JSON", tmp_path / "staging.json")
    monkeypatch.setattr(config, "CALIBRATION_JSON", tmp_path / "cal.json")
    (tmp_path / "cal.json").write_text(__import__("json").dumps(CAL))
    cal_args = ["--calibration", str(tmp_path / "cal.json")]
    assert cli.main(cal_args + ["teach", "--joint", "wrist_roll", "--sim"]) == 0
    taught = __import__("json").loads((tmp_path / "staging.json").read_text())["wrist_roll"]["pose"]
    assert set(taught) == set(config.JOINTS)
    assert cli.main(cal_args + ["home", "--joint", "wrist_roll", "--sim", "--end", "none",
                                "--runs-dir", str(tmp_path)]) == 0
    assert "staging pose from" in capsys.readouterr().out
