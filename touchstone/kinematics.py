"""Where the SO-101's links are and what gravity does to them, at a pose given in servo ticks.

From touchstone/model_kinematics.xml (bodies, joints and inertia of so101_new_calib, no meshes).
Ticks -> angle with lerobot's DEGREES formula (0 = middle of the calibrated range), the mapping
B.L.U.S.H checked against this arm. Used while homing to
  - tell a joint too weak for its load from one that is blocked (gravity at the stall pose), and
  - pick which other joint to move, and which way, to test a stall (link positions).
The link SHAPES are not used here: near the fold the collision model is 10-30 mm off.
"""
from __future__ import annotations

from pathlib import Path

import numpy as np

from . import config

MODEL = Path(__file__).resolve().parent / "model_kinematics.xml"
NM_PER_LOAD = 2.94 / 1000
# links that can run into things: everything that moves, but not the base
LINKS = ("shoulder", "upper_arm", "lower_arm", "wrist", "gripper", "moving_jaw_so101_v1")
CHAIN = {"shoulder_pan": "shoulder", "shoulder_lift": "upper_arm", "elbow_flex": "lower_arm",
         "wrist_flex": "wrist", "wrist_roll": "gripper", "gripper": "moving_jaw_so101_v1"}


class Kinematics:
    def __init__(self, calibration: dict[str, dict], model_path: Path = MODEL):
        import mujoco
        self.mj = mujoco
        self.m = mujoco.MjModel.from_xml_path(str(model_path))
        self.d = mujoco.MjData(self.m)
        self.cal = calibration
        self.qadr = {j: self.m.jnt_qposadr[self.m.joint(j).id] for j in config.JOINTS}
        self.dadr = {j: self.m.jnt_dofadr[self.m.joint(j).id] for j in config.JOINTS}

    def rad(self, joint: str, tick: float) -> float:
        c = self.cal[joint]
        if joint == "gripper":
            return float(np.radians(-10 + (tick - c["range_min"]) * 360 / 4095))
        return float(np.radians((tick - (c["range_min"] + c["range_max"]) / 2) * 360 / 4095))

    def _set(self, pose: dict[str, float]) -> None:
        self.d.qpos[:] = 0
        for j, t in pose.items():
            self.d.qpos[self.qadr[j]] = self.rad(j, t)
        self.mj.mj_forward(self.m, self.d)

    def gravity_load(self, pose: dict[str, float]) -> dict[str, float]:
        """Gravity torque on each joint at this pose, in Present_Load units (0..1000)."""
        self._set(pose)
        return {j: abs(float(self.d.qfrc_bias[self.dadr[j]])) / NM_PER_LOAD for j in config.JOINTS}

    def pan_axis_xy(self) -> np.ndarray:
        """Where the shoulder_pan axis meets the table plane (x, y), m."""
        self._set({})
        return self.d.xpos[self.m.body("shoulder").id][:2].copy()

    def corners(self, pose: dict[str, float]) -> dict[str, np.ndarray]:
        """World positions (m) of the 8 corners of each moving link's collision box (8 x 3)."""
        self._set(pose)
        return {b: np.array([self.d.site_xpos[self.m.site(f"{b}_c{k}").id] for k in range(8)]) for b in LINKS}
