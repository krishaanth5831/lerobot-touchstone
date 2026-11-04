"""Self-collision model of the SO-101 follower: which links touch, and how close they come.

Built from touchstone/model/so101/collision.xml: every link mesh split into convex parts
(scripts/decompose_so101.py), so concave links collide as their real shape, not as a hull.
A table plane sits at the lowest point of the base. Joint angles come from servo ticks with
lerobot's DEGREES formula (angle 0 = middle of the calibrated range), the mapping B.L.U.S.H
checked against this arm (sim/arm_map.py).

Parent/child links are never checked against each other: their contact IS a joint's hard stop.
"""
from __future__ import annotations

from pathlib import Path

import numpy as np

from . import config

MODEL = Path(__file__).resolve().parent / "model/so101/collision.xml"
REPORT_MM = 15.0          # contacts are reported out to this distance


class Collider:
    def __init__(self, calibration: dict[str, dict], model_path: Path = MODEL,
                 offsets_deg: dict[str, float] | None = None, table_dz_mm: float = 0.0):
        import mujoco
        self.mj = mujoco
        spec = mujoco.MjSpec.from_file(str(model_path))
        m0 = spec.compile()
        d0 = mujoco.MjData(m0)
        mujoco.mj_forward(m0, d0)
        base = m0.body("base").id
        zmin = min(d0.geom_xpos[g][2] - m0.geom_rbound[g] for g in range(m0.ngeom) if m0.geom_bodyid[g] == base)
        # tighter: lowest vertex of the base parts
        zs = []
        for g in range(m0.ngeom):
            if m0.geom_bodyid[g] == base and m0.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
                mid = m0.geom_dataid[g]
                v = m0.mesh_vert[m0.mesh_vertadr[mid]:m0.mesh_vertadr[mid] + m0.mesh_vertnum[mid]]
                zs.append((d0.geom_xmat[g].reshape(3, 3) @ v.T).T[:, 2].min() + d0.geom_xpos[g][2])
        self.table_z = (float(min(zs)) if zs else float(zmin)) + table_dz_mm / 1000
        self.offsets = {j: float(np.radians(v)) for j, v in (offsets_deg or {}).items()}
        t = spec.worldbody.add_geom(name="table", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[1, 1, 0.01],
                                    pos=[0, 0, self.table_z])
        t.contype, t.conaffinity = 1, 1
        # Neighbouring links are never checked: their contact IS the joint's hard stop. MuJoCo
        # skips parent/child pairs itself, except where the parent is fixed to the world (base).
        for b in range(1, m0.nbody):
            p = m0.body_parentid[b]
            if p > 0:
                spec.add_exclude(bodyname1=m0.body(p).name, bodyname2=m0.body(b).name)
        spec.memory = 64 * 1024 * 1024        # ~2000 convex parts make many contact candidates
        self.m = spec.compile()
        self.m.geom_margin[:] = REPORT_MM / 1000
        self.d = mujoco.MjData(self.m)
        self.cal = calibration
        self.qadr = {j: self.m.jnt_qposadr[self.m.joint(j).id] for j in config.JOINTS}
        self.dadr = {j: self.m.jnt_dofadr[self.m.joint(j).id] for j in config.JOINTS}
        self.table = self.m.geom("table").id
        par = self.m.body_parentid
        self._skip = {(a, b) for a in range(self.m.nbody) for b in range(self.m.nbody)
                      if a == b or par[a] == b or par[b] == a}

    def rad(self, joint: str, tick: float) -> float:
        c = self.cal[joint]
        if joint == "gripper":          # 0 rad-ish at the closed end, as in so101_new_calib
            return float(np.radians(-10 + (tick - c["range_min"]) * 360 / 4095))
        return float(np.radians((tick - (c["range_min"] + c["range_max"]) / 2) * 360 / 4095))

    def _set(self, pose: dict[str, float]) -> None:
        self.d.qpos[:] = 0
        for j, t in pose.items():
            self.d.qpos[self.qadr[j]] = self.rad(j, t) + self.offsets.get(j, 0.0)
        self.mj.mj_forward(self.m, self.d)

    def name(self, g: int) -> str:
        return "table" if g == self.table else self.m.body(self.m.geom_bodyid[g]).name

    def distances(self, pose: dict[str, float]) -> dict[tuple[str, str], float]:
        """Closest distance (mm, negative = overlapping) per pair of non-neighbouring links, for
        every pair closer than REPORT_MM."""
        self._set(pose)
        out: dict[tuple[str, str], float] = {}
        for c in self.d.contact[:self.d.ncon]:
            b1, b2 = self.m.geom_bodyid[c.geom1], self.m.geom_bodyid[c.geom2]
            if (b1, b2) in self._skip:
                continue
            key = tuple(sorted((self.name(c.geom1), self.name(c.geom2))))
            out[key] = min(out.get(key, 1e9), c.dist * 1000)
        return out

    def clearance(self, pose: dict[str, float]) -> float:
        """Smallest distance in mm between any two non-neighbouring links (or a link and the
        table); REPORT_MM when nothing is that close."""
        dist = self.distances(pose)
        return min(dist.values()) if dist else REPORT_MM

    def gravity(self, pose: dict[str, float]) -> dict[str, float]:
        """Gravity torque per joint in N.m at this pose."""
        self._set(pose)
        return {j: float(self.d.qfrc_bias[self.dadr[j]]) for j in config.JOINTS}
