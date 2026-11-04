"""Build touchstone's self-collision model of the SO-101: every link mesh split into convex parts.

    PYTHONPATH=<dir with coacd + trimesh> python scripts/decompose_so101.py [so101_new_calib.xml]

MuJoCo collides each mesh as its convex HULL, so a concave link (the U-shaped upper arm, the
jaw) "collides" where the real part has a gap, and the folded rest pose looks like a crash.
CoACD splits each mesh into convex parts that follow the real shape; MuJoCo then collides part
against part. Run once, offline; the output is committed, so homing needs only mujoco.

Input: TheRobotStudio/SO-ARM100 Simulation/SO101 (Apache-2.0), as fetched for B.L.U.S.H.
Output: touchstone/model/so101/ (collision.xml + parts/*.stl + NOTICE.md)
"""
import pathlib, sys, shutil
import numpy as np, mujoco, trimesh, coacd

SRC = pathlib.Path(sys.argv[1]) if len(sys.argv) > 1 else \
    pathlib.Path.home() / "Desktop/personal_projects/github/vinea-so101/B.L.U.S.H/sim/assets/so101/so101_new_calib.xml"
OUT = pathlib.Path(__file__).resolve().parents[1] / "touchstone/model/so101"
THRESHOLD = 0.03          # CoACD concavity threshold: smaller = more, tighter parts
BASE_MESHES = {"base_so101_v2", "base_motor_holder_so101_v1", "sts3215_03a_v1", "waveshare_mounting_plate_so101_v2"}

spec = mujoco.MjSpec.from_file(str(SRC))
meshdir = SRC.parent / "assets"
(OUT / "parts").mkdir(parents=True, exist_ok=True)     # parts already split are reused (slow step)

done: dict[str, list[str]] = {}
def parts_for(mesh_name, mesh_file):
    if mesh_name in done:
        return done[mesh_name]
    import re
    cached = sorted((c for c in (OUT / "parts").glob(f"{mesh_name}_p*.stl")
                     if re.fullmatch(rf"{re.escape(mesh_name)}_p\d+", c.stem)), key=lambda c: int(c.stem.rsplit("_p", 1)[1]))
    if cached:
        done[mesh_name] = [c.stem for c in cached]
        print(f"{mesh_name:<36} {len(cached):>3} parts   (cached)")
        return done[mesh_name]
    tm = trimesh.load(meshdir / mesh_file, force="mesh")
    res = coacd.run_coacd(coacd.Mesh(tm.vertices, tm.faces), threshold=THRESHOLD, max_convex_hull=-1)
    names, vol = [], 0.0
    for k, (v, f) in enumerate(res):
        p = trimesh.Trimesh(v, f).convex_hull
        vol += p.volume
        n = f"{mesh_name}_p{k:03d}"
        p.export(OUT / "parts" / f"{n}.stl")
        names.append(n)
    ratio = vol / tm.volume if tm.is_volume else float("nan")
    print(f"{mesh_name:<36} {len(names):>3} parts   parts/mesh volume {ratio:5.2f}   watertight {tm.is_watertight}")
    done[mesh_name] = names
    return names

mesh_file = {(m.name or pathlib.Path(m.file).stem): m.file for m in spec.meshes}
new = mujoco.MjSpec()
new.compiler.degree = False
new.compiler.meshdir = str(OUT / "parts")        # absolute to compile; written relative below
new.option.gravity = spec.option.gravity
# rebuild: same bodies, joints and inertia; collision geoms = convex parts; no visuals
def copy_body(src, dst_parent):
    b = dst_parent.add_body(name=src.name, pos=src.pos, quat=src.quat)
    b.mass, b.ipos, b.iquat, b.inertia, b.fullinertia = src.mass, src.ipos, src.iquat, src.inertia, src.fullinertia
    b.explicitinertial = True
    for j in src.joints:
        b.add_joint(name=j.name, type=j.type, axis=j.axis, range=j.range, limited=mujoco.mjtLimited.mjLIMITED_FALSE)
    for g in src.geoms:
        if g.type != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        is_collision = g.contype != 0 or g.conaffinity != 0
        if not is_collision and not (src.name == "base" and g.meshname in BASE_MESHES):
            continue
        for n in parts_for(g.meshname, mesh_file[g.meshname]):
            if new.mesh(n) is None:
                new.add_mesh(name=n, file=f"{n}.stl")
            new_g = b.add_geom(type=mujoco.mjtGeom.mjGEOM_MESH, meshname=n, pos=g.pos, quat=g.quat,
                               contype=1, conaffinity=1, group=3)
            new_g.name = f"{src.name}:{n}:{len(b.geoms)}"
    for c in src.bodies:
        copy_body(c, b)
for top in spec.worldbody.bodies:
    copy_body(top, new.worldbody)
new.compile()
(OUT / "collision.xml").write_text(new.to_xml().replace(str(OUT / "parts"), "parts"))
(OUT / "NOTICE.md").write_text(
    "# Third-party files\n\nDerived from TheRobotStudio/SO-ARM100 (https://github.com/TheRobotStudio/SO-ARM100),\n"
    "`Simulation/SO101/so101_new_calib.xml` and its meshes, Apache License 2.0\n"
    "(https://www.apache.org/licenses/LICENSE-2.0). Changed: each link mesh split into convex parts\n"
    f"with CoACD (threshold {THRESHOLD}) by `scripts/decompose_so101.py`; visual meshes dropped; the base's\n"
    "meshes added as colliders. Bodies, joints and inertia unchanged.\n")
print("wrote", OUT, sum(1 for _ in (OUT / "parts").iterdir()), "parts")
