"""Gravity torque on each SO-101 joint along its homing sweep, from the MJCF link masses.

    python scripts/gravity_torque.py [path/to/so101_new_calib.xml]

Each joint is swept over its homing window (calibrated range + widen) with every other joint
held at REST, the folded pose the arm homes from (read off `touchstone probe`, 2026-11-03).
Angle 0 = middle of the calibrated range, lerobot's DEGREES formula, the same zero as
so101_new_calib.xml (checked against the real follower in B.L.U.S.H sim/arm_map.py).
"worst pose anywhere" = 4000 random poses, for reference only. The numbers behind
config.HOMING_TORQUE_LIMIT. Needs mujoco (the lerobot env has it).
"""
import json, pathlib, sys, numpy as np, mujoco
XML = pathlib.Path(sys.argv[1]) if len(sys.argv) > 1 else \
    pathlib.Path.home()/"Desktop/personal_projects/github/vinea-so101/B.L.U.S.H/sim/assets/so101/so101_new_calib.xml"
CAL = json.load(open(pathlib.Path.home()/".cache/huggingface/lerobot/calibration/robots/so_follower/follower_white.json"))
J = ["shoulder_pan","shoulder_lift","elbow_flex","wrist_flex","wrist_roll","gripper"]
NM_PER_LOAD = 2.94/1000
m = mujoco.MjModel.from_xml_path(str(XML)); d = mujoco.MjData(m)
print("total moving mass (kg):", round(sum(m.body_mass[1:]),3), " bodies:", [(m.body(i).name, round(m.body_mass[i],3)) for i in range(1,m.nbody)])
qadr = {j: m.jnt_qposadr[m.joint(j).id] for j in J}; dadr = {j: m.jnt_dofadr[m.joint(j).id] for j in J}
mid = {j:(CAL[j]["range_min"]+CAL[j]["range_max"])/2 for j in J}
def rad(j,t):
    if j=="gripper": return np.radians(-10 + (t-CAL[j]["range_min"])*360/4095)
    return np.radians((t-mid[j])*360/4095)
def grav(pose):            # pose: joint -> ticks ; returns joint -> N.m (gravity only)
    d.qpos[:] = 0; d.qvel[:] = 0
    for j,t in pose.items(): d.qpos[qadr[j]] = rad(j,t)
    mujoco.mj_forward(m,d)
    return {j: d.qfrc_bias[dadr[j]] for j in J}
REST = {"shoulder_pan":2031,"shoulder_lift":791,"elbow_flex":3133,"wrist_flex":2930,"wrist_roll":3020,"gripper":2367}
WIDEN = {"wrist_roll":120,"shoulder_pan":150}
print("\nrest pose gravity (load /1000):", {j: round(abs(v)/NM_PER_LOAD) for j,v in grav(REST).items()})
out = {}
for j in J:
    w = WIDEN.get(j,100); ts = np.arange(CAL[j]["range_min"]-w, CAL[j]["range_max"]+w+1, 5)
    g = np.array([grav({**REST, j:t})[j] for t in ts])
    # worst case anywhere: random poses of the joints beyond it (and this joint)
    rng = np.random.default_rng(0); worst = 0
    for _ in range(4000):
        p = {k: rng.uniform(CAL[k]["range_min"], CAL[k]["range_max"]) for k in J}
        worst = max(worst, abs(grav(p)[j]))
    i = int(np.argmax(abs(g)))
    out[j] = dict(sweep_max_nm=float(abs(g).max()), at_tick=int(ts[i]), at_start_nm=float(abs(grav(REST)[j])), worst_nm=worst)
    print(f"{j:<14} homing sweep: max {abs(g).max():.3f} N.m = {abs(g).max()/NM_PER_LOAD:5.0f}/1000 at tick {ts[i]}"
          f" | at rest {abs(grav(REST)[j])/NM_PER_LOAD:4.0f} | worst pose anywhere {worst:.3f} N.m = {worst/NM_PER_LOAD:4.0f}/1000")
    # profile every ~300 ticks
    print("     profile:", " ".join(f"{t}:{abs(v)/NM_PER_LOAD:.0f}" for t,v in list(zip(ts,g))[::60]))
