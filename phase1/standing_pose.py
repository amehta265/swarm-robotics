"""
Phase 1B — Standing pose with useFixedBase=True.

The base is pinned to the world so we can focus entirely on joint coordination
without the lateral balance problem. This is standard practice when learning
gait generation — Boston Dynamics uses tethered rigs for exactly this reason.

What you learn here transfers directly to Phase 1C (gait generator):
- 12 joints running simultaneously with individual PD controllers
- Coordinated target angles producing a desired body configuration
- The cascade: desired pose → joint targets → PD torques → motion
"""
import time
import numpy as np
import pybullet as p
import pybullet_data
import os

# ─── Simulation setup ──────────────────────────────────────────────
DT       = 1 / 240
DURATION = 6.0
STEPS    = int(DURATION / DT)

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setTimeStep(DT)

plane = p.loadURDF("plane.urdf")
robot = p.loadURDF("a1/a1.urdf",
                   basePosition=[0, 0, 0.35],
                   useFixedBase=True)       # base pinned — focus on joints
n_joints = p.getNumJoints(robot)

# ─── Revolute joints only ──────────────────────────────────────────
revolute_indices = [i for i in range(n_joints)
                    if p.getJointInfo(robot, i)[2] != 4]
print(f"Controlling {len(revolute_indices)} revolute joints")

# ─── Standing pose target angles ───────────────────────────────────
standing_angles = {
    1:  0.0,   3:  0.67,  4: -1.3,
    6:  0.0,   8:  0.67,  9: -1.3,
    11: 0.0,  13:  0.67, 14: -1.3,
    16: 0.0,  18:  0.67, 19: -1.3,
}

# Pre-set joints to standing angles before simulation starts
for idx, angle in standing_angles.items():
    p.resetJointState(robot, idx, angle, 0.0)

# ─── PD gains ──────────────────────────────────────────────────────
KP        = 20.0
KD        = 2.0
MAX_FORCE = 33.0

# ─── Data recording ────────────────────────────────────────────────
times   = []
angles  = []
base_zs = []

# ─── Main control loop ─────────────────────────────────────────────
print(f"Kp={KP}  Kd={KD}  maxForce={MAX_FORCE}\n")

for step in range(STEPS):
    t = step * DT

    for idx in revolute_indices:
        target = standing_angles.get(idx, 0.0)
        p.setJointMotorControl2(
            robot, idx,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target,
            positionGain=KP,
            velocityGain=KD,
            force=MAX_FORCE
        )

    p.stepSimulation()
    time.sleep(DT * 2)

    pos, orn = p.getBasePositionAndOrientation(robot)
    fr_upper = p.getJointState(robot, 3)[0]
    times.append(t)
    angles.append(fr_upper)
    base_zs.append(pos[2])

    if step % 240 == 0:
        print(f"  t={t:.1f}s  FR_upper={np.degrees(fr_upper):.1f}deg  "
              f"target={np.degrees(standing_angles[3]):.1f}deg  "
              f"error={np.degrees(standing_angles[3] - fr_upper):.1f}deg")

# ─── Save ──────────────────────────────────────────────────────────
np.save('phase1/standing_data.npy', {
    'times':   np.array(times),
    'angles':  np.array(angles),
    'base_zs': np.array(base_zs),
    'target':  standing_angles[3]
})
print("\nData saved. Run: python3 phase1/plot_standing.py")
os._exit(0)