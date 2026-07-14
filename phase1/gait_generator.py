"""
Phase 1C — Trot gait generator.

The only change from Phase 1B: target angles are now functions of time.
The PD controller loop is identical — it still tracks whatever target we give it.

A trot is defined by two diagonal pairs moving in anti-phase:
  Pair A: FR (front-right) + RL (rear-left)  — phase offset = 0
  Pair B: FL (front-left)  + RR (rear-right) — phase offset = π

Each leg cycles through:
  STANCE phase (foot on ground): hip/knee hold a support angle
  SWING  phase (foot in air):    hip extends forward, knee lifts

The foot trajectory during swing is a half-sine arc in the hip flexion joint.
This is the simplest possible gait generator — no inverse kinematics, no foot
planning, just joint-space sinusoids with phase offsets.

Concept: walking = stance/swing alternation + phase coordination across legs.
The same pattern scaled up is how all legged robots walk.
"""
import time
import numpy as np
import pybullet as p
import pybullet_data
import os


def gait_targets(t, freq=1.0, swing_amp=0.3, lift_amp=0.2):
    """
    Compute target joint angles for all 12 joints at time t.

    Parameters:
        t         — current simulation time (seconds)
        freq      — gait frequency in Hz (cycles per second)
        swing_amp — amplitude of hip flexion swing (radians)
        lift_amp  — amplitude of knee lift during swing (radians)

    Returns:
        dict mapping joint_index -> target_angle (radians)

    Gait logic:
        phase = 2π * freq * t  (full cycle angle, increases with time)
        Pair A (FR+RL): uses phase directly
        Pair B (FL+RR): uses phase + π (half cycle offset = anti-phase)

        Hip flexion target  = standing_angle + swing_amp * sin(phase)
        Knee target during swing = standing_angle + lift_amp * |sin(phase)|
        Hip abduction: constant (not used for swing)
    """
    phase_A = 2 * np.pi * freq * t           # FR + RL
    phase_B = 2 * np.pi * freq * t + np.pi   # FL + RR (anti-phase)

    # Standing baseline angles
    hip_abduct  =  0.0
    hip_flex    =  0.67
    knee        = -1.3

    def leg_targets(phase):
        """Return (hip_abduct, hip_flex, knee) targets for a leg at this phase."""
        h_abduct = hip_abduct
        h_flex   = hip_flex + swing_amp * np.sin(phase)
        # Knee lifts during swing (positive half of sine = swing phase)
        # During stance (negative half) knee holds baseline
        k = knee + lift_amp * max(0.0, np.sin(phase))
        return h_abduct, h_flex, k

    fr_ab, fr_hf, fr_k = leg_targets(phase_A)   # FR — pair A
    rl_ab, rl_hf, rl_k = leg_targets(phase_A)   # RL — pair A (same phase)
    fl_ab, fl_hf, fl_k = leg_targets(phase_B)   # FL — pair B
    rr_ab, rr_hf, rr_k = leg_targets(phase_B)   # RR — pair B (same phase)

    return {
        1:  fr_ab,  3:  fr_hf,  4:  fr_k,   # FR
        6:  fl_ab,  8:  fl_hf,  9:  fl_k,   # FL
        11: rr_ab, 13:  rr_hf, 14:  rr_k,   # RR
        16: rl_ab, 18:  rl_hf, 19:  rl_k,   # RL
    }


# ─── Simulation setup ──────────────────────────────────────────────
DT       = 1 / 240
DURATION = 8.0
STEPS    = int(DURATION / DT)

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setTimeStep(DT)

plane = p.loadURDF("plane.urdf")
robot = p.loadURDF("a1/a1.urdf",
                   basePosition=[0, 0, 0.35],
                   useFixedBase=True)
n_joints = p.getNumJoints(robot)

revolute_indices = [i for i in range(n_joints)
                    if p.getJointInfo(robot, i)[2] != 4]

# Pre-set to standing pose before gait starts
standing = gait_targets(0.0)
for idx, angle in standing.items():
    p.resetJointState(robot, idx, angle, 0.0)

# ─── PD gains (same as Phase 1B) ───────────────────────────────────
KP        = 20.0
KD        = 2.0
MAX_FORCE = 33.0

# ─── Gait parameters ───────────────────────────────────────────────
FREQ      = 1.0    # 1 Hz — one full gait cycle per second
SWING_AMP = 0.3    # hip swing amplitude in radians (~17 degrees)
LIFT_AMP  = 0.2    # knee lift amplitude in radians (~11 degrees)

# ─── Data recording ────────────────────────────────────────────────
times      = []
fr_upper_a = []   # FR hip flexion actual
fl_upper_a = []   # FL hip flexion actual (should be anti-phase to FR)
fr_upper_t = []   # FR hip flexion target

# ─── Main control loop ─────────────────────────────────────────────
print(f"Running trot gait — freq={FREQ}Hz  swing={SWING_AMP}rad  lift={LIFT_AMP}rad")
print(f"Kp={KP}  Kd={KD}  maxForce={MAX_FORCE}\n")

for step in range(STEPS):
    t = step * DT

    # Compute target angles from gait generator
    targets = gait_targets(t, freq=FREQ,
                           swing_amp=SWING_AMP,
                           lift_amp=LIFT_AMP)

    # Apply PD control to each joint — identical to Phase 1B
    for idx in revolute_indices:
        target = targets.get(idx, 0.0)
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

    # Record FR and FL hip flexion to verify anti-phase relationship
    fr = p.getJointState(robot, 3)[0]
    fl = p.getJointState(robot, 8)[0]
    times.append(t)
    fr_upper_a.append(fr)
    fl_upper_a.append(fl)
    fr_upper_t.append(targets[3])

    if step % 240 == 0:
        print(f"  t={t:.1f}s  FR_hip={np.degrees(fr):.1f}deg  "
              f"FL_hip={np.degrees(fl):.1f}deg  "
              f"(should be anti-phase)")

# ─── Save ──────────────────────────────────────────────────────────
np.save('phase1/gait_data.npy', {
    'times':      np.array(times),
    'fr_upper_a': np.array(fr_upper_a),
    'fl_upper_a': np.array(fl_upper_a),
    'fr_upper_t': np.array(fr_upper_t),
})
print("\nData saved. Run: python3 phase1/plot_gait.py")
os._exit(0)