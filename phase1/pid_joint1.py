"""
Phase 1A — Single joint PID controller with gravity feedforward
τ_total = τ_PID + τ_gravity

τ_PID    = Kp*e + Ki*∫e + Kd*ė  (tracks the target angle)
τ_gravity = g(q)                 (cancels gravity so PID only handles error)
"""
import time
import numpy as np
import pybullet as p
import pybullet_data
import os


# ─── PID Controller ────────────────────────────────────────────────
class PIDController:
    def __init__(self, kp, ki, kd, dt):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integral   = 0.0
        self.prev_error = 0.0

    def reset(self):
        self.integral   = 0.0
        self.prev_error = 0.0

    def compute(self, setpoint, measurement):
        error           = setpoint - measurement
        self.integral  += error * self.dt
        derivative      = (error - self.prev_error) / self.dt
        self.prev_error = error
        return (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)


# ─── Simulation setup ──────────────────────────────────────────────
DT       = 1 / 240
DURATION = 5.0
STEPS    = int(DURATION / DT)

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setTimeStep(DT)

plane = p.loadURDF("plane.urdf")
robot = p.loadURDF("a1/a1.urdf",
                   basePosition=[0, 0, 0.42],
                   useFixedBase=False)
n_joints = p.getNumJoints(robot)

JOINT_INDEX  = 3      # FR_upper_joint
TARGET_ANGLE = 0.7    # radians (~40 degrees)

# Disable PyBullet's default velocity motors on all joints
for i in range(n_joints):
    p.setJointMotorControl2(robot, i,
                            controlMode=p.VELOCITY_CONTROL,
                            force=0)

# ─── Gravity feedforward calibration ──────────────────────────────
# Snap robot to a clean resting state with all joints at zero
p.resetBasePositionAndOrientation(robot,
                                  [0, 0, 0.42],
                                  p.getQuaternionFromEuler([0, 0, 0]))
p.resetBaseVelocity(robot, [0, 0, 0], [0, 0, 0])
for i in range(n_joints):
    p.resetJointState(robot, i, 0.0, 0.0)

# Hold the controlled joint at target using position control and let it settle
p.setJointMotorControl2(robot, JOINT_INDEX,
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=TARGET_ANGLE,
                        force=500)
for _ in range(500):
    p.stepSimulation()

# Read the torque the position controller applied to hold against gravity
state       = p.getJointState(robot, JOINT_INDEX)
tau_gravity = state[3]   # appliedJointMotorTorque = g(q) at this pose
print(f"Calibrated gravity torque: {tau_gravity:.4f} Nm")

# Reset everything back to zero for the real simulation
p.resetBasePositionAndOrientation(robot,
                                  [0, 0, 0.42],
                                  p.getQuaternionFromEuler([0, 0, 0]))
p.resetBaseVelocity(robot, [0, 0, 0], [0, 0, 0])
for i in range(n_joints):
    p.resetJointState(robot, i, 0.0, 0.0)

# Disable all motors again so only our torque commands act
for i in range(n_joints):
    p.setJointMotorControl2(robot, i,
                            controlMode=p.VELOCITY_CONTROL,
                            force=0)

# ─── PID controller ────────────────────────────────────────────────
pid = PIDController(kp=8.0, ki=0.5, kd=3.0, dt=DT)

# ─── Data recording ────────────────────────────────────────────────
times   = []
angles  = []
targets = []
torques = []

# ─── Main control loop ─────────────────────────────────────────────
print(f"\nRunning PID on joint {JOINT_INDEX} (FR_upper_joint)")
print(f"Target : {TARGET_ANGLE} rad ({np.degrees(TARGET_ANGLE):.1f} deg)")
print(f"Gains  : Kp={pid.kp}  Ki={pid.ki}  Kd={pid.kd}\n")

for step in range(STEPS):
    # 1. Read current joint state
    joint_state   = p.getJointState(robot, JOINT_INDEX)
    current_angle = joint_state[0]
    current_vel   = joint_state[1]

    # 2. PID correction (tracks error only — gravity handled separately)
    tau_pid = pid.compute(TARGET_ANGLE, current_angle)

    # 3. Total torque = PID + feedforward
    #    Subtract tau_gravity because state[3] sign convention is opposite
    #    to torque control sign convention in PyBullet
    tau_total = tau_pid - tau_gravity

    # 4. Apply torque to controlled joint
    p.setJointMotorControl2(robot, JOINT_INDEX,
                            controlMode=p.TORQUE_CONTROL,
                            force=tau_total)

    # 5. Step simulation
    p.stepSimulation()
    time.sleep(DT * 4)

    # 6. Record
    t = step * DT
    times.append(t)
    angles.append(current_angle)
    targets.append(TARGET_ANGLE)
    torques.append(tau_total)

    if step % 120 == 0:
        print(f"  t={t:.1f}s  angle={current_angle:+.4f} rad  "
              f"error={TARGET_ANGLE - current_angle:+.4f}  "
              f"tau_pid={tau_pid:+.3f}  tau_grav={tau_gravity:+.3f}  "
              f"tau_total={tau_total:+.3f} Nm")

# ─── Save data ─────────────────────────────────────────────────────
np.save('phase1/pid_data.npy', {
    'times':       np.array(times),
    'angles':      np.array(angles),
    'targets':     np.array(targets),
    'torques':     np.array(torques),
    'kp':          pid.kp,
    'ki':          pid.ki,
    'kd':          pid.kd,
    'tau_gravity': tau_gravity
})
print("\nData saved. Run: python3 phase1/plot_pid.py")

os._exit(0)