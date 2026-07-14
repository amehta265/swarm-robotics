"""
Phase 1A — Single joint PID controller
We manually compute torque each timestep: τ = Kp*e + Ki*∫e + Kd*ė
ANKIT: The above formula also comes from PID i.e. Proportional - Integral - Derivative Controller
and apply it to one joint. This is the inner control loop for everything
in Phase 1B (standing) and 1C (gait).
"""
import time
import numpy as np
import pybullet as p
import pybullet_data
import matplotlib.pyplot as plt
import os


class PIDController:
    def __init__(self, kp, ki, kd, dt):
        """
        kp — proportional gain: how hard to push toward target
        ki — integral gain: eliminates steady-state error over time
        kd — derivative gain: damps oscillation (velocity braking)
        dt — timestep in seconds (must match simulation timestep)
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt

        # Internal state — these accumulate over time
        self.integral   = 0.0   # running sum of error * dt
        self.prev_error = 0.0   # error from last timestep (for derivative)

    def reset(self):
        """Call this whenever you change the setpoint to avoid
        integral windup carrying over from a previous target."""
        self.integral   = 0.0
        self.prev_error = 0.0

    def compute(self, setpoint, measurement):
        """
        Given desired angle (setpoint) and actual angle (measurement),
        return the torque τ to apply to the joint this timestep.
        """
        """
        Ankit: This comes from the definition of PID controller. Error state is determined by the difference between
        the desired setpoint and the measured process variable. And then a corrective mechanism is put in place to make
        the system stable and accurate. Here the corrective mechanism is the torque needed to alter the leg.
        """
        # Error: how far are we from the target?
        error = setpoint - measurement

        # Integral: accumulate error over time
        # This is a discrete approximation of ∫e dt
        self.integral += error * self.dt

        # Derivative: rate of change of error
        # Negative velocity = how fast we're moving toward target
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error

        # PID output: the torque command
        tau = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        # Second term i.e. the integral term is basically - τ_integral = Ki * ∫e dt  ≈  Ki * (sum of all past errors * dt)

        return tau
    
"""
PHASE 1A - Part 2
"""
# ─── Simulation setup ──────────────────────────────────────────────
DT = 1/240          # simulation timestep — must match p.setTimeStep
DURATION = 5.0      # seconds to simulate
STEPS    = int(DURATION / DT)

# Connect and configure world (same as Phase 0)
p.connect(p.GUI)
# Second isolated physics client for inverse dynamics only.
# DIRECT = no GUI, completely separate from the main simulation.
client_dyn = p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81) #x, y, z
p.setTimeStep(DT)

plane = p.loadURDF("plane.urdf")
robot = p.loadURDF("a1/a1.urdf",
                   basePosition=[0, 0, 0.42],
                   useFixedBase=False)

# Fixed-base dynamics robot in the isolated client
p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client_dyn)
p.setGravity(0, 0, -9.81, physicsClientId=client_dyn)
robot_dyn = p.loadURDF("a1/a1.urdf",
                        basePosition=[0, 0, 0.42],
                        useFixedBase=True,
                        physicsClientId=client_dyn)
n_joints_dyn = p.getNumJoints(robot_dyn, physicsClientId=client_dyn)

# ─── Which joint to control ────────────────────────────────────────
# We'll control FR_upper_joint (index 3) — the front-right hip flexion.
# This is a Y-axis revolute joint: positive angle tilts the leg forward.
JOINT_INDEX  = 3
TARGET_ANGLE = 0.7   # radians (~40 degrees forward)

# Disable PyBullet's default velocity motor on every joint.
# By default PyBullet applies a damping motor that resists motion.
# We turn it off so OUR PID torque is the only thing acting.
n_joints = p.getNumJoints(robot)
for i in range(n_joints):
    p.setJointMotorControl2(
        robot, i,
        controlMode=p.VELOCITY_CONTROL,
        force=0    # zero force = motor disabled
    )

# ─── Create our PID controller ─────────────────────────────────────
# Start with these gains — we'll tune them after the first run
pid = PIDController(kp=8.0, ki=0.5, kd=3.0, dt=DT)

# calculateInverseDynamics only accepts movable (non-fixed) joints.
# We must filter out fixed joints (type 4) when building the q vector,
# but keep track of the original indices so we can look up the right torque.
# movable_indices = [i for i in range(n_joints)
#                 if p.getJointInfo(robot, i)[2] != 4]

"""
PHASE 1A - Part 3
"""

# ─── Data recording (for plotting after) ──────────────────────────
times       = []
angles      = []
targets     = []
torques     = []

# ─── Main control loop ─────────────────────────────────────────────
print(f"Running PID on joint {JOINT_INDEX} (FR_upper_joint)")
print(f"Target angle: {TARGET_ANGLE} rad ({np.degrees(TARGET_ANGLE):.1f}°)")
print(f"Gains: Kp={pid.kp}  Ki={pid.ki}  Kd={pid.kd}\n")

for step in range(STEPS):
    # 1. Read current joint angle
    joint_state   = p.getJointState(robot, JOINT_INDEX)
    current_angle = joint_state[0]
    current_vel   = joint_state[1]

    # 2. Compute gravity feedforward for ALL joints
    # We need positions and velocities of every joint for the dynamics model.
    # calculateInverseDynamics() solves: M(q)q̈ + C(q,q̇)q̇ + g(q) = τ
    # With q̈=0 and q̇=0, it reduces to just: g(q) = τ
    # i.e. "what torque does each joint need to hold still against gravity?"

    # calculateInverseDynamics requires ALL joints (including fixed),
    # passing full position/velocity vectors matching total joint count
    # Sync fixed-base copy's joint positions to match the real robot
    # so the gravity computation reflects the actual configuration
   # Sync dynamics client joint positions to real robot
    all_joint_states = p.getJointStates(robot, range(n_joints))
    for i in range(n_joints):
        p.resetJointState(robot_dyn, i,
                          all_joint_states[i][0],
                          all_joint_states[i][1],
                          physicsClientId=client_dyn)

    q   = [all_joint_states[i][0] for i in range(n_joints)]
    qd  = [all_joint_states[i][1] for i in range(n_joints)]
    qdd = [0.0] * n_joints

    gravity_torques = p.calculateInverseDynamics(
        robot_dyn, q, qd, qdd,
        physicsClientId=client_dyn
    )
    tau_gravity = gravity_torques[JOINT_INDEX]

    # 3. Compute PID torque — now only responsible for tracking error,
    # not for fighting gravity
    tau_pid = pid.compute(TARGET_ANGLE, current_angle)

    # 4. Total torque = PID correction + gravity compensation
    tau_total = tau_pid + tau_gravity

    # 5. Apply total torque to the joint
    p.setJointMotorControl2(
        robot,
        JOINT_INDEX,
        controlMode=p.TORQUE_CONTROL,
        force=tau_total
    )
    """
    Here below is old way without gravity feed forward
    """
    # # 1. Read current joint angle
    # joint_state   = p.getJointState(robot, JOINT_INDEX)
    # current_angle = joint_state[0]   # radians / ANKIT: Position
    # current_vel   = joint_state[1]   # rad/s

    # # 2. Compute PID torque need to make the movement happen
    # tau = pid.compute(TARGET_ANGLE, current_angle)

    # # 3. Apply torque to the joint
    # p.setJointMotorControl2(
    #     robot,
    #     JOINT_INDEX,
    #     controlMode=p.TORQUE_CONTROL,
    #     force=tau
    # )

    # 4. Step the physics engine one timestep
    p.stepSimulation()
    time.sleep(DT * 4)   # real-time playback

    # 5. Record data
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

# p.disconnect()

# ANKIT: Matplotlib's GUI conflicts with PyBullet's OpenGL context on WSL when both try to manage the display simultaneously. 
# DONT call any matplotlib GUI functions (like plt.show()) until after p.disconnect() and the PyBullet window is closed.

# Save raw data to a file — plot script reads this separately
np.save('phase1/pid_data.npy', {
    'times':   np.array(times),
    'angles':  np.array(angles),
    'targets': np.array(targets),
    'torques': np.array(torques),
    'kp': pid.kp, 'ki': pid.ki, 'kd': pid.kd
})
print("\nData saved to phase1/pid_data.npy")
print("Now run: python3 phase1/plot_pid.py")

os._exit(0)  # force exit to avoid PyBullet segfault when matplotlib GUI opens

# ─── Plot results ──────────────────────────────────────────────────
# fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

# ax1.plot(times, np.degrees(angles),  label='actual angle',  color='steelblue')
# ax1.plot(times, np.degrees(targets), label='target angle',
#          color='orange', linestyle='--')
# ax1.set_ylabel('Joint angle (degrees)')
# ax1.set_title(f'FR_upper_joint PID  —  Kp={pid.kp}  Ki={pid.ki}  Kd={pid.kd}')
# ax1.legend()
# ax1.grid(True, alpha=0.3)

# ax2.plot(times, torques, label='torque τ (Nm)', color='firebrick')
# ax2.set_ylabel('Torque (Nm)')
# ax2.set_xlabel('Time (s)')
# ax2.legend()
# ax2.grid(True, alpha=0.3)

# plt.tight_layout()
# plt.savefig('phase1/pid_result.png', dpi=150)
# plt.close()   # close cleanly — no plt.show() to avoid WSL OpenGL conflict
# print("\nPlot saved to phase1/pid_result.png")
# print("Open it with: explorer.exe phase1/pid_result.png")
# plt.show() 
