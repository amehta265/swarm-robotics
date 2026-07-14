import time
import pybullet as p
import pybullet_data
import numpy as np

# p.connect to the physics engine
# p.GUI for a graphical user interface, p.DIRECT is headless mode
client = p.connect(p.GUI, options="--background_color_red=0.8 --background_color_green=0.9 --background_color_blue=1.0")

# Tell PyBullet where to find its built-in assets (plane.urdf, a1.urdf etc.)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Set gravity to Earth's value in the Z direction (downward = negative Z)
p.setGravity(0, 0, -9.81)

# Set how much simulated time passes per step.
# 1/240 seconds = 240Hz. This is PyBullet's default and a good starting point.
p.setTimeStep(1/240)

# Load the ground plane — a flat infinite surface the robot can stand on
plane = p.loadURDF("plane.urdf")

print("World loaded.")
print(f"PyBullet data path: {pybullet_data.getDataPath()}")


# Load the A1 quadruped URDF.
# basePosition sets where it starts — 0.5m above ground so it falls and lands.
# useFixedBase=False means it's free to move (not bolted to the world).
start_pos = [0, 0, 0.5]
start_orn = p.getQuaternionFromEuler([0, 0, 0])  # no initial rotation

robot = p.loadURDF("a1/a1.urdf",
                   start_pos,
                   start_orn,
                   useFixedBase=False)

# Count how many joints the robot has
n_joints = p.getNumJoints(robot)
print(f"\nA1 quadruped loaded. Total joints: {n_joints}")
print("-" * 60)

# Loop through every joint and print its properties
for i in range(n_joints):
    info = p.getJointInfo(robot, i)

    # info is a tuple — these are the fields we care about:
    joint_index = info[0]
    joint_name  = info[1].decode()   # bytes → string
    joint_type  = info[2]            # 0=revolute, 1=prismatic, 4=fixed
    joint_axis  = info[13]           # (x,y,z) vector the joint rotates around

    type_name = {0: "revolute", 1: "prismatic", 4: "fixed"}.get(joint_type, "unknown")

    print(f"  [{joint_index:02d}] {joint_name:<30} type={type_name:<10} axis={joint_axis}")


# Run 2 seconds of simulation (480 steps × 1/240s = 2s)
# time.sleep keeps it at real-time speed so you can watch
print("\nSimulating 2 seconds under gravity — watch the robot fall and land...")
for step in range(480):
    p.stepSimulation()
    time.sleep(1/240)

# --- Read the full state vector after landing ---

# Where is the robot body in the world?
pos, orn = p.getBasePositionAndOrientation(robot)
lin_vel, ang_vel = p.getBaseVelocity(robot)

print("\n=== Base state after 2s ===")
print(f"  Position (x,y,z)        : {np.round(pos, 4)}")
print(f"  Orientation (quaternion) : {np.round(orn, 4)}")
print(f"  Linear velocity          : {np.round(lin_vel, 4)}")
print(f"  Angular velocity         : {np.round(ang_vel, 4)}")

# --- Joint states ---
print("\n=== Joint states after 2s ===")
print(f"  {'Index':<6} {'Name':<30} {'Angle (rad)':>12} {'Velocity (rad/s)':>18}")
print("  " + "-" * 68)

for i in range(n_joints):
    info = p.getJointInfo(robot, i)
    if info[2] == 4:       # skip fixed joints — they never move
        continue
    state = p.getJointState(robot, i)
    angle    = state[0]    # current joint angle in radians
    velocity = state[1]    # current angular velocity in rad/s
    print(f"  [{i:02d}]  {info[1].decode():<30} {angle:>+12.4f} {velocity:>18.4f}")

# Keep window open for inspection
print("\nWindow is open — orbit with mouse. Close window or Ctrl+C to exit.")
while p.isConnected():
    p.stepSimulation()
    time.sleep(1/240)
