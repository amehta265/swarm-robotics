"""
Phase 3 — Shared physics server.

This process owns the simulated world. Walnut and Hazel connect to it
as clients via PyBullet's shared memory mechanism.

IMPORTANT: This is a simulation artifact. In a real deployment there
is no physics server — each robot has its own sensors and actuators.
The shared memory here is ONLY for the physics simulation, not for
the swarm communication. Swarm comms happens via UDP only.

Start order:
  Terminal 1: python3 phase3/physics_server.py   (wait for "Server ready")
  Terminal 2: python3 phase3/walnut.py
  Terminal 3: python3 phase3/hazel.py
"""
import time
import math
import pybullet as p
import pybullet_data
import os

p.connect(p.GUI_SERVER)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setTimeStep(1/240)

p.loadURDF("plane.urdf")

# Walnut — body ID will be 1 (0 is the plane)
walnut = p.loadURDF("a1/a1.urdf",
                    basePosition=[-1.5, 0, 0.35],
                    useFixedBase=True)

# Hazel — body ID will be 2
hazel = p.loadURDF("a1/a1.urdf",
                   basePosition=[1.5, 0, 0.35],
                   useFixedBase=True)

# Target beacon — body ID will be 3
target_col = p.createCollisionShape(p.GEOM_SPHERE, radius=0.25)
target_vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.25,
                                 rgbaColor=[1, 0, 0, 1])
target = p.createMultiBody(baseMass=0,
                           baseCollisionShapeIndex=target_col,
                           baseVisualShapeIndex=target_vis,
                           basePosition=[0, 1.5, 0.3])

# Arena walls — two barriers to make sensor readings interesting
for y_pos in [2.5, -2.5]:
    wc = p.createCollisionShape(p.GEOM_BOX, halfExtents=[2.5, 0.05, 0.5])
    wv = p.createVisualShape(p.GEOM_BOX, halfExtents=[2.5, 0.05, 0.5],
                             rgbaColor=[0.6, 0.6, 0.6, 1])
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wc,
                      baseVisualShapeIndex=wv,
                      basePosition=[0, y_pos, 0.5])

print("=== Physics Server Ready ===")
print(f"  Plane  body ID : 0")
print(f"  Walnut body ID : {walnut}")
print(f"  Hazel  body ID : {hazel}")
print(f"  Target body ID : {target}")
print(f"\nNow start walnut.py and hazel.py in separate terminals.")
print(f"Ctrl+C here to shut down the world.\n")

# Server owns target motion — slow orbit around arena center
t = 0.0
DT = 1/240
try:
    while True:
        tx = 2.0 * math.cos(0.2 * t)
        ty = 2.0 * math.sin(0.2 * t)
        p.resetBasePositionAndOrientation(target, [tx, ty, 0.3], [0,0,0,1])
        p.stepSimulation()
        time.sleep(DT)
        t += DT
except KeyboardInterrupt:
    print("Physics server shutting down.")
    os._exit(0)