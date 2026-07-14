"""
Phase 2 — IR sensor simulation via raycasting.

Concepts covered:
  - Raycasting: cast rays from sensor origin, get distance to first hit
  - Sensor frame transforms: rays defined in body frame, rotated to world frame
  - Field of view: sensors have limited angular range
  - Noise: real sensors aren't perfect — add Gaussian noise
  - Target detection: beacon on target returns a distinguishable signature
  - Visualization: debug lines show sensor rays in the GUI (green=clear, red=hit)

The key insight: a ray cast from point A in direction D with max range R
returns hit_fraction in [0,1]. Actual distance = hit_fraction * R.
If hit_fraction == 1.0, nothing was hit within range.

This same sensor model will be used by both robots in Phase 3.
"""
import time
import numpy as np
import pybullet as p
import pybullet_data
import os


# ══════════════════════════════════════════════════════════════════
# IR Sensor class
# ══════════════════════════════════════════════════════════════════
class IRSensor:
    """
    Simulates a single IR proximity sensor via a raycast.

    A real IR sensor emits a beam and measures how much bounces back.
    We simulate this as: cast a ray from sensor_origin in sensor_direction,
    return the distance to the first object hit.

    Parameters:
        robot_id    — PyBullet body ID the sensor is mounted on
        offset      — sensor position relative to robot base (x,y,z) in body frame
        direction   — sensor pointing direction in body frame (unit vector)
        max_range   — maximum sensing distance in meters
        noise_std   — standard deviation of Gaussian noise (meters)
        target_id   — body ID of the target object (for beacon detection)
    """
    def __init__(self, robot_id, offset, direction,
                 max_range=2.0, noise_std=0.02, target_id=None):
        self.robot_id  = robot_id
        self.offset    = np.array(offset)
        self.direction = np.array(direction) / np.linalg.norm(direction)
        self.max_range = max_range
        self.noise_std = noise_std
        self.target_id = target_id

        # Debug line ID — updated each step to show ray in GUI
        self._line_id  = None

    def read(self):
        """
        Cast a ray and return a SensorReading.

        Returns:
            distance    — measured distance in meters (with noise)
            hit_object  — PyBullet body ID of what was hit (-1 = nothing)
            is_target   — True if the hit object is the beacon target
            hit_pos     — (x,y,z) world position of hit point
        """
        # 1. Get robot base position and orientation in world frame
        base_pos, base_orn = p.getBasePositionAndOrientation(self.robot_id)
        base_pos = np.array(base_pos)

        # 2. Rotate sensor offset and direction from body frame to world frame
        #    p.multiplyTransforms handles the quaternion rotation for us
        sensor_world_pos, _ = p.multiplyTransforms(
            base_pos, base_orn,
            self.offset.tolist(), [0, 0, 0, 1]
        )
        sensor_world_pos = np.array(sensor_world_pos)

        # Rotate direction vector: apply robot orientation to body-frame direction
        # We do this by treating the direction as a position offset from origin
        rotated_dir, _ = p.multiplyTransforms(
            [0, 0, 0], base_orn,
            self.direction.tolist(), [0, 0, 0, 1]
        )
        rotated_dir = np.array(rotated_dir)

        # 3. Compute ray end point
        ray_end = sensor_world_pos + rotated_dir * self.max_range

        # 4. Cast the ray
        result = p.rayTest(sensor_world_pos.tolist(), ray_end.tolist())
        result = [result[0]]
        hit_object_id = result[0][0]   # -1 if no hit
        hit_fraction  = result[0][2]   # fraction along ray where hit occurred
        hit_pos       = result[0][3]   # (x,y,z) hit position in world

        # 5. Compute true distance and add noise
        true_distance = hit_fraction * self.max_range
        if hit_object_id != -1:
            noise    = np.random.normal(0, self.noise_std)
            distance = np.clip(true_distance + noise, 0, self.max_range)
        else:
            distance = self.max_range   # no hit = max range reading

        # 6. Check if hit object is the target beacon
        is_target = (hit_object_id == self.target_id) and (self.target_id is not None)

        # 7. Draw debug ray in GUI
        #    Green = no hit or hit non-target | Red = obstacle | Magenta = target!
        if hit_object_id == -1:
            color = [0, 1, 0]       # green — clear path
        elif is_target:
            color = [1, 0, 1]       # magenta — TARGET FOUND
        else:
            color = [1, 0, 0]       # red — obstacle hit

        ray_draw_end = (sensor_world_pos + rotated_dir * distance).tolist()

        if self._line_id is None:
            self._line_id = p.addUserDebugLine(
                sensor_world_pos.tolist(), ray_draw_end, color,
                lineWidth=1.5, lifeTime=0
            )
        else:
            self._line_id = p.addUserDebugLine(
                sensor_world_pos.tolist(), ray_draw_end, color,
                lineWidth=1.5, lifeTime=0,
                replaceItemUniqueId=self._line_id
            )

        return {
            'distance':   distance,
            'hit_object': hit_object_id,
            'is_target':  is_target,
            'hit_pos':    hit_pos,
        }


# ══════════════════════════════════════════════════════════════════
# IRSensorArray — mounts multiple sensors on a robot
# ══════════════════════════════════════════════════════════════════
class IRSensorArray:
    """
    A ring of IR sensors mounted around the robot body.

    Sensors are evenly spaced in a horizontal ring, all pointing outward.
    This gives the robot a 360-degree awareness of its surroundings.

    Parameters:
        robot_id   — PyBullet body ID
        n_sensors  — number of sensors in the ring
        radius     — distance from body center to sensor (meters)
        height     — height of sensor ring above base (meters)
        max_range  — sensing range per sensor (meters)
        noise_std  — Gaussian noise standard deviation (meters)
        target_id  — body ID of the IR beacon target
    """
    def __init__(self, robot_id, n_sensors=8, radius=0.3,
                 height=0.1, max_range=2.0, noise_std=0.02, target_id=None):
        self.sensors = []
        for i in range(n_sensors):
            angle = 2 * np.pi * i / n_sensors
            # Sensor offset: evenly spaced around the body
            offset    = [radius * np.cos(angle), radius * np.sin(angle), height]
            # Direction: pointing outward from body center
            direction = [np.cos(angle), np.sin(angle), 0.0]
            self.sensors.append(
                IRSensor(robot_id, offset, direction,
                         max_range=max_range, noise_std=noise_std,
                         target_id=target_id)
            )

    def read_all(self):
        """Read all sensors. Returns list of readings."""
        return [s.read() for s in self.sensors]

    def detect_target(self, readings):
        """
        Check if any sensor detected the target beacon.
        Returns (detected, bearing_index, distance) or (False, None, None).
        """
        for i, r in enumerate(readings):
            if r['is_target']:
                return True, i, r['distance']
        return False, None, None


# ══════════════════════════════════════════════════════════════════
# Simulation setup
# ══════════════════════════════════════════════════════════════════
if __name__ == '__main__':
    DT       = 1 / 240
    DURATION = 15.0
    STEPS    = int(DURATION / DT)

    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(DT)

    # Ground plane
    plane = p.loadURDF("plane.urdf")

    # Robot — fixed base so we can focus on sensor behavior
    robot = p.loadURDF("a1/a1.urdf",
                    basePosition=[0, 0, 0.35],
                    useFixedBase=True)
    n_joints = p.getNumJoints(robot)

    # Pre-set standing pose (from Phase 1B)
    standing_angles = {
        1: 0.0, 3: 0.67, 4: -1.3,
        6: 0.0, 8: 0.67, 9: -1.3,
        11: 0.0, 13: 0.67, 14: -1.3,
        16: 0.0, 18: 0.67, 19: -1.3,
    }
    for idx, angle in standing_angles.items():
        p.resetJointState(robot, idx, angle, 0.0)

    # ── Target object (IR beacon) ──────────────────────────────────────
    # A bright red sphere placed 1.5m in front of the robot.
    # In Phase 3 this becomes the search target both robots look for.
    target_shape  = p.createCollisionShape(p.GEOM_SPHERE, radius=0.25)
    target_visual = p.createVisualShape(p.GEOM_SPHERE, radius=0.25,
                                        rgbaColor=[1, 0, 0, 1])  # red
    target = p.createMultiBody(baseMass=0,                        # static object
                                baseCollisionShapeIndex=target_shape,
                                baseVisualShapeIndex=target_visual,
                                basePosition=[1.5, 0, 0.3])
    print(f"Target beacon ID: {target}")

    # ── Obstacle walls ─────────────────────────────────────────────────
    # Two boxes to show sensor detecting non-target obstacles (turns red)
    wall_shape  = p.createCollisionShape(p.GEOM_BOX,
                                        halfExtents=[0.05, 0.5, 0.3])
    wall_visual = p.createVisualShape(p.GEOM_BOX,
                                    halfExtents=[0.05, 0.5, 0.3],
                                    rgbaColor=[0.5, 0.5, 0.5, 1])
    wall1 = p.createMultiBody(baseMass=0,
                            baseCollisionShapeIndex=wall_shape,
                            baseVisualShapeIndex=wall_visual,
                            basePosition=[0, 1.2, 0.3])
    wall2 = p.createMultiBody(baseMass=0,
                            baseCollisionShapeIndex=wall_shape,
                            baseVisualShapeIndex=wall_visual,
                            basePosition=[0, -1.2, 0.3])
    print(f"Obstacles created at y=±1.2m")

    # ── IR sensor array ────────────────────────────────────────────────
    # 8 sensors evenly spaced in a ring, 2m range, 2cm noise, target awareness
    sensors = IRSensorArray(
        robot_id=robot,
        n_sensors=8,
        radius=0.7,
        height=0.15,
        max_range=2.0,
        noise_std=0.02,
        target_id=target
    )
    print(f"IR sensor array: {len(sensors.sensors)} sensors, 2m range\n")

    # ── Hold standing pose with PD control ────────────────────────────
    revolute_indices = [i for i in range(n_joints)
                        if p.getJointInfo(robot, i)[2] != 4]

    # ── Data recording ─────────────────────────────────────────────────
    times            = []
    sensor0_dist     = []   # sensor 0 (pointing forward +X)
    target_detected  = []

    # ══════════════════════════════════════════════════════════════════
    # Main loop
    # ══════════════════════════════════════════════════════════════════
    print("Running Phase 2 — IR sensor demo")
    print("Watch the GUI: green rays = clear, red rays = obstacle, magenta = TARGET\n")

    for step in range(STEPS):
        t = step * DT

        # Hold standing pose
        for idx in revolute_indices:
            p.setJointMotorControl2(
                robot, idx,
                controlMode=p.POSITION_CONTROL,
                targetPosition=standing_angles.get(idx, 0.0),
                positionGain=20.0,
                velocityGain=2.0,
                force=33.0
            )

        # Move target slowly in a circle so sensors track it dynamically
        # This simulates a moving search target
        target_x = 1.0 * np.cos(0.3 * t)
        target_y = 1.0 * np.sin(0.3 * t)
        p.resetBasePositionAndOrientation(
            target,
            [target_x, target_y, 0.3],
            [0, 0, 0, 1]
        )

        # Read all IR sensors
        readings = sensors.read_all()

        # Check for target detection
        detected, bearing_idx, dist = sensors.detect_target(readings)

        p.stepSimulation()
        time.sleep(DT * 3)

        # Record
        times.append(t)
        sensor0_dist.append(readings[0]['distance'])
        target_detected.append(1.0 if detected else 0.0)

        if step % 240 == 0:
            closest = min(readings, key=lambda r: r['distance'])
            print(f"  t={t:.1f}s  "
                f"sensor0={readings[0]['distance']:.2f}m  "
                f"closest={closest['distance']:.2f}m  "
                f"target={'DETECTED at sensor '+str(bearing_idx)+' d='+f'{dist:.2f}m' if detected else 'not in range'}")

    # ── Save ───────────────────────────────────────────────────────────
    np.save('phase2/sensor_data.npy', {
        'times':           np.array(times),
        'sensor0_dist':    np.array(sensor0_dist),
        'target_detected': np.array(target_detected),
    })
    print("\nData saved. Run: python3 phase2/plot_sensor.py")
    os._exit(0)