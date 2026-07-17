"""
Phase 4 - Mobile swarm with Option B locomotion.

Option B: kinematic base movement + animated legs.
  - Base moves by direct position update each timestep (no physics on base)
  - Legs run the trot gait visually via joint position control
  - No balance problem, no runaway velocity, reliable navigation

This lets Phase 4 focus on what it is actually about:
  frontier-based search, APF collision avoidance, and swarm coordination.

The robots look like they are walking because the legs animate.
The base moves smoothly under navigation commands.
Sensors read correctly from the moving base position.

Navigation:
  - Frontier navigator picks nearest unexplored grid cell
  - APF computes velocity from frontier pull + wall/peer repulsion
  - No attractive force toward target (unknown until sensor fires)
  - Local minimum escape: random kick after stuck_steps at low speed
"""
"""
UPDATE: This uses the waypoint navigator instead of frontier navigator. The waypoint navigator is a simple deterministic navigator that visits a fixed set of waypoints in sequence around the arena. When all waypoints are visited, it repeats from the start. This guarantees coverage without depending on the pheromone grid being populated first.
"""
import time
import math
import threading
import numpy as np
import pybullet as p
import pybullet_data
import sys
import os
import json

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "phase2"))
from ir_sensor import IRSensorArray

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "phase3"))
from pheromone_grid import PheromoneGrid
from track_filter   import TrackFilter
from protocol       import (build_message, send, recv,
                             make_sender, make_receiver,
                             WALNUT_PORT, HAZEL_PORT)

from navigation import FrontierNavigator, APFNavigator

# Physics world
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setTimeStep(1/240)
p.loadURDF("plane.urdf")

WALNUT_ID = p.loadURDF("a1/a1.urdf",
                        basePosition=[-1.5, 0, 0.35],
                        useFixedBase=False)
HAZEL_ID  = p.loadURDF("a1/a1.urdf",
                        basePosition=[ 1.5, 0, 0.35],
                        useFixedBase=False)

target_col = p.createCollisionShape(p.GEOM_SPHERE, radius=0.35)
target_vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.35,
                                 rgbaColor=[1, 0, 0, 1])
TARGET_ID  = p.createMultiBody(baseMass=0,
                                baseCollisionShapeIndex=target_col,
                                baseVisualShapeIndex=target_vis,
                                basePosition=[-2.2, 0, 0.3])

# Arena boundaries (used by APF and for clamping robot position)
ARENA_LIMIT = 2.5   # robots stay within this radius from center

# Wall obstacle positions for APF repulsion
WALL_POSITIONS = [(0, 2.8), (0, -2.8), (2.8, 0), (-2.8, 0)]
WALL_RADII     = [0.1, 0.1, 0.1, 0.1]

for xp, yp, hx, hy in [(0, 2.8, 3.0, 0.05),
                         (0,-2.8, 3.0, 0.05),
                         (2.8, 0, 0.05, 3.0),
                         (-2.8,0, 0.05, 3.0)]:
    wc = p.createCollisionShape(p.GEOM_BOX, halfExtents=[hx, hy, 0.5])
    wv = p.createVisualShape( p.GEOM_BOX, halfExtents=[hx, hy, 0.5],
                              rgbaColor=[0.6, 0.6, 0.6, 1])
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wc,
                      baseVisualShapeIndex=wv,
                      basePosition=[xp, yp, 0.5])

# Disable gravity on robot bases — we control position kinematically
# Without this, free-base robots fall through the floor each step
p.changeDynamics(WALNUT_ID, -1, mass=0)  # -1 = base link
p.changeDynamics(HAZEL_ID,  -1, mass=0)

print(f"World: Walnut={WALNUT_ID} Hazel={HAZEL_ID} Target={TARGET_ID}")

pb_lock    = threading.Lock()
stop_event = threading.Event()

# Shared desired poses — agent threads write, physics thread applies after stepSimulation
# This ensures resetBasePositionAndOrientation happens AFTER stepSimulation, not before
desired_poses = {
    'walnut': None,
    'hazel':  None,
}
poses_lock = threading.Lock()

STANDING = {1:0.0,  3:0.67, 4:-1.3,
            6:0.0,  8:0.67, 9:-1.3,
            11:0.0, 13:0.67,14:-1.3,
            16:0.0, 18:0.67,19:-1.3}

n_joints = p.getNumJoints(WALNUT_ID)
revolute_indices = [i for i in range(n_joints)
                    if p.getJointInfo(WALNUT_ID, i)[2] != 4]

for robot_id in [WALNUT_ID, HAZEL_ID]:
    for idx, angle in STANDING.items():
        p.resetJointState(robot_id, idx, angle, 0.0)


def gait_targets(t, freq=1.0, swing_amp=0.25, lift_amp=0.3):
    """Trot gait - visual animation for Option B."""
    phase_A = 2 * math.pi * freq * t
    phase_B = phase_A + math.pi

    def leg(phase):
        h_flex = 0.67 + swing_amp * math.sin(phase)
        knee   = -1.3 + lift_amp * max(0.0, math.sin(phase))
        return 0.0, h_flex, knee

    fr = leg(phase_A)
    rl = leg(phase_A)
    fl = leg(phase_B)
    rr = leg(phase_B)

    return {
         1:fr[0],  3:fr[1],  4:fr[2],
         6:fl[0],  8:fl[1],  9:fl[2],
        11:rr[0], 13:rr[1], 14:rr[2],
        16:rl[0], 18:rl[1], 19:rl[2],
    }


class WaypointNavigator:
    """
    Simple deterministic waypoint navigator.
    Visits a fixed set of waypoints in sequence around the arena.
    When all waypoints visited, repeats from the start.
    This guarantees coverage without depending on the pheromone grid
    being populated first.
    """
    def __init__(self, waypoints, arrival_radius=0.4):
        self.waypoints      = waypoints
        self.arrival_radius = arrival_radius
        self.current_idx    = 0

    def update(self, robot_pos):
        rx, ry = robot_pos[0], robot_pos[1]
        wx, wy = self.waypoints[self.current_idx]
        dist = math.sqrt((rx - wx)**2 + (ry - wy)**2)
        if dist < self.arrival_radius:
            self.current_idx = (self.current_idx + 1) % len(self.waypoints)
            wx, wy = self.waypoints[self.current_idx]
        return (wx, wy)


class Agent:
    def __init__(self, name, robot_id, listen_port, send_port, peer_robot_id):
        self.name        = name
        self.robot_id    = robot_id
        self.peer_id     = peer_robot_id
        self.listen_port = listen_port
        self.send_port   = send_port
        self.N_SENSORS   = 8

        # Raise mu_thresh to keep patches small (fixes large message warnings)
        self.grid = PheromoneGrid(
            width_m=7.0, height_m=7.0, resolution=0.2,
            Q=0.0005, R=0.04, decay_rate=0.0002,
            sigma_init=1.0, mu_thresh=0.15
        )
        self.track    = TrackFilter(dt=1/240, q_pos=0.0001, q_vel=0.0005,
                                    r_pos=0.09, init_sigma=0.5,
                                    init_v_sigma=0.8)
        # Waypoint pattern — lawnmower coverage of arena
        # Offset by pi for each robot so they cover different areas
        offset = 0 if "walnut" in name else math.pi
        r = 1.8  # patrol radius
        n = 8    # number of waypoints
        self.waypoints = WaypointNavigator([
            (r * math.cos(2*math.pi*i/n + offset),
             r * math.sin(2*math.pi*i/n + offset))
            for i in range(n)
        ])  # circular patrol, no center waypoint to avoid APF deadlock
        self.apf = APFNavigator(
            k_att=1.0, k_rep=2.0, d0=1.0,
            max_speed=0.3, max_omega=1.5,
            stuck_steps=60, stuck_thresh=0.02
        )

        self.sensor_angles = [2*math.pi*i/self.N_SENSORS
                              for i in range(self.N_SENSORS)]
        self.sender   = make_sender()
        self.receiver = make_receiver(listen_port)

        with pb_lock:
            self.sensors = IRSensorArray(
                robot_id=robot_id, n_sensors=self.N_SENSORS,
                radius=0.7, height=0.0, max_range=2.5,
                noise_std=0.02, target_id=TARGET_ID
            )

        # Current robot state (maintained by Option B kinematic update)
        pos0, orn0 = p.getBasePositionAndOrientation(robot_id)
        e0         = p.getEulerFromQuaternion(orn0)
        self.rx      = pos0[0]
        self.ry      = pos0[1]
        self.heading = e0[2]

        self.log = {
            "times": [], "pos_x": [], "pos_y": [],
            "target_true_x": [], "target_true_y": [],
            "est_x": [], "est_y": [],
            "uncertainty": [], "detections": [],
            "frontier_x": [], "frontier_y": [],
            "msgs_sent": 0, "msgs_received": 0,
            "target_found_time": None
        }

    def _kinematic_step(self, vx, vy, omega, dt):
        """
        Option B: move base kinematically.

        Update heading and position from velocity command,
        clamp to arena bounds, then teleport the base.
        This bypasses physics on the base entirely - no tipping,
        no runaway, reliable movement.
        """
        # Update heading
        self.heading += omega * dt
        self.heading  = math.atan2(math.sin(self.heading),
                                    math.cos(self.heading))

        # Update position
        self.rx += vx * dt
        self.ry += vy * dt


        # Clamp to arena
        self.rx = max(-ARENA_LIMIT, min(ARENA_LIMIT, self.rx))
        self.ry = max(-ARENA_LIMIT, min(ARENA_LIMIT, self.ry))

        # Write desired pose to shared dict — physics thread applies after stepSimulation
        orn = p.getQuaternionFromEuler([0, 0, self.heading])
        with poses_lock:
            desired_poses[self.name] = ([self.rx, self.ry, 0.35], orn)

    def run(self):
        DT         = 1/240
        DURATION   = 90.0
        STEPS      = int(DURATION / DT)
        SEND_EVERY = 24
        LOG_EVERY  = 240

        print(f"[{self.name}] starting — Option B kinematic locomotion")
        fusions_this_sec = 0

        for step in range(STEPS):
            if stop_event.is_set():
                break
            t = step * DT

            # Read world state — sync kinematic pose to PyBullet BEFORE sensor read
            # so raycasts originate from the correct world position
            with pb_lock:
                kin_orn = p.getQuaternionFromEuler([0, 0, self.heading])
                p.resetBasePositionAndOrientation(
                    self.robot_id, [self.rx, self.ry, 0.35], kin_orn
                )
                _, orn       = p.getBasePositionAndOrientation(self.robot_id)
                euler        = p.getEulerFromQuaternion(orn)
                heading      = euler[2]
                readings     = self.sensors.read_all()
                true_tpos, _ = p.getBasePositionAndOrientation(TARGET_ID)
                peer_pos, _  = p.getBasePositionAndOrientation(self.peer_id)

            # Use kinematic state as authoritative position
            # PyBullet base position lags due to threading
            pos = (self.rx, self.ry, 0.35)

            # Perception
            self.grid.slide_to(self.rx, self.ry)
            if step % 10 == 0:
                self.grid.predict()
            if step % 5 == 0:
                self.grid.decay()
            self.track.predict()

            detected = False
            for i, r in enumerate(readings):
                if r["is_target"] and r["distance"] < 2.4:
                    bearing = heading + self.sensor_angles[i]
                    tx_est  = self.rx + r["distance"] * math.cos(bearing)
                    ty_est  = self.ry + r["distance"] * math.sin(bearing)
                    self.grid.deposit(tx_est, ty_est, measurement=1.0)
                    self.track.update(tx_est, ty_est)
                    detected = True
                    if self.log["target_found_time"] is None:
                        self.log["target_found_time"] = t
                        print(f"  [{self.name} t={t:.1f}s] TARGET FOUND! "
                              f"est=[{tx_est:.2f},{ty_est:.2f}]")

            if not detected:
                g_est, g_conf, _ = self.grid.best_target_estimate()
                if g_est and g_conf > 0.3:
                    self.track.update(g_est[0], g_est[1])

            # Communication
            msg = recv(self.receiver)
            if msg:
                self.log["msgs_received"] += 1
                patch     = msg.get("patch", {})
                t_sent    = patch.get("timestamp", t)
                staleness = max(0.0, t - t_sent)
                cells     = patch.get("cells", [])
                if cells:
                    self.grid.fuse_patch(cells, staleness_seconds=staleness)
                    fusions_this_sec += 1
                peer_track = msg.get("track_state")
                if peer_track and self.track.initialised:
                    try:
                        px = np.array(peer_track["x"])
                        pP = np.array(peer_track["P"])
                        self.track.fuse_peer_track(px, pP, staleness)
                    except Exception:
                        pass

            if step % SEND_EVERY == 0:
                patch = self.grid.build_patch(timestamp=t)
                track_state = None
                if self.track.initialised:
                    track_state = {
                        "x": self.track.x.tolist(),
                        "P": self.track.P.tolist(), "t": t
                    }
                out_msg = build_message(
                    self.name, t, list(pos), heading, patch
                )
                out_msg["track_state"] = track_state
                send(self.sender, out_msg, self.send_port)
                self.log["msgs_sent"] += 1

            # Navigation - APF with frontier
            frontier = self.waypoints.update((self.rx, self.ry, 0.35))
            obs_pos  = list(WALL_POSITIONS) + [(peer_pos[0], peer_pos[1])]
            obs_rad  = list(WALL_RADII)     + [0.5]
            vx, vy, omega = self.apf.compute(
                (self.rx, self.ry, 0.35), heading, frontier, obs_pos, obs_rad
            )

            # Locomotion - Option B
            # 1. Update kinematic state (writes desired pose to shared dict)
            self._kinematic_step(vx, vy, omega, DT)
            # 2. Command leg joints
            with pb_lock:
                targets = gait_targets(t)
                for idx in revolute_indices:
                    p.setJointMotorControl2(
                        self.robot_id, idx,
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=targets.get(idx, 0.0),
                        positionGain=20.0, velocityGain=2.0,
                        force=33.0
                    )

            # Logging
            if step % LOG_EVERY == 0:
                pos_est, _, unc = self.track.get_estimate()
                self.log["times"].append(t)
                self.log["pos_x"].append(self.rx)
                self.log["pos_y"].append(self.ry)
                self.log["target_true_x"].append(true_tpos[0])
                self.log["target_true_y"].append(true_tpos[1])
                self.log["est_x"].append(
                    pos_est[0] if pos_est else float("nan"))
                self.log["est_y"].append(
                    pos_est[1] if pos_est else float("nan"))
                self.log["uncertainty"].append(unc)
                self.log["detections"].append(1 if detected else 0)
                self.log["frontier_x"].append(
                    frontier[0] if frontier else float("nan"))
                self.log["frontier_y"].append(
                    frontier[1] if frontier else float("nan"))

                if step % (LOG_EVERY * 5) == 0:
                    est_str = (f"[{pos_est[0]:.2f},{pos_est[1]:.2f}]"
                               if pos_est else "none")
                    fr_str  = (f"[{frontier[0]:.2f},{frontier[1]:.2f}]"
                               if frontier else "none")
                    print(f"  [{self.name} t={t:.0f}s] "
                          f"pos=[{self.rx:.2f},{self.ry:.2f}] "
                          f"est={est_str} unc={unc:.3f} "
                          f"frontier={fr_str} "
                          f"fusions={fusions_this_sec}")
                fusions_this_sec = 0

            time.sleep(DT)

        print(f"[{self.name}] done — "
              f"sent={self.log["msgs_sent"]} "
              f"recv={self.log["msgs_received"]}")
        if self.log["target_found_time"]:
            print(f"  Target found at t={self.log["target_found_time"]:.1f}s")

    def save_log(self):
        os.makedirs("phase4", exist_ok=True)
        path = f"phase4/{self.name}_log.json"
        with open(path, "w") as f:
            json.dump(self.log, f, indent=2)
        print(f"[{self.name}] log saved to {path}")


def physics_loop():
    t  = 0.0
    DT = 1/240
    while not stop_event.is_set():
        with pb_lock:
            # Move target
            tx = 2.2 * math.cos(0.15 * t + math.pi)
            ty = 2.2 * math.sin(0.15 * t + math.pi)
            p.resetBasePositionAndOrientation(
                TARGET_ID, [tx, ty, 0.55], [0, 0, 0, 1]
            )
            p.stepSimulation()
            # Apply agent kinematic poses AFTER stepSimulation
            # so physics engine cannot overwrite our teleports
            with poses_lock:
                for name, robot_id in [("walnut", WALNUT_ID),
                                        ("hazel",  HAZEL_ID)]:
                    pose = desired_poses.get(name)
                    if pose is not None:
                        p.resetBasePositionAndOrientation(
                            robot_id, pose[0], pose[1]
                        )
        time.sleep(DT)
        t += DT


walnut = Agent("walnut", WALNUT_ID, WALNUT_PORT, HAZEL_PORT, HAZEL_ID)
hazel  = Agent("hazel",  HAZEL_ID,  HAZEL_PORT,  WALNUT_PORT, WALNUT_ID)

threads = [
    threading.Thread(target=physics_loop, daemon=True, name="physics"),
    threading.Thread(target=walnut.run,   daemon=True, name="walnut"),
    threading.Thread(target=hazel.run,    daemon=True, name="hazel"),
]

print("\nStarting Phase 4 — Option B locomotion\n")
for th in threads:
    th.start()

try:
    for th in threads[1:]:
        th.join()
except KeyboardInterrupt:
    print("\nShutting down...")
    stop_event.set()

walnut.save_log()
hazel.save_log()
print("\nDone. Run: python3 phase4/plot_phase4.py")
os._exit(0)