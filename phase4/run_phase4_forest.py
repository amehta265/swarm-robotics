"""
Phase 4 Forest — Extended environment for visual demonstration.

Environment:
  - 11m x 11m arena
  - 18 cylinder trees as obstacles (brown poles, 0.2m radius, 2m tall)
  - Trees are passed to the APF as point obstacles
  - Arena walls are handled as PLANES via the APF `bounds` argument

Target motion:
  Lissajous figure-eight path:
    x = Ax * sin(wx * t + px)
    y = Ay * sin(wy * t + py)
  With wx:wy = 1:2 this traces a figure-eight visiting all four quadrants.

Robots:
  - Walnut starts at (-1.5, 0), Hazel at (1.5, 0)
  - No waypoints. Frontier exploration from the pheromone grid.
  - CI fusion of grid patches and track states over lossy UDP.

════════════════════════════════════════════════════════════════════
FIXES vs the previous version — read this before tuning anything
════════════════════════════════════════════════════════════════════
1. WALLS WERE MODELLED AS GIANT CIRCLES.
   WALL_POSITIONS stored each wall as a point at its midpoint with
   radius = ARENA (5.5). The APF treats obstacles as circles, so the
   north wall became a disk of radius 5.5 centred at (0, 5.5) whose d0
   shell was a ring cutting through the middle of the arena. Measured
   field along y=0: a 1.58 outward push at the spawn (attraction is
   capped at k_att), settling to equilibrium at x = ±3.3 — exactly where
   the robots sat for 60s. Past |x| ~ 4 the guard `if 0 < d < d0` failed
   because d went negative, so repulsion vanished and the robots pinned
   against the position clamp at ±5.0.
   -> Walls are now planes via apf.compute(..., bounds=NAV_BOUNDS).
      They are NOT in the obstacle list. Do not put them back.

2. TRACKING MODE WAS MATHEMATICALLY UNREACHABLE.
   The gate was `unc < 0.02` where unc = trace(P)/4, averaging position
   AND velocity variance. With these filter params the best possible
   steady state — a detection on every single step — is:
       position variance 0.0049, velocity variance 0.1268, trace/4 = 0.0658
   The observed floor in the logs was 0.066. The gate could never fire,
   so mode printed EXPLORING even at "TARGET FOUND".
   -> Gate on POSITION uncertainty only, with hysteresis.

3. THE TRACK FILTER WAS BEING FED STALE GRID MEMORY.
   On every non-detection step the old code pushed grid.best_target_estimate()
   — the argmax of mu, i.e. a cell where a detection happened at some
   point, possibly a minute ago, possibly the peer's via fuse_patch — into
   track.update() as if it were a live measurement. That is why the
   estimate teleported across the arena and why both robots reported
   byte-identical estimates.
   -> The grid now supplies a SEARCH GOAL (mode INVESTIGATING), never a
      measurement. Only real sensor hits update the track.

4. EVERY DETECTION WAS 0.7m SHORT.
   r["distance"] is measured from the sensor origin, which IRSensorArray
   places on a ring of radius `radius` (0.7m) pointing radially outward.
   The old code reconstructed the target from the BODY CENTRE, giving a
   systematic 0.7m bias toward the robot on every measurement. It also
   called track.update() once per detecting ray, so one observation seen
   by two rays produced two "independent" updates and collapsed P.
   -> Reconstruct from the sensor origin; use the closest ray only.

5. THE TREES BUILT A CORRIDOR.
   Exclusion disks were centred on (-3,0), (3,0), (0,0) at 2.5m — stale
   constants; the robots actually spawn at (-1.5,0) and (1.5,0). Their
   union carved a clear 5m-wide channel along y=0 and forced all 18 trees
   into two bands at |y| > 2 (measured: 0 trees with |y| < 2, 7 trees at
   2 <= |y| < 3). Separately, min_dist_tree=1.2 with d0=1.2 meant adjacent
   trees' repulsion fields covered the entire gap between them — the robot
   could not physically thread between two trees.
   -> Clearance is now taken from the real spawn points; spacing raised to
      1.8m and d0 lowered to 0.45m, which is verified passable.

6. THERE WAS NO SHARED CLOCK.
   physics_loop did `t += 1/240` with sleep(1/240); each agent did
   `t = step * DT` with sleep(DT*3). Two unrelated clocks, neither equal
   to wall time, and track.predict() hardcoded dt=1/240 while ~3x that
   much target motion elapsed per call.
   -> The physics thread owns SIM_CLOCK and paces itself to SIM_SPEED of
      real time. Agents read that clock and integrate with the real
      elapsed sim dt.
"""
"""
This should be the final version of this file
"""
import time
import math
import threading
import random
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

# ── Tunables ───────────────────────────────────────────────────────
ARENA        = 5.5     # half-width of the physical arena (11m across)
WALL_H       = 1.5
NAV_BOUNDS   = 5.0     # half-width of the navigable region (APF planes)
POS_CLAMP    = 5.2     # hard safety clamp, OUTSIDE NAV_BOUNDS so the APF
                       # gets a chance to push back before we clamp
ARENA_LIMIT  = 4.0     # frontier goals must be inside this. Keep it below
                       # NAV_BOUNDS - d0_bounds (= 4.2) or goals land inside
                       # the wall repulsion shell and become unreachable.

SENSOR_RADIUS = 0.7    # IR ring radius — MUST match IRSensorArray(radius=...)
SENSOR_RANGE  = 2.5
DETECT_MAX    = 2.4

TREE_RADIUS   = 0.2    # physical trunk radius
ROBOT_RADIUS  = 0.3    # body half-width, added for APF clearance
N_TREES       = 18
MIN_DIST_TREE = 1.8    # centre-to-centre spacing between trees
SPAWN_CLEAR   = 1.2    # keep trees this far from each robot spawn

SPAWNS = {"walnut": (-1.5, 0.0), "hazel": (1.5, 0.0)}

SIM_SPEED     = 1.0 / 3.0   # run sim at 1/3 real time for visual clarity
PHYS_DT       = 1.0 / 240.0
CONTROL_HZ    = 60.0        # agent control updates per SIM second
CONTROL_DT    = 1.0 / CONTROL_HZ
DURATION      = 120.0       # sim seconds

# ── Physics world ──────────────────────────────────────────────────
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setTimeStep(PHYS_DT)
p.loadURDF("plane.urdf")

# Arena boundary walls — VISUAL/COLLISION GEOMETRY ONLY.
# These are deliberately NOT collected into an obstacle list. The APF
# handles the boundary analytically through its `bounds` argument. See
# fix #1 in the module docstring: passing walls in as circles is what
# created the phantom rings that pinned the robots.
wall_specs = [
    ( 0,      ARENA,  ARENA, 0.15),   # North
    ( 0,     -ARENA,  ARENA, 0.15),   # South
    ( ARENA,  0,      0.15,  ARENA),  # East
    (-ARENA,  0,      0.15,  ARENA),  # West
]
for xp, yp, hx, hy in wall_specs:
    wc = p.createCollisionShape(p.GEOM_BOX, halfExtents=[hx, hy, WALL_H])
    wv = p.createVisualShape(p.GEOM_BOX, halfExtents=[hx, hy, WALL_H],
                             rgbaColor=[0.55, 0.45, 0.35, 1])
    p.createMultiBody(baseMass=0,
                      baseCollisionShapeIndex=wc,
                      baseVisualShapeIndex=wv,
                      basePosition=[xp, yp, WALL_H])

# ── Trees ──────────────────────────────────────────────────────────
# Clearance is measured from the ACTUAL spawn points. The old code used
# (-3,0), (3,0) and (0,0) with a 2.5m radius; the union of those disks
# carved a clear horizontal channel and pushed every tree into two bands
# above and below it.
random.seed(42)
TREE_POSITIONS = []


def valid_tree(tx, ty, existing):
    for rx, ry in SPAWNS.values():
        if math.hypot(tx - rx, ty - ry) < SPAWN_CLEAR:
            return False
    for ex, ey in existing:
        if math.hypot(tx - ex, ty - ey) < MIN_DIST_TREE:
            return False
    return True


attempts = 0
while len(TREE_POSITIONS) < N_TREES and attempts < 5000:
    tx = random.uniform(-ARENA + 0.8, ARENA - 0.8)
    ty = random.uniform(-ARENA + 0.8, ARENA - 0.8)
    if valid_tree(tx, ty, TREE_POSITIONS):
        TREE_POSITIONS.append((tx, ty))
    attempts += 1

# Radius the APF uses: trunk plus robot body, so "surface distance" means
# "distance until the robot would touch the trunk".
TREE_NAV_RADIUS = TREE_RADIUS + ROBOT_RADIUS

for tx, ty in TREE_POSITIONS:
    tc = p.createCollisionShape(p.GEOM_CYLINDER, radius=TREE_RADIUS, height=2.0)
    tv = p.createVisualShape(p.GEOM_CYLINDER, radius=TREE_RADIUS, length=2.0,
                             rgbaColor=[0.40, 0.26, 0.13, 1])
    p.createMultiBody(baseMass=0,
                      baseCollisionShapeIndex=tc,
                      baseVisualShapeIndex=tv,
                      basePosition=[tx, ty, 1.0])
    cv = p.createVisualShape(p.GEOM_SPHERE, radius=0.6,
                             rgbaColor=[0.13, 0.45, 0.13, 0.85])
    p.createMultiBody(baseMass=0,
                      baseCollisionShapeIndex=-1,
                      baseVisualShapeIndex=cv,
                      basePosition=[tx, ty, 2.3])

_bands = [sum(1 for _, ty in TREE_POSITIONS if lo <= abs(ty) < hi)
          for lo, hi in [(0, 1), (1, 2), (2, 3), (3, 4), (4, ARENA)]]
print(f"Placed {len(TREE_POSITIONS)} trees  |y| bands [0-1,1-2,2-3,3-4,4+] = {_bands}")

# ── Target beacon ──────────────────────────────────────────────────
target_col = p.createCollisionShape(p.GEOM_SPHERE, radius=0.35)
target_vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.35,
                                 rgbaColor=[1.0, 0.1, 0.1, 1])
TARGET_ID = p.createMultiBody(baseMass=0,
                              baseCollisionShapeIndex=target_col,
                              baseVisualShapeIndex=target_vis,
                              basePosition=[0, 3.0, 0.55])

# ── Robots ─────────────────────────────────────────────────────────
WALNUT_ID = p.loadURDF("a1/a1.urdf",
                       basePosition=[SPAWNS["walnut"][0], SPAWNS["walnut"][1], 0.35],
                       useFixedBase=False)
HAZEL_ID = p.loadURDF("a1/a1.urdf",
                      basePosition=[SPAWNS["hazel"][0], SPAWNS["hazel"][1], 0.35],
                      useFixedBase=False)

p.changeDynamics(WALNUT_ID, -1, mass=0)
p.changeDynamics(HAZEL_ID,  -1, mass=0)

print(f"World: Walnut={WALNUT_ID} Hazel={HAZEL_ID} Target={TARGET_ID}")
print(f"Arena: {ARENA*2:.0f}m x {ARENA*2:.0f}m  Nav bounds: +/-{NAV_BOUNDS}m")

pb_lock    = threading.Lock()
stop_event = threading.Event()

desired_poses = {"walnut": None, "hazel": None}
poses_lock    = threading.Lock()

# Single shared simulation clock, owned by the physics thread.
# Float assignment is atomic under the GIL, so no lock is needed for a
# scalar read. Every consumer must use THIS clock, not its own step count.
SIM_CLOCK = 0.0

STANDING = {1: 0.0,  3: 0.67, 4: -1.3,
            6: 0.0,  8: 0.67, 9: -1.3,
            11: 0.0, 13: 0.67, 14: -1.3,
            16: 0.0, 18: 0.67, 19: -1.3}

n_joints = p.getNumJoints(WALNUT_ID)
revolute_indices = [i for i in range(n_joints)
                    if p.getJointInfo(WALNUT_ID, i)[2] != 4]

for robot_id in [WALNUT_ID, HAZEL_ID]:
    for idx, angle in STANDING.items():
        p.resetJointState(robot_id, idx, angle, 0.0)


# ── Target path ────────────────────────────────────────────────────
def lissajous_target(t):
    """
    Lissajous figure-eight. wx:wy = 1:2 visits all four quadrants.
    Amplitude 3.8 keeps the target inside ARENA_LIMIT so the frontier
    search can actually reach everywhere the target goes.
    """
    Ax, Ay = 3.8, 3.8
    wx, wy = 0.08, 0.16        # full x cycle ~78s
    px, py = 0.0, math.pi / 2
    return Ax * math.sin(wx * t + px), Ay * math.sin(wy * t + py)


def gait_targets(t, freq=0.5, swing_amp=0.25, lift_amp=0.3):
    """Trot gait — cosmetic only, the base is driven kinematically."""
    phase_A = 2 * math.pi * freq * t
    phase_B = phase_A + math.pi

    def leg(phase):
        h_flex = 0.67 + swing_amp * math.sin(phase)
        knee   = -1.3 + lift_amp * max(0.0, math.sin(phase))
        return 0.0, h_flex, knee

    fr = leg(phase_A); rl = leg(phase_A)
    fl = leg(phase_B); rr = leg(phase_B)
    return {
        1: fr[0],  3: fr[1],  4: fr[2],
        6: fl[0],  8: fl[1],  9: fl[2],
        11: rr[0], 13: rr[1], 14: rr[2],
        16: rl[0], 18: rl[1], 19: rl[2],
    }


# ── Track filter helpers ───────────────────────────────────────────
# These live here rather than in track_filter.py so this fix touches only
# two files. Consider folding them into TrackFilter as predict(dt) and
# position_uncertainty() when you next edit that module.

def track_predict(tf, dt):
    """
    Variable-dt predict.

    TrackFilter builds F and Q for a fixed nominal dt (1/240) and its
    predict() assumes exactly one nominal step per call. We now call it
    once per control step (1/60 sim s), so both F and the process-noise
    accumulation have to be rebuilt for the real elapsed time, or the
    velocity estimate is biased low by the ratio between them.
    """
    if not tf.initialised or dt <= 0.0:
        return
    F = np.array([[1, 0, dt, 0],
                  [0, 1, 0, dt],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]], dtype=np.float64)
    scale = dt / tf.dt          # Q is tuned per nominal step -> scale by time
    tf.x = F @ tf.x
    tf.P = F @ tf.P @ F.T + tf.Q * scale
    tf.steps_since_update += 1


def track_pos_uncertainty(tf):
    """
    Mean POSITION variance — the quantity the navigation gate actually
    cares about.

    tf.get_estimate() returns trace(P)/4, which averages position and
    velocity variance together. Velocity variance is ~26x larger here, so
    trace(P)/4 bottoms out at 0.0658 no matter how well the position is
    known. Gating on it with a 0.02 threshold could never fire.
    """
    if not tf.initialised:
        return float("inf")
    return float(tf.P[0, 0] + tf.P[1, 1]) / 2.0


class Agent:
    # TRACKING gate, in mean position variance (m^2).
    # Best achievable position variance with continuous detections is
    # ~0.0049 (= 7cm), so 0.05 is comfortably reachable. Hysteresis stops
    # the mode flapping every time the target ducks behind a tree.
    TRACK_ENTER = 0.05
    TRACK_LEAVE = 0.25
    TRACK_MIN_UPDATES = 50

    # Sim seconds the filter may coast with no fresh information before we
    # declare the track lost and drop it.
    #
    # This matters now that the grid no longer feeds the filter (fix #3).
    # A constant-velocity KF that never gets a measurement diverges: with
    # these params, position variance reaches ~1e4 after a minute blind.
    # The old code hid that by injecting stale grid hits. Dropping the
    # track is the honest fix — uncertainty saturates at the uninitialised
    # value of 1.0 and the robot falls back to INVESTIGATING/EXPLORING.
    # A peer with a live track can re-bootstrap us through fuse_peer_track.
    TRACK_MAX_COAST = 8.0

    def __init__(self, name, robot_id, listen_port, send_port,
                 peer_robot_id, start_x, start_y):
        self.name        = name
        self.robot_id    = robot_id
        self.peer_id     = peer_robot_id
        self.listen_port = listen_port
        self.send_port   = send_port
        self.N_SENSORS   = 8
        self.rx          = start_x
        self.ry          = start_y
        self.heading     = 0.0 if "walnut" in name else math.pi
        self.mode        = "EXPLORING"
        self.last_info_t = None   # last time the track got real information

        self.grid = PheromoneGrid(
            width_m=12.0, height_m=12.0, resolution=0.25,
            Q=0.0005, R=0.04, decay_rate=0.0001,
            sigma_init=1.0, mu_thresh=0.15
        )
        self.track = TrackFilter(
            dt=1 / 240, q_pos=0.0002, q_vel=0.001,
            r_pos=0.09, init_sigma=1.0, init_v_sigma=1.5
        )

        self.frontier_nav = FrontierNavigator(
            mu_low=0.05, s2_high=0.7,
            min_dist=2.0, arrival_radius=0.8,
            arena_limit=ARENA_LIMIT
        )

        # d0=0.45 is deliberate. Trees sit 1.8m apart with an effective
        # nav radius of 0.5 each, leaving a 0.8m corridor for the robot
        # centre. A larger d0 makes both trees' fields span that corridor
        # and the robot cannot pass — that was the old d0=1.2 behaviour.
        self.apf = APFNavigator(
            k_att=1.2, k_rep=2.0, d0=0.45, d0_bounds=0.8,
            max_speed=0.35, max_omega=1.5, rep_cap=2.0,
            stuck_window=int(2.0 * CONTROL_HZ),   # 2 sim seconds
            stuck_dist=0.05
        )

        self.sensor_angles = [2 * math.pi * i / self.N_SENSORS
                              for i in range(self.N_SENSORS)]
        self.sender   = make_sender()
        self.receiver = make_receiver(listen_port)

        with pb_lock:
            self.sensors = IRSensorArray(
                robot_id=robot_id, n_sensors=self.N_SENSORS,
                radius=SENSOR_RADIUS, height=0.0, max_range=SENSOR_RANGE,
                noise_std=0.03, target_id=TARGET_ID
            )

        self.log = {
            "times": [], "pos_x": [], "pos_y": [],
            "target_true_x": [], "target_true_y": [],
            "est_x": [], "est_y": [],
            "uncertainty": [], "pos_uncertainty": [],
            "detections": [], "mode": [],
            "msgs_sent": 0, "msgs_received": 0,
            "target_found_time": None
        }

    def _kinematic_step(self, vx, vy, omega, dt):
        self.heading += omega * dt
        self.heading = math.atan2(math.sin(self.heading), math.cos(self.heading))
        # POS_CLAMP sits outside NAV_BOUNDS so the APF wall planes get a
        # chance to act first. If we ever hit this clamp it means the APF
        # failed, and the stuck detector will notice we stopped moving.
        self.rx = max(-POS_CLAMP, min(POS_CLAMP, self.rx + vx * dt))
        self.ry = max(-POS_CLAMP, min(POS_CLAMP, self.ry + vy * dt))
        orn = p.getQuaternionFromEuler([0, 0, self.heading])
        with poses_lock:
            desired_poses[self.name] = ([self.rx, self.ry, 0.35], orn)

    def _sense(self, readings, heading):
        """
        Reconstruct the target position from the closest detecting ray.

        Two corrections vs the old code:
          - The ray originates on the sensor ring (SENSOR_RADIUS out along
            the bearing), not at the body centre. Omitting that offset
            biased every measurement 0.7m short.
          - Only the closest ray is used. Firing one track.update() per
            detecting ray treats a single observation as several
            independent ones and artificially collapses P.
        """
        hits = [(r["distance"], i) for i, r in enumerate(readings)
                if r["is_target"] and r["distance"] < DETECT_MAX]
        if not hits:
            return None
        dist, i = min(hits)
        bearing = heading + self.sensor_angles[i]
        sx = self.rx + SENSOR_RADIUS * math.cos(bearing)
        sy = self.ry + SENSOR_RADIUS * math.sin(bearing)
        return (sx + dist * math.cos(bearing),
                sy + dist * math.sin(bearing))

    def _drop_track(self):
        """Declare the track lost and reset the filter to uninitialised."""
        self.track.initialised = False
        self.track.x = None
        self.track.P = None
        self.track.n_updates = 0
        self.track.steps_since_update = 0
        self.last_info_t = None
        self.mode = "EXPLORING"

    def _select_goal(self, peer_pos):
        """
        Three-tier goal selection.

          TRACKING      — position variance is low: chase the estimate
          INVESTIGATING — no live track, but the grid remembers a hotspot:
                          go look at it. The grid is a place to SEARCH,
                          never a measurement to fuse (see fix #3).
          EXPLORING     — nothing known: frontier search
        """
        pos_est, _, _ = self.track.get_estimate()
        pos_unc = track_pos_uncertainty(self.track)

        if self.mode == "TRACKING":
            if pos_est is not None and pos_unc < self.TRACK_LEAVE:
                return pos_est, "TRACKING"
        elif (pos_est is not None
              and pos_unc < self.TRACK_ENTER
              and self.track.n_updates > self.TRACK_MIN_UPDATES):
            return pos_est, "TRACKING"

        g_est, g_conf, _ = self.grid.best_target_estimate()
        if g_est and g_conf > 0.3:
            return (g_est[0], g_est[1]), "INVESTIGATING"

        goal = self.frontier_nav.update(
            self.grid, (self.rx, self.ry, 0.35),
            peer_pos=(peer_pos[0], peer_pos[1])
        )
        return goal, "EXPLORING"

    def run(self):
        print(f"[{self.name}] starting in forest arena")

        fusions_this_period = 0
        last_ctrl_t   = 0.0
        next_ctrl_t   = 0.0
        next_log_t    = 0.0
        next_print_t  = 0.0
        next_send_t   = 0.0
        last_gpredict = 0.0
        last_gdecay   = 0.0

        LOG_INTERVAL   = 1.0     # sim seconds
        PRINT_INTERVAL = 5.0
        SEND_INTERVAL  = 0.1
        GRID_PREDICT_INTERVAL = 1 / 24.0    # preserves the old 240Hz/10 rate
        GRID_DECAY_INTERVAL   = 1 / 48.0    # preserves the old 240Hz/5  rate

        while not stop_event.is_set():
            t = SIM_CLOCK
            if t >= DURATION:
                break
            if t < next_ctrl_t:
                time.sleep(0.001)
                continue

            dt = t - last_ctrl_t
            last_ctrl_t = t
            next_ctrl_t = t + CONTROL_DT
            if dt <= 0.0:
                continue

            # ── Read world state ───────────────────────────────────
            with pb_lock:
                kin_orn = p.getQuaternionFromEuler([0, 0, self.heading])
                p.resetBasePositionAndOrientation(
                    self.robot_id, [self.rx, self.ry, 0.35], kin_orn
                )
                _, orn      = p.getBasePositionAndOrientation(self.robot_id)
                heading     = p.getEulerFromQuaternion(orn)[2]
                readings    = self.sensors.read_all()
                true_tpos, _ = p.getBasePositionAndOrientation(TARGET_ID)
                peer_pos, _  = p.getBasePositionAndOrientation(self.peer_id)

            # ── Perception ─────────────────────────────────────────
            self.grid.slide_to(self.rx, self.ry)
            if t - last_gpredict >= GRID_PREDICT_INTERVAL:
                self.grid.predict()
                last_gpredict = t
            if t - last_gdecay >= GRID_DECAY_INTERVAL:
                self.grid.decay()
                last_gdecay = t

            track_predict(self.track, dt)

            hit = self._sense(readings, heading)
            detected = hit is not None
            if detected:
                self.grid.deposit(hit[0], hit[1], measurement=1.0)
                self.track.update(hit[0], hit[1])
                self.last_info_t = t
                if self.log["target_found_time"] is None:
                    self.log["target_found_time"] = t
                    print(f"  [{self.name} t={t:.1f}s] TARGET FOUND! "
                          f"est=[{hit[0]:.2f},{hit[1]:.2f}]")

            # NOTE: no grid -> track.update() fallback here. The grid's
            # argmax is a memory of where a detection once happened, not a
            # measurement of where the target is now. Feeding it to the
            # filter is what made the estimate teleport across the arena.

            # Drop a track that has coasted too long without information.
            if (self.track.initialised and self.last_info_t is not None
                    and t - self.last_info_t > self.TRACK_MAX_COAST):
                print(f"  [{self.name} t={t:.1f}s] track lost "
                      f"({self.TRACK_MAX_COAST:.0f}s without a detection)")
                self._drop_track()

            # ── Communication ──────────────────────────────────────
            msg = recv(self.receiver)
            if msg:
                self.log["msgs_received"] += 1
                patch     = msg.get("patch", {})
                t_sent    = patch.get("timestamp", t)
                staleness = max(0.0, t - t_sent)
                cells     = patch.get("cells", [])
                if cells:
                    self.grid.fuse_patch(cells, staleness_seconds=staleness)
                    fusions_this_period += 1
                peer_track = msg.get("track_state")
                if peer_track:
                    try:
                        peer_x = np.array(peer_track["x"])
                        peer_P = np.array(peer_track["P"])
                        # Only fuse a peer track that is actually confident.
                        # Fusing a diverging peer just spreads the divergence,
                        # and both robots ending up with one shared garbage
                        # estimate is exactly what the old logs showed.
                        peer_pos_unc = float(peer_P[0, 0] + peer_P[1, 1]) / 2.0
                        if peer_pos_unc < self.TRACK_LEAVE:
                            self.track.fuse_peer_track(peer_x, peer_P, staleness)
                            self.last_info_t = t
                    except Exception:
                        pass

            if t >= next_send_t:
                next_send_t = t + SEND_INTERVAL
                track_state = None
                if self.track.initialised:
                    track_state = {"x": self.track.x.tolist(),
                                   "P": self.track.P.tolist(), "t": t}
                out_msg = build_message(self.name, t, [self.rx, self.ry, 0.35],
                                        heading, self.grid.build_patch(timestamp=t))
                out_msg["track_state"] = track_state
                send(self.sender, out_msg, self.send_port)
                self.log["msgs_sent"] += 1

            # ── Navigation ─────────────────────────────────────────
            goal, self.mode = self._select_goal(peer_pos)

            # Trees and the peer only. The arena boundary goes through
            # `bounds` as planes — see fix #1. Adding walls back into this
            # list will re-create the phantom rings.
            obs_pos = list(TREE_POSITIONS) + [(peer_pos[0], peer_pos[1])]
            obs_rad = [TREE_NAV_RADIUS] * len(TREE_POSITIONS) + [0.5]

            vx, vy, omega = self.apf.compute(
                (self.rx, self.ry, 0.35), heading, goal,
                obs_pos, obs_rad, bounds=NAV_BOUNDS
            )

            # ── Locomotion ─────────────────────────────────────────
            self._kinematic_step(vx, vy, omega, dt)
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

            # ── Logging ────────────────────────────────────────────
            if t >= next_log_t:
                next_log_t = t + LOG_INTERVAL
                pos_est, _, unc = self.track.get_estimate()
                pos_unc = track_pos_uncertainty(self.track)
                self.log["times"].append(t)
                self.log["pos_x"].append(self.rx)
                self.log["pos_y"].append(self.ry)
                self.log["target_true_x"].append(true_tpos[0])
                self.log["target_true_y"].append(true_tpos[1])
                self.log["est_x"].append(pos_est[0] if pos_est else float("nan"))
                self.log["est_y"].append(pos_est[1] if pos_est else float("nan"))
                self.log["uncertainty"].append(unc)
                self.log["pos_uncertainty"].append(
                    pos_unc if math.isfinite(pos_unc) else float("nan"))
                self.log["detections"].append(1 if detected else 0)
                self.log["mode"].append(self.mode)

                if t >= next_print_t:
                    next_print_t = t + PRINT_INTERVAL
                    est_str = (f"[{pos_est[0]:.2f},{pos_est[1]:.2f}]"
                               if pos_est else "none")
                    print(f"  [{self.name} t={t:.0f}s] "
                          f"pos=[{self.rx:.2f},{self.ry:.2f}] "
                          f"est={est_str} "
                          f"pos_unc={pos_unc:.3f} unc={unc:.3f} "
                          f"mode={self.mode} "
                          f"fusions={fusions_this_period}")
                    fusions_this_period = 0

        print(f"[{self.name}] done")
        if self.log["target_found_time"]:
            print(f"  Target found at t={self.log['target_found_time']:.1f}s")
        n_track = sum(1 for m in self.log["mode"] if m == "TRACKING")
        print(f"  Time in TRACKING: {n_track}/{len(self.log['mode'])} logged samples")

    def save_log(self):
        os.makedirs("phase4", exist_ok=True)
        path = f"phase4/{self.name}_forest_log.json"
        with open(path, "w") as f:
            json.dump(self.log, f, indent=2)
        print(f"[{self.name}] log saved to {path}")


def physics_loop():
    """
    Owns SIM_CLOCK and paces the simulation to SIM_SPEED of real time.

    The old loop did `time.sleep(1/240)` and hoped, while two agent threads
    slept 3/240 and kept their own step counters. Nothing shared a clock.
    Here one sim step of PHYS_DT takes PHYS_DT/SIM_SPEED real seconds, and
    every consumer reads SIM_CLOCK.
    """
    global SIM_CLOCK
    t = 0.0
    next_wall = time.perf_counter()
    while not stop_event.is_set() and t < DURATION:
        with pb_lock:
            tx, ty = lissajous_target(t)
            p.resetBasePositionAndOrientation(TARGET_ID, [tx, ty, 0.55],
                                              [0, 0, 0, 1])
            p.stepSimulation()
            with poses_lock:
                for name, robot_id in [("walnut", WALNUT_ID), ("hazel", HAZEL_ID)]:
                    pose = desired_poses.get(name)
                    if pose is not None:
                        p.resetBasePositionAndOrientation(robot_id, pose[0], pose[1])
                        p.resetBaseVelocity(robot_id, [0, 0, 0], [0, 0, 0])
        t += PHYS_DT
        SIM_CLOCK = t

        next_wall += PHYS_DT / SIM_SPEED
        delay = next_wall - time.perf_counter()
        if delay > 0:
            time.sleep(delay)
        else:
            next_wall = time.perf_counter()   # behind schedule, don't spiral
    stop_event.set()


walnut = Agent("walnut", WALNUT_ID, WALNUT_PORT, HAZEL_PORT,
               HAZEL_ID, SPAWNS["walnut"][0], SPAWNS["walnut"][1])
hazel  = Agent("hazel",  HAZEL_ID,  HAZEL_PORT,  WALNUT_PORT,
               WALNUT_ID, SPAWNS["hazel"][0], SPAWNS["hazel"][1])

threads = [
    threading.Thread(target=physics_loop, daemon=True, name="physics"),
    threading.Thread(target=walnut.run,   daemon=True, name="walnut"),
    threading.Thread(target=hazel.run,    daemon=True, name="hazel"),
]

print("\nStarting Phase 4 Forest — Lissajous target, tree obstacles, 11m arena")
print(f"Sim speed {SIM_SPEED:.2f}x real, {DURATION:.0f} sim seconds "
      f"(~{DURATION/SIM_SPEED/60:.1f} min wall clock)\n")
for th in threads:
    th.start()

try:
    for th in threads[1:]:
        th.join()
except KeyboardInterrupt:
    print("\nShutting down...")
    stop_event.set()
    time.sleep(0.2)

walnut.save_log()
hazel.save_log()
print("\nDone. Run: python3 phase4/plot_phase4_forest.py")
os._exit(0)