"""
Phase 4 — Navigation: Frontier-based exploration + APF collision avoidance.

FrontierNavigator: picks the FARTHEST unexplored cell from the pheromone
grid as the exploration goal. Farthest (not nearest) forces the robot to
commit to deep exploration rather than hovering at known boundaries.

APFNavigator: computes velocity from goal pull + obstacle push.

────────────────────────────────────────────────────────────────────
CHANGES vs the previous version
────────────────────────────────────────────────────────────────────
1. ARENA WALLS ARE NOW PLANES, NOT CIRCLES.
   Previously the caller passed each wall in as a *point* at the wall's
   midpoint with radius = half the arena width. The APF treats every
   obstacle as a circle, so the north wall became an invisible disk of
   radius 5.5m centred at (0, 5.5) — and its d0 repulsion shell was a
   RING slicing through the middle of the arena. Robots were pushed
   outward from the centre and parked at the equilibrium where that
   phantom force balanced attraction (x ≈ ±3.3). Worse, once past
   |x| ≈ 4 the `d = dist - radius` term went negative, the
   `if 0 < d < d0` guard failed, and repulsion vanished entirely — so
   the walls repelled in the centre of the arena and not at the walls.

   Walls are now handled by the `bounds` argument: four axis-aligned
   half-planes. Do NOT pass walls in via obstacle_positions any more.

2. NEGATIVE-DISTANCE DEAD ZONE FIXED.
   `if 0 < d < self.d0` meant a robot *inside* an obstacle's radius felt
   zero force and could sail straight through. Now d is clamped to a
   small positive floor so penetration produces maximum push.

3. STUCK DETECTION USES ACTUAL DISPLACEMENT, NOT COMMANDED SPEED.
   The caller clamps position to the arena in its own kinematic step, so
   a robot pinned against a wall still received a healthy commanded vx —
   `_stuck_count` never incremented and the escape behaviour never fired.
   We now measure how far the robot has really moved over a window.

4. REPULSION CAP LOWERED (3.0 → 2.0) and split into separate influence
   radii for point obstacles (d0) and arena bounds (d0_bounds). With
   k_att capped at 1.0, a cap of 3.0 meant repulsion could always beat
   attraction near anything.
"""
import numpy as np
import math
import random
from collections import deque


class FrontierNavigator:
    """
    Picks the farthest unexplored frontier cell from the pheromone grid.

    A frontier cell is: low mu (unexplored) AND high sigma2 (uncertain).
    Picking the FARTHEST cell rather than the nearest forces deep
    exploration — the robot commits to covering new territory rather
    than oscillating at the edge of already-seen space.

    Parameters:
        mu_low         — cells below this mu are considered unexplored
        s2_high        — cells above this sigma2 are considered uncertain
        min_dist       — minimum distance from robot to candidate frontier
        arrival_radius — how close before picking a new frontier
        arena_limit    — reject frontier cells outside |x|,|y| < arena_limit.
                         The sliding grid window extends past the walls and
                         those cells stay unexplored forever, so they would
                         always win the "farthest" contest and pin the robot
                         to the boundary.
    """

    def __init__(self, mu_low=0.05, s2_high=0.7,
                 min_dist=1.5, arrival_radius=0.5, arena_limit=None):
        self.mu_low          = mu_low
        self.s2_high         = s2_high
        self.min_dist        = min_dist
        self.arrival_radius  = arrival_radius
        self.arena_limit     = arena_limit
        self.current_frontier   = None
        self.steps_since_update = 0
        self.max_steps_on_frontier = 480   # replan if we never arrive

    def update(self, grid, robot_pos, peer_pos=None):
        """Returns the next exploration goal (wx, wy) in world frame."""
        rx, ry = robot_pos[0], robot_pos[1]
        self.steps_since_update += 1

        arrived = False
        if self.current_frontier is not None:
            fx, fy = self.current_frontier
            dist = math.hypot(rx - fx, ry - fy)
            arrived = (dist < self.arrival_radius or
                       self.steps_since_update > self.max_steps_on_frontier)

        if self.current_frontier is None or arrived:
            self.current_frontier = self._find_farthest_frontier(
                grid, rx, ry, peer_pos
            )
            self.steps_since_update = 0

        return self.current_frontier

    def _find_farthest_frontier(self, grid, rx, ry, peer_pos):
        """
        Scan grid for frontier cells, return the FARTHEST valid one.

        Score = distance from robot (reward exploration depth).
        Penalty: cells near the peer robot are deprioritised, which makes
        the two robots naturally partition the arena without explicit
        coordination.
        """
        mu, sigma2, ox, oy, res = grid.snapshot()
        rows, cols = np.where((mu < self.mu_low) & (sigma2 > self.s2_high))

        if len(rows) == 0:
            return self._random_escape(rx, ry)

        best_score = -1.0
        best_cell  = None

        for row, col in zip(rows, cols):
            wx = ox + (col + 0.5) * res
            wy = oy + (row + 0.5) * res

            if self.arena_limit is not None:
                if abs(wx) > self.arena_limit or abs(wy) > self.arena_limit:
                    continue

            d_robot = math.hypot(wx - rx, wy - ry)
            if d_robot < self.min_dist:
                continue

            score = d_robot
            if peer_pos is not None:
                d_peer = math.hypot(wx - peer_pos[0], wy - peer_pos[1])
                if d_peer < 1.5:
                    score *= 0.3

            if score > best_score:
                best_score = score
                best_cell  = (wx, wy)

        if best_cell is None:
            candidates = []
            for row, col in zip(rows, cols):
                wx = ox + (col + 0.5) * res
                wy = oy + (row + 0.5) * res
                if self.arena_limit is not None:
                    if abs(wx) > self.arena_limit or abs(wy) > self.arena_limit:
                        continue
                if math.hypot(wx - rx, wy - ry) > 0.3:
                    candidates.append((wx, wy))
            if candidates:
                return random.choice(candidates)
            return self._random_escape(rx, ry)

        return best_cell

    def _random_escape(self, rx, ry):
        """Grid fully explored (or nothing valid) — pick a random point."""
        angle = random.uniform(0, 2 * math.pi)
        dist  = random.uniform(2.0, 4.0)
        wx = rx + dist * math.cos(angle)
        wy = ry + dist * math.sin(angle)
        if self.arena_limit is not None:
            lim = self.arena_limit
            wx = max(-lim, min(lim, wx))
            wy = max(-lim, min(lim, wy))
        return (wx, wy)


class APFNavigator:
    """
    Artificial Potential Field navigation.

    Attractive: toward the goal the caller supplies (a frontier cell, a
                pheromone hotspot, or the track-filter estimate).
    Repulsive:  away from point obstacles within d0, and away from the
                arena boundary planes within d0_bounds.

    Parameters:
        k_att        — attractive gain (force is capped at this value)
        k_rep        — repulsive gain
        d0           — influence radius of point obstacles (trees, peer).
                       Keep this comfortably below the surface-to-surface
                       gap between adjacent obstacles, or their fields
                       merge and the robot cannot thread between them.
        d0_bounds    — influence radius of the arena boundary planes
        max_speed    — m/s
        max_omega    — rad/s
        rep_cap      — hard ceiling on any single repulsive magnitude.
                       Must be comparable to k_att or repulsion always wins.
        stuck_window — number of compute() calls over which displacement
                       is measured
        stuck_dist   — if the robot moves less than this (metres) across
                       the whole window, trigger a random escape
        stuck_steps, stuck_thresh — DEPRECATED, accepted and ignored for
                       backwards compatibility with run_phase4.py
    """

    def __init__(self, k_att=1.2, k_rep=2.0, d0=0.45, d0_bounds=0.8,
                 max_speed=0.3, max_omega=1.5, rep_cap=2.0,
                 stuck_window=240, stuck_dist=0.05,
                 stuck_steps=None, stuck_thresh=None):
        self.k_att     = k_att
        self.k_rep     = k_rep
        self.d0        = d0
        self.d0_bounds = d0_bounds
        self.max_speed = max_speed
        self.max_omega = max_omega
        self.rep_cap   = rep_cap

        # Distance floor. If the robot is inside an obstacle (d <= 0) we do
        # not want the force to vanish or flip sign — we want maximum push.
        self._d_floor = 0.05

        self.stuck_window = stuck_window
        self.stuck_dist   = stuck_dist
        self._pos_history = deque(maxlen=stuck_window)

        self._escape_vel   = None
        self._escape_steps = 0

    # ── Repulsion helper ───────────────────────────────────────────
    def _rep_magnitude(self, d, d0):
        """
        Standard APF repulsive magnitude, guarded against d <= 0.

            F = k_rep * (1/d - 1/d0) / d^2      for d < d0
            F = 0                               otherwise

        d is surface distance (centre distance minus radius). If the robot
        has penetrated the obstacle, d <= 0; we clamp to _d_floor so the
        force saturates at rep_cap instead of disappearing.
        """
        if d >= d0:
            return 0.0
        d_eff = max(d, self._d_floor)
        return min(self.k_rep * (1.0 / d_eff - 1.0 / d0) / (d_eff ** 2),
                   self.rep_cap)

    # ── Main ───────────────────────────────────────────────────────
    def compute(self, robot_pos, robot_heading, goal,
                obstacle_positions, obstacle_radii, bounds=None):
        """
        Returns (vx, vy, omega) world-frame velocity command.

        goal   : (gx, gy) or None
        bounds : half-width of the navigable square arena, or None.
                 Walls are modelled as four axis-aligned planes at
                 x = ±bounds and y = ±bounds. Do NOT also pass walls in
                 through obstacle_positions — that is the bug this
                 argument exists to fix.
        """
        rx, ry = robot_pos[0], robot_pos[1]

        # Track real displacement for stuck detection
        self._pos_history.append((rx, ry))

        if self._escape_steps > 0:
            self._escape_steps -= 1
            evx, evy = self._escape_vel
            omega = self._steer(robot_heading, rx + evx, ry + evy, rx, ry)
            return evx, evy, omega

        # ── Attractive force toward goal ───────────────────────────
        if goal is not None:
            gx, gy = goal[0], goal[1]
            dx, dy = gx - rx, gy - ry
            dist = math.hypot(dx, dy) + 1e-6
            f_att = min(self.k_att, self.k_att * dist)
            ax = f_att * dx / dist
            ay = f_att * dy / dist
        else:
            ax, ay = 0.0, 0.0

        # ── Repulsion from point obstacles (trees, peer robot) ─────
        rep_x, rep_y = 0.0, 0.0
        for (ox, oy), radius in zip(obstacle_positions, obstacle_radii):
            dx, dy = rx - ox, ry - oy
            n = math.hypot(dx, dy) + 1e-6
            mag = self._rep_magnitude(n - radius, self.d0)
            if mag > 0.0:
                rep_x += mag * dx / n
                rep_y += mag * dy / n

        # ── Repulsion from arena boundary PLANES ───────────────────
        # Each wall is a half-plane. Distance is a simple axis offset and
        # the push direction is the inward normal — no phantom rings.
        if bounds is not None:
            walls = (
                (bounds - rx, -1.0,  0.0),   # east  wall at x = +bounds
                (rx + bounds,  1.0,  0.0),   # west  wall at x = -bounds
                (bounds - ry,  0.0, -1.0),   # north wall at y = +bounds
                (ry + bounds,  0.0,  1.0),   # south wall at y = -bounds
            )
            for d, ux, uy in walls:
                mag = self._rep_magnitude(d, self.d0_bounds)
                if mag > 0.0:
                    rep_x += mag * ux
                    rep_y += mag * uy

        # ── Combine and clamp ──────────────────────────────────────
        vx, vy = ax + rep_x, ay + rep_y
        speed = math.hypot(vx, vy)
        if speed > self.max_speed:
            vx = vx / speed * self.max_speed
            vy = vy / speed * self.max_speed

        # ── Stuck detection from ACTUAL displacement ───────────────
        # Not from commanded speed: the caller clamps position to the arena,
        # so a pinned robot still reports a healthy vx.
        if len(self._pos_history) == self._pos_history.maxlen:
            x0, y0 = self._pos_history[0]
            if math.hypot(rx - x0, ry - y0) < self.stuck_dist:
                angle = random.uniform(0, 2 * math.pi)
                self._escape_vel = (
                    self.max_speed * 0.7 * math.cos(angle),
                    self.max_speed * 0.7 * math.sin(angle),
                )
                self._escape_steps = max(20, self.stuck_window // 4)
                self._pos_history.clear()

        omega = self._steer(robot_heading, rx + vx, ry + vy, rx, ry)
        return vx, vy, omega

    def _steer(self, heading, tx, ty, rx, ry):
        desired = math.atan2(ty - ry, tx - rx)
        err = desired - heading
        err = math.atan2(math.sin(err), math.cos(err))
        return float(np.clip(err * 2.0, -self.max_omega, self.max_omega))


class WaypointNavigator:
    """
    Deterministic waypoint patrol — kept for fixed-environment demos
    where the arena layout is known in advance (e.g. run_phase4.py).
    Not used in autonomous exploration (run_phase4_forest.py).
    """

    def __init__(self, waypoints, arrival_radius=0.4):
        self.waypoints      = waypoints
        self.arrival_radius = arrival_radius
        self.current_idx    = 0

    def update(self, robot_pos):
        rx, ry = robot_pos[0], robot_pos[1]
        wx, wy = self.waypoints[self.current_idx]
        if math.hypot(rx - wx, ry - wy) < self.arrival_radius:
            self.current_idx = (self.current_idx + 1) % len(self.waypoints)
            wx, wy = self.waypoints[self.current_idx]
        return (wx, wy)