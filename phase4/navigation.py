"""
Phase 4 - Navigation: Frontier-based search + APF collision avoidance.

No attractive force toward the target - target position is unknown
until a sensor actually detects it. Robots search frontiers blindly.
"""
import numpy as np
import math
import random


class FrontierNavigator:
    """
    Picks the next unexplored frontier cell from the pheromone grid.
    Frontier = low mu (never detected) AND high sigma2 (peer hasn't told us either).
    Robots naturally spread out - once Walnut heads somewhere and his patches
    reach Hazel via CI fusion, Hazel's frontier list excludes that region.
    Emergent coverage without explicit coordination.
    """
    def __init__(self, mu_low=0.05, s2_high=0.8, arrival_radius=0.4):
        self.mu_low = mu_low
        self.s2_high = s2_high
        self.arrival_radius = arrival_radius
        self.current_frontier = None

    def update(self, grid, robot_pos):
        rx, ry = robot_pos[0], robot_pos[1]
        if self.current_frontier is not None:
            fx, fy = self.current_frontier
            if math.sqrt((rx - fx)**2 + (ry - fy)**2) < self.arrival_radius:
                self.current_frontier = None
        if self.current_frontier is None:
            self.current_frontier = self._find_nearest(grid, rx, ry)
        return self.current_frontier

    def _find_nearest(self, grid, rx, ry):
        mu, sigma2, ox, oy, res = grid.snapshot()
        rows, cols = np.where((mu < self.mu_low) & (sigma2 > self.s2_high))
        candidates = []
        for row, col in zip(rows, cols):
            wx = ox + (col + 0.5) * res
            wy = oy + (row + 0.5) * res
            if math.sqrt((wx - rx)**2 + (wy - ry)**2) > 0.5:
                candidates.append((wx, wy))
        if not candidates:
            angle = random.uniform(0, 2 * math.pi)
            return (rx + 1.5 * math.cos(angle), ry + 1.5 * math.sin(angle))
        return min(candidates, key=lambda c: (c[0]-rx)**2 + (c[1]-ry)**2)


class APFNavigator:
    """
    Artificial Potential Field navigation.
    Attractive: toward frontier (NOT toward target - target is unknown)
    Repulsive: away from obstacles within influence radius d0
    Escape: random kick when stuck for stuck_steps timesteps
    """
    def __init__(self, k_att=0.8, k_rep=1.5, d0=0.8,
                 max_speed=0.25, max_omega=1.2,
                 stuck_steps=120, stuck_thresh=0.03):
        self.k_att = k_att
        self.k_rep = k_rep
        self.d0 = d0
        self.max_speed = max_speed
        self.max_omega = max_omega
        self.stuck_steps = stuck_steps
        self.stuck_thresh = stuck_thresh
        self._stuck_count = 0
        self._escape_vel = None
        self._escape_steps = 0

    def compute(self, robot_pos, robot_heading, frontier,
                obstacle_positions, obstacle_radii):
        rx, ry = robot_pos[0], robot_pos[1]

        if self._escape_steps > 0:
            self._escape_steps -= 1
            evx, evy = self._escape_vel
            omega = self._steer(robot_heading, rx+evx, ry+evy, rx, ry)
            return evx, evy, omega

        if frontier is not None:
            fx, fy = frontier
            dx, dy = fx - rx, fy - ry
            dist = math.sqrt(dx**2 + dy**2) + 1e-6
            f_att = min(self.k_att, self.k_att * dist)
            ax = f_att * dx / dist
            ay = f_att * dy / dist
        else:
            ax, ay = 0.0, 0.0

        rep_x, rep_y = 0.0, 0.0
        for (ox, oy), radius in zip(obstacle_positions, obstacle_radii):
            dx, dy = rx - ox, ry - oy
            d = math.sqrt(dx**2 + dy**2) - radius + 1e-6
            if 0 < d < self.d0:
                mag = min(self.k_rep * (1.0/d - 1.0/self.d0) / (d**2), 3.0)
                n = math.sqrt(dx**2 + dy**2) + 1e-6
                rep_x += mag * dx / n
                rep_y += mag * dy / n

        vx, vy = ax + rep_x, ay + rep_y
        speed = math.sqrt(vx**2 + vy**2)
        if speed > self.max_speed:
            vx = vx / speed * self.max_speed
            vy = vy / speed * self.max_speed
            speed = self.max_speed

        if speed < self.stuck_thresh:
            self._stuck_count += 1
            if self._stuck_count > self.stuck_steps:
                angle = random.uniform(0, 2 * math.pi)
                self._escape_vel = (
                    self.max_speed * 0.7 * math.cos(angle),
                    self.max_speed * 0.7 * math.sin(angle)
                )
                self._escape_steps = 80
                self._stuck_count = 0
        else:
            self._stuck_count = 0

        omega = self._steer(robot_heading, rx+vx, ry+vy, rx, ry)
        return vx, vy, omega

    def _steer(self, heading, tx, ty, rx, ry):
        desired = math.atan2(ty - ry, tx - rx)
        err = desired - heading
        while err >  math.pi: err -= 2 * math.pi
        while err < -math.pi: err += 2 * math.pi
        return float(np.clip(err * 2.0, -self.max_omega, self.max_omega))