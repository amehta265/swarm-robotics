"""
Phase 3 — Pheromone Grid with per-cell Distributed Kalman Filter.

Each cell in the grid maintains a Gaussian belief (mu, sigma²) about
the probability that the target is in that cell.

The grid is a sliding window centered on the robot's current position.
As the robot moves, the grid slides with it — O(W×H) memory always,
regardless of arena size.

Per-cell Kalman filter cycle each timestep:
  PREDICT → DEPOSIT → DECAY → (FUSE when peer patch arrives)

Fusion uses Covariance Intersection (CI) — the correct approach when
estimates may be correlated through prior information exchange.
Naive product-of-Gaussians fusion would double-count shared information
and produce falsely overconfident estimates (sigma² → 0).

Novel contribution: stigmergic pheromone field where each cell carries
a full Gaussian belief fused via CI across agents over lossy UDP comms.
"""
import numpy as np
import math


class PheromoneGrid:
    """
    A sliding-window 2D pheromone grid with per-cell Kalman filtering.

    Parameters:
        width_m    — physical width of the grid window in meters
        height_m   — physical height of the grid window in meters
        resolution — meters per cell (smaller = finer grid, more memory)
        Q          — process noise: how much uncertainty grows per timestep
                     (models target motion — higher Q = target moves faster)
        R          — measurement noise variance: how much to trust IR sensor
                     (higher R = less trust in sensor readings)
        decay_rate — pheromone evaporation rate per timestep (0-1)
        sigma_init — initial uncertainty for new cells (very high = no belief)
        mu_thresh  — minimum mu to include in broadcast patch (sparse comms)
    """
    def __init__(self,
                 width_m=6.0,
                 height_m=6.0,
                 resolution=0.1,
                 Q=0.001,
                 R=0.05,
                 decay_rate=0.002,
                 sigma_init=1.0,
                 mu_thresh=0.05):

        self.res        = resolution
        self.Q          = Q           # process noise
        self.R          = R           # measurement noise
        self.decay_rate = decay_rate
        self.sigma_init = sigma_init
        self.mu_thresh  = mu_thresh

        # Grid dimensions in cells
        self.W = int(width_m  / resolution)
        self.H = int(height_m / resolution)

        # Grid arrays — mu and sigma² for every cell
        # mu:     probability estimate that target is in this cell (0-1)
        # sigma²: uncertainty — high means we don't know, low means confident
        self.mu     = np.zeros((self.H, self.W), dtype=np.float32)
        self.sigma2 = np.full((self.H, self.W), sigma_init, dtype=np.float32)

        # World-frame position of the grid's top-left corner
        # Updated when the robot moves and the grid slides
        self.origin_x = -width_m  / 2.0   # start centered at world origin
        self.origin_y = -height_m / 2.0

        # Diffusion kernel — how pheromone spreads to neighbors
        # A 3×3 Gaussian kernel: center keeps most, neighbors get a little
        # This models the uncertainty about where exactly the target is
        self.diffusion_kernel = np.array([
            [0.05, 0.10, 0.05],
            [0.10, 0.40, 0.10],
            [0.05, 0.10, 0.05]
        ], dtype=np.float32)
        # Note: rows sum to 1.0 — no mu is created or destroyed by diffusion

    # ── Coordinate conversion ──────────────────────────────────────
    def world_to_cell(self, wx, wy):
        """
        Convert world-frame (x,y) position to grid cell (col, row).
        Returns None if the position is outside the current grid window.
        """
        col = int((wx - self.origin_x) / self.res)
        row = int((wy - self.origin_y) / self.res)
        if 0 <= col < self.W and 0 <= row < self.H:
            return col, row
        return None

    def cell_to_world(self, col, row):
        """Convert grid cell (col, row) to world-frame center position."""
        wx = self.origin_x + (col + 0.5) * self.res
        wy = self.origin_y + (row + 0.5) * self.res
        return wx, wy

    # ── Sliding window ─────────────────────────────────────────────
    def slide_to(self, robot_x, robot_y):
        """
        Slide the grid window to keep the robot centered.

        When the robot moves far enough that it's no longer in the
        middle third of the grid, we shift the grid. Cells that slide
        off the edge are lost. New cells entering get sigma²=sigma_init,
        mu=0 (maximum uncertainty — we know nothing about these areas yet).

        This keeps memory at O(W×H) regardless of how far the robot travels.
        """
        # Compute where robot currently is in cell coordinates
        robot_col = (robot_x - self.origin_x) / self.res
        robot_row = (robot_y - self.origin_y) / self.res

        # How far from center?
        shift_col = int(robot_col - self.W / 2)
        shift_row = int(robot_row - self.H / 2)

        # Only slide if robot has moved more than 20% of grid width
        threshold = 0.2
        if abs(shift_col) < self.W * threshold and abs(shift_row) < self.H * threshold:
            return   # robot still near center, no slide needed

        # Shift mu array
        self.mu     = np.roll(self.mu,     (-shift_row, -shift_col), axis=(0,1))
        self.sigma2 = np.roll(self.sigma2, (-shift_row, -shift_col), axis=(0,1))

        # Zero out the new cells that rolled in (they have no information)
        if shift_col > 0:
            self.mu[:, -shift_col:]  = 0.0
            self.sigma2[:, -shift_col:] = self.sigma_init
        elif shift_col < 0:
            self.mu[:, :-shift_col]  = 0.0
            self.sigma2[:, :-shift_col] = self.sigma_init

        if shift_row > 0:
            self.mu[-shift_row:, :]  = 0.0
            self.sigma2[-shift_row:, :] = self.sigma_init
        elif shift_row < 0:
            self.mu[:-shift_row, :]  = 0.0
            self.sigma2[:-shift_row, :] = self.sigma_init

        # Update origin
        self.origin_x += shift_col * self.res
        self.origin_y += shift_row * self.res

    # ── Kalman predict step ────────────────────────────────────────
    def predict(self):
        """
        KF predict step: time passes, target may have moved.

        Two things happen:
        1. Diffusion: mu spreads to neighboring cells via convolution.
           This models "the target was here last step, but it moves,
           so it's probably still near here but with some spread."

        2. Process noise: sigma² increases by Q everywhere.
           We're less certain about everything because time has passed
           and the target could have moved since our last observation.

        This is the F*sigma²*F^T + Q part of the Kalman predict step,
        simplified for scalar state (F=1, so F²=1).
        """
        from scipy.ndimage import convolve
        # Diffuse mu — spread belief to neighbors
        # mode='reflect' handles edges gracefully
        self.mu = convolve(self.mu, self.diffusion_kernel, mode='reflect')

        # Add process noise — uncertainty grows with time
        self.sigma2 = self.sigma2 + self.Q

        # Clip mu to valid probability range
        np.clip(self.mu, 0.0, 1.0, out=self.mu)

    # ── Kalman update step (deposit) ───────────────────────────────
    def deposit(self, world_x, world_y, measurement=1.0):
        """
        KF update step: sensor detected target at (world_x, world_y).

        This is the Kalman measurement update:
          K          = sigma²_pred / (sigma²_pred + R)
          mu_new     = mu_pred + K * (z - mu_pred)
          sigma²_new = (1 - K) * sigma²_pred

        Where z = measurement (1.0 = target definitely here),
        R = measurement noise variance.

        A large K (uncertain prediction, trusted sensor) means the
        measurement dominates. A small K means the prior dominates.
        """
        cell = self.world_to_cell(world_x, world_y)
        if cell is None:
            return   # target outside current grid window — ignore

        col, row = cell

        # Kalman gain for this cell
        K = self.sigma2[row, col] / (self.sigma2[row, col] + self.R)

        # Update mu — measurement pulls estimate toward z=1.0
        self.mu[row, col] = self.mu[row, col] + K * (measurement - self.mu[row, col])

        # Update sigma² — uncertainty shrinks after observation
        self.sigma2[row, col] = (1.0 - K) * self.sigma2[row, col]

    # ── Decay ──────────────────────────────────────────────────────
    def decay(self):
        """
        Pheromone evaporation: mu fades over time.

        This prevents the grid from accumulating stale beliefs indefinitely.
        Old detections fade, keeping the grid responsive to new information.
        Sigma² also grows slightly to reflect increasing staleness.
        """
        self.mu     *= (1.0 - self.decay_rate)
        self.sigma2 += self.decay_rate * 0.01   # small uncertainty growth

    # ── Covariance Intersection fusion ─────────────────────────────
    def fuse_patch(self, patch_cells, staleness_seconds, Q_drift=0.01):
        """
        Fuse a received grid patch using Covariance Intersection.

        Why CI and not naive product of Gaussians?
        ───────────────────────────────────────────
        Two robots that have been talking to each other have correlated
        beliefs — Walnut's current belief already contains things Hazel
        told him earlier. Naive fusion (product of Gaussians) treats them
        as independent and counts shared information twice, producing
        falsely overconfident estimates (sigma² → 0 over time).

        CI produces a conservative estimate that is ALWAYS correct
        regardless of the unknown correlation. It never overclaims certainty.

        The CI formula for scalar case:
          omega        = sigma²_peer / (sigma²_self + sigma²_peer)
          fused_sigma² = 1 / (omega/sigma²_self + (1-omega)/sigma²_peer)
          fused_mu     = fused_sigma² * (omega*mu_self/sigma²_self
                                       + (1-omega)*mu_peer/sigma²_peer)

        Staleness handling:
        ───────────────────
        Received beliefs are old by (staleness_seconds). We inflate the
        peer's sigma² proportionally before fusing — older information
        is treated as less certain. This is the lightweight timestamping
        approach: one multiplication per cell, negligible overhead.

        Parameters:
            patch_cells       — list of dicts from build_patch()
            staleness_seconds — how old this patch is (t_now - t_sent)
            Q_drift           — uncertainty growth per second of staleness
        """
        for cell_data in patch_cells:
            wx   = cell_data['wx']
            wy   = cell_data['wy']
            mu_p = cell_data['mu']
            s2_p = cell_data['sigma2']

            # Apply staleness penalty — inflate peer uncertainty
            # The older the information, the less we trust it
            s2_p_stale = s2_p + Q_drift * staleness_seconds

            # Map peer's world position to our local grid
            cell = self.world_to_cell(wx, wy)
            if cell is None:
                continue   # outside our current window — skip

            col, row = cell
            mu_s  = float(self.mu[row, col])
            s2_s  = float(self.sigma2[row, col])

            # Avoid division by zero
            if s2_s < 1e-9:   s2_s = 1e-9
            if s2_p_stale < 1e-9: s2_p_stale = 1e-9

            # CI optimal omega — minimizes trace of fused covariance
            # For scalar: omega = s2_peer / (s2_self + s2_peer)
            omega = s2_p_stale / (s2_s + s2_p_stale)

            # Information-form fusion
            info_self = omega       / s2_s
            info_peer = (1-omega)   / s2_p_stale
            fused_s2  = 1.0 / (info_self + info_peer)
            fused_mu  = fused_s2 * (info_self * mu_s + info_peer * mu_p)

            # Write back
            self.mu[row, col]     = float(np.clip(fused_mu, 0.0, 1.0))
            self.sigma2[row, col] = float(fused_s2)

    # ── Build broadcast patch ──────────────────────────────────────
    def build_patch(self, timestamp):
        """
        Extract a sparse patch for broadcasting to peers.

        Only cells where mu > mu_thresh are included.
        Each cell carries world-frame coordinates so the receiver
        can place it correctly in their own local grid frame.

        This is the key to "no shared memory" — instead of sharing
        the whole grid, we share only the informative cells, and we
        include enough information (world coords + timestamp) for the
        receiver to integrate it independently.

        Returns a dict ready for JSON serialization.
        """
        cells = []
        rows, cols = np.where(self.mu > self.mu_thresh)
        for row, col in zip(rows, cols):
            wx, wy = self.cell_to_world(col, row)
            cells.append({
                'wx':     round(float(wx), 3),
                'wy':     round(float(wy), 3),
                'mu':     round(float(self.mu[row, col]), 4),
                'sigma2': round(float(self.sigma2[row, col]), 4),
            })
        return {
            'timestamp': timestamp,
            'cells':     cells,
            'n_cells':   len(cells)
        }

    # ── Gradient query ─────────────────────────────────────────────
    def best_target_estimate(self):
        """
        Return the world-frame position of the cell with highest mu.

        This is used in Phase 4 for navigation — robots move toward
        the highest-probability region of their grid.

        Also returns confidence = mu_max and uncertainty = sigma² at that cell.
        """
        if self.mu.max() < self.mu_thresh:
            return None, 0.0, self.sigma_init   # no useful belief

        row, col = np.unravel_index(np.argmax(self.mu), self.mu.shape)
        wx, wy   = self.cell_to_world(col, row)
        return (wx, wy), float(self.mu[row, col]), float(self.sigma2[row, col])

    # ── Snapshot for plotting ──────────────────────────────────────
    def snapshot(self):
        """Return copies of mu and sigma² arrays for external plotting."""
        return self.mu.copy(), self.sigma2.copy(), \
               self.origin_x, self.origin_y, self.res