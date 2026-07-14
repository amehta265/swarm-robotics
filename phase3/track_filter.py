"""
Phase 3 — Track filter: 4-state Kalman filter for moving target tracking.

This runs on top of the pheromone grid. The grid provides spatial memory
of where detections happened. The track filter maintains a persistent
belief about where the target is RIGHT NOW, including a velocity estimate.

State vector: x = [px, py, vx, vy]
               position (m) and velocity (m/s) in world frame

Why 4 states and not 2?
───────────────────────
With only position [px, py], the predict step has no information about
which direction the target is moving. Uncertainty grows isotropically
every timestep and the filter has no way to predict ahead confidently.

Adding velocity [vx, vy] means:
  - After a few detections the filter estimates target speed and direction
  - Between detections it predicts "target was moving NE at 0.4m/s,
    so it's probably 0.4*dt metres NE of last known position"
  - Uncertainty grows much more slowly — only proportional to how much
    the target deviates from constant velocity (process noise Q)
  - After several orbits the velocity estimate converges and sigma²
    stabilizes at a low value rather than resetting each cycle

This is the constant velocity model — the simplest motion model that
captures directional information. It's used in radar tracking, drone
tracking, and autonomous vehicle perception.

Initialisation strategy:
  The filter starts uninitialised. On the first detection from the grid,
  it initialises with the detected position and zero velocity.
  After the second detection it can estimate velocity from displacement.
  From the third detection onward it runs the full predict-update cycle.
"""
import numpy as np


class TrackFilter:
    """
    4-state Kalman filter for tracking a moving target.

    Parameters:
        dt          — timestep in seconds (must match simulation DT)
        q_pos       — process noise on position (m²/step)
                      how much target deviates from constant velocity
        q_vel       — process noise on velocity (m²/s² per step)
                      how much target accelerates unpredictably
        r_pos       — measurement noise variance (m²)
                      how much we trust the pheromone grid's best estimate
        init_sigma  — initial position uncertainty when track starts (m)
        init_v_sigma— initial velocity uncertainty (m/s)
    """
    def __init__(self,
                 dt=1/240,
                 q_pos=0.001,
                 q_vel=0.005,
                 r_pos=0.25,
                 init_sigma=0.5,
                 init_v_sigma=1.0):

        self.dt = dt

        # State transition matrix F — constant velocity model
        # x_new = F @ x_old
        # [px]   [1  0  dt  0 ] [px]
        # [py] = [0  1  0   dt] [py]
        # [vx]   [0  0  1   0 ] [vx]
        # [vy]   [0  0  0   1 ] [vy]
        self.F = np.array([
            [1, 0, dt, 0 ],
            [0, 1, 0,  dt],
            [0, 0, 1,  0 ],
            [0, 0, 0,  1 ],
        ], dtype=np.float64)

        # Observation matrix H — we measure position only, not velocity
        # z = H @ x  →  z = [px, py]
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
        ], dtype=np.float64)

        # Process noise covariance Q
        # Models how much the target can deviate from constant velocity
        # Higher Q = target is more erratic = uncertainty grows faster
        self.Q = np.diag([q_pos, q_pos, q_vel, q_vel])

        # Measurement noise covariance R
        # Models how much we trust the position measurement from the grid
        # Higher R = less trust in sensor = filter relies more on prediction
        self.R = np.eye(2) * r_pos

        # Initial covariance for a fresh track
        self.P_init = np.diag([
            init_sigma**2,    # px uncertainty
            init_sigma**2,    # py uncertainty
            init_v_sigma**2,  # vx uncertainty
            init_v_sigma**2,  # vy uncertainty
        ])

        # State and covariance — None until first detection
        self.x = None    # [px, py, vx, vy]
        self.P = None    # 4×4 covariance matrix

        # Track health
        self.initialised   = False
        self.n_updates     = 0      # how many times updated from detections
        self.steps_since_update = 0  # steps since last measurement

        # History for analysis — store trace(P)/4 each logged step
        self.uncertainty_history = []

    # ── Initialisation ─────────────────────────────────────────────
    def initialise(self, px, py):
        """
        Start a fresh track at position (px, py) with zero velocity.
        Called on the first confident detection from the pheromone grid.
        """
        self.x = np.array([px, py, 0.0, 0.0], dtype=np.float64)
        self.P = self.P_init.copy()
        self.initialised   = True
        self.n_updates     = 1
        self.steps_since_update = 0

    # ── Predict step ───────────────────────────────────────────────
    def predict(self):
        """
        Propagate state forward one timestep using the motion model.

        This runs every simulation step, whether or not a detection arrives.
        The target keeps moving even when we can't see it — the filter
        predicts where it probably went based on its last known velocity.

        After this step:
          x_pred = F @ x          (position advances by velocity * dt)
          P_pred = F @ P @ F.T + Q (uncertainty grows — target may deviate)

        The growth in P is bounded by Q. With small Q, uncertainty grows
        slowly between detections. This is why sigma² no longer resets to 1.0.
        """
        if not self.initialised:
            return

        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        self.steps_since_update += 1

    # ── Update step ────────────────────────────────────────────────
    def update(self, px, py):
        """
        Fuse a new position measurement into the track.

        Called when the pheromone grid has a confident best estimate
        (mu > threshold at some cell).

        Kalman update equations:
          S = H P H.T + R            innovation covariance
          K = P H.T S^{-1}           Kalman gain
          x = x + K (z - H x)        state update
          P = (I - KH) P             covariance update

        The Kalman gain K balances prediction vs measurement:
          - If P is large (uncertain prediction) → K large → trust measurement
          - If R is large (noisy measurement)    → K small → trust prediction
          - After many updates P stabilises at a low value → K stabilises
        """
        if not self.initialised:
            self.initialise(px, py)
            return

        z = np.array([px, py], dtype=np.float64)

        # Innovation — difference between measurement and prediction
        y = z - self.H @ self.x

        # Innovation covariance
        S = self.H @ self.P @ self.H.T + self.R

        # Kalman gain
        K = self.P @ self.H.T @ np.linalg.inv(S)

        # State update — measurement pulls estimate toward z
        self.x = self.x + K @ y

        # Covariance update — uncertainty shrinks
        I = np.eye(4)
        self.P = (I - K @ self.H) @ self.P

        self.n_updates     += 1
        self.steps_since_update = 0

    # ── Query ──────────────────────────────────────────────────────
    def get_estimate(self):
        """
        Return current best estimate of target state.

        Returns:
            position   — (px, py) in world frame, or None if uninitialised
            velocity   — (vx, vy) in m/s, or None
            uncertainty — trace(P)/4 — average variance across all states
                          This is what we plot instead of per-cell sigma²
                          It never resets to 1.0 — grows slowly between
                          detections and shrinks on updates
        """
        if not self.initialised:
            return None, None, 1.0

        position    = (self.x[0], self.x[1])
        velocity    = (self.x[2], self.x[3])
        uncertainty = float(np.trace(self.P) / 4.0)

        return position, velocity, uncertainty

    def get_speed(self):
        """Return estimated target speed in m/s."""
        if not self.initialised:
            return 0.0
        return float(np.sqrt(self.x[2]**2 + self.x[3]**2))

    def is_confident(self, max_uncertainty=0.3):
        """True if track uncertainty is below threshold."""
        _, _, unc = self.get_estimate()
        return self.initialised and unc < max_uncertainty

    # ── CI fusion from peer track ──────────────────────────────────
    def fuse_peer_track(self, peer_x, peer_P, staleness_seconds, Q_drift=0.01):
        """
        Fuse a peer robot's track state into this track using CI.

        Same covariance intersection principle as the grid fusion:
        we don't know the correlation between our track and the peer's
        track (they share information through prior comms), so we use
        CI rather than naive fusion to avoid overconfidence.

        peer_x — peer's state vector [px, py, vx, vy]
        peer_P — peer's 4×4 covariance matrix
        staleness_seconds — how old the peer's state is
        """
        if not self.initialised:
            # Bootstrap our track from peer if we have nothing
            self.x = peer_x.copy()
            self.P = peer_P + np.eye(4) * Q_drift * staleness_seconds
            self.initialised = True
            self.n_updates = 1
            return

        # Apply staleness: inflate peer covariance
        P_peer_stale = peer_P + np.eye(4) * Q_drift * staleness_seconds

        # CI optimal omega — minimise trace of fused covariance
        # For matrices: solved numerically via simple line search
        best_omega = 0.5
        best_trace = float('inf')
        for omega in np.linspace(0.01, 0.99, 50):
            try:
                P_inv_self = np.linalg.inv(self.P)
                P_inv_peer = np.linalg.inv(P_peer_stale)
                P_fused_inv = omega * P_inv_self + (1-omega) * P_inv_peer
                P_fused = np.linalg.inv(P_fused_inv)
                tr = np.trace(P_fused)
                if tr < best_trace:
                    best_trace = tr
                    best_omega = omega
            except np.linalg.LinAlgError:
                continue

        # Apply optimal CI fusion
        try:
            P_inv_self = np.linalg.inv(self.P)
            P_inv_peer = np.linalg.inv(P_peer_stale)
            P_fused_inv = best_omega * P_inv_self + (1-best_omega) * P_inv_peer
            P_fused = np.linalg.inv(P_fused_inv)
            x_fused = P_fused @ (best_omega * P_inv_self @ self.x
                                + (1-best_omega) * P_inv_peer @ peer_x)
            self.x = x_fused
            self.P = P_fused
        except np.linalg.LinAlgError:
            pass  # keep current state if fusion fails numerically