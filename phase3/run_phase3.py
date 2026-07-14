"""
Phase 3 — Single-process multi-threaded swarm with track filter.

Two-layer estimation per robot:
  Layer 1: PheromoneGrid — spatial memory of detection history
  Layer 2: TrackFilter   — persistent 4-state KF tracking target position+velocity

The track filter never resets to sigma²=1.0 between detections.
Instead it predicts forward using the velocity estimate, so uncertainty
grows slowly between observations and shrinks on each detection.
After several orbits the velocity estimate converges and uncertainty
stabilises at a low value — this is the learning behavior.

CI fusion now happens at both layers:
  - Grid patches fused via scalar CI per cell (as before)
  - Track states fused via matrix CI on 4x4 covariance (new)
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

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'phase2'))
from ir_sensor import IRSensorArray

from pheromone_grid import PheromoneGrid
from track_filter   import TrackFilter
from protocol import (build_message, send, recv,
                      make_sender, make_receiver,
                      WALNUT_PORT, HAZEL_PORT)

# ── Physics world ──────────────────────────────────────────────────
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setTimeStep(1/240)
p.loadURDF("plane.urdf")

WALNUT_ID = p.loadURDF("a1/a1.urdf",
                        basePosition=[-1.5, 0, 0.35],
                        useFixedBase=True)
HAZEL_ID  = p.loadURDF("a1/a1.urdf",
                        basePosition=[ 1.5, 0, 0.35],
                        useFixedBase=True)

target_col = p.createCollisionShape(p.GEOM_SPHERE, radius=0.25)
target_vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.25,
                                 rgbaColor=[1, 0, 0, 1])
TARGET_ID  = p.createMultiBody(baseMass=0,
                                baseCollisionShapeIndex=target_col,
                                baseVisualShapeIndex=target_vis,
                                basePosition=[0, 1.5, 0.3])

for y_pos in [2.5, -2.5]:
    wc = p.createCollisionShape(p.GEOM_BOX, halfExtents=[2.5, 0.05, 0.5])
    wv = p.createVisualShape(p.GEOM_BOX, halfExtents=[2.5, 0.05, 0.5],
                             rgbaColor=[0.6, 0.6, 0.6, 1])
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wc,
                      baseVisualShapeIndex=wv,
                      basePosition=[0, y_pos, 0.5])

print(f"World — Walnut={WALNUT_ID} Hazel={HAZEL_ID} Target={TARGET_ID}")

pb_lock    = threading.Lock()
stop_event = threading.Event()

STANDING = {1:0.0, 3:0.67, 4:-1.3,
            6:0.0, 8:0.67, 9:-1.3,
            11:0.0, 13:0.67, 14:-1.3,
            16:0.0, 18:0.67, 19:-1.3}

n_joints = p.getNumJoints(WALNUT_ID)
for robot_id in [WALNUT_ID, HAZEL_ID]:
    for idx, angle in STANDING.items():
        p.resetJointState(robot_id, idx, angle, 0.0)

revolute_indices = [i for i in range(n_joints)
                    if p.getJointInfo(WALNUT_ID, i)[2] != 4]


class Agent:
    def __init__(self, name, robot_id, listen_port, send_port):
        self.name        = name
        self.robot_id    = robot_id
        self.listen_port = listen_port
        self.send_port   = send_port
        self.N_SENSORS   = 8

        # Layer 1: pheromone grid
        self.grid = PheromoneGrid(
            width_m=8.0, height_m=8.0, resolution=0.15,
            Q=0.0005, R=0.04, decay_rate=0.0002,
            sigma_init=1.0, mu_thresh=0.05
        )

        # Layer 2: track filter — persistent 4-state KF
        # q_pos/q_vel: how erratic is the target?
        # r_pos: how much do we trust the grid's best estimate?
        # Small Q → uncertainty grows slowly between detections
        # After several orbits velocity estimate converges → sigma² stabilises
        self.track = TrackFilter(
            dt=1/240,
            q_pos=0.0001,    # target barely deviates from constant velocity
            q_vel=0.0005,    # velocity changes slowly
            r_pos=0.09,      # trust grid estimate to within ~0.3m
            init_sigma=0.5,
            init_v_sigma=0.8
        )

        self.sensor_angles = [2*math.pi*i/self.N_SENSORS
                              for i in range(self.N_SENSORS)]
        self.sender   = make_sender()
        self.receiver = make_receiver(listen_port)

        with pb_lock:
            self.sensors = IRSensorArray(
                robot_id=robot_id,
                n_sensors=self.N_SENSORS,
                radius=0.7, height=0.2,
                max_range=2.5, noise_std=0.02,
                target_id=TARGET_ID
            )

        self.log = {
            'times':         [],
            'target_true_x': [], 'target_true_y': [],
            'est_x':         [], 'est_y':         [],
            'speed_est':     [],
            'uncertainty':   [],   # track filter trace(P)/4 — never resets
            'grid_sigma2':   [],   # grid best-cell sigma² — for comparison
            'fusions':       [], 'detections':    [],
            'msgs_sent': 0,  'msgs_received': 0
        }
        self.grid_snaps = {}

    def run(self):
        DT         = 1/240
        DURATION   = 60.0
        STEPS      = int(DURATION / DT)
        SEND_EVERY = 24
        LOG_EVERY  = 240

        print(f"[{self.name}] starting")
        fusions_this_sec = 0

        for step in range(STEPS):
            if stop_event.is_set():
                break
            t = step * DT

            with pb_lock:
                for idx in revolute_indices:
                    p.setJointMotorControl2(
                        self.robot_id, idx,
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=STANDING.get(idx, 0.0),
                        positionGain=20.0, velocityGain=2.0, force=33.0
                    )
                pos, orn   = p.getBasePositionAndOrientation(self.robot_id)
                euler      = p.getEulerFromQuaternion(orn)
                heading    = euler[2]
                readings   = self.sensors.read_all()
                true_pos, _ = p.getBasePositionAndOrientation(TARGET_ID)

            # Grid: slide + predict + decay
            self.grid.slide_to(pos[0], pos[1])
            if step % 10 == 0:
                self.grid.predict()
            if step % 5 == 0:
                self.grid.decay()

            # Track filter: predict every step
            # This is what keeps sigma² from resetting —
            # even with no detection, the filter predicts forward
            # using the velocity estimate
            self.track.predict()

            # Deposit on detections and update track filter
            detected = False
            for i, r in enumerate(readings):
                if r['is_target'] and r['distance'] < 2.4:
                    bearing = heading + self.sensor_angles[i]
                    tx_est  = pos[0] + r['distance'] * math.cos(bearing)
                    ty_est  = pos[1] + r['distance'] * math.sin(bearing)

                    # Layer 1: deposit pheromone
                    self.grid.deposit(tx_est, ty_est, measurement=1.0)

                    # Layer 2: update track filter with this measurement
                    # The grid gave us a noisy position — feed it to KF
                    self.track.update(tx_est, ty_est)
                    detected = True

                    if step % LOG_EVERY == 0:
                        pos_est, vel_est, unc = self.track.get_estimate()
                        print(f"  [{self.name} t={t:.1f}s] DETECTED "
                              f"sensor={i} d={r['distance']:.2f}m "
                              f"est=[{tx_est:.2f},{ty_est:.2f}] "
                              f"v={self.track.get_speed():.2f}m/s "
                              f"unc={unc:.4f}")

            # Also update track from grid's best estimate when confident
            # (catches cases where sensor didn't fire this step but
            #  grid still has useful accumulated belief)
            if not detected:
                grid_est, grid_conf, grid_s2 = self.grid.best_target_estimate()
                if grid_est and grid_conf > 0.3:
                    self.track.update(grid_est[0], grid_est[1])

            # Receive peer message
            msg = recv(self.receiver)
            if msg:
                self.log['msgs_received'] += 1
                patch     = msg.get('patch', {})
                t_sent    = patch.get('timestamp', t)
                staleness = max(0.0, t - t_sent)
                cells     = patch.get('cells', [])

                # Fuse grid patches (as before)
                if cells:
                    self.grid.fuse_patch(cells, staleness_seconds=staleness)
                    fusions_this_sec += 1

                # Fuse peer track state via matrix CI
                peer_track = msg.get('track_state')
                if peer_track and self.track.initialised:
                    try:
                        peer_x = np.array(peer_track['x'])
                        peer_P = np.array(peer_track['P'])
                        self.track.fuse_peer_track(
                            peer_x, peer_P, staleness
                        )
                    except Exception:
                        pass

            # Broadcast
            if step % SEND_EVERY == 0:
                patch   = self.grid.build_patch(timestamp=t)
                # Include track state in message for peer CI fusion
                track_state = None
                if self.track.initialised:
                    track_state = {
                        'x': self.track.x.tolist(),
                        'P': self.track.P.tolist(),
                        't': t
                    }
                out_msg = build_message(
                    self.name, t, list(pos), heading, patch
                )
                out_msg['track_state'] = track_state
                send(self.sender, out_msg, self.send_port)
                self.log['msgs_sent'] += 1

            # Log
            if step % LOG_EVERY == 0:
                pos_est, vel_est, unc = self.track.get_estimate()
                _, grid_conf, grid_s2 = self.grid.best_target_estimate()

                self.log['times'].append(t)
                self.log['target_true_x'].append(true_pos[0])
                self.log['target_true_y'].append(true_pos[1])
                self.log['est_x'].append(pos_est[0] if pos_est else float('nan'))
                self.log['est_y'].append(pos_est[1] if pos_est else float('nan'))
                self.log['speed_est'].append(
                    self.track.get_speed() if self.track.initialised else 0.0
                )
                self.log['uncertainty'].append(unc)      # track KF — never resets
                self.log['grid_sigma2'].append(grid_s2)  # grid — for comparison
                self.log['fusions'].append(fusions_this_sec)
                self.log['detections'].append(1 if detected else 0)

                if step % (LOG_EVERY * 5) == 0:
                    est_str = (f"[{pos_est[0]:.2f},{pos_est[1]:.2f}]"
                               if pos_est else "none")
                    print(f"  [{self.name} t={t:.0f}s] "
                          f"est={est_str} "
                          f"speed={self.track.get_speed():.2f}m/s "
                          f"unc={unc:.4f} "
                          f"n_updates={self.track.n_updates} "
                          f"fusions={fusions_this_sec}")

                fusions_this_sec = 0

            if step % (LOG_EVERY * 5) == 0:
                mu_s, s2_s, ox, oy, res = self.grid.snapshot()
                self.grid_snaps[f"t{int(t)}"] = {
                    'mu': mu_s.tolist(), 'sigma2': s2_s.tolist(),
                    'origin_x': ox, 'origin_y': oy, 'res': res
                }

            time.sleep(DT)

        print(f"[{self.name}] done — "
              f"sent={self.log['msgs_sent']} "
              f"recv={self.log['msgs_received']} "
              f"updates={self.track.n_updates}")

    def save_log(self):
        os.makedirs('phase3', exist_ok=True)
        full_log = dict(self.log)
        full_log['grid_snaps'] = self.grid_snaps
        path = f'phase3/{self.name}_log.json'
        with open(path, 'w') as f:
            json.dump(full_log, f, indent=2)
        print(f"[{self.name}] log → {path}")


def physics_loop():
    t  = 0.0
    DT = 1/240
    while not stop_event.is_set():
        with pb_lock:
            tx = 2.0 * math.cos(0.5 * t)
            ty = 2.0 * math.sin(0.5 * t)
            p.resetBasePositionAndOrientation(
                TARGET_ID, [tx, ty, 0.3], [0,0,0,1]
            )
            p.stepSimulation()
        time.sleep(DT)
        t += DT


walnut = Agent('walnut', WALNUT_ID, WALNUT_PORT, HAZEL_PORT)
hazel  = Agent('hazel',  HAZEL_ID,  HAZEL_PORT,  WALNUT_PORT)

threads = [
    threading.Thread(target=physics_loop, daemon=True, name='physics'),
    threading.Thread(target=walnut.run,   daemon=True, name='walnut'),
    threading.Thread(target=hazel.run,    daemon=True, name='hazel'),
]

print("\nStarting all threads...\n")
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
print("\nDone. Run: python3 phase3/plot_phase3.py")
os._exit(0)