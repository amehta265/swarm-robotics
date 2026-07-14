"""
Phase 3 — Walnut agent.

Walnut is a fully independent process. He:
  - Connects to the shared physics server (simulation only)
  - Controls his own robot (body ID 1) via joint position control
  - Maintains a private PheromoneGrid — no other process can read it
  - Runs an IRSensorArray to detect the target and obstacles
  - Broadcasts sparse grid patches to Hazel over UDP
  - Receives Hazel's patches and fuses them via Covariance Intersection
  - Records data for post-run analysis and plotting

The key architectural point: Walnut never reads Hazel's grid directly.
He only learns about Hazel's beliefs through received UDP messages.
If those messages stop arriving (network dropout), Walnut continues
with his own grid — graceful degradation, not catastrophic failure.
"""
import time
import math
import numpy as np
import pybullet as p
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'phase2'))
from ir_sensor import IRSensorArray

from pheromone_grid import PheromoneGrid
from protocol import (build_message, send, recv,
                      make_sender, make_receiver,
                      WALNUT_PORT, HAZEL_PORT)

# ── Connect to physics server ──────────────────────────────────────
client = p.connect(p.SHARED_MEMORY)
if client < 0:
    print("ERROR: physics_server.py must be running first.")
    os._exit(1)
print(f"Walnut connected (client={client})")

# Body IDs — must match physics_server.py print output
WALNUT_ID = 1
HAZEL_ID  = 2
TARGET_ID = 3

# ── Joint setup ────────────────────────────────────────────────────
n_joints = p.getNumJoints(WALNUT_ID)
revolute_indices = [i for i in range(n_joints)
                    if p.getJointInfo(WALNUT_ID, i)[2] != 4]
standing_angles = {
    1: 0.0, 3: 0.67, 4: -1.3,
    6: 0.0, 8: 0.67, 9: -1.3,
    11: 0.0, 13: 0.67, 14: -1.3,
    16: 0.0, 18: 0.67, 19: -1.3,
}
for idx, angle in standing_angles.items():
    p.resetJointState(WALNUT_ID, idx, angle, 0.0)

# ── IR sensor array ────────────────────────────────────────────────
N_SENSORS = 8
sensors = IRSensorArray(
    robot_id=WALNUT_ID,
    n_sensors=N_SENSORS,
    radius=0.7,
    height=0.2,
    max_range=2.5,
    noise_std=0.02,
    target_id=TARGET_ID
)
sensor_angles = [2 * math.pi * i / N_SENSORS for i in range(N_SENSORS)]

# ── Pheromone grid ─────────────────────────────────────────────────
# Private to Walnut — Hazel cannot access this object directly
grid = PheromoneGrid(
    width_m=8.0,
    height_m=8.0,
    resolution=0.15,    # 15cm per cell — good balance of resolution vs speed
    Q=0.0005,           # process noise — target moves slowly
    R=0.04,             # measurement noise — IR sensor is fairly reliable
    decay_rate=0.001,   # slow decay — pheromone persists ~1000 steps
    sigma_init=1.0,
    mu_thresh=0.05
)

# ── Communication ──────────────────────────────────────────────────
sender   = make_sender()
receiver = make_receiver(WALNUT_PORT)
print(f"Walnut comms: listening={WALNUT_PORT}, sending={HAZEL_PORT}")

# ── Data recording for post-run plotting ──────────────────────────
log = {
    'times':              [],
    'target_true_x':      [],
    'target_true_y':      [],
    'walnut_est_x':       [],
    'walnut_est_y':       [],
    'walnut_confidence':  [],
    'walnut_sigma2':      [],
    'fusions_received':   [],
    'detections':         [],
    'msgs_received':      0,
    'msgs_sent':          0,
}

# ── Main loop ──────────────────────────────────────────────────────
DT         = 1/240
DURATION   = 30.0
STEPS      = int(DURATION / DT)
SEND_EVERY = 24    # broadcast at 10Hz
LOG_EVERY  = 240   # log every 1 second of sim time

print(f"\nWalnut running for {DURATION}s. Ctrl+C to stop early.\n")
fusions_this_second = 0

try:
    for step in range(STEPS):
        t = step * DT

        # ── 1. Hold standing pose ──────────────────────────────────
        for idx in revolute_indices:
            p.setJointMotorControl2(
                WALNUT_ID, idx,
                controlMode=p.POSITION_CONTROL,
                targetPosition=standing_angles.get(idx, 0.0),
                positionGain=20.0, velocityGain=2.0, force=33.0
            )

        # ── 2. Get own position ────────────────────────────────────
        pos, orn = p.getBasePositionAndOrientation(WALNUT_ID)
        euler    = p.getEulerFromQuaternion(orn)
        heading  = euler[2]

        # ── 3. Slide grid window to stay centered on Walnut ───────
        grid.slide_to(pos[0], pos[1])

        # ── 4. KF predict step — time has passed, target may move ─
        if step % 10 == 0:   # every 10 steps to save compute
            grid.predict()

        # ── 5. Read sensors ───────────────────────────────────────
        readings = sensors.read_all()

        # ── 6. KF update step — deposit on target detections ──────
        detected_this_step = False
        for i, r in enumerate(readings):
            if r['is_target'] and r['distance'] < 2.4:
                # Estimate world position of target from sensor reading
                bearing = heading + sensor_angles[i]
                tx_est  = pos[0] + r['distance'] * math.cos(bearing)
                ty_est  = pos[1] + r['distance'] * math.sin(bearing)

                # KF measurement update — deposit pheromone at estimated position
                grid.deposit(tx_est, ty_est, measurement=1.0)
                detected_this_step = True

                if step % LOG_EVERY == 0:
                    print(f"  [Walnut t={t:.1f}s] TARGET DETECTED "
                          f"sensor={i} d={r['distance']:.2f}m "
                          f"est=[{tx_est:.2f},{ty_est:.2f}]")

        # ── 7. Decay — pheromone evaporates ───────────────────────
        if step % 5 == 0:
            grid.decay()

        # ── 8. Receive from Hazel, fuse via CI ────────────────────
        msg = recv(receiver)
        if msg:
            log['msgs_received'] += 1
            patch      = msg.get('patch', {})
            t_sent     = patch.get('timestamp', t)
            staleness  = max(0.0, t - t_sent)
            cells      = patch.get('cells', [])

            if cells:
                grid.fuse_patch(cells, staleness_seconds=staleness)
                fusions_this_second += 1

        # ── 9. Broadcast to Hazel ──────────────────────────────────
        if step % SEND_EVERY == 0:
            patch   = grid.build_patch(timestamp=t)
            out_msg = build_message('walnut', t, list(pos), heading, patch)
            send(sender, out_msg, HAZEL_PORT)
            log['msgs_sent'] += 1

        # ── 10. Log for plotting ───────────────────────────────────
        if step % LOG_EVERY == 0:
            true_pos, _ = p.getBasePositionAndOrientation(TARGET_ID)
            est, conf, sig2 = grid.best_target_estimate()

            log['times'].append(t)
            log['target_true_x'].append(true_pos[0])
            log['target_true_y'].append(true_pos[1])
            log['walnut_est_x'].append(est[0] if est else float('nan'))
            log['walnut_est_y'].append(est[1] if est else float('nan'))
            log['walnut_confidence'].append(conf)
            log['walnut_sigma2'].append(sig2)
            log['fusions_received'].append(fusions_this_second)
            log['detections'].append(1 if detected_this_step else 0)

            if step % (LOG_EVERY * 5) == 0:
                est_str = f"[{est[0]:.2f},{est[1]:.2f}]" if est else "none"
                print(f"  [Walnut t={t:.1f}s] "
                      f"est={est_str} conf={conf:.3f} "
                      f"sigma2={sig2:.3f} "
                      f"fusions={fusions_this_second} "
                      f"patch_cells={patch['n_cells']}")

            fusions_this_second = 0

        # ── 11. Save grid snapshot every 5 seconds ─────────────────
        if step % (LOG_EVERY * 5) == 0:
            mu_snap, s2_snap, ox, oy, res = grid.snapshot()
            snap_key = f"grid_t{int(t)}"
            log[snap_key] = {
                'mu': mu_snap.tolist(),
                'sigma2': s2_snap.tolist(),
                'origin_x': ox, 'origin_y': oy, 'res': res
            }

        time.sleep(DT)

except KeyboardInterrupt:
    pass

# ── Save all log data ──────────────────────────────────────────────
import json
log_path = 'phase3/walnut_log.json'
os.makedirs('phase3', exist_ok=True)
with open(log_path, 'w') as f:
    json.dump(log, f, indent=2)
print(f"\nWalnut done. Log saved to {log_path}")
print(f"Messages sent: {log['msgs_sent']}  received: {log['msgs_received']}")
os._exit(0)