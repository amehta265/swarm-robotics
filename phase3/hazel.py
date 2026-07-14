"""
Phase 3 — Hazel agent.

Structurally identical to walnut.py. The symmetry is intentional —
in a real swarm, every agent runs the same code with different IDs.
Adding a third robot (Pistachio) would be another copy of this file.

Differences from walnut.py:
  - Controls body ID 2 (not 1)
  - Listens on HAZEL_PORT, sends to WALNUT_PORT
  - Logs to hazel_log.json
  - Starts at position [1.5, 0, 0.35] — opposite side of arena

Everything else — grid, sensors, KF, CI fusion — is identical.
This is the decentralized property: no agent is special.
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

client = p.connect(p.SHARED_MEMORY)
if client < 0:
    print("ERROR: physics_server.py must be running first.")
    os._exit(1)
print(f"Hazel connected (client={client})")

WALNUT_ID = 1
HAZEL_ID  = 2
TARGET_ID = 3

n_joints = p.getNumJoints(HAZEL_ID)
revolute_indices = [i for i in range(n_joints)
                    if p.getJointInfo(HAZEL_ID, i)[2] != 4]
standing_angles = {
    1: 0.0, 3: 0.67, 4: -1.3,
    6: 0.0, 8: 0.67, 9: -1.3,
    11: 0.0, 13: 0.67, 14: -1.3,
    16: 0.0, 18: 0.67, 19: -1.3,
}
for idx, angle in standing_angles.items():
    p.resetJointState(HAZEL_ID, idx, angle, 0.0)

N_SENSORS = 8
sensors = IRSensorArray(
    robot_id=HAZEL_ID,
    n_sensors=N_SENSORS,
    radius=0.7,
    height=0.2,
    max_range=2.5,
    noise_std=0.02,
    target_id=TARGET_ID
)
sensor_angles = [2 * math.pi * i / N_SENSORS for i in range(N_SENSORS)]

# Hazel's private grid — Walnut cannot access this
grid = PheromoneGrid(
    width_m=8.0,
    height_m=8.0,
    resolution=0.15,
    Q=0.0005,
    R=0.04,
    decay_rate=0.001,
    sigma_init=1.0,
    mu_thresh=0.05
)

# Note ports are swapped vs walnut.py
sender   = make_sender()
receiver = make_receiver(HAZEL_PORT)
print(f"Hazel comms: listening={HAZEL_PORT}, sending={WALNUT_PORT}")

log = {
    'times':             [],
    'target_true_x':     [],
    'target_true_y':     [],
    'hazel_est_x':       [],
    'hazel_est_y':       [],
    'hazel_confidence':  [],
    'hazel_sigma2':      [],
    'fusions_received':  [],
    'detections':        [],
    'msgs_received':     0,
    'msgs_sent':         0,
}

DT         = 1/240
DURATION   = 30.0
STEPS      = int(DURATION / DT)
SEND_EVERY = 24
LOG_EVERY  = 240

print(f"\nHazel running for {DURATION}s. Ctrl+C to stop early.\n")
fusions_this_second = 0

try:
    for step in range(STEPS):
        t = step * DT

        for idx in revolute_indices:
            p.setJointMotorControl2(
                HAZEL_ID, idx,
                controlMode=p.POSITION_CONTROL,
                targetPosition=standing_angles.get(idx, 0.0),
                positionGain=20.0, velocityGain=2.0, force=33.0
            )

        pos, orn = p.getBasePositionAndOrientation(HAZEL_ID)
        euler    = p.getEulerFromQuaternion(orn)
        heading  = euler[2]

        grid.slide_to(pos[0], pos[1])

        if step % 10 == 0:
            grid.predict()

        readings = sensors.read_all()

        detected_this_step = False
        for i, r in enumerate(readings):
            if r['is_target'] and r['distance'] < 2.4:
                bearing = heading + sensor_angles[i]
                tx_est  = pos[0] + r['distance'] * math.cos(bearing)
                ty_est  = pos[1] + r['distance'] * math.sin(bearing)
                grid.deposit(tx_est, ty_est, measurement=1.0)
                detected_this_step = True

                if step % LOG_EVERY == 0:
                    print(f"  [Hazel t={t:.1f}s] TARGET DETECTED "
                          f"sensor={i} d={r['distance']:.2f}m "
                          f"est=[{tx_est:.2f},{ty_est:.2f}]")

        if step % 5 == 0:
            grid.decay()

        msg = recv(receiver)
        if msg:
            log['msgs_received'] += 1
            patch     = msg.get('patch', {})
            t_sent    = patch.get('timestamp', t)
            staleness = max(0.0, t - t_sent)
            cells     = patch.get('cells', [])
            if cells:
                grid.fuse_patch(cells, staleness_seconds=staleness)
                fusions_this_second += 1

        if step % SEND_EVERY == 0:
            patch   = grid.build_patch(timestamp=t)
            out_msg = build_message('hazel', t, list(pos), heading, patch)
            send(sender, out_msg, WALNUT_PORT)
            log['msgs_sent'] += 1

        if step % LOG_EVERY == 0:
            true_pos, _ = p.getBasePositionAndOrientation(TARGET_ID)
            est, conf, sig2 = grid.best_target_estimate()

            log['times'].append(t)
            log['target_true_x'].append(true_pos[0])
            log['target_true_y'].append(true_pos[1])
            log['hazel_est_x'].append(est[0] if est else float('nan'))
            log['hazel_est_y'].append(est[1] if est else float('nan'))
            log['hazel_confidence'].append(conf)
            log['hazel_sigma2'].append(sig2)
            log['fusions_received'].append(fusions_this_second)
            log['detections'].append(1 if detected_this_step else 0)

            if step % (LOG_EVERY * 5) == 0:
                est_str = f"[{est[0]:.2f},{est[1]:.2f}]" if est else "none"
                print(f"  [Hazel t={t:.1f}s] "
                      f"est={est_str} conf={conf:.3f} "
                      f"sigma2={sig2:.3f} "
                      f"fusions={fusions_this_second} "
                      f"patch_cells={patch['n_cells']}")

            fusions_this_second = 0

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

import json
log_path = 'phase3/hazel_log.json'
os.makedirs('phase3', exist_ok=True)
with open(log_path, 'w') as f:
    json.dump(log, f, indent=2)
print(f"\nHazel done. Log saved to {log_path}")
print(f"Messages sent: {log['msgs_sent']}  received: {log['msgs_received']}")
os._exit(0)