"""
Phase 3 — Analysis plots with two-layer estimation.

The key new plot is the uncertainty comparison:
  - Grid sigma²  (blue/red dashed) — resets to 1.0 between detections
  - Track KF unc (blue/red solid)  — never resets, decreases over time

This side-by-side comparison is the proof that Fix 1 worked:
the track filter learns the target's motion and maintains
persistent, improving confidence between detections.
"""
import json
import numpy as np
import matplotlib.pyplot as plt
import os

def load(path):
    if not os.path.exists(path):
        print(f"WARNING: {path} not found")
        return None
    with open(path) as f:
        return json.load(f)

w = load('phase3/walnut_log.json')
h = load('phase3/hazel_log.json')
if w is None or h is None:
    print("Run phase3/run_phase3.py first.")
    exit(1)

def dist_err(ex, ey, tx, ty):
    out = []
    for a, b, c, d in zip(ex, ey, tx, ty):
        if np.isnan(a) or np.isnan(b):
            out.append(float('nan'))
        else:
            out.append(np.sqrt((a-c)**2 + (b-d)**2))
    return out

times  = w['times']
htimes = h['times']
w_err  = dist_err(w['est_x'], w['est_y'], w['target_true_x'], w['target_true_y'])
h_err  = dist_err(h['est_x'], h['est_y'], h['target_true_x'], h['target_true_y'])

# ── 1. Trajectories ────────────────────────────────────────────────
fig, axes = plt.subplots(1, 2, figsize=(14, 6))
for ax, log, nm, rpos in [
    (axes[0], w, 'Walnut', (-1.5, 0)),
    (axes[1], h, 'Hazel',  ( 1.5, 0))
]:
    ax.plot(log['target_true_x'], log['target_true_y'],
            'k--', lw=1, label='True target', alpha=0.5)
    valid = [(x, y, c) for x, y, c in
             zip(log['est_x'], log['est_y'], log['uncertainty'])
             if not (np.isnan(x) or np.isnan(y))]
    if valid:
        xs, ys, us = zip(*valid)
        sc = ax.scatter(xs, ys, c=us, cmap='RdYlGn_r',
                        s=25, vmin=0, vmax=0.5, zorder=3,
                        label=f'{nm} track estimate')
    ax.plot(*rpos, 'b^' if nm=='Walnut' else 'r^',
            ms=12, label=nm, zorder=4)
    ax.set_title(f'{nm} — track estimate vs truth')
    ax.set_xlabel('World X (m)'); ax.set_ylabel('World Y (m)')
    ax.legend(); ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')
    ax.set_xlim(-3.5, 3.5); ax.set_ylim(-3.5, 3.5)
if 'sc' in dir():
    plt.colorbar(sc, ax=axes[1], label='Track uncertainty (lower=better)')
plt.tight_layout()
plt.savefig('phase3/trajectories.png', dpi=150); plt.close()
print("Saved trajectories.png")

# ── 2. Estimation error ────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(12, 4))
ax.plot(times,  w_err, color='steelblue', label='Walnut error (m)', alpha=0.8)
ax.plot(htimes, h_err, color='firebrick', label='Hazel error (m)',  alpha=0.8)
ax.set_xlabel('Time (s)'); ax.set_ylabel('Error (m)')
ax.set_title('Target estimation error over time')
ax.legend(); ax.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig('phase3/estimation_error.png', dpi=150); plt.close()
print("Saved estimation_error.png")

# ── 3. THE KEY PLOT — track KF vs grid sigma² ──────────────────────
# This shows Fix 1 working:
#   - Grid sigma² (dashed) resets to 1.0 between detections
#   - Track KF uncertainty (solid) never resets, trends downward
fig, axes = plt.subplots(2, 1, figsize=(14, 7), sharex=True)

ax1 = axes[0]
ax1.plot(times, w['grid_sigma2'], 'b--', alpha=0.5, lw=1,
         label='Walnut grid sigma² (resets between detections)')
ax1.plot(times, w['uncertainty'], 'b-',  alpha=0.9, lw=1.5,
         label='Walnut track KF uncertainty (never resets)')
ax1.plot(htimes, h['grid_sigma2'], 'r--', alpha=0.5, lw=1,
         label='Hazel grid sigma² (resets between detections)')
ax1.plot(htimes, h['uncertainty'], 'r-',  alpha=0.9, lw=1.5,
         label='Hazel track KF uncertainty (never resets)')
ax1.set_ylabel('Uncertainty')
ax1.set_title('Two-layer uncertainty comparison\n'
              'Dashed = grid sigma² (resets to 1.0).  '
              'Solid = track KF trace(P)/4 (should trend downward over time)')
ax1.legend(fontsize=9); ax1.grid(True, alpha=0.3)
ax1.set_ylim(-0.05, 1.1)

ax2 = axes[1]
ax2.plot(times,  w['speed_est'], color='steelblue',
         label='Walnut speed estimate (m/s)')
ax2.plot(htimes, h['speed_est'], color='firebrick',
         label='Hazel speed estimate (m/s)')
# True target speed: orbit radius 2m at angular velocity 0.5 rad/s → v = r*ω = 1.0 m/s
ax2.axhline(1.0, color='gray', linestyle=':', lw=1.5,
            label='True target speed (1.0 m/s)')
ax2.set_ylabel('Speed (m/s)')
ax2.set_xlabel('Time (s)')
ax2.set_title('Estimated target speed — should converge toward 1.0 m/s over time')
ax2.legend(fontsize=9); ax2.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('phase3/uncertainty_comparison.png', dpi=150); plt.close()
print("Saved uncertainty_comparison.png")

# ── 4. Grid snapshots ──────────────────────────────────────────────
w_snaps = w.get('grid_snaps', {})
keys    = sorted(w_snaps.keys(), key=lambda x: int(x[1:]))
if keys:
    last = keys[-1]
    h_snaps = h.get('grid_snaps', {})
    ws = w_snaps[last]
    hs = h_snaps.get(last, ws)

    fig, axes = plt.subplots(2, 2, figsize=(14, 12))
    for row, (snap, nm, rpos) in enumerate([
        (ws, 'Walnut', (-1.5, 0)),
        (hs, 'Hazel',  ( 1.5, 0))
    ]):
        mu  = np.array(snap['mu'])
        s2  = np.array(snap['sigma2'])
        ext = [snap['origin_x'],
               snap['origin_x'] + mu.shape[1]*snap['res'],
               snap['origin_y'],
               snap['origin_y'] + mu.shape[0]*snap['res']]
        im1 = axes[row,0].imshow(mu, origin='lower', cmap='hot',
                                  vmin=0, vmax=1, extent=ext)
        plt.colorbar(im1, ax=axes[row,0])
        axes[row,0].plot(*rpos,
                         'b^' if nm=='Walnut' else 'r^',
                         ms=10, label=nm)
        axes[row,0].set_title(f'{nm} pheromone mu (t={last[1:]}s)')
        axes[row,0].legend()
        im2 = axes[row,1].imshow(s2, origin='lower', cmap='Blues_r',
                                  extent=ext)
        plt.colorbar(im2, ax=axes[row,1])
        axes[row,1].set_title(f'{nm} sigma² (darker=more certain)')
    plt.suptitle('Grid snapshots — similar patterns = CI fusion working',
                 fontsize=11)
    plt.tight_layout()
    plt.savefig('phase3/grid_snapshots.png', dpi=150); plt.close()
    print("Saved grid_snapshots.png")

print("\nOpen with:")
print("  explorer.exe phase3/uncertainty_comparison.png  ← key plot")
print("  explorer.exe phase3/trajectories.png")
print("  explorer.exe phase3/estimation_error.png")
print("  explorer.exe phase3/grid_snapshots.png")