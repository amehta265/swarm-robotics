"""Phase 4 plots - robot trajectories, search coverage, target discovery."""
import json, math, numpy as np, matplotlib.pyplot as plt, os

def load(path):
    if not os.path.exists(path):
        print(f"WARNING: {path} not found"); return None
    with open(path) as f: return json.load(f)

w = load('phase4/walnut_log.json')
h = load('phase4/hazel_log.json')
if w is None or h is None:
    print("Run phase4/run_phase4.py first."); exit(1)

fig, axes = plt.subplots(1, 2, figsize=(14, 7))

for ax, log, nm, col, start in [
    (axes[0], w, 'Walnut', 'steelblue', (-1.5, 0)),
    (axes[1], h, 'Hazel',  'firebrick', ( 1.5, 0))
]:
    # Robot path
    ax.plot(log['pos_x'], log['pos_y'], color=col, lw=0.8,
            alpha=0.6, label=f'{nm} path')
    # Start position
    ax.plot(*start, marker='^', color=col, ms=12, zorder=5,
            label=f'{nm} start')
    # True target orbit
    ax.plot(log['target_true_x'], log['target_true_y'],
            'k--', lw=1, alpha=0.4, label='Target orbit')
    # Target estimates
    valid = [(x, y) for x, y in zip(log['est_x'], log['est_y'])
             if not (math.isnan(x) or math.isnan(y))]
    if valid:
        xs, ys = zip(*valid)
        ax.scatter(xs, ys, c='orange', s=15, zorder=4,
                   label='Track estimate', alpha=0.7)
    # Found marker
    ft = log.get('target_found_time')
    if ft is not None:
        idx = min(range(len(log['times'])),
                  key=lambda i: abs(log['times'][i] - ft))
        ax.plot(log['pos_x'][idx], log['pos_y'][idx],
                '*', color='gold', ms=18, zorder=6,
                label=f'Target found t={ft:.0f}s')

    # Arena boundary
    theta = np.linspace(0, 2*np.pi, 100)
    ax.set_xlim(-3.5, 3.5); ax.set_ylim(-3.5, 3.5)
    ax.set_aspect('equal')
    ax.set_title(f'{nm} — search path and target discovery')
    ax.set_xlabel('World X (m)'); ax.set_ylabel('World Y (m)')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

plt.suptitle('Phase 4 — Walnut and Hazel searching for hidden target\n'
             'No attractive force toward target — found by frontier search only',
             fontsize=11)
plt.tight_layout()
plt.savefig('phase4/search_paths.png', dpi=150)
plt.close()
print("Saved phase4/search_paths.png")

# Uncertainty over time
fig, ax = plt.subplots(figsize=(12, 4))
ax.plot(w['times'], w['uncertainty'], color='steelblue',
        label='Walnut track uncertainty', alpha=0.8)
ax.plot(h['times'], h['uncertainty'], color='firebrick',
        label='Hazel track uncertainty', alpha=0.8)
for log, col in [(w,'steelblue'),(h,'firebrick')]:
    ft = log.get('target_found_time')
    if ft:
        ax.axvline(ft, color=col, linestyle='--', alpha=0.6,
                   label=f'Target found t={ft:.0f}s')
ax.set_xlabel('Time (s)'); ax.set_ylabel('Track uncertainty')
ax.set_title('Uncertainty over time — drops when target detected')
ax.legend(); ax.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig('phase4/uncertainty.png', dpi=150)
plt.close()
print("Saved phase4/uncertainty.png")
print("\nexplorer.exe phase4/search_paths.png")
print("explorer.exe phase4/uncertainty.png")