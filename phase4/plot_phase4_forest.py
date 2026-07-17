"""
Phase 4 Forest — Analysis plots.
"""
import json, math, numpy as np, matplotlib.pyplot as plt, os

def load(path):
    if not os.path.exists(path):
        print(f"WARNING: {path} not found"); return None
    with open(path) as f: return json.load(f)

w = load("phase4/walnut_forest_log.json")
h = load("phase4/hazel_forest_log.json")
if w is None or h is None:
    print("Run phase4/run_phase4_forest.py first."); exit(1)

# Load tree positions from a saved file if available
ARENA = 5.5

fig, axes = plt.subplots(1, 2, figsize=(16, 8))

for ax, log, nm, col, sx, sy in [
    (axes[0], w, "Walnut", "steelblue", -3.0, 0.0),
    (axes[1], h, "Hazel",  "firebrick",  3.0, 0.0)
]:
    # Arena boundary
    rect = plt.Polygon([[-ARENA,-ARENA],[ARENA,-ARENA],
                         [ARENA,ARENA],[-ARENA,ARENA]],
                        fill=False, edgecolor="gray", linewidth=1.5,
                        linestyle="--")
    ax.add_patch(rect)

    # Robot path
    ax.plot(log["pos_x"], log["pos_y"], color=col, lw=0.8,
            alpha=0.7, label=f"{nm} path")

    # Start marker
    ax.plot(sx, sy, marker="^", color=col, ms=14, zorder=5,
            label=f"{nm} start")

    # True target path (Lissajous)
    t_arr = np.linspace(0, 120, 2000)
    lx = 3.8 * np.sin(0.08 * t_arr)
    ly = 3.8 * np.sin(0.16 * t_arr + math.pi/2)
    ax.plot(lx, ly, "k--", lw=1, alpha=0.4, label="Target path (Lissajous)")

    # Track estimates
    valid = [(x, y, u) for x, y, u in
             zip(log["est_x"], log["est_y"], log["uncertainty"])
             if not (math.isnan(x) or math.isnan(y))]
    if valid:
        xs, ys, us = zip(*valid)
        sc = ax.scatter(xs, ys, c=us, cmap="RdYlGn_r",
                        s=20, vmin=0, vmax=0.5, zorder=4,
                        alpha=0.8, label="Track estimate")

    # Target found marker
    ft = log.get("target_found_time")
    if ft is not None:
        idx = min(range(len(log["times"])),
                  key=lambda i: abs(log["times"][i] - ft))
        if idx < len(log["pos_x"]):
            ax.plot(log["pos_x"][idx], log["pos_y"][idx],
                    "*", color="gold", ms=20, zorder=6,
                    markeredgecolor="darkorange", markeredgewidth=1.5,
                    label=f"Target found t={ft:.0f}s")

    ax.set_xlim(-ARENA-0.5, ARENA+0.5)
    ax.set_ylim(-ARENA-0.5, ARENA+0.5)
    ax.set_aspect("equal")
    ax.set_title(f"{nm} — forest search path", fontsize=12)
    ax.set_xlabel("World X (m)"); ax.set_ylabel("World Y (m)")
    ax.legend(fontsize=8, loc="upper right")
    ax.grid(True, alpha=0.2)

plt.suptitle(
    "Phase 4 Forest — Walnut and Hazel searching for target\n"
    "Lissajous figure-eight target path  |  18 tree obstacles  |  11m x 11m arena",
    fontsize=11
)
plt.tight_layout()
plt.savefig("phase4/forest_search_paths.png", dpi=150)
plt.close()
print("Saved forest_search_paths.png")

# Uncertainty comparison
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 7), sharex=True)

ax1.plot(w["times"], w["uncertainty"], color="steelblue",
         label="Walnut track uncertainty", lw=1.2, alpha=0.9)
ax1.plot(h["times"], h["uncertainty"], color="firebrick",
         label="Hazel track uncertainty",  lw=1.2, alpha=0.9)
for log, col in [(w,"steelblue"),(h,"firebrick")]:
    ft = log.get("target_found_time")
    if ft:
        ax1.axvline(ft, color=col, linestyle="--", alpha=0.6, lw=1.5)
ax1.set_ylabel("Track uncertainty"); ax1.legend(fontsize=9)
ax1.set_title("Track filter uncertainty — Lissajous forest run\n"
              "Drops when target detected, rises when target moves away")
ax1.grid(True, alpha=0.3)

# Target true path x and y over time
t_arr = np.array(w["times"])
lx = 3.8 * np.sin(0.08 * t_arr)
ly = 3.8 * np.sin(0.16 * t_arr + math.pi/2)
ax2.plot(t_arr, lx, color="orange", label="Target X (true)", lw=1)
ax2.plot(t_arr, ly, color="purple", label="Target Y (true)", lw=1)
ax2.plot(w["times"], w["est_x"], color="steelblue",
         linestyle="--", label="Walnut est X", alpha=0.7, lw=0.8)
ax2.plot(w["times"], w["est_y"], color="steelblue",
         linestyle=":", label="Walnut est Y", alpha=0.7, lw=0.8)
ax2.set_ylabel("Position (m)"); ax2.set_xlabel("Time (s)")
ax2.set_title("Target true position vs Walnut estimate — Lissajous path")
ax2.legend(fontsize=8, ncol=2); ax2.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig("phase4/forest_uncertainty.png", dpi=150)
plt.close()
print("Saved forest_uncertainty.png")

print("\nexplorer.exe phase4/forest_search_paths.png")
print("explorer.exe phase4/forest_uncertainty.png")