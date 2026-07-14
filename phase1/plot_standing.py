import numpy as np
import matplotlib.pyplot as plt

data   = np.load('phase1/standing_data.npy', allow_pickle=True).item()
times  = data['times']
angles = data['angles']
base_zs = data['base_zs']
target = data['target']

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

ax1.plot(times, np.degrees(angles), color='steelblue', label='FR_upper actual')
ax1.axhline(np.degrees(target), color='orange',
            linestyle='--', label=f'final target {np.degrees(target):.1f}°')
ax1.set_ylabel('Joint angle (degrees)')
ax1.set_title('Phase 1B — Standing pose (ramped targets, 12 joints)')
ax1.legend()
ax1.grid(True, alpha=0.3)

ax2.plot(times, base_zs, color='firebrick', label='base height (m)')
ax2.axhline(0.27, color='gray', linestyle=':', label='expected standing height')
ax2.set_ylabel('Base Z (m)')
ax2.set_xlabel('Time (s)')
ax2.legend()
ax2.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('phase1/standing_result.png', dpi=150)
plt.close()
print("Plot saved to phase1/standing_result.png")