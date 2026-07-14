"""Plot Phase 1C gait data — FR and FL hip flexion should be anti-phase."""
import numpy as np
import matplotlib.pyplot as plt

data = np.load('phase1/gait_data.npy', allow_pickle=True).item()
times      = data['times']
fr_upper_a = data['fr_upper_a']
fl_upper_a = data['fl_upper_a']
fr_upper_t = data['fr_upper_t']

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 6), sharex=True)

# Top: FR vs FL hip flexion — should be mirror images (anti-phase)
ax1.plot(times, np.degrees(fr_upper_a), color='steelblue', label='FR hip (actual)')
ax1.plot(times, np.degrees(fl_upper_a), color='firebrick', label='FL hip (actual)')
ax1.plot(times, np.degrees(fr_upper_t), color='steelblue',
         linestyle='--', alpha=0.4, label='FR hip (target)')
ax1.set_ylabel('Hip flexion (degrees)')
ax1.set_title('Phase 1C — Trot gait: FR vs FL hip flexion (should be anti-phase)')
ax1.legend()
ax1.grid(True, alpha=0.3)

# Bottom: FR hip flexion only — should be clean sinusoid
ax2.plot(times, np.degrees(fr_upper_a), color='steelblue', label='FR hip actual')
ax2.plot(times, np.degrees(fr_upper_t), color='orange',
         linestyle='--', label='FR hip target')
ax2.set_ylabel('FR hip flexion (degrees)')
ax2.set_xlabel('Time (s)')
ax2.set_title('FR hip tracking — actual vs target sinusoid')
ax2.legend()
ax2.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('phase1/gait_result.png', dpi=150)
plt.close()
print("Plot saved to phase1/gait_result.png")