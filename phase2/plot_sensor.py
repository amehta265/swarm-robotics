"""Plot Phase 2 sensor data."""
import numpy as np
import matplotlib.pyplot as plt

data = np.load('phase2/sensor_data.npy', allow_pickle=True).item()
times           = data['times']
sensor0_dist    = data['sensor0_dist']
target_detected = data['target_detected']

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 6), sharex=True)

ax1.plot(times, sensor0_dist, color='steelblue', linewidth=0.8,
         label='Sensor 0 distance (m)')
ax1.axhline(2.0, color='gray', linestyle=':', label='max range (2m)')
ax1.set_ylabel('Distance (m)')
ax1.set_title('Phase 2 — IR sensor readings (sensor 0, pointing +X)')
ax1.legend()
ax1.grid(True, alpha=0.3)

ax2.fill_between(times, target_detected, alpha=0.4, color='magenta',
                 label='target detected')
ax2.plot(times, target_detected, color='magenta', linewidth=0.8)
ax2.set_ylabel('Target detected (1=yes)')
ax2.set_xlabel('Time (s)')
ax2.set_title('Target beacon detection across all sensors')
ax2.set_ylim(-0.1, 1.5)
ax2.legend()
ax2.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('phase2/sensor_result.png', dpi=150)
plt.close()
print("Plot saved to phase2/sensor_result.png")