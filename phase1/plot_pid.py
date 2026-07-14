"""
Run this separately after pid_joint.py finishes.
No PyBullet context = no segfault.
"""
import numpy as np
import matplotlib.pyplot as plt

data = np.load('phase1/pid_data.npy', allow_pickle=True).item()

times   = data['times']
angles  = data['angles']
targets = data['targets']
torques = data['torques']
kp, ki, kd = data['kp'], data['ki'], data['kd']

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

ax1.plot(times, np.degrees(angles),  label='actual angle',  color='steelblue')
ax1.plot(times, np.degrees(targets), label='target angle',
         color='orange', linestyle='--')
ax1.set_ylabel('Joint angle (degrees)')
ax1.set_title(f'FR_upper_joint PID  —  Kp={kp}  Ki={ki}  Kd={kd}')
ax1.legend()
ax1.grid(True, alpha=0.3)

ax2.plot(times, torques, label='torque τ (Nm)', color='firebrick')
ax2.set_ylabel('Torque (Nm)')
ax2.set_xlabel('Time (s)')
ax2.legend()
ax2.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('phase1/pid_result.png', dpi=150)
plt.close()
print("Plot saved to phase1/pid_result.png")