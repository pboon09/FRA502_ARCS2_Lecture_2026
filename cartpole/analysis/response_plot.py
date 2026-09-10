import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import matplotlib.pyplot as plt
import numpy as np
import model.model as model
from scipy.integrate import solve_ivp

A_c, B_c = model.get_state_space_matrices()
poles, _ = np.linalg.eig(A_c)

print("Poles of the system (continuous, s-plane):", poles)

u_force = 1.0
B_col = B_c[:, 0] if (B_c.ndim > 1 and B_c.shape[1] > 1) else B_c.reshape(-1)
input_term = B_col * u_force

def open_loop_dynamics(_t, x):
    return A_c @ x + input_term

t_span = (0, 1.5)
t_eval = np.linspace(0, 1.5, 300)

response = solve_ivp(open_loop_dynamics, t_span, np.zeros(4), t_eval=t_eval)

fig, (ax0, ax1, ax2) = plt.subplots(1, 3, figsize=(15, 5))

ax0.scatter(poles.real, poles.imag, color='red', marker='x', s=100, label='Poles')
ax0.axhline(0, color='black', linestyle='--', linewidth=0.8)
ax0.axvline(0, color='black', linestyle='--', linewidth=0.8)
ax0.set_title('S-Plane Pole Map')
ax0.set_xlabel(r'Real Axis ($\sigma$)')
ax0.set_ylabel(r'Imaginary Axis ($j\omega$)')
ax0.grid(True, which='both', linestyle=':', alpha=0.6)
ax0.legend()
ax0.axis('equal')

ax1.plot(response.t, response.y[0, :], label="Cart Position (m)", color="royalblue", linewidth=1.8)
ax1.plot(response.t, response.y[1, :], label="Cart Velocity (m/s)", color="skyblue", linestyle="--")
ax1.set_ylabel("Cart Profile")
ax1.grid(True, linestyle=":", alpha=0.6)
ax1.legend(loc="upper left")
ax1.set_title("Unstable Open-Loop Step Time Response")

ax2.plot(response.t, response.y[2, :], label="Pole Angle (rad)", color="crimson", linewidth=1.8)
ax2.plot(response.t, response.y[3, :], label="Pole Angular Velocity (rad/s)", color="lightcoral", linestyle="--")
ax2.set_xlabel("Time (seconds)")
ax2.set_ylabel("Pole Profile")
ax2.grid(True, linestyle=":", alpha=0.6)
ax2.legend(loc="upper left")

plt.tight_layout()
plt.show()
