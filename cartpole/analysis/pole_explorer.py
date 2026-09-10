import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import matplotlib.pyplot as plt
import model.model as model
from controller.pole_placement import PolePlacementController

# Set to "s" to enter continuous poles, or "z" to enter discrete poles directly
DESIGN_PLANE = "s"

# Cart Position Reference
CART_REFERENCE = 0.5

# Sample Period
DT = 0.02

# Desired Poles in the S Plane
S_POLES = []

# Desired Poles in the Z Plane
Z_POLES = []

C_out = np.array([[1.0, 0.0, 0.0, 0.0]])
A_c, B_c = model.get_state_space_matrices()
A_d, B_d = model.get_discrete_state_space_matrices(C_c=C_out, dt=DT)

OPEN_LOOP_POLES_S = np.linalg.eigvals(A_c)
OPEN_LOOP_POLES_Z = np.linalg.eigvals(A_d)

def get_s_poles():
    if DESIGN_PLANE == "s":
        return np.array(S_POLES, dtype=complex)
    elif DESIGN_PLANE == "z":
        return np.log(np.array(Z_POLES, dtype=complex)) / DT
    else:
        raise ValueError("DESIGN_PLANE must be 's' or 'z', got %r" % DESIGN_PLANE)

# Hand the poles to the course controller and take its gain back
def design_controller():
    return PolePlacementController(s_desired_poles=get_s_poles(), dt=DT)

# Simulate the discrete closed-loop recursion to the cart position reference
def simulate_discrete(K, Nbar, t_final=3.0, ref=CART_REFERENCE):
    Ad_cl = A_d - B_d @ K
    Bd_cl = (B_d @ Nbar).reshape(-1)
    n_steps = int(t_final / DT)
    t = np.arange(n_steps + 1) * DT
    x = np.zeros((4, n_steps + 1))
    for k in range(n_steps):
        x[:, k + 1] = Ad_cl @ x[:, k] + Bd_cl * ref
    return t, x

# Draw the S-plane pole map
def draw_s_plane(ax, s_poles):
    all_real = np.concatenate([OPEN_LOOP_POLES_S.real, s_poles.real])
    real_axis_max = max(1.0, float(np.max(all_real)) + 2.0)
    real_axis_min = min(-1.0, float(np.min(all_real)) - 2.0)
    ax.axvspan(0, real_axis_max, color="mistyrose", alpha=0.5)
    ax.axvline(0, color="black", linewidth=1.2, label="Stability boundary")
    ax.scatter(OPEN_LOOP_POLES_S.real, OPEN_LOOP_POLES_S.imag, color="red", marker="x", s=100, label="Open-loop")
    ax.scatter(s_poles.real, s_poles.imag, color="blue", marker="o", s=60, label="Closed-loop")
    all_imag = np.concatenate([OPEN_LOOP_POLES_S.imag, s_poles.imag])
    imag_axis_max = max(1.0, float(np.max(np.abs(all_imag))) * 1.3)
    ax.set_xlim(real_axis_min, real_axis_max)
    ax.set_ylim(-imag_axis_max, imag_axis_max)
    ax.set_title("S-Plane")
    ax.set_xlabel(r"Real Axis ($\sigma$)")
    ax.set_ylabel(r"Imaginary Axis ($j\omega$)")
    ax.grid(True, linestyle=":", alpha=0.6)
    ax.legend(loc="upper right", fontsize=8)

# Draw the Z-plane pole map
def draw_z_plane(ax, z_poles):
    circle_angle = np.linspace(0, 2 * np.pi, 200)
    plot_radius = max(1.6, float(np.max(np.abs(z_poles))) * 1.3, float(np.max(np.abs(OPEN_LOOP_POLES_Z))) * 1.3)
    ax.fill([-plot_radius, plot_radius, plot_radius, -plot_radius], [-plot_radius, -plot_radius, plot_radius, plot_radius], color="mistyrose", alpha=0.5, zorder=0)
    ax.fill(np.cos(circle_angle), np.sin(circle_angle), color="white", zorder=1)
    ax.plot(np.cos(circle_angle), np.sin(circle_angle), color="black", linewidth=1.2, label="Unit circle", zorder=2)
    ax.scatter(OPEN_LOOP_POLES_Z.real, OPEN_LOOP_POLES_Z.imag, color="red", marker="x", s=100, label="Open-loop", zorder=3)
    ax.scatter(z_poles.real, z_poles.imag, color="blue", marker="o", s=60, label="Closed-loop", zorder=3)
    ax.set_title("Z-Plane (T=%.3f s)" % DT)
    ax.set_xlabel("Real")
    ax.set_ylabel("Imaginary")
    ax.grid(True, linestyle=":", alpha=0.6, zorder=4)
    ax.set_xlim(-plot_radius, plot_radius)
    ax.set_ylim(-plot_radius, plot_radius)
    ax.set_aspect("equal")
    ax.legend(loc="upper right", fontsize=8)

def draw_cart_response(ax, t, x, ref):
    # Cart Response
    ax.plot(t, x[0, :], color="royalblue", linewidth=1.8, label="Cart Position (m)")
    ax.plot(t, x[1, :], color="darkorange", linewidth=1.8, label="Cart Velocity (m/s)")
    ax.axhline(ref, color="gray", linestyle="--", linewidth=0.8, label="Reference")
    ax.set_title("Cart Response")
    ax.set_xlabel("Time (s)")
    ax.grid(True, linestyle=":", alpha=0.6)
    ax.legend(loc="upper right", fontsize=8)

def draw_pole_response(ax, t, x):
    # Pole Response
    ax.plot(t, x[2, :], color="crimson", linewidth=1.8, label="Pole Angle (rad)")
    ax.plot(t, x[3, :], color="seagreen", linewidth=1.8, label="Pole Angular Velocity (rad/s)")
    ax.set_title("Pole Response")
    ax.set_xlabel("Time (s)")
    ax.grid(True, linestyle=":", alpha=0.6)
    ax.legend(loc="upper right", fontsize=8)

def build_figure():
    controller = design_controller()
    s_poles = np.array(controller.s_desired_poles, dtype=complex)
    z_poles = np.array(controller.z_desired_poles, dtype=complex)

    # Gain K
    K, Nbar = controller.K, controller.Nbar
    ref = CART_REFERENCE
    t, x = simulate_discrete(K, Nbar, ref=ref)

    fig, ((ax_s, ax_z), (ax_cart, ax_pole)) = plt.subplots(2, 2, figsize=(13, 9))

    draw_s_plane(ax_s, s_poles)
    draw_z_plane(ax_z, z_poles)
    draw_cart_response(ax_cart, t, x, ref)
    draw_pole_response(ax_pole, t, x)

    fig.suptitle(
        "Design Plane = %s | DT = %.3f s | K = %s | Nbar = %s"
        % (DESIGN_PLANE, DT, np.array2string(K, precision=3), np.array2string(Nbar, precision=3)),
        fontsize=9,
    )
    fig.tight_layout()
    return fig

fig = build_figure()

if __name__ == "__main__":
    plt.show()
