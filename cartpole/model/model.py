import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from model import manipulator_eq
import sympy as sp
import numpy as np
from scipy import signal

# State and Input Symbols
cart_position, cart_velocity, pole_angle, pole_velocity = sp.symbols('cart_position cart_velocity pole_angle pole_velocity')
cart_force = sp.symbols('cart_force')

# Equilibrium Point
equilibrium_point = {
    cart_position: 0,
    cart_velocity: 0,
    pole_angle: 0,
    pole_velocity: 0,
    cart_force: 0,
}

_A_c_numeric = None
_B_c_numeric = None

# Assemble the nonlinear state derivative f(x, u) from the manipulator equation
def build_nonlinear_dynamics():
    # Implement here
    raise NotImplementedError

# Jacobian Linearization
def build_symbolic_jacobians():
    # Implement here
    raise NotImplementedError

# State Space
def get_state_space_matrices():
    # Implement here
    raise NotImplementedError

# Zero Order Hold
def get_discrete_state_space_matrices(C_c=None, dt=0.02):
    # Implement here
    raise NotImplementedError

if __name__ == "__main__":
    A_c, B_c = get_state_space_matrices()
    print("A_c matrix:\n", A_c)
    print("B_c matrix:\n", B_c)

    print(np.linalg.eigvals(A_c))
    print("ctrb rank:", np.linalg.matrix_rank(
    np.hstack([np.linalg.matrix_power(A_c, i) @ B_c for i in range(4)])))
