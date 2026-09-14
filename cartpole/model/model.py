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
    state_vector = sp.Matrix([cart_position, pole_angle, cart_velocity, pole_velocity])
    input_vector = sp.Matrix([cart_force])

    M = sp.Matrix(manipulator_eq.mass_matrix(pole_angle))
    C = sp.Matrix(manipulator_eq.coriolis_matrix(pole_angle, pole_velocity))
    G = sp.Matrix(manipulator_eq.gravity_vector(pole_angle))

    velocity = sp.Matrix([cart_velocity, pole_velocity])

    tau = sp.Matrix([cart_force, 0])

    nonlinear_term = M.inv() * (tau - C * velocity - G)

    f = sp.Matrix([cart_velocity, pole_velocity, nonlinear_term[0], nonlinear_term[1]])

    return f, state_vector, input_vector

# Jacobian Linearization
def build_symbolic_jacobians():
    f, state_vector, input_vector = build_nonlinear_dynamics()
    A_symbolic = f.jacobian(state_vector)
    B_symbolic = f.jacobian(input_vector)
    return A_symbolic, B_symbolic

# State Space
def get_state_space_matrices():
    global _A_c_numeric, _B_c_numeric
    if _A_c_numeric is None or _B_c_numeric is None:
        A_symbolic, B_symbolic = build_symbolic_jacobians()
        _A_c_numeric = np.array(A_symbolic.subs(equilibrium_point)).astype(np.float64)
        _B_c_numeric = np.array(B_symbolic.subs(equilibrium_point)).astype(np.float64)
    return _A_c_numeric, _B_c_numeric

# Zero Order Hold
def get_discrete_state_space_matrices(C_c=None, dt=0.02):
    A_c, B_c = get_state_space_matrices()
    A_d, B_d, _, _, _ = signal.cont2discrete(
        (A_c, B_c, C_c, np.zeros((C_c.shape[0], B_c.shape[1]))),
        dt=dt,
        method='zoh'
    )
    return A_d, B_d

if __name__ == "__main__":
    A_c, B_c = get_state_space_matrices()
    print("A_c matrix:\n", A_c)
    print("B_c matrix:\n", B_c)

    print(np.linalg.eigvals(A_c)) #x xdot theta thetadot
    print("ctrb rank:", np.linalg.matrix_rank(
    np.hstack([np.linalg.matrix_power(A_c, i) @ B_c for i in range(4)])))
