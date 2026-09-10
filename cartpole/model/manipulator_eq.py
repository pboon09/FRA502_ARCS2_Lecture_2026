from . import config
import sympy as sp

# Build the inertia matrix M(q) of the cart-pole
def mass_matrix(pole_angle):
    # Implement here
    raise NotImplementedError

# Build the Coriolis/centripetal matrix C(q, q_dot) of the cart-pole
def coriolis_matrix(pole_angle, pole_velocity):
    # Implement here
    raise NotImplementedError

# Build the gravity vector G(q) of the cart-pole
def gravity_vector(pole_angle):
    # Implement here
    raise NotImplementedError
