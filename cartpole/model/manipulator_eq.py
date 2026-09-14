from . import config
import sympy as sp

# Build the inertia matrix M(q) of the cart-pole
def mass_matrix(pole_angle):
    M = sp.Matrix([[config.cart_mass + config.pole_mass, config.pole_mass * config.pole_length * sp.cos(pole_angle)],
                   [config.pole_mass * config.pole_length * sp.cos(pole_angle), config.pole_inertia + config.pole_mass * config.pole_length**2]])
    return M

# Build the Coriolis/centripetal matrix C(q, q_dot) of the cart-pole
def coriolis_matrix(pole_angle, pole_velocity):
    C = sp.Matrix([[0, -config.pole_mass * config.pole_length * pole_velocity * sp.sin(pole_angle)],
                   [0, 0]])
    return C

# Build the gravity vector G(q) of the cart-pole
def gravity_vector(pole_angle):
    G = sp.Matrix([[0],
                   [-config.pole_mass * config.gravity * config.pole_length * sp.sin(pole_angle)]])
    return G
