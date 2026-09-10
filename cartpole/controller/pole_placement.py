import numpy as np
from scipy import signal
from model.model import get_state_space_matrices, get_discrete_state_space_matrices


class PolePlacementController:
    def __init__(self, A_d=None, B_d=None, C=None, s_desired_poles=None, cart_ratio=0.4,
                 dt=0.02, max_force=15.0):
        self.dt = dt
        self.max_force = max_force

        self.C = np.array([[1.0, 0.0, 0.0, 0.0]]) if C is None else np.atleast_2d(C)

        self.A_d, self.B_d = self.load_model(A_d, B_d)
        self.s_desired_poles, self.z_desired_poles = self.map_poles_to_plane(s_desired_poles, cart_ratio)
        self.K = self.compute_gain()
        self.Nbar = self.compute_nbar()

        print("--- PolePlacementController Initialized ---")
        print("dt:", self.dt)
        print("Desired poles (s-plane):", self.s_desired_poles)
        print("Desired poles (z-plane):", self.z_desired_poles)
        print("Calculated Gain K:", self.K)
        print("Calculated Nbar:", self.Nbar)

    # Loads A_d, B_d from the model unless explicit matrices were given
    def load_model(self, A_d, B_d):
        # Implement here
        raise NotImplementedError

    # Desired Poles
    def map_poles_to_plane(self, s_desired_poles, cart_ratio):
        # Implement here
        raise NotImplementedError

    # Gain K
    def compute_gain(self):
        # Implement here
        raise NotImplementedError

    # Reference Scaling Nbar
    def compute_nbar(self):
        # Implement here
        raise NotImplementedError

    # Control Law
    def get_action(self, state, r=0.0):
        # Implement here
        raise NotImplementedError


# Mirrors the open-loop poles of A_c into the stable half plane
def mirror_open_loop_poles(A_c, cart_ratio=0.4):
    # Implement here
    raise NotImplementedError


if __name__ == "__main__":
    controller = PolePlacementController()
    eigvals = np.linalg.eigvals(controller.A_d - controller.B_d @ controller.K)
    print("dt:", controller.dt)
    print("K:", controller.K)
    print("Nbar:", controller.Nbar)
    print("Closed-loop eigenvalues:", eigvals)
    print("Closed-loop eigenvalue magnitudes:", np.abs(eigvals))
