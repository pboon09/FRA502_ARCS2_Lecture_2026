import numpy as np
import scipy
from model.model import get_state_space_matrices, get_discrete_state_space_matrices


class LQRController:
    def __init__(self, A_d=None, B_d=None, C=None, Q=None, R=None, dt=0.02, max_force=15.0):
        self.dt = dt
        self.max_force = max_force

        self.C = np.array([[1.0, 0.0, 0.0, 0.0]]) if C is None else np.atleast_2d(C)

        self.A_d, self.B_d = self.load_model(A_d, B_d)
        self.Q, self.R = self.build_weights(Q, R)
        self.K = self.compute_gain()
        self.Nbar = self.compute_nbar()

        print("--- LQRController Initialized ---")
        print("dt:", self.dt)
        print("Calculated Gain K:", self.K)
        print("Calculated Nbar:", self.Nbar)

    # Loads A_d, B_d from the model unless explicit matrices were given
    def load_model(self, A_d, B_d):
        # Implement here
        raise NotImplementedError

    # Bryson Rule Weights
    def build_weights(self, Q, R):
        # Implement here
        raise NotImplementedError

    # Discrete Riccati Equation
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


if __name__ == "__main__":
    controller = LQRController()
    eigvals = np.linalg.eigvals(controller.A_d - controller.B_d @ controller.K)
    print("dt:", controller.dt)
    print("K:", controller.K)
    print("Nbar:", controller.Nbar)
    print("Closed-loop eigenvalues:", eigvals)
    print("Closed-loop eigenvalue magnitudes:", np.abs(eigvals))
