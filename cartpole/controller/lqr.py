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
        if A_d is None or B_d is None:
            return get_discrete_state_space_matrices(C_c=self.C, dt=self.dt)
        return A_d, B_d

    # Bryson Rule Weights
    def build_weights(self, Q, R):
        tol_cart_pos = 0.5
        tol_pole_angle = 0.1
        tol_cart_vel = 1.0
        tol_pole_vel = 1.0
        max_control = 15.0

        Q = Q if Q is not None else np.diag([
            1.0 / (tol_cart_pos ** 2),
            1.0 / (tol_pole_angle ** 2),
            1.0 / (tol_cart_vel ** 2),
            1.0 / (tol_pole_vel ** 2)
        ])

        R = R if R is not None else np.array([
            1.0 / (max_control ** 2)
        ])

        return Q, R

    # Discrete Riccati Equation
    def compute_gain(self):
        P = scipy.linalg.solve_discrete_are(self.A_d, self.B_d, self.Q, self.R)
        K = np.linalg.inv(self.R + self.B_d.T @ P @ self.B_d) @ (self.B_d.T @ P @ self.A_d)
        return K

    # Reference Scaling Nbar
    def compute_nbar(self):
        I = np.eye(self.A_d.shape[0])
        return np.linalg.inv(self.C @ np.linalg.solve(I - (self.A_d - self.B_d @ self.K), self.B_d))

    # Control Law
    def get_action(self, state, r=0.0):
        x = np.asarray(state, dtype=np.float64).reshape(-1, 1)
        r = np.atleast_2d(np.asarray(r,dtype=np.float64)).reshape(-1, 1)
        u = self.Nbar @ r - self.K @ x
        u = np.clip(u.ravel()[0], -self.max_force, self.max_force)
        return np.array([u], dtype=np.float32)


if __name__ == "__main__":
    controller = LQRController()
    eigvals = np.linalg.eigvals(controller.A_d - controller.B_d @ controller.K)
    print("dt:", controller.dt)
    print("K:", controller.K)
    print("Nbar:", controller.Nbar)
    print("Closed-loop eigenvalues:", eigvals)
    print("Closed-loop eigenvalue magnitudes:", np.abs(eigvals))
