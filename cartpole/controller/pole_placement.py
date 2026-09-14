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
        if A_d is None or B_d is None:
            return get_discrete_state_space_matrices(self.C, self.dt)
        return A_d, B_d

    # Desired Poles
    def map_poles_to_plane(self, s_desired_poles, cart_ratio):
        if s_desired_poles is None:
            s_desired_poles = mirror_open_loop_poles(get_state_space_matrices()[0], cart_ratio)
        else:
            s_desired_poles = np.array(s_desired_poles)

        z_desired_poles = np.exp(s_desired_poles * self.dt)
        return s_desired_poles, z_desired_poles

    # Gain K
    def compute_gain(self):
        k = signal.place_poles(self.A_d, self.B_d, self.z_desired_poles)
        return k.gain_matrix

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


# Mirrors the open-loop poles of A_c into the stable half plane
def mirror_open_loop_poles(A_c, cart_ratio=0.4):
    unstable_rate = float(np.max(np.linalg.eigvals(A_c).real))
    cart_rate = cart_ratio * unstable_rate
    return np.array([-cart_rate, -cart_rate*1.001, -unstable_rate, -unstable_rate*1.001])


if __name__ == "__main__":
    controller = PolePlacementController()
    eigvals = np.linalg.eigvals(controller.A_d - controller.B_d @ controller.K)
    print("dt:", controller.dt)
    print("K:", controller.K)
    print("Nbar:", controller.Nbar)
    print("Closed-loop eigenvalues:", eigvals)
    print("Closed-loop eigenvalue magnitudes:", np.abs(eigvals))
