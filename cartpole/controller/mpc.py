import numpy as np
import scipy
import cvxpy as cp
from model.model import get_discrete_state_space_matrices

class MPCController:
    def __init__(self, A_d=None, B_d=None, C=None, Q=None, R=None, dt=0.05, horizon=20):
        self.C = np.array([[1.0, 0.0, 0.0, 0.0]]) if C is None else np.atleast_2d(C)
        self.dt = dt
        self.N = horizon
        self.A_d, self.B_d = get_discrete_state_space_matrices(C_c=self.C, dt=self.dt)
        self.num_states = self.A_d.shape[0]
        self.num_inputs = self.B_d.shape[1]

        self.build_bounds()
        self.Q, self.R = self.build_weights(Q, R)

        # Terminal weight
        self.Q_f = scipy.linalg.solve_discrete_are(self.A_d, self.B_d, self.Q, self.R)

        self.u_prev = None

        self.build_problem()

        print("--- MPCController Initialized ---")
        print(f"  horizon N = {self.N} ({self.N * self.dt:.2f} s), dt = {self.dt}")
        eig_d = np.linalg.eigvals(np.asarray(self.A_d, dtype=np.float64))
        print(f"  open-loop |eig(A_d)| = {np.round(np.abs(eig_d), 4)}")
        if not np.any(np.abs(eig_d) > 1.0):
            print("  [WARN] no eigenvalue with magnitude > 1 -- the linearisation point")
            print("         looks like 'pole hanging down', not 'pole upright'.")

    # Constraint Bounds
    # Set the state and input limits the optimizer must respect
    def build_bounds(self):
        # Implement here
        raise NotImplementedError

    # Build the state and input cost matrices
    def build_weights(self, Q, R):
        # Implement here
        raise NotImplementedError

    # Set up the cvxpy quadratic program and its parameters
    def build_problem(self):
        # Implement here
        raise NotImplementedError

    # Shift the previously cached plan forward when the solver has no fresh solution
    def fallback_action(self):
        # Implement here
        raise NotImplementedError

    # Receding Horizon: solve the QP for the current state and return the first input
    def get_action(self, current_state, r=0.0):
        # Implement here
        raise NotImplementedError

if __name__ == "__main__":
    mpc = MPCController()
    x0 = [0.0, 0.0, 0.05, 0.0]
    u = mpc.get_action(x0, r=0.5)
    print(f"solver status: {mpc.prob.status}")
    print(f"first optimal input: {u}")
