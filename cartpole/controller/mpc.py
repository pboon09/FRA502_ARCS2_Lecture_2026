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
        self.max_cart_pos = 2.0
        self.max_pole_angle = 0.15
        self.max_cart_vel = 2.5
        self.max_pole_vel = 3.0
        self.max_control = 15.0

        self.x_max = np.array([self.max_cart_pos, self.max_pole_angle, self.max_cart_vel, self.max_pole_vel])
        self.x_min = -self.x_max
        self.u_max = np.array([self.max_control])
        self.u_min = -self.u_max

    # Build the state and input cost matrices
    def build_weights(self, Q, R):
        tol_cart_pos = 0.5
        tol_pole_angle = 0.1
        tol_cart_vel = 1.0
        tol_pole_vel = 1.0

        Q = Q if Q is not None else np.diag([
            1.0 / (tol_cart_pos ** 2),
            1.0 / (tol_pole_angle ** 2),
            1.0 / (tol_cart_vel ** 2),
            1.0 / (tol_pole_vel ** 2)
        ])

        R = R if R is not None else np.array([[
            1.0 / (self.max_control ** 2)
        ]])

        self.rho_slack = 1e2

        return Q, R

    # Set up the cvxpy quadratic program and its parameters
    def build_problem(self):
        num_states, num_inputs, N = self.num_states, self.num_inputs, self.N
        self.x = cp.Variable((num_states, N + 1))
        self.u = cp.Variable((num_inputs, N))
        self.eps = cp.Variable((num_states, N), nonneg=True)
        self.p_x0 = cp.Parameter(num_states)
        self.p_xref = cp.Parameter(num_states)

        Lq = np.linalg.cholesky(self.Q).T
        Lr = np.linalg.cholesky(self.R).T
        Lqf = np.linalg.cholesky(self.Q_f).T

        cost = 0
        constraints = [self.x[:, 0] == self.p_x0]
        for k in range(N):
            constraints.append(self.x[:, k+1] == self.A_d @ self.x[:, k] + self.B_d @ self.u[:, k])
            constraints.append(self.x[:, k+1] <= self.x_max + self.eps[:, k])
            constraints.append(self.x[:, k+1] >= self.x_min - self.eps[:, k])
            constraints.append(self.u[:, k] <= self.u_max)
            constraints.append(self.u[:, k] >= self.u_min)

            cost += cp.sum_squares(Lq @ (self.x[:, k] - self.p_xref))
            cost += cp.sum_squares(Lr @ self.u[:, k])
            cost += self.rho_slack * cp.sum(self.eps[:, k])

        cost += cp.sum_squares(Lqf @ (self.x[:, N] - self.p_xref))

        self.prob = cp.Problem(cp.Minimize(cost), constraints)

    # Shift the previously cached plan forward when the solver has no fresh solution
    def fallback_action(self):
        if self.u_prev is None or self.u_prev.shape[1] < 2:
            self.u_prev = None
            return np.array([0.0], dtype=np.float32)

        self.u_prev = np.hstack([self.u_prev[:, 1:], np.zeros((self.u_prev.shape[0], 1))])
        u0 = float(np.clip(self.u_prev[0,0],self.u_min[0], self.u_max[0]))
        return np.array([u0], dtype=np.float32)

    # Receding Horizon: solve the QP for the current state and return the first input
    def get_action(self, current_state, r=0.0):
        x_init = np.asarray(current_state, dtype=np.float64).flatten()
        r = float(np.clip(r, self.x_min[0], self.x_max[0]))
        x_ref = np.array([r, 0.0, 0.0, 0.0])

        self.p_x0.value = x_init
        self.p_xref.value = x_ref

        try:
            self.prob.solve(solver=cp.OSQP, warm_start=True, max_iter=30000, verbose=False)
        except cp.SolverError:
            return self.fallback_action

        if self.prob.status not in ["optimal", "optimal_inaccurate"] or self.u.value is None:
            return self.fallback_action

        self.u_prev = np.asarray(self.u.value, dtype=np.float64)
        optimal_u = float(np.clip(self.u_prev[0,0],self.u_min[0], self.u_max[0]))

        return np.array([optimal_u], dtype=np.float32)

if __name__ == "__main__":
    mpc = MPCController()
    x0 = [0.0, 0.0, 0.05, 0.0]
    u = mpc.get_action(x0, r=0.5)
    print(f"solver status: {mpc.prob.status}")
    print(f"first optimal input: {u}")
