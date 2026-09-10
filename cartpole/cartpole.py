import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import argparse
import gymnasium as gym
import numpy as np
import pygame
from gymnasium.envs.classic_control.cartpole import CartPoleEnv

from model import config
from controller.pole_placement import PolePlacementController
from controller.lqr import LQRController
from controller.mpc import MPCController

class PureContinuousCartPole(CartPoleEnv):
    def __init__(self, render_mode="human", tau=None):
        super().__init__(render_mode=render_mode)

        self.gravity = config.gravity
        self.masscart = config.cart_mass
        self.masspole = config.pole_mass
        self.length = config.pole_length
        self.total_mass = self.masspole + self.masscart
        self.polemass_length = self.masspole * self.length

        if tau is not None:
            self.tau = tau

        self.action_space = gym.spaces.Box(low=-15.0, high=15.0, shape=(1,), dtype=np.float32)

    def step(self, action):
        assert self.action_space.contains(action), f"{action!r} ({type(action)}) invalid"
        assert self.state is not None, "Call reset before using step()"

        x, x_dot, theta, theta_dot = self.state

        force = float(np.clip(action[0], -15.0, 15.0))

        costheta = np.cos(theta)
        sintheta = np.sin(theta)

        temp = (force + self.polemass_length * theta_dot**2 * sintheta) / self.total_mass
        thetaacc = (self.gravity * sintheta - costheta * temp) / (
            self.length * (4.0 / 3.0 - self.masspole * costheta**2 / self.total_mass)
        )
        xacc = temp - self.polemass_length * thetaacc * costheta / self.total_mass

        if self.kinematics_integrator == "euler":
            x = x + self.tau * x_dot
            x_dot = x_dot + self.tau * xacc
            theta = theta + self.tau * theta_dot
            theta_dot = theta_dot + self.tau * thetaacc
        else:
            x_dot = x_dot + self.tau * xacc
            x = x + self.tau * x_dot
            theta_dot = theta_dot + self.tau * thetaacc
            theta = theta + self.tau * theta_dot

        self.state = (x, x_dot, theta, theta_dot)

        terminated = bool(
            x < -self.x_threshold
            or x > self.x_threshold
            or theta < -self.theta_threshold_radians
            or theta > self.theta_threshold_radians
        )

        if self.render_mode == "human":
            self.render()

        return np.array(self.state, dtype=np.float32), 1.0, terminated, False, {}

def build_controller(controller_type, dt):
    if controller_type == "pole_placement":
        return PolePlacementController(dt=dt)
    if controller_type == "lqr":
        return LQRController(dt=dt)
    if controller_type == "mpc":
        return MPCController(dt=dt)
    raise ValueError("Invalid controller type. Choose 'pole_placement', 'lqr' or 'mpc'.")

def main():
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--controller", choices=("lqr", "pole_placement", "mpc"))
    parser.add_argument("--dt", type=float, default=None)
    args, _ = parser.parse_known_args()

    controller_type = args.controller or input("Select controller type (lqr/pole_placement/mpc): ").strip().lower()

    if args.dt is not None:
        control_dt = args.dt
    elif controller_type == "mpc":
        control_dt = 0.05
    else:
        control_dt = 0.02

    controller = build_controller(controller_type, control_dt)

    env = PureContinuousCartPole(render_mode="human", tau=control_dt)
    state, info = env.reset()

    print(f"sim and control both {control_dt*1000:.0f} ms ({1/control_dt:.0f} Hz)")
    print("left/right move ref | 0 = center | SPACE = pause | Q/ESC = quit")

    x_ref = 0.0
    ref_step = 1.0
    running, paused = True, False

    try:
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key in (pygame.K_q, pygame.K_ESCAPE):
                        running = False
                    elif event.key == pygame.K_SPACE:
                        paused = not paused
                    elif event.key == pygame.K_LEFT:
                        x_ref = max(x_ref - ref_step, -2.0)
                    elif event.key == pygame.K_RIGHT:
                        x_ref = min(x_ref + ref_step, 2.0)
                    elif event.key == pygame.K_0:
                        x_ref = 0.0

            if paused:
                env.render()
                continue

            action = controller.get_action(state, r=x_ref)
            state, reward, terminated, truncated, info = env.step(action)

            print(f"\rCart Ref: {x_ref:+.3f} | Cart pos: {state[0]:+.3f} m | Pole angle: {state[2]:+.3f} rad",
                  end="", flush=True)
    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        env.close()


if __name__ == "__main__":
    main()
