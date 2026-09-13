# Cartpole

Gymnasium prototype of the cart-pole.

## Install

```bash
cd cartpole && pip install -r requirements.txt
```

## Run

Balance the pole in the simulator:

```bash
cd cartpole && python3 cartpole.py --controller pole_placement
```

Swap `pole_placement` for `lqr` or `mpc`. Leave `--controller` off and it asks.

Plot the poles and the closed-loop response:

```bash
cd cartpole && python3 analysis/pole_explorer.py
```

Edit the block at the top of `analysis/pole_explorer.py` to choose the poles, then rerun.

Plot the open loop instead:

```bash
cd cartpole && python3 analysis/response_plot.py
```

## Layout

| Path | What |
|---|---|
| `model/config.py` | plant parameters, mirrored into the gym env by `cartpole.py` |
| `model/manipulator_eq.py` | mass, Coriolis and gravity matrices |
| `model/model.py` | symbolic linearization, continuous and discrete state space |
| `controller/` | pole placement, LQR, MPC control laws |
| `analysis/` | pole map, step response, static pole explorer |
| `cartpole.py` | gym environment and the keyboard loop |
