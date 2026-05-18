# GustGurus — Robust Crazyflie Control under Wind Disturbance

A graduate project for **24-774 Advanced Control Systems Integration** at
Carnegie Mellon (Prof. Mark Bedillion, Fall 2024). The goal was to design,
simulate, and fly a quadrotor controller that holds position under
adversarial wind/gust disturbance, then evaluate it on real hardware.

We implemented three controller families on the [Bitcraze Crazyflie 2.1][cf21],
flew each against scripted step inputs in a controlled lab environment, and
compared tracking error.

**Demo videos:** [Google Drive folder][demos]

[cf21]: https://www.bitcraze.io/products/crazyflie-2-1/
[demos]: https://drive.google.com/drive/folders/19zOuI40kgHVWsKUbP4WhK3y6_nD7LDzT?usp=sharing

---

## What we built

| Controller | Where it runs | Notes |
|---|---|---|
| **PID** (baseline + tuned) | Crazyflie firmware | Project-tuned variant of the stock PID; sanity-check baseline. |
| **LQR** (full-state) | Crazyflie firmware | Discrete infinite-horizon gain solved offline in `analysis/Kinf_calc_final.py`, loaded as a constant `K` matrix. |
| **SMC** (sliding mode) | Crazyflie firmware + Simulink | First validated in Simulink against the nonlinear plant in `dynamics/`, then ported to C. |

Each is registered as a custom controller in the Crazyflie firmware
(see [`firmware/README.md`](firmware/README.md) for the integer mapping)
and is selected at runtime via the standard `stabilizer.controller`
parameter — no rebuild needed to switch between PID, LQR, and SMC.

A ROS 2 + Gazebo simulation under `simulation/` mirrors the firmware control
loop for development outside the lab.

---

## Why three controllers?

The course frames a tradeoff: simpler controllers (PID) are easier to tune
and survive model error well, but reject disturbances less aggressively.
Optimal controllers (LQR) reject disturbance optimally for the linearized
plant but assume the model is correct. Robust controllers (SMC) trade
chattering and tuning effort for guaranteed disturbance rejection without
an accurate model.

Flying all three on the same hardware, against the same gust profile, makes
the tradeoff concrete instead of theoretical.

---

## Repo layout

```
.
├── firmware/                 Crazyflie 2.1 firmware patches (modified files only)
│   ├── README.md             How to apply on top of upstream Bitcraze firmware
│   └── src/modules/...       Custom controllers + registration
├── simulation/               ROS 2 + Gazebo simulation
│   └── src/
│       ├── crazyflie_gazebo/ SDF models, world, launch files
│       ├── controllers/      Python motor-control node
│       └── cpp_controllers/  C++ port of the firmware PID for closed-loop sim
├── dynamics/                 MATLAB + Simulink rigid-body model
│   ├── Baseline.m            Numerical parameters + initial conditions
│   ├── p_dyn.m / p_lin_dyn.m Position dynamics (nonlinear / linearized)
│   ├── eta_dyn.m / eta_lin_dyn.m   Attitude dynamics
│   ├── Coriolis.m / inertia_matrix_J.m   Manipulator-form terms
│   ├── smc1.m / smc2.m       Sliding-mode controller scripts
│   └── control_stack.slx     Full Simulink control stack
└── analysis/                 Offline analysis + flight data
    ├── Kinf_calc_final.py    Discrete LQR gain solver (DARE)
    ├── square_wave_tracking.py   Step-response analysis utility
    ├── plotter1.py / plotter2.py Flight-log plotting
    └── data/                 Recorded error and motor logs from flights
```

The rigid-body derivation lives in the MATLAB source under
`dynamics/` -- `Baseline.m` sets numerical parameters,
`p_dyn.m` / `eta_dyn.m` build the position and attitude dynamics, and
`Coriolis.m` / `inertia_matrix_J.m` provide the manipulator-form terms
that `control_stack.slx` consumes.

---

## Quick start

### Simulation (ROS 2 Humble + Gazebo)

```bash
cd simulation
colcon build
source install/setup.bash

# Launch Gazebo, the ros_gz bridge, and the motor-control node
ros2 launch crazyflie_gazebo crazyflie_gazebo.launch.py
```

`motor_control_node` runs the C++ port of the firmware PID against a
goal state on `/crazyflie/goal_state_vector` (Float32MultiArray of
`[x, y, z, roll, pitch, yaw]`). Publish a setpoint to make the drone
fly to a target:

```bash
ros2 topic pub --once /crazyflie/goal_state_vector std_msgs/msg/Float32MultiArray \
  "{data: [0.0, 0.0, 1.0, 0.0, 0.0, 0.0]}"
```

For interactive keyboard control, run the optional bridge node and
`teleop_twist_keyboard` in two extra terminals:

```bash
ros2 run crazyflie_gazebo control_services
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

The world file uses `@MODELS_DIR@` placeholders that the launch file
substitutes at runtime, so no per-machine path editing is required.

### Hardware (Crazyflie 2.1)

The custom controllers are distributed as patches on top of upstream
firmware — see [`firmware/README.md`](firmware/README.md) for the build and
flash procedure. Short version:

```bash
git clone --recurse-submodules https://github.com/bitcraze/crazyflie-firmware.git
cp -r firmware/src/* crazyflie-firmware/src/
cd crazyflie-firmware && make cf2_defconfig && make -j && make cload
```

Switch controllers at runtime from `cfclient` (Parameters → `stabilizer.controller`).
See the integer mapping in [`firmware/README.md`](firmware/README.md).

### Offline analysis

```bash
# Solve the discrete LQR gain (uses scipy.linalg.solve_discrete_are)
python analysis/Kinf_calc_final.py

# Plot a recorded flight (plotters read fixed filenames from cwd)
cd analysis/data && python ../plotter1.py
```

---

## Results

Position tracking error against a scripted step profile, averaged across
runs (full data in `analysis/data/normalized_error_log.csv`):

- **PID (tuned):** holds altitude well in still air, drifts under sustained
  lateral gusts; recovery is slow.
- **LQR:** lower steady-state error than PID in roll/pitch; relies on the
  linearization being accurate near hover.
- **SMC:** strongest disturbance rejection of the three; visible chattering
  in motor commands, mitigated with a boundary-layer approximation.

See the demo videos for the qualitative difference, and the `dynamics/`
MATLAB sources for the model derivation that motivates each gain choice.

---

## Course context

- **Class:** [24-774 Advanced Control Systems Integration][class]
- **Instructor:** [Prof. Mark Bedillion][bedillion], CMU MechE
- **TA:** [Conor Igoe][igoe]
- **Term:** Fall 2024

The course covers state-space modeling, optimal control (LQR/LQG), robust
and sliding-mode control, and a hardware integration project. This repo is
that integration project.

[class]: https://www.meche.engineering.cmu.edu/education/courses/24-774.html
[bedillion]: https://www.meche.engineering.cmu.edu/directory/bios/bedillion-mark.html
[igoe]: https://www.andrew.cmu.edu/user/capn/

---

## Team

**GustGurus** — Will Kraus, Nikolaj Hindsbo, and project teammates.
Original repository: [github.com/willkraus9/GustGurus-Drone-Project][orig].

[orig]: https://github.com/willkraus9/GustGurus-Drone-Project

---

## License

[MIT](LICENSE). The vendored Bitcraze firmware patches in `firmware/`
remain subject to the upstream [crazyflie-firmware][cfw] license (GPLv3) —
apply them on top of an upstream checkout rather than redistributing the
combined tree.

[cfw]: https://github.com/bitcraze/crazyflie-firmware
