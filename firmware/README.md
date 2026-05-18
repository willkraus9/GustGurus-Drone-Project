# Crazyflie 2.1 firmware patches

This directory contains **only the files we modified** in the Bitcraze
[crazyflie-firmware](https://github.com/bitcraze/crazyflie-firmware) source
tree. Layout mirrors the upstream paths so the files can be dropped in
directly.

```
firmware/src/modules/
├── interface/controller/
│   ├── controller_custom1.h       SMC z + roll/pitch
│   ├── controller_custom2.h       LQR full-state
│   ├── controller_custom3.h       Direct-thrust PID (WIP)
│   ├── controller_custom4.h       Hybrid SMC/PID
│   ├── controller_custom5.h       Final tuned PID
│   └── controller_custom_types.h  Shared types for the custom controllers
└── src/controller/
    ├── controller.c               Registration of the custom controllers
    ├── controller_pid.c           Baseline PID with project tweaks
    └── controller_customN.c       Implementation for each custom variant
```

## Building

The custom controllers are registered alongside the stock controllers in
`controller.c`. To build:

```bash
# 1. Clone Bitcraze firmware at a known-good tag (we used 2024.10 / commit dac34d4)
git clone --recurse-submodules https://github.com/bitcraze/crazyflie-firmware.git
cd crazyflie-firmware

# 2. Copy the patches over
cp -r /path/to/this/repo/firmware/src/* src/

# 3. Configure -- enable the custom controllers
make cf2_defconfig
make menuconfig    # Expert -> Controllers -> include the desired CONTROLLER_CUSTOM_N

# 4. Build and flash
make -j$(nproc)
make cload
```

## Selecting a controller at runtime

The stock Crazyflie param subsystem exposes a `stabilizer.controller`
parameter. Set it via `cfclient` or `cflib`:

| Value | Controller |
|------:|------------|
| `1`   | PID (stock, with our tweaks in `controller_pid.c`) |
| `5`   | `CONTROLLER_CUSTOM1` -- SMC |
| `6`   | `CONTROLLER_CUSTOM2` -- LQR |
| `7`   | `CONTROLLER_CUSTOM3` -- Direct-thrust PID (WIP) |
| `8`   | `CONTROLLER_CUSTOM4` -- Hybrid |
| `9`   | `CONTROLLER_CUSTOM5` -- Final tuned PID |

(Exact integer mapping depends on the order they are registered in
`controller.c`. Use `cfclient` to confirm.)

## Why patches and not a full vendored copy?

The original commit history vendored the entire 26 MB Bitcraze tree, which
made the repo unreviewable and meant our 13 modified files were lost in 2,000
unrelated upstream files. Carrying only the diff keeps the project focused on
what we actually contributed and avoids drifting away from upstream.
