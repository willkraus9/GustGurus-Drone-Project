# Crazyflie 2.1 firmware patches

This directory contains **only the files we modified** in the Bitcraze
[crazyflie-firmware](https://github.com/bitcraze/crazyflie-firmware) source
tree. Layout mirrors the upstream paths so the files can be dropped in
directly.

```
firmware/src/modules/
├── interface/controller/
│   ├── controller_custom1.h       SMC (z + roll/pitch) -- registered
│   ├── controller_custom2.h       Direct-thrust PID    -- registered
│   ├── controller_custom3.h       LQR (full-state)     -- registered
│   ├── controller_custom4.h       LQR + Riccati solver -- WIP, commented out in controller.c
│   ├── controller_custom5.h       SMC variant          -- WIP, commented out in controller.c
│   └── controller_custom_types.h  Shared types
└── src/controller/
    ├── controller.c               Registration table (controllerFunctions[])
    ├── controller_pid.c           Stock PID with project tweaks
    └── controller_customN.c       Implementation for each custom variant
```

The two WIP controllers (Custom4, Custom5) are kept in-tree for the
record of what we explored but are not registered in
`controllerFunctions[]`. Uncomment the matching lines in
`controller.c` to flight-test them.

## Building

The custom controllers are wired into the stabilizer through the
`controllerFunctions[]` registration table in `controller.c` -- there is
no Kconfig switch, so the patched `controller.c` builds them all in by
default. To build:

```bash
# 1. Clone Bitcraze firmware (we used the 2024.10 release; commit dac34d4
#    of this repo is known good).
git clone --recurse-submodules https://github.com/bitcraze/crazyflie-firmware.git
cd crazyflie-firmware

# 2. Copy the patches over (preserves the upstream directory layout).
cp -r /path/to/this/repo/firmware/src/* src/

# 3. Configure for the Crazyflie 2.1 and build.
make cf2_defconfig
make -j$(nproc)

# 4. Flash over USB or radio.
make cload
```

If you also want the upstream `CONFIG_CONTROLLER_*` Kconfig switches to
recognize the custom controllers (for compile-time selection), add the
matching `config CONTROLLER_CUSTOM_N` blocks to
`src/modules/src/Kconfig`. The runtime path below works without it.

## Selecting a controller at runtime

The stock Crazyflie param subsystem exposes a `stabilizer.controller`
parameter. Set it via `cfclient` or `cflib`. The integer maps to the
position in `controllerFunctions[]`:

| Value | Controller | Notes |
|------:|------------|-------|
| `1`   | PID (stock + project tweaks) | `controller_pid.c` |
| `2`   | Mellinger | upstream |
| `3`   | INDI | upstream |
| `4`   | Brescianini | upstream |
| `5`   | Custom 1 -- SMC | our work |
| `6`   | Custom 2 -- Direct-thrust PID | our work |
| `7`   | Custom 3 -- LQR | our work |

Use `cfclient` -> Parameters -> `stabilizer.controller` to switch live
without re-flashing. The `controllerGetName()` helper in
`controller.c` returns the registered name string so you can confirm
which one is active in the log stream.

## Why patches and not a full vendored copy?

The original commit history vendored the entire 26 MB Bitcraze tree, which
made the repo unreviewable and meant our 13 modified files were lost in 2,000
unrelated upstream files. Carrying only the diff keeps the project focused on
what we actually contributed and avoids drifting away from upstream.
