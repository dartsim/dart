# DART Python Examples

## Overview

- Examples are organized into numbered, user-first categories that mirror the C++ examples layout.
- Start in `00_getting_started` for minimal, first-run examples.
- Continue with `01_rigid_bodies_and_frames` for basic rigid-body scenes.
- Explore `02_joints_and_constraints` for joint types and constraint behavior.
- Dig into `03_collisions_and_contacts` for collision and terrain workflows.
- Move on to `04_control_and_ik` for controllers and inverse kinematics.
- Visit `05_io_and_models` for model loading and resource retrieval.
- Check `06_soft_and_hybrid` for soft-body and mixed dynamics examples.
- Explore `07_visualization_and_interaction` for GUI and input patterns.
- See `08_performance_and_scaling` for timing and scaling examples.
- Check `09_integration_and_tools` for tooling and integration demos.
- Tutorials remain in `python/tutorials/`.

## Run Examples

If you are working inside the DART repo, prefer the `pixi run` entry points
documented in `docs/onboarding/building.md` when available.

For example:

    $ pixi run py-ex hello_world
    $ pixi run py-ex 00_getting_started/hello_world

Or, without pixi:

    $ PYTHONPATH=build/<env>/cpp/<build_type>/python \
      python python/examples/00_getting_started/hello_world/main.py
