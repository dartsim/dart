# DART Python Examples

## Overview

- Examples live flat under `python/examples/` for simpler discovery and tooling.
- Categories below mirror the C++ examples layout; not every category has a Python example yet.
- Tutorials remain in `python/tutorials/`.

## Categories (Ordered)

### 00 Getting Started

- `hello_world`
- `hello_world_gui`

### 01 Rigid Bodies and Frames

- `rigid_cubes`

### 02 Joints and Constraints

- `joint_chain`
- `joint_loop`

### 03 Collisions and Contacts

- `contacts_pointcloud`

### 04 Control and IK

- `atlas_puppet`
- `control_balance_biped`
- `control_operational_space`

### 07 Visualization and Interaction

- `drag_and_drop`
- `imgui_python`

## Run Examples

If you are working inside the DART repo, prefer the `pixi run` entry points
documented in `docs/onboarding/building.md` when available.

For example:

    $ pixi run py-ex hello_world

Or, without pixi:

    $ PYTHONPATH=build/<env>/cpp/<build_type>/python \
      python python/examples/hello_world/main.py
