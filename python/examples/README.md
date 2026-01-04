# DART Python Examples

## Overview

- Examples live flat under `python/examples/` for simpler discovery and tooling.
- Categories below mirror the C++ examples layout; empty sections are listed for consistency.
- Tutorials remain in `python/tutorials/`.

## Categories (Ordered)

### 00 Getting Started

- `hello_world` (use `--gui` to launch the viewer)

### 01 Rigid Bodies and Frames

- `rigid_cubes`

### 02 Joints and Constraints

- `joint_chain`
- `joint_loop`

### 03 Collisions and Contacts

- `collision_contact_points`

### 04 Control and IK

- `control_balance_biped`
- `control_operational_space`
- `ik_humanoid` (Atlas-only)

### 05 IO and Models

- (No Python examples yet.)

### 06 Soft and Hybrid

- (No Python examples yet.)

### 07 Visualization and Interaction

- `viz_drag_and_drop`
- `viz_imgui`

### 08 Performance and Scaling

- (No Python examples yet.)

### 09 Integration and Tools

- (No Python examples yet.)

## Run Examples

If you are working inside the DART repo, prefer the `pixi run` entry points
documented in `docs/onboarding/building.md` when available.

For example:

    $ pixi run py-ex hello_world
    $ pixi run py-ex hello_world -- --gui

Or, without pixi:

    $ PYTHONPATH=build/<env>/cpp/<build_type>/python \
      python python/examples/hello_world/main.py
    $ PYTHONPATH=build/<env>/cpp/<build_type>/python \
      python python/examples/hello_world/main.py --gui
