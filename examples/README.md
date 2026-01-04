# DART Examples README

## Overview

- Examples live flat under `examples/` for simpler discovery and tooling.
- Category order is defined below; directory names stay short and stable.
- Tutorials remain in `tutorials/`.

## Categories (Ordered)

### 00 Getting Started

- `hello_world`

### 01 Rigid Bodies and Frames

- `frame_hierarchy`
- `rigid_boxes`
- `rigid_hardcoded_design`
- `rigid_cubes`

### 02 Joints and Constraints

- `joint_coupler`
- `joint_free_cases`
- `joint_human_limits`
- `joint_constraints`
- `joint_lcp_solvers`
- `joint_mimic_pendulums`
- `joint_chain`
- `joint_loop`

### 03 Collisions and Contacts

- `box_stacking`
- `capsule_ground_contact`
- `heightmap`
- `rigid_shapes`

### 04 Control and IK

- `control_balance_biped`
- `control_operational_space`
- `control_vehicle`
- `control_walking_humanoid`
- `ik_analytic_wam`
- `ik_humanoid`

### 05 IO and Models

- `fetch`
- `unified_loading`

### 06 Soft and Hybrid

- `hybrid_dynamics`
- `mixed_chain`
- `soft_bodies`

### 07 Visualization and Interaction

- `add_delete_skels`
- `drag_and_drop`
- `empty`
- `imgui`
- `polyhedron_visual`
- `raylib`
- `simulation_event_handler`
- `tinkertoy`

### 08 Performance and Scaling

- `headless_simulation`

### 09 Integration and Tools

- `csv_logger`
- `point_cloud` (requires OctoMap)

## Build Each Example

Copy the example directory to your workspace and follow the instruction of any
README.md in that example directory.

If you are working inside the DART repo, prefer the `pixi run` entry points
documented in `docs/onboarding/building.md` when available.

## Build Examples as One Project

### Build Instructions

This project is dependent on DART. Please make sure a proper version of DART is
installed before building this project.

Copy this directory to your workspace (e.g., in Linux):

    $ cp -r examples /your/workspace/directory/dart_examples
    $ cd /your/workspace/directory/dart_examples

From the workspace directory:

    $ mkdir build
    $ cd build
    $ cmake ..
    $ make

### Execute Instructions

Launch the each executable from the build directory above (e.g.,):

    $ ./hello_world

Follow the instructions detailed in the console.
