# DART Examples README

## Overview

- Maintained examples live flat under `examples/` for simpler discovery and
  tooling.
- Filament is the maintained visualization path. Legacy OpenSceneGraph example
  sources were removed; common historical example names still route through the
  Filament scene runner where a replacement scene exists.
- Legacy OpenSceneGraph C++ tutorials were removed from `tutorials/`.

## Categories (Ordered)

### 00 Getting Started

- `unified_loading`

### 01 Visualization and Interaction

- `filament_gui`
- `gui_scene_diagnostics`

### 02 Performance and Scaling

- `headless_simulation`
- `speed_test`

### 03 Integration and Tools

- `csv_logger`
- `rerun`

## Build Each Example

Copy the example directory to your workspace and follow the instruction of any
README.md in that example directory.

If you are working inside the DART repo, prefer the `pixi run` entry points
documented in `docs/onboarding/building.md` when available.

For C++ examples, the generic in-tree runner is:

```bash
pixi run ex <example-target>
```

For example, `pixi run ex csv_logger` builds and runs `csv_logger`. Examples
with extra CMake requirements or redirected GUI runners, such as
`filament_gui`, declare those requirements in `scripts/run_cpp_example.py`.
Historical GUI example names such as `hello_world`, `boxes`, `rigid_chain`,
`atlas_simbicon`, and `heightmap` are compatibility aliases that build
`filament_gui` and select the matching Filament scene.

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

    $ ./csv_logger

Follow the instructions detailed in the console.
