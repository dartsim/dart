# Heightmap Example

## Summary

- Goal: interactively sculpt a heightmap and observe contact alignment.
- Concepts/APIs: `dynamics::HeightmapShape`, ImGui viewer controls.
- Expected output: the recommended in-tree runner opens a Filament viewer with
  a heightmap surface and reference markers.
- Controls: left drag orbits, right/middle drag pans, wheel zooms, and Escape
  exits. The legacy standalone source keeps the ImGui panel and optional
  `--demo alignment` mode for comparison.

## Notes

- Legacy standalone source only: use `--gui-scale` to scale the ImGui widgets.
- Legacy standalone source only: use `--demo alignment` to compare heightmap
  alignment with a reference box.
- The recommended in-tree runner now uses the Filament scene in
  `examples/filament_gui`; the standalone source in this directory remains as
  the legacy OSG/ImGui version until the promoted GUI API grows the replacement
  panel and contact-alignment controls.

## Run In Tree

From the repository root:

```bash
pixi run ex heightmap
```

This builds and runs `examples/filament_gui --scene heightmap`, so the
recommended visual path no longer depends on the legacy OSG viewer.

This project is dependent on DART. Please make sure a proper version of DART is
installed before building this project.

## Build Instructions

From this directory:

    $ mkdir build
    $ cd build
    $ cmake ..
    $ make

## Execute Instructions

Launch the executable from the build directory above:

    $ ./{generated_executable}

Follow the instructions detailed in the console.
