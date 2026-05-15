# Biped Stand Example

## Summary

- Goal: keep a biped standing with SPD control while applying perturbations.
- Concepts/APIs: SPD tracking, custom `RealTimeWorldNode`, external forces.
- Expected output: `pixi run ex biped_stand` opens the experimental Filament
  view with a posed fullbody biped and ground. The standalone legacy source
  shows a biped that tries to balance under scripted pushes.
- Controls: in the legacy standalone source, space starts simulation and 1-4
  apply directional pushes.

## In-Tree Runner

Inside the DART checkout:

```bash
pixi run ex biped_stand
```

The in-tree runner builds the experimental Filament GUI and launches the
shared `--scene hybrid-dynamics` fixture. The source in this directory remains
legacy OSG comparison material for the SPD controller and perturbation
controls until the promoted `dart::gui` API has controller/tool support.

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
