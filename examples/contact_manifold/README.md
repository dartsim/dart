# Contact Manifold Example

## Summary

- Goal: visualize how contact manifold caching stabilizes contact points over time.
- Concepts/APIs: `ContactManifoldCache`, `ConstraintSolver`, contact visualization with `SimpleFrame`.
- Expected output: an ImGui-driven OSG viewer with two box stacks side-by-side.
  - Left stack: Manifold OFF (contacts flicker each frame).
  - Right stack: Manifold ON (contacts persist and become stable).
- Controls: use the ImGui panel to toggle manifolds, adjust stack height, and control visualization.

## Contact Visualization

Contact points are visualized as colored spheres based on their age:
- **Red**: New contact (age 0)
- **Yellow**: Warming up (age ~5 frames)
- **Green**: Stable contact (age >= 10 frames)

The right stack (with manifold caching enabled) will show contacts quickly becoming green,
while the left stack (without caching) will show contacts constantly flickering red as they
are recreated each frame.

## Notes

- Use `--gui-scale` to scale the ImGui widgets.

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
