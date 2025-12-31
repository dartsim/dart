# Polyhedron Visual Example

## Summary

- Goal: render a convex polyhedron from a vertex list.
- Concepts/APIs: `gui::PolyhedronVisual`, `gui::GridVisual`.
- Expected output: a translucent convex hull displayed over a grid.
- Controls: none.

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

## Contact Manifold Cache Demo

This example now spawns two identical box stacks side-by-side:

- Left stack: persistent contacts OFF by default
- Right stack: persistent contacts ON by default

Use the ImGui panel to toggle persistent contacts for each stack and to reset
the stacks to their initial state. The diagnostics show the number of active
manifolds, persistent contacts, and contact constraints per world.
