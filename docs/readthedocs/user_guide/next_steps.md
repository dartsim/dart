# Next steps

You can now build a world, populate it with rigid bodies and articulated
systems, resolve contact, choose solvers, and visualize the result. Here is where
to go to deepen each thread.

## Go deeper on the concepts

- **{doc}`Key Topics </topics/index>`** — concept overviews and deep dives,
  including control theory, numerical methods, simulation stability, and inverse
  kinematics.
- **{doc}`Architecture </architecture>`** — how DART 7 supports multiple physics
  domains, solver methods, and compute backends in one step pipeline.

## Learn from runnable examples

The demo scenes are working DART 7 programs you can read, run, and modify:

- [Demo scene sources](https://github.com/dartsim/dart/tree/main/python/examples/demos/scenes)
  — rigid bodies, articulated arms, solver comparisons, and more.
- [Python examples](https://github.com/dartsim/dart/tree/main/python/examples)
  — the broader example tree.

Run any scene with `pixi run py-demos -- --scene <id>` (use `--list` to see them
all).

## Reference

- **{doc}`Python API reference </dartpy/python_api_reference>`** — the full
  `dartpy` surface.
- **{doc}`C++ API reference </dart/cpp_api_reference>`** — for C++ users; note the
  DART 7 C++ facade is still being finalized.
- **[Simulation API design notes](https://github.com/dartsim/dart/blob/main/docs/design/simulation_cpp_api.md)**
  — the rationale and current state of the DART 7 `World` API, including areas
  (like closed-chain loop closures) that are still maturing.

## Get help and follow along

- **Questions and discussion:**
  [GitHub Discussions](https://github.com/dartsim/dart/discussions).
- **Bugs and documentation fixes:**
  [GitHub Issues](https://github.com/dartsim/dart/issues) — DART 7 is under active
  development, so reports of anything that does not match these pages are
  especially welcome.
- **Stable release:** if you need production-ready DART today, use
  [DART 6 LTS](https://dart.readthedocs.io/en/stable/).

```{admonition} Help shape the guide
:class: tip

This user guide grows alongside DART 7. If a topic you need is missing — sensors,
deformable bodies, differentiable simulation, batched worlds — open an issue so
it can be prioritized.
```
