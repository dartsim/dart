# Agent Guidelines for DART

This file provides conventions and tips for agents working in this repository.  Read it before modifying code or documentation.

## Overview
- DART (Dynamic Animation and Robotics Toolkit) is a mixed C++/Python project that offers kinematic and dynamic algorithms for robotics and animation.
- The core library lives under `dart/` (C++).  Python bindings are built in `python/` and exposed as `dartpy`.

## Repository Practices
- Prefer [`rg`](https://github.com/BurntSushi/ripgrep) for searching the code base.
- Keep commits focused and run the relevant checks before committing.
- Follow the style guide in `CONTRIBUTING.md` (two‑space indent, camelCase functions, PascalCase classes, no cuddled braces).

## Building for Codex
These steps configure and compile DART from source in the Codex environment.
1. Configure the build (defaults to Release):
   ```bash
   pixi run config
   ```
2. Build the C++ library and utilities:
   ```bash
   pixi run build
   ```
3. Build the Python bindings (`dartpy`):
   ```bash
   pixi run build-dartpy
   ```
4. Run the test suites:
   ```bash
   pixi run test        # C++ tests
   pixi run test-dartpy # Python tests
   ```

If `pixi` is unavailable, install it from [https://pixi.sh](https://pixi.sh) and ensure the toolchain dependencies in `pixi.toml` are met.  The build system uses CMake and Ninja under the hood, so you can fall back to manual CMake builds if necessary.

## Additional Notes
- Documentation lives in `docs/` and `tutorials/`.
- Examples demonstrating API usage can be found in `examples/`.

