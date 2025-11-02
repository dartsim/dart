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

## Submitting Changes
- Always create a fresh topic branch from `main` before making edits:
  ```bash
  git checkout main
  git pull --rebase
  git checkout -b <topic-branch-name>
  ```
- Keep work in progress off `main`. Commit locally with focused messages, then push the branch (`git push origin <topic-branch-name>`).
- Review your diff (`git status`, `git diff`) and run the relevant `pixi` checks prior to committing.
- Open a draft PR using the template in `.github/PULL_REQUEST_TEMPLATE.md`. Replace the placeholder summary text instead of leaving it quoted and tick only the checklist items you completed.
- Convert the PR from draft once validation passes and the change is ready for review.
