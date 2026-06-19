# Agent Guidelines for DART

This file provides conventions and tips for agents working in this repository.  Read it before modifying code or documentation.

## Overview
- DART (Dynamic Animation and Robotics Toolkit) is a mixed C++/Python project that offers kinematic and dynamic algorithms for robotics and animation.
- The core library lives under `dart/` (C++).  Python bindings are built in `python/` and exposed as `dartpy`.

## Repository Practices
- Prefer [`rg`](https://github.com/BurntSushi/ripgrep) for searching the code base.
- Keep commits focused and run the relevant checks before committing.
- Follow the style guide in `CONTRIBUTING.md` (two‑space indent, camelCase functions, PascalCase classes, no cuddled braces).
- For DART 6.20 work, branch from `origin/release-6.20` with a topic branch,
  never by committing directly on `release-*`.
- Do not let a local topic branch track `origin/release-*`. Use
  `git switch --no-track -c <type>/<topic> origin/release-6.20`.
- Push topic branches with the same local and remote branch name:
  `branch=$(git branch --show-current); git push -u origin "HEAD:${branch}"`.

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
   pixi run build-py-dev
   ```
4. Run the test suites:
   ```bash
   pixi run test        # C++ tests
   pixi run test-py     # Python tests
   pixi run test-all    # Lint + build + tests
   ```

If `pixi` is unavailable, install it from [https://pixi.sh](https://pixi.sh) and ensure the toolchain dependencies in `pixi.toml` are met.  The build system uses CMake and Ninja under the hood, so you can fall back to manual CMake builds if necessary.

## Additional Notes
- Documentation lives in `docs/` and `tutorials/`.
- Examples demonstrating API usage can be found in `examples/`.

## AI Workflows

Release-branch AI workflow guidance lives in `docs/ai/README.md`,
`docs/ai/principles.md`, and `docs/ai/workflows.md`.

Editable sources live in `.claude/commands/` and `.claude/skills/`. Generated
Codex and OpenCode entrypoints live in `.codex/skills/` and
`.opencode/command/`; do not hand-edit generated files. Run
`pixi run check-ai-commands` to verify parity.
