# Resume: Examples Strategy

## Last Session Summary

Phase 1 of PLAN-103 landed: `python/examples/demos/` has a headless runner +
registry + 4 seed scenes (hello_world from the old example, three sx demos from
the experimental_rigid_body example); `pixi run py-demos` works; a pytest cycle
smoke (`python/tests/integration/test_demos_cycle.py`) verifies the runner.
The old `python/examples/{hello_world,experimental_rigid_body}` dirs were
removed (migrated). C++ `dart-demos` (PLAN-102) is unchanged and frozen.

## Current Branch

`demos-app` — Phase 1 commits land alongside the prior PLAN-102 work.

## Immediate Next Step

**Phase 2 (golden parity harness).** Define the 4 golden scenes
(`hello_world`, `rigid_chain`, `boxes`, `operational_space_control`); add a
fixture-generator script that runs each from C++ and Python deterministically
(fixed timestep, fixed step count, no RNG) and writes the expected state to
`python/examples/demos/golden/<scene>.json`; add `python/tests/unit/test_golden_parity.py`
asserting the Python golden scenes match the fixtures within tolerance; mirror
the assertion on the C++ side (`UNIT_demos_golden_parity` or extend an existing
GUI test target) reading the same fixtures.

For the screenshot floor, check the `dartpy.gui` headless surface
(`python/dartpy/gui/module.cpp`, `descriptors.cpp`) and extend it minimally if a
non-blank PPM screenshot from a scene is not yet reachable from Python.

## Context That Would Be Lost

- The Python scene contract: each scene module exposes module-level constants
  (`ID`, `TITLE`, `CATEGORY`, `SUMMARY`) and a `build() -> SceneSetup` where
  `SceneSetup` carries the world + optional custom `step(n)` + optional `info`.
  Registry is an explicit ordered list in `registry.py`.
- The runner CLI mirrors C++ `dart-demos` exactly: `--scene <id>`,
  `--cycle-scenes`, `--frames N`, `--screenshot <path>`, `--headless` (no-op;
  Python is always headless).
- Screenshot in Phase 1 writes a JSON state snapshot at the path; a real
  non-blank PPM via `dartpy.gui` is Phase 2 work.
- Source of truth for common example content is **Python**; C++ golden scenes
  are the frozen mirror, asserted against the shared fixture.
- `dartpy` uses both snake_case and CamelCase during the DART 7 transition
  (hello_world's `parseSkeleton`/`getName`/`getPositions` are camelCase; sx uses
  snake_case `add_multibody`, `mass`, `inertia`).

## Build / Run / Verify

- Build dartpy + run the demos cycle: `pixi run py-demos -- --cycle-scenes --frames 2`
- Smoke: `pixi run test-py -- python/tests/integration/test_demos_cycle.py`
- Standalone: `PYTHONPATH=build/default/cpp/Release/python python -m examples.demos --scene hello_world --frames 5`
  (run from `python/`).

## How to Resume

```bash
git checkout demos-app
git status && git log -10 --oneline
```

Then read `README.md` and continue with "Immediate Next Step".
