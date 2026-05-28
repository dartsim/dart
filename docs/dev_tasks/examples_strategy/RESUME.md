# Resume: Examples Strategy

## Last Session Summary

PLAN-103 Phases 1–4 are landed on branch `demos-app`; Phase 5 is the explicit
"not now" retire-later checklist. The Python `dart-demos` app exists with 11
scenes (including 5 minimal-viable modern scenarios), a cross-language golden
parity smoke runs in both languages against a shared fixture, and a notebook
gallery seed lives in `python/tutorials/`. C++ `dart-demos` (PLAN-102) remains
frozen-but-present.

## Current Branch

`demos-app`. PLAN-103 commits (newest last):

- PLAN-103 docs
- `feat(py-demos)`: Phase 1 runner + 4 seed scenes + pytest cycle smoke
- `feat(demos)`: Phase 2 golden parity (Python + C++ tests, shared fixture)
- `test(demos)`: Python golden parity test (missed in prior commit)
- `feat(demos)`: Phase 3 — 5 modern Python-first scenes (11 total)
- `docs(demos)`: Phase 4 — notebook gallery seed
- (pending) Phase 5 status docs

## Immediate Next Step

Phase 5 is "not now" by design. Three follow-ups close the retire-later gates:

1. **Grow Python breadth toward C++ coverage.** Add Python scene modules for
   the categories the 11-scene Python runner currently lacks; targets are
   listed in PLAN-103's "Resolved Decisions" section.
2. **Extend the golden set.** Add `rigid_chain`, `boxes` (Python ↔ C++
   parity), `operational_space_control` mirrors and regenerate fixtures via
   `python -m examples.demos.golden._generate`. The shared-fixture mechanism is
   in place — adding a scene is `(1)` write its Python mirror, `(2)` add it to
   `GOLDEN_SCENE_IDS` in `helpers.py`, `(3)` regenerate, `(4)` update the
   hardcoded values in `tests/unit/gui/test_demos_golden_parity.cpp`.
3. **PLAN-012 + PLAN-101 work.** Cloud Colab smoke (PLAN-012) and editor scene
   loading (PLAN-101) unblock conditions 2 and 3 of the retire-later checklist.

## What Is Done

- **Phase 1 (foundation)**: `python/examples/demos/` package; runner CLI
  mirroring `dart-demos` (`--scene`, `--cycle-scenes`, `--frames`,
  `--screenshot`, `--headless`, `--list`); ordered registry; soft-fail on
  scene-build errors. `pixi run py-demos`. pytest cycle smoke
  (`python/tests/integration/test_demos_cycle.py`).
- **Phase 2 (golden parity)**: `python/examples/demos/golden/{helpers,_generate}.py`
  - `golden/hello_world.json`. Python smoke
    (`python/tests/unit/test_golden_parity.py`) and C++ smoke
    (`tests/unit/gui/test_demos_golden_parity.cpp`) both pass within 1e-9.
    Split `examples/demos` into `demos_scenes` static lib + thin `dart-demos`
    app so tests can link the registry.
- **Phase 3 (modern content)**: 5 minimal-viable scenes
  (`legged_balance`, `arm_push_box`, `cartpole_gym_env` with a Gymnasium-style
  `CartPoleEnv` class, `cartpole_mpc` with LQR, `sensor_descriptors`).
- **Phase 4 (notebooks)**: `python/tutorials/01_browse_demos.ipynb` +
  `python/tutorials/README.md` documenting local run + Colab cell pattern.
- **Phase 5 (retire)**: explicit "not now" recorded in PLAN-103 with per-gate
  status; conditions 1–3 unmet, 4–5 met.
- **Deformable consolidation**: `experimental_deformable_gui` is migrated into
  `dart-demos` as the `experimental_deformable` scene under the Soft Bodies
  category; the standalone example directory is removed, and the references in
  `examples/CMakeLists.txt`, `examples/README.md`, `scripts/run_cpp_example.py`,
  and PLAN-081 are updated to the new scene path.

## Build / Run / Verify

- Python: `pixi run py-demos -- --cycle-scenes --frames 2` (cycles 11 scenes).
- Python tests:
  `pixi run bash -lc 'export PYTHONPATH="$PWD/build/default/cpp/Release/python:$PWD/python"; python -m pytest python/tests/integration/test_demos_cycle.py python/tests/unit/test_golden_parity.py -v'`
- C++ golden test:
  `pixi run python scripts/cmake_build.py --build-dir build/default/cpp/Release --target UNIT_gui_DemosGoldenParity`
  then `ctest --test-dir build/default/cpp/Release -R UNIT_gui_DemosGoldenParity`.
- Generator: `python -m examples.demos.golden._generate` (under the same
  PYTHONPATH).

## How to Resume

```bash
git checkout demos-app
git status && git log -15 --oneline
```

Then read `../../plans/103-examples-strategy.md` and continue with the
Immediate Next Step.
