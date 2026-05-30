# Resume: Examples Strategy

## Last Session Summary

PLAN-103 Phases 1–4 are landed on `main` (PR #2762). Phase 5 is the explicit
"not now" retire-later checklist. The Python `dart-demos` app now hosts 37
scenes (vs. 41 in C++ — 90% coverage) including modern scenarios and Tier
A/B/C C++ ports; a cross-language golden parity smoke runs in both languages
against shared fixtures (3 scenes wired); and a notebook gallery seed lives
in `python/tutorials/`. C++ `dart-demos` (PLAN-102) remains frozen-but-present.

Branch `examples-strategy-breadth` adds 9 more Python scenes
(`hybrid_dynamics`, `lcp_physics`, `heightmap`, `point_cloud`, `vehicle`,
`biped_stand`, `experimental_rigid_body_gui`, `collision_sandbox`,
`joint_constraints`) plus `HeightmapShape` / `PointCloudShape` dartpy
shape bindings and `BodyNode::{addExtForce, setExtForce,
clearExternalForces}` + `Skeleton::{getConstraintForces, getCOM,
clearExternalForces}` bindings that unblock the SPD biped controllers.

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

1. **Grow Python breadth toward C++ coverage.** The Python runner now hosts
   38 of 41 C++ scenes (93 %). Branch `examples-strategy-breadth` added 9
   more ports (`hybrid_dynamics`, `lcp_physics`, `heightmap`, `point_cloud`,
   `vehicle`, `biped_stand`, `experimental_rigid_body_gui`,
   `collision_sandbox`, `joint_constraints`) and the
   `HeightmapShape`/`PointCloudShape` shape bindings plus
   `BodyNode::{addExtForce, setExtForce, clearExternalForces}` and
   `Skeleton::{getConstraintForces, getCOM, clearExternalForces}` bindings
   that unlocked the SPD biped controller scenes. The remaining C++ scenes
   need new bindings or controller infrastructure that is out of scope for a
   pure Python port:
   - **Viewer-callback / interactive UI** — `drag_and_drop`, `imgui`,
     `tinkertoy`, `simulation_event_handler`. Need Python bindings for
     `dart::gui::KeyboardAction`, gizmo callbacks, and event handlers
     threaded through `ApplicationOptions`.
   - **Custom `ConstraintBase` subclass** — `human_joint_limits`. Needs a
     trampoline binding for `dart::constraint::ConstraintBase` so a
     `NeuralJointLimitConstraint` analogue can be written in Python.
   - **Heavyweight puppet IK / mocap controllers** — `atlas_puppet`,
     `hubo_puppet`, `fetch`, `g1_puppet`. Need gizmo + IK pendant +
     MJCF/ReadOptions package-resolver bindings.
   - **SIMBICON walking controller in C++** — `atlas_simbicon`. The
     controller and state-machine sources live under
     `tests/integration/atlas_simbicon/` and are not Python-bound.
   - **Runtime-loaded analytical IKFast** — `wam_ikfast`. Needs a
     `SharedLibraryIkFast` binding.
   - **`sx::World` deformable mesh build** — `experimental_deformable`.
     `DeformableBody`/`DeformableBodyOptions` bindings exist but the scene
     also depends on a tet-mesh construction helper (or a Python adapter to
     `dartpy.simulation_experimental.load_deformable_scene`).
2. **Extend the golden set.** `boxes` and `rigid_chain` are now wired. The C++
   `rigid_chain` scene replaced `dart::math::Random::uniform` with a
   deterministic damped-sine initial pose so cross-language state matches; the
   Python mirror loads `dart://sample/skel/chain.skel` via
   `dartpy.utils.SkelParser.read_world` and applies the same sine.
   `operational_space_control` is wired in Python (the new
   `examples/demos/scenes/operational_space_control.py` mirror exists, and the
   prerequisite dartpy bindings — `Skeleton::getMassMatrix`,
   `Skeleton::getCoriolisAndGravityForces`, `Skeleton::setForces`,
   `BodyNode::getLinearVelocity(offset)` — were added in
   `python/dartpy/dynamics/{skeleton,body_node}.cpp`). It is **not** in
   `GOLDEN_SCENE_IDS` because the OSC controller diverges between languages: at
   t=0 the Python controller's pos_err is exactly zero (numpy's matrix order
   produces bit-exact cancellation of the (0.05, 0, 0) offset), so the +cg
   term holds the arm motionless, while the C++ Eigen path picks up
   floating-point noise that amplifies through the Kp=50 gain into ~1e-3
   joint drift over 60 steps. Closing the parity requires either reordering
   the Python operations to match Eigen bit-for-bit or relaxing the per-scene
   tolerance for OSC; both are deliberate decisions, not mechanical follow-ups.
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
- **`py-demos` is interactive (Phase 2 viewer)**: the runner now delegates to
  `dartpy.gui.run_demos` (new C++ bindings in `python/dartpy/gui/viewer.cpp`)
  so `pixi run py-demos` opens the same Filament + ImGui multi-scene viewer
  as `pixi run demos`, with the Python scene catalog. The CLI is unchanged;
  `--screenshot <path>` now writes a real PPM via Filament instead of a JSON
  state stub (the old `_write_screenshot` JSON path is retired). Python-side
  controllers attached as `SceneSetup.step` / `SceneSetup.pre_step` are
  invoked by the headless runner used by golden-parity tests, but the
  interactive viewer doesn't forward them yet — a future binding pass should
  thread a Python `preStep` callable through `dart::gui::ApplicationOptions`.

## Build / Run / Verify

- Python (interactive): `pixi run py-demos` opens the Filament viewer with
  all 29 Python scenes (pick a scene from the Demos sidebar, same UX as
  `pixi run demos`).
- Python (headless): `pixi run py-demos -- --headless --cycle-scenes --frames 2`
  cycles every scene through the real renderer and exits.
- Python (single-scene screenshot): `pixi run py-demos -- --scene <id> --headless --width 640 --height 360 --screenshot <path>` writes a real PPM.
- Python catalog only (no viewer): `pixi run py-demos -- --list`.
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
