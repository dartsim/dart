# Deferred CHANGELOG drafts

These demo-consolidation entries move into `CHANGELOG.md` only in M6b's
milestone consolidation PR, following the program's hot-file rule.

- **Examples:** Replace the C++ `dart-demos` application with the Python demo
  catalog (`python -m examples.demos`). It runs on the same native workspace:
  - navigator and scene panels;
  - inspector, contact and drag tools, and profiler;
  - log console;
  - headless list, cycle and capture.

  `pixi run demos`, `cycle-demos` and `capture` launch it, and `py-demos` is
  removed. The `dart-demos`, `soft_bodies`, `soft_cubes` and
  `soft_open_chain` executables are removed, as is the TinyDNN-only
  `human_joint_limits` scene. Use
  `pixi run demos -- --scene soft_bodies`, with `--collision-detector` and
  `--threads` as before.
- **Python:** Add `dartpy.viewer`, a managed viewer that runs Python-defined
  scenes in the native workspace. It offers declarative panel controls and
  headless capture, and uses DART 7 names. `keyboard_capture_active()` lets
  event handlers respect the viewer's keyboard focus.
- **Python:** Add `dartpy.math.Random.uniform_float()` for single-precision
  random draws from the engine's seeded stream.
- **Python:** Add `dartpy.math.Quaternion.from_two_vectors()` to align two
  directions using the engine's quaternion routine.
- **Python:** Expose existing Skeleton sleep-candidacy and island-state
  setters, and constructors for Dantzig/PGS boxed-LCP solvers, used when waking
  a simulation after switching its constraint-solver backend.
- **Python:** Expose PointMass state copies, adaptive SoftBodyNode settings,
  checked point lookup and revolute soft-body construction through engine bindings.
- **Breaking Changes:** Bundled Dear ImGui is now the `v1.92.8-docking`
  release, so the installed `dart/external/imgui` headers use the docking
  layout; rebuild applications that include them. `DART_IMGUI_DOCKING` is
  removed. With a system ImGui that lacks docking, dart-gui still builds, but
  dartpy omits `dartpy.viewer`.
