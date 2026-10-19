# Deferred CHANGELOG drafts

M1c's native state packets defer these entries to the milestone
consolidation under the program's hot-file rule:

- **Simulation:** Add C++ `World::captureState()` and `World::restoreState()`
  for dynamic World snapshots: positions, velocities, accelerations, forces,
  commands, body external forces, soft point-mass state and time. A snapshot
  restores into the same World or a structurally identical one, such as a
  clone; parameters and solver or contact history are not part of it.
  Restoring clears the contact, solver and sleep history left by earlier
  steps, and may allocate with FCL or Bullet.
  Clones omit manual constraints; re-add them in the same order with
  endpoints mapped to the clone's bodies before restoring.
  Commands follow the target's actuator types and limits; EndEffector support
  flags remain target-owned.
  Manual ball, revolute, cylindrical and weld joint constraints are
  supported, and restoring resets their previous solve impulses; custom
  joints, stateful aspects and other manual constraints are rejected before
  any state changes.

M1d's native executor prerequisite also defers this entry to the milestone
consolidation under the program's hot-file rule:

- **Simulation:** Exceptions thrown during multithreaded collision detection,
  constraint solving or World stepping now reach the caller instead of
  terminating the process, and the World stays usable.

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
  headless capture, and uses DART 7 names.
- **Breaking Changes:** Bundled Dear ImGui is now the `v1.92.8-docking`
  release, so the installed `dart/external/imgui` headers use the docking
  layout; rebuild applications that include them. `DART_IMGUI_DOCKING` is
  removed. With a system ImGui that lacks docking, dart-gui still builds, but
  dartpy omits `dartpy.viewer`.
