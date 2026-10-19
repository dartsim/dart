# Deferred CHANGELOG drafts

M1c's native state packets defer these entries to the milestone
consolidation under the program's hot-file rule:

- **Simulation:** Add C++ `World::captureState()` and `World::restoreState()`
  for dynamic World snapshots: positions, velocities, accelerations, forces,
  commands, body external forces, soft point-mass state and time. A snapshot
  restores into the same World or a structurally identical one, such as a
  clone; parameters and solver or contact history are not part of it. Clones
  omit manual constraints; re-add them in the same order with endpoints
  mapped to the clone's bodies before restoring.
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
- **Python:** Expose owning engine configuration/composite snapshots and
  copied soft-point accelerations for recording and restoring simulation state.
- **Python:** Expose copied mesh vertex groups, integer engine random draws,
  point rendering and voxel-update preflight for Python sensor implementations.
- **Python:** Expose point mass metadata and reset operations, collision flags,
  copied soft-body point/face properties and owning triangle meshes through
  ordinary engine bindings for Python model construction and control.
- **Breaking Changes:** Bundled Dear ImGui is now the `v1.92.8-docking`
  release, so the installed `dart/external/imgui` headers use the docking
  layout; rebuild applications that include them. `DART_IMGUI_DOCKING` is
  removed. With a system ImGui that lacks docking, dart-gui still builds, but
  dartpy omits `dartpy.viewer`.
- **Breaking Changes:** Remove IKFast support: the
  `dart::dynamics::IkFast` and `SharedLibraryIkFast` classes,
  `dart::dynamics::wrapCyclicSolution`, and the installed
  `dart/dynamics/IkFast.hpp`, `dart/dynamics/SharedLibraryIkFast.hpp`,
  `dart/dynamics/ikfast.h` and `dart/external/ikfast/ikfast.h` headers.
  Implement `InverseKinematics::Analytical` for a C++ analytical solver, which
  can load a generated solver through `dart::common::SharedLibrary`, or pass a
  Python solver such as ssik to `InverseKinematics.setPythonAnalytical`.
- **Examples:** The WAM scene is now `wam_analytical_ik` and solves with
  ssik's closed-form analytical IK. Like `ssik_ik_gui`, it needs the optional
  ssik package (`pixi run install-ssik`) and is skipped without it.
