# DART Plan Dashboard

This dashboard is the operating view for DART development and release planning. It points to
the owner documents that hold detailed packet boards and evidence.

Priority order is document order. Active implementation handoff remains in
`docs/dev_tasks/`; this dashboard only records the roadmap view.

### PLAN-624: DART 7.0 Python-first Program

- Owner doc: [DART 7.0 project home](../dev_tasks/dart7_python_first/README.md);
  durable rules in the [DART 7 Python API design](../design/dart7_python_api.md).
- Status: Active
- Horizon: Now
- Dimension: Python-first API, core readiness for multi-physics, batching and
  replay, and Gazebo/gz-physics compatibility.
- Scope: Main's next release becomes DART 7.0. That release is a clean Python
  API break; the C++ API stays compatible for the gz-used subset. Main's
  unreleased work ships in 7.0, and `release-6.20` is the 6.x maintenance line.
- Demo consolidation: D31b's Python catalog replaces the C++ app; its
  temporary project home is retired. See the
  [host and model architecture](../onboarding/architecture.md#python-demo-catalog)
  and [remaining M6b scope](../dev_tasks/dart7_python_first/README.md#demo-consolidation-d31b).
- Next step: four chains run in parallel, Python first. The project home's
  [status](../dev_tasks/dart7_python_first/README.md#status) maps them.
  1. Python API (M5): the snapshot, package, arrays, errors and `Transform`
     packets and the naming rulebook (#3956) have merged. The facade slice
     and the remaining API areas come next.
  2. Native core (M1): compile and edit (M1b) and counted stepping (M1d,
     #3927) have merged. The remapping clone (M1i) and `WorldBatch` come next,
     then actuators, sensors and checkpoints.
  3. Multi-solver (M1e): the typed solver policy, the written contract and
     the rigid domain boundary.
  4. Rendering (M6b) through checkpoint C2.

  Step-path PRs land in the program's order, each re-qualified on current
  `main`; see the [M1 packet table](../dev_tasks/dart7_python_first/RESUME.md).
- Gate: Per-packet gates and two clean reviews; bit-identical refactors per
  detector; zero new steady-state allocations; Gazebo lanes for every core
  change.

### PLAN-625: DART 7 Dependency Floor And Minimization

- Owner doc: [DART 7 dependency project home](../dev_tasks/dart7_dependencies/README.md)
- Status: Active
- Horizon: Now
- Dimension: Dependency footprint, platform floor, packaging, and Gazebo
  compatibility.
- Scope: Ubuntu 24.04 becomes `main`'s oldest supported platform, with
  toolchain, CMake, Python and library floors raised to match; dependencies
  that the gz-facing C++ API does not bind are removed or owned in-tree; and
  DART's own code may build as C++20 while the exported interface stays
  C++17, subject to the maintainer decisions recorded in the project home.
  FCL and the collision-backend lifecycle stay with PLAN-620.
- Next step: the remaining packets, with DEP-21 (DART's own targets as C++20)
  last. Every PR listed in the project home has merged, including both
  release-6.20 backports, and no maintainer decision is queued; see its
  [status](../dev_tasks/dart7_dependencies/README.md#status).
- Gate: Per-packet gates in the project home, two clean reviews, and the
  Gazebo lanes against each packet's base for any change to installed
  headers, exported targets or the package config.

### PLAN-623: Active Contact Performance Generalization

- Owner doc: [performance methodology](../onboarding/profiling.md#performance-methodology)
- Status: Proposed
- Horizon: Next
- Dimension: Performance, determinism, and Gazebo/gz-sim compatibility.
- Predecessor: [PLAN-621: DART 6.20.0 performance closeout](archive.md#plan-621-dart-6200-performance-closeout).
- Next step: Fold the continuing [#3056](https://github.com/dartsim/dart/issues/3056)
  performance work into PLAN-624's executor and dense-data milestones. That
  covers representative same-host workload evidence, small-scene overhead, and
  time outside `World::step`. Fixes needed on the maintenance line are backported
  to `release-6.20`.
- Gate: Same-host DART revision comparisons, detector-specific behavior guards,
  and Gazebo-path compatibility evidence under the performance methodology.

<a id="plan-622-dart-6-deformable-body-feature-and-performance"></a>

### PLAN-622: DART 7.0 Deformable Body Feature And Performance

- Owner doc: [deformable body performance](../dev_tasks/dart6_deformable_body_performance/README.md)
- Status: Active
- Horizon: Next
- Dimension: Research feature parity, CPU performance, and compatibility.
- Scope: Jain/Liu point-mass surface flesh only. Volumetric FEM is outside this
  DART 7.0 plan's scope. See the
  [deformable-body design](../design/dart6_deformable_body.md).
- Evidence: Adaptive active vertices, CoP/force variance, and the LCP
  initial-point reset proxy have representative gates. Reduced demos do not
  establish full paper parity.
- Next step: The unmet full-parity 6.20 goal now targets DART 7.0, since there is
  no 6.21 release. Step-path changes outside the parity rows land in PLAN-624's
  program-owned order. Remaining rows are motor-noise push recovery,
  noisy-floor biped, soft-contact walking, four-link flexible-foot comparison,
  and hand/arm scenes (finger flick, arm fold, pinch grasp). Push recovery remains partial:
  single-trajectory thresholds do not establish robust parity (see #3431).
  Also complete normalized performance acceptance, multicore scaling evidence
  or an approved negative disposition, pre-default detector coverage gates,
  and a complete paired benchmark artifact or approved disposition. New GUI
  examples are Python scenes in the demo catalog (`python/examples/demos`),
  with controllers and demo models written in Python over the engine, IO,
  and GUI bindings. No demo implementation remains in C++.
- Gate: Focused soft-body tests; per-row correctness and same-host CPU evidence;
  one-thread and host-capped multi-thread determinism/scaling; allocation and
  Gazebo gates before any collision, constraint, or backend-default change.
  Parity rows close only with evidence that meets the owner doc's
  [acceptance gates](../dev_tasks/dart6_deformable_body_performance/README.md#acceptance);
  only multicore scaling and the paired benchmark artifact accept an approved
  disposition.

### PLAN-620: Dependency Minimization And Collision Backends

- Owner doc: [DART 6 collision backends](../design/dart6_collision_backends.md)
- Status: Active
- Horizon: Next
- Dimension: Compatibility, dependency footprint, and downstream support.
- Next step: Under PLAN-624's D13 amendment, the C++ `World` keeps FCL as its
  default in 7.0 for Gazebo compatibility, while dartpy's `dart.World` and the
  MJCF world loader default to the DART detector in 7.0. That Python switch
  lands before PLAN-624's API freeze and only after:
  - a published shape and query capability matrix;
  - strict rejection, when a World compiles, of shapes the native detector
    cannot handle (heightmaps today).

  Switching the C++ default to DART is a labeled 7.x change that also needs
  representative behavior comparisons against FCL and the deformable-body
  pre-default coverage gate. FCL remains a core dependency.
- Gate: `pixi run lint`; default configure/build; component/package smoke for
  touched dependencies; `pixi run -e gazebo test-gz` when collision,
  constraint, package, or default-solver behavior can affect gz-physics.
