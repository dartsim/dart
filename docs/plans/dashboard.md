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
- Next step: Complete M0: release-line rollover, project home, baseline pin,
  compatibility shim and bug-fix packets.
- Gate: Per-packet gates and two clean reviews; bit-identical refactors per
  detector; zero new steady-state allocations; Gazebo lanes for every core
  change.

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
  no 6.21 release. Core changes outside the parity rows pause during PLAN-624's
  step-path quiet window. Remaining rows are motor-noise push recovery,
  noisy-floor biped, soft-contact walking, four-link flexible-foot comparison,
  and hand/arm scenes (finger flick, arm fold, pinch grasp). Push recovery remains partial:
  single-trajectory thresholds do not establish robust parity (see #3431).
  Also complete normalized performance acceptance, multicore scaling evidence
  or an approved negative disposition, pre-default detector coverage gates,
  and a complete paired benchmark artifact or approved disposition. New GUI
  examples are Python scenes in the demo catalog (`python/examples/demos`),
  with heavy controllers and models kept native (D31b).
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
- Next step: DART 7.0 is the authorized release line for the default flip,
  under PLAN-624's capability gate. Prerequisites:
  - a published shape and query capability matrix;
  - strict rejection of shapes the native detector cannot handle (heightmaps
    today);
  - representative behavior comparisons;
  - the deformable-body pre-default coverage gate.

  If the gate does not pass by the M0 decision date, FCL stays the 7.0 default and
  the flip moves to a later 7.x release. FCL remains a core dependency.
- Gate: `pixi run lint`; default configure/build; component/package smoke for
  touched dependencies; `pixi run -e gazebo test-gz` when collision,
  constraint, package, or default-solver behavior can affect gz-physics.
