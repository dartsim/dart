# DART 6.20 Plan Dashboard

This dashboard is the operating view for release-branch planning. It points to
the owner documents that hold detailed packet boards and evidence.

Priority order is document order. Active implementation handoff remains in
`docs/dev_tasks/`; this dashboard only records the release-branch roadmap view.

### PLAN-623: Active Contact Performance Generalization

- Owner doc: [performance methodology](../onboarding/profiling.md#performance-methodology)
- Status: Proposed
- Horizon: Next (DART 6.21)
- Dimension: Performance, determinism, and Gazebo/gz-sim compatibility.
- Predecessor: [PLAN-621: DART 6.20.0 performance closeout](archive.md#plan-621-dart-6200-performance-closeout).
- Next step: Continuing [#3056](https://github.com/dartsim/dart/issues/3056)
  performance work targets 6.21, including representative same-host workload
  evidence, small-scene overhead, and time outside `World::step`. Open a new
  task home only when bounded follow-up needs multi-session tracking.
- Gate: Same-host DART revision comparisons, detector-specific behavior guards,
  and Gazebo-path compatibility evidence under the performance methodology.

### PLAN-622: DART 6 Deformable Body Feature And Performance

- Owner doc: [deformable body performance](../dev_tasks/dart6_deformable_body_performance/README.md)
- Status: Active
- Horizon: Next (DART 6.21)
- Dimension: Research feature parity, CPU performance, and compatibility.
- Scope: Jain/Liu point-mass surface flesh only; do not restart volumetric FEM
  on the DART 6 line. See the
  [deformable-body design](../design/dart6_deformable_body.md).
- Evidence: Adaptive active vertices, CoP/force variance, and the LCP
  initial-point reset proxy have representative gates. Reduced demos do not
  establish full paper parity.
- Next step: The unmet full-parity 6.20 goal is retargeted to 6.21 by maintainer
  decision. Remaining rows are motor-noise push recovery, noisy-floor biped,
  soft-contact walking, four-link flexible-foot comparison, and hand/arm
  scenes (finger flick, arm fold, pinch grasp). Push recovery remains partial:
  single-trajectory thresholds do not establish robust parity (see #3431).
  Also complete normalized performance acceptance, multicore scaling evidence
  or an approved negative disposition, pre-default detector coverage gates,
  and a complete paired benchmark artifact or approved disposition. New GUI
  examples belong in `dart-demos`.
- Gate: Focused soft-body tests; per-row correctness and same-host CPU evidence;
  one-thread and host-capped multi-thread determinism/scaling; allocation and
  Gazebo gates before any collision, constraint, or backend-default change.
  Parity rows close only with evidence; dispositions apply only where their
  acceptance rules permit them.

### PLAN-620: Dependency Minimization And Collision Backends

- Owner doc: [DART 6 collision backends](../design/dart6_collision_backends.md)
- Status: Parked
- Horizon: Parked
- Dimension: Compatibility, dependency footprint, and downstream support.
- Next step: Wait for an explicitly authorized future release line and
  milestone before proposing the default flip. DART 6.20 stops with
  `DARTCollisionDetector` selected by `"dart"` while FCL remains the default
  and a core dependency.
- Gate: `pixi run lint`; default configure/build; component/package smoke for
  touched dependencies; `pixi run -e gazebo test-gz` when collision,
  constraint, package, or default-solver behavior can affect gz-physics.
