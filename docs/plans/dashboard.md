# DART 6.20 Plan Dashboard

This dashboard is the operating view for release-branch planning. It points to
the owner documents that hold detailed packet boards and evidence.

Priority order is document order. Active implementation handoff remains in
`docs/dev_tasks/`; this dashboard only records the release-branch roadmap view.

### PLAN-620: Dependency Minimization And Native Collision

- Owner doc: [dependency minimization](../dev_tasks/dart6_dependency_minimization/README.md)
- Status: Active
- Horizon: Now
- Dimension: Compatibility, dependency footprint, and downstream support.
- Next step: Continue the native-collision port sequence from the
  dependency-minimization task, using DART 7 as reference evidence only and
  proving every release-branch compatibility surface directly.
- Gate: `pixi run lint`; default configure/build; component/package smoke for
  touched dependencies; `pixi run -e gazebo test-gz` when collision,
  constraint, package, or default-solver behavior can affect gz-physics.

### PLAN-621: Active Contact Performance Generalization

- Owner doc: [performance generalization](../dev_tasks/dart6_performance_generalization/README.md)
- Status: Active
- Horizon: Now
- Dimension: Performance, determinism, and Gazebo/gz-sim compatibility.
- Next step: Finish #3353 as the WP-PG.15/D7 evaluator and restoration PR,
  then choose the default-fix route from its S6 evidence without retiring the
  task while #3056 remains open.
- Gate: `pixi run lint`; capped C++ build; detector-specific final-state
  hash guards; benchmark evidence in the task-required report shape;
  `pixi run -e gazebo test-gz` for collision, solver, or `World::step`
  changes.

### PLAN-622: DART 6 Deformable Body Feature And Performance

- Owner doc: [deformable body performance](../dev_tasks/dart6_deformable_body_performance/README.md)
- Status: Active
- Horizon: Next
- Dimension: Research feature parity, CPU performance, and compatibility.
- Next step: Publish the verified `origin/release-6.20@75306efe770` merge and
  test-only Windows legacy-FCL CoP calibration, then shepherd #3382 through a
  fresh exact-head automated review and hosted CI. Keep PLAN-622 active after
  that representative release slice: capture the reviewed
  `bm-soft-body-paired` artifact only when the host passes its load/thermal
  gates (or obtain explicit maintainer disposition), decide the competitive
  envelope and the four-link flexible-rigid-foot versus deformable-foot row,
  retain WP-DB.07 larger-workload scaling and WP-DB.08 native-owned
  coverage/default work, and track a separate `main` PR for the zero-DoF soft
  point-mass assertion corrected by release commit `10c6b6055e4`. New GUI
  examples in this lane belong in `dart-demos`, not standalone executables.
- Gate: `pixi run lint`; focused soft-body integration tests; headless
  soft-body benchmarks with exact commands/raw rows; one-thread and host-capped
  multi-thread determinism/scaling evidence; allocation gates and Gazebo
  coverage before any collision, constraint, or backend-default change.
