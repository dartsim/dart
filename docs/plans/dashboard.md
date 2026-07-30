# DART 6.20 Plan Dashboard

This dashboard is the operating view for release-branch planning. It points to
the owner documents that hold detailed packet boards and evidence.

Priority order is document order. Active implementation handoff remains in
`docs/dev_tasks/`; this dashboard only records the release-branch roadmap view.

### PLAN-620: Dependency Minimization And Native Collision

- Owner doc: [dependency minimization](../dev_tasks/dart6_dependency_minimization/README.md)
- Status: Planning / maintainer decision
- Horizon: Later release
- Dimension: Compatibility, dependency footprint, and downstream support.
- Next step: Ratify the Phase 5 ODE-facade versus coordinated gz-physics choice
  and the target later-release sequence. The DART 6.20 consolidation is merged;
  do not continue the port or implement the default flip on `release-6.20`.
- Gate: `pixi run lint`; default configure/build; component/package smoke for
  touched dependencies; `pixi run -e gazebo test-gz` when collision,
  constraint, package, or default-solver behavior can affect gz-physics.

### PLAN-621: Active Contact Performance Generalization

- Owner doc: [performance generalization](../dev_tasks/dart6_performance_generalization/README.md)
- Status: Active
- Horizon: Now
- Dimension: Performance, determinism, and Gazebo/gz-sim compatibility.
- Next step: #3353 (WP-PG.15/D7) and #3361 (WP-PG.14/D3) are merged. Refresh
  the WS-G cross-engine and native-collision evidence on the current
  `release-6.20` base, then select the next measured gap without retiring the
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
- Scope (2026-07-29): DART 6 carries **one** deformable model, the Jain/Liu
  point-mass surface flesh that `SoftBodyNode` implements. The Kim/Pollard
  volumetric-FEM lane was removed from DART 6 and retargeted to DART 7, because
  the two papers need different discretizations and a compatibility release
  branch should not carry two parallel deformable architectures. Whether a
  reduced FEM is still the right DART 7 target is open given newer solvers such
  as AVBD. Durable owner:
  [deformable-body design](../design/dart6_deformable_body.md); working record:
  [`decisions.md`](../dev_tasks/dart6_deformable_body_performance/decisions.md);
  scope note in
  [`10-full-parity-execution-plan.md`](../dev_tasks/dart6_deformable_body_performance/10-full-parity-execution-plan.md).
- Next step: #3382 merged as `6c88ac1d774`, and #3407 merged as
  `2ffe228c14c`, removing the volumetric FEM subsystem from DART 6. Continue the
  Jain/Liu lane with PR-3a soft-foot SIMBICON, which reuses the existing
  `atlas_simbicon` controller and soft-feet Atlas asset
  (`12-pr3a-soft-foot-simbicon.md`). **Do not restart M2.x on
  `release-6.20`.** Still open: the competitive-envelope definition, the
  four-link flexible-foot comparison, WP-DB.07 scaling, WP-DB.08
  native-owned/default coverage, a valid `bm-soft-body-paired` artifact or an
  approved disposition, and the separate `main` PR for the zero-DoF soft
  point-mass assertion. New GUI examples belong in `dart-demos`.
- Gate: `pixi run lint`; focused soft-body integration tests; headless
  soft-body benchmarks with exact commands/raw rows; one-thread and host-capped
  multi-thread determinism/scaling evidence; allocation gates and Gazebo
  coverage before any collision, constraint, or backend-default change.
