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
- Scope (2026-07-29): DART 6 carries **one** deformable model, the Jain/Liu
  point-mass surface flesh that `SoftBodyNode` implements. The Kim/Pollard
  volumetric-FEM lane was removed from DART 6 and retargeted to DART 7, because
  the two papers need different discretizations and a compatibility release
  branch should not carry two parallel deformable architectures. Whether a
  reduced FEM is still the right DART 7 target is open given newer solvers such
  as AVBD. Reasoning:
  [`decisions.md`](../dev_tasks/dart6_deformable_body_performance/decisions.md);
  scope note in
  [`10-full-parity-execution-plan.md`](../dev_tasks/dart6_deformable_body_performance/10-full-parity-execution-plan.md).
- Next step: land the removal (#3407). Then continue the Jain/Liu lane on DART 6,
  starting with PR-3a soft-foot SIMBICON, which reuses the existing
  `atlas_simbicon` controller and the soft-feet Atlas asset
  (`12-pr3a-soft-foot-simbicon.md`). **Do not restart the volumetric FEM
  subsystem on `release-6.20`.** Still open: the competitive-envelope
  definition, the four-link flexible-foot comparison, WP-DB.07 scaling, WP-DB.08
  native-owned/default coverage, a valid `bm-soft-body-paired` artifact or an
  approved disposition, and the separate `main` PR for the zero-DoF soft
  point-mass assertion. New GUI examples belong in `dart-demos`.
- Gate: `pixi run lint`; focused soft-body integration tests; headless
  soft-body benchmarks with exact commands/raw rows; one-thread and host-capped
  multi-thread determinism/scaling evidence; allocation gates and Gazebo
  coverage before any collision, constraint, or backend-default change.
