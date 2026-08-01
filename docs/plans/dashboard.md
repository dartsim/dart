# DART 6.20 Plan Dashboard

This dashboard is the operating view for release-branch planning. It points to
the owner documents that hold detailed packet boards and evidence.

Priority order is document order. Active implementation handoff remains in
`docs/dev_tasks/`; this dashboard only records the release-branch roadmap view.

### PLAN-621: Active Contact Performance Generalization

- Owner doc: [performance generalization](../dev_tasks/dart6_performance_generalization/README.md)
- Status: Active
- Horizon: Now
- Dimension: Performance, determinism, and Gazebo/gz-sim compatibility.
- Next step: Obtain approval to push `wp-pg-wsg-rebaseline-20260731`
  and open the WP-PG.50 PR: the detector stream-quality bundle restores
  full manifolds and stable cylinder contacts, **meets criterion 2 on
  its original terms** (S6 fully deactivates with zero penetration; 4/5
  seeds within the 20-second window, all tested seeds by 60 s), and
  measures slightly faster than the audited pre-consolidation stack
  (criterion 1 ≈3.7x). D9/D10 records live in the task README; the
  2026-07-31 re-baseline and first full WS-G matrix (incl. HUM rows)
  are in the task folder. Keep the task active while #3056 remains
  open.
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
- Next step: continue the Jain/Liu lane on DART 6 with PR-3a soft-foot
  SIMBICON, which reuses the existing
  `atlas_simbicon` controller and the soft-feet Atlas asset
  (`12-pr3a-soft-foot-simbicon.md`). **Do not restart the volumetric FEM
  subsystem on `release-6.20`.** Still open: the competitive-envelope
  definition, the four-link flexible-foot comparison, WP-DB.07 scaling,
  WP-DB.08 DART-owned/pre-default coverage, a valid `bm-soft-body-paired`
  artifact or an
  approved disposition, and the separate `main` PR for the zero-DoF soft
  point-mass assertion. New GUI examples belong in `dart-demos`.
- Gate: `pixi run lint`; focused soft-body integration tests; headless
  soft-body benchmarks with exact commands/raw rows; one-thread and host-capped
  multi-thread determinism/scaling evidence; allocation gates and Gazebo
  coverage before any collision, constraint, or backend-default change.

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
