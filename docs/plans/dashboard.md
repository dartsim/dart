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
- Next step: Re-baseline the WS-G cross-engine matrix and `dart` detector
  rows on the current merged base, then use that evidence to select one
  consolidated implementation gap or the closeout route. Keep the task active
  while #3056 remains open.
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
- Next step: PR-3a soft-foot SIMBICON shipped (#3408, #3423): both Jain/Liu
  biped claims reproduce and are gate-asserted — contacts 51.2 soft vs 15.64
  rigid, recoverable push 18000 N soft vs 8000 N rigid
  (`12-pr3a-soft-foot-simbicon.md`, resolved decisions). Goal (maintainer
  direction, 2026-08-01): fully complete this task for the DART 6.20
  release, bundled into as few PRs as review quality allows — parity rows
  close only with gate evidence, and dispositions apply only to the
  acceptance items whose own text offers one. The ordered inventory of
  remaining items and the suggested PR bundles live in the task `RESUME.md`
  (single owner); this dashboard deliberately does not duplicate that list.
  **Do not restart the volumetric FEM subsystem on `release-6.20`.** New GUI
  examples belong in `dart-demos`.
- Gate: `pixi run lint`; focused soft-body integration tests; headless
  soft-body benchmarks with exact commands/raw rows; one-thread and host-capped
  multi-thread determinism/scaling evidence; allocation gates and Gazebo
  coverage before any collision, constraint, or backend-default change.

### PLAN-623: Citation-Driven Contact Trust

- Owner doc:
  [DART 6 citation-driven contact trust](../design/dart6_citation_driven_contact_trust.md)
- Status: Active
- Horizon: Now
- Dimension: Contact correctness, evidence, and LTS compatibility.
- Next step: The branch-local claim manifest, fail-closed
  `pixi run check-citation-evidence` gate, and permanent negative control
  landed with the 2026-08-14 audit (PLAN-621 owns CT-018, PLAN-622/PR #3431
  own CT-020, PR #3377 owns exact-Coulomb fixtures). Next: produce the first
  complete `release-6.20` evidence packet for a first-wave row not owned
  elsewhere (CT-001 rolling-direction across supported detectors), then the
  remaining first-wave contact rows. Active handoff:
  `docs/dev_tasks/dart6_citation_contact_trust/`.
- Gate: Preserve C++17, pybind11, ABI, installed components, FCL default, OSG,
  and unaffected behavior; every packet is `release-6.20`-qualified with
  explicit detector/solver/timestep/seed/claim boundaries and deterministic or
  ensemble evidence; `pixi run lint`, focused tests,
  `pixi run check-citation-evidence`, and
  `pixi run -e gazebo test-gz` for downstream-sensitive changes.

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
