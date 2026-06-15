# PLAN-091: DART 7 Architecture Hardening

- Operating state: `PLAN-091` in [`dashboard.md`](dashboard.md)
- Outcome: the DART 7 architecture documented in the design docs exists in
  code — an internal solver contract with one selection idiom, a physical
  Model/State/Control split with stable identity, a real compute-executor
  axis, an apples-to-apples metrics/scene substrate, and a facade with no
  solver vocabulary — so new solver families land through contracts instead
  of copying anti-patterns.
- Current evidence: the verified findings, competitor evidence, and standing
  rule in
  [`../design/dart7_architecture_assessment.md`](../design/dart7_architecture_assessment.md)
  (June 2026 architecture review; every critical/high finding adversarially
  verified against code).

## Owner Docs

- [`../design/dart7_architecture_assessment.md`](../design/dart7_architecture_assessment.md)
  — findings, standing rule, competitor lessons (the "why").
- [`../design/simulation_solver_architecture.md`](../design/simulation_solver_architecture.md)
  — target solver/coupling architecture the packets realize.
- [`../design/simulation_cpp_api.md`](../design/simulation_cpp_api.md),
  [`../design/simulation_python_api.md`](../design/simulation_python_api.md)
  — facade rules WS4 enforces.
- [`../design/scalable_compute_decisions.md`](../design/scalable_compute_decisions.md)
  — compute decision framework WS3 follows.
- [`../ai/orchestration.md`](../ai/orchestration.md) — the work-packet
  contract and orchestrator/executor roles this plan uses.

## Operating Model

This plan is executed as orchestrator-authored work packets per
[`../ai/orchestration.md`](../ai/orchestration.md). Each packet below follows
the packet contract (objective, scope, non-goals, acceptance evidence, gates,
dependencies). Executors pick up packets via `$dart-execute-packet` /
`/dart-execute-packet`; the orchestrator reviews against acceptance evidence
before a packet is marked done. Packet state is tracked by editing this file:
an executor starting a packet appends `[claimed]` to the packet heading and
records its results as an `Evidence:` bullet at the end of the packet; on
acceptance the orchestrator replaces the marker with
`[done — <evidence link>]`. Each packet's own Dependencies line is the single
source of truth for availability; within a workstream, available packets are
picked in document order.

Sequencing rule: WS0 runs first in priority order, and the behavior-lock
guardrail (WP-091.2 golden trajectories) hard-gates the refactor-heavy
packets through their own Dependencies lines — the only availability source.
The remaining WS0 packets are sequenced ahead of later workstreams by
document order but are deliberately not blockers, so a stalled
evidence-integrity packet (for example one waiting on a maintainer decision)
does not freeze WS1–WS4. The standing rule in
the assessment doc applies for the lifetime of this plan: new solver-family
work routes through
[`solver-family-intake.md`](solver-family-intake.md) and must not bypass the
contracts this plan is landing.

## Workstreams

### WS0 — Evidence integrity and guardrails

#### WP-091.1 Solver-identity recording and AVBD evidence relabel [done — PR #2990, merged 2026-06-13]

- Objective: every benchmark evidence packet machine-records the resolved
  solver configuration, and the AVBD contact-scene rows that timed the
  sequential-impulse path are relabeled.
- Scope: (a) identify the affected rows first — list every AVBD contact-scene
  packet row whose scene never emplaces the AVBD rigid-contact config (the
  per-body component that activates AVBD contact), and record that row list
  as packet evidence before editing; (b) add a resolved-solver-identity field
  to the AVBD packet-writer family's shared schema (bump its
  `json_schema_version`; other families adopt the same field contract in
  follow-up packets — do not sweep all writer scripts here) and extend the
  AVBD packet checker script to reject identity-less packets at the new
  schema version (version-gated: existing committed packets at older versions
  remain readable but their rows are relabeled in prose); (c) relabel the six
  mirrored prose claim sites — the PLAN-104 dashboard entry,
  [`104-vertex-block-descent-solver.md`](104-vertex-block-descent-solver.md)
  itself (contact-scene "faster than native" rows), the
  `104-vertex-block-descent-solver/` corpus sidecar and the AVBD paper-gap
  audit, and `docs/dev_tasks/avbd_solver/` `README.md`/`RESUME.md` (prepend a
  status entry per that task's house style, including its
  does-not-close-gates disclaimer). Leave committed `avbd-*-packet.json`
  bytes untouched; the relabel applies to prose surfaces only.
- Non-goals: promoting AVBD contact to a facade family (WS1); rerunning the
  native-reference benchmarks; migrating non-AVBD packet writers.
- Acceptance evidence: the affected-row list recorded in this packet's
  Evidence bullet; AVBD schema version bump with the identity field; a
  checker unit test (pytest under `tests/`, run via
  `pixi run python -m pytest`) proving identity-less new-version packets are
  rejected; all six mirrored claim sites relabeled consistently.
- Gates: `pixi run lint`, `pixi run check-docs-policy`, the new checker test.
- Dependencies: none. Hazards: curated `avbd-*-packet.json` files are
  referenced by tests and allowlisted in `.gitignore`; do not rename, move,
  or edit them; keep the PLAN-104 completion-gate framing consistent across
  all six sites; some warm-started AVBD contact paths do run through the
  default step, so classify rows by config-component presence, not by scene
  name.
- Evidence: affected rows were classified by config-component presence —
  `tests/benchmark/simulation/bm_avbd_rigid_fixed_joint.cpp` and the py-demo
  scenes never emplace `comps::RigidAvbdContactConfig` (the benchmark TU's
  only internal-AVBD use is `classifyAvbdRigidWorldEndpoint`), and
  `dart/simulation/compute/world_step_stage.cpp` routes every contact set
  without that config to the sequential-impulse path while keeping AVBD
  joint rows active. Affected contact rows: pure-contact (no AVBD code in
  the timed solve) — `avbd-demo2d-{dynamic-friction, static-friction,
pyramid, cards, stack, stack-ratio}` and `avbd-demo3d-{ground,
dynamic-friction, static-friction, pyramid, stack, stack-ratio}`;
  joint-plus-contact (AVBD point-joint/motor/spring rows with
  sequential-impulse contacts) — `avbd-demo2d-{fracture, soft-body,
joint-grid, net}` and `avbd-demo3d-{soft-body, bridge, breakable}`; chain
  rows whose edge-touching links' incidental contacts also ran sequential
  impulse — `avbd-demo2d-{rod, rope, heavy-rope, hanging-rope}` and
  `avbd-demo3d-{rope, heavy-rope}`. Fifteen of these carried "faster than
  native" claims (2D Fracture, Dynamic Friction, Static Friction, Pyramid,
  Stack, Stack Ratio; 3D Ground, Dynamic Friction, Static Friction, Pyramid,
  Stack, Stack Ratio, Soft Body, Bridge, Breakable). Schema:
  `scripts/avbd_packet_schema.py` bumps the AVBD packet family to schema
  version 2 with the required `resolved_solver_identity` field contract;
  `scripts/check_avbd_packets.py` (`pixi run check-avbd-packets`, in the
  `check-lint` aggregate) rejects identity-less packets at version >= 2 and
  new packet files below version 2 while the 48 committed v1 packets stay
  readable; `tests/test_check_avbd_packets.py` (9 pytest cases via
  `pixi run python -m pytest`) proves the rejection and acceptance paths,
  including the avbd-claim-without-emplaced-config consistency rule. All six
  mirrored claim sites relabeled consistently (dashboard PLAN-104 entry,
  `104-vertex-block-descent-solver.md`, the demo-corpus sidecar, the AVBD
  paper-gap audit, and the dev-task `README.md`/`RESUME.md` status entries
  with the does-not-close-gates disclaimer); committed `avbd-*-packet.json`
  bytes untouched. Gates: `pixi run lint`, `pixi run check-docs-policy`,
  `pixi run check-avbd-packets`, and
  `pixi run python -m pytest tests/test_check_avbd_packets.py` green.
  Writer-migration scope: per packet Scope (b) ("do not sweep all writer
  scripts here"), this packet establishes the shared schema contract
  (`scripts/avbd_packet_schema.py`), the enforcing checker, and the relabel
  only. The `scripts/write_avbd_*_packet.py` scripts still emit
  `schema_version: 1` and do not yet import the shared schema module or write
  `resolved_solver_identity`; the 48 committed v1 packets stay valid through
  the checker's legacy allowlist. Wiring the writer family to emit schema
  version 2 with a machine-captured identity (which requires running each
  scene under instrumentation and regenerating its packet bytes) is deferred
  follow-up work, not done here — track it as a follow-up packet before the
  next AVBD evidence refresh; until then, the contract binds new packet files
  via the checker rather than via the writers. Acceptance status: implemented
  with the focused gates above green and accepted by the maintainer via the
  merge of PR #2990 (2026-06-13) — the implementing session did not
  self-approve; the maintainer merge is the independent acceptance recorded by
  the `[done]` heading marker.

#### WP-091.2 Golden trajectories for the default step [done — PR #2994, merged 2026-06-14]

- Objective: committed reference trajectories (states, contact counts,
  tolerances) for a small scene matrix lock the default `World::step`
  behavior before restructuring begins.
- Scope: a fixture corpus seed under `tests/`; a regression test comparing
  trajectories across runs and recording the tolerance policy.
- Non-goals: the full scene-corpus/harness substrate (WP-091.24);
  cross-solver comparison.
- Acceptance evidence: golden files committed; regression test passes and
  fails on an injected physics change (demonstrated once in the packet
  evidence).
- Gates: `pixi run lint`, `pixi run build`, `pixi run test-unit`.
- Dependencies: none. Later refactor packets declare this packet in their own
  Dependencies lines, which are the source of truth. Handoff notes (verified
  2026-06-13): no golden/reference-trajectory fixture exists under `tests/`
  yet, so this seeds the first; follow the committed fixture convention —
  tab-separated `.tsv` under `tests/fixtures/` (pattern:
  `tests/fixtures/rigid_ipc/wrecking_ball_ccd.tsv`), not JSON; model the
  regression test on
  `tests/unit/simulation/world/test_world_contact_parity.cpp` (the `world/`
  directory is auto-registered via `dart_add_unit_test_dir(world …)` in
  `tests/unit/simulation/CMakeLists.txt`, so no per-directory CMake edit is
  needed); record per-step state (position, velocity, contact count, time)
  and the tolerance policy in the fixture/test.
- Evidence: `tests/unit/simulation/world/test_world_default_step_golden.cpp`
  (ctest `test_world_default_step_golden`, label `simulation`,
  `DefaultStepGolden.{FreeFall,SphereOnGround,BoxOnGround}`) locks the default
  `World::step` over a three-scene matrix using public facade headers only
  (`world.hpp`, `world_options.hpp`, `body/rigid_body.hpp`,
  `body/rigid_body_options.hpp`, `body/collision_shape.hpp`,
  `body/contact.hpp`, `frame/frame.hpp`; no `detail/` or `compute/` includes):
  free-fall (semi-implicit integrator,
  0 contacts), a sphere dropped onto a static box ground (contact onset at
  frame 99, count 0→1), and a box dropped onto ground (4-point manifold). Each
  step records frame, time, position, linear+angular velocity, and the
  post-step `World::collide()` contact count. Golden files committed under
  `tests/fixtures/default_step_golden/` (`free_fall.tsv` 30 rows,
  `sphere_on_ground.tsv` and `box_on_ground.tsv` 120 rows each), loaded
  `__FILE__`-relative; regenerate with
  `DART_REGENERATE_DEFAULT_STEP_GOLDEN=1`. Two-oracle design: (1) a
  correctness layer (`assertAnalyticalAnchors`) validates each trajectory
  against ground truth independent of the golden — the exact semi-implicit
  Euler closed form `z_n = z0 - g·dt²·n(n+1)/2`, `v_n = -g·n·dt` for the
  contact-free phase (to `1e-9`, so the golden cannot enshrine an integrator
  bug; forward Euler would diverge by `g·dt²·n`), plus physical invariants for
  the contact phase (no tunneling, settles at the geometric rest height
  `ground_top + half_size`, comes to rest, no spurious lateral/spin); it runs
  in both compare and regen modes so a regenerated golden is physics-validated
  before commit. (2) the behavior-lock snapshot compares the full per-step
  trajectory at `kStateTolerance = 1e-6` absolute (the single knob — tolerates
  floating-point reassociation from behavior-preserving refactors and
  cross-platform libm/FMA differences while catching real physics changes),
  contact counts exact. Verified: all three cases pass (snapshot + analytical);
  compare-mode passes 5/5 consecutive runs and two regenerations produce
  byte-identical goldens (same-toolchain bitwise determinism); an injected
  gravity perturbation makes `DefaultStepGolden.FreeFall` fail (snapshot
  `lin_z` drift `1e-5` ≫ tolerance). Demonstrating the correctness layer's
  value: perturbing the scene's gravity while leaving the analytical profile
  makes the closed-form anchor fail, and regeneration is then **refused** —
  `writeGolden` is gated on `::testing::Test::HasFailure()`, so the
  physically wrong golden is not serialized (verified: the `free_fall.tsv`
  sha256 is byte-identical before and after a bad-physics regen attempt). The
  snapshot path therefore cannot launder a physically wrong trajectory.
  Reverting restores green. Re-verified green after merging the latest `main` (#2992, which
  touched `world.cpp`/`rigid_body.cpp`), confirming #2992 preserved
  default-step behavior. Gates: `pixi run lint` (formatting clean),
  `pixi run build`, and `pixi run test-unit` (161/161, no regressions) all
  exit 0; the focused test runs green via `ctest -R default_step_golden`.
  Systematic energy/momentum-conservation and order-of-convergence validation
  across the scene corpus is deferred to WP-091.24.

#### WP-091.3 Architecture-page claim lint [claimed]

- Objective: every ✅/available claim in `docs/readthedocs/architecture.md`
  is CI-checkable: each marked row cites a header symbol and a test.
- Scope: (a) the marked-availability rows do not yet carry test citations and
  the tables (Physics domains, Solver method families, Cross-domain coupling,
  Collision/contacts, Compute backends — ~23 marked rows) have no test column,
  so the executor first adds, to each ✅/available row, a citation to a
  DART-owned test that exercises that claim (a new `Test` column or an inline
  citation in the existing `Owner`/`Selected by` cell — pick one form and
  apply it consistently across both table shapes); the existing
  header-symbol/entry-point citation stays. (b) a `scripts/check_*.py`
  following the completion-audit checker pattern
  (`scripts/check_plan083_completion_audit.py`) that, for every marked row,
  asserts the cited header symbol resolves in an installed public header and
  the cited test file exists under `tests/`; registered in `pixi.toml`
  `lint`/`check-lint` aggregates (add both a `lint-*` and a `check-*` task,
  mirroring the `lint-plan083-*`/`check-plan083-*` pairing); plus a meta-test.
  The added citations are the judgment work and the checker is mechanical, but
  they share one verification story (the checker passing on the now-cited doc),
  so they stay in this one packet.
- Non-goals: re-grading the markers themselves (done with this plan's
  creation); generating the tables; changing any claim's ✅/available status
  (a marked row with no real test is a finding to report back, not a marker to
  silently downgrade here).
- Acceptance evidence: every ✅/available row in `architecture.md` carries a
  header-symbol citation and a test citation; the new checker passes on the
  amended docs; a meta-test proves it rejects an injected unsupported claim (a
  marked row whose cited test does not exist, and one whose cited symbol does
  not resolve).
- Gates: `pixi run lint`, `pixi run check-lint`, new checker meta-test.
- Dependencies: none. Hazards: the two solver tables use different column
  layouts (Physics domains: Domain/Status/Public entry point/Owner; Solver
  method families: Domain/Method option/Status/Selected by), so the citation
  form must fit both; `architecture.md` cites `PLAN-082`, so coordinate with
  WP-091.5 if both are in flight (the renumber rewrites that citation).
- Evidence: all eight ✅-available rows in `docs/readthedocs/architecture.md`
  now carry an inline test citation (a `tests/...` link) alongside their
  header symbol — Rigid bodies → `test_world.cpp`; Articulated multibody and
  Multibody semi-implicit → `test_world_contact_parity.cpp`; Rigid
  sequential-impulse → `test_world_default_step_golden.cpp`; Native collision
  world → `test_collision_world.cpp`; CCD casts (symbol
  `CollisionGroup::sphereCast`/`capsuleCast` added — the row had none) →
  `test_ccd.cpp`; `SequentialExecutor`/`ParallelExecutor` →
  `test_compute_graph.cpp`. `scripts/check_architecture_page_lint.py`
  (`pixi run check-architecture-page`/`lint-architecture-page`, registered in
  both the `lint` and `check-lint` aggregates) parses every table row marked
  ✅ available and asserts (a) at least one cited backtick symbol resolves in
  `dart/**/*.hpp` and (b) every cited `tests/...` file exists; the ASCII
  pipeline diagram (box-drawing chars + `[A]` markers) is not matched, so
  exactly the eight real rows are checked. `tests/test_check_architecture_page_lint.py`
  (8 pytest cases via `pixi run python -m pytest`) proves the checker passes
  on the real doc and rejects an injected unsupported claim — a marked row
  whose cited test file does not exist, and one whose cited symbol does not
  resolve — plus the no-symbol, no-test, and no-available-rows failure modes.
  Gates: `pixi run lint`, `pixi run check-architecture-page` (and its inclusion
  in `check-lint`), `pixi run check-docs-policy`, and the meta-test all green;
  the only `check-lint` aggregate failure is the pre-existing, unrelated
  `check-dartpy-import-layout` stale-runtime drift (this packet changes no
  dartpy files).

#### WP-091.4 Legacy freeze gate and Decision 5

- Objective: PLAN-042 Decision 5 (which legacy DART 6 surfaces are removed,
  quarantined, wrapped, or promoted) is recorded, and CI rejects new public
  symbols/bindings/stubs in the quarantine lane unless tagged as bugfixes.
- Scope: PLAN-042 decision record; an extension of the API-boundary check
  scripts; carry genuinely new legacy capabilities (for example the orphaned
  cylindrical joint constraint) forward as DART 7 loop-closure families or
  record their retirement.
- Non-goals: deleting legacy trees; migrating the GUI/loader dependencies on
  `dart/dynamics` (tracked as PLAN-042 follow-up work; cut packets there once
  Decision 5 is recorded).
- Acceptance evidence: decision recorded in PLAN-042; freeze check exercised
  by a meta-test; orphaned-feature disposition recorded.
- Gates: `pixi run lint`, `pixi run check-lint`.
- Dependencies: maintainer direction on Decision 5 options. **BLOCKED as of
  2026-06-13:** "Decision 5" is item 5 ("Classic simulation and dynamics
  boundary") in the numbered `## Decisions To Make` list of
  `docs/plans/042-dart7-public-api-and-source-layout.md`, still open with no
  recorded maintainer choice; the same unresolved state is mirrored in
  `042-dart7-public-api-and-source-layout/post-promotion-source-layout-decision.md`
  (the "Legacy dynamics boundary" bullet under `## Required Follow-Up
Decisions`, and the `dart/dynamics` row of `## Current Folder
Classification`, marked "compatibility/quarantine lane; surviving concepts
  need explicit DART 7 names") and in
  `042-dart7-public-api-and-source-layout/api-source-layout-audit.md` (the
  `dart/dynamics` row, whose disposition cell still reads "Decide remove,
  wrap, or quarantine per concept"). Unblock by recording the Decision 5
  outcome — per DART 6 surface (World, Skeleton, BodyNode, the constraint
  concepts, the orphaned cylindrical joint), whether each is removed,
  quarantined, wrapped, or promoted — and resolving those three open anchors
  in place (no new `## Decision 5` heading is needed; the anchors already
  exist). The quarantine lane this packet's freeze check must enforce is
  undefined until that decision exists, so do not start the check before then.

#### WP-091.5 Renumber colliding plan IDs [claimed]

- Objective: `PLAN-080` and `PLAN-082` each identify exactly one initiative.
- Scope: assign fresh IDs to the Performance Dashboard and Linear-Time
  Variational Integrator entries; `git mv` the affected plan files/sidecars;
  update every cross-reference, in docs **and** in code/tests/scripts/CI — the
  design docs, dev-task folders, `docs/readthedocs/papers.md`,
  `docs/readthedocs/architecture.md` (its multibody solver table cites
  PLAN-082), and `solver-family-intake.md` all cite these IDs, and so do
  non-docs surfaces (for example `dart/simulation/compute/world_step_stage.cpp`
  and other `dart/simulation/**` headers, `tests/unit/simulation/**` and
  `tests/benchmark/simulation/**`, `python/examples/demos/**`,
  `scripts/run_performance_dashboard_benchmarks.py` and other `scripts/**`, and
  `.github/workflows/benchmark_pr_comparison.yml`); add a plan-ID uniqueness
  check to the docs-policy script. Disambiguate per reference: only the two
  renumbered initiatives change ID; references to the kept initiatives
  (Rigid-Body Dynamics Solver under PLAN-080, Rigid Implicit-Barrier Contact
  under PLAN-082) must not be rewritten.
- Non-goals: changing any plan's content or status.
- Acceptance evidence: one dashboard block per ID (today `dashboard.md` has
  two PLAN-080 blocks — Rigid-Body Dynamics Solver and Performance Dashboard —
  and two PLAN-082 blocks — Rigid Implicit-Barrier Contact and Linear-Time
  Variational Integrator); link checker / docs policy green; a plan-ID
  uniqueness check added to `scripts/check_docs_policy.py` (extend the plan
  lifecycle/lint logic to reject a dashboard with two blocks claiming one
  PLAN-ID) with a meta-test under `tests/` proving it rejects a duplicate-ID
  fixture.
- Gates: `pixi run lint`, `pixi run check-docs-policy`.
- Dependencies: none. Hazards: the collision is real and confirmed (verified
  2026-06-13 by `git grep -l` over tracked files) — `git grep -l PLAN-080`
  matches 29 files and `git grep -l PLAN-082` matches 54 files, for **71
  unique tracked files** referencing one or both IDs (42 under `docs/` and 29
  outside it: `dart/simulation/**`, `tests/**`, `python/examples/demos/**`,
  `scripts/**`, and one `.github/workflows/**`). Re-run those two `git grep`
  commands at execution time for the live list rather than trusting these
  counts. The hazard is that grep-by-ID hits both the kept and the renumbered
  initiative while duplicates exist, so disambiguate every hit; fix the
  duplicates in one packet and renumber the file plus its sidecar folder in
  tandem (`080-*`, `082-rigid-implicit-barrier-contact{,/}`,
  `082-variational-integrator-solver{,/}` all exist on disk). Coordinate with
  WP-091.3 if both are in flight (both rewrite the `architecture.md` PLAN-082
  citation).
- Evidence: the four colliding dashboard blocks now hold four unique IDs —
  Rigid-Body Dynamics Solver keeps `PLAN-080`, Rigid Implicit-Barrier Contact
  keeps `PLAN-082`, the Linear-Time Variational Integrator is renumbered to
  `PLAN-084` (chosen adjacent to the solver block; verified free), and the
  Performance Dashboard to `PLAN-092` (`PLAN-090` was already taken by the
  Filament Renderer Performance entry — caught and avoided). `git mv` renamed
  `082-variational-integrator-solver.md` and its sidecar directory to `084-`;
  every cross-reference was disambiguated per occurrence (variational →
  `PLAN-084`: 32 refs in 9 `variational_*`/`multibody.hpp` code+test files plus
  the variational plan/design/dev-task/papers docs; Performance Dashboard →
  `PLAN-092`: the dashboard block heading, `benchmark_pr_comparison.yml`, and
  `benchmark_pr_compare.py`), while every reference to the kept initiatives
  (rigid-body solver, rigid IPC) was left untouched — confirmed by a full
  `git grep` audit showing all remaining `PLAN-082` refs are rigid-IPC or
  cross-plan/exemplar context and all `PLAN-084` refs are variational. The
  classification was produced by a fan-out over the 45 referencing
  doc/script/CI files (line-numbered decisions applied surgically), and the
  two solver-table/path edits coordinate cleanly with WP-091.3 (different
  lines). The plan-ID uniqueness check landed in
  `scripts/check_docs_policy.py` (`check_plan_id_uniqueness`, run by
  `pixi run check-docs-policy`) with `tests/test_check_docs_policy.py` (4
  pytest cases proving it rejects a duplicate-ID dashboard). Gates:
  `pixi run lint` (markdown links + format), `pixi run check-docs-policy`
  (uniqueness + owner-link resolution), and the meta-test all green. (The
  `architecture.md` solver-table edit is the single variational PLAN-082 →
  PLAN-084 citation on a 🧪-experimental row; it does not overlap WP-091.3's
  ✅-row citations, which land separately in PR #2999.)

### WS1 — Internal solver contract and selection

#### WP-091.10 Solver contract step 1: virtual finalize/prepare [claimed]

- Objective: the hand-synchronized prepare-table-plus-switch pair is replaced
  by virtual `prepare()`/`finalize()` on the stage contract.
- Scope: `dart/simulation/compute/world_step_stage.*`,
  `dart/simulation/detail/world_step_schedule.hpp`, the stage-binding and
  prepare-dispatch code in `dart/simulation/world.cpp`.
- Non-goals: capabilities, schedule derivation, source moves.
- Acceptance evidence: the parallel boolean table and dispatch switch are
  deleted; existing unit suites green; golden trajectories unchanged.
- Gates: `pixi run lint`, `pixi run build`, `pixi run test-unit`.
- Dependencies: WP-091.2.
- Evidence: `compute::WorldStepStage` now declares a virtual
  `prepare(World&)` with a no-op default (defined in
  `world_step_stage.cpp`); all seven concrete stages that cache per-step state
  (`Kinematics`, `RigidBodyVelocity`, `RigidBodyContact`, `RigidIpcContact`,
  `DeformableDynamics` in `world_step_stage.hpp`, plus `UnifiedConstraintStage`
  in `multibody_dynamics.hpp` and `MultibodyVariationalIntegrationStage` in
  `variational_integration.hpp`) override it. The hand-synchronized
  prepare-table-plus-switch pair is deleted: `world.cpp`'s `prepareStage`
  (an 11-case dispatch `switch` gated on the table) collapses to
  `stageForSlot(slot).prepare(world)` in the schedule loop, and
  `detail::builtInWorldStepScheduleNeedsPreparation` is removed from
  `world_step_schedule.hpp` (net −69 lines). The replacement is provably
  behavior-preserving: the deleted table returned `true` for exactly the seven
  stages that own a `prepare()` and `false` for the four that do not, so
  virtual dispatch (no-op base for the four) reproduces it bit-for-bit. The
  obsolete `BuiltInWorldStepSchedule.PreparationContractCoversStatefulStages`
  unit test (which asserted the removed table) is dropped, with a note that
  preparation behavior is now covered by the World suites and the golden
  trajectories. Verified: `pixi run build` green; the WP-091.2 golden
  trajectories are **unchanged** (`test_world_default_step_golden` 3/3); and
  `test_world`, `test_world_contact_parity`, `test_world_step_schedule`, and
  `test_variational_integration` all pass (`pixi run lint` clean; full
  `test-unit` deferred to CI).

#### WP-091.11 Minimal capabilities and validated finalize [claimed]

- Objective: each solver family declares a minimal capability set (domain,
  supported joints/actuators/shapes, differentiability), the built-in
  schedule derives from declared domain presence plus capabilities, and
  `enterSimulationMode` converts silent fallbacks into errors or recorded
  decisions.
- Scope: the stage/solver contract from WP-091.10; schedule construction; a
  resolved-configuration report (requested family → resolved family with
  reasons) exposed alongside the step profile.
- Non-goals: the full documented capability matrix; public capability APIs.
- Acceptance evidence: a scene that previously silently substituted methods
  now either errors (strict) or records the substitution; report is
  serialized into benchmark packets (consumes WP-091.1 schema).
- Gates: `pixi run lint`, `pixi run build`, `pixi run test-unit`.
- Dependencies: WP-091.10, WP-091.1.
- Execution plan (committable slices, each golden-trajectory-verified before
  the next, landing on the consolidated PLAN-091 branch): **(1) Resolved-config
  report.** Add a `compute::ResolvedSolverConfiguration` value object (requested
  vs resolved per-domain family — rigid-body solver, contact method, multibody
  integration — plus a `std::vector` of substitution/decision notes with
  reasons). Record it in `enterSimulationMode` from the already-resolved
  members (`m_rigidBodySolver`, `m_contactSolverMethod`,
  `m_multibodyIntegrationMethod`) and expose it via
  `getResolvedConfiguration()` alongside `getLastStepProfile()`. Additive only,
  so the goldens are unchanged; covered by a new unit test. **(2) Make the
  known silent substitutions explicit.** Enumerate the current finalize-time
  and stage-entry fallbacks (the WP-091.1 finding: AVBD rigid contact silently
  runs sequential impulse when no body carries the opt-in config; the
  variational scene-support fallbacks) and, at finalize, classify each as a
  recorded decision (default) or an error under a strict flag on `WorldOptions`.
  This consumes the validate\* hooks already in `enterSimulationMode`. Re-verify
  goldens (a recorded decision must not change the chosen path; only reporting
  changes). **(3) Capability-derived schedule.** Have each built-in family
  declare a minimal capability set (domain + supported joints/actuators/shapes
  - differentiability) and derive `makeBuiltInWorldStepSchedule` inclusion from
    declared domain presence plus those capabilities rather than the current
    hand-listed domain-presence flags, keeping the emitted schedule identical for
    every existing scene (golden + `test_world_step_schedule` parity). **(4)
    Packet serialization.** Serialize the resolved-config report into the
    benchmark packet writers via the WP-091.1 `resolved_solver_identity` schema
    (the schema field already exists; this populates it from the report instead
    of by hand). Each slice is one commit with its own focused tests; the packet
    is done when slice 4 lands and a previously-silent-substitution scene errors
    (strict) or records the substitution in a packet.
- Evidence (slices 1–4 of 4 landed): **Slice 1** —
  `compute::ResolvedSolverConfiguration` and `compute::ResolvedConfigurationNote`
  value types (plain strings — no ECS, solver, or backend leak;
  `check-api-boundaries` green) added to `world_step_profile.hpp`;
  `World::getResolvedConfiguration()` exposes them alongside
  `getLastStepProfile()`. `World::recordResolvedConfiguration()` is called from
  the bake (`prepareStepPipelineCacheForCurrentConfiguration`) and records the
  resolved rigid-body, rigid-contact, and multibody families. **Slice 2** —
  the known silent substitution is now explicit: when the internal AVBD
  rigid-contact opt-in is present (not facade-selectable; PLAN-091 WP-091.1),
  the rigid-contact note records a substitution (`requested` =
  `sequential-impulse`, `resolved` = `… + avbd (opt-in)`, with the reason),
  and a new `WorldOptions::strictSolverResolution` flag makes
  `enterSimulationMode` throw on any substitution instead of recording it
  (default false = record). Additive on the default path — the WP-091.2 golden
  trajectories are **unchanged** (`test_world_default_step_golden` 3/3) and the
  126 AVBD/contact cases in `test_world` are unaffected.
  `tests/unit/simulation/world/test_world_resolved_configuration.cpp` (5 cases:
  empty before finalize, default families, requested-method reflection, AVBD
  substitution recorded, strict mode rejects) passes. Gates: `pixi run lint`,
  `pixi run check-api-boundaries` green. **Slice 3** — the built-in schedule
  no longer branches on the family enum directly: each family declares its
  scheduling-relevant capabilities
  (`builtInRigidSolverUsesSplitPipeline`/`builtInCombinedRigidStage`,
  `builtInMultibodyFusesWithUnifiedConstraint`/`builtInStandaloneMultibodyStage`
  in `detail/world_step_schedule.hpp`), and `makeBuiltInWorldStepSchedule`
  derives stage inclusion from declared domain presence plus those
  capabilities. Adding a family now means declaring its capabilities rather
  than threading another hand-listed domain-presence branch. The emitted
  schedule is identical for every existing scene: `test_world_step_schedule`
  (16 pinned scenes) and the WP-091.2 golden trajectories
  (`test_world_default_step_golden` 3/3) are **unchanged**; `pixi run lint`,
  `pixi run check-api-boundaries` green. **Slice 4** — the AVBD packet writers
  no longer hand-type the `resolved_solver_identity` object:
  `scripts/avbd_packet_schema.py` gains `make_resolved_solver_identity(...)`,
  which derives the packet's `rigid_contact_solver` enum from the C++ report's
  rigid-contact family string through one mapping
  (`resolved_rigid_contact_solver_from_report`: `sequential-impulse` →
  `sequential_impulse`, `boxed-lcp` → `boxed_lcp`, an `avbd` opt-in marker →
  `avbd`) and validates the result against the existing schema contract. The
  four writers (`write_avbd_{breakable_joint_scale, breakable_motor_scale,
friction_coefficient_sweep, paper_scale_high_ratio_iteration_sweep}_packet`)
  now construct their identity through the builder; the output is byte-identical
  to the committed packets (verified field-by-field against all four committed
  JSONs), so no packet is regenerated. `tests/test_avbd_packet_schema.py` (13
  cases) locks the builder against the committed identities and the report →
  enum mapping. Gates: `pixi run check-avbd-packets` (52 packets), `pixi run
python -m pytest tests/test_check_avbd_packets.py
tests/test_avbd_packet_schema.py`, `pixi run lint` green. The live
  per-benchmark emission (a benchmark `SetLabel` carrying
  `World::getResolvedConfiguration()` consumed in place of the per-scene family
  argument) is adopted the next time each packet is regenerated, per this
  module's standing writer-adoption convention; the packet's core acceptance —
  a previously-silent-substitution scene erroring under strict resolution —
  already landed in slice 2 (`StrictResolutionRejectsSubstitution`).

#### WP-091.12 Single selection idiom [claimed]

- Objective: every domain selects its method family through one idiom —
  typed per-domain policy value objects on `WorldOptions` resolved once at
  finalize.
- Scope: `world_options.hpp`, `multibody_options.hpp` (retire the parsed
  string), the AVBD rigid-contact selection path (facade-named family or
  explicitly test-only), bindings and stubs, serialization round-trip.
- Non-goals: adding new families; renaming algorithms.
- Acceptance evidence: the string selector is gone; AVBD selection is
  facade-visible or test-scoped; all four current selection mechanisms
  (rigid-solver enum, contact-method enum, multibody string, AVBD component)
  resolve through the one finalize-time policy path — including the two
  existing enums; options round-trip through save/load; Python surface
  matches.
- Gates: `pixi run lint`, `pixi run build`, `pixi run test-unit`,
  `pixi run test-py`, `pixi run check-api-boundaries`.
- Dependencies: WP-091.11.
- Evidence: `MultibodyOptions::integrationFamily` is now the typed
  `MultibodyIntegrationFamily` enum (`SemiImplicit` default, `Variational`) in
  `multibody/multibody_options.hpp`, replacing the parsed `std::string`; the
  string parse and its `InvalidArgumentException` are gone, and
  `setMultibodyOptions` validates the value by enum range
  (`isValidMultibodyIntegrationFamily`) and converts it once to the internal
  `MultibodyIntegrationMethod`. All four selection mechanisms — the
  `RigidBodySolver` enum, the `ContactSolverMethod` enum, this multibody family
  enum, and the AVBD `RigidAvbdContactConfig` component — resolve through the
  single finalize-time policy path: each is recorded as a note in
  `m_resolvedConfiguration` and gated by `m_strictSolverResolution &&
hasSubstitution()`. Python surface matches: nanobind exposes
  `MultibodyIntegrationFamily` (`SEMI_IMPLICIT`/`VARIATIONAL`), the stub,
  examples, and the constructor default use the enum. Serialization round-trips
  the enum (`test_serialization` 57/57). Gates: `pixi run lint` and
  `check-api-boundaries` clean; the default-step golden trajectories stay 3/3
  bit-identical (behavior-preserving — the resolved family is unchanged);
  `test_world` 371/372 (the lone failure,
  `BakedDynamicRigidIpcStepsDoNotGrowWorldBaseAllocator`, is the known
  pre-existing post-#2996 rigid-IPC allocator issue, orthogonal to this change);
  `test_variational_integration` 178/178; cross-family + corpus green;
  `pixi run test-py` green (1382 passed, 11 skipped).

#### WP-091.13 Canonical contact assembly [claimed]

- Objective: sequential-impulse, boxed-LCP, and the gradient path consume the
  one canonical contact-problem assembly, and the populated snapshot is
  handed to the differentiable capture instead of being rebuilt.
- Scope: converge on the existing canonical assembly seam in
  `dart/simulation/compute/rigid_body_constraint.*`; retire the mirrored
  copies in the sequential-impulse scratch (lives in
  `dart/simulation/compute/world_step_stage.{hpp,cpp}` — touch only the
  contact-assembly region of that multi-family file; WP-091.15 owns the
  split), `detail/boxed_lcp_contact.*`, and `detail/contact_jacobians.*`;
  single-source the contact constants.
- Non-goals: changing contact physics; AVBD's endpoint classification;
  restructuring `world_step_stage.cpp` beyond the assembly region.
- Acceptance evidence: one assembly producer; constants defined once; the
  existing contact parity and gradient suites green (at minimum
  `test_world_contact_parity.cpp`, `test_boxed_lcp_contact.cpp`, and the
  diff contact-gradient-mode tests under `tests/unit/simulation/`); golden
  trajectories unchanged.
- Gates: `pixi run lint`, `pixi run build`, `pixi run test-unit`.
- Dependencies: WP-091.2.
- Evidence (slice A of 2 landed): **Slice A — single-source the contact-assembly
  helpers and constants.** The six rigid-rigid contact helpers (`inverseMassOf`,
  `normalizeOrIdentity`, `inverseWorldInertiaOf`, `restitutionOf`, `frictionOf`,
  `hasPrescribedRigidBodyContactResponse`) and the two shared contact constants
  (`kRigidContactRestitutionThreshold` = 1e-3, `kRigidContactFrictionCfm` = 1e-5)
  were duplicated verbatim across the three rigid-contact translation units.
  They now live once in the new header
  `dart/simulation/detail/rigid_contact_assembly.hpp`;
  `compute/rigid_body_constraint.cpp`, `detail/boxed_lcp_contact.cpp`, and
  `detail/contact_jacobians.cpp` include it and use the shared definitions
  (`rigid_body_constraint` qualifies them `detail::` and drops its
  `inverseMass`/`inverseWorldInertia` spellings for the shared `…Of` names; its
  local `normalizeOrIdentity` was behaviorally identical). Behavior-preserving by
  construction (identical bodies, identical constant values). Verified:
  default-config `test_world_default_step_golden` 3/3, `test_world_contact_parity`
  5/5, `test_boxed_lcp_contact` 122/122 unchanged (these exercise the shared
  helpers at runtime through `rigid_body_constraint`/`boxed_lcp_contact`);
  `contact_jacobians.cpp`'s de-duplication is runtime-verified under
  `DART_BUILD_DIFF=ON`: `test_diff_contact_jacobian` 5/5,
  `test_diff_public_contact_jacobian` 5/5, `test_diff_contact_gradient_modes`
  6/6 (gradients unchanged); `pixi run lint` green. Running those diff suites
  first required fixing a pre-existing diff-only `main` build break unrelated to
  this packet — `world.cpp` passed the `StlAllocator`-typed
  `world_storage.hpp` `differentiableParameters` to
  `contactStepDerivativesWithParameters`, whose signature
  (`contact_jacobians.hpp:193`) takes a default-allocator
  `std::vector<ParameterRegistration>&` (allocator type introduced by main
  commit `64e61f1f04b` (#2863); `DART_BUILD_DIFF` is not exercised in CI). Fixed
  in the follow-up commit by adapting the parameter vector at the `world.cpp`
  call site (a trivial default-allocator copy; no API or storage-allocator
  change). A follow-up commit also routed the fourth `restitutionThreshold`
  copy (the sequential-impulse scratch in `world_step_stage.cpp`) to the shared
  `kRigidContactRestitutionThreshold`, so the restitution threshold is now
  defined once across all four contact paths (golden 3/3, contact parity 5/5,
  boxed-LCP 122/122 unchanged).
  **Slice B (remaining):** converge the four contact-problem _assemblers_ (the
  canonical `RigidBodyContactProblem` producer, the boxed-LCP assembly, the
  differentiable-capture rebuild, and the sequential-impulse scratch in
  `world_step_stage.cpp`) onto one producer, single-source the remaining
  positional-correction constants in `world_step_stage.cpp`, and hand the
  populated snapshot to the
  differentiable capture instead of rebuilding it — the higher-risk
  physics-convergence work this slice deliberately leaves for a focused
  follow-up (which also needs the diff build unbroken).

#### WP-091.14 Family-neutral joint model [claimed]

- Objective: shared `comps::Joint` carries only family-neutral semantics;
  per-family solver state lives in family-owned sidecar components; no family
  discovers constraints through another family's config.
- Scope: `comps/joint.hpp` AVBD fields → family component; the rigid-IPC and
  variational consumers read neutral fields; serializer registration moves to
  the owning family; re-home the rigid/articulated AVBD machinery out of the
  deformable-named directory as a family module.
- Non-goals: changing joint semantics or public handle APIs (WS4 owns knob
  renames).
- Acceptance evidence: no cross-family component queries remain (grep-clean);
  serialization round-trip preserved; existing joint suites green.
- Gates: `pixi run lint`, `pixi run build`, `pixi run test-unit`.
- Dependencies: WP-091.10.
- Evidence (re-homing slice landed; field move remaining): the rigid/articulated
  AVBD machinery — `rigid_block_kernel.hpp` and `rigid_world_contact.hpp` (the
  latter holds the `AvbdRigidWorld*` joint/spring sidecar components) — is
  re-homed out of the deformable-named `detail/deformable_vbd/` directory into a
  dedicated `detail/rigid_avbd/` family module; all 12 includers updated. The
  shared AVBD kernels (`avbd_constraint.hpp`, `contact_kernel.hpp`, used by both
  the rigid and deformable paths) stay in `deformable_vbd/` — relocating those
  shared kernels to a neutral shared directory is WP-091.15's scope, so the
  re-homed rigid module currently still includes them cross-directory. Pure file
  relocation + include-path updates; behavior-preserving (golden 3/3,
  `test_boxed_lcp_contact`, `test_avbd_rigid_block`, `test_serialization` green;
  `pixi run lint` green). **Core field move (landed):** the per-family AVBD
  joint stiffness fields (`hasAvbdStiffnessState`,
  `avbd{Start,Linear,Angular,Max}Stiffness`) are moved out of the shared
  `comps::Joint` into a new family-owned sidecar `comps::AvbdJointStiffness`
  (four stiffness values; the old `hasAvbdStiffnessState` flag becomes component
  presence — absent ⇒ defaults). The family-neutral `breakForce`/`broken` stay
  on `comps::Joint` (the variational and sequential-IPC families legitimately
  read those). The facade setters/getters/`materializeAvbdStiffnessState`
  (`multibody/joint.cpp`), the `rigid_avbd/rigid_world_contact.hpp`
  materialization, and the `world.cpp` `JointLayout` cache mirror now read/write
  the sidecar (registry + entity are in scope at every site). Serialization:
  `kBinaryFormatVersion` bumped to 20 — the new format serializes the sidecar as
  a registered component; legacy v17–19 packets read the inline bool+4-doubles
  block and stage it, then `postLoadComponent` materializes the sidecar only
  when the legacy flag was set (absent ⇒ defaults), preserving round-trip both
  ways. The 22 `config.avbdMaxStiffness` reads in `world_step_stage.cpp` are the
  _deformable_ AVBD config (a name collision) and are correctly untouched.
  Behavior-preserving: `test_serialization` 54/54 (round-trip), golden 3/3
  (trajectories unchanged), `test_avbd_rigid_block` 105/105,
  `test_boxed_lcp_contact` 122/122 (plus `test_joint` 5/5 and `test_world
--gtest_filter='*Avbd*:*Joint*'` 87/87, which exercise the JointLayout
  cache-equality path); `pixi run lint` and `pixi run check-api-boundaries`
  green. Net +279/−138 across 11 files. Residual: the v20 round-trip is fully
  tested, but the legacy v17–v19 inline-stiffness read path is validated by
  byte-order reasoning against the old PFR record (no committed pre-v20 binary
  fixture in the suite) — a follow-up could add one.

#### WP-091.15 Family-scoped source layout

- Objective: each solver family's runtime lives in its own module under
  `dart/simulation/detail/`, ending the multi-family monolithic translation
  units.
- Scope: mechanical split of the world-step-stage and facade translation
  units along existing stage-class boundaries; flat `detail/` family files
  move into family modules; shared kernels stay in shared component
  directories.
- Non-goals: behavior changes of any kind; renames of public symbols.
- Acceptance evidence: no multi-family translation unit remains; all suites
  green; golden trajectories unchanged; include-what-you-use clean for the
  moved headers.
- Gates: `pixi run lint`, `pixi run build`, `pixi run test-unit`.
- Dependencies: WP-091.2, WP-091.10, WP-091.14.

### WS2 — Physical data architecture

#### WP-091.20 Model/State/Control component split [claimed]

- Objective: the articulated and deformable component fusion is split so
  storage matches the documented Model/State/Control/Contacts contract (the
  rigid domain already complies).
- Scope: `comps/joint.hpp`, `comps/link.hpp`, deformable node components;
  realign component categories so serialization behavior derives from the
  contract; update the consumers that hand-pick fields (replay, state vector,
  batch extraction, diff rollouts).
- Non-goals: SoA layout changes; new public APIs.
- Acceptance evidence: each split component is Model-, State-, or
  Control-only; replay and serialization round-trips green; golden
  trajectories unchanged.
- Gates: `pixi run lint`, `pixi run build`, `pixi run test-unit`.
- Dependencies: WP-091.2.
- Slicing: the packet is landed in three independently-verifiable slices —
  20a `comps::Joint`, 20b `comps::Link`, 20c `DeformableNodeState`. Each follows
  the rigid-domain reference pattern (single-contract components) and must keep
  golden trajectories bit-identical and serialization/replay round-trips green.
- Evidence (slice 20a — joint, landed): `comps::Joint` is split into three
  contract-aligned components in `comps/joint.hpp` — `JointModel` (frozen
  topology/geometry and the passive/solver parameters: type, name, axis/axis2/
  pitch, limits, spring/damping/armature/coulombFriction/restPosition,
  breakForce, parent/child links, rigid-body-fixed anchors; carries getDOF() and
  entityFields(), and is the canonical "is a joint" component), `JointState`
  (per-step position/velocity/acceleration and the runtime broken flag), and
  `JointActuation` (actuatorType, torque, commandVelocity). Every consumer (the
  Joint facade's `getJointComponent` split into three accessors, the two bake
  paths in `world.cpp` + `multibody.cpp`, the multibody-dynamics/variational/
  stage/constraint/kinematics-graph hot paths, the AVBD contact code in
  `detail/rigid_avbd/rigid_world_contact.hpp` and `detail/smooth_jacobians.cpp`,
  replay capture/restore, and serialization) reads each field through its owning
  component. Serialization registers the three components via the generic
  `registerComponentIfNeeded` path (PFR auto-serialize; `JointModel` entity refs
  remapped via `entityFields`); the unified `comps.Joint` serializer and the now
  clean-break-dropped legacy v1–v19 joint deserializers are removed and the
  binary format is bumped 21 → 22. Gates: `pixi run lint` and
  `check-api-boundaries` clean; default-step golden trajectories stay **3/3
  bit-identical** (behavior-preserving); `pixi run test-unit` 163/163 binaries
  pass (serialization round-trip + the three new stable IDs; variational 178/178;
  cross-family; AVBD 107/107; boxed-LCP 122/122).
- Evidence (slice 20b — link, landed): `comps::Link` is split into three
  contract-aligned components in `comps/link.hpp` — `LinkModel` (frozen
  topology, geometry, and inertia: name, mass, transformFromParentToJoint,
  transformFromParentJoint, parentJoint, childJoints; carries entityFields();
  the canonical "is a link" component), `LinkState` (per-step worldTransform),
  and `LinkControl` (the user-applied externalForce, the articulated analog of
  the rigid `Force` component). Every consumer (the Link facade, the two bake
  paths in `multibody.cpp`, the multibody-dynamics/variational/constraint/stage/
  kinematics-graph hot paths, world.cpp readers + `ensureRegistryStorages`, AVBD
  contact, frame, skeleton-loader, collision-body, and tests) reads each field
  through its owning component. Serialization registers the three via the
  generic `registerComponentIfNeeded` path; the property-component
  auto-serializer gained a fixed `Eigen::Matrix<double,6,1>` branch (it
  previously existed only on the state path the unified `Link` used) so
  `LinkControl` round-trips. The unified `comps.Link` serializer and the legacy
  v8 link deserializer are removed (clean break); binary format bumped 22 → 23.
  Gates: `pixi run lint` and `check-api-boundaries` clean; default-step golden
  trajectories stay **3/3 bit-identical**; `pixi run test-unit` green
  (serialization round-trip + the three new stable IDs; variational 178/178;
  cross-family; boxed-LCP 122/122; test_world 372/372).
- Evidence (slice 20c — deformable, landed): `DeformableNodeState` is split into
  two contract-aligned components in `comps/deformable_body.hpp` —
  `DeformableNodeModel` (frozen per-node mass and pinning mask: masses, fixed)
  and `DeformableNodeState` (per-step node kinematics: positions,
  previousPositions, velocities). Both are non-aggregate (allocator
  constructors over `DeformableVector` fields), so each keeps a hand-written
  custom serializer (`DeformableNodeModelSerializer` /
  `DeformableNodeStateSerializer`); both are registered. The bake emplaces both
  (allocator-aware); the deformable solver stage, replay capture/restore, the
  allocator-rebind loop, the facade, scene diagnostics, and tests read masses/
  fixed via `DeformableNodeModel` (the `ScalarVector`/`MaskVector` typedefs moved
  with them) while `DeformableNodeState` stays the canonical per-node component
  for presence/iteration. Binary format bumped 23 → 24 (clean break). Gates:
  `pixi run lint` and `check-api-boundaries` clean; default-step golden
  trajectories stay **3/3 bit-identical**; `pixi run test-unit` green
  (serialization round-trip; AVBD 107/107; test_world 372/372 including the
  deformable suites). All three slices (20a/20b/20c) of WP-091.20 are now
  landed.

#### WP-091.21 Baked dense-index Model artifact

- Objective: `enterSimulationMode` bakes an immutable per-domain dense index
  (body/link/dof offsets, creation-ordered) plus per-multibody model arrays,
  so state addressing stops depending on registry iteration order and
  per-step model rebuilds stop.
- Scope: the bake path in `world.cpp`; multibody dynamics gather paths;
  facade queries (mass matrix, state vectors) reuse the baked artifact; the
  name-string cross-world validation is replaced by index identity.
- Non-goals: promoting the SoA batch layout to the canonical store (sequenced
  with WS3 batching); device residency.
- Acceptance evidence: state-vector layout is documented as dense-index
  ordered; per-step model-quantity rebuild eliminated on the steady-state
  path (assert via allocation/profile test); golden trajectories unchanged.
- Gates: `pixi run lint`, `pixi run build`, `pixi run test-unit`.
- Dependencies: WP-091.20.

#### WP-091.22 Frame arena: wire or delete

- Objective: the per-step frame-scratch arena either gains real consumers
  (transient contact rows, facade-query temporaries) with its diagnostics
  becoming a genuine regression signal, or the unused surface is removed.
- Scope: the World memory manager wiring; the boxed-LCP dense temporaries;
  per-call facade-query allocations; rewrite the allocator design doc against
  the DART 7 World it describes.
- Non-goals: component-payload allocators (post-split concern).
- Acceptance evidence: decision recorded in the allocator design doc; if
  wired, a no-alloc-in-step test for one reference scene passes.
- Gates: `pixi run lint`, `pixi run build`, `pixi run test-unit`.
- Dependencies: none. Sequencing note: landing WP-091.20 first reduces
  rework, since the component split converts most heap-fragmented payloads
  into slab storage — prefer it when both are available.

#### WP-091.23 Stable serialization identity [claimed]

- Objective: on-disk component identity is an explicit stable ID plus schema
  version, not a compiler-mangled type name.
- Scope: the component category macros and serializer registry; a migration
  shim for existing binary files or a recorded format break (DART 7 has no
  compatibility debt yet — prefer the clean break and record it).
- Non-goals: changing what is serialized.
- Acceptance evidence: round-trip tests green; cross-component rename test
  proves stability; format decision recorded.
- Gates: `pixi run lint`, `pixi run build`, `pixi run test-unit`.
- Dependencies: none (cheaper before WP-091.20 multiplies components).
- Evidence: the four component-category macros
  (`DART_SIMULATION_{PROPERTY,STATE,TAG,CACHE}_COMPONENT` in
  `comps/component_category.hpp`) now take an explicit `StableId` string literal
  whose value `getTypeName()` returns, replacing the compiler-mangled
  `typeid(TypeName).name()`; the SerializerRegistry keys components by that
  stable ID. Every serializable component declares a unique dotted ID
  (`comps.<Type>` / `compute.<Type>` — e.g. `comps.Joint`,
  `comps.AvbdJointStiffness`, `compute.MultibodyVariationalState`). Clean break
  recorded: `kBinaryFormatVersion` bumped 20 → 21 (`io/binary_io.hpp`
  changelog) — DART 7 has no compatibility debt, so packets ≤ v20 with
  mangled-name identities no longer round-trip components; the in-memory
  round-trip tests save/load in the new ID format. Three new tests in
  `tests/unit/simulation/world/test_serialization.cpp`:
  `ComponentsReturnExplicitStableId` (asserts components return their literal ID
  and `!= typeid().name()`), `RegisteredIdsAreUniqueAndNotMangled` (iterates the
  live registry — uniqueness + not-mangled guard), and
  `RoundTripIsKeyedByStableIdNotCppType` (looks the serializer up by the literal
  `"comps.ContactMaterial"`, proving the on-disk identity is decoupled from the
  C++ type). Behavior-preserving (non-goal: changing what is serialized):
  `test_serialization` 57/57 (54 + 3), golden 3/3 (trajectories bit-identical),
  `test_avbd_rigid_block` 105/105; `pixi run lint` and
  `pixi run check-api-boundaries` green. 17 files changed.

#### WP-091.24 Metrics contract, scene corpus, one harness [claimed]

- Objective: apples-to-apples comparison has a substrate: a solver-agnostic
  step-metrics value object populated by every family, a registered-builder
  scene corpus, and one benchmark harness emitting rows tagged with scene ID,
  resolved solver identity, and metrics.
- Scope: a small `StepMetrics`-style value object (per-domain energies,
  momenta, penetration, contact counts, solver iterations/residual); a scene
  builder registry under `tests/`; one harness binary; a manifest-driven
  packet writer replacing the per-scene script fleet; a tracked
  cross-family `World::step` scenario row set in the perf dashboard. This
  packet also owns the **systematic analytical/physical validation** layer
  built on those metrics — energy/momentum-conservation bounds and
  order-of-convergence checks (halve `dt`, confirm error drops at the scheme's
  rate) across the scene corpus. WP-091.2 seeds only the per-scene
  closed-form/invariant anchors inline in its golden test; the corpus-wide
  validation substrate (and a reusable closed-form reference helper) lands
  here once the metrics value object exists.
- Non-goals: forcing one contact representation across families (record
  per-family contact-source support in the corpus manifest instead);
  kernel-level microbenchmarks.
- Acceptance evidence: the same scene runs under two rigid families and two
  multibody families producing comparable metric rows from one command; the
  variational-vs-semi-implicit energy comparison consumes the public metrics
  instead of internal registry access.
- Gates: `pixi run lint`, `pixi run build`, `pixi run test-unit`, benchmark
  smoke.
- Dependencies: WP-091.1 (identity schema), WP-091.11 (resolved-config
  report).
- Evidence (foundational slice landed; corpus/harness/convergence remain): the
  solver-agnostic step-metrics value object exists and is consumed by a public
  query. `compute::StepMetrics` (in the promoted `compute/world_step_profile.hpp`
  alongside `WorldStepProfile`/`ResolvedSolverConfiguration` — pure value type,
  plain `double`/`size_t`/`Eigen::Vector3d`, no ECS/solver/backend leak;
  `check-api-boundaries` + the public-header poison-boundary smoke test green)
  carries per-World kinetic/potential/total energy, linear/angular momentum
  (about the world origin), active rigid-contact count and max penetration, and
  last-step iterations/residual. `World::computeStepMetrics() const` populates
  it as a **read-only** query (reads the registry + gravity only; no
  `collide()`, no step, no writes — proven side-effect-free: the WP-091.2
  goldens are bit-identical and an idempotence assertion holds), reusing the
  exact `RigidBody`/`computeMultibodyMechanicalEnergy` energy conventions so the
  numbers match existing diagnostics. The existing variational-vs-semi-implicit
  energy comparison (`test_variational_integration.cpp`,
  `PendulumConservesEnergyOverLongHorizon` / `SelectableThroughWorldStep`) now
  consumes `computeStepMetrics().totalEnergy` instead of internal registry
  access, at identical tolerances (178/178). Two conservation tests in
  `test_world.cpp` seed the validation layer:
  `ComputeStepMetricsConservesFreeBodyInvariants` asserts a free body conserves
  linear+angular momentum and kinetic energy to 1e-9 over 500 steps, and
  `ComputeStepMetricsConservesLinearMomentumThroughCollision` drives a 1-D
  head-on two-body inelastic collision (restitution 0, gravity-free) and asserts
  total linear momentum is conserved through the contact while kinetic energy is
  dissipated (never gained) — validating the public metrics on a contact-bearing
  multi-body scene. Gates: `pixi run lint`, `pixi run check-api-boundaries`,
  `pixi run check-dart7-promotion-surface` green; golden 3/3. **Cross-family
  comparison (landed):** `tests/unit/simulation/compute/test_cross_family_metrics.cpp`
  runs one reference scene under multiple solver families from one loop and
  compares the public `StepMetrics`, realizing the packet's core acceptance — a
  zero-gravity two-sphere head-on collision under `ContactSolverMethod`
  sequential-impulse vs boxed-LCP (both conserve total linear momentum to 1e-7
  and dissipate kinetic energy; final momenta agree to 5e-7, KE within 0.05 J),
  and a 1-link pendulum under semi-implicit vs variational multibody integration
  (both conserve `totalEnergy` to <1% drift; the two families' final energies
  agree within 2%). IPC is documented-excluded (it only handles free mesh-like
  bodies, not a sphere pair). Tolerances are physically derived (conservation
  budgets), comfortably above the observed spreads, so non-flaky.
  `test_cross_family_metrics` 2/2, golden 3/3. **Multibody split fix (landed):**
  the `StepMetrics` multibody kinetic/potential split was initially non-physical
  (it derived potential from the stored `comps::Link::worldTransform` cache,
  whose frame gauge differs from the variational integrator's, giving negative
  multibody `kineticEnergy` at rest). Fixed by exposing the split from the
  trusted helper itself: a new `compute::MultibodyMechanicalEnergyTerms` +
  `computeMultibodyMechanicalEnergyTerms(...)` returns the kinetic/potential
  terms computed on the SAME VarTree forward-kinematics transforms as
  `computeMultibodyMechanicalEnergy` (so `kinetic + potential` equals it
  exactly), and `computeStepMetrics` now uses it. `test_cross_family_metrics`
  now asserts the released-from-rest pendulum has `kineticEnergy ≈ 0` (both
  families), proving the fix; `test_variational_integration` 178/178 (the
  trusted helper is unchanged); golden 3/3; lint, `check-api-boundaries`,
  `check-dart7-promotion-surface` green. **Order-of-convergence seed (landed):**
  `test_world.cpp` `ComputeStepMetricsPotentialEnergyConvergesWithTimeStep`
  drops a free rigid body under constant gravity (exact `y(T) = y0 - ½ g T²`)
  and measures the discretization error in the height-encoding
  `computeStepMetrics().potentialEnergy` at `dt` and `dt/2`; it asserts the
  error is measurable and that halving `dt` genuinely reduces it (ratio in
  `[0.05, 0.7]`, scheme-agnostic and non-flaky). **Scene-corpus + harness
  (landed):** `tests/unit/simulation/compute/test_cross_family_corpus.cpp`
  generalizes the cross-family comparison into a registered scene corpus — a
  `SceneCase` registry whose entries declare a scene builder, the solver-family
  AXIS they exercise (`RigidContactMethod` via `WorldOptions.contactSolverMethod`
  vs `MultibodyIntegration` via `setMultibodyOptions`), the families to run, the
  step count, a physical `Invariant` (conserves-linear-momentum /
  conserves-total-energy / settles-on-static-support), and an anti-no-op
  "ran" signal — plus a single harness test (`AllScenesRunUnderEveryFamily`)
  that runs every (scene, family) pair from one loop, logs the `StepMetrics`
  rows tagged `<scene>:<family>`, and asserts finiteness, the per-family
  invariant, that each pair genuinely advanced the scene, and cross-family
  gross-outcome agreement (robust conservation-budget tolerances). Four cases:
  `rigid_head_on` + `sphere_drop` (rigid-contact axis) and `pendulum_1link` +
  `pendulum_2link` (integration axis); the 2-link case confirms the two
  integration families are genuinely distinct paths (final energy −18.2298 vs
  −18.2052). `test_cross_family_corpus` 1/1, golden 3/3, lint +
  `check-api-boundaries` green; pure-additive (no library change). **Remaining:**
  a standalone harness binary + manifest-driven packet writer (replacing the
  per-scene script fleet) and the cross-family dashboard row set,
  contact/iteration metric population (needs a non-const narrow-phase pass),
  world-frame multibody-link momentum aggregation, and a corpus-wide
  order-of-convergence sweep (this seeds it for one scene).
  NOTE: independent of this
  slice, the branch carries one **pre-existing** red test
  (`World.BakedDynamicRigidIpcStepsDoNotGrowWorldBaseAllocator`, an exact
  rigid-IPC allocation-counter off-by-one) introduced by the merged main
  #2996/#3005 allocator-modernization + C++23 raise — verified pre-existing by
  stashing this slice and reproducing it on the clean WP-091.23 tree; tracked
  for separate triage.

### WS3 — Compute axis reality

#### WP-091.30 Executor primitives and one converted stage per domain

- Objective: stages source CPU parallelism from the injected executor through
  two or three primitives (parallel-for over bodies/structures/islands/colors
  with deterministic chunked reduction); the per-solve thread pools and the
  public worker-thread knob are deleted.
- Scope: the executor contract and both executors; one reference stage
  conversion per domain (multibody structure loop, VBD color loop, contact
  island loop); deformable solver options cleanup; disposition of the native
  collision module's raw-thread batch fan-out — delete it if unused or hoist
  the parallel-for primitive so collision does not depend on
  simulation/compute (layering rule).
- Non-goals: GPU; graph-topology redesign (WP-091.34).
- Acceptance evidence: converted stages produce bitwise-identical results
  sequential-vs-parallel (extend the existing determinism-test pattern); the
  thread-pool code and the public knob are gone.
- Gates: `pixi run lint`, `pixi run build`, `pixi run test-unit`.
- Dependencies: WP-091.2.

#### WP-091.31 Per-World accelerator policy

- Objective: the process-global GPU function-pointer seam is replaced by a
  per-World accelerator handle resolved at bake; the registrar only
  advertises availability.
- Scope: the PSD projector backend seam; CUDA registrar; multi-World batch
  stepping interaction.
- Non-goals: new GPU kernels; wheel packaging.
- Acceptance evidence: two Worlds in one process can run different
  accelerator policies; concurrent batch stepping race-checked (first TSAN
  coverage for this path).
- Gates: `pixi run lint`, `pixi run build`, `pixi run test-unit`; CUDA smoke
  where a device is available.
- Dependencies: WP-091.30.

#### WP-091.32 O(n) shared articulated core

- Objective: the semi-implicit multibody family consumes a shared O(n)
  articulated inverse-mass apply (promoted from the variational family, with
  armature support), contact prep builds Delassus operators column-by-column
  via applies, and per-contact ownership scans use a baked hash.
- Scope: promote the existing apply into a shared multibody component
  consumed by both integration families; the unified constraint path accepts
  an apply-operator instead of dense inverse blocks; rename the family's
  documented method honestly until true ABA lands.
- Non-goals: new integration methods; variational solver changes beyond the
  shared extraction.
- Acceptance evidence: dense mass-matrix inversions eliminated from the
  steady-state step (profile evidence); dynamics parity tests green against
  golden trajectories; method naming updated in the architecture page and
  capability declarations.
- Gates: `pixi run lint`, `pixi run build`, `pixi run test-unit`, tracked
  benchmark delta.
- Dependencies: WP-091.2, WP-091.21.

#### WP-091.33 Batched-World and device-residency design notes

- Objective: the batching endgame has an owner and a contract — a design
  note pinning batched stepping semantics to "identical to n independent
  sequential Worlds", the baked SoA State-block architecture (leading batch
  dimension, immutable Model blocks), the precision policy (double/float per
  backend), and the residency seam a device backend needs.
- Scope: design notes under `docs/design/` (extending the scalable-compute
  owner docs); no production code beyond spikes.
- Non-goals: implementing the batched World (follow-up packets are cut from
  the accepted note).
- Acceptance evidence: notes accepted by the maintainer and indexed; the
  follow-up packet set is drafted into this plan; the existing batch seeds
  are labeled (canonical-direction vs heterogeneous-fallback) in their
  headers.
- Gates: docs-only gate set.
- Dependencies: WP-091.21 (index/Model artifact shapes the State blocks).

#### WP-091.34 Graph granularity policy

- Objective: compute-graph nodes are coarse units (islands, trees, bodies,
  colors) with the lowered task graph cached against a topology revision;
  the dead per-entity rebuild path is deleted.
- Scope: kinematics graph batching to O(workers) chunks; graph/lowering
  cache; remove the unused per-entity integration-stage graph path.
- Non-goals: converting additional stages (WP-091.30 owns conversions).
- Acceptance evidence: steady-state parallel step pays zero graph-build cost
  (profile evidence); parallel-vs-sequential determinism tests green.
- Gates: `pixi run lint`, `pixi run build`, `pixi run test-unit`.
- Dependencies: WP-091.30.

### WS4 — Facade and public API

#### WP-091.40 Reserve general state-vector semantics

- Objective: the general names (`getNumDofs`, `getStateVector`,
  `getControlVector`) stop carrying a translational-rigid-only slice; the
  slice moves behind an explicit scoped view and per-domain generalized
  coordinates are defined with stable dense-index ordering.
- Scope: the World state/control surface; wire the existing state-space
  machinery; dartpy properties and stubs; revise the differentiable-sim
  design doc that chose the current names.
- Non-goals: implementing articulated state vectors beyond the defined
  contract (tracked by the differentiable/batching plans).
- Acceptance evidence: a multibody-only world no longer reports zero DOFs or
  documents it explicitly via the scoped view; binding/stub/doc surfaces
  agree; existing consumers migrated.
- Gates: `pixi run lint`, `pixi run build`, `pixi run test-unit`,
  `pixi run test-py`, `pixi run check-api-boundaries`.
- Dependencies: WP-091.21 (ordering), maintainer sign-off on naming.

#### WP-091.41 JointSpec construction surface

- Objective: joint creation collapses to one verb taking a spec value object
  and frame-derived endpoints, with one query family; solver-family
  preconditions validate at bake instead of living in method names.
- Scope: the World joint-factory surface; `JointSpec` anchor extensions;
  bindings/stubs; migration of in-repo callers.
- Non-goals: new joint types; changing joint dynamics.
- Acceptance evidence: the parallel add/get method families are gone;
  bindings and stubs match; examples/tests migrated.
- Gates: `pixi run lint`, `pixi run build`, `pixi run test-unit`,
  `pixi run test-py`, `pixi run check-api-boundaries`.
- Dependencies: WP-091.12 (selection validation at bake).

#### WP-091.42 Policy objects for solver knobs on shared handles

- Objective: solver-family vocabulary leaves the shared handles: AVBD-named
  joint stiffness knobs and the deformable-solver booleans on rigid bodies
  become family policy value objects or physical-policy names.
- Scope: joint/rigid-body handle surfaces; the IPC stage options exposed as a
  facade policy object so precise tuning no longer requires the
  custom-pipeline overload; bindings/stubs.
- Non-goals: removing capabilities; default behavior changes.
- Acceptance evidence: no algorithm-branded names on shared public handles;
  IPC tuning reachable from `WorldOptions`; bindings match.
- Gates: `pixi run lint`, `pixi run build`, `pixi run test-unit`,
  `pixi run test-py`, `pixi run check-api-boundaries`.
- Dependencies: WP-091.12, WP-091.14.

#### WP-091.43 Installed-header split and curated diagnostics

- Objective: the installed public compute surface is the minimal custom-stage
  contract (stage base, pipeline, metadata/profile types); concrete family
  stages and their internal stats structs move to internal headers with
  curated diagnostics snapshots on the facade.
- Scope: the public install allowlist; the world-step-stage header split; a
  curated rigid-IPC diagnostics value object following the existing
  deformable-diagnostics pattern; resolve the public-API design doc's stance
  on stage objects explicitly.
- Non-goals: removing the custom-pipeline extension seam.
- Acceptance evidence: installed headers contain no per-family stats structs;
  the API-boundary inventory reflects the split; doc tension resolved in the
  C++ API design doc.
- Gates: `pixi run lint`, `pixi run build`, `pixi run test-unit`,
  `pixi run check-api-boundaries`.
- Dependencies: WP-091.15 (family modules give the internals a home).

#### WP-091.44 Python surface alignment and lifecycle polish

- Objective: the dartpy surface matches its design doc — core scene-authoring
  symbols flat, compute/diff in submodules — and the World lifecycle is
  predictable (idempotent finalize, bake-time validation, documented step
  overload semantics).
- Scope: dartpy module layout and stubs; `enterSimulationMode` idempotence
  and exception taxonomy; move per-step registry-scanning validations to bake
  and the mutating setters; document the step/sync overload contracts in the
  public header.
- Non-goals: new Python features.
- Acceptance evidence: Python module layout matches the design doc's declared
  symbol set; per-step validation eliminated (profile or counter evidence);
  every step overload documented; stubs regenerated.
- Gates: `pixi run lint`, `pixi run build`, `pixi run test-py`,
  `pixi run check-api-boundaries`.
- Dependencies: WP-091.11 (bake-time validation).

## Acceptance Criteria

PLAN-091 is complete when:

1. WS0: evidence packets carry machine-recorded solver identity; the AVBD
   relabel landed; availability claims are CI-checked; golden trajectories
   gate refactors; legacy freeze is enforced; plan IDs are unique.
2. WS1: one internal solver contract with one selection idiom; a
   resolved-configuration report; one canonical contact assembly; a
   family-neutral joint model; family-scoped source modules.
3. WS2: Model/State/Control is physical (split components, baked dense
   index); the allocator story is real or removed; serialization identity is
   stable; the metrics/scene-corpus/harness substrate produces cross-family
   comparison rows from one command.
4. WS3: converted stages parallelize through the executor with bitwise
   determinism; no process-global backend state; the articulated core is
   O(n); batching/device-residency design notes are accepted with follow-up
   packets cut.
5. WS4: no solver vocabulary or internal stats on the installed public
   surface; reserved state-vector semantics; spec-based joint construction;
   Python layout matches its design doc.
6. The solver-family intake checklist items introduced with this plan are
   exercised by at least one new family intake without bypass.

## Revision Triggers

- A verified finding in the assessment doc is resolved or refuted — update
  both documents in the same change.
- A new solver family is approved for intake before WS1 lands — re-sequence
  so the family enters through whatever contract subset exists, and record
  the migration debt explicitly.
- The batching/device-residency design notes (WP-091.33) are accepted —
  draft the follow-up packet set into WS3.
- Maintainer direction changes priority between workstreams — reorder the
  dashboard entry and packet sequencing, not the packet contents.
