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

#### WP-091.1 Solver-identity recording and AVBD evidence relabel

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

#### WP-091.2 Golden trajectories for the default step

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
  Dependencies lines, which are the source of truth.

#### WP-091.3 Architecture-page claim lint

- Objective: every ✅/available claim in `docs/readthedocs/architecture.md`
  is CI-checkable: each marked row cites a header symbol and a test.
- Scope: a `scripts/check_*.py` following the completion-audit checker
  pattern, registered in `pixi.toml` lint/check aggregates, plus a meta-test.
- Non-goals: re-grading the markers themselves (done with this plan's
  creation); generating the tables.
- Acceptance evidence: checker passes on current docs; meta-test proves it
  catches an injected unsupported claim.
- Gates: `pixi run lint`, `pixi run check-lint`, new checker meta-test.
- Dependencies: none.

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
- Dependencies: maintainer direction on Decision 5 options.

#### WP-091.5 Renumber colliding plan IDs

- Objective: `PLAN-080` and `PLAN-082` each identify exactly one initiative.
- Scope: assign fresh IDs to the Performance Dashboard and Linear-Time
  Variational Integrator entries; `git mv` the affected plan files/sidecars;
  update every cross-reference (the design docs, dev-task folders,
  `docs/readthedocs/papers.md`, `docs/readthedocs/architecture.md` (its
  multibody solver table cites PLAN-082), and `solver-family-intake.md` all
  cite these IDs); add a plan-ID uniqueness check to the docs-policy script.
- Non-goals: changing any plan's content or status.
- Acceptance evidence: one dashboard block per ID; link checker / docs policy
  green; uniqueness check has a meta-test.
- Gates: `pixi run lint`, `pixi run check-docs-policy`.
- Dependencies: none. Hazards: 20+ cross-referencing files; grep-by-ID hits
  the wrong block while duplicates exist — fix the duplicates in one packet.

### WS1 — Internal solver contract and selection

#### WP-091.10 Solver contract step 1: virtual finalize/prepare

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

#### WP-091.11 Minimal capabilities and validated finalize

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

#### WP-091.12 Single selection idiom

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

#### WP-091.13 Canonical contact assembly

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

#### WP-091.14 Family-neutral joint model

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

#### WP-091.20 Model/State/Control component split

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

#### WP-091.23 Stable serialization identity

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

#### WP-091.24 Metrics contract, scene corpus, one harness

- Objective: apples-to-apples comparison has a substrate: a solver-agnostic
  step-metrics value object populated by every family, a registered-builder
  scene corpus, and one benchmark harness emitting rows tagged with scene ID,
  resolved solver identity, and metrics.
- Scope: a small `StepMetrics`-style value object (per-domain energies,
  momenta, penetration, contact counts, solver iterations/residual); a scene
  builder registry under `tests/`; one harness binary; a manifest-driven
  packet writer replacing the per-scene script fleet; a tracked
  cross-family `World::step` scenario row set in the perf dashboard.
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
