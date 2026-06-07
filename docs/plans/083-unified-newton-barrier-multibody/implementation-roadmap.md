# PLAN-083 Implementation Roadmap

This sidecar turns the unified Newton-barrier objective into implementation
phases. It does not own priority, status, horizon, next step, or gate; those
remain in [`../dashboard.md`](../dashboard.md). It also does not replace the
paper/deck manifest. The manifest owns row coverage, while this file owns the
order in which DART should earn those rows.

## Completion Target

PLAN-083 is complete only when DART owns the full unified Newton-barrier method
family across CPU and GPU:

- shared deformable, rigid, affine, articulation, codimensional, contact,
  friction, restitution, and diagnostics contracts behind DART-owned
  DART 7 `World` capabilities;
- DART-owned tests proving correctness parity across CPU/GPU and across the
  variant owners named in
  [`ipc-variant-consolidation.md`](ipc-variant-consolidation.md);
- benchmark packets that beat DART incumbents, audited references, and the
  paper/deck numbers at matched accuracy, or record an explicit accepted
  limitation for rows that cannot be matched;
- every cited figure, table, unit family, performance row, and demo scene in
  [`paper-deck-manifest.md`](paper-deck-manifest.md) mapped to tests, py-demos,
  benchmark/profiling evidence, visual evidence, or a maintained
  not-applicable rationale.

CPU-only work may land first when that keeps reviewable slices small. It is not
the final target. GPU work stays private and benchmark-gated until same-scene
CPU/GPU parity and timing packets exist.

## Phase 0: Current Foundation

Current branch evidence:

- shared Newton-barrier primitive owners exist for primitive distances,
  clamped-log barriers, and tangent stencils;
- deformable compatibility headers forward to the shared owner, while rigid IPC
  consumes the shared owner directly;
- ABD has internal affine state/surface adapters, affine primitive barrier and
  friction chain-rule rows, orthogonality energy, and rigid projection oracles;
- `bm_affine_body_dynamics` has smoke rows for affine primitive mapping, a
  matched rigid IPC oracle row, and orthogonality energy.

Gate before leaving this phase: keep the current Phase 1/2 focused tests,
benchmark smokes, and API-boundary checks green after every base-branch merge.

## Phase 1: ABD Comparison Packet

Convert the current ABD benchmark smoke shape into the first comparison packet:

- define the row in `paper-deck-manifest.md` against a specific ABD deck or
  unified-paper artifact;
- record scene parameters, timestep, stiffness, contact geometry, accuracy
  metric, direct DART incumbent comparison, rigid IPC oracle comparison, and
  paper/deck target;
- keep the packet as benchmark/profiling evidence, not a runtime capability
  claim.

Gate: benchmark JSON and a manifest row prove the comparison is reproducible.
The current first packet is the `abd-alg-affine-body` microbench row in
[`paper-deck-manifest.md`](paper-deck-manifest.md), generated and checked with
`pixi run bm-abd-comparison-packet`; it intentionally compares the internal
affine point-triangle primitive mapping with the rigid IPC oracle and
orthogonality energy only, without claiming a runtime solver or paper-scale
scene result. This micro-packet does not need a two-body affine contact
micro-solve before shared-contract scouting starts; add that solved-state row
only when a broader ABD packet needs runtime residual or stepping evidence.

## Phase 2: Shared Solver Contracts

Promote solver pieces only after second-use evidence:

- PSD projection contract shared by deformable, rigid, and ABD rows;
- sparse projected-Newton residual, convergence, and failure diagnostics;
- conservative line-search feasibility contract for barrier-active steps;
- lagged friction contract with tangent basis, normal-force lagging, and work
  diagnostics;
- benchmark packet schema for per-step timing and solver subphase timing.

Gate: cross-variant unit tests show the shared contract is behaviorally
identical for its consumers, and variant-specific terms remain in their owner
plans until proven shared.

## Phase 3: Unified Articulation Constraints

Implement the paper/deck constraint families behind DART-owned names:

- point connection and fixed point;
- hinge and cone twist;
- sliding and relative sliding;
- distance and bounded distance;
- sliding range and rotation range.

Linear equality constraints should use the paper's sparse change-of-variable
approach only after rank, sparsity, residual, and solve-equivalence tests prove
it beats or matches the current DART path. Nonlinear equality and inequality
range constraints must keep derivative, PSD, barrier feasibility, and
diagnostic tests.

Gate: every articulation row has focused tests and a manifest link before any
py-demo or runtime promotion.

## Phase 4: Restitution, BDF-2, And Rayleigh Damping

Add energy-aware opt-in solver capabilities:

- BDF-2 incremental-potential stepping with restart/serialization behavior;
- falling-box energy diagnostics across timestep, Young's modulus, barrier
  stiffness, activation distance, and gravity sweeps;
- semi-implicit Rayleigh damping on barrier/contact/articulation potentials;
- rotating-board or hinge damping evidence.

Gate: energy curves and damping sweeps match the qualitative paper/deck trends,
controls have documented units/defaults, and existing rigid stepping defaults
remain unchanged until a separate promotion gate says otherwise.

## Phase 5: Mixed-Domain Coupling

Unify runtime contact buffers and coupler seams for:

- rigid, deformable, affine, particle, rod, shell, and codimensional state
  adapters;
- shared candidate generation and conservative CCD;
- contact, friction, and diagnostics that preserve the variant owners'
  correctness oracles;
- serialization and deterministic restart for mixed scenes.

Gate: mixed-domain stepping tests prove no-intersection/no-inversion where
applicable, finite energy, deterministic restart, and diagnostics sufficient to
debug failed line searches or non-convergence.

## Phase 6: CPU Scene Corpus And Py-Demos

Port the paper/deck corpus into DART-owned artifacts:

- py-demos categories from `paper-deck-manifest.md`;
- headless smoke commands and long-horizon visual evidence;
- benchmark/profiling packets for every performance scene;
- manual/not-applicable rationale for rows that cannot be reproduced.

Gate: row status moves only when the artifact, command, invariant, evidence
file or PR, and limitation status are explicit.

## Phase 7: Private GPU Parity And Speed

Promote GPU kernels in the same order as CPU evidence:

- contact stencils and candidate filtering;
- CCD and line-search kernels;
- barrier/friction local kernels and PSD projection;
- assembly and linear-solve paths only after CPU packet shape is stable.

Gate: same-scene CPU/GPU result parity, deterministic tolerance policy,
setup/transfer/readback timing, kernel/solve timing, and speedup over the DART
CPU path. Public APIs must not expose CUDA, streams, devices, memory pools,
backend tasks, or solver registries.

## Phase 8: Completion Audit

Before calling PLAN-083 complete:

- `paper-deck-manifest.md` has zero unclassified rows;
- every non-planned row links to concrete tests, demos, benchmarks, visual
  evidence, or a maintained rationale;
- CPU and GPU packets exist for rows with performance or GPU claims;
- public headers and dartpy bindings pass API-boundary checks with no upstream
  project, ECS, registry, coupler, solver, or backend-type leaks;
- durable design/onboarding docs own the landed architecture, and any temporary
  dev-task folder is retired according to `docs/dev_tasks/README.md`.
