# DART 6.20 Citation-Driven Contact Trust

## Status

Durable compatibility and evidence contract for `release-6.20`. Active
implementation state lives in
`docs/dev_tasks/dart6_citation_contact_trust/`. The DART 7 long-term owner is
`docs/design/contact_trust_and_observability.md` on `main`.

## Purpose

DART 6.20 should answer historical and current claims about DART 6 with
source-faithful reproductions, permanent tests/benchmarks, and honest
dispositions while preserving the LTS contract. This branch may improve
existing collision, contact, solver, sleeping, parser, and diagnostic behavior
when a defect is demonstrated, but it is not a vehicle for DART 7 architecture
or new public solver families.

## Compatibility boundary

All work preserves unless explicitly authorized otherwise:

- C++17 and pybind11;
- installed headers, symbols, class layouts, components, and parser surfaces;
- FCL `PRIMITIVE` as the built-in default;
- real FCL, Bullet, ODE, and DART-owned collision backends and factory keys;
- OSG and consolidated `dart-demos`;
- default simulation behavior for unaffected scenes;
- Gazebo/gz-physics/gz-sim compatibility;
- public `collision::Contact` and constraint/contact-surface semantics.

`main` is reference evidence only. Do not import C++23, nanobind, EnTT World
storage, DART 7 solver/backend APIs, or generated DART 7 workflows.

## Allowed work

- Source-bound reproductions of citation, benchmark, and issue claims.
- Additive tests, benchmarks, demos, scripts, and evidence schemas.
- Internal or opt-in diagnostics with compile-time/runtime-zero common-path
  overhead where the branch contract requires it.
- Implementation-local correctness, determinism, allocation, and performance
  fixes with negative controls.
- Additive non-virtual helpers only after ABI/source review.
- Documentation clarifying impulse/force/wrench semantics without changing
  established ABI.
- Separate `main` fixes for defects shared across branches.

## Excluded work

- New exact-cone/NCP, IPC, VBD/AVBD, differentiable, batch-World, rod/shell, or
  biomechanics solver architecture.
- Public model/state/contact redesign.
- Default detector or solver changes.
- Removal or facade replacement of FCL/Bullet/ODE.
- New required dependencies or language/binding/rendering floor changes.
- Broad refactors justified only by DART 7 cleanliness.
- Silent contact smoothing or changed downstream force conventions.

## Owner integration

Before creating new fixtures or task owners, audit and reuse:

- PLAN-621 and `docs/dev_tasks/dart6_performance_generalization/`;
- PLAN-622 and `docs/dev_tasks/dart6_deformable_body_performance/`;
- `docs/design/dart6_collision_backends.md`;
- existing DART 6 benchmark, capture, evidence, and AI verification tooling;
- current open PRs/issues and any newer completion/retirement state.

A corpus row already owned by PLAN-621/622 stays there. The citation task points
to its evidence and does not copy the implementation checklist.

## Claim and evidence contract

Use the same stable claim IDs and dispositions as DART 7, but every packet is
explicitly `release-6.20` and records:

- exact base/current commit;
- source model, license, conversion, and digest;
- detector, constraint solver, timestep, iterations, threads, and defaults;
- initial state/control/seed/ensemble/window;
- contact count/profile, state/rest hash, penetration, energy/momentum,
  residual/failure where available, timing, and allocation;
- exact commands and host validity;
- disposition and limitation;
- Gazebo/gz relevance;
- review evidence.

If DART 6 lacks a comparable metric, record it as unsupported or derive it in
the harness without changing public runtime APIs. Never encode unsupported as
numeric zero.

## Contact semantics

DART 6 public compatibility constrains API changes, but evidence must still
distinguish:

- collision query geometry;
- solver impulse;
- legacy per-step `Contact.force`/wrench data;
- interval-average or filtered analysis values;
- downstream Gazebo contact sensor transforms and sign/ownership.

Tests should pin object ordering, normal convention, point/frame transforms,
force/wrench sign, timestep dependence, reset/clone behavior, and downstream
conversion where applicable. Do not silently reinterpret released fields.

New analysis helpers should remain in tests/examples/scripts or additive
non-virtual APIs after compatibility review.

## Initial DART 6 rows

The durable row set for this branch (unordered — working priority lives with
the active task state, not here):

- the completed `3k_shapes` and sleeping/collision performance campaign
  (audit/guard);
- rolling/friction-direction behavior across FCL, DART, Bullet, and ODE where
  supported;
- dense inelastic/elastic contact finite-state and failure grids;
- heel-strike/toe-off raw impulse and legacy force/wrench interpretation;
- contact normal/object ordering and Gazebo wrench sensor regressions;
- high mass-ratio stacks/manipulation and current solver fallback behavior;
- existing PLAN-622 soft-contact robustness rows using perturbation ensembles
  (these contribute evidence without expanding the cap).

The cap is six common fixture families with dispositions. The working order
in which rows are attacked is mutable state and is owned by the PLAN-623
task home (`docs/dev_tasks/dart6_citation_contact_trust/README.md`) with the
branch manifest recording live lane status; this document records only which
rows exist and why.

## Fix policy

A DART 6 behavioral fix requires:

- baseline reproduction on a current clean release base;
- root cause;
- smallest compatibility-safe change;
- regression that fails before and passes after;
- state/contact/rest hash or explicit re-baseline;
- no unrelated default or backend changes;
- ABI/header/component/package audit when near a public boundary;
- `pixi run -e gazebo test-gz` for collision/constraint/World/downstream work;
- a separate DART 7 issue/PR assessment.

Performance gains cannot come from lost contacts, changed sleeping, cap hits,
skipped work that remains physically required, or a different model without
being labeled non-equivalent.

## Diagnostics and overhead

Diagnostics are opt-in and preferably test/harness-side. Any library
instrumentation must:

- preserve public class layout and vtables;
- be default-off when it adds work or state;
- compile out or have proved negligible common-path cost as appropriate;
- avoid global registries, locks, or heap allocation on every solve/step;
- report actual detector/solver/fallback and unsupported metrics honestly.

## Verification

Every slice runs:

- `pixi run lint`;
- focused build and C++/dartpy tests;
- relevant benchmarks with raw rows and validity checks;
- deterministic repeats/ensembles;
- visual evidence through the branch OSG/demos path when visible;
- `pixi run -e gazebo test-gz` when downstream-sensitive;
- two clean independent or role-separated reviews;
- explicit changelog decision.

Before completion, promote durable facts to this design, existing collision,
testing, profiling, user, or release owners, then remove the dev-task folder in
the completing PR.
