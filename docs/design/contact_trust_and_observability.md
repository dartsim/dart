# Contact Trust, Citation Reproduction, And Solver Observability

## Status

Durable DART 7 design and evidence contract. Mutable priority and execution
state live in `docs/plans/123-citation-driven-simulation-trust.md` and
`docs/dev_tasks/citation_driven_simulation_trust/`.

## Purpose

DART should be an auditable multi-solver research platform: a paper, benchmark,
application, or user report that makes a claim about DART must be convertible
into a reproducible scene, a branch/version-qualified result, and a permanent
regression or documented limitation. A benchmark must prove which method
actually ran, what physical quantity was measured, whether the solve converged,
and which fallback or substitution occurred.

The goal is not one universally best solver. The goal is a trustworthy
speed-accuracy-capability surface across sequential impulse, boxed LCP,
exact-cone contact, rigid/barrier contact, VBD/AVBD, differentiable paths, and
future methods while reusing one model, contact, state, diagnostics, and
evidence infrastructure.

## Existing DART 7 seams to extend

Do not create parallel replacements for existing DART-owned surfaces:

- `dart/simulation/body/contact.hpp` owns query contact data.
- `dart/simulation/compute/rigid_body_constraint.*` and
  `unified_constraint.*` own current rigid/articulated contact problem assembly.
- `dart/math/lcp/lcp_types.hpp` owns solver status, residual,
  complementarity, options, and problem/result vocabulary.
- `dart/simulation/compute/world_step_profile.hpp` owns
  `WorldStepProfile`, `ResolvedSolverConfiguration`, and `StepMetrics`.
- `docs/design/simulation_solver_architecture.md` owns solver-family,
  model/state/control/contact, coupler, and public-boundary rules.
- PLAN-080, PLAN-082, PLAN-083, PLAN-104, PLAN-110, and PLAN-122 own their
  existing solver, differentiability, and allocation work.

This design promotes shared contracts only where a second consumer proves the
need. Solver-specific mathematical state may remain local behind adapters.

## Branch boundary

### DART 7 `main`

DART 7 may add clean-break internal architecture and carefully promoted public
value types. It may add a solver-neutral contact-observation layer, exact-cone
problem family, contact-aware inverse dynamics, one-model/many-state execution,
and optional companion integrations. Public names remain DART-owned and
backend-neutral. ECS storage, solver registries, device handles, and reference
project names stay private.

### DART 6 `release-6.20`

DART 6 is evidence and maintenance, not an architecture backport. It may add
tests, benchmarks, demos, internal/opt-in diagnostics, and compatibility-safe
fixes to confirmed defects. It must preserve C++17, pybind11, installed
headers/components, ABI-sensitive layouts, default FCL behavior, OSG, and
Gazebo/gz-physics compatibility. See
`docs/design/dart6_citation_driven_contact_trust.md` on that branch.

A defect shared by both lines receives two independently reviewed PRs. DART 7
code is evidence for DART 6, never a mechanical backport justification.

## Citation claim contract

Every external or historical claim is a row with a stable identifier. Prose
alone cannot close a row. The row records:

- exact source and quoted/paraphrased claim;
- source publication date and DART version/commit when discoverable;
- target branch and current commit;
- scene/model source, license, conversion notes, and content digest;
- requested and resolved detector, solver family, integrator, precision,
  timestep, substeps, iterations/tolerances, threads/backend, and fallback;
- initial state, control, seed, perturbation ensemble, and measurement window;
- physical metrics, numerical metrics, timing methodology, and allocations;
- baseline/current commands and raw machine-readable evidence;
- disposition: `missing`, `reproduced`, `fixed`, `not-applicable`,
  `invalid-original-setup`, `version-specific`, or `unresolved`;
- claim boundary, limitations, and independent review evidence.

A row closes only when another person or clean-session agent can reproduce the
disposition from tracked commands and source-bound artifacts. A visually
plausible result never replaces a text/numeric oracle.

## Canonical contact lifecycle

```text
collision candidates
  -> canonical ordered contact observations
  -> persistent manifold/contact identities
  -> solver-family problem adapter
  -> solve
  -> typed solution and convergence report
  -> state update
  -> evidence/diagnostics snapshot
```

### Contact observation

A solver-neutral observation should eventually carry, where available:

- canonical ordered body and shape identities;
- shape-local feature identities and local/world witness points;
- normal orientation with a documented A-to-B convention;
- penetration or signed separation;
- stable manifold and point identities across small motions;
- deterministic tangent-frame seed/history;
- material inputs before combination;
- source detector and contact-generation policy.

Stable identity is required for warm starts, friction history, deterministic
ordering, gradients, replay, and fixed-capacity batches. Identity must be
invalidated explicitly when topology, shape, feature, or policy changes.

### Solver problem adapters

The observation layer does not force all methods into one equation. Adapters
may produce:

- velocity-level pyramidal boxed LCP;
- exact second-order-cone/NCP contact;
- barrier/variational contact;
- AVBD/VBD row systems;
- differentiable snapshots.

Shared geometry, materials, identity, and evidence stay common; formulation,
iteration state, and factorization remain solver-family-owned.

## Impulse, force, and wrench semantics

Impulse-based contact methods authoritatively produce an impulse over an
integration interval. `impulse / dt` is an average force over that named
interval, not an instantaneous continuous force.

DART should distinguish:

- raw contact impulse;
- average contact wrench with interval and origin;
- instantaneous force only for formulations that solve a continuous/compliant
  force;
- explicitly filtered/derived analysis signals;
- event-integrated impulse across an impact or gait phase.

Each result states frame, application point or wrench origin, interval,
manifold/contact identity, requested/resolved method, cone approximation,
convergence, and whether the value is raw or derived. Core simulation never
silently smooths or relabels impulses as instantaneous forces.

## Solver resolution and solve report

No benchmark, demo, recording, or published packet may identify a method only
from the requested option. Extend `ResolvedSolverConfiguration` rather than
inventing a second resolution system.

A comparable solve report should include, where meaningful:

- success/failure/maximum-iteration/numerical/unsupported status;
- termination reason and fallback/substitution;
- iteration and factorization counts;
- primal, dual, residual, complementarity, and cone violations;
- maximum penetration/minimum separation;
- active, separating, sticking, and sliding counts;
- warm-start use, rejection, and reset reason;
- regularization and physical-compliance values kept distinct;
- per-stage time and post-bake allocation counts.

Unsupported metrics are explicitly absent, not silently zero. Solver-specific
details may be nested, but cross-family fields retain common units and meaning.

## Evidence packet and Pareto comparison

A tracked evidence packet binds:

1. source claim and corpus row;
2. branch, commit, build configuration, dependency resolution, and machine;
3. scene/model digest and exact command;
4. requested/resolved configuration;
5. raw trajectory/metric output;
6. semantic visual evidence when the claim is visible;
7. statistical method, exclusions, and environmental validity checks;
8. result, limitation, and review record.

Performance comparisons use interleaved same-host runs or an equally justified
method, report distributions rather than one run, and compare at matched
accuracy/residual whenever methods solve different formulations. Changed
contact counts, sleeping outcomes, state hashes, or failure policies are
re-baselines, not speedups.

## Required initial corpus

The initial sidecar is
`docs/plans/123-citation-driven-simulation-trust/citation-claim-corpus.md`.
It includes:

- historical SimBenchmark rolling, dense-contact, energy/momentum, and
  articulated-control claims;
- heel-strike/toe-off force artifacts reported by exoskeleton work;
- exact-cone/high-conditioning motivation from compliant-to-rigid contact work;
- Nimble differentiability and performance claims;
- RobotDART reset/concurrency/research-workflow needs;
- PEEL and Fibration Trees planning/collision-query workloads;
- Behavior Policy Learning reset, controller, perception, and randomization
  needs;
- MASS/FARMS biomechanics and experiment-data needs;
- Gazebo Plants codimensional rod/plant gap;
- relevant DART issues and existing DART 6/7 benchmark packets.

The corpus is expanded only with a bounded acceptance row, not an open-ended
request to “support more papers.”

## Exact-cone and inverse-dynamics direction

An exact-cone initiative begins only after the common corpus and observability
foundation can measure its value. The missing unit is a DART-owned cone-contact
problem, not merely another class named ADMM.

The problem should separate physical compliance from numerical proximal
regularization and support at least one reference algorithm before public
promotion. Proximal ADMM and primal-dual interior point are candidate
algorithms over a shared problem; they are not separate public engines.

Promotion requires:

- materially lower directional friction error or better conditioning on named
  corpus rows;
- deterministic and fail-closed behavior;
- no post-bake allocation for the promoted step shape;
- matched-residual speed/accuracy evidence;
- stable warm starts/manifolds;
- serialization and requested/resolved reporting;
- finite-difference or implicit-gradient evidence before a differentiable
  claim.

Contact-aware inverse dynamics should reuse the same contact/material problem
and return torques, contact wrenches, feasibility, active constraints, and
residual explanations as a pure query. It must not mutate the primary World.

## One-model/many-state and companion layers

A baked immutable model with independent State, Control, Contacts, RNG, solver
history, and scratch per lane is the shared prerequisite for high-throughput
planning, RL, and differentiable rollouts. Reset semantics must explicitly
choose whether solver/contact history is preserved.

Planning adapters, environment/sensor APIs, and biomechanics should be
maintained companions unless a second core consumer proves a smaller shared
primitive. Gym, Torch, JAX, OMPL, dataset formats, neural controllers, muscles,
and clinical interpretation do not become core dependencies.

Rods and shells belong to existing deformable/contact solver families and reuse
shared distance, CCD, material, coupling, diagnostics, and evidence contracts.
They are sequenced after the trust foundation, not parallel feature expansion.

## Determinism, allocation, and failure policy

- Canonical ordering and identities are stable for the same model/state/policy.
- Baked repeated same-capacity steps do not grow World allocators or allocate
  through global heap/raw malloc paths.
- A solver failure cannot partially advance a World without an explicit,
  documented continuation policy.
- Fallback is recorded per island/group and cumulative across a step.
- Strict research modes fail closed; compatibility/realtime modes may use
  explicit configured fallback.
- Cloning, serialization, and reset preserve or intentionally clear all
  result-affecting configuration and history.
- Unsupported capability is an error or recorded substitution, never silence.

## Public API boundary

The common path remains simple: configure capabilities and step/query the
World. Advanced users receive DART-owned plain value objects and options.
Public APIs do not expose:

- solver registries or polymorphic implementation types;
- ECS components/storage;
- CUDA/SYCL streams or device pointers;
- reference repository/project names;
- factorization or reverse-pass cache ownership;
- third-party tensor/framework types in C++ core.

## Verification and promotion

Every behavioral slice requires:

- a negative control that failed before the fix or proves the metric is
  non-vacuous;
- focused unit/integration tests;
- deterministic repeated and perturbation-ensemble evidence where thresholds
  or chaotic outcomes are claimed;
- baseline/current raw packets;
- text-first physics oracle plus claim-tied visual review when visible;
- at least two clean independent or role-separated reviews on the post-fix
  state;
- branch-required lint/build/test/downstream gates;
- an explicit changelog decision.

The plan closes when the citation corpus, contact semantics, and evidence
packet schema have durable code/docs owners; DART 6 dispositions are promoted
to branch design/testing owners; and any remaining exact-cone, adoption, or
domain-extension work has a bounded durable plan rather than an eternal task
folder.
