# PLAN-123: Citation-Driven Simulation Trust

- Operating state: `PLAN-123` in [`dashboard.md`](dashboard.md)
- Outcome: every material external or historical claim about DART can be
  reproduced, classified, and guarded on the applicable branch; every DART 7
  solver benchmark proves the requested and resolved method, contact/force
  semantics, convergence, failure policy, and matched speed-accuracy evidence;
  the resulting evidence determines whether exact-cone contact, contact-aware
  inverse dynamics, and adoption/domain extensions are promoted.
- Current evidence:
  - DART 7 already provides `ResolvedSolverConfiguration`, `StepMetrics`,
    `WorldStepProfile`, structured `LcpResult`, rigid/unified contact problems,
    multiple solver families, differentiability, and post-bake allocation gates.
  - DART 6.20 has active collision/performance and deformable paper-parity work,
    a stable backend compatibility contract, and Gazebo/gz-physics downstream
    gates.
  - The DART citation audit found mostly neutral infrastructure use, clear
    positive extension evidence from Nimble/RobotDART, and bounded criticisms
    involving contact-force transients, friction/contact formulation,
    rigid-only scope, and test-specific failures.
  - Current plans own individual methods but no owner currently converts the
    cross-paper citation record into one branch-qualified claim corpus and one
    solver-neutral trust contract.

## Owner docs

- Durable architecture and semantics:
  [`../design/contact_trust_and_observability.md`](../design/contact_trust_and_observability.md)
- Citation and claim matrix:
  [`123-citation-driven-simulation-trust/citation-claim-corpus.md`](123-citation-driven-simulation-trust/citation-claim-corpus.md)
- Active implementation state:
  [`../dev_tasks/citation_driven_simulation_trust/`](../dev_tasks/citation_driven_simulation_trust/)
- Existing solver owners:
  PLAN-080, PLAN-082, PLAN-083, PLAN-104, PLAN-110, and PLAN-122.
- Archived heritage: PLAN-091 landed `ResolvedSolverConfiguration`
  (WP-091.11 slice 1), `StepMetrics`, `WorldStepProfile`, and
  `docs/design/dart7_cross_family_metrics_corpus.json`; PLAN-123 WS4 owns the
  recorded silent-substitution follow-up on those seams.
- DART 6 adaptation owner on `release-6.20`:
  `docs/design/dart6_citation_driven_contact_trust.md`.

## Scope

PLAN-123 is cross-cutting evidence and shared-contract work. It does not absorb
the detailed implementation ownership of existing solver plans. It owns:

- the stable citation-claim identifiers and dispositions;
- the evidence-packet and matched-comparison contract;
- common contact observation, identity, impulse/wrench semantics, and
  cross-family solve reporting;
- promotion decisions that require evidence across solver families;
- branch coordination and stopping conditions.

Existing plans continue to own their methods, kernels, paper corpora, and public
API slices. A PLAN-123 row links to the solver owner rather than duplicating its
checklist.

## Dependencies and sequencing

1. The initial corpus and evidence schema land before new solver architecture.
2. Existing DART 6 and DART 7 scenes/metrics are reused before adding fixtures.
3. Contact semantics and requested/resolved identity land before comparing
   methods.
4. Exact-cone work starts only after a recorded GO based on corpus gaps.
5. Contact-aware inverse dynamics follows a stable cone/contact problem.
6. One-model/many-state, planning, environment/sensor, biomechanics, and
   rod/shell work proceeds as bounded companion or existing-plan slices after
   the common trust foundation.

## Workstreams

### WS0 — Current-state and ownership audit

- Reconcile the corpus with current `main`, `release-6.20`, open PRs, issues,
  plans, active dev tasks, and benchmark/evidence infrastructure.
- Mark already-landed work; do not recreate it.
- Route every row to exactly one implementation owner and one durable evidence
  owner.
- Verify that PLAN-123 and any DART 6 plan ID do not conflict with newer state.

Exit: no duplicate owners, stale status, or unbounded “support all papers”
language.

### WS1 — Claim corpus and machine-readable evidence contract

- Implement a checked manifest/schema for corpus rows and dispositions.
- Require source/scene/build/configuration/metric/review provenance.
- Add validators for missing resolved-method data, unsupported zero-valued
  metrics, stale evidence commits, incomplete ensembles, and unbounded claims.
- Integrate with existing visual/evidence publication rather than inventing a
  parallel artifact system.

Exit: at least three representative rows fail the validator before completion
and pass after complete evidence is supplied.

### WS2 — Baseline reproduction campaign

First wave:

- directional rolling/friction sweep;
- dense inelastic and elastic contact stress;
- articulated energy/momentum and controlled-contact scenes;
- heel-strike/toe-off transition;
- high mass-ratio/ill-conditioned stacks and manipulation;
- reset/concurrency and planning collision-query throughput.

Each row runs on the relevant DART 6 and DART 7 configurations or records why a
lane is not applicable. Results are branch/version-qualified, never generalized
from one historical version.

Exit: each first-wave row has a disposition, raw packet, negative control, and
review record.

### WS3 — Contact observation and physical-result semantics

- Stabilize canonical body/shape/feature ordering and manifold/point identity.
- Define normal orientation and deterministic tangent-history policy.
- Distinguish raw impulse, interval-average wrench, continuous force, filtered
  analysis output, and event-integrated impulse.
- Preserve existing query APIs and add only the smallest value-type/API surface
  supported by multiple consumers.
- Add gait/grasping/stack tests for sign, frame, interval, identity, reset, and
  clone behavior.

Exit: a user can determine exactly what a contact value means without reading
the solver implementation.

### WS4 — Cross-family resolution, diagnostics, and Pareto dashboard

- Extend existing `ResolvedSolverConfiguration`, `StepMetrics`,
  `WorldStepProfile`, and solver results.
- Report fallback/substitution per group/island and cumulatively per step.
- Add comparable residual, complementarity/cone, penetration/separation,
  contact-mode, warm-start, iteration, timing, and allocation fields where
  mathematically meaningful.
- Publish matched-residual/accuracy comparisons and label re-baselines when
  outcomes/contact sets differ.

Exit: no benchmark or demo can silently claim a requested method that did not
run.

### WS5 — Exact-cone contact GO/no-GO and implementation

- Audit existing ADMM, Dojo/IPM, SAP, boxed-LCP, IPC, and AVBD seams.
- Define one DART-owned exact-cone problem separating physical compliance from
  numerical regularization.
- Build one CPU reference algorithm and evidence adapters before public API.
- Add a second algorithm only when it tests a real shared contract.
- Promote only if named corpus rows establish a useful Pareto region.

Exit: recorded GO with promotion evidence, or NO-GO with reusable findings and
no permanent experimental surface.

### WS6 — Contact-aware inverse dynamics

- Reuse the stable contact/material problem.
- Provide a pure query for torques, contact wrenches, feasibility, limits,
  active constraints, and residual explanations.
- Add whole-body, exoskeleton, and manipulation fixtures.
- Keep mutation, solver internals, and third-party optimization types private.

Exit: forward/inverse consistency and infeasibility tests pass on named corpus
scenes.

### WS7 — Execution and adoption layers

- Finish explicit one-model/many-state capture, restore, partial reset, RNG,
  solver-history, and batch semantics.
- Derive bounded planning and environment/sensor companion work.
- Extend differentiable validity diagnostics and batched/checkpointed VJPs
  through PLAN-110.
- Start biomechanics or rods/shells only with a named owner, shared-primitive
  audit, and bounded corpus.

Exit: each extension is a separate accepted initiative or companion with no
core dependency leakage.

## DART 6 coordination

DART 6 work is implemented on `release-6.20` under its branch-local design and
dev-task owners. PLAN-123 consumes its evidence but does not direct mechanical
backports. DART 6 may reproduce claims and fix confirmed defects while
preserving ABI, defaults, packages, C++17/pybind11/OSG, and gz compatibility.
New solver families, public contact architecture, one-model/many-state APIs,
and domain expansion are DART 7 only.

## Acceptance criteria

- Every initial corpus row has a source-bound disposition and reproducible
  commands; no row is closed by prose or screenshots alone.
- Requested and resolved detector/solver/integrator/backend/precision are
  recorded for every result-affecting packet.
- Contact impulse/wrench/force semantics are explicit and tested.
- Comparable metrics distinguish unsupported from measured zero.
- Failure, fallback, continuation, and partial-state policies are typed and
  tested.
- Promoted repeated step shapes satisfy PLAN-122 post-bake allocation gates.
- Performance claims use validated hosts and distributions; solver comparisons
  are matched by accuracy/residual or explicitly labeled non-equivalent.
- Behavioral claims use deterministic repeats or perturbation ensembles
  appropriate to the system.
- Public APIs remain DART-owned, easy on the common path, serializable when
  result-affecting, and backend/storage/framework neutral.
- DART 6 and DART 7 changes remain separate, branch-appropriate PRs.
- Each slice runs branch-required lint/build/tests, visual evidence when
  applicable, two clean independent reviews, and a changelog decision.
- Completing dev-task folders are removed in their completing PR after durable
  content is promoted.

## Non-goals

- Declaring one solver universally superior.
- Porting all citing-paper application stacks into core DART.
- Adding exact-cone, biomechanics, sensors, planning, rods, or shells to DART 6.
- Exposing reference-project names, solver registries, ECS/device storage, or
  tensor frameworks through the public C++ core.
- Using citation count or routine library use as evidence of physical accuracy.
- Keeping an unbounded paper-support backlog in this plan.

## Progress log

### 2026-08-14 — WS0/WS1 foundation and first-wave packets

- WS0 audit reconciled the corpus against `main` 20501341226 and
  `release-6.20` 39ccd52068b: plan IDs confirmed free, PLAN-091 heritage
  mapped, open PRs (#3432, #3377, #3431, #3428) and issue #3056 routed to
  existing owners, and existing demos scenes recorded as fixture seeds.
- WS1 landed the checked claim manifest and the fail-closed
  `pixi run check-citation-evidence` gate on both branches, wired into
  `check-lint`, with permanent intentionally incomplete negative controls.
  The gate rejects prose/non-JSON/outside-`evidence/`/negative-control/
  non-string lane references, dangling or directory `raw_paths`, digests that
  disagree with the published scene, single-run ensembles, spelled
  placeholders, and any exact zero that is not declared a real measurement or
  typed unsupported with a reason.
- WS2 first-wave packets on `main`: CT-001 rolling direction (`reproduced`),
  CT-002 dense inelastic contact (`reproduced`), CT-003 dense elastic contact
  (`unresolved`), CT-004 articulated energy drift (`unresolved`), CT-005 PD
  tracking (`unresolved`). On `release-6.20`: CT-001 detector sweep
  (`reproduced` on fcl/dart/ode; bullet excluded for lacking the pyramid
  signature). Three of six dispositions are negative because the cited
  behavior was not observed or the cited quantity was not measured.
- Review record: three independent rounds per branch found a validator
  bypass twice (nested paths, then non-string lane entries),
  unsupported-as-zero metrics, false provenance strings, a verification block
  imported from the wrong branch, and two dispositions that were not earned.
  All fixed in-branch; details live in
  `docs/dev_tasks/citation_driven_simulation_trust/verification.md`.
- WS4 first slice: `World.resolved_configuration` is now exposed to Python
  (`ResolvedSolverConfiguration` and `ResolvedConfigurationNote` bindings,
  stubs, and tests), so every packet records the World's own bake-time
  resolution per domain -- requested, resolved, reason, substitution flag --
  instead of echoing back the requested option. A benchmark can no longer
  name a method that did not run. A comparable per-solve residual remains
  unexposed and is still typed unsupported everywhere.
- Known gap: the validator does not arithmetically cross-check derived
  metrics against `raw_rows`, so a falsified summary would pass. Closing that
  needs the gate to know each packet's derivation (WS4-scale).

## Revision triggers

- A corpus row shows an existing plan already owns the proposed shared surface.
- Exact-cone evidence produces a GO/NO-GO decision.
- A contact metric proves formulation-specific and cannot retain common meaning.
- DART 6 compatibility or downstream evidence changes the branch boundary.
- PLAN-080/082/083/104/110/122 changes the shared seam.
- A task completes, splits, is parked, or requires a new bounded plan.
