# PLAN-123 Citation And Claim Corpus

## Purpose

This sidecar is the stable row-level intake for claims that motivate
PLAN-123. It is not a bibliography, sentiment table, or promise to reproduce
every citation. A row exists only when it has a bounded DART-relevant claim,
scene, metric, and closing condition.

The machine-readable manifest should be derived from this document once the
current repository evidence system is audited. Until then, this file is the
human authority for row identity and intended claim boundary.

## Dispositions

| Disposition              | Meaning                                                                                                                         |
| ------------------------ | ------------------------------------------------------------------------------------------------------------------------------- |
| `missing`                | No adequate current-branch evidence exists.                                                                                     |
| `reproduced`             | The claimed behavior is observed under a source-faithful or explicitly bounded reconstruction.                                  |
| `fixed`                  | A negative claim reproduces on the qualified baseline and no longer reproduces on the current target, with regression coverage. |
| `version-specific`       | The claim is valid for a named historical version but not generalized to other versions.                                        |
| `not-applicable`         | The branch/method cannot represent the claim and the reason is explicit.                                                        |
| `invalid-original-setup` | A demonstrated confound or invalid comparison prevents the original conclusion; the corrected experiment is recorded.           |
| `unresolved`             | Evidence is conflicting, incomplete, or not reproducible.                                                                       |

A `fixed` result is not automatically a performance improvement; changed
contact sets, sleeping, state hashes, failure policy, or physical model require
a re-baseline label.

## Required row fields

Each machine-readable row must carry:

- `id`, `title`, `source`, `source_date`, and exact claim text or bounded
  paraphrase;
- original DART version/commit and external model/source provenance when known;
- target branch/commit and owner plan/dev task;
- scene/model digest and license disposition;
- requested/resolved detector, contact method, integrator, precision, backend,
  timestep/substeps, iterations/tolerances, threads, and fallback policy;
- seed/perturbation ensemble and measurement window;
- physical, numerical, performance, allocation, and determinism metrics;
- baseline/current commands and evidence paths;
- disposition, limitations, claim boundary, and review record.

## Initial corpus

Row identity and claim boundaries below are stable. Live per-branch status,
owners, and dispositions are machine-readable in
[`claims-manifest.json`](claims-manifest.json), validated by
`pixi run check-citation-evidence`; this table is not re-edited per status
change. The 2026-08-14 current-state audit (WS0) is recorded after the table.

| ID     | Source / motivation                                      | Bounded DART claim or need                                                                                                               | First oracle                                                                                     | DART 6 lane                                                                   | DART 7 owner                                                             |
| ------ | -------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------ | ----------------------------------------------------------------------------- | ------------------------------------------------------------------------ |
| CT-001 | SimBenchmark / historical DART contact comparison        | Polyhedral friction can produce direction-dependent rolling/sliding behavior.                                                            | Rotate initial tangent direction; stopping distance, lateral drift, energy loss, cone violation. | Reproduce existing DART 6 methods/detectors only.                             | PLAN-080/123; compare SI, boxed LCP, future exact cone.                  |
| CT-002 | SimBenchmark dense `6x6x6` inelastic contact             | Dense contact may fail, become unstable, or scale poorly for some timestep/solver settings.                                              | Finite state, penetration, residual, energy, iterations, wall time over timestep grid.           | Compatibility-safe regression and benchmark.                                  | PLAN-080/122/123.                                                        |
| CT-003 | SimBenchmark dense elastic contact                       | Elastic dense contact may inject energy or expose solver failure.                                                                        | Restitution outcome, energy envelope, non-finite/failure status.                                 | Existing restitution/contact paths only.                                      | PLAN-080/123; formulation comparison.                                    |
| CT-004 | SimBenchmark articulated momentum/energy                 | Articulated integration/contact accuracy must be compared at matched cost.                                                               | Momentum/energy drift versus timestep and runtime.                                               | Use current Skeleton/World behavior.                                          | PLAN-080/084/123 using `StepMetrics`.                                    |
| CT-005 | SimBenchmark articulated PD control                      | Controlled robot tracking exposes whole-step speed/accuracy tradeoffs.                                                                   | Tracking error, constraint error, energy/control work, timing distribution.                      | License-clean DART 6 reconstruction.                                          | PLAN-080/123.                                                            |
| CT-006 | 2026 exoskeleton contact-force criticism                 | Velocity-level/polyhedral/finite-iteration contact can create heel-strike and toe-off transients that are easy to misinterpret as force. | Raw impulse, interval-average wrench, event-integrated impulse, CoP, mode/cone/residual traces.  | Clarify/diagnose current `Contact.force` behavior without ABI/default change. | PLAN-123 contact semantics; method comparison.                           |
| CT-007 | From Compliant to Rigid Contact Simulation               | Exact Coulomb cones and adaptive proximal methods may improve conditioning and remove friction-pyramid anisotropy.                       | Rolling isotropy, high mass ratio, dense stacks, matched residual/time.                          | Evidence only; no new solver family.                                          | PLAN-123 exact-cone GO/NO-GO.                                            |
| CT-008 | Same source                                              | Rigid and compliant contact can share a contact problem while keeping physical compliance distinct from numerical regularization.        | Compliance-limit sweep and parameter-unit/semantics tests.                                       | Document existing behavior only.                                              | PLAN-123 exact-cone problem design.                                      |
| CT-009 | Same source / robotics need                              | Contact-aware inverse dynamics should return feasible torques/contact wrenches and explain infeasibility.                                | Forward/inverse consistency, torque/friction limits, infeasible target.                          | Not applicable beyond existing APIs.                                          | PLAN-123 WS6.                                                            |
| CT-010 | Nimble                                                   | Analytic hard-contact derivatives should agree with finite differences within a fixed mode and outperform finite differencing.           | FD sweep, active-set margins, VJP runtime/memory.                                                | Not applicable as new architecture.                                           | PLAN-110 plus PLAN-123 validity evidence.                                |
| CT-011 | RobotDART                                                | Research workflows need fast reset, concurrency, low overhead, and deterministic synchronous stepping around DART.                       | Restore equivalence, thread/lane isolation, reset cost, allocation.                              | Existing clone/reset/Recording evidence only.                                 | PLAN-030/122/123 one-model/many-state.                                   |
| CT-012 | PEEL                                                     | Long-horizon disassembly needs high-throughput collision checking and reliable execution validation.                                     | Query throughput, determinism, disabled-part updates, continuous validation.                     | Optional benchmark/adapter evidence.                                          | PLAN-120/123 planning companion intake.                                  |
| CT-013 | Fibration Trees                                          | High-DOF multi-robot planning needs thread-safe state-validity and collision queries.                                                    | Independent query contexts, 2–N robot scaling, state mapping correctness.                        | Optional query benchmark.                                                     | PLAN-120/123.                                                            |
| CT-014 | Behavior Policy Learning                                 | Multi-stage manipulation needs reset, controller scheduling, observations, randomization, and sim-to-real evidence.                      | Deterministic episode reset; controller/observation timestamps; randomized replay.               | Companion-only evidence.                                                      | PLAN-103/123 environment companion intake.                               |
| CT-015 | MASS / musculoskeletal DART use                          | Biomechanics needs scalable muscle-actuated models and trustworthy gait/contact outputs.                                                 | Muscle/actuator scaling plus gait impulse/wrench semantics.                                      | Existing SoftBody/contact evidence only; no new subsystem.                    | Future companion after PLAN-123 foundation.                              |
| CT-016 | FARMS                                                    | Reproducible animal/robot experiments need standardized model, sensor, controller, data, and analysis contracts.                         | Schema round-trip, units, deterministic logging/replay.                                          | Companion-only.                                                               | PLAN-103/123 companion intake.                                           |
| CT-017 | Gazebo Plants                                            | Rigid-body engines do not represent flexible plants/codimensional rods naturally.                                                        | Rod bending/twist/contact scene and rigid approximation failure.                                 | Not applicable.                                                               | Existing deformable families; new bounded intake after trust foundation. |
| CT-018 | DART issue #3056 and DART 6 performance campaign         | Large settled worlds must not remain orders of magnitude slower because sleeping/collision paths continue unnecessary work.              | Source-faithful 3k-shape and generated scenes, state/rest/contact hashes, RTF distribution.      | Audit current DART 6 closure and guard it.                                    | DART 7 scalability evidence where equivalent.                            |
| CT-019 | Historical contact-normal / wrench issues (#1425, #1073) | Contact normal, object ordering, force sign, frame, and wrench ownership must be unambiguous downstream.                                 | Pair-order swap, static rest, grasp map, Gazebo sensor integration.                              | Compatibility regression/downstream test.                                     | PLAN-123 observation and wrench contract.                                |
| CT-020 | Current DART paper-parity work                           | Single trajectories and saturated/non-monotonic thresholds can create false claims.                                                      | Deterministic perturbation ensembles, prefix thresholds, geometry/control comparability.         | Reuse PLAN-621/622 evidence patterns.                                         | Apply to all threshold/robustness claims.                                |

## Current-state audit (2026-08-14, WS0)

Reconciled against `main` 20501341226, `release-6.20` 39ccd52068b, open
PRs/issues, and existing plan/dev-task owners. Corrections to the bootstrap
assumptions:

- PLAN-091 (DART 7 Architecture Hardening) is archived and already landed the
  seams this plan extends: `ResolvedSolverConfiguration` (WP-091.11 slice 1;
  requested == resolved today, silent-substitution classification is the open
  follow-up), `StepMetrics`/`WorldStepProfile`, and the cross-family metrics
  corpus at `docs/design/dart7_cross_family_metrics_corpus.json` with
  per-row `resolved_solver_identity`. WS4 continues that follow-up under
  PLAN-123 rather than a new system.
- `dartpy` already exposes `StepMetrics` (energy, momentum, contact count, max
  penetration, iterations, residual) and per-World
  `contact_solver_method` (`SEQUENTIAL_IMPULSE`, `BOXED_LCP`) and
  `rigid_body_solver` (`SEQUENTIAL_IMPULSE`, `IPC`) selection.
  `World::getResolvedConfiguration()` is C++-only as of the audit.
- Existing `python/examples/demos/scenes` seed first-wave fixtures:
  `rigid_spin_roll_coupling` (CT-001), `rigid_restitution_ladder` (CT-003),
  `atlas_simbicon` (CT-005), `rigid_multibody_solver_family`,
  `rigid_contact_inspector`, and the AVBD/IPC scene sets.
- Open PR #3432 (PLAN-104) owns VBD/AVBD paper parity with its own
  fail-closed 88-row contracts; PLAN-123 links, never duplicates.
- Open PR #3377 (`release-6.20` research lane) owns opt-in exact-Coulomb FBF
  friction, rolling/incline/turntable/Painleve scenes, and schema-v3 visual
  evidence on DART 6; CT-001/CT-007 DART 6 lanes reference it instead of
  recreating fixtures.
- Open PR #3431 owns the DART 6 soft-foot perturbation-ensemble methodology
  (CT-020 pattern); PR #3428 and open issue #3056 keep PLAN-621 the CT-018
  owner. PLAN-622 rows stay with their dev task.
- Plan IDs confirmed free at audit time: PLAN-123 on `main`, PLAN-623 on
  `release-6.20`.
- Machine-readable manifest decision: JSON (not YAML) to match every existing
  packet/validator in `scripts/` and add no dependency. The YAML skeleton
  below documents field intent; the checked schema is
  `dart.citation_claim_evidence/v1` JSON.

## Source anchors

Use primary sources and repository evidence during implementation:

- DART JOSS paper: <https://doi.org/10.21105/joss.00500>
- SimBenchmark: <https://leggedrobotics.github.io/SimBenchmark/>
- Predictable behavior during contact simulation:
  <https://doi.org/10.1002/cav.1712>
- From Compliant to Rigid Contact Simulation:
  <https://arxiv.org/abs/2405.17020>
- Nimble: <https://arxiv.org/abs/2103.16021>
- RobotDART: <https://doi.org/10.21105/joss.06771>
- PEEL: <https://arxiv.org/abs/2608.08773>
- Fibration Trees: <https://arxiv.org/abs/2606.12070>
- Gazebo Plants: <https://arxiv.org/abs/2402.02570>
- DART repository issues, PRs, plan docs, tests, and benchmark packets are
  primary evidence for current implementation state.

For sources whose final bibliographic record or license is uncertain, record
that uncertainty and do not vendor assets until provenance is resolved.

## First implementation wave

The first wave is intentionally capped at six fixture families:

1. rolling/friction-direction sweep;
2. dense inelastic/elastic contacts;
3. articulated energy/momentum/control;
4. heel-strike/toe-off transition;
5. high mass-ratio stack/manipulation;
6. reset/concurrency/planning collision query.

Do not add a seventh family until all six have branch-qualified dispositions or
a maintainer explicitly revises the cap.

## Evidence packet skeleton

```yaml
schema: dart.citation_claim_evidence/v1
claim_id: CT-001
source:
  url: ...
  claim: ...
target:
  branch: main
  commit: ...
scene:
  id: ...
  digest: ...
configuration:
  requested: ...
  resolved: ...
  detector: ...
  timestep: ...
  substeps: ...
  tolerances: ...
  fallback_policy: ...
ensemble:
  seeds: [...]
  perturbations: ...
metrics:
  physical: { ... }
  numerical: { ... }
  performance: { ... }
  allocation: { ... }
evidence:
  commands: [...]
  raw_paths: [...]
  visual_paths: [...]
result:
  disposition: unresolved
  claim_boundary: ...
  limitations: [...]
review:
  passes: [...]
```

The schema must fail closed on missing target commit, scene digest, requested
and resolved method, command, disposition, or claim boundary.
