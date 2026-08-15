# Citation-Driven Simulation Trust — Dev Task

## Current status

- [x] Phase 0: Reconcile these bootstrap docs with current `main` and
      `release-6.20` (2026-08-14 audit recorded in the corpus sidecar;
      PLAN-123/PLAN-623 IDs confirmed free; PLAN-091 heritage and open-PR
      owners mapped).
- [x] Phase 1: Land the checked citation-claim/evidence manifest and first
      negative-control fixtures (`claims-manifest.json`,
      `scripts/check_citation_evidence.py`, `pixi run check-citation-evidence`
      in `check-lint`, permanent negative control, CT-001 packet).
- [ ] Phase 2: Reproduce and classify the six-family first-wave corpus.
      Landed on `main`: CT-001 rolling/friction-direction (`reproduced`),
      CT-002 dense inelastic contact (`reproduced`), CT-003 dense elastic
      contact (`unresolved` -- no energy injection observed), CT-004 articulated
      energy drift versus timestep (`unresolved`: both integration families
      converge, but the cited matched-cost comparison is not measured), and
      CT-005 PD tracking (`unresolved`: error is controller-limited and step
      cost is not measured), and CT-007 high mass-ratio stack (`unresolved`
      for the comparative claim, but it found that the default contact solver
      fails completely at mass ratios 100 and 1000 where boxed LCP holds). Landed on
      `release-6.20`: CT-001 detector sweep (`reproduced` on fcl/dart/ode;
      bullet excluded for lacking the pyramid signature). Open: articulated
      energy/momentum/control, heel-strike/toe-off, high mass-ratio, and
      reset/concurrency families.
- [ ] Phase 3: Land contact identity and impulse/wrench semantics.
- [ ] Phase 4: Extend resolved-method and cross-family solve diagnostics.
- [ ] Phase 5: Record exact-cone GO/NO-GO; implement only after GO.
- [ ] Phase 6: Implement contact-aware inverse dynamics after a stable shared
      contact problem.
- [ ] Phase 7: Derive bounded adoption/domain follow-ups and close this task.

## Goal

Implement PLAN-123 so DART can convert material external claims into
branch-qualified, reproducible evidence and compare solver families without
silent substitution or ambiguous contact-force semantics.

## Specification intake

- **Value:** scientific credibility, regression prevention, solver selection,
  reusable research infrastructure, and evidence-backed prioritization.
- **Scope:** evidence schemas/harnesses, tests/benchmarks/demos, contact
  observations and result semantics, solver-resolution reports, branch
  coordination, and conditional solver/query work.
- **Assumptions:** existing DART 7 model/contact/metrics/evidence surfaces are
  extended rather than replaced; exact-cone work is gated by evidence.
- **Traceability:** PLAN-123, its corpus sidecar, the durable design doc, the
  DART citation audit, existing solver plans, DART issues/PRs, and primary
  papers.
- **Acceptance evidence:** source-bound packets, negative controls, deterministic
  or ensemble oracles, matched comparisons, visual review where applicable,
  branch gates, and independent post-fix reviews.

## Required reading

- `AGENTS.md`
- `docs/ai/principles.md`
- `docs/ai/north-star.md`
- `docs/ai/verification.md`
- `docs/design/contact_trust_and_observability.md`
- `docs/design/simulation_solver_architecture.md`
- `docs/plans/123-citation-driven-simulation-trust.md`
- `docs/plans/123-citation-driven-simulation-trust/citation-claim-corpus.md`
- `docs/plans/solver-family-intake.md`
- current owner plans named by PLAN-123
- this folder, especially `RESUME.md`

## Branch/worktree discipline

Use separate current worktrees and topic branches for:

- DART 7: latest `origin/main`;
- DART 6: latest `origin/release-6.20`, governed by its branch-local design and
  `docs/dev_tasks/dart6_citation_contact_trust/`.

Do not mix branch changes in one PR, rebase published PR branches, overwrite
uncommitted work, or mechanically port DART 7 architecture to DART 6. Shared
bugs get independently adapted fixes and evidence.

Before implementation, inspect open PRs, issues, active dev tasks, dashboards,
and current code. Mark already-landed work in the corpus; do not recreate it.

## Deliverables

1. Checked claim/evidence manifest and validator.
2. Six capped first-wave fixture families with branch-qualified packets.
3. Canonical contact identity/ordering and tested normal/tangent semantics.
4. Explicit impulse/average-wrench/continuous-force/filtered/event semantics.
5. Requested/resolved method plus comparable solve/fallback diagnostics.
6. Cross-family matched speed-accuracy dashboard or packets.
7. Exact-cone GO/NO-GO record; implementation only after GO.
8. Contact-aware inverse-dynamics query after a stable shared problem.
9. Bounded follow-up plans/companions for planning, environments/sensors,
   differentiability, biomechanics, and rods/shells where evidence supports
   them.
10. Durable docs/changelog/user examples and task-folder cleanup at completion.

## Non-goals

- One universal solver ranking.
- Reimplementing every citing paper.
- New DART 6 public solver/model architecture.
- Silent smoothing of raw contact outputs.
- Public solver registry, ECS/device storage, reference-project, or tensor
  framework types.
- Open-ended fixture or paper expansion.

## Work-package rules

- Each PR closes a bounded set of corpus rows or one shared contract.
- A feature PR includes the fixture/evidence that justified it.
- No exact-cone public option before the common harness and GO record.
- No companion or domain extension before a named owner and stopping condition.
- Changed physical outcomes are re-baselines, not performance wins.
- Every threshold/robustness claim uses an appropriate perturbation ensemble.
- Strict modes fail closed; continuation/fallback modes are explicit and typed.

## Acceptance evidence

- At least one pre-fix or negative-control failure for each behavioral slice.
- Machine-readable packets bound to commit, scene digest, command, and resolved
  configuration.
- Text/numeric physics oracle; semantic visual inspection when visible.
- Stable repeated/ensemble results and exact claim boundaries.
- Same-host interleaved or justified performance methodology.
- Post-bake allocation gates for promoted repeated paths.
- Two clean independent or role-separated reviews on the current post-fix
  state.
- Changelog decision and branch-required tests.

## Gates

Always use current branch-owned tasks; do not invent aliases.

Typical DART 7 gates:

- `pixi run lint`
- `pixi run check-docs-policy`
- `pixi run build`
- focused C++/Python/simulation tests
- benchmark/evidence validators
- `pixi run check-api-boundaries`
- `pixi run test-all`
- `pixi run -e cuda test-all` on a visible CUDA host when affected

DART 6 gates are owned by its task and include `pixi run -e gazebo test-gz`
for downstream-sensitive changes.

## Open decisions

- Exact-cone CPU algorithm and problem boundary: decide only after WS1–WS4
  evidence.
- Whether a second cone algorithm is warranted: require a distinct validation
  purpose.
- Public contact result surface: promote only fields with multiple stable
  consumers.
- Planning/environment/biomechanics/rod ownership: companion or existing plan,
  not assumed core.

## Immediate next steps

1. Read `RESUME.md` and verify both branch tips/worktrees.
2. Bring the CT-007 sequential-impulse failure to the maintainer: it is a
   default-path defect, the strongest WS5 GO input so far, and confirming its
   mechanism needs the per-solve residual WS4 has not exposed.
3. Continue Phase 2 with heel-strike/toe-off (CT-006), which depends on the
   WS3 contact semantics, and reset/concurrency queries (CT-011..013).
4. Start WS4's first slice in parallel where it unblocks packets: expose
   `ResolvedSolverConfiguration` to Python and a comparable per-solve
   residual, both currently typed unsupported in every packet.
5. Reuse PR #3377 fixtures for the DART 6 CT-007 lane rather than adding
   new scenes; keep PLAN-621/622 rows referenced, not copied.
