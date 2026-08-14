# DART 6 Citation-Driven Contact Trust — Dev Task

## Current status

- [x] Phase 0: Reconcile with current `release-6.20`, PLAN-621/622, open PRs,
      issues, and completed/retired task state (2026-08-14: PLAN-623 free and
      registered on the dashboard; PLAN-621 owns CT-018 with #3056/#3428;
      PLAN-622/PR #3431 own CT-020; PR #3377 owns exact-Coulomb fixtures;
      `dart.dynamics.ContactInverseDynamics` already exists on this branch).
- [x] Phase 1: Adopt the stable claim/evidence schema without DART 7 API
      backports (branch-local `claims-manifest.json` + fail-closed
      `pixi run check-citation-evidence` in `check-lint`, permanent negative
      control, pytest coverage, first CT-001 detector-sweep packet).
- [ ] Phase 2: Audit and guard existing performance/deformable evidence.
- [ ] Phase 3: Reproduce remaining first-wave DART 6 contact claims.
- [ ] Phase 4: Land compatibility-safe fixes/diagnostics for confirmed defects.
- [ ] Phase 5: Promote durable results and delete this task folder.

## Goal

Produce a branch-qualified, reproducible answer for material claims about DART
6 contact, force/wrench interpretation, stability, determinism, and performance
without changing the DART 6.20 compatibility contract.

## Required reading

- branch `AGENTS.md`
- `docs/ai/principles.md`
- `docs/ai/verification.md`
- `docs/plans/dashboard.md`
- `docs/design/dart6_collision_backends.md`
- `docs/design/dart6_citation_driven_contact_trust.md`
- `docs/dev_tasks/dart6_performance_generalization/` if present
- `docs/dev_tasks/dart6_deformable_body_performance/` if present
- DART 7 PLAN-123/design/corpus as reference evidence only
- this folder, especially `RESUME.md`

## Specification intake

- **Value:** protect LTS users/downstreams, close historical claims honestly,
  and feed branch-qualified evidence into DART 7 decisions.
- **Scope:** tests, benchmarks, demos, evidence packets, analysis helpers,
  internal/opt-in diagnostics, and narrowly proven fixes.
- **Assumptions:** C++17/pybind11/OSG, ABI, packages, default FCL behavior, and
  gz compatibility remain fixed.
- **Traceability:** PLAN-123 corpus IDs, PLAN-621/622, DART issues/PRs, primary
  sources, and current branch tests/benchmarks.
- **Acceptance evidence:** baseline/current packets, negative controls,
  deterministic/ensemble oracles, downstream tests, and review records.

## Work rules

- Work from latest `origin/release-6.20` on a non-tracking topic branch.
- Keep a separate DART 7 worktree/branch for shared bug assessment.
- Do not copy DART 7 code or APIs mechanically.
- Reuse PLAN-621/622 scenes/evidence; reference rather than duplicate.
- Preserve unaffected default outputs and explicitly re-baseline affected rows.
- Unsupported metrics are absent/typed as unsupported, never zero.
- Do not start a new solver family or public contact architecture.
- Do not push or mutate GitHub state without explicit approval.

## Deliverables

1. DART 6 entries for the stable claim/evidence manifest.
2. Guarded current evidence for completed performance/deformable rows.
3. First-wave contact fixtures not already owned elsewhere.
4. Contact normal/order/frame and force/wrench interpretation regressions.
5. Compatibility-safe fixes with baseline/current evidence.
6. Downstream gz verification for affected paths.
7. Durable documentation and task cleanup.

## Non-goals

- Exact-cone/NCP, IPC, AVBD/VBD, differentiability, batch World, sensors,
  biomechanics, rods, or shells.
- Default detector/solver changes.
- Dependency or public-layout cleanup.
- Public force-semantics redesign.
- Universal cross-version claims.

## Gates

Use current branch task names. Typical required gates:

- `pixi run lint`
- `pixi run check-lint`
- focused configure/build/tests
- `pixi run test-all`
- detector/solver/demos benchmark commands with raw evidence
- `pixi run -e gazebo test-gz` for collision, constraint, `World::step`,
  parser, package, or downstream-sensitive changes
- ABI/header/component checks when relevant
- visual capture and semantic review when the claim is visible

## Immediate next steps

1. Audit current PLAN-621/622 and open PR state.
2. Map PLAN-123 corpus rows to existing DART 6 tests/evidence.
3. Select the smallest missing row that can provide a negative control.
4. Add a branch-local manifest packet and validator/adaptor without public API
   change.
5. Update `RESUME.md`, `decisions.md`, and `verification.md`.
