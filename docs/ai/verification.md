---
type: ai-verification-policy
owner: self
---

# AI Verification

AI work is complete only when the requested outcome is mapped to concrete
evidence. Passing a broad verifier is useful only when it covers the actual
requirements.

## Completion Audit

Before finalizing substantial AI-assisted work:

1. Restate the objective as concrete deliverables.
2. Map each explicit request, file, command, gate, and deliverable to evidence.
3. Inspect the actual files, command output, review state, or artifacts.
4. Identify missing, weakly verified, or blocked requirements.
5. If the work used `docs/dev_tasks/<task>/`, verify durable artifacts were
   promoted and the completed task folder was removed.
6. If remaining dev-task work is blocked by a substantial maintainer decision,
   external dependency, or intentionally out-of-session scope boundary, verify
   that the human was asked when needed and that the parked work has a durable
   owner in `docs/plans/`, `docs/design/`, or `docs/onboarding/`.
7. Continue working until all required items are satisfied or a real blocker
   remains.

## Research Paper Implementation Evidence

When the objective is to implement a new solver, algorithm, or paper, do not
redefine completion around an MVP, a paper-inspired subset, or a CPU-only
prototype. The plan, dev task, and completion audit must preserve the full
paper-parity target unless the maintainer explicitly changes scope.

Completion requires direct evidence for all of the following:

- the solver-family intake checklist in `docs/plans/solver-family-intake.md`
  recorded in full, including its solver-contract conformance and
  solver-identity/metrics items — every benchmark or evidence packet must
  machine-record the resolved solver configuration that actually ran;
- every algorithm, feature, parameter, edge case, and limitation named by the
  paper, project page, reference source, videos, and supplemental material;
- CPU and GPU implementations when the method or user request includes both,
  with backend boundaries kept private until a public API decision is made;
- every paper/site/video experiment, comparison, benchmark, and demo ported into
  DART-owned tests, benchmark JSON, and `py-demos` or another durable demo
  surface as appropriate;
- performance optimization until DART beats the reference implementation and
  published paper numbers for every claimed CPU and GPU benchmark case, with the
  same hardware class recorded when the paper gives hardware-specific numbers;
- public API and pipeline cleanup needed for the long-term DART 7/8 architecture,
  preferring clean scalable design over backward-compatible compromises while
  the relevant surface is still experimental; and
- explicit "not complete yet" language for any missing experiment, feature,
  backend, benchmark packet, visual artifact, or comparison.

When an implementation spans multiple sessions, the plan or dev-task resume
surface must record both the newly completed slice and the next missing
paper-parity gap. Do not let checkpoint commits or focused green tests erase
the broader CPU/GPU, demo, benchmark, and performance requirements.

## Gate Selection

| Change type         | Required gates                                                                                                                                                     |
| ------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| AI docs or adapters | `pixi run lint-md`, `pixi run check-lint-md`, `pixi run sync-ai-commands`, `pixi run check-ai-commands`, `pixi run check-docs-policy`, `pixi run check-lint-spell` |
| Docs only           | `pixi run lint-md`, `pixi run check-lint-md`, `pixi run check-docs-policy`, `pixi run check-lint-spell`                                                            |
| C++ code            | `pixi run lint`, `pixi run build`, focused tests or `pixi run test-unit`                                                                                           |
| Python bindings     | `pixi run lint`, `pixi run build`, `pixi run test-py`                                                                                                              |
| IO/model parsing    | `pixi run lint`, focused parser tests, relevant examples if affected                                                                                               |
| CI workflow         | local reproduction when possible, `pixi run check-lint`, relevant build/test gate                                                                                  |
| Release work        | release-management docs, changelog/version checks, target branch gates                                                                                             |

Before any commit, run `pixi run lint` as required by `AGENTS.md`.

## DART 7 Simulation Allocation Evidence

Changes that add, migrate, or materially alter a DART 7 `World::step()` domain,
solver, stage, or hot-path scratch owner must update
`docs/plans/122-simulation-loop-allocation-hardening/coverage-matrix.md` and
add or extend the focused allocation gate in the same change. The final evidence
starts measuring on the first `World::step()` after bake; unmeasured warm-up
simulation steps before the counter starts are steady-state evidence only.

The required gate checks the relevant combination of World base allocator
growth, global heap allocation, and raw malloc-family allocation on supported
hosts. Classic DART 6 paths are excluded unless the change is migrating that
path into the DART 7 `World` pipeline.

## AI Infrastructure Evidence

For substantial changes to AI-facing docs, commands, skills, generated
adapters, planning workflows, or agent rules, run the principle audit in
`docs/ai/principles.md` and record the result as evidence. This file owns gate
selection and evidence mapping; `docs/ai/principles.md` owns the manual audit
questions; `docs/ai/components.md` owns the exact structural checks performed
by `pixi run check-ai-commands`.

When AI workflow changes derive implementation tasks, verify that the owning
plan packet or `docs/dev_tasks/<task>/README.md` records the DART
specification intake from `docs/ai/orchestration.md`: value, scope, non-goals,
assumptions or open decisions, acceptance evidence, gates, and dependencies.
Missing or vague acceptance evidence means the task is not ready for execution.

## Review Safety Evidence

When review feedback comes from an AI bot account:

- Do not reply inline to the bot.
- Verify the claim with code inspection or tests.
- Add or update tests when a false positive should be permanently refuted.
- Do not push, resolve threads, comment, or re-trigger review without explicit
  maintainer/user approval.

The final response should state which of those actions were local-only and which
external mutations, if any, were explicitly approved.
