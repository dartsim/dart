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
5. Verify review evidence: at least two clean independent or role-separated
   review passes on the current post-fix state, with substantive findings
   investigated rather than blindly accepted.
6. For model/scene, behavior-bearing physics/simulation, or GUI work, verify
   `dart-verify-sim` paired a text correctness oracle with assessed claim-tied
   visual/debug evidence, or recorded a justified unavailable exception.
7. If the work used `docs/dev_tasks/<task>/`, verify durable artifacts were
   promoted and the completed task folder was removed.
8. If remaining dev-task work is blocked by a substantial maintainer decision,
   external dependency, or intentionally out-of-session scope boundary, verify
   that the human was asked when needed and that the parked work has a durable
   owner in `docs/plans/`, `docs/design/`, or `docs/onboarding/`.
9. Continue working until all required items are satisfied or a real blocker
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

| Change type         | Required gates                                                                                                                                               |
| ------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| AI docs or adapters | `pixi run lint-md`, `pixi run check-lint-md`, `pixi run sync-ai-commands`, `pixi run check-ai-infra`, `pixi run test-ai-infra`, `pixi run check-lint-spell`  |
| Docs only           | `pixi run lint-md`, `pixi run check-lint-md`, `pixi run check-docs-policy`, `pixi run check-lint-spell`                                                      |
| C++ code            | `pixi run lint`, `pixi run build`, focused tests or `pixi run test-unit`                                                                                     |
| Python bindings     | `pixi run lint`, `pixi run build`, `pixi run test-py`                                                                                                        |
| IO/model parsing    | `pixi run lint`, focused parser tests, relevant examples if affected                                                                                         |
| Simulation behavior | `pixi run lint`, focused simulation tests, and `dart-verify-sim`: text correctness evidence plus assessed headless/debug-layer corroboration when applicable |
| CI workflow         | local reproduction when possible, `pixi run check-lint`, relevant build/test gate                                                                            |
| Release work        | release-management docs, changelog/version checks, target branch gates                                                                                       |

Before any commit, run `pixi run lint` as required by `AGENTS.md`.

When a change touches lint tooling, formatting config, or many files, verify with
the full `pixi run check-lint` aggregate, not only the sub-check you edited. CI
runs the whole aggregate in `ci_lint.yml`, so a drift in an unrelated sub-check
(for example codespell or clang-format) still fails the "Check Lint" step even
when the sub-check you ran is clean.

## GUI And Demo Evidence

Use the `dart-verify-sim` workflow and durable guide at
`docs/onboarding/agent-sim-verification.md`. Establish correctness with metrics,
scene/trajectory/contact comparison, or focused behavioral tests; then use
view assessment, `agent-capture`, and only the engine debug layers needed by
the claim. Record the runnable command, view report, expected observation,
`image-verdict` or inspected artifact, limitations, and what the image does not
prove. A self-contained GUI or demos-app example remains the preferred durable
user surface. If rendering is unavailable or genuinely irrelevant, record why
and name replacement evidence.

Linux CI runs an explicit settled-contact `agent-capture` under Xvfb with
contacts, collision bounds, and labels, then requires `image-verdict` to accept
the emitted frame. The same blocking step runs a focused A/B regression under
Xvfb: it holds the world and camera fixed, compares a plain capture with the
combined debug capture, then proves contacts, collision bounds, and labels each
change pixels independently. The Python suite also asserts that a same-renderer
debug overlay clears cleanly. Together these gates exercise view assessment,
the Filament renderer, every claim-relevant debug layer, artifact writing, and
image validation without relying on an optional display test.

## Review Evidence

Review findings are extra input, not orders. For each substantive review
finding, verify the claim with code inspection, tests, docs, benchmarks, or
visual artifacts before changing behavior. Record whether the finding was fixed,
deferred, or rejected with evidence. Completion requires two clean review
passes on the current post-fix state; a pass before later fixes does not count
as final evidence.

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
by `pixi run check-ai-commands` and `pixi run check-ai-infra`.

Use `pixi run ai-doctor` to diagnose discovery or setup failures without
mutating the checkout. Use `pixi run check-agent-hook` for frequent hook-sized
feedback only; it is intentionally not completion evidence. The focused
`test-ai-infra` suite and aggregate `check-ai-infra` gate must pass in addition
to generated-adapter sync. Run `pixi run exercise-agent-scenarios` directly
when changing routing fixtures or branch profiles so failures identify the
specific scenario.

When AI workflow changes derive implementation tasks, verify that the owning
plan packet or `docs/dev_tasks/<task>/README.md` records the DART
specification intake from `docs/ai/orchestration.md`: value, scope, non-goals,
assumptions or open decisions, acceptance evidence, gates, and dependencies.
Missing or vague acceptance evidence means the task is not ready for execution.

## Review Safety Evidence

When review feedback comes from an AI bot account, never reply inline, verify
each claim with code inspection or tests, and treat any push, comment, thread
resolution, or re-trigger as an external mutation needing explicit
maintainer/user approval; add or update a test when a false positive should be
permanently refuted. See `docs/onboarding/ai-tools.md` § "Handling Automated
Reviews" for the full loop. The final response should state which actions were
local-only and which external mutations, if any, were explicitly approved.
