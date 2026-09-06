# Resume: CI Optimization

Handoff for the CI optimization initiative. The goal, evidence, decisions,
packets, maintainer checklist, and gates live in [`README.md`](README.md);
the durable owner is [`../../onboarding/ci-cd.md`](../../onboarding/ci-cd.md).

## Stop Point

> **Current Reality (2026-09-06):** PR-1 is open as
> [#3485](https://github.com/dartsim/dart/pull/3485) from
> `ci/fix-compiler-cache-and-path-filter` (milestone DART 7.0), merged with
> the latest `main` (no rebase), with two Codex review rounds addressed
> (root README, LICENSE, and test-consumed READMEs kept in the code filter;
> this handoff refreshed). The docs-only probe PR #3486 is closed; its runs
> logged `Filter code = false` on every workflow with all platform and wheel
> jobs skipped. Verify with `git status --short --branch`,
> `gh pr view 3485 --json mergeStateStatus,headRefOid`, and
> `gh pr checks 3485` before acting; the branch state here is a snapshot.

## Next Action

1. Read the required checks on the current PR head; any failure in
   "Configure environment for compiler cache" is a cache-setup defect to fix,
   not a guard to weaken.
2. Record the warm-cache evidence from the head run's sccache-action post-step
   stats (hit/miss lines per job) and the build-step durations in the
   "Expected CI Times" table of `docs/onboarding/ci-cd.md`; commit and push
   that refresh (merge `main` first).
3. Merge #3485 once every required check is green, then start PR-2 from the
   README packet list, sized against the recorded warm numbers.

## Session Constraints

- Model/effort: Claude Code on Fable 5.1 at the session's `xhigh` effort with
  workflow orchestration; research and review were delegated to parallel
  read-only agents, implementation stayed with one writer.
- Authorization: the maintainer authorized branch creation, pushes, PR
  creation, CI re-triggers, and merges for this initiative on 2026-09-05.
- Required check names must not change (list in the README).
- Guard scripts constrain workflow edits: `scripts/check_diff_workflow.py`,
  `scripts/check_phase5_cuda_workflow.py`,
  `scripts/ai_infrastructure.py::check_ci_wiring` (ci_windows hook-smoke
  markers are file-wide; the ci_ubuntu visual-smoke section is bounded by the
  `filament-gui-smoke:` job key), and `tests/test_ci_code_filter.py`.
- Merge the latest `main` before every push; never rebase the published
  branch.

## Context At Risk

- The measured baseline numbers (README) come from PR #3455 and the
  2026-09-03 `main` push; the warm-cache numbers are still to be collected.
- The review findings that shaped PR-1 (daemon spawned before the backend
  variables were exported; MSBuild ignoring launchers; `package.xml` feeding
  the version) are recorded only as code comments and README text.
