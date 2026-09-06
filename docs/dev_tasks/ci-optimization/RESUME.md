# Resume: CI Optimization

Handoff for the CI optimization initiative. The goal, evidence, decisions,
packets, maintainer checklist, and gates live in [`README.md`](README.md);
the durable owner is [`../../onboarding/ci-cd.md`](../../onboarding/ci-cd.md).

## Stop Point

> **Current Reality (2026-09-05):** PR-1 branch
> `ci/fix-compiler-cache-and-path-filter` implements the PR-1 packet in the
> README. Verify with `git status --short --branch`, `git log --oneline -5`,
> and `gh pr list --head ci/fix-compiler-cache-and-path-filter` before acting;
> the branch state below is a snapshot, not a checkout command.

## Next Action

1. Push, open PR-1 (milestone `DART 7.0`), add the changelog line with the
   PR number.
2. Collect live evidence: cold run, docs-only push (expect skips), second
   code push (expect cache hits and materially shorter build steps).
3. Refresh the "Expected CI Times" table in `ci-cd.md` from those runs.
4. After merge, start PR-2 from the README packet list.

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

## Evidence Probe

This file was touched by a documentation-only pull request against the
PR-1 branch to demonstrate that the shared code filter reports
`code=false` and every platform and wheel job is skipped; the probe PR is
closed without merging once the run is recorded.

## Context At Risk

- The measured baseline numbers (README) come from PR #3455 and the
  2026-09-03 `main` push; the warm-cache numbers are still to be collected.
- The review findings that shaped PR-1 (daemon spawned before the backend
  variables were exported; MSBuild ignoring launchers; `package.xml` feeding
  the version) are recorded only as code comments and README text.
