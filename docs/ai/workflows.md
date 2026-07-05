# AI Workflow Map

DART 6.20 exposes a release-branch subset of the DART AI workflows across
supported AI tools.

- Claude Code and OpenCode use `/dart-*` commands.
- Codex uses generated `$dart-*` skills from `.codex/skills/`.
- `.claude/commands/` and `.claude/skills/` are the editable source.
- `.opencode/command/` and `.codex/skills/` are generated.

## User-Invoked Workflows

| Capability | Codex | Claude/OpenCode | Required docs and public path | Minimum gate or exception |
| --- | --- | --- | --- | --- |
| `dart-analyze` | `$dart-analyze` | `/dart-analyze` | `docs/ai/principles.md`, `docs/ai/workflows.md`, `docs/ai/verification.md`, `docs/onboarding/ai-tools.md` | Read-only analysis; no local mutation gate beyond inspected evidence |
| `dart-new-task` | `$dart-new-task` | `/dart-new-task` | `docs/onboarding/building.md`, `docs/onboarding/contributing.md`, `docs/onboarding/code-style.md`, `docs/dev_tasks/README.md`, `docs/ai/sessions.md`, `docs/ai/principles.md`, `docs/ai/verification.md` | Use target-specific gates and task-specific gates from `docs/ai/verification.md`; external mutations require explicit approval |
| `dart-new-team-task` | `$dart-new-team-task` | `/dart-new-team-task` | `docs/ai/principles.md`, `docs/ai/north-star.md`, `docs/ai/verification.md`, `docs/dev_tasks/README.md` | Decision interview before large work; per-packet task-specific gates from `docs/ai/verification.md`; `pixi run lint` before commits; external mutations require explicit approval |
| `dart-resume` | `$dart-resume` | `/dart-resume` | `docs/dev_tasks/README.md`, `docs/ai/sessions.md`, `docs/ai/verification.md`, `docs/onboarding/ci-cd.md`, `docs/onboarding/contributing.md` | Start with `git status --short --branch`; external mutations require explicit approval |
| `dart-fix-ci` | `$dart-fix-ci` | `/dart-fix-ci` | `docs/onboarding/ci-cd.md` | Use local reproduction when possible; external mutations require explicit approval |
| `dart-review-pr` | `$dart-review-pr` | `/dart-review-pr` | `docs/onboarding/code-style.md`, `docs/onboarding/ai-tools.md` | Read-only findings unless explicit approval allows mutations |
| `dart-pr` | `$dart-pr` | `/dart-pr` | `docs/onboarding/contributing.md`, `docs/onboarding/ai-tools.md`, `docs/onboarding/changelog.md`, `.github/PULL_REQUEST_TEMPLATE.md` | `pixi run lint`, target-specific gates, milestone and changelog decision; push and PR creation require explicit approval |
| `dart-manage-pr` | `$dart-manage-pr` | `/dart-manage-pr` | `docs/onboarding/contributing.md`, `docs/onboarding/ci-cd.md`, `docs/onboarding/ai-tools.md` | Status-only is read-only; push, PR updates, reruns, comments, and review mutations require explicit approval |
| `dart-docs-update` | `$dart-docs-update` | `/dart-docs-update` | `docs/README.md`, `docs/ai/principles.md`, `docs/ai/verification.md`, `docs/onboarding/ai-tools.md`, `docs/onboarding/changelog.md` | Use Relevant docs/AI checks from `docs/ai/verification.md`; external mutations require explicit approval |
| `dart-changelog` | `$dart-changelog` | `/dart-changelog` | `docs/onboarding/changelog.md`, `docs/onboarding/release-management.md`, `CHANGELOG.md`, and the named PR/issue/release or current diff | Changelog-only edits use the docs checks from `docs/ai/verification.md`; run `pixi run lint` before commits; PR updates or pushes require explicit approval |
| `dart-mechanical-refactor` | `$dart-mechanical-refactor` | `/dart-mechanical-refactor` | `CONTRIBUTING.md`, `docs/onboarding/code-style.md` | `pixi run lint` plus focused build/test proving behavior preservation; external mutations require explicit approval |
| `dart-downstream-fix` | `$dart-downstream-fix` | `/dart-downstream-fix` | `docs/onboarding/contributing.md`, `docs/onboarding/ci-cd.md` | `pixi run lint`, focused build/test, and regression test when applicable; external mutations require explicit approval |
| `dart-release-ci-fix` | `$dart-release-ci-fix` | `/dart-release-ci-fix` | `docs/onboarding/ci-cd.md`, `docs/onboarding/release-management.md` | Use local reproduction when possible; external mutations require explicit approval |
| `dart-backport-pr` | `$dart-backport-pr` | `/dart-backport-pr` | `docs/onboarding/contributing.md`, `docs/onboarding/release-management.md`, `docs/onboarding/changelog.md` | Use release-target focused tests plus `pixi run lint`; call `dart-changelog` for the backport decision; external mutations require explicit approval |
| `dart-branch-cleanup` | `$dart-branch-cleanup` | `/dart-branch-cleanup` | `docs/onboarding/ci-cd.md`, `docs/onboarding/contributing.md` | Read-only analysis unless explicit approval allows branch deletion |

## Domain Skills

| Capability | Codex | Manual public path |
| --- | --- | --- |
| `dart-build` | `$dart-build` | `docs/onboarding/building.md`, `docs/onboarding/build-system.md`, `pixi run build` |
| `dart-ci` | `$dart-ci` | `docs/onboarding/ci-cd.md` |
| `dart-contribute` | `$dart-contribute` | `docs/onboarding/contributing.md`, `CONTRIBUTING.md` |
| `dart-test` | `$dart-test` | `docs/onboarding/testing.md`, `pixi run test`, `pixi run test-py` |
| `dart-python` | `$dart-python` | `docs/onboarding/python-bindings.md`, `python/examples/`, `python/tests/` |
| `dart-io` | `$dart-io` | `docs/onboarding/io-parsing.md`, `dart/utils/`, `tests/integration/test_DartLoader.cpp` |
