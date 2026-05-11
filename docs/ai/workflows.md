# AI Workflow Map

DART exposes the same effective workflow set across supported AI tools:

- Codex uses generated `$dart-*` skills from `.codex/skills/`.
- Claude Code and OpenCode use `/dart-*` commands for workflows.
- Domain skills such as `dart-build` and `dart-test` are lightweight pointers to
  onboarding docs.

Codex is the primary implementation path for the initial AI-native rollout, but
all workflows must remain usable from public docs and `pixi run ...` commands.

## User-Invoked Workflows

| Capability                    | Codex                          | Claude/OpenCode                | Required docs and public path                                                                                                 | Minimum gate or exception                                                                                                                 |
| ----------------------------- | ------------------------------ | ------------------------------ | ----------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------- |
| `dart-new-task`               | `$dart-new-task`               | `/dart-new-task`               | `docs/onboarding/building.md`, `docs/onboarding/contributing.md`, `docs/onboarding/code-style.md`, `docs/dev_tasks/README.md` | No local gate until edits begin; then use task-specific gates; explicit approval before push/PR creation                                  |
| `dart-resume`                 | `$dart-resume`                 | `/dart-resume`                 | active `RESUME.md`, `docs/dev_tasks/README.md`, `docs/onboarding/ci-cd.md`, `docs/onboarding/contributing.md`                 | `git status --short --branch` before edits; explicit approval before push/PR update                                                       |
| `dart-fix-issue`              | `$dart-fix-issue`              | `/dart-fix-issue`              | GitHub issue, `docs/onboarding/contributing.md` dual-PR bugfix rule                                                           | `pixi run lint`, focused build/test; explicit approval before push/PR creation                                                            |
| `dart-fix-ci`                 | `$dart-fix-ci`                 | `/dart-fix-ci`                 | `docs/onboarding/ci-cd.md`, failing CI logs                                                                                   | local reproduction when possible; explicit approval before push, rerun, or CI re-trigger                                                  |
| `dart-review-pr`              | `$dart-review-pr`              | `/dart-review-pr`              | PR diff, `docs/onboarding/code-style.md`, `docs/onboarding/ai-tools.md` review rules                                          | Read-only findings unless explicit approval allows mutations                                                                              |
| `dart-pr`                     | `$dart-pr`                     | `/dart-pr`                     | `.github/PULL_REQUEST_TEMPLATE.md`, `docs/onboarding/contributing.md`, `docs/onboarding/ai-tools.md`                          | `pixi run lint`, target-specific gates, milestone and CHANGELOG decision; explicit approval before push/PR creation                       |
| `dart-manage-pr`              | `$dart-manage-pr`              | `/dart-manage-pr`              | PR checks, `docs/onboarding/contributing.md`, `docs/onboarding/ci-cd.md`, `docs/onboarding/ai-tools.md`                       | Read-only inspection unless explicit approval allows mutations                                                                            |
| `dart-merge-pr`               | `$dart-merge-pr`               | `/dart-merge-pr`               | `docs/onboarding/ci-cd.md`, `docs/onboarding/contributing.md`, PR checks                                                      | Maintainer-only; CI/review green and explicit merge approval                                                                              |
| `dart-docs-update`            | `$dart-docs-update`            | `/dart-docs-update`            | `docs/README.md`, `docs/onboarding/ai-tools.md`, relevant docs                                                                | `pixi run lint-md`, `pixi run check-lint-md`, `pixi run check-docs-policy`, `pixi run check-lint-spell`; explicit approval before push/PR |
| `dart-improve-docs`           | `$dart-improve-docs`           | `/dart-improve-docs`           | `docs/AGENTS.md`, `docs/onboarding/ai-tools.md`, relevant docs                                                                | `pixi run lint-md`, `pixi run check-lint-md`, `pixi run check-docs-policy`, `pixi run check-lint-spell`                                   |
| `dart-audit-agent-compliance` | `$dart-audit-agent-compliance` | `/dart-audit-agent-compliance` | `docs/onboarding/ai-tools.md`, `docs/onboarding/contributing.md`                                                              | Relevant docs/AI checks for touched files                                                                                                 |
| `dart-mechanical-refactor`    | `$dart-mechanical-refactor`    | `/dart-mechanical-refactor`    | `CONTRIBUTING.md`, `docs/onboarding/code-style.md`, affected module docs                                                      | `pixi run lint`, focused tests proving behavior preservation; explicit approval before push/PR creation                                   |
| `dart-branch-cleanup`         | `$dart-branch-cleanup`         | `/dart-branch-cleanup`         | branch list, `docs/onboarding/ci-cd.md`, `docs/onboarding/contributing.md`                                                    | Read-only analysis unless explicit approval allows branch deletion                                                                        |
| `dart-triage-issue`           | `$dart-triage-issue`           | `/dart-triage-issue`           | GitHub issue, `docs/onboarding/contributing.md`                                                                               | No local gate for read-only triage; explicit approval before labels/comments                                                              |
| `dart-close-issue`            | `$dart-close-issue`            | `/dart-close-issue`            | issue context, `docs/onboarding/contributing.md`, closing evidence                                                            | No local gate for draft text; explicit approval before posting/closing                                                                    |
| `dart-downstream-fix`         | `$dart-downstream-fix`         | `/dart-downstream-fix`         | downstream repro, `docs/onboarding/contributing.md` dual-PR bugfix rule, `docs/onboarding/ci-cd.md`                           | `pixi run lint`, regression test, focused build/test; explicit approval before push/PR creation                                           |
| `dart-backport-pr`            | `$dart-backport-pr`            | `/dart-backport-pr`            | `docs/onboarding/contributing.md`, `docs/onboarding/release-management.md`                                                    | `pixi run lint`, release-target focused tests; explicit approval before push/PR creation                                                  |
| `dart-release-ci-fix`         | `$dart-release-ci-fix`         | `/dart-release-ci-fix`         | `docs/onboarding/ci-cd.md`, `docs/onboarding/release-management.md`, release CI logs                                          | local reproduction when possible; explicit approval before push/PR update                                                                 |
| `dart-release-merge-main`     | `$dart-release-merge-main`     | `/dart-release-merge-main`     | `docs/onboarding/release-management.md`, `docs/onboarding/ci-cd.md`                                                           | Maintainer-only; explicit approval before push/PR creation plus release gates                                                             |
| `dart-release-packaging`      | `$dart-release-packaging`      | `/dart-release-packaging`      | `docs/onboarding/release-management.md`, `docs/onboarding/contributing.md`, changelog/version files                           | release verification gates and explicit maintainer/user approval before GitHub mutations                                                  |

## Domain Skills

| Capability        | Codex              | Manual public path                                                     |
| ----------------- | ------------------ | ---------------------------------------------------------------------- |
| `dart-build`      | `$dart-build`      | `docs/onboarding/building.md`, `pixi run build`                        |
| `dart-ci`         | `$dart-ci`         | `docs/onboarding/ci-cd.md`                                             |
| `dart-contribute` | `$dart-contribute` | `docs/onboarding/contributing.md`, `CONTRIBUTING.md`                   |
| `dart-io`         | `$dart-io`         | `docs/onboarding/io-parsing.md`                                        |
| `dart-python`     | `$dart-python`     | `docs/onboarding/python-bindings.md`                                   |
| `dart-test`       | `$dart-test`       | `docs/onboarding/testing.md`, `pixi run test-unit`, `pixi run test-py` |

## Expected Routing Scenarios

| Prompt shape             | Expected workflow  | Required docs                                      | Minimum gates                                                                                                                             |
| ------------------------ | ------------------ | -------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------- |
| "Fix issue #123"         | `dart-fix-issue`   | `contributing.md`, task-specific docs              | `pixi run lint`, focused build/test                                                                                                       |
| "CI is failing on my PR" | `dart-fix-ci`      | `ci-cd.md`, failing logs                           | reproduction command plus fixed check                                                                                                     |
| "Update docs for X"      | `dart-docs-update` | `docs/README.md`, `ai-tools.md`, relevant docs     | `pixi run lint-md`, `pixi run check-lint-md`, `pixi run check-docs-policy`, `pixi run check-lint-spell`; explicit approval before push/PR |
| "Review PR #123"         | `dart-review-pr`   | `code-style.md`, `ai-tools.md`, PR diff            | read-only findings unless explicit approval allows mutations                                                                              |
| "Create a PR"            | `dart-pr`          | PR template, `contributing.md`, `ai-tools.md`      | `pixi run lint`, target-specific gates, milestone/CHANGELOG decision, explicit approval before push/PR creation                           |
| "Continue the task"      | `dart-resume`      | active `RESUME.md`, dev task, CI/contributing docs | `git status --short --branch`; explicit approval before push/PR update                                                                    |
