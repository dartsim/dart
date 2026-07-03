---
type: ai-workflow-map
owner: self
---

# AI Workflow Map

DART exposes the same effective workflow set across supported AI tools:

- Codex uses generated `$dart-*` skills from `.codex/skills/`.
- Claude Code and OpenCode use `/dart-*` commands for workflows.
- Domain skills such as `dart-build` and `dart-test` are lightweight pointers to
  developer docs.
- `docs/ai/capabilities.json` owns machine-readable capability status,
  category, and gate profile.

Codex is the primary implementation path for the initial AI-native rollout, but
all workflows must remain usable from public docs and `pixi run ...` commands.

## User-Invoked Workflows

| Capability | Codex | Claude/OpenCode | Required docs and public path | Minimum gate or exception |
| --- | --- | --- | --- | --- |
| `dart-next` | `$dart-next` | `/dart-next` | `docs/ai/principles.md`, `docs/ai/north-star.md`, `docs/ai/workflows.md`, `docs/ai/verification.md`, `docs/plans/README.md`, `docs/plans/dashboard.md`, `docs/plans/north-star-roadmap.md`, `docs/dev_tasks/README.md`, `docs/onboarding/contributing.md`, `docs/onboarding/ci-cd.md`, `docs/onboarding/ai-tools.md` | No local gate for `mode=select`; after routing, use task-specific gates from `docs/ai/verification.md`; run `pixi run lint` before commits and get explicit approval before GitHub mutations |
| `dart-analyze` | `$dart-analyze` | `/dart-analyze` | `docs/ai/principles.md`, `docs/ai/workflows.md`, `docs/ai/verification.md`, `docs/onboarding/ai-tools.md`, task-specific docs and code/tests named by the question | Read-only analysis; no local gate beyond inspected evidence and no GitHub, CI, branch, or review-thread mutations |
| `dart-new-task` | `$dart-new-task` | `/dart-new-task` | `docs/onboarding/building.md`, `docs/onboarding/contributing.md`, `docs/onboarding/code-style.md`, `docs/dev_tasks/README.md`, `docs/ai/sessions.md`, `docs/ai/principles.md`, `docs/ai/verification.md` | No local gate until edits begin; then use task-specific gates; explicit approval before push/PR creation |
| `dart-resume` | `$dart-resume` | `/dart-resume` | active `RESUME.md`, `docs/dev_tasks/README.md`, `docs/ai/sessions.md`, `docs/ai/verification.md`, `docs/onboarding/ci-cd.md`, `docs/onboarding/contributing.md` | `git status --short --branch`; promote durable dashboards/matrices out of `docs/dev_tasks/` before task cleanup; explicit approval before push/PR update |
| `dart-execute-packet` | `$dart-execute-packet` | `/dart-execute-packet` | `docs/ai/orchestration.md`, `docs/ai/principles.md`, `docs/ai/verification.md`, `docs/plans/dashboard.md`, the owning numbered plan file and the packet's named owner docs | Packet-defined gates plus task-specific gates from `docs/ai/verification.md`; `pixi run lint` before commits; explicit approval before push/PR creation |
| `dart-fix-issue` | `$dart-fix-issue` | `/dart-fix-issue` | GitHub issue, `docs/onboarding/contributing.md` dual-PR bugfix rule, `docs/onboarding/changelog.md` | `pixi run lint`, focused build/test; explicit approval before push/PR creation |
| `dart-fix-ci` | `$dart-fix-ci` | `/dart-fix-ci` | `docs/onboarding/ci-cd.md`, `docs/onboarding/release-management.md`, failing CI logs | local reproduction when possible; explicit approval before push, rerun, or CI re-trigger |
| `dart-review-pr` | `$dart-review-pr` | `/dart-review-pr` | PR diff, `docs/onboarding/code-style.md`, `docs/onboarding/ai-tools.md` review rules | Read-only findings unless explicit approval allows mutations |
| `dart-pr` | `$dart-pr` | `/dart-pr` | `.github/PULL_REQUEST_TEMPLATE.md`, `docs/onboarding/contributing.md`, `docs/onboarding/ai-tools.md`, `docs/onboarding/changelog.md` | `pixi run lint`, target-specific gates, milestone and CHANGELOG decision; before/after headless visual evidence when rendering/model output changes; explicit approval before push/PR creation |
| `dart-manage-pr` | `$dart-manage-pr` | `/dart-manage-pr` | PR checks, `docs/onboarding/contributing.md`, `docs/onboarding/ci-cd.md`, `docs/onboarding/testing.md`, `docs/onboarding/ai-tools.md` | Status-only is read-only; an imperative `manage <PR>` request is explicit approval for routine PR maintenance, while `mode=merge` and other destructive actions still require separate explicit approval |
| `dart-deps` | `$dart-deps` | `/dart-deps` | open bot PRs, `docs/onboarding/ci-cd.md`, `docs/onboarding/contributing.md` | Read-only triage by default; merging a dependency or bot PR needs CI/review green and an explicit approval gate |
| `dart-docs-update` | `$dart-docs-update` | `/dart-docs-update` | `docs/README.md`, `docs/ai/principles.md`, `docs/ai/verification.md`, `docs/onboarding/ai-tools.md`, `docs/onboarding/changelog.md`, relevant docs | Relevant docs/AI checks from `docs/ai/verification.md`; explicit approval before push/PR |
| `dart-plan-update` | `$dart-plan-update` | `/dart-plan-update` | `docs/ai/principles.md`, `docs/ai/north-star.md`, `docs/plans/README.md`, `docs/plans/dashboard.md`, `docs/plans/north-star-roadmap.md`, `docs/ai/verification.md` | Relevant docs/AI checks from `docs/ai/verification.md`; use docs-only or AI docs/adapters gates based on touched files |
| `dart-retrospect` | `$dart-retrospect` | `/dart-retrospect` | `docs/AGENTS.md`, `docs/ai/principles.md`, `docs/ai/components.md`, `docs/ai/verification.md`, `docs/onboarding/ai-tools.md`, relevant docs | Relevant docs/AI checks from `docs/ai/verification.md` |
| `dart-audit-agent-compliance` | `$dart-audit-agent-compliance` | `/dart-audit-agent-compliance` | `docs/ai/principles.md`, `docs/ai/verification.md`, `docs/ai/components.md`, `docs/onboarding/ai-tools.md`, `docs/onboarding/contributing.md` | Relevant docs/AI checks plus the principle audit for touched AI-infra files |
| `dart-mechanical-refactor` | `$dart-mechanical-refactor` | `/dart-mechanical-refactor` | `CONTRIBUTING.md`, `docs/onboarding/code-style.md`, affected module docs | `pixi run lint`, focused tests proving behavior preservation; explicit approval before push/PR creation |
| `dart-branch-cleanup` | `$dart-branch-cleanup` | `/dart-branch-cleanup` | branch list, `docs/onboarding/ci-cd.md`, `docs/onboarding/contributing.md` | Read-only analysis unless explicit approval allows branch deletion |
| `dart-triage-issue` | `$dart-triage-issue` | `/dart-triage-issue` | GitHub issue, `docs/onboarding/contributing.md` | No local gate for read-only triage; explicit approval before labels/comments |
| `dart-close-issue` | `$dart-close-issue` | `/dart-close-issue` | issue context, `docs/onboarding/contributing.md`, closing evidence | No local gate for draft text; explicit approval before posting/closing |
| `dart-downstream-fix` | `$dart-downstream-fix` | `/dart-downstream-fix` | downstream repro, `docs/onboarding/contributing.md` dual-PR bugfix rule, `docs/onboarding/ci-cd.md` | `pixi run lint`, regression test, focused build/test; explicit approval before push/PR creation |
| `dart-benchmark-packet` | `$dart-benchmark-packet` | `/dart-benchmark-packet` | `docs/onboarding/profiling.md`, the owning plan file named by the packet | Packet checker plus task-specific gates from `docs/ai/verification.md`; run `pixi run check-*-packets` and the packet writer; explicit approval before push/PR |
| `dart-backport-pr` | `$dart-backport-pr` | `/dart-backport-pr` | `docs/onboarding/contributing.md`, `docs/onboarding/release-management.md` | `pixi run lint`, release-target focused tests; explicit approval before push/PR creation |
| `dart-release-merge-main` | `$dart-release-merge-main` | `/dart-release-merge-main` | `docs/onboarding/release-management.md`, `docs/onboarding/ci-cd.md` | Maintainer-only; explicit approval before push/PR creation plus release gates |
| `dart-release-packaging` | `$dart-release-packaging` | `/dart-release-packaging` | `docs/onboarding/release-management.md`, `docs/onboarding/contributing.md`, `docs/onboarding/changelog.md`, changelog/version files | release verification gates and explicit maintainer/user approval before GitHub mutations |

## Domain Skills

| Capability | Codex | Manual public path |
| --- | --- | --- |
| `dart-architecture` | `$dart-architecture` | `docs/readthedocs/architecture.md`, `docs/design/simulation_solver_architecture.md`, `docs/design/dart7_architecture_assessment.md` |
| `dart-build` | `$dart-build` | `docs/onboarding/building.md`, `pixi run build` |
| `dart-ci` | `$dart-ci` | `docs/onboarding/ci-cd.md` |
| `dart-contribute` | `$dart-contribute` | `docs/onboarding/contributing.md`, `CONTRIBUTING.md` |
| `dart-io` | `$dart-io` | `docs/onboarding/io-parsing.md` |
| `dart-python` | `$dart-python` | `docs/onboarding/python-bindings.md` |
| `dart-references` | `$dart-references` | `docs/readthedocs/papers.md` |
| `dart-test` | `$dart-test` | `docs/onboarding/testing.md`, `pixi run test-unit`, `pixi run test-py` |

## Expected Routing Scenarios

| Prompt shape | Expected workflow | Required docs | Minimum gates |
| --- | --- | --- | --- |
| "Pick the next task" | `dart-next` | `docs/ai/north-star.md`, `docs/plans/dashboard.md`, `docs/dev_tasks/README.md`, `docs/ai/verification.md`, routed workflow docs | No local gate for selection-only; after routing, use task-specific gates and explicit approval for GitHub mutations |
| "Analyze why X happens" | `dart-analyze` | `docs/ai/principles.md`, `docs/ai/workflows.md`, `docs/ai/verification.md`, task-specific code/docs/tests | Read-only evidence synthesis; no local edits or external mutations |
| "Fix issue #123" | `dart-fix-issue` | `contributing.md`, `changelog.md`, task-specific docs | `pixi run lint`, focused build/test |
| "CI is failing on my PR" | `dart-fix-ci` | `ci-cd.md`, `release-management.md` for a `release-*` base, failing logs | reproduction command plus fixed check |
| "Update docs for X" | `dart-docs-update` | `docs/README.md`, `docs/ai/principles.md`, `docs/ai/verification.md`, `ai-tools.md`, `changelog.md`, relevant docs | Relevant docs/AI checks from `docs/ai/verification.md`; explicit approval before push/PR |
| "Revise the plan" | `dart-plan-update` | `docs/ai/principles.md`, `docs/ai/north-star.md`, `docs/plans/README.md`, `docs/plans/dashboard.md`, `docs/plans/north-star-roadmap.md`, `docs/ai/verification.md` | Relevant docs/AI checks from `docs/ai/verification.md` |
| "Review PR #123" | `dart-review-pr` | `code-style.md`, `ai-tools.md`, PR diff | read-only findings unless explicit approval allows mutations |
| "Create a PR" | `dart-pr` | PR template, `contributing.md`, `ai-tools.md`, `changelog.md` | `pixi run lint`, target-specific gates, milestone/CHANGELOG decision, before/after headless visual evidence when visual output changes, explicit approval before push/PR creation |
| "Continue the task" | `dart-resume` | active `RESUME.md`, dev task, `docs/ai/sessions.md`, `docs/ai/verification.md`, CI/contributing docs | `git status --short --branch`; promote durable dashboards/matrices before dev-task cleanup; explicit approval before push/PR update |
| "Merge PR #123" | `dart-manage-pr` | PR checks, `contributing.md`, `ci-cd.md`, `testing.md`, `ai-tools.md` | `mode=merge` is maintainer-only; `pixi run test-all` plus `pixi run -e cuda test-all`, CI/review green, an independent review, and explicit merge approval |
| "Triage the dependabot PRs" | `dart-deps` | `ci-cd.md`, `contributing.md`, open bot PRs | Read-only triage; merge only with CI/review green and an explicit approval gate |
| "Refresh the benchmark packet" | `dart-benchmark-packet` | `profiling.md`, the owning plan file | Packet checker plus task-specific gates; run the packet writer and `pixi run check-*-packets` |
| "Execute packet WP-091.3" | `dart-execute-packet` | `docs/ai/orchestration.md`, the owning plan file, packet owner docs | Packet gates plus task-specific gates; `pixi run lint` before commits; explicit approval before push/PR creation |
| "Pick up the next packet" | `dart-execute-packet` | `docs/ai/orchestration.md`, `docs/plans/dashboard.md`, the selected plan file | Auto-select walks the dashboard in priority order and runs the claim-signal check; packet gates apply; explicit approval before push/PR creation |
