# DART 6.20 AI Workflows

This directory contains the release-branch AI operating model used by Codex,
Claude Code, and OpenCode. It is intentionally smaller than the DART 7 `main`
workflow surface and focuses on the workflows needed to maintain the DART 6 LTS
line.

## Visibility And Context Budget

`AGENTS.md`, this start list, and workflow `Required Reading` blocks are the
visibility contract for release-branch agents. Do not copy the same rule into
every workflow; move it to the owner doc, then add only the pointer needed for
the workflow to load it.

Always-loaded surfaces such as `AGENTS.md`, `docs/ai/principles.md`, and
`docs/ai/verification.md` must stay compact. Put procedures, compatibility
details, and examples in the owner docs named by those entrypoints.

When a documented rule is missed, diagnose whether the issue is owner
placement, weak wording, missing required reading, a workflow description, or
generated-adapter sync.

Start with:

- [`principles.md`](principles.md)
- [`terminology.md`](terminology.md)
- [`orchestration.md`](orchestration.md)
- [`workflows.md`](workflows.md)
- [`verification.md`](verification.md)
- [`sessions.md`](sessions.md)
- [`components.md`](components.md)
- [`capabilities.json`](capabilities.json)
- [`branch-profile.json`](branch-profile.json): machine-readable DART 6.20
  facts, required surfaces, and DART 7 exclusions.
- [`agent-scenarios.json`](agent-scenarios.json): the eight deterministic agent
  contracts, including model upgrades and simulation verification, from
  orientation through release maintenance.

## Architecture And Setup

Editable workflow and domain-skill sources live in `.claude/`. The sync tool
generates current Codex skills under `.agents/skills/` and OpenCode commands
under `.opencode/command/`. `.codex/` owns maintained Codex config, bounded
read-only subagents, and the advisory PreToolUse hook. The installed git hook is
the cross-tool commit safety path.

```bash
pixi run ai-setup
pixi run ai-doctor
```

Codex loads project config, agents, and hooks only for a trusted repository.
Inspect the project hook with `/hooks`. If hooks are unavailable, run
`pixi run python scripts/check_agent_hook.py --profile staged` manually; always
run `pixi run lint` before a commit.

## Focused And Full Checks

```bash
pixi run check-ai-commands
pixi run check-ai-infra
pixi run test-ai-infra
pixi run exercise-agent-scenarios
pixi run lint
```

`check-ai-infra` is the completion gate: it includes the structural checks and
validates the configured Release test graph through the CMake File API,
expanded target trace, configuration-selected CTest inventory, clean ambient
GTest contract, and conftest-free pytest provenance. It does not execute CTest
or pytest and does not replace `test-ai-infra`, `test-all`, hosted platform CI,
or claim-tied semantic image review. The direct structural checker form is
reserved for fast staged/setup internals and is not sufficient completion
evidence.

Edit `.claude/`, run `pixi run sync-ai-commands`, and never hand-edit generated
adapters. `.agents/skills/.dart-generated.json` owns only DART-generated paths;
unrelated skills in the shared discovery directory must be preserved.

For documentation placement, use
[`docs/information-architecture.md`](../information-architecture.md).
For living roadmap state, use [`docs/plans/dashboard.md`](../plans/dashboard.md).

Use `$dart-ultrawork` in Codex or `/dart-ultrawork` in
Claude/OpenCode when DART 6 work should run as an autonomous release-branch
project from either a provided brief or one up-front decision interview. The
workflow keeps `docs/dev_tasks/<task>/` as the project home and records
acceptance criteria, risks, decisions, verification, progress, and handoff
state before routing bounded work through the release-branch
orchestrator/executor model. Use `dart-new-task` for ordinary bounded
single-session work unless the user explicitly asks for autonomous project
handling.

## Updating Models And Coding Agents

Use `$dart-model-upgrade` in Codex or `/dart-model-upgrade` in
Claude/OpenCode when a request names a model, reasoning mode, coding-agent
release, migration, or compatibility audit. It captures the existing release
harness as a control, refreshes primary guidance, isolates model, effort,
prompt, configuration, and optional-agent changes, and records
preserve/update/remove/consolidate/add verdicts.

The workflow is deliberately self-evolving: its model-agnostic intake,
comparison, verification, and closeout core is audited on every run. An
`apply` run may improve that source when a target exposes a reusable gap, while
replacing stale target-specific guidance instead of cloning a workflow.

DART AI infrastructure includes tracked documentation that supplies in-session
context and across-session project state. Model upgrades therefore audit
`docs/ai/`, living state in `docs/plans/`, active handoffs in
`docs/dev_tasks/`, and the handbook/design/release owners routed into a task,
not only configuration, prompts, adapters, hooks, scripts, and tests.

Every target also exercises one representative DART 6 physics investigation
through `dart-verify-sim`: the same text oracle, assessed OSG view, and
claim-specific debug layers. Rendering or native image-review limitations are
reported rather than converted into model-quality claims.

For GPT-5.6, use Sol for hard ambiguous work, Terra for everyday or read-heavy
work, and Luna for clear repeatable work. Max spends more reasoning on one
difficult task; Ultra is useful only for independently parallelizable work when
delegation is authorized. The project pins neither a model nor an effort.
Treat this paragraph as replaceable versioned guidance, not a permanent model
taxonomy.

## Release Profile

`release-6.20` preserves C++17, pybind11, `dart::utils`, OSG, and
Gazebo/gz-physics compatibility. DART 7's C++23, nanobind, `dart::io`, solver,
and backend workflows remain on `main`. Use DART 7 as comparison evidence only;
adapt or omit each difference instead of copying the larger workflow surface.
The release component map is [`architecture.md`](../onboarding/architecture.md).
