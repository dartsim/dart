# DART AI Workflows

This directory contains the DART AI operating model used by Codex,
Claude Code, and Muse. It focuses on the workflows needed to
develop the Python-first DART 7 line on `main` while keeping the
Gazebo-facing C++ surface compatible.

## Visibility And Context Budget

`AGENTS.md`, this start list, and workflow `Required Reading` blocks are the
visibility contract for DART agents. Do not copy the same rule into
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
- [`branch-profile.json`](branch-profile.json): machine-readable DART
  compatibility facts, required surfaces, and forbidden markers and paths.
  Each branch names its line: `dart7` on `main` and `dart6` on
  `release-6.20`;
  [Release Management](../onboarding/release-management.md#release-target)
  owns each base branch's target.
- [`agent-scenarios.json`](agent-scenarios.json): the eight deterministic agent
  contracts, including model upgrades and simulation verification, from
  orientation through release maintenance.

## Architecture And Setup

Editable workflow and domain-skill sources live in `.claude/`. The sync tool
generates current Codex and Muse skills under
`.agents/skills/`. `.codex/` owns maintained Codex config, bounded
read-only subagents, and the advisory PreToolUse hook. The installed
git hooks are the cross-tool commit safety path.

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
GTest contract, and conftest-free pytest provenance. It also runs controlled
passing, failing, hostile-environment, and zero-body runner probes; it does not
run the project test suites and does not replace `test-ai-infra`, `test-all`,
hosted platform CI, or claim-tied semantic image review. `test-ai-infra` and
the visual debug-overlay gate use the same guarded pytest runner, which fails
when no test body executes. The direct structural checker form is reserved for
fast staged/setup internals and is not sufficient completion evidence.
The checker's delegation-policy reading uses the Pixi-supplied `markdown-it-py`;
run it through `pixi run` and it fails closed, never falling back to raw text,
under an interpreter without that parser.

Edit `.claude/`, run `pixi run sync-ai-commands`, and never hand-edit generated
adapters. `.agents/skills/.dart-generated.json` owns only DART-generated paths;
unrelated skills in the shared discovery directory must be preserved.

For documentation placement, use
[`docs/information-architecture.md`](../information-architecture.md).
For living roadmap state, use [`docs/plans/dashboard.md`](../plans/dashboard.md).

Use `$dart-ultrawork` in Codex or `/dart-ultrawork` in Claude and
Muse when DART work should run as an autonomous
project from either a provided brief or one up-front decision interview. The
workflow keeps `docs/dev_tasks/<task>/` as the project home and records
acceptance criteria, risks, decisions, verification, progress, and handoff
state before routing bounded work through the DART
orchestrator/executor model. Use `dart-new-task` for ordinary bounded
single-session work unless the user explicitly asks for autonomous project
handling.

## Updating Models And Coding Agents

Use `$dart-model-upgrade` in Codex or `/dart-model-upgrade` in
Claude and Muse when a request names a model, reasoning mode,
coding-agent
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

Every target also exercises one representative DART physics investigation
through `dart-verify-sim`: the same text oracle, assessed Filament view, and
claim-specific debug layers. Rendering or native image-review limitations are
reported rather than converted into model-quality claims.

The project pins neither a model nor an effort. Delegate by default under
[`orchestration.md`](orchestration.md), honoring a per-task `delegation=off`
opt-out; delegation does not require a particular reasoning setting. Routing
is per tool lane, one bounded entry per lane, refreshed from primary guidance
on 2026-10-17:

- **Codex — GPT-6 family.** Start with GPT-6.1 Sol (`gpt-6.1-sol`) for most
  work, including ambiguous multi-step work that needs planning and
  validation; use GPT-6 Astra (`gpt-6-astra`) for the hardest end-to-end work
  and GPT-6 Luna (`gpt-6-luna`) for clear, repeatable, or narrowly scoped
  packets. In the API, `gpt-6.1-sol` accepts reasoning effort `low`, `medium`
  (the API default), `high`, `xhigh`, and `max`, but not `none` or `minimal`;
  other models' ranges differ. In Codex, start from the effort your client
  offers by default. API reasoning modes `standard` and `pro` are independent of
  effort. In Codex, `max` gives one task more reasoning time, and `ultra`, on
  supported models, uses subagents for complex work. Select only settings the
  current client and model advertise. See OpenAI's
  [model guide](https://developers.openai.com/api/docs/guides/latest-model),
  [reasoning guide](https://developers.openai.com/api/docs/guides/reasoning),
  [Codex models](https://learn.chatgpt.com/docs/models), and
  [Codex subagents](https://learn.chatgpt.com/docs/agent-configuration/subagents).
- **Claude Code — current Claude models.** Use Fable 5.1 (`claude-fable-5-1`,
  the tier above Opus) for the hardest ambiguous or long-horizon work; Opus
  5.5 (`claude-opus-5-5`) for substantial maintenance work; Sonnet 5.5
  (`claude-sonnet-5-5`) for standard bounded work; and Haiku 5.5
  (`claude-haiku-5-5`) for quick lookups and repeatable packets. Claude Code
  offers `low`, `medium`, `high`, `xhigh`, and `max`; use `--effort` for a
  session override, while `/effort` normally saves per-model preferences.
  Reserve `max` for a hard task. Fable 5.1, Opus 5.5, Sonnet 5.5, and Haiku 5.5
  run safety classifiers that can occasionally decline dual-use content; treat
  such a refusal as expected model behavior rather than a DART harness defect.
  (Mythos-class models are restricted to approved organizations — do not try
  to select one.) If the refused task is legitimate DART work, restate it with
  its physics/simulation context made explicit, and surface it to a maintainer
  if it still refuses. See Anthropic's
  [model catalog](https://platform.claude.com/docs/en/models/overview) and
  [Claude Code effort guidance](https://code.claude.com/docs/en/model-config#adjust-effort-level).
- **Muse — Muse Code 1.4.10.** Use `muse-wonder-full-1.0`, available only on
  the Muse Code Power Usage and Maximum Usage plans, for the most critical
  work; `muse-wonder-1.0`, on every plan, for complex software engineering and
  long-horizon agentic work and as the fallback for Wonder Full; and
  `muse-spark-1.3` for high-volume, latency-sensitive, or cost-sensitive
  packets such as subagent tasks. Muse Wonder accepts reasoning effort
  `minimal` through `max`, and neither Wonder nor Spark accepts `none`; the
  Muse Code CLI also lists `ultra`, which the reasoning guide does not, so
  prefer the model-specific values and reserve `max` for a hard task. See the
  [reasoning guide](https://dev.meta.ai/docs/reasoning),
  [Wonder catalog](https://dev.meta.ai/models/muse-wonder),
  [model-selection guidance](https://dev.meta.ai/docs/models), and
  [Muse Code configuration](https://dev.meta.ai/docs/muse-code/configuration).

Published capabilities and installed-version probes do not establish behavioral
or visual-debug quality. See the exercised scope in
[`ai-tools.md`](../onboarding/ai-tools.md#codex-project-setup); Claude and Muse
still need their own representative Filament investigations.

Treat these entries as replaceable versioned guidance, not a permanent model
taxonomy.

## Development Profile

Development and release branches preserve C++17, `dart::utils`, and
Gazebo/gz-physics compatibility, and each keeps the dartpy binder and GUI
backend it ships with: nanobind under `python/dartpy/` and Filament on `main`
and DART 7 release branches; pybind11 and OSG on `release-6.20`. See
[`python-bindings.md`](../onboarding/python-bindings.md). The DART component
map is [`architecture.md`](../onboarding/architecture.md). Release and backport
policy and the branch-local next release live in
[`release-management.md`](../onboarding/release-management.md).
