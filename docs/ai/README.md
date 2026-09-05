---
type: ai-entrypoint
owner: self
---

# AI Agent Entrypoint

This directory is the shared AI-native starting point for DART. It keeps
durable agent workflow policy in tracked docs while tool-specific command and
skill files stay thin, generated, or compatibility-focused.

## Read Order

Start with `AGENTS.md` and `docs/ai/principles.md`, then the task-specific
owners listed in `AGENTS.md` or the selected skill. Use this index to find
policy; load the relevant workflow-catalog row and verification section when
routing work or choosing gates. Read `docs/ai/north-star.md` when project
direction matters. Do not load the entire catalog, every linked owner, or
already-read documents for each phase.

For multi-session work, read `docs/dev_tasks/README.md`.

For project planning, also read `docs/plans/README.md` and
`docs/plans/dashboard.md`.

For documentation structure, placement, or maintenance work, also read
`docs/README.md` and `docs/AGENTS.md`.

For authoring or executing orchestrator-defined work packets, also read
`docs/ai/orchestration.md`.

For AI component maintenance or durable AI-infra self-improvement, read
`docs/ai/terminology.md`, `docs/ai/components.md`,
and `docs/ai/verification.md`. Read `docs/onboarding/ai-tools.md` for
tool/runtime questions and before controlled agent runs.

## Visibility And Context Budget

`AGENTS.md`, this read order, and workflow `Required Reading` blocks are the
visibility contract for agents. Do not copy the same rule into every workflow;
move it to the owner doc, then add only the pointer needed for the workflow to
load it.

Always-loaded surfaces such as `AGENTS.md` and `docs/ai/principles.md` must
stay compact. Put procedures, compatibility details, and examples in the owner
docs named by those entrypoints.

When a documented rule is missed, use `dart-audit-agent-compliance` to diagnose
whether the issue is owner placement, weak wording, missing required reading, a
workflow description, or generated-adapter sync.

## Workflow Entrypoints

`docs/ai/workflows.md` is the capability catalog; each `.claude/commands/`
source owns its own procedure. Three entrypoints answer recurring questions:

- **Next task.** `dart-next` selects the next bounded task from the north
  star, dashboard, dev-task state, issues, PRs, and CI, then routes it to the
  most specific workflow; `focus=<topic>` is a preference, not a filter.
- **Autonomous project.** `dart-ultrawork` runs large or explicitly autonomous
  work from a brief or one up-front interview with `docs/dev_tasks/<task>/` as
  the project home; ordinary bounded work uses `dart-new-task`.
- **Model or coding-agent change.** `dart-model-upgrade` audits or updates the
  harness for a named model, reasoning mode, tool release, migration, or
  compatibility question, including a representative `dart-verify-sim` physics
  investigation. An incomplete target starts with installed-version and
  official-guidance discovery; a request to pin every agent still routes here,
  and the workflow evaluates it instead of treating it as approval for a
  blanket pin. A missed already-documented rule is `dart-audit-agent-compliance`
  work; an ordinary AI-doc edit is `dart-docs-update`.

## Source Ownership

| Surface                         | Role                                                                                |
| ------------------------------- | ----------------------------------------------------------------------------------- |
| `AGENTS.md`                     | Root pointer board and mandatory high-level rules                                   |
| `docs/ai/principles.md`         | AI-infra axioms and principle audit                                                 |
| `docs/ai/terminology.md`        | Canonical AI-facing terms and migration candidates                                  |
| `docs/ai/`                      | Durable AI policy not owned by a row below                                          |
| `docs/ai/workflows.md`          | Capability catalog: public paths, required docs, and minimum gates per workflow     |
| `docs/ai/verification.md`       | Gate selection, completion audit, and evidence expectations                         |
| `docs/ai/components.md`         | AI component mechanics, source surfaces, and the structural checks                  |
| `docs/ai/capabilities.json`     | Machine-readable capability status, category, and gate profile                      |
| `docs/ai/branch-profile.json`   | Machine-readable branch facts, required paths, exclusions, and AI-infra gates       |
| `docs/ai/agent-scenarios.json`  | Deterministic routing and verification contracts                                    |
| `docs/ai/orchestration.md`      | Orchestrator/executor roles and the work-packet contract                            |
| `docs/plans/`                   | Living project priority, current state, next steps, and acceptance gates            |
| `docs/dev_tasks/`               | Lifecycle owner (README) and temporary active task handoffs                         |
| `docs/onboarding/ai-tools.md`   | Tool compatibility and adapter maintenance details                                  |
| `docs/onboarding/ai-reviews.md` | Handling automated PR reviews and the review-fix loop                               |
| `.claude/commands/`             | Editable workflow source for DART user-invoked workflow capabilities                |
| `.claude/skills/`               | Editable domain-skill source for DART on-demand Agent Skills                        |
| `.agents/skills/`               | Generated Codex adapter entrypoints for DART workflow and domain-skill capabilities |
| `.codex/config.toml`            | Trusted-project bounded agent concurrency and delegation-depth policy               |
| `.codex/agents/`                | Discoverable read-only scout, reviewer, and release-auditor profiles                |
| `.codex/hooks.json`             | Maintained fast Codex command-hook configuration                                    |
| `.opencode/command/`            | Generated OpenCode command adapter entrypoints                                      |
| `scripts/sync_ai_commands.py`   | Adapter sync and AI docs consistency checker                                        |

Do not hand-edit generated `.agents/skills/` or `.opencode/command/` files.
Update the source surface, then run `pixi run sync-ai-commands`. Files under
`.codex/` are maintained project runtime configuration and must be reviewed
like scripts rather than regenerated.

## Agent-Friendly Setup And Diagnosis

Run `pixi run ai-setup` for checkout setup and `pixi run ai-doctor` for
read-only diagnosis. [Tool compatibility](../onboarding/ai-tools.md) owns
trust, hooks, and client-specific recovery; [components](components.md) owns
adapter maintenance.

## Model Routing

On `release-6.20` this routing lives under "Updating Models And Coding Agents"
in that branch's copy of this file; locate it by content, not by title.

DART uses the two-role operating model in `docs/ai/orchestration.md`: an
orchestrator session owns understanding, decomposition, sequencing, and review,
while executor sessions implement one well-defined work packet at a time.

No project model or reasoning effort is pinned: `.codex/config.toml`, the
Codex agent profiles, `.claude/settings.json`, and command or skill frontmatter
(`model`, `effort`) stay unset so the session choice wins, and
`pixi run check-ai-infra` rejects such pins. Routing is per tool lane, one
bounded entry per validated lane, and every lane states that its models accept
native image input because `dart-verify-sim` relies on it:

- **Codex — GPT-6 Astra (`gpt-6-astra`).** The maintainer's selected workflow
  uses only Astra with **Max** or **Ultra**. Astra sessions accept native image input.
  Max gives one difficult task more reasoning time; prefer it for tightly
  coupled planning and implementation. Ultra uses subagents for separable work;
  prefer it for substantial planning with independent research questions, or
  implementation with independent ownership scopes. Choose by task shape, not
  by phase alone. During planning, recommend the mode and record the approved
  delegation scope using `docs/ai/orchestration.md`; research/review approval
  does not authorize parallel writers. Keep delegated sessions on Astra with
  Max or Ultra and verify effective settings; inheritance alone does not prove
  child effort. If the requested model, mode, or child settings are unavailable,
  report the limitation without substituting another model or lower effort.
  These are session choices, not project configuration pins.
- **Claude Code — Fable 5.1 (`claude-fable-5-1`).** This frontier lane accepts
  native image input. Select effort within the user's authorized set using
  task evidence; effort names are not equivalent across model families.
  Preserve complete task scope, concise progress updates, and current-state
  handoffs. Current prompting/configuration guidance and dated runtime evidence
  live in `docs/onboarding/ai-tools.md`. Re-evaluate changes that compensate for
  an older model's behavior instead of carrying them forward automatically.

A model-upgrade audit re-verifies native image input before adding or
replacing a lane.

The read-only project profiles in `.codex/agents/` inherit the selected parent
model.

Treat this routing as versioned guidance, not a permanent model taxonomy. Run
`dart-model-upgrade` for future model or Codex changes, keep task contracts
lean, and choose within the user's authorized model and effort set. An explicit
restriction also bounds comparisons and delegated sessions; tool defaults do
not expand it.

Claude Code, OpenCode, Gemini CLI, future Codex models, and human contributors
remain supported: roles are not products, authoring and review stay separate,
and every workflow maps to public docs and `pixi run ...` commands. DART 6.20
has a separately maintained compatibility-first catalog; never infer that a
`main` path, task, or language/toolchain fact exists on the release branch.

## Safety Boundary

`docs/ai/principles.md` owns action/target/scope authorization.
`docs/onboarding/ai-reviews.md` owns bot-comment handling and PR review actions.

## Required Gates

`docs/ai/verification.md` owns gate selection and completion evidence, including
the principle audit and review requirements. `AGENTS.md` requires full
`pixi run lint` before every commit.
