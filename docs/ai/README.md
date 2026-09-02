---
type: ai-entrypoint
owner: self
---

# AI Agent Entrypoint

This directory is the shared AI-native starting point for DART. It keeps
durable agent workflow policy in tracked docs while tool-specific command and
skill files stay thin, generated, or compatibility-focused.

## Read Order

For general DART agent work:

1. `AGENTS.md`
2. `docs/ai/principles.md`
3. `docs/ai/README.md` (this file)
4. `docs/ai/north-star.md`
5. `docs/ai/workflows.md`
6. `docs/ai/verification.md`
7. The task-specific developer doc listed in `AGENTS.md`

For multi-session work, also read `docs/ai/sessions.md` and
`docs/dev_tasks/README.md`.

For project planning, also read `docs/plans/README.md`,
`docs/plans/dashboard.md`, and `docs/plans/north-star-roadmap.md`.

For documentation structure or placement work, also read `docs/README.md`,
`docs/information-architecture.md`, and `docs/AGENTS.md`.

For authoring or executing orchestrator-defined work packets, also read
`docs/ai/orchestration.md`.

For AI component maintenance or durable AI-infra self-improvement, read
`docs/ai/terminology.md`, `docs/ai/components.md`,
`docs/ai/verification.md`, and
`docs/onboarding/ai-tools.md`.

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
| `docs/ai/principles.md`         | AI-infra axioms and manual audit checklist                                          |
| `docs/ai/terminology.md`        | Canonical AI-facing terms and migration candidates                                  |
| `docs/ai/`                      | Durable AI-native mission and session rules not owned by a row below                |
| `docs/ai/workflows.md`          | Capability catalog: public paths, required docs, and minimum gates per workflow     |
| `docs/ai/verification.md`       | Gate selection, completion audit, and evidence expectations                         |
| `docs/ai/components.md`         | AI component mechanics, source surfaces, and the structural checks                  |
| `docs/ai/capabilities.json`     | Machine-readable capability status, category, and gate profile                      |
| `docs/ai/branch-profile.json`   | Machine-readable branch facts, required paths, exclusions, and AI-infra gates       |
| `docs/ai/agent-scenarios.json`  | Eight deterministic fresh-session routing and verification contracts                |
| `docs/ai/orchestration.md`      | Orchestrator/executor roles and the work-packet contract                            |
| `docs/plans/`                   | Living project priority, current state, next steps, and acceptance gates            |
| `docs/dev_tasks/`               | Temporary branch/session handoff state for active multi-session work                |
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

Run `pixi run ai-setup` once in a checkout to synchronize adapters and install
the cross-tool Git pre-commit guard. Run `pixi run ai-doctor` at session start
or after a discovery/setup failure; it reports versions, project trust-sensitive
surfaces, instruction chains, skills, agents, hooks, tasks, model pins, prompt
and skill-metadata sizes, and recovery commands without modifying the checkout.

Use `pixi run check-agent-hook` for the fast staged-file structural gate,
`pixi run test-ai-infra` for focused infrastructure tests, and
`pixi run check-ai-infra` for the aggregate non-mutating gate. The aggregate
also exercises deterministic orientation, small-change, failure-diagnosis,
documentation, model-upgrade, component, simulation-verification, and
release-maintenance scenarios. Full build/test
selection remains owned by `docs/ai/verification.md` and the task-specific
developer docs.

Codex loads project `.codex/` configuration and hooks only after the checkout
is trusted. Review project hooks with `/hooks`; changed hook definitions require
review again. A skipped Codex hook is not a correctness boundary: the installed
Git hook and explicit pre-commit gates remain authoritative.

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

- **Codex — GPT-5.6 family.** Use the Sol model for difficult ambiguous work,
  Terra for everyday or read-heavy work, and Luna for clear repeatable work.
  The Max reasoning mode gives one hard task more reasoning time; the Ultra
  mode is for independently parallelizable work when the user explicitly
  authorized delegation. Most tasks need neither mode. Sol, Terra, and Luna
  accept native image input.
- **Claude Code — current Claude models.** Use Fable 5.1 (`claude-fable-5-1`,
  the Mythos-class tier above Opus) for the hardest ambiguous or long-horizon
  work, or when Opus 5 at higher effort still falls short on the task; Opus 5
  (`claude-opus-5`) as the everyday strong default for substantial engineering
  work such as architecture, deep analysis, and agentic coding (Claude Code
  fast mode, toggled with `/fast`, runs only on Opus 5/4.8); Sonnet 5
  (`claude-sonnet-5`) for standard bounded work; and Haiku 4.5
  (`claude-haiku-4-5-20251001`) for quick lookups. Fable 5 (`claude-fable-5`)
  stays served as a legacy model; do not route new work to it. All four
  current models accept native image input. Reasoning effort is a
  session-level setting that defaults to `high`. On Fable 5.1, `medium`
  roughly matches Fable 5 at lower cost, so step down for routine or
  read-heavy turns; `xhigh` and `max` carry its largest gains but also its
  longest thinking time, so reserve them for one capability-sensitive task
  rather than a default. Fable 5.1 ships additional safety measures for
  dual-use capabilities that can occasionally refuse dual-use content; treat
  such a refusal as expected model behavior rather than a DART harness defect.
  (Mythos 5.1 is the same underlying model with safeguards tuned for its
  trusted-access program and is not generally available — do not try to
  select it.)
  If the refused task is legitimate DART work, restate it with its
  physics/simulation context made explicit, ask "are there bugs in this
  program?" rather than "does this compile?", keep base64 blobs out of tool
  output, and surface it to a maintainer if it still refuses.

A model-upgrade audit re-verifies native image input before adding or
replacing a lane.

The read-only project profiles in `.codex/agents/` inherit the selected parent
model.

Treat this routing as versioned guidance, not a permanent model taxonomy. Run
`dart-model-upgrade` for future model or Codex changes, keep task contracts
lean, and select the lightest model and effort that passes representative DART
checks.

Claude Code, OpenCode, Gemini CLI, future Codex models, and human contributors
remain supported: roles are not products, authoring and review stay separate,
and every workflow maps to public docs and `pixi run ...` commands. DART 6.20
has a separately maintained compatibility-first catalog; never infer that a
`main` path, task, or language/toolchain fact exists on the release branch.

## Safety Boundary

AI agents may inspect files, make local edits requested by the user, and run
local verification. GitHub mutations require explicit maintainer/user approval,
including pushes, PR comments, review-thread resolution, reviewer requests,
merges, and review re-triggers such as `@codex review`.

For automated review comments from bot accounts, follow
`docs/onboarding/ai-reviews.md`: never reply inline, verify each claim locally,
and treat any push, comment, thread resolution, or re-trigger as an external
mutation needing explicit maintainer/user approval.

## Required Gates

Use `docs/ai/verification.md` to select the strongest gate that matches the
work. AI-surface changes use its AI docs/adapters gate set.

Before committing, DART still requires `pixi run lint`. Code changes require
the build and test gates listed in `AGENTS.md` and the relevant developer docs.

Substantial AI-infra changes also require the principle audit in
`docs/ai/principles.md`; record the result in the final response or PR Testing
section.
