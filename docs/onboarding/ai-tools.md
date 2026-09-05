# AI Tools Compatibility

This file owns DART-specific setup, compatibility caveats, and dated evidence.
`docs/ai/README.md` owns model routing and the source map;
`docs/ai/components.md` owns authoring and adapter maintenance.

## Quick Reference

| Tool        | Instructions              | DART workflow / domain skill                         |
| ----------- | ------------------------- | ---------------------------------------------------- |
| Claude Code | `CLAUDE.md` → `AGENTS.md` | `/dart-*`: `.claude/commands/` and `.claude/skills/` |
| Codex       | `AGENTS.md`               | `$dart-*`: generated `.agents/skills/`               |

Run `pixi run ai-setup` once to synchronize adapters and install the Git guard;
`pixi run ai-doctor` diagnoses discovery/configuration without edits.
Tracked references are repository-relative. `@file` lines declare required
reading; automatic import behavior varies by tool, so load the files explicitly
when needed. Keep personal settings untracked.

Agents without a generated adapter read `AGENTS.md` and the `.claude/` sources
directly. Add a generated target in `scripts/sync_ai_commands.py` only when DART
adopts the tool.

## Detailed Compatibility

### Claude Code

**Tested Versions**: Claude Code 2.1.257 with Claude Fable 5.1
(`claude-fable-5-1`), 2026-09-01: command/skill loading, PreToolUse hook,
native capture review, and a controlled fresh-session comparison. The client
auto-updated to 2.1.258 during that session; the update was not separately
exercised. This is prior recorded evidence, not a new Fable runtime test.

**Notes**:

- For `/goal`, put the canonical command in the goal text, such as
  `/goal Run /dart-ultrawork with: <task>; done when: ...`. If goal text starts
  with `ulw:` or the common typo `ultrawok:`, normalize it to the canonical
  `/dart-ultrawork` workflow. These are prompt-level shorthands, not separate
  shared capabilities.
- Model and reasoning routing for current Claude models lives in
  `docs/ai/README.md` § "Model Routing"; do not duplicate or pin it here.
- For controlled model comparisons, `claude -p --model <id> [--effort <level>]`
  with the prompt on stdin runs a fresh non-interactive session that still
  loads the repository instructions and hooks. `--bare` drops hooks,
  `CLAUDE.md` discovery, and auto-memory and authenticates only with an API
  key or `apiKeyHelper`, so it strips exactly the harness a lane must
  exercise; do not use it for comparison lanes. Constrain lanes with
  `--allowedTools`/`--disallowedTools` and keep every lane on the same prompt,
  tools, and checkout state. Nested sessions also load the project's Claude
  Code auto-memory (`~/.claude/projects/<project>/memory/`), so quarantine
  audit notes written during the run before launching lanes and check each
  transcript for reads of them; a lane that saw the expected answer is not a
  control.

Current references:
[model configuration](https://code.claude.com/docs/en/model-config),
[Fable migration](https://platform.claude.com/docs/en/models/fable-5-1/migration-guide),
[Fable prompting](https://platform.claude.com/docs/en/build-with-claude/prompt-engineering/prompting-claude-fable-5-1),
[concise project instructions](https://code.claude.com/docs/en/best-practices), and
[skills](https://code.claude.com/docs/en/skills).
Guidance was refreshed on 2026-09-04. DART retains outcome, scope, evidence, and
handoff rules; client/API history handling belongs to the client, not a copied
prompting tutorial in repository skills.

### OpenAI Codex

**Tested Versions**: Codex CLI 0.153.2, 2026-09-04: strict-config startup,
discovery/config/hook checks, and fresh GPT-6 Astra Max/Ultra sessions.
A recorded Ultra parent explicitly spawned an Astra Max child; both recorded
memory disabled and read-only permissions. This supersedes the earlier local
account-access failure; availability must still be checked per account/client.

**Setup and diagnosis**:

1. Trust the checkout so project `.codex/` layers may load; run `codex doctor`
   when the installation, config, auth, or runtime itself looks unhealthy.
2. Run `pixi run ai-setup` to synchronize generated adapters and install the
   cross-tool Git hook.
3. Run `pixi run ai-doctor`. Resolve every reported missing or stale surface.
4. Open `/hooks`, review the exact project hook definition, and trust it if it
   matches the tracked file. Changed definitions require review again.
5. Use `$dart-*` skills; use `dart_scout`, `dart_reviewer`, or
   `dart_release_auditor` only for the bounded read-only contracts documented
   in `docs/ai/orchestration.md`.

Codex walks instruction files from repository root to the current directory;
the closest `AGENTS.md` augments or overrides broader guidance. Skills use
`$skill-name` syntax, including workflow-derived `$dart-*` adapters. CLI slash
commands are built-in session controls, not repository workflows. For goal
mode, put the generated adapter in the goal text, such as
`/goal $dart-ultrawork <task>`.

Use the current model and reasoning guidance in `docs/ai/README.md`; do not
duplicate or pin it here. Project agents inherit the active parent model.
`.codex/config.toml` bounds concurrency with `agents.max_threads` (Codex's
documented alias for `agents.max_concurrent_threads_per_session`; both spellings
load on the tested client and `check-ai-infra` accepts either, exactly one) and
delegation depth with `agents.max_depth`, while progressively loaded skills and
owner docs supply task procedures.

For controlled sessions, `--ignore-user-config` retains authentication while
excluding user configuration; disabling memories, plugins, apps, and hooks was
checked to retain DART skills without injecting the global memory/plugin
catalog. These are comparison controls, not recommended project defaults.
`--ephemeral` alone does not disable memory use. Keep rollout evidence when
checking child configuration: the CLI JSON event stream omits some delegation
details. Inspect each child's recorded model and effort, not just the parent's
requested settings. Explicit child overrides were verified; child defaults are
not an enforced model/effort allowlist. Supply the selected reasoning mode to
the task when the client does not expose it to the agent; reasoning mode and
Plan/Default collaboration mode are separate controls. A mode recommendation
does not itself switch the running session.

**Upgrade evidence (2026-09-04)**: fresh Astra Max/Ultra cases preserved
physics/text-image decisions, continuation constraints, and authorization
boundaries. Smaller declared intake improved some context measurements;
physics tokens increased and wall time was mixed. These bounded local cases
do not establish general quality or speed superiority. Raw comparisons belong
in the task's evidence artifacts, not this compatibility reference.

Project hooks are trusted-project automation, not complete enforcement.
`PreToolUse` does not intercept every possible mutation path, and a hook may be
skipped until trusted. The Codex hook therefore runs only the bounded,
noninteractive `check-agent-hook`; the installed Git hook and explicit
pre-commit/full gates remain authoritative.

On native Windows, `.claude/hooks/pre-commit-guard.ps1` and
`scripts/pretool_guard_bridge.py` forward hook input into the shared Git Bash
guard. Both pipeline and console input must preserve the payload and exit
semantics; keep their focused regression tests when changing the bridge.

Current references:
[GPT-6 Astra migration and prompting](https://developers.openai.com/api/docs/guides/model-guidance),
[Codex models and reasoning](https://learn.chatgpt.com/docs/models),
[Agent Skills](https://learn.chatgpt.com/docs/build-skills),
[subagents](https://learn.chatgpt.com/docs/agent-configuration/subagents),
[project configuration](https://learn.chatgpt.com/docs/config-file/config-advanced#project-config-files-codexconfigtoml),
[configuration reference](https://learn.chatgpt.com/docs/config-file/config-reference#configtoml), and
[hooks](https://learn.chatgpt.com/docs/hooks).

---

## Failure Recovery And Branch Differences

| Symptom                                | Recovery                                                                                                                        |
| -------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------- |
| `$dart-*` skill missing or stale       | Run `pixi run ai-doctor`, then `pixi run sync-ai-commands` and `pixi run check-ai-infra`                                        |
| Project agents or hooks do not appear  | Confirm the checkout is trusted; inspect `/hooks`; validate `.codex/` with `pixi run check-ai-infra`                            |
| Frequent hook blocks unexpectedly      | Run `pixi run check-agent-hook` directly; inspect JSON/input diagnostics; use the documented emergency bypass only if necessary |
| Full validation fails after quick gate | Select the task-specific focused/full gates in `docs/ai/verification.md`; the fast hook is not completion evidence              |
| A documented command/path is absent    | Confirm the current branch; run `pixi run ai-doctor`; fix the source owner rather than adding an unverified alias               |
| Generated file differs                 | Edit `.claude/` source, regenerate, and never patch `.agents/skills/` directly                                                  |

`main` is DART 7: C++23, nanobind, `dart::io`, the clean-break architecture,
CUDA validation, planning packets, benchmark packets, and DART 7 verification
skills belong there. `release-6.20` is DART 6: C++17, pybind11,
`dart::utils`, OSG, Gazebo compatibility, and release-maintenance workflows
belong there. The release catalog is intentionally smaller. Common AI-infra
changes use an apply/adapt/omit audit and branch-local gates; never copy a task,
path, command, or toolchain fact merely because it exists on the other branch.
`main` supports only Claude Code and Codex. `release-6.20` still carries
generated `.opencode/command/` adapters and `GEMINI.md` as release-local
surfaces; the release-to-`main` forward merge must drop them rather than
reintroduce them on `main`.

## Verification

After tool upgrades or discovery failures, run `pixi run ai-doctor`,
`pixi run check-ai-infra`, and `pixi run test-ai-infra`. Exercise a workflow,
domain skill, and affected hook in each tool whose behavior changed; record
the version, date, observed behavior, and untested boundaries here.
`dart-model-upgrade` owns model/harness comparisons;
`docs/ai/verification.md` owns gate selection and completion evidence.

Automated PR review handling lives in [ai-reviews.md](ai-reviews.md).
