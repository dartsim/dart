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

## Codex Hosted Review Settings

Recommended starting configuration, based on the maintainer's settings UI
confirmed on 2026-09-05:

| Setting                | Choice                  |
| ---------------------- | ----------------------- |
| Auto review            | On                      |
| Review trigger         | On PR open              |
| Exhaustive code review | On for a measured trial |
| Enable credits use     | Off                     |

These are account/repository preferences, not local agent model or effort
settings. Check the effective repository policy as well as personal preferences
before relying on an automatic trigger. See the
[official GitHub review documentation](https://learn.chatgpt.com/docs/third-party/github)
for automatic/manual requests and scoped repository review rules.

The observed UI exposes one general Exhaustive toggle. Treat it as enabled for
follow-up reviews too; there is no observed initial-review-only or per-round
control. Do not toggle it between rounds. Its description promises continued
search for additional findings until no new issues are found, not defect-free
code. Internal pass count, billing multiplier, and cost savings were not
established. Completion must use the evidence rules in
[ai-reviews.md](ai-reviews.md#codex-review-for-draft-prs).

PR-open automation plus deliberate manual follow-ups fits batched fixes.
Every-push automation can race those requests; experimental smart detection is
not proof of required current-head coverage. A different chosen configuration
must still obey the single-trigger-owner rule. Changing account settings or
enabling credits requires separate explicit authorization.

### Evaluating The Trial

Evaluate the next ten representative PRs using their existing verification
evidence, recording PR/head, settings, hosted round count, accepted/rejected
findings, repair-induced regressions, time to readiness, and local agent tokens
and hosted review usage where available. Add local correctness/contracts
review time, escaped defect families, and total review cost when measurable.
Compare with similar prior PRs and
separate physics, tooling, and documentation changes; unavailable usage is
unknown, not zero. Do not infer dollar savings from comment counts.

Retain Exhaustive if broader early discovery and fewer repair cycles justify
its review usage without degrading quality. Otherwise recommend disabling the
general toggle while retaining batching and the strategy checkpoint. Report
the sample and limitations; neither structural checks nor a small mixed sample
prove causal savings. This is a trial protocol, not evidence that ten PRs have
already been evaluated.

## Local Review Evidence Interface

**Implementation trial (2026-09)**: observed session settings confirmed GPT-6
Astra Max for the archived and contrast reviews. The [behavioral replay](ai-reviews.md#local-gate-behavioral-replay-2026-09)
records the results and limitations; this is tested-version evidence, not a
project model pin.

Run `pixi run install-hooks` once per repository and after checker updates.
The shared Git hooks directory contains both hooks and a dependency-free Python
3.11+ checker. The launcher verifies its installed bytes and runs Python in
isolated mode; truncated or modified checker files fail closed. Older linked
worktrees use that installed copy. Setup installs
it; `pixi run ai-doctor` reports a missing/stale hook or checker. Custom
`core.hooksPath` managers retain control: the installer refuses configured
values, including empty or whitespace-only values; use the [custom-manager integration](#custom-hook-managers)
below for both gates.
Foreign executable hooks are preserved as `pre-commit.local` and
`pre-push.local`; the push gate replays Git's original stdin and arguments and
propagates the foreign hook's exit status. A preservation collision stops
installation. An unowned installed checker also stops installation before any
hook changes. `DART_SKIP_HOOKS` applies only to the commit guard.

After fetching/merging the current base, committing and validating the candidate:

```bash
pixi run review-gate prepare --base origin/<base> --head HEAD --remote origin \
  --target refs/heads/<topic> --author-session <author-session-id>
# Give both read-only reviewers the returned candidate ID and candidate.json.
# Each returns its final JSON report; the parent imports it without rewriting it.
pixi run review-gate record <candidate> <correctness-report.json>
pixi run review-gate record <candidate> <contracts-report.json>
pixi run review-gate check <candidate>
# Only after existing explicit maintainer/user approval covers publication:
git push origin HEAD:refs/heads/<topic>
```

### Installed CLI For Older Worktrees

The installed checker also exposes the supported `prepare`, `record` and
`check` commands. Use it from the checkout being reviewed when that checkout
lacks the `review-gate` Pixi task or source script:

```bash
dart_review_gate="$(git rev-parse --git-path hooks)/dart-review-gate.py"
python3 -I "$dart_review_gate" prepare --base origin/<base> --head HEAD \
  --remote origin --target refs/heads/<topic> --author-session <author-session-id>
python3 -I "$dart_review_gate" record <candidate> <reviewer-report.json>
python3 -I "$dart_review_gate" check <candidate>
```

In PowerShell, set the same path with
`$dart_review_gate = Join-Path (git rev-parse --git-path hooks) "dart-review-gate.py"`
and use it with the commands above. `python3` must name an available Python
3.11+ interpreter; substitute its full path or the Windows `python` command
when needed. No third-party packages or Pixi task are required. The commands
use the current checkout and shared evidence store; returning to another
worktree is unnecessary. Custom hook managers use their retained runtime path
below.

### Evidence Records

`prepare` requires one push URL for the named remote and a fetched remote base
that is already an ancestor. Repeat `--author-session` for every authoring
session, including earlier tools or executors. It carries earlier authors and
findings for that publication target automatically. Reviewer sessions cannot
be authors, and the two scopes need different sessions. Agent session settings
must be observed, not guessed from the author model or a requested override.
No model runs inside this interface; humans and other tools use the same report
contract. The reviewer owns its verdict and dispositions; the parent records
its final output, not private reasoning or an invented clean result.

Records live in `<git-common-dir>/dart-review/` outside tracked files. A
versioned candidate binds commit, tree, fetched base ref and commit, merge base,
an opaque digest binding the exact push URL and target branch, author sessions,
and previous candidate. New records do not contain the raw push URL or embedded
credentials. Existing local records remain readable without rewriting their
history; preparing a legacy record creates a new opaque candidate.
Reports have an ordered hash manifest; missing or corrupted files block the
check. `prepare` returns the existing candidate for unchanged input. A changed
commit, author set, or fetched base produces a new candidate. Keep these records
across handoffs; do not remove them to discard a finding. A fresh clone has no
local evidence and must obtain reviews before publication.

Reviewer JSON schema (version 1; replace example values):

```json
{
  "schema_version": 1,
  "candidate": "<64-character candidate ID>",
  "reviewer": {
    "session": "<independent session ID>",
    "kind": "agent",
    "tool": "<observed tool>",
    "model": "<effective model>",
    "effort": "<effective effort>"
  },
  "scope": "correctness",
  "status": "complete",
  "verdict": "clean",
  "summary": "No actionable findings survive inspection.",
  "report": "Final reviewer output with evidence and limitations.",
  "coverage": ["Complete base-to-head diff and acceptance checks inspected."],
  "coverage_complete": true,
  "findings": [],
  "dispositions": []
}
```

Use `kind: human` for a human session; tool/model/effort are then unnecessary.
The second scope is `contracts`. `status` can be `incomplete`; `verdict` is
`clean` or `findings`. Incomplete work or missing required acceptance coverage
cannot count as clean. Apply the review owner's stage distinction: enumerate
hosted checks still pending after initial publication in `coverage` and `report`;
never report an unexecuted platform as passed. A finding has `id`, `summary`, and `evidence` strings.
Use unique IDs for different issues and keep each logical ID stable across
candidates. Later reports may refine its summary or evidence; all earlier
hashed reports remain intact. A disposition has that `id`, `status`
(`fixed` or `rejected`), and concrete `evidence`. Only a completed reviewer who
is independent of every active author can close findings, including through
ancestor reports. A reviewer becoming an author revokes their earlier
dispositions. `fixed` requires a later candidate with a different head from
the latest commit on which that finding was reported; `rejected` can apply
to the same commit. Amended repairs still require full independent reviews
and any applicable history-rewrite approval.
The gate evaluates accumulated findings and the latest report
per session and scope; a later clean report alone does not close an issue.

Each candidate, reviewer input, report and manifest is limited to 2 MiB.
The gate checks all serialized sizes before writing an update; an oversized
update is rejected before any evidence file changes. A validated transaction
record commits the complete update before individual files are replaced
atomically. The next command finishes interrupted publication under the store
lock without discarding earlier findings. The recovery envelope combines up
to three artifacts and is separately limited to 8 MiB. A recovered import may
report that it is already recorded; run `check` to inspect the resulting
candidate. After a killed process, verify it
has stopped before removing its empty lock directory; retain `pending.json`
for recovery. Corrupt or oversized transaction data blocks recovery.

For the policy's trivial exception use `scope: non-substantive`, add
`no_behavior_change: true` and a concrete `reason`. For an update also supply
`baseline: <previously passed candidate ID>`; it must be a reviewed ancestor
with the same base. Initial trivial publication may omit the baseline.

The pre-push entrypoint consumes the actual outgoing ref/SHA tuples, so a
review of HEAD cannot authorize another source branch. All branch updates must
pass; tags and deletions are outside this gate. The hook is offline and compares
the fetched base, not live remote state. A failure explains missing or stale
evidence. Restore corrupted evidence from its authentic source, rerun needed
reviews, or reinstall a missing checker/interpreter; never automatically bypass
a failure. Ordinary Git bypasses remain possible, and the records are local
attestations rather than authentication or proof that a reviewer found every bug.

### Custom Hook Managers

Keep the configured manager and its existing checks. The installer deliberately
does not modify `core.hooksPath`. The manager must run both handlers below,
propagate failures, and supply the original pre-push arguments and stdin.
Use an already available Python 3.11+ interpreter; these handlers perform no
environment installation, network, model, build or test work.

From a checkout containing the current gate, export its commit handler, verified
pre-push launcher and standalone checker outside branch-controlled files. Repeat
this command when the hooks or checker change:

```bash
pixi run install-hooks --custom-manager
```

The export checks ownership of all three output files before writing any of
them, then publishes `dart-review-gate.py`, `dart-review-pre-commit` and
`dart-review-pre-push` in that order directly in Git's canonical common
directory. Each file replacement is atomic. The manager's configuration and
handlers, and any earlier `dart-review-runtime` directory, remain intact.
The export refuses unowned or aliased output files and a common directory
that doubles as a hooks directory. The push launcher verifies the checker
digest before execution. Interrupted initial exports or mismatched refreshes
block pushes; rerun the export to finish installation. Stable ownership markers
permit recovery after partial installation or a damaged checker with an owned
pre-push launcher. An earlier two-file export can add the missing commit handler;
an existing unowned commit handler stops the whole refresh.

The manager's pre-commit handler invokes the canonical commit guard:

```sh
#!/bin/sh
dart_common=$(git rev-parse --git-common-dir) || exit 1
exec "$dart_common/dart-review-pre-commit" "$@"
```

Its pre-push handler invokes the exported launcher, including in older worktrees:

```sh
#!/bin/sh
dart_common=$(git rev-parse --git-common-dir) || exit 1
exec "$dart_common/dart-review-pre-push" "$@"
```

Set `DART_HOOK_PYTHON` to the chosen interpreter when `python3` is unavailable.
These are separate handlers: retain any additional manager-owned checks and
their ordering. Do not consume pre-push stdin before passing it to the checker.
For evidence preparation from an older checkout, use the installed-CLI recipe
with `$dart_common/dart-review-gate.py` as the runtime path. The commit guard
selects compatible Python and runs the staged guard with its local imports.
In older worktrees or without compatible Python, it runs Git's staged whitespace
check. It retains `DART_SKIP_HOOKS` and `DART_HOOK_DRY_RUN` commit behavior; neither
flag bypasses the push gate. Both exported handlers leave all chaining to the
manager; they do not run incidental `pre-commit.local` or `pre-push.local` files.
The doctor reports manager ownership; it cannot certify arbitrary manager
configuration. Verify integration with a disposable unreviewed branch push
that is blocked, followed by a reviewed push that succeeds.
