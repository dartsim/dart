# AI Tooling And Review Rules

This release branch supports Claude Code, OpenCode, and Codex workflow
entrypoints generated from `.claude/` sources.

## Source And Generated Files

- Edit workflow commands in `.claude/commands/`.
- Edit domain skills in `.claude/skills/`.
- Do not hand-edit `.agents/skills/` or `.opencode/command/`; run
  `pixi run sync-ai-commands` instead.
- Check parity with `pixi run check-ai-commands`.
- Use canonical DART capability names in prompt-driven goal modes: Claude goal
  text should name `/dart-ultrawork`, and Codex goal text can name
  `$dart-ultrawork`. If Claude goal text starts with `ulw:` or the common typo
  `ultrawok:`, normalize it to the canonical `/dart-ultrawork` workflow. These
  are prompt-level shorthands, not separate shared capabilities.

## Codex Project Setup

Current Codex discovers project skills under `.agents/skills/`. It reads
bounded agent profiles from `.codex/agents/`, project defaults from
`.codex/config.toml`, and the advisory PreToolUse hook from
`.codex/hooks.json` after the repository is trusted. Use:

```bash
pixi run ai-setup
pixi run ai-doctor
pixi run ai-doctor --json
```

The project does not pin a model or reasoning effort. Model and reasoning
routing lives in `docs/ai/README.md` § "Updating Models And Coding Agents";
do not duplicate or pin it here. The three read-only project agents inherit
the selected parent model: `dart_scout` gathers evidence, `dart_reviewer`
audits the current diff, and `dart_release_auditor` compares `main` with the
DART 6.20 compatibility surface.

**Tested Versions**: Claude Code CLI 2.1.252 (Claude Fable 5), Codex CLI
0.151.0, OpenCode 1.18.21, 2026-08-31 — discovery, config, and hook checks
exercised locally on this branch.

Use `dart-model-upgrade` for future model or coding-agent changes; its audit
includes configuration, prompts, generated adapters, runtime tooling, durable
`docs/` context, session handoffs, and a representative DART 6 OSG
visual-debug investigation.

Inspect project hooks with `/hooks`. Project hooks are advisory and may be
skipped in an untrusted repository. `pixi run install-hooks` installs the
cross-tool pre-commit and pre-push hooks. Re-run it after updating the
checker; `pixi run ai-doctor` reports missing or stale installation.

On native Windows, `.claude/hooks/pre-commit-guard.ps1` launches
`scripts/pretool_guard_bridge.py`, which forwards the unchanged hook payload to
the same Git Bash guard used on POSIX; commit classification has one shared
implementation. The manual fallback is:

```bash
pixi run python scripts/check_agent_hook.py --profile staged
```

Both paths are fast safety checks, not a substitute for `pixi run lint` before
commits.

## Other Clients And Manual Fallback

Claude Code uses the editable `.claude/` commands and skills. OpenCode uses the
generated `.opencode/command/` adapters. Codex uses `.agents/skills/` plus the
trusted `.codex/` runtime layer. Gemini and other clients that read
`AGENTS.md` can follow the same owner docs and `pixi run ...` gates without a
tool-specific command surface. Never make correctness depend only on a project
hook or one client's private state; the public docs, direct commands, and
installed git hook remain the fallback contract.

## Approval Boundaries

The following actions require explicit maintainer/user approval:

- pushing commits;
- opening, editing, marking ready, or merging PRs;
- posting PR or issue comments;
- rerunning CI;
- resolving review threads;
- deleting local or remote branches (explicit maintainer/user approval,
  like every item in this list).

## AI Review Comments

`docs/onboarding/ai-reviews.md` owns independent local publication reviews and
the automated review-fix loop: no inline bot replies, complete finding batches,
one trigger owner, current-head evidence, and the two-round strategy checkpoint.
Use `dart-review-pr` for a local candidate or hosted feedback.

## PR Branches

Before every push with explicit maintainer/user approval, fetch and merge the latest
target base branch into the topic branch. Use merge, not rebase, unless a
maintainer explicitly requests history rewriting.

Never push directly to `release-*` branches. Create a topic branch from the
release base without tracking the release ref:

```bash
git fetch origin release-6.20
git switch --no-track -c <type>/<topic> origin/release-6.20
```

After explicit maintainer/user approval, push the topic branch with the same
local and remote branch name:

```bash
branch=$(git branch --show-current)
# Requires explicit maintainer/user approval.
git push -u origin "HEAD:${branch}"
```

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

Run `pixi run install-hooks` once per repository and after checker updates.
The shared Git hooks directory contains both hooks and a dependency-free Python
3.11+ checker. The launcher verifies its installed bytes and runs Python in
isolated mode; truncated or modified checker files fail closed. Older linked
worktrees use that installed copy. Setup installs
it; `pixi run ai-doctor` reports a missing/stale hook or checker. Custom
`core.hooksPath` managers retain control: the installer refuses to overwrite
that configuration, so integrate equivalent hooks with the manager explicitly.
Foreign executable hooks are preserved as `pre-commit.local` and
`pre-push.local`; the push gate replays Git's original stdin and arguments and
propagates the foreign hook's exit status. A preservation collision stops
installation. `DART_SKIP_HOOKS` applies only to the commit guard.

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
remote push location, target branch, author sessions, and previous candidate.
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
(`fixed` or `rejected`), and concrete `evidence`. Only a completed reviewer can
close findings. The gate evaluates accumulated findings and the latest report
per session and scope; a later clean report alone does not close an issue.

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
