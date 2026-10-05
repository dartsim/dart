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
audits the current diff, and `dart_release_auditor` classifies named
reference material as apply/adapt/omit against the DART 6.20 compatibility
surface.

**Tested Versions**: Claude Code CLI 2.1.252 (Claude Fable 5), Codex CLI
0.151.0, OpenCode 1.18.21, 2026-08-31 — discovery, config, and hook checks
exercised locally on this branch.

Use `dart-model-upgrade` for future model or coding-agent changes; its audit
includes configuration, prompts, generated adapters, runtime tooling, durable
`docs/` context, session handoffs, and a representative DART 6 OSG
visual-debug investigation.

Inspect project hooks with `/hooks`. Project hooks are advisory and may be
skipped in an untrusted repository. `pixi run install-hooks` installs the
cross-tool git hook.

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

Never reply to AI-generated review comments from bot users such as
`chatgpt-codex-connector[bot]`, `github-code-quality[bot]`,
`github-actions[bot]`, or `copilot[bot]`.

Make fixes silently. After an approved follow-up push, request a new top-level
review only when explicit approval covers the PR comment.

## PR Branches

Before every approved push to a published PR branch, fetch and merge the latest
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

## PR Lifecycle

Agent-authored PRs on `release-6.*` branches follow these steps. Each GitHub
mutation still needs explicit maintainer/user approval: a `manage <PR>` request
covers the routine ones (see `dart-manage-pr`), and step 4 defines merge
approval.

1. Open the PR as a draft and post one top-level `@codex review`.
2. Leave draft only when Codex has reviewed the current head without findings
   and CI is green. Codex is the only AI reviewer on these branches: Copilot
   review requests are ignored for this repository, and
   `github-code-quality[bot]` covers only the default branch (`main`).
   Compare the `Reviewed commit` in Codex's result with the PR head; if a push
   moved the head past it, post a new `@codex review` for the new head before
   leaving draft. Fix findings as "AI Review Comments" describes. Address every
   Codex review on the PR, not only the one you requested: opening a PR for
   review and marking it ready also trigger reviews, one head can get several
   reviews with different findings, and findings arrive either as inline
   suggestions in a "Codex Review" review or as a top-level comment.
3. When that gate passes, run `gh pr ready <PR>`. This hands the PR to the
   maintainer and triggers one more Codex review, listed as
   `Draft marked ready` in Codex's review summary comment. Do not merge until
   it completes. Unaddressed AI review findings block the merge even after the
   maintainer's +1: fix them and get a clean re-review of the new head first.
4. Merge only after a maintainer (currently `jslee02`) adds a +1 reaction to
   the PR description (`gh api repos/dartsim/dart/issues/<PR>/reactions`).
   That reaction is explicit approval for the agent or the maintainer to merge
   that PR, including the step 5 base update. The +1 that
   `chatgpt-codex-connector[bot]` adds after a clean review is not. Agents
   merge through the `dart-manage-pr` `mode=merge` gate. Squash-merge: the
   repository allows squash and rebase merges, not merge commits. Fix AI
   review findings that arrive afterwards in a follow-up PR.
5. Merge a series in dependency order, noted in each PR body. Release branches
   require PR branches to be up to date, so before merging each PR, merge the
   latest base into its branch as "PR Branches" describes, push it under the
   same explicit approval, post a new `@codex review` for that head, and wait
   for green CI and a clean Codex review of it: a base merge also changes the
   reviewed head.
