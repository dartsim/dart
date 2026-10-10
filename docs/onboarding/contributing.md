# Contributing On DART 6 Branches

Keep changes focused and branch from `main`, the development branch for the next
release:

```bash
git fetch origin main
git switch --no-track -c <type>/<topic> origin/main
```

Do not push directly to `main` or `release-*`. After explicit maintainer or
user approval, push topic branches with matching local and remote names:

```bash
branch=$(git branch --show-current)
git push -u origin "HEAD:${branch}"
```

Before every commit, run:

```bash
pixi run lint
```

Install the commit and push safety gates once per clone:

```bash
pixi run install-hooks
```

It installs a `pre-commit` git hook that runs the fast staged command below
and blocks staged whitespace or relevant AI-infrastructure drift:

```bash
pixi run python scripts/check_agent_hook.py --profile staged
```

It also runs `scripts/check_local_paths.py --staged` to scan staged names and
added lines. A managed `commit-msg` hook scans the message Git will publish,
using the parent Git command, cleanup options and configuration. If that
invocation cannot be read or parsed, it falls back to editor-template
instructions. Comments and literal scissors that Git retains are scanned;
without invocation or template evidence, every line is scanned.

The managed `pre-push` hook scans every updated ref's commit messages, file
names and added lines, including commits created by cherry-pick, rebase,
revert, `git am` and sequencer operations that skip commit hooks. It runs
`scripts/check_local_paths.py --commit-range <base>..<local sha>` using the
remote SHA for existing refs or the merge base with the remote's default branch
for new refs. Missing base objects are fetched without changing refs or
`FETCH_HEAD`; an empty remote or unrelated history scans all local history.
Deletions skip scanning. Older branches without checkers and unavailable
interpreters print a notice and skip scanning. If a checker is missing from
the worktree but exists in HEAD (commit hooks) or the push base (pre-push), the
hook runs its tracked version from a temporary file; recovery, lookup and scan
errors block the operation. Temporary files are removed when the hook exits.

Existing hooks are preserved as `<hook>.local` and chained; a foreign pre-push
hook receives the same ref-update stdin as the managed hook.
Emergency escape hatch:
`DART_SKIP_HOOKS=1 git commit ...` or `DART_SKIP_HOOKS=1 git push ...`.
Codex and Claude sessions also use tracked
PreToolUse hooks for agent-issued `git commit` calls before `install-hooks` has
been run. The PR Text workflow is the backstop for commits that use
`--no-verify` or otherwise skip local hooks; it scans each PR commit as well
as the PR title and body using the base branch's checker before merge. These
fast checks do not replace `pixi run lint`.

For C++ or Python changes, also run `pixi run build` and focused tests. For
Gazebo/gz-physics compatibility surfaces, run:

```bash
N=${DART_SAFE_JOBS:-$(python3 scripts/parallel_jobs.py)}
DART_PARALLEL_JOBS=$N CTEST_PARALLEL_LEVEL=$N pixi run -e gazebo test-gz
```

Target `main` for new features and fixes. Backport merged fixes to a
`release-6.*` branch using `dart-backport-pr`. Release-specific packaging,
CI, and branch guidance may target that release branch directly. Start those
topic branches from its fetched remote ref without tracking it, as in the
`main` example above. Resolve the next release and exact open milestone using
[Release Management](release-management.md#release-target) from the fetched
target branch; that owner also defines stabilization cuts and release gates.
Dependency-minimization work on DART 6 must preserve installed
headers, package components, and downstream behavior unless a maintainer
explicitly approves a breaking change.

## PR Descriptions

Text-only descriptions make a PR's effect and value slow to assess. Every PR
starts with a short `## Effect` section that a human reviewer can absorb in
about a minute:

- State what changes for users or which risk the change removes.
- Give the headline measured result, naming the baseline, workload, metric,
  and important limitations. If no measurement applies or was made, say so.
- Show the key plot or visual beside the claim when applicable.

Use [the PR template](../../.github/PULL_REQUEST_TEMPLATE.md) in this order:
Effect, Motivation / Problem, Changes / Key Changes, optional Before / After,
Testing, Breaking Changes, Related Issues / PRs. Put mechanism, method, and
full data after Effect; write for users and downstream maintainers who do not
know the implementation. Include compatibility, opt-in, or migration steps
when relevant.

For meaningful API, workflow, behavior, or performance changes, include a
concise Before / After comparison of user-visible outcomes. Small tables or
bullets are fine; when performance A/B results, sweeps, or physics metrics
would need many or large tables, plot the key comparisons as images and put
the raw tables in collapsed `<details>` blocks with descriptive `<summary>`
labels.

For visible simulation improvements (motion, contact, settling, stability, or
rendering), show before/after 3D highlights with the same camera. Follow the
existing [simulation evidence and publication flow](../ai/verification.md#simulation-verification-route)
through `dart-verify-sim`; compute-only improvements with unchanged motion use
plots instead.

In Testing, list checks that the PR's CI does not run (reporter-toolchain
reproductions, negative checks, hardware runs, or review passes), or explain why
none were needed. Keep routine local gate results in task evidence. Mark
non-applicable checklist items N/A with a reason, and name related work or None.
