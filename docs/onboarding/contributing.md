# Contributing On The DART 6.20 Branch

Keep changes focused and branch from `main`, the development branch for the next
release (currently DART 6.20):

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

Target `main` (with the `DART 6.20.0` milestone); new patches land there, and
there is no maintenance branch. Backports to a `release-6.*` branch cut from a
release tag use `dart-backport-pr`. Dependency-minimization
work on DART 6.20 must preserve installed headers, package components, and
downstream behavior unless a maintainer explicitly approves a breaking change.

Use the matching DART 6.x release milestone for PRs.
