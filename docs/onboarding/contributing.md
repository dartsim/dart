# Contributing On The DART 6.20 Branch

Keep changes focused and branch from the target release branch:

```bash
git fetch origin release-6.20
git switch --no-track -c <type>/<topic> origin/release-6.20
```

Do not push directly to `release-*`. After explicit maintainer or user approval,
push topic branches with matching local and remote names:

```bash
branch=$(git branch --show-current)
git push -u origin "HEAD:${branch}"
```

Before every commit, run:

```bash
pixi run lint
```

Make the lint gate automatic once per clone:

```bash
pixi run install-hooks
```

It installs a `pre-commit` git hook that runs `pixi run check-lint-quick`
before every commit and blocks the commit if it fails. An existing `pre-commit`
hook is preserved as `pre-commit.local` and chained. Emergency escape hatch:
`DART_SKIP_HOOKS=1 git commit ...`. Claude Code sessions are additionally
guarded via the tracked `.claude/settings.json` hook, which runs the same gate
for agent-issued `git commit` calls even before `install-hooks` has been run.

For C++ or Python changes, also run `pixi run build` and focused tests. For
Gazebo/gz-physics compatibility surfaces, run:

```bash
N=${DART_SAFE_JOBS:-$(python3 scripts/parallel_jobs.py)}
DART_PARALLEL_JOBS=$N CTEST_PARALLEL_LEVEL=$N pixi run -e gazebo test-gz
```

Bug fixes that apply to both DART 6 and DART 7 need both a release-branch PR
and a `main` PR. Dependency-minimization work on DART 6.20 must preserve
installed headers, package components, and downstream behavior unless a
maintainer explicitly approves a breaking change.

Use the branch-matching DART 6.x milestone for release-branch PRs.
