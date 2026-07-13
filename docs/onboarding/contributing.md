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

Install the fast staged safety gate once per clone:

```bash
pixi run install-hooks
```

It installs a `pre-commit` git hook that runs the fast staged command below and
blocks the commit on staged whitespace or relevant AI-infrastructure drift:

```bash
pixi run python scripts/check_agent_hook.py --profile staged
```

An existing `pre-commit` hook is preserved as `pre-commit.local` and chained.
Emergency escape hatch:
`DART_SKIP_HOOKS=1 git commit ...`. Codex and Claude sessions also use tracked
PreToolUse hooks for agent-issued `git commit` calls before `install-hooks` has
been run. These fast checks do not replace `pixi run lint`.

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
