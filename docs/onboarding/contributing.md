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

Target `main` (with the `DART 6.20.0` milestone); new patches land there, and
there is no maintenance branch. Backports to a `release-6.*` branch cut from a
release tag use `dart-backport-pr`. Dependency-minimization
work on DART 6.20 must preserve installed headers, package components, and
downstream behavior unless a maintainer explicitly approves a breaking change.

Use the matching DART 6.x release milestone for PRs.

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
