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

Install the commit safety gates once per clone:

```bash
pixi run install-hooks
```

It installs a `pre-commit` git hook that runs the fast staged command below and
blocks the commit on staged whitespace, local file names or text, or relevant
AI-infrastructure drift. A managed `commit-msg` hook also scans the commit
message for local paths. Git's "Lines starting with" editor template instruction
identifies the comment string whose lines Git strips; the scan skips those
lines. The "Do not modify or remove the line above" instruction immediately
after a matching scissors line only enables stopping there; comment lines above
the cut are scanned because Git keeps them. Without a template instruction (`-m` or
`-F`), all lines are scanned,
including hash-prefixed, status-shaped and scissors-shaped lines. Only an editor
template makes matching scissors end the scan before a verbose diff:

```bash
pixi run python scripts/check_agent_hook.py --profile staged
```

Existing hooks are preserved as `pre-commit.local` or `commit-msg.local` and
chained.
Emergency escape hatch:
`DART_SKIP_HOOKS=1 git commit ...`. Codex and Claude sessions also use tracked
PreToolUse hooks for agent-issued `git commit` calls before `install-hooks` has
been run. When verification is bypassed with `--no-verify`/`-n` (including
accepted abbreviations), hooks are overridden, or managed hooks are
missing/outdated, the agent guard checks every
commit split by its shell tokenizer, scans supplied `-m`/`--message`, readable
`-F`/`--file` messages and trailers, and runs the staged gate once. It blocks
stdin, reused (`-C`/`-c`/`--reuse-message`/`--reedit-message`), autosquash
(`--fixup`/`--squash`, including `amend:`/`reword:`), and editor-only
messages when hooks cannot enforce them: supply `-m` or `-F <file>`, or let the
managed hooks run.
When managed hooks will not run, the guard also blocks commit-time staging
(`-a`/`--all`, `-i`/`--include`, `-o`/`--only`, or pathspecs); stage the files
first or let the hooks run so the staged scan can inspect all committed content.
Multiple commits without managed hooks are blocked when any earlier command can
change files or the index. Split the commits into separate tool calls or let the
hooks run; read-only Git commands between commits remain supported.
These fast checks do not replace `pixi run lint`.

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
