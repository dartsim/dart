# Contributing To DART

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
remote SHA for existing refs. New refs choose the merge base that leaves the
fewest commits to scan from locally available remote branch heads and the
remote's default HEAD. Missing existing-ref bases and default HEAD objects are
fetched without changing refs or `FETCH_HEAD`; an empty remote or unrelated
history scans all local history.
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
`release-*` branch using `dart-backport-pr`. Release-specific packaging,
CI, and branch guidance may target that release branch directly. Start those
topic branches from its fetched remote ref without tracking it, as in the
`main` example above. Resolve the next release and exact open milestone using
[Release Management](release-management.md#release-target) from the fetched
target branch; that owner also defines stabilization cuts and release gates.
Dependency-minimization work must preserve installed
headers, package components, and downstream behavior unless a maintainer
explicitly approves a breaking change.

## PR Descriptions

Help a reviewer understand the problem, the resulting behavior, and whether
the evidence supports it. Use [the PR template](../../.github/PULL_REQUEST_TEMPLATE.md)
with **Summary** and **Testing** by default:

- Keep Summary to one or two sentences or a few short bullets. Explain why the
  change matters; use a concrete before/after example when it helps.
- Add rationale the diff cannot explain, a reviewer note, related work, or
  compatibility/migration guidance only when it affects the review. Material
  risks and evidence gaps must remain visible.
- In Testing, report meaningful checks beyond the PR's CI and relevant gaps,
  including their impact. When CI covers the relevant verification, one short
  statement is enough; keep routine local gate results in task evidence.

Scale detail to the change. Omit routine file inventories, tool activity,
repeated assurances, empty sections, and None/N/A placeholders. Discuss missing
measurements or untouched behavior only when they limit a claim or affect a
reviewer's decision. Rewrite the title and description around the final change
when scope changes.

Choose the smallest format that explains the claim; media is useful when it
shows something the text cannot:

| What the reviewer needs to understand | Preferred format |
| ------------------------------------ | ---------------- |
| Several distinct points | Short bullets |
| A small comparison | Compact table |
| Trends, scaling, or distributions | Plot |
| A static visual difference | Matched before/after images |
| Motion, timing, or interaction | Short video |

Place evidence beside the claim it supports. For performance claims, name the
baseline, workload, metric, and material limitations. Put long raw tables,
environment/configuration details, and reproduction commands in descriptive
`<details>` blocks; keep failures, uncertainty, and limiting caveats outside.

For claims about model/scene structure, dynamics, collision/contact,
constraints, simulation, or rendering, use `dart-verify-sim` and the
[simulation evidence and publication flow](../ai/verification.md#simulation-verification-route).
Its required evidence remains required even though other added sections are
optional. Show visible simulation changes with matched cameras; compute-only
improvements with unchanged motion use plots. Record why visual corroboration
is unavailable or irrelevant and name replacement evidence when that applies.

Write each paragraph and list item on one line, and separate paragraphs with a
blank line. Unlike Markdown files in the repository, where a single newline is
a soft break, GitHub renders every newline in PR descriptions, comments,
issues, and release notes as a line break, so text wrapped at a fixed width,
as this repository's docs are, breaks mid-sentence.

This guidance follows [Google's description advice](https://google.github.io/eng-practices/review/developer/cl-descriptions.html)
and [GitHub's review guidance](https://docs.github.com/en/pull-requests/concepts/helping-others-review-your-changes):
explain the purpose and result, preserve useful rationale, and identify where
reviewers should focus. A five-template spot check on 2026-10-12 found no
`Effect` heading in [React](https://raw.githubusercontent.com/facebook/react/main/.github/PULL_REQUEST_TEMPLATE.md),
[Kubernetes](https://raw.githubusercontent.com/kubernetes/kubernetes/master/.github/PULL_REQUEST_TEMPLATE.md),
[PyTorch](https://raw.githubusercontent.com/pytorch/pytorch/main/.github/PULL_REQUEST_TEMPLATE.md),
[VS Code](https://raw.githubusercontent.com/microsoft/vscode/main/.github/pull_request_template.md),
or [CPython](https://raw.githubusercontent.com/python/cpython/main/.github/PULL_REQUEST_TEMPLATE.md).
React and PyTorch use Summary; the others vary. This bounded sample supports
our heading choice, not a claim about GitHub-wide prevalence. Older DART PRs
are examples; this owner guidance takes precedence.

## PR Author Checklist

Complete the applicable actions before publication; keep the checklist and
routine decisions in author/task notes rather than copying them into the PR
description:

- [ ] Resolve and confirm the open release milestone from the fetched target's
  [Release Management](release-management.md#release-target); set it on the PR.
- [ ] Decide whether `CHANGELOG.md` needs an entry using
  [Changelog](changelog.md); prepare any required entry and finalize its PR link
  after publication.
- [ ] Add or update meaningful tests for new functionality or changed behavior,
  and run the required validation for the touched surface.
- [ ] Document new methods/classes and any changed user workflow.
- [ ] Add or update dartpy bindings when applicable.
