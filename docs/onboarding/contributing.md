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

The `pre-commit` git hook runs the fast staged command below. It blocks staged
whitespace or relevant AI-infrastructure drift:

```bash
pixi run python scripts/check_agent_hook.py --profile staged
```

The hooks that `pixi run install-hooks` installs:

- `pre-commit`: also runs `scripts/check_local_paths.py --staged`. That scan
  checks staged names and added lines.
- `commit-msg`: runs
  `scripts/check_local_paths.py --commit-msg-file "$1" --git-pid "$PPID"`. It
  scans the message that Git will publish.
- `pre-push`: scans every updated ref (see below).

How the `commit-msg` hook reads the message:

- The parent Git command, its cleanup options, and its configuration decide
  which comments, scissors, and verbose diffs Git removes before publication.
- If the hook cannot read or parse that invocation, it falls back to the
  editor-template instructions.
- Without invocation or template evidence, it scans every line.
- It scans the comments and literal scissors that Git retains.

How the `pre-push` hook works:

- It reads Git's `<local ref> <local sha> <remote ref> <remote sha>` stdin.
- For each updated ref, it runs
  `scripts/check_local_paths.py --commit-range <base>..<local sha>`.
- It scans every commit's message, file names, and added lines. This includes
  commits from cherry-pick, rebase, revert, `git am`, imports, aliases, and
  sequencer operations that skip commit hooks, and from alternate
  `GIT_INDEX_FILE` indexes.
- For an existing ref, `<base>` is the remote SHA.
- A new ref scans only the commits that no branch head or default HEAD
  advertised by the destination reaches. The hook fetches a head this clone
  lacks only when the clone tracks that branch.
- A new ref scans those commits as one range when a single published base
  covers them. Otherwise it scans one commit at a time.
- A merge commit is still compared with its first parent. The scan can report
  content that a merged branch already published.
- The hook fetches missing existing-ref bases and default HEAD objects
  without changing refs or `FETCH_HEAD`.
- An empty remote or unrelated history scans all local history.
- A deletion skips scanning.
- Findings and lookup or scan errors block the push.

Hook environment:

- Each hook selects a compatible interpreter. The `commit-msg` hook, the
  `pre-push` hook, and the `pre-commit` staged path scan need Python 3.9+ and
  the standard library only.
- The `pre-commit` AI gate reselects Python with `tomllib` (3.11+) and the
  Pixi-supplied Markdown parser. If none exists, it falls back to the staged
  whitespace check, and the path scan still runs.
- An older branch without the checkers, or a missing compatible interpreter,
  prints a notice and skips the message and `pre-push` scans. The `pre-commit`
  hook keeps its staged whitespace fallback.
- If a checker is missing from the worktree but exists in HEAD (commit hooks)
  or the push base (`pre-push`), the hook runs its tracked version from a
  temporary file. Recovery, lookup, and scan errors block the operation. The
  hook removes the temporary files when it exits.
- `pixi run install-hooks` moves an existing hook to `<hook>.local`. The
  managed hook chains it when it is executable. A foreign `pre-push` hook
  receives the same ref-update stdin as the managed hook, and its failure
  blocks the push.
- All hooks share the foreign-hook chaining, `DART_SKIP_HOOKS=1`, and
  `DART_HOOK_DRY_RUN=1`.
- Emergency escape hatch: `DART_SKIP_HOOKS=1 git commit ...` or
  `DART_SKIP_HOOKS=1 git push ...`.

Codex and Claude sessions also use tracked PreToolUse hooks for agent-issued
`git commit` calls. These hooks apply before `install-hooks` has been run.

The PR Text workflow is the backstop for commits that use `--no-verify` or
otherwise skip local hooks.

- It scans each PR commit's messages, names, and added lines.
- It scans the PR title and body.
- It uses the base branch's checker before merge.
- GitHub may omit this workflow for SHA-like branch names. Those PRs rely on
  CI's tracked-file scan and review.

These fast checks do not replace `pixi run lint`.

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

## Writing Style

Write so that a reader can understand and act quickly. For a PR or a report,
the reader is a reviewer who must decide. These rules adapt the writing rules
of [ASD-STE100](https://www.asd-ste100.org/) Simplified Technical English.
DART does not use the STE dictionary: use the names in the code and the terms
in [AI Terminology](../ai/terminology.md).

- Start with the result, the decision you need, or the risk. Then give the
  method.
- Write one topic in each sentence and one instruction in each step. Put a
  condition before its instruction: "If the build fails, run ...".
- Aim for 20 words or fewer in an instruction and 25 or fewer in a
  description. Use a vertical list for steps, options, or many conditions.
- Give each paragraph one topic and six sentences or fewer.
- Use the active voice and name the actor: the agent, the maintainer, CI, or a
  hook.
- Use the same word for the same thing.
- Keep articles such as "the" and "a". Do not stack more than three nouns,
  except in exact names of files, commands, APIs, and checker markers.
- Start a warning with the command, then give the reason: "Do not hand-edit
  generated adapters in `.agents/skills/`. The sync script overwrites them."
- Keep code, commands, identifiers, paths, and quoted output exact.

Use a diagram when the point is a structure, a flow, or a state change. In
Markdown that GitHub renders, draw a small `mermaid` code block beside the
prose it explains, and change both together. Read the Docs does not render
Mermaid, so use an image in `docs/readthedocs/_static/` there. A diagram
explains a claim; it does not prove it.

Apply these rules to text that you write or substantially change. Put
style-only cleanup of other text in its own PR. In AI-harness files, the
checkers and their tests match some phrases verbatim, often on one line. Before
you reword or re-wrap such text, search `scripts/` and `tests/` for the phrase.
Keep a matched phrase unchanged and on one line, or change the checker and its
tests in the same PR.

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
when scope changes. Write both as [Writing Style](#writing-style) describes.

Choose the smallest format that explains the claim; media is useful when it
shows something the text cannot:

| What the reviewer needs to understand | Preferred format |
| ------------------------------------ | ---------------- |
| Several distinct points | Short bullets |
| A small comparison | Compact table |
| A structure, flow, or state change | Small Mermaid diagram |
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
- [ ] Decide whether the change needs a changelog entry using
  [Changelog](changelog.md); on `main`, add it as a `changelog.d/` fragment, and
  finalize its PR link after publication.
- [ ] Add or update meaningful tests for new functionality or changed behavior,
  and run the required validation for the touched surface.
- [ ] Document new methods/classes and any changed user workflow.
- [ ] Add or update dartpy bindings when applicable.
