# AI Principles

These principles apply to AI-assisted DART work on development,
stabilization, and maintenance branches.
Keep this file short: it spends always-loaded agent context, so put procedures
and compatibility detail in the owner docs named by `AGENTS.md`.

## Evidence First

- Inspect the current branch, worktree, and relevant source files before
  editing.
- Treat live GitHub state and current command output as authoritative.
- Do not rely on stale branch notes when a current checkout or PR can be
  inspected.
- Before substantial or consequential work, read enough development-branch
  context to understand the real invariant: `AGENTS.md`,
  `docs/information-architecture.md`, owner docs, affected modules and call
  paths, current release/PR state, and the verification bar. Use that context to
  choose the smallest compatibility-preserving change, not an ad hoc local
  patch or special case. Keep context proportionate for tiny fixes.
- Before a non-trivial fix, surface your unknowns rather than coding a guess:
  your plan is a map, not the territory of the real code. Convert consequential
  unknowns into knowns first — reproduce the bug, read the affected code, or get
  an independent blind-spot review — instead of discovering them mid-change.
- Back consequential decisions with proportionate evidence: code inspection,
  logs, focused tests, benchmarks, source research, or prototypes according to
  the claim. Do not decide major workflow or compatibility choices from
  intuition when direct evidence is practical.
- For unexpected in-scope failures, reproduce the smallest failing case, fix the
  underlying cause, and add regression coverage. Preserve the real invariant at
  the right owner doc or module boundary; do not suppress logs, loosen checks,
  skip cases, or route around symptoms. For pre-existing or external failures,
  classify them with evidence and report or park them without weakening gates.

## Compatibility First

- `main` is the Python-first DART 7 line: its Python API may change as the
  DART 7 design doc records, while the C++ API that Gazebo/gz-physics and
  gz-sim use, installed headers, and package components stay compatible. Any
  other breaking change needs explicit maintainer acceptance. `release-6.20`
  is a DART 6 compatibility lane.
- dartpy uses the approved nanobind binder exclusively on `main`; release
  branches keep the binder they shipped with. Binder contracts live in
  `docs/onboarding/python-bindings.md`.
- Target `main`, where new patches land; backports to a `release-*`
  stabilization or maintenance branch use `dart-backport-pr`. A stabilization
  branch may be cut before the release tag (see
  `docs/onboarding/release-management.md`).

## Public Text

- Published text is permanent and public; private plans and machine layouts
  are not. Describe private planning in prose or link public PRs/issues, and
  check files, commit messages, and PR text with `scripts/check_local_paths.py`
  before publication.

## Reviewable Output

- Agents do more of the work, and the maintainer checks it. Write reports,
  handoffs, PR text, and docs that a maintainer can check quickly. Lead with
  the result or the decision you need, and put evidence beside each claim. Use
  a table, a diagram, a plot, or a capture when it is clearer than prose.
  `docs/onboarding/contributing.md` § "Writing Style" owns the rules.
- Ask the maintainer one question at a time. Give its context and evidence,
  the options with their tradeoffs, and your recommendation first.
- Build a throwaway script, plot, or local HTML page when it makes a result
  easier to check. Keep it out of Git (for example, under `build/`, not in
  `docs/dev_tasks/`) and add no project dependency for it. It never proves a
  claim alone. Publishing it follows the approval rules below.

## Approval Boundaries

- GitHub mutations, pushes, PR creation or updates, branch deletion, CI reruns,
  and review-thread mutations require explicit maintainer/user approval.
- Never reply to AI-generated review comments. Make local fixes silently and
  request a fresh review only after an approved push.

## Scope Control

- Keep PRs focused. For dependency minimization, prefer one dependency or one
  vendored tree per PR unless a shared mechanical step is required.
- Use `docs/dev_tasks/<task>/` for multi-session state. Promote durable facts to
  the owner selected by `docs/information-architecture.md` before retiring a
  task folder.
