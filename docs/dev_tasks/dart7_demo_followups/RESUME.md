# Resume

## Handoff prompt

Paste this into a fresh session:

```text
Resume the follow-ups to DART's demo consolidation (decision D31b), which
landed on main on 2026-10-13. Start with AGENTS.md and docs/ai/principles.md,
then read docs/dev_tasks/dart7_demo_followups/README.md and RESUME.md and the
owner docs they link. Treat them as a snapshot: verify each stated fact
against the live repository and GitHub before acting.

Work through README "Remaining work" in order:
1. Check main's CI for the listed merge commits; fix each failure at its root
   cause in a focused follow-up PR with a regression test.
2. Fix the stale ssik_ik_gui solved marker with a regression test.
3. Run the Windows launch tasks if a Windows host is available, or record
   that they remain unverified.
4. Report the GitHub cleanup items to the maintainer; branch deletion, thread
   resolution and stack changes need explicit approval.

Follow docs/onboarding/ai-tools.md "PR Lifecycle". When every item is done,
promote anything durable to its owner doc and delete this folder.
```

## Current state (2026-10-13)

- `main` at `e378f985f` contains the whole consolidation. No consolidation PR
  is open.
- The follow-ups in the README are not started. When this was written, the
  post-merge CI runs on `main` were still queued.
- Local branches from the consolidation remain in the clone that did the work.
  Every one landed through a squash merge, so delete them only after checking
  that `main` has their content.

## Immediate next steps

1. Inspect main's CI for each merge commit, for example
   `gh api repos/dartsim/dart/commits/<sha>/check-runs --jq '.check_runs[] | select(.conclusion=="failure") | .name'`.
   Reproduce a failure locally before changing code; see
   [CI/CD](../../onboarding/ci-cd.md).
2. Fix the ssik marker on a `fix/<topic>` branch from `origin/main`. ssik is
   optional: install it with `pixi run install-ssik` to run the scene and its
   tests.
3. Open each fix as a draft PR with one `@codex review`, as the lifecycle
   describes.

## Recovery notes

- **Parity against the old C++ app.** `examples/demos` is gone from `main`.
  To compare a Python scene with its C++ original, build `dart-demos` from a
  worktree at `a1dfa943b`, the last `main` commit that still has it, and use
  its `--debug-print-world` and `--shot` options.
- **Squash-merged stacks.** After a parent PR is squash-merged, its children
  conflict with the squash commit even when nothing differs. If the new base
  tree equals a commit the branch already contains (compare
  `git rev-parse <commit>^{tree}`), merge it with `git merge -s ours`.
  Otherwise compute the result with
  `git merge-tree --write-tree --merge-base=<last merged base head> HEAD <base>`
  and commit that tree with both parents.
- **`gh stack`.** Stacked PRs cannot be merged with administrator bypass, and
  merged members cannot be unstacked. `gh stack link` retargets a chain's
  bottom PR to `main` when the chain starts on a helper branch.
- **Long background jobs.** Run each in its own process group and record its
  PID; stop only that group. Never signal a parent PID: orphaned shells are
  reparented to the user's session manager.

## Verification commands

```bash
pixi run lint && pixi run check-lint
pixi run build && pixi run test
pixi run build-py-dev && pixi run test-py
pixi run test-gui
pixi run cycle-demos
pixi run demos -- --list-scenes
pixi run capture sleeping
```
