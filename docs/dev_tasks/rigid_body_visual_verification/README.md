# Rigid Body Visual Verification - Handoff Dev Task

## Current Status

This folder is a temporary handoff checkpoint for an interrupted session. The
implementation and durable documentation for the rigid-body visual verification
packet were already moved to stable homes, and the previous dev-task folder was
retired in commit `2862c337291`.

- Branch: `feature/rigid-body-gui-visual-verification`
- Last known local state before this handoff doc: clean worktree, ahead of
  `origin/feature/rigid-body-gui-visual-verification` by 7 commits through
  `2862c337291`.
- This handoff doc should be the only additional work after that state.
- Per explicit user instruction, no further verification was run while creating
  this handoff.

## Durable Sources Of Truth

- `docs/plans/103-examples-strategy.md` records the PLAN-103 py-demos status.
- `docs/plans/103-examples-strategy/rigid-body-visual-verification.md` owns the
  36-row rigid workflow matrix, coverage evidence, deferred API gaps, and
  related packets.
- `python/examples/demos/README.md` documents the user-facing rigid workflow and
  capture commands.
- `python/examples/demos/registry.py` and
  `python/examples/demos/scenes/rigid_*` are the code-level source of truth for
  available scenes and hooks.
- `python/tests/integration/test_demos_cycle.py` contains the documentation and
  workflow guards.

## Recent Commits To Preserve

```text
2862c337291 Retire rigid visual verification dev task
aa2a7092fb5 Add executor equivalence replay timeline
eb949071f4d Link root README to rigid workflow capture docs
f6fc7ec7f43 Route source checkout to Python rigid demos
7caa0f6ab6e Clarify rigid comparison axes and capture evidence
35ec0a8610c Number rigid workflow README quick path
325280d4483 Harden rigid workflow search and evidence docs
f22db3b7751 Record rigid capture ownership visual evidence
```

## Validation Already Performed Before Stop Request

Do not rerun these just to refresh this handoff. The user explicitly asked for
handoff only, without further verification.

- Focused executor replay-timeline pytest passed:
  `python/tests/integration/test_demos_cycle.py::test_rigid_executor_equivalence_keeps_parallel_rollout_matched`
- Focused rigid workflow documentation guards passed: 5 tests in
  `python/tests/integration/test_demos_cycle.py`
- `pixi run lint` passed before the existing commits above.
- `git diff --check` passed before the existing commits above.
- `pixi run build` passed before the existing commits above.
- `pixi run test-py` passed with `956 passed, 10 skipped`.

## Known Open State

- `pixi run py-demos -- --cycle-scenes` was attempted after the cleanup commit
  and then stopped after roughly 8 minutes with no terminal completion. It
  emitted Mesa/DRI GL warnings and a Filament handle allocator slow-path warning.
  Treat this gate as inconclusive, not passed.
- No push or PR creation had happened before this handoff doc was written.
- A fresh session should verify whether this handoff commit has since been
  pushed before deciding next steps.

## Immediate Next Steps

1. Do not start new implementation from this folder.
2. Check the current branch and remote state with `git status -sb` and
   `git log --oneline -n 10`.
3. If the branch is pushed and the user wants PR work, open or update the PR
   using the normal DART PR rules.
4. If local GUI validation is required before PR work, investigate the
   `py-demos -- --cycle-scenes` long-running behavior as a separate, bounded
   task.
5. Once the handoff is no longer needed, delete this temporary folder again and
   keep durable rigid workflow context in the PLAN-103 sidecar.
