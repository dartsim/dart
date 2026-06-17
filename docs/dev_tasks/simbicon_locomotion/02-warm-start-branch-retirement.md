# Warm-Start Checkpoint Retirement

This note extracts the durable state from
`feature/dart7-unified-contact-warm-start` so that branch can be removed after
the retirement note lands on `main` and the maintainer approves deletion.

## Branch Audit

- Local and remote branch: `feature/dart7-unified-contact-warm-start`.
- Pull request: none found for this branch.
- Relationship to `origin/main` at audit time: 37 commits behind and 295 commits
  ahead.
- Diff size at audit time: hundreds of files, including broad DART 7 contact,
  multibody, actuator, solver, SIMBICON diagnostic, and planning edits.

Do not merge or rebase this branch into `main`. Treat it as an old checkpoint,
not as an integration branch. Any remaining useful code from it should be
reimplemented as fresh, bounded changes from current `main` with tests.

## Already Extracted

- PR [#3043](https://github.com/dartsim/dart/pull/3043) landed the small,
  reusable SIMBICON pose-window comparison utility from the checkpoint branch.
  The utility reports per-foot transform translation, trace-vertical position,
  local z-axis tilt relative to trace vertical, and stance/swing aliases when
  the sampled controller state identifies the active limb.
- Current SIMBICON follow-up work should use the landed utility on `main`; it
  should not ask agents to check out the checkpoint branch for normal work.

## Durable SIMBICON Findings

The checkpoint branch contains many probe attempts. The useful result is mostly
negative evidence and the next diagnostic direction:

- Do not continue global native surface tolerance or seed-depth changes as a
  keeper fix. The best positive native-surface-tolerance probes still fell, and
  zero seed-depth variants also failed.
- Do not return to blunt state-0/state-1 swing `hpx` or `hpy` target clamps
  without new evidence. Local right swing hip-roll and hip-pitch cap probes
  reached the same late fall mode or worsened it.
- Do not continue reactive late state-2 support-hold or hip-pitch clamps as the
  primary fix. Behind-COM support is a real symptom, but these late reactive
  probes were too late or destabilizing.
- The strongest remaining SIMBICON lead is state-0 stance reaction and support
  geometry around steps `3970`, `4000`, `4070`, and `4100`. Compare DART 6 and
  DART 7 stance-foot pose, stance-foot tilt, support-row counts, and support
  patch location before changing controller or contact parameters again.
- Contact-row evidence points at one-sided or sparse support packets during key
  handoff windows, not a simple "DART 7 has no contacts" failure. Some windows
  have many rows, but the support patch is poorly placed relative to COM or
  collapses before the next controller state can recover.

## Broader DART 7 Work

The checkpoint branch also carries broad DART 7 contact/warm-start and
multibody work. That code is stale relative to current `main` and should not be
ported wholesale. Use the maintained plans instead:

- `docs/plans/080-rigid-body-dynamics-solver.md` for rigid-body solver and
  contact-solver follow-up slices.
- `docs/plans/091-architecture-hardening.md` for solver architecture hardening
  and work-packet sequencing.
- `docs/dev_tasks/rigid_body_dynamics_solver/` for active implementation state.

## Removal Criteria

`feature/dart7-unified-contact-warm-start` is removable after:

1. This retirement note and the updated SIMBICON resume pointers land on
   `main`.
2. Any active agent handoff points at current `main`, PR #3043, and this note
   instead of the checkpoint branch.
3. The maintainer explicitly approves deleting the local and remote checkpoint
   branch.
