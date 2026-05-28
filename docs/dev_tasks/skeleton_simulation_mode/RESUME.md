# Resume: Skeleton Simulation-Mode Lifecycle

## Last Session Summary

Captured the `SimulationMode { Design, Simulation }` lifecycle marker design
from the retired `refactor/ownership` branch as a tracked dev task. The
prototype branch was retired because the simple half of its scope (removing
`World::createSkeletonFromUri`) already landed on main, and the surviving
design half (the enum + parser plumbing) needs a placement decision before it
can land cleanly under the snake_case + experimental-World layout.

## Current Branch

`main` — no implementation work on the lifecycle marker has started after the
prototype.

## Immediate Next Step

Decide whether the lifecycle marker belongs on the legacy
`dart::dynamics::Skeleton` (and propagate through five parsers) or only on
the experimental World's frame storage (PLAN-050). Record the decision as
`01-placement.md` here. Phase 1 implementation is blocked on this.

## Context That Would Be Lost

- The prototype attached `mSimulationMode = SimulationMode::Design` directly
  to `Skeleton` and threaded the enum through `SkelParser`, `VskParser`,
  `MjcfParser`, `SdfParser`, and `UrdfParser` — a wide surface change.
- The dartsim editor (PLAN-101) is the immediate consumer; it needs
  edit-time skeletons that do not yet satisfy simulation invariants.
- `World::createSkeletonFromUri` is _gone_ on main; do not re-introduce it.
  Caller-side `dart::io::read*` + `World::addSkeleton` is the only entry
  point.

## How to Resume

```bash
git checkout main
git pull --ff-only origin main
git checkout -b feature/simulation-mode-placement

# Inspect prototype (while reflog still holds it):
git show 9cf76c0b44e --stat
git show 9cf76c0b44e:dart/dynamics/SimulationMode.hpp
git show 9cf76c0b44e:dart/dynamics/Skeleton.hpp | grep -nE "SimulationMode"
```

Then write `docs/dev_tasks/skeleton_simulation_mode/01-placement.md`
recording the legacy-vs-experimental decision and unblock Phase 1.
