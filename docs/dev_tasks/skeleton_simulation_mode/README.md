# Skeleton Simulation-Mode Lifecycle — Dev Task

## Current Status

- [x] Phase 0: Captured prototype reference and design intent.
- [ ] Phase 1: Decide whether the lifecycle marker lives on the legacy
      `dart::dynamics::Skeleton` / `BodyNode` or only on the experimental
      World's frame storage (PLAN-050)
- [ ] Phase 2: Implement the chosen surface with snake_case headers, parser
      plumbing, and dartpy bindings
- [ ] Phase 3: Wire dartsim editor (PLAN-101) so edit-time selection,
      undo/redo, and project save/load operate on `Design` skeletons; promote
      to `Simulation` only when the editor enters Run mode

## Goal

Give every Skeleton (and BodyNode) a first-class lifecycle marker that
distinguishes an object being edited from one being actively simulated by a
World, so the dartsim editor (PLAN-101) can author skeletons without paying
the full simulation invariants and so World can reject ambiguous ownership
transitions at the API boundary.

## Non-Goals (for early phases)

- Re-introducing `World::createSkeletonFromUri` — that helper was already
  removed in `main` (see commits referenced below); caller-side
  `dart::io::read*` + `World::addSkeleton` stays the only entry point.
- Multi-World ownership or shared skeletons across Worlds.
- Re-implementing edit-time picking or transforms on top of this enum;
  that work belongs to the dartsim editor task
  (`docs/dev_tasks/dartsim_workbench_completion/`).

## Key Decisions

- **Surface**: an `enum class SimulationMode { Design, Simulation }` reachable
  from `Skeleton::getSimulationMode()` / `setSimulationMode()` /
  `isSimulationMode()` is the prototype's proposed API. Default state is
  `Design`; `World::addSkeleton` flips to `Simulation` and `removeSkeleton`
  flips back. The transition is the API boundary that enforces ownership.
- **Naming**: snake_case header `dart/dynamics/simulation_mode.hpp` (the
  prototype's `dart/dynamics/SimulationMode.hpp` predates the snake_case
  migration; `docs/onboarding/code-style.md` is the rule).
- **Parser plumbing**: parsers (`SkelParser`, `VskParser`, `MjcfParser`,
  `SdfParser`, `UrdfParser`) construct skeletons in `Design` mode so the
  result can be inspected or edited before being added to a World. The
  prototype already threaded the enum through all five parsers.
- **Open question, blocks Phase 1**: the DART 7 World (PLAN-050,
  complete on main) stores frames under
  `dart/simulation/frame/`, not `dart::dynamics::Skeleton`. The
  lifecycle marker may belong only on the experimental side, with the legacy
  Skeleton API left untouched until DART 8 removes it. This decision must be
  made before writing snake_case headers.

## Prototype Reference

The original prototype lived on the now-deleted `refactor/ownership` branch.
Commit SHAs (recoverable from reflog and `git fsck --unreachable` until
natural expiry):

- `9cf76c0b44e Refine skeleton ownership and tooling` — first cut that
  introduced the enum.
- `581d1ee60e5 Add World::createSkeletonFromUri` and
  `ce7feea4302 Keep World::addSkeleton available temporarily` — the
  intermediate ownership tightening; the second is the rollback that aligned
  with the eventual removal-only design.
- `d1b6c14e060 Remove World::createSkeletonFromUri` — the surviving half;
  this matches main's current state and is the _non-goal_ anchor.

Files of interest in the prototype: `dart/dynamics/SimulationMode.hpp`
(20-line enum), `dart/dynamics/{Skeleton,BodyNode}.{cpp,hpp}` and their
`detail/` private layout, all five parser files under `dart/utils/`, the
dartpy bindings in `python/dartpy/{dynamics/skeleton,simulation/world}.cpp`,
and the `docs/onboarding/self-collision.md` note that paired with the
removal.

## Immediate Next Steps

1. Before any Phase 1 implementation, resolve the legacy-vs-experimental
   placement question with a short design note here under the
   `01-placement.md` filename, or fold it into the DART 7 World
   onboarding doc if the answer is "experimental only".

## Verification Gates

- Phase 2: `pixi run lint`, `pixi run test-unit`, focused parser tests,
  `pixi run check-api-boundaries`.
- Phase 3: dartsim engine + headless smoke prove that opening a project file
  produces `Design` skeletons and that pressing Run promotes them to
  `Simulation` without leaking the enum publicly through the editor surface.
