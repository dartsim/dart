# Native Collision Visual Verification - Dev Task

## Current Status

- [x] Phase 0: Create PLAN-037 and this implementation tracker.
- [x] Phase 1: Define the pair registry and fixture model.
- [ ] Phase 2: Extend `examples/collision_sandbox` into the consolidated
      pair-by-pair debugger.
- [ ] Phase 3: Add contact/manifold/depth visualization and raw-value panels.
- [ ] Phase 4: Add broad-phase debug snapshots and overlays.
- [ ] Phase 5: Add deterministic headless visual smoke and focused tests.

## Goal

Build a dedicated native-collision GUI example for visual verification of every
supported collision pair, explicit unsupported placeholders, runtime shape
parameters, object posing, contact/manifold data, and broad-phase diagnostics.

## Non-Goals For Early Phases

- Do not restore FCL, Bullet, or ODE as runtime GUI dependencies.
- Do not make a stable public GUI API from the example-local controls.
- Do not rebase onto a Filament feature branch unless the chosen renderer path
  requires it; the current main-branch VSG collision sandbox is the starting
  point.
- Do not use visual smoke tests as a substitute for focused unit tests on new
  collision or broad-phase APIs.

## Existing Starting Points

- `examples/collision_sandbox` already has VSG/ImGui controls for primitive
  shape scenes, contacts, filtering, distance, raycast, CCD, picking, AABBs,
  and headless screenshot output.
- `examples/collision_viz` is a static demonstration of native collision
  overlays and is useful as smoke-test reference material.
- `dart/gui/vsg/collision_scene_builder.*` already draws collision objects,
  contact points/normals, AABBs, distance results, raycasts, and sphere casts.
- `dart/collision/native/collision_world.hpp` exposes batch snapshots and the
  selected broad-phase implementation, but the current `BroadPhaseSnapshot`
  records only candidate pairs and object count.
- `dart/collision/native/broad_phase/aabb_tree.hpp` keeps tree nodes private,
  so BVH/AABB-tree rendering needs an explicit copied debug snapshot rather
  than direct example access to internals.
- `examples/collision_sandbox/pair_registry.*` now owns the current
  pair-case registry, shape fixture factory, default pair poses, and pair
  classification for the dropdown and tests.

## Key Decisions

- Use PLAN-037 as the durable roadmap owner; use this folder only for active
  multi-session implementation tracking.
- Start from `examples/collision_sandbox` instead of creating another example,
  because it already has the right VSG/ImGui viewer, picking, headless capture,
  and native-collision runtime path.
- Treat pair coverage as data. The pair registry should drive the dropdown,
  UI status, headless smoke coverage, and tests so future native pair changes
  cannot silently drift away from the visual verifier.
- Keep `examples/collision_sandbox/pair_registry.*` as the source of truth for
  visual pair cases while the example remains example-local.
- Broad-phase visualization should use copied debug snapshots. That keeps tree
  internals inspectable without exposing mutable implementation state to the
  GUI example.

## Immediate Next Steps

1. Replace the remaining mode-oriented contact UI with richer object A/object B
   shape-parameter inspectors instead of the current uniform scale controls.
2. Add manifold-aware rendering in `dart/gui/vsg/collision_scene_builder.*`
   so the scene groups raw contact points by manifold and visualizes depth.
3. Design the broad-phase debug snapshot structure and add tests before using
   it in the GUI overlay.
4. Expand the headless smoke into a deterministic sweep over representative
   pair and broad-phase overlay presets.

## Completion Checklist

- [ ] The example covers every supported native pair or shows an explicit
      placeholder for unsupported/not-yet-visualized rows.
- [ ] Users can move and pose both objects and adjust shape parameters at
      runtime.
- [ ] Contact overlays and panels show raw point, normal, depth, and manifold
      membership.
- [ ] Broad-phase overlays show object AABBs, candidate pairs, and AABB tree
      node/edge visualization; Morton-code or spatial-order visualization is
      present when the selected broad phase uses it.
- [ ] Focused tests cover pair-registry invariants and any new broad-phase
      debug snapshot API.
- [ ] A headless smoke proves deterministic rendering of pair and broad-phase
      overlays.
- [ ] Durable lessons are promoted to the appropriate plan/onboarding docs and
      this folder is removed in the completion PR.
