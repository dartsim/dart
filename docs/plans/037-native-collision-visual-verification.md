# Native Collision Visual Verification

- Operating state:
  [`PLAN-037` in `dashboard.md`](dashboard.md#plan-037-native-collision-visual-verification)
- Outcome: a dedicated native-collision GUI example lets maintainers inspect
  every supported collision pair and every explicit unsupported placeholder with
  runtime shape parameters, object posing, contact/manifold overlays, and
  broad-phase visualization.
- Current evidence: native collision feature and performance coverage is
  complete in
  [`035-native-collision-dashboard.md`](035-native-collision-dashboard.md);
  `examples/collision_sandbox` already provides a VSG/ImGui native collision
  sandbox with primitive scenes, contacts, AABBs, distance, raycast, CCD, and
  picking; `dart/gui/vsg/collision_scene_builder.*` already draws collision
  objects, contact points/normals, AABBs, distance results, raycasts, and
  sphere casts.

## Scope

This follow-up hardens verification beyond unit tests, benchmarks, and
end-to-end physics simulations. It should make incorrect contact geometry,
manifold grouping, pair dispatch, or broad-phase candidate generation visible
from a single debug example.

The active implementation tracker is
[`docs/dev_tasks/native_collision_visual_verification/`](../dev_tasks/native_collision_visual_verification/).
Delete that task folder in the completion PR after durable results move to
code, examples, tests, this plan, and any needed onboarding notes.

## Requirements

- Provide a pair selector that covers the native narrow-phase matrix. Supported
  pairs should run live; missing or intentionally unsupported pairs should be
  selectable placeholders with clear disabled status instead of disappearing
  from the matrix.
- Make both shapes configurable at runtime: dimensions, pose, orientation,
  collision mask/group where relevant, and contact query options such as maximum
  contacts.
- Allow moving or posing either object independently, not only a single canned
  offset.
- Render collision state with shape tinting, raw contact point markers, contact
  normal arrows, penetration depth labels or scaled depth segments, and contact
  manifold grouping.
- Render broad-phase state: object AABBs, candidate pairs, AABB tree or BVH
  nodes, parent/child tree connections, and Morton-code or spatial-order
  visualization when that ordering is part of the active broad-phase path.
- Keep reference engines out of the runtime path. FCL, Bullet, and ODE remain
  comparison-only test/benchmark dependencies.

## Workstreams

1. **Pair registry and fixtures**
   - Create a data-driven registry for the visual example from the current
     native pair coverage: direct primitive pairs, plane pairs, cylinder pairs,
     convex/mesh fallback paths, compound recursion, SDF distance cases, and
     placeholders for explicit unsupported or not-yet-visualized rows.
   - Prefer a single registry that can feed the ImGui dropdown, headless smoke
     sweeps, and focused tests so the example cannot silently omit a pair.
2. **Runtime controls**
   - Replace one-off sliders with object A/object B inspectors for shape
     parameters, translation, rotation, broad-phase type, collision filtering,
     contact options, and reset presets.
   - Preserve deterministic presets for headless smoke tests.
3. **Contact and manifold visualization**
   - Extend the VSG collision scene builder to render per-manifold groups,
     contact point identity, normal direction, penetration depth, and textual or
     panel-side raw values.
   - Use tinting to separate non-colliding, colliding, selected, filtered, and
     unsupported states.
4. **Broad-phase diagnostics**
   - Add a native broad-phase debug snapshot API if existing snapshots are too
     coarse. The API should expose copied debug data such as leaf AABBs,
     internal node AABBs, parent/child edges, candidate pairs, object ids, and
     spatial ordering codes without exposing mutable tree internals.
   - Keep the first implementation useful for the current AABB tree while
     leaving room for sweep-and-prune, spatial hash, and brute-force
     diagnostics.
5. **Verification**
   - Add focused unit tests for any new broad-phase debug snapshot API and pair
     registry invariants.
   - Add a headless example smoke that renders deterministic pair and
     broad-phase overlay presets, writes screenshots, and verifies nonblank
     geometry plus overlay-region contrast or sampled pixels.
   - Keep `pixi run lint`, `pixi run build`, focused native-collision tests,
     and the example smoke as the minimum local gate before completion.

## Acceptance Criteria

- The GUI example is the single obvious place to inspect native collision
  pairs, not a collection of unrelated demos.
- Every current native pair row has one of three visible states in the example:
  supported and live, adapted/fallback and live, or unsupported placeholder with
  an explicit reason.
- Contact panels and overlays expose raw contact `point`, `normal`, `depth`,
  and manifold membership for the selected pair.
- Broad-phase overlays include object AABBs and candidate-pair visualization;
  AABB tree mode additionally exposes internal node boxes and tree edges.
- A deterministic headless smoke can prove that the example renders shapes,
  contacts, and broad-phase overlays without requiring interactive inspection.
- The final completion PR removes the dev-task folder after promoting durable
  evidence and leaves no dependency on FCL, Bullet, or ODE in the runtime GUI
  path.
