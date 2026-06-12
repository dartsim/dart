# Renderer Fidelity + First-Class Debug Visuals — Dev Task

## Current Status

- [x] Phase 1: Implement the "first slice" (A1 per-shape PBR, A3 texture mip
      chains, A4 `DART_GUI_HIGH_FIDELITY`, B1 `debugProvider`, B2 panel
      tuning, B3 joint-axis/velocity debug helpers) + unit tests + dartpy
      bindings + design doc
- [x] Phase 2: Internal code-review pass (0 critical/high; all actionable
      findings fixed: shared `kMinLitRoughness` clamp constant, provider
      hot-path docs, env-var token-set sync, hash-sentinel comment,
      clone round-trip + kind-gating tests)
- [x] Phase 3: Adapt to upstream #2932 ("Promote simulation to DART 7 public
      API") which retired the legacy world, `gui_scene_diagnostics`, and the
      legacy scene-extraction test mid-flight (merge commit `9185368b215`)
- [x] Phase 4: Open draft PR [#2984](https://github.com/dartsim/dart/pull/2984)
      (milestone DART 7.0) and trigger Codex review
- [ ] Phase 5: Shepherd PR through CI + Codex to merge
  - [x] Fix CI Lint: `check_docs_policy.py` requires the new design doc to be
        indexed in `docs/design/README.md` (verify with
        `pixi run python scripts/check_docs_policy.py`)
  - [x] Address Codex P2 finding: new panel toggles are inert (see Key
        Decisions / RESUME.md)
  - [ ] Watch remaining CI matrix; mark ready for review when green per
        `docs/onboarding/ai-tools.md` draft-ready fast path
- [ ] Phase 6 (post-merge follow-ups, tracked in the design doc): A2 IBL
      specular cubemap (needs real GPU), B4 depth-tested debug variant + view
      layers, A5/B5, re-wire world-derived debug overlays once debug
      extraction lands for the promoted `dart::simulation::World`

## Goal

Land the first slice of
[`docs/design/renderer_fidelity_and_debug_visuals.md`](../../design/renderer_fidelity_and_debug_visuals.md):
per-shape PBR materials, texture mip chains, high-fidelity headless capture
opt-in, and debug visualization as a `dart::gui` component feature
(`ApplicationOptions::debugProvider` + new debug primitives + panel tuning),
all renderer-neutral (backend-hidden rule).

## Non-Goals (this PR)

- A2 IBL reflections, B4 depth-tested debug material, A5/B5 (deferred in the
  design doc with rationale).
- Re-implementing world-derived debug extraction for the promoted
  `dart::simulation::World` (upstream stubbed `extractRenderables(world)` /
  `extractDebugLines(world, ...)` in #2932; re-wiring is a follow-up).

## Key Decisions

- **Sentinel→optional split**: `VisualAspect` PBR getters return negative
  sentinel = "renderer default"; extraction forwards only non-negative values
  into `MaterialDescriptor` `std::optional<double>`s; the Filament-side
  override clamps and applies to lit primitives only
  (`shapeUsesLitMaterialOverride`, exhaustive switch). Override values are
  hashed into the render-resource version so runtime edits rebuild.
- **#2932 adaptation** (merge `9185368b215`): keep upstream's stubbed
  legacy-world extraction; keep `makeJointAxisDebugLines` /
  `makeVelocityDebugLines` as public helpers operating on
  `dynamics::BodyNode` (usable from `debugProvider`); re-home tests in
  `tests/unit/gui/test_gui_debug_visuals.cpp` (`UNIT_gui_DebugVisuals`,
  links `dart-gui-core`) using the surviving `describeShapeFrame` and
  world-less `detail::AppOptions`/`createDartScene` seams; drop the
  `gui_scene_diagnostics` port (example retired upstream — its hand-rolled
  wiring no longer exists anywhere, which was B1's goal).
- **Stub policy**: committed `python/stubs/**` follow upstream's generator
  style. Local `pixi run generate-stubs` produces ~900 lines of
  formatting/staleness churn vs upstream's committed stubs (different
  nanobind/postprocess version) — do NOT blindly regenerate and commit; keep
  only genuine additions (our `dynamics.pyi` PBR methods are already in).
- **Codex P2 resolution**: body/joint/velocity built-in panel controls are
  hidden until the promoted `dart::simulation::World` has a real debug
  extraction path. `DebugDrawOptions` fields and the `BodyNode` helpers stay in
  place for `debugProvider`, which is the functional path for those overlays in
  this PR.

## Immediate Next Steps

See [`RESUME.md`](RESUME.md) — it has the exact uncommitted state and
command sequence.

## Cleanup Rule

Delete this folder in the PR/commit that completes Phase 5 (or fold the
remaining Phase 6 items into the design doc and delete then). Durable
content already lives in `docs/design/renderer_fidelity_and_debug_visuals.md`
and `docs/onboarding/gui-rendering.md`.
