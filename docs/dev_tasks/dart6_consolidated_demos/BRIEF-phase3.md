# Phase 3 Brief: Visual Debugging Suite

Builds on the Phase 1 host. All features are host-level (scene-agnostic),
live-toggleable, and crash-safe. Read PLAN.md + EVIDENCE-gui-capabilities.md
first. Two PRs: (a) tiny library prereq fix, (b) the suite.

## PR a — library prereq (do between scene batches; build dir must be free)

`InteractiveFrameDnD` leaks its 9 `new InteractiveToolDnD` sub-DnDs
(`dart/gui/osg/DragAndDrop.cpp:521-523`; `~InteractiveFrameDnD() = default`
in `DragAndDrop.hpp:243`). Fix: declare the destructor in the header,
define it out-of-line deleting `mDnDs` entries (ABI-safe: no member/layout
change). Add a CHANGELOG entry. Verify: build + existing tests +
drag_and_drop example smoke + demos cycle smoke. DART 7 rewrote the GUI —
confirm `origin/main` has no equivalent code path (single release-branch PR,
no dual-PR needed) and say so in the PR body.

## PR b — the suite (host features, examples/demos/ only)

1. **Log console upgrade**: capture DART's `dtmsg/dtwarn/dterr` stream
   output into the host log (scoped streambuf redirect, restored on exit),
   severity filter chips, autoscroll toggle, copy button, ring buffer
   (~2000 entries).
2. **Scene tree + inspector** (right panel, above scene controls):
   skeletons → bodies (tree), joints/dofs under each body. Selection state
   host-owned. Inspector shows: world/relative transform, linear/angular
   velocity, mass, and per-joint position/velocity/limits. Joint position
   sliders are editable ONLY while paused (clamped to limits; changes apply
   via queued command executed pre-step). Visual toggles per body: wireframe
   (osg PolygonMode), hide/show (VisualAspect), highlight selected
   (emissive/alpha flash).
3. **Contact visualizer (reusable)**: extract RigidCubesScene's contact
   arrows into a host facility: toggle in Diagnostics; reads
   `world->getLastCollisionResult()`; ArrowShape per contact scaled by
   force/penetration, color by magnitude; count shown in stats. Cap arrow
   count (e.g. 256) to bound cost; document the cap in the UI.
4. **Drag-force interaction (host-wide)**: generalize the Tinkertoy
   pattern: modifier+left-drag on any BodyNode applies a spring force
   (coeff slider, max-force clamp) at the picked point; LineSegment/Arrow
   visual; status line in toolbar shows target body + |F|; works in every
   scene; releases cleanly on scene switch (registered teardown).
5. **Gizmo on selection** (after PR a lands): "Attach gizmo" button in the
   inspector attaches an InteractiveFrame to the selected body's transform;
   dragging while paused teleports (setTransform via queued command),
   while running applies the drag-force spring instead. Detach on
   deselect/switch.
6. **Profiler panel**: gated on `DART_BUILD_PROFILE` (pixi config has it ON
   with builtin text backend). Toggle "Record profile" → `resetProfile()`,
   then render `getProfileSummaryText()` monospaced with per-frame refresh
   throttle (every ~30 frames); note in UI when built without profiling.
7. **Stats upgrades**: contact count, active constraint count if cheap,
   per-refresh step count, world timestep display; keep RTF "--" rule.
8. **View utilities** in toolbar menu: shadow toggle, grid toggle +
   plane/size, headlights toggle, GUI scale slider (ImGuiHandler::
   setGuiScale, 0.75–2.0), camera home reset.
9. **Contract addition**: optional `preRefresh` callback on DemoSceneSetup
   (crash-guarded like preStep, invoked from customPreRefresh every render
   frame incl. while paused). Motivation: sleeping scene's island
   recolor/aim-line originally ran per-render-frame; B1 had to demote it to
   postStep (visuals freeze while paused). Migrate sleeping to preRefresh
   once available.

## Acceptance

- Headless captures: scene tree with a selection + inspector open; contact
  visualizer on (rigid_cubes under load); drag-force status active
  (scripted: apply force programmatically for the shot); profiler panel
  recording; each capture reviewed by orchestrator.
- Robustness: cycle smoke stays green; rapid-toggle fuzz of every new
  toggle across 3 scene switches; paused joint-slider edits on a humanoid
  scene do not crash or NaN (verify world state stays finite).
- `pixi run lint` + full build green.
