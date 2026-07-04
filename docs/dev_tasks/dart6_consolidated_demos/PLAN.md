# Plan: DART 6 Consolidated Demos

Phases land as separate focused PRs on topic branches off `origin/release-6.20`.
Every phase ends with: build green, lint green, headless capture evidence for
UI-visible changes, and PLAN checkboxes updated.

## Architecture (decided)

- **Host app** `examples/demos/` → executable `dart-demos`:
  - `dart::gui::osg::ImGuiViewer` + `RealTimeWorldNode`, threading model
    `SingleThreaded` (repo-established pattern for ImGui apps).
  - **Scene-as-data registry** (adapted from DART 7 `DemoSceneEntry`):
    `struct DemoScene { id, title, category, summary, factory }` where
    `factory` returns a `SceneBundle`-style struct (world, optional custom
    `WorldNode` hooks as std::function pre/post-step callbacks, per-scene panel
    render callback, keyboard bindings, camera home, grid/shadow prefs,
    interactive frames / drag handles to enable).
  - **Runtime switching**: host owns one window; on switch it deactivates and
    `removeWorldNode`s the old node, tears down per-scene widgets/attachments/
    DnD registrations, then lazily builds the new scene inside try/catch.
    Factory failure → soft-fail to empty world + visible reason in UI (never
    crash, never trap the user).
  - **Crash-safety rule**: all world/scene mutations happen on the frame-loop
    thread; ImGui panel edits write into per-scene state applied in
    `customPreStep`/between frames. Scene switch requests are queued and
    executed between frames.
- **Panels (host chrome, vanilla ImGui 1.92 — no docking)**: programmatic
  layout against viewport work area; persistent across scene switches:
  - `Demos` navigator (left): categories in first-appearance order, search
    filter, current/pending selection state, failure reason surfacing.
  - `Simulation` toolbar (top): play/pause, single-step, reset scene, target
    RTF/frequency, timestep, gravity, solver iterations where safe.
  - `Inspector` (right): scene tree (skeletons → bodies → joints/dofs) with
    selection; transforms, velocities, joint positions/limits, contact list;
    per-scene custom controls section.
  - `Diagnostics` (bottom): log console (capture dart::common logging),
    stats (RTF smoothed/min/max, FPS, step count, sim time, contacts,
    bodies/dofs), live profiler summary (text profiler toggle;
    `DART_BUILD_PROFILE` builds), shadows/grid/wireframe toggles.
  - `setGuiScale` exposed for hi-dpi; style pass for a high-standard theme
    (consistent paddings, rounded corners, restrained palette).
- **Interactions** (host-level, per-scene opt-in):
  - Drag-force: Tinkertoy pattern generalized — click-pick BodyNode, spring
    force in pre-step, ArrowShape/LineSegment force visual, magnitude and
    state surfaced in UI.
  - Gizmo: `InteractiveFrame` + `enableDragAndDrop` on selected body/frame
    (translate/rotate/planar).
  - `BodyNodeDnD` IK-drag where scenes want it.
- **py-demos** `python/examples/demos/`: dartpy runner mirroring the host
  (dartpy has ImGuiViewer/ImGuiWidget/WorldNode bindings — verify per feature;
  degrade gracefully; no new bindings).
- **CLI**: `--scene <id>`, `--list-scenes`, `--cycle-scenes [frames]`
  (headless smoke), `--headless --shot <path> [--steps N]` (pbuffer capture,
  reusing the `examples/sleeping` recipe) for self-verification.

## Phase 0 — Recon & plan (this folder)  [done 2026-07-04]

- [x] DART 7 design doc + current demos/py-demos architecture reviewed.
- [x] GUI capability map (`EVIDENCE-gui-capabilities.md`).
- [x] Examples inventory + categorization (`EVIDENCE-examples-inventory.md`).
- [x] DART 7 historical dart-demos catalog reference notes
      (`EVIDENCE-dart7-demos.md`) — mine `1a5469960c2` for taxonomy/panels.
- [x] Verification harness proven (headless pbuffer PNG incl. ImGui panel).
- [x] Scene port list frozen (below). Port source = in-tree osg examples
      (already osg-based); `1a5469960c2` supplies ids/categories/summaries.

### Frozen scene catalog (categories → scenes)

- Getting Started: empty, hello_world-scene (KR5 intro; standalone example
  also kept), simple_frames
- Visualization: rigid_shapes ("shapes"), drag_and_drop, imgui-reference,
  heightmap, point_cloud (HAVE_OCTOMAP-gated)
- Rigid Body: boxes, rigid_cubes, rigid_chain, add_delete_skels,
  simulation_event_handler, sleeping, box_stacking
- Constraints & Joints: hardcoded_design, dynamic_joint_constraints,
  rigid_loop, tinkertoy, human_joint_limits, mixed_chain
- Control & IK: hybrid_dynamics, joint_constraints, biped_stand,
  operational_space_control, atlas_puppet, atlas_simbicon, wam_ikfast
  (ikfast-guarded), hubo_puppet, ssik_ik_gui, contact_inverse_dynamics
- Soft Bodies: soft_bodies
- Robots: fetch (MJCF), vehicle
- Kept standalone (not scenes): hello_world (canonical consumer sample),
  contact_benchmark (CI-load-bearing), speed_test (console benchmark),
  cylindrical_constraint (console; may gain a scene later). Deleted: rerun
  (orphan stub).

## Phase 1 — Host skeleton (PR 1)  [done 2026-07-04]

- [x] `examples/demos/` builds `dart-demos` (registered in
      examples/CMakeLists.txt; 2560 LOC).
- [x] Registry + 3 seed scenes (boxes, rigid_cubes with contact-force
      visuals, empty with gizmo) prove navigator + lazy factory + soft-fail.
- [x] Transactional runtime switch (factory-before-teardown, guarded hooks
      disable on first throw, ordered teardown registry).
- [x] Simulation toolbar (Play/Step/Rebuild/Reset, target-RTF slider,
      gravity toggle) + Diagnostics stats/log; RTF renders `--` when not
      free-running.
- [x] CLI: `--scene`, `--list-scenes`, `--cycle-scenes --frames N` (2x cycle
      + world-node leak audit), `--headless --shot`.
- [x] Acceptance: cycle smoke exit 0; headless PNGs of all 3 scenes reviewed
      by orchestrator (theme/layout verified; rigid_cubes Z-up + RTF display
      defects found in review and fixed). Implemented by Codex worker;
      committed as "Add dart-demos consolidated demos host...".
- Note: Y-up skel worlds need `reorientWorldToZUp` (RigidCubesScene.cpp
  helper — promote to shared scenes/ helper in Phase 2). rigid_cubes camera
  framing to tighten in Phase 2 B1.

## Phase 2 — Scene catalog ports (PR series)

- [ ] Port GUI examples as scenes, category by category (list frozen after
      recon; roughly: rigid/collision, soft bodies, constraints & joints,
      control & IK, humanoids/locomotion, misc).
- [ ] Each scene: parity with old example behavior (controllers via pre/post
      step callbacks, keyboard actions mapped and listed in UI, per-scene
      panel for its tunables).
- [ ] Per-scene on-the-fly tunables clamp to safe ranges; fuzz each panel.
- Acceptance per batch: headless capture of each ported scene; behavior parity
  notes in PR body.

## Phase 3 — Visual debugging suite (PR series)

- [ ] Log console capturing dart logging + app events, severity filter.
- [ ] Scene tree + inspector (read-only first; then safe live edits: joint
      positions when paused, visual aspect toggles).
- [ ] Contact visualizer (reusable): contact points/normals/force arrows from
      `getLastCollisionResult`, magnitude-scaled, color-coded.
- [ ] Drag-force + gizmo interactions host-wide with status surfaced in UI.
- [ ] Prereq library fix (small, separate PR): `InteractiveFrameDnD` leaks
      its 9 `new InteractiveToolDnD` sub-DnDs (`DragAndDrop.cpp:521-523`,
      `~InteractiveFrameDnD() = default` in `DragAndDrop.hpp:243`) →
      repeated gizmo teardown during scene switching leaks. Root-cause fix:
      out-of-line destructor deleting `mDnDs` (ABI-safe). DART 7 rewrote the
      GUI, so single release-branch PR (verify no equivalent on main first).
      Until landed, demos uses SimpleFrameDnD for switchable scenes (worker's
      Phase 1 mitigation, documented in `DemoScene.hpp`).
- [ ] Live profiler panel (text profiler summary + reset; Tracy hint when
      built with `DART_PROFILE_TRACY`).
- [ ] Stats: RTF (smoothed/min/max), FPS, steps/refresh, counts.
- [ ] COM/support-polygon toggles for legged scenes.

## Phase 4 — py-demos (PR)

- [x] Verify dartpy binding coverage — `EVIDENCE-dartpy-bindings.md`.
      Outcome: ImGui is unreachable from pure Python (no ImGuiWidget
      trampoline, no imgui module) and the task directive forbids new
      bindings for demos → py-demos is a keyboard-navigated runner
      (instruction-text overlay + terminal catalog), not an ImGui workspace.
- [ ] Runner + registry mirroring C++ id/category scheme; scenes as
      RealTimeWorldNode subclasses; switch via addWorldNode/
      setWorldNodeActive; `--shot` capture mode.
- [ ] Port python examples as scenes; graceful degradation notes where
      bindings missing.
- [ ] Smoke: import + headless capture via dartpy.
- [ ] Side-fix decision: `python/examples/atlas_puppet/main.py:552` calls
      unbound `SupportPolygonVisual` → broken today (pre-existing). Either
      bind the class (separate small PR, legit bugfix) or drop the call when
      porting.

## Phase 5 — Cleanup (PR)

- [ ] Retire superseded `examples/*` and `python/examples/*` dirs; keep
      specialized set (minimum `hello_world`; final list evidence-based —
      console/benchmark/external-dep examples judged individually).
- [ ] Prove no test/CI/tutorial/doc references break (rg sweep + CI dry run).
- [ ] Update `examples/README.md` + python examples README to point at demos.

## Phase 6 — Wrap-up

- [ ] CHANGELOG entry; design note `docs/design/` (DART 6 variant) promoting
      durable architecture facts from this folder.
- [ ] Final full verification: build, lint, cycle smoke, capture gallery,
      rapid-switch + panel fuzz robustness pass.
- [ ] Retire this dev task folder per `docs/dev_tasks/README.md`.

## Verification harness (used throughout)

- `pixi run build` + `pixi run lint` (baseline gates, docs/ai/verification.md).
- Headless self-check: `DISPLAY=:0` pbuffer capture (recipe in
  `EVIDENCE-gui-capabilities.md`) — screenshots reviewed by agent before
  claiming visual work done.
- Robustness: `--cycle-scenes` smoke, rapid-switch loop, per-panel fuzz of
  tunables at runtime.
- Workers: Codex/subagent implementers get one phase-scoped brief per PR;
  orchestrator reviews diffs + evidence before accepting.
