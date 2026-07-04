# Phase 2 Brief: Scene Catalog Ports

Worker brief for porting the frozen scene catalog (PLAN.md) into
`examples/demos/scenes/`. One worker per batch; one local commit per batch.
Read PLAN.md, EVIDENCE-*.md, and the Phase 1 sources first — especially
`examples/demos/DemoScene.hpp` (the contract) and
`examples/demos/scenes/RigidCubesScene.cpp` (the template: skel loading,
key actions mirrored in panel, contact-force visuals, per-scene state in a
shared_ptr captured by callbacks).

## Ground rules (all batches)

- Port from the ORIGINAL example under `examples/<name>/` (they are already
  `dart::gui::osg`-based). Use `git show 1a5469960c2:examples/demos/scenes/
  <name>.cpp` (origin/main history) for id/title/summary/category wording and
  panel content ideas ONLY — that code targets DART 7 APIs.
- Behavior parity: same world setup, same controller logic (WorldNode
  customPreStep bodies move into `setup.preStep` lambdas over a shared_ptr
  state struct), same keyboard actions (as `KeyAction`s → auto-mirrored as
  panel buttons), equivalent camera framing. Y-up `.skel` worlds must be
  reoriented to Z-up (see the shared helper introduced in Phase 1/2).
- Per-scene panel: expose the scene's tunables with safe clamped ranges
  (sliders/checkboxes); changing anything at runtime must never crash —
  mutate only inside preStep-applied state or paused-safe operations.
- A scene factory may throw on missing assets (host soft-fails); never
  abort/exit from scene code. No changes outside `examples/demos/` except
  registering sources in `examples/demos/CMakeLists.txt`.
- Multi-file controllers (e.g. atlas_simbicon) go to
  `examples/demos/scenes/<name>/` subdirs, namespaced `dart_demos::<name>`.
- Optional deps: point_cloud scene compiled only under `HAVE_OCTOMAP`;
  wam_ikfast under its existing ikfast availability condition (mirror the
  original example's CMake guard; registry omits the entry when unavailable
  rather than showing a broken row).

## Batches (each = one worker, one commit)

- B1 "Rigid & basics": boxes*, rigid_cubes* (*already done Phase 1 — just
  reconcile ids/titles with 1a5469960c2 wording), rigid_chain, rigid_loop,
  rigid_shapes, add_delete_skels, box_stacking, sleeping,
  simulation_event_handler, simple_frames, hardcoded_design, empty*.
- B2 "Constraints & soft": dynamic_joint_constraints, joint_constraints,
  human_joint_limits, mixed_chain, soft_bodies, tinkertoy, heightmap.
- B3 "Control & IK": hybrid_dynamics, biped_stand,
  operational_space_control, contact_inverse_dynamics, ssik_ik_gui,
  wam_ikfast.
- B4 "Humanoids & robots": atlas_puppet, atlas_simbicon (multi-file),
  hubo_puppet, fetch (MJCF), vehicle, point_cloud, drag_and_drop, imgui
  (fold as "ImGui reference" scene or drop if fully redundant with host —
  propose in report).

## Per-batch verification (mandatory before commit)

```bash
pixi run cmake --build build/default/cpp/Release --target dart-demos
./build/default/cpp/Release/bin/dart-demos --list-scenes
timeout 300 ./build/default/cpp/Release/bin/dart-demos --cycle-scenes --frames 10
for id in <each new scene id>; do
  DISPLAY=:0 ./build/default/cpp/Release/bin/dart-demos --headless \
    --shot /tmp/p2_<id>.png --scene $id --steps 400
done
pixi run lint
```

Report per scene: parity notes (what the original did that the port
does/doesn't), tunables exposed, PNG path. Orchestrator reviews every PNG
before the batch is accepted.

## Parity baselines (acceptance criteria)

`parity/parity-b{1..4}.json` hold per-scene checklists extracted from the
original sources (worldSetup, controllers, every key binding, tunables,
interactions, assets, portRisks). Workers implement against them; the
orchestrator reviews each batch against them. A port that drops a key
binding, tunable, or controller behavior without a documented reason fails
review. `portRisks` entries are mandatory reading before coding a scene.
