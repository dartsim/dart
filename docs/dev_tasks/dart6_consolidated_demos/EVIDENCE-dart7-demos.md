# Evidence: DART 7 Demos Architecture (Porting Reference)

Collected 2026-07-04 from `origin/main` by recon agent. Use as reference
evidence only (per release-branch principles); DART 6 behavior is proven
directly on this branch.

## Key commits / paths

- Current C++ shell: `origin/main:examples/demos/` — `DemoSceneEntry{id,
  title, category, summary, factory}` + `runDemos`; scenes built lazily on
  first selection; `demos_scenes` static lib + thin `app/main.cpp`.
- **Peak DART 6 catalog: `1a5469960c2703accb2762e03fe8a6bb1156dc08`** (parent
  of prune PR #2868) — 46 ported scenes under `examples/demos/scenes/` with
  registry categories: Getting Started, Visualization, Rigid Body, Collision,
  Constraints & Joints, Control & IK, Soft Bodies, Robots, Experimental.
  Reuse for: scene ids/titles/summaries, category taxonomy, per-scene panel
  content, port recipe. NOTE: these ports target DART 7 `ApplicationOptions`
  (Filament); DART 6 scenes port from the in-tree osg examples instead, which
  are already osg-based.
- Port recipe proven there: standalone `main()` + custom WorldNode/handlers →
  one `makeXScene()` factory; per-scene mutable state in a `shared_ptr`
  captured by `preStep`/panel/keyboard lambdas; scenes signal failure by
  throwing from the factory.

## Host behaviors to replicate on DART 6 (osg + vanilla ImGui)

- Scene switch is transactional: request queued (`requestSceneSwitch`),
  executed end-of-frame; on factory throw or startup-budget overrun, restore
  previous scene (or empty world) and surface a human-readable reason:
  "Starting demo 'X'...", "Failed to start demo 'X': <what>", "Restored
  previous demo 'A' after 'B' failed: <reason>". Host never crashes; sidebar
  stays usable; pending row shows "(starting)".
- Persistent host state (window, theme, panels layout) survives switches;
  per-scene state (world node, widgets, DnD, attachments) is torn down fully.
- Panels: Top `Simulation` toolbar (Play/Pause/Step/Rebuild/Restart/Layout
  reset); Left `Demos` navigator (status line, search + Clear, "Showing
  N/total", collapsing category headers, selectable rows, tooltip=summary);
  Right = scene-specific panels; Bottom = diagnostics/replay.
- CLI: `--list`, `--scene <id>` (+ env var), `--cycle-scenes --frames N`
  (headless smoke), `--headless --width --height --screenshot`. Scripted test
  hooks emit a JSON event log.
- CTest smokes: list + cycle-headless (software GL on CI); capture tool
  rejects blank/no-UI screenshots.

## Theme (port to DART 6 demos app-side)

`origin/main:dart/gui/detail/imgui_overlay.cpp:275` `applyModernDarkColors` +
`applyModernDarkMetrics`: cool neutral surfaces (window #1b1d23, panel
#21242b, header #16181d), single blue accent #4c8cf0 (hover/active/soft
variants), text #cdd3de with dim #7e8794, hairline borders #323843@60%;
WindowPadding 12x10, ItemSpacing 9x7, rounding: window 7 / frame 5 / tab 6 /
scrollbar 9, 1px window border, 0px frame border, left-aligned titles.
Docking-only bits (#ifdef IMGUI_HAS_DOCK) are skipped on DART 6 (system ImGui
1.92.8 master, no docking).

## py-demos (Python contract on main)

`PythonDemoScene{id,title,category,summary,build}` → `SceneSetup{world,
pre_step, step, force_drag, panels[ScenePanel], renderable_provider,
debug_provider, info}`; runner delegates the switch loop to the C++ host via
`dart.gui.run_demos(catalog, argv)`; Python adds SIGALRM startup budget
(`DART_DEMO_SCENE_STARTUP_TIMEOUT_MS`, `DART_PY_DEMO_SCENE_BUILD_TIMEOUT_MS`)
so hangs become catchable rollbacks. DART 6 py-demos will instead be a
dartpy-side runner over `dartpy.gui.osg` (ImGuiViewer bindings) since DART 6
has no `run_demos` host; same id/category scheme as C++.
