# Filament GUI - Live Steering Note (2026-05-14)

> Live supervisor note from a parallel evaluator pass. Read after `RESUME.md`.
> This is **strategic guidance**, not a task list. Owner: Codex.

## 2026-05-15 supervisor update (translation of live human direction)

Synthesis of a parallel architect / code-reviewer / verifier / critic pass over
this folder plus the current code state. Sticky — do not delete; mark items
`~~done~~` with a one-line evidence pointer when they land, and surface
disagreement under "Open Issues" instead of editing "Decisions in force".

### Status (verified at 8796ed5ad99)

- `examples/filament_gui/` is renamed to `examples/dartsim/`. The README
  documents `dartsim` as the application-level viewer with legacy launchers as
  sibling examples.
- `examples/dartsim/main.cpp` includes `<dart/gui/application.hpp>` and calls
  `dart::gui::runApplication(argc, argv)`. Public include path is therefore
  `dart/gui/application.hpp` (NOT `detail/application.hpp` as some older
  doc passages still claim).
- CMake target / `OUTPUT_NAME` is `dartsim`. Checkpoint `30b879458f8` renamed
  the old private `dart_filament_gui_add_example` helper to
  `dart_gui_filament_add_example`, keeping backend details out of user-facing
  example names.
- Linux headless CI was red because the workflow invoked the removed legacy
  `rigid_cubes` executable with the old `--out` contract. The local repair
  restores `rigid_cubes` as a `dart::gui` launcher and updates the workflow to
  validate `--screenshot` PPM output.
- The boundary-guard test `UNIT_gui_FilamentSceneExtraction`
  (`tests/unit/gui/test_filament_scene_extraction.cpp`) is retargeted locally
  to the promoted `examples/dartsim/` entry point and `dart/gui` public header
  surface.
- CI Lint, CI Linux, CI macOS, CI Windows, and CodeQL were manually dispatched
  for `d343c3c64bc` without opening a PR.
- `c9ccedfebe8` adds promoted `dart/gui/*.hpp` wrapper headers and explicit
  `dart::gui` aliases for renderer-independent scene, viewer, geometry,
  interaction, debug, and profiling concepts, while keeping
  `dart/gui/experimental/*` as compatibility shims for now.
- `8796ed5ad99` finishes the C++ namespace promotion checkpoint: promoted
  renderer-independent declarations and definitions now live under
  `dart::gui`, `dart/gui/experimental/*.hpp` remains compatibility-only, and
  private Filament implementation names use `dart::gui::filament` while the
  physical `dart/gui/experimental/detail/filament` path remains later
  file-layout debt.
- CI Lint on `d343c3c64bc` failed at CMake configure because
  `examples/simple_frames/CMakeLists.txt` was ignored by `.gitignore`'s
  `*_frames/` rule. The local repair force-adds that launcher before the next
  CI dispatch.
- CI Lint on `8796ed5ad99` passed. Linux, macOS, Windows, and CodeQL were
  running at the latest check and should not block independent local progress.
- Current pivot after `5d514a4558e`: the application source directory is
  `apps/dartsim/`; `examples/dartsim/` was only an interim location.
- `36684d68e67` moves the `dartsim` application source to `apps/dartsim/`
  while preserving the `dartsim` target and binary.
- `a23ea52a9b0` restores `examples/hello_world/main.cpp` as a real
  source-defined `dart::gui` program and adds
  `dart::gui::ApplicationOptions::world` as the public handoff from example
  code to the official runner.
- Next checkpoint scope: restore the rigid-box example family
  (`boxes`, `rigid_cubes`, `box_stacking`) as source-defined public-API
  examples. Use public DART dynamics/simulation APIs to construct worlds,
  pass them through `ApplicationOptions::world`, and use `Panel` only for the
  former custom controls that still matter after the OSG removal.
- Following checkpoint scope: restore `simple_frames` and
  `capsule_ground_contact` as source-defined public-API examples. Use
  `SimpleFrame`/shape APIs for the former and a small `Panel` for the
  capsule reset controls that used to live in an OSG key handler.
- `f812a4afc1d` completes the rigid-box family source migration. `45e44a0318a`
  completes the simple-frame/contact source migration.

### Decisions in force (do NOT reopen)

1. **Naming is final**. Application-level binary is `dartsim`; application
   directory is `apps/dartsim/`; public include is
   `<dart/gui/application.hpp>`; entry symbol is `dart::gui::runApplication`.
   No `gui_viewer`, no `filament_gui`, no `dart_filament_gui` in user-facing
   strings. Private helper names were also moved away from the old
   `filament_gui` compound in `30b879458f8`; keep new private names aligned
   with `gui_filament`/`DART_GUI_FILAMENT` wording unless a real backend
   identifier is needed.
   Branding distinction: `DART` is the project and C++ library family identity,
   `libdart` is appropriate for package/library artifacts, and `dartsim` is the
   application-level simulator/viewer identity, analogous to Isaac Sim.
2. **Restoration semantics for legacy examples**: each historical user-facing
   example (`hello_world`, `rigid_cubes`, `drag_and_drop`, `imgui`,
   `simple_frames`, etc.) becomes its own `examples/<name>/` directory that
   builds a real executable through the shared `gui_scene_launcher.cpp` and
   supplies that example's default scene to
   `dart::gui::runApplication(argc, argv, defaultScene)`. Pixi `ex` aliases
   are not a substitute — CI invokes the binary on disk.
3. **`--scene <name>` fixtures inside `dartsim`** stay as the developer
   fixture menu and the source of truth for headless smoke generation. Per-
   example binaries become the user-facing surface; `dartsim --scene foo`
   remains the developer entry point. This is a permanent split, not interim
   debt — the `--scene` retirement plan called for in the critic pass is
   resolved by accepting this split.
4. **Promoted public API surface stays DART-owned and backend-hidden.**
   `dart::gui::runApplication` is the application entry point, and
   renderer-independent scene/viewer/geometry/interaction/debug/profile
   concepts now live in stable `dart/gui/*.hpp` headers under `dart::gui`.
   Do not expose Filament, GLFW, Dear ImGui, OpenGL, Vulkan, Metal, OSG, or
   Raylib types through this surface. Do not add a broad `dart::gui::Viewer`
   object model until the north-star viewer/tool/panel phases have concrete
   implementation evidence.
5. **Out of scope for this branch (do not start, even opportunistically)**:
   ImGui Docking, dockable 3D widget, video capture, font atlas/label work,
   broader product/packaging work beyond the `dartsim` naming distinction,
   macOS/Windows port work, conda-forge feedstock changes. Track these as
   follow-up application/capture tasks; do not let a refactor accidentally land
   any of them.
6. **Capture compatibility is in scope now.** Restore historical
   `--out <dir>` image-sequence output through the promoted `dart::gui` capture
   path while keeping `--screenshot <path>` as the current single-frame and CI
   smoke contract. This is image-sequence compatibility, not video capture.

### Order of operations (CI repair first, then promotion debt)

1. ~~**Unblock Linux headless CI**~~: `d343c3c64bc` restores the `rigid_cubes`
   executable and updates the workflow to validate
   `rigid_cubes --headless --frames 10 --screenshot ...` PPM output. This
   intentionally chooses the promoted capture contract over restoring `--out`
   PNG sequences in the same checkpoint.
2. ~~**Retarget the boundary-guard test**~~ (`UNIT_gui_FilamentSceneExtraction`
   in `tests/unit/gui/test_filament_scene_extraction.cpp`) to scan
   the `dartsim` application entry point and the promoted `dart/gui` header
   surface. The
   configure-time application check is now exposed through
   `dart_gui_add_application()` and continues to guard the `dartsim` entry
   point.
3. ~~**Restore legacy examples as real per-dir binaries**~~: `d343c3c64bc`
   restores the historical GUI example executable names as thin launchers using
   `examples/gui_scene_example.cmake` and `examples/gui_scene_launcher.cpp`,
   not just the first three examples.
4. ~~**Sweep stale user-facing `filament_gui` strings**~~ across the repo:
   runner
   `scripts/run_cpp_example.py`, `python/tests/unit/test_run_cpp_example.py`,
   smoke test names, status messages in
   `dart/gui/experimental/detail/filament/testing/run_headless_smoke.cmake`,
   `CHANGELOG.md`, `dart/gui/AGENTS.md`, and skill docs under
   `.claude/skills/dart-build/` and `.codex/skills/dart-build/`. Private
   CMake helper names under `dart/gui/experimental/detail/filament/` were
   swept in `30b879458f8`.
5. ~~Rerun `pixi run lint`, commit, and push the checkpoint as usual. Do not
   open a PR.~~ `d343c3c64bc` is pushed to the tracked remote branch.
6. ~~**Track ignored `simple_frames` launcher**~~: `cd04ba4862ac` force-added
   `examples/simple_frames/CMakeLists.txt`, reran lint, committed, pushed, and
   redispatched CI. This was a Git tracking repair for the restored examples,
   not a renderer behavior change.

### Next promoted-header and capture slice

- The initial promoted-header checkpoint landed in `c9ccedfebe8`; the C++
  namespace/definition promotion landed in `8796ed5ad99`.
- Continue moving maintained public-facing examples, tests, and bindings off
  `dart/gui/experimental/*.hpp` / `dartpy.gui.experimental` where those
  concepts are now official.
- Keep `dart/gui/experimental/*.hpp` as compatibility shims for this checkpoint.
- Keep private Filament/GLFW/ImGui implementation under
  `dart/gui/experimental/detail` until a later file-layout sweep.
- The next implementation checkpoint should restore `--out <dir>`
  image-sequence capture from the shared `dartsim` command-line path, with
  focused tests and one restored historical executable headless proof.

## 2026-05-15 supervisor pivot: restore original examples; promote `dartsim` to a real app

**This block supersedes Decisions 2, 3, 5 and item 7 above.** The shared
`gui_scene_launcher.cpp` macro shim is the wrong long-term shape. The
`dartsim` directory is also in the wrong place — it should be an
**application**, not an example.

### New architecture

```
<repo-root>/
├── dart/
│   └── gui/                       ← promoted public API: dart::gui::*
│       ├── application.hpp        ← dart::gui::runApplication (entry helper)
│       ├── viewer.hpp / scene.hpp / interaction.hpp / debug.hpp / ...
│       └── detail/filament/       ← private backend (after item 8)
│
├── apps/
│   └── dartsim/                   ← NEW: real application, not an example
│       ├── CMakeLists.txt
│       ├── main.cpp               ← parses CLI, opens scene description file
│       ├── app/                   ← app-only code (panels, project model, etc.)
│       │   ├── docking_layout.cpp
│       │   ├── scene_loader.cpp   ← reads URDF / SDF / SKEL / MJCF / project file
│       │   ├── timeline.cpp       ← play / pause / step / scrub / replay
│       │   ├── inspector.cpp      ← skeleton / joint / body inspector panel
│       │   ├── log_panel.cpp      ← in-app log + diagnostics
│       │   ├── recording.cpp      ← screenshot + image-sequence + future video
│       │   └── project.cpp        ← .dartsim project file + recent files
│       └── README.md
│
└── examples/
    ├── hello_world/main.cpp       ← restored ORIGINAL educational source,
    │                                migrated to dart::gui (no scene-string macro)
    ├── rigid_cubes/main.cpp       ← restored ORIGINAL educational source
    ├── drag_and_drop/main.cpp     ← restored ORIGINAL educational source
    ├── atlas_puppet/main.cpp      ← restored ORIGINAL educational source
    ├── ... one main.cpp per example, written like a user would write it ...
    └── README.md                  ← "if you want a full app, see apps/dartsim"
```

### Decisions in force (supersede the earlier numbered list where they conflict)

1. **`apps/dartsim/` is a top-level application directory.** It is not an
   example. Its purpose is to be DART's IDE-style simulation viewer:
   - **Inputs**: URDF / SDF / MJCF / SKEL files passed as CLI args
     (`dartsim path/to/world.urdf`); also a `.dartsim` project file format
     (JSON or TOML) listing the world file, default camera, simulation
     settings, panel layout, recording presets.
   - **Stack**: Filament for rendering, GLFW3 for windowing, **Dear ImGui
     with Docking enabled** for the panel system. No `--scene <name>`
     flag in the user-facing app (developer fixtures stay under
     `dartsim --dev-scene <name>` or are pulled out of the app entirely).
   - **Initial panel set**: Scene tree (skeletons / joints / bodies),
     Inspector (selected entity properties), Timeline (play / pause /
     step / scrub / loop), Log (in-app log capture from
     `dart::common::Logger`), Recording (screenshot, image-sequence,
     future video), Console (placeholder for a future Python REPL — no
     scripting yet).
   - **Long-term direction**: docked 3D viewport, project file
     persistence, replay of recorded simulation runs, plotting of
     joint/contact data, headless batch mode for CI.
   - **Out of scope for the first `apps/dartsim/` checkpoint**: video
     capture (image-sequence is enough), Python scripting, multi-window
     layouts, macOS/Windows ports, custom themes, internationalization.

2. **Examples go back to the pre-cleanup shape.** Each `examples/<name>/`
   contains its own real `main.cpp` + `CMakeLists.txt` (+ optional
   per-example assets). The shared `gui_scene_launcher.cpp` /
   `gui_scene_example.cmake` macro path is removed. Each example is the
   smallest possible standalone DART program demonstrating ONE concept,
   written the way a new user would write it. Educational value is the
   primary acceptance criterion — not size.

3. **Each example consumes only public DART API.** Allowed includes:
   `dart/<module>.hpp` (`dart/dynamics.hpp`, `dart/simulation.hpp`,
   `dart/io.hpp`, `dart/gui.hpp`, …). Forbidden: `<filament/...>`,
   `<GLFW/...>`, `<imgui.h>`, anything under `dart/gui/detail/` or
   `dart/gui/experimental/`. The boundary-guard test gains a per-example
   scan that enforces this.

4. **Promotion of the panel/tool API moves up.** `dart::gui::Panel` /
   `dart::gui::Tool` (or the equivalent callback / hook API) is now
   load-bearing — both `apps/dartsim/` and the migrated `imgui` /
   `drag_and_drop` / `tinkertoy` / `lcp_physics` examples need it. Land
   it before any per-example migration that touches custom widgets.

5. **`scene_fixtures.cpp` becomes pure dev/test infrastructure.** Once
   examples have their own real sources, the matching `createXxxScene()`
   factories in `scene_fixtures.cpp` become redundant. They can either
   move out of `dart/gui/experimental/detail/filament/` into a separate
   test-only target (e.g. `tests/fixtures/gui_scenes/`) or be deleted.
   Library code should not contain example fixtures.

6. **Item 8 (drop `experimental/`) order changes.** The `experimental/`
   directory still gets removed, but it now follows BOTH the example
   restoration AND the `apps/dartsim/` extraction, because both new
   surfaces will reveal which `dart::gui` symbols are actually consumed
   by external-shaped code.

### Revised order of operations (replaces items 7+8 below)

7'. **Land the minimum panel/tool public API.** `dart::gui::Panel` and
`dart::gui::Tool` (names TBD by Codex) under `dart/gui/panel.hpp` and
`dart/gui/tool.hpp`. The current MVP `dartsim` (now `apps/dartsim/`
after step 8') stops reaching into private
`detail/filament/imgui_overlay.hpp` for its built-in panel and
consumes the new public API instead. This is the precondition for
both the application work and the per-example migration. Headless
smoke gate: a custom panel's `onUpdate` callback fires N times in N
rendered frames.

8'. **Move `examples/dartsim/` → `apps/dartsim/` and grow it into a real
application.** Single `git mv`, then build out the panels listed
under Decision 1 above.

Status: source-directory extraction is implemented in the next code checkpoint
after `5d514a4558e`; app feature growth remains outstanding.

Initial scope:

- `apps/dartsim/main.cpp` accepts a positional arg
  (`dartsim <world-file>`) and recognized extensions
  `.urdf|.sdf|.mjcf|.skel|.dartsim`. With no arg, opens an empty
  world and surfaces a "File → Open…" panel button.
- `apps/dartsim/app/scene_loader.cpp` dispatches by extension to
  existing `dart::io::DartLoader` / `SdfParser` / `MjcfParser` /
  `SkelParser`.
- `apps/dartsim/app/docking_layout.cpp` configures ImGui docking
  with a central 3D viewport node and side-docked
  Scene/Inspector/Timeline/Log panels. **This requires enabling
  ImGui Docking**, which Decision 5 of the original block previously
  listed as out-of-scope; that exclusion is **lifted specifically
  for ImGui Docking** to enable `apps/dartsim/`.
- `apps/dartsim/app/recording.cpp` consumes the promoted
  `dart::gui::Recorder` (screenshot + image-sequence) added in item 6.
- The ~30 named `--scene` fixtures are NOT exposed in the app's user
  surface. They become dev-only via `dartsim --dev-scene <name>` for
  Codex's continued use, or migrate to the test-fixture target per
  Decision 5.

9'. **Restore each `examples/<name>/main.cpp` from the pre-cleanup OSG
sources, migrated to `dart::gui`.** Process per example: - Recover the original source via `git log --diff-filter=D --
      examples/<name>/main.cpp` then `git show <sha>:examples/<name>/main.cpp`. - Rewrite the OSG-based viewer/world setup to use `dart::gui::*`
API: `dart::gui::Application` / `runApplication`, the new
`dart::gui::Panel`/`Tool` from item 7', and standard DART
dynamics/simulation/io. - Delete the matching `createXxxScene()` factory from
`scene_fixtures.cpp` (or move it to the test-fixture target). - Drop the `examples/<name>/CMakeLists.txt` from the macro shim;
replace with a normal `add_executable(<name> main.cpp)` + link
against `dart-gui` and required `dart-*` modules. - Acceptance: example builds standalone, links only public DART
libraries, `examples/<name>/main.cpp` reads as something a new
user could plausibly have written, headless smoke (kept under the
dev `--dev-scene` path or via direct binary invocation) still
produces the expected first frame. - Order: `hello_world` first (canonical template), `drag_and_drop`
second (forces interaction API completeness), `imgui` third
(forces the Panel/Tool API completeness), then batch by family.

10'. **Remove the macro shim entirely** once every example has a real
`main.cpp`. Delete `examples/gui_scene_launcher.cpp` and
`examples/gui_scene_example.cmake`. Update
`examples/CMakeLists.txt` to use plain `add_subdirectory()` calls.

11'. **Then proceed to the original item 8 (drop `experimental/`)**, with
the relocation now informed by the real public-API consumption
pattern from `apps/dartsim/` and `examples/<name>/main.cpp`.

### What the per-example checklist becomes

The "Per-scene graduation checklist" below is **repurposed** as a
**per-example restoration checklist** with the same item set, but the
acceptance criteria now read:

- [ ] `examples/<name>/main.cpp` exists as a real, educational source
      (recovered from pre-cleanup history and migrated to `dart::gui`).
- [ ] `examples/<name>/CMakeLists.txt` uses `add_executable` + plain
      `target_link_libraries(<name> PRIVATE dart-gui dart-...)`.
- [ ] No `<filament/...>`, `<GLFW/...>`, `<imgui.h>` includes anywhere
      under `examples/**/*`.
- [ ] The matching `createXxxScene()` is removed from `scene_fixtures.cpp`
      (or moved to the test-fixture target).
- [ ] Headless smoke (developer route) still passes.

The Tier-A vs Tier-B distinction is dropped — every example gets its own
real source. The Tier-B set (`hello_world`, `boxes`, `simple_frames`,
`capsule_ground_contact`, `soft_bodies`, `empty`) are the simplest
templates to restore first; the API-forcing examples (`imgui`,
`drag_and_drop`) gate item 7'.

Rigid-box family slice:

- [x] `examples/boxes/main.cpp`: restore the stacked 5x5x5 dynamic-box
      educational source through `ApplicationOptions::world`; do not expose
      Bullet, OSG, Filament, GLFW, or ImGui in the example source.
- [x] `examples/rigid_cubes/main.cpp`: restore the cube world as a standalone
      source-defined example. Replace OSG keyboard handlers with a small
      public `dart::gui::Panel` only if the force-control affordance is kept.
- [x] `examples/box_stacking/main.cpp`: restore the stacking setup as a
      public-API world plus a compact `Panel` for simulation/solver status if
      supported by the current promoted panel API.
- [x] Remove the `--scene boxes` runner defaults for these binaries after their
      own sources are active.
- [x] Validate with focused builds, the GUI boundary unit test, Python example
      runner coverage, one direct headless screenshot, `pixi run lint`, commit,
      and push. Evidence: `f812a4afc1d`.

Simple frame/contact slice:

- [x] `examples/simple_frames/main.cpp`: restore the frame hierarchy with
      public `SimpleFrame`, box, ellipsoid, and line-segment shape APIs.
- [x] `examples/capsule_ground_contact/main.cpp`: restore the capsule/ground
      contact world with public dynamics/simulation APIs and a promoted
      `Panel` for pose reset/status controls.
- [x] Remove `--scene simple-frames` and `--scene capsule-ground-contact`
      runner defaults once the binaries own their scenes.
- [x] Validate with focused builds, the GUI boundary unit test, Python example
      runner coverage, direct headless screenshots, `pixi run lint`, commit,
      and push. Evidence: `45e44a0318a`.

### Stop condition for this pivot

- `apps/dartsim/` builds, opens a URDF/SDF/MJCF/SKEL file from CLI,
  shows the docked panel layout, plays simulation through Timeline,
  captures an image sequence through Recording.
- Every `examples/<name>/` directory contains its own real `main.cpp`
  consuming only public `dart::gui` API; no shared launcher, no
  `DART_GUI_DEFAULT_SCENE` macro, no per-example link to backend
  internals.
- `scene_fixtures.cpp` either shrinks to zero (factories deleted) or
  moves to a test-only fixture target.
- `dart/gui/experimental/` contains only `[[deprecated]]` shim headers.

### Default lighting: brighten shadow side without flattening contrast

**Problem:** With the orbiting key light off (or before it has rotated
into a useful position), the side of the scene facing away from the
key reads as nearly black. The user has to wait for the orbit to
rotate before geometry on the shadow side is legible. Cranking the
orbit period faster is a workaround, not a fix.

**Root cause:** the dark side is filled almost entirely by Filament's
indirect lighting (IBL spherical-harmonics ambient). The current IBL
intensity and SH ambient floor are tuned for one well-placed key
light; when that key is misaligned, there is no fill source.

**Fix: apply all three of the levers below at modest amounts.**
Modern real-time DCCs (Unity HDRP default sky, Unreal Lumen scene
capture default, Isaac Sim default stage) all combine these.

1. **Raise default IBL intensity by ~1.6×.**
   - The current `IndirectLight::Builder().intensity(...)` setting in
     `dart/gui/experimental/detail/filament/render_environment.cpp`
     should multiply by ~1.6. Filament defaults run ~30k–60k cd/m²;
     this lifts shadow-side luminance without washing out the lit side.
   - Zero perf cost, zero asset change.

2. **Add a fixed fill light opposite the key, shadow-disabled.**
   - Classic 3-point cinematography: a second directional light at
     ~30% of the key's intensity, pointing from the opposite
     hemisphere, with shadow casting **off**. This is the largest
     contributor to shadow-side legibility and the closest analog to
     what Isaac Sim ships by default.
   - Add it during scene environment setup; no per-fixture knob needed.
   - Cost: one extra non-shadow-casting directional light (~0 GPU cost).

3. **Set a small SH ambient floor (~0.18).**
   - Prevents pitch-black corners even when both directionals miss.
   - Tuned via the SH band-0 coefficient on the IBL, or as an explicit
     ambient term added in the lit material. Whichever path keeps the
     existing IBL workflow intact.
   - Cost: zero perf.

**Hard constraint: shadows must stay visible by default.** The fix
must not flatten contact-grounding cast shadows from the key light.
After the lighting tweak:

- A box on a ground plane must still cast a clearly readable shadow
  in the default scene with **no CLI flags**.
- The shadow must remain readable across the orbit (not just at
  flattering key angles).
- The fill light from lever 2 is **shadow-disabled** specifically so
  it does not soften or lift the key's shadow.
- The ratio of "lit-side luminance : shadow-side fill" should stay
  roughly **3:1 to 4:1** (cinematography fill ratio). Anything above
  ~2:1 starts looking flat. If the ×1.6 IBL boost compresses below
  3:1, dial it back rather than disabling the fill.

**What NOT to do:**

- Do not crank only IBL/ambient — the scene flattens and shadows lose
  contact grounding (which kills the visual-quality shadow gate in
  `06-visual-quality.md`).
- Do not disable shadows — they are required by the visual-quality
  gate and are a useful debugging signal for contact and penetration.
- Do not change the orbit-light default period to paper over the
  issue. Orbit is a shading-diagnostics aid, not a required scene
  element. The static lighting must be self-sufficient.
- Do not introduce per-scene lighting tuning in `scene_fixtures.cpp`
  for this fix — the change belongs in the renderer's default lighting
  setup so every scene benefits.

**Flip the orbit-light default to off (opt-in).** The orbit was on by
default to compensate for the shadow-side darkness; with the three
lighting levers above in place, the static setup is sufficient and the
orbit becomes purely a diagnostic toggle.

- New default: `--orbit-light` = OFF. Users opt in with `--orbit-light`
  or the panel checkbox when they want the diagnostic motion.
- The CLI flag pair stays symmetric: `--orbit-light` enables it,
  `--no-orbit-light` is now redundant but kept for compatibility.
- The panel checkbox stays; just flip its initial state to unchecked.
- Update `--help` text to read "enable orbiting key light (diagnostic
  shading aid; off by default)".
- Update `--orbit-light-period <seconds>` description to clarify it
  only takes effect when orbit is enabled.
- Document the change in `CHANGELOG.md` as a behavior break: previous
  default was orbit-on with an 80-second period; new default is
  orbit-off with a static key.

**Implementation notes for Codex:**

- Single focused commit: edit `render_environment.{hpp,cpp}` for IBL
  intensity scaling, fill-light creation, and SH ambient floor.
- Update the headless smoke contrast analyzer threshold if needed:
  the `analyze_contrast` mode in
  `dart/gui/experimental/detail/filament/testing/analyze_headless_smoke.py`
  checks dark/mid/bright bucket counts. The fix raises the dark
  bucket floor; verify the threshold still passes a flat-but-nonblank
  scene as failing (the contrast gate must still reject vacuous
  output).
- Add a focused unit/snapshot test for the lighting setup: build a
  one-box scene with the orbiting key disabled, headless render, and
  assert the back-side luminance is above some legibility floor (e.g.
  the back-face mid pixel value is ≥ 60/255 in the rendered PPM).
  This locks in the regression.
- After landing, re-run all per-scene smokes and visually spot-check
  `--scene atlas-puppet` and `--scene g1` (the most lighting-sensitive
  due to large robot meshes in shadow).

**Acceptance:**

- With `--no-orbit-light` set, every scene's shadow side is legibly
  shaded — no pure-black geometry — without flattening the lit side.
- The visual-quality shadow gate in `06-visual-quality.md` still
  passes (shadows still visible, contact grounding preserved).
- **Default scene with no CLI flags shows a clearly cast shadow from
  the key light onto the ground plane.** This is the user-visible
  regression check for the "I'd like to see shadow by default"
  requirement.
- **Default scene with no CLI flags has a static key light** — orbit
  is opt-in via `--orbit-light` or the panel checkbox.
- Lit:fill luminance ratio stays in the 3:1–4:1 cinematography range;
  if exceeded, dial back the IBL boost rather than removing the fill.
- The contrast smoke analyzer still rejects flat output; the fix
  raises the floor without removing the upper-bound check.

---

### Mouse input bindings (replaces current LMB-overloaded scheme)

The current scheme overloads LMB for camera orbit AND selection AND
forces `Ctrl+LMB` for dragging a selected manipulator. This breaks
direct manipulation: the Atlas / Hubo / G1 IK target end-effectors
look draggable but a plain LMB drag does nothing — the user has to
discover the Ctrl modifier from the README. Move to the
Unreal / Unity / Isaac Sim convention with Maya/Blender modifier-key
fallbacks for trackpad and single-button-mouse users.

**Core rule: LMB is for selecting AND dragging selectables.** No
Ctrl modifier needed. If a renderable is selectable (free-joint body,
`SimpleFrame` visual, IK target marker, robot end-effector handle), an
LMB drag on it directly moves it in the camera plane. The previous
"Ctrl+LMB to confirm you really want to drag" is gone — selectability
itself is the gate.

**Primary bindings:**

| Gesture                 | Action                                                |
| ----------------------- | ----------------------------------------------------- |
| LMB click               | pick / select single renderable under cursor          |
| LMB drag on selectable  | drag-move that selectable in the camera plane         |
| LMB drag on empty space | drag selection rectangle (or nothing, MVP-acceptable) |
| MMB drag                | pan                                                   |
| RMB drag                | orbit (rotate camera)                                 |
| Scroll wheel            | zoom                                                  |

**Modifier-key fallbacks (trackpad / one-button parity):**

| Gesture          | Action                   |
| ---------------- | ------------------------ |
| Alt + LMB drag   | orbit (Maya / Isaac Sim) |
| Shift + MMB drag | pan (Blender)            |
| Alt + RMB drag   | zoom (drag) (Maya)       |

**Axis-constrained manipulation (keep current behavior):**

| Gesture                            | Action                                          |
| ---------------------------------- | ----------------------------------------------- |
| X / Y / Z + LMB drag on selectable | constrain drag-move to that world axis          |
| Arrow / PageUp / PageDown          | nudge selected free-joint body or `SimpleFrame` |

(Note: the X/Y/Z keys are now sticky modifiers held during the drag,
not Ctrl-prefixed. The previous `Ctrl+X/Y/Z+LMB` becomes just
`X/Y/Z + LMB`.)

**Disambiguation rules (must be explicit in implementation):**

- A click is a press+release with cursor movement below a small
  threshold (e.g. 4 px). Above the threshold, the press starts a drag.
- Pick-test runs on **press**, not release: the renderable under the
  cursor at press time is the drag target. This makes drag-on-selectable
  feel responsive and avoids "I started dragging the marker but the
  camera moved instead" race conditions.
- Drag-on-selectable suppresses the camera controller for the duration
  of the press — orbit/pan modifiers do not fire while a manipulator
  drag is active.
- A press on empty space starts an LMB-drag-rect (post-MVP) or a no-op
  (MVP); it does NOT start an orbit. RMB is the only camera-rotate
  gesture in primary bindings.

**Implementation notes for Codex:**

- Land this as one focused slice inside the promoted `dart::gui`
  interaction layer (`dart/gui/interaction.hpp` and the GLFW input
  bridge under `dart/gui/experimental/detail/filament/input.cpp` until
  item 8b/c moves it). Do NOT change behavior in two places at once.
- The change touches: orbit-camera input handling, selection click
  handling, selection drag handling (now no-modifier), the README
  mouse-bindings table in `apps/dartsim/README.md`, and any end-user docs that
  mention mouse controls.
- Add a focused unit test that asserts each binding maps to the right
  controller action without requiring a graphics context (input event
  → expected `OrbitCameraDelta` / `SelectionRequest` /
  `SelectableDragRequest` output). The existing backend-hidden
  manipulation math tests are the model.
- Add an end-to-end gesture test for `--scene atlas-puppet`: simulate
  LMB-press on the right-hand IK target, drag 50 px, release; assert
  the IK target's world position changed. Repeat for `hubo-puppet` and
  `g1` to lock in the regression.
- Keep the headless smoke unaffected — it doesn't consume mouse input.
- Update `--help` output and the in-app status panel hint string.
- Document the trade-off in the changelog: this is a deliberate break
  from the MVP behavior; users who want LMB orbit have the
  Alt + LMB fallback.

**Acceptance:**

- Mouse interaction in `apps/dartsim/` and any example that opens a
  window matches the table above.
- A plain LMB drag on the Atlas / Hubo / G1 right-hand IK target
  moves the target (regression for the user-reported bug); no Ctrl
  modifier required.
- The fallback modifier keys work on a single-button trackpad.
- Click-without-drag never triggers an orbit.
- Drag-without-click never triggers a selection.
- Drag-on-selectable never triggers a camera move.

---

### 2026-05-15 supervisor follow-up: per-example sources + drop `experimental/`

> **Superseded by the pivot above.** Items 7 and 8 below are reframed as
> items 9' and 11' in the pivot. Tier-A/Tier-B partitioning is dropped;
> the per-scene checklist is repurposed as the per-example restoration
> checklist defined in the pivot block.

Two new threads, **in this order** (capture slice still comes first):

7. **Per-example real-source migration.** Right now every restored example
   directory contains only a 7-line `CMakeLists.txt` that compiles the shared
   `examples/gui_scene_launcher.cpp` with a `DART_GUI_DEFAULT_SCENE` define.
   That ships a binary on disk but does NOT migrate the original example's
   distinctive behavior (custom key callbacks, panel widgets, scenario reset
   logic, etc.) to the promoted `dart::gui` API — that behavior currently
   lives only inside the `--scene <name>` fixture in `scene_fixtures.cpp`.
   Decide which examples need their own `main.cpp` and migrate accordingly:
   - **Tier-A (real `main.cpp` required, full migration)**: `imgui`,
     `drag_and_drop`, `tinkertoy`, `lcp_physics`, `mimic_pendulums`,
     `coupler_constraint`, `add_delete_skels`, `vehicle`, `wam_ikfast`,
     `fetch`, `heightmap`, `atlas_simbicon`, `joint_constraints`,
     `operational_space_control`, `g1_puppet`, `hubo_puppet`, `atlas_puppet`,
     `point_cloud`, `simulation_event_handler`, `human_joint_limits`,
     `hybrid_dynamics`, `biped_stand`, `box_stacking`, `rigid_cubes`,
     `polyhedron_visual`, `rigid_shapes`, `mixed_chain`, `rigid_chain`,
     `rigid_loop`, `free_joint_cases`, `hardcoded_design`. These all had
     unique controls/widgets/scenario logic in their pre-cleanup OSG sources.
     Each needs its own `main.cpp` calling promoted `dart::gui::*` API
     (panel callbacks, key handlers, world hooks) — not the shared launcher
     macro.
   - **Tier-B (shared launcher OK)**: `hello_world`, `boxes`, `empty`,
     `simple_frames`, `capsule_ground_contact`, `soft_bodies`. These were
     visual-only legacy examples; the shared launcher + scene preset is
     sufficient.
   - **Required new public API surface to support Tier-A migration**: a
     `dart::gui::Panel` / `dart::gui::Tool` (or equivalent callback hook)
     interface so per-example `main.cpp` files can register custom
     widgets/key handlers without reaching into private
     `detail/filament/imgui_overlay.hpp`. This is the minimum panel/tool API
     that STEERING previously deferred — it's now load-bearing for Tier-A.
   - **Acceptance per migrated example**: example builds and links against
     `dart::gui` only (no `dart::gui::experimental::filament`, no
     `<filament/...>`, no `<GLFW/...>`, no `<imgui.h>`); a focused unit or
     headless smoke confirms the migrated controls fire; the corresponding
     `--scene <name>` fixture in `scene_fixtures.cpp` can then shrink to a
     visual-only fallback.

   #### Where the example code currently lives (and why this matters)

   This is the structural problem item 7 fixes:

   ```
   examples/<name>/CMakeLists.txt   ← 7 lines, calls macro with scene name
           ↓
   examples/gui_scene_launcher.cpp  ← 42 lines, ONE shared main() for all
           ↓ runtime: dart::gui::runApplication(argc, argv, "<scene>")
           ↓
   dart/gui/experimental/detail/filament/scenes.cpp     ← name → factory dispatch
           ↓ if (name == "atlas-puppet") return createAtlasPuppetScene();
           ↓
   dart/gui/experimental/detail/filament/scene_fixtures.cpp  ← 4082 LOC,
                                                              32 createXxxScene()
                                                              factories
   ```

   Every example's distinctive behavior — world setup, custom keybindings,
   panel widgets, scenario reset, force/torque overlays — lives as a
   `DartScene createXxxScene()` factory inside the **GUI library's private
   detail dir**, not under `examples/`. That is the inverse of the typical
   example layout: examples should _consume_ the library, not _be defined
   inside_ it. A user reading `examples/atlas_puppet/CMakeLists.txt` learns
   nothing about how to write their own `dart::gui` app — they see only
   `dart_build_gui_scene_example(atlas_puppet "atlas-puppet")`.

   The migration target is to invert this so each Tier-A example's behavior
   moves out of `scene_fixtures.cpp` into `examples/<name>/main.cpp`, calling
   the promoted `dart::gui` public API the way an external user would. Once
   migrated, the `createXxxScene()` factory is deleted from
   `scene_fixtures.cpp`. Tier-B (visual-only) examples keep using the shared
   launcher; their scene factories also stay, since they're useful as
   `dartsim --scene <name>` fixtures for headless smoke generation.

   #### Per-scene graduation checklist (Codex: keep this current)

   Mark `[x]` when the example has its own `main.cpp` consuming `dart::gui`
   public API and the corresponding factory in `scene_fixtures.cpp` has been
   deleted. Mark `[~]` for partial (own `main.cpp` exists but factory still
   present, or `main.cpp` still uses `experimental::filament`). Add the
   commit short-SHA next to each completed item.

   **Tier-A (need real `main.cpp` migrated to `dart::gui` public API):**
   - [ ] `add_delete_skels` — `createAddDeleteSkelsScene()` (live add/delete
         q/w controls)
   - [ ] `atlas_puppet` — `createAtlasPuppetScene()` (selectable IK targets)
   - [ ] `atlas_simbicon` — `createAtlasSimbiconScene()` (gait controller +
         perturbation panel)
   - [ ] `biped_stand` — shares `createHybridDynamicsScene()` (SPD control + perturbation)
   - [ ] `box_stacking` — shares `createBoxesScene()` (solver/gravity panel,
         custom key callbacks)
   - [ ] `coupler_constraint` — `createCouplerConstraintScene()` (status
         overlay + reset controls)
   - [~] `drag_and_drop` — `createDragAndDropScene()` (own promoted
     `dart::gui` `main.cpp` with context-aware panel exists; private
     factory and full promoted plane/axis-drag API migration remain)
   - [ ] `empty` — shares `createDragAndDropScene()` (custom world hooks + raw key-event callbacks)
   - [ ] `fetch` — `createFetchScene()` (panel, drag controls, mocap target
         update loop)
   - [ ] `free_joint_cases` — `createFreeJointCasesScene()` (numeric checks + reference-model controls)
   - [ ] `g1_puppet` — `createG1Scene()` (IK targets + support-polygon
         overlay; required for project-README animation)
   - [ ] `hardcoded_design` — `createHardcodedDesignScene()` (direct
         key-controlled joint motion + wireframe rendering)
   - [ ] `heightmap` — `createHeightmapScene()` (panel-driven sculpting +
         contact-alignment controls)
   - [ ] `hubo_puppet` — `createHuboPuppetScene()` (teleoperation widget +
         keyboard controls)
   - [ ] `human_joint_limits` — `createHumanJointLimitsScene()` (custom
         TinyDNN-backed arm/leg constraints)
   - [ ] `hybrid_dynamics` — `createHybridDynamicsScene()` (scripted joint
         commands + harness toggling)
   - [~] `imgui` — uses default MVP scene + custom-widget extension points
     (own promoted `dart::gui` `main.cpp` with panel callback exists;
     broader tool API remains)
   - [ ] `joint_constraints` — `createJointConstraintsScene()` (perturbation
         shortcuts + harness toggling)
   - [ ] `lcp_physics` — `createLcpPhysicsScene()` (solver controls, plots,
         scenario switching, frame recording)
   - [ ] `mimic_pendulums` — `createMimicPendulumsScene()` (ImGui solver/debug
         table)
   - [ ] `mixed_chain` — `createMixedChainScene()` (keyboard-applied external
         forces)
   - [ ] `operational_space_control` — `createOperationalSpaceControlScene()`
         (drag-and-drop axis constraints)
   - [ ] `point_cloud` — `createPointCloudScene()` (robot-mesh sampling
         controls)
   - [ ] `polyhedron_visual` — `createPolyhedronScene()` (convex hull +
         wireframe inspection)
   - [ ] `rigid_chain` — `createRigidChainScene()` (custom per-step damping
         hook)
   - [ ] `rigid_cubes` — shares `createBoxesScene()` (directional force
         controls + frame-recording options; load-bearing for legacy CI
         workflow)
   - [ ] `rigid_loop` — `createRigidLoopScene()` (damping + constraint
         setup)
   - [ ] `rigid_shapes` — uses default MVP scene (shape spawning, contact
         toggles, collision-detector controls)
   - [ ] `simulation_event_handler` — `createSimulationEventHandlerScene()`
         (force/torque controls + force-arrow visualization)
   - [~] `tinkertoy` — `createTinkertoyScene()` (own promoted `dart::gui`
     `main.cpp` with context-aware panel exists; private factory plus
     full mouse picking and block-add/delete controls remain)
   - [ ] `vehicle` — `createVehicleScene()` (live throttle + steering
         controls)
   - [ ] `wam_ikfast` — `createWamIkFastScene()` (IKFast solver + drag modes + posture reset)

   **Tier-B (shared launcher OK; factory stays for `--scene` use):**
   - [ ] `boxes` — visual-only multi-box (factory stays as
         `--scene boxes` developer fixture)
   - [ ] `capsule_ground_contact` — visual-only contact setup
   - [~] `hello_world` — single dynamic blue box; canonical "first example"
     (own promoted `dart::gui` `main.cpp` with public `WorldPtr` handoff
     exists; private fixture deletion remains)
   - [ ] `simple_frames` — visual `SimpleFrame` hierarchy
   - [ ] `soft_bodies` — visual soft-body SKEL playback

   **Migration ordering recommendation:**
   1. `imgui` first — it forces the `dart::gui::Panel` / `dart::gui::Tool`
      public API into existence, which unblocks every other Tier-A item.
   2. `drag_and_drop` next — it forces a clean public manipulation API.
   3. `hello_world` (Tier-B) third — establishes the canonical user-facing
      `main.cpp` template that other Tier-B examples copy.
   4. Then batch the remaining Tier-A items by family: rigid demos, joint
      demos, robot demos, fixture demos. One PR per family is fine; the
      branch already does not open PRs, so commit boundaries are the unit
      of review.

   **Stop condition for item 7:** every Tier-A example has its own
   `main.cpp` consuming only promoted `dart::gui` public API; every
   migrated factory is removed from `scene_fixtures.cpp`; remaining
   factories in `scene_fixtures.cpp` are explicitly Tier-B / dev-only.
   `scene_fixtures.cpp` should drop from 4082 LOC to <1500 LOC after this
   migration completes.

8. **Drop the `experimental/` segment from code paths.** Now that
   `dart::gui::*` is the promoted namespace and `dart/gui/*.hpp` are the
   stable headers, the physical `dart/gui/experimental/` directory and the
   `dart/gui/experimental/detail/filament/` private path are debt. Sequence:
   - **8a**: Move public compatibility shims `dart/gui/experimental/*.hpp` →
     thin `#include`-only headers that re-export from `dart/gui/*.hpp`. They
     should keep working for one release cycle, but have ZERO declarations
     of their own. Mark them `[[deprecated]]` with a one-cycle removal
     window.
   - **8b**: Move the private backend implementation
     `dart/gui/experimental/detail/filament/*` → `dart/gui/detail/filament/*`
     (one mechanical rename, preserve commit history with `git mv`). Update
     all `#include "dart/gui/experimental/detail/filament/..."` paths in the
     same commit. The boundary-guard test
     (`UNIT_gui_FilamentSceneExtraction`) and the configure-time check in
     `dart_gui_filament_add_example()` need their hardcoded paths updated
     in the same commit.
   - **8c**: Move the renderer-independent implementation files
     `dart/gui/experimental/{viewer,scene,renderable,interaction,debug,geometry,profile,shape_descriptions}.cpp`
     → `dart/gui/*.cpp` to match where the headers now live. After this,
     `dart/gui/experimental/` contains only `[[deprecated]]` shim headers.
   - **8d**: Remove the `experimental` token from the dartpy module name
     (`dartpy.gui.experimental` → `dartpy.gui`), with a `dartpy.gui.experimental`
     proxy module that emits a `DeprecationWarning` and re-exports.
   - **8e**: After one release cycle (NOT in this branch), delete the shims
     and `dart/gui/experimental/` entirely. This branch should leave
     `experimental/` containing only deprecation shims so the path itself
     becomes a reading-cue for "this name is going away."
   - **Constraint for 8b**: do NOT change file boundaries inside
     `detail/filament/*` during the rename — the over/under-fragmentation
     review issues are a separate slice from the directory rename. One
     mechanical concern per commit.
   - **CMake helper rename in same slice**: `dart_gui_filament_add_example`
     stays under `dart/gui/detail/filament/filament_sources.cmake`. The
     macro's name is acceptable (it names the backend, not the path).

The order is: capture slice (item 6 above) → item 7 (per-example real
sources, with the panel/tool API as a precondition) → item 8 (drop
`experimental/`). Item 8 should NOT start before item 7 lands a real
per-example `main.cpp` for at least 3 Tier-A examples, because the
deprecation shims need to know which `dart::gui` symbols are actually
consumed externally.

### Open issues raised by the parallel review pass

- **[verifier]** Per-scene headless smokes (~30 scenes) all run
  `ANALYSIS_MODE basic` (nonzero-pixel only). The "dark/mid/bright +
  luminance spread" gate documented in `03-milestones.md` only applies to
  the single aggregate `EXAMPLE_dartsim_headless_smoke`. Either upgrade the
  per-scene smokes to `contrast` mode or amend the docs to match reality.
- ~~**[verifier]** `scanHeadersForBackendTokens` has a documented hook for
  promoted `dart/gui/*.hpp` headers, but
  `guiHeaderDirectoriesForBackendTokenScan()` returns only the experimental
  directory. Now that `dart/gui/application.hpp` is promoted, register
  `dart/gui` in that hook so the guard becomes live, not aspirational.~~
  Local repair adds `dart/gui` to that scan.
- **[code-reviewer] over-fragmentation** under
  `dart/gui/experimental/detail/filament/`: collapse the per-frame trio
  `simulation_stepper.{hpp,cpp}` (72/110), `frame_renderer.{hpp,cpp}`
  (67/112), and `frame_viewport.{hpp,cpp}` (75/85) into one `frame_loop`
  unit (~520 LOC); fold `application_teardown.{hpp,cpp}` (61/73) back into
  `application.cpp`; consider merging `scene_startup.*` + `scene_frame.*`
  into `scene_lifecycle`. 19 of 49 files in that directory are <100 LOC.
- **[code-reviewer] under-fragmentation**: `scene_fixtures.cpp` is **4082
  LOC** and `scene_requirements.cpp` is **1844 LOC** with a 130-field
  `SceneContentCounts` God Struct that is a Shotgun-Surgery magnet. Split
  fixtures by family (rigid_demos, joint_demos, robot_demos, geometry_demos)
  and replace `SceneContentCounts` with a `std::unordered_map<SceneKind,
std::size_t>`.
- **[code-reviewer] encapsulation leaks under `detail/`**: `scene_frame.hpp`,
  `frame_viewport.hpp`, `renderable_resources.hpp`, and `scene_startup.hpp`
  expose `GLFWwindow*` / `::filament::Engine` / `::filament::View&` /
  `ImGuiIO&` in public function signatures. Move these behind pImpl or a
  renderer-context handle so `detail/` becomes a real seam, not just a
  directory.
- **[critic]** Reconcile this file with `10-active-execution.md` regarding
  the promoted application header. Decision 4 above is the resolution;
  record an explicit cross-reference in both docs so future sessions don't
  reopen the question.
- ~~**[critic] no branch-level definition of done**. Add an explicit "Branch
  is done when:" checklist to `10-active-execution.md` covering: (a) zero
  `experimental` token in promoted names, (b) Linux headless CI green,
  (c) the agreed N restored example binaries exist and produce screenshots,
  (d) no `filament_gui` user-facing strings remain in any maintained file.~~
  Local repair adds `Branch Done Checklist` to `10-active-execution.md`.

### How the supervisor uses this section

- Each new live human direction gets translated into the lists above (not
  into a new file). Codex reads `STEERING.md` after `RESUME.md` per the
  existing convention.
- When a numbered step in "Order of operations" lands, mark it `~~done~~`
  with a one-line evidence pointer (commit short SHA + file). Do not delete
  it for one review cycle so review agents can tell what just shipped.
- The "Decisions in force" block is sticky. If you need to change one, flag
  it under "Open issues" and wait for the next supervisor pass to confirm.

---

## Original 2026-05-14 evaluator pass (kept for history)

## Current branch update

Since this steering note was first written, the post-MVP branch landed the
recommended maintainability slices:

- `dart/gui/experimental/debug.cpp` owns debug descriptor generation.
- `dart/gui/experimental/geometry.hpp` and `.cpp` own backend-hidden mesh
  builders for generated primitive and descriptor geometry.
- `dart/gui/experimental/detail/filament/scenes.hpp` and `.cpp` own scene
  option parsing and dispatch, while `scene_fixtures.hpp` and `.cpp` own
  reusable DART world fixtures.
- `dart/gui/experimental/interaction.cpp` owns picking, plane/axis-drag,
  frame-translation, and renderable set update planning helpers.
- `dart/gui/experimental/renderable.hpp`, `interaction.hpp`, and `debug.hpp`
  now split the constrained public surface while `scene.hpp` remains an
  aggregate compatibility include.
- `dart/gui/experimental/shape_descriptions.cpp` owns `describeShape`, leaving
  `renderable.cpp` focused on renderable identity, resource versioning, and
  world extraction.
- `dart/gui/experimental/detail/filament/panel.hpp` and `.cpp` own the
  private built-in status panel rendering while promoted panel/tool API design
  remains deferred.
- `dart/gui/experimental/detail/filament/debug_overlay.hpp` and `.cpp` own
  debug-line overlay option defaults and refresh/cleanup for static, contact,
  and selection overlays, including selected-renderable descriptor lookup.
- `dart/gui/experimental/detail/filament/imgui_overlay.hpp` and `.cpp` own
  ImGui context setup, style/font initialization, current IO access, draw-data
  upload, and overlay teardown.
- `dart/gui/experimental/detail/filament/ui_frame.hpp` and `.cpp` own
  per-frame UI input sync, built-in status panel dispatch, panel-triggered
  debug-overlay refreshes, and ImGui overlay upload.
- `dart/gui/experimental/detail/filament/application_teardown.hpp` and `.cpp`
  own ordered teardown of the private Filament application resources.
- `dart/gui/experimental/detail/filament/frame_renderer.hpp` and `.cpp` own the
  private render-frame, skipped-frame, screenshot-request, and frame-accounting
  policy.
- `dart/gui/experimental/detail/filament/frame_viewport.hpp` and `.cpp` own
  private per-frame framebuffer sizing, ImGui display metrics,
  viewport/camera updates, and camera-controller suppression while selection
  or the built-in UI consumes the pointer.
- `dart/gui/experimental/detail/filament/screenshot.hpp` and `.cpp` own the
  private Filament screenshot readback, wait/save finalization, and screenshot
  profile accounting.
- `dart/gui/experimental/detail/filament/input.hpp` and `.cpp` own private
  GLFW/ImGui input bridging, orbit-camera scroll callback attachment,
  camera-controller input translation, and application hotkey polling/key-edge
  state.
- `dart/gui/experimental/detail/filament/selection.hpp` and `.cpp` own the
  private selection controller state and GLFW event translation for keyboard
  nudging, click selection, and Ctrl-left dragging.
- `dart/gui/experimental/detail/filament/native_window.hpp` and `.cpp` own the
  private GLFW initialization/window lifecycle, window-close loop predicate, and
  platform-native handle selection.
- `dart/gui/experimental/detail/filament/renderable_sync.hpp` and `.cpp` own
  private initial scene renderable creation, scene-entity synchronization,
  unsupported-renderable logging, and per-frame renderable
  transform/selection/shadow application.
- `dart/gui/experimental/detail/filament/materials/` owns the Filament material
  shader sources, and `dart/gui/experimental/detail/filament/testing/` owns the
  opt-in headless smoke CMake/Python helpers.
- `dart/gui/experimental/detail/filament/filament_sources.cmake` owns the
  private backend source/header list, dependency wiring, target setup, material
  inputs, material header generation, and smoke-test registration that the
  minimal example CMake file consumes.
- `dart/gui/experimental/detail/application.hpp` owns the generic private GUI
  application entry point consumed by `examples/filament_gui/main.cpp`; the
  backend-specific application entry point stays under
  `dart/gui/experimental/detail/filament`.
- `dart/gui/experimental/detail/filament/scene_requirements.hpp` and `.cpp`
  own created-renderable content counting for startup validation.
- `dart/gui/experimental/detail/filament/scene_startup.hpp` and `.cpp` own
  initial scene extraction, startup validation, first renderable
  synchronization, and startup debug-overlay validation.
- `dart/gui/experimental/detail/filament/scene_frame.hpp` and `.cpp` own
  per-frame simulation stepping, descriptor extraction, scene synchronization,
  selected-renderable interaction updates, selection-debug refresh, and
  orbiting-light updates.
- `dart/gui/experimental/detail/filament/simulation_stepper.hpp` and `.cpp` own
  the bounded realtime simulation step-count accumulator, world-step
  application, lifecycle marking, and simulation profile accounting.
- `dart/gui/experimental/detail/filament/application.cpp` now depends on the
  private GUI runtime surfaces directly instead of retaining stale DART scene
  fixture/import dependencies after the `scenes.*` split.
- `dart/gui/experimental/profile.hpp` and `.cpp` own viewer-loop profile
  accumulation, and `dart/gui/experimental/detail/filament/render_context.*`
  plus `screenshot.*`, `render_environment.*`, `textures.*`, and
  `renderable_*.*` now own the first Filament-private runtime slices outside
  the example tree.

The remaining guidance is still useful for promotion sequencing, but the
original "next" ordering below is partially complete.

## What just shipped (verified)

- Commit `877fababbc6` "Split experimental GUI viewer helpers" landed:
  `dart/gui/experimental/viewer.{hpp,cpp}` (228 + 432 lines) is now the
  formal home for `RunOptions`, `ViewerLifecycleState`, `OrbitCamera*`,
  `PerspectiveProjection`, `PickRay`, `writeRgbaPpm`, frame
  accounting, and pause/step helpers. `scene.hpp` remains an aggregate
  compatibility include for the constrained experimental surface.

### Independent verification results

- **Public-header purity (HARD gate):** PASS. `viewer.hpp` includes only
  Eigen + STL + `dart/gui/export.hpp`. Zero forbidden tokens
  (`filament::`, `GLFW`, `ImGui`, `osg::`, `Vulkan`, `Metal`, `Raylib`, etc.).
  `tests/unit/gui/test_filament_scene_extraction.cpp:232-250` uses a
  `directory_iterator` glob over `*.hpp`, so the new header is picked up
  automatically by `ExperimentalPublicHeadersStayBackendHidden`.
- **Build system:** `dart/gui/experimental/CMakeLists.txt` uses
  `file(GLOB hdrs "*.hpp")` and `file(GLOB srcs "*.cpp")`. No CMake edit
  required — focused experimental headers and sources are picked up
  automatically after reconfigure.
- **Behavior preservation:** `git show 877fababbc6 -- scene.cpp` showed the
  then-current file had zero added lines, only deletions. Pure mechanical move.
  Bodies in
  `viewer.cpp` byte-equivalent to deleted bodies.
- **API symmetry:** All callers (tests, dartpy bindings, example) reach
  the moved symbols transitively through `scene.hpp`. No downstream
  include updates required.

**Verdict:** the viewer split is safe to keep. No follow-up code edits needed.

## Completed debug split

`dart/gui/experimental/debug.cpp` now owns debug-line generation
(`makeGridDebugLines`, `makeFrameDebugLines`, `makeSelectionDebugLines`,
`makeInertiaDebugLines`, `makeCollisionShapeDebugLines`,
`makeSupportPolygonDebugLines`, `extractContactDebugLines`,
`extractDebugLines`, plus internal helpers). Its public surface remains in
`scene.hpp` for source compatibility.

## Completed scene interface split

`scene.hpp` is now only an aggregate compatibility include. Focused public
headers carry the constrained concepts: `renderable.hpp` for descriptors and
extraction planning, `interaction.hpp` for picking/dragging/translation, and
`debug.hpp` for debug-line descriptors. `shape_descriptions.cpp` owns
`describeShape`, while `renderable.cpp` owns renderable IDs, render-resource
version hashing, and `extractRenderables`.

## Original recommended priority order

1. **Completed: promote example-local geometry builders into
   `dart-gui-experimental`.**
   `dart/gui/experimental/geometry.hpp` and `.cpp` now expose backend-hidden
   mesh buffers and builders consumed by the Filament example. Graphics-free
   tests cover the generated geometry path.
   - Anchors: `08-north-star-migration.md` Phase C deliverables and exit
     criterion ("The Filament example is a thin consumer of the GUI
     library rather than the owner of renderer architecture");
     `07-completion-audit.md` row 34 "Provide renderer-hidden scene
     extraction" — currently Satisfied for the constrained API but the
     geometry path is the obvious next descriptor surface.
2. **Completed: move scene fixtures under private GUI detail.**
   `dart/gui/experimental/detail/filament/scenes.hpp` and `.cpp` now own scene
   option parsing and dispatch, while
   `dart/gui/experimental/detail/filament/scene_fixtures.hpp` and `.cpp` own
   MVP, hello-world, boxes, hardcoded-design, rigid-chain, rigid-loop,
   mixed-chain, coupler-constraint, add-delete-skels, vehicle,
   hybrid-dynamics, joint-constraints, free-joint-cases, mimic-pendulums,
   atlas-puppet, operational-space-control, drag-and-drop, simple-frames, soft-bodies,
   point-cloud, capsule-ground-contact, simulation-event-handler, polyhedron,
   heightmap, and G1 DART world fixtures. The `biped_stand` runner reuses the
   hybrid-dynamics fullbody fixture rather than adding duplicate scene code.
   `dart/gui/experimental/detail/filament/application.hpp` and `.cpp` now own
   Filament frame orchestration and the built-in panel, leaving `main.cpp` as a
   minimal entry point.
   `dart/gui/experimental/detail/filament/scene_requirements.hpp` and `.cpp`
   now own scene fixture requirement counting and MVP/G1/hello-world/boxes/
   hardcoded-design/rigid-chain/rigid-loop/mixed-chain/coupler-constraint/
   add-delete-skels/vehicle/hybrid-dynamics/joint-constraints/
   free-joint-cases/mimic-pendulums/atlas-puppet/operational-space-control/
   drag/simple-frames/soft-bodies/point-cloud/
   capsule-ground-contact/simulation-event-handler/polyhedron/heightmap
   validation gates. Runner-only reuse, such as `biped_stand` using
   hybrid-dynamics, should stay in the runner/docs layer unless it needs a
   distinct scene fixture.
3. **Completed: generalize the public-header leakage scan.**
   `UNIT_gui_FilamentSceneExtraction` now routes the backend-token check
   through `scanHeadersForBackendTokens`, and
   `guiHeaderDirectoriesForBackendTokenScan` is the reserved hook for promoted
   `dart/gui/*.hpp` headers before first-class promotion.

## Doc hygiene notes

The docs-sync slice should keep these items current as commits land:

- `RESUME.md` should mention the concrete implementation split so resumed
  sessions do not treat `scene.hpp` as the only experimental implementation
  surface.
- `09-legacy-surface-audit.md` should describe the experimental headers in
  `dart/gui/experimental/` as the clean API fence, not only `scene.hpp`.
- `02-mvp-example.md` and `05-testing.md` should distinguish scene extraction,
  interaction helpers, and viewer-runtime helpers.
- `CHANGELOG.md` should mention that `scene.hpp` remains an aggregate
  compatibility include while implementation moves into focused `.cpp` files.
- `07-completion-audit.md` should keep external packaging and macOS/Windows
  platform gates framed as tracked promotion blockers, not branch-local work.

## Explicitly do NOT do next

- **Do not draft the promoted `dart::gui::Viewer` / `ViewerOptions`
  public API yet** (the sketch in `08-north-star-migration.md:331-344`).
  Phase H requires Phases C, E, F, G to be substantially complete first.
  The private implementation still owns Filament frame orchestration and
  built-in overlay policy. Defining the promoted API now would either bake in
  MVP-shaped concessions or force a second rewrite when Phase C extraction
  reveals the right resource boundaries.
- **Do not add panel/tool abstractions** (`08-north-star-migration.md`
  §11). Keep the built-in ImGui panel/tool policy MVP-scoped until Phase E
  demands it.
- **Do not start macOS/Windows port work** — no machines in this
  branch, not actionable for Codex.
- **Do not modify the conda-forge feedstock PR** — external dependency,
  Codex can only track via `gh pr checks 33297 --repo conda-forge/staged-recipes`.

## Stay-in-lane summary

The Phase C/D promotion lane (move the example from "owns the
renderer architecture" to "thin consumer of the GUI library") is
the highest-leverage path right now. Every commit in the recent
log fits that lane: GUI scale → projection → nudge → camera
controller → viewer split → debug-line split → geometry builders → scene
fixtures → interaction helpers. Stay in this lane.
The completion metric for this lane is stronger than direct include hygiene:
any surviving `examples/filament_gui/` directory should contain only a minimal
`main.cpp` entry-point shell plus unavoidable build/docs files, while renderer
setup, frame lifecycle, resources, synchronization, capture, overlays, input
translation, and fixture logic live under `dart::gui` or private GUI
implementation units.
The promoted `dart::gui` API shape will fall out naturally from what remains
private implementation detail after the example entry point bottoms out.
