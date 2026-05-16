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
- Correction: these checkpoints are not the end of example restoration. Every
  remaining `dart_build_gui_scene_example(...)` target, including
  `examples/fetch`, is still incomplete until it owns a real public-API source.
- `19eae1d2d91` restores `examples/fetch` as a source-defined public-API
  example with `ApplicationOptions::preStep`. `fdd9d248033` restores the
  rigid/constraint batch (`rigid_chain`, `rigid_loop`, `mixed_chain`,
  `coupler_constraint`, `add_delete_skels`, and `rigid_shapes`). The next
  active checkpoint is the joint/dynamics batch: `hybrid_dynamics`,
  `biped_stand`, `joint_constraints`, `free_joint_cases`, and
  `human_joint_limits`.

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

## 2026-05-15 Round 12 — Branch is NOT done; close the gap on `apps/dartsim/` first

**Trigger:** Codex's `3b4ac2cae7b` ("Update GUI restoration checkpoint
status") and prior status edits read as "this slice is wrapping up."
The supervisor verified the working tree against the pivot's stop
conditions and **none of items 7' through 11' are actually complete.**
This round is a hard reset on the completion narrative. Do not commit
a "branch done" or "session done" message.

The user's two directives this round:

- **"Codex is trying to claim the session is done, where it has tons of
  TODOs left."** → enforce the verification-before-completion rule.
- **"Work fully on `apps/dartsim/` to make the application perfect."**
  → `apps/dartsim/` is the highest-priority remaining surface; the
  per-example migration moves to a parallel later batch.

### Verified evidence (working tree as of 3b4ac2cae7b)

```
$ ls dart/gui/experimental/*.{hpp,cpp} | wc -l
   16 files (real declarations + definitions, NOT [[deprecated]] shims)

$ find dart/gui/experimental/detail/filament -type f \( -name '*.hpp' -o -name '*.cpp' \) | wc -l
   48 files (the entire Filament backend still lives here)

$ ls apps/dartsim/
   CMakeLists.txt  main.cpp  README.md
   ↑ no app/ subdir, no panels, no scene_loader, no timeline,
     no docking_layout, no log_panel, no recording.cpp.

$ wc -l apps/dartsim/main.cpp
   ~40 lines — same as the old example main; no positional-arg
   parsing, no scene-loader dispatch, no panel registration.

$ for d in examples/*/; do test -f "$d/main.cpp" || echo "MISSING: $d"; done
   29 example dirs are still macro shims with NO main.cpp.

$ ls examples/gui_scene_launcher.cpp examples/gui_scene_example.cmake
   both present — macro shim NOT removed.
```

### Stop-condition table (the only valid "done" gate)

| Pivot deliverable                                                     | Actual state                                                                                                                                        | Status  |
| --------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------- | ------- |
| `apps/dartsim/` opens URDF/SDF/MJCF/SKEL from CLI                     | `main.cpp` is `dart::gui::runApplication(argc, argv)`; no positional-arg parsing, no `dart::io` dispatch                                            | **GAP** |
| `apps/dartsim/` shows docked panel layout                             | No `apps/dartsim/app/` dir; no panels; ImGui Docking not enabled                                                                                    | **GAP** |
| `apps/dartsim/` plays simulation through Timeline                     | No timeline panel                                                                                                                                   | **GAP** |
| `apps/dartsim/` captures image sequence through Recording             | `--out` PNG-sequence capture restored at `05c59c17ad2`; not surfaced as a Recording panel                                                           | PARTIAL |
| `apps/dartsim/` Scene tree, Inspector, Log, Console panels            | None exist                                                                                                                                          | **GAP** |
| Every `examples/<name>/` has its own real `main.cpp`                  | 14 of 43 example dirs have `main.cpp`; 29 are macro shims                                                                                           | PARTIAL |
| No shared launcher, no `DART_GUI_DEFAULT_SCENE` macro                 | `examples/gui_scene_launcher.cpp` + `gui_scene_example.cmake` still present and used                                                                | **GAP** |
| `scene_fixtures.cpp` shrunk to zero (or moved to test fixtures)       | Still 4082 LOC of `createXxxScene()` factories under `dart/gui/experimental/detail/filament/`                                                       | **GAP** |
| `dart/gui/experimental/` contains only `[[deprecated]]` shim headers  | 16 files of real declarations + 48 files of Filament backend live here                                                                              | **GAP** |
| `dart::gui::Panel` / `dart::gui::Tool` is the public API examples use | `panel.hpp` exists at `38a53e9b86d` / `c3b303ad267`; only consumed inside the dartsim built-in panel, not by per-example sources or `apps/dartsim/` | PARTIAL |
| Mouse: LMB drags selectables (atlas_puppet IK targets)                | Steering written; no implementing commit                                                                                                            | **GAP** |
| Default lighting: shadow visible by default, orbit opt-in             | Steering written; no implementing commit                                                                                                            | **GAP** |

### Hard rules for the next 5+ rounds

1. **No "session done" / "branch done" / "wrap-up" commit messages
   until every row above flips to DONE.** "Checkpoint N landed" is
   fine; "wrap up" framing is not.
2. **The `~~strikethrough~~` convention requires `~~done~~` ONLY when
   the listed deliverable is fully implemented in code, not when a
   related precursor commit landed.** Items 7'..11' above are NOT
   eligible for `~~done~~` until the working-tree evidence flips.
3. **Status-edit-only commits ("Update GUI restoration checkpoint
   status") must include a one-line GAP summary in the commit body
   listing what is still NOT done.** This prevents the status doc
   from drifting into completion-theater.
4. **Every non-status round produces at least one named code commit
   touching the surface it claims to advance**, before any status
   edit on that row.

### Order of operations (corrective, replaces 7'..11' ordering)

The user's directive is to make `apps/dartsim/` perfect first. The
per-example restoration (R12-5) moves AFTER `apps/dartsim/` is
solid, because the panel/tool API surface needed for examples is
the same surface needed for `apps/dartsim/` panels — landing it once
in the app is the cheapest way to validate it before fanning out to
29 examples.

R12-1. **Implement the mouse-binding fix.** LMB selects + drags
selectables (no Ctrl required); RMB orbit; MMB pan; scroll
zoom; modifier fallbacks (Alt+LMB orbit, Shift+MMB pan,
Alt+RMB zoom-drag); X/Y/Z constraint on selectable drag
(no Ctrl). Acceptance: `pixi run ex dartsim --scene
       atlas-puppet`, plain LMB drag on right-hand IK target moves
the target. Lock in via the gesture test described in the
Mouse Bindings section above.

R12-2. **Implement the default lighting fix.** IBL ×1.6, opposite
fill light at 30% no-shadow, SH ambient floor ~0.18,
orbit-light default OFF (opt-in via `--orbit-light` or panel
checkbox). Acceptance: `pixi run ex dartsim` (no flags)
shows clear ground shadow AND legible shadow-side geometry
AND no orbiting light unless opted in.

R12-3. **Make `apps/dartsim/` a real application.** This is the
biggest single piece of remaining work. Five sub-steps,
each its own commit:

- **R12-3a — CLI + scene loader.** `apps/dartsim/main.cpp` parses
  a positional world-file argument. `apps/dartsim/app/scene_loader.cpp`
  dispatches by extension (`.urdf` → `dart::io::DartLoader`,
  `.sdf` → `SdfParser`, `.mjcf` → `MjcfParser`, `.skel` →
  `SkelParser`, `.dartsim` → JSON/TOML project file). No arg
  opens an empty world.

- **R12-3b — Docking layout.** `apps/dartsim/app/docking_layout.cpp`
  enables ImGui Docking and configures a central 3D viewport
  dock node with side-docked spaces for Scene, Inspector,
  Timeline, Log, Recording. The 3D viewport is the actual
  Filament render target, not an ImGui image proxy if possible.

- **R12-3c — Scene + Inspector panels.** Scene tree shows
  skeletons → joints → bodies in a tree view. Inspector shows
  properties of the selection (name, transform, mass, geometry
  summary). Selection in the Scene panel highlights the body
  in the 3D view; clicking in the 3D view updates the Scene
  panel selection.

- **R12-3d — Timeline + Log panels.** Timeline: play / pause /
  step / reset / time display / framerate. Log: capture from
  `dart::common::Logger`, scrollable, with severity filter
  buttons. Both consume `dart::gui::Panel`; do NOT reach into
  `detail/filament/imgui_overlay.hpp` from `apps/`.

- **R12-3e — Recording panel.** Wraps the existing `--out` PNG
  sequence and `--screenshot` PPM paths in a UI: Output dir
  picker, frame-count input, format dropdown, Start/Stop
  button. The CLI flags continue to work for headless use.

Stop condition for R12-3: `pixi run ex dartsim
   /path/to/world.urdf` opens the world, shows the docked layout,
plays simulation through Timeline, captures an image sequence
through Recording. ImGui Docking enabled (this lifts the
original Decision 5 exclusion specifically for `apps/dartsim/`).

R12-4. **Promote any panel API surface gaps surfaced by R12-3.**
If R12-3 had to add private-detail accommodations for
Scene-tree iteration, selection callbacks, log capture, or
3D-viewport docking, fold those back into `dart/gui/panel.hpp`
/ `dart/gui/tool.hpp` so external consumers can do the same
thing. This is the natural API-shape feedback step.

R12-5. **Migrate the 29 macro-shim examples to real `main.cpp`** in
batches of 5–8 per commit, in this order: - **rigid family (6)**: rigid_chain, rigid_loop, rigid_shapes,
mixed_chain, coupler_constraint, add_delete_skels - **joint/dynamics family (5)**: joint_constraints,
free_joint_cases, hybrid_dynamics, biped_stand,
human_joint_limits - **robot/IK family (7)**: atlas_puppet, atlas_simbicon,
hubo_puppet, g1_puppet, fetch, wam_ikfast,
operational_space_control - **geometry/visual family (11)**: polyhedron_visual,
heightmap, point_cloud, soft_bodies,
simulation_event_handler, vehicle, lcp_physics,
mimic_pendulums, hardcoded_design, empty, tinkertoy
After each batch: delete the matching `createXxxScene()` from
`scene_fixtures.cpp` and report the LOC drop in the commit
body.

R12-6. **Delete the macro shim** (`examples/gui_scene_launcher.cpp`,
`examples/gui_scene_example.cmake`) in the same commit that
lands the last per-example `main.cpp`. Update
`examples/CMakeLists.txt` to use plain `add_subdirectory()`.

R12-7. **Drop `experimental/`**: - 7a: replace `dart/gui/experimental/*.hpp` with thin
`[[deprecated]]` re-exports of `dart/gui/*.hpp`. - 7b: `git mv dart/gui/experimental/detail/filament/* dart/gui/detail/filament/` + update all `#include` paths + the boundary-guard test +
the configure-time check, all in one commit. - 7c: `git mv dart/gui/experimental/{viewer,scene,renderable,interaction,debug,geometry,profile,shape_descriptions}.cpp dart/gui/`. - 7d: rename `dartpy.gui.experimental` → `dartpy.gui` with a
deprecation-shim proxy module.
Each in its own commit; do NOT bundle.

### Stop condition for Round 12

Every row in the table above reads `DONE`. Until then, all of items
7'–11' (and now R12-1..R12-7) remain open. The pivot's stop
condition remains authoritative; this round only enforces it.

### What Codex should NOT do next

- Do not edit STEERING.md or 10-active-execution.md to mark items
  as ~~done~~ that fail the table's verification.
- Do not open a PR. (Reaffirms the existing rule.)
- Do not start a "celebration commit" / "wrap-up doc" commit.
- Do not bundle the lighting fix and the mouse-binding fix into one
  commit — they touch different layers and deserve independent
  smoke gates.
- Do not implement R12-3 by reaching into private `detail/filament/`
  panel internals — `apps/dartsim/app/` must consume `dart::gui::Panel`
  via the public header. If the public Panel API is missing surfaces
  needed for Scene/Timeline/Log panels, raise a new Open Question
  in this section rather than adding `detail/` includes from `apps/`.
- Do not start R12-5 (per-example migration) before R12-3 (apps/dartsim
  panel build-out) is substantially landed. The user explicitly
  prioritized `apps/dartsim/` perfection.

### How users run `dartsim` (for reference; place in apps/dartsim/README.md too)

```bash
# Default scene, windowed
pixi run ex dartsim

# Open a specific world file (after R12-3a)
pixi run ex dartsim path/to/world.urdf

# Developer fixture (kept for headless smoke + manual diagnosis)
pixi run ex dartsim --scene atlas-puppet
pixi run ex dartsim --scene g1 --gui-scale 2

# Headless screenshot
pixi run ex dartsim --headless --frames 60 --screenshot /tmp/dartsim.ppm

# All headless smokes
pixi run test-filament-gui-smoke

# Restored legacy examples (each is its own binary)
pixi run ex hello_world
pixi run ex drag_and_drop
pixi run ex atlas_puppet     # currently still macro-shim — see R12-5
```

(`pixi run ex` resolves via `scripts/run_cpp_example.py` → `bin/dartsim`
or `bin/<example_name>`. The pixi task is defined at `pixi.toml:860`.)

### R12-8 — Promote `dartsim` to a top-level pixi task

Add a top-level pixi task so the canonical user invocation is just:

```bash
pixi run dartsim                          # default scene, windowed
pixi run dartsim path/to/world.urdf       # opens the world (after R12-3a)
pixi run dartsim --scene atlas-puppet     # developer fixture
pixi run dartsim --headless --frames 60 --screenshot /tmp/dartsim.ppm
```

Implementation:

- Add `dartsim = { cmd = "python scripts/run_cpp_example.py dartsim",
depends-on = ["config"] }` to `pixi.toml` near the existing
  `ex = ...` task at line 860.
- All positional / option args after `pixi run dartsim` flow through
  to the binary (pixi already passes trailing args through).
- Keep `pixi run ex dartsim` working — it stays the developer / runner
  entry point that handles smoke-test pairing and binary discovery.
  The new `pixi run dartsim` is the user-friendly alias.
- Document the new alias in `apps/dartsim/README.md` AND in the top-
  level `README.md` user-facing quickstart (where any prior
  `pixi run ex filament_gui` etc. should already have been swept).
- Acceptance: `pixi run dartsim --help` prints the dartsim CLI help,
  and `pixi run dartsim --scene boxes --headless --frames 5
--screenshot /tmp/x.ppm` produces a valid PPM. Add a unit-style
  check in `python/tests/unit/test_run_cpp_example.py` (or sibling)
  asserting the new pixi task name resolves to `dartsim`.

This is a small, self-contained slice — land it independently, do
not bundle with R12-1/2/3.

---

## 2026-05-15 Round 19 — Inverse dependency: examples MUST own their scene, not look it up

**Trigger (user direction, with evidence):**

> The example and dart::gui is inverse dependency:
>
> ```
> dart::gui::ApplicationOptions options;
> options.defaultScene = "drag-and-drop";
> options.panels.push_back(std::move(controls));
> return dart::gui::runApplication(argc, argv, options);
> ```
>
> the example should be defined in the example folder not in
> `dart/gui/**`

This is the **biggest architectural problem** in the current state.
It supersedes the "this example is restored" claims for every
example that uses `options.defaultScene = "<name>"`.

### Verified inverse-dependency chain

```
examples/drag_and_drop/main.cpp:67
  └─→ options.defaultScene = "drag-and-drop";       ← string handoff
      └─→ dart::gui::runApplication(argc, argv, options)
          └─→ dart/gui/experimental/detail/filament/scenes.cpp:294
              if (name == "drag-and-drop") {
                scene = ExampleScene::DragAndDrop;
              }
              └─→ scenes.cpp:736
                  case ExampleScene::DragAndDrop:
                    return createDragAndDropScene();
                  └─→ scene_fixtures.cpp:3652
                      DartScene createDragAndDropScene() {
                        // ALL example behavior lives here:
                        // world setup, target frames, axis markers,
                        // selection handles, interaction targets, ...
                      }
```

The example folder owns ZERO of the example's actual content.
Renaming `examples/<name>/main.cpp` to a real source did not solve
the inversion — it only added a thin wrapper that names a fixture
defined inside the GUI library. The library is still the
authoritative source of every example's behavior, which means:

- A new user reading `examples/drag_and_drop/main.cpp` learns
  nothing about how to build their own DART simulation with target
  frames and selection handles.
- The library cannot be split from the examples — every
  `dart::gui` build pulls in 4082 LOC of fixture code that has
  nothing to do with rendering.
- Removing one example doesn't shrink the library at all.
- Adding a new example requires editing the library's `enum
ExampleScene`, the `name → enum` mapping, the `enum → factory`
  switch, and the `scene_fixtures.cpp` factory itself —
  user-hostile.

### What "real example ownership" means (the fix)

Every restored `examples/<name>/main.cpp` must:

1. **Build its own `dart::simulation::WorldPtr`** in-source, using
   `dart::dynamics`, `dart::io`, `dart::collision`, etc. No
   `options.defaultScene = "<name>"` lookup.
2. **Pass that world to `runApplication` via
   `options.world = world`** (the `ApplicationOptions::world`
   handoff added at `a23ea52a9b0`). This is the existing public
   API — it just isn't being used.
3. **Build its own `Panel` instances** for any custom UI controls,
   register them via `options.panels`. (Already happening for some
   examples.)
4. **Build its own `preStep` / `postStep` callbacks** for
   per-iteration logic (controllers, force application, etc.) via
   `options.preStep` (added at `19eae1d2d91`). (Already happening
   for some examples.)
5. **Use NO `options.defaultScene` string.** The
   `--scene <name>` CLI flag continues to work for `dartsim`
   itself (developer fixture menu), but per-example binaries must
   not consume a scene string — their world is hard-coded in
   their `main.cpp`.
6. **Delete the corresponding `createXxxScene()` factory from
   `scene_fixtures.cpp`** in the same commit, OR (if the fixture
   is ALSO needed by `dartsim --scene <name>`) move it under a
   test/dev fixture target outside `dart/gui/`.

### Audit of the 26 currently-"restored" examples

Codex needs to flag, in `examples/<name>/main.cpp` for every
restored example, whether it uses `options.defaultScene` or
`options.world`. Likely state right now:

- **Inverse-dependent (BAD)** — uses `options.defaultScene`:
  `drag_and_drop` (verified), and likely most of the
  recently-migrated rigid/joint family.
- **Self-owned (GOOD)** — uses `options.world` with in-source
  construction: `hello_world` is documented as adding
  `ApplicationOptions::world` at `a23ea52a9b0`; verify others
  follow.

The audit goes in this section under a "Per-example dependency
status" subsection that Codex updates as each example flips from
inverse to self-owned. Acceptance is the same as the original
per-example checklist plus: zero `options.defaultScene` strings in
`examples/**/*.cpp`.

### Per-example dependency status

Audit command:

```bash
rg -n "options\\.defaultScene" examples
```

Current result after the Atlas Puppet ownership repair:

- **Still inverse-dependent:** none under `examples/**/*.cpp`.
- **Current promoted API repair:** `dart::gui::ApplicationOptions::ikHandles`
  carries public DART IK target handles so puppet examples can move to
  `options.world` without losing hotkey selection, selected labels, or
  solve-on-drag behavior.
- **Next puppet repairs:** `examples/hubo_puppet` and `examples/g1_puppet`
  should move from macro launchers to real source-owned `main.cpp` files that
  load their robots, create visible IK targets, and pass `options.ikHandles`
  through promoted `dart::gui`.
- **Flipped to self-owned in the latest slice:** `examples/atlas_puppet` now
  owns the Atlas URDF load, start pose, visual-only setup, ground, root handle,
  four IK target frames, support-foot geometry, hotkeys, panel, and
  `options.ikHandles` handoff in-source. The runner no longer injects
  `--scene atlas-puppet`.
- **Recently flipped to self-owned:**
  `examples/atlas_simbicon/main.cpp` now loads the Atlas SDF, owns the
  visual-only setup and ground in-source, and the runner no longer injects
  `--scene atlas-simbicon`.
- **Recently flipped:**
  `examples/operational_space_control/main.cpp` now loads the WAM URDF, owns
  the target, ground, and operational-space pre-step controller in-source, and
  `examples/wam_ikfast/main.cpp` now loads the WAM URDF, owns the visual-only
  robot setup, target, and ground in-source. The runner no longer injects
  `--scene` for either executable.
- **Recently flipped:** `examples/imgui/main.cpp`
  now constructs a small panel-demo DART world in-source, and
  `examples/tinkertoy/main.cpp` now constructs its reference axes, target,
  force-line, and initial Tinkertoy assemblies in-source. The runner no longer
  injects `--scene tinkertoy`.
- **Earlier flipped:** `examples/drag_and_drop/main.cpp` constructs its
  `SimpleFrame` anchor, child draggable frame, and X/Y/Z marker frames
  in-source and passes `options.world` to
  `runApplication(argc, argv, options)`.
- **Already self-owned:** `hello_world`, `boxes`, `box_stacking`,
  `rigid_cubes`, `simple_frames`, `capsule_ground_contact`, `fetch`,
  `rigid_chain`, `rigid_loop`, `mixed_chain`, `coupler_constraint`,
  `add_delete_skels`, `rigid_shapes`, `hybrid_dynamics`, `biped_stand`,
  `joint_constraints`, `free_joint_cases`, `human_joint_limits`,
  `operational_space_control`, `wam_ikfast`, and `atlas_simbicon`.
- **Fixture cleanup gap:** the private `createDragAndDropScene()` remains for
  `empty` and `dartsim --scene drag-and-drop` developer coverage. Deleting or
  moving that fixture requires migrating `empty` and then deciding the
  developer fixture target shape.

### Where the fixture code goes after the inversion

`dart/gui/experimental/detail/filament/scene_fixtures.cpp` is
4082 LOC today. After every per-example fixture moves into its
example, the file should contain only the developer-only fixtures
that `dartsim --scene <name>` exposes (the MVP scene, glTF panels,
broad robot/PBR coverage). If a fixture is genuinely useful to
keep available for both an example AND the `dartsim` developer
menu, the example owns the canonical version and `scenes.cpp`'s
factory becomes a thin call into the example's source — but
keeping the example as the source of truth.

Better long-term shape: move the developer fixtures to
`tests/fixtures/gui_scenes/` (a test-only target) so the GUI
library carries zero example/fixture code. `dartsim --scene
<name>` then loads them through a fixture-test linkage; the
production `dart-gui` library stays slim.

### Acceptance for Round 19

- `git grep -nE 'options\.defaultScene\s*=' examples/` returns
  ZERO matches.
- `git grep -nE 'options\.world\s*=' examples/` returns at least
  one match per restored example.
- `examples/drag_and_drop/main.cpp` constructs the SimpleFrame
  anchor, child frame, and axis markers in-source; deletes
  `createDragAndDropScene()` from `scene_fixtures.cpp`.
- The same for every other previously-"restored" example
  (`hello_world` may already be done — confirm; otherwise migrate).
- `wc -l dart/gui/experimental/detail/filament/scene_fixtures.cpp`
  drops by at least 500 LOC per migrated example family.
- `dartsim --scene drag-and-drop` continues to work for
  developers, either by linking the example's world-building code
  into the developer fixture target OR by providing a slim
  fixture target equivalent under `tests/fixtures/`.

### Order: this round is HIGH PRIORITY (above R12-3 / R12-5)

This restructures the work already done. Sequence:

R19-1. **Audit pass** — list every `examples/<name>/main.cpp`
that uses `options.defaultScene`. Add the list to this
section.
R19-2. **Migrate `drag_and_drop` first** as the canonical
template (because it's the example with the user-reported
bug AND the user's cited example of the inversion). After
migration: `drag_and_drop/main.cpp` builds its own world
with `SimpleFrame` anchor + child + axis markers; the
`createDragAndDropScene()` factory is deleted; `dartsim
       --scene drag-and-drop` either keeps working via a slim
linkage or is removed.
R19-3. **Migrate the rest of the inverse-dependent examples**
(`imgui`, `tinkertoy`) so they own their worlds in-source and then continue
fixture cleanup in the same order Codex used for the original "restoration":
rigid family → joint/dynamics → robot/IK → geometry/visual.
Each migration is one commit per example (or per family
if examples share fixture code).
R19-4. **Apply the same rule to NEW migrations** going forward.
The 17 examples not yet migrated (per Round 12 / R12-5
table) skip the macro-shim step and skip the
`options.defaultScene` step — they go directly to the
self-owned `options.world` shape.

R19 supersedes the parts of R12-5 that produced inverse-dependent
examples. R12-3 (`apps/dartsim/`), R12-1 (mouse), R12-2 (lighting)
keep their previous priority.

### What Codex should NOT do for Round 19

- Do NOT defend the `options.defaultScene` shape as "DRY because
  the fixture is shared with dartsim". The DRY argument is
  backward: the GUI library should not depend on example fixtures.
  If sharing is needed, the example owns the canonical version
  and the developer menu pulls from it (or from a test fixture
  target).
- Do NOT bulk-rewrite all 26 restored examples in one commit.
  One example or one family per commit so any regression in the
  shared inversion-removal infrastructure is bisectable.
- Do NOT keep any per-example `createXxxScene()` factory in
  `scene_fixtures.cpp` "for safety" after the example owns its
  world. Delete the factory in the same commit; the boundary
  guard / test will catch regressions.
- Do NOT promote `options.defaultScene` to a richer typed
  enum-based API. The whole concept of "library knows about
  named example scenes" is what we're removing; making it more
  ergonomic to call would entrench the inversion.

---

## 2026-05-15 Round 20 — "Self-owned" is not the same as fully restored

**Trigger (user correction):**

> Wait, there are many more examples that are not fully restored, such as
> `examples/fetch/`

The Round 19 audit correctly separated examples that use `options.world` from
examples that still use `options.defaultScene`, but that is only an
ownership/dependency audit. It is **not** enough to call an example fully
restored. A restored example must also preserve the educational behavior and
important user-visible controls from the pre-OSG source unless the missing
piece is deliberately replaced by a documented `dart::gui` feature.

### Fetch parity gap

`examples/fetch/main.cpp` currently owns a world and uses public `dart::gui`,
but the migration is still partial compared with the historical Fetch example:

- The legacy example preferred the Bullet collision detector when available.
- The legacy interactive target was an `InteractiveFrame` with a visible
  cross/handle affordance and drag behavior; the current source reduces this
  to a small green sphere.
- The private fixture marks the loaded skeletons visual-only for developer
  render coverage, but the historical example was a live simulation. The
  restored example should prefer historical behavior and document any remaining
  fixture/example divergence instead of silently treating the fixture as
  canonical.
- The legacy panel text described the whole-body target-following behavior and
  exposed the old viewer help/menu affordances. The current panel is only a
  minimal status panel.
- The legacy camera/home-view intent is not expressed by the public example
  path yet; until `dart::gui` exposes per-example camera defaults, this remains
  a parity gap to track explicitly.

### Restoration bar for all examples

For every restored `examples/<name>/main.cpp`, track both:

- **Ownership:** no `options.defaultScene`; source constructs or loads the
  world and passes `options.world`.
- **Parity:** source preserves the example's essential simulation behavior,
  visual affordances, controls, and headless renderability under the promoted
  `dart::gui` API.

Do not mark an example "fully restored" based only on `options.world`. Use
"source-owned, parity pending" when behavior or controls are still reduced.

### Immediate order

R20-1. Update the active execution notes so `fetch` is no longer described as
fully restored; call it source-owned with parity gaps.

R20-2. Repair `examples/fetch/` first: restore the visual target affordance
using public DART shapes/frames, prefer Bullet when available, preserve the
historical live target-following simulation over the fixture-only visual setup,
and strengthen the panel copy without exposing Filament, GLFW, Dear ImGui,
OSG, or private GUI headers.

R20-3. Add or update a boundary/unit guard that checks the Fetch source keeps
the expected public-only ownership/parity markers.

R20-4. Validate `fetch` directly and through `pixi run ex fetch`, then commit
and push a checkpoint without opening a PR.

### Implementation state

- The current code slice repairs `examples/fetch/main.cpp` by preferring
  Bullet when available, preserving the live mocap target-following `preStep`,
  restoring a visible green cross target with public `SimpleFrame` and shape
  APIs, and strengthening the panel text around the whole-body target behavior.
- `UNIT_gui_FilamentSceneExtraction` now includes Fetch-specific parity
  markers so the example cannot regress back to a source-owned but reduced
  marker-only version without a test failure.
- Remaining explicit gap: the legacy OSG camera home-position API has no
  public `dart::gui` equivalent yet, so Fetch's exact initial view remains
  tracked as a future `ApplicationOptions`/camera-default API question rather
  than hidden in the example.

---

## 2026-05-15 Round 21 — `scene_fixtures.cpp` still carries 4082 LOC of dead-code duplicates

**Trigger:** Supervisor audit of working tree (orthogonal to
Codex's Round 20 parity audit; addresses a different gap).

```
$ wc -l dart/gui/experimental/detail/filament/scene_fixtures.cpp
   4082    ← unchanged since R19 began

$ grep -cE '^DartScene create[A-Z][a-zA-Z]*Scene\(' \
        dart/gui/experimental/detail/filament/scene_fixtures.cpp
   31    ← all factories still resident
```

R19 removed every `options.defaultScene = "<name>"` from the
example tree (✅ ownership inversion fixed at the example side),
and Codex's Round 20 documents the remaining parity gaps inside
self-owned examples. **But the duplicate fixtures still resident
in the GUI library itself** — the
`createXxxScene()` factories that the examples used to look up —
have not been removed.

19 of 30 migrated examples have a duplicate fixture inside
`scene_fixtures.cpp`, reachable via `scenes.cpp`'s `name → enum →
factory` dispatch as `dartsim --scene <name>`:

```
hello_world, boxes, drag_and_drop, simple_frames,
capsule_ground_contact, rigid_chain, rigid_loop, mixed_chain,
coupler_constraint, add_delete_skels, hybrid_dynamics,
joint_constraints, free_joint_cases, human_joint_limits, fetch,
atlas_simbicon, tinkertoy, wam_ikfast, operational_space_control
```

These are dead-code duplicates of the per-example sources Codex
just authored. They inflate `scene_fixtures.cpp` to 4082 LOC, force
the production `dart-gui` library to carry example fixture code,
and make every future example edit land in two places.

### How this differs from Codex's Round 20

- **Codex's R20** is about _parity_ — does the self-owned example
  preserve the original example's behavior (controls, affordances,
  collision detector, camera home, etc.)? That's a per-example
  quality bar, addressed inside `examples/<name>/main.cpp`.
- **This R21** is about _cleanup_ — the library-side factory that
  the example used to reference is now an unreachable duplicate
  (or, if `dartsim --scene <name>` still reaches it, a
  competing-source-of-truth).

Both rounds must land. R20 ensures the example is pedagogically
real; R21 ensures the library doesn't carry the same content
twice.

### Two acceptable terminal states (pick one PER FIXTURE)

**Option A — DELETE the duplicate factory + its `enum
ExampleScene` entry + its `name → factory` dispatch case.**
`dartsim --scene <name>` then no longer recognizes that name.
The per-example binary is the only way to run the fixture.
Headless smoke for the fixture runs `pixi run ex <name>
--headless --frames 5 --screenshot ...` against the per-example
binary instead of `dartsim --scene <name>`.

**Option B — MOVE the factory to a test-only target
(`tests/fixtures/gui_scenes/<name>_scene.cpp`)** and link it into
the `dartsim` developer menu through that test target. The
production `dart-gui` library carries zero fixture code.
Acceptable if the `dartsim --scene <name>` developer menu must
keep working as the canonical batch-screenshot surface.

**Default to A.** Choose B only when there's a concrete need to
keep the developer menu name (e.g. existing CI pipeline depends
on it). Pick consistently per fixture; do not mix within one
example.

### Acceptance for Round 21

- `wc -l dart/gui/experimental/detail/filament/scene_fixtures.cpp`
  drops by AT LEAST 2000 LOC.
- `grep -cE '^DartScene create' scene_fixtures.cpp` returns ≤ 12
  (only `MvpDartScene` plus the not-yet-migrated examples:
  `vehicle`, `g1_puppet`, `hubo_puppet`, `lcp_physics`,
  `mimic_pendulums`, `point_cloud`, `polyhedron_visual`,
  `simulation_event_handler`, `soft_bodies`, `heightmap`,
  `hardcoded_design`, `empty`).
- For each of the 19 names listed above, EITHER (Option A)
  `grep -nE "name == \"<scene-name>\"" scenes.cpp` returns zero
  matches AND `dartsim --scene <name>` exits with "unknown scene"
  AND the smoke pipeline calls the per-example binary;
  OR (Option B) the dispatch points at a `tests/fixtures/`-backed
  loader.
- `MvpDartScene` stays in `scene_fixtures.cpp` — that's the
  default `dartsim` (no flags) scene, not an example fixture.
- Standing rule for new R12-5 migrations: **the example's commit
  also deletes its factory in the same commit** (or moves it to
  `tests/fixtures/`). This avoids re-accumulating the cleanup
  debt R21 catches up on.

### Order

R21 is independent of R12-1/R12-2/R12-3/R15. Run it in PARALLEL
with the next mouse-fix / lighting-fix / app-build-out commits.
Land as one big cleanup commit (or 4-commit family sequence:
rigid → joint → robot → geometry) so reviewers see the LOC drop
in one place.

After R21 lands, R19 may be marked `~~done~~` with evidence:
"Factories removed in commit XYZ; `scene_fixtures.cpp` dropped
from 4082 → <new> LOC."

### What Codex should NOT do for Round 21

- Do NOT keep factories "for safety" or "in case the developer
  menu needs them later". If the menu needs them, Option B
  (move to `tests/fixtures/`) is the correct response, not
  "leave them in the library".
- Do NOT mark R19 as done while the factories still live in the
  library. Working-tree evidence trumps doc claims.
- Do NOT bundle this round with R12-5 future migrations. R21 is
  the catch-up commit for the 19 that already shipped without
  cleanup; R12-5 going forward bundles cleanup with each new
  migration.
- Do NOT delete `MvpDartScene`.

---

## 2026-05-15 Round 18 — `--gui-scale` parser robustness + runner argv-merge clarity

**Trigger:** Live user output running
`pixi run ex drag_and_drop --gui-scale`:

```
Running: build/.../bin/drag_and_drop --gui-scale --scene drag-and-drop --width 1280 --height 720
Invalid --gui-scale value '--scene'. Expected a positive number.
```

Two distinct bugs surfaced in one transcript:

### Bug A: binary treats next flag as a value when value is missing

`scenes.cpp:576-582` does this:

```cpp
} else if (arg == "--gui-scale" && i + 1 < argc) {
  const char* value = argv[++i];                 // ← consumes "--scene"
  const float guiScale = std::strtof(value, &end);
  if (end == value || *end != '\0' || !std::isfinite(guiScale)
      || guiScale <= 0.0f) {
    std::cerr << "Invalid --gui-scale value '" << value
              << "'. Expected a positive number.\n";
```

The user typed `--gui-scale` without a value, but the parser
greedily consumed `--scene` (which the runner appended) as the
value, then complained about the value being non-numeric. The
diagnostic blames the wrong thing.

**Fix:** before `++i`, peek at `argv[i+1]`. If it starts with `--`
(or `-` for short flags), report `"--gui-scale requires a positive
numeric value"` and exit 2 BEFORE consuming the next argument. Same
fix for every other value-taking flag in `scenes.cpp` (`--frames`,
`--width`, `--height`, `--screenshot`, `--orbit-light-period`,
`--scene`, etc.). Make this a shared helper that every
value-taking flag calls.

### Bug B: runner argv merge order is surprising

`scripts/run_cpp_example.py:369` returns
`[*default_args, *run_args]` — runner-injected scene defaults
(`--scene <name> --width 1280 --height 720`) come BEFORE the
user's argv. The `_run_args_with_defaults` helper currently strips
the default `--scene` pair only when the user supplies one;
extend the dedup to every defaulted pair flag (`--width`,
`--height`, `--gui-scale`, `--frames`, `--screenshot`,
`--orbit-light-period`).

### Acceptance for Round 18

- `pixi run ex drag_and_drop --gui-scale` (no value) reports
  `--gui-scale requires a positive numeric value` from the
  BINARY, exit 2. The diagnostic names the right flag.
- `pixi run ex drag_and_drop --width 1920 --height 1080` runs
  with EXACTLY one `--width 1920 --height 1080` pair on the
  command line.
- New tests cover both the binary parser (each value-taking flag
  rejects missing-value with a clear message) and the runner
  dedup logic.

Both fixes ship in separate commits. Don't bundle.

### What Codex should NOT do for Round 18

- Do NOT add per-flag short aliases.
- Do NOT change the binary's left-to-right last-wins option
  semantics.
- Do NOT silently drop a flag with missing value.

---

## 2026-05-15 Round 17 — Every restored example must accept `--gui-scale`

**Trigger:** User direction: "all the GUI examples should be able to
take `--gui-scale`."

### Verified evidence (the plumbing is mostly in place)

- `dart/gui/experimental/detail/filament/scenes.cpp:576-587` parses
  `--gui-scale <factor>` from `argv` into
  `options.run.guiScale`, clamped to `[0.5, 4.0]`.
- `dart/gui/experimental/viewer.cpp:50-53` normalizes the value
  with a NaN/inf guard.
- All restored examples (`hello_world`, `drag_and_drop`,
  `biped_stand`, `boxes`, `box_stacking`, `capsule_ground_contact`,
  `coupler_constraint`, `fetch`, `free_joint_cases`,
  `human_joint_limits`, `hybrid_dynamics`, `imgui`,
  `joint_constraints`, `mixed_chain`, `rigid_chain`, `rigid_cubes`,
  `rigid_loop`, `rigid_shapes`, `simple_frames`, `tinkertoy`)
  call `dart::gui::runApplication(argc, argv, options)` — meaning
  the CLI parser they reach already supports `--gui-scale`.

So the option **technically works today** for every restored
example. The gap is in three other places:

### Gaps to close

1. **Verify the option survives `ApplicationOptions` overrides.**
   When an example sets `options.defaultScene` /
   `options.preStep` / `options.panels` / `options.world` and
   passes them to `runApplication(argc, argv, options)`, the CLI
   parser should still parse `--gui-scale` from `argv` and apply
   it to `options.run.guiScale` — not be stomped by an example-set
   default. Audit the merge order in
   `dart/gui/experimental/detail/filament/scenes.cpp` (where
   `argv` parsing happens) versus the `ApplicationOptions` overlay
   point. If any example-set value silently wins over the parsed
   CLI value, fix the merge to make CLI authoritative for run
   options like `guiScale`, `frames`, `headless`, `screenshot`.

2. **Document `--gui-scale` consistently.** Each restored
   example's `--help` output and any per-example README should
   list `--gui-scale <factor>` (range 0.5–4.0) alongside
   `--frames`, `--headless`, `--screenshot`. The text should be
   identical across examples — surface it through a shared
   `dart::gui::printCommonOptions()` helper rather than copy-paste,
   so future option additions don't drift per example.

3. **Headless smoke for the option.** Add a focused unit /
   smoke test:
   - Run `<example> --headless --frames 5 --gui-scale 2.0
--screenshot /tmp/x_2x.ppm` and assert the PPM is roughly 2×
     the linear dimensions of `<example> --headless --frames 5
--screenshot /tmp/x_1x.ppm`.
   - Pick one example per family for the smoke (e.g.
     `hello_world` for visual-only, `drag_and_drop` for
     interaction, `atlas_puppet` once migrated for robot meshes).
     Don't add the smoke to all 30+ examples — that's CI noise.

### Acceptance for Round 17

- `pixi run ex hello_world --gui-scale 2` produces a window roughly
  2× the default size (or, headless, a PPM 2× the default linear
  dimensions).
- The same works for `drag_and_drop`, `biped_stand`, every other
  restored example, AND every example migrated under R12-5 going
  forward (this is a permanent acceptance criterion for any new
  per-example `main.cpp`).
- `pixi run ex <example> --help` lists `--gui-scale` in a uniform
  format across all examples.
- A headless smoke proves the option propagates to the actual
  framebuffer dimensions.
- `apps/dartsim/` (after R12-3) inherits `--gui-scale` for free
  via `runApplication`; no additional plumbing needed in
  `apps/dartsim/main.cpp`.

### Implementation surface (short)

- Audit `scenes.cpp` argv parsing vs `ApplicationOptions` overlay
  precedence; fix merge order if needed.
- Add `dart::gui::printCommonOptions()` helper consumed by every
  example's `--help` (or, simpler: `--help` is already centrally
  printed by `runApplication`, so just confirm it lists
  `--gui-scale`; if the example overrides `--help`, restructure
  to delegate the common section).
- Add `tests/unit/gui/test_gui_scale_propagation.cpp` (graphics-
  free) asserting the parsed `options.run.guiScale` matches the
  string passed in `argv` for the supported range and rejects
  out-of-range / non-numeric inputs.
- Add one or two pixi smoke tests under
  `pixi run test-filament-gui-smoke` that verify framebuffer
  scaling at 1.0 vs 2.0.

### What Codex should NOT do for Round 17

- Do NOT add a per-example CLI parser. Every example must
  continue to delegate to `runApplication(argc, argv, options)`.
  The option lives in the central parser; examples just inherit.
- Do NOT clamp `guiScale` differently per example. The
  `[0.5, 4.0]` range in `viewer.cpp:53` is global.
- Do NOT add `options.run.guiScale = ...;` overrides inside
  example `main.cpp` files except as a per-example default
  applied BEFORE `runApplication` parses `argv`. CLI must win
  over example defaults.

---

## 2026-05-15 Round 16 — ImGui Docking is REQUIRED for `apps/dartsim/` (overrides original Decision 5)

**Trigger:** User direction: "for dartsim, it should use ImGui
Docking." This makes the contradiction in earlier rounds explicit
and removes ambiguity:

- The original `### Decisions in force` line 116-118 lists
  `"ImGui Docking, dockable 3D widget, video capture, font atlas/label
work, ... — Out of scope for this branch (do not start, even
opportunistically)"`.
- Round 14 line 621 lists `"Docked panel layout — ImGui Docking
with at least these distinct panel files"` as a divergence
  requirement.
- Round 12 step R12-3b (line ~288) already enables ImGui Docking
  for `apps/dartsim/` specifically.

These conflict on paper. This round resolves the conflict in writing.

### Decision (overrides Decision 5)

**ImGui Docking is REQUIRED for `apps/dartsim/`.** It is no longer
"out of scope". Codex must:

- Build Filament's bundled ImGui (or vendor a Docking-enabled ImGui
  source) with `IMGUI_HAS_DOCK` defined; the configure step needs
  to verify the active ImGui supports docking. If the current
  vendored ImGui in the Filament tree does not have Docking, this
  round includes the work to switch to the Docking branch (it is
  the same upstream ImGui repo, just the `docking` branch).
- Set `ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_DockingEnable`
  during the dartsim ImGui init.
- Add a top-level dockspace via `ImGui::DockSpaceOverViewport()` so
  every panel `apps/dartsim/app/*_panel.cpp` registers can be
  docked / undocked / re-arranged at runtime.
- Persist the dock layout as part of the `.dartsim` project file
  (per Round 14 divergence point #9). On startup, restore the saved
  layout if present; otherwise apply a sensible default
  (3D viewport central, Scene + Inspector left, Timeline + Log
  bottom, Recording right).

### What stays out of scope (Decision 5 partial-survival)

The original Decision 5 list still excludes — UNLESS the user lifts
them in a later round:

- ~~ImGui Docking~~ — **lifted by this round**.
- Multi-viewport / multi-window (`ImGuiConfigFlags_ViewportsEnable`)
  — out. One window with internal docking only.
- Video capture (`.mp4` / `.webm` encoding) — out. Image-sequence
  capture (Decision 6) is the only recording in scope.
- Font atlas / label rendering work — out.
- Custom theme / internationalization — out.
- macOS / Windows port work — out.
- conda-forge feedstock changes — out.

### Implementation notes for Codex

- ImGui Docking adds new symbols (`ImGui::DockSpace*`,
  `ImGuiID DockSpaceOverViewport(...)`, `ImGuiDockNodeFlags_*`).
  If the current build does not link these, the first commit in
  R12-3b is the ImGui-source / build switch, NOT the docking
  layout itself. Land that as its own commit so any build breakage
  is bisectable.
- The boundary-guard test (`UNIT_gui_FilamentSceneExtraction`)
  currently rejects `<imgui.h>` in `examples/`. That rejection
  must continue. `apps/dartsim/app/docking_layout.cpp` is the one
  ImGui consumer outside `dart/gui/detail/filament/`; the
  boundary-guard's `dart_gui_add_application()` configure check
  needs an explicit allowance for that one file (or, preferably,
  the ImGui Docking calls move behind a `dart::gui::DockingLayout`
  wrapper added in R12-4 so `apps/` doesn't include `<imgui.h>`
  at all — that's the better long-term shape).
- The dartsim built-in status panel (the MVP debug overlay) keeps
  working; it just becomes one more dockable window in the new
  layout. Do NOT delete it as part of this round.

### Acceptance for Round 16

- `apps/dartsim/main.cpp` (or `apps/dartsim/app/docking_layout.cpp`
  after R12-3b) calls `ImGui::DockSpaceOverViewport()` and the
  panel windows are dockable in a running session.
- The user can drag a panel out, dock it to a different edge, and
  the new layout persists across restart (via `.dartsim` project
  file from Round 14 / R12-3a).
- The boundary-guard test still rejects `<imgui.h>` in
  `examples/**`, but allows it in `apps/dartsim/app/docking_layout.cpp`
  (or rejects it everywhere if the `dart::gui::DockingLayout`
  wrapper lands first).
- `pixi run dartsim` (no args, after R12-8) opens with the default
  dock layout: central 3D viewport, Scene + Inspector docked left,
  Timeline + Log docked bottom, Recording docked right.

### What Codex should NOT do for Round 16

- Do NOT enable `ImGuiConfigFlags_ViewportsEnable` (multi-window).
  That's a separate, larger change (cross-platform window
  management) that stays out of scope.
- Do NOT add docking to the dartsim built-in status panel as a
  half-measure. The status panel works fine inside a docked
  viewport once the dockspace exists; no changes needed there.
- Do NOT block this on `apps/dartsim/app/` panels existing —
  enable docking and add an empty dockspace first; panels register
  as they're built in R12-3c..3e.
- Do NOT change Decision 5 lines 116-118 in place; the override is
  recorded here for traceability. (If you do edit Decision 5, mark
  the ImGui-Docking entry as `~~ImGui Docking~~ — lifted by
Round 16` rather than deleting it.)

---

## 2026-05-15 Round 15 — Two user-reported bugs in `drag_and_drop` + ImGui-vs-camera leakage

**Trigger:** User reported, on running `pixi run ex drag_and_drop`:

1. "Mouse handling is still not ready for drag and drop" — i.e. the
   selectable targets (anchor / child frame / axis markers) cannot be
   dragged with a plain LMB drag.
2. "While interacting with ImGui widget, it still manipulates the
   camera" — clicking inside a panel (drag a slider, click a button)
   also rotates the camera underneath.

This is a regression-class bug, not a polish item. Both must land in
focused commits with regression tests. **Promote both above
R12-3/4/5/8 in the order of operations**: R12-1 (mouse bindings) and
the new R15-2 (ImGui capture) are now the next two commits.

### Verified evidence

- `examples/drag_and_drop/main.cpp:79` panel tip text still reads
  `"Ctrl-left drag moves the selected object."` — proves the OLD
  Ctrl-modifier scheme is still authoritative in code; Round 12's
  mouse-binding decision (LMB drags selectables, no Ctrl required)
  has not landed.
- `git log --all --oneline` shows zero commits matching `mouse |
binding | drag.*select | imgui.*capture` since the Round 12
  steering was written. R12-1 has not started.
- `dart/gui/experimental/detail/filament/input.cpp` has no
  references to ImGui's `WantCaptureMouse` / `WantCaptureKeyboard`.
  The input bridge unconditionally forwards mouse events to the
  camera controller, regardless of whether ImGui is consuming them
  for a panel widget.

### R15-1: Implement the LMB-drags-selectables binding (was R12-1)

This was already specified in the **Mouse input bindings** section
above. Restating the user-visible acceptance:

- `pixi run ex drag_and_drop` — plain LMB drag on the anchor /
  child frame / axis marker moves the selectable. **No Ctrl
  modifier.**
- `pixi run ex atlas_puppet` — plain LMB drag on the right-hand
  IK target moves the target.
- `pixi run ex hubo_puppet` — same on hand/foot/wrist-peg targets.
- `pixi run ex g1_puppet` — same on the four IK markers (also
  reachable by `1`-`4` keys per the existing G1 docs).

Code surface:

- `dart/gui/experimental/detail/filament/input.cpp` — change the
  LMB press path: pick-test on press, if a selectable is hit start
  a drag (no Ctrl required), otherwise fall through to camera-orbit
  start (RMB-drag in the new scheme).
- `dart/gui/experimental/detail/filament/selection.cpp` — remove
  the Ctrl gate around plane-drag dispatch; X/Y/Z keys are
  modifier-style (held during press) instead of Ctrl-prefixed.
- Update `examples/drag_and_drop/main.cpp:79` panel-tip text to
  match: `"Drag a selectable with the left mouse button."`
  (and remove `"Ctrl-left drag…"` references everywhere they
  appear in the example tree, the `dartsim` README, and any
  in-app `--help` output).
- Update `apps/dartsim/README.md` mouse-binding table once R12-3
  rebuilds the app's docs.

Regression test:

- `tests/unit/gui/test_input_bindings.cpp` (new file). Builds a
  mock `SelectionController`, feeds synthetic `MouseButtonEvent`
  / `MouseMoveEvent` sequences, asserts: (a) LMB press on a
  selectable hits + LMB drag emits `SelectableDragRequest` (NOT
  `OrbitCameraDelta`); (b) RMB drag emits `OrbitCameraDelta`;
  (c) MMB drag emits `PanCameraDelta`; (d) scroll emits
  `ZoomCameraDelta`; (e) X / Y / Z + LMB drag on selectable
  emits axis-constrained drag; (f) LMB on empty space + drag
  does NOT emit `OrbitCameraDelta`.
- Add a smoke gesture test for `--scene drag-and-drop`,
  `--scene atlas-puppet`, `--scene g1` (per Round 12 acceptance):
  simulate LMB-press on the well-known selectable position,
  drag 50 px, release; assert position changed.

### R15-2: ImGui must capture mouse + keyboard before camera

When the cursor is over an ImGui panel, ImGui owns the input.
The camera controller must NOT also process those events.

Code surface:

- `dart/gui/experimental/detail/filament/input.cpp` — at the top
  of every mouse / keyboard / scroll callback, query
  `ImGui::GetIO().WantCaptureMouse` (or
  `WantCaptureKeyboard` for key events). If true, return early
  without forwarding to the camera controller, selection
  controller, or hotkey poller. The ImGui overlay's own callback
  ordering must run BEFORE this check (ImGui needs the event to
  decide whether to capture it).
- The dartsim built-in status panel and the new
  `examples/drag_and_drop` `dart::gui::Panel` must both benefit
  — this is a global suppression in the input bridge, not a
  per-panel hook.
- One subtle case: pick-test on LMB press. The press position
  must be tested against ImGui's panel rects FIRST. If
  `WantCaptureMouse` is true on press, do not start a
  selectable-drag either — the user is interacting with the
  widget, not the 3D scene.

Regression test:

- Add to `tests/unit/gui/test_input_bindings.cpp`: simulate
  `ImGui::GetIO().WantCaptureMouse = true`, feed an LMB-down +
  drag sequence; assert ZERO `OrbitCameraDelta` /
  `SelectableDragRequest` events emitted. Repeat with
  `WantCaptureKeyboard = true` for a key event sequence.
- End-to-end: in `--scene drag-and-drop`, position the cursor
  over the `Drag Controls` panel area (computed from window
  size + panel layout), inject LMB-down + 100 px move + LMB-up;
  assert camera transform is unchanged from the pre-event state.

### Order update (overrides earlier R12-1..R12-7 ordering)

R15-1 (mouse drag-selectable) → R15-2 (ImGui capture) →
R12-2 (lighting) → R12-8 (pixi alias) → R12-3 (apps/dartsim
build-out) → R12-4 (panel/tool API gaps) → R12-5 (remaining 17
example migrations) → R12-6 (delete macro shim) → R12-7 (drop
experimental/).

R15-1 and R15-2 are SEPARATE commits — they touch the same file
but have independently provable acceptance tests, and either fix
on its own is valuable.

### What Codex should NOT do for Round 15

- Do not paper over R15-1 by adding a panel checkbox "Drag mode
  enabled" or a CLI `--enable-drag` flag — the binding change is
  the fix.
- Do not implement R15-2 by hard-coding which screen regions are
  "panel area"; use `ImGui::GetIO().WantCaptureMouse` /
  `WantCaptureKeyboard` so the suppression follows ImGui's own
  hit-testing.
- Do not skip the `examples/drag_and_drop/main.cpp:79` text
  update — the example is the regression demo for R15-1; its
  tip text is the user's first hint about how to interact.
- Do not bundle these with example migrations or app build-out.
  Land R15-1 and R15-2 first; then resume R12-3 / R12-5.

---

**Trigger:** Verified working-tree:

```
$ wc -l apps/dartsim/main.cpp
   41 apps/dartsim/main.cpp

$ tail -7 apps/dartsim/main.cpp
   #include <dart/gui/application.hpp>

   int main(int argc, char* argv[])
   {
     return dart::gui::runApplication(argc, argv);
   }

$ ls apps/dartsim/
   CMakeLists.txt  main.cpp  README.md
```

This is the same shape as the old `examples/filament_gui/main.cpp`,
just under a different directory. The "application" exists in name
only. R12-3 already names this gap; this round is the **explicit
divergence requirement** so Codex doesn't read the existing skeleton
as "good enough" and start migrating examples before the app is
real.

### Concrete divergence requirements (`apps/dartsim/` MUST differ from a thin example)

If `apps/dartsim/` ends up indistinguishable from `examples/<name>/`,
the pivot has failed. Here is what the application must own that an
example does NOT:

1. **CLI surface** — `dartsim <world-file>` accepts a positional
   path argument and dispatches by extension to `dart::io`. An
   example never opens an arbitrary user file.
2. **Project file (`.dartsim`)** — JSON or TOML descriptor naming
   the world file, default camera, sim settings, panel layout,
   recording presets. An example has hard-coded everything.
3. **Recent files / file menu** — File → Open / File → Recent
   reads from project state. An example has no file menu.
4. **Docked panel layout** — ImGui Docking with at least these
   distinct panel files under `apps/dartsim/app/`:
   `scene_panel.cpp`, `inspector_panel.cpp`, `timeline_panel.cpp`,
   `log_panel.cpp`, `recording_panel.cpp`. An example has only the
   dartsim built-in status panel.
5. **Scene tree + Inspector** — interactive selection sync between
   the 3D viewport and the panels. An example does not need a
   skeleton/joint/body inspector.
6. **Timeline** — play / pause / step / scrub / time display /
   framerate. The example's `Space` / `n` keys are sufficient for
   an example; an app needs a visible Timeline panel with a
   scrubber.
7. **Log capture** — `dart::common::Logger` output mirrored into a
   scrollable, filterable Log panel. An example doesn't capture
   logs.
8. **Recording UI** — output dir picker, frame-count input, format
   dropdown, Start/Stop. The CLI flags continue to work for
   headless. An example never offers a recording UI.
9. **Project persistence** — File → Save Project writes camera,
   panel layout, recording defaults to a `.dartsim` file. Examples
   do not persist anything.
10. **About / Help dialog** — application identity (DART version,
    Filament version, build date). Examples do not need this.

### Target structure of `apps/dartsim/`

```
apps/dartsim/
├── CMakeLists.txt
├── README.md
├── main.cpp                        ← CLI parse, project load, app run
└── app/
    ├── application.hpp / .cpp      ← top-level app object: world, panels, project, recording
    ├── cli.hpp / .cpp              ← argv → CliOptions struct
    ├── scene_loader.hpp / .cpp     ← extension → dart::io dispatch
    ├── project.hpp / .cpp          ← .dartsim file read/write
    ├── docking_layout.hpp / .cpp   ← ImGui docking node setup
    ├── scene_panel.hpp / .cpp
    ├── inspector_panel.hpp / .cpp
    ├── timeline_panel.hpp / .cpp
    ├── log_panel.hpp / .cpp
    └── recording_panel.hpp / .cpp
```

`main.cpp` size goal: small (calls `app::Application::run`), but the
`app/` tree should grow to ~1500–3000 LOC of real application
logic. **If `apps/dartsim/main.cpp` is still the only C++ file when
R12-3 claims done, R12-3 is not done.**

### Non-divergence (explicit non-goals)

- `apps/dartsim/` does NOT add new public `dart::gui::*` API
  declarations. If a panel needs functionality the public Panel
  API doesn't expose, R12-4 promotes it inside `dart::gui`; do not
  stash app-private extension points in `dart::gui::detail`.
- `apps/dartsim/` does NOT depend on `dart::gui::experimental` once
  R12-7 (drop experimental) lands. Until then, transitive use
  through compatibility shims is fine.
- `apps/dartsim/` does NOT include any `<filament/...>`,
  `<GLFW/...>`, or `<imgui.h>` directly outside
  `app/docking_layout.cpp` (the one allowed exception for ImGui
  Docking config — and even that should ideally route through a
  `dart::gui::DockingLayout` API surface added in R12-4).
- `apps/dartsim/` is NOT a place for renderer-internal experiments.
  Those stay under `dart/gui/detail/filament/`.

### Acceptance for Round 14

- `wc -l apps/dartsim/app/*.cpp | tail -1` reports ≥ 1000 LOC.
- `apps/dartsim/main.cpp` is ≤ 80 LOC and delegates to
  `apps::dartsim::Application::run` (or equivalent).
- `pixi run dartsim path/to/world.urdf` (after R12-8 pixi alias)
  opens that world.
- `pixi run dartsim` (no args) opens an empty world AND surfaces a
  visible "File → Open…" affordance.
- Each of the 10 divergence points above is observable in the
  running app (not just in code).
- The app's `--help` lists application-specific options
  (positional file, project file, panel layout) in addition to the
  scene-fixture / headless / screenshot options inherited from
  `dart::gui::runApplication`.

### What Codex should NOT do for Round 14

- Do not declare R12-3 done while `apps/dartsim/main.cpp` is the
  only C++ file in the directory. R12-3a–3e each produce at least
  one new file under `app/`.
- Do not implement these panels by reusing the dartsim built-in
  status panel and renaming it. The built-in status panel is the
  MVP debug overlay; the app panels are first-class user-facing
  surfaces with their own state and persistence.
- Do not migrate the per-example sources (R12-5) before R12-3 +
  R12-4 land — the panel/tool API shape that emerges from the app
  build-out is the API the examples will consume.

---

## 2026-05-15 Round 13 — Side-channel steering MUST be polled, not assumed inert

**Trigger:** Observed risk: when this `docs/dev_tasks/filament_gui/`
folder is updated externally (by the supervisor agent or by the
human directly) between Codex's commits, Codex may not re-read it
on the next turn — treating its in-context view as authoritative —
or may even flag the external edit as a merge conflict / error.
That breaks the paired-agent loop: live human direction reaches the
side-channel but never reaches the implementation agent.

**The contract for any implementing agent on this branch:**

This folder (`docs/dev_tasks/filament_gui/`) is a **live side-channel
shared with the supervisor and the human user**. It IS expected to
change between your turns without you authoring the change. Treat
external edits as legitimate steering input, not as conflict signals.

### Mandatory re-read protocol (every turn)

Before starting any implementation work in a new turn:

1. **Re-read `docs/dev_tasks/filament_gui/STEERING.md`** in full —
   not just the section you remember. Look specifically for:
   - New `## 2026-MM-DD Round N` sections at the top.
   - New `### Open Question Qn` blocks anywhere.
   - `**Qn ANSWER (...)**` blocks attached to existing questions.
   - Modifications to the "Decisions in force" or "Stop condition"
     blocks.
   - Any `~~done~~` strikethrough additions made by the supervisor
     (these supersede your own status claims).
2. **Re-read `docs/dev_tasks/filament_gui/RESUME.md`** if its
   `Last updated` line or `## Live Supervisor Steering` section
   moved.
3. **Re-read `docs/dev_tasks/filament_gui/10-active-execution.md`**
   if a Round in STEERING.md cites it.
4. **Diff your in-context model against the file**: if the on-disk
   STEERING.md ends at a Round number greater than the highest you
   acted on, the new Rounds are unread steering. Read them now.
5. **If a tool call against this folder reports "file modified
   externally", that is GOOD news, not an error.** Re-read the file
   and continue. Do NOT revert the external change unless the user
   explicitly asks for a revert.

### Side-channel update sources (all are valid)

The supervisor agent OR the human user OR another tooling agent
may write to this folder between your turns. None of these are
errors:

- New `## Round N` sections appended by the supervisor.
- New `**Qn ANSWER**` blocks added by the supervisor on the user's
  behalf.
- `~~done~~` strikethrough added by the supervisor when working-tree
  evidence confirms a deliverable.
- A whole-file rewrite by the human (rare, but allowed).

If you encounter a section you don't remember writing, ASSUME it
is legitimate steering. Do NOT delete it as "stale" or "duplicate"
or "already addressed" without explicit confirmation in a Round.

### What to do when you find new steering mid-turn

- If the new steering changes the current task's scope: **stop the
  current code change**, re-plan, and write a "Round N Local
  Completion Notes" sub-section saying "rescoped due to Round (N+1)
  arrival mid-turn" before starting the new work.
- If the new steering only adds future work: continue the current
  task and address the new steering in a later round.
- If the new steering contradicts a Decision in force you were
  acting on: open a new Open Question Qn referencing both the old
  Decision and the new steering, and pause that work until the
  user resolves the contradiction.

### What to do when an external edit conflicts with your draft

If you have an in-flight edit to STEERING.md and the file changed
externally:

- **Re-read the file in full.**
- **Rebase your edit on top of the new content** (your edit
  appended a new section; the external edit added or modified a
  different section — both can coexist if you re-anchor your
  insertion point).
- **Do NOT discard the external edit.**
- If your edit and the external edit truly conflict (same lines),
  open a new Open Question explaining what you wanted to add and
  what was already there.

### Symmetric: same rules for the supervisor agent

This protocol is symmetric. If the supervisor agent finds that
Codex has written `~~done~~` strikethroughs the supervisor did not
add, the supervisor should:

- Verify the working-tree evidence for the claim (run the same
  audit commands the Round 12 table uses).
- If the evidence checks out, accept the strikethrough.
- If the evidence does NOT check out, open a new Round flagging
  the unsupported completion claim with the verification commands
  that disprove it (the Round 12 table is the model).

### Doc reference for skill capture

This protocol is generic enough to belong in a reusable AI-agent
skill. The supervisor's `dart-pair-loop` skill (in
`~/.llms/skills/claude-templates/dart-pair-loop/SKILL.md`) covers
the steering-side discipline; an analogous "side-channel-aware
implementer" skill should be created or extended to cover the
implementing-side re-read discipline above. Track this as a
follow-up task; it is not a code change in this branch.

### Acceptance for Round 13

- The next Codex turn that touches code in this branch begins by
  re-reading STEERING.md in full and acknowledging the latest
  Round number in its status update.
- No Codex commit reverts a supervisor-authored Round, ANSWER
  block, or `~~done~~` strikethrough without an explicit Open
  Question raising the disagreement.
- The supervisor's `dart-pair-loop` skill (or its successor)
  documents this protocol so future paired-agent collaborations
  inherit the discipline.

---

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

Fetch slice:

- [x] Add `dart::gui::ApplicationOptions::preStep` as a renderer-neutral
      lifecycle callback so example-owned worlds can preserve behavior that
      previously lived in private `DartScene::preStep` fixtures.
- [x] `examples/fetch/main.cpp`: restore MJCF loading, initial robot/object
      setup, mocap-target synchronization, and a compact promoted `Panel`
      without including backend UI or renderer headers.
- [x] Remove the `--scene fetch` runner default once the source-defined world
      is active.
- [x] Validate with focused build, GUI boundary unit test, Python example
      runner coverage, direct headless screenshot, `pixi run lint`, commit,
      push, and dispatch CI.

Rigid/constraint family slice:

- [x] Restore `examples/rigid_chain/main.cpp`, `examples/rigid_loop/main.cpp`,
      `examples/mixed_chain/main.cpp`, `examples/coupler_constraint/main.cpp`,
      `examples/add_delete_skels/main.cpp`, and
      `examples/rigid_shapes/main.cpp` as real public-API sources rather than
      `dart_build_gui_scene_example(...)` macro shims.
- [x] Preserve the legacy educational intent by recovering removed sources
      from history and migrating OSG viewer behavior to promoted
      `dart::gui::ApplicationOptions`, `Panel`, and lifecycle hooks as needed.
- [x] Remove the corresponding `--scene` runner defaults after the binaries own
      their worlds by default.
- [x] Validate with focused build, GUI boundary unit test, Python example
      runner coverage, representative headless screenshots, full `examples`
      build, `pixi run lint`, commit, push, and dispatch CI. Evidence:
      `fdd9d248033`.

Joint/dynamics family slice:

- [x] Restore `examples/hybrid_dynamics/main.cpp`,
      `examples/biped_stand/main.cpp`, `examples/joint_constraints/main.cpp`,
      `examples/free_joint_cases/main.cpp`, and
      `examples/human_joint_limits/main.cpp` as real public-API sources rather
      than `dart_build_gui_scene_example(...)` macro shims.
- [x] Preserve the key educational behavior: hybrid scripted velocity
      commands, biped SPD standing control, joint-constraint balance control,
      free-joint reference-body integration, and human joint-limit pose setup.
- [x] Use `dart::gui::ApplicationOptions::world`, `preStep`, and `Panel`
      without including Filament, GLFW, Dear ImGui, OSG, or
      `dart/gui/experimental` headers.
- [x] Remove the corresponding `--scene` runner defaults after the binaries own
      their worlds by default.
- [x] Validate with focused build, GUI boundary unit test, Python example
      runner coverage, representative direct and Pixi headless screenshots,
      inherited `--gui-scale` parsing, full `examples` build, and
      `pixi run lint`; commit, push, and dispatch CI as the checkpoint
      following this doc update.

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
   - [~] `atlas_puppet` — `createAtlasPuppetScene()` (own promoted
     `dart::gui` source exists; private fixture cleanup remains)
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
