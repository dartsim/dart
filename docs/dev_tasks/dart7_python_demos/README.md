# DART 7 Python demo app

This project home tracks decision D31b of the
[DART 7.0 Python-first program](../dart7_python_first/README.md): the demo app
becomes Python-only, and every feature behind it is implemented in C++.

- [Decisions](decisions.md): the local decisions L1–L13 and their evidence.
- [Verification](verification.md): checks and review passes per packet.
- [Resume](RESUME.md): the next-session handoff.

## North star

DART ships one demo app, launched with `python -m examples.demos`:

- The C++ host provides the docking ImGui workspace, scene navigator, scene
  panels, inspector, tools, profiler, log console, and headless list, cycle
  and capture. GLFW provides windows and Filament renders.
- The scenes are written only in Python. They build worlds, bind keys and add
  light step hooks.
- Heavy controllers and models stay native, behind the private
  `dartpy._demos`.

## Deliverable and acceptance

The project is complete when:

1. `python -m examples.demos` runs all 37 scenes in the parity matrix below.
   Each scene matches the C++ app on its `--debug-print-world` counts and an
   assessed capture pair. Scenes driven by keys or panel controls also pass
   scripted interaction checks (see [Gates](#gates)).
2. The host rows H1–H12 are reachable from the Python app, each proven by a
   test or a capture.
3. `pixi run demos`, `pixi run cycle-demos` and `pixi run capture` launch the
   Python app.
4. `examples/demos/` is deleted, together with the `dart-demos` executable
   and its `soft_bodies`, `soft_cubes` and `soft_open_chain` aliases.
5. Bundled ImGui is the docking release in every Pixi build and every wheel.
   A vanilla system ImGui builds DART with the host compiled out.
6. The device-free catalog tests, the GUI tests, the wheel checks and CI
   pass on Linux, macOS and Windows.
7. Every packet has two clean review passes recorded on its final state.
8. Durable facts are promoted, and this folder is deleted in the last packet.

## Scope and non-goals

In scope:

- moving the C++ host and the GUI-free models into private targets;
- the 7.0-style `dartpy.viewer` and private `dartpy._demos` bindings;
- the core bindings the scenes need;
- Python ports of every scene;
- the ImGui docking build;
- task, CI, checker and docs migration.

Non-goals:

- **No Python ImGui API**, and no Python per-frame render callbacks.
- **No new public surface:** no installed headers, package components or
  gz-facing API.
- **M6b's other work is out of scope:** `dart.render` sensors, and porting
  the interim `dartpy.gui` primitives (InteractiveFrame, drag and drop,
  GridVisual, SupportPolygonVisual, DebugOverlay, GUIEventHandler) that the
  scenes still use.
- **Pre-existing host gaps stay out of scope:**
  - LogCapture misses spdlog and Python output, and is not thread-safe.
  - Copy does not reach the system clipboard.
  - `GuiScaleTheme` duplicates the Viewer's scaling.

## Architecture

| Piece | Location and target | Namespace or module |
| --- | --- | --- |
| App host | `dart/gui/app/` → `dart-gui-app` (static, `EXCLUDE_FROM_ALL`, never installed) | `dart::gui::app` |
| Docking gate | `dart/gui/CMakeLists.txt` adds `app/` only when the copied `dart/external/imgui/imgui.h` defines `IMGUI_HAS_DOCK`. Otherwise it prints a "compiled out" status line. | — |
| GUI-free models | `python/native/demo_models/` → `dart-demo-models` (static, `EXCLUDE_FROM_ALL`). It is added right after `dart` in the top-level CMake file. IO-dependent sources are added only when `dart-utils` and `dart-utils-urdf` exist. | `dart_demos` |
| Host bindings | `python/dartpy/viewer/`, compiled only when `dart-gui-app` exists | `dartpy.viewer` |
| Model bindings | `python/dartpy/demos/` | `dartpy._demos` (private) |
| Scenes | `python/examples/demos/scenes/*.py`, `scenes/common.py`, `registry.py`, `__main__.py` | — |

Both static targets link into the shared dartpy module. GCC and Clang builds
already compile every target with `-fPIC` (the top-level `add_compile_options`),
and MSVC needs no position-independent code, so neither target sets the
property itself.

The Python surface:

```python
run(scenes, argv) -> int      # main thread; releases the GIL; re-raises a stashed BaseException after teardown
AssetsNotPrefetchedError(RuntimeError)
Scene(id, *, title, category, summary="", factory)
SceneSetup(world, *, pre_step=None, post_step=None, pre_refresh=None, on_activate=None,
           key_actions=(), panel=None, camera_home=None, shadows=True, allow_simulation=True)
KeyAction(key, label, callback)
CameraHome(*, eye, center, up=(0, 0, 1))
SceneContext: viewer, world_node; add_teardown, add_attachment, add_event_handler,
              enable_drag_and_drop, log
Panel() -> Control            # Control.Kind.GROUP ... SEPARATOR; add_* builders on groups
```

How callbacks behave:

- **Ownership.** Each callable belongs to the Python wrapper that accepted
  it. That wrapper's `tp_traverse` and `tp_clear` keep it visible to the
  garbage collector. The host holds only weak thunks.
- **Active scene.** During `run()`, a binding-side session keeps the active
  scene's `SceneSetup` wrapper alive, including its controls and key actions,
  until that scene's teardown has run. A factory's returned setup would
  otherwise lose its last reference, and its weak thunks would expire.
- **Ordinary exceptions.** They reach the host's guards as C++ errors. The
  host logs them, disables only the failing callback, and counts the
  failure. `--cycle-scenes` exits nonzero if any count is above zero.
- **KeyboardInterrupt and SystemExit.** They are stashed, stop the loop, and
  are re-raised after an orderly teardown.
- **Ctrl+C.** `stopRequested` polls `PyErr_CheckSignals` once per frame and
  once per step.

## Packets

Each packet is one branch and one PR. The C++ app keeps working until
WP-PD.18.

| ID | Title | Depends on | Lane | Status |
| --- | --- | --- | --- | --- |
| WP-PD.0 | Record the demo-host decision and project home | — | Claude | In progress |
| WP-PD.1 | Bundle the Dear ImGui docking release | 0 | Claude | Planned |
| WP-PD.2 | Move the GUI-free demo models into `dart-demo-models` | 0 | Codex | Planned |
| WP-PD.3 | Move the demo host into private `dart/gui/app` | 1, 2 | Codex | Planned |
| WP-PD.4 | Add declarative panels and the Python-ready scene contract | 3 | Claude | Planned |
| WP-PD.5 | Bind the host as `dartpy.viewer` | 4 | Claude | Planned |
| WP-PD.6 | Bind the models as `dartpy._demos` | 2 | Codex | Planned |
| WP-PD.7 | Run the Python catalog on the native host | 5 | Claude | Planned |
| WP-PD.8 | Bind resting state, `HeightmapShapef` and `VoxelGridShape` | 0 | Codex | Planned |
| WP-PD.9 | Bind end effectors, supports, whole-body IK and balance constraints | 0 | Codex | Planned |
| WP-PD.10 | Port scene batch S1 | 7 | Codex | Planned |
| WP-PD.11 | Port scene batch S2 | 7 | Codex | Planned |
| WP-PD.12 | Port scene batch S3 | 6, 7, 8 | Codex | Planned |
| WP-PD.13 | Port scene batch S4 | 6, 7, 8 | Codex | Planned |
| WP-PD.14 | Port scene batch S5 | 6, 7 | Codex | Planned |
| WP-PD.15 | Port scene batch S6a | 6, 7, 9 | Codex | Planned |
| WP-PD.16 | Port scene batch S6b | 6, 7, 9 | Codex | Planned |
| WP-PD.17 | Launch the Python catalog from the demo tasks | 10–16 | Codex | Planned |
| WP-PD.18 | Remove the C++ demo application | 17 | Codex | Planned |

Packet scope:

- **WP-PD.0.** D31b and the program README edits; the `dart7_python_api.md`
  Rendering bullet; the PLAN-622 policy line; this project home.
- **WP-PD.1.** Remove `DART_IMGUI_DOCKING`, printing a status line and
  unsetting any cached value.
  - Fetch tag `v1.92.8-docking`. Build the ImGui sources with `-w` or `/w`
    and `SKIP_UNITY_BUILD_INCLUSION`.
  - Add a Viewer `#error` if a bundled build lacks docking.
  - Pixi `config`, `config-py`, `build-py` (both OSes) and `config-tracy`
    pass `-DDART_USE_SYSTEM_IMGUI=OFF`.
  - Delete `config-demos`; repoint `demos`, `cycle-demos` and `capture` to
    `config`.
  - Change the default of `wheel-build-core`'s argument and of
    `scripts/wheel_build.py` to `OFF`.
  - Update the docs.
- **WP-PD.2.** `git mv` `ContactArrowLayout`, `atlas_simbicon/*`,
  `SoftFootSimbiconModel`, `AdaptiveSoftContactModel` and `SoftWormModel`.
  - `test_ConstraintSolver` links `dart-demo-models` instead of compiling
    these sources by path. CTest names stay the same.
  - Edit the includes in the four model tests.
  - Make `examples/demos` an in-source-only build that links the library.
  - Exclude `*/python/native/*` from coverage.
- **WP-PD.3.** Move the host files (`DemoHost` becomes `App`) and remove the
  `GuiBackend` aliases.
  - Delete the fixed-layout fallback and its `IMGUI_HAS_DOCK` guards, with an
    `#error` backstop. Split the CLI into `Cli.cpp`.
  - Add shims so the C++ scenes compile unchanged.
  - Add `test_GuiApp` and `test_GuiAppMemoryDiagnostics`.
  - Checker: a `test_GuiApp` approval scoped to `if:TARGET dart-gui-app`
    with cache `DART_USE_SYSTEM_IMGUI=ON`, plus fixtures.
  - Update the CI Filament target lists. Add an install guard against
    `dart/gui/app` and a Doxygen exclude.
- **WP-PD.4.** The `Control` panel model.
  - `SceneSetup` gains `panel` and `allowSimulation`.
  - `SceneContext` gains a validity token.
  - Composite hooks isolate the scene's part, and the host counts failed
    callbacks.
  - New host hooks and modes: `stopRequested`, a display-free `pre_refresh`,
    strict CLI integers, a strict cycle mode, and `--debug-print-world`.
- **WP-PD.5.** `python/dartpy/viewer/`, with the callback ownership and the
  GIL and exception model.
  - Add a dartpy POST_BUILD check that memory diagnostics are compiled out.
  - Add the device-free tests and a GUI-only `test_viewer_render.py`.
  - Add a `test_wheel.py` smoke test and the `python-bindings.md` notes.
- **WP-PD.6.** Bind simbicon, soft worm, adaptive contact (with point
  markers) and soft foot.
  - Validation fixes: an unknown state-machine name raises, the index
    off-by-one is fixed, DOFs are checked before use, and the binding keeps
    the World that owns the solver alive.
- **WP-PD.7.** `__main__.py` calls `run()`, or exits 1 with a rebuild hint
  when `dartpy.viewer` is absent.
  - Delete `runner.py`, `hello_world_gui` and `contacts_pointcloud`.
  - Port the existing modules to `SceneSetup` mechanically.
  - Rewrite `test_filament_examples.py`.
  - `drag_and_drop` closes its row here.
- **WP-PD.8.** `Skeleton.isResting`, `setResting` and `getIslandIndex`;
  `HeightmapShapef`; `VoxelGridShape(resolution)` under `HAVE_OCTOMAP`.
  Names stay camelCase, so M5's rename table covers them.
- **WP-PD.9.**
  - `createEndEffector`, `EndEffector`, `Support` and `getEndEffector`.
  - `Skeleton.getIK(createIfNull)`, fixing the unregistered return type.
  - `HierarchicalIK`, `WholeBodyIK`, and `BalanceConstraint` with its enums.
  - Affiliated-Skeleton retention, and GC traversal into the whole-body and
    end-effector IKs.
- **WP-PD.10 to WP-PD.16.** The scene batches below. Each batch moves the
  native helpers its scenes need into `dart-demo-models`, binds them in
  `dartpy._demos`, and adds C++ or Python tests for them:
  - S3 (WP-PD.12): `set_boxed_lcp_solver`.
  - S4 (WP-PD.13): the SPD and operational-space `Controllers`, and
    `PointCloudSensor`.
  - S5 (WP-PD.14): `WorldHistory`.
  - S6a (WP-PD.15): `WamIkFast` and `set_wam_ikfast`. S6a also deletes the
    `SharedLibraryWamIkFast` test target and keeps `GeneratedWamIkFast`.
  - S6b (WP-PD.16): `RelaxedPosture`. S6b also migrates the standalone ssik
    example, its test and its docs.
- **WP-PD.17.** `demos`, `cycle-demos` and `capture` run
  `python -m examples.demos`, with Windows overrides.
  - Delete `py-demos` and update its doc references.
  - Update the README, the gallery and examples pages, the Korean catalogs,
    the robot-models README and the deformable docs.
- **WP-PD.18.** Delete `examples/demos`, `data/humanJointLimits`, the
  transitional fields and `--debug-print-world`.
  - Update the checker clauses and fixtures, `sync_ai_commands.py`, the CI
    lists and the `test-gui` targets.
  - Promote durable facts and delete this folder.

## Parity matrix

Each row closes with evidence in [verification.md](verification.md), and
with two clean reviews of the packet that closes it.

### Host rows

| Row | Feature | C++ source (at `9e4bb4e6f`) | Proven by | Status |
| --- | --- | --- | --- | --- |
| H1 | Toolbar: play, step, rebuild, RTF, gravity, dt, collision detector, threads | `DemoHost.cpp:1534-1682` | `test_GuiApp`, capture | Open |
| H2 | View menu: fit, camera reset, shadows, grid, headlights, DPI and GUI scale, layout reset | `DemoHost.cpp:1685-1772` | capture | Open |
| H3 | Navigator: search, categories, tooltips | `DemoHost.cpp:1775-1834` | capture | Open |
| H4 | Scene tab: panel controls, key-action buttons, error isolation | `DemoHost.cpp:1869-1901` | `test_viewer.py`, capture | Open |
| H5 | Inspector: tree, details, paused edits, wireframe, hide, highlight | `Inspector.cpp:83-361` | `--debug-select-body` capture | Open |
| H6 | Tools: contact visualizer, drag force, profiler | `ContactVisualizer.cpp`, `DragForce.cpp`, `Profiler.cpp` | `--debug-record-profile` capture | Open |
| H7 | Memory tab and `DART_DEMOS_MEMORY_DIAGNOSTICS` | `memory_diagnostics*.cpp` | `test_GuiAppMemoryDiagnostics`, capture | Open |
| H8 | Diagnostics and log console | `DemoHost.cpp:1986-2099` | capture | Open |
| H9 | Input: case-folded keys, Ctrl-drag spring | `DemoHost.cpp:239-265,1349-1375`, `DragForce.cpp:59-108` | `test_viewer.py` | Open |
| H10 | CLI: list, scene, cycle, headless shot, size, scale, detector, threads, debug flags, environment overrides | `main.cpp:97-304` | `test_GuiApp`, `test_filament_examples.py` | Open |
| H11 | Scene switching, factory failure, asset skip | `DemoHost.cpp:748-818,865-983` | `test_GuiApp`, `test_viewer.py` | Open |
| H12 | Teardown, lifetime, Python GC, interrupts, finalization | `DemoHost.cpp:519-526,821-862` | `test_viewer.py` | Open |

### Scene rows

The C++ sources are in `examples/demos/scenes/`, and every Python module is
in `python/examples/demos/scenes/`. Column key:

- `exact`: the print-world counts must match.
- `counts`: the scene is seeded randomly, so only the construction counts
  must match.
- `n/a`: the scene loads no world; it is gated on the model assets.

| Scene id | C++ source | Python module | Batch | Native or bindings needed | Oracle | Status |
| --- | --- | --- | --- | --- | --- | --- |
| empty | `EmptyScene.cpp` | `empty.py` | S1 | core | exact | Open |
| hello_world | `HelloWorldScene.cpp` | `hello_world.py` | S1 | core | counts | Open |
| simple_frames | `SimpleFramesScene.cpp` | `simple_frames.py` | S1 | core | exact | Open |
| boxes | `BoxesScene.cpp` | `boxes.py` | S1 | core | counts | Open |
| rigid_cubes | `RigidCubesScene.cpp` | `rigid_cubes.py` | S1 | core; rewrite to C++ keys and Z-up | exact | Open |
| rigid_chain | `RigidChainScene.cpp` | `rigid_chain.py` | S1 | core; rewrite to the C++ damping law | counts | Open |
| add_delete_skels | `AddDeleteSkelsScene.cpp` | `add_delete_skels.py` | S1 | core | counts | Open |
| simulation_event_handler | `SimulationEventHandlerScene.cpp` | `simulation_event_handler.py` | S1 | core | exact | Open |
| rigid_shapes | `RigidShapesScene.cpp` | `rigid_shapes.py` | S1 | core | counts | Open |
| hardcoded_design | `HardcodedDesignScene.cpp` | `hardcoded_design.py` | S2 | core | exact | Open |
| rigid_loop | `RigidLoopScene.cpp` | `rigid_loop.py` | S2 | core; rewrite to the C++ damping law | counts | Open |
| dynamic_joint_constraints | `DynamicJointConstraintsScene.cpp` | `dynamic_joint_constraints.py` | S2 | core | exact | Open |
| tinkertoy | `tinkertoy/` | `tinkertoy.py` | S2 | core (`GUIEventHandler` + `computeIntersections`) | exact | Open |
| hybrid_dynamics | `HybridDynamicsScene.cpp` | `hybrid_dynamics.py` | S2 | core | exact | Open |
| mixed_chain | `MixedChainScene.cpp` | `mixed_chain.py` | S2 | core | counts | Open |
| vehicle | `VehicleScene.cpp` | `vehicle.py` | S2 | core | exact | Open |
| atlas_v5 | `ModernHumanoidsScene.cpp` | `modern_humanoids.py` | S2 | core; panel replaces keys | n/a | Open |
| unitree_g1 | `ModernHumanoidsScene.cpp` | `modern_humanoids.py` | S2 | core; panel replaces keys | n/a | Open |
| drag_and_drop | `DragAndDropScene.cpp` | `drag_and_drop.py` | WP-PD.7 | core | exact | Open |
| sleeping | `SleepingScene.cpp` | `sleeping.py` | S3 | WP-PD.8 resting bindings | exact | Open |
| box_stacking | `BoxStackingScene.cpp` | `box_stacking.py` | S3 | `_demos.set_boxed_lcp_solver` | counts | Open |
| heightmap | `HeightmapScene.cpp` | `heightmap.py` | S3 | WP-PD.8 `HeightmapShapef` | counts | Open |
| biped_stand | `BipedStandScene.cpp` | `biped_stand.py` | S4 | `_demos.BipedStandController` | exact | Open |
| joint_constraints | `JointConstraintsScene.cpp` | `joint_constraints.py` | S4 | `_demos.JointConstraintsController` | exact | Open |
| operational_space_control | `OperationalSpaceControlScene.cpp` | `operational_space_control.py` | S4 | `_demos.OperationalSpaceController` | exact | Open |
| point_cloud | `PointCloudScene.cpp` | `point_cloud.py` | S4 | `_demos.PointCloudSensor`, WP-PD.8 `VoxelGridShape` | counts | Open |
| atlas_simbicon | `atlas_simbicon/` | `atlas_simbicon.py` | S5 | `_demos.simbicon.Controller` | exact | Open |
| soft_foot_simbicon | `soft_foot_simbicon/` | `soft_foot_simbicon.py` | S5 | `_demos.soft_foot_simbicon` | exact | Open |
| adaptive_soft_contact | `AdaptiveSoftContactScene.cpp` | `adaptive_soft_contact.py` | S5 | `_demos.adaptive_soft_contact` | exact | Open |
| soft_worm | `SoftWormScene.cpp` | `soft_worm.py` | S5 | `_demos.soft_worm` | exact | Open |
| soft_bodies | `SoftBodiesScene.cpp` | `soft_bodies.py` | S5 | `_demos.WorldHistory` | exact | Open |
| soft_cubes | `SoftBodiesScene.cpp` | `soft_bodies.py` | S5 | `_demos.WorldHistory` | exact | Open |
| soft_open_chain | `SoftBodiesScene.cpp` | `soft_bodies.py` | S5 | `_demos.WorldHistory` | exact | Open |
| wam_ikfast | `wam_ikfast/` | `wam_ikfast.py` | S6a | WP-PD.9; `_demos.set_wam_ikfast` | exact | Open |
| contact_inverse_dynamics | `ContactInverseDynamicsScene.cpp` | `contact_inverse_dynamics.py` | S6a | WP-PD.9 `getIK(True)` | exact | Open |
| atlas_puppet | `AtlasPuppetScene.cpp` | `atlas_puppet.py` | S6b | WP-PD.9; `_demos.RelaxedPosture` | exact | Open |
| ssik_ik_gui | `ssik_ik_gui/` | `ssik_ik_gui.py` | S6b | core `setPythonAnalytical` plus `ssik` | exact | Open |
| human_joint_limits | `human_joint_limits/` | — | — | dropped (L5) | — | Dropped |

Scene-specific notes:

- **Frozen per-frame logic.** Five scenes run per-frame logic inside
  `renderPanel`, so they freeze when another tab is open: atlas_puppet,
  contact_inverse_dynamics, tinkertoy, ssik_ik_gui and wam_ikfast. Their
  ports move that logic to `pre_refresh`.
- **Z-up.** The Y-up skel scenes use `common.reorient_z_up`, a port of
  `ZUp.hpp`. vehicle, atlas_simbicon and soft_foot_simbicon stay Y-up, as in
  C++.
- **Registration gates.** `point_cloud` registers only when `VoxelGridShape`
  exists. The humanoid scenes register only when `ModelResourceRetriever`
  exists. On an empty asset cache they raise `AssetsNotPrefetchedError`, and
  `ssik_ik_gui` raises it when `ssik` is missing; the cycle reports both as
  skipped.
- **agent-capture.** `modern_humanoids.atlas_world` and `g1_world` keep
  returning Worlds, so agent-capture's `--factory` contract still holds.
- **ssik_ik_gui is a full port**, not just a move. It brings the 19-arm combo
  box, position and RPY drags, Home and Read-dragged, the solver options, max
  solutions, the seed metric, Solve and Seed, the solution browser with
  diagnostics, and up to 16 ghost solution skeletons.

## Gates

Every packet:

```bash
pixi run lint
pixi run check-lint
pixi run python scripts/check_local_paths.py --commit-range origin/main..HEAD
```

By change type:

- **C++:** `pixi run build` and `pixi run test`.
- **Bindings and scenes:** `pixi run build-py-dev` and `pixi run test-py`.
- **GUI:** `pixi run test-gui`.
- **Vanilla lane** (a local gate, because the CI unity lane only compiles):
  configure with `-DDART_USE_SYSTEM_IMGUI=ON`, then build `tests` and
  `dartpy`.
  - From WP-PD.3 on, configure prints the "compiled out" line.
  - From WP-PD.5 on, `import dartpy` works without `dartpy.viewer`.
  - From WP-PD.7 on, `python -m examples.demos` exits 1 with the rebuild hint.
- **GUI-off lane:** configure with
  `-DDART_BUILD_GUI=OFF -DDART_BUILD_DARTPY=OFF`, build
  `test_ConstraintSolver`, and run the four model gates.
- **Install and wheel:**
  - `pixi run python scripts/test_gui_install.py build/default/cpp/Release`,
    after removing any `gui-install*` staging directories in that tree;
  - `python3 scripts/wheel_linux.py cp312`, then
    `python3 scripts/check_headless_wheel.py dist/dartpy-*-cp312-abi3-manylinux_2_28_*.whl`;
  - record the wheel size;
  - snapshot the API on the base and on the head with
    `PYTHONPATH=build/default/cpp/Release/python/dartpy pixi run python scripts/nanobind/api_surface.py --snapshot`,
    and diff the two outputs.
- **Docs:** `pixi run docs-build` and `pixi run docs-build-ko` when Read the
  Docs pages change.
- **AI infrastructure:** `pixi run test-ai-infra` when the checker changes.

Per scene, the parity evidence is:

1. Run the C++ app:
   `dart-demos --scene <id> --headless --steps 200 --width 960 --height 600 --shot <png> --debug-print-world`.
2. Run the Python app with the same flags, from `python/`:
   `PYTHONPATH=../build/default/cpp/Release/python/dartpy python -m examples.demos …`.
   From WP-PD.17 on, `pixi run demos -- …` runs it from the repository root.
3. Compare the print-world lines (exact or counts, per the matrix).
4. Compose the two captures with `pixi run image-sheet` and review them
   semantically through `dart-verify-sim`.
5. For a scene whose behavior is driven by key actions or panel controls,
   such as atlas_puppet, wam_ikfast, ssik_ik_gui, tinkertoy or
   operational_space_control, the scene's pytest also builds the scene
   headlessly. It triggers each key action and control callback, steps, and
   asserts the effect: a moved target, a toggled support, a new solution, a
   changed force. Where the effect is visible, a capture after that
   interaction joins the pair.

Images never enter the repository.

Review: before each push, two clean local `codex review --base origin/main`
passes on the final state, recorded in `verification.md`.

## CHANGELOG drafts

These land in M6b's consolidation PR, following the program's hot-file rule.

- **Examples:** Replace the C++ `dart-demos` application with the Python demo
  catalog (`python -m examples.demos`). It runs on the same native workspace:
  - navigator and scene panels;
  - inspector, contact and drag tools, and profiler;
  - log console;
  - headless list, cycle and capture.

  `pixi run demos`, `cycle-demos` and `capture` launch it, and `py-demos` is
  removed. The `dart-demos`, `soft_bodies`, `soft_cubes` and
  `soft_open_chain` executables are removed, as is the TinyDNN-only
  `human_joint_limits` scene. Use
  `pixi run demos -- --scene soft_bodies`, with `--collision-detector` and
  `--threads` as before.
- **Python:** Add `dartpy.viewer`, a managed viewer that runs Python-defined
  scenes in the native workspace. It offers declarative panel controls and
  headless capture, and uses DART 7 names.
- **Breaking Changes:** Bundled Dear ImGui is now the `v1.92.8-docking`
  release, so the installed `dart/external/imgui` headers use the docking
  layout; rebuild applications that include them. `DART_IMGUI_DOCKING` is
  removed. With a system ImGui that lacks docking, dart-gui still builds, but
  dartpy omits `dartpy.viewer`.

## Risks

1. **The release path depends on D31b** (`dart7_python_first/README.md`,
   "Release train and checkpoints"). If parity misses `F`, WP-PD.18 moves to
   7.1.
2. **Bundled docking ImGui compiles in CI for the first time** on MSVC unity
   and AppleClang. `test_Issue2516` and `test_Issue2668` also start running.
3. **Packagers with a vanilla ImGui** ship dartpy without `viewer`.
   Document `FETCHCONTENT_SOURCE_DIR_DART_IMGUI` for offline builds.
4. **Releasing the GIL** lets other Python threads touch the World until
   `WorldBusyError` exists.
5. **Wheel growth.** The host, the models and one 3 MB generated IKFast
   translation unit go into the wheel. Measure it in WP-PD.15.

## Current milestone

WP-PD.0 (this folder and D31b) is in progress.

Next action: land WP-PD.0. Then run WP-PD.1 in the main checkout while
WP-PD.2 runs in parallel in a Codex worktree.

Blockers: none. Every push and every PR waits for explicit maintainer
approval.
