# dart-demos (Python)

A scene-registry runner that hosts DART's Python demo scenes in the shared
Filament viewer. This is the Python-first World demo surface from PLAN-103; C++
`dart-demos` (PLAN-102) is the smaller World-only C++ companion.

## Run

```bash
pixi run py-demos                                # default: open replay timeline
pixi run py-demos -- --scene replay_scrubber     # open the replay timeline
pixi run py-demos -- --scene articulated         # select a scene by id
pixi run py-demos -- --cycle-scenes --frames 4   # cycle through every scene
pixi run py-demos -- --list                      # print the scene catalog
```

Or directly (without pixi), from the repo root:

```bash
PYTHONPATH=build/default/cpp/Release/python:python \
    .pixi/envs/default/bin/python -m examples.demos --scene articulated --frames 5
```

## CLI

The runner mirrors C++ `dart-demos` to keep the cross-language UX consistent:

| Flag                | Meaning                                                    |
| ------------------- | ---------------------------------------------------------- |
| `--scene <id>`      | Select the initial scene (default: replay timeline)        |
| `--cycle-scenes`    | Advance through every scene for `--frames` frames and exit |
| `--frames N`        | Per-scene step budget (default 60 single, 4 cycle)         |
| `--screenshot PATH` | Write the final rendered frame as a binary PPM image       |
| `--out DIR`         | Write a numbered binary PPM frame sequence                 |
| `--headless`        | Render without opening an interactive window               |
| `--show-ui`         | Include ImGui panels in headless screenshots               |
| `--list`            | Print the catalog and exit                                 |

`--screenshot` and `--out` use the real Filament render path. For visual
debugging, avoid `--backend noop`: it can exercise CPU code but produces blank
pixels and is not evidence for layout, camera, lighting, or material quality.

## Visual debugging

Capture one frame:

```bash
pixi run py-demo-capture -- --scene articulated --frames 2 \
    --width 640 --height 360
```

The helper writes a PPM, converts it to PNG, rejects blank captures, and prints
the artifact paths. With `--show-ui`, it also rejects screenshots that do not
show the docked workspace and drops early warm-up frames before ImGui is visible
from the converted frame sequence. Capture the docked ImGui workspace:

```bash
pixi run py-demo-capture -- --scene articulated --show-ui --frames 2 \
    --width 1280 --height 720
```

The docked workspace has a top `Simulation` toolbar, a searchable `Demos`
navigator, scene-specific panels on the right, bottom scene panels when a demo
owns timeline controls, and a docked DART diagnostics panel. Use `Rebuild` to
reload the active scene, `Restart` to reload and run it from the beginning, and
`Layout` to restore the default docks after rearranging panels. When frame
capture is active, the `Simulation` panel also exposes a timeline scrubber over
the recorded PPM sequence with first/previous/play/next/last controls and the
selected frame path.

World-backed scenes also get a bottom `Replay` panel. `Save replay` is enabled
by default and records bounded experimental-World state snapshots while the
scene runs. Use the replay transport or scrubber to pause the live simulation,
restore a saved frame, and play the saved states without stepping physics again;
`Resume live` continues simulation from the selected restored state.

Capture a short frame sequence and request MP4 encoding when `ffmpeg` is
available:

```bash
pixi run py-demo-capture -- --scene articulated --frames 24 \
    --width 640 --height 360 --video
```

The lower-level viewer still accepts `--screenshot` and `--out` directly when
you need raw PPM output. Inspect the generated PNG or MP4 before calling a
visual change done.

## AVBD Rigid Constraints (sx)

The dedicated **`AVBD Rigid Constraints (sx)`** category groups the first
user-visible Augmented VBD rigid-constraint scenes from PLAN-104:

| Scene id                         | Shows                                                | AVBD capability exercised                          |
| -------------------------------- | ---------------------------------------------------- | -------------------------------------------------- |
| `avbd_rigid_fixed_joint_contact` | A fixed rigid payload sliding against static contact | Public fixed-joint rows plus ordinary contact      |
| `avbd_rigid_revolute_motor`      | A bounded revolute motor driving a free hinge        | Public velocity actuator mapped to AVBD motor rows |
| `avbd_rigid_breakable_joint`     | A weak fixed joint releasing a rigid payload         | Public break-force threshold and broken-state path |

This is an early AVBD rigid-row showcase, not a paper-complete reproduction.
The remaining AVBD corpus still needs the full 2D/3D reference demos, paper
figures, video/headline scenes, CPU/GPU benchmark packets, and performance
comparisons recorded in PLAN-104.

## Planned World Ports

The **`Planned World Ports`** category keeps important DART 6 demo concepts
visible without keeping the removed DART 6 implementations in the catalog. These
entries are lightweight launchable placeholders with status panels. They track
World-native follow-ups for inverse kinematics, SIMBICON walking,
operational-space control, robot puppets, collision sandbox workflows, and
mobile manipulation.

## IPC Deformable scenes

The dedicated **`IPC Deformable`** category groups the deformable-body
scenes driven by the `dartpy` IPC solver
(point-mass/spring with IPC-style clamped-log barriers, sparse projected Newton,
lagged smoothed-Coulomb friction, and conservative CCD). They live in their own
category so the IPC showcases stay together:

| Scene id                            | Shows                                                 | IPC capability exercised                   |
| ----------------------------------- | ----------------------------------------------------- | ------------------------------------------ |
| `ipc_deformable_net`                | A pinned spring net sagging/swaying under gravity     | Projected-Newton elastodynamics            |
| `ipc_deformable_drape`              | A mat draping over a step onto a ground barrier       | Ground + self-contact clamped-log barrier  |
| `ipc_deformable_trampoline`         | A corner-pinned membrane sagging and rebounding       | Taut-membrane projected-Newton dynamics    |
| `ipc_deformable_friction_slide`     | A launched mat skidding to rest on a frictional floor | Lagged smoothed-Coulomb ground friction    |
| `ipc_deformable_fem_bar`            | A tetrahedral cantilever sagging under gravity        | Stable neo-Hookean **FEM** elasticity      |
| `ipc_deformable_fem_twist`          | A tetrahedral bar twisted at both ends, then released | **FEM** volumetric shear (toward Fig 4/14) |
| `ipc_deformable_fem_drop`           | A FEM cube dropped onto a ground barrier, settling    | **FEM** volumetric body + IPC contact      |
| `ipc_deformable_fem_sphere`         | A FEM slab draping over a sphere obstacle             | **FEM** + sphere obstacle barrier (Newton) |
| `ipc_deformable_fem_box`            | A FEM slab draping over a box "table" obstacle        | **FEM** + box obstacle barrier (Newton)    |
| `ipc_deformable_fem_msh`            | A FEM cantilever loaded from a GMSH `.msh` tet mesh   | **FEM** body from an imported tet mesh     |
| `ipc_deformable_scripted_dirichlet` | A banner billowing under a scripted Dirichlet BC      | Scripted Dirichlet boundary conditions     |

These are DART-native showcases, **not** faithful IPC paper-figure
reproductions. The _contact, barrier, friction, and projected-Newton machinery_
is genuine IPC; most scenes use a mass-spring elasticity model, while the
`ipc_deformable_fem_*` scenes (cantilever, twist, ground-barrier drop) exercise
the new opt-in stable neo-Hookean **FEM** volumetric elasticity (PLAN-081 M1,
the keystone toward the paper's volumetric scenes), including a volumetric body
in IPC contact. Codimensional collision and the upstream mesh corpus are still
needed for true figure reproduction. The catalog of upstream paper scenes lives at
`docs/plans/081-deformable-implicit-barrier-solver/ipc_scene_corpus_manifest.json`;
faithful corpus reproduction stays `planned` (it needs vendored assets,
codimensional collision, and broader FEM coverage), so these scenes evoke the
paper's _themes_ (volumetric elasticity, draping, self-contact, friction,
scripted boundaries) rather than reproducing
its figures.

The scenes share `_ipc_deformable_bridge.py`, which builds the grid topology and
mirrors each deformable onto the render world as per-node spheres plus a spring
wireframe (the dynamic surface-mesh render path is not yet exposed through
`dartpy.gui.run_demos`). `IpcDeformableBridge.build_diagnostics_panel(...)`
adds shared solver diagnostics such as node counts, fixed nodes, z-range, and a
minimum-height plot to custom scene panels. Add another deformable scene by
building its `DeformableBodyOptions` with `build_grid_options(...)` and
rendering it through `IpcDeformableBridge`.

## Simulation Replay

`replay_scrubber` demonstrates the experimental `World` replay recorder:
the scene records a rigid-body rollout once, restores the first frame, and
exposes a bottom-docked replay timeline with a scrubber, frame marks, a cursor
track, transport controls, loop/rate controls, and cursor details. The scene
uses the reusable `PanelBuilder.timeline(...)` widget from `dartpy.gui` for the
timeline lanes. Moving the scrubber calls `World.restore_replay_frame(...)` at
timestep resolution and does not re-run physics.

The same saved-state replay path is injected by the runner into every
`SceneSetup` that exposes an experimental `World` in `info["sx_world"]` or
`info["physics_world"]`. The shared panel stores only bounded mutable World
snapshots; static topology, geometry, materials, and scene construction data
remain owned by the scene and render bridge.

The shared demos toolbar uses the same timeline widget for captured-frame
playback after `Capture` records viewer frames. That path stores only the
captured image files and derives bounded marker/cursor tracks for the UI each
frame, so large capture directories do not require an equally large in-memory
timeline cache.

## Add a scene

1. Create `scenes/<name>.py` defining a module-level `SCENE` of type
   `PythonDemoScene` whose `build()` returns a `SceneSetup` (a `world` plus an
   optional custom `pre_step`, `step(n)`, `force_drag`, scene panels, and
   `info` dict).
2. Import it in `registry.py` and append it to the ordered list in
   `make_demo_scenes()`, placed within its category group.

That's the only change needed; no CMake edits, no test edits beyond the cycle
smoke (which iterates the registry).

Scene-specific controls use renderer-neutral `ScenePanel` callbacks:

```python
def build_panel(builder, context):
    builder.text(f"time: {world.time:.3f} s")
    changed, friction = builder.slider("Friction", body.friction, 0.0, 1.0)
    if changed:
        body.friction = friction

return SceneSetup(
    world=bridge.render_world,
    pre_step=bridge.pre_step,
    panels=[ScenePanel("Rigid IPC Slide", build_panel)],
)
```

The callback receives DART's `PanelBuilder` abstraction, not ImGui. Use the
builder for text, buttons, checkboxes, sliders, selects, plots, and tables; use
`context` only as a read-only viewer-state snapshot.
