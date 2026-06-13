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
by default and records bounded DART 7 World state snapshots while the
scene runs. Use the replay transport or scrubber to pause the live simulation,
restore a saved frame, and play the saved states without stepping physics again;
`Resume live` continues simulation from the selected restored state. Scenes with
live controller state outside the `World` can provide small replay-state
capture/restore callbacks; the shared panel stores those mutable controller
snapshots beside the World frames instead of storing static scene assets.

Capture a short frame sequence and request MP4 encoding when `ffmpeg` is
available:

```bash
pixi run py-demo-capture -- --scene articulated --frames 24 \
    --width 640 --height 360 --video
```

The lower-level viewer still accepts `--screenshot` and `--out` directly when
you need raw PPM output. Inspect the generated PNG or MP4 before calling a
visual change done.

## LCP Physics

The **`lcp_physics`** scene is the Python demo baseline for DART 7 contact LCP
work. It runs two matched `World` instances side by side: the default
sequential-impulse contact path and the boxed-LCP contact path. Both worlds run
the same representative packets:

| Packet                | Shows                                      | Metric surfaced in the panel               |
| --------------------- | ------------------------------------------ | ------------------------------------------ |
| Sliding friction      | Tangential Coulomb response on a flat slab | Sliding speed and contacts                 |
| Static-friction ramp  | Hold/slip behavior near the friction limit | Ramp-parallel slide                        |
| Billiard collision    | Symmetric contact impulse transfer         | Momentum and kinetic-energy error          |
| High-mass-ratio stack | Contact stability under large mass ratios  | Stack lateral drift and per-step wall time |
| Thin card pile        | Thin high-aspect-ratio stacked contacts    | Card spread and height loss                |

This scene compares the DART 7 public contact methods. The panel also surfaces
the 24-solver standalone LCP manifest, solver support matrix, and benchmark
packet map used for apples-to-apples solver work. It runs the flat `dartpy`
standalone LCP binding on a shared standard smoke problem for every solver, so
the panel can show status, iteration count, and solution error without going
through a contact `World`. The representative problem table lists every solver
on each packet and separates native support from delegated fallback solves, with
the fastest native solve highlighted for quick triage. The detailed table also
surfaces residual and complementarity so accuracy comparisons do not rely only
on solution error. A per-solver profile rolls those rows back up by solver so
native surfaces, delegated coverage, pass counts, worst error, worst residual,
and total demo-measured solve time are visible in one apples-to-apples matrix.
Its standard-LCP packet set includes well-conditioned, ill-conditioned,
near-singular, and moderate scale cases so conditioning and size changes are
visible without changing the solver roster. Its friction-index rows include both
a simple normal-scaled contact and a coupled two-contact active-bound packet.
The benchmark packet map includes per-packet benchmark filters, including the
active friction-index microbenchmark and the native `world_card_pile` packet for
high-aspect-ratio card-pile contact scaling.
Authoritative performance runs remain owned by
`tests/common/lcpsolver`, `tests/unit/math/lcp`, and `tests/benchmark/lcpsolver`.
The panel also points to the checked performance-profile CSVs under
`docs/background/lcp/figures` and summarizes the current leader/laggard
families per Standard, Boxed, and FrictionIndex profile row family. Refresh
those artifacts with:

```bash
pixi run python scripts/lcp_performance_profile.py --run \
    --cache build/lcp_profile_full.json \
    --output docs/background/lcp/figures
```

For a quick profile-pipeline smoke that allows partial native solver coverage
without rewriting checked artifacts, write to a scratch output directory:

```bash
pixi run python scripts/lcp_performance_profile.py --run \
    --allow-partial \
    --benchmark-filter BM_LcpCompare/Standard/Dantzig/12 \
    --benchmark-min-time 0.01 \
    --cache build/lcp_profile_smoke.json \
    --output build/lcp_profile_smoke \
    --benchmark-timeout 120
```

Run the benchmark smoke with:

```bash
pixi run bm lcp_compare -- --benchmark_filter=BM_LCP_COMPARE_SMOKE
```

## AVBD Rigid Constraints (sx)

The dedicated **`AVBD Rigid Constraints (sx)`** category groups the first
user-visible Augmented VBD rigid-constraint scenes from PLAN-104:

| Scene id                                                | Shows                                                                                       | AVBD capability exercised                                                        |
| ------------------------------------------------------- | ------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------- |
| `avbd_empty_baseline`                                   | An empty variational-integrator World stepping with render-only axes                        | Source-demo corpus baseline with 2D/3D empty-row revision/default metadata       |
| `avbd_demo2d_ground`                                    | The `avbd-demo2d` Ground source scene as its static 2D ground slab                          | Source-demo static ground row with matched source geometry and metadata          |
| `avbd_demo2d_motor`                                     | The `avbd-demo2d` Motor source scene as a pinned rigid bar with a velocity motor            | First non-empty source-demo row using public AVBD revolute motor rows            |
| `avbd_demo2d_dynamic_friction`                          | The `avbd-demo2d` Dynamic Friction source scene as sliding boxes over a ground slab         | Source-demo rigid friction row with matched source geometry and metadata         |
| `avbd_demo2d_static_friction`                           | The `avbd-demo2d` Static Friction source scene as rotated 2D boxes on an inclined slab      | Source-demo inclined-slab friction row with matched source geometry/metadata     |
| `avbd_demo2d_pyramid`                                   | The `avbd-demo2d` Pyramid source scene as a triangular pile of 2D rigid boxes               | Source-demo rigid stacking row with matched source geometry and metadata         |
| `avbd_demo2d_cards`                                     | The `avbd-demo2d` Cards source scene as a thin-card rigid contact tower                     | Source-demo card-tower contact row with matched source geometry and metadata     |
| `avbd_demo2d_stack`                                     | The `avbd-demo2d` Stack source scene as falling 2D boxes over a ground slab                 | Source-demo rigid stacking row with matched source geometry and metadata         |
| `avbd_demo2d_stack_ratio`                               | The `avbd-demo2d` Stack Ratio source scene as geometric-size 2D rigid boxes                 | Source-demo size-ratio stacking row with matched source geometry and metadata    |
| `avbd_demo2d_rod`                                       | The `avbd-demo2d` Rod source scene as a horizontal all-axis fixed-joint chain               | Source-demo rod row with matched hard constraints, geometry, and metadata        |
| `avbd_demo2d_soft_body`                                 | The `avbd-demo2d` Soft Body source scene as two finite-stiffness box lattices               | Source-demo soft-body row with finite all-axis joint stiffness and metadata      |
| `avbd_demo2d_joint_grid`                                | The `avbd-demo2d` Joint Grid source scene as a 25x25 fixed-joint box lattice                | Source-demo large joint-grid row with matched hard constraints and metadata      |
| `avbd_demo2d_rope`                                      | The `avbd-demo2d` Rope source scene as a horizontal point-jointed rigid chain               | Source-demo rope row with matched anchors, geometry, and metadata                |
| `avbd_demo2d_heavy_rope`                                | The `avbd-demo2d` Heavy Rope source scene with a large endpoint block                       | Source-demo high-ratio rope row with matched anchors, geometry, and metadata     |
| `avbd_demo2d_hanging_rope`                              | The `avbd-demo2d` Hanging Rope source scene as vertical point-jointed links                 | Source-demo long rope row with matched anchors, geometry, and metadata           |
| `avbd_demo2d_spring`                                    | The `avbd-demo2d` Spring source scene as one radial rigid distance spring                   | Source-demo finite-stiffness spring row with matched anchors and metadata        |
| `avbd_demo2d_spring_ratio`                              | The `avbd-demo2d` Spring Ratio source scene as alternating-stiffness radial springs         | Source-demo stiffness-ratio row with matched anchors, geometry, and metadata     |
| `avbd_demo2d_net`                                       | The `avbd-demo2d` Net source scene as endpoint-pinned links under falling rigid blocks      | Source-demo net row with matched anchors, geometry, and metadata                 |
| `avbd_demo2d_fracture`                                  | The `avbd-demo2d` Fracture source scene as falling 2D boxes over weak fixed joints          | Source-demo breakable fixed-joint row with public AVBD break thresholds          |
| `avbd_demo3d_ground`                                    | The `avbd-demo3d` Ground source scene as a falling rigid box over a static ground box       | Source-demo rigid contact row with matched source geometry and metadata          |
| `avbd_demo3d_dynamic_friction`                          | The `avbd-demo3d` Dynamic Friction source scene as sliding rigid boxes over ground          | Source-demo rigid friction row with matched source geometry and metadata         |
| `avbd_demo3d_static_friction`                           | The `avbd-demo3d` Static Friction source scene as rigid boxes on an inclined ramp           | Source-demo inclined-ramp friction row with matched source geometry/metadata     |
| `avbd_demo3d_pyramid`                                   | The `avbd-demo3d` Pyramid source scene as a triangular pile of rigid boxes                  | Source-demo rigid stacking row with matched source geometry and metadata         |
| `avbd_demo3d_rope`                                      | The `avbd-demo3d` Rope source scene as anchored rigid links with linear-only point joints   | Source-demo rope row with matched anchors, geometry, and metadata                |
| `avbd_demo3d_heavy_rope`                                | The `avbd-demo3d` Heavy Rope source scene with a large endpoint block                       | Source-demo high-ratio rope row with matched anchors, geometry, and metadata     |
| `avbd_demo3d_stack`                                     | The `avbd-demo3d` Stack source scene as ten rigid boxes over a static ground box            | Source-demo vertical stacking row with matched source geometry and metadata      |
| `avbd_demo3d_stack_ratio`                               | The `avbd-demo3d` Stack Ratio source scene as a four-box geometric size tower               | Source-demo size-ratio stacking row with matched source geometry and metadata    |
| `avbd_demo3d_soft_body`                                 | The `avbd-demo3d` Soft Body source scene as three finite-stiffness box lattices             | Source-demo 3D soft-body row with finite all-axis joint stiffness and metadata   |
| `avbd_demo3d_bridge`                                    | The `avbd-demo3d` Bridge source scene with paired point-jointed planks and load boxes       | Source-demo bridge row with matched anchors, geometry, and metadata              |
| `avbd_demo3d_breakable`                                 | The `avbd-demo3d` Breakable source scene as rigid boxes, contact shapes, and weak joints    | Source-demo breakable fixed-joint row with public AVBD break thresholds          |
| `avbd_rigid_fixed_joint_contact`                        | A fixed rigid payload sliding against static contact                                        | Public fixed-joint rows plus ordinary contact                                    |
| `avbd_rigid_revolute_motor`                             | A bounded revolute motor driving a free hinge                                               | Public velocity actuator mapped to AVBD motor rows                               |
| `avbd_rigid_prismatic_motor`                            | A bounded prismatic motor driving a free rigid slider                                       | Public linear velocity actuator mapped to AVBD motor rows                        |
| `avbd_articulated_revolute_motor`                       | A bounded revolute motor reversing an articulated floating link                             | Public articulated velocity motor updates through the bridge                     |
| `avbd_articulated_prismatic_motor`                      | A bounded prismatic motor reversing an articulated floating carriage                        | Public articulated linear motor updates through the bridge                       |
| `avbd_articulated_motor_breakable_joint`                | A weak articulated revolute motor releasing and re-engaging an articulated floating link    | Public articulated motor break/reset lifecycle through the bridge                |
| `avbd_articulated_prismatic_pair_motor_breakable_joint` | A weak same-multibody prismatic motor releasing and re-engaging an articulated carriage     | Public articulated linear motor break/reset lifecycle through the bridge         |
| `avbd_articulated_prismatic_motor_breakable_joint`      | A weak world-anchored prismatic motor releasing and re-engaging an articulated carriage     | Public articulated linear motor break/reset lifecycle through the bridge         |
| `avbd_articulated_world_revolute_motor_breakable_joint` | A weak world-anchored revolute motor releasing and re-engaging an articulated floating link | Public world-anchored articulated motor break/reset lifecycle through the bridge |
| `avbd_articulated_high_ratio_chain`                     | A five-link variational chain swinging with a 200:1 heavy tip                               | Narrow articulated high mass-ratio smoke for the AVBD paper gap                  |
| `avbd_paper_scale_high_ratio_chain`                     | A 50-link variational chain swinging with a 50,000:1 heavy tip                              | Paper-scale articulated high mass-ratio visual smoke for the AVBD paper gap      |
| `avbd_rigid_breakable_joint`                            | A weak fixed joint releasing and re-engaging a rigid payload                                | Public break-force threshold, reset, and broken-state path                       |
| `avbd_rigid_spherical_breakable_joint`                  | A weak spherical point joint releasing and re-engaging only a rigid payload anchor          | Public free-rigid spherical break/reset while orientation stays free             |
| `avbd_articulated_breakable_joint`                      | A world-anchored floating link released and reset through a public articulated fixed joint  | Public articulated world-link break/reset path through the variational bridge    |
| `avbd_articulated_fixed_pair_breakable_joint`           | A same-multibody fixed joint releasing and restoring a captured relative pose               | Public articulated link-link fixed break/reset through the variational bridge    |
| `avbd_articulated_spherical_breakable_joint`            | A world-anchored floating socket that releases and resets only its anchor                   | Public articulated spherical break/reset with orientation left free              |
| `avbd_articulated_spherical_pair_breakable_joint`       | A same-multibody floating socket that releases and resets only its anchor                   | Public articulated link-link spherical break/reset with orientation left free    |

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

## PLAN-083 CPU Corpus

The **`PLAN-083 Mixed Corpus`**, **`PLAN-083 Constraints Corpus`**,
**`PLAN-083 Robot Corpus`**, and **`PLAN-083 ABD Corpus`** categories expose
launchable placeholders for the unified Newton-barrier paper/deck scene rows.
Each placeholder records its manifest row IDs, smoke command, long-horizon
visual capture command, benchmark packet target, and current limitation. They
are not paper-scene reproductions yet; the checked corpus sidecar lives at
`docs/plans/083-unified-newton-barrier-multibody/cpu-scene-corpus.json` and
keeps the py-demo, visual evidence, and CPU benchmark obligations explicit until
runtime mixed-domain stepping and ABD scene support land.

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

`replay_scrubber` demonstrates the DART 7 `World` replay recorder:
the scene records a rigid-body rollout once, restores the first frame, and
exposes a bottom-docked replay timeline with a scrubber, frame marks, a cursor
track, transport controls, loop/rate controls, and cursor details. The scene
uses the reusable `PanelBuilder.timeline(...)` widget from `dartpy.gui` for the
timeline lanes. Moving the scrubber calls `World.restore_replay_frame(...)` at
timestep resolution and does not re-run physics.

The same saved-state replay path is injected by the runner into every
`SceneSetup` that exposes a DART 7 `World` in `info["sx_world"]` or
`info["physics_world"]`. The shared panel stores only bounded mutable World
snapshots plus optional small scene-provided mutable controller snapshots;
static topology, geometry, materials, and scene construction data remain owned
by the scene and render bridge.

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
