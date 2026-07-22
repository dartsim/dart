# dart-demos

`dart-demos` is the consolidated DART 6 demo application: a single
`dart::gui::osg::ImGuiViewer` host that runs every GUI demo as a
runtime-switchable *scene*, selected from a categorized navigator. It replaces
the scattered per-example programs that used to live under `examples/*`.

Run it with `pixi run demos` (or `pixi run demos --scene <id>`). The dartpy
counterpart is `pixi run py-demos` (see `python/examples/demos/`).

The FBF exact-Coulomb paper inspection scenes live under the `Research`
category. Each FBF scene's `Scene` panel includes a self-contained overview,
expected result, coverage-limit note, solver-mode controls, contact count, and
available exact-FBF diagnostics so the example can be understood without reading
the source:

```bash
pixi run demos -- --list-scenes
pixi run demos -- --verify-fbf-scene-docs
pixi run demos -- --scene fbf_paper_incline
pixi run demos -- --scene fbf_paper_backspin
pixi run demos -- --scene fbf_paper_turntable
pixi run demos -- --scene fbf_paper_turntable_mu_0_2_omega_2
pixi run demos -- --scene fbf_paper_turntable_mu_0_2_omega_5
pixi run demos -- --scene fbf_paper_turntable_mu_0_5_omega_5
pixi run demos -- --scene fbf_author_turntable_mu_0_2_omega_2
pixi run demos -- --scene fbf_author_turntable_mu_0_2_omega_5
pixi run demos -- --scene fbf_author_turntable_mu_0_5_omega_2
pixi run demos -- --scene fbf_author_turntable_mu_0_5_omega_5
pixi run demos -- --scene fbf_author_card_house_5_construction
pixi run demos -- --scene fbf_author_card_house_4_impact_current_source
pixi run demos -- --scene fbf_author_card_house_4_impact_source_continuation_current_source
pixi run demos -- --scene fbf_author_card_house_10_impact_current_source
pixi run demos -- --scene fbf_author_card_house_10_impact_source_continuation_current_source
pixi run demos -- --scene fbf_author_masonry_arch_25_crown_impact_current_source
pixi run demos -- --scene fbf_author_masonry_arch_25_crown_impact_source_continuation_current_source
pixi run demos -- --scene fbf_paper_painleve
pixi run demos -- --scene fbf_paper_painleve_mu_0_55
pixi run demos -- --scene fbf_author_painleve_mu_0_5
pixi run demos -- --scene fbf_author_painleve_mu_0_55
pixi run demos -- --scene fbf_paper_card_aframe
pixi run demos -- --scene fbf_paper_card_house_26
pixi run demos -- --scene fbf_paper_card_house_10
pixi run demos -- --scene fbf_paper_card_house_10_dynamic
pixi run demos -- --scene fbf_paper_masonry_arch_25_literal_standing
pixi run demos -- --scene fbf_paper_masonry_arch_25
pixi run demos -- --scene fbf_paper_masonry_arch_101
```

The generic turntable scene starts at `mu=.5`, `omega=2 rad/s` and keeps its
interactive angular-speed slider. The three parameter-suffixed turntable scenes
are fixed capture targets; together with the generic default they cover the
paper's four parameter cells. Likewise, the generic Painleve proxy is the
`mu=.5` cell and the suffixed scene fixes `mu=.55`.

The separate `fbf_author_painleve_mu_0_5` and
`fbf_author_painleve_mu_0_55` scenes preserve the current public author's
0.3 x 1.2 x 0.6 m box, 200 kg/m^3 density (43.2 kg mass), upright pose,
4 m/s launch speed, finite 10 x 3 x .1 m ground, 1/60 s step, and two-second
horizon. The `.55` value is the source default; the `.5/.55` pair is a
source-supported CLI sweep selected for the paper-video cells, not a recovered
historical invocation. The exact scene maps the public CLI's `gamma_c=5` onto
DART's adaptive safe-step convention (scale ten), retains strict `1e-6`
convergence, and allows up to 1,000 DART outer iterations; this is an adapter
policy, not solver-backend equivalence. Both lanes use Native FourPointPlanar
contact. Source `gap=.005`, `ke=1e4`, and `kd=1e3` are recorded but not claimed
as equivalent DART contact semantics. The older `fbf_paper_painleve*` scenes
remain historical proxy diagnostics with different geometry, density, pose,
velocity, and duration.

The unsuffixed ten-level card scene remains a static construction inspector.
`fbf_paper_card_house_10_dynamic` is a separate 155-mobile-card exact-FBF
adapter. Its 512-contact, eight-per-pair budget is already known to saturate in
the boxed-LCP construction probe, so the dynamic scene is not full-natural-
manifold or performance evidence. Accept a timed capture only when its sidecar
reports exact-FBF solves with zero exact failures and zero boxed-LCP fallbacks.

The literal 25-stone masonry scene is a separate no-projectile standing
contract using convex voussoir wedges, exact prism inertia, Native
FourPointPlanar contacts, and scoped ERP restoration; the older
`fbf_paper_masonry_arch_25` scene remains the reduced oriented-box projectile
proxy.

`fbf_author_card_house_4_impact_current_source` is the dynamic counterpart to
the five-level construction still. It selects the public author's supported
CLI arguments `--solvers fbf --levels 4 --frames 600 --drop-frame 400
--num-cubes 4 --mu 0.8 --cube-size 0.4 --cube-density 500 --drop-height 1.0
--device cpu --profile --usd`: 26 source-sized cards and four initially
kinematic 0.8 m cubes. Interactive `p` releases those existing cubes
immediately; the evidence runner invokes it after completed substep 1600 in a
2400-substep, 10-second schedule. Source `ke=1e4`, `kd=1e3`, and `gap=.005`
are recorded source semantics, not contact semantics implemented equivalently
by the DART adapter. The older `fbf_paper_card_house_26` remains a distinct
reconstructed diagnostic with different card and projectile parameters. The
new adapter does not claim that its selected CLI arguments were the historical
paper invocation, nor source-backend, trajectory, outcome, or timing
equivalence, Fig. 6/video parity, or final media. It remains an adapter-only
lane.

`fbf_author_card_house_4_impact_source_continuation_current_source` reuses the
same card-house bodies, release action, time step, capture schedule, and
source-style inner initialization while additionally requesting the separate
source-continuation termination and gamma policy. Its headless gate is
deliberately separate from the strict exact-FBF fail-fast gate: it accepts only
finite `success`, `plateau_accepted`, or
`max_iterations_accepted` group outcomes with complete per-group telemetry and
no boxed-LCP fallback. The strict scene and its residual/cap gate remain
unchanged, so the two policies produce distinct evidence lanes rather than
silently weakening the existing one.

The separate ten-level current-source pair follows the same boundary.
`fbf_author_card_house_10_impact_current_source` binds the supported
`--levels 10 --frames 800` selection to 155 cards, four existing cubes, the
source's heterogeneous per-shape gap values, 3,200 DART substeps, and a
runner-scheduled release after completed step 1,600. The additive
`fbf_author_card_house_10_impact_source_continuation_current_source` scene
keeps that geometry, contact frontend, clock, camera, and release action while
requesting continuation only in its exact-FBF lane. The paper video contains
no ten-level segment, and the available source output has no post-release
physical oracle, so successful synchronized media remains DART continuation
evidence rather than strict convergence, Tables 6-7 reproduction, trajectory
or physical parity, or solver superiority.

`fbf_author_masonry_arch_25_crown_impact_current_source` preserves the public
author repository's raw numeric 25-wedge geometry, mass/friction values, three
initially kinematic cubes, and the declared 500-frame diagnostic schedule. Its
`p` action releases those existing cubes immediately without respawning,
moving, or accelerating them; the evidence runner invokes that action after
completed substep 1600. DART Native collision, split impulse, float64
arithmetic, exact/boxed solvers, camera, and rendering remain adapter choices;
the scene does not claim source trajectory/outcome, the paper's 100-contact
timing row, Fig. 7/video parity, or timing comparability.

`fbf_author_masonry_arch_25_crown_impact_source_continuation_current_source`
is a separately named policy lane over the same geometry, collision frontend,
clock, 2,000-substep horizon, and completed-step 1,600 cube release. Its exact
solver requests bounded source-continuation termination plus strict
ordered-body-B cross-step matching; switching to the boxed solver disables
continuation without changing the scene physics. This additive lane does not
weaken the existing strict scene and does not claim strict convergence, source
trajectory or physical-outcome equivalence, Fig. 7/video parity, timing
comparability, or solver superiority.

For a bounded off-screen GUI smoke of one scene, use the same app through the
Pixi capture task. The final argument is the number of deterministic simulation
steps before the screenshot:

```bash
pixi run capture fbf_paper_backspin /tmp/fbf_paper_backspin.png 640 480 1
pixi run image-verdict /tmp/fbf_paper_backspin.png
```

For a bounded capture of a scene state exposed through a GUI/key action, use
the action-aware capture task. The 26-card FBF paper scene binds `p` to a
reconstructed vertical drop of four projectile cubes, and the 25-stone arch
scene binds `p` to a reconstructed row of small cubes dropped over the crown:

```bash
pixi run capture-action fbf_paper_card_house_26 p /tmp/fbf_card_house_projectiles.png 640 480 0
pixi run image-verdict /tmp/fbf_card_house_projectiles.png
pixi run capture-action fbf_paper_masonry_arch_25 p /tmp/fbf_arch_projectile.png 640 480 0
pixi run image-verdict /tmp/fbf_arch_projectile.png
```

FBF paper-parity scenes may extend the reusable OSG renderer, `dart-demos`
host, or ImGui widgets when a fixture needs better overlays, camera/snapshot
capture, inspection controls, or in-GUI explanation. A GUI scene is not enough
for this task unless its `Scene` panel explains the example's overview,
expected result, and coverage limits without requiring source-code context.

## Architecture

- **Host** (`DemoHost`, `main.cpp`): owns the one window, the ImGui theme
  (`Theme.*`), the scene registry, and the persistent chrome (simulation
  toolbar, `Demos` navigator, `Diagnostics` panel, and the visual-debugging
  panels). It swaps the active scene at runtime.
- **Registry** (`Registry.cpp`, `Scenes.hpp`): `makeDemoScenes()` returns an
  ordered `std::vector<DemoScene>`. A `DemoScene` is data — `{id, title,
  category, summary, factory}` — where `factory` is a lazily-invoked
  `std::function` that builds the scene only when it is first selected. Research
  scenes that need to be self-contained also fill `scenePanelDocumentation`,
  which can be checked with `pixi run demos -- --verify-fbf-scene-docs`.
  Categories render in first-appearance order; scenes in registry order
  within a category.
- **Scenes** (`scenes/*.cpp`, one factory each; multi-file scenes in
  subdirectories): each factory returns a `DemoSceneSetup` (`DemoScene.hpp`) —
  the world, optional `preStep`/`postStep`/`preRefresh` controller hooks, a
  per-scene ImGui `renderPanel`, `KeyAction`s (auto-mirrored as panel
  buttons), a camera home, drag frames, and an `onActivate` hook for
  viewer-level extras. Per-scene mutable state lives in a `shared_ptr`
  captured by the lambdas.
- **Visual debugging** (`Inspector`, `ContactVisualizer`, `DragForce`,
  `LogCapture`, `Profiler`): host-level, scene-agnostic facilities wired into
  the composed step/refresh hooks and the Diagnostics panel.

## Adding a scene

1. Write `scenes/MyScene.cpp` with a `dart_demos::makeMyScene()` factory
   returning a `DemoSceneSetup` (copy an existing scene as a template;
   `RigidCubesScene.cpp` is a good controller+visual example).
2. Declare it in `Scenes.hpp` and register it in `Registry.cpp`.
3. The executable globs `scenes/*.cpp`, so no CMake edit is needed for a
   single-file scene. Multi-file scenes and optional-dependency scenes are
   added in `CMakeLists.txt` (see the `wam_ikfast`, `atlas_simbicon`,
   `human_joint_limits`, and `ssik_ik_gui` blocks).

## Host conventions (enforced in review)

These keep the app crash-safe and consistent — a demo must **never crash**,
whatever the user changes at runtime:

- **Clamp every tunable.** Every `ImGui::SliderFloat`/`InputFloat` passes
  `ImGuiSliderFlags_AlwaysClamp` *and* commits through a local copy guarded by
  `std::clamp` + `std::isfinite` (ImGui's Ctrl+click text entry bypasses the
  drag range, so `AlwaysClamp` alone is not enough — a typed NaN must never
  reach physics or geometry).
- **World-owned visuals only.** Debug geometry is a `SimpleFrame`/`ArrowShape`
  added to the scene's world (torn down with it), never a raw node injected
  into the OSG graph.
- **Full teardown.** Anything registered through `DemoHostContext`
  (attachments, event handlers, drag-and-drop) is released via
  `ctx.addTeardown`. Facilities that hold `BodyNode*`/`Skeleton*` across a
  possible mid-scene deletion derive from `dart::common::Observer` and drop
  the pointer in `handleDestructionNotification` (see `Inspector`,
  `DragForce`).
- **Frame-loop thread only.** All world mutation happens on the frame-loop
  thread (the viewer runs `SingleThreaded`); panel edits apply in
  `preStep`/`preRefresh` or via queued commands, never mid-render.
- **Re-read `dt` per step.** Controllers that use the timestep read
  `world->getTimeStep()` each step — the toolbar Timestep control changes it
  live.
- **Lowercase keys, ImGui-gated.** Key actions register lowercase (the host
  matches case-insensitively) and skip when
  `ImGui::GetIO().WantCaptureKeyboard` (typing in the search box must not
  drive a scene).
- **Z-up.** Y-up `.skel` worlds are reoriented via `scenes/ZUp.hpp`, which
  preserves the source gravity magnitude; force/velocity vectors and their
  UI labels are remapped together.

## CLI

- `--list-scenes` — print the catalog grouped by category, exit 0.
- `--scene <id>` — start on a specific scene.
- `--cycle-scenes [--frames N]` — headless: build every scene, step N frames
  each, twice (a leak/robustness audit); exit nonzero on any factory failure.
- `--headless --shot <path> [--steps N]` — off-screen pbuffer capture
  (requires a DISPLAY/GPU; a local self-verification tool, not a CI gate).
- `--headless-action <key>` — invoke a scene key action before an off-screen
  capture step sequence; may be repeated.
- `--collision-detector <name>` / `COLLISION_DETECTOR=<name>` — start a
  scene with a specific registered backend (`fcl`, `dart`, `native`, `bullet`,
  or `ode` when available). The toolbar can switch backends while the scene is
  running. Soft-body scenes should use `dart` or `fcl` for apples-to-apples
  comparisons; the `native` adapter is selectable for diagnostics, but does not
  yet cover `SoftMeshShape`.
- `--threads <n>` / `THREADS=<n>` — start with a specific simulation worker
  count (`0` selects hardware concurrency). The toolbar can change this live.
- `--debug-select-body <name>` / `--debug-record-profile` — hidden hooks that
  let a headless capture exercise the inspector/profiler panels.

The legacy soft-body commands are retained as thin aliases over `dart-demos`:

    $ pixi run ex soft_bodies -- --collision-detector dart --threads 16
    $ pixi run ex soft_cubes -- --collision-detector fcl --threads 1
    $ pixi run ex soft_open_chain -- --collision-detector dart --threads 4
