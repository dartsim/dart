# DART Changelog

## DART 7

### [DART 7.0.0 (TBD)](https://github.com/dartsim/dart/milestone/TBD?closed=1)

- Breaking Changes
  - Increased required C++ standard from C++17 to C++20. ([#2068](https://github.com/dartsim/dart/pull/2068))
    - See [Compatibility Policy](docs/onboarding/compatibility-policy.md) for details
  - Increased the minimum required CMake version to 4.2.3 for source builds,
    Pixi environments, and installed-package smoke projects.
  - Removed the `dart/common/filesystem.hpp` compatibility header; include `<filesystem>` and use `std::filesystem`/`std::error_code` directly.
  - Renamed `RootJointType` enum values to PascalCase (`Floating`, `Fixed`) across `dart::utils::SdfParser`, `dart::utils::UrdfParser` (formerly `DartLoader`), and their dartpy bindings to align with the code-style guidelines.
  - Renamed component meta headers to `All.hpp` and added the `dart/all.hpp` umbrella header; deprecated `<component>.hpp`/`all.hpp` headers now forward to `All.hpp`. ([#2047](https://github.com/dartsim/dart/pull/2047), [#2070](https://github.com/dartsim/dart/pull/2070), [#2084](https://github.com/dartsim/dart/pull/2084), [#2166](https://github.com/dartsim/dart/pull/2166))
  - Renamed installed C++ headers to snake_case; PascalCase compatibility wrappers are generated with deprecation warnings for legacy includes. ([#2475](https://github.com/dartsim/dart/pull/2475))
  - Removed all GLUT code and converted GLUT examples to OSG. ([#2044](https://github.com/dartsim/dart/pull/2044))
  - Renamed the OpenSceneGraph GUI component/target to `gui`/`dart-gui` (previously `gui-osg`/`dart-gui-osg`) and replaced the `DART_BUILD_GUI_OSG` toggle with `DART_BUILD_GUI`. ([#2209](https://github.com/dartsim/dart/pull/2209), [#2257](https://github.com/dartsim/dart/pull/2257))
  - Renamed `DartLoader` to `UrdfParser` and standardized parser naming in model-loading APIs. ([#2269](https://github.com/dartsim/dart/pull/2269), [#2270](https://github.com/dartsim/dart/pull/2270))
  - Replaced the pybind11 dartpy bindings with nanobind and flattened the dartpy namespace; legacy modules and camelCase remain available with `DeprecationWarning` only while DART 7 clean-break gates are being closed, and they are not part of the DART 7 public contract (see `DARTPY_ENABLE_LEGACY_MODULES`, `DARTPY_WARN_ON_LEGACY_MODULES`, `DARTPY_ENABLE_SNAKE_CASE`, `DARTPY_WARN_ON_CAMELCASE`). ([#2249](https://github.com/dartsim/dart/pull/2249), [#2256](https://github.com/dartsim/dart/pull/2256), [#2259](https://github.com/dartsim/dart/pull/2259))
  - Removed the legacy integration module and moved the optimizer component to `dart-optimization`; optional optimizer plugins and pagmo APIs are removed. ([#2201](https://github.com/dartsim/dart/pull/2201), [#2204](https://github.com/dartsim/dart/pull/2204))
  - Removed build-mode macros from public config headers and prefixed feature macros with `DART_`. ([#2275](https://github.com/dartsim/dart/pull/2275), [#2278](https://github.com/dartsim/dart/pull/2278))
  - Removed deprecated experimental example and benchmark directories.
  - Removed C3D/VSK utilities, parsers, and sample data. ([#2363](https://github.com/dartsim/dart/pull/2363))
  - Dropped the deprecated `docker/dev/v6.15` images; use the maintained v6.16 images instead.
  - Promoted Filament with GLFW3 and Dear ImGui as the only maintained renderer,
    replacing OpenSceneGraph and the Raylib experiment. The legacy OSG GUI
    sources, OSG/Raylib CMake dependency discovery, legacy C++/Python GUI
    examples and tutorials, and legacy dartpy GUI bindings/stubs were removed.
  - Restored historical GUI example executable names as `dart::gui` launchers
    backed by `dartsim`, and added `--out <dir>` PPM image-sequence capture
    alongside the existing `--screenshot <path>` final-frame capture.
  - Consolidated the standalone GUI example executables into a single
    `dart-demos` application (`examples/demos`) that hosts each example as a
    scene selectable at runtime from a categorized sidebar (`--scene <id>`
    selects the initial scene; `--cycle-scenes` drives the headless smoke).
    Added a runtime scene-switch capability to `dart::gui` via
    `dart::gui::runDemos`. The classic standalone GUI examples
    (`hello_world`, `gui_scene_diagnostics`, and `speed_test`) were retired
    from `main`; release-6.\* branches remain the parity source for those DART 6
    examples. The headless `csv_logger`, `headless_simulation`, and
    `unified_loading` examples were also retired because they taught the removed
    DART 6 whole-World loading API.
  - Pruned `pixi run demos` and `pixi run py-demos` to the DART 7 World demo
    catalog. The old DART 6 demo scene modules and cross-language golden parity
    fixtures were removed; remaining scene ids/categories use World, IPC,
    differentiable, variational, and VBD names without experimental/sx labels.
    High-value DART 6 examples that still need full World-native ports now
    appear as lightweight `Planned World Ports` placeholders or conservative
    previews: `atlas_puppet` and `hubo_puppet` load bundled humanoids through
    the experimental World with pose controls, `g1_puppet` remains asset-gated,
    `atlas_simbicon` cycles SIMBICON target poses, and IK, full walking
    balance, operational-space control, collision sandbox, and
    mobile-manipulation follow-ups stay visible without keeping legacy scene
    implementations.
  - Added runtime rendering-backend (graphics API) selection through
    `dart::gui::RunOptions::renderBackend`, the `--render-backend` flag, and the
    `DART_FILAMENT_BACKEND` environment variable (`default`/`opengl`/`vulkan`
    both GPU, `noop` for CPU-only/no-render) with graceful fallback and a startup
    log line; embedded materials are now compiled for both OpenGL and Vulkan so
    the backend is selectable without a separate build. No backend types are
    exposed through public headers.
  - Added a diagnostic Filament offscreen render-to-texture parity self-check
    (`DART_GUI_OFFSCREEN_PARITY` on the headless path, runnable via
    `pixi run gui-offscreen-parity`) that renders the scene to an offscreen
    `RenderTarget` and verifies it matches the swapchain render. The proven
    render-to-texture path is the prerequisite for headless sensor cameras and a
    future composited or streamed viewer.
  - Added a live in-app performance HUD (`--perf-hud`, also toggleable at runtime
    with `F2`) that overlays smoothed CPU per-phase timings, GPU frame time (from
    the Filament frame-info history), FPS, a real-time-factor (sim-vs-wall)
    readout, fixed-scale CPU/GPU history plots against the 60 FPS budget, the
    active backend, and frame counters, building on the existing GUI
    `ProfileAccumulator` without a separate timing system.
  - Restyled the `dart::gui` Dear ImGui viewer with a cohesive modern dark
    theme (VS Code/Blender/Unity-inspired) in place of the stock `StyleColorsDark`
    look: cool neutral surfaces, a single restrained blue accent used for
    selection/focus/active state, VS Code-style active-tab accent overlines,
    flat hairline-bordered widgets with soft rounding, pill scrollbars, and more
    breathable spacing. Centralized in `configureImGuiStyle` so every panel, the
    perf HUD, and the docked regions share one look; colors are scale-independent
    while metrics still scale with `--gui-scale`/DPI.
  - Enabled GPU (CUDA) deformable solve in the Python demos: `pixi run -e cuda
py-demos` now builds a CUDA-enabled dartpy + Filament GUI and offloads the
    deformable projected-Newton PSD projection to the GPU by default. New
    backend-neutral dartpy functions
    `simulation_experimental.is_accelerated_deformable_solve_available()`,
    `set_accelerated_deformable_solve(enabled)`, and
    `is_accelerated_deformable_solve_enabled()` expose the process-wide toggle.
    The CUDA sidecar registers itself with a backend-neutral core control, so the
    public API never names a device technology and is a safe no-op on non-CUDA
    builds, leaving the default environment unchanged. The demo runner adds `--gpu`/`--no-gpu`
    flags and a `DART_PY_DEMOS_GPU` env override (default `auto`: on when a CUDA
    device is present), an in-viewer GPU toggle panel, and a startup status
    line. The `config-py` build task forwards the experimental CUDA opt-in so
    the lean compute-only `build-cuda`/`test-cuda` paths are unaffected, and it
    resets stale CMake compiler cache metadata before CMake can auto-rerun with
    default options and drop the `dartpy` target.
  - Added public `World` lookup/list/count accessors for experimental
    rigid-body fixed joints, plus matching dartpy bindings and py-demo
    diagnostics, so users can recover fixed-joint handles and inspect their
    rigid endpoints after construction or save/load without touching ECS
    internals.
  - Added public experimental `World` rigid-body revolute and prismatic joint
    facades with C++ and dartpy bindings, generated stubs, tests, and a
    `sx_rigid_limited_joints` py-demo so users can exercise the narrow AVBD
    one-DOF rigid-joint path without touching ECS internals.
  - Added public experimental joint break-force, broken-state, and reset
    accessors, wired free rigid-body AVBD point joints to mark and skip broken
    joints, and added `avbd_rigid_breakable_joint` and
    `avbd_rigid_spherical_breakable_joint` py-demos for narrow free-rigid
    break/re-engagement lifecycles, with a tracked
    `avbd-rigid-breakable-joint-packet.json` visual/benchmark packet for the
    fixed point-joint path and a tracked
    `avbd-rigid-spherical-breakable-joint-packet.json` visual/benchmark packet
    for the spherical point-joint path.
  - Bridged private hard fixed-link AVBD point-joint endpoints into the
    articulated variational solve path, covered that private path with C++ tests
    and a benchmark row, and added a `variational_endpoint_loop_closure` py-demo
    for public multibody-link point-closure behavior. Non-topology multibody-link
    private point-joint entities can now also generate hard AVBD configs from
    the simulation-entry current pose while tree-topology joints stay skipped,
    and public experimental `World` articulated
    fixed/revolute/prismatic/spherical link-link and world-link facades now
    expose that narrow AVBD bridge through
    C++ and dartpy, including focused public revolute/prismatic bounded
    velocity-actuator, same-multibody/world-anchored command-update and tiny
    effort-limit coverage, private revolute/prismatic command-update plus
    fixed-row and revolute/prismatic break/reset re-engagement, and
    same-multibody/world-anchored public one-DOF motor break/skip coverage
    including focused dartpy stepping tests for same-multibody/world-link
    revolute and prismatic explicit-anchor break/skip and reset/re-engagement
    cases, plus
    dartpy fixed point-joint break/skip/reset coverage for same-multibody,
    movable-movable same-multibody, and world-link explicit all-axis anchor
    rows, plus
    same-multibody/world-anchored public one-DOF motor break/reset
    re-engagement, plus movable-movable same-multibody revolute/prismatic motor
    break/reset regressions with explicit off-origin anchors covered in C++ and
    dartpy and fixed break/reset regressions, plus a world-fixed
    break/reset regression. The
    articulated point-joint facade also accepts explicit local/world anchor
    points in C++ and dartpy, with same-multibody and world-anchored
    fixed/revolute/prismatic off-origin regressions, and exposes spherical
    articulated point joints as linear-only pinned-anchor rows that leave
    relative orientation free, including explicit link-link and world-link
    anchor overload coverage plus focused spherical break/skip and
    same-multibody/world-link reset coverage for those linear-only rows,
    including explicit-anchor dartpy stepping coverage and C++ save/load
    regressions proving same-multibody link-link
    and world-link fixed/spherical facade state rebuilds the private AVBD
    all-axis or linear rows after load, plus same-multibody and world-link
    revolute/prismatic motor
    facade state rebuilds the private hard rows and free-axis motor row after
    load, plus same-multibody/world-link fixed/spherical/revolute/prismatic
    broken-state save/load regressions, including explicit-anchor fixed,
    movable-movable fixed, and one-DOF motor cases, proving restored broken
    joints remain skipped until reset re-engages the AVBD rows.
    Direct private world-link `AvbdRigidWorldPointJointConfig`
    revolute/prismatic rows now also survive broken-state binary save/load,
    preserving anchors, bases, masks, stiffness fields, actuator command, and
    effort-limit state until reset re-engages the hard rows plus updated
    free-axis motor motion.
    Multibody position writes now also mark link frame caches dirty before the
    clean-cache kinematics graph shortcut, keeping public link transforms
    current after semi-implicit and variational multibody steps.
    Added
    `avbd_articulated_revolute_motor`
    and `avbd_articulated_prismatic_motor` py-demos for public articulated
    velocity-motor command updates, with a tracked
    `avbd-articulated-revolute-motor-packet.json` visual/benchmark packet for
    the revolute path and a tracked
    `avbd-articulated-prismatic-motor-packet.json` visual/benchmark packet for
    the prismatic path,
    an `avbd_articulated_motor_breakable_joint` py-demo for same-multibody
    public articulated motor break/reset with a tracked
    `avbd-articulated-breakable-motor-packet.json` visual/benchmark packet,
    an `avbd_articulated_prismatic_pair_motor_breakable_joint` py-demo for
    same-multibody public articulated prismatic motor break/reset with a
    tracked `avbd-articulated-prismatic-pair-breakable-motor-packet.json`
    visual/benchmark packet,
    an `avbd_articulated_prismatic_motor_breakable_joint` py-demo for
    world-anchored public articulated prismatic motor break/reset with a
    tracked `avbd-articulated-world-prismatic-breakable-motor-packet.json`
    visual/benchmark packet, an
    `avbd_articulated_world_revolute_motor_breakable_joint` py-demo for
    world-anchored public articulated revolute motor break/reset with a tracked
    `avbd-articulated-world-revolute-breakable-motor-packet.json`
    visual/benchmark packet, a
    `avbd_articulated_breakable_joint` py-demo for public articulated fixed
    point-joint break/reset with a tracked
    `avbd-articulated-breakable-joint-packet.json` visual/benchmark packet, a
    `BM_AvbdArticulatedBreakableMotorStep` dashboard row for the active
    break-force armed articulated revolute motor path, a
    `BM_AvbdArticulatedPrismaticBreakableMotorStep` dashboard row for the
    active break-force armed articulated prismatic motor path, a
    `BM_AvbdArticulatedWorldPrismaticBreakableMotorStep` dashboard row for the
    active break-force armed world-anchored articulated prismatic motor path,
    a `BM_AvbdArticulatedWorldRevoluteBreakableMotorStep` dashboard row for
    the active break-force armed world-anchored articulated revolute motor path,
    and an `avbd_articulated_fixed_pair_breakable_joint` py-demo
    for the public same-multibody fixed break/reset path with a tracked
    `avbd-articulated-fixed-pair-breakable-joint-packet.json` visual/benchmark
    packet, an
    `avbd_articulated_spherical_breakable_joint` py-demo for
    the public world-link spherical break/reset path with a tracked
    `avbd-articulated-spherical-breakable-joint-packet.json` visual/benchmark
    packet, and an
    `avbd_articulated_spherical_pair_breakable_joint` py-demo for the public
    same-multibody spherical break/reset path over the variational bridge with a
    tracked `avbd-articulated-spherical-pair-breakable-joint-packet.json`
    visual/benchmark packet, plus
    an `avbd_articulated_high_ratio_chain` py-demo for a narrow 200:1
    articulated high mass-ratio variational-chain smoke case and
    `BM_AvbdArticulatedHighRatioChainStep` dashboard row with a tracked
    `avbd-articulated-high-ratio-chain-packet.json` visual/benchmark packet,
    plus `avbd_paper_scale_high_ratio_chain`,
    `BM_AvbdPaperScaleHighRatioChainStep`, and a tracked
    `avbd-paper-scale-high-ratio-chain-packet.json` visual/benchmark packet for
    a 50-link/50,000:1 CPU smoke that keeps the same-hardware and GPU gates open,
    plus tracked `avbd-rigid-revolute-motor-packet.json` and
    `avbd-rigid-prismatic-motor-packet.json` visual/benchmark packets for the
    public free-rigid revolute and prismatic motor rows,
    plus
    an `avbd_empty_baseline` py-demo
    for the first source-demo empty-scene smoke baseline,
    `avbd_demo2d_ground` for the static `avbd-demo2d` Ground source row, and
    `avbd_demo2d_motor` for the first one-DOF motor `avbd-demo2d` source
    row, plus `avbd_demo2d_hanging_rope` for the `avbd-demo2d` Hanging Rope
    source row, plus `avbd_demo2d_spring` and
    `avbd_demo2d_spring_ratio` for the first radial rigid distance-spring source
    harnesses, plus `avbd_demo2d_fracture` for the `avbd-demo2d` Fracture
    source row, plus `avbd_demo2d_dynamic_friction` for the `avbd-demo2d`
    Dynamic Friction source row, plus `avbd_demo2d_static_friction` for the
    `avbd-demo2d` Static Friction source row, plus `avbd_demo2d_pyramid` for
    the `avbd-demo2d` Pyramid source row, plus `avbd_demo2d_cards` for the
    `avbd-demo2d` Cards source row, plus `avbd_demo2d_stack` for the
    `avbd-demo2d` Stack source row, plus `avbd_demo2d_stack_ratio` for the
    `avbd-demo2d` Stack Ratio source row, plus `avbd_demo2d_rod` for the
    `avbd-demo2d` Rod source row, plus `avbd_demo2d_soft_body` for the
    `avbd-demo2d` Soft Body source row, plus `avbd_demo2d_joint_grid` for the
    `avbd-demo2d` Joint Grid source row, plus `avbd_demo2d_rope` for the
    `avbd-demo2d` Rope source row, plus `avbd_demo2d_heavy_rope` for the
    `avbd-demo2d` Heavy Rope source row, plus `avbd_demo2d_net` for the
    `avbd-demo2d` Net source row, plus `avbd_demo3d_ground`,
    `avbd_demo3d_dynamic_friction`,
    `avbd_demo3d_static_friction`, `avbd_demo3d_pyramid`,
    `avbd_demo3d_rope`, `avbd_demo3d_heavy_rope`,
    `avbd_demo3d_spring`, `avbd_demo3d_spring_ratio`,
    `avbd_demo3d_stack`, `avbd_demo3d_stack_ratio`,
    `avbd_demo3d_soft_body`,
    `avbd_demo3d_bridge`, and `avbd_demo3d_breakable` py-demos for the first
    non-empty `avbd-demo3d`
    source rows, each with reference revision
    and default-parameter
    metadata. The empty
    baseline is backed by a tracked visual/benchmark packet. The 2D Ground row
    has a tracked visual/DART-benchmark/native-reference timing packet that
    keeps the CPU-win gate explicit: after skipping static-only contact queries,
    no-op rigid dynamics stages, clean frame-cache graph execution, and the
    clean no-work default step pipeline with a cheap scratch reset, DART records
    the static Ground row about 1.51x faster than the native source runner on
    this host, closing that narrow source row. The Motor row
    has a tracked visual/DART-benchmark/native-reference timing packet that keeps
    the CPU-win gate explicit: after skipping contact queries for
    no-collision-geometry worlds in contact-stage prepare/execute, DART still
    records the public World Motor row about 6.18x slower than the native source
    runner on this host. The Hanging
    Rope row has a tracked visual/DART-benchmark/native-reference timing packet
    and records DART about 7.84x slower than the native source runner on this
    host. The 2D Spring and Spring Ratio rows have tracked
    visual/DART-benchmark/native-reference timing packets that record DART
    about 17.8x and 11.1x slower than their native source runners on this host,
    keeping those CPU-win gates open. The 3D Spring and Spring Ratio rows have
    tracked visual/DART-benchmark/native-reference timing packets that record
    DART about 3.17x and 5.78x slower than their native source runners on this
    host, keeping those CPU-win gates open. The Fracture row has a tracked
    visual/DART-benchmark/native-reference
    timing packet and records DART about 1.20x faster than the native source
    runner on this host after a refreshed same-source timing run. The
    rigid AVBD contact stage now also reuses its extracted point-joint input
    scratch, solve-row scratch, and per-body row-index scratch, and avoids
    copying already-predicted inertial targets in the World contact path. The
    rigid contact stage also no longer
    performs a duplicate prepare-time collision query just to reserve
    sequential-contact scratch; execute-time contact discovery remains the
    authoritative query.
    Collision queries now also suppress live public rigid-body joint endpoint
    pairs from reusable query-cache scratch, mirroring the source demo solver's
    constrained-pair broadphase rule while allowing broken AVBD joints to
    become collidable again; focused coverage verifies warmed filtered queries
    do not allocate. The Fracture source-row metadata and packet writer record
    the 10 joint-connected collision pairs, but the packet was not refreshed
    because diagnostic timing remained noisy. Known and unknown-index rigid
    contact snapshots now map the world contact point through body and shape
    transforms before feature coding, while explicit endpoint-A/B compound
    shape-index feature coding now uses the narrow-phase shape-local contact
    point. Focused coverage now also verifies actual `World::collide()`
    sphere/cylinder/capsule/plane/mesh contacts feed their narrow-phase
    endpoints into AVBD feature coding, including shape-scoped sphere body
    features. Unknown-index inference now also recognizes unique
    compound plane and triangle-mesh matches, assigning shape-scoped plane face
    IDs and mesh face/edge/vertex feature IDs while preserving the body fallback
    for ambiguous compounds; same-feature manifold row ordinals are now assigned
    from deterministic canonical-local contact-point ordering so reversed
    narrow-phase contact order or swapped contact body endpoints do not swap
    warm-start state, with actual `World::collide()` sphere/plane contact-point
    replay and live box-box manifold box-face feature coverage. Persisting
    private rigid contact friction rows now also store their tangent directions
    and project the warm-started paired dual into the current tangent basis when
    the contact normal rotates. The `avbd-demo2d`
    Dynamic Friction row has a tracked
    visual/DART-benchmark/native-reference timing packet and records DART about
    1.83x faster than the native source runner on this host. The
    `avbd-demo2d` Static Friction row has a tracked
    visual/DART-benchmark/native-reference timing packet and records DART about
    2.68x faster than the native source runner on this host. The
    `avbd-demo2d` Pyramid row has a tracked
    visual/DART-benchmark/native-reference timing packet and records DART about
    9.84x faster than the 10,000-step native source runner on this host. The
    `avbd-demo2d` Cards row has a tracked
    visual/DART-benchmark/native-reference timing packet and records DART about
    5.38x slower than the native source runner on this host. The
    `avbd-demo2d` Stack row has a tracked
    visual/DART-benchmark/native-reference timing packet and records DART about
    2.17x faster than the native source runner on this host. The
    `avbd-demo2d` Stack Ratio row has a tracked
    visual/DART-benchmark/native-reference timing packet and records DART about
    2.22x faster than the native source runner on this host. The
    `avbd-demo2d` Rod row has a tracked
    visual/DART-benchmark/native-reference timing packet and records DART about
    9.58x slower than the native source runner on this host. The
    `avbd-demo2d` Soft Body row has a tracked
    visual/DART-benchmark/native-reference timing packet and records DART about
    6.30x slower than the native source runner on this host. The AVBD scalar
    row inventory now reuses stable same-order descriptor lists in place,
    avoiding the previous per-step previous-row map rebuild for unchanged row
    sets while preserving the existing fallback for dropped or reordered rows;
    rigid World contact snapshots also use reserved endpoint-pair hash counters
    for contact/joint/spring row ordinals instead of per-step tree maps, and
    append paths seed the body-index cache when they fall back to an existing
    snapshot entity scan. The rigid row driver skips per-body row-index scratch
    setup for row families absent from a solve.
    The
    `avbd-demo2d` Joint Grid row has a tracked
    visual/DART-benchmark/native-reference timing packet and records DART about
    2.06x slower than the native source runner on this host after reusing
    per-body row-index scratch and caching snapshot body indices, while
    configuring the source's 1152 diagonal ignore-collision pairs through the
    public per-pair collision filter. The
    `avbd-demo2d` Rope row has a tracked
    visual/DART-benchmark/native-reference timing packet and records DART about
    5.20x slower than the native source runner on this host. The
    `avbd-demo2d` Heavy Rope row has a tracked
    visual/DART-benchmark/native-reference timing packet and records DART about
    5.85x slower than the native source runner on this host after reusing
    per-body row-index scratch and caching snapshot body indices. The
    `avbd-demo3d`
    Ground
    row has a tracked visual/DART-benchmark/native-reference timing packet and
    records DART about 1.11x faster than the native source runner on this host,
    the Dynamic Friction row records DART about 1.41x faster, while the
    Static Friction row records DART about 1.08x faster, the Pyramid row records
    DART about 2.83x faster, the Rope row records DART about 3.75x slower, the
    Heavy Rope row records DART about 3.84x slower, the
    Stack row records DART about 1.80x faster, the
    Stack Ratio row records DART about 2.32x faster, the Soft Body row records
    DART about 2.33x slower, the Bridge row records DART about 1.61x faster,
    and the Breakable row records DART about 1.42x faster.
    The dashboard's AVBD World slice now also tracks an empty-baseline step row,
    an `avbd-demo2d` Motor source-row step benchmark,
    an `avbd-demo2d` Ground source-row step benchmark,
    an `avbd-demo2d` Hanging Rope source-row step benchmark,
    an `avbd-demo2d` Fracture source-row step benchmark,
    an `avbd-demo2d` Dynamic Friction source-row step benchmark,
    an `avbd-demo2d` Static Friction source-row step benchmark,
    an `avbd-demo2d` Pyramid source-row step benchmark,
    an `avbd-demo2d` Cards source-row step benchmark,
    an `avbd-demo2d` Stack source-row step benchmark,
    an `avbd-demo2d` Stack Ratio source-row step benchmark,
    an `avbd-demo2d` Rod source-row step benchmark,
    an `avbd-demo2d` Soft Body source-row step benchmark,
    an `avbd-demo2d` Joint Grid source-row step benchmark,
    an `avbd-demo2d` Rope source-row step benchmark,
    an `avbd-demo2d` Heavy Rope source-row step benchmark,
    an `avbd-demo3d` Ground source-row step benchmark,
    an `avbd-demo3d` Dynamic Friction source-row step benchmark,
    an `avbd-demo3d` Static Friction source-row step benchmark,
    an `avbd-demo3d` Pyramid source-row step benchmark,
    an `avbd-demo3d` Rope source-row step benchmark,
    an `avbd-demo3d` Heavy Rope source-row step benchmark,
    an `avbd-demo3d` Stack source-row step benchmark,
    an `avbd-demo3d` Stack Ratio source-row step benchmark,
    an `avbd-demo3d` Soft Body source-row step benchmark,
    an `avbd-demo3d` Bridge source-row step benchmark,
    an `avbd-demo3d` Breakable source-row step benchmark,
    public articulated
    revolute and prismatic motor step benchmark rows, plus public
    free-rigid/articulated breakable fixed point-joint benchmark rows, and adds a
    durable AVBD demo/benchmark corpus matrix that maps the required 2D/3D
    source-demo, paper, website/video, and performance rows to missing or partial
    DART evidence.
  - Made `dart::gui` UI scaling DPI-aware: `--gui-scale` now acts as a manual
    user multiplier on top of GLFW content-scale detection, `DART_GUI_DPI_SCALE`
    can override misreported DPI, implicit interactive app windows now use a
    larger monitor-aware automatic default and grow with the manual user
    multiplier and DPI scale, automatic windows resize when GLFW reports a DPI
    scale change, and explicit/headless render or screenshot dimensions remain
    fixed.
  - Sped up per-frame scene extraction with a `dart::gui::RenderableExtractor`
    that caches each shape's geometry by shape version, so `describeShape` is not
    rebuilt every frame for static geometry; soft meshes, point clouds, and voxel
    grids are still rebuilt every frame so deforming/live geometry stays correct.
  - Added a renderer-neutral `dart::gui::makeDeformableSurfaceRenderable()`
    helper for dynamic deformable surface meshes, including per-frame normals,
    bounds, material options, and stable resource versioning for Filament-backed
    captures.
  - Fixed `LineSegmentShape` mutation versioning so animated line geometry is
    rebuilt correctly by cached GUI renderable extraction.
  - Added a renderer-neutral `dart::gui` panel callback surface for examples
    that need custom controls without including backend UI headers.
  - Added a reusable renderer-neutral `PanelBuilder.timeline(...)` control and
    made the Python demos default to a replay-timeline scene with a
    bottom-docked scrubber. The shared demos GUI now keeps bottom scene panels
    visible by default, uses clearer `Rebuild`/`Restart`/`Capture` toolbar
    labels, and exposes a bounded captured-frame timeline when recording
    viewer frames. Python demos backed by an experimental `World` now also get
    a shared bottom-docked `Replay` panel: `Save replay` records bounded World
    state snapshots during normal playback, optional scene replay-state hooks
    capture small Python controller state beside the World frames, and the same
    timeline widget can scrub or play those saved states without re-running
    physics.
  - Added `pixi run bm-allocator-comparative-check`, a strict allocator
    benchmark gate that compares DART allocator workloads against
    foonathan/memory and `std::pmr` baselines.
  - Added required comparative rows for foonathan/memory static fixed-storage
    stacks, scoped temporary allocators, and two-iteration frame allocators,
    with matching DART frame-allocator fast-path rows and
    `std::pmr::monotonic_buffer_resource` baselines.
  - Added required comparative rows for foonathan/memory raw heap/malloc/new
    allocators plus aligned, fallback, segregator, tracked, and deeply tracked
    allocator adapters, with matching DART frame/pool HMM rows and standard
    allocator baselines.
  - Extended the comparative allocator benchmarks with opt-in allocator-aware
    EnTT registry/component storage churn against foonathan/memory and
    standard-registry baselines, and added a CV/noise guard so strict allocator
    comparisons do not treat noisy benchmark rows as evidence. Added a focused
    EnTT-registry-only checker mode for allocator-policy optimization loops,
    benchmark counters that fail the DART EnTT row if reserved churn calls the
    configured allocator after prewarm, and separate EnTT build/growth rows for
    bake-time registry storage allocation. Added `FrameStlAllocator` no-growth
    EnTT coverage for world-lifetime arena policy exploration, while keeping
    EnTT registry benchmark rows disabled when the benchmark target is
    configured without an existing EnTT target. The EnTT registry benchmark rows
    now discover installed EnTT package metadata without invoking DART's
    FetchContent-backed dependency helper. The EnTT rows now benchmark cached
    component storage handles, use free-list-backed DART storage for persistent
    no-growth registry churn, and use a resettable frame-backed DART bake arena
    for one-shot registry build/growth storage construction. The foonathan
    build/growth baseline uses stack marker/unwind bulk lifetime storage,
    matching the one-shot construction role rather than persistent pool reuse.
    The no-growth row reports allocator-call counters and fails if
    reserved churn grows after prewarm. The build/growth row times the
    uninstrumented construction/destruction path. The focused checker now
    distinguishes timing loss, allocator-call regressions, frame growth, and
    noisy benchmark evidence. The checker now also requires the expected
    benchmark rows for the selected mode, so missing foonathan/memory coverage
    cannot be treated as passing evidence.
  - Optimized the frame allocator reset/accounting fast path, default 32-byte
    frame allocations, and cache-line-aligned `FrameStlAllocator` allocations
    while preserving over-aligned STL storage; frame-backed STL blocks are now
    cache-line aligned for container hot loops such as allocator-aware EnTT
    storage pages, and the comparative STL-vector row now batches reserve-only
    allocator-adapter work to measure allocator throughput directly.
  - Added an explicit `PoolAllocator::DiagnosticsPolicy` so hot-path pool
    allocation can opt out of live/peak counter updates while preserving
    counter behavior for existing direct `PoolAllocator` users by default.
    Release `MemoryManager` pool allocation now uses the non-diagnostic policy,
    and comparative DART pool workloads use that HMM hot path.
  - Added optional CPU affinity, warmup, and random-interleaving controls to the
    C++ benchmark runner and comparative allocator checker; the realistic
    comparative allocator row now lets the checker control benchmark min-time,
    and short stack/tracked-stack rows batch repeated allocator cycles for
    stricter low-noise comparisons.
  - Prewarmed the benchmark runner's selected CPU immediately before launching
    an affinity-pinned C++ benchmark binary, reducing cold-frequency bias from
    configure/build work that happens before benchmark execution.
  - Kept `StlAllocator` allocation and deallocation alignment-aware for
    allocator-backed STL storage, including fixed-pool-backed max-aligned
    values and cache-line-aligned large storage pages for allocator-aware
    container hot loops such as the production World registry.
  - Made `StlAllocator` a lightweight stateful allocator instead of deriving
    from `std::allocator`, matching foonathan-style container propagation
    traits and letting `std::allocator_traits` handle object
    construction/destruction for allocator-aware container hot loops.
  - Added `DefaultStlAllocator`, a stateless STL adapter backed by DART's
    default C heap, so allocator-aware containers that use the process-wide
    default C allocator resource avoid per-container allocator state and the
    virtual default-allocator hop.
  - Added explicit STL allocator construct/destroy hooks that keep non-trivial
    object lifetimes correct while avoiding unnecessary trivial-destructor work
    in allocator-aware container hot loops.
  - Refined the comparative allocator checker so high-CV rows can pass only
    when the benchmark mean/stddev/repetition aggregates still show DART's
    normal-approximation 95% confidence interval strictly below the selected
    baseline's confidence interval.
  - Applied the same lightweight stateful allocator traits to
    `FrameStlAllocator`, letting `std::allocator_traits` own construction and
    destruction in frame-backed allocator-aware container builds.
  - Added a fixed-capacity growth policy to `FreeListAllocator` so
    preallocated free-list arenas can fail deterministically instead of growing
    from the base allocator after bake/build, and exposed construction-time
    free-list capacity/policy knobs through `MemoryManager` and experimental
    `WorldOptions`.
  - Added cache-line coloring for large 64-byte-aligned
    `FreeListAllocator` array allocations so consecutive page-sized
    component/storage arrays avoid starting on the same L1 cache sets while
    preserving their requested alignment.
  - Extended shape-aware cache-line coloring to large 64-byte-aligned
    `FrameAllocator` and frame-backed STL storage allocations, improving
    one-shot component storage build/unwind locality without changing arena
    lifetime semantics or padding smaller packed entity pages.
  - Switched the EnTT build/growth comparative benchmark to DART's
    frame-native `FrameStlAllocator`, matching foonathan's stack-native
    `std_allocator` adapter for one-shot arena lifetimes.
  - Tightened `FrameStlAllocator` small natural-aligned allocations to use the
    frame allocator's inline default fast path directly instead of routing
    through the public aligned-allocation dispatcher.
  - Fixed fixed-capacity `FreeListAllocator` aligned allocations so
    `PoolAllocator` can satisfy aligned size-class chunks from reserved arena
    bytes without growing from the base allocator.
  - Routed expand-policy `FreeListAllocator` over-aligned allocations through
    reserved free-list arena blocks instead of delegating them to the base
    allocator.
  - Hardened `FreeListAllocator` reservation arithmetic so impossible fixed
    capacities and expansion sizes fail before touching the base allocator.
  - Hardened common allocator count and size overflow guards for
    `MemoryAllocator::allocateAs`, `FrameAllocator`, `FrameStlAllocator`, and
    `StlAllocator`; fixed fixed-capacity `FreeListAllocator` aligned-allocation
    diagnostics so public live/peak counters track requested bytes instead of
    internal header padding, and added allocator-root isolation coverage for
    independent `MemoryManager` and experimental `World` instances.
  - Added query methods on `dart::common::MemoryAllocatorDebugger`,
    `FreeListAllocator`, and `PoolAllocator` for current live bytes, peak live
    bytes, and live allocation count so allocator diagnostics can consume
    structured counters instead of parsing debug text.
  - Added structured `MemoryManager` debug diagnostics and surfaced them through
    experimental `World` memory diagnostics for free/pool allocator accounting,
    including typed borrowed allocator use.
  - Added a read-only Memory panel to the standalone `dartsim` editor for
    frame-scratch counters, `MemoryManager` debug counters, and ECS storage
    capacity diagnostics.
  - Made experimental `World::clear()` recreate its internal allocator-backed
    registry storage so ECS capacities and debug-tracked registry allocations
    are released at the rebuild boundary while preserving the World memory
    hierarchy.
  - Reused legacy graph-backed `RigidBodyIntegrationStage` scratch for rigid-body
    entity lists and dependency nodes instead of allocating per execute.
  - Extended experimental `World` base-allocator no-growth coverage to baked
    rigid-body resting contact, non-cross articulated resting contact, and
    same-DOF cross-articulated link-contact scenes after contact prewarm.
  - Removed per-assembly heap containers from the experimental unified
    constraint assembler for row directions, rigid/articulated row ends, and
    shared rigid-body inertia lookup, and routed the boxed-LCP stage's
    per-multibody link-contact row assembly through reusable
    `MultibodyDynamicsScratch`. Cross-multibody row completion now reuses that
    scratch for other-link point Jacobians and joint-space denominator work.
    The Dantzig boxed-LCP solver now accepts caller-owned reusable scratch,
    avoids `LcpProblem` input copies for already assembled systems, and is
    routed through reusable unified-constraint solve scratch for island
    remapping, island sub-problems, normal-only fallback, and fallback tangent
    accumulators. Same-shape no-heap regressions cover both Dantzig solves and
    unified island solves. The unified assembler now reuses same-shape link
    block row storage without per-row Eigen temporaries, and the boxed-LCP
    stage borrows per-multibody contact problems from persistent dynamics
    scratch instead of copying them into staging containers first. Rigid
    contact problem assembly now has an in-place scratch overload, and boxed-LCP
    fallback world gates cover mixed/different-DOF, stacked, and coupled
    multi-row cross-articulated contact scenes for base-allocator no-growth and
    first baked-step global-heap no-allocation by priming unified constraint
    scratch during `enterSimulationMode()`. Public multibody link-contact
    assembly now has reusable scratch storage that can be borrowed by the
    in-place unified assembler without same-shape heap growth, and the
    boxed-LCP fallback gates include larger five- and eight-multibody stacked
    contact sets plus a disconnected multi-island mixed rigid/articulated
    contact set. Unified island solves now reserve island traversal scratch and
    build the local row remap once per solve instead of once per island, and
    successful unified link impulse application can reuse the same solve scratch
    for generalized-impulse and velocity-delta buffers.
    Rigid IPC accepted/rejected writeback now reuses stage-owned entity-order
    scratch instead of allocating local traversal vectors, and the
    resting-contact no-op predicate reuses stage-owned per-body contact-power
    and stationary-flag scratch. The rigid IPC projected-Newton loop now reuses
    solve-local surface buffers across line-search and sufficient-decrease
    backtracking candidates, and its repeated solve-internal barrier assembly
    and line-search calls reuse surface-pair/triplet scratch, including the
    lagged-friction barrier pass. The rigid IPC contact stage now uses the
    caller-owned projected-Newton solve overload so per-solve surface candidate
    buffers persist in stage scratch across steps.
    Convenience return-by-value unified problem wrappers remain a separate
    allocation target.
  - Reused `DeformableDynamicsStage` scratch for deformable surface snapshots,
    static/moving rigid surface-CCD snapshots, and rigid obstacle barrier lists,
    and primed deformable surface-contact candidate buffers during
    `enterSimulationMode()`, including the wider swept-AABB candidate envelope
    needed by non-square self-contact grids. Inter-body/rigid surface-CCD sweep
    buffers are now stage/entity scratch-backed too, and VBD
    topology/static-contact scratch is primed during `enterSimulationMode()`, so
    baked surface-snapshot steps and first-baked-step active VBD static rigid
    surface-CCD point crossing do not allocate from the global heap. Scripted
    deformable boundary processing now
    reuses per-body Dirichlet/Neumann count masks instead of allocating local
    per-node vectors each step. Default projected-Newton deformable solves now
    reuse per-body RHS, sparse Hessian assembly, PSD block batch, sparse
    pattern, and solution scratch for covered mass-spring and static rigid
    surface-CCD steps, and FEM rest-shape caches are primed for covered
    one-tetrahedron FEM projected-Newton steps. Self-contact barrier scratch is
    sized from bake-primed candidates for covered two-triangle
    projected-Newton self-contact steps. AVBD ground contact/friction rows and
    self-contact normal/friction rows, including a rectangular self-contact
    grid row workload, now reuse row-inventory and self-contact adjacency
    storage, with bake-time reserve sizing for covered active contact steps;
    row and friction-projection warm-start lookup now use sorted reserved
    storage instead of map allocation or per-row linear scans. Rigid
    AVBD contact projection now reuses stage-owned snapshot, point-joint,
    row-counter, row-inventory, and solve scratch for covered active rigid
    contacts and no-contact fixed-joint rows, with baked base-allocator and
    global-heap no-growth guards. Active inter-body deformable surface-CCD
    crossings and two non-square production-scale deformable frictional
    self-contact grids now have the same baked no-growth guard coverage.
  - Hardened `dart::common::FixedPoolAllocator` against base-allocator failures
    during construction and block-table growth, with coverage for deterministic
    failure, fallback, reuse, and debug-guard paths.
  - Added the standalone `dartsim/` GUI simulator (a runtime executable, not a
    library) built only on the experimental World API. Its headless editor
    engine (`dartsim/engine`) provides scene/object, selection, command
    (undo/redo, grouped macros), and name managers, a typed event bus, an
    in-engine logger, an Edit/Run simulation controller, binary record/replay,
    and a human-readable project format, covered by `UNIT_dartsim_engine` unit
    tests. The `dartsim/ui` panel layer wires the engine to the backend-hidden
    `dart-gui` editor (menu bar, Scene Tree, Inspector, Console, simulation
    controls, replay timeline), and a public additive
    `dart::gui::ApplicationOptions::renderableProvider` hook renders the
    experimental scene in the viewport without exposing renderer types. The
    editor runs as an ImGui docking workspace via `pixi run dartsim`, which
    builds against the ImGui docking branch (`DART_USE_SYSTEM_IMGUI=OFF`) and
    opens with an IDE-style default layout (driven by a per-panel
    `dart::gui::DockSide` hint); docking is guarded by `IMGUI_HAS_DOCK`, so
    non-docking ImGui builds are unaffected. See
    `docs/design/dartsim_gui_simulator.md` and
    `docs/onboarding/gui-rendering.md`.
  - Moved the `dartsim` viewer source to a dedicated application tree and added
    a public `dart::gui::ApplicationOptions::world` handoff for example-owned
    worlds.
  - Restored `boxes`, `rigid_cubes`, and `box_stacking` as real public-API
    `dart::gui` example sources with source-defined DART worlds instead of
    shared private scene-fixture launchers.
  - Restored the `boxes` example's historical box labels, light-gray ground,
    instruction text, standalone README build/execute instructions, and marker
    coverage through public `dart::gui`.
  - Restored the `box_stacking` example's historical stack placement, random
    box colors, floor geometry, panel labels/help text, standalone README
    build/execute instructions, and marker coverage through public
    `dart::gui`.
  - Restored `simple_frames` and `capsule_ground_contact` as public-API
    `dart::gui` example sources using source-defined DART frame and contact
    worlds.
  - Restored the `simple_frames` example's historical frame names, marker
    names, ArrowShape marker, standalone README build/execute instructions, and
    marker coverage through public `dart::gui`.
  - Stopped generated in-memory mesh shapes from attempting material loads from
    empty file URIs, allowing examples such as `simple_frames` to use
    `ArrowShape` without renderer startup warnings.
  - Added promoted panel window/menu/collapsible helpers and used them to
    restore the Fetch example panel geometry, menu bar, scrollbar, Help
    section, and default-open Simulation controls.
  - Restored the `g1_puppet` example's historical gravity, ground, root pose,
    XY grid, support-polygon overlay, camera/run defaults, standalone README,
    and IK target activation messages through public `dart::gui`.
  - Restored the `hubo_puppet` example's historical ground, target activation
    toggles, support toggles, posture reset, DOF print action,
    support-polygon overlay, camera/run defaults, and standalone README through
    public `dart::gui`.
  - Restored the `capsule_ground_contact` example's pose-reset keyboard
    controls, velocity-clear action, camera/run defaults, persistent-manifold
    instructions, and README through public `dart::gui`.
  - Restored the `rigid_chain` example's historical random initial pose,
    camera/run defaults, README, and marker coverage while keeping its damping
    callback on public `dart::gui`.
  - Restored the `rigid_loop` example's constrained-link colors, console
    instructions, run defaults, README, and marker coverage while keeping its
    damping callback on public `dart::gui`.
  - Restored the `rigid_cubes` example's shipped `cubes.skel` world, Y-down
    gravity, decaying directional force keys, camera/run defaults, README, and
    marker coverage through public `dart::gui`.
  - Added a public `dart::gui::ApplicationOptions::preStep` lifecycle callback
    and restored `fetch` as a source-defined `dart::gui` example with its live
    mocap target-following behavior, Bullet preference when available, and
    public-API target affordance.
  - Replaced OctoMap-backed `VoxelGridShape` storage with native
    `SparseOccupancyGrid` storage. OctoMap is no longer discovered or linked by
    core DART libraries and remains available only to tests and benchmarks for
    reference comparisons.
  - Added `dart::math::GroupProduct` for typed direct products of Lie groups
    and additive `dart::math::Rn` factors for Euclidean joint coordinates,
    including componentwise composition, inverse expressions, Eigen map support,
    block-diagonal matrix views, and focused lie-group unit coverage.
  - Fixed Filament mesh material extraction so legacy textured meshes without
    PBR metalness import as dielectric surfaces, restoring visible Atlas
    texture detail in `dartsim`, `atlas_puppet`, and `atlas_simbicon`.
  - Fixed Atlas/whole-body puppet target dragging to solve through the owning
    skeleton IK hierarchy with damped least-squares, avoiding regressions where
    active target drags moved end-effectors away from the target.
  - Fixed `dart::gui` IK handle solving to distinguish local target handles
    from whole-body puppet handles and restored Atlas hand root-gradient
    weighting, keeping G1/simple IK interaction local while Atlas and Hubo
    puppet targets use the skeleton hierarchy consistently.
  - Restored the `fetch` example's historical pick-and-place work-area grid as
    source-owned public DART line geometry at the original grid offset.
  - Restored the `fetch` example panel title and instructional copy against
    the historical example source.
  - Restored the `fetch` example's target cue as a source-owned mesh cross made
    from two transparent green bars, matching the historical end-effector
    affordance, while moving mouse translation/rotation to a public
    `dart::gui::Gizmo` and retaining reset controls and image-sequence capture
    docs through promoted `dart::gui`.
  - Restored the `biped_stand` example's historical space/start and push
    instruction text plus standalone README build/execute instructions.
  - Restored the rigid/constraint example batch (`rigid_chain`, `rigid_loop`,
    `mixed_chain`, `coupler_constraint`, `add_delete_skels`, and
    `rigid_shapes`) as source-defined `dart::gui` examples.
  - Restored the `mixed_chain` example's random startup pose, 640x480 launch
    default, console instructions, README, and marker coverage through public
    `dart::gui`.
  - Restored the `rigid_shapes` example's historical `shapes.skel` startup
    scene, random spawn behavior, console instructions, collision-detector
    printout, README, and marker coverage through public `dart::gui`.
  - Restored the `soft_bodies` example's historical 640x480 launch default,
    console playback instructions, README, and marker coverage through public
    `dart::gui`.
  - Restored the `tinkertoy` example's historical panel layout, selected
    pick-point behavior, explicit 1280x720 launch default, README, and marker
    coverage through public `dart::gui`.
  - Restored the `add_delete_skels` example's README, historical 640x480
    launch default, and spawn-only-on-command startup behavior through public
    `dart::gui`.
  - Restored the `coupler_constraint` example's controller-driven mimic
    comparison, reset key, live diagnostics, grid, camera/run defaults, README,
    and marker coverage through public `dart::gui`.
  - Restored the joint/dynamics example batch (`hybrid_dynamics`,
    `biped_stand`, `joint_constraints`, `free_joint_cases`, and
    `human_joint_limits`) as source-defined `dart::gui` examples.
  - Added public `dart::gui::OrbitCamera` up-vector control and restored
    historical camera roll/up-vector defaults across migrated GUI examples.
  - Added public `dart::gui::PanelContext` camera inspection state and restored
    Eye/Center/Up panel readouts in migrated GUI examples.
  - Added public key-release triggers for `dart::gui::KeyboardAction` and
    restored historical keydown/key-release callbacks in migrated GUI examples.
  - Added shifted-slash shortcut handling for `dart::gui::KeyboardAction` and
    restored the Simulation Event Handler `?` help alias.
  - Added public `dart::gui::PanelContext` lighting state and restored
    historical headlight panel checkboxes in migrated GUI examples.
  - Added public `dart::gui::ApplicationOptions` post-step and render
    callbacks and restored historical lifecycle-hook demos in migrated GUI
    examples.
  - Added public `dart::gui::RenderSettings` shadow control and restored
    historical shadow toggles in migrated GUI examples.
  - Added public `dart::gui::PanelBuilder` color-edit controls and restored
    terrain, point-cloud, and voxel-grid color editors in migrated GUI
    examples.
  - Added public `dart::gui::PanelBuilder` line plotting and restored the
    `lcp_physics` step-time history plot.
  - Added public `dart::gui::PanelContext` UI metrics and restored the
    `lcp_physics` display/font debug diagnostics.
  - Added public runtime frame-output capture controls and restored the
    Tinkertoy example's Enter-key recording workflow.
  - Fixed the dartpy bindings for GUI frame-output capture helpers after
    viewer-lifecycle overloads were added.
  - Added public `dart::gui::PanelBuilder` modal helpers and restored
    Tinkertoy's About DART modal workflow.
  - Added public `dart::gui::ApplicationOptions::simulateWorld` and restored
    the WAM IKFast example's kinematic no-world-step workflow.
  - Added public `dart::gui::RenderOutputMode::Depth` and restored the Atlas
    Simbicon example's depth output toggle and capture path.
  - Fixed Atlas GUI example readability and up-axis consistency by keeping
    Atlas Simbicon z-up, lifting overly dark Atlas mesh visuals, and widening
    Filament shadow coverage for large orbit-camera views.
  - Added public `dart::gui::BodyNodeDragHandle` and restored the G1 Puppet and
    WAM IKFast examples' articulated body-node drag workflows.
  - Restored the Heightmap and Point Cloud examples' fine-grained scene-grid
    controls with source-owned DART line geometry.
  - Restored the `hubo_puppet` support overlay's blue/red center-of-mass
    validity marker with source-owned DART line geometry.
  - Restored the `hubo_puppet` example's relaxed-posture objective, balance
    constraint, whole-body IK solve path, and hold/release R balance controls.
  - Restored the `hubo_puppet` example's source-owned analytical arm and leg IK
    methods.
  - Restored the `hubo_puppet` example's historical Shift-amplified root
    movement controls.
  - Restored the `atlas_puppet` example's number-key target activation,
    support toggles, DOF print/reset shortcuts, and source-owned support
    polygon/center-of-mass validity overlays.
  - Restored the `atlas_puppet` example's relaxed-posture objective, balance
    constraint, whole-body IK solve path, and hold/release R balance controls.
  - Added public `dart::gui::PanelBuilder` table and color-swatch controls and
    restored the Mimic Pendulums diagnostics table and rig legend.
  - Restored the `hardcoded_design` example's historical wireframe link
    appearance with source-owned DART line geometry.
  - Restored the `atlas_simbicon` example's Simbicon controller/state files,
    pre-step control loop, perturbation and state-machine keyboard actions,
    gravity/harness/stride panel controls, native window title,
    camera/run defaults, and README through public `dart::gui`.
  - Audited the `gui_scene_diagnostics` source before retiring it from `main`
    with the rest of the classic standalone GUI example surface.
  - Removed the no-source `rerun` placeholder example because it had no
    concrete integration workflow, executable, or known downstream user.
  - Restored the `lcp_physics` example's historical scene counts, names,
    placement, live scenario/solver switching, reset/timestep/gravity panel
    controls, README option inventory, and marker coverage through public
    `dart::gui`.
  - Restored the `lcp_physics` panel's render FPS, rendered/skipped frame
    counts, and rolling step-time diagnostics using source-owned metrics.
  - Restored the `human_joint_limits` example's live SKEL world, joint-limit
    enforcement, neural custom arm/leg constraints, run defaults, README, and
    marker coverage through public `dart::gui`.
  - Moved `drag_and_drop` off the private named-scene fixture path so the
    example source owns its DART world and passes it through
    `dart::gui::ApplicationOptions::world`.
  - Restored the `drag_and_drop` example's public transform gizmo, historical
    marker layout, camera/run defaults, README, and marker coverage through
    public `dart::gui`.
  - Moved `imgui` and `tinkertoy` off private named-scene fixture defaults so
    their example sources own their DART worlds through
    `dart::gui::ApplicationOptions::world`.
  - Restored the `imgui` panel-extension example's target frame with a public
    `dart::gui::Gizmo`, promoted keydown callbacks, gravity control, viewer
    help, camera/run defaults, README, and marker coverage through public
    `dart::gui`, with later public APIs covering the headlight,
    camera-inspector, key-release, and render-hook parity paths.
  - Moved `operational_space_control` and `wam_ikfast` off private named-scene
    fixture launchers so their example sources load the WAM robot, create their
    visible targets, and run through promoted `dart::gui` options directly.
  - Restored the `operational_space_control` example's historical KR5 robot,
    KR5 ground, target ball, task-space controller, 640x480 launch default,
    camera framing, control instructions, README, and marker coverage through
    public `dart::gui`.
  - Restored the `wam_ikfast` example's generated IKFast shared library,
    `SharedLibraryIkFast` setup, target activation workflow, relaxed-posture
    reset, joint-value printing, 1280x960 launch default, camera framing,
    README, and marker coverage through public `dart::gui`.
  - Moved `atlas_simbicon` off the private named-scene fixture launcher so the
    example source loads Atlas, creates its ground, and runs through promoted
    `dart::gui` options directly.
  - Added a promoted `dart::gui::ApplicationOptions` IK-handle handoff so
    source-owned examples can preserve target hotkeys, selection labels, and
    solve-on-drag behavior without depending on private renderer fixtures.
  - Added a promoted `dart::gui::Gizmo` registration surface with debug-line
    visualization, X/Y/Z axis-handle dragging, plane-handle dragging,
    rotation-ring dragging, and per-target visibility, plus hover/active handle
    highlighting for source-owned transform target affordances. Gizmo-only
    scenes can start without dummy renderable geometry, and `fetch`,
    `drag_and_drop`, `atlas_puppet`, `hubo_puppet`, `g1_puppet`,
    `operational_space_control`, `wam_ikfast`, and `imgui` now use those public
    gizmos instead of source-owned frame-handle, target-handle, or target-ball
    geometry; Tinkertoy uses the same public gizmo for its force target while
    keeping its source-owned axes and force-line geometry.
  - Improved Filament GUI puppet interaction and rendering parity with
    OSG-style active IK target gizmos, patterned filled rotation rings,
    non-overlapping default ImGui panels, close-camera clipping tolerance, and
    faithful textured mesh colors.
  - Moved `atlas_puppet` off the private named-scene fixture launcher so the
    example source loads Atlas, creates its ground/root handle/IK targets, and
    passes public `dart::gui` IK handles directly.
  - Restored the `atlas_puppet` example's README, historical 1280x960 launch
    default, and camera home while keeping target activation, posture/balance,
    and support visualization gaps explicit.
  - Moved `hubo_puppet` and `g1_puppet` off private named-scene fixture
    launchers so their example sources load the robots, create visible IK
    targets, and pass public `dart::gui` IK handles directly.
  - Moved the static geometry examples (`hardcoded_design`, `heightmap`,
    `point_cloud`, and `polyhedron_visual`) off private named-scene fixture
    launchers so their example sources own the visual DART worlds directly.
  - Restored the `heightmap` example's local demo selector, interactive
    regeneration panel, alignment scene, camera/run defaults, and README
    through public `dart::gui`.
  - Restored the `point_cloud` example's KR5 robot sampling workflow,
    point-cloud and voxel-grid updates, sensor visualization, panel controls,
    camera/run defaults, scene grid, and README through public `dart::gui`.
  - Restored the `polyhedron_visual` example's scene grid, camera/run
    defaults, and README through public `dart::gui`.
  - Moved the LCP and mimic dynamics examples (`lcp_physics` and
    `mimic_pendulums`) off private named-scene fixture launchers so their
    example sources own solver setup, imported worlds, and public GUI panels.
  - Restored the `mimic_pendulums` example's live reset/collision/solver
    controls, XY reference grid, original rig names/base colors, diagnostics,
    README options, and marker coverage through public `dart::gui`.
  - Moved the interaction/event examples (`empty` and
    `simulation_event_handler`) off private named-scene fixture launchers so
    their example sources own public frames, sensors, and panels directly.
  - Restored the `empty` example's public keyboard-action scaffold,
    camera/run defaults, and README through public `dart::gui`.
  - Restored the `simulation_event_handler` example's selected-body
    force/torque controls, force-arrow visualization, camera/run defaults, and
    README through public `dart::gui`.
  - Moved the soft-body and vehicle examples (`soft_bodies` and `vehicle`) off
    private named-scene fixture launchers so their example sources own imported
    worlds, public panels, and simulation controls directly.
  - Removed the unused GUI example scene-launcher shim so restored examples
    must build real source files instead of private named-scene fixtures.
  - Promoted the GUI extraction/rendering implementation into the official
    `dart-gui-core` target and removed the old experimental C++ and Python
    compatibility surfaces.
  - Added a renderer-neutral `dart::gui::ApplicationOptions::camera` hook and
    used it to restore the Fetch example's historical camera framing while
    keeping the example on public `dart::gui` APIs.
  - Added renderer-neutral `dart::gui::ApplicationOptions::runDefaults`,
    restored the Fetch example's historical 1280x960 default launch size, and
    restored its example README.
  - Restored visible IK target controls and promoted control text for the G1,
    Atlas, and Hubo puppet examples.
  - Restored visible target controls and promoted control text for the
    operational-space control, WAM IKFast, and Tinkertoy examples.
  - Restored Tinkertoy builder state through public `dart::gui` panel controls,
    including selected-block add/delete actions, gravity and force-coefficient
    controls, public-gizmo target movement, target reorientation, force-line
    updates, external force application, and collision/dynamics block
    construction.
  - Added renderer-neutral `dart::gui` keyboard actions and used them to
    restore Tinkertoy add/delete, force coefficient, clear-pick, and target
    reorientation hotkeys without exposing backend input headers.
  - Restored Atlas and Hubo puppet continuous IK solving and WASD/QE/FZ root
    teleoperation through renderer-neutral `dart::gui` keyboard actions.
  - Added a renderer-neutral keyboard action camera reset callback and used it
    to restore Tinkertoy's Tab camera-home hotkey.
  - Restored G1 puppet number-key target activation/deactivation and
    active-target IK solving through public `dart::gui` keyboard actions.
  - Added a public `dart::gui` lifecycle exit request helper and used it to
    restore Fetch example panel Exit, Play/Pause, Help, and About affordances.
  - Restored add/delete skeleton example `q`/`w` keyboard controls, Bullet
    preference, and camera defaults through public `dart::gui`.
  - Restored mixed-chain example `q`/`w`, `e`/`r`, and `t`/`y` impulse
    keyboard controls and camera defaults through public `dart::gui`.
  - Restored vehicle example `w`/`s`/`x`/`a`/`d` command keys, camera
    defaults, 640x480 run default, console instructions, and README through
    public `dart::gui`.
  - Restored soft-bodies example playback keys for frame stepping, restart, and
    jump-to-latest through public `dart::gui` keyboard actions.
  - Restored hybrid-dynamics example `h` harness toggle, historical
    lock/unlock console messages, 640x480 launch default, loaded SKEL visuals,
    camera defaults, and README through public `dart::gui`.
  - Restored joint-constraints example perturbation keys, `h` harness toggle,
    historical console messages, 640x480 launch default, loaded SKEL visuals,
    README, and camera defaults through public `dart::gui`.
  - Restored rigid-shapes example spawn/delete/contact keyboard controls,
    convex mesh spawning, contact-point visualization, rigid-shapes-specific
    command-line options, and camera/run defaults through public `dart::gui`.
  - Restored biped-stand example perturbation keyboard controls, camera/run
    defaults, and example README through public `dart::gui`.
  - Restored free-joint-cases example numeric/reference controls, local
    command-line flags, camera/run defaults, and example README through public
    `dart::gui`.
  - Restored LCP physics example scenario and solver command-line selection,
    list mode, camera/run defaults, panel context, and example README through
    public `dart::gui`.
  - Restored mimic-pendulums example baseline retargeting, mimic diagnostics,
    solver/collision launch flags, camera/run defaults, and example README
    through public `dart::gui`.
  - Restored box-stacking example solver selection, split-impulse controls,
    camera/run defaults, and example README through public `dart::gui`.
  - Restored boxes example Bullet preference, camera/run defaults, and example
    README through public `dart::gui`.
  - Restored simple-frames example camera/run defaults and example README
    through public `dart::gui`.
  - Restored hardcoded-design example joint keyboard controls, camera default,
    and example README through public `dart::gui`.
  - Retired the `csv_logger`, `headless_simulation`, and `unified_loading`
    examples from `main` with release-6.\* branches as the parity source for
    their DART 6 whole-World loading behavior.
  - Removed dartpy legacy collision detector aliases `DARTCollisionDetector`,
    `FCLCollisionDetector`, `BulletCollisionDetector`, and
    `OdeCollisionDetector`; use `DartCollisionDetector` or the default detector.

- Minimum Compiler Requirements
  - Linux: GCC 11.0+
  - macOS: Clang 12.0+
  - Windows: MSVC 19.50+ (Visual Studio 2026)

- Tested Platforms
  - TBD (will be updated upon release)

- Build
  - Minimum C++ standard: C++20 (previously C++17). ([#2068](https://github.com/dartsim/dart/pull/2068))
  - Consolidated and modernized the CMake build system, including Eigen version enforcement, build-type defaults for multi-config generators, distro toggles, and corrected utils source registration. ([#2060](https://github.com/dartsim/dart/pull/2060), [#2107](https://github.com/dartsim/dart/pull/2107), [#2216](https://github.com/dartsim/dart/pull/2216), [#2177](https://github.com/dartsim/dart/pull/2177), [#2314](https://github.com/dartsim/dart/pull/2314))
  - Added `DART_BUILD_TESTS`, `DART_BUILD_EXAMPLES`, and `DART_BUILD_TUTORIALS` to allow optionally skipping the tests/examples/tutorial targets (examples/tutorials now auto-disable if `dart-gui` is not built).
  - `dart.pc` now reports the installed include directory via `Cflags`, improving downstream `pkg-config` usage without breaking relocatable installs.
  - Added `DART_EXAMPLES_INSTALL_PATH` CMake cache variable to customize where example sources are installed or disable their installation. ([#2100](https://github.com/dartsim/dart/pull/2100))
  - Added `libsdformat` as a required dependency so that SDF files are normalized through the official parser before being handed to DART: [#264](https://github.com/dartsim/dart/issues/264)
  - Introduced per-target export headers (`dart/<component>/Export.hpp`) that define `DART_<COMPONENT>_API` macros. Each library now controls symbol visibility independently on Windows instead of sharing the monolithic `DART_API`, which fixes long‑standing DLL import inconsistencies. ([#2163](https://github.com/dartsim/dart/pull/2163))
  - Packaging and distribution updates: migrated to `pyproject.toml`, switched dartpy wheel builds to pixi, added Windows wheel builds, standardized wheel cleanup, removed manylinux artifacts, and fixed the `package.xml` version. ([#2043](https://github.com/dartsim/dart/pull/2043), [#2072](https://github.com/dartsim/dart/pull/2072), [#2266](https://github.com/dartsim/dart/pull/2266), [#2268](https://github.com/dartsim/dart/pull/2268), [#2080](https://github.com/dartsim/dart/pull/2080), [#2207](https://github.com/dartsim/dart/pull/2207), [#2172](https://github.com/dartsim/dart/pull/2172))
  - Developer workflow updates: refactored pixi tasks, tuned pixi parallelism, simplified the devcontainer, and bumped the DART version to 7.0.0. ([#2083](https://github.com/dartsim/dart/pull/2083), [#2208](https://github.com/dartsim/dart/pull/2208), [#2255](https://github.com/dartsim/dart/pull/2255), [#2046](https://github.com/dartsim/dart/pull/2046))
  - Limited `pixi run test-unit` to the built C++ `UNIT_` test set so
    simulation-experimental tests remain covered by their dedicated task.
  - Fixed `pixi run test-math` so its build step uses the current math,
    optimization, LCP, and lie-group CMake test target names before running the
    `^UNIT_math_` ctest slice.
  - Added ASan build mode and example install destination controls. ([#2101](https://github.com/dartsim/dart/pull/2101), [#2100](https://github.com/dartsim/dart/pull/2100))
  - GUI dependency handling updates: switched ImGui to FetchContent, prefer local Vulkan loader, and removed bundled lodepng. ([#2056](https://github.com/dartsim/dart/pull/2056), [#2085](https://github.com/dartsim/dart/pull/2085), [#2051](https://github.com/dartsim/dart/pull/2051))
  - Hide fetched ImGui internal formatting helpers from shared library exports. ([#2671](https://github.com/dartsim/dart/issues/2671))
  - Pixi tasks and helper scripts now guard optional targets (dartpy, GUI examples) automatically, detect missing generator targets before invoking `cmake --build`, and expose `DART_BUILD_*_OVERRIDE` environment hooks so CI and local workflows can toggle bindings/apps without editing `pixi.toml`.
  - Ensured the aggregate CMake `tests` target builds simulation-experimental
    test executables when that component is enabled, so local and CI test runs
    do not execute stale or missing binaries.
  - Raised the Windows compiler floor to MSVC 19.50+ (Visual Studio 2026),
    pinned Windows CI and wheel builds to `windows-2025-vs2026`, and made
    `GroupProduct` static size sums robust on MSVC 19.51.
  - Exported `DynamicJointConstraint` and `JointConstraint` on Windows so constraint unit tests link successfully.
  - Exported soft contact constraints, DART collision helpers, `computeIntersection`, and IK property types on Windows to fix shared-library unit test linking. ([#2462](https://github.com/dartsim/dart/pull/2462))
  - Exported existing FCL, joint Coulomb friction, and MJCF detail parser declarations on Windows so shared-library consumers and tests can link the header-declared symbols consistently. ([#2648](https://github.com/dartsim/dart/pull/2648))
  - Exported `ZeroDofJoint` on Windows so shared-library consumers can link inherited zero-DoF joint symbols consistently.
  - Linked `dart-gui` to Foundation on Apple platforms so OpenSceneGraph's macOS resource-path initialization has the Objective-C Foundation runtime it requires.
  - Removed the Filament GUI smoke CI job's distro libc++ package dependency and made its example runner use Pixi-provided libc++/libc++abi libraries when available.
  - Limited Codecov CI uploads to the generated `coverage.info` report so the uploader does not rescan gcov files after the coverage task has already produced lcov output.
  - Included simulation-experimental and simulation detail implementation files
    in Codecov reports by removing stale Codecov path exclusions. ([#2835](https://github.com/dartsim/dart/pull/2835))
  - Added support for assimp 6.x while maintaining backward compatibility with assimp 5.x
  - Added opt-in CUDA smoke support for experimental simulation builds, including
    a gated CMake option, Pixi CUDA environment, private SoA integration test and
    benchmark coverage, and manual CUDA CI workflow.
  - Upgraded pinned build and runtime dependencies to current conda-forge
    releases via `pixi upgrade`/`pixi update` (notably Eigen 5, fmt 12,
    spdlog 1.17, Boost 1.91, urdfdom 5, and assimp 6), and migrated DART's
    `Eigen::JacobiSVD` usage to the Eigen 5 API that takes SVD computation
    options as template parameters
    (`Eigen::JacobiSVD<MatrixType, Eigen::ComputeFullV>`) instead of the
    deprecated runtime `compute()`/constructor arguments.

- Tooling and Docs
  - Added a tiered local verification pipeline with a systematic task-naming
    scheme (`verify-*`/`test-*`/`bench-*`, tiers `quick` < bare < `full`),
    load-aware build/test parallelism (ninja `-l`, ctest `--test-load`) shared
    across clones, parallel test execution with per-test timeouts and
    `--rerun-failed`, and opt-in `DART_USE_MOLD` (mold linker) and
    `DART_NORMALIZE_BUILD_PATHS` (cross-clone compiler-cache sharing). See
    `docs/design/local_verification_pipeline.md`.
  - Bumped developer tooling to current releases (clang-format 22, black 26)
    and reformatted the C++ and Python sources to match.
  - Added AI-native documentation architecture with AGENTS.md, module-specific guides, slash commands, and command sync automation. ([#2446](https://github.com/dartsim/dart/pull/2446), [#2447](https://github.com/dartsim/dart/pull/2447), [#2448](https://github.com/dartsim/dart/pull/2448), [#2449](https://github.com/dartsim/dart/pull/2449))
  - Added the shared `docs/ai/` agent entrypoint and tightened AI workflow verification, approval-boundary checks, and dev-task cleanup guidance. ([#2649](https://github.com/dartsim/dart/pull/2649))
  - Updated GUI onboarding, module agent docs, ReadTheDocs pages, examples, and tutorial indexes to identify Filament as the maintained renderer and the removed OSG/Raylib paths as unsupported.
  - Removed the OpenSceneGraph GUI implementation sources, Raylib experiment, legacy C++ and Python GUI examples/tutorials, legacy dartpy GUI bindings/stubs, and OSG/Raylib dependency discovery. Filament with GLFW3 and Dear ImGui is now the official renderer surface.
  - Added AI-infra principles, living plan dashboard, `dart-plan-update`, and `dart-retrospect` workflows to track research-focused roadmap work and durable session learnings from one source of truth, and aligned public and onboarding documentation entrypoints with that direction.
  - Extended AI command and skill synchronization so Claude Code, OpenCode, and Codex expose the same DART workflow capabilities with parity checks for generated command and skill files.
  - Added a single-page DART 7 architecture overview (`docs/readthedocs/architecture.md`, published as **Architecture** on the docs site) mapping the multi-physics, multi-solver, and multi-backend simulation pipeline as abstracted boxes with the available options at each seam, plus a `dart-architecture` domain skill and pointers from the north star, onboarding, and design indexes.
  - Promoted prompt-only AI workflows into synced workflow commands and removed the separate prompt-template folder.
  - Fixed generated Codex DART skill frontmatter so strict YAML parsers load AI workflow skills without warnings. ([#2546](https://github.com/dartsim/dart/pull/2546))
  - Simplified copyright headers to first publication year (REUSE compliance). ([#2438](https://github.com/dartsim/dart/pull/2438))
  - Tightened GitHub Actions CI workflows with granular platform checks, single-pass docs validation, targeted pixi environment installs, shared SIMD compiler-cache setup, protected-branch push triggers, retrying Alt Linux package bootstrap, importable Ubuntu Debug dartpy test builds, and macOS Debug C++ coverage without multi-hour Debug dartpy rebuilds. ([#2539](https://github.com/dartsim/dart/pull/2539))
  - CI workflow optimizations, caching, and scheduling controls (including compiler cache guards). ([#2055](https://github.com/dartsim/dart/pull/2055), [#2079](https://github.com/dartsim/dart/pull/2079), [#2135](https://github.com/dartsim/dart/pull/2135), [#2160](https://github.com/dartsim/dart/pull/2160), [#2165](https://github.com/dartsim/dart/pull/2165), [#2192](https://github.com/dartsim/dart/pull/2192), [#2313](https://github.com/dartsim/dart/pull/2313), [#2129](https://github.com/dartsim/dart/pull/2129), [#2265](https://github.com/dartsim/dart/pull/2265), [#2267](https://github.com/dartsim/dart/pull/2267), [#2180](https://github.com/dartsim/dart/pull/2180))
  - Added an Eigen 64-byte over-alignment CI/local test task to catch allocator, placement-new, and Eigen storage assumptions without requiring AVX-512 hardware. ([#2541](https://github.com/dartsim/dart/pull/2541))
  - Added alignment-aware `dart::common::MemoryAllocator` and `StlAllocator`
    paths so over-aligned objects and allocator-aware EnTT registries can be
    backed by DART allocators, including simulation-mode checkpoint reloads
    that reserve registry storage before the first resumed step.
  - Added `dart::common::FixedPoolAllocator` for fixed-size slot workloads and
    routed the fixed-size allocator comparison benchmark through it, while
    keeping mixed-size pool workloads on `PoolAllocator`.
  - Fixed the scheduled/manual collision benchmark guard artifact upload so `.benchmark_results/collision_check_*.json` files are retained by GitHub Actions.
  - Added a performance dashboard that runs DART's Google Benchmark suites and
    publishes per-benchmark history to GitHub Pages via
    `benchmark-action/github-action-benchmark`, with a local
    `pixi run bm-dashboard-preview` to render the same dashboard before it is
    published.
  - Expanded the performance dashboard to track the new DART 7 solver families'
    end-to-end `World::step` surfaces (rigid-body sequential-impulse/IPC, VBD and
    default deformable grid, FEM bar, AVBD fixed-joint) and made the charts
    human-readable: `scripts/benchmark_display_names.py` rewrites raw Google
    Benchmark names into readable titles (merge `--humanize`), and the local
    preview groups charts by solver family with labelled, thousands-formatted
    axes (commit on x, time-per-op on y, lower is better).
  - Added an opt-in `pixi run abi-check` task (Linux only) that builds two refs
    with identical options and compares shared libraries with libabigail's
    `abidiff`. The task is diagnostic only and is not wired into CI; ABI
    stability between minor releases is still a deferred topic (see issue
    [#1026](https://github.com/dartsim/dart/issues/1026)).
  - Added `pixi run lint-cmake` / `check-lint-cmake` CMake formatting with
    `gersemi`, wired into the `lint` and `check-lint` chains so `CMakeLists.txt`
    and `*.cmake` files are auto-formatted and CI-checked like every other
    tracked file type. Configuration in `.gersemirc` matches DART's C++ style
    (80-column, 2-space indent).
  - Added `pixi run bm-compute-check`, contact-shaped experimental compute
    coverage, and a Phase 5 CPU-baseline dashboard surface so the full expected
    scalable-compute benchmark corpus is checked in CI.
  - Added `pixi run bm-lie-group-batch` to benchmark the DART 7 SO(3)/SE(3)
    batch `expBatch`, `logBatch`, and `adjointBatch` paths against explicit
    scalar loops before introducing SIMD or other compute backends.
  - Added `pixi run bm-phase5-gpu-packet-check --write-template <packet.json>` /
    `--input <packet.json>` to create and validate the manual Phase 5 GPU
    go/no-go benchmark packet once a project GPU runner exists, including
    build/import and policy-gate evidence booleans.
  - Added `pixi run -e cuda bm-phase5-cuda-full`,
    `pixi run bm-phase5-cuda-packet`, and a manual CUDA workflow artifact path
    that converts Phase 5 CUDA benchmark JSON into a validated GPU go/no-go
    packet after the CUDA build/import and policy gates pass.
  - Added `pixi run check-compute-backend-boundaries` to keep CUDA/SYCL/device
    backend concepts out of public experimental C++ headers and the default
    dartpy experimental bindings.
  - Added `pixi run check-no-gpu-runtime-dependencies` to keep default Pixi and
    dartpy wheel manifests free of GPU runtime dependencies before any optional
    accelerator sidecar exists.
  - Added `pixi run check-phase5-cuda-benchmark-contract` to ensure optional
    CUDA benchmark files register the Phase 5 GPU go/no-go row consumed by the
    packet validator.
  - Added `pixi run check-phase5-cuda-workflow` to keep the manual CUDA workflow
    wired to the Phase 5 policy gates, full benchmark row, packet validator, and
    artifact upload paths.
  - Tightened generated Doxygen API documentation exclusions so internal symbols
    remain filtered after local documentation builds.
  - CodeQL scope updates and coverage workflow fixes. ([#2121](https://github.com/dartsim/dart/pull/2121), [#2128](https://github.com/dartsim/dart/pull/2128), [#2144](https://github.com/dartsim/dart/pull/2144), [#2147](https://github.com/dartsim/dart/pull/2147), [#2197](https://github.com/dartsim/dart/pull/2197), [#2198](https://github.com/dartsim/dart/pull/2198))
  - Documented scalable coverage organization and memory-bounded validation guidance for targeted local builds. ([#2648](https://github.com/dartsim/dart/pull/2648))
  - GitHub Actions and tooling dependency bumps. ([#2049](https://github.com/dartsim/dart/pull/2049), [#2050](https://github.com/dartsim/dart/pull/2050), [#2063](https://github.com/dartsim/dart/pull/2063), [#2064](https://github.com/dartsim/dart/pull/2064), [#2065](https://github.com/dartsim/dart/pull/2065), [#2074](https://github.com/dartsim/dart/pull/2074), [#2075](https://github.com/dartsim/dart/pull/2075), [#2076](https://github.com/dartsim/dart/pull/2076), [#2102](https://github.com/dartsim/dart/pull/2102), [#2103](https://github.com/dartsim/dart/pull/2103), [#2200](https://github.com/dartsim/dart/pull/2200), [#2264](https://github.com/dartsim/dart/pull/2264), [#2344](https://github.com/dartsim/dart/pull/2344), [#2345](https://github.com/dartsim/dart/pull/2345), [#2346](https://github.com/dartsim/dart/pull/2346))
  - Linting and formatting tasks updated (Python formatting, TOML, Markdown, RST, spelling, and Doxygen command prefix). ([#2054](https://github.com/dartsim/dart/pull/2054), [#2185](https://github.com/dartsim/dart/pull/2185), [#2237](https://github.com/dartsim/dart/pull/2237), [#2248](https://github.com/dartsim/dart/pull/2248), [#2251](https://github.com/dartsim/dart/pull/2251), [#2244](https://github.com/dartsim/dart/pull/2244))
  - Pixi lockfile refreshes. ([#2052](https://github.com/dartsim/dart/pull/2052), [#2066](https://github.com/dartsim/dart/pull/2066), [#2078](https://github.com/dartsim/dart/pull/2078), [#2123](https://github.com/dartsim/dart/pull/2123), [#2179](https://github.com/dartsim/dart/pull/2179), [#2235](https://github.com/dartsim/dart/pull/2235), [#2289](https://github.com/dartsim/dart/pull/2289), [#2293](https://github.com/dartsim/dart/pull/2293), [#2306](https://github.com/dartsim/dart/pull/2306), [#2336](https://github.com/dartsim/dart/pull/2336), [#2348](https://github.com/dartsim/dart/pull/2348))
  - Documentation and onboarding improvements (developer guides, CI/testing workflows, IK and servo docs, RTD updates, and translations). ([#2059](https://github.com/dartsim/dart/pull/2059), [#2062](https://github.com/dartsim/dart/pull/2062), [#2089](https://github.com/dartsim/dart/pull/2089), [#2095](https://github.com/dartsim/dart/pull/2095), [#2112](https://github.com/dartsim/dart/pull/2112), [#2156](https://github.com/dartsim/dart/pull/2156), [#2171](https://github.com/dartsim/dart/pull/2171), [#2173](https://github.com/dartsim/dart/pull/2173), [#2174](https://github.com/dartsim/dart/pull/2174), [#2183](https://github.com/dartsim/dart/pull/2183), [#2191](https://github.com/dartsim/dart/pull/2191), [#2196](https://github.com/dartsim/dart/pull/2196), [#2199](https://github.com/dartsim/dart/pull/2199), [#2203](https://github.com/dartsim/dart/pull/2203), [#2213](https://github.com/dartsim/dart/pull/2213), [#2215](https://github.com/dartsim/dart/pull/2215), [#2302](https://github.com/dartsim/dart/pull/2302), [#2303](https://github.com/dartsim/dart/pull/2303), [#2312](https://github.com/dartsim/dart/pull/2312), [#2087](https://github.com/dartsim/dart/pull/2087))
  - Maintenance merges and stability fixes for release branches and simulation-experimental builds. ([#2145](https://github.com/dartsim/dart/pull/2145), [#2188](https://github.com/dartsim/dart/pull/2188), [#2301](https://github.com/dartsim/dart/pull/2301), [#2341](https://github.com/dartsim/dart/pull/2341), [#2357](https://github.com/dartsim/dart/pull/2357), [#2223](https://github.com/dartsim/dart/pull/2223), [#2228](https://github.com/dartsim/dart/pull/2228), [#2236](https://github.com/dartsim/dart/pull/2236))
  - Miscellaneous repo hygiene: Docker build script fixes, Dependabot path cleanup, template updates, and badge/documentation cleanup. ([#2058](https://github.com/dartsim/dart/pull/2058), [#2291](https://github.com/dartsim/dart/pull/2291))
  - Fixed the Read the Docs dartpy autodoc stub fallback so the generated stubs
    import cleanly. Class-body forward references emitted by `nanobind`
    (snake_case method aliases placed before the method, enclosing-class-qualified
    nested enum members, and aliases to module classes defined later) are now
    reordered or rewritten, and runtime-only submodules such as
    `dartpy.simulation_experimental.diff` are bound to placeholder modules. The
    `dartpy`, `gui`, `simulation_experimental`, and `io` API reference pages no
    longer fail to import with `NameError`.
  - Made `clang-format` discovery resilient to toolchain upgrades by dropping a
    stale `CLANG_FORMAT_EXECUTABLE` cache entry when the cached binary no longer
    exists, so the `format`/`check-format` targets re-resolve against the current
    toolchain instead of failing with a dangling dependency when the pinned
    `clang-format` version changes under an existing build tree.

- Simulation
  - Added opt-in runtime replay recording to the experimental `World`, with C++
    and dartpy controls for enabling recording, querying recorded timestep
    metadata, clearing the buffer, and restoring a recorded frame in place
    without re-running physics. Replay frames deliberately store only mutable
    runtime state needed to restore an already-simulated frame, not topology,
    geometry, material, or static construction data, and layout changes are
    rejected on restore. Added focused C++/Python regressions and a
    `replay_scrubber` `py-demos` scene with timestep-resolution
    slider/controller playback.
  - Consolidated the experimental CUDA solver modules (rigid-body batch, vertex
    block descent, deformable PSD projection) onto a shared device-runtime
    substrate so new GPU solvers reuse common blocks instead of reinventing them:
    one `isCudaRuntimeAvailable`/`throwIfCudaError`/`checkLastError`/`launchGrid1D`
    (`compute/cuda/cuda_runtime.cuh`), one owning `DeviceBuffer<T>`
    (`compute/cuda/device_buffer.cuh`), and a single-sourced per-body
    `__host__ __device__` orientation integration core
    (`compute/detail/rigid_integration_core.hpp`) shared by the CPU batch kernel
    and the CUDA kernel (deleting a divergent device re-derivation). The
    build-only `-cuda` static library, backend-neutral public API, and
    no-GPU-runtime-dependency packaging are unchanged; the experimental CUDA
    failure path now throws `InvalidOperationException` (backward compatible — it
    derives from `std::runtime_error`) for one consistent error contract. Design:
    `docs/design/shared_cuda_device_substrate.md` (PLAN-031).
  - Added opt-in analytic differentiable simulation to the experimental `World`
    (the Nimble method, arXiv:2103.16021): a build-time `DART_BUILD_DIFF` option
    plus a runtime `WorldOptions::differentiable` flag (off by default, with
    bitwise-identical results and no snapshot allocation when off). When enabled,
    `World::getStepDerivatives()` returns DART-owned state/control/parameter
    Jacobians and `World::applyStepVjp()` an efficient vector-Jacobian product,
    computed by implicit differentiation of the boxed-LCP contact solve (the
    clamping-block `A_CC⁻¹` gradient with the friction upper-bound mapping) and
    the articulated-body dynamics. Covers all joint types (including SO(3)/SE(3)
    manifold position Jacobians), frictionless and Coulomb-friction contact,
    rotational/off-COM and multi-contact, and mass/inertia/friction parameter
    derivatives; adds an opt-in boxed-LCP rigid-body contact path
    (`WorldOptions::contactSolverMethod`), a framework-neutral `diff::rollout`,
    contact-gradient refinement modes (`ContactGradientMode`), and a dartpy
    `sx.diff` bridge (`timestep` as a `torch.autograd.Function`, `rollout`,
    `get_step_derivatives`/`apply_step_vjp`) with lazy torch import. The
    articulated-body dynamics derivatives are analytic (`O(dof²)`
    recursive-Newton-Euler derivative recursions, with a finite-difference
    fallback for manifold/configuration-dependent joints) and the contact
    gradient exploits the block-sparse Delassus system, so the
    analytic-vs-finite-difference speedup grows with system size — reproducing
    the paper's scaling result and, on that hardware-normalized metric, matching
    or beating the reference `nimblephysics` implementation measured on the same
    machine. The paper's gradient-based trajectory-optimization experiment is
    reproduced on a cartpole. All gradients are finite-difference-of-step
    verified; no solver, reverse-pass cache, ECS, or tensor-backend types are
    exposed publicly.
  - Added the first internal experimental AVBD rigid-body contact-stage
    activation for supported free rigid-body contacts. The first World slice
    routes privately opted-in rigid contacts through private 6-DOF AVBD
    point-pair rows as a velocity-level projection while unsupported envelopes
    fall back to the existing sequential-impulse path; no AVBD row storage or
    solver registry is exposed through the public facade. The private rigid
    contact snapshot now also derives box face/edge/corner and capsule
    side/top-cap/bottom-cap endpoint feature IDs and scopes contact row ordinals
    per canonical endpoint pair so unrelated contact manifolds do not perturb
    warm-start identity. Added private rigid point-joint linear, angular, and
    combined AVBD row builders so fixed-anchor joint translation and orientation
    rows can share the rigid row driver; the builders seed step-start constraint
    values for AVBD alpha regularization. This is not articulated World joint
    wiring yet.
  - Extended the private AVBD rigid contact feature identity path with cylinder
    side/cap/rim endpoint features, so cylinder contacts warm-start across
    same-feature motion but reset when they move between barrel, cap, and rim
    manifolds. Capsule side/top-cap/bottom-cap endpoint features provide the
    same private warm-start partitioning for capsule contacts. Known-shape
    contacts now map world contact points through body and collision-shape local
    transforms before feature coding, and unknown-index contacts use the same
    shape-frame path when exactly one collision shape can be inferred.
  - Added a private AVBD rigid World point-joint snapshot path that appends
    world-space point-joint inputs to the same rigid snapshot/solve/apply
    wrapper and combined step helper with persistent linear and angular joint
    row inventories. A detail-only fixed-joint ECS extractor and step-helper
    overload can now feed that private path for rigid-body-linked joint
    entities, and the internal contact-stage AVBD opt-in can project those
    fixed joint rows with or without active contact rows. Public multibody
    joint wiring is still out of scope.
  - Added experimental C++ `World::addRigidBodyFixedJoint()` and dartpy
    `World.add_rigid_body_fixed_joint()` for design-mode fixed constraints
    between two free rigid bodies, backed by the private AVBD fixed-joint
    projection path and covered by a `py-demos` rigid fixed-joint scene.
  - Added experimental AVBD point-joint stiffness controls on public joint
    handles (`Joint::setAvbdStartStiffness()`, `setAvbdLinearStiffness()`, and
    `setAvbdAngularStiffness()`, plus dartpy properties) so rigid point-joint
    facades can choose hard rows or finite material-stiffness rows without
    exposing solver row storage.
  - Added explicit rigid-body endpoint accessors for public experimental
    rigid-body fixed joints (`Joint::getParentRigidBody()`/
    `getChildRigidBody()`, dartpy `joint.parent_rigid_body`/
    `joint.child_rigid_body`) with tests, benchmark coverage, and py-demo
    panel output.
  - Added an experimental computation-graph substrate with sequential and
    parallel executors, routed experimental `World::updateKinematics()` and
    `World::step()` through graph-backed rigid-body linear-force integration
    and kinematics stages, introduced the first swappable `WorldStepStage`
    contract and domain-neutral `WorldStepPipeline`, added stage metadata for
    solver domains and acceleration opportunities, added opt-in execution
    profiles and DOT visualization for per-node load, observed parallelism, and
    graph inspection, and added focused tests and benchmark coverage for graph
    batching overhead.
  - Promoted experimental `World::step()` to the batched SoA rigid-body
    integration path by default, including parent-before-child local-transform
    write-back for frame-coupled rigid bodies while keeping executor injection as
    the only public concurrency seam.
  - Added public experimental multibody joint/link API surface for DART 7 and
    dartpy 7, including `JointType`, `JointSpec`, `Link`, `Joint`, Python
    link/joint lookup, and design guidance for closed-chain topology as
    symmetric graph constraints over a tree-shaped multibody.
  - Added public experimental C++ `JointSpec` construction and reused it in the
    dartpy `JointSpec` binding, including finite, non-zero joint-axis
    validation at the API boundary.
  - Added construction-ordered experimental link and joint enumeration on
    `Multibody` in C++ and dartpy, exposing public handles and name snapshots
    without introducing dict-style collection lookup before the uniqueness
    contract is finalized.
  - Added experimental `Multibody::isValid()` and dartpy `Multibody.is_valid`
    so world lifecycle invalidation can be observed through the public handle
    facade.
  - Tightened experimental topology naming so `World` multibody/rigid-body
    names and `Multibody` link/joint names are unique within their owners,
    autogenerated names skip collisions, and cross-multibody parent links are
    rejected.
  - Added experimental C++ `World::hasMultibody()` and dartpy
    `world.has_multibody()` so world-owned multibody lookup has the same
    optional lookup, presence-query, and count shape as rigid bodies and loop
    closures.
  - Added executor overloads for experimental C++ `World::updateKinematics()`
    and repeated `World::step(...)` calls so kinematics-only updates and
    multi-step simulation can reuse caller-owned backend-neutral execution
    policy.
  - Added `compute::ParallelExecutor` as the preferred experimental C++
    parallel compute-graph executor name, keeping the implementation backend
    behind the public executor facade.
  - Added experimental C++ resource-access metadata for compute nodes
    (`compute::ComputeAccessMode`, `compute::ComputeResourceAccess`, and
    `ComputeGraph::findResourceHazards()`) with DOT visualization of declared
    accesses; it reports read/write/reduce/scratch hazards between unordered
    nodes as diagnostics while explicit dependencies stay authoritative, and the
    parallel executor debug-asserts the absence of such hazards.
  - Added experimental C++ and dartpy `LoopClosureRuntimePolicy` metadata for
    closed-chain runtime intent, including residual-only, kinematic projection,
    and dynamic solving policy selections without exposing solver internals.
  - Added experimental C++ `LoopClosure::computeResidual()` and dartpy
    `LoopClosure.compute_residual()` diagnostics for closed-chain residual
    vectors, norms, activation state, coordinate convention, and force
    availability metadata without exposing backend solver rows.
  - Added experimental C++ `World::sync(WorldSyncStage::Kinematics)` and
    dartpy `world.sync(sx.WorldSyncStage.KINEMATICS)` for explicit
    kinematics-only work placement without advancing simulation time.
  - Added experimental C++ `World::setGravity()`/`getGravity()` and dartpy
    `world.gravity`, applying a uniform gravitational acceleration (default
    `(0, 0, -9.81)`) to dynamic rigid bodies through a transient step force
    buffer without storing it in any per-body force accumulator.
  - Added experimental World memory hooks:
    `WorldOptions::baseAllocator`, `WorldOptions::frameScratchInitialCapacity`,
    `World::getMemoryManager()`, and `World::getMemoryDiagnostics()` give each
    experimental World a `MemoryManager` root and report per-step frame-scratch
    usable capacity, usage, peak usage, overflow count, overflow bytes, and
    reset count. Memory diagnostics also include structured allocator debug
    counters plus plain aggregate and per-storage ECS registry layout counters
    for profiler/debugger tooling without exposing EnTT types in the public
    header, and dartpy exposes the same read-only snapshot through
    `World.memory_diagnostics`.
  - Added opt-in per-stage step profiling to the experimental `World`, a
    non-GUI text-first performance surface for tools and AI agents:
    `World::setStepProfilingEnabled()` records each pipeline stage's wall-clock
    time, and `World::getLastStepProfile()` returns a `compute::WorldStepProfile`
    (per-stage name, domain, and duration plus total wall time) with a
    `toSummaryText()` breakdown. Exposed to dartpy as
    `world.step_profiling_enabled`, `world.last_step_profile`, and
    `WorldStepProfile.summary()`. Requires `DART_BUILD_PROFILE=ON`; when the
    profiling build option is off, the runtime toggle is a no-op and the step
    path compiles without profile cache fields, profile branches, or clock
    reads.
    Removed the unused, never-built `dart::simulation::experimental::common`
    `ScopedTimer`/`Stopwatch`/`ProfileStats` profiler (its
    `DART_EXPERIMENTAL_ENABLE_PROFILING` macros were wired into no build and
    duplicated `dart::common::profile`) so the World step profile is the single
    experimental profiling surface.
    Extended the profile with nested `ComputeExecutionProfile` records for
    compute graphs run inside a stage, including worker count, max/average
    parallelism, critical path timing, per-node records, and text summaries;
    stage profiles also report backend-neutral acceleration metadata and
    whether an accelerated backend was active (for example the optional
    deformable PSD accelerator) without exposing device-specific API. dartpy now
    exposes the experimental sequential/parallel compute executors so Python
    profiling can request the multi-threaded World step path. The built-in
    non-experimental `dart::common::profile` text backend now also exposes
    `Profiler::toSummaryText()` and `DART_PROFILE_TEXT_SUMMARY()` so existing
    collision, constraint, and GUI profiling scopes can be consumed as text
    without printing directly to `std::cout`. The same backend is now reachable
    through no-op-safe C++ helpers (`isProfilingEnabled()`,
    `isTextProfilingEnabled()`, `resetProfile()`, `markProfileFrame()`,
    `getProfileSummaryText()`) and dartpy flat functions such as
    `dartpy.get_profile_summary_text()`, making text profiling usable outside
    the experimental World API.
  - Routed the experimental World's internal EnTT registry, component storage,
    and differentiable-parameter list through the World free allocator, with
    state mapper support for World-owned registries and free-list alignment
    fixes for Eigen-backed component storage.
  - Fixed experimental `VectorMapper::toVector()` in-place output so unmapped
    state-space variables are zero-filled even when no mapper slot has been
    registered.
  - Added bake-time reservation for the experimental World's current
    EnTT registry/component storage and private multibody/deformable
    step-scratch storage at `enterSimulationMode()`, including no-growth
    coverage for repeated IPC kinematic, multibody, and deformable steps.
  - Updated experimental `WorldStepPipeline` to store its non-owning stage list
    inline with an eight-stage capacity, eliminating stage-list heap allocation
    during default step pipeline assembly and rejecting overflow with
    `InvalidArgumentException`.
  - Centralized the experimental World's built-in step schedule selection so
    sequential impulse, rigid IPC, semi-implicit/variational multibody,
    deformable, and custom-final-stage paths share one internal slot map.
    Method-family changes after simulation bake now re-run one shared
    preparation path, empty solver domains no longer add placeholder stages, and
    custom-final-stage steps reuse the baked solver stages while clearing
    caller-owned final stage pointers after execution.
  - Added construction-time rigid and multibody solver-family selection to
    experimental `WorldOptions`, and exposed the same validated path through the
    dartpy `sx.World(...)` constructor.
  - Pre-baked the experimental World's default step stage bundle, kinematics
    graph traversal, and rigid IPC kinematic scratch at `enterSimulationMode()`,
    eliminating global heap allocation for repeated baked kinematic IPC steps
    and covering it with a regression test.
  - Reused baked semi-implicit multibody dynamics/contact scratch and persistent
    staged velocity buffers, extending the baked global-heap guard to
    articulated link resting-contact steps.
  - Added a default sequential shortcut for cross-multibody articulated link
    contact impulses, keeping repeated baked cross-link contact steps off the
    global heap for isolated same-DOF pairs while leaving mixed, stacked,
    coupled multi-row, and boxed-LCP unified solve paths on the fallback.
  - Fixed the experimental World's sequential articulated-contact shortcut to
    preflight zero-DOF multibody contacts before applying shortcut impulses, so
    mixed zero-DOF and solvable link-contact scenes fall back to the unified
    solve without double-applying impulses.
  - Reused `UnifiedConstraintStage` boxed-LCP assembly scratch and added an
    in-place unified constraint assembler, reducing per-step transient container
    churn while keeping the full boxed-LCP zero-allocation gate open.
  - Made experimental rigid-body external force/torque components persistent
    applied loads: each step reads them into the transient force buffer and
    leaves the components intact for callers to clear or update explicitly.
  - Added an experimental rigid-body `dart-gui` example that steps an
    experimental `World` and mirrors its rigid-body poses into live rendered
    DART frame geometry.
  - Added experimental rigid-body derived dynamic quantities in C++ and dartpy:
    linear/angular momentum, kinetic energy, and gravitational potential energy.
  - Added an optional pre-joint (parent-side) offset to the experimental
    multibody joint (`JointSpec`/`LinkOptions::transformToParent`, dartpy
    `JointSpec.transform_to_parent`). The joint relative transform is now
    `transformToParent * jointMotion(q) * transformFromParent`, matching the
    legacy `A * Q(q) * C^-1` form, so a joint can be anchored away from the
    parent link's frame origin (offset joints, and sibling joints at different
    locations on a branching parent). The default identity preserves existing
    behavior; the change threads through the forward dynamics, forward
    kinematics, and the variational integrator (the motion subspace is
    unaffected).
  - Extended the experimental rigid-body contact boxed-LCP to solve Coulomb
    friction jointly with the normal impulses. Each contact now contributes
    three rows -- a normal row and two friction-tangent rows whose bounds are
    coupled to the solved normal impulse through the solver's friction index
    (`|lambda_t| <= mu * lambda_n`) -- so normal and friction are resolved in
    one consistent solve instead of a normal solve followed by a separate
    Gauss-Seidel friction sweep. A single isolated contact with no tangential
    motion leaves the friction rows inactive (the elastic-collision velocity
    swap stays exact); a rank-deficient contact set (a box resting flat on a
    plane) falls back to the coupled normal-only solve plus the sequential
    friction sweep. Verified by the existing drop/rest/bounce/friction tests
    and a new sliding-sphere test that checks friction drives the contact slip
    to zero (rolling) through the coupled solve.
  - Corrected shared boxed-LCP solution validation for collapsed intervals
    (`lo == hi`): fixed variables now require the solution to sit on the bound
    but do not require the residual force to be zero. This matches the normal
    cone semantics of a fixed interval and removes false negatives for DART 7
    friction-index rows whose tangent bounds collapse when the normal impulse is
    zero.
  - Fixed boxed semi-smooth Newton LCP solves with friction-index moving
    bounds by including the bound derivative in the natural-residual
    Jacobian. Added manifest-driven coverage for coupled mildly
    ill-conditioned 4-contact friction-index cases across the DART 7 LCP
    solver set, and extended generated standard, boxed, and friction-index
    coverage to larger deterministic cases, including a scoped scalable-solver
    slice up to standard 128-row, boxed 64-row, friction-index 24-contact, and
    coupled friction-index 12-contact problems, plus a scoped robust
    near-singular known-solution slice for standard 8-row, boxed 8-row, and
    coupled friction-index 3-, 6-, 9-, 12-, 16-, 24-, 32-, 48-, 64-, 96-,
    128-, 192-, and 256-contact packets,
    with the friction-index exact-solution check narrowed to Dantzig after
    `ShockPropagation` contract-succeeded but missed the selected generated
    solution tolerance on those coupled packets,
    and active-set
    transition coverage near lower, upper, and friction-cone boundaries. The
    active-set transition correctness grid now reaches standard 128-row, boxed
    128-row, and coupled friction-index 16-contact packets over scoped
    scalable solvers, plus production coupled friction-index 24-, 32-, 48-,
    64-, 96-, 128-, 192-, and 256-contact active-set transition packets with
    stronger cross-contact coupling. Added a scoped larger mildly
    ill-conditioned known-solution slice
    for standard
    32/64-row, boxed 16/32-row, friction-index 8-contact, and
    1x-/4x-/8x-coupled friction-index
    6-/8-/12-/16-/24-/32-/48-/64-/96-contact packets and a 16x-coupled
    128-contact packet, plus an
    exact rank-deficient
    singular-degenerate slice for standard 16-row, boxed 16-row, and coupled
    friction-index 6-contact packets and a larger exact rank-deficient
    singular-degenerate slice for standard 32-row, boxed 32-row, and coupled
    friction-index 8-contact packets, plus a stress exact rank-deficient
    singular-degenerate slice for standard 64-row, boxed 64-row, and coupled
    friction-index 12-contact packets over the solver scope proven by the
    generated harness, plus an extreme exact rank-deficient
    singular-degenerate slice for standard 128-row, boxed 128-row, and coupled
    friction-index 16-, 24-, 32-, 48-, 64-, 96-, 128-, 192-, and 256-contact
    packets.
  - Added an opt-in CPU worker-thread update path for the DART 7 projected
    Jacobi LCP solver (`JacobiSolver::Parameters::workerThreads`), with
    generated 128-row correctness coverage and focused dense plus banded
    serial-vs-worker benchmark rows up to 8192 banded rows and up to 32 worker
    threads on the banded rows. The local benchmark
    rows are comparison evidence, not a speedup claim.
  - Extended DART 7 Red-Black Gauss-Seidel and Blocked Jacobi LCP
    solver-internal CPU threading benchmark evidence from 128-row banded
    packets to 512-, 1024-, and 2048-row banded packets with serial, 4-worker,
    and 8-worker rows. These rows are correctness/comparison evidence, not
    solver speedup or CUDA-kernel claims.
  - Added DART 7 contact-derived block-structure coverage for BGS and Blocked
    Jacobi LCP solvers, proving real boxed-LCP world-contact snapshots solve
    with `findex`-derived non-contiguous per-contact blocks and reject explicit
    block partitions that split tangent rows from their owning normal rows;
    focused world-contact, stack-contact, and serial/parallel batch benchmark
    rows for both solvers also pass the LCP contract.
  - Added opt-in projected gradient-descent warm starts for the DART 7
    standard-LCP Minimum Map, Fischer-Burmeister, and Penalized
    Fischer-Burmeister Newton solvers. Focused unit coverage proves each
    initializer reduces its solver-specific merit before Newton line search.
    Added opt-in PGS warm starts for those same standard Newton paths, accepting
    PGS seeds only when they reduce the solver-specific Newton merit. Boxed/findex
    problems remain delegated for those three solvers. Added 36 benchmark rows
    comparing no seed, PGS, projected gradient descent, and PGS-then-gradient
    modes on identical 32-row, 64-row, and 128-row standard active-set
    transition packets for those solvers, verified in default, SIMD-enabled,
    and CUDA-enabled build trees. Added 72 batch benchmark rows for the same
    warm-start mode matrix over batch-size-4 serial and DART 7
    `ParallelExecutor` standard active-set transition packets. The
    CUDA-enabled rows are CPU solver rows, not CUDA LCP kernel execution.
  - Added focused DART 7 PGS/PSOR, symmetric PSOR, and Red-Black Gauss-Seidel
    relaxation sweep benchmark rows for standard, boxed, and 8-/16-contact
    friction-index fixtures at relaxation 0.5, 1.0, and 1.3, verified in default,
    SIMD-enabled, and CUDA-enabled build trees. The rows distinguish
    under-relaxation, plain relaxation, and over-relaxation; Red-Black rows also
    report two-color partition counters. CUDA-enabled rows are CPU solver rows,
    not CUDA LCP kernel execution.
  - Added DART 7 APGD restart-policy comparison benchmark rows for standard,
    boxed, and 8-/16-contact friction-index LCP fixtures, with backend
    build-state counters distinguishing default, SIMD-enabled, and CUDA-enabled
    CPU solver runs.
  - Added DART 7 TGS iteration-budget comparison benchmark rows for standard,
    boxed, and 8-/16-contact friction-index LCP fixtures, with backend
    build-state counters distinguishing default, SIMD-enabled, and CUDA-enabled
    CPU solver runs.
  - Added DART 7 NNCG PGS-preconditioner iteration comparison benchmark rows
    for standard, boxed, and 8-/16-contact friction-index LCP fixtures, with backend
    build-state counters distinguishing default, SIMD-enabled, and
    CUDA-enabled CPU solver runs.
  - Added DART 7 SubspaceMinimization PGS active-set-estimation iteration
    comparison benchmark rows for standard, boxed, and 8-/16-contact
    friction-index LCP fixtures, with backend build-state counters distinguishing default,
    SIMD-enabled, and CUDA-enabled CPU solver runs.
  - Added DART 7 ShockPropagation layer-layout comparison benchmark rows for
    standard, boxed, and 8-/16-contact friction-index LCP fixtures, comparing
    single-layer, two-layer, and serial schedules with backend build-state
    counters distinguishing default, SIMD-enabled, and CUDA-enabled CPU solver
    runs.
  - Added DART 7 MPRGP SPD/positive-definite-check comparison benchmark rows
    for dense, banded, mildly ill-conditioned, and near-singular standard-LCP
    fixtures up to 128 dense/banded rows and 16 near-singular rows, with
    backend build-state counters distinguishing default, SIMD-enabled, and
    CUDA-enabled CPU solver runs.
  - Added DART 7 Interior Point path-parameter comparison benchmark rows for
    dense, banded, mildly ill-conditioned, and near-singular standard-LCP
    fixtures up to 128 dense/banded rows and 16 near-singular rows, with
    backend build-state counters distinguishing default, SIMD-enabled, and
    CUDA-enabled CPU solver runs.
  - Added DART 7 Staggering contact-pipeline comparison benchmark rows for
    separated world-contact, coupled stack-contact, and articulated unified
    contact fixtures up to 8 contacts, with normal/friction split counters and
    backend build-state counters distinguishing default, SIMD-enabled, and
    CUDA-enabled CPU solver runs.
  - Added DART 7 Boxed Semi-Smooth Newton line-search comparison benchmark
    rows for standard, boxed, and 8-/16-contact friction-index fixtures, with
    backend build-state counters distinguishing default, SIMD-enabled, and
    CUDA-enabled CPU solver runs.
  - Added DART 7 pivoting scale comparison benchmark rows for Direct 2D/3D
    enumeration, Lemke and Baraff standard fixtures, and Dantzig standard
    through 32 rows, boxed through 48 rows, and friction-index through 16
    contacts, with backend build-state counters
    distinguishing default, SIMD-enabled, and CUDA-enabled CPU solver runs.
  - Added DART 7 BGS and Blocked Jacobi block-partition comparison benchmark
    rows for full-block, 3-row block, auto `findex`, and explicit contact-block
    partitions on standard, boxed, and 4-/8-contact friction-index fixtures,
    with backend build-state counters distinguishing default, SIMD-enabled, and
    CUDA-enabled CPU solver runs.
  - Added manifest-generated serial and DART 7 `ParallelExecutor` batch LCP
    benchmarks so standard, boxed, and friction-index solver families compare
    the same independent-problem batches across every supporting solver. The
    benchmark suite also includes active-set transition rows for standard,
    boxed, and coupled friction-index generated packets near lower, upper, and
    friction-cone boundaries, plus 49 larger active-set transition rows for
    standard 32-row, boxed 32-row, and coupled friction-index 8-contact
    packets and 49 stress active-set transition rows for standard 64-row,
    boxed 64-row, and coupled friction-index 12-contact packets verified across
    default, SIMD-enabled, and CUDA-enabled build trees, plus 49 extreme
    active-set transition rows for standard 128-row, boxed 128-row, and coupled
    friction-index 16-contact packets verified across default, SIMD-enabled,
    and CUDA-enabled build trees, plus generated production active-set
    correctness coverage through a 256-contact/768-row packet and 128
    production active-set transition benchmark rows for
    24-contact/72-row, 32-contact/96-row, 48-contact/144-row, and
    64-contact/192-row, 96-contact/288-row, 128-contact/384-row, and
    192-contact/576-row and 256-contact/768-row coupled friction-index packets
    verified across default, SIMD-enabled, and CUDA-enabled build trees, plus
    550 production active-set transition batch
    rows for batch-size-4 serial and DART 7 `ParallelExecutor` runs over the
    standard 32/64/128-row, boxed 32/64/128-row, and coupled friction-index
    8-/12-/16-/24-/32-/48-/64-/96-/128-/192-/256-contact active-set packets
    verified across default, SIMD-enabled, and CUDA-enabled build trees, plus
    larger mildly ill-conditioned generated correctness coverage for standard
    32/64-row, boxed 16/32-row, friction-index 8-contact,
    1x-/4x-/8x-coupled 6-/8-/12-/16-/24-/32-/48-/64-/96-contact packets, and
    16x-coupled packets through 192 contacts over solvers that reproduce the
    selected generated solution, plus 629 benchmark rows with 16x-coupled
    single-problem packets through 256 contacts. Boxed Semi-Smooth Newton is
    included across those coupled single-problem rows. The
    256-contact single rows are verified across default, SIMD-enabled, and
    CUDA-enabled build trees. The
    SIMD-enabled SAP 192-contact row is contract-correct but slow and does not
    claim a speedup. Added
    1258 larger mildly ill-conditioned batch rows for batch-size-4 serial and
    DART 7 `ParallelExecutor` runs over standard 32-row, boxed 16-row,
    friction-index 8-contact, coupled friction-index
    6-/8-/12-/16-/24-/32-/48-/64-/96-contact, 4x-coupled
    6-/8-/12-/16-/24-/32-/48-/64-/96-contact, 8x-coupled
    6-/8-/12-/16-/24-/32-/48-/64-/96-contact, and 15-solver 16x-coupled
    6-/8-/12-/16-/24-/32-/48-/64-/96-/128-/192-/256-contact rows. Boxed
    Semi-Smooth Newton reports tuned line-search settings on the 16x rows. The
    new 192- and 256-contact batch rows are verified in the default build, and
    focused SIMD/CUDA-enabled all-solver serial/parallel 192-contact batch
    contract gates now pass. Focused SIMD/CUDA-enabled selected-solver
    256-contact batch gates now pass for PGS, NNCG, APGD, TGS, ADMM, and Boxed
    Semi-Smooth Newton, while the full all-solver 256-contact SIMD/CUDA batch
    gate remains unclaimed. Added 31 near-singular benchmark rows for standard 8-row,
    boxed 8-row, and
    coupled friction-index 3-, 6-, 9-, 12-, 16-, 24-, 32-, 48-, 64-, 96-,
    128-, 192-, and 256-contact
    packets verified across default, SIMD-enabled, and CUDA-enabled build
    trees, plus 62 near-singular batch rows for batch-size-4 serial and
    DART 7 `ParallelExecutor` runs over standard 8-row, boxed 8-row, and those
    coupled friction-index packets verified across default, SIMD-enabled, and
    CUDA-enabled build trees, plus
    27 exact
    rank-deficient singular-degenerate benchmark rows for standard 16-row,
    boxed 16-row, and coupled friction-index 6-contact packets, plus 27 larger
    exact rank-deficient singular-degenerate benchmark rows for standard
    32-row, boxed 32-row, and coupled friction-index 8-contact packets, plus
    27 stress exact rank-deficient singular-degenerate benchmark rows for
    standard 64-row, boxed 64-row, and coupled friction-index 12-contact
    packets, each verified across default, SIMD-enabled, and CUDA-enabled build
    trees, plus 51 extreme exact rank-deficient singular-degenerate rows for
    standard 128-row, boxed 128-row, and coupled friction-index
    16-/24-/32-/48-/64-/96-/128-/192-/256-contact packets verified across default,
    SIMD-enabled, and CUDA-enabled build
    trees, plus 72 exact rank-deficient singular-degenerate friction-index
    batch rows for batch-size-4 serial and DART 7 `ParallelExecutor` runs over
    coupled friction-index
    6-/8-/12-/16-/24-/32-/48-/64-/96-/128-/192-/256-contact packets verified
    across default, SIMD-enabled, and CUDA-enabled build trees, plus 192 exact
    rank-deficient singular-degenerate standard/boxed batch rows for
    batch-size-4 serial and DART 7 `ParallelExecutor` runs over
    16-/32-/64-/128-row packets verified across default, SIMD-enabled, and
    CUDA-enabled build trees. The
    LCP benchmark rows now report scalar/SIMD/CUDA build-state counters so
    backend-specific runs are distinguishable in the benchmark output.
  - Added DART 7 experimental CUDA LCP execution paths for fixed-iteration
    projected Jacobi and PGS batch solves over homogeneous dense standard,
    boxed, and friction-index packets now reaching 256-row standard/boxed and
    96-contact friction-index synthetic cases, grouped variable-size synthetic
    standard, boxed, and friction-index packets now reaching
    16/32/48/96/128/192-row standard/boxed groups and
    4/8/16/32/48/64-contact friction-index groups with
    two- and three-variant grouped rows,
    matching CPU serial and DART 7 `ParallelExecutor` Jacobi/PGS batch rows
    verified in default, SIMD-enabled, and CUDA-enabled builds at both those
    grouped packets and the
    24-/48-/96-/128-/192-/256-row and 8-/16-/32-/48-/64-/96-contact
    direct CUDA packet sizes,
    homogeneous 4-, 8-, and 16-contact world-contact,
    homogeneous 5-/6-/7-/8-/9-/10-/11-/12-/13-/14-/15-/16-sphere coupled stack-contact, grouped variable-size
    1/2/4/8/16-contact separated world-contact LCP packets with additional
    three-variant grouped rows, plus grouped
    variable-size 2/3/4/5/6/7/8/9/10/11/12/13/14/15/16-sphere coupled stack-contact LCP packets with
    additional three-variant grouped stack benchmark rows, plus
    grouped variable-size manually assembled 1-/4-/8-/16-contact articulated
    unified-contact LCP packets with additional three-variant grouped rows,
    including cross-multibody link-vs-link cases,
    plus mixed grouped contact batches combining separated, stack, and
    1-/4-/8-/16-contact articulated fixture families including cross-multibody link-vs-link
    packets, with
    CUDA unit coverage and benchmark rows that report CUDA LCP, CUDA batch
    execution, CUDA grouped-batch execution, world-contact batch, and
    stack-contact batch counters. General CUDA execution across the full solver
    manifest remains future work.
  - Added DART 7 `dart::simulation::World` boxed-LCP snapshot coverage for one
    sphere-ground friction contact and for two separated sphere-ground contacts
    assembled into one boxed/friction-index LCP, plus a 200-step two-sphere
    `World::step()` invariant test. Added contact-derived LCP benchmark rows
    that compare all friction-index-capable solvers on identical 1/2/4
    separated sphere-ground boxed/findex snapshots, plus boxed-LCP
    contact assembly/solve benchmark rows for the same contact counts. Added
    coupled 2/3-sphere vertical-stack benchmark rows for the same solver set,
    plus scoped 4-, 5-, and 6-sphere stack rows for all of those solvers
    (`NNCG` uses 20 PGS preconditioner iterations for this coupled contact
    family), plus 7-sphere rows for that full solver set
    (`RedBlackGaussSeidel` reports a 512-iteration stack-contact cap for this
    family), plus 8-/9-/10-/11-/12-/13-sphere rows for that full solver set (`Pgs`,
    `Jacobi`, `BlockedJacobi`, `RedBlackGaussSeidel`, and `ShockPropagation`
    use a 512-iteration stack-contact cap; `SymmetricPsor`, `BGS`, and `Tgs`
    use that cap on the 11-/12-/13-sphere rows; and `NNCG` uses 20 PGS
    preconditioner iterations through 11 spheres and 40 at 12+ spheres). Focused
    default, SIMD-enabled, and CUDA-enabled build-tree rows pass for the
    11-/12-/13-sphere all-solver stack slice; the CUDA-enabled rows are CPU solver
    rows in a CUDA-enabled build, not CUDA LCP kernel execution. Added 3-, 4-,
    5-, 6-, 7-, 8-, 9-, 10-, 11-, 12-, 13-, 14-, 15-, and 16-sphere stack
    snapshot tests that validate nonzero normal-contact coupling, plus boxed-LCP
    Baumgarte velocity-bias stabilization that preserves kinematic contacts'
    static-obstacle compatibility behavior, and
    3-sphere 200-step, 3-sphere 500-step,
    4-sphere 200-step, 5-sphere 500-step, and 6-sphere 1000-step
    `World::step()` invariant tests with matching benchmark rows for the
    public boxed-LCP stack path. Added 4- and 16-sphere separated-contact
    `World::step()` invariant tests and 4-/8-/16-sphere separated-contact
    `World::step()` benchmark rows for the public boxed-LCP path. Stack
    assembly/solve benchmark rows now include 7-sphere, 7-contact, 21-row,
    8-sphere, 8-contact, 24-row, 9-sphere, 9-contact, 27-row, 10-sphere,
    10-contact, 30-row, 11-sphere, 11-contact, 33-row, 12-sphere,
    12-contact, 36-row, 13-sphere, 13-contact, 39-row, 14-sphere,
    14-contact, 42-row, 15-sphere, 15-contact, 45-row, and 16-sphere,
    16-contact, 48-row coupled stack snapshots. Added
    mixed world-contact batch benchmark rows that compare every
    friction-index-capable solver on the same five separated-contact and
    stacked-contact snapshots, both serially and through the DART 7
    experimental `ParallelExecutor`. Added stress mixed world-contact batch
    benchmark rows for every friction-index-capable solver except `NNCG`, over
    the same separated-contact snapshots plus 2/3/4/5-sphere coupled stack
    snapshots, both serially and through `ParallelExecutor`. Added fixed-base
    dense box-face contact evidence: a 4-contact, 12-row boxed/findex snapshot
    APGD-verified in tests, dense contact assertions in the sliding/static box
    `World::step()` tests, and 72 scoped
    `BM_LcpWorldBoxContact/FrictionIndex` benchmark rows over
    1/2/4/8/16/24/32/48/64/96/128/192-box snapshots verified in default, SIMD-enabled, and
    CUDA-enabled build trees, plus 72 serial and DART 7
    `ParallelExecutor` dense box-face batch rows for `Pgs`,
    `RedBlackGaussSeidel`, `NNCG`, `Apgd`, `Tgs`, and `Admm` over
    24/64/96/128/192-box snapshots, with `Pgs` additionally covering
    1/4/8/16/32/48-box snapshots so the CPU rows match homogeneous CUDA PGS
    packet sizes through 96 boxes in default, SIMD-enabled, and CUDA-enabled
    build trees,
    plus CUDA batch coverage for homogeneous Jacobi
    1/4/8/16/24/32/48/64/96-box batch-size-4 rows and a 128-box
    batch-size-1 row,
    homogeneous PGS 1/4/8/16/24/32/48/64/96-box batch-size-4 rows and a
    128-box batch-size-1 row, and Jacobi/PGS grouped
    1/2/4/8/16/24/32/48/64/96-box dense box-face packets with two and three velocity
    variants per box-count shape through
    `BM_LcpCudaJacobiWorldBoxContactBatch_FrictionIndex`,
    `BM_LcpCudaJacobiWorldBoxContactGroupedBatch_FrictionIndex`,
    `BM_LcpCudaPgsWorldBoxContactBatch_FrictionIndex`, and
    `BM_LcpCudaPgsWorldBoxContactGroupedBatch_FrictionIndex`, and
    1/2/4/8/16/24/32-box
    `BM_LcpWorldBoxStep_BoxedLcp` end-to-end invariant benchmark rows verified
    in default, SIMD-enabled, and CUDA-enabled build trees plus a focused
    default-build 48-box/192-contact row, plus a
    `FourBoxWorldStepMaintainsDenseContactInvariants` unit test for a 16-contact
    dense box-face public-step scene and
    `EightBoxWorldStepMaintainsDenseContactInvariants` and
    `SixteenBoxWorldStepMaintainsDenseContactInvariants` for 32-contact and
    64-contact scenes, plus
    `TwentyFourBoxWorldStepMaintainsDenseContactInvariants` for a 96-contact
    small-timestep scene and
    `ThirtyTwoBoxWorldStepMaintainsDenseContactInvariants` for a
    128-contact small-timestep scene, plus
    `FortyEightBoxWorldStepMaintainsDenseContactInvariants` for a
    192-contact small-timestep scene.
    Added fixed-base prismatic articulated
    link-ground `World::step()` invariant coverage for one-link and four-link
    contact scenes, plus 1-/4-/8-/16-link articulated ground-step benchmark
    rows through the public boxed-LCP unified constraint path. Added connected
    fixed-base three-axis prismatic Cartesian-chain link-ground `World::step()`
    invariant coverage and 1-/4-/8-/16-chain articulated Cartesian ground-step
    benchmark rows through the same public unified path. Added fixed-base
    prismatic link-vs-dynamic-rigid `World::step()` invariant coverage and
    1-/4-/8-/16-pair articulated rigid-impact benchmark rows through the same
    public unified path. Added cross-multibody fixed-base prismatic
    link-vs-link `World::step()` invariant coverage and
    1-/4-/8-/16-pair articulated link-impact benchmark rows through the same
    public unified path. Added
    all-solver articulated unified-contact benchmark rows for manually
    assembled fixed-base three-axis prismatic link-ground and
    link-vs-dynamic-rigid LCP snapshots, now extended to cross-multibody
    link-vs-link LCP snapshots and 32-contact packets, plus CUDA grouped-batch Jacobi/PGS
    execution rows over manually assembled 1-/4-/8-/16-contact articulated
    unified-contact packets covering link-ground, link-vs-dynamic-rigid, and
    cross-multibody link-vs-link cases and mixed CUDA grouped-batch rows that
    combine separated, stack, and 1-/4-/8-/16-contact articulated
    cross-multibody contact packets in one benchmark family with two- and
    three-variant mixed scenario rows.
  - Updated the DART 7 LCP background taxonomy and selection guide to include
    APGD, TGS, boxed semi-smooth Newton, ADMM, and SAP alongside the shared
    solver manifest, correctness coverage, and benchmark registration.
    The selection guide's available-solver block is now checked against
    `kLcpSolverManifest` by the all-solvers smoke test so documented DART 7 LCP
    solver availability cannot drift silently from implementation coverage.
  - Added DART 7 ADMM rho/adaptive-rho comparison benchmark rows for standard,
    boxed, and 8-/16-contact friction-index LCP fixtures, with backend
    build-state counters distinguishing default, SIMD-enabled, and CUDA-enabled
    CPU solver runs.
  - Added DART 7 SAP regularization comparison benchmark rows for standard,
    boxed, and 8-/16-contact friction-index LCP fixtures, with backend
    build-state counters distinguishing default, SIMD-enabled, and CUDA-enabled
    CPU solver runs.
  - Added DART 7 ADMM, SAP, and Boxed Semi-Smooth Newton contact comparison
    benchmark rows for separated world-contact, coupled stack-contact, and
    articulated unified-contact friction-index fixtures up to 8 contacts, with
    backend build-state counters distinguishing default, SIMD-enabled, and
    CUDA-enabled CPU solver runs.
  - Added DART 7 generated correctness coverage for
    1x-/4x-/8x-coupled mildly ill-conditioned friction-index LCP packets
    through 96 contacts and 16x-coupled packets through 192 contacts, plus
    single/batch benchmark rows through 256 contacts across the scoped
    iterative solver set plus Boxed Semi-Smooth Newton. Boxed
    Semi-Smooth Newton reports tuned line-search settings on the 16x rows.
    Focused SIMD/CUDA-enabled all-solver serial/parallel 192-contact batch
    contract gates now pass; CUDA-enabled rows are CPU solver rows in that
    build tree, not CUDA LCP kernel execution.
  - Added DART 7 contact-normal standard-LCP comparison benchmark rows for
    Dantzig, Lemke, Baraff, Direct, Minimum Map Newton,
    Fischer-Burmeister Newton, Penalized Fischer-Burmeister Newton, Interior
    Point, and MPRGP on normal-only subproblems extracted from separated
    world-contact, coupled stack-contact, and articulated unified-contact
    snapshots up to 8 contacts. Direct rows are limited to 1-, 2-, and 3-row
    subproblems to avoid benchmarking its Dantzig fallback as direct
    enumeration.
  - Replaced the experimental rigid-body contact stage's per-contact
    sequential normal impulses with a coupled boxed-LCP solve over all
    rigid-rigid contacts (`dart/math/lcp` Dantzig solver). It assembles the
    contact-space inverse-mass (Delassus) operator and drives each contact's
    normal approach velocity to its restitution target in one consistent
    solve, so coupled contacts (a stack, where the lower contact must carry
    the weight above it) resolve together rather than through a single
    Gauss-Seidel sweep. A single isolated contact reduces to the previous
    closed-form impulse exactly, and a rank-deficient contact set (a box
    resting flat on a plane) falls back to an uncoupled diagonal projection;
    the proven friction pass and positional correction are unchanged. Verified
    by the existing drop/rest/bounce/friction tests and a new coupled
    two-sphere-stack rest test.
  - Added an experimental model-loading bridge that builds a `Multibody` from a
    legacy `dynamics::Skeleton`: C++ `io::buildMultibodyFromSkeleton(world,
skeleton, options)` and dartpy `build_multibody_from_skeleton` /
    `SkeletonToMultibodyOptions`. It maps weld, revolute, prismatic, screw,
    universal, ball, free (floating base), and planar joints in arbitrary trees
    on a fixed base — including offset and branching joints, by placing each
    joint with the new pre-joint offset (transformToParent = A, transformFromParent
    = C^-1) so each link frame coincides with its legacy body frame. It converts
    the screw pitch and free-joint coordinate conventions, carries
    revolute/prismatic joint properties (position/velocity/effort limits,
    damping, spring stiffness and rest position, Coulomb friction), and
    reproduces the legacy skeleton's mass matrix and Coriolis/gravity dynamics.
    Combined with `dart::io` URDF/SDF parsing this loads a model file into the
    experimental World, and `io::buildMultibodiesFromWorld` (dartpy
    `build_multibodies_from_world`) loads every skeleton of a legacy
    `simulation::World` as its own multibody (a whole scene). Sphere, box,
    capsule, cylinder, plane, triangular `MeshShape`, `ConvexMeshShape`,
    `HeightmapShape`, and `SoftMeshShape` collision shapes are carried onto the
    links with their shape-node offsets preserved (gated by
    `loadCollisionShapes`). Other collision shape types and rotated parent-side
    offsets on
    ball/free/planar joints are not yet translated (the former is skipped; the
    latter raises a descriptive error).
  - Added experimental articulated-body forward dynamics for fixed-base
    multibodies: `World::step()` now integrates revolute and prismatic joint
    accelerations from joint efforts, gravity, and Coriolis/centrifugal terms
    using a recursive Newton-Euler formulation. Added joint effort/acceleration
    accessors (`Joint::getForce`/`setForce`/`getAcceleration`, dartpy
    `joint.force`/`joint.acceleration`), link inertial accessors
    (`Link::getMass`/`setMass`/`getInertia`/`setInertia`, dartpy
    `link.mass`/`link.inertia`), and a `JointSpec`/`LinkOptions`
    `transformFromParent` link offset (dartpy `transform_from_parent`).
  - Added experimental screw-joint forward dynamics: a `Screw` joint couples
    rotation and translation along its axis by a pitch (`Joint::setPitch`/
    `getPitch`, dartpy `joint.pitch`), with motion subspace `[axis; pitch*axis]`.
    Verified by the analytic mass matrix `M = I_axis + m pitch^2` and gravity
    acceleration.
  - Added experimental universal-joint (2-DOF) forward dynamics: a `Universal`
    joint rotates about `axis` then `axis2` (`JointSpec`/`LinkOptions::axis2`,
    dartpy `JointSpec.axis2` and read-only `joint.axis2`). Its first motion
    subspace column is configuration dependent, so the recursive Newton-Euler
    acceleration recursion now adds the joint velocity-product bias
    `cJ = Sdot qdot`; constant-subspace joints (revolute/prismatic/screw) are
    unaffected. Verified by the closed-form mass matrix and gravity at the zero
    configuration and by matching the Coriolis force to the Christoffel symbols
    derived from the configuration-dependent mass matrix.
  - Added experimental planar-joint (3-DOF) forward dynamics: a `Planar` joint
    provides two in-plane translations plus a rotation about the plane normal
    (`axis` is the normal, `axis2` the first in-plane direction), matching the
    existing kinematics convention. Its in-plane translation subspace columns
    rotate with the rotation coordinate, so they are configuration dependent and
    contribute velocity-product bias terms `cJ` (reusing the universal joint's
    mechanism). Verified by the closed-form mass matrix and gravity at the zero
    configuration and by the Christoffel-symbol Coriolis cross-check with a
    nonzero link offset.
  - Added experimental ball-joint (3-DOF) and free-joint (6-DOF) forward
    dynamics with manifold integration, which also enables a floating base
    (model it as a fixed base link connected to the moving link by a `Free`
    joint). Both use body-twist generalized velocities (the ball uses the body
    angular velocity; the free joint uses `[linear; angular]` to match its
    `[translation; rotation vector]` position layout), so their motion subspaces
    are constant. Orientation is integrated on SO(3)/SE(3) via the exponential
    map instead of `q += qdot*dt`. Verified by the closed-form mass matrix and
    gravity (ball spherical pendulum), torque-free isotropic spin, free-fall,
    and combined translate-and-spin closed-form integration.
  - Added experimental two-sided link-vs-dynamic-rigid-body contact: a multibody
    link contacting a dynamic rigid body now applies the equal-and-opposite
    contact impulse (normal + two-tangent Coulomb friction) to that body's
    velocity, coupling the rigid body's inverse mass/inertia into the
    articulated contact solve (an immovable obstacle reduces to the existing
    one-sided case). Verified by total linear-momentum conservation when a
    moving link strikes a free body. Link-vs-link and a coupled simultaneous
    boxed-LCP over all contacts remain future work.
  - Added experimental link center-of-mass offset: `Link::setCenterOfMass`/
    `getCenterOfMass` (dartpy `link.center_of_mass`) place a link's center of
    mass away from its link-frame origin, with the inertia tensor taken about
    the center of mass. The articulated-body spatial inertia now uses the full
    COM-coupled form, so a link no longer has to sit at its center of mass
    (matching legacy DART and easing model loading). Verified by an offset-COM
    pendulum matching the parallel-axis mass matrix, gravity torque, and
    acceleration.
  - Added an experimental C++ `simulation::experimental::io::addSkeleton`
    bridge that translates already-parsed legacy `dynamics::Skeleton` trees into
    experimental multibodies for the Weld/Revolute/Prismatic/Screw/Universal/
    Ball/Planar/Free tree-joint families, preserving names, root anchors, joint
    transforms/state/limits/passive properties, mass, inertia, and local COM
    offsets, plus one centered collidable Box/Sphere/Capsule/Cylinder/Mesh
    collision shape per link when that legacy geometry maps exactly to the
    experimental `CollisionShape` facade. URI-loading overloads now accept
    explicit `dart::io::ReadOptions`, including dartpy `ReadOptions` bindings
    for format selection, SDF default root-joint selection, and URDF package
    directories.
    The dartpy `CollisionShape` facade now exposes the cylinder and mesh types,
    constructors, mesh vertices, and triangle indices. The bridge is exposed to
    dartpy as `dartpy.simulation_experimental.add_skeleton()` with
    `SkeletonLoadOptions` for both already-parsed Skeleton objects and URI
    strings that use the default `dart::io::readSkeleton()` reader
    configuration. C++ `addWorld` and dartpy `add_world()` now compose the same
    importer over every Skeleton in an already-parsed or URI-loaded legacy
    World. Parser-specific options, remaining legacy-only joint families,
    offset/multiple/visual shape import, diagnostics, and richer load-result
    ergonomics remain future model-loading
    work.
  - Added experimental generalized-coordinate dynamics accessors on `Multibody`:
    `getMassMatrix`/`getInverseMassMatrix`, `getCoriolisForces`,
    `getGravityForces`, and `getCoriolisAndGravityForces` (dartpy `mass_matrix`,
    `inverse_mass_matrix`, `coriolis_forces`, `gravity_forces`,
    `coriolis_and_gravity_forces`). The terms reuse the forward-dynamics
    recursive Newton-Euler computation and satisfy `M qddot + C + g = tau`.
  - Added an experimental inverse-dynamics accessor on `Multibody`:
    `computeInverseDynamics(qddot)` (dartpy `compute_inverse_dynamics`) returns
    the generalized joint forces `tau = M qddot + C qdot + g` (including joint
    armature) via the recursive Newton-Euler algorithm.
  - Added an experimental generalized impulse-response primitive on `Multibody`:
    `computeImpulseResponse(f)` (dartpy `compute_impulse_response`) returns the
    generalized velocity change `dqdot = M^-1 f` — the joint-space building
    block for impulse-based constraint dynamics.
  - Added experimental link Jacobians: `Multibody::getJacobian(link)`
    (body-frame) and `getWorldJacobian(link)` (world axes, link-origin
    referenced) (dartpy `get_jacobian`/`get_world_jacobian`) return the 6 x DOF
    spatial Jacobian mapping the generalized velocity to the link's spatial
    velocity `[angular; linear]`. Both are derived from the joint configuration
    (and the base world transform for the world frame); center-of-mass
    Jacobians are not yet provided.
  - Added experimental joint position limits: `Joint::setPositionLimits` with
    `getPositionLowerLimits`/`getPositionUpperLimits` (dartpy
    `set_position_limits`, `position_lower_limits`, `position_upper_limits`).
    The articulated-body integration enforces them as hard stops, clamping the
    coordinate and arresting the velocity driving it past a limit.
  - Added experimental joint velocity and effort (force/torque) limits:
    `Joint::setVelocityLimits`/`getVelocityLowerLimits`/`getVelocityUpperLimits`
    and `Joint::setEffortLimits`/`getEffortLowerLimits`/`getEffortUpperLimits`
    (dartpy `set_velocity_limits`/`velocity_lower_limits`/`velocity_upper_limits`
    and `set_effort_limits`/`effort_lower_limits`/`effort_upper_limits`). The
    forward dynamics clamps the commanded joint effort before solving and clamps
    the generalized velocity each step.
  - Added experimental joint armature (rotor/reflected inertia):
    `Joint::setArmature`/`getArmature` (dartpy `joint.armature`). Armature is
    added to the joint-space mass-matrix diagonal in the forward dynamics and in
    the public mass-matrix accessor, improving integration stability for stiff
    geared actuators (an improvement over the legacy DART rigid-body API, which
    lacks armature).
  - Added experimental Coulomb (dry) joint friction:
    `Joint::setCoulombFriction`/`getCoulombFriction` (dartpy
    `joint.coulomb_friction`). The forward dynamics applies a bounded
    velocity-level friction impulse per coordinate that holds the joint at rest
    while the driving effort stays within the bound (stiction) and otherwise
    opposes motion at the friction magnitude (kinetic).
  - Added experimental joint actuator types: `Joint::setActuatorType`/
    `getActuatorType` with `ActuatorType` (dartpy `joint.actuator_type`,
    `ActuatorType`). `Force` (default) applies the commanded joint effort,
    `Passive` ignores it (passive spring/damping/friction still apply), and
    `Velocity` drives the joint to its commanded velocity
    (`Joint::setCommandVelocity`, dartpy `joint.command_velocity`) via a coupled
    velocity-level equality constraint `lambda = (J M^-1 J^T)^-1 (target - J
qdot)` that reaches the target exactly even under inertial coupling. The
    remaining modes (`Servo`/`Acceleration`/`Locked`/`Mimic`) are reserved and
    rejected by the forward dynamics until the full constraint solver lands.
  - Added experimental joint passive dynamics: per-coordinate spring stiffness
    with rest position and damping coefficient contribute restoring/dissipative
    generalized forces in the articulated-body forward dynamics
    (`Joint::getSpringStiffness`/`setSpringStiffness`,
    `getRestPosition`/`setRestPosition`,
    `getDampingCoefficient`/`setDampingCoefficient`; dartpy
    `joint.spring_stiffness`, `joint.rest_position`, `joint.damping_coefficient`).
  - Added experimental rigid-body collision queries: attach a `CollisionShape`
    (sphere, box, capsule, cylinder, plane, or triangular mesh) to a rigid body and call
    `World::collide()` to get contact points (position, world-frame normal,
    penetration depth) via the maintained native collision engine. dartpy adds
    `CollisionShape`, `CollisionShapeType`, `Contact`,
    `body.set_collision_shape`/`collision_shape`/`has_collision_shape`, and
    `world.collide()`.
  - Generalized experimental collision queries to multibody links: links can now
    carry collision shapes (`Link::setCollisionShape`/`getCollisionShape`/
    `hasCollisionShape`, dartpy `link.set_collision_shape`/`collision_shape`/
    `has_collision_shape`) and participate in `World::collide()`, posed by their
    forward-kinematics world transform. `Contact` now references a `CollisionBody`
    handle (a rigid body or a link) via `body_a`/`body_b`, with
    `name`/`is_rigid_body`/`is_link`/`as_rigid_body`/`as_link`. The rigid-body
    contact response stage skips link pairs (their response is a later slice).
  - Experimental rigid bodies and links can now carry multiple collision shapes
    as compound geometry. `setCollisionShape` remains a replace-with-one API,
    while `addCollisionShape` appends shapes and `getCollisionShapes` returns
    the full list (dartpy: `add_collision_shape`, `collision_shapes`). Model
    loading imports every representable collision shape node on a body, binary
    serialization stores the full shape list, and `World::collide()` skips
    same-entity shape pairs so compound pieces do not self-collide.
  - Added `CollisionQueryOptions` for experimental collision queries, including
    an explicit `includeSameMultibodyLinkPairs` / dartpy
    `include_same_multibody_link_pairs` switch to filter same-multibody link
    self-collision pairs without changing the default query behavior.
    Collision queries can also independently include or exclude rigid-body,
    rigid-body/link, and link/link body-type pairs while keeping the default
    all-pairs behavior.
  - `World::collide()` now uses the native collision broad phase to prune
    candidate pairs before narrow-phase contact generation while preserving the
    same `Contact` reporting semantics.
  - `World::collide()` now keeps a persistent native collision world between
    queries, updating cached object transforms and rebuilding only when the
    experimental collision geometry topology changes.
  - Added an experimental articulated contact response: multibody links with
    collision shapes now rest on static rigid-body obstacles. `World::step()`
    resolves each link-vs-static contact with a unilateral normal impulse using
    the contact-point normal Jacobian and `m_eff = 1 / (Jn M^-1 Jn^T)`, plus a
    Baumgarte bias to remove residual penetration (reusing the constraint-solve
    machinery), plus accumulated-impulse two-tangent Coulomb friction bounded by
    the friction cone and per-contact restitution. A fixed-base prismatic "leg"
    with a sphere drops under gravity and rests on the ground, a sliding link is
    braked to rest by friction, and a dropped link bounces off a near-elastic
    ground. Link-vs-dynamic/link-vs-link contacts and a boxed-LCP for coupled
    contacts are still pending.
  - Added an experimental rigid-body contact response: `World::step()` now
    resolves contacts between free rigid bodies with sequential normal impulses
    (frictionless, fully inelastic) plus a positional correction that prevents
    resting bodies from sinking. Rigid-body integration is split into velocity
    and position stages so the contact solve runs at the velocity level between
    them.
  - Added an experimental static rigid-body convention (`RigidBodyOptions::isStatic`,
    `RigidBody::setStatic`/`isStatic`, dartpy `is_static`): static bodies ignore
    gravity and forces, are not integrated, and are treated as immovable by the
    contact solver, so a dynamic body dropped onto a static ground comes to rest.
  - Added experimental rigid-body contact restitution and Coulomb friction:
    `RigidBody::setRestitution`/`getRestitution` and `setFriction`/`getFriction`
    (dartpy `restitution`/`friction`). The contact solver now uses accumulated
    normal impulses with restitution (a perfectly elastic equal-mass head-on
    collision swaps velocities) and a two-tangent friction-pyramid Coulomb model
    bounded by the normal impulse (a body sliding on a static ground decelerates
    and stops).
  - Added an experimental deformable-body world stage with grid-mesh body
    construction, explicit mass-spring elastic integration, IPC-style
    ground-barrier contact with feasibility line search, rigid-body barrier
    opt-in controls, a Filament GUI example for long-horizon visual inspection,
    and focused unit/benchmark coverage.
  - Added experimental deformable mesh/material state for surface triangles,
    tetrahedra, positive rest volumes, material validation, density-based
    tetrahedral mass assembly, serialization, and GUI surface rendering.
  - Added a contact-free experimental deformable scene loader/replay
    foundation for upstream-style tetra mesh scene records, scripted
    Dirichlet/Neumann boundary conditions, restart diagnostics, benchmarks, and
    headless GUI scene capture.
  - Added internal experimental IPC primitive-distance kernels for
    point-triangle and edge-edge squared distances, closest-feature
    classification, derivative regression tests, edge-edge mollifier
    derivatives, and a microbenchmark surface for future deformable contact
    assembly.
  - Replaced internal experimental IPC finite-difference distance Hessian
    placeholders with feature-wise analytic point-triangle and edge-edge
    Hessian paths, reducing local Hessian microbenchmark timings for future
    deformable contact assembly.
  - Added internal experimental IPC clamped-log barrier kernel scaffolding for
    point-triangle and edge-edge primitive candidates, including raw analytic
    gradients/Hessians, edge-edge mollifier product-rule derivatives, regression
    tests, and microbenchmark coverage.
  - Added internal experimental IPC tangent-stencil scaffolding for
    point-triangle, edge-edge, point-edge, and point-point friction/contact
    stencils, including upstream-style tangent bases, closest-point parameters,
    tangent projection matrices, metric matrices, regression tests, and
    microbenchmark coverage.
  - Added internal experimental IPC contact candidate assembly for
    point-triangle and edge-edge primitive pairs with deterministic surface-edge
    extraction, incident/adjacent filtering, sweep-versus-brute-force
    regression tests, and indexed/brute-force benchmark counters.
  - Added internal experimental IPC motion-aware candidate culling over
    start/end swept primitive AABBs, preserving fast point-triangle and
    edge-edge crossings that static endpoint candidate filters miss, with
    regression tests and benchmark counters for future CCD line-search wiring.
  - Added reusable internal IPC contact-candidate builder overloads that clear
    stale candidate state while preserving candidate vector capacity, with
    regression tests and return-wrapper versus reusable-buffer benchmark
    counters for future solver-owned contact buffers.
  - Wired internal IPC motion-aware surface-contact candidates into the
    deformable line search as a conservative per-body CCD limiter, with
    primitive-CCD uncertainty status, no-edge surface-contact regressions, and
    focused benchmark counters.
  - Added reusable internal sweep-item scratch for IPC static and motion-aware
    contact candidate builders and wired the deformable surface-contact line
    search to reuse it per body.
  - Optimized internal IPC cross-set sweep-pair traversal used by experimental
    surface candidate and CCD checks so the sorted right-hand-side AABB prefix
    already expired for the current left-hand-side AABB is skipped.
  - Completed the internal IPC cross-set sweep traversal into a full active-set
    sweep-and-prune: right-hand-side AABBs are kept in a reusable next-index
    live list and unlinked as soon as their maximum x falls behind the sweep
    line, so a long-lived early interval no longer pins the prefix cursor and
    forces the expired intervals behind it to be rescanned for every
    left-hand-side item (the prior slice's known limitation). The visited
    `(lhs, rhs)` pair sequence is identical to the naive nested scan, so the
    surface candidate sets, solver-wired CCD limiters, and barrier candidate
    assembly are behavior-preserving. Adds a staggered-expiry active-set
    regression and a long-lived-interval microbenchmark
    (`BM_IpcCandidateSetCrossSweepLongLivedInterval`).
  - Added internal inter-body deformable surface CCD line-search limiting with
    stage-start surface snapshots, cross-body point-triangle and edge-edge
    conservative CCD checks, focused regressions, and benchmark counters.
  - Added internal static-ground-barrier CCD line-search limiting for
    point-mass deformable nodes sweeping against explicitly opted-in static
    rigid ground barriers, with continuous finite-footprint regressions and
    benchmark counters.
  - Added experimental static-box surface CCD obstacle opt-in
    (`RigidBody::setDeformableSurfaceCcdObstacle`/`isDeformableSurfaceCcdObstacle`,
    dartpy `is_deformable_surface_ccd_obstacle`) for the deformable
    line-search limiter, with point-triangle and physical box-edge CCD
    regressions and benchmark counters. This is a CCD limiter only, not rigid
    contact response or IPC parity.
  - Added an opt-in experimental rigid IPC world-step stage that builds
    mesh-like rigid surfaces and physical dynamics objective terms from runtime
    rigid-body state, runs the internal projected-Newton barrier solve, and
    writes solved poses/velocities back without replacing the default rigid
    contact stage.
  - Added experimental `World::setRigidBodySolver()` selection so callers can
    keep the default sequential-impulse rigid pipeline or opt into the rigid
    IPC free-rigid dynamics stage without exposing solver registries.
  - Added deterministic runtime sphere triangulation for the experimental rigid
    IPC world-step stage, allowing analytic sphere collision shapes to
    participate in the internal mesh-like barrier surface assembly.
  - Added durable diagnostics to the opt-in experimental rigid IPC world-step
    stage, reporting solve status, last step and line-search bounds, and
    aggregate conservative CCD line-search counters.
  - Added the first activated-contact runtime regression for the opt-in
    experimental rigid IPC world-step stage, verifying mesh barriers move a
    dynamic body away from a static surface while reporting line-search
    diagnostics.
  - Added point-point curved CCD to rigid IPC conservative line search so
    vertex-vertex barrier rows can limit unsafe steps and report point-point
    diagnostics.
  - Hardened the opt-in experimental rigid IPC world-step stage so malformed
    runtime mesh topology, non-finite mesh vertices, and invalid box extents
    are skipped before internal barrier assembly or CCD.
  - Made the opt-in experimental rigid IPC world-step stage skip non-converged
    solve results instead of applying partial runtime poses silently, with
    diagnostics for result application.
  - Added internal lagged smoothed Coulomb friction potentials for rigid IPC
    vertex-vertex, edge-vertex, edge-edge, and face-vertex contacts, including
    world-coordinate derivative coverage and reduced-coordinate coverage for
    vertex-vertex and edge-vertex terms.
  - Assembled the first internal lagged friction rows into the rigid IPC
    projected-Newton objective using lagged active barrier constraints, per-body
    friction coefficients, and runtime active-friction diagnostics.
  - Added bounded outer lagged-friction passes to the internal rigid IPC
    projected-Newton solve, including a zero-iteration friction disable,
    refreshed momentum-balance diagnostics, and runtime active-pass counts.
  - Added sufficient-decrease backtracking to the internal rigid IPC
    projected-Newton solve. Feasible Newton candidates are now checked against
    the assembled objective after conservative CCD scaling, with diagnostics for
    objective checks/backtracks exposed through the opt-in runtime stage and a
    safe decreasing-candidate fallback for lagged-friction active-set changes.
  - Added a runtime regression proving lagged rigid IPC friction observably
    brakes a tangential slide at an activated mesh contact relative to the
    frictionless solve, and reports active friction constraints/passes.
  - Added a rigid IPC fixture replay regression proving parsed fixture friction
    metadata carries into the opt-in runtime IPC stage: identical replayed
    inline-polygon scenes now differentiate frictionless and frictional slide
    while reporting active friction diagnostics.
  - Added `RigidIpcContactStageOptions` for the opt-in experimental rigid IPC
    stage, allowing callers to configure max iterations, barrier activation
    distance, and lagged-friction passes without exposing a solver registry.
    Fixture replay coverage now applies parsed `dHat` and friction-iteration
    metadata to that stage configuration.
  - Extended `RigidIpcContactStageOptions` with lagged-friction static-speed
    and convergence-tolerance controls, allowing fixture/comparison `epsv` and
    velocity-tolerance metadata to drive the opt-in runtime IPC friction solve.
  - Made rigid IPC fixture replay populate parsed kinematic bodies as runtime
    kinematic rigid bodies, so the opt-in IPC stage advances their prescribed
    linear and angular motion instead of treating them as immovable static
    bodies.
  - Added an internal rigid IPC fixture-to-stage-options bridge so replay
    drivers can consistently apply parsed barrier, friction, and absolute
    convergence-tolerance metadata to the opt-in runtime IPC stage.
  - Added an internal one-step rigid IPC fixture runtime replay helper that
    populates an experimental `World`, applies parsed fixture stage policy, runs
    one opt-in IPC contact-stage step, and can return solver diagnostics for
    manifest drivers.
  - Hardened the internal rigid IPC interval-root CCD evaluator so accepted root
    boxes that start at `t=0` no longer report a zero-time hit when the input
    row starts separated, matching the audited kinematic CCD rows' guard.
    The generated rigid IPC fixture manifest now marks
    `tests/data/kinematic/ccd-test-000..012.json` as implemented by that
    regression.
  - Added tracked wrecking-ball direct-CCD coverage for
    `tests/data/wrecking-ball/ccd-test-000..385.json`, preserving the upstream
    conservative-TOI check that a replay truncated at DART's reported impact
    bound does not report another hit. The generated rigid IPC fixture manifest
    now marks those rows as implemented by that regression.
  - Extended rigid IPC fixture mesh replay to legacy VTK unstructured-grid
    surface meshes, covering the remaining mesh extension present in the
    audited upstream rigid-ipc corpus while preserving native mesh collision
    shape replay semantics.
  - Added the first rigid IPC performance benchmark (`bm_rigid_ipc_solver`)
    covering the per-primitive reduced barrier kernels, scene-level assembly,
    the projected-Newton solve, and the conservative CCD line search, with a
    benchmark methodology and baseline tracker. Initial measurements show the
    scene assembly is O(N^2) (no broad phase yet).
  - Optimized rigid IPC barrier assembly with a conservative world-AABB
    broad-phase cull that skips surface pairs already beyond the activation
    distance. The cull is behavior-preserving (covered by an equivalence
    regression) and drops the measured scene-assembly complexity from O(N^2) to
    O(N) (~4.4x faster at 32 bodies, widening with scene size).
  - Added a same-scene per-step comparison benchmark of the incumbent
    sequential-impulse rigid path against the opt-in rigid IPC path via
    `World::setRigidBodySolver()`, establishing the DART rigid baseline to beat.
    The rigid IPC scaffold is currently ~3 orders of magnitude slower per step,
    quantifying the optimization gap.
  - Optimized the rigid IPC conservative line search with a swept broad-phase
    cull that skips surface pairs whose start-pose AABBs are farther apart than
    the bodies can move over the step, reusing the curved-trajectory speed bound
    so it stays consistent with the per-primitive CCD (including a rotational
    motion bound, the CCD convergence tolerance, and the minimum separation).
    The cull is behavior-preserving, with anti-tunneling, far-skip, and
    tolerance-band regressions.
  - Replaced the rigid IPC barrier-assembly and line-search all-pairs O(N^2)
    surface enumeration with a sort-and-sweep broad phase that reuses the
    deformable IPC sweep utilities (shared IPC primitives), keeping the exact
    distance/reach cull on the sweep candidates so the assembled system and
    line-search result are unchanged while large multi-body scenes become
    sub-quadratic.
  - Added a `sx_rigid_ipc` Python demo scene (registered in the py-demos
    Experimental category) showing a free box settle on static ground through
    the opt-in rigid IPC barrier solver (`World.rigid_body_solver = IPC`).
  - Fixed the rigid IPC freeze-on-contact "sink-then-stick" failure by adding
    IPC adaptive barrier stiffness (porting the reference
    `initial_barrier_stiffness`/`update_barrier_stiffness` onto DART's
    squared-distance clamped-log barrier). A fixed `kappa = 1` was orders of
    magnitude too soft, so a body under gravity crept into the barrier band
    until a step penetrated and the conservative line search then blocked every
    subsequent step, freezing the body permanently. The solve now picks an
    initial `kappa` that balances the barrier gradient against the inertial
    energy gradient, clamped to `[kappa_min, 100*kappa_min]`, and doubles it
    when the closest pair keeps approaching. A box on static ground now produces
    continued contact dynamics (slides and is friction-braked toward rest) while
    staying intersection-free, instead of sticking. Added a relative
    projected-Newton gradient-convergence floor (the stiff barrier makes the
    absolute tolerance unreachable at a resting contact) and made the opt-in
    stage apply the best intersection-free configuration a bounded solve reaches
    (matching the reference, which steps with the optimizer's best feasible
    iterate) instead of discarding any not-fully-converged result. The
    anti-tunneling guarantee is unchanged: unsafe failed solves are still never
    written back. Covered by adaptive-stiffness unit tests and a no-freeze
    sliding-contact runtime regression.
  - Hardened dense exact-contact resting plateaus in the opt-in rigid IPC
    runtime stage. A zero-step line-search block now gives adaptive kappa a
    bounded stiffness-increase retry before failing, the stage carries raised
    kappa forward across active-contact runtime steps, and an exact
    zero-progress resting-contact plateau writes back the unchanged safe pose
    instead of reporting a persistent failed solve. The Fig. 11 arch regression
    now uses five voussoirs and asserts the stage does not fail while the arch
    remains intersection-free.
  - Added a `sx_rigid_ipc_slide` Python demo scene (registered in the py-demos
    Experimental category) showing a box slide across static ground and be
    friction-braked to rest through the rigid IPC barrier solver, now viable
    thanks to the freeze-on-contact fix.
  - Added a two-box-stack runtime regression proving the adaptive-stiffness fix
    generalizes to multiple dynamic bodies and body-body contact: a stack of two
    free boxes on static ground settles into a stable, intersection-free
    equilibrium (simultaneous body-ground and body-body barriers, a single
    scene-level adaptive kappa) instead of freezing, interpenetrating, or
    separating.
  - Expanded the rigid IPC py-demos suite (Experimental category) with three
    more real-time scenes, each verified to behave correctly (no freeze,
    penetration, or explosion) and to run at an interactive frame rate:
    `sx_rigid_ipc_incline` (a box sliding down a tilted ramp under friction),
    `sx_rigid_ipc_pile` (boxes dropped into a pile, multi-body contact), and
    `sx_rigid_ipc_tunnel` (a fast box stopped dead by a thin wall, showcasing the
    intersection-free / no-tunneling guarantee). Heavier contact scenes (a
    triangulated sphere, a tight box stack) are intentionally not shipped as GUI
    demos yet because the rigid IPC solver currently runs at only a few frames
    per second for those; that capability stays covered by C++ regressions, and
    the demos will follow once the rigid IPC performance work lands.
  - Extended the static rigid surface CCD obstacle opt-in to static SPHERE
    collision shapes (PLAN-081 Phase 2 non-box rigid surface coverage). An
    opted-in static sphere is tessellated into a UV-sphere triangle mesh that
    conservatively CIRCUMSCRIBES the analytic sphere (vertices placed at a
    slightly inflated radius so every flat face stays outside the true surface),
    so the existing point-triangle / edge-edge deformable CCD limiter stops a
    deformable at the sphere without a new CCD primitive and never lets it
    penetrate the real surface. Reuses the same `setDeformableSurfaceCcdObstacle`
    opt-in (untagged spheres and boxes are unaffected, so existing scenes are
    behavior-preserving) and reports a new `staticRigidSurfaceCcdSphereCount`
    stat. Adds regressions that a fast node is limited before an opted-in sphere
    and passes through an untagged one. Still a one-way conservative CCD limiter,
    not rigid contact response or IPC parity.
  - Added internal experimental moving rigid box surface CCD limiting for free
    (non-static) deformable-surface CCD obstacles: the deformable stage predicts
    each obstacle's end-of-step transform from its velocity (mirroring the rigid
    position integrator that runs after the deformable stage) and tiles the
    swept motion with overlapping static pose samples so the deformable cannot
    settle in or tunnel through the obstacle's swept corridor, with focused
    regressions and benchmark counters. This is a one-way conservative CCD
    limiter only (timing-agnostic, no rigid contact response or two-way
    coupling) and is not IPC parity.
  - Added internal experimental IPC self-contact barrier forces: the deformable
    solve now adds the clamped-log barrier energy/gradient over the active
    self-contact point-triangle and edge-edge candidate set (assembled per outer
    iteration within the activation distance d_hat) as a new objective term, so
    a deformable surface folding onto itself experiences smooth repulsive
    contact forces and settles near d_hat instead of pinning at the CCD
    minimum separation. The conservative CCD limiters remain the hard
    no-penetration guarantee. First-order (steepest-descent) solve with fixed
    barrier stiffness; projected Newton and adaptive stiffness are later slices.
    Includes focused regressions and a benchmark with barrier counters.
  - Added experimental IPC barrier forces for static sphere obstacles (PLAN-081
    Phase 3). A static rigid sphere opted in as a deformable surface-CCD obstacle
    now exerts a full radial clamped-log barrier force: each deformable node
    within the activation band of the sphere's surface is pushed out along the
    outward radial normal, so a deformable settles smoothly against any side of
    the sphere -- a true 3D contact force, unlike the vertical-only ground
    barrier. Reuses the existing `setDeformableSurfaceCcdObstacle` opt-in
    (untagged shapes, boxes, and non-obstacle scenes are unchanged, so existing
    behavior is preserved; on the prior code surface-CCD-obstacle spheres were a
    no-op). Energy + gradient only -- the projected-Newton Hessian, box and
    codimensional obstacles are later increments; the line search on the
    barrier-inclusive energy keeps slowly-approaching nodes outside, and the
    surface CCD limiter remains the tunnelling guard for fast motion. Adds
    regressions that a node in the band is pushed radially outward and that an
    untagged sphere is inert.
  - Added the projected-Newton Hessian for the static sphere obstacle barrier
    (PLAN-081 M2 increment), making obstacle contact a first-class Newton term
    rather than relying on the steepest-descent fallback. The full per-node
    barrier Hessian's tangential eigenvalues are non-positive, so its PSD
    projection is the rank-1 radial curvature `max(0, B''(d)) n n^T` along the
    outward normal -- the sphere analogue of the vertical ground-barrier
    curvature. Equilibrium-preserving (the Hessian changes only the search
    direction), so the existing sphere obstacle regressions are unchanged. Adds
    a `FemCubeSettlesOnSphereObstacleWithoutPenetrating` regression (a FEM cube
    dropped onto a sphere obstacle settles intersection-free) and a
    `Deformable FEM over Sphere (IPC)` py-demos scene (a FEM slab draping over a
    sphere obstacle), exercising FEM elasticity with the sphere + ground
    barriers together.
  - Added a clamped-log barrier force (energy + gradient + projected-Newton
    Hessian) for static oriented BOX obstacles (PLAN-081 M2 increment), the box
    analogue of the sphere obstacle barrier. The closest point on the box surface
    -- and hence the surface distance and outward normal -- is found by clamping
    the node into the box's local frame, which handles face, edge, and corner
    contact uniformly; the barrier pushes nearby deformable nodes out along that
    normal, and its PSD-projected Hessian is the rank-1 radial curvature along
    it. Reuses the existing `setDeformableSurfaceCcdObstacle` opt-in (the box CCD
    limiter stays the tunnelling guard; box obstacles previously had no smooth
    contact force); purely additive (all existing box surface-CCD regressions
    unchanged). Adds `BoxObstacleBarrierRepelsNodeAlongFaceNormal` and
    `FemCubeSettlesOnBoxObstacleWithoutPenetrating` regressions and a
    `Deformable FEM over Box (IPC)` py-demos scene (a FEM slab draping over a box
    obstacle).
  - Added an opt-in **barrier-only obstacle** mode that completes mesh-vs-obstacle
    friction for sphere and box obstacles (PLAN-081 M5). `RigidBody`'s new
    `setDeformableObstacleBarrierOnly` (dartpy
    `is_deformable_obstacle_barrier_only`) excludes a deformable obstacle from the
    surface-CCD line-search limiter while keeping its clamped-log contact barrier:
    the CCD otherwise scales the _whole_ step (normal + tangential) to prevent
    penetration, which masks tangential sliding and so obstacle friction; the
    barrier alone prevents penetration for the quasi-static contact this targets.
    With the CCD excluded, the sphere/box obstacle barrier normal forces feed the
    lagged Coulomb friction term (`addSphereObstacleNormalForces` /
    `addBoxObstacleNormalForces`, dominant per-node contact wins), so a deformable
    sliding across a barrier-only plate is decelerated by friction. Existing
    CCD-obstacle scenes are unchanged (the mode is opt-in; the CCD obstacle set is
    `entt::exclude`-filtered). Adds a C++ and a dartpy regression (a strip shoved
    across a barrier-only box plate slides far frictionless but is held back under
    friction, staying on the face) and a `Deformable Friction on Box Plate (IPC)`
    py-demos scene.
  - Added **mesh-vs-obstacle friction** against the capsule rod obstacle
    (PLAN-081 M5). A node in contact with a static capsule obstacle now feeds the
    capsule barrier's radial normal force and direction into the existing lagged
    smoothed Coulomb friction term (the dominant per-node contact -- ground or
    capsule -- wins), so a deformable sliding along a rod is decelerated by
    friction proportional to `frictionCoefficient`. Because the capsule obstacle
    is barrier-only (no over-limiting surface CCD), the tangential slide is
    unconstrained and the friction force is effective -- unblocking obstacle
    friction that the sphere/box surface-CCD obstacles still mask. Adds a C++ and
    a dartpy regression (a strip shoved along a rod slides far frictionless but is
    held back under friction, staying on the rod) and a `Deformable Friction on
Capsule Rod (IPC)` py-demos scene.
  - Exposed the iterative-solve count through the public deformable solver
    diagnostics (PLAN-081 M7). `DeformableSolverDiagnostics` (dartpy
    `last_deformable_solver_diagnostics`) now carries
    `projectedNewtonIterativeSolves` / `projected_newton_iterative_solves`, the
    public mirror of the internal stat: it counts the Newton iterations whose
    linear solve took the iterative (incomplete-Cholesky-preconditioned CG) path
    instead of the sparse Cholesky factorization, so callers can observe and tune
    which solve path a body uses (zero means every solve was direct). Adds C++
    and Python regressions that the default direct solve reports zero while an
    opt-in iterative body reports a nonzero count.
  - Extended the public deformable projected-Newton diagnostics with CG effort,
    residual, and sparse-Hessian footprint counters (PLAN-081 M7).
    `DeformableSolverDiagnostics` (dartpy
    `last_deformable_solver_diagnostics`) now also carries
    `projectedNewtonIterativeIterations` /
    `projected_newton_iterative_iterations`,
    `projectedNewtonIterativeMaxError` /
    `projected_newton_iterative_max_error`,
    `projectedNewtonHessianNonZeros` /
    `projected_newton_hessian_nonzeros`, and
    `projectedNewtonHessianStorageBytes` /
    `projected_newton_hessian_storage_bytes`, so benchmark and tuning code can
    distinguish "CG path was used" from "CG converged cheaply" and can track the
    assembled sparse matrix footprint that future matrix-free CG must remove.
    The FEM-bar and chunky 3D cube benchmarks now emit `cg_iters_per_step`,
    `cg_max_error`, `hessian_nonzeros`, and `hessian_storage_bytes` counters
    toward the PLAN-081 Fig. 23 / Table 1 profiling surface.
  - Added an explicit matrix-free deformable projected-Newton CG path
    (PLAN-081 M7). `DeformableMaterialProperties.useMatrixFreeLinearSolver`
    (dartpy `use_matrix_free_linear_solver`) bypasses Eigen `SparseMatrix`
    Hessian assembly and applies local Hessian blocks directly with a
    block-Jacobi preconditioner. The default direct and sparse IC-CG paths are
    unchanged. `DeformableSolverDiagnostics` now reports
    `projectedNewtonMatrixFreeSolves` /
    `projected_newton_matrix_free_solves`; matrix-free benchmark rows emit
    `matrix_free_solves_per_step` and zero sparse-Hessian footprint counters.
    Adds C++ and dartpy ground-contact regressions showing matrix-free CG reaches
    the same contact equilibrium as the direct sparse solve, while the C++ FEM
    cube regression also compares sparse IC-CG against both paths. The
    simulation-experimental binary format is bumped so legacy v8 deformable
    materials load with the new matrix-free flag defaulted off instead of
    consuming the following byte.
  - Added a chunky 3D FEM-cube **direct-vs-iterative scaling benchmark** for the
    experimental deformable solver (PLAN-081 M7). `BM_DeformableCube3dDirectStep`
    and `BM_DeformableCube3dCgStep` step a solid N^3 cube of FEM tetrahedra
    (pinned at one face) through the sparse Cholesky direct solve and the
    incomplete-Cholesky-preconditioned conjugate gradient at matching cell
    counts. Unlike the thin `BM_DeformableFemBarStep` beam (2x2 cross-section,
    too small a bandwidth), the solid cube has a wide Hessian bandwidth, so the
    direct solve's 3D fill-in makes its per-step time climb super-linearly while
    the iterative solve stays near O(nnz): the iterative path overtakes the
    direct solve by a few thousand nodes (on the development machine the per-step
    cost crosses over near ~1k nodes and the iterative solve runs several times
    faster by ~1.3k nodes). This is the per-step-scaling axis of the IPC paper's
    Fig. 23 / Table 1 on the mesh shape where the iterative solver bites. Adds a
    regression that a chunky 3D cube sags to the same equilibrium under both
    solvers (mutually exclusive solve paths -- the iterative run never
    factorizes), guarding the benchmark fixture.
  - Strengthened the experimental deformable iterative solve's preconditioner
    from diagonal (Jacobi) to **incomplete-Cholesky** (PLAN-081 M7). Stiff
    barrier contact makes the projected-Newton Hessian ill-conditioned, where a
    diagonal preconditioner leaves the conjugate gradient stalling against its
    iteration cap and falling back to steepest descent; the incomplete-Cholesky
    preconditioner (a sparse approximate factorization that drops fill, so the
    solve still never fully factorizes and stays near O(nnz)) collapses the CG
    iteration count so the iterative path carries stiff contact within the cap.
    On a settling stiff FEM contact scene this cuts the iterative solver's
    fallbacks below the direct solver's and reduces Newton iterations per step,
    while remaining byte-compatible at the API level (still opt-in via
    `useIterativeLinearSolver`). An incomplete-Cholesky breakdown that Eigen's
    diagonal shifting cannot repair falls back to steepest descent, exactly as
    the direct path does on an indefinite factorization. Adds a regression in
    which a stiff FEM cube settles on a ground barrier through the iterative path
    (never factorizing) with accepted CG steps dominating fallbacks and the same
    equilibrium as the direct solver, plus a `cg_contact` py-demo of a stiff FEM
    cube settling on a barrier via the incomplete-Cholesky CG solve.
  - Added an opt-in **iterative (conjugate-gradient) linear solve** for the
    experimental deformable projected-Newton step (PLAN-081 M7). When a body
    sets `DeformableMaterialProperties.useIterativeLinearSolver` (dartpy
    `use_iterative_linear_solver`), each Newton iteration solves its
    symmetric-positive-definite Hessian with a Jacobi-preconditioned conjugate
    gradient instead of the sparse Cholesky (SimplicialLDLT) factorization. CG
    never factorizes, so its memory stays near O(nnz) and its per-step cost
    grows more gently than direct fill-in as the mesh chunks up -- the scaling
    axis of the IPC paper's Fig. 23 / Table 1. The same inertia floor plus
    PSD-projected element blocks that keep the direct solve well-posed guarantee
    CG convergence; a non-converged or non-finite solve falls back to mass-scaled
    steepest descent, exactly as the direct path does on an indefinite
    factorization. Meshes above the direct-solve node cap now take the iterative
    path automatically (raising the effective solver ceiling from 20k to 1M
    nodes) instead of degrading to gradient descent. Off (the default) is
    byte-identical -- existing scenes keep the direct solve. Adds a regression in
    which a dropped FEM cube settles to the same configuration under both solvers
    while taking mutually exclusive solve paths (the iterative run never
    factorizes), a `cg_solver` py-demo of a large CG-driven cantilever, and a
    `BM_DeformableCgBarStep` benchmark mirroring the direct FEM-bar benchmark for
    per-step scaling comparison.
  - Added opt-in **adaptive barrier stiffness** (kappa) to the experimental
    deformable solver (PLAN-081 M6). When a body sets
    `DeformableMaterialProperties.useAdaptiveBarrierStiffness` (dartpy
    `use_adaptive_barrier_stiffness`), its ground and obstacle (sphere/box/capsule)
    contact barriers scale kappa per step from the mass / time-step force balance
    `kappa = clamp((maxNodalMass / dt^2) * d_hat^2, 25, 1e6)` instead of the fixed
    default. The barrier's curvature contribution scales like `kappa / d_hat^2`
    and a node's inertial stiffness like `mass / dt^2`, so this balance keeps the
    contact equally well-conditioned across mass/stiffness ratios; for a unit
    nodal mass at `dt = 1/250`, `d_hat = 2e-2` it evaluates to exactly the
    historical fixed `kappa = 25`, which the adaptive form generalizes (toward the
    paper's Fig. 12 large-ratio robustness). It is floored at the fixed default
    so contact robustness never regresses and capped to keep the Hessian
    well-conditioned. Off (the default) is byte-identical, so every existing scene
    is unchanged; the lagged-friction normal-force estimate and the self-contact
    barrier (which carries its own stiffness) keep the fixed kappa. Adds a
    regression in which a heavy node settles measurably higher (the stiffer
    adaptive barrier balances gravity farther from the surface, mass-independently)
    than with the fixed kappa.
  - Added a capsule (rod/wire) collision shape and a static **capsule obstacle
    barrier** for the experimental deformable solver (PLAN-081 M3 codimensional
    collision objects). `CollisionShape::makeCapsule(radius, halfHeight)` adds a
    `Capsule` shape type (a body-z-axis segment swept by a radius; exposed to
    dartpy as `CollisionShape.capsule`). A static capsule opted in as a
    deformable obstacle exerts the same clamped-log barrier (energy, gradient,
    and rank-1 radial Hessian) as the sphere/box obstacles, with the surface
    distance and outward normal taken from the analytic point-to-segment closest
    point (`|node - closestOnAxis| - radius`). Unlike the sphere/box surface-CCD
    obstacles the capsule is barrier-only, so a connected sheet drapes over it
    freely. Adds a `CapsuleObstacleBarrierRepelsNodeRadially` solver regression,
    a draped-cloth intersection-free regression, and a `Deformable Cloth over
Capsule Rod (IPC)` py-demos scene (a cloth draping over a horizontal rod,
    toward the paper's Fig. 18 codimensional-roller theme). The native collision
    engine approximates a capsule by its bounding box for rigid-rigid queries.
  - Added `.seg` (segment) and `.pt` (point) codimensional-geometry importers
    (PLAN-081 M3 asset pipeline), completing the experimental importer set
    alongside `.msh` (tets) and `.obj` (triangles).
    `io::loadSegLineMesh*` parses Wavefront-style `v` vertices and `l` polylines
    (each `l v1 v2 ... vk` contributes the k-1 consecutive segments), and
    `io::loadPointSet*` parses bare `x y z` or `v x y z` point lines; both resolve
    1-based / negative indices and raise `InvalidArgumentException` on malformed
    or empty input. Exposed to `dartpy` as `load_seg_line_mesh(path)` (positions +
    spring edges -> a mass-spring strand) and `load_point_set(path)` (positions ->
    free particles), with `build_strand_from_seg` / `build_particles_from_pt`
    helpers, importer regressions, a draped-strand / falling-particles solver
    regression, and `Deformable .seg Strand (IPC)` and `Deformable .pt Particles
(IPC)` py-demos scenes (a bundled strand hanging from a pinned end; a bundled
    point cloud stacking on the ground barrier).
  - Added a Wavefront `.obj` triangle-mesh importer (PLAN-081 M3 asset pipeline)
    so deformable surface bodies can be built from external triangle meshes.
    `dart::simulation::experimental::io::loadObjTriangleMesh` /
    `loadObjTriangleMeshFile` parse `v` vertices and `f` faces (each face vertex
    token may be `v`, `v/vt`, `v/vt/vn` or `v//vn`; 1-based and negative indices
    are resolved; polygon faces are fan-triangulated), ignoring normals /
    texcoords / groups / materials and raising `InvalidArgumentException` on a
    malformed or out-of-range face. Exposed to `dartpy` as
    `load_obj_triangle_mesh(path)` returning a `DeformableBodyOptions` (positions
    - surface triangles). Adds parse / triangulation / rejection regressions, a
      `build_cloth_from_obj` helper (derives unique-edge springs + uniform masses),
      a draped-cloth solver regression, and a `Deformable .obj Cloth (IPC)`
      py-demos scene (a bundled cloth mesh pinned at one edge draping under gravity
      onto the ground barrier).
  - Added a GMSH `.msh` tetrahedral-mesh importer (PLAN-081 M4) so deformable
    FEM bodies can be built from external tet meshes rather than only procedural
    grids. `dart::simulation::experimental::io::loadGmshTetMesh` /
    `loadGmshTetMeshFile` parse GMSH ASCII format 2.x and 4.x (both the legacy
    flat layout and the entity-block layout): the `$Nodes` and the 4-node
    tetrahedron (`$Elements` type 4) connectivity, remapping node ids to
    0-based indices and ignoring non-tetra elements, raising
    `InvalidArgumentException` on malformed or unsupported (binary, or
    version &lt; 2 / &ge; 5) input. Exposed to `dartpy` as
    `load_gmsh_tet_mesh(path)` returning a
    `DeformableBodyOptions` (positions + tetrahedra). Adds parse / rejection /
    FEM-simulation regressions and a `Deformable FEM from .msh (IPC)` py-demos
    scene that loads a bundled tet-bar mesh and sags it as an FEM cantilever.
  - Added stable neo-Hookean tetrahedral FEM elasticity to the experimental
    deformable solver (PLAN-081 M1, the paper-parity keystone). A new
    header-only `deformable_elasticity/fem_tet_element.hpp` kernel produces, per
    tetrahedron, the stable neo-Hookean (Smith et al. 2018) strain energy, its
    12x1 nodal gradient, and the true 12x12 Hessian, validated by
    finite-difference gradient/Hessian tests (plus rest-state zero force,
    inversion-robustness, and zero-Poisson stability). The solver wires it in as
    an opt-in elasticity model via the new
    `DeformableMaterialProperties.useFiniteElementElasticity` flag (also exposed
    to `dartpy` as `use_finite_element_elasticity`): when set, each tetrahedron
    contributes its energy/gradient to the objective and its PSD-projected 12x12
    Hessian to the sparse projected-Newton assembly through the same batched
    projection seam as the spring and barrier blocks. The model is inversion-safe
    (finite for every deformation gradient, including det F <= 0). Default (flag
    off) leaves the mass-spring path byte-identical -- all existing deformable
    regressions are unchanged. Adds solver regressions (a pinned FEM tetrahedron
    resists gravity where a springless body free-falls; stationary at rest;
    restores a perturbed node toward rest), a `BM_DeformableFemBarStep`
    benchmark, and a `Deformable FEM Bar (IPC)` py-demos scene (a tetrahedral
    cantilever sagging under gravity) in the `IPC Deformable (sx)` category.
  - Added the fixed-corotational (FCR) tetrahedral FEM material to the
    experimental deformable solver (PLAN-081 M1 follow-up), the IPC paper's
    other isotropic elasticity model alongside neo-Hookean. The
    `fem_tet_element.hpp` kernel gains the FCR energy `mu*||F - R||^2 +
(lambda/2)*(J - 1)^2` (with `R` the polar-decomposition rotation), its exact
    first Piola-Kirchhoff stress, and a positive-definite Gauss-Newton element
    Hessian, validated by rest-state zero-force, rotation-invariance, and
    finite-difference gradient / SPD-Hessian tests. It is selected by the new
    opt-in `DeformableMaterialProperties.useFixedCorotationalElasticity` flag
    (also exposed to `dartpy` as `use_fixed_corotational_elasticity`) on top of
    `useFiniteElementElasticity`; the objective and projected-Newton assembly
    dispatch to whichever material is configured through a shared element seam,
    so the default stable neo-Hookean path is unchanged. Adds solver regressions
    (an FCR tetrahedron stationary at rest; restores a perturbed node toward
    rest) and a `Deformable FCR Twist (IPC)` py-demos scene (a tetrahedral bar
    twisted at both ends, untwisting under the fixed-corotational material).
  - Added a `BM_DeformableFcrBarStep` per-step benchmark for the
    fixed-corotational FEM material (mirroring `BM_DeformableFemBarStep` at the
    same cell counts) so the two FEM materials' per-step solver cost can be
    compared at matching mesh resolution -- the per-step-time axis of the IPC
    paper's Fig. 23 / Table 1. Shares one polar-decomposition SVD per element
    between the fixed-corotational energy and stress (the line search evaluates
    both every probe), measurably cutting the per-step cost; the kernel math is
    unchanged (all fixed-corotational tests still pass bit-for-bit). Adds a
    closed-form fixed-corotational energy regression under a uniform scale
    `F = c*I` (`3*mu*(c-1)^2 + (lambda/2)*(c^3-1)^2`), which pins the absolute
    energy value that the finite-difference gradient test cannot.
  - Replaced the fixed-corotational element's Gauss-Newton Hessian with the exact
    analytic Hessian `2*mu*(I9 - dR/dF) + lambda*(g*g^T + (J - 1)*d^2J/dF^2)`,
    where the polar-rotation gradient `dR/dF` is solved from the corotational
    identity `(tr(S) I - S) w = axl(R^T dF - dF^T R)`, validated by a
    finite-difference Hessian match. Like the IPC paper's per-element Hessian it
    is generally indefinite, so the solver PSD-projects it through the existing
    batched seam; severely inverted elements (where the rotation gradient is
    undefined) fall back to the always-PSD Gauss-Newton form (regression added).
    The exact Newton curvature converges the line search far faster than the
    Gauss-Newton approximation, cutting the `BM_DeformableFcrBarStep` per-step
    time about 7x (to near stable-neo-Hookean parity at equal mesh resolution)
    while leaving the energy, gradient, and settled solution unchanged (every
    fixed-corotational kernel and solver test still passes).
  - Added a `newton_iters_per_step` counter to the `BM_DeformableFemBarStep` and
    `BM_DeformableFcrBarStep` benchmarks (averaged from the per-step solver
    diagnostics), surfacing the projected-Newton iteration count -- the
    convergence axis the IPC paper's Table 1 reports -- alongside the wall-clock
    per-step time. It quantifies the exact fixed-corotational Hessian's
    convergence: both materials now take ~5-6 Newton iterations per step at
    matching mesh resolution, confirming the exact curvature brought
    fixed-corotational convergence to stable-neo-Hookean parity (the earlier
    Gauss-Newton form's slowness was extra iterations, not per-iteration cost).
  - Exposed a read-only `DeformableSolverDiagnostics` snapshot of the deformable
    solver's per-step statistics on the experimental `World`
    (`World::getLastDeformableSolverDiagnostics`, dartpy
    `world.last_deformable_solver_diagnostics`). It is a curated, stable subset
    of the internal `DeformableSolverStats` -- mesh sizes, projected-Newton
    convergence (iterations, objective evaluations, line-search trials, Newton
    steps vs steepest-descent fallbacks), self-contact activity, friction
    dissipation, and the contact closest-approach diagnostic -- captured after
    each built-in-pipeline step so tools and Python scripts can observe solver
    behavior (e.g. per-step Newton iteration counts) without the explicit
    stage/pipeline API. The user-supplied-pipeline `step` overloads leave it
    unchanged (read the stage's own `getLastStats` there). Adds C++ and Python
    regressions.
  - Added a `Deformable FEM Self-Contact (IPC)` py-demos scene: a slender
    tetrahedral FEM beam whose pinned ends are driven toward each other by
    opposing scripted Dirichlet boundaries, so the soft FEM core buckles and
    folds onto itself; the always-on clamped-log self-contact barrier holds the
    folding surface apart at a positive separation. This is the first DART-native
    showcase of self-collision (the heart of the IPC method) on a volumetric FEM
    body, toward the paper's self-collision stress tests, with a reusable
    `build_fem_compression_bar` bridge helper. A regression drives the beam
    through the buckle and asserts -- via the new solver diagnostics -- that the
    self-contact barrier activates, holds every active contact at a strictly
    positive distance (no interpenetration), and keeps the solve finite.
  - Added a `Deformable FEM Twist (IPC)` py-demos scene: a tetrahedral FEM beam
    counter-rotated at both ends by opposing scripted Dirichlet boundary
    conditions, then released so the stable neo-Hookean core untwists
    elastically -- a DART-native step toward the IPC paper's Fig. 4 (rod twist)
    / Fig. 14 (mat twist) volumetric-shear themes. Shares a reusable
    hexahedral-to-tetrahedral bar mesh helper with the FEM cantilever scene; the
    demos cycle smoke covers it.
  - Validated and showcased stable neo-Hookean FEM volumetric bodies in IPC
    contact: a free tetrahedral FEM cube dropped onto a static ground barrier
    falls, squashes, and settles on the barrier surface intersection-free (no
    node crosses the ground top), exercising the FEM elasticity and the
    clamped-log ground barrier together in one solve. Adds a
    `FemCubeSettlesOnGroundBarrierWithoutPenetrating` regression and a
    `Deformable FEM Drop (IPC)` py-demos scene -- a DART-native step toward the
    paper's volumetric drop scenes (e.g. Fig. 12).
  - Added internal experimental IPC projected-Newton search direction for the
    deformable solve: each iteration assembles the per-step Hessian (inertia +
    spring + self-contact barrier + static ground barrier) with per-element
    PSD projection and solves it (dense LDLT, guarded by a positive-definite
    check) for a Newton direction, replacing mass-scaled steepest descent and
    falling back to it when the dense solve is skipped (large bodies), fails,
    or its line search cannot make progress. This lets the stiff barrier term
    converge cleanly and matches the analytic implicit-Euler spring solution.
    Sparse assembly and CPU/GPU optimization of the per-element
    eigen-decompositions are later slices. Includes focused regressions (Newton
    engaged, few-iteration convergence, analytic parity) and solver-step
    counters.
  - Optimized the experimental IPC projected-Newton solve to assemble a SPARSE
    per-step Hessian (Eigen triplet assembly) and factorize it with a sparse
    Cholesky (`SimplicialLDLT`, guarded by a positive-diagonal check), lifting
    the former 256-node dense-solve cap to thousands of nodes so paper-scale
    deformable meshes are solved on the Newton path instead of falling back to
    steepest descent. Fixed DOFs are pinned at assembly time (unit diagonal,
    free-free element blocks only), preserving the exact dense result; the
    steepest-descent fallback and PD guard are unchanged. The solve refactorizes
    from scratch each iteration (per-step cost is not yet optimized);
    symbolic-factorization reuse, matrix-free CG, and GPU offload of the
    assembly/solve remain later perf slices. Adds a 300-node-chain regression
    and a 320-node grid-on-ground-barrier regression (Newton engaged past the
    old cap) plus sparse-Newton step/fallback counters on the scaling benchmark.
  - Added a `drape` showcase scene to the `experimental_deformable_gui` example
    (`--deformable-scene-kind drape`): a 572-node deformable mat drapes over a
    raised ground-barrier step onto the ground, exercising the landed IPC
    contact pipeline (self-contact + ground barrier + sparse projected Newton)
    at a mesh scale past the former 256-node cap. It is a DART-native showcase,
    not a faithful paper-figure reproduction (mass-spring, no codimensional/FEM
    elasticity or friction). Adds a library-level drape regression
    (non-penetrating, conforming over the step) and a `BM_DeformableDrapeStage`
    benchmark of the multi-barrier solve.
  - Added an optional CUDA primitive for the deformable solver's data-parallel
    hotspot: `projectSymmetricBlocksToPsdCuda` batches the per-element PSD
    projection (the symmetric eigendecomposition that the projected-Newton
    assembly applies to every spring 6x6 and barrier 12x12 block) onto the GPU
    via a per-block cyclic-Jacobi kernel, with an identical-semantics CPU
    reference (`projectSymmetricBlocksToPsdReference`). It ships as part of the
    opt-in `dart-simulation-experimental-cuda` sidecar (built only with
    `DART_ENABLE_EXPERIMENTAL_CUDA=ON`); the default CPU runtime keeps no GPU
    dependency. A CUDA unit test validates the GPU path against the CPU
    reference for spring and barrier block sizes. This is the standalone,
    validated building block: wiring it into the live solve needs an optional
    GPU compute-backend injection path (so `world_step_stage` stays GPU-free per
    the runtime-dependency policy) and is a later slice.
  - Optimized the experimental IPC sparse projected-Newton solve to reuse its
    fill-reducing symbolic factorization (`SimplicialLDLT::analyzePattern`)
    across iterations and steps whenever the assembled Hessian sparsity pattern
    is unchanged, so the ordering runs once and only the numeric factorization
    repeats. It is behavior-preserving (a structural mismatch or failed
    factorization re-analyzes) and roughly halves the per-step solve on a
    settled 512-node grid (~21.7 ms to ~11.6 ms). Adds symbolic/numeric
    factorization counters and a regression asserting a steady step performs
    zero new symbolic analyses.
  - Added an experimental IPC solver convergence diagnostic: the deformable
    stage now reports `finalGradientResidualNorm`, the largest projected-Newton
    gradient norm at solve termination across the step's bodies (near the
    gradient tolerance means converged; a large value flags an iteration-cap or
    stall), surfaced on the grid/drape stage benchmarks toward the paper's
    benchmark-statistics tables (Fig. 23 / Table 1).
  - Added experimental IPC lagged smoothed Coulomb friction for deformable
    contact against static ground barriers (PLAN-081 Phase 4 first increment).
    A new `DeformableMaterialProperties.frictionCoefficient` (default 0, so
    existing behavior is unchanged) drives a friction force opposing each
    contacting node's tangential displacement over the step. The IPC mollifier
    (f0/f1, velocity threshold epsv) makes the force C1: it saturates at
    mu \* normalForce when sliding and ramps smoothly to zero at rest. The normal
    force is lagged once per outer iteration; the lagged friction Hessian is a
    positive-semidefinite tangential 2x2 block, so it integrates cleanly into
    the projected-Newton solve. Adds regressions (a sliding node decelerates
    versus the frictionless control; friction is inactive without contact), a
    friction variant of the drape benchmark, and serialization of the new
    coefficient. Self-contact and codimensional friction are later increments.
  - Extended experimental IPC friction to deformable SELF-CONTACT (point-triangle
    pairs), reusing the same `frictionCoefficient`. The lagged normal force is
    the barrier force magnitude on the point node, and the tangent projection
    comes from the point-triangle tangent stencil; friction opposes the stencil's
    tangential relative displacement with the same IPC f0/f1 mollifier. Adds a
    regression (one surface sliding tangentially on another in self-contact
    decelerates versus the frictionless control while the barrier keeps them
    separated). Energy+gradient only for now (the self-contact friction Hessian,
    edge-edge friction, and non-flat ground normals are later increments); the
    line search on the friction-inclusive energy ensures descent.
  - Added a deterministic scene-replay regression for the experimental
    deformable scene pipeline (PLAN-081 Phase 5 validation harness): a
    DART-native tutorial-style scene (one Dirichlet-anchored cube, one free cube
    falling under gravity) is replayed for many frames through the real
    loader -> solver -> diagnostics path, asserting the anchor stays put, the
    free body falls, mass is conserved, the diagnostics are finite, and a second
    identical replay reproduces the trajectory. This is the per-scene invariant
    pattern the upstream ipc-sim/IPC corpus rows require; the full 154-scene port
    additionally needs the upstream scene assets vendored and the
    contact-capable solver (later phases), so the manifest rows stay `planned`.
  - Exposed the experimental deformable-body API to `dartpy` (PLAN-081 Phase 8):
    `World.add_deformable_body` / `get_deformable_body` / `has_deformable_body` /
    `get_deformable_body_count`, and `DeformableBodyOptions`,
    `DeformableMaterialProperties` (including `friction_coefficient`),
    `DeformableEdge`, and `DeformableBody` (`node_count`, `edge_count`,
    `node_position` / `set_node_position`, `node_velocity` / `set_node_velocity`,
    `is_fixed_node`, `edge`, `material_properties`). Regenerated the type stubs
    and added a Python regression that builds a spring strand, sets a friction
    coefficient, steps the world, and reads the resulting state. Surface/tetra
    topology and boundary-condition bindings remain a later increment.
  - Added the lagged self-contact friction Hessian to the experimental IPC
    projected-Newton solve, completing self-contact friction as a proper Newton
    term (previously energy+gradient only). Per active point-triangle contact it
    assembles a positive-semidefinite 12x12 block,
    `projection^T * H_2x2 * projection` (the tangent-stencil projection times the
    same PSD tangential matrix the ground-friction Hessian uses), scattered to
    the four stencil nodes. Behavior-preserving (the line search still resolves
    the same friction-inclusive energy); existing tests pass unchanged. The
    edge-edge self-contact friction Hessian remains a later increment.
  - Extended experimental IPC self-contact friction (force and Hessian) to
    edge-edge contacts. The friction energy/gradient/Hessian are generic over a
    four-node stencil + tangent projection, so only the lagged-contact assembly
    is extended: active edge-edge barrier candidates contribute a friction
    contact whose lagged normal force is the net barrier force on the first
    edge and whose tangent projection comes from the edge-edge tangent stencil.
    Adds a regression where one crossing edge slides over another in edge-edge
    self-contact and decelerates versus the frictionless control while the
    barrier holds them apart. Self-contact friction now covers both
    point-triangle and edge-edge primitives.
  - Generalized experimental IPC static-ground friction to follow the true
    geometric ground normal instead of a hardcoded xy tangent plane. Each
    contacting node now lags the supporting barrier surface's normal (radial for
    a sphere, the supporting-face normal for a rotated/tilted box), and the
    friction energy, gradient, and tangential Hessian resolve against the plane
    orthogonal to it (the Hessian is now a positive-semidefinite 3x3 tangent
    block). For flat or box-top ground the normal is +z and the behavior reduces
    exactly to the previous xy tangent plane, so existing flat-ground friction
    tests are unchanged; the underlying barrier force stays a vertical height
    field. Adds a regression on a 45-degree tilted slope where tilt-aware
    friction couples the normal and tangential directions so a node dropped
    straight down deflects down-slope, whereas the frictionless control (and any
    xy-only tangent model) leaves it on the drop line. Codimensional-obstacle
    friction and friction-specific diagnostics remain later increments.
  - Exposed the experimental deformable scene loader and replay diagnostics to
    `dartpy`, completing the PLAN-081 Phase 8 Python facade. Added
    `load_deformable_scene` and `collect_deformable_scene_diagnostics` module
    functions, plus `DeformableSceneLoadOptions`, `DeformableSceneInfo`,
    `DeformableSceneBodyInfo`, and `DeformableSceneDiagnostics` bindings, so a
    contact-free tutorial-style scene file can be loaded into a `World` from
    Python and its per-body counts, total mass, and bounds read back.
    Regenerated the type stubs and API boundary inventory, and added a Python
    regression that loads a single-tetrahedron scene and checks the reported
    topology counts and total mass. (The loader still covers only the
    contact-free subset; it is not IPC scene parity.)
  - Added a dedicated `IPC Deformable (sx)` category of deformable scenes to the
    consolidated `py-demos` standalone (`pixi run py-demos`), grouping the IPC
    showcases in their own menu category instead of mixing them into the general
    `Experimental` (sx) scenes: `ipc_deformable_net` (a pinned spring net sagging
    under gravity), `ipc_deformable_drape` (a mat draping over a step onto a
    ground barrier), `ipc_deformable_trampoline` (a corner-pinned membrane sagging
    and rebounding under projected Newton), `ipc_deformable_friction_slide` (a
    launched mat skidding to rest under lagged smoothed-Coulomb ground friction),
    and `ipc_deformable_scripted_dirichlet` (a banner billowing under a scripted
    Dirichlet boundary condition). A shared `_ipc_deformable_bridge.py` builds the
    grid topology and mirrors each `dartpy.simulation_experimental` deformable
    body onto the Filament render world as per-node spheres plus a spring
    wireframe (the dynamic surface-mesh render path is not yet exposed through
    `dartpy.gui.run_demos`). To enable the drape and friction scenes, exposed
    `RigidBody.is_deformable_ground_barrier` to `dartpy` (mirroring the existing
    `is_deformable_surface_ccd_obstacle` property) so a static rigid box can be
    tagged as a deformable ground barrier from Python; regenerated the type
    stubs and added a binding regression. These are DART-native point-mass/spring
    showcases that evoke the IPC paper's themes (draping, self-contact, friction,
    scripted boundaries) but are not faithful paper-figure reproductions; the
    upstream scene corpus stays `planned` pending vendored assets and an
    FEM/codimensional material model. The demos cycle smoke covers every scene,
    plus a solver-free grid-builder topology test and a registry test that pins
    the dedicated category.
  - Extended the experimental deformable-body `dartpy` facade with scripted
    boundary conditions (PLAN-081 Phase 8). Added
    `DeformableDirichletBoundaryCondition` (`nodes`, `linear_velocity`,
    `angular_velocity`, `center`, `start_time`, `end_time`) and
    `DeformableNeumannBoundaryCondition` (`nodes`, `acceleration`, `start_time`,
    `end_time`) bindings, plus the `DeformableBodyOptions`
    `dirichlet_boundary_conditions` / `neumann_boundary_conditions` fields.
    Regenerated the type stubs and API boundary inventory, and added a Python
    regression where a Dirichlet-scripted node follows its prescribed linear
    motion while a Neumann-accelerated node falls under the applied
    acceleration. Scene-loader Python access and diagnostics exposure remain a
    later increment.
  - Extended the experimental deformable-body `dartpy` facade with surface and
    tetrahedral topology (PLAN-081 Phase 8). Added `DeformableSurfaceTriangle`
    and `DeformableTetrahedron` bindings, the `DeformableBodyOptions`
    `surface_triangles` / `tetrahedra` fields, and `DeformableBody` read
    accessors `surface_triangle_count` / `surface_triangle`,
    `tetrahedron_count` / `tetrahedron` / `tetrahedron_rest_volume`, and
    `node_mass`. Regenerated the type stubs and API boundary inventory, and
    added a Python regression that builds a single tetrahedron with an explicit
    boundary surface and reads back the topology, rest volume, and node mass.
    Boundary-condition (DBC/NBC) and scene-loader bindings remain a later
    increment.
  - Added a GPU-vs-CPU performance gate for the experimental IPC deformable PSD
    projection and tuned the GPU offload threshold to the measured crossover. A
    CUDA test now projects 12x12 self-contact barrier batches across a sweep of
    sizes, asserting GPU/CPU parity at every scale and logging each path's wall
    time. On an RTX 5000 Ada the GPU path runs ~0.4x at 256 blocks, ~1.4x at
    1024, ~4x at 4096, and ~9x at 16384, so the backend adapter's minimum GPU
    batch size is raised from 64 to ~1024 blocks (small batches stay on the CPU
    backend where the host/device round trip would otherwise dominate).
  - Gave the experimental IPC deformable GPU PSD projection a resident device
    buffer (PLAN-081 Phase 3). The CUDA backend previously allocated and freed a
    device buffer (and copied through a temporary host vector) on every
    projected-Newton iteration; it now reuses one persistent device allocation
    that grows on demand and is freed when the GPU backend is uninstalled, and
    the backend adapter projects in place on the caller's packed buffer (no
    per-call host copy). Results are bit-identical to before (same kernel and
    transfers), so it is purely a performance change confined to the opt-in
    `dart-simulation-experimental-cuda` sidecar; the default CPU runtime is
    unaffected. A CUDA test asserts the resident buffer is reused across
    same-or-smaller batches, grows once for a larger batch, and is released on
    restore, with GPU/CPU parity preserved throughout.
  - Wired an optional GPU backend into the experimental IPC deformable solver's
    per-element PSD projection (PLAN-081 Phase 3). The projected-Newton assembly
    now collects its per-element spring (6x6) and self-contact barrier (12x12)
    Hessian blocks into one packed batch and projects them through a pluggable
    backend (`compute::projectSymmetricBlocksToPsd`) before scattering, instead
    of projecting each block inline. The default CPU backend is bit-identical to
    the previous per-element projection, so behavior is unchanged. The optional
    CUDA sidecar can install a GPU backend via `installCudaDeformablePsdBackend`
    that offloads large batches to the device (and defers small batches or the
    no-device case to the CPU backend), so the offload never changes results.
    The default runtime never links CUDA -- the no-GPU-runtime-dependency check
    still passes -- and the GPU path is validated against the CPU backend through
    the core seam in the CUDA test suite.
  - Added an experimental IPC converged-ness diagnostic to the deformable solver
    stats. Each step now reports `finalStepInfinityNorm`, the largest last
    accepted per-node position update across the step's bodies. Unlike the
    gradient residual, which can stay large at a stiff clamped-log barrier
    contact because the barrier Hessian is near-singular, the step norm shrinks
    toward zero as the solve reaches a feasible equilibrium, so it is the honest
    companion to `finalGradientResidualNorm` for judging convergence on
    barrier-dominated problems. Behavior-preserving (a diagnostic only; the solve
    is unchanged). Adds a regression where a stiff grid pressed onto the ground
    barrier accepts a measurable step while settling and then drives the step
    norm to a negligible value at equilibrium.
  - Added experimental IPC friction diagnostics to the deformable solver stats
    (PLAN-081 Phase 4). Each step now reports `frictionDissipation` (the IPC
    Coulomb work mu \* normalForce \* f1(y) \* y summed over active friction
    contacts at the converged iterate -- force times tangential slip, equal to
    mu \* normalForce \* slip in the kinetic regime and ramped smoothly to zero
    at rest by the f0/f1 mollifier) and `activeFrictionContacts` (the number of
    static-ground and self-contact friction contacts carrying a nonzero lagged
    normal force). Both are zero when friction is disabled and are computed once
    per step outside the line-search hot path. Adds a regression that a sliding
    ground-contact node reports positive dissipation over a nonzero active set
    while the frictionless control reports zero. These feed the paper's friction
    benchmark statistics (Fig. 23 / Table 1) alongside the existing
    `finalGradientResidualNorm` convergence diagnostic.
  - Added an experimental IPC contact closest-approach diagnostic to the
    deformable solver stats. Each step now reports `minActiveContactDistance`
    (the smallest point-triangle / edge-edge distance among the active
    self-contact barrier set at the converged iterate -- the IPC
    intersection-free "minimum distance" statistic, Fig. 23 / Table 1) and
    `convergedActiveContactCount` (the size of that active set at solve
    termination, a single-iteration snapshot distinct from the cumulative
    `selfContactBarrierActiveContacts`). Both are zero for bodies without active
    self-contact, the distance is meaningful only when the count is positive,
    and both are read once per step outside the line-search hot path.
    Behavior-preserving (a diagnostic only; the solve is unchanged). Adds
    regressions that a self-contact step reports a positive closest approach
    strictly inside the activation band while a far-apart configuration reports
    an empty active set, plus self-contact stage benchmark counters.
  - Added internal experimental IPC conservative continuous-collision step
    bounds for point-triangle and edge-edge primitive candidate pairs by
    wrapping native primitive CCD, with exact-CCD regression tests, sampled
    safety checks, and benchmark counters.
  - Added internal experimental Vertex Block Descent (VBD) per-vertex block
    kernels for the variational implicit-Euler objective: the inertia
    force/Hessian, the mass-spring force/Hessian with an opt-in PSD clamp, and a
    regularized symmetric-positive-definite 3x3 block Newton solve with a
    zero-step fallback, with finite-difference derivative regression tests and a
    microbenchmark.
  - Added internal experimental VBD vertex graph coloring (element-induced
    vertex adjacency plus deterministic Welsh-Powell greedy coloring and a
    conflict-free verifier) so same-color vertex blocks can be updated in
    parallel while colors are swept Gauss-Seidel, with regression tests and a
    coloring benchmark.
  - Added an internal experimental single-body VBD block-descent driver
    (colored Gauss-Seidel mass-spring solve over the same objective the existing
    deformable stage minimizes), with regression tests for monotone energy
    decrease, residual convergence, converged-state parity against an
    independent gradient-descent minimizer, same-color independence, and a
    grid-step benchmark. This is internal solver groundwork only and is not yet
    wired into the deformable world step.
  - Added an internal experimental Stable Neo-Hookean tetrahedral energy kernel
    for VBD (energy density, analytic first Piola stress, and per-vertex force
    plus exact 3x3 Hessian via the analytic stress differential), with
    finite-difference force/Hessian regression tests, rest-state equilibrium,
    element-inversion stability, and a microbenchmark.
  - Wired the VBD Stable Neo-Hookean tetrahedral blocks into a single-body
    block-descent driver (tet adjacency, tet-induced vertex coloring, and a
    colored Gauss-Seidel volumetric solve), validated for rest stability,
    residual convergence, monotone energy decrease, and converged-state parity
    against an independent global gradient-descent minimizer of the tet
    objective, with a tetrahedral-bar step benchmark. Internal solver
    groundwork only; not wired into the deformable world step.
  - Added internal experimental VBD acceleration and damping primitives:
    adaptive inertial/previous-step initialization (gravity-projected
    acceleration blend), Chebyshev semi-iterative weights and over-relaxation,
    and stiffness-proportional Rayleigh damping for the per-vertex block, each
    unit-tested. These are standalone helpers, not yet threaded through a
    multi-step VBD stepping loop.
  - Added an internal experimental single-body VBD mass-spring stepper that
    performs one implicit-Euler step (inertial targets, adaptive warm start,
    colored block-descent sweeps with optional Chebyshev over-relaxation,
    velocity update, and previous-velocity bookkeeping), validated against the
    implicit-Euler free-fall trajectory, fixed-vertex immobility, multi-step
    stability, Chebyshev no-op/convergence, and adaptive-init convergence.
    Internal solver groundwork only; not wired behind the deformable world
    stage.
  - Wired VBD into the experimental deformable world stage as an internal,
    opt-in inner solver for contact-free mass-spring bodies, selected per body
    through a non-public `comps::DeformableVbdConfig` so the public deformable
    stage stays algorithm-neutral. The stage caches the spring
    coloring/adjacency per body and reuses the existing inertial-target setup
    and write-back. Integration tests confirm the VBD path runs only when opted
    in, the default solver runs otherwise, VBD agrees with the gradient-descent
    solver on a contact-free scene, and a hanging chain is stable, with no
    regression in the existing deformable suite. Tetrahedral, surface-contact,
    ground-barrier, and rigid-obstacle bodies still use the default solver.
  - Added residual-based early termination to the VBD block-descent solver (a
    convergence-displacement threshold, exposed through the internal config) and
    a `bm_vbd_world_solver` benchmark comparing the VBD inner solver against the
    default gradient-descent solver on a matched contact-free mass-spring grid
    in the real World pipeline. With early termination, single-threaded VBD is
    roughly 2-4x faster per step than the default solver on the benchmarked
    scenes. Multithreaded CPU sweeps remain future work.
  - Added an experimental CUDA Vertex Block Descent mass-spring kernel
    (`compute/cuda/vbd_block_descent_cuda`) behind `DART_ENABLE_EXPERIMENTAL_CUDA`:
    a per-color parallel block-update kernel with an analytic SPD 3x3 solve and a
    single-stream Gauss-Seidel sweep over colors. A device-skipping test confirms
    the GPU result matches the CPU block-descent solve, and a GPU-vs-CPU
    benchmark shows the GPU roughly 9x faster at 4k vertices and 26x faster at
    16k vertices, scaling near-flat while the CPU path grows linearly. A
    tetrahedral GPU kernel and paper-scene comparisons remain future work.
  - Added a device-resident CUDA VBD rollout (`vbdRolloutMassSpringCuda`) that
    runs the full per-step pipeline (inertial-target prediction, colored sweeps,
    velocity update) on the GPU for many steps with one upload and one download,
    plus inertial-target and velocity-update kernels. A device-skipping test
    confirms it matches the CPU stepper over 20 steps; the rollout benchmark
    measures ~1.08 ms/step steady-state, about 45x faster than the
    single-threaded CPU at 16k vertices.
  - Added a CUDA Stable Neo-Hookean tetrahedral VBD kernel
    (`vbdStepTetMeshCuda`) with a device port of the analytic per-vertex
    force/Hessian and a tet-colored sweep. A device-skipping test confirms the
    GPU tetrahedral solve matches the CPU `blockDescentTetMesh` to 1e-6,
    extending the GPU win to the paper's volumetric domain. A tet GPU-vs-CPU
    benchmark shows the GPU about 1.5x faster at ~500 vertices and 4.4x at ~2000
    vertices, with the margin growing with mesh size.
  - Added a device-resident CUDA tetrahedral VBD rollout
    (`vbdRolloutTetMeshCuda`), the volumetric counterpart of the mass-spring
    rollout: the full per-step pipeline (inertial-target prediction, colored
    Neo-Hookean sweeps, velocity update) runs on the GPU for many steps with a
    single upload and download. A device-skipping test confirms it matches the
    equivalent CPU per-step tet pipeline over 20 steps.
  - Added optional CUDA-graph capture to both device-resident rollouts
    (`useCudaGraph`): one step's identical launch sequence is captured into a
    CUDA graph and replayed for every step, amortizing the per-launch overhead
    of the many small per-color kernels. A device-skipping test confirms the
    graph rollout matches the directly-launched rollout bit-for-bit, and the tet
    rollout benchmark runs with and without graph capture.
  - Added a single-precision (mixed-precision) device-resident mass-spring
    rollout. The device kernels are templated on the scalar type, and the
    `useSinglePrecision` flag runs the rollout in float (host state stays double
    and is converted on upload/download). A device-skipping test confirms the
    float rollout tracks the double rollout to float accuracy. On the RTX 5000
    Ada (whose FP64 throughput is 1/64 of FP32) the float path is markedly
    faster — about 2.7x at 16k vertices and roughly 6x at 4k vertices in the
    rollout benchmark — at float precision.
  - Extended single precision to the tetrahedral CUDA rollout: the tet device
    path (the `DVec3` vector type, the Stable Neo-Hookean accumulator, and the
    tet color-sweep kernel) is templated on the scalar type, and
    `useSinglePrecision` runs `vbdRolloutTetMeshCuda` in float. A device-skipping
    test confirms the float tet rollout tracks the double rollout to float
    accuracy; on the RTX 5000 Ada the float tet rollout is about 9-13x faster
    (the heavy per-vertex Neo-Hookean FP64 math benefits even more than the
    mass-spring path), at float precision.
  - Added a VBD benchmark on the upstream TinyVBD reference default scene (a
    20-vertex tilted strand with structural and skip springs, 100 iterations per
    step). On a single CPU thread, DART's VBD runs at roughly 0.21 ms/step
    versus the TinyVBD reference on the same scene across sizes (the strand
    length is configurable in both), with DART about 2.7-3.0x faster at 20, 100,
    and 400 vertices (e.g. 0.16 vs 0.49 ms, 0.78 vs 2.33 ms, 3.18 vs 8.67 ms per
    step). DART's CPU VBD thus wins robustly, not just at one size; it uses a
    double LDLT 3x3 block solve plus tight assembly while TinyVBD uses a float
    colPivHouseholderQr solve. The Gaia GPU framework and the paper's
    tetrahedral RTX-4090 scene numbers remain future work.
  - Added a multithreaded VBD block-descent driver
    (`parallelBlockDescentMassSpring`) that runs the colored Gauss-Seidel sweep
    on a fixed worker-thread pool with a `std::barrier` between colors. Because
    same-color vertices are independent, the parallel result is bit-identical to
    the serial driver (verified). Thread scaling on a 96x96 grid gave roughly
    1.65x/2.6x/3.5x speedups at 2/4/8 threads.
  - Added an internal experimental VBD half-space penalty-contact kernel
    (`contact_kernel.hpp`: `ContactPlane`, `addHalfSpacePenaltyContact`, energy
    `E_c = (k_c/2) d^2`) and a contact-aware mass-spring driver
    (`blockDescentMassSpringGround`), so deformable bodies rest on static
    ground/obstacle half-spaces instead of tunneling. Tests cover the
    finite-difference force match, inactivity above the plane, the
    positive-semidefinite contact Hessian, a particle resting on the ground, and
    a spring net sagging onto the ground.
  - Added semi-implicit Coulomb friction for VBD half-space contact
    (`addHalfSpaceFriction`: tangential penalty clamped to the Coulomb limit
    `mu * lambda`, with a positive-semidefinite Hessian in both the sticking and
    sliding regimes) and the contact+friction driver
    `blockDescentMassSpringGroundFriction`. Tests cover the sticking-regime
    finite-difference force match, the Coulomb-capped sliding force, and a
    sliding particle decelerated to rest by kinetic friction.
    Vertex-triangle/edge-edge contact and self-collision remain future work.
  - Added a `dart-demos` GUI scene `experimental_vbd` ("Deformable VBD
    (Experimental)") that drives a contact-free hanging cloth through the VBD
    inner solver, selected via the internal `comps::DeformableVbdConfig` through
    `World::getRegistry()` so the public deformable facade stays
    solver-agnostic. Run it with `pixi run demos -- --scene experimental_vbd`;
    it joins the headless demo-cycle smoke test for visual-regression coverage.
    This is the first user-runnable VBD visual showcase (PLAN-104 Phase 10).
  - Added a combined springs + tetrahedra VBD block-descent driver
    (`blockDescentDeformable` with `colorDeformable` and a combined per-vertex
    assembler) that minimizes inertia + distance-spring + Stable Neo-Hookean
    tetrahedron energy in one graph-colored Gauss-Seidel sweep. It reduces
    bit-for-bit to `blockDescentMassSpring` with no tets and to
    `blockDescentTetMesh` with no springs, and its converged state matches an
    independent global gradient-descent minimizer of the combined objective.
  - Extended the experimental World VBD path to tetrahedral bodies: the stage
    now builds Stable Neo-Hookean tetrahedra from the body topology and its
    `DeformableMaterial` Lame parameters, so VBD models material-dependent
    volumetric elasticity that the default gradient solver does not. The opt-in
    gate was relaxed to run VBD on bodies free of external contact (ground and
    rigid-surface) sources; surface self-contact for VBD bodies remains a later
    slice. Tests confirm a tetrahedral body routes through VBD, stays stable
    under gravity, and that a stiffer material settles measurably closer to its
    rest shape.
  - Threaded Chebyshev semi-iterative acceleration and stiffness-proportional
    Rayleigh damping into the World VBD path. `blockDescentDeformable` now
    optionally over-relaxes each sweep with Chebyshev (same fixed point, faster
    convergence when the spectral radius is matched) and adds Rayleigh damping
    opposing the per-step displacement, both selected through new
    `DeformableVbdConfig` knobs (`useChebyshev`/`chebyshevRho`/
    `rayleighDamping`). Tests confirm Chebyshev converges to the same minimizer,
    Rayleigh keeps the per-vertex system positive definite while changing the
    iterate, the World accelerated solve matches the plain one, and World
    Rayleigh damping dissipates kinetic energy. Chebyshev is opt-in and
    conservative by default because too high a spectral radius can over-relax.
  - Wired the deterministic multithreaded color sweep into the World VBD path.
    Added `parallelBlockDescentDeformable` (the springs + tetrahedra counterpart
    of the mass-spring parallel driver: a worker pool updates each color's
    disjoint vertex slices with a `std::barrier` between colors) and a
    `workerThreads` knob on `DeformableVbdConfig`. With more than one worker the
    World VBD solve runs the multithreaded sweep, which is bit-identical to the
    serial driver (verified at the kernel level for springs + tets and at the
    World level over ten steps). The serial path keeps Chebyshev and residual
    early termination; the multithreaded path omits them (they need a global
    extrapolation / cross-thread reduction) and falls back to the serial driver
    for a single worker.
  - Wired static ground/obstacle contact (and optional Coulomb friction) into
    the World VBD path. `blockDescentDeformable` and
    `parallelBlockDescentDeformable` now accept per-vertex `ContactPlane`s and a
    friction coefficient, adding the VBD half-space penalty and semi-implicit
    Coulomb friction terms per vertex. The stage builds the per-vertex planes
    each step from the static ground-barrier set (z-up half-spaces at the
    per-xy barrier top via `staticGroundTopAt`, lagged at the warm-start
    position), gated by a new `DeformableVbdConfig::contactStiffness`. The VBD
    gate was relaxed to run on bodies with ground barriers when the contact
    stiffness is positive; a body with barriers but no VBD contact stiffness
    still falls back to the default solver so it rests on the ground, and
    rigid-surface CCD and surface self-contact remain default-solver-only.
    Tests confirm a VBD spring patch rests on a ground barrier instead of
    falling through and that Coulomb friction shortens a sliding patch's travel.
  - Added a public, algorithm-neutral way to select the iterative deformable
    inner solver from the experimental World facade:
    `World::configureDeformableSolver(name, DeformableSolverOptions)`. The
    options struct (iterations, convergence tolerance, acceleration, stiffness
    damping, worker threads, ground-contact stiffness) carries no solver-name
    vocabulary; the World step pipeline owns the translation to the internal
    opt-in inner-solver component, so the public surface stays solver-agnostic
    and the binding never touches `comps/`. This is the supported way for
    dartpy and examples to opt a deformable body into the VBD solver. Tests
    confirm it routes a body through the VBD path, honors ground contact, and
    throws for an unknown body name.
  - Bound the deformable solver config to dartpy: `DeformableSolverOptions` and
    `World.configure_deformable_solver(name, options)` are now available from
    Python (the binding never includes internal `comps/` headers, satisfying the
    API-boundary rule). This is what lets the Python demos select the VBD solver.
  - Added Vertex Block Descent deformable scenes to the Python `py-demos` app —
    the VBD paper demos that the current contact-free World VBD path reproduces:
    a mass-spring hanging cloth (`vbd_cloth`) and net (`vbd_net`), and a
    tetrahedral cantilever beam (`vbd_beam`, the single-body analogue of the
    paper's twisting beams). Each builds an experimental `sx.World`, opts the
    body into VBD via `configure_deformable_solver`, and renders through a new
    `SxRenderBridge.add_deformable_visual` as a live deforming surface
    wireframe (spring edges for the mass-spring cloth/net, boundary-triangle
    edges for the tetrahedral beam) with pinned nodes marked, instead of an
    isolated per-node point cloud. The paper's multi-body / self-collision
    scenes (216 squishy balls, 10,368 models, tearing cloth) need surface
    self-contact and are deferred.
  - Added internal experimental Augmented VBD (AVBD) row foundations on top of
    the VBD deformable path: scalar row updates, deterministic row inventory,
    mass-spring contact-normal / hard-attachment / finite-stiffness /
    friction-tangent rows, finite-stiffness tetrahedral material rows, and
    point-triangle / edge-edge self-contact normal rows.
    Narrow World opt-ins now cover supported serial mass-spring row
    combinations, supported serial mass-spring self-contact normal rows, and
    supported pure-tet finite-material rows with diagnostics and explicit
    fallback coverage, including bounded friction tangents for supported
    static-contact mass-spring scenes. The supported mass-spring friction
    tangent pairs now use the previous tangential dual to switch between static
    sticking and dynamic sliding and project pair forces to the circular
    Coulomb cone. AVBD self-contact friction tangent rows now reuse lagged
    point-triangle / edge-edge tangent stencils in the combined mass-spring row
    driver, with supported World generation for serial self-contact scenes plus
    pairwise static/dynamic switching and cone-projection coverage. Full AVBD
    contact/friction, broader self-contact friction envelopes, rigid/soft
    coupling, GPU parity, demos, and benchmark parity remain future work.
  - Made dartpy experimental `world.step(n=...)` reject negative step counts
    explicitly while preserving zero-count no-op behavior.
  - Updated experimental kinematics refresh so generalized joint-position
    writes drive open-chain link transforms and loop-closure residuals for
    standard tree joints.
  - Added experimental joint DOF count, generalized position, and generalized
    velocity accessors in C++ and snake_case dartpy bindings.
  - Tightened the experimental dartpy API so data-like frame, joint,
    loop-closure, and rigid-body state is exposed through Python properties
    instead of parallel getter/setter-style aliases.
  - Added dartpy bindings for experimental `StateSpace` metadata so Python
    users can define named flat-vector dimensions and bounds without exposing
    ECS storage or component mappers.
  - Added Pythonic loop-closure conveniences for auto-named construction and
    direct runtime participation properties.
  - Made experimental loop closures reject active projection or solve policies
    at runtime until compatible stages are present.
  - Rejected invalid experimental `RigidBodyOptions` at the C++ and dartpy API
    boundaries, including non-positive mass, non-finite pose or velocity data,
    zero orientation quaternions, and non-symmetric-positive-definite inertia.
  - Added first-class dartpy bindings for experimental `Frame`, `FreeFrame`,
    and `FixedFrame`, and kept descendant frame reads fresh after parent-frame
    local transform changes.
  - Added experimental rigid-body lookup by name in C++ and dartpy so users can
    retrieve first-class `RigidBody` handles without touching ECS storage.
  - Added experimental rigid-body transform setters in C++ and dartpy for
    kinematics-only pose updates through the public `RigidBody` facade.
  - Added experimental rigid-body velocity accessors in C++ and dartpy so
    velocity-driven stepping no longer requires direct ECS component access.
  - Added experimental rigid-body mass and inertia accessors in C++ and dartpy
    so inertial properties are available through the public `RigidBody` facade.
  - Added experimental rigid-body force and torque accessors in C++ and dartpy
    so force-driven stepping no longer requires direct ECS component access.
  - Added topology-only experimental loop-closure APIs in C++ and dartpy for
    named closed-chain relations between public frames, links, and rigid bodies.
  - Fixed experimental parallel executor paths to propagate compute-node
    exceptions instead of reporting graph execution success after failed tasks.
  - Fixed experimental compute profiling and rigid-body stepping edge cases for
    equal-timestamp profile events, reparented rigid bodies, and parented
    rigid-body task dependencies.
  - Added `WorldConfig` support for collision detector selection plus multi-solver world scaffolding and sensor integration. ([#2168](https://github.com/dartsim/dart/pull/2168), [#2349](https://github.com/dartsim/dart/pull/2349), [#2352](https://github.com/dartsim/dart/pull/2352))
  - Joint and constraint enhancements: state-independent `Joint::integratePositions`, per-DoF mimic actuator modes, revolute joint constraint for closed-loop hinges, WeldJoint merge, PlanarJoint SE2 helpers, and joint coordinate charts. ([#2309](https://github.com/dartsim/dart/pull/2309), [#2222](https://github.com/dartsim/dart/pull/2222), [#2252](https://github.com/dartsim/dart/pull/2252), [#2242](https://github.com/dartsim/dart/pull/2242), [#2231](https://github.com/dartsim/dart/pull/2231), [#2351](https://github.com/dartsim/dart/pull/2351))
  - Coupler and mimic updates: new coupler constraint support, Gazebo-aligned mimic constraints, and URDF transmission coupling. ([#2212](https://github.com/dartsim/dart/pull/2212), [#2247](https://github.com/dartsim/dart/pull/2247), [#2281](https://github.com/dartsim/dart/pull/2281))
  - Dynamics fixes: FreeJoint world Jacobian translation, joint impulse statefulness, Aspect lifecycle replacement, servo limit recovery, stale joint command handling, RealTimeWorldNode stall fixes, and graceful handling of invalid (negative/NaN/Inf) joint physics parameters. ([#2298](https://github.com/dartsim/dart/pull/2298), [#2308](https://github.com/dartsim/dart/pull/2308), [#2304](https://github.com/dartsim/dart/pull/2304), [#2094](https://github.com/dartsim/dart/pull/2094), [#2098](https://github.com/dartsim/dart/pull/2098), [#2088](https://github.com/dartsim/dart/pull/2088), [#2431](https://github.com/dartsim/dart/pull/2431))
  - Reject invalid `World::setTimeStep()` values (NaN, infinity, zero, and negative) at the API boundary to prevent invalid timesteps from reaching constraint solvers. ([#2533](https://github.com/dartsim/dart/pull/2533), [#2531](https://github.com/dartsim/dart/issues/2531), [#2532](https://github.com/dartsim/dart/pull/2532))
  - Fixed `BodyNodePool` alignment handling for over-aligned body nodes so AVX-512 builds no longer fail when Eigen raises `BodyNode` alignment to 64 bytes. ([#2537](https://github.com/dartsim/dart/pull/2537), [#2535](https://github.com/dartsim/dart/issues/2535))
  - Fixed simulation-experimental Debug logging source context so builds without `DART_EXPERIMENTAL_SOURCE_DIR` still report the source file name.
  - Fixed Windows DLL exports for experimental GMSH, OBJ, SEG, and point-set
    mesh importer APIs so C++ callers can link them from shared builds.
  - Fixed intermittent SEGFAULT in TranslationalJoint2D on macOS ARM64 (Release mode) caused by missing `EIGEN_MAKE_ALIGNED_OPERATOR_NEW` on `TranslationalJoint2DUniqueProperties` which contains `Eigen::Matrix<double, 3, 2>`. NEON vectorized instructions require 16-byte alignment that was not guaranteed without this macro.
  - Fixed `TranslationalJoint2D::copy(const TranslationalJoint2D*)` and `UniversalJoint::copy(const UniversalJoint*)` performing self-copy instead of copying from the argument.
  - Fixed weak inverse-kinematics pointer lifetime tracking and `SharedLibraryIkFast` forward-kinematics symbol loading.
  - Fixed `MeshShape::loadMesh(filePath)` so Windows drive-letter paths are converted to valid file URIs before Assimp loading.
  - Fixed intermittent SEGFAULT in `Icosphere::computeIcosahedron()` on macOS ARM64 (Release mode) by removing `static` from the triangles vector, which can cause alignment issues during static initialization when using Eigen aligned allocators.
  - Fix assertion failure crash in JointConstraint when joint limits are invalid (lower > upper). Now emits a warning and skips limit enforcement for that DOF instead of crashing. ([gz-physics#846](https://github.com/gazebosim/gz-physics/issues/846))
  - Added split impulse contact correction and clarified zero-contact handling. ([#2354](https://github.com/dartsim/dart/pull/2354), [#2220](https://github.com/dartsim/dart/pull/2220), [#201](https://github.com/dartsim/dart/issues/201))
  - Added validation for invalid contact surface parameters (NaN/Inf/negative friction, restitution, and slip compliance) to prevent LCP solver crashes. ([gazebosim/gz-physics#841](https://github.com/gazebosim/gz-physics/issues/841))
  - Ignore malformed contacts with null collision objects instead of dereferencing them while creating `ContactConstraint`. ([#2669](https://github.com/dartsim/dart/issues/2669))
  - Fixed slip compliance validation to silently handle the -1.0 sentinel value (meaning "use default") instead of logging spurious warnings. Also use `DART_WARN_ONCE` for genuine NaN/Inf warnings to prevent log spam. ([gazebosim/gz-sim#3289](https://github.com/gazebosim/gz-sim/issues/3289))
  - Warn and continue when LCP matrices are non-symmetric to avoid assertion failures with invalid contacts. ([gz-physics#848](https://github.com/gazebosim/gz-physics/issues/848))
  - Guard against non-finite articulated body computations from zero/epsilon mass or extreme spring values. ([gz-physics#849](https://github.com/gazebosim/gz-physics/issues/849), [gz-physics#850](https://github.com/gazebosim/gz-physics/issues/850), [gz-physics#851](https://github.com/gazebosim/gz-physics/issues/851), [gz-physics#854](https://github.com/gazebosim/gz-physics/issues/854), [gz-physics#856](https://github.com/gazebosim/gz-physics/issues/856))
  - Replace crash-causing `DART_ASSERT` with `DART_WARN` + graceful recovery for non-finite transforms in Joint setters, BodyNode update pipeline, GenericJoint inertia propagation, and MJCF parser validation. ([gz-physics#861](https://github.com/gazebosim/gz-physics/issues/861), [gz-physics#862](https://github.com/gazebosim/gz-physics/issues/862))

- Collision and Geometry
  - Added public DART and dartpy continuous collision queries for swept sphere
    and capsule casts, including detector/group options and results plus native
    swept-AABB broad-phase pruning for world-level cast candidates.
  - Added IPC-class primitive continuous collision detection to the native
    collision backend: conservative, minimum-separation-aware point-triangle and
    edge-edge time-of-impact queries (additive conservative advancement) for
    independently moving vertices, with an exact coplanarity-cubic validation
    path. These are the continuous queries deformable / barrier (IPC-style)
    solvers require and that the rigid-only reference engines do not provide.
  - Added a both-moving rigid convex cast (`convexCast`) to the native backend,
    generalizing conservative advancement so each convex body carries its own
    motion (matching reference-engine continuous collision, which the prior
    single-body `conservativeAdvancement` did not cover).
  - Added a cubic-Bezier spline cast (`splineCast`) covering polynomial motion —
    the one motion model the linear/screw casts cannot represent. The default
    conservative mode finds the true first contact (the only spline cast that
    does so; the reference engine's spline returns a wrong, later contact because
    its bound is non-conservative), and an opt-in `CcdAdvancement::Fast` mode
    trades that no-tunnelling guarantee for larger steps that run 1.9× faster
    than the reference engine's spline cast.
  - Optimized the conservative-advancement loop (an unchanged-orientation fast
    path that skips the per-iteration quaternion slerp for translational sweeps,
    a cached motion sampler that builds endpoint quaternions once per cast instead
    of every iteration, GJK warm-starting that reseeds each closest-point query
    from the previous step's simplex, a templated/de-virtualized GJK query for the
    cast hot path, a directed conservative-advancement bound that projects the
    rotational term onto the closest direction so a coaxial screw's spin no longer
    inflates the step bound, and an acceleration-bounded spline step), so the
    native convex cast beats the reference engines' own continuous-collision
    queries across the swept box, octahedron, sphere, sphere/box, capsule,
    cylinder, rotational, screw, and spline motion cases.
  - Decoupled FCL, Bullet, and ODE collision backends from `libdart.so` into
    explicit test/benchmark reference targets (`dart-test-reference-fcl`,
    `dart-test-reference-bullet`, `dart-test-reference-ode`) while promoting the
    native DART backend to the default detector (`"dart"` factory key). The
    legacy `collision-fcl`, `collision-bullet`, and `collision-ode` package
    components plus installed/source-tree detector headers are native-backed
    compatibility facades; direct C++ legacy facade objects may preserve legacy
    display type strings for gz-physics compatibility, but old engines remain
    available only through `tests/dart/test/reference_collision/` targets and
    `createReference()` APIs for tests and benchmarks during the migration.
  - Marked retained C++ FCL/Bullet/ODE detector facades and legacy factory keys
    as deprecated native-backed compatibility paths through
    `DART_COLLISION_DEPRECATE_LEGACY_NAMES` while keeping explicit
    `dart-test-reference-*` targets for tests and benchmarks.
  - Added native plane/mesh collision dispatch, unbounded PlaneShape AABB transform handling, and regression coverage for mesh-plane contacts through both native tests and the public DART collision group path.
  - Fixed GLTF PBR material import with Assimp 5.x so `MeshShape` preserves
    combined metallic-roughness texture paths on GLTF fixtures.
  - Fixed public DART raycast adapter paths to query mesh-adapted surfaces
    double-sided, including heightmap raycasts through `DartCollisionGroup`.
  - Fixed Atlas Simbicon state terminal-condition ownership so the native
    collision stability integration path exits cleanly under AddressSanitizer.
  - Added collision benchmark regression checks that parse Google Benchmark JSON and compare native collision timings against the best enabled FCL, Bullet, or ODE reference result across narrowphase, distance, raycast, mixed primitive, mesh-heavy, raycast-batch, and public DART adapter scenarios, with a scheduled/manual CI Linux guard that uploads the JSON artifacts.
  - Added deterministic native collision scenario benchmarks for stacked box
    grids, compound box structures, scaled mixed primitives, convex hull stacks,
    terrain contacts, moving raycasts, and Halton-seeded convex cell packs.
  - Added dartpy wheel verification that rejects legacy collision runtime libraries, reference collision libraries, and old reference collision component exports from wheel artifacts while allowing native-backed compatibility component facades.
  - Added a runtime source isolation check that fails if non-reference DART source paths include FCL, Bullet, ODE, libccd, or explicit collision reference backend headers.
  - Removed per-engine FCL/Bullet/ODE collision build switches from the normal
    build surface. Explicit reference test/benchmark gates now build the
    optional `dart-test-reference-*` targets; core `dart`, `dartpy`,
    gz-physics runtime integration, and native-backed
    `collision-fcl`/`collision-bullet`/`collision-ode` compatibility facades
    do not depend on those reference gates.
  - Broke the collision→dynamics source dependency using a bridge pattern: dynamics-dependent implementations moved from `dart/collision/*.cpp` to `dart/dynamics/detail/*_bridge.cpp`. Collision `.cpp` files no longer include any `dart/dynamics/` headers.
  - Added built-in native collision module (`dart/collision/native/`) with 575+ tests covering all primitive shape pairs, GJK/EPA, distance queries, raycast, CCD, four broad-phase algorithms, collision filtering, compound shapes, and parallel narrowphase.
  - Added a Filament-backed native collision sandbox with pair selection, shape
    parameter and pose controls, contact/manifold overlays, broad-phase
    selection, native group/mask filtering, AABB-tree, spatial-hash,
    sweep-and-prune debug rendering, unsupported-pair placeholders, and
    headless screenshot smoke coverage.
  - Added mouse-driven object transform gizmos to the native collision sandbox
    so pair poses can be translated and rotated directly in the scene.
  - Added `dart::gui::applyDebugVisualStyle` for persistent debug
    `ShapeFrame` visuals and updated GUI overlays, including the native
    collision sandbox, to use consistent no-shadow debug styling.
  - Updated Filament debug overlays so gizmos, contacts, AABB edges, and
    selection lines draw through solid scene geometry without writing depth.
  - Made Filament ImGui panels movable after their initial placement, added
    opt-out debug name tags for collision sandbox overlays, and annotated the
    GUI render loop with built-in profiling scopes.
  - Wired the native DART backend as a full `CollisionDetector` implementation with distance queries, raycast, and expanded shape adapters (Cone, Ellipsoid, Heightmap, MultiSphere). The legacy `"experimental"` factory key remains as an alias for compatibility.
  - Added native sparse occupancy-grid storage for `VoxelGridShape`, including native voxel collision support without requiring OctoMap.
  - Optimized native sparse occupancy-grid point-cloud insertion and occupied-cell extraction, added opt-in threaded point-cloud insertion, expanded OctoMap reference-comparison benchmarks, and enabled native `VoxelGridShape` GUI rendering without OctoMap.
  - Updated the `point_cloud` GUI example to use native voxel-grid point-cloud APIs without requiring OctoMap.
  - Allowed installed DART 7 CMake packages to satisfy legacy DART 6.10+ `find_package(DART ...)` requests so pinned gz-physics checkouts can configure without a local source patch.
  - Fixed native capsule-box duplicate filtering to stay pair-local so accumulated collision results do not suppress new contacts in dense worlds.
  - Native collision: added MPR convex penetration and optional libccd parity tests/bench.
  - Native collision: handle degenerate triangle cases in GJK/MPR for robust convex queries.
  - Native collision: improved broad-phase early exit, contact-count tracking, box-box distance, cylinder-box narrow phase, and primitive-mesh traversal performance.
  - Native collision: reduced single-contact result overhead, optimized
    sphere-sphere, sphere-box, capsule-sphere, and capsule-box primitive hot
    paths, batched public DART contact-manifold warm starts, cached unchanged
    public-adapter geometry, and kept public DART collision object transforms
    synchronized through shape-frame transform notifications.
  - Native collision: skipped redundant one-contact public-adapter manifold
    refresh work, fixed scoped profiling macros so profile ranges span the
    caller scope, and made the collision benchmark guard collect production
    timing evidence with profiling instrumentation disabled.
  - Fixed native `CollisionResult::getContact()` to reject stale indices after
    the result is cleared.
  - Native collision: fixed the spatial hash broad phase so unbounded
    `PlaneShape` AABBs are paired without hashing infinite cell coordinates.
  - Native collision: stabilized tilted cylinder contacts against plane-like large boxes, matching gz-physics plane fallback behavior without selecting external collision backends.
  - Native collision: stabilized capsule contacts against plane-like large boxes so slender capsules no longer tunnel through the default world ground path.
  - Native collision: capped large flat box/mesh contact patches to keep gz-physics max-contact selection tests expressive without overwhelming the solver, and preserved gz-required unsupported raycast behavior on FCL/ODE compatibility facades while keeping the built-in native detector behind those names.
  - Native collision: fixed primitive-vs-mesh contact normal orientation for primitive-first query order and added pair-order coverage for capsule/cylinder mesh and convex contacts.
  - Native collision: added cylinder-, convex-, and mesh-vs-SDF distance support with pair-order coverage through the native narrow-phase distance dispatcher.
  - Native collision: fixed box-box contact points for large ground boxes so default DART worlds keep upright and rotated boxes resting on the ground, added stable SAT/face-clipping contact patches for rotated box-ground contacts, and made invalid convex/soft mesh data non-collidable with a warning instead of synthesizing native fallback geometry.
  - Native collision: added scalar-matching batch entry points and benchmark coverage for box-box, sphere-sphere, capsule-capsule, cylinder-cylinder, convex-convex, mesh-mesh, and mixed narrow-phase dispatcher pairs.
  - Restored `BodyNodeDistanceFilter` so distance queries honor body collidability, self-collision, and adjacent-body filtering, and made `DistanceFilter` safely deletable through its base interface.
  - Fixed native sphere-box CCD when the sphere starts inside the expanded box so Debug builds no longer assert before reporting an initial overlap.
  - Fixed native capsule-vs-convex CCD so casts use the full capsule support shape instead of relying on numerically fragile endpoint-sphere hits.
  - Fixed native sphere/capsule-vs-mesh CCD so casts against open or thin mesh
    triangles no longer miss direct hits.
  - Fixed native convex-convex signed-distance penetration witnesses so
    reported nearest points remain finite and separated by the reported
    penetration depth.
  - Fixed native world-level raycast, sphere-cast, and capsule-cast results so returned collision-object handles remain valid after the query returns.
  - Fixed native mesh contact adaptation so public DART contacts preserve mesh
    triangle IDs for soft-contact consumers.
  - Fixed TriMesh-backed `MeshShape` construction with an empty URI so it does not probe `file://` for material metadata, eliminating spurious mesh-load warnings while preserving real URI/path material extraction.
  - Fixed PascalCase ODE compatibility headers to preserve legacy includes such as `OdeCollisionDetector.hpp`. ([#2475](https://github.com/dartsim/dart/pull/2475))
  - Fixed ODE contact history to snapshot current contacts before restoring cached contacts, preventing duplicate or invalid restored contacts from destabilizing capsule-ground simulations. ([#2648](https://github.com/dartsim/dart/pull/2648))
  - Validate `SphereShape::setRadius()` to reject NaN, infinity, and non-positive values with a warning instead of crashing via assertion. ([#2442](https://github.com/dartsim/dart/pull/2442), [gz-physics#843](https://github.com/gazebosim/gz-physics/issues/843))
  - Mesh and shape pipeline upgrades: convex mesh collisions/rendering, polygon mesh support, TriMesh internal representation, Collada unit scaling, aiScene ownership fixes, deep-copy shape cloning, ArrowShape material ownership, heightmap fixes, and allocator/heightmap cleanup. ([#2338](https://github.com/dartsim/dart/pull/2338), [#2340](https://github.com/dartsim/dart/pull/2340), [#2325](https://github.com/dartsim/dart/pull/2325), [#2152](https://github.com/dartsim/dart/pull/2152), [#2285](https://github.com/dartsim/dart/pull/2285), [#2239](https://github.com/dartsim/dart/pull/2239), [#2290](https://github.com/dartsim/dart/pull/2290), [#2305](https://github.com/dartsim/dart/pull/2305), [#2287](https://github.com/dartsim/dart/pull/2287), [#2286](https://github.com/dartsim/dart/pull/2286), [#2069](https://github.com/dartsim/dart/pull/2069), [#453](https://github.com/dartsim/dart/issues/453), [#872](https://github.com/dartsim/dart/issues/872), [#896](https://github.com/dartsim/dart/issues/896), [#287](https://github.com/dartsim/dart/issues/287))
  - Collision backend fixes across FCL/Bullet/ODE: contact normalization, mesh regressions, contact ordering, plane handling, phantom contact filtering, ellipsoid rolling, sphere containment, and capsule contact stabilization. ([#2339](https://github.com/dartsim/dart/pull/2339), [#2258](https://github.com/dartsim/dart/pull/2258), [#2282](https://github.com/dartsim/dart/pull/2282), [#2243](https://github.com/dartsim/dart/pull/2243), [#2276](https://github.com/dartsim/dart/pull/2276), [#2246](https://github.com/dartsim/dart/pull/2246), [#2274](https://github.com/dartsim/dart/pull/2274), [#2271](https://github.com/dartsim/dart/pull/2271), [#2219](https://github.com/dartsim/dart/pull/2219), [#2125](https://github.com/dartsim/dart/pull/2125), [#1539](https://github.com/dartsim/dart/issues/1539), [#1184](https://github.com/dartsim/dart/issues/1184))
  - Collision utilities and filtering updates: raycast filters, collision shape signals, collision filter guards, MetaSkeleton subscription handling, contact helpers, and optional backend embedding. ([#2279](https://github.com/dartsim/dart/pull/2279), [#2250](https://github.com/dartsim/dart/pull/2250), [#2343](https://github.com/dartsim/dart/pull/2343), [#2277](https://github.com/dartsim/dart/pull/2277), [#2245](https://github.com/dartsim/dart/pull/2245), [#2194](https://github.com/dartsim/dart/pull/2194))
  - Ensure Bullet double-precision builds include `dart/collision/bullet/BulletInclude.hpp` before Bullet headers so `BT_USE_DOUBLE_PRECISION` is honored: [#2334](https://github.com/dartsim/dart/pull/2334).

- LCP and Optimization
  - Added `dart::constraint::CylindricalJointConstraint` for runtime
    slide-and-rotate attachments that leave translation along an axis and
    rotation about that axis free, with dartpy bindings, unit tests, a focused
    headless smoke example, and a consolidated `dart-demos` dynamic joint
    constraint catalog.
  - Added dimension validation to `Problem::setDimension()` to reject dimensions exceeding `Problem::kMaxDimension` (1,000,000) with `InvalidArgumentException` instead of attempting multi-GB allocations that cause ASan aborts or `std::bad_alloc`. ([#2500](https://github.com/dartsim/dart/issues/2500))
  - Refactored the LCP solver stack under `dart::math::lcp` with unified APIs plus new solver implementations and ImGui dashboard support. ([#2202](https://github.com/dartsim/dart/pull/2202), [#2241](https://github.com/dartsim/dart/pull/2241), [#2318](https://github.com/dartsim/dart/pull/2318), [#2355](https://github.com/dartsim/dart/pull/2355))
  - Added opt-in solver-internal CPU worker threads for the Red-Black
    Gauss-Seidel and Blocked Jacobi LCP solvers, with focused correctness tests
    and benchmark counters for threaded color/block updates.
  - Modernized the Dantzig solver and improved stability (C++20 optimizations, NaN fallback, warning suppression). ([#2081](https://github.com/dartsim/dart/pull/2081), [#2253](https://github.com/dartsim/dart/pull/2253), [#2111](https://github.com/dartsim/dart/pull/2111))
  - Fixed Lemke solver segfaults on macOS arm64 by avoiding Eigen stack allocations. ([#2462](https://github.com/dartsim/dart/pull/2462))
  - Test coverage audit with Codecov infrastructure improvements and 200+ new unit/integration tests across common, collision, constraint, dynamics, io, sensor, and simulation modules. ([#2462](https://github.com/dartsim/dart/pull/2462))
  - Added LCP documentation. ([#2240](https://github.com/dartsim/dart/pull/2240))

- IO and Parsing
  - MJCF parser improvements for MuJoCo 3.x compatibility: fixed `limited` attribute parsing to use `std::optional<bool>`, added `autolimits` compiler attribute (default: true), added `<freejoint>` element support, wired `<option>` timestep/gravity to World, mapped joint stiffness/frictionloss to DART spring/friction APIs, added unknown element warnings for unsupported MJCF elements, and fixed null pointer crash for unnamed joints on root bodies.
  - MJCF `<actuator>` parsing: motor, position, velocity, and general actuator types are mapped to DART Joint actuator types (FORCE, SERVO, VELOCITY) with force limits scaled by gear ratio. Includes `ActuatorAttributes` for shared attributes (dyntype, gaintype, biastype, forcelimited, forcerange, gear, ctrlrange).
  - MJCF `<contact><exclude>` collision filtering: body-pair exclusions are mapped to DART's `BodyNodeCollisionFilter` blacklist on the world's constraint solver.
  - MJCF `<asset>` texture and material parsing: `<texture>` (name, type, file, builtin, rgb1/rgb2, width/height) and `<material>` (name, texture, rgba, emission, specular, shininess, reflectance) elements are parsed and accessible by name or index.
  - MJCF `<body>` camera and light parsing: `<camera>` (fovy, pos, mode, target, quat) and `<light>` (pos, dir, active, diffuse, specular, directional, castshadow) elements are parsed as child elements of bodies.
  - Fixed `Option::getApiRate()` and `Option::getImpRatio()` returning the wrong value (both returned timestep).
  - Fixed MJCF default weld inheritance, `noslip_iterations` parsing, and worldbody site parsing/processing. Also fixed unsupported-root fallback and Windows relative include resolution in SDF parsing, removed incomplete HTTP cache files after failed downloads, and added SKEL recognition for screw and translational2d joints. ([#2648](https://github.com/dartsim/dart/pull/2648))
  - Fix null pointer dereference in XmlHelpers getValue\* functions when child element is missing. ([#2428](https://github.com/dartsim/dart/pull/2428))
  - Return safe defaults from XML attribute helpers when callers pass a null element. ([#2678](https://github.com/dartsim/dart/issues/2678))
  - Unified model loading under `dart::io` and added HTTP retriever support. ([#2316](https://github.com/dartsim/dart/pull/2316), [#2138](https://github.com/dartsim/dart/pull/2138), [#604](https://github.com/dartsim/dart/issues/604))
  - SDF/URDF parsing improvements: libsdformat integration, SDF mimic metadata, SDF joint limits, tiny inertial handling, URDF multi-DoF limits, and transmission coupling. ([#2154](https://github.com/dartsim/dart/pull/2154), [#2254](https://github.com/dartsim/dart/pull/2254), [#2232](https://github.com/dartsim/dart/pull/2232), [#2284](https://github.com/dartsim/dart/pull/2284), [#2233](https://github.com/dartsim/dart/pull/2233), [#2281](https://github.com/dartsim/dart/pull/2281), [#264](https://github.com/dartsim/dart/issues/264))
  - Parser naming cleanup: renamed `DartLoader` to `UrdfParser` and standardized parser variable names. ([#2269](https://github.com/dartsim/dart/pull/2269), [#2270](https://github.com/dartsim/dart/pull/2270))
  - Resource handling updates: deprecated `ResourceRetriever::getFilePath()` and fixed aiScene ownership with shared_ptr deleters. ([#2217](https://github.com/dartsim/dart/pull/2217), [#2285](https://github.com/dartsim/dart/pull/2285))
  - Documented SKEL as a legacy format.

- GUI and Rendering
  - Added the Filament-backed GUI path with backend-hidden `dart::gui` scene
    descriptors, bounded/headless smoke support,
    debug overlays, selection/manipulation helpers, constrained dartpy bindings,
    a pinned explicit fetch fallback, and GCC/Clang smoke coverage for local and
    CI validation.
    ([#2647](https://github.com/dartsim/dart/pull/2647))
  - Added a Filament-rendered Unitree G1 scene and routed the in-tree
    `g1_puppet` example runner through that Filament path with visible IK
    targets for hand and foot manipulation.
  - Extended Filament GUI descriptor and renderer coverage to
    `PyramidShape`, `MultiSphereConvexHullShape`, `LineSegmentShape`, and
    `ConvexMeshShape`, `PointCloudShape`, `HeightmapShape`, and
    `SoftMeshShape`, plus `VoxelGridShape` when OctoMap is available, with the
    MVP scene validating that all enabled shape fixtures are extracted and
    rendered, `MeshShape` triangle, material, UV, normal, and submesh metadata
    exposed through descriptors,
    nearest-pick results reporting bounds hit normals plus primitive
    sphere/ellipsoid/cylinder/capsule/cone/pyramid/multi-sphere/point-cloud/voxel-grid/plane
    hit normals and triangle-backed convex-mesh/heightmap/soft-mesh/MeshShape
    hit normals, descriptor-owned triangle rendering for convex-mesh,
    heightmap, soft-mesh, and `MeshShape` descriptors, unsupported shapes producing diagnostic descriptors, and backend-hidden renderable set update planning
    for add/remove/visibility and render-resource synchronization, including
    dynamic soft-mesh vertex changes and diagnostics for supported descriptors
    that cannot produce renderer resources, and retained Filament renderables
    now synchronize changed descriptor shadow flags. `MeshShape` alpha-mode
    policy is also exposed through descriptors and visual alpha changes now
    invalidate renderer resources when material transparency can change. The
    Filament point-cloud path also consumes per-point color descriptors, and
    the public GUI headers now have unit coverage that guards against
    leaking Filament, GLFW, Dear ImGui, OpenGL, Vulkan, Metal, OSG, or Raylib
    implementation tokens. The Filament example now includes the `dart::gui`
    API directly instead of carrying an example-local namespace re-export
    header, and checked-in dartpy stubs and API docs now expose `dartpy.gui`
    without backend-specific names.
    Orbit-camera controller state, perspective projection, near/far clipping
    policy, GUI scale normalization, and camera-relative keyboard nudge math now
    also live in a dedicated `dart/gui/viewer.hpp` runtime helper
    and are consumed by the Filament example through `dart-gui-core`.
    Axis-constrained drag translation now also lives in the backend-hidden
    interaction helpers with C++ and Python coverage, and the Filament example
    uses it for Ctrl-X/Y/Z-left constrained selection dragging.
    The backend-hidden implementation is now split by responsibility across
    `viewer.cpp`, `debug.cpp`, `geometry.cpp`, `interaction.cpp`,
    `shape_descriptions.cpp`, and `profile.cpp`, with focused
    headers for renderable, interaction, debug, geometry, and viewer concepts
    while `scene.hpp` remains the scene descriptor aggregate. Filament renderer
    setup, frame lifecycle, material
    and texture resources, descriptor synchronization, selection state, input
    translation, simulation stepping, smoke-test registration, material header
    generation, and reusable scene fixtures now live under private
    `dart/gui/detail` implementation units; scene option
    parsing and dispatch remain in `scenes.cpp`, while fixture construction is
    split into `scene_fixtures.cpp`. The `simple_frames` runner now routes to
    a `--scene simple-frames` Filament fixture that renders its `SimpleFrame`
    hierarchy, marker ellipsoids, and arrow marker through backend-hidden
    descriptors, and the `soft_bodies` runner now routes to a
    `--scene soft-bodies` fixture that loads `softBodies.skel` through
    backend-hidden soft-mesh descriptors. The `point_cloud` runner now routes
    to a `--scene point-cloud` fixture that renders point-cloud and voxel-grid
    descriptors through Filament, and the `capsule_ground_contact` runner now
    routes to a `--scene capsule-ground-contact` fixture that renders capsule
    and ground descriptors through Filament. The `simulation_event_handler`
    runner now routes to a `--scene simulation-event-handler` fixture that
    renders falling bodies and sensor marker descriptors through Filament. The
    `imgui` runner now routes to the same Filament viewer and private built-in
    status panel. The `rigid_shapes` runner now routes to the default Filament
    MVP fixture for broad primitive, mesh, point-cloud, heightmap, soft-mesh,
    and robot visual coverage. The `rigid_cubes` runner now routes to the
    `--scene boxes` Filament fixture for dynamic cube-grid coverage. The
    `box_stacking` runner now routes to the same Filament boxes fixture. The example tree is left as a minimal
    `main.cpp` entry point plus CMake and README files. The `empty` runner now
    routes to the `--scene drag-and-drop` Filament fixture for minimal
    interactive-frame coverage. The `hardcoded_design` runner now routes to a
    `--scene hardcoded-design` Filament fixture for a hand-built three-link
    skeleton. The `rigid_chain` runner now routes to a `--scene rigid-chain`
    Filament fixture for SKEL-loaded chain renderables. The `rigid_loop`
    runner now routes to a `--scene rigid-loop` Filament fixture for
    constrained SKEL-loaded chain renderables. The `mixed_chain` runner now
    routes to a `--scene mixed-chain` Filament fixture for mixed rigid and soft
    chain renderables. The `coupler_constraint` runner now routes to a
    `--scene coupler-constraint` Filament fixture for paired mimic/coupler rig
    renderables. The `add_delete_skels` runner now routes to a
    `--scene add-delete-skels` Filament fixture for deterministic
    add/delete-style cube skeleton renderables. The `vehicle` runner now routes
    to a `--scene vehicle` Filament fixture for the SKEL-loaded car, wheel,
    ground, and obstacle renderables. The `hybrid_dynamics` runner now routes
    to a `--scene hybrid-dynamics` Filament fixture for the posed fullbody
    humanoid and ground renderables. The `biped_stand` runner also routes to
    that Filament fixture for a standing fullbody visual workflow.
    The `mimic_pendulums` runner now routes to a `--scene mimic-pendulums`
    Filament fixture for the SDF-loaded pendulum rigs, base poles, and ground
    renderables. The `atlas_puppet` runner now routes to a
    `--scene atlas-puppet` Filament fixture that loads Atlas, exposes generic
    selectable IK targets for its hands and feet.
    The `hubo_puppet` runner now routes to a `--scene hubo-puppet` Filament
    fixture that loads Hubo, exposes selectable IK targets for its hands, feet,
    and wrist pegs.
    The `atlas_simbicon` runner now routes to a `--scene atlas-simbicon`
    Filament fixture that loads the legacy Atlas SDF and ground in the
    Simbicon start orientation.
    The `operational_space_control` runner now routes to a
    `--scene operational-space-control` Filament fixture that loads the WAM arm,
    runs the task-space controller through a private scene pre-step hook, and
    exposes the red target as a selectable `SimpleFrame`.
    The `wam_ikfast` runner now routes to a `--scene wam-ikfast` Filament
    fixture that loads the WAM arm, ground, and end-effector target through
    descriptor-owned renderables.
    The `fetch` runner now routes to a `--scene fetch` Filament fixture that
    loads the legacy MJCF pick-and-place world, applies the initial robot and
    object pose, and exposes the target frame through descriptor-owned
    renderables.
    The `tinkertoy` runner now routes to a `--scene tinkertoy` Filament fixture
    that recreates the legacy builder's initial block assemblies, target
    marker, force line, and reference axes through descriptor-owned renderables.
    The `joint_constraints` runner now routes to a `--scene joint-constraints`
    Filament fixture that loads the legacy fullbody world and runs the SPD
    balance controller through the same private scene pre-step hook.
    The `free_joint_cases` runner now routes to a `--scene free-joint-cases`
    Filament fixture for the zero-gravity free-joint bodies and transparent
    torque-free reference boxes.
    The `human_joint_limits` runner now routes to a
    `--scene human-joint-limits` Filament fixture for the Kima human visual,
    DART joint-limit enforcement, and descriptor-owned mesh, multi-sphere,
    box, and ground renderables.
    The `lcp_physics` runner now routes to a `--scene lcp-physics` Filament
    fixture for deterministic mass-ratio, stacking, domino, ball-drop, and
    ground contact renderables.
    The focused GUI unit test now checks that `examples/dartsim` contains
    no C++ source/header files other than `main.cpp`, that the example has no
    direct Filament header includes, and that `main.cpp` stays a single-include
    delegated entry point without backend implementation tokens. The Filament
    example CMake helper now also compiles only that entry point and rejects
    unexpected example-tree regular files or direct Filament entry-point
    includes at configure time. The
    `run_cpp_example.py` unit coverage now also checks that migrated visual
    runners resolve to their Filament scenes instead of stale legacy binaries,
    and that the `--scene all` runner list, smoke-test regex, and
    loop-generated CMake Filament smoke scene registrations stay aligned.
    All generated Filament headless scene smokes now run the screenshot
    analyzer in a basic full-image nonzero mode, while the default shadowed
    fixture keeps its stricter luminance-contrast gate. Focused Python unit
    coverage now checks the analyzer's basic mode, contrast mode, and default
    CLI mode against synthetic PPM fixtures, and the example-runner tests guard
    that the CMake smoke registration keeps those analysis modes wired.
    The north-star audit now records the removed OSG/Raylib GUI surfaces and
    the remaining Filament-only renderer ownership boundary. The renderer-hidden debug descriptor path
    also covers
    support-polygon outlines
    and support-centroid markers, plus arrowheads for contact normal and force
    overlays, equivalent inertia-box overlays, and collision-shape bounds for
    collision-only shape nodes.
  - Fixed Filament renderable bounds and voxel-grid transparency handling, and
    cleaned up the `point_cloud` example so native voxel occupancy is visible
    without the optional source-owned grid overlay by default.
  - Fix SEGV in `ImFontAtlas::AddFontFromMemoryCompressedTTF` when null pointer is passed as compressed font data. ([#2516](https://github.com/dartsim/dart/issues/2516))
  - Fix `ImGui::ColorPicker3`/`ColorPicker4` crashes when called without an active ImGui context or window. ([#2668](https://github.com/dartsim/dart/issues/2668))
  - Added headless rendering support via `ViewerConfig` and pbuffer graphics context for CI pipelines and batch frame capture. Includes `Viewer::captureBuffer()` for raw RGBA pixel readback and a new `headless-rendering` CI job. ([#2466](https://github.com/dartsim/dart/pull/2466))
  - Added `ImGuiViewer` construction from `ViewerConfig` to support headless ImGui rendering and example frame capture workflows.
  - GUI naming updates and backend cleanup (including the osg suffix removal). ([#2209](https://github.com/dartsim/dart/pull/2209), [#2257](https://github.com/dartsim/dart/pull/2257))
  - ImGui integration updates: FetchContent, Vulkan detection/loader fixes, GUI scale controls, and ImGuiWidget subclassing. ([#2056](https://github.com/dartsim/dart/pull/2056), [#2085](https://github.com/dartsim/dart/pull/2085), [#2261](https://github.com/dartsim/dart/pull/2261), [#2280](https://github.com/dartsim/dart/pull/2280), [#2356](https://github.com/dartsim/dart/pull/2356))
  - Rendering updates: PolyhedronVisual attachments and VisualAspect color handling. ([#2214](https://github.com/dartsim/dart/pull/2214), [#2230](https://github.com/dartsim/dart/pull/2230))
  - Skip `FontGlobalScale` when ImGui font atlases are rebuilt for GUI scaling, normalize framebuffer scale detection to avoid downscaling on HiDPI setups, and scale lcp_physics ImGui panel sizing with font size.

- Core
  - Added SIMD abstraction layer (`dart/simd/`) with portable vectorized math primitives supporting SSE4.2, AVX, AVX2, AVX-512, and ARM NEON backends with automatic runtime dispatch. Includes `Vec<T, N>`, `VecMask<T, N>`, aligned allocators, Eigen interop utilities, and dynamic vector/matrix types for batch computation. ([#2490](https://github.com/dartsim/dart/pull/2490))
  - Added a backend-neutral lie-group batch adjoint helper and fixed SO3, SE3,
    and group-product log Jacobian plumbing so DART 7 math paths have tested
    scalar baselines ready for SIMD, multi-threaded, and CUDA compute backends.
  - Added `<numbers>`-style variable templates (`dart::math::pi`, `phi`, `two_pi`, etc.) plus numeric-limits helpers (`inf_v`, `max_v`, `min_v`, `eps_v`) in `dart/math/Constants.hpp` and deprecated `dart::math::constants<T>` (the legacy struct/header will be removed in DART 7.1). ([#2150](https://github.com/dartsim/dart/pull/2150), [#2157](https://github.com/dartsim/dart/pull/2157), [#2225](https://github.com/dartsim/dart/pull/2225))
  - Fix spdlog/fmt 12 builds by treating DART logging format parameters as runtime format strings. ([#2542](https://github.com/dartsim/dart/pull/2542), [#2538](https://github.com/dartsim/dart/issues/2538))
  - Logging and profiling updates: conditional logging macros, source-context metadata, `DART_ASSERT` adoption, log prefix cleanup, and the text profiling backend. ([#2099](https://github.com/dartsim/dart/pull/2099), [#2104](https://github.com/dartsim/dart/pull/2104), [#2105](https://github.com/dartsim/dart/pull/2105), [#2109](https://github.com/dartsim/dart/pull/2109), [#2110](https://github.com/dartsim/dart/pull/2110), [#2238](https://github.com/dartsim/dart/pull/2238))
  - Core API organization updates: dart7 core library scaffold, per-module forward declaration headers, IkFast header relocation, and C++20 modernization. ([#2097](https://github.com/dartsim/dart/pull/2097), [#2195](https://github.com/dartsim/dart/pull/2195), [#2057](https://github.com/dartsim/dart/pull/2057), [#2073](https://github.com/dartsim/dart/pull/2073))
  - Core API fixes: BodyNode transform derivative APIs, frame transform helpers for Inertia, COM-based potential energy, signal thread-safety, trivial accessor noexcept annotations, and non-finite joint input asserts. ([#2131](https://github.com/dartsim/dart/pull/2131), [#2153](https://github.com/dartsim/dart/pull/2153), [#2224](https://github.com/dartsim/dart/pull/2224), [#2181](https://github.com/dartsim/dart/pull/2181), [#2218](https://github.com/dartsim/dart/pull/2218), [#2273](https://github.com/dartsim/dart/pull/2273))
  - Fixed `PoolAllocator` leaking old memory block tables when the table grows.
  - Fixed `MultiLockableReference::try_lock()` leaving earlier acquired locks
    held when a later lock acquisition fails.
  - Removed all APIs deprecated in DART 6.0 (legacy BodyNode collision flags, Skeleton self-collision aliases, Joint `getLocal*`/`updateLocal*` accessors, `World::checkCollision(bool)`, `ConstraintSolver::setCollisionDetector(raw*)`, Marker `getBodyNode()`, `SdfParser::readSdfFile`, and deprecated XML helpers). ([#2132](https://github.com/dartsim/dart/pull/2132))
  - Removed all APIs deprecated in DART 6.1. ([#2119](https://github.com/dartsim/dart/pull/2119))
  - Removed all APIs deprecated in DART 6.2 (legacy Entity/BodyNode/JacobianNode/Joints/Skeleton notifiers, `Shape::notify*Update`, `EllipsoidShape::getSize`/`setSize`, `MultiSphereShape` alias, and `Eigen::make_aligned_shared` alias). ([#2122](https://github.com/dartsim/dart/pull/2122))
  - Removed `CollisionFilter::needCollision()` (deprecated in DART 6.3). ([#2126](https://github.com/dartsim/dart/pull/2126))
  - Removed `DART_COMMON_MAKE_SHARED_WEAK` macro (deprecated in DART 6.4). ([#2127](https://github.com/dartsim/dart/pull/2127))
  - Removed all APIs deprecated in DART 6.7 (legacy math random helpers, `Skeleton::clone()` overloads, and `ConstraintSolver::set/getLCPSolver()`). ([#2133](https://github.com/dartsim/dart/pull/2133))
  - Removed all APIs deprecated in DART 6.8. ([#2136](https://github.com/dartsim/dart/pull/2136))
  - Removed all APIs deprecated in DART 6.9 (`dart::common::make_unique`, `FreeJoint::setTransform` static helpers, and `NloptSolver` overloads taking raw `nlopt::algorithm` values). ([#2139](https://github.com/dartsim/dart/pull/2139))
  - Removed all APIs deprecated in DART 6.10 (common::Signal::cleanupConnections, `SharedLibrary`/`SharedLibraryManager` filesystem-path overloads, BodyNode friction/restitution helpers and aspect properties, and `Joint::{set,is}PositionLimitEnforced()` aliases). ([#2140](https://github.com/dartsim/dart/pull/2140))
  - Removed all APIs deprecated in DART 6.11 (`DartLoader::Flags` and the `parseSkeleton`/`parseWorld` overloads that accepted explicit resource retrievers and flag arguments). ([#2141](https://github.com/dartsim/dart/pull/2141))
  - Removed all APIs deprecated in DART 6.12 (the `SdfParser::readWorld`/`readSkeleton` overloads that accepted direct `ResourceRetriever` parameters). ([#2142](https://github.com/dartsim/dart/pull/2142))
  - Removed all APIs deprecated in DART 6.13 (the legacy `dart::common::Timer` utility, `ConstraintSolver::getConstraints()`/`containSkeleton()`, `ContactConstraint`'s raw constructor and material helper statistics, and the `MetaSkeleton` vector-returning `getBodyNodes()`/`getJoints()` accessors). ([#2143](https://github.com/dartsim/dart/pull/2143))
  - Removed the legacy optional shim. ([#2137](https://github.com/dartsim/dart/pull/2137))
  - Removed the final compatibility headers that only re-included their replacements (`dart/collision/Option.hpp`, `dart/collision/Result.hpp`, and `dart/dynamics/MultiSphereShape.hpp`) and scrubbed the remaining deprecated documentation strings.
  - Removed the remaining 6.13 compatibility shims: deleted `dart/utils/urdf/URDFTypes.hpp`, the Eigen alias typedefs in `math/MathTypes.hpp`, the `dart7::comps::NameComponent` alias, and the legacy `dInfinity`/`dPAD` helpers, and tightened `SkelParser` plane parsing to treat `<point>` as an error.
  - Fixed iterator invalidation in `Subject::sendDestructionNotification()`, `Observer::~Observer()`, and `Observer::removeAllSubjects()` that caused non-deterministic SEGFAULT on macOS arm64 Debug builds when an observer callback mutated the observer/subject set during iteration.

- dartpy
  - Added bindings for `dynamics::EndEffector` (including the `Support` aspect) and exposed `BodyNode::createEndEffector`/`getEndEffector` plus the `Skeleton::getEndEffector` overloads to unblock the Atlas puppet Python example and IK tests.
  - Nanobind migration and namespace flattening (including dartpy7 scaffold and cleanup of pybind11 artifacts). ([#2090](https://github.com/dartsim/dart/pull/2090), [#2249](https://github.com/dartsim/dart/pull/2249), [#2256](https://github.com/dartsim/dart/pull/2256), [#2259](https://github.com/dartsim/dart/pull/2259))
  - Binding improvements: joint bindings, lifetime fixes, MI casting, vector conversions with GIL release, repr bindings, and ImGuiWidget subclassing. ([#2272](https://github.com/dartsim/dart/pull/2272), [#2319](https://github.com/dartsim/dart/pull/2319), [#2321](https://github.com/dartsim/dart/pull/2321), [#2328](https://github.com/dartsim/dart/pull/2328), [#2297](https://github.com/dartsim/dart/pull/2297), [#2356](https://github.com/dartsim/dart/pull/2356))
  - dartpy documentation and tutorials: Python API docs, RTD fixes, and Atlas IK tutorial material. ([#2045](https://github.com/dartsim/dart/pull/2045), [#2182](https://github.com/dartsim/dart/pull/2182), [#2184](https://github.com/dartsim/dart/pull/2184), [#2262](https://github.com/dartsim/dart/pull/2262), [#2299](https://github.com/dartsim/dart/pull/2299), [#2159](https://github.com/dartsim/dart/pull/2159), [#2176](https://github.com/dartsim/dart/pull/2176))
  - dartpy packaging updates: Windows wheel builds, Windows CI toggles, removal of dartpy8 bindings, and dart7/dartpy7 prerelease renames. ([#2266](https://github.com/dartsim/dart/pull/2266), [#2268](https://github.com/dartsim/dart/pull/2268), [#2091](https://github.com/dartsim/dart/pull/2091), [#2206](https://github.com/dartsim/dart/pull/2206), [#2148](https://github.com/dartsim/dart/pull/2148), [#2164](https://github.com/dartsim/dart/pull/2164))

- Gazebo Integration
  - Upgraded the Gazebo integration to gz-physics9, restored DART 7 compatibility, and fixed gtest header mismatches. ([#2093](https://github.com/dartsim/dart/pull/2093), [#2315](https://github.com/dartsim/dart/pull/2315), [#2320](https://github.com/dartsim/dart/pull/2320))
  - Aligned mimic and coupler constraints with Gazebo repros and parsed SDF mimic joints. ([#2247](https://github.com/dartsim/dart/pull/2247), [#2254](https://github.com/dartsim/dart/pull/2254))

- Examples and Tutorials
  - Added a new HTTP retriever and G1 puppet example, plus refreshed demo media. ([#2138](https://github.com/dartsim/dart/pull/2138))
  - Added the experimental Raylib example, PolyhedronVisual example, and rolling cylinder demo. ([#2310](https://github.com/dartsim/dart/pull/2310), [#2214](https://github.com/dartsim/dart/pull/2214), [#2353](https://github.com/dartsim/dart/pull/2353))
  - Replaced the legacy Atlas v3 sample data with an Atlas v5 no-head URDF sample model and moved the Atlas puppet, Simbicon, whole-body IK, benchmark, and visual smoke fixtures to it.
  - Added Atlas IK dartpy bindings/tutorials and control theory + servo primers. ([#2176](https://github.com/dartsim/dart/pull/2176), [#2303](https://github.com/dartsim/dart/pull/2303), [#2156](https://github.com/dartsim/dart/pull/2156))
  - Fixed python example discovery and added the Issue #743 regression. ([#2311](https://github.com/dartsim/dart/pull/2311), [#743](https://github.com/dartsim/dart/issues/743))
  - Added explicit placeholder bodies to unfinished domino and biped Python tutorials so users can import/run the scaffolds without `IndentationError`s.
  - Reoriented the Y-up `pixi run py-demos` scenes (`rigid_chain`, `rigid_cubes`, `mixed_chain`, `add_delete_skels`, `rigid_loop`, `soft_bodies`, `kr5_arm`) to the canonical Z-up convention so gravity and the camera up-vector are consistent across the catalog; golden-set parity is preserved because geometry and gravity are rotated by the same rigid transform.
  - Reoriented the matching Y-up C++ `pixi run demos` scenes to Z-up via shared `reorientWorldToZUp`/`reorientSkeletonToZUp` helpers, keeping the C++ golden-set parity smoke green. Covered: `rigid_chain`, `rigid_cubes`, `mixed_chain`, `add_delete_skels`, `rigid_loop`, `soft_bodies`, `vehicle`, `hybrid_dynamics`, `human_joint_limits`, `rigid_shapes`, `lcp_physics`, and the balance-controller scenes `biped_stand` and `joint_constraints`. The biped controllers compute balance from the sagittal (X) center-of-mass/pressure offset and track joint-space targets, both invariant under the rigid `RotX(+90°)` rotation; per-scene touch-ups flipped Y-up cameras to Z-up and redirected the lateral push perturbations from world `Z` to `Y`.
  - Fixed `atlas_simbicon`: its model and SIMBICON controller are Z-up, but the sagittal torso-balance gain was inverted (a leftover from the Y-up→Z-up port), so the Atlas toppled backward. Corrected the sign (`State::_updateTorqueForStanceLeg`, both stance branches) so the pelvis feedback is restorative; the Atlas now balances upright. The coronal term was already correct.
  - Added a robot-agnostic Python SIMBICON controller to `pixi run py-demos` with new `atlas_simbicon`, `g1_simbicon`, and `simbicon_duo` scenes. A single config-driven controller (`scenes/_simbicon.py`) implements the SIMBICON FSM, world-frame torso/swing-hip control, and the `theta_d = theta_d0 + c_d*d + c_v*v` balance feedback, parameterized per robot (`scenes/_simbicon_robots.py`); both the bundled Atlas v5 and the (locally cached) Unitree G1 balance/step under it, including together in one world. Uses the paper's hip-midpoint COM proxy (dartpy exposes no `getCOM`), a foot-height contact proxy, and Stable PD so stiff gains stay stable on G1's light, low-inertia joints.
  - Improved the Python SIMBICON balance robustness with two evidence-driven control fixes. State traces showed both robots fail the same way: the stance leg creeps into a slightly deeper crouch each cycle so the pelvis gradually sinks until it drops out of the control window. (1) Added stance-leg height regulation (`height_kp`) that extends the stance knee back toward the standing pelvis height captured at spawn, and (2) completed the world-frame torso control as a true PD by wiring in the previously-unused `torso_kd` damping term (derived from the pelvis angular velocity). With `height_kp=2.0`, Atlas balances and steps in place for 3000+ steps (it toppled around step 2800 before) and G1's time-to-fall nearly doubles (1170 → 2204 steps). A residual lateral (coronal) instability still topples both over longer horizons; sweeping the coronal feedback gain and a per-side sign flip did not improve it, so the deeper lateral foot-placement work is left as a documented limit rather than masked.
  - Made `dart::gui::runDemos`/`--scene` accept both hyphenated and snake_case scene ids (`atlas-simbicon` and `atlas_simbicon`) and print the available scenes on an unknown id instead of silently starting the first scene, which makes headless screenshot capture reliable.
  - Hardened `pixi run py-demos` runtime scene switching: Python scene
    builders and startup `pre_step` callbacks now share the demo startup
    watchdog, and switched demos that throw, fail render-state creation, or
    return their first frame over budget restore the previous active demo
    instead of leaving the workspace stuck on the requested scene. Pending
    sidebar switches can also be retargeted by clicking a different demo before
    the candidate starts.
  - Tightened Python demo visual debugging: the docked `Simulation` panel now
    uses compact transport controls for simulation and recorded-frame playback,
    and `py-demo-capture --show-ui` rejects captures without the docked ImGui
    workspace while dropping warm-up frames before the UI is visible.
  - Made sx Python demo external-force panels target-aware: they now list
    mapped dynamic drag targets before a drag starts while continuing to report
    disabled, static, unmapped, invalid, and applying states.
  - Added an `Experimental focus` toggle to the Python demos navigator. When
    the active scene is an sx/experimental demo, the sidebar opens focused on
    simulation-experimental categories while still allowing users to browse the
    full legacy DART API catalog by unchecking the toggle.
  - Kept docked Python demo panes resizable across runtime demo switches by
    storing dock-layout initialization in the viewer lifecycle and clearing
    no-resize flags from the default dock nodes.
  - Updated the GLFW-backed ImGui input bridge so docked Python demo pane
    edges show the appropriate resize cursors while preserving resizable pane
    behavior.
  - Forwarded GLFW keyboard and character input into the ImGui bridge so the
    docked Python demos sidebar search box accepts typed text while app-level
    shortcuts yield to focused text fields.

- Tests
  - Test organization and naming updates: reorganized test directories, normalized PascalCase names, and split integration test binaries. ([#2071](https://github.com/dartsim/dart/pull/2071), [#2116](https://github.com/dartsim/dart/pull/2116), [#2193](https://github.com/dartsim/dart/pull/2193), [#2210](https://github.com/dartsim/dart/pull/2210), [#2260](https://github.com/dartsim/dart/pull/2260))
  - Coverage expansions across math, collision, constraints, and ShapeNode inertia. ([#2162](https://github.com/dartsim/dart/pull/2162), [#2167](https://github.com/dartsim/dart/pull/2167), [#2169](https://github.com/dartsim/dart/pull/2169), [#2170](https://github.com/dartsim/dart/pull/2170), [#2175](https://github.com/dartsim/dart/pull/2175), [#2187](https://github.com/dartsim/dart/pull/2187), [#2286](https://github.com/dartsim/dart/pull/2286))
  - Regression and backend-specific tests: plane shape collision coverage, Bullet box stacking, FCL thin-plane regressions, FCL mesh contact regressions, servo/mimic consistency, PlaneShape GUI smoke tests, and uninitialized Isometry fixes. ([#2092](https://github.com/dartsim/dart/pull/2092), [#2227](https://github.com/dartsim/dart/pull/2227), [#2229](https://github.com/dartsim/dart/pull/2229), [#2276](https://github.com/dartsim/dart/pull/2276), [#2258](https://github.com/dartsim/dart/pull/2258), [#2263](https://github.com/dartsim/dart/pull/2263), [#2283](https://github.com/dartsim/dart/pull/2283), [#2350](https://github.com/dartsim/dart/pull/2350), [#2342](https://github.com/dartsim/dart/pull/2342), [#2130](https://github.com/dartsim/dart/pull/2130), [#2120](https://github.com/dartsim/dart/pull/2120), [#2281](https://github.com/dartsim/dart/pull/2281), [#870](https://github.com/dartsim/dart/issues/870), [#915](https://github.com/dartsim/dart/issues/915), [#410](https://github.com/dartsim/dart/issues/410))

## DART 6

### [DART 6.16.4 (2026-01-06)](https://github.com/dartsim/dart/milestone/89?closed=1)

- Physics
  - Fix ODE box-cylinder contact stability with libccd: [#2389](https://github.com/dartsim/dart/pull/2389)
  - Fix ODE cylinder mesh fallback on FreeBSD: [#2388](https://github.com/dartsim/dart/pull/2388)

- Parsers
  - Fix URDF shape parsing on FreeBSD: [#2379](https://github.com/dartsim/dart/pull/2379)

- Tooling and Docs
  - Add Alt Linux repro tasks: [#2381](https://github.com/dartsim/dart/pull/2381)
  - Add FreeBSD VM repro patches: [#2374](https://github.com/dartsim/dart/pull/2374)
  - Update pixi lockfile: [#2377](https://github.com/dartsim/dart/pull/2377)
  - Fix cache Docker CI for release-6.16: [#2362](https://github.com/dartsim/dart/pull/2362)

### [DART 6.16.3 (2025-12-31)](https://github.com/dartsim/dart/milestone/88?closed=1)

- Collision
  - Fix Bullet double-precision includes on release-6.16: [#2334](https://github.com/dartsim/dart/pull/2334)

- Tooling and Docs
  - Fix pixi ex example selection: [f716548](https://github.com/dartsim/dart/commit/f7165480d66)

### [DART 6.16.2 (2025-12-20)](https://github.com/dartsim/dart/milestone/87?closed=1)

- Build
  - Remove DART_BUILD_MODE_DEBUG guards to fix NDEBUG compilation: [#2326](https://github.com/dartsim/dart/pull/2326)
  - Use package.xml version for dartpy packaging: [#2327](https://github.com/dartsim/dart/pull/2327)

- Tests
  - Stabilize World::Cloning in asserts-enabled CI: [#2331](https://github.com/dartsim/dart/pull/2331)

- Tooling and Docs
  - Cover asserts-enabled build and fix API docs PR ref: [#2330](https://github.com/dartsim/dart/pull/2330)
  - Fix publish_dartpy workflow YAML: [#2329](https://github.com/dartsim/dart/pull/2329)
  - Fix manual publish validation on Windows: [#2324](https://github.com/dartsim/dart/pull/2324)
  - Restrict manual dartpy publishing to tags: [#2323](https://github.com/dartsim/dart/pull/2323)
  - Fix dartpy PyPI publishing on release-6.16: [#2322](https://github.com/dartsim/dart/pull/2322)
  - Update pixi lockfile: [#2335](https://github.com/dartsim/dart/pull/2335), [#2307](https://github.com/dartsim/dart/pull/2307)

### [DART 6.16.1 (2025-12-12)](https://github.com/dartsim/dart/milestone/86?closed=1)

- Build
  - Fix FreeListAllocator assertions when building without NDEBUG (e.g., Ubuntu 24.04 GCC 13): [#2295](https://github.com/dartsim/dart/pull/2295)

### [DART 6.16.0 (2025-11-09)](https://github.com/dartsim/dart/milestone/83?closed=1)

- Tested Platforms
  - Linux
    - Ubuntu 22.04 LTS / GCC 11.4 / x86_64
    - Ubuntu 24.04 LTS / GCC 13.2 / x86_64
  - macOS 14 / Clang 15 / arm64
  - Windows / MSVC 19.40 / x86_64

- Simulation
  - Allow servo joints to recover from position limits: [#2086](https://github.com/dartsim/dart/pull/2086)
  - Fix passive joint commands to respect joint actuation limits: [#1997](https://github.com/dartsim/dart/pull/1997)

- Core
  - Replace legacy `assert` macros with `DART_ASSERT` on release-6.16 (backport of #2109): [#2117](https://github.com/dartsim/dart/pull/2117)

- Build
  - Fix pybind11 detection and assert include handling with pixi builds: [#2118](https://github.com/dartsim/dart/pull/2118)
  - Port Eigen compatibility guard to keep 3.4+ builds working on release-6.16: [#2108](https://github.com/dartsim/dart/pull/2108)
  - Add `DART_USE_SYSTEM_TRACY`, `DART_USE_SYSTEM_PYBIND11`, and `DART_USE_SYSTEM_GOOGLEBENCHMARK` toggles: [#1911](https://github.com/dartsim/dart/pull/1911), [#1907](https://github.com/dartsim/dart/pull/1907), [#1904](https://github.com/dartsim/dart/pull/1904)
  - Fix absolute install directory handling in CMake exports: [#2006](https://github.com/dartsim/dart/pull/2006)

- Tooling and Docs
  - Use system `googletest` and `googlebenchmark` in pixi environments and upgrade bundled dependencies: [#1905](https://github.com/dartsim/dart/pull/1905)
  - Switch coverage and API docs workflows to pixi tasks and GitHub Pages deploy actions: [#2036](https://github.com/dartsim/dart/pull/2036), [#2032](https://github.com/dartsim/dart/pull/2032)
  - Install OpenSceneGraph from source on macOS builds to avoid flaky CI: [#2037](https://github.com/dartsim/dart/pull/2037)
  - Restructure documentation to clearly separate C++ and Python guidance: [#2040](https://github.com/dartsim/dart/pull/2040)
  - Add pixi-powered developer tasks (including Python workflows) and refresh GitHub templates: [#2034](https://github.com/dartsim/dart/pull/2034), [#2039](https://github.com/dartsim/dart/pull/2039)
  - Add gz-physics integration tests to CI to guard the public plugin: [#2000](https://github.com/dartsim/dart/pull/2000)

### [DART 6.15.0 (2024-11-15)](https://github.com/dartsim/dart/milestone/77?closed=1)

- Tested Platforms
  - Linux
    - Ubuntu 22.04 LTS / GCC 11.4 / x86_64
    - Ubuntu 24.04 LTS / GCC 13.2 / x86_64
  - macOS 14 / Clang 15 / arm64
  - Windows / MSVC 19.40 / x86_64

- Build
  - Added ImGui 1.91.5 support: [#1872](https://github.com/dartsim/dart/pull/1872)
  - Added nlopt 2.9.0 support: [#1875](https://github.com/dartsim/dart/pull/1875)
  - Fixed imgui is not added as transitive dependency: [#1877](https://github.com/dartsim/dart/pull/1877)

### [DART 6.14.5 (2024-09-08)](https://github.com/dartsim/dart/milestone/82?closed=1)

- Tested Platforms
  - Linux
    - Ubuntu 22.04 LTS / GCC 11.4 / x86_64
    - Ubuntu 24.04 LTS / GCC 13.2 / x86_64
  - macOS 14 / Clang 15 / arm64
  - Windows / MSVC 19.40 / x86_64

- Fixed missing parentheses in config.hpp: [#1838](https://github.com/dartsim/dart/pull/1838)
- Allowed negative scale for MeshShape: [#1841](https://github.com/dartsim/dart/pull/1841)

### [DART 6.14.4 (2024-07-06)](https://github.com/dartsim/dart/milestone/81?closed=1)

- Tested Platforms
  - Linux
    - Ubuntu 22.04 LTS / GCC 11.4 / x86_64
    - Ubuntu 24.04 LTS / GCC 13.2 / x86_64
  - macOS 14 / Clang 15 / arm64
  - Windows / MSVC 19.40 / x86_64

- Fixed GLUT dependency handling on Windows: [#1827](https://github.com/dartsim/dart/pull/1827)

### [DART 6.14.3 (2024-07-05)](https://github.com/dartsim/dart/milestone/80?closed=1)

- Tested Platforms
  - Linux
    - Ubuntu 22.04 LTS / GCC 11.4 / x86_64
    - Ubuntu 24.04 LTS / GCC 13.2 / x86_64
  - macOS 14 / Clang 15 / arm64
  - Windows / MSVC 19.40 / x86_64

- Changed the default CMake option to DART_ENABLE_SIMD=OFF: [#1825](https://github.com/dartsim/dart/pull/1825)

### [DART 6.14.2 (2024-06-28)](https://github.com/dartsim/dart/milestone/79?closed=1)

- Tested Platforms
  - Linux
    - Ubuntu 22.04 LTS / GCC 11.4 / x86_64
    - Ubuntu 24.04 LTS / GCC 13.2 / x86_64
  - macOS 14 / Clang 15 / arm64
  - Windows / MSVC 19.40 / x86_64

- Fixed version definitions: [#1820](https://github.com/dartsim/dart/pull/1820)

### [DART 6.14.2 (2024-06-26)](https://github.com/dartsim/dart/milestone/78?closed=1)

- Tested Platforms
  - Linux
    - Ubuntu 22.04 LTS / GCC 11.4 / x86_64
    - Ubuntu 24.04 LTS / GCC 13.2 / x86_64
  - macOS 14 / Clang 15 / arm64
  - Windows / MSVC 19.40 / x86_64

- Included CTest for BUILD_TESTING option: [#1819](https://github.com/dartsim/dart/pull/1819)

### [DART 6.14.0 (2024-06-24)](https://github.com/dartsim/dart/milestone/73?closed=1)

This release is mostly a maintenance update, including various CI updates and build fixes for recent development environment toolsets and dependencies.

- Tested Platforms
  - Linux
    - Ubuntu 22.04 LTS / GCC 11.4 / x86_64
    - Ubuntu 24.04 LTS / GCC 13.2 / x86_64
  - macOS 14 / Clang 15 / arm64
  - Windows / MSVC 19.40 / x86_64

- Breaking Changes
  - Removed planning component

- Build
  - Added Pixi support

- General
  - Added profile interface with Tracy backend support
  - Added benchmark setup, including boxes and kinematics benchmarks

- Dynamics
  - Allowed specifying mimic joint properties per DoF: [#1752](https://github.com/dartsim/dart/pull/1752)
  - [Improved performance in constructing LCP problem](https://github.com/dartsim/dart/commit/76d8fe1a72f6925c06f64eea3d2cd135234b59de)

### [DART 6.13.2 (2024-03-17)](https://github.com/dartsim/dart/milestone/75?closed=1)

- Tested Platforms
  - Linux
    - Ubuntu 22.04 LTS on amd64 / GCC 11.2 / amd64
    - Ubuntu 24.04 LTS on amd64 / GCC 13.2 / amd64
  - macOS 12 (Monterey) / AppleClang 14 / amd64
  - Windows / MSVC 19.38 / amd64

- Build
  - Fixed build with GCC >= 13: [#1793](https://github.com/dartsim/dart/pull/1793)

### [DART 6.13.1 (2024-01-04)](https://github.com/dartsim/dart/milestone/74?closed=1)

- Tested Platforms
  - Ubuntu Focal on amd64 / GCC 9.4 / amd64
  - Ubuntu Jammy on amd64 / GCC 11.3 / amd64
  - macOS 12 (Monterey) / Clang 14 / amd64
  - Windows / MSVC 19.37 / amd64

- Build
  - Fixed build with urdfdom 4.0.0: [#1779](https://github.com/dartsim/dart/pull/1779)
  - Fixed invalid array access in moving skeleton subtree: [#1778](https://github.com/dartsim/dart/pull/1778)

- Dynamics
  - Fixed joint not recovering after reaching position limits in servo mode: [#1774](https://github.com/dartsim/dart/pull/1774)

### [DART 6.13.0 (2022-12-31)](https://github.com/dartsim/dart/milestone/69?closed=1)

- Supported Platforms
  - Ubuntu Focal on amd64 / GCC 9.3 / amd64
  - Ubuntu Jammy on amd64 / GCC 11.2 / amd64
  - macOS 12 (Monterey) / Clang 13 / amd64
  - Windows / Microsoft Visual Studio 2019 / amd64

- Dependency
  - Added required dependencies: fmt
  - Added optional dependencies: spdlog
  - Removed required dependencies: Boost

- Build
  - Dropped supporting FCL < 0.5: [#1647](https://github.com/dartsim/dart/pull/1647)

- Common
  - Added Castable class: [#1634](https://github.com/dartsim/dart/pull/1634)
  - Added spdlog support as underlying logging framework: [#1633](https://github.com/dartsim/dart/pull/1633)
  - Added custom memory allocators: [#1636](https://github.com/dartsim/dart/pull/1636), [#1637](https://github.com/dartsim/dart/pull/1637), [#1639](https://github.com/dartsim/dart/pull/1639), [#1645](https://github.com/dartsim/dart/pull/1645), [#1646](https://github.com/dartsim/dart/pull/1646)
  - Added Stopwatch class to replace Timer: [#1638](https://github.com/dartsim/dart/pull/1638)
  - Removed Boost dependency: [#1648](https://github.com/dartsim/dart/pull/1648), [#1651](https://github.com/dartsim/dart/pull/1651)

- Collision Detection
  - Updated to use convex mesh of Bullet when possible: [#1664](https://github.com/dartsim/dart/pull/1664), [#1667](https://github.com/dartsim/dart/pull/1667)

- Dynamics
  - Fixed setResitutionCoeff() calling setFrictionCoeff(): [#1677](https://github.com/dartsim/dart/pull/1677)
  - Made inertial warnings optional when setting tensor: [#1672](https://github.com/dartsim/dart/pull/1672)
  - Added deep copy for shapes: [#1612](https://github.com/dartsim/dart/pull/1612)
  - Added iterator methods to container-like classes: [#1644](https://github.com/dartsim/dart/pull/1644)
  - Fixed grouping of constraints: [#1624](https://github.com/dartsim/dart/pull/1624), [#1628](https://github.com/dartsim/dart/pull/1628)
  - Fixed issue with removing skeletons without shapes: [#1625](https://github.com/dartsim/dart/pull/1625)
  - Added support for custom contact surface handlers: [#1626](https://github.com/dartsim/dart/pull/1626)

- GUI
  - Fixed depth testing for transparent objects: [#1643](https://github.com/dartsim/dart/pull/1643)
  - Added depth rendering mode: [#1652](https://github.com/dartsim/dart/pull/1652)

### [DART 6.12.2 (2022-07-31)](https://github.com/dartsim/dart/milestone/72?closed=1)

- Build
  - Fixed build with urdfdom 3.1.0 on Windows: [#1675](https://github.com/dartsim/dart/pull/1675)

### [DART 6.12.1 (2021-11-04)](https://github.com/dartsim/dart/milestone/71?closed=1)

- Build
  - Fixed bullet header include: [#1620](https://github.com/dartsim/dart/pull/1620)

- dartpy
  - Added Python bindings for Joint::getWrenchTo{Child|Parent}BodyNode: [#1621](https://github.com/dartsim/dart/pull/1621)

### [DART 6.12.0 (2021-11-01)](https://github.com/dartsim/dart/milestone/66?closed=1)

- API Breaking Changes
  - DART 6.12.0 and later require compilers that support C++17: [#1600](https://github.com/dartsim/dart/pull/1600)
    - Increased minimum CMake version to 3.10.2
    - Increased minimum compiler versions to GCC 7.3.0, Clang 6.0, MSVC 16.0
    - Dropped Ubuntu Xenial (16.04 LTS) support

- Build
  - Remove DART_BUILD_DARTPY option: [#1600](https://github.com/dartsim/dart/pull/1600)

- Dynamics
  - Added joint force/torque getter: [#1616](https://github.com/dartsim/dart/pull/1616)

- Parsers
  - Added default options to DartLoader for missing properties in URDF files: [#1605](https://github.com/dartsim/dart/pull/1605)
  - Allowed SdfParser to set default root joint type: [#1617](https://github.com/dartsim/dart/pull/1617)

- GUI
  - Updated ImGui to 1.84.2: [#1608](https://github.com/dartsim/dart/pull/1608)

- dartpy
  - Added Python bindings for ResourceRetriever and SdfParser: [#1610](https://github.com/dartsim/dart/pull/1610)

### [DART 6.11.2 (2021-10-29)](https://github.com/dartsim/dart/milestone/68?closed=1)

- dartpy
  - Added Python binding for global lighting mode setting: [#1615](https://github.com/dartsim/dart/pull/1615)

### [DART 6.11.1 (2021-08-23)](https://github.com/dartsim/dart/milestone/67?closed=1)

- Dynamics
  - Fixed incorrect LCP construction in JointConstraint for multi-DOFs joints: [#1597](https://github.com/dartsim/dart/pull/1597)

### [DART 6.11.0 (2021-07-15)](https://github.com/dartsim/dart/milestone/64?closed=1)

- Math
  - Added `Mesh`, `TriMesh`, and `Icosphere` classes: [#1579](https://github.com/dartsim/dart/pull/1579)

- Collision
  - Fixed incorrect group-group collision checking for BulletCollisionDetector: [#1585](https://github.com/dartsim/dart/pull/1585), [#717](https://github.com/dartsim/dart/issues/717)

- Dynamics
  - Fixed servo motor doesn't respect joint position limits: [#1587](https://github.com/dartsim/dart/pull/1587)

- GUI
  - Fixed incorrect MultiSphereConvexHull rendering: [#1579](https://github.com/dartsim/dart/pull/1579)
  - Use GLVND over the legacy OpenGL libraries: [#1584](https://github.com/dartsim/dart/pull/1584)

- Build and testing
  - Add DART\_ prefix to macros to avoid potential conflicts: [#1586](https://github.com/dartsim/dart/pull/1586)

### [DART 6.10.1 (2021-04-19)](https://github.com/dartsim/dart/milestone/65?closed=1)

- Dynamics
  - Fixed inertia calculation of CapsuleShape: [#1561](https://github.com/dartsim/dart/pull/1561)

- GUI
  - Changed to protect OpenGL attributes shared by ImGui and OSG: [#1558](https://github.com/dartsim/dart/pull/1558)
  - Changed to set backface culling by default: [#1559](https://github.com/dartsim/dart/pull/1559)

- dartpy
  - Added Python binding for BodyNode::getBodyForce(): [#1563](https://github.com/dartsim/dart/pull/1563)

### [DART 6.10.0 (2021-04-09)](https://github.com/dartsim/dart/milestone/58?closed=1)

- Common
  - Removed use of boost::filesystem in public APIs: [#1417](https://github.com/dartsim/dart/pull/1417)
  - Changed Signal to remove connection when they're being disconnected: [#1462](https://github.com/dartsim/dart/pull/1462)

- Collision
  - Added ConeShape support for FCLCollisionDetector: [#1447](https://github.com/dartsim/dart/pull/1447)
  - Fixed segfault from raycast when no ray hit: [#1461](https://github.com/dartsim/dart/pull/1461)
  - Added PyramidShape class: [#1466](https://github.com/dartsim/dart/pull/1466)

- Kinematics
  - Added IkFast parameter accessors to IkFast class: [#1396](https://github.com/dartsim/dart/pull/1396)
  - Changed IkFast to wrap IK solutions into the joint limits for RevoluteJoint: [#1452](https://github.com/dartsim/dart/pull/1452)
  - Added option to specify reference frame of TaskSpaceRegion: [#1548](https://github.com/dartsim/dart/pull/1548)

- Dynamics
  - Fixed friction and restitution of individual shapes in a body: [#1369](https://github.com/dartsim/dart/pull/1369)
  - Fixed soft body simulation when command input is not reset: [#1372](https://github.com/dartsim/dart/pull/1372)
  - Added joint velocity limit constraint support: [#1407](https://github.com/dartsim/dart/pull/1407)
  - Added type property to constrain classes: [#1415](https://github.com/dartsim/dart/pull/1415)
  - Allowed to set joint rest position out of joint limits: [#1418](https://github.com/dartsim/dart/pull/1418)
  - Added secondary friction coefficient parameter: [#1424](https://github.com/dartsim/dart/pull/1424)
  - Allowed to set friction direction per ShapeFrame: [#1427](https://github.com/dartsim/dart/pull/1427)
  - Fixed incorrect vector resizing in BoxedLcpConstraintSolver: [#1459](https://github.com/dartsim/dart/pull/1459)
  - Changed to increment BodyNode version when it's being removed from Skeleton: [#1489](https://github.com/dartsim/dart/pull/1489)
  - Changed to print warning only once from BulletCollisionDetector::collide/distance: [#1546](https://github.com/dartsim/dart/pull/1546)
  - Added force dependent slip: [#1505](https://github.com/dartsim/dart/pull/1505)

- GUI
  - Fixed memory leaks from dart::gui::osg::Viewer: [#1349](https://github.com/dartsim/dart/pull/1349)
  - Added point rendering mode to PointCloudShape: [#1351](https://github.com/dartsim/dart/pull/1351), [#1355](https://github.com/dartsim/dart/pull/1355)
  - Updated ImGui to 1.71: [#1362](https://github.com/dartsim/dart/pull/1362)
  - Updated ImGui to 1.79: [#1498](https://github.com/dartsim/dart/pull/1498)
  - Fixed refresh of LineSegmentShapeNode: [#1381](https://github.com/dartsim/dart/pull/1381)
  - Fixed OSG transparent object sorting: [#1414](https://github.com/dartsim/dart/pull/1414)
  - Added modifier key support to ImGuiHandler: [#1436](https://github.com/dartsim/dart/pull/1436)
  - Fixed mixed intrinsic and extrinsic camera parameters in OpenGL projection matrix: [#1485](https://github.com/dartsim/dart/pull/1485)
  - Enabled mouse middle and right buttons in ImGuiHandler: [#1500](https://github.com/dartsim/dart/pull/1500)

- Parser
  - Allowed parsing SDF up to version 1.6: [#1385](https://github.com/dartsim/dart/pull/1385)
  - Fixed SDF parser not creating dynamics aspect for collision shape: [#1386](https://github.com/dartsim/dart/pull/1386)
  - Added root joint parsing option in URDF parser: [#1399](https://github.com/dartsim/dart/pull/1399), [#1406](https://github.com/dartsim/dart/pull/1406)
  - Enabled URDF parser to read visual and collision names: [#1410](https://github.com/dartsim/dart/pull/1410)
  - Added (experimental) MJCF parser: [#1416](https://github.com/dartsim/dart/pull/1416)

- dartpy
  - Added raycast option and result: [#1343](https://github.com/dartsim/dart/pull/1343)
  - Added GUI event handler: [#1346](https://github.com/dartsim/dart/pull/1346)
  - Added shadow technique: [#1348](https://github.com/dartsim/dart/pull/1348)
  - Added findSolution and solveAndApply to InverseKinematics: [#1358](https://github.com/dartsim/dart/pull/1358)
  - Added InteractiveFrame and ImGui APIs: [#1359](https://github.com/dartsim/dart/pull/1359)
  - Added bindings for Joint::getTransformFrom{Parent|Child}BodyNode(): [#1377](https://github.com/dartsim/dart/pull/1377)
  - Added bindings for BodyNode::getChild{BodyNode|Joint}(): [#1387](https://github.com/dartsim/dart/pull/1387)
  - Added bindings for Inertia: [#1388](https://github.com/dartsim/dart/pull/1388)
  - Added bindings for getting all BodyNodes from a Skeleton: [#1397](https://github.com/dartsim/dart/pull/1397)
  - Added bindings for background color support in osg viewer: [#1398](https://github.com/dartsim/dart/pull/1398)
  - Added bindings for BallJoint::convertToPositions(): [#1408](https://github.com/dartsim/dart/pull/1408)
  - Fixed typos in Skeleton: [#1392](https://github.com/dartsim/dart/pull/1392)
  - Fixed enabling drag and drop for InteractiveFrame: [#1432](https://github.com/dartsim/dart/pull/1432)
  - Added bindings for pointcloud and contact retrieval: [#1455](https://github.com/dartsim/dart/pull/1455)
  - Fixed TypeError from dartpy.dynamics.Node.getBodyNodePtr(): [#1463](https://github.com/dartsim/dart/pull/1463)
  - Added pybind/eigen.h to DistanceResult.cpp for read/write of eigen types: [#1480](https://github.com/dartsim/dart/pull/1480)
  - Added bindings for adding ShapeFrames to CollisionGroup: [#1490](https://github.com/dartsim/dart/pull/1490)
  - Changed dartpy install command to make install-dartpy: [#1503](https://github.com/dartsim/dart/pull/1503)
  - Added bindings for CollisionFilter, CollisionGroup, and Node: [#1545](https://github.com/dartsim/dart/pull/1545)
  - Added bindings for TaskSpaceRegion: [#1550](https://github.com/dartsim/dart/pull/1550)

- Build and testing
  - Fixed compiler warnings from GCC 9.1: [#1366](https://github.com/dartsim/dart/pull/1366)
  - Replaced M_PI with dart::math::pi: [#1367](https://github.com/dartsim/dart/pull/1367)
  - Enabled octomap support on macOS: [#1078](https://github.com/dartsim/dart/pull/1078)
  - Removed dependency on Boost::regex: [#1412](https://github.com/dartsim/dart/pull/1412)
  - Added support new if() IN_LIST operator in DARTConfig.cmake: [#1434](https://github.com/dartsim/dart/pull/1434)
  - Updated Findfcl.cmake to support FCL 0.6: [#1441](https://github.com/dartsim/dart/pull/1441)
  - Added gtest macros for Eigen object comparisons: [#1443](https://github.com/dartsim/dart/pull/1443)
  - Removed gccfilter: [#1464](https://github.com/dartsim/dart/pull/1464)
  - Allowed to set CMAKE_INSTALL_PREFIX on Windows: [#1478](https://github.com/dartsim/dart/pull/1478)
  - Enforced to use OpenSceneGraph 3.7.0 or greater on macOS Catalina: [#1479](https://github.com/dartsim/dart/pull/1479)
  - Fixed compatibility with clang-cl: [#1509](https://github.com/dartsim/dart/pull/1509)
  - Fixed MSVC linking for assimp and fcl: [#1510](https://github.com/dartsim/dart/pull/1510)
  - Fixed AspectWithState-relate compile error on Windows: [#1528](https://github.com/dartsim/dart/pull/1528)
  - Made `dart.pc` relocatable: [#1529](https://github.com/dartsim/dart/pull/1529)
  - Added CI for multiple Linux platforms: arm64 and ppc64le: [#1531](https://github.com/dartsim/dart/pull/1531)
  - Fixed Aspect/Composite-relate tests on Windows/MSVC: [#1541](https://github.com/dartsim/dart/pull/1541), [#1542](https://github.com/dartsim/dart/pull/1542)
  - Added `DART_SKIP_<dep>`\_advanced option: [#1529](https://github.com/dartsim/dart/pull/1529)
  - Replaced OpenGL dependency variables with targets: [#1552](https://github.com/dartsim/dart/pull/1552)

- Documentation
  - Updated tutorial documentation and code to reflect new APIs: [#1481](https://github.com/dartsim/dart/pull/1481)

### [DART 6.9.5 (2020-10-17)](https://github.com/dartsim/dart/milestone/63?closed=1)

- Optimization
  - Added Ipopt >= 3.13 support: [#1506](https://github.com/dartsim/dart/pull/1506)

### [DART 6.9.4 (2020-08-30)](https://github.com/dartsim/dart/milestone/62?closed=1)

- Build
  - Added support new if() IN_LIST operator in DARTConfig.cmake (6.9 backport): [#1494](https://github.com/dartsim/dart/pull/1494)

### [DART 6.9.3 (2020-08-26)](https://github.com/dartsim/dart/milestone/61?closed=1)

- Dynamics
  - Changed to update the Properties version of a BodyNode when moved to a new Skeleton: [#1445](https://github.com/dartsim/dart/pull/1445)
  - Fixed incorrect implicit joint damping/spring force computation in inverse dynamics: [#1451](https://github.com/dartsim/dart/pull/1451)

### [DART 6.9.2 (2019-08-16)](https://github.com/dartsim/dart/milestone/60?closed=1)

- Dynamics
  - Allowed constraint force mixing > 1: [#1371](https://github.com/dartsim/dart/pull/1371)

### [DART 6.9.1 (2019-06-06)](https://github.com/dartsim/dart/milestone/59?closed=1)

- Collision
  - Added default constructor to RayHit: [#1345](https://github.com/dartsim/dart/pull/1345)

- dartpy
  - Updated build scripts for uploading dartpy to PyPI: [#1341](https://github.com/dartsim/dart/pull/1341)

### [DART 6.9.0 (2019-05-26)](https://github.com/dartsim/dart/milestone/52?closed=1)

- API Breaking Changes
  - DART 6.9.0 and later require compilers that support C++14.

- Common
  - Deprecated custom make_unique in favor of std::make_unique: [#1317](https://github.com/dartsim/dart/pull/1317)

- Collision Detection
  - Added raycast query to BulletCollisionDetector: [#1309](https://github.com/dartsim/dart/pull/1309)

- Dynamics
  - Added safeguard for accessing Assimp color: [#1313](https://github.com/dartsim/dart/pull/1313)

- Parser
  - Changed URDF parser to use URDF material color when specified: [#1295](https://github.com/dartsim/dart/pull/1295)

- GUI
  - Added heightmap support to OSG renderer: [#1293](https://github.com/dartsim/dart/pull/1293)
  - Improved voxel grid and point cloud rendering performance: [#1294](https://github.com/dartsim/dart/pull/1294)
  - Fixed incorrect alpha value update of InteractiveFrame: [#1297](https://github.com/dartsim/dart/pull/1297)
  - Fixed dereferencing a dangling pointer in WorldNode: [#1311](https://github.com/dartsim/dart/pull/1311)
  - Removed warning of ImGuiViewer + OSG shadow: [#1312](https://github.com/dartsim/dart/pull/1312)
  - Added shape type and color options to PointCloudShape: [#1314](https://github.com/dartsim/dart/pull/1314), [#1316](https://github.com/dartsim/dart/pull/1316)
  - Fixed incorrect transparency of MeshShape for MATERIAL_COLOR mode: [#1315](https://github.com/dartsim/dart/pull/1315)
  - Fixed incorrect PointCloudShape transparency: [#1330](https://github.com/dartsim/dart/pull/1330)
  - Improved voxel rendering by using multiple nodes instead of CompositeShape: [#1334](https://github.com/dartsim/dart/pull/1334)

- Examples and Tutorials
  - Updated examples directory to make dart::gui::osg more accessible: [#1305](https://github.com/dartsim/dart/pull/1305)

- dartpy
  - Switched to pybind11: [#1307](https://github.com/dartsim/dart/pull/1307)
  - Added grid visual: [#1318](https://github.com/dartsim/dart/pull/1318)
  - Added ReferentialSkeleton, Linkage, and Chain: [#1321](https://github.com/dartsim/dart/pull/1321)
  - Enabled WorldNode classes to overload virtual functions in Python: [#1322](https://github.com/dartsim/dart/pull/1322)
  - Added JacobianNode and operational space controller example: [#1323](https://github.com/dartsim/dart/pull/1323)
  - Removed static create() functions in favor of custom constructors: [#1324](https://github.com/dartsim/dart/pull/1324)
  - Added optimizer APIs with GradientDescentSolver and NloptSolver: [#1325](https://github.com/dartsim/dart/pull/1325)
  - Added SimpleFrame: [#1326](https://github.com/dartsim/dart/pull/1326)
  - Added basic inverse kinematics APIs: [#1327](https://github.com/dartsim/dart/pull/1327)
  - Added shapes and ShapeFrame aspects: [#1328](https://github.com/dartsim/dart/pull/1328)
  - Added collision APIs: [#1329](https://github.com/dartsim/dart/pull/1329)
  - Added DegreeOfFreedom and ShapeNode: [#1332](https://github.com/dartsim/dart/pull/1332)
  - Added constraint APIs: [#1333](https://github.com/dartsim/dart/pull/1333)
  - Added BallJoint, RevoluteJoint, joint properties, and chain tutorial (incomplete): [#1335](https://github.com/dartsim/dart/pull/1335)
  - Added all the joints: [#1337](https://github.com/dartsim/dart/pull/1337)
  - Added DART, Bullet, Ode collision detectors: [#1339](https://github.com/dartsim/dart/pull/1339)

### [DART 6.8.5 (2019-05-26)](https://github.com/dartsim/dart/milestone/57?closed=1)

- Collision
  - Fixed handling of submeshes in ODE collision detector: [#1336](https://github.com/dartsim/dart/pull/1336)

#### Compilers Tested

- Linux
  - GCC 64-bit: 5.4.0, 7.3.0, 8.2.0
  - GCC 32-bit: 5.4.0

- macOS
  - AppleClang: 9.1.0, 10.0.0

### [DART 6.8.4 (2019-05-03)](https://github.com/dartsim/dart/milestone/56?closed=1)

#### Changes

- GUI
  - Fixed crashing on exiting OSG + ImGui applications: [#1303](https://github.com/dartsim/dart/pull/1303)

#### Compilers Tested

- Linux
  - GCC 64-bit: 5.4.0, 7.3.0, 8.2.0
  - GCC 32-bit: 5.4.0

- macOS
  - AppleClang: 9.1.0, 10.0.0

### [DART 6.8.3 (2019-05-01)](https://github.com/dartsim/dart/milestone/55?closed=1)

#### Changes

- Parser
  - Fixed VskParker returning incorrect resource retriever: [#1300](https://github.com/dartsim/dart/pull/1300)

- Build
  - Fixed building with pagmo's optional dependencies: [#1301](https://github.com/dartsim/dart/pull/1301)

#### Compilers Tested

- Linux
  - GCC 64-bit: 5.4.0, 7.3.0, 8.2.0
  - GCC 32-bit: 5.4.0

- macOS
  - AppleClang: 9.1.0, 10.0.0

### [DART 6.8.2 (2019-04-23)](https://github.com/dartsim/dart/milestone/54?closed=1)

#### Changes

- Dynamics
  - Fixed BoxedLcpConstraintSolver is not API compatible with 6.7: [#1291](https://github.com/dartsim/dart/pull/1291)

- Build
  - Fixed building with FCL built without Octomap: [#1292](https://github.com/dartsim/dart/pull/1292)

#### Compilers Tested

- Linux
  - GCC 64-bit: 5.4.0, 7.3.0, 8.2.0
  - GCC 32-bit: 5.4.0

- macOS
  - AppleClang: 9.1.0, 10.0.0

### [DART 6.8.1 (2019-04-23)](https://github.com/dartsim/dart/milestone/53?closed=1)

#### Changes

- Build System
  - Fixed invalid double quotation marks in DARTFindBoost.cmake: [#1283](https://github.com/dartsim/dart/pull/1283)
  - Disabled octomap support on macOS: [#1284](https://github.com/dartsim/dart/pull/1284)

#### Compilers Tested

- Linux
  - GCC 64-bit: 5.4.0, 7.3.0, 8.2.0
  - GCC 32-bit: 5.4.0

- macOS
  - AppleClang: 9.1.0, 10.0.0

### [DART 6.8.0 (2019-04-22)](https://github.com/dartsim/dart/milestone/48?closed=1)

#### Changes

- Kinematics
  - Added findSolution() and solveAndApply() to InverseKinematics and HierarchicalIk classes and deprecated solve(~) member functions: [#1266](https://github.com/dartsim/dart/pull/1266)
  - Added an utility constructor to Linkage::Criteria to create sequence Linkage: [#1273](https://github.com/dartsim/dart/pull/1273)

- Dynamics
  - Fixed incorrect transpose check in Inertia::verifySpatialTensor(): [#1258](https://github.com/dartsim/dart/pull/1258)
  - Allowed BoxedLcpConstraintSolver to have a secondary LCP solver: [#1265](https://github.com/dartsim/dart/pull/1265)

- Simulation
  - The LCP solver will be less aggressive about printing out unnecessary warnings: [#1238](https://github.com/dartsim/dart/pull/1238)
  - Fixed not copying constraints in World::setConstraintSolver(): [#1260](https://github.com/dartsim/dart/pull/1260)

- Collision Detection
  - The BodyNodeCollisionFilter will ignore contacts between immobile bodies: [#1232](https://github.com/dartsim/dart/pull/1232)

- Planning
  - Fixed linking error of FLANN by explicitly linking to lz4: [#1221](https://github.com/dartsim/dart/pull/1221)

- Python
  - Added (experimental) Python binding: [#1237](https://github.com/dartsim/dart/pull/1237)

- Parsers
  - Changed urdf parser to warn if robot model has multi-tree: [#1270](https://github.com/dartsim/dart/pull/1270)

- GUI
  - Updated ImGui to 1.69: [#1274](https://github.com/dartsim/dart/pull/1274)
  - Added VoxelGridShape support to OSG renderer: [#1276](https://github.com/dartsim/dart/pull/1276)
  - Added PointCloudShape and its OSG rendering: [#1277](https://github.com/dartsim/dart/pull/1277)
  - Added grid visual to OSG renderer: [#1278](https://github.com/dartsim/dart/pull/1278), [#1280](https://github.com/dartsim/dart/pull/1280)

- Build System
  - Changed to use GNUInstallDirs for install paths: [#1241](https://github.com/dartsim/dart/pull/1241)
  - Fixed not failing for missing required dependencies: [#1250](https://github.com/dartsim/dart/pull/1250)
  - Fixed attempting to link octomap when not imported: [#1253](https://github.com/dartsim/dart/pull/1253)
  - Fixed not defining boost targets: [#1254](https://github.com/dartsim/dart/pull/1254)

#### Compilers Tested

- Linux
  - GCC 64-bit: 5.4.0, 7.3.0, 8.2.0
  - GCC 32-bit: 5.4.0

- macOS
  - AppleClang: 9.1.0, 10.0.0

### [DART 6.7.3 (2019-02-19)](https://github.com/dartsim/dart/milestone/51?closed=1)

#### Changes

- Dynamics
  - Fixed Skeleton::setState(): [#1245](https://github.com/dartsim/dart/pull/1245)

#### Compilers Tested

- Linux
  - GCC (C++11): 5.4.0, 7.3.0, 8.2.0

- Linux (32-bit)
  - GCC (C++11): 5.4.0

- macOS
  - AppleClang (C++11): 9.1.0

### [DART 6.7.2 (2019-01-17)](https://github.com/dartsim/dart/milestone/50?closed=1)

#### Changes

- Build system
  - Fixed #1223 for the recursive case: [#1227](https://github.com/dartsim/dart/pull/1227)
  - Specified mode for find_package(): [#1228](https://github.com/dartsim/dart/pull/1228)

#### Compilers Tested

- Linux
  - GCC (C++11): 5.4.0, 7.3.0, 8.2.0

- Linux (32-bit)
  - GCC (C++11): 5.4.0

- macOS
  - AppleClang (C++11): 9.1.0

### [DART 6.7.1 (2019-01-15)](https://github.com/dartsim/dart/milestone/49?closed=1)

#### Changes

- Build system
  - Ensure that imported targets of dependencies are always created when finding the dart package: [#1222](https://github.com/dartsim/dart/pull/1222)
  - Set components to not-found when their external dependencies are missing: [#1223](https://github.com/dartsim/dart/pull/1223)

#### Compilers Tested

- Linux
  - GCC (C++11): 5.4.0, 7.3.0, 8.2.0

- Linux (32-bit)
  - GCC (C++11): 5.4.0

- macOS
  - AppleClang (C++11): 9.1.0

### [DART 6.7.0 (2019-01-10)](https://github.com/dartsim/dart/milestone/45?closed=1)

#### Changes

- Build system
  - Fixed compilation warnings for newer versions of compilers: [#1177](https://github.com/dartsim/dart/pull/1177)
  - Changed to generate namespace headers without requiring \*.hpp.in files: [#1192](https://github.com/dartsim/dart/pull/1192)
  - Dropped supporting Ubuntu Trusty and started using imported targets of dependencies: [#1212](https://github.com/dartsim/dart/pull/1212)

- Collision Detection
  - CollisionGroups will automatically update their objects when any changes occur to Skeletons or BodyNodes that they are subscribed to: [#1112](https://github.com/dartsim/dart/pull/1112)
  - Contact points with negative penetration depth will be ignored: [#1185](https://github.com/dartsim/dart/pull/1185)

- Math
  - Consolidated random functions into Random class: [#1109](https://github.com/dartsim/dart/pull/1109)

- Dynamics
  - Refactor constraint solver: [#1099](https://github.com/dartsim/dart/pull/1099), [#1101](https://github.com/dartsim/dart/pull/1101)
  - Added mimic joint functionality as a new actuator type: [#1178](https://github.com/dartsim/dart/pull/1178)
  - Added clone function to MetaSkeleton: [#1201](https://github.com/dartsim/dart/pull/1201)

- Optimization
  - Added multi-objective optimization with pagmo2 support: [#1106](https://github.com/dartsim/dart/pull/1106)

- GUI
  - Reorganized OpenGL and GLUT files: [#1088](https://github.com/dartsim/dart/pull/1088)
  - Added the RealTimeWorldNode to display simulations at real-time rates: [#1216](https://github.com/dartsim/dart/pull/1216)

- Misc
  - Updated Googletest to version 1.8.1: [#1214](https://github.com/dartsim/dart/pull/1214)

#### Compilers Tested

- Linux
  - GCC (C++11): 5.4.0, 7.3.0, 8.2.0

- Linux (32-bit)
  - GCC (C++11): 5.4.0

- macOS
  - AppleClang (C++11): 9.1.0

### [DART 6.6.2 (2018-09-03)](https://github.com/dartsim/dart/milestone/47?closed=1)

- Utils
  - Fixed checking file existence in DartResourceRetriever: [#1107](https://github.com/dartsim/dart/pull/1107)

### [DART 6.6.1 (2018-08-04)](https://github.com/dartsim/dart/milestone/46?closed=1)

- Utils
  - Added option to DartResourceRetriever to search from environment variable DART_DATA_PATH: [#1095](https://github.com/dartsim/dart/pull/1095)

- Examples
  - Fixed CMakeLists.txt of humanJointLimits: [#1094](https://github.com/dartsim/dart/pull/1094)

### [DART 6.6.0 (2018-08-02)](https://github.com/dartsim/dart/milestone/44?closed=1)

- Collision detection
  - Added voxel grid map: [#1076](https://github.com/dartsim/dart/pull/1076), [#1083](https://github.com/dartsim/dart/pull/1083)
  - Added heightmap support: [#1069](https://github.com/dartsim/dart/pull/1069)

### [DART 6.5.0 (2018-05-12)](https://github.com/dartsim/dart/milestone/41?closed=1)

- Common
  - Added LockableReference classes: [#1011](https://github.com/dartsim/dart/pull/1011)
  - Added missing \<vector\> to Memory.hpp: [#1057](https://github.com/dartsim/dart/pull/1057)

- GUI
  - Added FOV API to OSG viewer: [#1048](https://github.com/dartsim/dart/pull/1048)

- Parsers
  - Fixed incorrect parsing of continuous joints specified in URDF [#1064](https://github.com/dartsim/dart/pull/1064)

- Simulation
  - Added World::hasSkeleton(): [#1050](https://github.com/dartsim/dart/pull/1050)

- Misc
  - Fixed memory leaks in mesh loading: [#1066](https://github.com/dartsim/dart/pull/1066)

### [DART 6.4.0 (2018-03-26)](https://github.com/dartsim/dart/milestone/39?closed=1)

- Common
  - Added DART_COMMON_DECLARE_SMART_POINTERS macro: [#1022](https://github.com/dartsim/dart/pull/1022)
  - Added ResourceRetriever::getFilePath(): [#972](https://github.com/dartsim/dart/pull/972)

- Kinematics/Dynamics
  - Added relative Jacobian functions to MetaSkeleton: [#997](https://github.com/dartsim/dart/pull/997)
  - Added vectorized joint limit functions: [#996](https://github.com/dartsim/dart/pull/996)
  - Added lazy evaluation for shape's volume and bounding-box computation: [#959](https://github.com/dartsim/dart/pull/959)
  - Added IkFast support as analytic IK solver: [#887](https://github.com/dartsim/dart/pull/887)
  - Added TranslationalJoint2D: [#1003](https://github.com/dartsim/dart/pull/1003)
  - Fixed NaN values caused by zero-length normals in ContactConstraint: [#881](https://github.com/dartsim/dart/pull/881)
  - Extended BodyNode::createShapeNode() to accept more types of arguments: [#986](https://github.com/dartsim/dart/pull/986)

- Collision detection
  - Added FCL 0.6 support (backport of #873): [#936](https://github.com/dartsim/dart/pull/936)

- GUI
  - Added support of rendering texture images: [#973](https://github.com/dartsim/dart/pull/973)
  - Added OSG shadows: [#978](https://github.com/dartsim/dart/pull/978)

- Examples
  - Added humanJointLimits: [#1016](https://github.com/dartsim/dart/pull/1016)

- License
  - Added Personal Robotics Lab and Open Source Robotics Foundation as contributors: [#929](https://github.com/dartsim/dart/pull/929)

- Misc
  - Added World::create(): [#962](https://github.com/dartsim/dart/pull/962)
  - Added MetaSkeleton::hasBodyNode() and MetaSkeleton::hasJoint(): [#1000](https://github.com/dartsim/dart/pull/1000)
  - Suppressed -Winjected-class-name warnings from Clang 5.0.0: [#964](https://github.com/dartsim/dart/pull/964)
  - Suppressed -Wdangling-else warnings from GCC 7.2.0: [#937](https://github.com/dartsim/dart/pull/937)
  - Changed console macros to use global namespace resolutions: [#1010](https://github.com/dartsim/dart/pull/1010)
  - Fixed build with Eigen 3.2.1-3.2.8: [#1042](https://github.com/dartsim/dart/pull/1042)
  - Fixed various build issues with Visual Studio: [#956](https://github.com/dartsim/dart/pull/956)
  - Removed TinyXML dependency: [#993](https://github.com/dartsim/dart/pull/993)

### [DART 6.3.1 (2018-03-21)](https://github.com/dartsim/dart/milestone/42?closed=1)

- Build system
  - Removed an undefined cmake macro/function: [#1036](https://github.com/dartsim/dart/pull/1036)

- ROS support
  - Tweaked package.xml for catkin support: [#1027](https://github.com/dartsim/dart/pull/1027), [#1029](https://github.com/dartsim/dart/pull/1029), [#1031](https://github.com/dartsim/dart/pull/1031), [#1032](https://github.com/dartsim/dart/pull/1031), [#1033](https://github.com/dartsim/dart/pull/1033)

### [DART 6.3.0 (2017-10-04)](https://github.com/dartsim/dart/milestone/36?closed=1)

- Collision detection
  - Added a feature of disabling body node pairs to BodyNodeCollisionFilter: [#911](https://github.com/dartsim/dart/pull/911)

- Kinematics/Dynamics
  - Added setter and getter for WeldJointConstraint::mRelativeTransform: [#910](https://github.com/dartsim/dart/pull/910)

- Parsers
  - Improved SkelParser to read alpha value: [#914](https://github.com/dartsim/dart/pull/914)

- Misc
  - Changed not to use lambda function as an workaround for DART python binding: [#916](https://github.com/dartsim/dart/pull/916)

### [DART 6.2.1 (2017-08-08)](https://github.com/dartsim/dart/milestone/37?closed=1)

- Collision detection
  - Fixed collision checking between objects from the same body node: [#894](https://github.com/dartsim/dart/pull/894)

- Kinematics/Dynamics
  - Fixed transform of ScrewJoint with thread pitch: [#855](https://github.com/dartsim/dart/pull/855)

- Parsers
  - Fixed incorrect reading of <use_parent_model_frame> from SDF: [#893](https://github.com/dartsim/dart/pull/893)
  - Fixed missing reading of joint friction from URDF: [#891](https://github.com/dartsim/dart/pull/891)

- Testing
  - Fixed testing ODE collision detector on macOS: [#884](https://github.com/dartsim/dart/pull/884)
  - Removed redundant main body for each test source file: [#856](https://github.com/dartsim/dart/pull/856)

- Misc
  - Fixed build of the OpenSceneGraph GUI library (`dart-gui`, historically `dart-gui-osg`) that depends on the presence of OSG: [#898](https://github.com/dartsim/dart/pull/898)
  - Fixed build of examples and tutorials on macOS: [#889](https://github.com/dartsim/dart/pull/889)
  - Fixed missing overriding method OdePlane::isPlaceable(): [#886](https://github.com/dartsim/dart/pull/886)
  - Replaced use of enum by static constexpr: [#852](https://github.com/dartsim/dart/pull/852), [#904](https://github.com/dartsim/dart/pull/904)

### [DART 6.2.0 (2017-05-15)](https://github.com/dartsim/dart/milestone/30?closed=1)

- Common
  - Added Factory class and applied it to collision detection creation: [#864](https://github.com/dartsim/dart/pull/864)
  - Added readAll() to Resource and ResourceRetriever: [#875](https://github.com/dartsim/dart/pull/875)

- Math
  - Added accessors for diameters and radii of EllipsoidShape, and deprecated EllipsoidShape::get/setSize(): [#829](https://github.com/dartsim/dart/pull/829)
  - Fixed Lemke LCP solver (#808 for DART 6): [#812](https://github.com/dartsim/dart/pull/812)

- Collision Detection
  - Added support of ODE collision detector: [#861](https://github.com/dartsim/dart/pull/861)
  - Fixed incorrect collision filtering of BulletCollisionDetector: [#859](https://github.com/dartsim/dart/pull/859)

- Simulation
  - Fixed World didn't clear collision results on reset: [#863](https://github.com/dartsim/dart/pull/863)

- Parsers
  - Fixed incorrect creation of resource retriever in SkelParser and SdfParser: [#847](https://github.com/dartsim/dart/pull/847), [#849](https://github.com/dartsim/dart/pull/849)

- GUI
  - Added MotionBlurSimWindow: [#840](https://github.com/dartsim/dart/pull/840)
  - Improved MultiSphereShape rendering in GLUT renderer: [#862](https://github.com/dartsim/dart/pull/862)
  - Fixed incorrect parsing of materials and normal scaling from URDF: [#851](https://github.com/dartsim/dart/pull/851)
  - Fixed the OSG renderer not rendering collision geometries: [#851](https://github.com/dartsim/dart/pull/851)
  - Fixed that GUI was rendering white lines with nvidia drivers: [#805](https://github.com/dartsim/dart/pull/805)

- Misc
  - Added createShared() and createUnique() pattern: [#844](https://github.com/dartsim/dart/pull/844)
  - Added Skeleton::getRootJoint(): [#832](https://github.com/dartsim/dart/pull/832)
  - Added CMake targets for code formatting using clang-format: [#811](https://github.com/dartsim/dart/pull/811), [#817](https://github.com/dartsim/dart/pull/817)
  - Renamed MultiSphereShape to MultiSphereConvexHullShape: [#865](https://github.com/dartsim/dart/pull/865)
  - Modified the member function names pertain to lazy evaluation to be more relevant to their functionalities: [#833](https://github.com/dartsim/dart/pull/833)

- Tutorials & Examples
  - Allowed tutorials and examples to be built out of DART source tree: [#842](https://github.com/dartsim/dart/pull/842)
  - Fixed tutorialDominoes-Finished that didn't work with the latest DART: [#807](https://github.com/dartsim/dart/pull/807)

### DART 6.1.2 (2017-01-13)

- Dynamics
  - Fixed bug of ContactConstraint with kinematic joints: [#809](https://github.com/dartsim/dart/pull/809)

- Misc
  - Fixed that ZeroDofJoint::getIndexInTree was called: [#818](https://github.com/dartsim/dart/pull/818)

### DART 6.1.1 (2016-10-14)

- Build
  - Modified to build DART without SIMD options by default: [#790](https://github.com/dartsim/dart/pull/790)
  - Modified to build external libraries as separately build targets: [#787](https://github.com/dartsim/dart/pull/787)
  - Modified to export CMake target files separately per target: [#786](https://github.com/dartsim/dart/pull/786)

- Misc
  - Updated lodepng up to version 20160501: [#791](https://github.com/dartsim/dart/pull/791)

### DART 6.1.0 (2016-10-07)

- Collision detection
  - Added distance API: [#744](https://github.com/dartsim/dart/pull/744)
  - Fixed direction of contact normal of BulletCollisionDetector: [#763](https://github.com/dartsim/dart/pull/763)

- Dynamics
  - Added `computeLagrangian()` to `MetaSkeleton` and `BodyNode`: [#746](https://github.com/dartsim/dart/pull/746)
  - Added new shapes: sphere, capsule, cone, and multi-sphere: [#770](https://github.com/dartsim/dart/pull/770), [#769](https://github.com/dartsim/dart/pull/769), [#745](https://github.com/dartsim/dart/pull/745)
  - Changed base class of joint from SingleDofJoint/MultiDofJoint to GenericJoint: [#747](https://github.com/dartsim/dart/pull/747)

- Planning
  - Fixed incorrect linking to flann library: [#761](https://github.com/dartsim/dart/pull/761)

- Parsers
  - Added `sdf` parsing for `fixed` joint and `material` tag of visual shape: [#775](https://github.com/dartsim/dart/pull/775)
  - Added support of urdfdom_headers 1.0: [#766](https://github.com/dartsim/dart/pull/766)

- GUI
  - Added ImGui for 2D graphical interface: [#781](https://github.com/dartsim/dart/pull/781)

- Examples
  - Added osgAtlasSimbicon and osgTinkertoy: [#781](https://github.com/dartsim/dart/pull/781)

- Misc improvements and bug fixes
  - Added `virtual Shape::getType()` and deprecated `ShapeType Shape::getShapeType()`: [#725](https://github.com/dartsim/dart/pull/725)
  - Changed building with SIMD optional: [#765](https://github.com/dartsim/dart/pull/765), [#760](https://github.com/dartsim/dart/pull/760)
  - Fixed minor build and install issues: [#773](https://github.com/dartsim/dart/pull/773), [#772](https://github.com/dartsim/dart/pull/772)
  - Fixed Doxyfile to show missing member functions in API documentation: [#768](https://github.com/dartsim/dart/pull/768)
  - Fixed typo: [#756](https://github.com/dartsim/dart/pull/756), [#755](https://github.com/dartsim/dart/pull/755)

### DART 6.0.1 (2016-06-29)

- Collision detection
  - Added support of FCL 0.5 and tinyxml2 4.0: [#749](https://github.com/dartsim/dart/pull/749)
  - Added warnings for unsupported shape pairs of DARTCollisionDetector: [#722](https://github.com/dartsim/dart/pull/722)

- Dynamics
  - Fixed total mass is not being updated when bodies removed from Skeleton: [#731](https://github.com/dartsim/dart/pull/731)

- Misc improvements and bug fixes
  - Renamed `DEPRECATED` and `FORCEINLINE` to `DART_DEPRECATED` and `DART_FORCEINLINE` to avoid name conflicts: [#742](https://github.com/dartsim/dart/pull/742)
  - Updated copyright: added CMU to copyright holder, moved individual contributors to CONTRIBUTING.md: [#723](https://github.com/dartsim/dart/pull/723)

### DART 6.0.0 (2016-05-10)

- Common data structures
  - Added `Node`, `Aspect`, `State`, and `Properties`: [#713](https://github.com/dartsim/dart/pull/713), [#712](https://github.com/dartsim/dart/issues/712), [#708](https://github.com/dartsim/dart/pull/708), [#707](https://github.com/dartsim/dart/pull/707), [#659](https://github.com/dartsim/dart/pull/659), [#649](https://github.com/dartsim/dart/pull/649), [#645](https://github.com/dartsim/dart/issues/645), [#607](https://github.com/dartsim/dart/pull/607), [#598](https://github.com/dartsim/dart/pull/598), [#591](https://github.com/dartsim/dart/pull/591), [#531](https://github.com/dartsim/dart/pull/531)
  - Added mathematical constants and user-defined literals for radian, degree, and pi: [#669](https://github.com/dartsim/dart/pull/669), [#314](https://github.com/dartsim/dart/issues/314)
  - Added `ShapeFrame` and `ShapeNode`: [#608](https://github.com/dartsim/dart/pull/608)
  - Added `BoundingBox`: [#547](https://github.com/dartsim/dart/pull/547), [#546](https://github.com/dartsim/dart/issues/546)

- Kinematics
  - Added convenient functions for setting joint limits: [#703](https://github.com/dartsim/dart/pull/703)
  - Added more description on `InverseKinematics::solve()`: [#624](https://github.com/dartsim/dart/pull/624)
  - Added API for utilizing analytical inverse kinematics: [#530](https://github.com/dartsim/dart/pull/530), [#463](https://github.com/dartsim/dart/issues/463)
  - Added color property to `Marker`: [#187](https://github.com/dartsim/dart/issues/187)
  - Improved `Skeleton` to clone `State` as well: [#691](https://github.com/dartsim/dart/pull/691)
  - Improved `ReferentialSkeleton` to be able to add and remove `BodyNode`s and `DegreeOfFreedom`s to/from `Group`s freely: [#557](https://github.com/dartsim/dart/pull/557), [#556](https://github.com/dartsim/dart/issues/556), [#548](https://github.com/dartsim/dart/issues/548)
  - Changed `Marker` into `Node`: [#692](https://github.com/dartsim/dart/pull/692), [#609](https://github.com/dartsim/dart/issues/609)
  - Renamed `Joint::get/setLocal[~]` to `Joint::get/setRelative[~]`: [#715](https://github.com/dartsim/dart/pull/715), [#714](https://github.com/dartsim/dart/issues/714)
  - Renamed `PositionLimited` to `PositionLimitEnforced`: [#447](https://github.com/dartsim/dart/issues/447)
  - Fixed initialization of joint position and velocity: [#691](https://github.com/dartsim/dart/pull/691), [#621](https://github.com/dartsim/dart/pull/621)
  - Fixed `InverseKinematics` when it's used with `FreeJoint` and `BallJoint`: [#683](https://github.com/dartsim/dart/pull/683)
  - Fixed ambiguous overload on `MetaSkeleton::getLinearJacobianDeriv`: [#628](https://github.com/dartsim/dart/pull/628), [#626](https://github.com/dartsim/dart/issues/626)

- Dynamics
  - Added `get/setLCPSolver` functions to `ConstraintSolver`: [#633](https://github.com/dartsim/dart/pull/633)
  - Added `ServoMotorConstraint` as a preliminary implementation for `SERVO` actuator type: [#566](https://github.com/dartsim/dart/pull/566)
  - Improved `ConstraintSolver` to obey C++11 ownership conventions: [#616](https://github.com/dartsim/dart/pull/616)
  - Fixed segfualting of `DantzigLCPSolver` when the constraint dimension is zero: [#634](https://github.com/dartsim/dart/pull/634)
  - Fixed missing implementations in ConstrainedGroup: [#586](https://github.com/dartsim/dart/pull/586)
  - Fixed incorrect applying of joint constraint impulses: [#317](https://github.com/dartsim/dart/issues/317)
  - Deprecated `draw()` functions of dynamics classes: [#654](https://github.com/dartsim/dart/pull/654)

- Collision detection
  - Added `CollisionGroup` and refactored `CollisionDetector` to be more versatile: [#711](https://github.com/dartsim/dart/pull/711), [#704](https://github.com/dartsim/dart/pull/704), [#689](https://github.com/dartsim/dart/pull/689), [#631](https://github.com/dartsim/dart/pull/631), [#642](https://github.com/dartsim/dart/issues/642), [#20](https://github.com/dartsim/dart/issues/20)
  - Improved API for self collision checking options: [#718](https://github.com/dartsim/dart/pull/718), [#702](https://github.com/dartsim/dart/issues/702)
  - Deprecated `BodyNode::isColliding`; collision sets are moved to `CollisionResult`: [#694](https://github.com/dartsim/dart/pull/694), [#670](https://github.com/dartsim/dart/pull/670), [#668](https://github.com/dartsim/dart/pull/668), [#666](https://github.com/dartsim/dart/issues/666)

- Parsers
  - Fixed segfault of `SdfParser` when `nullptr` `ResourceRetriever` is passed: [#663](https://github.com/dartsim/dart/pull/663)

- GUI features
  - Merged `renderer` namespace into `gui` namespace: [#652](https://github.com/dartsim/dart/pull/652), [#589](https://github.com/dartsim/dart/issues/589)
  - Moved `osgDart` under `dart::gui` namespace as `dart::gui::osg`: [#651](https://github.com/dartsim/dart/pull/651)
  - Fixed GlutWindow::screenshot(): [#623](https://github.com/dartsim/dart/pull/623), [#395](https://github.com/dartsim/dart/issues/395)

- Simulation
  - Fixed `World::clone()` didn't clone the collision detector: [#658](https://github.com/dartsim/dart/pull/658)
  - Fixed bug of `World` concurrency: [#577](https://github.com/dartsim/dart/pull/577), [#576](https://github.com/dartsim/dart/issues/576)

- Misc improvements and bug fixes
  - Added `make_unique<T>` that was omitted from C++11: [#639](https://github.com/dartsim/dart/pull/639)
  - Added missing `override` keywords: [#617](https://github.com/dartsim/dart/pull/617), [#535](https://github.com/dartsim/dart/pull/535)
  - Added gcc warning flag `-Wextra`: [#600](https://github.com/dartsim/dart/pull/600)
  - Improved memory management of `constraint` namespace: [#584](https://github.com/dartsim/dart/pull/584), [#583](https://github.com/dartsim/dart/issues/583)
  - Changed the extension of headers from `.h` to `.hpp`: [#709](https://github.com/dartsim/dart/pull/709), [#693](https://github.com/dartsim/dart/pull/693), [#568](https://github.com/dartsim/dart/issues/568)
  - Changed Doxyfile to generate tag file: [#690](https://github.com/dartsim/dart/pull/690)
  - Changed the convention to use `std::size_t` over `size_t`: [#681](https://github.com/dartsim/dart/pull/681), [#656](https://github.com/dartsim/dart/issues/656)
  - Changed CMake to configure preprocessors using `#cmakedefine`: [#648](https://github.com/dartsim/dart/pull/648), [#641](https://github.com/dartsim/dart/pull/641)
  - Updated copyright years: [#679](https://github.com/dartsim/dart/pull/679), [#160](https://github.com/dartsim/dart/issues/160)
  - Renamed directory name `apps` to `examples`: [#685](https://github.com/dartsim/dart/pull/685)
  - Fixed warnings of unused variables in release mode: [#646](https://github.com/dartsim/dart/pull/646)
  - Fixed typo of `getNumPluralAddoName` in utility macro: [#615](https://github.com/dartsim/dart/issues/615)
  - Fixed linker error by adding namespace-scope definitions for `constexpr static` members: [#603](https://github.com/dartsim/dart/pull/603)
  - Fixed segfault from nullptr meshes: [#585](https://github.com/dartsim/dart/pull/585)
  - Fixed typo of tutorial with minor improvements: [#573](https://github.com/dartsim/dart/pull/573)
  - Fixed `NameManager<T>::removeEntries(~)` called a function that does not exist: [#564](https://github.com/dartsim/dart/pull/564), [#554](https://github.com/dartsim/dart/issues/554)
  - Fixed missing definitions for various functions: [#558](https://github.com/dartsim/dart/pull/558), [#555](https://github.com/dartsim/dart/issues/555)
  - Fixed const correctness of `BodyNode::getMomentsOfInertia()`: [#541](https://github.com/dartsim/dart/pull/541), [#540](https://github.com/dartsim/dart/issues/540)
  - Fixed `ftel` bug in Linux with an workaround: [#533](https://github.com/dartsim/dart/pull/533)
  - Removed unnecessary `virtual` keyword for overriding functions: [#680](https://github.com/dartsim/dart/pull/680)
  - Removed deprecated APIs in DART 5: [#678](https://github.com/dartsim/dart/pull/678)

- Build and test issues
  - Added CMake target for code coverage testing, and automatic reporting: [#688](https://github.com/dartsim/dart/pull/688), [#687](https://github.com/dartsim/dart/issues/687), [#638](https://github.com/dartsim/dart/pull/638), [#632](https://github.com/dartsim/dart/pull/632)
  - Added missing `liburdfdom-dev` dependency in Ubuntu package: [#574](https://github.com/dartsim/dart/pull/574)
  - Modulized DART libraries: [#706](https://github.com/dartsim/dart/pull/706), [#675](https://github.com/dartsim/dart/pull/675), [#652](https://github.com/dartsim/dart/pull/652), [#477](https://github.com/dartsim/dart/issues/477)
  - Improved Travis-CI script: [#655](https://github.com/dartsim/dart/pull/655)
  - Improved CMake script by splitting tutorials, examples, and tests into separate targets: [#644](https://github.com/dartsim/dart/pull/644)
  - Improved wording of the cmake warning messages for ASSIMP: [#553](https://github.com/dartsim/dart/pull/553)
  - Changed Travis-CI to treat warning as errors using `-Werror` flags: [#682](https://github.com/dartsim/dart/pull/682), [#677](https://github.com/dartsim/dart/issues/677)
  - Changed Travis-CI to test DART with bullet collision detector: [#650](https://github.com/dartsim/dart/pull/650), [#376](https://github.com/dartsim/dart/issues/376)
  - Changed the minimum requirement of Visual Studio version to 2015: [#592](https://github.com/dartsim/dart/issues/592)
  - Changed CMake to build gui::osg examples when `DART_BUILD_EXAMPLES` is on: [#536](https://github.com/dartsim/dart/pull/536)
  - Simplified Travis-CI tests for general pushes: [#700](https://github.com/dartsim/dart/pull/700)
  - Fixed Eigen memory alignment issue in testCollision.cpp: [#719](https://github.com/dartsim/dart/pull/719)
  - Fixed `BULLET_INCLUDE_DIRS` in `DARTConfig.cmake`: [#697](https://github.com/dartsim/dart/pull/697)
  - Fixed linking with Bullet on OS X El Capitan by supporting for Bullet built with double precision: [#660](https://github.com/dartsim/dart/pull/660), [#657](https://github.com/dartsim/dart/issues/657)
  - Fixed FCL version check logic in the main `CMakeLists.txt`: [#640](https://github.com/dartsim/dart/pull/640)
  - Fixed `find_package(DART)` on optimizer components: [#637](https://github.com/dartsim/dart/pull/637)
  - Fixed linking against `${DART_LIBRARIES}` not working in Ubuntu 14.04: [#630](https://github.com/dartsim/dart/pull/630), [#629](https://github.com/dartsim/dart/issues/629)
  - Fixed Visual Studio 2015 build errors: [#580](https://github.com/dartsim/dart/pull/580)
  - Removed OpenGL dependency from `dart` library: [#667](https://github.com/dartsim/dart/pull/667)
  - Removed version check for Bullet: [#636](https://github.com/dartsim/dart/pull/636), [#625](https://github.com/dartsim/dart/issues/625)

## DART 5

### Version 5.1.6 (2017-08-08)

1. Improved camera movement of OpenGL GUI: smooth zooming and translation
   - [Pull request #843](https://github.com/dartsim/dart/pull/843)

2. Removed debian meta files from the main DART repository
   - [Pull request #853](https://github.com/dartsim/dart/pull/853)

### Version 5.1.5 (2017-01-20)

1. Fixed Lemke LCP solver for several failing cases
   - [Pull request #808](https://github.com/dartsim/dart/pull/808)

1. Increase minimum required Ipopt version to 3.11.9
   - [Pull request #800](https://github.com/dartsim/dart/pull/800)

1. Added support of urdfdom_headers 1.0 for DART 5.1 (backport of [#766](https://github.com/dartsim/dart/pull/766))
   - [Pull request #799](https://github.com/dartsim/dart/pull/799)

### Version 5.1.4 (2016-10-14)

1. Fixed inconsistent frame rate of GlutWindow
   - [Pull request #794](https://github.com/dartsim/dart/pull/794)

### Version 5.1.3 (2016-10-07)

1. Updated to support Bullet built with double precision (backport of [#660](https://github.com/dartsim/dart/pull/660))
   - [Pull request #777](https://github.com/dartsim/dart/pull/777)

1. Modified to use btGImpactMeshShape instead of btConvexTriangleMeshShape for mesh
   - [Pull request #764](https://github.com/dartsim/dart/pull/764)

1. Updated to support FCL 0.5 and tinyxml 4.0 (backport of [#749](https://github.com/dartsim/dart/pull/749))
   - [Pull request #759](https://github.com/dartsim/dart/pull/759)

### Version 5.1.2 (2016-04-25)

1. Fixed inverse kinematics (backporting)
   - [Pull request #684](https://github.com/dartsim/dart/pull/684)

1. Fixed aligned memory allocation with Eigen objects in loading meshes
   - [Pull request #606](https://github.com/dartsim/dart/pull/606)

1. Fixed incorrect applying joint constraint impulses (backporting)
   - [Pull request #579](https://github.com/dartsim/dart/pull/579)

1. Fixed some build and packaging issues
   - [Pull request #559](https://github.com/dartsim/dart/pull/559)
   - [Pull request #595](https://github.com/dartsim/dart/pull/595)
   - [Pull request #696](https://github.com/dartsim/dart/pull/696)

### Version 5.1.1 (2015-11-06)

1. Add bullet dependency to package.xml
   - [Pull request #523](https://github.com/dartsim/dart/pull/523)

1. Improved handling of missing symbols of Assimp package
   - [Pull request #542](https://github.com/dartsim/dart/pull/542)

1. Improved travis-ci build log for Mac
   - [Pull request #529](https://github.com/dartsim/dart/pull/529)

1. Fixed warnings in Function.cpp
   - [Pull request #550](https://github.com/dartsim/dart/pull/550)

1. Fixed build failures on AppVeyor
   - [Pull request #543](https://github.com/dartsim/dart/pull/543)

1. Fixed const qualification of ResourceRetriever
   - [Pull request #534](https://github.com/dartsim/dart/pull/534)
   - [Issue #532](https://github.com/dartsim/dart/issues/532)

1. Fixed aligned memory allocation with Eigen objects
   - [Pull request #527](https://github.com/dartsim/dart/pull/527)

1. Fixed copy safety for various classes
   - [Pull request #526](https://github.com/dartsim/dart/pull/526)
   - [Pull request #539](https://github.com/dartsim/dart/pull/539)
   - [Issue #524](https://github.com/dartsim/dart/issues/524)

### Version 5.1.0 (2015-10-15)

1. Fixed incorrect rotational motion of BallJoint and FreeJoint
   - [Pull request #518](https://github.com/dartsim/dart/pull/518)

1. Removed old documents: dart-tutorial, programmingGuide
   - [Pull request #515](https://github.com/dartsim/dart/pull/515)

1. Fixed aligned memory allocation with Eigen objects
   - [Pull request #513](https://github.com/dartsim/dart/pull/513)

1. Fixed segfault in Linkage::Criteria
   - [Pull request #491](https://github.com/dartsim/dart/pull/491)
   - [Issue #489](https://github.com/dartsim/dart/issues/489)

1. Improved sdf/urdf parser
   - [Pull request #497](https://github.com/dartsim/dart/pull/497)
   - [Pull request #485](https://github.com/dartsim/dart/pull/485)

1. Fixed CMake warnings
   - [Pull request #483](https://github.com/dartsim/dart/pull/483)

1. Fixed build issues on Windows
   - [Pull request #516](https://github.com/dartsim/dart/pull/516)
   - [Pull request #509](https://github.com/dartsim/dart/pull/509)
   - [Pull request #486](https://github.com/dartsim/dart/pull/486)
   - [Pull request #482](https://github.com/dartsim/dart/pull/482)
   - [Issue #487](https://github.com/dartsim/dart/issues/487)

1. Fixed IpoptSolver bugs
   - [Pull request #481](https://github.com/dartsim/dart/pull/481)

1. Added Frame::getTransform(withRespecTo, inCoordinatesOf)
   - [Pull request #475](https://github.com/dartsim/dart/pull/475)
   - [Issue #471](https://github.com/dartsim/dart/issues/471)

1. Improved API documentation -- set the SHOW_USED_FILES tag to NO
   - [Pull request #474](https://github.com/dartsim/dart/pull/474)

1. Added convenience setters for generalized coordinates of FreeJoint
   - [Pull request #470](https://github.com/dartsim/dart/pull/470)
   - [Pull request #507](https://github.com/dartsim/dart/pull/507)

1. Fixed compilation warnings
   - [Pull request #480](https://github.com/dartsim/dart/pull/480)
   - [Pull request #469](https://github.com/dartsim/dart/pull/469)
   - [Issue #418](https://github.com/dartsim/dart/issues/418)

1. Added a mutex to Skeleton
   - [Pull request #466](https://github.com/dartsim/dart/pull/466)

1. Added generic URIs support
   - [Pull request #464](https://github.com/dartsim/dart/pull/464)
   - [Pull request #517](https://github.com/dartsim/dart/pull/517)

1. Added End Effector, Inverse Kinematics, and osgDart
   - [Pull request #461](https://github.com/dartsim/dart/pull/461)
   - [Pull request #495](https://github.com/dartsim/dart/pull/495)
   - [Pull request #502](https://github.com/dartsim/dart/pull/502)
   - [Pull request #506](https://github.com/dartsim/dart/pull/506)
   - [Pull request #514](https://github.com/dartsim/dart/pull/514)
   - [Issue #381](https://github.com/dartsim/dart/issues/381)
   - [Issue #454](https://github.com/dartsim/dart/issues/454)
   - [Issue #478](https://github.com/dartsim/dart/issues/478)

1. Removed outdated packaging scripts
   - [Pull request #456](https://github.com/dartsim/dart/pull/456)

1. Added initial position and initial velocity properties
   - [Pull request #449](https://github.com/dartsim/dart/pull/449)

1. Added a package.xml file for REP-136 support
   - [Pull request #446](https://github.com/dartsim/dart/pull/446)

1. Improved Linkage and Chain Criteria
   - [Pull request #443](https://github.com/dartsim/dart/pull/443)
   - [Issue #437](https://github.com/dartsim/dart/issues/437)

1. Added Joint::isCyclic to mark SO(2) topology
   - [Pull request #441](https://github.com/dartsim/dart/pull/441)

1. Fixed SEGFAULTs in DartLoader
   - [Pull request #439](https://github.com/dartsim/dart/pull/439)

1. Added the SYSTEM flag to include_directories
   - [Pull request #435](https://github.com/dartsim/dart/pull/435)

1. Improved Joint warning
   - [Pull request #430](https://github.com/dartsim/dart/pull/430)

1. Added tutorials (http://dart.readthedocs.org/)
   - [Pull request #504](https://github.com/dartsim/dart/pull/504)
   - [Pull request #484](https://github.com/dartsim/dart/pull/484)
   - [Pull request #423](https://github.com/dartsim/dart/pull/423)
   - [Pull request #511](https://github.com/dartsim/dart/pull/511)

### Version 5.0.2 (2015-09-28)

1. Fixed bug in Jacobian update notifications
   - [Pull request #500](https://github.com/dartsim/dart/pull/500)
   - [Issue #499](https://github.com/dartsim/dart/issues/499)

### Version 5.0.1 (2015-07-28)

1. Improved app indexing for bipedStand and atlasSimbicon
   - [Pull request #417](https://github.com/dartsim/dart/pull/417)

1. Added clipping command when it exceeds the limits
   - [Pull request #419](https://github.com/dartsim/dart/pull/419)

1. Improved CollisionNode's index validity check
   - [Pull request #421](https://github.com/dartsim/dart/pull/421)

1. Standardized warning messages for Joints
   - [Pull request #425](https://github.com/dartsim/dart/pull/425)
   - [Pull request #429](https://github.com/dartsim/dart/pull/429)

1. Fixed bug in SDF parser -- correct child for a joint
   - [Pull request #431](https://github.com/dartsim/dart/pull/431)

1. Fixed SDF parsing for single link model without joint
   - [Pull request #444](https://github.com/dartsim/dart/pull/444)

1. Added missing virtual destructors to Properties in Entity and [Soft]BodyNode
   - [Pull request #458](https://github.com/dartsim/dart/pull/458)

1. Limited maximum required version of Assimp less than 3.0~dfsg-4
   - [Pull request #459](https://github.com/dartsim/dart/pull/459)

1. Fixed SEGFAULTs in DartLoader
   - [Pull request #472](https://github.com/dartsim/dart/pull/472)

### Version 5.0.0 (2015-06-15)

1. Fixed aligned memory allocation with Eigen objects
   - [Pull request #414](https://github.com/dartsim/dart/pull/414)

1. Added some missing API for DegreeOfFreedom
   - [Pull request #408](https://github.com/dartsim/dart/pull/408)

1. Replaced logMaps with Eigen::AngleAxisd
   - [Pull request #407](https://github.com/dartsim/dart/pull/407)

1. Improved FCL collision detector
   - [Pull request #405](https://github.com/dartsim/dart/pull/405)

1. Removed deprecated API and suppressed warnings
   - [Pull request #404](https://github.com/dartsim/dart/pull/404)

1. Added use of OpenGL's multisample anti-aliasing
   - [Pull request #402](https://github.com/dartsim/dart/pull/402)

1. Added computation of differences of generalized coordinates
   - [Pull request #389](https://github.com/dartsim/dart/pull/389)
   - [Issue #290](https://github.com/dartsim/dart/issues/290)

1. Added deprecated and force-linline definitions for clang
   - [Pull request #384](https://github.com/dartsim/dart/pull/384)
   - [Issue #379](https://github.com/dartsim/dart/issues/379)

1. Eradicated memory leaks and made classes copy-safe and clonable
   - [Pull request #369](https://github.com/dartsim/dart/pull/369)
   - [Pull request #390](https://github.com/dartsim/dart/pull/390)
   - [Pull request #391](https://github.com/dartsim/dart/pull/391)
   - [Pull request #392](https://github.com/dartsim/dart/pull/392)
   - [Pull request #397](https://github.com/dartsim/dart/pull/397)
   - [Pull request #415](https://github.com/dartsim/dart/pull/415)
   - [Issue #280](https://github.com/dartsim/dart/issues/280)
   - [Issue #339](https://github.com/dartsim/dart/issues/339)
   - [Issue #370](https://github.com/dartsim/dart/issues/370)
   - [Issue #383](https://github.com/dartsim/dart/issues/383)

1. Improved PlaneShape constructors
   - [Pull request #366](https://github.com/dartsim/dart/pull/366)
   - [Pull request #377](https://github.com/dartsim/dart/pull/377)
   - [Issue #373](https://github.com/dartsim/dart/issues/373)

1. Added appveyor options for parallel build and detailed log
   - [Pull request #365](https://github.com/dartsim/dart/pull/365)

1. Improved robustness and package handling for URDF parsing
   - [Pull request #364](https://github.com/dartsim/dart/pull/364)

1. Fixed bug in BodyNode::\_updateBodyJacobianSpatialDeriv()
   - [Pull request #363](https://github.com/dartsim/dart/pull/363)

1. Added alpha channel and Color functions
   - [Pull request #359](https://github.com/dartsim/dart/pull/359)
   - [Issue #358](https://github.com/dartsim/dart/issues/358)

1. Added Jacobian getters to Skeleton
   - [Pull request #357](https://github.com/dartsim/dart/pull/357)

1. Added ArrowShape for visualizing arrows
   - [Pull request #356](https://github.com/dartsim/dart/pull/356)

1. Fixed matrix dimension bug in operationalSpaceControl app
   - [Pull request #354](https://github.com/dartsim/dart/pull/354)

1. Added build type definitions
   - [Pull request #353](https://github.com/dartsim/dart/pull/353)

1. Added Signal class
   - [Pull request #350](https://github.com/dartsim/dart/pull/350)

1. Added LineSegmentShape for visualizing line segments
   - [Pull request #349](https://github.com/dartsim/dart/pull/349)
   - [Issue #346](https://github.com/dartsim/dart/issues/346)

1. Fixed segfault in SoftSdfParser
   - [Pull request #345](https://github.com/dartsim/dart/pull/345)

1. Added subscriptions for destructions and notifications
   - [Pull request #343](https://github.com/dartsim/dart/pull/343)

1. Added NloptSolver::[get/set]NumMaxEvaluations()
   - [Pull request #342](https://github.com/dartsim/dart/pull/342)

1. Added support of Eigen::VectorXd in parser
   - [Pull request #341](https://github.com/dartsim/dart/pull/341)

1. Added Skeleton::getNumJoints()
   - [Pull request #335](https://github.com/dartsim/dart/pull/335)

1. Fixed bug in DARTCollide for sphere-sphere collision
   - [Pull request #332](https://github.com/dartsim/dart/pull/332)

1. Fixed naming issues for Skeletons in World
   - [Pull request #331](https://github.com/dartsim/dart/pull/331)
   - [Issue #330](https://github.com/dartsim/dart/issues/330)

1. Added PlanarJoint support for URDF loader
   - [Pull request #326](https://github.com/dartsim/dart/pull/326)

1. Fixed rotation of the inertia reference frame for URDF loader
   - [Pull request #326](https://github.com/dartsim/dart/pull/326)
   - [Issue #47](https://github.com/dartsim/dart/issues/47)

1. Fixed bug in loading WorldFile
   - [Pull request #325](https://github.com/dartsim/dart/pull/325)

1. Added plotting of 2D trajectories
   - [Pull request #324](https://github.com/dartsim/dart/pull/324)

1. Removed unsupported axis orders of EulerJoint
   - [Pull request #323](https://github.com/dartsim/dart/pull/323)
   - [Issue #321](https://github.com/dartsim/dart/issues/321)

1. Added convenience functions to help with setting joint positions
   - [Pull request #322](https://github.com/dartsim/dart/pull/322)
   - [Pull request #338](https://github.com/dartsim/dart/pull/338)

1. Added Frame class and auto-updating for forward kinematics
   - [Pull request #319](https://github.com/dartsim/dart/pull/319)
   - [Pull request #344](https://github.com/dartsim/dart/pull/344)
   - [Pull request #367](https://github.com/dartsim/dart/pull/367)
   - [Pull request #380](https://github.com/dartsim/dart/pull/380)
   - [Issue #289](https://github.com/dartsim/dart/issues/289)
   - [Issue #294](https://github.com/dartsim/dart/issues/294)
   - [Issue #305](https://github.com/dartsim/dart/issues/305)

1. Added Travis-CI build test for OSX
   - [Pull request #313](https://github.com/dartsim/dart/pull/313)
   - [Issue #258](https://github.com/dartsim/dart/issues/258)

1. Added specification of minimum dependency version
   - [Pull request #306](https://github.com/dartsim/dart/pull/306)

## DART 4

### Version 4.3.7 (2018-01-05)

1. Updated DART 4.3 to be compatible with urdf 1.0/tinyxml2 6/flann 1.9.1
   - [Pull request #955](https://github.com/dartsim/dart/pull/955)

### Version 4.3.6 (2016-04-16)

1. Fixed duplicate entries in Skeleton::mBodyNodes causing segfault in destructor
   - [Issue #671](https://github.com/dartsim/dart/issues/671)
   - [Pull request #672](https://github.com/dartsim/dart/pull/672)

### Version 4.3.5 (2016-01-09)

1. Fixed incorrect applying of joint constraint impulses (backported from 6.0.0)
   - [Pull request #578](https://github.com/dartsim/dart/pull/578)

### Version 4.3.4 (2015-01-24)

1. Fixed build issue with gtest on Mac
   - [Pull request #315](https://github.com/dartsim/dart/pull/315)

### Version 4.3.3 (2015-01-23)

1. Fixed joint Coulomb friction
   - [Pull request #311](https://github.com/dartsim/dart/pull/311)

### Version 4.3.2 (2015-01-22)

1. Fixed installation -- missing headers (utils/urdf, utils/sdf)

### Version 4.3.1 (2015-01-21)

1. Fixed API incompatibility introduced by dart-4.3.0
   - [Issue #303](https://github.com/dartsim/dart/issues/303)
   - [Pull request #309](https://github.com/dartsim/dart/pull/309)

### Version 4.3.0 (2015-01-19)

1. Added name manager for efficient name look-up and unique naming
   - [Pull request #277](https://github.com/dartsim/dart/pull/277)
1. Added all-inclusive header and namespace headers
   - [Pull request #278](https://github.com/dartsim/dart/pull/278)
1. Added DegreeOfFreedom class for getting/setting data of individual generalized coordinates
   - [Pull request #288](https://github.com/dartsim/dart/pull/288)
1. Added hybrid dynamics
   - [Pull request #298](https://github.com/dartsim/dart/pull/298)
1. Added joint actuator types
   - [Pull request #298](https://github.com/dartsim/dart/pull/298)
1. Added Coulomb joint friction
   - [Pull request #301](https://github.com/dartsim/dart/pull/301)
1. Migrated to C++11
   - [Pull request #268](https://github.com/dartsim/dart/pull/268)
   - [Pull request #299](https://github.com/dartsim/dart/pull/299)
1. Improved readability of CMake output messages
   - [Pull request #272](https://github.com/dartsim/dart/pull/272)
1. Fixed const-correctneess of member functions
   - [Pull request #277](https://github.com/dartsim/dart/pull/277)
1. Added handling use of 'package:/' in URDF
   - [Pull request #273](https://github.com/dartsim/dart/pull/273)
   - [Issue #271](https://github.com/dartsim/dart/issues/271)

### Version 4.2.1 (2015-01-07)

1. Fixed version numbering of shared libraries in debian packages
   - [Pull request #286](https://github.com/dartsim/dart/pull/286)
1. Fixed Jacobian and its derivatives of FreeJoint/BallJoint
   - [Pull request #284](https://github.com/dartsim/dart/pull/284)

### Version 4.2.0 (2014-11-22)

1. Added reset functions for Simulation and Recording class
   - [Pull request #231](https://github.com/dartsim/dart/pull/231)
1. Added operational space control example
   - [Pull request #257](https://github.com/dartsim/dart/pull/257)
1. Fixed misuse of Bullet collision shapes
   - [Pull request #228](https://github.com/dartsim/dart/pull/228)
1. Fixed adjacent body pair check for Bullet collision detector
   - [Pull request #246](https://github.com/dartsim/dart/pull/246)
1. Fixed incorrect computation of constraint impulse for BallJointConstraint and WeldJointContraint
   - [Pull request #247](https://github.com/dartsim/dart/pull/247)
1. Improved generation of soft box shape for soft body
   - [Commit ec31f44](https://github.com/dartsim/dart/commit/ec31f44)

### Version 4.1.1 (2014-07-17)

1. Added ABI check script
   - [Pull request #226](https://github.com/dartsim/dart/pull/226)
   - [Pull request #227](https://github.com/dartsim/dart/pull/227)
1. Fixed build issues on Linux
   - [Pull request #214](https://github.com/dartsim/dart/pull/214)
   - [Pull request #219](https://github.com/dartsim/dart/pull/219)
1. Fixed build issues on Windows
   - [Pull request #215](https://github.com/dartsim/dart/pull/215)
   - [Pull request #217](https://github.com/dartsim/dart/pull/217)
1. Fixed unintended warning messages
   - [Pull request #220](https://github.com/dartsim/dart/pull/220)

### Version 4.1.0 (2014-07-02)

1. Fixed bug in switching collision detectors
   - [Issue #127](https://github.com/dartsim/dart/issues/127)
   - [Pull request #195](https://github.com/dartsim/dart/pull/195)
1. Fixed kinematics and dynamics when a skeleton has multiple parent-less bodies
   - [Pull request #196](https://github.com/dartsim/dart/pull/196)
1. Fixed issue on installing DART 4 alongside DART 3 on Linux
   - [Issue #122](https://github.com/dartsim/dart/issues/122)
   - [Pull request #203](https://github.com/dartsim/dart/pull/203)
1. Fixed warnings on gcc
   - [Pull request #206](https://github.com/dartsim/dart/pull/206)
1. Renamed getDof() to getNumDofs()
   - [Pull request #209](https://github.com/dartsim/dart/pull/209)
1. Added cylinder shape for soft body
   - [Pull request #210](https://github.com/dartsim/dart/pull/210)

### Version 4.0.0 (2014-06-02)

1. Added implicit joint spring force and damping force
1. Added planar joint
1. Added soft body dynamics
1. Added computation of velocity and acceleration of COM
1. Added bullet collision detector

- [Pull request #156](https://github.com/dartsim/dart/pull/156)

1. Improved performance of forward dynamics algorithm

- [Pull request #188](https://github.com/dartsim/dart/pull/188)

1. Improved dynamics API for Skeleton and Joint

- [Pull request #161](https://github.com/dartsim/dart/pull/161)
- [Pull request #192](https://github.com/dartsim/dart/pull/192)
- [Pull request #193](https://github.com/dartsim/dart/pull/193)

1. Improved constraint dynamics solver

- [Pull request #184](https://github.com/dartsim/dart/pull/184)

1. Improved calculation of equations of motion using Featherstone algorithm

- [Issue #85](https://github.com/dartsim/dart/issues/87)

1. Improved optimizer interface and added nlopt solver

- [Pull request #152](https://github.com/dartsim/dart/pull/152)

1. Fixed self collision bug

- [Issue #125](https://github.com/dartsim/dart/issues/125)

1. Fixed incorrect integration of BallJoint and FreeJoint

- [Issue #122](https://github.com/dartsim/dart/issues/122)
- [Pull request #168](https://github.com/dartsim/dart/pull/168)

## DART 3

### Version 3.0 (2013-11-04)

1. Removed Transformation classes. Their functionality is now included in joint classes.
1. Added Featherstone algorithm. Can currently only be used without collision handling. The old algorithm is still present and used for that case.
1. Removed kinematics namespace. Functionality is moved to dynamics classes.
1. Added dart root namespace
1. A lot of function and variable renames
1. Added constraint namespace
1. Added "common" namespace

## DART 2

### Version 2.6 (2013-09-07)

1. Clean-up of build system:

- Renamed DART_INCLUDEDIR to the standard-compliant DART_INCLUDE_DIRS in CMake files. Users need to adapt their CMake files for this change.
- Users no longer need to call find_package(DARTExt) in the CMake files. A call to find_package(DART) also finds its dependencies now.
- Allow user to overwrite installation prefix
- Add possibility to include DART header files as '#include \<dart/dynamics/Skeleton.h\>' in addition to '#include \<dynamics/Skeleton.h\>'
- Allow out-of-source builds

1. URDF loader:

- Major clean-up
- Consider mesh scaling factor

### Version 2.5 (2013-07-16)

1. Replaced robotics::World with simulation::World
1. Removed robotics::Robot
1. Added simulation::SimWindow
1. Some speed-up of Eigen calculations
1. Added abstract trajectory interface
1. ConstraintDynamics handles contact, joint limit and other constraint forces simultaneously
1. Improved Lemke algorithm for solving LCP
1. Renamed skeletonDynamics::getQDotVector() to getPoseVelocity()
1. Added abstract CollisionDetector interface allowing for multiple different collision detector implementations.
1. Created math namespace
1. Added System class as base class to Skeleton and Joint
1. URDF loader: Removed ability to load nonstandard URDF files with an object tag
1. URDF loader: Removed ability to load nonstandard URDF files with an object tag
1. Added support for multiple shapes per BodyNode
1. Made urdfdom a dependency instead of including it in the DART source
1. Added function to CollisionDetector to let user check a specific pair of BodyNodes for collision

### Version 2.4 (2013-03-05)

1. Mass and inertia are no longer stored in Shape but in BodyNode.
1. Different shapes for collision and visualization (not just different meshes)
1. Shapes are no longer centered at the COM but can be transformed independently relative to the link frame.
1. Improved URDF support

- Support for non-mesh shapes
- Does not create dummy root nodes anymore
- Support for continuous joints
- Support for arbitrary joint axes for revolute joints (but not for prismatic joints) instead of only axis-aligned joint axes
- Support for relative mesh paths even if the robot and world URDF files are in different directories
- All supported joint types can be root joints

1. Clean-up of the Robot class
1. Removed Object class
1. More robust build and installation process on Linux
