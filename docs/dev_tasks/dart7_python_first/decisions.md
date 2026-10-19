# Decisions

All decisions below were taken by the maintainer on 2026-10-10 and 2026-10-11,
one at a time, after read-only research and independent reviews; D13 was
amended on 2026-10-12. Durable API rules derived from them live in the
[DART 7 Python API design](../../design/dart7_python_api.md). Revisit a decision
only with new evidence and maintainer approval.

## Product and release

- **D0. Every audience is primary**: robot learning, robotics and control
  research, character/biomechanics/soft-body research, and education.
  - Conflicts are resolved by layering: a simple top level, the full dynamics
    surface, bulk and batched I/O, and first-class soft bodies.
- **D1. Clean Python break as DART 7.0.** There are no runtime aliases or
  deprecation shims. Users get a codemod and a migration guide; legacy users pin
  `dartpy<7`.
  - Evidence: earlier runtime snake_case shims produced untyped stubs,
    alias-order bugs that crashed, broken introspection and silenced warnings.
    No codemod ever shipped.
- **D3. One program, core first.** The Python API is designed in parallel and
  bound to the same objects. The C++ compatibility of the gz-used subset is a
  hard gate throughout.
- **D4. Keep `pip install dartpy` / `import dartpy`.** Version 7 signals the
  break.
- **D5. C++ and Python both become 7.0.**
  - The CMake package accepts requests back to 6.0 (`AnyNewerVersion`
    semantics with a 6.0 floor, through a custom version file), so
    gz-physics' request for DART 6.10 keeps resolving. Today `SameMajorVersion` would reject 7.x, and
    gz-cmake only warns and skips the plugin.
  - No deprecated C++ API is removed during 7.x; gz uses several of them,
    including the version macros.
- **D20. Python 3.12 or newer**: one stable-ABI wheel per platform (Linux x86_64
  and aarch64, macOS arm64, Windows x64). Wheel-only.
- **D22. Conventions bundle**:
  - errors and the logging bridge;
  - stubs with CI gates;
  - generated docstrings and an API reference;
  - reprs and `UPPER_CASE` enums;
  - keyword-only options;
  - modern packaging;
  - sample data and a fetcher;
  - an example runner in CI;
  - the codemod;
  - semantic versioning;
  - a `dartpy.experimental` namespace.
- **D25. Public decisions, private research.** Public documents state rationale
  in DART terms only.
- **D30. No 6.21.**
  - Main's unreleased 6.21 work ships as 7.0.0.
  - The `release-6.20` branch is the only 6.x maintenance line (C++ for Gazebo,
    `dartpy<7` for legacy Python users), and gains wheels for newer Python
    versions.
  - The active performance work folds into M1d and M2. PLAN-622's core changes
    outside its parity rows pause during the step-path quiet window.
- **D32. DART 7.0 waits for the Filament renderer.** There is no render-less
  fallback.

## Core engine

- **D2. Physics scope for 7.0**: articulated rigid bodies plus point-mass soft
  bodies, behind explicit seams. No public plugin ABI until a second domain
  exists.
- **D7. In-place stage extraction.**
  - `World::step` is split into internal stages; `getConstraintSolver()` is
    unchanged.
  - The contract for a second physics domain will be specified in a design
    doc; no code ships for it in 7.0.
  - Point-mass soft bodies already share the articulated recursion, the
    constraint problem and the collision group. They are a body-type/material
    seam, not a separate solver.
  - Evidence: a public multi-solver API inside the classic World was reverted
    within days, and a separate engine never met its readiness gates.
- **D8. Compiled dense data, no entity-component system.**
  - Use per-multibody dense blocks with producer-owned caches and devirtualized
    kernels, keeping a virtual fallback for custom subclasses. C++17, zero new
    dependencies.
  - Evidence: the current EnTT major requires C++20, while gz-physics builds as
    C++17. DART's own measurements found per-phase structure-of-arrays mirrors
    slower than the copies they save. gz relies on pointer-stable objects.
- **D9. State ownership is phased.**
  - Gathered snapshots come first, dense caches second.
  - Authoritative contiguous state follows in 7.x by default, and must be
    additive if it lands after 7.0.
  - Whole-multibody position getters/setters and the reset functions keep
    their value semantics, because gz uses them; gz reads velocities through
    per-joint accessors.
- **D10. Refactors are bit-identical**, within a pinned build and
  floating-point environment, each detector against its own baseline. Numeric
  changes ship as labeled PRs with physical-correctness acceptance.
- **D11. Layered state and replay**: a schema, dynamic snapshots, exact
  checkpoints, an optional recorder, and replay. Zero new dependencies;
  versioned formats.
- **D12. Explicit editing vs simulation modes on the same objects.**
  - Simulation mode is compiled with zero steady-state allocation.
  - Structural edits go inside `edit()`.
  - The C++ default stays lazy, so downstream runtime edits keep working.
- **D13. Zero-allocation contract on the DART and Bullet detectors.**
  - Amended by the maintainer on 2026-10-12: the C++ `World` keeps FCL as its
    default detector in 7.0, for Gazebo compatibility; switching the C++
    default to DART is a labeled 7.x change. dartpy's `dart.World` and
    `dart.io.load_world` for MJCF default to the DART detector in 7.0.
    The MJCF switch changes only `dart.io.load_world`, never the C++
    `dart::utils::MjcfParser::readWorld` or its direct dartpy `MjcfParser`
    binding: changing that parser would also switch C++ callers from FCL.
    The parser and its direct binding keep FCL. That breaking Python
    default change is a must-have that lands before `F`, after the
    published shape and query capability matrix and the strict rejection of
    unsupported shapes when a World compiles (M1b). Behavior comparisons
    against FCL and PLAN-622's pre-default coverage gate remain prerequisites
    of the C++ switch.
  - Before the amendment, the default switched from FCL to DART in 7.0 only if
    its capability prerequisites passed before M1 started, scheduled first so
    later rebaselines started from the final default.
  - Known gaps: the native detector currently warns once and skips heightmaps,
    and it approximates ellipsoids and cones.
- **D14. One deterministic executor**, a GIL-free `step(n)`, and `WorldBatch`
  built from remapping clones (the existing clone drops filters, handlers and
  manual constraints).
- **D15. No GPU code in 7.0.** New dense kernels follow review rules that keep a
  GPU path open.
- **D21. Layered extension**:
  - native actuator channels, with `PositionServo` and `VelocityServo` as
    bounded compliant motor rows inside the constraint solve;
  - user loops with decimation;
  - `StepHook`;
  - compiled hooks once contiguous state exists;
  - C++ hooks.

  Legacy forces and commands keep the existing `World::step` reset semantics.

  Evidence: the joint spring-damper is stable on its own, but contacts see the
  plain inverse inertia, so blocked reactions are attenuated. Spring forces also
  bypass effort limits.
- **D26. Built-in native sensors** under `world.sensors`: IMU, force-torque,
  contact and joint state, with seeded noise. Recording arrives with the
  recorder (M4) and batch support with `WorldBatch`.
- **D33. DART 7 drops IKFast (maintainer, 2026-10-18).** The `IkFast` and
  `SharedLibraryIkFast` analytical IK classes, `wrapCyclicSolution`, and the
  installed `dart/dynamics/ikfast.h` header with its
  `dart/external/ikfast/ikfast.h` forwarding header are removed without
  replacement. C++ analytical solvers implement
  `InverseKinematics::Analytical` directly, and Python solvers such as ssik
  plug in through `InverseKinematics.setPythonAnalytical`.
  - Evidence: Gazebo's API inventory uses none of these names. The WAM demo
    was the last IKFast user, and it moves to ssik (D31b).

## Python API

- **D6. Native rename in the binding sources**, driven by one reviewed table.
  dartpy becomes a package with a private extension and pure-Python helpers.
  There are no wrapper classes except the `Collider`/`Visual` handles.
- **D16. Read-only array copies by default.** `out=` fills caller buffers
  without allocating; it copies and never aliases. Zero-copy live views arrive
  with authoritative contiguous state (M3) and are named separately.
- **D17. Modern names in Python 7.0.** C++ adopts them after the release, with
  aliases and forwarders.
- **D18. Vocabulary**: `Skeleton→Multibody`, `BodyNode→Link`,
  `DegreeOfFreedom→Dof`. `ShapeNode` is replaced by native `Collider` and
  `Visual` handles on the same node.
- **D19. A curated top-level allowlist** with one canonical module per type.
- **D23. Builders and loaders**: `add_*` builders with immutable keyword-only
  specs, and `io.load_model()` / `io.load_world()`. MJCF is world-only.
- **D27. Camera frames use the optical convention**, plus `Camera.look_at`.
- **D28. One immutable `Transform` and `Rotation`**, scalar or batched, backed by
  the existing C++ rigid-transform type. `@` composes and applies.
  - Evidence: the C++ type matches gz-physics' own pose type, and changing the
    storage brings no arithmetic gain. A compact affine form would silently turn
    inverses and rotation extraction into general solves at dozens of call
    sites.
- **D29. Quaternion order is always named; `wxyz` is canonical.** Order mix-ups
  fail silently, because both orders yield valid unit quaternions.

## Rendering

- **D24. Filament rendering**: `dart.render` (headless arrays, frame-attached
  cameras), `dart.viewer` (passive and managed) and a core PBR `dart.Material`.
- **D31. Ownership is split.** The parallel Filament effort ports the backend
  (M6a). This program owns sensor rendering and the Python render/viewer API
  (M6b).
- **D31a. Interim GUI bindings.** M6a lands with the Filament effort's tested
  camelCase GUI bindings as an interim, undocumented module. M6b replaces them
  with the 7.0 API and deletes them before the API freeze `F`, so release
  candidates carry the final render API.
- **D31b. The demo host becomes the managed viewer's core.** The native
  host lives in private C++ linked into the dartpy extension,
  with no installed headers, package components or gz-facing API. The host
  covers the docking ImGui workspace, scene navigator, scene panels,
  inspector, contact and drag tools, profiler, log console, and headless
  list, cycle and capture.
  - Its bindings start in the 7.0 style as `dartpy.viewer`, the managed-loop
    core that M6b's `launch()` can reuse.
  - Python declares scenes, key actions, simulation hooks and declarative
    panel controls. There is no Python ImGui API. On 2026-10-13 the maintainer
    revised the implementation boundary: demo controllers and models move to
    Python over the engine, IO, and GUI bindings. All demo implementation is
    now Python, and `dartpy._demos` and `dart-demo-models` are removed.
  - On 2026-10-18 the maintainer accepted a change of joint selection for the
    WAM scene. ssik's closed-form `seven_r.srs` solver replaces the generated
    IKFast solver, which held `/j3` as a free joint. The scene, now
    `wam_analytical_ik`, needs the optional ssik package, like `ssik_ik_gui`.
  - The Python demo catalog is D22's example runner. All 37 retained scenes
    reached parity before WP-PD.18 removed the C++ app and its aliases.
    The TinyDNN-only `human_joint_limits` scene is retired: no supported
    environment provided its dependency or a running parity oracle. The
    duplicate `hello_world_gui` and `contacts_pointcloud` scenes are retired
    in favor of the main catalog and its contact visualizer.
  - Bundled ImGui uses the docking release; with a vanilla system ImGui the
    host is compiled out. This is a labeled 7.0 build change.
  - Evidence: maintaining a C++ and a Python demo app duplicated every scene
    and host feature. The former consolidation project's local decisions
    are now owned by [architecture](../../onboarding/architecture.md#python-demo-catalog),
    [scene rules](../../../examples/README.md#adding-catalog-scenes),
    [callback contracts](../../onboarding/python-bindings.md#viewer-app-callbacks),
    [memory diagnostics](../../onboarding/profiling.md#interactive-memory-inspection),
    and [parity methodology](../../onboarding/testing.md#demo-catalog-verification).
    The construction-count oracle was transitional and was removed with
    the C++ app. Scene tests now carry those assertions directly.
  - Existing CLI recipes keep their flags and strict bounded integers.
    `--cycle-scenes` fails on callback errors. Ordinary exceptions disable
    and count one callback; interrupts stop the loop and are re-raised only
    after teardown. Panels remain declarative C++ control trees, with
    numeric precision instead of printf formats.
  - Both ssik scenes call the solver directly from Python, without an
    embedded-interpreter bridge. The memory-diagnostics option keeps
    its existing name and defaults to off. `DART_IMGUI_DOCKING` is removed;
    the ImGui provider determines host availability.

## Scope-gate refinement (2026-10-11)

The maintainer refined the program's plans to express when work happens through
dependencies, order and priority. Specific future dates and date-relative gates
are removed; past dates on decisions and evidence stay.

- D13's default-detector switch was gated on its prerequisites passing before
  the start of M1; D13's 2026-10-12 amendment replaces that gate.
- The API freeze `F` is declared once every 7.0 must-have, breaking and
  additive, has merged and C2 has passed. Only additive work that is not a
  must-have and misses `F` goes to 7.1. This is the no-date form of the
  approved release train: breaking changes land before `F`, 7.0 requires its
  must-haves, and additive work that misses `F` goes to 7.1. The maintainer
  confirmed this reading on 2026-10-12.
- C1, the M3 go/no-go, is decided after the first three M2 kernel PRs, at the
  M2 stop rule. M3 must be able to land before `F` without delaying any other
  must-have, and so without delaying `F`. Gazebo gates must stay green through
  those three PRs; the flagged contiguous-state spike is limited to
  revolute/free joint positions and velocities.
- C2 passes before `F`, so release candidates carry the final render API.

The [project home](README.md) owns the must-have list, checkpoint criteria and
priority order.
