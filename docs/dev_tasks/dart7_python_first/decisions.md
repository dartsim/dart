# Decisions

All decisions below were taken by the maintainer on 2026-10-10 and 2026-10-11,
one at a time, after read-only research and independent reviews. Durable API
rules derived from them live in the
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
  - The default detector switches from FCL to DART only if its capability gate
    passes by the M0 decision date; otherwise the switch is a 7.x labeled
    change.
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
  with the 7.0 API and deletes them before the first release candidate.
