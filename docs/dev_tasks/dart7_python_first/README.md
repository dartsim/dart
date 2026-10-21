# DART 7.0 Python-first program

This project home tracks the program that turns the next DART release into
**DART 7.0**: a Python-first library whose core keeps the existing solver as the
baseline and becomes ready for multi-physics, batched and replayable simulation.
[PLAN-624](../../plans/dashboard.md#plan-624-dart-70-python-first-program)
tracks its roadmap status. The durable API and engine rules live in the
[DART 7 Python API design](../../design/dart7_python_api.md).

- [Decisions](decisions.md): the maintainer decisions D0–D32, D31a and D31b, with
  their rationale.
- [Verification](verification.md): evidence, review passes and gates per
  packet.
- [Shape/query capability draft](shape-query-capabilities.md): current native
  and FCL conversion, pair/query routes and remaining M1f qualification.
- [Progress log](progress-log.md): completed chunks.
- [Resume](RESUME.md): the next-session handoff.
- [Deferred changelog drafts](changelog-drafts.md): entries that become
  `changelog.d/` fragments once the fragment tooling lands.

## North star

`import dartpy as dart` gives every audience a modern, typed, performant
simulator:

- robot learning;
- robotics and control research;
- character, biomechanics and soft-body research;
- education.

The C++ engine underneath stays source- and behavior-compatible for gz-physics
and gz-sim. It is refactored **in place**, behind bit-identical gates, with no
second engine.

## Final deliverable and acceptance

DART 7.0 is released when all of these hold:

- **The Python API follows the design doc.** Specifically:
  - a curated, typed package with PEP 8 names;
  - the `Multibody`/`Link`/`Dof` vocabulary;
  - immutable `Transform`/`Rotation`;
  - read-only array copies;
  - explicit editing vs simulation modes;
  - snapshots and checkpoints;
  - actuators, hooks and sensors;
  - `WorldBatch`;
  - `dart.render`/`dart.viewer`.
- **The acceptance tests pass from release-candidate wheels** in fresh
  environments on Linux x86_64/aarch64, macOS arm64 and Windows x64. They
  cover: load, compile, step, read state, snapshot round trip, headless render,
  and batch.
- **`import dartpy` works without graphics libraries** in a slim container.
- **Unmodified gz-physics and gz-sim pass** the Ionic and Jetty compatibility
  lanes, each building `dart-io` against its distribution's sdformat, with no
  new failures or slow cases against baselines from the last 6.20.x release.
  DART 7 supports Ubuntu 24.04 LTS and newer; Gazebo Harmonic and Ubuntu 22.04
  users stay on DART 6.20.x.
- **Performance and determinism hold.** Each performance and determinism row is
  equal or better against the pinned pre-refactor baseline, each detector
  compared with its own baseline.
- **The migration path is proven.** The codemod and migration guide have been
  exercised on at least three external codebases.

### Target shape

This script is the acceptance contract. WP-7.0.7 splits it into per-feature
tests in `python/tests/acceptance/test_target_shape.py`, each marked
`xfail(strict=True)`. The headless-render test needs a graphics device, so it
lives in `python/tests/unit/gui/test_target_shape_render.py`, which
`pixi run test-gui` and `CI Filament` run and `pixi run test-py` leaves out.
Each feature PR removes its marker, so the tests double as a burn-down. Feature
fixtures use the final 7.0 facade; a separate baseline-native contact preflight
validates the physical setup until M5 retires or migrates it. M1e/M5 settle the
bound configuration fields for the keyword-only `World(config=...)` overload.
The contract stays here instead of in `python/examples/target_shape.py`,
alongside automatically discovered runnable examples, while its features are
unimplemented.

```python
import numpy as np
import dartpy as dart

world = dart.World(time_step=1e-3, gravity=(0, 0, -9.81))
with world.edit():
    robot = world.add(dart.io.load_model(dart.data.path("urdf/KR5/KR5 sixx R650.urdf")))
    robot.joints["shoulder_yaw"].actuator = dart.PositionServo(kp=400, kd=40, effort_limit=80)
    imu = world.sensors.add(dart.sensors.IMUSensor(link=robot.links["wrist"]))
world.compile()                                          # simulation mode, no allocation

robot.dofs.position_targets = np.zeros(robot.num_dofs)   # persistent target channel
world.step(20)                                           # releases the GIL
X_WP = robot.links["palm"].world_transform @ dart.Transform(translation=(0, 0, 0.1))
state = world.capture_state()                            # dynamic snapshot
world.step(5)
world.restore_state(state)
rgb = dart.render.Renderer(world).render(
    dart.render.Camera.look_at(eye=(2, 0, 1), target=(0, 0, 0.5)))
assert rgb.shape == (480, 640, 3) and rgb.dtype == np.uint8

batch = dart.WorldBatch(world, count=64)
policy = lambda q: np.zeros_like(q)
batch.set("position_targets", policy(batch.positions))  # shape (64, num_dofs)
batch.step(20)
```

## 7.0 scope

### Must-have

7.0 needs everything that the
[acceptance criteria](#final-deliverable-and-acceptance) and the
[design doc](../../design/dart7_python_api.md) require, plus the work those
requirements wait for; this list summarizes it. Every must-have, breaking or
additive, merges before the API freeze `F`; see the
[release rules](#release-train-and-checkpoints).

- **Every breaking Python change:** the curated API designed area by area
  under the approved naming rulebook (the `Multibody`/`Link`/`Dof` vocabulary,
  `Collider`/`Visual` handles and named collections), the package layout with
  its curated top level,
  read-only arrays, error types, `UPPER_CASE` enums, keyword-only arguments,
  `Transform`/`Rotation`, raising loaders, the breaking parts of otherwise
  additive work, such as `step` rejecting `bool` arguments, and the DART
  detector as the default of dartpy's `dart.World` and the MJCF world loader
  ([M1f](#default-detector-switch)).
- Type stubs (`.pyi` plus `py.typed`) in every wheel with stub-drift checks,
  and the `dartpy` logging bridge.
- `io.load_model`/`io.load_world`, `World.add`, the `add_*` builders with
  keyword-only specs, and the `dart.data` sample data.
- Editing vs compiled simulation modes, where `compile()` freezes topology with
  zero steady-state allocation, and GIL-free `step(n)` on the deterministic
  executor.
- The typed per-World solver policy (M1e), which the constraint module's 7.0
  bindings wait for; the written `World` and solver contract; and the rigid
  step code behind an internal domain boundary with no behavior change.
- `WorldBatch` built from remapping clones.
- `capture_state`/`restore_state` dynamic snapshots and exact checkpoints:
  record/replay levels L0 and L1. Exact checkpoints restore into the same
  World for the DART detector (and ODE if it qualifies) with the Dantzig or
  PGS solver; FCL and Bullet raise.
- Compliant `PositionServo`/`VelocityServo` actuators, `StepHook` step hooks
  and native sensors.
- `dart.render`, `dart.viewer` and `dart.Material` on Filament, with the
  interim camelCase GUI module deleted. M6a merged the backend in #3696; M6b
  owns sensor rendering and the Python API.
- The codemod and migration guide, generated from the old-to-new mapping.
- A generated Python API reference built in CI from the stubs and docstrings,
  with a docstring coverage floor.
- One `cp312-abi3` wheel per platform (Linux x86_64/aarch64, macOS arm64 and
  Windows x64) that imports without graphics libraries.

### Deferrals

- M3 authoritative contiguous state, live views and compiled hooks are 7.x by
  default and must be additive if they land after 7.0; C1 decides whether M3
  enters 7.0.
- Record/replay L2/L3 (the recorder and replay) are optional for 7.0.
- GPU execution is outside 7.0; the C++ vocabulary rename follows 7.0 (M8).
- The C++ `World` keeps FCL as its default detector in 7.0; switching it to
  DART is a labeled 7.x change (see the
  [default-detector switch](#default-detector-switch)).
- Items allowed to slip to 7.1: notebooks, the example runner and the weekly
  Gazebo main lane (the gz-physics development branch).
- Sharing one executor across independent Worlds moves after 7.0;
  `WorldBatch` owns its executor.
- M2's dense articulated-body kernels are optional for 7.0 unless a
  milestone-exit run regresses against `dart7-base`; the
  [M2 stop rule](#release-train-and-checkpoints) can end them early.
- The coupling seam and a first experimental domain (rods) ship in
  `dartpy.experimental` in 7.0 if ready before `F`, and otherwise in 7.1.

### Priority and thresholds

The Python API leads (D3, amended 2026-10-19). Four chains run in parallel:
M5's package, arrays, errors and `Transform` packets, then the naming
rulebook, the facade slice and the rulebook-driven API areas with the codemod,
stubs and API reference; compile
and edit (M1b), counted stepping (M1d), the remapping clone (M1i) and
`WorldBatch`, then actuators, sensors and checkpoints; the typed solver policy
with its per-solver parameters (M1e), the written contract and the rigid domain
boundary; and M6b through checkpoint C2. Native work is ordered by the
acceptance tests it unblocks, and each native feature ships its Python
binding. The Python and MJCF default-detector switch (M1f) lands after the
capability matrix and M1b's strict shape rejection, and before `F`.

M3's C1 gate requires M2 to cut instruction counts (`Ir`) on the `dyn`/`robot`
rows by at least 15% and improve K = 64 wall time. The flagged contiguous-state
spike must pass the guards and the shim, with `memcpy` snapshots costing at
most 0.1× the gather path. The M2 stop rule is less than 10% `Ir` gain from the
first three kernel PRs; [C1](#release-train-and-checkpoints) applies it before
deciding M3.

## Non-goals

- GPU execution. The design keeps a path open, but no GPU code ships in 7.0.
- New physics domains in the stable API. A first experimental domain (rods) may
  ship in `dartpy.experimental` (see [Deferrals](#deferrals)); volumetric FEM
  stays out of scope, and there is no public plugin ABI until a second domain
  exists.
- An entity-component system or third-party data-layout library in the core.
- Python compatibility shims for 6.x names.
- Removing deprecated C++ APIs during 7.x.
- Third-party native extensions on wheel-installed DART.

## Constraints

- C++17, with zero new core dependencies.
- The gz-used C++ subset stays source- and behavior-compatible. Gazebo gates run
  for every core change.
- Refactor PRs are bit-identical per detector at one and several threads, with
  no new steady-state allocations. Numeric changes are separate, labeled PRs.
- Step-path merge order: changes to World stepping, the constraint solver,
  collision or generic-joint dynamics land in an order this program owns,
  each re-qualified on current `main`. Plans cite symbols, with line numbers
  only as of a named revision. (This replaced the quiet window that ran from
  the M1a PR until M1d's executor PR merged.)
- **Behavior evidence.** Bit-identical refactors are not behavior-bearing; the
  bit-identical gate is their evidence. Behavior-bearing changes (labeled
  numeric changes, actuators, sensors, snapshots) use `dart-verify-sim` with a
  text oracle and an assessed Filament capture. M6a landed in #3696; renderer
  availability no longer justifies a program-wide exception to GUI evidence.
- **Evidence hygiene.** Every empirical claim records its revision, command,
  date and scope. Counts from earlier research stay "reported" until a script in
  this repository reproduces them. Historical timings are not release evidence.
- **Hot files:**
  - each PR adds its changelog entry as a `changelog.d/` fragment once
    [#3874](https://github.com/dartsim/dart/pull/3874) lands; an open PR
    converts its `CHANGELOG.md` hunk into a fragment during its next base
    merge;
  - `pixi.lock` is regenerated by the merging PR, never hand-merged;
  - M5's package-layout PR retains M6a's wheel notices and checks before M6b
    wheel acceptance;
  - M6a delivered the GUI CI variants and the GUI-backend flip in the branch
    profile in #3696.
- Pushes, PRs, CI reruns and other GitHub mutations need explicit maintainer
  approval each time.

## Milestones

| ID | Milestone | Status | Notes |
| --- | --- | --- | --- |
| M0 | Release-line rollover, project home, baselines, safety nets | Done | Exited 2026-10-14; [packets](#m0-packets) |
| M1 | Core seams | Active | M1a merged in [#3775](https://github.com/dartsim/dart/pull/3775); M1b's World-local preparation and M1c's snapshots merged, and the next packets are tracked in [RESUME](RESUME.md); remaining [sub-milestones](#m1-core-seams) are open |
| M2 | Dense articulated-body data | Optional for 7.0 | Needed only if a milestone-exit run regresses; [stop rule](#release-train-and-checkpoints) |
| M3 | Authoritative contiguous state | 7.x by default | Go/no-go at checkpoint C1 |
| M4 | Recorder and replay | Optional for 7.0 | Snapshots and checkpoints are required |
| M5 | dartpy 7.0 API | Active | [Order](#m5-dartpy-70-api); designed first under the naming rulebook |
| M6a | Filament backend port | Complete | Merged in #3696; wheels qualified; its remaining release gates moved to [C2](#release-train-and-checkpoints) ([interface](#m6a-interface)) |
| M6b | Sensor rendering and Python render/viewer API | Planned | Checkpoint C2 gates the release; the D31b [demo consolidation](#demo-consolidation-d31b) is complete locally |
| M7 | Release 7.0 (release candidates first) | Planned | Waits for M6b and checkpoint C2 |
| M8 | C++ vocabulary rename | After 7.0 | Aliases and forwarders |

Remaining critical path, in four chains that run in parallel (Python first):
the snapshot, arrays, errors and `Transform` packets still in review (the
package layout merged in #3847), then the naming rulebook, the facade slice and the rulebook-driven API areas, codemod, stubs
and reference; compile and edit
(M1b), counted stepping (M1d), the remapping clone (M1i) and `WorldBatch`; the
typed solver policy with its per-solver parameters (M1e), the written contract
and the rigid domain boundary; and M6b → checkpoint C2.

### M1 core seams

| ID | Content | Notes |
| --- | --- | --- |
| M1a | `World::step` split into internal stages: private, non-virtual stage member functions and a forward-declared private `StepContext`, with no data member, vtable or layout change | One PR with one commit per stage; moved lines stay verbatim and the gates are bit-identical. `getConstraintSolver()` is unchanged. Opened the step-path quiet window, since replaced by the program-owned merge order |
| M1b | `compile()`, `edit()`, the strict flag, per-World revisions and strict tiers, including strict rejection of unsupported shapes when a World compiles | Mutation classes in the design doc. M1f waits for the shape rejection |
| M1c | Snapshots, checkpoints, and the field and frame oracle | One PR per hidden-state gap; moving the solver's random-number state is a labeled numeric change |
| M1d | One deterministic executor, GIL release, `step(n)` | The 64-world row must show scaling from 1 to 8 threads before batch bindings start |
| M1e | Typed per-World solver policy | Per-World solver parameters are opt-in; the process-wide statistics keep last-write-wins |
| M1f | DART detector as the default of dartpy's `dart.World` and `dart.io.load_world` for MJCF | A breaking Python default change, so a must-have before `F`. Lands after the published capability matrix and M1b's strict shape rejection. The C++ `World`, `dart::utils::MjcfParser::readWorld` and its direct dartpy binding keep FCL; see the [switch](#default-detector-switch) |
| M1g | Actuator channels, compliant motor rows, `StepHook` | |
| M1h | Native sensors, without recording | |
| M1i | Remapping clone | `WorldBatch` prerequisite |

### M2 dense articulated-body data

M2 PRs come in this order: differential tests, dense indexing with the old
arithmetic, one joint family per PR, storage authority, then labeled numeric
changes. The numerical rules they follow live in the design doc's
"Determinism and the executor" section.

### M5 dartpy 7.0 API

M5a is open since M0 exited. The API is designed first: the maintainer
approves a naming rulebook once, each API area lands as a normal PR showing
the designed API, and the old-to-new mapping generated from those PRs drives
the codemod, the stubs, the generated docstrings and the migration guide. An
API-surface ratchet keeps the parity surface from growing meanwhile. In this
order:

1. Mechanism packets, in review or merged: world-state snapshots; the package
   layout (private extension, pure-Python helpers, scikit-build-core, one
   stable-ABI wheel for Python 3.12+, and the 7.0 changelog's Python-support
   entries); read-only array copies; errors and logging; and `Transform` and
   `Rotation`. They can precede the rulebook because approved decisions fix
   their central public names: the `dartpy` package (D4, D6),
   `dartpy.experimental` (D22), `capture_state` and `restore_state` (the design doc's `World` surface, under D11), the
   `dartpy` logger and the error classes the design doc names (D22), and
   `Transform` and `Rotation` (D28) with their named quaternion orders (D29).
   The rulebook adopts those names. Every other public name these packets add,
   including `WorldState`, the error base class and the transform constructors
   that the design doc leaves to their packets, is provisional and follows the
   rulebook once it is approved.
2. The naming rulebook, which also decides the curated top level.
3. A vertical slice of the facade: `World` keyword construction, `Multibody`,
   `Link`, `Joint` and `Dof` with named collections, the builders, `world.add`
   and `Collider`/`Visual` handles.
4. The remaining API areas: math, dynamics, collision, io, common and
   optimizer, each designed under the rulebook.

Defects found on the way (missing `None` checks, dropped out-parameters,
unregistered types) ship as separate PRs.

M5b binds each M1 feature when it merges, directly in the 7.0 style; names that
no decision fixes stay provisional until the rulebook is approved. The
constraint module's 7.0 bindings start after M1e and the simulation module's
after M1d; their renames are breaking, so both land before `F`. M5 does not
rename bindings that later milestones replace: recording (M4), aspect state
(M3) and the interim GUI module (M6).

Tooling added on the way: `pyperf`, `libcst` and `pyright` in pixi; `python/`
no longer excluded from the Python formatters; a `--renames` mode for
`api_surface.py`.

## Release train and checkpoints

WP-7.0.7 records the API freeze `F` and two checkpoints. `F` is declared once
every [must-have](#must-have), breaking and additive, has merged and C2 has
passed. Only additive work that is not a must-have and misses `F` moves to 7.1.

- **C1: M3 go/no-go after the first three M2 kernel PRs**, applying the M2
  stop rule below. M3 goes into 7.0 only if all of these hold:
  1. guards are unchanged apart from approved labeled PRs, and the Gazebo gates
     stayed green through the first three M2 kernel PRs;
  2. M2 cut instruction counts on the dynamics and robot rows by at least 15%
     and improved 64-world wall time (thresholds confirmed in M0);
  3. a flagged spike limited to storing revolute and free-joint
     positions and velocities contiguously, passes the guards and the shim test,
     and its memory-copy snapshots cost at most 0.1× of the gather path;
  4. M3 can land before `F` without delaying any other must-have, and so
     without delaying `F`.

  Otherwise M3 moves to 7.x and must be purely additive when it lands.
  Aspect-state semantics change only in 8.0.
- **C2: M6 complete before `F`**, so the release candidates carry the final
  render API. The gate:
  - headless RGB, depth and segmentation from the wheel in a fresh environment,
    in the render environments defined under [M6a interface](#m6a-interface);
  - native Windows and macOS CI green;
  - Filament performance re-measured on a quiet Linux host, on a GPU and on
    Lavapipe: completed frames, setup time, full-RGBA capture, RSS and GPU
    memory, judged against a maintainer-approved fast-profile disposition
    (D32). No renderer baseline is required, and the maintainer deferred
    Windows and macOS performance. Record the host state, and never stop
    other work to quiet the host;
  - two clean independent reviews of the renderer's release state, covering
    the fixes that wheel publication and Nightly required after #3696, such
    as #3784.

  There is no render-less fallback. Escalate early if C2 is at risk.

Release rules:

- Every must-have lands before `F`, including every breaking change: renames,
  package layout, read-only arrays, error types, enums, keyword-only options,
  `Transform`/`Rotation`, raising loaders, the breaking parts of otherwise
  additive work such as `step` rejecting `bool` arguments, and the DART
  detector as the Python and MJCF default. Only additive work that is not a
  must-have and misses `F` moves to 7.1.
- 7.0 requires snapshots and checkpoints. The recorder (M4) is optional.
- Items allowed to slip to 7.1: notebooks, the example runner and the
  scheduled lane against the gz-physics development branch. The generated API
  reference is a 7.0 must-have.
- While 7.0 waits for M6b and the remaining rendering gates, only fixes and
  M6b land on the release path. M6b includes the D31b demo host, the Python
  demo catalog that runs on it, and the core bindings its scenes need.
  `pixi run demos`, `pixi run cycle-demos` and `pixi run capture` launch the
  Python catalog (`python -m examples.demos`) against the built dartpy.
- Release candidates publish `7.0.0rcN` to PyPI. The C++ version stays numeric;
  `DARTPY_VERSION_SUFFIX` carries the candidate tag.
- The final release needs release-candidate feedback, the codemod run on at
  least three external codebases, and fresh-environment acceptance.
- After 7.0, Python-only fixes ship as patch releases and features as planned
  minor releases. Each 7.x minor release changes the C++ SONAME (MAJOR.MINOR)
  and is announced to downstream packagers.
- **M2 stop rule:** if the first three kernel PRs cut instruction counts on the
  dynamics and robot rows by less than 10%, M2 stops and M3 defaults to no-go.

## Default-detector switch

The native detector differs from FCL in known ways: it targets four contacts
per pair, keeps persistent manifolds, approximates ellipsoids and cones with
meshes, and warns once and then skips heightmaps.

D13, as amended on 2026-10-12, splits the switch:

- **Python, including MJCF loading, in 7.0 (M1f).** dartpy's `dart.World` and
  `dart.io.load_world` for MJCF default to the DART detector. The MJCF switch
  changes only `dart.io.load_world`, never the C++
  `dart::utils::MjcfParser::readWorld`. The dartpy `MjcfParser` binding
  exposes that C++ parser directly; changing it would also switch C++
  callers from FCL. The parser and its direct binding keep FCL. This
  breaking Python default change is a must-have that lands before `F`, and
  only after:
  - a published shape and query capability matrix;
  - strict rejection, when a World compiles (M1b), of shapes the native
    detector cannot handle.
- **C++, in 7.x.** The C++ `World` keeps FCL as its default in 7.0, for Gazebo
  compatibility. Switching it to DART is a labeled 7.x change that also needs:
  - representative behavior comparisons against FCL;
  - PLAN-622's pre-default coverage gate.

Mechanics of the C++ switch:

- The switch is one PR.
- Both `ConstraintSolver` constructors call `DARTCollisionDetector::create()`
  directly, never through the collision-detector factory, because downstream
  plugins can unregister every factory creator.
- The `WorldConfig` default and the skel-parser fallback change in the same PR,
  following the [collision-backend design](../../design/dart6_collision_backends.md).
- FCL remains a dependency.

## M6a interface

M6a merged in [#3696](https://github.com/dartsim/dart/pull/3696) on 2026-10-11
(merge `b63cc07f6`). Main now uses Filament. The merged change delivers:

- the renderer port, on a fresh branch from current main;
- `dart-gui` links the core library, with IO optional, and keeps main's option
  defaults and legacy components;
- the manylinux image's OSG removal and pinned Filament SDK build during the
  wheel build;
- Filament licenses in the wheels; the current wheel staging ships them, and
  M5's package-layout PR carries them over;
- the `CI Filament` functional lanes on Linux, Windows and macOS, all green
  on the merged PR;
- the GUI-backend flip in `docs/ai/branch-profile.json`.

M6a closed on 2026-10-18. Its task folder is retired, and the
[closeout evidence](verification.md#m6a-closeout) is in the program record.
The Nightly `dartpy-wheels` rows build, repair, verify and test all eight
Filament wheels: Linux x86_64 and aarch64 for CPython 3.10, 3.11 and the 3.12
stable ABI, plus the Windows and macOS stable-ABI wheels. The stable-ABI
wheels also pass `abi3audit`, and both Linux stable-ABI rows pass the
slim-container headless import. The slowest row finishes in under an hour of
its 120-minute limit, so the image keeps building the pinned SDK during the
wheel build; the separately cached SDK stage that M6a planned is unnecessary.
M6a's two open release gates, quiet-host renderer performance and the final
renderer reviews, are part of [C2](#release-train-and-checkpoints).

Follow-ups M6a left open, none of them a release blocker:

- The native app host opens a window with an explicit width or height at the
  screen origin instead of centering it (`App::run`).
- The installed `dart/gui/Viewer.hpp` exports the test-only
  `detail::readViewerPixelsRGBA` and `detail::captureViewerPixelsRGBA` hooks.
- In C++, a viewer's own event handler must not destroy it; in Python, the
  call keeps the viewer alive until `frame()` or `run()` returns.
- Configuration checks that `matc` and the Filament runtime agree on the
  material version, but it does not enforce the 1.77.3 SDK minimum.
- On Windows, the static-CRT Filament library selection
  (`DART_RUNTIME_LIBRARY=/MT`) and the Pixi `test-gui` task override are
  unqualified: wheel builds use `/MD`, and `CI Filament` runs CTest and pytest
  directly.

**Interim bindings (D31a).** M6a landed the effort's tested camelCase GUI
module as the interim `dartpy.gui`. Main may mix styles until M6b; the
7.0 release may not. M6b builds `dart.render` and `dart.viewer` in the 7.0
style, ports the tutorials, docs and tests, and deletes the interim module
before the API freeze `F`, so release candidates carry the final render API.
The target-shape render test in `python/tests/unit/gui/` already uses the 7.0
API, so it stays when the interim tests are ported.
New viewer bindings land in the 7.0 style as `dartpy.viewer` (D31b). Demo
scenes may use interim `dartpy.gui` primitives until M6b ports them.

### Demo consolidation (D31b)

WP-PD.0–WP-PD.18 deliver one Python catalog on the private native app host:
all 37 retained scenes are ported, the Pixi launch tasks use that catalog,
and the C++ application, aliases, and transitional host features are removed.
The demo project home is retired. Durable facts now live in
[architecture](../../onboarding/architecture.md#python-demo-catalog),
[scene rules](../../../examples/README.md#adding-catalog-scenes),
[callback ownership](../../onboarding/python-bindings.md#viewer-app-callbacks),
and [catalog verification](../../onboarding/testing.md#demo-catalog-verification).
The [changelog drafts](changelog-drafts.md) become fragments after #3874 lands;
the packet verification remains in [the program record](verification.md).

This closes demo consolidation, not M6b's sensor or render/viewer API work.
The maintainer subsequently directed demo implementation into Python over the
engine, IO, and GUI bindings. Every native demo family was ported after
behavior comparisons with the native code, and `dartpy._demos` and
`dart-demo-models` are removed. The WAM scene uses ssik instead of IKFast,
which DART 7 drops (D33); the evidence is in
[the program record](verification.md#python-demo-implementation-evidence).
The Windows variants of the `demos`, `cycle-demos` and `capture` tasks have
not run on a Windows host; hosted CI runs the catalog through Python directly.
M6b still owns `dart.render`, the final viewer API, and migration of the
interim `dartpy.gui` primitives (InteractiveFrame, drag and drop, GridVisual,
SupportPolygonVisual, DebugOverlay, and GUIEventHandler). Pre-existing host
gaps also remain: LogCapture misses Python output and is not
thread-safe, Copy does not reach the system clipboard, and `GuiScaleTheme`
duplicates Viewer scaling. A native host loop releases the GIL; until the
program's World-busy guard lands, other Python threads can still touch its
World.

**OSG removal:**

- `DART_BUILD_GUI_OSG` stays as a legacy switch. `OFF` forces and caches
  `DART_BUILD_GUI=OFF`, including inside an embedding project. An explicit
  `ON` fails with a migration message, while a cached default left by the old
  option is dropped so existing build trees reconfigure.
- A missing Filament SDK is a fatal configure error that names
  `DART_BUILD_GUI=OFF`.
- The `gui-osg` package component fails a required request with migration
  text; optional probes see it as not found. It is never forwarded to the
  Filament component.
- In-repository OSG consumers (examples, Python demos and tutorials, package
  tools, the wheel test script) were ported or retired in #3696.
- GUI binding sites adopted WP-7.0.4's interpreter-finalization rule in
  [#3713](https://github.com/dartsim/dart/pull/3713).
- The M5 codemod marks GUI code with a `TODO(dart7)` note to port it to
  `dart.viewer`.

**Headless import:**

- The extension and its bundled libraries never depend on libGL, libEGL,
  libOpenGL, libX11, libxcb, libwayland or libvulkan. The wheel verifier checks
  this.
- `Publish dartpy` runs `scripts/check_headless_wheel.py` for Linux stable-ABI
  wheels; it imports dartpy and steps a World in a slim Python 3.12 container.
- M6a raises a typed backend error that names the missing system packages, and
  M6b's `dart.render` maps it to its own typed error.
- Render acceptance environments: on Linux, headless capture uses Vulkan (a
  GPU driver or Mesa's software rasterizer) and interactive windows use OpenGL
  over GLX; on Windows, Mesa's software rasterizer; on macOS, Metal in the
  `macos-15` functional CI lane.

## M0 packets

| ID | Objective | Status |
| --- | --- | --- |
| WP-7.0.1 | Roll main's release line and AI policy from DART 6 to 7.0 | Done ([#3697](https://github.com/dartsim/dart/pull/3697)) |
| WP-7.0.1b | Reconcile main's stale 6.20.0 changelog section with the released one | Done |
| WP-7.0.2 | Create this project home, the design doc and the PLAN-624 dashboard entry | Done ([#3697](https://github.com/dartsim/dart/pull/3697)) |
| WP-7.0.3 | Make the fixed-size XML value helpers reject wrong element counts instead of writing out of bounds | Done ([#3698](https://github.com/dartsim/dart/pull/3698)); 6.20 backport merged ([#3717](https://github.com/dartsim/dart/pull/3717)) |
| WP-7.0.3b | Honor or reject SDF pose attributes (`rotation_format`, `degrees`, `relative_to`) instead of ignoring them | Done: [#3719](https://github.com/dartsim/dart/pull/3719) and the [four-gap follow-up #3782](https://github.com/dartsim/dart/pull/3782) merged ([verification](verification.md#wp-703b-follow-up-pose-consumers-and-version-admission)) |
| WP-7.0.4 | Make dartpy's GIL acquisitions safe during interpreter finalization | Done ([#3699](https://github.com/dartsim/dart/pull/3699)); the GUI binding sites followed in [#3713](https://github.com/dartsim/dart/pull/3713) |
| WP-7.0.5 | Pin the pre-refactor performance baseline and add many-World and Python-loop rows | Baseline complete: `dart7-base` at `c3134f51b86d`, 78 passing rows; [evidence](verification.md#m1a-world-step-stages) |
| WP-7.0.6 | Add the gz compatibility shim test, the contact-surface header probe, version plumbing and inventories | Done: [#3725](https://github.com/dartsim/dart/pull/3725), [#3741](https://github.com/dartsim/dart/pull/3741) (replacing the closed [#3731](https://github.com/dartsim/dart/pull/3731)), [#3729](https://github.com/dartsim/dart/pull/3729) and its dependency fix [#3787](https://github.com/dartsim/dart/pull/3787) |
| WP-7.0.7 | Record the API freeze, checkpoints, must-have list and acceptance tests | Done ([#3728](https://github.com/dartsim/dart/pull/3728)) |
| WP-7.0.8 | Add newer-Python wheels to the 6.20.x maintenance line | In progress: [#3704](https://github.com/dartsim/dart/pull/3704) added 6.20.1 wheels for Python 3.12 to 3.14 and [#3730](https://github.com/dartsim/dart/pull/3730) Linux Python 3.15 wheels; macOS and Windows Python 3.15 wheels wait for conda-forge's Python 3.15 builds of `bullet-cpp` |

Each packet records its scope, non-goals, gates and evidence in
[verification.md](verification.md) before it is marked done. M0 exited on
2026-10-14, when #3728 (WP-7.0.7) and #3729 (the gz API inventory) merged.
The capability matrix (#3788) followed. M1 and M5a are in progress;
[RESUME.md](RESUME.md) tracks the packets.

## Gates

- **Every PR:**
  - `pixi run lint`, then `pixi run check-lint`, which includes the AI command
    and infrastructure checks;
  - `pixi run build`, `pixi run test` and `pixi run test-py` when code changes;
  - `scripts/check_local_paths.py` on public text: files, commit messages and
    PR text.
- **AI-policy changes:** `pixi run test-ai-infra` and
  `pixi run exercise-agent-scenarios`.
- **Core changes:**
  - `pixi run python scripts/perf_regression.py local --base origin/main --head HEAD --rows nightly`
    on the named performance host;
  - the step-allocation ratchet;
  - `pixi run -e gazebo test-gz`;
  - the gz shim test and the contact-surface header probe, which
    `pixi run test` runs (added in #3725);
  - the many-World rows (added in #3720);
  - the snapshot and checkpoint oracle (arrives with M1c);
  - for every PR that changes step, solver, collision, packaging or
    Gazebo-lock files, `pixi run -e gazebo test-gz` plus the unpatched Ionic
    and Jetty lanes (`pixi run gz-compat-ionic` and `gz-compat-jetty`)
    against a base variant (`GZ_COMPAT_BASE_VARIANT`) with no `NEW` or `SLOW`
    results, and the sleep oracle (`pixi run gz-compat-ionic sleep-oracle`) on
    head and base. Building `dart-io` against each distribution's sdformat in
    those lanes is a pending lane change: the Ionic lane's DART still uses
    sdformat 16.
- **Milestone exits:** the core gates with `--base dart7-base`.
- **Python changes:**
  - stub-drift, new-`Any` and pyright checks (arrive with M5's stub PRs);
  - the rename-aware `api_surface.py` diff;
  - the Python-loop benchmark, `pixi run perf-dartpy-loop` (added in #3720);
  - the example runner in test mode, once it exists;
  - the codemod round trip on non-GUI tutorial code.
- **GUI changes:** `pixi run test-gui`, delivered in #3696; `CI Filament`
  on three operating systems. Quiet-host Filament measurements on GPU and
  Lavapipe require an approved fast-profile disposition (decision D32).
- **Gazebo gates:**
  - The shim test is generated from the gz-physics development branch at a
    recorded revision and refreshed on a schedule. It covers the
    collision-detector overrides (ray casts and `createCollisionGroup`), the
    detector registrar, the direct Bullet header include, and that
    unregistering every factory creator still leaves `World::create()` on the
    selected default detector (FCL, which stays the C++ default in 7.0). It
    asserts that each override runs.
  - A CTest compiles `#include <dart/constraint/ContactSurface.hpp>` against the
    installed DART using only DART and Eigen include paths. Unix uses C++14
    with existing compiler extensions; MSVC uses DART's C++17 dialect and
    exported template-parsing options, as fixed in #3741. Downstream feature
    detection depends on this, so new headers stay out of that include tree.
  - Acceptance: the compatibility lanes pass their comparisons with no new
    failures or slow cases against baselines from the last 6.20.x release, and
    each delta is annotated. The development-branch lane runs weekly and does
    not block.
- **7.0 acceptance:**
  - the target-shape tests with no `xfail` markers left, from release-candidate
    wheels in fresh environments on all four platforms;
  - the slim-container import test;
  - the Gazebo lanes;
  - performance rows equal or better against `dart7-base`, each detector
    against its own baseline, with the contact-benchmark rows compared
    against `812310ffb` because #3789 changed their workload;
  - the codemod on at least three external codebases.

Every packet needs two clean review passes on its final state.

## Risks

- **Scope.** The program spans about 90–130 PRs. Mitigations: Python-first
  priority with native work ordered by the acceptance tests it unblocks, one
  PR per change, the M2 stop rule, and M3 is 7.x by default.
- **The release waits for the renderer.** Mitigation: escalate early if
  checkpoint C2 is at risk.
- **Downstream drift.** Mitigation: the shim test runs against the current
  gz-physics development branch, plus a scheduled lane.
- **Concurrent work on core files.** Mitigation: the program-owned step-path
  merge order.

## Blockers and next actions

- WP-7.0.8's macOS and Windows Python 3.15 wheels wait for conda-forge's
  Python 3.15 builds of `bullet-cpp`.
- The "DART 7.0.0" milestone (#105, formerly "DART 6.21.0") tracks the release,
  and main's release target reads 7.0.0.
- Publishing each packet needs maintainer approval.
- Pending maintainer action: check PyPI's project history for deleted 7.0.0
  files; PyPI never accepts a reused file name.
- The maintainer decided the program's open questions on 2026-10-19; see
  [Re-prioritization](decisions.md#re-prioritization-2026-10-19). Questions
  specific to later packets (servos and actuators, native sensors, the solver
  policy) are settled when each packet starts.
- Pending: the C2 fast-profile criteria, settled with M6b.
- See [RESUME.md](RESUME.md) for the immediate next steps.
