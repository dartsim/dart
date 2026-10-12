# DART 7.0 Python-first program

This project home tracks the program that turns the next DART release into
**DART 7.0**: a Python-first library whose core keeps the existing solver as the
baseline and becomes ready for multi-physics, batched and replayable simulation.
[PLAN-624](../../plans/dashboard.md#plan-624-dart-70-python-first-program)
tracks its roadmap status. The durable API and engine rules live in the
[DART 7 Python API design](../../design/dart7_python_api.md).

- [Decisions](decisions.md): the maintainer decisions D0–D32 and D31a, with
  their rationale.
- [Verification](verification.md): evidence, review passes and gates per
  packet.
- [Progress log](progress-log.md): completed chunks.
- [Resume](RESUME.md): the next-session handoff.

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
- **Unmodified gz-physics and gz-sim pass** the Harmonic, Ionic and Jetty
  compatibility lanes, with no new failures or slow cases against baselines from
  the last 6.20.x release.
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
Each feature PR removes its marker, so the tests double as a burn-down. The
tests use existing constructors for independent setup until the package rename
lands. The contract stays here instead of in `python/examples/target_shape.py`,
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
requirements wait for; this list summarizes it. How the API freeze `F` treats
the additive must-haves awaits a maintainer decision; see the
[release rules](#release-train-and-checkpoints).

- **Every breaking Python change lands before `F`:** native renames from the
  reviewed rename table (the `Multibody`/`Link`/`Dof` vocabulary,
  `Collider`/`Visual` handles and named collections), the package layout with
  its curated top level, read-only arrays, error types, `UPPER_CASE` enums,
  keyword-only arguments, `Transform`/`Rotation`, raising loaders, and the
  breaking parts of otherwise additive work, such as `step` rejecting `bool`
  arguments. The default-detector switch ships in 7.0 only if its
  [prerequisites](#default-detector-switch) pass before M1 starts; otherwise it
  is a labeled 7.x change.
- Type stubs (`.pyi` plus `py.typed`) in every wheel with stub-drift checks,
  and the `dartpy` logging bridge.
- `io.load_model`/`io.load_world`, `World.add`, the `add_*` builders with
  keyword-only specs, and the `dart.data` sample data.
- Editing vs compiled simulation modes, where `compile()`, or a lazy world's
  first step, prepares the dense articulated-body data (M2), and GIL-free
  `step(n)` on the deterministic executor. The
  [M2 stop rule](#release-train-and-checkpoints) can end M2's kernel PRs early.
- Typed solver selection in `WorldConfig` (M1e), which the constraint module's
  7.0 bindings wait for.
- `WorldBatch` built from remapping clones.
- `capture_state`/`restore_state` dynamic snapshots and exact checkpoints:
  record/replay levels L0 and L1.
- Compliant `PositionServo`/`VelocityServo` actuators, `StepHook` step hooks
  and native sensors.
- `dart.render`, `dart.viewer` and `dart.Material` on Filament, with the
  interim camelCase GUI module deleted. M6a merged the backend in #3696; M6b
  owns sensor rendering and the Python API.
- The codemod and migration guide.
- One `cp312-abi3` wheel per platform (Linux x86_64/aarch64, macOS arm64 and
  Windows x64) that imports without graphics libraries.

### Deferrals

- M3 authoritative contiguous state, live views and compiled hooks are 7.x by
  default and must be additive if they land after 7.0; C1 decides whether M3
  enters 7.0.
- Record/replay L2/L3 (the recorder and replay) are optional for 7.0.
- GPU execution is outside 7.0; the C++ vocabulary rename follows 7.0 (M8).
- Items allowed to slip to 7.1: autoapi docs, notebooks, the example runner and
  the weekly Gazebo main lane (the gz-physics development branch).

### Priority and thresholds

The critical path starts with the baseline pin, then M1 stage extraction
(M1a), the executor (M1d) and remapping clone (M1i), then `WorldBatch`. M5 and
M6b run in parallel with that path. The gated default-detector switch (M1f)
comes before the other M1 work, so later rebaselines start from the final
default.

M3's C1 gate requires M2 to cut instruction counts (`Ir`) on the `dyn`/`robot`
rows by at least 15% and improve K = 64 wall time. The flagged contiguous-state
spike must pass the guards and the shim, with `memcpy` snapshots costing at
most 0.1× the gather path. The M2 stop rule is less than 10% `Ir` gain from the
first three kernel PRs; [C1](#release-train-and-checkpoints) applies it before
deciding M3.

## Non-goals

- GPU execution. The design keeps a path open, but no GPU code ships in 7.0.
- New physics domains. Volumetric FEM stays out of scope, and there is no public
  plugin ABI until a second domain exists.
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
- Step-path quiet window: from the first M1a PR until the M1d executor PR
  merges, changes to World stepping, the constraint solver, the native
  collision detector or generic-joint dynamics go through this program. Plans
  cite symbols, with line numbers only as of a named revision.
- **Behavior evidence.** Bit-identical refactors are not behavior-bearing; the
  bit-identical gate is their evidence. Behavior-bearing changes (labeled
  numeric changes, actuators, sensors, snapshots) use `dart-verify-sim` with a
  text oracle and an assessed Filament capture. M6a landed in #3696; renderer
  availability no longer justifies a program-wide exception to GUI evidence.
- **Evidence hygiene.** Every empirical claim records its revision, command,
  date and scope. Counts from earlier research stay "reported" until a script in
  this repository reproduces them. Historical timings are not release evidence.
- **Hot files:**
  - program feature PRs defer their CHANGELOG entries to one consolidation PR
    per milestone exit; standalone bug fixes keep their own entries;
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
| M0 | Release-line rollover, project home, baselines, safety nets | Active | [Packets](#m0-packets) |
| M1 | Core seams | Planned | [Sub-milestones](#m1-core-seams); bit-identical unless labeled |
| M2 | Dense articulated-body data | Planned | [Stop rule](#release-train-and-checkpoints) |
| M3 | Authoritative contiguous state | 7.x by default | Go/no-go at checkpoint C1 |
| M4 | Recorder and replay | Optional for 7.0 | Snapshots and checkpoints are required |
| M5 | dartpy 7.0 API | Planned | [Order](#m5-dartpy-70-api); the rename table needs maintainer approval first |
| M6a | Filament backend port | Merged in #3696 | [Interface](#m6a-interface); remaining release gates below |
| M6b | Sensor rendering and Python render/viewer API | Planned | Checkpoint C2 gates the release |
| M7 | Release 7.0 (release candidates first) | Planned | Waits for M6b and the remaining rendering release gates |
| M8 | C++ vocabulary rename | After 7.0 | Aliases and forwarders |

Critical path: WP-7.0.1 rollover → baseline pin → (M1f) → M1a → M1d → M1i →
`WorldBatch` → M5b simulation bindings → acceptance. In parallel: M6a → M6b →
checkpoint C2. M5a starts when M0 exits.

### M1 core seams

| ID | Content | Notes |
| --- | --- | --- |
| M1f | Default collision-detector switch | Ships in 7.0 only if its [gate](#default-detector-switch) passes before M1 starts; otherwise a labeled 7.x change. Scheduled first, so later rebaselines start from the final default |
| M1a | `World::step` split into internal stages (member objects or functions, no pimpl) | `getConstraintSolver()` is unchanged. Opens the quiet window |
| M1b | `compile()`, `edit()`, the strict flag, per-World revisions and strict tiers | Mutation classes in the design doc |
| M1c | Snapshots, checkpoints, and the field and frame oracle | One PR per hidden-state gap; moving the solver's random-number state is a labeled numeric change |
| M1d | One deterministic executor, GIL release, `step(n)` | Closes the quiet window. The 64-world row must show scaling from 1 to 8 threads before batch bindings start |
| M1e | Typed solver selection in `WorldConfig` | Per-World solver parameters are opt-in; the process-wide statistics keep last-write-wins |
| M1g | Actuator channels, compliant motor rows, `StepHook` | |
| M1h | Native sensors, without recording | |
| M1i | Remapping clone | `WorldBatch` prerequisite |

### M2 dense articulated-body data

M2 PRs come in this order: differential tests, dense indexing with the old
arithmetic, one joint family per PR, storage authority, then labeled numeric
changes. The numerical rules they follow live in the design doc's
"Determinism and the executor" section.

### M5 dartpy 7.0 API

M5a starts when M0 exits, in this order:

1. The rename table v0, generated by `scripts/nanobind/api_surface.py
   --snapshot` and approved by the maintainer in one PR. It gates everything
   else.
2. The package layout: private extension, pure-Python helpers, scikit-build-core,
   one stable-ABI wheel for Python 3.12+. It also updates the 7.0 changelog's
   Python-support entries.
3. The array casters: read-only copies.
4. Errors and logging.
5. `Transform` and `Rotation`.
6. Module renames: math, then dynamics (member renames, then the class
   vocabulary, then `Collider`/`Visual` and collections), then collision, io,
   common and optimizer.

Defects found on the way (missing `None` checks, dropped out-parameters,
unregistered types) ship as separate PRs.

M5b binds each M1 feature when it merges, directly in the 7.0 style. The
constraint module's 7.0 bindings start after M1e and the simulation module's
after M1d; their renames are breaking, so both land before `F`. M5 does not
rename bindings that later milestones replace: recording (M4), aspect state
(M3) and the interim GUI module (M6).

Tooling added on the way: `pyperf`, `libcst` and `pyright` in pixi; `python/`
no longer excluded from the Python formatters; a `--renames` mode for
`api_surface.py`.

## Release train and checkpoints

WP-7.0.7 records the API freeze `F` and two checkpoints. `F` is declared once
every breaking change has merged and C2 has passed, and additive work that
misses `F` moves to 7.1. The [must-haves](#must-have) also include additive
work, such as snapshots and checkpoints, `WorldBatch`, actuators and sensors.
**Pending maintainer decision**, needed before `F` is declared: either `F` also
waits for every must-have, or the additive must-haves are exempt from the move
to 7.1.

- **C1: M3 go/no-go after the first three M2 kernel PRs**, applying the M2
  stop rule below. M3 goes into 7.0 only if all of these hold:
  1. guards are unchanged apart from approved labeled PRs, and the Gazebo gates
     stayed green through the first three M2 kernel PRs;
  2. M2 cut instruction counts on the dynamics and robot rows by at least 15%
     and improved 64-world wall time (thresholds confirmed in M0);
  3. a flagged spike limited to storing revolute and free-joint
     positions and velocities contiguously, passes the guards and the shim test,
     and its memory-copy snapshots cost at most 0.1× of the gather path;
  4. M3 can land before `F` without delaying the remaining breaking changes.

  Otherwise M3 moves to 7.x and must be purely additive when it lands.
  Aspect-state semantics change only in 8.0.
- **C2: M6 complete before `F`**, so the release candidates carry the final
  render API. The gate:
  - headless RGB, depth and segmentation from the wheel in a fresh environment,
    in the render environments defined under [M6a interface](#m6a-interface);
  - native Windows and macOS CI green;
  - performance re-measured on a quiet host, with GPU and software rendering,
    and the fast-profile disposition approved.

  There is no render-less fallback. Escalate early if C2 is at risk.

Release rules:

- Every breaking change lands before `F`: renames, package layout, read-only
  arrays, error types, enums, keyword-only options, `Transform`/`Rotation`,
  raising loaders, the breaking parts of otherwise additive work such as `step`
  rejecting `bool` arguments, and the default-detector switch if its gate
  passes. Additive work that misses `F` moves to 7.1, subject to the pending
  decision above.
- 7.0 requires snapshots and checkpoints. The recorder (M4) is optional.
- Items allowed to slip to 7.1: the generated API reference, notebooks, the
  example runner, and the scheduled lane against the gz-physics development
  branch.
- While 7.0 waits for M6b and the remaining rendering gates, only fixes and
  M6b land on the release path.
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

The switch ships in 7.0 only if these prerequisites pass before M1 starts.
It is scheduled first, so later rebaselines start from the final default.
Otherwise the switch is a labeled 7.x change and FCL stays the 7.0 default.

Prerequisites:

- a published shape and query capability matrix;
- strict rejection, when a World compiles, of shapes the native detector
  cannot handle;
- representative behavior comparisons;
- PLAN-622's pre-default coverage gate.

Mechanics:

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

Merged evidence on 2026-10-11: the Linux x86_64 stable-ABI wheel builds,
repairs, passes `abi3audit`, and passes fresh cp312/cp313 container smoke tests.
The [merged handoff](../dart6_filament_renderer/RESUME.md) still identifies the
separately cached SDK stage and Windows/macOS wheel qualification as open;
aarch64 wheel qualification is not recorded in the merged evidence and also
remains a release gate. Quiet-host renderer performance is still unmeasured.
The headless-import check is already wired into `Publish dartpy` for Linux
stable-ABI wheels; its wiring is complete, while full wheel qualification
remains a release gate.

**Interim bindings (D31a).** M6a landed the effort's tested camelCase GUI
module as the interim `dartpy.gui`. Main may mix styles until M6b; the
7.0 release may not. M6b builds `dart.render` and `dart.viewer` in the 7.0
style, ports the tutorials, docs and tests, and deletes the interim module
before the API freeze `F`, so release candidates carry the final render API.
The target-shape render test in `python/tests/unit/gui/` already uses the 7.0
API, so it stays when the interim tests are ported.

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
- GUI binding sites adopt WP-7.0.4's interpreter-finalization rule once it
  lands; until then they raise when no thread state can be attached.
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
| WP-7.0.3b | Honor or reject SDF pose attributes (`rotation_format`, `degrees`, `relative_to`) instead of ignoring them | Merged ([#3719](https://github.com/dartsim/dart/pull/3719)); four review findings remain open ([verification](verification.md#wp-703b-sdf-pose-attributes)) |
| WP-7.0.4 | Make dartpy's GIL acquisitions safe during interpreter finalization | Done ([#3699](https://github.com/dartsim/dart/pull/3699)); the GUI binding sites followed in [#3713](https://github.com/dartsim/dart/pull/3713) |
| WP-7.0.5 | Pin the pre-refactor performance baseline and add many-World and Python-loop rows | In progress: [#3720](https://github.com/dartsim/dart/pull/3720) added the rows; the `dart7-base` pin is recorded right before the first M1 PR |
| WP-7.0.6 | Add the gz compatibility shim test, the contact-surface header probe, version plumbing and inventories | In progress: [#3725](https://github.com/dartsim/dart/pull/3725) merged the version file, shim test and header probe; [#3729](https://github.com/dartsim/dart/pull/3729) (in review) adds the gz API inventory |
| WP-7.0.7 | Record the API freeze, checkpoints, must-have list and acceptance tests | In review ([#3728](https://github.com/dartsim/dart/pull/3728)) |
| WP-7.0.8 | Add newer-Python wheels to the 6.20.x maintenance line | In progress: [#3704](https://github.com/dartsim/dart/pull/3704) added 6.20.1 wheels for Python 3.12 to 3.14 and [#3730](https://github.com/dartsim/dart/pull/3730) Linux Python 3.15 wheels; macOS and Windows Python 3.15 wheels wait for conda-forge's Python 3.15 builds of `bullet-cpp` |

Each packet records its scope, non-goals, gates and evidence in
[verification.md](verification.md) before it is marked done. M0 exits when
every packet above is done, and M5a starts then. M1 starts right after the
`dart7-base` pin, as the [critical path](#milestones) shows.

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
  - the snapshot and checkpoint oracle (arrives with M1c).
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
    selected default detector (FCL until the gated switch lands). It asserts that each override runs.
  - A CTest compiles `#include <dart/constraint/ContactSurface.hpp>` against the
    installed DART as C++14, the dialect gz-physics probes with, using only DART
    and Eigen include paths. Downstream feature
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
    against its own baseline;
  - the codemod on at least three external codebases.

Every packet needs two clean review passes on its final state.

## Risks

- **Scope.** The program spans about 90–130 PRs. Mitigations: core-first gates,
  one PR per change, the M2 stop rule, and M3 is 7.x by default.
- **The release waits for the renderer.** Mitigation: escalate early if
  checkpoint C2 is at risk.
- **Downstream drift.** Mitigation: the shim test runs against the current
  gz-physics development branch, plus a scheduled lane.
- **Concurrent work on core files.** Mitigation: the step-path quiet window.

## Blockers and next actions

- WP-7.0.8's macOS and Windows Python 3.15 wheels wait for conda-forge's
  Python 3.15 builds of `bullet-cpp`. Pending maintainer decision: whether they
  hold M0's exit, and with it M5a's start.
- Pending maintainer decision, needed before `F` is declared: whether `F` also
  waits for the additive must-haves, or they are exempt from the move to 7.1
  (see the [release rules](#release-train-and-checkpoints)).
- The "DART 7.0.0" milestone (#105, formerly "DART 6.21.0") tracks the release,
  and main's release target reads 7.0.0.
- Publishing each packet needs maintainer approval.
- Pending maintainer action: check PyPI's project history for deleted 7.0.0
  files; PyPI never accepts a reused file name.
- See [RESUME.md](RESUME.md) for the immediate next steps.
