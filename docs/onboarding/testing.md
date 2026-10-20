# Testing

Use Pixi tasks from the repository root.

```bash
pixi run test
pixi run test-py
pixi run test-all
```

`pixi run test` builds the `tests_and_run` target and runs the C++ test suite.
`pixi run test-py` runs the Python binding tests. `pixi run test-all` builds the
default CMake `ALL` target. The branch configuration pins `BUILD_TESTING=ON`,
and `ALL` depends on `tests_and_run` and `pytest`, so this aggregate also runs
CTest and the Python binding tests. It does not run lint; run `pixi run lint`
separately. Use `test` or `test-py` for focused reruns and clearer failure
attribution.

These gates build the GUI but leave out GUI tests that need a graphics device:
CTest tests labeled `gui-device` and the Python rendering tests. Device-free
GUI tests still run in these gates. The `gui` label covers all GUI tests.
For GUI, Filament, `dartpy.gui`, or GUI example changes, also run
`pixi run test-gui`. It builds the tests and dartpy in the same tree, runs
`ctest -L gui -LE filament-native`, then runs `python/tests/unit/gui` and
`python/tests/integration`. Linux capture uses Vulkan and needs no display
server; to use the Pixi environment's Mesa Lavapipe driver as CI does, set
`VK_DRIVER_FILES` to the absolute path of
`.pixi/envs/<env>/share/vulkan/icd.d/lvp_icd.x86_64.json`. Native-window tests
need a desktop session; Linux windows use X11, through XWayland on Wayland
desktops (Xvfb also works):

```bash
pixi run ctest --test-dir build/default/cpp/Release -L filament-native --output-on-failure
```

`DART_ENABLE_GUI_SMOKE_TESTS` defaults to `OFF` and replaces
`DART_ENABLE_GUI_OSG_SMOKE_TESTS`. It adds the device-dependent offscreen
capture smoke test. To include it when reproducing `CI Filament`, configure
with `pixi run config -- -DDART_ENABLE_GUI_SMOKE_TESTS=ON` before running
`pixi run test-gui`.

Pixel bounds in the GUI tests are tuned on a GPU and on Lavapipe. Widen a
bound only with captures from the backend that fails it.

`pixi run check-ai-infra` also reconfigures the current platform, queries
CMake's File API, inspects the expanded configure trace, and inventories CTest
registrations without executing tests. It requires the effective `ALL` target
to depend directly on `tests_and_run` and `pytest`, requires those targets to
cover every configured C++ and Python test except the `gui-device`-labeled
CTest tests and the six GUI-only Python modules (`test_filament`,
`test_gui_offscreen`, `test_agent_capture`, `test_agent_debug_overlay`,
`test_target_shape_render`, `test_viewer_render`), which it requires
`pixi run test-gui` to run, rejects omitted test directories or unowned test
sources, and admits inactive platform tests or the one compile-only C test
only through exact branch-owned predicates and target contracts. It
walks the active generated CTest subdirectory graph, maps registrations back to
configured executables, and admits the required package-version CMake script
only with its exact source, output directory, version, pointer size and active
CMake command. It rejects unapproved command filters or non-executing modes,
selects only the requested branch of multi-config registrations, rejects
result-neutralizing test properties or GTest environment modifications, and
requires selected registrations to use a digit-bounded zero-test failure guard.
Both `pixi run test` and the aggregate reach `tests_and_run`, which removes
every ambient `GTEST_*` variable before invoking CTest while branch-owned
per-test selections remain explicit. Repository pytest gates use
`scripts/run_pytest.py`; it removes every ambient `PYTEST_*` control, disables
external plugin autoload, runs under Python isolation before adding only the
task-owned import path, pins the root configuration, disables `conftest.py`,
and fails if no test body executes. CMake stages the `dartpy` package under a
configuration-aware root (`DART_DARTPY_BUILD_DIR`, the parent of
`$<TARGET_FILE_DIR:dartpy>`) and imports from that root, so Visual Studio,
Xcode, and future multi-config generators use the selected configuration's
extension module instead of a guessed build directory. The checker rejects explicit local
`pytest_plugins` and resolves the installed package without importing a local
shadow. Its semantic probes execute controlled passing and failing CTest/pytest
cases, so source markers alone cannot claim that sanitization, test-body
execution, or failure propagation works. The configured-graph probe also ties
each broad target back to its validated source command and confirms Release
commands use the active Pixi CTest and Python implementations. It catches early
exits, inactive lexical decoys, variable poisoning, command shadowing,
disabled, list-only, skipped, or result-inverted CTest entries, collection-only
or spoofed pytest substitutions, stale generated test directories, and decoy
targets. Native hosted CI remains the platform-specific execution proof.

For model, simulation, collision/contact, or Filament claims, also use
`dart-verify-sim` with the text-first and claim-tied visual/debug path in
[`docs/ai/verification.md`](../ai/verification.md).

## Demo Catalog Verification

The Python catalog is the only demo app. `pixi run cycle-demos` constructs,
steps, and tears down every registered scene without a graphics context.
Its strict cycle exits nonzero if a callback fails. Device-free `test_GuiApp`
and Python viewer tests cover the host, CLI, lifecycle, controls, callback
failures, garbage collection, and interrupts. Tests for Python demo models live
under `python/tests/unit/demos`. `test_GuiAppMemoryDiagnostics` covers the opt-in
diagnostic model. Rendered catalog and viewer checks run through `pixi run test-gui`.

Provision `ssik` and cached robot assets with `pixi run install-ssik` and
`pixi run fetch-robot-assets` before claiming a full 37-scene cycle. Otherwise
ssik and missing cached humanoids are skipped; without optional bindings,
the corresponding scenes are absent from the catalog. Read the cycle's
scene and skip counts as well as its exit status.

The consolidation established each Python port's parity against the C++
catalog before that app was removed. Preserve that method for later scene
changes:

1. Record the revisions and use the same scene, detector, threads, step
   count, camera, and capture dimensions on both versions. Compare
   construction counts (skeletons, body nodes, soft bodies, point masses,
   DOFs, and constraints) exactly for deterministic scenes; compare counts
   rather than random state for randomly generated scenes. Assert counts in
   scene tests or inspect the constructed Python World.
2. Capture both versions to separate output paths, for example with
   `pixi run demos -- --scene <id> --headless --steps 200 --width 960 --height 600 --shot <output>.png`.
   Compose the pair with `pixi run image-sheet`, then assess it semantically
   with `dart-verify-sim`. Images corroborate the numerical oracle and stay
   outside the repository.
3. Script each key and panel callback through the scene's tests; step and
   assert the effect, such as a moved target, toggled support, new solution,
   or changed force. For mouse interactions, script the event handler's
   pick and drag and verify that the spring force moves the picked body.
   Add an interaction capture when the effect is visible.

For changes to the private app host, also configure a vanilla-ImGui build
with `-DDART_USE_SYSTEM_IMGUI=ON` and build `tests` and dartpy: the host is
compiled out, dartpy still imports, and the catalog gives a rebuild hint.
Configure a GUI-off build with
`-DDART_BUILD_GUI=OFF -DDART_BUILD_DARTPY=OFF`, build `tests`, and run the
affected C++ tests. These checks preserve the library boundaries described
in [the architecture owner](architecture.md#python-demo-catalog).

For dependency minimization, run focused tests for the touched component and
then broaden when the dependency affects shared package, collision, constraint,
or GUI behavior.

## Gazebo extension gates in CTest

`pixi run test` also runs two DART-owned compatibility gates:

- `ContactSurfaceProbe` compiles the same header-check source that gz-physics
  uses to detect `dart/constraint/ContactSurface.hpp`. One compiler invocation
  checks C++14 on Unix and C++17 with MSVC, which rejects the C++17 header
  syntax that Unix compilers accept as extensions in C++14. It uses only DART
  and Eigen include directories and no extra definitions. Source and generated
  `dart/` headers provide the installed include layout; the probe inherits no
  library target flags or dependencies.
  The Unix check preserves downstream C++14 acceptance, including tolerated
  extensions; the MSVC check uses DART's required C++17 dialect and exported
  template-parsing options (`/permissive- /Zc:twoPhase-`).
- `test_GazeboShim` mirrors the gz-physics detector subclasses, factory
  registrars, collision-group accessors, ray-cast entry points, and
  `ContactSurfaceHandler` overrides. Counters prove dispatch through collision
  groups and `World::step`, including contact-constraint and surface-parameter
  creation. With all factory creators removed, `World::create()` and both
  `ConstraintSolver` constructors must still select FCL. Bullet and ODE cases
  run only when their components are built; Bullet also checks the direct
  `btCollisionWorld.h` include used by the plugin.

The test-source comments pin the mirrored gz-physics main revision and describe
how to refresh it. After building the tests, run these gates alone with:

```bash
pixi run ctest --test-dir build/default/cpp/Release \
  -R '^(ContactSurfaceProbe|test_GazeboShim)$' --output-on-failure
```

These gates cover DART extension contracts. Continue to use the downstream
Gazebo suites below for plugin and application compatibility.

## GUI Visual Checks

The Linux wheel C22 headless gate runs in the `Publish dartpy` workflow
(nightly, release tags, and manual dispatch) for the abi3 wheels; to run
it locally:

```bash
python3 scripts/wheel_linux.py cp312
python3 scripts/check_headless_wheel.py dist/dartpy-*-cp312-abi3-manylinux_2_28_x86_64.whl
```

The check requires Docker and a native wheel architecture. It runs
`auditwheel show` and lists every ELF object's `readelf -d` NEEDED entries in a
manylinux container, rejecting GL, EGL, OpenGL, X11, xcb, Wayland and Vulkan
dependencies (including auditwheel-renamed libraries). A fresh
`python:3.12-slim` container installs the wheel without system graphics
packages, imports dartpy and steps a world ten times. Capture and both window
setup overloads must raise `dartpy.gui.RenderingUnavailableError`, a
`RuntimeError` subclass, with the required system package names. Each rendering
case runs in its own process so an abort or segfault fails the gate. Linux
capture needs `libvulkan1` plus a usable ICD such as `mesa-vulkan-drivers`;
windows need an X11/OpenGL stack and an X11 or XWayland display. The GUI import
unit tests also cover an installed Vulkan loader with a missing ICD.

For rendering, model, mesh, texture, GUI, or visual-example changes, inspect
the captured screenshot or frame sequence instead of treating command success
as enough evidence. After the render loop finishes, verify that the requested
artifact exists, is non-empty, and is a regular file; a frame that Filament
skips leaves the queued capture unwritten, and write errors throw from the
`frame()` call that performs the capture.

Live ImGui controls that change `VisualAspect` color or alpha after renderable
nodes already exist must also update renderer-visible state. Do not assume that
mutating a `VisualAspect` invalidates an existing drawable when the refresh path
is gated by shape version, first-frame initialization, or dynamic color flags.

At high `--gui-scale` values, avoid long labels trailing after wide widgets.
Put the visible label on its own line, give the control a hidden ImGui ID such
as `##soft_mesh_alpha`, and size sliders or inputs from
`ImGui::GetContentRegionAvail().x` so text does not clip in scaled overlays.

## Simulation Allocation Gates

Changes that touch `World::step` or its private stages (see
[World step stages](architecture.md#world-step-stages)),
`World::enterSimulationMode`, `MemoryManager`, profiler storage, or
constraint-solver scratch must preserve the DART-owned zero-allocation contract
for same-shape simulation steps using the `dart` detector after preparation.

Run the focused allocation gate before broadening:

```bash
ctest -R '(Profile|StepAllocation)' --output-on-failure
```

The strict global `operator new` and raw malloc-family counters are meaningful
for scenes using the `dart` detector. Bullet, ODE, and other external collision
backends may allocate internally, so their allocation coverage should be scoped
to the World-owned base allocator surface instead of global heap counters.

For simulation changes, also run:

```bash
pixi run test-all
pixi run -e gazebo test-gz
```

`pixi run -e gazebo test-gz` builds and tests the pinned gz-physics suite, then
builds the pinned gz-sim smoke test against the source-built DART plugin. Treat
it as the downstream compatibility gate for additive `World` API and
simulation-loop behavior changes. It is the forward lane: it applies
`tools/gazebo/patches/` to gz-physics first, so it does not prove that released
Gazebo still works.

For collision, contact, constraint-solver, sleeping, `World::step`, or
step-stage changes, and before a release, also run the unpatched lanes, which
build released gz-physics and gz-sim against this checkout, run both full
suites, and fail on any failure DART 6.19.4 did not have:

```bash
pixi run gz-compat-ionic       # gz-physics 8.4.0, gz-sim 9.5.0
pixi run gz-compat-jetty       # gz-physics 9.5.2, gz-sim 10.5.0
pixi run gz-compat-harmonic    # gz-physics 7.8.0, gz-sim 8.10.0
```

A first run of a lane builds DART, gz-physics, and gz-sim (about 15 minutes
with 12 build jobs on a 32-core workstation) and then runs the gz-sim suite
serially (13 to 16 minutes); later runs rebuild incrementally. Release
verification of DART 6.20 at `48eb618bb81` passed all three unpatched
gz-physics and gz-sim compatibility gates, with results equal to or better
than DART 6.19.5. The only gz-physics failure is the accepted
ray-intersection expectation described in the expected-failure files.
The 3,000-shape world settles and all bodies sleep with gz-physics' custom
collision filter since [#3632](https://github.com/dartsim/dart/pull/3632).

To distinguish inherited failures from new ones, build the change's base as
a second variant and set `GZ_COMPAT_BASE_VARIANT`; explain inherited failures
as well. Sleeping changes must also run
`pixi run gz-compat-ionic sleep-oracle` without `--allow-unexercised` and
explain every mismatch. Release verification matched 33 of 35 scenarios;
the two differences concern a detector-private contact-cap edit without
notification and attachment sensitivity to the prior resting pose. DART
6.19.5 had 6 mismatches and 27 unexercised scenarios. ODE or
collision-group changes run
`pixi run gz-compat-jetty raycast-probe`. `tools/gazebo/README.md` covers the
steps, the comparison, the expected-failure files, and the benchmark drivers;
`pixi run test-gz-compat-tools` tests the comparison script, the benchmark
world generator, the DART version override, and the DART C++ API inventory
extractor.
