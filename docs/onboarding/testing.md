# Testing

Use Pixi tasks from the repository root.

```bash
pixi run test
pixi run test-py
pixi run test-all
```

`pixi run test` runs the C++ test suite after building tests. `pixi run test-py`
runs the Python binding tests. `pixi run test-all` builds the default CMake
`ALL` target. The branch configuration pins `BUILD_TESTING=ON`, and `ALL`
depends on `tests_and_run` and `pytest`, so this aggregate also runs CTest and
the Python binding tests. It does not run lint; run `pixi run lint` separately.
Use `test` or `test-py` for focused reruns and clearer failure attribution.

`pixi run check-ai-infra` also reconfigures the current platform, queries
CMake's File API, inspects the expanded configure trace, and inventories CTest
registrations without executing tests. It requires the effective `ALL` target
to depend directly on `tests_and_run` and `pytest`, requires those targets to
cover every configured C++ and Python test, rejects omitted test directories or
unowned test sources, and admits inactive platform tests or the one compile-only
C test only through exact branch-owned predicates and target contracts. It
walks the active generated CTest subdirectory graph, maps registrations back to
configured executables, rejects unapproved command filters or non-executing
modes, and requires selected GTest registrations to fail when they match zero
tests. The pytest target pins the root configuration, clears ambient/plugin
overrides, and resolves the installed package without importing a local shadow.
The probe also ties each broad target back to its validated source command and
confirms Release commands use the active Pixi CTest and Python implementations.
It catches early exits, inactive lexical decoys, variable poisoning, command
shadowing, disabled or list-only CTest entries, collection-only or spoofed
pytest substitutions, stale generated test directories, and decoy targets that
a source-marker check alone cannot distinguish. Native hosted CI remains the
platform-specific execution proof.

For model, simulation, collision/contact, or OSG claims, also use
`dart-verify-sim` with the text-first and claim-tied visual/debug path in
[`docs/ai/verification.md`](../ai/verification.md).

For dependency minimization, run focused tests for the touched component and
then broaden when the dependency affects shared package, collision, constraint,
or GUI behavior.

## GUI Visual Checks

For rendering, model, mesh, texture, GUI, or visual-example changes, inspect
the captured screenshot or frame sequence instead of treating command success
as enough evidence. After the render loop finishes, verify that the requested
artifact exists, is non-empty, and is a regular file; capture callbacks can
report write failures after the final frame has already advanced.

Live ImGui controls that change `VisualAspect` color or alpha after renderable
nodes already exist must also update renderer-visible state. Do not assume that
mutating a `VisualAspect` invalidates an existing drawable when the refresh path
is gated by shape version, first-frame initialization, or dynamic color flags.

At high `--gui-scale` values, avoid long labels trailing after wide widgets.
Put the visible label on its own line, give the control a hidden ImGui ID such
as `##soft_mesh_alpha`, and size sliders or inputs from
`ImGui::GetContentRegionAvail().x` so text does not clip in scaled overlays.

## Simulation Allocation Gates

Changes that touch `World::step`, `World::enterSimulationMode`,
`MemoryManager`, profiler storage, or constraint-solver scratch must preserve
the DART-owned zero-allocation contract for same-shape simulation steps using
the `dart` detector after preparation.

Run the focused allocation gate before broadening:

```bash
ctest -R '(Profile|StepAllocation)' --output-on-failure
```

The strict global `operator new` and raw malloc-family counters are meaningful
for scenes using the `dart` detector. Bullet, ODE, and other external collision
backends may allocate internally, so their allocation coverage should be scoped
to the World-owned base allocator surface instead of global heap counters.

For release-branch simulation changes, also run:

```bash
pixi run test-all
pixi run -e gazebo test-gz
```

`pixi run -e gazebo test-gz` builds and tests the pinned gz-physics suite, then
builds the pinned gz-sim smoke test against the source-built DART plugin. Treat
it as the downstream compatibility gate for additive `World` API and
simulation-loop behavior changes.
