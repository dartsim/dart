# Testing

Use Pixi tasks from the repository root.

```bash
pixi run test
pixi run test-py
pixi run test-all
```

`pixi run test` runs the C++ test suite after building tests. `pixi run test-py`
runs the Python binding tests. `pixi run test-all` is the broad default gate for
the release branch.

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
the DART-owned zero-allocation contract for same-shape native-collision
simulation steps after preparation.

Run the focused allocation gate before broadening:

```bash
ctest -R '(Profile|StepAllocation)' --output-on-failure
```

The strict global `operator new` and raw malloc-family counters are meaningful
for native DART collision scenes. Bullet, ODE, and other external collision
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
