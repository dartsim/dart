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
