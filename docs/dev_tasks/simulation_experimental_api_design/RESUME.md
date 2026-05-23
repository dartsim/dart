# Resume: Simulation Experimental API Design

## Last Session Summary

Work is continuing on `docs/simulation-api-design`, which is a pushed branch
tracking `origin/docs/simulation-api-design`. The last pushed base before the
current MultiBody enumeration slice was `83628bfb9a7` (`Add backend-neutral
parallel executor API`).

Completed slices on this branch include:

- `StateSpace` bound into `dartpy.simulation_experimental` as a Pythonic
  metadata value object.
- Data-like dartpy state tightened to Python properties without parallel
  getter/setter aliases.
- Loop-closure ergonomics improved with optional auto-naming and direct
  `enabled`, `kinematics`, and `dynamics` runtime participation properties.
- Unsupported active loop-closure projection/solve policies rejected until
  compatible runtime stages exist.
- `World::sync(WorldSyncStage::Kinematics)` / `world.sync(...)` added for
  explicit kinematics-only work placement.
- `compute::ParallelExecutor` added as the preferred backend-neutral public C++
  executor name, with `TaskflowExecutor` kept as a DART 7 source-compatibility
  alias.

## Current Branch

`docs/simulation-api-design` — intended to track
`origin/docs/simulation-api-design`.

## Immediate Next Step

The current slice adds construction-ordered `MultiBody` link/joint enumeration
in C++ and Pythonic dartpy list/name properties. Validate it, commit it, merge
the latest `origin/main`, and push the branch. Then continue auditing the next
narrow public API gap from the design docs.

## Context That Would Be Lost

- User requirements:
  - DART 7/dartpy 7 APIs are experimental; DART 8/dartpy 8 promotes the API and
    removes legacy DART 6 APIs.
  - Python bindings in `dartpy.simulation_experimental` must be Python-style
    only, with no C++ camelCase aliases and no parallel data getter/setter
    aliases.
  - Prefer algorithm, approach, paper, or DART-owned names over external engine
    names.
  - Closed-chain kinematic/dynamic structure should be first-class.
  - World should support physics and kinematics-only workflows.
  - Synchronous stepping is the reference behavior; async needs a real
    job/future contract before becoming public API.
  - Freshness principle: ordinary reads should be fresh by default, with
    explicit sync hooks for predictable work placement, and no public dirty
    flags.
  - Do not modify `/home/js/multiphysics-api-design.md`.
- Current StateSpace implementation:
  - C++ source: `dart/simulation/experimental/space/state_space.hpp` and
    `.cpp`.
  - It owns variables with `name`, `startIndex`, `dimension`, `lowerBound`, and
    `upperBound`.
  - It supports `addVariable`, `addVariables`, `finalize`, `isFinalized`,
    `getDimension`, `getNumVariables`, `getVariables`, `getVariable`,
    `getVariableIndex`, `getVariableNames`, lower/upper bounds, and
    `hasVariable`.
- Python `StateSpace` binding surface:
  - Exposes `StateVariable` read-only metadata properties: `name`,
    `start_index`, `dimension`, `lower_bound`, and `upper_bound`.
  - Exposes `StateSpace` metadata methods/properties:
    `add_variable(...)`, `add_variables(...)`, `finalize()`,
    `has_variable(...)`, `get_variable(...)`, `get_variable_index(...)`,
    `dimension`, `num_variables`, `is_finalized`, `variables`,
    `variable_names`, `lower_bounds`, and `upper_bounds`.
  - Uses Pythonic spelling and keyword names only.
- Keep `StateSpace` binding metadata-only. Do not expose `AutoMapper`,
  `VectorMapper`, component mappers, raw registry state, or world-state
  storage yet.
- Python loop-closure ergonomics:
  - `world.add_loop_closure(frame_a=..., frame_b=...)` and
    `world.add_loop_closure(spec)` can request an auto-generated closure name.
  - `closure.enabled`, `closure.kinematics`, and `closure.dynamics` update the
    stored `LoopClosureRuntimePolicy` one field at a time.
  - `closure.runtime_policy` remains available for batch assignment.
- Existing lookup-style `get_*` methods such as `get_link`, `get_joint`,
  `get_multi_body`, `get_loop_closure`, and `get_rigid_body` remain allowed
  until collection views and uniqueness policy are designed.
- Completed compute-executor slice:
  - Added `dart/simulation/experimental/compute/parallel_executor.hpp` and
    `.cpp` with `compute::ParallelExecutor`.
  - Converted `taskflow_executor.hpp` to a compatibility include/alias for
    DART 7 source users.
  - Updated docs, tests, benchmarks, forward declarations, CMake, and
    changelog wording to prefer the backend-neutral public name.
  - The Taskflow implementation dependency remains private to
    `parallel_executor.cpp`.
- Completed compute-executor slice validation:
  - `pixi run ninja -C build/default/cpp/Release test_compute_graph test_world`
  - Focused CTest for `test_compute_graph` and `test_world`
  - `pixi run lint`
  - `pixi run build`
  - `pixi run test-unit`
  - `git diff --check`
- Current MultiBody enumeration slice:
  - Adds C++ `MultiBody::getLinks()`, `getJoints()`, `getLinkNames()`, and
    `getJointNames()` as construction-ordered snapshots of public handles and
    names.
  - Adds dartpy properties `links`, `joints`, `link_names`, and `joint_names`
    on `sx.MultiBody`.
  - Keeps Python lookup methods as `get_link()` / `get_joint()` for the
    current experimental staging surface, and does not add dict-style collection
    indexing before uniqueness/invalidation policy is documented.

## How To Resume

```bash
cd /home/js/dev/dartsim/dart/main
git checkout docs/simulation-api-design
git status --short --branch
git log --oneline --decorate --max-count=5
```

Then inspect:

```bash
git log --oneline --decorate --max-count=10
sed -n '1,260p' docs/design/simulation_experimental_cpp_api.md
sed -n '1,340p' docs/design/simulation_experimental_python_api.md
```

The `StateSpace` slice was validated with:

```bash
pixi run ninja -C build/default/cpp/Release dartpy
PYTHONPATH=build/default/cpp/Release/python pixi run pytest python/tests/unit/simulation/test_experimental_world.py -q
pixi run lint
pixi run test-py
```

Before pushing a new slice, merge the latest main branch into this branch:

```bash
git fetch origin main
git merge --no-edit origin/main
git push origin docs/simulation-api-design
```
