# Resume: Simulation Experimental API Design

## Last Session Summary

Work is continuing on `docs/simulation-api-design`, which is a pushed branch
tracking `origin/docs/simulation-api-design`. The previous completed commit was
`93bb43f6ae6` (`Tighten experimental dartpy property API`), which removed
data-style getter/setter aliases from `dartpy.simulation_experimental`, added
`Frame.relative_transform(...)`, updated stubs/docs/tests, and passed
`pixi run lint`, focused experimental Python tests, and `pixi run test-py`.

The current slice bound `StateSpace` into `dartpy.simulation_experimental`,
because the Python design doc lists it as an intended public symbol and the C++
`StateSpace` already exists as a storage-independent value object.

## Current Branch

`docs/simulation-api-design` — intended to track
`origin/docs/simulation-api-design`.

## Immediate Next Step

Merge the latest `origin/main` into `docs/simulation-api-design`, push the
current branch, and continue with the next narrow public API gap from the design
docs.

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
- Python binding surface:
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
- Existing lookup-style `get_*` methods such as `get_link`, `get_joint`,
  `get_multi_body`, `get_loop_closure`, and `get_rigid_body` remain allowed
  until collection views and uniqueness policy are designed.

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

Before pushing or continuing a new slice, merge the latest main branch into this
branch:

```bash
git fetch origin main
git merge --no-edit origin/main
git push origin docs/simulation-api-design
```
