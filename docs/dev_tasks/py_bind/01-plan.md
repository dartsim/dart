# dartpy nanobind binding improvements plan

## Status

- State: proposed
- Scope: nanobind bindings under `python/dartpy/**` + packaging/build glue
- Tracker: see `02-tracker.md`

## Goals

- Prevent lifetime/ownership bugs (especially raw-pointer APIs)
- Improve Python ergonomics (consistent snake_case naming + signatures)
- Make builds/packaging robust (GUI required for `dartpy`, dependency constraints)

## Non-goals

- Full feature parity with historical bindings
- Large, high-level Python wrapper layer (keep changes close to bindings)

## Work Plan

### 1) Lifetime & Ownership (blocker)

- Add keep-alive relationships where C++ stores raw pointers.
  - `SimpleFrame` created with a parent `Frame*` should remain safe if the parent is destroyed (avoid ref-cycles when passed through shared_ptr APIs) (`python/dartpy/dynamics/simple_frame.cpp`, `python/dartpy/dynamics/frame.cpp`).
  - `BodyNode.createSimpleFrame` + free `createSimpleFrame` helper should keep the parent alive, or return an owning `shared_ptr` with clear semantics (`python/dartpy/dynamics/body_node.cpp`, `python/dartpy/dynamics/inverse_kinematics.cpp`).
  - Constraints that capture `BodyNode*` / `Joint*` should keep those objects alive (`python/dartpy/constraint/*`).
  - Avoid in-place construction for Eigen-aligned types (e.g., `EIGEN_MAKE_ALIGNED_OPERATOR_NEW`) by allocating on the heap (e.g., `nb::new_`) to prevent alignment-related crashes.
  - Criteria objects storing weak pointers should keep nodes (or their owning skeleton) alive to avoid surprising invalidation (`python/dartpy/dynamics/chain.cpp`, `python/dartpy/dynamics/linkage.cpp`).
- Prefer shared ownership where the C++ API is factory-based (e.g., bind `World` through `World::create(...)` if feasible) (`python/dartpy/simulation/world.cpp`).

**Acceptance**

- No segfaults in Python when objects are used after dropping references to their parents.
- Add/extend unit tests that exercise GC + lifetime edges (under `python/tests/unit/`).

### 2) API Consistency & Python Ergonomics

- Pick a single strategy for camelCase ↔ snake_case.
  - If runtime aliasing is the source of truth, avoid hand-authored duplicates that suppress deprecation warnings (`python/dartpy/_naming.py` + bindings).
- Use snake_case for `nb::arg(...)` so `help()` signatures match the promoted public API (`python/dartpy/**.cpp`).
- Fix obvious signature/argument typos as part of the naming pass (e.g., `MjcfParser.Options` argument name in `python/dartpy/utils/mjcf_parser.cpp`).
- Make out-parameter APIs more Pythonic.
  - Prefer returning result objects or `(ok, result)` tuples over requiring a pointer argument (e.g., collision queries) (`python/dartpy/simulation/world.cpp`, `python/dartpy/collision/collision_group.cpp`).
- Reuse shared conversion helpers for numeric inputs.
  - Prefer `toVector3` / `toVector` helpers for “3-vector” and “vector” parsing to keep numpy + sequence behavior consistent (`python/dartpy/common/eigen_utils.hpp`).

**Acceptance**

- Public signatures are consistently snake_case (method names + argument names).
- Collision APIs are ergonomic without requiring callers to manage raw pointers.

### 3) Build / Packaging Correctness

- Require GUI when building dartpy.
  - Enforce `DART_BUILD_GUI=ON` when `DART_BUILD_DARTPY=ON` and keep `dartpy.gui` always available (`python/dartpy/dartpy.cpp`, `python/dartpy/CMakeLists.txt`).
- Align nanobind version constraints between CMake and `pyproject.toml` (single source of truth).
- Improve extension discovery in the Python shim.
  - Avoid hard-coded extension suffixes; prefer Python’s runtime extension suffix list (`python/dartpy/__init__.py`).
- Remove unconditional debug flags in the extension build; let build type/wheel config drive that (`python/dartpy/CMakeLists.txt`).

**Acceptance**

- `DART_BUILD_DARTPY=ON` implies `DART_BUILD_GUI=ON` (enforced at configure time).
- Version constraints are consistent across build systems.

### 4) Performance (follow-up)

- Release the GIL on long-running, non-callback paths (e.g., stepping + collision checks) via `nb::call_guard<nb::gil_scoped_release>()` where safe.

**Acceptance**

- Stepping/collision-heavy workloads do not block unrelated Python threads.

## Test Plan

- Run `pixi run test-all`.
- Add focused tests for ownership/lifetime and signature expectations under `python/tests/unit/`.

## Definition Of Done

- Lifetime issues addressed with explicit keep-alive or shared ownership patterns.
- Public API is consistent (snake_case names + args) with a documented compatibility story.
- GUI requirement + packaging constraints are coherent and tested.
