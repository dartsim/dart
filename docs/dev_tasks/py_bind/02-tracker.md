# dartpy nanobind binding improvements tracker

This file tracks execution progress. See `01-plan.md` for background and goals.

## Status

- State: active
- Focus (current): lifetime & ownership fixes

## Open Decisions

- Naming strategy:
  - Prefer runtime snake_case aliasing (`python/dartpy/_naming.py`) as the source of truth, or
  - Prefer hand-authored snake_case APIs in bindings and reduce aliasing?
- Collision API shape:
  - Keep C++-style “out parameter” support, or
  - Move to Pythonic return values (e.g., return a `CollisionResult` / tuple)?
- `World` ownership:
  - Keep `World()` as a value-like binding, or
  - Bind `World` primarily via `World::create(...)` to align with shared ownership patterns?

## Tasks

### 1) Lifetime & Ownership

- [x] Ensure `SimpleFrame` remains safe when its parent is destroyed (avoid ref-cycles with shared_ptr APIs) (`python/dartpy/dynamics/simple_frame.cpp`, `python/dartpy/dynamics/frame.cpp`).
- [x] Fix `BodyNode.createSimpleFrame` ownership (return `shared_ptr`) (`python/dartpy/dynamics/body_node.cpp`).
- [x] Fix free `createSimpleFrame` helper ownership (return `shared_ptr`) (`python/dartpy/dynamics/inverse_kinematics.cpp`).
- [x] Add keep-alive to constraints that capture `BodyNode*` / `Joint*` (`python/dartpy/constraint/*.cpp`).
- [x] Avoid in-place construction for Eigen-aligned constraints (fix segfaults) (`python/dartpy/constraint/dynamic_joint_constraint.cpp`).
- [x] Ensure `Joint` args use polymorphic caster (fix base-pointer offsets) (`python/dartpy/constraint/joint_constraint.cpp`, `python/dartpy/constraint/joint_coulomb_friction_constraint.cpp`).
- [x] Ensure `ChainCriteria` / `LinkageCriteria` lifetimes are safe given weak pointer storage (`python/dartpy/dynamics/chain.cpp`, `python/dartpy/dynamics/linkage.cpp`).
- [x] Add unit tests for lifetime/GC regressions (`python/tests/unit/test_lifetime.py`).

### 2) API Consistency & Ergonomics

- [ ] Remove redundant hand-authored snake_case methods that conflict with aliasing policy (or document exception cases).
- [ ] Rename `nb::arg(...)` to snake_case across bindings (signatures + docs).
- [ ] Make collision APIs ergonomic without requiring raw-pointer arguments (`python/dartpy/simulation/world.cpp`, `python/dartpy/collision/collision_group.cpp`).
- [x] Reuse shared conversion helpers (`toVector3`) in constraint constructors (`python/dartpy/constraint/dynamic_joint_constraint.cpp`).
- [x] Fix MJCF options argument name typo (`python/dartpy/utils/mjcf_parser.cpp`).
- [ ] Reuse shared conversion helpers (`toVector3`, `toVector`) more broadly (`python/dartpy/common/eigen_utils.hpp` + other bindings).

### 3) Build / Packaging

- [x] Enforce GUI requirement for dartpy (fail configure if `DART_BUILD_GUI=OFF`) (`python/dartpy/dartpy.cpp`, `python/dartpy/CMakeLists.txt`).
- [x] Align nanobind version constraints between `python/dartpy/CMakeLists.txt` and `pyproject.toml`.
- [x] Improve extension discovery in `python/dartpy/__init__.py` (avoid hard-coded suffix list).
- [x] Remove unconditional debug flags from the extension build (`python/dartpy/CMakeLists.txt`).

### 4) Performance

- [ ] Release the GIL for long-running, non-callback C++ calls (e.g., stepping, collision checks).

## Validation

- [x] Run `pixi run test-py`.
- [ ] Run `pixi run test-all`.
- [x] Investigate nanobind ref-leaks output (resolve shared_ptr/keep_alive cycles; no leaks under `pixi run test-py`).
