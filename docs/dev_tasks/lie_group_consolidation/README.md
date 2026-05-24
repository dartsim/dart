# Lie-Group Consolidation — Dev Task

## Current Status

- [x] Phase 1: Port the typed Lie-group base API (SO3/SE3, tangents,
      SpatialInertia, batch layer) into `main` — landed via PR #2697
      (`0c0d63c3d5c`), with C++ and dartpy (nanobind) tests.
- [ ] Phase 2: Port `GroupProduct` (composite Lie group) into `main` —
      deferred as incomplete WIP; this is the remaining consolidation work.

## Goal

Finish consolidating the DART 7 `dart/math/lie_group/` module by porting the
deferred `GroupProduct` (and its inverse/map machinery) from
`origin/7/nested_group_product` into `main`, adopting main conventions and
mirroring the already-merged `SO3`/`SE3` design.

## Source And Conventions

- Source branch: `origin/7/nested_group_product` (PascalCase, pybind11). It
  holds `dart/math/lie_group/GroupProduct{,Base,Inverse,Map}.hpp` and
  `tests/unit/math/lie_group/test_GroupProduct.cpp`. `main` has none of it.
- Adopt main conventions when porting: snake_case file names,
  `<dart/export.hpp>`, `DART_ASSERT` from `<dart/common/macros.hpp>`, nanobind
  bindings under `dart::python_nb`, and C++20. Model the structure on the
  merged `so3*.hpp` / `se3*.hpp` and especially the working `so3_inverse.hpp` /
  `se3_inverse.hpp`.

## Known Gaps In The WIP (resolve while porting)

1. Class-name typo: `GrouProductInverse` is defined but `GroupProductInverse`
   is referenced.
2. `traits<GroupProductInverse<D>>` recurses into `GroupProductInverse<Scalar>`
   instead of inheriting the plain product's traits. Fix by inheriting
   `traits<typename D::LieGroup>`, mirroring `SO3Inverse`.
3. Missing `PlainObject` alias (it equals `LieGroup`).
4. `GroupProduct::operator=` is implicitly deleted; define it explicitly.
5. `GroupProductBase::operator*` only accepts `GroupProductBase` operands, not
   the `LieGroupBase` operands the inverse machinery and general use need
   (compare `SO3Base::operator*`).

Gaps 1-3 are mechanical. Gaps 4-5 are expression-template design work that needs
the original author's intent; resolve them against the now-merged
`SO3Inverse`/`SE3Inverse` as the reference pattern.

## Non-Goals

- The other `7/*` branches (`7/multibody`, `7/collision_dev`, `7/ecs_view`,
  etc.) are separate consolidation increments and are out of scope here.
- Do not delete `7/nested_group_product` until this work lands — it is the only
  copy of the deferred `GroupProduct` source.

## Verification

- `pixi run lint`, then build, then `ctest -R UNIT_math_lie_group`.
- Add a `test_group_product` mirroring the merged `test_so3`/`test_se3`.
- If exposing `GroupProduct` to dartpy, mirror the merged nanobind binding and
  Python test pattern (`python/dartpy/math/lie_groups.*`, `python/tests/...`).

## Immediate Next Steps

1. Start a fresh branch from `main`.
2. Fetch `origin/7/nested_group_product` and port `GroupProduct*` into
   `dart/math/lie_group/` under main conventions, resolving gaps 1-5.
3. Register the new sources/tests in `dart/math/CMakeLists.txt` and
   `tests/unit/CMakeLists.txt`; verify with the gates above.

## Operating State

Tracked as `PLAN-100` in [`../../plans/dashboard.md`](../../plans/dashboard.md).
