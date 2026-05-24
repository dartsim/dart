# Resume: Lie-Group Consolidation

## Last Session Summary

The typed Lie-group base API (SO3/SE3, tangents, SpatialInertia, batch) was
ported from `7/nested_group_product` and merged into `main` via PR #2697
(squash `0c0d63c3d5c`), including four Codex-review bug fixes. `GroupProduct`
was deliberately deferred as incomplete WIP and is the only remaining piece.

## Current Branch

None yet — start a fresh branch from `main`. `main` already contains the merged
base API; `7/nested_group_product` still holds the deferred `GroupProduct`
source.

## Immediate Next Step

Port `GroupProduct{,Base,Inverse,Map}` from `origin/7/nested_group_product` into
`dart/math/lie_group/` under main conventions, resolving the five known gaps in
`README.md` and mirroring the merged `so3_inverse.hpp` / `se3_inverse.hpp`.

## Context That Would Be Lost

- `main` has the full lie-group base API but no `GroupProduct`; the WIP source
  lives only on `origin/7/nested_group_product` (PascalCase, pybind11). Do not
  delete that branch until this lands.
- Mechanical gaps: name typo `GrouProductInverse`, recursive
  `traits<GroupProductInverse>`, missing `PlainObject` alias.
- Design gaps needing author intent: implicitly-deleted
  `GroupProduct::operator=`, and `GroupProductBase::operator*` accepting only
  `GroupProductBase` operands instead of `LieGroupBase` operands.
- The merged `SO3Inverse`/`SE3Inverse` are the reference for the expression
  templates; build/lint/test conventions match the other lie_group units.

## How to Resume

```bash
git switch main && git pull --ff-only
git switch -c <new-branch>
git fetch origin 7/nested_group_product   # inspect the WIP source (read-only)
git ls-tree -r --name-only FETCH_HEAD -- dart/math/lie_group | grep -i group
```

Then port `GroupProduct*` into `dart/math/lie_group/` (snake_case), resolve the
five gaps in `README.md`, register sources/tests in CMake, and verify with
`pixi run lint`, build, and `ctest -R UNIT_math_lie_group`.
