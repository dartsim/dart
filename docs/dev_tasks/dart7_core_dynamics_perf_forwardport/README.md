# DART 7 Core-Dynamics Perf Forward-Port

> Active dev-task. Source of truth for scope, status, and decisions of the
> release-6.20 → main (DART 7) **performance** forward-port. Remove this folder
> in the completing PR after promoting durable insights.

## Goal

Forward-port the bit-exact core-dynamics/constraint performance optimizations
authored on `release-6.20` (DART 6 LTS) into `main` (DART 7), where the same hot
paths still exist. These were never dual-PR'd (perf is not bug-fix scope) and are
genuine gaps on main. Aligned with the north star (accurate articulated-body
dynamics; multi-core CPU first-class).

## Scope (candidates from the forward-port audit)

| Sub-opt | Source PR/commit | Target on main | Bit-exact | Freeze status |
| --- | --- | --- | --- | --- |
| isNan/isInf templated on `MatrixBase` | #3028 `32081a2b81b` | `dart/math/helpers.hpp` | yes | out-of-scope dir (clean) |
| `!X.allFinite()` in art-inertia paths | #3028 | `dart/dynamics/detail/generic_joint.hpp` | yes | body-only (clean) |
| `mSkeletonRawPtr` cache (skip weak_ptr::lock) | #3028 | `dart/dynamics/body_node.{hpp,cpp}` | yes | add as **private** member (clean) |
| force-aggregation `noalias()` | #3037 `16632b39902` | `dart/dynamics/body_node.cpp` | yes | .cpp (clean) |
| ContactConstraint `noalias()` | #3033 `346e315958e` | `dart/constraint/contact_constraint.cpp` | yes | .cpp (clean) |
| fixed-cap `mSpatialNormal` storage | #3040 `ec8b3a6a5d3` (4b only) | `dart/constraint/contact_constraint.hpp` | yes | member is **private** (clean) |
| ContactConstraint skeleton/reactive/tangent cache | #3023 `fab90e3da41` (5a) | `dart/constraint/contact_constraint.{hpp,cpp}` | yes | new members **private** (clean) |
| contact-pair map → unordered/thread_local | #3023 (5c) | `dart/constraint/constraint_solver.cpp` | yes | .cpp (clean) |

**Excluded:** #3028 `isInf` loop→vectorized half (already on main); #3023 5b
(already on main via span-based assembly); #3040 4a (`hasExternalDisturbance` —
DART-6-only sleeping subsystem, absent on main); #3193 FreeJoint fast-path (NOT
byte-exact — deferred to a separate follow-up PR with its equivalence test);
the ~30-commit legacy-`DARTCollide` perf cluster (DART 7 uses
`dart/collision/native/`).

## Freeze-policy note (verified)

`scripts/check_dart7_legacy_freeze.py` scans only `dart/dynamics` + `dart/constraint`
headers and flags **public/protected** member/function/type surface. It does NOT
flag: `.cpp` files, `dart/math`, function bodies, or **private** members. All
sub-opts above stay clean by placing new/changed members in private sections.
No `bugfix-port` tag is appropriate here (perf is not a bug-fix port); staying
private avoids the gate legitimately.

## Status

- [ ] Workflow: derive + adversarially verify exact patches per sub-opt
- [ ] Apply patches (sequential — overlapping files), build incrementally
- [ ] Run dynamics + constraint unit/integration tests
- [ ] Lint (freeze gate must pass clean, no tags)
- [ ] CHANGELOG entry
- [ ] Open PR to `main`, milestone DART 7.0

## Verification bar

`pixi run check-lint`; build `dart` + affected tests
(`UNIT_dynamics_*`, `UNIT_constraint_*`, `INTEGRATION_dynamics_*`); behavior
preserved (existing numerical dynamics tests are the bit-exactness guard).

See [RESUME.md](RESUME.md) to continue.
