# DART 7 Core-Dynamics Perf Forward-Port

> Active dev-task tracking the **multi-PR** release-6.20 → main (DART 7)
> **performance** forward-port. The effort spans several PRs and still has open
> work (see "Remaining" below); the folder stays until those land, then the
> completing PR deletes it and promotes durable insights per
> `docs/dev_tasks/README.md`. Source of truth for scope, status, and decisions.

## Goal

Forward-port the bit-exact core-dynamics/constraint performance optimizations
authored on `release-6.20` (DART 6 LTS) into `main` (DART 7), where the same hot
paths still exist. These were never dual-PR'd (perf is not bug-fix scope) and are
genuine gaps on main. Aligned with the north star (accurate articulated-body
dynamics; multi-core CPU first-class).

## Scope (candidates from the forward-port audit)

| Sub-opt                                           | Source PR/commit              | Target on main                                 | Bit-exact | Freeze status                     |
| ------------------------------------------------- | ----------------------------- | ---------------------------------------------- | --------- | --------------------------------- |
| isNan/isInf templated on `MatrixBase`             | #3028 `32081a2b81b`           | `dart/math/helpers.hpp`                        | yes       | out-of-scope dir (clean)          |
| `!X.allFinite()` in art-inertia paths             | #3028                         | `dart/dynamics/detail/generic_joint.hpp`       | yes       | body-only (clean)                 |
| `mSkeletonRawPtr` cache (skip weak_ptr::lock)     | #3028                         | `dart/dynamics/body_node.{hpp,cpp}`            | yes       | add as **private** member (clean) |
| force-aggregation `noalias()`                     | #3037 `16632b39902`           | `dart/dynamics/body_node.cpp`                  | yes       | .cpp (clean)                      |
| ContactConstraint `noalias()`                     | #3033 `346e315958e`           | `dart/constraint/contact_constraint.cpp`       | yes       | .cpp (clean)                      |
| fixed-cap `mSpatialNormal` storage                | #3040 `ec8b3a6a5d3` (4b only) | `dart/constraint/contact_constraint.hpp`       | yes       | member is **private** (clean)     |
| ContactConstraint skeleton/reactive/tangent cache | #3023 `fab90e3da41` (5a)      | `dart/constraint/contact_constraint.{hpp,cpp}` | yes       | new members **private** (clean)   |
| contact-pair map → unordered/thread_local         | #3023 (5c)                    | `dart/constraint/constraint_solver.cpp`        | yes       | .cpp (clean)                      |

**Excluded:** #3028 `isInf` loop→vectorized half (already on main); #3023 5b
(already on main via span-based assembly); #3040 4a (`hasExternalDisturbance` —
DART-6-only sleeping subsystem, absent on main); #3193 FreeJoint fast-path (NOT
byte-exact) shipped separately in **#3206** with its equivalence test; and
the ~30-commit legacy-`DARTCollide` perf cluster (DART 7 uses
`dart/collision/native/`).

## Freeze-policy note (verified)

`scripts/check_dart7_legacy_freeze.py` scans only `dart/dynamics` + `dart/constraint`
headers and flags **public/protected** member/function/type surface. It does NOT
flag: `.cpp` files, `dart/math`, function bodies, or **private** members. All
sub-opts above stay clean by placing new/changed members in private sections.
No `bugfix-port` tag is appropriate here (perf is not a bug-fix port); staying
private avoids the gate legitimately.

## Status — multi-PR effort

Shipped PRs (all base `main`, milestone DART 7.0):

- **#3204** — per-DoF actuator override heap-corruption fix (the one dual-PR gap).
- **#3205** — bit-exact core-dynamics batch: `isnan-isinf-template`,
  `allfinite-art-inertia` (9 sites), `skeleton-rawptr-cache` (private member),
  `force-agg-noalias` (3), `contact-noalias` (2), `spatialnormal-fixedcap`
  (private member type). 11/11 dynamics+constraint tests pass; lint + legacy
  freeze clean.
- **#3206** — `#3193` FreeJoint identity fast-path (NOT byte-exact; shipped with
  an equivalence regression test).

### Remaining (active work — why this folder stays)

- **`#3023` 5c — contact-pair counting** (`constraint_solver.cpp`): the first
  `static thread_local std::unordered_map` re-expression was **reverted from
  #3205** after Codex flagged (a) `clear()` reuses only the bucket array, not the
  per-entry nodes, and (b) a **reentrancy hazard** — a shared thread-local map is
  corrupted if a custom `ContactSurfaceHandler::createConstraint()` re-enters
  `updateConstraints()`, dropping outer `numContacts` to 1 and mis-normalizing
  contact params/slip compliance. Redo on fresh `main` as a reentrancy-safe,
  truly allocation-free flat counter (matching the release's vector-backed
  structure), or drop.
- **`#3023` 5a — ContactConstraint skeleton/reactive/tangent caching**
  (`contact_constraint.{hpp,cpp}`): medium-risk, entangled with `#3033`'s
  conditions; overlaps #3205's files, so do on fresh `main` AFTER #3205 merges.

Retire this folder in the PR that lands (or formally drops) both remaining items,
promoting any durable insight to onboarding/design docs.

## Verification bar

`pixi run check-lint`; build `dart` + affected tests
(`UNIT_dynamics_*`, `UNIT_constraint_*`, `INTEGRATION_dynamics_*`); behavior
preserved (existing numerical dynamics tests are the bit-exactness guard).

See [RESUME.md](RESUME.md) to continue.
