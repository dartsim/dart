# RESUME: DART 6 Backport Audit (release-6.20)

Exact next steps for a later session. This file is the actionable companion to
`README.md` (scope, evidence, decisions) and
[`01-backport-inventory.md`](01-backport-inventory.md) (per-item evidence). Work
the **compat-critical backport queue first**, in the order below; everything
else is gated on an explicit maintainer decision or owned by a sibling task.

All backport PRs target the **DART 6.20** milestone and land on a new branch off
`origin/release-6.20`. Do not push or open a PR without explicit maintainer
approval (per `/dart-backport-pr` and
[`docs/onboarding/release-management.md`](../../onboarding/release-management.md)).

## Scope reminder (read before porting anything)

- **In scope by default:** compat-critical bug fixes + ci-tooling, per the
  release-roadmap backport policy. (The one build item, #3108, is
  `feature_optional` build-convenience — see section 3 — not an in-scope build
  fix.)
- **Out of scope by default:** every `feature_optional` item below. Several are
  Gazebo/URDF-compat-adjacent and worth raising, but they are a
  **maintainer decision**, never an auto-backport.
- **Performance: no actionable port gap.** release-6.20 met its perf needs
  independently (#3028 / #3023 / #3033 / #3037 / #3040 / #3125 plus the
  `issue_3056_dart6_performance` task). Do **not** port main's DART-7-only perf
  (#2504 / #2537 / #3011 / #3096 and the `dart/collision/native` stack); those
  live on paths that do not exist on release-6.20.
- **Native collision is owned elsewhere.** The
  `dart/collision/native` port (FCL/Bullet/ODE become optional, fcl drops from
  core) is the big-ticket DART-7->6.20 lever, owned by
  [`dart6_dependency_minimization`](../dart6_dependency_minimization/) lane 3 and
  [issue #3056](https://github.com/dartsim/dart/issues/3056). Only its first
  slice ([#3123](https://github.com/dartsim/dart/pull/3123)) has merged. Do
  **not** re-plan it here.

## 1. Prioritized compat-critical backport queue (do these first)

Ordered by risk/impact. Each row: cherry-pick the listed commit onto a fresh
branch `backport/<pr>-to-release-6.20` off `origin/release-6.20`, milestone
**DART 6.20**, then run the listed gate. Mind the layout adaptation (DART 6 is
PascalCase `dart/utils/...`; main is snake_case `dart/io|...`; pybind11 not
nanobind) — see the recipe in section 2.

- [x] **[#2509](https://github.com/dartsim/dart/pull/2509) — ARM64 SEGFAULT
  (TranslationalJoint2D / Icosphere) + self-copy bugs** *(MOST URGENT — crash
  class)*
  - Backported by [#3130](https://github.com/dartsim/dart/pull/3130).
  - Commit `25b77a15984`.
  - Three sub-fixes, DART 6 PascalCase paths:
    (1) `copy(*this)` -> `copy(*otherJoint…)` in
    `dart/dynamics/TranslationalJoint2D.cpp:97` and `UniversalJoint.cpp:92`;
    (2) drop `static` from the triangles table in
    `dart/dynamics/Icosphere-impl.hpp:93` (release uses
    `std::vector<Triangle>`; semantics identical, fix is removal of `static`);
    (3) add `EIGEN_MAKE_ALIGNED_OPERATOR_NEW` to
    `TranslationalJoint2DUniqueProperties` in
    `dart/dynamics/detail/TranslationalJoint2DAspect.hpp`.
  - Gate: build + `dynamics`/`math` focused unit tests. No collision/constraint
    surface, so the gz gate is not required.
- [x] **[#2339](https://github.com/dartsim/dart/pull/2339) — FCL primitive
  contact-normal normalization** *(collision correctness, gz-physics-facing)*
  - Backported by [#3131](https://github.com/dartsim/dart/pull/3131).
  - Commit `a1d6e416984`. FCL detector still lives at
    `dart/collision/fcl/FCLCollisionDetector.cpp` on release-6.20, so the fix
    applies directly there.
  - Add `getContactGeometryOrder` / `FclContactGeometryOrder` normal-orientation
    logic; flip the `mPrimitiveShapeType` default `MESH`->`PRIMITIVE`
    (line 941); use `fcl::Cylinder`/`fcl::Cone` for `PRIMITIVE`; propagate
    settings in `cloneWithoutCollisionObjects` (line 676).
  - **Behavior-default change** (`MESH`->`PRIMITIVE`): flag it in the DART 6.x
    release notes.
  - Gate: build + `collision` focused unit tests **+ gz gate**
    (`pixi run -e gazebo test-gz`) — collision surface.
- [x] **[#3115](https://github.com/dartsim/dart/pull/3115) — skip non-finite
  contacts + validate shape dimensions** *(forward-port; 6.19 is ahead of 6.20)*
  - Backported by [#3132](https://github.com/dartsim/dart/pull/3132).
  - Do **not** cherry-pick the main commit `15c0567ff50` (snake_case). Instead
    **forward-port the #3117 DART 6 LTS twin** — commit `a99369923c7`, already on
    `release-6.19`, confirmed **not** on `release-6.20`. It is already in DART 6
    PascalCase layout and applies cleanly.
  - Adds the non-finite skip (`point.allFinite()` / `normal.allFinite()` /
    `std::isfinite(penetrationDepth)`) in
    `dart/constraint/ConstraintSolver.cpp::updateConstraints` (before the
    `isZeroNormal` check at line 989), plus `setSize()` validation across
    Box/Capsule/Cone/Cylinder/Ellipsoid/Pyramid, plus the
    `NonFiniteContact` / `ShapeDimensionValidation` tests.
  - Motivated by gz-physics
    [issue #1010](https://github.com/gazebosim/gz-physics/issues/1010)
    (NaN/Inf corrupting the LCP solve).
  - Cherry-pick: `git cherry-pick -x a99369923c7`.
  - Gate: build + `constraint` focused unit tests **+ gz gate** — constraint
    surface.
- [x] **[#2233](https://github.com/dartsim/dart/pull/2233) — URDF multidof joint
  limits + vector-form GenericJoint setters** *(parser correctness,
  gz-physics-facing)*
  - Backported by [#3134](https://github.com/dartsim/dart/pull/3134).
  - Commit `5e8ca03720bd1b5cc4cc65c0b056a49a207fb50d`. On release-6.20, URDF
    joints with >1 DOF get limits only on DOF[0], leaving other DOFs unbounded.
  - Port the multidof limit handling in `dart/utils/urdf/DartLoader.cpp`
    (replace the single-DOF `[0]` indexing at line 518 with the multidof
    `setConstant(...)` path) **plus** the supporting vector-form
    `setRestPositions` / `setDampingCoefficients` / `setFrictions` on
    `GenericJoint` (DIM_MISMATCH-checked; DartLoader depends on them). Carry the
    `test_DartLoader.cpp` regression cases.
  - **Drop** the `.github/ci_macos.yml` and `pixi.toml` / `build_helpers.py`
    hunks from the PR — unrelated infra churn.
  - Gate: build + `io`/`utils` (DartLoader) focused unit tests **+ gz gate** —
    parser surface.
- [x] **[#2271](https://github.com/dartsim/dart/pull/2271) — Bullet phantom /
  negative-penetration-depth contact filter** *(gz-physics Bullet users,
  non-trivial new file)*
  - Backport PR [#3136](https://github.com/dartsim/dart/pull/3136) opened as a
    draft against `release-6.20`.
  - Local gates passed: `pixi run lint`, `pixi run build`, `test_BulletContact`
    build + CTest, `pixi run test-py`, and
    `DART_PARALLEL_JOBS=8 CTEST_PARALLEL_LEVEL=8 pixi run -e gazebo test-gz`.
  - Commit `c70716324c0ccaba9a01c90a39a4695d08acb8c4`. Resolves
    [issue #1184](https://github.com/dartsim/dart/issues/1184). The DART 6 Bullet
    detector still exists on release-6.20, so the fix applies there.
  - Port to DART 6 layout: add `allowNegativePenetrationDepthContacts` field to
    `dart/collision/CollisionOption.{hpp,cpp}`; add new file
    `dart/collision/bullet/detail/BulletContact.hpp` with
    `shouldReportContact()`; wire into
    `dart/collision/bullet/BulletCollisionDetector.cpp`; add the dartpy
    `collision_option` binding and `test_BulletContact.cpp`.
  - Non-trivial (new `BulletContact.hpp` file) but mechanically tractable on the
    DART 6 Bullet stack.
  - Gate: build + `collision` (Bullet) focused unit tests **+ gz gate** —
    collision surface.
- [ ] **[#2285](https://github.com/dartsim/dart/pull/2285) — aiScene ownership
  via shared_ptr custom deleters (makeMeshHandle)** *(memory-correctness;
  public API change)*
  - Commit `ee5be94c6f898e762767141aa44b3cb1bf6b169a`. release-6.20 owns a raw
    `const aiScene* mMesh` with manual `aiReleaseImport` — a lifetime /
    double-free / leak hazard.
  - Port to DART 6 PascalCase paths `dart/dynamics/MeshShape.{cpp,hpp}` (+
    `gui/InteractiveFrame.cpp`): move `getMesh()`/MeshHandle from raw
    `aiScene*` to `std::shared_ptr<const aiScene>` with per-ownership deleters.
    The MetaSkeleton/ReferentialSkeleton/Skeleton hunks in the PR are incidental
    (1–2 lines each) and likely unneeded.
  - **Public API change** (return type of `getMesh()`): acceptable in the
    unreleased DART 6.x line, but have the reviewer confirm ABI/source-compat
    expectations.
  - Gate: build + `dynamics` (MeshShape) focused unit tests **+ gz gate**
    (`pixi run -e gazebo test-gz`). Not a collision/constraint/parser surface,
    but `getMesh()`/`MeshHandle` is an **installed public header** that
    gz-physics builds against, so `docs/ai/verification.md` requires the Gazebo
    gate for installed-header changes that can affect gz-physics. **Superseded on
    `main` by #2325** (TriMesh adoption, §4) — prefer porting #2325, whose
    `getMesh()` stays raw `const aiScene*` (deprecated) so it needs no
    `shared_ptr` return-type break.
- [ ] **[#2247](https://github.com/dartsim/dart/pull/2247) — mimic/coupler
  constraint correctness (ERP + force-mixing/limit-clamp)** *(gz mimic repro;
  most surgical — must split bug fix from feature)*
  - Commit `94a5d5bcad5fe663c3850929f8bc274d3118792d`. PR title targets the
    Gazebo mimic repro.
  - **Cherry-pick only the `MimicMotorConstraint` behavioral changes**
    (`dart/constraint/MimicMotorConstraint.{cpp,hpp}`): ERP-based force mixing,
    `DART_CFM 1e-9` -> `kConstraintForceMixing 1e-6` (clamped `max(cfm, 1e-9)`),
    `setErrorReductionParameter()` clamp to `[0,1]`, finite vel/force-limit
    fallbacks, ERP-scaled `desiredVelocity`. Add a focused regression test.
  - **Drop** the co-introduced `CouplerConstraint` class, the `mimic_pendulums`
    example, and the 651-line SDF fixture — those are NEW FEATURES (tracked as
    #2212 in the maintainer-decision list below), not the bug fix.
  - **Prerequisite:** verify #2212-family per-DoF mimic plumbing is already on
    release-6.20 before porting (the per-DoF mimic runtime is present; confirm).
  - Gate: build + `constraint` focused unit tests **+ gz gate** — constraint /
    default-solver surface.

## 2. Backport recipe (per surface)

Follows [`docs/onboarding/release-management.md`](../../onboarding/release-management.md)
and `/dart-backport-pr`. Run once per queue row.

```bash
# 0. Confirm the source PR/commit is merged on main and not already on release.
gh pr view <SOURCE_PR> --json state,mergedAt,baseRefName,mergeCommit
git fetch origin release-6.20 release-6.19 main  # release-6.19 carries the #3115 LTS twin a99369923c7
git cherry -v --abbrev=40 origin/release-6.20 origin/main | grep <COMMIT_HASH>
# For #3115's twin #3117 (lives only on release-6.19, not main), check that head
# instead — `git cherry ... origin/main` will never list it:
#   git cherry -v origin/release-6.20 origin/release-6.19 | grep a99369923c7
#   # (present on 6.19, absent on 6.20 => forward-port it)

# 1. Branch off release-6.20 without resetting an existing local branch.
BRANCH=backport/<SOURCE_PR>-to-release-6.20
if git show-ref --verify --quiet "refs/heads/$BRANCH"; then
  git switch "$BRANCH"
else
  git switch --no-track -c "$BRANCH" origin/release-6.20
fi

# 2. Cherry-pick with provenance (-x records the source commit).
git cherry-pick -x <COMMIT_HASH>
# For #3115: cherry-pick the DART 6 LTS twin a99369923c7 (already on
# release-6.19), NOT the snake_case main commit 15c0567ff50.
```

**Layout adaptation while resolving the cherry-pick** (main -> release-6.20):

- `dart/io/...` (DART 7) -> `dart/utils/...` (DART 6) — the utils<->io rename.
- snake_case headers/files -> PascalCase
  (e.g. `constraint_solver.cpp` -> `ConstraintSolver.cpp`,
  `dart_loader.cpp` -> `DartLoader.cpp`).
- nanobind bindings (DART 7) -> pybind11 (DART 6 dartpy).
- If conflicts are broad or change behavior, **stop and ask** — do not
  improvise a rewrite.

**Validate (per-surface gate), then lint:**

```bash
# Build + focused unit test for the touched module (e.g. collision/constraint/
# dynamics/io). Use the smallest relevant release-branch target.
pixi run build && pixi run test <focused-target>

# gz-physics gate — REQUIRED for any collision / constraint / parser /
# default-solver surface (#2339, #3115, #2233, #2271, #2247) AND any installed
# public-header change that can affect gz-physics (#2285, #2325 — per
# docs/ai/verification.md):
pixi run -e gazebo test-gz

# Always:
pixi run lint
```

Then request explicit maintainer approval before pushing / opening the PR.
Open against `release-6.20` with `--milestone "DART 6.20"` and the PR template.
ABSOLUTELY NO AI/assistant attribution (no "generated by", no `Co-Authored-By`).

## 3. Batched build / ci-tooling step (low priority, optional)

These have **no DART 6 build/runtime correctness impact** and the policy
excludes normal CI tooling. Do **not** auto-backport. If the maintainer ever
wants release-6.20 dev ergonomics aligned with main, batch them into a single
`backport/ci-tooling-to-release-6.20` branch (milestone DART 6.20,
`pixi run lint` gate only):

- [ ] **[#3108](https://github.com/dartsim/dart/pull/3108)** — `DART_USE_SYSTEM_FMT`
  + FetchContent fmt fallback. **`feature_optional` (build-convenience) — do NOT
  backport by default.** **In-flight note:** the local WIP branch
  `altlinux-fmt-fetchcontent` (commit `a0abf624bd2`) is **gone from the remote**;
  its follow-up landed on main, not on release-6.20. release-6.20 builds fine
  against system fmt via `find_package(fmt)`; backport only if release CI starts
  hitting the broken-system-fmt failure mode. (Alt Linux is a non-required
  canary.)
- [ ] **[#2541](https://github.com/dartsim/dart/pull/2541)** — Eigen
  over-alignment CI guard. Low value on a stable DART 6 ABI; watch only.
- [ ] **[#2736](https://github.com/dartsim/dart/pull/2736)** — gersemi CMake
  formatting (`lint-cmake`). Dev formatting only.
- [ ] **[#2185](https://github.com/dartsim/dart/pull/2185)** — Taplo TOML
  linting (`lint-toml`). Dev formatting only.
- [ ] **[#2251](https://github.com/dartsim/dart/pull/2251)** — codespell spell
  lint + cosmetic typo fixes. Backport a typo only if it touches a user-facing
  string a DART 6 consumer depends on (none identified).
- [ ] **[#2655](https://github.com/dartsim/dart/pull/2655)** — centralized CI
  path filters (`ci-code.yml` + `ci-scope`). Coupled to main's multi-workflow
  layout; a port would require reworking release-6.20's distinct workflow set
  with no correctness benefit.
- [ ] **GitHub Action version currency (Dependabot pin skew)** — checkout
  v6->v7, codecov v5->v7, etc. **Not a porting obligation:** Dependabot updates
  pins per-branch on its own cadence. Monitor only.

## 4. Maintainer-decision feature list (raise; do NOT auto-backport)

All `feature_optional`, confirmed absent on release-6.20, outside the default
backport policy. Surface these to the maintainer for an explicit
keep-on-main-only vs. port decision. Several are Gazebo/URDF-compat-adjacent.
These 14 rows mirror section 4 of
[`01-backport-inventory.md`](01-backport-inventory.md).

| PR | Feature | Port notes if approved |
| --- | --- | --- |
| [#2338](https://github.com/dartsim/dart/pull/2338) | Convex mesh collisions/rendering (`ConvexMeshShape`) | **Largest** — needs the brand-new `ConvexMeshShape` Shape type ported too, plus renderer node + FCL/Bullet/ODE integration + tests (~1688 LOC). Not a clean small backport. |
| [#2325](https://github.com/dartsim/dart/pull/2325) | MeshShape adopts `TriMesh` internal representation (issue [#453](https://github.com/dartsim/dart/issues/453)) | **Supersedes [#2285](https://github.com/dartsim/dart/pull/2285)** — prefer this over #2285 if either is taken; `getMesh()` stays raw `const aiScene*` (deprecated), so no `shared_ptr` return-type break. **Large** (72 files, ~4056/-1116), API-evolution **not** a dependency drop — `dart::math::TriMesh` already on release-6.20, Assimp stays `REQUIRED` on both branches until DART 8. No clean cherry-pick: absent MeshMaterial/MeshLoader/AssimpMeshLoader/detail infra, divergent gui/osg renderer, full nanobind->pybind11 dartpy rewrite. If pursued, KEEP the deprecated `aiScene` ctors/`getMesh()`/`setMesh()`, keep `mMesh`/Shape dirty flags protected, backport `MeshHandle::operator=(const aiScene*)` + the `getTriMesh()` lazy-conversion shim (load-bearing for gz `CustomMeshShape`), scope to #2325 only (exclude later snake_case rename / `string_view`), and gate on `pixi run -e gazebo test-gz`. Minimal alt: port only the additive TriMesh mutator slice (~S) and leave MeshShape on aiScene. |
| [#2354](https://github.com/dartsim/dart/pull/2354) | Split-impulse contact correction (issue #201) | Invasive: touches BodyNode/Skeleton/ConstraintSolver/ClassicRigidSolver. Existing Baumgarte ERP path on release-6.20 remains correct. |
| [#2212](https://github.com/dartsim/dart/pull/2212) | `CouplerConstraint` (gear-like ratio coupling) | New public constraint class + Joint/MimicDofProperties surface; PascalCase port. This is the feature split out of #2247 above. |
| [#2252](https://github.com/dartsim/dart/pull/2252) | `RevoluteJointConstraint` (closed-loop hinges) | Self-contained derived class; base `DynamicJointConstraint` already on release-6.20. (Release has only doc forward-refs, no impl.) |
| [#2254](https://github.com/dartsim/dart/pull/2254) | SDF `<mimic>` parser wiring | Runtime already on release-6.20; only the `SdfParser.cpp` hookup + retargeted example are missing. URDF/SDF-compat-adjacent. |
| [#2279](https://github.com/dartsim/dart/pull/2279) | Raycast filter option (`RaycastFilter`) | Clean, small (~220 LOC), fills the explicit `// TODO(JS): Add filter` on release-6.20. Good candidate if a 6.20 consumer needs filtered raycasts. |
| [#2222](https://github.com/dartsim/dart/pull/2222) | Per-DoF actuator type API | Backport only if a gz-physics consumer needs per-DoF actuator selection on 6.x. |
| [#2242](https://github.com/dartsim/dart/pull/2242) | `WeldJoint::merge()` + `BodyNode::getNodes()` | Model-construction conveniences; on explicit demand only. |
| [#2153](https://github.com/dartsim/dart/pull/2153) | `Inertia::transformed()`/`transform()` | Prerequisite for #2170; bundle the two if either is approved. |
| [#2170](https://github.com/dartsim/dart/pull/2170) | `computeTransformedInertia[FromDensity]` + parser inertia inference | Depends on #2153; bundle. |
| [#2340](https://github.com/dartsim/dart/pull/2340) | Polygon/n-gon mesh support (PolygonMesh) | Large new feature: new `dart/math/PolygonMesh` type + MeshLoader n-gon handling + renderer + assets + tests (~1179 LOC). Stays on main unless a downstream consumer needs n-gon OBJ loading. |
| [#2351](https://github.com/dartsim/dart/pull/2351) | Coordinate charts/atlas for Ball/FreeJoint | Math/representation feature. |
| [#2168](https://github.com/dartsim/dart/pull/2168) | `WorldConfig` + `CollisionDetectorType` enum World selector | **Caution:** the same classic-World API was superseded on main's ECS World; porting re-introduces a now-deprecated config shape. Only on explicit Gazebo/gz-physics need. |

**Not-applicable (recorded so future agents do not re-open them):**

- **`ssik` GUI followups** — ssik is **release-only** (never on main), so there
  is nothing to backport. The described followups (ImGuiViewer gizmo,
  drag-follow, show-all/ghost IK solutions, arm visualization, shared GUI
  scale) are substantially already on release-6.20 tip via merged
  [#3092](https://github.com/dartsim/dart/pull/3092). If the
  `feature/ssik-dartpy-release-6.20` branch resurfaces, diff it against
  release-6.20 tip before treating anything as outstanding.
- **[#2515](https://github.com/dartsim/dart/pull/2515) / #2513 (native FreeBSD
  source support)** — release-6.20 already delivers FreeBSD via the ports
  overlay (`tools/freebsd/patches/*`) plus the Docker-free FreeBSD CI (#2865).
  The in-tree change is equivalent; no action needed.

## 5. Owned elsewhere (one-liner)

Performance and the `dart/collision/native` port are **not** in this audit's
scope: perf was met independently on release-6.20 (see README + scope reminder),
and native collision is owned by
[`dart6_dependency_minimization`](../dart6_dependency_minimization/) lane 3 and
[issue #3056](https://github.com/dartsim/dart/issues/3056) (only slice #3123
merged). Do not port them from here.
