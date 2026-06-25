# DART 6 Backport Audit (main → release-6.20)

Durably track DART-6-relevant changes that landed on `main` (DART 7) but are
**not yet on `release-6.20`**, so the right ones get backported for the
(unreleased) DART 6.20 release.

## Scope

This task is the single source of truth for the **backport gap** between
`origin/main` and `origin/release-6.20`. It is an audit/triage artifact: it
classifies each verified item and routes it, but it does **not** itself port
code. Each accepted item becomes its own backport PR (dual-PR convention; see
`docs/onboarding/release-management.md`).

Full per-item evidence tables (PR / commit / what / why DART-6-relevant /
main-present-release-absent evidence / scope / verdict / recommendation) live in
[`01-backport-inventory.md`](01-backport-inventory.md). This README owns scope,
framing, and decisions; the inventory owns the detailed evidence;
[`RESUME.md`](RESUME.md) owns the exact, prioritized next steps; and
[`02-execution-log.md`](02-execution-log.md) owns the record of what was actually
ported, how it was verified, and the conflict resolutions — this effort is
all backports merged and every post-gridlock failure fixed; the required CI matrix
has no remaining known failures (a final, uninterrupted green run is the closing
confirmation — see that log).

What this task is **not**:

- It does **not** re-plan or own the native-collision port. That is the largest
  DART-7→6.20 lever and is owned by the sibling tasks (see
  [Cross-lane references](#cross-lane-references)).
- It does **not** chase a performance backport. There is **no actionable perf
  port gap** (see [Performance: no port gap](#performance-no-port-gap)).

Working branch: `docs/dart6-backport-audit-release-6.20`, based on
`origin/release-6.20`. DART 7 reference is `origin/main`. Evidence was collected
on 2026-06-22 after fetching `origin/main`, `origin/release-6.20`, and
`origin/release-6.19`; commit presence was checked with `git branch --contains`.

## Two-bucket framing (read this first)

The audit splits every item into exactly two buckets. This split is the whole
point of the task — keep it structurally clear in any update.

1. **In-scope backports** — compat-critical bug fixes and CI/tooling. These
   match the default release-branch backport policy (`release-management.md`:
   preserve DART 6 compatibility, document dependency/build changes, run the gz
   gate when downstream behavior can be affected). They are candidates to port
   now.
2. **Maintainer-decision features** — `feature_optional` work that sits
   **outside** the default backport scope. Several are Gazebo/URDF-compat-
   adjacent and so are worth an explicit maintainer decision, but they are
   **not** auto-backport and must not be ported without sign-off.

## In-scope backports

The primary backport queue is the **six compat-critical bug fixes** below plus
the **#3115/#3117 forward-port**. All are `compat-critical`,
`verdict=gap-confirmed`. See section 1 of
[`01-backport-inventory.md`](01-backport-inventory.md) for the per-item evidence
and section 1 of [`RESUME.md`](RESUME.md) for the prioritized, ready-to-run
queue.

### Compat-critical bug fixes (primary queue)

| PR | Category | What | Lane status | Scope |
| --- | --- | --- | --- | --- |
| [#2509](https://github.com/dartsim/dart/pull/2509) | compat-critical bug | ARM64/NEON SEGFAULT (`Icosphere` static table) + self-copy bugs (`TranslationalJoint2D`/`UniversalJoint`) + missing `EIGEN_MAKE_ALIGNED_OPERATOR_NEW` | Landed by [#3130](https://github.com/dartsim/dart/pull/3130) | Done |
| [#2339](https://github.com/dartsim/dart/pull/2339) | compat-critical bug | FCL primitive contact-normal normalization; default `mPrimitiveShapeType` MESH→PRIMITIVE | Landed by [#3131](https://github.com/dartsim/dart/pull/3131) | Done |
| [#3115](https://github.com/dartsim/dart/pull/3115) / [#3117](https://github.com/dartsim/dart/pull/3117) | compat-critical bug | Skip non-finite contacts and validate shape dimensions | On `main` (#3115) **and** `release-6.19` (#3117 twin); **missing from `release-6.20`** | Backported by [#3132](https://github.com/dartsim/dart/pull/3132) |
| [#2233](https://github.com/dartsim/dart/pull/2233) | compat-critical bug | URDF multidof joint limits + vector-form `GenericJoint` setters | `main` only; **missing from `release-6.20`** | Backport; parser surface (gz gate) |
| [#2271](https://github.com/dartsim/dart/pull/2271) | compat-critical bug | Bullet phantom / negative-penetration-depth contact filter (new `BulletContact.hpp`) | Backport PR [#3136](https://github.com/dartsim/dart/pull/3136) opened against `release-6.20` | Backport in review; collision surface (gz gate passed locally) |
| [#2285](https://github.com/dartsim/dart/pull/2285) | compat-critical bug | aiScene ownership via `shared_ptr` custom deleters (`makeMeshHandle`) | `main` only; **missing from `release-6.20`** | Backport; public installed-header API change (`getMesh()` return type) — gz gate; see #2325 (§4, supersedes) |
| [#2247](https://github.com/dartsim/dart/pull/2247) | compat-critical bug | mimic-constraint correctness (ERP + force-mixing/limit-clamp) — bug fix only, split from the #2212 feature | `main` only; **missing from `release-6.20`** | Backport the bug fix only; constraint surface (gz gate) |

### CI/tooling

| PR | Category | What | Lane status | Scope |
| --- | --- | --- | --- | --- |
| GitHub Action version bumps | ci-tooling | GitHub Actions version drift (see table below) | `main` ahead of `release-6.20` | **Watch / monitor only — not a porting obligation** (Dependabot updates pins per-branch; see Inventory §5 and RESUME §3). Optional selective bump only if a maintainer prioritizes it. |

### #3115 / #3117 — the forward-port nuance (do not lose this)

This is **not** a normal "missing from the release branch" item. The fix exists
in two places already:

- [#3115](https://github.com/dartsim/dart/pull/3115) "Skip non-finite contacts
  and validate shape dimensions" is on `main`
  (`git branch --contains 15c0567ff50` → `main`).
- [#3117](https://github.com/dartsim/dart/pull/3117) "Skip non-finite contacts
  and validate shape dimensions (DART 6 LTS)" — the DART 6 twin — is on
  **`release-6.19`** (`git branch --contains a99369923c7` → `release-6.19`).

It is **absent from `release-6.20`**. So this is a **forward-port gap where
`release-6.19` is ahead of `release-6.20`**: the DART 6 LTS fix already exists
in 6.x form and just needs to be carried forward onto the newer support branch.
Port the **#3117** twin (already DART-6-shaped), not the `main` #3115 form. This
is a contact/shape-validation surface, so it needs the gz gate.

### #3108 — fmt FetchContent fallback (build/dependency note, NOT auto-backport)

[#3108](https://github.com/dartsim/dart/pull/3108) adds `DART_USE_SYSTEM_FMT`
with a FetchContent `fmt` fallback, to survive the Alt Linux upstream
`libfmt-devel` breakage. It is on `main` only
(`git branch --contains cab0e695dc1` → `main`). **Scope: `feature_optional`
(build-convenience) — do NOT backport by default.** `release-6.20` builds fine
against system fmt via plain `find_package(fmt)`, and the job it protects, the
"Alt Linux repro (Docker)", is a **non-required canary** (the upstream
`libfmt-devel` breakage fails it but does not block `release-6.20` merges). The
backport value is restoring a clean build under that repro, not unblocking a
required gate.

A local WIP branch `altlinux-fmt-fetchcontent` was reported for this backport;
**no such branch exists in this checkout**
(`git for-each-ref … | grep altlinux` → empty), and its follow-up landed on
`main`, not on `release-6.20`. Recorded here as **not confirmed in-flight**;
re-check for that branch (or an open PR) before any work, to avoid duplication.
See section 2 of [`01-backport-inventory.md`](01-backport-inventory.md) for the
full evidence.

### CI/tooling action bumps

`release-6.20` workflows trail `main` on several GitHub Actions. The already-
merged [#3127](https://github.com/dartsim/dart/pull/3127) bumped
`actions/checkout` v6→v7 **on `main`**
(`git branch --contains 98605cae1f4` → `main`); `release-6.20` is still on v6.
Captured version deltas (2026-06-22):

| Action | `release-6.20` | `main` |
| --- | --- | --- |
| `actions/checkout` | `v6` | `v7` (via #3127) |
| `codecov/codecov-action` | `v5` | `v7` |
| `actions/setup-python` | `v6` | — (not used on `main`) |
| `actions/upload-artifact` | `v4` | `v7` |
| `actions/download-artifact` | `v5` | `v8` |
| `pypa/cibuildwheel` | `v3.2.0` | — (not on `main`) |
| `pypa/gh-action-pypi-publish` | `v1.13.0` | `v1.14.0` |
| `awalsh128/cache-apt-pkgs-action` | `v1.6.0` | `v1.6.1` |

These are **monitor-only, not a backport obligation** — Dependabot updates pins
per-branch on its own cadence (see Inventory §5 and RESUME §3). Bump a
`release-6.20` workflow only if a maintainer explicitly prioritizes it, and only
for actions whose workflows actually exist on `release-6.20`; `main`-only actions
(`github-script`, `paths-filter`, `sccache-action`, `benchmark-action`) belong to
DART-7-only workflows and are **not** targets.

## Maintainer-decision features (NOT auto-backport)

These are `feature_optional`. They are listed so the maintainer can decide, not
so an agent ports them by default. Default verdict is **defer** unless the
maintainer explicitly approves. The complete 14-item feature inventory lives in
[`01-backport-inventory.md`](01-backport-inventory.md) section 4 and in
[`RESUME.md`](RESUME.md) section 4; the highest-profile ones are below.

| Item | What | Why it needs a decision | Cost |
| --- | --- | --- | --- |
| [#2338](https://github.com/dartsim/dart/pull/2338) | Support convex mesh collisions and rendering (fixes [#872](https://github.com/dartsim/dart/issues/872)) | Gazebo/URDF-compat-adjacent (convex collision geometry) | **Large** — needs the new `ConvexMeshShape` type ported too |
| [#2325](https://github.com/dartsim/dart/pull/2325) | MeshShape adopts `TriMesh` internal representation; Assimp APIs deprecated (removed in DART 8) — **the aiScene/aiMesh-removal item; supersedes #2285** | Decouples MeshShape from Assimp *types* (not a dep drop); gz/gz-sim compat preserved under conditions | **Large** (72 files); maintainer go/no-go |

### #2338 — ConvexMesh is bigger than a bug fix

[#2338](https://github.com/dartsim/dart/pull/2338) is **not** a drop-in backport:
it introduces a **new `ConvexMeshShape` shape type** plus its collision and
rendering paths. A 6.20 backport must port that new type (a public-API surface
addition on a release branch), not just the call sites. It is on `main` and
several feature branches, not on any release lane
(`git branch --contains a6b93bf762f`). Treat it as a feature port behind a
maintainer go/no-go, with the gz gate (it is a collision surface).

### #2325 — MeshShape adopts TriMesh internal representation (supersedes #2285)

[#2325](https://github.com/dartsim/dart/pull/2325) ("MeshShape: adopt TriMesh
internal representation", issue [#453](https://github.com/dartsim/dart/issues/453),
commit `fadd164d14e`) makes `dart::math::TriMesh<double>` the canonical in-memory
representation of `MeshShape`, adds a full Assimp-free public API
(`getTriMesh()`/`getPolygonMesh()`, span-based material/submesh/texture-coord
accessors), and pushes every `aiScene`/`aiMesh`-typed entry point behind
`[[deprecated(... removed in DART 8)]]`. It is on `main` only
(`git branch --contains fadd164d14e` -> `main`), missing from `release-6.20`.
**Scope: `feature_optional` (API-evolution), Large.** This is internal-
representation **decoupling, not a dependency drop** — `dart::math::TriMesh`
already ships on `release-6.20`, and Assimp stays a hard `REQUIRED` build/runtime
dep (the OBJ/DAE/STL loader) on both branches until DART 8.

**Supersedes [#2285](https://github.com/dartsim/dart/pull/2285).** #2325 landed
after #2285 and is the later state on `main`: `getMesh()` returns raw
`const aiScene*` again (deprecated), so #2325 needs **no** `shared_ptr` return-
type break while still subsuming #2285's lifetime fix via `makeMeshHandle`. If
either is taken to 6.20, **prefer porting #2325**, not #2285.

**Backward-compat (one line):** COMPATIBLE-WITH-CONDITIONS for both gz-physics
and gz-sim (gz-sim transitively, via the gz-physics dartsim plugin) — provided
the deprecated `aiScene` ctors/`getMesh()`/`setMesh()` are kept, `mMesh`/Shape
dirty flags stay protected, `MeshHandle::operator=(const aiScene*)` and the
`getTriMesh()` lazy-conversion shim are backported intact, and the port is gated
on `pixi run -e gazebo test-gz` (gz-physics9_9.0.0, `EntityManagement_TEST`).
This is a feature port behind a maintainer go/no-go, not an auto-backport.

## Performance: no port gap

**There is no actionable performance backport gap. Do not port `main`'s
DART-7-only perf work.** State this explicitly so future agents do not chase it.

`release-6.20` met its performance needs **independently**, via DART-6-native
PRs that already landed on the release lane:

- [#3028](https://github.com/dartsim/dart/pull/3028) (articulated forward
  dynamics ~18%, bit-exact),
  [#3023](https://github.com/dartsim/dart/pull/3023) (constraint-solver hot path
  ~1.28x, bit-exact),
  [#3033](https://github.com/dartsim/dart/pull/3033) (ContactConstraint
  per-pivot alloc, bit-exact),
  [#3037](https://github.com/dartsim/dart/pull/3037) (BodyNode force-aggregation
  heap temp, bit-exact),
  [#3040](https://github.com/dartsim/dart/pull/3040) (per-step alloc removal,
  bit-exact),
  [#3125](https://github.com/dartsim/dart/pull/3125) (DART native active contact
  aggregation),
- plus the `issue_3056_dart6_performance` task (island deactivation, parallel-
  safe solves, native-collision perf slices).

`main`'s perf work —
[#2504](https://github.com/dartsim/dart/pull/2504) (allocator benchmark suite +
hot-path opt),
[#2537](https://github.com/dartsim/dart/pull/2537) (BodyNodePool AVX-512
alignment),
[#3011](https://github.com/dartsim/dart/pull/3011) (Dantzig interior fast-path
skip for allocator scratch),
[#3096](https://github.com/dartsim/dart/pull/3096) (boxed-LCP fallback per-step
allocations),
and the `dart/collision/native` stack — lives on **DART-7-only paths that do not
exist on `release-6.20`** (allocator framework, EnTT/C++23 native collision
world). Porting it would mean reimplementing DART 7 architecture, not
cherry-picking a fix. That is out of scope here, and where it *is* wanted it is
already covered DART-6-natively by the lanes above.

## Cross-lane references

This audit references but does **not** re-plan these sibling tasks:

- **Native-collision port (the big-ticket lever).** Porting DART 7
  `dart/collision/native` so FCL/Bullet/ODE become optional and `fcl` drops from
  core is the **largest DART-7→6.20 change**. It is owned by
  `docs/dev_tasks/dart6_dependency_minimization/` (lane 3, scoped in
  `03-native-collision-port-scoping.md`) and tracked in
  [issue #3056](https://github.com/dartsim/dart/issues/3056). Only its first
  slice ([#3123](https://github.com/dartsim/dart/pull/3123)) has merged. This
  audit treats the native-collision work as **owned there**; it is the
  native-collision backport item, and it is not duplicated or re-planned here.
- **DART 6.20 perf/parallelism.**
  `docs/dev_tasks/issue_3056_dart6_performance/` implements DART 6.20
  perf/parallelism **independently** of `main`'s DART-7-only perf. It is the
  reason the [Performance: no port gap](#performance-no-port-gap) section holds.

## Decisions

- **Bucketing is policy-driven, not preference.** In-scope = compat-critical bug
  + ci-tooling (matches `release-management.md`). Everything `feature_optional`
  is maintainer-decision, even when Gazebo/URDF-adjacent. The #3108 fmt fallback
  is `feature_optional` (build-convenience), not an in-scope build fix.
- **#3115/#3117 ports the 6.19 twin forward**, not the `main` form. 6.19 is
  ahead of 6.20 on this fix.
- **No perf port.** Locked: `release-6.20` perf is met independently; `main`
  perf is DART-7-path-only.
- **#2338 is a feature port** (new `ConvexMeshShape` type), gated on maintainer
  sign-off, not an auto bug-fix backport.

## Validation gate (per backport surface)

Each backport PR carries its own gate, sized to the touched surface:

- Build + the focused unit test(s) for the touched module.
- The gz-physics gate for any collision/constraint/parser/default-solver
  surface — **or any installed/public-header change that can affect gz-physics**
  (`docs/ai/verification.md`) — so #2339, #3115/#3117, #2233, #2271, #2247,
  #2338, **#2285** (installed `MeshShape::getMesh` API change), and **#2325**
  (MeshShape TriMesh adoption — installed `MeshShape` header) all need it:

  ```bash
  N=${DART_SAFE_JOBS:-$(python3 scripts/parallel_jobs.py)}
  DART_PARALLEL_JOBS=$N CTEST_PARALLEL_LEVEL=$N pixi run -e gazebo test-gz
  ```

- Build-only surfaces (CI/tooling bumps, and #3108 only if it is ever pursued):
  a clean configure/build is the gate; the fmt fallback should additionally be
  checked under the Alt Linux repro path it targets.
- **Always** run `pixi run lint`.
- Milestone **DART 6.20** on every backport PR.

## Remaining work

1. **Compat-critical backport queue** — #2509 is carried by
   [#3130](https://github.com/dartsim/dart/pull/3130), and #2339 is carried by
   [#3131](https://github.com/dartsim/dart/pull/3131). Port the remaining five
   compat-critical items (#3115/#3117, #2233, #2271, #2285, #2247) in the order
   and with the per-surface gates given in [`RESUME.md`](RESUME.md) section 1.
   #3115/#3117 is a forward-port of the DART-6-shaped #3117 twin. Run the gz
   gate on every collision/constraint/parser surface.
2. **CI/tooling bumps** — selective action-version bumps to `release-6.20`
   workflows only where the workflow is shared with `main`; skip `main`-only
   actions.
3. **#3108 fmt fallback** — do **not** backport by default; it is
   `feature_optional`. Open it only if `release-6.20` CI starts hitting the
   broken-system-fmt mode, and first confirm whether an
   `altlinux-fmt-fetchcontent` branch / open PR exists to avoid duplicate work.
4. **Maintainer decisions** — get go/no-go on the `feature_optional` list (#2338
   convex-mesh and #2325 MeshShape TriMesh costed, plus the rest of the 14 in the
   inventory). Default is defer.
5. Keep this README and `RESUME.md` in sync as items land; move durable
   outcomes to `release-management.md` / changelog when a backport merges.

See [`RESUME.md`](RESUME.md) for exact next steps and
[`01-backport-inventory.md`](01-backport-inventory.md) for the full evidence.
