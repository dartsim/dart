# Orchestration dashboard — DART 6.20 dependency minimization

> **Live status board for the whole multi-lane effort.** This is the
> orchestration/monitoring artifact owned by the **dependency-reduction lane**
> (which, having finished its own removals, now runs coordination/monitoring for
> the effort). It tracks every lane's PRs, CI, conflicts, and sequencing.
>
> - **Plan of record:** `README.md` (overall SSOT) + `02-default-environment-split.md`
>   (this lane) + `03-native-collision-port-scoping.md` (native-collision scope).
>   This board does **not** duplicate plans — it tracks **status**.
> - **Naming:** lanes are named by **work scope**, never by local working-directory
>   names (a project convention; see the naming note in
>   `02-default-environment-split.md`).
>
> _Last updated: 2026-07-09._

## Lanes & owners

| Lane (work scope) | Charter | State |
| --- | --- | --- |
| **Dependency-reduction lane** (this one) | Optimizer removal; default-env analysis; **now orchestration/monitoring** | Own removals **complete**; running this board |
| **Native-replacement lane** | `dart/external/*` → native built-ins; **GUI/OSG + GLUT removal** | External replacements + **GLUT/lodepng removal done** (#3116 merged) |
| **Native-collision-port lane** | Port DART 7 `dart/collision/native/` → DART 6.20 (make FCL/Bullet/ODE optional) | Phase 0 (#3271), phase 1 (#3281), phase 2 (#3303, #3306, #3318, #3319, #3321, #3322, #3324, #3325, #3343, #3350), phase 3 (#3352, #3355, #3358, #3359, #3360), and first phase-4 slices (#3362, #3364) merged; phase 4 closeout / remaining measured optimization is next |
| **Perf / parallelism lane** (issue #3056) | Island deactivation, parallel-safe solves, benchmarks | Round 1 landed through #3199/#3203 (guardrails); **round 2 active in `docs/dev_tasks/dart6_performance_generalization/`** — WP-PG.01 baseline packet **#3263 merged** (tracks the native-collision port as its WS-F lane, external owner) |

## PR tracker

### ✅ Merged — external native-replacement (native-replacement lane)
- **#3076** convhull_3d → native `dart/math` detail
- **#3078** IKFast header → `dart/dynamics`
- **#3081** vendored ImGui → system/fetch
- **#3088** odelcpsolver → native Dantzig kernel
- _Status: this workstream is essentially complete (lodepng removal folds into GLUT removal, #3116)._

### ✅ Merged — optimizer removal + planning/orchestration (this lane)
- **#3105** remove deprecated optimizer backends (→ dart-optimization)
- **#3107** default-env split lane plan + native-collision scoping docs
- **#3119** work-based lane names + lane handoff record

### ✅ Merged — GUI/OSG (native-replacement lane)
- **#3113** GLUT-to-OSG removal plan
- **#3116** Remove GLUT GUI stack (merged 2026-06-22) — drops GLUT/Xi/Xmu/freeglut **and removes lodepng** (`dart/external/lodepng` deleted in the same PR)

### ✅ Merged — perf/parallelism + enabling fixes (issue #3056)
- **#3071** opt-in simulation threading · **#3085** plane/contact-cap collision ·
  **#3086** resting-world deactivation · **#3089** contact benchmark example ·
  **#3111** constrain unsafe parallel island solves · **#3112** accelerate settled worlds ·
  **#3118** inverse-dynamics profiling driver (merged 2026-06-22)
- **#3114** FCL null-contact fix (unblocked the coverage/Asserts CI regression)

### ✅ Merged — code-footprint
- **#3122** Remove legacy `dart/integration` module (merged 2026-06-22) — deletes the unused
  `Integrator`/`EulerIntegrator`/`RK4Integrator`/`SemiImplicitEulerIntegrator`/`IntegrableSystem`
  + their installed `include/dart/integration` headers (a public-surface footprint reduction).

### ✅ Merged — CI / hygiene
- **#3120** stop coverage job double-running tests (merged 2026-06-22) — the coverage-job fix
- **#3102** ignore `.omo` · **#3100** pixi lockfile · **#3074/#3075** plan + AI workflows import
- **#3348** centralize MSVC toolchain policy for DART 6 LTS (merged
  2026-07-08).

### ✅ Merged — native-collision-port lane
- **#3123** Speed up DART primitive plane collision — **MERGED & PR closed 2026-06-22 19:05**
  (squash commit `22f1a13d61b`, now in `release-6.20`'s history; `gh pr view 3123 --json state`
  → `MERGED`). Dependency-free primitive plane contacts + finite-shape broadphase pruning —
  **first piece** of the native collision port (the FCL/Bullet/ODE-reduction lever); not yet
  the FCL-optional default-flip.
- Follow-up #3056 release work has landed through #3199/#3203. This resolves the
  measured headless-performance path, but it does **not** port DART 7
  `dart/collision/native/` or make FCL optional.
- **#3271** phase-0 baseline packet (merged 2026-07-05) — captures the
  incumbent guard rows, default-flip verdict, and phase-6 acceptance envelope.
- **#3281** native collision math core (merged 2026-07-05, squash commit
  `135cc8f20765`) — phase-1 internal-only C++17/no-EnTT math core under
  `dart/collision/native/`; no installed component, no detector adapter, and no
  default change. Its PR evidence re-ran the phase-0 guard rows bit-identically.
- **#3298** release-6.20 native sphere-sphere zero-contact-limit fix (merged
  2026-07-06).
- **#3302** phase-2 adapter execution plan (merged 2026-07-06).
- **#3303** phase-2 P1 BruteForce broadphase, internal-only (merged
  2026-07-06).
- **#3306** phase-2 P2 narrowphase dispatcher, sphere/box only (merged
  2026-07-06).
- **#3318** phase-2 P3a adapter skeleton + sphere/box conversion, intentionally
  unregistered (merged 2026-07-07).
- **#3319** phase-2 P3b bridge translation, `"native"` registration, `sphere_box`,
  and parity coverage (merged 2026-07-07).
- **#3321** phase-2 P4 capsule primitive pairs (merged 2026-07-07).
- **#3322** phase-2 P5 convex foundation and capsule-capsule (merged
  2026-07-07).
- **#3324** phase-2 P6 cylinder collision pairs (merged 2026-07-07).
- **#3325** phase-2 P7 mesh collision pairs (merged 2026-07-08).
- **#3343** phase-2 P8/P9 distance module and plane primitive/convex coverage
  (merged 2026-07-08).
- **#3350** phase-2 P10 mixed-scene FCL/DART/native parity coverage (merged
  2026-07-08).
- **#3352** phase-3 D1 native detector distance queries and native basename
  normalization (merged 2026-07-08).
- **#3355** phase-3 D2 native detector raycast adapter and native-vs-Bullet
  raycast benchmarks (merged 2026-07-08).
- **#3357** DART 6 autonomous AI workflow rename to `dart-ultrawork` (merged
  2026-07-09).
- **#3358** phase-3 D3 native `VoxelGridShape`/octree replacement via compound
  voxel boxes and compound collision/distance/raycast routing (merged
  2026-07-09).
- **#3359** phase-3 D4 native CCD engine support (merged 2026-07-09).
- **#3360** phase-3 D5 native persistent manifold cache, contact reduction,
  and cached-impulse seed/write-back (merged 2026-07-09).
- **#3362** phase-4 native contact-container dashboard rows (merged
  2026-07-09).
- **#3364** phase-4 solver-facing native manifold cap with parent-vs-PR and
  detector-vs-detector evidence (merged 2026-07-09).
- **#3283** main-branch dual for the native sphere-sphere binary-check fix
  (merged 2026-07-07).

### ✅ Merged — former monitoring queue (all landed by 2026-07-04)

- **#3209** contact-rich container benchmark (a required default-flip gate) ·
  **#3226** deactivation final-quiet gate · **#3227** gz-physics joint-detach
  fix · **#3229** C++17 SIMD abstraction (`dart/simd`; optimization packet,
  not a correctness prerequisite) · **#3230** DART 6 performance dashboard
  (durable benchmark capture path) · **#3234** native-collision port planning
  refresh (this task's plan of record) · **#3241** perf round-2 plan ·
  **#3239** release-branch AI enforcement stack · **#3245** MSVC SIMD fix ·
  **#3233** release CI concurrency fix.

### 🔄 Open — monitoring (checked 2026-07-09)

- **Phase 4** — #3362 and #3364 are merged. The next native-collision session
  should refresh benchmark evidence on the current base, close out Phase 4 if
  native still clears the performance bar, or make one cohesive measured
  follow-up PR before moving to Phase 5.
- **#3353** is merged on `release-6.20` for the separate
  performance-generalization plan parking lane.
- The earlier monitoring queue has landed: #3283, #3317, #3319, #3321, #3322,
  #3324, #3325, #3357, #3358, #3359, #3360, #3362, and #3364 are merged. The
  perf lane's WP-PG.01 baseline packet **#3263** also merged.

Related remote heads still visible: `feature/native-occupancy-grid`,
`task/native-collision-performance-exec`, and six `perf/dart6-*` round-1
experiment branches (published for reference; triaged in the perf lane's
WP-PG.01).

_(Note for automated reviewers: a just-merged PR can briefly still show "Open" on its page
due to GitHub merge-state lag — confirm via `gh pr view <n> --json state` and `git log`
before treating it as an open/active PR.)_

### 🛠️ Native-collision-port lane (the largest dependency lever — FCL/Bullet/ODE)
- **Current state:** DART 6.20 now contains #3281's internal native math core,
  phase-2 P1-P10, the DART 6 adapter, and the opt-in `"native"` factory key,
  but it has no FCL-optional default.
  `release-6.20` still uses FCL as the default detector — created in *both*
  `ConstraintSolver` constructors.
- **Phase 0 (captured 2026-07-04, recaptured 2026-07-05):** all
  evidence-harness prerequisites merged (#3209 container workload, #3230
  dashboard capture path); the baseline packet is recorded in
  `05-phase0-baseline-packet.md` (raw evidence: `05-artifacts.md`) on
  `1e6a8332a730` after merging `origin/release-6.20` = `949a9c2ff5ed`, with
  the verdict "native default NOT allowed at this tip" and the phase-6
  acceptance envelope. Consume the committed summaries for phase-1
  sequencing. For the phase-6 tolerance gate, retrieve JSONL dumps matching
  the recorded SHA-256 digests or recapture dumps on the flip PR's parent
  and compare within that same recapture.
- **Phase 2 status:** P1-P10 are merged: broadphase, dispatcher, adapter bridge,
  `"native"` registration, sphere/box/capsule/convex/cylinder/mesh/plane
  coverage, distance helpers, mixed-scene parity, and associated parity/
  performance tests. **Phase 3 D1-D4 are merged (#3352/#3355/#3358/#3359):**
  native detector distance, raycast, VoxelGrid/compound wiring, and CCD support
  against the incumbent support gaps. **Phase 3 D5 is merged (#3360):**
  persistent manifold cache/reuse and cached-impulse seed/write-back. **Phase 4
  has begun (#3362/#3364):** native dashboard rows and solver-facing manifold
  contact reduction are merged, but Phase 4 still needs closeout evidence or one
  consolidated measured follow-up before Phase 5.
- **Default flip:** still late-phase only. Do not flip defaults until `03`'s full
  A/B packet and gz gate pass.
- _Hold each follow-up to `03`'s bar: gz-compat (`pixi run -e gazebo test-gz`),
  feature/contact parity, evidence-driven perf ≥ Bullet/ODE/FCL, and
  outcome/hash/scene-dump tolerances. The FCL-optional default-flip (the actual
  dependency drop) is still ahead._

## Coordination flags / blockers

1. **Base / conflict status**:
   - Current planning baseline: `origin/release-6.20` =
     `613241385ae58fed2d2a47e9ff53beb2972d4b76`; `origin/main` =
     `a70fc2ed5cb7ea40f72dce68b7d374583ab7feee`.
   - Open PRs routinely fall behind as the base advances; a maintainer merge-up
     clears it. Exact behind-counts aren't tracked here (too volatile).
   - Remote mutations are maintainer-owned unless the maintainer explicitly
     requests a direct maintenance push.
2. **Shared hot files:** `pixi.toml` / `pixi.lock` are touched by multiple lanes —
   **merge `origin/release-6.20` before pushing**, never rebase a published PR branch
   (per `AGENTS.md` / `02`).
3. **CI health (2026-07-04):** the known branch-level failures (macOS arm64
   SIMD `-Werror`, dartpy format preflight, shallow-support/SIMD rows) were
   fixed by **#3267/#3272/#3273, all merged 2026-07-04**. Residual red rows
   should be triaged against base-push history before blaming PR content:
   Windows `Install`-step and coverage `Build with coverage` failures
   reproduced on base pushes, and FreeBSD ssh exit-8 / runner `Setup pixi`
   failures are infra flakes that clear on re-run.
4. **Native-collision lane: stay inside the active phase.** The #3056
   performance stack is useful evidence, but the DART 7 native default flip is
   still a later phase. Phase 4 measured performance work is active; do not treat
   `feature/native-occupancy-grid` or `task/native-collision-performance-exec`
   as release-branch PRs unless a live PR exists and is based on
   `release-6.20`.

## Effort-level status

- **Removed so far:**
  - _Package/pixi dependencies removed:_ optimizer backends (ipopt/nlopt/pagmo/snopt).
  - _Vendored `dart/external` source replaced with native/DART-owned code:_
    convhull_3d (→ native math), ikfast (→ `dart/dynamics` header), odelcpsolver
    (→ native Dantzig).
  - _Vendored source + packages deleted (GLUT chain, #3116):_ the `lodepng` tree
    + GLUT/Xi/Xmu/freeglut.
  - _Not removed — rehomed:_ ImGui's vendored tree was swapped for a
    FetchContent/system target (#3081), but `imgui` is **still a declared
    dependency**; it folds into the GUI/OSG work, not the removed set.
- **Just landed:** GLUT removal (#3116, merged 2026-06-22) → drops GLUT/Xi/Xmu/freeglut
  **and removes lodepng** (`dart/external/lodepng` deleted in the same PR).
- **Just landed (2026-06-22):** legacy `dart/integration` dead-code removal (#3122);
  native-collision **#3123** (primitive plane contacts + broadphase pruning) — first
  piece of the native collision port.
- **Open queue (2026-07-09):** no open native-collision release PRs remain after
  #3364. Phase 4 should resume from current `origin/release-6.20`; the former
  #3263/#3271/#3281/#3302/#3303/#3306/#3318/#3319, plus
  #3321/#3322/#3324/#3325/#3343/#3350/#3352/#3355/#3358/#3359/#3360/#3362/#3364
  lane milestones, main-branch dual #3283, workflow rename #3357, and MSVC
  policy #3348 are merged.
- **Largest remaining win:** native-collision port → makes FCL/Bullet/ODE
  optional and eventually drops `fcl` from core. The opt-in native detector and
  capability parity pieces are now on DART 6.20, but default-flip and dependency
  decoupling are still late-phase decisions.
- **Confirmed non-removable standalone:** `boost` (OSG-coupled), core deps
  (Eigen/assimp/fmt/tinyxml2/urdfdom), `octomap` (exported-header contract).

## How this board is maintained

- Refresh on each orchestration pass (note: `gh pr list` defaults to
  `--state open`, so query merged **explicitly** — otherwise PRs that merged
  since the last pass are missed and the merged/no-longer-tracked sections go
  stale):
  - `gh pr list --state open` **and** `gh pr list --state merged --base release-6.20`,
  - per-PR `mergeable`/`mergeStateStatus`/checks,
  - `git ls-remote --heads origin` for new lane branches.
- Flag: PRs gone red, PRs fallen behind `release-6.20`, new PRs from any lane,
  shared-file collisions, and the native-collision PR when it appears.
- This lane does **not** execute other lanes' code — it tracks, flags, and
  coordinates (parallel, non-overlapping).
