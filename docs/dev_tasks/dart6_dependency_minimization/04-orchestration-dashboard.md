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
> _Last updated: 2026-06-22._

## Lanes & owners

| Lane (work scope) | Charter | State |
| --- | --- | --- |
| **Dependency-reduction lane** (this one) | Optimizer removal; default-env analysis; **now orchestration/monitoring** | Own removals **complete**; running this board |
| **Native-replacement lane** | `dart/external/*` → native built-ins; **GUI/OSG + GLUT removal** | External replacements + **GLUT/lodepng removal done** (#3116 merged) |
| **Native-collision-port lane** | Port DART 7 `dart/collision/native/` → DART 6.20 (make FCL/Bullet/ODE optional) | **In progress — first PR merged (#3123)**; WIP branches continue |
| **Perf / parallelism lane** (issue #3056) | Island deactivation, parallel-safe solves, benchmarks | Merged (incl. #3118 profiling driver) |

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

### ✅ Merged — native-collision-port lane
- **#3123** Speed up DART primitive plane collision — **MERGED & PR closed 2026-06-22 19:05**
  (squash commit `22f1a13d61b`, now in `release-6.20`'s history; `gh pr view 3123 --json state`
  → `MERGED`). Dependency-free primitive plane contacts + finite-shape broadphase pruning —
  **first piece** of the native collision port (the FCL/Bullet/ODE-reduction lever); not yet
  the FCL-optional default-flip.

### 🔄 Open — monitoring
**No open dependency-minimization PRs** — the work queue is clear; all tracked dep-min PRs
have merged (see the merged sections above). Next activity is expected from the
native-collision-port lane's WIP branches (`feature/native-occupancy-grid`,
`task/native-collision-performance-exec`).

_(Note for automated reviewers: a just-merged PR can briefly still show "Open" on its page
due to GitHub merge-state lag — confirm via `gh pr view <n> --json state` and `git log`
before treating it as an open/active PR.)_

### 🛠️ Native-collision-port lane (the largest dependency lever — FCL/Bullet/ODE)
- **#3123 merged** — first piece (primitive plane contacts + broadphase pruning).
- WIP branches (no PR yet): `feature/native-occupancy-grid`, `task/native-collision-performance-exec` — expect follow-up PRs.
- _Hold each follow-up to `03`'s bar: gz-compat (`pixi run -e gazebo test-gz`), feature/contact parity, evidence-driven perf ≥ Bullet/ODE/FCL. The FCL-optional default-flip (the actual dependency drop) is still ahead._

## Coordination flags / blockers

1. **Base / conflict status** (base `9b0c68a5a82`+, advancing as PRs merge):
   - **No real conflicts open.** (#3122's earlier `modify/delete` on the deleted
     GLUT example was resolved by accepting the deletion, as diagnosed.)
   - Open PRs routinely fall ~1 behind as the base advances; a maintainer merge-up
     clears it. Exact behind-counts aren't tracked here (too volatile).
   - All owned by the maintainer.
2. **Shared hot files:** `pixi.toml` / `pixi.lock` are touched by multiple lanes —
   **merge `origin/release-6.20` before pushing**, never rebase a published PR branch
   (per `AGENTS.md` / `02`).
3. **CI health (backlog now draining):**
   - **`coverage`** had a recurring failure; **#3120 — the coverage-job fix —
     is now MERGED** (2026-06-22), so coverage CI should improve on subsequent
     runs. Watch the next coverage results to confirm.
   - **`Debug` / `API Documentation` show flaky failures** on some runs while
     **passing** on sibling PRs with the same base — i.e. not a base regression.
     A CI re-run would confirm/clear them, but **rerunning CI needs explicit
     maintainer approval** (`docs/onboarding/ai-tools.md` Approval Boundaries), so
     this lane only flags it rather than acting. (Coverage logs also show benign
     `Cannot generate a safe runtime search path` RPATH warnings across
     pre-existing targets.)
4. **Native-collision lane: first PR merged (#3123)** — "dependency-free primitive
   plane contacts + finite-shape broadphase pruning". The largest dependency lever
   has begun landing; hold each follow-up to `03`'s gz-compat / parity / perf bar.
   WIP branches `feature/native-occupancy-grid` + `task/native-collision-performance-exec` continue.

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
- **Open queue: empty** — all dep-min PRs merged; next from the native-collision WIP branches.
- **Largest remaining win (started, #3123 merged):** native-collision port → makes
  FCL/Bullet/ODE optional and eventually drops `fcl` from core. The default-flip
  (the actual dependency drop) is still ahead, via the WIP branches' follow-up PRs.
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
