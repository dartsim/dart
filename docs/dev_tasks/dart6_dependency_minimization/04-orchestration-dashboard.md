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
>   names (per [[feedback-no-local-dir-names-in-repo]] convention; see the naming
>   note in `02`).
>
> _Last updated: 2026-06-22._

## Lanes & owners

| Lane (work scope) | Charter | State |
| --- | --- | --- |
| **Dependency-reduction lane** (this one) | Optimizer removal; default-env analysis; **now orchestration/monitoring** | Own removals **complete**; running this board |
| **Native-replacement lane** | `dart/external/*` → native built-ins; **GUI/OSG + GLUT removal** | External replacements + **GLUT/lodepng removal done** (#3116 merged) |
| **Native-collision-port lane** | Port DART 7 `dart/collision/native/` → DART 6.20 (make FCL/Bullet/ODE optional) | **In progress — first PR open (#3123)**, + WIP branches |
| **Perf / parallelism lane** (issue #3056) | Island deactivation, parallel-safe solves, benchmarks | Largely merged; one PR open (#3118) |

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
  **#3111** constrain unsafe parallel island solves · **#3112** accelerate settled worlds
- **#3114** FCL null-contact fix (unblocked the coverage/Asserts CI regression)

### ✅ Merged — CI / hygiene
- **#3120** stop coverage job double-running tests (merged 2026-06-22) — the coverage-job fix
- **#3102** ignore `.omo` · **#3100** pixi lockfile · **#3074/#3075** plan + AI workflows import

### 🔄 Open — monitoring
| PR | Lane | Title | Health |
| --- | --- | --- | --- |
| **#3123** | **native-collision-port** | Speed up DART primitive plane collision | **first native-collision PR** — dependency-free primitive plane contacts + finite-shape broadphase pruning (branch `perf/dart6-native-collision`). `MERGEABLE`, CI pending. Milestone 6.20.0. _Verify vs `03` gz-compat/parity/perf bar._ |
| **#3122** | code-footprint | Remove legacy `dart/integration` module (dead code + installed headers) | ✅ conflict **resolved** (GLUT-example deletion accepted) — now `MERGEABLE`, **1 behind**, CI pending. Milestone 6.20.0 |
| **#3118** | perf | Inverse-dynamics profiling driver | mergeable, **1 behind**, CI pending |

_(This board itself is PR #3121 — intentionally **not** listed above, so the
committed copy is not permanently stale once it merges. #3092 "ssik analytical IK",
a non-dep-min feature, merged 2026-06-22 00:04 and is no longer tracked.)_

### 🛠️ Native-collision-port lane (the largest dependency lever — FCL/Bullet/ODE)
- **#3123 is its first PR** (see Open table) — primitive plane contacts + broadphase pruning.
- WIP branches (no PR yet): `feature/native-occupancy-grid`, `task/native-collision-performance-exec`.
- _As these land, hold them to `03`'s bar: gz-compat (`pixi run -e gazebo test-gz`), feature/contact parity, and evidence-driven perf ≥ Bullet/ODE/FCL._

## Coordination flags / blockers

1. **Base / conflict status** (base now `9b0c68a5a82`, after #3116 + #3120 merged):
   - **#3122** — `modify/delete` conflict **resolved** (the GLUT-example deletion
     was accepted, as diagnosed); now `MERGEABLE`, 1 behind → simple merge-up.
   - **#3118** — 1 behind; simple merge-up.
   - All owned by the maintainer.
2. **Shared hot files:** `pixi.toml` / `pixi.lock` are touched by multiple lanes —
   **merge `origin/release-6.20` before pushing**, never rebase a published PR branch
   (per `AGENTS.md` / `02`).
3. **CI health (backlog now draining):**
   - **`coverage`** had a recurring failure; **#3120 — the coverage-job fix —
     is now MERGED** (2026-06-22), so coverage CI should improve on subsequent
     runs. Watch the next coverage results to confirm.
   - **`Debug` / `API Documentation` show flaky failures** on some runs while
     **passing** on sibling PRs with the same base — re-trigger to clear; not a
     base regression. (Coverage logs also show benign `Cannot generate a safe
     runtime search path` RPATH warnings across pre-existing targets.)
4. **Native-collision lane: first PR is open (#3123)** — "dependency-free primitive
   plane contacts + finite-shape broadphase pruning". The largest dependency lever
   is now in flight; hold it (and follow-ups) to `03`'s gz-compat / parity / perf
   bar. WIP branches `feature/native-occupancy-grid` + `task/native-collision-performance-exec` continue.

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
- **In flight:** native-collision port — first PR **#3123** (primitive plane contacts +
  broadphase pruning); legacy `dart/integration` dead-code removal (#3122, mergeable).
- **Largest remaining win (now in flight, #3123):** native-collision port → makes
  FCL/Bullet/ODE optional and eventually drops `fcl` from core.
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
