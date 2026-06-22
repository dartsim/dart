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
| **Native-replacement lane** | `dart/external/*` → native built-ins; **GUI/OSG + GLUT removal** | External replacements **done**; GLUT removal **in flight** (#3116) |
| **Native-collision-port lane** | Port DART 7 `dart/collision/native/` → DART 6.20 (make FCL/Bullet/ODE optional) | **In progress** (WIP branches, no PR yet) |
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

### ✅ Merged — GUI/OSG planning (native-replacement lane)
- **#3113** GLUT-to-OSG removal plan

### ✅ Merged — perf/parallelism + enabling fixes (issue #3056)
- **#3071** opt-in simulation threading · **#3085** plane/contact-cap collision ·
  **#3086** resting-world deactivation · **#3089** contact benchmark example ·
  **#3111** constrain unsafe parallel island solves · **#3112** accelerate settled worlds
- **#3114** FCL null-contact fix (unblocked the coverage/Asserts CI regression)

### ✅ Merged — hygiene
- **#3102** ignore `.omo` · **#3100** pixi lockfile · **#3074/#3075** plan + AI workflows import

### 🔄 Open — monitoring
| PR | Lane | Title | Health |
| --- | --- | --- | --- |
| **#3116** | native-replacement | Remove GLUT GUI stack | mergeable, **caught up** with `release-6.20`, CI queued — _key dep-min PR; also enables lodepng drop_ |
| **#3118** | perf | Inverse-dynamics profiling driver | mergeable, **caught up**, CI queued |
| **#3120** | dependency-reduction/CI | Stop coverage job double-running tests | ⚠️ **`CONFLICTING` (real)** — conflict is **`CHANGELOG.md` only** (`pixi.toml` auto-merges); resolve by merging `release-6.20` up and keeping both changelog entries |
| **#3122** | code-footprint | Remove legacy `dart/integration` module (dead code + installed headers) | GitHub shows `CONFLICTING` but it's **stale** — `merge-tree --write-tree` is clean and the branch is 0 behind; a no-op nudge/recompute clears it. Milestone: 6.20.0 |

_(This board itself is PR #3121 — intentionally **not** listed above, so the
committed copy is not permanently stale once it merges. #3092 "ssik analytical IK",
a non-dep-min feature, merged 2026-06-22 00:04 and is no longer tracked.)_

### 🛠️ Active branches without a PR yet (native-collision-port lane)
- `feature/native-occupancy-grid` — tip _"Checkpoint native occupancy grid performance"_
- `task/native-collision-performance-exec` — tip _"Improve native collision primitive hot paths"_
- _Monitor for PR creation; this is the FCL/Bullet/ODE-reduction lever (the largest dependency win)._

## Coordination flags / blockers

1. **Base catch-up:** #3116/#3118 are now caught up with `release-6.20` (CI-queued
   only). **#3120 is `CONFLICTING`**, but narrowly — the conflict is in
   `CHANGELOG.md` only (`pixi.toml` auto-merges); a `release-6.20` merge-up that
   keeps both changelog entries resolves it. **#3122's `CONFLICTING` is stale**,
   by contrast (`merge-tree` clean, 0 behind) — it needs only a recompute, not
   resolution. All owned by the maintainer.
   _(Latest base merge: #3092 ssik IK → `release-6.20` HEAD `6bca946b40`.)_
2. **Shared hot files:** `pixi.toml` / `pixi.lock` are touched by multiple lanes —
   **merge `origin/release-6.20` before pushing**, never rebase a published PR branch
   (per `AGENTS.md` / `02`).
3. **CI health (backlog now draining):**
   - **`coverage` is failing on open PRs** (e.g. #3118) — a recurring coverage-job
     issue, **not** content-specific (it also appears on docs-only PRs). **#3120
     directly targets this job** ("stop coverage running tests twice"), so
     prioritizing #3120 (its only conflict is the trivial `CHANGELOG.md` one in
     flag 1) likely improves CI health effort-wide.
   - **`Debug` / `API Documentation` show flaky failures** on some runs while
     **passing** on sibling PRs with the same base — re-trigger to clear; not a
     base regression. (Coverage logs also show benign `Cannot generate a safe
     runtime search path` RPATH warnings across pre-existing targets.)
4. **Native-collision lane has WIP branches but no PR** — no gz-compat / parity /
   perf evidence visible yet. Watch for the PR; its scope/requirements are in `03`.

## Effort-level status

- **Removed so far:**
  - _Package/pixi dependencies removed:_ optimizer backends (ipopt/nlopt/pagmo/snopt).
  - _Vendored `dart/external` source replaced with native/DART-owned code:_
    convhull_3d (→ native math), ikfast (→ `dart/dynamics` header), odelcpsolver
    (→ native Dantzig).
  - _Not removed — rehomed:_ ImGui's vendored tree was swapped for a
    FetchContent/system target (#3081), but `imgui` is **still a declared
    dependency**; it folds into the GUI/OSG work, not the removed set.
- **In flight:** GLUT removal (#3116) → also drops lodepng + GLUT/Xi/Xmu/freeglut.
- **Largest remaining win (not yet PR'd):** native-collision port → makes
  FCL/Bullet/ODE optional and drops `fcl` from core.
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
