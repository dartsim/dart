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
| **#3116** | native-replacement | Remove GLUT GUI stack | mergeable, **1 behind**, CI pending — _key dep-min PR; also enables lodepng drop_ |
| **#3120** | dependency-reduction/CI | Stop coverage job double-running tests | mergeable, **1 behind**, CI pending |
| **#3118** | perf | Inverse-dynamics profiling driver | mergeable, **1 behind**, CI pending |
| **#3092** | feature (not dep-min) | ssik analytical IK | mergeable, **1 behind**, CI pending |

### 🛠️ Active branches without a PR yet (native-collision-port lane)
- `feature/native-occupancy-grid` — tip _"Checkpoint native occupancy grid performance"_
- `task/native-collision-performance-exec` — tip _"Improve native collision primitive hot paths"_
- _Monitor for PR creation; this is the FCL/Bullet/ODE-reduction lever (the largest dependency win)._

## Coordination flags / blockers

1. **Open PRs are 1 commit behind `release-6.20`** (after #3119): #3116/#3118/#3120/#3092
   each need `release-6.20` merged up before merge. All owned by the maintainer.
2. **Shared hot files:** `pixi.toml` / `pixi.lock` are touched by multiple lanes —
   **merge `origin/release-6.20` before pushing**, never rebase a published PR branch
   (per `AGENTS.md` / `02`).
3. **CI runner backlog:** GitHub Actions jobs have been queueing for hours; PRs sit
   `BLOCKED` on pending (not failing). Factor into merge-timing expectations.
4. **Native-collision lane has WIP branches but no PR** — no gz-compat / parity /
   perf evidence visible yet. Watch for the PR; its scope/requirements are in `03`.

## Effort-level status

- **Dependency footprint removed so far:** optimizer backends (ipopt/nlopt/pagmo/snopt)
  + the four vendored `dart/external` trees (convhull/ikfast/imgui/odelcpsolver).
- **In flight:** GLUT removal (#3116) → also drops lodepng + GLUT/Xi/Xmu/freeglut.
- **Largest remaining win (not yet PR'd):** native-collision port → makes
  FCL/Bullet/ODE optional and drops `fcl` from core.
- **Confirmed non-removable standalone:** `boost` (OSG-coupled), core deps
  (Eigen/assimp/fmt/tinyxml2/urdfdom), `octomap` (exported-header contract).

## How this board is maintained

- Refresh on each orchestration pass: `gh pr list` (open + recently merged),
  per-PR `mergeable`/`mergeStateStatus`/checks, and `git ls-remote` for new
  lane branches.
- Flag: PRs gone red, PRs fallen behind `release-6.20`, new PRs from any lane,
  shared-file collisions, and the native-collision PR when it appears.
- This lane does **not** execute other lanes' code — it tracks, flags, and
  coordinates (parallel, non-overlapping).
