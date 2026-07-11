# Orchestration dashboard — DART 6 performance generalization

Status board only; packet definitions live in the lane docs. Update this
file (and RESUME.md) in every PR that claims or completes a packet.

Branch point: `origin/release-6.20` @ `70b92010311` (2026-07-04 plan
branch point).
Guard baseline: `origin/release-6.20` @ `b9e6910c066` (2026-07-05
WP-PG.01 current-base guard refresh).
Related open queue at last refresh: none blocking (enablers merged:
#3209, #3226, #3229, #3230, #3234, #3281). WP-PG.40 standalone PR
#3270 was closed by maintainer direction and its D1/D2 evidence now rides
with the first real SIMD-kernel PR.

Current handoff (2026-07-10): the completion audit ran (see RESUME.md —
criteria 1-3 MET on the merged head). The maintainer broadened criterion 4 to
cross-engine evidence vs MuJoCo across DART's major workloads; lane **WS-G**
(08-mujoco-comparison-lane.md) owns that work. #3366 (dartpy getDofs ownership
bugfix) and #3367 (MuJoCo comparison harness + mujoco env + dartpy native
binding) have merged. Open PRs from the 2026-07-10 cycle: #3368 (native
AABB-tree broadphase, dep-min lane) and #3369 (MJCF stacked hinge/slide joint
support). In-flight packets: MJCF contype/conaffinity+friction fidelity and
native small-scene overhead (WP-SS family); the S6 native resting-profile row
is resolved by the dep-min lane's documented acceptance re-scope.

## Lane status

| Lane | Owner doc | Packets | Status |
| --- | --- | --- | --- |
| WS-A constraint/LCP | 02-constraint-lcp-lane.md | PG.10–PG.15 | gated (PG.10 #3339; PG.13 evidence-gated by PG.10 census; PG.14 done #3361 as D3 opt-in matrix-free path; PG.15 done #3353) |
| WS-B ODE backend | 03-ode-backend-lane.md | PG.20–PG.23 | gated (PG.20 #3329; PG.21 current-base gate rejected after span-only PG.20; PG.22 local cpp-only route rejected; PG.23 blocked D8; lane re-review at WS-F phase 5) |
| WS-C dynamics batching | 04-dynamics-batching-lane.md | PG.30–PG.33 | gated (PG.30 #3310; PG.31 #3341; PG.32 delivered by #3297/#3307; PG.33 gated) |
| WS-D SIMD enablement | 05-simd-enablement-lane.md | PG.40–PG.42 | active (PG.40 folded into PG.42; PG.41 waits for PG.10 seam evidence) |
| WS-E infra/evidence | 06-infra-evidence-lane.md | PG.01–PG.04 | open (PG.01 done; PG.02 #3327; PG.03 #3337; PG.04 blocked D4) |
| WS-F native collision port | ../dart6_dependency_minimization/03-native-collision-port-scoping.md | phases 0–7 | external owner; phases 0–3 complete (native adapter + capability parity); phase 4 active after #3364, with AABB-tree broadphase in open PR #3368 |

## Packet board

| Packet | Lane | Status | Branch / PR | Evidence |
| --- | --- | --- | --- | --- |
| WP-PG.01 baseline packet | WS-E | done — #3263 | `wp-pg-01-baseline-evidence` / #3263 | 01-baseline-evidence.md (S1–S6 guard rows, profile splits, prior-art triage) |
| WP-PG.02 benchmark matrix | WS-E | done — #3327 | `wp-pg-02-contact-container-matrix` / #3327 | Active rows now cover DART/ODE/FCL/Bullet at 60/120 objects plus 4-thread sweep; bounded DART/ODE deactivation rows are in the dashboard filter; 900 dense-container rows are registered for manual filters but excluded from the default dashboard slice after local budget smoke |
| WP-PG.03 profiling doc | WS-E | done — #3337 | `wp-pg-03-profiling-doc` / #3337 | Adds `docs/onboarding/profiling.md` for the DART 6.20 text profiler and Tracy workflow, a profile-env `config-tracy` task, and the Tracy callstack compatibility fix required by the packaged dependency; merged 2026-07-07 |
| WP-PG.04 executor tooling | WS-E | blocked (D4) | — | — |
| WP-PG.10 LCP instrumentation | WS-A | done — #3339 | `wp-pg-10-lcp-profile-census` / #3339 | Adds runtime-gated text-profiler counters for constrained-group island census plus solver/LCP stage scopes. Local profile artifact `/tmp/wp_pg10_profile_20260707T132241`: S1 dense container is solve-proper dominated (primary solve 90.0%, construct 5.69%, one 816-row island); S3 active 3k has 3003 islands / 15015 rows with build 5.44%, construct 0.89%, primary solve 1.02%; S4/S5 generated scenes have 900/90 islands and max 12-row groups. Final local gates passed lint/check-lint, profile smoke, allocation gate, capped `ALL`, and `test-gz`; merged 2026-07-08 |
| WP-PG.11 solver RTTI removal | WS-A | evidence-gated (current-base rejected) | — | Local 2026-07-06 cpp-only mining rejected after refreshed current-base A/B on `2e11928288c`: S2 ODE 0.87x, S4 DART 0.93x, S4 Bullet 0.99x, S4 ODE 0.98x medians; guards identical |
| WP-PG.12 direct assembly | WS-A | deprioritized (PG.01 evidence: assembly ≤ ~8%) | — | — |
| WP-PG.13 row islanding | WS-A | evidence-gated (PG.10 census) | — | — |
| WP-PG.14 matrix-free path | WS-A | done — #3361 | `wp-pg-14-matrix-free-lcp` / #3361 | Merged as `91c158fc3e5` after clean Codex review on `5751c7ed84c`. Adds default-off matrix-free contact PGS for supported large single-free-body contact islands, prepared scratch reuse, cached-impulse residual seeding, dense fallback on non-convergence, mixed per-DoF FreeJoint actuator rejection, C++/dartpy options, benchmark flags, and profiler counters. Full artifact `/tmp/wp_pg14_matrix_free_ab_20260709T040443Z`: option-off S1 120 DART median avg-step 7.404 ms, hash `0x123ee9779bccacfb`; option-on 30-iter matrix-free median avg-step 0.663 ms, finite, hash `0xbf538ac9d35f145e`; option-on S3 active-3k fallback preserved hash `0xcf0ba6eaa97be038`. Final current-head smoke `/tmp/wp_pg14_matrix_free_review_5751c7ed84c_repeat_20260709T223525Z`: dense 7.93478 ms, matrix-free 1.38782 ms, finite. Local validation passed lint/check-lint, full `test_ConstraintSolver`, dartpy constraint pytest, capped builds, and `test-gz`; hosted Codecov patch/project passed before merge. |
| WP-PG.15 creep vs rest-veto | WS-A | done — #3353 | `docs/close-dart6-performance-generalization` / #3353 | Restores the active tracker after mistaken retirement, promotes the evidenced D7 policy into defaults with adaptive ERV (`0.1` only for dense mobile-mobile islands, legacy `0.001` effective cap for single-mobile static-support islands), dense-island rest tolerance (`0.005` only under the default solver rest-veto policy), dense-contact-island sleep candidacy for sub-wake jitter, `contact_benchmark` old/new override knobs, and focused sleeping/CI tests. Explicit `1e-5` sleep-tolerance overrides still preserve the strict legacy rest-veto policy, and `resetMaxErrorReductionVelocity()` restores the adaptive contact ERV policy after temporary overrides. Local S6 refresh `/tmp/wp_pg15_ab_plane_fallback_20260709T023141Z`: S6 old-default override took 212.08 s, RTF 0.0943, 162 contacts, max penetration 0.364241, 0/71 resting; current defaults took 91.9572 s, RTF 0.217, zero contacts, max penetration 0, 71/71 resting. S4/S5 new-default hashes matched old-default rows across DART/FCL/Bullet/ODE in `/tmp/wp_pg15_ab_review_20260708T235540Z`. GUI/final-scene evidence: `/tmp/wp_pg15_gui_20260708T223653Z/S6_gui.png`, `/tmp/wp_pg15_visual_20260708T223506Z/S6_final_scene.jsonl`. Merged after local validation and clean Codex review. |
| WP-PG.20 history spans | WS-B | done — #3329 | `wp-pg-20-ode-history-spans` / #3329 | Current-base refresh @ `9ff8b1d77a1`: hashes bit-identical; ODE rows improved (`S2_ode` 0.0933→0.0521 ms, `S3_ode` 115.9→19.7 ms, `S4_ode` 0.2269→0.1549 ms; GB ODE rows 3.2–7.6% faster) |
| WP-PG.21 history map/pruning | WS-B | evidence-gated (current-base rejected) | — | 2026-07-07 gate on `b78a8b8cbe7`: hashes identical but mixed/regressive vs span-only (`S2_ode` +12.6%, `S3_ode` -11.4%, `S4_ode` +19.2%; GB 60 rows faster, 120 rows +6.8-9.4% slower), so do not claim without new profile evidence |
| WP-PG.22 version-gated pose push | WS-B | evidence-gated (current-base rejected) | — | Local 2026-07-06 exact-transform fallback rejected by A/B; protected `Skeleton::getKinematicVersion()` blocks cpp-only route |
| WP-PG.23 ODE manifold reduction | WS-B | blocked (D8) | — | — |
| WP-PG.30 free-body cache + FD path | WS-C | done — PR #3310 | `wp-pg-30-single-free-body-cache` | A/B: S5 −12.2%, S4 −5.3%, S3 −2.3%, solve-bound rows flat; 8/8 guard hashes bit-identical |
| WP-PG.31 shallow-support scratch | WS-C | done — #3341 | `wp-pg-31-shallow-support-scratch` / #3341 | Current-base A/B artifact `/tmp/wp_pg31_ab_20260707T184319` (`3964108a675` -> `21f691311df`): no-root-FreeJoint `double_pendulum.world` hashes identical and median step time improved 0.002106 -> 0.001836 ms (DART), 0.001903 -> 0.001644 ms (ODE); generated 120-object DART/ODE guard hashes identical; ODE `BM_ContactContainerActive/120/1/{1,16}` medians 6588 -> 6433 ms and 6998 -> 6468 ms; base and branch both passed 30/30 default 16-thread crash stressors; merged 2026-07-08 |
| WP-PG.32 frame arena + alloc gate | WS-C | done — #3297/#3307 | `wp-pg-32-frame-allocation-gate` (tracker reconciliation) | Merged #3297 added `FrameAllocator`, `FrameStlAllocator`, World-owned `MemoryManager` preparation, solver/profiler/Dantzig scratch reuse, and `INTEGRATION_StepAllocation` allocation-counting gates. Merged #3307 extended the allocation discipline to soft/deformable paths and recorded the #3307-style performance report: strict zero-regression checker PASS on `.benchmark_results/pr3307-bafbd4b-full-parent-base/summary.json`, native DART winning every checksum-equivalent soft-scene/thread row against FCL, and strict soft allocation gates reporting zero `operator new`, zero raw `malloc`, and zero counted allocator growth where available. Local reconciliation-branch verification passed `pixi run test-eigen-overalignment` (148/148), targeted Release allocator build, and `ctest -R '(StepAllocation|FrameAllocator|MemoryManager)'` (3/3) |
| WP-PG.33 SoA integration | WS-C | gated | — | — |
| WP-PG.40 FP/ISA contracts | WS-D | folded into WP-PG.42 | #3270 closed | maintainer direction: carry D1/D2 evidence with actual SIMD kernel PR |
| WP-PG.41 batch math seam | WS-D | blocked (PG.10 seam evidence) | — | — |
| WP-PG.42 SoA broadphase | WS-D | done — PR #3299 | `wp-pg-42-soa-broadphase-simd` | AVX-width finite sweep SIMD screen, scalar/SSE/NEON fallback, SIMD CI consumer coverage, contact-container macro rows, finite-finite profile scope |

Claim flow: set the packet row to `claimed — <who/session>` with the
`wp-pg-<nn>-<slug>` branch name, update RESUME.md, and open the packet PR
titled `WP-PG.<nn>: <title>`. On completion set `done — <PR#>` and add the
evidence link. Author never self-approves; an independent reviewer accepts
against the packet's acceptance evidence.

## Cross-lane coordination notes

- WS-F consumes WS-D kernels in its phase 4. Phases 0–3 are complete, including
  the DART 6 native detector adapter and capability parity; phase 4 is active
  after #3364. Open PR #3368 carries the branch-local AABB-tree broadphase and
  must merge before that result is treated as base-branch evidence.
- WS-B investment is re-reviewed when WS-F reaches phase 5 (facade
  decision) — see D5 in the README.
- WS-A WP-PG.12 and WS-C WP-PG.30 share the single-free-body
  classification; land WP-PG.30 first when possible.
- Deactivation-gate work (#3226) is merged; any packet touching sleeping
  behavior must include deactivation counters in its evidence table.
- Six round-1 experiment branches live on origin (`perf/dart6-*`, pushed
  2026-07-03, 21–28 commits behind); prior-art inventory + rejected list
  in 01-baseline-evidence.md. WP-PG.01 triages them; packets overlapping
  them (PG.30, PG.42) must read the inventory before claiming.
