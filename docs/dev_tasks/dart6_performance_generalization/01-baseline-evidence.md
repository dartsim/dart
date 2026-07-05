# Baseline evidence (round 2)

Owner doc for the round-2 measurement protocol, the canonical guard-scene
commands, and the captured baselines. The durable, commit-comparable
capture path is the #3230 dashboard tooling; tables here are the guard
reference every later PR must match cell-for-cell.

## Prerequisites

```bash
pixi run config
pixi run cmake --build build/default/cpp/Release \
  --target contact_benchmark BM_INTEGRATION_contact_container --parallel 8
# Scenes S2/S3 need the gz-sim checkout (pinned tag) under .deps/gz-sim:
pixi run -e gazebo download-gz-sim
```

Record for every capture: binary's commit SHA, compiler, CPU model +
governor state, pixi environment, exact command lines, which optional
detectors were built. Known pitfalls: Google Benchmark filters are
prefix-matched (anchor with `$`); stale build trees may predate #3209;
host CPU scaling adds noise; use `--parallel 8` builds to avoid
host-contention OOM.

## Canonical guard scenes (exact commands)

`CB=./build/default/cpp/Release/bin/contact_benchmark`; `<det>` ranges
over `dart fcl bullet ode` unless a row pins it. **ODE rows are only
valid with `--max-contacts-per-pair 4`** (#3209 finding 3: the trimesh
cylinder fallback floods larger caps and silently drops contacts).

- **S1 — active container (primary fixture)**:
  `BM_INTEGRATION_contact_container` full suite,
  `--benchmark_repetitions=3 --benchmark_report_aggregates_only=true`
  (engines dart+ode × threads 1/16 × 60/120 objects; deactivation
  disabled by benchmark design).
- **S2 — settled 3k (sleeping regime)**:
  `pixi run $CB .deps/gz-sim/examples/worlds/3k_shapes.sdf --steps 3000
  --sdf-plane-shapes --quiet --checkpoint 0 --collision <det>
  --world-threads 1 --max-contacts 12000 --max-contacts-per-pair 4`
  (deactivation default ON). Round-1 guardrail: native RTF ≈ 81, hash
  `0x131b6af79a44ff90`, 3003/3003 resting.
- **S3 — active 3k**:
  same world, `--steps 300 --disable-deactivation --world-threads 16
  --max-contacts 12000 --max-contacts-per-pair 4 --quiet --checkpoint 0
  --sdf-plane-shapes --collision <det>`.
- **S4 — generated 900**:
  `pixi run $CB --generate-objects 900 --steps 300 --warmup 0
  --checkpoint 0 --quiet --collision <det> --world-threads 16
  --max-contacts 20000 --max-contacts-per-pair 4`.
- **S5 — serial 90**:
  `pixi run $CB --generate-objects 90 --steps 300 --warmup 0
  --checkpoint 0 --quiet --collision <det> --world-threads 1
  --max-contacts 20000 --max-contacts-per-pair 4`.
- **S6 — penetration-creep reproducer** (#3209 finding 2):
  `pixi run $CB --generate-container 71 --steps 20000 --checkpoint 5000
  --collision dart` — watch `max_penetration` (grows ~5–10 mm/s at
  baseline) and `resting` (stays 0: the ≤ 1e-5 m island-rest veto never
  passes while creeping). This is WP-PG.15's acceptance fixture.

Cell matrix for WP-PG.01 (and for any packet's evidence table — rows a
packet does not affect may cite the baseline instead of re-running):

| Scene | Detectors | Threads | Deactivation | Extra |
| --- | --- | --- | --- | --- |
| S1 | dart, ode (benchmark-fixed) | 1, 16 (fixed) | off (fixed) | 60 + 120 objects |
| S2 | dart, fcl, bullet, ode | 1 | on | `--dump-final-scene` per det |
| S3 | dart, fcl, bullet, ode | 16 | off | — |
| S4 | dart, fcl, bullet, ode | 16 | on | — |
| S5 | dart, fcl, bullet, ode | 1 | on | — |
| S6 | dart | 1 | on | checkpoints at 5000 steps |

Each cell reports: RTF; resting N/M; contacts (+ cap-hit flag);
final-state hash; finite-state flag; S6 additionally max_penetration per
checkpoint. Untouched detectors must keep bit-identical hashes across
compared commits. Every performance PR carries the 3-way comparison —
recorded baseline SHA, immediate parent, PR head — for the affected rows
plus the untouched-detector guard rows.

Profile shares: append `--profile` (text profiler; `DART_BUILD_PROFILE=ON`
in the pixi config). WP-PG.01 must record the `Construct LCP` vs
Dantzig-solve-proper split per scene — the round-2 smoke number (below)
did not separate them.

## Round-2 baseline (WP-PG.01, refreshed 2026-07-05)

Metadata: guard rows refreshed on `origin/release-6.20` @
`b9e6910c066` (PR head `13d0614dabb`); GCC 15.2.0 (Ubuntu), Release;
Intel i9-13950HX (32 threads), governor **powersave**, CPU scaling
ENABLED — RTF values are host-relative (treat ±5% as noise, and more
under visible host load); **guard values are the hashes, contact/pair
counts, resting counts, and finite flags**, not the RTF cells. The
2026-07-04 `5bee91ad6be` capture generated the original dashboard
artifacts and profile splits; the 2026-07-05 current-base refresh below
updates the durable guard rows after #3273 and later release-branch
merges. The pre-plan probe on `fdf89784e8d` matched the S1 rows within
noise, so the fixture is stable across #3226/#3230/#3241.

### S1 — `BM_INTEGRATION_contact_container` (3-rep means)

| Scenario (objects/engine/threads) | cpu ms/iter | sim_s/s (≈RTF) | Contacts (cap) | Pairs | Resting | Finite | Hash |
| --- | ---: | ---: | ---: | ---: | --- | --- | --- |
| 60 / dart / 1 | 466 | 0.429 | 96 (false) | 87 | 0/60 | true | `0xbe5900b3cf874742` |
| 60 / dart / 16 | 460 | 0.435 | 96 (false) | 87 | 0/60 | true | `0xbe5900b3cf874742` |
| 60 / ode / 1 | 1789 | 0.112 | 249 (false) | 86 | 0/60 | true | `0x54680005ed76f66` |
| 60 / ode / 16 | 1783 | 0.112 | 249 (false) | 86 | 0/60 | true | `0x54680005ed76f66` |
| 120 / dart / 1 | 5552 | 0.036 | 272 (false) | 204 | 0/120 | true | `0x2757590b13e917ee` |
| 120 / dart / 16 | 5592 | 0.036 | 272 (false) | 204 | 0/120 | true | `0x2757590b13e917ee` |
| 120 / ode / 1 | 9956 | 0.021 | 542 (false) | 200 | 0/120 | true | `0x7dd44240329b6f4f` |
| 120 / ode / 16 | 8792 | 0.023 | 542 (false) | 200 | 0/120 | true | `0x7dd44240329b6f4f` |

60→120 objects costs ~11.9x (native) — super-quadratic scaling; threads
do not provide reliable speedup in either engine; ODE emits ~2x the
contacts of native
(physically different profiles; hashes only comparable within one
detector). The Google Benchmark suite above reports timing/contacts only;
it does not print pairs/finite/hash, so the S1 hash/pair/finite guard
columns come from separate scripted CLI captures, one per table row
(`{60, 120} objects × {dart, ode} × {1, 16} threads`; 8 captures total):

```bash
pixi run $CB --generate-container <60|120> --steps 200 --checkpoint 0 \
  --collision <dart|ode> --disable-deactivation --world-threads <1|16> \
  --max-contacts 20000 --max-contacts-per-pair 4 --quiet
```

`--disable-deactivation` mirrors the benchmark's fixed deactivation-off
design; `--max-contacts 20000 --max-contacts-per-pair 4` keeps every S1
cell's cap-hit flag `false` (measured contacts top out at 542) and matches
the ODE trimesh-cap caveat above. The independent 16-thread CLI captures
match the 1-thread hash/pair/finite values for each object-count/detector
pair, so the duplicate hashes in the table are measured equivalence rather
than a reused 1-thread digest. Timing columns remain the 3-rep Google
Benchmark means from the suite invocation, independent of this CLI capture.

### S2–S6 guard rows (canonical commands above)

| Row | RTF | Avg step ms | Contacts (cap) | Pairs | Resting | Finite | Max pen | Hash |
| --- | ---: | ---: | ---: | ---: | --- | --- | ---: | --- |
| S2_dart | 29.46 | 0.034 | 0 (false) | 0 | 3003/3003 | true | 0 | `0x8ddc9a81f2d28a7f` |
| S2_fcl | 30.60 | 0.033 | 0 (false) | 0 | 3003/3003 | true | 0 | `0x266da31836a314a6` |
| S2_bullet | 8.25 | 0.121 | 0 (false) | 0 | 3003/3003 | true | 0 | `0x2375f1927218cd43` |
| S2_ode | 13.51 | 0.074 | 0 (false) | 0 | 3003/3003 | true | 0 | `0x10f80b0408cede90` |
| S3_dart | 0.115 | 8.68 | 5005 (false) | 3003 | 0/3003 | true | 9.3e-09 | `0xcf0ba6eaa97be038` |
| S3_fcl | 0.102 | 9.79 | 3003 (false) | 3003 | 0/3003 | true | 9.3e-09 | `0x6088ea0177efa6a` |
| S3_bullet | 0.096 | 10.43 | 5005 (false) | 3003 | 0/3003 | true | 3.0e-07 | `0x22e27960cbabe83e` |
| S3_ode | 0.0092 | 108.19 | 9009 (false) | 3003 | 0/3003 | true | 9.3e-09 | `0x4904c09a93a36442` |
| S4_dart | 0.400 | 2.50 | 1800 (false) | 900 | 600/900 | true | 1.0e-03 | `0x76205ad68f4293bb` |
| S4_fcl | 0.304 | 3.29 | 1800 (false) | 900 | 450/900 | true | 9.8e-04 | `0x7a0974e837912472` |
| S4_bullet | 0.175 | 5.71 | 2353 (false) | 900 | 269/900 | true | 1.1e-02 | `0x2a5577952e2de925` |
| S4_ode | 6.72 | 0.149 | 0 (false) | 0 | 900/900 | true | 0 | `0x429b65bc5c4a14b6` |
| S5_dart | 4.21 | 0.238 | 180 (false) | 90 | 60/90 | true | 1.0e-03 | `0x726d1ff51bdb717` |
| S5_fcl | 2.83 | 0.353 | 180 (false) | 90 | 45/90 | true | 7.6e-04 | `0x99bfaef49c254203` |
| S5_bullet | 2.56 | 0.390 | 210 (false) | 90 | 56/90 | true | 1.0e-05 | `0xf78e3bd075780c83` |
| S5_ode | 87.57 | 0.011 | 0 (false) | 0 | 90/90 | true | 0 | `0x5f2afc7230ee8d10` |
| S6_dart | 0.0939 | 10.65 | 160 (false) | 139 | 0/71 | true | **0.3624** | `0x6eb6ff3911ac9d04` |

### Headline findings (round-2 evidence; supersedes the pre-plan smoke run)

1. **The two active regimes have opposite bottlenecks** (profile splits,
   serial, native detector):
   - **P1, dense pile** (container 120, one big island):
     `DantzigBoxedLcpSolver::solve` (solve-proper) = **88.1%** of step
     time; `Construct LCP` (assembly incl. unit-impulse tests) = only
     **7.4%** (unit-impulse tests 0.74% — #3142's direct path visible).
     Consequence: WP-PG.12's assembly optimization has ≤ ~8% headroom on
     the primary fixture; **the D3 revisit trigger has FIRED** — only a
     solve-side change (WP-PG.14) or making piles sleep (WP-PG.15/D7)
     moves the dense-pile fixture materially.
   - **P2, many small islands** (active 3k, 3003 one-body islands):
     **integration = 50.1%** of step time (positions 27.9% + velocity
     22.2%); per-island constraint machinery 21.9% (Construct LCP 10.9%
     at ~1 µs/island — per-island overhead, solve-proper only 3.7%);
     collision 7.8%. Consequence: WS-C batching (WP-PG.30/33) and
     per-island overhead trimming (WP-PG.11 + mined single-reactive
     commits) are the levers for the many-bodies regime.
2. **ODE active-3k is ~12x slower than native** (RTF 0.0092 vs 0.115,
   108 ms/step) — the WS-B lane is worth more than the settled-scene
   2.4x gap suggested.
3. **ODE remains physically divergent** on generated scenes: S4/S5 end
   with 0 contacts and everything resting (vs native 600/900 resting,
   1800 contacts) — RTF-only comparisons across detectors stay banned.
4. **Creep confirmed on the round-2 tip** (S6): max penetration grows to
   0.362 m over 20 s (~18 mm/s), resting stays 0/71 — the WP-PG.15/D7
   fixture behaves exactly as #3209 documented.
5. Success criterion 1 (3x on the dense-pile fixture via default-on work)
   is **not reachable from assembly/build work alone** given finding 1a;
   it needs D3 (solve-side) and/or D7 (sleep the pile) — flagged for the
   maintainer alongside those decisions.

## Prior art — round-1 experiment branches (read before claiming packets)

Six unpushed round-1 experiment branches were published to origin on
2026-07-03 for reference (tips 21–28 commits behind current
`release-6.20`; none patch-equivalent to merged work; the deepest carries
the round-1 experiment journal):

Triage (WP-PG.01, 2026-07-04; each branch has exactly one unique code
commit beyond shared/journal commits):

| Branch (origin) | Unique change | Triage verdict |
| --- | --- | --- |
| `perf/dart6-broadphase-cache-refresh` | Move shape-cache refresh into broadphase entry build (clean rebuilt commit + regression test) | **Mine for WP-PG.42** — never benchmarked; collision is only ~8% of active step time, measure first |
| `perf/dart6-lazy-dart-shape-cache` | Same refresh idea (byte-identical subset of four sibling branches) | **Delete** — measured tie vs parent (RTF 0.1976 vs 0.1894 fresh; 0.1333 vs 0.1332 throttled); journal marks it local-experiment-only |
| `perf/dart6-broadphase-cache-refresh-order` | Refresh reordering + 60-line stale-shape-cache regression test | **Mine for WP-PG.42** — reordering measured within noise (RTF 0.2484–0.2529); salvage the regression test and the rejected eager-transform note (RTF 0.2449), then deletable |
| `perf/dart6-parallel-pair-flag-resize` | `assign(pairCount, 0)` → `resize(pairCount)` for the parallel pair-flag buffer | **Mine for WP-PG.31/32** — one-liner still applies verbatim (`DARTCollisionDetector.cpp:1342`); round-1 RTF-neutral (0.2476), hash-stable |
| `perf/dart6-single-reactive-union-reset` | Reset only the stamped `mUnionIndex` on single-reactive groups instead of the all-skeleton `resetUnion()` scan | **Mine for WP-PG.11** — ~4x reduction in its profile scope, hash-preserving, still unmerged |
| `perf/dart6-single-reactive-raw-root` | Drop the `mRootSkeleton` shared_ptr assignment on the all-single-reactive path (refcount churn) | **Mine for WP-PG.11** — measured improvement: RTF 0.272 vs 0.245 parent, `buildConstrainedGroups` 67.9 → 40.8 ms, same hash/contacts/pairs — strongest unmerged candidate; also hosts the journal |

Branch-cleanup recommendation (maintainer approval before deletion):
delete `perf/dart6-lazy-dart-shape-cache` now; delete
`perf/dart6-broadphase-cache-refresh-order` after WP-PG.42 salvages its
regression test; keep the rest until their packets record re-measured
verdicts on the round-2 baseline.

Measured-and-**rejected** round-1 experiments (journal commits
`72b24654702`, `1afdf2ffdf7`, `b1c89d498b6` on
`origin/perf/dart6-single-reactive-raw-root`) — do not re-attempt without
new evidence:

- Threaded-reset duplicate-check removal: no improvement.
- Lazy single-reactive activation dependency check: regressed.
- Skipping repeated deprecated colliding-flag writes: regressed.
- **Skipping the FreeJoint `updateJointForceFD()` no-op for one-body
  roots: regressed (RTF 0.2497)** — direct caveat for WP-PG.30, whose
  scope must differ (classification cache + FD fast path) and re-measure.
- Joint-constraint cache revision via membership+structural revisions:
  no improvement, needed a new `Skeleton` friendship.
- Caching the previous single-reactive skeleton during group build:
  regressed despite preserved hashes.

WP-PG.01 includes triaging these branches (mine → record verdicts here →
propose deletion of dead branches to the maintainer).

## WP-PG.01 status

**Captured and refreshed**: the original 2026-07-04 capture on
`origin/release-6.20` @ `5bee91ad6be` recorded the full matrix, scene
dumps, profile splits, dashboard artifacts, and prior-art triage. The
2026-07-05 refresh on `origin/release-6.20` @ `b9e6910c066` re-ran the
S1–S6 guard rows on the current base and updates the durable table above.
Zero command failures; Bullet S3/S4/S5 and S6 produced changed guard
hashes/counts relative to the older base and are now the round-2
determinism reference until a maintainer-approved re-baseline (D1/D7/D8
packets record old/new pairs here).
