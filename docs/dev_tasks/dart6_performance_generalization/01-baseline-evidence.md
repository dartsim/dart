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

## Initial probe — `BM_INTEGRATION_contact_container` (pre-plan)

Captured on `origin/release-6.20` @ `fdf89784e8d` (before #3230/#3226;
S1 is deactivation-disabled by design, so #3226 does not affect these
rows). Host: 32 CPUs, 5.3 GHz, CPU scaling ENABLED (treat ±5% as noise),
Release, GCC, 3 repetitions, means:

| Scenario (objects/engine/threads) | cpu ms/iter | sim_s/s (≈RTF) | contacts | resting/mobile |
| --- | ---: | ---: | ---: | --- |
| 60 / dart / 1 | 414 | 0.484 | 96 | 0/60 |
| 60 / dart / 16 | 436 | 0.459 | 96 | 0/60 |
| 60 / ode / 1 | 1850 | 0.109 | 249 | 0/60 |
| 60 / ode / 16 | 1755 | 0.114 | 249 | 0/60 |
| 120 / dart / 1 | 4793 | 0.042 | 272 | 0/120 |
| 120 / dart / 16 | 4440 | 0.045 | 272 | 0/120 |
| 120 / ode / 1 | 6808 | 0.029 | 542 | 0/120 |
| 120 / ode / 16 | 8278 | 0.025 | 542 | 0/120 |

Headline findings:

- 60→120 objects costs ~11.6x (native) — super-quadratic scaling
  consistent with dense per-island LCP assembly/solve.
- Threads are flat (native 1.08x at 120) or negative (ODE 0.85x, cv 17.6%).
- ODE emits ~2x the contacts of native on the same scene — physically
  different profiles; hashes are only comparable within one detector.

Profile smoke (active 60-body container, native detector, `--profile`,
single run): Dantzig LCP solve scope ≈ 66.2% of step time, collision
≈ 8%. **Caveat**: single run, one scene, and the number is the solver
scope without the `Construct LCP` split — re-validated by WP-PG.01 and
per-island histograms by WP-PG.10 before WS-A effort commits.

## Prior art — round-1 experiment branches (read before claiming packets)

Six unpushed round-1 experiment branches were published to origin on
2026-07-03 for reference (tips 21–28 commits behind current
`release-6.20`; none patch-equivalent to merged work; the deepest carries
the round-1 experiment journal):

| Branch (origin) | What it tries | Recorded outcome |
| --- | --- | --- |
| `perf/dart6-broadphase-cache-refresh` | Refresh native broadphase caches during entry build | not journaled — measure before reuse |
| `perf/dart6-lazy-dart-shape-cache` | Refresh DART shape caches during broadphase setup | not journaled — measure before reuse |
| `perf/dart6-broadphase-cache-refresh-order` | Cache refresh ordering stack | not journaled — measure before reuse |
| `perf/dart6-parallel-pair-flag-resize` | Avoid zeroing parallel collision flags | not journaled — measure before reuse |
| `perf/dart6-single-reactive-union-reset` | Narrow single-reactive union resets | not journaled — measure before reuse |
| `perf/dart6-single-reactive-raw-root` | Deepest stack + journal commits | journal below |

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

## WP-PG.01 baseline packet (to capture on the round-2 branch point)

Recapture the full cell matrix above on `origin/release-6.20` @
`70b92010311` via the dashboard tooling
(`pixi run python scripts/run_performance_dashboard_benchmarks.py`) plus
the scenario-CLI rows, with scene dumps, hashes, profile share splits
(incl. `Construct LCP` vs solve-proper), and host metadata; record the
guard rows in this file. This packet is the prerequisite for every other
packet.
