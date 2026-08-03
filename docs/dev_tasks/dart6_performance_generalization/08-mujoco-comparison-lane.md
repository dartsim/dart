# WS-G — MuJoCo cross-engine comparison lane (generalized performance)

> Owner doc for the maintainer-directed generalization goal: DART must
> outperform MuJoCo across its major workloads — robot arms/manipulators,
> humanoids, many-object scenes, sleeping-body cases, and highly dynamic
> non-sleeping cases — with full collision + constraint pipelines exercised
> (no sleep-shortcut wins on the dynamic classes).

## Harness

`scripts/mujoco_comparison/` (merged in #3367): split-process runners
(`dart_runner.py` in the default env against the dartpy dev build;
`mujoco_runner.py` in the lean `mujoco` pixi env pinning conda-forge
`mujoco-python`), one seeded SceneSpec emitting both a dartpy world and MJCF,
fairness contract in its README (same timestep, single-threaded, identical
seeded per-joint sinusoidal torques applied directly to joints on both
engines, warmup excluded, median-of-reps, finite/contact telemetry,
FFI-overhead calibration row, MuJoCo integrator normalized to Euler with
model-default sensitivity row, 1.1x win/noise band).

Reproduce from current `release-6.20` (the harness and the `mujoco` pixi
environment merged in #3367; #3369 merged the MJCF stacked-joint support and
collision-fidelity work needed by the affected rows):

```sh
pixi run build-py-dev && pixi install -e mujoco
PYTHONPATH=$PWD/build/default/cpp/Release/python/dartpy \
  pixi run python scripts/mujoco_comparison/run_comparison.py \
  --reps 5 --detector dart --dart-sleep off \
  --scene ARM-REACHER --scene ARM-PUSHER \
  --scene HUM-FALL --scene HUM-ACTIVE \
  --scene PILE-120 --scene PILE-900 --scene DYN-STIR-120 \
  --scene FFI-OVERHEAD \
  --mujoco-python $PWD/.pixi/envs/mujoco/bin/python --out-dir /tmp/mj_cmp
```

The explicit scene list is required: the runner's default set still omits the
two HUM rows. `--dart-sleep off` makes every active comparison independent of
deactivation. Do not accept the cross-engine matrix unless both HUM rows run;
a parser or runtime failure remains a blocked row rather than evidence.

## Standings (2026-07-31 full matrix on `718651d0d6e`, pre-manifold-fix)

First complete 8-scene run (both HUM rows ran and stayed finite — the
#3369 stacked-joint parsing works). Provenance: session branch
`wp-pg-wsg-rebaseline-20260731`, MuJoCo 3.10.0 (conda-forge, Python
3.14.6), Euler-normalized headline, `--reps 5` medians, `--detector
dart`, `--dart-sleep off`, single-threaded both engines, seeded
identical scenes, finite + contact telemetry recorded per rep. Artifacts
+ exact commands: `/tmp/mj_cmp_20260731` (`results.json`, `results.md`,
`raw/`, `provenance.txt`). Host in powersave with large clock swings, so
cross-session absolute steps/s are not comparable; the DART/MuJoCo ratio
per row is same-session and fair.

| Class | Scene | DART steps/s | MuJoCo steps/s | Ratio | Verdict |
| --- | --- | ---: | ---: | ---: | --- |
| Arms | ARM-REACHER | 72208 | 61039 | 1.18x | **DART wins** |
| Arms | ARM-PUSHER | 19985 | 27589 | 0.72x | DART loses |
| Humanoid | HUM-FALL | 8791 | 11384 | 0.77x | DART loses |
| Humanoid | HUM-ACTIVE | 13784 | 15724 | 0.88x | DART loses (near band) |
| Many objects | PILE-120 | 280 | 673 | 0.42x | DART loses |
| Many objects | PILE-900 | 123 | 3.1 | **39.9x** | **DART wins** (MuJoCo collapses at 900 bodies) |
| Dynamic | DYN-STIR-120 | 773 | 676 | 1.14x | **DART wins** |
| Reference | FFI-OVERHEAD | 343461 | 568478 | 0.60x | not scored (per-step FFI floor) |

MuJoCo model-default-integrator sensitivity rows (RK4 for the MJCF
scenes) ran and are slower than the Euler headline for MuJoCo, so the
Euler normalization is conservative toward MuJoCo. Deltas vs the
2026-07-10 prototypes: REACHER flipped to a DART win (was 2.3x behind),
PUSHER narrowed 5.7x → 1.4x behind, DYN-STIR flipped to a DART win (was
3.0x behind). The scored classes DART still loses: PUSHER (small-scene
per-step overhead, WP-SS family), HUM-FALL/HUM-ACTIVE (close), and
PILE-120 — the worst gap, measured on the pre-fix detector whose
3-point box manifolds keep piles rocking (see the criterion-2 regression
in 01-baseline-evidence.md); re-run the pile rows after the manifold
fix before cutting further pile packets.

### Post-manifold-fix partial rerun (2026-07-31, same session)

The box-manifold candidate fix (see 01-baseline-evidence.md) changes the
`dart` detector's stream in exactly the box scenes, so those rows were
rerun post-fix (`/tmp/mj_cmp_postfix_20260731`, reps 5, sensitivity
skipped; ARM-REACHER and the HUM rows carry over — no box-box face
stacking in those scenes):

| Scene | DART steps/s | MuJoCo steps/s | Ratio | Verdict (vs pre-fix run) |
| --- | ---: | ---: | ---: | --- |
| ARM-PUSHER | 36535 | 28771 | **1.27x** | **DART wins** (was 0.72x loss) |
| PILE-120 | 436 | 1025 | 0.43x | loses (ratio unchanged, 0.42–0.43x) |
| PILE-900 | 189 | 2.7 | ~70x | **DART wins** (was ~40x; see caveat) |
| DYN-STIR-120 | 584 | 685 | 0.85x | loses (was 1.14x win) |

(Displayed ratios come from the unrounded per-rep medians in
`results.json`, not the rounded steps/s columns.) Cross-run absolutes
are host-state-relative per the preamble, so per-scene deltas are
judged by whether MuJoCo's own number moved: MuJoCo is stable on
ARM-PUSHER (+4%) and DYN-STIR (+1%), so those verdict flips are
DART-side and real — the stabilized 4-point sliding-box stream wins
PUSHER outright, while the perpetually stirred DYN-STIR pile pays the
fourth-row solver cost with no stability dividend. On the PILE rows
MuJoCo itself moved (+52% / −13%), so treat PILE-120 by its unchanged
ratio and PILE-900 as "remains a ≥~40x DART win with an unstable
MuJoCo denominator at its collapse point", not as a widened margin.
Net scored standings post-fix: 3 wins (REACHER, PUSHER, PILE-900) /
4 losses (HUM-FALL, HUM-ACTIVE, PILE-120, DYN-STIR-120) — same count
as pre-fix, different composition. The maintainer ship/hold decision on
the manifold fix (README D9) picks which composition the branch
carries.

Sleeping class: the harness matrix runs with `--dart-sleep off` by
design, so the deactivation advantage is evidenced by the guard rows
instead: current-base `S2_dart` steps the settled 3k-shapes scene with
3003/3003 skeletons resting at ~0.04 ms/step in the 2026-07-31 capture
(`/tmp/wsg_rebaseline_guards_20260731/summary.tsv`; timing host-state
relative per 01-baseline-evidence.md), while MuJoCo has no sleeping
concept and pays full active cost on settled scenes. A formal same-scene
DART-sleep-on vs MuJoCo row remains future work; do not quote the old
informal "~60x" figure without capturing that comparison
reproducibly.

The prototype ant row remains deferred: the merged orchestrator does not
register an ant scenario, and no exact direct-runner command, artifact,
and engine SHAs were retained for the earlier measurement. Do not treat
that result as accepted evidence; restore a locomotion row only with
reproducible provenance.

## Gap analysis -> packets (figures predate the 2026-07-31 standings; superseded where they conflict)

1. **MJCF stacked-joint and collision fidelity**: #3369 merged stacked
   hinge/slide support, contype/conaffinity filtering, and per-geom friction.
   Re-run the affected rows on the merged base before accepting standings.
3. **Small-scene per-step overhead** (arms class): pusher profile shows
   ~11.35 µs per tiny LCP group (constructLcpTerms ~5 µs/call),
   updateConstraints self ~11.8 µs, integration ~12 µs, vs MuJoCo's whole
   step at 6.5 µs. Packet family WP-SS.1 (small-LCP construct/solve fast
   path), WP-SS.2 (`dart` small-scene collide overhead; also the S1-60
   parity gap), WP-SS.3 (integration overhead for small skeletons). Cut
   after fidelity packets land (do not optimize against invalid scenes).
4. **Active-pile gap (~2.7x)**: #3368's merged AABB-tree removes the
   many-object broadphase bottleneck and leaves solver-side cost (Dantzig
   per-island + contact allocation churn). Re-baseline on the merged base
   with the fidelity fixes before deciding packets (matrix-free option rows are
   diagnostic only, default-off per D3).
5. **dartpy getDofs ownership bug** (#3366): fixed; harness torque-drive
   depends on it.

## Rules

- Wins on ARM/HUM-ACTIVE/PILE(active)/DYN classes are accepted only when DART
  deactivation is disabled for the row, or when a bound resting-state API
  reports and the harness asserts zero sleeping bodies. A missing/unknown
  sleeping-body count is not evidence of zero sleeping bodies.
- SLEEP class is the only row where deactivation is the measured feature.
- Detector rows: `dart` is the DART-owned headline. FCL remains the global
  default, so capsule-sensitive MJCF rows must select `dart` explicitly.
- Every claimed row needs: exact command, SHAs, versions, governor, median
  of >=5 reps, finite + contact telemetry both engines.
