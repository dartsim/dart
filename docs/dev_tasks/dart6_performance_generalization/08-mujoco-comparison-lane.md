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
  --reps 5 --detector native \
  --scene ARM-REACHER --scene ARM-PUSHER \
  --scene HUM-FALL --scene HUM-ACTIVE \
  --scene PILE-120 --scene PILE-900 --scene DYN-STIR-120 \
  --scene FFI-OVERHEAD \
  --mujoco-python $PWD/.pixi/envs/mujoco/bin/python --out-dir /tmp/mj_cmp
```

The explicit scene list is required: the runner's default set still omits the
two HUM rows. Do not accept the cross-engine matrix unless both HUM rows run;
a parser or runtime failure remains a blocked row rather than evidence.

## Standings (2026-07-10 prototype rows; quiet-host full matrix pending)

| Class | Scene | DART (native det) | MuJoCo | Verdict |
| --- | --- | ---: | ---: | --- |
| Arms | reacher | 0.0200 | 0.0086 | MJ 2.3x (µs-scale) |
| Arms | pusher | 0.0373 | 0.0065 | MJ 5.7x |
| Arms | striker/thrower | — | — | scenario registration + merged-base rerun pending |
| Humanoid | humanoid.xml | — | — | merged-base HUM-FALL/HUM-ACTIVE rerun pending |
| Many objects | PILE-120 active | 1.55 ms/step | 0.57 | MJ ~2.7x |
| Dynamic | DYN-STIR-120 | 1.82 | 0.61 | MJ ~3.0x |
| Sleeping | settled 3k | 0.034 ms/step (deactivation) | ~2 (no sleeping exists) | **DART ~60x** (to be formalized) |

The prototype ant row is deferred: the merged orchestrator does not register
an ant scenario, and no exact direct-runner command, artifact, and engine SHAs
were retained for the earlier measurement. Do not treat that result as accepted
evidence; restore a locomotion row only with reproducible provenance.

## Gap analysis -> packets

1. **MJCF stacked-joint and collision fidelity**: #3369 merged stacked
   hinge/slide support, contype/conaffinity filtering, and per-geom friction.
   Re-run the affected rows on the merged base before accepting standings.
3. **Small-scene per-step overhead** (arms class): pusher profile shows
   ~11.35 µs per tiny LCP group (constructLcpTerms ~5 µs/call),
   updateConstraints self ~11.8 µs, integration ~12 µs, vs MuJoCo's whole
   step at 6.5 µs. Packet family WP-SS.1 (small-LCP construct/solve fast
   path), WP-SS.2 (native small-scene collide overhead; also the S1-60
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
- Detector rows: `dart` is the headline after the phase-6 consolidation and
  default flip; reserve `native` for explicit pre-consolidation runs (FCL
  default mangles capsules -> MJCF rows under the default detector are invalid).
- Every claimed row needs: exact command, SHAs, versions, governor, median
  of >=5 reps, finite + contact telemetry both engines.
