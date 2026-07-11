# WS-G — MuJoCo cross-engine comparison lane (generalized performance)

> Owner doc for the maintainer-directed generalization goal: DART must
> outperform MuJoCo across its major workloads — robot arms/manipulators,
> humanoids, many-object scenes, sleeping-body cases, and highly dynamic
> non-sleeping cases — with full collision + constraint pipelines exercised
> (no sleep-shortcut wins on the dynamic classes).

## Harness

`scripts/mujoco_comparison/` (proposed in open PR #3367): split-process runners
(`dart_runner.py` in the default env against the dartpy dev build;
`mujoco_runner.py` in the lean `mujoco` pixi env pinning conda-forge
`mujoco-python`), one seeded SceneSpec emitting both a dartpy world and MJCF,
fairness contract in its README (same timestep, single-threaded, identical
seeded per-joint sinusoidal torques applied directly to joints on both
engines, warmup excluded, median-of-reps, finite/contact telemetry,
FFI-overhead calibration row, MuJoCo integrator normalized to Euler with
model-default sensitivity row, 1.1x win/noise band).

Reproduce (the harness and the `mujoco` pixi environment are in open PR #3367;
until that merges, run from the `feature/mujoco-comparison-harness` branch —
and the humanoid rows additionally need #3369's MJCF stacked-joint support):

```sh
pixi run build-py-dev && pixi install -e mujoco
PYTHONPATH=$PWD/build/default/cpp/Release/python/dartpy \
  pixi run python scripts/mujoco_comparison/run_comparison.py \
  --reps 5 --detector native \
  --mujoco-python $PWD/.pixi/envs/mujoco/bin/python --out-dir /tmp/mj_cmp
```

## Standings (2026-07-10 prototype rows; quiet-host full matrix pending)

| Class | Scene | DART (native det) | MuJoCo | Verdict |
| --- | --- | ---: | ---: | --- |
| Locomotion | ant | 0.0306 ms/step | 0.0398 | **DART +30%** |
| Arms | reacher | 0.0200 | 0.0086 | MJ 2.3x (µs-scale) |
| Arms | pusher | 0.0373 | 0.0065 | MJ 5.7x |
| Arms | striker/thrower | — | — | INVALID until contype fidelity lands |
| Humanoid | humanoid.xml | parse fails | 0.113 | blocked on stacked-joint parser |
| Many objects | PILE-120 active | 1.55 ms/step | 0.57 | MJ ~2.7x |
| Dynamic | DYN-STIR-120 | 1.82 | 0.61 | MJ ~3.0x |
| Sleeping | settled 3k | 0.034 ms/step (deactivation) | ~2 (no sleeping exists) | **DART ~60x** (to be formalized) |

## Gap analysis -> packets

1. **MJCF stacked-joint compositions** (humanoid/walker2d/hopper/half_cheetah/
   swimmer fail to load; MjcfParser.cpp:386): chain-of-1-DOF-joints design;
   packet delegated (in flight).
2. **MJCF collision/friction fidelity**: contype/conaffinity ignored (DART
   collides marker geoms MuJoCo excludes — striker's density-1e6 cylinder
   makes Dantzig fail every step and forces the PGS fallback + double
   constructLcpTerms); per-geom friction ignored. Blocks fair arm rows.
3. **Small-scene per-step overhead** (arms class): pusher profile shows
   ~11.35 µs per tiny LCP group (constructLcpTerms ~5 µs/call),
   updateConstraints self ~11.8 µs, integration ~12 µs, vs MuJoCo's whole
   step at 6.5 µs. Packet family WP-SS.1 (small-LCP construct/solve fast
   path), WP-SS.2 (native small-scene collide overhead; also the S1-60
   parity gap), WP-SS.3 (integration overhead for small skeletons). Cut
   after fidelity packets land (do not optimize against invalid scenes).
4. **Active-pile gap (~2.7x)**: on open PR #3368's branch, the AABB-tree
   removes the many-object broadphase bottleneck and leaves solver-side cost
   (Dantzig per-island + contact allocation churn). This conclusion is
   branch-local until #3368 merges; re-baseline on the merged base with the
   fidelity fixes before deciding packets (matrix-free option rows are
   diagnostic only, default-off per D3).
5. **dartpy getDofs ownership bug** (#3366): fixed; harness torque-drive
   depends on it.

## Rules

- Wins on ARM/HUM-ACTIVE/PILE(active)/DYN classes must show zero sleeping
  bodies in DART (deactivation disabled on DYN rows; asserted).
- SLEEP class is the only row where deactivation is the measured feature.
- Detector rows: native is the headline once phase 6 flips; until then
  report native explicitly (FCL default mangles capsules -> MJCF rows under
  the default detector are invalid).
- Every claimed row needs: exact command, SHAs, versions, governor, median
  of >=5 reps, finite + contact telemetry both engines.
