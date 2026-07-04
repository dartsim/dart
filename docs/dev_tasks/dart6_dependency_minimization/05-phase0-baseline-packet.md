# Phase-0 baseline packet — native collision port (DART 6.20)

> Deliverable of phase 0 in
> [03-native-collision-port-scoping.md](03-native-collision-port-scoping.md).
> This packet freezes the incumbent-detector evidence envelope that the
> native detector (phases 1–6) must be judged against, and records the
> default-flip verdict for the current tip.

## Capture context

- Captured 2026-07-04 (UTC) on `origin/release-6.20` =
  `b5b2a5bee7740ce35f9ee64a598528a9cdd42833` (binaries built at this SHA,
  Release, `--parallel 8` capped build, fresh configure).
- `origin/main` (DART 7 reference) =
  `dbe6fcccb1c2d42eb6827188cf3d523f0a732285`.
- Host: 13th Gen Intel Core i9-13950HX, 32 CPUs, `powersave` governor
  (CPU scaling enabled — RTF cells carry noise; hash/contact/resting
  cells do not). Toolchain: pixi 0.72.0 default env, GCC 15.2.0.
- Protocol: canonical guard scenes from
  `../dart6_performance_generalization/01-baseline-evidence.md` (S2/S3/
  S4/S5 reused cell-for-cell; S1/S6 are perf-lane-owned and
  cross-referenced from its WP-PG.01 packet, PR #3263), plus this lane's
  delta rows g120 (continuity with the 2026-07-03 scoping probe, now
  under `--max-contacts-per-pair 4` — hashes are not comparable to the
  uncapped probe) and g3000 (scoping-doc scale row). ODE rows are only
  valid with `--max-contacts-per-pair 4`.
- Raw outputs, JSONL scene dumps, and host metadata:
  `.omc/artifacts/native-collision-phase0/` (local, git-ignored).

## Row matrix

Command shapes (`CB=./build/default/cpp/Release/bin/contact_benchmark`,
`SDF=.deps/gz-sim/examples/worlds/3k_shapes.sdf`):

- g120: `$CB --generate-objects 120 --steps 300 --warmup 0 --checkpoint 0
  --quiet --collision <det> --world-threads 1 --max-contacts 20000
  --max-contacts-per-pair 4 --dump-final-scene <path>`
- S5: as g120 with `--generate-objects 90`, no dump.
- S4: `--generate-objects 900 --world-threads 16`, dump.
- g3000: `--generate-objects 3000 --world-threads 16 --max-contacts
  60000`, dump.
- S2: `$CB $SDF --steps 3000 --sdf-plane-shapes --quiet --checkpoint 0
  --collision <det> --world-threads 1 --max-contacts 12000
  --max-contacts-per-pair 4`, dump.
- S3: `$CB $SDF --steps 300 --disable-deactivation --world-threads 16
  --max-contacts 12000 --max-contacts-per-pair 4 --quiet --checkpoint 0
  --sdf-plane-shapes --collision <det>`.

| Row | RTF | Avg step (ms) | Contacts (cap hit) | Pairs | Resting | Max pen (m) | Finite | Hash |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- |
| g120-default | 1.068 | 0.936 | 240 (false) | 120 | 60/120 | 7.5e-04 | true | `0xe0fe5d6773190ca1` |
| g120-dart | 2.662 | 0.376 | 240 (false) | 120 | 80/120 | 1.0e-03 | true | `0xbd8a15e877f70822` |
| g120-fcl | 1.540 | 0.649 | 240 (false) | 120 | 60/120 | 7.5e-04 | true | `0xe0fe5d6773190ca1` |
| g120-bullet | 0.532 | 1.879 | 268 (false) | 120 | 76/120 | 9.8e-06 | true | `0x55069b7f62a734e6` |
| g120-ode | 19.159 | 0.052 | 0 (false) | 0 | 120/120 | 0 | true | `0xe52ba4524e3c5db2` |
| S5-90-dart | 1.462 | 0.684 | 180 (false) | 90 | 60/90 | 1.0e-03 | true | `0x726d1ff51bdb717` |
| S5-90-fcl | 2.517 | 0.397 | 180 (false) | 90 | 45/90 | 7.6e-04 | true | `0x99bfaef49c254203` |
| S5-90-bullet | 1.038 | 0.963 | 210 (false) | 90 | 55/90 | 1.0e-05 | true | `0xbeb495c1ab0d6ca6` |
| S5-90-ode | 32.296 | 0.031 | 0 (false) | 0 | 90/90 | 0 | true | `0x5f2afc7230ee8d10` |
| S4-900-dart | 0.309 | 3.237 | 1800 (false) | 900 | 600/900 | 1.0e-03 | true | `0x76205ad68f4293bb` |
| S4-900-fcl | 0.252 | 3.969 | 1800 (false) | 900 | 450/900 | 9.8e-04 | true | `0x7a0974e837912472` |
| S4-900-bullet | 0.129 | 7.742 | 2364 (false) | 900 | 269/900 | 1.1e-02 | true | `0x72bf270cdda36104` |
| S4-900-ode | 3.155 | 0.317 | 0 (false) | 0 | 900/900 | 0 | true | `0x429b65bc5c4a14b6` |
| g3000-dart | 0.070 | 14.28 | 6000 (false) | 3000 | 2000/3000 | 1.1e-03 | true | `0x48680325dda0fd3f` |
| g3000-fcl | 0.057 | 17.47 | 6000 (false) | 3000 | 1500/3000 | 1.0e-03 | true | `0xe1873605e7be1a9` |
| g3000-bullet | 0.033 | 30.67 | 8695 (false) | 3000 | 635/3000 | 4.4e-01 | true | `0xb18a18cf6549222a` |
| g3000-ode | 0.992 | 1.008 | 0 (false) | 0 | 3000/3000 | 0 | true | `0x559483fad1bd256f` |
| S2-3k-dart | 23.35 | 0.043 | 0 (false) | 0 | 3003/3003 | 0 | true | `0x8ddc9a81f2d28a7f` |
| S2-3k-fcl | 23.72 | 0.042 | 0 (false) | 0 | 3003/3003 | 0 | true | `0x266da31836a314a6` |
| S2-3k-bullet | 2.490 | 0.402 | 0 (false) | 0 | 3003/3003 | 0 | true | `0x2375f1927218cd43` |
| S2-3k-ode | 10.37 | 0.096 | 0 (false) | 0 | 3003/3003 | 0 | true | `0x10f80b0408cede90` |
| S3-3k-dart | 0.105 | 9.54 | 5005 (false) | 3003 | 0/3003 | 9.3e-09 | true | `0xcf0ba6eaa97be038` |
| S3-3k-fcl | 0.065 | 15.41 | 3003 (false) | 3003 | 0/3003 | 9.3e-09 | true | `0x6088ea0177efa6a` |
| S3-3k-bullet | 0.013 | 79.26 | 5005 (false) | 3003 | 0/3003 | 3.0e-07 | true | `0xabf3edd146317478` |
| S3-3k-ode | 0.002 | 486.9 | 9009 (false) | 3003 | 0/3003 | 9.3e-09 | true | `0x4904c09a93a36442` |

## Validity cross-checks

- **Determinism (run-to-run):** g120-fcl and g120-dart re-runs at the
  same binary reproduce their hashes bit-exactly.
- **Determinism (cross-SHA):** the S4 (all four detectors) and S5-dart
  hashes match the perf lane's WP-PG.01 cells (PR #3263, captured at
  `5bee91ad6be`) bit-for-bit — the intervening docs + `pixi.lock`
  commits did not perturb numerics. RTF cells differ between the two
  packets (governor noise); hash/contact/resting cells are the
  determinism reference, per the shared protocol.
- **`default` ≡ `fcl`:** identical hash and a scene-dump delta of
  exactly zero on g120 — consistent with `FCLCollisionDetector::create()`
  in both `ConstraintSolver` constructors
  (`dart/constraint/ConstraintSolver.cpp:336`, `:353`).

## Scene-dump tolerance analysis (reference: fcl)

Per-mobile-shape final-position distance and resting-flag mismatches:

| Scene | Detector | mean (m) | p95 (m) | max (m) | resting mismatch |
| --- | --- | ---: | ---: | ---: | ---: |
| g120 | default | 0 | 0 | 0 | 0 |
| g120 | dart | 1.1e-05 | 3.9e-05 | 2.5e-04 | 20/120 |
| g120 | bullet | 2.3e-04 | 7.1e-04 | 7.5e-04 | 60/120 |
| g120 | ode | 2.3e-04 | 7.1e-04 | 7.5e-04 | 60/120 |
| S4-900 | dart | 7.4e-06 | 2.8e-05 | 3.6e-04 | 150/900 |
| S4-900 | bullet | 2.6e-04 | 7.1e-04 | 1.1e-02 | 397/900 |
| S4-900 | ode | 2.3e-04 | 7.1e-04 | 9.7e-04 | 450/900 |
| g3000 | dart | 7.4e-06 | 3.1e-05 | 3.6e-04 | 500/3000 |
| g3000 | bullet | 4.1e-02 | 4.4e-01 | 4.4e-01 | 1313/3000 |
| g3000 | ode | 2.4e-04 | 7.1e-04 | 1.0e-03 | 1500/3000 |
| S2-3k | dart | 8.2e-12 | 2.5e-11 | 2.5e-11 | 0 |
| S2-3k | bullet | 1.8e-06 | 3.4e-06 | 3.4e-06 | 0 |
| S2-3k | ode | 8.2e-12 | 2.5e-11 | 2.5e-11 | 0 |

Readings:

- The in-tree `dart` detector tracks FCL positions to ≤ 0.4 mm on every
  active scene and ≤ 2.5e-11 m settled — that is the achievable
  same-physics envelope for a DART-owned detector and the working
  tolerance target for the native port.
- Bullet materially diverges at scale (0.44 m max drift and 0.44 m final
  penetration on g3000) — a different-physics profile, not a porting
  target.
- ODE's generated-scene rows are physically divergent in a subtler way:
  final positions stay sub-mm (the piles are near rest by design) but it
  reports **zero contacts with everything asleep** on all generated rows
  (g120/S5/S4/g3000) while emitting 3x FCL's contacts on active-3k. Its
  headline RTF wins are an artifact of that divergence (also flagged as
  WP-PG.01 finding: "ODE remains physically divergent").
- On the SDF plane scene (S3), `dart` emits 5005 contacts vs FCL's 3003
  with identical max penetration — contact-count parity must be judged
  per capability incumbent, not as a single global number.

## Capability incumbents (parity targets, re-verified at this SHA)

- Default detector: FCL, created in **both** `ConstraintSolver`
  constructors (`dart/constraint/ConstraintSolver.cpp:336`, `:353`) —
  the phase-6 flip must change both.
- Distance queries: FCL only (others warn-and-return stubs).
- Raycast: Bullet only (`BulletCollisionDetector::raycast` is the sole
  override).
- VoxelGrid/octree: FCL only (`fcl::OcTree`).

## Verdict

**Native default: NOT allowed at this tip.** No native engine exists on
`release-6.20` (`dart/collision/native/` absent; the in-tree `dart`
detector is the old limited narrowphase-only detector). FCL remains the
default. This is the expected phase-0 outcome; the packet's purpose is
the envelope below.

**Acceptance envelope for the phase-6 default flip** (all against this
packet's rows, re-captured on the flip PR's parent for drift control):

1. Performance: native RTF ≥ FCL on every row above, and ≥ the in-tree
   `dart` detector on the active rows where it currently leads (g120,
   g3000, S3) — speed-only wins that alter contact/resting profiles are
   rejected.
2. Correctness: finite state and no cap hit on every row; run-to-run
   hash determinism; position drift vs FCL within the `dart` detector's
   demonstrated band (≤ 1e-3 m max on active scenes, ≤ 1e-5 m settled);
   resting counts within the FCL–dart band per scene; contact counts
   explained per capability incumbent.
3. Capability parity: distance vs FCL, raycast vs Bullet, VoxelGrid/
   octree vs FCL, CCD + persistent manifolds + per-pair capping per
   `03`'s matrix.
4. Gates: full A/B packet, `pixi run test-all` build + `pixi run test`/
   `test-py` runtime, and `DART_PARALLEL_JOBS=8 pixi run -e gazebo
   test-gz` green.

Phase 1 (native core skeleton, C++17, no EnTT, internal-only, FCL stays
default) is cleared to start once this packet is reviewed.
