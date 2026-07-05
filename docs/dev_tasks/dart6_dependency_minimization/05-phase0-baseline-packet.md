# Phase-0 baseline packet — native collision port (DART 6.20)

> Deliverable of phase 0 in
> [03-native-collision-port-scoping.md](03-native-collision-port-scoping.md).
> This packet freezes the incumbent-detector evidence envelope that the
> native detector (phases 1–6) must be judged against, and records the
> default-flip verdict for the current tip.

## Capture context

- Captured 2026-07-05 (UTC) on this PR branch head =
  `1e6a8332a730a994450c14ee8a780780c5e069bb` after merging `origin/release-6.20` =
  `949a9c2ff5ed6309beef0aa1345101d36c813f02` (binaries built at the branch head,
  Release, `--parallel 8` capped build, fresh configure).
- `origin/main` (DART 7 reference) =
  `5c75381f79a0431909f8c1b0a04fca1fbaa256ed`.
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
- Raw evidence: the committed appendix
  [05-artifacts.md](05-artifacts.md) carries the host metadata, every
  row's raw final summary, the capture driver, the tolerance analyzer,
  and SHA-256 digests of the 17 JSONL scene dumps. The full dumps are
  not committed; they stayed local under
  `.omc/artifacts/native-collision-phase0-1e6a8332a730/` (git-ignored).
  The digests can verify a copied/published dump archive, but a clean
  checkout cannot reproduce the per-body tolerance table from hashes
  alone. Any phase-6/default-flip PR must either retrieve JSONL dumps
  matching these digests or recapture dumps on its own parent and use
  that same recapture for the tolerance gate. The driver default remains
  `.omc/artifacts/native-collision-phase0/` for future recaptures.

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
| g120-default | 0.2093 | 4.778 | 240 (false) | 120 | 60/120 | 7.52e-04 | true | `0xe0fe5d6773190ca1` |
| g120-dart | 0.4853 | 2.061 | 240 (false) | 120 | 80/120 | 1.00e-03 | true | `0xbd8a15e877f70822` |
| g120-fcl | 0.5349 | 1.87 | 240 (false) | 120 | 60/120 | 7.52e-04 | true | `0xe0fe5d6773190ca1` |
| g120-bullet | 0.4895 | 2.043 | 268 (false) | 120 | 78/120 | 9.99e-06 | true | `0x3508439b0c95d6be` |
| g120-ode | 11.34 | 0.08815 | 0 (false) | 0 | 120/120 | 0 | true | `0xe52ba4524e3c5db2` |
| S5-90-dart | 1.18 | 0.8474 | 180 (false) | 90 | 60/90 | 1.00e-03 | true | `0x726d1ff51bdb717` |
| S5-90-fcl | 0.2164 | 4.622 | 180 (false) | 90 | 45/90 | 7.59e-04 | true | `0x99bfaef49c254203` |
| S5-90-bullet | 0.4868 | 2.054 | 210 (false) | 90 | 56/90 | 9.99e-06 | true | `0xf78e3bd075780c83` |
| S5-90-ode | 45.11 | 0.02217 | 0 (false) | 0 | 90/90 | 0 | true | `0x5f2afc7230ee8d10` |
| S4-900-dart | 0.03892 | 25.69 | 1800 (false) | 900 | 600/900 | 1.05e-03 | true | `0x76205ad68f4293bb` |
| S4-900-fcl | 0.02881 | 34.71 | 1800 (false) | 900 | 450/900 | 9.76e-04 | true | `0x7a0974e837912472` |
| S4-900-bullet | 0.03252 | 30.75 | 2353 (false) | 900 | 269/900 | 0.0113 | true | `0x2a5577952e2de925` |
| S4-900-ode | 3.144 | 0.3181 | 0 (false) | 0 | 900/900 | 0 | true | `0x429b65bc5c4a14b6` |
| g3000-dart | 0.01004 | 99.57 | 6000 (false) | 3000 | 2000/3000 | 1.06e-03 | true | `0x48680325dda0fd3f` |
| g3000-fcl | 7.47e-03 | 133.8 | 6000 (false) | 3000 | 1500/3000 | 1.04e-03 | true | `0xe1873605e7be1a9` |
| g3000-bullet | 0.01386 | 72.15 | 8633 (false) | 3000 | 635/3000 | 0.44 | true | `0xd49e015847f0a8f` |
| g3000-ode | 0.5167 | 1.935 | 0 (false) | 0 | 3000/3000 | 0 | true | `0x559483fad1bd256f` |
| S2-3k-dart | 7.098 | 0.1409 | 0 (false) | 0 | 3003/3003 | 0 | true | `0x8ddc9a81f2d28a7f` |
| S2-3k-fcl | 6.746 | 0.1482 | 0 (false) | 0 | 3003/3003 | 0 | true | `0x266da31836a314a6` |
| S2-3k-bullet | 4.212 | 0.2374 | 0 (false) | 0 | 3003/3003 | 0 | true | `0x2375f1927218cd43` |
| S2-3k-ode | 4.264 | 0.2345 | 0 (false) | 0 | 3003/3003 | 0 | true | `0x10f80b0408cede90` |
| S3-3k-dart | 0.01226 | 81.57 | 5005 (false) | 3003 | 0/3003 | 9.32e-09 | true | `0xcf0ba6eaa97be038` |
| S3-3k-fcl | 0.01185 | 84.39 | 3003 (false) | 3003 | 0/3003 | 9.32e-09 | true | `0x6088ea0177efa6a` |
| S3-3k-bullet | 0.02281 | 43.84 | 5005 (false) | 3003 | 0/3003 | 2.98e-07 | true | `0x22e27960cbabe83e` |
| S3-3k-ode | 4.84e-03 | 206.8 | 9009 (false) | 3003 | 0/3003 | 9.32e-09 | true | `0x4904c09a93a36442` |

## Validity cross-checks

- **Recaptured on branch tip:** the table above was regenerated after
  merging the latest `origin/release-6.20` (949a9c2ff5ed) into this PR
  branch and rebuilding the benchmark binary at `1e6a8332a730`. This
  supersedes the earlier 2026-07-04 `b5b2a5bee77` capture for any
  phase-6 comparison.
- **Determinism (run-to-run):** g120-fcl and g120-dart re-runs at the
  same binary reproduce their hashes bit-exactly (`0xe0fe5d6773190ca1`,
  `0xbd8a15e877f70822`).
- **Determinism (cross-SHA):** S4/S5 dart, FCL, and ODE hashes still
  match the perf lane's WP-PG.01 cells (PR #3263, captured at
  `5bee91ad6be`) bit-for-bit. Bullet's generated-scene hashes changed
  after the release-branch solver/resting fixes, so this recaptured
  packet is the current Bullet baseline for this lane.
- **`default` ≡ `fcl`:** identical hash and a scene-dump delta of
  exactly zero on g120 — consistent with `FCLCollisionDetector::create()`
  in both `ConstraintSolver` constructors
  (`dart/constraint/ConstraintSolver.cpp:336`, `:353`).

## Scene-dump tolerance analysis (reference: fcl)

Per-mobile-shape final-position distance and resting-flag mismatches:

| Scene | Detector | mean (m) | p95 (m) | max (m) | resting mismatch |
| --- | --- | ---: | ---: | ---: | ---: |
| g120 | default | 0 | 0 | 0 | 0/120 |
| g120 | dart | 1.13e-05 | 3.90e-05 | 2.53e-04 | 20/120 |
| g120 | bullet | 2.33e-04 | 7.08e-04 | 7.49e-04 | 62/120 |
| g120 | ode | 2.33e-04 | 7.09e-04 | 7.50e-04 | 60/120 |
| S4-900 | dart | 7.44e-06 | 2.85e-05 | 3.60e-04 | 150/900 |
| S4-900 | bullet | 2.63e-04 | 7.07e-04 | 0.0106 | 397/900 |
| S4-900 | ode | 2.34e-04 | 7.08e-04 | 9.73e-04 | 450/900 |
| g3000 | dart | 7.45e-06 | 3.12e-05 | 3.64e-04 | 500/3000 |
| g3000 | bullet | 0.04034 | 0.4429 | 0.4429 | 1313/3000 |
| g3000 | ode | 2.35e-04 | 7.12e-04 | 1.05e-03 | 1500/3000 |
| S2-3k | dart | 8.18e-12 | 2.45e-11 | 2.45e-11 | 0/3003 |
| S2-3k | bullet | 1.84e-06 | 3.43e-06 | 3.43e-06 | 0/3003 |
| S2-3k | ode | 8.18e-12 | 2.45e-11 | 2.45e-11 | 0/3003 |

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
this phase-0 branch (`dart/collision/native/` absent; the in-tree `dart`
detector is the old limited narrowphase-only detector). FCL remains the
default. This is the expected phase-0 outcome; the packet's purpose is
the envelope below.

**Acceptance envelope for the phase-6 default flip** (all against this
packet's rows, re-captured on the flip PR's parent for drift control):

1. Performance: native RTF ≥ FCL on every row above, and ≥ the stronger
   in-tree incumbent (`fcl` or `dart`) for each generated/active row in
   the same recapture — speed-only wins that alter contact/resting
   profiles are rejected.
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
