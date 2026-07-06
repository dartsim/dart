# WS-D — SIMD enablement lane

`dart/simd` (#3229) is merged and tested but has **zero consumers** in
dynamics/collision/constraint code. This lane makes it earn its keep at
proven seams — following main's `lie_group_batch.md` discipline (free
functions, 4-wide batches, `simd::cross3` for cross products, scalar
tail) and the native-port scoping rule ("SIMD is an optimization packet,
not a substitute": scalar correctness first, measured before/after
always).

Round-1 evidence says the cost is structural, not arithmetic — so this
lane is deliberately *third* in sequencing: kernels land where WS-A/WS-C
restructuring has exposed batchable loops, not before.

Current execution note (2026-07-05): standalone WP-PG.40 PR #3270 was
closed by maintainer direction so the D1/D2 contract and prototype evidence
ride with the first real SIMD kernel PR, not a docs-only packet. The first
production-consumer slice is `wp-pg-42-soa-broadphase-simd`: it keeps the
DART 6 packaged default at baseline ISA, extends `ci_simd.yml` so the
DART detector builds/runs under scalar/SSE4.2/AVX/AVX2, and starts
WP-PG.42 with AVX-width SIMD-screened finite broadphase candidate
batches while scalar/SSE/NEON builds keep the previous scalar sweep shape.
WS-F phase 1 (#3281) has landed only the internal native collision math
core; there is still no DART 6 detector adapter or phase-4 native
broadphase SIMD work, so this slice does not duplicate WS-F.

Local evidence (2026-07-05, js workstation, GCC 15.2, CPU scaling enabled;
treat small deltas as noisy): `origin/release-6.20@c371060c9fa` was compared
against the `wp-pg-42-soa-broadphase-simd` worktree. The focused detector test
passed in default, scalar-forced, SSE4.2, AVX, and AVX2 builds; after adding
finite-finite text-profiler scopes, the focused detector test was rebuilt and
re-run in scalar/SSE4.2/AVX/AVX2. The workflow-equivalent SIMD target set also
passed in scalar/SSE4.2/AVX/AVX2 before the profiler-label-only edit.
`BM_INTEGRATION_contact_container` DART rows preserved contacts/resting/mobile
exactly (60 rows: 96/0/60; 120 rows: 272/0/120). Default/package-safe builds
compile out the AVX-width batch loop and showed no regression signal; after the
profiler-label edit, the default DART rows were 60/1 341 ms, 60/16 340 ms,
120/1 3939 ms, and 120/16 3881 ms, still below the recorded default baseline
120 rows (4152 ms and 4110 ms). AVX2 builds exercise the batch loop; post-edit
macro rows were 60/1 249 ms, 60/16 254 ms, 120/1 2965 ms, and 120/16 2977 ms,
with counters unchanged. A profiled native contact-container sample shows the
new finite-finite same-group sweep scope is material on the 60-object row
(24.34 ms / 7.18% of 200 profiled steps) and visible but solver-dominated on
the 120-object row (30.29 ms / 1.61% of 100 profiled steps).

#### WP-PG.40 — FP-determinism + ISA delivery contracts (design packet)

- Status: folded into the first actual SIMD kernel PR per #3270 maintainer
  direction; WP-PG.42 carries the active D1/D2 evidence path.
- Objective: decide and document (in the README decisions section plus a
  compatibility note owner doc): (a) the FP contract for SIMD kernels on
  state-affecting paths (proposal: bit-identical to scalar — no
  reassociation, no FMA divergence between backends; hash re-baselining
  only with maintainer sign-off); (b) ISA delivery (proposal: baseline-ISA
  in packaged binaries now; a runtime-dispatch design sketch with
  per-backend TUs + CPU detection as a follow-up packet if evidence shows
  AVX2 gains worth shipping).
- Value: unblocks every SIMD packet; prevents a class of silent gz drift
  (the gz gate's ChangedWorldPoses patch is exact-equality).
- Scope: docs + a compile-only prototype demonstrating the chosen contract
  on one kernel (e.g. batched cross products) with a
  scalar-vs-SIMD bit-equality test.
- Acceptance evidence: decision record; prototype test proving
  bit-equality under the chosen flags on scalar/SSE4.2/AVX2 backends
  (`DART_SIMD_FORCE_SCALAR` matrix in ci_simd.yml).
- Dependencies: none (can run parallel to everything).

#### WP-PG.41 — Batch math helpers at the contact-Jacobian seam

- Status: blocked on WP-PG.40
- Objective: add main-style batch helpers (AdT/AdInvT/dAdT-style spatial
  transforms and cross3 batches, 4-wide, scalar tail) to `dart/math` as
  free functions consuming `dart::simd` types strictly inside cpp TUs,
  and consume them in the contact-Jacobian/impulse assembly loops
  identified by WP-PG.10's profile.
- Value: first real SIMD win on the hot path with ABI-safe delivery
  (no exported layout/mangling change).
- Scope: `dart/math` additive helpers + `dart/constraint` cpp call sites;
  benchmark with `bm_simd` size ranges (16–4096) and the
  contact-container gate.
- Non-goals: header-inline SIMD in public headers; Dantzig inner loops;
  fixed-size Eigen expression rewrites (already vectorized).
- Acceptance evidence: bit-identical hashes per the D1 contract; before/
  after `bm_simd` micro rows + contact-container macro rows; SIGILL-safe
  packaging posture per D2 (no `-march` on exported targets).
- Dependencies: WP-PG.40, WP-PG.10 (seam selection).

#### WP-PG.42 — SoA broadphase sweep batching (coordinate with WS-F)

- Status: claimed — `wp-pg-42-soa-broadphase-simd` starts with
  AVX-width SIMD-screened finite candidate batches in the existing DART
  detector, plus CI matrix coverage for the production consumer. Remaining
  scope in this packet still includes measured before/after evidence and
  any version-gated AABB recomputation follow-up that survives profiling.
- Objective: split the DART-native detector's `BroadphaseEntry` into SoA
  min/max arrays and batch the y/z overlap tests inside the x-sweep
  (`DARTCollisionDetector.cpp:2329-2347`) with `dart/simd`; version-gate
  AABB recomputation for resting objects (`:1710`).
- Value: broadphase share on active large scenes; also a template for the
  native-port optimization phase.
- Scope: `dart/collision/dart/*` cpp; SoA scratch internal to the
  detector. Prior art: `origin/perf/dart6-broadphase-cache-refresh`,
  `perf/dart6-lazy-dart-shape-cache`, and
  `perf/dart6-broadphase-cache-refresh-order` attempted adjacent cache
  refresh ideas in round 1 with unrecorded outcomes — mine and measure
  them (see 01-baseline-evidence.md prior-art inventory) before writing
  new code.
- Non-goals: duplicating WS-F phase-4 work — if the native port's
  broadphase supersedes this detector, this packet moves there. Check the
  orchestration dashboard before claiming.
- Acceptance evidence: bit-identical native-detector outcomes; broadphase
  stage share before/after (WP-PG.10 scopes); coordination note with
  WS-F recorded.
- Dependencies: WP-PG.40; WS-F phase status check.
