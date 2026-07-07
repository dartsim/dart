> **SUPERSEDED IN PART (2026-07-06).** Read
> [`base-release620-heap-crash.md`](base-release620-heap-crash.md) and
> [`HANDOFF-wp-pg-20.md`](HANDOFF-wp-pg-20.md) first — they carry the current,
> corrected understanding. What still stands from this note: the **WP-PG.31**
> load-dependent 16-thread heap crash (base clean, PG.30+31 crashes under the
> same load). What was later shown to be a **missing-binary false positive** (do
> NOT trust here): any claim that the *base* crashes at single-thread, or that
> #3297 introduced a base bug. The base is clean; WP-PG.20's suspected crash
> mechanism is ruled out by code analysis (a code-level argument — the crash A/B
> is still a required pre-merge gate, not yet run). This file is retained for
> history only; "exonerated"/"EXONERATED" further below is older wording,
> superseded by the corrected docs named above.

# PR-A crash investigation (working note — resolve before merge)

Branch `wp-pg-30-single-free-body-cache` consolidates WP-PG.30 (committed)
+ WP-PG.31 + WP-PG.11 (uncommitted in the working tree at handoff).

## BLOCKER: heap corruption under combined tree (must clear before PR)

- Symptom: `free(): invalid size` then `malloc(): invalid size (unsorted)`
  during the full `BM_INTEGRATION_contact_container` suite (16-thread rows).
- Bisection (executor, non-ASan, varying host load): clean parent = no
  crash; WP-PG.31 alone = no crash; WP-PG.11 alone = no crash; **both
  together = crashed twice**, then 8+ clean runs afterward.
- Executor's hypothesis: environmental (host load avg 83 from ~5 concurrent
  LTO builds). **Orchestrator dissent:** `free(): invalid size` is glibc
  heap-metadata corruption — not a symptom of host load (which causes
  slowness / OOM-kill). "Innocent alone, corrupts together" + both packets
  touching cross-step reused memory (PG.31 retained scratch buffers; PG.11
  cache fields on pooled `mReusableContactConstraints`) = classic latent
  OOB/UAF signature. Treat as likely-real until proven otherwise.
- **REPRODUCED INDEPENDENTLY (orchestrator, 2026-07-05):** clean-host
  takeover run crashed at the same `BM_INTEGRATION_contact_container` suite
  with `malloc(): invalid size (unsorted)`, core dumped, at **Load Average
  88.32**. Both crashes now correlate with extreme load (83, 88) — this is
  the signature of a **timing-dependent data race** in the 16-thread path
  (high load perturbs scheduling → exposes the race window), NOT host
  thrash causing corruption. Environmental hypothesis is REJECTED. Guard
  hashes remain bit-identical (single-thread determinism intact); the
  defect is concurrency-only. Prime suspect: PG.31 turned per-call-local
  scratch into retained World members — if written by multiple worker
  threads (parallel constraint/collision paths) it races; and/or PG.11
  cache writes on pooled ContactConstraints across the threaded build pass.
- Tool order: ASan (spatial OOB/UAF, deterministic) then **TSan** (the race
  hypothesis — highest-signal tool for this symptom). ctest 11/11 and the
  serial P2 profile run clean (single-threaded), consistent with a
  threading-only bug.
- **Gate: ASan build + full S1 suite + repeated 16-thread stress must run
  clean before this ships.** If ASan is clean but suspicion remains (race,
  not overflow), follow with TSan on the 16-thread scenario. If either
  flags a real error: root-cause, fix or drop the offending packet.

## Verified findings (executor, to fold into PR body once crash cleared)

- Guard hashes bit-identical parent vs combined: S2 `0x8ddc9a81f2d28a7f`,
  S3 `0xcf0ba6eaa97be038`, S4 `0x76205ad68f4293bb`, S5 `0x726d1ff51bdb717`.
  ALL build + ctest 127/127 + dartpy 78/78 passed on the combined tree.
- WP-PG.31: original stash could not plain-pop (base diverged via #3273's
  `getRootBodyNodeIfAny` + #3280's `captureResetShallowSupportFreeRoot...`);
  reimplemented scratch-retention against merged base, upstream preserved.
  Skip-when-no-candidate proven no-op: `suppressShallowSupportedFreeRootDrift`
  and `clearUnsupportedShallowSupportFreeRootVelocityStates` gate on the same
  `getCachedRootFreeJoint()` cache as `hasAnyShallowSupportFreeRootCandidate`.
- WP-PG.11: original stash added cache infra to ConstraintBase/ContactConstraint
  but never wired it; executor wired `getCachedContactConstraintCast()` into the
  resting-group classify loop (~:2064) and `isCachedExactContactConstraintType()`
  into 5 build-pass `isExactDynamicType<ContactConstraint>` sites (~:1174, 1359,
  1400, 1630, 1964). Operates on pooled ContactConstraints (cross-step win).
  The >=128-group threaded lambdas (~:2226/2236) left untouched (out of scope).
- Prior-art folds: `mRootSkeleton` is write-only repo-wide (skip safe);
  narrowed union-index reset safe because `uniteSkeletons()` never runs on the
  all-single-reactive path.
- gz compat: `.deps/gz-physics` only instantiates `WeldJointConstraint`, does
  not subclass `ConstraintBase`/`ContactConstraint`; `ConstraintSolver` already
  friends both — accessors stayed non-public.
- A/B (host-noisy): clean back-to-back S3/S4/S5 step-time ~5–25% better; S2
  insensitive (settled all-resting fast path dominates).

## Code-level root-cause narrowing (orchestrator, diff analysis)

- **WP-PG.31 RULED OUT as the race.** Its retained World member scratch
  (`mShallowSupportedFreeRootsScratch`, `mShallowSupportSkeletonToIndexScratch`,
  `mFreeRootVelocitySnapshotScratch`) is written at `World.cpp:1062-1084`,
  which is SERIAL — between the parallel "Integrate velocity" (1045-1056)
  and "Integrate positions" (1088+) blocks, on the single `World::step`
  thread. Not concurrent. (The prior free-function returned a local vector;
  the retained-member conversion is serial-safe.)
- **WP-PG.11 is the prime suspect.** Its cache is consumed INSIDE the
  16-thread region: `ContactConstraint::isCachedExactContactConstraintType()`
  is called at `ConstraintSolver.cpp:1638` inside
  `buildDefaultContactConstraintsForPair` (run via
  `mConstraintThreadPool->parallelFor` at :1682), and at :1408 inside
  `canBuildDefaultContactsByPairInParallel` (:1394) — the predicate that
  DECIDES whether to take the parallel build path. Shared-vector push_backs
  (`mActiveConstraints` :1387, `mContactConstraints` :1717) live in the same
  machinery. Two failure modes to confirm with the sanitizer stack:
  (a) the lazy cache write (`mCachedContactConstraintCast`/`...Cached`,
  `mCachedIsExactContactConstraintType`) races across threads on a shared
  pooled ContactConstraint — UB but idempotent-valued, so likely TSan-only,
  not heap-corrupting by itself; (b) PG.11's cached predicate changed the
  parallel-eligibility result vs the original `isExactDynamicType<>` check,
  enabling the parallel path (and its shared-vector push_back) in a case the
  original code kept serial → concurrent std::vector mutation → the observed
  `malloc(): invalid size`. (b) is the heap-corruption-consistent mechanism.
- **Fallback if PG.11 is hard to fix:** PG.30 + PG.31 are clean (deterministic
  + serial). Ship those two as PR-A and DROP or defer PG.11 to its own PR
  once the parallel-path interaction is fixed. This is the drop-the-regressor
  policy — do not ship a crash-risk to keep the consolidation "complete".

## RESOLUTION (2026-07-06): PR-A = WP-PG.30 only; PG.31 & PG.11 deferred

Bisect complete: **base 0/30, WP-PG.30-only 0/30, WP-PG.30+31 30/30** under
the controlled-load harness → **WP-PG.31 is the culprit** (surprising, since
its scratch writes are serial — likely its new `getCachedRootFreeJoint` call
pattern in `hasAnyShallowSupportFreeRootCandidate` exposing a latent
concurrency issue in the lazy Skeleton cache under load; exact stack pending).

**Decision:** ship **PR-A = WP-PG.30 only** (proven clean at 0/30, real wins).
Defer WP-PG.31 (`pr-c-pg31-scratch-retention.patch`) and WP-PG.11
(`pr-b-pg11-contact-cache.patch`) to their own PRs; each must root-cause its
concurrency and re-verify at 0/N under the harness (see memory
`pg31-parallel-crash`). The harness is the new required gate for any
parallel-path perf change — sanitizers + tests + gz are insufficient.

Reproduction harness (durable):
```
# 30 CPU stressors hold the load window; 30 runs; expect 0/N for a clean tree
for n in $(seq 30); do sha256sum /dev/zero >/dev/null 2>&1 & done
for i in $(seq 30); do BM_INTEGRATION_contact_container \
  --benchmark_filter='BM_ContactContainerActive/120/[01]/16$'; done
```

## (evidence) PR-A REGRESSION — controlled-load harness

A reliable reproduction was found: **30 background CPU stressors + 30 runs of
the 16-thread scene.** Results are now 100% deterministic:
- **PG.30+31: 30/30 crashes** (`malloc(): invalid size`).
- **BASE (736d116b731): 0/30 crashes** at load 39-47 (in/above the crash
  window — a FAIR high-load comparison, unlike the earlier ambient-load base
  test).
=> **PR-A introduced the crash; NOT pre-existing.** Sanitizers stay clean
(ASan masked by its serializing slowdown; TSan clean → the racing op is in an
uninstrumented path or the corruption is a glibc-heap double-free/OOB that
TSan's DART-only instrumentation doesn't see). The crash aborts during the
FIRST 16-thread benchmark's run before any result prints — the parallel
region, not startup physics per se.
- Bisect under this harness in progress: PG.30-only (30/30 → PG.30 culprit;
  0/30 → PG.31 culprit). Next: `MALLOC_CHECK_=3` + gdb (glibc aborts at first
  corruption, closest to source) to get the faulting stack, since sanitizers
  are blind to it.

## (superseded) SANITIZERS BOTH CLEAN — reframing to load comparison

- **ASan (Release+ASan, avoids the RelWithDebInfo `-fno-inline` link error):
  CLEAN** — full suite + 40× 16-thread, zero heap-buffer-overflow/UAF. Rules
  out a spatial memory bug.
- **TSan (Release+TSan): CLEAN** — dart-engine ×6 + ode-engine ×3
  16-thread runs, zero data-race reports. Rules out an unsynchronized
  concurrent access **in instrumented DART code** (happens-before, so
  load-independent).
- Implication: PR-A's code has no ASan/TSan-detectable defect. The
  reproducible `malloc(): invalid size` under high-load 16-thread must be
  either (1) PRE-EXISTING and exposed by load (the base test was at load
  30-44, never the 45-88 crash window — an unfair comparison), or (2) a
  sanitizer-blind path: DART's custom pool allocators (FreeListAllocator/
  PoolAllocator/MemoryManager) or the uninstrumented ODE library used
  concurrently. The `120/1/16` (ode-engine) rows are in the crash filter.
- **Decisive test in progress:** base vs PG.30+31 under IDENTICAL controlled
  high load (30 CPU stressors holding load in-window + 30 runs each). If both
  crash equally → not a PR-A regression (given sanitizers clean), proceed and
  file upstream. If only PG.30+31 crashes → PR-A regression via a
  sanitizer-blind mechanism, root-cause further.

## (earlier) PIVOT: PG.11 EXONERATED — testing base

The PG.30+PG.31 tree (PG.11 fully reverted) **also crashes** under 50×
16-thread stress: iters 39-50 all `malloc(): invalid size (unsorted)`,
core dumped, at load 45-62. So PG.11 is NOT the culprit, and the executor's
"each packet alone is clean" bisection was unreliable (load-dependent,
under-sampled — only heavy-stress tests are trustworthy). The mechanism
section below (PG.11 read-only→lazy-write) is a real latent race but NOT
the crash source.

**Decisive test in progress:** does UNMODIFIED `origin/release-6.20`
(736d116b731) crash under heavy CONCURRENT 16-thread stress (4 instances ×
25 rounds, ~64 threads / 32 cores to force the load-88 window)? If yes →
**pre-existing upstream concurrency bug in the round-1 parallel contact-build
path (#3183-#3194)**, PR-A exonerated, report upstream separately. If no →
bisect PG.30 (lazy Skeleton root-FreeJoint cache, touched near the parallel
integrate path) vs PG.31 (serial — unlikely).

Repo state during test: detached at base; PG.31 World.* changes parked in
`git stash` ("PG.31 World scratch (PR-A) — parked for base-crash test");
PG.30 commits safe on branch `wp-pg-30-single-free-body-cache`; PG.11
preserved as `pr-b-pg11-contact-cache.patch`.

## (superseded) Confirmed mechanism + DECISION (2026-07-05)

**Precise mechanism (diff-confirmed):** WP-PG.11 replaced the read-only free
function `isExactDynamicType<ContactConstraint>(ptr)` (inspects typeid only —
safe to call concurrently) with `ptr->isCachedExactContactConstraintType()`,
a member that **lazily WRITES** `mCachedIsExactContactConstraintType` on the
object; and `dynamic_cast<...>(constraint)` with
`constraint->getCachedContactConstraintCast()` (lazily writes
`mCachedContactConstraintCast`/`...Cached`). These are called INSIDE the
16-thread contact-build path (`ConstraintSolver.cpp:1408` in the
parallel-eligibility predicate, `:1638` in `buildDefaultContactConstraintsForPair`
run by `mConstraintThreadPool->parallelFor` at `:1682`). PG.11 turned a
read-only concurrent op into a writing one → data race in the parallel path,
consistent with the load-timing-dependent heap corruption. ASan build to
confirm is blocked by a PRE-EXISTING sanitizer-config breakage (BallJoint
`-fno-inline` undefined refs, unrelated to PR-A) — proper confirmation is
TSan, deferred to PR-B rather than blind-fixing (would be "patch around").

**DECISION — split, do not ship the crash risk:**
- **PR-A = PG.30 + PG.31 only.** Both proven clean: PG.30 deterministic
  (guard hashes), PG.31 serial (scratch writes at World.cpp:1062-1084, outside
  the parallel blocks). PG.11 reverted from the tree. Empirical stress
  verification (3 suites + 50× 16-thread) in progress to add weight to the
  structural argument.
- **PR-B = PG.11** (ContactConstraint pooled-cache + the two mined prior-art
  commits). Preserved as `pr-b-pg11-contact-cache.patch` in this folder
  (integrity-verified). PR-B must: reproduce under TSan (fix the sanitizer
  build first, or use a Release+ASan/TSan config without `-fno-inline`),
  make the cache population SERIAL (pre-populate before the parallel region,
  or per-thread), and prove clean under TSan + the 16-thread stress before
  shipping. This is the right home for a change touching the parallel path.

## Orchestrator plan (updated)

1. ~~Takeover + combined A/B~~ done: crash reproduced independently
   (`malloc(): invalid size`, Load 88), core dumped. Environmental REJECTED.
2. ~~ASan gate~~ blocked by pre-existing BallJoint `-fno-inline` link error;
   mechanism established by diff analysis instead; TSan moved to PR-B.
3. **PR-A (PG.30+31): stress-verify clean → adversarial review → commit
   PG.31 → gazebo gate → PR with evidence table.** (in progress)
4. PR-B (PG.11): separate, TSan-gated. Patch preserved.
