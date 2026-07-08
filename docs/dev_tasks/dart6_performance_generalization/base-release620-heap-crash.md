# contact_container benchmark crash (`free(): invalid size`) — investigation

**Status:** OPEN — earlier root-cause chain RETRACTED as a test-harness
artifact; re-establishing ground truth with verified binaries under controlled
load.
**Severity:** unresolved; likely LOW (no verified crash on a quiet host yet).

## Headline

A **later** detour concluded the `origin/release-6.20` base contains a
**deterministic single-thread heap bug** caused by #3297 "allocation hardening".
That detour was **wrong** — it rested on a missing-binary false positive. With
verified binaries the base is **clean** at single-thread (and under ASan). This
does **not** overturn the *original* finding; it reinforces it: the base is
clean, and the load-dependent 16-thread crash lives in the **WP-PG.31** packet
(by the original load-controlled bisect). For **WP-PG.20**, code analysis rules
out its *originally-suspected* mechanism (ODE collision is serial even at 16
threads), which makes its own 16-thread crash implausible — but that is a
code-level argument, not a run, so its **crash A/B stays a required pre-merge
gate** (see below).

Scope of the retraction: **only** the later "base has a deterministic
single-thread bug / #3297 is the cause" claims are void. The original
`120/[01]/16 + 30-stressor` bisect used real binaries (its `base 0/30` arm and
the real `malloc(): invalid size` messages prove the binaries existed) and is a
load-controlled relative comparison, so it stands.

## Why the earlier chain was void (two confounds)

1. **Missing-binary false positive.** The crash harness counted **any** non-zero
   exit of `$GB --benchmark_filter=…` as a crash. An earlier "clean rebuild"
   used `cmake --build … --target ALL`, which does **not** build
   `BM_INTEGRATION_contact_container` (wrong target name), so the binary was
   absent. Every run returned **exit 127** ("no such file or directory"),
   mis-counted as a crash. Proof: the captured `base_*.log` files contain only
   `no such file or directory: …`. This invalidates every "3/3 crashes" /
   "clean rebuild still crashes" result and all reasoning built on them
   (single-thread determinism, dart-and-ode symmetry, the `/1`-vs-`/16` "clue").

2. **Uncontrolled host load.** Throughout the investigation the host carried
   load ~34–43 from a *different checkout/lane on the same machine* building PR
   #3307. The only runs that produced **real** `free(): invalid
   size` aborts (WP-PG.20 tree, `120/1/16`, +30 stressors) ran at effective
   load ~64, so host-contention / OOM is not excluded.

## Established with VERIFIED binaries (still valid)

- Pure base (`38656f3dbea`, WP-PG.20 stashed), `120/0/1`, no stressors:
  **CLEAN — 0/5** (rc 0).
- Pure base under **ASan** (Release + `-fsanitize=address`), `120/0/1` forced to
  `--benchmark_min_time=40s`: **CLEAN — rc 0**, no AddressSanitizer report.
- **WP-PG.30 (#3310, merged) is not the corruption** — code argument
  independent of the harness bug: its diff is a pure version-keyed pointer cache
  (`Skeleton::getCachedRootFreeJoint()`) that allocates/frees nothing; worst
  case is a stale-but-valid `FreeJoint*` (wrong classification), which cannot
  corrupt glibc heap metadata. Consistent with the base-with-PG.30 being clean
  at single-thread.
- **#3297 hot scratch/pool sites hand-audited size-correct** (not a proof of
  overall correctness, only that these are not the bug): BoxedLcp thread scratch;
  the Dantzig `ldltRemoveTmp` buffer (`dEstimateLDLTRemoveTmpbufSize(n2,nskip) =
  (n2 + 2·nskip)·sizeof(Scalar)` exactly matches `dLDLTRemove`'s usage and is
  size-equivalent to the prior `nullptr` internal-vector path); and the World
  shallow-support scratch vectors (`resize()`d to `mSkeletons.size()` each step
  before indexed use at `World.cpp:1298/1496`).

## RETRACTED (were artifacts of confound #1)

1. ~~"The base crashes deterministically, single-threaded, on both engines."~~
   The base is clean at single-thread on a verified binary.
2. ~~"#3297 is the prime suspect / there is a base heap bug it introduced."~~
   No verified evidence; #3297 is no longer implicated by anything measured.
3. ~~"Thread/iteration determinism: crashes at `/1`, not `/16`."~~ The `/1`
   "crashes" were the missing binary; `/1` is actually clean.
4. ~~"WP-PG.20 is innocent because the base crashes without it."~~ The premise
   (base crashes single-threaded) is retracted. WP-PG.20's own 16-thread crash is
   made **implausible** by the code analysis below (ODE collision is serial), and
   its `120/1/16` "30/30" (real `free(): invalid size` under combined load ~64)
   is best explained by the missing-binary artifact + contention — but "ruled-out
   mechanism" is not "cleared". The crash A/B on a quiet host is still a required
   pre-merge gate.
5. ~~"WP-PG.31's attribution is confounded/re-opened by `/16`-vs-`/1`
   under-sampling."~~ That was a detour claim; it is itself void. The **original
   WP-PG.31 attribution stands**: `120/[01]/16 +30-stressor` bisect with real
   binaries gave `base 0/30`, `PG.30-only 0/30`, `PG.30+31 30/30` — same harness,
   same host load, only the code differs, so PG.31 is the load-dependent
   16-thread culprit. Base-clean-at-single-thread is consistent with (not
   contrary to) this.

## WP-PG.20: originally-suspected crash mechanism ruled out (code analysis — NOT a full clearance)

This is a code-level argument that the *specific* mechanism WP-PG.20 was
suspected of is absent. It is **not** a run-verified clearance — see the caveats.

- ODE `collide()` and `mContactHistory` access (`FindPairInHist`,
  `pruneContactHistory`) are invoked from `ConstraintSolver::solve()`, which
  `World` calls **serially** at `World.cpp:1264` — between the parallel
  velocity-integration (`parallelForIndexRange` @1246) and position-integration
  (@1278) regions. Collision detection runs once at the start of `solve()`,
  before the parallel constraint-**group** solve; the parallel work is the
  dynamics recursions and group solve, neither of which touches
  `mContactHistory`.
- So the originally-suspected mechanism — a **concurrent `unordered_map` rehash**
  from the `std::vector`→`std::unordered_map` swap — is implausible: the map is
  touched single-threaded. (Note: `unordered_map` also does *not* invalidate
  element references/pointers on insert or rehash, so the `FindPairInHist`
  returned reference is actually safer than the old `vector::back()`.) The
  `120/1/16` "30/30" is therefore best explained by the missing-binary artifact
  + base-under-load contention, not this code.

**Caveats — why this is "mechanism ruled out", not "exonerated":**
- I have **not** run the crash A/B on a verified WP-PG.20 binary vs base on a
  quiet host. The argument is code-reading only, and this session already showed
  a confident conclusion (base "3/3 crashes") collapse under a test artifact.
- `solve()`'s internals were **inferred** from `World::step`'s structure, not
  fully traced; if any collision/history call were reachable from a parallel
  region the argument would weaken.
- The new span-indexing (`result.getContact(i)` over `[pairContactsBegin,
  pairSpanEnd)`) is a **new logic path**; a bad index would be an out-of-bounds
  read that the determinism hash might not catch. The determinism guard + the
  crash A/B together cover this.

**Consequence:** the crash A/B stays a **required pre-merge gate** (below), not
"nice-to-have". WP-PG.20 also needs the ODE determinism guard, lint, and the A/B
benchmark table on a quiet host.

## Next steps

1. **WP-PG.20 crash A/B — REQUIRED before merge** (the code argument reduces the
   prior probability but does not replace the run). On a **quiet host** (load was
   ~82 from an external PR-#3307 build): one verified-binary run, pure base vs
   WP-PG.20, **default iterations** (matches the original fast-abort conditions,
   not `200x`), `120/1/16 +~30 stressors`, classifying real-heap SIGABRT vs
   environmental. Expected both-clean; a WP-PG.20-only real-heap crash would
   reopen the code analysis.
2. **Non-crash gates** (also quiet host): ODE determinism guard (bit-identical
   hashes), lint, A/B benchmark table. Then ship WP-PG.20.
3. Harness hardening (applied in `groundtruth.sh`): assert the binary
   exists+executable before any sweep; classify by rc/message (127=missing,
   134+heap-msg=real, 137/bad_alloc=environmental); never count bare non-zero
   exit as "crash". See [[crash-harness-verify-binary]].
