# HANDOFF — WP-PG.20 (ODE contact-history perf) + crash-investigation resolution

**Date:** 2026-07-06 · **Status refresh:** 2026-07-07 · **Branch:**
`wp-pg-20-ode-history-spans` (merged as PR #3329; current
`origin/release-6.20` includes it) · **For:** any fresh AI agent
(Codex/Claude/other) resuming this lane.

This branch's WP-PG.20 code is merged. The gate and crash-investigation notes
below are retained as audit history for future ODE work, not as an active PR
checklist. Read [RESUME.md](RESUME.md) and
[07-orchestration-dashboard.md](07-orchestration-dashboard.md) for the next
packet.

---

## 1. Goal & north star (the `/goal`)

> **Optimize and generalize performance for DART 6**, continuing
> https://github.com/dartsim/dart/issues/3056, per the plan in
> `docs/dev_tasks/dart6_performance_generalization/`.

Hard rules:
- **Evidence-based only.** Every perf PR must carry a benchmark table with three
  columns — *very-first-baseline* vs *base/parent* vs *this-PR* — for **ODE and
  native (DART) collision** cases. Do all necessary A/B testing. Call out
  "unknown unknowns" explicitly.
- **Backward compatibility for gz-physics / gz-sim is CRITICAL.** ODE and native
  collision results must stay **bit-identical** (determinism guard) unless a
  behavior change is explicitly justified.
- **Fewer, bolder PRs.** No evidence-only / intermediate-scaffold PRs (the user
  rejected #3270 for this). A PR must be a real perf-code improvement with its
  evidence table inline.
- DART 6 = `release-6.20` LTS, **C++17**. DART 7 = `main`, C++23. Dual-PR to
  `main` only if the same code exists there.
- Determinism guard: bit-identical `contact_benchmark` final-state hashes per
  collision detector; untouched detectors must stay bit-identical.

**Orchestration model:** you are orchestrator/supervisor. Delegate focused
implementation + A/B to Codex (`codex:codex-rescue` subagent / `/codex:*`),
ensuring Codex works in the correct dir + branch, runs under `/goal`, and does
its own A/B. Follow the durable approval boundaries in
`docs/onboarding/ai-tools.md`: pushes, PR edits/comments, CI reruns, and thread
resolution require explicit maintainer/user approval. **Merges stay with the
maintainer** (the user, jslee02).

---

## 2. What WP-PG.20 is (the code on this branch)

The final PR keeps WP-PG.20 to **span-only** ODE contact-history bookkeeping in
`dart/collision/ode/OdeCollisionDetector.{hpp,cpp}` (serial collision path only
— see §4). The pair-keyed map/pruning variant was measured separately and
deferred back to WP-PG.21 because it added small-row overhead on the refreshed
current-base matrix.

1. **Contact-span tracking** — `reportContacts()` used to do
   `auto results_vec_copy = result.getContacts();` (a **full copy of the growing
   contact vector, once per pair**) and re-scan it three times. Replaced with
   `pairContactsBegin = result.getNumContacts()` before appending this pair's
   contacts, and `pairSpanEnd = begin + contactsToCopy`, then indexing
   `result.getContact(i)` over `[begin, end)`. Valid because ODE visits each
   object pair at most once per `collide()` call, so a pair's contacts are one
   contiguous span.
2. **Avoid unused history entries** — if ODE already emits the target manifold
   for a pair and that pair has no prior history, `reportContacts()` now returns
   before creating a new `ContactHistoryItem`.
3. **Empty-result cleanup** — `pruneContactHistory()` clears the vector-backed
   history when the current result has no contacts. The remaining non-empty
   prune path intentionally stays vector-backed for WP-PG.21 to revisit with
   fresh evidence.

The final implementation keeps `mContactHistory` as `std::vector` and does not
introduce `std::unordered_map` or `CollObjPairHash`.

---

## 3. Status of the crash investigation (RESOLVED — read carefully)

A long detour concluded WP-PG.20 (and even the `release-6.20` base) had a
deterministic heap crash. **That was wrong — a test-harness artifact.** The full
corrected record is in
[`base-release620-heap-crash.md`](base-release620-heap-crash.md). Summary:

- **The "base crashes 3/3" result was a missing-binary false positive.** An
  earlier rebuild used `cmake --build … --target ALL`, which does **not** build
  `BM_INTEGRATION_contact_container` (wrong target name). The binary was absent,
  every run returned **exit 127**, and the crash harness mis-counted non-zero
  exit as a crash. Proof: captured logs contained only `no such file or
  directory`.
- **With a verified binary the base is CLEAN** at single-thread (`120/0/1`,
  0/5) and under ASan (Release+ASan, forced `--benchmark_min_time=40s`, rc 0, no
  report).
- **WP-PG.20's originally-suspected crash mechanism is RULED OUT by code
  analysis (host-independent) — but this is not a run-verified clearance:** ODE
  `collide()` and all `mContactHistory` access run inside
  `ConstraintSolver::solve()`, which `World` calls **serially** at
  `World.cpp:1264` — between the parallel velocity-integration
  (`parallelForIndexRange` @~1246) and position-integration (@~1278) regions.
  Even at `setNumSimulationThreads(16)`, collision detection is single-threaded;
  only the dynamics recursions and the constraint-**group** solve parallelize,
  and neither touches `mContactHistory`. The earlier **concurrent
  `unordered_map` rehash** suspicion applied only to a discarded WP-PG.21-style
  map variant; the final PR keeps vector-backed history. The `120/1/16` "30/30"
  is best explained by the missing-binary artifact + contention.
  **Caveats (do not read this as "cleared"):** the crash A/B was never run on a
  verified WP-PG.20 binary; `solve()`'s internals were inferred from `World::step`
  not fully traced; and the new span-indexing (`result.getContact(i)`) is a fresh
  logic path whose out-of-bounds read a determinism hash might miss. → the crash
  A/B is a **required** pre-merge gate (§5 Gate D), not optional.
- **What still stands (unrelated to WP-PG.20):** the **WP-PG.31** load-dependent
  16-thread crash (deferred; patch preserved — see §6). Its original bisect
  (`base 0/30`, `PG.30 0/30`, `PG.30+31 30/30` at `120/[01]/16` + 30 stressors,
  real binaries, load held constant) is a valid relative comparison.

**Consequence:** WP-PG.20 has **no *identified* crash blocker**, but its crash
status is confirmed only by the code argument above — run the §5 gates (incl. the
required crash A/B) on a quiet host before merge.

---

## 4. Threading model reference (why §3's mechanism-rule-out holds)

`World::step()` (deactivation-disabled path, `dart/simulation/World.cpp`):
- ~1246 `parallelForIndexRange(... computeForwardDynamics + integrateVelocities)` — PARALLEL
- ~1260 `snapshotFreeRootVelocities()` — serial
- **~1264 `mConstraintSolver->solve()`** — SERIAL (collision detect + LCP). ODE
  `collide()` / `mContactHistory` happen here.
- ~1267 `findShallowSupportedFreeRoots(...)` — serial (this is the PG.30/PG.31 area)
- ~1278 `parallelForIndexRange(... integratePositions + suppressShallowSupportedFreeRootDrift)` — PARALLEL

So: collision = serial; only dynamics + LCP-group-solve = parallel.

---

## 5. Remaining work for WP-PG.20 (needs a QUIET host)

The host was saturated (load ~82) by an external PR-#3307 build sharing the
machine, which blocks trustworthy benchmark/determinism numbers. On a quiet host
(check `pgrep -c cc1plus` ≈ 0 and `uptime` load < ~10):

**Gate A — Determinism (bit-identical).** Build `contact_benchmark`, run these
three scenes on **base (parent)** and on **WP-PG.20**, and require identical
`Final State Hash`. WP-PG.20 must not change ODE *or* DART results.
- Recorded reference hashes (re-derive from the base arm to be safe):
  - `S2_ode = 0x10f80b0408cede90`
  - `S4_ode = 0x429b65bc5c4a14b6`
  - `S3_dart = 0xcf0ba6eaa97be038` (untouched detector — must be unchanged)
- Scene commands (binary: `build/default/cpp/Release/bin/contact_benchmark`):
  - S2_ode: `contact_benchmark .deps/gz-sim/examples/worlds/3k_shapes.sdf --steps 3000 --sdf-plane-shapes --quiet --checkpoint 0 --collision ode --world-threads 1 --max-contacts 12000 --max-contacts-per-pair 4`
  - S4_ode: `contact_benchmark --generate-objects 900 --steps 300 --warmup 0 --checkpoint 0 --quiet --collision ode --world-threads 16 --max-contacts 20000 --max-contacts-per-pair 4`
  - S3_dart: `contact_benchmark .deps/gz-sim/examples/worlds/3k_shapes.sdf --steps 300 --disable-deactivation --sdf-plane-shapes --quiet --checkpoint 0 --collision dart --world-threads 16 --max-contacts 12000 --max-contacts-per-pair 4`
  - Hash extract: `grep -oE 'Final State Hash:\s+0x[0-9a-f]+' | awk '{print $NF}'`

**Gate B — Lint.** `pixi run check-lint` (or `pixi run lint` to auto-fix). Must
be clean.

**Gate C — A/B benchmark table (the deliverable).** Compare **base(parent)** vs
**WP-PG.20** on ODE rows, and pull the *very-first-baseline* column from
`01-baseline-evidence.md`. Use the same `contact_benchmark` ODE scenes (RTF +
Avg Step Time) plus the GB rows:
`BM_INTEGRATION_contact_container --benchmark_filter='BM_ContactContainerActive/120/1/1$' --benchmark_repetitions=5 --benchmark_report_aggregates_only=true`
(and `120/1/16` if desired). Expect WP-PG.20 ≥ base on ODE rows; native (DART)
rows unchanged. Build targets (NEVER `--target ALL`):
`pixi run cmake --build build/default/cpp/Release --target contact_benchmark BM_INTEGRATION_contact_container --parallel 8`.

**Gate D — crash A/B (REQUIRED before merge).** The §3 code argument lowers the
prior but does not replace the run. One verified-binary run (`[ -x "$GB" ]`
first), base vs WP-PG.20, **default iterations**, `120/1/16` +~30 stressors,
classifying real-heap SIGABRT vs environmental. Expected both-clean; a
WP-PG.20-only real-heap crash reopens §3.

**A/B mechanics — the shared base↔WP-PG.20 swap for Gates A, C, and D**
**(WP-PG.20 is COMMITTED — do NOT use `git stash`, the working tree is clean so
it would stash nothing and the two arms would be identical).**
Use the current branch tip as the WP-PG.20 arm and current
`origin/release-6.20` as the base arm; do not hard-code older pre-merge SHAs.
Swap only the two ODE files between arms:
- **WP-PG.20 arm:** on the branch tip as-is -> build the *specific* targets -> run.
- **base arm:** restore just the two files from the current base:
  `git restore --source=origin/release-6.20 --worktree -- dart/collision/ode/OdeCollisionDetector.cpp dart/collision/ode/OdeCollisionDetector.hpp`
  -> rebuild the specific targets -> run -> then restore WP-PG.20 with
  `git restore --source=HEAD --worktree -- dart/collision/ode/OdeCollisionDetector.cpp dart/collision/ode/OdeCollisionDetector.hpp`.
  (`git diff --stat` should be empty again afterward.)

**Always assert `[ -x "$GB" ]` before running** and classify failures by
rc/message (127=missing-binary, 134+`free(): invalid size`=real,
137/`bad_alloc`=environmental, 124=timeout). A ready harness with these guards
is in the scratchpad as `groundtruth.sh`; the older gate logic is in
`wppg20_gates.sh` (has the determinism refs above) — note that harness predated
the commit and used the now-wrong `stash` approach; use the `checkout`-based
swap above instead.

PR #3329 is merged. Use the evidence table there as the reference format for
future performance PRs that change runtime behavior.

---

## 6. Other packets & PR states

- **WP-PG.30** (single-free-body root-joint cache) — **MERGED as #3310**
  (`38656f3dbea`, the base of this branch). Clean.
- **WP-PG.31** (World shallow-support scratch retention) — **DEFERRED**; causes
  a real load-dependent 16-thread heap crash (see
  [`project memory pg31-parallel-crash`] / `base-release620-heap-crash.md`).
  Preserved as `pr-c-pg31-scratch-retention.patch`. **⚠ This patch does NOT
  apply to the current base** — `git apply --check` fails (`patch does not apply`
  at `World.cpp:171`, `World.hpp:473`). It was diffed against the pre-#3297
  `wp-pg-30` consolidation tree, and #3297 "allocation hardening" rewrote the
  same `World.cpp`/`World.hpp` shallow-support region (+315/+79 lines). **Treat
  it as a reference snapshot, not an applicable patch:** either
  `git apply --3way pr-c-pg31-scratch-retention.patch` and resolve the
  World.cpp/.hpp conflicts by hand, or (recommended) re-implement the
  scratch-retention idea fresh against the current `World.cpp` allocator
  hierarchy. Root-cause the 16-thread crash (parallel-dynamics path) before
  shipping either way.
- **WP-PG.11** (ContactConstraint pooled-cache) — DEFERRED; preserved as
  `pr-b-pg11-contact-cache.patch`. **This patch does NOT apply cleanly** to the
  current base; `git apply --check` fails at
  `dart/constraint/ConstraintSolver.cpp:2152`. Treat it as a reference
  snapshot and either refresh the patch against current `release-6.20` or
  re-implement the idea fresh. Re-verify the wiring sites after #3297's
  `ConstraintSolver.cpp` changes, and confirm the parallel-eligibility concern
  in `pr-a-crash-investigation.md` before shipping.
- **#3311** (arm64/clang `-Werror` CI fix, `[[maybe_unused]]`) — **CLOSED without
  merge** by the maintainer (was green). Confirm whether the arm64 build is
  still broken; if so a replacement may be needed. (#3267 for a prior instance
  was merged.)
- Plan lanes still open: WS-B (ODE backend), WS-C (dynamics batching/alloc),
  WS-D (SIMD), WS-F native collision port (#3234, external owner). See
  `README.md` + `07-orchestration-dashboard.md` in this folder.
- Open maintainer decisions: D3 (matrix-free large-island solver), D7
  (penetration-creep pile-sleep closer).

---

## 7. How to resume (checklist)

1. Start from [RESUME.md](RESUME.md), not the merged WP-PG.20 branch.
2. Claim the next open packet in
   [07-orchestration-dashboard.md](07-orchestration-dashboard.md) on a fresh
   branch from current `origin/release-6.20`.
3. For any runtime performance PR, copy the #3329 evidence shape: recorded
   baseline, parent/current-base, PR head, exact commands, guard hashes/counts,
   and explicit unknowns.
4. If revisiting WP-PG.21, compare against the merged span-only WP-PG.20
   baseline and require a fresh current-base matrix before claiming the packet.

---

## 8. Environment gotchas (do not relearn these the hard way)

- **NEVER `cmake --build --target ALL`** for benchmarks — it silently builds
  nothing and leaves a stale/absent binary. Use specific target names.
- **A missing binary fakes "N/N crashes"** in any harness that treats non-zero
  exit as a crash. Assert `-x $GB` first; classify by rc/message. (This trap
  cost this session a multi-hour phantom-crash detour.)
- **Host contention from a sibling checkout** on the same machine can push load
  to ~98 and make every 16-thread run time out / OOM — meaningless timing/crash
  results. Run decisive A/Bs on a quiet host, back-to-back, verified binaries.
- Sanitizers: `pixi run config-asan` breaks on a BallJoint `-fno-inline` link
  error. Workaround = a Release build dir with
  `-DCMAKE_CXX_FLAGS='-fsanitize=address -fno-omit-frame-pointer -g'` (+ matching
  C/linker flags), `-DDART_USE_SYSTEM_GOOGLEBENCHMARK=ON`,
  `-DDART_USE_SYSTEM_GOOGLETEST=ON`. ASan's slowdown reduces GB iteration count —
  force iterations with `--benchmark_min_time=Ns` when hunting rare bugs.
- Local build OOM under host contention: use `DART_PARALLEL_JOBS=8`
  (see project memory `dart-local-build-gotchas`).
- `pixi run` tasks: `config`, `build`, `lint`, `check-lint`, `test-all`,
  `test-gz`, `config-asan`.
- Determinism guard uses `contact_benchmark`; the crash/perf GB suite is
  `BM_INTEGRATION_contact_container` (`BM_ContactContainerActive/<size>/<engine 0=dart,1=ode>/<threads>`).
- Branch rule: local and remote branch names MUST match; create local off the
  base, push `-u` under the identical name.
- Commit/PR rule: **no AI attribution** (no `Co-Authored-By: Claude`, no
  "Generated with Claude Code" footer).

---

## 9. Key files & references

- This handoff: `HANDOFF-wp-pg-20.md`
- Corrected crash record: `base-release620-heap-crash.md`
- Historical (superseded-in-part) note: `pr-a-crash-investigation.md`
- Preserved packets (see §6 for applicability): `pr-b-pg11-contact-cache.patch`
  (does NOT apply cleanly to current base — reference snapshot; refresh or
  re-implement), `pr-c-pg31-scratch-retention.patch` (does NOT apply to the
  current base — reference snapshot; needs `--3way`/re-implement vs #3297)
- Plan: `README.md`, `01-baseline-evidence.md` (very-first-baseline numbers),
  `07-orchestration-dashboard.md`, `RESUME.md`
- Code: `dart/collision/ode/OdeCollisionDetector.{hpp,cpp}` (WP-PG.20),
  `dart/simulation/World.cpp` (threading model, §4),
  `dart/dynamics/Skeleton.{hpp,cpp}` (WP-PG.30 cache, merged)
- Issue: https://github.com/dartsim/dart/issues/3056
