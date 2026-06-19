# Local Verification Pipeline

## Status

Proposal. Captures the durable design for a tiered, cache-aware local
verification workflow and the task-naming scheme it standardizes. Sequencing and
active rollout state belong in `docs/plans/` / `docs/dev_tasks/`, not here.

## Problem

`pixi run test-all` is the only well-known local gate, and it grows slower as the
project grows. Contributors either pay its full cost on every check or invent
ad-hoc shortcuts. Two things are missing: (1) a small, predictable set of
verification _tiers_ so a contributor can pick the cheapest check that still
proves what they changed, and (2) a build/test pipeline that is actually fast —
fully using the compiler cache, parallelism, and linker the toolchain already
ships.

## Findings (measured)

Measured on a default Release build (2026-06-05). These numbers drive every
decision below, so they are recorded here as the evidence basis.

| Observation                        | Measurement                                                                                                        | Implication                                                                                    |
| ---------------------------------- | ------------------------------------------------------------------------------------------------------------------ | ---------------------------------------------------------------------------------------------- |
| Full C++ suite, **sequential**     | 343 tests, **580.6 s**                                                                                             | Sequential test execution alone is ~10 min                                                     |
| ├ 5 simulation long-pole binaries  | **524 s** of the 580 s (`test_rigid_ipc_paper_experiments` alone **352 s**)                                        | One outlier bounded the whole _parallel_ suite before CTest sharding                           |
| └ the other 338 tests, sequential  | **56.1 s**                                                                                                         | Routine test work is cheap once the long poles are excluded                                    |
| Tier-1 core (unit + integration)   | 279 tests, **48.6 s** sequential                                                                                   | Excludes the simulation long-pole tests                                                        |
| Tier-0 unit (`^UNIT_`, parallel)   | 184 tests, **0.5 s**                                                                                               | Ideal inner-loop content                                                                       |
| Link 241 test exes, `bfd` → `mold` | **3.0 s → 1.5 s (2.0×)** on the real GUI build                                                                     | mold halves link time and links the prebuilt Filament archive cleanly                          |
| `pixi run test-all` build work     | lint(autofix) → build Release → build-tests → build-examples → build **Debug** → unit → simulation → python → docs | The full gate also rebuilds Debug + examples + tutorials + docs                                |
| sccache                            | 47 % hit, 10 GiB cache 100 % full, shared across 15+ clones (each `build/` 14–31 GB)                               | Undersized for intra-clone reuse; cross-clone reuse blocked (see below)                        |
| Linker                             | `mold` 2.40.4 installed, **not wired** (default `bfd`); ~340 link targets                                          | Linking is a large, avoidable cost                                                             |
| Build parallelism                  | `cmake_build.py` uses ¾-core cap; `build_helpers.py` uses `os.cpu_count()`; neither emits ninja `-l`               | Contradictory defaults; no load governor                                                       |
| Test parallelism                   | `test` / `test-simulation` call plain `ctest` with no `--parallel`                                                 | Tests run **sequentially** by default                                                          |
| Baseline ctest properties          | no `TIMEOUT`, `PROCESSORS`, `RESOURCE_GROUPS`, `--rerun-failed`, link `JOB_POOL` anywhere in the 2026-06-05 scan   | Missing standard fast-iteration ergonomics; long-pole shards now add targeted `COST`/`TIMEOUT` |

Follow-up measurements on the same workstation (2026-06-19):

| Check                                      | Before                                                                                      | After                                                                                                         | Result                                                                  |
| ------------------------------------------ | ------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------- |
| Default `test-simulation-full` CTest phase | monolithic tail: `test_rigid_ipc_paper_experiments` **403.19 s**, `test_world` **219.71 s** | shard tail: **182.97 s** total CTest, longest shard `test_rigid_ipc_paper_experiments_turntable` **182.96 s** | ~55 % shorter tail for the default `pixi run test-all` simulation phase |
| Default merged-tree validation             | `origin/main` advanced during the change and expanded `test_world` from 373 to 414 cases    | **529.88 s** total CTest; longest shard `test_rigid_ipc_paper_experiments_friction` **278.92 s**              | all individual shards remain below the 5 min split threshold            |
| CUDA `test-simulation-full` CTest phase    | broad `-L simulation` also selected 8 `simulation-cuda` tests: **252.12 s**, 88 selected    | exact `-L '^simulation$'`: **121.21 s**, 81 selected after the zero-case paper catch-all shard                | avoids duplicate CUDA tests in `pixi run -e cuda test-all`              |
| CUDA `test-cuda` CTest phase               | `test_lcp_jacobi_batch_cuda` standalone/serial profile: **241-256 s**                       | kept as one scheduled test with `COST`/`TIMEOUT`; small CUDA tests overlap beside it                          | below the 5 min split threshold; sharding worsened GPU contention       |

**Reframe:** two costs dominate routine local verification — the simulation
**long-pole tests** (524 s, and `test-all` runs them every time) and the **cold
build** (Release + Debug + examples + docs). Tiering removes both from the inner
loop (`verify-quick` runs the 0.5 s unit pass; `verify` runs the 48.6 s core
without the long poles), while mold, the resized compiler cache, load-aware
parallelism, and CTest shards for the heaviest simulation binaries cut what
remains. Raw `-j` on the _full_ suite barely helped when a single 352 s CTest
entry bounded it, which is exactly why the tiers exclude those entries and the
full tier now splits them.

## Task naming scheme

The scheme has two orthogonal axes plus the existing pixi environment axis. The
goal is that a contributor can guess the right task without consulting a list.

### Axis 1 — tier (depth / speed), monotonic supersets

| Suffix        | Tier | Intent                                   | Wall-clock target |
| ------------- | ---- | ---------------------------------------- | ----------------- |
| `-quick`      | 0    | inner loop, every save / agent iteration | seconds (warm)    |
| _(bare verb)_ | 1    | pre-commit default                       | ~1–3 min (warm)   |
| `-full`       | 2    | pre-push / authoritative gate            | minutes           |

The unadorned verb is the everyday default; `-quick` is the faster subset and
`-full` is the exhaustive superset. Each tier is a strict superset of the one
before it, so passing a higher tier always implies the lower tiers pass.

### Axis 2 — action (verb)

| Verb     | Meaning                                                          |
| -------- | ---------------------------------------------------------------- |
| `build`  | compile only                                                     |
| `test`   | run tests (assumes built)                                        |
| `verify` | composite gate: lint + build + test (+ docs/examples at `-full`) |
| `bench`  | run benchmarks                                                   |

`verify` is the umbrella for "local verification": it is what `test-all` is today,
generalized across tiers. `test` stays the pure test runner.

### The matrix

| Action ↓ \ Tier →                 | Tier 0 (`-quick`)     | Tier 1 (bare) | Tier 2 (`-full`)                             |
| --------------------------------- | --------------------- | ------------- | -------------------------------------------- |
| **verify** (lint+build+test gate) | `verify-quick`        | `verify`      | `verify-full`                                |
| **test** (tests only)             | `test-quick`          | `test`        | `test-full`                                  |
| **bench**                         | `bench-quick`         | `bench`       | `bench-full`                                 |
| **build**                         | `build` (incremental) | `build`       | `build-full` _(Release+Debug+examples+docs)_ |

### Axis 3 — scope (subsystem), orthogonal to tier

Scoped runners select a subsystem instead of a tier and keep the existing
`<verb>-<scope>` shape:

```
test-math   test-collision   test-io   test-py   test-cuda
test-simulation   bench-collision   bench-lcpsolver
```

Tier words (`quick`, `full`) are reserved and are never subsystem names, so
`test-quick` (tier) and `test-math` (scope) never collide. New subsystems extend
the scheme without touching it.

### Environment axis (unchanged)

Environment stays a pixi `-e` flag, never baked into a name:

```
pixi run verify            # default env, Tier 1
pixi run verify-quick      # default env, Tier 0
pixi run -e cuda verify-full
pixi run -e gazebo test-gz
```

### Backward-compatible aliases

To avoid breaking CI, muscle memory, and existing docs, keep these as thin
aliases of the new names:

| Existing         | Becomes alias of |
| ---------------- | ---------------- |
| `test-all`       | `verify-full`    |
| `test-unit`      | `test-quick`     |
| `bm`, `bm-check` | `bench*` family  |

## Tier contents

Each tier is a strict superset. Boundaries follow the measured numbers, so the
simulation long tail is split out of Tier 1 (it alone is ~60 s of the 68 s
simulation total and would blow the Tier-1 budget). Tier 2 still runs the long
tail, but the largest simulation binaries are split into filtered CTest shards
so one GTest executable no longer occupies a single CTest worker for the whole
paper-scale corpus.

|                      | Lint                      | Build                                    | Tests                                                          | Extras                                                           |
| -------------------- | ------------------------- | ---------------------------------------- | -------------------------------------------------------------- | ---------------------------------------------------------------- |
| **Tier 0** `*-quick` | `check-lint` (no autofix) | incremental Release lib + UNIT test exes | `^UNIT_` minus long poles (`UNIT_simulation_World`)            | cheap correctness gates (see below)                              |
| **Tier 1** bare      | `lint`                    | Release                                  | + `INTEGRATION_` + `simulation-quick` subset + python (dartpy) | —                                                                |
| **Tier 2** `*-full`  | `lint`                    | Release **+ Debug**                      | all tests incl simulation long tail                            | examples, tutorials, docs, ASan, eigen-overalignment, (cuda env) |

### Long-test threshold policy

Use Bazel's public timeout bands as calibration, not as a build-system
dependency: short/small is 1 min, moderate/medium is 5 min, long/large is 15
min, and eternal/enormous is 60 min. For DART local verification:

- keep ordinary CTest entries under 60 s when practical;
- track any 60-300 s entry as a long-pole watchlist item and give it explicit
  `COST` when it stays in Tier 2 or a scoped full gate;
- split, optimize, or move any Release full-gate CTest entry expected to exceed
  300 s before it lands in `test-all` or `pixi run -e cuda test-all`;
- keep any expected >900 s case out of routine local validation unless it has a
  documented full-gate reason; those usually belong in benchmark, nightly, or
  manual validation surfaces.

The 5 min threshold is therefore the action point, not the definition of "slow":
above 1 min the test is visible to scheduling and developer latency, and above
5 min it must be split or justified. CTest `COST` and per-test `TIMEOUT`
properties are the local scheduling/guard rails; GoogleTest filters are the
preferred shard mechanism for large GTest binaries.

**Cheap correctness gates belong in Tier 0/1, not only Tier 2.** The pure-Python
AST scans (e.g. `check-simulation-public-header-smoke`, the API-boundary
linters, `check-dart7-promotion-surface`) cost well under a second and catch
real breakage; if they live only in the full gate, "quick is green" misleads.

**Tier 2 stays authoritative, but its skips must be loud.** `test_all.py` today
prints "Ready to submit PR!" even when it silently skipped python / examples /
cuda because a CMake flag was off. As the only full-coverage tier, a skip caused
by local misconfiguration must be reported as a failure-by-policy, not a warning.

## Cross-cutting design

### Parallelism (unify, then make it load-aware)

There must be exactly one build-parallelism policy. Today `cmake_build.py`
(¾-core cap) and `build_helpers.py` (`os.cpu_count()`) disagree, and neither
emits a load governor. The design:

- One resolver controls both build and test parallelism, honoring
  `DART_PARALLEL_JOBS` (hard cap) and falling back to a sensible core fraction.
- Default to **load-aware** scheduling so concurrent clones/agents share the CPU
  with no central coordinator: `ninja -l <N>` for builds and `ctest --test-load
<N>` for tests, where `N` ≈ logical cores. Because `cmake --build --parallel`
  cannot express `-l`, the build path must pass the ninja flag through (or invoke
  ninja directly) — the load limit is a real plumbing change, not a config knob.
- Guarantee a floor of ≥1 running job regardless of load so a busy machine
  throttles instead of wedging (ninja's 1-minute load average lags).
- A GNU Make–style jobserver/token scheme across clones is the fuller solution;
  adopt it only if load-average sharing proves insufficient in practice.

A host-local `~/.dart-dev/` config (not in the repo) may pin a machine-wide cap
read by the resolver, so every clone on one workstation shares one policy.

### Compiler cache

- **Size for intra-clone reuse.** The default 10 GiB cache thrashes across many
  large clones. Raising the ceiling (host-local, e.g. via
  `~/.config/sccache/config`) and enabling preprocessor-cache mode improves reuse
  _within_ a clone (rebuilds, config switches, rebases) — the common loop.
- **Cross-clone reuse is a separate, harder win.** With `-S .`, CMake bakes
  per-clone absolute `-I` paths into every compile command, so objects from one
  clone never hit for another regardless of cache size. Genuine cross-clone
  sharing needs `-ffile-prefix-map` / `-fmacro-prefix-map` normalization of the
  per-clone source root to one shared token. Validate the `sccache --show-stats`
  miss-reason breakdown before assuming a sizing change moves the hit rate.
- The cache only ever accelerates **compile**, never link — see the linker and
  job-pool items below.

### Linker (mold) — scoped and gated

Wiring `mold` is the largest build win after the cache (340 link targets), but
`CMAKE_LINKER_TYPE=MOLD` is global and two link paths are fragile:

- Filament links **prebuilt static archives** (`cmake/dart_find_filament.cmake`);
  a different linker over third-party `.a` files is the canonical silent-breakage
  case.
- The CUDA path does hand-rolled system-compiler / glibc-sysroot linker surgery
  (`pixi.toml` config-py); a global linker change interacts with its `-Wl` flags.

Therefore: enable mold for non-GUI, non-CUDA configurations first, behind a flag,
and gate adoption on **runnable** GUI and CUDA smokes (not merely "it linked"),
plus confirmation that the toolchain compiler accepts `-fuse-ld=mold`.

### Test ergonomics

- Default `ctest` to parallel with `--test-load`, and set a per-test default
  `--timeout` (none exists today; one hung test stalls a whole tier).
- Expose `--rerun-failed` for the inner loop — a large, cheap human win.
- Add a link `JOB_POOL` (`CMAKE_JOB_POOL_LINK`): at high `-j` the link phase, not
  compile, is the memory-pressure risk, and the cache does not help linking.
- Set `PROCESSORS` on known multi-threaded heavy tests so `--test-load`
  accounts for them.

### Build shape (avoid)

Unity builds shrink the cache hit rate (any member change invalidates the whole
translation unit) and work against the caching goal; do not enable them for
cached configurations. Evaluate PCH only with measurement.

## Recommended sequencing

Ordered by leverage and risk, lowest-risk enabling work first:

1. Unify the parallelism resolver and plumb `-l` / `--test-load` / `--timeout` /
   `--rerun-failed` and a link job pool. This is the biggest, lowest-risk win and
   it stabilizes the `-j` base that later measurements depend on.
2. Validate and tune the compiler cache (miss-reason histogram first; size and
   preprocessor mode; then `-ffile-prefix-map` normalization if cross-clone reuse
   is the dominant miss).
3. Scope and gate `mold` per the linker section.
4. Introduce the tiered `verify-*` / `test-*` / `bench-*` tasks and aliases, then
   update `docs/onboarding/testing.md` to teach the tier ladder.

Each step is validated empirically before it lands: cold/warm wall-clock deltas,
the sccache miss-reason breakdown, runnable GUI/CUDA smokes for mold, a
flip-every-flag check if any reconfigure-skip optimization is attempted, the
Tier-1 wall-clock with the simulation subset, and a concurrent multi-clone
throughput measurement for the load-aware default.

## Non-goals

- A bespoke cross-clone build scheduler/daemon. Start with load-average sharing.
- Changing test _content_ or coverage. This is about _when_ and _how_ tests build
  and run, not what they assert.
- A reconfigure-skip "stamp" keyed on file mtimes. CMake already reconfigures on
  input change; a naive stamp would skip needed reconfigures when an environment
  override flips, and the ~1.6 s saved is not worth that risk.

## Open questions (unverified)

- Whether the toolchain compiler in the pixi env accepts `-fuse-ld=mold`.
- The realistic warm Tier-0 / Tier-1 wall-clocks on a cold-ish tree.
- The sccache miss-reason breakdown (eviction vs not-found vs per-clone path
  hash) and therefore the true ceiling of a size-only change.
