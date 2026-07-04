# WP-PG.40 — FP-determinism + ISA delivery contracts (D1/D2)

> **Status: awaiting maintainer ratification (D1, D2).** This document is the
> design output of WP-PG.40 (see
> [05-simd-enablement-lane.md](05-simd-enablement-lane.md)). It proposes
> contracts for the two open decisions blocking WP-PG.41+ and records the
> concrete evidence gathered while drafting them. Nothing in this document
> changes `dart/simd` or any consumer code; it is docs + a compile-only
> prototype (see [prototypes/](prototypes/)).

## Summary of findings

Investigating D1 turned up three separate, concrete sources of FP
non-determinism relevant to the "bit-identical scalar/SIMD" goal — one of
them already latent in code merged and CI-tested today, not merely a future
risk:

1. **Compiler auto-contraction is orthogonal to `dart::simd` and already
   live in the build.** GCC 15.2 and Clang 22 (this host's toolchain) both
   fuse plain `a*b+c` into a single hardware FMA instruction by default
   whenever the target ISA supports it — no `-ffast-math` or
   `-ffp-contract=fast` needed — and do not fuse otherwise. Confirmed by
   compiling a one-line probe to assembly (below). This means the existing
   `DART_ENABLE_SIMD` option (`CMakeLists.txt:139`, off by default,
   `-march=native` on the whole `dart` target when on) already silently
   changes the rounding of *every* multiply-add-shaped expression in DART's
   floating-point code the moment it is enabled — not just `dart::simd`
   kernels.
2. **`dart::simd`'s own `fmadd`/`fmsub` are not yet bit-identical across the
   backends CI already builds and tests.** The SSE4.2 backend's no-FMA
   fallback and the plain-AVX (non-AVX2) backend both compute `fmadd`/`fmsub`
   as unfused `mul` then `add`/`sub`
   (`dart/simd/detail/sse42/operations.hpp:206-232`,
   `dart/simd/detail/avx/operations.hpp:145-155`) — a genuinely different,
   two-rounding result from the scalar backend's `std::fma`-based
   implementation (`dart/simd/detail/scalar/operations.hpp:121-140`) and
   from the AVX2/AVX-512 backends' native hardware FMA instructions. Verified
   this fallback path is exactly what `ci_simd.yml`'s own `sse42` and `avx`
   matrix cells build (`-msse4.2` and `-mavx -mno-avx2`, neither defines
   `__FMA__`) — see the macro dump below. `ci_simd.yml` runs each backend's
   unit tests independently and never diffs SSE4.2's numeric output against
   scalar's, so this divergence is untested, not absent.
3. **Two cross-product formulas already coexist in the merged module with
   different fusion policies.** The AoS `Vector3<T>::cross()`
   (`dart/simd/geometry/vector3.hpp:322-331`) uses plain infix
   `a.y()*b.z() - a.z()*b.y()` (unfused); the SoA batch `cross()`
   (`dart/simd/geometry/batch.hpp:536-544`, already implemented, already
   zero consumers per the lane doc's own framing) uses
   `fmsub(a.y, b.z, a.z * b.y)` (fused). This is exactly the kind of
   drift the D1 contract needs to close before WP-PG.41 adds call sites that
   might mix the two, and is what the prototype in this packet exercises
   directly.

Investigating D2 established that, today, **`dart::simd` resolves to its
Scalar backend in every default and packaged DART build**: backend selection
in `dart/simd/config.hpp` is driven purely by ambient `__SSE4_2__` /
`__AVX__` / `__AVX2__` / `__FMA__` macros, i.e. whatever `-m` flags the
compiling TU happens to receive, and no CI workflow or `pixi.toml` build task
outside `ci_simd.yml` passes any such flag to the main `dart`/`dart-simd`
targets (checked via grep, see below). This means WP-PG.41's kernels will
ship as scalar code in the actual packaged product unless D2 changes that,
which sharpens the ISA-delivery decision considerably — see D2 below.

## D1 — FP-determinism contract

### Proposal

Default posture for state-affecting paths (anything that can change
`World::step` output, contact results, or final pose/velocity state):
**bit-identical results between the scalar reference and every enabled SIMD
backend**, achieved by:

- **No reassociation**: never let the compiler silently reorder or fuse
  floating-point operations differently between the scalar reference and a
  SIMD kernel computing the same quantity.
- **Consistent, explicit FMA usage**: use `dart::simd::fmadd`/`fmsub`/
  `fnmadd` (or `std::fma` directly in non-`dart::simd` code) for any
  operation where a SIMD backend uses a fused multiply-add, and use the
  *same* explicit fusion in the scalar reference that validates it. Do not
  rely on plain infix `a*b+c` as "the" scalar reference for anything that
  must match a fused SIMD kernel bit-for-bit — findings 2 and 3 above show
  this is not a hypothetical footgun but an already-present one.
- **Hash re-baselining only with maintainer sign-off**, following the
  existing `#3188-#3194` pattern already codified in the README's
  compatibility envelope (rule 1): any change to a guard scene's final-state
  hash, contact count, pair count, or resting count is a **behavior-changing
  PR** (envelope rule 2), never bundled with behavior-preserving work, and
  must record the old and new hash side by side with the rationale for the
  change. The external gz gate's `ChangedWorldPoses` patch is exact-equality
  (README:175, 05-simd-enablement-lane.md:27) — pose-path drift fails that
  gate outright, so this is not just an internal guard-scene convention.

### Compiler-flag implications (evidence)

Probed this host's toolchain directly (`g++ (Ubuntu 15.2.0-16ubuntu1) 15.2.0`,
`clang++ 22.1.8`) rather than assuming behavior from prior knowledge, since
this is a load-bearing claim:

```
$ cat > fma_probe.cpp <<'EOF'
double madd(double a, double b, double c) { return a * b + c; }
EOF

$ g++ -std=c++17 -O2 -mavx2 -mfma -S -o - fma_probe.cpp | grep -E "vfmadd|mulsd|addsd"
        vfmadd132sd     %xmm1, %xmm2, %xmm0

$ g++ -std=c++17 -O2 -S -o - fma_probe.cpp | grep -E "vfmadd|mulsd|addsd"
        mulsd   %xmm1, %xmm0
        addsd   %xmm2, %xmm0

$ g++ -std=c++17 -O2 -mavx2 -mfma -ffp-contract=off -S -o - fma_probe.cpp | grep -E "vfmadd|mulsd|addsd"
        vmulsd  %xmm1, %xmm0, %xmm0
        vaddsd  %xmm2, %xmm0, %xmm0

$ clang++ -std=c++17 -O2 -mavx2 -mfma -S -o - fma_probe.cpp | grep -E "vfmadd|mulsd|addsd"
        vfmadd213sd     %xmm2, %xmm1, %xmm0

$ g++ -mavx2 -mfma -dM -E -x c++ /dev/null | grep FP_FAST_FMA
#define __FP_FAST_FMA 1
```

Both compilers auto-contract `a*b+c` into a single FMA instruction whenever
the target ISA advertises FMA support, with **no** `-ffast-math` or explicit
`-ffp-contract=fast` — this is each compiler's default for C++. `
-ffp-contract=off` reliably disables it.

**Recommendation**: pin `-ffp-contract=off` on any target where the D1
bit-identical contract applies (at minimum `dart` and `dart-simd`; arguably
DART-wide, since Eigen's own vectorized paths are gated on the same
`__FMA__` macro and are equally subject to silent auto-contraction — an
Eigen-specific audit is out of scope for this packet, see Open unknowns).
This closes the *compiler-driven* divergence class independently of anything
`dart::simd` does explicitly. It also means the existing `DART_ENABLE_SIMD`
option's documented risk (`CMakeLists.txt:132-138`, alignment errors across
machines) is narrower than it could be — `-ffp-contract=off` would prevent
the *rounding* divergence that option can introduce, though the underlying
"different `-march` per build machine" risk it already documents is
unrelated to FMA and out of scope here.

### FMA policy across backends (evidence)

Backend-by-backend `fmadd`/`fmsub` implementation, from the merged code:

| Backend | Implementation | Rounding | Bit-identical to scalar? |
| --- | --- | --- | --- |
| Scalar | `std::fma(a, b, c)` (`detail/scalar/operations.hpp:121-140`) | Single (correctly-rounded per IEEE-754-2008, mandated by the C/C++ standard regardless of hardware support) | — (reference) |
| SSE4.2, `DART_SIMD_FMA` defined | `_mm_fmadd_pd`/`_mm_fmsub_pd` (`detail/sse42/operations.hpp:160-202`) | Single (hardware FMA3) | **Yes** (IEEE-754 FMA is backend-independent for identical operands) |
| SSE4.2, `DART_SIMD_FMA` undefined | `_mm_add_pd(_mm_mul_pd(a,b), c)` (`detail/sse42/operations.hpp:204-232`) | **Two** (separate mul, then add/sub) | **No** |
| AVX (no AVX2) | `_mm256_add_pd(_mm256_mul_pd(a,b), c)` (`detail/avx/operations.hpp:145-155`), unconditional | **Two** | **No** |
| AVX2 | `_mm256_fmadd_pd`/`_mm256_fmsub_pd` (`detail/avx2/operations.hpp:161-187`) | Single (hardware FMA3) | **Yes** |
| AVX-512 | `_mm512_fmadd_pd`-style (`detail/avx512/operations.hpp:164-185`), unconditional | Single | **Yes** (every real AVX-512F CPU has FMA architecturally) |
| NEON (AArch64) | native `vfma`-style, unconditional (`detail/neon/operations.hpp:162-183`) | Single | **Yes** (AArch64 NEON always includes FMA) |

`DART_SIMD_FMA` is set purely from `defined(__FMA__)`
(`dart/simd/config.hpp:97-99`), independent of which SIMD width backend gets
selected. Checked which of `ci_simd.yml`'s own matrix cells actually reach
the no-FMA fallback:

```
$ g++ -msse4.2 -dM -E -x c++ /dev/null | grep -E "__FMA__|__SSE4_2__"
#define __SSE4_2__ 1              # __FMA__ NOT defined

$ g++ -mavx -mno-avx2 -dM -E -x c++ /dev/null | grep -E "__FMA__|__AVX__"
#define __AVX__ 1                 # __FMA__ NOT defined
```

Both the `sse42` and `avx` cells of `ci_simd.yml`'s matrix
(`.github/workflows/ci_simd.yml:50-58`) build the unfused, two-rounding
fallback today. This is a real gap relative to the lane doc's own stated D1
proposal ("no reassociation, no FMA divergence between backends",
`05-simd-enablement-lane.md:22`), not a hypothetical one, and it predates
this packet — it is a property of already-merged, already-CI-passing code,
simply never checked cross-backend.

**Two ways to close this gap; the maintainer choice affects WP-PG.41's
implementation cost, so it is folded into this D1 ratification:**

- **(i) Make the fallback route through `std::fma` per-lane** instead of
  native unfused `mul`/`add`, matching the scalar backend exactly on any
  hardware lacking FMA3. Correct by construction, but loses SIMD-width
  parallelism for this one operation on FMA-less hardware (extract lanes,
  call libm `fma()`, repack) — a real perf regression specifically on
  pre-2013-ish x86-64 CPUs (SSE4.2-only, e.g. Nehalem–Ivy Bridge) and on the
  AVX-without-AVX2 tier (rare in practice; AVX-capable CPUs are AVX2-capable
  except some early Sandy Bridge/Ivy Bridge/Bulldozer-family parts).
- **(ii) Scope "bit-identical" per deployed build, not across all possible
  builds**: require that any single packaged/deployed DART binary uses
  exactly one backend end-to-end (already true today, since backend
  selection is compile-time), and require guard-scene hash baselines to be
  recorded *per backend* rather than assuming one universal hash. This is
  weaker than the lane doc's literal wording but matches how this project
  already treats baselines elsewhere (per-commit, per-detector hash tables
  in the README's evidence format) and avoids the perf cost of (i).

This document does not resolve (i) vs (ii) — that is squarely a maintainer
call, since it trades correctness-by-construction against a real
regression on genuinely-still-deployed old hardware. Recorded here so
WP-PG.41 does not have to rediscover the gap.

### Prototype: bit-equality mechanism demonstration

See [prototypes/wp_pg_40_cross3_batch.hpp](prototypes/wp_pg_40_cross3_batch.hpp)
and
[prototypes/wp_pg_40_cross3_batch_test.cpp](prototypes/wp_pg_40_cross3_batch_test.cpp).
This exercises finding 3 above directly: the already-merged SoA batch
`dart::simd::cross()` compared against two scalar references — a "naive"
one mirroring the existing AoS `Vector3<T>::cross()` formula (unfused), and
a "fused" one using explicit `std::fma` matching the SoA kernel's own
internal fusion policy. Both references and the comparator are placed in
this packet's own prototypes/ folder (see "Prototype placement" below for
why, not `dart/math`).

**Compile matrix (this packet's verification scope — see hard constraints
below)**:

```
g++ -std=c++17 -fsyntax-only -Wall -Wextra -Werror \
  -I <worktree-root> -I <eigen3-include> \
  wp_pg_40_cross3_batch_test.cpp
```

| Flags | Result |
| --- | --- |
| (a) default (no `-m` flags → Scalar backend) | PASS |
| (b) `-DDART_SIMD_FORCE_SCALAR` | PASS |
| (c) `-mavx2 -mfma` (AVX2 backend) | PASS |

All three compile clean under `-Wall -Wextra -Werror`. **Not executed** in
this packet (see "Hard constraints" below) — the bit-equality assertions
themselves (matches-fused-reference should be true on every backend;
matches-naive-reference is not expected to be) have not been run. Running
this harness is a fast, cheap first step for whoever picks up WP-PG.41 or a
follow-up, and would also be the natural place to add a direct check of the
SSE4.2/AVX fallback gap (finding 2) by comparing `dart::simd::fmadd` output
against `std::fma` under `-msse4.2` alone.

**Prototype placement rationale**: placed under
`docs/dev_tasks/dart6_performance_generalization/prototypes/`, not
`dart/math/detail/`. This is a design-packet sketch demonstrating a
mechanism, not a proposed production API — `dart::simd::cross()` for
`Vector3SoA` already exists and is already the kernel WP-PG.41 would
consume directly; this prototype does not duplicate or replace it. Picking
the real seam location, integrating with `ContactConstraint`/Jacobian
assembly call sites, and promoting the harness to an executed `UNIT_simd_*`
gtest case wired into `ci_simd.yml`'s matrix is WP-PG.41 scope, once D1/D2
are ratified and a profiled call site exists to attach it to
(WP-PG.10 seam selection).

## D2 — ISA delivery contract

### Current state (evidence)

- `dart::simd` backend selection (`dart/simd/config.hpp:41-104`) is
  compile-time, driven solely by ambient `__SSE4_2__`/`__AVX__`/`__AVX2__`/
  `__FMA__`/`__ARM_NEON__` macros — i.e. whatever `-m` flags the compiling
  TU receives.
- No CI workflow or `pixi.toml` task outside `ci_simd.yml` passes any such
  flag to the `dart`/`dart-simd` targets (checked via
  `grep -rn "march\|mavx\|mfma\|msse" .github/workflows/*.yml pixi.toml`,
  excluding `ci_simd.yml` itself — no matches). The `dart-simd` interface
  target's own `CMakeLists.txt` (`dart/simd/CMakeLists.txt:66-73`) adds no
  ISA flags either, just `Eigen3::Eigen` and `cxx_std_17`.
- **Consequence**: `dart::simd` resolves to its **Scalar** backend in every
  default DART build today, including whatever CI builds and (as far as this
  repo records) whatever gets packaged. The pre-existing `DART_ENABLE_SIMD`
  option (`CMakeLists.txt:139-141`, default `OFF`) is the only way a
  same-machine build gets SSE4.2/AVX/AVX2, and it does so by adding
  `-march=native` to the *entire* `dart` target — already documented as a
  cross-machine footgun (`CMakeLists.txt:132-138`).
- `ci_simd.yml`'s matrix validates `dart::simd`'s own unit tests per ISA
  level via a **separate** CMake build (`build/simd`,
  `.github/workflows/ci_simd.yml:78-105`) that only builds the
  `UNIT_simd_*` targets — it does not build the real `dart` shared library
  under those flags. So today, nothing in CI exercises SSE4.2/AVX/AVX2
  `dart::simd` code reachable from the actual `dart` library, only from
  dedicated test binaries.
- No conda recipe, Debian packaging, or vcpkg portfile exists in this repo
  (checked: no `meta.yaml`, `conda_build_config.yaml`, `.spec`, or `debian/`
  found). Packaging-baseline flags are set externally (conda-forge
  feedstock, gz-physics's own build recipes) — **unknown from this repo**,
  see Open unknowns.

This sharpens the decision: it is not "should packaged binaries stay at
baseline ISA," it is "packaged binaries **already are** baseline ISA
(effectively no-`dart::simd`-vectorization-at-all, i.e. Scalar), and
WP-PG.41's kernels will ship that way unless D2 changes something."

### Option (a) — Baseline-ISA-only (status quo extension)

WP-PG.41+ kernels compile under whatever flags the `dart` target already
gets — Scalar in the packaged/default case, SSE4.2/AVX/AVX2 only for a
locally-built, `DART_ENABLE_SIMD=ON`, single-machine consumer.

- Pros: zero packaging risk (no SIGILL possible — baseline ships baseline);
  zero implementation cost; matches envelope rule 6 exactly as written
  today; no dispatch machinery to build, test, or maintain.
- Cons: **the WS-D lane's promised wins deliver zero measured benefit in the
  actual shipped/packaged product** unless something else changes (a
  packaging-side floor bump, or option (c) below). This should be stated
  plainly to whoever ratifies D2: choosing (a) means WP-PG.41's contact-
  Jacobian work is validated and merged, but its performance benefit is
  real only for downstream integrators who separately opt into
  `DART_ENABLE_SIMD=ON` on their own build machine.

### Option (b) — Baseline floor bump to SSE4.2

An intermediate option not named in the lane doc's D2 framing, but implied
once the "packaged = Scalar" fact above is established: bump the *default*
compiled-in floor from implicit-SSE2-only to `-msse4.2` (not
`-march=native`) for the `dart`/`dart-simd` targets. SSE4.2 has been
universal on x86-64 since Nehalem (2008) and is exactly the `x86-64-v2`
microarchitecture level GCC/Clang have standardized since ~2020 — a much
narrower commitment than `-march=native`, with no cross-machine alignment
risk (it is a fixed, portable target, not "whatever this machine has").

- Pros: unlocks the SSE4.2 `dart::simd` backend for essentially all real
  consumers with a single, low-risk, one-line CMake change — no dispatch
  machinery at all.
- Cons: still a **policy decision that raises the minimum supported CPU**
  for packaged DART binaries — not free the way (a) is. Whether this is
  acceptable depends on DART's stated hardware-support policy for 6.20
  packaged binaries, which this repo does not record (see Open unknowns).
  Also does not reach AVX2/FMA — the win WP-PG.41's contact-Jacobian
  batching is actually chasing per the lane doc's framing — so may not be
  worth a policy change on its own merits.

### Option (c) — Runtime dispatch

Per-backend TUs + CPU feature detection, selecting the compiled variant at
first use. Sketch for the batch-helper seam WP-PG.41 will add (e.g. a
`batchCross3`/spatial-transform batch helper consumed from
contact-Jacobian/impulse assembly):

- **Per-ISA object files**: each batch helper gets one `.cpp` TU per
  supported ISA tier (`_scalar.cpp`, `_sse42.cpp`, `_avx2.cpp` at minimum —
  AVX-512/NEON/SVE need their own packaging-target discussion, out of scope
  here), each compiled as its own CMake `OBJECT` library target with
  `target_compile_options` scoped to that one target only (never the
  exported `dart` target — keeps envelope rule 6 intact). Each TU defines
  the same function under an ISA-suffixed symbol name (or in an anonymous
  namespace with a suffix, to avoid ODR collisions across TUs compiled with
  different flags for the same signature).
- **Dispatch mechanism — three shapes considered**:
  - *GNU IFUNC* (`__attribute__((ifunc(...)))`) or GCC/Clang's
    `target_clones`/multiversioning attributes: resolved once at load time
    by the dynamic linker, effectively zero per-call cost. **Not
    recommended**: ELF/glibc-specific, so unavailable on the Windows/macOS
    targets DART's conda-forge packaging also covers — a portability
    blocker for a cross-platform physics library, not just a style
    preference.
  - *Dispatch-on-first-call* (a single lazily-initialized function pointer,
    guarded by a function-local static or `std::call_once`): simplest to
    implement for one or two kernels, fully portable. Scales poorly if many
    kernels each need their own hand-written dispatch point — more
    boilerplate per kernel as WP-PG.41/42 add more helpers.
  - *Function-pointer table, populated once*: same portability as
    dispatch-on-first-call, but centralizes CPU-feature detection into one
    init routine that fills in a table of function pointers (one row per
    kernel), so adding a new batch helper means adding one table entry, not
    rewriting a detection routine. **Recommended** — the standard pattern
    used by portable SIMD-dispatching libraries (e.g. simdjson, zlib-ng)
    for exactly this reason, and it is the shape that scales with WS-D
    adding more kernels across WP-PG.41/42 without repeating detection
    logic per kernel.
- **CPU feature detection**: `__builtin_cpu_supports("avx2")` /
  `__builtin_cpu_supports("sse4.2")` on GCC/Clang (available since GCC 4.8 /
  Clang 3.7 — comfortably within DART 6.20's toolchain floor of GCC
  10.2.1+/11.2.0+ and Clang 6.0+, `CMakeLists.txt:391-398,448`); MSVC has no
  equivalent and needs a small `__cpuid`/`__cpuidex`-based probe instead — a
  well-trodden, bounded amount of extra code, not a research question.
- **Packaging constraints preserved**: the exported `dart` target's public
  headers and default compile options never gain a `-march` flag (envelope
  rule 6 stays exactly as written); only the internal per-ISA `OBJECT`
  libraries get ISA-specific flags, and they are never installed as public
  headers (this is squarely the "cpp-only where possible" pattern the
  envelope already prefers, rule 4).
- **Cost**: real, bounded, but non-trivial — N object files and an
  `OBJECT` library per ISA tier per kernel, a feature-detection routine, a
  dispatch table, and CI coverage for the dispatch path itself (does the
  table actually pick AVX2 on an AVX2 host? does it fall back correctly
  when `DART_SIMD_FORCE_SCALAR`-equivalent forcing is requested?). Justified
  only if WP-PG.41's own before/after `bm_simd` evidence shows an AVX2 win
  on the actual contact-Jacobian seam worth this maintenance burden — which
  does not exist yet (WP-PG.41 has not landed).

### Recommendation

**Option (a) now** — ship WP-PG.41's kernels at whatever the `dart` target
already builds at (Scalar in the packaged case), explicitly documenting that
this means zero default-on performance benefit until either (b) or (c) is
separately decided. Do not build the dispatch layer speculatively; the sketch
above is recorded so a follow-up packet does not have to redesign it. This
matches the lane doc's own gating language ("a follow-up packet if evidence
shows AVX2 gains worth shipping," `05-simd-enablement-lane.md:24-25`) and
avoids taking on the packaging/dispatch complexity before there is a single
measured before/after number to justify it. **Revisit trigger**: once
WP-PG.41 lands and its `bm_simd` before/after evidence exists, re-decide
between (b) and (c) using that evidence — (b) if the win is modest and
SSE4.2-reachable, (c) if it specifically requires AVX2/FMA and is large
enough to carry the dispatch maintenance cost.

## Open unknowns (not answerable from this repo)

These are named explicitly per this project's evidence-first discipline —
consequential unknowns are either resolved or recorded, never silently
assumed:

1. **Conda-forge/vcpkg packaging baseline flags for DART.** This repo has no
   `meta.yaml`/`conda_build_config.yaml`/portfile — packaging lives in an
   external feedstock repo. Probe: check the `dartsim`/`dart` conda-forge
   feedstock's `build.sh`/`bld.bat` and `conda_build_config.yaml` for any
   `-march`, `CMAKE_CXX_FLAGS`, or `DART_ENABLE_SIMD` override, and check any
   vcpkg portfile similarly.
2. **Whether gz-physics/gz-sim build DART with FMA-enabling flags anywhere.**
   PARTIALLY RESOLVED (orchestrator probe, 2026-07-04): the pinned local
   checkouts used by the gz gate (`gz-physics8_8.0.0` and the matching
   gz-sim tag under `.deps/`) contain **no** `-march`/`-mavx`/`-mfma`/
   `DART_ENABLE_SIMD` in any CMake, script, Dockerfile, or workflow file —
   the gz stack consumes DART at baseline ISA with compiler-default
   contraction. Residual unknown: gz's own external CI Docker images and
   conda recipes (outside these pinned checkouts). Probe for the residual:
   grep the gazebosim CI image definitions and conda-forge gz recipes for
   `-m` flags or `DART_ENABLE_SIMD=ON`.
3. **Full audit of Eigen's own FMA usage under the same auto-contraction
   risk.** Established that no CI workflow outside `ci_simd.yml` passes
   ISA flags to the main `dart` target, so Eigen's own vectorization
   defaults to no-FMA today (mirroring `dart::simd`) — but a full audit of
   whether Eigen's `pmadd`/expression-template paths would themselves
   silently start auto-contracting under a future `-ffp-contract`-permissive
   AVX2 build is out of scope for this packet. Probe: build a trivial Eigen
   matrix-multiply TU under `-mavx2 -mfma` with and without
   `-ffp-contract=off` and diff the assembly, the same way this document
   probed `dart::simd`'s own primitives.
4. **DART's stated minimum-CPU support policy for packaged 6.20 binaries**,
   needed to evaluate option (b) (SSE4.2 floor bump) on its merits. Not
   recorded anywhere found in this repo's docs. Probe: ask the maintainer
   directly, or check historical issue/PR discussion around `DART_ENABLE_SIMD`
   or any prior minimum-CPU decisions.
5. **Whether the SSE4.2/AVX no-FMA `fmadd`/`fmsub` fallback gap (D1 finding
   2) should be fixed via (i) routing through `std::fma` per-lane, or (ii)
   scoping "bit-identical" to be per-deployed-backend rather than universal.**
   Not resolved in this document — see the D1 section above; recorded
   explicitly as a sub-decision folded into D1's ratification rather than
   left implicit.

## Hard constraints observed in this packet

Per the WP-PG.40 task scope: no `cmake` builds, no `pixi` environment
creation, no test executions were run. All verification above is either (a)
direct reading of already-merged source (file:line citations throughout), or
(b) single-TU `g++`/`clang++` invocations against the system toolchain
(`-fsyntax-only` for the prototype's three-flag matrix; `-S`/`-dM -E` probes
for the compiler-contraction and macro-gating evidence). No benchmark,
`ctest`, or `pixi run` command was executed. The full
`DART_SIMD_FORCE_SCALAR` matrix validation via `ci_simd.yml`'s actual CMake
build is explicitly deferred, as scoped by the orchestrator, to a follow-up
step outside this packet.
