# Literal 101-Stone Standing Protocol v1

## Purpose and claim boundary

This protocol freezes the first authoritative full-duration test of DART's
source-derived, exact-inertia, literal 101-stone wedge reconstruction. It asks
whether the current Native exact-FBF colored path preserves that reconstructed
arch for 600 steps (10 simulation seconds at 60 Hz).

The scene is not the unavailable author scene. A passing result would be local
DART evidence for a separately labeled reconstruction; it would not establish
Figure 8, Table 6/7, Kamino, GPU, author-scene, or paper-timing parity. A failed
gate is retained as scientific-negative evidence and must not be tuned into a
passing v1.

## Pre-freeze diagnostic disclosure

Before this protocol was written, a one-step, non-evidence probe reused the
already selected 25-stone literal configuration. It exposed a probe-only
configuration bug: colored BGS was enabled before `World::setConstraintSolver`,
which copied state from the previous boxed solver and cleared the sidecar flag.
After moving the opt-in setter to the installed solver, the same one-step smoke
reported 400 contacts, 100 adjacent body pairs, three colors, maximum color
width 34, actual colored execution, one accepted 5,000-outer cap, and residual
approximately `0.7815`. No scene or solver parameter was changed in response.
The smoke is not evidence; it makes the expected negative risk explicit.

## Frozen scene contract

- scenario name: `masonry_arch_101_literal_wedge`
- 101 source-derived eight-vertex convex wedges
- exact uniform-prism mass properties at density `1000 kg/m^3`
- friction coefficient `mu=0.8`
- source barrier offsets omitted
- adjacent end faces expanded by `1 um`
- all stones shifted downward by `1.001 mm`
- Native collision with `FourPointPlanar` manifolds
- springers 0 and 100 pinned; all 99 interior stones dynamic
- horizontal plane; gravity `(0, 0, -9.81) m/s^2`
- no projectile, settling pre-roll, deactivation, fallback, or hidden rescue
- time step `1/60 s`; exactly 600 requested steps

The expected initial graph, frozen after the disclosed mechanics smoke, is 400
contacts on 100 adjacent stone pairs, represented by 100 manifolds in three
deterministic colors with maximum width 34. Any collision-cap truncation or
non-adjacent graph is a contract failure.

## Frozen solver and execution contract

- solver: `exact_fbf`
- trace scope: `performance`
- solver contract: `dart_best_colored_bgs`
- maximum outer iterations: 5,000
- tolerance: `1e-6`
- exact H-metric local cone solve
- 30 fixed inner sweeps and one exact local iteration
- adaptive outer step size, scale 35, relaxation 1.1
- previous-step impulse warm start enabled; adaptive gamma reset each frame
- diagonal seed, matrix-free seed, dense snapshot/polish, projected-gradient
  retry, and boxed-LCP fallback disabled
- split impulse enabled and velocity Baumgarte ERP zero
- four simulation threads on logical CPUs `8,10,12,14`, one logical CPU per
  physical P-core
- participant affinity and residency evidence enabled
- float64 DART on the recorded local x86-64 Linux host

## Frozen v1 acceptance gates

The authoritative runner must independently validate all fields and fail
closed. v1 passes only if every gate below passes without parameter changes:

1. the process returns zero and emits exactly 600 ordered rows for steps
   1 through 600;
2. all numeric fields and body states are finite;
3. every step has 400 contacts, 100 colliding body pairs/manifolds, three
   colors, and maximum color width 34;
4. every step performs exactly one exact attempt and one exact solve;
5. every residual is `<=1e-6` and every status is `success`;
6. accepted caps, exact failures, and boxed-LCP fallbacks are all zero;
7. colored BGS is requested and used, with one persistent dispatch per exact
   attempt and participants resident on the declared P-core set;
8. maximum displacement of any of the 101 stones from constructed time zero is
   `<=0.05 m` at every step;
9. minimum orientation alignment of every stone from constructed time zero is
   `>=0.80` at every step; and
10. the normalized scene/work fingerprint is constant across all rows and is
    pinned in the finalized metadata.

If a solver gate fails, the runner may terminate the authoritative attempt at
the first invalid step. Such a prefix is a valid scientific-negative artifact
only when the runner preserves the raw output, stderr, command, source/binary
hashes, environment/affinity record, independently recomputed gates, and an
explicit `artifact_valid=false` / `standing_claim_passed=false` verdict. The
failed prefix must not be used for timing or physical-outcome claims.

## Required durable outputs

- immutable raw trace and stderr
- invocation, affinity, build, source, binary, and runner provenance
- independently recomputed summary and gate report
- metadata with SHA-256 bindings for every artifact
- a report that states either the exact local reconstruction result or the
  precise first failed gate

Media is downstream. A current-source still/video bundle may be created only
after the numeric v1 gates pass. No media is promoted from a failed prefix.

## No-retuning rule

Do not change the v1 geometry, closure, downward shift, solver budget, inner
sweeps, relaxation, warm-start/gamma policy, collision graph, thread placement,
or acceptance thresholds after observing the authoritative result. Any later
experiment must use a new protocol version and state the hypothesis selected
before execution.

## Frozen v1 result

The preregistration wording above records the source-availability state at the
time the contract was frozen. A public author masonry-arch reference is now
pinned, but this DART reconstruction is still not that source scene and the
frozen numerical protocol is unchanged.

The provenance-complete current-source authoritative bundle is
`assets/paper_evidence/fig08_arch101_literal_v1_negative_final_v6/`.

Classification: `failed_prefix_scientific_negative`. Artifact valid: `false`.
Standing claim passed: `false`. Timing evidence eligible: `false`.

The frozen run stopped after step 1 as required. It observed the expected 400
contacts, 100 body pairs/manifolds, three colors, maximum color width 34, four
colored participants resident on logical CPUs `8,10,12,14`, and one persistent
dispatch. The exact solve consumed all 5,000 outer iterations, returned
`fbf_failed`, recorded one exact failure and zero boxed-LCP fallbacks, and had
residual `0.78153646143524735`. State remained finite at the failed prefix;
maximum all-stone displacement was `0.0027250000000000052 m` and minimum
orientation alignment was `1.0`. The physical prefix is not an admissible
trajectory because the exact solve failed.

The normalized scene/work fingerprint is
`8d275edcaa82a2f628fdd5d9f846b2daed7e6864fdc4b7e80eca9a8c93bcf527`.
The frozen protocol-contract SHA-256 remains
`031140a359fcb1c59f8b7377ce95e31720ec00368f175b70c7daed5fe761ffc3`.

Current source and artifact SHA-256 values are:

- runner: `e3e3b7c998e038f6bf1499b1b4a8eea04261dc0aa7a6e4cf668486482c56a6f5`
- runner tests: `bc66de20160b91b6db5e2ee56ebde1b60a1d32de2fed4bc1d6f22ba80d10c1fc`
- trace source: `b00eea0c87f75f17259fac433bafd98c20a961d14389a80e91742d3c8a678f76`
- trace binary: `0923bf7df1eaa518f9a2ffacd0a42fd7330f5cbf6c35cb5749fbc122db1310ff`
- collision-probe source: `a31e3472c3ec4a914bcd1f2c62a32471993154eb00fdabcc230b33d08cbf9c18`
- collision-probe binary: `bf61ff3b430d75c8a988eda614e82914b00bc4cd460d2f0a3c0da137fc633e54`
- `raw.csv`: `03fba1d83f9209b0ade3698bdada42ca3927c176397a5ddee8ce0ca175e82947`
- `stderr.txt`: `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855`
- `collision_probe_stdout.txt`: `de2a0184e24cebf3bcbde181108ccc39e73d1263f428d3cd2179011fde87c073`
- `collision_probe_stderr.txt`: `526d79acfb872e6b0eb141f6447a99ef376cb9dcbd1addcaeecbd2dafeeca39b`
- `invocation.json`: `66dd6f2ceef2b50d5b0960c86fc291d27f6353d228c59161c22f18dd94c12522`
- `summary.json`: `2e03653745a1e8d94c7b18d446004d16bfd4e176d7b9315aa198e3c08538bbfa`
- `metadata.json`: `5e0b166171e08349847664b504b0a9b15cc1f582514388b532259ad6edad2bec`
- `REPORT.md`: `52e5592ba5ad32a919f221e1a8e98ecb4b4f82c0228e16de44318b436bf58958`

Runtime provenance binds both executables, normalized `ldd` resolution maps,
all resolved regular shared-library files, `/usr/bin/ldd`, and
`/usr/bin/taskset`, and rechecks the identities after the collision probe and
after the trace. The trace and collision-probe normalized `ldd` map hashes are
`8706fb6af371df72f31279ef6ced098d3826c0cc46409ed5f4e40e5fa32846b1`
and `c2e87ab93cc406bcab70c59938d82b915c6e55806c28d507a002bd40be843717`,
respectively. Their resolved build `libdart.so.6.19.3` hash is
`8fae2320858e49fdda309d89df8cb1158c1cc5dc11d345e14f5adca0ff63cf3d`;
the `ldd` and `taskset` hashes are
`5af792061ad344564c8305df2914d9b0f007ed1c108df9bab0068c598f4aec15`
and `3ac0b42cbe242b78c63b3ecef3023f3bcc17432b9d4731ac874848df6a87c914`.
The trace resolves 14 regular shared libraries and the probe resolves 17. This
file-identity record is not a claim over all host runtime state.

The graph evidence has two deliberately separate scopes:

- The Compact collision-only probe audits the constructed time-zero scene. In
  two stable repetitions it finds 102 contacts/unique pairs: exactly 100
  adjacent-stone pairs plus two springer-ground pairs, with zero non-adjacent,
  unexpected-ground, or non-finite contacts.
- The FourPointPlanar dynamics trace begins at step 1 and reports aggregate
  fields: 400 contacts, 100 constraint pairs/manifolds, three colors, and
  maximum color width 34. Its 95-column schema does not expose dynamic pair
  identities.

The first scope proves the intended constructed initial graph only; it does
not prove the identities of the dynamic FourPointPlanar pairs. Any positive
long-run promotion would still require dynamic pair-identity evidence. This
run already fails its standing claim at step 1.

Older generations remain explicitly noncurrent historical diagnostics.
The superseded v2 directory,
`fig08_arch101_literal_v1_negative_final_v2/`, was replaced by v3 after a
formatting-only change altered the collision-probe source identity; its
recorded result was not retroactively relabeled. V3 was then replaced by v4
after additive card-manifold trace instrumentation changed the trace source and
executable identities without changing this explicit 101-stone command or
scientific result. V4 was then replaced by v5 when the current trace and
Native-collision identities advanced. V5 was rebaselined as v6 after the
resolved current-build libdart identity advanced. Neither rebaseline changed
the frozen command or scientific result. The first invalid directory,
`fig08_arch101_literal_v1_negative/`, was rejected because the runner source
changed during execution. The second,
`fig08_arch101_literal_v1_negative_final/`, bound neither the dynamically
resolved DART/shared-library build nor an independent collision-graph probe.
Only v6 is current outcome evidence; the older bytes are retained only as
superseded diagnostic history.
