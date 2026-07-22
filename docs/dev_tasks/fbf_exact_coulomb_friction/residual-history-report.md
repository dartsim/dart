# FBF Residual-History Report

## Current Status

Residual-history infrastructure exists, but paper-matched Figure 9 evidence is
partial. The prior-source 600-step strict full-card attempt remains the
authoritative paper-profile contact-rich result and fails closed at step 89. A
separate current-source Figure 6 adapter now fails its strict exact lane at
completed step 35 on a 56-contact island. Additive `last_failure` diagnostics
identify outer dual/complementarity convergence as the terminal condition, but
the tested option matrix supplies no strict correction. A one-factor
c95-bound probe exercises the colored ordering/path with one participant
and zero parallel dispatches but changes the failed residual by only
`2.19e-14` relative. Reject it only as the next Figure 6 blocker discriminator;
this is not a multicore or general colored-BGS result. A subsequent isolated
c95-bound global-scope probe also fails at completed step 35, with the failed
68-contact aggregate's native-partition sub-audit localizing nonconvergence to
the 56-contact island and exactly zero off-block operator coupling. Reject
solve scope only as this blocker
discriminator; it changes the within-tolerance trajectory and is not a general
equivalence result. A c95-bound source-sized-gap diagnostic also fails the
strict 36-step gate, now after completed step 31 on a 31-contact group at
residual `1.0006073317077885e-5`; its ancestor-bound comparison stream differs
from step 1. Reject only that gap
representation as the strict-prefix fix; it is not a general gap, trajectory,
performance, superiority, or source-equivalence result. A c95 cadence-5
candidate likewise fails at step 35; its ancestor-bound stock comparator makes
all deltas contextual, so the valid result is only candidate-local. A
same-binary `last_norm10` terminal-estimate candidate also fails the step-35
gate; its recorded state summaries diverge before failure and exclude `W` and
product vectors, so the valid result is only that it does not clear the strict
prefix. A historical
debugger-mutated accept-cap preview reached all 2,400 steps with 1,106 capped
solves. A separate reproducible source-continuation lane now completes exact
and boxed with explicit plateau/cap/shrink telemetry; it remains
continuation-policy evidence rather than strict convergence or a Figure 9
residual-history result. The current-source, one-factor card-manifold comparison also
completes 600 emitted rows in both modes, but neither trajectory is strict and
its physical, timing, and paper verdicts remain null. A separate locally sealed bundle
establishes a positive 600-step exact result for a non-paper literal-wedge
arch; none of these current bundles provides the paper's Figure 9 per-outer
residual history.
The separate pinned-author incline packet preserves the current Figure 1
seven-cell FBF/MuJoCo/Kamino numeric sweep, including a configured FBF
convergence negative; it is not a Figure 9 curve or timing evidence.
The enclosing task remains active and incomplete at 24 partial, 5 blocked, and
0 complete across 29 requirements; six local visual bundles are finalized,
and the visual workflow declares 28 schedules.

## Prior-Source Strict Full-Card Evidence

The byte-identical ignored local cache is
`assets/dart_cpu_evidence/2026-07-12_prior_source_paper_cpu_card600_negative/`;
its metadata retains the original
`/tmp/fbf_cpu_paper_postreview_20260712_card600` output path. It requests:

- `card_house_26_settle_projectile_full` for 600 steps;
- Native collision and exact FBF;
- exact H-metric local cone solves;
- one simulation thread pinned to one physical core;
- tolerance `1e-6`, a 200-outer cap, and paper inner sweeps;
- automatic safe gamma and scenario-specific split impulse; and
- no diagnostic retry, dense polish, seed shortcut, or boxed-LCP fallback.

The process stops at step 89/600:

| Field | Observed value | Verdict |
| --- | ---: | --- |
| Successful steps | 0 | Prefix only |
| Accepted 200-iteration caps | 88 | Above tolerance; not convergence |
| `fbf_failed` steps | 1 | Terminal failure |
| Terminal failed residual | `1.8612e-2` | Fails `1e-6` |
| Maximum residual in attempted prefix | `30.02797095` | Fails `1e-6` |
| Residual pass fraction | `0.0` | Fails |
| Exact failures | 1 | Fails strict contract |
| Boxed-LCP fallbacks | 0 | No mixed-solver rescue |
| Contact range | 90-155 | Reconstructed DART trajectory |
| Raw mean step time | `59.8234 ms` | Diagnostic only |
| Raw p95 step time | `74.3458 ms` | Diagnostic only |
| Raw maximum step time | `79.5385 ms` | Diagnostic only |
| Real-time factor | `0.2786` | Below real time |

The trajectory is incomplete and its solver contract fails. Consequently it
has no valid full-duration physical outcome, real-time verdict, paper ratio, or
paper timing verdict. Zero boxed fallbacks is necessary but not sufficient.

An independent row-level diagnosis narrows the failure without overstating its
subtype. Primal feasibility never dominates and peaks at only `1.70e-16`;
dual feasibility dominates 75 rows and complementarity 14. The smallest
recorded residual is still `2.47e-3`. Accepted gamma equals the safe bound on
every row, shrink iterations are always zero, and coupling variation peaks at
only `0.017936`, so the retained evidence does not support adaptive-step
rejection as the cause. Instead, all 88 advanced rows consume the complete
200-outer budget, while contact count changes on 34 of 88 transitions, body-pair
count changes on 31, mean warm-start match is `0.7452`, and persistent gamma is
never reused. This supports slow global dual/complementarity convergence under
a redundant, churning reconstructed contact graph.

The step-89 row records one wrapper-level `fbf_failed` after 183 iterations.
That excludes an ordinary 200-iteration cap. An inner frozen-cone failure is
the leading code-path inference, but the trace does not serialize the failed
internal FBF enum, so `InnerSolverFailed` is not a confirmed artifact fact.
This statement applies to the retained prior-source artifact; the current
opt-in sensitivity schema serializes last-group internal status and
best-iterate diagnostics, while the current demo sidecar separately retains a
typed failed-group record across later successful groups.

With fallback disabled, the failed group is left unsolved and the evidence
caller stops. Later states are not generated from an invalid impulse.

## Current-Source Figure 6 Strict Failure And Ablations

The source-selected `fbf_author_card_house_4_impact_current_source` adapter
uses 26 cards, four initially kinematic cubes, 2,400 substeps at `dt=1/240 s`,
and a runner action that releases the cubes after completed substep 1,600. Its
checked-in exact contract remains fail closed: outer cap 200, tolerance
`1e-6`, ten fixed inner sweeps, no accepted outer cap, no regularization, and
no boxed fallback. The focused full record is
[`FIGURE6_CONVERGENCE_BLOCKER.md`](FIGURE6_CONVERGENCE_BLOCKER.md).

A fresh 100-step request again stops at completed step 35. Collision contacts
rise from 44 at step 34 to 68 at step 35. The failed 56-contact group is attempt
101; attempts 102 and 103 subsequently solve groups with 8 and 4 contacts.
Those later successes leave the ordinary last-group fields at `success` and
residual zero, but cumulative accounting correctly records 103 attempts, 102
solves, one exact failure, zero accepted caps, and zero boxed fallbacks.

The additive `solver_diagnostics.last_failure` object now retains the failed
group instead of allowing the later successes to mask its subtype:

| Field | Retained value |
| --- | ---: |
| Adapter / build / FBF status | `fbf_failed` / `success` / `max_iterations` |
| Failed-group contacts | 56 |
| Iterations / best iteration | 200 / 200 |
| Final and best residual | `4.1039190451256334e-4` |
| Primal feasibility | `0` |
| Dual feasibility | `4.1039190451256334e-4` |
| Complementarity | `2.4220067503580449e-4` |
| Worst dual / complementarity contact | 11 / 11 |
| Final / safe gamma | `2.7728679142517763` / `0.27728679142517765` |
| Coupling-variation ratio / shrink count | `0.0053281581128910033` / 0 |

The fresh local sidecar is
`/tmp/fbf_author_card_house_4_exact100_last_failure_current_source_20260721/timeline.json`,
SHA-256
`2d04d31134426ac2c4fc87b1774d5285b77740acaeb3ec3a005557b85944bb9d`.
It is reproducible diagnostic output, not a repository artifact. The new
object changes observability only; the cumulative failure remains the strict
gate and no failed state is promoted to convergence.

A bounded process-local option matrix found no existing strict option that
completed 100 steps:

| Strict change | Completed step | Result / worst residual |
| --- | ---: | --- |
| Frozen `max_outer=200` | 35 | failed, `4.103919e-4` |
| `max_outer=400` | 35 | failed, `3.922832e-4` |
| `max_outer=1,000` | 35 | failed, `1.033468e-4` |
| `max_outer=5,000` | 35 | failed, `9.234912e-6` |
| `max_outer=10,000` | 35 | failed, `9.234378e-6` |
| `max_outer=20,000` | 37 | failed, `1.928336e-6` |
| `max_outer=50,000` | 48 | failed, `2.242023e-6` |
| DART constraint regularization | 29 | eight failures, `5.811062e-6` |

Warm start off, persistence off, the fuller author-inspired cross-step policy,
30 inner sweeps, 20 shrink attempts, projected-gradient retry or local solves,
dense polish or snapshot, matrix-free and seed variants, step scales 1 and 5,
and outer relaxation 0.8 all failed at or before step 35. Increasing only the
outer budget moves the first failure but does not supply a strict solution.
This localizes the observed blocker to slow outer dual/complementarity
convergence on a contact-rich island; it does not prove one underlying causal
defect.

An isolated c95-bound one-factor diagnostic then merged all native constrained
groups only at the real solve boundary. Both native and global 36-step runs
fail closed at completed step 35. Native solves 56/8/4 contacts and reproduces
the retained `4.0844850280896461e-4` failed residual. The global 68-contact
solve ends at `4.0848243204467147e-4`; its reaction re-evaluates at
`4.0848243204472058e-4` on the 56-contact native island, while the 8- and
4-contact islands pass at `6.1744758287200254e-11` and zero. The independently
assembled global `W` has exactly zero off-block norm under the native partition
for every generation.

The two modes share identical contact fingerprints through generation 28.
Both accept generation 28 below `1e-6`, but their admissible reactions differ,
and fingerprints first diverge at generation 29. Global scope passes through
generation 33, then its 68-contact aggregate fails at generation 34; the
sub-audit identifies the same dominant native island. This rejects native-
island scope alone as the step-35 cause; it does not prove global/per-island
trajectory equivalence, source equivalence, outcome, performance, superiority,
Figure 6, or paper parity. The sealed isolated report and manifest are
`/tmp/fbf_fig06_global_scope_c95.TSfONI/RESULTS.md` and
`SHA256SUMS`, with SHA-256 values
`633828adbe08577b6d0973ca817194530ed8a08cbe27e85d2bcb004689919fe9` and
`90d72452c6b3ed09e0bc1e408b56e70092557784fd2089e6895d7a31a0c809d3`.

The subsequent isolated c95-bound source-gap diagnostic changes only the
existing four-level scenario flag: Native predictive negative-depth closure is
enabled with a `0.1 m` ground gap and `0.005 m` on all 30 dynamic shapes. The
strict source-inner serial solver contract is otherwise unchanged. Fail-fast
stops after completed step 31 with 52 aggregate contacts; the failed 31-contact
group reaches 200 iterations, residual/dual
`1.0006073317077885e-5`, primal `1.8622984303104533e-10`, and
complementarity `5.456227252287576e-8`. Aggregate counts are 186 attempts, 185
solves, one failure, zero cap accepts, and zero fallbacks.

The contact stream differs from stock from step 1, while the comparison stock
sidecar embeds ancestor `844c9c316195897cf2bf51f38eafc8ec9dcf959a` rather
than a fresh c95 binary. This rejects only the hypothesis that this DART gap
representation makes the first 36 strict steps complete at `1e-6`. It does not
establish general gap harm or benefit, source contact-model equivalence,
trajectory, outcome, backend, float32, timing, performance, superiority,
Figure 6, video, or paper parity. Keep the checked-in scene unchanged. The
verified package is `/tmp/fbf_fig06_gap_c95.m6bsif/`; `RESULTS.md` and
`SHA256SUMS` hash to
`3b0948c80871d19cbe29495a8abc57ac4f3e92dc518a9ae6551238a9aad9b17a` and
`11888f98a24175f50c09ce95509d754d0bbc1963e5d2294ad982ece280292119`.

The cadence diagnostic changes only the c95 internal exact-FBF residual-check
default from `1` to source cadence `5`. Its one strict run fails after completed
step 35 on a 56-contact group at 200 iterations and residual
`4.0845024466967225e-4`: 103 attempts, 102 solves, one failure, 3,450 total
outer iterations, zero accepted caps, and zero fallback. All nonzero per-step
successful-iteration sums are divisible by five.

The copied stock timeline is bound to ancestor `844c9c3`, not a fresh c95
binary, and the sidecar source hashes do not cover the patched math header.
Stock/candidate residual and iteration differences are therefore context only;
this is not a same-revision, same-binary, controlled A/B or causal cadence
estimate. Two cadence-specific tests pass, while the full math binary exposes
two expected legacy-default failures, so the global-default diagnostic patch
must not ship. The candidate establishes no trajectory, outcome, backend,
float32, performance, superiority, video, Figure 6, or paper-parity result.
Verified package: `/tmp/fbf_fig06_residual_cadence_c95.0QXC5c/`; `RESULTS.md`
`1f57c569f7feacb2c681cb17a70743782f07822abcbc1eb13d7822d81e9df18f`;
`SHA256SUMS`
`69db5e8915fadc31aae34d94c5f484928841b286566e172eea9535ee262d7645`.

The terminal spectral-estimate diagnostic uses the same c95 instrumented
Release binary for stock `rayleigh11` and candidate `last_norm10`. The exact
control reproduces completed step 35 / attempt 101 / 56 contacts, residual
`4.0844850280896461e-4`, 103 attempts, 102 solves, and one failure. The single
recorded variant satisfies all 103 ten-product/no-Rayleigh trace invariants but
also fails there with residual `4.07679549813362e-4`.

Residuals first diverge at attempt 57 / step 29. Recorded contact-frame and
reduced-state hashes plus product-norm sequences first diverge at attempt 67 /
step 30. The reduced-state hash covers only contact count, `freeVelocity`, and
coefficients; `W`, an operator digest, the initial reaction, the complete
reduced problem, and product vectors are unrecorded. Failure-state
residual/gamma deltas are contextual, not a same-problem local
causal estimate. The controlled result rejects only `last_norm10` as the
strict-prefix fix. It supplies no source-estimator, longer-trajectory, outcome,
backend, float32, performance, superiority, video, Figure 6, Figure 9, or
paper-parity result. The marker/timeline/trace triplets are internally
consistent with the preregistered protocol but cannot externally prove no
discarded run occurred. Verified package:
`/tmp/fbf_fig06_spectral_terminal_c95.OjNIB4/evidence/`; `RESULTS.md`
`e33894ab0b771544209d48724641716c491b04073ec5bec533c07df653e54cda`;
`comparison.json`
`8b7af123ccaa42fd9c6bbeb0916c5b691ed3234c428ae62e404e6f26449227f6`;
`SHA256SUMS`
`f18efba2ffb1f7f8ee0f88798c9bcd38103b571210949de5c0cc625fed3fd553`.

The pinned source recovers and advances a finite iterate even when its
configured convergence flag is false. One process-local GDB preview tested only
that advancement semantic by setting `accept_outer_max_iterations=true` and
disabling fail-fast. It completed 2,400 steps and executed the release action at
step 1,600, but accepted 1,106 of 3,231 exact attempts at the cap (379 before
release and 727 after), accumulated 284,435 outer iterations, and reached worst
residual `0.61608914241359314`. Its final group happened to succeed in 26
iterations at residual `7.9979707788026939e-7`; that last-group value does not
erase the cumulative caps or worst residual.

The preview timeline is
`/tmp/fbf_author_card_house_4_exact_accept_caps_2400_20260721/timeline.json`,
SHA-256
`034fa50433620a0050839fa26408700a86ad486e2e022edd3dbd29d19a0ad2ec`.
Its sidecar contract records the changed option, but its `runtime_command`
cannot reproduce the debugger mutation. The accompanying 64x64 image is not
legible enough for an outcome verdict. This is an unsealed finite-runtime
continuation diagnostic, not strict trajectory, physical-outcome, media,
solver-equivalence, superiority, Figure 6, or paper-parity evidence.

### Reproducible Source-Continuation Full Run

The separate
`fbf_author_card_house_4_impact_source_continuation_current_source` lane
completes all 2,400 steps with the same scene, clock, and successful step-1,600
release as its paired boxed control. Exact records 3,351 attempts and solves,
zero exact failures or boxed fallbacks, 2,605 ordinary successes, 113 plateau
accepts, 633 max-iteration accepts, and zero line-search shrink caps. The 746
accepts are 22.262% of 3,351 solves and occur across 723 steps. The cumulative
worst final residual is `0.91712002943322535`, first reached at step 2,101.

These are per-group/per-step continuation diagnostics, not the paper's
per-outer Figure 9 curves. The accepted outcomes remain typed rather than being
relabeled as `success`, so the run cannot promote the strict step-35 negative.
Independent exact/boxed inspection is useful for the separate visual claim:
the viewports are identical only at step 0, differ by 0.165% at step 1,600 and
11.985% at the endpoint, where exact retains more upright structure. It does
not make this a quantitative trajectory, outcome, backend, timing, mechanism,
paper-lane mapping, or superiority result.

The ignored durable root is
`assets/paper_evidence/fig06_card_house_source_continuation_current_v1/`,
resealed from `/tmp/fbf_fig6_source_continuation_pair_20260721T1414_v2/`.
The summary, exact timeline, boxed timeline, and paired clip SHA-256 values are
`6888f4729c99d41753c9c8ec9a1ec2ec9e2367c71da76aab973f8f8c5e8674cc`,
`a9eb12711419b7801037d17059560559893be2898e07d14425a5f572175482ff`,
`1618e284f97ff7ed49e3288636269f5bea6131faa3bae45428e42e23de660bd8`,
and `282aebfb9e2e38fe3741db28e2ce909fb548d7aa46d048302a3b0e0bea9e1786`.
The artifacts remain outside Git and the clip has no PR attachment URL.

## Current-Source Card Manifold Sensitivity v2

The preregistered one-factor comparison is now complete at
`assets/paper_evidence/card_house_26_manifold_sensitivity_v2_r3/`. It changes only
Native contact-manifold mode between `Compact` and `FourPointPlanar`, holds the
reconstructed 26-card scene and every `paper_cpu` solver knob fixed, pins one
simulation thread to logical CPU 8, and excludes raw wall time from every
verdict. The v1 artifact is explicitly invalid because its frozen exit taxonomy
could not represent the observed full-duration terminal convergence-gate exit;
no scene, solver, collision, affinity, or order parameter changed for v2.

Both modes emit all 600 requested rows, but both have zero strict-success rows
and 600 rows containing at least one aggregate accepted capped group:

| Field | `Compact` | `FourPointPlanar` |
| --- | ---: | ---: |
| Process exit class | `complete_exit_zero` | `complete_terminal_convergence_gate_failure` |
| Aggregate capped groups / exact attempts | 3,495 / 5,757 | 682 / 745 |
| Exact failures / boxed fallbacks | 0 / 0 | 0 / 0 |
| Terminal last-group status | `success` | `max_iterations_accepted` |
| Terminal last-group residual | `8.525678738415048e-7` | `0.016582575623909489` |
| Contact range / mean | 39-155 / `73.63` | 124-200 / `167.4283333333` |
| Unique-pair range / mean | 30-57 / `43.5533333333` | 35-54 / `45.9283333333` |
| Mean pair multiplicity | `1.6905709475` | `3.6454258446` |
| Pair-identity transition rows | 538 | 140 |
| Multiplicity-transition rows | 542 | 166 |

The preregistered directional contact-multiplicity hypothesis is supported:
`FourPointPlanar - Compact` mean contacts is `+93.7983333333`, and mean
per-pair multiplicity is `+1.9548548971`. The convergence evidence is mixed but
does not support improvement: `FourPointPlanar` has 2,813 fewer aggregate
capped groups and fewer graph transitions, yet its terminal last-group residual
is `0.0165817230560` higher and fails the executable's terminal gate. Neither
trajectory is strict, so physical outcomes, timing, real-time, and paper-parity
verdicts are all null.

The frozen protocol SHA-256 is
`eeb1c8c1d09a29e197a3c402217ecd0af9a9878749cb4756cf94bc714f2b60e9`.
Current identity and integrity hashes are trace source
`b00eea0c87f75f17259fac433bafd98c20a961d14389a80e91742d3c8a678f76`,
trace executable
`0923bf7df1eaa518f9a2ffacd0a42fd7330f5cbf6c35cb5749fbc122db1310ff`,
runner
`e03356c772560f061e9b90fb4cd9f5df0c569631cd5e9fdd0857c337ff840562`,
summary
`52a082ab15e8b9c314d706474cc7be557ddfc58c4961faad0d3da9d347f59f4f`,
comparison
`051605c25ccd5aa4de2f243c4dafe547c82f7298f8018cee53a8701d018ff297`,
metadata
`5890ab138179f4d7aaee6cd04c63799086439bae58fbde4f994048785dd0b8ac`,
artifact index
`1703f995bd5d2c6edeb12ba936010a80609c3e952f2c782b554bd0be7ca40627`,
and report
`0429b12f90e4fef534c9b707324a180eaac78829a3b3667505111228aba58d2e`.
The unsuffixed v2 path is historical current-at-capture evidence. The current
v2_r3 invocation and whole-tree hashes are
`6e48eb835ab9aafca0b8d4bd29acb5ec932645fb37ca6124ce3c23b3265847f0`
and `953d8efd4d43ee2a74cede6b5e3a0a766ecd1fd8914485b50b340f515b7ecb77`;
diagnostic semantics are unchanged.

## Locally Validated Non-Paper Literal-Wedge Evidence

The locally sealed schema-v8 bundle is
`assets/dart_cpu_evidence/2026-07-19_mark26_native25_colored_v10_archwide_pcore`.
Its explicit contract is a reconstructed exact-inertia 25-wedge arch with
`1 um` closure, pinned springers, Native `FourPointPlanar`, and deterministic
manifold-colored inner BGS. It uses scale 35, a 5,000-outer cap, 30 fixed inner
sweeps, relaxation 1.1, fresh per-step gamma, and zero diagonal/matrix-free
seed. This is not the paper's 200-outer timing profile.

One warmup plus three measured 600-step runs complete at each of one and four
threads. Each thread count therefore contributes 1,800 measured steps. Every
measured step has 96 contacts and 24 colliding body pairs; the schedule remains
24 manifolds, three colors, and width eight.

| Field | One thread | Four threads |
| --- | ---: | ---: |
| Exact-success steps | 1,800 | 1,800 |
| Accepted caps | 0 | 0 |
| Exact failures | 0 | 0 |
| Boxed-LCP fallbacks | 0 | 0 |
| Maximum residual | `9.999807145410957e-7` | `9.999807145410957e-7` |
| Physical outcome | Valid standing trajectory | Identical valid standing trajectory |
| Whole-arch maximum displacement from initial | `5.431169776791696e-6 m` | `5.431169776791696e-6 m` |
| Minimum orientation alignment from initial | `0.9999999999111284` | `0.9999999999111284` |
| Mean step time | `6.122883 ms` | `4.269397 ms` |
| Median step time | `2.4966535 ms` | `1.9047965 ms` |
| p95 step time | `21.663237 ms` | `14.396602 ms` |
| Maximum step time | `287.473818 ms` | `180.504588 ms` |
| Mean below 60 Hz budget | Yes | Yes |
| Every step below 60 Hz budget | No | No |

The matched 1-to-4-thread pair has validated speedup `1.434133x`. This is a
mean-throughput result: average `World::step()` time is below 16.667 ms. It is
not an every-step latency guarantee: the one-thread p95 and both maxima exceed
the deadline, while the four-thread p95 is `14.396602 ms` and remains below
it. The bundle stores per-step terminal
residual diagnostics; it does not yet contain the paper-matched per-outer
Figure 9 curve, author impact trajectory, or comparable paper timing.

The finalized standing capture at
`assets/paper_evidence/fig07_arch25_literal/` independently steps the same
literal Native/`FourPointPlanar` colored-FBF contract. Its equivalence audit
compares all 600 capture and reference-trace rows with zero numeric or integer
differences, including the five zero-iteration warm-start rows. The visual
bundle is current-source, manually inspected, and has 19 indexed artifacts in
a 21-file physical directory plus five selected local stills; its metadata
SHA-256 is
`b217b0a43200ca558f09776e464d50f0f9dc5c5a175bcfa66ae1b656076871d1`
and artifact-index SHA-256 is
`4d173c78e2c6584d6d8a85867160cc0c0687fe0f35e6878f66b0cfcd7c1c15ee`.
This corroborates the per-step standing trajectory but still does not supply a
paper-matched per-outer Figure 9 curve. Its 10 simulated seconds are played in
6.1 seconds at 10 fps (`1.639344x` time-lapse), so it is not timing evidence.

The 70-file raw capture staging directory was pruned after local sealing. With
the compact local bundle present, verify-only uses the selected stills, clip,
timeline, trace, and provenance without that raw staging directory. The capture source, binary, numeric
trajectory, and decoded media remained unchanged through later additive
scenario work.
Fresh revalidation against the final current trace and Native source binds the
current trace source/binary hashes as
`b00eea0c87f75f17259fac433bafd98c20a961d14389a80e91742d3c8a678f76`
and
`0923bf7df1eaa518f9a2ffacd0a42fd7330f5cbf6c35cb5749fbc122db1310ff`.
A fresh reference trace with SHA-256
`0e41af230fb4c27143966c3f8522702280e6746b31e84155a9a047326e46dbb3`
was generated from that binary and again matched all 600 standing rows with
zero differences.

## Preregistered Crown-Impact v1 Negative

The locally retained numeric artifact is
`assets/paper_evidence/fig07_arch25_literal_impact_v1_negative_final_v9/`,
generated by `scripts/run_fbf_literal_crown_impact_negative.py`. The first
frozen run completed 720 steps and retained a projectile-free 600-step standing
prefix with zero mismatches across 88 eligible trace fields. First
projectile-arch contact occurs at step 607, before projectile-ground contact at
step 616.

All bodies remain finite with zero exact failures and zero boxed-LCP fallbacks,
but the preregistered acceptance claim fails independently on convergence and
physical localization:

- five solves are accepted at the iteration cap instead of zero;
- worst residual is `9.1545317042653963e-5` versus `1e-6`;
- final maximum arch displacement is `0.07093964431215687 m` versus
  `0.07 m`; and
- final far-field displacement is `0.060523747030465196 m` versus `0.007 m`.

The artifact is therefore a valid scientific negative, not a passing impact
or paper-parity result. No parameter or threshold was changed after observing
it. The final runner SHA-256 is
`622b388142fed881191dcb6efb103266eb2ea1e3ffbadaf402f63b89c558bb67`.
Bundle SHA-256 values are raw
`42cc94f5f111442da42b03af33722f713d66b853d3ac21f21083f28ba24e7ba4`,
stderr `7964b07fd5396f10013ff8d9100d2b36e7dfc151764210d8c60bd0ae853a2d94`,
standing reference
`22fabfe2e8f35bf329f32873845a60599ed2df171c15728f14311b91d888c387`,
summary `e3baa479111a999d03359a0642533cad1baa3a48c415609bafb9a7a7cfa18e7c`,
metadata `0bd19797953a10223bbd7183f6e8002e490b21c29b2304befee7e6618ff9ee73`,
and report `c5711b0fe06e679f889f6a7ca3812aa955c3b8c61270ef038def06b2a3f173ff`.
The normalized trace fingerprint is
`86b37ec37d28259514453c9ecc9e7d5f12afe08118a0b04abd40e72acd147384`.
The v6, v7, and v8 paths are historical current-at-capture evidence; v9
preserves the same frozen negative semantics and records the executed
`taskset` identity in its runtime closure.

## Literal 101-Stone v1 Scientific Negative

The current provenance-bound artifact is
`assets/paper_evidence/fig08_arch101_literal_v1_negative_final_v7/`. The frozen
Native exact-inertia reconstruction emits one row, then fails closed on step 1
after 5,000 outer iterations with `fbf_failed`, residual
`0.7815364614352474`, one exact failure, and zero boxed fallbacks. Its dynamic
`FourPointPlanar` aggregate fields record 400 contacts, 100 constraint pairs,
three colors, and width 34. A separate repeat-2 Compact collision-only probe
proves only the constructed time-zero graph of 100 adjacent-stone pairs plus
two springer-ground pairs. The v7 one-step FourPointPlanar companion resolves
the failed step-1 pre-solve graph as exactly the 100-edge adjacent-stone chain:
100 unique adjacent pairs, 400 contacts, multiplicity four, zero non-adjacent
pairs, and zero ground pairs. Its aggregates and residual match the frozen
trace. The companion accepts the capped iterate and does not follow the frozen
trace participant-affinity contract, so solver-taxonomy and affinity
equivalence remain false. This narrow identity result supplies no source
equivalence, valid trajectory, standing/physical outcome, timing, media,
long-run behavior, or paper parity.

The artifact remains `artifact_valid=false`, `standing_claim_passed=false`,
and `timing_evidence_eligible=false`. Its normalized fingerprint is
`8d275edcaa82a2f628fdd5d9f846b2daed7e6864fdc4b7e80eca9a8c93bcf527`.
Current raw, invocation, summary, metadata, and report hashes are
`fc0705ff497b44658dfe186ed27c50b2ab9d19a268f20cf50875a3155d1dc07d`,
`feb21f2094a1ec4103fd79b0474e3ce59e2815d2f502373998861d838fa85b15`,
`2cae961048b776c069caeccda2d95f2f0fd0969cae9e3de3782f0e5e5b7b640d`,
`770d662f7021320982700fe699266bc94d2448679c8c4dc60bd127c660facb8a`,
and `1ba3987af896f68401faa1153f6b388aed38b087214d07592f5a060cab429e2a`.
The current v7 whole-tree hash is
`e65107a6d2e2880742e02d01949bc4b3f9adb2f48b6be552a720fa83c9bcc4b3`.
The v3 bundle was superseded after additive instrumentation, and v4 is
historical current-at-capture evidence. The unchanged command was rebaselined
as v5 and again as v6 after the current-build libdart identity advanced. V7
adds the identity-resolved one-step dynamics companion; the frozen trace and
scientific result are unchanged.

## Pinned-Author Masonry-Arch Scientific Negative

The sealed current-source bundle is
`assets/paper_evidence/author_masonry_arch_reference_v1/`. The author
invocation uses 500 frames, four substeps per frame, and releases three cubes
at frame 400 / substep 1,600. The checked-in default is 400 frames with
`drop_frame=400`, so it never releases the cubes. This is therefore a newly
declared diagnostic, not a historical or paper invocation.

A deterministic projection represents every one of the 2,000 substeps and is
lossless with respect to the declared claim fields. The 382,753,953-byte raw
source history (SHA-256 `cec0e4b86837e7542c498c7ddad40538983ec023332b88ebddee7766997e3ac1`)
is size/hash-bound but omitted. The projection separates two residual concepts
that must not be conflated:

| Field | Observation |
| --- | --- |
| Configured outer termination gate | nonnegative `coulomb_rel < 1e-6` |
| Initial shortcut | natural initial residual `< 1e-6`; 40 true flags, all before release |
| Author convergence flags | 157 true: 40 initial-shortcut plus 117 configured-outer-gate; 1,843 false outer solves |
| Pre-release flags | 142 true (40 initial-shortcut, 102 configured-outer-gate), 1,458 false |
| Post-release flags | 15 true (all configured-outer-gate), 385 false |
| Natural `final_residual <= 1e-6` | 47 substeps |
| Release substep 1,600 | nonconverged; 100 contacts; natural residual `0.017456069692858667` |
| Final substep 1,999 | nonconverged; 108 contacts; natural residual `0.5161195175386001` |
| Maximum natural residual | `4.1130565788445415` at substep 226 |

The first post-release contact-count increase is inferred at substep 1,944,
but the projection carries no pair identities, so it is not definitive
cube-arch contact evidence. Exit zero and artifact integrity mean that the
diagnostic completed and was preserved; they do not make the solver contract
valid.

The companion DART specification records author geometry and solver constants
but is explicitly configuration-only. It implements no source
collision/contact-gap/backend/float32 dynamics semantics and executes no DART
dynamics. The bundle establishes no DART or cross-solver dynamics, trajectory,
physical-outcome, Fig. 7/video.07, timing, repeatability, or media parity.

A separately named DART dynamics adapter now binds the raw-scale initial state
and exact/boxed solver contracts. Read-only probes isolated the former step-68
failure to finite, well-conditioned 3x3 boundary systems whose large reactions
made the scalar complementarity dot product cancellation-sensitive. The
current local-QP change preserves the ordinary KKT fast path and its
primal/dual tolerances, then checks a rejected positive-friction boundary
candidate through fused long-double normal-cone stationarity. This is an
independent approximate KKT certificate: exact KKT is equivalent, while its
floating-point tolerance decision deliberately avoids multiplying
component-level gradient error by the large reaction. The ordinary scalar-gap
predicate is unchanged, and the current path does not perturb the candidate.

The clean no-compiler-cache 100-step run at
`/tmp/fbf_author_arch_exact100_normal_cone_clean_v3/` completes every requested
step. At step 68 it reports residual `9.8373696597135509e-7`, 60/60 exact
attempts, zero accepted caps/failures/fallbacks, 58 warm starts, and 24
contacts. At step 100 it reports residual `9.9838678932416298e-7`, 124/124
exact attempts, zero accepted caps/failures/fallbacks, 122 warm starts, 48
contacts, and trajectory worst residual `9.9936331058309156e-7`. The timeline
SHA-256 is `6ba8978194286532bb3d253b87154c2454941bc00ee90c536a522adbcc7efb21`.

The clean no-compiler-cache 200-step fail-fast run at
`/tmp/fbf_author_arch_exact200_normal_cone_clean_v3/` reaches a distinct outer
convergence failure. Step 141 succeeds at residual
`9.9900048932672015e-7` after 839 iterations, with 210/210 local solves, zero
caps/failures/fallbacks, and 88 contacts. Step 142 raises contacts to 96; all
211 local attempts have solved, but the outer solve exhausts 5,000 iterations,
is accepted at the cap, and ends at residual `8.6992951837150444e-4`. The
fail-fast reason is `iteration_cap`; the timeline SHA-256 is
`be8819eabe57127edcb234bf03250c9e4875452710f6eeb8d7310b45dbceace8`.
Both commands are preserved in the timeline `runtime_command` fields and use
the named scene, `--threads 1`, `--headless-exact-fbf-fail-fast`, and
respectively `--steps 100` and `--steps 200`. This is a local diagnostic, not a
2,000-step residual history, release, or impact result.

Rejected experiments remain non-evidence. FMA-only and long-double
scalar-gap-only variants still failed step 68 after a clean rebuild. Global
gamma/tolerance widening changed the acceptance boundary and was rejected.
The existing projected-gradient retry only moved the failure to step 69.
Radial/cubic ULP-neighbor searches produced later cascades and were removed.
Earlier apparent long-run successes were contaminated by a stale rejected
gamma-bound build and must not be cited. A long-double boundary-multiplier
Newton prototype was also removed after review because the rejected candidates
already satisfied the normal-cone certificate and no Newton update was needed.

## Pinned-Author Incline Sweep Scientific Negative

The numeric current-source packet is
`assets/paper_evidence/author_incline_sweep_reference_v1/`. Independent FBF,
MuJoCo, and Kamino CPU invocations each use
`mu=.3,.4,.45,.5,.55,.6,.8` and 120 steps per cell, for 840 rows per lane.
The retained FBF histories record four contacts per FBF step; the MuJoCo and
Kamino result records contain no contact-count field.

| Field | Observation |
| --- | --- |
| FBF configured convergence | 839/840 true; `mu=.55` is 119/120 |
| Sole false flag | `mu=.55`, step 1, after 200 outer iterations |
| True-flag mechanism | 235 initial natural-residual shortcuts; 604 configured outer nonnegative `coulomb_rel < 1e-6` accepts |
| Natural residual among true flags | 456 at or below `1e-6`; 383 above `1e-6` |
| False-row metric split | Natural `final_residual=3.273267262002487e-8`; configured terminal `r_coulomb=1.5311460572898186e-6` |

The natural residual and configured decision are different metrics and must
not be substituted. The normalized displacement projection places FBF and
Kamino close while the current MuJoCo curve is nonmonotone, but no full-state
or cross-solver parity follows. JIT, history collection, warmup handling, and
timer-boundary differences make all timing values diagnostic-only. This
packet establishes no DART match, historical paper invocation, approved
golden, media, timing, performance, or parity result, and it does not change
the six-bundle visual inventory.

## Rank-Deficiency Diagnosis

The contact-rich system is necessarily semidefinite and highly redundant. At
90 contacts, 270 contact rows act through 156 generalized velocities. The
contact response therefore has nullity at least `270 - 156 = 114`. At 155
contacts, 465 rows give a lower bound of `465 - 156 = 309`.

This, together with contact-set churn, is the leading diagnosis for the
contact-rich global failure. It is consistent with a robust local 3x3 solve
and a globally difficult redundant system. It is not proof that rank
deficiency is the only cause, and it does not weaken the convergence gate.

## Small-Scenario Residual Matrix

The current-source strict small artifact is retained in the ignored local evidence cache at
`assets/dart_cpu_evidence/2026-07-19_current_source_paper_cpu_small_r7/`.
Its exact 60-file index binds three repetitions per row plus source, executable,
runtime, affinity, and post-run identity evidence:

| Scenario | Steps | Maximum residual | Solver residual gate | Physical outcome gate |
| --- | ---: | ---: | --- | --- |
| Backspin | 240 | `9.994e-7` | Pass | Pass |
| Incline, `mu=0.4` | 120 | `9.988e-7` | Pass | Pass |
| Incline, `mu=0.5` | 120 | `1.439e-6` | Fail: three accepted caps | Pass: `8.634e-7 m` displacement |
| Painleve, `mu=0.5` | 150 | `9.996e-7` | Pass | Pass: upright |
| Painleve, `mu=0.55` | 150 | `9.950e-7` | Pass | Pass: tumbled |
| Turntable, `mu=0.2`, `omega=2` | 240 | `1.000e-6` | Pass | Pass: ejected |
| Turntable, `mu=0.2`, `omega=5` | 240 | `9.981e-7` | Pass | Pass: ejected |
| Turntable, `mu=0.5`, `omega=2` | 240 | `9.999e-7` | Pass | Pass: retained on support over the measured horizon |
| Turntable, `mu=0.5`, `omega=5` | 240 | `3.050e-4` | Fail: three failed processes | Classifier pass: ejected |

All nine physical classifiers pass. Seven of nine rows satisfy the combined
local residual, affinity, physical-outcome, and real-time contract. Paper
timing remains null for all nine because the source workload and host are
unmatched, and this run used zero warmups.

Core r7 SHA-256 values are report
`008bc94667893cd26bfc04a720caca3d3a5703601c8739bc06856f93818c63d5`,
artifact index
`06594c1e1cc3c6858bc78a80630aefb0e85eeef92204b0a09b99425411c3ebeb`,
summary JSON
`9137f1b2db0909a96897632a66617f2cfa58476a369d2adfe232b73e46cd0fa2`,
and metadata
`e227d7aef3b273ff81385f227f9e6766a7e40a622bc4298e1807b7c2068bc417`.
Earlier working runs are historical; only the indexed r7 bundle is canonical
current-source evidence.

## Finalized Incline Lane-Separated Residual Result

The current-source `dart_best` visual/trace bundle is
`assets/paper_evidence/fig01_02_incline_current_v1/`. With the compact local
bundle present, finalization and verify-only pass with status
`valid_current_source_nonpaper_incline`. The directory contains 23 physical
files and its exact-membership index binds 21 artifacts.

The combined capture records 240 exact attempts/solves with zero accepted
caps, exact failures, or boxed-LCP fallbacks and maximum residual
`9.999836962261359e-7`. Five selected local stills and a 61-frame decoded
660x506 H.264 schedule retain the visual evidence at 30 fps. The 70-file raw
capture staging directory is pruned after local sealing, so verify-only does
not require that raw staging. The capture reports eight contacts per post-initial step.

The two independent tracked traces each contain 121 rows, 120 exact solves,
119 warm starts, zero fallbacks, three contacts per post-initial step, and
continuous post-initial tracked contact:

| Trace | Downhill displacement | Reference/threshold | Maximum residual | Final speed |
| --- | ---: | --- | ---: | ---: |
| `mu=.4` | `1.7686892884927794 m` | analytical `1.7548661487418349 m` | `9.986952135669881e-7` | downhill `1.7544655347780056 m/s` |
| `mu=.5` | `0.0008905412965980523 m` | stick tolerance `0.02 m` | `9.997210606407098e-7` | maximum/final stick speed `0.001116442058867632 m/s` |

The tracked downhill-displacement separation is
`1.7677987471961814 m`. Capture and aggregate trace projections are
byte-identical only for `step`, cumulative exact solves, and cumulative
boxed-LCP fallbacks, SHA-256
`f03dff0aaec5f0fa6615609c2ea97aa31f072c14c6212fffe15858ea969d88c2`.
Capture contacts 8 do not match aggregate trace contacts 6;
`contact_count_match=false` and `contact_counts_compared=false`. The combined
renderer and independent traces use different placements, so state, residual,
status, warm-start, per-cell, and full-trace equivalence are not established.

Core SHA-256 values are metadata
`7a5f973a9b7264911058ec91e253dfcf5d72a7ec46fa7020df0020af1a259b7d`,
index `b758bd28965bf9a96be7668c0dbb738b72c1493d83f195a3e726ae891f8f6e85`,
manual inspection
`3c3af65d62c629ae836302910a2fe7f928ab398628f280221f6d0b5d94d5a848`,
trace summary
`4df130e878f1e58d478870c8f132ee165061a752520e926f44c331d32f14f20d`,
verification
`2681073ee44f7fecd2782081826b414ee6541bb930149ced91f60a17dc2416d5`,
report `f75efcd40bd0452bcbdc7bbc82eee0fdbd78d7a1059f974e83999467b1688fa5`,
panel `f9f211fb376c97d98bccc67806ba3e1c9905d7d27764c794e1c480af7b4df9d3`,
clip `ad6d00ae6a614b0edbc836396c30621b589295a0eec826d060663f6782cee3f9`,
and traces
`449acf19feef2e0aa7fb04bb9f45f865727ba59f626b3964114cd900169ecd8a`
and
`2b30e8033b123876ad1cdea755741fd230a4d72d3747e55b907e6427962659c5`.
Bound finalizer/test, runner/test, demo source/binary, trace source/binary,
fixture source, and libdart hashes are
`705da2a308697b4b4b923894d10f23622310e024aeab49862a24593c79142e23`,
`02f3800cf7cf5df5d85b1950512e78df6e413fea42cfa89880775f010d208e1c`,
`d848afa53caf14b9fb3ea061d658eef274e8d917151937bd6340283b79ab5432`,
`6e378252fa6a7cb51c6813c9d5a2b30b8c8129eacdb01df0ee19a58a270cbc5e`,
`84fd1330a13d548760e537f9734790ab1b5c91fcda3e446bf76b9b36f3e1aa99`,
`d838f23e9fb04224ff194658bd6e53d8cbb95ac94ce6676e54c49b8ea25916e4`,
`b00eea0c87f75f17259fac433bafd98c20a961d14389a80e91742d3c8a678f76`,
`0923bf7df1eaa518f9a2ffacd0a42fd7330f5cbf6c35cb5749fbc122db1310ff`,
`a4bed7372213d544fef62923b88bc9accee7086165d7b3cb20245bee8bade05f`,
and
`8fae2320858e49fdda309d89df8cb1158c1cc5dc11d345e14f5adca0ff63cf3d`.

This result is lane-separated from the strict small matrix above. The strict
`paper_cpu`/Native `mu=.5` row passes its physical classifier with displacement
`8.63436433e-7 m`, but one accepted cap per repetition raises maximum residual
to `1.4392081500753078e-6`; its strict solver/local-real-time contract remains
failed. The valid `dart_best` threshold result cannot erase that strict
negative. Fig. 1, Fig. 2, and video.03 remain `partial` without the full
friction sweep/plot, matched external rows, approved source golden/diff,
paper contact-count match, full 11 s semantic edit, paper timing, and
real-time parity.

## Author-Pinned Fig. 4 Lane-Separated Residual Result

The finalized bundle
`assets/paper_evidence/fig04_turntable_author_current_v1/` binds four
author-configured cells at pinned commit
`b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0`. In the current
`dart_best`/Native `FourPointPlanar` visual lane, all four 360-step runs satisfy
their residual/solver contracts with no fallbacks and support the exact
six-second outcome ejected, ejected, retained on support, ejected.

The separately recorded strict `paper_cpu_native` lane has no capture
comparison and no cross-lane substitution. Its `mu=.5, omega=2` run fails at
step 40 with residual `7.407835021099202e-6`; the other three strict rows pass,
so `paper_cpu_native_all_solver_contract_valid=false`. This strict negative
does not invalidate the visual-lane finite-horizon physical predicate, and the
visual pass does not erase the strict residual failure. Retention through 6 s
does not prove zero slip, perfect sticking, co-rotation, or longer-horizon
stability.

Bundle status is `valid_author_source_pinned_nonpaper_turntable_matrix`.
The bundle has 58 indexed artifacts in a 60-file physical directory, including
four selected local, timeline-bound outcome stills. With the compact local
bundle present, verify-only does not require the pruned raw capture staging.
Core hashes are
report
`930cc12b95ab78c6e61d084064b584f5872633f2432e434c68d071818cec7fb1`,
artifact index
`209b677ce35e2b9c11248ab414727016394831084881a5e9c2866f68b51f6cdf`,
metadata
`854ef0c8aad75b4b200f512951d56b4dbef7aa82c46cc5e82123f0583dcc6ac5`,
manual inspection
`095652d3df70a144be31be49b0e25b3265df54a3815df21742333d5fdfb4529a`,
trace summary
`c50ad532d1c95564c2dc7d236ebb263e83932cafcd0ec77013ed8a433336ab22`,
verification
`455da7686eaeeb989e0d122c4724ea66ff161e8d652230eff9fb8ad20590bacd`.
Current visual runner/test bindings are
`d848afa53caf14b9fb3ea061d658eef274e8d917151937bd6340283b79ab5432`
and `6e378252fa6a7cb51c6813c9d5a2b30b8c8129eacdb01df0ee19a58a270cbc5e`.
The six-field capture/trace projections are byte-identical for all four cells,
but that is not full-state equivalence or paper parity.

The finalized author card-house construction bundle has 12 indexed artifacts
in a 14-file physical directory and shows the public-author default
five-level, 40-card configuration at step zero. It executes zero simulation
substeps, so it supplies no residual, solver, release, standing, trajectory,
contact-dynamics, physical-outcome, historical four-level/26-card trajectory,
Fig. 6/video, timing, performance, or parity evidence.

## DART-Best And Multicore Boundary

The raw one-thread/eight-thread DART-best matrix is retained at
`/tmp/fbf_cpu_dart_best_postreview_20260712_t1_t8_r3`. It is not a paper
profile. Its legacy exact solve for a coupled island is serial, so requested
world threads do not demonstrate parallel exact-solver work and that matrix
supports no multicore claim even where a raw timing ratio exceeds one.

The literal-wedge bundle uses a different, explicitly opt-in non-paper
contract. It records persistent four-worker colored dispatch, width eight,
and per-phase residency on pinned CPUs 8, 10, 12, and 14 before validating the
matched-work speedup. That result must not be transferred back to the legacy
matrix or to the paper CPU rows.

## Residual Infrastructure

`fbf_paper_trace` retains bounded per-outer samples and exports fields such as:

```text
step,time,scenario,solver,solve_index,solve_contacts,outer_iteration,
residual,primal_feasibility,dual_feasibility,complementarity,step_size,
safe_step_size,coupling_variation_ratio,shrink_iterations,contacts,
exact_solves,warm_starts,fallbacks,status
```

The demo-host timeline is a separate additive diagnostic surface. For exact
FBF, `solver_diagnostics.last_failure` is `null` when no exact failure has been
recorded. When cumulative `exact_failures` is nonzero, it retains the failed
group's adapter/build/FBF statuses, contact count, residual components and
worst-contact indices, iteration/best-iterate data, gamma diagnostics, coupling
ratio, and shrink count even if a later constrained group succeeds. Cumulative
attempt/solve/failure/cap/fallback counters remain the acceptance authority;
the retained object must never be read as permission to advance a failed
strict trajectory.

Schema-v8 performance rows also disclose and validate outer cap, tolerance,
cap acceptance, local iterations, adaptive step, warm start, retry, dense
polish, fallback, seeds, scale, relaxation, initial-gamma contract, split
impulse, affinity, scenario outcome, and complete-trajectory status. Schema v8
preserves the default 83-column trace byte-for-byte; the newline-terminated
default header SHA-256 is
`396c866d782e626b5c5b52e74c392de1175f5e46dfd90b809d1f219417092e50`.
The explicitly non-paper colored contract appends dispatch, schedule, and CPU
residency fields plus whole-arch initial-pose outcome fields in a separate
95-column trace. Its newline-terminated header SHA-256 is
`424195336cb42753179130c9f6fcba3a8ddea9bf669cfa70c0f77fcb4c6335a5`.
The explicit card-manifold sensitivity selector instead appends eleven
instrumentation fields to the default schema, producing a separate 94-column
trace with newline-terminated header SHA-256
`007311fb28062377dd6a0d26cad1ab4f7e2c99f359afd33554651f3cc0929ef5`.

Interpret the predicates independently:

- `artifact_valid` means the raw/summary files and schema are intact;
- `solver_contract_valid` means every required solve passes;
- `physical_outcome_valid` means the scenario endpoint/trajectory check passes;
- `manual_inspected` applies to a rendered plot or video, not a numeric row;
- `claim_valid` requires every predicate and comparability condition needed by
  that claim.

The current card-v2 comparison is integrity-valid while both strict trajectory
claims are false and its physical, timing, and paper verdicts are null.

## Geometry Boundary For Figure 9

Production arch traces use weighted-catenary oriented boxes:

- 25-stone natural box manifold: 96 contacts; GUI reduced profile: 48;
- 101-stone full profile: reaches the 512 cap, so the uncapped count is
  unknown; GUI reduced profile: 38.

They are not literal tapered author geometry. The separate exact-inertia
literal-wedge collision probe remains collision-only and yields 24 nominal
adjacent pairs or 26 with its closure/ground audit. It proves no dynamic
residual or paper-100-contact result.

The locally retained literal-wedge dynamic trace is a distinct Native
`FourPointPlanar`, `1 um`-closure reconstruction. Its 96 contacts, 24 pairs,
successful 600-step outcomes, and residual bound establish local DART dynamics
for that named contract only. They do not retroactively turn the collision
probe into dynamics evidence or reproduce the author 100-contact row.

The pinned-author 500-frame diagnostic separately records 100 contacts at
release. Forty true flags use the initial natural-residual shortcut, 117 use
the configured outer `coulomb_rel` gate, and 1,843 outer solves are
nonconverged. Its history does not serialize contact-pair identities. It is an
author-source scientific negative, not a source-matched DART trajectory or a
passing Figure 9 curve.

The bounded oriented-box visual runs independently show the 25-stone
reconstruction collapsed by step 24/360 (0.4 s) and the 101-stone
reconstruction collapsed by step 120/600 (2 s). No valid long sidecars/media
were produced for those contracts. The separate finalized literal-wedge bundle
validates only its no-projectile standing reconstruction and does not override
the oriented-box failures. The preregistered impact v1 bundle adds locally
sealed negative crown-impact evidence, not a passing outcome. The separate literal
101-stone v7 artifact provides an identity-resolved step-1 exact-dynamics
blocker, not a standing trajectory or media claim.

## Superseded Artifacts

Retain the following only as historical diagnostics:

- inverse/Euclidean local-update traces and timings;
- reduced-contact or one-step rows used to imply full-trajectory success;
- the earlier ten-step full-card table as a prefix diagnostic;
- old 56/60-contact fallback-boundary narratives;
- full-manifold SVGs with superseded arch geometry/count labels;
- the prior 10 s card sequence with 24 post-impact boxed fallbacks;
- high-budget DART-best traces described as strict paper evidence.

None overrides the prior-source strict-card step-89 fail-closed result, the
current-source non-strict card-v2 sensitivity result, or the separately scoped
positive literal-wedge bundle.

## Reproduction Entry Point

Rebuild the trace target before generating evidence, use the full card
scenario, and preserve the nonzero result:

```bash
cmake --build build/default/cpp/Release --target fbf_paper_trace --parallel 4

taskset -c 4 .pixi/envs/default/bin/python \
  scripts/run_fbf_cpu_evidence.py \
  --binary \
    build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace \
  --output-dir /tmp/fbf-paper-cpu-strict \
  --case card_house_26_settle_projectile_full:600 \
  --threads 1 --cpu-list 4 --repetitions 1 \
  --solver exact_fbf --contract paper_cpu \
  --collision-frontend native --accept-nonzero
```

Reproduce the separate non-paper literal-wedge scaling contract with a fresh
output directory:

```bash
.pixi/envs/default/bin/python scripts/run_fbf_cpu_evidence.py \
  --binary \
    build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace \
  --output-dir /tmp/fbf-literal-wedge-colored-v9 \
  --case masonry_arch_25_literal_wedge:600 \
  --threads 1,4 --cpu-list-for 1:8 --cpu-list-for 4:8,10,12,14 \
  --repetitions 3 --warmup-repetitions 1 \
  --solver exact_fbf --contract dart_best_colored_bgs \
  --collision-frontend native
```

Replay the finalized incline residual and count-projection contracts without
modifying the bundle:

```bash
.pixi/envs/default/bin/python scripts/finalize_fbf_incline_visual.py \
  --bundle \
    docs/dev_tasks/fbf_exact_coulomb_friction/assets/paper_evidence/fig01_02_incline_current_v1 \
  --verify-only
```

Verify the sealed author masonry-arch diagnostic without modifying it:

```bash
.pixi/envs/default/bin/python \
  scripts/finalize_fbf_author_masonry_arch_reference.py --verify-only
```

Verify the pinned-author incline sweep without modifying it:

```bash
python3 scripts/finalize_fbf_author_incline_reference.py --verify-only
```

Reproduce the frozen current-source card-manifold comparison with a fresh
output directory:

```bash
.pixi/envs/default/bin/python \
  scripts/run_fbf_card_manifold_sensitivity_v2.py \
  --binary \
    build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace \
  --output-dir /tmp/fbf-card-manifold-sensitivity-v2-rerun
```

Consult `--help` if the runner CLI changes. Keep raw rows, stderr, invocation,
metadata, and summary together. A failed residual contract is negative
evidence, not a reason to omit the run or substitute a reduced scenario.

## Remaining Figure 9 Work

- Make the strict full-card trajectory finish within the paper cap and
  tolerance without fallback.
- Preserve the source-selected Figure 6 step-35 record as a strict negative.
  Preserve the terminal-estimate A/B as a numeric strict-prefix negative, not
  Figure 9 residual-history evidence.
  Keep both the historical debugger preview and the reproducible
  source-continuation full run outside strict trajectory/outcome and Figure 9
  claims; only the latter is a validated, separately labeled visual attachment
  candidate.
- Preserve the completed v2 sensitivity result as diagnostic-only evidence;
  do not promote either non-strict trajectory or its raw wall times.
- Preserve the pinned-author run as scientific-negative source evidence;
  never equate its natural residual with the configured convergence flag or
  promote it into a DART-parity curve.
- Generate full-duration strict card and arch histories from final reviewed
  source.
- Generate a per-outer literal-wedge residual-history artifact if it is to be
  compared with Figure 9; the current schema-v8 timing bundle is per-step.
- Preserve scene/contact/precision/profile/affinity metadata with every plot.
- Produce paper-matched panels only when author data are available, or retain
  the external blocker explicitly.
- Revalidate CSV/SVG integrity and manually inspect every rendered plot.

Until those gates close, Figure 9 status remains partial and the strict
full-card result remains an honest negative.
