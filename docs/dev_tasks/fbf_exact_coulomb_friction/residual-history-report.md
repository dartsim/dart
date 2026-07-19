# FBF Residual-History Report

## Current Status

Residual-history infrastructure exists, but paper-matched Figure 9 evidence is
partial. The prior-source 600-step strict full-card attempt remains the
authoritative paper-profile contact-rich result and fails closed at step 89. A
current-source, one-factor card-manifold comparison now completes 600 emitted
rows in both modes, but neither trajectory is strict and its physical, timing,
and paper verdicts remain null. A separate durable bundle establishes a
positive 600-step exact result for a non-paper literal-wedge arch; neither
current bundle provides the paper's Figure 9 per-outer residual history.
The separate pinned-author incline packet preserves the current Figure 1
seven-cell FBF/MuJoCo/Kamino numeric sweep, including a configured FBF
convergence negative; it is not a Figure 9 curve or timing evidence.
The enclosing task remains active and incomplete at 24 partial, 5 blocked, and
0 complete across 29 requirements; six local visual bundles are finalized,
and the visual workflow declares 17 schedules.

## Prior-Source Strict Full-Card Evidence

The byte-identical repository archive is
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
best-iterate diagnostics.

With fallback disabled, the failed group is left unsolved and the evidence
caller stops. Later states are not generated from an invalid impulse.

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

## Durable Non-Paper Literal-Wedge Evidence

The durable schema-v8 bundle is
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
a 21-file physical directory plus five durable stills; its metadata
SHA-256 is
`b217b0a43200ca558f09776e464d50f0f9dc5c5a175bcfa66ae1b656076871d1`
and artifact-index SHA-256 is
`4d173c78e2c6584d6d8a85867160cc0c0687fe0f35e6878f66b0cfcd7c1c15ee`.
This corroborates the per-step standing trajectory but still does not supply a
paper-matched per-outer Figure 9 curve. Its 10 simulated seconds are played in
6.1 seconds at 10 fps (`1.639344x` time-lapse), so it is not timing evidence.

The 70-file raw capture staging directory was pruned after sealing;
clean-checkout verify-only uses the durable stills, clip, timeline, trace, and
provenance without ignored files. The capture source, binary, numeric
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

The durable numeric artifact is
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
`assets/paper_evidence/fig08_arch101_literal_v1_negative_final_v6/`. The frozen
Native exact-inertia reconstruction emits one row, then fails closed on step 1
after 5,000 outer iterations with `fbf_failed`, residual
`0.7815364614352474`, one exact failure, and zero boxed fallbacks. Its dynamic
`FourPointPlanar` aggregate fields record 400 contacts, 100 constraint pairs,
three colors, and width 34. A separate repeat-2 Compact collision-only probe
proves only the constructed time-zero graph of 100 adjacent-stone pairs plus
two springer-ground pairs; it is not dynamic pair-identity evidence.

The artifact remains `artifact_valid=false`, `standing_claim_passed=false`,
and `timing_evidence_eligible=false`. Its normalized fingerprint is
`8d275edcaa82a2f628fdd5d9f846b2daed7e6864fdc4b7e80eca9a8c93bcf527`.
Current raw, invocation, summary, metadata, and report hashes are
`03fba1d83f9209b0ade3698bdada42ca3927c176397a5ddee8ce0ca175e82947`,
`66dd6f2ceef2b50d5b0960c86fc291d27f6353d228c59161c22f18dd94c12522`,
`2e03653745a1e8d94c7b18d446004d16bfd4e176d7b9315aa198e3c08538bbfa`,
`5e0b166171e08349847664b504b0a9b15cc1f582514388b532259ad6edad2bec`,
and `52e5592ba5ad32a919f221e1a8e98ecb4b4f82c0228e16de44318b436bf58958`.
The current v6 whole-tree hash is
`ce5731fa6e4f967845d664341761eca7b0ce9ffb4cb93d55ac71df79f9243947`.
The v3 bundle was superseded after additive instrumentation, and v4 is
historical current-at-capture evidence. The unchanged command was rebaselined
as v5 and again as v6 after the current-build libdart identity advanced; the
scientific result is unchanged.

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

The current-source strict small artifact is repository-archived at
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
`assets/paper_evidence/fig01_02_incline_current_v1/`. Finalization and a
clean-checkout verify-only pass with status
`valid_current_source_nonpaper_incline`. The directory contains 23 physical
files and its exact-membership index binds 21 artifacts.

The combined capture records 240 exact attempts/solves with zero accepted
caps, exact failures, or boxed-LCP fallbacks and maximum residual
`9.999836962261359e-7`. Five durable stills and a 61-frame decoded 660x506
H.264 schedule retain the visual evidence at 30 fps. The 70-file raw capture
staging directory is pruned after sealing, so verify-only needs no ignored
staging. The capture reports eight contacts per post-initial step.

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
four durable timeline-selected outcome stills. Capture staging is pruned after
sealing, and clean-checkout verify-only needs no ignored files. Core hashes are
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

The durable literal-wedge dynamic trace is a distinct Native
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
the oriented-box failures. The preregistered impact v1 bundle adds durable
negative crown-impact evidence, not a passing outcome. The separate literal
101-stone v6 artifact provides a precise step-1 exact-dynamics blocker, not a
standing trajectory or media claim.

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
