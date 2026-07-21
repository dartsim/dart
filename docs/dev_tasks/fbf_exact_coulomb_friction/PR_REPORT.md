# PR Report: Exact Coulomb FBF Reconstruction

This is the PR-facing status report for the DART 6.20 exact-Coulomb FBF work.
It states what the current branch demonstrates and, just as importantly, what
it does not demonstrate.

## Direct Coverage Answer

No, this branch does not reproduce every test, benchmark, GUI example,
physical outcome, and performance result from the paper.

The implementation is active and partial. It now has durable evidence for
mean-real-time throughput and multicore acceleration on one explicitly
non-paper reconstructed literal-wedge workload. Do not broaden that result to
paper parity, an every-step real-time guarantee, the paper's timing rows, or a
claim that DART beats the paper.

The manifest remains 29 requirements: 24 partial, 5 blocked, and 0 complete.
The local visual inventory has six finalized bundles, and the visual workflow
declares 20 schedules.

The sealed pinned-author masonry-arch run is scientific-negative source
evidence, not a DART comparison: 40 true flags use the initial
natural-residual shortcut, 117 use the configured outer `coulomb_rel` gate,
and the other 1,843 outer solves are nonconverged.

A separately named source-configuration DART demo and 2,000-step capture
schedule are now registered. A clean current-build exact run clears the former
step-68 local-QP failure and completes 100 steps with 124/124 exact attempts,
zero accepted caps/failures/fallbacks, and worst residual `9.9936331e-7`. A
fail-fast run instead reaches an outer iteration cap at step 142: all 211 local
exact attempts solve, but the 5,000-iteration outer solve ends at residual
`8.6992952e-4` with 96 contacts. This is a diagnostic solver blocker, not
completed impact evidence or promotable media.

The separate pinned-author incline sweep is also numeric current-source
scientific-negative/reference evidence. It covers all seven Figure 1 friction
values in the public FBF, MuJoCo, and Kamino runners, but it is not a matched
DART result, a historical paper run, a seventh visual bundle, or timing,
performance, golden, media, or parity evidence.

## Durable Literal-Wedge CPU Evidence

The current durable bundle is
`assets/dart_cpu_evidence/2026-07-19_mark26_native25_colored_v10_archwide_pcore`.
It records one successful warmup plus three successful measured 600-step
trajectories at each of one and four threads, or 1,800 measured steps per
thread count. The aggregate `raw.csv` retains the warmups as well as the
measured rows; timing summaries exclude warmups.

This is the explicit
`reconstructed_literal_wedge_arch_nonpaper_native_collision_frontend`
contract: 25 exact-inertia wedges, `1 um` closure, pinned springers, Native
`FourPointPlanar` collision, float64 DART on x86-64 Linux, and the opt-in
deterministic manifold-colored inner BGS. The exact options are scale 35,
5,000 outer iterations, 30 fixed inner sweeps, outer relaxation 1.1, fresh
per-step gamma, and no diagonal or matrix-free seed.

Across every measured step, the trace records 96 contacts, 24 colliding body
pairs/manifolds, three colors, maximum color width eight, exact success,
residual `<=9.999807145410957e-7`, and zero accepted caps, exact failures, or
fallbacks. The one- and four-thread runs have the same valid standing physical
outcome. Across all 25 stones, maximum displacement from the constructed
initial state is `5.431169776791696e-6 m` and minimum orientation alignment is
`0.9999999999111284`.

| Threads | Mean ms | Median ms | p95 ms | Max ms | Mean below 16.667 ms | Every step below 16.667 ms | Validated speedup |
| ---: | ---: | ---: | ---: | ---: | --- | --- | ---: |
| 1 | `6.122883` | `2.4966535` | `21.663237` | `287.473818` | Yes | No | `1.0x` |
| 4 | `4.269397` | `1.9047965` | `14.396602` | `180.504588` | Yes | No | `1.434133x` |

The supported claim is mean-real-time throughput and validated 1-to-4-thread
scaling for this reconstruction only. The one-thread p95 and both maxima cross
the frame budget; the four-thread p95 is `14.396602 ms` and remains below it.
Because both maxima cross the budget, this is not an every-step deadline
guarantee. It is also not the
paper's 100-contact author scene, float32 Warp/Newton kernel, Apple-silicon
host, impact sequence, or timing boundary; every paper timing verdict remains
null.

Schema v8 preserves the default 83-column trace. Its newline-terminated header
SHA-256 is
`396c866d782e626b5c5b52e74c392de1175f5e46dfd90b809d1f219417092e50`;
the colored contract uses a separate 95-column trace whose newline-terminated
header SHA-256 is
`424195336cb42753179130c9f6fcba3a8ddea9bf669cfa70c0f77fcb4c6335a5`.
The opt-in Native manifold-sensitivity selector uses a separate 94-column
trace whose newline-terminated header SHA-256 is
`007311fb28062377dd6a0d26cad1ab4f7e2c99f359afd33554651f3cc0929ef5`.
Bundle integrity is pinned by these SHA-256 values:

- `artifact-index.json`: `a60899cc12a53f03424c02c2647f233d5c75f3ccce367ea2a604f9a7ee18bf11`;
- `metadata.json`: `5507ed80140a146d4247c4f0b05fd9503879ce79856189a15759b101c2cab789`;
- `invocations.json`: `719ba3491fad8ac12aa290faa401d0c46b10af52da7eb7a28487a5b2aac44812`;
- `raw.csv`: `91a379f832ca52bbce7011308640012d9f199b98e39cba4eaf661cf17fb0f017`;
- `summary.csv`: `3c08b251c340aa9ae7909d715ebcc2985ba34350a6e03b0b6b351d82d1ec82a2`;
- `summary.json`: `304736d6b871c4498a6c0de4c4448635e712fb3a8a455dd54d9ffefeef2ec170`;
- `REPORT.md`: `4e55d7f4dc0532ab15b86cfeb72ea9526d3f9b63194dc5a25f3974186b1a7ba7`;
  and
- trace binary: `0923bf7df1eaa518f9a2ffacd0a42fd7330f5cbf6c35cb5749fbc122db1310ff`.

## Current-Source Strict Small CPU Evidence

The repository-finalized current-source small `paper_cpu` artifact is
`assets/dart_cpu_evidence/2026-07-19_current_source_paper_cpu_small_r7/`.
It contains three repetitions of all nine reconstructed small-scenario
trajectories on pinned physical CPU 4. Its 60-file exact-membership index binds
the runner, trace source, executable, resolved runtime closure, raw rows,
affinity records, summaries, and the post-run identity recheck.
Core r7 SHA-256 values are report
`008bc94667893cd26bfc04a720caca3d3a5703601c8739bc06856f93818c63d5`,
artifact index
`06594c1e1cc3c6858bc78a80630aefb0e85eeef92204b0a09b99425411c3ebeb`,
metadata
`e227d7aef3b273ff81385f227f9e6766a7e40a622bc4298e1807b7c2068bc417`,
summary JSON
`9137f1b2db0909a96897632a66617f2cfa58476a369d2adfe232b73e46cd0fa2`,
and raw CSV
`ba062cf359da85d21b5ea83b722d26375267212dfc44a4ce9f48701ad8a79a5a`.

The retained prior-source full-card negative remains separately archived at
`assets/dart_cpu_evidence/2026-07-12_prior_source_paper_cpu_card600_negative/`;
its metadata retains its original `/tmp` output path.

All paper timing verdicts are null. The pinned public reference now exposes the
current Warp/Newton implementation, scene configurations, local kernel, and
baseline runners, but DART does not reproduce that float32 pipeline. The exact
historical Apple-silicon host and paper invocation/timing/warmup attestation
also remain unavailable. Current author invocations were independently run and
preserved, but do not fill the historical gap.

All nine small trajectories pass their reconstructed physical classifiers.
Seven satisfy the strict solver and local real-time evidence contracts; two do
not:

| Scenario | Solver result | Physical outcome | Real-time claim |
| --- | --- | --- | --- |
| Backspin | Residual `<=1e-6` | Pass | Locally valid |
| Incline, `mu=0.4` | Residual `<=1e-6` | Pass | Locally valid |
| Incline, `mu=0.5` | Max residual `1.43920815e-6`; three accepted caps | Pass: `8.634e-7 m` displacement | Invalid: residual above tolerance |
| Painleve, `mu=0.5` | Residual `<=1e-6` | Pass | Locally valid |
| Painleve, `mu=0.55` | Residual `<=1e-6` | Pass | Locally valid |
| Turntable, `mu=0.2`, `omega=2` | Residual `<=1e-6` | Pass | Locally valid |
| Turntable, `mu=0.2`, `omega=5` | Residual `<=1e-6` | Pass | Locally valid |
| Turntable, `mu=0.5`, `omega=2` | Residual `<=1e-6` | Pass: retained-on-support classifier over the measured horizon | Locally valid |
| Turntable, `mu=0.5`, `omega=5` | Max residual `3.05038653e-4`; three failed processes | Classifier pass: ejected | Invalid: solver contract |

“Locally valid” means only that the reconstructed DART trajectory satisfied
the solver, affinity, physical-outcome, and `1/60 s` gates on this host. It is
not a paper comparison.

The prior-source full-card process stops at step 89 of 600. The terminal failed solve has
residual `1.8612e-2`, one exact-solve failure, and zero boxed-LCP fallbacks.
The attempted prefix contains 88 accepted 200-iteration caps and one
`fbf_failed` step. Its raw mean/p95/max step times are
`59.8234/74.3458/79.5385 ms`, and its real-time factor is `0.2786`. Because
the requested trajectory is incomplete and the solver contract fails, none of
those raw times is a real-time or paper-performance result.

The coupled full-card system is strongly rank deficient. At 90 contacts it
has 270 contact rows for 156 generalized velocities, so the contact response
has nullity at least 114. At 155 contacts it has 465 rows, so the lower bound
rises to 309. This supports the redundant-contact/churn diagnosis; it does not
excuse or relabel the failed solve.

The retained rows further show that primal feasibility peaks at only
`1.70e-16`, while dual or complementarity residuals dominate every row. Gamma
always equals its safe bound, shrink iterations are zero, contact count changes
34 times, pair count changes 31 times, mean warm-start match is `0.7452`, and
persistent gamma is never reused. The terminal 183-iteration `fbf_failed` is
not an ordinary 200-iteration cap. An inner frozen-cone failure is the leading
code-path inference, but this prior-source artifact predates the current
internal-status/best-iterate columns and therefore cannot confirm that subtype.

## Current-Source Card Manifold Sensitivity

The provenance-complete v2 bundle is
`assets/paper_evidence/card_house_26_manifold_sensitivity_v2_r3/`. It is a frozen
one-factor comparison of Native `Compact` and `FourPointPlanar` manifolds on
the reconstructed 26-card 600-step settle/projectile command. It is not a
paper, timing, real-time, author-scene, or physical-outcome comparison.

Both modes emit all 600 rows with zero exact failures and zero boxed-LCP
fallbacks, but neither trajectory is strict and neither has a strict-success
row. Compact returns zero, records 3,495 accepted capped groups across 5,757
exact attempts, and ends with a successful last group at residual
`8.525678738415048e-7`. FourPointPlanar records 682 capped groups across 745
attempts and returns the preregistered terminal-convergence-gate failure at
residual `0.016582575623909489`.

The directional contact hypothesis is supported: FourPointPlanar raises mean
contacts by `93.7983333333` and mean per-pair multiplicity by
`1.9548548971`. Its 2,813 fewer capped groups do not establish strict
convergence improvement because its terminal gate fails. Raw wall time is
excluded from the verdict, and physical and timing verdicts remain null.
The unsuffixed v2 path is historical current-at-capture evidence; v2_r3 is the
current-source rebaseline with unchanged semantics. Its
report/index/metadata/invocation/tree hashes are
`0429b12f90e4fef534c9b707324a180eaac78829a3b3667505111228aba58d2e`,
`1703f995bd5d2c6edeb12ba936010a80609c3e952f2c782b554bd0be7ca40627`,
`5890ab138179f4d7aaee6cd04c63799086439bae58fbde4f994048785dd0b8ac`,
`6e48eb835ab9aafca0b8d4bd29acb5ec932645fb37ca6124ce3c23b3265847f0`,
and `953d8efd4d43ee2a74cede6b5e3a0a766ecd1fd8914485b50b340f515b7ecb77`.

A separate raw `dart_best` one-thread/eight-thread matrix exists at
`/tmp/fbf_cpu_dart_best_postreview_20260712_t1_t8_r3`. Its legacy exact solve
for one coupled island is serial, so requested world threads alone do not
establish parallel exact-solver work and that matrix still supports no
multicore claim. The durable literal-wedge bundle above is different: its
explicitly non-paper colored-inner-BGS contract records four-worker dispatch
and per-phase residency on CPUs 8, 10, 12, and 14, then validates the matched
1-to-4-thread scaling pair.

## Exact Local Solver And Integration

The branch provides:

- normal-first primal/dual Coulomb-cone operations and the dimensionless
  primal/dual/complementarity residual;
- matrix-free contact response and diagonal-block support;
- the FBF outer iteration and fixed-scale residual diagnostics;
- an exact H-metric 3x3 cone-QP local solve;
- KKT-certified projected-gradient recovery for difficult finite local
  systems;
- an opt-in exact constraint-solver route with warm starts, split impulse,
  failure isolation, and explicit fallback diagnostics;
- an opt-in deterministic manifold-colored exact inner BGS with separate
  dispatch and CPU-residency diagnostics; and
- trace, residual-history, gamma-sweep, benchmark, CPU-evidence, and visual
  evidence tooling.

The existing boxed-LCP solver remains DART's default. Exact FBF is opt-in.
With boxed fallback disabled, a failed group is deliberately left unsolved;
callers must stop rather than advance and interpret the state as valid.

Current focused gates are:

- exact math: 47/47;
- exact constraint-solver tests: 25/25;
- `ConstraintSolver` integration: 64/64;
- Native collision: 42/42;
- masonry-arch geometry: 3/3;
- paper fixtures: 19 passing, 3 explicit stress cases skipped;
- focused Release and Debug CTest matrices: 9/9 in each configuration;
- author-incline reference finalizer unit tests: 64/64; verify-only reports 37
  indexed artifacts and 39 physical files;
- focused manifest/backspin/incline/author-masonry/author-incline evidence
  suite: 859 passed post-merge in 163.95 s;
- full no-cache dartpy Python suite: 1,555 passed in 165.09 s; and
- colored dispatch/barrier stress: 1,000 consecutive successful runs.

The retained strict full-card failure above is scientific negative evidence,
not a green paper-parity gate.

The manifest validator now hashes repository artifacts, materializes the CPU,
standing-visual, and prior-source indexes, cross-binds recorded runtime
identities without requiring ignored local build products, and recomputes the
CPU timing/scaling/physical antecedents from all 3,600 measured raw rows. It
also binds every warmup/measured process result, taskset command, per-CPU
topology aggregate, worker-residency identity, and archived prior-source
semantic/provenance claim; cross-checks the card-v2_r3, impact-v8, and
arch101-v7 truth fields; and rejects fabricated paper comparability,
all-step real-time,
strict-card, physical, timing, or positive-outcome promotions.

## Pinned-Author Incline Sweep Reference

The numeric packet at
`assets/paper_evidence/author_incline_sweep_reference_v1/` pins author commit
`b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0` and preserves independent FBF,
MuJoCo, and Kamino CPU runs. Each lane uses the exact grid
`mu=.3,.4,.45,.5,.55,.6,.8`, with 120 steps per cell: seven cells and 840
rows per lane. The retained FBF histories record four contacts per FBF step;
the MuJoCo and Kamino result records contain no contact-count field.

FBF records 839/840 configured convergence flags. The `mu=.55` cell is
119/120 because step 1 reaches the 200-outer cap. Of the 839 true flags, 235
use the initial natural-residual shortcut and 604 use the configured outer
nonnegative `coulomb_rel < 1e-6` gate. Natural `final_residual` is not that
gate: 456 configured-true rows are at or below `1e-6`, 383 configured-true
rows are above it, and the configured-false row has natural residual
`3.273267262002487e-8` while terminal
`r_coulomb=1.5311460572898186e-6`.

The normalized displacements are:

| `mu` | FBF | MuJoCo | Kamino |
| ---: | ---: | ---: | ---: |
| `.3` | `3.5392831695743054` | `3.5794330878127263` | `3.5392822099580354` |
| `.4` | `1.7698922978656797` | `2.1118162364510042` | `1.7698918846975635` |
| `.45` | `0.8851976117115778` | `1.8350010889171866` | `0.8851977916396285` |
| `.5` | `0.0005018926371855115` | `0.35929869745695187` | `0.0005009063649080669` |
| `.55` | `0.0004281487924409106` | `1.8135318873257669` | `0.0004281487924409106` |
| `.6` | `0.00035627086822120473` | `1.2524510782262557` | `0.0003563241802362017` |
| `.8` | `0.00006883913936487685` | `0.18109838262409816` | `0.00006887912337612457` |

The close FBF/Kamino projection and nonmonotone MuJoCo projection describe
only these current runs; they are not full-state or cross-solver parity.
First-use JIT work, always-on history collection, ineffective warmup
exclusion, and lane-dependent timer boundaries make every source timing field
diagnostic-only. Fig. 1, Fig. 2, and video.03 therefore remain partial.

Verify the numeric packet without rewriting it:

```bash
python3 scripts/finalize_fbf_author_incline_reference.py --verify-only
```

## Visual Evidence

The source audit pins the combined 82 s video, teaser, and paper and validates
all nine contiguous source segments. The SHA-256 values are:

- video: `d5356e1b31487be62b75af05efbfecdb70ad5d98501a8efd378fcedf066e4794`;
- teaser: `99527da7a84f7b9ac0031f794d9b16adadfba846d2165e7da22fd51d986c8db0`;
- paper: `af7cb8df58288f4323fa4340e1590b09643b8702116a6c47cefe7ffa9a51e2f4`.

The post-review small-scene capture matrix at
`/tmp/fbf_visual_evidence_postreview_20260712` contains eight dynamic schedules and
one static 10-level construction frame. Every completed dynamic step and every
solver group reports residual `<=1e-6`, with zero accepted caps, exact
failures, or fallbacks. The synchronized outputs are:

- turntable: 1320x1060, 30 fps, 121 frames, SHA-256
  `e5bf08986d6e70bdf25797e66c958c9ac947f9b2dc863b30d37afe1efd29f11e`;
- Painleve: 1320x530, 30 fps, 76 frames, SHA-256
  `d25a93abf964df707e3c10c30202bf75d76e0b216b9f2301b393dca63322afd0`.

Manual inspection found the historical reconstructed incline stick/slide presentation,
backspin reversal, three turntable ejections plus the `mu=0.5, omega=2`
retained-on-support presentation, and the two Painleve upright/tumble
presentations.
These are qualitative reconstruction observations. They do not override the
quantitative small-CPU failures above and are not author-golden comparisons.

These historical matrix captures were produced after the recorded
solver/validator review fixes and manually inspected. Except for the incline,
Painleve, and backspin subsets independently finalized below, they remain reconstructed
`/tmp` evidence, not repository-published media or paper-golden comparisons.

The incline subset has now been independently recaptured and finalized at
`assets/paper_evidence/fig01_02_incline_current_v1`. Finalization and
clean-checkout verify-only pass with status
`valid_current_source_nonpaper_incline`. The directory has 23 physical files
and an exact-membership index of 21 artifacts. The combined 660x506 capture
retains five durable stills and a 61-frame decoded 30 fps clip schedule. Its
70-file raw capture staging directory is pruned after sealing, so verify-only
needs no ignored staging. It records 240 exact attempts/solves, zero caps,
failures, or fallbacks,
maximum residual `9.999836962261359e-7`, and eight contacts per post-initial
step.

The independent `mu=.4` and `mu=.5` traces each contain 121 rows, 120 exact
solves, 119 warm starts, zero fallbacks, three contacts per post-initial step,
and continuous post-initial tracked contact. At `mu=.4`, downhill displacement
is `1.7686892884927794 m` versus analytical `1.7548661487418349 m`, final
downhill speed is `1.7544655347780056 m/s`, and maximum residual is
`9.986952135669881e-7`. At `mu=.5`, downhill displacement is
`0.0008905412965980523 m`, maximum/final stick speed is
`0.001116442058867632 m/s`, and maximum residual is
`9.997210606407098e-7`. The displacement separation is
`1.7677987471961814 m`.

Only aggregate `step`/exact-solve/fallback projections are byte-identical, at
SHA-256
`f03dff0aaec5f0fa6615609c2ea97aa31f072c14c6212fffe15858ea969d88c2`.
The capture reports 8 contacts while the traces report 6 in aggregate;
`contact_count_match=false` and `contact_counts_compared=false`. Different
placements also prohibit state, residual, status, warm-start, per-cell, or
full-trace equivalence claims.

Core finalized SHA-256 values are metadata
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
The finalizer/test, runner/test, demo source/binary, trace source/binary,
fixture source, and libdart are bound at
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

This does not erase the separate strict `paper_cpu`/Native `mu=.5` negative:
its physical displacement is `8.63436433e-7 m`, but one accepted cap per
repetition raises maximum residual to `1.4392081500753078e-6`, so strict
solver/local-real-time remains failed. `fig.01`, `fig.02`, and `video.03`
remain `partial` without the full sweep/plot, matched external rows, approved
source golden/diff, paper contact-count match, full 11 s semantic edit, paper
timing, and real-time parity.

The Painleve subset has since been independently recaptured and finalized at
`assets/paper_evidence/fig05_painleve_proxy_current_v1`. Its repository-published
paired clip is 1320x530 at 30 fps for 76 frames with SHA-256
`dd4cdda2410b71f8e86035f1c0ff278f9dda77133c702e409ee5d533da443a4b`.
Both exact-FBF proxy member clips, hardened sidecars, manual inspection, and
separate 151-row physical traces are bound in a 27-artifact index within a
29-file physical directory. Raw capture frames are pruned after sealing, and
clean-checkout verify-only needs no ignored staging. This proves
only the local proxy presentation: `mu=.5` returns upright, while `mu=.55`
crosses the tracked fixture's tumble threshold after 0.03836187995520213 m less
pre-tumble travel and remains visually horizontal. The rendered demos and
tracked fixtures are separate implementations, so this is not trace
equivalence, author-scene parity, external-solver parity, an approved source
golden/diff, paper timing, or real-time evidence.

The backspin subset has also been independently recaptured and finalized at
`assets/paper_evidence/fig03_backspin_current_v3`. Its exact-membership index
binds 18 artifacts within a 20-file physical directory. The MP4/GIF preserve
the full motion schedule, three durable stills retain steps 0, 1, and 2, and
the 140-file raw capture staging directory is pruned after sealing.
Clean-checkout verify-only needs no ignored staging. The capture records 129
exact attempts/solves with zero accepted caps, exact failures, or boxed-LCP
fallbacks and maximum residual `9.96497154974839e-7`; the separate trace
maximum is `9.964971544991853e-7`. That trace reaches maximum
`x=1.5959314363310166` at step 48, first records negative `vx` at step 49,
and ends at `x=-2.9362508912363654`, `vx=-6.628158971623909`. Step 120 is
the sole contact-free post-initial step.

The renderer applies a high-contrast 6x4 ivory/charcoal checker texture with
one coral registration tile through a visual-only UV `MeshShape` under
`VisualAspect`. The physical `SphereShape` remains unchanged and continues to
own collision, dynamics, inertia, and friction.

The 131-row trace and capture solver/contact projections are byte-identical
at SHA-256
`973d544311bac3b5927cc73b335b1a375d0339403a2f713707bb928076aa2b22`.
Finalized SHA-256 values are metadata
`a42ce0521a7c2af31662eff6a000ef5e68fe8bddf631d4f323be1ea8230c25a7`,
index `429a0888fa7a002cbc0c93e708569e4bf8e18195421c9663d5ce3b3a1968ab7f`,
manual inspection `b86c596da631503a3831fd55adeb934505cf758dfc8a612d0a0f32b2a55067cb`,
trace summary `0f6221fd32742b849e9ac79ec750de71b46ec24ba66b8d6e5a0e25048223ab48`,
verification `fb9de609e52df648465dc0dbe73af7fbbd1b98c1c4671cde504200ff72c15c01`,
trace `dc205297fa4cbffa1b497f12507b919f344bbee45a5820ae46145e0ed91bbd98`,
panel `72bf8d6098e8fb17b98bbedeaa00cc539866cf570d91d725f77dc75f1971b067`,
MP4 `7d4606f4da0a57ffbdfa0528906b21a20d7e1a4e47a6e7eb5387242aecc71928`,
GIF `773365f624ba1326855f2ad99c0196f761ab776001da568f7cdaa84054adacc8`,
and report `f9178c14c1930361afc1d97cb5bf08afe04d3fd451e8a94144b02bb0b46e61cf`.
Manual inspection passes checker-texture and coral-registration-tile
legibility only. At `-200 rad/s`, 30/15 fps media can alias and do not prove
signed angular direction. The contact-free step rules out continuous contact;
rest and an airborne landing phase are unproven. Separate demo and CSV scenes
are not full-state trace-equivalent. No external-solver, paper,
approved-golden, timing, or real-time parity follows. Both `fig.03` and
`video.02_backspin` remain `partial`.

### Finalized Author-Pinned Fig. 4 Turntable Bundle

The finalized repository bundle is
`assets/paper_evidence/fig04_turntable_author_current_v1/`. It pins author
commit `b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0`, binds 58 indexed artifacts
within a 60-file physical directory, and preserves source order
`mu=.2, omega=2`, `mu=.2, omega=5`,
`mu=.5, omega=2`, `mu=.5, omega=5`.

The current `dart_best`/Native `FourPointPlanar` visual lane completes all four
360-step cells with valid solver contracts and no fallbacks. The finite-horizon
outcomes are ejected, ejected, retained on support through 6 s, ejected.
Retention does not prove zero slip, perfect sticking, co-rotation, or behavior
beyond 6 s. Manual inspection passes the segmented visual disc, one coral
registration wedge, labels, and ordering; the physical cylinder remains the
sole collision/dynamics geometry.
Four durable timeline-selected outcome stills bind steps 136, 120, 360, and
90 in source order. Capture staging is pruned after sealing; clean-checkout
verify-only needs no ignored files.

For every cell, capture and trace projections are byte-identical over the six
fields `step`, `contacts`, `exact_solves`, `warm_starts`,
`boxed_lcp_fallbacks`, and `status`; this is not full-state equivalence. The
separate strict `paper_cpu_native` lane has no capture comparison and cannot be
substituted across lanes. Its `mu=.5, omega=2` row fails at step 40 with
residual `7.407835021099202e-6`; its other three rows pass.

The bundle status is
`valid_author_source_pinned_nonpaper_turntable_matrix`, with current visual
artifact, solver, physical-outcome, manual-inspection, and pass gates true.
Core SHA-256 values are report
`930cc12b95ab78c6e61d084064b584f5872633f2432e434c68d071818cec7fb1`,
index `209b677ce35e2b9c11248ab414727016394831084881a5e9c2866f68b51f6cdf`,
metadata `854ef0c8aad75b4b200f512951d56b4dbef7aa82c46cc5e82123f0583dcc6ac5`,
verification `455da7686eaeeb989e0d122c4724ea66ff161e8d652230eff9fb8ad20590bacd`,
and group clip
`b241463658e6b48dfb2c74815e85317c2f0eccd46a7f7ab978b8f2701ce80d6d`.
Author-spec/visual-OBJ/visual-MTL/manual-inspection hashes are
`1680cd8351fa62937c0318826f7abc75917234cb3888f983acce06f13698bc6c`,
`bc86f1ef1f5fae1510f23b1586ae20efe788c499373370a66af81b06818f1b14`,
`619352b9ac14e89a4d467dde867019e0d01540b6f11852df565f23fb26a01752`,
and `095652d3df70a144be31be49b0e25b3265df54a3815df21742333d5fdfb4529a`.
Current visual runner/test bindings are
`d848afa53caf14b9fb3ea061d658eef274e8d917151937bd6340283b79ab5432`
and `6e378252fa6a7cb51c6813c9d5a2b30b8c8129eacdb01df0ee19a58a270cbc5e`.
This is author-source-pinned finite-horizon non-paper evidence, not paper,
approved-golden, timing, or real-time parity.

### Finalized Author Card-House Construction Bundle

The finalized repository bundle is
`assets/paper_evidence/card_house_author_5_construction_current_v1/`. It has
12 indexed artifacts / 14 physical files and shows the public-author default
five-level, 40-card construction with four suspended cubes at step zero.
Index, metadata, and manual-inspection SHA-256 values are
`d6cbc6f9600b8bc5c3094dd85974eae8e71d64a9e3d6e99c1783ace36be9741d`,
`b97ac795c9368f2632fe422f975914f412e8d1cb0667023e8d83c6224547df00`,
and `7bc672e9dd95b52853c5c7e56680190d564fc9514add9b263de0c33c3f94e2a4`.

This is construction-only evidence. It executes zero simulation substeps and
does not prove release, standing, trajectory, solver behavior, contact
dynamics, physical outcome, the historical four-level/26-card trajectory,
Fig. 6 or video parity, timing, performance, or paper parity.

### Pinned-Author Masonry-Arch Scientific Negative

The sealed bundle is
`assets/paper_evidence/author_masonry_arch_reference_v1/`. It records a
500-frame, four-substep-per-frame author invocation that releases three cubes
at frame 400 / substep 1,600. The source default is 400 frames with release at
frame 400 and therefore never releases the cubes; this is a newly declared
current-source diagnostic, not a historical or paper invocation.

A deterministic projection represents every one of the 2,000 substeps and is
lossless with respect to the declared claim fields. The 382,753,953-byte raw
source history (SHA-256 `cec0e4b86837e7542c498c7ddad40538983ec023332b88ebddee7766997e3ac1`)
is size/hash-bound but omitted. In the projection, 40 author convergence flags
are true through the initial natural-residual shortcut, 117 are true through
the configured outer nonnegative `coulomb_rel < 1e-6` gate, and 1,843 outer
solves are false. The projected `final_residual` is the separate natural
residual, with only 47 values at or below `1e-6`. Release substep 1,600 is
nonconverged with 100 contacts and natural residual
`0.017456069692858667`; final substep 1,999 is nonconverged with 108 contacts
and residual `0.5161195175386001`. The projection has no contact-pair identities,
so a later count increase is not definitive cube-arch contact evidence.

Exit zero and valid bundle integrity prove preservation only. The companion
DART spec records configuration but executes no dynamics and does not
implement the source collision/contact-gap/backend/float32 semantics. This
artifact establishes no all-substep solver success, DART or cross-solver
dynamics/trajectory/outcome equivalence, Fig. 7/video.07 parity, timing,
repeatability, or media/golden evidence.

The selected nine retained schedules and their exact group outputs were also
reverified successfully against that matrix's recorded `dart-demos` binary
(`6ac1b6fb167bdcdbfbf2fea831eac7755a751ed72bb6f1d55e4f80a4d4e25165`).
The `all-runnable` verifier still fails closed because the long
`card_house_26` timeline is absent; this is confirmation of partial state, not
completion evidence.

Each 120 s bounded long-scene attempt failed to produce valid sidecars or
media:

| Schedule | Furthest observation | Evidence verdict |
| --- | --- | --- |
| Full 26-card sequence | Step 6/600 | Incomplete; no valid artifact |
| 25-stone oriented-box arch | Step 24/360; visibly collapsed at 0.4 s | Physical mismatch; no valid artifact |
| 101-stone arch | Step 120/600; visibly collapsed at 2 s | Physical mismatch; no valid artifact |
| Dynamic 10-level card house | No completed step 1 | No valid artifact |

The repository literal-wedge standing bundle at
`assets/paper_evidence/fig07_arch25_literal/` is finalized as valid
current-source, non-paper reconstruction evidence. It contains five durable
1280x720 stills, a fully decoded 61-frame H.264 schedule, and a timeline. Its
independent capture and trace paths compare all 600 rows with zero differences
in the nine mapped integer and five mapped floating-point fields, alongside
fixed reference/scene-contract checks. The five durable stills, timeline, and
a separately decoded video midpoint were manually inspected. The retained
pending hash DAG passed before final writes, and the final index covers 19
artifacts in a 21-file physical directory. The 70-file raw capture staging
directory is pruned after sealing, and clean-checkout verify-only needs no
ignored files.

The video samples 10 simulated seconds into 6.1 seconds at 10 fps, or
`1.639344x` time-lapse; it is not real-time playback evidence. Integrity is
pinned by:

- `metadata.json`: `b217b0a43200ca558f09776e464d50f0f9dc5c5a175bcfa66ae1b656076871d1`;
- `provenance.json`: `eca349842e4121584145cd039a554aa13c51e5242ff924b37f5f49a9dee0ac2f`;
- `artifact-index.json`: `4d173c78e2c6584d6d8a85867160cc0c0687fe0f35e6878f66b0cfcd7c1c15ee`;
- `manual-inspection.json`: `4b52bcd26ed88d184bd695d77070420aeec2c95edfb203efda7ae6241150c343`;
- `pending-metadata.json`: `e300103dbb950b97e217ca0bea6f1c1dc78598ddf485cdaca26cc7ad58835b3c`;
- video: `e0c111f96511bbda41af145787bd9f68782b8c9e8a909d45eb03d3c3d647feb1`;
- timeline: `926380c5c5768cc9ca1e414cbfe2631584daa99387090926d5a1f035d62b6ec9`;
  and
- decoded midpoint: `75a88bb317441ed71803f784b2eaa099211c4e026f8547d6fe5f2ff3ee95909f`.

The durable capture identity is immutable: capture source
`c3efeac52d02a0c373f733598db81e545d062195ba6e96c2a65bcb607cd0207f`,
capture binary
`8b3cad15220c8fdb69c3ebdf7fa3923fda6fd812a49d0e54c8aeb07e62f0a7e9`,
and media hashes are unchanged. Fresh revalidation against the final current
trace and Native source binds trace source and binary
`b00eea0c87f75f17259fac433bafd98c20a961d14389a80e91742d3c8a678f76`
and
`0923bf7df1eaa518f9a2ffacd0a42fd7330f5cbf6c35cb5749fbc122db1310ff`.
The finalizer driver is
`0f4e27b0c58e9dd3774c6be48ad4c70a857e2956fd81f11710837561f08f7243`;
its test source is
`f06ba295006cf7e1f0fb692fc5eacf62e4e7f3be89bd166c041752d915779bdd`.
The finalizer regenerated reference trace
`0e41af230fb4c27143966c3f8522702280e6746b31e84155a9a047326e46dbb3`
from the current binary and again revalidated all 600 rows with zero
differences.

The separately preregistered crown-impact v1 run is retained at
`assets/paper_evidence/fig07_arch25_literal_impact_v1_negative_final_v9/` as a valid
scientific negative. The runner
`scripts/run_fbf_literal_crown_impact_negative.py` completes 720 steps; its
projectile-free 600-step standing prefix matches all 88 eligible fields. First
projectile-arch contact is step 607, before ground contact at step 616. All
bodies remain finite with zero exact failures or fallbacks, but five accepted
iteration caps, worst residual `9.1545317042653963e-5`, final all-arch
displacement `0.07093964431215687 m > 0.07 m`, and far-field displacement
`0.060523747030465196 m > 0.007 m` make the impact claim false. No parameter or
threshold was tuned after the run.

The current runner independently recomputes every gate from finite metrics, requires
post-launch exact-solve progress, pins the normalized fingerprint and frozen
preregistration-contract hash, and stores a separately rerun 600-step standing
reference with 88 exact field matches per row.

Its final runner SHA-256 is
`622b388142fed881191dcb6efb103266eb2ea1e3ffbadaf402f63b89c558bb67`.
Bundle SHA-256 values are raw
`42cc94f5f111442da42b03af33722f713d66b853d3ac21f21083f28ba24e7ba4`,
stderr `7964b07fd5396f10013ff8d9100d2b36e7dfc151764210d8c60bd0ae853a2d94`,
summary `e3baa479111a999d03359a0642533cad1baa3a48c415609bafb9a7a7cfa18e7c`,
metadata `0bd19797953a10223bbd7183f6e8002e490b21c29b2304befee7e6618ff9ee73`,
and report `c5711b0fe06e679f889f6a7ca3812aa955c3b8c61270ef038def06b2a3f173ff`.
The v6, v7, and v8 paths are historical current-at-capture evidence; v9
preserves the same frozen negative semantics and records the executed
`taskset` identity in its runtime closure.
The normalized trace fingerprint is
`86b37ec37d28259514453c9ecc9e7d5f12afe08118a0b04abd40e72acd147384`.
The frozen preregistration-contract SHA-256 is
`4cdea674f366fc2d18eadf11ef4333d491786d5d85e3fc16fa611ea7dede3f37`;
the standing-reference CSV SHA-256 is
`22fabfe2e8f35bf329f32873845a60599ed2df171c15728f14311b91d888c387`.

The standing bundle validates only the no-projectile reconstruction. The CPU
bundle remains the timing owner, and the impact negative supplies neither
passing crown-impact media/outcome nor paper parity.

The separately frozen 101-stone literal protocol is
`LITERAL_ARCH_101_V1.md`; its current provenance-bound bundle is
`assets/paper_evidence/fig08_arch101_literal_v1_negative_final_v7/`. It promotes
the exact-inertia 101-wedge reconstruction from collision-only scaffolding to
a fail-closed dynamics test, but the result is negative on step 1: 5,000
outers, `fbf_failed`, residual `0.78153646143524735`, one exact failure, and
zero fallbacks. The dynamic `FourPointPlanar` trace reports aggregate fields
of 400 contacts, 100 constraint pairs, three colors, width 34, and four-P-core
colored execution. The separate collision-only Compact repeat-2 probe proves
only the constructed time-zero graph: 102 pairs are 100 adjacent-stone pairs
plus two springer-ground pairs. The v7 one-step FourPointPlanar companion
resolves the failed step-1 pre-solve graph as exactly the 100-edge
adjacent-stone chain: 100 unique adjacent pairs, 400 contacts, multiplicity
four, zero non-adjacent pairs, and zero ground pairs. Its aggregates and
residual match the frozen trace. The companion accepts the capped iterate and
does not follow the frozen trace participant-affinity contract, so
solver-taxonomy and affinity equivalence remain false. This narrow identity
result supplies no source equivalence, valid trajectory, standing/physical
outcome, timing, media, long-run behavior, or paper parity. The runner sets
`artifact_valid=false`, `standing_claim_passed=false`, and
`timing_evidence_eligible=false`; no trajectory, media, or timing is promoted.
It binds and rechecks the protocol, runner, trace/collision/dynamics-probe
sources and executables, `taskset`, `ldd`, and all resolved regular
shared-library files. That is source/executable/shared-library-bound
provenance, not a claim over all host runtime state. Its normalized fingerprint
is
`8d275edcaa82a2f628fdd5d9f846b2daed7e6864fdc4b7e80eca9a8c93bcf527`.
Runner/raw/summary/metadata/report hashes are
`7155b9bc6082e79aca317be6626fea587b58538df2f34e94f865cd54c15eb993`,
`fc0705ff497b44658dfe186ed27c50b2ab9d19a268f20cf50875a3155d1dc07d`,
`2cae961048b776c069caeccda2d95f2f0fd0969cae9e3de3782f0e5e5b7b640d`,
`770d662f7021320982700fe699266bc94d2448679c8c4dc60bd127c660facb8a`,
and `1ba3987af896f68401faa1153f6b388aed38b087214d07592f5a060cab429e2a`.
The current v7 whole-tree hash is
`e65107a6d2e2880742e02d01949bc4b3f9adb2f48b6be552a720fa83c9bcc4b3`.
The prior `fig08_arch101_literal_v1_negative_final/` bundle is retained only as
invalid historical evidence because it lacked the current shared-library and
independent graph bindings.
The v2 bundle is provenance-complete historical evidence, but a
clang-format-only collision-probe source identity change superseded it for
current-source claims. Additive card-manifold trace instrumentation later
superseded v3, and v4 is historical current-at-capture evidence. The unchanged
command was rebaselined as v5 and again as v6 after the current-build libdart
identity advanced. V7 adds the identity-resolved one-step dynamics companion;
the frozen trace and scientific result remain unchanged.

## Evidence Vocabulary

Reports and manifests keep these predicates separate:

- `artifact_valid`: bytes, schema, hashes, ordering, dimensions, frame count,
  and full decode are valid;
- `solver_contract_valid`: every completed requested step and every solver
  group satisfies the declared status, residual, cap, exact-failure, and
  fallback contract;
- `physical_outcome_valid`: a separate scenario-specific quantitative outcome
  check passes;
- `manual_inspected`: the rendering was actually viewed and the observation
  was recorded; and
- `claim_valid`: all predicates required for the particular claim, including
  source/build identity and comparability, pass.

No one predicate implies another. In particular, valid media plus manual
inspection cannot turn a failed physical trace into a valid claim.

## Arch Geometry And Contact Truth

Production fixture, trace, benchmark, and demo arches use source-derived
weighted-catenary placement with oriented `BoxShape` stones. They are not
literal tapered voussoirs or an author-faithful dynamic wedge port. A separate
trace scenario now exercises literal exact-inertia wedges without changing
those production box scenes.

| Surface | Geometry | Observed contact contract | Allowed claim |
| --- | --- | --- | --- |
| 25-stone production dynamics | Oriented boxes | Natural manifold 96; GUI reduced profile 48 | Reconstructed DART box arch |
| 101-stone production dynamics | Oriented boxes | Full profile reaches cap 512; GUI reduced profile 38 | Reconstructed capped DART box arch |
| 25-stone collision audit | Literal prisms/wedges with exact uniform-prism inertia | 24 nominal adjacent pairs or 26 with closure/ground | Collision/inertia audit only |
| 25-stone literal dynamics | Exact-inertia literal wedges, `1 um` closure, Native `FourPointPlanar` | 96 contacts, 24 pairs/manifolds, 3 colors, width 8 for all measured steps | Non-paper 600-step standing trajectory, validated standing media, and matched 1-to-4-thread scaling only |
| Literal crown-impact v1 | Standing contract plus three frozen projectiles; numeric only | Arch contact step 607 before ground step 616; finite, but 5 caps and residual/outcome gates fail | Preregistered non-paper scientific negative only |
| Pinned-author 25-stone diagnostic | Author meshes and Warp/Newton float32 pipeline; 500 frames / 2,000 substeps | 100 contacts at release; 40 initial-shortcut and 117 configured-outer-gate convergences, plus 1,843 nonconverged outer solves; no pair identities | Current-source scientific negative only; no DART dynamics, trajectory, outcome, timing, repeatability, media, or paper-invocation parity |
| Literal 101-stone v1 | Exact-inertia wedges, `1 um` closure; Compact time-zero probe, Native `FourPointPlanar` trace, and v7 one-step identity companion; numeric only | Compact repeat-2 time zero has 100 adjacent plus 2 springer-ground pairs; the failed step-1 FourPointPlanar graph is identity-resolved as exactly 100 adjacent pairs/400 contacts with multiplicity 4, zero non-adjacent or ground pairs, but solver-taxonomy/affinity equivalence fail and the frozen solve fails after 5,000 outers at residual `0.7815364614` | Frozen non-paper failed-prefix scientific negative; no source-equivalence, standing, physical, timing, media, long-run, or parity claim |
| Card-house manifold sensitivity v2 | Reconstructed 26-card command with Native `Compact` or `FourPointPlanar`; numeric only | Both emit 600 rows but are non-strict; FourPointPlanar increases mean contacts/multiplicity and fails the terminal convergence gate | Frozen one-factor diagnostic only; no physical/timing/media/paper claim |

The collision probe remains collision-only. The standing bundle establishes a
stable reconstructed trajectory and matching media; the separately frozen
impact result is negative and has no media. None matches the paper's
25-stone/100-contact author row or supplies a passing crown impact, float32
kernel, Apple-silicon host, or timing contract.

## Public Reference And Historical Comparison Boundary

The 2026-07-13 observation that the official page said `Code (coming soon)`
and `matthcsong/fbf-sca-2026` returned 404 is superseded. The MIT-licensed
repository is public and pinned here at
`b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0`. It contains the current
Warp/Newton FBF solver, all six runnable example/configuration sources, pinned
dependencies, optional MuJoCo/Kamino runners, and masonry-arch meshes. The
current local cone-QP kernel, block-GS policy, warm-start matching, and gamma
controller are therefore inspectable.

This removes source availability as an external blocker; source-port and
matched-run work are now internal. Current author invocations were independently
run and preserved. The sealed 500-frame masonry-arch run is a current-source
scientific negative with 1,843/2,000 nonconverged flags; it is neither the
400-frame source default nor a DART or historical-paper comparison. Only
masonry-arch mesh assets are shipped; the public
repository does not contain the historical paper renderer,
cameras/materials/goldens, exact Apple-silicon model, or original paper
invocation/timing logs and warmup/aggregation attestation. Local
evidence remains x86-64/float64 DART on reconstructed scenes, and the paper
publishes no multicore CPU reference.

## Current Verification

- Author-incline reference finalizer unit tests: 64/64; verify-only reports 37
  indexed artifacts and 39 physical files.
- Focused manifest/backspin/incline/author-masonry/author-incline evidence
  suite: 859 passed post-merge in 163.95 s.
- Full no-cache dartpy Python suite: 1,555 passed in 165.09 s.
- Author masonry-arch focused CTest: 1/1 passed; all four current sealed
  bundles pass verify-only.
- The manifest validates 29 canonical requirements with status `partial`.
  Under the sealed producer closure, live mode performed 118 file-identity
  rechecks with zero skipped; explicit archive mode reported zero live
  rechecks and 118 skipped.

The sealed live closure resolved `libdart.so.6.19` to the recorded
`libdart.so.6.19.3`. The normal development symlink is restored to
`libdart.so.6.19.4`; the files are byte-identical, but resolved path is part of
the fail-closed identity contract, so the current default live run reports five
path mismatches. Archive mode remains clean; only recreate the historical
symlink for an explicit live-closure recheck.

## PR State

PR #3374 is merged. Its historical final head is abbreviated `1f816` and its
merge commit is abbreviated `fa17fad`; the relevant visual-evidence changes
are integrated in this worktree.

The #3377 topic contains `origin/release-6.20` through `75306efe770`. Topic
head, divergence, merge state, checks, and reviews are mutable and are
deliberately not frozen into this report. Before reporting or publishing, run:

```bash
git fetch origin
git merge-base --is-ancestor origin/release-6.20 HEAD
gh pr view 3377 --json state,isDraft,headRefOid,baseRefName,mergeable,mergeStateStatus,statusCheckRollup,reviews,reviewRequests
```

Merge the target only when the ancestry check fails. PR #3377 remains work in
progress and is not completion evidence.

The historical failed-check audit separated branch regressions from
base/workflow issues; these observations must not be reported as current-head
results without fresh logs:

- `SplitImpulseKeepsRestingContactVelocityZero` fails by `+0.001` across Linux
  Release/Debug, Eigen64, newest GCC/Clang, macOS Release/Debug, and FreeBSD.
  The published BoxedLCP velocity assembly leaves
  `ConstraintInfo.useSplitImpulse=false`; the dirty local one-line propagation
  fixed the focused local test in the then-current local tree.
- FreeBSD also times out the paper fixtures at 1,800 seconds. The dirty local
  tree gates the three worst scientific stress fixtures behind an explicit
  environment opt-in. Coverage/assert failures from SplitImpulse, that timeout,
  and the invalid-gamma assertion were locally addressed.
- The MJCF NaN assertion remains open and base-owned: the current `75306efe770`
  target fails it too. It is not evidence of an exact-Coulomb branch regression.
- Windows reaches the five-hour job timeout while compiling, then reports
  cancellation and `EBUSY` cleanup. The current target exhibits the same
  workflow/runtime failure and no product test runs.

Local fixes and historical classifications do not substitute for current-head
cross-platform CI.

## Remaining Completion Gates

- Make the strict full-card profile complete and reach `<=1e-6` within the
  paper's 200 outer iterations without fallback. The completed manifold-v2
  sensitivity narrows the contact representation question but does not close
  this gate because both trajectories are non-strict.
- Make the failed small-scene physical/residual contracts pass without
  changing the source claim.
- Keep multicore claims confined to the validated opt-in colored literal-arch
  workload until other exact workloads have equivalent dispatch, residency,
  matched-work, and outcome evidence.
- Complete strict full-duration physical outcomes for every card and arch row.
- Audit and port the pinned public author scenes, solver configuration, local
  kernel, and current external runners; preserve the sealed masonry-arch run
  as a negative and turn its configuration-only DART spec into a dynamics port
  only through a separately validated contract. Produce matched runs or record
  exact DART/source differences. Keep the unavailable historical
  renderer/goldens, Apple hardware, precision, and paper timing-attestation
  gaps explicit.
- Preserve the finalized incline, Painleve, backspin, and turntable bundles
  within their lane-specific claim boundaries. Promote only the remaining
  post-review small rows through validated immutable bundles; produce
  long-scene sidecars/media only when their solver/outcome gates pass.
- Complete the final integrated lint, build, test, Python, docs, manifest,
  trace-schema, demo, media, and independent-review batteries.

Until these gates close, the correct status is partial implementation with
positive small-scene evidence, one positive non-paper contact-rich
mean-throughput/scaling result, and explicit negative paper-profile evidence,
not completion or paper parity.
