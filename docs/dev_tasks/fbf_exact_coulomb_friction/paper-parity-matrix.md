# Paper Parity Matrix

This matrix maps the SCA 2026 paper, project page, and video to DART-facing
requirements. Statuses describe current evidence, not aspirations. The task is
active and incomplete. The manifest remains 24 partial, 5 blocked, and 0
complete across 29 requirements. The local visual inventory has six finalized
bundles, and the visual workflow declares 17 schedules.

## Source And Comparability Boundary

- The MIT-licensed author reference is public at
  `https://github.com/matthcsong/fbf-sca-2026` and pinned here at
  `b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0`.
- The pinned source contains the Warp/Newton FBF solver, all six runnable
  example/configuration sources, pinned dependencies, optional MuJoCo/Kamino
  runners, and masonry-arch meshes. Scene/configuration audit and matched-run
  work are therefore internal rather than externally blocked.
- The author pipeline uses float32 state. Its exact 3x3 cone boundary solve
  promotes the local system to float64 and casts the result back to float32;
  the warm-start matcher and cross-step gamma controller are also public.
- The sealed 500-frame author masonry-arch run is a newly declared
  current-source diagnostic. Its 2,000 substeps contain 157 true flags: 40
  from the initial natural-residual shortcut and 117 from the configured outer
  `coulomb_rel` gate. The remaining 1,843 outer solves are nonconverged. This
  is scientific-negative evidence, not a historical paper invocation or a
  DART-parity result. The source default is 400 frames with release also at
  frame 400, so it never releases the cubes.
- The numeric `author_incline_sweep_reference_v1` packet preserves independent
  current-source FBF, MuJoCo, and Kamino CPU runs on the full Figure 1 grid
  `mu=.3,.4,.45,.5,.55,.6,.8`. Each lane contains seven 120-step cells. The
  retained FBF histories record four contacts per FBF step; the MuJoCo and
  Kamino result records contain no contact-count field. This is source-run
  scientific-negative/reference evidence, not historical, DART-matched,
  golden, media, timing, performance, or parity evidence; the visual bundle
  count remains six.
- The repository does not contain the historical paper renderer
  cameras/materials/goldens, exact Apple-silicon model, or original timing-run
  and warmup/aggregation attestation. Only masonry-arch meshes are shipped.
  Current author invocations were independently run and preserved, but do not
  fill the missing historical invocation/timing record. Local evidence uses
  reconstructed scenes, Native collision, float64, and x86-64 Linux.

Therefore every paper timing verdict is null. A raw time below a source table
value would not be an apples-to-apples result.

The author incline packet records 839/840 configured FBF convergence flags;
`mu=.55` is 119/120, with step 1 false at the 200-outer cap. Of the 839 true
flags, 235 use the initial natural-residual shortcut and 604 use the
configured outer nonnegative `coulomb_rel < 1e-6` gate. Natural
`final_residual` is separate: 456 configured-true rows are at or below
`1e-6`, 383 configured-true rows are above it, and the sole configured-false
row has natural residual `3.273267262002487e-8` but terminal
`r_coulomb=1.5311460572898186e-6`. The close FBF/Kamino displacement
projection and nonmonotone current MuJoCo projection are observations, not
full-state equivalence. First-use JIT, history collection, warmup handling,
and timer-boundary differences exclude every timing field.

Verify the packet without rewriting it:

```bash
python3 scripts/finalize_fbf_author_incline_reference.py --verify-only
```

## Evidence Predicates

The matrix deliberately separates:

| Predicate | Meaning |
| --- | --- |
| `artifact_valid` | File/schema/hash/order/dimensions/frame-count/decode integrity passes |
| `solver_contract_valid` | Every completed required step and solver group passes status, residual, cap, exact-failure, and fallback gates |
| `physical_outcome_valid` | A separate scenario-specific quantitative outcome check passes |
| `manual_inspected` | The rendered result was viewed and a qualitative observation recorded |
| `claim_valid` | Every predicate required by the particular claim, including source/build identity and comparability, passes |

These are not aliases. `artifact_valid` and `manual_inspected` do not imply a
correct trajectory, and `solver_contract_valid` does not imply paper parity.

## Solver-Level Parity

| Requirement | Paper target | Current DART evidence | Status/gap |
| --- | --- | --- | --- |
| Exact reduced Coulomb law | Primal cone, augmented-velocity dual cone, complementarity | Shared cone/residual math, exact local H-metric cone-QP solve, opt-in constraint solver | Implemented locally; broader production and author equivalence remain open |
| Exact local solve | Strongly convex 3x3 cone subproblem | Analytic H-metric solve plus KKT-certified projected-gradient recovery; 47/47 exact-math tests | Robust on tested local systems; the pinned author kernel is inspectable, but DART equivalence and matched performance are not established |
| FBF outer loop | 200 outer iterations, `1e-6`, fixed inner sweeps, adaptive gamma | `paper_cpu` resolves and validates those knobs; current card-manifold v2 holds them fixed across two Native modes; the non-paper literal arch uses a separately declared 5,000-outer tuned profile | Small scenes mostly converge and the tuned literal reconstruction passes; the prior-source strict full card fails, and neither current v2 card trajectory is strict |
| Residual | Dimensionless primal/dual/gap maximum | Per-outer and per-step diagnostics exist; the literal bundle has maximum per-step residual `9.999807145410957e-7` | Positive reconstructed contact-rich evidence exists, but paper-profile convergence is not established |
| Matrix-free response | Scatter/inverse-mass/gather | Contact-row operator exists for supported groups | Complete paper workload equivalence open |
| Warm start | Reuse previous solution | Pair-keyed cross-step records and tests exist | The pinned author point/normal/pair matching is inspectable; DART equivalence remains unaudited |
| Fallback policy | Pure FBF result | Strict profile disables boxed fallback | Prior-source full-card prefix has zero fallbacks but ends in an exact failure; both current v2 modes have zero fallbacks but accepted capped groups on every row |
| No-fallback failure | Do not mix an invalid state into evidence | A failed group is left unsolved and evidence callers must stop | Explicit negative result, not a valid trajectory |
| Threading | Paper CPU rows are sequential | One-core affinity is audited; an opt-in deterministic colored inner BGS records dispatch and per-phase residency separately | Validated 1-to-4-thread scaling exists only for the non-paper literal reconstruction; no paper multicore claim |

The existing boxed-LCP solver remains DART's default. Exact FBF is opt-in.

## Current CPU Matrices

The durable non-paper literal-wedge artifact is
`assets/dart_cpu_evidence/2026-07-19_mark26_native25_colored_v10_archwide_pcore`.
It uses Native `FourPointPlanar`, `1 um` closure, exact-inertia wedges, float64
x86-64 Linux, scale 35, 5,000 outer iterations, 30 fixed inner sweeps,
relaxation 1.1, fresh gamma, and zero diagonal/matrix-free seed. One warmup and
three measured 600-step trajectories complete at each thread count.

Every one of the 1,800 measured steps per thread count has 96 contacts, 24
colliding body pairs/manifolds, three colors, width eight, exact success,
residual `<=9.999807145410957e-7`, zero caps/failures/fallbacks, and the same
valid standing physical outcome. Its whole-arch gate covers all 25 stones:
maximum displacement from the constructed initial state is
`5.431169776791696e-6 m`, and minimum orientation alignment is
`0.9999999999111284`.

| Threads | Mean ms | Median ms | p95 ms | Max ms | Mean 60 Hz | Every-step 60 Hz | Validated speedup | Paper timing |
| ---: | ---: | ---: | ---: | ---: | --- | --- | ---: | --- |
| 1 | `6.122883` | `2.4966535` | `21.663237` | `287.473818` | Pass | Fail | `1.0x` | Null |
| 4 | `4.269397` | `1.9047965` | `14.396602` | `180.504588` | Pass | Fail | `1.434133x` | Null |

This validates mean-real-time throughput and multicore scaling for the named
reconstructed workload only. The one-thread p95 and both maximum times prevent
an every-step deadline claim; the four-thread p95 is below the 60 Hz budget.
The scene is not the author 100-contact impact scene, and the precision,
kernel, host, and timer are not paper-comparable.

Schema v8 preserves the newline-terminated default 83-column header at
SHA-256
`396c866d782e626b5c5b52e74c392de1175f5e46dfd90b809d1f219417092e50`;
the opt-in colored trace has 95 columns and newline-terminated header SHA-256
`424195336cb42753179130c9f6fcba3a8ddea9bf669cfa70c0f77fcb4c6335a5`.

The current-source strict small artifact is repository-archived at
`assets/dart_cpu_evidence/2026-07-19_current_source_paper_cpu_small_r7/`.
It binds 60 indexed artifacts and three repetitions per row, including the
runner, trace source, executable, resolved runtime closure, and a post-run
identity recheck.
Core r7 hashes are report
`008bc94667893cd26bfc04a720caca3d3a5703601c8739bc06856f93818c63d5`,
index `06594c1e1cc3c6858bc78a80630aefb0e85eeef92204b0a09b99425411c3ebeb`,
metadata `e227d7aef3b273ff81385f227f9e6766a7e40a622bc4298e1807b7c2068bc417`,
summary `9137f1b2db0909a96897632a66617f2cfa58476a369d2adfe232b73e46cd0fa2`,
and raw `ba062cf359da85d21b5ea83b722d26375267212dfc44a4ce9f48701ad8a79a5a`.

| Scenario | Steps | Max residual | Physical outcome | Local real-time contract | Paper timing |
| --- | ---: | ---: | --- | --- | --- |
| Backspin | 240 | `9.994e-7` | Pass | Valid | Null |
| Incline, `mu=0.4` | 120 | `9.988e-7` | Pass | Valid | Null |
| Incline, `mu=0.5` | 120 | `1.439e-6` | Pass: `8.634e-7 m` displacement | Invalid: three accepted caps exceed tolerance | Null |
| Painleve, `mu=0.5` | 150 | `9.996e-7` | Pass: upright | Valid | Null |
| Painleve, `mu=0.55` | 150 | `9.950e-7` | Pass: tumbled | Valid | Null |
| Turntable, `mu=0.2`, `omega=2` | 240 | `1.000e-6` | Pass: ejected | Valid | Null |
| Turntable, `mu=0.2`, `omega=5` | 240 | `9.981e-7` | Pass: ejected | Valid | Null |
| Turntable, `mu=0.5`, `omega=2` | 240 | `9.999e-7` | Pass: retained-on-support classifier over the measured horizon | Valid | Null |
| Turntable, `mu=0.5`, `omega=5` | 240 | `3.050e-4` | Classifier pass: ejected | Invalid: three failed processes at the accepted cap | Null |

All nine reconstructed physical classifiers pass. Seven of nine trajectories
satisfy the strict solver and local real-time contracts. This is not 7/9 paper
parity; every paper timing verdict remains null.

The retained prior-source strict full-card artifact is repository-archived at
`assets/dart_cpu_evidence/2026-07-12_prior_source_paper_cpu_card600_negative/`.
Its metadata retains the original `/tmp/fbf_cpu_paper_postreview_20260712_card600`
path. It stops at step 89/600. The
terminal failed residual is `1.8612e-2`; the attempted prefix has one exact
failure, zero fallbacks, and 88 accepted maximum-iteration statuses. Raw
mean/p95/max step time is `59.8234/74.3458/79.5385 ms`, with real-time factor
`0.2786`. The incomplete trajectory and failed solver contract
make every real-time and paper verdict invalid/null.

The current-source, preregistered Native manifold comparison is
`assets/paper_evidence/card_house_26_manifold_sensitivity_v2_r3/`. It holds the
reconstructed 26-card scene and every `paper_cpu` solver option fixed while
changing only `Compact` versus `FourPointPlanar`. Both modes emit all 600 rows,
but both have zero strict-success rows and accepted capped groups on every row.
`Compact` returns zero with 3,495 capped groups across 5,757 exact attempts and
a successful terminal last group at residual `8.525678738415048e-7`.
`FourPointPlanar` uses the valid
`complete_terminal_convergence_gate_failure` exit class with 682 capped groups
across 745 attempts and terminal last-group residual
`0.016582575623909489`. Both have zero exact failures and fallbacks.

The preregistered directional hypothesis is supported: `FourPointPlanar` raises
mean contacts by `93.7983333333` and mean pair multiplicity by
`1.9548548971`. It does not improve strict convergence: despite fewer capped
groups and graph transitions, its terminal residual is higher and fails the
executable's terminal gate. Comparison integrity passes, but both physical and
timing verdicts are null, raw wall time is excluded, and this is reconstruction
sensitivity rather than paper parity. Core hashes are protocol
`eeb1c8c1d09a29e197a3c402217ecd0af9a9878749cb4756cf94bc714f2b60e9`,
trace source
`b00eea0c87f75f17259fac433bafd98c20a961d14389a80e91742d3c8a678f76`,
trace executable
`0923bf7df1eaa518f9a2ffacd0a42fd7330f5cbf6c35cb5749fbc122db1310ff`,
summary
`52a082ab15e8b9c314d706474cc7be557ddfc58c4961faad0d3da9d347f59f4f`,
comparison
`051605c25ccd5aa4de2f243c4dafe547c82f7298f8018cee53a8701d018ff297`,
and metadata
`5890ab138179f4d7aaee6cd04c63799086439bae58fbde4f994048785dd0b8ac`.
The current artifact-index, invocation, report, and whole-tree hashes are
`1703f995bd5d2c6edeb12ba936010a80609c3e952f2c782b554bd0be7ca40627`,
`6e48eb835ab9aafca0b8d4bd29acb5ec932645fb37ca6124ce3c23b3265847f0`,
`0429b12f90e4fef534c9b707324a180eaac78829a3b3667505111228aba58d2e`,
and `953d8efd4d43ee2a74cede6b5e3a0a766ecd1fd8914485b50b340f515b7ecb77`.
The unsuffixed v2 path is historical current-at-capture evidence; v2_r3 has
unchanged diagnostic semantics.

At 90 contacts, 270 contact rows act through 156 generalized velocities,
which gives contact-response nullity at least 114. At 155 contacts, 465 rows
give nullity at least 309. The redundant semidefinite contact system and
contact churn are the leading diagnosis; the local 3x3 KKT certificate is not
the observed limiting failure.

The raw DART-best t1/t8 matrix at
`/tmp/fbf_cpu_dart_best_postreview_20260712_t1_t8_r3` is diagnostic only. Its
legacy path does not parallelize the one coupled exact island, so
`multicore_claim_valid=false` for every row in that matrix. It is not the
colored-inner-BGS contract validated above.

## Figures, Video Segments, And Demos

| ID | Paper fixture/result | Current evidence | Current status and missing proof |
| --- | --- | --- | --- |
| Fig. 1/2 | Incline `mu=0.4` slides and `mu=0.5` sticks | Finalized 21-indexed/23-physical DART visual/trace bundle plus numeric author-pinned FBF/MuJoCo/Kamino runs over `mu=.3,.4,.45,.5,.55,.6,.8` | Partial: DART threshold separation passes, but capture contacts 8 do not match aggregate trace contacts 6; author FBF has 839/840 configured convergence flags, the separate strict DART `mu=.5` row has three accepted caps, and matched DART/full-state external/golden/paper-contact/history/timing/real-time proof remains missing |
| Fig. 3 | Backspin sphere reverses toward analytical terminal state | Finalized `fig03_backspin_current_v3` trace, high-contrast 6x4 checker-textured panel, MP4/GIF, exact-membership index, and manual inspection | Partial: translational advance/reversal plus checker-texture and coral-registration-tile legibility pass for the DART reconstruction; signed angular direction, continuous contact, rest, landing, full-state trace equivalence, and external/paper/golden/timing/real-time parity remain unproven |
| Fig. 4 | Four author-configured turntable cells classify finite-horizon support retention/ejection | Finalized author-pinned 58-indexed/60-physical four-cell repository bundle with four outcome stills, synchronized media, manual inspection, and separate visual/strict lanes | Partial: the current visual lane proves three ejections and `mu=.5, omega=2` retained through 6 s; it does not prove zero slip/co-rotation, full-state equivalence, approved-golden parity, paper timing, real-time performance, or behavior beyond 6 s, while the separate strict lane fails that retained cell at step 40 |
| Fig. 5 | Painleve coefficients produce distinct slide/tumble outcomes | Finalized 27-indexed/29-physical DART-only paired proxy clip/panel plus separate tracked traces and manual inspection | Local proxy outcome only; the author geometry/configuration is public, but the rendered demo and trace are not source-matched or equivalent, faithful external/golden proof is missing, and historical paper timing is unattested |
| Fig. 6 | 26-card house settles, then receives four projectiles | Finalized 12-indexed/14-physical public-author default five-level/40-card construction-only bundle, prior-source strict step-89 negative, current-source 600-row manifold sensitivity v2, and bounded visual attempt | Blocked: the author bundle is step zero with zero simulation substeps; both current v2 trajectories are non-strict with null physical verdicts; prior strict trace fails at step 89, visual reaches only step 6, and no valid long media exist |
| Fig. 7 | 25 tapered voussoirs, pinned springers, crown impact | Finalized literal-wedge standing trace/media, preregistered 720-step DART crown-impact v1 negative, and sealed pinned-author 500-frame scientific negative | Partial: the no-projectile DART reconstruction stands, but DART impact v1 fails its cap/residual/outcome gates and the author run has 1,843/2,000 nonconverged flags; the author run is not the source default or a DART dynamics comparison, and no passing source-matched DART impact/media or paper-timing row exists |
| Fig. 8 | 101-stone arch remains balanced | Oriented-box bounded visual attempt plus provenance-bound exact-inertia literal v6 numeric run | Blocked: oriented-box attempt collapses by step 120/600; literal v6 fails closed on step 1 at residual `0.7815364614`, so neither supplies a standing trajectory or valid media |
| Fig. 9 | Residual histories for backspin, house, arches | Export infrastructure, prior-source strict card negative, current-source card-manifold sensitivity, positive per-step literal-arch timing rows, and a deterministic pinned-author claim-history projection representing every one of 2,000 substeps | Partial: the author run is a new nonconvergent diagnostic whose configured `coulomb_rel` flag must not be conflated with its projected natural residual; the raw author history is hash/size-bound but omitted, paper-profile/matched per-outer card and arch histories remain incomplete, and the DART literal bundle is per-step |
| Fig. 10 | Gamma sweeps with physical-outcome interpretation | Sweep/trace infrastructure | Partial: the pinned recovery policy is inspectable, but DART equivalence and paper-profile contact-rich outcomes are not established |
| GPU table | Card/arch outcomes and timing | Construction adapter and reconstructed Kamino runner | Non-comparable; dynamic 10-level capture does not complete step 1 |

The finalized Fig. 1/2 and video.03 bundle is
`assets/paper_evidence/fig01_02_incline_current_v1/`. Finalization and
clean-checkout verify-only pass with status
`valid_current_source_nonpaper_incline`; the directory contains 23 physical
files and the exact-membership index binds 21 artifacts. The combined 660x506
capture retains five durable stills and a 61-frame decoded 30 fps schedule.
Its 70-file raw capture staging directory is pruned after sealing, so
verify-only needs no ignored staging. It records 240 exact attempts/solves,
zero accepted caps, exact failures,
or fallbacks, maximum residual `9.999836962261359e-7`, and eight contacts per
post-initial step.

Each independent tracked trace contains 121 rows, 120 exact solves, 119 warm
starts, zero fallbacks, three contacts per post-initial step, and continuous
post-initial tracked contact. At `mu=.4`, downhill displacement is
`1.7686892884927794 m` versus analytical `1.7548661487418349 m`, final
downhill speed is `1.7544655347780056 m/s`, and maximum residual is
`9.986952135669881e-7`. At `mu=.5`, downhill displacement is
`0.0008905412965980523 m`, maximum/final stick speed is
`0.001116442058867632 m/s`, and maximum residual is
`9.997210606407098e-7`. The tracked displacement separation is
`1.7677987471961814 m`.

The only byte-identical capture/trace comparison is the aggregate
`step`/exact-solve/fallback projection at SHA-256
`f03dff0aaec5f0fa6615609c2ea97aa31f072c14c6212fffe15858ea969d88c2`.
Capture contacts 8 versus aggregate trace contacts 6 are explicitly excluded:
`contact_count_match=false` and `contact_counts_compared=false`. Different
placements prohibit state, residual, status, warm-start, per-cell, and
full-trace equivalence claims.

Finalized SHA-256 values are metadata
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

The separately finalized current-source strict `paper_cpu`/Native matrix is
still authoritative for that lane. Its `mu=.5` physical classifier passes at
only `8.63436433e-7 m` displacement, but one accepted cap per repetition
raises maximum residual to `1.4392081500753078e-6`; strict
solver/local-real-time remains failed. Fig. 1, Fig. 2, and video.03 therefore
remain `partial` pending the full friction sweep/plot, matched external rows,
approved source golden/diff, paper contact-count match, full 11 s semantic
edit, paper timing, and real-time parity.

The finalized Fig. 3/video.02 bundle binds 18 indexed artifacts in a 20-file
physical directory. The MP4/GIF preserve the motion schedule, three durable
stills retain steps 0, 1, and 2, and the 140-file raw capture staging directory
is pruned after sealing; clean-checkout verify-only needs no ignored staging.
The bundle records 129 exact attempts/solves with zero accepted caps, failures,
or fallbacks. Trace/capture maximum residuals are
`9.964971544991853e-7`/`9.96497154974839e-7`. The trace reaches maximum
`x=1.5959314363310166` at step 48, first records negative `vx` at step 49,
and ends at `x=-2.9362508912363654`, `vx=-6.628158971623909`; step 120 is
the sole contact-free post-initial step. Trace and capture solver/contact
projections share SHA-256
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
The renderer applies a high-contrast 6x4 ivory/charcoal checker texture with
one coral registration tile through a visual-only UV `MeshShape` under
`VisualAspect`. The physical `SphereShape` remains unchanged and continues to
own collision, dynamics, inertia, and friction. Manual inspection passes
checker-texture and registration-tile legibility only. The 30/15 fps media can
alias the `-200 rad/s` motion and do not prove signed angular direction. Step
120 rules out continuous contact; neither rest nor an airborne landing phase
is proven. Separate demo and CSV scenes are not full-state trace-equivalent.
External-solver, paper, approved-golden, timing, and real-time parity remain
unproven. Both `fig.03` and `video.02_backspin` remain `partial`.

The finalized Fig. 4 bundle is
`assets/paper_evidence/fig04_turntable_author_current_v1/`. It pins author
commit `b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0`, binds 58 indexed artifacts
in a 60-file physical directory, and preserves source order
`mu=.2/omega=2`, `mu=.2/omega=5`,
`mu=.5/omega=2`, `mu=.5/omega=5`. All four current `dart_best`/Native
`FourPointPlanar` visual cells complete 360 steps with valid solver contracts
and no fallbacks. Their exact finite-horizon outcome is ejected, ejected,
retained on support through 6 s, ejected. The retained cell does not establish
zero slip, perfect sticking, co-rotation, or a longer-horizon outcome.

Manual inspection passes the segmented visual disc, one coral registration
wedge, labels, and source order; the physical cylinder remains the sole
collision/dynamics geometry. Capture and trace projections are byte-identical
for all four cells over six solver/contact fields, not full state. The separate
strict `paper_cpu_native` lane has no capture comparison: `mu=.5, omega=2`
fails at step 40 with residual `7.407835021099202e-6`, while its other three
rows pass. The lanes must not be substituted for one another.
Four durable timeline-selected outcome stills bind steps 136, 120, 360, and
90 in source order. Capture staging is pruned after sealing;
clean-checkout verify-only needs no ignored files.

Bundle status is `valid_author_source_pinned_nonpaper_turntable_matrix`, with
the current visual artifact, solver, physical, manual-inspection, and pass
gates true. Core hashes are report
`930cc12b95ab78c6e61d084064b584f5872633f2432e434c68d071818cec7fb1`,
index `209b677ce35e2b9c11248ab414727016394831084881a5e9c2866f68b51f6cdf`,
metadata `854ef0c8aad75b4b200f512951d56b4dbef7aa82c46cc5e82123f0583dcc6ac5`,
verification `455da7686eaeeb989e0d122c4724ea66ff161e8d652230eff9fb8ad20590bacd`,
and group clip `b241463658e6b48dfb2c74815e85317c2f0eccd46a7f7ab978b8f2701ce80d6d`.
Author-spec/visual-OBJ/visual-MTL/manual-inspection hashes are
`1680cd8351fa62937c0318826f7abc75917234cb3888f983acce06f13698bc6c`,
`bc86f1ef1f5fae1510f23b1586ae20efe788c499373370a66af81b06818f1b14`,
`619352b9ac14e89a4d467dde867019e0d01540b6f11852df565f23fb26a01752`,
and `095652d3df70a144be31be49b0e25b3265df54a3815df21742333d5fdfb4529a`.
Current visual runner/test bindings are
`d848afa53caf14b9fb3ea061d658eef274e8d917151937bd6340283b79ab5432`
and `6e378252fa6a7cb51c6813c9d5a2b30b8c8129eacdb01df0ee19a58a270cbc5e`.
It is not paper-comparable, approved-golden, paper-timing, or real-time
evidence.

The source audit validates the pinned video, teaser, and paper and all nine
contiguous segments across 82 s. This proves source identity and coverage, not
local parity.

The repository bundle at `assets/paper_evidence/fig07_arch25_literal/` is valid
current-source evidence for the non-paper, no-projectile literal-wedge standing
reconstruction. Its five durable 1280x720 stills and 61-frame H.264 schedule
fully decode; 600 independently generated capture/trace rows have zero
differences. Manual inspection covers the durable stills, timeline, and a
separately decoded midpoint. The retained pending hash DAG passed before final
writes, and the final artifact index validates 19 artifacts in a 21-file
physical directory. The 70-file raw capture staging directory is pruned after
sealing, and clean-checkout verify-only needs no ignored files. Ten simulated
seconds play in 6.1 seconds at 10 fps (`1.639344x` time-lapse), so the clip is
not a real-time playback result. Core hashes are metadata
`b217b0a43200ca558f09776e464d50f0f9dc5c5a175bcfa66ae1b656076871d1`,
index `4d173c78e2c6584d6d8a85867160cc0c0687fe0f35e6878f66b0cfcd7c1c15ee`,
video `e0c111f96511bbda41af145787bd9f68782b8c9e8a909d45eb03d3c3d647feb1`,
and timeline
`926380c5c5768cc9ca1e414cbfe2631584daa99387090926d5a1f035d62b6ec9`.
This closes standing-media integrity only, not the paper crown impact or parity
gate.

The capture source, capture binary, trajectory, durable stills, and media remained
immutable through later additive scenario work. Fresh revalidation against the
final current trace and Native source regenerated reference trace
`0e41af230fb4c27143966c3f8522702280e6746b31e84155a9a047326e46dbb3`
and again validated 600 zero-difference standing rows. The current trace
source/binary hashes are
`b00eea0c87f75f17259fac433bafd98c20a961d14389a80e91742d3c8a678f76`
and
`0923bf7df1eaa518f9a2ffacd0a42fd7330f5cbf6c35cb5749fbc122db1310ff`.

The durable impact bundle is
`assets/paper_evidence/fig07_arch25_literal_impact_v1_negative_final_v9/`. Its
600-step projectile-free prefix has zero mismatches across 88 eligible fields. In the
720-step frozen run, projectile-arch contact begins at step 607 before ground
contact at 616; every body remains finite with zero exact failures/fallbacks.
The result is nevertheless a scientific negative: five caps are accepted,
worst residual is `9.1545317042653963e-5`, final all-arch displacement is
`0.07093964431215687 m > 0.07 m`, and far-field displacement is
`0.060523747030465196 m > 0.007 m`. No post-result tuning was allowed. The raw
and summary hashes are
`42cc94f5f111442da42b03af33722f713d66b853d3ac21f21083f28ba24e7ba4`
and
`e3baa479111a999d03359a0642533cad1baa3a48c415609bafb9a7a7cfa18e7c`.
The independently regenerated standing reference SHA-256 is
`22fabfe2e8f35bf329f32873845a60599ed2df171c15728f14311b91d888c387`.
The final runner and metadata hashes are
`622b388142fed881191dcb6efb103266eb2ea1e3ffbadaf402f63b89c558bb67`
and
`0bd19797953a10223bbd7183f6e8002e490b21c29b2304befee7e6618ff9ee73`;
the normalized trace fingerprint is
`86b37ec37d28259514453c9ecc9e7d5f12afe08118a0b04abd40e72acd147384`.
This directly falsifies the preregistered v1 impact claim; it does not weaken
the positive standing result or establish paper parity.
The v6, v7, and v8 paths are historical current-at-capture evidence; v9
preserves the same frozen negative semantics and records the executed
`taskset` identity in its runtime closure.

The current literal 101-stone v1 artifact is
`assets/paper_evidence/fig08_arch101_literal_v1_negative_final_v6/`. It fails
closed on step 1 after 5,000 outer iterations with `fbf_failed`, residual
`0.7815364614352474`, one exact failure, and zero fallbacks. The dynamic
`FourPointPlanar` row reports 400 contacts, 100 constraint pairs, three colors,
and width 34. The separate repeat-2 Compact collision probe proves only the
constructed time-zero graph of 100 adjacent-stone pairs plus two
springer-ground pairs. The failed prefix supports no standing, physical,
timing, or media claim. Raw, invocation, summary, metadata, and report hashes
are
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

The post-review small visual matrix contains eight dynamic schedules with
cumulative all-group/every-completed-step residual `<=1e-6` and zero caps,
failures, or fallbacks. Its turntable video is 1320x1060 at 30 fps for 121
frames with SHA-256
`e5bf08986d6e70bdf25797e66c958c9ac947f9b2dc863b30d37afe1efd29f11e`.
Its Painleve video is 1320x530 at 30 fps for 76 frames with SHA-256
`d25a93abf964df707e3c10c30202bf75d76e0b216b9f2301b393dca63322afd0`.
They were captured after the final review fixes and manually inspected. The
turntable output and the historical Painleve hash remain external reconstructed
evidence rather than repository-published or paper-golden media.

The Painleve subset now also has a distinct finalized repository bundle at
`assets/paper_evidence/fig05_painleve_proxy_current_v1`. Its synchronized
1320x530, 30 fps, 76-frame clip has SHA-256
`dd4cdda2410b71f8e86035f1c0ff278f9dda77133c702e409ee5d533da443a4b`.
Manual inspection records `mu=.5` returning upright and `mu=.55` tumbling and
remaining visually horizontal. Separate tracked traces corroborate the local
classifier and the 0.03836187995520213 m shorter pre-tumble distance. They do
not establish trace equivalence, pinned-source geometry/configuration
equivalence, external-solver parity, an approved source golden/diff,
historical paper timing, or real-time performance.

## Arch Evidence Tiers

| Tier | Geometry and inertia | Observed contacts | Permitted claim |
| --- | --- | --- | --- |
| Production, 25 stones | Weighted-catenary oriented boxes | Natural 96; GUI reduced 48 | DART reconstructed box fixture |
| Production, 101 stones | Weighted-catenary oriented boxes | Full profile reaches cap 512; GUI reduced 38 | DART reconstructed capped box fixture |
| Literal 25-stone collision audit | Prism/wedge collision shapes with exact uniform-prism inertia | 24 nominal pairs; 26 with closure/ground | Collision graph/inertia audit only |
| Literal 25-stone exact dynamics | Exact-inertia wedges, `1 um` closure, Native `FourPointPlanar` | 96 contacts, 24 pairs/manifolds, 3 colors, width 8 throughout 600 steps | Non-paper standing/outcome, validated standing media, and matched 1-to-4-thread evidence only |
| Literal crown-impact v1 | Same standing arch plus three frozen projectiles; numeric only | Contact-order and finite-state gates pass; 5 caps, residual, all-body, and far-field gates fail | Preregistered non-paper scientific negative only |
| Pinned-author 25-stone diagnostic | Author meshes and Warp/Newton float32 pipeline; 500 frames / 2,000 substeps | Release substep has 100 contacts; 40 initial-shortcut and 117 configured-outer-gate convergences, plus 1,843 nonconverged outer solves; no contact-pair identities | Current-source author scientific negative only; no DART dynamics, trajectory, outcome, timing, repeatability, media, or paper-invocation parity |
| Literal 101-stone v1 | Exact-inertia wedges, `1 um` closure, Native `FourPointPlanar`; current v6 numeric bundle only | Step 1: 400 contacts, 100 manifolds, 3 colors/width 34; 5,000 outers then `fbf_failed`, residual `0.7815364614` | Frozen non-paper failed-prefix scientific negative only; no standing, timing, or media claim |

The DART dynamic tiers are reconstructions. The pinned-author row is a separate
source-run diagnostic, not a DART tier. No row reproduces a passing matched
25-stone/100-contact impact result or paper timing contract.

## Paper CPU Targets

These are source targets, not achieved DART results.

| Scene | Paper contacts | Paper FBF CPU target |
| --- | ---: | ---: |
| Backspin | 1 | 6.0 ms |
| Incline, `mu=0.5` | 4 | 5.5 ms |
| Incline, `mu=0.4` | 4 | 5.3 ms |
| Painleve, `mu=0.5` | 4 | 7.0 ms |
| Painleve, `mu=0.55` | 4 | 6.4 ms |
| Turntable cells | Includes contact-free/ejected cells | 3.1-6.8 ms |
| Four-level card house | 214 | 199 ms |
| 25-stone arch | 100 | 595 ms |
| 101-stone arch | Not in the same contact-count row | 1234 ms |
| 10-level card house | Not in the same contact-count row | 853 ms |

No local row may be labeled as meeting or beating these targets.

## Validation And PR State

The last recorded focused gates report 47/47 exact-math tests, 25/25 exact
constraint-solver tests, 64/64 `ConstraintSolver` tests, 42/42 Native-collision
tests, 3/3 masonry-geometry tests, 19 passing paper fixtures with 3 skipped,
9/9 focused CTests in both Release and Debug, 12/12 author-incline finalizer
tests, 777/777 focused manifest/backspin/incline/author-masonry/author-incline
evidence tests, and 1,555/1,555 full
no-cache dartpy Python tests. A clean-checkout finalized-incline verify-only pass also
succeeds with 21 indexed artifacts and no ignored staging dependency, its
focused finalizer suite passes 62/62, and 1,000
consecutive successful colored dispatch/barrier
stress runs. The prior-source strict full-card failure and current-source v2
sensitivity result remain retained negative/non-strict evidence.

PR #3374 is merged at final head `1f816` and merge commit `fa17fad` (abbreviated
hashes). The #3377 topic contains `origin/release-6.20` through `75306efe770`.
Its topic head, divergence, merge state, checks, and reviews are intentionally
not frozen into this matrix; fetch and query them live before reporting or
publishing. PR #3377 remains work in progress and is not completion evidence.
`PR_REPORT.md` records the live-verification policy and historical
branch-regression versus base/workflow CI classification.

## Completion Bar

The matrix is not complete until every row is reproduced or precisely blocked
with current source evidence. Reproduced rows require a full-duration physical
outcome, passing exact-solver contract, reproducible affinity/workload data,
current-build media with manual inspection, synchronized reports/manifest,
and clean integrated review. Lane-specific mismatches, including the incline
capture's eight contacts versus six in the aggregate traces, must remain
explicit. Pinned-source equivalence, historical hardware,
precision, renderer, and timer mismatches must remain explicit. This bar is
not met.
