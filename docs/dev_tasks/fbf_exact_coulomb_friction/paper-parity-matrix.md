# Paper Parity Matrix

This matrix maps the SCA 2026 paper, project page, and video to DART-facing
requirements. Statuses describe current evidence, not aspirations. The task is
active and incomplete. The manifest remains 24 partial, 5 blocked, and 0
complete across 29 requirements. The local visual inventory has six locally finalized
bundles, and the visual workflow declares 31 schedules.

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
- The current DART Figure 6 adapter selects the supported author CLI arguments
  `--solvers fbf --levels 4 --frames 600 --drop-frame 400 --num-cubes 4
  --mu 0.8 --cube-size 0.4 --cube-density 500 --drop-height 1.0 --device cpu
  --profile --usd`. It is not the no-argument five-level/800-frame source
  default and is not known to be the historical paper invocation. Source
  `ke=1e4`, `kd=1e3`, and `gap=.005` are recorded source semantics, not contact
  semantics implemented equivalently by the DART adapter.
- A separate public-source-default DART adapter now binds the no-argument
  five-level, 800-frame card-house configuration: 40 cards, four cubes,
  3,200 substeps, and release after completed step 1,600. This is a non-paper
  source row. Strict exact fails closed after step 31 on a retained 39-contact
  group; the separately named continuation exact and boxed lanes complete the
  full clock. Exact requests source continuation while boxed does not, so their
  synchronized media is explicitly policy-asymmetric and not a solver-only
  A/B. It does not recover the historical Tables 6-7 invocation, Warp/Newton
  float32 collision/stiffness/backend semantics, source trajectory, or a
  paper-video five-level segment.
- The sealed 500-frame author masonry-arch run is a newly declared
  current-source diagnostic. Its 2,000 substeps contain 157 true flags: 40
  from the initial natural-residual shortcut and 117 from the configured outer
  `coulomb_rel` gate. The remaining 1,843 outer solves are nonconverged. This
  is scientific-negative evidence, not a historical paper invocation or a
  DART-parity result. The source default is 400 frames with release also at
  frame 400, so it never releases the cubes.
- A separate DART adapter binds the current source-supported `--stones 101`
  selection, all 101 author meshes, and the 400-frame no-release schedule. No
  historical Figure 8 invocation was recovered. DART Native collision and
  float64 dynamics are not the author Warp/Newton float32 backend; the current
  exact step-209 cap and boxed collapse are scientific-negative evidence, not
  source or paper parity.
- Independent full current-source FBF and Kamino controls also fail the local
  standing criterion. Kamino completes all 400 frames with finite arrays, but
  98/99 mobile stones cross the three-unit height-change gate and the keystone
  drops `82.03050136566162`. It saves no convergence/contact history, full
  poses, rotations, cube trajectory, or media, so it is not a historical
  Figure 8 oracle.
- The numeric `author_incline_sweep_reference_v1` packet preserves independent
  current-source FBF, MuJoCo, and Kamino CPU runs on the full Figure 1 grid
  `mu=.3,.4,.45,.5,.55,.6,.8`. Each lane contains seven 120-step cells. The
  retained FBF histories record four contacts per FBF step; the MuJoCo and
  Kamino result records contain no contact-count field. This is source-run
  scientific-negative/reference evidence, not historical, DART-matched,
  golden, media, timing, performance, or parity evidence; the visual bundle
  count remains six.
- A separate DART adapter now binds the source geometry/clock and the
  operator-selected seven-value grid to simultaneous labeled lanes. Exact and
  boxed complete 120 steps and pass supported/upright/in-lane/contact plus
  first-three-slide/last-four-stick gates. Their retained-source-FBF maximum
  endpoint deltas are `0.002426469449185232 m` / `0.0011201728594518558 m/s`
  and `0.0011521317667995284 m` / `0.00030012480388411203 m/s`. The 61-frame
  ignored paired clip independently verifies, but this is only a
  current-source terminal/outcome slice—not source trajectory/backend/solver
  or full physical equivalence, historical video/golden, timing, superiority,
  or paper parity.
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
| Exact local solve | Strongly convex 3x3 cone subproblem | Analytic H-metric solve plus KKT-certified projected-gradient recovery; 56/56 exact-math tests | Robust on tested local systems; the pinned author kernel is inspectable, but DART equivalence and matched performance are not established |
| FBF outer loop | 200 outer iterations, `1e-6`, fixed inner sweeps, adaptive gamma | `paper_cpu` resolves and validates those knobs; current card-manifold v2 holds them fixed across two Native modes; the non-paper literal arch uses a separately declared 5,000-outer tuned profile | Small scenes mostly converge and the tuned literal reconstruction passes; the prior-source strict full card fails, and neither current v2 card trajectory is strict |
| Residual | Dimensionless primal/dual/gap maximum | Per-outer and per-step diagnostics exist; the literal bundle has maximum per-step residual `9.999807145410957e-7` | Positive reconstructed contact-rich evidence exists, but paper-profile convergence is not established |
| Matrix-free response | Scatter/inverse-mass/gather | Contact-row operator exists for supported groups | Complete paper workload equivalence open |
| Warm start | Reuse previous solution | Pair-keyed cross-step records and tests exist | The pinned author point/normal/pair matching is inspectable; DART equivalence remains unaudited |
| Fallback policy | Pure FBF result | Strict profile disables boxed fallback | Prior-source full-card prefix has zero fallbacks but ends in an exact failure; both current v2 modes have zero fallbacks but accepted capped groups on every row |
| No-fallback failure | Do not mix an invalid state into evidence | A failed group is left unsolved and evidence callers must stop | Explicit negative result, not a valid trajectory |
| Threading | Paper CPU rows are sequential | One-core affinity is audited; an opt-in deterministic colored inner BGS records dispatch and per-phase residency separately | Validated 1-to-4-thread scaling exists only for the non-paper literal reconstruction; no paper multicore claim |

The existing boxed-LCP solver remains DART's default. Exact FBF is opt-in.

## Current CPU Matrices

The locally sealed non-paper literal-wedge artifact is
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

The current-source strict small artifact is retained in the ignored local
evidence cache at
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

The retained prior-source strict full-card artifact is retained in the ignored
local evidence cache at
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
| Fig. 1/2 | Incline `mu=0.4` slides and `mu=0.5` sticks | New source-bound seven-cell DART demo/schedule over `mu=.3,.4,.45,.5,.55,.6,.8`, independently verified exact/boxed paired media, the older two-cell visual/trace bundle, and numeric author FBF/MuJoCo/Kamino runs | Partial: both DART lanes classify `.3/.4/.45` slide and `.5/.55/.6/.8` stick and match the retained source-FBF terminal projection within `.01 m/.01 m/s`. The source oracle is non-strict at 839/840 configured flags (`mu=.55`, step 1 at 200/200); source trajectory/backend/full-state, historical invocation/golden/video, paper contact/history/timing/real-time, superiority, and paper parity remain missing |
| Fig. 3 | Backspin sphere reverses toward analytical terminal state | Locally finalized `fig03_backspin_current_v3` trace, high-contrast 6x4 checker-textured panel, MP4/GIF, exact-membership index, and manual inspection | Partial: translational advance/reversal plus checker-texture and coral-registration-tile legibility pass for the DART reconstruction; signed angular direction, continuous contact, rest, landing, full-state trace equivalence, and external/paper/golden/timing/real-time parity remain unproven |
| Fig. 4 | Four author-configured turntable cells classify finite-horizon support retention/ejection | Locally finalized author-pinned 58-indexed/60-physical four-cell bundle with four outcome stills, synchronized media, manual inspection, and separate visual/strict lanes | Partial: the current visual lane proves three ejections and `mu=.5, omega=2` retained through 6 s; it does not prove zero slip/co-rotation, full-state equivalence, approved-golden parity, paper timing, real-time performance, or behavior beyond 6 s, while the separate strict lane fails that retained cell at step 40 |
| Fig. 5 | Painleve coefficients produce distinct slide/tumble outcomes | The source-pinned `fig05_painleve_author_current_v1` exact/boxed DART bundle passes capture summary and independent verify with four complete traces, four 61-frame decoded member/group results, strict exact audit, classified outcomes, and manual inspection; the older DART-only proxy remains historical diagnostic evidence | Partial: under the pinned current DART adapter, exact and boxed diverge at `mu=.55`; GitHub attachment URLs remain pending, and source-backend equivalence, trajectory equivalence, paper Figure 5 parity, timing comparability, and solver superiority remain false |
| Fig. 6 | 26-card house settles, then receives four projectiles | Strict source-selected adapter plus a separate telemetry-rich source-continuation exact/boxed capture, six bounded c95 colored/global-scope/source-gap/cadence/terminal-estimate/source-seed-values diagnostics, the distinct five-level construction bundle, reconstructed prior-source strict step-89 negative, and manifold sensitivity v2 | Blocked for strict/parity completion: strict exact fails closed at step 35 on the retained 56-contact group. Colored ordering and global scope are bounded rejects. A source-sized-gap candidate fails the 36-step gate at step 31, while c95 cadence-5, same-binary `last_norm10`, and same-binary source-seed-values candidates still fail at step 35; each has explicit claim limits and leaves the scene unchanged. The continuation pair completes 2,400/2,400 through release; exact records 3,351/3,351 solves, 0 failures/fallbacks, 113 plateau and 633 max-iteration accepts, 0 shrink caps, and worst residual `0.917120`. Manual inspection finds both standing through release and more endpoint structure in exact, but this is qualitative continuation evidence, not strict convergence, quantitative outcome/trajectory/golden/backend/timing parity, superiority, or paper parity |
| Public-source default (non-paper row) | No-argument five-level, 800-frame card house | Strict and separately named source-continuation DART adapters, plus final v3 exact/boxed members and a policy-qualified synchronized presentation clip | Supplemental only; it does not alter the canonical 29-row manifest or close Fig. 6 / Tables 6-7. Strict fails after 31 completed substeps before release on a 39-contact group (`9.022404720646783e-6` vs `1e-6`). Continuation exact and boxed complete 3,200/3,200 and release at 1,600; exact records 7,337/7,337 solves, zero failures/fallbacks, 2,245 plateau and 836 max-iteration accepts. Manual v3 inspection finds exact retaining substantial standing structure while boxed is largely flattened, but the exact-only continuation policy makes this a qualitative presentation—not strict success, solver-only A/B, automated physical outcome, superiority, historical/source trajectory, timing, paper-video, or paper parity evidence |
| Fig. 7 | 25 tapered voussoirs, pinned springers, crown impact | Finalized literal-wedge standing trace/media, preregistered 720-step DART crown-impact v1 negative, sealed pinned-author 500-frame scientific negative, and a newly registered 2,000-substep source-configuration DART capture schedule | Partial: the no-projectile DART reconstruction stands, but DART impact v1 fails its cap/residual/outcome gates and the author run has 1,843/2,000 nonconverged flags. The new DART exact lane clears the former step-68 local failure and completes 100 strict steps, then fails closed on a distinct outer iteration cap at step 142 (`96` contacts, residual `8.6992952e-4`, zero local exact failures/fallbacks); neither lane has completed the 2,000-step physical/media gates, the 500-frame invocation is not the source default, and no source-matched DART impact or paper-timing row is established |
| Fig. 8 | 101-stone arch remains balanced | Source-pinned current-DART 101-mesh adapter and boxed-collapse media, independent full current-source FBF/Kamino numeric negatives, plus the older oriented-box bounded attempt and provenance-bound literal v7 failed-step-1 run | Partial blocker evidence; parity blocked: source-pinned exact stops after step 209 on an iteration cap (`208` contacts, residual `1.2582804496e-6`), and boxed completes 1,600 steps but fails the standing oracle and collapses. Current-source FBF and Kamino also fail the local standing criterion. The diagnostic hstack freezes exact at step 208. No lane supplies standing, the historical invocation/backend, full-pose source trajectory, golden media, timing, or paper-parity proof |
| Fig. 9 | Residual histories for backspin, house, arches | Export infrastructure, prior-source strict card negative, current-source card-manifold sensitivity, positive per-step literal-arch timing rows, and a deterministic pinned-author claim-history projection representing every one of 2,000 substeps | Partial: the author run is a new nonconvergent diagnostic whose configured `coulomb_rel` flag must not be conflated with its projected natural residual; the raw author history is hash/size-bound but omitted, paper-profile/matched per-outer card and arch histories remain incomplete, and the DART literal bundle is per-step |
| Fig. 10 | Gamma sweeps with physical-outcome interpretation | Sweep/trace infrastructure | Partial: the pinned recovery policy is inspectable, but DART equivalence and paper-profile contact-rich outcomes are not established |
| GPU table | Card/arch outcomes and timing | Construction adapter and reconstructed Kamino runner | Non-comparable; dynamic 10-level capture does not complete step 1 |

The ignored Figure 5 bundle is
`docs/dev_tasks/fbf_exact_coulomb_friction/assets/paper_evidence/fig05_painleve_author_current_v1/`.
At `mu=.5`, exact and boxed classify `upright_near_rest` after
`1.5986787381 m` and `1.5977005918 m`. At `mu=.55`, exact classifies
`tumbled_near_rest` after `1.5399225956 m`, while boxed classifies
`upright_near_rest` after `1.6623056217 m`. Exact `mu=.5` records 119
attempts/solves, zero failures/fallbacks, final residual `5.2255077e-7`, and
worst `9.7391465e-7`; exact `mu=.55` records 108 attempts/solves, zero
failures/fallbacks, final `9.1964345e-7`, and worst `9.9977460e-7`. The adapter
also binds the exact-options header hash. Capture summary and independent
verify pass, all 61-frame H.264/yuv420p member/composite clips fully decode,
and the panels/keyframes were manually audited. This evidence supports only
current-DART-adapter divergence at `mu=.55`; the PR videos remain pending
manual browser-composer upload.

The Figure 6 adapter pins author commit
`b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0`, uses 20 leaning plus 6 bridge
cards, and starts four `0.8 m`, `256 kg` cubes kinematic. The `p` action
releases them immediately when used interactively; the evidence runner invokes
`p` after completed substep 1,600. The horizon is 2,400 substeps at
`dt=1/240 s`. Exact and boxed use the same Native
`FourPointPlanar` frontend with contact capacity 4,096 and manifold subdivision
4. The demo build, 13 focused headless/continuation C++ tests, 359 runner Python
tests, and both contract-smoke validators pass.

The strict exact 100-step prefix records 103 attempts, 102 solves, one failure,
zero fallbacks, zero accepted caps, and worst residual
`4.1039190451256334e-4`; steps through 34 were clean with prior worst residual
`9.826274595482653e-7`. At step 35 the constrained groups are 56, 8, and 4
contacts. Attempt 101 reaches its 200-iteration cap on the 56-contact group;
attempts 102 and 103 then succeed. The additive `last_failure` object retains
the failed group's `fbf_failed` / `success` / `max_iterations` status, contact
count, residual terms, worst-contact indices, iterations, and gamma diagnostics
after those later successes. Its timeline SHA-256 is
`2d04d31134426ac2c4fc87b1774d5285b77740acaeb3ec3a005557b85944bb9d`.
The boxed 100-step timeline SHA-256 is
`fdd3d9e96058176faa51b148d1bcf5a4c0a7f1c4e7da64e15490dcae4ce6fafc`.
Bounded existing-option tuning did not produce a strict solution: even a
50,000-iteration budget only moved the first failure to step 48, and the tested
policy, seed, retry, regularization, and warm-start ablations failed at or
before the same prefix. A process-local GDB override accepting capped iterates
completed all 2,400 steps and invoked `p` at step 1,600, but its runtime command
cannot reproduce the mutation. It accepted 1,106 of 3,231 solves at cap and
recorded worst residual `0.61608914241359314`. This unsealed preview establishes
finite continuation only and remains a historical negative.

The isolated c95-bound global-scope A/B merges all active constraints only at
the solve boundary while preserving the strict scene and solver contract. Both
paired 36-step runs fail-fast at completed step 35. Native reproduces the
56/8/4-contact partition and `4.0844850280896461e-4` failure. The global
68-contact solve reaches `4.0848243204467147e-4`; its stock-native-partition
sub-audit leaves only the 56-contact island nonconverged at
`4.0848243204472058e-4`, with the other islands passing and every off-block
`W` coefficient zero in all 35 observed generations. The two modes enter
generation 28 with identical problems and both accept within tolerance, but
their reactions differ; generation 29 is the first divergent contact/state
fingerprint. This rejects native-island scope alone as the step-35 cause, not
global/per-island equivalence or any source, trajectory, outcome, performance,
superiority, video, Figure 6, or paper claim. The isolated report and manifest
under `/tmp/fbf_fig06_global_scope_c95.TSfONI/` have SHA-256 values
`633828adbe08577b6d0973ca817194530ed8a08cbe27e85d2bcb004689919fe9` and
`90d72452c6b3ed09e0bc1e408b56e70092557784fd2089e6895d7a31a0c809d3`.

The c95-bound one-factor source-gap diagnostic enables only the existing
four-level scenario flag for Native predictive negative-depth closure, with
`0.1 m` on the ground and `0.005 m` on all 30 dynamic shapes. It preserves the
strict source-inner serial contract but fails after completed step 31 on a
31-contact group at 200 iterations and residual
`1.0006073317077885e-5`: 186 attempts, 185 solves, one failure, zero cap
acceptances, and zero fallbacks. Contacts differ from stock immediately, and
the stock comparison sidecar is ancestor-bound rather than a fresh c95 binary.
This rejects only that DART gap representation as a strict 36-step fix; it is
not general gap, source-contact, trajectory, outcome, backend, float32, timing,
performance, superiority, video, Figure 6, or paper-parity evidence. Keep the
scene unchanged. The verified `/tmp/fbf_fig06_gap_c95.m6bsif/` `RESULTS.md`
and `SHA256SUMS` hash to
`3b0948c80871d19cbe29495a8abc57ac4f3e92dc518a9ae6551238a9aad9b17a` and
`11888f98a24175f50c09ce95509d754d0bbc1963e5d2294ad982ece280292119`.

The c95 cadence-5 candidate changes one internal exact-FBF default and still
fails after completed step 35 on the 56-contact group at 200 iterations and
residual `4.0845024466967225e-4`: 103 attempts, 102 solves, one failure, zero
accepted caps, and zero fallback. Successful-iteration sums are all divisible
by five. Its stock comparator is ancestor-bound to `844c9c3`, not fresh c95,
and the sidecar hashes do not cover the patched math header. This establishes
only that the candidate does not clear the strict prefix; stock deltas are
context-only. Two cadence tests pass, but two legacy-default tests fail, so the
global-default patch must not ship. It supplies no trajectory, outcome,
source/backend/float32, timing, performance, superiority, video, Figure 6, or
paper-parity evidence. Verified `/tmp/fbf_fig06_residual_cadence_c95.0QXC5c/`
`RESULTS.md` and `SHA256SUMS` hash to
`1f57c569f7feacb2c681cb17a70743782f07822abcbc1eb13d7822d81e9df18f` and
`69db5e8915fadc31aae34d94c5f484928841b286566e172eea9535ee262d7645`.

The c95 terminal spectral-estimate A/B uses one instrumented Release binary
and changes only stock `rayleigh11` to `last_norm10`. The control exactly
reproduces the completed-step-35 / attempt-101 / 56-contact gate, while the
single recorded candidate satisfies all 103 ten-product/no-Rayleigh trace
invariants but still fails there. Residuals first diverge at step 29; recorded
contact-frame/reduced-state hashes and product-norm sequences first diverge at
step 30. The reduced-state hash covers only contact count, `freeVelocity`, and
coefficients; `product_norms` stores norms only. These summaries do not cover
`W`, operator identity, the initial reaction, the complete reduced problem, or
product vectors, so final-state deltas are contextual. This rejects only
`last_norm10` as the strict-prefix fix; it supplies no source-estimator,
trajectory, outcome, timing, performance, superiority, video, Figure 6, or
paper-parity evidence. The sealed marker/timeline/trace triplets are internally
consistent with the preregistered protocol, and the guard refuses output-path
reuse, but neither is external proof of no discarded run. Verified
`/tmp/fbf_fig06_spectral_terminal_c95.OjNIB4/evidence/`
`RESULTS.md`, `comparison.json`, and `SHA256SUMS` hash to
`e33894ab0b771544209d48724641716c491b04073ec5bec533c07df653e54cda`,
`8b7af123ccaa42fd9c6bbeb0916c5b691ed3234c428ae62e404e6f26449227f6`, and
`f18efba2ffb1f7f8ee0f88798c9bcd38103b571210949de5c0cc625fed3fd553`.

The sixth bounded c95 diagnostic uses the same instrumented Release binary in
both arms and changes only the initial-vector selector from stock `ones64` to
`rs42_f32_values_dart_norm64`. Both arms retain `rayleigh11`, DART
`[n,t1,t2]` order, float64 Eigen normalization/products, ten configured
products plus the terminal Rayleigh product, and the frozen strict contract.
The variant promotes a raw NumPy-2.4.4 `RandomState(42)` float32 prefix to
double before DART normalization; its registered 168-value prefix has SHA-256
`7506d5e093b6e3787fccb4c91aee3a26feffd8548637a9a76825ad1a9f3ccfe1`
and aborts above that dimension. The control exactly reproduces step 35 /
attempt 101 / 56 contacts / 200 iterations, residual
`4.0844850280896461e-4`, 103 attempts, 102 solves, and one failure. The sole
variant also fails there, with residual `4.1638905763175730e-4` and best
residual `4.1593800452634807e-4` at iteration 199.

Seed/product-norm/retained-estimate telemetry differs from attempt 1. Residual
and iteration count first differ at attempt 57 / step 29; contact-frame and
recorded reduced-state hashes first differ at attempt 67 / step 30. The
reduced-state hash omits `W` and the complete solver input, while
`product_norms` omits product vectors, so all post-divergence deltas are
contextual. The valid verdict rejects only the source-derived raw float32
values, promoted to double and normalized by DART's unchanged float64 path, as
sufficient for the frozen 36-step gate. It supplies no source-estimator or
coordinate-order parity, general root cause, longer trajectory, outcome,
timing, performance, superiority, video, Figure 6, or paper-parity evidence.
The one-shot artifacts cannot externally prove no discarded invocation.
Verified `/tmp/fbf_fig06_source_seed_c95.Uemp3S/evidence/` `RESULTS.md`,
`comparison.json`, and `SHA256SUMS` hash to
`07b2f08f55bcb0210149e441c1886601d2a1f1d60d4f094b53f475ceaec88da3`,
`8897b3d826789baaba11ec9c1fea47569f108f82937b41978445f51aad028aeb`, and
`b2ecc0cf5c84a58448b8a1eafbb03ecda05e4f9935be193d3cd79ded87676a41`.

The separate
`fbf_author_card_house_4_impact_source_continuation_current_source` capture
completes both exact and boxed for 2,400/2,400 steps on the same scene, clock,
and successful step-1,600 release schedule. Exact records 2,605 ordinary
successes, 113 plateau accepts, 633 max-iteration accepts, zero shrink caps,
and zero failures/fallbacks. The 746 accepts are 22.262% of 3,351 solves and
occur across 723 steps. Worst final residual is `0.91712002943322535`, first
reached at step 2,101. Independent inspection finds both houses standing
through release, but exact and boxed are identical only at step 0; viewport
difference is 0.165% at step 1,600 and 11.985% at the endpoint. Exact visibly
retains more upright structure after impact. The official MuJoCo panel degrades
while settling, whereas DART boxed remains upright until impact, so do not map
DART lanes to paper lanes or infer a mechanism. This is a qualitative
continuation result only. It does not provide
strict convergence, a quantitative trajectory/outcome comparison, approved
golden, source-backend or timing equivalence, Fig. 6/historical/paper parity, or
solver superiority.

The ignored durable capture root is
`assets/paper_evidence/fig06_card_house_source_continuation_current_v1/`,
resealed from `/tmp/fbf_fig6_source_continuation_pair_20260721T1414_v2/`.
Summary, exact timeline, boxed timeline, and paired-clip SHA-256 values are
respectively
`6888f4729c99d41753c9c8ec9a1ec2ec9e2367c71da76aab973f8f8c5e8674cc`,
`a9eb12711419b7801037d17059560559893be2898e07d14425a5f572175482ff`,
`1618e284f97ff7ed49e3288636269f5bea6131faa3bae45428e42e23de660bd8`,
and `282aebfb9e2e38fe3741db28e2ce909fb548d7aa46d048302a3b0e0bea9e1786`.
The official-video refresh matches its audited SHA-256
`d5356e1b31487be62b75af05efbfecdb70ad5d98501a8efd378fcedf066e4794`.
The DART clip remains outside Git and has no PR user-attachment URL. See
[`FIGURE6_CONVERGENCE_BLOCKER.md`](FIGURE6_CONVERGENCE_BLOCKER.md). This remains
a strict-adapter blocker plus separately labeled continuation-evidence lane.
The older `fbf_paper_card_house_26` scene is a separate reconstruction and must
not be merged into this claim.

The locally finalized Fig. 1/2 and video.03 bundle is
`assets/paper_evidence/fig01_02_incline_current_v1/`. With the compact local
bundle present, finalization and verify-only pass with status
`valid_current_source_nonpaper_incline`; the directory contains 23 physical
files and the exact-membership index binds 21 artifacts. The combined 660x506
capture retains five selected local stills and a 61-frame decoded 30 fps
schedule. Its 70-file raw capture staging directory is pruned after local
sealing, so verify-only does not require that raw staging directory. It records
240 exact attempts/solves,
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

The locally finalized Fig. 3/video.02 bundle binds 18 indexed artifacts in a
20-file physical directory. The MP4/GIF preserve the motion schedule, three
selected local stills retain steps 0, 1, and 2, and the 140-file raw capture
staging directory is pruned after local sealing. With the compact local bundle
present, verify-only does not require that raw staging directory.
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

The locally finalized Fig. 4 bundle is
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
Four selected local, timeline-bound outcome stills cover steps 136, 120, 360,
and 90 in source order. With the compact local bundle present, verify-only does
not require the pruned raw capture staging.

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

The locally finalized bundle at
`assets/paper_evidence/fig07_arch25_literal/` is valid current-source evidence
for the non-paper, no-projectile literal-wedge standing reconstruction. Its
five selected local 1280x720 stills and 61-frame H.264 schedule
fully decode; 600 independently generated capture/trace rows have zero
differences. Manual inspection covers the selected stills, timeline, and a
separately decoded midpoint. The retained pending hash DAG passed before final
writes, and the final artifact index validates 19 artifacts in a 21-file
physical directory. The 70-file raw capture staging directory is pruned after
local sealing. With the compact local bundle present, verify-only does not
require that raw staging directory. Ten simulated
seconds play in 6.1 seconds at 10 fps (`1.639344x` time-lapse), so the clip is
not a real-time playback result. Core hashes are metadata
`b217b0a43200ca558f09776e464d50f0f9dc5c5a175bcfa66ae1b656076871d1`,
index `4d173c78e2c6584d6d8a85867160cc0c0687fe0f35e6878f66b0cfcd7c1c15ee`,
video `e0c111f96511bbda41af145787bd9f68782b8c9e8a909d45eb03d3c3d647feb1`,
and timeline
`926380c5c5768cc9ca1e414cbfe2631584daa99387090926d5a1f035d62b6ec9`.
This closes standing-media integrity only, not the paper crown impact or parity
gate.

The capture source, capture binary, trajectory, selected local stills, and media remained
immutable through later additive scenario work. Fresh revalidation against the
final current trace and Native source regenerated reference trace
`0e41af230fb4c27143966c3f8522702280e6746b31e84155a9a047326e46dbb3`
and again validated 600 zero-difference standing rows. The current trace
source/binary hashes are
`b00eea0c87f75f17259fac433bafd98c20a961d14389a80e91742d3c8a678f76`
and
`0923bf7df1eaa518f9a2ffacd0a42fd7330f5cbf6c35cb5749fbc122db1310ff`.

The locally sealed impact bundle is
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

The source-pinned 101-stone DART packet at
`assets/paper_evidence/fig08_arch101_author_current_v1/` binds the public
`--stones 101` selection, 101-mesh tree
`e0c209235673d2f69c3c5de7708ab1dfadec96e3`, and 400-frame /
1,600-substep source-supported no-release schedule. Strict exact fails closed
after step 209 on an iteration cap with 208 contacts, residual
`1.2582804496066107e-6`, 342/342 attempts/solves, one accepted cap, zero exact
failures, and zero boxed fallbacks. Its incomplete trace fails the standing
oracle. Boxed completes all 1,600 steps with a valid inventory/finite/cube-
pinning trace but fails standing: maximum displacement `21.2188459736`, maximum
rotation `3.14152663339 rad`, and crown minimum/final `58.3806809854` /
`61.1013192467`.

The fully decoded boxed-collapse clip SHA-256 is
`7635c2722b20fb8bcb0255054cc9172153d1dd640fd8e81df4df52c0e515d3c0`.
Manual inspection shows crown loss by step 800 and visible collapse by steps
1,200/1,600. The diagnostic hstack freezes exact at its final step-208 frame
while boxed continues; SHA-256
`d6f5f658e4fb027edb23e0911acd34b74dfd749daace41b5d9c9204af3163b94`.
Capture and independent boxed reuse verification pass; compact summary
SHA-256 is
`1c19c6c3c36171a5e85f330b2863b429956652fb894aae0aa0b82d68291e3481`.
This closes only a current-DART blocker-media slice. It does not supply a
complete exact comparison, source backend/trajectory/outcome equivalence,
standing, a historical source/Kamino golden, timing, performance, Fig. 8/video.08 parity,
or solver-superiority evidence. Both Figure 8 GitHub media URLs remain pending
manual browser-composer upload.

The current literal 101-stone v1 artifact is
`assets/paper_evidence/fig08_arch101_literal_v1_negative_final_v7/`. It fails
closed on step 1 after 5,000 outer iterations with `fbf_failed`, residual
`0.7815364614352474`, one exact failure, and zero fallbacks. The dynamic
`FourPointPlanar` row reports 400 contacts, 100 constraint pairs, three colors,
and width 34. The separate repeat-2 Compact collision probe proves only the
constructed time-zero graph of 100 adjacent-stone pairs plus two
springer-ground pairs. The v7 one-step FourPointPlanar companion resolves the
failed step-1 pre-solve graph as exactly 100 adjacent pairs and 400 contacts,
four contacts per pair, with zero non-adjacent and zero ground pairs. Its
aggregates and residual match the frozen trace. The companion accepts the
capped iterate and does not follow the trace participant-affinity contract, so
solver-taxonomy and affinity equivalence remain false. This identity result is
limited to the failed prefix and supports no source equivalence, valid
trajectory, standing/physical outcome, timing, media, long-run behavior, or
paper parity. Raw, invocation, summary, metadata, and report hashes
are
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

The historical Painleve proxy subset also has a distinct locally finalized bundle at
`assets/paper_evidence/fig05_painleve_proxy_current_v1`. Its synchronized
1320x530, 30 fps, 76-frame clip has SHA-256
`dd4cdda2410b71f8e86035f1c0ff278f9dda77133c702e409ee5d533da443a4b`.
Manual inspection records `mu=.5` returning upright and `mu=.55` tumbling and
remaining visually horizontal. Separate tracked traces corroborate the local
classifier and the 0.03836187995520213 m shorter pre-tumble distance. They do
not establish trace equivalence, pinned-source geometry/configuration
equivalence, external-solver parity, an approved source golden/diff,
historical paper timing, or real-time performance, and it does not complete the
new source-pinned Figure 5 lane or supersede its independently validated
current-DART-adapter evidence.

## Arch Evidence Tiers

| Tier | Geometry and inertia | Observed contacts | Permitted claim |
| --- | --- | --- | --- |
| Production, 25 stones | Weighted-catenary oriented boxes | Natural 96; GUI reduced 48 | DART reconstructed box fixture |
| Production, 101 stones | Weighted-catenary oriented boxes | Full profile reaches cap 512; GUI reduced 38 | DART reconstructed capped box fixture |
| Literal 25-stone collision audit | Prism/wedge collision shapes with exact uniform-prism inertia | 24 nominal pairs; 26 with closure/ground | Collision graph/inertia audit only |
| Literal 25-stone exact dynamics | Exact-inertia wedges, `1 um` closure, Native `FourPointPlanar` | 96 contacts, 24 pairs/manifolds, 3 colors, width 8 throughout 600 steps | Non-paper standing/outcome, validated standing media, and matched 1-to-4-thread evidence only |
| Literal crown-impact v1 | Same standing arch plus three frozen projectiles; numeric only | Contact-order and finite-state gates pass; 5 caps, residual, all-body, and far-field gates fail | Preregistered non-paper scientific negative only |
| Pinned-author 25-stone diagnostic | Author meshes and Warp/Newton float32 pipeline; 500 frames / 2,000 substeps | Release substep has 100 contacts; 40 initial-shortcut and 117 configured-outer-gate convergences, plus 1,843 nonconverged outer solves; no contact-pair identities | Current-source author scientific negative only; no DART dynamics, trajectory, outcome, timing, repeatability, media, or paper-invocation parity |
| Source-pinned 101-stone DART adapter | All 101 author meshes, Native `FourPointPlanar`, float64 DART; 400 frames / 1,600 no-release substeps | Exact stops after step 209 on an iteration cap at residual `1.2582804496e-6`; boxed completes but fails standing and collapses; full current-source FBF/Kamino controls also fail standing | Current-DART/current-source scientific negatives and blocker media only; no historical source/backend/full-pose outcome, golden media, timing, or Fig. 8 parity |
| Literal 101-stone v1 | Exact-inertia wedges, `1 um` closure, Native `FourPointPlanar`; current v7 numeric bundle only | Failed step 1: identity-resolved 100-edge adjacent chain, 400 contacts/100 pairs, multiplicity 4, zero non-adjacent/ground pairs; 5,000 outers then `fbf_failed`, residual `0.7815364614`; companion taxonomy/affinity are not equivalent | Frozen non-paper failed-prefix scientific negative only; no source-equivalence, standing, physical, timing, media, long-run, or parity claim |

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

The last recorded focused gates report 56/56 exact-math tests, 38/38 exact
constraint-solver tests, 66/66 `ConstraintSolver` tests, 50/50 Native-collision
tests, 13/13 `SplitImpulse` tests, 3/3 masonry-wedge-dynamics tests, 39 passing
paper fixtures with 3 skipped,
9/9 focused CTests in both Release and Debug, 8/8 author-masonry adapter tests,
5/5 author-incline production-world contract tests, 359/359 visual-runner tests,
32/32 demo scene-doc contracts, 64/64 author-incline finalizer
tests, 859/859 focused manifest/backspin/incline/author-masonry/author-incline
evidence tests, and 1,555/1,555 full
no-cache dartpy Python tests. With the compact local incline bundle present,
verify-only also succeeds with 21 indexed artifacts and no raw
capture-staging dependency, its
focused finalizer suite passes 62/62, and 1,000
consecutive successful colored dispatch/barrier
stress runs. The prior-source strict full-card failure and current-source v2
sensitivity result remain retained negative/non-strict evidence.

The 118 live host-identity rechecks passed under the sealed producer closure,
where `libdart.so.6.19` resolved to the recorded `libdart.so.6.19.3`. The normal
development symlink is restored to `libdart.so.6.19.4`; `.3` and `.4` are
byte-identical, but the validator intentionally treats the resolved path as
identity and therefore reports five live path mismatches until that historical
closure is explicitly recreated. Archive mode remains the clean validation
path for the sealed local evidence.

PR #3374 is merged at final head `1f816` and merge commit `fa17fad` (abbreviated
hashes). The last live audit before five-level checkpoint `0e3937e6294` found
PR #3377 open and draft at remote head `c5765b37a08`, targeting
`6a1d377f616`. The five-level implementation/media is bound to
`0e3937e6294`; the older Figures 1-5 reseal remains separately bound to
`c95bd5fb916`. Target ancestry, topic head, divergence, merge state, checks,
and reviews are intentionally not frozen into this matrix; fetch and query
them live before reporting or publishing. PR #3377 remains work in progress
and is not completion evidence.
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
