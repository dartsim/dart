# PR Report: Exact Coulomb FBF Reconstruction

This is the PR-facing status report for the DART 6.20 exact-Coulomb FBF work.
It states what the current branch demonstrates and, just as importantly, what
it does not demonstrate. The older Figures 1-5 reseal is bound to
`c95bd5fb916`; the source-default five-level implementation/media is bound to
checkpoint `0e3937e6294` and demo SHA-256
`74d989f2419734c1767d60fedf7961935e78fbf42ed33f69b68d71699a9b4067`.
That is historical capture identity, not the current build: the metadata's
recorded mutable build path now resolves to a later binary, and live
current-head reuse also fails the source-hash gate. Reuse validation remains
pending a stable capture-binary archive or a current-head recapture/reseal.
Current `source_binding` hashes the whole monolithic
`FbfPaperFrictionScene.cpp`, so unrelated UI or text edits stale every
author-scene reuse check. Before the next bulk media reseal, split per-scene
semantic physics provenance from the broader UI/source hash. For the five-level
lane, archive and rebind the exact `0e3937e6294` binary or recapture at current
head, then rerun exact, boxed, and group reuse verification before calling
current reuse green.
The live PR body/head/check state must still be queried and synchronized after
each push.

## Direct Coverage Answer

No, this branch does not reproduce every test, benchmark, GUI example,
physical outcome, and performance result from the paper.

The implementation is active and partial. It now has locally validated,
hash-bound evidence for mean-real-time throughput and multicore acceleration on one explicitly
non-paper reconstructed literal-wedge workload. Do not broaden that result to
paper parity, an every-step real-time guarantee, the paper's timing rows, or a
claim that DART beats the paper.

The manifest remains 29 requirements: 24 partial, 5 blocked, and 0 complete.
The local visual inventory has six locally finalized bundles, and the visual
workflow now declares 33 runnable schedules, 30 of which encode MP4. The added
schedule is a DART-only numeric diagnostic, not a required paper or video row.
The workflow also includes separately named strict
and source-continuation four-level, source-default five-level, and ten-level
card-house schedules. The four-level continuation schedule has validated local
exact/boxed media. The five-level source-default and ten-level continuation
schedules each have independently verified 3,200-step exact and boxed members
plus decoded policy-asymmetric presentation-only pairs. None increases the
six-bundle inventory.

The current topic tip tracks zero files under
`docs/dev_tasks/fbf_exact_coulomb_friction/assets/`, but the topic history still
contains 493 unique task-asset blob IDs: 492 topic-added nonempty blobs
totaling 96,573,227 bytes plus the shared empty blob already reachable from the
base. PR #3377 must use a true squash merge so the 492 topic-added blobs do not
enter `release-6.20`, unless the topic history is explicitly rewritten first.
A merge commit or rebase merge does not satisfy this history-size gate.

A fresh pre-push audit against `6a1d377f616` classifies a 260-path intended
base-to-tip/worktree union: it finds zero changed or tracked task-asset paths
and zero generated-capture/log/media suffix matches. The only three visible
untracked paths are the intended new backspin specification, finalizer, and
finalizer test; they become ordinary tracked source in this checkpoint. The
intentional P3 checker PPM is a runtime input, not generated evidence. The
role-based `docs/dev_tasks/.gitignore` rule keeps all 13,480 present generated
evidence files totaling 999,659,711 bytes out of Git. The current remote PR
snapshot contains 257 changed paths, zero task-asset paths, and zero media/raw-
evidence suffix matches. Generators must keep writing task evidence below an
`assets/` output root;
global extension ignores would also hide intentional runtime inputs such as
the Figure 3 checker texture.

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

Checkpoint `34d9b66e97c` adds a distinct bounded source-continuation lane for
that same 2,000-step release diagnostic. Its exact/boxed capture and
independent reuse verification pass, but this does not clear the strict
step-142 blocker or establish strict convergence, superiority, outcome,
timing, backend, or paper parity.

A distinct source-pinned 101-stone DART adapter now executes the public
`--stones 101`, 400-frame no-release schedule. Strict exact fails closed after
step 209 at residual `1.2582804496066107e-6`; boxed completes 1,600 steps but
fails the standing oracle and visibly collapses. The decoded boxed clip and a
frozen-prefix hstack are precise blocker evidence, not Fig. 8/video.08 parity,
a historical source/Kamino golden, or solver-superiority evidence. A separate
full current-source Kamino run is now a finite numeric standing negative, not
parity evidence.

The separate pinned-author incline sweep is also numeric current-source
scientific-negative/reference evidence. It covers all seven Figure 1 friction
values in the public FBF, MuJoCo, and Kamino runners, but it is not a matched
DART result, a historical paper run, a seventh visual bundle, or timing,
performance, golden, media, or parity evidence.

## Locally Validated Literal-Wedge CPU Evidence

The current locally sealed bundle is
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

The locally finalized current-source small `paper_cpu` bundle is
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
multicore claim. The locally finalized literal-wedge bundle above is different: its
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

- exact math: 56/56;
- exact constraint-solver tests: 38/38;
- `ConstraintSolver` integration: 66/66;
- Native collision: 50/50;
- `SplitImpulse`: 13/13;
- masonry-arch geometry: 3/3;
- paper fixtures: 36 passing, 3 explicit opt-in cases skipped;
- focused Release and Debug CTest matrices: 9/9 in each configuration;
- author-incline reference finalizer unit tests: 64/64; verify-only reports 37
  indexed artifacts and 39 physical files;
- focused manifest/backspin/incline/author-masonry/author-incline evidence
  suite: 859 passed post-merge in 163.95 s;
- full no-cache dartpy Python suite: 1,555 passed in 165.09 s; and
- colored dispatch/barrier stress: 1,000 consecutive successful runs.

The retained strict full-card failure above is scientific negative evidence,
not a green paper-parity gate.

The manifest validator now hashes local bundle artifacts, materializes the CPU,
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

### Seven-Cell DART Incline Adapter And Upload Candidate

The new `fbf_author_incline_sweep_current_source` demo and
`incline_author_sweep_current_source` evidence schedule bind the public-source
geometry and 1/60 s, 120-step clock to the operator-selected Figure 1 grid
`mu=.3,.4,.45,.5,.55,.6,.8`. The source runs the values independently; DART
shows them in simultaneous Y-translated labeled lanes. Exact and boxed use the
same DART float64 world and Native `FourPointPlanar` frontend, and exact boxed
fallback is disabled.

Both complete traces pass the supported/upright/in-lane/contact gates, with
the first three cells sliding and the last four sticking. Against the retained
current-source FBF terminal oracle, the maximum absolute displacement/velocity
deltas are `0.002426469449185232 m` / `0.0011201728594518558 m/s` for exact and
`0.0011521317667995284 m` / `0.00030012480388411203 m/s` for boxed, within the
preregistered `.01 m` / `.01 m/s` tolerances. The oracle binds raw
`sweep_results.json` SHA-256
`f5cc26d2b0ca542b2b98f7fe94a8e2f7f7c9b7cccb3d23c35234ebe45d0d9d12`,
canonical projection
`e8b3b5c93a543480bae5c2f50106ecc1b137f65337cc1e725ef8c840efdb8921`,
and `mu=.55` history
`c0aa2d65cbbee24447e7ece9aa97bf83da4cc666ccf16da7edd6874abc22422f`.
The source reference has 839/840 configured convergence flags; `mu=.55`, step
1 reaches the 200/200 cap, so `source_reference_strictly_converged=false`.

The ignored synchronized upload candidate is
`assets/pr_media_author_incline_final_candidate_v6/groups/incline_author_sweep_current_source__exact_vs_boxed/clip.mp4`.
Capture plus independent reuse verification pass. It is H.264/yuv420p,
2600x890, 61 frames at 30 fps over 2.033333 s, SHA-256
`a750350c7f210953bf3292f79faef2bdacb160c9652676a9f98695165357f723`;
the group panel and metadata SHA-256 values are
`1c49eddac9fe2959ea4475e29f68ced2e5ce779ae7261b03f5fb3b8f58a04e42`
and `1e5bc5293d5c83a72075a2884c6fdf7ef17e5b8795b55da8028c9bc7f2602505`.
The copied demo binary, capture summary, and independent verification summary
SHA-256 values are
`67d399eee85ffd286984a877b8f4181b9ce3030acf5f9b2bc03886e54e7a5f20`,
`243ba16ef500fc8d3bb71e1b264e0bd2e99dbd23257a3bb9cfa809a4fbeaacba`, and
`02305f4faeeb792198dc7e85cf4348ffa3f3d52a742fa4e63f8b01e52bd27b4c`.
Group metadata sets only the narrow
`automated_current_source_fbf_terminal_outcome_slice_validated=true` claim;
generic `automated_semantic_outcome_validated` remains false.
This is a current-source terminal/outcome-slice result only. It establishes no
source trajectory/backend/solver/full-physical equivalence, source or paper
video/golden match, timing comparability, paper parity, or superiority. The
MP4 remains ignored and must be attached through the GitHub PR browser
composer; no attachment URL is recorded yet.

## Visual Evidence

The retained Figures 1-2 and 4-5 upload source is the ignored c95-bound reseal
`assets/pr_media_current_head_c95_small_rows/`; Figure 3 uses the stronger
source-pinned author-backspin capture documented below. The c95
reseal was produced from committed implementation head `c95bd5fb916`. Demo
and runner SHA-256 values are
`5725672a0305fb6e2d824533f7e28b2a779074cc88e7f24ffa164c14cdb78149`
and `89f76166954d0d8cb85996980c7235b6def387dd8994de3ef001f56b99744116`.
The capture summary passes with 20 members, 13 groups, zero failures, and five
expected boxed skips for the exact-only author-turntable schedules/group;
summary SHA-256 is
`8f227ab567c4d4b3a871cdaf29336e40b3ebb6732aa37cf401ce8d01025a18af`.
Independent reuse verification passes with the same 20 members, 13 groups,
and five expected skips; `/tmp/fbf_small_rows_c95_verify.json` has SHA-256
`264ac6ebdb461c99218070571900ee0b49e1a0925ffdb8101fdcf86f117b5f1e`.
Every member and group records `actual_simulator=true`,
`generated_imagery=false`, `paper_comparable=false`, and
`automated_semantic_outcome_validated=false`.
The superseded c95 temporal panels confirm the reconstructed Figure 3 checker
ball rotates in both lanes; the preferred source-pinned capture below binds the
checker resources and signed state trace. Painleve `mu=.55` remains exact-tumbled versus
boxed-upright. The c95 turntable group hashes are byte-identical to previously
audited clips/timelines, which retain the narrow source-order
ejected/ejected/retained/ejected and proxy classifications. These are narrow current-DART observations, not paper
parity or general solver-superiority evidence. No MP4 is tracked; browser-only
PR upload and resulting user-attachment URLs remain pending.

The consolidated browser handoff in `PAPER_DEMO_VIDEO_MATRIX.md` selects 16
recommended clips: nine minimum one-per-source-row uploads and seven
supplemental direct comparisons. An independent audit verified every local
path and SHA-256, H.264/yuv420p at 30 fps, and a complete
`ffmpeg -xerror` decode. This establishes upload readiness only. Figure 4's
author group remains exact-only; Figures 6/7/Tables 6-7 remain continuation
evidence; Figure 8 remains a frozen-prefix diagnostic; and the Figure 4 proxy
pairs are reconstructed. The Tables 6-7 pair includes a continuation-policy
difference and is presentation-only.

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

The incline subset has now been independently recaptured and locally finalized
at `assets/paper_evidence/fig01_02_incline_current_v1`. With that compact local
bundle present, finalization and verify-only pass with status
`valid_current_source_nonpaper_incline`. The directory has 23 physical files
and an exact-membership index of 21 artifacts. The combined 660x506 capture
retains five selected local stills and a 61-frame decoded 30 fps clip schedule.
Its 70-file raw capture staging directory is pruned after local sealing, so
verify-only does not require that raw staging directory. It records 240 exact
attempts/solves, zero caps,
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

The north-star Figure 5 lane is now the source-pinned
`painleve_author_mu05` / `painleve_author_mu055` exact/boxed adapter. Its
ignored durable bundle at
`docs/dev_tasks/fbf_exact_coulomb_friction/assets/paper_evidence/fig05_painleve_author_current_v1/`
passes capture summary and independent verify with four member results, four
groups, complete 121-sample traces, 61-frame H.264/yuv420p member/composite
clips, full decode, and manually audited panels/keyframes. At `mu=.5`, exact
and boxed classify `upright_near_rest` after `1.5986787381 m` and
`1.5977005918 m`; at `mu=.55`, exact classifies `tumbled_near_rest` after
`1.5399225956 m` while boxed remains `upright_near_rest` after
`1.6623056217 m`. Exact `mu=.5` records 119 attempts/solves, zero
failures/fallbacks, final residual `5.2255077e-7`, and worst
`9.7391465e-7`; exact `mu=.55` records 108 attempts/solves, zero
failures/fallbacks, final `9.1964345e-7`, and worst `9.9977460e-7`. The
adapter binds the exact-options header hash.

The defensible claim is exactly: under the pinned current DART adapter, exact
and boxed lanes diverge at `mu=.55`. Source-backend equivalence, trajectory
equivalence, paper Figure 5 parity, timing comparability, and solver
superiority remain false. The two exact-vs-boxed candidates are
`groups/painleve_author_mu05__exact_vs_boxed/clip.mp4` (SHA-256
`77d3286dde96785a6c36cd901e92f183409098ba2bd8dbb426489f537fe71209`)
and `groups/painleve_author_mu055__exact_vs_boxed/clip.mp4` (SHA-256
`2c71e565559dea513870b56bba3c709cf015707b171cfdb45b5cf64fde31f70f`).
Their GitHub attachment URLs remain pending manual browser-composer upload;
the media stays outside Git.

The historical Painleve proxy subset has since been independently recaptured
and locally finalized at
`assets/paper_evidence/fig05_painleve_proxy_current_v1`. Its
local-only paired clip is 1320x530 at 30 fps for 76 frames with SHA-256
`dd4cdda2410b71f8e86035f1c0ff278f9dda77133c702e409ee5d533da443a4b`.
Both exact-FBF proxy member clips, hardened sidecars, manual inspection, and
separate 151-row physical traces are bound in a 27-artifact index within a
29-file physical directory. With the compact local bundle present, verify-only
passes without the pruned raw capture frames. This proves
only the local proxy presentation: `mu=.5` returns upright, while `mu=.55`
crosses the tracked fixture's tumble threshold after 0.03836187995520213 m less
pre-tumble travel and remains visually horizontal. The rendered demos and
tracked fixtures are separate implementations, so this is not trace
equivalence, author-scene parity, external-solver parity, an approved source
golden/diff, paper timing, or real-time evidence.

### Source-Pinned Author Figure 3 Backspin

The preferred PR media is now the ignored
`assets/pr_media_author_backspin_v2/` capture of
`fbf_author_backspin_current_source` under schedule
`backspin_author_current_source`, not the reconstructed c95 bundle below. It
binds author commit `b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0`, the sealed
source-reference manifest SHA-256
`7d4248f6431a902571b569b3477e61b4fa8ad0897f6c957e10a223cf32eb0b98`,
the DART implementation, exact options, explicit ground/sphere primary and
secondary friction plus restitution, process state, and the checker OBJ/MTL/PPM
before and after capture. The source's configured terminal flag is true for all
240 steps, but 183 projected natural residuals exceed `1e-6`; this is not
strict natural-residual convergence. Capture and independent reuse verification
pass for two members and one group; their summary SHA-256 values are
`f0780db1a420f799f6850395711342fda5406e1f760218334488fa119e21468f` and
`390ec386f51cd6a8081b5494e05039efd96beda4571782f9c9cb8670b5005171`.

Exact and boxed each complete 240 steps / 241 states and pass prompt contact,
contiguous support through step 206, a five-sample rolling tail, zero measured
off-axis motion, left-edge roll-off, airborne terminal state, and the
preregistered source-terminal tolerances. Exact records 205/205 solves, no
accepted caps, failures, or fallbacks, and worst residual
`9.990141261260073e-7`. Exact terminal `vx/wy/slip/z` is
`-11.4796920141/-46.2074988932/0.0721827092/-1.3729371427`; boxed is
`-11.4821219179/-46.2486366906/0.0800372547/-1.3731065400`.

Manual inspection confirms visible checker-facet orientation changes in
consecutive frames and the temporal panel. The exact 121-frame H.264/yuv420p
clip is the preferred upload at SHA-256
`b2c268aa337f8d4e753408c1bbf17ca29dc4300597b64782fcb7344f6c676b30`;
the labeled exact/boxed group is supplemental at SHA-256
`e321c711eae7daf8e2a289df71f4d08c0d813d6c84e204c0930594d4a561e15b`.
The imagery does not prove signed direction/rate because 30 fps aliases the
initial `-200 rad/s`; the trace owns signed `wy`. Both solvers pass, so no
solver equivalence/superiority, source backend/full-trajectory or video
equivalence, timing/real-time evidence, historical Figure 3 invocation, or
paper parity follows. Browser upload and GitHub attachment URLs remain pending.

### Historical Reconstructed Figure 3 Backspin

The historical backspin subset was independently recaptured and locally
finalized at
`assets/paper_evidence/fig03_backspin_current_v3`. Its exact-membership index
binds 18 artifacts within a 20-file physical directory. The MP4/GIF preserve
the full motion schedule, three selected local stills retain steps 0, 1, and 2,
and the 140-file raw capture staging directory is pruned after local sealing.
With the compact local bundle present, verify-only does not require that raw
staging directory. The capture records 129
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
`video.02_backspin` remain `partial`. It is superseded as the preferred PR
upload by the source-pinned author lane above.

### Finalized Author-Pinned Fig. 4 Turntable Bundle

The locally finalized bundle is
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
Four selected local, timeline-bound outcome stills cover steps 136, 120, 360,
and 90 in source order. With the compact local bundle present, verify-only does
not require the pruned raw capture staging.

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
The reseal-time visual runner/test bindings are
`d848afa53caf14b9fb3ea061d658eef274e8d917151937bd6340283b79ab5432`
and `6e378252fa6a7cb51c6813c9d5a2b30b8c8129eacdb01df0ee19a58a270cbc5e`.
This is author-source-pinned finite-horizon non-paper evidence, not paper,
approved-golden, timing, or real-time parity.

### Finalized Author Card-House Construction Bundle

The locally finalized bundle is
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

### Supplemental Source-Default Five-Level Card-House Diagnostic

The separate `fbf_author_card_house_5_impact_current_source` and
`fbf_author_card_house_5_impact_source_continuation_current_source` scenes,
with matching capture schedules that omit the `fbf_` prefix, bind the current
public no-argument five-level default: 40 cards, four initially kinematic
cubes, 800 display frames / 3,200 DART substeps, and cube release after
completed step 1,600. Both use `dt=1/240 s`, Native `FourPointPlanar`, 4,096
contact capacity, four contacts per pair, one `0.1 m` ground gap, and 44
`0.005 m` dynamic-shape gaps. This is a non-paper source-default diagnostic,
not the source-selected four-level Figure 6 row, a recovered historical paper
invocation, or the ten-level Tables 6-7 row.

The strict exact lane fails closed after completed step 31, before release, on
a 39-contact group at 200 iterations. Its final residual is
`9.022404720646783e-6` against the `1e-6` tolerance; 248 attempts produce 247
solves and one failure, with zero accepted caps and zero boxed fallbacks. It
therefore supplies no strict success or complete strict trajectory.

The separately named continuation exact member and the boxed member each
complete 3,200/3,200 steps, capture 401 shots, and execute the step-1,600
release successfully. Exact records 7,337/7,337 attempts/solves, zero
failures/fallbacks, 2,245 plateau accepts, 836 maximum-iteration accepts, zero
shrink-cap accepts, 7,298 warm starts, and 303,900 total iterations. Its final
and worst residuals are `1.2757511844995566e-7` and
`0.6378480998790657`; maximum and final contact counts are 266 and 248. Exact
has 399 unique images because two settled shot pairs are duplicates; boxed has
401. Both 660x506 member clips are H.264/yuv420p, 30 fps, 401 frames,
13.366667 s, and pass full decode.

Independent exact and boxed reuse audits pass; their summary SHA-256 values
are `1b7c42c0836aa17fd55f952e6335167a5786d37e54236f9088fa5d1a6a1885fd`
and `078212b68ae07e234c94b2537d41158cd5c944492e930de7e1fc2b329f5a6453`.
Exact timeline/clip/metadata SHA-256 values are
`35c7fddedc2dbdb6f2b00323f19dcc6df98ac1e4188d246f06ef562ad44aea80`,
`956ca7c32fcc23501a863d0e7ec2668fe7ffe5986343a5dcaf49ba6886be8816`,
and `3afe8c7a3bc795827ff4318438b150d176699dc9dd55057bb48dd025684ffbdf`.
Boxed values are
`ba771601affe997b07a66ddc161f4a110ea4eced0b4d94773b219270b63a322f`,
`319747d1a24a8a735ab4b4485b44a39905e3f6d40fe00257f78ba9b9451fcaa7`,
and `f77af90deac3f740d06ba0bec2eca17d837cf83974b253c35f3b76dbca35113e`.
The boxed independent audit also finds every rendered foreground pixel at
least 40 px from the right edge and 30 px from the bottom edge; its endpoint
margins are about 41 px and 33 px.

The final ignored presentation candidate is
`assets/pr_media_card5_source_default_group_v3/card_house_author_5_impact_source_continuation_current_source__exact_plus_continuation_vs_boxed_no_continuation/clip.mp4`.
It labels the lanes `EXACT COULOMB FBF + SOURCE CONTINUATION` and
`EXISTING BOXED LCP (NO SOURCE CONTINUATION)`, making the policy asymmetry
explicit. The 1320x530 H.264/yuv420p clip has 401 frames at 30 fps over
13.366667 s and passes full decode. Clip, panel, and presentation-manifest
SHA-256 values are
`b46aeb3d9f09e95151e26fef4838432b6b071a5d3c39c3c9a489c6f1d42e875b`,
`484fdc35aed15ba06e253be63e5ff9bb46f88bc6c273150d8f78a646da0dc7f8`,
and `2c80a8cca4cb3a0a11f49a1747bb5cc90092f884e05f0b30d3959e8e2a3eb3cf`.
Manual inspection finds different pre-release and endpoint structure, but the
records deliberately set `automated_semantic_outcome_validated=false`.

This pair is not strict success, a solver-only A/B, solver superiority, an
automated semantic or physical outcome, a historical Tables 6-7 invocation,
source trajectory/backend/timing evidence, paper-video parity, or paper
parity. The v1 and v2 captures are superseded framing probes and excluded from
upload evidence. The v3 assets remain ignored, and the final clip has no
GitHub user-attachment URL until it is uploaded through the PR browser
composer.

### Current-Source Four-Level Figure 6 Adapter

The separate `fbf_author_card_house_4_impact_current_source` demo scene and
`card_house_author_4_impact_current_source` capture schedule pin author commit
`b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0`. They use the source-supported
selection `--solvers fbf --levels 4 --frames 600 --drop-frame 400
--num-cubes 4 --mu 0.8 --cube-size 0.4 --cube-density 500 --drop-height 1.0
--device cpu --profile --usd`, not the source's no-argument
five-level/800-frame default or a known historical paper command. Source
`ke=1e4`, `kd=1e3`, and `gap=.005` are recorded source semantics, not contact
semantics implemented equivalently by the DART adapter.

The adapter has 26 source-sized cards (20 leaning and 6 bridges) and four
initially kinematic `0.8 m`, `256 kg` cubes. Interactive `p` releases the cubes
immediately; the evidence runner invokes `p` after completed substep 1,600 in
the declared 2,400-step, `dt=1/240 s` run. Exact and boxed lanes share the same
Native `FourPointPlanar` frontend, 4,096-contact capacity, and manifold
subdivision 4.
The demo build, 13 focused headless/continuation C++ fixtures, 454 runner Python
tests, and both adapter-contract smoke validators pass.

The strict exact 100-step request fails closed at completed step 35 when the
contact count jumps from 44 to 68. Steps through 34 are clean, with prior worst
residual `9.826274595482653e-7`; the failed prefix records 103 attempts, 102
solves, one exact failure, zero fallbacks, zero accepted caps, and worst
residual `4.1039190451256334e-4`. Timeline SHA-256
`2d04d31134426ac2c4fc87b1774d5285b77740acaeb3ec3a005557b85944bb9d`
belongs to
`/tmp/fbf_author_card_house_4_exact100_last_failure_current_source_20260721/timeline.json`.

The pinned source defaults `project_after_correction=false`, unlike the
projected DART baseline. The current checkpoint adds an ABI-neutral,
default-on DART policy and disables it only for this source-selected exact
adapter. Strict 36- and 100-step replays still fail at completed step 35 with
identical diagnostics: 56 contacts, 200 iterations, residual/best/dual
`4.0845653576327421e-4`, primal `3.9380158679450451e-6`, complementarity
`2.3818176330330057e-4`, zero accepted caps, and zero fallback. Evidence:

- `/tmp/fbf_author_card_house_4_source_correction_exact36_20260721/timeline.json`,
  SHA-256 `686be7170e3c217bfa917698a449e7ecde40e500a2c87d073ed58ba2ac833bfb`;
- `/tmp/fbf_author_card_house_4_source_correction_exact100_20260721/timeline.json`,
  SHA-256 `1a76b71fc4558c7cb978eab410a95948ae50e66522e45dbded07dd36aeb11a77`.

The residual improves only `0.471590382%`, and the small primal violation is
new. This closes the post-correction mismatch but does not clear or move the
blocker.

The pinned author's block-GS solve also copies the current outer reaction into
every inner solve and rejected step-size trial without projecting that seed.
The current checkpoint adds an ABI-neutral, default-off source-inner policy and
enables it only for this adapter's exact lane; DART's carried, projected seed
remains the default elsewhere. With both source-selected policies active,
strict 36- and 100-step v3 replays again fail closed at completed step 35 with
byte-identical diagnostics: 56 contacts, 200 iterations, final/best residual
and dual `4.0844850280896461e-4`, primal `3.9375947649884479e-6`,
complementarity `2.3815426453852184e-4`, zero accepted caps, zero boxed
fallbacks, and zero line-search shrinks. Evidence:

- `/tmp/fbf_author_card_house_4_source_inner_exact36_v3_20260721/timeline.json`,
  SHA-256 `8909e915b63bb2c412a5c5289a5aa690dc1a9ef1d712fe531d12a38d626f0d2e`;
- `/tmp/fbf_author_card_house_4_source_inner_exact100_v3_20260721/timeline.json`,
  SHA-256 `3e379747bac636c259fe7e9bbd711bb57d5a719d5a1d8d6b9e6317e20b639f73`.

The Figure 6 adapter and strict replay are serial because colored block
Gauss-Seidel is disabled. A c95-bound one-factor probe changes only that
switch and executes the colored ordering/path for 200 solves with one
participant and zero parallel dispatches. It still fails the same step-35
group and improves residual by only `2.19e-14` relative. Reject it only as the
next Figure 6 blocker discriminator, not as a multicore or general colored-BGS
result. Source shrink-cap, plateau, and continuation semantics remain separate
and unchanged by this A/B.

The subsequent isolated c95-bound global-scope A/B also stops at completed
step 35. Stock native scope reproduces the 56/8/4-contact partition and
`4.0844850280896461e-4` failed residual. The global 68-contact solve reaches
`4.0848243204467147e-4`; slicing that solution under the native partition
localizes `4.0848243204472058e-4` to the same 56-contact island, with the 8-
and 4-contact islands converged. Off-block `W` coupling is exactly zero in all
35 generations. Both modes pass generation 28 from identical contact
fingerprints with different within-tolerance reactions, and fingerprints first
diverge at generation 29. This rejects native-island scope alone as the
step-35 cause; it does not prove global/per-island equivalence or any source,
trajectory, physical-outcome, performance, superiority, video, Figure 6, or
paper-parity claim. The isolated report and hash manifest are under
`/tmp/fbf_fig06_global_scope_c95.TSfONI/`; their SHA-256 values are
`633828adbe08577b6d0973ca817194530ed8a08cbe27e85d2bcb004689919fe9` and
`90d72452c6b3ed09e0bc1e408b56e70092557784fd2089e6895d7a31a0c809d3`.

A separate c95-bound one-factor source-gap diagnostic enables only the
four-level scenario's existing source-gap flag. Native then admits predictive
negative-depth contacts with `0.1 m` on the ground and `0.005 m` on all 30
dynamic shapes; the strict source-inner serial-BGS, no-projection,
no-cap-acceptance, no-fallback contract remains unchanged. The preregistered
36-step gate fails after completed step 31 on a 31-contact group at 200
iterations and residual `1.0006073317077885e-5`: 186 attempts, 185 solves,
one failure, zero accepted caps, and zero fallbacks.

The compared contact streams differ from step 1, and the existing stock
sidecar embeds ancestor `844c9c316195897cf2bf51f38eafc8ec9dcf959a` rather than a
fresh c95 binary. This rejects only the hypothesis that the DART gap
representation alone clears the strict prefix. It proves no general gap harm
or benefit, source-contact equivalence, trajectory, outcome, backend, float32,
timing, performance, superiority, video, Figure 6, or paper parity; keep the
checked-in scene unchanged. The verified package is
`/tmp/fbf_fig06_gap_c95.m6bsif/`: `RESULTS.md`
`3b0948c80871d19cbe29495a8abc57ac4f3e92dc518a9ae6551238a9aad9b17a`,
one-factor patch
`a5269addaae4bb2864ad2b9fa3768cc99bd41e22758af5950d78eae163ab6695`, and
`SHA256SUMS`
`11888f98a24175f50c09ce95509d754d0bbc1963e5d2294ad982ece280292119`.

A separate c95-bound candidate changes only the internal exact-FBF residual
cadence from `1` to the pinned source value `5`. Its single preregistered strict
run still stops after completed step 35 on the 56-contact group at 200
iterations and residual `4.0845024466967225e-4`. It records 103 attempts, 102
solves, one failure, 3,450 total outer iterations, zero accepted caps, and zero
fallback; every nonzero successful-iteration sum is divisible by five.

The copied stock sidecar maps to ancestor `844c9c3`, not a freshly built c95
binary, and neither sidecar source-binding hash covers the patched math header.
This proves only that the cadence-5 candidate does not clear the strict prefix;
stock deltas are contextual, not a same-revision/same-binary controlled A/B or
causal cadence estimate. The global-default patch must not ship: both cadence
tests pass, but the full math binary has 64 passes and two expected
legacy-default failures. No trajectory, outcome, backend, float32, timing,
performance, superiority, video, Figure 6, or paper-parity claim follows. The
verified package is `/tmp/fbf_fig06_residual_cadence_c95.0QXC5c/`:
`RESULTS.md`
`1f57c569f7feacb2c681cb17a70743782f07822abcbc1eb13d7822d81e9df18f`,
one-factor patch
`1c1346021972e563df3d81fa2ea77313eea33923b8c5b719afcd454ceebd5e86`, and
`SHA256SUMS`
`69db5e8915fadc31aae34d94c5f484928841b286566e172eea9535ee262d7645`.

A c95 terminal spectral-estimate A/B uses the same instrumented Release binary
for stock `rayleigh11` and candidate `last_norm10`; only the terminal estimate
selector changes. The control exactly reproduces completed step 35, exact
attempt 101, 56 contacts, residual `4.0844850280896461e-4`, safe gamma
`0.27728679157546576`, final gamma `2.7728679157546576`, 103 attempts, 102
solves, and one failure. The single recorded candidate satisfies all 103
ten-product/no-Rayleigh trace invariants but still fails at the same gate
coordinates with residual `4.07679549813362e-4`.

Residuals first diverge at attempt 57 / step 29. Recorded contact-frame and
reduced-state hashes plus ten product-norm sequences first diverge at attempt
67 / step 30, and iterations first diverge at attempt 77 / step 31. The raw
`reduced_problem_fnv1a64` field hashes only contact count, `freeVelocity`, and
coefficients; it omits `W`, the initial reaction, and the complete reduced
problem. `product_norms` stores norms only, so product vectors and an operator
digest are also unrecorded. Failure-state
residual/gamma deltas are therefore
contextual, not a same-problem local causal estimate. The valid controlled
verdict is only that `last_norm10` does not clear the strict prefix. The DART
seed, scalar type, coordinate order, and backend remain source mismatches, and
no trajectory, outcome, timing, performance, superiority, video, Figure 6, or
paper-parity claim follows. The sealed marker/timeline/trace triplets are
internally consistent with the protocol, and the guard refuses output-path
reuse, but neither externally proves no discarded run occurred.

Release build 350/350, both focused unit suites, `git diff --check`, the
comparison replay, and all package checksums pass. Verified package:
`/tmp/fbf_fig06_spectral_terminal_c95.OjNIB4/evidence/`; `RESULTS.md`
`e33894ab0b771544209d48724641716c491b04073ec5bec533c07df653e54cda`;
`comparison.json`
`8b7af123ccaa42fd9c6bbeb0916c5b691ed3234c428ae62e404e6f26449227f6`;
diagnostic patch
`0d79fe90284ff481463716ba0a0bfa87b11272bacd4f8a5fe0960c4835c64227`;
`SHA256SUMS`
`f18efba2ffb1f7f8ee0f88798c9bcd38103b571210949de5c0cc625fed3fd553`.

A sixth bounded four-level strict-prefix diagnostic uses the same c95
instrumented Release binary in both arms and changes only the initial-vector
selector from stock `ones64` to `rs42_f32_values_dart_norm64`. Both arms retain
`rayleigh11`, DART `[n,t1,t2]` order, float64 Eigen normalization and power
products, ten configured products plus the terminal Rayleigh product, and the
frozen scene and strict policies. The source variant promotes the raw
NumPy-2.4.4 `RandomState(42).randn(4096 * 3).astype(float32)` values to double
before unchanged DART normalization. Its registered 168-value raw prefix has
SHA-256
`7506d5e093b6e3787fccb4c91aee3a26feffd8548637a9a76825ad1a9f3ccfe1`
and aborts above that dimension.

The control exactly reproduces completed step 35, attempt 101, 56 contacts,
200 iterations, residual `4.0844850280896461e-4`, safe gamma
`0.27728679157546576`, final gamma `2.7728679157546576`, 103 attempts, 102
solves, and one failure. The sole recorded variant also fails at those gate
coordinates, with residual `4.1638905763175730e-4`, best residual
`4.1593800452634807e-4` at iteration 199, safe gamma
`0.26989166211867666`, and final gamma `2.6989166211867666`. Seed,
product-norm, and retained-estimate telemetry differs from attempt 1; residual
and iteration count first differ at attempt 57 / step 29, while contact-frame
and recorded reduced-state hashes first differ at attempt 67 / step 30.

The reduced-state hash covers contact count, `freeVelocity`, and coefficients,
but not `W` or the complete solver input; `product_norms` records norms, not
product vectors. Post-divergence residual and gamma deltas are therefore
contextual. The only supported verdict is that these source-derived raw
float32 values, promoted to double and normalized by the unchanged DART
float64 path, do not clear the frozen 36-step gate. This does not establish
source-estimator or coordinate-order parity, a general root cause, a longer
trajectory, Figure 6/video parity, timing, performance, or superiority. No
visual verdict applies. The one marker/timeline/trace triplet per arm and
path-reuse guard are internally consistent with the one-shot protocol, but do
not externally prove that no run was discarded. Release build 356/356, both
focused unit suites, both validators, independent replay, and
`git diff --check` pass. Verified package:
`/tmp/fbf_fig06_source_seed_c95.Uemp3S/evidence/`; `RESULTS.md`
`07b2f08f55bcb0210149e441c1886601d2a1f1d60d4f094b53f475ceaec88da3`;
`comparison.json`
`8897b3d826789baaba11ec9c1fea47569f108f82937b41978445f51aad028aeb`;
`SHA256SUMS`
`b2ecc0cf5c84a58448b8a1eafbb03ecda05e4f9935be193d3cd79ded87676a41`.

The pinned author control completes all 2,400 substeps but reports 1,455
converged and 945 unconverged flags: 632 caps and 313 plateaus. The pre-release
split is 1,332/268 and the release-and-after split is 123/677; first false and
first cap indices are 33 and 35. Worst natural `final_residual` is
`2.59804445965485`, and worst per-step final checked `r_coulomb` is
`7.597910320688573`. History:
`/tmp/fbf-sca-2026-author/paper_examples/card-house/results/20260721T175341Z/fbf/history.json`,
SHA-256 `b67d3c86f106171008dfbb0aca0a2ca72a9d3747c1a7a6694f57f211d3f83afd`.
Zero-cap completion is therefore a strict scientific gate, not
source-equivalent continuation semantics. The separately labeled telemetry-rich
continuation lane below now captures finite accepted iterates; it does not alter
the strict blocker.

The boxed control completes 100 steps. Its timeline is
`/tmp/fbf_author_card_house_4_boxed100_20260721_contract_v2/timeline.json`,
SHA-256 `fdd3d9e96058176faa51b148d1bcf5a4c0a7f1c4e7da64e15490dcae4ce6fafc`.
The additive `last_failure` sidecar record precisely identifies the failed
56-contact island after the later groups succeed: 200 iterations,
residual/dual `4.1039190451256334e-4`, complementarity
`2.4220067503580449e-4`, and worst dual/complementarity local contact 11.
Bounded option tuning did not produce a strict 100-step completion. See
[FIGURE6_CONVERGENCE_BLOCKER.md](FIGURE6_CONVERGENCE_BLOCKER.md).

An unsealed GDB-mutated accepted-cap preview reaches release and all 2,400
steps, but 1,106/3,231 solves cap and worst residual reaches
`0.61608914241359314`. This is finite continuation only. The strict lane still
does not reach release and boxed remains bounded to 100 steps. This does not
establish a valid strict trajectory, physical outcome, source-backend or timing
equivalence, final media or PR attachment, Fig. 6/paper parity, or solver
superiority. It remains an adapter-only lane. Any next strict A/B must isolate
one remaining source mismatch; colored ordering, one-global-group scope,
source-sized gaps, residual cadence, terminal spectral estimation, and source
seed values are now rejected as six bounded strict-prefix discriminators,
while
source shrink-cap or
continuation semantics remain separate work. The older reconstructed
`fbf_paper_card_house_26` lane remains distinct.

### Figure 6 Source-Continuation Attachment Candidate

The separately named
`fbf_author_card_house_4_impact_source_continuation_current_source` scene and
`card_house_author_4_impact_source_continuation_current_source` schedule use the
same 26-card/four-cube source-parameterized DART scene, Native
`FourPointPlanar` frontend, 2,400-step clock, and step-1,600 `p` action for exact
and boxed. Only exact requests the explicit continuation policy.

Both lanes complete 2,400/2,400 steps and the action succeeds. Exact records
3,351 attempts/solves, zero exact failures or boxed fallbacks, 2,605 ordinary
successes, 113 plateau accepts, 633 max-iteration accepts, and zero shrink caps.
The 746 accepted continuation outcomes are 22.262% of 3,351 solves and occur
across 723 steps. Worst final residual is `0.91712002943322535`, first reached
at step 2,101. This is source-continuation evidence, not strict convergence.

Independent media audit finds exact and boxed pixel-identical only at step 0.
Their viewports already differ by 0.165% at step 1,600, so this is not an
identical-state-at-impact comparison; endpoint divergence is 11.985%. In this
DART source-parameterized four-level scene, the exact source-continuation lane
completes without exact-solver failures/fallbacks and visibly retains more
upright card-house structure after impact than DART boxed LCP. Do not infer a
mechanism or solver superiority: the official MuJoCo panel degrades while
settling, whereas DART boxed remains upright until impact, so the DART lanes do
not map to the paper's solver lanes. There is no quantitative trajectory or
physical-outcome equivalence, approved golden, source-backend equivalence,
timing comparison, or paper parity.

The ignored durable root is
`assets/paper_evidence/fig06_card_house_source_continuation_current_v1/`,
resealed from `/tmp/fbf_fig6_source_continuation_pair_20260721T1414_v2/`.
Integrity anchors are:

- summary:
  `6888f4729c99d41753c9c8ec9a1ec2ec9e2367c71da76aab973f8f8c5e8674cc`;
- exact timeline:
  `a9eb12711419b7801037d17059560559893be2898e07d14425a5f572175482ff`;
- boxed timeline:
  `1618e284f97ff7ed49e3288636269f5bea6131faa3bae45428e42e23de660bd8`;
  and
- paired H.264/yuv420p clip:
  `282aebfb9e2e38fe3741db28e2ce909fb548d7aa46d048302a3b0e0bea9e1786`.

The fresh official-video download matches audited SHA-256
`d5356e1b31487be62b75af05efbfecdb70ad5d98501a8efd378fcedf066e4794`.
The paired clip remains outside Git and has no
`github.com/user-attachments/...` URL. It may be uploaded only as a narrowly
captioned continuation comparison; PR #3377 must remain draft.

### Source-Supported Ten-Level Card-House Current Status

The separate `fbf_author_card_house_10_impact_current_source` scene and
`card_house_author_10_impact_current_source` schedule bind the pinned public
source's supported `--levels 10` selection: 155 cards, four initially
kinematic cubes, 800 display frames / 3,200 DART substeps, and release after
completed step 1,600. This is neither the historical Tables 6-7 invocation nor
the older `fbf_paper_card_house_10_dynamic` reconstruction.

The pinned source inspection resolves one ground gap at `0.1 m` and 159
card/cube gaps at `0.005 m`. DART represents those values on all 160 Native
collision ShapeFrames, but does not claim equivalent Newton/Warp collision,
stiffness/damping, float32, or backend semantics. The pinned source's
30-display-frame control stays finite but reports 33 converged and 87
non-converged substeps, first false at source index 33, so it is not a strict
trajectory oracle.

The Release demo/test build, three focused C++ fixtures, both C++ lint gates,
and the then-current 331-test visual runner passed for pushed checkpoint
`ffe23d347b0`. That checkpoint retains the historical strict completed-step-1
failure: 264 total contacts, 18 exact attempts, 17 exact solves, one failure,
and a 39-contact group at residual `8.891154359157548e-6` after 200 iterations.
Boxed LCP completes the historical one-step control. Capacity 4,096 is not the
immediate blocker.

Historical exact and boxed timeline SHA-256 values are
`c0af3c2b03d38b68bd30374394bebb83286b08e91414641796f8ff58ec202bbf`
and
`059d8d8c21db86df9b8708cf0da9b8bd63e024f2a9723d5a491c0bee2d3e78b0`.
They remain valid historical checkpoint diagnostics.

Predictive checkpoint `3647959a188` is included in the current PR head. Its
speculative-contact correction matches the source's scalar
`separation / dt` velocity allowance narrowly while retaining physical
negative separation and zero penetration correction. Corrected exact now
clears completed step 1 with 18/18 exact solves and zero failures. A strict
40-step request then reaches completed step 31 before a 79-contact group fails
after 200 outer iterations at residual/best residual
`1.072805023427092e-5`; boxed completes all 40 steps.

Corrected local timeline paths and SHA-256 values are:

- `/tmp/card10-predictive-scoped-exact1.0VwT5s/timeline.json`:
  `bb1c352a3a2e35b7ee0796899dfbffb59358790fe8871cc8a936cbf62404066f`;
- `/tmp/card10-predictive-scoped-boxed1.Kye74m/timeline.json`:
  `8947284c4719212722a67ef920d9e5e60892ae1ecf5195f4312703cb2061fbc7`;
- `/tmp/card10-predictive-scoped-exact40.YMlQ9q/timeline.json`:
  `8154d5e4eeeec934e717717f9381dd1e5f300f2691702143881a6bcf047a2495`;
- `/tmp/card10-predictive-scoped-boxed40.JE0tj6/timeline.json`:
  `b08bedc459bd0ea946c9b7a0bedf8030215c847b3377ebc3e127861ca5096b94`.

The source row comparable to DART completed step 31 is `step_idx=30`: it
converges 422 contacts in one global workspace at Coulomb-relative `7.59e-7`.
DART has 304 contacts across 18 islands and fails one 79-contact group. Source
uses colored BGS with an inner convergence check every five sweeps; current
DART uses sequential BGS with ten fixed sweeps. These are not the same operator
or contact problem. The source first later reports `converged=false` at
`step_idx=33`.

Final local gates pass: `ConstraintSolver` 66/66, Native collision detector
50/50, `SplitImpulse` 13/13, exact solver 38/38, paper fixtures 36 passed with
3 explicit opt-in skips, visual runner 332/332, and independent post-fix
re-review `ALLOW`.

The separately named
`fbf_author_card_house_10_impact_source_continuation_current_source` scene and
`card_house_author_10_impact_source_continuation_current_source` schedule are
now implemented. They preserve the strict lane's geometry, Native frontend,
3,200-step clock, and step-1,600 release; only exact opts into source
continuation.

The final exact reseal passes 3,200/3,200 and the release action. It records
7,702/7,702 attempts/solves, zero failures/fallbacks, 2,427 plateau
accepts, 763 max-iteration accepts, zero shrink caps, 310,880 total iterations,
7,630 warm starts, 1,071 maximum/987 final contacts, final residual
`7.709159985211234e-8`, and worst residual
`0.59964511064890469`. Timeline validation passes with 3,201 represented
states and 401 captured/unique frames. The 660x506 H.264/yuv420p clip contains
401 frames at 30 fps over 13.366667 s, and full decode passes. Final
panel/keyframe inspection confirms legibility, release, visible post-release
evolution, and lower structure remaining at the endpoint.

The schedule's exact blockers are now empty only within this narrow
continuation-evidence boundary. Metadata keeps `paper_comparable=false` and
`automated_semantic_outcome_validated=false`. The final run summary reports
`pass=true`; independent reuse verification also passes.

The ignored authoritative exact root is
`assets/pr_media_current_head_c95_card10_same_binary_exact_v2/card_house_author_10_impact_source_continuation_current_source/`.
Timeline, clip, panel, and metadata SHA-256 values are
`edddf5bab098f655f6fa6a0adf50bc236474f987fa99f630a1b18d15d6d232ce`,
`19637c4255c890f1f32383e7e7e680169688e5d8b071168bc6b4ffdebf33061d`,
`e5ed0d63ca9818292c5a373f476f2841f280f3e01492e0065b2aec8eb95a74d6`,
and `23fe61063c024d3e93466395798951b4942755ef6bd0c4b3650f5ee00c48c84d`.
The separate run summary `/tmp/card10_same_binary_exact_c95_v2_summary.json`
has SHA-256
`ebf02723ab30875204bed78ebcffe1ef53bebfee8d25e84c5e5649aeb4b0ebf1`.
Independent reuse verification passes; its separate summary
`/tmp/card10_same_binary_exact_c95_v2_verify.json` has SHA-256
`6701bcdea5664d095380e7fa5870972965dec76fdf1595d2e3ca3d8038463055`,
kind `verification`, one result, no skips or groups, full-decode success, and
the matching metadata hash.

A clean boxed control completes 80/80 steps in about 4 minutes 46 seconds with
`BoxedLcpConstraintSolver`; its timeline SHA-256 is
`ccbdc322791a06d5a8858818acae63e8540ca7770e635545e3c017d84bf96d7d`.
That bounded control is not a full outcome. The subsequent full boxed member
completes 3,200/3,200 and passes capture plus independent reuse verification
against the same demo SHA-256 `5725672a...` as exact. Its
timeline/clip/panel/metadata SHA-256 values are
`7d1d272913f4bb72bb0f98bff3d8417668ed86d2522fe913ca3f0bbfca658b43`,
`c3bf391fafa0913e53ce857c497e6411a2810d71f8201a5cffb56e4dd6eb2f20`,
`918eec24dbb1c30876a6d6f4a38fbb209100fe0e2fc7728d8518d233ac19db76`,
and `54414a7ab170569a1645bfaace87ea08b8d7f0fb5ce1ae51b9df87da75c19aae`.

The synchronized labeled comparison is H.264/yuv420p, 1320x530, 401 frames,
30 fps, 13.366667 s, and fully decodes. Its clip/panel/manifest SHA-256 values
are `d09d8a4b6c962eef84620f5fc4aebd709c8631f4c274a302217c56e9163547b2`,
`848805bece727c73e35e51261edd9a02a655cefdb2facd75affdd4667b972794`,
and `800d03fcf8ca5c461b9ce18bbef0ea948a30864fa2bdb739774cf20ca0b333dc`.
The manifest records `presentation_only=true`, `same_demo_binary=true`, and
`runner_group_contract_validated=false`. Manual endpoint inspection finds
retained upright multi-level structure in exact and a largely collapsed boxed
endpoint, but exact requests source continuation while boxed does not.

The 2026-07-22 manually paired, same-binary, one-thread colored-BGS A/B is now
closed as a bounded reject. Serial and colored members both stop after
completed step 31 on a 79-contact group in each run after 200 iterations. No
stable group-membership fingerprint or state-prefix hash was retained, so this
is a same-sized-group proxy rather than proof of identical group identity. The
residuals are
`1.072805023427092e-5` and `1.0728050229273756e-5`. Their absolute delta is
`4.997164045950943e-15`; the colored/serial ratio is
`0.9999999995341964`, or only `1.0000000004658036x` improvement. The retained
failed-attempt telemetry records 200 colored solves, one participant, 39
manifolds, five colors, maximum color width 10, zero dispatches, disabled
affinity, and empty CPU-residency sets. The failure does not move later and
the improvement is far below the preregistered `10x` threshold, so colored
ordering is rejected as this blocker discriminator with no longer run. The
ignored durable sidecar/text-only root is
`assets/sealed_diagnostics/card10_colored_bgs_2026-07-22`;
`RESULTS.md` and `SHA256SUMS` have SHA-256
`c5eebed4feb84b5756e42a4b70404fb2ef3c24cfc88709d0d47958eeb2fc4e2a`
and `2f20edd5cc4baca2f98f2719d0c275c3f066abecde05ac7e717529c1e05b0e9c`,
and all five manifest entries verify.

The 2026-07-22 detached one-global-group diagnostic is also a bounded reject.
Clean, disabled, and native-observe controls reproduce the completed-step-31
failure exactly. Candidate and native inputs have identical contact order and
multiset, native partition, every `mu`, `q`, post-warm-start `lambda0`, and
per-island `W` through generation 27. Every candidate generation has exactly
zero off-block `W` maximum, Frobenius, and relative Frobenius norm. The global
candidate instead stops after completed step 28: its 264-contact production
solve accepts global residual `9.783085822289067e-7` after 83 iterations, but
the independent native-slice audit finds the 39-contact island at
`2.120936044948513e-6`. Stock native scope solves that same island at
`9.487211884987307e-7` after 38 iterations. This rejects native constrained
grouping as a sufficient cause: global scaling/stopping masks a strict
native-slice violation. The audit intentionally invalidates that accepted
reaction, so the ungated trajectory after step 28 is unknown.

The sealed global evidence root is
`assets/sealed_diagnostics/card10_global_scope_82877`. `RESULTS.md`,
`analysis/RESULTS.json`, the final diagnostic patch, and `SHA256SUMS` have
SHA-256
`af3052c38594049adc3c266449e0c13f655cf92752a0ab85ac32bd82d1b3ee62`,
`c1724ac1dfb30550ed38400e83bc59e68bbb23df6a3510fbb900fc4dbad4b160`,
`04316087130d87558546129a36d21aef70e5930215e3cf70f56b2999bfb6ac7b`,
and `99dff645e0d1f55e7e6519da0557d3b385afe8c554f09d34417642439b785b90`;
all 52 manifest entries verify.

No ten-level media has a GitHub URL; manual browser-composer upload remains
pending. Neither the strict prefix nor the qualitative continuation-policy
pair establishes source, trajectory, automated physical outcome, a solver-only
A/B, strict convergence, Tables 6-7, timing, superiority, or paper parity.
The colored and global-scope results are numerical bounded rejects only. The
colored scene is registered as an opt-in runnable diagnostic while its solver
path remains disabled by default; the detached global patch is not shipped.
Neither authorizes a 100-step/3,200-step extension or
supports visual, performance, source/backend, trajectory, physical, Tables
6-7, superiority, or paper-parity claims. A new strict A/B requires a new
source-backed preregistered mismatch without loosening tolerance, iteration
caps, fallback, fail-fast, or accepted-cap policy. See
[CARD_HOUSE_10_CURRENT_SOURCE_DIAGNOSIS.md](CARD_HOUSE_10_CURRENT_SOURCE_DIAGNOSIS.md).

### Pinned-Author Masonry-Arch Scientific Negative

The locally sealed bundle is
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
| Older oriented-box 101-stone proxy | Step 120/600; visibly collapsed at 2 s | Physical mismatch; no valid artifact; distinct from the source-pinned adapter below |
| Older reconstructed dynamic 10-level card house | No completed step 1 | No valid artifact; distinct from the source-supported lane |
| Source-supported ten-level card house | Previous checkpoint `ffe23d347b0` retains the historical step-1 blocker; predictive checkpoint `3647959a188` clears step 1, then fails after completed step 31 on a 79-contact group while boxed completes 40. Separate continuation exact/boxed members complete 3,200/3,200 and have a same-binary labeled presentation pair | Strict remains blocked. The long pair is qualitative continuation-policy media only; no solver-only A/B, automated physical outcome, strict convergence, source/trajectory/Tables 6-7/paper parity, or superiority claim |

The c95-bound exact/boxed literal-standing recapture is ignored outside Git
at `assets/pr_media_current_head_fig07/`. Capture and independent reuse
verification pass for two 600-step members and their synchronized group. Their
external summary SHA-256 values are
`5a1de1f915d75c373f06aeb48978b92a540bc245427c55584f68ad178ea491bb` and
`e4b3d44d5f2afebef9f79bcb92b38ee282f5635c38fa4be2f4264a5f961acce5`.

Exact records 600/600 solves, 96 contacts, zero accepted
caps/failures/fallbacks, final/worst residual
`9.778093504499096e-7` / `9.999807145410957e-7`, 599 warm starts, and 7,933
iterations. Exact timeline/clip/panel/metadata hashes are
`6041addd27a79a747cdbcdaafb495f787d4e90a906ec0616aa16e5a33d9c9b74`,
`24c110421572500bec9f43a431061ae5a386e7e59940e86030e3059cc90d9676`,
`cd3498c90fd549365dadc9cf96908c4c3d86da81cbd5dd64ff4d891407b4ee6b`,
and `614704cc1ed70065d81b789c019e618ac54d7013f706b26a346ea236ef876802`.
Boxed completes 600/600 with `BoxedLcpConstraintSolver`; its corresponding
hashes are
`809ca91a475fdd0ebe3ad6b5cba73115c9ec2b3dd4a98e478beca229abf62321`,
`80e79fa6b356f951e9615dd94aad2de2f55d0bbab07d7b116cb07c1b3bef686c`,
`078990e4d7a950ba9e207102624d651d948f546f1153bf85077e8084c01b040a`,
and `944a6636bd6febb79ab9d6abda0a92165acd34ab3aa96b42f55f1c36731e6d45`.
The labeled group clip/panel/metadata hashes are
`89c4d7372f68c6c9ad1a5d0e0e0388ffa1f198c2446e04fe30b9bc66325d8f9e`,
`5ce6efcebcb5f6a2385f6aea6de7933cccfa7446979b7ea0e46ef2aab5199633`,
and `5cc2513eb16db191454b27126783df70adbe3ad2617d3faa3f51597ab43966bb`.
The grouped H.264/yuv420p clip is 1320x530, 301 frames at 30 fps over
10.033333 s, and fully decodes.

Manual final-endpoint and group-panel inspection finds both arches visibly
standing with clear labels/framing. That is qualitative observation only:
exact, boxed, and group metadata all set
`automated_semantic_outcome_validated=false`. The composite is presentation,
not a physical oracle or solver-superiority test. This recapture proves no
source/paper trajectory, outcome, timing, crown impact, or Fig. 7 parity. The
separate source-configuration crown-impact adapter remains blocked at strict
step 142. The group clip still needs manual PR-browser-composer upload and a
recorded GitHub user-attachment URL.

The separate c95-bound crown-impact continuation root is ignored at
`assets/pr_media_current_head_fig07_crown_continuation/`. Its paired capture
summary and two-result/one-group independent reuse verification pass with
SHA-256 values
`f0e45526d648d7c8d6052c3a4f32ec47a29033e4ed687b89fecc52c1ce04396f`
and `969ef6143185716e8441704829d4643a1d804fc5580288dae28fe47257fce0f3`.
Exact and boxed complete 2,000/2,000 steps with successful release at step
1,600. Exact records 2,122/2,122 attempts/solves, zero failures/fallbacks,
1,940 plateau accepts, 98 max-iteration accepts, and final residual
`0.004493046465992133`. Group metadata/panel/clip SHA-256 values are
`4229307f7d6d91f4b347fecc53db9f290061c6dc76482e684a94064f764601d7`,
`f3bdb5a20ad57ee20e1a2cf6508a701f9bbd79bf0532ffc303d87989b4dfa802`,
and `c4ffe2488520a5c22608c9117443cf9ff5de5396f4353d4bced5d1afff6bf0c8`.

Manual inspection finds both arches standing and cubes reaching the crown,
with nearly identical visible outcomes. All records keep
`paper_comparable=false` and
`automated_semantic_outcome_validated=false`. This is bounded non-strict
continuation evidence only, not strict convergence, solver superiority,
physical outcome, source/paper trajectory or Figure 7 parity, timing, or
backend evidence. Upload the group clip through the GitHub browser composer
and record its URL; do not commit the generated bundle.

The earlier trace-equivalent local literal-wedge standing bundle at
`assets/paper_evidence/fig07_arch25_literal/` is locally finalized as valid
current-source, non-paper reconstruction evidence. It contains five selected local
1280x720 stills, a fully decoded 61-frame H.264 schedule, and a timeline. Its
independent capture and trace paths compare all 600 rows with zero differences
in the nine mapped integer and five mapped floating-point fields, alongside
fixed reference/scene-contract checks. The five selected local stills, timeline, and
a separately decoded video midpoint were manually inspected. The retained
pending hash DAG passed before final writes, and the final index covers 19
artifacts in a 21-file physical directory. The 70-file raw capture staging
directory is pruned after local sealing. With the compact local bundle present,
verify-only does not require that raw staging directory.

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

The recorded local capture identity is hash-bound: capture source
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

The source-pinned 101-stone DART packet is separately retained at
`assets/paper_evidence/fig08_arch101_author_current_v1/`. It binds the author
commit, the `--stones 101` selection, 101-mesh tree
`e0c209235673d2f69c3c5de7708ab1dfadec96e3`, and path-manifest SHA-256
`7198f71730d06dd70af8703065541765bd6b6f5da137f28f9befdf7acc5f96bf`.
The scene contains 101 tapered meshes with 99 mobile stones, two fixed
springers, and three pinned cubes. Its source-supported 400-frame /
1,600-substep schedule ends at `drop_frame=400` and performs no release.

Strict exact fails closed after 209/1,600 completed steps on an iteration cap:
208 contacts, 5,000 iterations, residual/best/worst
`1.2582804496066107e-6`, 342/342 attempts/solves, one accepted cap, zero exact
failures, and zero boxed fallbacks. Its 210 state samples are incomplete and
fail the standing oracle; crown height falls from `66.1385625` to
`62.4010546875`, and maximum mobile-body displacement is `3.7375078125`.
Boxed completes 1,600/1,600 with a valid 1,601-sample inventory, finite-state,
and cube-pinning trace, but fails standing: maximum displacement
`21.2188459736`, maximum rotation `3.14152663339 rad`, and crown minimum/final
`58.3806809854` / `61.1013192467`.

The boxed H.264/yuv420p clip is 660x506, 201 frames, 30 fps, 6.7 s, fully
decoded, and SHA-256
`7635c2722b20fb8bcb0255054cc9172153d1dd640fd8e81df4df52c0e515d3c0`.
Manual inspection finds an intact arch through step 400, crown loss by step
800, and visible collapse by steps 1,200 and 1,600. The diagnostic 1320x506
hstack freezes exact at its last rendered step-208 frame while boxed continues;
its SHA-256 is
`d6f5f658e4fb027edb23e0911acd34b74dfd749daace41b5d9c9204af3163b94`.
Capture and independent boxed reuse verification pass. The compact summary
SHA-256 is
`1c19c6c3c36171a5e85f330b2863b429956652fb894aae0aa0b82d68291e3481`,
and exact/boxed timeline SHA-256 values are
`df1ed4afc9ef5aa74f7c0b6da0560ae0d1b63fca28f45051ed27c5dfb3632889`
and `a8caee71c9356a72fa65210207d7b4209d9e305363974ec07c81f19ec14bfa1e`.

Independent full current-source controls do not supply a standing golden. FBF
continues after 1,473/1,600 capped substeps and 57/99 mobile stones cross the
three-unit height-change gate. Kamino completes all 400 frames with finite
arrays, but 98/99 cross that gate, maximum change is `87.23839569091797`, and
the keystone drops `82.03050136566162`. Kamino result and trajectory SHA-256
values are `86b351c212c0e69df371fa66c67c53d6c3421575afbafae3a6b8d339f574dfb3`
and `63c47426019a218942afe3edae31cdf5dbbbd8d8926732ea59120e23bb6cf1a4`.
It saves no convergence/contact history, full poses, rotations, cube
trajectory, or media.

GitHub attachment URLs remain pending. This is a current-DART scientific
negative, not a complete exact/boxed comparison, source trajectory/outcome
equivalence, Fig. 8/video.08 parity, historical source/Kamino golden, timing,
performance, or superiority evidence.

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
| Source-pinned 101-stone DART adapter | All 101 author meshes, Native `FourPointPlanar`, float64 DART; 400 frames / 1,600 no-release substeps | Exact stops at step 209 on an iteration cap with 208 contacts and residual `1.2582804496e-6`; boxed completes but fails the standing oracle and collapses | Current-DART source-parameterized scientific negative and blocker media only; no source/backend/outcome, Kamino, timing, or Fig. 8 parity |
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

The new 101-stone adapter ports the current source-supported geometry,
selection, initial state, and 400-frame schedule, not the Warp/Newton float32
backend or a recovered historical Figure 8 invocation. Its precise exact and
boxed negatives do not close those comparison gaps.

## Current Verification

- Current-source four-level author-card demo build: passed.
- Exact-Coulomb math: 56/56 passed.
- Exact constraint solver: 38/38 passed.
- `ConstraintSolver`: 66/66; Native collision detector: 50/50; and
  `SplitImpulse`: 13/13 passed.
- Current-source four-level headless/continuation C++ fixtures: 13/13 passed;
  the full
  paper fixture binary reports 42 passed and 3 explicit opt-in skips.
- Source-pinned author-backspin visual-runner/finalizer Python suites: 463/463
  passed; Black and isort checks pass without changes.
- Author-backspin numeric reference verify-only passes with four artifacts,
  240 configured convergence flags, 240 trajectory states, and status
  `valid_current_source_numeric_reference`.
- Source-supported ten-level card-house build: passed; all five
  `AuthorCardHouseTenLevel*` C++ tests pass, including the continuation and
  colored-diagnostic contracts. Its step-1 blocker remains historical;
  predictive checkpoint `3647959a188` has separate exact-step-1,
  exact-prefix-31, and boxed-prefix-40 evidence.
- Colored `ConstraintSolver` filter: 3/3 passed, including the two
  `ExactCoulombColored*` tests and the existing default-off/copy test.
- Source-default five-level card-house build and both focused strict/
  continuation C++ fixtures: passed. The final exact and boxed v3 captures and
  both independent reuse audits pass; the labeled 401-frame presentation pair
  passes H.264/yuv420p probe and full decode.
- Author-incline shared-specification and production-world C++ contract: 5/5
  passed for exact/boxed solver wiring, finite stepped state, and contact
  inventory.
- Visual runner, including source-pinned 101-stone, both ten-level card-house
  schedule/oracle contracts, Figure 7 crown-impact continuation, and the
  seven-cell author incline: 454/454
  passed.
- Independent post-fix re-review of `3647959a188`: `ALLOW`.
- Shared-library ABI symbol inspection retains the pre-existing nine-argument
  `recordLastFailedExactCoulombAttempt` symbol and the post-correction policy
  methods, and exports the additive source-inner setter/getter plus all 12
  nonvirtual colored-diagnostic accessors. Colored state remains in
  implementation-side maps, with no public data-member or vtable change.
- Exact and boxed adapter-contract smoke validators: passed.
- The earlier Clang/FreeBSD polymorphic-`typeid` warning-as-error was fixed
  before implementation/media head `c95bd5fb916`; a Clang 22
  `-Werror=potentially-evaluated-expression` syntax check passes. Current-head
  remote confirmation is mutable and must be read live.
- Author-incline reference finalizer unit tests: 64/64; verify-only reports 37
  indexed artifacts and 39 physical files.
- Focused manifest/backspin/incline/author-masonry/author-incline evidence
  suite: 859 passed post-merge in 163.95 s.
- Full no-cache dartpy Python suite: 1,555 passed in 165.09 s.
- Author masonry-arch focused CTest: 1/1 target and 8/8 contained tests passed;
  all four previously sealed bundles pass verify-only. The new boxed 101-stone
  capture and independent reuse verification also pass.
- Demo scene documentation verifier: 34 scenes passed; exact and boxed real
  step-zero JSON contracts match the validator.
- The manifest validates 29 canonical requirements with status `partial`.
  Under the local sealed producer closure, live mode performed 118 file-identity
  rechecks with zero skipped; explicit archive mode reported zero live
  rechecks and 118 skipped.
- Ten-level colored-diagnostic post-fix review: two independent read-only
  passes are `CLEAN`; per-scene semantic provenance and an automated paired
  verifier remain nonblocking follow-ups.

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

The last live audit before five-level checkpoint `0e3937e6294` found PR #3377
open and draft at remote head `c5765b37a08`, targeting `6a1d377f616`. Topic
head, target ancestry, merge state, checks, and reviews are mutable and are
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
- The MJCF NaN assertion was classified as base-owned against target
  `75306efe770`; refresh that classification if it recurs on a later target.
  It was not evidence of an exact-Coulomb branch regression in that audit.
- Windows reaches the five-hour job timeout while compiling, then reports
  cancellation and `EBUSY` cleanup. The current target exhibits the same
  workflow/runtime failure and no product test runs.

Local fixes and historical classifications do not substitute for current-head
cross-platform CI.

## Remaining Completion Gates

- Correct the source-selected four-level adapter's completed-step-35 exact
  failure and complete its strict 2,400-step exact and boxed trajectories
  through the step-1,600 release. Keep any next strict A/B isolated from
  tolerance, iteration-cap, fallback, fail-fast, or accepted-cap changes.
  The colored-ordering, one-global-group, source-sized-gap, residual-cadence,
  terminal spectral-estimate, and source-seed-values candidates are ineffective
  for this Figure 6 blocker. Preserve all six bounded rejects. Require a new
  source-backed,
  preregistered mismatch before another strict solver A/B. Preserve and
  independently review the separately labeled telemetry-rich continuation
  capture without presenting accepted finite iterates as strict exact success;
  upload it only through the PR editor and record the URL. Keep those lanes
  distinct from the reconstructed
  manifold-v2 lane, whose two trajectories remain non-strict.
- Preserve the independently reverified full exact and boxed ten-level members,
  their summaries, and the same-binary presentation-only labeled pair. Keep the
  superseded interrupted step-112 attempt and partial frames classified as
  non-evidence. Upload only the labeled pair through the PR
  browser composer and record its URL. Keep predictive checkpoint
  `3647959a188` distinct from previous checkpoint `ffe23d347b0`, and keep
  continuation evidence distinct from strict convergence. Preserve the
  completed ten-level colored and global-scope bounded rejects; do not rerun or
  extend either without a new source-backed preregistered mismatch, promote
  colored ordering to the default, or ship the detached global patch, and do
  not loosen tolerance, iteration caps, fallback, fail-fast, or accepted-cap
  policy.
- Preserve the independently verified five-level source-default v3 members and
  the policy-asymmetric labeled pair as supplemental non-paper diagnostics.
  Keep the strict step-31 failure distinct from the exact continuation member,
  and keep v1/v2 classified as superseded framing probes. Upload only the v3
  pair through the PR browser composer and record its GitHub user-attachment
  URL; do not present it as strict success, a solver-only A/B, superiority, an
  automated physical outcome, historical Tables 6-7 evidence, or paper parity.
- Make the failed small-scene physical/residual contracts pass without
  changing the source claim.
- Keep multicore claims confined to the validated opt-in colored literal-arch
  workload until other exact workloads have equivalent dispatch, residency,
  matched-work, and outcome evidence.
- Complete strict full-duration physical outcomes for every card and arch row.
- Audit and port the pinned public author scenes, solver configuration, local
  kernel, and current external runners; preserve the sealed masonry-arch run
  and the new source-pinned 101-stone DART run as distinct negatives. The 101
  adapter now executes current-source geometry and schedule, while full
  current-source FBF and Kamino controls also fail standing. Recover the
  historical invocation/backend or a matched historical oracle before an
  approved visual comparison. Preserve exact DART/source differences.
  Keep the unavailable historical
  renderer/goldens, Apple hardware, precision, and paper timing-attestation
  gaps explicit.
- Preserve the finalized incline, historical Painleve proxy, source-pinned
  Painleve adapter, backspin, and turntable bundles within their lane-specific
  claim boundaries. Upload the two source-pinned Painleve exact-vs-boxed clips
  and both c95-bound Figure 7 groups (literal standing and crown-impact
  continuation) only through the browser composer and record their GitHub
  URLs. Keep both visible-outcome observations manual-only, label continuation
  non-strict, and preserve the separate step-142 strict impact blocker.
  Use the validated implementation/media-head `c95bd5fb916` small-row reseal as the
  retained Figures 1-2 and 4-5 upload source, with the source-pinned author
  capture used for Figure 3; produce
  long-scene sidecars/media only when their solver/outcome gates pass.
- Complete the final integrated lint, build, test, Python, docs, manifest,
  trace-schema, demo, media, and independent-review batteries.

Until these gates close, the correct status is partial implementation with
positive small-scene evidence, one positive non-paper contact-rich
mean-throughput/scaling result, and explicit negative paper-profile evidence,
not completion or paper parity.
