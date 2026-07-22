# FBF exact Coulomb friction for DART 6.20

## Status

This task is active and incomplete. It is a DART-side reconstruction of the
SCA 2026 paper "A Splitting Architecture for Exact Reduced Coulomb Friction,"
not yet a source-equivalent reproduction of the authors' public reference
implementation pinned at
`b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0`. Do not retire this folder.

> **Evidence storage contract:** `assets/` is ignored local working state and
> is absent from a clean checkout. "Finalized" or "sealed" describes local
> validation state only, not repository publication. Paths under `assets/` are
> provenance identifiers, not repository links. Local `--verify-only` checks
> require the compact bundle to be present, although they do not require pruned
> raw-capture staging. Review media becomes published only after its
> `github.com/user-attachments/...` URL is recorded; reusable source-controlled
> documentation media belongs under `docs/assets/`.

The required answer to "Do we have all tests, benchmarks, and GUI examples
from the paper?" remains:

```text
No.
```

[AGENT_CONTINUATION.md](AGENT_CONTINUATION.md) is the authoritative truth
ledger. [RESUME.md](RESUME.md) is the concise next-session checkpoint, and
[HANDOFF.md](HANDOFF.md) is a copyable fresh-session prompt.

## Current Reality

| Area | Current truth | Verdict |
| --- | --- | --- |
| Author source | The MIT-licensed `matthcsong/fbf-sca-2026` reference implementation is public and pinned at `b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0`; it contains the Warp/Newton FBF solver, six runnable examples, current scene/configuration source, optional MuJoCo/Kamino runners, and masonry meshes | Source-port and matched-run work are internal; current author invocations were independently run and preserved, but historical renderer/camera/material/golden assets, original invocation/timing logs and warmup/aggregation attestation, and the exact Apple-silicon model remain unavailable |
| PR integration | PR #3374 is historically merged at `fa17fad`; #3377 was verified open and draft on 2026-07-21, while target ancestry, topic head, merge state, checks, and reviews must be verified live | #3377 is work in progress, not completion evidence |
| Exact math | Contact-row signs and conventions are validated; row-operator versus impulse-test `W` has relative error `1.33e-16`; spectral-nullspace regressions pass | The tested algebraic path is supported |
| Literal 25-stone exact trajectory | The reconstructed Native FourPointPlanar `1 um`-closure wedge arch completes 600 steps in every warmup and measured run at both one and four threads | Valid local non-paper exact-FBF evidence |
| Colored inner BGS | The arch has 96 contacts, 24 colliding pairs/manifolds, 3 deterministic colors, and maximum color width 8 throughout | Four pinned workers execute the measured colored phases; one- and four-thread trajectories and outcomes match |
| Local timing | Three measured 600-step runs per thread count give 1-thread mean `6.122883343333333 ms` and 4-thread mean `4.26939745 ms`, with validated speedup `1.4341328993236115x` | Both means meet 60 Hz; neither run set meets an every-step deadline |
| Paper timing | The recorded timing row is reconstructed float64 DART on x86-64 Linux, not the public Warp/Newton float32 workload, and the historical Apple host/timing protocol is unattested | Paper target and timing verdict remain null |
| Current small CPU matrix | Current-source `paper_cpu`/Native bundle has 60 indexed artifacts, 27 complete pinned invocations, and 5,220 raw rows: all 9 physical classifiers pass, while 7/9 scenarios pass the strict solver and local real-time contracts | Valid current-source reconstructed evidence; zero warmups and unmatched paper contracts make every paper timing verdict null |
| Literal 25-stone visual evidence | Locally finalized current-source bundle has 19 indexed artifacts / 21 physical files, five selected local stills, a 61-frame decoded clip schedule, 600 zero-difference trace rows, and bound manual inspection | Valid reconstructed no-projectile, non-paper visual evidence; the 6.1 s clip is a 1.639344x time-lapse |
| Incline visual evidence | Locally finalized `fig01_02_incline_current_v1` has 21 indexed artifacts / 23 physical files, five selected local stills, a 61-frame decoded clip schedule, two independent 121-row traces, a passing manual inspection, and byte-identical aggregate exact-solve/fallback projections | Valid current-source DART threshold evidence only; capture contacts are 8 while the traces report 6 in aggregate, and `fig.01`, `fig.02`, and `video.03` remain partial |
| Pinned-author incline sweep | `author_incline_sweep_reference_v1` preserves independent current-source FBF, MuJoCo, and Kamino CPU runs over `mu=.3,.4,.45,.5,.55,.6,.8`; each lane has seven 120-step cells, and the retained FBF histories record four contacts per FBF step | Numeric current-source scientific-negative/reference evidence only; FBF has 839/840 configured convergence flags, every timing field is diagnostic-only, and no DART, historical-paper, full-state, golden, media, timing, performance, or parity claim follows |
| Source-pinned Painleve adapter | The ignored `fig05_painleve_author_current_v1` bundle binds the public author configuration and exact-options header hash; capture and independent verify pass for four exact/boxed members and four 61-frame decoded groups with complete traces, measured outcomes, and manual audit | Under the pinned current DART adapter, exact and boxed diverge at `mu=.55`; GitHub video URLs remain pending, and source-backend, trajectory, paper-Figure-5, timing, and solver-superiority claims remain false |
| Historical Painleve proxy visual evidence | Locally finalized `current_v1` bundle has 27 indexed artifacts / 29 physical files, two 151-row tracked traces, fully decoded paired media, and bound manual inspection | Valid historical DART-side nonpaper proxy evidence only; it does not satisfy the source-pinned Fig. 5 row, rendered demos and tracked fixtures are not trace-equivalent, and no paper, external-solver, golden, timing, real-time, or strict-rest claim follows |
| Backspin visual evidence | Locally finalized `fig03_backspin_current_v3` has 18 indexed artifacts / 20 physical files, three selected local stills, MP4/GIF media, 129 exact attempts/solves, zero caps/failures/fallbacks, a corroborating translational trace, and a passing manual inspection of the renderer-applied high-contrast 6x4 ivory/charcoal checker texture and coral registration tile | Valid current-source DART evidence only; `fig.03` and `video.02_backspin` both remain partial |
| Author-pinned turntable visual evidence | Locally finalized `fig04_turntable_author_current_v1` has 58 indexed artifacts / 60 physical files, four timeline-bound outcome stills, and all four 360-step author-configured cells; the current visual lane records three ejections and `mu=.5, omega=2` retained on support through 6 s | Valid author-source-pinned non-paper finite-horizon DART evidence only; zero slip, co-rotation, full-state equivalence, paper timing, approved-golden, and paper-parity claims remain unproven |
| Author card-house construction | Locally finalized `card_house_author_5_construction_current_v1` has 12 indexed artifacts / 14 physical files and shows the public-author default five-level, 40-card configuration at step zero | Construction-only evidence: zero simulation substeps; no release, standing, trajectory, solver, contact-dynamics, physical-outcome, Fig. 6/video, timing, performance, or parity claim |
| Current-source four-level card-house adapter | The strict `fbf_author_card_house_4_impact_current_source` lane and the separately named `fbf_author_card_house_4_impact_source_continuation_current_source` lane bind the pinned author geometry and source-supported four-level, 600-frame selection to 2,400 DART substeps; exact/boxed contracts pass | Strict source-inner replays still fail the 56-contact group at step 35. The telemetry-rich continuation capture completes exact and boxed through the step-1,600 release: exact records 3,351/3,351 solves, 0 failures/fallbacks, 2,605 successes, 113 plateau accepts, 633 max-iteration accepts, and 0 shrink caps. Manual inspection shows both standing through release and more retained multi-level structure in exact at the endpoint, but the run is continuation evidence, not strict convergence, solver superiority, trajectory/golden/backend/timing parity, or Fig. 6/paper parity |
| Pinned-author masonry arch | Sealed `author_masonry_arch_reference_v1` records a 500-frame, 2,000-substep run with cube release at substep 1,600; a deterministic claim-history projection represents every substep, with 157 true and 1,843 false author convergence flags | Valid current-source scientific negative only; the invocation is not the 400-frame source default or a historical paper run, and no DART, cross-solver, trajectory, outcome, timing, repeatability, contact-pair, or media parity follows |
| Source-pinned 101-stone DART adapter | The new author-mesh scene binds `--stones 101` and the 400-frame / 1,600-substep source-supported no-release schedule. Strict exact stops at step 209 on an iteration cap; boxed completes but fails the standing oracle and visibly collapses | Precise current-DART scientific negative only; no source-backend, float32, trajectory/outcome, Fig. 8/video.08, Kamino, golden, timing, performance, or superiority claim follows |
| Reconstructed crown impact | Frozen three-cube v1 completes 720 steps and contacts the arch before the ground, but fails exactness and far-field preservation gates | Valid scientific negative; impact claim is false and no parameter was tuned |
| Card-manifold sensitivity | Current-source v2 compares only Native `Compact` versus `FourPointPlanar`; both emit 600 rows but have zero strict-success rows and accepted capped groups on every row | Integrity-valid reconstruction diagnostic; physical, timing, real-time, and paper verdicts are null |
| Paper-media parity | The passing literal video has no projectile. The source-pinned 101-stone lane now has boxed-collapse media and a frozen-prefix diagnostic comparison, but no complete exact clip, matched Kamino panel, or historical renderer camera/material/golden bundle | No paper impact, Fig. 8/video.08, GUI, or golden-frame parity claim |
| Evidence manifest | Current audit: 29 requirements, 24 `partial`, 5 `blocked`, 0 complete; the local visual inventory includes six locally finalized bundles and the visual workflow declares 25 schedules; the gate hashes local bundle artifacts, materializes bundle indexes, binds provenance, recomputes CPU claims, and enforces semantic boundaries | Honest overall status remains `partial` |

## Current Small CPU Matrix

The locally finalized current-source bundle is
`assets/dart_cpu_evidence/2026-07-19_current_source_paper_cpu_small_r7/`.
It binds the runner, trace source, evidence binary, resolved runtime-library
closure, all 27 CPU-4 invocations, 5,220 raw rows, and 60 indexed artifacts.
Across three repetitions of each of the nine small reconstructed scenarios, all
nine physical classifiers pass and seven satisfy both the strict solver and
local real-time contracts.

The two strict negatives are deliberate evidence, not missing data. Incline
`mu=.5` moves only `8.63436433e-7 m`, but one accepted-cap row per repetition
raises its maximum residual to `1.4392081500753078e-6`. Turntable `mu=.5,
omega=5` passes the physical ejection classifier in all three repetitions, but
all three measured processes fail; they each contain one
`max_iterations_accepted` row, and the maximum residual is
`3.050386527672585e-4`. The run used zero warmups, and its precision, workload,
hardware, scene/frontend, kernel, and timer contracts do not match the paper.
Its local timings therefore cannot be used as an apples-to-apples paper
comparison. Core r7 hashes are report
`008bc94667893cd26bfc04a720caca3d3a5703601c8739bc06856f93818c63d5`,
artifact index
`06594c1e1cc3c6858bc78a80630aefb0e85eeef92204b0a09b99425411c3ebeb`,
summary JSON
`9137f1b2db0909a96897632a66617f2cfa58476a369d2adfe232b73e46cd0fa2`,
and metadata
`e227d7aef3b273ff81385f227f9e6766a7e40a622bc4298e1807b7c2068bc417`.

## Locally Validated Arch Performance And Scaling Evidence

The authoritative mark26 schema-v8 P-core performance/scaling bundle for the
reconstructed literal arch is
`assets/dart_cpu_evidence/2026-07-19_mark26_native25_colored_v10_archwide_pcore/`.
It contains one successful warmup plus three successful measured 600-step
trajectories for each of one and four threads, or 1,800 measured steps per
thread count. `raw.csv` contains 4,800 rows because it preserves the warmups;
the summary timings use only the 1,800 measured rows per thread count.

Across all measured steps:

- contacts remain 96 and colliding body pairs remain 24;
- the colored schedule remains 24 manifolds, 3 colors, and width 8;
- exact FBF succeeds on every step with maximum residual
  `9.999807145410957e-7`;
- exact failures, accepted caps, and fallbacks are all zero; and
- the physical outcome contract passes and the one- and four-thread
  trajectories are identical.

| Threads | Mean ms | Median ms | p95 ms | Max ms | 60 Hz mean | Every step below 60 Hz | Validated speedup |
| ---: | ---: | ---: | ---: | ---: | --- | --- | ---: |
| 1 | `6.122883343333333` | `2.4966535` | `21.663236899999994` | `287.473818` | Pass | No | `1.0x` |
| 4 | `4.26939745` | `1.9047965` | `14.396602399999995` | `180.504588` | Pass | No | `1.4341328993236115x` |

This supports local mean-real-time throughput and validated multicore scaling
for this reconstructed arch. It is not a worst-case deadline guarantee and
does not establish impact/media parity or an apples-to-apples paper result.

Schema v8 preserves the default 83-column trace contract byte-for-byte. The
newline-terminated default header SHA-256 is
`396c866d782e626b5c5b52e74c392de1175f5e46dfd90b809d1f219417092e50`.
The explicitly non-paper colored contract uses a separate 95-column trace;
its newline-terminated header SHA-256 is
`424195336cb42753179130c9f6fcba3a8ddea9bf669cfa70c0f77fcb4c6335a5`.
Its final two columns harden the literal-arch outcome against a tracked-crown
blind spot by recording maximum all-stone displacement and minimum all-stone
orientation alignment from constructed t0. Both thread counts have the same
measured-work fingerprint:
`9d8df2edba609314432ff17f63768fded23577703537040d75ce082ab4233a36`.
The trace binary SHA-256 is
`0923bf7df1eaa518f9a2ffacd0a42fd7330f5cbf6c35cb5749fbc122db1310ff`.

Authoritative bundle file SHA-256 values are:

- `artifact-index.json`: `a60899cc12a53f03424c02c2647f233d5c75f3ccce367ea2a604f9a7ee18bf11`;
- `metadata.json`: `5507ed80140a146d4247c4f0b05fd9503879ce79856189a15759b101c2cab789`;
- `invocations.json`: `719ba3491fad8ac12aa290faa401d0c46b10af52da7eb7a28487a5b2aac44812`;
- `raw.csv`: `91a379f832ca52bbce7011308640012d9f199b98e39cba4eaf661cf17fb0f017`;
- `summary.csv`: `3c08b251c340aa9ae7909d715ebcc2985ba34350a6e03b0b6b351d82d1ec82a2`;
- `summary.json`: `304736d6b871c4498a6c0de4c4448635e712fb3a8a455dd54d9ffefeef2ec170`; and
- `REPORT.md`: `4e55d7f4dc0532ab15b86cfeb72ea9526d3f9b63194dc5a25f3974186b1a7ba7`.

## Current Literal-Arch Visual Evidence

The current-source bundle is
`assets/paper_evidence/fig07_arch25_literal/`.
It binds an independently rendered off-screen capture to the current
`fbf_paper_trace` workload with 600 compared rows and zero differences in all
declared integer and floating-point fields.

- The exact trajectory completes 600/600 steps with zero exact failures and
  zero boxed-LCP fallbacks. Worst residual is
  `9.9998071454109575e-7`.
- All-stone displacement from constructed time zero is at most
  `5.431169776791696e-6 m`; minimum orientation alignment is
  `0.9999999999111284`.
- The bundle contains 19 indexed artifacts / 21 physical files, including five
  selected local 1280x720 stills, a five-panel timeline, a 61-frame decoded H.264
  schedule, and a separately decoded midpoint.
- The video compresses 10 simulation seconds into 6.1 playback seconds at
  10 fps, a `1.639344262295082x` time-lapse rather than real-time playback.
- The five selected local stills, the timeline, and the separately
  decoded video midpoint passed manual inspection.
- Finalization retained the immutable pending metadata and verified its hash
  DAG before writing the final provenance, artifact index, and metadata.

The capture source, capture binary, numeric trajectory, and decoded media are
unchanged. The 70-file raw capture staging directory was pruned after local
sealing; its schedule and hashes remain bound by provenance, while the five
selected local stills retain inspectable frame evidence. With the compact local
bundle present, verify-only does not require that raw staging directory. The bundle was freshly
revalidated against the final current trace and Native source after later
additive scenario work. Finalization regenerated the standing reference trace
with the current executable and again proved all 600 standing rows
zero-difference. Original trace hashes remain bound separately for capture
provenance.

Final visual source, binary, workflow, and media SHA-256 values are:

- 12-function/16-case visual unit file: `f06ba295006cf7e1f0fb692fc5eacf62e4e7f3be89bd166c041752d915779bdd`;
- driver: `0f4e27b0c58e9dd3774c6be48ad4c70a857e2956fd81f11710837561f08f7243`;
- C++ capture source: `c3efeac52d02a0c373f733598db81e545d062195ba6e96c2a65bcb607cd0207f`;
- capture binary: `8b3cad15220c8fdb69c3ebdf7fa3923fda6fd812a49d0e54c8aeb07e62f0a7e9`;
- current trace source: `b00eea0c87f75f17259fac433bafd98c20a961d14389a80e91742d3c8a678f76`;
- current trace binary: `0923bf7df1eaa518f9a2ffacd0a42fd7330f5cbf6c35cb5749fbc122db1310ff`;
- original trace source: `2e55298496d76a3a3fe002fcfaf5391332fdcf2da813b5df400affbea431e7cd`;
- original trace binary: `0923bf7df1eaa518f9a2ffacd0a42fd7330f5cbf6c35cb5749fbc122db1310ff`;
- final `metadata.json`: `b217b0a43200ca558f09776e464d50f0f9dc5c5a175bcfa66ae1b656076871d1`;
- retained `pending-metadata.json`: `e300103dbb950b97e217ca0bea6f1c1dc78598ddf485cdaca26cc7ad58835b3c`;
- `manual-inspection.json`: `4b52bcd26ed88d184bd695d77070420aeec2c95edfb203efda7ae6241150c343`;
- final `provenance.json`: `eca349842e4121584145cd039a554aa13c51e5242ff924b37f5f49a9dee0ac2f`;
- final `artifact-index.json`: `4d173c78e2c6584d6d8a85867160cc0c0687fe0f35e6878f66b0cfcd7c1c15ee`;
- current reference trace: `0e41af230fb4c27143966c3f8522702280e6746b31e84155a9a047326e46dbb3`;
- trace equivalence: `537f6e497b6fb6810240f006db47c91304581741370e319787635e6bcbfdd2e3`;
- frame validation: `a00dd7f4971756589ea834ac1a04f302ba8b7b7af96300d98e9cbc17413b3330`;
- video: `e0c111f96511bbda41af145787bd9f68782b8c9e8a909d45eb03d3c3d647feb1`;
- timeline: `926380c5c5768cc9ca1e414cbfe2631584daa99387090926d5a1f035d62b6ec9`; and
- decoded midpoint: `75a88bb317441ed71803f784b2eaa099211c4e026f8547d6fe5f2ff3ee95909f`.

The allowed claim is narrow: this reconstructed float64 x86-64 literal arch
remains standing and visually stable for the declared 600-step no-projectile
run. It is not a projectile-impact result, a paper or author-scene parity
result, a 101-stone result, or an author-golden image comparison.

## Finalized Fig. 01/02 And Video.03 Incline Evidence

The locally finalized current-source bundle is
`assets/paper_evidence/fig01_02_incline_current_v1/`.
With the compact local bundle present, finalization and verify-only pass with status
`valid_current_source_nonpaper_incline`. The directory contains 23 physical
files; its exact-membership index binds 21 artifacts because
`artifact-index.json` and `metadata.json` are the two declared exclusions.

- The combined capture retains five selected local 660x506 stills and a 61-frame
  decoded H.264 schedule at 30 fps. The 70-file raw capture staging directory
  is pruned after local sealing, so verify-only does not require that raw
  staging directory. It records
  240 exact attempts/solves, zero accepted caps,
  exact failures, or boxed-LCP fallbacks, and worst residual
  `9.999836962261359e-7`. It reports eight contacts per post-initial step.
- The independent `mu=.4` and `mu=.5` traces each contain 121 rows, 120 exact
  solves, 119 warm starts, zero fallbacks, and three contacts per
  post-initial step. Both preserve continuous post-initial tracked contact.
- At `mu=.4`, downhill displacement is `1.7686892884927794 m` versus the
  analytical `1.7548661487418349 m`; final downhill speed is
  `1.7544655347780056 m/s`, and maximum residual is
  `9.986952135669881e-7`.
- At `mu=.5`, downhill displacement is `0.0008905412965980523 m`, maximum and
  final stick speed are `0.001116442058867632 m/s`, and maximum residual is
  `9.997210606407098e-7`. The two tracked outcomes are separated by
  `1.7677987471961814 m` of downhill displacement.
- The capture and summed traces have byte-identical projections only for
  `step`, cumulative exact solves, and cumulative boxed-LCP fallbacks, at
  SHA-256
  `f03dff0aaec5f0fa6615609c2ea97aa31f072c14c6212fffe15858ea969d88c2`.
  Capture contact count 8 does not match aggregate trace contact count 6, so
  `contact_count_match=false` and `contact_counts_compared=false`.
  Full-state, residual, status, warm-start, per-cell, and full-trace
  equivalence are not claimed; the combined renderer and independent tracked
  traces use different placements.

Core finalized SHA-256 values are metadata
`7a5f973a9b7264911058ec91e253dfcf5d72a7ec46fa7020df0020af1a259b7d`,
artifact index
`b758bd28965bf9a96be7668c0dbb738b72c1493d83f195a3e726ae891f8f6e85`,
manual inspection
`3c3af65d62c629ae836302910a2fe7f928ab398628f280221f6d0b5d94d5a848`,
trace summary
`4df130e878f1e58d478870c8f132ee165061a752520e926f44c331d32f14f20d`,
verification
`2681073ee44f7fecd2782081826b414ee6541bb930149ced91f60a17dc2416d5`,
report
`f75efcd40bd0452bcbdc7bbc82eee0fdbd78d7a1059f974e83999467b1688fa5`,
panel
`f9f211fb376c97d98bccc67806ba3e1c9905d7d27764c794e1c480af7b4df9d3`,
and clip
`ad6d00ae6a614b0edbc836396c30621b589295a0eec826d060663f6782cee3f9`.
The `mu=.4` and `mu=.5` trace hashes are respectively
`449acf19feef2e0aa7fb04bb9f45f865727ba59f626b3964114cd900169ecd8a`
and
`2b30e8033b123876ad1cdea755741fd230a4d72d3747e55b907e6427962659c5`.
Finalizer/test, visual-runner/test, demo-source/binary, trace-source/binary,
fixture-source, and resolved-libdart hashes are respectively
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

This visual/trace result does not erase the separate strict
`paper_cpu`/Native result: `mu=.5` moves only `8.63436433e-7 m` there, but one
accepted cap per repetition raises the maximum residual to
`1.4392081500753078e-6`, so its strict solver/local-real-time contract remains
failed. Fig. 1, Fig. 2, and video.03 remain `partial`: the full friction sweep
and plot, matched external-solver rows, approved source golden/diff, paper
contact-count match, full 11 s semantic edit, paper timing, and real-time
parity remain unproven.

## Pinned-Author Incline Sweep Scientific Negative

The numeric current-source packet is
`assets/paper_evidence/author_incline_sweep_reference_v1/`.
It preserves three independent CPU invocations of the pinned public runner at
commit `b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0`: FBF, MuJoCo, and Kamino each
use the exact Figure 1 grid `mu=.3,.4,.45,.5,.55,.6,.8`. Every cell emits 120
steps, so each lane has seven cells and 840 rows. The retained FBF histories
record four contacts per FBF step; the MuJoCo and Kamino result records contain
no contact-count field.

FBF records 839/840 configured convergence flags. The only false flag is
`mu=.55`, step 1, after the 200-outer cap, giving that cell 119/120 true
flags. Of the 839 true flags, 235 use the initial natural-residual shortcut
and 604 use the configured outer nonnegative `coulomb_rel < 1e-6` gate. The
projected natural `final_residual` is a distinct metric: only 456 of those
true flags are also at or below `1e-6`, while 383 are above it. Conversely,
the sole configured-false row has natural residual
`3.273267262002487e-8` but configured terminal
`r_coulomb=1.5311460572898186e-6`. Never substitute the natural residual for
the configured convergence decision.

The normalized displacement data show close FBF/Kamino values and a
nonmonotone MuJoCo curve in this current public runner. That observation is
not full-state or cross-solver parity. First-use JIT work, always-on history
collection, ineffective warmup exclusion, and scene-dependent timer
boundaries make the source timing fields diagnostic-only. The packet is not a
historical paper run, a DART/source trajectory comparison, an approved
golden, media evidence, or timing/performance evidence, and it does not add a
seventh visual bundle. Fig. 1, Fig. 2, and video.03 remain partial.

## Source-Pinned Figure 5 Painleve Adapter

The north-star Figure 5 path now uses two source-parameterized demo schedules:

- `painleve_author_mu05` runs `fbf_author_painleve_mu_0_5` at `mu=.5`;
- `painleve_author_mu055` runs `fbf_author_painleve_mu_0_55` at `mu=.55`; and
- `painleve_author` groups those cells, with both exact and boxed solver lanes
  captured separately and compared under the same DART collision frontend.

The adapter pins author commit
`b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0`, tree
`ffcdafb61adeda2239c8366d054b548b50d26685e`, and Painleve `run.py` blob
`afaa03613b0ad0a30290168d2fd64221fc3523b7` (SHA-256
`818fa8f75c2c73e2dd08f0e0e9f9f5d58f63d8073dce38f874e2da24b2aa46e3`).
Its pinned public-source configuration is a `0.3 x 1.2 x 0.6 m` box in DART xyz
order, density `200 kg/m^3`, mass `43.2 kg`, upright center at `z=.3 m`, initial
velocity `(4,0,0) m/s`, gravity `9.81 m/s^2`, `dt=1/60 s`, and 120 steps over
2 s. The selected public-source sweep is `mu=.5,.55`; the source default is
only `mu=.55`, and the historical paper invocation is unknown. Source
`gap=.005`, `ke=1e4`, and `kd=1e3` are recorded but are not implemented with
equivalent semantics by the DART Native `FourPointPlanar` adapter.

This lane is implemented as a DART configuration adapter, not a port of the
authors' Warp/Newton float32 backend. Its exact policy maps the public
`gamma_c=5` request onto DART's adaptive safe-step convention, while the boxed
lane uses DART's existing boxed-LCP solver under the same scene and collision
contract. The adapter also binds SHA-256
`c48867ded0c3523e10eb47690aa5bf980db40281b219165cb8e31b0e492890f8`
for `dart/constraint/ExactCoulombFbfConstraintSolver.hpp`, so exact-option
default drift fails closed.

The ignored durable bundle is
`docs/dev_tasks/fbf_exact_coulomb_friction/assets/paper_evidence/fig05_painleve_author_current_v1/`.
Capture summary and independent verify both pass with four member results and
four group results. Every member and composite clip is a fully decoded
61-frame H.264/yuv420p MP4; panels and selected keyframes were manually
audited.

| Cell/lane | Current-DART-adapter outcome | Horizontal travel |
| --- | --- | ---: |
| `mu=.5`, exact | `upright_near_rest` | `1.5986787381 m` |
| `mu=.5`, boxed | `upright_near_rest` | `1.5977005918 m` |
| `mu=.55`, exact | `tumbled_near_rest` | `1.5399225956 m` |
| `mu=.55`, boxed | `upright_near_rest` | `1.6623056217 m` |

Exact `mu=.5` records 119 attempts/solves, zero failures/fallbacks, final
residual `5.2255077e-7`, and worst `9.7391465e-7`. Exact `mu=.55` records 108
attempts/solves, zero failures/fallbacks, final `9.1964345e-7`, and worst
`9.9977460e-7`.

The defensible result is exactly: under the pinned current DART adapter, exact
and boxed lanes diverge at `mu=.55`. Source-backend equivalence, trajectory
equivalence, paper Figure 5 parity, timing comparability, and solver
superiority remain false. GitHub attachment URLs remain pending manual upload
through the PR browser composer; the bundle remains ignored and outside Git.
See [PAPER_DEMO_VIDEO_MATRIX.md](PAPER_DEMO_VIDEO_MATRIX.md) for clip hashes
and exact upload paths.

## Historical Finalized Painleve Proxy Visual Evidence

The locally finalized current-source bundle is
`assets/paper_evidence/fig05_painleve_proxy_current_v1/`.
It replaces the Painleve cells from the older session-local `/tmp` visual
matrix as the retained local DART-side proxy evidence. It remains a historical
diagnostic and does not complete the source-pinned Figure 5 lane above. With
the compact local bundle present, finalization and verify-only both pass with status
`valid_current_source_nonpaper_proxy`, 27 indexed artifacts, and 29 physical
files; pruned raw capture staging is not required.

- The artifact index binds 27 files by exact membership, byte size, and
  SHA-256. The selected local member and paired clips each fully decode to 76 frames;
  the synchronized pair is 1320x530 at 30 fps. Raw capture frames are pruned
  after sealing.
- Both separately tracked fixtures complete 150 steps and write 151 rows with
  zero exact failures and zero boxed-LCP fallbacks. The maximum tracked
  residual is `9.998574150559113e-7`.
- At `mu=.50`, the tracked box remains upright and reaches its settled proxy at
  `x=1.298081699724907 m`, with final `up_z=0.999998178998452`.
- At `mu=.55`, the first fixture-defined tumble occurs at step 36,
  `t=0.6000000000000002 s`, and `x=1.2597198197697048 m`, which is
  `0.03836187995520213 m` before the `mu=.50` rest distance.
- Recorded manual inspection confirms that the rendered `mu=.50` member
  returns upright and the `mu=.55` member tumbles and remains visually
  horizontal. The physical trace, not the paired panel alone, supports the
  shorter-pre-tumble-distance classifier.

The verified current-source source/binary SHA-256 values are:

- finalizer: `31b2b560a3a6a7f06e514a8bc3dce9f4b766b3c4e62fe520435bfaa1e3ba77a9`;
- visual runner: `d848afa53caf14b9fb3ea061d658eef274e8d917151937bd6340283b79ab5432`;
- visual runner test: `6e378252fa6a7cb51c6813c9d5a2b30b8c8129eacdb01df0ee19a58a270cbc5e`;
- demo source: `84fd1330a13d548760e537f9734790ab1b5c91fcda3e446bf76b9b36f3e1aa99`;
- demo binary: `d838f23e9fb04224ff194658bd6e53d8cbb95ac94ce6676e54c49b8ea25916e4`;
- trace source: `b00eea0c87f75f17259fac433bafd98c20a961d14389a80e91742d3c8a678f76`;
- trace binary: `0923bf7df1eaa518f9a2ffacd0a42fd7330f5cbf6c35cb5749fbc122db1310ff`; and
- fixture source: `a4bed7372213d544fef62923b88bc9accee7086165d7b3cb20245bee8bade05f`.

Final bundle SHA-256 values are:

- `metadata.json`: `0845988dd05b18c965eba5fb5163d43dafc901ca8f30f66b01b4f37163d72f30`;
- `artifact-index.json`: `7880c62d39c3f47109b50394cd84e20919565b70af19c1473c618841f682ff43`;
- `manual-inspection.json`: `27c2632e427ec83b3612de6003cd16523fd483ab066d6618f9cdee38efdd2d0c`;
- `trace-summary.json`: `115b50d92338477022435911df38d53a54e880c1daed4004dedd7d8507336164`;
- `verification.json`: `6fd34e54f7958c45777731ab888fe3a5e8e24d06b9fdddb96bdace269fcb515e`;
- paired panel: `bef97d04d59e2937195151cbf36a0b713bb2c2bef43d111513c05885595546e2`;
- paired clip: `dd4cdda2410b71f8e86035f1c0ff278f9dda77133c702e409ee5d533da443a4b`; and
- `REPORT.md`: `a04c37f9f3db428994252c8486030aa6b97b367a627fc3a1a22106dfa8a9f51f`.

The scientific claim boundary remains strict: the rendered demo and tracked
fixture are separate proxy implementations and are not trace-equivalent.
Angular velocity is not exported, so strict rigid-body rest is not proven.
This bundle does not establish author-scene or paper parity, faithful
external-solver parity, an approved source golden/diff, paper timing, or
real-time performance.

## Finalized Fig. 03 And Video.02 Backspin Evidence

The locally finalized current-source bundle is
`assets/paper_evidence/fig03_backspin_current_v3/`.
Its exact-membership index binds 18 artifacts in a 20-file physical directory.
The MP4/GIF media preserve the full motion schedule, and three selected local stills
retain steps 0, 1, and 2 where the checker orientation changes are easiest to
inspect. The 140-file raw capture staging directory is pruned after local
sealing. With the compact local bundle present, verify-only does not require
that raw staging directory. The capture records 129
exact attempts and solves, zero accepted caps,
exact failures, or boxed-LCP fallbacks, and maximum residual
`9.96497154974839e-7`. The separate 131-row trace has maximum residual
`9.964971544991853e-7`; it reaches maximum forward travel
`x=1.5959314363310166` at step 48, first records negative `vx` at step 49,
and ends at `x=-2.9362508912363654`, `vx=-6.628158971623909`. Step 120 is
the sole contact-free post-initial step.

The sphere uses a renderer-applied high-contrast 6x4 ivory/charcoal checker
texture with a coral registration tile. Its UV `MeshShape` is attached with
`VisualAspect` only; collision, inertia, friction, and dynamics continue to use
the unchanged physical `SphereShape`.

The trace and capture solver/contact projections are byte-identical with
SHA-256
`973d544311bac3b5927cc73b335b1a375d0339403a2f713707bb928076aa2b22`.
Manual inspection passes the checker pattern and coral registration tile as
legible in consecutive source frames. The finalized artifact SHA-256 values
are:

- metadata: `a42ce0521a7c2af31662eff6a000ef5e68fe8bddf631d4f323be1ea8230c25a7`;
- index: `429a0888fa7a002cbc0c93e708569e4bf8e18195421c9663d5ce3b3a1968ab7f`;
- manual inspection: `b86c596da631503a3831fd55adeb934505cf758dfc8a612d0a0f32b2a55067cb`;
- trace summary: `0f6221fd32742b849e9ac79ec750de71b46ec24ba66b8d6e5a0e25048223ab48`;
- verification: `fb9de609e52df648465dc0dbe73af7fbbd1b98c1c4671cde504200ff72c15c01`;
- trace: `dc205297fa4cbffa1b497f12507b919f344bbee45a5820ae46145e0ed91bbd98`;
- panel: `72bf8d6098e8fb17b98bbedeaa00cc539866cf570d91d725f77dc75f1971b067`;
- MP4: `7d4606f4da0a57ffbdfa0528906b21a20d7e1a4e47a6e7eb5387242aecc71928`;
  and
- GIF: `773365f624ba1326855f2ad99c0196f761ab776001da568f7cdaa84054adacc8`;
- `REPORT.md`: `f9178c14c1930361afc1d97cb5bf08afe04d3fd451e8a94144b02bb0b46e61cf`.

The claim boundary remains strict. At `-200 rad/s`, the 30/15 fps media can
alias and do not prove signed angular direction. The contact-free step rules
out continuous contact, and the bundle proves neither rest nor an airborne
landing phase. The rendered demo and CSV exporter are separate scene
implementations, so the shared diagnostic projection is not full-state trace
equivalence. No faithful external-solver, paper, approved-golden, timing, or
real-time parity follows. Both `fig.03` and `video.02_backspin` remain
`partial`.

## Finalized Author-Pinned Fig. 04 Turntable Evidence

The locally finalized bundle is
`assets/paper_evidence/fig04_turntable_author_current_v1/`.
It pins the public author configuration at commit
`b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0`, binds 58 indexed artifacts in a
60-file physical directory, and preserves the source order `mu=.2, omega=2`,
`mu=.2, omega=5`,
`mu=.5, omega=2`, `mu=.5, omega=5`.

All four current visual-lane runs use `dart_best` with Native
`FourPointPlanar`, complete 360 steps, pass the solver contract, and record no
fallbacks. Their exact finite-horizon outcome is ejected, ejected, retained on
support through 6 s, ejected. The retained `mu=.5, omega=2` cell proves only
that 6-second support-retention predicate; it does not prove zero slip,
perfect sticking, co-rotation, or behavior beyond the six-second horizon.
Manual inspection passes the segmented disc, one coral registration wedge,
labels, and source order. The segmented disc and wedge are visual-only; the
physical cylinder remains the sole collision and dynamics geometry.
Four selected local, timeline-bound outcome stills cover the decisive steps for
the four cells (136, 120, 360, and 90 in source order). With the compact local
bundle present, verify-only uses the selected clips, panels, timelines, stills,
traces, and provenance without the pruned raw capture staging.

For all four cells, capture and trace projections of `step`, `contacts`,
`exact_solves`, `warm_starts`, `boxed_lcp_fallbacks`, and `status` are
byte-identical. This is not full-state equivalence. The separately recorded
strict `paper_cpu_native` lane is not substituted for the visual lane and has
no capture comparison: its `mu=.5, omega=2` row fails at step 40 with residual
`7.407835021099202e-6`, while its other three rows pass. Thus
`paper_cpu_native_all_solver_contract_valid=false` does not invalidate the
current visual-lane finite-horizon outcome, and the visual result does not
erase the strict-lane failure.

The bundle status is
`valid_author_source_pinned_nonpaper_turntable_matrix`, with
`artifact_valid=true`, visual-lane `solver_contract_valid=true`,
`physical_outcome_valid=true`, `manual_inspection.pass=true`, and `pass=true`.
Core SHA-256 values are report
`930cc12b95ab78c6e61d084064b584f5872633f2432e434c68d071818cec7fb1`,
artifact index
`209b677ce35e2b9c11248ab414727016394831084881a5e9c2866f68b51f6cdf`,
metadata
`854ef0c8aad75b4b200f512951d56b4dbef7aa82c46cc5e82123f0583dcc6ac5`,
verification
`455da7686eaeeb989e0d122c4724ea66ff161e8d652230eff9fb8ad20590bacd`,
group clip
`b241463658e6b48dfb2c74815e85317c2f0eccd46a7f7ab978b8f2701ce80d6d`.
The pinned author spec, visual disc OBJ/MTL, and manual-inspection hashes are
`1680cd8351fa62937c0318826f7abc75917234cb3888f983acce06f13698bc6c`,
`bc86f1ef1f5fae1510f23b1586ae20efe788c499373370a66af81b06818f1b14`,
`619352b9ac14e89a4d467dde867019e0d01540b6f11852df565f23fb26a01752`,
and `095652d3df70a144be31be49b0e25b3265df54a3815df21742333d5fdfb4529a`.
The current visual runner and runner-test hashes bound by the reseal are
`d848afa53caf14b9fb3ea061d658eef274e8d917151937bd6340283b79ab5432`
and `6e378252fa6a7cb51c6813c9d5a2b30b8c8129eacdb01df0ee19a58a270cbc5e`.
The invocation, run-summary, capture-provenance, and group-metadata hashes are
`ed0bcce48f4c70ef6fb0a6ca37dda736e0023aae2278d29fc121290d4c249ff8`,
`3970da8be868c512ff0e1aafe4eb8add2814e3a6394b4cbfe2422ad870cc7829`,
`b38c9adccd6de4d6dcd90d8fdadde9cd61234b4822dfb3eb444bba0787198e4b`,
and `a8b7f0e6d76fa1a6993aec2038b0f0bda403e79024ef8a559d9ff938f031cfc3`.
The demo and trace CMake source hashes are
`3f4c9ac29ac5bd2c7eee07e970d5ad03a9f4b3c7fce88a5582c5027c00cfa0e5`
and `4574bc5b1243c4722c2d8f1e3849715b566c5b7ab2d6b853f3659c3fc653e2d9`.
This remains non-paper, non-golden, non-timing, and non-real-time evidence.

## Finalized Author Card-House Construction Evidence

The locally finalized bundle is
`assets/paper_evidence/card_house_author_5_construction_current_v1/`.
It contains 12 indexed artifacts / 14 physical files and records the public
author default five-level, 40-card construction with four suspended cubes at
step zero. The exact-membership index, metadata, and manual-inspection hashes
are respectively
`d6cbc6f9600b8bc5c3094dd85974eae8e71d64a9e3d6e99c1783ace36be9741d`,
`b97ac795c9368f2632fe422f975914f412e8d1cb0667023e8d83c6224547df00`,
and `7bc672e9dd95b52853c5c7e56680190d564fc9514add9b263de0c33c3f94e2a4`.

This is construction-only evidence. The capture executes zero simulation
substeps and makes no claim about release, standing, trajectory, solver
behavior, contact dynamics, physical outcome, the historical four-level,
26-card paper trajectory, Fig. 6 or video parity, source renderer colors,
paper timing, or performance.

## Current-Source Four-Level Figure 6 Adapter

The separate `dart-demos` scene
`fbf_author_card_house_4_impact_current_source` and capture schedule
`card_house_author_4_impact_current_source` bind author commit
`b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0` to the source-supported CLI
selection `--solvers fbf --levels 4 --frames 600 --drop-frame 400
--num-cubes 4 --mu 0.8 --cube-size 0.4 --cube-density 500 --drop-height 1.0
--device cpu --profile --usd`. This is not the source's no-argument
five-level/800-frame default and is not a known historical paper invocation.
Source `ke=1e4`, `kd=1e3`, and `gap=.005` are recorded source semantics, not
contact semantics implemented equivalently by the DART adapter.

The DART adapter has 26 source-sized cards (20 leaning and 6 bridges) and four
initially kinematic cubes. Each cube is `0.8 m` on an edge and `256 kg`; the
interactive `p` action releases the four cubes immediately, while the evidence
runner invokes `p` after completed substep 1,600. The declared horizon is
2,400 substeps at `dt=1/240 s`. Exact FBF and boxed LCP use the same Native
`FourPointPlanar` collision frontend with contact capacity 4,096 and manifold
subdivision 4.

The demo build, 13 focused headless/continuation C++ tests, 259 visual-runner
Python tests, and exact/boxed contract-smoke validators pass. The first live strict
exact request for 100 steps fails closed at completed step 35 when contacts
jump from 44 to 68. Steps through 34 are clean, with prior worst residual
`9.826274595482653e-7`; the failing prefix records 103 exact attempts, 102
solves, one exact failure, zero fallbacks, zero accepted caps, and worst
residual `4.1039190451256334e-4`. Its timeline is
`/tmp/fbf_author_card_house_4_exact100_last_failure_current_source_20260721/timeline.json`,
SHA-256 `2d04d31134426ac2c4fc87b1774d5285b77740acaeb3ec3a005557b85944bb9d`.

The pinned source defaults `project_after_correction=false`; the baseline DART
path always projected that correction. An ABI-neutral default-on policy now
preserves existing DART behavior while this exact adapter opts out. Strict
36- and 100-step replays both stop at the same completed step 35 with identical
solver diagnostics: 56 contacts, 200 iterations, final/best residual and dual
`4.0845653576327421e-4`, primal `3.9380158679450451e-6`, complementarity
`2.3818176330330057e-4`, zero caps, and zero fallback. The timelines are
`/tmp/fbf_author_card_house_4_source_correction_exact36_20260721/timeline.json`
(SHA-256 `686be7170e3c217bfa917698a449e7ecde40e500a2c87d073ed58ba2ac833bfb`)
and
`/tmp/fbf_author_card_house_4_source_correction_exact100_20260721/timeline.json`
(SHA-256 `1a76b71fc4558c7cb978eab410a95948ae50e66522e45dbded07dd36aeb11a77`).
This closes the source post-correction mismatch but does not clear or move the
strict blocker.

The pinned author inner solver also copies the current outer reaction into
every inner solve and rejected step-size trial without projecting that seed.
An ABI-neutral, default-off source-inner policy now reproduces that behavior
only for this adapter's exact lane; DART's carried, projected inner seed remains
the default elsewhere. With both source-selected policies active, strict 36-
and 100-step v3 replays remain byte-identical in solver diagnostics and stop on
the same 56-contact group at completed step 35 after 200 iterations. Final and
best residual and dual are `4.0844850280896461e-4`, primal is
`3.9375947649884479e-6`, complementarity is
`2.3815426453852184e-4`, and accepted caps, boxed fallbacks, and line-search
shrinks are all zero. The timelines are
`/tmp/fbf_author_card_house_4_source_inner_exact36_v3_20260721/timeline.json`
(SHA-256 `8909e915b63bb2c412a5c5289a5aa690dc1a9ef1d712fe531d12a38d626f0d2e`)
and
`/tmp/fbf_author_card_house_4_source_inner_exact100_v3_20260721/timeline.json`
(SHA-256 `3e379747bac636c259fe7e9bbd711bb57d5a719d5a1d8d6b9e6317e20b639f73`).
The Figure 6 adapter and strict replay disable colored block Gauss-Seidel;
colored source parity remains unproven and separate. Source shrink-cap,
plateau, and continuation semantics are unchanged by this strict A/B and are
exercised only by the separately labeled lane below.

The pinned author control completes 2,400 substeps while reporting only 1,455
converged and 945 unconverged steps: 632 caps and 313 plateau stops. Before
release the split is 1,332/268; from release onward it is 123/677. The first
false flag is source step index 33, the first cap is 35, worst natural
`final_residual` is `2.59804445965485`, and worst per-step final checked
`r_coulomb` is `7.597910320688573`. Its history is
`/tmp/fbf-sca-2026-author/paper_examples/card-house/results/20260721T175341Z/fbf/history.json`,
SHA-256 `b67d3c86f106171008dfbb0aca0a2ca72a9d3747c1a7a6694f57f211d3f83afd`.
Therefore zero-cap completion remains DART's strict scientific gate but is not
source-equivalent continuation semantics. A separately labeled,
telemetry-rich source-continuation physical/video lane is now implemented and
captured below; it does not change the strict lane's failure verdict.

The boxed control completes 100 steps; its timeline is
`/tmp/fbf_author_card_house_4_boxed100_20260721_contract_v2/timeline.json`,
SHA-256 `fdd3d9e96058176faa51b148d1bcf5a4c0a7f1c4e7da64e15490dcae4ce6fafc`.
The additive `last_failure` record now preserves the exact failed island after
the later groups succeed: 56 contacts, 200 iterations, residual/dual
`4.1039190451256334e-4`, complementarity `2.4220067503580449e-4`, and worst
dual/complementarity local contact 11. Bounded option tuning did not produce a
strict 100-step completion. See
[FIGURE6_CONVERGENCE_BLOCKER.md](FIGURE6_CONVERGENCE_BLOCKER.md).

An unsealed GDB-mutated accepted-cap preview reaches release and all 2,400
steps, but 1,106/3,231 solves cap and worst residual reaches
`0.61608914241359314`. It is finite continuation only. The strict lane still
does not reach release, and boxed is bounded to 100 steps. No valid strict
physical-outcome verdict, source-backend or timing equivalence, captured media
or PR video, paper trajectory, Fig. 6/video parity, or exact-versus-boxed
superiority claim exists. Keep the older reconstructed
`fbf_paper_card_house_26` scene and its evidence distinct.

### Source-Continuation Figure 6 Capture (Not Strict Convergence)

The separately named scene and schedule
`fbf_author_card_house_4_impact_source_continuation_current_source` and
`card_house_author_4_impact_source_continuation_current_source` preserve the
same 26-card/four-cube geometry, Native `FourPointPlanar` frontend, 2,400-step
clock, and step-1,600 `p` release. Only the exact lane requests the explicit
source-continuation policy; the paired boxed lane uses the same scene, action,
and capture schedule without exact-only policy metadata.

The validated local capture root is
`assets/paper_evidence/fig06_card_house_source_continuation_current_v1/`;
generated frames and media remain outside Git. The current-contract reseal was
produced from `/tmp/fbf_fig6_source_continuation_pair_20260721T1414_v2/`. Both
lanes complete 2,400/2,400 steps and the release
action succeeds at step 1,600. The exact lane records 3,351 attempts and 3,351
solves, zero exact failures or boxed fallbacks, 2,605 ordinary successes, 113
`plateau_accepted` outcomes, 633 `max_iterations_accepted` outcomes, and zero
line-search shrink caps. The 746 continuation accepts are 22.262% of 3,351
solves and occur across 723 steps.
The cumulative worst final residual is `0.91712002943322535`, first reached at
step 2,101. These accepted finite iterates make the full run
source-continuation evidence, not a strict-convergence trajectory.

Independent inspection of the synchronized 301-frame, 10.033333-second clip
finds that both houses stand through release. Exact and boxed are pixel-identical
only at step 0; viewport difference is already 0.165% at step 1,600 and reaches
11.985% at the endpoint. In this DART source-parameterized four-level scene,
the exact source-continuation lane completes without exact-solver
failures/fallbacks and visibly retains more upright card-house structure after
impact than DART boxed LCP. The official MuJoCo panel degrades while settling,
whereas DART boxed remains upright until impact, so do not map DART lanes to the
paper lanes or infer a mechanism. This is not a quantitative trajectory or
physical-outcome equivalence result, approved golden, source collision or
solver-backend equivalence, timing result, or solver-superiority result. The
capture summary records `paper_comparable=false` and no automated
semantic-outcome validation.

Integrity anchors are:

- run summary:
  `6888f4729c99d41753c9c8ec9a1ec2ec9e2367c71da76aab973f8f8c5e8674cc`;
- exact timeline:
  `a9eb12711419b7801037d17059560559893be2898e07d14425a5f572175482ff`;
- boxed timeline:
  `1618e284f97ff7ed49e3288636269f5bea6131faa3bae45428e42e23de660bd8`;
  and
- paired clip:
  `282aebfb9e2e38fe3741db28e2ce909fb548d7aa46d048302a3b0e0bea9e1786`.

The freshly downloaded official video at
`/tmp/fbf_official_5THad4PAGmI_360p.mp4` has SHA-256
`d5356e1b31487be62b75af05efbfecdb70ad5d98501a8efd378fcedf066e4794`,
exactly matching the audited source-video hash. The paired DART clip is only a
local PR-attachment candidate until a `github.com/user-attachments/...` URL is
recorded. The strict step-35 blocker remains open.

## Pinned-Author Masonry-Arch Scientific Negative

The locally sealed current-source bundle is
`assets/paper_evidence/author_masonry_arch_reference_v1/`.
It records the pinned author invocation with 500 frames, four substeps per
frame, and cube release at frame 400 / substep 1,600. The source default is
400 frames with `drop_frame=400`, so it never releases the cubes; the recorded
run is a newly declared diagnostic, not a historical or paper invocation.

A deterministic projection represents every one of the 2,000 substeps and is
lossless with respect to the declared claim fields. The 382,753,953-byte raw
source history (SHA-256 `cec0e4b86837e7542c498c7ddad40538983ec023332b88ebddee7766997e3ac1`)
is size/hash-bound but omitted. Of 157 true author convergence flags, 40 use
the initial natural-residual shortcut and 117 use the configured outer
nonnegative `coulomb_rel < 1e-6` gate; 1,843 outer solves are nonconverged,
including 1,458/1,600 substeps before release and 385/400 after release. The
projected `final_residual` is a different, natural-residual metric: only 47
values are at or below `1e-6`. Release substep 1,600 is nonconverged with 100
contacts and natural residual `0.017456069692858667`; the final substep is
nonconverged with 108 contacts and residual `0.5161195175386001`. A
contact-count increase at substep 1,944 is only inferred because the projection
contains no pair identities.

Exit code zero and bundle integrity validate this sealed scientific
negative, not solver success. The shared DART specification is configuration
only: it records the author geometry and solver constants but explicitly does
not implement the source collision/contact-gap/backend/float32 semantics or
execute DART dynamics. No DART or cross-solver dynamics, trajectory, physical
outcome, Fig. 7/video.07, timing, repeatability, or media parity is established.

The separately named current-source DART adapter now executes that raw-scale
configuration through exact-FBF or boxed LCP and declares the runner's
2,000-step schedule without changing the frozen source or reconstructed-impact
contracts. A clean current-build exact run clears the former step-68 local-QP
failure and completes 100/100 steps: 124/124 exact attempts solve, with zero
accepted caps, exact failures, or boxed fallbacks and worst residual
`9.9936331058309156e-7`. A 200-step fail-fast run reaches a distinct outer
convergence blocker at completed step 142, when contacts rise from 88 to 96:
all 211 local exact attempts solve, but the outer loop exhausts 5,000
iterations with residual `8.6992951837150444e-4` and one accepted cap. These
local runs are diagnostic only: no full schedule, cube release, impact oracle,
promotable media, or Fig. 7 parity follows.

## Source-Pinned 101-Stone Standing Scientific Negative

The ignored packet is
`assets/paper_evidence/fig08_arch101_author_current_v1/`. Scene
`fbf_author_masonry_arch_101_standing_current_source` binds the pinned author
commit, `--stones 101`, 101-mesh tree
`e0c209235673d2f69c3c5de7708ab1dfadec96e3`, and path-manifest SHA-256
`7198f71730d06dd70af8703065541765bd6b6f5da137f28f9befdf7acc5f96bf`.
It contains 101 tapered meshes, 99 mobile stones, two fixed springers, and three
pinned cubes. The source-supported 400-frame, four-substep schedule ends at
`drop_frame=400`, so its 1,600 substeps / 6.67 s include no release.

Strict exact fails closed after 209/1,600 completed steps on an iteration cap:
208 contacts, 5,000 iterations, residual `1.2582804496066107e-6`, 342/342
attempts/solves, one accepted cap, zero exact failures, and zero boxed
fallbacks. Its incomplete 210-sample state trace fails the standing oracle;
the crown falls from `66.1385625` to `62.4010546875`, and maximum mobile-body
displacement is `3.7375078125`. Boxed completes all 1,600 steps with a valid
1,601-sample inventory/finite/cube-pinning trace, but also fails standing:
maximum displacement `21.2188459736`, maximum rotation `3.14152663339 rad`,
and crown minimum/final `58.3806809854` / `61.1013192467`.

The boxed 660x506 H.264/yuv420p clip has 201 frames at 30 fps over 6.7 s,
fully decodes, and has SHA-256
`7635c2722b20fb8bcb0255054cc9172153d1dd640fd8e81df4df52c0e515d3c0`.
Manual inspection finds the arch intact through step 400, losing its
crown-standing configuration by step 800, and visibly collapsed by steps 1,200
and 1,600. A
1320x506 diagnostic hstack freezes exact at its last step-208 rendered frame
while boxed continues; its SHA-256 is
`d6f5f658e4fb027edb23e0911acd34b74dfd749daace41b5d9c9204af3163b94`.
Capture and independent boxed reuse verification pass. The compact summary
SHA-256 is
`1c19c6c3c36171a5e85f330b2863b429956652fb894aae0aa0b82d68291e3481`;
exact and boxed timeline SHA-256 values are
`df1ed4afc9ef5aa74f7c0b6da0560ae0d1b63fca28f45051ed27c5dfb3632889`
and `a8caee71c9356a72fa65210207d7b4209d9e305363974ec07c81f19ec14bfa1e`.

This is a current-DART scientific negative and a local PR-attachment candidate,
not a complete exact/boxed comparison or evidence of source backend/float32,
trajectory/outcome equivalence, Fig. 8/video.08 parity, matched Kamino,
approved goldens, timing, performance, or solver superiority. GitHub URLs
remain pending. The frozen literal 101-stone v7 negative below remains a
separate reconstruction contract.

## Preregistered Crown-Impact v1 Negative

The frozen non-paper contract and result are documented in
[LITERAL_CROWN_IMPACT_V1.md](LITERAL_CROWN_IMPACT_V1.md). The final locally
sealed bundle is
`assets/paper_evidence/fig07_arch25_literal_impact_v1_negative_final_v9/`.
It launches three fixed 35 mm cubes after the unchanged 600-step standing
prefix, then observes 120 impact steps. The prefix matches the standalone
standing scenario across 88 eligible fields with zero mismatches.

The run completed 720/720 steps, stayed finite, and recorded first arch
contact at step 607 before first ground contact at step 616, with zero exact
failures or boxed-LCP fallbacks. It nevertheless failed the preregistered
claim: five solves were accepted at the cap, worst residual was
`9.154531704265396e-5`, final maximum arch displacement was
`0.07093964431215687 m` versus a `0.07 m` limit, and far-field displacement
was `0.060523747030465196 m` versus a `0.007 m` limit. The runner requires
this fail-closed pattern and sets `impact_claim_passed=false`; it rejects a
zero child exit or a passing-impact interpretation. The current runner recomputes
all gates from finite metrics, requires post-launch exact progress, pins the
normalized fingerprint and frozen preregistration hash, and stores the
independent 600-step by 88-field standing reference. No parameter or threshold
was changed after observing the result.

Final lint-clean bundle hashes are:

- runner: `622b388142fed881191dcb6efb103266eb2ea1e3ffbadaf402f63b89c558bb67`;
- preregistration contract: `4cdea674f366fc2d18eadf11ef4333d491786d5d85e3fc16fa611ea7dede3f37`;
- normalized fingerprint: `86b37ec37d28259514453c9ecc9e7d5f12afe08118a0b04abd40e72acd147384`;
- standing reference: `22fabfe2e8f35bf329f32873845a60599ed2df171c15728f14311b91d888c387`;
- `raw.csv`: `42cc94f5f111442da42b03af33722f713d66b853d3ac21f21083f28ba24e7ba4`;
- `stderr.txt`: `7964b07fd5396f10013ff8d9100d2b36e7dfc151764210d8c60bd0ae853a2d94`;
- `summary.json`: `e3baa479111a999d03359a0642533cad1baa3a48c415609bafb9a7a7cfa18e7c`;
- `metadata.json`: `0bd19797953a10223bbd7183f6e8002e490b21c29b2304befee7e6618ff9ee73`; and
- `REPORT.md`: `c5711b0fe06e679f889f6a7ca3812aa955c3b8c61270ef038def06b2a3f173ff`.

The v6, v7, and v8 paths are historical current-at-capture evidence. The v9
rebaseline has the same frozen negative semantics and records the executed
`taskset` identity in its runtime closure.

## Frozen Literal 101-Stone v1 Negative

[LITERAL_ARCH_101_V1.md](LITERAL_ARCH_101_V1.md) freezes the separately
labeled Native exact-inertia reconstruction before its authoritative run. The
current provenance-bound bundle is
`assets/paper_evidence/fig08_arch101_literal_v1_negative_final_v7/`.

The run fails closed at step 1 after all 5,000 outer iterations. It records
aggregate dynamic `FourPointPlanar` fields of 400 contacts, 100 constraint
pairs, three colors, width 34, and four colored participants on P-cores
`8,10,12,14`, plus one exact failure, zero fallbacks, and residual
`0.78153646143524735`. Separately, the repeat-2 collision-only Compact probe
proves the constructed time-zero graph has 102 pairs: 100 adjacent-stone pairs
and two springer-ground pairs. The v7 one-step FourPointPlanar companion
resolves the failed step-1 pre-solve graph as exactly the 100-edge
adjacent-stone chain: 100 unique adjacent pairs, 400 contacts, multiplicity
four, zero non-adjacent pairs, and zero ground pairs. Its aggregates and
residual match the frozen trace. The companion accepts the capped iterate and
does not follow the frozen trace's participant-affinity contract, so the
solver acceptance taxonomy and participant-affinity contract are not
equivalent. The identity result is limited to the failed prefix and does not
promote source equivalence, a valid trajectory, standing/physical outcome,
timing, media, long-run behavior, or paper parity. The runner sets
`artifact_valid=false`,
`standing_claim_passed=false`, and `timing_evidence_eligible=false`; no media
may be promoted. No v1 parameter or threshold was changed after the result.

The runner binds and rechecks the protocol, runner, trace/collision/dynamics
probe sources and executables, `taskset`, `ldd`, and every resolved regular
shared-library file. This is source/executable/shared-library-bound
provenance, not a claim over all host runtime state. The earlier
`assets/paper_evidence/fig08_arch101_literal_v1_negative_final/` bundle is
retained only as invalid historical evidence because it lacked those
shared-library and independent graph bindings.
The v2 bundle is provenance-complete historical evidence, but a
clang-format-only collision-probe source identity change superseded it for
current-source claims. The v3 bundle was superseded after additive
card-sensitivity instrumentation, and v4 is historical current-at-capture
evidence. The unchanged command was rebaselined as v5 and again as v6 after the
current-build libdart identity advanced. V7 adds the identity-resolved
one-step dynamics companion; the frozen trace and scientific result are
unchanged.

The normalized fingerprint is
`8d275edcaa82a2f628fdd5d9f846b2daed7e6864fdc4b7e80eca9a8c93bcf527`.
Current hashes are runner
`7155b9bc6082e79aca317be6626fea587b58538df2f34e94f865cd54c15eb993`,
raw `fc0705ff497b44658dfe186ed27c50b2ab9d19a268f20cf50875a3155d1dc07d`,
summary `2cae961048b776c069caeccda2d95f2f0fd0969cae9e3de3782f0e5e5b7b640d`,
metadata `770d662f7021320982700fe699266bc94d2448679c8c4dc60bd127c660facb8a`,
report `1ba3987af896f68401faa1153f6b388aed38b087214d07592f5a060cab429e2a`,
and whole tree
`e65107a6d2e2880742e02d01949bc4b3f9adb2f48b6be552a720fa83c9bcc4b3`.

## Current Card-Manifold Sensitivity v2

[CARD_HOUSE_MANIFOLD_SENSITIVITY_V2.md](CARD_HOUSE_MANIFOLD_SENSITIVITY_V2.md)
freezes a one-factor Native `Compact`-versus-`FourPointPlanar` comparison for
the reconstructed 26-card scene. The current provenance-complete bundle is
`assets/paper_evidence/card_house_26_manifold_sensitivity_v2_r3/`.
It holds every `paper_cpu` solver knob fixed and excludes raw wall time from
all scientific verdicts.

Both modes emit all 600 requested rows with zero exact failures or boxed
fallbacks, but both have zero strict-success rows and accepted capped groups on
every row. `Compact` records 3,495 capped groups across 5,757 exact attempts and
a successful terminal last group at residual `8.525678738415048e-7`.
`FourPointPlanar` records 682 capped groups across 745 attempts and exits
through the represented terminal convergence gate at residual
`0.016582575623909489`.

The preregistered directional hypothesis is supported: `FourPointPlanar`
increases mean contacts by `93.7983333333` and mean pair multiplicity by
`1.9548548971`. Strict convergence is not improved. Both trajectories remain
non-strict, so physical, timing, real-time, and paper-parity verdicts are null.
The earlier prior-source strict paper-profile artifact remains a separate
step-89 fail-closed negative, retained in the ignored local evidence cache at
`assets/dart_cpu_evidence/2026-07-12_prior_source_paper_cpu_card600_negative/`;
v2 does not replace or relabel it.

Current SHA-256 values are protocol
`eeb1c8c1d09a29e197a3c402217ecd0af9a9878749cb4756cf94bc714f2b60e9`,
runner
`e03356c772560f061e9b90fb4cd9f5df0c569631cd5e9fdd0857c337ff840562`,
trace source
`b00eea0c87f75f17259fac433bafd98c20a961d14389a80e91742d3c8a678f76`,
trace executable
`0923bf7df1eaa518f9a2ffacd0a42fd7330f5cbf6c35cb5749fbc122db1310ff`,
summary
`52a082ab15e8b9c314d706474cc7be557ddfc58c4961faad0d3da9d347f59f4f`,
comparison
`051605c25ccd5aa4de2f243c4dafe547c82f7298f8018cee53a8701d018ff297`,
metadata
`5890ab138179f4d7aaee6cd04c63799086439bae58fbde4f994048785dd0b8ac`,
artifact index
`1703f995bd5d2c6edeb12ba936010a80609c3e952f2c782b554bd0be7ca40627`,
invocation
`6e48eb835ab9aafca0b8d4bd29acb5ec932645fb37ca6124ce3c23b3265847f0`,
report
`0429b12f90e4fef534c9b707324a180eaac78829a3b3667505111228aba58d2e`,
and whole tree
`953d8efd4d43ee2a74cede6b5e3a0a766ecd1fd8914485b50b340f515b7ecb77`.
The unsuffixed v2 path is historical current-at-capture evidence; v2_r3 is the
current-source rebaseline with unchanged diagnostic semantics.

## Historical Negative Diagnostics

Keep these results as solver-history evidence, not as the current result:

- Under the older scale-1 profile, the literal arch needed 28,981 cold outer
  iterations. A standard 200-outer continuation passed steps 60 and 180,
  capped near 240, and collapsed by 300/600. A 2,000-outer continuation was
  stable through 300 but capped near 420/480 and was too slow.
- Local-diagonal seeding reduced that cold count to 22,726; dense-global
  seeding gave no material benefit, and velocity ERP `0.01` or `0.1` worsened
  or failed convergence.
- The strict paper-parameter card bootstrap reached residual `0.004311` after
  30,000 outers in `15.167 s` on step 1, then residual `0.026233` after 200
  outers in `0.136 s` on step 2. It failed residual, trajectory, and timing
  contracts despite zero local-kernel failures and zero fallbacks.

None of those historical rows is a paper parameter match or current
performance artifact.

## Current Recorded Local Gates

The latest recorded focused results on the current source are:

- exact-Coulomb math: 56/56 in Release;
- exact constraint solver: 32/32;
- `ConstraintSolver` integration: 64/64;
- Native collision: 42/42;
- masonry wedge dynamics: 3/3;
- default paper fixtures: 31 passed, 3 explicit stress cases skipped;
- focused Release and Debug CTest matrices: 9/9 in each configuration;
- schema-v8 CPU evidence tests: 230/230;
- literal-wedge visual finalization tests: 16/16;
- crown-impact trace and negative-runner tests: 25/25;
- literal 101-stone trace/probe/runner tests: 41/41;
- finalized incline finalizer unit tests: 62/62; with the compact local bundle
  present, verify-only passes with status
  `valid_current_source_nonpaper_incline`, 21 indexed artifacts, and no raw
  capture-staging dependency;
- author-incline reference finalizer unit tests: 64/64; verify-only passes with
  status `valid_current_source_scientific_negative`, 37 indexed artifacts, and
  39 physical files;
- focused manifest/backspin/incline/author-masonry/author-incline evidence
  suite: 859 passed post-merge in 163.95 s;
- full no-cache dartpy Python suite: 1,555 passed in 165.09 s;
- current-source four-level author-card demo build: passed;
- current-source four-level headless/continuation C++ fixture tests: 13/13;
- visual-runner Python tests, including the source-pinned 101-stone
  schedule/oracle contracts: 320/320;
- current-source four-level exact/boxed adapter contract-smoke validators:
  passed;
- author masonry-arch C++ specification/adapter: 1/1 focused CTest target and
  7/7 contained tests passed;
- demo scene documentation verifier: 26 scenes passed;
- manifest validation: under the sealed producer closure, live mode performed
  118 file-identity rechecks with zero skipped; explicit archive mode reported
  all 118 skipped rechecks; and
- deterministic colored-scheduler stress: 1,000 runs passed.

The sealed live closure resolved `libdart.so.6.19` to the recorded
`libdart.so.6.19.3`. The normal development symlink is restored to
`libdart.so.6.19.4`; the two files are byte-identical, but the fail-closed
validator includes the resolved path in identity and currently reports five
live path mismatches. Use explicit archive mode for the sealed-tree check, or
recreate the recorded symlink only for an intentional live-closure recheck.

## Immediate Next Work

1. Preserve the authoritative P-core, standing-visual, frozen impact-v1, and
   frozen 101-v1 negative bundles. The 101 reconstruction is now precisely
   blocked at step 1; preserve the separate source-pinned 101-stone DART
   step-209/collapse negative and its independently verified boxed clip without
   relabeling the frozen-prefix hstack as a full exact comparison. Card-manifold
   v2 is complete as non-strict diagnostic evidence. Preserve the finalized incline, the pinned-author numeric
   incline sweep, historical current-source Painleve proxy, finalized
   source-pinned Painleve adapter, and backspin bundles. Preserve its
   trace/outcome audit, independent replay, and manual inspection, upload the
   two exact-vs-boxed clips through the PR browser composer, then
   continue with the other
   smaller-figure recaptures and strict card work. Continue isolated
   one-factor-at-a-time source-mismatch A/Bs without combining tolerance, cap,
   fallback, or fail-fast changes. Preserve and independently review the
   completed telemetry-rich source-continuation clip, do not relabel it as
   strict convergence or solver superiority, and publish it only through the PR
   editor after explicit approval. Continue
   source-equivalent impact and strict source-pinned 101-stone work
   without tuning
   either frozen v1
   contract or promoting the v2 raw timings.
2. Keep all 29 manifest rows synchronized with every new sidecar, hash,
   semantic verdict, and report binding from one identified build.
3. Keep the parity matrix and PR-facing report synchronized with that evidence.
4. Run the final integrated gates and obtain two clean independent reviews.
5. Only with explicit approval, update the branch, PR body, and current-head
   CI.

## Official Sources

- project page: <https://www.cs.ubc.ca/research/fbf-friction/>;
- paper: <https://www.cs.ubc.ca/research/fbf-friction/paper.pdf>;
- video: <https://www.youtube.com/watch?v=5THad4PAGmI>; and
- named repository: <https://github.com/matthcsong/fbf-sca-2026>.

## Completion Bar

Do not mark the task complete until every paper requirement is reproduced or
precisely blocked, every exact trajectory satisfies its full residual,
fallback, and physical-outcome contract, current-build visual artifacts are
fully decoded and manually inspected, performance metadata proves each local
claim, source comparability is either matched or its remaining historical gap
is stated precisely, lane-specific count mismatches and equivalence boundaries
remain explicit, the
manifest and reports agree, default and downstream behavior remain covered,
and final independent reviews are clean.

For PR-facing wording and ongoing synchronization of the truthful draft PR
body, use [PR_REPORT.md](PR_REPORT.md).
