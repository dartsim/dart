# Agent Continuation And Truth Ledger

This is the authoritative status for the active
`fbf_exact_coulomb_friction` dev task as of 2026-07-19. It supersedes older
solver, performance, geometry, GUI, PR, and completion claims in this task
folder. Do not mark the task complete or retire the folder.

## Required First Answer

Until every completion gate below is satisfied, the answer to "Do we have all
tests, benchmarks, and GUI examples from the paper?" is:

```text
No.
```

The current work reconstructs the method in DART. The authors' public
Warp/Newton reference implementation is pinned at
`b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0`, but DART does not yet reproduce
its scenes or trajectories source-equivalently and does not reproduce the
historical renderer, Apple hardware, or paper timer boundary.

## Truth Ledger

| Claim | Current evidence | Verdict |
| --- | --- | --- |
| Public author reference | The MIT-licensed repository is available at pinned commit `b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0` with the solver, six runnable examples, current configurations, pinned dependencies, and optional MuJoCo/Kamino runners | Available and source-auditable; porting and matched runs are now internal work |
| PR #3374 | Merged at `fa17fad` | Required visual infrastructure is available on the release branch |
| PR #3377 | The topic contains `origin/release-6.20` through `75306efe770`; topic head, divergence, merge state, checks, and reviews are mutable and must be queried live; body overclaims completion and performance | WIP, not completion evidence |
| Exact algebra | Row signs validated; row-operator versus impulse-test `W` relative error `1.33e-16`; spectral-nullspace regressions pass | Tested formulation is internally consistent |
| Literal 25-stone exact dynamics | Schema-v8 mark26 has eight complete 600-step processes: one warmup plus three measured runs at each of one and four threads | Full local exact trajectory is proven for the reconstructed non-paper scene |
| Exact solve contract | 1,800 measured steps per thread count; maximum residual `9.999807145410957e-7`; zero exact failures, caps, or fallbacks | Pass |
| Physical outcome | 96 contacts and 24 colliding pairs throughout; physical outcome valid and identical at one and four threads | Pass for the declared reconstructed scene |
| Colored inner BGS | 24 manifolds, 3 colors, width 8; four-thread runs observe the pinned four-CPU colored phase and pass the scaling-pair contract | Real-scene multicore execution and matched-work speedup proven locally |
| Local mean real time | 1-thread mean `6.122883343333333 ms`; 4-thread mean `4.26939745 ms`; validated speedup `1.4341328993236115x` | Both means pass 60 Hz |
| Every-step deadline | Maxima are `287.473818 ms` and `180.504588 ms` | Neither thread count passes an all-step 60 Hz deadline |
| Paper timing | The current author workload and kernel are inspectable, but DART's engine, precision, host, and timer boundary are unmatched and the historical Apple/timing attestation is absent | Paper timing target and verdict remain null |
| Current small CPU matrix | Repository-finalized current-source `paper_cpu`/Native bundle binds 60 artifacts, 27 complete CPU-4 invocations, and 5,220 rows | 9/9 physical classifiers pass; 7/9 strict-solver/local-real-time contracts pass; zero warmups and unmatched paper contracts prohibit a paper timing claim |
| Literal 25-stone visual evidence | Finalized current-source bundle has 19 indexed artifacts / 21 physical files, five durable stills, a 61-frame decoded clip schedule, 600 zero-difference trace rows, and bound manual inspection | Pass for the reconstructed no-projectile scene; the 6.1 s clip is a 1.639344x time-lapse |
| Incline visual evidence | Finalized `fig01_02_incline_current_v1` has 21 indexed artifacts / 23 physical files, five durable stills, a 61-frame decoded clip schedule, two independent 121-row traces, manual inspection, and byte-identical aggregate exact-solve/fallback projections | Valid current-source non-paper threshold evidence; capture contacts 8 versus aggregate trace contacts 6 are explicitly not compared, and Fig. 1/2 plus video.03 remain partial |
| Pinned-author incline sweep | `author_incline_sweep_reference_v1` preserves separate current-source FBF, MuJoCo, and Kamino CPU runs on `mu=.3,.4,.45,.5,.55,.6,.8`; every lane has seven 120-step cells, and the retained FBF histories record four contacts per FBF step | Numeric source-pinned scientific-negative/reference evidence only; FBF records 839/840 configured convergence flags, timing is excluded, and no DART/full-state/historical/golden/media/timing/performance/parity claim follows |
| Painleve proxy visual evidence | Finalized `current_v1` repository bundle has 27 indexed artifacts / 29 physical files, two 151-row traces, fully decoded paired media, and bound manual inspection | Valid DART-side nonpaper proxy evidence only; rendered demos and tracked fixtures are not trace-equivalent, and paper/external/golden/timing/real-time/strict-rest claims remain unproven |
| Backspin visual evidence | Finalized `fig03_backspin_current_v3` has 18 indexed artifacts / 20 physical files, three durable stills, MP4/GIF media, 129 exact attempts/solves, zero caps/failures/fallbacks, a corroborating translational trace, and a passing manual inspection of the renderer-applied high-contrast 6x4 ivory/charcoal checker texture and coral registration tile | Valid current-source DART evidence only; `fig.03` and `video.02_backspin` remain partial |
| Author-pinned turntable visual evidence | Finalized `fig04_turntable_author_current_v1` has 58 indexed artifacts / 60 physical files, four timeline-bound outcome stills, and four complete 360-step author-configured visual-lane cells: three eject and `mu=.5, omega=2` remains on support through 6 s | Valid finite-horizon author-source-pinned non-paper DART evidence; no zero-slip, co-rotation, full-state, paper-golden, timing, real-time, or parity claim |
| Author card-house construction | Finalized `card_house_author_5_construction_current_v1` has 12 indexed artifacts / 14 physical files and shows the public-author default five-level, 40-card configuration at step zero | Construction-only evidence; zero simulation substeps and no release, standing, dynamics, solver, physical-outcome, Fig. 6/video, timing, performance, or parity claim |
| Pinned-author masonry arch | `author_masonry_arch_reference_v1` records the 500-frame / 2,000-substep author run with release at substep 1,600; a deterministic projection represents every substep, with only 157 author convergence flags true and 1,843 false | Valid source-pinned scientific negative; not the 400-frame source default or a paper invocation, and not DART/cross-solver dynamics, trajectory, outcome, timing, repeatability, pair-contact, or media parity |
| Reconstructed crown impact | Frozen three-cube v1 reaches the arch before the ground and stays finite, but fails cap, residual, whole-arch, and far-field gates | Durable scientific negative; `impact_claim_passed=false`, no tuning |
| Card-house manifold sensitivity | Current-source v2 emits 600 rows for Compact and FourPointPlanar; both are non-strict, while FourPointPlanar raises mean contacts by `93.7983` and mean multiplicity by `1.95485` | Valid one-factor diagnostic only; physical, timing, real-time, and paper verdicts remain null |
| Paper-media parity | The passing literal bundle contains no projectile; source-matched impact media and 101-stone coverage remain missing, and the public author repository ships no historical camera/material/golden render bundle | No paper impact, GUI, or golden-frame parity claim |
| Manifest | Current audit: 29 rows, 24 partial, 5 blocked, 0 complete; six durable local visual bundles are finalized and the visual workflow declares 20 schedules | Validator passes; overall status honestly remains partial |

## External And Comparison Boundary

Official project page: <https://www.cs.ubc.ca/research/fbf-friction/>. The
MIT-licensed public reference at
<https://github.com/matthcsong/fbf-sca-2026> is pinned for this audit at commit
`b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0`. It contains the Warp/Newton FBF
solver, all six current paper-example runners and configurations, a pinned
Python dependency snapshot, optional MuJoCo/Kamino runners, and masonry-arch
meshes. The local exact 3x3 cone solve, block-GS policy, warm-start matching,
and cross-step gamma controller are therefore source-auditable.

Only the masonry-arch mesh assets are shipped. The public repository does not
ship the historical paper renderer setup, cameras, materials, or approved
frame goldens, and it does not identify the exact historical Apple-silicon
host or retain the original paper invocation/timing logs and
warmup/aggregation attestation. Current author invocations were independently
run and preserved, but do not fill those historical gaps. Mark26 remains
reconstructed float64 DART on x86-64 Linux with DART collision and
`World::step()` timing. The paper target is deliberately blank and its timing
verdict is null. Do not call the local result a source match, paper match, or
win.

The preserved author incline runs cover the complete current Figure 1 grid in
the public FBF, MuJoCo, and Kamino lanes, but they are numeric current-source
diagnostics rather than historical paper evidence. Their first-use JIT work,
always-on history collection, ineffective warmup exclusion, and
lane-dependent timer boundaries prohibit a timing comparison.

## Git And PR Reality

- Preserve the broad dirty worktree. Inspect scoped diffs before editing and
  do not revert unrelated changes.
- PR #3374 is merged at merge commit prefix `fa17fad`.
- The topic contains `origin/release-6.20` through `75306efe770`. Fetch origin
  and verify target ancestry before every publication; do not hard-code the
  topic head or divergence in owner docs.
- Query PR #3377 live for its head, merge state, checks, and review state.
- The live #3377 body describes card/arch coverage and performance more
  strongly than current evidence permits. Use [PR_REPORT.md](PR_REPORT.md) as
  the replacement source when a PR update is explicitly approved.
- Do not commit, push, edit the PR, rerun CI, post comments, or otherwise
  mutate GitHub without explicit maintainer/user approval.
- Before any approved follow-up push, fetch the latest real target branch and
  merge it if it is not already an ancestor; do not rebase unless explicitly
  requested.

## Durable Mark26 CPU Evidence

The authoritative bundle is
[`assets/dart_cpu_evidence/2026-07-19_mark26_native25_colored_v10_archwide_pcore/`](assets/dart_cpu_evidence/2026-07-19_mark26_native25_colored_v10_archwide_pcore/).
Its metadata records one sequential warmup and three sequential measured
600-step processes for each thread count. The measured aggregate is 1,800
steps for one thread and 1,800 steps for four threads. The 4,800-row raw CSV
also preserves both warmups; summary timings exclude those warmup rows.

Declared workload and solver contract:

- `masonry_arch_25_literal_wedge`, Native FourPointPlanar, `1 um` closure;
- exact FBF, float64, no boxed-LCP fallback, no accepted capped solve;
- fixed 30 inner sweeps, maximum 5,000 outer iterations, step-size scale 35,
  outer relaxation 1.1, and fresh rather than persistent step-size state;
- deterministic manifold-colored inner BGS; and
- one-thread CPU 8 and four-thread CPUs 8, 10, 12, and 14, one logical CPU per
  matched physical core.

Every measured step has 96 contacts and 24 colliding body pairs. The schedule
has 24 manifolds, 3 colors, and maximum width 8. Exact FBF succeeds on every
step; the maximum observed residual is `9.999807145410957e-7`; exact failures,
accepted caps, and fallbacks are all zero. Every measured trajectory passes
the declared physical-outcome contract, and the one- and four-thread outcome
fingerprints are identical.

| Threads | Measured trajectories/steps | Mean ms | Median ms | p95 ms | Max ms | Mean 60 Hz | All-step 60 Hz |
| ---: | ---: | ---: | ---: | ---: | ---: | --- | --- |
| 1 | 3 / 1,800 | `6.122883343333333` | `2.4966535` | `21.663236899999994` | `287.473818` | Pass | Fail |
| 4 | 3 / 1,800 | `4.26939745` | `1.9047965` | `14.396602399999995` | `180.504588` | Pass | Fail |

The matched-work validated speedup is `1.4341328993236115x`. This proves
mean-real-time local throughput and multicore scaling for this reconstruction.
It is not a worst-case real-time guarantee and proves neither author-scene
parity nor the paper's impact/media results.

## Schema-v8 Evidence Contract

`scripts/run_fbf_cpu_evidence.py` uses schema version 8. It preserves the
default 83-column trace contract byte-for-byte; the newline-terminated default
header SHA-256 remains
`396c866d782e626b5c5b52e74c392de1175f5e46dfd90b809d1f219417092e50`.
The explicitly non-paper colored contract emits a separate 95-column trace.
Its newline-terminated header SHA-256 is
`424195336cb42753179130c9f6fcba3a8ddea9bf669cfa70c0f77fcb4c6335a5`.
The opt-in Native manifold-sensitivity contract emits a separate 94-column
trace with newline-terminated header SHA-256
`007311fb28062377dd6a0d26cad1ab4f7e2c99f359afd33554651f3cc0929ef5`.
The final two columns record maximum all-stone displacement and minimum
all-stone orientation alignment from constructed t0, so the literal-arch
outcome no longer relies only on the tracked crown.

Both thread counts have measured-work fingerprint
`9d8df2edba609314432ff17f63768fded23577703537040d75ce082ab4233a36`.
The trace binary SHA-256 is
`0923bf7df1eaa518f9a2ffacd0a42fd7330f5cbf6c35cb5749fbc122db1310ff`.
The authoritative bundle file SHA-256 values are:

- `artifact-index.json`: `a60899cc12a53f03424c02c2647f233d5c75f3ccce367ea2a604f9a7ee18bf11`;
- `metadata.json`: `5507ed80140a146d4247c4f0b05fd9503879ce79856189a15759b101c2cab789`;
- `invocations.json`: `719ba3491fad8ac12aa290faa401d0c46b10af52da7eb7a28487a5b2aac44812`;
- `raw.csv`: `91a379f832ca52bbce7011308640012d9f199b98e39cba4eaf661cf17fb0f017`;
- `summary.csv`: `3c08b251c340aa9ae7909d715ebcc2985ba34350a6e03b0b6b351d82d1ec82a2`;
- `summary.json`: `304736d6b871c4498a6c0de4c4448635e712fb3a8a455dd54d9ffefeef2ec170`; and
- `REPORT.md`: `4e55d7f4dc0532ab15b86cfeb72ea9526d3f9b63194dc5a25f3974186b1a7ba7`.

The colored contract fails closed on:

- completed warmups and measured trajectories;
- exact residual, failure, cap, fallback, and physical-outcome gates;
- colored-path use, persistent dispatch, participant count, schedule width,
  and same-phase CPU residency in every measured repetition;
- controlled taskset affinity and distinct physical-core mapping; and
- nested one-thread affinity, matched options, topology/core class, and exact
  measured-work trajectory fingerprints.

Requested threads, dispatch counts, or lifetime CPU unions alone remain
insufficient. Mark26 passes the applicable one-thread contract and the
four-thread multicore/scaling-pair contract.

The P-core bundle path, timings, speedup, fingerprint, and hashes above are the
current CPU ledger. The separate E-core high-load run is a diagnostic
negative, not a replacement for this canonical result.

## Current Literal-Arch Visual Evidence

The current-source visual bundle is
[`assets/paper_evidence/fig07_arch25_literal/`](assets/paper_evidence/fig07_arch25_literal/).
The off-screen capture independently mirrors the current
`masonry_arch_25_literal_wedge` trace contract. Its equivalence audit compares
600 rows and reports no integer mismatches and zero maximum absolute
difference for every compared floating-point field.

| Property | Current evidence |
| --- | --- |
| Exact trajectory | 600/600 steps, zero exact failures, zero boxed-LCP fallbacks |
| Worst exact residual | `9.9998071454109575e-7` |
| All-stone displacement from constructed t0 | maximum `5.431169776791696e-6 m` |
| All-stone orientation alignment from constructed t0 | minimum `0.9999999999111284` |
| Final artifacts | 19 indexed / 21 physical files; five durable 1280x720 stills at steps 0, 150, 300, 450, and 600 |
| Video | H.264, 1280x720, 10 fps, 61 decoded frames; 10 simulation seconds play in 6.1 s, a `1.639344262295082x` time-lapse |
| Timeline and provenance | Five-panel timeline, source/runtime metadata, frame/video validation, and retained pending state |
| Finalization | Pending hash DAG verified before final provenance/index/metadata writes; immutable pending metadata retained |
| Manual inspection | Complete and passing for the five durable stills, timeline, and separately decoded video midpoint |

The capture source, binary, numeric trajectory, and decoded media are
unchanged. The 70-file raw capture staging directory was pruned after sealing;
provenance binds its schedule and hashes, and clean-checkout verify-only needs
no ignored staging files. The bundle was freshly revalidated against the final
current trace and Native source after later additive scenario work.
Finalization regenerated the standing reference with the current executable
and again proved all 600 standing rows zero-difference. The original trace
hashes remain separate capture-provenance bindings.

Final visual identity:

- 12-function/16-case visual unit file SHA-256: `f06ba295006cf7e1f0fb692fc5eacf62e4e7f3be89bd166c041752d915779bdd`;
- driver SHA-256: `0f4e27b0c58e9dd3774c6be48ad4c70a857e2956fd81f11710837561f08f7243`;
- C++ capture source SHA-256: `c3efeac52d02a0c373f733598db81e545d062195ba6e96c2a65bcb607cd0207f`;
- capture binary SHA-256: `8b3cad15220c8fdb69c3ebdf7fa3923fda6fd812a49d0e54c8aeb07e62f0a7e9`;
- current trace source SHA-256: `b00eea0c87f75f17259fac433bafd98c20a961d14389a80e91742d3c8a678f76`;
- current trace binary SHA-256: `0923bf7df1eaa518f9a2ffacd0a42fd7330f5cbf6c35cb5749fbc122db1310ff`;
- original trace source SHA-256: `2e55298496d76a3a3fe002fcfaf5391332fdcf2da813b5df400affbea431e7cd`;
- original trace binary SHA-256: `0923bf7df1eaa518f9a2ffacd0a42fd7330f5cbf6c35cb5749fbc122db1310ff`;
- final metadata SHA-256: `b217b0a43200ca558f09776e464d50f0f9dc5c5a175bcfa66ae1b656076871d1`;
- retained pending-metadata SHA-256: `e300103dbb950b97e217ca0bea6f1c1dc78598ddf485cdaca26cc7ad58835b3c`;
- manual-inspection SHA-256: `4b52bcd26ed88d184bd695d77070420aeec2c95edfb203efda7ae6241150c343`;
- final provenance SHA-256: `eca349842e4121584145cd039a554aa13c51e5242ff924b37f5f49a9dee0ac2f`;
- final artifact-index SHA-256: `4d173c78e2c6584d6d8a85867160cc0c0687fe0f35e6878f66b0cfcd7c1c15ee`;
- current reference-trace SHA-256: `0e41af230fb4c27143966c3f8522702280e6746b31e84155a9a047326e46dbb3`;
- trace-equivalence SHA-256: `537f6e497b6fb6810240f006db47c91304581741370e319787635e6bcbfdd2e3`;
- frame-validation SHA-256: `a00dd7f4971756589ea834ac1a04f302ba8b7b7af96300d98e9cbc17413b3330`;
- video SHA-256: `e0c111f96511bbda41af145787bd9f68782b8c9e8a909d45eb03d3c3d647feb1`;
- timeline SHA-256: `926380c5c5768cc9ca1e414cbfe2631584daa99387090926d5a1f035d62b6ec9`; and
- decoded midpoint SHA-256: `75a88bb317441ed71803f784b2eaa099211c4e026f8547d6fe5f2ff3ee95909f`.

The only allowed visual claim is that the reconstructed float64 x86-64
literal 25-wedge arch remains standing and visually stable during this
declared 600-step no-projectile trajectory. The bundle is not evidence for a
projectile impact, the paper or author scene, a 101-stone arch, paper video/GUI
parity, or an author-golden frame comparison.

## Finalized Fig. 01/02 And Video.03 Incline Evidence

The durable current-source bundle is
[`assets/paper_evidence/fig01_02_incline_current_v1/`](assets/paper_evidence/fig01_02_incline_current_v1/).
Finalization and clean-checkout verify-only pass with status
`valid_current_source_nonpaper_incline`. The directory contains 23 physical
files; the fail-closed index binds 21 and declares only
`artifact-index.json` and `metadata.json` outside its indexed membership.

| Property | Finalized evidence |
| --- | --- |
| Capture media | Five durable 660x506 stills; 30 fps, 61-frame decoded H.264 schedule; 70 staging files pruned after sealing |
| Capture solver | 240 exact attempts/solves; zero caps, failures, or fallbacks; maximum residual `9.999836962261359e-7` |
| Independent traces | 121 rows each; 120 exact solves, 119 warm starts, zero fallbacks, and three contacts per post-initial step |
| `mu=.4` outcome | `1.7686892884927794 m` downhill versus analytical `1.7548661487418349 m`; final downhill speed `1.7544655347780056 m/s`; maximum residual `9.986952135669881e-7` |
| `mu=.5` outcome | `0.0008905412965980523 m` downhill; maximum/final stick speed `0.001116442058867632 m/s`; maximum residual `9.997210606407098e-7` |
| Outcome separation | `1.7677987471961814 m`; both traces preserve continuous post-initial tracked contact |
| Allowed equivalence | Aggregate `step`/exact-solve/fallback projection only; byte-identical SHA-256 `f03dff0aaec5f0fa6615609c2ea97aa31f072c14c6212fffe15858ea969d88c2` |
| Contact boundary | Capture reports 8 contacts; traces report 6 in aggregate; `contact_count_match=false`, `contact_counts_compared=false` |

The rendered combined scene and the two independent tracked traces use
different placements. State, residual, status, warm-start, per-cell, and
full-trace equivalence are all unproven and must not be inferred from the
passing aggregate count projection.

Core SHA-256 values are metadata
`7a5f973a9b7264911058ec91e253dfcf5d72a7ec46fa7020df0020af1a259b7d`,
artifact index
`b758bd28965bf9a96be7668c0dbb738b72c1493d83f195a3e726ae891f8f6e85`,
manual inspection
`3c3af65d62c629ae836302910a2fe7f928ab398628f280221f6d0b5d94d5a848`,
trace summary
`4df130e878f1e58d478870c8f132ee165061a752520e926f44c331d32f14f20d`,
verification
`2681073ee44f7fecd2782081826b414ee6541bb930149ced91f60a17dc2416d5`,
invocations
`b4e199a8e06ac6dd0c638b261a4d969fa973fb0bdbb376e0579f19fb2f6c56c3`,
run summary
`cfed42cb3b09ab8559d84a1dae4041a26d2e0937374787f34193c8363795b5d1`,
report `f75efcd40bd0452bcbdc7bbc82eee0fdbd78d7a1059f974e83999467b1688fa5`,
panel `f9f211fb376c97d98bccc67806ba3e1c9905d7d27764c794e1c480af7b4df9d3`,
clip `ad6d00ae6a614b0edbc836396c30621b589295a0eec826d060663f6782cee3f9`,
and traces
`449acf19feef2e0aa7fb04bb9f45f865727ba59f626b3964114cd900169ecd8a`
and
`2b30e8033b123876ad1cdea755741fd230a4d72d3747e55b907e6427962659c5`.

Current bound implementation identity is finalizer/test
`705da2a308697b4b4b923894d10f23622310e024aeab49862a24593c79142e23`/
`02f3800cf7cf5df5d85b1950512e78df6e413fea42cfa89880775f010d208e1c`,
runner/test
`d848afa53caf14b9fb3ea061d658eef274e8d917151937bd6340283b79ab5432`/
`6e378252fa6a7cb51c6813c9d5a2b30b8c8129eacdb01df0ee19a58a270cbc5e`,
demo source/binary
`84fd1330a13d548760e537f9734790ab1b5c91fcda3e446bf76b9b36f3e1aa99`/
`d838f23e9fb04224ff194658bd6e53d8cbb95ac94ce6676e54c49b8ea25916e4`,
trace source/binary
`b00eea0c87f75f17259fac433bafd98c20a961d14389a80e91742d3c8a678f76`/
`0923bf7df1eaa518f9a2ffacd0a42fd7330f5cbf6c35cb5749fbc122db1310ff`,
fixture source
`a4bed7372213d544fef62923b88bc9accee7086165d7b3cb20245bee8bade05f`,
and resolved libdart
`8fae2320858e49fdda309d89df8cb1158c1cc5dc11d345e14f5adca0ff63cf3d`.

Lane separation is mandatory. The finalized visual/trace lane is
`dart_best`; the separate current `paper_cpu`/Native matrix remains the strict
authority. Its `mu=.5` classifier passes with displacement
`8.63436433e-7 m`, but one accepted cap per repetition produces maximum
residual `1.4392081500753078e-6`, so strict solver/local-real-time remains
failed. Fig. 1, Fig. 2, and video.03 therefore remain `partial` pending a full
friction sweep/plot, matched external rows, approved source golden/diff,
paper contact-count match, the full 11 s semantic edit, paper timing, and
real-time parity.

## Pinned-Author Incline Sweep Scientific Negative

Preserve
[`assets/paper_evidence/author_incline_sweep_reference_v1/`](assets/paper_evidence/author_incline_sweep_reference_v1/).
This numeric packet pins author commit
`b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0` and preserves independent FBF,
MuJoCo, and Kamino CPU invocations. Each lane runs the exact seven-cell grid
`mu=.3,.4,.45,.5,.55,.6,.8`; every cell emits 120 steps, for 840 rows per
lane. The retained FBF histories record four contacts per FBF step; the
MuJoCo and Kamino result records contain no contact-count field.

| Property | Current-source observation |
| --- | --- |
| FBF configured convergence flags | 839/840 true; `mu=.55` is 119/120 |
| Sole false flag | `mu=.55`, step 1, at the 200-outer cap |
| True-flag mechanisms | 235 initial natural-residual shortcuts; 604 configured outer nonnegative `coulomb_rel < 1e-6` accepts |
| Natural residual among true flags | 456 at or below `1e-6`; 383 above `1e-6` |
| False-row residual split | Natural `final_residual=3.273267262002487e-8`; configured terminal `r_coulomb=1.5311460572898186e-6` |

The configured convergence flag and projected natural residual are distinct;
never substitute one for the other. The normalized displacements place FBF
and Kamino close and make the current MuJoCo curve nonmonotone, but that is not
full-state or cross-solver parity. The bundle contains numeric source-run
evidence only and does not increase the six-bundle visual inventory. It
establishes no DART/source trajectory match, historical invocation, approved
golden, media, timing, performance, or paper-parity result. Fig. 1, Fig. 2,
and video.03 remain partial.

## Finalized Painleve Proxy Visual Evidence

The durable current-source bundle is
[`assets/paper_evidence/fig05_painleve_proxy_current_v1/`](assets/paper_evidence/fig05_painleve_proxy_current_v1/).
It supersedes the Painleve cells in the older session-local `/tmp` matrix.
Finalization and clean-checkout verify-only both pass with status
`valid_current_source_nonpaper_proxy`, 27 indexed artifacts, and 29 physical
files; ignored capture staging is not required.

| Property | Finalized evidence |
| --- | --- |
| Artifact contract | 27 indexed artifacts / 29 physical files with exact indexed membership, sizes, and SHA-256 |
| Rendered members | Member clips fully decode to 76 frames; raw capture frames are pruned after sealing |
| Synchronized media | 1320x530, 30 fps, 76 frames |
| Tracked fixtures | 150 completed steps and 151 rows each; zero exact failures and boxed-LCP fallbacks |
| Maximum tracked residual | `9.998574150559113e-7` |
| `mu=.50` outcome | Upright settled proxy at `x=1.298081699724907 m`; final `up_z=0.999998178998452` |
| `mu=.55` classifier | First tumble at step 36, `t=0.6000000000000002 s`, `x=1.2597198197697048 m` |
| Shorter pre-tumble distance | `0.03836187995520213 m` before the `mu=.50` rest distance |
| Manual outcome | `mu=.50` returns upright; `mu=.55` tumbles and remains visually horizontal |

Verified current-source source/binary SHA-256 identity:

- finalizer: `31b2b560a3a6a7f06e514a8bc3dce9f4b766b3c4e62fe520435bfaa1e3ba77a9`;
- visual runner: `d848afa53caf14b9fb3ea061d658eef274e8d917151937bd6340283b79ab5432`;
- visual runner test:
  `6e378252fa6a7cb51c6813c9d5a2b30b8c8129eacdb01df0ee19a58a270cbc5e`;
- demo source/binary:
  `84fd1330a13d548760e537f9734790ab1b5c91fcda3e446bf76b9b36f3e1aa99` /
  `d838f23e9fb04224ff194658bd6e53d8cbb95ac94ce6676e54c49b8ea25916e4`;
- trace source/binary:
  `b00eea0c87f75f17259fac433bafd98c20a961d14389a80e91742d3c8a678f76` /
  `0923bf7df1eaa518f9a2ffacd0a42fd7330f5cbf6c35cb5749fbc122db1310ff`; and
- fixture source: `a4bed7372213d544fef62923b88bc9accee7086165d7b3cb20245bee8bade05f`.

Final bundle SHA-256 identity:

- metadata: `0845988dd05b18c965eba5fb5163d43dafc901ca8f30f66b01b4f37163d72f30`;
- artifact index: `7880c62d39c3f47109b50394cd84e20919565b70af19c1473c618841f682ff43`;
- manual inspection: `27c2632e427ec83b3612de6003cd16523fd483ab066d6618f9cdee38efdd2d0c`;
- trace summary: `115b50d92338477022435911df38d53a54e880c1daed4004dedd7d8507336164`;
- verification: `6fd34e54f7958c45777731ab888fe3a5e8e24d06b9fdddb96bdace269fcb515e`;
- paired panel: `bef97d04d59e2937195151cbf36a0b713bb2c2bef43d111513c05885595546e2`;
- paired clip: `dd4cdda2410b71f8e86035f1c0ff278f9dda77133c702e409ee5d533da443a4b`; and
- report: `a04c37f9f3db428994252c8486030aa6b97b367a627fc3a1a22106dfa8a9f51f`.

Manual visual classification and separate trace corroboration are distinct;
the rendered demos and tracked fixtures are not trace-equivalent. Angular
velocity is not exported, so strict rigid-body rest is unproven. This is not
author-scene or paper parity, faithful external-solver parity, an approved
source golden/diff, paper timing, or real-time evidence.

## Finalized Fig. 03 And Video.02 Backspin Evidence

The finalized current-source repository bundle is
[`assets/paper_evidence/fig03_backspin_current_v3/`](assets/paper_evidence/fig03_backspin_current_v3/).
Its index binds 18 artifacts in a 20-file physical directory. The MP4/GIF
preserve the full motion schedule; three durable stills retain steps 0, 1, and
2, and the 140-file raw capture staging directory is pruned after sealing.
Clean-checkout verify-only needs no ignored staging. Capture evidence has 129
exact attempts/solves, zero accepted caps, exact failures, or boxed-LCP
fallbacks, and maximum residual `9.96497154974839e-7`. The separate trace
maximum is `9.964971544991853e-7`. Both 131-row solver/contact projections
are byte-identical at SHA-256
`973d544311bac3b5927cc73b335b1a375d0339403a2f713707bb928076aa2b22`.

The trace reaches maximum `x=1.5959314363310166` at step 48, first records
negative `vx` at step 49, and ends at `x=-2.9362508912363654`,
`vx=-6.628158971623909`. Step 120 is the sole contact-free post-initial step.
The sphere uses a renderer-applied high-contrast 6x4 ivory/charcoal checker
texture with a coral registration tile. Its UV `MeshShape` is
`VisualAspect`-only; collision, inertia, friction, and dynamics remain bound to
the unchanged physical `SphereShape`. Manual inspection passes the checker
pattern and coral registration tile as legible. Finalized SHA-256 values are
metadata `a42ce0521a7c2af31662eff6a000ef5e68fe8bddf631d4f323be1ea8230c25a7`,
index `429a0888fa7a002cbc0c93e708569e4bf8e18195421c9663d5ce3b3a1968ab7f`,
manual inspection `b86c596da631503a3831fd55adeb934505cf758dfc8a612d0a0f32b2a55067cb`,
trace summary `0f6221fd32742b849e9ac79ec750de71b46ec24ba66b8d6e5a0e25048223ab48`,
verification `fb9de609e52df648465dc0dbe73af7fbbd1b98c1c4671cde504200ff72c15c01`,
trace `dc205297fa4cbffa1b497f12507b919f344bbee45a5820ae46145e0ed91bbd98`,
panel `72bf8d6098e8fb17b98bbedeaa00cc539866cf570d91d725f77dc75f1971b067`,
MP4 `7d4606f4da0a57ffbdfa0528906b21a20d7e1a4e47a6e7eb5387242aecc71928`,
GIF `773365f624ba1326855f2ad99c0196f761ab776001da568f7cdaa84054adacc8`,
and report `f9178c14c1930361afc1d97cb5bf08afe04d3fd451e8a94144b02bb0b46e61cf`.

The texture proves orientation legibility only. At `-200 rad/s`, 30/15 fps
sampling can alias and does not prove signed angular direction. Step 120 rules
out continuous contact; neither rest nor an airborne landing phase is
established. The separate rendered and CSV scene implementations are not
full-state trace-equivalent. External-solver, paper, approved-golden, timing,
and real-time parity remain unproven. Both `fig.03` and
`video.02_backspin` remain `partial`.

## Finalized Author-Pinned Fig. 04 Turntable Evidence

Use
[`assets/paper_evidence/fig04_turntable_author_current_v1/`](assets/paper_evidence/fig04_turntable_author_current_v1/).
It pins author commit `b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0`, binds 58
indexed artifacts in a 60-file physical directory, and preserves source order
`mu=.2, omega=2`,
`mu=.2, omega=5`, `mu=.5, omega=2`, `mu=.5, omega=5`.

The current visual lane is `dart_best` with Native `FourPointPlanar`. All four
cells complete 360 steps, pass the visual-lane solver contract, and record zero
fallbacks. The exact finite-horizon classification is ejected, ejected,
retained on support through 6 s, ejected. Do not call the retained cell
captured, stuck, zero-slip, co-rotating, or stable beyond 6 s. The segmented
disc and coral registration wedge are manual-inspection aids attached only to
the visual geometry; the physical cylinder remains the sole collision and
dynamics geometry.
Four durable, timeline-selected outcome stills bind steps 136, 120, 360, and
90 in source order. Capture staging is pruned after sealing; clean-checkout
verify-only relies on the durable clips, panels, timelines, stills, traces,
and provenance without ignored files.

Capture and trace projections are byte-identical for all four cells over the
six fields `step`, `contacts`, `exact_solves`, `warm_starts`,
`boxed_lcp_fallbacks`, and `status`. This is not full-state equivalence. The
separate strict `paper_cpu_native` lane has no capture comparison and cannot be
substituted across lanes: `mu=.5, omega=2` fails at step 40 with residual
`7.407835021099202e-6`; its other three rows pass, so the strict-lane aggregate
solver verdict is false.

Status is `valid_author_source_pinned_nonpaper_turntable_matrix`, with
`artifact_valid=true`, current visual-lane `solver_contract_valid=true`,
`physical_outcome_valid=true`, `manual_inspection.pass=true`, and `pass=true`.
SHA-256 values are report
`930cc12b95ab78c6e61d084064b584f5872633f2432e434c68d071818cec7fb1`,
artifact index
`209b677ce35e2b9c11248ab414727016394831084881a5e9c2866f68b51f6cdf`,
metadata
`854ef0c8aad75b4b200f512951d56b4dbef7aa82c46cc5e82123f0583dcc6ac5`,
verification
`455da7686eaeeb989e0d122c4724ea66ff161e8d652230eff9fb8ad20590bacd`,
group clip
`b241463658e6b48dfb2c74815e85317c2f0eccd46a7f7ab978b8f2701ce80d6d`.
Author-spec/visual-OBJ/visual-MTL/manual-inspection hashes are
`1680cd8351fa62937c0318826f7abc75917234cb3888f983acce06f13698bc6c`,
`bc86f1ef1f5fae1510f23b1586ae20efe788c499373370a66af81b06818f1b14`,
`619352b9ac14e89a4d467dde867019e0d01540b6f11852df565f23fb26a01752`,
and `095652d3df70a144be31be49b0e25b3265df54a3815df21742333d5fdfb4529a`.
Current visual runner/test bindings are
`d848afa53caf14b9fb3ea061d658eef274e8d917151937bd6340283b79ab5432`
and `6e378252fa6a7cb51c6813c9d5a2b30b8c8129eacdb01df0ee19a58a270cbc5e`.
No paper-comparability, approved-golden, paper-timing, or real-time claim
follows.

## Finalized Author Card-House Construction Evidence

Use
[`assets/paper_evidence/card_house_author_5_construction_current_v1/`](assets/paper_evidence/card_house_author_5_construction_current_v1/).
It has 12 indexed artifacts / 14 physical files and shows the public-author
default five-level, 40-card construction with four suspended cubes at step
zero. Index, metadata, and manual-inspection SHA-256 values are
`d6cbc6f9600b8bc5c3094dd85974eae8e71d64a9e3d6e99c1783ace36be9741d`,
`b97ac795c9368f2632fe422f975914f412e8d1cb0667023e8d83c6224547df00`,
and `7bc672e9dd95b52853c5c7e56680190d564fc9514add9b263de0c33c3f94e2a4`.

The boundary is construction-only: zero simulation substeps, with no release,
standing, trajectory, solver, contact-dynamics, physical-outcome, historical
four-level/26-card trajectory, Fig. 6/video, timing, performance, or parity
claim.

## Pinned-Author Masonry-Arch Scientific Negative

Preserve
[`assets/paper_evidence/author_masonry_arch_reference_v1/`](assets/paper_evidence/author_masonry_arch_reference_v1/).
The source-pinned invocation uses 500 frames, four substeps per frame, and
releases the three cubes at frame 400 / substep 1,600. The checked-in source
default is 400 frames with `drop_frame=400`, so it never releases the cubes;
this run is a new diagnostic rather than a historical or paper invocation.

A deterministic projection represents every one of the 2,000 substeps and is
lossless with respect to the declared claim fields. The 382,753,953-byte raw
source history (SHA-256 `cec0e4b86837e7542c498c7ddad40538983ec023332b88ebddee7766997e3ac1`)
is size/hash-bound but omitted. The projection records 157 true and 1,843 false
author convergence flags: 142/1,458 before release and 15/385 after release.
Of the true flags, 40 use the initial natural-residual shortcut and 117 use the
configured outer nonnegative `coulomb_rel < 1e-6` gate; all 40 shortcut accepts
occur before release. `final_residual` is the separate natural residual, with
only 47 values at or below `1e-6`. Release
substep 1,600 is nonconverged at 100 contacts and residual
`0.017456069692858667`; final substep 1,999 is nonconverged at 108 contacts and
residual `0.5161195175386001`. The first post-release contact-count increase
is inferred at substep 1,944, but no pair identities are available in the
projection.

Treat exit zero as artifact-preservation success only. The corresponding DART
specification is configuration-only and executes no dynamics; source
collision/contact-gap/backend/float32 semantics remain unimplemented. Do not
claim all-substep solver success, cube-arch pair contact, DART or cross-solver
dynamics/trajectory/outcome equivalence, Fig. 7/video.07 parity, timing,
repeatability, or visual/golden evidence.

The separately named current-source DART adapter executes the same raw numeric
configuration without changing the frozen source or crown-impact-v1 contracts.
The clean current-build exact lane clears the former step-68 local-QP failure
and completes 100/100 steps with 124/124 exact attempts, zero accepted
caps/failures/fallbacks, and worst residual `9.9936331058309156e-7`. A
200-step fail-fast run reaches a distinct outer iteration cap at step 142:
211/211 local solves succeed, but the 5,000-iteration outer solve ends at
residual `8.6992951837150444e-4` when contacts rise from 88 to 96. The current
change uses a boundary normal-cone stationarity certificate after the unchanged
ordinary KKT fast path. This remains a local diagnostic only; the 2,000-step
schedule, release/impact oracle, valid media, and Fig. 7 parity remain blocked.

## Preregistered Crown-Impact v1 Negative

Use [LITERAL_CROWN_IMPACT_V1.md](LITERAL_CROWN_IMPACT_V1.md) and
[`assets/paper_evidence/fig07_arch25_literal_impact_v1_negative_final_v9/`](assets/paper_evidence/fig07_arch25_literal_impact_v1_negative_final_v9/).
This separate 720-step reconstructed contract preserves the 600-step standing
prefix, injects the frozen three-cube cluster, and appends impact-only fields
without changing the 83- or 95-column standing schemas.

The standing prefix compares across 88 eligible fields with zero mismatches.
First arch contact is step 607 and first ground contact is step 616. The run
stays finite with zero exact failures/fallbacks, but it is a scientific
negative: five accepted caps, worst residual `9.154531704265396e-5`, final
arch displacement `0.07093964431215687 m > 0.07 m`, and far-field displacement
`0.060523747030465196 m > 0.007 m`. The evidence runner requires child exit 1
and the exact failed-gate pattern; it rejects a passing impact claim. No
parameter or threshold was tuned. The current runner independently recomputes finite gates,
requires post-launch exact progress, pins the normalized fingerprint and
frozen preregistration hash, and stores the 600-step by 88-field reference.

Final bundle hashes: runner
`622b388142fed881191dcb6efb103266eb2ea1e3ffbadaf402f63b89c558bb67`,
preregistration `4cdea674f366fc2d18eadf11ef4333d491786d5d85e3fc16fa611ea7dede3f37`,
fingerprint `86b37ec37d28259514453c9ecc9e7d5f12afe08118a0b04abd40e72acd147384`,
reference `22fabfe2e8f35bf329f32873845a60599ed2df171c15728f14311b91d888c387`,
raw `42cc94f5f111442da42b03af33722f713d66b853d3ac21f21083f28ba24e7ba4`,
stderr `7964b07fd5396f10013ff8d9100d2b36e7dfc151764210d8c60bd0ae853a2d94`,
summary `e3baa479111a999d03359a0642533cad1baa3a48c415609bafb9a7a7cfa18e7c`,
metadata `0bd19797953a10223bbd7183f6e8002e490b21c29b2304befee7e6618ff9ee73`,
report `c5711b0fe06e679f889f6a7ca3812aa955c3b8c61270ef038def06b2a3f173ff`.
The v6, v7, and v8 paths are historical current-at-capture evidence; v9
preserves the same frozen negative semantics and records the executed
`taskset` identity in its runtime closure.

## Literal 101-Stone v1 Scientific Negative

Use [LITERAL_ARCH_101_V1.md](LITERAL_ARCH_101_V1.md) and
[`assets/paper_evidence/fig08_arch101_literal_v1_negative_final_v7/`](assets/paper_evidence/fig08_arch101_literal_v1_negative_final_v7/).
The current Native exact-inertia 101-wedge reconstruction is no longer merely
a collision audit: it has a frozen, fail-closed dynamics result. Step 1 reaches
the 5,000-outer cap and returns `fbf_failed` at residual
`0.78153646143524735`, with one exact failure and zero fallbacks. The
`FourPointPlanar` dynamic aggregate fields are 400 contacts, 100 constraint
pairs, three colors, width 34, and four-P-core colored execution. A separate
repeat-2, collision-only Compact probe proves only the constructed time-zero
graph: 102 pairs equal 100 adjacent-stone pairs plus two springer-ground pairs.
The v7 one-step FourPointPlanar companion resolves the failed step-1 pre-solve
graph as exactly the 100-edge adjacent-stone chain: 100 unique adjacent pairs,
400 contacts, multiplicity four, zero non-adjacent pairs, and zero ground
pairs. Its aggregates and residual match the frozen trace. The companion
accepts the capped iterate and does not follow the frozen trace
participant-affinity contract, so solver-taxonomy and affinity equivalence
remain false. This narrow failed-prefix identity result cannot support source
equivalence, a valid trajectory, standing/physical outcome, timing, media,
long-run behavior, or paper parity. No parameter was tuned.

The current runner binds and rechecks source, executable, `ldd`, `taskset`, and
resolved regular shared-library file identities for the trace, collision
probe, and dynamics companion. That is
source/executable/shared-library-bound provenance, not complete host
runtime-state provenance. The prior
`assets/paper_evidence/fig08_arch101_literal_v1_negative_final/` bundle remains
only as invalid historical evidence because its shared-library and independent
graph provenance was incomplete.
The v2 bundle is provenance-complete historical evidence, but a
clang-format-only collision-probe source identity change superseded it for
current-source claims. Additive card-manifold trace instrumentation later
superseded v3, and v4 is historical current-at-capture evidence. The unchanged
explicit 101-stone command was rebaselined as v5 and again as v6 after the
current-build libdart identity advanced. V7 adds the identity-resolved one-step
dynamics companion; the frozen trace and scientific result remain unchanged.

Result hashes: runner
`7155b9bc6082e79aca317be6626fea587b58538df2f34e94f865cd54c15eb993`,
fingerprint `8d275edcaa82a2f628fdd5d9f846b2daed7e6864fdc4b7e80eca9a8c93bcf527`,
raw `fc0705ff497b44658dfe186ed27c50b2ab9d19a268f20cf50875a3155d1dc07d`,
summary `2cae961048b776c069caeccda2d95f2f0fd0969cae9e3de3782f0e5e5b7b640d`,
metadata `770d662f7021320982700fe699266bc94d2448679c8c4dc60bd127c660facb8a`,
report `1ba3987af896f68401faa1153f6b388aed38b087214d07592f5a060cab429e2a`,
and tree `e65107a6d2e2880742e02d01949bc4b3f9adb2f48b6be552a720fa83c9bcc4b3`.

## Card-House Native Manifold Sensitivity v2

Use [CARD_HOUSE_MANIFOLD_SENSITIVITY_V2.md](CARD_HOUSE_MANIFOLD_SENSITIVITY_V2.md)
and
[`assets/paper_evidence/card_house_26_manifold_sensitivity_v2_r3/`](assets/paper_evidence/card_house_26_manifold_sensitivity_v2_r3/).
This current-source, one-factor Native `Compact` versus `FourPointPlanar`
comparison is provenance-complete, but both trajectories are non-strict.
Compact emits 600 rows, records 3,495 accepted capped groups across 5,757 exact
attempts, and ends with last-group residual `8.525678738415048e-7`.
FourPointPlanar emits 600 rows, records 682 capped groups across 745 attempts,
and returns the frozen terminal-convergence-gate failure at residual
`0.016582575623909489`. Both have zero exact failures and fallbacks and zero
strict-success rows.

The preregistered direction is supported: FourPointPlanar increases mean
contacts by `93.7983333333` and mean per-pair multiplicity by
`1.9548548971`. This does not establish strict convergence improvement.
Physical and timing verdicts remain null, raw wall time is excluded, and no
real-time, author-scene, or paper-parity claim follows.
The unsuffixed v2 path is historical current-at-capture evidence; v2_r3 is the
current-source rebaseline with unchanged diagnostic semantics. Its
report/index/metadata/invocation/tree hashes are
`0429b12f90e4fef534c9b707324a180eaac78829a3b3667505111228aba58d2e`,
`1703f995bd5d2c6edeb12ba936010a80609c3e952f2c782b554bd0be7ca40627`,
`5890ab138179f4d7aaee6cd04c63799086439bae58fbde4f994048785dd0b8ac`,
`6e48eb835ab9aafca0b8d4bd29acb5ec932645fb37ca6124ce3c23b3265847f0`,
and `953d8efd4d43ee2a74cede6b5e3a0a766ecd1fd8914485b50b340f515b7ecb77`.

## Historical Negative Diagnostics

The following runs remain useful history but are superseded by mark26 as the
current local arch result.

### Scale-1 literal arch

| Configuration | Historical observation | Allowed interpretation |
| --- | --- | --- |
| 30,000-outer cold bootstrap, then standard 200 outers | Cold solve needed 28,981 outers; continuation passed steps 60/180, capped near 240, and collapsed by 300/600 | The old scale-1 profile was incomplete |
| 2,000 outers after bootstrap | Stable through step 300; capped near 420/480; too slow | More budget delayed but did not close that profile |
| Local-diagonal first-state seed | Cold count fell to 22,726, 21.6 percent lower | Diagnostic only |
| Dense-global first-state seed | No material benefit | Dense response was not the missing cure |
| Velocity ERP `0.01` or `0.1` | Worse convergence or failure | ERP tuning did not establish parity |

Do not present scale 35, relaxation 1.1, the reconstructed closure, or any of
the old bootstrap/seed/ERP experiments as author parameters. Mark26 is an
explicitly non-paper DART configuration.

### Strict paper-parameter card bootstrap

| Step | Contacts | Outer iterations | Step time | Residual | Warm start |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | 119 | 30,000 | 15.167 s | `0.004311` | cold |
| 2 | 119 | 200 | 0.136 s | `0.026233` | 87/119 |

Both steps reported zero exact failures and zero boxed-LCP fallbacks, but both
failed `1e-6`, and the trajectory was incomplete. Preserve this as a negative
historical diagnostic, not performance evidence.

## Evidence And Manifest State

The current manifest audit is:

```text
29 requirements = 24 partial + 5 blocked + 0 complete
```

The manifest validator currently passes and fail-closed hashes repository
artifacts, materializes the current bundle indexes, binds recorded process,
taskset, topology, residency, and archived-prior-source provenance, recomputes
CPU claims from raw rows, and enforces every current-truth promotion boundary.
Fig. 1, Fig. 2, and video.03 bind the finalized lane-separated incline
threshold evidence plus the pinned-author seven-cell numeric sweep while
retaining their partial status; Fig. 5 and video.05
bind the finalized Painleve DART-proxy traces and media;
Fig. 7 and video.07 bind the validated standing
trace/still/video/outcome/claim-map artifacts, while impact-v1 and arch101-v1
are recorded as scientific negatives and card-manifold v2 as non-strict
diagnostic comparison evidence. Do not mechanically promote
requirements whose source-equivalence, golden, external-baseline, or impact gates
remain missing.

The P-core mark26 bundle is the authoritative durable performance/scaling
evidence for the reconstructed literal arch. The separate current-source small
`paper_cpu`/Native matrix at
`assets/dart_cpu_evidence/2026-07-19_current_source_paper_cpu_small_r7/` is also
repository-finalized: it binds 60 artifacts, 27 complete CPU-4 invocations, and
5,220 rows; 9/9 physical classifiers and 7/9 strict-solver/local-real-time
contracts pass. It used zero warmups and is not paper-comparable. Older small
CPU bundles and the high-load E-core diagnostic remain historical. The literal
fig07 bundle is source-bound, trace-equivalent, validated, and manually
inspected within its narrow stable/standing claim.
The r7 report/index/metadata/summary/raw SHA-256 values are respectively
`008bc94667893cd26bfc04a720caca3d3a5703601c8739bc06856f93818c63d5`,
`06594c1e1cc3c6858bc78a80630aefb0e85eeef92204b0a09b99425411c3ebeb`,
`e227d7aef3b273ff81385f227f9e6766a7e40a622bc4298e1807b7c2068bc417`,
`9137f1b2db0909a96897632a66617f2cfa58476a369d2adfe232b73e46cd0fa2`,
and `ba062cf359da85d21b5ea83b722d26375267212dfc44a4ce9f48701ad8a79a5a`.

## Current Recorded Verification

These are the latest recorded focused results on the current source, not yet a
final integrated closeout run:

| Gate | Result |
| --- | --- |
| Exact-Coulomb math, Release | 47/47 |
| Exact constraint solver | 25/25 |
| `ConstraintSolver` integration | 64/64 |
| Native collision | 42/42 |
| Masonry wedge dynamics | 3/3 |
| Default paper fixtures | 19 passed; 3 explicit stress cases skipped |
| Focused Release/Debug CTest matrix | 9/9 in each configuration |
| Schema-v8 CPU evidence unit tests | 230/230 |
| Literal-wedge visual finalization unit tests | 16/16 |
| Crown-impact trace and negative-runner unit tests | 25/25 |
| Literal 101-stone trace/probe/runner unit tests | 41/41 |
| Finalized incline unit/verify-only gates | 62/62; clean-checkout verify-only passes with 21 indexed artifacts and no ignored staging dependency |
| Author-incline reference finalizer unit tests | 64/64; verify-only reports 37 indexed artifacts / 39 physical files |
| Focused manifest/backspin/incline/author-masonry/author-incline evidence suite | 859 passed post-merge in 163.95 s |
| Full no-cache dartpy Python suite | 1,555 passed in 165.09 s |
| Author masonry-arch focused CTest | 1/1 passed |
| Manifest host-identity modes | Sealed producer closure: 118 live file rechecks, 0 skipped; explicit archive: 0 live, 118 skipped; both validate 29 requirements with status `partial` |
| Deterministic colored-scheduler stress | 1,000 runs passed |

The sealed live closure resolved `libdart.so.6.19` to the recorded
`libdart.so.6.19.3`. The normal development symlink is restored to
`libdart.so.6.19.4`; the files are byte-identical, but the resolved path is
identity, so the current default live run reports five path mismatches. Archive
mode remains clean; recreate the historical symlink only for an explicit live
closure check.

## Active Completion Gates

| Gate | Current state | Required next evidence |
| --- | --- | --- |
| Literal 25-stone static arch | Exact trajectory, residual, outcome, mean-real-time, multicore, trace-equivalent visual, and manual-inspection gates pass locally | Preserve the authoritative P-core bundle and narrow reconstructed stable/standing claim |
| Projectile impact sequences | Frozen v1 is a durable scientific negative: contact order/finite/fallback gates pass, but cap/residual/global/far-field gates fail | Preserve v1 without tuning; obtain source-equivalent passing evidence and inspected impact media only from a separately declared contract |
| Card house | Current-source manifold sensitivity comparison is complete, but both modes are non-strict and no physical/media verdict is promotable | Strict full paper-profile trajectory, physical outcome, and current-build source-matched media |
| 101-stone arch and 10-level card house | Identity-resolved 101-stone v7 step-1 failure and partial reconstructions | Full exact trajectories, physical outcomes, and current-build media or precise blockers |
| Small figures and video | Incline `fig01_02_incline_current_v1`, Painleve `current_v1`, backspin `fig03_backspin_current_v3`, and author-pinned turntable `fig04_turntable_author_current_v1` are repository-finalized within narrow DART-only boundaries; the separate author incline sweep is numeric source-run evidence | Preserve and reverify all four visual bundles plus the numeric sweep; retain the incline 8-versus-6 contact mismatch, strict-lane failure, and missing DART/external/history/golden/media comparisons, then finalize only the remaining affected rows before promotion |
| Paper performance | Current source is available, but matched DART runs are pending and historical comparability remains partly external | Audit and run the pinned author workloads; keep the exact historical Apple host, renderer, and timing-attestation gaps explicit |
| Manifest | Validator passes with 24 partial, 5 blocked, and 0 complete | Keep every row bound to current artifacts while retaining missing source-equivalence and historical-parity blockers |
| CI/PR | Local dirty state and old PR head/body | Approved base merge, coherent commits/push, truthful body, and current-head CI |
| Final review | Not current | Two clean independent reviews of the final post-fix state |

## Immediate Work Order

1. Inspect current diffs and active agents before taking file or build-tree
   ownership.
2. Preserve the P-core, standing-visual, finalized current-source Painleve-proxy,
   finalized incline and backspin, pinned-author numeric incline sweep,
   frozen impact-v1/arch101-v1 negatives, and card-manifold-v2 diagnostic while
   generating the remaining strict card, 101-stone, figure, and
   source-equivalent impact artifacts. Decode and manually inspect media; do
   not tune frozen protocols or infer outcomes from nonblank frames.
3. Keep all 29 manifest rows and their sidecars, hashes, semantic verdicts,
   and report entries synchronized from one identified build.
4. Synchronize the parity matrix, GUI report, residual report, and PR-facing
   report with the same evidence set.
5. Preserve the 638-passed integrated evidence suite; rerun the final full
   integrated test battery and obtain two clean independent reviews.
6. With explicit approval only, merge the latest target base, create coherent
   commits, push, replace the #3377 body, and obtain current-head CI.

## Reproduction Commands

Build only after checking that no other agent owns the shared Ninja tree:

```bash
cmake --build build/default/cpp/Release \
  --target fbf_paper_trace fbf_paper_arch_wedge_dynamics_probe \
  test_ExactCoulombFbfPaperFixtures test_ConstraintSolver \
  test_NativeCollisionDetector \
  --parallel 4
```

Focused Python gate:

```bash
.pixi/envs/default/bin/python -m pytest -q \
  python/tests/unit/test_run_fbf_cpu_evidence.py
```

Mark26 reproduction shape; use a fresh output directory:

```bash
.pixi/envs/default/bin/python scripts/run_fbf_cpu_evidence.py \
  --binary \
    build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace \
  --output-dir /tmp/fbf-mark26-native25-colored-v9-archwide-pcore-rerun \
  --case masonry_arch_25_literal_wedge:600 \
  --threads 1,4 \
  --cpu-list-for 1:8 \
  --cpu-list-for 4:8,10,12,14 \
  --repetitions 3 \
  --warmup-repetitions 1 \
  --solver exact_fbf \
  --contract dart_best_colored_bgs \
    --collision-frontend native
```

Verify the finalized incline bundle without rewriting it:

```bash
.pixi/envs/default/bin/python scripts/finalize_fbf_incline_visual.py \
  --bundle \
    docs/dev_tasks/fbf_exact_coulomb_friction/assets/paper_evidence/fig01_02_incline_current_v1 \
  --verify-only
```

Verify the pinned-author numeric incline sweep without rewriting it:

```bash
python3 scripts/finalize_fbf_author_incline_reference.py --verify-only
```

## Guardrails

- Do not convert `max_iterations_accepted` or a finite local solve into global
  convergence.
- Do not advance evidence after an unresolved exact group when fallback is
  disabled.
- Do not infer physical correctness from a decoded or nonblank image.
- Do not infer paper parity from local physical correctness or speedup.
- Do not call mean-real-time throughput an every-step deadline guarantee.
- Do not use stale small artifacts or uncommitted diagnostics as evidence.
- Do not equate the incline capture's eight contacts with the independent
  traces' six aggregate contacts or broaden the three-field count projection.
- Preserve boxed LCP as DART's default and keep applicable regression
  coverage.
- Keep paper timing null until the source workload and comparison contract can
  actually be matched.
