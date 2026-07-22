# Agent Continuation And Truth Ledger

This is the authoritative status for the active
`fbf_exact_coulomb_friction` dev task as of 2026-07-22. It supersedes older
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
`b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0`. DART now has separately named
strict and source-continuation four-level card-house adapters. The continuation
lane completes and has local exact/boxed media. A source-pinned 101-stone DART
adapter now also executes the 400-frame source-supported standing schedule, but
its strict exact lane stops at step 209 and its complete boxed lane collapses.
A separate ten-level continuation lane now has a final resealed and
independently reverified exact member; the full boxed member and browser upload
remain pending, and it has no automated physical-outcome oracle. DART therefore
does not reproduce the source
trajectory equivalently, all scenes source-equivalently, the historical
renderer, Apple hardware, or paper timer boundary.

## Truth Ledger

| Claim | Current evidence | Verdict |
| --- | --- | --- |
| Public author reference | The MIT-licensed repository is available at pinned commit `b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0` with the solver, six runnable examples, current configurations, pinned dependencies, and optional MuJoCo/Kamino runners | Available and source-auditable; porting and matched runs are now internal work |
| PR #3374 | Merged at `fa17fad` | Required visual infrastructure is available on the release branch |
| PR #3377 | Verified open and draft at implementation/media head `c95bd5fb916` on 2026-07-22; topic head, target ancestry, merge state, checks, and reviews remain mutable and must be queried live | WIP, not completion evidence; later documentation-only checkpoints do not change the c95 binary/media identity |
| Exact algebra | Row signs validated; row-operator versus impulse-test `W` relative error `1.33e-16`; spectral-nullspace regressions pass | Tested formulation is internally consistent |
| Literal 25-stone exact dynamics | Schema-v8 mark26 has eight complete 600-step processes: one warmup plus three measured runs at each of one and four threads | Full local exact trajectory is proven for the reconstructed non-paper scene |
| Exact solve contract | 1,800 measured steps per thread count; maximum residual `9.999807145410957e-7`; zero exact failures, caps, or fallbacks | Pass |
| Physical outcome | 96 contacts and 24 colliding pairs throughout; physical outcome valid and identical at one and four threads | Pass for the declared reconstructed scene |
| Colored inner BGS | 24 manifolds, 3 colors, width 8; four-thread runs observe the pinned four-CPU colored phase and pass the scaling-pair contract | Real-scene multicore execution and matched-work speedup proven locally |
| Local mean real time | 1-thread mean `6.122883343333333 ms`; 4-thread mean `4.26939745 ms`; validated speedup `1.4341328993236115x` | Both means pass 60 Hz |
| Every-step deadline | Maxima are `287.473818 ms` and `180.504588 ms` | Neither thread count passes an all-step 60 Hz deadline |
| Paper timing | The current author workload and kernel are inspectable, but DART's engine, precision, host, and timer boundary are unmatched and the historical Apple/timing attestation is absent | Paper timing target and verdict remain null |
| Current small CPU matrix | Locally finalized current-source `paper_cpu`/Native bundle binds 60 artifacts, 27 complete CPU-4 invocations, and 5,220 rows | 9/9 physical classifiers pass; 7/9 strict-solver/local-real-time contracts pass; zero warmups and unmatched paper contracts prohibit a paper timing claim |
| Literal 25-stone visual evidence | The c95-bound capture and independent reuse verification pass for exact, boxed, and their synchronized group. Exact completes 600/600 with 96 contacts and zero caps/failures/fallbacks; boxed completes 600/600 | Capture/integrity and exact-telemetry pass for the reconstructed no-projectile scene; both visibly standing is manual-only because no automated semantic/physical oracle exists |
| Figure 7 crown-impact source continuation | Checkpoint `34d9b66e97c` adds a separate bounded continuation lane. Exact and boxed complete 2,000/2,000 with cube release at step 1,600; exact has 2,122/2,122 solves, zero failures/fallbacks, 1,940 plateau accepts, and 98 max-iteration accepts | Capture/integrity and continuation telemetry pass only. Manual outcomes are nearly identical; no strict convergence, superiority, physical outcome, source/paper parity, timing, or backend claim follows, and strict remains blocked at step 142 |
| C95-bound Figures 1-5 reseal | Ignored `assets/pr_media_current_head_c95_small_rows/` binds 20 members and 13 groups to implementation/media head `c95bd5fb916`. Capture and independent reuse verification pass with zero failures and five expected exact-only author-turntable boxed skips; summary hashes are `8f227ab...` and `264ac6e...` | Authoritative local browser-upload source only. All paper/automated-semantic flags remain false; manual DART observations do not establish paper parity or general solver superiority |
| Browser-upload handoff | `PAPER_DEMO_VIDEO_MATRIX.md` consolidates nine minimum source-row clips plus five supplemental direct comparisons; an independent audit verifies all 14 local paths/hashes, H.264/yuv420p at 30 fps, and complete `ffmpeg -xerror` decodes | Locally upload-ready only. No GitHub user-attachment URL exists, and every exact-only, proxy, non-strict, frozen-prefix, and missing-boxed caption boundary remains mandatory |
| Incline visual evidence | Locally finalized `fig01_02_incline_current_v1` has 21 indexed artifacts / 23 physical files, five selected local stills, a 61-frame decoded clip schedule, two independent 121-row traces, manual inspection, and byte-identical aggregate exact-solve/fallback projections | Valid current-source non-paper threshold evidence; capture contacts 8 versus aggregate trace contacts 6 are explicitly not compared, and Fig. 1/2 plus video.03 remain partial |
| Pinned-author incline sweep | `author_incline_sweep_reference_v1` preserves separate current-source FBF, MuJoCo, and Kamino CPU runs on `mu=.3,.4,.45,.5,.55,.6,.8`; every lane has seven 120-step cells, and the retained FBF histories record four contacts per FBF step | Numeric source-pinned scientific-negative/reference evidence only; FBF records 839/840 configured convergence flags, timing is excluded, and no DART/full-state/historical/golden/media/timing/performance/parity claim follows |
| Source-pinned Painleve adapter | The ignored `fig05_painleve_author_current_v1` bundle passes capture and independent verify with four 121-sample exact/boxed traces, four 61-frame H.264/yuv420p members, four decoded groups, machine-classified outcomes, and manually audited panels/keyframes | Under the pinned current DART adapter, exact and boxed diverge at `mu=.55`; GitHub attachment URLs remain pending, and no source-backend, trajectory, paper-Figure-5, timing, or solver-superiority claim follows |
| Historical Painleve proxy visual evidence | Locally finalized `current_v1` bundle has 27 indexed artifacts / 29 physical files, two 151-row traces, fully decoded paired media, and bound manual inspection | Valid historical DART-side nonpaper proxy evidence only; it does not satisfy the source-pinned Fig. 5 row, rendered demos and tracked fixtures are not trace-equivalent, and paper/external/golden/timing/real-time/strict-rest claims remain unproven |
| Backspin visual evidence | Locally finalized `fig03_backspin_current_v3` has 18 indexed artifacts / 20 physical files, three selected local stills, MP4/GIF media, 129 exact attempts/solves, zero caps/failures/fallbacks, a corroborating translational trace, and a passing manual inspection of the renderer-applied high-contrast 6x4 ivory/charcoal checker texture and coral registration tile | Valid current-source DART evidence only; `fig.03` and `video.02_backspin` remain partial |
| Author-pinned turntable visual evidence | Locally finalized `fig04_turntable_author_current_v1` has 58 indexed artifacts / 60 physical files, four timeline-bound outcome stills, and four complete 360-step author-configured visual-lane cells: three eject and `mu=.5, omega=2` remains on support through 6 s | Valid finite-horizon author-source-pinned non-paper DART evidence; no zero-slip, co-rotation, full-state, paper-golden, timing, real-time, or parity claim |
| Author card-house construction | Locally finalized `card_house_author_5_construction_current_v1` has 12 indexed artifacts / 14 physical files and shows the public-author default five-level, 40-card configuration at step zero | Construction-only evidence; zero simulation substeps and no release, standing, dynamics, solver, physical-outcome, Fig. 6/video, timing, performance, or parity claim |
| Current-source four-level card-house adapter | Strict and source-continuation adapters bind the same supported four-level, 600-frame selection to 26 source-sized cards, four initially kinematic cubes, and a 2,400-substep exact/boxed DART schedule | Strict source-inner replays still fail the 56-contact group at step 35. Colored ordering and global scope are bounded rejects. A source-sized-gap candidate fails the 36-step gate at step 31, while c95 cadence-5, same-binary `last_norm10`, and same-binary source-seed-values candidates still fail at step 35; all six bounded strict-prefix diagnostics retain explicit claim limits and leave the scene unchanged. The separate continuation capture completes both lanes through release; exact has 3,351/3,351 solves, zero failures/fallbacks, 113 plateau and 633 max-iteration accepts, zero shrink caps, and worst residual `0.917120`. Manual inspection finds more endpoint structure in exact, but this is not strict convergence, quantitative physical/trajectory parity, golden/backend/timing parity, superiority, or Fig. 6 paper parity |
| Current-source ten-level card-house adapter | Separate strict and source-continuation scene/schedule pairs bind `--levels 10` to 155 cards, four cubes, 3,200 DART substeps, release after step 1,600, and the pinned heterogeneous gaps. Predictive checkpoint `3647959a188` matches only the scalar source `separation / dt` allowance | Strict reaches completed step 31 before a 79-contact failure. The final exact continuation reseal and independent reuse verification pass: 3,200/3,200, 7,702/7,702 solves, zero failures/fallbacks, 2,427 plateau and 763 max-iteration accepts, and no automated semantic-outcome validation. A clean boxed control completes 80/80, but the full run reached only step 112, was interrupted, and is non-evidence. Paired media remains blocked. No source, trajectory, physical, Tables 6-7, superiority, or paper parity is valid; see `CARD_HOUSE_10_CURRENT_SOURCE_DIAGNOSIS.md` |
| Pinned-author masonry arch | `author_masonry_arch_reference_v1` records the 500-frame / 2,000-substep author run with release at substep 1,600; a deterministic projection represents every substep, with only 157 author convergence flags true and 1,843 false | Valid source-pinned scientific negative; not the 400-frame source default or a paper invocation, and not DART/cross-solver dynamics, trajectory, outcome, timing, repeatability, pair-contact, or media parity |
| Source-pinned 101-stone DART adapter | `fbf_author_masonry_arch_101_standing_current_source` binds all 101 author meshes and the source-supported `--stones 101`, 400-frame / 1,600-substep no-release schedule. The strict exact lane fails closed at completed step 209 on an iteration cap; boxed completes but fails the local standing oracle and visibly collapses. Full current-source FBF and Kamino controls also fail the same local standing criterion | Precise current-DART/current-source scientific negatives. They are not a historical invocation/backend, converged golden, full trajectory/outcome, Fig. 8/video.08, timing, performance, or solver-superiority evidence |
| Reconstructed crown impact | Frozen three-cube v1 reaches the arch before the ground and stays finite, but fails cap, residual, whole-arch, and far-field gates | Locally sealed scientific negative; `impact_claim_passed=false`, no tuning |
| Card-house manifold sensitivity | Current-source v2 emits 600 rows for Compact and FourPointPlanar; both are non-strict, while FourPointPlanar raises mean contacts by `93.7983` and mean multiplicity by `1.95485` | Valid one-factor diagnostic only; physical, timing, real-time, and paper verdicts remain null |
| Paper-media parity | The passing literal bundle contains no projectile. The source-pinned 101-stone lane now has a decoded boxed-collapse clip and a frozen-prefix diagnostic comparison, but no complete exact clip, current-source Kamino media/full-pose trace, or historical camera/material/golden render bundle | No paper impact, Fig. 8/video.08, GUI, or golden-frame parity claim |
| Manifest | Current audit: 29 rows, 24 partial, 5 blocked, 0 complete; six local visual bundles are finalized and the visual workflow declares 28 schedules | Validator passes; overall status honestly remains partial |

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
- The target was an ancestor of the topic when last checked on 2026-07-22.
  Fetch origin and verify target ancestry before every publication; do not
  hard-code the topic head or divergence in owner docs.
- Query PR #3377 live for its head, merge state, checks, and review state.
- Keep the live #3377 body synchronized from [PR_REPORT.md](PR_REPORT.md) and
  retain draft state until every documented completion gate is met.
- Do not commit, push, edit the PR, rerun CI, post comments, or otherwise
  mutate GitHub without explicit maintainer/user approval.
- Before any approved follow-up push, fetch the latest real target branch and
  merge it if it is not already an ancestor; do not rebase unless explicitly
  requested.

## Locally Validated Mark26 CPU Evidence

The authoritative bundle is
`assets/dart_cpu_evidence/2026-07-19_mark26_native25_colored_v10_archwide_pcore/`.
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

The ignored c95-bound recapture root is
`assets/pr_media_current_head_fig07/`. The capture and independent reuse
verification summaries pass for two members plus one group with no failures or
skips. Their external summary SHA-256 values are
`5a1de1f915d75c373f06aeb48978b92a540bc245427c55584f68ad178ea491bb` and
`e4b3d44d5f2afebef9f79bcb92b38ee282f5635c38fa4be2f4264a5f961acce5`.

| C95-bound artifact | Result | SHA-256 identity |
| --- | --- | --- |
| Exact member | 600/600 exact solves; 96 contacts; zero accepted caps/failures/fallbacks; final/worst residual `9.778093504499096e-7` / `9.999807145410957e-7`; 599 warm starts; 7,933 iterations | timeline `6041addd27a79a747cdbcdaafb495f787d4e90a906ec0616aa16e5a33d9c9b74`; clip `24c110421572500bec9f43a431061ae5a386e7e59940e86030e3059cc90d9676`; panel `cd3498c90fd549365dadc9cf96908c4c3d86da81cbd5dd64ff4d891407b4ee6b`; metadata `614704cc1ed70065d81b789c019e618ac54d7013f706b26a346ea236ef876802` |
| Boxed member | 600/600 with `BoxedLcpConstraintSolver` | timeline `809ca91a475fdd0ebe3ad6b5cba73115c9ec2b3dd4a98e478beca229abf62321`; clip `80e79fa6b356f951e9615dd94aad2de2f55d0bbab07d7b116cb07c1b3bef686c`; panel `078990e4d7a950ba9e207102624d651d948f546f1153bf85077e8084c01b040a`; metadata `944a6636bd6febb79ab9d6abda0a92165acd34ab3aa96b42f55f1c36731e6d45` |
| Labeled exact-versus-boxed group | 1320x530 H.264/yuv420p; 301 frames; 30 fps; 10.033333 s; full decode passes | clip `89c4d7372f68c6c9ad1a5d0e0e0388ffa1f198c2446e04fe30b9bc66325d8f9e`; panel `5ce6efcebcb5f6a2385f6aea6de7933cccfa7446979b7ea0e46ef2aab5199633`; metadata `5cc2513eb16db191454b27126783df70adbe3ad2617d3faa3f51597ab43966bb` |

Manual endpoint/group inspection finds both arches visibly standing and the
labels/framing clear. This is qualitative only: every metadata record has
`automated_semantic_outcome_validated=false`, and the group is presentation,
not a physical oracle or superiority test. Claim no source/paper trajectory,
outcome, timing, solver superiority, crown-impact, or Fig. 7 parity. The
separate source-configuration crown-impact adapter remains blocked at strict
step 142. GitHub browser-composer upload remains pending.

The separate Figure 7 crown-impact continuation pair is ignored at
`assets/pr_media_current_head_fig07_crown_continuation/`. Its paired run and
independent two-result/one-group reuse verification pass with SHA-256 values
`f0e45526d648d7c8d6052c3a4f32ec47a29033e4ed687b89fecc52c1ce04396f`
and `969ef6143185716e8441704829d4643a1d804fc5580288dae28fe47257fce0f3`.
Both lanes complete 2,000/2,000 and release the cubes at step 1,600. Exact has
2,122/2,122 attempts/solves, zero failures/fallbacks, 1,940 plateau accepts,
98 max-iteration accepts, and final residual `0.004493046465992133`.
Group metadata/panel/clip hashes are
`4229307f7d6d91f4b347fecc53db9f290061c6dc76482e684a94064f764601d7`,
`f3bdb5a20ad57ee20e1a2cf6508a701f9bbd79bf0532ffc303d87989b4dfa802`,
and `c4ffe2488520a5c22608c9117443cf9ff5de5396f4353d4bced5d1afff6bf0c8`.

Manual inspection finds both arches standing and cubes reaching the crown,
with nearly identical visible outcomes. Metadata sets
`paper_comparable=false` and
`automated_semantic_outcome_validated=false`. Keep the claim bounded to
non-strict continuation capture/integrity and telemetry: it proves no strict
convergence, superiority, physical outcome, source/paper trajectory or Figure
7 parity, timing, or backend equivalence. It does not clear the strict
step-142 blocker. Upload the group clip through the GitHub browser composer
and record its URL.

The earlier trace-equivalent current-source visual bundle is
`assets/paper_evidence/fig07_arch25_literal/`.
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
| Final artifacts | 19 indexed / 21 physical files; five selected local 1280x720 stills at steps 0, 150, 300, 450, and 600 |
| Video | H.264, 1280x720, 10 fps, 61 decoded frames; 10 simulation seconds play in 6.1 s, a `1.639344262295082x` time-lapse |
| Timeline and provenance | Five-panel timeline, source/runtime metadata, frame/video validation, and retained pending state |
| Finalization | Pending hash DAG verified before final provenance/index/metadata writes; immutable pending metadata retained |
| Manual inspection | Complete and passing for the five selected local stills, timeline, and separately decoded video midpoint |

The capture source, binary, numeric trajectory, and decoded media are
unchanged. The 70-file raw capture staging directory was pruned after sealing;
provenance binds its schedule and hashes. With the compact local bundle present,
verify-only does not require the pruned raw staging files. The bundle was
freshly revalidated against the final
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

The locally finalized current-source bundle is
`assets/paper_evidence/fig01_02_incline_current_v1/`.
With the compact local bundle present, finalization and verify-only pass with status
`valid_current_source_nonpaper_incline`. The directory contains 23 physical
files; the fail-closed index binds 21 and declares only
`artifact-index.json` and `metadata.json` outside its indexed membership.

| Property | Finalized evidence |
| --- | --- |
| Capture media | Five selected local 660x506 stills; 30 fps, 61-frame decoded H.264 schedule; 70 staging files pruned after sealing |
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
`assets/paper_evidence/author_incline_sweep_reference_v1/`.
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

## Source-Pinned Figure 5 Painleve Adapter

`painleve_author_mu05` / `fbf_author_painleve_mu_0_5` and
`painleve_author_mu055` / `fbf_author_painleve_mu_0_55` bind the public author
configuration at commit `b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0` to
paired exact/boxed DART capture schedules. The pinned configuration uses the
`0.3 x 1.2 x 0.6 m`, `43.2 kg` box, `(4,0,0) m/s` initial velocity,
`dt=1/60 s`, 120 steps, and selected `mu=.5,.55` sweep. See
[PAPER_DEMO_VIDEO_MATRIX.md](PAPER_DEMO_VIDEO_MATRIX.md) for the complete
source identity, adapter boundary, and acceptance gates.

The ignored durable bundle is
`assets/paper_evidence/fig05_painleve_author_current_v1/`. Capture summary and
independent verify both pass with four member results and four group results.
Every member and composite clip is a fully decoded 61-frame H.264/yuv420p MP4,
and the panels and selected keyframes were manually audited.

| Cell/lane | Current-DART-adapter outcome | Horizontal travel |
| --- | --- | ---: |
| `mu=.5`, exact | `upright_near_rest` | `1.5986787381 m` |
| `mu=.5`, boxed | `upright_near_rest` | `1.5977005918 m` |
| `mu=.55`, exact | `tumbled_near_rest` | `1.5399225956 m` |
| `mu=.55`, boxed | `upright_near_rest` | `1.6623056217 m` |

The exact `mu=.5` member records 119 attempts/solves, zero failures/fallbacks,
final residual `5.2255077e-7`, and worst residual `9.7391465e-7`; exact
`mu=.55` records 108 attempts/solves, zero failures/fallbacks, final residual
`9.1964345e-7`, and worst residual `9.9977460e-7`. The adapter identity also
binds the exact-options header hash, so solver-default drift fails closed.

The defensible result is exactly: under the pinned current DART adapter, exact
and boxed lanes diverge at `mu=.55`. Source-backend equivalence, trajectory
equivalence, paper Figure 5 parity, timing comparability, and solver
superiority remain false. GitHub user-attachment URLs remain pending manual
browser-composer upload; the local capture must not be checked into Git.

## Historical Finalized Painleve Proxy Visual Evidence

The locally finalized current-source bundle is
`assets/paper_evidence/fig05_painleve_proxy_current_v1/`.
It supersedes the Painleve cells in the older session-local `/tmp` matrix.
With the compact local bundle present, finalization and verify-only both pass with status
`valid_current_source_nonpaper_proxy`, 27 indexed artifacts, and 29 physical
files; pruned raw capture staging is not required.

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

The locally finalized current-source bundle is
`assets/paper_evidence/fig03_backspin_current_v3/`.
Its index binds 18 artifacts in a 20-file physical directory. The MP4/GIF
preserve the full motion schedule; three selected local stills retain steps 0,
1, and 2, and the 140-file raw capture staging directory is pruned after local
sealing. With the compact local bundle present, verify-only does not require
that raw staging directory. Capture evidence has 129
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
`assets/paper_evidence/fig04_turntable_author_current_v1/`.
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
Four selected local, timeline-bound outcome stills cover steps 136, 120, 360,
and 90 in source order. With the compact local bundle present, verify-only uses
the selected clips, panels, timelines, stills, traces, and provenance without
the pruned raw capture staging.

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
`assets/paper_evidence/card_house_author_5_construction_current_v1/`.
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

## Current-Source Four-Level Figure 6 Adapter

Preserve this lane separately from the reconstructed
`fbf_paper_card_house_26` scene:

- scene: `fbf_author_card_house_4_impact_current_source`;
- schedule: `card_house_author_4_impact_current_source`;
- author commit: `b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0`;
- source-supported selection: `--solvers fbf --levels 4 --frames 600
  --drop-frame 400 --num-cubes 4 --mu 0.8 --cube-size 0.4
  --cube-density 500 --drop-height 1.0 --device cpu --profile --usd`;
- source-default boundary: no-argument source is five levels and 800 frames;
  the selected command is not known to be the historical paper invocation;
- source-contact boundary: source `ke=1e4`, `kd=1e3`, and `gap=.005` are
  recorded source semantics, not contact semantics implemented equivalently by
  the DART adapter;
- inventory: 20 leaning plus 6 bridge cards, and four initially kinematic
  `0.8 m`, `256 kg` cubes; interactive `p` releases them immediately, while
  the evidence runner invokes `p` after completed substep 1,600;
- horizon: 2,400 substeps at `dt=1/240 s`; and
- lane parity: exact and boxed use Native `FourPointPlanar`, contact capacity
  4,096, and manifold subdivision 4.

The demo build, 13 headless/continuation C++ tests, 338 runner Python tests,
and both contract-smoke validators pass. The first strict exact request for 100 steps
fails closed at completed step 35 as contacts jump from 44 to 68. Steps through
34 are clean with prior worst residual `9.826274595482653e-7`; the failed
prefix has 103 exact attempts, 102 solves, one failure, zero fallbacks, zero
accepted caps, and worst residual `4.1039190451256334e-4`. Evidence:
`/tmp/fbf_author_card_house_4_exact100_last_failure_current_source_20260721/timeline.json`,
SHA-256 `2d04d31134426ac2c4fc87b1774d5285b77740acaeb3ec3a005557b85944bb9d`.

The exact adapter now matches source `project_after_correction=false` through
an ABI-neutral option whose DART default remains enabled. Strict 36- and
100-step replays still fail at step 35 with identical diagnostics:
residual/best/dual `4.0845653576327421e-4`, primal
`3.9380158679450451e-6`, complementarity `2.3818176330330057e-4`, 200
iterations, zero caps, and zero fallback. Timelines and SHA-256:

- `/tmp/fbf_author_card_house_4_source_correction_exact36_20260721/timeline.json`,
  `686be7170e3c217bfa917698a449e7ecde40e500a2c87d073ed58ba2ac833bfb`;
- `/tmp/fbf_author_card_house_4_source_correction_exact100_20260721/timeline.json`,
  `1a76b71fc4558c7cb978eab410a95948ae50e66522e45dbded07dd36aeb11a77`.

The exact adapter also matches the pinned source's raw inner initialization:
each inner solve and rejected step-size trial starts from the current outer
reaction without first projecting the seed. This is an ABI-neutral,
default-off option; DART's carried, projected inner seed remains the default
outside this adapter. Strict 36- and 100-step v3 replays with both source
policies active still fail at step 35 with byte-identical diagnostics: 56
contacts, 200 iterations, final/best residual and dual
`4.0844850280896461e-4`, primal `3.9375947649884479e-6`, complementarity
`2.3815426453852184e-4`, zero accepted caps, zero boxed fallbacks, and zero
line-search shrinks. Timelines and SHA-256:

- `/tmp/fbf_author_card_house_4_source_inner_exact36_v3_20260721/timeline.json`,
  `8909e915b63bb2c412a5c5289a5aa690dc1a9ef1d712fe531d12a38d626f0d2e`;
- `/tmp/fbf_author_card_house_4_source_inner_exact100_v3_20260721/timeline.json`,
  `3e379747bac636c259fe7e9bbd711bb57d5a719d5a1d8d6b9e6317e20b639f73`.

The Figure 6 adapter and strict replay disable colored block Gauss-Seidel. A
one-factor c95-bound probe changes only that switch and executes the colored
ordering/path for 200 solves with one participant and zero parallel dispatches.
It still fails the same step-35 group and improves residual by only `2.19e-14`
relative. Reject it only as the next Figure 6 blocker discriminator, not as a
multicore or general colored-BGS result. Source shrink-cap,
plateau, and continuation semantics are unchanged by this strict A/B and are
exercised only by the separately labeled continuation lane below.

The isolated c95-bound one-global-group A/B also exits at completed step 35.
The native 56/8/4-contact solve reproduces the failed
`4.0844850280896461e-4` residual. The global 68-contact solve reaches
`4.0848243204467147e-4`; its sliced 56-contact residual is
`4.0848243204472058e-4`, while the other two native islands pass. Every
off-block `W` coefficient is exactly zero. Both modes pass generation 28 from
identical contact fingerprints with different within-tolerance reactions;
contact fingerprints first diverge at generation 29. Reject native-island
scope alone as the step-35 cause, not global/per-island equivalence or any
source, trajectory, outcome, timing, performance, superiority, video, Figure
6, or paper claim. The isolated report and `SHA256SUMS` manifest under
`/tmp/fbf_fig06_global_scope_c95.TSfONI/` hash to
`633828adbe08577b6d0973ca817194530ed8a08cbe27e85d2bcb004689919fe9` and
`90d72452c6b3ed09e0bc1e408b56e70092557784fd2089e6895d7a31a0c809d3`.

An isolated c95-bound source-gap diagnostic changes only the four-level
scenario's existing source-gap flag. The resulting Native contract enables
negative-depth predictive closure with a `0.1 m` ground gap and `0.005 m` on
all 30 dynamic shapes while retaining strict source-inner serial BGS and zero
cap acceptance/fallback. It fails after completed step 31 on a 31-contact group
at 200 iterations and residual `1.0006073317077885e-5`, with 186 attempts, 185
solves, one failure, zero accepted caps, and zero fallbacks. The compared
contact streams differ from step 1; the existing stock sidecar embeds ancestor `844c9c3`,
not a fresh c95 binary. Reject only the hypothesis that this gap representation
clears the strict 36-step prefix. Preserve the checked-in scene and make no
general gap, trajectory, outcome, backend, float32, timing, performance,
superiority, video, Figure 6, or paper-parity claim. Verified package:
`/tmp/fbf_fig06_gap_c95.m6bsif/`; `RESULTS.md`
`3b0948c80871d19cbe29495a8abc57ac4f3e92dc518a9ae6551238a9aad9b17a`;
`SHA256SUMS`
`11888f98a24175f50c09ce95509d754d0bbc1963e5d2294ad982ece280292119`.

The c95 cadence candidate changes only the internal exact-FBF residual sampling
default from `1` to source cadence `5`. Its single strict run still fails at
completed step 35 on the 56-contact group: 200 iterations, residual
`4.0845024466967225e-4`, 103 attempts, 102 solves, one failure, 3,450 total
iterations, zero accepted caps, and zero fallback. Successful-iteration sums
are all divisible by five. The copied stock sidecar is ancestor-bound to
`844c9c3`, and the two source-binding hashes do not cover the patched math
header. Stock deltas are contextual only; no same-binary controlled A/B or
causal cadence estimate exists. Do not ship the global-default patch: two
cadence-specific tests pass, but two legacy-default tests fail. Preserve the
main tree and make no trajectory, outcome, backend, float32, performance,
superiority, video, Figure 6, or paper-parity claim. Verified package:
`/tmp/fbf_fig06_residual_cadence_c95.0QXC5c/`; `RESULTS.md`
`1f57c569f7feacb2c681cb17a70743782f07822abcbc1eb13d7822d81e9df18f`;
`SHA256SUMS`
`69db5e8915fadc31aae34d94c5f484928841b286566e172eea9535ee262d7645`.

The c95 terminal spectral-estimate A/B uses one instrumented Release binary
and changes only `rayleigh11` to `last_norm10`: retain the tenth product norm
and skip the terminal Rayleigh product. The exact control reproduces completed
step 35 / attempt 101 / 56 contacts, residual `4.0844850280896461e-4`, 103
attempts, 102 solves, and one failure. The single recorded variant satisfies
all 103 ten-product/no-Rayleigh trace checks but still fails at the same
step/attempt/contact count with residual `4.07679549813362e-4`. The residual
first diverges at attempt 57 / step 29; recorded contact-frame/reduced-state
hashes and product-norm sequences first diverge at attempt 67 / step 30. The
reduced-state hash covers only contact count, `freeVelocity`, and coefficients;
the sequence stores norms only. These summaries do not cover `W`, operator
identity, the initial reaction, the complete reduced problem, or product
vectors, so final residual/gamma deltas are contextual.
Reject only `last_norm10` as this strict
gate fix; DART's float64 all-ones seed and coordinate order remain unmatched
to the source. The sealed marker/timeline/trace triplet per selector is
internally consistent with the one-shot protocol, and the run guard refuses
reuse of those output directories, but neither proves no discarded run
occurred elsewhere. Verified
package: `/tmp/fbf_fig06_spectral_terminal_c95.OjNIB4/evidence/`;
`RESULTS.md`
`e33894ab0b771544209d48724641716c491b04073ec5bec533c07df653e54cda`;
`comparison.json`
`8b7af123ccaa42fd9c6bbeb0916c5b691ed3234c428ae62e404e6f26449227f6`;
`SHA256SUMS`
`f18efba2ffb1f7f8ee0f88798c9bcd38103b571210949de5c0cc625fed3fd553`.

The sixth bounded four-level strict-prefix diagnostic is a same-binary c95
source-seed-values A/B. It changes only the initial-vector selector from stock
`ones64` to `rs42_f32_values_dart_norm64`; both arms retain `rayleigh11`, DART
`[n,t1,t2]` order, float64 Eigen normalization and power products, ten
configured products plus the terminal Rayleigh product, and the frozen scene
and strict policies. The variant promotes the raw NumPy-2.4.4
`RandomState(42).randn(4096 * 3).astype(float32)` prefix to double before the
unchanged DART normalization. Its registered 168-value raw prefix has SHA-256
`7506d5e093b6e3787fccb4c91aee3a26feffd8548637a9a76825ad1a9f3ccfe1`,
and the selector aborts above that dimension.

The control exactly reproduces completed step 35 / exact attempt 101 / 56
contacts / 200 iterations, residual `4.0844850280896461e-4`, 103 attempts, 102
solves, and one failure. The sole recorded variant also fails there with
residual `4.1638905763175730e-4` and best residual
`4.1593800452634807e-4` at iteration 199. Seed, product-norm, and retained
estimate telemetry differs from attempt 1; final residual and iteration count
first differ at attempt 57 / step 29, and contact-frame plus recorded
reduced-state hashes first differ at attempt 67 / step 30. The reduced-state
hash omits `W` and the complete solver input, while `product_norms` omits
product vectors, so all post-divergence residual and gamma deltas are
contextual. The supported verdict is only that these raw source values,
promoted to double and normalized by DART's unchanged float64 path, do not
clear the frozen 36-step gate. This does not establish source-estimator or
coordinate-order parity, a root cause, a longer trajectory, Figure 6/video
parity, timing, performance, or superiority. No visual verdict applies. The
one marker/timeline/trace triplet per arm and path-reuse guard are internal
protocol evidence, not external proof that no invocation was discarded.
Verified package:
`/tmp/fbf_fig06_source_seed_c95.Uemp3S/evidence/`; `RESULTS.md`
`07b2f08f55bcb0210149e441c1886601d2a1f1d60d4f094b53f475ceaec88da3`;
`comparison.json`
`8897b3d826789baaba11ec9c1fea47569f108f82937b41978445f51aad028aeb`;
`SHA256SUMS`
`b2ecc0cf5c84a58448b8a1eafbb03ecda05e4f9935be193d3cd79ded87676a41`.

The pinned author control completes 2,400 steps with 1,455 converged and 945
unconverged flags (632 caps, 313 plateaus): 1,332/268 before release and
123/677 from release onward. First false/cap indices are 33/35; worst natural
`final_residual` is `2.59804445965485` and worst per-step final checked
`r_coulomb` is `7.597910320688573`. History:
`/tmp/fbf-sca-2026-author/paper_examples/card-house/results/20260721T175341Z/fbf/history.json`,
SHA-256 `b67d3c86f106171008dfbb0aca0a2ca72a9d3747c1a7a6694f57f211d3f83afd`.
Zero-cap completion is a strict scientific gate, not source-equivalent
continuation. The separately labeled telemetry-rich continuation lane is now
implemented and captured below; it does not alter the strict blocker.

The boxed control completes 100 steps:
`/tmp/fbf_author_card_house_4_boxed100_20260721_contract_v2/timeline.json`,
SHA-256 `fdd3d9e96058176faa51b148d1bcf5a4c0a7f1c4e7da64e15490dcae4ce6fafc`.
The additive sidecar `last_failure` object now retains the 56-contact failed
island after the subsequent 8- and 4-contact groups succeed. It records 200
iterations, residual/dual `4.1039190451256334e-4`, complementarity
`2.4220067503580449e-4`, and worst dual/complementarity local contact 11.
Bounded option tuning did not produce a strict 100-step completion. See
[FIGURE6_CONVERGENCE_BLOCKER.md](FIGURE6_CONVERGENCE_BLOCKER.md).

An unsealed debugger-mutated accepted-cap preview reaches release and all
2,400 steps, but 1,106/3,231 solves cap and worst residual reaches
`0.61608914241359314`. It proves finite continuation only. The strict exact
lane still does not reach release and boxed remains bounded to 100 steps.
There is no valid strict physical-outcome verdict, strict final media or PR
video, paper trajectory, source-backend or timing equivalence, Fig. 6 parity,
or solver-superiority claim.

### Source-Continuation Capture Truth

The separate scene/schedule pair
`fbf_author_card_house_4_impact_source_continuation_current_source` and
`card_house_author_4_impact_source_continuation_current_source` uses the same
26 cards, four cubes, Native `FourPointPlanar` frontend, 2,400-step clock, and
step-1,600 `p` release as the strict adapter. Only its exact lane opts into the
source-continuation policy. The paired boxed run carries no exact-only policy
claim.

Ignored durable bundle:
`assets/paper_evidence/fig06_card_house_source_continuation_current_v1/`.
Current-contract reseal source:
`/tmp/fbf_fig6_source_continuation_pair_20260721T1414_v2/`. Both lanes complete
2,400/2,400 steps and the action succeeds at step 1,600. Exact records 3,351
attempts/solves, zero exact failures or boxed fallbacks, 2,605 ordinary
successes, 113 plateau accepts, 633 max-iteration accepts, and zero shrink
caps. The 746 accepts are 22.262% of 3,351 solves and occur across 723 steps.
Worst final residual is
`0.91712002943322535`, first reached at step 2,101. Therefore this is
source-continuation evidence, not strict convergence.

Independent inspection of the synchronized 301-frame clip finds both
structures standing through release. Exact and boxed are pixel-identical only
at step 0; viewport difference is 0.165% at step 1,600 and 11.985% at the
endpoint. In this DART source-parameterized scene, exact completes without
exact-solver failures/fallbacks and visibly retains more upright card-house
structure after impact than boxed. The official MuJoCo panel degrades while
settling, whereas DART boxed remains upright until impact, so do not map the
DART lanes to the paper lanes or infer a mechanism. This is not a quantitative
trajectory or physical-outcome match, approved golden, source-backend
equivalence, timing comparison, or superiority result. The generated summary
records `paper_comparable=false` and no automated semantic-outcome validation.

SHA-256 anchors:

- summary:
  `6888f4729c99d41753c9c8ec9a1ec2ec9e2367c71da76aab973f8f8c5e8674cc`;
- exact timeline:
  `a9eb12711419b7801037d17059560559893be2898e07d14425a5f572175482ff`;
- boxed timeline:
  `1618e284f97ff7ed49e3288636269f5bea6131faa3bae45428e42e23de660bd8`;
  and
- paired clip:
  `282aebfb9e2e38fe3741db28e2ce909fb548d7aa46d048302a3b0e0bea9e1786`.

The freshly downloaded official video SHA-256 is
`d5356e1b31487be62b75af05efbfecdb70ad5d98501a8efd378fcedf066e4794`,
exactly the audited value. The local media remains outside Git and is not
published until its PR user-attachment URL is recorded. Preserve the strict
step-35 negative independently.

## Source-Continuation Ten-Level Capture Truth

The separate
`fbf_author_card_house_10_impact_source_continuation_current_source` scene and
`card_house_author_10_impact_source_continuation_current_source` schedule use
the same 155 cards, four cubes, Native frontend, 3,200-step clock, and
step-1,600 release as the strict ten-level adapter. Only the exact lane enables
source continuation. This is not a strict-lane fix.

The final exact reseal passes every requested step and the release action. It
records 7,702 attempts/solves, zero failures or boxed fallbacks,
2,427 plateau accepts, 763 max-iteration accepts, zero shrink caps, 310,880
total iterations, 7,630 warm starts, 1,071 maximum/987 final contacts, final
residual `7.709159985211234e-8`, and worst residual
`0.59964511064890469`. Timeline validation passes for 3,201 represented states
and 401 captured/unique frames. Its 660x506 H.264/yuv420p clip has 401 frames
at 30 fps over 13.366667 s, and full decode passes.

The metadata remains intentionally fail-closed with `paper_comparable=false`
and `automated_semantic_outcome_validated=false`. Final panel/keyframe
inspection confirms legibility, successful release, visible post-release
evolution, and lower structure remaining at the endpoint; it is not a
physical-outcome oracle. The final artifact hashes are timeline
`7a4b7d878f73068e10c59073b8e1260444a02529db62ab42eaf5c46425a190ae`,
clip `19637c4255c890f1f32383e7e7e680169688e5d8b071168bc6b4ffdebf33061d`,
panel `e5ed0d63ca9818292c5a373f476f2841f280f3e01492e0065b2aec8eb95a74d6`,
and metadata
`223e828a5284f9fc6aad0b7f57ef010d58db004d85759d036f47883b3753ed9f`.

The final ignored root is
`assets/pr_media_final/card_house_author_10_impact_source_continuation_current_source/`.
The separate run summary `/tmp/card10_exact_final_summary.json` has SHA-256
`9a551a96176e5112fc9f1443586c8aee115e1c25c10f766d0088efe4a088e3b2`
and reports `pass=true`. Independent reuse verification passes in 352.27 s;
its separate summary `/tmp/card10_exact_final_verify.json` has SHA-256
`83f9e9db5d013ab8359d5ee5dfb2d05fb4a116082d090b168ec02708ea5a348e`,
kind `verification`, one result, no skips or groups, full-decode success, and
the matching metadata hash. The role-separated
`/tmp/ten-cont-final-review-verify.json` is byte-identical at the same SHA-256.
The exact schedule has no blockers only within its
narrow continuation-evidence boundary.

A clean boxed control completes 80/80 in about 4 minutes 46 seconds with
`BoxedLcpConstraintSolver`; its timeline SHA-256 is
`ccbdc322791a06d5a8858818acae63e8540ca7770e635545e3c017d84bf96d7d`.
It is a bounded solver-identity/completion result, not a trajectory or
outcome. The full boxed attempt reached only
step 112 in about the same wall-time budget, was interrupted, and produced no
complete sidecar. Its partial frames are non-evidence; there is no full boxed
trajectory or paired clip. The runtime-prefix observation is not a performance
or solver-superiority claim.

Keep source/paper physical parity, Tables 6-7 parity, strict convergence,
trajectory parity, and solver superiority false. The final exact clip still
requires manual browser-composer upload and a recorded GitHub user-attachment
URL. Resolve the boxed runtime blocker before claiming paired media.

## Pinned-Author Masonry-Arch Scientific Negative

Preserve
`assets/paper_evidence/author_masonry_arch_reference_v1/`.
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

## Source-Pinned 101-Stone Standing Scientific Negative

Read [FIG08_ARCH101_DIAGNOSIS.md](FIG08_ARCH101_DIAGNOSIS.md) before changing
this scene or tuning the exact solver. Post-capture diagnosis shows that the
source's summed proximity gaps (`0.010` stone-to-stone and `0.105`
stone-to-ground) are absent from the zero-gap Native adapter. The unsupported
crown follows analytic free fall and crosses the standing displacement gate at
step 188, before the exact solver caps at step 209. Four one-factor solver
trials were rejected; even doubling inner sweeps only delayed the cap to step
235 at about 2.8x cost after the physical gate had already failed. The isolated
source-sized contact-gap trial is rejected too: it advances first contact from
step 39 to 37 but caps at step 161, 48 steps earlier, at residual
`1.5168150500676777e-6`. Keep the sealed scene unchanged and do not combine the
rejected knobs.
The independent pinned-source control also completes 1,600/1,600 only by
continuing after 1,473 capped/nonconverged substeps; its saved keystone drops
`7.2349853515625` raw units and 57/99 mobile stones exceed the local
three-unit height-change limit. Audit SHA-256 is
`56844eee3d908a1078fe7e76c6a92e31f2de79eb6e1e503ca7173d4e078c6cd4`.
It supplies neither a converged golden nor a visual-collapse/paper-parity
claim because the public runner saves no final x/y, rotations, or media.

The separate official current-source Kamino command completes all 400 frames
/ 1,600 substeps with finite arrays, but 98/99 mobile stones exceed that same
height-change gate. Maximum change is `87.23839569091797`; the keystone drops
`82.03050136566162`. Result and trajectory SHA-256 values are
`86b351c212c0e69df371fa66c67c53d6c3421575afbafae3a6b8d339f574dfb3`
and `63c47426019a218942afe3edae31cdf5dbbbd8d8926732ea59120e23bb6cf1a4`.
It persists no convergence/contact history, full poses, rotations, cube
trajectory, or media. Treat it as a current-source numeric negative, not a
historical Figure 8 oracle.

Preserve the ignored
`assets/paper_evidence/fig08_arch101_author_current_v1/` packet. The
`fbf_author_masonry_arch_101_standing_current_source` scene and
`masonry_arch_101_author_standing_current_source` schedule bind author commit
`b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0`, run blob
`35a052d7ef0975e7c828c9678d163054dfbb3ef2`, run.py SHA-256
`7e9158240267bb0ec1d0316b1badd4f3c8e1cd10270322de2e205cfea96f6f73`,
the 101-mesh tree
`e0c209235673d2f69c3c5de7708ab1dfadec96e3`, its path-sensitive manifest
`7198f71730d06dd70af8703065541765bd6b6f5da137f28f9befdf7acc5f96bf`,
and the public `--stones 101` selection. The inventory is 101 tapered meshes,
99 mobile stones, two fixed springers, and three pinned kinematic cubes. It uses
400 display frames at four 240 Hz substeps per frame. Because
`drop_frame=400`, this 1,600-substep / 6.67 s schedule performs no release.

The strict exact runner fails closed after 209/1,600 completed substeps on an
`iteration_cap`: 208 contacts, 5,000 iterations, residual/best/worst
`1.2582804496066107e-6`, 342/342 cumulative exact attempts/solves, one accepted
cap at the offending state, zero exact failures, and zero boxed fallbacks. Its
210 state samples end with crown height `62.4010546875` from
`66.1385625`, maximum mobile-body displacement `3.7375078125`, and maximum
rotation `0.00361745 rad`; the incomplete trace and standing oracle are false.
Exact timeline SHA-256 is
`df1ed4afc9ef5aa74f7c0b6da0560ae0d1b63fca28f45051ed27c5dfb3632889`.

Boxed LCP completes 1,600/1,600 with all 1,601 inventory, finite-state, and
cube-pinning samples valid, but fails the standing oracle. Maximum mobile-body
displacement is `21.2188459736`, maximum rotation is `3.14152663339 rad`, and
the crown finishes at `61.1013192467` after reaching `58.3806809854`. Its
timeline SHA-256 is
`a8caee71c9356a72fa65210207d7b4209d9e305363974ec07c81f19ec14bfa1e`.
The 201-frame H.264/yuv420p clip is 660x506, 30 fps, 6.7 s, fully decodes, and
has SHA-256
`7635c2722b20fb8bcb0255054cc9172153d1dd640fd8e81df4df52c0e515d3c0`.
Manual inspection records an intact arch at steps 0 and 400, loss of the
crown-standing configuration by step 800, and visible collapse by steps 1,200
and 1,600.

The 1320x506 diagnostic comparison freezes exact at its last rendered step-208
frame while boxed continues; its 201-frame, 30 fps, 6.7 s full decode has
SHA-256
`d6f5f658e4fb027edb23e0911acd34b74dfd749daace41b5d9c9204af3163b94`.
It presents the blocker rather than a complete exact/boxed comparison. Capture
and independent boxed reuse verification pass: run-summary, verification, and
boxed-member metadata SHA-256 values are respectively
`f8db81bebaf11e794f39bd3c50cb2f9b870c40007c53764dd9cc602f82d57dfa`,
`7696977f68e071613bfcacae2c9430830d42d38b61421365a11e42884a2c42ef`,
and `6a1d7360e6a6834bda656a626ce3f579824d5d0d04f418c356ad8efb28c2db87`.
The compact `fig08_summary.json` SHA-256 is
`1c19c6c3c36171a5e85f330b2863b429956652fb894aae0aa0b82d68291e3481`.
GitHub attachment URLs remain pending. This current-DART negative does not
establish source collision/backend/float32 semantics, trajectory or outcome
equivalence, Fig. 8/video.08 parity, a historical source/Kamino golden, approved goldens,
timing, performance, or solver superiority.

## Preregistered Crown-Impact v1 Negative

Use [LITERAL_CROWN_IMPACT_V1.md](LITERAL_CROWN_IMPACT_V1.md) and
`assets/paper_evidence/fig07_arch25_literal_impact_v1_negative_final_v9/`.
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
`assets/paper_evidence/fig08_arch101_literal_v1_negative_final_v7/`.
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
`assets/paper_evidence/card_house_26_manifold_sensitivity_v2_r3/`.
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

The manifest validator currently passes and fail-closed hashes local bundle
artifacts, materializes the current bundle indexes, binds recorded process,
taskset, topology, residency, and archived-prior-source provenance, recomputes
CPU claims from raw rows, and enforces every current-truth promotion boundary.
Fig. 1, Fig. 2, and video.03 bind the finalized lane-separated incline
threshold evidence plus the pinned-author seven-cell numeric sweep while
retaining their partial status. Fig. 5 and video.05 now bind the locally
validated source-pinned DART-adapter outcome evidence above as well as the
separately labeled historical proxy. They remain incomplete pending GitHub
publication and the explicitly false source/backend/trajectory/paper gates.
Fig. 7 and video.07 bind the validated standing
trace/still/video/outcome/claim-map artifacts, while impact-v1 and arch101-v1
are recorded as scientific negatives and card-manifold v2 as non-strict
diagnostic comparison evidence. Do not mechanically promote
requirements whose source-equivalence, golden, external-baseline, or impact gates
remain missing.
The task docs now record the separately named source-pinned 101-stone DART
scientific negative above, but the local manifest sidecar has not yet been
synchronized to that packet. Keep the 29-row counts unchanged until its
validator binding is updated. The boxed media, exact failed prefix, and full
current-source Kamino numeric negative improve the blocker evidence, but
video.08 remains blocked by the incomplete exact lane and unavailable
historical invocation/backend, full-pose oracle, and golden-media contract.

The P-core mark26 bundle is the authoritative local performance/scaling
evidence for the reconstructed literal arch. The separate current-source small
`paper_cpu`/Native matrix at
`assets/dart_cpu_evidence/2026-07-19_current_source_paper_cpu_small_r7/` is also
locally finalized: it binds 60 artifacts, 27 complete CPU-4 invocations, and
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
| Exact-Coulomb math, Release | 56/56 |
| Exact constraint solver | 38/38 |
| `ConstraintSolver` integration | 66/66 |
| Native collision | 50/50 |
| `SplitImpulse` | 13/13 |
| Masonry wedge dynamics | 3/3 |
| Default paper fixtures | 36 passed; 3 explicit opt-in skips |
| Focused Release/Debug CTest matrix | 9/9 in each configuration |
| Schema-v8 CPU evidence unit tests | 230/230 |
| Literal-wedge visual finalization unit tests | 16/16 |
| Crown-impact trace and negative-runner unit tests | 25/25 |
| Literal 101-stone trace/probe/runner unit tests | 41/41 |
| Finalized incline unit/verify-only gates | 62/62; with the compact local bundle present, verify-only passes with 21 indexed artifacts and no raw capture-staging dependency |
| Author-incline reference finalizer unit tests | 64/64; verify-only reports 37 indexed artifacts / 39 physical files |
| Focused manifest/backspin/incline/author-masonry/author-incline evidence suite | 859 passed post-merge in 163.95 s |
| Full no-cache dartpy Python suite | 1,555 passed in 165.09 s |
| Current-source four-level author-card demo build | Passed |
| Current-source four-level headless/continuation C++ fixtures | 13/13 passed |
| Source-supported ten-level demo and focused C++ fixtures | Build passed; all four `AuthorCardHouseTenLevel*` tests passed, including the continuation contract |
| Visual runner, including source-pinned 101-stone, both ten-level card-house schedule/oracle contracts, and Figure 7 crown-impact continuation | 338/338 passed |
| Shared-library ABI symbols | Existing nine-argument failure-record method and correction-policy methods retained; additive source-inner setter/getter exported without a public class-layout change |
| Four-level exact/boxed adapter contract smoke | Both passed |
| Author masonry-arch focused CTest | 1/1 target passed; 8/8 contained tests |
| Demo scene documentation verifier | 29 scenes passed |
| Manifest host-identity modes | Sealed producer closure: 118 live file rechecks, 0 skipped; explicit archive: 0 live, 118 skipped; both validate 29 requirements with status `partial` |
| Deterministic colored-scheduler stress | 1,000 runs passed |
| Independent post-fix re-review of `3647959a188` | `ALLOW` |

The sealed live closure resolved `libdart.so.6.19` to the recorded
`libdart.so.6.19.3`. The normal development symlink is restored to
`libdart.so.6.19.4`; the files are byte-identical, but the resolved path is
identity, so the current default live run reports five path mismatches. Archive
mode remains clean; recreate the historical symlink only for an explicit live
closure check.

## Active Completion Gates

| Gate | Current state | Required next evidence |
| --- | --- | --- |
| Literal 25-stone static arch | Exact trajectory, residual, outcome, mean-real-time, multicore, and earlier trace-equivalent visual gates pass. The c95-bound exact/boxed recapture and independent verification pass, but its visible-standing assessment is manual-only | Preserve both evidence roots and the narrow reconstructed no-projectile claim; upload the labeled c95-bound group through the PR browser composer and record its URL without promoting a physical/superiority/Fig. 7 claim |
| Projectile impact sequences | Frozen v1 remains a scientific negative. The separate current-source continuation pair completes through release, but has accepted non-strict outcomes and only manual, nearly identical visual observations; strict source configuration remains blocked at step 142 | Preserve v1 without tuning and continuation without promotion; upload its group through the browser composer, then obtain source-equivalent strict evidence and a real outcome oracle from a separately declared contract |
| Card house | The strict adapter still fails the 56-contact group at step 35. A separate source-continuation exact/boxed capture completes 2,400 steps through release with full acceptance telemetry and qualitative manual inspection. Its colored-ordering, one-global-group, source-sized-gap, residual-cadence, terminal spectral-estimate, and source-seed-values candidates are ineffective | Preserve all six bounded rejects and require a new source-backed, preregistered mismatch before another strict solver A/B; none rejects the literal-arch colored result or pending ten-level diagnostics. Publish the continuation clip only as continuation evidence, never present accepted finite iterates as strict success |
| 101-stone arch | Source-pinned DART exact stops at step 209 on an iteration cap; boxed completes but collapses. Full current-source FBF and Kamino controls also fail the standing gate. The older literal v7 and oriented-box negatives remain distinct | Preserve the precise negatives and decoded blocker media; recover the historical invocation/backend or a matched historical oracle before attempting parity-eligible media |
| 10-level card house | Strict remains blocked after completed step 31. The final continuation exact member passes 3,200/3,200 and independent reuse verification, but has no automated semantic-outcome validation. A clean boxed control completes 80/80; full boxed reached only step 112, was interrupted, and is non-evidence | Preserve both strict checkpoints and final hashes. Resolve boxed runtime before paired media, and manually upload only the final exact clip. Keep colored/global-scope work one-factor-at-a-time; do not loosen tolerance, caps, fallback, or accepted-cap policy |
| Small figures and video | The implementation/media-head `c95bd5fb916` reseal covers 20 Figures 1-5 exact/boxed members and 13 groups with zero failures; five exact-only author-turntable boxed skips are expected | Preserve the ignored reseal and independent verification; manually upload the accepted group clips through the PR composer and record their URLs while retaining every DART/source/history/golden/media boundary |
| Paper performance | Current source is available, but matched DART runs are pending and historical comparability remains partly external | Audit and run the pinned author workloads; keep the exact historical Apple host, renderer, and timing-attestation gaps explicit |
| Manifest | Validator passes with 24 partial, 5 blocked, and 0 complete | Keep every row bound to current artifacts while retaining missing source-equivalence and historical-parity blockers |
| CI/PR | PR #3377 is draft; implementation/media evidence is bound to `c95bd5fb916`, while later documentation-only checkpoints do not change that binary/media identity | Keep it draft, synchronize the truthful body after each push, and require current-head CI |
| Final review | Not current | Two clean independent reviews of the final post-fix state |

## Immediate Work Order

1. Inspect current diffs and active agents before taking file or build-tree
   ownership.
2. Preserve the P-core, standing-visual, historical finalized Painleve proxy,
   finalized incline and backspin, pinned-author numeric incline sweep,
   frozen impact-v1/arch101-v1 negatives, the source-pinned 101-stone DART
   negative, card-manifold-v2 diagnostic, and
   finalized source-pinned Painleve exact/boxed capture with its complete state
   traces, outcome audit, independent replay, and manual inspection. Continue
   one-factor-at-a-time strict work on the step-35 failure without
   changing tolerance/caps/fallback/fail-fast. The one-participant Figure 6
   colored-ordering, one-global-group, source-sized-gap, residual-cadence,
   terminal spectral-estimate, and source-seed-values discriminators are
   ineffective; require a new source-backed, preregistered mismatch before
   another strict solver A/B, while keeping the ten-level colored/global-scope
   diagnostics pending. Preserve and
   independently review the completed telemetry-rich
   source-continuation capture without calling accepted finite iterates strict
   success; upload it only through the PR editor after explicit approval and
   record the user-attachment URL. For the ten-level lane, preserve the final,
   independently reverified exact continuation member. Preserve the clean 80-step
   boxed control, but treat the interrupted step-112 boxed
   attempt and partial frames as non-evidence; resolve that runtime blocker
   before paired media. Keep global-scope diagnostics one factor at a time; do
   not loosen tolerance or iteration caps. Use the ignored c95-bound
   `c95bd5fb916` reseal as the authoritative Figures 1-5 upload source. Upload any
   final ten-level exact clip manually through the PR browser composer and
   record its URL. Upload both c95-bound Figure 7 exact-vs-boxed groups and the
   two Painleve exact-vs-boxed clips
   through the same browser-composer path and record their URLs. Continue
   generating the remaining full card, strict 101-stone, figure, and
   source-equivalent impact artifacts. Upload the 101-stone diagnostic only as
   a frozen-prefix blocker. Decode and manually inspect media; do
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
