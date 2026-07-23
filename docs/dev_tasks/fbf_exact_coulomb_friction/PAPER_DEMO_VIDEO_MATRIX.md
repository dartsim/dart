# Paper Demo and PR Video Matrix

Status: active draft gate for PR #3377. The PR must remain draft until every
required row below has a current GitHub-hosted video URL and passes its solver,
physical-outcome, and claim-boundary checks.

Generated frames, sidecars, traces, and encoded media belong in the ignored
`assets/` working directory or `/tmp`. They are not committed. Review clips are
H.264/yuv420p MP4 files produced and decoded by
`scripts/run_fbf_visual_evidence.py`, then attached through the pull-request
editor. Record the resulting `github.com/user-attachments/...` URL here and in
the PR description.

Checkpoint `67073f4f575` adds generic no-render
`dart-demos --scene-physics-contract <scene-id>` queries and fail-closed
semantic physics provenance for 10 provider schemas. Provider-backed captures
use result schema v2 only after exact canonical live/sidecar equality. The
semantic digest separates exactly one schema-declared monolithic implementation
hash, but full demo-binary and broad source gates remain mandatory, and the
binary is hashed before and after capture. Card-house construction and
author-turntable consumers validate v2; legacy/no-provider captures remain v1.
This does not make any historical v1 clip current-head-reusable: each still
requires current-head recapture/reseal or its archived original binary.
Checkpoint `c0364afd390` separately updates the backspin finalizer's checker
source contract for the current two-argument helper and call site, fixing the
observed macOS CI failure caused by the old one-argument expectation. The
no-cache `dart-demos` build and 665 focused provenance/runner/consumer tests
pass; two independent current-implementation reviews are clean, while fresh CI
and future provider/consumer changes remain explicit watches.

The current-head v2 reseal is complete under ignored
`assets/pr_media_current_head_67073/` for the five minimum Figures 1-5 source
rows. Both source-pinned Figure 3 lanes and their labeled group pass capture
and independent verification. The compact Figures 1-2/4-5 run separately
passes with 10 members, 6 groups, zero failures, and five expected exact-only
author-turntable boxed skips; run/verify summary SHA-256 values are
`8a8b83bba2d19ea9a1a03d90daa80554e909507bad9f95ccc4bc23c3ac36d0c7`
and `7599ecf8fc5ceb5338cf97633e710da3985981557b1babd61e52a5a653dec22f`.
All ten compact members use schema v2, bind demo SHA-256
`69879e77e55099f67c11530dfdde9dd2d4e1e4ac0d3167a1bb8b0b3945559efa`,
and have exact live/sidecar contract matches. All six intended upload hashes
including the Figure 3 supplement are byte-identical to the audited staging
copies. Browser upload remains pending.

The same ignored root now contains current-head v2 reseals for the Figure 6
and Figure 7 continuation pairs and the supplemental source-default five-level
pair. Capture and independent verification pass for all six members and all
three synchronized groups. Every member uses schema v2, binds demo SHA-256
`69879e77e55099f67c11530dfdde9dd2d4e1e4ac0d3167a1bb8b0b3945559efa`
and broad-source SHA-256
`e2cc2351a3043ec8301677d990adfe2c7da2b9762499b19bd81c5e3be1559337`,
and has exact canonical live/sidecar contract equality. Group provenance is
transitive through the bound member metadata; the group records do not claim
their own live/sidecar scene-contract query.

| Current-head row | Run summary | Independent verification | Synchronized group |
| --- | --- | --- | --- |
| Figure 6 source continuation | `assets/pr_media_current_head_67073/fig06_current_head_run_summary.json`, SHA-256 `cac41831322fcc58fcd9be405c7f7b15de45f88c21ca12a8e0f1007483bebfbd` | `assets/pr_media_current_head_67073/fig06_current_head_verify_summary.json`, SHA-256 `88890d3efba87432836ec9854a7415c8a7b2c07ba506adda1bac9aa3744360ef` | `assets/pr_media_current_head_67073/groups/card_house_author_4_impact_source_continuation_current_source__exact_vs_boxed/clip.mp4`, SHA-256 `7e87661d4b3bf34cdae192af069f159ead8407b4408b42cc6d15788ab6683b9e` |
| Figure 7 crown continuation | `assets/pr_media_current_head_67073/fig07_current_head_run_summary.json`, SHA-256 `ad1684d2c9917da6905a96b981ecdd8b50174b175f6af567127d6fa861c25e36` | `assets/pr_media_current_head_67073/fig07_current_head_verify_summary.json`, SHA-256 `fd0e0bef586425358d5613764c7878054e6895aee9b13de4310ad3b1edc96ec8` | `assets/pr_media_current_head_67073/groups/masonry_arch_25_author_crown_impact_source_continuation_current_source__exact_vs_boxed/clip.mp4`, SHA-256 `9a9a68229faeb1958cb8ed0449387ca310b0f41e692434ed58bcd39988792177` |
| Supplemental source-default five-level card house | `assets/pr_media_current_head_67073/card5_current_head_run_summary.json`, SHA-256 `155479cb42ddb90472b4816e9b65a63a788d78f5deeee78a6f219b2964130a65` | `assets/pr_media_current_head_67073/card5_current_head_verify_summary.json`, SHA-256 `22ba0e733695c1759dee8338d9d58e41975d0af02ed9be5796ede703baff918f` | `assets/pr_media_current_head_67073/groups/card_house_author_5_impact_source_continuation_current_source__exact_vs_boxed/clip.mp4`, SHA-256 `b46aeb3d9f09e95151e26fef4838432b6b071a5d3c39c3c9a489c6f1d42e875b` |

Manual endpoint panels show more retained multi-level structure in Figure 6
exact than boxed, nearly identical standing arches in Figure 7, and retained
upright multi-level structure in the five-level exact lane versus a collapsed
boxed endpoint. These are presentation-only observations: each exact lane
requests source continuation while boxed does not, automated semantic outcome
validation remains false, and no solver-only, strict-convergence, superiority,
source-backend, trajectory, or paper-parity claim follows. The ten-level
current-head recapture is active under the same root but has no final
current-head capture/verification result yet.

Checkpoint `8ad9961e56c` adds the standalone fail-closed Figure 8 failed-prefix
finalizer and verifier. It owns the expected strict failure after completed
step 209, freezes imagery only after the last rendered step 208, and requires a
complete boxed member plus an explicit manual seal. No current-head Figure 8
bundle has completed finalize/seal/verify yet, so the historical v1
frozen-prefix diagnostic remains the only local attachment candidate and must
not be called current-head v2 evidence.

## Completion Rule

A row is complete only when all of the following are true:

1. The named `dart-demos` scene runs for the declared duration from a clean
   build and the capture sidecar matches the requested scene and timeline.
2. The exact-FBF lane has zero exact failures, zero boxed-LCP fallbacks, no
   accepted-at-cap solves, residual at or below `1e-6`, and the expected
   physical outcome.
3. A paired boxed-LCP run is shown when the scene exposes the existing-solver
   toggle. The capture runner uses `--solver-lane both`, writes boxed artifacts
   under lane-separated `__boxed` paths, and validates the active solver from
   the sidecar. A successful old-solver outcome is acceptable; the comparison
   must report what actually happened rather than require failure.
4. Source-configuration differences and unavailable external-solver panels are
   stated in the clip caption. A visually plausible proxy is not paper parity.
5. The final MP4 is decoded and checked with `ffprobe`, is small enough for a
   normal GitHub PR attachment, and its GitHub-hosted URL is present below.

A separately labeled source-continuation clip may be a useful review attachment
without completing a row under rule 2. It must expose every plateau/cap/shrink
acceptance, remain distinct from the strict lane, and state that it is neither
strict convergence nor paper or solver-superiority evidence.

## Source Examples

| Paper/video example | Primary demo scene(s) and capture schedule(s) | Existing-solver comparison | Current gate | PR video |
| --- | --- | --- | --- | --- |
| Figs. 1-2 / incline segment | Source-bound `fbf_author_incline_sweep_current_source`; `incline_author_sweep_current_source`. The older `fbf_paper_incline`; `incline` pair remains a two-cell regression fixture | Paired exact/boxed capture and a solver-labeled synchronized seven-cell comparison are automated | Capture and independent reuse verification pass for both complete 120-step traces and the 61-frame group. All cells remain supported/upright/in-lane/contact-participating; `.3/.4/.45` slide and `.5/.55/.6/.8` stick. Exact/boxed maximum source-FBF endpoint deltas are `0.002426469449185232 m` / `0.0011201728594518558 m/s` and `0.0011521317667995284 m` / `0.00030012480388411203 m/s`. The retained source has 839/840 configured convergence flags, so this is a non-strict current-source terminal/outcome slice—not source trajectory/backend/full-physical/video/timing/paper parity or superiority | Local seven-cell exact-vs-boxed clip ready below; GitHub URL pending browser-composer upload |
| Fig. 3 / backspin segment | Source-pinned `fbf_author_backspin_current_source`; `backspin_author_current_source`. The older `fbf_paper_backspin`; `backspin` pair remains a reconstructed regression | Paired exact/boxed capture and a solver-labeled synchronized comparison are automated. The exact member is the preferred primary upload because its checker remains larger than in the side-by-side group | Both complete 240 steps / 241 state samples and pass prompt-contact, contiguous support, five-sample rolling tail, planar-motion, left-edge roll-off, airborne terminal, and source-terminal tolerance gates. Both solvers pass under the same DART scene; no solver failure, superiority, or equivalence is claimed. The checker is visual-only; signed `wy` is trace-bound, while 30 fps imagery cannot establish signed direction/rate. Source backend/full-trajectory/renderer/video/timing/Figure 3/paper parity remain false | Local exact primary and exact-vs-boxed supplemental clips ready below; GitHub URLs pending browser-composer upload |
| Fig. 4 / turntable segment | Four `fbf_author_turntable_*` scenes; four `turntable_author_*` schedules and `turntable_author` 2x2 group | Author-pinned scenes are exact-only. Paired exact/boxed capture and lane-separated groups are automated for the four `fbf_paper_turntable_*` proxies, which accompany the source-pinned group. | The current-head v2 author group passes solver, timeline, stream, decode, canonical live/sidecar contract, and independent replay gates. Its clip hash is byte-identical to staged upload media; manual source order is ejected/ejected/retained-through-6-s/ejected. The older c95 proxy groups remain supplemental and both proxy solvers classify ejected/ejected/captured/ejected over 4 s. These are manual current-DART classifications rather than Warp/Newton or paper trace parity. | Local current-head author group and four historical proxy exact-vs-boxed clips ready below; GitHub URLs pending |
| Fig. 5 / Painleve segment | Source-pinned `fbf_author_painleve_mu_0_5` and `fbf_author_painleve_mu_0_55`; `painleve_author_mu05`, `painleve_author_mu055`, and `painleve_author` group. The older `fbf_paper_painleve*` scenes remain historical diagnostics. | Paired exact/boxed capture and lane-separated grouping pass for both source-parameterized cells | The current-head v2 bundle passes capture, independent verification, complete traces, strict exact audit, outcome classification, full decode, and canonical live/sidecar contract checks. Manual panels show both lanes upright near rest at `mu=.5`, while exact tumbles near rest and boxed remains upright near rest at `mu=.55`. Both comparison clip hashes are byte-identical to staging. This is current-DART-adapter divergence, not source-backend, trajectory, paper, timing, or superiority evidence. | Two local current-head exact-vs-boxed clips ready below; GitHub URLs pending manual browser-composer upload |
| Fig. 6 / 26-card segment | Strict `fbf_author_card_house_4_impact_current_source` plus separate `fbf_author_card_house_4_impact_source_continuation_current_source`; the older `fbf_paper_card_house_26` reconstruction remains distinct | Both source-selected schedules support paired exact/boxed capture with Native `FourPointPlanar`, capacity 4,096, and subdivision 4 fixed. Only the continuation exact lane requests continuation policy | Strict remains blocked at step 35 on the 56-contact group. Colored ordering and global scope are bounded rejects; source-sized-gap, residual-cadence, same-binary terminal spectral-estimate, and same-binary source-seed-values candidates also fail the 36-step gate with explicit claim limits. These are six bounded strict-prefix rejects. The separate continuation pair completes 2,400/2,400 and releases at 1,600; exact records 3,351/3,351 solves, 0 failures/fallbacks, 2,605 successes, 113 plateau accepts, 633 max-iteration accepts, 0 shrink caps, and worst residual `0.917120`. Manual inspection shows both standing through release and more endpoint structure in exact. This is qualitative continuation evidence, not strict convergence, trajectory/outcome/golden/backend/timing parity, superiority, or paper parity | Local source-continuation exact-vs-boxed clip ready; strict/parity gate blocked; GitHub URL pending |
| Fig. 7 / 25-stone arch segment | Strict `fbf_author_masonry_arch_25_crown_impact_current_source` plus separate `fbf_author_masonry_arch_25_crown_impact_source_continuation_current_source`; corresponding schedules omit the `fbf_` prefix. Literal-standing and reduced-proxy lanes remain separate | Literal-standing and source-continuation exact/boxed pairs plus independent reuse verification pass | Strict source configuration remains blocked at step 142. The continuation pair completes 2,000/2,000 and releases at 1,600; exact has 2,122/2,122 solves, zero failures/fallbacks, 1,940 plateau and 98 max-iteration accepts. Both arches standing and cubes reaching the crown is a manual, nearly identical visual observation only. Metadata has false paper/semantic flags; no strict convergence, superiority, outcome, timing, backend, trajectory, or paper parity follows | Literal-standing and bounded continuation groups ready locally; strict gate blocked at step 142; GitHub URLs pending browser-composer upload |
| Fig. 8 / 101-stone arch segment | Source-pinned `fbf_author_masonry_arch_101_standing_current_source`; `masonry_arch_101_author_standing_current_source`. The older `fbf_paper_masonry_arch_101`; `masonry_arch_101` proxy remains diagnostic-only | Exact/boxed capture is automated, but the current exact failed prefix prevents a valid full paired comparison. Boxed has a complete decoded member clip; a diagnostic hstack freezes exact at its last frame and is labeled as such | Partial blocker evidence, parity blocked: strict exact stops after step 209 on an iteration cap (`208` contacts, residual `1.2582804496e-6`); boxed completes 1,600/1,600 but fails the standing oracle and visibly collapses. Independent full current-source FBF and Kamino controls also fail the same local standing criterion; neither supplies a historical invocation, convergence/contact oracle, full poses, or parity-eligible media. The isolated source-sized contact-gap variant is rejected because it caps earlier at step 161. None is a converged standing result, historical source trajectory/backend match, golden, timing, or paper-parity result | Local boxed-collapse and frozen-prefix diagnostic clips ready below; GitHub URL pending |
| Tables 6-7 / ten-level card house | Strict `fbf_author_card_house_10_impact_current_source` plus separate `fbf_author_card_house_10_impact_source_continuation_current_source`; corresponding schedules use the same names without the `fbf_` prefix. The older reconstruction remains diagnostic-only | Both schedules use Native `FourPointPlanar`, capacity 4,096, four contacts per pair, one `0.1 m` ground gap, and 159 `0.005 m` card/cube gaps. Only the continuation exact lane requests continuation policy | Strict reaches completed step 31 before a 79-contact failure. Same-binary exact and boxed continuation members complete 3,200/3,200 and pass independent verification. Exact records 7,702/7,702 solves, zero failures/fallbacks, 2,427 plateau accepts, and 763 max-iteration accepts; boxed has no exact-FBF telemetry. Their synchronized labeled pair fully decodes. Manual inspection finds retained upright multi-level structure in exact and a largely collapsed boxed endpoint, but the lanes differ in continuation policy and have `automated_semantic_outcome_validated=false`. This is not a solver-only A/B, strict convergence, automated physical outcome, source/trajectory/Tables 6-7 parity, superiority, or paper parity. See `CARD_HOUSE_10_CURRENT_SOURCE_DIAGNOSIS.md` | Same-binary presentation-only labeled pair is locally verified; manual browser upload remains pending |

The source-pinned Figure 5 adapter binds author commit
`b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0`, tree
`ffcdafb61adeda2239c8366d054b548b50d26685`, and Painleve `run.py` blob
`afaa03613b0ad0a30290168d2fd64221fc3523b7` (SHA-256
`818fa8f75c2c73e2dd08f0e0e9f9f5d58f63d8073dce38f874e2da24b2aa46e3`).
The adapter contract also binds SHA-256
`c48867ded0c3523e10eb47690aa5bf980db40281b219165cb8e31b0e492890f8`
for `dart/constraint/ExactCoulombFbfConstraintSolver.hpp`, so exact-option
default drift fails closed.
It reproduces the public source configuration in a DART adapter: a
`0.3 x 1.2 x 0.6 m` box in DART xyz order, density `200 kg/m^3`, mass
`43.2 kg`, upright center at `z=.3 m`, initial velocity `(4,0,0) m/s`, gravity
`9.81 m/s^2`, `dt=1/60 s`, 120 steps over 2 s, and the selected
source-supported `mu=.5,.55` sweep. The public source defaults to `mu=.55` and
does not identify the historical paper invocation. Source `gap=.005`,
`ke=1e4`, and `kd=1e3` remain recorded source semantics; DART Native
`FourPointPlanar` contact does not implement them equivalently. Exact and boxed
lanes therefore compare two DART solvers under one pinned adapter. This is a
DART adapter reproduction, not Warp/Newton backend, float32, trajectory,
physical-outcome-equivalence, timing, renderer/golden, Fig. 5, video, or paper
parity.

The source-selected Figure 6 adapter pins author commit
`b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0` and uses the supported selection
`--solvers fbf --levels 4 --frames 600 --drop-frame 400 --num-cubes 4
--mu 0.8 --cube-size 0.4 --cube-density 500 --drop-height 1.0 --device cpu
--profile --usd`. That is not the five-level/800-frame no-argument source
default or a known historical paper command. Source `ke=1e4`, `kd=1e3`, and
`gap=.005` are recorded source semantics, not contact semantics implemented
equivalently by the DART adapter. It contains 20 leaning plus 6 bridge cards
and four initially kinematic `0.8 m`, `256 kg` cubes. Interactive `p` releases
them immediately; the evidence runner invokes `p` after completed substep
1,600. The 2,400-step schedule uses `dt=1/240 s`.

The demo build, 13 focused headless/continuation C++ tests, five author-incline
production-world C++ contract tests, 665 focused semantic-provenance/runner/
consumer Python tests, and exact/boxed contract-smoke validators pass. The
exact 100-step prefix records
103 attempts, 102 solves, one failure, zero fallbacks, zero accepted caps, and
worst residual `4.1039190451256334e-4`; steps through 34 were clean with prior
worst residual `9.826274595482653e-7`. At step 35, attempt 101 is the failed
56-contact group; it reaches the 200-iteration cap before the later 8- and
4-contact groups succeed. The additive `last_failure` object retains
`fbf_failed` / `success` / `max_iterations`, the 56-contact count, and the
failed residual after those later successes. Its timeline is
`/tmp/fbf_author_card_house_4_exact100_last_failure_current_source_20260721/timeline.json`,
SHA-256 `2d04d31134426ac2c4fc87b1774d5285b77740acaeb3ec3a005557b85944bb9d`.

The pinned source's no-projection correction policy is now explicit in both
the source and DART adapter contracts. Strict 36- and 100-step replays produce
identical solver diagnostics and still fail at step 35: residual/best/dual
`4.0845653576327421e-4`, primal `3.9380158679450451e-6`, complementarity
`2.3818176330330057e-4`, 200 iterations, zero caps, and zero fallback. Their
timelines and SHA-256 values are
`/tmp/fbf_author_card_house_4_source_correction_exact36_20260721/timeline.json`
(`686be7170e3c217bfa917698a449e7ecde40e500a2c87d073ed58ba2ac833bfb`)
and
`/tmp/fbf_author_card_house_4_source_correction_exact100_20260721/timeline.json`
(`1a76b71fc4558c7cb978eab410a95948ae50e66522e45dbded07dd36aeb11a77`).
This closes the post-correction mismatch but does not clear or move the media
blocker.

The pinned author's inner solver also copies the current outer reaction into
each inner solve and rejected step-size trial without projecting that seed.
The current checkpoint provides that policy as an ABI-neutral, default-off
option and enables it only for this adapter's exact lane. Strict 36- and
100-step v3 replays with both source policies active still fail at step 35 with
byte-identical diagnostics: final/best residual and dual
`4.0844850280896461e-4`, primal `3.9375947649884479e-6`, complementarity
`2.3815426453852184e-4`, 200 iterations, zero accepted caps, zero boxed
fallbacks, and zero line-search shrinks. Evidence:

- `/tmp/fbf_author_card_house_4_source_inner_exact36_v3_20260721/timeline.json`,
  SHA-256 `8909e915b63bb2c412a5c5289a5aa690dc1a9ef1d712fe531d12a38d626f0d2e`;
- `/tmp/fbf_author_card_house_4_source_inner_exact100_v3_20260721/timeline.json`,
  SHA-256 `3e379747bac636c259fe7e9bbd711bb57d5a719d5a1d8d6b9e6317e20b639f73`.

The adapter and strict replay disable colored block Gauss-Seidel. A later
one-factor c95-bound trial exercises the colored ordering/path for 200
solves with one participant and zero parallel dispatches, but changes the
failed residual by only `2.19e-14` relative. Reject it only as the next Figure
6 blocker discriminator, not as a multicore or general colored-BGS result.
Source shrink-cap, plateau, and continuation semantics are unchanged by this
strict A/B and are exercised only by the separately labeled continuation lane.

The pinned author control nevertheless completes 2,400 substeps with 1,455
converged and 945 unconverged flags (632 caps and 313 plateaus). The split is
1,332/268 before release and 123/677 from release onward; the first false flag
is source step index 33 and the first cap is 35. Worst natural
`final_residual` is `2.59804445965485`, and worst per-step final checked
`r_coulomb` is `7.597910320688573`. The retained history is
`/tmp/fbf-sca-2026-author/paper_examples/card-house/results/20260721T175341Z/fbf/history.json`,
SHA-256 `b67d3c86f106171008dfbb0aca0a2ca72a9d3747c1a7a6694f57f211d3f83afd`.
Thus zero-cap completion remains the strict scientific gate, not a claim of
source-equivalent continuation. The separately labeled telemetry-rich
continuation lane is now implemented and captured; it does not change the
strict row's blocked status.
The boxed control timeline is
`/tmp/fbf_author_card_house_4_boxed100_20260721_contract_v2/timeline.json`,
SHA-256 `fdd3d9e96058176faa51b148d1bcf5a4c0a7f1c4e7da64e15490dcae4ce6fafc`.
The bounded existing-option matrix did not produce a strict solution: even a
50,000-iteration budget only moved the first failure to step 48, while the
other tested policy, seed, retry, regularization, and warm-start ablations
failed at or before the same prefix. A process-local GDB override accepting
capped iterates completed 2,400 steps and executed `p` at step 1,600, but its
runtime command cannot reproduce the mutation. It accepted 1,106 of 3,231
solves at cap and recorded worst residual `0.61608914241359314`. That unsealed
preview establishes finite continuation only and remains a historical
negative. A distinct reproducible continuation capture now completes both
lanes: exact has 3,351/3,351 attempts/solves, zero failures/fallbacks, 2,605
successes, 113 plateau accepts, 633 max-iteration accepts, and zero shrink
caps. The 746 accepts are 22.262% of 3,351 solves and occur across 723 steps;
worst residual `0.91712002943322535` is first reached at step 2,101.
Independent inspection finds both houses standing through release, but exact
and boxed are identical only at step 0; viewport difference is 0.165% at step
1,600 and 11.985% at the endpoint. Exact visibly retains more upright structure
after impact. The official MuJoCo panel degrades during settling, whereas DART
boxed remains upright until impact, so do not map DART lanes to paper lanes or infer a
mechanism. This qualitative difference is not solver superiority or
quantitative parity. Any next strict A/B must isolate
one remaining mismatch without changing tolerance, caps, fallback, or
fail-fast. The c95-bound colored-ordering and one-global-group probes are both
bounded rejects for the step-35 blocker: the latter still fails at step 35,
and its stock-partition sub-audit localizes the only nonconverged residual to
the 56-contact island with exactly zero off-block `W` coupling. The isolated
source-sized-gap probe likewise does not clear the 36-step gate: it fails at
step 31 on a 31-contact group with residual `1.0006073317077885e-5`. The
compared streams differ from step 1, and the stock sidecar is ancestor-bound
rather than a fresh c95 binary. This is a numeric strict-prefix diagnostic
with no visual verdict; keep the checked-in scene unchanged.

The separate c95 cadence-5 candidate still fails at step 35 on the 56-contact
group with residual `4.0845024466967225e-4`. Its copied stock comparator is
ancestor-bound, so the result proves only that this candidate does not clear
the strict prefix; it supplies no controlled delta or visual verdict. The
global-default diagnostic patch fails two legacy-default tests and remains out
of the main tree.

The same-binary terminal-estimate diagnostic changes only `rayleigh11` to
`last_norm10`. Its exact control reproduces the step-35 / attempt-101 /
56-contact gate; the single recorded variant passes all 103
ten-product/no-Rayleigh trace invariants but also fails there. Residuals first
diverge at step 29, and recorded contact-frame/reduced-state hashes plus
product-norm sequences first diverge at step 30. The reduced-state hash covers
only contact count, `freeVelocity`, and coefficients; `product_norms` stores
norms only. Those summaries do not cover `W`, operator identity, the initial
reaction, the complete reduced problem, or product vectors, so the final
residual/gamma deltas are contextual. It has no visual
verdict and does not establish source
estimator, trajectory, performance, superiority, Figure 6, or paper parity.
Verified package:
`/tmp/fbf_fig06_spectral_terminal_c95.OjNIB4/evidence/`; `RESULTS.md`
`e33894ab0b771544209d48724641716c491b04073ec5bec533c07df653e54cda`;
`SHA256SUMS`
`f18efba2ffb1f7f8ee0f88798c9bcd38103b571210949de5c0cc625fed3fd553`.

The sixth bounded diagnostic uses one c95 instrumented Release binary and
changes only the initial-vector selector from stock `ones64` to
`rs42_f32_values_dart_norm64`. Both arms retain `rayleigh11`, DART
`[n,t1,t2]` order, float64 normalization/products, ten configured products plus
the terminal Rayleigh product, and the frozen strict contract. The registered
variant promotes a raw NumPy-2.4.4 `RandomState(42)` float32 prefix to double
before DART normalization; its 168-value prefix SHA-256 is
`7506d5e093b6e3787fccb4c91aee3a26feffd8548637a9a76825ad1a9f3ccfe1`
and it aborts above that dimension. The control exactly reproduces the step-35
/ attempt-101 / 56-contact / 200-iteration gate with residual
`4.0844850280896461e-4`, 103 attempts, 102 solves, and one failure. The sole
variant also fails there with residual `4.1638905763175730e-4` and best
residual `4.1593800452634807e-4` at iteration 199.

Seed/product-norm/retained-estimate telemetry differs from attempt 1. Residual
and iteration count first differ at attempt 57 / step 29; contact-frame and
recorded reduced-state hashes first differ at attempt 67 / step 30. The
reduced-state hash omits `W` and the complete solver input, while
`product_norms` omits product vectors, so post-divergence deltas are
contextual. This numeric diagnostic has no visual verdict. It rejects only the
source-derived raw float32 values, promoted to double and normalized by DART's
unchanged float64 path, as sufficient for the frozen 36-step gate. It does not
establish source-estimator or coordinate-order parity, a root cause, a longer
trajectory, timing, performance, superiority, Figure 6, or paper parity. The
one-shot artifacts cannot externally prove that no invocation was discarded.
Verified package: `/tmp/fbf_fig06_source_seed_c95.Uemp3S/evidence/`;
`RESULTS.md`
`07b2f08f55bcb0210149e441c1886601d2a1f1d60d4f094b53f475ceaec88da3`;
`comparison.json`
`8897b3d826789baaba11ec9c1fea47569f108f82937b41978445f51aad028aeb`;
`SHA256SUMS`
`b2ecc0cf5c84a58448b8a1eafbb03ecda05e4f9935be193d3cd79ded87676a41`.

No strict release or
strict full-run trajectory, quantitative physical outcome, source-backend or
timing equivalence, approved golden, Fig. 6 or paper parity, or superiority
follows. See
[`FIGURE6_CONVERGENCE_BLOCKER.md`](FIGURE6_CONVERGENCE_BLOCKER.md). This is an
adapter plus separately labeled continuation-evidence lane.

The two-card A-frame and five-level author-card construction remain useful
inspection/diagnostic scenes. The older reconstructed 26-card lane remains
useful negative evidence. None replaces or can be relabeled as the new
source-selected row.

## Supplemental Non-Paper Source-Default Five-Level Card House

The visual workflow now declares 33 runnable schedules, 30 of which encode
MP4, including distinct strict
`card_house_author_5_impact_current_source` and continuation
`card_house_author_5_impact_source_continuation_current_source` schedules. They
bind the current public no-argument five-level default: 40 cards, four
initially kinematic cubes, 800 display frames / 3,200 DART substeps, and
release after completed step 1,600. Both use `dt=1/240 s`, Native
`FourPointPlanar`, 4,096-contact capacity, four contacts per pair, one `0.1 m`
ground gap, and 44 `0.005 m` dynamic-shape gaps. This is supplemental current
source-default evidence, not a paper row, the source-selected four-level
Figure 6 scene, a historical Tables 6-7 invocation, or a recovered paper
command. The added ten-level colored-BGS schedule is a DART-only numeric
diagnostic and is not a required paper/video row.

Strict exact fails closed after completed step 31, before release, on a
39-contact group at 200 iterations. Its residual is
`9.022404720646783e-6` against the `1e-6` tolerance; 248 attempts produce 247
solves and one failure, with zero accepted caps and zero boxed fallbacks.

The separate continuation exact member and boxed member each complete
3,200/3,200 steps, capture 401 shots, execute the step-1,600 release
successfully, and produce fully decoded 660x506 H.264/yuv420p clips at 30 fps
for 13.366667 s. Exact has 399 unique images because two settled shot pairs
are duplicates; boxed has 401. Exact records 7,337/7,337 attempts/solves, zero
failures/fallbacks, 2,245 plateau accepts, 836 max-iteration accepts, zero
shrink-cap accepts, 7,298 warm starts, and 303,900 iterations. Its final/worst
residuals are `1.2757511844995566e-7` / `0.6378480998790657`, with 266 maximum
and 248 final contacts. The historical v3 exact and boxed reuse audits pass;
their summary SHA-256 values are
`1b7c42c0836aa17fd55f952e6335167a5786d37e54236f9088fa5d1a6a1885fd`
and `078212b68ae07e234c94b2537d41158cd5c944492e930de7e1fc2b329f5a6453`.
The boxed audit measures all-frame right/bottom margins of at least 40/30 px
and endpoint margins of about 41/33 px.

Exact timeline/clip/metadata SHA-256 values are
`35c7fddedc2dbdb6f2b00323f19dcc6df98ac1e4188d246f06ef562ad44aea80`,
`956ca7c32fcc23501a863d0e7ec2668fe7ffe5986343a5dcaf49ba6886be8816`,
and `3afe8c7a3bc795827ff4318438b150d176699dc9dd55057bb48dd025684ffbdf`.
Boxed values are
`ba771601affe997b07a66ddc161f4a110ea4eced0b4d94773b219270b63a322f`,
`319747d1a24a8a735ab4b4485b44a39905e3f6d40fe00257f78ba9b9451fcaa7`,
and `f77af90deac3f740d06ba0bec2eca17d837cf83974b253c35f3b76dbca35113e`.

The historical ignored labeled candidate is
`assets/pr_media_card5_source_default_group_v3/card_house_author_5_impact_source_continuation_current_source__exact_plus_continuation_vs_boxed_no_continuation/clip.mp4`.
Its labels are `EXACT COULOMB FBF + SOURCE CONTINUATION` and
`EXISTING BOXED LCP (NO SOURCE CONTINUATION)`, so the policy asymmetry is
explicit. The 1320x530 H.264/yuv420p clip has 401 frames at 30 fps over
13.366667 s and passes full decode. Clip, panel, and presentation-manifest
SHA-256 values are
`b46aeb3d9f09e95151e26fef4838432b6b071a5d3c39c3c9a489c6f1d42e875b`,
`484fdc35aed15ba06e253be63e5ff9bb46f88bc6c273150d8f78a646da0dc7f8`,
and `2c80a8cca4cb3a0a11f49a1747bb5cc90092f884e05f0b30d3959e8e2a3eb3cf`.
Manual inspection records visible pre-release and endpoint differences, while
`automated_semantic_outcome_validated` remains false.

This clip is not strict success, a solver-only A/B, solver superiority, an
automated semantic or physical outcome, source trajectory/backend/timing
evidence, historical Tables 6-7 evidence, paper-video parity, or paper parity.
The v1 and v2 captures are superseded framing probes and excluded. The v3
assets remain ignored historical evidence. The current-head v2 group listed
in the attachment table is byte-identical and is now the upload source; its
browser-upload URL is pending.

## Current Local Attachment Candidates

The retained five minimum Figures 1-5 source-row upload root is the ignored
current-head v2 reseal `assets/pr_media_current_head_67073/`. Its compact
Figures 1-2/4-5 run and independent verification pass with 10 members, 6
groups, zero failures, and five expected boxed skips for exact-only
author-turntable schedules/group. Run/verify summary SHA-256 values are
`8a8b83bba2d19ea9a1a03d90daa80554e909507bad9f95ccc4bc23c3ac36d0c7`
and `7599ecf8fc5ceb5338cf97633e710da3985981557b1babd61e52a5a653dec22f`.
All ten members use schema v2, bind demo SHA-256
`69879e77e55099f67c11530dfdde9dd2d4e1e4ac0d3167a1bb8b0b3945559efa`,
and have exact live/sidecar contract matches. The Figure 3 capture in the same
root independently passes as exact, boxed, and synchronized checker group.
The five minimum clips and Figure 3 supplement are byte-identical to staging.
Manual panel inspection finds no cropping or gross rendering failure and shows
the intended seven-lane incline, author-turntable 2x2, and Painleve outcomes.
This qualitative audit validates no source-backend equivalence, paper parity,
timing equivalence, or solver superiority.

The older c95-bound `assets/pr_media_current_head_c95_small_rows/` reseal is
superseded for minimum source-row uploads. Retain only its four reconstructed
Figure 4 exact-vs-boxed proxy comparisons and historical diagnostics. Its 20
members / 13 groups and independent reuse verification remain internally
valid under their recorded c95 binary and all record false paper/automated
semantic claims. Every retained member and comparison MP4 below is
H.264/yuv420p at 30 fps and passes a full decode. The direct comparison labels
are intentionally limited to
`EXACT COULOMB FBF` and `EXISTING BOXED LCP`; they identify lanes without
claiming that a visible outcome is numerically or paper-valid.

### Browser-upload manifest

An independent handoff audit verified the following 16 recommended files.
Ignored `assets/pr_upload_3377/` now contains the current-head Figure 6 and
Figure 7 groups in staging slots 06 and 07 with SHA-256 values
`7e87661d4b3bf34cdae192af069f159ead8407b4408b42cc6d15788ab6683b9e`
and `9a9a68229faeb1958cb8ed0449387ca310b0f41e692434ed58bcd39988792177`.
Slot 15 remains byte-identical to the current-head five-level group, and the
updated `SHA256SUMS` passes `sha256sum -c` for all 16 H.264/yuv420p MP4 staging
copies. The ten-level recapture and Figure 8 standalone finalization remain
pending, so their audited entries stay explicitly historical. None of the
files is tracked. The first nine clips are the minimum one-per-source-row set;
the last seven retain promised direct comparisons that the minimum set would
hide. Every browser-composer upload and resulting GitHub user-attachment URL
remains pending. Upload order does not change any row's scientific status.

| Tier | Row | Local file under `docs/dev_tasks/fbf_exact_coulomb_friction/` | SHA-256 |
| --- | --- | --- | --- |
| Minimum | Figs. 1-2 seven-cell current-source incline exact vs boxed | `assets/pr_media_current_head_67073/groups/incline_author_sweep_current_source__exact_vs_boxed/clip.mp4` | `a750350c7f210953bf3292f79faef2bdacb160c9652676a9f98695165357f723` |
| Minimum | Fig. 3 source-pinned checker backspin exact | `assets/pr_media_current_head_67073/backspin_author_current_source/clip.mp4` | `b2c268aa337f8d4e753408c1bbf17ca29dc4300597b64782fcb7344f6c676b30` |
| Minimum | Fig. 4 author-pinned exact 2x2 | `assets/pr_media_current_head_67073/groups/turntable_author/clip.mp4` | `b241463658e6b48dfb2c74815e85317c2f0eccd46a7f7ab978b8f2701ce80d6d` |
| Minimum | Fig. 5 `mu=.5` exact vs boxed | `assets/pr_media_current_head_67073/groups/painleve_author_mu05__exact_vs_boxed/clip.mp4` | `77d3286dde96785a6c36cd901e92f183409098ba2bd8dbb426489f537fe71209` |
| Minimum | Fig. 5 `mu=.55` exact vs boxed | `assets/pr_media_current_head_67073/groups/painleve_author_mu055__exact_vs_boxed/clip.mp4` | `2c71e565559dea513870b56bba3c709cf015707b171cfdb45b5cf64fde31f70f` |
| Minimum | Fig. 6 current-head source continuation exact vs boxed | `assets/pr_media_current_head_67073/groups/card_house_author_4_impact_source_continuation_current_source__exact_vs_boxed/clip.mp4` | `7e87661d4b3bf34cdae192af069f159ead8407b4408b42cc6d15788ab6683b9e` |
| Minimum | Fig. 7 current-head crown continuation exact vs boxed | `assets/pr_media_current_head_67073/groups/masonry_arch_25_author_crown_impact_source_continuation_current_source__exact_vs_boxed/clip.mp4` | `9a9a68229faeb1958cb8ed0449387ca310b0f41e692434ed58bcd39988792177` |
| Minimum | Fig. 8 historical v1 frozen-prefix diagnostic; current-head finalization pending | `assets/paper_evidence/fig08_arch101_author_current_v1/groups/fig08_arch101_strict_exact_vs_boxed_diagnostic/clip.mp4` | `d6f5f658e4fb027edb23e0911acd34b74dfd749daace41b5d9c9204af3163b94` |
| Minimum | Tables 6-7 historical c95 qualitative continuation-policy pair; current-head recapture active | `assets/pr_media_current_head_c95_card10_same_binary_exact_v2/groups/card_house_author_10_impact_source_continuation_current_source__exact_vs_boxed_same_binary_qualitative/clip.mp4` | `d09d8a4b6c962eef84620f5fc4aebd709c8631f4c274a302217c56e9163547b2` |
| Supplemental | Fig. 4 proxy `mu=.2, omega=2` exact vs boxed | `assets/pr_media_current_head_c95_small_rows/groups/turntable_mu02_omega2__exact_vs_boxed/clip.mp4` | `56f2a7ee598dffd6716cdf4a7b6b8560e7587ec9bc871f74d0d6e28c4d00daf3` |
| Supplemental | Fig. 4 proxy `mu=.2, omega=5` exact vs boxed | `assets/pr_media_current_head_c95_small_rows/groups/turntable_mu02_omega5__exact_vs_boxed/clip.mp4` | `2cab4896f79a116949e360871ac319b6193849fd04ad7bf1bfd43cdec16814e3` |
| Supplemental | Fig. 4 proxy `mu=.5, omega=2` exact vs boxed | `assets/pr_media_current_head_c95_small_rows/groups/turntable_mu05_omega2__exact_vs_boxed/clip.mp4` | `5d16450c622dd21ae1144ce66df3c04419f0bf88448fe3ccd32f90f170fc6b62` |
| Supplemental | Fig. 4 proxy `mu=.5, omega=5` exact vs boxed | `assets/pr_media_current_head_c95_small_rows/groups/turntable_mu05_omega5__exact_vs_boxed/clip.mp4` | `83bd9494cca0968fd18b23a68d02c9bc19d66e4b7559cb3f893b6b3891229ad8` |
| Supplemental | Fig. 7 literal-standing baseline exact vs boxed | `assets/pr_media_current_head_fig07/groups/masonry_arch_25_literal_standing__exact_vs_boxed/clip.mp4` | `89c4d7372f68c6c9ad1a5d0e0e0388ffa1f198c2446e04fe30b9bc66325d8f9e` |
| Supplemental | Non-paper five-level current-head source-default exact-with-continuation vs boxed-without-continuation | `assets/pr_media_current_head_67073/groups/card_house_author_5_impact_source_continuation_current_source__exact_vs_boxed/clip.mp4` | `b46aeb3d9f09e95151e26fef4838432b6b071a5d3c39c3c9a489c6f1d42e875b` |
| Supplemental | Fig. 3 source-pinned checker backspin exact vs boxed | `assets/pr_media_current_head_67073/groups/backspin_author_current_source__exact_vs_boxed/clip.mp4` | `e321c711eae7daf8e2a289df71f4d08c0d813d6c84e204c0930594d4a561e15b` |

The mandatory caption boundaries remain: Figure 3's checker is a visual-only
rotation cue and its 30 fps sampling does not establish signed angular
direction or rate; Figure 4's author group is
exact-only; Figure 6/7/Tables 6-7 continuation clips are non-strict; Figure 8
freezes exact after its valid prefix; and the Tables 6-7 pair includes a
continuation-policy difference. The supplemental five-level pair is non-paper
source-default evidence and also compares asymmetric policies. The four Figure 4 proxy pairs are reconstructed DART comparisons, not
the author-pinned scene. No GitHub user-attachment URL is recorded, so the
manifest remains a browser-composer handoff rather than published PR media.

### Figure 6: source-continuation card house

The current-head v2 Figure 6 capture and verification identities are listed in
the table above and supersede the older bundle below as the upload source.
The local exact/boxed pair uses the same 26-card/four-cube scene, 2,400-step
clock, and successful step-1,600 `p` release. It is a continuation-policy review
artifact and does not satisfy the strict zero-accept rule above. The 301-frame
paired H.264/yuv420p clip is 1320x530, 30 fps, 10.033333 s, and passes full
decode. The summary records `paper_comparable=false` and no automated
semantic-outcome validation.

- Historical ignored durable bundle:
  `assets/paper_evidence/fig06_card_house_source_continuation_current_v1/`
  (resealed from
  `/tmp/fbf_fig6_source_continuation_pair_20260721T1414_v2/`).
- Historical direct comparison:
  `groups/card_house_author_4_impact_source_continuation_current_source__exact_vs_boxed/clip.mp4`,
  SHA-256
  `282aebfb9e2e38fe3741db28e2ce909fb548d7aa46d048302a3b0e0bea9e1786`.
- Run-summary SHA-256:
  `6888f4729c99d41753c9c8ec9a1ec2ec9e2367c71da76aab973f8f8c5e8674cc`.
- Exact timeline SHA-256:
  `a9eb12711419b7801037d17059560559893be2898e07d14425a5f572175482ff`;
  boxed timeline SHA-256:
  `1618e284f97ff7ed49e3288636269f5bea6131faa3bae45428e42e23de660bd8`.

The freshly downloaded official video has SHA-256
`d5356e1b31487be62b75af05efbfecdb70ad5d98501a8efd378fcedf066e4794`,
exactly matching the audited hash. The official segment is contextual
presentation evidence only: the DART lanes do not map to the paper lanes, and
no frame golden or trajectory comparison exists. The candidate remains outside
Git until uploaded through the PR editor.

### Tables 6-7: ten-level same-binary continuation-policy presentation pair

The current-head ten-level recapture is active under
`assets/pr_media_current_head_67073/`, but no final current-head capture or
independent-verification result is recorded yet. The c95-bound evidence below
therefore remains the historical local attachment identity. Its exact member
is locally resealed and independently reverified. The
separately named continuation schedule passes all 3,200 steps and the
step-1,600 release with 7,702/7,702 exact solves, zero
failures/fallbacks, 2,427 plateau accepts, 763 max-iteration accepts, zero
shrink caps, 310,880 total iterations, 7,630 warm starts, 1,071 maximum/987
final contacts, final residual `7.709159985211234e-8`, and worst residual
`0.59964511064890469`.

Timeline validation passes with 3,201 represented states and 401
captured/unique frames. The 660x506 H.264/yuv420p clip has 401 frames at 30 fps
over 13.366667 s, and full decode passes. Final panel/keyframe inspection
confirms only legibility, release, visible post-release evolution, and lower
structure remaining at the endpoint. Metadata keeps `paper_comparable=false` and
`automated_semantic_outcome_validated=false`.

- Ignored exact root:
  `assets/pr_media_current_head_c95_card10_same_binary_exact_v2/card_house_author_10_impact_source_continuation_current_source/`.
- Timeline SHA-256:
  `edddf5bab098f655f6fa6a0adf50bc236474f987fa99f630a1b18d15d6d232ce`.
- Clip SHA-256:
  `19637c4255c890f1f32383e7e7e680169688e5d8b071168bc6b4ffdebf33061d`.
- Panel SHA-256:
  `e5ed0d63ca9818292c5a373f476f2841f280f3e01492e0065b2aec8eb95a74d6`.
- Metadata SHA-256:
  `23fe61063c024d3e93466395798951b4942755ef6bd0c4b3650f5ee00c48c84d`.

The separate run summary `/tmp/card10_same_binary_exact_c95_v2_summary.json`
has SHA-256
`ebf02723ab30875204bed78ebcffe1ef53bebfee8d25e84c5e5649aeb4b0ebf1`
and reports `pass=true`. Independent reuse verification passes; its separate
`/tmp/card10_same_binary_exact_c95_v2_verify.json` summary has SHA-256
`6701bcdea5664d095380e7fa5870972965dec76fdf1595d2e3ca3d8038463055`,
kind `verification`, one result, no skips or groups, full-decode success, and
the matching metadata hash. The exact schedule's blockers are empty only
within the narrow continuation boundary.

A clean boxed control completes 80/80 in about 4 minutes 46 seconds with
`BoxedLcpConstraintSolver`; its timeline SHA-256 is
`ccbdc322791a06d5a8858818acae63e8540ca7770e635545e3c017d84bf96d7d`.
It is not a full outcome. The subsequent full boxed member completes
3,200/3,200 and passes capture plus independent reuse verification against the
same `dart-demos` SHA-256 `5725672a...` as exact. Its decoded 401-frame clip has
SHA-256 `c3bf391fafa0913e53ce857c497e6411a2810d71f8201a5cffb56e4dd6eb2f20`;
timeline, panel, and metadata hashes are
`7d1d272913f4bb72bb0f98bff3d8417668ed86d2522fe913ca3f0bbfca658b43`,
`918eec24dbb1c30876a6d6f4a38fbb209100fe0e2fc7728d8518d233ac19db76`,
and `54414a7ab170569a1645bfaace87ea08b8d7f0fb5ce1ae51b9df87da75c19aae`.

The synchronized labeled comparison is H.264/yuv420p, 1320x530, 401 frames,
30 fps, 13.366667 s, and fully decodes. Its clip/panel/manifest SHA-256 values
are `d09d8a4b6c962eef84620f5fc4aebd709c8631f4c274a302217c56e9163547b2`,
`848805bece727c73e35e51261edd9a02a655cefdb2facd75affdd4667b972794`,
and `800d03fcf8ca5c461b9ce18bbef0ea948a30864fa2bdb739774cf20ca0b333dc`.
The manifest records `presentation_only=true`, `same_demo_binary=true`, and
`runner_group_contract_validated=false`. Manual inspection finds retained
upright multi-level structure in exact and a largely collapsed boxed endpoint,
but exact requests source continuation while boxed does not. This is
qualitative continuation-policy presentation evidence—not a solver-only A/B,
automated physical outcome, strict convergence, performance/superiority,
source/trajectory/Tables 6-7 parity, or paper parity. The labeled pair must be
uploaded manually through the PR browser composer; no GitHub user-attachment
URL exists.

### Figure 7: c95-bound literal 25-stone standing pair

The ignored c95-bound root is `assets/pr_media_current_head_fig07/`.
Capture and independent reuse verification pass for two 600-step members and
their synchronized group. The external summary hashes are
`5a1de1f915d75c373f06aeb48978b92a540bc245427c55584f68ad178ea491bb` and
`e4b3d44d5f2afebef9f79bcb92b38ee282f5635c38fa4be2f4264a5f961acce5`.

- Exact timeline/clip/panel/metadata:
  `6041addd27a79a747cdbcdaafb495f787d4e90a906ec0616aa16e5a33d9c9b74`,
  `24c110421572500bec9f43a431061ae5a386e7e59940e86030e3059cc90d9676`,
  `cd3498c90fd549365dadc9cf96908c4c3d86da81cbd5dd64ff4d891407b4ee6b`,
  `614704cc1ed70065d81b789c019e618ac54d7013f706b26a346ea236ef876802`.
- Boxed timeline/clip/panel/metadata:
  `809ca91a475fdd0ebe3ad6b5cba73115c9ec2b3dd4a98e478beca229abf62321`,
  `80e79fa6b356f951e9615dd94aad2de2f55d0bbab07d7b116cb07c1b3bef686c`,
  `078990e4d7a950ba9e207102624d651d948f546f1153bf85077e8084c01b040a`,
  `944a6636bd6febb79ab9d6abda0a92165acd34ab3aa96b42f55f1c36731e6d45`.
- Group clip/panel/metadata:
  `89c4d7372f68c6c9ad1a5d0e0e0388ffa1f198c2446e04fe30b9bc66325d8f9e`,
  `5ce6efcebcb5f6a2385f6aea6de7933cccfa7446979b7ea0e46ef2aab5199633`,
  `5cc2513eb16db191454b27126783df70adbe3ad2617d3faa3f51597ab43966bb`.

Exact records 600/600 solves, 96 final contacts, zero accepted
caps/failures/fallbacks, final/worst residual
`9.778093504499096e-7` / `9.999807145410957e-7`, 599 warm starts, and 7,933
iterations. Boxed completes 600/600 with `BoxedLcpConstraintSolver`. The
1320x530 grouped H.264/yuv420p clip has 301 frames at 30 fps over 10.033333 s
and fully decodes. Manual endpoint/group inspection finds both visibly
standing and the presentation legible. All three metadata records set
`automated_semantic_outcome_validated=false`, so that observation is not an
automated physical oracle, trajectory/outcome comparison, solver-superiority
result, crown-impact result, or Fig. 7 parity. The separate crown-impact
adapter remains blocked at strict step 142. Upload the group clip only through
the PR browser composer and record the resulting URL; do not commit the bundle.

### Figure 7: c95-bound crown-impact continuation pair

Checkpoint `34d9b66e97c` adds a separately named bounded continuation schedule.
The current-head v2 capture and verification identities listed above supersede
the following c95-bound root as the upload source. The historical ignored root
is `assets/pr_media_current_head_fig07_crown_continuation/`.
Paired capture and independent reuse verification pass; their SHA-256 values
are `f0e45526d648d7c8d6052c3a4f32ec47a29033e4ed687b89fecc52c1ce04396f`
and `969ef6143185716e8441704829d4643a1d804fc5580288dae28fe47257fce0f3`,
with two verified results and one group.

Both lanes complete 2,000/2,000 and release the cubes successfully at step
1,600. Exact records 2,122/2,122 attempts/solves, zero failures/fallbacks,
1,940 plateau accepts, 98 max-iteration accepts, and final residual
`0.004493046465992133`. The group metadata/panel/clip hashes are
`4229307f7d6d91f4b347fecc53db9f290061c6dc76482e684a94064f764601d7`,
`f3bdb5a20ad57ee20e1a2cf6508a701f9bbd79bf0532ffc303d87989b4dfa802`,
and `c4ffe2488520a5c22608c9117443cf9ff5de5396f4353d4bced5d1afff6bf0c8`.

Manual inspection finds both arches standing and cubes reaching the crown,
with nearly identical visible outcomes. Records set `paper_comparable=false`
and `automated_semantic_outcome_validated=false`. This is bounded non-strict
continuation evidence, not strict convergence, solver superiority, physical
outcome, source/paper trajectory or Figure 7 parity, timing, or backend
evidence. The strict lane remains blocked at step 142. Upload the group clip
through the GitHub browser composer, record its URL, and keep the bundle out
of Git.

### Figure 8: source-pinned 101-stone standing blocker

Checkpoint `8ad9961e56c` adds a standalone finalizer with distinct
`finalize`, `seal`, and reuse-only `verify` phases for this expected-failure
comparison. It requires the strict failure after completed step 209, binds the
last rendered exact frame at step 208, labels and freezes only the remaining
exact display interval, and compares it with the complete boxed capture. No
current-head bundle has completed all three phases, so the packet below remains
historical v1 evidence rather than a current-head sealed result.

The ignored packet is
`assets/paper_evidence/fig08_arch101_author_current_v1/`. It binds the public
`--stones 101` selection, all 101 author meshes, 99 mobile stones, two fixed
springers, three pinned cubes, and the source-supported 400-frame /
1,600-substep schedule. `drop_frame=400` is the endpoint, so no release occurs.

Strict exact fails closed after 209 completed steps on an iteration cap with
208 contacts, 5,000 iterations, residual `1.2582804496066107e-6`, 342/342
attempts/solves, one accepted cap, zero exact failures, and zero boxed
fallbacks. Its timeline SHA-256 is
`df1ed4afc9ef5aa74f7c0b6da0560ae0d1b63fca28f45051ed27c5dfb3632889`.
Boxed completes all 1,600 steps with a valid inventory/finite/cube-pinning
trace but fails standing; maximum mobile-body displacement is
`21.2188459736`, maximum rotation is `3.14152663339 rad`, and its timeline
SHA-256 is
`a8caee71c9356a72fa65210207d7b4209d9e305363974ec07c81f19ec14bfa1e`.

- Boxed-collapse candidate:
  `masonry_arch_101_author_standing_current_source__boxed/clip.mp4`, 660x506,
  201 frames, H.264/yuv420p, 30 fps, 6.7 s, full decode, SHA-256
  `7635c2722b20fb8bcb0255054cc9172153d1dd640fd8e81df4df52c0e515d3c0`.
- Diagnostic frozen-prefix hstack:
  `groups/fig08_arch101_strict_exact_vs_boxed_diagnostic/clip.mp4`,
  1320x506, 201 frames, 30 fps, 6.7 s, full decode, SHA-256
  `d6f5f658e4fb027edb23e0911acd34b74dfd749daace41b5d9c9204af3163b94`.
- Compact summary SHA-256:
  `1c19c6c3c36171a5e85f330b2863b429956652fb894aae0aa0b82d68291e3481`.

Capture and independent boxed reuse verification pass. Manual inspection finds
the arch intact at steps 0 and 400, loss of the crown-standing configuration by
step 800, and visible collapse by steps 1,200 and 1,600. The hstack freezes
exact at its last rendered step-208 frame while boxed continues and explicitly
claims no superiority or parity. The official 74-80 s segment is contextual
source-video evidence only; no historical source/Kamino golden, source
camera/golden, or complete exact trajectory exists. Both candidates remain outside Git until a
maintainer uploads a narrowly captioned blocker clip through the PR editor.

The independent pinned public-source FBF control completes all 1,600 substeps
and exits zero, but only 127 steps converge; 1,473 (`92.0625%`) exhaust
`max_outer=200` and continue. The saved keystone falls
`7.2349853515625` raw units and 57/99 mobile stones exceed the local three-unit
height-change limit. Its compact audit SHA-256 is
`56844eee3d908a1078fe7e76c6a92e31f2de79eb6e1e503ca7173d4e078c6cd4`.
The source persists no final x/y, rotations, or media, so this proves a finite
current-source standing-criterion negative, not visual collapse, a converged
golden, historical Figure 8 parity, or solver superiority.

The separate official current-source Kamino command also exits zero and saves
all 400 frames / 1,600 substeps with finite arrays, but it fails the same local
vertical-height criterion for more stones and with a larger maximum change on
that metric: 98/99 mobile stones change height by more than three raw units,
maximum change is `87.23839569091797`, and the
keystone drops `82.03050136566162`. The result and trajectory SHA-256 values
are `86b351c212c0e69df371fa66c67c53d6c3421575afbafae3a6b8d339f574dfb3`
and `63c47426019a218942afe3edae31cdf5dbbbd8d8926732ea59120e23bb6cf1a4`.
The runner preserves no convergence/contact history, full poses, rotations,
cube trajectory, or media. This is another full-horizon current-source numeric
negative, not a matched historical Figure 8 oracle or an attachment candidate.

### Figures 1-2: seven-cell current-source incline sweep

The source-bound exact and boxed captures each contain a complete 120-step
trace and 61 distinct H.264/yuv420p frames. The synchronized group is
2600x890, 30 fps, 2.033333 s, passes full decode and independent reuse
verification, and has SHA-256
`a750350c7f210953bf3292f79faef2bdacb160c9652676a9f98695165357f723`.
Its local ignored path is
`assets/pr_media_author_incline_final_candidate_v6/groups/incline_author_sweep_current_source__exact_vs_boxed/clip.mp4`.
The group panel and metadata hashes are
`1c49eddac9fe2959ea4475e29f68ced2e5ce779ae7261b03f5fb3b8f58a04e42`
and `1e5bc5293d5c83a72075a2884c6fdf7ef17e5b8795b55da8028c9bc7f2602505`.
The copied demo binary, capture summary, and independent verification summary
hashes are respectively
`67d399eee85ffd286984a877b8f4181b9ce3030acf5f9b2bc03886e54e7a5f20`,
`243ba16ef500fc8d3bb71e1b264e0bd2e99dbd23257a3bb9cfa809a4fbeaacba`, and
`02305f4faeeb792198dc7e85cf4348ffa3f3d52a742fa4e63f8b01e52bd27b4c`.
The group metadata asserts the narrow current-source FBF terminal/outcome-slice
validation flag and deliberately leaves generic automated semantic validation
false.

All seven DART cells remain supported, upright, in-lane, and
contact-participating. Both solvers classify `.3/.4/.45` as slide and
`.5/.55/.6/.8` as stick. Exact maximum absolute displacement/velocity deltas
from the retained source-FBF terminal projection are
`0.002426469449185232 m` / `0.0011201728594518558 m/s`; boxed deltas are
`0.0011521317667995284 m` / `0.00030012480388411203 m/s`. The projection binds
raw results, canonical projection, and `mu=.55` history SHA-256 values
`f5cc26d2b0ca542b2b98f7fe94a8e2f7f7c9b7cccb3d23c35234ebe45d0d9d12`,
`e8b3b5c93a543480bae5c2f50106ecc1b137f65337cc1e725ef8c840efdb8921`,
and `c0aa2d65cbbee24447e7ece9aa97bf83da4cc666ccf16da7edd6874abc22422f`.
The retained source records only 839/840 configured convergence flags because
`mu=.55`, step 1 reaches 200/200. Therefore the comparison supports only a
current-source terminal/outcome slice, not strict source convergence,
trajectory/backend/solver/full-state or physical equivalence, historical
video/golden, timing, paper parity, or superiority.

The older two-cell `incline` pair remains a regression fixture; it is no
longer the recommended Figures 1-2 browser upload.

### Figure 3: backspin reversal

The preferred Figure 3 candidate is now the separately named source-pinned
`fbf_author_backspin_current_source` scene and
`backspin_author_current_source` schedule. They bind the current public
four-second configuration at author commit
`b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0` and an independently sealed
240-step numeric reference. The source's configured terminal Coulomb flag is
true for all 240 steps, but 183 projected natural residuals exceed `1e-6`, so
that fact is not reported as strict natural-residual convergence.

Exact and boxed DART lanes each complete 240 steps and emit 241-state traces.
Both first become supported at step 2, remain supported contiguously through
step 206, satisfy the five-sample rolling-tail gate, leave the finite slab at
its left edge, finish airborne beyond the slab, preserve zero measured
off-axis motion, and remain within the preregistered source-terminal
tolerances. Exact finishes at `vx=-11.4796920141 m/s`,
`wy=-46.2074988932 rad/s`, slip `0.0721827092 m/s`, and
`z=-1.3729371427 m`; boxed finishes at `vx=-11.4821219179 m/s`,
`wy=-46.2486366906 rad/s`, slip `0.0800372547 m/s`, and
`z=-1.3731065400 m`. The source terminal projection is
`vx=-11.4285717010 m/s`, `wy=-45.7142868042 rad/s`, zero slip, and
`z=-1.3714094162 m`. Exact records 205/205 solves, zero accepted caps,
failures, or boxed fallbacks, and worst residual
`9.990141261260073e-7`; the boxed lane exposes no exact-FBF diagnostics.
The current-head capture summary and independent reuse-verification summary
pass for two members and one group at SHA-256
`e2709dcc8aedb7c8deb52d4fd9e5ccf881cf64b92490b85d6760bffb1ae929c7` and
`f82e74fb762906a525ee442f878c489aa833820fa910e3c137f54341b94242fc`.
The demo binary SHA-256 is
`69879e77e55099f67c11530dfdde9dd2d4e1e4ac0d3167a1bb8b0b3945559efa`;
the bound configuration spec / demo implementation SHA-256 values are
`bd8ff3660f05d63fb4be699389166f04c8887e9d6285bf0e68b851fb4126bd64`
and `e2cc2351a3043ec8301677d990adfe2c7da2b9762499b19bd81c5e3be1559337`.
Exact v2 metadata binds contract/semantic SHA-256 values
`fd64d02e50d90c1a3571bf575c97c379718c177a13a5141894d902aff2123252`
and `e8589021edba1504d84046b2a975c0f38598dec54195dcac93a95393bb9f627d`;
boxed v2 metadata binds
`c38242d5949204e06c0fb8ded191496127c3e0510e40308e036d40bb2593a58e`
and `55b4244df4e0d416d7c9a5f2a87bead887a900bf177c0956e46efeae2c5ace40`.
Both record `sidecar_contract_match=true`.

The renderer attaches a high-contrast ivory/charcoal checker mesh as
`VisualAspect` only; the physical sphere remains the collision, inertia,
friction, and dynamics geometry. Capture metadata binds the OBJ, MTL, and PPM
before and after capture at SHA-256
`1a10b78223e0af8b786452458f5d62b904afc398f4630003830742092dac014e`,
`a8cd05a3cf64c8909bafc2ab36ac8d0ef9f5e59538e96eefe77389ddcdf9f004`,
and `324819cc828864dc0e5459593b0801f48e1754f49d7a096c8e6469531a62a3fb`.
Manual inspection confirms visible facet-orientation
changes in consecutive frames and across the temporal panel. That observation
is only a rotation-visibility check: 30 fps sampling aliases the initial
`-200 rad/s` rate, so signed direction and rate remain trace-backed rather
than video-backed.

- Preferred exact attachment candidate:
  `assets/pr_media_current_head_67073/backspin_author_current_source/clip.mp4`,
  SHA-256
  `b2c268aa337f8d4e753408c1bbf17ca29dc4300597b64782fcb7344f6c676b30`.
- Boxed member candidate:
  `assets/pr_media_current_head_67073/backspin_author_current_source__boxed/clip.mp4`,
  SHA-256
  `dc3228e2aa8cd18798807325ea6a3bc13dbb79cd3564a3a95b520f0bd56ddd7f`.
- Supplemental labeled comparison candidate:
  `assets/pr_media_current_head_67073/groups/backspin_author_current_source__exact_vs_boxed/clip.mp4`,
  SHA-256
  `e321c711eae7daf8e2a289df71f4d08c0d813d6c84e204c0930594d4a561e15b`.

All three clips have 121 H.264/yuv420p frames at 30 fps and pass full decode.
The 1300x506 exact member is preferred because the texture is larger than in
the 2600x530 comparison. Both solvers pass the bounded outcome, so this is not
solver superiority or equivalence, source backend/full-trajectory or video
equivalence, timing/real-time evidence, a historical Figure 3 invocation, or
paper parity. The older c95 `backspin` bundle remains historical reconstructed
DART regression evidence and must not replace this upload candidate.

### Figure 4: turntable parameter matrix

The source-configuration-pinned group was resealed at current implementation
checkpoint `67073f4f575` under result schema v2 with the demo and semantic
physics identities recorded above.
Each cell has 181 distinct frames. In source order, the exact solver reports
139, 120, 348, and 86 successful attempts; every cell has zero accepted caps,
failures, or boxed fallbacks, and the largest trajectory residual is
`9.999742483197972e-7`. Independent replay verification passes.

Manual inspection at the six-second horizon shows the expected source-order
classification: `mu=.2, omega=2` ejected; `mu=.2, omega=5` ejected;
`mu=.5, omega=2` retained on the support; and `mu=.5, omega=5` ejected.
Retention through six seconds does not establish zero slip, perfect
co-rotation, or longer-horizon stability.

- Author-pinned exact 2x2 candidate:
  `assets/pr_media_current_head_67073/groups/turntable_author/clip.mp4`, SHA-256
  `b241463658e6b48dfb2c74815e85317c2f0eccd46a7f7ab978b8f2701ce80d6d`.
- Author member SHA-256 values in source order:
  `ea3d8d01c843bfcf0451b2a0aec8a523f80a6bdd5e8327bdafc7729c7cd8a0ee`,
  `7fe6dee15ac32b41b3807fd06752393fe29a2fe08a3a34f2702be0675593e7a0`,
  `fb816fda0e43503d27eb9cd88ea0596907a90878bc006d3e298850ad0e8c5440`,
  and
  `7128bdb42de60ac2e7fa0e8c99439cbf0204783e76c8cef3192cbf3a10ec7aa8`.

The author group is 1320x1060 for 181 frames (6.033333 s). Its geometry,
mass, placement, friction, and speed ramp are pinned to the public source, but
it is still a DART float64/Native implementation with DART's solver,
frontend, camera, materials, and renderer. It is not Warp/Newton trace,
approved-golden, timing, or paper-parity evidence.

The separate paired proxy capture uses rebuilt demo binary SHA-256
`a6fac1c2d2d6ca809f723b46ead080ab22065b0bb41830ea3dc495237c8f83f7`
and the same runner SHA above. A focused source fix makes the turntable solver
toggle preserve the runner-selected DART collision frontend; all eight
sidecars report `dart`, with exact and boxed solver identities in paired
order. The four exact cells report 145, 100, 240, and 89 successful attempts,
zero accepted caps, failures, or boxed fallbacks, and worst residuals no
larger than `9.99641629919505e-7`. The boxed cells correctly declare exact
diagnostics unavailable. Independent replay verification passes all eight
members and all six group/comparison artifacts.

Manual inspection shows the same four-second proxy classification in both
solver lanes: ejected/ejected/captured/ejected. No visible solver difference
or general solver-superiority claim is made.

- `mu=.2, omega=2` direct comparison:
  `assets/pr_media_current_head_c95_small_rows/groups/turntable_mu02_omega2__exact_vs_boxed/clip.mp4`,
  SHA-256
  `56f2a7ee598dffd6716cdf4a7b6b8560e7587ec9bc871f74d0d6e28c4d00daf3`.
- `mu=.2, omega=5` direct comparison:
  `assets/pr_media_current_head_c95_small_rows/groups/turntable_mu02_omega5__exact_vs_boxed/clip.mp4`,
  SHA-256
  `2cab4896f79a116949e360871ac319b6193849fd04ad7bf1bfd43cdec16814e3`.
- `mu=.5, omega=2` direct comparison:
  `assets/pr_media_current_head_c95_small_rows/groups/turntable_mu05_omega2__exact_vs_boxed/clip.mp4`,
  SHA-256
  `5d16450c622dd21ae1144ce66df3c04419f0bf88448fe3ccd32f90f170fc6b62`.
- `mu=.5, omega=5` direct comparison:
  `assets/pr_media_current_head_c95_small_rows/groups/turntable_mu05_omega5__exact_vs_boxed/clip.mp4`,
  SHA-256
  `83bd9494cca0968fd18b23a68d02c9bc19d66e4b7559cb3f893b6b3891229ad8`.

Each direct comparison is 1320x530, H.264/yuv420p at 30 fps, and passes a
full decode. The 61 distinct captured images are timeline-preserving encoded
as 121 frames over 4.033333 s, so the four-second physics horizon is shown at
approximately real time rather than as a two-times time lapse. These proxy
scenes use a square support and reconstructed ramp, body, camera, and horizon;
they do not replace the source-pinned group or establish paper parity.

### Figure 5: source-pinned Painleve adapter

The north-star Figure 5 lane is now the paired source-parameterized schedule,
not the earlier proxy. `painleve_author_mu05` runs
`fbf_author_painleve_mu_0_5`, `painleve_author_mu055` runs
`fbf_author_painleve_mu_0_55`, and `painleve_author` groups the two cells. Both
cells support exact and boxed lanes under the same DART Native
`FourPointPlanar` frontend. The exact lane's named
`source_gamma_c_5_strict_dart_adapter` policy maps the public `gamma_c=5`
request onto DART's adaptive safe-step convention; it remains a DART solver
policy rather than a port of the authors' Warp/Newton backend.

The authoritative ignored current-head v2 bundle is
`docs/dev_tasks/fbf_exact_coulomb_friction/assets/pr_media_current_head_67073/`;
it supersedes the older dedicated Figure 5 and c95-bound captures as the
PR-upload source. Its capture and independent verification summaries pass; the
Figure 5 slice contains four member results and four group results. All four
member records have exact live/sidecar physics-contract matches, and the groups
are transitively bound to those member metadata hashes. Each member and
composite is a
61-frame, 30 fps H.264/yuv420p MP4 with a passing full decode; the generated
panels and selected keyframes were manually audited.

| Cell/lane | Classified current-DART-adapter outcome | Horizontal travel | Exact strict audit |
| --- | --- | ---: | --- |
| `mu=.5`, exact | `upright_near_rest` | `1.5986787381 m` | 119 attempts/solves; 0 failures/fallbacks; final `5.2255077e-7`; worst `9.7391465e-7` |
| `mu=.5`, boxed | `upright_near_rest` | `1.5977005918 m` | Not applicable |
| `mu=.55`, exact | `tumbled_near_rest` | `1.5399225956 m` | 108 attempts/solves; 0 failures/fallbacks; final `9.1964345e-7`; worst `9.9977460e-7` |
| `mu=.55`, boxed | `upright_near_rest` | `1.6623056217 m` | Not applicable |

Group clip candidates and SHA-256 identities:

- exact `mu=.5` / `mu=.55` group:
  `groups/painleve_author/clip.mp4`,
  `eba811334e0c4df5e9368196011607bcdd4f70d540b9b3f74717ff6163d97a3c`;
- boxed `mu=.5` / `mu=.55` group:
  `groups/painleve_author__boxed/clip.mp4`,
  `be87ae107ba0afd4fc9dcf5e9f89e128d68862f1ae0b7db83a9bfc2eb1f10687`;
- `mu=.5` exact-vs-boxed comparison:
  `groups/painleve_author_mu05__exact_vs_boxed/clip.mp4`,
  `77d3286dde96785a6c36cd901e92f183409098ba2bd8dbb426489f537fe71209`;
- `mu=.55` exact-vs-boxed comparison:
  `groups/painleve_author_mu055__exact_vs_boxed/clip.mp4`,
  `2c71e565559dea513870b56bba3c709cf015707b171cfdb45b5cf64fde31f70f`.

The four member hashes are, in exact `.5`, exact `.55`, boxed `.5`, boxed
`.55` order: `098c61ebf02bc8bb8e728c01f3c1a328d02c9abd7237d0f8f81df2ec8307668f`,
`a7afe4e0c550a4a644451a9f146a4dd1178daf75df56e5f7c1ba16cea4cc037f`,
`e2ae95ad2c9f189ef282374b31206686d85926e0448ae0d0dd2f964a9239d35d`,
and `cd847896829bb67122d889fe57ffdf20d416392f7a825b71a94e99ca620321cf`.

The defensible claim is exactly: under the pinned current DART adapter, exact
and boxed lanes diverge at `mu=.55`. Source-backend equivalence, trajectory
equivalence, paper Figure 5 parity, timing comparability, and solver
superiority remain false. The local evidence closes the capture gates, but the
PR row remains unpublished until a maintainer manually drags the two
exact-vs-boxed clips into the browser composer and records the resulting
GitHub user-attachment URLs.

### Figure 5: historical Painleve-style proxies

Both exact captures have 76 distinct frames. The `mu=.5` lane reports 150/150
exact solves and worst residual `9.99349498066255e-7`; `mu=.55` reports
142/142 exact solves and worst residual `9.982525703977317e-7`. Both have zero
accepted caps, failures, or boxed fallbacks. Manual inspection shows exact
`mu=.5` rocking and returning upright, exact `mu=.55` tumbling and remaining
horizontal, and both boxed bodies remaining upright.

- `mu=.5` direct comparison:
  `assets/pr_media/current_head/groups/painleve_mu05__exact_vs_boxed/clip.mp4`,
  SHA-256
  `aa78667045df7e4fbe9ad42fb2de90e9e26064fbbcf7181ff91107231bc98bed`.
- `mu=.55` direct comparison:
  `assets/pr_media/current_head/groups/painleve_mu055__exact_vs_boxed/clip.mp4`,
  SHA-256
  `870220659a473efd716a60c6bbb2727bef7f818e73c1340426c1730ae24a107d`.

Each comparison is 1320x530 for 76 frames (2.533333 s). This retained evidence
is a valid comparison of the declared DART proxy scenes, but it is historical
diagnostic evidence and does not satisfy the north-star source-pinned Figure 5
row. It is not proof of source or paper geometry, source-renderer parity,
solver superiority in general, or strict rest. The separate tracked fixtures
corroborate only their own local classifier and are not trajectory-equivalent
to the rendered demos.

All candidates remain ignored local evidence until a maintainer uploads them
through the pull-request editor and records the resulting
`github.com/user-attachments/...` URLs above.

## Reproducible Capture Entry Points

List the exact current schedule contract without launching a simulation:

```bash
python3 scripts/run_fbf_visual_evidence.py plan \
  --scenario all \
  --solver-lane both \
  --output-root docs/dev_tasks/fbf_exact_coulomb_friction/assets/pr_media
```

Capture the runnable exact-FBF rows, including long schedules, into ignored
working state:

```bash
python3 scripts/run_fbf_visual_evidence.py run \
  --scenario all-runnable \
  --solver-lane both \
  --allow-long \
  --output-root docs/dev_tasks/fbf_exact_coulomb_friction/assets/pr_media
```

After each capture, use the script's `verify` command, manually inspect the
motion and expected outcome, then attach only the final review MP4 through the
PR editor. Raw CSV, frames, logs, sidecars, GIFs, and intermediate panels stay
local.
