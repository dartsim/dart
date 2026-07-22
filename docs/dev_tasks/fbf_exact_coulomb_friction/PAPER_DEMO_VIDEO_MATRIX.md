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
| Figs. 1-2 / incline segment | `fbf_paper_incline`; `incline` | Paired exact/boxed capture and a solver-labeled synchronized comparison are automated | The current member and comparison artifacts pass timeline, solver-identity, stream, and decode gates. Both lanes visibly reproduce the DART threshold pair, with no material solver difference. Separate exact traces corroborate slide/stick behavior, but the combined render has eight contacts versus six across the traces and four per cell in the paper timing row. | Local exact-vs-boxed clip ready below; GitHub URL pending |
| Fig. 3 / backspin segment | `fbf_paper_backspin`; `backspin` | Paired exact/boxed capture and a solver-labeled synchronized comparison are automated | Current-head exact and boxed captures pass timeline, solver-identity, full-decode, H.264/yuv420p, visible translational-reversal, and paired 240-step rolling-state gates. Both solvers reproduce the DART reconstruction; no old-solver failure is claimed. The checker cue is visual-only, and signed angular telemetry remains outside the claim. | Local exact-vs-boxed clip ready below; GitHub URL pending |
| Fig. 4 / turntable segment | Four `fbf_author_turntable_*` scenes; four `turntable_author_*` schedules and `turntable_author` 2x2 group | Author-pinned scenes are exact-only. Paired exact/boxed capture and lane-separated groups are automated for the four `fbf_paper_turntable_*` proxies, which accompany the source-pinned group. | The author group passes solver, timeline, stream, decode, and independent replay gates; manual inspection shows ejected/ejected/retained-through-6-s/ejected in source order. All four paired proxies pass with the DART collision frontend fixed across lanes, and both solvers visibly produce ejected/ejected/captured/ejected over 4 s. These remain DART reconstructions rather than Warp/Newton or paper trace parity. | Local author group and four exact-vs-boxed clips ready below; GitHub URLs pending |
| Fig. 5 / Painleve segment | Source-pinned `fbf_author_painleve_mu_0_5` and `fbf_author_painleve_mu_0_55`; `painleve_author_mu05`, `painleve_author_mu055`, and `painleve_author` group. The older `fbf_paper_painleve*` scenes remain historical diagnostics. | Paired exact/boxed capture and lane-separated grouping pass for both source-parameterized cells | The ignored current-head bundle passes capture summary, independent verify, complete state traces, strict exact audit, outcome classification, full decode, and manual panel/keyframe audit. Both lanes are upright near rest at `mu=.5`; exact tumbles near rest while boxed remains upright near rest at `mu=.55`. This is current-DART-adapter divergence, not source-backend, trajectory, paper, timing, or superiority evidence. | Two local exact-vs-boxed clips ready below; GitHub URLs pending manual browser-composer upload |
| Fig. 6 / 26-card segment | Strict `fbf_author_card_house_4_impact_current_source` plus separate `fbf_author_card_house_4_impact_source_continuation_current_source`; the older `fbf_paper_card_house_26` reconstruction remains distinct | Both source-selected schedules support paired exact/boxed capture with Native `FourPointPlanar`, capacity 4,096, and subdivision 4 fixed. Only the continuation exact lane requests continuation policy | Strict remains blocked at step 35 on the 56-contact group. The separate continuation pair completes 2,400/2,400 and releases at 1,600; exact records 3,351/3,351 solves, 0 failures/fallbacks, 2,605 successes, 113 plateau accepts, 633 max-iteration accepts, 0 shrink caps, and worst residual `0.917120`. Manual inspection shows both standing through release and more endpoint structure in exact. This is qualitative continuation evidence, not strict convergence, trajectory/outcome/golden/backend/timing parity, superiority, or paper parity | Local source-continuation exact-vs-boxed clip ready; strict/parity gate blocked; GitHub URL pending |
| Fig. 7 / 25-stone arch segment | Strict `fbf_author_masonry_arch_25_crown_impact_current_source` plus separate `fbf_author_masonry_arch_25_crown_impact_source_continuation_current_source`; corresponding schedules omit the `fbf_` prefix. Literal-standing and reduced-proxy lanes remain separate | Literal-standing and source-continuation exact/boxed pairs plus independent reuse verification pass | Strict source configuration remains blocked at step 142. The continuation pair completes 2,000/2,000 and releases at 1,600; exact has 2,122/2,122 solves, zero failures/fallbacks, 1,940 plateau and 98 max-iteration accepts. Both arches standing and cubes reaching the crown is a manual, nearly identical visual observation only. Metadata has false paper/semantic flags; no strict convergence, superiority, outcome, timing, backend, trajectory, or paper parity follows | Literal-standing and bounded continuation groups ready locally; strict gate blocked at step 142; GitHub URLs pending browser-composer upload |
| Fig. 8 / 101-stone arch segment | Source-pinned `fbf_author_masonry_arch_101_standing_current_source`; `masonry_arch_101_author_standing_current_source`. The older `fbf_paper_masonry_arch_101`; `masonry_arch_101` proxy remains diagnostic-only | Exact/boxed capture is automated, but the current exact failed prefix prevents a valid full paired comparison. Boxed has a complete decoded member clip; a diagnostic hstack freezes exact at its last frame and is labeled as such | Partial blocker evidence, parity blocked: strict exact stops after step 209 on an iteration cap (`208` contacts, residual `1.2582804496e-6`); boxed completes 1,600/1,600 but fails the standing oracle and visibly collapses. The independent current public-source control also fails the local standing criterion by saved vertical displacement and continues after 1,473/1,600 capped steps. The isolated source-sized contact-gap variant is rejected because it caps earlier at step 161. None is a converged standing result, matched Kamino comparison, historical source trajectory/backend match, golden, timing, or paper-parity result | Local boxed-collapse and frozen-prefix diagnostic clips ready below; GitHub URL pending |
| Tables 6-7 / ten-level card house | Strict `fbf_author_card_house_10_impact_current_source` plus separate `fbf_author_card_house_10_impact_source_continuation_current_source`; corresponding schedules use the same names without the `fbf_` prefix. The older reconstruction remains diagnostic-only | Both schedules use Native `FourPointPlanar`, capacity 4,096, four contacts per pair, one `0.1 m` ground gap, and 159 `0.005 m` card/cube gaps. Only the continuation exact lane requests continuation policy | Strict reaches completed step 31 before a 79-contact failure. The final exact continuation reseal and independent reuse verification pass: 3,200/3,200, 7,702/7,702 solves, zero failures/fallbacks, 2,427 plateau accepts, 763 max-iteration accepts, and `automated_semantic_outcome_validated=false`. A clean boxed control completes 80/80, but the full attempt reached only step 112, was interrupted, and is non-evidence. Paired media, boxed outcome, source/trajectory/physical parity, Tables 6-7 parity, strict convergence, and superiority remain blocked. See `CARD_HOUSE_10_CURRENT_SOURCE_DIAGNOSIS.md` | Final exact-only clip is locally resealed and reverified; full boxed member, paired media, and manual browser upload remain pending |

The source-pinned Figure 5 adapter binds author commit
`b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0`, tree
`ffcdafb61adeda2239c8366d054b548b50d26685e`, and Painleve `run.py` blob
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

The demo build, 13 focused headless/continuation C++ tests, 259 runner Python
tests, and exact/boxed contract-smoke validators pass. The exact 100-step prefix records
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

The adapter and strict replay disable colored block Gauss-Seidel, so colored
source parity remains separate and unproven. Source shrink-cap, plateau, and
continuation semantics are unchanged by this strict A/B and are exercised only
by the separately labeled continuation lane.

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
fail-fast. Colored source parity remains separate. No strict release or
strict full-run trajectory, quantitative physical outcome, source-backend or
timing equivalence, approved golden, Fig. 6 or paper parity, or superiority
follows. See
[`FIGURE6_CONVERGENCE_BLOCKER.md`](FIGURE6_CONVERGENCE_BLOCKER.md). This is an
adapter plus separately labeled continuation-evidence lane.

The two-card A-frame and five-level author-card construction remain useful
inspection/diagnostic scenes. The older reconstructed 26-card lane remains
useful negative evidence. None replaces or can be relabeled as the new
source-selected row.

## Current Local Attachment Candidates

The Figures 1-2 and 5 member clips were captured and independently revalidated
from committed head `165f8541a96` using demo binary SHA-256
`7bcc39725b4488b5796cd23d86a09cbedbaf4befa7bbcb0a840852441a2438b9`.
The direct comparisons bind those member metadata and clips and use the
checked-in compositor. Every member and comparison MP4 below is
H.264/yuv420p at 30 fps and passes a full decode. The direct comparison labels
are intentionally limited to
`EXACT COULOMB FBF` and `EXISTING BOXED LCP`; they identify lanes without
claiming that a visible outcome is numerically or paper-valid.

### Figure 6: source-continuation card house

The local exact/boxed pair uses the same 26-card/four-cube scene, 2,400-step
clock, and successful step-1,600 `p` release. It is a continuation-policy review
artifact and does not satisfy the strict zero-accept rule above. The 301-frame
paired H.264/yuv420p clip is 1320x530, 30 fps, 10.033333 s, and passes full
decode. The summary records `paper_comparable=false` and no automated
semantic-outcome validation.

- Ignored durable bundle:
  `assets/paper_evidence/fig06_card_house_source_continuation_current_v1/`
  (resealed from
  `/tmp/fbf_fig6_source_continuation_pair_20260721T1414_v2/`).
- Direct comparison candidate:
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

### Tables 6-7: ten-level source-continuation final exact member

This exact-only member is locally resealed and independently reverified. The
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

- Ignored final root:
  `assets/pr_media_final/card_house_author_10_impact_source_continuation_current_source/`.
- Timeline SHA-256:
  `7a4b7d878f73068e10c59073b8e1260444a02529db62ab42eaf5c46425a190ae`.
- Clip SHA-256:
  `19637c4255c890f1f32383e7e7e680169688e5d8b071168bc6b4ffdebf33061d`.
- Panel SHA-256:
  `e5ed0d63ca9818292c5a373f476f2841f280f3e01492e0065b2aec8eb95a74d6`.
- Metadata SHA-256:
  `223e828a5284f9fc6aad0b7f57ef010d58db004d85759d036f47883b3753ed9f`.

The separate run summary `/tmp/card10_exact_final_summary.json` has SHA-256
`9a551a96176e5112fc9f1443586c8aee115e1c25c10f766d0088efe4a088e3b2`
and reports `pass=true`. Independent reuse verification passes in 352.27 s;
its separate `/tmp/card10_exact_final_verify.json` summary has SHA-256
`83f9e9db5d013ab8359d5ee5dfb2d05fb4a116082d090b168ec02708ea5a348e`,
kind `verification`, one result, no skips or groups, full-decode success, and
the matching metadata hash. The role-separated
`/tmp/ten-cont-final-review-verify.json` is byte-identical at the same SHA-256.
The exact schedule's blockers are empty only
within the narrow continuation boundary.

A clean boxed control completes 80/80 in about 4 minutes 46 seconds with
`BoxedLcpConstraintSolver`; its timeline SHA-256 is
`ccbdc322791a06d5a8858818acae63e8540ca7770e635545e3c017d84bf96d7d`.
It is not a full outcome. The attempted full boxed member
reached only step 112/3,200 in approximately the same wall budget and was
interrupted without a complete timeline sidecar; all partial frames are
non-evidence. No paired clip or boxed physical outcome exists. This observation
does not establish a performance result or solver superiority. The final exact
member must be uploaded manually through the PR browser composer; no GitHub
user-attachment URL exists.

### Figure 7: current-head literal 25-stone standing pair

The ignored current-head root is `assets/pr_media_current_head_fig07/`.
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

### Figure 7: current-head crown-impact continuation pair

Checkpoint `34d9b66e97c` adds a separately named bounded continuation schedule.
The ignored root is `assets/pr_media_current_head_fig07_crown_continuation/`.
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
source-video evidence only; no matched Kamino result, source camera/golden, or
complete exact trajectory exists. Both candidates remain outside Git until a
maintainer uploads a narrowly captioned blocker clip through the PR editor.

The independent pinned public-source control completes all 1,600 substeps and
exits zero, but only 127 steps converge; 1,473 (`92.0625%`) exhaust
`max_outer=200` and continue. The saved keystone falls
`7.2349853515625` raw units and 57/99 mobile stones exceed the local three-unit
height-change limit. Its compact audit SHA-256 is
`56844eee3d908a1078fe7e76c6a92e31f2de79eb6e1e503ca7173d4e078c6cd4`.
The source persists no final x/y, rotations, or media, so this proves a finite
current-source standing-criterion negative, not visual collapse, a converged
golden, historical Figure 8 parity, or solver superiority.

### Figures 1-2: incline threshold pair

The exact capture has 61 distinct frames and reports 240/240 exact solves,
zero accepted caps, failures, or boxed fallbacks, and worst residual
`9.999836962261359e-7`. Manual inspection shows the same reconstructed
slide/stick threshold separation in both solver lanes; it does not show a
meaningful exact-versus-boxed difference.

- Direct comparison candidate:
  `assets/pr_media/current_head/groups/incline__exact_vs_boxed/clip.mp4`,
  SHA-256
  `ea4341982f2c3b6510a27b9999db0768c8c271453f630fe47b6fd35d45c300fa`.
- Exact member SHA-256:
  `ad6d00ae6a614b0edbc836396c30621b589295a0eec826d060663f6782cee3f9`;
  boxed member SHA-256:
  `aa0e1408b8946d991ab0bf2857775750b341bf087e95b605def6ae3d609166cb`.

The comparison is 1320x530 for 61 frames (2.033333 s). Separate exact traces
own the quantitative threshold claim: `mu=.4` travels `1.7686892884927794 m`,
while `mu=.5` travels `0.0008905412965980523 m`. The combined renderer and
independent traces use different placements and contact counts, so no
full-state equivalence or paper contact/timing parity is claimed.

### Figure 3: backspin reversal

Figure 3 was captured from committed head `fe14820f795` with 131 distinct
frames per lane. The exact-FBF sidecar reports 129/129 exact solves, zero
accepted caps, failures, or boxed fallbacks, and a worst residual of
`9.96497154974839e-7`. The boxed sidecar reports
`BoxedLcpConstraintSolver` with the expected exact-diagnostics-unavailable
gap. Manual panel inspection confirms that the ivory/charcoal checker cells
and coral registration tile visibly change orientation.

The paired 130-step traces reach their forward maxima at step 48, first report
negative translational velocity at step 49, and finish behind the initial
position at about `x=-2.936 m`. Independent 240-step traces pass the full DART
rolling-state gate for both solvers: exact finishes at `vx=-11.5671 m/s` and
boxed at `vx=-11.5658 m/s`, against the `-11.4286 m/s` analytical target with
a `0.5 m/s` tolerance. This demonstrates the reconstructed translational
reversal; it does not prove signed angular-velocity or paper-renderer parity.

- Exact attachment candidate:
  `assets/pr_media/current_head/backspin/clip.mp4`, SHA-256
  `7d4606f4da0a57ffbdfa0528906b21a20d7e1a4e47a6e7eb5387242aecc71928`.
- Boxed attachment candidate:
  `assets/pr_media/current_head/backspin__boxed/clip.mp4`, SHA-256
  `c9855a7a0bdcd5b5540baf5fba5fa40779ee2908be7a09149ddb1bc7e06851d0`.
- Direct comparison candidate:
  `assets/pr_media/current_head/groups/backspin__exact_vs_boxed/clip.mp4`,
  SHA-256
  `2056924dae3ccbffcc66e7203513e00e7a8b890c002243b84bc46dad2adc2498`.

The members are 2.2-second, 66-frame, 1300x506 clips; the direct comparison is
2600x530 with the same duration and frame count. They remain ignored local
evidence until a maintainer uploads the comparison through the pull-request
editor and records the resulting GitHub-hosted URL in the Figure 3 row.

### Figure 4: turntable parameter matrix

The source-configuration-pinned group was captured from committed head
`2255067e6fa` with demo binary SHA-256
`7bcc39725b4488b5796cd23d86a09cbedbaf4befa7bbcb0a840852441a2438b9`
and runner SHA-256
`2433def190621e8f07d35c01af0e9a246cac90a199cd02f3d6a8340d74ee6e28`.
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
  `assets/pr_media/current_head/groups/turntable_author/clip.mp4`, SHA-256
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
  `assets/pr_media/turntable_toggle_fix_v1/groups/turntable_mu02_omega2__exact_vs_boxed/clip.mp4`,
  SHA-256
  `56f2a7ee598dffd6716cdf4a7b6b8560e7587ec9bc871f74d0d6e28c4d00daf3`.
- `mu=.2, omega=5` direct comparison:
  `assets/pr_media/turntable_toggle_fix_v1/groups/turntable_mu02_omega5__exact_vs_boxed/clip.mp4`,
  SHA-256
  `2cab4896f79a116949e360871ac319b6193849fd04ad7bf1bfd43cdec16814e3`.
- `mu=.5, omega=2` direct comparison:
  `assets/pr_media/turntable_toggle_fix_v1/groups/turntable_mu05_omega2__exact_vs_boxed/clip.mp4`,
  SHA-256
  `5d16450c622dd21ae1144ce66df3c04419f0bf88448fe3ccd32f90f170fc6b62`.
- `mu=.5, omega=5` direct comparison:
  `assets/pr_media/turntable_toggle_fix_v1/groups/turntable_mu05_omega5__exact_vs_boxed/clip.mp4`,
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

The ignored durable bundle is
`docs/dev_tasks/fbf_exact_coulomb_friction/assets/paper_evidence/fig05_painleve_author_current_v1/`.
Its capture summary and independent `verify-summary.json` both pass with four
member results and four group results. Each member and composite is a
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
