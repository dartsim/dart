# Figure 6 Strict Convergence Blocker And Continuation Evidence

## Status And Claim Boundary

This record covers the source-selected four-level/26-card adapter at commit
`a099ca0b29d38f2df13438da13b4d22599bdedb2` plus the additive diagnostics,
source-style correction policy, and source-style inner-initialization policy
in the current local checkpoint. The strict exact lane does not reach the
projectile release. A separately named telemetry-rich source-continuation lane
now completes and has local exact/boxed media, but it does not clear this strict
solver blocker or establish paper parity or superiority.

The adapter pins author commit
`b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0` and the source-supported
four-level, 600-frame invocation already recorded in the parity and video
matrices. The DART schedule contains 2,400 substeps at `dt=1/240 s` and invokes
`p` after completed substep 1,600. The frozen exact contract uses:

- `max_outer=200`, tolerance `1e-6`, and residual checks every five iterations;
- ten fixed inner block-Gauss-Seidel sweeps;
- adaptive gamma with the source-selected `gamma_c=5` mapping;
- exact-metric local cone projection and warm start/gamma persistence;
- source `project_after_correction=false`, opt-in only for this adapter while
  the existing DART post-correction projection remains enabled by default;
- source-style inner initialization, which copies the current outer reaction
  without cone projection for every inner solve and rejected step-size trial,
  opt-in only for this adapter while DART's carried, projected inner seed
  remains the default;
- no DART constraint regularization, boxed fallback, dense polish, or
  projected-gradient retry; and
- `accept_outer_max_iterations=false` under the strict fail-fast gate.

## Strict Failure Forensics

A fresh rebuilt-binary request again failed closed at completed step 35. Step
34 had 44 collision contacts, 100 exact attempts/solves, no failure or accepted
cap, and worst residual `9.826274595482653e-7`. Step 35 had 68 collision
contacts split into constrained groups of 56, 8, and 4 contacts:

- attempt 101, the 56-contact group, reached the 200-iteration cap;
- attempts 102 and 103 subsequently solved the 8- and 4-contact groups; and
- the later successes previously left the sidecar's last-attempt status at
  `success`, obscuring which island failed even though cumulative accounting
  correctly recorded one failure.

The retained failed-group diagnostics are:

| Field | Value |
| --- | ---: |
| Adapter / build / FBF status | `fbf_failed` / `success` / `max_iterations` |
| Failed-group contacts | 56 |
| Iterations / best iteration | 200 / 200 |
| Final and best residual | `4.1039190451256334e-4` |
| Primal feasibility | `0` |
| Dual feasibility | `4.1039190451256334e-4` |
| Complementarity | `2.4220067503580449e-4` |
| Worst dual / complementarity contact | 11 / 11 |
| Final / safe gamma | `2.7728679142517763` / `0.27728679142517765` |
| Coupling-variation ratio | `0.0053281581128910033` |
| Line-search shrinks | 0 |

The new additive `last_failure` sidecar object preserves these fields across
later successful groups. It is backed by nonvirtual getters and external
solver-keyed state for the newly required status/build/contact fields, so it
does not change the DART 6 public class layout. The evidence validator requires
`last_failure=null` when there are no failures and a typed object when a
failure exists. A failure-then-success unit regression verifies that every
retained field survives the masking sequence.

Fresh local timeline:

```text
/tmp/fbf_author_card_house_4_exact100_last_failure_current_source_20260721/timeline.json
SHA-256 2d04d31134426ac2c4fc87b1774d5285b77740acaeb3ec3a005557b85944bb9d
```

This `/tmp` file is reproducible diagnostic output, not a repository artifact.

## Source Post-Correction Projection A/B

The pinned author solver defaults `project_after_correction=false`, while the
initial DART reconstruction always projected the accepted FBF correction back
onto the cone. The current checkpoint adds an ABI-neutral, default-on DART
policy and disables it only for the source-selected four-level/26-card exact
adapter. The five-level construction and every other exact-FBF caller retain
the prior projected default. The adapter sidecar binds both the source and
DART fields as `project_after_correction=false`.

Strict replays isolate that single difference; no tolerance, iteration cap,
fail-fast rule, fallback, or accepted-cap behavior changed:

| Policy | Requested steps | First failure | Residual / primal / dual / complementarity |
| --- | ---: | ---: | --- |
| Projected DART baseline | 100 | 35 | `4.1039190451256334e-4` / `0` / `4.1039190451256334e-4` / `2.4220067503580449e-4` |
| Source no-projection | 36 and 100 | 35 | `4.0845653576327421e-4` / `3.9380158679450451e-6` / `4.0845653576327421e-4` / `2.3818176330330057e-4` |

Both no-projection requests stop on the same 56-contact group after 200 outer
iterations. They record 103 attempts, 102 solves, one exact failure, zero
accepted caps, and zero boxed fallbacks. The final/best residual improves only
`0.471590382%`; the raw correction also introduces the recorded small primal
violation. Gamma remains `2.7728679142488124` against safe gamma
`0.27728679142488122`, the coupling-variation ratio is
`0.015034809428848014`, and there are zero line-search shrinks.

```text
/tmp/fbf_author_card_house_4_source_correction_exact36_20260721/timeline.json
SHA-256 686be7170e3c217bfa917698a449e7ecde40e500a2c87d073ed58ba2ac833bfb

/tmp/fbf_author_card_house_4_source_correction_exact100_20260721/timeline.json
SHA-256 1a76b71fc4558c7cb978eab410a95948ae50e66522e45dbded07dd36aeb11a77
```

The two replays have byte-identical `solver_diagnostics`; only the requested
horizon and output path differ. This closes a real pinned-source semantic
mismatch but does not clear or move the strict blocker.

## Source Inner Initialization A/B

The pinned author's `BlockGSSolver.solve` starts each inner solve by copying
the current outer reaction into the inner iterate. That raw seed is not
projected before the first sweep, and the same current outer reaction is
supplied again for every rejected step-size trial. The established DART path
instead carries the previous accepted cone-QP iterate and projects the seed.

The current checkpoint adds an ABI-neutral, default-off source-inner option.
Only the exact lane of the source-selected four-level/26-card adapter enables
it; all other exact-FBF callers retain DART's carried, projected default. The
adapter sidecar records the requested, active, restart, and initial-projection
fields. Strict replays isolate this policy on top of the already selected
no-post-correction-projection behavior:

| Policy | Requested steps | First failure | Residual / primal / dual / complementarity |
| --- | ---: | ---: | --- |
| Source no-post-correction projection only | 36 and 100 | 35 | `4.0845653576327421e-4` / `3.9380158679450451e-6` / `4.0845653576327421e-4` / `2.3818176330330057e-4` |
| Source inner initialization also enabled | 36 and 100 | 35 | `4.0844850280896461e-4` / `3.9375947649884479e-6` / `4.0844850280896461e-4` / `2.3815426453852184e-4` |

Both final replays stop on the same 56-contact group after 200 outer
iterations. They record 103 attempts, 102 solves, one exact failure, zero
accepted caps, zero boxed fallbacks, and zero line-search shrinks. Final and
best residual are byte-identical across the two requested horizons. Final
gamma is `2.7728679157546576`, safe gamma is `0.27728679157546576`, and the
coupling-variation ratio is `0.015034989503967002`.

```text
/tmp/fbf_author_card_house_4_source_inner_exact36_v3_20260721/timeline.json
SHA-256 8909e915b63bb2c412a5c5289a5aa690dc1a9ef1d712fe531d12a38d626f0d2e

/tmp/fbf_author_card_house_4_source_inner_exact100_v3_20260721/timeline.json
SHA-256 3e379747bac636c259fe7e9bbd711bb57d5a719d5a1d8d6b9e6317e20b639f73
```

The strict replay is serial because the Figure 6 adapter explicitly disables
colored block Gauss-Seidel. The later one-factor colored trial exercised that
ordering/path with one participant and zero parallel dispatches, and changed
the failed residual only at numerical-noise scale. Reject it only as the next
Figure 6 step-35 discriminator; it is not a multicore colored test or a
general rejection of colored scheduling. Source shrink, cap, plateau, and
continuation semantics remain separate; none changed in this A/B.
The source-inner option therefore closes another pinned-source semantic
mismatch without clearing or moving the strict blocker.

## Pinned Author Continuation Control

The pinned author four-level/600-frame CPU run completes all 2,400 substeps,
but its configured `converged` flag is false on 945 of them. The source solver
continues those finite iterates rather than treating every false flag as a
process-stopping failure:

| Segment | Converged | Unconverged |
| --- | ---: | ---: |
| All 2,400 substeps | 1,455 | 945 |
| Before release, indices 0-1,599 | 1,332 | 268 |
| Release and after, indices 1,600-2,399 | 123 | 677 |

Of the 945 false flags, 632 reach `max_outer=200` and 313 stop on the source
plateau rule. The first false flag is step index 33 and the first cap is index
35. Across the run, the worst natural `final_residual` is
`2.59804445965485`; the worst per-step final checked `r_coulomb` is
`7.597910320688573`.

```text
/tmp/fbf-sca-2026-author/paper_examples/card-house/results/20260721T175341Z/fbf/history.json
SHA-256 b67d3c86f106171008dfbb0aca0a2ca72a9d3747c1a7a6694f57f211d3f83afd
```

This source control is current-source evidence, not a historical paper run,
golden trajectory, or solver-success oracle. It changes the wording of the
DART gate: zero caps/failures remains the strict scientific convergence bar,
but it is stricter than and must not be called source-equivalent execution
semantics. The separately labeled source-continuation physical/video lane below
retains per-step convergence/cap/plateau and residual telemetry. It must not
turn an accepted finite iterate into strict exact-solver success or a
superiority claim.

## Bounded Option Matrix

Process-local option interception was used to test whether an existing option
could clear the blocker without changing checked-in defaults. These runs are
diagnostics only; the temporary wrappers are not evidence producers.

| Change | Completed step | Result / worst residual |
| --- | ---: | --- |
| Frozen `max_outer=200` | 35 | failed, `4.103919e-4` |
| `max_outer=1,000` | 35 | failed, `1.033468e-4` |
| `max_outer=5,000` | 35 | failed, `9.234912e-6` |
| `max_outer=10,000` | 35 | failed, `9.234378e-6` |
| `max_outer=20,000` | 37 | failed, `1.928336e-6` |
| `max_outer=50,000` | 48 | failed, `2.242023e-6` |
| DART constraint regularization | 29 | eight failures, `5.811062e-6` |
| Accept caps, strict fail-fast | 32 | rejected as `iteration_cap` |
| Accept caps, no fail-fast, 100 steps | 100 | 81 accepted caps, `0.1455523` |
| Colored BGS only, c95-bound binary `c95bd5fb916` | 35 | failed, `4.0844850280895566e-4` |
| One global constrained group, c95-bound isolated diagnostic | 35 | failed as a 68-contact aggregate at `4.0848243204467147e-4`; the diagnostic native-partition sub-audit localizes nonconvergence to the 56-contact island |

Warm start off, persistence off, the fuller author-inspired cross-step policy,
30 inner sweeps, 20 shrink attempts, projected-gradient retry, dense polish,
projected-gradient local solves, matrix-free operator, dense snapshot, seed
ablations, step scales 1 and 5, and outer relaxation 0.8 all failed at or
before step 35. Raising the outer budget by 100x or 250x only moved the first
failure, so global option tuning is not a strict solution.

The final colored-BGS trial changed only
`colored_block_gauss_seidel_enabled=false -> true`; participant affinity
remained false and the normalized physics contracts otherwise match. A
process-local interceptor recorded actual use on failed exact attempt 101:
200 colored solves, 17 manifolds, 9 colors, maximum 3 manifolds per color, one
participant, and zero parallel dispatches. The same 56-contact group still reached 200 iterations at
step 35. Its residual differs from the serial c95-bound control
`4.0844850280896461e-4` by only `2.19e-14` relative, far below the
preregistered 10x-improvement continuation gate. This rejects this one-factor
setting as the next Figure 6 blocker lane; it does not test multicore colored
execution or reject the separately validated literal-arch colored result or
pending ten-level colored diagnostic.

```text
/tmp/fbf_fig06_colored_ab_c95_serial.4vH25H/timeline.json
SHA-256 cf2a15fecd9071de7a9b1ce54f8f7182485f77ceb3c140e8c2c02a1174a008c9

/tmp/fbf_fig06_colored_ab_c95_colored_v2.TAnBSG/timeline.json
SHA-256 8458bfc8bc0c8b127dbd11be0c8f4f45c44383c04a5868736c9d59b61bcb890f

/tmp/fbf_fig06_colored_ab_c95_colored_v2.TAnBSG/stderr.log
SHA-256 543c35944d7c33e2b314e811af9381a9c2854274d4dadb5931d233f0ea868716
```

## Global Constrained-Scope A/B

An isolated diagnostic clone pinned to implementation/media head
`c95bd5fb916832c46c60d2ca1948b52d1e2cd7ba` compared the stock native
constrained-group partition with a single global exact-FBF group. The
environment-gated patch was not applied to the topic branch. It retained the
strict current-source scene contract, the contact-row operator, one simulation
thread, zero ERP, no split impulse or deactivation, no fallback/retry/polish,
and no continuation or accepted-cap policy. It merged groups only immediately
before the real solve and independently reconstructed a dense diagnostic
operator after each solve.

Both 36-step requests exited through the exact fail-fast at completed step 35.
The stock run reproduces the c95-bound result: the 56/8/4-contact groups end at
residuals `4.0844850280896461e-4`, `9.6415993787912730e-7`, and zero. The
global run solves all 68 contacts together and ends at
`4.0848243204467147e-4`; slicing that reaction under the original partition
gives `4.0848243204472058e-4`, `6.1744758287200254e-11`, and zero. Every
off-block coefficient of the global `W` is exactly zero under the native
partition for all 35 generations.

At generation 28, both modes start from identical contact fingerprints and
pass their declared residual gates. Their accepted, within-tolerance reactions
differ, and the contact fingerprints first diverge at generation 29. The
global run remains clean through generation 33, then fails at generation 34 on
the 68-contact aggregate; the diagnostic native-partition sub-audit identifies
the same dominant 56-contact island as the only nonconverged partition. Neither
run accepts a capped iterate or invokes boxed fallback. The preregistered
100-step extension was therefore not run.

This rejects only the hypothesis that DART's native-island solve scope is, by
itself, the Figure 6 step-35 cause. It does not show general equivalence between
global and per-island solves: the different admissible generation-28 reactions
change the later trajectory. It establishes no trajectory, physical outcome,
source-backend, timing, performance, solver-superiority, video, Figure 6, or
paper-parity claim. The global result has one final replicate. No raw `W`
matrix is retained; the zero-coupling statement is bound to the reviewed
instrumentation and its complete per-generation logged aggregates.

The isolated evidence package is
`/tmp/fbf_fig06_global_scope_c95.TSfONI/`:

- `RESULTS.md` SHA-256
  `633828adbe08577b6d0973ca817194530ed8a08cbe27e85d2bcb004689919fe9`;
- diagnostic patch SHA-256
  `c1ded53666dd0c515bc81591f22644ace2ccb59a22ae172a60467cef2f5395e2`;
- native sidecar SHA-256
  `9960523cab291c47c51f8d92f5360acb40d1a5f22658e2f7486b32c32396c7aa`;
- global sidecar SHA-256
  `2690bd7a217bcb380f2a0fd83f78e6e14838d0286189e0f875e8a66e5fb5e33d`;
  and
- complete `SHA256SUMS` manifest SHA-256
  `90d72452c6b3ed09e0bc1e408b56e70092557784fd2089e6895d7a31a0c809d3`.

## Source-Sized Contact-Gap Diagnostic

An isolated c95-bound one-factor diagnostic enabled only the existing
four-level scenario's source-gap flag. Native predictive closure then admitted
negative-depth contacts, the ground shape carried a `0.1 m` gap, and all 30
dynamic shapes carried `0.005 m`; the strict source-inner, serial-BGS,
no-projection, no-cap-acceptance, no-fallback contract was otherwise preserved.
The preregistered 36-step gate still failed closed, now after completed step 31
on a 31-contact group at 200 iterations and residual
`1.0006073317077885e-5`. The run recorded 186 attempts, 185 solves, one
failure, zero accepted caps, and zero fallbacks. Contacts were 36 through steps
1-29, 44 at step 30, and 52 at step 31.

This rejects only the hypothesis that representing the recorded source-sized
gaps with DART's predictive contact allowance makes this strict lane complete
the first 36 steps at `1e-6`. The existing stock sidecar stops after step 35 on
a 56-contact group at residual `4.0844850280896461e-4`, but its embedded source
hashes identify ancestor `844c9c316195897cf2bf51f38eafc8ec9dcf959a`, not a
fresh c95 control. Because the compared contact streams differ from step 1, the
residual magnitudes and attempt counts are not performance or superiority
comparisons. The result establishes no general benefit or harm, source contact
model, trajectory, outcome, backend, float32, timing, video, Figure 6, or paper
parity. Keep the checked-in four-level scenario unchanged.

The verified isolated package is `/tmp/fbf_fig06_gap_c95.m6bsif/`:

- `RESULTS.md` SHA-256
  `3b0948c80871d19cbe29495a8abc57ac4f3e92dc518a9ae6551238a9aad9b17a`;
- one-factor patch SHA-256
  `a5269addaae4bb2864ad2b9fa3768cc99bd41e22758af5950d78eae163ab6695`;
- gap-run sidecar SHA-256
  `8683ca69a72ef29ca089fc31647f070d0b5e3e583af028375586ff4d1e3c66b9`;
- analysis SHA-256
  `3a0017fdfa16e9fd3556d4e336ae2c9bd660a715eab644b04255cb764b098e22`;
  and
- complete `SHA256SUMS` manifest SHA-256
  `11888f98a24175f50c09ce95509d754d0bbc1963e5d2294ad982ece280292119`.

## Source-Style Continuation Preview

The pinned author runtime recovers and advances a finite iterate even when its
configured convergence flag is false. That semantic difference motivated one
process-local GDB override setting `accept_outer_max_iterations=true` with
fail-fast disabled. The sidecar physics contract records the changed option,
but its `runtime_command` cannot reproduce the debugger mutation; this is an
unsealed diagnostic preview. It completed all 2,400 steps and successfully
executed the release action at step 1,600, but it is not exact-FBF validity
evidence:

| Metric | Preview result |
| --- | ---: |
| Completed steps / simulation time | 2,400 / 10 s |
| Exact attempts / solves | 3,231 / 3,231 |
| Accepted at cap | 1,106 (34.2%) |
| Caps before / after release | 379 / 727 |
| Exact failures / boxed fallbacks | 0 / 0 |
| Worst residual | `0.61608914241359314` |
| Total outer iterations | 284,435 |
| Final-group residual / iterations | `7.99797e-7` / 26 |

All recorded simulation times and post-initial diagnostic values were finite.
That establishes finite runtime continuation only. The 64x64 final image is
nonblank, but the physical outcome is not legible enough for a visual verdict.
It must not be uploaded to the PR or presented as a completed Figure 6 video.

```text
/tmp/fbf_author_card_house_4_exact_accept_caps_2400_20260721/timeline.json
SHA-256 034fa50433620a0050839fa26408700a86ad486e2e022edd3dbd29d19a0ad2ec

/tmp/fbf_author_card_house_4_exact_accept_caps_2400_20260721/final.png
SHA-256 0ade57d83cff8ed96ac19a253de79d51bbec84eaae60f55ae0237190e90e52ce
```

## Reproducible Source-Continuation Capture

The separately named scene and schedule
`fbf_author_card_house_4_impact_source_continuation_current_source` and
`card_house_author_4_impact_source_continuation_current_source` replace the
debugger mutation with an explicit, fail-closed continuation policy and full
telemetry. They keep the same 26-card/four-cube geometry, Native
`FourPointPlanar` frontend, 2,400-step clock, and step-1,600 release. Only the
exact lane requests the continuation policy; boxed is the same physical scene
and schedule without exact-only policy claims.

The capture completes both lanes for 2,400/2,400 steps and records a successful
`p` action at step 1,600. Exact records:

| Metric | Source-continuation result |
| --- | ---: |
| Exact attempts / solves | 3,351 / 3,351 |
| Ordinary successes | 2,605 |
| Plateau accepts | 113 |
| Max-iteration accepts | 633 |
| Accepted continuation outcomes / share of solves | 746 / 22.262% |
| Steps with at least one continuation accept | 723 |
| Line-search shrink caps | 0 |
| Exact failures / boxed fallbacks | 0 / 0 |
| Worst final residual | `0.91712002943322535`, first reached at step 2,101 |

Every accepted plateau or max-iteration result is serialized as such; none is
relabeled `success`. The all-step run is therefore source-continuation evidence,
not strict convergence.

Independent inspection of the synchronized 301-frame, 10.033333-second clip
finds both houses standing through release. Exact and boxed are pixel-identical
only at step 0; viewport difference is 0.165% at step 1,600 and 11.985% at the
endpoint. In this DART source-parameterized four-level scene, exact completes
without exact-solver failures/fallbacks and visibly retains more upright
card-house structure after impact than boxed. The official MuJoCo panel degrades
while settling, whereas DART boxed remains upright until impact, so do not map
DART lanes to paper lanes or infer a mechanism. This is not a quantitative
physical-outcome or trajectory equivalence result, approved golden, source
collision/solver-backend equivalence, timing result, or proof that exact is
superior. The capture summary records
`paper_comparable=false` and no automated semantic-outcome validation.

The ignored durable evidence root is
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
- paired clip:
  `282aebfb9e2e38fe3741db28e2ce909fb548d7aa46d048302a3b0e0bea9e1786`.

A fresh download of the official video has SHA-256
`d5356e1b31487be62b75af05efbfecdb70ad5d98501a8efd378fcedf066e4794`,
exactly matching the prior source audit. Generated files remain outside Git.
The paired MP4 is a local PR-attachment candidate only; publication requires a
GitHub user-attachment URL.

## Decision And Next Work

Keep the strict lane fail closed and keep PR #3377 in draft. Retain the
source-style no-projection and inner-initialization options because they close
pinned-source mismatches, not because they establish convergence. Do not
change the default to accept capped iterates, relabel the preview as a solver
success, or publish the debugger preview. The reproducible continuation clip
may be attached only with its acceptance telemetry and narrow claim boundary.

The row remains blocked until the strict scientific lane completes all 2,400
steps with zero accepted caps, failures, and fallbacks. Any next strict A/B
must isolate one remaining source mismatch without combining tolerance,
budget, fallback, fail-fast, or accepted-cap changes. The one-participant
colored-ordering, one-global-group, and source-sized-gap candidates are all
rejected for this Figure 6 strict-prefix blocker. Do not promote another solver
knob without a new source-backed, preregistered mismatch. Source
shrink-cap, plateau, and continuation semantics likewise require a separately
labeled, telemetry-rich physical/video lane before producing strict outcome
evidence. The continuation lane now has a validated local full-resolution
attachment candidate, but no GitHub URL and no strict, quantitative-parity, or
superiority verdict.
