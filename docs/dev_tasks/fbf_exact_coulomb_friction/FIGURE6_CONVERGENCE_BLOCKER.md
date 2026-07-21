# Figure 6 Source-Selected Convergence Blocker

## Status And Claim Boundary

This record covers the source-selected four-level/26-card adapter at commit
`a099ca0b29d38f2df13438da13b4d22599bdedb2` plus the additive diagnostics,
source-style correction policy, and source-style inner-initialization policy
in the current local checkpoint. The scene and capture contract are
implemented, but the strict exact lane does not yet reach the projectile
release. This is a solver blocker, not Figure 6 outcome, media, paper-parity,
or superiority evidence.

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
colored block Gauss-Seidel. The colored implementation accepts the same raw
source-style seed for internal consistency, but colored source parity has not
been demonstrated and remains a separate lane. Source shrink, cap, plateau,
and continuation semantics also remain separate; none changed in this A/B.
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
semantics. A future source-continuation physical/video lane must be separately
labeled, must retain per-step convergence/cap/plateau and residual telemetry,
and must not turn an unconverged step into an exact-solver success or
superiority claim. That lane is not implemented in this checkpoint.

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

Warm start off, persistence off, the fuller author-inspired cross-step policy,
30 inner sweeps, 20 shrink attempts, projected-gradient retry, dense polish,
projected-gradient local solves, matrix-free operator, dense snapshot, seed
ablations, step scales 1 and 5, and outer relaxation 0.8 all failed at or
before step 35. Raising the outer budget by 100x or 250x only moved the first
failure, so global option tuning is not a strict solution.

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

## Decision And Next Work

Keep the checked-in lane strict and keep PR #3377 in draft. Retain the
source-style no-projection and inner-initialization options because they close
pinned-source mismatches, not because they establish convergence. Do not
change the default to accept capped iterates, relabel the preview as a solver
success, or publish Figure 6 media from it.

The row remains blocked until the strict scientific lane completes all 2,400
steps with zero accepted caps, failures, and fallbacks. Any next strict A/B
must isolate one remaining source mismatch without combining tolerance,
budget, fallback, fail-fast, or accepted-cap changes. Colored source parity is
a separate unproven lane. Source shrink-cap, plateau, and continuation
semantics likewise require a separately labeled, telemetry-rich physical/video
lane before producing outcome or media evidence. Neither lane currently has
promotable full-resolution media.
