# Figure 6 Source-Selected Convergence Blocker

## Status And Claim Boundary

This record covers the source-selected four-level/26-card adapter at commit
`a099ca0b29d38f2df13438da13b4d22599bdedb2` plus the additive diagnostics
change that followed it. The scene and capture contract are implemented, but
the strict exact lane does not yet reach the projectile release. This is a
solver blocker, not Figure 6 outcome, media, paper-parity, or superiority
evidence.

The adapter pins author commit
`b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0` and the source-supported
four-level, 600-frame invocation already recorded in the parity and video
matrices. The DART schedule contains 2,400 substeps at `dt=1/240 s` and invokes
`p` after completed substep 1,600. The frozen exact contract uses:

- `max_outer=200`, tolerance `1e-6`, and residual checks every five iterations;
- ten fixed inner block-Gauss-Seidel sweeps;
- adaptive gamma with the source-selected `gamma_c=5` mapping;
- exact-metric local cone projection and warm start/gamma persistence;
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

Keep the checked-in lane strict and keep PR #3377 in draft. Do not change the
default to accept capped iterates, relabel the preview as a solver success, or
publish Figure 6 media from it. The next useful work is a targeted correction
or a demonstrated model/algorithm mismatch for the 56-contact island, followed
by the unchanged 2,400-step strict exact/boxed capture and visual gates. The
row remains blocked until the strict lane completes with zero accepted caps,
failures, and fallbacks and the resulting full-resolution media passes manual
inspection.
