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

## Source Examples

| Paper/video example | Primary demo scene(s) and capture schedule(s) | Existing-solver comparison | Current gate | PR video |
| --- | --- | --- | --- | --- |
| Figs. 1-2 / incline segment | `fbf_paper_incline`; `incline` | Paired exact/boxed capture is automated | Exact capture is runnable, but the DART reconstruction has three contacts per cell rather than the paper timing row's four. Recapture and inspect both lanes. | Pending |
| Fig. 3 / backspin segment | `fbf_paper_backspin`; `backspin` | Paired exact/boxed capture is automated | Current-head exact and boxed captures pass timeline, solver-identity, full-decode, H.264/yuv420p, visible translational-reversal, and paired 240-step rolling-state gates. Both solvers reproduce the DART reconstruction; no old-solver failure is claimed. The checker cue is visual-only, and signed angular telemetry remains outside the claim. | Local exact/boxed clips ready below; GitHub URLs pending |
| Fig. 4 / turntable segment | Four `fbf_author_turntable_*` scenes; four `turntable_author_*` schedules and `turntable_author` 2x2 group | Author-pinned scenes are exact-only. Paired exact/boxed capture and lane-separated groups are automated for the four `fbf_paper_turntable_*` proxies, which must accompany the source-pinned group. | Source geometry/control schedule is pinned, but DART remains a float64/Native reconstruction rather than Warp/Newton trace parity. Capture both the author-pinned exact group and the paired proxy comparison. | Pending |
| Fig. 5 / Painleve segment | `fbf_paper_painleve`, `fbf_paper_painleve_mu_0_55`; `painleve_mu05`, `painleve_mu055`, and `painleve` group | Paired exact/boxed captures and lane-separated groups are automated | Runnable DART proxies only: the paper does not publish the dimensions, mass, launch state, or timestamps. Do not label these as source-scene parity. | Pending |
| Fig. 6 / 26-card segment | `fbf_paper_card_house_26`; `card_house_26` with projectile action | Paired exact/boxed capture is automated | Blocked: the strict paper-tolerance trajectory is not yet complete. Current known runs either fail `1e-6` or are shorter than the 10 s settle/impact sequence. | Pending |
| Fig. 7 / 25-stone arch segment | `fbf_paper_masonry_arch_25`; `masonry_arch_25` with crown-projectile action | Paired exact/boxed capture is automated | Blocked for parity: the GUI uses reduced-contact oriented boxes and reconstructed projectile parameters, not the literal-wedge 100-contact paper contract. A full-duration exact outcome is still required. | Pending |
| Fig. 8 / 101-stone arch segment | `fbf_paper_masonry_arch_101`; `masonry_arch_101` | Paired exact/boxed capture is automated | Blocked for parity: this is a reduced 38-contact oriented-box approximation; full-manifold long-run standing behavior and a matched Kamino comparison are unproven. | Pending |
| Tables 6-7 / ten-level card house | `fbf_paper_card_house_10_dynamic`; `card_house_10_dynamics` (construction inspection is separate) | Paired exact/boxed capture is automated | Blocked: the 512-contact budget is known to saturate, and no completed zero-failure, zero-fallback 155-card trajectory exists. The source video has no corresponding segment. | Pending |

The two-card A-frame and the five-level author-card construction remain useful
inspection/diagnostic scenes, but they do not replace any row above.

## Current Local Attachment Candidates

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

Both are 2.2-second, 66-frame, 1300x506, 30 fps H.264/yuv420p clips and pass a
full decode. They remain ignored local evidence until a maintainer uploads
them through the pull-request editor and records the resulting GitHub-hosted
URLs in the Figure 3 row.

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
