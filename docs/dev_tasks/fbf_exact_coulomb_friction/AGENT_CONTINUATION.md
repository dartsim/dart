# Agent Continuation And Progress Tracker

This dev task is active research and implementation work. Future agents must
not treat a focused green test slice, a passing demo smoke, or a partial report
as completion of the task.

Last tracker update: 2026-07-09 (second session), after landing the
incremental inner block-Gauss-Seidel solver and the scratch-backed
contact-row Delassus operator, which together cleared the long-standing
60/64-contact card-house fallback boundary and made full natural contact
manifolds solvable. The branch also moved to the newer `release-6.20` base
`db255a08e8e` (which includes the WP-PG.14 opt-in matrix-free contact LCP);
that base's contact-manifold changes (#3360/#3364) made two previously green
rows harder, and both were re-fixed in this slice.

Headline evidence from this slice (all on the current base):

- Inner solve rework (`ExactCoulombFbfSolver.hpp`): per-solve cached diagonal
  Delassus blocks, incremental block-column updates with a periodic full
  refresh, cached local Lipschitz steps, and 3-row segment gradients. The
  reference 6-argument overload is preserved and forwards to the new
  two-operator overload.
- New `dart/constraint/detail/ExactCoulombContactRowOperator.hpp`
  (WP-PG.14 pattern; `ContactConstraint` friend declaration): O(n) `W*x`
  scatter/gather products, O(1) per-contact diagonal blocks, body-incidence
  block-column updates, and impulse-test-free dense snapshot assembly for
  supported groups (exact-typed non-self-collision three-row contacts between
  single free rigid bodies). Enabled by default through the new
  `useContactRowDelassusOperator` solver option with automatic impulse-test
  fallback for unsupported groups; parity against the impulse-test snapshot is
  proven by `ConstraintSolver.ExactCoulombContactRowOperatorMatchesImpulseTests`
  and the `ExactCoulombFbfWorldSmokeUsesContactRowOperator` world smoke.
- Card house one-step probes: 56/1 clean in `277 ms` (was `4971 ms`); 60/1
  clean in `1.59 s` (was the fallback boundary); the natural per-pair-1
  manifold (63 contacts) clean in `1.76 s`; the full natural per-pair-4
  manifold (108 contacts) clean in `2.5 s` at `8479` max FBF iterations with
  zero fallbacks. The one-step "full contact budget" blocker is closed.
- Masonry arches: the truncated caps were the hard cases, not the full
  manifolds. The 101-stone arch full per-pair-4 manifold (512 contacts)
  solves cleanly in `1626` iterations (~50 s at probe defaults), while capped
  100/2-256/2 rows stall on the current base. The 25-stone natural manifold
  is 96 contacts (per-pair 4 and 8 both produce 96) and needs
  `outer_relaxation = 1.5` plus a larger outer budget.
- Base-refresh regressions fixed: `MasonryArch25ProjectileScaffoldRuns`
  stalled at dual `4.13e-6` after the base refresh with identical values on
  all three product routes; adding `kArchOuterRelaxation = 1.5` and raising
  `kArchMaxOuterIterations` to `120000` makes it pass in `9.9 s`. The
  historical "101-stone 100/2 clean" note is stale on the current base at
  relaxation 1.0.
- Full paper-fixture suite: 14/14 green in well under a minute (previously
  74 s for 13/14 on this base), including the reduced arch probes at
  `412 ms` and `1287 ms`.

This is still not full paper parity: long-run settle/projectile outcomes,
author/Rigid-IPC geometry, Figure 9/10 long-run evidence, snapshots, timing
parity, and external baselines remain open, and the reduced-cap defaults have
not yet been promoted to the measured full-manifold rungs across
test/benchmark/trace/gamma/GUI surfaces.

## Tracker Update Protocol

Every implementation slice must update this file before reporting status. Do
not rely only on a chat summary, passing CTest output, or `PR_REPORT.md`.

For each slice, update all applicable sections below:

- Current status snapshot: one-line state, latest implemented slice, and most
  important open blocker.
- Progress ledger: row status, evidence, and still-missing cell for each area
  touched by the slice.
- Active work board: move rows between active/open/done and add the next
  concrete command or file.
- Verification ledger: record the command, result, and date after the final
  code or docs edit in the slice.
- PR report: mirror user-facing commands/results and keep the "not covered
  yet" section visible.

If any paper-parity row remains partial, proxy-only, construction-only, not
started, or small-fixture-only, the task is still active.

Before answering the user or opening a PR, check the completion gate below.
Do not compress this task into "tests passed" or "GUI scenes exist"; the
paper-parity question is about coverage of the paper's tests, benchmarks,
interactive examples, visual evidence, and comparison report.

## Completion Gate Checklist

Leave this checklist in the task folder until all rows are checked and the
durable pieces have been promoted out of `docs/dev_tasks/`.

| Gate | Status | Required evidence before checking |
| --- | --- | --- |
| Paper-scene tests | Partial | Headless tests for every paper fixture, including full-contact 26-card settle/projectile, 10-level house dynamics, 25-stone arch projectile/rest outcome, and 101-stone balance outcome. One-step full-manifold coverage now exists (card-house 108/4 clean; 25-stone arch 52 contacts and 101-stone arch 204 contacts clean under author Rigid-IPC geometry), but the multi-step full-contact card-house settle still falls back (steps 2+ at 122-138 contacts) and long-run settle/balance outcomes for both arches remain open |
| Paper benchmarks | Partial | Google Benchmark or CSV timing rows for every paper benchmark, with exact-FBF and boxed/external baselines where applicable, plus hardware/build metadata |
| GUI examples | Partial | `dart-demos` scenes for every paper example, each with visible `Scene` tab overview, expected result, coverage limits, useful diagnostics/overlays, and paper-matched snapshot/capture evidence |
| OSG/GUI support | Tracked | Renderer, capture, camera, overlay, and reusable ImGui widget improvements applied whenever the current viewer cannot make a promoted paper GUI example self-contained or visually verifiable |
| Residual and sweep figures | Partial | Figure 9 residual histories and Figure 10 gamma sweeps for full-contact/long-run paper scenes, not only reduced one-step scaffolds |
| External comparisons | Wired (MuJoCo) / probe (Kamino) | Kamino, MuJoCo, and paper implementation comparisons wired only as test/example/benchmark dependencies, or a source-level note explaining why a baseline is unavailable. `fbf_paper_mujoco_baseline.py` is validated end-to-end (backspin/incline/turntable plus opt-in `masonry_arch_101_rigid_ipc`) behind the non-default `fbf-baselines` pixi feature; `fbf_paper_kamino_baseline.py` is an availability probe only (SolverKamino importable, CUDA-gated, not yet wired into a paper-parity scene); paper-code repository remains HTTP 404 |
| Citation and credit | Partial | Source comments, RTD page, website copy, PR report, and any GUI/docs text clearly credit Song, Fan, Ascher, and Pai's SCA 2026 paper and the Rigid-IPC masonry-arch dataset where used |
| PR report | Partial | `PR_REPORT.md` contains the full current command/results report, explicit "No" coverage answer, GUI commands, benchmark rows, trace/sweep outputs, negative results, and remaining gaps |
| Default-solver safety | Done so far | Existing boxed-LCP/default behavior remains unchanged unless a later explicitly reviewed public exposure decision changes it |

## Current Status Snapshot

| Field | Current value |
| --- | --- |
| Overall state | Active; partial DART-side prototype, not paper parity |
| Required answer to full-coverage question | No |
| Latest implemented slice | Incremental inner block-GS solver (cached diagonal blocks, incremental block-column updates, segment gradients) plus the scratch-backed `ExactCoulombContactRowOperator` (WP-PG.14 pattern, default-on with impulse-test fallback), clearing the 60/64-contact card-house boundary, solving the full natural card-house (108/4) and 101-stone arch (512/4) manifolds, and re-fixing the two base-refresh regressions with `kArchOuterRelaxation = 1.5` and a `120000` arch outer budget |
| Current strongest evidence | Focused unit/integration gates, full paper-fixture CTest, small-fixture benchmark/trace/sweep smokes with zero exact-FBF fallbacks, cap-aware one-step reduced-contact card-house/arch gamma-sweep smokes with explicit contact-cap and solver-budget columns, benchmark-only `fbf_paper_arch_probe` rows for 25/101-stone arch cap boundaries, reduced 25-stone arch projectile test/trace/benchmark/GUI capture evidence, all-group residual-history CSV smokes and regenerated individual/combined SVG plots for backspin, reduced 26-card, and reduced 25/101-stone arch one-step scenes, `pixi run demos -- --verify-fbf-scene-docs` checking all nine FBF Scene-tab explanations, demo catalog/factory smoke, host-rendered scrollable `dart-demos` Scene-tab metadata with FBF overview/expected/coverage text plus solver diagnostics, 26-card and 25-stone-arch GUI phase-scaffold launch controls, action-aware `pixi run capture-action` projectile captures for `fbf_paper_card_house_26` and `fbf_paper_masonry_arch_25`, regenerated one-step GUI capture sheet plus non-blank verdicts for all nine `fbf_paper_*` scenes after the host renderer change, reproducible `pixi run demos -- --scene ...`, `pixi run capture ...`, and `pixi run capture-action ...` GUI commands, reduced-contact 26-card dynamic probe plus tracked/full-scene/phase-summary trace smokes, reduced 26-card two-step settle/projectile benchmark rows, construction-only 10-level card-house scaffold, reduced-contact 25-stone arch default row at 48/2 with higher-cap probe rows clean at 64/4 and 80/4 but the 100/4 row timing out under 120 s, reduced-contact 101-stone arch default row at 38/2 plus cap-probe rows clean through 100/2, tracked/full-scene trace smokes, paper-scaled card-house cap probes showing 59 contacts can solve without fallback but is too slow for the default rung and 60 contacts still falls back, history-enabled cap-probe rows showing the 60-contact failed residual moves from `1.146e-5` to pre-polish `5.338e-6` over 30000 retained iterations and dense polish updates the failed final residual to `5.334e-6` without clearing tolerance, docs/lint/build gates |
| Most important blocker | The multi-step full-contact settle frontier is RESOLVED (split-impulse operation for the exact path plus the base split-impulse impulse-preservation fix; see the 2026-07-11 verification-ledger row): full-manifold settle steps now solve exactly at `1e-6` with zero fallbacks and genuine no-creep physics at ~`1.4 s`/step. The remaining open items are evidence collection, not solver capability: the 10 s card-house settle/projectile sequence and 10 s arch-balance runs are executing on the fixed base; still open after those land are the 101-stone long-run balance, 10-level dynamics, Figure 9/10 long-run artifacts, paper-matched snapshots, a hardware-metadata timing report, and a CUDA-hardware Kamino comparison |
| Next technical target | Converge the multi-step full-contact card-house settle (deeper solver/scratch work or a warm-start/step-size change that clears the `2.9e-6`-`1.5e-4` residual range), then land the in-progress long-run warm-started runs (10 s 25-stone arch balance, 101-stone arch balance, full-contact card-house settle) with traces, residual histories, sweeps, snapshots, and timing rows |
| External baseline state | MuJoCo 3.10.0 is verified working with measured backspin comparison evidence (paper `dt=1/60`: ~20% deviation plus a spurious 11 m bounce versus DART exact-FBF asserting the analytic terminal state; converges at `dt=1/2000`) and a benchmark-only harness is being wired behind a non-default `fbf-baselines` pixi feature. Kamino is publicly available as `newton.solvers.SolverKamino` in NVIDIA's open-source Newton engine (`pip install newton`, Apache-2.0, import verified 2026-07-09) but unwired and CUDA-gated — an open integration task, not an availability gap. The paper-code repository remains HTTP 404 (page, API, and unauthenticated `git ls-remote`, rechecked 2026-07-09). All such dependencies stay test/example/benchmark-only |

## Required First Answer

Until every completion criterion below is satisfied, the answer to:

```text
Do we have all tests, benchmarks, and GUI examples from the paper?
```

is:

```text
No.
```

Then state the implemented subset and the exact missing paper-parity rows.

## Progress Ledger

Update this table whenever a slice changes the implementation state.

| Area | Status | Evidence | Still missing |
| --- | --- | --- | --- |
| Source digest and citation | Done | `paper-notes.md`, `README.md`, exact-FBF source comments, RTD research-papers page | Keep citation current if canonical BibTeX or public scene assets appear |
| Core exact-Coulomb math | Done for reference path | `CoulombCone.hpp`, `ExactCoulombContactProblem.hpp`, `ExactCoulombFbfSolver.hpp`, automatic paper residual scales, bounded opt-in residual-history samples, retained best-iterate diagnostics on max-iteration failures, unit tests | Production tuning and broader numerical stress tests |
| DART 6 contact adapter | Default-on scratch-backed operator for supported groups; dense snapshot still used for staging/polish | `ExactCoulombConstraintAdapter.hpp` staging bridge plus the new scratch-backed `dart/constraint/detail/ExactCoulombContactRowOperator.hpp` (WP-PG.14 pattern, `ContactConstraint` friend declaration), enabled by default through `useContactRowDelassusOperator`: O(n) `W*x` scatter/gather products, O(1) per-contact diagonal blocks, body-incidence block-column updates, and impulse-test-free dense snapshot assembly for supported groups (exact-typed non-self-collision three-row contacts between single free rigid bodies), with automatic impulse-test fallback for unsupported groups. Parity against the impulse-test snapshot is proven by `ConstraintSolver.ExactCoulombContactRowOperatorMatchesImpulseTests` and `ExactCoulombFbfWorldSmokeUsesContactRowOperator`. Cross-step pair-keyed contact-manifold warm starts (world-space reaction records matched by body pair and nearest contact point) are proven by `ExactCoulombFbfWorldSmokeManifoldWarmStartAcrossSteps`. Retains the residual-selected projected local/global Delassus cold-start seed with diagonal fallback, the operator-extracted local diagonal-block seed helper, and `applyExactCoulombConstraintDelassus(...)` | The one-step full-manifold gap is closed for supported groups, but multi-step full-contact convergence is not (see the card-house settle row); dense snapshot dependence remains for staging/dense-polish recovery and mixed non-isotropic/joint-coupled groups; second adversarial review pass on the post-hardening state |
| Opt-in constraint solver | One-step full natural manifolds solve for supported groups; multi-step full-contact settle does not yet converge | `ExactCoulombFbfConstraintSolver` with the incremental inner block-Gauss-Seidel solve (cached per-solve diagonal Delassus blocks, incremental block-column updates with a periodic full refresh, cached local Lipschitz steps, 3-row segment gradients; reference 6-argument overload preserved) cleared the former 60/64-contact one-step card-house fallback boundary and made full natural contact manifolds solvable one-step: card-house 108 contacts clean in `2.5 s` (`8479` max FBF iterations, zero fallbacks); 25-stone arch natural manifold clean under author Rigid-IPC geometry at 52 contacts in `256 ms`/`892` iterations; 101-stone arch natural manifold clean at 204 contacts in `17.2 s`/`28750` iterations; all zero-fallback at `1e-6`. Cross-step pair-keyed manifold warm starts, fallback diagnostics, PG retry continuing from a finite best failed block-GS FBF reaction, bounded dense residual polish, opt-in last-solve/failed-solve best residuals, dense-polish counters, retained per-solve residual-history diagnostics (`kResidualHistoryMaxSamples` raised to `130001` for full-manifold scale), opt-in `useMatrixFreeDelassusOperator`/`useMatrixFreeDelassusSeed`, diagnostic getters, synthetic plus `World` smokes, and benchmark/probe columns all carry over. Independent adversarial review pass 1 found no blockers or majors on the solver-algebra change and commissioned three hardening tests (refresh-boundary stress, flipped-pair warm start, multi-contact/count-change warm start), now landed | The multi-step full-contact card-house settle is the current honest blocker: steps 2+ reach 122-138 contacts at per-pair-4, cross-step warm-started, with a `120000`-outer budget, but still fall back to boxed LCP at failed residuals ranging `2.9e-6`-`1.5e-4`. Also open: true long-run manifold persistence beyond the proven cross-step matching smoke, a second adversarial review pass on the post-hardening state, scratch reuse beyond the contact-row operator, removing the dense snapshot requirement for staging/polish, and the public exposure decision |
| Default DART 6 solver behavior | Preserved | FBF route is opt-in | gz/default-solver decision if public/default behavior changes |
| Incline fixture | Partial paper parity | Headless regression, boxed baseline, benchmark rows, CSV trace, GUI scene; final exact-FBF sweep rows use `step_size_scale=2`, report zero fallbacks for both stick/slide cases, classify `incline_mu_0_5` as `stick_like` with max residual `9.998e-7`, and classify `incline_mu_0_4` as `slide_like` with max residual `9.991e-7` | Full sweep, paper snapshot parity, external comparisons |
| Backspin fixture | Partial paper parity | Headless regression, boxed baseline, benchmark rows, CSV trace, GUI scene | Paper trajectory plot, residual convergence plot, external comparisons |
| Turntable fixture | Partial paper parity | Headless capture/ejection grid, boxed baseline, benchmark rows, CSV trace, GUI scene | Full radial trajectories, snapshots, timing parity, external comparisons |
| Painleve fixture | Proxy only | DART-side proxy regression, boxed baseline, benchmark rows, CSV trace, GUI scene | Authors' exact parameters/scene, paper snapshot and trajectory parity |
| House of cards, two-card precursor | Done as precursor | Enabled `CardHouseAFramePrecursorStands` at `1e-6` | Not a substitute for Fig. 6 four-level house |
| House of cards, 26-card four-level | One-step full natural manifold (108 contacts) solves clean; multi-step full-contact settle is the new open blocker | Enabled `CardHouseFourLevelSceneBuilds`. The incremental inner block-Gauss-Seidel solver plus the default-on scratch-backed `ExactCoulombContactRowOperator` cleared the former 60/64-contact one-step fallback boundary: 56/1 clean in `277 ms` (was `4971 ms`), 60/1 clean in `1.59 s`, the natural per-pair-1 manifold (63 contacts) clean in `1.76 s`, and the full natural per-pair-4 manifold (108 contacts) clean in `2.5 s` at `8479` max FBF iterations with zero fallbacks. Promoted benchmark row `card_house_26_reduced_contact_exact_fbf` now reports 108 contacts, `1867 ms`, max FBF iterations `8479`, max residual `9.995e-7`, zero failures/fallbacks; trace and gamma-sweep rows agree; the full 16-gate focused CTest set (including the two full-manifold arch probes) passed 16/16 in `167 s` with assertions un-weakened (`1e-6` tolerance, zero-fallback), only the two new full-manifold arch tests carrying a widened `120 s` elapsed bound. `fbf_paper_card_house_probe 56,58,59,60,64` remains available as a regression probe (its historical role as the fallback-boundary command is obsolete now that full manifolds solve). The new open frontier is the multi-step full-contact settle: probing `card_house_26_settle_projectile_full` (the full-manifold `fbf_paper_trace` scenario, with its new `warm_start` CLI argument) across steps shows settle steps 2+ reaching 122-138 contacts at per-pair-4, cross-step pair-keyed warm-started, with a `120000`-outer budget, but the exact-FBF solve still falls back to boxed LCP at failed residuals ranging `2.9e-6`-`1.5e-4`. This is a harder, distinct blocker from the one-step manifold-size gap that is now closed; the historical 56/58/59/60/64-contact one-step boundary evidence above remains valid history but no longer describes the current limit | Multi-step full-contact settle convergence (the new blocker above), the paper's 6.7 s no-creep settle outcome, real projectile impacts/outcomes at full contact budget, 10 s post-impact outcome, long-run trace rows, plotted residual convergence at full scale, paper-matched dynamic snapshots, timing parity |
| House of cards, 26-card settle/projectile phase | Reduced-contact two-step scaffold still green; a full-manifold multi-step trace scenario now exists but does not yet converge | `CardHouseFourLevelSettleProjectileScaffoldRuns` still passes at a bounded 16-contact smoke cap with four projectiles launched after one settle step, zero exact-FBF failures, zero boxed-LCP fallback, and final residual at the paper tolerance; `fbf_paper_trace card_house_26_settle_projectile_reduced_contact exact_fbf 1 2 nan phase_summary` still emits initial, settle, and projectile rows at the 56-contact reduced cap (26 cards, 4 projectiles, 56 contacts, 6 exact solves, zero warm starts/fallbacks, residual `4.8919831295901383e-17`, `success`); matching boxed-LCP phase trace and benchmark rows (`..._boxed_lcp`/`..._exact_fbf`, `12057 ms`, max FBF iterations `3631`, total `5539`) are unchanged; GUI scene `fbf_paper_card_house_26` still exposes the Scene-panel `Launch 4 projectiles` control. New: `fbf_paper_trace` now also has a `card_house_26_settle_projectile_full` scenario built on the full natural manifold cap, with a `warm_start` CLI argument to enable cross-step pair-keyed warm starting across the settle/projectile sequence. Multi-step probing of this full scenario is the new Fig. 6 frontier: settle steps 2+ reach 122-138 contacts at per-pair-4, cross-step warm-started, with a `120000`-outer budget, but the exact-FBF solve still falls back to boxed LCP at failed residuals ranging `2.9e-6`-`1.5e-4` rather than converging. A long-run warm-started card-house run using this scenario is in progress but not yet landed | Multi-step full-contact settle convergence (the new blocker), the paper's 6.7 s no-creep settle, real projectile impacts/outcomes at full contact budget, 10 s post-impact outcome, long-run trace rows, paper-matched dynamic snapshots, and timing/external comparisons |
| Masonry arch, 25 stones | Author Rigid-IPC geometry landed; one-step full natural manifold (52 contacts) solves clean; long-run balance/projectile outcome open | The approximate static-endpoint scaffold is replaced by author-faithful geometry in shared `dart/math/detail/MasonryArchGeometry.hpp`, which ports the Rigid-IPC weighted-catenary generator (MIT, `ipc-sim/rigid-ipc` commit `23b6ba6fbf8`, attribution in the header; the earlier gap hack is dropped, base flattening is kept, units converted cm-to-m, `mu=0.5`, density `1000`, all 25 stones dynamic plus a fixed ground). It is wired into fixtures, benchmark, trace, gamma-sweep, and GUI scene builders, the projectile crown constants, and the `fbf_paper_arch_probe` world builder. Under this geometry the natural contact manifold is 52 contacts (both per-pair 4 and 8 produce the same 52; this supersedes the earlier approximate-scaffold's 96-contact number, which belonged to the now-replaced overlapping construction), solving cleanly in `256 ms`/`892` FBF iterations at `1e-6` zero-fallback; the reduced 48/2 probe row is also clean at `302 ms`. New `MasonryArch25FullManifoldOneStepProbe` covers the full-manifold cap (`kArchFullManifoldMaxContacts=512`, `kArchFullManifoldMaxContactsPerPair=4`) mirrored with `kArchOuterRelaxation=1.5` and `kArchMaxOuterIterations=120000` across bench/trace/gamma/GUI. The base-refresh regression in `MasonryArch25ProjectileScaffoldRuns` (dual stall at `4.13e-6`) is fixed by the same relaxation/budget options and passes in `9.9 s`. Full paper-fixture suite is 16/16 in `27.7 s` (was `107.9 s` pre-port); arch captures were refreshed with passing verdicts and visual inspection confirmed the catenary profile; `--verify-fbf-scene-docs` 9/9. A 10 s warm-started balance run is in progress but not yet landed | Long-run 10 s balance/settle outcome under author geometry (run in progress, not yet landed), real projectile/post-impact physical outcome under author geometry, long-run trace rows, plotted residual convergence at full scale, paper-matched snapshots, and timing parity |
| Masonry arch, 101 stones | Author Rigid-IPC geometry landed; one-step full natural manifold (204 contacts) solves clean; long-run balance outcome open | Uses the same author-faithful `MasonryArchGeometry.hpp` port (MIT, `ipc-sim/rigid-ipc` commit `23b6ba6fbf8`) in place of the approximate static-endpoint scaffold. Under this geometry the natural contact manifold is 204 contacts, solving cleanly in `17.2 s`/`28750` FBF iterations at `1e-6` zero-fallback (this supersedes the earlier approximate-scaffold's 512-contact number, which belonged to the now-replaced overlapping construction); the reduced 38/2 probe row is clean at `595 ms`. New `MasonryArch101FullManifoldOneStepProbe` covers the same full-manifold cap (`kArchFullManifoldMaxContacts=512`, `kArchFullManifoldMaxContactsPerPair=4`, `kArchOuterRelaxation=1.5`, `kArchMaxOuterIterations=120000`) mirrored across bench/trace/gamma/GUI. Full paper-fixture suite is 16/16 in `27.7 s` (was `107.9 s` pre-port); GUI scene `fbf_paper_masonry_arch_101` still explains overview, expected result, and limits in the Scene panel; arch captures refreshed with passing verdicts; `--verify-fbf-scene-docs` 9/9. The historical `fbf_paper_arch_probe` cap-boundary evidence (39/2 through 100/2 on the old approximate geometry) is now stale and superseded by the natural-manifold number above. A long-run warm-started balance run under the author geometry has not yet been run | Long-run balance outcome under author geometry (a warm-started run is needed; not yet landed), full-contact physical parity, long-run trace rows, plotted residual convergence at full scale, snapshots, and timing parity |
| House of cards, 10 levels | Construction scaffold only | `CardHouseTenLevelSceneBuilds` builds 155 card skeletons plus ground with the exact solver configured; benchmark row `card_house_10_construction_boxed_lcp` runs one boxed-LCP/default-solver construction step with 155 cards, 512 contacts, zero exact-FBF solves, and `1010 ms` local smoke time; GUI scene `fbf_paper_card_house_10` renders a static construction scaffold and explains that exact-FBF dynamics are intentionally unavailable | Exact-FBF dynamic outcome, residual trace, full timing parity, external baselines, snapshots, and paper 10-level stand/topple comparison |
| Figure 9 residual plots | Partial | Sampled residual guards and CSV traces for small fixtures; `residual_history` CSV scope for backspin step 1, reduced 26-card exact groups, and reduced/author-geometry 25/101-stone arch exact groups; current results, individual SVG plots, and a combined reduced-scaffold residual-history panel were regenerated after the fixture-step updates and are summarized in `residual-history-report.md`; `kResidualHistoryMaxSamples` was raised to `130001` to support full-manifold-scale histories, so one-step full-manifold residual histories are now capturable for card-house (108 contacts), 25-stone arch (52 contacts, author geometry), and 101-stone arch (204 contacts, author geometry); the multi-step card-house settle frontier's failed-residual trajectory (2.9e-6-1.5e-4 range at fallback) is available as diagnostic evidence through the same recording mechanism; history-enabled cap-probe diagnostics still summarize the historical 60-contact one-step failed residual trajectory | Paper-matched plotted comparisons; the full-manifold and settle-frontier histories exist as diagnostic data but have not yet been regenerated into SVG artifacts; long-run/full-contact histories and full house/arch convergence evidence beyond one step |
| Figure 10 step-size sweep | One-step reduced- and full-manifold harness; long-run sweep still open | `fbf_paper_trace` accepts optional `initial_gamma`; `fbf_paper_gamma_sweep` emits CSV rows with gamma, configured `max_contacts`, configured `max_contacts_per_pair`, solver-budget columns, clean-exact flag, physical outcome, residual, fallback, and final-state columns for the small fixtures plus one-step card-house and arch scaffolds; after the full-manifold promotion, gamma-sweep rows now mirror the full-manifold caps (card-house 108/4; 25/101-stone arch under author geometry at 52/204 natural contacts) and agree with the promoted benchmark/trace rows; historical explicit-cap smokes at the earlier reduced caps (card-house `56`, arch `48`/`38`) remain valid as reduced-scaffold evidence | Full-contact house/arch sweep over long-run physical outcomes (not just one step), paper Figure 10 layout, and external comparisons |
| External baselines | Wired (MuJoCo) / probe (Kamino) | `fbf_paper_mujoco_baseline.py` (backspin/incline/turntable plus opt-in `masonry_arch_101_rigid_ipc`) validated end-to-end in a scratch env behind the non-default `fbf-baselines` pixi feature (first `pypi-dependencies` use in `pixi.toml`, flagged for maintainer sign-off; default env untouched): paper `dt=1/60` backspin diverges ~20% with a spurious 11 m bounce while DART exact-FBF asserts the analytic terminal state, and MuJoCo converges only at `dt=1/2000` (0.02%). New `fbf_paper_kamino_baseline.py` is an availability probe only: it skips gracefully without `warp`/`newton`, and records GPU/driver metadata plus an open-integration-task note when they are present (`SolverKamino` verified importable, `newton` 1.3.0/`warp` 1.15.0, CUDA-gated). Paper-code repository rechecked unauthenticated on 2026-07-09 and still not publicly readable (page, API, and `git ls-remote` all fail) | Kamino wired end-to-end on a CUDA device with a hardware-comparable timing row; paper-code comparison if the repository ever becomes available |
| GUI examples | Partial | `dart-demos` Research scenes for small fixtures, two-card precursor, reduced-contact dynamic 26-card scaffold, construction-only 10-level card-house scaffold, and reduced-contact 25/101-stone masonry-arch scaffolds; the shared host now renders each scene's `ScenePanelDocumentation` metadata before custom controls, so FBF overview, expected result, and coverage limits are visible in the Scene tab and not only verifiable in the catalog; `pixi run demos -- --verify-fbf-scene-docs` checks all nine `fbf_paper_*` entries have that metadata; the shared Scene tab scrolls independently so long self-contained explanations and diagnostics remain usable in smaller/captured windows; all nine one-step captures and the contact sheet were regenerated after the host-rendered metadata change and passed default non-blank verdicts; the 26-card and arch scenes use the same caps/options as their tests/benchmarks; the 25-stone arch scene now exposes a reduced projectile launch control and action capture; the 10-level construction scene defaults to boxed-LCP and omits the exact toggle; exact diagnostics show last/max/total FBF iterations, gamma/safe-gamma, gamma scale, outer relaxation, shrink count, coupling-variation ratio, and last failed status/residual when failures occur; pre-solve exact diagnostics now display `not run yet` instead of a misleading initial FBF status; `gui-capture-report.md` records one-step `dart-demos --headless --shot` captures for all nine `fbf_paper_*` scenes, a contact sheet, the 26-card projectile capture, and the 25-stone arch projectile capture with passing default non-blank verdicts | Full-contact dynamic 26-card, paper-parity 25-stone arch, 101-arch, and dynamic 10-level card-house GUI scenes with paper-matched snapshots/overlays if needed |
| 26-card GUI phase scaffold | Reduced GUI control and capture only | `fbf_paper_card_house_26` now has a Scene-panel `Launch 4 projectiles` control and a mirrored `p - Launch 4 projectiles` key action; projectiles share the trace/benchmark scaffold positions and velocity, and the Scene-tab expected/coverage text explains that this is a reduced phase scaffold. `pixi run capture-action fbf_paper_card_house_26 p docs/dev_tasks/fbf_exact_coulomb_friction/assets/gui_captures/fbf_paper_card_house_26_projectiles.png 1280 720 0` produced a visible projectile capture, and `fbf_paper_card_house_26_projectiles.verdict.json` passed the default non-blank gate. `pixi run demos -- --verify-fbf-scene-docs` still checks all nine FBF scenes, and direct `dart-demos --list-scenes` still lists `fbf_paper_card_house_26` under `Research` | Paper-matched dynamic snapshots after real impacts, impact outcome overlays, and full 6.7 s / 10 s visual sequence |
| OSG renderer and GUI widgets | Tracked as needed | Current FBF scenes use `dart-demos` ImGui controls and world-owned visuals; `DemoScene` has `scenePanelDocumentation` metadata and the shared `DemoHost` Scene tab now renders that metadata before scene controls, making the self-containedness contract catalog-verifiable and visible; the shared Scene tab uses a scrollable child region, and FBF diagnostics remain below the reusable explanation block; the Pixi `capture` task accepts an explicit headless step count for bounded GUI smokes; `dart-demos --headless-action <key>` and `pixi run capture-action` now exercise scene key actions before headless captures; an earlier slice added a reusable scene-level control path for construction-only examples where exact-FBF dynamics are unavailable; this slice improved the shared exact-FBF diagnostics widget so unstepped scenes report `Exact diagnostics: not run yet`. No renderer-level OSG scene-graph changes have been required for the current reduced GUI scenes, but renderer, host, or widget changes are explicitly in scope when a paper GUI example cannot otherwise explain itself, expose expected outcome/limits, show useful overlays, capture a stable snapshot, or support robust inspection | Improve the OSG renderer, reusable widgets, camera/snapshot path, overlays, or inspection controls whenever paper-parity GUI examples cannot be self-contained or visually verifiable with the current viewer |
| PR report | Current report added | `PR_REPORT.md` summarizes coverage, GUI commands, benchmark rows, trace commands, verification, and missing rows | Keep commands/results current after every verification run |

## Active Work Board

This board tracks open implementation work. Move or edit rows when progress
changes; do not delete an unfinished row unless its replacement is more
specific.

| Priority | State | Work item | Next concrete evidence |
| --- | --- | --- | --- |
| P0 | Active | Keep task tracker and PR report synchronized with implementation changes | Update this file, `README.md`, `RESUME.md`, `paper-parity-matrix.md`, and `PR_REPORT.md` in the same slice |
| P0 | Done | Make the 26-card four-level house run exact-FBF dynamics at the paper one-step contact budget | The incremental inner solver plus the default-on scratch-backed contact-row operator clear the 60/64-contact one-step boundary; the full natural per-pair-4 manifold (108 contacts) solves clean in `2.5 s` at `8479` max FBF iterations with zero fallbacks, and the promoted benchmark/trace/gamma-sweep rows agree. This closes the one-step manifold-size gap; the multi-step settle frontier below is the follow-on item |
| P0 | Done | Convert the DART integration route from dense prototype toward paper-style matrix-free/scratch-backed products | The scratch-backed `ExactCoulombContactRowOperator` (WP-PG.14 pattern) is default-on via `useContactRowDelassusOperator`, serving `W*x` products, diagonal blocks, and block-column updates from per-row body Jacobians/articulated-inertia solves instead of impulse tests, with automatic fallback for unsupported groups; parity is proven against the impulse-test snapshot. `applyExactCoulombConstraintDelassus(...)`/`useMatrixFreeDelassusOperator` and `useMatrixFreeDelassusSeed` remain available as the earlier impulse-test-based matrix-free route for cases the row operator does not cover. Remaining lower-priority follow-on: removing the dense snapshot from the staging/dense-polish recovery path and hardening the mixed-group policy |
| P0 | In progress | Converge the multi-step full-contact card-house settle (new Fig. 6 blocker) | Probing the full-manifold `card_house_26_settle_projectile_full` `fbf_paper_trace` scenario (with its new `warm_start` CLI argument) across steps shows settle steps 2+ reaching 122-138 contacts at per-pair-4, cross-step pair-keyed warm-started, with a `120000`-outer budget, but the exact-FBF solve still falls back to boxed LCP at failed residuals ranging `2.9e-6`-`1.5e-4`. This is the current honest remaining blocker for Fig. 6, distinct from and harder than the one-step manifold-size gap that is now closed; next evidence is deeper solver/scratch work or a warm-start/step-size change that gets the multi-step failed residual under `1e-6`, not another one-step cap probe |
| P0 | In progress | Extend the 26-card scene to paper Fig. 6 settle/projectile parity | The reduced two-step phase scaffold (`CardHouseFourLevelSettleProjectileScaffoldRuns`, `card_house_26_settle_projectile_reduced_contact`) is unchanged and still green; the new full-manifold `card_house_26_settle_projectile_full` scenario exists but does not yet converge multi-step (see the row above); next evidence is a converged multi-step full-contact settle, then the paper's bounded longer no-creep settle, real projectile impacts/outcomes, dynamic GUI snapshots, long-run full-scene trace, and full-contact benchmark |
| P0 | Open | Replace or corroborate the Painleve proxy | Author scene parameters/files or a recorded source-level reason they cannot be obtained |
| P0 | In progress | Long-run evidence runs (arch balance, card-house settle) | A 10 s 25-stone arch balance run under the author Rigid-IPC geometry, warm-started, is in progress but not yet landed; the 101-stone arch and the full-manifold card-house settle both need their own warm-started long-run runs once the multi-step convergence item above is resolved. Use `card_house_26_settle_projectile_full ... warm_start=1` in `fbf_paper_trace` for the card-house side |
| P1 | Open | 101-stone masonry-arch long-run balance | Needs a warm-started long-run run under the author Rigid-IPC geometry once the 25-stone run above establishes the harness; not yet started |
| P1 | Done | Promote 25-stone masonry arch from reduced scaffold to author-geometry one-step parity | Author-faithful Rigid-IPC geometry (`dart/math/detail/MasonryArchGeometry.hpp`, MIT, commit `23b6ba6fbf8`) replaces the approximate static-endpoint scaffold; the natural manifold (52 contacts) solves clean in `256 ms`; `MasonryArch25FullManifoldOneStepProbe` covers the full-manifold cap. Remaining work is long-run balance/projectile evidence, tracked in the long-run row above, not further geometry work |
| P1 | Done | Promote 101-stone masonry arch from reduced scaffold to author-geometry one-step parity | Same geometry port; the natural manifold (204 contacts) solves clean in `17.2 s`. The old `fbf_paper_arch_probe` cap-boundary evidence (39/2 through 100/2) is now stale and superseded. Remaining work is long-run balance evidence, tracked in the P1 row above |
| P1 | In progress | Promote the 10-level house-of-cards fixture beyond construction scaffold | Current construction scaffold has `CardHouseTenLevelSceneBuilds`, GUI scene `fbf_paper_card_house_10`, and boxed-LCP benchmark row; next evidence is exact-FBF dynamic outcome, residual trace, snapshots, timing, and external baselines |
| P1 | Tracked | Improve OSG renderer and GUI widgets when required by paper-parity examples | The Scene tab now scrolls and renders `ScenePanelDocumentation` generically before scene controls; `pixi run demos -- --verify-fbf-scene-docs` enforces the FBF overview/expected/coverage metadata; `--headless-action` plus `pixi run capture-action` can capture scene states exposed through key actions; the refreshed projectile captures show the 26-card and 25-stone-arch launch scaffolds plus visible Scene-tab explanations; and the exact-FBF diagnostics widget now reports `not run yet` before the first solve. For every new or promoted FBF paper GUI row, first check whether the current viewer can explain the scene, show the expected outcome/limits, capture a stable image, and expose useful diagnostics; add renderer/widget support in this task whenever it cannot |
| P1 | In progress | Regenerate Figure 9/10 artifacts at full-manifold and long-run scale | `residual_history` can now capture one-step full-manifold histories (card-house 108, arch 52/204) and the multi-step settle-frontier failed trajectory, and `fbf_paper_gamma_sweep` rows now mirror the full-manifold caps, but the SVG plots and sweep artifacts checked into this task still reflect the earlier reduced-scaffold caps; next evidence is regenerated SVGs/CSVs at the full-manifold scale, then long-run/full-contact histories once the multi-step settle converges |
| P2 | Done | Wire an optional external comparison harness (MuJoCo) | `fbf_paper_mujoco_baseline.py` validated end-to-end behind the non-default `fbf-baselines` pixi feature; default env untouched |
| P2 | Open | Wire Kamino end-to-end on a CUDA device | `fbf_paper_kamino_baseline.py` is an availability probe only (`SolverKamino` importable, CUDA-gated); paper-code repository still not publicly readable unauthenticated on 2026-07-09 |
| P1 | Active | Final audit before this task can be reported as promotable | Re-verify all touched surfaces (tests/benchmarks/trace/gamma/GUI) together after the multi-step settle and long-run items above land, re-run the full docs/lint/build gate sequence, and re-check every row in `paper-parity-matrix.md` against the current code state before compressing or retiring this dev-task folder |

## Immediate Todo List

- Keep this file, `README.md`, `RESUME.md`, `paper-parity-matrix.md`, and
  `PR_REPORT.md` synchronized when progress changes.
- Update the current status snapshot, active work board, progress ledger, todo
  lists, and verification ledger after every implementation slice.
- Maintain the direct answer that the PR does not yet contain all paper tests,
  benchmarks, and GUI examples.
- Re-run `pixi run demos -- --verify-fbf-scene-docs`, the `dart-demos`
  Research scene catalog, and the one-frame scene factory smoke after any demo
  edits.
- For interactive GUI inspection, use `pixi run demos -- --scene <scene_id>`;
  for bounded visual smoke, use
  `pixi run capture <scene_id> <out.png> 640 480 1` plus
  `pixi run image-verdict <out.png>`.
- For scene states exposed through a GUI/key action, use
  `pixi run capture-action <scene_id> <key> <out.png> <width> <height> <steps>`
  plus `pixi run image-verdict <out.png>`; for the 26-card projectile scaffold
  and the 25-stone masonry-arch projectile scaffold, use key `p`.
- Treat OSG renderer, `dart-demos` host, and reusable ImGui widget
  improvements as in scope when a paper GUI example needs them to be
  self-contained, inspectable, or visually verifiable.
- Do not promote a GUI example as covered unless its `Scene` panel explains
  overview, expected result, and current limitations; if current OSG/ImGui
  surfaces make that impossible, improve those surfaces in the same task.
- For every new or promoted `fbf_paper_*` scene, record whether the existing
  OSG renderer, camera/capture path, and Scene-tab widgets were sufficient. If
  not, implement the needed viewer/widget support before marking the GUI row
  covered.
- Run focused CTest gates for the math, adapter, solver, world smoke, and
  paper fixtures after code changes.
- Run the benchmark list and representative benchmark rows after benchmark
  edits; copy the current command/output summary into `PR_REPORT.md`.
- Run `fbf_paper_card_house_probe 56,58,59,60,64` as a regression probe after
  changing card-house contact caps or exact-FBF options; it no longer defines
  the fallback boundary (full manifolds solve one-step), so use it to catch
  regressions, not to redefine the cap.
- Run `fbf_paper_arch_probe` after changing masonry-arch geometry, contact
  caps, or exact-FBF arch options. The old approximate-scaffold cap-boundary
  rows (25-stone 49/2 through 100/4; 101-stone 39/2 through 100/2) are stale
  under the author Rigid-IPC geometry; prefer the natural-manifold probes
  (`MasonryArch25FullManifoldOneStepProbe`, `MasonryArch101FullManifoldOneStepProbe`)
  for current cap evidence.
- Use `card_house_26_settle_projectile_full ... warm_start=1` in
  `fbf_paper_trace` to probe the multi-step full-contact card-house settle
  frontier; record the per-step contact count and failed/clean residual in
  this tracker after any change that touches multi-step convergence.
- Track long-run warm-started runs (10 s+ arch balance for both stone counts,
  full-contact card-house settle) as they land; do not report a long-run
  outcome as done without a completed run's residual, timing, and physical
  state.
- Regenerate Figure 9 SVGs and Figure 10 sweep CSVs at the full-manifold scale
  after the multi-step settle converges; do not leave the checked-in plots on
  the superseded reduced-scaffold caps once full-manifold histories exist.
- Before reporting this task as promotable, run a final audit pass: re-verify
  every touched surface together, rerun the full docs/lint/build gate
  sequence, and re-check every `paper-parity-matrix.md` row against the
  current code state.
- When changing the Delassus operator route, rebuild every touched executable
  that embeds `ExactCoulombFbfConstraintSolverOptions`; stale benchmark
  binaries can misread the option layout and produce misleading diagnostics.
- When running `fbf_paper_card_house_probe` matrix-free diagnostics, pass the
  final optional `use_matrix_free_delassus_operator` and
  `use_matrix_free_delassus_seed` booleans and record the requested columns
  plus `matrix_free_delassus_operator_used` and
  `matrix_free_delassus_seed_used`.
- Run trace exporter smokes for at least one exact-FBF and one boxed-LCP
  fixture after trace/export edits; include the touched contact-rich scenario
  when the edit is for a house or arch trace.
- Run `fbf_paper_trace ... residual_history` smokes after residual-diagnostic
  edits, and record whether contact-rich rows cover all groups or only the
  last exact group.
- Run `fbf_paper_gamma_sweep` smokes for both a small fixture and any touched
  reduced contact-rich scenario after sweep-helper edits; include explicit
  cap arguments when the edit affects cap reporting or contact-rich rows.
- Run `pixi run docs-build`, `pixi run lint`, `pixi run build`,
  `git diff --check`, and the untracked-file whitespace loop before presenting
  this as PR-ready.

## Recent Negative Results

- The paper residual-scale formula was implemented and kept because it matches
  the source paper's dimensionless stopping criterion instead of using unit
  velocity/impulse scales. It did not clear the 60-contact card-house
  boundary; it made the quasi-static row stricter. The paper-scaled short
  `56,60 ... 3001` diagnostic kept 56 contacts clean but left 60 contacts
  failing at `6.800e-5`. The full 60-contact diagnostic still failed after
  30000 outer iterations with best/final failed residual `5.334e-6`,
  pre-polish history residual `5.338e-6`, and one boxed-LCP fallback.
- The bounded dense residual-polish path was implemented and kept because it
  recovers a focused one-contact failed solve and can improve a failed
  near-converged residual after PG retry. It did not clear the 60-contact
  card-house boundary. A short 3000-outer diagnostic kept 56 contacts clean
  with zero dense polishes and still sent 60 contacts to boxed-LCP fallback
  with one dense polish and failed residual `3.768e-5`. The full 30000-outer
  diagnostic still failed with one dense polish: retained history ended at
  `3.023e-6`, dense polish updated the failed final residual to the best
  residual `3.021e-6`, and the result remained above the paper `1e-6`
  tolerance.
- Best-iterate retention and projected-gradient retry seeding from that
  retained best reaction were implemented and kept because they improve
  diagnostics and prevent retrying from a worse finite failed iterate. They did
  not clear the 60-contact boundary. The refreshed 5000-outer diagnostic showed
  the 60-contact failed group's best residual equals the final residual
  `2.635e-5` at iteration `5000`; the refreshed 30000-outer diagnostic showed
  best failed residual `3.021e-6` at iteration `29355`, final failed residual
  `3.023e-6`, and tail ratio `1.0000019`, with one boxed-LCP fallback.
- A residual-selected projected-gradient refinement of the dense adapter's
  local/global Delassus seed was implemented, built, measured, and reverted.
  It left the promoted 56-contact row clean and slightly reduced the short
  60-contact residual at 5000 outer iterations (`2.499e-5` versus the prior
  `2.635e-5`), but it did not clear the real boundary: the full
  `fbf_paper_card_house_probe 60 1 30000 120 32 200 nan 0 0 0.9 10 1.5 0`
  run still failed after `108381.8 ms` with failed residual `3.024e-6`,
  essentially unchanged from the previous `3.023e-6` and slower. The
  reverted patch's adapter unit test passed after restoration.
- A residual-selected over-relaxation guard was implemented, built, and
  rejected because it made the promoted 56-contact row slower and did not clear
  60 contacts. The rejected run reported 56 contacts clean but at
  `7166.8 ms` with max FBF iterations `1841`, and 60 contacts still failing
  after `113139.0 ms` with failed residual `3.673e-6`.
- A residual-selected depth-one Anderson acceleration experiment was
  implemented, built, measured, and reverted. With coefficient cap `5`, the
  short 3000-outer `56,60` diagnostic made the promoted 56-contact row fall
  back and left 60 contacts failing at `6.407e-5`. With coefficient cap `1`,
  the 56-contact row stayed clean and faster (`1389` max FBF iterations), but
  60 contacts still fell back with failed residual `6.391e-5` and a drifting
  retained tail. This does not clear the 60-contact boundary and should not be
  repeated as a standalone acceleration knob.
- A projected boxed-LCP cold-start experiment was implemented, built,
  measured, and reverted. It made the promoted 56-contact row faster
  (`1165` max FBF iterations) but worsened the 60-contact short diagnostic:
  60 contacts still fell back with failed residual `1.544e-4`. The boxed-LCP
  seed is therefore not a solution path for the contact-rich exact-FBF
  boundary.
- The card-house probe was extended with benchmark-only worst-contact
  residual diagnostics and geometry knobs, then used to test whether the
  60-contact failure is a reduced-manifold artifact. Allowing
  `max_contacts_per_pair=2` at a 60-contact cap still fell back and worsened
  the failed dual residual to `2.366e-4`. Reducing initial penetration to
  `0.001` kept 60 contacts but still failed at `6.771e-5`; zero penetration
  solved only because it generated just 27 contacts. Frame spacing `0.57`
  improved the short failed residual to `4.735e-5` but still fell back with a
  drifting retained tail; spacing `0.53` and `0.59` were worse. Do not promote
  geometry tuning as paper parity without a full-contact, long-run outcome.
- A multi-contact local dense-polish candidate-order experiment was
  implemented, built, measured, and reverted. It tried local dense corrections
  in descending dual-correction magnitude after the global dense polish. The
  focused solver unit test passed, and the short
  `fbf_paper_card_house_probe 56,60 1 3000 120 32 200 nan 0 0 0.9 10 1.5 3001`
  probe kept 56 contacts clean in `4925.2 ms`, but the 60-contact row still
  fell back at the same `6.166e-5` failed residual as the single
  worst-dual-contact local polish. Do not reintroduce the ordered multi-contact
  local polish unless a new residual target or line-search rule gives it
  measurable value.
- A solver-side batched matrix-free Delassus product experiment was
  investigated and reverted in the same slice. The tempting approach was to
  apply the full contact-space input impulse once, update touched skeleton
  velocity changes once, and gather all contact velocities. The real
  `ContactConstraint` path needs bias-impulse propagation after
  `applyImpulse()`, and generic `ConstraintBase` does not expose the touched
  body/skeleton data needed to do that safely as a local solver-only patch. A
  one-contact `World` smoke fell back with residual near `101`, so the
  experiment was removed. Do not reattempt this as a local solver-only patch;
  either add a deliberate dynamics/constraint API for scratch-backed products
  or keep using the proven rowwise impulse-test route.
- Adding the opt-in constraint-row Delassus product route did not change the
  promoted card-house boundary in the default dense-snapshot configuration.
  The post-rebuild short probe kept 56 contacts clean and left 60 contacts
  falling back at `6.166e-5`; this is expected route-staging evidence, not a
  contact-rich convergence fix.
- Exposing the opt-in constraint-row Delassus product route in
  `fbf_paper_card_house_probe` shows the route is correctness-compatible on
  bounded reduced card-house rows but currently too slow to promote as a
  performance path. With `max_outer_iterations=1000`, 16 contacts solved in
  `24.4 ms` dense versus `324.2 ms` matrix-free; 32 contacts solved in
  `170.4 ms` dense versus `3203.0 ms` matrix-free; and 40 contacts failed the
  capped row at the same `3.966e-5` residual but took `1057.4 ms` dense versus
  `18937.8 ms` matrix-free. Do not treat this impulse-test operator as the
  final paper-style matrix-free implementation; it needs scratch-backed body
  scatter/inverse-mass/gather or equivalent reuse.
- Short 60-contact, 5000-outer-iteration diagnostics after reverting that
  guard show the current default remains the best tested direction:

| Variant | Failed residual at 5000 | Tail ratio | Note |
| --- | ---: | ---: | --- |
| Default: PG 200, sweeps/local `120`/`32`, scale `10`, relaxation `1.5` | `2.635e-5` | `0.9995126` | Current best short-cap row |
| Step-size scale `12` | `3.837e-5` | `1.0075026` | Worse |
| Step-size scale `8` | `5.964e-5` | `0.9996260` | Worse |
| Outer relaxation `1.0` | `6.072e-5` | `1.0005204` | Worse |
| Outer relaxation `1.8` | `5.805e-5` | `0.7119379` | Worse final residual despite a lower tail ratio |
| Projected-gradient retry iterations `1000` | `2.635e-5` | `0.9995126` | Same as default |
| Block-GS sweeps/local `240`/`64` | `2.635e-5` | `0.9995126` | Same as default |

These are not completion evidence. They only rule out repeating one-knob
option probes as the next solution path; the 60/64-contact blocker still needs
deeper solver, scratch, or matrix-free work.

## Verification Ledger

Update this table after the last edit in each slice. If a command is too slow
or blocked, record the exact blocker and the next command instead of implying
success.

| Gate | Last recorded result | Date | Notes |
| --- | --- | --- | --- |
| 25-stone author-arch long-run balance attempt | Collapse recorded; outcome gated on the settle frontier | 2026-07-11 | `fbf_paper_trace masonry_arch_25_full_manifold exact_fbf 30 600 nan tracked 1` (warm starts on); CSV preserved at `assets/arch25_balance_partial_collapse.csv`. Step 30 (0.5 s): crown upright (`up_z ~= 1.0`) at `z = 0.578` (from the built `0.607`) but sinking at `0.30 m/s`, 84 contacts, 2 boxed-LCP fallbacks; step 60 (1.0 s): crown toppled (`up_z = 0.115`) at `z = 0.240`, 120 contacts, 8 fallbacks. Cross-step manifold warm starts functioned (27/28 and 71/74 solves warm-started). Honest reading: this does NOT test the paper's claim yet — the sequence was contaminated by boxed-LCP fallback steps (the same multi-step convergence frontier), and initial seating dynamics (formula-exact stone faces plus the 0.001 m ground gap) inject settle energy. Fig. 7/8 long-run balance therefore requires fallback-free multi-step solving first; the ERP/position-bias hypothesis is strengthened. Run stopped after collapse was unambiguous |
| Fig. 7/8 arch standing outcome | Characterized: seating-transient slump, solver healthy; standing outcome needs equilibrium initialization | 2026-07-11 | Five controlled long-run experiments on the author-geometry 25-stone arch under split impulse, each with preserved trajectories: (1) ERP on: the position pass inflates the exact-face voussoirs apart at the error-reduction rate (crown +3.4 cm in 0.5 s) and the arch collapses by 1.5 s (`assets/arch25_alldynamic_seating_collapse.csv`); (2) ERP off: no inflation, but the 1 mm seating drop's chatter breaks the mu 0.5 friction lock and the arch slumps (crown 0.607 to 0.160) then holds PERFECTLY static (z constant to `1e-7` over 90+ steps); (3) 0.5 percent seating pre-stress: worse (stronger push-apart); (4) 0.2 mm ground pre-seat: no change; (5) paper-described pinned springers: interior still slumps (crown to 0.105 by 1 s) then holds static at drift `+3e-7`/step (`assets/arch25_pinned_springers_5s.csv`). Solver health is unambiguous throughout: every configuration solves at `1e-6` (0-5 fallbacks only at the hardest seating-chatter steps) and post-slump equilibria are held with near-zero drift for hundreds of steps. Conclusion: reproducing the paper's STANDING catenary in DART requires equilibrium initialization (quasistatic pre-solve or kinematic seating) or persistent face-face contact manifolds - DART scene/collision infrastructure work, not exact-FBF solver capability. The trace tool's arch scenarios permanently document the current best configuration (split impulse, ERP zero, pinned springers per the paper's Fig. 7 description). Context from `paper-notes.md`: the paper itself diagnoses arches as its main remaining limitation (the outer FBF correction can lose contractivity on globally coupled contact graphs such as arches), so the DART arch findings align with the source's own admitted weak spot rather than contradicting a solved case |
| Fig. 6 full 10 s settle/projectile sequence | COMPLETED end-to-end | 2026-07-11 | `fbf_paper_trace card_house_26_settle_projectile_full exact_fbf 30 600 nan phase_summary` on the fixed base; CSV preserved at `assets/card_house_full_sequence_10s.csv`. Settle phase (0-6.7 s, 402 steps): zero fallbacks, all-exact at `1e-6`, ~3 mm total compaction — the paper's no-visible-creep outcome. Four projectiles launch at 6.7 s; impact phase (6.7-10 s): localized failure exactly as the paper describes — cards knocked over (max travel `1.9 m` by 10 s, one card axis flipped to `-0.99`) while the house core keeps standing (min card center height `0.416` at 10 s). Full-sequence solver totals: `1617` exact solves, `1613` warm-started (99.75%), `24` boxed-LCP fallbacks, all concentrated in the violent post-impact chaos steps (fallbacks were zero through the entire settle). Final sampled step residual `6.96e-7` |
| Multi-step full-contact settle frontier | RESOLVED (two-part fix, both validated) | 2026-07-11 | Part 1 (convergence): the stall's root cause was DART's ERP position-correction bias inside the velocity-phase right-hand side (`v_free = -b`); running the exact path under split impulse removes it. `ExactCoulombFbfConstraintSolver` now forwards `isSplitImpulseEnabled()` into the adapter build, and the long-run trace scenarios enable split impulse (set AFTER `World::setConstraintSolver`, which copies the previous solver's flag — setting it before installation silently no-ops). Part 2 (base bug found and fixed): DART's split-impulse position pass assembled its LCP with unit-impulse tests whose `clearConstraintImpulses()` wiped the velocity-phase constraint impulses before `World::step` integrates them, so resting bodies free-fell and tunneled while contacts persisted (minimal repro: a single sphere on a plane under the plain boxed solver). Fixed by saving/restoring body and joint constraint impulses around the position pass in `ConstraintSolver::solvePositionConstrainedGroups`; base regression `ConstraintSolver.SplitImpulsePreservesVelocityPhaseContactResponse` and fixture `CardHouseFourLevelFullManifoldSplitImpulseSettleProbe` (10 full-manifold settle steps, `120000` outer budget, warm starts) both green. Validated together: 5 settle steps all-exact at `1e-6`, zero fallbacks, minimum card height drift below `0.25 mm` (genuine no-creep), ~`1.4 s`/step (was ~2 min/failing step). The base fix is a DART bugfix — the dual-PR rule applies when publishing. `fbf_paper_trace` also gained a `split_impulse=default\|0\|1` CLI argument, and the boxed baseline uses the same stepping mode for the long-run scenarios |
| Final gate battery after review-pass-2 fixes and the docs sync | Passed | 2026-07-11 | `pixi run lint` clean; full FBF target build clean; focused 16-gate CTest set `16/16` in `28.9 s` (post arch-port, hardening tests, pair-keyed warm-start multi-island fix, and pointer-cache-only failure invalidation); `git diff --check` clean; `pixi run demos -- --verify-fbf-scene-docs` 9/9; `pixi run docs-build` succeeds. Review pass 2 findings applied: multi-island warm-start records now survive another group's failure (`invalidateExactCoulombWarmStartPointerCache`), refresh-boundary test margin documented at the measured 36 sweeps with a `>16` assertion, and the Kamino probe skips cleanly on warp runtime failures. The 10 s warm-started 25-stone-arch balance trace is running as a long-running artifact job |
| Full 16-gate focused CTest set after the full-manifold promotion | Passed `16/16` in `167 s` | 2026-07-09 | Independent run after reviewing the five-surface promotion (card house 512/4 with 108 actual contacts; new `MasonryArch25FullManifoldOneStepProbe` 96 contacts and `MasonryArch101FullManifoldOneStepProbe` 512 contacts; arch options `120000`/`1.5` mirrored to bench/trace/gamma/GUI; `kResidualHistoryMaxSamples` raised to `130001`); assertions verified un-weakened (`1e-6` tolerance, zero-fallback), only the two new full-manifold tests carry a widened `120 s` elapsed bound |
| Promoted card-house full-manifold rows | Clean | 2026-07-09 | Benchmark `card_house_26_reduced_contact_exact_fbf` `1867 ms`, 108 contacts, max FBF iterations `8479`, max residual `9.995e-7`, zero failures/fallbacks; trace and gamma-sweep rows agree; `pixi run demos -- --verify-fbf-scene-docs` checked 9 scenes; `pixi run lint` clean |
| Incremental inner solver + contact-row operator unit/parity gates | Passed | 2026-07-09 | `UNIT_math_ExactCoulombFbfSolver` (22 tests incl. block-column overload parity and cached-block mismatch rejection), `ConstraintSolver.ExactCoulombContactRowOperatorMatchesImpulseTests`, `ExactCoulombFbfWorldSmokeUsesContactRowOperator`, `ExactCoulombFbfWorldSmokeManifoldWarmStartAcrossSteps` all green |
| Rigid-IPC author-geometry arch port | Passed, all surfaces | 2026-07-10 | New shared `dart/math/detail/MasonryArchGeometry.hpp` ports the Rigid-IPC weighted-catenary generator (MIT, commit `23b6ba6fbf8`, attribution in the header; gap hack dropped, base flattening kept, cm-to-m, mu `0.5`, density `1000`, ALL stones dynamic plus fixed ground). Wired into fixtures/bench/trace/gamma by the geometry worker and finished (GUI scene builders, projectile crown constants, `fbf_paper_arch_probe` world builder, Scene-tab text) after that worker hit its session limit. Full fixture suite 16/16 in `27.7 s` (was `107.9 s`); author-geometry natural manifolds: 25-stone `52` contacts clean in `256 ms` / `892` iterations, 101-stone `204` contacts clean in `17.2 s` / `28750` iterations, both `1e-6` zero-fallback (the earlier 96/512 numbers belonged to the replaced overlapping scaffold); reduced 48/2 and 38/2 rows clean in `302 ms` / `595 ms`; `--verify-fbf-scene-docs` 9/9; arch captures refreshed with passing verdicts and visual inspection of the catenary profile; one-second full-manifold trace run emits success rows (settle compaction visible, long-run outcome is the packet-D scope) |
| MuJoCo/Kamino baseline harness | Wired benchmark-only | 2026-07-10 | `fbf_paper_mujoco_baseline.py` (backspin/incline/turntable plus opt-in `masonry_arch_101_rigid_ipc`) validated end-to-end in a scratch env: paper `dt=1/60` backspin diverges ~20% with a spurious 11 m bounce while DART exact-FBF asserts the analytic terminal state; MuJoCo converges at `dt=1/2000` (0.02%). New `fbf_paper_kamino_baseline.py` availability probe tested on both paths (graceful skip without warp/newton; records GPU/driver metadata and the open-integration-task note when present — SolverKamino verified importable, newton 1.3.0/warp 1.15.0). Optional non-default `fbf-baselines` pixi feature/env added (first `pypi-dependencies` use in `pixi.toml`, flagged for maintainer sign-off); default env untouched |
| Independent adversarial review, pass 1 (solver changes) | No blockers, no majors | 2026-07-09 | Independent reviewer hand-derived the incremental block-GS gradient algebra (exact, not approximate), cross-validated the 16-sweep refresh cadence with a standalone 60-contact chain-coupled stress probe matching a projected-gradient reference to `2.5e-13`, verified row-operator semantics against `solveMatrixFreeContactGroup` (incl. B-only-reactive, double-shared-body, LDLT-failure, typeid gates), confirmed flipped-pair warm-start negation against DART's contact-Jacobian sign convention, confirmed fallback-path flag/ordering equivalence and no read-before-fill path for the deferred snapshot, and classified the dangling-`BodyNode*`-comparison risk as convergence-speed-only (residual-gated success, pointers compared never dereferenced). Five minor test-coverage notes accepted; the three recommended hardening tests (refresh-boundary stress, flipped-pair warm start, multi-contact/count-change warm start) were commissioned in the same session. Second clean pass to run on the post-hardening state |
| Focused FBF CMake targets | Passed | 2026-07-09 | After adding the operator-local Delassus seed and reverting the unsafe batched-product experiment, `pixi run build && pixi run cmake --build build/default/cpp/Release --target test_ExactCoulombConstraintAdapter test_ExactCoulombFbfConstraintSolver test_ConstraintSolver fbf_paper_card_house_probe --parallel 8` passed; after tightening the warm-start-vs-seed diagnostic, `pixi run cmake --build build/default/cpp/Release --target test_ExactCoulombFbfConstraintSolver --parallel 8` also passed |
| Focused CTest regex | Passed `3/3` | 2026-07-09 | `ctest --test-dir build/default/cpp/Release -R '(test_ExactCoulombConstraintAdapter\|test_ExactCoulombFbfConstraintSolver\|test_ConstraintSolver)' --output-on-failure` passed after the operator-local Delassus seed, batched-product rollback, and warm-start diagnostic test |
| Full paper-fixture CTest | Passed | 2026-07-09 | `ctest --test-dir build/default/cpp/Release -R test_ExactCoulombFbfPaperFixtures --output-on-failure` passed after final fixture-step updates; prior failure modes were incline threshold fallbacks and the paper-scaled 101-stone reduced arch row |
| Reduced 25-stone arch XML probe | Passed | 2026-07-09 | `MasonryArch25OneStepReducedContactProbe`: `max_contacts=48`, `max_contacts_per_pair=2`, `contacts=48`, `step_size_scale=10`, `elapsed_ms=6693.7547709999999`, residual `9.9650461738801354e-07`, one exact solve, zero exact-FBF failures, zero boxed-LCP fallback, max/total FBF iterations `2645`; a 49/2 gamma-sweep diagnostic still produced only 48 actual contacts with the current scaffold |
| Reduced 25-stone arch trace smokes | Passed | 2026-07-09 | Exact-FBF and boxed-LCP `fbf_paper_trace masonry_arch_25_reduced_contact ... 1` commands emitted crown-body rows for `masonry_arch_stone_12_body`; exact-FBF row reported 48 contacts, one exact solve, zero fallbacks, residual `9.9650461738801354e-07`, and `success`; residual-history scope ends at iteration `2645` with 2646 data rows |
| Reduced 25-stone arch projectile scaffold | Passed as reduced scaffold | 2026-07-09 | Added and built `MasonryArch25ProjectileScaffoldRuns`, `masonry_arch_25_projectile_reduced_contact`, benchmark rows, and GUI action state. After `pixi run lint`, `./build/default/cpp/Release/tests/integration/test_ExactCoulombFbfPaperFixtures --gtest_filter=ExactCoulombFbfPaperFixtures.MasonryArch25ProjectileScaffoldRuns` passed in `10611 ms`. Exact-FBF tracked trace ended with projectile body position `(2.3658678860997049e-05, -0.24838572818678054, 1.1973289982365207)`, 48 contacts, one exact solve, zero fallbacks, residual `9.9901843912475669e-07`, and `success`; boxed-LCP tracked trace emitted the same scenario for comparison. Dynamic-body trace emitted 23 mobile stones plus the projectile at step 1. Post-lint focused benchmark filter `masonry_arch_25_projectile_reduced_contact_(boxed_lcp\|exact_fbf)$` reported boxed LCP `0.497 ms` and exact FBF `4921 ms`, one exact solve, zero failures/fallbacks, max residual `999.018n`, and `2205` max/total FBF iterations. `pixi run demos -- --verify-fbf-scene-docs` checked all nine FBF scenes, `pixi run demos -- --list-scenes` listed `fbf_paper_masonry_arch_25` under Research, and `pixi run capture-action fbf_paper_masonry_arch_25 p ...` plus `pixi run image-verdict` produced `assets/fbf_paper_masonry_arch_25_projectile.png` with `pass: true`; visual inspection showed the projectile, Scene-tab explanation, and `Exact diagnostics: not run yet`. This row does not prove the paper's real projectile outcome |
| Reduced 101-stone arch XML probe | Passed | 2026-07-09 | `MasonryArch101OneStepReducedContactProbe`: `max_contacts=38`, `max_contacts_per_pair=2`, `contacts=38`, `step_size_scale=10`, `elapsed_ms=2458.3270210000001`, residual `9.9001648754006507e-07`, one exact solve, zero exact-FBF failures, zero boxed-LCP fallback, max/total FBF iterations `3912`; higher one-step cap evidence now lives in `fbf_paper_arch_probe` |
| Reduced 101-stone arch trace smokes | Passed | 2026-07-09 | Exact-FBF and boxed-LCP `fbf_paper_trace masonry_arch_101_reduced_contact ... 1` commands emitted crown-body rows for `masonry_arch_stone_50_body`; exact-FBF row reported 38 contacts, one exact solve, zero fallbacks, residual `9.9001648754006507e-07`, and `success`; residual-history scope ends at iteration `3912` with 3913 data rows |
| Masonry-arch contact-cap probe | Passed/blocked boundary recorded | 2026-07-09 | Added and built `fbf_paper_arch_probe` with `pixi run cmake --build build/default/cpp/Release --target fbf_paper_arch_probe --parallel 8`. Default `pixi run build/default/cpp/Release/tests/benchmark/integration/fbf_paper_arch_probe` emitted 25-stone 48/2 and 101-stone 38/2 success rows. `arch25 48,49 2 20000` confirmed 49/2 still generates only 48 actual contacts. `arch25 64 4 20000` solved 64 contacts in `31118.9 ms`; `timeout 120s ... arch25 80 4 20000` solved 80 contacts in `117628.4 ms`; earlier `timeout 120s ... arch25 100 4 20000` exited `124` before a row. `arch101 38,39 2 20000`, `arch101 40,48 2 20000`, `arch101 56,64 2 20000`, `timeout 120s ... arch101 80 2 20000`, and `timeout 120s ... arch101 100 2 20000` all solved without fallback; the 100/2 row reported 100 contacts, `56137.1 ms`, residual `9.9444848815471616e-07`, 6892 FBF iterations, one exact solve, zero failures, and zero boxed-LCP fallbacks |
| 10-level card-house scene build | Passed | 2026-07-08 | `CardHouseTenLevelSceneBuilds` built 155 card skeletons plus ground with exact solver configured, `max_contacts=512`, and `max_contacts_per_pair=8` |
| Benchmark list/all-row smoke | Passed | 2026-07-09 | Benchmark list now has 29 rows after adding the reduced 26-card settle/projectile and reduced 25-stone arch projectile boxed/exact rows; all pre-existing rows ran at 21:00:43 -0700 on 2026-07-08, then focused changed exact-FBF rows were rerun after final fixture-step updates: incline stick `21.7 ms`, 3 contacts, 120 exact solves, zero failures/fallbacks, max/total FBF iterations `382`/`1917`, max residual `999.835n`; incline slide `3.62 ms`, max/total FBF iterations `59`/`433`, max residual `999.142n`; 56-contact card-house `5028 ms`, 3 exact solves, zero failures/fallbacks, max/total FBF iterations `1908`/`1908`, max residual `221.834a`; 25-stone arch `6514 ms`, max/total FBF iterations `2645`/`2645`, max residual `996.505n`; 101-stone arch `2365 ms`, max/total FBF iterations `3912`/`3912`, max residual `990.016n`; the earlier 10-level construction boxed row reported 155 cards, 512 contacts, and `1010 ms`; the two-step phase benchmark rerun on 2026-07-09 reported boxed LCP `2.17 ms` and exact FBF `12057 ms`, 56 contacts, 6 exact solves, zero failures/fallbacks, max FBF iterations `3631`, total FBF iterations `5539`, and max residual `221.834a`; the post-lint reduced 25-stone arch projectile benchmark rows report boxed LCP `0.497 ms` and exact FBF `4921 ms`, 48 contacts, one exact solve, zero failures/fallbacks, max residual `999.018n`, one projectile, and `2205` max/total FBF iterations |
| Backspin trace smokes | Passed | 2026-07-08 | Exact FBF, boxed LCP, and explicit `initial_gamma=0.05` CSV traces emitted current rows; explicit gamma `0.05` stayed clean with zero fallbacks |
| Gamma sweep smokes | Passed | 2026-07-09 | Final default sweep rows after fixture-step updates: `incline_mu_0_5 nan 120` clean, `stick_like`, `step_size_scale=2`, max residual `9.998e-7`, zero fallbacks; `incline_mu_0_4 nan 120` clean, `slide_like`, max residual `9.991e-7`, zero fallbacks; `masonry_arch_25_reduced_contact nan 1` clean at 48/2, `step_size_scale=10`, residual `9.9650461738801354e-07`; `masonry_arch_101_reduced_contact nan 1` clean at 38/2, `step_size_scale=10`, residual `9.9001648754006507e-07`; `card_house_26_reduced_contact nan 1` clean at 56/1, `step_size_scale=10`, `outer_relaxation=1.5`, residual `2.2183427389532158e-16`; higher arch cap evidence was refreshed in `fbf_paper_arch_probe` after these sweep smokes |
| Paper-code availability check | Still blocked | 2026-07-09 | `curl -I -L https://github.com/matthcsong/fbf-sca-2026` returned HTTP 404, and unauthenticated `GIT_TERMINAL_PROMPT=0 git ls-remote https://github.com/matthcsong/fbf-sca-2026.git` could not read the repository; do not wire paper-code comparison until it is publicly readable or credentials are explicitly provided |
| Card-house contact-cap probe | Passed/blocked boundary recorded | 2026-07-09 | After paper residual-scale updates, `fbf_paper_card_house_probe 56,60 1 3000 120 32 200 nan 0 0 0.9 10 1.5 3001` kept 56 clean in `4971.0 ms` with 3 exact solves, zero PG retries, zero dense polishes, paper-scaled residual `2.218e-16`, and 1911 history rows; 60 still failed in `12867.2 ms` with one exact failure, one boxed-LCP fallback, one PG retry, one dense polish, failed best/final residual `6.800e-5`, and pre-polish history last residual `6.899e-5`. After reverting rejected Anderson/boxed-seed experiments, the same command preserved the baseline shape: 56 clean in `4998.7 ms`, 60 still failed in `12393.7 ms` with failed best/final residual `6.800e-5`. After adding worst-contact residual diagnostics, the same command passed as a diagnostic and reported 56 clean in `5011.7 ms`; 60 still failed in `12632.4 ms` with failed dual dominated by contact `13` (`card_house_l0_support0_body\|card_house_l0_f1_left_body`) and failed complementarity dominated by contact `2` (`BodyNode\|card_house_l0_f1_left_body`). After appending optional geometry columns, the old command shape still worked: 56 clean in `4873.7 ms`, 60 failed in `12398.2 ms`, with defaults `initial_penetration=0.003` and `frame_spacing=0.55`. Geometry/cap diagnostics did not clear the row: `60 2 ... 3001` failed at `2.366e-4`; `initial_penetration=0.001` failed at `6.771e-5`; zero penetration generated only 27 contacts; spacing `0.57` failed at `4.735e-5`, while `0.53` and `0.59` failed at `9.048e-5` and `8.829e-5`. After adding residual-selected local dense polish, the default short row kept 56 clean in `4889.8 ms` and improved the 60-contact failed residual to `6.166e-5` while still falling back; a `10000`-outer diagnostic improved to `2.079e-5` but still fell back. A multi-contact local dense-polish candidate-order experiment also kept 56 clean but did not improve 60 (`6.166e-5`), so it was reverted. After adding the matrix-free probe columns, the standard dense command stayed compatible: 56 clean in `4874.2 ms` with `use_matrix_free_delassus_operator=0` and `matrix_free_delassus_operator_used=0`, while 60 fell back in `12523.0 ms` with one PG retry, one dense polish, failed residual `6.166e-5`, failed dual contact `38`, and failed complementarity contact `2`. Matrix-free route diagnostics with `max_outer_iterations=1000` showed 16 contacts clean in `324.2 ms` vs `24.4 ms` dense, 32 contacts clean in `3203.0 ms` vs `170.4 ms` dense, and 40 contacts failing the capped row with the same `3.966e-5` residual in `18937.8 ms` vs `1057.4 ms` dense; `matrix_free_delassus_operator_used=1` on those matrix-free rows. The paper-scaled `58,59 1 30000 ... 0` rows remained clean but too slow: 58 in `68879.1 ms`, 59 in `76402.9 ms`, each with one PG retry and zero dense polishes. The full `fbf_paper_card_house_probe 60 1 30000 120 32 200 nan 0 0 0.9 10 1.5 30001` row failed in `94606.3 ms` with one boxed-LCP fallback, one PG retry, one dense polish, 30001 failed-history rows, first residual `1.146e-5`, best failed residual `5.334e-6` at iteration `29355`, dense-polished final failed residual `5.334e-6`, pre-polish history last residual `5.338e-6`, and tail ratio `1.0000019`; 64 not cleared |
| Matrix-free seed diagnostic | Passed for seed toggle, blocked for larger product rows | 2026-07-09 | Final-code `fbf_paper_card_house_probe 16 1 30000 120 32 200 nan 0 0 0.9 10 1.5 0 0.003 0.55 1 0` and the same command with trailing `1 1` both solved exactly with residual `3.212e-16`, 16 contacts, 5 exact solves, zero failures/fallbacks, and 336 total FBF iterations; the CSV showed `matrix_free_delassus_seed_used=0` for the first row and `1` for the second, with elapsed `354.1 ms` versus `354.5 ms`. Attempts to run `32,40` and `56,60` matrix-free product rows under a 120 s timeout produced no first CSV row, confirming product cost remains the blocker |
| 26-card settle/projectile scaffold | Passed as reduced scaffold | 2026-07-09 | `CardHouseFourLevelSettleProjectileScaffoldRuns` passed in `78 ms` at a 16-contact smoke cap with four projectiles launched after one settle step. `fbf_paper_trace card_house_26_settle_projectile_reduced_contact exact_fbf 1 2 nan phase_summary` emitted initial/settle/projectile rows; the final projectile row reported 26 cards, 4 projectiles, finite state, 56 contacts, 6 exact solves, zero fallbacks, residual `4.8919831295901383e-17`, and `success`. The boxed-LCP phase-summary trace also emitted the three rows. This is not a 6.7 s no-creep or impact-outcome parity run |
| Reduced 26-card trace smokes | Passed | 2026-07-09 | After the final fixture-step updates, exact-FBF `fbf_paper_trace card_house_26_reduced_contact ... 1` emits the top-card row for `card_house_l3_f0_left_body` with 56 contacts, 3 exact solves, zero fallbacks, residual `2.218e-16`, and `success`; boxed-LCP smoke still exists from the previous trace slice |
| Contact-rich full-scene trace smokes | Passed | 2026-07-09 | `fbf_paper_trace ... 1 1 nan dynamic_bodies` emitted step-1 mobile-body rows: 26 rows for reduced 26-card exact-FBF/boxed-LCP, 23 rows for reduced 25-stone exact-FBF, and 99 rows for reduced 101-stone exact-FBF; after the final fixture-step updates, the reduced 26-card exact row reports residual `2.218e-16`, while the latest arch exact rows report 48 and 38 contacts with residuals `9.9650461738801354e-07` and `9.9001648754006507e-07` |
| Residual-history trace smokes | Passed | 2026-07-09 | Regenerated CSV/SVG artifacts from the current trace executable: `fbf_paper_trace backspin exact_fbf 1 1 nan residual_history` emitted 18 data rows for one exact solve, ending at iteration `17` with residual `7.8906612729366679e-07`; `card_house_26_reduced_contact exact_fbf 1 1 nan residual_history` emitted 1911 data rows across 3 exact groups, with solve 0 ending at iteration `1908` and residual `9.943369051914846e-07`, and solve 2 ending with residual `2.2183427389532158e-16`; `masonry_arch_25_reduced_contact exact_fbf 1 1 nan residual_history` emitted 2646 data rows for one 48-contact group, residual `9.9650461738801354e-07`; `masonry_arch_101_reduced_contact exact_fbf 1 1 nan residual_history` emitted 3913 data rows for one 38-contact group, residual `9.9001648754006507e-07`; all reported zero fallbacks and `success`; individual SVGs and `assets/fbf_residual_history_panel.svg` were regenerated |
| Demo catalog/factory smoke | Passed | 2026-07-09 | After the host-level Scene-tab documentation renderer change and the 26-card phase-control update, `pixi run demos -- --verify-fbf-scene-docs` rebuilt `dart-demos` and checked all 9 FBF paper scenes. Direct `./build/default/cpp/Release/bin/dart-demos --list-scenes \| rg "fbf_paper_card_house_26\|Research"` confirmed `fbf_paper_card_house_26` under `Research`; previous `pixi run demos -- --cycle-scenes --frames 1` cycled 43 demos x2 with only pre-existing non-FBF warnings. A `pixi run demos -- --list-scenes \| rg ...` wrapper probe returned no grep match in this slice, so the direct binary check is the recorded catalog evidence. Do not run two `pixi run demos` invocations in parallel because their target builds can race on `libdart-demos-scenes.a` |
| FBF GUI headless captures | Passed | 2026-07-09 | After the host-level metadata renderer update, regenerated all nine `fbf_paper_*` scenes with `dart-demos --headless --shot ... --steps 1 --width 640 --height 480`, generated `assets/fbf_gui_capture_sheet.png`, and wrote one default `image_verdict.py` JSON per scene; all nine verdicts passed the default non-blank gate and the sheet shows the Scene tab visible for each row. A focused `pixi run capture fbf_paper_backspin /tmp/fbf_paper_backspin.png 640 480 5` plus `pixi run image-verdict /tmp/fbf_paper_backspin.png` also passed. An earlier simultaneous `demos`/`capture` invocation raced while archiving `dart-demos` and passed when rerun alone |
| 26-card GUI action capture | Passed | 2026-07-09 | `pixi run capture-action fbf_paper_card_house_26 p docs/dev_tasks/fbf_exact_coulomb_friction/assets/gui_captures/fbf_paper_card_house_26_projectiles.png 1280 720 0` invoked the scene key action before capture, wrote a visible four-projectile phase-scaffold screenshot, and `pixi run image-verdict .../fbf_paper_card_house_26_projectiles.png` passed the default non-blank gate. The saved verdict JSON records `pass: true`; contrast is not required and remains weak. An earlier parallel `pixi run demos -- --verify-fbf-scene-docs` plus `pixi run capture-action ...` reproduced the known `dart-demos` archive/link race, and both commands passed when rerun serially |
| `pixi run docs-build` | Passed | 2026-07-09 | Reran after the reduced 25-stone arch projectile scaffold, GUI-widget/report/tracker/resume updates, and screenshot refresh; Sphinx reported `build succeeded` |
| `pixi run lint` | Passed | 2026-07-09 | Reran after the reduced 25-stone arch projectile scaffold and report/resume/tracker updates; formatter/codespell completed cleanly |
| `pixi run build` | Passed | 2026-07-09 | Reran after lint; target `all` completed |
| `git diff --check` plus untracked whitespace loop | Passed | 2026-07-09 | `git diff --check` covered tracked edits; a binary-skipping `git ls-files --others --exclude-standard` loop checked untracked text files. The first zsh loop used `path` and clobbered `PATH`; the corrected loop used `f` and passed |
| Arch-probe final docs/lint/build gates | Passed | 2026-07-09 | After adding `fbf_paper_arch_probe` and updating the tracker, README, resume, parity matrix, and PR report, `pixi run docs-build`, `pixi run lint`, `pixi run build`, `git diff --check`, and the binary-skipping untracked-file whitespace loop passed |

Command note: quote benchmark filters that contain parentheses when running
under zsh. The unquoted representative benchmark filter failed locally with
`no matches found`; the quoted command recorded in `PR_REPORT.md` passed.

## Next Paper-Parity Todo List

- Replace or corroborate the Painleve proxy with the authors' exact scene once
  the implementation or parameter files are available.
- Add full parameter sweeps, visual snapshots, trajectory plots, and timing
  reports for incline, backspin, turntable, and Painleve.
- Remove the 60-contact and 64-contact one-step exact-FBF `MaxIterations`
  failures and boxed-LCP fallback without relying on near-cap runtimes. The
  59-contact diagnostic can now solve without fallback but is too slow to be a
  promoted default rung; continue raising the cap toward the full contact
  budget only after the 60-contact path is bounded.
- Promote the 25-stone arch beyond the current reduced scaffold by
  adding author/Rigid-IPC geometry, pinned/projectile outcome checks,
  long-run trace rows, snapshots, and timing; 49/2 still yields only 48 actual
  contacts with the current scaffold, 80/4 is clean but takes about 118 s, and
  100/4 timed out under a 120 s guard.
- Promote the 101-stone arch beyond the current reduced scaffold by
  adding author/Rigid-IPC geometry, long-run balance checks, long-run trace
  rows, snapshots, and timing; the approximate one-step probe now solves
  100/2, but this is not long-run balance or paper geometry parity.
- Promote the 10-level card-house scaffold from construction/default-solver
  evidence to exact-FBF dynamics, residual trace, timing, snapshots, and external
  comparison evidence.
- Keep the current Figure 9 residual SVG artifacts regenerated from the latest
  `residual_history` CSV artifacts, then extend the histories from one-step
  reduced scaffolds to long-run and full-contact house/arch scenes.
- Add GUI snapshots or overlays as needed for paper-figure parity; if current
  OSG rendering or reusable widgets cannot make the examples self-contained,
  improve the viewer instead of leaving the limitation outside this task.
- Extend the Figure 10 sweep from the reduced one-step `fbf_paper_gamma_sweep`
  contact-rich rows to the full house/arch scenes with long-run residual and
  physical-outcome columns.
- Add optional external comparison harnesses for Kamino, MuJoCo, and the paper
  implementation if available; keep these dependencies out of core DART
  library dependencies.

## Completion Criteria

Do not call this task complete unless all of the following are true:

- Every row in `paper-parity-matrix.md` has either an implemented DART fixture
  or a recorded source-level reason it cannot be reproduced.
- Small paper fixtures have tests, benchmark rows, trace/export coverage, and
  GUI scenes where a GUI scene is meaningful.
- Contact-rich house/card and arch fixtures have bounded exact-FBF execution,
  physical outcome checks, residual evidence, timing data, and visual/snapshot
  evidence.
- GUI examples are self-contained in the `Scene` panel, and any OSG renderer
  or reusable GUI widget improvements needed to inspect the paper fixtures
  have been implemented rather than deferred outside the task.
- Figure 10 has an implemented step-size sweep harness.
- `PR_REPORT.md` records current command outputs, hardware/build context,
  benchmark rows, trace commands, GUI commands, and a visible "not covered yet"
  section.
- Source and website credit for Song, Fan, Ascher, and Pai's SCA 2026 paper is
  present and kept current.
- Verification gates have been rerun after the last code change, or any gate
  that could not be run has a concrete blocker and next command.
- Durable docs have been promoted and this dev-task folder has either been
  retired according to the DART resume workflow or explicitly kept because real
  remaining work is recorded here.

If `docs/dev_tasks/fbf_exact_coulomb_friction/` still exists, assume the task
is active unless the tracker says otherwise and all criteria above are met.

## Reporting Rules

- Use "implemented in this slice" for partial progress.
- Keep a "still missing" list in any status report or PR report.
- Do not write "complete", "fully covered", or "paper parity achieved" while
  any completion criterion is false.
- If work must pause, leave the next concrete command or file to edit; do not
  summarize the pause as success.
