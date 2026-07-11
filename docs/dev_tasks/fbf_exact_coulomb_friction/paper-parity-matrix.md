# Paper Parity Matrix

This matrix translates the paper, figures, video, and tables into DART-facing
requirements. It is not complete until the authors' code or parameter files are
available, but it captures the current source evidence.

## Solver-Level Parity

| Requirement | Paper target | DART 6.20 gap |
| --- | --- | --- |
| Exact reduced Coulomb law | Primal cone, dual cone for augmented velocity, complementarity | Shared cone, augmented-velocity, and residual helpers now exist; first opt-in `World` single-contact smoke plus incline/backspin/turntable fixtures and a Painleve proxy fixture exist |
| FBF outer loop | Tseng-style explicit coupling update, cone QP, correction, cone projection | Internal outer-loop shell exists and `ExactCoulombFbfConstraintSolver` can route supported constrained groups, one capped `World` contact, three small paper-parameter fixtures, and a Painleve proxy through it |
| Inner cone solve | Strongly convex SOCP/QP over product of Coulomb cones | Matrix-free projected-gradient and reference block Gauss-Seidel solves now exist for unit problems; the DART route tries block-GS first, can retry with projected-gradient after exact-solve failure, and can run a bounded dense residual-polish cleanup when both fail. The DART route can now opt into constraint-row Delassus products for FBF `W*x`, but it still assembles the dense snapshot for staging, cold-start seeding, and dense polish, so matrix-free/scratch production work, broader fixture validation, and performance tuning remain open |
| Step-size control | Safe base step plus safeguarded adaptive shrink/growth | Safe spectral initialization and shrink-only local variation line search now exist; no cross-step warm-start/growth policy yet |
| Residual | Dimensionless max of primal cone, dual cone, and gap terms | Shared residual helper now exists in `dart/math/detail/CoulombCone.hpp`; solver diagnostics, sampled per-step guards, bounded opt-in per-outer residual histories, and a CSV state/residual trace exporter exist for the small headless fixtures plus reduced-contact 26-card and 25/101-stone scaffolds; the exporter can emit representative tracked-body rows, one row per mobile body, or exact-FBF residual-history rows for retained exact-solve records, including multiple exact groups in a sampled step; individual and combined SVG plots now exist for the current reduced-scaffold histories, but paper-matched full-contact/long-run convergence evidence is still missing |
| Matrix-free `W` | Scatter, inverse mass, gather; no dense full Delassus required | Internal matrix-free contact-problem helper exists, and the DART 6 adapter now has `applyExactCoulombConstraintDelassus(...)` to apply the adapted Delassus product through retained constraints. `ExactCoulombFbfConstraintSolverOptions::useMatrixFreeDelassusOperator` routes FBF `W*x` products through that helper in focused tests, a one-contact `World` smoke, and benchmark-only reduced card-house probe rows. `useMatrixFreeDelassusSeed` can also seed the opt-in route from operator-extracted local diagonal blocks, and the 16-contact probe verifies the seed diagnostic toggles correctly. Paper-style matrix-free `World` performance is still missing because the dense snapshot is still assembled and retained for the current staging bridge and dense-polish recovery path, and the current repeated impulse-test product route is 13-19x slower than dense multiplication on measured 16/32-contact reduced-card rows; larger matrix-free product rows hit a 120 s bound before a first CSV row |
| Warm start | Reuse previous contact solution when contact set is unchanged | Conservative same-constraint-sequence cache now exists; true contact-manifold identity and persistence across changing contact sets still missing |

## Figures And Demos

| ID | Fixture | Parameters and expected result | Evidence target for DART |
| --- | --- | --- | --- |
| Fig. 1 | Cube on incline | Slope `atan(0.5)`, threshold `mu = 0.5`, `T = 2 s`, `dt = 1/60`; FBF and Kamino match analytical displacement, MuJoCo drifts | First headless threshold regression exists for `mu = 0.5`/`0.4` with analytical slide check, sampled residual guard, DART boxed-LCP baseline, DART-side boxed/FBF benchmark rows, CSV trace export, and GUI scene `fbf_paper_incline`; full sweep, external comparison, and paper-hardware timing still missing |
| Fig. 2 | Incline snapshots | `mu = 0.4` slides; `mu = 0.5` sticks; FBF/Kamino stay in sustained contact | State-level stick/slide regression, CSV state/contact trace export, and GUI scene `fbf_paper_incline` exist; paper snapshot parity still missing |
| Fig. 3 | Backspin sphere | `r = 0.25 m`, `v0 = 4 m/s`, `omega0 = -200 rad/s`, `mu = 0.5`, `dt = 1/60`; exact law reverses and approaches `v_inf = -11.429 m/s`, `omega_inf = -45.71 rad/s` | First headless regression now checks analytical terminal velocity/omega, sampled residual guard, no boxed-LCP fallback, contact persistence, finite boxed-LCP baseline, DART-side boxed/FBF benchmark rows, CSV trace export, `residual_history` rows for the first exact-FBF step, and GUI scene `fbf_paper_backspin`; paper trajectory comparison, full residual plot, and paper-hardware timing still missing |
| Fig. 4 | Rotating turntable | `mu = 0.2`, `omega = 2`: ejected; `mu = 0.2`, `omega = 5`: ejected; `mu = 0.5`, `omega = 2`: captured; `mu = 0.5`, `omega = 5`: ejected | First headless capture/ejection classification regression exists with sampled residual/no-fallback checks plus finite boxed-LCP baseline, DART-side boxed/FBF benchmark rows, CSV trace export, and GUI scene `fbf_paper_turntable`; radial trajectory comparison, snapshots, and paper-hardware timing still missing |
| Fig. 5 | Painleve box | `mu = 0.5`: slide/rest without tumble for FBF/Kamino; `mu = 0.55`: shorter travel then tumble; MuJoCo jumps/travels farther | A headless Painleve-style proxy now checks `mu = 0.5` slide/rest upright and `mu = 0.55` shorter travel-to-tumble with sampled residual/no-fallback diagnostics, finite DART boxed-LCP baseline, DART-side boxed/FBF benchmark rows, CSV trace export, and GUI scene `fbf_paper_painleve`; exact author-scene parameters, trajectory comparison, snapshots, and paper-hardware timing still missing |
| Fig. 6 | House of cards | 26 thin plates, four levels, `mu = 0.8`, `dt = 1/60`; gravity settle, stand at `t = 6.7 s`, four projectiles, post-impact at `t = 10 s`; FBF/Kamino no visible creep before localized impact failure | The two-card A-frame precursor is now enabled as `CardHouseAFramePrecursorStands` and solves at `1e-6` with zero boxed-LCP fallback after raising the contact-rich outer cap to `5000`; GUI scene `fbf_paper_card_aframe` shows this precursor; an enabled approximate 26-card four-level construction scaffold exists as `CardHouseFourLevelSceneBuilds`; `CardHouseFourLevelOneStepReducedContactProbe` now runs one exact-FBF dynamic step on the 26-card world with a practical reduced `max_contacts=56`, `max_contacts_per_pair=1`, `step_size_scale=10`, `outer_relaxation=1.5` cap, residual `2.218e-16`, zero exact-FBF failures, zero boxed-LCP fallback, and max/total group FBF iterations `1908`/`1908`; reduced-contact benchmark rows, top-card and 26-mobile-card CSV trace rows, and GUI scene `fbf_paper_card_house_26` use the same dynamic cap and expose max/total iteration, gamma-scale, and outer-relaxation diagnostics; after the paper residual-scale slice, `fbf_paper_card_house_probe` shows 58 contacts is clean but slow, 59 contacts is clean but too slow and uses the projected-gradient retry, 60 contacts is still the current fallback boundary with one dense polish, retained failed-history samples from residual `1.146e-5` to pre-polish history residual `5.338e-6` through iteration 30000, dense-polished best/final failed residual `5.334e-6` at iteration `29355`, and 64 contacts still has no clean no-fallback result; the probe now localizes the short 60-contact failure to failed dual contact `13` (`card_house_l0_support0_body|card_house_l0_f1_left_body`) and failed complementarity contact `2` (`BodyNode|card_house_l0_f1_left_body`); quick cap/geometry diagnostics did not clear the boundary (`max_contacts_per_pair=2`, smaller initial penetration, and spacing `0.53`/`0.57`/`0.59` still either fell back or changed contact count); a kept residual-selected local dense polish improves failed residuals (`6.166e-5` at 3000 outer iterations, `2.079e-5` at 10000) but still does not clear 60 contacts; an ordered multi-contact local dense-polish experiment was reverted because it preserved 56 contacts but did not improve the short 60-contact `6.166e-5` failed residual; full scene recreation from author assets, full-contact bounded execution, no-creep guard, projectile phase, long-run traces, timing, and dynamic visual snapshots are still missing |
| Fig. 7 | Masonry arch | 25 voussoirs, endpoints pinned, 23 dynamic interior stones, projectile at crown; FBF/Kamino preserve arch, MuJoCo slumps/collapses | A reduced-contact approximate scaffold now exists with 25 stones, static endpoint supports, 23 dynamic interior stones, headless scene-build and one-step exact-FBF tests, DART-side boxed/FBF benchmark rows, crown-stone and 23-mobile-stone CSV trace export, and GUI scene `fbf_paper_masonry_arch_25`; the default one-step exact-FBF probe passes at `max_contacts=48`, `max_contacts_per_pair=2`, `step_size_scale=10`, residual `9.9650461738801354e-07`, one exact solve, zero exact-FBF failures, zero boxed-LCP fallback, and `2645` FBF iterations using a 20000 outer-iteration arch budget; a reduced projectile scaffold now also exists as `MasonryArch25ProjectileScaffoldRuns`, trace scenario `masonry_arch_25_projectile_reduced_contact`, benchmark rows, and GUI `Launch projectile` / `p` action. Its exact-FBF trace row solves one step at 48 contacts with residual `9.9901843912475669e-07`, one exact solve, zero failures/fallbacks, and `success`; its focused exact-FBF benchmark row reports about `4921 ms` and `2205` FBF iterations; the action capture shows the projectile plus self-contained Scene-tab text. Benchmark-only `fbf_paper_arch_probe` rows show 49/2 still generates only 48 actual contacts, 64/4 solves in `31118.9 ms`, 80/4 solves in `117628.4 ms`, and 100/4 timed out under a 120 s guard before a row; author/Rigid-IPC geometry, real pinned/projectile physical outcome, bounded 100-contact row, long-run traces, paper snapshots, and timing parity are still missing |
| Fig. 8 | 101-stone arch | FBF keeps the arch balanced; Kamino fails under same setup | A reduced-contact approximate scaffold now exists with 101 stones, static endpoint supports, 99 dynamic interior stones, headless scene-build and one-step exact-FBF tests, DART-side boxed/FBF benchmark rows, crown-stone and 99-mobile-stone CSV trace export, and GUI scene `fbf_paper_masonry_arch_101`; the default one-step exact-FBF probe passes at `max_contacts=38`, `max_contacts_per_pair=2`, `step_size_scale=10`, residual `9.9001648754006507e-07`, one exact solve, zero exact-FBF failures, zero boxed-LCP fallback, and `3912` FBF iterations using a 20000 outer-iteration arch budget; benchmark-only `fbf_paper_arch_probe` rows correct the old 39/2 boundary and show clean one-step rows through 100/2, with the 100/2 row taking `56137.1 ms`, residual `9.9444848815471616e-07`, one exact solve, zero failures, and zero fallbacks; author/Rigid-IPC geometry, long-run balance outcome, full-contact physical parity, long-run traces, snapshots, and timing parity are still missing |
| Fig. 9 | Residual convergence | Backspin geometric residual descent; house and arch settle near float32 floor | Sampled per-step residual guards and CSV state/residual trace export exist for the small headless fixtures, the reduced 26-card top-card/26-mobile-card scaffold, and the reduced 25/101-stone arch crown-stone/mobile-stone scaffolds; `residual_history` export now emits bounded per-outer rows for backspin step 1, all three reduced 26-card exact groups at step 1, and the reduced 25/101-stone arch exact groups; individual SVG plots and a combined reduced-scaffold residual-history panel are generated from those CSVs; the enabled two-card A-frame precursor now passes the paper `1e-6` target, but full-contact histories, long-run histories, paper-matched comparison plots, and full house/arch convergence evidence are still missing |
| Fig. 10 | Step-size sweep | Fixed-gamma sweep and adaptive rule on house/arch; too-small gamma can fail physically even with low residual | `fbf_paper_trace` now accepts an optional `initial_gamma`, and `fbf_paper_gamma_sweep` emits fixed-gamma CSV rows with configured `max_contacts`, configured `max_contacts_per_pair`, solver-budget columns, clean-exact, residual, fallback, physical-outcome, and final-state columns for the small fixtures plus the reduced one-step 26-card card-house and 25/101-stone arch scaffolds; latest cap-aware smokes cover full-duration backspin at `max_contacts=1`, card-house cap override `32`, promoted card-house cap `56`, 25-stone arch cap `48` / two contacts per pair, and 101-stone arch cap `38` / two contacts per pair with `nan,0.001` arch rows, where the fixed `0.001` row falls back for both arch scaffolds; the full-contact, long-run house/arch Figure 10 sweep with paper physical outcomes is still missing |
| GPU table | House of cards, 10 levels | FBF stands; MuJoCo topples; Kamino stands but slower | A construction-only DART scaffold now exists with 155 cards, `CardHouseTenLevelSceneBuilds`, boxed-LCP one-step benchmark row `card_house_10_construction_boxed_lcp`, and GUI scene `fbf_paper_card_house_10`; exact-FBF dynamics, residual trace, timing parity, snapshots, and external baseline comparisons are still missing |

Additional Fig. 6 scaffold evidence added on 2026-07-09:
`CardHouseFourLevelSettleProjectileScaffoldRuns` is a fast 16-contact CTest
for one settle step followed by four launched projectiles; `fbf_paper_trace
card_house_26_settle_projectile_reduced_contact ... phase_summary` emits
initial, settle, and projectile rows at the promoted 56-contact reduced cap;
and `BM_PaperFixtureStepTime/card_house_26_settle_projectile_reduced_contact_*`
registers boxed-LCP and exact-FBF two-step benchmark rows. The exact-FBF trace
and benchmark run with zero failures/fallbacks on the reduced scaffold, and
`fbf_paper_card_house_26` has a Scene-panel control to launch the same four
projectiles. The same launch is also available as the `p` key action, and
`pixi run capture-action fbf_paper_card_house_26 p ... 1280 720 0` now records
a visible reduced-projectile GUI capture with the Scene-tab explanation
visible. This does not close Fig. 6 parity: the 6.7 s no-creep settle,
full-contact run, real impact outcome, 10 s post-impact trace, paper snapshots,
and external timing comparisons remain missing.

Additional Fig. 7 scaffold evidence added on 2026-07-09:
`MasonryArch25ProjectileScaffoldRuns` is a reduced 25-stone arch projectile
smoke, `fbf_paper_trace masonry_arch_25_projectile_reduced_contact ...`
emits tracked-projectile and dynamic-body rows, and
`BM_PaperFixtureStepTime/masonry_arch_25_projectile_reduced_contact_*`
registers boxed-LCP and exact-FBF benchmark rows. The exact-FBF trace and
benchmark run with zero failures/fallbacks on the reduced scaffold, and
`fbf_paper_masonry_arch_25` has a Scene-panel launch control plus a `p` key
action for the same projectile. A refreshed action capture records the
projectile at the crown with the Scene-tab explanation visible. This does not
close Fig. 7 parity: the author/Rigid-IPC geometry, real post-impact arch
outcome, 100-contact row, long-run trace, paper snapshots, and external timing
comparisons remain missing.

GUI parity for any row requires a self-contained `dart-demos` Scene panel and
visual evidence that can be understood without reading source. The shared
`dart-demos` host now renders each scene's `ScenePanelDocumentation` metadata
before custom controls, so overview, expected-result, and coverage text are
both catalog-verifiable and visible in the GUI. If the current OSG renderer,
camera/capture path, `dart-demos` host, or ImGui widgets are not enough for a
fixture, improving those viewer surfaces is part of this task for that row and
must be recorded before the GUI row is counted as covered.
The current one-step GUI capture smoke is recorded in
[gui-capture-report.md](gui-capture-report.md): after the host-rendered
metadata change, all nine `fbf_paper_*` scenes produce non-blank
`dart-demos --headless --shot` captures with the `Scene` tab visible, and
`assets/fbf_gui_capture_sheet.png` was regenerated from those captures. This
is visual smoke evidence only; paper-matched snapshot parity for the
contact-rich figures remains missing.

## Timing Targets From The Paper

The paper reports single-precision CPU runs on one Apple-silicon Mac and GPU
runs on an RTX 3090. Treat the numbers as external targets, not DART proof,
until the fixture and implementation details are reproduced.

Initial DART-side timing infrastructure now exists in
`BM_INTEGRATION_exact_coulomb_fbf_paper`. It covers the small incline,
backspin, turntable, Painleve-proxy, reduced-contact 26-card card-house,
construction-only 10-level card-house, and reduced-contact 25/101-stone
masonry-arch scenes. The implemented exact-FBF rows carry residual/fallback
counters; the 10-level row is boxed-LCP construction-only. This infrastructure
does not yet produce a complete timing report with hardware metadata,
statistical sampling, paper hardware comparison, Kamino/MuJoCo runs, or
full-contact house/arch timings.

The companion `fbf_paper_trace` executable exports CSV samples for the same
small scenes plus the reduced 26-card card-house and 25/101-stone arch
scaffolds under exact FBF or boxed LCP. By default it prints one representative
tracked body; with `dynamic_bodies` or `full_scene` as the sixth argument it
prints one row per mobile body. With `residual_history` as the sixth argument,
exact-FBF runs print bounded per-outer residual rows for retained exact-solve
records, including multiple exact groups in one sampled step. Use it to
produce trajectory and residual comparison artifacts; it is not a benchmark
and does not replace the paper convergence plots.

| Scene | Contacts | FBF CPU target | Paper comparison |
| --- | ---: | ---: | --- |
| Backspin | 1 | 6.0 ms | MuJoCo 0.3x, Kamino 1.8x |
| Incline, `mu=0.5` | 4 | 5.5 ms | MuJoCo 0.4x, Kamino 3.1x |
| Incline, `mu=0.4` | 4 | 5.3 ms | MuJoCo 0.6x, Kamino 4.1x |
| Painleve, `mu=0.5` | 4 | 7.0 ms | MuJoCo 0.9x, Kamino 1.1x |
| Painleve, `mu=0.55` | 4 | 6.4 ms | MuJoCo 0.6x, Kamino 0.7x |
| Turntable contact-free/ejected cells | 0 | 3.1-3.7 ms | Contact-free after ejection |
| House of cards, 4 levels | 214 | 199 ms | MuJoCo 8.7x slower; Kamino at least 50x slower/lower bound |
| Masonry arch, 25 stones | 100 | 595 ms | MuJoCo 23.3x slower; Kamino 0.38x |
| Arch, 101 stones | not listed in table | 1234 ms | MuJoCo DNF at least 30x; Kamino contact-free trajectory |
| House of cards, 10 levels | not listed in table | 853 ms | MuJoCo DNF at least 68x; Kamino at least 70x |

GPU outcome table:

| Scene | FBF | MuJoCo | Kamino |
| --- | --- | --- | --- |
| Card, 5 levels | 479 ms, stands | 37 ms, topples | 4126 ms, stands |
| Card, 10 levels | 1513 ms, stands | 621 ms, topples | 4285 ms, stands |
| Arch, 25 stones | 2051 ms, stands | 61 ms, collapses | 355 ms, stands |
| Arch, 101 stones | 3731 ms, stands | 821 ms, collapses | 615 ms, collapses |

## Baselines DART Must Report

- Existing DART 6.20 default: `BoxedLcpConstraintSolver` with Dantzig primary
  and PGS secondary.
- Existing DART 6.20 alternate detectors where relevant: FCL default, DART
  native explicit factory, Bullet/ODE when built.
- Paper baselines if runnable: MuJoCo and Kamino.
- Paper implementation if the code repository becomes available.
- These external comparison dependencies may be used for tests, examples, and
  benchmarks, but should not become core DART library dependencies.

## Completion Bar

This bar is intentionally strict. If a future agent is unsure whether the bar
is met, the answer is "not complete"; update
[AGENT_CONTINUATION.md](AGENT_CONTINUATION.md) with the current progress and
todo list instead of reporting completion.

This task is not complete until all rows above have an implemented fixture or
a recorded source-level reason they cannot be reproduced, plus:

- physical outcome match,
- exact-Coulomb residual match,
- DART boxed-LCP baseline comparison,
- performance report with commands, hardware, build mode, sample count, and
  raw data,
- gz-physics compatibility decision for any public/default change.

## Solver Evidence Update, 2026-07-09 Second Session

The incremental inner block-Gauss-Seidel solver and the scratch-backed
`ExactCoulombContactRowOperator` (WP-PG.14 pattern, default-on with automatic
impulse-test fallback) changed the contact-rich rows above materially. On the
current base `db255a08e8e`:

- Fig. 6 26-card house: the 60/64-contact fallback boundary is cleared. One
  step at 56/1 is clean in `277 ms` (was `4971 ms`), 60/1 clean in `1.59 s`,
  the natural per-pair-1 manifold (63 contacts) clean in `1.76 s`, and the
  full natural per-pair-4 manifold (108 contacts) clean in `2.5 s` with
  `8479` max FBF iterations and zero fallbacks. The stale 60-contact failure
  localization above is historical.
- Fig. 7 25-stone arch: the natural manifold is 96 contacts (per-pair 4 and
  8 both produce 96). It stalls at relaxation 1.0 but solves cleanly with
  `outer_relaxation=1.5` and a `120000` outer budget (residual `4.70e-7`,
  ~74 s). The base-refresh regression in
  `MasonryArch25ProjectileScaffoldRuns` (dual stall at `4.13e-6`) is fixed by
  the same options (passes in `9.9 s`).
- Fig. 8 101-stone arch: the full per-pair-4 manifold (512 contacts) solves
  cleanly in `1626` iterations (~50 s at probe defaults, ~37 s with the arch
  fixture options); truncated 100/2-256/2 caps stall on the current base, so
  the older "clean through 100/2" note is stale. Truncated caps, not full
  manifolds, are the hard cases.
- Warm start: cross-step contact-manifold matching landed (world-space
  impulse records matched by body pair and nearest contact point), with the
  passing world smoke
  `ConstraintSolver.ExactCoulombFbfWorldSmokeManifoldWarmStartAcrossSteps`.
  The "true contact-manifold identity" gap in the solver-parity table is now
  closed at the matching level; long-run persistence evidence remains open.
- External baselines: MuJoCo 3.10.0 runs locally; a backspin prototype at the
  paper `dt = 1/60` misses the analytic terminal state by ~20 percent
  (`v = -13.73` versus `-11.43 m/s`) and converges only at `dt = 1/2000`
  (0.02 percent), matching the paper's exact-versus-regularized contrast.
  Kamino IS publicly available as `newton.solvers.SolverKamino` inside
  NVIDIA's open-source Newton engine (`pip install newton`, Apache-2.0,
  import verified 2026-07-09); it is not yet wired into any FBF paper-parity
  scene, and a hardware-comparable timing row needs a CUDA device that CI
  cannot assume — an open integration task, not an availability gap. The
  paper's own repository remains unavailable (HTTP 404 via page, API, and
  unauthenticated `git ls-remote`, rechecked 2026-07-09). Author-faithful
  Rigid-IPC arch geometry (MIT, commit `23b6ba6fbf8`) with a verified
  closed-form generator is staged for the Fig. 7/8 fixtures.

## 2026-07-10/11 Update

- Fig. 7/8 arch geometry: the author-faithful Rigid-IPC arch port staged above
  landed in `dart/math/detail/MasonryArchGeometry.hpp` (MIT, commit
  `23b6ba6fbf8`; gap hack dropped, base flattening kept, cm-to-m, `mu = 0.5`,
  density `1000`, all stones dynamic plus a fixed ground) and is wired into
  fixtures, benchmark, trace, gamma-sweep, and GUI surfaces. Under this
  geometry the natural contact manifolds are **52 contacts** for the 25-stone
  arch (clean in `256 ms`/`892` FBF iterations) and **204 contacts** for the
  101-stone arch (clean in `17.2 s`/`28750` FBF iterations), both `1e-6`
  zero-fallback. These numbers **supersede** the 96-contact/512-contact
  natural-manifold figures reported in the "Solver Evidence Update, 2026-07-09
  Second Session" appendix above, which belonged to the earlier approximate,
  static-endpoint overlapping scaffold that this port replaces. The full
  paper-fixture suite runs 16/16 in `27.7 s` (was `107.9 s` before the port).
- Fig. 6 26-card house: the one-step full natural manifold (108 contacts,
  per-pair-4) is unchanged from the prior appendix and remains clean in
  `2.5 s`. The new measured frontier is multi-step: probing the full-manifold
  settle/projectile sequence (`fbf_paper_trace card_house_26_settle_projectile_full`,
  with a new `warm_start` CLI argument enabling cross-step pair-keyed warm
  starts) shows settle steps 2+ reaching 122-138 contacts at per-pair-4,
  cross-step warm-started, with a `120000`-outer budget, but the exact-FBF
  solve still falls back to boxed LCP at failed residuals ranging from
  `2.9e-6` to `1.5e-4`. This is the honest remaining Fig. 6 blocker: the
  one-step manifold-size gap is closed, but multi-step full-contact
  convergence is not yet.
- Warm starts: the cross-step contact-manifold matching described in the prior
  appendix (body-pair plus nearest-contact-point matching of world-space
  impulse records) is confirmed pair-keyed and is the same mechanism used by
  the multi-step card-house settle probe above; long-run persistence past a
  handful of steps is not yet demonstrated.
- External baselines: `fbf_paper_mujoco_baseline.py` is now validated
  end-to-end (backspin/incline/turntable plus opt-in
  `masonry_arch_101_rigid_ipc`) behind a new non-default `fbf-baselines` pixi
  feature/environment (first `pypi-dependencies` use in `pixi.toml`, flagged
  for maintainer sign-off; default environment untouched). A new
  `fbf_paper_kamino_baseline.py` availability probe corrects the prior
  appendix's framing from an open question to a concrete state: `SolverKamino`
  is importable (`newton` 1.3.0, `warp` 1.15.0) and the probe records
  GPU/driver metadata when present, but wiring a hardware-comparable timing
  row still needs a CUDA device, which remains an open integration task rather
  than an availability gap. The paper's own code repository remains
  unavailable (HTTP 404, rechecked 2026-07-09).
- Long-run evidence (10 s+ warm-started arch balance for both stone counts,
  the converged multi-step card-house settle, and regenerated Figure 9/10
  artifacts at full-manifold scale) is in progress or not yet started; see
  `AGENT_CONTINUATION.md` for the current work-board state.

## 2026-07-11 Frontier Resolution

The multi-step full-contact settle blocker recorded in the update above is
RESOLVED by a two-part fix, both parts validated:

- Convergence: the stall's root cause was DART's ERP position-correction bias
  inside the velocity-phase right-hand side (`v_free = -b`). Running the
  exact path under split impulse removes the bias and matches the paper's
  formulation (friction solve over pure dynamics).
  `ExactCoulombFbfConstraintSolver` forwards `isSplitImpulseEnabled()` into
  the adapter build, and the long-run `fbf_paper_trace` scenarios enable
  split impulse for both solver modes (plus a `split_impulse=default|0|1`
  CLI argument).
- Base bug found and fixed: DART's split-impulse position pass assembled its
  LCP with unit-impulse tests that cleared the velocity-phase constraint
  impulses before `World::step` integrated them, so resting bodies free-fell
  and tunneled while contacts persisted (minimal repro: one sphere on a
  plane under the plain boxed solver). Fixed by preserving body and joint
  constraint impulses around the position pass in
  `ConstraintSolver::solvePositionConstrainedGroups`, guarded by
  `ConstraintSolver.SplitImpulsePreservesVelocityPhaseContactResponse`.

Validated together: full-manifold settle steps solve exactly at `1e-6` with
zero fallbacks and genuine no-creep physics (minimum card height drift below
`0.25 mm` over the bounded fixture probe
`CardHouseFourLevelFullManifoldSplitImpulseSettleProbe`), at ~`1.4 s`/step.
The full 10 s Fig. 6 sequence reached its 6.7 s four-projectile launch with
the house standing (~3 mm total settle compaction, zero fallbacks,
all-exact); Figure 9 residual histories and SVG plots were regenerated at
full-manifold scale (108/52/204-contact scenes). The base fix is a DART
bugfix and follows the dual-PR rule when published.
