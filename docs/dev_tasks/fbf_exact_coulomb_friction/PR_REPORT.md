# PR Report: Exact Coulomb FBF Prototype

This report is the PR-facing summary for the DART 6.20 exact-Coulomb FBF
prototype. It records what is covered, how to reproduce the GUI examples,
which tests and benchmarks exist, and what remains missing before paper parity
can be claimed.

## Direct Coverage Answer

No, this branch does not yet contain all tests, benchmarks, and GUI examples
from the paper.

## 2026-07-09 Second-Session Solver Update

The contact-rich convergence blocker that dominated earlier report sections is
resolved at the one-step level:

- The inner block-Gauss-Seidel frozen-cone solve is now incremental: cached
  per-solve diagonal Delassus blocks, incremental block-column updates with a
  periodic full-product refresh, cached local Lipschitz steps, and per-contact
  segment gradients. The reference entry point is preserved.
- The new scratch-backed `ExactCoulombContactRowOperator`
  (`dart/constraint/detail/ExactCoulombContactRowOperator.hpp`, default-on via
  `useContactRowDelassusOperator`) serves `W*x` products, per-contact diagonal
  blocks, block-column updates, and the dense snapshot from per-row body
  Jacobians and articulated-inertia solves instead of DART impulse tests, for
  supported groups of exactly-typed three-row contacts between single free
  rigid bodies; unsupported groups keep the impulse-test route automatically.
  Snapshot parity is proven by
  `ConstraintSolver.ExactCoulombContactRowOperatorMatchesImpulseTests`.
- Measured on the current base: card-house one-step 56/1 clean in `277 ms`
  (was `4971 ms`); the former 60-contact fallback boundary is clean in
  `1.59 s`; the full natural per-pair-4 card-house manifold (108 contacts) is
  clean in `2.5 s`; the 101-stone arch full 512-contact manifold solves in
  `1626` iterations; the 25-stone arch natural 96-contact manifold solves with
  `outer_relaxation=1.5` and a `120000` outer budget.
- Contact-manifold warm starts now work across `World` steps (body-pair plus
  nearest-point matching of world-space reaction records), covered by
  `ConstraintSolver.ExactCoulombFbfWorldSmokeManifoldWarmStartAcrossSteps`.
- Two regressions introduced by the base refresh to `db255a08e8e` were found
  and fixed (arch projectile fixture; stale 101-stone cap notes).
- MuJoCo 3.10.0 backspin comparison evidence exists (paper `dt=1/60`:
  ~20 percent deviation from the analytic terminal state and a spurious 11 m
  bounce, versus DART exact-FBF asserting the analytic value; MuJoCo
  converges only at `dt=1/2000`). Kamino is publicly available as
  `newton.solvers.SolverKamino` in NVIDIA's open-source Newton engine
  (`pip install newton`, Apache-2.0, import verified 2026-07-09) but is not
  yet wired into paper-parity scenes and needs a CUDA device for a fair
  timing row — an open integration task, not an availability gap. The
  paper's own repository remains unavailable (HTTP 404 via page, API, and
  unauthenticated `git ls-remote`, rechecked 2026-07-09). Author-faithful
  Rigid-IPC arch geometry (MIT) is staged for integration.

Earlier sections below describe the pre-session state; numbers superseded by
this update are historical.

Implemented in this slice:

- Core exact-Coulomb cone math, residuals, contact problem helpers, and FBF
  reference solver utilities.
- An opt-in `ExactCoulombFbfConstraintSolver` for supported isotropic contact
  groups, with boxed-LCP fallback diagnostics, conservative warm starts, and a
  projected-gradient robustness retry that continues from a finite failed
  block-GS FBF reaction when available.
- A constraint-row Delassus product helper plus opt-in solver route
  (`useMatrixFreeDelassusOperator`) that applies FBF `W*x` through retained
  constraints in focused adapter, solver, and one-contact `World` smokes.
  This is matrix-free product staging only; the dense snapshot is still
  retained for current staging and dense-polish recovery. A benchmark/staging
  `useMatrixFreeDelassusSeed` option can seed the opt-in route from
  operator-extracted local diagonal blocks, avoiding the dense global seed
  while still leaving dense assembly in place.
- A DART 6 exact-contact adapter cold-start seed that initializes zero contact
  row guesses from a residual-selected projected local/global Delassus seed
  while preserving explicit nonzero guesses and solver-level warm starts.
- Headless small-fixture regressions for incline, backspin, turntable, a
  Painleve-style proxy, a two-card A-frame precursor, and construction of a
  26-card four-level card-house scaffold.
- A reduced-contact one-step exact-FBF dynamic probe and exact-FBF/boxed-LCP
  tracked-body plus mobile-body trace rows for the 26-card four-level
  card-house world.
- A reduced-contact two-step 26-card settle/projectile scaffold: a focused
  CTest launches four projectiles after one bounded settle step, the
  `phase_summary` trace scope emits initial/settle/projectile rows, benchmark
  rows time exact-FBF and boxed-LCP versions, and the GUI scene exposes a
  Scene-panel launch control. This is scaffold evidence only, not the paper's
  6.7 s settle and 10 s impact outcome.
- A reduced-contact 25-stone masonry-arch scaffold with static endpoint
  supports, 23 dynamic interior stones, scene-build and one-step exact-FBF
  tests, benchmark rows, trace rows, and a GUI scene. A reduced
  crown-projectile scaffold is also wired through an enabled test, a trace
  scenario, exact/boxed benchmark rows, and a GUI launch control; this is not
  the paper's real post-impact outcome.
- A reduced-contact 101-stone masonry-arch scaffold with static endpoint
  supports, 99 dynamic interior stones, scene-build and one-step exact-FBF
  tests, benchmark rows, trace rows, and a GUI scene.
- Google Benchmark rows for the small DART-side fixtures and the
  reduced-contact 26-card card-house, reduced 26-card settle/projectile, and
  25/101-stone masonry-arch probes under both exact FBF and the existing
  boxed-LCP baseline, plus reduced 25-stone arch projectile scaffold rows.
- A headless CSV trace exporter for the small DART-side fixtures,
  reduced-contact 26-card card-house scaffold, and reduced-contact
  25/101-stone masonry-arch scaffolds under exact FBF and boxed LCP. It can
  emit the representative tracked body, one row per mobile body, or bounded
  exact-FBF residual-history rows for retained exact-solve records; the
  reduced 26-card settle/projectile scenario can also emit a phase-summary
  trace, and the reduced 25-stone arch projectile scenario emits tracked
  projectile plus dynamic-body rows.
- A fixed-gamma CSV sweep helper with residual, fallback, physical-outcome,
  and final-state columns for small fixtures plus reduced one-step 26-card
  card-house and 25/101-stone masonry-arch scaffolds.
- A benchmark-only 26-card contact-cap CSV probe for reduced card-house
  diagnostics, including optional retained residual-history summaries for
  failed exact-FBF cap rows, best-residual/best-iteration diagnostics, and
  worst residual contact indices plus failed contact-pair labels. It now also
  has benchmark-only `use_matrix_free_delassus_operator` and
  `use_matrix_free_delassus_seed` arguments plus CSV columns that record the
  requested options and whether the solver used the constraint-row Delassus
  product route and operator-local seed.
- A benchmark-only masonry-arch contact-cap CSV probe for the reduced
  25/101-stone scaffolds. It records requested caps, actual contacts, solver
  options, residual components, exact/fallback counters, failed-contact labels,
  route flags, and crown/state sanity metrics. This probe corrected the stale
  101-stone 39/2 fallback-boundary note and now shows one-step 101-stone rows
  clean through 100/2.
- `dart-demos` Research GUI scenes for the small fixtures, two-card precursor,
  reduced-contact dynamic 26-card card-house scaffold, construction-only
  10-level card-house scaffold, and reduced-contact 25/101-stone masonry-arch
  scaffolds. The shared demo host renders each FBF scene's overview, expected
  result, and coverage-limit metadata in the scrollable `Scene` panel before
  scene-specific solver controls/diagnostics; the 26-card scene also exposes
  the reduced phase-scaffold projectile launch control there, and the
  25-stone arch scene exposes a reduced crown-projectile launch control there.
- One-step `dart-demos --headless --shot` captures for all nine `fbf_paper_*`
  GUI scenes, a contact sheet, default non-blank image verdicts, and
  action-aware captures for the 26-card and 25-stone arch projectile states.
- Source and RTD website citation for Song, Fan, Ascher, and Pai's SCA 2026
  exact reduced Coulomb friction paper.

Not covered yet:

- The exact authors' scene files and implementation are not anonymously
  readable in this environment, so the Painleve scene remains a proxy.
- The 26-card card-house scene has only a reduced-contact one-step exact-FBF
  dynamic probe plus a reduced two-step settle/projectile scaffold. The current
  practical clean cap is 56 contacts at a 30000 outer-iteration card-house
  budget. A 58-contact diagnostic row is clean but slow; 59 contacts now solves
  without fallback through the projected-gradient retry-continuation path but
  is too slow for the default rung; 60 contacts is the current one-step
  fallback boundary with one projected-gradient retry, one dense residual
  polish, a retained failed-history row through iteration 30000, pre-polish
  history residual `5.338e-6`, and best/final failed residual `5.334e-6` at
  iteration `29355`; and 64 contacts still has no clean no-fallback result.
  Full-contact bounded execution, the paper's 6.7 s no-creep settle,
  real projectile impact/outcome sequence through 10 s, long-run traces,
  snapshots, and timing remain missing.
- The 25-stone masonry arch has only reduced-contact one-step and projectile
  scaffolds. The current default test/benchmark/GUI rung is 48 contacts / two
  contacts per pair with a 20000 outer-iteration arch budget, and the new
  projectile scaffold solves the same reduced cap rather than the paper's real
  pinned/projectile outcome. The cap probe shows 64/4 and 80/4 solve cleanly,
  but 80/4 takes about 118 seconds and 100/4 did not produce a row under a
  120-second timeout. It does not yet use the author/Rigid-IPC geometry,
  real pinned/projectile physical outcome, bounded 100-contact row, long-run
  trace rows, paper snapshots, or timing parity.
- The 101-stone masonry arch has only a reduced-contact one-step scaffold. The
  current default test/benchmark/GUI rung is 38 contacts / two contacts per
  pair with a 20000 outer-iteration arch budget, while the new cap probe shows
  one-step rows through 100/2 solve cleanly on the approximate scaffold. It
  does not yet use the author/Rigid-IPC geometry, long-run balance outcome,
  full-contact physical parity, long-run trace rows, snapshots, or timing
  parity.
- The paper's full 26-card settle/projectile sequence, paper-parity 25-stone
  arch, 101-stone arch, exact-FBF 10-level card-house dynamics, Figure 9 full
  residual plots, and full-contact long-run Figure 10 house/arch step-size
  sweep are not implemented.
- Current `residual_history` trace rows and SVG plots cover one-step reduced
  scaffolds, not full-contact or long-run paper convergence comparisons.
- Paper-style matrix-free `World` performance is not complete. The focused
  route can apply FBF products without reading the dense snapshot, but the
  DART 6 bridge still assembles that snapshot for staging and recovery paths.
  The opt-in operator-local seed removes the dense global seed from that
  diagnostic route, but the current repeated impulse-test product route is
  still much slower than dense multiplication on reduced card-house probe rows.
- Kamino, MuJoCo, and paper-code comparisons are not wired. The paper-code
  GitHub repository was rechecked unauthenticated on 2026-07-09 and still was
  not publicly readable. These comparisons may be added later as
  test/example/benchmark dependencies only, not as core DART library
  dependencies.
- The default DART 6 solver remains unchanged; this is an opt-in prototype.

See [AGENT_CONTINUATION.md](AGENT_CONTINUATION.md) for the live progress ledger
and todo list. See [residual-history-report.md](residual-history-report.md) for
the current Figure 9 residual-history command/results report and SVG plot
artifacts. See [gui-capture-report.md](gui-capture-report.md) for the current
one-step GUI capture evidence and contact sheet.

Recent 60-contact diagnostics that were measured and rejected:

- Residual-selected depth-one Anderson acceleration was implemented as a local
  experiment and reverted. A coefficient cap of `5` made the promoted
  56-contact row fall back; a cap of `1` preserved 56 contacts but left the
  60-contact row falling back at failed residual `6.391e-5`.
- A projected boxed-LCP cold start was implemented as a local experiment and
  reverted. It reduced the 56-contact row to `1165` max FBF iterations but
  worsened the 60-contact failed residual to `1.544e-4`.
- The retained 60-contact failed residual is now localized in the CSV probe:
  failed dual residual is dominated by contact index `13`, pair
  `card_house_l0_support0_body|card_house_l0_f1_left_body`; failed
  complementarity is dominated by contact index `2`, pair
  `BodyNode|card_house_l0_f1_left_body`.
- Contact-manifold and geometry diagnostics did not clear the boundary:
  `max_contacts_per_pair=2` worsened the short 60-contact failed residual to
  `2.366e-4`; `initial_penetration=0.001` still failed at `6.771e-5`; zero
  penetration solved only because it generated 27 contacts; frame spacing
  `0.57` improved the short failed residual to `4.735e-5` but still fell
  back, while `0.53` and `0.59` were worse.
- A residual-selected local dense-polish pass for the worst dual contact was
  kept because it lowers failed residuals without changing the success
  criterion: the default short 60-contact row improved to `6.166e-5`, and a
  10000-outer diagnostic improved to `2.079e-5`. It still does not clear the
  60-contact fallback boundary.
- A multi-contact local dense-polish candidate-order experiment was built,
  measured, and reverted. It preserved the promoted 56-contact row, but the
  short 60-contact row still fell back at `6.166e-5`, the same failed residual
  as the kept single worst-dual-contact local polish.

## 2026-07-10/11 Update: Arch Port, Baselines, Hardening, Warm Starts

This update supersedes the 56/58/59/60/64-contact one-step boundary numbers
and the approximate-scaffold arch numbers reported above; those numbers are
kept as historical record but no longer describe the current code state.

- **Author-geometry arch port.** `dart/math/detail/MasonryArchGeometry.hpp`
  ports the Rigid-IPC weighted-catenary generator (MIT, `ipc-sim/rigid-ipc`
  commit `23b6ba6fbf8`, attribution in the header; the earlier gap hack is
  dropped, base flattening kept, units converted cm-to-m, `mu = 0.5`, density
  `1000`, all stones dynamic plus a fixed ground) and replaces the approximate
  static-endpoint scaffold in fixtures, benchmark, trace, gamma-sweep, and GUI
  surfaces. The natural contact manifolds under this geometry are 52 contacts
  for the 25-stone arch (clean in `256 ms`/`892` FBF iterations) and 204
  contacts for the 101-stone arch (clean in `17.2 s`/`28750` FBF iterations),
  both `1e-6` zero-fallback; these supersede the earlier 96/512-contact
  natural-manifold numbers, which belonged to the replaced overlapping
  scaffold. New `MasonryArch25FullManifoldOneStepProbe` and
  `MasonryArch101FullManifoldOneStepProbe` tests cover the full-manifold cap
  (`kArchFullManifoldMaxContacts=512`, `kArchFullManifoldMaxContactsPerPair=4`,
  `kArchOuterRelaxation=1.5`, `kArchMaxOuterIterations=120000`). The full
  paper-fixture suite runs 16/16 in `27.7 s` (was `107.9 s` pre-port); the
  `MasonryArch25ProjectileScaffoldRuns` base-refresh regression (dual stall at
  `4.13e-6`) is fixed by the same relaxation/budget options and passes in
  `9.9 s`; arch captures were refreshed with passing verdicts and visual
  inspection confirmed the catenary profile.
- **Card-house full-manifold promotion.** The incremental inner
  block-Gauss-Seidel solver plus the default-on scratch-backed
  `ExactCoulombContactRowOperator` clear the former 60/64-contact one-step
  fallback boundary. The full natural per-pair-4 manifold (108 contacts)
  solves clean in `2.5 s` at `8479` max FBF iterations with zero fallbacks;
  promoted benchmark row `card_house_26_reduced_contact_exact_fbf` reports
  `1867 ms`, 108 contacts, max FBF iterations `8479`, max residual `9.995e-7`,
  zero failures/fallbacks, and trace/gamma-sweep rows agree. The full 16-gate
  focused CTest set passed 16/16 in `167 s` with assertions un-weakened (`1e-6`
  tolerance, zero-fallback); only the two new full-manifold arch tests carry a
  widened `120 s` elapsed bound.
- **Pair-keyed cross-step manifold warm starts.** Contact-manifold warm starts
  now work across `World` steps by matching world-space reaction records by
  body pair and nearest contact point, covered by
  `ConstraintSolver.ExactCoulombFbfWorldSmokeManifoldWarmStartAcrossSteps`.
  This is the same mechanism exercised by the multi-step settle probe below.
- **New Fig. 6 frontier: multi-step full-contact settle.** `fbf_paper_trace`
  gained a `card_house_26_settle_projectile_full` scenario (the full-manifold
  settle/projectile scaffold) and a `warm_start` CLI argument. Probing this
  scenario across steps with cross-step pair-keyed warm starts and a
  `120000`-outer budget shows settle steps 2+ reaching 122-138 contacts at
  per-pair-4, but the exact-FBF solve still falls back to boxed LCP at failed
  residuals ranging `2.9e-6`-`1.5e-4`. This is the current honest remaining
  Fig. 6 blocker: the one-step manifold-size gap is closed, but multi-step
  full-contact convergence is not.
- **Hardening tests.** An independent adversarial review (pass 1) hand-derived
  the incremental block-GS gradient algebra, cross-validated the refresh
  cadence against a standalone 60-contact chain-coupled stress probe, verified
  row-operator semantics against the matrix-free reference path, confirmed
  warm-start sign conventions and fallback-path ordering, and found no
  blockers or majors. Three hardening tests it commissioned (refresh-boundary
  stress, flipped-pair warm start, multi-contact/count-change warm start) are
  now landed; a second clean review pass on the post-hardening state has not
  yet run.
- **Baseline harness.** `fbf_paper_mujoco_baseline.py` (backspin/incline/
  turntable plus opt-in `masonry_arch_101_rigid_ipc`) is validated end-to-end
  behind a new non-default `fbf-baselines` pixi feature/environment (first
  `pypi-dependencies` use in `pixi.toml`, flagged for maintainer sign-off;
  default environment untouched): paper `dt=1/60` backspin diverges ~20% with
  a spurious 11 m bounce while DART exact-FBF asserts the analytic terminal
  state, and MuJoCo converges only at `dt=1/2000` (0.02%). A new
  `fbf_paper_kamino_baseline.py` availability probe skips gracefully without
  `warp`/`newton` and records GPU/driver metadata plus an open-integration-task
  note when present (`SolverKamino` verified importable, `newton` 1.3.0/`warp`
  1.15.0, CUDA-gated). The paper's own repository remains unavailable (HTTP
  404, rechecked 2026-07-09).
- **Still open.** Long-run warm-started evidence (10 s+ arch balance for both
  stone counts, the converged multi-step card-house settle) is in progress or
  not yet started; Figure 9/10 artifacts have not yet been regenerated at
  full-manifold scale; a hardware-comparable Kamino timing row needs a CUDA
  device. See `AGENT_CONTINUATION.md` for the current work-board state.

## GUI Examples

The GUI examples are integrated into the consolidated `dart-demos` app under
the `Research` category. Use Pixi so the built executable can resolve the
toolchain runtime libraries:

```bash
pixi run demos
pixi run demos -- --list-scenes
pixi run demos -- --verify-fbf-scene-docs
pixi run demos -- --scene fbf_paper_incline
pixi run demos -- --scene fbf_paper_backspin
pixi run demos -- --scene fbf_paper_turntable
pixi run demos -- --scene fbf_paper_painleve
pixi run demos -- --scene fbf_paper_card_aframe
pixi run demos -- --scene fbf_paper_card_house_26
pixi run demos -- --scene fbf_paper_card_house_10
pixi run demos -- --scene fbf_paper_masonry_arch_25
pixi run demos -- --scene fbf_paper_masonry_arch_101
```

Each scene's `Scene` panel includes a self-contained overview, expected
result, and coverage-limit note so the example can be understood without
reading the source. The `dart-demos` host renders that metadata generically
before custom scene controls, so future paper scenes that fill
`scenePanelDocumentation` are visible in the GUI instead of only passing the
catalog verifier. The `Scene` tab scrolls independently, so long paper notes
and solver diagnostics remain reachable in smaller windows and headless
captures. Most scenes also include an `Exact FBF` / `Boxed LCP`
selector, an `e` key toggle, last-contact count, and exact-FBF diagnostics
when the exact solver is active. The construction-only 10-level card-house scene
defaults to boxed-LCP diagnostics and omits the exact-FBF toggle because
exact-FBF dynamics are intentionally unavailable for that contact-rich scaffold.
The turntable scene also exposes a live angular-speed slider.
When exact-FBF failures occur, the Scene panel also reports the last failed FBF
status and residual so reduced-contact card-house debugging is visible without
reading logs. The panel now also reports last/max/total FBF iterations,
gamma/safe-gamma, gamma scale, outer relaxation, shrink count, and
coupling-variation ratio; those diagnostics plus the scrollable Scene tab are
the GUI widget improvements needed for the current 56-contact card-house rung.
Before the first exact solve, the shared diagnostics widget now reports
`Exact diagnostics: not run yet` instead of showing a misleading initial FBF
status.
The 26-card scene also has a Scene-panel phase-scaffold control that launches
four projectiles from the same positions and velocities used by the reduced
settle/projectile test, trace, and benchmark rows; the Scene tab text explains
that this is not yet the paper's full settle or impact sequence. The same
launch is bound to `p - Launch 4 projectiles`, and the host can now invoke
scene key actions before an off-screen capture through
`--headless-action <key>` / `pixi run capture-action`.
The 25-stone arch scene similarly has a Scene-panel `Launch projectile`
control and `p - Launch projectile` action for the reduced crown-projectile
scaffold. Its Scene text explains that this is not the paper's real Fig. 7
post-impact outcome.
The scene catalog carries `scenePanelDocumentation` metadata for these FBF
examples, and `pixi run demos -- --verify-fbf-scene-docs` fails if any
`fbf_paper_*` scene lacks overview, expected-result, or coverage text. The
custom turntable panel uses the same host-rendered metadata path as the shared
FBF scene factory.
The GUI support also includes the scene-control path used by the
construction-only 10-level card-house scene to omit the exact-FBF toggle. No
renderer-level OSG hook has been needed yet; the current slice needed only a
reusable ImGui diagnostics-widget improvement.

Future paper-parity GUI examples should improve the OSG renderer,
`dart-demos` host, or reusable ImGui widgets when the current viewer is not
enough to make the scene self-contained, inspectable, visually verifiable, or
stable under headless snapshot capture. Those improvements are part of this
dev task when they are needed by the FBF paper examples, not a separate
prerequisite; each promoted GUI row should record whether the existing viewer
was sufficient or what viewer/widget work was added.

Current GUI scene IDs:

| Scene ID | Paper fixture represented | Status |
| --- | --- | --- |
| `fbf_paper_incline` | Figures 1 and 2 incline threshold | Visual counterpart for implemented headless fixture |
| `fbf_paper_backspin` | Figure 3 backspin sphere | Visual counterpart for implemented headless fixture |
| `fbf_paper_turntable` | Figure 4 turntable grid | Visual counterpart with live angular-speed control |
| `fbf_paper_painleve` | Figure 5 Painleve outcome | DART-side proxy, not author-scene parity |
| `fbf_paper_card_aframe` | Figure 6 card-house precursor | Two-card precursor, not full 26-card scene |
| `fbf_paper_card_house_26` | Figure 6 four-level card house | Reduced-contact dynamic 26-card scaffold using the same 56-contact cap as the test/benchmark, plus a Scene-panel four-projectile launch control for the reduced phase scaffold; not full-contact Fig. 6 parity |
| `fbf_paper_card_house_10` | GPU-table 10-level card house | Construction-only 155-card scaffold; exact-FBF dynamics, residual trace, timing, and snapshots are missing |
| `fbf_paper_masonry_arch_25` | Figure 7 masonry arch | Reduced-contact 25-stone scaffold using the same 48-contact / two-contact-per-pair cap as the test/benchmark, plus a Scene-panel reduced crown-projectile launch control; not full Fig. 7 parity |
| `fbf_paper_masonry_arch_101` | Figure 8 large masonry arch | Reduced-contact 101-stone scaffold using the same 38-contact / two-contact-per-pair cap as the test/benchmark, not full Fig. 8 parity |

Headless visual smoke commands:

```bash
pixi run capture fbf_paper_backspin /tmp/fbf_paper_backspin.png 640 480 1
pixi run image-verdict /tmp/fbf_paper_backspin.png
```

After the scrollable Scene-tab/widget update, this verification command passed:

```bash
pixi run capture fbf_paper_backspin /tmp/fbf_paper_backspin_scroll.png 640 480 1
pixi run image-verdict /tmp/fbf_paper_backspin_scroll.png
```

After moving FBF explanations into the shared host renderer, this fresh smoke
also passed and the capture shows the `Scene` tab explanation visible:

```bash
pixi run demos -- --verify-fbf-scene-docs
pixi run demos -- --list-scenes
pixi run capture fbf_paper_backspin /tmp/fbf_paper_backspin.png 640 480 5
pixi run image-verdict /tmp/fbf_paper_backspin.png
```

After adding action-aware headless captures, this reduced 26-card projectile
GUI smoke also passed the default non-blank verdict and produced
`assets/gui_captures/fbf_paper_card_house_26_projectiles.png`:

```bash
pixi run capture-action fbf_paper_card_house_26 p docs/dev_tasks/fbf_exact_coulomb_friction/assets/gui_captures/fbf_paper_card_house_26_projectiles.png 1280 720 0
pixi run image-verdict docs/dev_tasks/fbf_exact_coulomb_friction/assets/gui_captures/fbf_paper_card_house_26_projectiles.png
```

After adding the reduced 25-stone arch projectile GUI scaffold and the
pre-solve diagnostics-widget fix, this action capture passed the default
non-blank verdict and produced
`assets/fbf_paper_masonry_arch_25_projectile.png`:

```bash
pixi run capture-action fbf_paper_masonry_arch_25 p docs/dev_tasks/fbf_exact_coulomb_friction/assets/fbf_paper_masonry_arch_25_projectile.png 1280 720 0
pixi run image-verdict docs/dev_tasks/fbf_exact_coulomb_friction/assets/fbf_paper_masonry_arch_25_projectile.png
```

Do not run two `pixi run demos` invocations in parallel; both rebuild the same
demo target and can race on `libdart-demos-scenes.a`. The same warning applies
to parallel `pixi run demos` plus `pixi run capture-action`; a parallel rerun
hit the known archive/link race and passed when rerun serially.

The latest full GUI capture smoke, recorded in
[gui-capture-report.md](gui-capture-report.md), was regenerated after the
host-rendered Scene-tab metadata update. It used the same one-step
`dart-demos --headless --shot` pattern for all nine `fbf_paper_*` scenes inside
a `pixi run bash -lc` batch, generated
[the contact sheet](assets/fbf_gui_capture_sheet.png), and wrote one
`image_verdict.py` JSON per scene under `assets/gui_captures/`. All nine
captures passed the default non-blank verdict. These are GUI smoke snapshots,
not paper-matched figure snapshots.

## Tests

Build focused targets:

```bash
pixi run cmake --build build/default/cpp/Release --target UNIT_math_CoulombCone --parallel 8
pixi run cmake --build build/default/cpp/Release --target UNIT_math_ExactCoulombContactProblem --parallel 8
pixi run cmake --build build/default/cpp/Release --target UNIT_math_ExactCoulombFbfSolver --parallel 8
pixi run cmake --build build/default/cpp/Release --target test_ExactCoulombConstraintAdapter --parallel 8
pixi run cmake --build build/default/cpp/Release --target test_ExactCoulombFbfConstraintSolver --parallel 8
pixi run cmake --build build/default/cpp/Release --target test_ConstraintSolver --parallel 8
pixi run cmake --build build/default/cpp/Release --target test_ExactCoulombFbfPaperFixtures --parallel 8
```

Run focused tests:

```bash
ctest --test-dir build/default/cpp/Release -R UNIT_math_CoulombCone --output-on-failure
ctest --test-dir build/default/cpp/Release -R UNIT_math_ExactCoulombContactProblem --output-on-failure
ctest --test-dir build/default/cpp/Release -R UNIT_math_ExactCoulombFbfSolver --output-on-failure
ctest --test-dir build/default/cpp/Release -R '(test_ExactCoulombConstraintAdapter|test_ExactCoulombFbfConstraintSolver|test_ConstraintSolver|test_ExactCoulombFbfPaperFixtures)' --output-on-failure
```

Paper-fixture tests currently covered:

| Test | Paper coverage | Notes |
| --- | --- | --- |
| `InclineCubeThresholdStickSlide` | Figures 1 and 2 | `mu = 0.5` stick and `mu = 0.4` slide against analytical displacement |
| `BackspinSphereReachesAnalyticalRollingState` | Figure 3 | Checks analytical no-slip terminal linear/angular velocity |
| `TurntableCaptureEjectionGrid` | Figure 4 | Covers four `mu` / `omega` capture-ejection cells |
| `PainleveProxySlideTumbleThreshold` | Figure 5 proxy | Proxy only until author scene is available |
| `CardHouseAFramePrecursorStands` | Figure 6 precursor | Two-card A-frame, paper `1e-6` residual tolerance |
| `CardHouseFourLevelSceneBuilds` | Figure 6 construction | Builds 26 cards but does not run exact-FBF dynamics |
| `CardHouseFourLevelOneStepReducedContactProbe` | Figure 6 reduced dynamic rung | Runs one exact-FBF step on the 26-card world with `max_contacts=56`, `max_contacts_per_pair=1`, `step_size_scale=10`, `outer_relaxation=1.5`; not full Fig. 6 parity |
| `CardHouseFourLevelSettleProjectileScaffoldRuns` | Figure 6 reduced phase scaffold | Runs one bounded settle step, launches four projectiles, then runs one projectile-phase step at a 16-contact smoke cap; not the paper's 6.7 s settle or 10 s impact outcome |
| `CardHouseTenLevelSceneBuilds` | GPU-table 10-level construction scaffold | Builds 155 cards plus ground with exact solver configured; does not run exact-FBF dynamics |
| `MasonryArch25SceneBuilds` | Figure 7 construction scaffold | Builds the approximate 25-stone arch with static endpoint supports and 23 dynamic interior stones |
| `MasonryArch25OneStepReducedContactProbe` | Figure 7 reduced dynamic rung | Runs one exact-FBF step with `max_contacts=48`, `max_contacts_per_pair=2`, `step_size_scale=10`, residual `9.9650461738801354e-07`, one exact solve, zero exact-FBF failures, and zero boxed-LCP fallback; not full Fig. 7 parity |
| `MasonryArch101SceneBuilds` | Figure 8 construction scaffold | Builds the approximate 101-stone arch with static endpoint supports and 99 dynamic interior stones |
| `MasonryArch101OneStepReducedContactProbe` | Figure 8 reduced dynamic rung | Runs one exact-FBF step with `max_contacts=38`, `max_contacts_per_pair=2`, `step_size_scale=10`, residual `9.9001648754006507e-07`, one exact solve, zero exact-FBF failures, and zero boxed-LCP fallback; not full Fig. 8 parity |

## Benchmarks

Build the benchmark and trace exporter:

```bash
pixi run cmake --build build/default/cpp/Release --target BM_INTEGRATION_exact_coulomb_fbf_paper --parallel 8
pixi run cmake --build build/default/cpp/Release --target fbf_paper_trace --parallel 8
pixi run cmake --build build/default/cpp/Release --target fbf_paper_gamma_sweep --parallel 8
pixi run cmake --build build/default/cpp/Release --target fbf_paper_card_house_probe --parallel 8
pixi run cmake --build build/default/cpp/Release --target fbf_paper_arch_probe --parallel 8
```

List benchmark rows:

```bash
./build/default/cpp/Release/bin/BM_INTEGRATION_exact_coulomb_fbf_paper --benchmark_list_tests
```

Run all current benchmark rows:

```bash
./build/default/cpp/Release/bin/BM_INTEGRATION_exact_coulomb_fbf_paper --benchmark_filter='BM_PaperFixtureStepTime/.*' --benchmark_min_time=0.001s --benchmark_repetitions=1
```

Rows currently registered:

- `incline_mu_0_5_boxed_lcp`
- `incline_mu_0_5_exact_fbf`
- `incline_mu_0_4_boxed_lcp`
- `incline_mu_0_4_exact_fbf`
- `backspin_boxed_lcp`
- `backspin_exact_fbf`
- `turntable_mu_0_2_omega_2_boxed_lcp`
- `turntable_mu_0_2_omega_2_exact_fbf`
- `turntable_mu_0_2_omega_5_boxed_lcp`
- `turntable_mu_0_2_omega_5_exact_fbf`
- `turntable_mu_0_5_omega_2_boxed_lcp`
- `turntable_mu_0_5_omega_2_exact_fbf`
- `turntable_mu_0_5_omega_5_boxed_lcp`
- `turntable_mu_0_5_omega_5_exact_fbf`
- `painleve_mu_0_5_boxed_lcp`
- `painleve_mu_0_5_exact_fbf`
- `painleve_mu_0_55_boxed_lcp`
- `painleve_mu_0_55_exact_fbf`
- `card_house_26_reduced_contact_boxed_lcp`
- `card_house_26_reduced_contact_exact_fbf`
- `card_house_26_settle_projectile_reduced_contact_boxed_lcp`
- `card_house_26_settle_projectile_reduced_contact_exact_fbf`
- `card_house_10_construction_boxed_lcp`
- `masonry_arch_25_reduced_contact_boxed_lcp`
- `masonry_arch_25_reduced_contact_exact_fbf`
- `masonry_arch_25_projectile_reduced_contact_boxed_lcp`
- `masonry_arch_25_projectile_reduced_contact_exact_fbf`
- `masonry_arch_101_reduced_contact_boxed_lcp`
- `masonry_arch_101_reduced_contact_exact_fbf`

Latest local benchmark evidence from this branch, Release build. Rows not
touched by the final fixture-step update are from the 2026-07-08 21:00:43
-0700 all-row smoke, single repetition, `--benchmark_min_time=0.001s`, 32
reported CPU threads, load average `7.70, 10.50, 6.53`, and CPU-scaling
warning present. The incline, reduced 26-card card-house, and reduced 25/101
masonry-arch exact rows below were rerun after the paper residual-scale and
fixture-step updates on 2026-07-09. The reduced 25-stone arch projectile rows
were added and run on 2026-07-09 with `--benchmark_min_time=0.01s`:

| Row | Real time | Exact solves | Failures | Fallbacks | Max residual | Max FBF iters |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `incline_mu_0_5_boxed_lcp` | 0.470 ms | 0 | 0 | 0 | n/a | 0 |
| `incline_mu_0_5_exact_fbf` | 21.7 ms | 120 | 0 | 0 | `9.998e-7` | 382 |
| `incline_mu_0_4_boxed_lcp` | 0.468 ms | 0 | 0 | 0 | n/a | 0 |
| `incline_mu_0_4_exact_fbf` | 3.62 ms | 120 | 0 | 0 | `9.991e-7` | 59 |
| `backspin_boxed_lcp` | 0.626 ms | 0 | 0 | 0 | n/a | 0 |
| `backspin_exact_fbf` | 1.27 ms | 240 | 0 | 0 | `9.999e-7` | 48 |
| `turntable_mu_0_2_omega_2_boxed_lcp` | 1.47 ms | 0 | 0 | 0 | n/a | 0 |
| `turntable_mu_0_2_omega_2_exact_fbf` | 28.1 ms | 145 | 0 | 0 | `9.994e-7` | 59 |
| `turntable_mu_0_2_omega_5_boxed_lcp` | 0.969 ms | 0 | 0 | 0 | n/a | 0 |
| `turntable_mu_0_2_omega_5_exact_fbf` | 12.8 ms | 100 | 0 | 0 | `9.925e-7` | 54 |
| `turntable_mu_0_5_omega_2_boxed_lcp` | 1.57 ms | 0 | 0 | 0 | n/a | 0 |
| `turntable_mu_0_5_omega_2_exact_fbf` | 156 ms | 240 | 0 | 0 | `9.999e-7` | 155 |
| `turntable_mu_0_5_omega_5_boxed_lcp` | 0.925 ms | 0 | 0 | 0 | n/a | 0 |
| `turntable_mu_0_5_omega_5_exact_fbf` | 37.5 ms | 89 | 0 | 0 | `9.953e-7` | 436 |
| `painleve_mu_0_5_boxed_lcp` | 0.636 ms | 0 | 0 | 0 | n/a | 0 |
| `painleve_mu_0_5_exact_fbf` | 17.7 ms | 150 | 0 | 0 | `9.992e-7` | 448 |
| `painleve_mu_0_55_boxed_lcp` | 0.678 ms | 0 | 0 | 0 | n/a | 0 |
| `painleve_mu_0_55_exact_fbf` | 25.0 ms | 142 | 0 | 0 | `9.986e-7` | 241 |
| `card_house_26_reduced_contact_boxed_lcp` | 0.643 ms | 0 | 0 | 0 | n/a | 0 |
| `card_house_26_reduced_contact_exact_fbf` | 5028 ms | 3 | 0 | 0 | `2.218e-16` | 1908 |
| `card_house_26_settle_projectile_reduced_contact_boxed_lcp` | 2.17 ms | 0 | 0 | 0 | n/a | 0 |
| `card_house_26_settle_projectile_reduced_contact_exact_fbf` | 12057 ms | 6 | 0 | 0 | `2.218e-16` | 3631 |
| `card_house_10_construction_boxed_lcp` | 1010 ms | 0 | 0 | 0 | n/a | 0 |
| `masonry_arch_25_reduced_contact_boxed_lcp` | 0.597 ms | 0 | 0 | 0 | n/a | 0 |
| `masonry_arch_25_reduced_contact_exact_fbf` | 6514 ms | 1 | 0 | 0 | `9.9650461738801354e-07` | 2645 |
| `masonry_arch_25_projectile_reduced_contact_boxed_lcp` | 0.497 ms | 0 | 0 | 0 | n/a | 0 |
| `masonry_arch_25_projectile_reduced_contact_exact_fbf` | 4921 ms | 1 | 0 | 0 | `9.9901843912475669e-07` | 2205 |
| `masonry_arch_101_reduced_contact_boxed_lcp` | 0.345 ms | 0 | 0 | 0 | n/a | 0 |
| `masonry_arch_101_reduced_contact_exact_fbf` | 2365 ms | 1 | 0 | 0 | `9.9001648754006507e-07` | 3912 |

These are DART-side smoke timings, not paper-hardware parity numbers. The
card-house row is a reduced-contact one-step benchmark with 56 contacts,
`step_size_scale=10`, `outer_relaxation=1.5`, 3 exact solves, zero exact
failures, zero boxed-LCP fallbacks, and total FBF iterations `1908`; it is not
the paper's full 26-card settle/projectile timing. The reduced
settle/projectile row is a two-step scaffold at the same 56-contact cap with
four launched projectiles, 6 exact solves, zero exact failures/fallbacks, max
FBF iterations `3631`, total FBF iterations `5539`, and max residual
`2.218e-16`; it is still not the paper's 6.7 s no-creep settle, 10 s impact
outcome, or full-contact timing. The 10-level row is a
boxed-LCP construction/default-solver row with 155 cards and 512 contacts; it is
not exact-FBF dynamic parity. The masonry-arch rows are reduced-contact one-step
benchmarks with 48 contacts for the 25-stone arch and 38 contacts for the
101-stone arch, two contacts per pair, one exact solve, zero exact failures,
and zero boxed-LCP fallbacks; the 25-stone row reported max/total FBF
iterations `2645`, while the 101-stone row reported `3912`. Higher arch
contact-cap rows are reported by `fbf_paper_arch_probe` below; none of these
replace the paper's author-geometry pinned/projectile or long-run arch timings.

## Card-House Cap Probe

The benchmark-only cap probe emits one CSV row per requested reduced contact
cap:

```bash
./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_card_house_probe 56,58,59,60,64
./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_card_house_probe 56,60 1 30000 120 32 200 nan 0 0 0.9 10 1.5 30001
./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_card_house_probe 32 1 1000 120 32 200 nan 0 0 0.9 10 1.5 0 0.003 0.55 1
./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_card_house_probe 16 1 30000 120 32 200 nan 0 0 0.9 10 1.5 0 0.003 0.55 1 1
```

Columns include contact caps, exact/FBF status, residual components, failed
residual components, best/final residual comparison columns, projected-gradient
retry and dense-residual-polish counters, fallback counters, max/total FBF
iterations, step-size diagnostics, card-state sanity metrics, option knobs for
diagnostic runs, `use_matrix_free_delassus_operator`,
`use_matrix_free_delassus_seed`, `matrix_free_delassus_operator_used`,
`matrix_free_delassus_seed_used`, and optional retained residual-history row
counts plus failed-history first/last residual summaries when a positive sample
cap is supplied. The helper is not an installed API and does not add core DART
dependencies.

Latest local focused dense cap-probe boundary after the residual-scale,
dense-polish, and matrix-free-product staging updates:

| Max contacts | Clean exact | Contacts | Elapsed | Exact solves | Failures | Fallbacks | PG retries | Dense polishes | Max/failed FBF iter | Failed residual |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 56 | 1 | 56 | 4874 ms | 3 | 0 | 0 | 0 | 0 | 1908 | n/a |
| 58 | 1 | 58 | 68879 ms | 3 | 0 | 0 | 1 | 0 | 64 | n/a |
| 59 | 1 | 59 | 76403 ms | 3 | 0 | 0 | 1 | 0 | 7888 | n/a |
| 60 | 0 | 60 | 12523 ms | 1 | 1 | 1 | 1 | 1 | 3000 | `6.166e-5` |

The 59-contact row is now clean but depends on the projected-gradient retry
and is too slow to promote into the default test/benchmark/GUI rung. The
60-contact row is the current one-step fallback boundary and remains
dual-feasibility dominated at the default card-house options. The latest
recorded 64-contact diagnostic from this branch still has one exact-FBF
`MaxIterations` failure, one boxed-LCP fallback, and failed dual residual
`6.955e-6`; it has not been re-cleared by the local/global seed or retry
continuation.

The 58- and 59-contact rows are clean under the paper-scaled residual, but
they remain too slow to promote into the default test/benchmark/GUI rung. The
60-contact row is still the current one-step fallback boundary.

## Masonry-Arch Cap Probe

The benchmark-only arch cap probe emits one CSV row per requested reduced
contact cap for the current approximate 25/101-stone masonry-arch scaffolds:

```bash
pixi run cmake --build build/default/cpp/Release --target fbf_paper_arch_probe --parallel 8
pixi run build/default/cpp/Release/tests/benchmark/integration/fbf_paper_arch_probe
pixi run build/default/cpp/Release/tests/benchmark/integration/fbf_paper_arch_probe arch25 48,49 2 20000
pixi run build/default/cpp/Release/tests/benchmark/integration/fbf_paper_arch_probe arch25 64 4 20000
pixi run timeout 120s build/default/cpp/Release/tests/benchmark/integration/fbf_paper_arch_probe arch25 80 4 20000
pixi run timeout 120s build/default/cpp/Release/tests/benchmark/integration/fbf_paper_arch_probe arch25 100 4 20000
pixi run build/default/cpp/Release/tests/benchmark/integration/fbf_paper_arch_probe arch101 38,39 2 20000
pixi run build/default/cpp/Release/tests/benchmark/integration/fbf_paper_arch_probe arch101 40,48 2 20000
pixi run build/default/cpp/Release/tests/benchmark/integration/fbf_paper_arch_probe arch101 56,64 2 20000
pixi run timeout 120s build/default/cpp/Release/tests/benchmark/integration/fbf_paper_arch_probe arch101 80 2 20000
pixi run timeout 120s build/default/cpp/Release/tests/benchmark/integration/fbf_paper_arch_probe arch101 100 2 20000
```

Columns include scenario, requested and actual contacts, exact/FBF status,
residual components, failed residual components and contact-pair labels,
fallback counters, projected-gradient retry and dense-polish counters,
max/total FBF iterations, step-size diagnostics, matrix-free route flags,
optional residual-history counts, and crown/state sanity metrics.

Latest local arch cap-probe evidence:

| Scenario | Request | Actual contacts | Clean exact | Elapsed | Residual | FBF iters | Note |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 25-stone | 48/2 | 48 | 1 | 6679.8 ms | `9.965e-7` | 2645 | Default promoted rung |
| 25-stone | 49/2 | 48 | 1 | 6798.2 ms | `9.965e-7` | 2645 | Geometry/contact generation saturates at 48 |
| 25-stone | 64/4 | 64 | 1 | 31118.9 ms | `9.993e-7` | 5186 | Clean but expensive |
| 25-stone | 80/4 | 80 | 1 | 117628.4 ms | `9.890e-7` | 7310 | Clean but near the 120 s guard |
| 25-stone | 100/4 | n/a | n/a | timed out | n/a | n/a | `timeout 120s` exited `124` before a row |
| 101-stone | 38/2 | 38 | 1 | 2439.5 ms | `9.900e-7` | 3912 | Default promoted rung |
| 101-stone | 39/2 | 39 | 1 | 2967.4 ms | `9.675e-7` | 4548 | Corrects stale fallback-boundary note |
| 101-stone | 48/2 | 48 | 1 | 13686.5 ms | `9.949e-7` | 11472 | Clean |
| 101-stone | 64/2 | 64 | 1 | 40638.6 ms | `9.997e-7` | 17474 | Clean |
| 101-stone | 80/2 | 80 | 1 | 30422.9 ms | `9.996e-7` | 7312 | Clean |
| 101-stone | 100/2 | 100 | 1 | 56137.1 ms | `9.944e-7` | 6892 | Clean one-step cap row |

These rows are not paper parity. They use the current approximate DART arch
geometry, run one step, and do not include the paper's 25-stone projectile/rest
outcome, 101-stone long-run balance outcome, snapshots, or external timings.

Latest matrix-free-product probe smoke rows:

| Max contacts | Matrix-free product requested/used | Matrix-free seed requested/used | Clean exact | Contacts | Elapsed | Max FBF iters | Failed residual |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 16 | 0/0 | 0/0 | 1 | 16 | 24.4 ms | 336 | n/a |
| 16 | 1/1 | 0/0 | 1 | 16 | 354.1 ms | 336 | n/a |
| 16 | 1/1 | 1/1 | 1 | 16 | 354.5 ms | 336 | n/a |
| 32 | 0/0 | 0/0 | 1 | 32 | 170.4 ms | 453 | n/a |
| 32 | 1/1 | 0/0 | 1 | 32 | 3203.0 ms | 453 | n/a |
| 40 | 0/0 | 0/0 | 0 | 40 | 1057.4 ms | 1000 | `3.966e-5` |
| 40 | 1/1 | 0/0 | 0 | 40 | 18937.8 ms | 1000 | `3.966e-5` |

These rows prove the benchmark surface can exercise and report the
constraint-row Delassus product route on reduced card-house cases. They also
show that repeated impulse-test `W*x` products are not the final performance
answer. The operator-local seed removes one dense-seed dependency on the
diagnostic route but does not materially change the 16-contact timing; larger
`32,40` and `56,60` matrix-free product attempts hit a 120 s bound before a
first CSV row. The next matrix-free work needs scratch-backed body
scatter/inverse-mass/gather or an explicit dynamics/constraint API extension,
not just this diagnostic route. A local solver-only batched product experiment
was tried and reverted because generic `ConstraintBase` does not expose safe
touched-body bias-impulse propagation.

The latest history-enabled focused cap probe:

| Max contacts | Clean exact | Elapsed | Records | Rows | Failed rows | Dense polishes | Failed last iter | Failed first residual | Failed best iter | Failed best/final residual | Pre-polish history last | Tail ratio |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 56 | 1 | 4971 ms | 3 | 1911 | 0 | 0 | 0 | n/a | n/a | n/a | n/a | n/a |
| 60 | 0 | 94606 ms | 2 | 30002 | 30001 | 1 | 30000 | `1.146e-5` | 29355 | `5.334e-6` | `5.338e-6` | `1.0000019` |

This confirms that the current 60-contact failure reaches the configured
30000 outer-iteration cap with slow near-cap dual convergence. Best-iterate
retention and dense residual polish improve the failed reaction/residual
reported at the end of the solve, but the result is still above `1e-6`, so
this is not a missing retained-history sample or a reporting-only issue.

Additional option diagnostics:

| Max contacts | Step-size scale | Outer relaxation | Clean exact | Elapsed | Exact solves | Failures | Fallbacks | Max FBF iters | Failed residual |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 60 | 12 | 1.5 | 0 | 108620 ms | 1 | 1 | 1 | 30000 | `3.005e-6` |
| 60 | 10 | 2.0 | 0 | 142263 ms | 1 | 1 | 1 | 30000 | `9.257e-6` |

The default test/benchmark/GUI rung therefore remains at 56 contacts. The next
paper-parity blocker is removing the 60-contact fallback without relying on a
near-cap or multi-minute diagnostic run.

Short 5000-outer-iteration diagnostics were also run to avoid repeating
unhelpful option directions. The current default option set is the best tested
short-cap row. `step_size_scale=8`, `step_size_scale=12`,
`outer_relaxation=1.0`, and `outer_relaxation=1.8` were worse at 5000
iterations; increasing the projected-gradient retry cap to `1000` and
doubling block-GS sweeps/local iterations to `240`/`64` did not change the
failed residual. A residual-selected over-relaxation guard was also tested and
reverted because it made 56 contacts slower and left 60 contacts failing with
a worse failed residual. After the best-iterate diagnostic slice, the short
`56,60 ... 5001` run kept 56 contacts clean in `4722 ms` and reported the
60-contact failed group's best residual equal to the final residual
`2.635e-5` at iteration `5000`.

## Gamma Sweep

The DART-side sweep helper emits one CSV row per `initial_gamma` value for
the small fixtures and reduced one-step contact-rich scaffolds. The optional
last two arguments set `max_contacts` and `max_contacts_per_pair`; the CSV
records both configured caps so reduced, boundary, and future full-contact rows
are not confused:

```bash
./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_gamma_sweep backspin nan,0.5,0.05,0.01 240 1 1
./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_gamma_sweep card_house_26_reduced_contact nan 1 56 1
./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_gamma_sweep masonry_arch_25_reduced_contact nan,0.001 1 48 2
./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_gamma_sweep masonry_arch_101_reduced_contact nan,0.001 1 38 2
```

Columns include:

- `gamma_mode`, `gamma`, `steps`, `max_contacts`,
  `max_contacts_per_pair`, and the solver-budget columns
  `max_outer_iterations`, `inner_sweeps`, `inner_local_iterations`,
  `projected_gradient_iterations`, `step_size_scale`, and
  `outer_relaxation`,
- `clean_exact`, `exact_solves`, `warm_starts`, and `fallbacks`,
- residual sample/failure/non-finite counters,
- max/final residual and final exact status,
- `physical_outcome` plus final position, velocity, up-axis, radius, and
  angular velocity columns.

Latest local backspin sweep smoke:

| Gamma | Clean exact | Outcome | Exact solves | Fallbacks | Max residual | Final status |
| ---: | ---: | --- | ---: | ---: | ---: | --- |
| safe | 1 | `reversed` | 240 | 0 | `9.943e-7` | `success` |
| `0.5` | 1 | `reversed` | 240 | 0 | `9.943e-7` | `success` |
| `0.05` | 1 | `reversed` | 240 | 0 | `9.998e-7` | `success` |
| `0.01` | 0 | `reversed` | 50 | 190 | `9.997e-7` | `success` |

Latest local reduced-scaffold sweep smoke:

| Scenario | Gamma | Max contacts | Max per pair | Clean exact | Outcome | Contacts | Exact solves | Fallbacks | Final residual | Final status |
| --- | ---: | ---: | ---: | ---: | --- | ---: | ---: | ---: | ---: | --- |
| `card_house_26_reduced_contact` | safe | 56 | 1 | 1 | `reduced_one_step_finite` | 56 | 3 | 0 | `2.2183427389532158e-16` | `success` |
| `masonry_arch_25_reduced_contact` | safe | 48 | 2 | 1 | `reduced_one_step_supported` | 48 | 1 | 0 | `9.9650461738801354e-07` | `success` |
| `masonry_arch_25_reduced_contact` | `0.001` | 48 | 2 | 0 | `reduced_one_step_supported` | 48 | 0 | 1 | `1.631e-3` | `boxed_lcp_fallback` |
| `masonry_arch_101_reduced_contact` | safe | 38 | 2 | 1 | `reduced_one_step_supported` | 38 | 1 | 0 | `9.9001648754006507e-07` | `success` |
| `masonry_arch_101_reduced_contact` | `0.001` | 38 | 2 | 0 | `reduced_one_step_supported` | 38 | 0 | 1 | `1.454e-3` | `boxed_lcp_fallback` |

This is still reduced one-step scaffold evidence. It is not the paper's full
Figure 10 house/arch sweep with full-contact long-run physical outcomes.

## Trace Exporter

Trace commands:

```bash
./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace backspin exact_fbf 120
./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace backspin boxed_lcp 120
./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace backspin exact_fbf 120 240 0.05
./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace backspin exact_fbf 1 1 nan residual_history
./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace card_house_26_reduced_contact exact_fbf 1
./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace card_house_26_reduced_contact boxed_lcp 1
./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace card_house_26_reduced_contact exact_fbf 1 1 nan dynamic_bodies
./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace card_house_26_reduced_contact boxed_lcp 1 1 nan dynamic_bodies
./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace card_house_26_reduced_contact exact_fbf 1 1 nan residual_history
./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace card_house_26_settle_projectile_reduced_contact exact_fbf 1 2 nan phase_summary
./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace card_house_26_settle_projectile_reduced_contact boxed_lcp 1 2 nan phase_summary
./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace masonry_arch_25_reduced_contact exact_fbf 1
./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace masonry_arch_25_reduced_contact boxed_lcp 1
./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace masonry_arch_25_reduced_contact exact_fbf 1 1 nan dynamic_bodies
./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace masonry_arch_25_reduced_contact exact_fbf 1 1 nan residual_history
./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace masonry_arch_25_projectile_reduced_contact exact_fbf 1 1 nan tracked
./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace masonry_arch_25_projectile_reduced_contact boxed_lcp 1 1 nan tracked
./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace masonry_arch_25_projectile_reduced_contact exact_fbf 1 1 nan dynamic_bodies
./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace masonry_arch_101_reduced_contact exact_fbf 1
./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace masonry_arch_101_reduced_contact boxed_lcp 1
./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace masonry_arch_101_reduced_contact exact_fbf 1 1 nan dynamic_bodies
./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace masonry_arch_101_reduced_contact exact_fbf 1 1 nan residual_history
```

The optional fifth argument is `initial_gamma`. Omit it or pass `nan` to use
the safe spectral default. Positive finite values allow small-fixture
step-size sweep scripting for trace output; this is not the paper's full
Figure 10 house/arch sweep. The optional sixth argument is the trace scope:
`tracked` by default, `dynamic_bodies`/`full_scene` for one CSV row per mobile
body, or `residual_history` for bounded exact-FBF per-outer residual rows from
retained exact-solve records, including multiple exact groups in one sampled
step. The reduced 26-card settle/projectile scaffold also supports
`phase_summary`, which emits one row each for the initial, settle, and
projectile phases. In the current backspin smoke, `0.05` passes with zero boxed-LCP
fallbacks; smaller values such as `0.005` and `0.01` return nonzero with
boxed-LCP fallback, which is a sweep outcome rather than a PR gate.

Supported trace scenarios:

- `incline_mu_0_5`
- `incline_mu_0_4`
- `backspin`
- `turntable_mu_0_2_omega_2`
- `turntable_mu_0_2_omega_5`
- `turntable_mu_0_5_omega_2`
- `turntable_mu_0_5_omega_5`
- `painleve_mu_0_5`
- `painleve_mu_0_55`
- `card_house_26_reduced_contact`
- `card_house_26_settle_projectile_reduced_contact`
- `masonry_arch_25_reduced_contact`
- `masonry_arch_25_projectile_reduced_contact`
- `masonry_arch_101_reduced_contact`

Supported trace solvers:

- `exact_fbf`
- `boxed_lcp`

Supported trace scopes:

- `tracked`
- `dynamic_bodies`
- `full_scene`
- `residual_history` for `exact_fbf` only
- `phase_summary` for `card_house_26_settle_projectile_reduced_contact`

## Residual-History Plot Artifacts

The current reduced-scaffold Figure 9 evidence is generated from the
`residual_history` trace CSVs:

```bash
./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace backspin exact_fbf 1 1 nan residual_history > /tmp/fbf_backspin_residual_history.csv
./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace card_house_26_reduced_contact exact_fbf 1 1 nan residual_history > /tmp/fbf_card_house_residual_history.csv
./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace masonry_arch_25_reduced_contact exact_fbf 1 1 nan residual_history > /tmp/fbf_arch25_residual_history.csv
./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace masonry_arch_101_reduced_contact exact_fbf 1 1 nan residual_history > /tmp/fbf_arch101_residual_history.csv
pixi run python tests/benchmark/integration/fbf_paper_residual_svg.py /tmp/fbf_backspin_residual_history.csv docs/dev_tasks/fbf_exact_coulomb_friction/assets/fbf_backspin_residual_history.svg --title "Backspin residual history"
pixi run python tests/benchmark/integration/fbf_paper_residual_svg.py /tmp/fbf_card_house_residual_history.csv docs/dev_tasks/fbf_exact_coulomb_friction/assets/fbf_card_house_residual_history.svg --title "Reduced 26-card residual history"
pixi run python tests/benchmark/integration/fbf_paper_residual_svg.py /tmp/fbf_arch25_residual_history.csv docs/dev_tasks/fbf_exact_coulomb_friction/assets/fbf_arch25_residual_history.svg --title "Reduced 25-stone arch residual history"
pixi run python tests/benchmark/integration/fbf_paper_residual_svg.py /tmp/fbf_arch101_residual_history.csv docs/dev_tasks/fbf_exact_coulomb_friction/assets/fbf_arch101_residual_history.svg --title "Reduced 101-stone arch residual history"
pixi run python tests/benchmark/integration/fbf_paper_residual_svg.py \
  --title "Current reduced exact-FBF residual histories" \
  --output docs/dev_tasks/fbf_exact_coulomb_friction/assets/fbf_residual_history_panel.svg \
  --panel /tmp/fbf_backspin_residual_history.csv "Backspin, first exact step" \
  --panel /tmp/fbf_card_house_residual_history.csv "Reduced 26-card, one step" \
  --panel /tmp/fbf_arch25_residual_history.csv "Reduced 25-stone arch, one step" \
  --panel /tmp/fbf_arch101_residual_history.csv "Reduced 101-stone arch, one step"
```

Latest regenerated local results:

| Scenario | Data rows | Exact groups | Last group contacts | Last outer iteration | Last residual | Fallbacks |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `backspin` | 18 | 1 | 1 | 17 | `7.8906612729366679e-07` | 0 |
| `card_house_26_reduced_contact` | 1911 | 3 | 1 | 0 | `2.2183427389532158e-16` | 0 |
| `masonry_arch_25_reduced_contact` | 2646 | 1 | 48 | 2645 | `9.9650461738801354e-07` | 0 |
| `masonry_arch_101_reduced_contact` | 3913 | 1 | 38 | 3912 | `9.9001648754006507e-07` | 0 |

Artifacts:

- `docs/dev_tasks/fbf_exact_coulomb_friction/assets/fbf_backspin_residual_history.svg`
- `docs/dev_tasks/fbf_exact_coulomb_friction/assets/fbf_card_house_residual_history.svg`
- `docs/dev_tasks/fbf_exact_coulomb_friction/assets/fbf_arch25_residual_history.svg`
- `docs/dev_tasks/fbf_exact_coulomb_friction/assets/fbf_arch101_residual_history.svg`
- `docs/dev_tasks/fbf_exact_coulomb_friction/assets/fbf_residual_history_panel.svg`

These plots are still reduced-scaffold diagnostics. They are not a substitute
for full-contact, long-run, paper-matched Figure 9 comparisons.

## Verification Checklist For This PR

Before publishing or updating a PR, rerun and update this report with:

- `pixi run demos -- --list-scenes`
- `pixi run demos -- --verify-fbf-scene-docs`
- `pixi run demos -- --cycle-scenes --frames 1`
- one-step GUI captures through
  `pixi run capture <scene_id> <out.png> 640 480 1` plus
  `pixi run image-verdict <out.png>` when GUI evidence changes
- action-state GUI captures through
  `pixi run capture-action <scene_id> <key> <out.png> <width> <height> <steps>`
  plus `pixi run image-verdict <out.png>` when an example exposes important
  state through a scene key action
- focused build targets and CTest commands listed above
- reduced-contact 26-card one-step probe with XML properties
- reduced-contact 26-card settle/projectile scaffold CTest
- reduced-contact 25-stone masonry-arch one-step probe with XML properties
- reduced-contact 25-stone masonry-arch projectile scaffold test, trace,
  benchmark, and action capture
- reduced-contact 101-stone masonry-arch one-step probe with XML properties
- 10-level card-house construction scene-build test and boxed-LCP benchmark row
- card-house contact-cap probe boundary
- history-enabled card-house cap-probe row when diagnosing a fallback boundary
- benchmark list and representative/all benchmark rows
- exact-FBF and boxed-LCP trace smoke commands
- reduced-contact 26-card card-house exact-FBF and boxed-LCP trace smoke
  commands
- reduced-contact 26-card settle/projectile `phase_summary` trace smoke
  commands under exact-FBF and boxed-LCP
- reduced-contact contact-rich `dynamic_bodies` trace smoke commands
- exact-FBF `residual_history` trace smoke commands, including contact-rich
  multi-group row counts
- residual-history SVG plot generation, including the combined panel
- reduced-contact masonry-arch exact-FBF and boxed-LCP trace smoke commands
- reduced-contact 101-stone masonry-arch exact-FBF and boxed-LCP trace smoke
  commands
- small-fixture and reduced contact-rich gamma sweep smokes
- focused reduced 26-card settle/projectile benchmark rows
- `pixi run docs-build`
- `pixi run lint`
- `pixi run build`
- `git diff --check`
- untracked-file whitespace check while the new files remain untracked

Any failed or skipped command must be recorded with the concrete blocker and
the next command to run.

Latest local verification in this branch:

- After adding the reduced 25-stone masonry-arch projectile scaffold on
  2026-07-09,
  `pixi run cmake --build build/default/cpp/Release --target test_ExactCoulombFbfPaperFixtures fbf_paper_trace BM_INTEGRATION_exact_coulomb_fbf_paper dart-demos --parallel 8`
  passed after formatting. The focused
  `ExactCoulombFbfPaperFixtures.MasonryArch25ProjectileScaffoldRuns` binary
  filter passed in `10611 ms`. Exact-FBF
  `fbf_paper_trace masonry_arch_25_projectile_reduced_contact exact_fbf 1 1 nan tracked`
  ended with 48 contacts, one exact solve, zero fallbacks, residual
  `9.9901843912475669e-07`, and `success`; the boxed-LCP tracked trace and
  exact-FBF `dynamic_bodies` trace also emitted rows. The focused benchmark
  filter for `masonry_arch_25_projectile_reduced_contact_(boxed_lcp|exact_fbf)$`
  reported boxed LCP `0.497 ms` and exact FBF `4921 ms`, one exact solve, zero
  failures/fallbacks, max residual `999.018n`, and `2205` max/total FBF
  iterations. `pixi run demos -- --verify-fbf-scene-docs` checked all nine FBF
  scenes; `pixi run demos -- --list-scenes` listed
  `fbf_paper_masonry_arch_25` under Research; and the refreshed
  `fbf_paper_masonry_arch_25` action capture plus `pixi run image-verdict`
  passed. A parallel local build attempt against the same Ninja tree hit a
  generated-deps race; rerun build/demo commands serially.

- After adding action-aware headless captures on 2026-07-09,
  `pixi run cmake --build build/default/cpp/Release --target dart-demos --parallel 8`
  passed. The `capture-action` command for `fbf_paper_card_house_26` with key
  `p`, output
  `assets/gui_captures/fbf_paper_card_house_26_projectiles.png`, size
  1280x720, and zero steps invoked the 26-card scene's projectile key action
  before the shot and wrote a visible four-projectile scaffold capture.
  `pixi run image-verdict` on that PNG passed the default non-blank gate, with
  contrast still weak but not required.
  `pixi run demos -- --verify-fbf-scene-docs` passed when rerun serially. A
  parallel verifier/capture-action run reproduced the known `dart-demos`
  archive/link race, so keep demo build/capture tasks serial.

- After refreshing `AGENT_CONTINUATION.md`, `README.md`, `RESUME.md`,
  `gui-capture-report.md`, `paper-parity-matrix.md`, and this report for the
  action-aware 26-card GUI capture on 2026-07-09, the FBF scene-doc verifier
  checked all nine FBF scenes, `pixi run docs-build` passed, `pixi run lint`
  passed, `pixi run build` passed, `git diff --check` passed, and the
  output-based binary-skipping untracked-file whitespace loop produced no
  diagnostics.

- After the reduced 26-card settle/projectile phase scaffold on 2026-07-09,
  `pixi run cmake --build build/default/cpp/Release --target test_ExactCoulombFbfPaperFixtures fbf_paper_trace BM_INTEGRATION_exact_coulomb_fbf_paper dart-demos --parallel 8`
  passed. The focused
  `ExactCoulombFbfPaperFixtures.CardHouseFourLevelSettleProjectileScaffoldRuns`
  CTest passed, launching four projectiles after one bounded settle step with
  zero exact-FBF failures or boxed-LCP fallback. Exact-FBF
  `phase_summary` trace for
  `card_house_26_settle_projectile_reduced_contact` emitted initial, settle,
  and projectile rows; the final projectile row reported 26 cards, 4
  projectiles, finite state, 56 contacts, 6 exact solves, zero fallbacks,
  residual `4.8919831295901383e-17`, and `success`. The boxed-LCP
  `phase_summary` trace emitted the same three phase rows. Focused benchmark
  rows passed: boxed-LCP ran in `2.17 ms`, exact-FBF ran in `12057 ms` with 56
  contacts, 6 exact solves, zero failures/fallbacks, max FBF iterations
  `3631`, total FBF iterations `5539`, and max residual `2.218e-16`.
  `pixi run demos -- --verify-fbf-scene-docs` checked all nine FBF scenes, and
  direct `./build/default/cpp/Release/bin/dart-demos --list-scenes` confirmed
  `fbf_paper_card_house_26` under `Research`. This is still reduced
  two-step scaffold evidence, not full Fig. 6 parity.

- After adding the operator-local Delassus seed and reverting an unsafe
  batched-product experiment on 2026-07-09,
  `pixi run build && pixi run cmake --build build/default/cpp/Release --target test_ExactCoulombConstraintAdapter test_ExactCoulombFbfConstraintSolver test_ConstraintSolver fbf_paper_card_house_probe --parallel 8`
  passed. `ctest --test-dir build/default/cpp/Release -R
  '(test_ExactCoulombConstraintAdapter|test_ExactCoulombFbfConstraintSolver|test_ConstraintSolver)'
  --output-on-failure` passed `3/3`.

- Final-code matrix-free seed probe rows passed:
  `fbf_paper_card_house_probe 16 1 30000 120 32 200 nan 0 0 0.9 10 1.5 0 0.003 0.55 1 0`
  and the same command with trailing `1 1` both solved with residual
  `3.212e-16`, 16 contacts, five exact solves, zero failures/fallbacks, and
  336 total FBF iterations. The CSV proved
  `matrix_free_delassus_seed_used=0` versus `1`; elapsed time was `354.1 ms`
  versus `354.5 ms`.

- Larger matrix-free product probes remain blocked by product cost: `32,40`
  and `56,60` matrix-free attempts under a 120 s timeout did not produce a
  first CSV row. The old dense boundary evidence remains: promoted 56 contacts
  is clean, 58/59 are clean but too slow, and 60 contacts remains the current
  fallback boundary.

- After the final fixture-step update on 2026-07-09,
  `pixi run cmake --build build/default/cpp/Release --target test_ExactCoulombFbfPaperFixtures fbf_paper_trace fbf_paper_gamma_sweep BM_INTEGRATION_exact_coulomb_fbf_paper dart-demos --parallel 8`
  passed, then the focused CTest regex covering math, adapter, solver, world
  smoke, and paper fixtures passed `7/7`. The final
  reduced arch XML probe passed with the 25-stone row at 48 contacts,
  residual `9.9650461738801354e-07`, and max/total FBF iterations `2645`;
  the 101-stone row at 38 contacts, residual
  `9.9001648754006507e-07`, and max/total FBF iterations `3912`. Final
  default gamma-sweep rows were clean for both incline cases, the 56-contact
  reduced card-house row, and the 48/38-contact reduced arch rows. The focused
  exact-FBF benchmark rows reran successfully with zero exact failures and
  zero boxed-LCP fallbacks. The FBF GUI verifier checked all nine
  `fbf_paper_*` scene explanations, the scene-list command found all nine
  scene IDs, and `pixi run demos -- --cycle-scenes --frames 1` cycled 43
  demos x2 with only known non-FBF warnings. `pixi run docs-build`,
  `pixi run lint`, `pixi run build`, `git diff --check`, and the untracked
  whitespace loop passed after the final report edits.

- After the paper residual-scale slice on 2026-07-09,
  `pixi run cmake --build build/default/cpp/Release --target UNIT_math_ExactCoulombFbfSolver test_ExactCoulombFbfConstraintSolver fbf_paper_card_house_probe --parallel 8`
  passed. `ctest --test-dir build/default/cpp/Release -R
  '(UNIT_math_ExactCoulombFbfSolver|test_ExactCoulombFbfConstraintSolver)'
  --output-on-failure` passed `2/2`. The paper-scaled
  `fbf_paper_card_house_probe 56,60 1 3000 120 32 200 nan 0 0 0.9 10 1.5 3001`
  diagnostic kept 56 clean and showed 60 still falling back at failed
  residual `6.800e-5`. The paper-scaled `58,59 1 30000 ... 0` run kept both
  rows clean but too slow. The full paper-scaled
  `fbf_paper_card_house_probe 60 1 30000 120 32 200 nan 0 0 0.9 10 1.5 30001`
  diagnostic still fell back, with one dense polish, best/final failed
  residual `5.334e-6` at iteration `29355`, pre-polish history last residual
  `5.338e-6`, and tail ratio `1.0000019`.

- Final gates for the dense residual-polish tracker/report refresh on
  2026-07-09: after the report/tracker/README/parity/resume/residual-history
  refresh, focused CMake targets passed, focused CTest passed `2/2`,
  `pixi run demos -- --verify-fbf-scene-docs` checked all 9 FBF paper scenes,
  `pixi run docs-build`, `pixi run lint`, and `pixi run build` passed. Final
  whitespace checks were rerun with `git diff --check` plus the
  binary-skipping untracked-file whitespace loop.

- After the dense residual-polish slice on 2026-07-09,
  `pixi run cmake --build build/default/cpp/Release --target test_ExactCoulombFbfConstraintSolver fbf_paper_card_house_probe --parallel 8`
  passed. `ctest --test-dir build/default/cpp/Release -R
  test_ExactCoulombFbfConstraintSolver --output-on-failure` passed. The short
  `fbf_paper_card_house_probe 56,60 1 3000 120 32 200 nan 0 0 0.9 10 1.5 3001`
  diagnostic kept 56 clean with zero dense polishes and showed the 60-contact
  row still falling back with one dense polish and failed residual `3.768e-5`.
  The full
  `fbf_paper_card_house_probe 60 1 30000 120 32 200 nan 0 0 0.9 10 1.5 30001`
  diagnostic still fell back, with one dense polish, best/final failed
  residual `3.021e-6` at iteration `29355`, pre-polish history last residual
  `3.023e-6`, and tail ratio `1.0000019`.

- Final gates for the best-iterate diagnostic slice on 2026-07-09: after the
  report/tracker/README/parity/resume refresh, `pixi run lint`,
  `pixi run docs-build`, `pixi run demos -- --verify-fbf-scene-docs`, and
  `pixi run build` passed. Final whitespace checks were rerun with
  `git diff --check` plus the binary-skipping untracked-file whitespace loop.

- After the best-iterate diagnostic slice on 2026-07-09,
  `pixi run cmake --build build/default/cpp/Release --target UNIT_math_ExactCoulombFbfSolver test_ExactCoulombFbfConstraintSolver fbf_paper_card_house_probe --parallel 8`
  passed. `ctest --test-dir build/default/cpp/Release -R
  '(UNIT_math_ExactCoulombFbfSolver|test_ExactCoulombFbfConstraintSolver)'
  --output-on-failure` passed `2/2`. The short
  `fbf_paper_card_house_probe 56,60 1 5000 120 32 200 nan 0 0 0.9 10 1.5 5001`
  diagnostic kept 56 clean and showed the 60-contact failed best residual was
  still `2.635e-5` at iteration `5000`. The full
  `fbf_paper_card_house_probe 60 1 30000 120 32 200 nan 0 0 0.9 10 1.5 30001`
  diagnostic still fell back, with best failed residual `3.021e-6` at
  iteration `29355`, final failed residual `3.023e-6`, and tail ratio
  `1.0000019`.

- After the 48/2 and 38/2 masonry-arch tracker/report refresh on 2026-07-09,
  `pixi run docs-build`, `pixi run lint`, `pixi run build`, focused CMake
  targets, `git diff --check`, and the binary-skipping untracked-file
  whitespace loop passed.

- After the 48/2 and 26/2 masonry-arch tracker/report refresh on 2026-07-09,
  `pixi run docs-build`, `pixi run lint`, `pixi run build`, `git diff --check`,
  and the binary-skipping untracked-file whitespace loop passed. A first
  untracked whitespace-loop attempt used zsh's read-only `status` variable and
  was rerun with `diff_status`.

- The earlier residual-history plot refresh on 2026-07-09 was superseded by
  the final fixture-step regeneration recorded above and in
  `residual-history-report.md`. Use the final report rows, not the intermediate
  pre-step-update row counts, when evaluating current Figure 9 evidence.

- After the 48/2 and 38/2 masonry-arch promotion on 2026-07-09, focused CMake
  rebuilt `test_ExactCoulombFbfPaperFixtures`,
  `BM_INTEGRATION_exact_coulomb_fbf_paper`, `fbf_paper_trace`,
  `fbf_paper_gamma_sweep`, and `dart-demos`. The focused arch XML tests,
  exact/boxed trace smokes, residual-history trace smokes, gamma-sweep smokes,
  focused arch benchmark rows, and `pixi run demos -- --verify-fbf-scene-docs`
  passed.

- After the 47/2 and 25/2 masonry-arch promotion and tracker/report refresh on
  2026-07-09, `pixi run docs-build`, `pixi run lint`, `pixi run build`,
  `git diff --check`, and the binary-skipping untracked-file whitespace loop
  passed. `pixi run demos -- --verify-fbf-scene-docs` also passed and checked
  all 9 FBF paper scenes.

- After the Pixi capture-task and GUI report updates on 2026-07-08,
  `pixi run lint`, `pixi run docs-build`, and `pixi run build` passed.
  `pixi run demos -- --list-scenes | rg 'fbf_paper_'` found all nine
  `fbf_paper_*` scenes, `pixi run demos -- --cycle-scenes --frames 1` cycled
  43 demos x2 with only pre-existing non-FBF warnings, and
  `pixi run capture fbf_paper_backspin /tmp/fbf_paper_backspin.png 640 480 1`
  followed by `pixi run image-verdict /tmp/fbf_paper_backspin.png` passed the
  default non-blank gate. A simultaneous parallel `demos`/`capture` attempt
  raced while archiving `dart-demos` and passed when the capture command was
  rerun alone. Final `git diff --check` plus the binary-skipping untracked-file
  whitespace loop also passed after the report/tracker updates.

- A projected-gradient refinement of the dense adapter's local/global Delassus
  seed was tested and reverted on 2026-07-08. It kept the 56-contact reduced
  card-house row clean and improved the 5000-outer 60-contact failed residual
  from the prior `2.635e-5` to `2.499e-5`, but the full 30000-outer
  `fbf_paper_card_house_probe 60 1 30000 120 32 200 nan 0 0 0.9 10 1.5 0`
  row still failed after `108381.8 ms` at `3.024e-6`, essentially unchanged
  from the previous `3.023e-6` and slower. `test_ExactCoulombConstraintAdapter`
  passed after reverting the experiment.

- After the GUI capture evidence slice on 2026-07-08, all nine
  `fbf_paper_*` scenes were captured with
  `dart-demos --headless --shot ... --steps 1 --width 640 --height 480`, the
  default `image_verdict.py` gate passed for every PNG, and
  `assets/fbf_gui_capture_sheet.png` was generated.

- After the residual-history card-house cap-probe slice on 2026-07-08,
  `fbf_paper_card_house_probe` and `test_ExactCoulombFbfConstraintSolver`
  rebuilt, the focused FBF CTest regex passed `7/7`, the history-enabled
  `fbf_paper_card_house_probe 56,60 1 30000 120 32 200 nan 0 0 0.9 10 1.5 30001`
  diagnostic produced the 56/60 rows recorded above, `pixi run docs-build`,
  `pixi run lint`, and `pixi run build` passed, the filtered demo scene list
  command found all nine FBF paper scenes, the one-frame demo cycle command
  cycled 43 demos x2 with only pre-existing non-FBF warnings, and the tracked
  plus untracked whitespace checks passed.

- After the residual-selected local/global seed and projected-gradient
  retry-continuation slice on 2026-07-08, focused CMake targets rebuilt:
  `UNIT_math_CoulombCone`, `UNIT_math_ExactCoulombContactProblem`,
  `UNIT_math_ExactCoulombFbfSolver`, `test_ExactCoulombConstraintAdapter`,
  `test_ExactCoulombFbfConstraintSolver`, `test_ConstraintSolver`,
  `test_ExactCoulombFbfPaperFixtures`, `fbf_paper_trace`,
  `fbf_paper_card_house_probe`, and
  `BM_INTEGRATION_exact_coulomb_fbf_paper`. Focused CTest passed `7/7`.
  The focused reduced-card-house benchmark row passed after rebuilding the
  benchmark binary.
- Final local gates after the report updates: `pixi run docs-build`,
  `pixi run lint`, `pixi run build`, `git diff --check`, and an untracked-file
  whitespace loop passed. `pixi run demos -- --list-scenes` listed all nine
  `fbf_paper_*` scenes and `pixi run demos -- --cycle-scenes --frames 1`
  cycled 43 demos x2 with only pre-existing non-FBF warnings.

- After the residual-history trace slice on 2026-07-08, focused math and
  constraint-solver targets rebuilt, focused CTest passed `7/7`, and
  `pixi run docs-build`, `pixi run lint`, and `pixi run build` passed.
  The latest regenerated residual-history CSV/SVG evidence is recorded in the
  2026-07-09 plot-refresh bullet above.

- After the contact-rich `dynamic_bodies` trace-scope slice on 2026-07-08,
  `fbf_paper_trace` rebuilt/up-to-date, focused CTest passed `7/7`,
  tracked-body compatibility and contact-rich mobile-body trace smokes passed
  before and after lint/build, and `pixi run docs-build`, `pixi run lint`, and
  `pixi run build` passed.

- After the reduced 26-card trace-export slice on 2026-07-08,
  `fbf_paper_trace` rebuilt/up-to-date, focused CTest passed `7/7`, the new
  exact-FBF and boxed-LCP card-house trace smokes passed before and after
  lint/build, and `pixi run docs-build`, `pixi run lint`, and
  `pixi run build` passed.

- After the 56-contact promotion and lint/formatting on 2026-07-08, focused
  builds for the math, adapter, opt-in solver, `test_ConstraintSolver`, paper
  fixture, benchmark, trace, gamma-sweep, cap-probe, and `dart-demos` targets
  passed.
- Focused CTest passed `7/7` for the math units, constraint adapter, opt-in
  solver, `test_ConstraintSolver`, and paper fixtures after the final
  post-lint focused rebuild.
- `CardHouseTenLevelSceneBuilds` passed locally and built 155 card skeletons
  plus ground with the exact solver configured, `max_contacts=512`, and
  `max_contacts_per_pair=8`.
- `CardHouseFourLevelOneStepReducedContactProbe` uses `max_contacts=56`,
  `max_contacts_per_pair=1`, `step_size_scale=10`, and
  `outer_relaxation=1.5`. The final sweep/benchmark evidence reports
  `contacts=56`, `residual=2.2183427389532158e-16`, `exact_solves=3`,
  `exact_failures=0`, `max_fbf_iterations=1908`,
  `total_fbf_iterations=1908`, and zero boxed-LCP fallback.
- `MasonryArch25OneStepReducedContactProbe` XML properties from the latest
  local arch run: `max_contacts=48`, `max_contacts_per_pair=2`,
  `contacts=48`, `step_size_scale=10`, `elapsed_ms=6693.7547709999999`,
  `residual=9.9650461738801354e-07`, `exact_solves=1`,
  `exact_failures=0`, `fallbacks=0`, `fbf_iterations=2645`,
  `max_fbf_iterations=2645`, and `total_fbf_iterations=2645`.
- `MasonryArch101OneStepReducedContactProbe` XML properties from the latest
  local arch run: `max_contacts=38`, `max_contacts_per_pair=2`,
  `contacts=38`, `step_size_scale=10`, `elapsed_ms=2458.3270210000001`,
  `residual=9.9001648754006507e-07`, `exact_solves=1`,
  `exact_failures=0`, `fallbacks=0`, `fbf_iterations=3912`,
  `max_fbf_iterations=3912`, and `total_fbf_iterations=3912`.
- Paper-scaled `fbf_paper_card_house_probe 56`, `58`, `59`, and `60` rows
  reported 56 contacts clean in `4971.0 ms`, 58 contacts clean in
  `68879.1 ms`, 59 contacts clean in `76402.9 ms` with one PG retry and max
  FBF iterations `7888`, and 60 contacts still failing.
- The latest history-enabled
  `fbf_paper_card_house_probe 56,60 1 3000 120 32 200 nan 0 0 0.9 10 1.5 3001`
  run reported 56 contacts clean in `4971.0 ms` with zero dense polishes and
  1911 retained history rows, and 60 contacts failing in `12867.2 ms` with one
  dense polish and failed residual `6.800e-5`. The latest full 60-contact
  `... 30000 ... 30001` run failed in `94606.3 ms` with 30001 failed-history
  rows, one dense polish, last failed iteration `30000`, first failed residual
  `1.146e-5`, best/final failed residual `5.334e-6`, pre-polish history last
  residual `5.338e-6`, and tail ratio `1.0000019`.
- 60-contact diagnostics still fail: default `step_size_scale=10`,
  `outer_relaxation=1.5` failed with paper-scaled pre-polish history residual
  `5.338e-6` and dense-polished failed final residual `5.334e-6`; the earlier
  `step_size_scale=12` and `outer_relaxation=2.0` rows were measured before
  the paper residual-scale fix and should be rechecked only if those option
  paths become active again.
- All benchmark rows passed at 2026-07-08 21:00:43 -0700. Exact-FBF rows
  reported zero exact-FBF failures and zero boxed-LCP fallbacks; after the
  final fixture-step update, focused changed exact-FBF rows were rerun. The
  reduced-contact exact card-house row reported 56 contacts, 5028 ms, 3 exact
  solves, `step_size_scale=10`, `outer_relaxation=1.5`, max/total FBF
  iterations `1908`/`1908`, and max residual `2.218e-16`; the 10-level
  construction boxed-LCP row reported 155 cards, 512 contacts, 1010 ms, and
  zero exact-FBF solves; the reduced-contact exact 25-stone masonry-arch row
  reported 48 contacts, 6514 ms, one exact solve, zero failures/fallbacks,
  max/total FBF iterations `2645`/`2645`, and max residual
  `9.9650461738801354e-07`; the reduced-contact exact 101-stone row reported
  38 contacts, 2365 ms, one exact solve, zero failures/fallbacks, max/total
  FBF iterations `3912`/`3912`, and max residual
  `9.9001648754006507e-07`.
- After the scrollable Scene-tab/widget update,
  `pixi run demos -- --list-scenes | rg 'fbf_paper_'` listed all nine
  `fbf_paper_*` Research scenes, including `fbf_paper_card_house_10` and
  `fbf_paper_masonry_arch_101`.
- `pixi run demos -- --cycle-scenes --frames 1` passed for 43 demos x2 after
  the scrollable Scene-tab/widget update, with existing non-FBF warnings from
  unsupported Bullet pyramid shapes, soft-body fragment clamping, and MJCF
  inertial inference. Do not run two `pixi run demos` invocations in parallel;
  their target builds can race on `libdart-demos-scenes.a`.
- After the catalog-verifiable Scene-tab metadata update on 2026-07-09,
  `pixi run demos -- --verify-fbf-scene-docs` checked all 9 FBF paper scenes,
  `pixi run demos -- --list-scenes | rg 'fbf_paper_'` listed all nine scene
  IDs, and `pixi run demos -- --cycle-scenes --frames 1` again cycled 43 demos
  x2 with only the known non-FBF warnings.
- `pixi run capture fbf_paper_backspin /tmp/fbf_paper_backspin_scroll.png 640 480 1`
  followed by `pixi run image-verdict /tmp/fbf_paper_backspin_scroll.png`
  passed after the scrollable Scene-tab/widget update.
- Backspin exact-FBF, boxed-LCP, and explicit `initial_gamma=0.05` trace
  smokes emitted CSV rows successfully after rebuilding stale trace/sweep
  helpers for the new options layout.
- Reduced-contact 26-card card-house exact-FBF and boxed-LCP trace smokes
  emitted top-card CSV rows for `card_house_l3_f0_left_body`; the exact-FBF
  row reported 56 contacts, 3 exact solves, zero fallbacks, residual
  `2.218e-16`, and `success`.
- Contact-rich `dynamic_bodies` trace smokes emitted one-step mobile-body rows:
  26 step-1 rows for reduced 26-card exact-FBF and boxed-LCP, 23 step-1 rows
  for reduced 25-stone exact-FBF, and 99 step-1 rows for reduced 101-stone
  exact-FBF. The exact-FBF rows reported zero fallbacks; after the final
  fixture-step updates, the reduced 26-card residual is `2.218e-16`, while the
  latest recorded arch residuals at 48 and 38 contacts are
  `9.9650461738801354e-07` and `9.9001648754006507e-07`.
- Reduced-contact masonry-arch exact-FBF and boxed-LCP trace smokes emitted
  crown-body CSV rows for `masonry_arch_stone_12_body`; the exact-FBF row
  reported 48 contacts, one exact solve, zero fallbacks, residual
  `9.9650461738801354e-07`, and `success`.
- Reduced-contact 101-stone masonry-arch exact-FBF and boxed-LCP trace smokes
  emitted crown-body CSV rows for `masonry_arch_stone_50_body`; the exact-FBF
  row reported 38 contacts, one exact solve, zero fallbacks, residual
  `9.9001648754006507e-07`, and `success`.
- Gamma sweep smokes now record contact caps and solver-budget columns in
  every CSV row. The explicit-cap backspin sweep emitted clean rows for
  safe/`0.5`/`0.05` and a recorded fallback row for `0.01`; reduced-scaffold
  sweeps emitted clean exact 32- and 56-contact card-house rows, a clean
  safe-gamma row and fixed-`0.001` boxed-LCP fallback row for the 25-stone
  arch at 48/2, and a clean safe-gamma row plus fixed-`0.001` boxed-LCP
  fallback row for the 101-stone arch at 38/2. The newer
  `fbf_paper_arch_probe` supersedes older ad hoc arch-cap notes: 25-stone 49/2
  still yields only 48 actual contacts, 25-stone 80/4 is clean but takes about
  118 s, 25-stone 100/4 timed out under a 120 s guard, and 101-stone one-step
  rows are clean through 100/2.
- The new `fbf_paper_arch_probe` target rebuilt cleanly with
  `pixi run cmake --build build/default/cpp/Release --target
  fbf_paper_arch_probe --parallel 8`; its default, 25-stone boundary, and
  101-stone high-cap rows produced the results in the Masonry-Arch Cap Probe
  section above.
- Paper-code availability was rechecked on 2026-07-09. The GitHub page for
  `https://github.com/matthcsong/fbf-sca-2026` returned HTTP 404, and
  unauthenticated `GIT_TERMINAL_PROMPT=0 git ls-remote
  https://github.com/matthcsong/fbf-sca-2026.git` could not read it.
- After adding worst-contact residual diagnostics, benchmark-only card-house
  geometry knobs, and the residual-selected local dense polish, focused CMake
  targets `UNIT_math_CoulombCone`, `test_ExactCoulombFbfConstraintSolver`, and
  `fbf_paper_card_house_probe` rebuilt cleanly; focused CTests for
  `UNIT_math_CoulombCone` and `test_ExactCoulombFbfConstraintSolver` passed;
  `pixi run lint`, `git diff --check`, and the untracked-file whitespace loop
  passed.
- After rejecting the ordered multi-contact local dense-polish experiment,
  focused CMake targets `UNIT_math_CoulombCone`,
  `test_ExactCoulombFbfConstraintSolver`, and `fbf_paper_card_house_probe`
  rebuilt cleanly; focused CTests for `UNIT_math_CoulombCone` and
  `test_ExactCoulombFbfConstraintSolver` passed; the post-revert
  `fbf_paper_card_house_probe 56,60 1 3000 120 32 200 nan 0 0 0.9 10 1.5 3001`
  row kept 56 contacts clean in `4895.3 ms` and left 60 contacts falling back
  with failed residual `6.166e-5`.
- `pixi run demos -- --verify-fbf-scene-docs`, `pixi run docs-build`,
  `pixi run lint`, and `pixi run build` passed after the reduced 25-stone
  arch projectile scaffold, GUI-widget update, screenshot refresh, PR report,
  tracker, resume, and parity-matrix edits. `git diff --check` and the
  corrected binary-skipping untracked-file whitespace loop also passed after
  the final verification-ledger edits.
- A local unquoted zsh benchmark filter attempt failed earlier with
  `no matches found`; use the quoted
  `--benchmark_filter='BM_PaperFixtureStepTime/(backspin_boxed_lcp|backspin_exact_fbf)$'`
  form shown above.

## Citation

This prototype is based on:

Hongcheng Song, Ye Fan, Uri M. Ascher, and Dinesh K. Pai, "A Splitting
Architecture for Exact Reduced Coulomb Friction", SCA 2026.

The branch credits the paper in implementation comments and in the DART 6 RTD
`Research Papers And References` page. Keep those citations current if the
authors publish canonical BibTeX or scene files.

## 2026-07-11 Update: Multi-Step Settle Frontier Resolved

The multi-step full-contact convergence blocker is resolved by split-impulse
operation of the exact path (removing DART's ERP bias from the velocity-phase
right-hand side) plus a base DART bugfix: the split-impulse position pass was
wiping velocity-phase constraint impulses before integration, making resting
bodies free-fall while contacts persisted (single-sphere repro under the
plain boxed solver). The fix preserves body/joint constraint impulses around
the position pass in `ConstraintSolver::solvePositionConstrainedGroups`
(dual-PR rule applies). New regression coverage:
`ConstraintSolver.SplitImpulsePreservesVelocityPhaseContactResponse` and
`ExactCoulombFbfPaperFixtures.CardHouseFourLevelFullManifoldSplitImpulseSettleProbe`.
Measured: full-manifold settle steps all-exact at `1e-6`, zero fallbacks,
no-creep physics, ~`1.4 s`/step; the 10 s Fig. 6 sequence reached its 6.7 s
projectile launch with the house standing. Figure 9 residual histories and
plots regenerated at full-manifold scale under author Rigid-IPC geometry.

Final evidence added after the frontier resolution: the complete 10 s Fig. 6
sequence ran end-to-end (`assets/card_house_full_sequence_10s.csv`) — settle
0-6.7 s with zero fallbacks, all-exact at `1e-6`, ~3 mm compaction (no
visible creep); four projectiles at 6.7 s cause localized failure (cards
knocked over, max travel `1.9 m`) while the house core stands, with
`1617` exact solves, `1613` warm-started, and `24` fallbacks confined to the
post-impact chaos. The arch standing outcome was characterized through five
controlled experiments (see the tracker ledger): the solver holds post-slump
equilibria to `1e-7`/step drift, and reproducing the paper's standing
catenary requires equilibrium initialization or persistent face-face contact
manifolds — DART scene/collision infrastructure, not exact-FBF capability.
Figure 9 residual histories and SVGs were regenerated at full-manifold scale
and Figure 10 sweeps refreshed at the promoted options.
