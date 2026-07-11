# Resume: FBF exact Coulomb friction

## Current Checkpoint

Branch: `research/fbf-friction-release620`

Read [AGENT_CONTINUATION.md](AGENT_CONTINUATION.md) before continuing. It owns
the current progress ledger, immediate todo list, paper-parity todo list, and
the anti-premature-completion rule for this task.

Current state as of 2026-07-09 (second session): the incremental inner
block-Gauss-Seidel solver and the scratch-backed
`ExactCoulombContactRowOperator` (WP-PG.14 pattern, default-on) landed and
cleared the 60/64-contact card-house fallback boundary; full natural contact
manifolds now solve one-step (card house 108 contacts in 2.5 s, 101-stone
arch 512 contacts in ~37 s, 25-stone arch 96 contacts in ~74 s with
`outer_relaxation=1.5` and a `120000` arch outer budget). Contact-manifold
warm starts across `World` steps landed
(`ExactCoulombFbfWorldSmokeManifoldWarmStartAcrossSteps`). The branch base
moved to `db255a08e8e`; its manifold changes regressed two rows, both
re-fixed (see `AGENT_CONTINUATION.md`, which carries the full evidence for
this slice). Author-faithful Rigid-IPC arch geometry (MIT, commit
`23b6ba6fbf8`) plus a working MuJoCo 3.10.0 backspin comparison prototype
were acquired and are staged for integration; the full-manifold promotion of
test/benchmark/trace/gamma/GUI surfaces is in flight. The paragraphs below
describe the state before this session and remain valid as history.

Prior state as of 2026-07-09: the exact-FBF path uses the paper's fixed
per-solve residual scales, the small fixtures use `step_size_scale=2`, the
two-card precursor keeps the conservative scale `1`, the reduced 26-card row
uses `step_size_scale=10` and `outer_relaxation=1.5`, and the reduced masonry
arches use `step_size_scale=10`. The full `test_ExactCoulombFbfPaperFixtures`
CTest passes. The final reduced-scaffold evidence is 56-card-house contacts
clean at residual `2.2183427389532158e-16` with max/total FBF iterations
`1908`/`1908`, the 25-stone arch clean at residual
`9.9650461738801354e-07` with `2645` FBF iterations, and the 101-stone arch
clean at residual `9.9001648754006507e-07` with `3912` FBF iterations.
Residual-history CSV/SVG artifacts were regenerated from the current trace
executable with 18 backspin rows, 1911 reduced-card-house rows, 2646
25-stone-arch rows, and 3913 101-stone-arch rows. This is still not paper
parity: the 26-card row is only a 56-contact one-step default, 60 contacts
still falls back, the 10-level card-house row is construction/default-solver
only, the arch rows are reduced-contact scaffolds, external comparisons are
not wired, and the dev-task folder remains active. The latest continuation
also tested and reverted two 60-contact solver-side shortcuts: residual-selected
depth-one Anderson acceleration and a projected boxed-LCP cold start. Anderson
either broke the 56-contact promoted row or left 60 contacts failing around
`6.39e-5`; the boxed-LCP seed sped up 56 contacts but worsened the 60-contact
failed residual to `1.544e-4`. The card-house cap probe now reports worst
residual contact indices and failed contact-pair labels; the current 60-contact
short diagnostic is dual-dominated at contact `13`,
`card_house_l0_support0_body|card_house_l0_f1_left_body`, with
complementarity dominated by contact `2`, `BodyNode|card_house_l0_f1_left_body`.
Quick manifold/geometry diagnostics did not clear the boundary:
`max_contacts_per_pair=2` worsened the 60-contact failed residual, smaller
`0.001` initial penetration barely changed it, zero penetration dropped to only
27 contacts, and frame spacing `0.57` improved the short residual but still
fell back. A residual-selected local dense-polish pass for the worst dual
contact is now kept because it improves failed residuals without changing the
success criterion, but it still does not clear 60 contacts (`6.166e-5` at 3000
outer iterations and `2.079e-5` at 10000). A follow-up multi-contact local
dense-polish candidate-order experiment was built, measured, and reverted
because it preserved 56 contacts but left 60 contacts failing at the same
`6.166e-5` short residual. The latest solver-route slice added
`applyExactCoulombConstraintDelassus(...)` and
`useMatrixFreeDelassusOperator`, so focused FBF `W*x` products can now run
through retained DART constraint rows instead of multiplying by the dense
snapshot. Adapter, solver, and one-contact `World` smokes cover that opt-in
route, but the dense snapshot is still assembled and retained for the DART 6
staging bridge, default cold-start seed, and prototype dense residual polish. The
post-rebuild short card-house probe kept 56 contacts clean and left 60 contacts
falling back at `6.166e-5`, so this is route-staging progress rather than a
contact-rich convergence fix. The current card-house cap probe now exposes the
route through `use_matrix_free_delassus_operator` and CSV columns. Dense versus
matrix-free probe rows show 16 contacts solving in `24.4 ms` versus
`324.2 ms`, 32 contacts solving in `170.4 ms` versus `3203.0 ms`, and a
40-contact capped row failing at the same `3.966e-5` residual while taking
`1057.4 ms` versus `18937.8 ms`. Treat this as measured route visibility, not
as the final paper-style scatter/inverse-mass/gather implementation. The
latest solver-route slice added `useMatrixFreeDelassusSeed`, which uses
operator-extracted local diagonal Delassus blocks for the opt-in matrix-free
seed path. Final-code 16-contact probe rows with trailing
`use_matrix_free_delassus_seed=0` and `1` both solved at residual
`3.212e-16`; the new `matrix_free_delassus_seed_used` CSV column toggled from
`0` to `1`, but elapsed time stayed about `354 ms`, so this removes one dense
seed dependency without addressing product cost. Larger `32,40` and `56,60`
matrix-free product rows hit a 120 s bound before a first CSV row. A local
batched scatter/gather product experiment was also reverted because generic
`ConstraintBase` does not expose safe touched-body bias-impulse propagation;
do not repeat that solver-only shortcut. The GUI task scope explicitly
includes OSG renderer, `dart-demos` host, and reusable ImGui widget
improvements whenever a paper GUI example needs them to be self-contained,
inspectable, visually verifiable, or stable under headless snapshot capture.
The latest continuation added a reduced 26-card settle/projectile phase
scaffold: `CardHouseFourLevelSettleProjectileScaffoldRuns` launches four
projectiles after one bounded settle step, `fbf_paper_trace
card_house_26_settle_projectile_reduced_contact ... phase_summary` emits
initial/settle/projectile rows, exact-FBF and boxed-LCP benchmark rows are
registered, and `fbf_paper_card_house_26` has a Scene-panel projectile-launch
control with self-contained text explaining the limitation. The exact-FBF
phase-summary trace ended with 26 cards, 4 projectiles, finite state, 56
contacts, 6 exact solves, zero fallbacks, residual
`4.8919831295901383e-17`, and `success`; the focused exact-FBF benchmark row
ran in `12057 ms` with max/total FBF iterations `3631`/`5539`. This is still
only two-step reduced scaffold evidence, not the paper's 6.7 s no-creep settle,
10 s impact sequence, full-contact timing, or dynamic snapshot parity.
The latest GUI continuation added action-aware headless capture support:
`dart-demos --headless-action <key>` and `pixi run capture-action` can invoke a
scene key action before a screenshot, and `fbf_paper_card_house_26` binds `p`
to launch the four projectile spheres. The saved 1280x720 projectile capture
under `assets/gui_captures/fbf_paper_card_house_26_projectiles.png` shows the
four spheres beside the card house with the self-contained Scene-tab text
visible; the verdict JSON passes the default non-blank gate. This is a GUI
reproducibility improvement, not paper-matched Fig. 6 snapshot parity.
The latest contact-rich continuation added benchmark-only
`fbf_paper_arch_probe`, a CSV cap probe for the reduced 25/101-stone masonry
arches. It builds as a benchmark/report executable and does not add a core DART
dependency. Fresh rows show 25-stone 49/2 still generates only 48 actual
contacts, 25-stone 64/4 solves in `31118.9 ms`, 25-stone 80/4 solves in
`117628.4 ms`, and 25-stone 100/4 timed out under a 120 s guard before a row.
The same probe shows the older 101-stone 39/2 fallback-boundary note is stale:
the current approximate one-step 101-stone scaffold solves through 100/2, with
the 100/2 row taking `56137.1 ms`, residual `9.9444848815471616e-07`, one exact
solve, zero exact-FBF failures, and zero boxed-LCP fallbacks. This improves
arch cap evidence only; author/Rigid-IPC geometry, long-run physical outcomes,
snapshots, and paper timing parity remain missing.
The latest continuation added a reduced 25-stone masonry-arch projectile
scaffold: `MasonryArch25ProjectileScaffoldRuns`,
`masonry_arch_25_projectile_reduced_contact` trace rows, exact-FBF/boxed-LCP
benchmark rows, and a `fbf_paper_masonry_arch_25` Scene-panel launch control
plus `p` action. The focused test passes in about `10611 ms`; the exact-FBF
tracked trace ends with 48 contacts, one exact solve, zero fallbacks, residual
`9.9901843912475669e-07`, and `success`; the exact-FBF benchmark row reports
about `4921 ms`, max residual `999.018n`, and `2205` FBF iterations. The
GUI action capture at
`assets/fbf_paper_masonry_arch_25_projectile.png` passes the default image
verdict and shows the projectile plus self-contained Scene-tab text. The
shared diagnostics widget now reports `not run yet` before the first exact
solve. This is reduced GUI/test/trace/benchmark scaffold evidence only, not
paper Fig. 7 parity.

The first session established the source digest, paper-parity matrix, DART 6
baseline inventory, DART 7 candidate-port list, and added the first shared math
component: normal-first primal/dual Coulomb cone projection in
`dart/math/detail/CoulombCone.hpp`.

The second slice added the dimensionless exact-Coulomb residual helper in the
same header. It reports primal cone distance, dual cone distance, the
complementarity gap, and their max for one contact or a product of contacts.
Focused coverage for both slices lives in `tests/unit/math/test_CoulombCone.cpp`.

The third slice added `dart/math/detail/ExactCoulombContactProblem.hpp`, a
matrix-free reduced contact-space helper for `v(lambda) = W lambda + v_free`,
De Saxce-Feng augmented velocity assembly, exact-Coulomb residual assembly, and
deterministic power-iteration estimation of the largest Delassus eigenvalue.
Focused coverage lives in
`tests/unit/math/test_ExactCoulombContactProblem.cpp`.

The fourth slice added `dart/math/detail/ExactCoulombFbfSolver.hpp`, an
internal FBF outer-loop shell with safe spectral step initialization,
coupling-variation line search, safeguarded shrink, FBF correction, projection
back to the Coulomb cone, and residual stopping around an injected frozen
cone-QP solver. Focused coverage lives in
`tests/unit/math/test_ExactCoulombFbfSolver.cpp`.

The fifth slice extended `ExactCoulombFbfSolver.hpp` with
`solveExactCoulombFrozenConeProjectedGradient(...)`, a matrix-free
projected-gradient approximation for the frozen strongly convex cone QP. The
tests now cover single-contact analytical cases, projection onto the circular
cone, a small product-cone dense case, and running the FBF shell with this
inner solver.

The sixth slice added `solveExactCoulombFrozenConeBlockGaussSeidel(...)`, a
reference version of the paper-style matrix-free contact-block Gauss-Seidel
inner solve. It extracts per-contact 3x3 diagonal Delassus blocks through
matrix-free products, uses current global products for off-diagonal coupling,
and solves each 3D block by local projected-gradient steps. Focused coverage
compares it against analytical one-contact/product-cone cases, a coupled dense
problem solved by projected gradient, and the FBF shell with this inner solver.
This is still a math-helper path, not integrated DART contact assembly.

The seventh slice added
`dart/constraint/detail/ExactCoulombConstraintAdapter.hpp`, a staging bridge
from existing DART 6 contact-style `ConstraintBase` rows into
`ExactCoulombContactProblem`. It accepts only three-row isotropic friction
contacts, converts the boxed-LCP right-hand side to `v_free = -b`, assembles a
dense Delassus snapshot by DART 6 impulse tests, and can apply solved reaction
triples back through the original constraints. Synthetic coverage lives in
`tests/unit/constraint/test_ExactCoulombConstraintAdapter.cpp`; a real
`ContactConstraint` smoke lives in `tests/integration/test_ConstraintSolver.cpp`.
This is still not a default solver path.

The eighth slice added `dart/constraint/ExactCoulombFbfConstraintSolver.hpp`
and `.cpp`, an opt-in solver subclass that attempts exact-Coulomb FBF on
supported isotropic contact groups and falls back to the existing boxed-LCP
group solve for unsupported or failed groups unless fallback is disabled. It
uses the dense adapter, the FBF outer loop, and the block Gauss-Seidel frozen
cone solver. Focused coverage lives in
`tests/unit/constraint/test_ExactCoulombFbfConstraintSolver.cpp`. This is the
first routed constrained-group path, but it is still a dense prototype and has
not paper-fixture parity.

The ninth slice added a capped single-contact `World` smoke in
`tests/integration/test_ConstraintSolver.cpp`. It installs
`ExactCoulombFbfConstraintSolver` through `World::setConstraintSolver`, steps a
box/ground contact scene, checks that the exact path solved with no boxed-LCP
fallback, asserts residual convergence, and performs a bounded one-step state
comparison against the default boxed-LCP path. This proves first public-world
routing only; at that point it was not warm-started, matrix-free at the DART
integration layer, or a paper fixture.

The tenth slice added a conservative warm-start cache to
`ExactCoulombFbfConstraintSolver`. A successful exact solve caches the reaction
for the next solve only when the constraint pointer sequence is identical; the
cached reaction is projected into the current Coulomb cones before use.
Diagnostics report warm-start use and outer FBF iteration count. Focused tests
prove reuse for the same synthetic contact group and rejection for a different
constraint pointer. This is not yet true contact-manifold identity.

The eleventh slice added the first paper-parameter fixture in
`tests/integration/test_ExactCoulombFbfPaperFixtures.cpp`. It recreates the
incline cube threshold with slope `atan(0.5)`, `dt = 1/60`, `T = 2 s`, and
`mu = 0.5`/`0.4`; routes the exact worlds through
`ExactCoulombFbfConstraintSolver`; verifies exact residuals, no boxed-LCP
fallbacks, stick/slide separation, and analytical sliding displacement; and
runs the same worlds through the existing boxed-LCP solver as the DART 6
baseline. This is a first fixture regression, not the full paper sweep,
snapshot, or timing report.

The twelfth slice added the backspin sphere fixture to the same integration
test. It uses the paper parameters `r = 0.25 m`, `v0 = 4 m/s`,
`omega0 = -200 rad/s`, `mu = 0.5`, and `dt = 1/60`, then runs long enough to
check the analytical no-slip terminal state
`v_inf = -11.429 m/s`, `omega_inf = -45.71 rad/s`. It verifies exact residuals,
no boxed-LCP fallbacks, contact persistence, and finite boxed-LCP baseline
state. This is still not a full residual trace or timing benchmark.

The thirteenth slice added the turntable capture/ejection grid from Figure 4
to `tests/integration/test_ExactCoulombFbfPaperFixtures.cpp`. It drives an
immobile support with explicit angular velocity before each step, classifies
`mu = 0.2, omega = 2` and `mu = 0.2, omega = 5` as ejected, classifies
`mu = 0.5, omega = 2` as captured, and classifies
`mu = 0.5, omega = 5` as ejected. It verifies exact residuals, no boxed-LCP
fallbacks, and finite boxed-LCP baseline states. The high-speed ejection cell
needs the larger paper-fixture solver budget now recorded in the test.

The fourteenth slice added a Painleve-style proxy fixture for Figure 5 to the
same integration test. Because the paper gives only qualitative scene
description for the Painleve box and the authors' implementation is still not
anonymously readable here, this is explicitly a proxy rather than author-scene
parity. It uses a slightly leaned tall box sliding on a rough horizontal plane,
checks that `mu = 0.5` slides/rests upright, checks that `mu = 0.55` reaches
the tumble threshold after a shorter travel distance, verifies exact residuals
and no boxed-LCP fallbacks, and keeps the DART 6 boxed-LCP run as a finite
baseline.

The fifteenth slice promoted the small fixture checks from final-state residual
assertions to sampled per-step residual traces. The incline, backspin,
turntable, and Painleve-proxy exact runs now record each step that performs an
exact-Coulomb solve and assert no sampled exact status failures, no non-finite
sampled residuals, no boxed-LCP fallback, and maximum sampled residual at or
below `1e-6`. This is an in-tree regression guard toward Figure 9, not the
paper's full per-substep/per-outer-iteration convergence plot.

The sixteenth slice added
`tests/benchmark/integration/bm_exact_coulomb_fbf_paper.cpp`, an in-tree
Google Benchmark timing harness for the small paper fixtures. It registers
boxed-LCP and exact-FBF rows for incline `mu = 0.5`/`0.4`, backspin, all four
turntable cells, and the Painleve proxy `mu = 0.5`/`0.55`. The FBF rows report
exact solves, warm starts, boxed fallbacks, contact count, and max sampled
residual so timing output is tied to solver correctness counters. This is only
DART-side timing infrastructure; paper-hardware timing parity and external
Kamino/MuJoCo comparisons remain missing.

The seventeenth slice added
`tests/benchmark/integration/fbf_paper_trace.cpp`, a headless CSV trace
exporter for the small paper fixtures. It can run each scene with `exact_fbf`
or `boxed_lcp` and emits body state, contact count, exact solve counters,
warm-start counters, fallback counters, residual, and exact-path status at a
chosen sample stride.
This produces comparison artifacts for later trajectory/residual parity work;
it is not an installed API, visual snapshot, full residual-convergence plot,
paper-hardware timing report, or external-baseline comparison.

The eighteenth slice added a compiled but disabled contact-rich precursor:
`ExactCoulombFbfPaperFixtures.DISABLED_CardHouseAFramePrecursorStands`.
It constructs a two-card A-frame with `mu = 0.8` as the first Fig. 6
card-house scaffold. It remains disabled because the current dense exact-FBF
prototype does not yet solve this coupled card-card/card-ground contact set:
with a larger local budget and relaxed `1e-5` precursor tolerance, the live
run still fell back to boxed LCP on `59` of `60` steps and ended around
`3.7e-5` residual. This is a concrete reproduction target, not paper parity.

The nineteenth slice added a projected-gradient robustness retry inside
`ExactCoulombFbfConstraintSolver`. The default exact path still tries the
paper-style contact-block Gauss-Seidel frozen cone solve first; if that exact
solve fails, the solver can retry the same exact-Coulomb problem with the
matrix-free projected-gradient frozen cone solver before falling back to boxed
LCP. Focused unit coverage forces block-GS failure with `innerMaxSweeps = 0`
and verifies the projected-gradient retry solves the supported contact group.
This retry did not solve the disabled two-card A-frame: with the retry enabled
and warm start disabled for the precursor, the run still had `59` boxed-LCP
fallbacks over `60` steps and ended around `3.7e-5` residual.

The twentieth slice exposed exact-FBF residual components and
projected-gradient retry diagnostics through
`ExactCoulombFbfConstraintSolver`. The disabled two-card A-frame now reports
why it fails: the latest run is dual-feasibility dominated
(`dual = 3.72e-5`, `primal = 0`, `complementarity ~= 3.9e-9`), with
projected-gradient retry attempted on all `59` failed exact solves.

The twenty-first slice made the two-card A-frame precursor an enabled
regression. The failure was slow outer convergence rather than inner-solver or
line-search failure: the old contact-rich budget hit `MaxIterations` at
`1200` outer iterations. Raising the precursor cap to `5000` and restoring the
paper `1e-6` tolerance makes
`ExactCoulombFbfPaperFixtures.CardHouseAFramePrecursorStands` pass with zero
boxed-LCP fallback. The same slice added a disabled 26-card four-level Fig. 6
scene scaffold, then converted it to the enabled construction regression
`CardHouseFourLevelSceneBuilds` so the normal suite does not hang when disabled
tests are run. A manual one-step exact-FBF probe still did not finish within
30 seconds on the current dense Delassus prototype, so the next card-house
blocker is matrix-free/scratch contact solving. It also added source comments
crediting Song, Fan, Ascher, and Pai's SCA 2026 paper and a DART 6 RTD
`Research Papers And References` page.

The twenty-second slice added task-local anti-premature-completion guardrails
and a live progress/todo ledger in `AGENT_CONTINUATION.md`, added the
PR-facing coverage and verification report in `PR_REPORT.md`, and integrated
the first six FBF paper inspection scenes into the consolidated `dart-demos`
app under the `Research` category:
`fbf_paper_incline`, `fbf_paper_backspin`, `fbf_paper_turntable`,
`fbf_paper_painleve`, `fbf_paper_card_aframe`, and
`fbf_paper_card_house_26`. The same slice added an optional
`initial_gamma` argument to `fbf_paper_trace` and the standalone
`fbf_paper_gamma_sweep` CSV helper so small-fixture traces can be used for
fixed-step sweep scripting with residual, fallback, physical-outcome, and
final-state columns. These GUI scenes are visual inspection counterparts for
the implemented small fixtures, two-card precursor, and immobile 26-card
construction scaffold; the trace/sweep helpers are not the full paper
Figure 10 house/arch sweep. At that historical point, local verification
showed the scene catalog listed those six, `--cycle-scenes --frames 1` passed,
focused
CTest passed `7/7`, the benchmark list/all-row smoke ran with zero exact-FBF
boxed-LCP fallbacks, backspin trace smokes emit CSV rows for exact FBF, boxed
LCP, and explicit-gamma exact FBF, and `docs-build`, `lint`, `build`, and
`git diff --check` pass.

The twenty-third slice strengthened the dev-task tracker itself so future
agents do not infer completion from a green subset. `AGENT_CONTINUATION.md`
now owns a required tracker-update protocol, current status snapshot, active
work board, immediate todo list, paper-parity todo list, completion criteria,
reporting rules, and verification ledger. `README.md` now points to that
tracker as the canonical progress and todo surface. This slice is task
maintenance, not paper-parity progress. The post-tracker verification pass
reran docs-build, lint, focused FBF targets, gamma sweep, demo catalog/factory
smoke, benchmark list, representative quoted backspin benchmark smoke,
exact/boxed/explicit-gamma traces, focused CTest `7/7`, `pixi run build`, and
`git diff --check`; because the new files are still untracked, it also ran a
no-index whitespace loop over untracked files.

The twenty-fourth slice added
`ExactCoulombFbfPaperFixtures.CardHouseFourLevelOneStepReducedContactProbe`.
It runs one exact-FBF dynamic step on the 26-card four-level card-house world
with a reduced `max_contacts = 19`, `max_contacts_per_pair = 1` cap and records
GTest XML properties for cap, actual contacts, elapsed time, residual, exact
solve count, and FBF iterations. The local run passed with `contacts=19`,
`elapsed_ms=309.228`, `residual=9.613e-7`, `exact_solves=5`, zero exact-FBF
failures, zero boxed-LCP fallback, and `fbf_iterations=174`. Exploratory 20-
and 24-contact runs each produced one boxed-LCP fallback; increasing the
20-contact inner-solve and
projected-gradient retry budgets did not remove that fallback. This is a
bounded contact-rich rung, not full Fig. 6 paper parity; the next target is
removing the 20-contact fallback and then raising the one-step probe toward the
full contact budget.
The GUI scene for the 26-card rung now uses the same 19-contact cap on both
initial construction and the `Exact FBF` / `Boxed LCP` toggle path. Future GUI
paper-parity work may improve the OSG renderer, `dart-demos` host, or reusable
ImGui widgets when a fixture needs better overlays, camera/snapshot capture,
inspection controls, or self-contained explanation.

The twenty-fifth slice added failure-specific exact-FBF diagnostics and raised
the reduced-contact 26-card one-step rung. The solver now preserves the most
recent failed exact-FBF status, residual components, and iteration count even
when later constrained groups solve successfully, and the GUI shows the last
failed status/residual when failures occur. The 20-contact failure was
diagnosed as `MaxIterations` at the old `5000` outer cap with dual residual
`1.529e-6`; raising the card-house one-step outer cap to `10000` clears the
20-, 21-, 24-, and 32-contact rungs at the paper `1e-6` tolerance. The durable
test, benchmark, and `fbf_paper_card_house_26` GUI scene now use
`max_contacts = 32`, `max_contacts_per_pair = 1`. The local 32-contact XML
probe passed with `elapsed_ms=1353.103`, residual `9.613e-7`, 5 exact solves,
zero exact-FBF failures, zero boxed-LCP fallback, and `fbf_iterations=174`.
A 64-contact probe still failed after 27.8 s with one exact-FBF
`MaxIterations` failure, dual residual `9.84e-4`, zero exact solves, and one
boxed-LCP fallback. The next target is the 64-contact no-fallback rung before
full-contact settle/projectile parity.
This paragraph is historical and is superseded by the twenty-sixth slice below.

The twenty-sixth slice added a deterministic dense-contact cold-start seed and
a benchmark-only card-house contact-cap probe. The adapter now seeds zero
contact-row guesses with `max(0, -v_free_n / W_nn)` on the normal row, while
preserving explicit nonzero row guesses and solver-level warm starts. The
`fbf_paper_card_house_probe` CSV helper reports contact cap, exact status,
residual components, failure residual components, fallback counts, and option
knobs without adding any core dependency. With the seed enabled, the durable
test, benchmark, and `fbf_paper_card_house_26` GUI scene now use
`max_contacts = 36`, `max_contacts_per_pair = 1`. The local 36-contact XML
probe passed with `elapsed_ms=2105.879`, residual `9.790e-7`, 5 exact solves,
zero exact-FBF failures, zero boxed-LCP fallback, and `fbf_iterations=165`.
The final all-row benchmark reported `1843 ms` for the exact reduced-contact
card-house row, 36 contacts, 5 exact
solves, zero exact failures, zero fallbacks, and max residual `9.790e-7`.
The contact-cap probe established the next boundary: 36 contacts is clean, 38
contacts fails with one `MaxIterations` exact failure, dual residual
`4.61e-4`, and one fallback, and 64 contacts fails with dual residual
`9.88e-4` and one fallback. Testing `includeConstraintRegularization`,
`innerDiagonalRegularization=1e-6`, and `couplingVariationTolerance=1.5` did
not remove the 40-contact fallback. The next target is the 38-contact
no-fallback rung, then 64 contacts and full-contact settle/projectile parity.
This paragraph is historical and is superseded by the twenty-seventh slice
below.

The twenty-seventh slice promoted the reduced-contact 26-card one-step rung to
38 contacts. The twenty-eighth slice added an opt-in FBF `stepSizeScale`
diagnostic knob, recovers the configured base step after accepted line-search
shrinks, and moved the card-house test/benchmark/probe/GUI rung to
`step_size_scale=10`. The card-house FBF budget remains `30000` outer
iterations. The opt-in solver exposes last/max/total FBF iteration diagnostics
plus step-size, safe-step, coupling-variation, shrink-count, and gamma-scale
diagnostics; the CSV probe, benchmark counters, XML test properties, and GUI
Scene panel report the current values. The local 38-contact XML probe passed
with `elapsed_ms=1898.197`, residual `8.482e-7`, 5 exact solves, zero exact
failures, zero fallbacks, last-group FBF iterations `37`, max group FBF
iterations `2667`, and total FBF iterations `2815`. The focused card-house
benchmark row reported `1853 ms`, 38 contacts, 5 exact solves, zero exact
failures, zero fallbacks, max FBF iterations `2667`, total FBF iterations
`2815`, max residual `8.482e-7`, and `step_size_scale=10`. The updated cap
probe `fbf_paper_card_house_probe 38,64` confirms 38 is clean and 64 remains
the first failing rung: 64 contacts produced one exact-FBF `MaxIterations`
failure, one boxed-LCP fallback, failed dual residual `1.52e-5`, step size
`9.346e-3`, safe step size `9.346e-4`, and coupling-variation ratio
`1.29e-2`. The next target is the 64-contact no-fallback rung, then full
contact budget, settle/projectile parity, and richer visual/snapshot evidence.
This paragraph is historical and is superseded by the twenty-ninth slice
below.

The twenty-ninth slice added an opt-in FBF `outerRelaxation` knob and promoted
only the reduced card-house test/benchmark/probe/GUI rung to
`outer_relaxation=1.5`; the core exact-FBF solver default remains `1.0`.
The local 38-contact XML probe now passes with `elapsed_ms=1283.317`,
residual `3.383e-7`, 5 exact solves, zero exact failures, zero fallbacks,
last-group FBF iterations `23`, max group FBF iterations `1774`, and total
FBF iterations `1866`. The all-row benchmark smoke reported the reduced
card-house exact row at `1328 ms`, 38 contacts, 5 exact solves, zero exact
failures, zero fallbacks, max FBF iterations `1774`, total FBF iterations
`1866`, max residual `3.383e-7`, `step_size_scale=10`, and
`outer_relaxation=1.5`. The default cap probe
`fbf_paper_card_house_probe 38,64` now reports 38 clean at residual
`3.383e-7`; 64 contacts still fails after 30000 FBF outer iterations with one
exact-FBF `MaxIterations` failure, one boxed-LCP fallback, failed dual
residual `6.955e-6`, step size `9.346e-3`, safe step size `9.346e-4`, and
coupling-variation ratio `1.513e-2`. The GUI Scene diagnostics now expose
outer relaxation alongside gamma scale. The next target remains the
64-contact no-fallback rung, then full contact budget, settle/projectile
parity, and richer visual/snapshot evidence.
This paragraph is historical and is superseded by the thirtieth slice below.

The thirtieth slice promoted the practical reduced-contact 26-card one-step
rung from 38 contacts to 56 contacts while keeping `step_size_scale=10`,
`outer_relaxation=1.5`, and the 30000 outer-iteration card-house budget. The
local 56-contact XML probe passed with `elapsed_ms=6953.902`, residual
`3.383e-7`, 3 exact solves, zero exact failures, zero fallbacks, last-group
FBF iterations `23`, max group FBF iterations `1744`, and total FBF
iterations `1790`. The focused promoted card-house benchmark row reported
`7448 ms`, 56 contacts, 3 exact solves, zero exact failures, zero fallbacks,
max FBF iterations `1744`, total FBF iterations `1790`, max residual
`3.383e-7`, `step_size_scale=10`, and `outer_relaxation=1.5`; the boxed-LCP
row for the same cap reported `1.06 ms`. The cap probe
`fbf_paper_card_house_probe 56,58,59,60,64` showed 56 contacts clean, 58
contacts clean but slow at about `67641 ms` with max FBF iterations `28425`,
59 contacts as the first default fallback boundary with failed dual residual
`2.701e-6`, 60 contacts still failing with failed dual residual `6.477e-6`,
and 64 contacts still failing with failed dual residual `6.955e-6` and one
boxed-LCP fallback. A follow-up `step_size_scale=12` diagnostic clears
59 contacts but takes `77173 ms` and reaches max FBF iterations `29567`, while
60 contacts with the same scale still fails after `105511 ms` with failed dual
residual `4.242e-6`. The next target is removing the 60-contact and
64-contact fallbacks without relying on near-cap runtimes, or replacing the
dense path with a matrix-free/scratch-reusing route before attempting the full
contact budget, settle/projectile parity, and richer visual/snapshot evidence.

The thirty-first slice added the first reduced-contact 25-stone masonry-arch
scaffold for Fig. 7. The scaffold is approximate DART-side geometry with static
endpoint supports and 23 dynamic interior stones; it is not yet the author
Rigid-IPC geometry, pinned/projectile setup, or 100-contact paper scene.
`MasonryArch25SceneBuilds` and `MasonryArch25OneStepReducedContactProbe` are
enabled in `test_ExactCoulombFbfPaperFixtures`. The one-step arch probe passed
locally with `max_contacts=12`, `max_contacts_per_pair=1`,
`elapsed_ms=142.908`, residual `9.963e-7`, one exact solve, zero exact-FBF
failures, zero boxed-LCP fallback, and `2969` FBF iterations. The all-row
benchmark smoke reported `masonry_arch_25_reduced_contact_exact_fbf` at
`130 ms`, 12 contacts, one exact solve, zero failures/fallbacks, max/total FBF
iterations `2969`, and max residual `9.963e-7`; the boxed-LCP row reported
`0.128 ms`. The trace exporter now emits one-step exact-FBF and boxed-LCP
crown-stone rows for `masonry_arch_25_reduced_contact`, with the exact row
reporting 12 contacts, one exact solve, zero fallbacks, residual `9.963e-7`,
and `success`. The GUI scene `fbf_paper_masonry_arch_25` is registered under
`Research` and its Scene panel explains overview, expected result, and coverage
limits. Higher-contact diagnostics are still blocked: 24 contacts failed with
residual `1.503e-5`, and 64 contacts failed with residual `8.939e-4`. The next
arch target is replacing/corroborating the geometry, raising the contact cap,
and adding projectile outcome, long-run traces, snapshots, and timing.

The thirty-second slice added the first reduced-contact 101-stone
masonry-arch scaffold for Fig. 8. The scaffold reuses the approximate DART-side
arch construction with static endpoint supports and 99 dynamic interior stones;
it is not author geometry or the paper's long-run balance setup. The enabled
`MasonryArch101SceneBuilds` and
`MasonryArch101OneStepReducedContactProbe` tests pass at `max_contacts=12` and
`max_contacts_per_pair=1`; the XML probe reported `elapsed_ms=24.488`,
residual `9.998e-7`, one exact solve, zero exact-FBF failures, zero boxed-LCP
fallback, and `811` FBF iterations. The all-row benchmark smoke reported
`masonry_arch_101_reduced_contact_exact_fbf` at `24.0 ms`, 12 contacts, one
exact solve, zero failures/fallbacks, max/total FBF iterations `811`, and max
residual `9.998e-7`; the boxed-LCP row reported `0.251 ms`. The trace exporter
emits one-step exact-FBF and boxed-LCP crown-stone rows for
`masonry_arch_101_reduced_contact`, with the exact row reporting 12 contacts,
one exact solve, zero fallbacks, residual `9.998e-7`, and `success`. The GUI
scene `fbf_paper_masonry_arch_101` is registered under `Research` and its
Scene panel explains overview, expected result, and coverage limits. The next
101-arch target is replacing/corroborating the geometry, raising the contact
cap, and adding long-run balance, long-run traces, snapshots, and timing.

The thirty-third slice added the first 10-level card-house scaffold for the
paper's GPU-table scene. `CardHouseTenLevelSceneBuilds` builds the triangular
155-card construction plus ground with the exact solver configured,
`max_contacts=512`, and `max_contacts_per_pair=8`; it does not run exact-FBF
dynamics. The benchmark row `card_house_10_construction_boxed_lcp` runs one
boxed-LCP/default-solver construction step and reported 155 cards, 512 contacts,
zero exact-FBF solves, and `1010 ms` in the latest all-row benchmark smoke. The
GUI scene `fbf_paper_card_house_10` is registered under `Research`, renders a
static construction scaffold, defaults to boxed-LCP diagnostics, and its Scene
panel explains that exact-FBF dynamics are intentionally unavailable. This slice
also added the reusable demo-control path needed for construction-only paper
scenes to omit the exact-FBF toggle. The next 10-level target is exact-FBF
dynamic outcome, residual trace, timing, snapshots, and external baselines.

The thirty-fourth slice added reduced-contact 26-card card-house trace export
for `fbf_paper_trace`. The new scenario
`card_house_26_reduced_contact` shares the same 56-contact cap,
`max_contacts_per_pair=1`, `step_size_scale=10`, `outer_relaxation=1.5`, and
30000 outer-iteration budget as the current test/benchmark/GUI rung. The
exact-FBF and boxed-LCP smoke commands emit one-step rows for
`card_house_l3_f0_left_body`; the exact-FBF row reported 56 contacts,
3 exact solves, zero boxed-LCP fallbacks, residual `3.383e-7`, and `success`.
This is only a top-card trace sanity check. The 26-card paper target still
needs full-contact bounded execution, a 6.7 s no-creep settle, four-projectile
phase, long-run trace rows, residual convergence plots, visual
snapshots, timing, and external baselines.

The thirty-fifth slice extended `fbf_paper_trace` with an optional sixth
argument for trace scope. The default `tracked` behavior is unchanged; passing
`dynamic_bodies` or `full_scene` emits one CSV row per mobile body at each
sample. Local one-step smokes emitted 26 step-1 mobile-card rows for the
reduced 26-card card-house exact-FBF and boxed-LCP modes, 23 step-1 mobile
stone rows for the reduced 25-stone arch exact-FBF mode, and 99 step-1 mobile
stone rows for the reduced 101-stone arch exact-FBF mode. The exact rows
reported zero fallbacks and residuals `3.383e-7`, `9.963e-7`, and `9.998e-7`
respectively. This improves state-export coverage, but it is still one-step
reduced-contact evidence; full-contact execution, long-run trajectories,
per-substep/per-outer residual traces, visual snapshots, and timing parity
remain open.

The thirty-sixth slice added bounded opt-in residual-history diagnostics for
the exact-FBF math helper and opt-in DART constraint solver. The default solver
path does not retain per-outer samples; tests, examples, or benchmarks must
set an explicit `maxResidualHistorySamples` cap. A follow-up in the same slice
added bounded retained per-solve records, so contact-rich steps can expose all
exact groups rather than only the last solve. `fbf_paper_trace` now accepts
`residual_history` as the sixth argument for `exact_fbf` runs and emits
per-outer CSV rows with `solve_index`, `solve_contacts`, residual components,
step-size diagnostics, shrink counts, contacts, exact solve counters, fallback
count, and status. Local smokes showed `backspin exact_fbf 1 1 nan
residual_history` emitting 49 rows from outer iteration 0 residual `4.35925`
to iteration 48 residual `9.9707e-7`. The reduced 26-card contact-rich smoke
`card_house_26_reduced_contact exact_fbf 1 1 nan residual_history` emitted
1793 rows across 3 exact groups: a 54-contact group ending at residual
`9.984e-7` and two one-contact groups ending near residual `3.383e-7`. The
reduced 25/101-stone arch smokes emitted 2970 and 812 rows for their
12-contact exact groups, ending at residuals `9.963e-7` and `9.998e-7`. This
is useful Figure 9 evidence, but it is still one-step reduced-scaffold data;
full-contact histories, long-run histories, and paper-style plots remain open.
The same slice added `residual-history-report.md` as the durable summary of
commands, CSV schema, row counts, and current limitations.

The thirty-seventh slice changed the dense DART contact adapter cold start from
a normal-only diagonal seed to a residual-selected projected local/global
Delassus seed with a diagonal fallback. Explicit nonzero DART row guesses and
solver-level warm starts are still preserved. The same slice changed the
projected-gradient robustness retry so that, when the block-GS exact-FBF
attempt fails with a finite reaction, the retry starts from that failed
block-GS reaction instead of the original cold row guess. Focused adapter/solver
tests pass. The card-house cap probe now shows 56 contacts clean in
`4825.7 ms`, 58 contacts clean but slow in `64426.6 ms`, and 59 contacts clean
with zero fallback in `74310.6 ms` using one projected-gradient retry and max
FBF iterations `5450`. This is not a default promotion because it is still a
multi-minute diagnostic. The 60-contact row remains the current one-step
fallback boundary: `step_size_scale=10` fails with failed dual residual
`3.023e-6`; earlier 60-contact option probes with `step_size_scale=12` and
`outer_relaxation=2.0` did not clear the boundary. The next target remains
removing the 60/64-contact fallback without near-cap runtimes, likely by deeper
solver/scratch/matrix-free work rather than another one-knob option probe.

The thirty-eighth slice added optional retained residual-history summaries to
the benchmark-only `fbf_paper_card_house_probe` helper. Passing a positive
`max_residual_history_samples` argument keeps the normal cap-probe CSV row
shape but adds record counts, total retained rows, failed-row counts, failed
first/last residuals, failed last iteration, and a tail ratio. The focused
history-enabled diagnostic
`fbf_paper_card_house_probe 56,60 1 30000 120 32 200 nan 0 0 0.9 10 1.5 30001`
reported 56 contacts clean in `4685.6 ms` with 1746 retained history rows and
60 contacts still failing in `96114.7 ms` with 30001 failed-history rows
through iteration 30000. The failed 60-contact exact group moved from residual
`6.491e-6` to `3.023e-6` with tail ratio `1.0000019`, confirming slow
near-cap dual convergence rather than a reporting-only missing-history issue.
The same slice updated the task tracker, parity matrix, PR report,
residual-history report, and README to keep GUI/OSG improvements in scope
when paper examples need viewer, widget, overlay, or snapshot support to be
self-contained.

The thirty-ninth slice was a measured dead end and was reverted. A
residual-selected over-relaxation guard for `outerRelaxation > 1` was
implemented and built, but the history-enabled 56/60 probe showed it made the
promoted 56-contact row slower (`7166.8 ms`, max FBF iterations `1841`) and
left 60 contacts failing after `113139.0 ms` with a worse failed residual
`3.673e-6`. After reverting the guard, shorter 5000-outer diagnostics showed
the current default 60-contact option set remains the best tested short-cap
direction: default failed residual `2.635e-5`; `step_size_scale=12`,
`step_size_scale=8`, `outer_relaxation=1.0`, and `outer_relaxation=1.8` were
worse; increasing projected-gradient retry iterations to `1000` and doubling
block-GS sweeps/local iterations to `240`/`64` did not change the failed
residual. Do not repeat these one-knob probes as the next primary path; the
next 60/64-contact attempt should be deeper solver, scratch, matrix-free, or
fixture/contact-reduction work.

The fortieth slice added durable GUI capture evidence for the current
`dart-demos` FBF scenes. It used the existing `dart-demos --headless --shot`
path with `--steps 1 --width 640 --height 480` on all nine `fbf_paper_*`
scene IDs, wrote PNGs and default `image_verdict.py` JSON files under
`docs/dev_tasks/fbf_exact_coulomb_friction/assets/gui_captures/`, and
generated `assets/fbf_gui_capture_sheet.png`. All nine verdicts passed the
default non-blank gate, and the sheet shows the rendered world plus the
self-contained `Scene` tab text for each example. This is GUI smoke evidence
only; paper-matched long-run snapshots, full-contact house/arch captures, and
external baseline images remain open.

The forty-first slice made the GUI capture path easier to reproduce through
Pixi and refreshed the task/reporting docs. `pixi run capture` now accepts a
fifth `steps` argument while preserving the demo host's existing default of
`150` headless settle steps; FBF smoke captures use
`pixi run capture <scene_id> <out.png> 640 480 1`. `README.md`,
`AGENT_CONTINUATION.md`, `PR_REPORT.md`, `gui-capture-report.md`, and
`examples/demos/README.md` now spell out the interactive `pixi run demos --`
commands, the bounded capture command, and the rule that OSG renderer,
`dart-demos` host, or reusable ImGui-widget improvements are part of this task
whenever a paper GUI example cannot otherwise be self-contained or visually
verifiable. This is task maintenance and GUI reproducibility work, not new
paper parity.

The forty-second slice was another measured dead end and was reverted. A
projected-gradient refinement of the dense adapter's residual-selected
local/global Delassus seed was implemented and built, but it did not move the
real 60-contact boundary. The short 56/60 diagnostic with a 5000 outer-iteration
cap kept 56 contacts clean and reduced the 60-contact failed residual from the
prior `2.635e-5` to `2.499e-5`, but the full
`fbf_paper_card_house_probe 60 1 30000 120 32 200 nan 0 0 0.9 10 1.5 0`
row still failed after `108381.8 ms` with failed residual `3.024e-6`, which is
effectively unchanged from the previous `3.023e-6` and slower. The code/test
patch was reverted, and `test_ExactCoulombConstraintAdapter` passed after the
revert. Do not repeat adapter projected-gradient seed refinement as the next
60/64-contact strategy.

The forty-third slice extended `fbf_paper_gamma_sweep` beyond the small
fixtures to reduced one-step contact-rich scaffolds:
`card_house_26_reduced_contact`, `masonry_arch_25_reduced_contact`, and
`masonry_arch_101_reduced_contact`. The helper now has one-step reduced
card-house and arch builders with the same caps and exact-FBF options as the
current reduced benchmark/test rungs, and it reports the same clean-exact,
fallback, residual, final-state, and physical-outcome columns as the earlier
small-fixture sweep. Local smokes were:

```bash
./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_gamma_sweep backspin nan,0.05 10
./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_gamma_sweep card_house_26_reduced_contact nan,0.001 1
./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_gamma_sweep masonry_arch_25_reduced_contact nan,0.001 1
./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_gamma_sweep masonry_arch_101_reduced_contact nan,0.001 1
```

The short backspin smoke kept both safe and fixed-`0.05` rows clean exact. The
26-card reduced scaffold produced two clean 56-contact rows with zero fallbacks
and final residual `2.697e-19`. The 25-stone reduced scaffold produced a clean
safe-gamma row at residual `9.946e-7`, while fixed `0.001` intentionally
records a boxed-LCP fallback row with final residual `3.839e-5`. The 101-stone
reduced scaffold produced two clean 12-contact rows with final residuals
`9.952e-7` and `9.993e-7`. This is still reduced one-step scaffold evidence,
not full Figure 10 parity; full-contact long-run house/arch sweeps with
paper-level physical outcomes remain open. After the helper/report/tracker
edits, `fbf_paper_gamma_sweep` rebuilt cleanly, all four smoke commands above
passed, and `pixi run lint`, `pixi run docs-build`, `pixi run build`,
`git diff --check`, and the binary-skipping untracked-file whitespace loop
passed.

The forty-fourth slice improved the GUI widget support required for
self-contained FBF examples. The shared `dart-demos` Scene tab now wraps the
scene-specific panel in an independent scroll child, so long paper
explanations and exact-FBF diagnostics remain reachable in small windows and
headless captures. The FBF panel also visually groups `Overview`,
`Expected Result`, `Coverage`, `Solver`, and `Diagnostics` content. This is a
GUI-host/widget improvement, not renderer-level OSG work; no OSG scene-graph
hook has been required yet. Verification:

```bash
pixi run cmake --build build/default/cpp/Release --target dart-demos --parallel 8
pixi run demos -- --list-scenes | rg 'fbf_paper_'
pixi run demos -- --cycle-scenes --frames 1
pixi run capture fbf_paper_backspin /tmp/fbf_paper_backspin_scroll.png 640 480 1
pixi run image-verdict /tmp/fbf_paper_backspin_scroll.png
```

The list command found all nine `fbf_paper_*` scenes, cycle-scenes passed for
43 demos x2 with the known non-FBF Bullet/soft-body/MJCF warnings, and the
backspin one-step capture passed the default non-blank verdict. Avoid running
two `pixi run demos` invocations in parallel: a parallel list/cycle attempt
raced on `libdart-demos-scenes.a`, and the resulting generated Ninja
dependency/log metadata had to be moved aside and regenerated before the
target could build again.
After the widget/report/tracker edits, `pixi run lint`,
`pixi run docs-build`, `pixi run build`, `git diff --check`, and the
binary-skipping untracked-file whitespace loop passed.

The forty-fifth slice rechecked the paper implementation repository named by
the project/video. On 2026-07-09,
`curl -I -L https://github.com/matthcsong/fbf-sca-2026` still returned HTTP
404, and unauthenticated `GIT_TERMINAL_PROMPT=0 git ls-remote
https://github.com/matthcsong/fbf-sca-2026.git` could not read it. Keep
paper-code comparison as an optional test/example/benchmark-only dependency
blocked on public availability or explicit credentials; do not treat it as
forgotten or as a core-library dependency.

The forty-sixth slice made the Figure 10 sweep helper cap-aware. The
`fbf_paper_gamma_sweep` command now accepts optional trailing
`max_contacts` and `max_contacts_per_pair` arguments and emits those
configured caps in every CSV row. Existing commands keep their scenario
defaults, but reduced and boundary rows can now be reproduced without reading
the C++ source. Verification for this slice:

```bash
pixi run cmake --build build/default/cpp/Release --target fbf_paper_gamma_sweep --parallel 8
./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_gamma_sweep backspin nan,0.5,0.05,0.01 240 1 1
./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_gamma_sweep card_house_26_reduced_contact nan 1 32 1
./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_gamma_sweep card_house_26_reduced_contact nan 1 56 1
./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_gamma_sweep masonry_arch_25_reduced_contact nan,0.001 1 12 1
./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_gamma_sweep masonry_arch_101_reduced_contact nan,0.001 1 12 1
```

The full-duration backspin rows record `max_contacts=1` and keep safe,
`0.5`, and `0.05` clean exact while fixed `0.01` records 50 exact solves and
190 boxed-LCP fallbacks. The card-house override row at 32 contacts and the
promoted reduced row at 56 contacts are both clean exact with zero fallbacks
and final residual `2.697e-19`. The 25-stone arch still records the useful
safe-gamma pass and fixed-`0.001` boxed-LCP fallback contrast, while the
101-stone arch rows remain clean at the 12-contact reduced cap. This is still
CSV/reporting infrastructure for reduced scaffolds, not full Figure 10 paper
parity.

The forty-seventh slice made the FBF GUI explanation requirement
catalog-verifiable and fixed a custom Scene-tab wiring issue. `DemoScene` now
has `scenePanelDocumentation` metadata, every `fbf_paper_*` scene fills
overview, expected-result, and coverage text, and `dart-demos` exposes
`--verify-fbf-scene-docs` to check that metadata without opening a viewer or
building worlds. The custom turntable scene previously passed its first text
string into `renderSolverControls`' `exactFbfAvailable` argument because it
does not use the shared FBF scene factory; that call now passes `true` followed
by the intended overview, expected-result, and coverage strings. Verification:

```bash
pixi run cmake --build build/default/cpp/Release --target dart-demos --parallel 8
pixi run demos -- --verify-fbf-scene-docs
pixi run demos -- --list-scenes | rg 'fbf_paper_'
pixi run demos -- --cycle-scenes --frames 1
```

The verifier checked all 9 FBF paper scenes, the list command found all nine
scene IDs, and the cycle smoke ran 43 demos x2 with only the known non-FBF
Bullet/soft-body/MJCF warnings. Continue to avoid running two `pixi run demos`
commands in parallel because they can race while rebuilding/archiving
`libdart-demos-scenes.a`.

The forty-eighth slice promoted the reduced masonry-arch default rung from
12 contacts / one contact per pair to 24 contacts / two contacts per pair
across tests, benchmarks, traces, gamma sweep defaults, and GUI scene text.
The clean 24/2 XML probes reported:

```text
MasonryArch25OneStepReducedContactProbe:
  max_contacts=24, max_contacts_per_pair=2, contacts=24,
  elapsed_ms=2100.600817, residual=9.9958878506145373e-07,
  exact_solves=1, exact_failures=0, fallbacks=0,
  max_fbf_iterations=4499, total_fbf_iterations=4499

MasonryArch101OneStepReducedContactProbe:
  max_contacts=24, max_contacts_per_pair=2, contacts=24,
  elapsed_ms=1604.4318599999999, residual=9.9989746767431062e-07,
  exact_solves=1, exact_failures=0, fallbacks=0,
  max_fbf_iterations=4854, total_fbf_iterations=4854
```

The corresponding trace smokes reported 24 contacts, zero fallbacks, and
`success`; residual-history scope ended at iteration `4499` for the 25-stone
arch (4500 rows) and iteration `4854` for the 101-stone arch (4855 rows).
Focused arch benchmark rows reported `2134 ms` for the 25-stone exact row and
`1711 ms` for the 101-stone exact row, both with one exact solve, zero
failures/fallbacks, and the iteration counts above. Gamma-sweep smokes now
default the arch rows to 24/2; safe gamma is clean for both arch scaffolds,
while fixed `0.001` falls back with residuals `3.798e-3` for the 25-stone arch
and `2.688e-3` for the 101-stone arch. A 64-contact / four-contact-per-pair
diagnostic still falls back for both arch scaffolds (`8.019e-4` final residual
for 25 stones and `1.090e-3` for 101 stones). This is still reduced one-step
scaffold evidence, not Fig. 7/8 paper parity.

This paragraph is historical and is superseded by the fifty-first slice below.
The forty-ninth slice promoted the reduced masonry-arch rungs again after
explicit cap probes. For the 25-stone arch, 47 contacts / two contacts per pair
is clean, while 48/2 falls back at final residual `1.503e-6`. For the 101-stone
arch, 25 contacts / two contacts per pair is clean, while 26/2 falls back at
final residual `1.386e-4`. The promoted XML probes reported:

```text
MasonryArch25OneStepReducedContactProbe:
  max_contacts=47, max_contacts_per_pair=2, contacts=47,
  elapsed_ms=12998.086638999999, residual=9.9964368370063166e-07,
  exact_solves=1, exact_failures=0, fallbacks=0,
  max_fbf_iterations=4562, total_fbf_iterations=4562

MasonryArch101OneStepReducedContactProbe:
  max_contacts=25, max_contacts_per_pair=2, contacts=25,
  elapsed_ms=1950.9752619999999, residual=9.9930057504281537e-07,
  exact_solves=1, exact_failures=0, fallbacks=0,
  max_fbf_iterations=5003, total_fbf_iterations=5003
```

`fbf_paper_trace` now retains up to 6000 residual-history samples so the
101-stone promoted row includes the final successful residual at iteration
`5003`. The latest residual-history smokes emitted 4563 data rows for the
25-stone 47-contact group and 5004 data rows for the 101-stone 25-contact
group. Focused benchmark rows reported `13499 ms` for the 25-stone exact row
and `1940 ms` for the 101-stone exact row, both with one exact solve, zero
failures/fallbacks, and the iteration counts above. Gamma-sweep smokes now
default the arch rows to 47/2 and 25/2; safe gamma is clean for both arch
scaffolds, while fixed `0.001` falls back with residuals `8.990e-3` for the
25-stone arch and `2.688e-3` for the 101-stone arch. This is still reduced
one-step scaffold evidence, not Fig. 7/8 paper parity.

This paragraph is historical and is superseded by the fifty-first slice below.
The fiftieth slice refreshed the residual-history report and plot artifacts
from the current trace executable. It regenerated the four residual-history
CSVs, the individual SVG plots, and a new combined reduced-scaffold residual
panel at `assets/fbf_residual_history_panel.svg` using a backward-compatible
multi-panel mode in `fbf_paper_residual_svg.py`. The refreshed CSV evidence is:
backspin 65 data rows ending at iteration `64` with residual
`8.3507091286350033e-07`; reduced 26-card 1746 data rows across three exact
groups ending at `2.6973775615580738e-19`; reduced 25-stone arch 4563 data
rows ending at `9.9964368370063166e-07`; and reduced 101-stone arch 5004 data
rows ending at `9.9930057504281537e-07`. This is current Figure 9-style
reduced-scaffold evidence, not full-contact or long-run paper parity.

The fifty-first slice promoted the reduced masonry-arch defaults with a bounded
budget change and better sweep reporting. `fbf_paper_gamma_sweep` now accepts
and emits solver-budget columns after the contact-cap arguments:
`max_outer_iterations`, `inner_sweeps`, `inner_local_iterations`,
`projected_gradient_iterations`, `step_size_scale`, and `outer_relaxation`.
Using the new helper, the 25-stone arch 48/2 row and 101-stone arch 26/2 row
both solved with a 20000 outer-iteration arch budget. The code paths in the
paper-fixture test, benchmark, trace exporter, gamma sweep, and GUI scene now
default to 48/2 and 26/2. Local XML probes reported:

```text
MasonryArch25OneStepReducedContactProbe:
  max_contacts=48, max_contacts_per_pair=2, contacts=48,
  elapsed_ms=23474.030498, residual=9.9992693083518742e-07,
  exact_solves=1, exact_failures=0, fallbacks=0,
  max_fbf_iterations=1063, total_fbf_iterations=1063

MasonryArch101OneStepReducedContactProbe:
  max_contacts=26, max_contacts_per_pair=2, contacts=26,
  elapsed_ms=3563.7646920000002, residual=9.9958535070553984e-07,
  exact_solves=1, exact_failures=0, fallbacks=0,
  max_fbf_iterations=10117, total_fbf_iterations=10117
```

The promoted residual-history traces emitted 1064 data rows for the 25-stone
48-contact group and 10118 data rows for the 101-stone 26-contact group.
Focused benchmark rows reported `22436 ms` for the 25-stone exact row and
`3731 ms` for the 101-stone exact row, both with one exact solve, zero
failures/fallbacks, and the iteration counts above. Gamma-sweep smokes now
default the arch rows to 48/2 and 26/2; safe gamma is clean for both arch
scaffolds, while fixed `0.001` falls back with residuals `1.631e-3` for the
25-stone arch and `5.363e-4` for the 101-stone arch. This is still reduced
one-step scaffold evidence, not Fig. 7/8 paper parity.

The fifty-second slice promoted only the reduced 101-stone masonry-arch
default after probing the next contact-cap boundary. A 25-stone `49/2`
diagnostic still produced only 48 actual contacts with the current approximate
scaffold, so the 25-stone default remains 48/2 and the next step there is
geometry/contact generation, not just raising the cap. For the 101-stone arch,
27/2, 28/2, 32/2, 36/2, and 38/2 were clean under the 20000 outer-iteration
budget; 39/2 and 40/2 fell back. The code paths in the paper-fixture test,
benchmark, trace exporter, gamma sweep, and GUI scene now default the
101-stone arch to 38/2. Local XML and smoke evidence:

This historical boundary is superseded by the latest `fbf_paper_arch_probe`
slice above: under the current solver/options, the approximate 101-stone
one-step scaffold solves through 100/2 without fallback. Keep the historical
record for provenance, but do not use it as the current boundary.

```text
MasonryArch101OneStepReducedContactProbe:
  max_contacts=38, max_contacts_per_pair=2, contacts=38,
  elapsed_ms=8820.9953690000002, residual=9.986829558113057e-07,
  exact_solves=1, exact_failures=0, fallbacks=0,
  max_fbf_iterations=18079, total_fbf_iterations=18079
```

The 101-stone exact/boxed trace smokes emitted crown-body rows with 38 contacts,
one exact solve, zero fallbacks, and `success`. The residual-history trace
emitted 18080 data rows for the 38-contact exact group, ending at iteration
`18079` with residual `9.986829558113057e-07`, and the 101-stone residual SVG
plus combined residual panel were regenerated. The focused benchmark rows
reported `0.345 ms` for boxed LCP and `8549 ms` for exact FBF, both at 38
contacts, with one exact solve, zero failures/fallbacks, max/total FBF
iterations `18079`, and max residual `998.683n`. The gamma sweep defaults now
use 38/2; safe gamma is clean with residual `9.986829558113057e-07`, while
fixed `0.001` falls back with final residual `1.4536023855508393e-3`.
`pixi run demos -- --verify-fbf-scene-docs` checked all nine FBF paper scenes
after the GUI Scene-tab text was updated to describe the 38-contact rung.
This is still reduced one-step scaffold evidence, not Fig. 8 paper parity.

The fifty-third slice added best-iterate retention to
`ExactCoulombFbfSolver` and exposed last/failed best-residual diagnostics
through `ExactCoulombFbfConstraintSolver` and
`fbf_paper_card_house_probe`. The projected-gradient retry now seeds from the
best finite failed block-GS FBF reaction when that residual is no worse than
the final failed iterate. Focused unit coverage was added for retaining the
best iterate on max iterations and for forwarding failed-best diagnostics
through the opt-in constraint solver. Local validation rebuilt
`UNIT_math_ExactCoulombFbfSolver`, `test_ExactCoulombFbfConstraintSolver`, and
`fbf_paper_card_house_probe`; focused CTest passed `2/2`. The refreshed
card-house probes kept the 56-contact row clean and confirmed the 60-contact
boundary is not caused by losing a better iterate:

```text
fbf_paper_card_house_probe 56,60 ... 5001:
  56 contacts: clean, 4721.95886 ms, residual 2.697e-19
  60 contacts: fallback, 20242.270291 ms,
    failed best residual = failed final residual = 2.6347214015278393e-05
    at iteration 5000

fbf_paper_card_house_probe 60 ... 30001:
  fallback, 95826.391932 ms, failed rows 30001,
  first failed residual 6.49149148432107e-06,
  best failed residual 3.0213258880441068e-06 at iteration 29355,
  final failed residual 3.023314204767257e-06 at iteration 30000,
  tail ratio 1.000001906543134
```

This is diagnostic progress only. The promoted 26-card default remains
56 contacts, and 60 contacts is still the one-step fallback boundary.

The fifty-fourth slice added a bounded dense residual-polish path to
`ExactCoulombFbfConstraintSolver` for failed exact-FBF solves after block-GS
FBF and projected-gradient retry have not reached tolerance. The polish solves
a dense regularized dual-correction system against the current Delassus
snapshot, line-searches projected reaction candidates, and accepts only
residual-improving updates. This is explicitly a DART 6 dense-prototype
cleanup/recovery aid, not the paper's matrix-free production route. Focused
unit coverage now proves a synthetic one-contact failed solve can be recovered
by dense polish, and the card-house cap probe reports
`dense_residual_polishes`.

Local validation rebuilt `test_ExactCoulombFbfConstraintSolver` and
`fbf_paper_card_house_probe`; focused CTest for
`test_ExactCoulombFbfConstraintSolver` passed. The refreshed probe evidence is:

```text
fbf_paper_card_house_probe 56,60 ... 3001:
  56 contacts: clean, 4658.102842 ms, 3 exact solves,
    0 PG retries, 0 dense polishes, residual 2.697e-19
  60 contacts: fallback, 12366.680421 ms,
    1 PG retry, 1 dense polish,
    failed best/final residual 3.7677715215169138e-05
    pre-polish history last residual 3.8227040656782017e-05

fbf_paper_card_house_probe 60 ... 30001:
  fallback, 93841.375018 ms, failed rows 30001,
  one PG retry and one dense polish,
  first failed residual 6.49149148432107e-06,
  best/final failed residual 3.0213258880441068e-06 at iteration 29355,
  pre-polish history last residual 3.023314204767257e-06,
  tail ratio 1.000001906543134
```

This slice confirms dense polish is useful for cleanup and reporting, but it
does not clear the card-house boundary. The promoted 26-card default remains
56 contacts, and 60 contacts is still the one-step fallback boundary.

The fifty-fifth slice implemented the paper's fixed per-solve residual scales
in `ExactCoulombFbfSolver`. The solver now computes
`s_r = max(||lambda_0||, ||v_f|| / lambda_max(W), 1e-12)` and
`s_u = max(||v_tilde(lambda_0)||, ||v_f||, 1e-12)`, stores the effective
scales in the result, and uses them for residual history, best/final residual
tracking, projected-gradient retry comparison, and dense residual polish. This
is paper-correctness work, not a tolerance relaxation.

Focused validation rebuilt `UNIT_math_ExactCoulombFbfSolver`,
`test_ExactCoulombFbfConstraintSolver`, and `fbf_paper_card_house_probe`;
focused CTest passed `2/2`. The paper-scaled card-house boundary is:

```text
fbf_paper_card_house_probe 56,60 ... 3001:
  56 contacts: clean, 4971.024299 ms, max FBF iterations 1908,
    0 PG retries, 0 dense polishes, residual 2.218e-16
  60 contacts: fallback, 12867.184982 ms,
    1 PG retry, 1 dense polish,
    failed best/final residual 6.7995858406507377e-05
    pre-polish history last residual 6.8987209785795989e-05

fbf_paper_card_house_probe 58,59 ... 30000:
  58 contacts: clean, 68879.067410 ms, 1 PG retry, 0 dense polishes
  59 contacts: clean, 76402.944395 ms, 1 PG retry, 0 dense polishes,
    max FBF iterations 7888

fbf_paper_card_house_probe 60 ... 30001:
  fallback, 94606.334623 ms, failed rows 30001,
  one PG retry and one dense polish,
  first failed residual 1.1460517049589893e-05,
  best/final failed residual 5.3340525726528403e-06 at iteration 29355,
  pre-polish history last residual 5.3375628811483052e-06,
  tail ratio 1.0000019065431343
```

The promoted 26-card default remains 56 contacts, and 60 contacts is still the
one-step fallback boundary under the paper-scaled residual.

The fifty-sixth slice stabilized the paper-fixture surfaces after the
paper-scale residual change. The small DART-side paper fixtures now use
`step_size_scale=2`; the two-card precursor explicitly keeps
`step_size_scale=1`; the reduced 26-card row keeps `step_size_scale=10` and
`outer_relaxation=1.5`; and the reduced 25/101-stone arch rows now use
`step_size_scale=10`. The incline sweep outcome classifier in
`fbf_paper_gamma_sweep` now measures displacement along the downhill direction
instead of using the absolute initial `x` offset.

Final local evidence for this slice:

```text
ctest --test-dir build/default/cpp/Release -R test_ExactCoulombFbfPaperFixtures --output-on-failure
  passed

MasonryArch25OneStepReducedContactProbe:
  max_contacts=48, max_contacts_per_pair=2, contacts=48,
  step_size_scale=10, elapsed_ms=6693.7547709999999,
  residual=9.9650461738801354e-07,
  exact_solves=1, exact_failures=0, fallbacks=0,
  max_fbf_iterations=2645, total_fbf_iterations=2645

MasonryArch101OneStepReducedContactProbe:
  max_contacts=38, max_contacts_per_pair=2, contacts=38,
  step_size_scale=10, elapsed_ms=2458.3270210000001,
  residual=9.9001648754006507e-07,
  exact_solves=1, exact_failures=0, fallbacks=0,
  max_fbf_iterations=3912, total_fbf_iterations=3912

Default gamma-sweep rows:
  incline_mu_0_5: clean, stick_like, max residual 9.998e-7
  incline_mu_0_4: clean, slide_like, max residual 9.991e-7
  card_house_26_reduced_contact: clean, 56 contacts, residual 2.218e-16
  masonry_arch_25_reduced_contact: clean, 48 contacts, residual 9.965e-7
  masonry_arch_101_reduced_contact: clean, 38 contacts, residual 9.900e-7

Focused exact-FBF benchmark rows:
  incline_mu_0_5_exact_fbf: 21.7 ms, max FBF iterations 382
  incline_mu_0_4_exact_fbf: 3.62 ms, max FBF iterations 59
  card_house_26_reduced_contact_exact_fbf: 5028 ms, max FBF iterations 1908
  masonry_arch_25_reduced_contact_exact_fbf: 6514 ms, max FBF iterations 2645
  masonry_arch_101_reduced_contact_exact_fbf: 2365 ms, max FBF iterations 3912
```

The residual-history CSV/SVG artifacts were regenerated from the current trace
executable. The final row counts are backspin 18 data rows ending at residual
`7.8906612729366679e-07`, reduced 26-card 1911 data rows across three exact
groups ending at residual `2.2183427389532158e-16`, reduced 25-stone arch 2646
data rows ending at residual `9.9650461738801354e-07`, and reduced 101-stone
arch 3913 data rows ending at residual `9.9001648754006507e-07`.

The report/tracker files were synchronized so future agents keep answering
"No" to the full paper-coverage question until full-contact, long-run,
snapshot, timing, and external-baseline gaps are closed.

The fifty-seventh slice tightened the task tracking and rejected another local
solver shortcut. `AGENT_CONTINUATION.md` now explicitly records that OSG
renderer, `dart-demos` host, and reusable ImGui widget improvements are part of
this task whenever a paper GUI example cannot otherwise explain the scene,
show expected outcome/limits, expose useful overlays/diagnostics, capture a
stable image, or support robust interaction. The same slice tried ordering
local dense-polish contact corrections by descending dual-correction magnitude
after the existing global dense polish. The focused solver unit test passed and
the short card-house probe kept 56 contacts clean in `4925.2 ms`, but 60
contacts still fell back with failed residual `6.166e-5`, matching the kept
single worst-dual-contact local polish. The experiment was reverted and should
not be repeated without a new residual target or acceptance rule.

## Continue From Here

1. Run the focused gates if they are not already green:

   ```bash
   pixi run cmake --build build/default/cpp/Release --target UNIT_math_CoulombCone --parallel 8
   pixi run cmake --build build/default/cpp/Release --target UNIT_math_ExactCoulombContactProblem --parallel 8
   pixi run cmake --build build/default/cpp/Release --target UNIT_math_ExactCoulombFbfSolver --parallel 8
   pixi run cmake --build build/default/cpp/Release --target test_ExactCoulombFbfPaperFixtures --parallel 8
   pixi run cmake --build build/default/cpp/Release --target BM_INTEGRATION_exact_coulomb_fbf_paper --parallel 8
   pixi run cmake --build build/default/cpp/Release --target fbf_paper_trace --parallel 8
   pixi run cmake --build build/default/cpp/Release --target fbf_paper_gamma_sweep --parallel 8
   pixi run cmake --build build/default/cpp/Release --target fbf_paper_card_house_probe --parallel 8
   ctest --test-dir build/default/cpp/Release -R UNIT_math_CoulombCone --output-on-failure
   ctest --test-dir build/default/cpp/Release -R UNIT_math_ExactCoulombContactProblem --output-on-failure
   ctest --test-dir build/default/cpp/Release -R UNIT_math_ExactCoulombFbfSolver --output-on-failure
   ctest --test-dir build/default/cpp/Release -R test_ExactCoulombFbfPaperFixtures --output-on-failure
   ./build/default/cpp/Release/bin/BM_INTEGRATION_exact_coulomb_fbf_paper --benchmark_list_tests
   ./build/default/cpp/Release/bin/BM_INTEGRATION_exact_coulomb_fbf_paper --benchmark_filter='BM_PaperFixtureStepTime/(backspin_boxed_lcp|backspin_exact_fbf)$' --benchmark_min_time=0.001s --benchmark_repetitions=1
   ./build/default/cpp/Release/tests/integration/test_ExactCoulombFbfPaperFixtures --gtest_filter='ExactCoulombFbfPaperFixtures.MasonryArch25*' --gtest_output=xml:/tmp/fbf_arch25.xml
   ./build/default/cpp/Release/tests/integration/test_ExactCoulombFbfPaperFixtures --gtest_filter='ExactCoulombFbfPaperFixtures.MasonryArch101*' --gtest_output=xml:/tmp/fbf_arch101.xml
   ./build/default/cpp/Release/tests/integration/test_ExactCoulombFbfPaperFixtures --gtest_filter='ExactCoulombFbfPaperFixtures.CardHouseTenLevelSceneBuilds'
   ./build/default/cpp/Release/bin/BM_INTEGRATION_exact_coulomb_fbf_paper --benchmark_filter='BM_PaperFixtureStepTime/card_house_10_construction_boxed_lcp$' --benchmark_min_time=0.001s --benchmark_repetitions=1
   ./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace backspin exact_fbf 120
   ./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace backspin boxed_lcp 120
   ./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace backspin exact_fbf 1 1 nan residual_history
   ./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace card_house_26_reduced_contact exact_fbf 1
   ./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace card_house_26_reduced_contact boxed_lcp 1
   ./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace card_house_26_reduced_contact exact_fbf 1 1 nan dynamic_bodies
   ./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace card_house_26_reduced_contact boxed_lcp 1 1 nan dynamic_bodies
   ./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace card_house_26_reduced_contact exact_fbf 1 1 nan residual_history
   ./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace masonry_arch_25_reduced_contact exact_fbf 1
   ./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace masonry_arch_25_reduced_contact boxed_lcp 1
   ./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace masonry_arch_25_reduced_contact exact_fbf 1 1 nan dynamic_bodies
   ./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace masonry_arch_25_reduced_contact exact_fbf 1 1 nan residual_history
   ./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace masonry_arch_101_reduced_contact exact_fbf 1
   ./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace masonry_arch_101_reduced_contact boxed_lcp 1
   ./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace masonry_arch_101_reduced_contact exact_fbf 1 1 nan dynamic_bodies
   ./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace masonry_arch_101_reduced_contact exact_fbf 1 1 nan residual_history
   ./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace backspin exact_fbf 120 240 0.05
   ./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_gamma_sweep backspin nan,0.5,0.05,0.01 240 1 1
   ./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_gamma_sweep card_house_26_reduced_contact nan 1 56 1
   ./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_gamma_sweep masonry_arch_25_reduced_contact nan,0.001 1 48 2
   ./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_gamma_sweep masonry_arch_101_reduced_contact nan,0.001 1 38 2
   ./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_card_house_probe 56,58,59,60,64
   ./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_card_house_probe 56,60 1 30000 120 32 200 nan 0 0 0.9 10 1.5 30001
   pixi run demos -- --list-scenes
   pixi run demos -- --verify-fbf-scene-docs
   pixi run demos -- --cycle-scenes --frames 1
   pixi run capture fbf_paper_backspin /tmp/fbf_paper_backspin.png 640 480 1
   pixi run image-verdict /tmp/fbf_paper_backspin.png
   pixi run docs-build
   pixi run lint
   pixi run build
   ```

   Interactive GUI inspection commands, not batch gates:

   ```bash
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

2. Expand the small paper fixtures:

   - replace or corroborate the Painleve proxy with the authors' exact scene if
     the paper implementation becomes available,
   - expand the incline, backspin, turntable, and Painleve fixtures into full
     parameter sweeps, visual/state snapshots, exported residual traces, and
     full timing reports with hardware/build metadata and external baselines
     when available.

3. Add true contact-manifold identity and scratch reuse for the opt-in FBF path
   once the first paper fixtures are connected to real contact sets.

4. Keep the enabled two-card A-frame green, then make the 26-card Fig. 6
   scaffold execute within a bounded time:

   ```bash
   ./build/default/cpp/Release/tests/integration/test_ExactCoulombFbfPaperFixtures --gtest_filter=ExactCoulombFbfPaperFixtures.CardHouseAFramePrecursorStands
   ./build/default/cpp/Release/tests/integration/test_ExactCoulombFbfPaperFixtures --gtest_filter=ExactCoulombFbfPaperFixtures.CardHouseFourLevelSceneBuilds
   ./build/default/cpp/Release/tests/integration/test_ExactCoulombFbfPaperFixtures --gtest_filter=ExactCoulombFbfPaperFixtures.CardHouseFourLevelOneStepReducedContactProbe --gtest_output=xml:/tmp/fbf_card_probe.xml
   ./build/default/cpp/Release/tests/integration/test_ExactCoulombFbfPaperFixtures --gtest_filter=ExactCoulombFbfPaperFixtures.CardHouseFourLevelSettleProjectileScaffoldRuns
   ./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace card_house_26_reduced_contact exact_fbf 1
   ./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace card_house_26_reduced_contact boxed_lcp 1
   ./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace card_house_26_reduced_contact exact_fbf 1 1 nan dynamic_bodies
   ./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace card_house_26_reduced_contact boxed_lcp 1 1 nan dynamic_bodies
   ./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace card_house_26_reduced_contact exact_fbf 1 1 nan residual_history
   ./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace card_house_26_settle_projectile_reduced_contact exact_fbf 1 2 nan phase_summary
   ./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace card_house_26_settle_projectile_reduced_contact boxed_lcp 1 2 nan phase_summary
   ./build/default/cpp/Release/bin/BM_INTEGRATION_exact_coulomb_fbf_paper --benchmark_filter='BM_PaperFixtureStepTime/card_house_26_settle_projectile_reduced_contact_(boxed_lcp|exact_fbf)$' --benchmark_min_time=0.001s --benchmark_repetitions=1
   ```

   The 56-contact reduced one-step probe passes with `step_size_scale=10` and
   `outer_relaxation=1.5`; 58 contacts is clean but too slow for the default
   rung, 59 contacts can solve without fallback through the projected-gradient
   retry-continuation path but is still too slow for the default rung, and
   60 contacts is the current one-step fallback boundary; the history-enabled
   probe confirms the failed residual reaches the 30000-iteration cap and is
   nearly flat at the tail, with best failed residual only slightly below the
   final failed residual. The next technical target is removing the
   60/64-contact fallbacks, or adding a
   matrix-free/scratch-reusing contact solve path that can handle the full
   contact count before extending to the paper's 6.7 s no-creep settle,
   four-projectile impact outcome, visual snapshots, and timing report. The
   current reduced settle/projectile scaffold is useful for wiring tests,
   traces, benchmarks, and GUI controls, but it is not yet that parity target.

5. Keep the source comments and DART 6 RTD citation page current if the
   authors publish canonical BibTeX or make the scene/code repository public.

6. Promote the 25-stone arch beyond the reduced scaffold:

   ```bash
   ./build/default/cpp/Release/tests/integration/test_ExactCoulombFbfPaperFixtures --gtest_filter='ExactCoulombFbfPaperFixtures.MasonryArch25*' --gtest_output=xml:/tmp/fbf_arch25.xml
   ./build/default/cpp/Release/bin/BM_INTEGRATION_exact_coulomb_fbf_paper --benchmark_filter='BM_PaperFixtureStepTime/masonry_arch_25_reduced_contact_.*' --benchmark_min_time=0.001s --benchmark_repetitions=1
   ./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace masonry_arch_25_reduced_contact exact_fbf 1
   ./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace masonry_arch_25_reduced_contact boxed_lcp 1
   ./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace masonry_arch_25_reduced_contact exact_fbf 1 1 nan dynamic_bodies
   ./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace masonry_arch_25_reduced_contact exact_fbf 1 1 nan residual_history
   pixi run demos -- --scene fbf_paper_masonry_arch_25
   ```

   The current default 48-contact / two-contact-per-pair exact-FBF rung is
   clean with a 20000 outer-iteration arch budget. Use this probe to verify
   that 49/2 still produces only 48 actual contacts:

   ```bash
   ./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_arch_probe arch25 48,49 2 20000
   ```

   Higher cap diagnostics are clean but expensive: 64/4 takes about
   `31.1 s`, 80/4 takes about `117.6 s`, and 100/4 timed out under a 120 s
   guard before producing a row. No projectile/rest/post-impact parity has
   been implemented.

7. Promote the 101-stone arch beyond the reduced scaffold:

   ```bash
   ./build/default/cpp/Release/tests/integration/test_ExactCoulombFbfPaperFixtures --gtest_filter='ExactCoulombFbfPaperFixtures.MasonryArch101*' --gtest_output=xml:/tmp/fbf_arch101.xml
   ./build/default/cpp/Release/bin/BM_INTEGRATION_exact_coulomb_fbf_paper --benchmark_filter='BM_PaperFixtureStepTime/masonry_arch_101_reduced_contact_.*' --benchmark_min_time=0.001s --benchmark_repetitions=1
   ./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace masonry_arch_101_reduced_contact exact_fbf 1
   ./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace masonry_arch_101_reduced_contact boxed_lcp 1
   ./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace masonry_arch_101_reduced_contact exact_fbf 1 1 nan dynamic_bodies
   ./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace masonry_arch_101_reduced_contact exact_fbf 1 1 nan residual_history
   pixi run demos -- --scene fbf_paper_masonry_arch_101
   ```

   The current default 38-contact / two-contact-per-pair exact-FBF rung is
   clean with a 20000 outer-iteration arch budget, and
   `fbf_paper_arch_probe` now shows one-step cap rows through 100/2 are also
   clean on the current approximate scaffold. The 100/2 probe row takes about
   `56.1 s` and ends at residual `9.944e-7`. No long-run balance or
   full-contact physical parity has been implemented.

8. Continue broader contact-rich fixtures and performance: four-level house of
   cards full settle/projectile phase, paper-parity 25-stone arch, 101-stone
   arch, and 10-level card house.

## Latest Slice: Masonry-arch contact-cap probe

This slice added `tests/benchmark/integration/fbf_paper_arch_probe.cpp` and
the matching CMake target. The helper is benchmark/report tooling only: it
does not add a core library dependency or change the default solver. It emits
CSV rows for `arch25`, `arch101`, or both, with requested cap, actual contacts,
solver options, residual components, exact/fallback counters, failed-contact
labels, matrix-free route flags, residual-history counters, and crown/state
sanity metrics.

Verification from 2026-07-09:

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

Results: the default command emitted clean 25-stone 48/2 and 101-stone 38/2
rows. The 25-stone 49/2 row still generated only 48 actual contacts; 64/4
solved in `31118.9 ms` with residual `9.9929351901614826e-07`; 80/4 solved in
`117628.4 ms` with residual `9.8899682550441172e-07`; and 100/4 timed out
under a 120 s guard before producing a row. The 101-stone rows corrected stale
boundary notes: 39/2, 40/2, 48/2, 56/2, 64/2, 80/2, and 100/2 all solved
without fallback, with the 100/2 row taking `56137.1 ms`, residual
`9.9444848815471616e-07`, 6892 FBF iterations, one exact solve, zero
exact-FBF failures, and zero boxed-LCP fallbacks.

This is still approximate one-step arch evidence, not Fig. 7/8 paper parity:
author/Rigid-IPC geometry, projectile/rest outcome, long-run 101-stone balance,
snapshots, external comparisons, and timing parity remain open.

## Prior Slice: Action-aware GUI capture for 26-card projectiles

## Prior Slice: Reduced 25-stone arch projectile scaffold

This slice added a bounded reduced-contact scaffold for the Fig. 7
projectile-at-crown setup without claiming paper parity. The integration test
`MasonryArch25ProjectileScaffoldRuns` runs the 25-stone arch at the current
48-contact / two-contact-per-pair reduced cap, launches one sphere toward the
crown after a bounded settle step, and checks finite state plus zero exact-FBF
failures/fallbacks. The trace exporter now accepts
`masonry_arch_25_projectile_reduced_contact`, with tracked projectile and
`dynamic_bodies` scopes. The benchmark app registers exact-FBF and boxed-LCP
rows for the same one-step reduced projectile scaffold. The GUI scene
`fbf_paper_masonry_arch_25` exposes a `Launch projectile` control in the Scene
panel and a `p` key action; its overview/expected/coverage text explains that
this is not the paper's complete post-impact arch outcome. The shared FBF
diagnostics widget now displays `Exact diagnostics: not run yet` before the
first exact solve, which avoids a misleading initial status in zero-step
captures.

Verification from 2026-07-09:

```bash
pixi run cmake --build build/default/cpp/Release --target test_ExactCoulombFbfPaperFixtures fbf_paper_trace BM_INTEGRATION_exact_coulomb_fbf_paper dart-demos --parallel 8
./build/default/cpp/Release/tests/integration/test_ExactCoulombFbfPaperFixtures --gtest_filter=ExactCoulombFbfPaperFixtures.MasonryArch25ProjectileScaffoldRuns
./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace masonry_arch_25_projectile_reduced_contact exact_fbf 1 1 nan tracked
./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace masonry_arch_25_projectile_reduced_contact boxed_lcp 1 1 nan tracked
./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace masonry_arch_25_projectile_reduced_contact exact_fbf 1 1 nan dynamic_bodies
./build/default/cpp/Release/bin/BM_INTEGRATION_exact_coulomb_fbf_paper --benchmark_filter='BM_PaperFixtureStepTime/masonry_arch_25_projectile_reduced_contact_(boxed_lcp|exact_fbf)$' --benchmark_min_time=0.01s --benchmark_repetitions=1 --benchmark_display_aggregates_only=false
pixi run demos -- --verify-fbf-scene-docs
pixi run demos -- --list-scenes
pixi run capture-action fbf_paper_masonry_arch_25 p docs/dev_tasks/fbf_exact_coulomb_friction/assets/fbf_paper_masonry_arch_25_projectile.png 1280 720 0
pixi run image-verdict docs/dev_tasks/fbf_exact_coulomb_friction/assets/fbf_paper_masonry_arch_25_projectile.png
```

The focused test passed in `10611 ms`. The exact-FBF tracked trace ended with
the projectile body at `(2.3658678860997049e-05, -0.24838572818678054,
1.1973289982365207)`, 48 contacts, one exact solve, zero fallbacks, residual
`9.9901843912475669e-07`, and `success`; the boxed-LCP tracked trace and
exact-FBF dynamic-body trace also emitted rows. The benchmark rows passed:
boxed-LCP ran in `0.497 ms`, and exact-FBF ran in `4921 ms` with 48 contacts,
one exact solve, zero failures/fallbacks, max residual `999.018n`, and `2205`
max/total FBF iterations. The FBF scene-doc verifier checked all nine FBF
scenes, the scene list showed `fbf_paper_masonry_arch_25` under `Research`,
and the action capture wrote a visible 1280x720 projectile screenshot with a
passing default image verdict. A parallel local build attempt against the same
Ninja tree hit a generated-deps race, so rerun build/demo commands serially.

This slice refreshed `AGENT_CONTINUATION.md`, `README.md`,
`gui-capture-report.md`, `paper-parity-matrix.md`, and `PR_REPORT.md` so the
new evidence is tracked without weakening the active-task guardrails.

## Prior Slice: Action-aware GUI capture for 26-card projectiles

This slice made the GUI projectile state reproducible in headless captures.
`DemoHost` now stores `--headless-action <key>` requests and invokes matching
scene key actions after the scene installs and before capture steps run.
`pixi.toml` now exposes that path as `pixi run capture-action`, and
`fbf_paper_card_house_26` binds `p - Launch 4 projectiles` to the same launch
callback used by the Scene-panel button. This is a reusable host/widget
improvement for GUI examples whose important state is reached through a
control, not an FBF-specific special case in the host.

Verification from 2026-07-09:

```bash
pixi run cmake --build build/default/cpp/Release --target dart-demos --parallel 8
pixi run capture-action fbf_paper_card_house_26 p docs/dev_tasks/fbf_exact_coulomb_friction/assets/gui_captures/fbf_paper_card_house_26_projectiles.png 1280 720 0
pixi run image-verdict docs/dev_tasks/fbf_exact_coulomb_friction/assets/gui_captures/fbf_paper_card_house_26_projectiles.png
pixi run demos -- --verify-fbf-scene-docs
```

The capture command wrote the saved projectile screenshot, and visual
inspection confirmed four projectile spheres beside the card house with the
Scene-tab overview/expected/coverage text visible. The image verdict passed
the default non-blank gate and wrote
`assets/gui_captures/fbf_paper_card_house_26_projectiles.verdict.json`;
contrast remains weak but is not required by the default verdict. A parallel
`pixi run demos -- --verify-fbf-scene-docs` plus `pixi run capture-action ...`
attempt reproduced the known `dart-demos` archive/link race, and both commands
passed when rerun serially. This is still a reduced scaffold GUI smoke, not
paper-matched Fig. 6 dynamic snapshot parity.

Final gates after the action-capture report refresh:

```bash
pixi run demos -- --verify-fbf-scene-docs
pixi run docs-build
pixi run lint
pixi run build
git diff --check
```

The binary-skipping untracked-file whitespace loop was also rerun. All of
these passed after the latest documentation edits.

## Prior Slice: Reduced 26-card settle/projectile phase scaffold

This slice added a bounded reduced-contact scaffold for the Fig. 6
settle/projectile sequence without claiming paper parity. The integration test
`CardHouseFourLevelSettleProjectileScaffoldRuns` runs a smoke-capped exact-FBF
world for one settle step, launches four projectiles, then steps the projectile
phase and checks finite state plus zero exact-FBF failures/fallbacks. The trace
exporter now accepts `card_house_26_settle_projectile_reduced_contact` and the
`phase_summary` scope, which emits initial, settle, and projectile rows. The
benchmark app registers exact-FBF and boxed-LCP rows for the same two-step
scaffold. The GUI scene `fbf_paper_card_house_26` exposes a `Launch 4
projectiles` control in the Scene panel and its overview/expected/coverage
text explains that this is a reduced phase scaffold, not the paper's complete
6.7 s settle and 10 s impact sequence.

Verification from 2026-07-09:

```bash
pixi run cmake --build build/default/cpp/Release --target test_ExactCoulombFbfPaperFixtures fbf_paper_trace BM_INTEGRATION_exact_coulomb_fbf_paper dart-demos --parallel 8
./build/default/cpp/Release/tests/integration/test_ExactCoulombFbfPaperFixtures --gtest_filter=ExactCoulombFbfPaperFixtures.CardHouseFourLevelSettleProjectileScaffoldRuns
./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace card_house_26_settle_projectile_reduced_contact exact_fbf 1 2 nan phase_summary
./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace card_house_26_settle_projectile_reduced_contact boxed_lcp 1 2 nan phase_summary
./build/default/cpp/Release/bin/BM_INTEGRATION_exact_coulomb_fbf_paper --benchmark_filter='BM_PaperFixtureStepTime/card_house_26_settle_projectile_reduced_contact_(boxed_lcp|exact_fbf)$' --benchmark_min_time=0.001s --benchmark_repetitions=1
pixi run demos -- --verify-fbf-scene-docs
./build/default/cpp/Release/bin/dart-demos --list-scenes | rg "fbf_paper_card_house_26|Research"
```

The CTest passed. The exact-FBF phase-summary trace ended with 26 cards, 4
projectiles, finite state, 56 contacts, 6 exact solves, zero fallbacks,
residual `4.8919831295901383e-17`, and `success`. The boxed-LCP trace emitted
the same phase rows with 56 contacts. The benchmark rows passed: boxed-LCP ran
in `2.17 ms`, and exact-FBF ran in `12057 ms` with 56 contacts, 6 exact solves,
zero failures/fallbacks, max FBF iterations `3631`, total FBF iterations
`5539`, and max residual `2.218e-16`. The
`pixi run demos -- --verify-fbf-scene-docs` command checked all nine FBF
scenes, and the direct catalog check confirmed `fbf_paper_card_house_26` under
`Research`. A
`pixi run demos -- --list-scenes | rg ...` wrapper probe returned no grep match
in this slice, so prefer the direct binary catalog command until that wrapper
behavior is understood.

This slice also refreshed `AGENT_CONTINUATION.md`, `README.md`,
`paper-parity-matrix.md`, and `PR_REPORT.md` so the new evidence is tracked
without weakening the active-task guardrails. The final verification pass after
those report edits reran `pixi run demos -- --verify-fbf-scene-docs`,
`pixi run docs-build`, `pixi run lint`, `pixi run build`, `git diff --check`,
and the output-based binary-skipping untracked-file whitespace loop.

## Prior Slice: Host-rendered FBF Scene-tab documentation

The latest slice moved FBF GUI explanation rendering into the reusable
`dart-demos` host. `DemoHost` now stores the active
`ScenePanelDocumentation`, clears it during scene teardown, and renders the
overview, expected result, and coverage text at the top of the scrollable
`Scene` tab before custom scene controls. `FbfPaperFrictionScene.cpp` no
longer renders the same explanation block itself; its custom panel now starts
at solver controls and diagnostics. This closes the gap where the metadata was
catalog-verifiable but only FBF-specific code made it visible.

Verification from 2026-07-09:

```bash
pixi run demos -- --verify-fbf-scene-docs
pixi run demos -- --list-scenes
pixi run capture fbf_paper_backspin /tmp/fbf_paper_backspin.png 640 480 5
pixi run image-verdict /tmp/fbf_paper_backspin.png
pixi run lint
pixi run docs-build
pixi run build
git diff --check
```

The verifier checked all 9 FBF paper scenes. The catalog listed all nine
`fbf_paper_*` scene IDs under `Research`. The capture wrote
`/tmp/fbf_paper_backspin.png`, showed the host-rendered `Scene` tab overview,
expected result, and coverage text beside the rendered backspin sphere, and
passed the default non-blank image verdict. The all-scene capture batch was
also rerun after the host-renderer change: it refreshed all nine
`assets/gui_captures/fbf_paper_*.png` files, their verdict JSON files, and
`assets/fbf_gui_capture_sheet.png`; all nine verdicts passed and the sheet
shows the Scene tab visible for each row. `pixi run lint`, `pixi run
docs-build`, `pixi run build`, `git diff --check`, and the binary-skipping
untracked-file whitespace loop passed after the edits. This is GUI
self-containedness evidence only; it does not add missing paper trajectories,
full-contact house/arch runs, long-run snapshots, or external baselines.

## Guardrails

- Do not claim paper completion from unit tests or from one small scene.
- Do not change the default DART 6 solver until the opt-in FBF path has
  explicit parity evidence and gz compatibility is considered.
- External comparison dependencies such as Kamino, MuJoCo, or the paper
  implementation are acceptable for tests, examples, and benchmarks when
  available, but not as core DART library dependencies.
- Do not hand-edit generated AI command files.
- Keep durable theory in `docs/background/` only when this task is ready to
  retire; while active, this folder owns the handoff state.
