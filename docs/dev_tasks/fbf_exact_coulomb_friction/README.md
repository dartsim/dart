# FBF exact Coulomb friction for DART 6.20

## Status

Active research and implementation task for integrating the SCA 2026
"A Splitting Architecture for Exact Reduced Coulomb Friction" method into the
DART 6.20 release lane.

Before making or reporting progress, read
[AGENT_CONTINUATION.md](AGENT_CONTINUATION.md). It is the task-local progress
ledger, active work board, todo list, verification ledger, and guardrail
against prematurely calling this work complete. Focused passing tests,
small-fixture benchmarks, and demo smoke tests are progress evidence, not
proof that all paper tests, benchmarks, and GUI examples exist. Each slice that
changes implementation state must update that tracker and mirror user-facing
commands/results into [PR_REPORT.md](PR_REPORT.md).

Current branch: `research/fbf-friction-release620`, originally based on
`origin/release-6.20` at `c1deaca67b8`. As of 2026-07-08,
`origin/release-6.20` has advanced to `f343528708e`; merge or otherwise
refresh the topic branch before publishing.

Current conclusion: the method is implementable in DART 6.20, but it is not a
drop-in replacement for the existing boxed-LCP backend. It needs a new
reduced contact-space exact-Coulomb solver path, contact-problem assembly that
preserves a circular Coulomb cone per contact, shared cone math/residual
utilities, and paper-specific benchmark scenes. The default solver and gz
compatibility surfaces must remain unchanged until parity evidence justifies
any broader exposure.

## Source Material Checked

- Project page:
  <https://www.cs.ubc.ca/research/fbf-friction/>
- Paper PDF:
  <https://www.cs.ubc.ca/research/fbf-friction/paper.pdf>
- Video:
  <https://www.youtube.com/watch?v=5THad4PAGmI>
- Video metadata checked on 2026-07-08: published
  `2026-07-06T20:53:01-07:00`, length 82 seconds, description links the
  project page, paper, and code repository.
- Code repository named by the video:
  <https://github.com/matthcsong/fbf-sca-2026>. Rechecked on 2026-07-09:
  the GitHub page still returned HTTP 404 and unauthenticated `git ls-remote`
  could not read it. Treat paper and video as the source of record until that
  repository becomes available.
- The paper credits the masonry-arch geometry to the Rigid-IPC dataset:
  <https://github.com/ipc-sim/rigid-ipc>.

## Paper Summary

The paper targets the exact reduced Coulomb friction law in contact space.
After semi-implicit Euler time stepping, velocities are eliminated to obtain

```text
v(lambda) = W lambda + v_free,       W = J M^-1 J^T
```

where each contact reaction has one normal and two tangential components. The
Coulomb cone is

```text
K_mu = { lambda | lambda_n >= 0, ||lambda_t|| <= mu lambda_n }.
```

The exact law uses the De Saxce-Feng augmented velocity

```text
v_tilde = v + mu ||v_t|| e_n
```

and requires `v_tilde` in the dual cone, `lambda` in the primal cone, and
orthogonality between them.

The solver splits the reduced inclusion into:

- `A(lambda) = W lambda + v_free + N_K(lambda)`, a cone-constrained linear
  response.
- `B(lambda) = mu ||v_t(lambda)|| e_n`, the scalar non-associated coupling.

The outer loop is a Tseng-style forward-backward-forward iteration:

1. Evaluate `v(lambda_k)` and the coupling `g_k = B(lambda_k)`.
2. Solve a strongly convex cone QP with `W + gamma^-1 I` and frozen `g_k`.
3. Evaluate `B` at the intermediate reaction.
4. Correct explicitly by the coupling change and project back to the Coulomb
   cone.
5. Stop on a dimensionless Coulomb residual that separately measures primal
   cone feasibility, dual cone feasibility, and complementarity gap.

Important numerical details from the paper:

- The outer step size `gamma` is algorithmic, not the time step.
- The base step is `0.5 / (mu_max * lambda_max(W))`, with `lambda_max(W)`
  estimated by about ten power iterations.
- The safeguarded rule accepts a trial if the local coupling-variation ratio is
  at most `0.9`, otherwise shrinks by `0.7`. Growth is asymmetric: no growth
  inside a solve, and cross-step growth is capped by the safe base step.
- The reported tolerance is `1e-6`; the implementation is single precision, so
  tighter residuals would mostly test rounding noise on large contact sets.
- Residual normalization uses fixed per-solve impulse and velocity scales:
  `s_r = max(||lambda_0||, ||v_f|| / lambda_max(W), 1e-12)` and
  `s_u = max(||v_tilde(lambda_0)||, ||v_f||, 1e-12)`.
- The default inner solver is matrix-free block Gauss-Seidel, with 10 sweeps
  per outer iteration and 30 on the arch. The outer architecture is intended
  to allow other strongly convex SOCP solvers.

See [paper-notes.md](paper-notes.md) for the retained source digest and
[paper-parity-matrix.md](paper-parity-matrix.md) for the examples, figures,
tables, and benchmark targets that DART needs to reproduce. See
[PR_REPORT.md](PR_REPORT.md) for the PR-facing command and result summary,
[residual-history-report.md](residual-history-report.md) for the current
Figure 9 residual-history evidence, and
[gui-capture-report.md](gui-capture-report.md) for the current one-step
`dart-demos` GUI capture evidence. The GUI examples are run through the
consolidated app with commands like
`pixi run demos -- --scene fbf_paper_backspin`; the FBF Scene-tab explanation
metadata is checked by `pixi run demos -- --verify-fbf-scene-docs`; bounded
headless GUI smoke uses
`pixi run capture fbf_paper_backspin /tmp/fbf_paper_backspin.png 640 480 1`
followed by `pixi run image-verdict /tmp/fbf_paper_backspin.png`. For
action-state GUI scaffolds, use `pixi run capture-action`; current examples
include `fbf_paper_card_house_26 p` for the reduced four-projectile phase and
`fbf_paper_masonry_arch_25 p` for the reduced crown-projectile scaffold.

## DART 6.20 Baseline

The current DART 6.20 contact path is the legacy `ConstraintSolver` /
`BoxedLcpConstraintSolver` pipeline:

- `ContactConstraint` contributes one normal row and two tangential rows per
  contact when friction is enabled.
- Tangential rows use ODE-style `findex` coupling with box bounds
  `[-mu, +mu]` scaled by the normal impulse.
- `BoxedLcpConstraintSolver` assembles a dense Delassus matrix by impulse
  tests, solves with the configured boxed-LCP solver, and falls back to a
  secondary solver when configured.
- This representation approximates the Coulomb disk with two fixed tangent
  directions. It does not represent the exact circular cone projection used by
  the FBF paper and does not contain the De Saxce-Feng coupling.

Therefore, replacing `DantzigBoxedLcpSolver` or `PgsBoxedLcpSolver` alone is
not enough. FBF needs a separate contact-space solver family that can own the
exact cone law while preserving the existing boxed-LCP default.

## Shared Components Landed

Current state note: a later slice added an incremental inner block-Gauss-Seidel
solver and a default-on scratch-backed `ExactCoulombContactRowOperator`, which
together cleared the 60/64-contact one-step card-house fallback boundary
described below and made full natural contact manifolds solve one-step (card
house 108 contacts clean in `2.5 s`). A subsequent slice replaced the
approximate arch scaffolds' static-endpoint geometry with an author-faithful
Rigid-IPC weighted-catenary port (`dart/math/detail/MasonryArchGeometry.hpp`,
MIT, commit `23b6ba6fbf8`, all stones dynamic), under which the natural
manifolds are 52 contacts (25-stone) and 204 contacts (101-stone), both clean
one-step. The current open frontier is multi-step convergence: probing the
full-manifold card-house settle across steps shows steps 2+ reaching
122-138 contacts, cross-step warm-started, that still fall back at residuals
`2.9e-6`-`1.5e-4`. See `AGENT_CONTINUATION.md` for the full current-state
ledger; the 56/58/59/60/64-contact one-step boundary narrative and the
approximate/static-endpoint arch description below are historical and
superseded where noted.

This branch adds reusable normal-first exact-Coulomb math utilities:

- `dart/math/detail/CoulombCone.hpp`
  - `projectCoulombConeNormalFirst([normal, tangent1, tangent2], mu)`
  - `projectCoulombDualConeNormalFirst([normal, tangent1, tangent2], mu)`
  - `computeCoulombConeResidualNormalFirst(...)`
- `dart/math/detail/ExactCoulombContactProblem.hpp`
  - `ExactCoulombContactProblem`
  - `computeExactCoulombContactVelocityNormalFirst(...)`
  - `computeExactCoulombAugmentedVelocityNormalFirst(...)`
  - `computeExactCoulombContactResidualNormalFirst(...)`
  - `estimateLargestExactCoulombDelassusEigenvalue(...)`
- `dart/math/detail/ExactCoulombFbfSolver.hpp`
  - `solveExactCoulombFrozenConeProjectedGradient(...)`
  - `solveExactCoulombFrozenConeBlockGaussSeidel(...)`
  - `computeExactCoulombDelassusDiagonalBlocksNormalFirst(...)`
  - `computeExactCoulombFbfSafeStepSize(...)`
  - `computeExactCoulombFbfCouplingNormalFirst(...)`
  - `computeExactCoulombFbfCouplingVariationRatio(...)`
  - `computeExactCoulombFbfResidualScales(...)`
  - `solveExactCoulombFbf(...)`
  - bounded opt-in `residualHistory` diagnostics for per-outer residual
    convergence sampling
- `dart/constraint/detail/ExactCoulombConstraintAdapter.hpp`
  - `buildExactCoulombConstraintProblem(...)`
  - `applyExactCoulombConstraintDelassus(...)`
  - `applyExactCoulombConstraintImpulses(...)`
- `dart/constraint/ExactCoulombFbfConstraintSolver.hpp`
  - `ExactCoulombFbfConstraintSolver`
  - `ExactCoulombFbfConstraintSolverOptions`
  - boxed-LCP fallback diagnostics for unsupported or failed groups
  - opt-in last exact solve residual-history diagnostics

The residual helper reports the paper's dimensionless primal cone feasibility,
dual cone feasibility, complementarity gap, and max residual for one contact or
a product of contact cones. These helpers use DART's contact row order, handle
zero friction explicitly, take explicit reaction/velocity scales, and have unit
coverage in `tests/unit/math/test_CoulombCone.cpp`.

The contact-problem helper keeps `W = J M^-1 J^T` matrix-free: callers supply a
Delassus operator, while the reusable math layer owns contact-space velocity,
De Saxce-Feng augmentation, residual assembly, and deterministic power
iteration for the safe FBF base-step eigenvalue estimate. Focused coverage
lives in `tests/unit/math/test_ExactCoulombContactProblem.cpp`.

The FBF helper implements the paper's outer-loop structure, a matrix-free
projected-gradient frozen cone-QP reference, and a reference contact-block
Gauss-Seidel frozen cone-QP solver using per-contact 3x3 diagonal Delassus
blocks plus matrix-free global products. It covers safe spectral step
initialization, local coupling-variation acceptance, safeguarded shrink by
`0.7`, explicit FBF correction, projection back to the Coulomb cone, the
paper's fixed per-solve residual scales, and residual-based stopping.
Residual-history recording is off by default and is bounded by an explicit
sample cap when enabled, so normal solver use does not store per-outer
diagnostics. Focused coverage lives in
`tests/unit/math/test_ExactCoulombFbfSolver.cpp`.
The helper itself remains an internal math path, but it is now exercised
through the opt-in DART 6 constraint solver route described below. Production
performance tuning, full paper-scene validation, and contact-manifold-level
warm starts remain open.

The first DART 6 contact-assembly bridge now builds an exact-Coulomb contact
problem from existing three-row contact-style `ConstraintBase` rows. It
validates isotropic ODE-style friction rows, converts the boxed-LCP right-hand
side into `v_free = -b`, assembles a dense Delassus snapshot with the same
impulse-test mechanism used by `BoxedLcpConstraintSolver`, can apply solved
reaction triples back through the original constraints, and now has a helper
for applying the adapted Delassus operator through the retained constraint rows
without reading that dense snapshot. Focused coverage lives in
`tests/unit/constraint/test_ExactCoulombConstraintAdapter.cpp`, with real
`ContactConstraint` smokes in `tests/integration/test_ConstraintSolver.cpp`.
This is a staging bridge, not the final solver route: unsupported
non-contact/scalar/anisotropic groups still need to remain on boxed LCP, and
the production path still needs to remove dense snapshot dependence from
seeding and recovery, add scratch reuse, harden split or mixed-group policy,
and decide opt-in `World` exposure.

The first opt-in DART 6 solver route now exists as
`ExactCoulombFbfConstraintSolver`. It subclasses `BoxedLcpConstraintSolver`,
attempts the adapter + FBF + block-Gauss-Seidel solve for supported
isotropic contact groups, applies solved reaction triples through the original
constraints, and falls back to the existing boxed-LCP group solve for
unsupported or failed groups unless fallback is disabled in options. Focused
coverage lives in
`tests/unit/constraint/test_ExactCoulombFbfConstraintSolver.cpp`.
It now carries a conservative warm-start cache: after a successful exact solve,
the next solve reuses the previous reaction only when the constraint pointer
sequence is identical, projects that cached reaction back into the current
Coulomb cones, and exposes diagnostics for whether warm start was used and how
many outer FBF iterations ran.
The opt-in solver also now has a reference robustness retry: if the default
contact-block Gauss-Seidel frozen cone solve fails the exact-FBF outer solve,
it can retry the same exact problem with the matrix-free projected-gradient
frozen cone solver. When the block-GS FBF attempt reaches a finite
near-converged reaction, the projected-gradient retry now continues from that
reaction instead of restarting from the cold row guess. This is slower than
the paper-style block-GS route but keeps unsupported groups on the existing
boxed-LCP fallback and has focused unit coverage in
`test_ExactCoulombFbfConstraintSolver.cpp`. The solver exposes the last
exact-Coulomb residual components and projected-gradient retry diagnostics so
contact-rich failures can distinguish primal cone, dual cone, and
complementarity errors. It also has a bounded dense residual-polish attempt
for failed solves after block-GS FBF and projected-gradient retry have not
reached tolerance. This polish path is a DART 6 dense-prototype recovery aid,
not the paper-style matrix-free route; it is covered by a focused solver
regression and currently improves failed card-house residual reporting without
clearing the 60-contact boundary. The solver can also retain bounded
residual-history samples for the last exact solve when explicitly requested
by tests or benchmark/example trace tools.
The solver also has an opt-in `useMatrixFreeDelassusOperator` route that uses
the adapter's constraint-row Delassus helper for FBF `W*x` products. Focused
adapter tests, a synthetic solver test, and a one-contact `World` smoke verify
this route against the dense snapshot on small cases. This is partial
matrix-free integration only: the DART 6 bridge still assembles and retains the
dense snapshot for current staging and prototype dense residual polish. A new
benchmark/staging option, `useMatrixFreeDelassusSeed`, lets the solver seed
zero contact guesses from operator-extracted local diagonal Delassus blocks
when the matrix-free product route is requested; this removes the dense global
seed from that opt-in route but does not remove dense assembly. The card-house
cap probe can request this route with benchmark-only
`use_matrix_free_delassus_operator` and `use_matrix_free_delassus_seed`
arguments and records both requested options plus whether the solver used the
operator product and operator seed. Reduced-card probe rows confirm matching
residual behavior on 16/32/40-contact capped rows, but the repeated
impulse-test product route is much slower than dense multiplication and is not
the final paper-style performance path.
An opt-in `World` smoke in `tests/integration/test_ConstraintSolver.cpp`
installs this solver through `World::setConstraintSolver`, runs a capped
single-contact box/ground scene, checks exact-path diagnostics and residual,
and performs a bounded one-step state comparison against the default boxed-LCP
path.
The first paper-parameter fixture now lives in
`tests/integration/test_ExactCoulombFbfPaperFixtures.cpp`: it recreates the
incline cube threshold with slope `atan(0.5)`, `dt = 1/60`, `T = 2 s`, and
`mu = 0.5`/`0.4`, checks stick/slide behavior against the analytical
displacement, records per-step exact-path residual/fallback diagnostics, and
runs the same setup through the existing boxed-LCP solver as the DART 6
baseline.
The same test file now also covers the backspin sphere fixture with
`r = 0.25 m`, `v0 = 4 m/s`, `omega0 = -200 rad/s`, `mu = 0.5`, and
`dt = 1/60`, checking convergence to the analytical rolling state
`v_inf = -11.429 m/s`, `omega_inf = -45.71 rad/s` while keeping boxed-LCP as a
finite DART 6 baseline and checking the sampled exact residual trace.
It also now covers the turntable capture/ejection grid from Figure 4:
`mu = 0.2` with `omega = 2` and `5` ejects, while `mu = 0.5` with
`omega = 2` captures and `omega = 5` ejects. This is a headless
classification regression with DART boxed-LCP finite-baseline runs and sampled
exact residual trace checks; it is not yet the full radial-trajectory, visual
snapshot, residual-convergence plot, or timing report from the paper.
The same integration file now includes a Painleve-style proxy fixture for
Figure 5. The paper gives qualitative outcomes for a sliding box at
`mu = 0.5` and `mu = 0.55`, but does not publish the exact scene parameters;
the authors' code repository is still not anonymously readable in this
environment. The proxy therefore uses a slightly leaned tall box on a rough
horizontal plane, verifies that the `mu = 0.5` exact run slides/rests upright,
verifies that the `mu = 0.55` exact run reaches the tumble threshold after a
shorter travel distance, and keeps the existing DART 6 boxed-LCP solver as a
finite baseline while checking the sampled exact residual trace. This is not
yet author-scene parity and should be replaced or backed by the paper
implementation if the scene becomes available.
The first in-tree timing harness now lives in
`tests/benchmark/integration/bm_exact_coulomb_fbf_paper.cpp`. It registers
Google Benchmark rows for the small incline, backspin, turntable,
Painleve-proxy, reduced-contact 26-card one-step, and reduced-contact 26-card
settle/projectile scaffold scenes under both the opt-in exact-Coulomb FBF
solver and the existing DART 6 boxed-LCP solver. The FBF rows report exact
solve counts, warm starts, exact-FBF failure counts, boxed fallback counts,
contact counts, and max sampled residual so timing output cannot silently hide
a failed exact solve. This is DART-side
timing infrastructure only; it is not yet a paper hardware match, external
Kamino/MuJoCo comparison, or contact-rich benchmark result.
The first headless trace exporter now lives in
`tests/benchmark/integration/fbf_paper_trace.cpp`. It emits CSV rows for the
same small paper fixtures under either `exact_fbf` or `boxed_lcp`, including
body state, contact count, exact solve/warm-start/fallback counters, residual,
and exact-path status. It also covers the reduced-contact 26-card card-house
and 25/101-stone masonry-arch scaffolds as one-step top-card/crown-stone trace
probes. The optional sixth argument accepts `dynamic_bodies` or `full_scene`
to emit one row per mobile body instead of only the representative tracked
body, which gives scene-level CSV state rows for the reduced contact-rich
scaffolds. The same sixth argument accepts `residual_history` for exact-FBF
runs; that scope emits bounded per-outer residual rows for retained exact-solve
records, including multiple exact groups from one `World::step()` when the
solver is configured to keep them. The dedicated scenario
`card_house_26_settle_projectile_reduced_contact` accepts `phase_summary` as
the sixth argument and emits initial, settle, and projectile rows for a bounded
two-step reduced 26-card scaffold. This is a comparison artifact producer for
later trajectory/residual parity work; it is not an installed API, visual
snapshot, paper-hardware timing report, or external-baseline comparison.
The first contact-rich scaffold now exists as an enabled compiled test in
`tests/integration/test_ExactCoulombFbfPaperFixtures.cpp`:
`CardHouseAFramePrecursorStands`. It builds a two-card A-frame with
`mu = 0.8` as a minimal Fig. 6 precursor. Earlier runs with a `1200` outer cap
and relaxed `1e-5` tolerance fell back on most steps (`59` boxed-LCP fallbacks
over `60` steps, final residual about `3.7e-5`) and showed a dual-feasibility
dominated gap. Raising only the contact-rich precursor budget to a `5000`
outer cap and the paper `1e-6` tolerance makes the two-card A-frame solve with
zero boxed-LCP fallbacks, so it is now an enabled regression.

The first four-level, 26-card Fig. 6 scene scaffold also exists as the enabled
construction regression `CardHouseFourLevelSceneBuilds`. It constructs the
paper's plate count using four base A-frames, 3/2/1 support cards, and 3/2/1
upper A-frame rows. The scene remains approximate until the authors' files are
available. A manual one-step exact-FBF probe did not finish within 30 seconds
on the current dense Delassus prototype, so the next card-house blocker is
paper-style matrix-free/scratch contact solving, not just more fixture
layering. A reduced-contact dynamic rung now exists as
`CardHouseFourLevelOneStepReducedContactProbe`: it runs one exact-FBF step on
the same 26-card world with a practical default `max_contacts = 56` and
`max_contacts_per_pair = 1`, `step_size_scale = 10`,
`outer_relaxation = 1.5`, records residual/iteration/timing properties, and
passes locally after the paper residual-scale update at residual `2.218e-16`
with 3 exact solves, zero boxed-LCP fallbacks, zero exact-FBF failures,
last-group FBF iterations `0`, max group FBF iterations `1908`, and total FBF
iterations `1908`. The DART 6 contact adapter gives zero contact-row guesses a
residual-selected projected local/global Delassus cold-start seed with a
diagonal fallback, which preserves explicit row guesses and later solver warm
starts while improving the dense card-house cold start. The FBF loop recovers
the configured base step after an accepted shrink instead of permanently
pinning the step to an early reduced value, and the optional outer-relaxation
knob lets the reduced card-house rung over-relax accepted corrections without
changing the core solver default. The benchmark-only
`fbf_paper_card_house_probe`
helper records CSV rows for contact-cap, option, FBF step-size, and
outer-relaxation diagnostics. It also reports best residual/best iteration for
last and failed exact-FBF attempts, and now has a benchmark-only switch for
the opt-in constraint-row Delassus product route. It confirmed 56 contacts as the
practical clean default rung at a 30000 outer-iteration card-house budget at
the time; this is superseded (see the note at the top of this section) now
that the full natural manifold (108 contacts) solves clean one-step.
After switching the FBF residual to the paper's fixed per-solve scales, 56
contacts remains clean (`4971.0 ms`, max FBF iterations `1908`), 58 contacts
remains clean but slow (`68879.1 ms`, one projected-gradient retry), and 59
contacts remains clean but too slow (`76402.9 ms`, one projected-gradient
retry, max FBF iterations `7888`). The 60-contact row was the one-step
fallback boundary at `step_size_scale=10` at the time (superseded: the
incremental inner solver and contact-row operator now clear this boundary and
the full 108-contact manifold solves clean, per the note at the top of this
section). A history-enabled cap-probe
run retaining 30001 samples confirms the 60-contact failed group reaches
iteration 30000, moves from residual `1.146e-5` to pre-polish history residual
`5.338e-6`, and then dense-polishes the failed final residual to the best
failed residual `5.334e-6` at iteration `29355` with tail ratio `1.0000019`.
The current failure is slow near-cap dual convergence above the `1e-6`
paper-scaled tolerance, not a discarded converged iterate, missing final
success row, or renderer/reporting issue. Earlier 60-contact option probes
with `step_size_scale=12` and `outer_relaxation=2.0` did not clear the
boundary under the previous unit-scaled residual and need rechecking only if
that option path becomes active again.
The first dense-vs-matrix-free product probe rows show that the current
constraint-row route is correctness-compatible but not performance-ready: 16
and 32 contacts solve with the same iteration counts as dense rows while taking
`324.2 ms` versus `24.4 ms` and `3203.0 ms` versus `170.4 ms`; a 40-contact
1000-outer capped row fails at the same `3.966e-5` residual while taking
`18937.8 ms` versus `1057.4 ms`. A later operator-seed diagnostic rebuilt the
probe and ran final-code 16-contact matrix-free rows with
`use_matrix_free_delassus_seed=0` and `1`: both solved exactly with residual
`3.212e-16`, `336` total FBF iterations, and matching state diagnostics, while
the CSV proved `matrix_free_delassus_seed_used` toggled from `0` to `1`
(`354.1 ms` versus `354.5 ms`). Larger `32,40` and `56,60` row attempts with
the repeated impulse-test product were stopped by a 120 s bound before the
first CSV row, so the current blocker is still product cost/scratch design, not
seed selection. A solver-side batched scatter/gather product was investigated
and rejected in this slice because the generic `ConstraintBase` API does not
expose enough touched-body bias-impulse propagation to make it safe without a
broader dynamics API change.
The 64-contact row still has
no clean no-fallback result; the latest recorded 64-contact diagnostic failed
after 30000 outer iterations with failed dual residual `6.955e-6` and one
boxed-LCP fallback. This is useful progress against the contact-rich blocker,
but it is not the full Fig. 6 contact budget or dynamic parity. The trace
exporter now supports
`card_house_26_reduced_contact` for the same reduced-contact cap; exact-FBF and
boxed-LCP smoke commands emit one-step rows for `card_house_l3_f0_left_body`,
with the exact row reporting 56 contacts, 3 exact solves, zero fallbacks,
residual `2.218e-16`, and `success`. This is a top-card trace sanity check,
not the paper's full 6.7 s settle trace, projectile sequence, or convergence
plot.
The optional `dynamic_bodies` scope now emits 26 mobile-card rows at each
sample; the one-step exact-FBF full-scene smoke reports 56 contacts, 3 exact
solves, zero fallbacks, residual `2.218e-16`, and `success`. The
`residual_history` scope emits 1911 rows across the three exact groups on the
same one-step reduced scene: one 54-contact group ending at iteration `1908`
with residual `9.943e-7` and two one-contact groups already below tolerance at
iteration 0. Individual and combined reduced-scaffold Figure 9-style SVG plots
now exist, but paper-matched comparison plots, long-run histories, and
full-contact histories remain open.

A reduced settle/projectile phase scaffold now exists but remains deliberately
bounded. `CardHouseFourLevelSettleProjectileScaffoldRuns` launches four
projectiles after one settle step at a 16-contact smoke cap so CTest has a fast
guard for the phase wiring. The trace scenario
`card_house_26_settle_projectile_reduced_contact` runs the same phase shape at
the promoted 56-contact reduced cap: the exact-FBF phase-summary smoke emits
initial, settle, and projectile rows, ends with 26 cards, 4 projectiles, finite
state, 56 contacts, 6 exact solves, zero fallbacks, residual
`4.8919831295901383e-17`, and `success`. The matching boxed-LCP trace also
emits phase rows. Benchmark rows
`card_house_26_settle_projectile_reduced_contact_boxed_lcp` and
`card_house_26_settle_projectile_reduced_contact_exact_fbf` are registered; the
latest exact-FBF row ran in `12057 ms` with 56 contacts, 6 exact solves, zero
failures/fallbacks, max FBF iterations `3631`, total FBF iterations `5539`, and
max residual `2.218e-16`. The GUI scene `fbf_paper_card_house_26` has a
Scene-panel control to launch the same four projectiles. This is not the
paper's 6.7 s no-creep settle, real projectile-impact outcome, full-contact
run, long-run trace, paper snapshot, or timing parity.

The first 10-level card-house scaffold now exists as
`CardHouseTenLevelSceneBuilds`, benchmark row
`card_house_10_construction_boxed_lcp`, and GUI scene
`fbf_paper_card_house_10`. It builds the triangular 10-level construction with
155 cards plus ground. The GUI scene is intentionally static and defaults to
boxed-LCP diagnostics because exact-FBF dynamics for this contact-rich scene are
not implemented yet; the Scene panel explains that limitation. The boxed-LCP
one-step construction benchmark reports 155 cards, 512 contacts, zero exact-FBF
solves, and `1010 ms` local smoke time. This is construction/default-solver
evidence only, not the paper's dynamic 10-level stand/topple comparison,
residual trace, timing parity, external baseline, or snapshot evidence.

The first 25-stone masonry-arch scaffold now exists as
`MasonryArch25SceneBuilds` and `MasonryArch25OneStepReducedContactProbe`, plus
benchmark rows and GUI scene `fbf_paper_masonry_arch_25`. It was originally an
approximate DART-side scaffold with static endpoint supports and 23 dynamic
interior stones, not the author's Rigid-IPC geometry; this is superseded by
the author-faithful Rigid-IPC weighted-catenary port noted at the top of this
section, which makes all 25 stones dynamic and produces a 52-contact natural
manifold. The default
one-step exact-FBF probe now uses `max_contacts = 48` and
`max_contacts_per_pair = 2` with a 20000 outer-iteration arch budget; the
latest XML probe passes with `step_size_scale = 10`, residual
`9.9650461738801354e-07`, one exact solve, zero exact-FBF failures, zero
boxed-LCP fallback, and `2645` FBF iterations (`6693.7547709999999 ms`). This
is a reduced-contact Fig. 7 scaffold
only. The trace exporter can now emit
one-step exact-FBF and boxed-LCP crown-stone rows for
`masonry_arch_25_reduced_contact`, and the benchmark-only
`fbf_paper_arch_probe` records higher cap diagnostics. Current probe rows show
49/2 still produces only 48 actual contacts with this approximate geometry,
64/4 solves in `31118.9 ms`, 80/4 solves in `117628.4 ms`, and 100/4 did not
produce a row under a 120 s timeout. Full author geometry, pinned/projectile
physical outcome, a bounded 100-contact row, long-run traces, snapshots, and
timing remain open. The optional `dynamic_bodies` trace scope emits 23
mobile-stone rows at each sample for the reduced scaffold.

The same approximate 25-stone arch now has a reduced crown-projectile scaffold.
`MasonryArch25ProjectileScaffoldRuns` launches one reduced sphere toward the
crown after a bounded settle step and passes in about `10611 ms` with finite
state, one projectile, residual at tolerance, zero exact-FBF failures, and zero
boxed-LCP fallback. The new trace scenario
`masonry_arch_25_projectile_reduced_contact` emits tracked projectile rows and
optional `dynamic_bodies` rows; the exact-FBF one-step trace ends with 48
contacts, one exact solve, zero fallbacks, residual
`9.9901843912475669e-07`, and `success`. The benchmark rows
`masonry_arch_25_projectile_reduced_contact_boxed_lcp` and
`masonry_arch_25_projectile_reduced_contact_exact_fbf` are registered; the
focused exact-FBF row reports `4921 ms`, 48 contacts, one exact solve, zero
failures/fallbacks, max residual `999.018n`, and `2205` FBF iterations. The
GUI scene exposes a `Launch projectile` button and `p` key action. This is a
reduced GUI/test/trace scaffold only; it does not prove the paper's arch
post-impact physical outcome.

The first 101-stone masonry-arch scaffold now exists as
`MasonryArch101SceneBuilds` and `MasonryArch101OneStepReducedContactProbe`,
plus benchmark rows, trace rows, and GUI scene
`fbf_paper_masonry_arch_101`. It originally used the same approximate
DART-side arch construction with static endpoint supports; this is superseded
by the same author-faithful Rigid-IPC geometry port, which produces a
204-contact natural manifold. The default one-step exact-FBF probe now uses
`max_contacts = 38` and
`max_contacts_per_pair = 2` with a 20000 outer-iteration arch budget; the
latest XML probe passes with `step_size_scale = 10`, residual
`9.9001648754006507e-07`, one exact solve, zero exact-FBF failures, zero
boxed-LCP fallback, and `3912` FBF iterations (`2458.3270210000001 ms`). This
is a reduced-contact Fig. 8 scaffold only. The new
`fbf_paper_arch_probe` invalidates the older 39/2 fallback-boundary note:
39/2, 40/2, 48/2, 56/2, 64/2, 80/2, and 100/2 all solve without fallback on
the current approximate one-step scaffold; the 100/2 row takes `56137.1 ms`
and ends at residual `9.9444848815471616e-07`. Full author geometry, long-run
balance outcome, full-contact physical parity, long-run traces, snapshots, and
timing remain open. The
optional `dynamic_bodies` trace scope emits 99 mobile-stone rows at each sample
for the reduced scaffold.
The `residual_history` trace scope now retains up to 22000 samples and emits
2646 data rows for the reduced 25-stone 48-contact exact group and 3913 data
rows for the reduced 101-stone 38-contact exact group, ending at residuals
`9.9650461738801354e-07` and `9.9001648754006507e-07` respectively. These are
one-step reduced-scaffold convergence artifacts, not full paper plots.
GUI work for this dev task is not limited to registering scenes. If the
paper-parity examples need richer OSG rendering, reusable ImGui widgets,
camera/snapshot capture, overlays, or inspection controls to be self-contained
and visually verifiable, those viewer improvements are in scope for the FBF
task.
The FBF scenes also carry catalog-level Scene-tab documentation metadata, and
the shared `dart-demos` host renders that metadata before the scene-specific
controls. `pixi run demos -- --verify-fbf-scene-docs` can fail fast if any
`fbf_paper_*` scene loses its overview, expected-result, or coverage text, and
the same text is visible in the GUI rather than living only in a verifier.
The current one-step GUI capture smoke is recorded in
[gui-capture-report.md](gui-capture-report.md). It shows all nine
`fbf_paper_*` scenes with the `Scene` tab visible and passing non-blank image
verdicts, but it is not paper snapshot parity.
For scene states exposed through a key action, the demo host now supports
`--headless-action <key>` and Pixi exposes `capture-action`. The current
26-card phase-scaffold GUI evidence uses
`pixi run capture-action fbf_paper_card_house_26 p
docs/dev_tasks/fbf_exact_coulomb_friction/assets/gui_captures/fbf_paper_card_house_26_projectiles.png
1280 720 0` followed by `pixi run image-verdict` on that PNG. The saved
projectile capture shows the four launched spheres beside the reduced
card-house scaffold with the Scene-tab explanation visible, and the verdict
JSON passes the default non-blank gate. This is still a reduced GUI smoke, not
paper-matched Fig. 6 snapshot parity.
The current 25-stone arch projectile GUI evidence uses
`pixi run capture-action fbf_paper_masonry_arch_25 p
docs/dev_tasks/fbf_exact_coulomb_friction/assets/fbf_paper_masonry_arch_25_projectile.png
1280 720 0` followed by `pixi run image-verdict` on that PNG. The saved
capture shows the reduced projectile at the crown, visible Scene-tab
overview/expected/coverage text, and the shared diagnostics widget correctly
reports `not run yet` before the first exact solve. This is still a reduced
GUI smoke, not paper-matched Fig. 7 snapshot parity.
This still keeps the default DART 6 solver unchanged and retains the dense
snapshot for key prototype paths; it is not yet full paper fixture parity,
contact-manifold warm-started, fully matrix-free at the DART integration
layer, or optimized.

## DART 7 Candidate Ports

Use `origin/main` as the comparison source, not a machine-local path.

Useful candidates:

- `dart/math/lcp/*`: solver interface, problem classification, validation, and
  Dantzig/PGS solver modernization. This is useful infrastructure, but it is
  still boxed-LCP oriented and does not implement FBF.
- `dart/simulation/detail/rigid_contact/boxed_lcp_contact.*`: clean DART 7
  rigid contact assembly with an explicit Delassus snapshot and `J` matrix.
  The assembly pattern is useful evidence for a DART 6 FBF contact-problem
  builder, though DART 6 lacks the same ECS `World` pipeline.
- `dart/simulation/world_options.hpp`: `ContactSolverMethod` demonstrates a
  minimal public option shape in DART 7. DART 6 should not copy this wholesale,
  but it is useful when deciding eventual opt-in exposure.
- `dart/common/memory_manager` and solver scratch patterns: useful for the
  no-allocation and warm-start requirements once the FBF loop exists.
- Native collision additions on `main` that are still missing or incomplete on
  `release-6.20`: collision world, AABB-tree/spatial-hash/sweep broadphases,
  persistent manifold cache, CCD/raycast support, and denser SDF fields. These
  matter for paper-parity contact-rich scenes.

Non-candidates for DART 6.20:

- CUDA-specific kernels and GPU-only execution paths. The user explicitly
  excluded CUDA for this release-lane work.
- DART 7 ECS `World` plumbing as a wholesale backport. The release branch
  compatibility surface is different, and gz-physics integration still depends
  on the legacy `ConstraintSolver` path.
- Rigid-IPC friction as a direct substitute. It is a smoothed/lagged
  barrier-friction formulation, whereas FBF targets the exact reduced law.
  Its scratch, stats, and friction-iteration patterns are still useful.

## Integration Strategy

Proposed staged path:

1. Keep the default DART 6 solver unchanged. Add FBF as an opt-in solver path
   until paper-parity and downstream gates are proved.
2. Build an internal `ExactCoulombContactProblem` representation using
   normal-first per-contact triples, circular cone coefficients, deterministic
   contact frames, free contact velocities, and matrix-free `W` products.
   The math-detail problem prototype exists; DART 6 `World` assembly still
   needs to feed it from real contacts.
3. Implement and unit-test shared cone math, dual cone projection, scaled
   residual computation, and step-size adaptation independently of `World`.
   Cone math, residuals, matrix-free problem evaluation, and the FBF outer-loop
   shell now exist.
4. Implement a CPU/double-precision FBF reference solver first. It should be
   correct and inspectable before performance work starts. Matrix-free
   projected-gradient and block Gauss-Seidel inner solves now exist for unit
   problems and the opt-in DART 6 route; they still need broader paper-scene
   validation and performance tuning.
5. Add a DART 6 integration path that can solve all-contact groups through FBF
   and route unsupported/mixed groups through the existing boxed-LCP solver
   until joint/multibody coupling policy is decided. A dense internal adapter
   can now build and apply exact-Coulomb contact rows from DART 6 constraints,
   and an opt-in solver subclass can route supported groups through FBF without
   changing defaults. The first capped single-contact `World` smoke is now
   covered, the incline, backspin, turntable, and Painleve-proxy fixtures are
   covered at headless regression level, and a conservative
   same-constraint-sequence warm start now exists. The remaining integration
   work is broader multi-contact and mixed-group `World` coverage, true
   contact-manifold identity, mixed-group policy hardening, and matrix-free
   performance.
6. Recreate the paper fixtures and run them against FBF, the existing DART 6
   solver, and paper baselines where available.
7. Optimize only after correctness gates exist: matrix-free `W`, warm starts,
   contact graph/coloring, scratch reuse, and native-collision manifold
   stability.

The most likely DART 6 API shape is an opt-in solver object or option that does
not change the default `World` behavior. Avoid adding virtual methods to
gz-visible classes (`ConstraintSolver`, `BoxedLcpConstraintSolver`,
`ContactSurfaceHandler`) unless maintainers explicitly approve the ABI/API
impact.

## Current Blockers And Risks

- The paper's code repository is not anonymously readable yet. Exact parameter
  files, scene construction details, and any non-paper implementation choices
  are unavailable.
- The paper benchmarks use NVIDIA Warp/Newton, Kamino, and MuJoCo. DART 6.20
  does not ship these as in-tree comparison backends. External comparison
  dependencies such as Kamino, MuJoCo, or the paper implementation are allowed
  for tests, examples, and benchmarks when available, but they must remain out
  of the core library dependency surface.
- Initial public/source credit now exists in the exact-FBF implementation
  comments and in `docs/readthedocs/community/research_papers.rst`. Keep this
  citation current if the authors publish canonical BibTeX or additional scene
  assets.
- DART 6.20 has partial native collision support, but not all DART 7 native
  collision infrastructure needed for robust contact-rich paper fixtures.
- Exact paper performance parity cannot be claimed from focused unit tests.
  Completion requires the full fixture suite, residual checks, physical outcome
  checks, and timing reports.

## Verification Expectations

Near-term gates for this tracker:

- `pixi run cmake --build build/default/cpp/Release --target UNIT_math_CoulombCone --parallel 8`
- `pixi run cmake --build build/default/cpp/Release --target UNIT_math_ExactCoulombContactProblem --parallel 8`
- `pixi run cmake --build build/default/cpp/Release --target UNIT_math_ExactCoulombFbfSolver --parallel 8`
- `pixi run cmake --build build/default/cpp/Release --target test_ExactCoulombConstraintAdapter --parallel 8`
- `pixi run cmake --build build/default/cpp/Release --target test_ExactCoulombFbfConstraintSolver --parallel 8`
- `pixi run cmake --build build/default/cpp/Release --target test_ConstraintSolver --parallel 8`
- `pixi run cmake --build build/default/cpp/Release --target test_ExactCoulombFbfPaperFixtures --parallel 8`
- `pixi run cmake --build build/default/cpp/Release --target BM_INTEGRATION_exact_coulomb_fbf_paper --parallel 8`
- `pixi run cmake --build build/default/cpp/Release --target fbf_paper_trace --parallel 8`
- `pixi run cmake --build build/default/cpp/Release --target fbf_paper_gamma_sweep --parallel 8`
- `pixi run cmake --build build/default/cpp/Release --target fbf_paper_card_house_probe --parallel 8`
- `ctest --test-dir build/default/cpp/Release -R UNIT_math_CoulombCone --output-on-failure`
- `ctest --test-dir build/default/cpp/Release -R UNIT_math_ExactCoulombContactProblem --output-on-failure`
- `ctest --test-dir build/default/cpp/Release -R UNIT_math_ExactCoulombFbfSolver --output-on-failure`
- `ctest --test-dir build/default/cpp/Release -R '(test_ExactCoulombConstraintAdapter|test_ExactCoulombFbfConstraintSolver|test_ConstraintSolver|test_ExactCoulombFbfPaperFixtures)' --output-on-failure`
- `./build/default/cpp/Release/bin/BM_INTEGRATION_exact_coulomb_fbf_paper --benchmark_list_tests`
- `./build/default/cpp/Release/bin/BM_INTEGRATION_exact_coulomb_fbf_paper --benchmark_filter='BM_PaperFixtureStepTime/(backspin_boxed_lcp|backspin_exact_fbf)$' --benchmark_min_time=0.001s --benchmark_repetitions=1`
- `./build/default/cpp/Release/tests/integration/test_ExactCoulombFbfPaperFixtures --gtest_filter='ExactCoulombFbfPaperFixtures.MasonryArch25*' --gtest_output=xml:/tmp/fbf_arch25.xml`
- `./build/default/cpp/Release/tests/integration/test_ExactCoulombFbfPaperFixtures --gtest_filter='ExactCoulombFbfPaperFixtures.MasonryArch101*' --gtest_output=xml:/tmp/fbf_arch101.xml`
- `./build/default/cpp/Release/tests/integration/test_ExactCoulombFbfPaperFixtures --gtest_filter='ExactCoulombFbfPaperFixtures.CardHouseFourLevelSettleProjectileScaffoldRuns'`
- `./build/default/cpp/Release/tests/integration/test_ExactCoulombFbfPaperFixtures --gtest_filter='ExactCoulombFbfPaperFixtures.MasonryArch25ProjectileScaffoldRuns'`
- `./build/default/cpp/Release/bin/BM_INTEGRATION_exact_coulomb_fbf_paper --benchmark_filter='BM_PaperFixtureStepTime/card_house_26_settle_projectile_reduced_contact_(boxed_lcp|exact_fbf)$' --benchmark_min_time=0.001s --benchmark_repetitions=1`
- `./build/default/cpp/Release/bin/BM_INTEGRATION_exact_coulomb_fbf_paper --benchmark_filter='BM_PaperFixtureStepTime/masonry_arch_25_projectile_reduced_contact_(boxed_lcp|exact_fbf)$' --benchmark_min_time=0.01s --benchmark_repetitions=1`
- `pixi run capture-action fbf_paper_card_house_26 p docs/dev_tasks/fbf_exact_coulomb_friction/assets/gui_captures/fbf_paper_card_house_26_projectiles.png 1280 720 0`
- `pixi run image-verdict docs/dev_tasks/fbf_exact_coulomb_friction/assets/gui_captures/fbf_paper_card_house_26_projectiles.png`
- `pixi run capture-action fbf_paper_masonry_arch_25 p docs/dev_tasks/fbf_exact_coulomb_friction/assets/fbf_paper_masonry_arch_25_projectile.png 1280 720 0`
- `pixi run image-verdict docs/dev_tasks/fbf_exact_coulomb_friction/assets/fbf_paper_masonry_arch_25_projectile.png`
- `./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace backspin exact_fbf 120`
- `./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace backspin boxed_lcp 120`
- `./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace backspin exact_fbf 1 1 nan residual_history`
- `./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace card_house_26_reduced_contact exact_fbf 1`
- `./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace card_house_26_reduced_contact boxed_lcp 1`
- `./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace card_house_26_reduced_contact exact_fbf 1 1 nan dynamic_bodies`
- `./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace card_house_26_reduced_contact boxed_lcp 1 1 nan dynamic_bodies`
- `./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace card_house_26_reduced_contact exact_fbf 1 1 nan residual_history`
- `./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace card_house_26_settle_projectile_reduced_contact exact_fbf 1 2 nan phase_summary`
- `./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace card_house_26_settle_projectile_reduced_contact boxed_lcp 1 2 nan phase_summary`
- `./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace masonry_arch_25_reduced_contact exact_fbf 1`
- `./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace masonry_arch_25_reduced_contact boxed_lcp 1`
- `./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace masonry_arch_25_reduced_contact exact_fbf 1 1 nan dynamic_bodies`
- `./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace masonry_arch_25_reduced_contact exact_fbf 1 1 nan residual_history`
- `./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace masonry_arch_25_projectile_reduced_contact exact_fbf 1 1 nan tracked`
- `./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace masonry_arch_25_projectile_reduced_contact boxed_lcp 1 1 nan tracked`
- `./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace masonry_arch_25_projectile_reduced_contact exact_fbf 1 1 nan dynamic_bodies`
- `./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace masonry_arch_101_reduced_contact exact_fbf 1`
- `./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace masonry_arch_101_reduced_contact boxed_lcp 1`
- `./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace masonry_arch_101_reduced_contact exact_fbf 1 1 nan dynamic_bodies`
- `./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace masonry_arch_101_reduced_contact exact_fbf 1 1 nan residual_history`
- `./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_gamma_sweep backspin nan,0.5,0.05,0.01 240 1 1`
- `./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_card_house_probe 56,58,59,60,64`
- `./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_card_house_probe 16 1 30000 120 32 200 nan 0 0 0.9 10 1.5 0 0.003 0.55 1 1`
- `pixi run demos -- --list-scenes`
- `pixi run demos -- --cycle-scenes --frames 1`
- `pixi run docs-build`
- `pixi run lint`
- `pixi run build`

Later implementation gates:

- DART 6 contact-assembly tests that feed real `ConstraintSolver` contact data
  into the exact-Coulomb problem without changing the default boxed-LCP path.
- Broader opt-in `World` smoke tests beyond the first capped single-contact
  box/ground case, including multi-contact contact sets and unsupported mixed
  groups that intentionally fall back to boxed LCP.
- True contact-manifold warm-start identity and scratch-reuse tests for the FBF
  path once it is connected to real contact sets.
- Keep `ExactCoulombFbfPaperFixtures.CardHouseAFramePrecursorStands` enabled
  at the paper residual tolerance, keep
  `ExactCoulombFbfPaperFixtures.CardHouseFourLevelSceneBuilds` covering the
  26-card scaffold, keep the reduced-contact
  `ExactCoulombFbfPaperFixtures.CardHouseFourLevelOneStepReducedContactProbe`
  and `CardHouseFourLevelSettleProjectileScaffoldRuns` green, then raise the
  one-step and phase probes toward the full contact budget before extending
  them to the 6.7 s no-creep settle, real projectile impacts, and 10 s
  post-impact outcome.
- Keep `ExactCoulombFbfPaperFixtures.CardHouseTenLevelSceneBuilds` covering the
  10-level construction scaffold while adding exact-FBF dynamic outcome,
  residual trace, timing, snapshots, and external comparisons.
- Keep the reduced-contact
  `ExactCoulombFbfPaperFixtures.MasonryArch25OneStepReducedContactProbe`
  green while replacing the approximate scaffold with author/Rigid-IPC
  geometry, raising the cap beyond the current 48/2 default and 80/4
  near-timeout probe toward a bounded 100-contact row, and adding projectile
  outcome, trace, snapshot, and timing evidence. A 49/2 diagnostic still
  produced only 48 actual contacts with the current approximate scaffold, so
  geometry/contact generation is still part of the next step.
- Keep the reduced-contact
  `ExactCoulombFbfPaperFixtures.MasonryArch101OneStepReducedContactProbe`
  green while replacing the approximate scaffold with author/Rigid-IPC
  geometry, turning the new 100/2 one-step cap evidence into long-run balance,
  trace, snapshot, and timing evidence. The older 39/2 fallback-boundary note
  is stale on the current code.
- Expand the incline, backspin, turntable, and Painleve fixtures into the full
  displacement/trajectory sweeps, snapshots, residual traces, and timing
  evidence from the paper, replace or corroborate the Painleve proxy with the
  authors' scene when available, and add full-parity headless scene tests for
  house of cards, 25-stone arch, 101-stone arch, and 10-level house of cards.
- Benchmark comparison against existing DART 6 boxed LCP and, if available,
  the paper code, MuJoCo, and Kamino.
- Keep optional parity/comparison dependencies scoped to tests, examples, or
  benchmarks; do not add them as core DART library dependencies.
- Gazebo/gz-physics gate for any default solver, collision, public header, or
  package-surface change.
- Keep the source comments and DART 6 RTD paper citation page current as the
  experimental exact-FBF route moves toward public exposure.
