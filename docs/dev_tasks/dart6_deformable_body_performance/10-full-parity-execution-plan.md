# DART 6 deformable parity — execution plan

> **Scope change, 2026-07-29.** The Kim/Pollard lane has been removed from
> DART 6 and retargeted to DART 7; see the decision record in `decisions.md`.
> The two papers need different discretizations, and DART 6 should carry one
> deformable model rather than two parallel subsystems. Everything below that
> concerns the volumetric FEM backend (PR-2, M2.x) is therefore **not** DART 6
> work, and whether a reduced FEM is even the right DART 7 target is open given
> newer solvers such as AVBD. The Jain/Liu lane (PR-3) remains DART 6 work and
> is unaffected.

Status: **active for the Jain/Liu lane only.** Part A below is the DART 6 plan.
Part B is retired: the Kim/Pollard volumetric-FEM lane was removed from DART 6 on
2026-07-29 and retargeted to DART 7, and its sections are kept as DART 7
reference material, **not** as instructions. Nothing in Part B is DART 6 work.

Next DART 6 build step: **PR-3a soft-foot SIMBICON**
(`12-pr3a-soft-foot-simbicon.md`). Still-open decision the plan does not close on
its own: the competitive-envelope definition (§8).

# Part A — active DART 6 plan (Jain/Liu lane)

## 1. North star & success definition

DART 6 reproduces every **Jain/Liu 2011** demo/example, each
with (a) a runnable DART scene, (b) deterministic correctness/stability evidence,
(c) CPU performance matching the paper's real-time/near-real-time character on a
normalized target, and (d) `dart`-versus-FCL parity for collision-dependent rows —
while adding **zero overhead to pure rigid-body simulation** and preserving
`release-6.20` public API/ABI (new behavior is opt-in, layout-neutral).

Done = every **Jain/Liu** row of `02-paper-parity-matrix.md` meets the matrix's
acceptance rule with at least two clean independent review passes and a durable
demo artifact, and none relies on a deferral. The Kim/Pollard rows in that matrix
are **not** part of this completion rule: they are retargeted to DART 7 and
marked accordingly in the matrix itself.

## 2. Constraint: additive and ABI-safe on release-6.20

DART 6 work stays additive and ABI-safe: new behavior is opt-in, and existing
public class layouts, vtables, and default semantics are unchanged. The Jain/Liu
lane fits that comfortably, because it builds controllers, models, and scenes on
the point-mass `SoftBodyNode` that already ships rather than adding a new
dynamics subsystem.

The question of whether a *second* deformable architecture could live here was
settled on 2026-07-29: it cannot. The volumetric FEM lane was removed from DART 6
and retargeted to DART 7 (`decisions.md`). The two papers need different
discretizations, and a compatibility release branch should carry one deformable
model. That is why the FEM subsystem needed uninstalled headers, absence from the
generated aggregate, and a Doxygen exclusion just to exist here — symptoms of
being in the wrong place.

## 3. Performance-parity semantics (important nuance)

`deformable_body_paper_targets.md` states the paper CPU seconds were measured on
2011 hardware (2.8 GHz Core 2 Quad) and "are therefore model and hardware
reference points, not portable wall-clock thresholds for DART." So performance
parity is defined as, per row:

- reproduce the paper's **real-time / near-real-time character** (compute-per-
  simulated-second ratio ≤ the paper's ratio, on a normalized modern-CPU target
  captured on our own benchmark host), AND
- meet the matrix acceptance rule: 1-thread + host-capped multi-core rows,
  SIMD-off/on where a vectorizable kernel is touched, deterministic checksums,
  and "beat/tie eligible in-tree competing backends" where collision-dependent.

We record raw rows + revision SHA + host state (the existing
`bm-soft-body-paired` discipline), never a manual disposition over a
machine-readable FAIL.

## 4. Current state (milestone 1 = #3382)

Implemented and in review (see README work-packet table): adaptive contact
activation (WP-DB.05), soft face-interior contacts (WP-DB.08), DART-owned soft
collision, coupled-equation correctness (WP-DB.04), stability/CoP/LCP
gates (WP-DB.02), CPU cache/data-layout slices (WP-DB.06), representative
`soft_worm` + `adaptive_soft_contact` demos (WP-DB.09), and the
`ConstraintSolver` hot-path zero-overhead correction. These already satisfy the
Jain/Liu **adaptive-active-vertices**, **CoP/force-variance**, and
**LCP-robustness** rows.

## 5. Gap analysis (what Jain/Liu parity still needs)

### Jain/Liu (controllers + hand scenes missing)
| Row | Needs |
| --- | --- |
| Biped push recovery (soft vs rigid) | SIMBICON controller + soft-foot; push-threshold gate |
| Noisy-floor biped | seeded 5×5cm tile floor with 0–2cm offsets; rigid-vs-soft outcome |
| Biped walk | SIMBICON walk, LCP every 8 controller steps |
| Finger flick / arm fold / pinch grasp | hand/arm models + manipulation + adaptive-DOF/contact/LCP-time rows |
| Flexible-foot comparison | four-link hinge rigid foot vs simple-rigid vs deformable, same controller/seed |

## 6. PR structure

- **PR 1 — #3382** (milestone 1, in review): performance/compat slice + the
  three already-satisfied Jain/Liu rows. Merge as-is.
- **PR 2 — Kim/Pollard parity: RETIRED from DART 6.** The volumetric FEM backend
  was removed on 2026-07-29 and retargeted to DART 7 (`decisions.md`). Do not
  start it here.
- **PR 3 — Jain/Liu parity**: SIMBICON controller infra + soft-foot locomotion +
  hand scenes + flexible-foot comparison. Splittable (3a controller infra,
  3b locomotion, 3c hands, 3d flexible-foot).

PR-3 reuses the point-mass `SoftBodyNode` and the adaptive activation already
shipped, so the remaining work is controllers, models, and scenes rather than new
dynamics. It is the whole of the active DART 6 lane.

## 7. Jain/Liu architecture — controllers, models, scenes (grounded)

**Good news: PR-3 is largely assembly of existing parts, not new dynamics.** The
shipped point-mass SoftBodyNode + adaptive activation (WP-DB.05) is the Jain/Liu
deformable substrate. What exists to reuse:

- **A full SIMBICON controller** at `examples/demos/scenes/atlas_simbicon/`
  (`Controller`, `StateMachine`, `State`, `TerminalCondition`): torque PD + COM
  feedback (`State::computeControlForce`), standing/walk-in-place/walk/run state
  machines, harness helpers, push perturbation via `addExtForce`. Crucially its
  `BodyContactCondition` already uses `BodyNode::isColliding()`, which works for
  **rigid or SoftBodyNode** — so soft feet plug into the terminal logic with no
  controller rewrite. It is currently **rigid-only and hardcodes**
  `atlas_v3_no_head.sdf`.
- **A soft-feet Atlas asset** `atlas_v3_no_head_soft_feet.sdf` (SoftBodyNode feet,
  kv=50000/ke=100) — today only parsed+stepped in `test_SdfParser` with no
  ground/controller/perturbation.
- **Actuation**: joint `ActuatorType` (FORCE/PASSIVE/SERVO/…); the `SoftWorm`
  traveling-wave SERVO gait is the actuated-soft-character reference;
  `setForces`/`addExtForce`/`setCommand` all available. No muscle model (not
  needed — SERVO joints + SoftBodyNode kv/ke springs cover it).
- **Scene/test pattern** (copy verbatim): 3-part — GUI-free
  `scenes/XxxModel.{hpp,cpp}` (free functions: createWorld/step/checksum/isFinite)
  + thin `scenes/XxxScene.cpp` host adapter (`makeXxxScene()` → `DemoSceneSetup`,
  registered in `Registry.cpp` + `Scenes.hpp`) + GUI-free
  `tests/integration/test_XxxModel.cpp` (locomotion + finite-state +
  cross-run-determinism, compiled into the `test_ConstraintSolver` target with an
  `add_test --gtest_filter`). Headless PNG capture via `--headless --shot --steps`.

What must be **built/authored** for PR-3:
- soft-foot SIMBICON integration: point the existing controller at the soft-feet
  Atlas + a ground plane; rigid-vs-soft **push-recovery threshold** regression;
  contact-count time series; finite-state gate.
- **noisy-floor** scene: seeded 5×5 cm tiles with 0–2 cm random offsets;
  deterministic rigid-vs-soft outcome.
- **biped walk** with soft contact (LCP cadence per paper).
- **hand/arm models** (authoring required — no articulated-hand skeleton exists;
  `RHand.dof`/`fixedHand.dof` give a 30-DOF anatomical joint structure to build
  from) + finger-flick / arm-fold / pinch-grasp manipulation scenes with
  adaptive-DOF/contact/LCP-time rows.
- **four-link flexible-rigid foot** comparator asset (hinge-linked along the foot
  center line) for the same-controller/same-seed simple-rigid vs four-link-rigid
  vs deformable comparison — the one row not even on the old deferral list.

## 8. Cross-cutting

- **Zero rigid-body overhead**: audit every new subsystem for unconditional cost
  in `World::step`/`ConstraintSolver::solve`/`Skeleton::computeForwardDynamics`;
  keep all new work size-gated/opt-in. Regression-gate with a pure-rigid
  benchmark + the Gazebo plugin-boundary A/B (the pattern already used for the
  registry removal).
- **Multi-core scaling (WP-DB.07)**: still-open acceptance. The contact-heavy
  Jain/Liu scenes — noisy-floor locomotion and the hand/manipulation rows, which
  carry many simultaneous soft contacts — are the DART 6 place to demonstrate a
  real threads>1 speedup.
- **Competitive envelope**: `decisions.md` item 2 (in-tree backends + normalized
  paper metrics; external engines out of scope) still needs formal sign-off —
  needed to define "beat competing implementations". **Decision request.**
- **SIMD**: apply the `dart/simd/` contract to the vectorizable point-mass and
  soft-contact kernels; report SIMD-off/on.

## 9. Risks & open decisions

1. **Branch strategy — settled 2026-07-29.** The volumetric FEM lane does not
   belong on a compatibility release branch and was removed from DART 6; DART 6
   carries one deformable model. No longer an open risk here.
2. **FEM integration seam — no longer a DART 6 question.** What was learned about
   DART 6's per-step seam is retained in `11-fem-integration-seam.md` because it
   constrains any future per-step extension, deformable or not.
3. **Competitive-envelope definition**: needs sign-off (see §8).
4. **Model authoring**: the hand/arm assets and the four-link flexible foot must
   be created or sourced, with licensing and provenance confirmed. No tet meshes
   are needed; those belonged to the retired lane.
5. **Scope realism**: this is a multi-month, multi-PR research reproduction;
   milestones are independently shippable and gated so value lands incrementally.

## 10. Sequencing (DART 6)

1. Land the FEM removal (#3407).
2. **PR-3a soft-foot SIMBICON** (`12-pr3a-soft-foot-simbicon.md`): aim the
   existing GUI-free `atlas_simbicon` controller at
   `atlas_v3_no_head_soft_feet.sdf` plus a ground plane, and add the
   rigid-vs-soft push-recovery threshold, contact-count, and finite-state gates.
   The controller, the asset, and the scene/model-test pattern all already
   exist, so this is assembly and gating rather than new dynamics.
3. Remaining Jain/Liu rows: noisy-floor biped, soft-contact walk, then the
   hand/arm models that must be authored and their manipulation scenes, then the
   four-link flexible-foot comparison.
4. Confirm the competitive-envelope definition (§8) before the
   performance-acceptance stage.
5. Per-row acceptance plus at least two clean independent reviews and a durable
   demo artifact for each behavior-bearing row; promote durable facts to their
   owners; retire the task folder only when every remaining row has one.

The Kim/Pollard sequencing that used to appear here is retired with Part B.

# Part B — retired: Kim/Pollard lane (DART 7 reference only)

> Removed from DART 6 on 2026-07-29. The sections below record what was learned
> and are kept so a DART 7 effort does not start from zero. They are **not**
> instructions and nothing here is DART 6 work. On DART 7 the first question is
> whether a reduced FEM in this style is still the right target at all, given
> newer solvers such as AVBD. The implementation is preserved in the
> `wp-db-fem-foundation` and `wp-db-fem-elastic` branches and in #3404.

## B.1 Kim/Pollard architecture — FEM backend design (retired)

DART today has **only** the mass-spring `PointMass` model; there is no FEM,
material model, or reduced basis anywhere in `dart/`. The backend is greenfield.
Proposed additive design (no existing-layout/vtable change):

- **Geometry/state**: a new `dart::dynamics::Shape` subclass for the tet volume +
  embedded surface (runtime-string `getType()`, additive per-detector support),
  and a custom `Node`/`EmbeddedStateAndPropertiesAspect` on the owning BodyNode
  to hold FEM node positions/velocities, rest state, material, and the reduced
  modal basis. (Node/Aspect = fully additive, but has **no per-step update
  virtual** — it stores, it cannot self-integrate.)
- **Dynamics/integration seam (the hard part)**: because `Skeleton::
  integratePositions/Velocities` and `computeForwardDynamics` are **non-virtual**
  and the ABI-safe rule forbids adding a `Skeleton` FEM-registry or a
  `BodyNode::asFemBodyNode()` virtual, the FEM internal DOFs must be advanced by
  one of:
  1. a **custom `BodyNode` subclass** (`FemBodyNode` via
     `EmbedStateAndPropertiesOnTopOf<…, BodyNode>`) that overrides the existing
     `BodyNode` update virtuals (`updateBiasForce`/`updateAccelerationFD`/…) to
     fold FEM element forces into the articulated pass — same mechanism
     SoftBodyNode uses, minus the two ABI intrusions; and
  2. a **custom `constraint::ConstraintBase`** (registered via
     `addConstraint` → `mManualConstraints`, processed every `solve()`) to apply
     implicit FEM internal forces and FEM↔environment contact coupling in the
     LCP, and/or
  3. a **subclassed `ConstraintSolver`** (set via `World::setConstraintSolver`)
     that adds the global FEM integration pass — the only ABI-safe place for a
     "system-wide" per-step pass, since `World::step` has no generic hook.
  On DART 6 this seam question was answered before the lane was retired: the
  `ConstraintBase` hook worked, `ConstraintSolver::solve()` is not virtual, and
  the hook is silently skipped once deactivation rests the scene. Those findings
  are recorded in `11-fem-integration-seam.md`. A DART 7 effort would face a
  different engine and should not assume any of it carries over.
- **Reduced nonlinear FEM math**: corotational or StVK element forces on the tet
  mesh, modal/subspace reduction, and Kim/Pollard's selective diagonalization
  (SVD-based, paper reports 1.16×–3.60×). Correctness gated against analytic
  small-strain cases + energy behavior before scenes.
- **Skeleton coupling**: one-way (prescribed skeleton drives embedded FEM) →
  Fatman; two-way (FEM reaction affects skeleton + environment) → starfish/fish/
  worm, via the coupling constraint from seam #2.
- **Collision**: FEM surface point-triangle contact through the `dart` detector
  (extending the WP-DB.08 soft lanes) for the obstacle-escape row.

Zero-rigid-overhead: the FEM work is naturally size-gated (no FEM bodies ⇒ no
FEM constraints/nodes ⇒ zero iterations), mirroring how the soft loops cost
nothing when empty. Gate with the existing rigid benchmark + Gazebo
plugin-boundary A/B.

## B.2 Kim/Pollard gap ledger (retired; what was never built)
| Row | Needs |
| --- | --- |
| Fatman (one-way jiggle, 4,887/60 DOF) | Volumetric FEM body + embedded surface + one-way skeleton drive |
| Starfish (two-way jump-turn) | Two-way skeleton↔FEM↔env coupling |
| Fish (two-way actuated jump/landing) | + internal actuation + contact during landing |
| Worm (two-way actuated roll) at paper scale | + large-deformation-near-joints, ground contact |
| Obstacle-escape starfish | + contact-heavy point-triangle collision, profiler breakdown |
| CPU scaling / selective diagonalization | reduced/modal solve + SVD-based speedup, 1/multi-thread, SIMD |
