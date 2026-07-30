# DART 6 full two-paper deformable parity — execution plan (PROPOSAL)

Status: **accepted 2026-07-23**; first build step = **M2.0 FEM integration-seam
prototype** (§11 step 2). Requested by the 2026-07-23 directive: full replication
of both reference papers' demos/examples (correctness AND performance, no
compromise) + zero rigid-body overhead, as ABI-safe additive work on
`release-6.20`, delivered in a ~3-PR structure, with this research-and-propose
plan first. Still-open decisions the plan does not close on its own: the
competitive-envelope definition (§9/§10.3, needs maintainer sign-off) and the
branch-strategy checkpoint that M2.0 evidence will inform (§2/§10.1).

## 1. North star & success definition

DART 6 reproduces **every** Kim/Pollard 2011 and Jain/Liu 2011 demo/example, each
with (a) a runnable DART scene, (b) deterministic correctness/stability evidence,
(c) CPU performance matching the paper's real-time/near-real-time character on a
normalized target, and (d) native-vs-FCL parity for collision-dependent rows —
while adding **zero overhead to pure rigid-body simulation** and preserving
`release-6.20` public API/ABI (new behavior is opt-in, layout-neutral).

Done = every row of `02-paper-parity-matrix.md` meets the matrix's acceptance
rule with ≥2 clean independent review passes and a durable demo artifact; no row
relies on a deferral.

## 2. Constraint: ABI-safe additive on release-6.20 (+ risk flag)

Maintainer chose additive/ABI-safe on `release-6.20` (new opt-in types/APIs; no
change to existing public class layouts, vtables, or default semantics), over a
clean-break `main`/DART-7 line. Delivered as ~3 PRs (#3382 perf slice; PR-2
Kim/Pollard; PR-3 Jain/Liu), split further if a paper is too large for one PR.

**Risk to weigh (evidence-based, per the research-first directive):** a reduced
nonlinear volumetric-FEM backend + SIMBICON controllers are large new
subsystems; `docs/design/dart6_deformable_body.md` currently frames such work as
a clean-break line. The architecture below shows the additive path is *feasible*,
but it forces the FEM type to integrate through existing `BodyNode` virtuals / a
`ConstraintBase` / a subclassed solver rather than the natural (ABI-breaking)
`Skeleton`-registry pattern SoftBodyNode uses. If prototyping (M2.0) shows that
constraint materially blocks correctness or performance parity, revisit branch
strategy. This is the single biggest open decision.

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
activation (WP-DB.05), soft face-interior contacts (WP-DB.08), native soft
collision lanes, coupled-equation correctness (WP-DB.04), stability/CoP/LCP
gates (WP-DB.02), CPU cache/data-layout slices (WP-DB.06), representative
`soft_worm` + `adaptive_soft_contact` demos (WP-DB.09), and the
`ConstraintSolver` hot-path zero-overhead correction. These already satisfy the
Jain/Liu **adaptive-active-vertices**, **CoP/force-variance**, and
**LCP-robustness** rows.

## 5. Gap analysis (what full parity still needs)

### Kim/Pollard (entirely missing — no FEM backend)
| Row | Needs |
| --- | --- |
| Fatman (one-way jiggle, 4,887/60 DOF) | Volumetric FEM body + embedded surface + one-way skeleton drive |
| Starfish (two-way jump-turn) | Two-way skeleton↔FEM↔env coupling |
| Fish (two-way actuated jump/landing) | + internal actuation + contact during landing |
| Worm (two-way actuated roll) at paper scale | + large-deformation-near-joints, ground contact |
| Obstacle-escape starfish | + contact-heavy point-triangle collision, profiler breakdown |
| CPU scaling / selective diagonalization | reduced/modal solve + SVD-based speedup, 1/multi-thread, SIMD |

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
- **PR 2 — Kim/Pollard parity**: the volumetric FEM backend + 4 characters +
  obstacle-escape + FEM performance. Large — expect to split into sub-PRs
  (2a foundation/geometry, 2b FEM dynamics, 2c coupling, 2d collision+demos,
  2e performance).
- **PR 3 — Jain/Liu parity**: SIMBICON controller infra + soft-foot locomotion +
  hand scenes + flexible-foot comparison. Splittable (3a controller infra,
  3b locomotion, 3c hands, 3d flexible-foot).

PR-3 has **lower architectural risk** (it reuses the existing point-mass
SoftBodyNode + adaptive activation already shipped; the new work is controllers,
models, and scenes). PR-2 carries the FEM-backend risk. Recommend **starting PR-3
foundations in parallel with PR-2's FEM prototype**, so controller/scene progress
is not blocked on the hardest subsystem — but sequence per your preference.

## 7. Kim/Pollard architecture — ABI-safe FEM backend design (grounded)

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
  **M2.0 must prototype and pick this seam before anything else** — it is the
  make-or-break ABI/feasibility question. Open sub-question to verify in M2.0:
  is `ConstraintSolver::solve()` (and the needed hooks) virtual/overridable, and
  can a custom solver integrate FEM DOFs while preserving rigid determinism?
- **Reduced nonlinear FEM math**: corotational or StVK element forces on the tet
  mesh, modal/subspace reduction, and Kim/Pollard's selective diagonalization
  (SVD-based, paper reports 1.16×–3.60×). Correctness gated against analytic
  small-strain cases + energy behavior before scenes.
- **Skeleton coupling**: one-way (prescribed skeleton drives embedded FEM) →
  Fatman; two-way (FEM reaction affects skeleton + environment) → starfish/fish/
  worm, via the coupling constraint from seam #2.
- **Collision**: FEM surface point-triangle contact through the native detector
  (extending the WP-DB.08 soft lanes) for the obstacle-escape row.

Zero-rigid-overhead: the FEM work is naturally size-gated (no FEM bodies ⇒ no
FEM constraints/nodes ⇒ zero iterations), mirroring how the soft loops cost
nothing when empty. Gate with the existing rigid benchmark + Gazebo
plugin-boundary A/B.

## 8. Jain/Liu architecture — controllers, models, scenes (grounded)

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

## 9. Cross-cutting

- **Zero rigid-body overhead**: audit every new subsystem for unconditional cost
  in `World::step`/`ConstraintSolver::solve`/`Skeleton::computeForwardDynamics`;
  keep all new work size-gated/opt-in. Regression-gate with a pure-rigid
  benchmark + the Gazebo plugin-boundary A/B (the pattern already used for the
  registry removal).
- **Multi-core scaling (WP-DB.07)**: still-open acceptance; the larger FEM/paper
  workloads are the natural place to demonstrate real threads>1 speedup.
- **Competitive envelope**: `decisions.md` item 2 (in-tree backends + normalized
  paper metrics; external engines out of scope) still needs formal sign-off —
  needed to define "beat competing implementations". **Decision request.**
- **SIMD**: apply the `dart/simd/` contract to the FEM element/modal kernels
  where vectorizable; report SIMD-off/on.

## 10. Risks & open decisions (for review)

1. **Branch strategy** (biggest): additive FEM on a release branch vs clean-break
   `main`. Proceeding additive per your call; M2.0 prototype is the evidence
   checkpoint to confirm or revisit.
2. **FEM integration seam** (custom BodyNode virtuals vs ConstraintBase vs
   subclassed solver): resolved by the M2.0 prototype.
3. **Competitive-envelope definition**: needs sign-off (see §9).
4. **Model authoring**: Fatman/starfish/fish tet meshes, biped/hand assets, and
   the four-link foot must be created or sourced; licensing/provenance to
   confirm.
5. **Scope realism**: this is a multi-month, multi-PR research reproduction;
   milestones are independently shippable and gated so value lands incrementally.

## 11. Proposed sequencing (high level)

1. Land #3382 (milestone 1).
2. **M2.0 FEM integration-seam prototype** (de-risk the ABI-safe path: pick among
   custom `FemBodyNode` virtuals / `ConstraintBase` / subclassed `ConstraintSolver`;
   confirm rigid determinism preserved) — hard gate before PR-2's full build.
3. In parallel (lower risk, reuses the **existing SIMBICON controller** + shipped
   soft substrate), **PR-3a soft-foot locomotion**: aim the existing controller
   at `atlas_v3_no_head_soft_feet.sdf` + ground; add the rigid-vs-soft
   push-recovery threshold + contact-count + finite-state gates. This is the
   fastest visible parity win and validates the scene/test pattern for the rest.
4. PR-2 build-out (geometry/tet+embedded-surface → FEM dynamics/reduced+selective-
   diagonalization → 1-way then 2-way coupling → surface collision → demos →
   perf), sub-PR by sub-PR, each independently gated and shippable.
5. PR-3 build-out (noisy floor → walk → author hand/arm models → manipulation
   scenes → four-link flexible-foot comparison).
6. Per-row acceptance + ≥2 reviews + durable demo per behavior-bearing row;
   promote durable facts to owners; retire the task folder only when every row
   has a durable owner.

**Fastest path to first visible parity progress after #3382: step 3** (soft-foot
SIMBICON), because the controller, the soft-feet asset, and the scene/test
pattern all already exist — it is assembly + gates, not new dynamics.
