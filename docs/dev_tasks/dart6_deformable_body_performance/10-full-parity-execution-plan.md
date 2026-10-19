# DART 6 deformable parity — execution plan

Status: **active for the Jain/Liu lane.** The Kim/Pollard volumetric-FEM lane
was removed from DART 6 on 2026-07-29 (`decisions.md`). Its design notes and
implementation remain in [PR #3404](https://github.com/dartsim/dart/pull/3404),
and what it taught about the per-step seam is in the
[design doc](../../design/dart6_deformable_body.md#per-step-extension-seam).

PR-3a soft-foot SIMBICON shipped 2026-08-01 (#3408, #3423): the biped
push-recovery row's contact claim reproduces and is gate-asserted, but its push
claim does not, and its motor-noise clause stays open in §5. Goal: fully complete this plan, bundling the remaining rows
into as few PRs as review quality allows. The maintainer's 2026-08-01 goal
targeted DART 6.20; the 2026-10-10 release closeout retargeted the unmet rows
to DART 6.21. The competitive-envelope definition is settled (the 2026-07-23
acceptance decision in `decisions.md`); what remains of it is the
performance-acceptance evidence it defines (§8).

## 1. North star & success definition

DART 6 reproduces every **Jain/Liu 2011** demo/example, each
with (a) a runnable DART scene, (b) deterministic correctness/stability evidence,
(c) CPU performance matching the paper's real-time/near-real-time character on a
normalized target, and (d) `dart`-versus-FCL parity for collision-dependent rows —
while adding **zero overhead to pure rigid-body simulation** and preserving
DART 6 public API/ABI (new behavior is opt-in, layout-neutral).

Done = every **Jain/Liu** row of `02-paper-parity-matrix.md` meets the matrix's
acceptance rule with at least two clean independent review passes and a durable
demo artifact, and none relies on a deferral. The Kim/Pollard rows in that matrix
are **not** part of this completion rule: they are out of DART 6 scope and
marked accordingly in the matrix itself.

## 2. Constraint: additive and ABI-safe on DART 6

DART 6 work stays additive and ABI-safe: new behavior is opt-in, and existing
public class layouts, vtables, and default semantics are unchanged. The Jain/Liu
lane fits that comfortably, because it builds controllers, models, and scenes on
the point-mass `SoftBodyNode` that already ships rather than adding a new
dynamics subsystem.

The question of whether a *second* deformable architecture could live here was
settled on 2026-07-29: it cannot. The volumetric FEM lane was removed from DART 6
(`decisions.md`). The two papers need different
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

## 4. Current state (PR-3a shipped; remaining rows in §5)

DART 6.20 contains adaptive contact activation, soft
face-interior contacts, coupled-equation correctness,
stability/CoP/LCP gates, CPU cache/data-layout slices,
representative `soft_worm` + `adaptive_soft_contact` demos, and the
`ConstraintSolver` hot-path zero-overhead correction. These satisfy the
Jain/Liu **adaptive-active-vertices**, **CoP/force-variance**, and
**LCP-robustness** rows.

No prerequisite integration or removal remains. PR-3a soft-foot SIMBICON
shipped through #3408/#3423 (`decisions.md` records the
resolved decisions), completing the contact portion of the **biped push
recovery** row: 51.2 soft vs 15.64 rigid contacts, gate-asserted. Its push
claim does not reproduce: the soft biped falls from the 6000 and 8000 N pushes
the rigid control recovers from, and the former 18000 N soft maximum was an
isolated recovery (#3431, #3692). The row stays open against the §1 Done rule
until the push claim and three more things have evidence: its motor-noise
clause (§5), and the matrix acceptance rule's `dart`-versus-FCL
and 1-thread/host-capped multi-thread rows — PR-3a supplies neither, since
its comparison deliberately pins FCL and one thread for validity (the two
foot representations only share a narrow phase under FCL, and single-thread
keeps runs bit-identical), so that evidence needs its own configuration.

## 5. Gap analysis (what Jain/Liu parity still needs)

### Jain/Liu (controllers + hand scenes missing)
| Row | Needs |
| --- | --- |
| Biped push recovery (soft vs rigid) | **Contact portion shipped** (#3408/#3423); still open: the push claim, which asset tuning alone does not restore (#3431), the motor-noise variant, plus the matrix acceptance rule's `dart`-versus-FCL and 1-thread/host-capped multi-thread evidence (§4 — PR-3a's comparison deliberately pins FCL and one thread) |
| Noisy-floor biped | seeded 5×5cm tile floor with 0–2cm offsets; rigid-vs-soft outcome |
| Biped walk | SIMBICON walk, LCP every 8 controller steps |
| Finger flick / arm fold / pinch grasp | hand/arm models + manipulation + adaptive-DOF/contact/LCP-time rows |
| Flexible-foot comparison | four-link hinge rigid foot vs simple-rigid vs deformable, same controller/seed |

## 6. PR structure

- **Landed foundation**: the performance/compatibility slice and the three
  already-satisfied Jain/Liu rows are part of DART 6.20.
- **Kim/Pollard parity: RETIRED from DART 6.** The volumetric FEM backend was
  removed on 2026-07-29 (`decisions.md`). Do not start it here.
- **Active Jain/Liu series**: PR-3a shipped; continue with locomotion (3b),
  hand scenes (3c), and the flexible-foot comparison (3d), bundled into as
  few PRs as review quality allows.

PR-3 reuses the point-mass `SoftBodyNode` and the adaptive activation already
shipped, so the remaining work is controllers, models, and scenes rather than new
dynamics. It is the whole of the active DART 6 lane.

## 7. Jain/Liu architecture — controllers, models, scenes (grounded)

**Good news: PR-3 is largely assembly of existing parts, not new dynamics.** The
shipped point-mass SoftBodyNode + adaptive activation is the Jain/Liu
deformable substrate. What exists to reuse:

- **A full SIMBICON controller** at `examples/demos/scenes/atlas_simbicon/`
  (`Controller`, `StateMachine`, `State`, `TerminalCondition`): torque PD + COM
  feedback (`State::computeControlForce`), standing/walk-in-place/walk/run state
  machines, harness helpers, push perturbation via `addExtForce`. Crucially its
  `BodyContactCondition` already uses `BodyNode::isColliding()`, which works for
  **rigid or SoftBodyNode** — so soft feet plug into the terminal logic with no
  controller rewrite. Since #3408/#3423 it drives either foot geometry through
  `soft_foot_simbicon`, and `State`'s COM feedback is point-mass aware.
- **A soft-feet Atlas asset** `atlas_v3_no_head_soft_feet.sdf` (SoftBodyNode
  feet, kv=50000, `<damp>` re-tuned 1000 -> 4000 by #3423) — exercised on ground
  under the SIMBICON controller with scheduled pushes by `soft_foot_simbicon`
  and its gates since #3408/#3423.
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
- ~~soft-foot SIMBICON integration~~ — **shipped** by #3408/#3423 as
  `soft_foot_simbicon` with the push-recovery threshold, contact-count, and
  finite-state gates. What remains from that row is the **motor-noise
  variant** of the same rigid-vs-soft comparison (deterministic seeded noise
  on the controller torques; soft sustains at least the rigid level).
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
- **Multi-core scaling**: still-open acceptance. The contact-heavy
  Jain/Liu scenes — noisy-floor locomotion and the hand/manipulation rows, which
  carry many simultaneous soft contacts — are the DART 6 place to demonstrate a
  real threads>1 speedup.
- **Competitive envelope**: settled. The 2026-07-23 acceptance decision in
  `decisions.md` (in-tree backends + normalized paper metrics; external engines
  out of scope) is the maintainer-confirmed definition of "beat competing
  implementations"; apply it, do not reopen it.
- **SIMD**: apply the `dart/simd/` contract to the vectorizable point-mass and
  soft-contact kernels; report SIMD-off/on.

## 9. Risks & open decisions

1. **Branch strategy — settled 2026-07-29.** The volumetric FEM lane does not
   belong on a compatibility release branch and was removed from DART 6; DART 6
   carries one deformable model. No longer an open risk here.
2. **FEM integration seam — no longer a DART 6 question.** What was learned about
   DART 6's per-step seam is retained in the design doc's
   [per-step extension seam](../../design/dart6_deformable_body.md#per-step-extension-seam)
   section because it constrains any future per-step extension, deformable or
   not.
3. **Competitive-envelope definition — settled 2026-07-23** (the acceptance
   decision in `decisions.md`; see §8). No longer an open decision.
4. **Model authoring**: the hand/arm assets and the four-link flexible foot must
   be created or sourced, with licensing and provenance confirmed. No tet meshes
   are needed; those belonged to the retired lane.
5. **Scope realism**: this is a multi-month, multi-PR research reproduction;
   milestones are independently shippable and gated so value lands incrementally.

## 10. Sequencing (DART 6)

1. **PR-3a soft-foot SIMBICON** — shipped 2026-08-01 via #3408/#3423
   (`decisions.md` records the resolved decisions); the
   motor-noise clause of its row moves to the remaining rows below.
2. Remaining Jain/Liu rows: noisy-floor biped, soft-contact walk, then the
   hand/arm models that must be authored and their manipulation scenes, then the
   four-link flexible-foot comparison.
3. Apply the approved competitive envelope (the 2026-07-23 acceptance
   decision in `decisions.md`) at the performance-acceptance stage.
4. Per-row acceptance plus at least two clean independent reviews and a durable
   demo artifact for each behavior-bearing row; promote durable facts to their
   owners; retire the task folder only when every remaining row has one.

The Kim/Pollard sequencing is retired with that lane.
