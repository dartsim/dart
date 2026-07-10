# WP-DB.05 adaptive contact activation design

Drafted 2026-07-09 by the orchestrator session; revised the same day after a
three-lens design review (physics/paper fidelity, ABI/compat,
determinism/performance) whose findings are folded in below. Implementation is
tracked against this revision. Owner packet: WP-DB.05 (README.md). Paper
source: Jain and Liu, "Controlling Physics-Based Characters Using Soft
Contacts" (SIGGRAPH Asia 2011) — adaptive active surface vertices near
contact.

## Problem

DART 6 `SoftBodyNode` integrates every point mass every step: all vertex and
edge springs contribute to `updateBiasForce`, every point contributes to
`updateArtInertia`, and all point states advance, regardless of whether the
soft surface region is anywhere near contact. Jain/Liu keep only vertices near
contact "simulated" (their tables report roughly 10-25% of total DOFs active)
and let the rest ride rigidly with the parent body. That is the remaining
untapped structural speedup for contact-sparse soft scenes, and it is the
paper feature the `adaptive_deformable` scene name was reserved for.

## Physical model: frozen points are rigid lumps

`SoftBodyNodeHelper` distributes the body's whole flesh mass across the point
masses, so a point's mass cannot simply be dropped from the dynamics when it
deactivates. A **frozen point is modeled as a point mass rigidly attached to
the parent body at its frozen local offset**:

- **Articulated inertia**: a frozen point contributes its full point inertia
  at its frozen local position — `_addPiToArtInertia(frozenLocalPos, mass)`
  and the implicit variant with the same full mass. (An active point
  contributes the articulated `Pi = m - m^2 Psi`, which is 0 in the explicit
  case; a rigid attachment transmits everything, so `Pi_frozen = m`.) Total
  system mass and inertia are conserved for any active/frozen split.
- **Bias force**: the frozen point's rigid-ride terms (gravity `-m g_local`
  and the angular-velocity coupling `omega x (m v_frozen)`) enter the parent
  body's bias force at the frozen offset, exactly as the rigid-body inertia
  integrals would if the flesh were welded.
- **Boundary springs (Newton's third law)**: an edge spring between active
  point A and frozen point B pulls A toward B as usual (A's `mAlpha` reads
  B's frozen state), and B's equal-and-opposite reaction is applied to the
  parent body's bias force at B's frozen offset. Internal forces therefore
  still cancel over the whole body; no spurious net skeleton force appears.
- **State**: a frozen point's local position is held (see linger decay
  below) and its local velocity reads zero; its world position tracks the
  parent transform. It renders and collides normally.

## Activation lifecycle

Per soft body with activation enabled:

- **Seeding** happens inside the constraint pipeline, captured directly when
  `SoftContactConstraint` selects contact point masses (not by re-scanning
  the deprecated `PointMass::isColliding()` lifecycle): the three vertices of
  every contact face on this soft body seed the set. There is **no
  world-position margin scan** — seeding is O(contacts), and the ring
  expansion below supplies the spatial margin.
- **Expansion**: seeds expand along `Properties::mConnectedPointMassIndices`
  by `ringCount` rings (default 2), iterating connections in stored index
  order. Cost O(active), allocation-free with retained visited stamps and a
  ring queue sized at preparation.
- **One-step lag (explicit semantics)**: DART's step runs forward dynamics
  before `ConstraintSolver::solve()`, so a set selected during the solve can
  only take effect in the NEXT step's dynamics phases. This lag is part of
  the contract. For the step in which a contact lands on a currently-frozen
  point, the frozen point is exactly a rigid point on the parent body, so
  the contact is handled as a **rigid contact on the parent BodyNode** for
  that step (the Jain/Liu ride-along response); the point activates for the
  following step. No constraint ever addresses a frozen point's flesh DOFs,
  and no stale-`Psi` LCP row can arise.
- **Hysteresis with rest gating**: a point leaves the active set only after
  (a) it has been outside the seed/ring set for `linger` steps (default 8),
  AND (b) its local velocity magnitude and its distance to the rest position
  `X0` are below deactivation tolerances. Freezing is therefore
  near-momentum-free (the discarded local momentum is bounded by the
  velocity tolerance; it is documented as an approximation of the same order
  as the paper's).
- **Linger decay**: during the linger window the point's dynamics are
  active, but an additional critical-damping-style decay drives it toward
  `X0`, so points freeze at (near) the undeformed surface. This removes
  stored boundary-spring energy (no reactivation snap) and keeps the
  collision shape of frozen regions near the rest surface instead of a
  permanently deformed snapshot.
- **Empty active set**: with no contacts and nothing lingering, the soft
  body steps as a rigid ride-along (all points frozen at rest shape) with
  zero flesh DOFs integrated.
- **Default**: activation disabled; behavior is bit-identical to current
  DART 6 all-active dynamics. Enabling is per soft body at runtime.

## Matrix/vector API semantics (WP-DB.04 interaction)

The public `Skeleton` matrix/vector APIs (`getMassMatrix`,
`getAugMassMatrix`, inverse variants, gravity/Coriolis/external vectors) and
the WP-DB.04 equation gates describe the **all-active model** and are
unchanged by activation in this packet: activation is a forward-dynamics
approximation toggle, not a reduced-coordinate model. This keeps the
soft-tree dense inverses nonsingular and the merged equation gates valid
(they run activation-off; one added assertion verifies the public matrices
are identical with activation on, since they depend on state, not activity).
Reduced matrix semantics for frozen DOFs are recorded as explicit follow-up,
not silently approximated.

## API, storage, and lifecycle (ABI rules)

- Public API: additive non-virtual `SoftBodyNode` methods only —
  `setAdaptiveContactActivationEnabled(bool)`,
  `isAdaptiveContactActivationEnabled()`, tuning setters/getters (ring
  count, linger steps, deactivation tolerances), and
  `getNumActivePointMasses()` for instrumentation. No data members added to
  any public class layout (`SoftBodyNode`, `PointMass`,
  `PointMass::State/Properties`, Aspect state/properties structs are frozen
  ABI).
- Retained state (enabled flag, tuning, active flags, linger counters,
  frozen local positions, visited stamps, ring queue) lives in the
  **node-owned `.cpp`-internal notifier-subclass storage** established by the
  WP-DB.06 SoA scratch slice. A raw-pointer side registry is explicitly
  rejected: it would need destructor-time erasure plus cross-World locking,
  and address reuse could leak stale activation onto new nodes. **The
  WP-DB.06 storage slice is an ordering prerequisite for this packet.**
- **Clone/copy**: `SoftBodyNode::clone()` and the copy paths transfer the
  enabled flag and tuning inside `SoftBodyNode.cpp` (both live in the
  node-owned storage), so clones keep their configuration. Transient
  bookkeeping (active flags, linger counters, frozen positions) resets to
  all-active-on-next-step in the clone; a determinism test covers
  clone-then-step equivalence from identical states.
- **State restore**: `setAspectState`/`Skeleton::setState`/`World::bake`
  replay restore point-mass states but not side bookkeeping; restoring state
  therefore **clears activation bookkeeping to all-active and reseeds on the
  next step**. A bake/replay determinism test is part of acceptance.
- SKEL parsing support (an attribute on `<soft_shape>`) is follow-up; when
  it lands it must round-trip through the same clone/copy transfer paths.

## Determinism preconditions

- Selection input is the contact set from the collision/constraint pipeline.
  This is thread-count-invariant today because `DARTCollisionDetector`
  merges parallel soft-soft worker results in indexed pair order and refuses
  to parallelize when the contact cap could short-circuit. That invariant is
  a named precondition of this packet; the collision lane must preserve it.
- Expansion iterates connections in stored index order; linger counters
  update in point-index order; no time, RNG, pointer-order, or thread-order
  dependence anywhere in selection.
- Gate: with activation enabled, headless checksums are bit-identical across
  `THREADS=1/4/16` and across reruns, including a bake/replay window.

## Performance scope (honest premise)

Activation gates the point-mass dynamics phases; on its own it cannot win
scenes dominated by collision-object refresh (the initial FCL profile
recorded `CollisionGroup update objects` at 83% inclusive; the native lane
reduced but did not eliminate that structure). The packet therefore pairs
activation with **collision-refresh gating that falls out of frozen
geometry**: frozen points do not change local positions after linger decay,
so the existing unchanged-position refit/cache-refresh skips (FCL soft mesh
replace path, native soft cache) see clean regions; an all-frozen soft body
skips flesh refresh entirely.

Acceptance rows are scoped accordingly:

1. Phase-level: `updateBiasForce`/`updateArtInertia`/integration profile
   rows drop roughly proportionally to the inactive fraction on the
   representative scenes.
2. Scene-level: activation-on beats all-active wall-clock on the
   dynamics-weighted native-lane scenes (e.g. `adaptive_deformable`,
   `soft_open_chain` variants with sparse contact); collision-dominated
   scenes are reported without a win claim if refresh gating does not carry
   them.
3. Fewer active DOFs: instrumented counts show a large inactive fraction
   (target >50% on the ellipsoid drop steady state; scene-dependent).

## Acceptance evidence (maps to README "Done when")

1. **Matching contact behavior**: on soft box drop, soft stack, and the
   `adaptive_deformable` ellipsoid drop, activation-on vs activation-off
   skeleton trajectories stay within documented tolerances over a 200-step
   window with identical checkpoint contact counts; free-surface jiggle away
   from contact may differ by design.
2. **Fewer active DOFs** per the instrumented counts above.
3. **Deterministic state hashes** per the determinism gate above.
4. **Performance** per the scoped rows above (smoke until the idle-host
   matrix rerun).
5. Focused `test_SoftDynamics` regressions: default-off bit-equivalence;
   on/off contact-behavior tolerance; rest-gated hysteresis (an oscillating
   vertex does not freeze; a quiet one freezes after `linger`);
   reactivation-on-approach (rigid first-contact step then activation);
   frozen-mass conservation (skeleton trajectory of an all-frozen soft body
   matches an equivalent rigid body); equation gates unchanged with
   activation on; allocation gates with activation enabled;
   clone-then-step and bake/replay determinism.

## Out of scope for this packet

- Changing the LCP/active-set solver itself (Jain/Liu's LCP-time observation
  is tracked by the paper-parity matrix, not this packet).
- Reduced matrix semantics for frozen DOFs (documented follow-up).
- Reduced-coordinate or FEM flesh models.
- Making activation the default.
