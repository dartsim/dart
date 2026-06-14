# PLAN-084 Sidecar: Contact, Friction & Inequality-Constraint Roadmap

Roadmap for extending the variational integrator (PLAN-084) to contact and
friction. The source paper (`lee-vi-2016`) handles only joint equality
constraints; closed loops are PLAN-084 Phase B2. The original Phase C work was
go/no-go gated; the current status is that C1-C3 landed for the scoped envelope
(ground points, lagged friction, augmented-Lagrangian centering, and a
sphere-sphere link slice), while arbitrary link geometry and the optional C4 hard
barrier remain separate follow-up work. Durable rationale lives in
[`../../design/simulation_variational_integrator.md`](../../design/simulation_variational_integrator.md);
this sidecar owns the sequencing and the comparative evidence.

## Central Constraint (why ordering matters)

Contact/friction can enter the variational integrator **only as a generalized
force / spatial impulse** in the forced DEL residual (`Fᵏ`, `Qᵢ`, `Fᵢ^{ext}`).
RIQN's quasi-Newton preconditioner `Δt·M⁻¹` represents articulated-inertia
curvature only — not stiff barrier Hessians, active sets, or inequality
multipliers. Two consequences fix the ordering:

- **Smooth potential ⇒ structure survives.** A smooth contact potential added to
  the discrete Lagrangian keeps the integrator a _forced variational integrator_
  (symplectic structure preserved; nonsmooth-/forced-VI theory: Fetecau–Marsden–
  Ortiz–West 2003; Kane–Marsden–Ortiz). Projection / velocity-level
  complementarity bolt-ons break it and inject numerical damping.
- **Bounded stiffness ⇒ O(n) survives.** Penalty / compliant / augmented-
  Lagrangian contact adds bounded curvature RIQN can absorb; a stiff log-barrier
  injects large local curvature `Δt·M⁻¹` mis-scales → iteration blow-up risk.

## Hard Prerequisite (remaining for arbitrary geometry)

A root-finder needs contact/distance evaluated at the **trial** `qᵏ⁺¹` on each
inner RIQN iteration. The implemented C1-C3 slice satisfies that requirement with
analytic ground and sphere-sphere hooks. For arbitrary link geometry,
`World::collide()` still rebuilds the entire collision world once per step and
does not provide a warm-started distance/gradient query at arbitrary
configurations. That geometry adapter is still a sizeable workstream and remains
the main prerequisite before broadening the envelope.

## Rungs (recommended order)

| Rung   | What                                                                                                                                                                                                                                                                                                              | Property preserved                                                | Risk                                                                                                                                                                            |
| ------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **C1** | **Friction kernels.** Add the IPC lagged, smoothed friction _force law_ (`f₀/f₁/f₂` mollifier, `ε_v`, `μ`, normal-force coupling) next to the existing tangent stencils.                                                                                                                                          | Lagged ⇒ inner solve stays SPD/symmetric; dissipative by design.  | Low. DART already has the tangent _geometry_ (`tangent_stencil.hpp`); only the force law is missing.                                                                            |
| **C2** | **Compliant/penalty contact** as a smooth potential in the discrete Lagrangian (XPBD-`α̃` compliance or VBD quadratic penalty); gradient → `Fᵏ`. Use the existing CCD step bound (`continuous_collision_step.*`) as a RIQN line-search cap.                                                                        | Symplectic structure + O(n).                                      | Low–Med. Soft contact ⇒ small residual penetration; tune compliance. **Recommended first contact rung.**                                                                        |
| **C3** | **Augmented-Lagrangian bounded forces** (AVBD-style): per-contact dual `λ`, penalty ramp, `f = clamp(k·C+λ, fmin, fmax)` (normal `[−∞,0]`, friction `[−μ\|λn\|, μ\|λn\|]`) as an outer loop around RIQN, with cross-step warmstart.                                                                               | O(n); near-symplectic (AL → smooth penalty in the limit).         | Med. Hard, drift-free non-penetration + Coulomb cone **without a global PSD solve**. **Recommended target for rigid articulated contact.**                                      |
| **C4** | **Hard IPC log-barrier** (only if a guaranteed intersection-free articulated solver is required). Reuse DART's `barrier_kernel.hpp`, `primitive_distance.hpp`, tangent stencils, CCD; inject per-contact barrier curvature as local articulated stiffness into the ABI pass + mandatory CCD-filtered line search. | Symplectic (smooth potential) but step sizes shrink near contact. | **High / research.** No published precedent for an IPC barrier inside an O(n) recursive articulated quasi-Newton VI. Prototype on sparse contact and measure before committing. |

Equality constraints (loop closures) are **not** a contact rung — they are
PLAN-084 Phase B2 (a constraint Jacobian + impulse-based constraint solve), the
natural precursor that proves the "constraints-as-impulses-in-RIQN" mechanism
before inequality contact builds on it.

## Comparative Evidence (IPC / VBD / AVBD / XPBD)

All pose a timestep as minimizing an incremental potential
`½‖x−x̂‖²_M + E(x)` (the same momentum/inertial form as a backward-Euler
variational step); they differ in how contact/friction enter `E` and how the
step is solved. Papers and reference repos reviewed:
`ipc-2020`, `vbd-2024`, `avbd-2025`, `macklin-xpbd-2016`
(`ipc-toolkit`, `avbd-demo2d` reference code).

| Axis                             | IPC                                                                          | VBD                          | AVBD                                      | XPBD                                               |
| -------------------------------- | ---------------------------------------------------------------------------- | ---------------------------- | ----------------------------------------- | -------------------------------------------------- |
| Contact model                    | C²-clamped log barrier                                                       | quadratic penalty            | bounded AL force                          | unilateral compliant/hard constraint               |
| Friction                         | lagged smoothed `f₁`, max-dissipation                                        | IPC model (lagged)           | Coulomb via `fmax=μ\|λn\|`                | position-level Coulomb projection                  |
| Inequality                       | interior-point barrier + CCD                                                 | `max(0,·)` penalty (soft)    | AL dual + force-bound clamp               | skip-when-inactive GS projection                   |
| Coordinates                      | maximal/vertex                                                               | vertex                       | vertex/per-body block                     | agnostic (incl. generalized)                       |
| Solver                           | global projected Newton                                                      | vertex block Gauss–Seidel    | block GS + AL primal/dual                 | per-constraint GS, `K≈M`                           |
| Intersection-free                | yes (barrier+CCD)                                                            | no                           | no                                        | no                                                 |
| Energy                           | dissipative (implicit)                                                       | uncond. stable, dissipative  | uncond. stable → implicit Euler           | dissipative, not symplectic                        |
| Fit to generalized-coord O(n) VI | barrier/CCD/stencils reusable; global PSD solve & maximal coords do not port | block idea redundant vs RIQN | **AL bounded-force: best structural fit** | **compliant constraint: best fit to RIQN's `K≈M`** |

Takeaways driving C1–C4: IPC's _lagged friction_ is the lowest-risk friction
model and DART already has its tangent geometry; **XPBD compliance** is the
closest match to RIQN's `Δt·M⁻¹` approximation; **AVBD's augmented-Lagrangian
bounded forces** are the cleanest way to hard, drift-free non-penetration +
Coulomb friction without a global solve; the **IPC hard barrier** is the only
intersection-free option but the highest risk inside an O(n) recursive VI.

## Reusable vs. Adapter-Required (DART 7 IPC stack)

Reusable geometry (coordinate-agnostic, in `detail/deformable_contact/`):
`barrier_kernel.hpp` (matches IPC's clamped log-barrier), `primitive_distance.hpp`,
`tangent_stencil.hpp` (the friction _geometry_), `continuous_collision_step.*`
(CCD step bound — highest-value reuse as a RIQN line-search cap).

Adapter-required / net-new: candidate generation (`candidate_set.hpp` is
mesh-vertex-specific; needs rigid/articulated link-geometry feeding); the
friction _force law_ (`f₀/f₁`, `μ`, `ε_v`, normal-force lagging — not present);
and the core glue: (world contact point) → (link, body-frame point) → `Sᵢᵀ`
generalized force, which maps a 12-DOF mesh-vertex contact quantity into the
articulated reduced-coordinate residual.

## Go/No-Go Gates

Before any Phase C work:

1. Contact-query-at-trial-configuration redesign is scoped with an owner.
2. A spike confirms RIQN robustness to added (bounded) contact curvature on a
   single-contact articulated scene (C2 prototype).

Before committing C4 (hard barrier): a measured prototype showing acceptable
RIQN iteration counts with barrier curvature injected into the ABI pass on
sparse contact; otherwise stop at C3.

## Recorded Decision (2026-05-28): NO-GO

Phase C (contact/friction) stays **deferred**; neither entry gate is met:

1. The contact-query-at-trial-configuration redesign is **not scoped** — today
   `World::collide()` rebuilds the whole collision world once per step and
   offers no distance/gradient query at an arbitrary trial `qᵏ⁺¹`, which every
   contact rung needs inside the RIQN loop.
2. No C2 single-contact robustness spike has been run.

Reinforcing evidence: the A2 scaling benchmark (`bm_variational_integration`)
showed the RIQN `Δt·M⁻¹` quasi-Newton's iteration count already degrades for
long/stiff systems (≥~64 links); a stiff contact barrier (C4) would compound
this, so even when Phase C opens it must start at the compliant/AL rungs
(C1–C3), not the barrier. Revisit when gate 1 has a scoped owner and the RIQN
convergence follow-ups (relative tolerance / acceleration) have landed.

## Contact-Query Redesign Scoping (Gate 1) — 2026-05-30

This section addresses **Go/No-Go gate 1** by scoping the
contact-query-at-trial-configuration workstream. It is the hard prerequisite
for every rung (C1–C4): RIQN needs the contact set evaluated at the _trial_
`qᵏ⁺¹` on each inner iteration, and `World::collide()` (a once-per-step full
rebuild, no distance-at-configuration query) cannot serve that loop.

### Required seam

A contact potential enters the forced DEL residual as a generalized force
`Q_c = −∂E_c/∂q = −barrier'(d(q)) · ∂d/∂q`. So the query must return, for each
active pair at a trial configuration, the signed distance `d(q)` **and** its
generalized gradient `∂d/∂q` (a row of the contact Jacobian). Internal seam,
not public surface:

```
ContactQuery built once per step from link geometry + a persistent broad phase;
  query.evaluate(trialLinkTransforms) -> [{pair, d, ∂d/∂q}]   // reused per RIQN iter
```

### Design (reuse-first)

1. **Persistent broad phase** — refit/advance an AABB sweep-and-prune across the
   step instead of rebuilding; **reuse** the DART 7 IPC active-set
   sweep-and-prune (`#2770`).
2. **Incremental narrow phase** — primitive distances at the trial config via
   the existing `detail/deformable_contact/primitive_distance.hpp`
   (point-triangle / edge-edge / point-plane); warm-started, since the active
   set changes slowly within a step.
3. **Generalized gradient** — `∂d/∂q = (∂d/∂x_world)·(∂x_world/∂q)`. The
   primitive-distance kernels already give `∂d/∂x` (closest-point normal); the
   contact-point Jacobian `∂x_world/∂q` comes from the shared spatial-algebra
   helpers (the link body Jacobian mapped to the contact point).
4. **CCD line-search cap** — **reuse** `continuous_collision_step.*` as a RIQN
   step-length bound near contact.

### Net-new (the adapter work)

- **Rigid/articulated candidate generation**: `candidate_set.hpp` is
  mesh-vertex-specific; feed it (or a sibling) rigid/articulated link geometry.
- **The reduced-coordinate glue**: (world contact point) → (link, body-frame
  point) → `Sᵢᵀ` generalized force — mapping a Cartesian contact quantity into
  the articulated residual.
- **The per-step warm-started query object** with the active-set bookkeeping.

### Sizing, owner, and the remaining gate-1 step

This is a mid-size collision workstream (comparable to a PLAN phase), not a
slice. It **must coordinate with PLAN-081 (deformable IPC)**, which owns the
collision/CCD/sweep stack the design reuses — that plan, or a jointly-owned
sub-plan, is the natural owner. Gate 1 is now **scoped** (this section); the
remaining gate-1 action is to assign that owner and open the workstream as its
own `PLAN-` entry. Gate 2 (the C2 single-contact robustness spike) is tracked
separately and stays open until run.

**Update (2026-05-30): gate 1 re-pathed by the rigid IPC contact solver
(`#2777`).** PLAN-082-rigid-implicit-barrier-contact landed a DART-owned
rigid-body contact stack — `detail/contact_jacobians.{hpp,cpp}` (rigid contact
Jacobians, the Cartesian half of the reduced-coordinate glue),
`detail/rigid_ipc_ccd.*` (curved-trajectory CCD), and the shared
`detail/deformable_contact/` geometry (`candidate_set.hpp`,
`continuous_collision_step.*`, `tangent_stencil.hpp`, `primitive_distance.hpp`).
That is the candidate-generation + distance/CCD infrastructure gate 1 was scoped
to build, so the remaining gate-1 work shrinks from "stand up a new
contact-query stack" to "**adapt that stack into a signed-distance + `∂d/∂q`
query at the trial `qᵏ⁺¹`**" for the VI's RIQN loop, then feed it the C1-C3 force
laws (now implemented). Natural owner: a jointly-owned sub-plan with
PLAN-082-rigid-implicit-barrier-contact / PLAN-081. With both the C1-C3 rungs and
that stack landed, this adapter is the single dominant remaining gap.

## C2 spike result (gate 2) — 2026-05-29

This section records **Go/No-Go gate 2**: a spike confirming RIQN robustness to
added (bounded) contact curvature on a single-contact articulated scene (a C2
compliant-contact prototype). It does **not** open Phase C — gate 1 (the
contact-query redesign) still has no scoped owner/PLAN entry — but it removes the
"no robustness spike has been run" half of the 2026-05-28 NO-GO and measures the
bounded-curvature envelope the rung ordering depends on.

### Method

An opt-in in-loop contact hook was added to `integrateMultibodyVariational`
(`compute/variational_integration.{hpp,cpp}`): an optional
`VariationalContactHook` that, given the **trial** `q^{k+1}` kinematics (per-link
world transforms + body Jacobians at the current iterate), returns a per-DOF
generalized contact force `Q_c`, folded into the forced-DEL residual on the same
forcing side as the applied/external force (`residual -= Δt·Q_c`). The hook is
re-evaluated **every RIQN iteration and every line-search trial** on the trial
configuration, so the contact force's curvature couples into the root-find. The
default empty hook is zero overhead and byte-for-byte identical to the
no-contact integrator (verified by a dedicated test; the existing 24 ctest
binaries stay green).

Scene (spike scope, hard-coded ground plane — no collision query, that is
gate 1): a vertical prismatic slider (a Euclidean coordinate, so the
**exact-Newton recursive-Jacobian** RIQN path — the one carrying the O(n)
articulated structure) of mass `m = 1 kg` dropped from `z₀ = 0.05 m` onto the
plane `z = 0` at `dt = 1 ms` for 3000 steps. The hook applies a smooth one-sided
linear-spring penalty `F = k(−z)` upward at the carriage origin when it
penetrates, mapped to generalized force by the trial-config point Jacobian
(`Jᵀ·F`). Contact stiffness is swept over mg-scaled values `k = K·mg`.

### Measured iteration counts and penetration vs. stiffness

| `K = k/(mg)` | `k` (N/m) | max RIQN iters | mean RIQN iters | rest penetration (m) | analytic `mg/k` (m) |
| ------------ | --------- | -------------- | --------------- | -------------------- | ------------------- |
| 1e2          | 981       | 4              | 2.54            | 2.09e-2              | 1.00e-2             |
| 1e3          | 9810      | 6              | 4.00            | 1.00e-3              | 1.00e-3             |
| 1e4          | 98100     | 11             | 8.08            | 1.00e-4              | 1.00e-4             |

(Budget = 100 iterations; every step converged with no non-convergence throw and
no NaN/Inf at all three stiffnesses.)

Two trends, both as the roadmap's "Central Constraint" predicts:

- **Penetration tracks `mg/k`** and decreases monotonically with stiffness —
  at `k ≥ 1e3·mg` it matches the analytic spring compression to the printed
  precision (the body rests near the plane; soft contact leaves a small,
  bounded, k-controllable residual penetration). At `K = 1e2` the discrete-time
  settle leaves it ~2× the static `mg/k`, still bounded and small.
- **Iteration count rises with stiffness but stays well inside budget** (mean
  2.5 → 4.0 → 8.1, max 4 → 6 → 11 as `k` grows 1e2 → 1e4 ×mg). RIQN absorbs the
  added curvature; the climb is the visible "strain" of the bounded-curvature
  envelope. The exact-Newton Jacobian models only articulated-inertia curvature
  (the contact force enters on the forcing side, re-evaluated per iterate, and
  the backtracking line search globalizes the missing contact curvature), so as
  `k` grows the line search works progressively harder — the expected precursor
  to the stiff-barrier (C4) blow-up, here still firmly in the tolerated regime.

### Conclusion: GO for the compliant/AL rungs (C1–C3); envelope confirmed

**Gate 2 = GO (for opening the compliant-contact rung once gate 1 is owned).**
The spike confirms RIQN tolerates a bounded contact potential's curvature folded
in-loop at the trial configuration on a single-contact articulated scene: the
root-find converges every step within budget with no NaN, the body rests near
the plane with `mg/k` penetration, and the iteration cost grows gracefully with
stiffness rather than blowing up. This is direct evidence for the roadmap's claim
that **compliant/AL contact (C1–C3) is the right entry** — bounded curvature is
RIQN-absorbable — while a stiff log-barrier (C4) is not: the monotone
iteration-vs-stiffness climb already visible here (×~3 over two stiffness
decades) is the leading edge of the curvature mis-scaling C4 would push past the
budget. Recommended envelope: stay at `k ≲ 1e4·mg` for the prototype contact
rung; revisit the barrier only with the dedicated C4 measured prototype the
"Before committing C4" gate already requires.

**This does not open Phase C.** Gate 1 (a scoped owner + `PLAN-` entry for the
contact-query-at-trial-configuration workstream) remains the blocking
prerequisite; the spike's hard-coded ground plane stands in for the real
distance/gradient query that workstream must deliver. The 2026-05-28 NO-GO's
gate-2 half is now satisfied; gate 1 still gates the rung.

## C1-C3 implemented (2026-05-30): compliant contact, lagged friction, AL

The compliant-contact rung (**C2**) now ships for the **link-point-vs-analytic-
ground** case, promoting the gate-2 spike's hard-coded `z = 0` plane into a real,
configurable distance/gradient query. `makeVariationalGroundContactHook`
(`compute/variational_integration.{hpp,cpp}`) builds a `VariationalContactHook`
from a `VariationalGroundContact` config — an analytic half-space
`{x : n·(x − p₀) ≥ 0}` plus a set of body-fixed `VariationalContactPoint`s. Each
RIQN iteration, at the trial `qᵏ⁺¹`, it evaluates every point's world position,
computes the signed distance `d = n·(p − p₀)`, and for penetrating points
(`d < 0`) folds the VBD/XPBD quadratic-penalty force `F = k(−d)·n` into the
forced-DEL residual via the reduced-coordinate glue `variationalContactPointForce`
(`J(p)ᵀF`). This realizes the seam's `evaluate(trial) → {d, ∂d/∂q}` for analytic
ground geometry and the (world contact point) → (link, body-frame point) → `Sᵢᵀ`
generalized-force mapping that gate 1 named as the net-new adapter work.

Verified (`VariationalGroundContact.*`, 4 tests):

- **Configurable-plane rest** — a slider dropped onto a plane at a non-zero
  offset settles at `offset − mg/k` (not the spike's `−mg/k`), confirming a real
  configurable query and the bounded `mg/k` residual penetration.
- **Rotational Jacobian / anti-tunneling** — a revolute chain's tip contact point
  is held near the plane where the contact-free swing dips far below, exercising
  the rotational point Jacobian `R(J_linear − [p]J_angular)` a slider cannot.
- **Inertness** — zero stiffness reproduces the no-contact trajectory; degenerate
  configs (zero normal / negative stiffness) throw at construction.

Bounded curvature stays inside the gate-2 `k ≲ 1e4·mg` envelope; the default-off
path is still byte-for-byte identical (the existing spike identity test).

**C1 lagged friction** ships alongside it: `VariationalGroundContact` gains
`frictionCoefficient` (Coulomb `μ`) and `frictionRegularization` (`ε_v`), and the
hook adds a regularized-Coulomb tangential force `−μ·|Fₙ|·v̂ₜ` opposing the
contact point's sliding. It is **lagged** — the friction direction (the contact
point's tangential velocity) and the normal magnitude are evaluated at `qᵏ` (the
step-start velocity mapped through the `qᵏ` point Jacobian), so the friction
force is _constant across the step's RIQN iterates_. That keeps it smooth for the
root-find (it converges like the frictionless case, dodging the tangential-
regularization stiffness a trial-config friction would inject near `vₜ = 0`) —
exactly the roadmap's "lagged ⇒ inner solve stays well-conditioned." Verified
(`LaggedFrictionDeceleratesSlidingBlock`): a block sliding on the plane
decelerates to rest under kinetic friction `~μ·mg`, while the frictionless block
keeps sliding at its initial speed.

**C3 augmented Lagrangian** ships as `VariationalGroundContactSolver`: a stateful
wrapper holding a per-contact dual `λ ≥ 0`, with the normal force
`max(0, λ + k(-d))` and the dual ascent `λ ← max(0, λ + k(-d))` warm-started
across steps. As the dual accumulates the steady contact load, the penetration
drives toward ~0 (drift-free) at finite stiffness — unlike the pure-penalty
`mg/k` residual. Two findings, both as the roadmap's "AL → implicit Euler"
(dissipative) note anticipates, were needed for clean settling on the
**undamped symplectic** VI: (1) a Kelvin-Voigt **normal damping** term
(`dampingCoefficient`, lagged like friction) — without dissipation the dual
feedback is marginally unstable (the constraint-feedback cubic `m·s³ + k·s +
β/Δt` lacks an `s²` damping term); and (2) updating the dual on an **outer-loop
cadence** (after the damped inner dynamics settle), not every step (which
overshoots before the body can move). Verified
(`AugmentedLagrangianCentersContactAtZeroPenetration`): the AL slider centers at
`d ≈ 0` with `λ ≈ mg`, where the pure penalty rests at `−mg/k`.

### Still future (the rest of Phase C)

- **Link-vs-link contact — sphere-sphere first slice landed; arbitrary geometry
  re-pathed onto the rigid IPC stack.** A first link-vs-link query ships:
  `makeVariationalLinkSphereContactHook` does compliant **sphere-sphere
  self-contact** between links (penalty + Kelvin-Voigt damping along the center
  line, an equal-and-opposite force mapped to both links' point Jacobians),
  verified by `SphereContactStopsSlidingLink` (a link sliding into a fixed base
  sphere is stopped, no pass-through). **Arbitrary link geometry** is the
  remaining gap, **re-pathed onto the rigid IPC contact solver** (`#2777`,
  [`082-rigid-implicit-barrier-contact.md`](../082-rigid-implicit-barrier-contact.md)).
  Concrete scope (from a 2026-05-30 stack audit):
  - **Reusable today** — `detail/deformable_contact/primitive_distance.hpp` has
    the analytic squared-distance + gradient (+ Hessian) kernels for all primitive
    pairs (`pointTriangle`/`pointEdge`/`edgeEdge`/`pointPoint`), coordinate-
    agnostic in world space; `detail/rigid_ipc_ccd.*` gives the curved-trajectory
    CCD usable as a RIQN line-search step bound. Distance/gradient is **separable**
    from the IPC log-barrier (`barrier_kernel.hpp`), so the VI can feed its own
    C1–C3 force law instead. Extracting signed `d` and `∂d/∂q` from the
    squared-distance kernels is a thin wrapper (`∂d = ∂(d²)/(2d)`), chained to
    generalized coordinates via the link point Jacobians already in
    `VariationalContactContext`.
  - **The dominant missing piece** — **rigid/articulated candidate generation**.
    `detail/deformable_contact/candidate_set.hpp` is mesh-vertex specific (broad
    phase over surface vertices/edges/triangles), not over articulated-link
    geometry, and there is no persistent warm-started "refit link AABBs at the
    trial config, re-check the active set" query object. That broad-phase adapter
    - a per-step query object is the net-new work.
  - **Estimate** — ~2–3 weeks of focused work (no fundamental blocker), which is
    why this stays a PLAN-scale workstream coordinated with
    PLAN-082-rigid-implicit-barrier-contact / PLAN-081, not an in-task slice. The
    hand-specified sphere-sphere hook is the gate-1 minimum already landed.

- **World-surface productionization + a contact GUI demo — landed.** C1/C2
  (compliant penalty + lagged friction, with Kelvin-Voigt damping) are now
  reachable from `World::step()`: a per-multibody `comps::VariationalContact`
  config component, `Multibody::setGroundContact(...)` /
  `addGroundContactPoint(link, localPoint)` handle methods (+ dartpy bindings),
  and `MultibodyVariationalIntegrationStage::execute` `try_get`s the component and
  folds `makeVariationalGroundContactHook(...)` into the step. Verified by a C++
  `world.step()` rest test, a Python smoke (`sx_variational_contact` — a VI
  pendulum tip caught by the ground plane), and the demos-cycle. The contact
  config now **persists across binary save/load**: `comps::VariationalContact` is
  a serialized `Property` component (registered in `io/serializer.cpp`,
  `kBinaryFormatVersion` bumped to 8; its link-index parallel array round-trips
  via a new generic POD-vector path in `auto_serialization.hpp`), verified by
  `ConfigRoundTripsThroughBinarySaveLoad`. The **stateful C3 AL rung is now
  wired through `World::step()`** too, opt-in via
  `setGroundContact(..., dualUpdateCadence)`: `0` keeps the robust C2 compliant
  default; `N>0` has the stage seed a `VariationalGroundContactSolver` from the
  persisted duals, fold its hook into the step, and advance the duals every `N`
  steps from the post-step transforms (the outer-loop cadence the undamped
  symplectic step needs for stability). The duals + cadence counter live in the
  serialized `comps::VariationalContactDualState`, so an AL scene resumes
  bit-identically across save/load — verified by
  `WorldSurfaceAugmentedLagrangianCentersContact` (drift-free centering on the
  `world.step()` path) and `AugmentedLagrangianDualStateRoundTrips`. The
  `VariationalGroundContactSolver` C++ API remains for direct cadence control.
