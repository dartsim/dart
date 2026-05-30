# PLAN-082 Sidecar: Contact, Friction & Inequality-Constraint Roadmap

Deferred, go/no-go-gated extension of the variational integrator (PLAN-082) to
contact and friction. The source paper (`lee-vi-2016`) handles only joint
equality constraints; closed loops are PLAN-082 Phase B2; everything here
(contact, friction, inequality constraints) is **out of the initial PLAN-082
commitment** and requires a separate go/no-go before any work starts. Durable
rationale lives in
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

## Hard Prerequisite (blocks every rung)

A root-finder needs contact/distance evaluated at the **trial** `qᵏ⁺¹` on each
inner RIQN iteration. Today `World::collide()` rebuilds the entire collision
world once per step (O(n²) narrow phase; no distance-at-configuration query).
**Phase C cannot start until a contact-query redesign** provides cheap distance
and gradient queries at arbitrary configurations (persistent broad phase,
incremental narrow phase). This is itself a sizeable workstream and the first
go/no-go gate.

## Rungs (recommended order)

| Rung   | What                                                                                                                                                                                                                                                                                                              | Property preserved                                                | Risk                                                                                                                                                                            |
| ------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **C1** | **Friction kernels.** Add the IPC lagged, smoothed friction _force law_ (`f₀/f₁/f₂` mollifier, `ε_v`, `μ`, normal-force coupling) next to the existing tangent stencils.                                                                                                                                          | Lagged ⇒ inner solve stays SPD/symmetric; dissipative by design.  | Low. DART already has the tangent _geometry_ (`tangent_stencil.hpp`); only the force law is missing.                                                                            |
| **C2** | **Compliant/penalty contact** as a smooth potential in the discrete Lagrangian (XPBD-`α̃` compliance or VBD quadratic penalty); gradient → `Fᵏ`. Use the existing CCD step bound (`continuous_collision_step.*`) as a RIQN line-search cap.                                                                        | Symplectic structure + O(n).                                      | Low–Med. Soft contact ⇒ small residual penetration; tune compliance. **Recommended first contact rung.**                                                                        |
| **C3** | **Augmented-Lagrangian bounded forces** (AVBD-style): per-contact dual `λ`, penalty ramp, `f = clamp(k·C+λ, fmin, fmax)` (normal `[−∞,0]`, friction `[−μ\|λn\|, μ\|λn\|]`) as an outer loop around RIQN, with cross-step warmstart.                                                                               | O(n); near-symplectic (AL → smooth penalty in the limit).         | Med. Hard, drift-free non-penetration + Coulomb cone **without a global PSD solve**. **Recommended target for rigid articulated contact.**                                      |
| **C4** | **Hard IPC log-barrier** (only if a guaranteed intersection-free articulated solver is required). Reuse DART's `barrier_kernel.hpp`, `primitive_distance.hpp`, tangent stencils, CCD; inject per-contact barrier curvature as local articulated stiffness into the ABI pass + mandatory CCD-filtered line search. | Symplectic (smooth potential) but step sizes shrink near contact. | **High / research.** No published precedent for an IPC barrier inside an O(n) recursive articulated quasi-Newton VI. Prototype on sparse contact and measure before committing. |

Equality constraints (loop closures) are **not** a contact rung — they are
PLAN-082 Phase B2 (a constraint Jacobian + impulse-based constraint solve), the
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

## Reusable vs. Adapter-Required (DART experimental IPC stack)

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
   step instead of rebuilding; **reuse** the experimental IPC active-set
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
