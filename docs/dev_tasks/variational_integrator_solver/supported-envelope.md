# VI Solver — Supported Envelope

The variational integrator (VI) is an **experimental** `World` integration family
(opt-in via `World::setMultibodyOptions({.integrationFamily = "variational
integrator"})`). "Experimental" means we do not yet make the API-stability
promise (see [`graduation-criteria.md`](graduation-criteria.md)), but the solver
is fully functional **within the envelope below**. This file states that envelope
explicitly so the bounds are a first-class, documented property rather than folk
knowledge — it is the concrete answer to the graduation criterion _"document the
supported DOF/stiffness bounds."_

The envelope is defined by **where the forced-DEL root-find converges within
budget**. Outside it the integrator does not silently degrade: a non-converged
step raises the defined non-convergence error (no NaN, no silent fallback), so
stepping past the envelope fails loudly.

## Inside the envelope (supported today)

| Regime                                                  | Mechanism                                                                                                                | Evidence                                                                                                                           | Bound                                                                                                                                                                                                                                |
| ------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------ | ---------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| **Euclidean chains** (prismatic / revolute, fixed base) | Exact recursive-Jacobian Newton preconditioner (O(n) non-symmetric ABA + backtracking line search)                       | `LongChainExactPreconditionerConvergesWithinBudget` (100 & 128 links); quadratic, **~3 iterations independent of length**          | Verified to **128 DOF**; expected to extend further (length-independent iteration count)                                                                                                                                             |
| **Floating base + spherical joints**                    | Manifold quasi-Newton (`Δt·M⁻¹`) + tangent-space Anderson acceleration (per-joint log-difference mixing, `jointRetract`) | `ManifoldAndersonAcceleratesSphericalChain` (20-link spherical: **200/200 steps, ≤9 iters**), `FloatingSphericalChainStaysCorrect` | **20-link spherical** chains; floating base + spherical retraction FD-verified                                                                                                                                                       |
| **Holonomic loop closures** (Point / Distance / Rigid)  | Newton projection onto the constraint manifold reusing the O(n) ABI                                                      | Long-horizon hold; constraint Jacobians FD-verified; `sx_loop_closure` demo holds to machine precision over 100 steps              | Well-conditioned closures (rank-deficient configs — e.g. POINT/RIGID on a planar arm — are excluded; use DISTANCE)                                                                                                                   |
| **Compliant contact** (opt-in spike)                    | Smooth contact potential in the forced-DEL residual, evaluated at the trial `qᵏ⁺¹` each RIQN iteration                   | `VariationalContactSpike`: penetration ≈ `mg/k`, RIQN converges every step (max **4 / 6 / 11** iters for `k = 1e2 / 1e3 / 1e4·mg`) | Stiffness **`k ≲ 1e4·mg`**; contact is experimental-within-experimental (Phase C gated — see [`../../plans/082-variational-integrator-solver/contact-roadmap.md`](../../plans/082-variational-integrator-solver/contact-roadmap.md)) |

Common to the whole envelope:

- **Fixed time step `Δt`.** The symplectic / no-secular-drift guarantees assume a
  fixed step. Pick `Δt` so the fastest natural period is resolved by ≳20 steps;
  the energy gate (≥50× better than semi-implicit Euler) was verified over `1e5`
  fixed steps.
- **Zero overhead when not selected.** Outside the VI family the default
  semi-implicit-Euler path is byte-for-byte unchanged
  (`DefaultPathDoesNotEngageVariationalIntegrator`).
- **Determinism.** Bit-identical reruns; binary save/load round-trips the
  two-step history.

## Outside the envelope (not yet supported)

| Out-of-envelope                                     | Why                                                                                                                                         | Path in                                                                                                      |
| --------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------ |
| **Stiffness `k ≳ 1e4·mg`** without reducing `Δt`    | RIQN iteration count grows past budget; the compliant penalty alone is insufficient                                                         | The augmented-Lagrangian rung (Phase C2/C3) bounds force without unbounded stiffness; until then reduce `Δt` |
| **Production contact & friction**                   | Only the opt-in compliant-contact _spike_ exists (hard-coded plane stand-in); the real contact-query-at-trial-configuration seam is unbuilt | Phase C (gated on the contact-query redesign owner)                                                          |
| **Variable `Δt`**                                   | Breaks the fixed-step symplectic guarantee                                                                                                  | Explicit non-goal for graduation; a separate research item                                                   |
| **GPU / batched execution**                         | Not implemented                                                                                                                             | Stretch goal                                                                                                 |
| **Very long spherical/floating chains** (≫20 links) | Anderson verified to 20 links; longer floating chains may need the manifold-aware exact preconditioner                                      | A manifold extension of the recursive-Jacobian preconditioner                                                |

## How to read a non-convergence error

If a step raises the non-convergence error, you are at or past the envelope edge.
In order of likelihood: (1) `Δt` too large for the stiffness/speed; (2) contact
stiffness above `~1e4·mg`; (3) a rank-deficient loop-closure family; (4) a
spherical/floating chain longer than the Anderson-verified range. Reduce `Δt`
first — it is the single most effective lever and preserves all guarantees.

See also: [`README.md` North Star + Gaps](README.md#north-star),
[`performance.md`](performance.md) (per-step cost vs DOF),
[`graduation-criteria.md`](graduation-criteria.md).
