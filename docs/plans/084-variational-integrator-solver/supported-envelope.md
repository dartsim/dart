# VI Solver — Supported Envelope

The variational integrator (VI) is an **experimental** `World` integration family
(opt-in via `WorldOptions::multibodyOptions` at construction or
`World::setMultibodyOptions({.integrationFamily = MultibodyIntegrationFamily::Variational})`).
"Experimental" means we do not yet make the API-stability
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

| Regime                                                  | Mechanism                                                                                                                | Evidence                                                                                                                           | Bound                                                                                                                                                                                        |
| ------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------ | ---------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Euclidean chains** (prismatic / revolute, fixed base) | Exact recursive-Jacobian Newton preconditioner (O(n) non-symmetric ABA + backtracking line search)                       | `LongChainExactPreconditionerConvergesWithinBudget` (100 & 128 links); quadratic, **~3 iterations independent of length**          | Verified to **128 DOF**; expected to extend further (length-independent iteration count)                                                                                                     |
| **Floating base + spherical joints**                    | Manifold quasi-Newton (`Δt·M⁻¹`) + tangent-space Anderson acceleration (per-joint log-difference mixing, `jointRetract`) | `ManifoldAndersonAcceleratesSphericalChain` (20-link spherical: **200/200 steps, ≤9 iters**), `FloatingSphericalChainStaysCorrect` | **20-link spherical** chains; floating base + spherical retraction FD-verified                                                                                                               |
| **Holonomic loop closures** (Point / Distance / Rigid)  | Newton projection onto the constraint manifold reusing the O(n) ABI                                                      | Long-horizon hold; constraint Jacobians FD-verified; `sx_loop_closure` demo holds to machine precision over 100 steps              | Well-conditioned closures (rank-deficient configs — e.g. POINT/RIGID on a planar arm — are excluded; use DISTANCE)                                                                           |
| **Scoped contact & friction**                           | Smooth contact/friction forces in the forced-DEL residual, evaluated at the trial `qᵏ⁺¹` each RIQN iteration             | Ground contact/friction and AL save/load tests; `VariationalContactSpike`; `VariationalLinkContact.SphereContactStopsSlidingLink`  | Link-point-vs-analytic-ground and link sphere-sphere pairs; stiffness **`k ≲ 1e4·mg`**; C1 lagged friction + C2 compliant contact by default, C3 AL centering opt-in via `dualUpdateCadence` |

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

| Out-of-envelope                                     | Why                                                                                                                                         | Path in                                                                             |
| --------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------- |
| **Stiffness `k ≳ 1e4·mg`** without reducing `Δt`    | RIQN iteration count grows past budget; the compliant/AL envelope is documented below that stiffness                                        | Reduce `Δt`; treat harder contacts as future C4/barrier or sparse-contact research  |
| **Arbitrary link geometry contact**                 | Ground points and sphere-sphere link pairs are implemented; warm-started arbitrary-geometry candidate generation at trial `qᵏ⁺¹` is unbuilt | PLAN-scale follow-up coordinated with the rigid IPC / deformable IPC geometry stack |
| **Hard IPC barrier contact (C4)**                   | Stiff barrier curvature mis-scales the `Δt·M⁻¹` quasi-Newton preconditioner and was intentionally deferred                                  | Optional last rung after sparse-contact experiments                                 |
| **Variable `Δt`**                                   | Breaks the fixed-step symplectic guarantee                                                                                                  | Explicit non-goal for graduation; a separate research item                          |
| **GPU / batched execution**                         | Not implemented                                                                                                                             | Stretch goal                                                                        |
| **Very long spherical/floating chains** (≫20 links) | Anderson verified to 20 links; longer floating chains may need the manifold-aware exact preconditioner                                      | A manifold extension of the recursive-Jacobian preconditioner                       |

## How to read a non-convergence error

If a step raises the non-convergence error, you are at or past the envelope edge.
In order of likelihood: (1) `Δt` too large for the stiffness/speed; (2) contact
stiffness above `~1e4·mg`; (3) a rank-deficient loop-closure family; (4) a
spherical/floating chain longer than the Anderson-verified range. Reduce `Δt`
first — it is the single most effective lever and preserves all guarantees.

See also:
[`PLAN-084 current state`](../084-variational-integrator-solver.md#current-state),
[`performance.md`](performance.md) (per-step cost vs DOF),
[`graduation-criteria.md`](graduation-criteria.md).
