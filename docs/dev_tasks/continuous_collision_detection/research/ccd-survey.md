# Primitive-Level CCD Survey (research notes)

Source: focused literature survey of IPC-class and reference-engine CCD. Captured
by algorithm/paper name to ground the implementation. Distilled to what the DART
implementation depends on; see citations for the full derivations.

## What IPC needs from CCD

IPC (Li et al., _Incremental Potential Contact_, SIGGRAPH 2020) line-searches the
largest step `α` that keeps the linearized trajectory `x → x + αp`
intersection-free; CCD computes that maximal safe `α` once per line-search /
backtrack. For triangle meshes it enumerates two non-degenerate stencils —
**point–triangle (vertex–face)** and **edge–edge** — with point–point /
point–edge as degenerate sub-cases. CCD must return a **conservative TOI**
`t ∈ [0,1]` (a lower bound, never overshooting a real contact); a false negative
evaluates the log-barrier at an interpenetrated state (`B → ∞`) and breaks the
method.

## ACCD (the default we implement)

Codimensional IPC (Li/Kaufman/Jiang, SIGGRAPH 2021), Algorithm 1 — conservative
advancement on _deforming_ primitives, distance-only (no root finding):

1. Mean-center the stencil displacements `pᵢ`: `p̄ᵢ = pᵢ − mean(p)`. Collision
   depends only on relative motion, so this is exact and shrinks the bound.
2. Additive relative-velocity bound
   `lₚ = max‖p̄‖(primitive 1) + max‖p̄‖(primitive 2)`.
   PT: `‖p̄_p‖ + max(‖p̄_a‖,‖p̄_b‖,‖p̄_c‖)`. EE:
   `max(‖p̄_a‖,‖p̄_b‖) + max(‖p̄_c‖,‖p̄_d‖)`. If `lₚ = 0`, no relative motion.
3. With gap `ξ` (minimum separation) and conservative scaling `s∈(0,1)`, the
   safe step in time simplifies to `l_t = (1−s)(d−ξ)/lₚ` (the published
   `(1−s)(d²−ξ²)/((d+ξ)lₚ)` reduces to this). Accumulate `t`, recompute distance,
   stop when the clearance `d−ξ` has converged; `s>0` guarantees finite
   iterations (geometric convergence, ratio ≈ s).

The minimum-separation gap `ξ` is mandatory for the barrier method (it diverges
as distance→0), and an outer conservative rescale `s∈(0.1,0.2)` is applied to the
returned TOI.

## TICCD (validation-grade exact alternative, future)

Wang et al., _A Large Scale Benchmark and an Inclusion-Based Algorithm for CCD_,
ACM TOG 2021. Inclusion-based interval root finder on the multivariate forms
`F_vf(t,u,v)`, `F_ee(t,u,v)`; the corner-evaluated AABB is the tightest inclusion
because `F` is bilinear per variable. Parameters: tolerance `δ` (1e-6), max-checks
`m_I` (1e6). Zero false negatives, controllable false positives, supports minimum
separation. ~3× the cost of the fastest _inaccurate_ methods.

## Classic cubic (test-only validation)

Provot 1997 / Bridson, Fedkiw, Anderson, _Robust treatment of collisions … for
cloth_, SIGGRAPH 2002. Four moving points are coplanar when a scalar triple
product vanishes → a **cubic in `t`**; real roots in `[0,1]` are coplanarity
times, each validated by inside-triangle (PT) or on-both-segments (EE). Exact in
real arithmetic but **not conservative under floating point** (thousands of false
negatives reported in the TICCD benchmark). DART ships it only to cross-validate
ACCD in tests (assert `τ_ACCD ≤ τ_cubic`).

## Reference rigid-body CCD (by algorithm name)

- **Conservative advancement + motion bounds** (Mirtich 1996; Tang/Kim/Manocha,
  _C2A_, ICRA 2009): advance by `Δt ≤ d/μ` with `μ` an upper bound on
  closest-point-directed motion; for linear+angular motion
  `μ ≤ |v·n| + ‖ω×n‖·(r + max‖cᵢ⊥‖)`. DART already does this for convex–convex.
- **GJK ray-cast** (van den Bergen, JGT 2004): cast a ray into the Minkowski
  difference (CSO); first touch = ray hits `∂(A⊖B)`. Basis of the reference
  convex-cast. Constant linear motion only.
- **FCL-style motion models** (Pan/Chitta/Manocha, ICRA 2012):
  `TOC = inf{ t : A(q_A(t)) ∩ B(q_B(t)) ≠ ∅ }` solved by CA on a swept-sphere/RSS
  BVH; supports translational, interpolated/screw, and spline motions, each
  supplying a different `μ` to the same `Δt = d/μ` rule. → DART M2 `CcdMotion`.
- **ODE-style**: essentially discrete; no built-in primitive CCD (fast objects
  tunnel). The superset bar here is trivially met.

## Recommended superset approach (adopted)

Swept-AABB broad-phase (inflated by `ξ`) → **ACCD narrow-phase default**
(robust, allocation-free, gap-aware, no false negatives) → exact/inclusion path
(cubic now, TICCD later) for validation and degenerate cases → optional GJK
ray-cast / C2A fast path for rigid convex pairs.

Pitfalls: never ship the FP cubic as the sole CCD (false negatives); `ξ>0` is
mandatory; cap ACCD iterations and fall back for the large-displacement /
small-clearance worst case; floating-point-filter all sign predicates.

## Citations

Li et al. IPC TOG 2020 · Li/Kaufman/Jiang Codim-IPC TOG 2021 (ACCD) · Wang et al.
TOG 2021 (TICCD) · Provot 1997 / Bridson et al. SIGGRAPH 2002 (cubic) · Mirtich
1996 / Tang/Kim/Manocha C2A ICRA 2009 · van den Bergen JGT 2004 · Pan/Chitta/
Manocha FCL ICRA 2012.
