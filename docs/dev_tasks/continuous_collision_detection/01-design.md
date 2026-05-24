# CCD Architecture & Algorithms

## 1. Layered design

```
public detector API  (dart/collision/dart/, dartpy)        [Phase 5]
        |
world-level CCD      (CollisionWorld: casts, swept pairs)   [Phase 2-3]
        |
narrow-phase CCD
  ├─ rigid casts: sphereCast*, capsuleCast*                 [exists]
  ├─ convex–convex conservative advancement                 [exists]
  └─ primitive CCD: point–triangle, edge–edge (ACCD)        [Phase 1, NEW]
        |
geometry kernels: GJK/EPA/MPR, closest-distance primitives
```

The new Phase-1 work is the bottom-right box: primitive CCD. It does not depend
on GJK; it is a closed-form distance + conservative-advancement loop over linear
vertex trajectories.

## 2. Why primitive CCD is the superset gap

The reference engines provide _rigid_ continuous collision: advance a convex (or
swept-sphere) shape under a rigid motion and report a time of impact. DART
already matches that surface (sphere/capsule casts vs. all primitives, plus
convex–convex conservative advancement).

What none of them expose, and what every IPC-class / cloth / FEM solver needs,
is **per-primitive** continuous queries between _independently moving vertices_:

- **point–triangle (PT)**: vertex `p` vs. triangle `(a,b,c)`, all four moving.
- **edge–edge (EE)**: segment `(a,b)` vs. segment `(c,d)`, all four moving.

These are the atoms of mesh self-collision and deformable contact. Supporting
them is what makes the DART CCD a strict superset rather than parity.

## 3. Moving-primitive model

Each involved vertex follows a linear trajectory over the step `t ∈ [0,1]`:

```
x_i(t) = x_i^0 + t · (x_i^1 − x_i^0)        Δx_i = x_i^1 − x_i^0
```

Rigid meshes are the special case where every vertex of a body shares the same
rigid motion sampled at the endpoints; deformables pass arbitrary per-vertex
`x_i^0, x_i^1`. The query returns a conservative time of impact `τ ∈ [0,1]`
(the earliest time the two primitives are within `minSeparation`), or "no
contact in the interval".

## 4. Additive Conservative Advancement (ACCD) — default solver

ACCD (Li et al., _Incremental Potential Contact_, 2020; _Codimensional IPC_, 2021) is conservative: it never returns a `τ` greater than the true first
contact time, so a barrier solver that line-searches to `τ` can never tunnel.

Let the primitive pair involve 4 moving vertices with displacements `Δx_i`.

1. **Recenter** (tightening): collision depends only on _relative_ motion, so
   subtract the mean displacement: `Δx_i ← Δx_i − mean_j(Δx_j)`. This shrinks
   the velocity bound without changing the answer.
2. **Velocity bound** `L` (Lipschitz bound on the closest distance):
   - PT: `L = ‖Δp‖ + max(‖Δa‖, ‖Δb‖, ‖Δc‖)`
   - EE: `L = max(‖Δa‖, ‖Δb‖) + max(‖Δc‖, ‖Δd‖)`
     Any point of a triangle/segment moves at a velocity that is a convex
     combination of its vertices', hence bounded by the per-vertex max. If
     `L = 0`, the primitives do not move relative to each other → no new contact.
3. **Advance**: with `d(t)` the unsigned distance between the primitives at time
   `t` and target gap `g = minSeparation`:
   ```
   τ = 0;  d = distance(τ)
   if d ≤ g: return τ = 0                       # already in contact
   repeat (≤ maxIterations):
       step = (1 − ε)·(d − g) / L               # provably cannot reach contact sooner
       τ += step
       if τ ≥ 1: return  no contact in [0,1]
       d = distance(τ)
       if d − g ≤ tolerance: return τ           # converged to first contact
   ```
   Because the distance can shrink by at most `L·step = (1−ε)(d−g) < d−g` over a
   step, `d` stays above `g`; the iterate approaches the true TOI from below.

`distance(τ)` uses the closed-form closest-distance primitives below.

## 5. Closest-distance primitives

- **point–triangle**: closest point on a triangle to a point (Voronoi-region
  test over face / 3 edges / 3 vertices). Standard formulation (Ericson,
  _Real-Time Collision Detection_, §5.1.5).
- **segment–segment**: clamped parametric solve for the closest points of two
  segments, with the parallel-segment degenerate case (Ericson §5.1.9).

Both are branch-light and allocation-free.

## 6. Exact coplanarity cubic (validation only)

For cross-checking in tests we also implement the Bridson/Provot cubic: the four
moving points are coplanar when a scalar triple product vanishes, giving a cubic
in `t`. Real roots in `[0,1]` are candidate contact times; each is validated by
an inside-triangle (PT) or on-both-segments (EE) test. This is exact in real
arithmetic but **not** conservative under floating point (a missed root tunnels),
so it is test-only. Tests assert `τ_ACCD ≤ τ_cubic` (conservativeness) and that
they agree to tolerance on clean impacts.

## 7. Superset matrix (target)

| Query family                       | Ref engines | DART native | Phase |
| ---------------------------------- | ----------- | ----------- | ----- |
| sphere cast vs. primitives         | partial     | yes         | done  |
| capsule (swept-sphere) cast        | yes         | yes         | done  |
| convex–convex conservative advance | yes         | yes         | done  |
| rigid motion types (linear/screw)  | yes         | partial     | 2     |
| point–triangle CCD                 | no          | **yes**     | 1     |
| edge–edge CCD                      | no          | **yes**     | 1     |
| minimum-separation / gap-aware TOI | no          | **yes**     | 1     |
| swept broad-phase pairs            | partial     | planned     | 3     |

## 8. Performance plan

- Scalar primitive CCD must beat a naive substepping baseline and be allocation-
  free; benchmarks live in `tests/benchmark/collision/native/bm_ccd.cpp`.
- Add comparative CCD rows so the manifest generator scores native vs. the
  strongest reference convex-cast where the query is comparable (sphere/capsule
  cast, convex advancement). Primitive PT/EE rows are native-only (`non-
comparable`) because the reference engines have no equivalent.
