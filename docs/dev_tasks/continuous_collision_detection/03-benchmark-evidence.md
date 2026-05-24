# CCD Benchmark Evidence

Reproducible measurements backing the performance claims. Numbers are noisy
(host CPU scaling enabled — same caveat as the native collision manifest) but the
ratios are large and stable. Regenerate with:

```bash
cmake --build build/default/cpp/Release --target bm_native_ccd
build/default/cpp/Release/bin/bm_native_ccd \
  --benchmark_filter='ConservativeAdvancement_Convex|Substepping|PointTriangle|EdgeEdge'
```

## Conservative advancement vs. uniform substepping (same GJK kernel)

A continuous-collision engine without a dedicated convex cast must sample the
trajectory at K substeps and run a discrete intersection test at each. Both rows
below use the identical native GJK kernel, isolating the algorithmic win.

| Method                                        | Time (ns) | vs. advancement | TOI resolution |
| --------------------------------------------- | --------- | --------------- | -------------- |
| Conservative advancement (native convex cast) | ~486      | 1×              | tolerance 1e-4 |
| Uniform substepping, 256 samples              | ~26,950   | ~55× slower     | 1/256 ≈ 4e-3   |
| Uniform substepping, 1024 samples             | ~97,660   | ~200× slower    | 1/1024 ≈ 1e-3  |

Conservative advancement is both faster _and_ more accurate: it converges to the
time of impact in a handful of distance evaluations regardless of trajectory
length, whereas substepping pays for every sample and still only resolves the TOI
to the substep size. This is the algorithmic reason the native path dominates the
naive baseline that a discrete engine falls back to.

## Primitive CCD (IPC-class) — native-only (no reference equivalent)

| Query               | Time (ns) |
| ------------------- | --------- |
| Point–triangle ACCD | ~236      |
| Edge–edge ACCD      | ~262      |

These are `non-comparable` rows in manifest terms: the rigid-only reference
engines provide no point–triangle / edge–edge continuous query, so there is no
baseline to beat — the superset advantage is the existence of the query at all.

## Comparative: native cast vs. each engine's OWN best CCD

`tests/benchmark/collision/comparative/bm_ccd.cpp`, built/run in the
`collision-reference` pixi env (FCL/Bullet/ODE present). Native answers each swept
query with one cast; each reference row calls that engine's _best available_
continuous query:

- Reference A — dedicated conservative advancement (interpolated motion).
- Reference B — dedicated GJK convex cast (ray cast against the
  configuration-space obstacle).
- Reference C — substepped discrete narrow phase (this engine has no native
  continuous query; trajectory sampling is its only option).

The cases span the shape families, the native algorithm paths, and
translational, rotational, screw, and spline motion. Ref B uses the linear GJK
convex cast for translation and the angular continuous convex collision for
rotation / screw. Reported as **native speedup** (× faster than each engine); the
host runs under load with CPU scaling on, so absolute timings drift run to run
while the speedup ratios are stable across runs. Speedups use CPU-time medians.

| Case (native algorithm)             | vs Ref A (cons. adv.) | vs Ref B (convex cast) | vs Ref C (substepped) |
| ----------------------------------- | --------------------- | ---------------------- | --------------------- |
| Box–box (GJK cons. advancement)     | 2.3×                  | 1.2×                   | 175×                  |
| Octahedron (GJK cons. advance.)     | 2.9×                  | 1.5×                   | — (no convex CCD)     |
| Sphere–sphere (closed-form)         | 72×                   | 23×                    | 6467×                 |
| Sphere–box (closed-form slab)       | 21×                   | 11×                    | 1841×                 |
| Capsule–capsule (swept-sphere)      | 11×                   | 2.6×                   | 518×                  |
| Sphere–cylinder (closed-form)       | 15×                   | 16×                    | 1287×                 |
| Box–box, translating + rotating 90° | 1.4×                  | 1.9×                   | 46×                   |
| Box–box, screw motion (90° about Z) | **1.85×**             | 5.4×                   | 188×                  |
| Box–box, spline motion (fast mode)  | **1.9×**              | — (no spline)          | 107×                  |

**Native beats all three engines on every one of the nine cases** — every shape
family (primitive, mixed, convex polytope, cylinder) and translational,
rotational, screw, and spline motion. The tightest margin is box–box vs. the GJK
convex cast (1.2×). The screw row — which native lost (~1.37× behind) before the
directed-angular bound below — is now a 1.85× win over the dedicated screw-motion
model. (Octahedron and spline have no substepped Ref-C-vs-convex-CCD caveat: ODE
has no convex CCD, and only Ref A has any spline motion at all.)

### The spline row: native is the only _correct_ spline cast

A spline (cubic-Bezier) path is the one motion model the linear/screw casts
cannot represent — it can curve away from the chord between its endpoints, so a
straight or screw cast over the same endpoints misses a collision the spline
actually makes. Bullet and ODE have no spline cast; only Ref A does.

The headline number for the spline row hides the more important finding. On the
benchmark sweep the box first contacts the obstacle at **t = 0.326**:

- **Native, conservative (default): t = 0.326 — correct.** ~1.9µs.
- **Ref A: t = 0.598 — wrong.** It strides past the first contact and reports one
  deep inside the overlap (the box has already tunnelled ~27% through the
  obstacle). ~1.07µs. Its speed comes entirely from a non-conservative bound that
  skips the contact — exactly the failure CCD exists to prevent, and the property
  barrier/IPC solvers cannot tolerate.
- **Native, fast (`CcdAdvancement::Fast`): ~0.56µs.** Opt-in non-conservative mode
  that matches Ref A's larger steps (and its overshoot) — **1.9× faster than Ref
  A** when first-contact precision is not required.

So native wins the spline case both ways: its fast mode is the fastest spline
cast measured, and its conservative mode is the _only_ spline cast that returns
the true first contact. Native offers both modes; the reference engine offers
only the non-conservative one. (Switching Ref A from `fcl::Box` to a general
`fcl::Convex` of the same box left its time unchanged, confirming the gap is the
bound, not an analytic box support.) Conservativeness is verified by the spline
unit tests (`τ_native ≤ τ_substep`, and a chord cast over the same endpoints
misses).

### How the rotational / screw rows were won (the directed bound)

The conservative-advancement step size is `distance / μ`, so a tighter motion
bound `μ` means fewer steps. The naive bound `μ = |v| + θ·r_max` (translation +
arc length at the full radius) is far too loose for rotation/screw. Native now
uses a **C2A-style directed bound** (Tang/Kim/Manocha, ICRA 2009) recomputed each
step from the GJK closest direction `n`:

```
μ = |v_rel| + Σ_bodies  θ · r⊥ · |n × axis|
```

The linear term stays undirected (`|v_rel|`) so translation is strictly
conservative (no sliding-miss). Only the rotational term is projected: the spin
contributes to closing the gap only through its component perpendicular to `n`,
i.e. `(ω×r)·n ≤ |ω|·r⊥·|n×axis|`. This makes the term **vanish for a coaxial
screw** (`n ∥ axis ⇒ |n×axis| = 0`), where the spin is perpendicular to the
approach and does not close the gap — so `μ = |v|`, the exact closing rate, and
the screw converges in ~5 steps instead of ~12. For an ordinary rotation
(`n ⊥ axis`) the factor is 1, recovering the full angular term (no regression).
The bound is conservative (`μ ≥` the instantaneous approach rate); the
rotational-sweep and starting-in-contact unit tests verify it does not overshoot
(`test_ccd` 75/75).

### Targeted optimizations landed

1. **Translation slerp fast-path** — `interpolateTransform` recomputed a
   quaternion slerp every step even for translational sweeps; skipping it when
   orientation is unchanged turned the box–box row from a parity/loss into a lead.
2. **Cached motion sampling (`MotionSample`)** — endpoint quaternions are built
   once per cast instead of every iteration for both bodies.
3. **GJK warm-starting** — each conservative-advancement step reseeds the GJK
   closest-point query from the previous step's simplex (configurations barely
   change, so the prior search directions converge in fewer inner iterations;
   supports are recomputed at the current configuration, so results are
   unchanged).
4. **De-virtualized GJK support callbacks** — the casts call a templated GJK
   query (`detail::queryT`, `gjk_inl.hpp`) instead of the `std::function` form,
   removing an indirect call per support evaluation. This took the rotating row
   from parity to a clear lead and tightened every convex row. The
   `std::function` `Gjk::query` is retained (now forwarding to the same templated
   core) for all other callers.
5. **Directed conservative-advancement bound** — `μ = |v_rel| + Σ θ·r⊥·|n×axis|`
   recomputed per step from the GJK closest direction (see above). Conservative;
   reduces to `|v_rel|` for translation (no regression) and to the full angular
   term for ordinary rotation, but collapses the angular term for coaxial screws.
   This is what turned the screw row from a ~1.37× loss into a 1.85× win.
6. **Acceleration-bounded spline step** — the spline cast's step is the larger
   root of `(accel/2)·dt² + (v0+angular)·dt − distance = 0`, bounding the point
   speed over the step by `v0 + s·accel` instead of a flat max speed. Tighter
   when a curve is slow now but accelerates later; cut the box–box spline cast
   from ~4.4µs to ~2.3µs with no loss of conservativeness.
7. **Translation-only support fast path** — when a body has no rotation (a
   translational spline sweep, or the static target), its support skips two
   matrix-vector products per call; ~2.3µs → ~1.9µs on the spline cast.

All are behavior-preserving: `test_gjk` 32/32, `test_narrow_phase` 24/24,
`test_ccd` 75/75, and `test_primitive_ccd` 16/16 pass.

Reproduce:

```bash
pixi run -e collision-reference config OFF Release
pixi run -e collision-reference bash -lc \
  'CMAKE_BUILD_DIR=build/collision-reference/cpp/Release \
   python scripts/cmake_build.py --target bm_comparative_ccd'
taskset -c 2 build/collision-reference/cpp/Release/bin/bm_comparative_ccd \
  --benchmark_min_time=0.4s --benchmark_repetitions=9 \
  --benchmark_report_aggregates_only=true
```

### Scope of the claim & what remains

Nine cases are measured and **native wins all nine** vs. each engine's own best
CCD — every shape family (primitive, mixed, convex polytope, cylinder) and
translational, rotational, screw, and spline motion. The directed-angular bound
closed the screw row; the spline row is won by the fast mode on speed and the
conservative mode on correctness (the reference spline returns a wrong first
contact). Remaining breadth (not blocking the claim):

1. Remaining shape pairs (triangle-mesh — note non-convex mesh CCD is native's
   primitive point-triangle / edge-edge path, a superset with no reference
   equivalent) and hit/miss/grazing configurations.
2. Re-confirm ratios on a quiesced host (this one is loaded; ratios are stable
   but absolute numbers are inflated).
3. Then M3 (swept broad-phase) and M5 (public detector-level CCD API + dartpy).
