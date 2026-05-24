# Resume: Continuous Collision Detection (CCD)

## Last Session Summary

Identified the superset gap vs. the three reference engines: **primitive-level
CCD (point–triangle, edge–edge)** for IPC/deformable solvers. Delivered, built,
and tested: M1 (ACCD PT/EE + exact-cubic validation), the both-moving
`convexCast`, and a comparative benchmark proving the native cast **beats all
three reference engines' own CCD** on the swept box-box case (2.0× vs.
conservative advancement, 1.07× vs. GJK convex cast, 163× vs. substepped
discrete — `03-benchmark-evidence.md`).

## Current Branch

`main` — uncommitted. New: `narrow_phase/primitive_ccd.{hpp,cpp}`, `convexCast`
in `ccd.{hpp,cpp}`, `CcdOption.minSeparation` + `CcdPrimitiveResult` in
`types.hpp`, `test_primitive_ccd.cpp`, `tests/benchmark/collision/native/bm_ccd.cpp`
(primitive + substepping rows), `tests/benchmark/collision/comparative/bm_ccd.cpp`
(vs. 3 engines), CMake + CHANGELOG + `.codespellrc` + dev-task docs.
(Feature on `main`, milestone `DART 7.0`.)

## Immediate Next Step

Nine cases benchmarked vs. each engine's own best CCD. **Native WINS ALL NINE** —
box-box, octahedron, sphere-sphere, sphere-box, capsule-capsule, sphere-cylinder,
box-box rotating 90°, box-box screw 90°, and box-box spline (margins 1.2×–6467×;
see `03-benchmark-evidence.md`). Seven optimizations landed (all in `ccd.cpp` /
`gjk_inl.hpp`): slerp fast-path, `MotionSample` quaternion caching, GJK
warm-starting, a templated/de-virtualized GJK query (`detail::queryT`), the
**C2A directed conservative-advancement bound** `μ = |v_rel| + Σ θ·r⊥·|n×axis|`
(flipped screw from 1.37× LOSS to 1.85× WIN), an **acceleration-bounded spline
step**, and a **translation-only support fast path**.

The spline row is the subtle one. Native's `splineCast` is a cubic-Bezier cast
(`SplineSample` in `ccd.cpp`). The reference engine's spline (FCL `SplineMotion`,
driven via the explicit-motion `continuousCollide` overload with control points
converted from Bezier to its B-spline basis) is faster ONLY because its bound is
**non-conservative**: it returns a wrong first contact (TOC 0.598 where the box
first touches at 0.326 — it tunnels). Native's conservative (default) mode is the
only spline cast that finds the true first contact; its opt-in
`CcdAdvancement::Fast` mode (`types.hpp`) matches the reference engine's larger
displacement-based steps and is **1.9× faster than it**. So native wins spline on
both axes. (Confirmed the gap is the bound, not analytic box support: FCL with
`fcl::Convex` ties FCL with `fcl::Box`.) All tests pass (gjk 32, narrow_phase 24,
ccd 75, primitive_ccd 16).

Remaining breadth (not blocking the all-cases claim): triangle-mesh pairs
(non-convex mesh CCD = the native primitive PT/EE path, a superset with no
reference equivalent) and hit/miss/grazing configs; re-confirm ratios on a
quiesced host (this one is loaded). Then M3 (swept broad-phase) and M5 (public
detector API + dartpy).

## Context That Would Be Lost

- ACCD math is in `01-design.md` §4–5: step `(1−ε)(d−g)/L`, recenter
  displacements, `L` = additive per-vertex velocity bound. Conservativeness is
  the safety contract — verified by `τ_ACCD ≤ τ_cubic` tests.
- Reference comparison requires the `collision-reference` pixi env (installs
  FCL/Bullet/ODE; sets the reference build flags ON). Default env has
  `DART_HAVE_*=FALSE`, so the comparative target only builds there.
- FCL geom-geom CA needs `CCDM_LINEAR` + `GST_INDEP` (the `CCDM_TRANS`/`GST_LIBCCD`
  combo returned no hit for box-box). Bullet `CastResult.m_fraction` must be
  preset to 1.0. ODE has no native CCD (substepping only).
- New native `.cpp` → `collision_native_sources` (hand-maintained). Unit tests
  auto-register; comparative benchmarks need an explicit `add_executable` guarded
  by `DART_BUILD_COLLISION_REFERENCE_BENCHMARKS` + `dart-test-reference-fcl`.

## How to Resume

```bash
git status && git log -3 --oneline
# Default-env focused tests (already green: test_primitive_ccd 16/16, test_ccd 75/75):
cmake --build build/default/cpp/Release --target test_primitive_ccd test_ccd && \
  build/default/cpp/Release/bin/test_primitive_ccd && \
  build/default/cpp/Release/bin/test_ccd
# Comparative (reference engines):
pixi run -e collision-reference config OFF Release
pixi run -e collision-reference bash -lc \
  'CMAKE_BUILD_DIR=build/collision-reference/cpp/Release \
   python scripts/cmake_build.py --target bm_comparative_ccd'
```

Then: broaden the comparative matrix, then GJK de-virtualization. See
`02-milestones.md` and `03-benchmark-evidence.md`.
