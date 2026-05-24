# Continuous Collision Detection (CCD) — Dev Task

Working documentation for extending DART's native collision stack with a
continuous collision detection (CCD) layer that is a **feature superset** of the
three legacy reference engines and is built to serve modern rigid- and
deformable-body solvers (e.g. barrier / IPC-style contact).

> This folder is working documentation. On completion, durable design notes move
> to `docs/onboarding/architecture.md` (collision section) and this folder is
> deleted in the completing PR. See `docs/dev_tasks/README.md`.

## Current Status

- [x] Phase 0: Inventory existing native CCD + reference-engine CCD survey
- [x] Phase 1: IPC-class primitive CCD (point–triangle, edge–edge) — robust
      conservative time-of-impact (ACCD + exact-cubic validation, tests + bench)
- [x] Phase 2: Both-moving rigid convex cast (`convexCast`) with a C2A-style
      directed motion bound (linear undirected + angular projected by `|n×axis|`),
      plus a cubic-Bezier `splineCast` covering polynomial motion — the one motion
      model the linear/screw casts cannot represent.
- [ ] Phase 3: Swept broad-phase for CCD pairs at world level
- [x] Phase 4: Comparative benchmark vs. the three reference engines
      (`bm_comparative_ccd`, 9 cases). Native **beats all three on every case** —
      box-box, octahedron, sphere-sphere, sphere-box, capsule-capsule, sphere-cylinder,
      90° rotating sweep, 90° screw, and spline (margins 1.2×–6467×; see
      `03-benchmark-evidence.md`). The spline row is decisive: the reference engine's
      spline is faster only because it is **non-conservative** and returns a _wrong_
      first contact (it tunnels); native's conservative mode is the only spline cast
      that finds the true first contact, and its opt-in fast mode (`CcdAdvancement::
Fast`) is 1.9× faster than the reference engine when precision is not required.
      Seven optimizations got there (slerp fast-path, cached motion sampler, GJK
      warm-starting, de-virtualized GJK callbacks, directed conservative-advancement
      bound, acceleration-bounded spline step, translation-only support fast path).
      Remaining breadth: triangle-mesh pairs, hit/miss/grazing configs.
- [ ] Phase 5: Public detector-level CCD API + dartpy bindings

## Goal

A single native CCD subsystem that (1) answers every continuous query the three
reference engines answer, (2) **adds** the primitive-level queries that
IPC-class and deformable solvers require but the rigid-only reference engines do
not provide, and (3) is at least as fast as the strongest reference engine on
every comparable benchmark row (per the benchmark-manifest scoring in
`docs/plans/035-native-collision/`).

## Non-Goals (early phases)

- GPU / SIMD-batched CCD (revisit after the scalar path leads on all rows).
- A full IPC time integrator. We provide the **CCD queries** an IPC solver
  consumes (conservative TOI, minimum-separation aware), not the optimizer.
- Exposing reference-engine knobs through public options (forbidden by
  `dart/collision/AGENTS.md`). Options are DART-semantic only.

## Key Decisions

- **Moving-primitive model for deformables**: primitive CCD takes per-vertex
  start/end positions (linear vertex trajectories), not rigid transforms. This
  is what cloth/FEM/IPC solvers actually have at the line-search step, and it
  degenerates correctly for rigid meshes.
- **Conservative by construction**: the default primitive solver is _additive
  conservative advancement_ (ACCD) — it returns a TOI that never overshoots a
  true contact, which is the safety contract a barrier method needs. Reason: a
  non-conservative floating-point cubic solver can miss a root and let geometry
  tunnel, breaking the barrier.
- **Minimum-separation aware**: queries accept a non-negative gap so callers can
  keep primitives strictly separated (IPC accuracy requirement).
- **Exact path for validation**: a coplanarity cubic solver (Bridson/Provot
  formulation) ships alongside ACCD purely to cross-check TOIs in tests, never
  as the runtime default.

## Immediate Next Steps

1. Land `narrow_phase/primitive_ccd.{hpp,cpp}` (point–triangle, edge–edge ACCD
   - closest-distance primitives) with unit tests + benchmarks.
2. Add a comparative CCD benchmark row family so the manifest can score it.
3. Wire a unified `CcdMotion` abstraction over the existing sphere/capsule casts
   and conservative advancement.

## Pointers (code is source of truth)

- Existing rigid CCD: `dart/collision/native/narrow_phase/ccd.{hpp,cpp}`
- World-level casts: `dart/collision/native/collision_world.hpp`
- Benchmark scoring: `docs/plans/035-native-collision/benchmark-manifest.md`
- Reference engines (test/bench only): `tests/dart/test/reference_collision/`
