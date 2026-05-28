# IPC Deformable Solver - Dev Task

## Current Status

- [x] Phase 0: establish the upstream IPC scene corpus manifest and validation
      tooling.
- [ ] Phase 1: mesh/material state, scene loading, boundary conditions, restart,
      diagnostics, and contact-free stepping.
  - [x] Mesh/material-state sub-slice: optional surface/tetrahedral topology,
        material validation, density-based tetrahedral mass assembly, boundary
        surface extraction, serialization, benchmark counters, and body-owned
        GUI surface topology.
  - [x] Contact-free scene/boundary/diagnostics sub-slice: upstream-style
        tetra mesh scene loading, generated spring replay edges, scripted
        Dirichlet/Neumann controls, restart diagnostics, replay benchmarks, and
        headless GUI scene capture.
  - [ ] Remaining Phase 1 work: broader scene-option coverage, BE/NM state,
        output-file compatibility decisions, and additional contact-free mesh
        scene replays.
- [ ] Phase 2: PT/EE distance kernels, broad-phase candidates, and conservative
      CCD line-search bounds.
  - [x] Internal primitive-distance kernel sub-slice: point-triangle and
        edge-edge squared-distance values, closest-feature classification,
        gradients, the first solver-facing Hessian contract, IPC-style
        edge-edge mollifier threshold/value/gradient/Hessian,
        finite-difference regression tests, and `bm_ipc_distance_kernels`.
  - [x] Internal analytic-Hessian optimization sub-slice: feature-wise exact
        point-triangle and edge-edge distance Hessians for vertex, edge, face,
        and interior closest features, degenerate triangle edge fallback, and
        benchmark evidence replacing the finite-difference Hessian placeholder.
  - [x] Internal candidate-set sub-slice: deterministic unique surface-edge
        extraction, point-triangle and edge-edge primitive candidate assembly,
        incident/adjacent exclusion filters, exact activation-distance filtering
        through the primitive distance kernels, sweep-versus-brute-force
        regression tests, and `bm_ipc_candidate_set`.
  - [x] Internal CCD step-bound sub-slice: conservative point-triangle and
        edge-edge normalized step bounds through native primitive CCD, initial
        separation-band handling, deterministic candidate aggregation, exact CCD
        regression tests, sampled safety checks, and
        `bm_ipc_continuous_collision_step`.
  - [x] Internal barrier-kernel sub-slice: IPC C2 clamped-log scalar barrier
        functions over squared distances, raw analytic point-triangle and
        edge-edge barrier gradients/Hessians, explicit-threshold edge-edge
        mollifier product-rule derivatives, finite-difference regression tests,
        and `bm_ipc_barrier_kernel`.
  - [x] Internal tangent-stencil sub-slice: upstream-style point-triangle,
        edge-edge, point-edge, and point-point tangent bases, closest-point
        parameters, tangent projection matrices, tangent metric matrices,
        regression tests, and `bm_ipc_tangent_stencil`.
  - [x] Internal motion-aware candidate-culling sub-slice: conservative
        start/end swept-AABB point-triangle and edge-edge candidate assembly,
        endpoint-distance metadata, static-miss crossing regressions, and
        `bm_ipc_motion_aware_candidate_set`.
  - [x] Internal candidate-buffer reuse sub-slice: reusable output overloads
        for static and motion-aware candidate builders, stale-state reset,
        capacity-preserving candidate storage, regression tests, and
        return-wrapper versus reusable-buffer benchmark counters.
  - [ ] Remaining Phase 2 work: solver-owned contact buffers and solver-wired
        CCD line search.
- [ ] Phase 3: clamped barriers, projected Newton, sparse assembly, and solver
      statistics.
- [ ] Phase 4: lagged smoothed friction and friction diagnostics.
- [ ] Phase 5: complete the upstream scene corpus as DART-native tests,
      examples, benchmarks, profiling artifacts, and headless Filament evidence.

## Goal

Complete PLAN-081's remaining IPC-class deformable solver work in bounded PRs
until DART covers the IPC paper method family, upstream example corpus,
material/property options, tests, benchmarks, and visual evidence with a
DART-owned implementation.

## Non-Goals For The Current Implementation Slices

- No claim that DART has full IPC, mesh contact, projected Newton, friction, or
  paper-level parity.
- No claim that imported `energy`, `timeIntegration`, contact, ground, or
  friction directives are honored beyond explicit warnings and contact-free
  replay scaffolding.
- No FEM elasticity, material-driven stiffness, solver-wired CCD line search,
  or projected Newton solve in the current point-mass/spring stepping path.
- No vendored or runtime dependency on `ipc-sim/IPC`.

## Key Decisions

- The durable scene inventory lives in
  [`../../plans/081-deformable-implicit-barrier-solver/ipc_scene_corpus_manifest.json`](../../plans/081-deformable-implicit-barrier-solver/ipc_scene_corpus_manifest.json).
  This dev task links to it instead of duplicating row-level state.
- The durable per-figure showcase plan lives in
  [`../../plans/081-deformable-implicit-barrier-solver/ipc-paper-figure-showcase.md`](../../plans/081-deformable-implicit-barrier-solver/ipc-paper-figure-showcase.md).
  Every IPC kernel slice must advance at least one row there toward `landed`
  before the stack can claim PLAN-081 completion.
- The manifest uses upstream commit
  `573d2c7e04104d3f9baf526bdaee7745891a571a` and tracks 154 `.txt` scene paths:
  144 regular scene files plus 10 symlink aliases under the SQP benchmark
  corpus.
- Every upstream scene path has a non-`unclassified` family and a planned DART
  target type: `test`, `benchmark`, `example`, `manual`, or `not-applicable`.
- Public DART APIs should keep DART-owned names and avoid exposing upstream
  IPC solver vocabulary as user-facing solver selectors.

## Immediate Next Steps

1. Continue Phase 2 with solver-owned contact buffers and solver-wired CCD line
   search. The current primitive kernels, candidate sets, analytic Hessians,
   clamped-log barrier kernels, tangent stencils, motion-aware swept candidate
   culling, reusable candidate-output buffers, and CCD step-bound helpers are
   internal scaffolding only and are not yet wired into `World::step()`.
2. Finish the rest of Slice 1 from PLAN-081 in parallel when needed for corpus
   scenes: broader scene asset loading, BE/NM state, output diagnostics
   compatibility decisions, and more contact-free mesh replays. The
   mesh/material-state and scene/boundary sub-slices are scaffolding only and
   still use the existing point-mass/spring stepping path.
3. Use the scene corpus manifest to select the first tutorial and paper-facing
   scenes, then replace planned artifacts with implemented DART commands as
   each scene lands.
4. Add focused unit tests and benchmarks for each landed slice before promoting
   scene-level examples.
5. For every GUI-facing scene, attach long-horizon headless Filament evidence
   to the PR rather than committing transient screenshots or videos.

## Verification

Run the manifest checks before changing or relying on scene coverage:

```bash
pixi run python scripts/generate_ipc_scene_manifest.py --upstream-dir /tmp/ipc-upstream
pixi run python scripts/check_ipc_scene_manifest.py --upstream-dir /tmp/ipc-upstream
pixi run python scripts/check_ipc_scene_manifest.py
```

The upstream checkout must be at
`573d2c7e04104d3f9baf526bdaee7745891a571a`.

For the mesh/material-state sub-slice, keep the verification language precise:
it covers topology/material validation, density mass assembly, serialization,
benchmark setup/step counters, and long-horizon rendering of body-owned surface
topology. It does not cover FEM elasticity, mesh contact, no-intersection or
no-inversion guarantees, CCD line search, projected Newton, friction, upstream
scene parity, or full IPC parity.

For the contact-free scene/boundary/diagnostics sub-slice, keep the verification
language precise: it covers the audited tetra-mesh scene text subset, generated
spring replay edges, scripted DBC/NBC controls, binary restart continuity,
diagnostics JSON, replay/load benchmark counters, and long-horizon GUI captures
through `--deformable-scene`. It does not cover FEM elasticity, material-driven
stiffness, mesh contact, ground/friction behavior from upstream scene files,
CCD line search, projected Newton, or full scene-corpus parity.

For the primitive-distance kernel sub-slice, keep the verification language
precise: it covers internal squared-distance values, closest-feature
classification, gradients, feature-wise analytic Hessians, the edge-edge
mollifier, finite-difference derivative tests, and microbenchmark timings. It
does not yet cover tangent bases, broad-phase candidate assembly, CCD line
search, barrier assembly, projected Newton, friction, or scene-level IPC
contact behavior.

For the candidate-set sub-slice, keep the verification language precise: it
covers deterministic surface-edge extraction, internal point-triangle and
edge-edge candidate assembly, incident/adjacent filtering, exact
activation-distance filtering through the primitive distance kernels,
sweep-versus-brute-force regression tests, and benchmark counters. It does not
yet cover motion-aware candidate culling, barrier assembly, solver integration,
projected Newton, friction, persistent contact caches, tangent bases, or
scene-level IPC contact behavior.

For the CCD step-bound sub-slice, keep the verification language precise: it
covers conservative internal point-triangle and edge-edge normalized step
bounds through native primitive CCD, initial separation-band handling,
deterministic candidate aggregation, exact-CCD comparison where available,
sampled distance safety before the returned bound, and benchmark counters. It
does not yet cover motion-aware broad-phase culling, barrier assembly,
solver-wired CCD line search, projected Newton, friction, persistent contact
caches, tangent bases, or scene-level IPC contact behavior.

For the barrier-kernel sub-slice, keep the verification language precise: it
covers internal C2 clamped-log scalar barrier functions over squared distances,
raw point-triangle and edge-edge barrier gradients/Hessians through the existing
distance kernels, and explicit-threshold edge-edge mollifier product-rule
derivatives. It does not yet cover barrier stiffness adaptation, PSD projection,
projected Newton, candidate-buffer assembly, solver-owned contact caches,
tangent bases, friction, solver-wired CCD line search, or scene-level IPC
contact behavior.

For the tangent-stencil sub-slice, keep the verification language precise: it
covers internal point-triangle, edge-edge, point-edge, and point-point tangent
bases, closest-point parameters, tangent projection matrices, and tangent
metric matrices for future contact/friction assembly. It does not yet cover
friction energy/gradient/Hessian, lagged friction convergence, barrier
stiffness adaptation, PSD projection, projected Newton, candidate-buffer
assembly, solver-owned contact caches, solver-wired CCD line search, or
scene-level IPC contact behavior.

For the motion-aware candidate-culling sub-slice, keep the verification
language precise: it covers internal conservative swept-AABB point-triangle and
edge-edge candidate assembly over start/end positions, default topology
filters, endpoint-distance metadata, static-endpoint miss regressions, and a
brute-force swept-AABB oracle. It does not yet cover solver-owned persistent
contact buffers, barrier assembly, solver-wired CCD line search, projected
Newton, friction, or scene-level IPC contact behavior.

For the candidate-buffer reuse sub-slice, keep the verification language
precise: it covers internal reusable-output overloads for static and
motion-aware candidate builders, clearing stale state across changing topology,
preserving candidate vector capacity, and keeping existing return-by-value
wrappers behavior-equivalent. It does not yet cover reusable sweep-item scratch
buffers, solver-owned persistent contact caches, barrier assembly,
solver-wired CCD line search, projected Newton, friction, or scene-level IPC
contact behavior.

Current primitive-distance local gates:

```bash
pixi run lint
cmake --build build/default/cpp/Release --target test_primitive_distance bm_ipc_distance_kernels
./build/default/cpp/Release/bin/test_primitive_distance
ctest --test-dir build/default/cpp/Release -R '^test_primitive_distance$' --output-on-failure
./build/default/cpp/Release/bin/bm_ipc_distance_kernels --benchmark_min_time=0.05s --benchmark_filter='BM_Ipc'
pixi run check-api-boundaries
```

The first local gate pass on 2026-05-27 passed `pixi run lint`, the focused
target build, 13 `test_primitive_distance` cases, the CTest registration path,
`pixi run check-api-boundaries`, and `pixi run check-lint`. The latest analytic
Hessian optimization gate passed the same 13-case test binary and reported
roughly 8-16 ns for value/gradient distance paths, 487 ns for the
point-triangle face distance Hessian, 486 ns for the point-triangle edge
Hessian, 26 ns for the point-triangle vertex Hessian, 515 ns for the edge-edge
interior distance Hessian, 490 ns for the edge-edge point-edge Hessian, 22 ns
for the edge-edge point-point Hessian, and 203 ns for the analytic edge-edge
mollifier Hessian path. CPU scaling was enabled, so treat these as local smoke
numbers rather than a final performance claim.

Current candidate-set local gates:

```bash
cmake --build build/default/cpp/Release --target test_contact_candidate_set bm_ipc_candidate_set
./build/default/cpp/Release/bin/test_contact_candidate_set
./build/default/cpp/Release/bin/bm_ipc_candidate_set --benchmark_min_time=0.05s --benchmark_filter='BM_IpcCandidateSet'
```

The latest local candidate-set gate pass on 2026-05-27 passed the focused
target build and 6 `test_contact_candidate_set` cases. The benchmark smoke
reported the sweep path faster than brute force on the checked workloads: cloth
8x8 was about 27 us versus 327 us, cloth 16x16 was about 0.35 ms versus
5.10 ms, tetra-surface 4x4 was about 9.4 us versus 83 us, and tetra-surface
8x8 was about 89 us versus 1.51 ms. CPU scaling was enabled, so treat these as
local smoke numbers rather than a final performance claim.

Current motion-aware candidate-culling local gates:

```bash
cmake --build build/default/cpp/Release --target test_contact_candidate_set test_continuous_collision_step bm_ipc_motion_aware_candidate_set
./build/default/cpp/Release/bin/test_contact_candidate_set
ctest --test-dir build/default/cpp/Release -R '^(test_contact_candidate_set|test_continuous_collision_step)$' --output-on-failure
./build/default/cpp/Release/bin/bm_ipc_motion_aware_candidate_set --benchmark_min_time=0.05s --benchmark_filter='BM_IpcMotionAwareCandidateSet'
```

The motion-aware gate should prove that static endpoint candidate construction
misses fast point-triangle and edge-edge crossings that the swept builder keeps,
and should compare the swept builder against the brute-force swept-AABB oracle.
Treat benchmark timings as local smoke numbers unless they are collected in a
controlled profiling run with CPU frequency scaling addressed.

The first local motion-aware candidate-culling gate pass on 2026-05-27 passed
the focused target build, 12 `test_contact_candidate_set` cases, 7
`test_continuous_collision_step` cases, the CTest registration path,
`pixi run build`, 183 `pixi run test-unit` cases, `pixi run
check-api-boundaries`, and `git diff --check`. The new benchmark smoke reported
the swept builder faster than the brute-force swept-AABB oracle on the checked
falling-point workloads: 16 pairs were about 2.3 us versus 6.0 us, and 64 pairs
were about 17.9 us versus 84.9 us. The 128-pair swept path was about 40.6 us,
and the coherent-translation inflation probes were about 3.9 us for 16 pairs
and 22.0 us for 64 pairs. CPU scaling was enabled, so treat these as local
smoke numbers rather than a final performance claim.

Current candidate-buffer reuse local gates:

```bash
cmake --build build/default/cpp/Release --target test_contact_candidate_set test_continuous_collision_step bm_ipc_candidate_set bm_ipc_motion_aware_candidate_set
./build/default/cpp/Release/bin/test_contact_candidate_set
ctest --test-dir build/default/cpp/Release -R '^(test_contact_candidate_set|test_continuous_collision_step)$' --output-on-failure
./build/default/cpp/Release/bin/bm_ipc_candidate_set --benchmark_min_time=0.05s --benchmark_filter='BM_IpcCandidateSet'
./build/default/cpp/Release/bin/bm_ipc_motion_aware_candidate_set --benchmark_min_time=0.05s --benchmark_filter='BM_IpcMotionAwareCandidateSet'
```

The candidate-buffer gate should prove that reusable-output overloads match the
return-by-value wrappers, reset stale stats/candidates across empty and
topology-changing rebuilds, and preserve candidate vector capacity. Benchmark
evidence should compare return wrappers with reusable-output variants while
stating that temporary sweep-item arrays are still rebuilt per call.

The first local candidate-buffer reuse gate pass on 2026-05-27 passed the
focused target build, 16 `test_contact_candidate_set` cases, the CTest
registration path for `test_contact_candidate_set` and
`test_continuous_collision_step`, and `bm_ipc_candidate_set` plus
`bm_ipc_motion_aware_candidate_set` smoke runs. The motion-aware benchmark
reported reusable sweep timings slightly faster than the return-wrapper path on
the checked falling-point workloads: 16 pairs were about 3.0 us versus 3.4 us,
64 pairs were about 19.1 us versus 20.7 us, and 128 pairs were about 50.6 us
versus 57.4 us. Reusable brute-force swept-AABB timings were about 7.2 us
versus 7.7 us at 16 pairs and about 100.8 us versus 104.8 us at 64 pairs. CPU
scaling was enabled, so treat these as local smoke numbers rather than a final
performance claim.

Current CCD step-bound local gates:

```bash
cmake --build build/default/cpp/Release --target test_continuous_collision_step bm_ipc_continuous_collision_step
./build/default/cpp/Release/bin/test_continuous_collision_step
./build/default/cpp/Release/bin/bm_ipc_continuous_collision_step --benchmark_min_time=0.05s --benchmark_filter='BM_Ipc'
```

The latest local CCD step-bound gate pass on 2026-05-27 passed the focused
target build and 6 `test_continuous_collision_step` cases. The benchmark smoke
reported about 0.20 us for a point-triangle step-bound query, about 0.18 us for
an edge-edge step-bound query, and about 0.13 ms, 1.81 ms, and 9.25 ms for the
falling-patch candidate aggregate at resolutions 8, 16, and 24. CPU scaling was
enabled, so treat these as local smoke numbers rather than a final performance
claim.

Current barrier-kernel local gates:

```bash
cmake --build build/default/cpp/Release --target test_barrier_kernel bm_ipc_barrier_kernel
./build/default/cpp/Release/bin/test_barrier_kernel
./build/default/cpp/Release/bin/bm_ipc_barrier_kernel --benchmark_min_time=0.05s --benchmark_filter='BM_Ipc'
```

The first local barrier-kernel gate pass on 2026-05-27 passed the focused
target build and 7 `test_barrier_kernel` cases. The latest benchmark smoke
reported about 7.8 ns for the active scalar C2 barrier, 1.5 ns for the inactive
scalar path, 557 ns for a point-triangle barrier value/gradient/Hessian query,
286 ns for an edge-edge barrier query, and 810 ns for a mollified edge-edge
barrier query. CPU scaling was enabled, so treat these as local smoke numbers
rather than a final performance claim.

Current tangent-stencil local gates:

```bash
cmake --build build/default/cpp/Release --target test_tangent_stencil bm_ipc_tangent_stencil
./build/default/cpp/Release/bin/test_tangent_stencil
./build/default/cpp/Release/bin/bm_ipc_tangent_stencil --benchmark_min_time=0.05s --benchmark_filter='BM_Ipc'
```
