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
        gradients, finite-difference Hessians for the first solver-facing
        contract, IPC-style edge-edge mollifier threshold/value/gradient/
        Hessian, finite-difference regression tests, and
        `bm_ipc_distance_kernels`.
  - [x] Internal candidate-set sub-slice: deterministic unique surface-edge
        extraction, point-triangle and edge-edge primitive candidate assembly,
        incident/adjacent exclusion filters, exact activation-distance filtering
        through the primitive distance kernels, sweep-versus-brute-force
        regression tests, and `bm_ipc_candidate_set`.
  - [ ] Remaining Phase 2 work: analytic distance Hessians, tangent bases,
        conservative PT/EE CCD step bounds, barrier/candidate integration, and
        solver-owned contact buffers.
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
- No FEM elasticity, material-driven stiffness, CCD line search, or projected
  Newton solve in the current point-mass/spring stepping path.
- No vendored or runtime dependency on `ipc-sim/IPC`.

## Key Decisions

- The durable scene inventory lives in
  [`../../plans/081-deformable-implicit-barrier-solver/ipc_scene_corpus_manifest.json`](../../plans/081-deformable-implicit-barrier-solver/ipc_scene_corpus_manifest.json).
  This dev task links to it instead of duplicating row-level state.
- The manifest uses upstream commit
  `573d2c7e04104d3f9baf526bdaee7745891a571a` and tracks 154 `.txt` scene paths:
  144 regular scene files plus 10 symlink aliases under the SQP benchmark
  corpus.
- Every upstream scene path has a non-`unclassified` family and a planned DART
  target type: `test`, `benchmark`, `example`, `manual`, or `not-applicable`.
- Public DART APIs should keep DART-owned names and avoid exposing upstream
  IPC solver vocabulary as user-facing solver selectors.

## Immediate Next Steps

1. Continue Phase 2 with conservative CCD line-search bounds, analytic distance
   Hessian optimization, tangent bases, barrier/candidate integration, and
   solver-owned contact buffers. The current primitive kernels and candidate
   sets are internal scaffolding only and are not yet wired into `World::step()`.
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
classification, gradients, finite-difference Hessians, the edge-edge mollifier,
finite-difference derivative tests, and microbenchmark timings. It does not yet
cover analytic distance Hessians, tangent bases, broad-phase candidate assembly,
CCD line search, barrier assembly, projected Newton, friction, or scene-level
IPC contact behavior.

For the candidate-set sub-slice, keep the verification language precise: it
covers deterministic surface-edge extraction, internal point-triangle and
edge-edge candidate assembly, incident/adjacent filtering, exact
activation-distance filtering through the primitive distance kernels,
sweep-versus-brute-force regression tests, and benchmark counters. It does not
yet cover conservative CCD bounds, barrier assembly, solver integration,
projected Newton, friction, persistent contact caches, tangent bases, or
scene-level IPC contact behavior.

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
`pixi run check-api-boundaries`, and `pixi run check-lint`. The benchmark smoke
reported roughly 8-16 ns for value/gradient distance paths, 228 ns for the
analytic edge-edge mollifier Hessian path, and about 0.59-0.61 us for the
finite-difference distance Hessian placeholders. Treat those Hessian numbers as
a temporary optimization target, not as final IPC performance evidence.

Current candidate-set local gates:

```bash
cmake --build build/default/cpp/Release --target test_contact_candidate_set bm_ipc_candidate_set
./build/default/cpp/Release/bin/test_contact_candidate_set
./build/default/cpp/Release/bin/bm_ipc_candidate_set --benchmark_min_time=0.05s --benchmark_filter='BM_IpcCandidateSet'
```

The latest local candidate-set gate pass on 2026-05-27 passed the focused
target build and 6 `test_contact_candidate_set` cases. The benchmark smoke
reported the sweep path faster than brute force on the checked workloads: cloth
8x8 was about 37 us versus 358 us, cloth 16x16 was about 0.42 ms versus
5.20 ms, tetra-surface 4x4 was about 9.7 us versus 81 us, and tetra-surface
8x8 was about 87 us versus 1.45 ms. CPU scaling was enabled, so treat these as
local smoke numbers rather than a final performance claim.
