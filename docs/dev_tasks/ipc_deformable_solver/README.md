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
  - [ ] Remaining Phase 1 work: scene loading, DBC/NBC, scripted motion,
        restart/output diagnostics, and contact-free mesh scene replays.
- [ ] Phase 2: PT/EE distance kernels, broad-phase candidates, and conservative
      CCD line-search bounds.
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

## Non-Goals For The Current Manifest Slice

- No new solver behavior, renderer behavior, tests, benchmarks, or GUI examples.
- No claim that DART has full IPC, mesh contact, projected Newton, friction, or
  paper-level parity.
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

1. Finish the rest of Slice 1 from PLAN-081: scene asset loading, BE/NM state,
   DBC/NBC, scripted motion, restart, output diagnostics, and contact-free mesh
   replays. The mesh/material-state sub-slice is scaffolding only and still uses
   the existing point-mass/spring stepping path.
2. Use the scene corpus manifest to select the first tutorial and paper-facing
   scenes, then replace planned artifacts with implemented DART commands as
   each scene lands.
3. Add focused unit tests and benchmarks for each landed slice before promoting
   scene-level examples.
4. For every GUI-facing scene, attach long-horizon headless Filament evidence
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
