# Resume: IPC Deformable Solver

## Last Session Summary

The branch established a machine-checkable manifest for the audited upstream
`ipc-sim/IPC` scene corpus and added validation tooling for the 154 tracked
`.txt` scene paths. The manifest is a planning/data grounding artifact only; it
does not implement new solver, renderer, benchmark, or GUI behavior.

The follow-up mesh/material-state sub-slice adds optional surface and
tetrahedral topology, material validation, density-based lumped mass assembly
for tetrahedral bodies, boundary surface extraction, serialization, benchmark
counters, and GUI rendering from body-owned surface topology. It still steps
through the existing point-mass/spring path and must not be described as FEM,
mesh contact, or full IPC.

The scene/boundary/diagnostics sub-slice adds a contact-free upstream-style
scene text loader, a Gmsh 4.1 tetra-mesh subset importer, generated structural
spring replay edges, scripted DBC/NBC controls, binary restart continuity,
diagnostics JSON, replay/load benchmarks, and `experimental_deformable_gui
--deformable-scene` headless capture. It still ignores/reports contact and
friction directives and must not be described as IPC scene parity.

The primitive-distance kernel sub-slice starts PLAN-081 Phase 2 with internal
point-triangle and edge-edge squared-distance kernels, closest-feature
classification, gradients, finite-difference Hessians for the first
solver-facing validation contract, IPC-style edge-edge mollifier derivatives,
feature-region regression tests, and `bm_ipc_distance_kernels`. It is still
scaffolding: analytic distance Hessians, tangent bases, candidate-set assembly,
broad phase, CCD line-search bounds, barrier assembly, projected Newton, and
friction are not implemented yet.

## Current Branch

`feature/ipc-deformable-contact-kernels` - stacked on
`feature/ipc-scene-boundary-diagnostics`, adding internal primitive distance
kernels for the next deformable contact slices.

## Immediate Next Step

After this sub-slice lands, continue Phase 2 with analytic distance Hessian
optimization, tangent bases, surface candidate-set assembly, adjacency
exclusion filters, broad-phase versus brute-force tests, and conservative
PT/EE CCD line-search bounds.

## Context That Would Be Lost

- The upstream IPC audit is pinned to commit
  `573d2c7e04104d3f9baf526bdaee7745891a571a`.
- Use `git ls-tree` to enumerate scene paths; `find -type f` misses the 10 SQP
  benchmark symlink aliases.
- The durable row-level source of truth is
  `docs/plans/081-deformable-implicit-barrier-solver/ipc_scene_corpus_manifest.json`.
- The validator must keep zero unclassified family or target-type rows before
  any future IPC parity claim.

## How To Resume

```bash
git checkout feature/ipc-deformable-contact-kernels
git status && git log -3 --oneline
cmake --build build/default/cpp/Release --target test_primitive_distance bm_ipc_distance_kernels
ctest --test-dir build/default/cpp/Release -R '^test_primitive_distance$' --output-on-failure
./build/default/cpp/Release/bin/bm_ipc_distance_kernels --benchmark_min_time=0.05s --benchmark_filter='BM_Ipc'
```

Switch to `feature/ipc-scene-boundary-diagnostics` when reviewing the stacked
scene replay base, `feature/ipc-paper-corpus-manifest` only when updating the
scene corpus manifest itself, or `feature/ipc-mesh-material-state` when
reviewing the stacked mesh/material-state base.
