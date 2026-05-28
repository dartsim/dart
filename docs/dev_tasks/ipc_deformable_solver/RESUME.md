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
classification, gradients, the first solver-facing Hessian contract, IPC-style
edge-edge mollifier derivatives, feature-region regression tests, and
`bm_ipc_distance_kernels`.

The analytic-Hessian optimization sub-slice replaces the finite-difference
distance Hessian placeholder with feature-wise exact point-triangle and
edge-edge Hessian paths for vertex, edge, face, and interior closest features.
It is still scaffolding: tangent bases, solver-wired CCD line search, barrier
assembly, projected Newton, and friction are not implemented yet.

The barrier-kernel sub-slice adds internal IPC C2 clamped-log scalar barriers
over squared distances, raw analytic point-triangle and edge-edge barrier
gradients/Hessians through the distance kernels, and explicit-threshold
edge-edge mollifier product-rule derivatives. It is still scaffolding: barrier
stiffness adaptation, PSD projection, candidate-buffer assembly, solver-wired
CCD line search, projected Newton, and friction are not implemented yet.

The tangent-stencil sub-slice adds internal point-triangle, edge-edge,
point-edge, and point-point tangent bases, closest-point parameters, tangent
projection matrices, and tangent metric matrices for future IPC
contact/friction assembly. It is still scaffolding: friction energy,
friction gradients/Hessians, lagged friction convergence, solver-owned contact
caches, solver-wired CCD line search, projected Newton, and scene-level contact
behavior are not implemented yet.

The motion-aware candidate-culling sub-slice adds conservative start/end
swept-AABB point-triangle and edge-edge candidate assembly, preserving fast
crossings that static endpoint distance filters miss, and stores minimum
endpoint squared distance as representative metadata. It is still scaffolding:
the swept candidates are not wired into `World::step()`, solver-owned contact
buffers, barrier assembly, projected Newton, or friction.

The candidate-buffer reuse sub-slice adds reusable-output overloads for static
and motion-aware candidate builders. The overloads clear stale candidates and
stats while preserving candidate vector capacity, and the return-by-value
builders remain wrappers. It is still scaffolding: temporary sweep-item arrays
are rebuilt per call, and the buffers are not yet owned by `World::step()` or a
persistent IPC contact cache.

The per-body surface-contact CCD line-search sub-slice wires motion-aware
same-body surface candidates into `DeformableDynamicsStage` for conservative
line-search limiting. It adds primitive CCD status propagation so iteration
exhaustion is treated as uncertainty, uses per-body reusable contact candidate
scratch, disables the no-edge/no-ground fast path when surface contact topology
exists, shortens accepted trials before Armijo evaluation, and rejects zero-step
initial separation-band hits. For volumetric bodies, the point-triangle CCD
candidate path is filtered to boundary-surface nodes so interior tetrahedral
nodes do not over-limit surface contact. It is still scaffolding:
inter-body contact, deformable-rigid contact, barrier assembly, projected
Newton, and friction are not implemented yet.

The sweep-scratch reuse sub-slice adds reusable internal point/triangle/edge
sweep-item arrays for static and motion-aware contact candidate sweep builders.
The world-stage same-body surface-contact line search owns this scratch per
body, alongside the reusable candidate set. This removes per-trial sweep-vector
allocation from the CCD line-search path while preserving the existing
return-by-value and reusable-candidate overloads as wrappers. It is still
scaffolding: sweep-pair traversal remains simple O(n^2) over sorted active
AABBs, and inter-body/deformable-rigid contact, barrier assembly, projected
Newton, and friction are not implemented yet.

The inter-body surface-contact CCD sub-slice adds stage-start deformable
surface snapshots and a cross-surface point-triangle/edge-edge CCD limiter
inside `DeformableDynamicsStage`. It covers moving-point versus
stationary-triangle, moving-triangle versus stationary-point, moving-edge
versus stationary-edge, and insertion-order regressions. It is still
scaffolding: other bodies are stationary obstacles for the current sequential
line search, not a coupled multi-body Newton solve, and deformable-rigid
contact, barrier forces, projected Newton, and friction are not implemented yet.

The candidate-set sub-slice adds deterministic unique surface-edge extraction,
internal point-triangle and edge-edge primitive candidate assembly,
incident/adjacent filtering, exact activation-distance filtering through the
primitive distance kernels, sweep-versus-brute-force regression tests, and
`bm_ipc_candidate_set`. It is still scaffolding: candidates are not wired into
`World::step()`, conservative CCD, barrier assembly, projected Newton, or
friction.

The CCD step-bound sub-slice adds conservative internal point-triangle and
edge-edge normalized step bounds through native primitive CCD, initial
separation-band handling, deterministic minimum aggregation over assembled
candidate sets, exact-CCD regression checks where available, sampled safety
checks before the returned bound, and `bm_ipc_continuous_collision_step`. It is
still scaffolding: the bounds are not wired into `World::step()`, motion-aware
candidate culling, barrier assembly, projected Newton, or friction.

## Current Branch

`feature/ipc-interbody-surface-ccd` - stacked on
`feature/ipc-sweep-scratch`, adding a conservative inter-body surface CCD
line-search limiter.

## Immediate Next Step

After this sub-slice lands, continue Phase 2 with deformable-rigid contact
candidates, broader solver-wired CCD coverage, and allocation-free sweep-pair
traversal for larger meshes.

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
git checkout feature/ipc-ccd-line-search
git status && git log -3 --oneline
cmake --build build/default/cpp/Release --target test_primitive_distance bm_ipc_distance_kernels
ctest --test-dir build/default/cpp/Release -R '^test_primitive_distance$' --output-on-failure
./build/default/cpp/Release/bin/bm_ipc_distance_kernels --benchmark_min_time=0.05s --benchmark_filter='BM_Ipc'
cmake --build build/default/cpp/Release --target test_contact_candidate_set bm_ipc_candidate_set
./build/default/cpp/Release/bin/test_contact_candidate_set
./build/default/cpp/Release/bin/bm_ipc_candidate_set --benchmark_min_time=0.05s --benchmark_filter='BM_IpcCandidateSet'
cmake --build build/default/cpp/Release --target test_continuous_collision_step bm_ipc_continuous_collision_step
./build/default/cpp/Release/bin/test_continuous_collision_step
./build/default/cpp/Release/bin/bm_ipc_continuous_collision_step --benchmark_min_time=0.05s --benchmark_filter='BM_Ipc'
cmake --build build/default/cpp/Release --target test_barrier_kernel bm_ipc_barrier_kernel
./build/default/cpp/Release/bin/test_barrier_kernel
./build/default/cpp/Release/bin/bm_ipc_barrier_kernel --benchmark_min_time=0.05s --benchmark_filter='BM_Ipc'
cmake --build build/default/cpp/Release --target test_tangent_stencil bm_ipc_tangent_stencil
./build/default/cpp/Release/bin/test_tangent_stencil
./build/default/cpp/Release/bin/bm_ipc_tangent_stencil --benchmark_min_time=0.05s --benchmark_filter='BM_Ipc'
cmake --build build/default/cpp/Release --target test_contact_candidate_set test_continuous_collision_step bm_ipc_motion_aware_candidate_set
./build/default/cpp/Release/bin/test_contact_candidate_set
ctest --test-dir build/default/cpp/Release -R '^(test_contact_candidate_set|test_continuous_collision_step)$' --output-on-failure
./build/default/cpp/Release/bin/bm_ipc_motion_aware_candidate_set --benchmark_min_time=0.05s --benchmark_filter='BM_IpcMotionAwareCandidateSet'
cmake --build build/default/cpp/Release --target test_contact_candidate_set test_continuous_collision_step bm_ipc_candidate_set bm_ipc_motion_aware_candidate_set
./build/default/cpp/Release/bin/bm_ipc_candidate_set --benchmark_min_time=0.05s --benchmark_filter='BM_IpcCandidateSet'
./build/default/cpp/Release/bin/bm_ipc_motion_aware_candidate_set --benchmark_min_time=0.05s --benchmark_filter='BM_IpcMotionAwareCandidateSet'
cmake --build build/default/cpp/Release --target test_primitive_ccd test_continuous_collision_step test_deformable_body bm_deformable_body
./build/default/cpp/Release/bin/test_primitive_ccd --gtest_filter='PointTriangleCcd.*'
./build/default/cpp/Release/bin/test_deformable_body --gtest_filter='DeformableBody.SurfaceContactCcd*:DeformableBody.SurfaceFreeParticlesKeepFastPath:DeformableBody.SurfaceContactScratchIsPerBody:DeformableBody.StageMetadataUsesDeformableDomain'
./build/default/cpp/Release/bin/bm_deformable_body --benchmark_min_time=0.03s --benchmark_filter='BM_DeformableSurfaceContactStage'
cmake --build build/default/cpp/Release --target test_contact_candidate_set bm_ipc_candidate_set bm_ipc_motion_aware_candidate_set
./build/default/cpp/Release/bin/test_contact_candidate_set --gtest_filter='IpcContactCandidateSet.*Reusable*:IpcContactCandidateSet.*SweepScratch*'
./build/default/cpp/Release/bin/bm_ipc_candidate_set --benchmark_min_time=0.03s --benchmark_filter='BM_IpcCandidateSet(Reusable|Scratch)?Sweep'
./build/default/cpp/Release/bin/bm_ipc_motion_aware_candidate_set --benchmark_min_time=0.03s --benchmark_filter='BM_IpcMotionAwareCandidateSet(Reusable|Scratch)?Sweep'
cmake --build build/default/cpp/Release --target test_deformable_body bm_deformable_body
./build/default/cpp/Release/bin/test_deformable_body --gtest_filter='DeformableBody.InterBodySurfaceContactCcd*:DeformableBody.SurfaceContactCcd*'
./build/default/cpp/Release/bin/bm_deformable_body --benchmark_min_time=0.03s --benchmark_filter='BM_DeformableInterBodySurfaceContactStage'
```

Switch to `feature/ipc-scene-boundary-diagnostics` when reviewing the stacked
scene replay base, `feature/ipc-deformable-contact-kernels` for the candidate
set/CCD base, `feature/ipc-barrier-kernels` for clamped barrier scaffolding,
`feature/ipc-tangent-stencils` for tangent scaffolding,
`feature/ipc-motion-aware-candidates` for swept candidate culling,
`feature/ipc-candidate-buffer-reuse` for reusable candidate buffers, or
`feature/ipc-ccd-line-search`, `feature/ipc-sweep-scratch`, or
`feature/ipc-interbody-surface-ccd` for the immediate stacked bases.
