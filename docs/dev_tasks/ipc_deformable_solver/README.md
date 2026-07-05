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
  - [x] Internal per-body surface-contact CCD line-search sub-slice:
        solver-owned reusable contact candidates, explicit mesh-topology stage
        reads, conservative primitive-CCD uncertainty handling, no-edge fast
        crossing regressions, and deformable-stage benchmark counters.
  - [x] Internal sweep-scratch reuse sub-slice: reusable point/triangle/edge
        sweep-item scratch for static and motion-aware candidate builders,
        per-body world-stage reuse, stale-scratch regressions, and benchmark
        counters for reusable candidate plus reusable sweep buffers.
  - [x] Internal inter-body surface CCD line-search sub-slice: stage-start
        deformable surface snapshots, cross-surface point-triangle and
        edge-edge CCD limiting, order-regression tests, and inter-body
        deformable-stage benchmark counters.
  - [x] Internal static-ground-barrier CCD line-search sub-slice: analytic
        point-mass node sweeps against explicitly opted-in static rigid ground
        barriers, finite-footprint regressions, and benchmark counters.
  - [x] Internal sweep-pair traversal sub-slice: optimized cross-set sorted
        sweep traversal used by experimental surface candidate and CCD checks
        so the expired right-hand-side AABB prefix is no longer rescanned for
        every left-hand-side item.
  - [x] Internal active-set sweep-and-prune sub-slice: completed the cross-set
        sweep into a full active-set traversal (a reusable next-index live list
        that unlinks each right-hand-side AABB as its maximum x falls behind the
        sweep line), removing the prior slice's limitation where a long-lived
        early interval pinned the prefix cursor and forced the expired intervals
        behind it to be rescanned for every left-hand-side item. The visited
        pair sequence is identical to the naive scan (behavior-preserving), with
        a staggered-expiry regression and a long-lived-interval microbenchmark.
  - [x] Internal static rigid surface CCD line-search sub-slice: explicitly
        opted-in static box collision shapes triangulated into stationary
        surface snapshots with physical box edges for deformable line-search
        CCD limiting.
  - [x] Internal static sphere surface CCD sub-slice: opted-in static sphere
        collision shapes tessellated into a UV-sphere triangle mesh that
        conservatively circumscribes the analytic sphere (vertices at a slightly
        inflated radius so every flat face stays outside the true surface),
        reusing the same point-triangle / edge-edge CCD limiter with no new
        primitive. Same `DeformableObstaclePolicy::surfaceObstacle` opt-in
        (untagged shapes unaffected); `staticRigidSurfaceCcdSphereCount` stat
        and opted-in/untagged regressions.
  - [x] Internal moving rigid box surface CCD line-search sub-slice: free
        (non-static) opted-in box obstacles whose end-of-step transform is
        predicted from velocity (mirroring the rigid position integrator that
        runs after the deformable stage) and whose swept motion is tiled by
        overlapping static pose samples, so the deformable cannot settle in or
        tunnel through the obstacle's swept corridor. One-way, timing-agnostic
        conservative limiter with focused regressions and benchmark counters.
  - [ ] Remaining Phase 2 work: further non-box rigid surface coverage
        (arbitrary meshes; deforming/moving spheres), timing-aware
        (non-over-conservative) moving-obstacle CCD, broader solver-wired CCD
        line-search coverage, and stronger spatial acceleration for larger
        meshes. (Static box and sphere obstacles are covered.)
- [ ] Phase 3: clamped barriers, projected Newton, sparse assembly, and solver
      statistics.
  - [x] Self-contact barrier forces sub-slice: the deformable objective now adds
        the IPC clamped-log barrier energy/gradient over the active self-contact
        point-triangle and edge-edge candidate set (assembled per outer iteration
        within d_hat), producing smooth repulsive contact forces so a surface
        folding onto itself settles near d_hat instead of pinning at the CCD
        minimum separation. First contact-FORCE slice (prior contact slices were
        one-way CCD limiters). First-order steepest-descent solve with fixed
        barrier stiffness; the CCD limiters remain the hard no-penetration
        guarantee. Focused regressions + benchmark counters.
  - [x] Projected-Newton sub-slice: per-step Hessian assembly (inertia + spring + self-contact barrier + static ground barrier) with per-element PSD
        projection and a dense LDLT solve for the Newton search direction,
        replacing mass-scaled steepest descent (which remains the fallback for
        large bodies or a failed factorization). Matches the analytic
        implicit-Euler spring solution and converges the stiff barrier in few
        iterations. Focused regressions + solver-step counters.
  - [x] Sparse-solve sub-slice: assemble the per-step Hessian as a sparse
        matrix (Eigen triplets) and factorize with sparse Cholesky
        (`SimplicialLDLT`, positive-diagonal guard), lifting the 256-node dense
        cap to thousands of nodes so paper-scale meshes stay on the Newton path.
        Fixed DOFs pinned at assembly time (unit diagonal, free-free blocks
        only); exact-dense parity preserved. 300-node-chain regression + sparse
        step/fallback benchmark counters (512/1152-node grids).
  - [x] GPU PSD-projection primitive (prototype): an opt-in CUDA sidecar
        (`projectSymmetricBlocksToPsdCuda`, `DART_ENABLE_EXPERIMENTAL_CUDA`)
        batches the per-element symmetric eigendecomposition + eigenvalue clamp
        (the projected-Newton hotspot) on the GPU via per-block cyclic Jacobi,
        with an identical-semantics CPU reference and a CUDA equivalence test.
        Standalone building block only: live-solver wiring needs an optional GPU
        compute-backend injection path (keeping `world_step_stage` GPU-free per
        the runtime-dependency policy), and the formal GPU-vs-CPU perf gate is a
        follow-up.
  - [x] Symbolic-factorization reuse sub-slice: the sparse Cholesky reuses its
        fill-reducing ordering (`analyzePattern`) across iterations/steps when
        the Hessian sparsity pattern is unchanged, so only the numeric
        factorization repeats. Behavior-preserving (structural mismatch
        re-analyzes); ~halves the per-step solve on a settled 512-node grid
        (~21.7->~11.6 ms). Symbolic/numeric counters + reuse regression.
  - [x] Convergence diagnostic: the stage reports `finalGradientResidualNorm`
        (worst-case projected-Newton gradient norm at solve termination across
        the step's bodies), surfaced on the grid/drape benchmarks toward the
        paper's benchmark-statistics tables (Fig. 23 / Table 1).
  - [x] Converged-ness step-norm diagnostic: the stage also reports
        `finalStepInfinityNorm` (largest last accepted per-node step across the
        step's bodies). It shrinks toward zero at a feasible equilibrium even
        when the near-singular barrier Hessian keeps the gradient residual
        large, so it is the honest convergence companion for stiff
        barrier-dominated problems. Behavior-preserving (diagnostic only).
  - [x] Live GPU PSD-backend injection: the projected-Newton assembly now packs
        its per-element spring (6x6) and barrier (12x12) Hessian blocks into one
        batch and projects them through a pluggable backend
        (`compute::projectSymmetricBlocksToPsd`) before scattering. The default
        CPU backend is bit-identical to the previous inline per-element
        projection; the CUDA sidecar can install a GPU backend
        (`installCudaDeformablePsdBackend`) that offloads large batches and
        defers small batches / no-device to the CPU backend, so the offload
        never changes results. `world_step_stage` stays GPU-free (the
        no-GPU-runtime-dependency check passes); the GPU path is validated
        against the CPU backend through the seam in the CUDA test suite.
  - [x] GPU-vs-CPU perf gate: a CUDA test projects 12x12 barrier batches across
        a size sweep, asserting GPU/CPU parity at every scale and logging each
        path's wall time. The measured crossover on an RTX 5000 Ada (~0.4x at
        256 blocks, ~1.4x at 1024, ~4x at 4096, ~9x at 16384) sets the backend
        adapter's minimum GPU batch size (~1024 blocks); smaller batches stay on
        the CPU backend.
  - [x] Resident GPU device buffer: the CUDA PSD backend reuses one persistent
        device allocation that grows on demand (freed when the backend is
        uninstalled) and projects in place on the caller's packed buffer,
        removing the per-iteration `cudaMalloc`/`cudaFree` and the temporary
        host copy. Bit-identical results (only the device storage is reused);
        a CUDA test asserts the buffer is reused across same-or-smaller batches,
        grows once for a larger batch, and is released on restore.
  - [x] Contact closest-approach diagnostic: the stage also reports
        `minActiveContactDistance` (the smallest point-triangle / edge-edge
        distance among the active self-contact barrier set at the converged
        iterate -- the IPC intersection-free "minimum distance" statistic) and
        `convergedActiveContactCount` (the size of that active set at
        termination, a single-iteration snapshot distinct from the cumulative
        `selfContactBarrierActiveContacts`). Behavior-preserving (diagnostic
        only); feeds the Fig. 23 / Table 1 contact statistics.
  - [x] Static sphere obstacle barrier force: a static rigid sphere opted in via
        `DeformableObstaclePolicy::surfaceObstacle` now exerts a full radial
        clamped-log barrier (energy + gradient) that pushes deformable nodes out
        along the outward radial normal -- a 3D contact force, unlike the
        vertical-only ground barrier. Additive (surface-CCD-obstacle spheres were
        previously a no-op; boxes and non-opted scenes unchanged); the surface
        CCD limiter is the tunnelling guard for fast motion. Box obstacles, the
        projected-Newton Hessian, and codimensional obstacles are later
        increments.
  - [ ] Remaining Phase 3 work: a fully resident GPU solve path (the per-batch
        host<->device copies remain; keeping the assembly/solve on-device is a
        follow-up), automatic large-mesh selection for the explicit matrix-free
        CG path, adaptive barrier stiffness, barrier forces for rigid BOX and
        codimensional obstacles (the
        sphere obstacle barrier is covered) plus their projected-Newton Hessian,
        and complementarity/solver-stat diagnostics. Known approximation: the
        contact active set is rebuilt once per outer iteration and held fixed
        across the inner Newton/line-search step (standard IPC), rather than
        re-queried within the line search.
- [~] Phase 4: lagged smoothed friction and friction diagnostics.
  - [x] Static-ground friction (first increment): lagged smoothed Coulomb
        friction (IPC f0/f1 mollifier, velocity threshold epsv) opposing each
        contacting node's tangential displacement, gated by
        `DeformableMaterialProperties.frictionCoefficient` (default 0, additive).
        Lagged normal force per outer iteration; PSD tangential friction Hessian
        in the projected-Newton solve. Sliding-deceleration + no-contact-no-
        friction regressions, drape friction benchmark, serialized coefficient.
  - [x] Self-contact friction (point-triangle): reuses `frictionCoefficient`;
        lagged normal force = barrier force on the point node, tangent
        projection from the point-triangle tangent stencil, same f0/f1 mollifier
        opposing the stencil's tangential relative displacement. Regression: a
        surface sliding on another in self-contact decelerates vs the
        frictionless control while staying separated.
  - [x] Self-contact friction Hessian: a PSD 12x12 block per point-triangle
        contact (`projection^T * H_2x2 * projection`) in the projected-Newton
        assembly, completing self-contact friction as a proper Newton term
        (behavior-preserving; the line search still resolves the same energy).
  - [x] Edge-edge self-contact friction (force + Hessian): the friction
        energy/gradient/Hessian are generic over a four-node stencil, so only
        the lagged-contact assembly is extended to edge-edge barrier candidates
        (net edge force + edge-edge tangent stencil). Crossing-edge slide
        regression. Self-contact friction now covers point-triangle and
        edge-edge.
  - [x] Non-flat ground-normal friction: static-ground friction now lags the
        true geometric surface normal (radial for a sphere, supporting-face
        normal for a rotated/tilted box) and resolves its tangent plane and
        positive-semidefinite 3x3 tangential Hessian against it; flat/box-top
        ground (normal +z) reduces exactly to the previous xy tangent plane.
        Tilted-slope deflection regression; the barrier force itself stays a
        vertical height field.
  - [x] Friction diagnostics: each step reports `frictionDissipation` (the IPC
        Coulomb work mu _ normalForce _ f1(y) \* y summed over active friction
        contacts at the converged iterate) and `activeFrictionContacts` (the
        active static-ground + self-contact friction set size), both zero when
        friction is disabled and computed outside the line-search hot path.
        Sliding-dissipation regression; feeds the Fig-23/Table-1 statistics.
  - [ ] Remaining Phase 4 work: codimensional-obstacle friction, blocked on the
        codimensional-obstacle barrier (a remaining Phase 3 item) -- there is no
        barrier to lag friction against yet.
- [~] Phase 5: complete the upstream scene corpus as DART-native tests,
  examples, benchmarks, profiling artifacts, and headless Filament evidence.
  - [x] Scene-replay validation harness: the loader -> solver -> diagnostics
        pipeline is exercised by a deterministic multi-frame replay regression
        on a DART-native tutorial scene (anchor stays, free body falls, mass
        conserved, reproducible) -- the per-scene invariant pattern corpus rows
        use. Loader, Gmsh import, DBC/NBC, diagnostics JSON, restart, and the
        `experimental_deformable_gui --deformable-scene` headless capture +
        scene-load/replay benchmarks already exist.
  - [ ] Remaining Phase 5 work (BLOCKED on prerequisites): port the 154 upstream
        ipc-sim/IPC corpus scenes (`ipc_scene_corpus_manifest.json`, all
        `planned`). Needs (a) the upstream scene assets vendored or fetched
        (none are vendored today) and (b) the contact-capable solver for the
        contact-heavy scenes (the loader currently ignores contact/friction
        directives; barrier/CCD/friction land via Phases 3-4). Until then only
        contact-free DBC/NBC tutorial-family scenes are replayable.
- [x] Phase 8: Python facade for the deformable-body API (core, topology,
      boundary conditions, scene loader, and diagnostics; binary restart
      deferred pending a Python bytes interface).
  - [x] Core bindings: `dartpy` exposes `World.add_deformable_body` /
        `get_deformable_body` / `has_deformable_body` /
        `get_deformable_body_count`, plus `DeformableBodyOptions`,
        `DeformableMaterialProperties` (incl. `friction_coefficient`),
        `DeformableEdge`, and `DeformableBody` (counts, per-node
        position/velocity get/set, `is_fixed_node`, `edge`,
        `material_properties`). Stubs regenerated; Python regression covers
        create/configure/step/read.
  - [x] Topology bindings: `dartpy` exposes `DeformableSurfaceTriangle` and
        `DeformableTetrahedron`, the `DeformableBodyOptions` `surface_triangles`
        / `tetrahedra` fields, and `DeformableBody` read accessors
        (`surface_triangle_count` / `surface_triangle`, `tetrahedron_count` /
        `tetrahedron` / `tetrahedron_rest_volume`, `node_mass`). Stubs and the
        API boundary inventory regenerated; single-tetrahedron Python
        regression.
  - [x] Boundary-condition bindings: `dartpy` exposes
        `DeformableDirichletBoundaryCondition` and
        `DeformableNeumannBoundaryCondition`, plus the `DeformableBodyOptions`
        `dirichlet_boundary_conditions` / `neumann_boundary_conditions` fields.
        Stubs and the API boundary inventory regenerated; a Python regression
        scripts a Dirichlet node's motion and a Neumann node's acceleration.
  - [x] Scene-loader and diagnostics bindings: `dartpy` exposes
        `load_deformable_scene` / `collect_deformable_scene_diagnostics` and the
        `DeformableSceneLoadOptions` / `DeformableSceneInfo` /
        `DeformableSceneBodyInfo` / `DeformableSceneDiagnostics` structs, so a
        contact-free scene file loads into a `World` from Python and its replay
        diagnostics read back. Stubs and inventory regenerated;
        single-tetrahedron scene Python regression.
  - [ ] Remaining Phase 8 work: none for the contact-free facade; binary restart
        save/load (`std::iostream`-based) is deferred until a Python-friendly
        bytes interface is designed.

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
- No FEM elasticity, material-driven stiffness, general deformable-rigid mesh
  contact, coupled inter-body solve, or projected Newton solve in the current
  point-mass/spring stepping path.
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
- PLAN-083 is the cross-variant Newton-barrier owner. Before adding another
  deformable-local distance, barrier, tangent, CCD, friction, PSD, sparse
  Newton, diagnostics, benchmark, or visual-evidence primitive, check whether it
  should become a shared internal contract with rigid IPC or the planned ABD
  track instead.

## Immediate Next Steps

1. Continue PLAN-081 M7 scale/performance work from the merged
   `feature/ipc-deformable-*` PR train (#2810-#2813). The local diagnostics
   continuation reports successful iterative linear-solve iterations, residual
   estimates, and sparse-Hessian matrix footprint through the public deformable
   solver diagnostics and the M7 direct/CG benchmarks.
2. Continue the matrix-free CG slice: the explicit
   `useMatrixFreeLinearSolver` path now bypasses triplets -> `SparseMatrix`
   assembly with local Hessian-vector products and block-Jacobi preconditioning.
   It is now covered on ground contact by C++ direct/sparse-IC-CG/matrix-free
   parity and dartpy direct/matrix-free regressions. Next, harden it on larger
   contact-heavy meshes and decide when it can become the automatic path for
   Fig. 22-scale meshes.
3. Follow-on performance slices: AMG / multigrid preconditioning for the
   largest systems, on-device GPU assembly + solve beyond the existing PSD
   projection backend, and the Fig. 22 / Table 1 reference-comparison runs.
4. Build the profiling-grade Fig. 23 statistics surface as the solver scales:
   per-scene avg/max contacts, Newton iterations, CG iterations/residuals,
   assembled Hessian footprint, peak memory, seconds per step, and reference
   CPU comparison. A shape-parity scaffold has landed: the peak active-contact
   diagnostic (`maxActiveContactCount`) plus a machine-checkable statistics
   packet (`scripts/write_plan081_deformable_fig23_packet.py` +
   `docs/plans/081-.../fig23_deformable_statistics_corpus.json`, pytest
   `python/tests/unit/test_write_plan081_deformable_fig23_packet.py`) emit the
   Fig-23-shaped axes over DART-runnable scenes (`paper_scale: false`).
   Remaining: process
   peak-memory tracking, and the paper-scale scenes + Table-1 CPU comparison
   (blocked on the M4 upstream asset pipeline).
5. Use the scene corpus manifest to select paper-facing scenes only when their
   prerequisite kernels are present, and attach long-horizon headless Filament
   evidence for every GUI-facing scene rather than committing transient media.

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
wrappers behavior-equivalent. It does not yet cover solver-owned persistent
contact caches, barrier assembly, solver-wired CCD line search, projected
Newton, friction, or scene-level IPC contact behavior.

For the per-body surface-contact CCD line-search sub-slice, keep the
verification language precise: it covers same-body surface point-triangle and
edge-edge motion-aware candidate generation inside `DeformableDynamicsStage`,
primitive CCD status propagation so iteration exhaustion is treated as
uncertainty rather than a certified miss, line-search step shortening before
Armijo evaluation, and zero-step rejection for nodes already inside the
internal separation band. For volumetric bodies, it filters point-triangle CCD
points to nodes referenced by the boundary surface so interior tetrahedral nodes
do not over-limit surface contact. It does not yet cover inter-body contact,
deformable-rigid contact, IPC barrier assembly, projected Newton, friction, or
full scene-level contact behavior.

For the sweep-scratch reuse sub-slice, keep the verification language precise:
it covers reusable internal point/triangle/edge sweep-item arrays for static and
motion-aware candidate builders, clearing stale scratch across empty and
changing topologies, preserving scratch capacity, and per-body reuse in the
same-body surface-contact line search. It does not yet remove the O(n^2)
sweep-pair traversal itself, add deformable-rigid contact, or claim full IPC
contact behavior.

For the sweep-pair traversal sub-slice, keep the verification language precise:
it covers internal cross-set sweep traversal used by experimental surface
candidate generation and inter-body surface CCD checks. Sorted right-hand-side
AABBs whose maximum x coordinate is already behind the current left-hand-side
minimum x are skipped as a prefix instead of rescanned. This was not yet a full
sweep-and-prune active set (a long early right-hand-side interval could still
keep later expired items in the scan); the follow-up active-set sub-slice
removes that limitation. It preserves the existing visitor pair semantics and
does not change public APIs, exact distance filters, topology filters, same-set
edge-edge self traversal, deformable-rigid contact, or full IPC contact
behavior.

For the active-set sweep-and-prune sub-slice, keep the verification language
precise: it replaces the cross-set prefix skip with a full active-set traversal
(a reusable next-index live list whose entries are unlinked once their maximum x
falls behind the sweep line), so expired right-hand-side AABBs behind a
long-lived early interval are no longer rescanned for every left-hand-side item.
It is behavior-preserving: the visited `(lhs, rhs)` pair sequence is identical
to the naive nested scan (verified against the naive reference, including a
staggered-expiry case), and it does not change public APIs, exact distance
filters, topology filters, same-set edge-edge self traversal, candidate sets,
solver-wired CCD limiters, barrier candidate assembly, or full IPC contact
behavior. Benchmark evidence is the long-lived-interval microbenchmark
(`BM_IpcCandidateSetCrossSweepLongLivedInterval`) staying near-linear where the
prefix skip degraded toward quadratic.

For the inter-body surface-contact CCD line-search sub-slice, keep the
verification language precise: it covers cross-surface point-triangle and
edge-edge CCD limiting against stage-start deformable surface snapshots,
including both point-triangle directions and insertion-order regression tests.
Other deformables are treated as stationary obstacle surfaces for the current
body's sequential line search. It does not yet cover a coupled multi-body
Newton solve, barrier forces, deformable-rigid contact, friction, or full
scene-level IPC contact behavior.

For the static-ground-barrier CCD line-search sub-slice, keep the verification
language precise: it covers point-mass deformable nodes sweeping against
existing explicitly opted-in static rigid ground barriers via the current
analytic box/sphere top-height barrier semantics in `DeformableDynamicsStage`.
It shortens line-search trials before objective evaluation and preserves the
`DeformableObstaclePolicy::groundBarrier` opt-in boundary. It does not implement
deformable-rigid contact, rigid collision response, side-face collision, moving
obstacles, mesh/codimensional contact, IPC barrier-force assembly, projected
Newton, adaptive barrier stiffness, friction, no-intersection/no-inversion
guarantees, Python bindings, solver selection, or full IPC paper parity.
Ordinary static collision shapes remain ignored unless opted in as deformable
ground barriers.

For the static rigid surface CCD line-search sub-slice, keep the verification
language precise: it covers explicitly opted-in static box collision shapes
collected at `DeformableDynamicsStage` start and triangulated into stationary
world-space surface snapshots with 12 physical box edges. The deformable
line-search limiter reuses the primitive point-triangle and edge-edge CCD
reducers for point-only deformables, deformable surface edges, rotated boxes,
and stage-start rigid transforms. It preserves the
`DeformableObstaclePolicy::surfaceObstacle` opt-in boundary and ignores ordinary
static boxes, non-static bodies, and non-box collision shapes. It does not
implement rigid collision response, moving obstacle contact, arbitrary mesh
obstacles, barrier/contact forces, friction, projected Newton, adaptive
barrier stiffness, no-intersection/no-inversion guarantees, solver selection,
or full IPC paper parity.

For the moving rigid box surface CCD line-search sub-slice, keep the
verification language precise: it covers free (non-static) opted-in box
obstacles collected at `DeformableDynamicsStage` start, whose end-of-step
transform is predicted from `comps::Velocity` using the exact
`integrateRigidBodyPosition` formula (so the prediction matches the pose
`RigidBodyPositionStage` applies after the deformable stage), and whose swept
motion is tiled by overlapping static pose samples spaced at most one box
min-half-extent apart (capped at 64 samples). The deformable line-search
limiter reuses the static-pose point-triangle and edge-edge CCD reducers over
those samples. The moving collector is disjoint from the static one
(`entt::exclude<StaticBodyTag>`), so a body is never limited or counted twice.
It is a one-way, timing-agnostic conservative limiter: it never misses a
penetration but is more conservative than a timing-aware sweep for fast
obstacles, and it does not implement rigid collision response, two-way
coupling, contact forces, friction, projected Newton, arbitrary mesh
obstacles, kinematically scripted (velocity-free `setTransform`) obstacles,
or full IPC paper parity. The sample count is capped (64); past the cap the
sampled boxes are isotropically inflated to keep consecutive samples
overlapping, so the swept corridor stays covered (more over-conservative)
rather than opening tunnelable gaps. Coverage is exact along the box min axis
for axis-aligned motion; diagonal motion leaves a bounded, half-extent-scale
thin-corner under-coverage inherent to box supersampling.

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

Current per-body surface-contact CCD line-search local gates:

```bash
cmake --build build/default/cpp/Release --target test_primitive_ccd test_continuous_collision_step test_deformable_body bm_deformable_body
./build/default/cpp/Release/bin/test_primitive_ccd --gtest_filter='PointTriangleCcd.*'
./build/default/cpp/Release/bin/test_continuous_collision_step
./build/default/cpp/Release/bin/test_deformable_body --gtest_filter='DeformableBody.SurfaceContactCcd*:DeformableBody.SurfaceFreeParticlesKeepFastPath:DeformableBody.SurfaceContactScratchIsPerBody:DeformableBody.StageMetadataUsesDeformableDomain'
./build/default/cpp/Release/bin/bm_deformable_body --benchmark_min_time=0.03s --benchmark_filter='BM_DeformableSurfaceContactStage'
```

The surface-contact CCD gate should prove that a surface-only body with no
springs and no static ground no longer tunnels through its own surface topology,
that volumetric interior nodes are not treated as surface contact points, that
primitive CCD iteration exhaustion is visible as indeterminate status, that
custom-stage stats report candidate builds and CCD checks/hits/limited trials,
that initial separation-band hits reject the trial, and that bodies without
surface topology keep the free-particle fast path.

The first local surface-contact CCD line-search gate pass on 2026-05-27 passed
the focused target build, 10 point-triangle primitive CCD tests, 7
`test_continuous_collision_step` cases, and 6 focused `test_deformable_body`
cases. The focused benchmark smoke reported about 0.57 us for a one-pair
no-contact surface stage, about 5.1 us for a one-pair crossing stage, and about
0.16 ms for 32 crossing pairs. The crossing cases reported nonzero candidate
build, point-triangle candidate, CCD hit, and CCD-limited-step counters. CPU
scaling was enabled, so treat these as local smoke numbers rather than a final
performance claim.

The Codex-review follow-up on 2026-05-27 added the volumetric interior-node
filter regression and passed `DeformableBody.SurfaceContactCcd*` with 4 focused
tests after rebuilding `test_deformable_body` and `bm_deformable_body`.

Current sweep-scratch reuse local gates:

```bash
cmake --build build/default/cpp/Release --target test_contact_candidate_set test_deformable_body bm_ipc_candidate_set bm_ipc_motion_aware_candidate_set bm_deformable_body
./build/default/cpp/Release/bin/test_contact_candidate_set --gtest_filter='IpcContactCandidateSet.*Reusable*:IpcContactCandidateSet.*SweepScratch*'
./build/default/cpp/Release/bin/test_deformable_body --gtest_filter='DeformableBody.SurfaceContactCcd*:DeformableBody.SurfaceFreeParticlesKeepFastPath:DeformableBody.SurfaceContactScratchIsPerBody:DeformableBody.StageMetadataUsesDeformableDomain'
./build/default/cpp/Release/bin/bm_ipc_candidate_set --benchmark_min_time=0.03s --benchmark_filter='BM_IpcCandidateSet(Reusable|Scratch)?Sweep'
./build/default/cpp/Release/bin/bm_ipc_motion_aware_candidate_set --benchmark_min_time=0.03s --benchmark_filter='BM_IpcMotionAwareCandidateSet(Reusable|Scratch)?Sweep'
./build/default/cpp/Release/bin/bm_deformable_body --benchmark_min_time=0.03s --benchmark_filter='BM_DeformableSurfaceContactStage'
```

The sweep-scratch reuse gate should prove that the scratch overloads match the
fresh return wrappers, that stale scratch does not leak across dense, empty, and
smaller topologies, that capacities are retained for reuse, and that the
world-stage surface-contact benchmark still reports the same CCD/candidate
activity after moving the line search onto per-body sweep scratch.

The latest local sweep-scratch gate pass on 2026-05-27 passed the focused target
build, 6 filtered `test_contact_candidate_set` cases, and 7 filtered
`test_deformable_body` cases. The benchmark smoke reported representative
static sweep timings of about 0.30 ms, 0.37 ms, and 0.37 ms for return-wrapper,
reusable-candidate, and reusable-candidate-plus-scratch cloth resolution 16;
about 80 us, 81 us, and 78 us for tetra-surface resolution 8; motion-aware
falling-points resolution 128 at about 48.2 us, 48.6 us, and 44.2 us; and
coherent-translation resolution 64 at about 25.8 us for the return wrapper and
22.0 us for reusable scratch. The deformable surface-contact stage smoke
reported about 0.44 us for a one-pair no-contact stage, about 3.5 us for a
one-pair crossing stage, and about 0.13 ms for 32 crossing pairs with matching
candidate/CCD counters. CPU scaling was enabled, so treat these as local smoke
numbers rather than final performance claims. `perf stat` could not run because
`perf_event_paranoid=4`; `/usr/bin/time -v` on the motion-aware 128-pair
reusable-versus-scratch benchmark reported 0.12 s elapsed, 98% CPU, 4400 KB max
RSS, 0 major page faults, and 380 minor page faults.

Current sweep-pair traversal local gates:

```bash
cmake --build build/default/cpp/Release --target test_contact_candidate_set test_continuous_collision_step test_deformable_body bm_ipc_candidate_set bm_ipc_motion_aware_candidate_set bm_deformable_body
./build/default/cpp/Release/bin/test_contact_candidate_set --gtest_filter='IpcContactCandidateSet.VisitSweepPairsMatchesNaiveReference:IpcContactCandidateSet.*Sweep*:IpcContactCandidateSet.*Reusable*:IpcContactCandidateSet.MotionAware*'
./build/default/cpp/Release/bin/test_deformable_body --gtest_filter='DeformableBody.SurfaceContactCcd*:DeformableBody.InterBodySurfaceContactCcd*:DeformableBody.SurfaceFreeParticlesKeepFastPath:DeformableBody.SurfaceContactScratchIsPerBody:DeformableBody.StageMetadataUsesDeformableDomain'
ctest --test-dir build/default/cpp/Release -R '^(test_contact_candidate_set|test_continuous_collision_step)$' --output-on-failure
./build/default/cpp/Release/bin/bm_ipc_candidate_set --benchmark_min_time=0.03s --benchmark_filter='BM_IpcCandidateSet(CrossSweepExpiredPrefix|(Scratch|Reusable)?Sweep)'
./build/default/cpp/Release/bin/bm_ipc_motion_aware_candidate_set --benchmark_min_time=0.03s --benchmark_filter='BM_IpcMotionAwareCandidateSet(Scratch|Reusable)?Sweep'
./build/default/cpp/Release/bin/bm_deformable_body --benchmark_min_time=0.03s --benchmark_filter='BM_Deformable(SurfaceContactStage|InterBodySurfaceContactStage)'
```

The sweep-pair traversal gate should prove that optimized cross-set traversal
still matches the naive helper reference, still matches brute-force/static and
motion-aware candidate assembly, exercises the expired-prefix microbenchmark,
preserves reusable builder behavior, and leaves solver-wired surface/inter-body
CCD counters unchanged.

The latest local sweep-pair traversal gate pass on 2026-05-27 passed the
focused target build, 13 filtered `test_contact_candidate_set` cases, 11
filtered `test_deformable_body` cases, and the
`test_contact_candidate_set`/`test_continuous_collision_step` ctest subset.
Representative benchmark smoke numbers included the direct expired-prefix
cross-sweep case at about 1.1 us, 7.0 us, and 31.0 us for 64, 256, and 1024
left/right items; static cloth resolution 16 at about 328 us, 312 us, and 315 us
for return-wrapper, reusable-candidate, and reusable-candidate-plus-scratch
paths; tetra-surface resolution 8 at about 50 us, 47 us, and 58 us;
motion-aware falling-points resolution 128 at about 19.7 us, 20.3 us, and
21.2 us; and coherent translation resolution 64 at about 16.7 us for the return
path and 16.6 us for reusable scratch. The deformable surface/inter-body stage
smokes preserved the expected candidate, CCD-hit, limited-step, and line-search
counters. CPU scaling was enabled, so treat these as local smoke numbers rather
than final performance claims.

Current inter-body surface-contact CCD local gates:

```bash
cmake --build build/default/cpp/Release --target test_deformable_body bm_deformable_body
./build/default/cpp/Release/bin/test_deformable_body --gtest_filter='DeformableBody.InterBodySurfaceContactCcd*:DeformableBody.SurfaceContactCcd*'
./build/default/cpp/Release/bin/bm_deformable_body --benchmark_min_time=0.03s --benchmark_filter='BM_DeformableInterBodySurfaceContactStage'
```

The inter-body surface-contact CCD gate should prove that the current body's
line search is limited by another deformable body's stage-start surface
snapshot for moving-point versus stationary-triangle, moving-triangle versus
stationary-point, and moving-edge versus stationary-edge cases. It should also
prove insertion-order stability for the stationary-obstacle slice and keep
same-body candidates out of the explicit inter-body stats.

The latest local inter-body gate pass on 2026-05-27 passed the focused target
build and 8 filtered `test_deformable_body` cases covering same-body and
inter-body CCD. The inter-body benchmark smoke reported about 0.87 us for a
one-obstacle no-contact stage, about 6.1 us for a one-obstacle crossing stage,
about 0.11 ms for 8 crossing obstacles, and about 1.59 ms for 32 crossing
obstacles. The crossing cases reported nonzero inter-body point-triangle
candidates, CCD hits, and CCD-limited-step counters. CPU scaling was enabled,
so treat these as local smoke numbers rather than final performance claims.

Current static-ground-barrier CCD local gates:

```bash
cmake --build build/default/cpp/Release --target test_deformable_body bm_deformable_body
./build/default/cpp/Release/bin/test_deformable_body --gtest_filter='DeformableBody.StaticGroundBarrier*:DeformableBody.ActiveStaticGroundContactAllowsTangentialMotion:DeformableBody.ActiveDirichletNodesDoNotBlockGroundBarrierSolve:DeformableBody.StaticCollisionRequiresGroundBarrierOptIn'
./build/default/cpp/Release/bin/bm_deformable_body --benchmark_min_time=0.03s --benchmark_filter='BM_DeformableStaticGroundBarrierCcdStage'
```

The static-ground-barrier CCD gate should prove that fast vertical node sweeps
are limited against flat box and sphere top surfaces, that finite-footprint
fly-through paths are limited before entering an opted-in barrier footprint,
that ordinary untagged static collision shapes remain ignored, and that fixed
nodes do not contribute to ground CCD stats.

The latest local static-ground gate pass on 2026-05-27 passed the focused
target build and 13 filtered `test_deformable_body` cases covering existing
static-ground behavior, active tangential contact, boundary-condition contact,
opt-in semantics, new box/sphere/fly-through/narrow-offset CCD regressions, and
fixed-node skipping. The benchmark smoke reported about 0.52 us for a
one-barrier no-contact stage, about 19 us for one crossing barrier, about
0.43 ms for 8 crossing barriers, and about 4.95 ms for 32 crossing barriers. The
vertical fast path reported two clearance samples per node check; crossing
cases reported nonzero ground CCD hits and limited-step counters. CPU scaling
was enabled, so treat these as local smoke numbers rather than final
performance claims.

Current static rigid surface CCD local gates:

```bash
cmake --build build/default/cpp/Release --target test_deformable_body test_serialization bm_deformable_body dartpy
./build/default/cpp/Release/bin/test_deformable_body --gtest_filter='DeformableBody.StaticRigidSurfaceCcd*:DeformableBody.SurfaceFreeParticlesKeepFastPath:DeformableBody.StaticGroundBarrier*:DeformableBody.InterBodySurfaceContactCcd*'
./build/default/cpp/Release/bin/test_serialization --gtest_filter='Serialization.PreservesRigidBodyCollisionComponents'
PYTHONPATH=build/default/cpp/Release/python ./.pixi/envs/default/bin/python -m pytest python/tests/unit/simulation/test_experimental_world.py -k 'collision_query'
./build/default/cpp/Release/bin/bm_deformable_body --benchmark_min_time=0.03s --benchmark_filter='BM_DeformableStaticRigidSurfaceCcdStage'
```

The static rigid surface CCD gate should prove that point-only deformables do
not stay on the no-contact fast path when an opted-in static box surface can be
crossed, that ordinary untagged static boxes remain ignored, that physical box
edges rather than triangulation diagonals drive edge-edge checks, that
stage-start rigid transforms are used, and that rotated box sides report
nonzero CCD hit and limited-step counters.

The latest local static rigid surface gate pass on 2026-05-27 passed the
focused target build, 20 filtered `test_deformable_body` cases covering
existing ground/inter-body paths and the new point-only, opt-in, edge-edge,
stage-start transform, and rotated-box regressions, the rigid-body collision
serialization test, and the dartpy collision-query subset. The benchmark smoke
reported about 0.38 us for no rigid-surface obstacles, 0.84 us for one
non-crossing static box, 8.6 us for one crossing static box, 0.080 ms for 8
crossing static boxes, and 0.47 ms for 32 crossing static boxes, with expected
box/triangle/edge counts and nonzero point-triangle CCD hit and limited-step
counters in crossing cases. CPU scaling was enabled, so treat these as local
smoke numbers rather than final performance claims.

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
