# PLAN-081: Deformable Implicit-Barrier Solver

- Operating state: `PLAN-081` in [`dashboard.md`](dashboard.md)
- Outcome: the DART 7 `World` can host deformable bodies as a second
  physics domain, stepped by a DART-owned implicit-barrier method inspired by
  Incremental Potential Contact (IPC), without exposing solver registries,
  backend/project names, ECS storage, or execution resources through the public
  facade.
- Current evidence:
  - PR #2700 added native primitive CCD queries (`pointTriangleCcd`,
    `edgeEdgeCcd`) and conservative continuous casts needed by IPC-class
    contact pipelines.
  - PR #2705 provides the rigid-body solver surface, collision bridge, staged
    `WorldStepPipeline`, and the multi-solver architecture document that this
    plan extends.
  - `docs/design/simulation_solver_architecture.md` already defines the
    solver/coupler direction, domain-driven assignment, substep windowing, and
    model/state/control/contact separation.

## Owner Docs

- Architecture rationale:
  [`../design/simulation_solver_architecture.md`](../design/simulation_solver_architecture.md)
- Public facade:
  [`../design/simulation_cpp_api.md`](../design/simulation_cpp_api.md),
  [`../design/simulation_python_api.md`](../design/simulation_python_api.md)
- Research references:
  [`../readthedocs/papers.md`](../readthedocs/papers.md)
- IPC paper/repository gap audit:
  [`081-deformable-implicit-barrier-solver/ipc-paper-gap-audit.md`](081-deformable-implicit-barrier-solver/ipc-paper-gap-audit.md)
  owns the next-session checklist for paper sections, Algorithm 1 phases,
  upstream scene corpus, material/property options, tests, benchmarks,
  profiling, and visual evidence.
- IPC upstream scene corpus manifest:
  [`081-deformable-implicit-barrier-solver/ipc_scene_corpus_manifest.json`](081-deformable-implicit-barrier-solver/ipc_scene_corpus_manifest.json)
  is the machine-checkable source of truth for every audited upstream
  `IPC/input` `.txt` scene path, including symlink aliases, target type,
  expected invariant, profiling requirement, and visual evidence requirement.
- IPC paper figure showcase plan:
  [`081-deformable-implicit-barrier-solver/ipc-paper-figure-showcase.md`](081-deformable-implicit-barrier-solver/ipc-paper-figure-showcase.md)
  enumerates the paper figures and benchmark tables (Fig. 2, 4, 5, 7, 8, 10,
  11, 12, 13, 14, 15, 16, 17, 18, 20, 22, 23, Table 1) that DART must
  reproduce as examples, tests, or benchmark reports, with per-row
  prerequisite kernels, performance targets, and `Status` tracking. Every
  paper-parity slice should advance at least one row toward `landed`.
- Unified Newton-barrier family:
  [`083-unified-newton-barrier-multibody.md`](083-unified-newton-barrier-multibody.md)
  owns the cross-variant plan for sharing IPC, rigid IPC, and ABD distance,
  barrier, CCD, tangent, friction, PSD, projected-Newton, diagnostics, CPU/GPU
  benchmark, and py-demos evidence. PLAN-081 remains the deformable IPC variant
  owner; shared primitives should move only when a second variant needs them.
- SPB self-intersection recovery audit:
  [`081-deformable-implicit-barrier-solver/spb-gap-audit.md`](081-deformable-implicit-barrier-solver/spb-gap-audit.md)
  owns the Shortest Path to Boundary research and implementation sequence for
  tetrahedral self-intersection recovery.
- PD-IPC GPU audit:
  [`081-deformable-implicit-barrier-solver/pd-ipc-gpu-gap-audit.md`](081-deformable-implicit-barrier-solver/pd-ipc-gpu-gap-audit.md)
  owns the Penetration-free Projective Dynamics on the GPU research and
  implementation sequence for GPU-accelerated IPC-class deformable contact.
- Implementation tracking: the first C++ slice is complete in this plan; future
  slices continue from the workstreams below. Because full IPC parity is now a
  multi-session implementation, start a new `docs/dev_tasks/` folder when that
  work begins, then promote durable output and delete it in the completing PR.

## Workstreams

1. **Vertical model/state slice** — Add a minimal public `DeformableBody`
   handle and options value object for point-mass nodes, edges, masses,
   per-body spring/damping parameters, and fixed nodes. Store runtime data in
   internal ECS components; keep components out of dartpy and public user
   docs. The public handle must be world-created and must not expose
   `entt::entity`, `entt::registry`, `comps::*`, execution backends, or solver
   identifiers in public signatures or user-facing docs.
2. **Implicit-barrier step stage** — Add a default deformable dynamics stage
   with algorithm-neutral public names and metadata. Internally it performs one
   deterministic implicit Euler step with line-search feasibility against
   explicitly opted-in static ground geometry. The first slice uses edge
   springs and an analytic point-vs-static-ground barrier so the architecture,
   state updates, and no-crossing semantics are testable before triangle/edge
   contact assembly lands. There is no hidden default ground: users/tests must
   add the public static ground object and opt it into deformable ground-barrier
   behavior.
3. **Optimization contract** — For each deformable body, dynamic variables are
   the non-fixed node positions; fixed nodes are eliminated Dirichlet DOFs. The
   first stage minimizes the mass-weighted implicit-Euler inertia term plus
   edge-spring elastic energy and analytic ground barrier energy. Gravity and
   damping are folded into the inertial target for this first point-mass slice,
   not represented as a separate public force model. Infeasible candidates
   (`distance <= xi`) are rejected by line search. Accepted candidates must be
   feasible and satisfy a deterministic descent/residual stopping rule.
   Velocities are updated from `(x_next - x_prev) / dt`.
4. **IPC paper parity expansion** — Follow the audited paper/repository gap
   matrix in
   [`ipc-paper-gap-audit.md`](081-deformable-implicit-barrier-solver/ipc-paper-gap-audit.md).
   The next implementation session should first add mesh-backed deformables,
   material parameters, scene loading, BE/Newmark integration options,
   DBC/NBC, restart/output diagnostics, and per-step stats, then wire
   point-triangle/edge-edge distance derivatives, conservative CCD line search,
   projected Newton, barrier stiffness adaptation, friction, and the complete
   upstream example/test/benchmark corpus.
5. **IPC-class collision expansion** — Use PR #2700 primitive CCD to assemble
   point-triangle and edge-edge candidate constraints for deformable
   self-contact and deformable-rigid contact. The default runtime path must use
   conservative queries; exact roots remain validation-only. PR #2709's public
   continuous-cast API is useful for reusable user-facing CCD exposure but is
   not a blocker for internal PLAN-081 solver work.
   5a. **SPB self-intersection recovery** — Follow
   [`spb-gap-audit.md`](081-deformable-implicit-barrier-solver/spb-gap-audit.md):
   source/code audit, internal tetrahedral shortest-path-to-boundary query,
   DCD vertex-tet and edge-tet candidates, recovery constraints or penalty
   terms, hybrid CCD/DCD behavior, reduced paper-scene corpus, benchmark JSON,
   and headless visual evidence. Keep SPB internal and tetrahedral-only until
   those gates pass.
   5b. **PD-IPC GPU evaluation** - Follow
   [`pd-ipc-gpu-gap-audit.md`](081-deformable-implicit-barrier-solver/pd-ipc-gpu-gap-audit.md):
   source/code audit, CPU-verifiable two-level projective IPC slice, fast-CCD
   validation against conservative DART CCD, A-Jacobi CPU/GPU prototype,
   patch-based GPU culling, reduced paper-scene corpus, benchmark JSON,
   same-host CPU/GPU packets, and headless visual evidence. Keep PD-IPC
   internal and backend-neutral until those gates pass.
6. **Coupling expansion** — Introduce pairwise rigid/deformable couplers behind
   the solver architecture once contact buffers expose the needed state views.
   Keep common `World::step()` free of coupler or solver vocabulary.
7. **Examples and Python facade** — Add a C++ GUI visual smoke scene once the
   focused C++ behavior tests exist. Bind only the user-facing deformable body
   handle/options and basic state accessors after the C++ slice is stable. Add a
   small dartpy example in a later Python facade slice.
8. **Unified Newton-barrier extraction** — Coordinate with PLAN-083 before
   duplicating any distance, barrier, tangent, CCD, friction, PSD, sparse
   Newton, diagnostics, benchmark, or visual-evidence infrastructure. Deformable
   IPC may keep variant-local code while the second-use contract is unstable,
   but a promoted second use should define an internal shared interface and
   focused cross-variant tests.

## First PR Acceptance Criteria

- The branch is based on PR #2705 so it can use the rigid solver architecture
  and collision bridge.
- `World` supports adding, looking up, counting, and stepping a deformable body
  without requiring solver names, registries, or backend concepts.
- A focused C++ test proves:
  - public `world.step()` / `world.step(count, executor)` execute the
    deformable stage, not only a custom pipeline;
  - fixed node positions and velocities remain unchanged under gravity and
    spring forces;
  - rest-length springs remain stationary without external load;
  - a stretched edge contracts under the implicit spring step;
  - a high-speed falling node added above an explicit static ground descends
    but remains above the internal clearance over repeated large-time-step
    steps;
  - ordinary static collision shapes, including same-footprint boxes or
    spheres, do not become deformable ground unless explicitly opted in;
  - invalid models reject nonpositive mass, negative stiffness/damping, invalid
    edge/fixed-node indices, and nonfinite inputs;
  - repeated runs from identical state and options are deterministic;
  - the deformable stage advertises `ComputeStageDomain::DeformableBody`.
- Public headers are documented and exported; internal storage remains in
  `comps`.
- New focused tests are either placed in an existing registered experimental
  test directory or a new registered test directory is added explicitly.
- `pixi run lint`, `pixi run build`, focused C++ test build/run, and
  `pixi run check-api-boundaries` pass.
- The `experimental_deformable` demos scene renders a long-horizon headless
  Filament frame sequence that visibly exercises fixed anchors, spring
  deformation, and the explicit static ground barrier.
- `bm_deformable_body` records node/edge throughput for spring grids with and
  without the static-ground barrier, a rigid-only/no-deformable baseline, and
  stage-level solver counters so performance changes have a local
  benchmark/profiling surface.

## First C++ Slice Evidence

- Public `DeformableBody` and `DeformableBodyOptions` expose node state,
  topology, masses, fixed nodes, and spring parameters without public ECS or
  solver-registry types.
- `DeformableDynamicsStage` is wired into the default `World::step()` pipeline
  and uses deterministic energy descent with feasibility-rejecting line search
  for explicitly opted-in static-ground barriers while keeping barrier names out
  of the public stage surface.
- `test_deformable_body` covers public stepping, `step(count, executor)`,
  custom step overloads, invalid model rejection, fixed-node invariants, spring
  contraction, mass-scaled free-particle dynamics, finite static-ground
  footprint, active-contact sliding, static-ground barrier feasibility,
  ordinary static collision opt-out behavior, determinism, stage metadata, and
  binary serialization.
- The `experimental_deformable` demos scene can be run with:
  `pixi run demos -- --scene experimental_deformable --headless --frames 180 --width 960 --height 540 --screenshot /tmp/experimental_deformable.ppm`.
  Toggle the `Surface Mesh`, `Point Masses`, and `Spring Edges` checkboxes in
  the scene's panel when recording final visual evidence for renderer or
  example changes.
- Reusable deformable rendering is covered by `UNIT_gui_FilamentSceneExtraction`
  for the `dart::gui` descriptor path.
- `bm_deformable_body` can be run with:
  `pixi run bm bm_deformable_body -- --benchmark_filter='BM_(WorldStepWithoutDeformables|DeformableGridStep|DeformableGridStage|DeformableTetraMeshStep|DeformableSceneLoad|DeformableSceneReplay)' --benchmark_min_time=0.1s`.
- Local gates run for the slice: `pixi run lint`, `pixi run build`, focused
  `test_deformable_body`, `test_deformable_scene_io`, `test_serialization`,
  `UNIT_gui_FilamentSceneExtraction`, and `pixi run check-api-boundaries`.
- This evidence is limited to point masses, springs, an analytic static-ground
  barrier, and render-only deformable surface descriptors. It is not evidence
  for mesh IPC contact, FEM elasticity, projected Newton, CCD line search,
  friction, upstream scene parity, or IPC paper parity.

## IPC Paper Parity Gap

The first C++ slice is intentionally not full IPC. The line-by-line paper audit
and upstream repository/example inventory lives in
[`081-deformable-implicit-barrier-solver/ipc-paper-gap-audit.md`](081-deformable-implicit-barrier-solver/ipc-paper-gap-audit.md).
Treat that file as the next-session checklist for completing the rest of the
method family. In short, the remaining gap is:

- volumetric and surface mesh-backed deformable state, not only point masses;
- density, Young's modulus, Poisson ratio, neo-Hookean and fixed-corotational
  material models;
- implicit Newmark plus the complete backward-Euler incremental potential;
- Dirichlet/Neumann boundary conditions, time ranges, moving collision objects,
  restart/output diagnostics, and material/property scene loading;
- point-triangle and edge-edge distance values, derivatives, tangent bases,
  edge-edge mollifier, and conservative CCD line search;
- sparse projected-Newton solve, local Hessian PSD projection, barrier stiffness
  adaptation, and IPC accuracy diagnostics;
- deformable self-contact, deformable-rigid/codimensional contact, and
  pair-culling broad phase;
- recovery from already self-intersecting tetrahedral states via the SPB sidecar
  if the corpus shows that CCD/barrier-only slices leave unresolved
  penetrations in fast deformable solvers;
- GPU-accelerated projective IPC via the PD-IPC sidecar if CPU IPC, VBD, and OGC
  evidence shows a performance gap that can be closed without weakening DART's
  conservative CCD contract or leaking backend concepts;
- smoothed lagged friction with `epsilon_v`, tangent bases, contact-force
  lagging, and friction convergence diagnostics;
- upstream tutorial, paper, stress, friction, scaling, SQP-comparison, and
  failure-mode visual examples ported to DART-native tests, benchmarks,
  examples, profiling, and long-horizon headless Filament evidence.

## Mesh/Material-State Sub-Slice Evidence

The mesh/material-state sub-slice is a narrow PLAN-081 Slice 1 increment. It
adds optional surface triangles, tetrahedra, material properties, density-based
tetrahedral lumped mass assembly, deterministic boundary-surface extraction,
custom binary serialization, mesh setup/step benchmark counters, and updates
`experimental_deformable_gui` to render body-owned surface topology. The
existing point-mass/spring solver remains the only stepping path.

The sub-slice must not be used as evidence for FEM elasticity, material-driven
stiffness, mesh contact, no-intersection or no-inversion guarantees, CCD line
search, projected Newton, friction, upstream scene parity, or full IPC parity.

## Contact-Free Scene/Boundary Sub-Slice Evidence

The scene/boundary/diagnostics sub-slice is a second narrow PLAN-081 Slice 1
increment. It adds a DART-owned loader for the audited upstream-style text scene
subset, a Gmsh 4.1 tetra-mesh subset importer with boundary-surface fallback
when `$Surface` is absent, generated structural spring edges for contact-free
replay, scripted Dirichlet and Neumann controls with time ranges, binary
restart continuity, compact diagnostics JSON, replay/load benchmark counters,
and `experimental_deformable_gui --deformable-scene` capture in
combined/surface/points modes. Scene gravity follows the imported reference
convention for these replay files; public `DeformableBodyOptions` remain
DART-owned and do not expose upstream solver selectors. Imported IPC metadata
such as `energy` and `timeIntegration` is reported as ignored until the
matching solver slices exist.

The sub-slice must not be used as evidence for FEM elasticity, material-driven
stiffness, upstream ground/contact/friction behavior, mesh self-contact,
intersection or inversion guarantees, CCD line search, projected Newton,
friction, complete scene-corpus coverage, or full IPC parity.

## Non-Goals For The First PR

- Full upstream `ipc-sim/IPC` feature parity.
- Public solver registry, plugin loader, or explicit "IPC" solver selection.
- Hidden default ground or unmodeled collision surfaces.
- Deformable self-contact, triangle/edge contact, FEM tetrahedral elasticity,
  differentiability, batched rollout, or GPU execution. The first PR's
  no-crossing claim applies only to the analytic static-ground barrier; mesh
  self-contact and deformable-rigid contact become IPC-class only when the
  conservative CCD candidate assembly lands.
- Vendoring or linking the upstream IPC repository as a runtime dependency.

## Revision Triggers

- Mesh/material public API shape would expose solver/backend/project names,
  upstream IPC-specific vocabulary, or ECS storage.
- PT/EE distance, conservative CCD, or candidate-filtering APIs cannot be kept
  internal while preserving testability.
- Projected-Newton diagnostics, sparse solver outputs, or barrier/friction
  metrics cannot be serialized or benchmarked without leaking internal solver
  types.
- Long-horizon headless Filament captures reveal that surface, point, or debug
  rendering no longer matches simulated deformation.
- Benchmark/profiling evidence shows the next slice regresses rigid-only worlds
  or the first-slice spring-grid baseline without an accepted tradeoff.
- Maintainers decide to import an external library instead of implementing the
  method in DART-owned code.
- PLAN-083 promotes a shared Newton-barrier primitive or ABI that changes this
  plan's variant-local ownership.

## Progress log

Relocated from the dashboard on 2026-07-03; newest first.

Use the PLAN-081 IPC paper/repository gap audit to implement the
full mesh-backed IPC-class follow-up. Start a dedicated `docs/dev_tasks/`
folder for that multi-session implementation, then work through
mesh/material state, scene loading, BE/Newmark integration, PT/EE distance
derivatives, conservative CCD line search, projected Newton, friction,
diagnostics, and the complete upstream example/test/benchmark/visual corpus.
Track Shortest Path to Boundary as a separate self-intersection recovery
sidecar in
[`081-deformable-implicit-barrier-solver/spb-gap-audit.md`](081-deformable-implicit-barrier-solver/spb-gap-audit.md):
first source/code audit, then a standalone tetrahedral query and DCD recovery
spike before any solver or public API claim.
Track Penetration-free Projective Dynamics on the GPU as a separate
GPU-accelerated IPC sidecar in
[`081-deformable-implicit-barrier-solver/pd-ipc-gpu-gap-audit.md`](081-deformable-implicit-barrier-solver/pd-ipc-gpu-gap-audit.md):
first source/code audit, then a CPU-verifiable projective IPC slice and
fast-CCD validation before any A-Jacobi, GPU-culling, or speedup claim. Use
PLAN-083 to decide which distance, barrier, tangent, CCD, friction, PSD,
sparse-Newton, diagnostics, benchmark, and visual-evidence primitives should
become shared Newton-barrier infrastructure rather than another
deformable-local variant.
