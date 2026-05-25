# PLAN-081: Deformable Implicit-Barrier Solver

- Operating state: `PLAN-081` in [`dashboard.md`](dashboard.md)
- Outcome: the experimental `World` can host deformable bodies as a second
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
  [`../design/simulation_experimental_cpp_api.md`](../design/simulation_experimental_cpp_api.md),
  [`../design/simulation_experimental_python_api.md`](../design/simulation_experimental_python_api.md)
- Research references:
  [`../design/simulation_experimental_references.md`](../design/simulation_experimental_references.md)
- Implementation tracking: the first C++ slice is complete in this plan; future
  slices continue from the workstreams below without keeping a completed
  `docs/dev_tasks/` folder alive.

## Workstreams

1. **Vertical model/state slice** — Add a minimal public `DeformableBody`
   handle and options value object for point-mass nodes, edges, masses,
   per-body spring/damping parameters, and fixed nodes. Store runtime data in
   experimental ECS components; keep components out of dartpy and public user
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
   edge-spring elastic energy and gravity load, with an internal positive
   clearance `xi` for the analytic ground barrier. Infeasible candidates
   (`distance <= xi`) are rejected by line search. Accepted candidates must be
   feasible and satisfy a deterministic descent/residual stopping rule.
   Velocities are updated from `(x_next - x_prev) / dt`.
4. **IPC-class collision expansion** — Use PR #2700 primitive CCD to assemble
   point-triangle and edge-edge candidate constraints for deformable
   self-contact and deformable-rigid contact. The default runtime path must use
   conservative queries; exact roots remain validation-only.
5. **Coupling expansion** — Introduce pairwise rigid/deformable couplers behind
   the solver architecture once contact buffers expose the needed state views.
   Keep common `World::step()` free of coupler or solver vocabulary.
6. **Examples and Python facade** — Add a C++ GUI visual smoke scene once the
   focused C++ behavior tests exist. Bind only the user-facing deformable body
   handle/options and basic state accessors after the C++ slice is stable. Add a
   small dartpy example in a later Python facade slice.

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
- `experimental_deformable_gui` renders a long-horizon headless Filament frame
  sequence that visibly exercises fixed anchors, spring deformation, and the
  explicit static ground barrier.
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
  invalid model rejection, fixed-node invariants, spring contraction,
  mass-scaled free-particle dynamics, finite static-ground footprint,
  active-contact sliding, static-ground barrier feasibility, ordinary static
  collision opt-out behavior, determinism, stage metadata, and binary
  serialization.
- `experimental_deformable_gui` can be run with:
  `pixi run ex experimental_deformable_gui --headless --frames 180 --width 960 --height 540 --out /tmp/experimental_deformable_gui_frames --screenshot /tmp/experimental_deformable_gui.ppm`.
- `bm_deformable_body` can be run with:
  `pixi run bm bm_deformable_body -- --benchmark_filter='BM_(WorldStepWithoutDeformables|DeformableGridStep|DeformableGridStage)' --benchmark_min_time=0.1s`.
- Local gates run for the slice: `pixi run lint`, `pixi run build`, focused
  `test_deformable_body`, and `pixi run check-api-boundaries`.

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

- PR #2705 changes the `WorldStepPipeline`, collision bridge, or deformable
  extension seam before merge.
- Primitive CCD APIs from PR #2700 change shape.
- First-slice tests show the barrier step cannot maintain feasibility with the
  current model/state split.
- Maintainers decide to import an external library instead of implementing the
  method in DART-owned code.
