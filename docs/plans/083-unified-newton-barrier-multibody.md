# PLAN-083: Unified Newton-Barrier Multibody Solver

- Operating state: `PLAN-083` in [`dashboard.md`](dashboard.md)
- Outcome: DART has a DART-owned Newton-barrier multibody solver family that
  unifies deformable IPC, rigid IPC, affine/stiff-body dynamics, mixed
  rigid-deformable codimensional contact, articulation constraints, friction,
  restitution, CPU/GPU execution, benchmarks, and py-demos examples behind the
  experimental `World` facade without exposing upstream project names, solver
  registries, ECS storage, or backend resources as public API.
- Current evidence:
  - Chen et al. 2022, "A Unified Newton Barrier Method for Multibody Dynamics,"
    unifies rigid and soft bodies as Lagrangian nodal displacements; linear
    equality joints through a change-of-variable strategy; nonlinear equality
    constraints through stiff potentials; inequality contacts, joint ranges,
    friction, and restitution through barrier potentials and variational IPC;
    and evaluates point connections, hinges, cone twists, sliding ranges,
    friction, BDF-2 restitution, Rayleigh damping, and mixed rigid/deformable
    scenes.
  - The supplied multibody/ABD deck frames affine body dynamics as the stiff
    body representation that keeps compact coordinates while using
    piecewise-linear trajectories for IPC contact. It reports large CPU gains
    over rigid IPC, comparable-or-faster rows versus Bullet at larger contact
    workloads, and a GPU acceleration target for collision-heavy ABD scenes.
  - PLAN-081 already owns DART's deformable IPC work: FEM state, PT/EE
    distances, conservative CCD, clamped-log barriers, projected Newton,
    friction, diagnostics, a GPU PSD-projection backend, and IPC py-demos.
  - PLAN-082 already owns DART's rigid IPC work: upstream fixture/test/benchmark
    manifests, curved-trajectory CCD, reduced rigid barrier derivatives,
    projected Newton scaffolding, lagged friction, opt-in runtime stage,
    solver selection, diagnostics, and first rigid IPC py-demos.
  - The active PLAN-083 task has already promoted pure world-primitive
    distance, barrier, tangent-stencil, and tangential-friction math into the
    internal `detail/newton_barrier` owner. Deformable compatibility headers
    forward to it, rigid IPC consumes it directly, ABD consumes it for affine
    barrier/friction chain-rule rows, and `test_newton_barrier_primitives`
    covers old/new alias and rigid-consumer parity.
  - The internal ABD foundation now has `detail/affine_body_dynamics`, affine
    state/surface adapters, orthogonality energy, rigid barrier/friction
    equivalence oracles, and `bm_affine_body_dynamics` smoke rows. Those are
    correctness foundations, not a runtime solver or paper-scale completion
    claim.
  - PLAN-080 owns the experimental rigid-body dynamics and articulation path;
    PLAN-030 owns the private CPU/GPU compute gates; PLAN-103 owns the
    Python-first examples surface.

## Owner Docs

- Deformable IPC variant:
  [`081-deformable-implicit-barrier-solver.md`](081-deformable-implicit-barrier-solver.md)
- Rigid IPC variant:
  [`082-rigid-implicit-barrier-contact.md`](082-rigid-implicit-barrier-contact.md)
- Rigid/articulated baseline:
  [`080-rigid-body-dynamics-solver.md`](080-rigid-body-dynamics-solver.md)
- Solver architecture and public facade:
  [`../design/simulation_solver_architecture.md`](../design/simulation_solver_architecture.md),
  [`../design/simulation_experimental_cpp_api.md`](../design/simulation_experimental_cpp_api.md),
  [`../design/simulation_experimental_python_api.md`](../design/simulation_experimental_python_api.md)
- Compute/GPU policy:
  [`../design/scalable_compute_decisions.md`](../design/scalable_compute_decisions.md)
- Research references: `ipc-2020`, `rigid-ipc-2021`, `lan-2022-abd`, and
  `chen-2022-unified-newton-barrier` in
  [`../readthedocs/papers.md`](../readthedocs/papers.md)
- IPC-family variant consolidation map:
  [`083-unified-newton-barrier-multibody/ipc-variant-consolidation.md`](083-unified-newton-barrier-multibody/ipc-variant-consolidation.md)
- Implementation roadmap:
  [`083-unified-newton-barrier-multibody/implementation-roadmap.md`](083-unified-newton-barrier-multibody/implementation-roadmap.md)
- Unified paper/deck manifest:
  [`083-unified-newton-barrier-multibody/paper-deck-manifest.md`](083-unified-newton-barrier-multibody/paper-deck-manifest.md)
- Shared primitive audit:
  [`083-unified-newton-barrier-multibody/shared-primitive-audit.md`](083-unified-newton-barrier-multibody/shared-primitive-audit.md)
- First implementation designs:
  [`083-unified-newton-barrier-multibody/primitive-promotion-slice.md`](083-unified-newton-barrier-multibody/primitive-promotion-slice.md),
  [`083-unified-newton-barrier-multibody/abd-first-slice-design.md`](083-unified-newton-barrier-multibody/abd-first-slice-design.md)
- Active implementation tracker:
  [`../dev_tasks/unified_newton_barrier_multibody/`](../dev_tasks/unified_newton_barrier_multibody/)
- Examples policy:
  [`103-examples-strategy.md`](103-examples-strategy.md)

## Consolidation Decisions

1. **One DART method family, multiple variant owners.** PLAN-083 owns the
   shared Newton-barrier multibody direction and the
   [`ipc-variant-consolidation.md`](083-unified-newton-barrier-multibody/ipc-variant-consolidation.md)
   routing map. PLAN-081 keeps deformable FEM/IPC, codimensional IPC, PD-IPC
   GPU, and SPB recovery obligations. PLAN-082 keeps exact/reduced rigid IPC
   obligations. PLAN-104 keeps VBD/OGC-adjacent contact obligations. PLAN-080
   keeps the DART 6 parity rigid/articulated baseline. A future ABD slice must
   join this family; it does not erase the rigid IPC correctness oracle until it
   beats it on matched fixtures.
2. **Public names stay DART-owned.** User-facing API should name method
   families and capabilities such as implicit barrier contact, affine stiff-body
   dynamics, variational restitution, BDF-2 integration, and Newton-barrier
   constraints. Internal tests, manifests, and benchmark provenance may cite
   IPC, rigid IPC, ABD, and paper figure names.
3. **Shared primitives become internal contracts.** Barrier scalar functions,
   PT/EE/PE/PP distances, tangent stencils, conservative CCD, candidate sets,
   PSD projection, sparse projected Newton, lagged friction, Rayleigh damping,
   diagnostics, benchmark JSON, and headless visual evidence should be shared
   when a second variant uses them. The first distance/barrier/tangent/friction
   promotion now lives under `detail/newton_barrier`; future variant-specific
   code must justify any divergence in its plan or dev-task handoff.
4. **Affine body dynamics is a new stiff-body representation track.** ABD's
   12-DOF affine body with stiff orthogonality potential is planned as an
   alternate stiff/rigid representation. It must coexist with DART's generalized
   and rigid IPC paths until correctness, energy behavior, contact quality, and
   performance prove when it can replace or augment them.
5. **Articulation constraints join the barrier family.** Point connections,
   fixed points, hinges, cone twists, sliding constraints, distance constraints,
   relative sliding, bounded distances, sliding ranges, and rotation ranges are
   part of the unified target. Linear equality constraints should use the
   paper's sparse change-of-variable approach where it beats DART's current
   solve path without breaking public topology.
6. **Restitution is energy-based and opt-in first.** BDF-2 and semi-implicit
   Rayleigh damping for contact and inequality articulation constraints are
   planned as opt-in solver capabilities. Existing default rigid stepping and
   compatibility behavior must remain unchanged until parity gates promote a
   new default.
7. **CPU and GPU are both completion requirements.** CPU correctness lands
   first only when needed for reviewability. Full PLAN-083 completion requires
   CPU and GPU benchmark packets at matched accuracy. Where a paper has no GPU
   baseline, DART must record that absence and still prove GPU speedup over the
   DART CPU path on the same scenes without changing results.
8. **Examples belong in py-demos.** Every promoted paper scene or unit family
   gets a py-demos entry in the appropriate category, plus headless Filament
   evidence, nonblank/motion checks, and benchmark/profiling artifacts when the
   paper uses the scene for performance.

## Paper Obligation Map

| Obligation                     | Required DART outcome                                                                                                                                                                                                                      |
| ------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| Unified DOFs                   | Deformables, affine stiff bodies, rigid bodies, rods, shells, particles, and codimensional objects advance through one solver-family abstraction with explicit state adapters.                                                             |
| Linear equality constraints    | Point/fixed connections, hinge constructions, cone-twist structures, and sliding constraints can be represented through sparse change-of-variable or an equivalent tested DART solve.                                                      |
| Nonlinear equality constraints | Distance and relative-sliding constraints have stiff-potential implementations, residual diagnostics, and failure tests.                                                                                                                   |
| Inequality barriers            | Contact, bounded distance, sliding range, and rotation range constraints use clamped barriers with derivative, PSD, line-search, and range-satisfaction tests.                                                                             |
| Contact and friction           | Mixed-dimensional rigid/deformable/codimensional contact uses shared candidate sets, conservative CCD, barriers, lagged Coulomb friction, tangent bases, normal-force lagging, and convergence diagnostics.                                |
| Restitution                    | BDF-2 and semi-implicit Rayleigh damping reproduce contact and joint restitution energy studies and expose damping controls with documented units and defaults.                                                                            |
| Mixed scenes                   | Hanging bridge, pulley, umbrella, terrain vehicle, ragdolls, precession, lying-flat, windmill, candy, ABD chain nets/gears, and Bullet/reference comparison scenes become DART tests, py-demos, benchmarks, or manual rows with rationale. |
| Performance                    | DART records per-scene body/node/contact counts, timestep, friction, iteration counts, CPU timing, GPU timing where applicable, memory, and ratios against DART incumbents, upstream references, and paper numbers.                        |

## Workstreams

1. **Unified paper/figure/table manifest** - Build a sidecar manifest from the
   two supplied references plus existing IPC/rigid-IPC manifests. Every figure,
   unit test, benchmark table, comparison, and slide-deck performance row must
   map to a DART artifact, target category, command, and completion status.
2. **Shared internal primitive layer** - Maintain the promoted
   `detail/newton_barrier` primitive owner for distance, barrier, tangent, and
   friction math, then promote additional second-use pieces such as PSD
   projection, line-search contracts, projected-Newton diagnostics, and
   benchmark schemas only after focused cross-variant tests prove the shared
   behavior. The consolidation map records which variant owns each obligation
   until that second-use evidence exists.
3. **ABD stiff-body track** - Add affine-body model/state representation,
   orthogonality energy, inertia/contact/friction chain rules, stiffness
   controls, and solver diagnostics. Compare against the rigid IPC oracle,
   current DART rigid contact, ABD paper/deck scenes, and Bullet where the paper
   does.
4. **Unified articulation constraints** - Implement the paper's point, hinge,
   cone-twist, sliding, distance, bounded-distance, sliding-range, and
   rotation-range families behind DART-owned topology and policy names.
5. **Unified restitution and damping** - Add BDF-2 support, energy diagnostics,
   and lagged Rayleigh damping for contact and inequality articulation
   constraints with falling-box and rotating-board parity tests.
6. **Mixed-domain coupling** - Convert rigid/deformable/codimensional contact
   from variant-local scaffolding into shared contact buffers and coupler seams
   compatible with the experimental solver architecture.
7. **CPU/GPU performance program** - Extend PLAN-030-style private GPU gates to
   contact stencils, CCD, barrier/friction kernels, PSD projection, assembly,
   and linear solves. Each optimization must keep CPU/GPU result parity at
   matched tolerances before claiming a speedup.
8. **Examples and visual evidence** - Port the paper/unit/benchmark scenes into
   py-demos categories, keep scene logic single-sourced where possible, and
   attach long-horizon headless Filament evidence for promoted examples.

## Acceptance Criteria

- The unified manifest has zero unclassified rows and links every paper/deck
  figure, table, unit test, and benchmark/comparison scene to a DART artifact or
  a maintainer-visible manual/not-applicable rationale.
- PLAN-081 and PLAN-082 either consume the shared internal Newton-barrier
  primitives or document why a variant-specific primitive remains necessary.
- The experimental public facade remains backend-neutral and DART-owned: no
  public `IPC`, `RigidIPC`, `ABD`, upstream repository, solver registry,
  coupler registry, ECS, device, stream, memory-pool, or backend task type.
- Solver options remain easy on the common `World` creation path: defaults and
  presets cover ordinary use, advanced method-specific knobs live in nested
  option objects with typed validation, incompatible combinations fail with
  actionable diagnostics, and result-affecting options have serialization/
  restart behavior or an explicit runtime-only rationale.
- Correctness tests prove no-intersection, no-inversion where applicable,
  constraint satisfaction, finite energy, derivative correctness, PSD projection
  behavior, line-search feasibility, friction/stiction behavior, restitution
  energy behavior, serialization/restart, determinism, and mixed-domain stepping.
- CPU benchmark packets beat or explicitly justify any failure to beat the
  DART incumbent, the audited reference implementations, and the paper/deck
  numbers on matched scenes at matched accuracy.
- GPU benchmark packets prove result parity with CPU and beat the matched DART
  CPU path on representative workloads; when a cited paper has no GPU number,
  the packet records that fact instead of fabricating a baseline.
- Every promoted example family has a py-demos scene, a headless smoke or
  capture command, and long-horizon visual evidence sufficient to inspect
  contact, articulation, and motion.
- Before any commit, run the required gates from `docs/ai/verification.md`;
  implementation slices also keep `pixi run lint`, `pixi run build`, focused
  C++/Python tests, benchmark smoke packets, and `check-api-boundaries` green.

## Immediate Next Steps

1. Use the PLAN-083 sidecar manifest to drive status changes for every unified
   paper/deck figure, unit-test row, benchmark row, comparison row, and py-demos
   target category.
2. Continue the active
   [`../dev_tasks/unified_newton_barrier_multibody/`](../dev_tasks/unified_newton_barrier_multibody/):
   promote the first ABD benchmark packet from smoke shape to a comparison
   manifest row now that affine barrier and friction primitive derivative
   oracles are stable.
3. Generalize PSD projection, projected-Newton, line-search, diagnostics, and
   benchmark-schema contracts when ABD or another solver-family slice creates a
   second-use contract beyond deformable and rigid local needs.
4. Keep new IPC-family papers or solver components under the solver-family
   intake gate in
   [`../design/simulation_solver_architecture.md`](../design/simulation_solver_architecture.md):
   route to an existing variant owner, inventory shared components, define the
   user-facing configuration surface, and define apples-to-apples evidence
   before duplicating primitives or adding broad public options.
5. Keep dev-task status, resume context, and implementation gates synchronized
   with this owner plan until the active task can be retired into durable
   design, plan, benchmark, and example artifacts.

## Non-Goals

- No runtime vendoring or linking of `ipc-sim/IPC`, `ipc-sim/rigid-ipc`, ABD
  code, Bullet, or other reference implementations.
- No public upstream-project solver names or backend-specific resource handles.
- No claim that ABD replaces exact rigid IPC, generalized-coordinate rigid
  dynamics, or deformable IPC until correctness, performance, and example
  parity prove it on shared scenes.
- No GPU public API commitment before private benchmark packets justify it.

## Revision Triggers

- A shared primitive is promoted from a variant-local implementation.
- ABD evidence shows it should replace, augment, or stay separate from the
  current rigid IPC path.
- Paper/deck manifest rows change status, split, or reveal missing assets.
- CPU/GPU benchmark evidence changes the sequencing or exposes a correctness
  gap.
- Public API naming would leak upstream project names, solver registries, ECS
  storage, or backend resources.
