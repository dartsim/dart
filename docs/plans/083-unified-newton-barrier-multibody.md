# PLAN-083: Unified Newton-Barrier Multibody Solver

- Operating state: `PLAN-083` in [`dashboard.md`](dashboard.md)
- Outcome: DART has a DART-owned IPC solver family whose unified
  Newton-barrier multibody implementation brings together deformable IPC,
  rigid IPC, affine/stiff-body dynamics, mixed
  rigid-deformable codimensional contact, articulation constraints, friction,
  restitution, CPU/GPU execution, benchmarks, and py-demos examples behind the
  DART 7 `World` facade without exposing upstream project names, solver
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
  - ZOZO's PPF Contact Solver and Ando 2024 provide a GPU-first shell/solid/rod
    contact and strain-limiting software baseline plus a cubic-barrier paper
    that should be evaluated inside the Newton-barrier family. DART treats this
    as method, API, diagnostic, example, and platform evidence, not as a
    dependency or public solver identity.
  - PLAN-080 owns the DART 7 rigid-body dynamics and articulation path;
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
  [`../design/simulation_cpp_api.md`](../design/simulation_cpp_api.md),
  [`../design/simulation_python_api.md`](../design/simulation_python_api.md)
- Compute/GPU policy:
  [`../design/scalable_compute_decisions.md`](../design/scalable_compute_decisions.md)
- Research references: `ipc-2020`, `rigid-ipc-2021`, `lan-2022-abd`,
  `chen-2022-unified-newton-barrier`, and `ando-2024-cubic-barrier` in
  [`../readthedocs/papers.md`](../readthedocs/papers.md)
- IPC-family variant consolidation map:
  [`083-unified-newton-barrier-multibody/ipc-variant-consolidation.md`](083-unified-newton-barrier-multibody/ipc-variant-consolidation.md)
- PPF contact solver intake:
  [`083-unified-newton-barrier-multibody/ppf-contact-solver-intake.md`](083-unified-newton-barrier-multibody/ppf-contact-solver-intake.md)
- Implementation roadmap:
  [`083-unified-newton-barrier-multibody/implementation-roadmap.md`](083-unified-newton-barrier-multibody/implementation-roadmap.md)
- Unified paper/deck manifest:
  [`083-unified-newton-barrier-multibody/paper-deck-manifest.md`](083-unified-newton-barrier-multibody/paper-deck-manifest.md)
- Shared primitive audit:
  [`083-unified-newton-barrier-multibody/shared-primitive-audit.md`](083-unified-newton-barrier-multibody/shared-primitive-audit.md)
- First implementation designs:
  [`083-unified-newton-barrier-multibody/primitive-promotion-slice.md`](083-unified-newton-barrier-multibody/primitive-promotion-slice.md),
  [`083-unified-newton-barrier-multibody/abd-first-slice-design.md`](083-unified-newton-barrier-multibody/abd-first-slice-design.md)
- Completion audit and remaining-work tracker:
  [`083-unified-newton-barrier-multibody/completion-audit.md`](083-unified-newton-barrier-multibody/completion-audit.md)
  (the temporary dev-task folder was retired on 2026-07-04)
- Examples policy:
  [`103-examples-strategy.md`](103-examples-strategy.md)

## Consolidation Decisions

1. **One DART IPC method family, multiple variant owners.** Use IPC as the
   representative solver-family name when the unified Newton-barrier method is
   the most advanced shared IPC variant. PLAN-083 owns the shared
   Newton-barrier multibody implementation direction and the
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
   compatible with the DART 7 solver architecture.
7. **PPF cubic-barrier and stack intake** - Use the PPF sidecar to evaluate
   cubic barriers, elasticity-inclusive dynamic stiffness, strain limiting,
   ACCD precision diagnostics, shell/solid/rod model requirements, solver logs,
   and GPU/platform lessons before adding new shared primitives or public
   options.
8. **CPU/GPU performance program** - Extend PLAN-030-style private GPU gates to
   contact stencils, CCD, barrier/friction kernels, PSD projection, assembly,
   and linear solves. Each optimization must keep CPU/GPU result parity at
   matched tolerances before claiming a speedup.
9. **Examples and visual evidence** - Port the paper/unit/benchmark scenes into
   py-demos categories, keep scene logic single-sourced where possible, and
   attach long-horizon headless Filament evidence for promoted examples.

## Acceptance Criteria

- The unified manifest has zero unclassified rows and links every paper/deck
  figure, table, unit test, and benchmark/comparison scene to a DART artifact or
  a maintainer-visible manual/not-applicable rationale.
- PLAN-081 and PLAN-082 either consume the shared internal Newton-barrier
  primitives or document why a variant-specific primitive remains necessary.
- The DART 7 public facade remains backend-neutral and DART-owned: no
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
  contact, articulation, and motion. Use the agent-verification visual template
  from `docs/onboarding/agent-sim-verification.md`: capture the solver scene
  with `pixi run py-demo-capture -- --scene <scene> --view three-quarter --fit`
  and gate the resulting still with
  `pixi run image-verdict -- <capture.png> --metadata plan=PLAN-083`. For
  stable renderer-regression lanes, curate a backend-local golden with
  `pixi run render-golden-gate -- --golden tests/fixtures/render_goldens/plan083_<scene>.png --update`
  and compare it with the same command without `--update`; these PNG goldens
  stay opt-in and backend-specific, not default CI inputs.
- Before any commit, run the required gates from `docs/ai/verification.md`;
  implementation slices also keep `pixi run lint`, `pixi run build`, focused
  C++/Python tests, benchmark smoke packets, and `check-api-boundaries` green.

## Immediate Next Steps

1. Use the PLAN-083 sidecar manifest to drive status changes for every unified
   paper/deck figure, unit-test row, benchmark row, comparison row, and py-demos
   target category.
2. Drive remaining PLAN-083 work from the durable sidecars (the
   [`paper-deck-manifest.md`](083-unified-newton-barrier-multibody/paper-deck-manifest.md)
   and [`completion-audit.md`](083-unified-newton-barrier-multibody/completion-audit.md)):
   promote the first ABD benchmark packet from smoke shape to a comparison
   manifest row now that affine barrier and friction primitive derivative
   oracles are stable.
3. Generalize PSD projection, projected-Newton, line-search, diagnostics, and
   benchmark-schema contracts when ABD or another solver-family slice creates a
   second-use contract beyond deformable and rigid local needs.
4. Keep new IPC-family papers or solver components under the plan-owned
   solver-family intake checklist in
   [`solver-family-intake.md`](solver-family-intake.md):
   route to an existing variant owner, inventory shared components, define the
   user-facing configuration surface, and define apples-to-apples evidence
   before duplicating primitives or adding broad public options.
5. Classify PPF's cubic-barrier paper, repository examples, solver logs,
   shell/solid/rod assets, frontends, and GPU platform claims through
   [`ppf-contact-solver-intake.md`](083-unified-newton-barrier-multibody/ppf-contact-solver-intake.md)
   before promoting any cubic-barrier, strain-limit, ACCD, or public API
   change.
6. Keep the durable plan sidecars, benchmark packets, and implementation gates
   synchronized with this owner plan. The temporary dev-task folder was retired
   on 2026-07-04, so remaining work is tracked here and in the sidecars, not in
   `docs/dev_tasks/`.

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
- PPF intake evidence proves a cubic barrier, strain-limit row, ACCD diagnostic,
  shell/rod adapter, solver-log schema, or GPU platform pattern should become a
  DART-owned shared primitive or API surface.
- CPU/GPU benchmark evidence changes the sequencing or exposes a correctness
  gap.
- Public API naming would leak upstream project names, solver registries, ECS
  storage, or backend resources.

## Progress log

Relocated from the dashboard on 2026-07-03; newest first.

2026-07-04: retired the temporary
`docs/dev_tasks/unified_newton_barrier_multibody/` working folder (README,
RESUME, HANDOFF) by maintainer decision. PLAN-083 stays Active and NOT COMPLETE;
every remaining blocker is now owned by this plan file, the
[`dashboard.md`](dashboard.md) PLAN-083 entry, and the plan sidecars
([`completion-audit.md`](083-unified-newton-barrier-multibody/completion-audit.md),
[`paper-deck-manifest.md`](083-unified-newton-barrier-multibody/paper-deck-manifest.md),
and [`gpu-slowness-root-cause.md`](083-unified-newton-barrier-multibody/gpu-slowness-root-cause.md),
plus siblings). The PLAN-083 benchmark scripts (`scripts/*plan083*`) and tests
(`tests/*plan083*`) are unchanged by the retirement.

The consolidated follow-up work was tracked on
`simx/plan083-gpu-contact-candidate-packet` / PR #2978. PR #2960 landed
implementation-roadmap Phases 3-8; PR #2961 measured the private GPU PSD
projection packet, added the Fig. 17 barrier-force diagnostic, and aligned
articulation-only figure rows with landed private diagnostics; PR #2970
landed runtime wiring for point/fixed and hinge constraints, opt-in BDF-2,
deformable fixed obstacles, and the reduced hanging-bridge py-demo; PR #2971
landed reduced CPU packets for lying-flat, hanging-bridge, pulley, umbrella,
terrain vehicle, ragdoll, nunchaku, nunchaku scaling, windmill, Candy,
precession, reduced timing-breakdown, reduced Table 2, the sparse equality
change-of-variable rigid IPC path, and the reduced affine point-triangle
micro-solve diagnostic; PR #2974 added reduced ABD runtime-step evidence for
`abd-vs-rigid-cards`, `abd-vs-rigid-wreck`, `abd-chain-8`, `abd-chain-16`,
and `abd-chain-96`; PR #2976 adds reduced ABD gears/Bullet comparison
packets without claiming gear assets, Bullet/reference baselines, GPU parity,
or paper-scale parity. PR #2978 is the single consolidated follow-up for the
remaining private packet work: it adds point-triangle and edge-edge contact
stencil parity plus brute-force all-pairs point-triangle and edge-edge
candidate masks and motion-aware swept-AABB point-triangle/edge-edge
candidate-list packets with device-side compacted candidate ids and distance
metadata, compact device-sorted sweep-and-prune broad-phase packets, plus
compact runtime sweep-buffer endpoint-distance packets, reduced
scene-owned runtime candidate-buffer packets, and reduced scene-owned
runtime sweep broad-phase packets, plus reduced combined scene runtime
sweep-filter and candidate-filter rows extracted from the same DART `World`
surface,
endpoint-linear point-triangle/edge-edge CCD/line-search parity plus sampled
rigid-curved point-triangle/edge-edge CCD/line-search parity plus reduced
scene-owned runtime point-triangle/edge-edge CCD rows plus a reduced combined
scene runtime CCD line-search row, scalar
barrier/friction local-kernel parity
plus point-triangle
primitive barrier-gradient and point-triangle/edge-edge/point-edge/point-point
tangent-stencil parity, point-triangle/point-point/point-edge/edge-edge
primitive barrier-Hessian parity, point-triangle/point-point/point-edge
primitive barrier-Hessian PSD-projection parity, reduced scene-owned
point-triangle, point-edge, point-point, and edge-edge barrier-Hessian runtime
rows plus a reduced combined all-family scene runtime barrier-Hessian row,
reduced diagonal assembly/solve plus
pair-slot off-diagonal sparse-block assembly plus a reduced scene-owned
sparse off-diagonal surface-edge assembly row plus a reduced scene-owned
sparse graph construction/assembly row plus a reduced scene-owned nonlinear
distance-equality assembly row plus a reduced scene-owned nonlinear
distance-equality solve row plus a reduced scene-owned capped nonlinear
distance-equality convergence row plus sparse block residual matvec plus a
reduced scene-owned sparse residual row plus fixed-iteration sparse Jacobi
solve plus a reduced scene-owned sparse Jacobi row plus capped sparse CG solve
plus a reduced scene-owned sparse CG row plus bounded reduced direct sparse
factor solve plus reduced scene-owned bounded direct sparse factor solve plus
sparse equality-reduced diagonal solve parity plus a reduced scene-owned
equality-reduced diagonal solve row plus a reduced scene-owned sparse graph
unique-edge dedup row plus a reduced scene-owned diagonal assembly/solve row,
reduced hanging-bridge scene
state-batch CPU/GPU parity and speedup, reduced ABD complex-geometry packets,
an ABD/FEM coupled micro-solve packet with external surface CCD sidecar
witnesses, built-in deformable `World::step` self-surface candidate/CCD
diagnostics in the reduced lying-flat, Candy, and ABD/FEM CPU scene packets,
and public built-in inter-body/static-rigid/
moving-rigid surface CCD diagnostics, then serializes those external counters
into the reduced deformable CPU scene packet rows and adds a dedicated reduced
external surface CCD CPU diagnostic packet with nonzero inter-body/static-rigid/
moving-rigid counters, including one mixed reduced `World::step` witness that
activates all three external families, plus reduced lying-flat
inter-body/static-rigid/moving-rigid surface CCD witness rows, one reduced
hanging-bridge inter-body/static-rigid/moving-rigid external CCD sidecar row,
one reduced pulley inter-body/static-rigid/moving-rigid external CCD sidecar
row,
one reduced umbrella inter-body/static-rigid/moving-rigid external CCD sidecar
row, one reduced terrain vehicle inter-body/static-rigid/moving-rigid
external CCD sidecar row, one reduced ragdoll inter-body/static-rigid/
moving-rigid external CCD sidecar row, one reduced precession
inter-body/static-rigid/moving-rigid external CCD sidecar row, one reduced
Candy static-rigid/moving-rigid surface CCD witness row, and one reduced
ABD/FEM external sidecar witness row. Other
broader figure/demo scene rows still have zero external candidate/check/hit
counts.
It still keeps
production runtime scene filtering, production analytic curved CCD,
production scene-level line search inside
`World::step`, full runtime
sparse Hessian graph construction and assembly beyond the reduced dedup row,
unbounded production direct/global sparse factorization, production nonlinear equality convergence
policy/solving, GPU `World::step`, paper-scale assets, full runtime
affine/FEM coupling, production runtime scene filtering inside `World::step`,
and accepted reference timings as future evidence. The
completion audit still records PLAN-083 as incomplete while in-progress
CPU/GPU/scene limitations remain. (Superseded 2026-07-04: the dev-task folder
was retired; see the newest progress-log entry above.)
