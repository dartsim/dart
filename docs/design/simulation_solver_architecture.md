# Simulation Solver Architecture

## Status And Ownership

Accepted direction with partial implementation. Current source evidence is in
[the architecture assessment](dart7_architecture_assessment.md), refreshed at
`afe6c7d` on 2026-09-04. [PLAN-040](../plans/040-dart7-release-hardening.md) owns
readiness milestones; subsystem plans own executable packets. Closed
PLAN-020/091 work is background. The [public architecture page](../readthedocs/architecture.md)
is the shorter overview; this document owns solver/coupling rationale.

DART 7 is a new research engine. A simple World facade should support a
portfolio of compatible methods, reproducible explicit choices and useful
defaults. Neither a universal best solver nor arbitrary automatic composition
is assumed. M1 validates the contracts through one complete rigid method;
broader selection and coupled physics follow evidence-led intake.

## Independent Capability Axes

| Axis                          | Examples                                                                                                            | Why it is separate                                                                                                     |
| ----------------------------- | ------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------- |
| Physical model/domain         | Rigid bodies, elastic solids, fluids, discrete grains, continuum granular material, cloth, hair, aerodynamic forces | Defines the physical assumptions and conserved/exchanged quantities. Articulated rigid bodies are still rigid physics. |
| Intrinsic/embedding dimension | Volumes 3D in 3D, surfaces 2D in 3D, curves 1D in 3D                                                                | Codimension is embedding minus intrinsic dimension; it does not select a constitutive law or solver.                   |
| Representation/discretization | Maximal/reduced coordinates, tetrahedra, shells, rods, particles, grids, reduced bases                              | Particles can represent fluid, solid or grains; a surface mesh can bound a volume or represent a shell.                |
| Numerical method              | SI, Newton/barrier, VBD/AVBD, XPBD, projective or fluid methods                                                     | A method can support one or several domains/representations.                                                           |
| Interaction/coupling          | Contact, attachments, drag, pressure, thermal exchange                                                              | Requires explicit ownership, exchanged quantities and time/convergence rules.                                          |
| Execution/capabilities        | CPU/CUDA, Float64/Float32, deterministic mode, gradients, checkpoint continuation                                   | Not every algorithm implements every backend, precision or derivative mode.                                            |

A capability record needs a supported envelope, validation and tests, not just
boolean tags. Include geometry/material restrictions, joints/constraints,
contact law, capacity, timestep assumptions and continuation requirements.
Unsupported combinations fail before stepping, with an actionable explanation.

## Current Implementation And Remaining Gap

The built-in `WorldStepPipeline` is an ordered stage list from
`dart/simulation/detail/world_step_schedule.hpp`. Stages have virtual
preflight/prepare/execute operations and some emit compute graphs. Method
changes share preparation; empty domains do not require placeholder work.
This is a useful composition seam, not an implemented universal Solver/Coupler
framework. An executor schedules work; it does not supply missing GPU kernels.

`WorldOptions` and runtime setters provide DART-owned configuration, with
resolved configuration and several fail-closed checks. Current families include:

| Selection               | Implemented scope                                                                            | Limits relevant to readiness                                                                                     |
| ----------------------- | -------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------- |
| Free rigid default      | Semi-implicit dynamics and SI contact; boxed-LCP is another contact solve within that family | General asymmetric rotation, friction isotropy and full CUDA contact require M1 work.                            |
| Articulated integration | Semi-implicit and opt-in variational paths, with rigid/articulated shared constraints        | Qualify the admitted joint/actuator/contact envelope in M3; do not infer O(n) execution from an unused operator. |
| Rigid IPC               | Opt-in barrier-family runtime and selected geometry/contact behavior                         | Full paper/geometry/benchmark claims remain PLAN-082/083 work.                                                   |
| VBD / AVBD              | Public rigid choices and bounded deformable/rigid/articulated mechanisms                     | Paper parity is partial. Existing public selection is not proof of all paper scenes or complete GPU stepping.    |
| Deformables             | Mass-spring/FEM and selected IPC/VBD/AVBD paths                                              | Rigid obstacles and selected shared constraints do not establish generic two-way multiphysics coupling.          |
| Differentiation         | Opt-in scoped derivatives and trajectory machinery                                           | Gradients are capability-specific; no universal contact/coupling/backend guarantee.                              |

Model/state/control components, baked dense indices, metrics and replay exist.
General state vectors and binary continuation remain incomplete. The
[assessment](dart7_architecture_assessment.md) links exact source/test owners;
retain that inventory instead of copying an absence list here.

## Proposed Solver And Interaction Contract

The internal contract describes responsibilities, not a required inheritance
hierarchy or public plugin ABI:

- Declare supported domain/representation/interaction/backend combinations.
- Validate assignment and prepare model/state-bound workspace at bake.
- Declare the state variables, physical interactions and clock intervals the
  method advances, plus its read/write/reduction resources.
- Emit an ordered compute plan or a fused operation with explicit completion,
  error and continuation semantics.
- Report resolved variant, actual execution path, convergence/physical metrics,
  and required checkpoint history through DART-owned values.

A solver may own one domain, a subset, or a coupled block spanning several.
Each evolving degree of freedom and interaction must have one authoritative
update/force owner per phase. Detect missing or double integration and double
contact forces when composing methods. World-level time/topology and common
public handles remain independent of the selected implementation.

Coupling has two legitimate forms:

1. **Shared solve:** one method assembles and solves a coupled problem over
   several domains or representations. Its internal interface terms are not
   forced through artificial pairwise objects.
2. **Partitioned solve:** methods exchange typed quantities through an explicit
   coupling strategy, with declared order/subcycling/iterations, interpolation,
   conservation/work budgets and convergence/failure handling.

A pairwise adapter is useful for a genuinely pairwise interaction. It is not a
universal abstraction: a global constraint group can span many objects/domains,
and a naive matrix of every solver pair grows combinatorially. Prefer a small
set of physical exchange contracts with tested compatibility. Do not assume a
shared signature makes every numerical combination stable or conservative.

Prepare / pre-couple / couple / post-couple is one possible partitioned
protocol, not a mandatory four-phase window. Implicit monolithic solves,
multirate methods and iterative partitioned schemes need different schedules.
Keep the present ordered stage path while prototypes determine the smallest
useful contract. Differentiating a coupled method requires the exchange and
iteration semantics to be part of its derivative contract too.

## Selection Without Requiring Solver Expertise

Long-term selection is a **versioned selection policy over a tested solver
portfolio**, not a universal native algorithm:

1. Compile scene requirements from physical models, representations,
   interactions, requested precision/device/gradients and constraints.
2. Filter supported method/coupling combinations using capability records.
3. Choose from tested combinations using a small documented policy, such as
   balanced, accuracy-oriented or throughput-oriented. Estimates guide ranking;
   they cannot relax correctness constraints.
4. Expose the resolved algorithm/variant, coupling strategy, execution and any
   permitted fallback in diagnostics and snapshots. Explain rejection when no
   supported combination exists.

Start with an explicit compatibility table and deterministic rules. Do not
build a general search planner, learned tuner or combinatorial optimizer in M1.
A simple scene should use the default constructor and `step()`. Advanced users
can request a DART-owned method family and versioned options/preset; explicit
choices pass the same validation and must not silently fall back to another
method or device. An `auto` preference may select a documented supported path,
with the resolved choice visible. DART-owned device preference values are
allowed by design; framework, pool, stream, ECS and solver objects stay private.
WP-040.2 owns the corresponding existing-checker migration.

Reproducibility needs two identities: **algorithm variant version** and
**selection-policy version**. A checkpoint pins the resolved variant and all
result-affecting choices, not just `auto` or the current default. If a pinned
variant is unavailable, reject exact continuation; an explicitly requested
migration can transfer physical state with documented loss of equivalence.

## Research Variants And Default Evolution

Implement papers as named, versioned variants within an appropriate solver
family when their contract fits; use another family when formulation or
capabilities differ materially. A research variant should be reproducible and
traceable to the paper/reference, with separate conformance and performance
results. It need not duplicate model loading, collision, algebra, snapshots or
benchmark infrastructure when those primitives have matching semantics.

The default selects a maintained variant or tested combination. Incorporating
an advance creates a new version, with numerical/performance comparisons and
migration notes. Keep old versions for an explicit support period; deprecate
with recorded ownership and replacement limits. Avoid an unversioned “native”
solver that silently accumulates incompatible paper ideas. No one family needs
to dominate every workload to remain useful to researchers.

[Solver intake](../plans/solver-family-intake.md) binds complete paper
manifests, shared-scene metrics, actual solver/backend identity and all existing
paper obligations. The current requirement to beat every claimed reference
benchmark remains policy; PLAN-040 lists an unresolved proposal to separate
reproduction from performance improvement. A new default does not waive older
family completion targets.

## Recommended Algorithm Sequence

| When                  | Recommendation                                                                                                | Tradeoffs and evidence required                                                                                                                                                                                                                                                                                                                                                                                                 |
| --------------------- | ------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| M1                    | Semi-implicit translation, correct manifold-aware rotation, hard unilateral SI with circular Coulomb friction | Small implementation and interpretable analytic tests; sequential contact updates are a useful CPU baseline. Finite iterations/order can affect stacking; GPU correctness need not imply single-body speedup. [Solver2D](https://box2d.org/posts/2024/02/solver2d/) explains iterative contact variants; general rotation follows [Euler rigid dynamics](https://www.cs.cmu.edu/afs/cs.cmu.edu/user/baraff/www/pbm/rigid1.pdf). |
| M2                    | Optimize collision/islands and measured batching before replacing the physical method                         | Parallel independent islands preserve internal update order. Coloring or Jacobi can expose more GPU work but changes convergence and sometimes the variant; compare at matched physical residuals. Retain an independent small-problem reference.                                                                                                                                                                               |
| M3 and research lanes | Qualify existing articulated methods and compare compatible SI/boxed-LCP, VBD/AVBD and barrier variants       | Reuse current implementation investment. [VBD](https://ankachan.github.io/Projects/VertexBlockDescent/index.html) and [AVBD](https://graphics.cs.utah.edu/research/projects/avbd/) offer local-block parallelism and research value, but nonlinear convergence, constraint/friction coverage and paper fidelity need their own evidence.                                                                                        |
| M4                    | Fixed-corotational tetrahedral solid; compare a shared coupled solve with a partitioned rigid-solid prototype | Clear material/deformation references and a concrete interface test. [IPC](https://ipc-sim.github.io/) offers a strong contact framework but robust geometry, CCD, derivatives and global solves raise complexity; no unconditional first-default choice.                                                                                                                                                                       |
| M5                    | Evaluate XPBD for a bounded cloth/rod baseline alongside energy-based methods                                 | [XPBD](https://matthias-research.github.io/pages/publications/XPBD.pdf) is simple and compliance-aware with parallel opportunities, but accuracy/constitutive behavior and convergence must be measured. It does not automatically supply all shell/rod physics.                                                                                                                                                                |
| M6                    | Choose fluid/granular formulation from a concrete use case and coupling experiment                            | MPM, SPH and grid formulations have different material, boundary, conservation and hardware tradeoffs. Particle storage alone is not a fluid solver. Defer this choice until a bounded problem and reference exist.                                                                                                                                                                                                             |

Perfect restitution does not imply exact energy conservation for a discretized
trajectory. M1 separates flight, impact and positional-correction error and
requires timestep convergence. Friction and inelastic contact should dissipate
energy; testing them against conservation would be incorrect.

## Verified Engine Lessons And DART Inferences

Scope decision (2026-09-04 milestone-planning request): the requested comparisons
explicitly permit named external engines in cited research documentation. This
supersedes the earlier rigid-body task's no-names restriction for these
comparisons. DART solver, preset and example names still describe methods or
capabilities rather than borrowing engine names.

Research checked 2026-09-04. Upstream `latest` pages can describe development
behavior; pin source versions when implementing a comparison. These systems
provide examples, not evidence that DART has implemented their capabilities.

| Engine  | Verified upstream behavior                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | Inference/recommendation for DART                                                                                                                                                                                                                                                                                 |
| ------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Genesis | [Material/model selection](https://genesis-world.readthedocs.io/en/latest/user_guide/physics/beyond_rigid_bodies.html) routes entities to different formulations. [Coupling documentation](https://genesis-world.readthedocs.io/en/latest/user_guide/theory/coupling/index.html) lists supported combinations and distinct legacy/SAP/IPC restrictions. [Checkpoints](https://genesis-world.readthedocs.io/en/latest/user_guide/configuration/checkpoints.html) require a matching scene/model rather than reconstructing the entire model structure.                       | Scene-driven defaults are useful, but broad domain support does not imply arbitrary solver compatibility. DART should validate combinations and provide model/schema fingerprints plus self-contained M1 checkpoints. This is a DART design recommendation, not a claim of universal Genesis coupling or restart. |
| Newton  | [Solver portfolio](https://newton-physics.github.io/newton/stable/solvers/index.html) exposes multiple methods over Model/State/Control/Contacts. [Coupling](https://newton-physics.github.io/newton/latest/concepts/coupling.html) includes explicit and experimental mechanisms with restricted combinations/proxies. [VBD API](https://newton-physics.github.io/newton/latest/api/_generated/newton.solvers.SolverVBD.html) documents evolving method behavior; development documentation is not a stable-release guarantee.                                             | Reuse shared data/contracts while keeping solver choice and supported coupling explicit underneath a simple facade. DART's proposed automatic policy is additional design, not verified Newton behavior. Solver history/reset and variant versioning need explicit contracts.                                     |
| MuJoCo  | [Simulation state](https://mujoco.readthedocs.io/en/stable/programming/simulation.html#state-and-control) separates model/data and includes continuation-relevant state. [Reproducibility](https://mujoco.readthedocs.io/en/stable/computation/index.html#reproducibility) has version/platform limits. [Flex objects](https://mujoco.readthedocs.io/en/stable/modeling.html#deformable-objects) support multiple intrinsic dimensions; [fluid forces](https://mujoco.readthedocs.io/en/stable/computation/fluid.html) are reduced force models, not a resolved CFD domain. | Enforce physical model/state separation and a narrow, tested formulation. PGS/CG/Newton alternatives within a contact formulation should not be confused with arbitrary physical-domain solvers. Distinguish aerodynamic forces from resolved flow and same-backend replay from cross-device tolerance.           |

No evidence here establishes that one engine or solver is universally superior,
or that named domains automatically couple accurately. Compare matched scenes,
physical laws, precision, convergence and complete costs.

## M1 Minimum And Later Commitments

M1 designs capability/version identity, full rigid ownership, numerical/error
contracts, graph completion and checkpoint semantics. It implements one complete
rigid path on CPU/CUDA, independent two-state use, simple public workflows and
honest rejection. A tiny nonstepping variable-block test challenges layout
extensibility; a second physics domain is unnecessary for that purpose.

M2 validates many-body graphs and scaling; M3 qualifies explicit compatible
method substitution and installed research workflows. M4 prototypes bidirectional
coupling before a general coupling API hardens. M5/M6 admit new representations
and formulations with their own evidence. Public solver plugins/ABI, arbitrary
automatic composition, universal gradients and all-domain checkpoints remain
separate decisions. The architecture is audited continuously, not frozen by M1.
