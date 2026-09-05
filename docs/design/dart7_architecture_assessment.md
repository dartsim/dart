# DART 7 Architecture Assessment

## Status And Evidence Boundary

Living source audit refreshed on **2026-09-04**, at parent PR
[#3479](https://github.com/dartsim/dart/pull/3479), revision
`afe6c7d5212d6a2fa3a4b56a2d72701ea84b00cd`. Source and test definitions were
inspected; this inventory does not assert those tests were executed by the
review. Historical PLAN-091 work established useful contracts, but its
closeout did not close all architectural gaps. Active readiness ownership is
[PLAN-040](../plans/040-dart7-release-hardening.md).

**Implemented** means a source path and scoped test exist. **Partial** means
useful implementation exists but the stated contract has uncovered cells.
**Planned** means accepted work without qualifying implementation/evidence.
**Undecided** means an explicit design/prototype decision remains. These are
capability assessments, separate from dashboard initiative status. A prototype
or passing broad suite cannot establish a wider supported envelope.

The central gap is integration and qualification of existing pieces: DART has
substantial CPU solvers, model/state components, metrics, replay, graph
execution and CUDA kernels. It does not yet meet the complete M1 CPU/CUDA
rigid-example, state-ownership and checkpoint contract. DART 7 is a new engine;
DART 6 behavior is comparison evidence rather than the definition of physics.

## Standing Audit Rule

At packet intake and acceptance, examine affected model/state, solver,
coupling, compute, precision, checkpoint, public API and evidence invariants.
Record the source baseline, changed invariants, tests and unresolved owners in
an **Architecture impact** field. New families and major components use the
[solver-family intake](../plans/solver-family-intake.md). Milestone exits,
new coupling, changed storage/layout, execution ordering, result-affecting
state, and public selection trigger a cross-family audit. Update this document
and affected capability/allocation matrices in the same change.

An unchanged invariant can cite prior evidence with a reason it remains valid.
A resolved finding is narrowed or removed with evidence; do not preserve false
absence claims as historical guidance. Closed PLAN-020/091 work is background,
not an active implementation owner.

## Current Findings

### F1 — Partial Solver Lifecycle And Composition Contract

[`WorldStepStage`](../../dart/simulation/compute/world_step_stage.hpp) now has
virtual preflight/prepare operations. The
[step schedule](../../dart/simulation/detail/world_step_schedule.hpp) consumes
limited family capabilities; [schedule tests](../../tests/unit/simulation/world/test_world_step_schedule.cpp)
cover ordering. Canonical contact constants/assembly and separately owned
AVBD joint stiffness also exist. The former non-virtual preparation and fused
`Joint.avbd*` absence findings are obsolete.

What remains: capabilities are not yet a complete scene-to-solver/coupling
contract covering domains, representation, interaction ownership, backend,
precision and continuation. Existing validation and resolved configuration in
[`world.cpp`](../../dart/simulation/world.cpp) reject several unsupported
combinations and distinguish selection sources; they do not prove arbitrary
multi-domain composition. **Owner: PLAN-080, coordinated by PLAN-040.**

### F2 — Partial Physical Model And State Ownership

[Joint](../../dart/simulation/comps/joint.hpp),
[link](../../dart/simulation/comps/link.hpp) and
[deformable](../../dart/simulation/comps/deformable_body.hpp) components separate
model/state/control data. [BakedWorldModel](../../dart/simulation/detail/world_storage.hpp)
contains dense rigid/multibody indices; serialization uses
[stable component IDs](../../dart/simulation/comps/component_category.hpp).
The [World tests](../../tests/unit/simulation/world/test_world.cpp) include
`BatchedRigidBodyIntegrationStageReusesBakedModel` and
`BakedRigidBodyBatchOwnerReusesModelAcrossRolloutSegments`; these prove the
scoped baked-batch reuse behavior. The split is more than conceptual.

Remaining gaps: the baked model is World-owned and lacks a deformable block;
[rigid batch storage](../../dart/simulation/compute/rigid_body_state_batch.cpp)
replicates model arrays per lane. M1 must demonstrate one rigid model with two
independent states, full orientation/angular velocity, stable index/schema
rules, lifetime-safe views and independent controls/history/scratch. A tiny
variable-block layout test is sufficient for the M1 extensibility challenge;
it need not implement a second physics domain.
**Owners: WP-080.1; PLAN-122 for allocation evidence.**

### F3 — Partial Compute Execution; Complete M1 CUDA Stepping Missing

[ParallelExecutor](../../dart/simulation/compute/parallel_executor.cpp) executes
Taskflow graphs and caches lowering of the most recent graph shape.
Kinematics caches a graph; rigid integration and
[WorldBatch](../../dart/simulation/compute/world_batch.cpp) construct graphs per call.
Resource metadata diagnoses hazards; it does not infer dependencies, and
`Reduce`/`Reduce` compatibility alone does not implement safe reductions.

Default SI velocity/position integration in
[rigid_body_integration_stage.cpp](../../dart/simulation/compute/rigid_body_integration_stage.cpp)
and collision/contact solve in
[rigid_body_contact_stage.cpp](../../dart/simulation/compute/rigid_body_contact_stage.cpp)
remain host paths. The resident
[CUDA rigid kernel](../../dart/simulation/compute/cuda/rigid_body_state_batch_cuda.cu)
integrates force with constant angular velocity; it is not the full torque and
contact pipeline. [CUDA tests](../../tests/unit/simulation/cuda/test_rigid_body_state_batch_cuda.cpp)
cover integration/resident transfer behavior, not all M1 examples.

The executor owns both a Taskflow pool and a separate range pool. Nested ranges
fall inline, while graph execution shares mutable cache/profiler state without
an equivalent guard. The
[profiling adapter](../../dart/simulation/compute/world_step_pipeline.cpp)
does not forward `parallelFor`, so instrumented execution can become serial.
Nodes are synchronous host closures. Resident CUDA stepping synchronizes the
device; generic asynchronous completion edges are not implemented. An isolated
[VBD CUDA rollout](../../dart/simulation/compute/cuda/vbd_block_descent_cuda.cu)
already captures/replays a CUDA graph, which does not qualify World execution.

**Owner: WP-030.1 through WP-030.5.** M1 fixes total worker budgets, graph/state
ownership, completion and failure semantics, then qualifies every RB example
on CUDA Float64. Explicit ordered execution and synchronous completion are
acceptable. M2 expands automatic dependency inference and useful task grouping.

### F4 — Implemented Comparison Primitives; Bounded Coverage

Public [StepMetrics](../../dart/simulation/compute/world_step_profile.hpp),
resolved configuration, [cross-family metrics](../../tests/unit/simulation/compute/test_cross_family_metrics.cpp)
and a [shared corpus](../../tests/unit/simulation/compute/test_cross_family_corpus.cpp)
exist. Earlier claims of no metrics or only wall-clock comparisons are false.
These tests cover selected scenes/families, not universal method or backend
parity. The VBD/AVBD paper matrix still contains partial/missing rows; older
unbound packets are not current-build performance evidence.

M1 needs independent physical oracles so CPU/GPU agreement cannot validate a
shared error. M2 needs an end-to-end current-build 100-body packet; the audited
small-box and older larger AVBD packets do not provide that claim. Record
resolved algorithm/variant, scene fingerprint, accuracy and actual execution
path alongside setup, step and transfer costs. **Owners: PLAN-040/030/080.**

### F5 — Partial State And Checkpoint Contracts

The general state-vector API in [world.hpp](../../dart/simulation/world.hpp)
covers rigid translation/linear velocity and multibody coordinates, but omits
free-rigid orientation/angular velocity and deformables. It is not yet a full
simulation state-vector contract. The rigid batch already stores seven pose
scalars and six velocity scalars, whereas
[StateSpace](../../dart/simulation/space/state_space.hpp) describes named flat
vectors without a configuration/tangent distinction. WP-080.1 must record
configuration, velocity and control dimensions independently before this API
expands; six rigid DOFs do not imply six quaternion pose-storage scalars.

Replay in [world.cpp](../../dart/simulation/world.cpp) includes substantial
continuation state, including AVBD, with restore tests in
[test_world.cpp](../../tests/unit/simulation/world/test_world.cpp). Binary
snapshots explicitly reject populated AVBD continuation and stepped opaque
custom stages. Stable IDs and replay are useful seeds; portable all-domain,
all-family checkpointing remains partial. M1 qualifies only the full supported
SI scene across CPU→CPU, CUDA→CUDA, CPU→CUDA and CUDA→CPU fresh-process
continuation. **Owner: WP-041.1.**

### F6 — Clean-Break Boundaries And Scoped Foundation Evidence

Legacy Skeleton [loading bridges](../../dart/simulation/io/skeleton_loader.hpp)
remain, while a [USD parser](../../dart/io/usd/usd_parser.cpp) and
[default-step goldens](../../tests/unit/simulation/world/test_world_default_step_golden.cpp)
exist. Do not claim all standard loaders are absent, no golden infrastructure
exists, duplicate active plan IDs persist, or all CUDA kernels use Float32.
Existing sensors do not prove a complete World-native observation pipeline;
qualify that workflow in M3. **Owners: PLAN-040/041/042 and admitted subsystem
plans.**

Broad first-post-bake zero-allocation tests exist in `test_world.cpp`; the
[PLAN-122 matrix](../plans/122-simulation-loop-allocation-hardening/coverage-matrix.md)
records the current row statuses. That is scoped evidence. Parallel unified islands
allocate fresh vector/per-island scratch in
[unified_constraint.cpp](../../dart/simulation/compute/unified_constraint.cpp).
The [cached graph allocation test](../../tests/unit/simulation/compute/test_compute_graph.cpp)
accepts a constant Taskflow submission allocation floor, not zero allocation.
Function-static CUDA PSD buffers in
[deformable_psd_projection_cuda.cpp](../../dart/simulation/compute/cuda/deformable_psd_projection_cuda.cpp)
also warrant an ownership/concurrency audit before wider multi-state claims.
**Owners: PLAN-122 R-005/G-002 and PLAN-030 for the already selectable
parallel-island and deformable-device paths; WP-122.8 qualifies only the selected
M1 pipeline.** Closing M1 cannot retire those wider findings. The compute
decision owns runtime viability, including the M1 submission-allocation gap.

## Early Decisions, Prototypes And Deferrals

| Timing                        | Required decision or evidence                                                                                                                                                      | Owner                                     |
| ----------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------- |
| Before broader implementation | Independent numerical budgets; units/frames; full rigid model/state and stable schema; precision and determinism tiers                                                             | WP-040.1, WP-080.1                        |
| M1 prototypes                 | Runtime/kernel shortlist; worker/nesting budget; dependency/alias and completion semantics; first-step allocation including submission; failure cleanup                            | WP-030.1–030.4                            |
| M1 integration                | General rotation, friction/contact law, complete CPU/CUDA stepping, portable continuation and simple installed workflows                                                           | WP-080.2/080.3, WP-030.5, WP-041.1        |
| M2                            | Many-body broadphase/capacity, graph groups and coarsening, asynchronous overlap and cost-guided scheduling where measured useful                                                  | PLAN-080/030/122                          |
| Before first coupling         | Interaction ownership, exchanged quantities, conservation/work budgets, solver capability selection and state history                                                              | PLAN-081/083/104 and solver intake        |
| Defer beyond M1               | Universal solver selection, all-domain checkpoints, public plugin ABI, distributed runtime, multiple production GPU vendors, Float32/mixed precision, differentiating every solver | Successor owner required before admission |

Differentiation and other advanced paths already have scoped implementations;
deferral here concerns universal coverage, not deletion. M1 should preserve
room for explicit differentiability capabilities, reproducible seeds, failure
diagnostics, unit/frame conventions, contact capacity and topology changes,
observations and installed-package support without building every feature.

## Related Owners

[Solver architecture](simulation_solver_architecture.md) owns the portfolio and
verified competitor lessons; [compute research](compute_backend_research.md)
and [compute decisions](scalable_compute_decisions.md) own library evidence and
graph contracts; [C++](simulation_cpp_api.md) and
[Python](simulation_python_api.md) own facade design;
[device residency](batched_world_device_residency.md) owns state/residency rules;
[release strategy](dart7_clean_break_strategy.md) owns the support/package split.
Reconcile contradictory claims in the owning documents when this audit changes.
