# DART 7 Architecture Assessment (2026-06)

## Status

Living assessment record. This document owns the durable, evidence-backed
findings of the June 2026 DART 7 architecture review and the standing
architecture rules derived from them. PLAN-091 closed its living work-packet
plan in [`../plans/dashboard.md`](../plans/dashboard.md), but that closeout
does not mark every verified architecture gap below as resolved in code. When
a finding is resolved, update or remove its entry here and record the evidence;
this file should always describe the current verified state, not a historical
snapshot.

PLAN-091 closeout folded the completed packet plan into durable owner docs
instead of keeping the numbered plan as an archive. The durable operating
state is now: this assessment owns the verified findings and standing rule;
[`../plans/solver-family-intake.md`](../plans/solver-family-intake.md) owns the
new-family gate; [`simulation_solver_architecture.md`](simulation_solver_architecture.md)
and
[`algorithm_extension_contracts.md`](algorithm_extension_contracts.md) own the
solver-contract architecture; [`batched_world_device_residency.md`](batched_world_device_residency.md)
owns the batched/device-residency contract; and
[`../plans/042-dart7-public-api-and-source-layout.md`](../plans/042-dart7-public-api-and-source-layout.md)
owns the DART 6 legacy quarantine/removal sequence. The closeout PR history is
summarized in the dashboard entry.

## How this assessment was produced

A multi-agent review mapped every DART 7 subsystem from code (docs were
treated as claims to check, headers as truth), reviewed eight architecture
dimensions, and adversarially verified every critical/high finding by
re-reading the cited code and attempting to refute it. Competitor evidence was
gathered for MuJoCo/MJX, Newton/Warp, Drake, Genesis/Taichi, PhysX/Isaac, and
research stacks (IPC Toolkit, Dojo, Brax) plus DART 6 itself. Findings below
are only those that survived verification; verifier corrections are folded in.
File references cite repo-relative paths as of the review date and may drift —
re-verify before acting on a specific line number.

## Verdict

The DART 7 north star — one simulation pipeline generalized over physics
domains × solver method families × compute backends, behind a facade that
exposes only method-family names and policy value objects — is validated by
competitor evidence, not just aspiration. NVIDIA abandoned PhysX's fixed
one-solver-per-domain design and built Newton (a multi-solver
Model/State/Control engine, the closest convergent design to DART 7); Drake's
config-value solver selection enabled a years-long apples-to-apples comparison
that ended in a clean retirement of the losing family; MuJoCo's mjModel/mjData
split is the root enabler of its batching and GPU ports. No competitor
combines all three axes with a research-grade shared substrate.

The gap: the architecture currently exists in the design docs while the code
accretes against it. The four load-bearing findings are listed below. None is
fatal — the repo's own design docs already prescribe most fixes, and clean
seeds exist in-tree — but every new solver family that lands before the
internal contracts exist copies the anti-patterns and raises the migration
cost.

## The standing rule

> A new solver family, paper implementation, or major solver component may
> start only through the solver-family intake checklist
> ([`../plans/solver-family-intake.md`](../plans/solver-family-intake.md)),
> whose items 9–10 bind new families to the architecture contracts landing
> through PLAN-091. Work that bypasses this rule multiplies the cost of every
> axis the north star promises.

## Verified findings

### F1 — No internal Solver contract in code

The documented contract (capabilities / domain / finalize / substep work in
[`simulation_solver_architecture.md`](simulation_solver_architecture.md))
exists nowhere in code. The real mechanism is `WorldStepStage{name, metadata,
execute}` with non-virtual per-class `prepare()` dispatched by hand-rolled
switches that nothing forces to agree, plus a hand-enumerated flat schedule
(`dart/simulation/detail/world_step_schedule.hpp`) capped at 8 inline slots,
already 7/8 deep on its densest branch. Adding one method family costs ~9
files and ~10 hand-synchronized edit sites across two monolithic translation
units (`world_step_stage.cpp` ~10.9k lines, `world.cpp` ~6.2k lines).

The domain selectors now share a typed-enum idiom on the facade: the
`RigidBodySolver` enum (runtime-mutable, alters the schedule), the
`ContactSolverMethod` enum (construction-only, a branch inside one stage), and
the `MultibodyOptions.integrationFamily` enum (`MultibodyIntegrationFamily`,
resolved once at finalize). AVBD rigid contact remains selected by a private ECS
component invisible to the facade. Scene content
silently swaps algorithms (VBD falls back per body to projected Newton on
unsupported scene features with no facade-visible diagnostic; mixed scenes
reroute contact solves across structurally different assemblies).

Rigid contact-row physics is assembled five times (sequential-impulse
scratch, boxed-LCP, unified constraint, AVBD snapshot, and the gradient
path's re-derivation), synchronized by "mirroring" comments, with
already-diverged constants. A canonical assembly seam exists
(`compute::assembleRigidBodyContactProblemInto`) and should become the single
producer. One family's vocabulary leaked into the shared model layer:
`comps::Joint` carries `avbd*` fields, rigid IPC discovers joints through
AVBD's private config component, and ~6.6k lines of rigid/articulated AVBD
machinery live under a deformable-named directory.

### F2 — Model/State/Control/Contacts separation is conceptual only

The component taxonomy (Property/State/Cache/Tag) encodes serialization
behavior, not the data contract. Flagship components fuse model, state,
control, and solver state in one struct (`comps::Joint`, `comps::Link`,
`DeformableNodeState`); the rigid domain and the variational family already
split correctly and prove the pattern. There is no stable dense body/joint
index — state layout is registry iteration order, validated across worlds by
comparing name strings — so one Model cannot serve many States: replicated
worlds (`n_envs`) require full World copies and replay deep-copies the model
every frame. The World-owned frame-scratch arena is reset every step with
zero allocation sites, while hot paths heap-allocate dense matrices per step
and rebuild model quantities (spatial inertias, joint subspaces) the bake
boundary should own. Binary serialization is keyed on `typeid(T).name()`
(compiler-specific, refactor-unstable).

MuJoCo evidence: the mjModel/mjData split enforced as an allocation
discipline — zero allocations after initialization — is the single property
that later enabled thread-per-data sampling, MJX pytree batching, and the
Warp port without redesign. Drake counter-evidence: the System/Context split
alone did not yield batching; scattered per-context heap state blocked it for
years. The split must be physical (dense indices + SoA-capable state slabs),
not just named.

### F3 — The compute axis is currently metadata

All dynamics/contact/constraint stages accept and discard the injected
`ComputeExecutor`; only kinematics (per-frame nodes far below task
profitability) and free-rigid integration execute as multi-node graphs. The
parallelism that exists bypasses the executor seam: the parallel VBD path
spawns per-solve `std::thread` pools (with a public worker-thread knob that
contradicts the documented single-seam invariant), native collision batching
fans out raw threads, and the only live GPU path is a process-global mutable
function pointer installed by a static-initializer registrar. CUDA modules
are per-call packet offload (upload, solve, download); no state survives on
device across steps, and `execute(const ComputeGraph&)` over host
`std::function` closures cannot express residency, streams, or a batch
dimension. The articulated "semi-implicit ABA" family is not ABA: it builds
M(q) by RNEA columns plus dense LDLT and computes dense `M.inverse()` every
step even with zero contacts, while an O(n) articulated inverse-mass apply
sits exported but unused in the variational family.

PhysX evidence (what to avoid): per-feature silent CPU fallbacks and a
mutually exclusive direct-GPU vs CPU API split make CPU runs unfaithful debug
proxies. Newton evidence: determinism must be designed into the compute seam
from day one (deterministic reductions, stable contact sort keys);
retrofitting it cost coordinated changes in three layers.

### F4 — Apples-to-apples comparison has process but no substrate

No cross-family metrics contract exists: the only in-tree solver A/B
comparisons are wall-clock-only; the single accuracy comparison requires
internal registry access. The capability matrix is documentation-only, so a
researcher cannot ask "did the configured method actually run, and does it
support this scene?". There is no shared scene corpus (the same box-stack
scene is hand-built in four files) and no generic harness (dozens of
per-scene packet-writer scripts). Worst, benchmark evidence packets do not
record solver identity, and flagship AVBD contact-scene packets recorded
"DART beats reference" rows on scenes that actually ran the
sequential-impulse contact path, because AVBD contact is not
facade-selectable. Evidence integrity is the cheapest and most urgent fix:
machine-record resolved solver configuration in every packet, and relabel the
affected rows.

### F5 — Facade erosion at the edges

The easy path genuinely delivers in both C++ and Python, but PLAN-091 WS4 was
needed to remove several compatibility liabilities before adoption. The
general-named `getNumDofs`/`getStateVector`/`getControlVector` now reserve the
full dense world semantics (dynamic rigid-body translations, then multibody
joint coordinates), and the prior translational-rigid-only slice is explicit
behind `getRigidBodyStateVector` / `getRigidBodyControlVector`. PR #3072 also
removed solver-family vocabulary from shared handles, moved built-in stage
internals out of the installed public header set, aligned the Python surface,
and collapsed joint construction onto the `JointSpec`-based verb. Keep future
facade additions on those scoped names and policy value objects; avoid
reintroducing solver-branded shared handles or internal stats on installed
headers.

### F6 — Clean-break hygiene and doc truth

Legacy DART 6 trees continue to accrete new public features while the
quarantine boundary (PLAN-042 Decision 5) stays open, and coupling is
bidirectional (legacy constraint code includes DART 7 collision machinery;
DART 7 loaders, demos, and GUI descriptors depend on `dart/dynamics` types).
The flagship architecture page overstated availability markers (substep
coupling phases, persistent-manifold/SDF pipeline integration, SIMD as a
consumed backend, graph coverage); those markers are corrected as of this
assessment, and ✅ claims should be CI-checkable against header symbols and
tests going forward. The planning corpus carries duplicate plan IDs (two
PLAN-080s, two PLAN-082s) that grep-by-ID edits can mismatch.

## Coverage gaps beyond the findings

Verified as absent rather than broken: an observation/sensor seam for the
DART 7 World (RL and batching need one; competitors confine extension points
to such leaf stages); a DART-7-native scene IO story (all standard-format
loaders return legacy skeletons across a lossy one-way bridge; no USD); an
error-handling/diagnostics channel on `World::step` (void return, no failure
reporting outside per-family stats structs); thread-sanitizer coverage for
the existing thread pools and batched stepping; a numerical-precision policy
(public API hardwires double while CUDA kernels run float); a CUDA
distribution story (wheels never carry GPU support); per-family
ownership/retirement mechanics (Drake's lesson: families get deprecated and
defaults migrate); and golden-trajectory regression infrastructure to lock
behavior before the refactors above (a prerequisite — current unit tolerances
are too loose to catch silent physics changes).

## Competitor lessons (evidence-backed)

| Engine                             | Lesson for DART 7                                                                                                                                                                                                                                                                                                                                                                                                              |
| ---------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| MuJoCo / MJX                       | Make the Model/State split an enforced no-alloc-in-step discipline; pin the per-family problem contract so solver swap is an enum; confine plugins to leaf stages (sensors, actuators); design contact buffers with capacity-vs-count separation for fixed-shape backends.                                                                                                                                                     |
| Newton / Warp                      | Closest convergent design (Model/State/Control/Contacts + solver base + shared collision); its pain points are the warning list: solver names leaking into shared state, determinism retrofit, per-solver feature support documented as footnotes instead of tested artifacts.                                                                                                                                                 |
| Drake                              | Config-value solver selection plus a written internal-stability boundary enables side-by-side comparison and clean family retirement; System/Context separation alone does not yield batching — SoA layout is the missing half.                                                                                                                                                                                                |
| PhysX / Isaac                      | One-solver-per-domain is a documented dead end; per-feature silent CPU fallbacks are the anti-pattern; tiered honest determinism contracts are the right shape; the batched RL API expectation is fixed: leading `(num_envs, …)` dimension with indexed partial resets and control buffered separately from state writes.                                                                                                      |
| Genesis / Taichi                   | Do not make a third-party JIT substrate load-bearing; declare differentiability per family with gradient tests; benchmark overclaiming is independently falsified within days — machine-recorded solver identity is reputation insurance.                                                                                                                                                                                      |
| IPC Toolkit / Dojo / Brax / DART 6 | A shared contact substrate across families works when one owner also holds the orchestration protocol; expect the internal contact decomposition to be wrong the first time, so keep it internal; monolithic method coupling is unmaintainable; one weak family behind a shared API poisons trust in all of them; keep extensibility data-shaped (components/columns), never type-shaped (the DART 6 Aspect/Composite lesson). |

## Resolution map

Each finding was executed through PLAN-091 work packets; this table is a
durable closeout pointer, not an active tracker.

| Finding                             | Direction                                                                                                                                            | Executed by  |
| ----------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------- | ------------ |
| F4 evidence integrity, F6 doc truth | Relabel affected packets, machine-record solver identity, CI-checkable availability claims, golden trajectories, legacy freeze                       | PLAN-091 WS0 |
| F1 solver contract                  | Internal detail-tier solver contract, single selection idiom, resolved-configuration report, canonical contact assembly, family-scoped source layout | PLAN-091 WS1 |
| F2 data architecture                | Component split, baked dense-index Model artifact, arena wire-or-delete, stable serialization IDs                                                    | PLAN-091 WS2 |
| F3 compute reality                  | Executor parallel-for primitives, O(n) shared articulated core, batched-World owner contract, device-resident state design                           | PLAN-091 WS3 |
| F5 facade                           | Reserved state-vector semantics, `JointSpec` construction, policy objects for solver knobs, installed-header split                                   | PLAN-091 WS4 |

## Relationship to other owner docs

This assessment records findings and standing rules. It does not replace:
[`simulation_solver_architecture.md`](simulation_solver_architecture.md)
(target solver/coupling architecture),
[`simulation_cpp_api.md`](simulation_cpp_api.md) and
[`simulation_python_api.md`](simulation_python_api.md) (public API shape),
[`algorithm_extension_contracts.md`](algorithm_extension_contracts.md)
(research extension contracts),
[`scalable_compute_decisions.md`](scalable_compute_decisions.md) (compute
decision framework), or
[`dart7_clean_break_strategy.md`](dart7_clean_break_strategy.md) (release
topology). Where this assessment and an owner doc disagree, reconcile the
owner doc and update both in the same change.
