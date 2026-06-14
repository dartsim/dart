# Hierarchical Memory Manager — Dev Task

## Hard Stop Handoff (2026-06-14, Nested Profile Storage Reuse)

Checkpoint PR #2996 (`Reuse profiled World step snapshots`) is open against
`main` from `pr/hmm-phase45-replay-snapshot-allocators` at `97733411196`, with
milestone `DART 7.0`. Live GitHub recon on 2026-06-14 reported it is no longer
draft and has merge state `BLOCKED`. Treat that PR as the checkpoint for the
profile snapshot reuse slice; do not add follow-up work to that branch unless a
maintainer explicitly redirects the work.

Continue current HMM work from the local follow-up branch
`pr/hmm-phase45-followup-allocation-slice`, based on checkpoint head
`97733411196`. This branch is local only at the time of this handoff. Do not
push it or open a follow-up PR without explicit maintainer approval.

Latest local slice: profiled `World::step()` now reuses nested same-shape
compute-graph profile storage for built-in inline graph execution. The
world-step profiling wrapper preserves warmed `WorldStepStageProfile`
`graphProfiles` slots instead of clearing them every step, and built-in
sequential/inline parallel executor profiling can fill an existing
`ComputeExecutionProfile` so node-profile storage and critical-path scratch are
reused.

The focused proof is
`WorldStepProfileIntegration.WarmedNestedGraphProfileDoesNotAllocate`, which
warms a profiled `World` with a kinematics graph, runs a third profiled
`World::step()` through `ParallelExecutor`, and verifies zero global-heap
allocations while preserving top-level stage, nested graph-profile, and nested
node-profile capacities. This closes the warmed same-shape inline nested
compute-graph profile gap for built-in World stages. It does not claim
allocation-free profiling for Taskflow-backed non-inline parallel graphs, public
return-by-value profile copies, profile summary rendering, or arbitrary custom
executors that only implement the legacy return-by-value profiling API.

Current next-slice notes: start from a fresh measured gap rather than older
historical candidates. Source inspection on this branch shows the previously
listed `World::saveBinary()` ignored-pair staging vector now uses the
`ignoredCollisionPairs` allocator, live `DeformableNodeState` payloads are
allocator-backed and emplaced with the World allocator, and `comps::Joint`
stores bounded `JointVector` payloads instead of raw dynamic `Eigen::VectorXd`
fields. Remaining work should be selected from a failing no-growth/no-heap gate
or a newly proven DART-owned stage scratch gap; do not broaden public
return-by-value convenience APIs unless built-in World code consumes that
storage.

Focused validation for the current follow-up slice:

```bash
pixi run cmake --build build/default/cpp/Release --target test_world_step_profiling -j 8
pixi run build/default/cpp/Release/bin/test_world_step_profiling \
  --gtest_filter=WorldStepProfileIntegration.WarmedNestedGraphProfileDoesNotAllocate
pixi run build/default/cpp/Release/bin/test_world_step_profiling
pixi run cmake --build build/default/cpp/Release --target test_compute_graph -j 8
pixi run build/default/cpp/Release/bin/test_compute_graph
```

Latest local validation before commit:

```bash
pixi run lint
git diff --check
DART_PARALLEL_JOBS=8 pixi run build
DART_PARALLEL_JOBS=8 pixi run -e cuda test-all
```

Historical checkpoint validation for PR #2996:

```bash
pixi run lint
git diff --check
pixi run build
pixi run cmake --build build/default/cpp/Release --target test_world_step_profiling -j 8
pixi run build/default/cpp/Release/bin/test_world_step_profiling
pixi run -e cuda cmake --build build/cuda/cpp/Release --target test_world_step_profiling -j 8
pixi run -e cuda build/cuda/cpp/Release/bin/test_world_step_profiling
```

## Historical Slices Below

The sections below are retained as chronological evidence for previous HMM
slices. They are not current instructions. A fresh agent should use the top
hard-stop section as the authoritative handoff surface.

## Hard Stop Handoff (2026-06-13, Unified Constraint Scratch Vector Aliases)

Resume from exactly one branch:
`pr/hmm-phase45-replay-snapshot-allocators`, tracking
`origin/pr/hmm-phase45-replay-snapshot-allocators`. This remains the single
HMM handoff entry point unless a maintainer explicitly redirects the work.
The branch currently has no open PR. It includes `origin/main` at
`a122e8e0f3e` via local merge commit `57c8b1cd608`; a fresh resume recon found
the branch clean and still ahead of its remote tracking branch.

Latest local slice: `UnifiedConstraintSolveScratch` now gives its allocator
backed scratch vectors local `IndexVector`, `CharVector`, `DoubleVector`, and
`SizeVector` aliases instead of repeating `std::vector<T, Allocator>` for each
field. This keeps future unified-scratch allocator spelling changes localized
while preserving the existing field names and allocator provenance.

This is a mechanical maintainability slice. It does not change solve behavior,
public APIs, allocator provenance, or the existing unified-constraint
no-growth/no-heap claims.

Validation for this slice:

```bash
pixi run cmake --build build/default/cpp/Release --target test_unified_constraint -j 8
pixi run build/default/cpp/Release/bin/test_unified_constraint \
  --gtest_filter=UnifiedConstraint.SolveScratchVectorsUseProvidedAllocator
pixi run lint
git diff --check
```

Before publishing or opening a PR from this branch, get explicit maintainer
approval before pushing.

## Historical Slices Below

The sections below are retained as chronological evidence for previous HMM
slices. They are not current instructions. A fresh agent should use the top
hard-stop section as the authoritative handoff surface.

## Hard Stop Handoff (2026-06-13, Replay Scratch Vector Aliases)

Resume from exactly one branch:
`pr/hmm-phase45-replay-snapshot-allocators`, tracking
`origin/pr/hmm-phase45-replay-snapshot-allocators`. This remains the single
HMM handoff entry point unless a maintainer explicitly redirects the work.
The branch currently has no open PR. It includes `origin/main` at
`a122e8e0f3e` via local merge commit `57c8b1cd608`; a fresh
`git fetch origin main && git merge origin/main` reported "Already up to date."

Latest local slice: replay capture/restore helper scratch vectors now use local
`ReplayScratchAllocator<T>` and `ReplayScratchVector<T>` aliases instead of
repeating `std::vector<T, common::StlAllocator<T>>` at each helper call site.
This follows the surrounding replay-state allocator-vector pattern and keeps
future allocator spelling changes localized.

This is a mechanical maintainability slice. It does not change allocator
provenance, replay semantics, public APIs, or the existing replay no-global-heap
contract.

Validation for this slice:

```bash
pixi run cmake --build build/default/cpp/Release --target test_world -j 8
pixi run build/default/cpp/Release/bin/test_world \
  --gtest_filter='World.WorldPersistentStorageUsesWorldFreeAllocator:World.Replay*'
pixi run lint
git diff --check
```

Before publishing or opening a PR from this branch, get explicit maintainer
approval before pushing.

## Historical Slices Below

The sections below are retained as chronological evidence for previous HMM
slices. They are not current instructions. A fresh agent should use the top
hard-stop section as the authoritative handoff surface.

## Hard Stop Handoff (2026-06-13, AVBD Rigid-World Motor Row Gates)

Resume from exactly one branch:
`pr/hmm-phase45-replay-snapshot-allocators`, tracking
`origin/pr/hmm-phase45-replay-snapshot-allocators`. This remains the single
HMM handoff entry point unless a maintainer explicitly redirects the work.
The branch currently has no open PR. It includes `origin/main` at
`a122e8e0f3e` via local merge commit `57c8b1cd608`; a fresh
`git fetch origin main && git merge origin/main` reported "Already up to date."

Latest local slice: World-level rigid AVBD velocity-motor scenes now exercise
both angular motor rows from a revolute joint and linear motor rows from a
prismatic joint through the same baked-step allocation gates used for contact,
fixed-joint, and distance-spring rows.

The proof has these parts:

- `configureRigidAvbdRevoluteMotorRowsScene(...)` builds a collision-free
  rigid AVBD revolute velocity actuator scene and verifies the motor row moves
  the child body;
- `configureRigidAvbdPrismaticMotorRowsScene(...)` builds the corresponding
  prismatic velocity actuator scene and verifies the linear motor row moves the
  child body;
- `World.RigidAvbdRegistryStorageRebuildsAfterClear` now includes both motor
  scenes, proving the point-joint ECS storage still rebuilds cleanly after
  `World::clear()`;
- `World.BakedStepsDoNotGrowWorldBaseAllocatorForReservedEcsPaths` now includes
  both motor scenes, proving baked World steps do not grow the World base
  allocator for reserved rigid AVBD motor-row paths;
- `World.BakedRigidBodyContactStepsDoNotAllocateGlobalHeap` now includes both
  motor scenes, proving the baked rigid AVBD motor-row World paths do not touch
  the global heap.

This is an evidence slice. It does not change production allocator plumbing,
does not claim every AVBD helper overload is allocation-free, and does not
expand the claim beyond the baked World-level rigid AVBD motor-row paths under
the tested scenes.

Validation for this slice:

```bash
pixi run cmake --build build/default/cpp/Release --target test_world -j 8
pixi run build/default/cpp/Release/bin/test_world \
  --gtest_filter='World.RigidAvbdRevoluteMotorRowsAreActiveWithoutContacts:World.RigidAvbdPrismaticMotorRowsAreActiveWithoutContacts:World.RigidAvbdRegistryStorageRebuildsAfterClear:World.BakedStepsDoNotGrowWorldBaseAllocatorForReservedEcsPaths:World.BakedRigidBodyContactStepsDoNotAllocateGlobalHeap'
pixi run lint
git diff --check
```

Before publishing or opening a PR from this branch, get explicit maintainer
approval before pushing.

## Historical Slices Below

The sections below are retained as chronological evidence for previous HMM
slices. They are not current instructions. A fresh agent should use the top
hard-stop section as the authoritative handoff surface.

## Hard Stop Handoff (2026-06-13, AVBD Rigid-World Step Fallback Scratch)

Resume from exactly one branch:
`pr/hmm-phase45-replay-snapshot-allocators`, tracking
`origin/pr/hmm-phase45-replay-snapshot-allocators`. This remains the single
HMM handoff entry point unless a maintainer explicitly redirects the work.
The branch currently has no open PR. It includes `origin/main` at
`a122e8e0f3e` via local merge commit `57c8b1cd608`.

Latest local slice: AVBD rigid-world no-scratch step/solve fallback helpers now
derive their internally owned snapshot, row-build scratch, solve scratch,
registry-extracted point-joint and distance-spring inputs, and internally
created row inventories from the allocator already attached to the caller's
normal-row inventory. This keeps registry-based AVBD fallback step storage on
the caller-provided DART allocator when the caller supplied allocator-backed
row inventories.

The fix has these parts:

- `AvbdScalarRowInventory` can be constructed from a converted
  `StlAllocator`, so overload-created inventories can borrow an existing row
  inventory allocator;
- AVBD rigid row-index, contact-manifold, point-joint, motor, and
  distance-spring scratch objects accept converted `StlAllocator`s for their
  local vectors;
- `AvbdRigidWorldContactSnapshot`,
  `AvbdRigidWorldContactBuildScratch`, and
  `AvbdRigidWorldContactSolveScratch` accept converted `StlAllocator`s for
  their allocator-backed containers;
- no-scratch `solveAvbdRigidWorldContactSnapshot(...)` overloads construct
  internal solve scratch and missing row inventories from
  `normalInventory.records().get_allocator()`;
- registry-based no-scratch `runAvbdRigidWorldContactStep(...)` overloads
  construct extracted point-joint inputs, extracted distance-spring inputs,
  internal row inventories, the contact snapshot, and solve scratch from the
  same caller inventory allocator;
- `AvbdRigidBlock.RigidWorldContactStepFallbackUsesInventoryAllocator`
  pre-reserves the caller-visible row inventories and verifies the fallback
  step helper still routes internal allocations through that inventory
  allocator.

This still does not claim that return-by-value AVBD registry extractors are
allocation-free, that default-constructed inventories use anything other than
the default allocator, that explicit caller-provided input vectors changed
behavior, that Eigen internals are allocator-backed, or that the whole AVBD
step path is globally allocation-free. The closed gap is allocator provenance
for registry-based no-scratch AVBD rigid-world fallback helpers when a caller
already provided allocator-backed row inventories.

Validation for this slice:

```bash
pixi run cmake --build build/default/cpp/Release --target test_avbd_rigid_block -j 8
pixi run build/default/cpp/Release/bin/test_avbd_rigid_block \
  --gtest_filter=AvbdRigidBlock.RigidWorldContactStepFallbackUsesInventoryAllocator
pixi run build/default/cpp/Release/bin/test_avbd_rigid_block
pixi run lint
git diff --check
pixi run build
pixi run test-unit
pixi run -e cuda test-all
```

The fresh merged-tree CUDA `test-all` run passed with the existing no-GUI
`dartpy._world_render_bridge` autodoc warnings during documentation generation
and CPU-scaling warnings during CUDA benchmark smoke. The long CUDA simulation
tests completed successfully (`test_rigid_ipc_paper_experiments`,
`test_world`, and `test_lcp_jacobi_batch_cuda` were the slow tests in this
run).

Before publishing or opening a PR from this branch, get explicit maintainer
approval before pushing.

## Historical Slices Below

The sections below are retained as chronological evidence for previous HMM
slices. They are not current instructions. A fresh agent should use the top
hard-stop section as the authoritative handoff surface.

## Hard Stop Handoff (2026-06-13, Contact Candidate Sweep Fallback Scratch)

Resume from exactly one branch:
`pr/hmm-phase45-replay-snapshot-allocators`, tracking
`origin/pr/hmm-phase45-replay-snapshot-allocators`. This remains the single
HMM handoff entry point unless a maintainer explicitly redirects the work.
The branch currently has no open PR.

Latest local slice: deformable contact candidate no-scratch sweep builders now
derive their short-lived `ContactCandidateSweepScratch` from the allocator
already attached to `ContactCandidateSet::surfaceEdges`. This keeps fallback
point-item, triangle-item, edge-item, and sweep-link temporaries on the
caller-provided DART allocator when the caller supplied an allocator-backed
candidate set.

The fix has these parts:

- `ContactCandidateSet` uses local `SurfaceEdgeVector`,
  `PointTriangleVector`, and `EdgeEdgeVector` aliases for its allocator-backed
  result vectors;
- `ContactCandidateSweepScratch` uses local `SweepItemVector` and
  `SweepLinkVector` aliases for its allocator-backed temporary vectors;
- `ContactCandidateSweepScratch` can be constructed from a candidate-set
  surface-edge allocator, converting that allocator to its sweep-item and
  sweep-link vector allocators;
- the no-scratch `buildContactCandidatesSweep(..., candidates)` fallback now
  builds scratch from `candidates.surfaceEdges.get_allocator()`;
- the no-scratch `buildMotionAwareContactCandidatesSweep(..., candidates)`
  fallback now borrows the same candidate-set allocator for its internal sweep
  scratch;
- `IpcContactCandidateSet.NoScratchSweepBuildersBorrowCandidateAllocator`
  verifies both fallback builders route local sweep-scratch allocations through
  the candidate-set allocator after pre-reserving result storage.

This still does not claim that return-by-value candidate-set helpers are
allocation-free, that fallback scratch is retained across calls, that explicit
scratch callers changed behavior, or that unrelated deformable/contact builder
payloads are allocator-backed. The closed gap is allocator provenance for the
no-scratch deformable contact candidate sweep fallback helpers when a caller
already provided an allocator-backed candidate set.

Validation for this slice:

```bash
pixi run cmake --build build/default/cpp/Release --target test_contact_candidate_set -j 8
pixi run build/default/cpp/Release/bin/test_contact_candidate_set \
  --gtest_filter=IpcContactCandidateSet.NoScratchSweepBuildersBorrowCandidateAllocator
pixi run build/default/cpp/Release/bin/test_contact_candidate_set
pixi run lint
git diff --check
pixi run build
pixi run test-unit
pixi run -e cuda test-all
```

The CUDA `test-all` run passed with the existing no-GUI
`dartpy._world_render_bridge` autodoc warnings during documentation generation
and CPU-scaling warnings during CUDA benchmark smoke.

Before publishing or opening a PR from this branch, get explicit maintainer
approval before pushing.

## Historical Slices Below

The sections below are retained as chronological evidence for previous HMM
slices. They are not current instructions. A fresh agent should use the top
hard-stop section as the authoritative handoff surface.

## Hard Stop Handoff (2026-06-13, AVBD Rigid-World Fallback Build Scratch)

Resume from exactly one branch:
`pr/hmm-phase45-replay-snapshot-allocators`, tracking
`origin/pr/hmm-phase45-replay-snapshot-allocators`. This remains the single
HMM handoff entry point unless a maintainer explicitly redirects the work.
The branch currently has no open PR.

Latest local slice: AVBD rigid-world no-scratch fallback helpers now derive
their short-lived `AvbdRigidWorldContactBuildScratch` from the allocator already
attached to `AvbdRigidWorldContactSnapshot::contacts`. This keeps fallback
row-order and row-counter temporaries on the caller-provided DART allocator
when the caller supplied an allocator-backed snapshot.

The fix has these parts:

- `AvbdRigidWorldContactBuildScratch` uses local `RowCounterVector` and
  `RowOrderVector` aliases for its allocator-backed temporary vectors;
- `AvbdRigidWorldContactBuildScratch` can be constructed from a snapshot
  contact allocator, converting that allocator to its row-counter and row-order
  vector allocators;
- the no-scratch `detail::assignAvbdRigidWorldContactRows(snapshot)` fallback
  now builds scratch from `snapshot.contacts.get_allocator()`;
- the no-scratch `buildAvbdRigidWorldContactSnapshotInto(...)`,
  `appendAvbdRigidWorldPointJoints(...)`, and
  `appendAvbdRigidWorldDistanceSprings(...)` overloads now borrow the snapshot
  contact allocator for their internal build scratch;
- `AvbdRigidBlock.RigidWorldContactRowsFallbackUsesSnapshotAllocator` verifies
  the short-lived row-assignment fallback scratch routes allocations through
  the snapshot allocator.

This still does not claim that return-by-value AVBD rigid-world snapshot
helpers are allocation-free, that fallback scratch is retained across calls, or
that unrelated AVBD row builders have been converted. The closed gap is
allocator provenance for the no-scratch rigid-world fallback helpers when a
caller already provided an allocator-backed snapshot.

Validation for this slice:

```bash
pixi run cmake --build build/default/cpp/Release --target test_avbd_rigid_block
pixi run build/default/cpp/Release/bin/test_avbd_rigid_block \
  --gtest_filter=AvbdRigidBlock.RigidWorldContactRowsFallbackUsesSnapshotAllocator
pixi run build/default/cpp/Release/bin/test_avbd_rigid_block
pixi run lint
git diff --check
pixi run build
pixi run test-unit
pixi run -e cuda test-all
```

The CUDA `test-all` run passed with the existing no-GUI
`dartpy._world_render_bridge` autodoc warnings during documentation generation
and CPU-scaling benchmark warnings during CUDA benchmark smoke.

Before publishing or opening a PR from this branch, get explicit maintainer
approval before pushing.

## Historical Slices Below

The sections below are retained as chronological evidence for previous HMM
slices. They are not current instructions. A fresh agent should use the top
hard-stop section as the authoritative handoff surface.

## Hard Stop Handoff (2026-06-13, Boxed-LCP Contact Snapshot Payload Scratch)

Resume from exactly one branch:
`pr/hmm-phase45-replay-snapshot-allocators`, tracking
`origin/pr/hmm-phase45-replay-snapshot-allocators`. This remains the single
HMM handoff entry point unless a maintainer explicitly redirects the work.
The branch currently has no open PR.

Latest local slice: boxed-LCP contact assembly now keeps the LCP snapshot
payload used by the world-step solve in allocator-backed
`BoxedLcpContactScratch` vectors. The reusable scratch owns `systemA`,
`systemB`, `systemLo`, `systemHi`, `systemFindex`, `systemF`, and `systemJ`
alongside the prior dense temporary buffers. The new
`applyBoxedLcpContacts(...)` world-step entry point maps those buffers as Eigen
views, solves into `systemF`, applies impulses, and intentionally leaves the
diagnostic `BoxedLcpContactSnapshot` empty. The existing
`solveBoxedLcpContacts(...)` helpers still populate the Eigen snapshot for
diagnostics, CUDA fixtures, and tests by copying from scratch after the solve.

The fix has these parts:

- `BoxedLcpContactScratch` uses local `DoubleVector` and `IntVector` aliases for
  allocator-backed boxed-LCP system payload buffers;
- `BoxedLcpContactScratch::reserve(...)` sizes the scratch-owned LCP payload and
  no longer preallocates Eigen-owned diagnostic snapshot members;
- `DantzigSolver` exposes a non-virtual `Eigen::Ref` solve overload so callers
  can solve directly into mapped scratch storage while preserving the existing
  owning-`Eigen::VectorXd` API;
- `RigidBodyContactStage` calls `applyBoxedLcpContacts(...)` for the boxed-LCP
  world-step path, avoiding diagnostic snapshot payload writes during baked
  steps;
- `World.BoxedLcpContactScratchUsesProvidedAllocator` verifies allocator-backed
  system payload storage and an empty diagnostic snapshot after reserve, while
  `World.BoxedLcpContactApplySkipsDiagnosticSnapshotPayload` verifies the
  no-snapshot solve still applies a contact impulse.

This still does not claim that diagnostic `BoxedLcpContactSnapshot` Eigen
payloads borrow a DART allocator, that the one-shot return-by-value boxed-LCP
helper is allocation-free, or that arbitrary LCP solver APIs accept custom
allocators. The closed gap is the boxed-LCP world-step assembly/solve payload
owned by reusable DART scratch.

Validation for this slice:

```bash
pixi run cmake --build build/default/cpp/Release --target test_world -j 8
build/default/cpp/Release/bin/test_world \
  --gtest_filter='World.BoxedLcpContactScratchUsesProvidedAllocator:World.BoxedLcpContactApplySkipsDiagnosticSnapshotPayload:World.BakedBoxedLcpFallbackContactsDoNotGrowWorldBaseAllocator:World.BakedBoxedLcpFallbackContactStepsDoNotAllocateGlobalHeap'
build/default/cpp/Release/bin/test_boxed_lcp_contact \
  --gtest_filter='BoxedLcpContact.WorldContactSnapshotSatisfiesLcpContract:BoxedLcpContact.TwoSphereWorldContactSnapshotSatisfiesLcpContract'
build/default/cpp/Release/bin/UNIT_math_lcp_math_lcp_dantzig_solver
pixi run lint
git diff --check
pixi run build
pixi run test-unit
pixi run -e cuda test-all
```

The CUDA `test-all` run passed with the existing no-GUI
`dartpy._world_render_bridge` autodoc warnings during documentation generation
and CPU-scaling benchmark warnings during CUDA benchmark smoke. Before
publishing or opening a PR from this branch, get explicit maintainer approval
before pushing.

## Historical Slices Below

The sections below are retained as chronological evidence for previous HMM
slices. They are not current instructions. A fresh agent should use the top
hard-stop section as the authoritative handoff surface.

## Hard Stop Handoff (2026-06-13, Boxed-LCP Contact Scratch Temporary Allocator)

Resume from exactly one branch:
`pr/hmm-phase45-replay-snapshot-allocators`, tracking
`origin/pr/hmm-phase45-replay-snapshot-allocators`. This remains the single
HMM handoff entry point unless a maintainer explicitly redirects the work.
The branch currently has no open PR.

Latest local slice: reusable boxed-LCP contact scratch now borrows the provided
DART allocator for its retained dense solve temporaries. `Minv`, `vFree`,
`JMinv`, `jtImpulse`, and `deltaV` are allocator-backed vectors, and the solve
path maps them as Eigen views while assembling the dense contact system and
applying solved impulses. The previous Dantzig validation, work-array,
pivot-state, and World boxed-LCP/unified-constraint nested scratch allocator
wiring remains intact.

The fix has these parts:

- `BoxedLcpContactScratch` defines allocator-backed double buffers for the
  retained dense contact solve temporaries;
- `reserveBoxedLcpContactScratch(...)` sizes those temporary buffers alongside
  the existing contact snapshot, body-column map, normal rows, and nested
  Dantzig scratch;
- `solveBoxedLcpContacts(..., BoxedLcpContactScratch&)` maps the retained
  buffers as Eigen matrices/vectors for dense inverse-mass, free-velocity,
  `J M^-1`, `J^T f`, and `deltaV` operations without owning Eigen temporaries;
- the prior `DantzigSolver::Scratch` allocator wiring for validation vectors,
  Dantzig work arrays, and pivot state remains in place for the nested solve;
- `World.BoxedLcpContactScratchUsesProvidedAllocator` verifies allocator-backed
  contact temporary storage, expected reserve sizes, nested Dantzig storage, and
  release back to the provided free allocator.

This still does not claim that Eigen dynamic payloads inside boxed-LCP
snapshots, arbitrary one-shot LCP solve APIs, or return-by-value contact
snapshot helpers are allocation-free or allocator-backed. The closed gap is the
retained boxed-LCP contact solve temporary storage owned by reusable
`BoxedLcpContactScratch` when a caller supplies DART scratch storage.

Validation for this slice:

```bash
pixi run cmake --build build/default/cpp/Release --target test_world -j 8
build/default/cpp/Release/bin/test_world \
  --gtest_filter='World.BoxedLcpContactScratchUsesProvidedAllocator:World.BakedBoxedLcpFallbackContactStepsDoNotAllocateGlobalHeap'
pixi run lint
pixi run build
pixi run test-unit
pixi run -e cuda test-all
```

The CUDA `test-all` run passed with the existing no-GUI
`dartpy._world_render_bridge` autodoc warnings during documentation generation.
Before publishing or opening a PR from this branch, get explicit maintainer
approval before pushing.

## Historical Slices Below

The sections below are retained as chronological evidence for previous HMM
slices. They are not current instructions. A fresh agent should use the top
hard-stop section as the authoritative handoff surface.

## Hard Stop Handoff (2026-06-13, Contact-Free Derivative Dynamics-Terms Scratch)

Resume from exactly one branch:
`pr/hmm-phase45-replay-snapshot-allocators`, tracking
`origin/pr/hmm-phase45-replay-snapshot-allocators`. This remains the single
HMM handoff entry point unless a maintainer explicitly redirects the work.
The branch currently has no open PR.

Latest local slice: the differentiable contact-free multibody Jacobian path now
borrows reusable dynamics-terms scratch from `WorldStorage` in addition to its
existing coordinate and inverse-dynamics scratch. The internal
`ContactFreeStepDynamicsTermsScratch` bundle retains
`MultibodyDynamicsTermsScratch` plus the base/perturbed
`MultibodyDynamicsTerms` result payloads used by
`detail::contactFreeStepDerivatives(...)`.

The fix has these parts:

- `ContactFreeStepDynamicsTermsScratch` exposes allocator-backed reuse for the
  dynamics-term evaluations inside the contact-free differentiable Jacobian
  helper;
- `WorldStorage` constructs that scratch from the World free allocator and
  passes it through `World::captureStepDerivatives()`;
- `World.DifferentiableContactFreeStepScratchUsesProvidedAllocator` verifies
  first-use allocation through the provided allocator and no additional
  configured-allocator growth on a same-shape follow-up derivative call.

This still does not claim that arbitrary user-provided
`VariationalContactHook` callbacks avoid return-by-value forces, that every
Eigen dynamic result payload borrows a DART allocator, or that the
return-by-value `Multibody::getMassMatrix()` / `getCoriolisForces()` /
`getGravityForces()` convenience accessors are allocation-free. The closed gap
is the DART-owned dynamics-tree/RNEA/mass-bias scratch and retained
dynamics-term result capacity used inside the differentiable contact-free
multibody Jacobian path.

Validation for this slice:

```bash
DART_BUILD_DIFF_OVERRIDE=ON pixi run config
pixi run cmake --build build/default/cpp/Release --target test_world test_diff_smooth_jacobian -j 8
build/default/cpp/Release/bin/test_world \
  --gtest_filter='World.DifferentiableContactFreeStepScratchUsesProvidedAllocator:World.MultibodyDynamicsTermsScratchUsesProvidedAllocator:World.MultibodyEquationOfMotionConsistency'
build/default/cpp/Release/bin/test_diff_smooth_jacobian
pixi run lint
pixi run build
pixi run test-unit
pixi run -e cuda test-all
```

Before publishing or opening a PR from this branch, rerun the relevant
lint/build/test gates from a clean source state and get explicit maintainer
approval before pushing.

## Historical Slices Below

The sections below are retained as chronological evidence for previous HMM
slices. They are not current instructions. A fresh agent should use the top
hard-stop section as the authoritative handoff surface.

## Hard Stop Handoff (2026-06-13, Variational Contact Point Force Output)

Resume from exactly one branch:
`pr/hmm-phase45-replay-snapshot-allocators`, tracking
`origin/pr/hmm-phase45-replay-snapshot-allocators`. This remains the single
HMM handoff entry point unless a maintainer explicitly redirects the work.
The branch currently has no open PR.

Latest local slice: direct variational contact point-force callers can now
reuse caller-owned generalized-force output storage. The public
`variationalContactPointForceInto(...)` helper overwrites a retained
`Eigen::VectorXd` result, while the old return-by-value
`variationalContactPointForce(...)` helper remains source-compatible and
delegates through the same implementation. Internal built-in ground-contact,
link-sphere, and compliant-loop force paths continue to accumulate into their
existing reusable force scratch.

The fix has these parts:

- `variationalContactPointForceInto(...)` exposes the existing no-temporary
  point-force math as a public overwrite-style helper;
- `variationalContactPointForce(...)` delegates through the new helper instead
  of constructing temporary dynamic Jacobian matrices;
- internal multi-contact force assembly uses a private accumulator so built-in
  World-stage paths preserve their existing reusable scratch behavior;
- `World.VariationalContactPointForceIntoReusesOutputStorage` verifies the
  helper matches the direct Jacobian formula and performs zero global heap
  allocations on a warmed same-shape call with retained output storage.

This still does not claim that arbitrary user-provided
`VariationalContactHook` callbacks avoid return-by-value forces, that every
Eigen dynamic result payload borrows a DART allocator, or that public
return-by-value convenience helpers are allocation-free. The closed gap is the
direct point-force helper's caller-retained output payload path and removal of
its avoidable temporary dynamic Jacobian matrices.

Validation for this slice:

```bash
pixi run cmake --build build/default/cpp/Release --target test_world -j 8
build/default/cpp/Release/bin/test_world \
  --gtest_filter='World.VariationalContactPointForceIntoReusesOutputStorage:World.VariationalConstraintLinearizationScratchUsesProvidedAllocator:World.BakedMultibodyAndDeformableStepsDoNotAllocateGlobalHeap'
```

Before publishing or opening a PR from this branch, rerun the relevant
lint/build/test gates from a clean source state and get explicit maintainer
approval before pushing.

## Historical Slices Below

The sections below are retained as chronological evidence for previous HMM
slices. They are not current instructions. A fresh agent should use the top
hard-stop section as the authoritative handoff surface.

## Hard Stop Handoff (2026-06-13, Variational Constraint Linearization Scratch Helper)

Resume from exactly one branch:
`pr/hmm-phase45-replay-snapshot-allocators`, tracking
`origin/pr/hmm-phase45-replay-snapshot-allocators`. This remains the single
HMM handoff entry point unless a maintainer explicitly redirects the work.
The branch currently has no open PR.

Latest local slice: direct variational loop-constraint linearization callers can
now reuse caller-owned tree and projection scratch plus caller-retained
residual/Jacobian output capacity. `computeVariationalConstraintLinearizationInto(...)`
builds the variational tree into `MultibodyVariationalTreeScratch`, evaluates
the loop-closure residual/Jacobian through
`VariationalConstraintProjectionScratch`, and writes into a retained
`VariationalConstraintLinearization` result.

The fix has these parts:

- `computeVariationalConstraintLinearizationInto(...)` exposes the existing
  scratch-backed loop-constraint residual/Jacobian path without changing the old
  return-by-value helper;
- repeated same-shape direct-helper calls can retain DART-owned tree/projection
  storage through a provided allocator and retain Eigen output capacity in the
  caller;
- `World.VariationalConstraintLinearizationScratchUsesProvidedAllocator`
  verifies first-use scratch allocation through the provided allocator and zero
  global heap allocation on the warmed same-shape second call.

This still does not claim that every public variational helper avoids
return-by-value payloads, that generic `VariationalContactHook` callbacks avoid
return-by-value forces, or that Eigen dynamic result storage itself borrows the
DART allocator. The closed gap is the direct loop-constraint linearization
helper's reusable DART-owned tree/projection scratch path and caller-retained
result capacity.

Validation for this slice:

```bash
pixi run cmake --build build/default/cpp/Release \
  --target test_world test_variational_integration -j 8
build/default/cpp/Release/bin/test_world \
  --gtest_filter='World.VariationalConstraintLinearizationScratchUsesProvidedAllocator:World.VariationalInverseMassProductScratchUsesProvidedAllocator:World.BakedMultibodyAndDeformableStepsDoNotAllocateGlobalHeap'
build/default/cpp/Release/bin/test_variational_integration \
  --gtest_filter='VariationalIntegration.ConstraintJacobianMatchesFiniteDifference'
```

Before publishing or opening a PR from this branch, rerun the relevant
lint/build/test gates from a clean source state and get explicit maintainer
approval before pushing.

## Historical Slices Below (Older)

The sections below are retained as chronological evidence for previous HMM
slices. They are not current instructions. A fresh agent should use the top
hard-stop section as the authoritative handoff surface.

## Hard Stop Handoff (2026-06-13, Variational Inverse-Mass Scratch Helper)

Resume from exactly one branch:
`pr/hmm-phase45-replay-snapshot-allocators`, tracking
`origin/pr/hmm-phase45-replay-snapshot-allocators`. This remains the single
HMM handoff entry point unless a maintainer explicitly redirects the work.
The branch currently has no open PR.

Latest local slice: direct variational inverse-mass product callers can now
reuse caller-owned tree and linear-solve scratch plus caller-owned output
storage. `computeMultibodyInverseMassProductInto(...)` builds the variational
tree into `MultibodyVariationalTreeScratch`, applies the articulated inverse
mass through `VariationalLinearSolveScratch`, and writes into a retained
`Eigen::VectorXd` result.

The fix has these parts:

- `computeMultibodyInverseMassProductInto(...)` exposes the existing internal
  scratch-backed inverse-mass kernel without changing the old return-by-value
  helper;
- repeated same-shape direct-helper calls can retain DART-owned tree/linear
  solve storage through a provided allocator and retain Eigen output capacity
  in the caller;
- `World.VariationalInverseMassProductScratchUsesProvidedAllocator` verifies
  first-use scratch allocation through the provided allocator and zero global
  heap allocation on the warmed same-shape second call.

This still does not claim that every public variational helper avoids
return-by-value payloads, that generic `VariationalContactHook` callbacks avoid
return-by-value forces, or that Eigen dynamic result storage itself borrows the
DART allocator. The closed gap is the direct inverse-mass helper's reusable
DART-owned tree/solve scratch path and caller-retained result capacity.

Validation for this slice:

```bash
pixi run cmake --build build/default/cpp/Release --target test_world -j 8
build/default/cpp/Release/bin/test_world \
  --gtest_filter='World.VariationalInverseMassProductScratchUsesProvidedAllocator:World.VariationalMechanicalEnergyScratchUsesProvidedAllocator:World.BakedMultibodyAndDeformableStepsDoNotAllocateGlobalHeap'
```

Before publishing or opening a PR from this branch, rerun the relevant
lint/build/test gates from a clean source state and get explicit maintainer
approval before pushing.

## Historical Slices Below (Older)

The sections below are retained as chronological evidence for previous HMM
slices. They are not current instructions. A fresh agent should use the top
hard-stop section as the authoritative handoff surface.

## Hard Stop Handoff (2026-06-13, Variational Writeback Scratch)

Resume from exactly one branch:
`pr/hmm-phase45-replay-snapshot-allocators`, tracking
`origin/pr/hmm-phase45-replay-snapshot-allocators`. This remains the single
HMM handoff entry point unless a maintainer explicitly redirects the work.
The branch currently has no open PR.

Latest local slice: accepted variational multibody steps no longer create
per-joint dynamic `Eigen::VectorXd` copies while writing the accepted position,
velocity, and acceleration back to registry joints. `World::step()` now reserves
`VariationalStepScratch::previousJointVelocity` once per same-shape bake and
uses segment views plus `jointLogDifferenceInto(...)` for in-place writeback.

The fix has these parts:

- `VariationalStepScratch` owns a reusable `previousJointVelocity` buffer;
- `reserveVariationalStepScratch(...)` sizes that buffer to the maximum joint
  DOF in the variational tree;
- `integrateMultibodyVariationalImpl(...)` writes accepted joint velocities and
  accelerations in place, avoiding the old `previousVelocity` and `newPosition`
  per-joint `VectorXd` temporaries;
- `World.EnterSimulationModeReservesCompliantVariationalContactScratch` now
  asserts that the baked variational step scratch includes the writeback
  velocity buffer.

This still does not claim that public generic `VariationalContactHook`
callbacks avoid return-by-value forces, that public no-scratch/direct
variational helpers use World-owned scratch, or that every Eigen dynamic matrix
or vector in the variational pipeline is allocator-backed. The closed gap is
the built-in `World::step()` variational accepted-state writeback path for
same-shape baked worlds.

Validation for this slice:

```bash
pixi run cmake --build build/default/cpp/Release --target test_world -j 8
build/default/cpp/Release/bin/test_world \
  --gtest_filter='World.EnterSimulationModeReservesCompliantVariationalContactScratch:World.ContactHeavyRegistryStorageRebuildsAfterClear:World.VariationalArticulatedPointJointLinkIndexScratchUsesWorldAllocator:World.BakedMultibodyAndDeformableStepsDoNotAllocateGlobalHeap'
```

Before publishing or opening a PR from this branch, rerun the relevant
lint/build/test gates from a clean source state and get explicit maintainer
approval before pushing.

## Historical Slices Below (Older)

The sections below are retained as chronological evidence for previous HMM
slices. They are not current instructions. A fresh agent should use the top
hard-stop section as the authoritative handoff surface.

## Hard Stop Handoff (2026-06-13, Variational Compliant-Loop Force Scratch)

Resume from exactly one branch:
`pr/hmm-phase45-replay-snapshot-allocators`, tracking
`origin/pr/hmm-phase45-replay-snapshot-allocators`. This remains the single
HMM handoff entry point unless a maintainer explicitly redirects the work.
The branch currently has no open PR.

Latest local slice: the built-in variational compliant articulated point-joint
path no longer routes DART-owned compliant loop rows through a generic
return-by-value contact hook. `World::step()` now evaluates the linear and
angular compliant-loop forces directly into
`VariationalContactEvaluationScratch::contactForce`, and the post-step row
refresh rebuilds the variational tree through
`MultibodyVariationalScratch::tree`.

The fix has these parts:

- `evaluateVariationalCompliantLoopForceInto(...)` accumulates compliant loop
  rows into caller-owned generalized-force storage;
- `variationalWorldTorqueInto(...)` removes the angular-row temporary dynamic
  world-angular-Jacobian/vector path;
- `integrateMultibodyVariationalImpl(...)` accepts an optional
  `VariationalCompliantLoopScratch*` for World-owned compliant rows while the
  public generic `VariationalContactHook` API stays unchanged;
- `updateVariationalCompliantLoopConstraintRows(...)` reuses the World-owned
  variational tree scratch for post-step row updates.

This still does not claim that public generic `VariationalContactHook`
callbacks avoid return-by-value forces, that public no-scratch/direct
variational helpers use World-owned scratch, or that every Eigen dynamic matrix
or vector in the variational pipeline is allocator-backed. The closed gap is the
built-in `World::step()` compliant articulated point-joint force/row-refresh
path.

Validation for this slice:

```bash
pixi run cmake --build build/default/cpp/Release \
  --target test_world test_variational_integration -j 8
pixi run ./build/default/cpp/Release/bin/test_world \
  --gtest_filter='World.VariationalArticulatedPointJointLinkIndexScratchUsesWorldAllocator:World.BakedMultibodyAndDeformableStepsDoNotAllocateGlobalHeap'
```

Before publishing or opening a PR from this branch, rerun the relevant
lint/build/test gates from a clean source state and get explicit maintainer
approval before pushing.

## Hard Stop Handoff (2026-06-13, Rigid AVBD Distance-Spring Gates)

Resume from exactly one branch:
`pr/hmm-phase45-replay-snapshot-allocators`, tracking
`origin/pr/hmm-phase45-replay-snapshot-allocators`. This remains the single
HMM handoff entry point unless a maintainer explicitly redirects the work.
The branch currently has no open PR.

Latest local slice: the baked rigid AVBD allocator regressions now cover the
distance-spring row path as a peer of contact rows and fixed-joint rows. The
production rigid AVBD stage already owns allocator-backed `distanceSprings`,
distance-spring inventory, build scratch, and solve scratch; this slice closes
the missing evidence by driving that path through active behavior, registry
storage rebuild, World-base no-growth, and global-heap no-allocation gates.

The fix has these parts:

- `configureRigidAvbdDistanceSpringRowsScene(...)` creates a contact-free rigid
  distance-spring scene that exercises the built-in rigid AVBD projection path;
- `World.RigidAvbdDistanceSpringRowsAreActiveWithoutContacts` verifies the
  distance-spring rows move the stretched body without contacts;
- `World.RigidAvbdRegistryStorageRebuildsAfterClear` now checks
  `AvbdRigidWorldDistanceSpringConfig` storage capacity across clear/rebuild
  cycles;
- the baked World-base no-growth and global-heap no-allocation suites now
  include `"rigid AVBD distance-spring rows"`.

This still does not claim that the public owning AVBD helper overloads avoid
default-allocator vectors, that every AVBD direct helper call is
allocator-backed, or that unrelated public return-by-value APIs are allocation
free. The closed gap is the built-in `World::step()` rigid AVBD distance-spring
row path and its registry storage coverage.

Validation for this slice:

```bash
pixi run cmake --build build/default/cpp/Release --target test_world -j 8
./build/default/cpp/Release/bin/test_world \
  --gtest_filter='World.RigidAvbdDistanceSpringRowsAreActiveWithoutContacts:World.RigidAvbdRegistryStorageRebuildsAfterClear:World.BakedStepsDoNotGrowWorldBaseAllocatorForReservedEcsPaths:World.BakedRigidBodyContactStepsDoNotAllocateGlobalHeap' \
  --gtest_color=no
```

Before publishing or opening a PR from this branch, rerun the relevant
lint/build/test gates from a clean source state and get explicit maintainer
approval before pushing.

## Hard Stop Handoff (2026-06-13, Mechanical-Energy Velocity Scratch)

Resume from exactly one branch:
`pr/hmm-phase45-replay-snapshot-allocators`, tracking
`origin/pr/hmm-phase45-replay-snapshot-allocators`. This remains the single
HMM handoff entry point unless a maintainer explicitly redirects the work.
The branch currently has no open PR.

Latest local slice: the variational multibody mechanical-energy diagnostic now
has a scratch-aware overload that reuses caller-owned
`compute::MultibodyVariationalTreeScratch` and
`compute::VariationalStepScratch`. Repeated same-shape energy samples can keep
the DART-owned variational tree storage and spatial-velocity vector capacity
instead of rebuilding those default-allocator diagnostic temporaries on every
call. The existing no-scratch `computeMultibodyMechanicalEnergy(...)` overload
remains source-compatible and delegates through local scratch objects.

The fix has these parts:

- `computeMultibodyMechanicalEnergy(...)` now has an overload that accepts
  caller-owned variational tree and step scratch;
- the energy path gathers only generalized velocity and fills
  `VariationalStepScratch::currentSpatialVelocities`, avoiding the broader
  integration-only position/force/residual scratch bundle;
- the old no-scratch overload delegates to the scratch-aware implementation;
- `World.VariationalMechanicalEnergyScratchUsesProvidedAllocator` verifies the
  first diagnostic call grows provided allocator-backed scratch and a
  same-shape second call reuses capacity without global-heap allocation.

This still does not claim that every Eigen dynamic buffer in variational
diagnostics is backed by the DART allocator, that public no-scratch diagnostic
calls reuse World-owned storage, or that all simulation-loop energy/profiling
payloads are allocation-free. The closed gap is the DART-owned variational tree
storage and spatial-velocity vector used by repeated mechanical-energy
diagnostics.

Validation for this slice:

```bash
pixi run cmake --build build/default/cpp/Release \
  --target test_world test_variational_integration -j 8
./build/default/cpp/Release/bin/test_world \
  --gtest_filter='World.VariationalMechanicalEnergyScratchUsesProvidedAllocator' \
  --gtest_color=no
./build/default/cpp/Release/bin/test_variational_integration \
  --gtest_filter='VariationalIntegration.PendulumConservesEnergyOverLongHorizon:VariationalIntegration.PassiveChainEnergyCoverageSmoke' \
  --gtest_color=no
```

Before publishing or opening a PR from this branch, rerun the relevant
lint/build/test gates from a clean source state and get explicit maintainer
approval before pushing.

## Hard Stop Handoff (2026-06-13, Contact-Free Coordinate Scratch)

Resume from exactly one branch:
`pr/hmm-phase45-replay-snapshot-allocators`, tracking
`origin/pr/hmm-phase45-replay-snapshot-allocators`. This remains the single
HMM handoff entry point unless a maintainer explicitly redirects the work.
The branch currently has no open PR.

Latest local slice: the contact-free differentiable multibody Jacobian path now
collects generalized-coordinate references into reusable
`detail::ContactFreeStepCoordinateScratch` instead of rebuilding a
default-allocator `std::vector` on every `World::captureStepDerivatives()` WS1
call. `WorldStorage` retains that coordinate scratch with the World free
allocator and passes it into `detail::contactFreeStepDerivatives()`.

The fix has these parts:

- `detail::ContactFreeStepCoordinate` and
  `detail::ContactFreeStepCoordinateScratch` name the internal coordinate list
  used by the contact-free smooth Jacobian helper;
- `collectCoordinatesInto()` clears and refills caller-owned scratch, preserving
  same-shape capacity across differentiable steps;
- `WorldStorage` owns `differentiableCoordinateScratch` with the World free
  allocator and passes it alongside the existing inverse-dynamics derivative
  scratch;
- `World.DifferentiableContactFreeCoordinateScratchUsesProvidedAllocator`
  verifies the first direct smooth-Jacobian call grows caller-provided
  coordinate scratch through the selected allocator and a same-shape second call
  reuses that capacity.

This still does not claim that derivative Eigen result matrices,
`computeMultibodyDynamicsTerms()` fallback storage, finite-difference temporary
matrices, or public return-by-value payloads are under the World allocator. The
closed gap is only the DART-owned coordinate-reference list used before
assembling contact-free differentiable multibody derivatives.

Validation for this slice:

```bash
DART_BUILD_DIFF_OVERRIDE=ON pixi run config
pixi run cmake --build build/default/cpp/Release --target test_world test_diff_smooth_jacobian -j 8
./build/default/cpp/Release/bin/test_world \
  --gtest_filter='World.DifferentiableContactFreeCoordinateScratchUsesProvidedAllocator:World.DifferentiableMultibodyTorqueScratchUsesWorldAllocator' \
  --gtest_color=no
./build/default/cpp/Release/bin/test_diff_smooth_jacobian --gtest_color=no
```

Before publishing or opening a PR from this branch, rerun the relevant
lint/build/test gates from a clean source state and get explicit maintainer
approval before pushing.

## Hard Stop Handoff (2026-06-13, Inverse-Dynamics Derivative Scratch)

Resume from exactly one branch:
`pr/hmm-phase45-replay-snapshot-allocators`, tracking
`origin/pr/hmm-phase45-replay-snapshot-allocators`. This remains the single
HMM handoff entry point unless a maintainer explicitly redirects the work.
The branch currently has no open PR.

Latest local slice: the analytic inverse-dynamics derivative path now reuses
allocator-backed spatial-vector scratch through
`compute::MultibodyInverseDynamicsScratch` instead of rebuilding
default-allocator `std::vector<Vector6>` storage for every derivative column.
`WorldStorage` retains the same scratch with the World free allocator and passes
it into the contact-free differentiable multibody Jacobian path.

The fix has these parts:

- `MultibodyDynamicsScratch` owns reusable RNEA derivative buffers for the base
  velocity/acceleration/force sweep, accumulated force, and per-column delta
  sweeps;
- `computeMultibodyInverseDynamicsDerivativesInto()` lets stage-loop callers
  reuse those buffers while the return-by-value wrapper stays source-compatible;
- `detail::contactFreeStepDerivatives()` accepts optional inverse-dynamics
  scratch, and `World::captureStepDerivatives()` passes the World-owned scratch
  for differentiable WS1 steps;
- `World.MultibodyInverseDynamicsDerivativeScratchUsesProvidedAllocator`
  verifies the first analytic derivative call grows caller-provided scratch
  through the selected allocator and a same-shape second call reuses that
  capacity.

This still does not claim that derivative Eigen result matrices, dynamics-term
finite-difference fallback storage, or public return-by-value payloads are under
the World allocator. The closed gap is only the DART-owned spatial-vector scratch
used by the analytic RNEA derivative recursion and its World differentiable
call site.

Validation for this slice:

```bash
pixi run cmake --build build/default/cpp/Release --target test_world -j 8
./build/default/cpp/Release/bin/test_world \
  --gtest_filter='World.MultibodyInverseDynamicsDerivativeScratchUsesProvidedAllocator' \
  --gtest_color=no
DART_BUILD_DIFF_OVERRIDE=ON pixi run config
pixi run cmake --build build/default/cpp/Release --target test_world test_diff_smooth_jacobian -j 8
./build/default/cpp/Release/bin/test_world \
  --gtest_filter='World.DifferentiableMultibodyTorqueScratchUsesWorldAllocator' \
  --gtest_color=no
./build/default/cpp/Release/bin/test_diff_smooth_jacobian --gtest_color=no
```

Before publishing or opening a PR from this branch, rerun the relevant
lint/build/test gates from a clean source state and get explicit maintainer
approval before pushing.

## Historical Slices Below

The sections below are retained as chronological evidence for previous HMM
slices. They are not current instructions. A fresh agent should use the top
hard-stop section as the authoritative handoff surface.

## Hard Stop Handoff (2026-06-13, Articulated Point-Joint Link-Index Scratch)

Resume from exactly one branch:
`pr/hmm-phase45-replay-snapshot-allocators`, tracking
`origin/pr/hmm-phase45-replay-snapshot-allocators`. This remains the single
HMM handoff entry point unless a maintainer explicitly redirects the work.
The branch currently has no open PR.

Latest local slice: the variational articulated point-joint constraint-gather
path now reuses a World-allocator-backed large-structure link-index map from
`VariationalCompliantLoopScratch` instead of rebuilding a default-allocator
`std::unordered_map` whenever a point-joint structure has more than 16 links.
The map is populated during bake/prepare, retained across same-shape steps, and
revalidated before reuse so topology changes still rebuild it correctly.

The fix has these parts:

- `VariationalCompliantLoopScratch` owns a `LinkIndexMap` constructed with the
  configured memory allocator;
- `appendAvbdRigidWorldArticulatedPointJointConstraints()` reuses the scratch
  map when provided, falls back to a local default-allocator map only for
  scratch-less callers, and skips rebuilding when the current structure-link
  order already matches the cached map;
- `World.VariationalArticulatedPointJointLinkIndexScratchUsesWorldAllocator`
  builds a long variational multibody with a world-fixed articulated point
  joint, enters simulation mode before measurement, and verifies same-shape
  steps do not grow the base allocator or registry storage;
- `World.BakedMultibodyAndDeformableStepsDoNotAllocateGlobalHeap` now includes
  the same long articulated point-joint scene so the 16-link map threshold is
  covered by the monolithic no-global-heap baked-step gate.

This still does not claim that public variational diagnostic helpers such as
`computeMultibodyMechanicalEnergy()` or public return-by-value containers are
under the World allocator. The closed gap is only the DART-owned large-structure
link-index lookup used while gathering articulated point-joint constraints on
the variational `World::step` path.

Validation for this slice:

```bash
pixi run cmake --build build/default/cpp/Release --target test_world -j 8
./build/default/cpp/Release/bin/test_world \
  --gtest_filter='World.VariationalArticulatedPointJointLinkIndexScratchUsesWorldAllocator:World.BakedMultibodyAndDeformableStepsDoNotAllocateGlobalHeap' \
  --gtest_color=no
```

Before publishing or opening a PR from this branch, rerun the relevant
lint/build/test gates from a clean source state and get explicit maintainer
approval before pushing.

## Historical Slices Below

The sections below are retained as chronological evidence for previous HMM
slices. They are not current instructions. A fresh agent should use the top
hard-stop section as the authoritative handoff surface.

## Hard Stop Handoff (2026-06-13, Differentiable Torque Scratch Allocator)

Resume from exactly one branch:
`pr/hmm-phase45-replay-snapshot-allocators`, tracking
`origin/pr/hmm-phase45-replay-snapshot-allocators`. This remains the single
HMM handoff entry point unless a maintainer explicitly redirects the work.
The branch currently has no open PR.

Latest local slice: the contact-free differentiable multibody derivative path
now reuses `WorldStorage::differentiableTorqueScratch`, an allocator-backed
`std::vector<double>` owned by the World free allocator, instead of rebuilding
default-allocator torque collection storage on every differentiable step. The
smooth Jacobian helper now accepts the applied torque vector through
`Eigen::Ref`, so `World::captureStepDerivatives()` maps the reusable scratch
directly instead of copying it into an owning `Eigen::VectorXd`.

The fix has these parts:

- `WorldStorage` owns reusable differentiable torque scratch constructed with
  `common::StlAllocator<double>` from the World memory allocator;
- `World::captureStepDerivatives()` clears/reserves/fills that scratch for the
  construction-order joint efforts, maps it into Eigen, and leaves the capacity
  available for same-shape follow-up differentiable steps;
- `detail::contactFreeStepDerivatives()` accepts
  `Eigen::Ref<const Eigen::VectorXd>` so the mapped scratch can flow through the
  internal smooth Jacobian path without an extra owning torque copy;
- `detail::contactStepDerivativesWithParameters()` accepts
  `std::span<const ParameterRegistration>` so the diff-only contact-parameter
  path consumes allocator-backed `WorldStorage::differentiableParameters`
  without requiring a default-allocator `std::vector`;
- `World.DifferentiableMultibodyTorqueScratchUsesWorldAllocator` builds a large
  differentiable revolute chain, enters simulation mode before measurement,
  verifies the first differentiable step grows reusable World-owned scratch,
  verifies the control Jacobian column count, and verifies a same-size second
  step reuses that free-list capacity without growing live bytes or peak bytes.

This still does not claim that derivative Eigen result storage is under the
World allocator. `StepDerivatives` remains an Eigen value payload and the
Jacobian assembly still allocates Eigen matrices outside this slice. The closed
gap is the DART-owned torque collection scratch and the avoidable owning torque
copy in the internal contact-free differentiable multibody path.

Validation for this slice:

```bash
DART_BUILD_DIFF_OVERRIDE=ON pixi run config
pixi run cmake --build build/default/cpp/Release --target test_world test_diff_smooth_jacobian -j 8
./build/default/cpp/Release/bin/test_world \
  --gtest_filter='World.DifferentiableMultibodyTorqueScratchUsesWorldAllocator:World.NumDofsDynamicBodyCollectionUsesWorldAllocator:World.SaveBinaryIgnoredCollisionPairFilterUsesWorldAllocator:World.WorldPersistentStorageUsesWorldFreeAllocator' \
  --gtest_color=no
./build/default/cpp/Release/bin/test_diff_smooth_jacobian --gtest_color=no
```

Before publishing or opening a PR from this branch, rerun the relevant
lint/build/test gates from a clean source state and get explicit maintainer
approval before pushing.

## Historical Slices Below

The sections below are retained as chronological evidence for previous HMM
slices. They are not current instructions. A fresh agent should use the top
hard-stop section as the authoritative handoff surface.

## Hard Stop Handoff (2026-06-13, Dynamic-Body Query Allocator)

Resume from exactly one branch:
`pr/hmm-phase45-replay-snapshot-allocators`, tracking
`origin/pr/hmm-phase45-replay-snapshot-allocators`. This remains the single
HMM handoff entry point unless a maintainer explicitly redirects the work.
The branch currently has no open PR.

Latest local slice: experimental `World` state/control coordinate helpers now
collect dynamic rigid-body entities in allocator-backed scratch owned by the
World free allocator. The pre-fix gap was that `collectDynamicRigidBodies()`
returned a default-allocator `std::vector<entt::entity>` used by
`getNumDofs()`, `getStateVector()`, `setStateVector()`, `getControlVector()`,
and `setControlVector()`. Large dynamic-rigid-body worlds could therefore
grow global heap storage while only building DART-owned internal entity lists
for state/control API traversal.

The fix has these parts:

- `WorldStorage` retains an internal reference to the World free allocator so
  const query helpers can construct allocator-backed scratch without changing
  the public `World` API;
- `collectDynamicRigidBodies()` returns a
  `std::vector<entt::entity, common::StlAllocator<entt::entity>>` and all
  state/control vector helpers pass the `WorldStorage` allocator through that
  path;
- `World.NumDofsDynamicBodyCollectionUsesWorldAllocator` builds 1,024 dynamic
  bodies plus static bodies, verifies `getNumDofs()` ignores static bodies,
  verifies no global heap allocation occurs during dynamic-body collection,
  verifies the World free-list peak grows for the temporary vector, and
  verifies live bytes return to the pre-query value.

This still does not claim that public `Eigen::VectorXd` return payloads,
caller-provided Eigen vectors, EnTT registry internals, native collision
internals, or public return-by-value `std::vector` APIs are under the World
allocator. The closed gap is only the DART-owned dynamic-body entity list used
internally by experimental `World` state/control coordinate helpers.

Validation for this slice:

```bash
pixi run cmake --build build/default/cpp/Release --target test_world -j 8
./build/default/cpp/Release/bin/test_world \
  --gtest_filter='World.NumDofsDynamicBodyCollectionUsesWorldAllocator:World.SaveBinaryIgnoredCollisionPairFilterUsesWorldAllocator' \
  --gtest_color=no
```

Before publishing or opening a PR from this branch, rerun the relevant
lint/build/test gates from a clean source state and get explicit maintainer
approval before pushing.

## Hard Stop Handoff (2026-06-13, Ignored-Pair Save Allocator)

Resume from exactly one branch:
`pr/hmm-phase45-replay-snapshot-allocators`, tracking
`origin/pr/hmm-phase45-replay-snapshot-allocators`. This remains the single
HMM handoff entry point unless a maintainer explicitly redirects the work.
The branch currently has no open PR.

Latest local slice: experimental `World::saveBinary()` now builds the
temporary ignored collision-pair filter vector with the same World allocator
already owned by `WorldStorage::ignoredCollisionPairs`. The pre-fix gap was
that persistent ignored-pair storage used the World free allocator, but binary
save filtered those pairs through a default-allocator
`std::vector<CollisionPairKey>` before writing entity-map-remapped endpoints.
Large worlds with many persisted ignored pairs could therefore grow default
heap storage during save even though the source pair set was already under the
World memory hierarchy.

The fix has these parts:

- `World::saveBinary()` constructs `savedIgnoredPairs` with
  `m_storage->ignoredCollisionPairs.get_allocator()`, preserving logical const
  behavior while routing the temporary filter storage through the World-owned
  allocator;
- `World.SaveBinaryIgnoredCollisionPairFilterUsesWorldAllocator` creates more
  than 2,000 persisted ignored pairs, saves the world, verifies the World
  free-list peak grows during the temporary filter allocation, verifies live
  bytes return to the pre-save value, and round-trips the ignored-pair count
  through `loadBinary()`;
- the previous contact-query and Rigid IPC allocator slice remains intact below
  as historical context for this branch.

This still does not claim that serialization's public stream buffers,
`io::EntityMap`, native collision internals, or public return-by-value
`std::vector` APIs are under the World allocator. The closed gap is only the
DART-owned ignored-pair filter vector inside `World::saveBinary()`.

Validation for this slice:

```bash
pixi run cmake --build build/default/cpp/Release --target test_world -j 8
./build/default/cpp/Release/bin/test_world \
  --gtest_filter=World.SaveBinaryIgnoredCollisionPairFilterUsesWorldAllocator
```

Before publishing or opening a PR from this branch, rerun the relevant
lint/build/test gates from a clean source state and get explicit maintainer
approval before pushing.

## Historical Slices Below

The sections below are retained as chronological evidence for previous HMM
slices. They are not current instructions. A fresh agent should use the top
hard-stop section as the authoritative handoff surface.

## Hard Stop Handoff (2026-06-12, Contact Query and Rigid IPC Allocators)

Resume from exactly one branch:
`pr/hmm-phase45-replay-snapshot-allocators`, tracking
`origin/pr/hmm-phase45-replay-snapshot-allocators`. This remains the single
HMM handoff entry point unless a maintainer explicitly redirects the work.
The branch currently has no open PR.

Latest local slice: DART-owned cached collision-query contact results now use
the World free allocator, and the validation fallout closed two adjacent Rigid
IPC allocator/policy gaps. The previous cache slice routed shape specs, object
keys, object-id lookup, native object entries, and live rigid-body joint-pair
filter scratch through World memory, but `CollisionQueryCache::contacts` still
used default STL storage. Built-in contact stages and differentiable boxed-LCP
capture consume that cached contact list directly, so first contact-result
population could still grow default-allocator storage after the shape/object
cache was warm. Full `test_world` validation also exposed that Rigid IPC
projected-Newton step deltas still owned default-allocator Eigen storage during
stage prepare, and that the contact-free diagonal fast path ignored the
documented zero-iteration non-converged policy.

The fix has these parts:

- `CollisionQueryCache::contacts` is an allocator-backed vector constructed
  from the World's free allocator;
- private `World::queryContacts()` returns `std::span<const Contact>` so
  built-in stages can read the allocator-backed cache without exposing its
  concrete container type;
- multibody, unified-constraint, boxed-LCP, rigid AVBD contact, and
  differentiable contact-gradient helpers consume contact spans instead of
  requiring a default-allocator `std::vector<Contact>`;
- public `World::collide()` keeps the existing return-by-value
  `std::vector<Contact>` convenience by copying from the cached span;
- `RigidIpcProjectedNewtonStep` now stores its delta in a
  `common::StlAllocator<double>`-backed buffer and exposes Eigen maps at solver
  call sites, so Rigid IPC prepare/solve scratch can reserve from the provided
  free allocator;
- the Rigid IPC contact-free diagonal shortcut now requires a positive
  iteration budget, preserving the zero-budget rejected/non-converged stage
  policy used by regression tests;
- `FreeListAllocator` pre-reserves a small amount of internal block metadata so
  allocator-backed stage prepare does not grow the allocator's bookkeeping
  vector through global `operator new` during no-heap checks.

This still does not claim that native collision internals or the public
`std::vector<Contact>` return object are under the World allocator; those
remain API/native-boundary work. The closed gaps are DART-owned cached contact
result storage used by built-in simulation paths and the Rigid IPC
projected-Newton step delta storage used by stage scratch.

New regression coverage:

- `World.CollisionQueryCacheScratchUsesWorldAllocator` warms a same-shape
  non-contact query, moves one body into contact, checks that first
  contact-result population grows the World free allocator, and checks that a
  same-shape contact query reuses capacity.
- `World.RigidIpcContactStageScratchPayloadUsesProvidedAllocator` now covers
  allocator-backed projected-Newton step delta scratch and allocator metadata
  prewarming during mixed rigid/deformable IPC stage prepare.
- `World.RigidIpcContactStageAdvancesUnsupportedKinematicBody` and
  `World.RigidIpcContactStageSkipsUnconvergedSolveResult` cover the
  zero-iteration rejected-solve policy against the contact-free diagonal fast
  path.
- `RigidIpcBarrier.ProjectedNewtonSolveScratchUsesProvidedAllocator` now
  includes the allocator-backed step delta as part of detail solver scratch
  ownership.

Validation for this slice:

```bash
cmake --build build/default/cpp/Release \
  --target test_world test_rigid_ipc_barrier \
  --parallel ${JOBS:-8}
build/default/cpp/Release/bin/test_world \
  --gtest_filter='World.RigidIpcContactStageScratchPayloadUsesProvidedAllocator:World.RigidIpcContactStageAdvancesUnsupportedKinematicBody:World.RigidIpcContactStageSkipsUnconvergedSolveResult:World.CollisionQueryCacheScratchUsesWorldAllocator' \
  --gtest_color=no
build/default/cpp/Release/bin/test_rigid_ipc_barrier \
  --gtest_filter='RigidIpcBarrier.ProjectedNewtonStepSolvesBarrierSystem:RigidIpcBarrier.ProjectedNewtonStepHonorsLineSearchBound:RigidIpcBarrier.ProjectedNewtonStepBlocksUnsafeLineSearch:RigidIpcBarrier.ProjectedNewtonSolveScratchUsesProvidedAllocator' \
  --gtest_color=no
cmake --build build/cuda/cpp/Release \
  --target test_world test_rigid_ipc_barrier \
  --parallel ${JOBS:-8}
build/cuda/cpp/Release/bin/test_world \
  --gtest_filter='World.RigidIpcContactStageScratchPayloadUsesProvidedAllocator:World.RigidIpcContactStageAdvancesUnsupportedKinematicBody:World.RigidIpcContactStageSkipsUnconvergedSolveResult:World.CollisionQueryCacheScratchUsesWorldAllocator' \
  --gtest_color=no
build/cuda/cpp/Release/bin/test_rigid_ipc_barrier \
  --gtest_filter='RigidIpcBarrier.ProjectedNewtonStepSolvesBarrierSystem:RigidIpcBarrier.ProjectedNewtonStepHonorsLineSearchBound:RigidIpcBarrier.ProjectedNewtonStepBlocksUnsafeLineSearch:RigidIpcBarrier.ProjectedNewtonSolveScratchUsesProvidedAllocator' \
  --gtest_color=no
cmake --build build/default/cpp/Release \
  --target test_multibody_link_contact test_unified_constraint test_boxed_lcp_contact \
  --parallel ${JOBS:-8}
build/default/cpp/Release/bin/test_multibody_link_contact --gtest_color=no
build/default/cpp/Release/bin/test_unified_constraint --gtest_color=no
build/default/cpp/Release/bin/test_boxed_lcp_contact \
  --gtest_filter='BoxedLcpContact*' \
  --gtest_color=no
pixi run lint
pixi run build
pixi run test-unit
pixi run -e cuda test-all
```

Before publishing or opening a PR from this branch, rerun the relevant
lint/build/test gates from a clean source state and get explicit maintainer
approval before pushing.

## Hard Stop Handoff (2026-06-12, AVBD Rigid Writeback Dirty Stack)

Resume from exactly one branch:
`pr/hmm-phase45-replay-snapshot-allocators`, tracking
`origin/pr/hmm-phase45-replay-snapshot-allocators`. This remains the single
HMM handoff entry point unless a maintainer explicitly redirects the work.
The branch currently has no open PR.

Current local stop state:

- latest local commit: `01796e9dd1a`
  (`Reuse AVBD writeback dirty-stack scratch`);
- upstream tracking branch at this stop point:
  `origin/pr/hmm-phase45-replay-snapshot-allocators` was `6ab47a5fb63`
  (`Record HMM stop-state handoff`);
- local branch is ahead of origin by 1 commit and has no uncommitted changes at
  the start of this docs-only handoff edit.

Latest local slice: AVBD rigid-world contact transform writeback now reuses an
allocator-aware frame-dirty traversal stack owned by
`AvbdRigidWorldContactSnapshot`. The pre-fix gap was that
`applyAvbdRigidWorldContactSnapshot()` marked updated rigid-frame subtrees with
a local default-allocator `std::vector<entt::entity>`, so detail callers that
apply solved snapshots could grow default heap storage instead of reusing
snapshot scratch.

The fix has three parts:

- `AvbdRigidWorldContactSnapshot` owns `frameDirtyStack` with the same
  `common::StlAllocator<entt::entity>` as its entity lists;
- snapshot clearing/reservation clears and reserves that stack with body
  capacity, so same-shape writeback reuses capacity;
- `applyAvbdRigidWorldContactSnapshot()` uses the snapshot-owned stack for
  mutable snapshots and preserves a const overload that builds temporary stack
  storage from the snapshot allocator.

New regression coverage:

- `AvbdRigidBlock.RigidWorldContactApplyReusesSnapshotDirtyStackAllocator`
  checks that the first writeback allocation comes from the provided
  `MemoryManager` free allocator and that a same-shape second writeback does
  not grow that allocator again.

Validation for this slice:

```bash
cmake --build build/default/cpp/Release --target test_avbd_rigid_block \
  --parallel "$JOBS"
build/default/cpp/Release/bin/test_avbd_rigid_block \
  --gtest_filter='AvbdRigidBlock.RigidWorldContactApplyReusesSnapshotDirtyStackAllocator' \
  --gtest_color=no
build/default/cpp/Release/bin/test_avbd_rigid_block --gtest_color=no
pixi run lint
pixi run build
pixi run test-unit
```

Do not continue optimization, add scenes, run verification, push, or open a PR
from this stop state unless the maintainer explicitly resumes the work. The
validation list above records the already-completed AVBD slice evidence; no
lint, build, test, or benchmark command was run for this handoff update, by
explicit maintainer instruction.

## Hard Stop Handoff (2026-06-12, Stop-on-Handoff)

Resume from exactly one branch:
`pr/hmm-phase45-replay-snapshot-allocators`, tracking
`origin/pr/hmm-phase45-replay-snapshot-allocators`. Keep this as the single
handoff branch unless a maintainer explicitly redirects the work.

Current local stop state:

- the latest implementation/evidence commit before this docs-only handoff is
  `0cf49869195` (`Record HMM EnTT comparative evidence`);
- the previous local commit is `e68b88d5756`
  (`Route public dirty traversal through World allocator`);
- at the start of this handoff update,
  `origin/pr/hmm-phase45-replay-snapshot-allocators` was still
  `a679d98c0d2` (`Record HMM hard stop handoff`);
- before this docs-only handoff edit, the local branch was ahead of origin by
  the 2 commits above and had no open PR at this handoff.

Do not continue optimization, add scenes, or run further verification from this
handoff state unless the maintainer explicitly resumes the work. If the next
agent is asked to publish or continue implementation, start from this same
branch and create the next bounded slice only after rechecking the current
diff/status.

No lint, build, test, or benchmark command was run for this handoff update, by
explicit maintainer instruction.

## Historical Slice (2026-06-12, Public Setter Dirty Traversal Allocators)

Resume from exactly one branch:
`pr/hmm-phase45-replay-snapshot-allocators`, tracking
`origin/pr/hmm-phase45-replay-snapshot-allocators`. This branch is the single
entry point for the current HMM follow-up unless a maintainer explicitly
redirects the work. It currently has no open PR and may be locally ahead of
origin until a maintainer approves pushing.

Latest local slice: public live `Joint` setters are covered by the
persistent-storage no-heap gate. The pre-fix gap was that direct component
assignment used bounded World-backed payload storage, but `Joint::setPosition()`
still dirtied descendant frame caches with a local default-allocator
`std::vector<entt::entity>` traversal stack.

The fix has three parts:

- the `Joint::setPosition()` dirty-subtree traversal stack now uses
  `common::StlAllocator<entt::entity>` backed by the owning World's free
  allocator;
- the parallel public `Frame::markSubtreeTransformCacheDirty()` helper routes
  its traversal stack through the same World allocator path;
- `World.WorldPersistentStorageUsesWorldFreeAllocator` now calls every public
  6-DOF live joint payload/limit setter under the global heap counter, covering
  bounded payload reuse plus `setPosition()` dirty-traversal scratch.

Validation for this slice:

```bash
cmake --build build/default/cpp/Release --target test_world --parallel "$JOBS"
build/default/cpp/Release/bin/test_world \
  --gtest_filter='World.WorldPersistentStorageUsesWorldFreeAllocator' \
  --gtest_color=no
pixi run lint
pixi run build
pixi run test-unit
```

Immediate follow-up candidates:

- Allocator comparative matrix: remaining standard/foonathan misses are still
  EnTT steady-state registry rows. Do not tune thresholds or keep allocator
  policy changes without a clean, apple-to-apples EnTT matrix improvement.
- Further Phase 4/5 deformable work should start from focused heap/no-growth
  probes around default-solver projected-Newton storage or production-scale
  deformable cases. Avoid broad scene expansion without a bounded follow-up PR
  scope.

Before publishing or opening any PR from this branch, rerun the relevant gates
from a clean source state and get explicit maintainer approval before pushing.

EnTT comparative evidence update:

- Source-policy run:
  `.benchmark_results/allocator_entt_public_setter_continuation_cpuauto.json`
  from
  `pixi run bm-allocator-comparative-check --only-entt-registry --baseline foonathan --baseline std --verbose --cpu-affinity auto --benchmark-random-interleaving`.
  It performed all 12 EnTT foonathan/std comparisons: 8 passed and 4 failed.
  The DART no-growth rows still reported zero allocator allocations and
  deallocations after prewarm. The current misses were
  `BM_EnttRegistry/256` vs std (ratio 1.002),
  `BM_EnttRegistry/2048` vs std (ratio 1.024),
  `BM_EnttRegistryBuild/256` vs foonathan (ratio 1.010), and
  `BM_EnttRegistryBuild/512` vs foonathan (ratio 1.012).
- Rejected probe:
  `.benchmark_results/allocator_entt_frame_large_align_probe_cpuauto.json`.
  Cache-aligning all large frame-backed STL blocks regressed the no-growth
  rows and produced 8 failed/noisy comparisons, so that code change was
  reverted.
- Rejected probe:
  `.benchmark_results/allocator_entt_frame_reset_probe_cpuauto.json`.
  Avoiding the redundant frame color reset store did not clear the matrix and
  still failed 4 comparisons, so that code change was reverted.

Next EnTT work should inspect storage layout/cache placement or run on a
quieter host before changing allocator policy. Keep the strict 12-row focused
checker green against both foonathan and std before retaining an optimization.

## Historical Slice (2026-06-12, Live Joint and Loaded Component Allocators)

Resume from exactly one branch:
`pr/hmm-phase45-replay-snapshot-allocators`, tracking
`origin/pr/hmm-phase45-replay-snapshot-allocators`. It remains the single
fresh-session entry point unless a maintainer explicitly redirects the work.
The branch currently has no open PR and may be locally ahead of origin until a
maintainer approves pushing.

Latest local slice: live joint component payloads and multibody adjacency
vectors have been moved off global-heap dynamic storage for the creation path.
The pre-fix gap was that replay joint snapshots were allocator-aware, but the
live `comps::Joint` fields still owned `Eigen::VectorXd` storage and creating a
floating joint still made three tiny global allocations from
`Link::childJoints` and `MultibodyStructure::{links,joints}`.

Follow-up in the same branch: binary load now rebinds serialized
allocator-aware component vectors to the loaded `World` free allocator. The
pre-fix gap was that serializer default construction left loaded multibody
adjacency, deformable persistent payloads, and serialized variational state on
the default allocator even when normal creation used the World hierarchy.

The fix has six parts:

- `comps::Joint` now stores runtime, passive-dynamics, command, and limit
  payloads in a bounded dynamic `JointVector` sized for DART's supported 0-6
  DOF joint types;
- joint creation, articulated-point joint creation, multibody link creation,
  and legacy joint deserialization initialize those fields with
  `comps::makeJointVector()` instead of temporary `Eigen::VectorXd`
  heap-backed payloads;
- replay vector capture/restore/layout comparison now accepts Eigen-compatible
  vector types instead of requiring exact `Eigen::VectorXd`;
- binary/PFR serialization treats bounded dynamic double column vectors like
  `Eigen::VectorXd`, preserving existing joint binary layout;
- `Link::childJoints` and `MultibodyStructure::{links,joints}` use
  `common::StlAllocator` and are rebound to each World's free allocator during
  normal creation while keeping the component structs aggregate-compatible for
  Boost.PFR serialization.
- `World::loadBinary()` rebinds loaded allocator-aware component vectors for
  multibody adjacency, deformable persistent payloads and boundaries,
  variational multibody history, and variational contact state to the loaded
  World's free allocator after entity remapping.

New regression coverage:

- `World.WorldPersistentStorageUsesWorldFreeAllocator` now gates 6-DOF floating
  joint creation and direct assignment of every live joint payload/limit vector
  under the global heap counter. It also covers the multibody adjacency vectors
  that are appended during link/joint creation.
- The same gate now saves/loads representative multibody and deformable worlds
  and checks that loaded allocator-aware component vectors use the loaded
  World's free allocator.

Validation for this slice:

```bash
cmake --build build/default/cpp/Release --target test_world --parallel "$JOBS"
build/default/cpp/Release/bin/test_world \
  --gtest_filter='World.WorldPersistentStorageUsesWorldFreeAllocator' \
  --gtest_color=no
build/default/cpp/Release/bin/test_world \
  --gtest_filter='World.Replay*:World.WorldPersistentStorageUsesWorldFreeAllocator' \
  --gtest_color=no
cmake --build build/default/cpp/Release --target test_serialization \
  --parallel "$JOBS"
build/default/cpp/Release/bin/test_serialization --gtest_color=no
pixi run lint
pixi run build
pixi run test-unit
```

Immediate follow-up candidates:

- Public `Joint` setter no-heap coverage was closed by the current continuation
  above.
- Allocator comparative matrix: remaining standard/foonathan misses are still
  EnTT steady-state registry rows. Do not tune thresholds or keep allocator
  policy changes without a clean, apple-to-apples EnTT matrix improvement.
- Further Phase 4/5 deformable work should start from focused heap/no-growth
  probes around default-solver projected-Newton storage or production-scale
  deformable cases. Avoid broad scene expansion without a bounded follow-up PR
  scope.

Before publishing or opening any PR from this branch, rerun the relevant gates
from a clean source state and get explicit maintainer approval before pushing.

## Previous Hard Stop Handoff (2026-06-12, Live Joint WIP Superseded)

Stop point for a fresh agent:
`pr/hmm-phase45-replay-snapshot-allocators`, tracking
`origin/pr/hmm-phase45-replay-snapshot-allocators`, remains the only branch to
resume from unless a maintainer explicitly redirects the work. The branch has
no open PR at this handoff.

Latest completed implementation commit:
`9e36753e090` (`Route deformable creation scratch through World allocator`).
That commit is the last validated implementation slice. It routes deformable
creation validation and derived-topology scratch through the World free
allocator and extends `World.WorldPersistentStorageUsesWorldFreeAllocator` to
cover richer deformable creation inputs.

Important local stop-state note: this session was stopped while an
experimental live-joint storage slice was dirty in the worktree. These edits
were not completed, not verified, and must not be treated as a passing state:

- `dart/simulation/comps/joint.hpp`
- `dart/simulation/io/auto_serialization.hpp`
- `dart/simulation/io/binary_io.hpp`

The intended direction of that unfinished slice was to replace live joint
`Eigen::VectorXd` payload fields with a bounded inline dynamic Eigen vector
for DART's supported joint DOF range, then teach serializer helpers to accept
that vector type. The work stopped before the required initialization and
compile fallout was handled in `world.cpp`, multibody code, serializer code,
and replay helper templates. A fresh agent should first inspect the dirty diff
and make an explicit choice to either continue that bounded live-joint slice or
discard it; do not assume it compiles.

No verification was run for this handoff update, by explicit maintainer
instruction. Before publishing, opening a PR, or using any result as current
evidence, rerun the relevant lint/build/test gates from a clean and intentional
source state.

## Historical Slice (2026-06-12, Deformable Creation Scratch Allocators)

Resume from exactly one branch:
`pr/hmm-phase45-replay-snapshot-allocators`, tracking
`origin/pr/hmm-phase45-replay-snapshot-allocators`. It remains the single
fresh-session entry point unless a maintainer explicitly redirects the work.
The branch currently has no open PR.

Latest local slice: deformable body creation validation and derived-topology
scratch now use the World free allocator. The pre-fix gap was that live
deformable persistent payloads had been moved to World-backed vectors, but
`World::addDeformableBody()` still used native-heap `std::set`/`std::map`
scratch for boundary node uniqueness, explicit surface-triangle uniqueness,
tetrahedron uniqueness, and tetrahedral boundary-surface derivation.

The fix has three parts:

- validation-time uniqueness sets now use `common::StlAllocator` backed by the
  World free allocator;
- derived tetrahedral boundary-surface face maps now use the same allocator;
- `World.WorldPersistentStorageUsesWorldFreeAllocator` now covers richer
  deformable creation inputs: spring edges, tetrahedra, derived surface
  topology, Dirichlet/Neumann boundary conditions with nested vectors, and a
  separate explicit surface-triangle topology body.

Validation for this slice:

```bash
cmake --build build/default/cpp/Release --target test_world --parallel "$JOBS"
build/default/cpp/Release/bin/test_world \
  --gtest_filter='World.WorldPersistentStorageUsesWorldFreeAllocator' \
  --gtest_color=no
pixi run lint
pixi run build
pixi run test-unit
```

Immediate follow-up candidates:

- Live joint component storage still uses `Eigen::VectorXd`; replay snapshots
  are allocator-aware but live multibody dynamic payload storage remains a
  separate evidence-first slice.
- Allocator comparative matrix: remaining standard/foonathan misses are still
  EnTT steady-state registry rows. Do not tune thresholds or keep allocator
  policy changes without a clean, apple-to-apples EnTT matrix improvement.
- Further Phase 4/5 deformable work should start from focused heap/no-growth
  probes around default-solver projected-Newton storage or production-scale
  deformable cases. Avoid broad scene expansion without a bounded follow-up PR
  scope.

Before publishing or opening any PR from this branch, rerun the relevant gates
from a clean source state and get explicit maintainer approval before pushing.

## Previous Hard Stop Handoff (2026-06-12, Superseded)

Resume from exactly one branch:
`pr/hmm-phase45-replay-snapshot-allocators`, tracking
`origin/pr/hmm-phase45-replay-snapshot-allocators`. This branch supersedes
`pr/hmm-phase45-follow-up-clean` and all older HMM follow-up branches as the
single fresh-session entry point unless a maintainer explicitly redirects the
work.

Last implementation slice before this stop: `69e242c5258` (`Route live
deformable storage through World memory`). It routes live deformable
persistent component payloads through the World free allocator and updates the
focused persistent-storage gate. The detailed scope, prior validation commands,
and remaining gaps are recorded in the continuation section below.

No verification was run for this hard-stop documentation update, by explicit
maintainer instruction. A fresh session must rerun the relevant lint/build/test
gate before publishing, opening a PR, or using any result as current evidence.

Stop-state rules for the next agent:

- Start from `pr/hmm-phase45-replay-snapshot-allocators`; do not resume from
  older HMM branches unless a maintainer says so.
- Treat this document and `RESUME.md` as the hand-off source of truth.
- Do not continue optimizing from this stopped session. Start a fresh bounded
  slice, collect evidence first, and avoid adding broad new scenes unless the
  follow-up PR scope explicitly calls for them.

## Historical Slice (2026-06-12, Live Deformable Storage Allocators)

Resume from exactly one branch:
`pr/hmm-phase45-replay-snapshot-allocators`, tracking
`origin/pr/hmm-phase45-replay-snapshot-allocators`. It was created from
`pr/hmm-phase45-follow-up-clean` at `31e0daa6877` after PR #2955 and PR #2956
merged. Treat `pr/hmm-phase45-follow-up-clean` and older HMM branches as
historical base branches unless a maintainer explicitly redirects the work.

Latest local slice: live deformable persistent component payloads are being
routed through the World free allocator during `World::addDeformableBody()`.
The pre-fix gap was that replay-owned `DeformableNodeState` snapshots were
allocator-aware, but the live `DeformableNodeState` and adjacent persistent
deformable component vectors still used plain `std::vector` storage.

The fix has four parts:

- `DeformableNodeState`, `DeformableSpringModel`,
  `DeformableMeshTopology`, and `DeformableBoundaryConditions` now expose
  allocator-aware vector storage plus World-allocator constructors;
- `World::addDeformableBody()` prepares and emplaces those components with
  `world.getMemoryManager().getFreeAllocator()`;
- serializer helpers and VBD/block-descent APIs accept allocator-neutral
  vectors via spans/templates instead of requiring default-allocator
  `std::vector` payloads;
- the stable neo-Hookean FEM input path stores tetrahedra as a span, so
  allocator-aware topology vectors remain compatible with the default solver.

New regression coverage:

- `World.WorldPersistentStorageUsesWorldFreeAllocator` now wraps creation of a
  minimal deformable body and asserts zero global-heap allocations while the
  World free-list allocation count grows. In debug builds it also checks the
  live node-state and rest-position vector payloads are owned by the World
  memory manager.

Focused validation run for this local slice:

```bash
cmake --build build/default/cpp/Release \
  --target test_world test_vbd_combined_descent \
           test_vbd_parallel_block_descent test_vbd_stepper \
  --parallel "$JOBS"
build/default/cpp/Release/bin/test_world \
  --gtest_filter='World.WorldPersistentStorageUsesWorldFreeAllocator' \
  --gtest_color=no
build/default/cpp/Release/bin/test_vbd_combined_descent --gtest_color=no
build/default/cpp/Release/bin/test_vbd_parallel_block_descent --gtest_color=no
build/default/cpp/Release/bin/test_vbd_stepper --gtest_color=no
```

Previous completed slice: transient replay restore reinserts recorded
`MultibodyVariationalState` and `VariationalContactDualState` components
without global-heap allocation when those transient components are missing
live. The pre-fix probe reproduced 21 global allocations / 793 bytes during
`restoreReplayFrame(1)` after manually removing the live transient components.

That fix has three parts:

- replay transient restore uses World-allocated scratch and allocator-aware
  construction/copy for the two variational transient payload types;
- replay restore only invalidates frame topology and rebuilds step-pipeline
  caches when the restored frame actually changes topology or solver policy;
- compute graph resource metadata and `WorldKinematicsGraph` resource ids use
  the graph allocator, closing the remaining allocation stack from
  kinematics/compute graph resource-name construction.

New regression coverage:

- `World.ReplayRecordingRestoresTransientVariationalComponents` removes the
  live transient variational components while replay recording is active, then
  asserts `restoreReplayFrame(1)` reinserts the recorded components with zero
  global heap allocations.
- `SimulationComputeStageMetadata.ResourceStorageUsesProvidedAllocator`
  asserts long compute resource identifiers allocate through the supplied
  allocator rather than the global heap.

Validation run for this local slice:

```bash
cmake --build build/default/cpp/Release \
  --target test_world test_compute_graph --parallel "$JOBS"
build/default/cpp/Release/bin/test_world \
  --gtest_filter='World.Replay*:World.WorldPersistentStorageUsesWorldFreeAllocator:World.AvbdGroundFrictionRowsAreActive:World.AvbdSelfContactFrictionGridRowsAreActive:World.AvbdSelfContactFrictionProductionGridRowsAreActive' \
  --gtest_color=no
build/default/cpp/Release/bin/test_compute_graph --gtest_color=no
```

Fresh-session entry point:

```bash
git fetch origin
git checkout pr/hmm-phase45-replay-snapshot-allocators
git pull --ff-only
git status -sb
git log --oneline --decorate -8
git diff --stat
```

Before any future publishing or PR work, rerun the relevant lint/build/test
gates from a clean source state. The branch currently has no open PR.

Immediate follow-up candidates for the next fresh session:

- Broaden deformable creation gates beyond the minimal node/rest-position case
  once validation-time uniqueness maps and derived topology scratch are routed
  through World memory or explicitly scoped out of the persistent-storage gate.
- Live joint component storage still uses `Eigen::VectorXd`; replay snapshots
  are allocator-aware but live multibody dynamic payload storage remains a
  separate evidence-first slice.
- Allocator comparative matrix: remaining standard/foonathan misses are still
  EnTT steady-state registry rows. Do not tune thresholds or keep allocator
  policy changes without a clean, apple-to-apples EnTT matrix improvement.

Before publishing or opening any PR from this branch, run the relevant lint,
build, and focused test gates from a clean source state and get explicit
maintainer approval before pushing.

## Historical Slice (2026-06-12, Replay Snapshot Payload Allocators)

Use exactly one branch as the fresh-session resume point:
`pr/hmm-phase45-replay-snapshot-allocators`. It was created from
`pr/hmm-phase45-follow-up-clean` at `31e0daa6877` after PR #2955 and PR #2956
merged. Treat `pr/hmm-phase45-follow-up-clean` and the older HMM branches as
historical base branches only unless a maintainer explicitly redirects the
work.

Current branch state:

- Committed locally: `40fe160da8c` (`Route replay snapshots through World
allocator`).
- Committed locally: `f5e7625a6e0` (`Route replay restore scratch through
World allocator`).
- Committed locally: `6568764f1f8` (`Route replay names through World
allocator`).
- Committed locally: `20d25cb08e6` (`Route replay joint payloads through World
allocator`).
- Committed locally: `53c57088f3a` (`Reject replay joint vector size drift`).
- Committed locally: `194bc3ad206` (`Record HMM replay allocator handoff`).
- Committed locally: `0c6ac25a7ef` (`Route deformable replay payloads through
World allocator`).
- Committed and pushed: `f46c6f19239` (`Record HMM final stop handoff`).
- Latest local slice: transient variational replay restore now reinserts
  missing recorded components without global-heap allocation by routing replay
  transient payloads and kinematics graph metadata through World/graph
  allocators.

The committed replay slices close the next replay ownership gaps. A new
non-empty replay ownership assertion first reproduced one global heap
allocation / 368 bytes when enabling replay recording for a World with one
rigid body. The fix routes `ReplayState::Frame` snapshot vectors through the
World free allocator and adds an allocator-aware AVBD warm-start replay capture
overload so World-owned replay recording can also allocate AVBD row snapshots
from the same allocator root. Extending the same test to restore that recorded
frame then reproduced three global heap allocations / 16 bytes from replay
restore scratch. The restore path now uses World-allocated rigid-body ordering
and transient component cleanup scratch, and writes restored rigid-body
transform components directly so it does not enter the generic subtree-dirty
helper that allocated the final default 4-byte stack node.

The replay-name slice closes the next measured heap allocation in richer
replay snapshots. Replay joint-layout and loop-closure names now use
World-allocated `ReplayState::SnapshotString` storage, and replay capture,
layout comparison, and loop-closure validation preserve/compare those
allocator-owned names. Before that fix, the long joint-name probe reproduced
one global heap allocation / 81 bytes during `setReplayRecordingEnabled(true)`.
The ownership gate now covers long joint-name and long loop-closure-name replay
record/restore paths.

The replay-payload slice removes replay-owned `Eigen::VectorXd` storage from
joint snapshots. Joint runtime vectors, joint layout vectors, and joint limit
vectors are captured as World-allocated scalar snapshot vectors and restored
back into the live joint component vectors. The ownership gate and replay
runtime restore test now exercise a 6-DOF floating joint so the snapshot code
copies multi-coordinate dynamic payloads rather than only scalar revolute
payloads.

The replay-restore hardening slice makes joint runtime vector size drift a
layout error. Restore now copies into existing live vectors only after checking
that the live vector size matches the recorded snapshot, and
`World.ReplayRecordingRejectsJointRuntimeVectorSizeChanges` covers the
allocation-avoidance invariant.

The deformable replay-payload slice closes the next non-joint replay ownership
gap. A focused assertion in
`World.WorldPersistentStorageUsesWorldFreeAllocator` reproduced five global
heap allocations / 324 bytes during deformable node replay record, and the
same count during restore. `ReplayState::DeformableNodeStateSnapshot` now owns
World-allocated payload vectors for positions, previous positions, velocities,
masses, and fixed flags, and restore copies into existing live vectors only
after size checks. `World.ReplayRecordingRejectsDeformableNodeVectorSizeChanges`
covers the allocation-avoidance invariant for corrupted live node-state sizes.

The transient replay-restore slice closes the next replay restore allocation
gap. A focused probe in
`World.ReplayRecordingRestoresTransientVariationalComponents` reproduced 21
global heap allocations / 793 bytes when restoring a recorded frame after the
live `MultibodyVariationalState` and `VariationalContactDualState` components
were removed. Restore now uses allocator-aware construction/copy for those
transient payloads, skips unnecessary step-pipeline cache rebuilds when replay
restore does not change topology or solver policy, and routes compute graph
resource metadata plus `WorldKinematicsGraph` resource ids through the graph
allocator. `SimulationComputeStageMetadata.ResourceStorageUsesProvidedAllocator`
covers the compute metadata allocator invariant directly.

Validation for the replay-name, replay-payload, restore-size, deformable
replay-payload, and transient replay-restore slices:

```bash
pixi run lint
pixi run build
pixi run test-unit
cmake --build build/default/cpp/Release --target test_world --parallel 3
build/default/cpp/Release/bin/test_world \
  --gtest_filter='World.WorldPersistentStorageUsesWorldFreeAllocator' \
  --gtest_color=no
build/default/cpp/Release/bin/test_world \
  --gtest_filter='World.Replay*:World.AvbdGroundFrictionRowsAreActive:World.AvbdSelfContactFrictionGridRowsAreActive:World.AvbdSelfContactFrictionProductionGridRowsAreActive' \
  --gtest_color=no
build/default/cpp/Release/bin/test_compute_graph --gtest_color=no
```

Live joint component storage still uses `Eigen::VectorXd`, and live deformable
node component storage still uses plain `std::vector`; the replay changes only
change replay-owned snapshot payloads and reject runtime vector size drift
during restore. Richer replay restores may still allocate if they touch other
non-joint replay payloads that remain native heap-owned. Treat those as
separate evidence-first slices; do not claim full World replay zero-heap
coverage from the current replay gates.

The first continuation probe re-ran the focused EnTT comparative gate on the
retained source policy:
`.benchmark_results/allocator_entt_followup_current_random_cpuauto_reps7_20260612.json`.
It is not decision-quality allocator evidence: all 12 EnTT comparisons were
rejected by the strict checker as noisy under the current host load, while the
DART no-growth rows still reported zero post-prewarm allocator calls. Do not
change allocator policy from that run; use it only as evidence that timing
measurement is currently noisy.

Fresh-session entry point:

```bash
git fetch origin
git checkout pr/hmm-phase45-replay-snapshot-allocators
git pull --ff-only
git status -sb
git log --oneline --decorate -8
git diff --stat
```

Then verify from scratch before any new code, benchmark, or PR work.

Immediate follow-up candidates for the next fresh session:

- Non-joint replay payloads: `comps::DeformableNodeState` replay snapshots are
  now allocator-aware, but live `DeformableNodeState` component storage still
  uses plain `std::vector`. Decide separately whether persistent deformable
  node storage should move to World-backed vectors; that is broader than the
  replay snapshot fix.
- Additional transient replay component types: `MultibodyVariationalState` and
  `VariationalContactDualState` now have allocator-aware restore paths. Use a
  focused heap counter before adding similar paths for any other transient
  component type.
- Allocator comparative matrix: the remaining standard/foonathan misses are
  still EnTT steady-state registry rows, especially fixed 1024-element
  component payload pages plus sparse/packed entity pages. Do not lower
  checker thresholds or keep cache-line/coloring tuning unless the full EnTT
  matrix improves for an apple-to-apples workload reason.

Before publishing or opening a PR from this branch, run `pixi run lint` plus
the selected focused test/build gate for the final changed scope. Do not
push/open a PR without explicit maintainer approval.

## Historical Stop Handoff (2026-06-12, Superseded)

This stop point is superseded by the replay snapshot allocator handoff above.
Do not resume from this section unless a maintainer explicitly asks to discard
the replay snapshot allocator continuation.

Use exactly one branch as the resume point:
`pr/hmm-phase45-follow-up-clean`, tracking
`origin/pr/hmm-phase45-follow-up-clean`. PR #2955 and PR #2956 are merged.
Other HMM branches are historical/no-resume targets unless a maintainer
explicitly redirects the work.

The only post-PR #2956 continuation branch to preserve is
`pr/hmm-phase45-follow-up-clean`. It contains the dynamic rigid IPC no-growth
slice plus the latest EnTT comparative handoff evidence. No open PR exists for
this branch at this checkpoint.

Stop-state notes for the next fresh Claude/Codex session:

- The interrupted dense-page `StlAllocator` experiment for 1024-element EnTT
  component payload pages is not retained. The source tree is intentionally
  restored to the pushed allocator policy.
- Do not use any benchmark output produced after the stop request as
  validation for this handoff. Restart evidence collection from a clean
  session before keeping allocator or simulation changes.
- Continue only after explicitly choosing a fresh follow-up branch/PR. Do not
  add more implementation work to the merged PR #2956 history.

Current evidence narrows the remaining allocator-comparative surface to EnTT
steady-state registry layout rather than allocator calls:

- A focused EnTT checker on the pushed source policy,
  `.benchmark_results/allocator_entt_resume_cpu5_reps5_20260612.json`, had clean
  CVs for most rows and showed DART no-growth rows making zero allocator calls
  after prewarm. `BM_EnttRegistry/512` passed both foonathan and std, but
  `BM_EnttRegistry/256` missed foonathan/std, `BM_EnttRegistry/2048` missed
  foonathan/std, and `BM_EnttRegistryBuild/2048` missed std.
- A natural-alignment `StlAllocator` probe,
  `.benchmark_results/allocator_entt_natural_stl_cpu5_reps5_20260612.json`,
  fixed the 256-entity no-growth row and the 2048 build row, but still missed
  2048 no-growth, made the 512 no-growth rows noisy, and missed the 512
  build-vs-foonathan row. That broad policy is not retained.
- A later L1-sized alignment-threshold probe ran during severe host load
  (`/proc/loadavg` over 40) and is not decision-quality evidence.

No allocator code change is retained from these probes. Continue from a clean
source state. The next useful allocator step is storage-layout analysis for
the EnTT no-growth rows, especially the fixed 1024-element EnTT component
payload pages and sparse/packed entity pages. Do not lower checker thresholds
or keep free-list color-cycle tuning unless it improves the full EnTT matrix
for a workload reason that applies beyond one entity count.

Fresh-session entry point:

```bash
git fetch origin
git checkout pr/hmm-phase45-follow-up-clean
git pull --ff-only
git status -sb
git log --oneline --decorate -8
```

Then read `RESUME.md` and verify from scratch before any new code, benchmark,
or PR work.

## Authoritative Stop Handoff (2026-06-11, Final)

Maintainer instruction for this checkpoint: stop all implementation,
optimization, benchmark, build, lint, test, and CI-followup work. This update
is handoff documentation only. No verification is intentionally run for this
handoff commit.

Use exactly one branch as the resume point:
`pr/hmm-phase45-follow-up-clean`, tracking
`origin/pr/hmm-phase45-follow-up-clean`. PR #2955 and PR #2956 are merged.
Other HMM branches are historical/no-resume targets unless a maintainer
explicitly redirects the work.

Latest pushed source checkpoint before this docs-only handoff:
`ebecddd5afb` (`Gate dynamic rigid IPC no-growth`). That checkpoint is the
source state a fresh agent should verify before any new edits. The branch had
no open PR as of the handoff check.

Current stop-state notes for the next fresh session:

- The temporary allocator experiment that changed
  `FreeListAllocator::mLargeAlignedAllocationColor` from `4` to `0` was not
  retained. It moved EnTT comparative failures/noise rather than producing a
  clean matrix, so the source is restored to the pushed color-4 policy.
- Before the stop request, focused baked dynamic rigid IPC no-growth/no-heap
  gates passed locally. Broader existing default deformable/contact-family
  no-growth/no-heap gates also passed locally in a focused run. Treat those as
  prior-session evidence, not validation of this handoff commit.
- The latest allocator comparative investigation found the broad DART-vs-
  foonathan/std matrix still blocked on EnTT registry rows, not on allocator
  call counts. A full checker run performed 94 comparisons and failed 9 EnTT
  rows. A focused 9-repetition random-interleaved EnTT run reduced that to one
  small 256-entity no-growth miss versus foonathan. A warmup-focused run still
  missed three no-growth rows. A longer min-time run was too noisy to use.
- Do not lower thresholds, change checker policy, or tune cache-line coloring
  just to satisfy one row. Any allocator fix must be apple-to-apples for
  persistent EnTT registry/component storage and must keep broad allocator
  behavior green.

Fresh-session entry point:

```bash
git fetch origin
git checkout pr/hmm-phase45-follow-up-clean
git pull --ff-only
git status -sb
git log --oneline --decorate -8
```

Then verify from scratch before editing. Future Phase 4/5 implementation work
should start from this branch state and move to a fresh follow-up PR branch
when code changes resume.

## Historical Slice (2026-06-11, Dynamic Rigid IPC No-Growth)

Work resumed from the prior stop handoff on the same single continuation
branch: `pr/hmm-phase45-follow-up-clean`, tracking
`origin/pr/hmm-phase45-follow-up-clean`.

The latest Phase 4/5 slice closes a dynamic rigid IPC coverage mismatch without
adding a new scene. `World.BakedDynamicRigidIpcStepsDoNotAllocateGlobalHeap`
already covered the dynamic IPC solve graph, active barrier, fixed/revolute
joint constraints, two-box stack, mixed rigid/deformable surface obstacle,
kinematic conveyor, and kinematic turntable. The World-base no-growth side only
covered the mixed rigid/deformable surface and the two kinematic-contact cases.

`tests/unit/simulation/world/test_world.cpp` now adds the focused
`World.BakedDynamicRigidIpcStepsDoNotGrowWorldBaseAllocator` gate using those
same existing scene helpers. The three existing dynamic rigid IPC no-growth
rows were moved out of the monolithic reserved-ECS no-growth test so the full
dynamic rigid IPC matrix is owned by one focused test.

Verification for this slice:

- `pixi run lint`
- `pixi run build`
- `cmake --build build/default/cpp/Release --target test_world --parallel 2`
- `build/default/cpp/Release/bin/test_world --gtest_filter='World.BakedDynamicRigidIpcStepsDoNotGrowWorldBaseAllocator' --gtest_color=no`
- `build/default/cpp/Release/bin/test_world --gtest_filter='World.BakedDynamicRigidIpcStepsDoNotGrowWorldBaseAllocator:World.BakedDynamicRigidIpcStepsDoNotAllocateGlobalHeap' --gtest_color=no`

## Prior Critical Handoff Stop (2026-06-11, Historical)

Maintainer request at that point: stop all optimization, scene expansion, and
verification work immediately. This section is retained as history and is
superseded by the current continuation section above.

Use exactly one continuation branch:
`pr/hmm-phase45-follow-up-clean`, tracking
`origin/pr/hmm-phase45-follow-up-clean`. PR #2955 and PR #2956 are merged.
Other HMM branches are historical/no-resume targets unless a maintainer
explicitly redirects the work.

Current branch-head intent:

- Preserve the post-PR #2956 follow-up work already on
  `pr/hmm-phase45-follow-up-clean`.
- Preserve the latest in-progress Phase 4/5 slice for the default-solver
  deformable projected-Newton iterative sparse path.
- Do not add more scenes, scratch-reuse work, benchmarks, or allocator probes
  in this handoff.

Latest code slice captured by this handoff:

- `dart/simulation/compute/world_step_stage.cpp` replaces the opt-in Eigen
  incomplete-Cholesky CG branch for assembled projected-Newton Hessians with a
  DART-owned sparse Jacobi-CG loop that reuses the existing
  projected-Newton scratch vectors plus a pre-sized inverse-diagonal vector.
- `tests/unit/simulation/world/test_world.cpp` adds
  `configureDeformableIterativeFemGroundFrictionBlockScene()`, gates that shape
  in both baked World-base and global-heap guards, and adds
  `World.DeformableIterativeFemGroundFrictionBlockIsActive` so the case must
  exercise iterative projected-Newton solves.
- This was motivated by a measured pre-fix failure in
  `World.BakedMultibodyAndDeformableStepsDoNotAllocateGlobalHeap`: the
  iterative FEM ground-friction subcase made 25,064 global heap allocations /
  651,456 bytes over four baked steps while the World-base no-growth gate
  passed.

Verification boundary:

- No lint, build, test, benchmark, allocator probe, or CI verification was run
  after this stop request.
- Before the stop request, focused `test_world` coverage for the iterative FEM
  slice had passed locally, and `pixi run lint` had passed. A broader
  `pixi run build` command had been started before the stop request; do not use
  any post-stop output from that command as validation for this handoff.
- A fresh agent must verify the branch head before additional code changes or
  before opening a PR.

Fresh-session entry point:

```bash
git fetch origin
git checkout pr/hmm-phase45-follow-up-clean
git pull --ff-only
git status -sb
git log --oneline --decorate -8
```

Then read `RESUME.md` and the remaining Phase 4/5 follow-up list below.
Continue evidence-first only after reproducing a real allocator/no-heap gap.

## Historical Slice (2026-06-11)

Work resumed from the prior handoff on the same single continuation branch:
`pr/hmm-phase45-follow-up-clean`, tracking
`origin/pr/hmm-phase45-follow-up-clean`.

The latest Phase 4/5 slice closes a default-solver deformable projected-Newton
iterative sparse-solver gap. A new FEM ground-friction subcase with
`useIterativeLinearSolver=true` did not grow the World base allocator after
bake, but failed the baked global-heap gate before the fix with 25,064 global
heap allocations / 651,456 bytes over four baked steps. The allocation source
was Eigen's local incomplete-Cholesky CG path in the opt-in iterative sparse
projected-Newton solve. That path now uses a DART-owned sparse Jacobi-CG loop
over the assembled Hessian, reusing the existing projected-Newton scratch
vectors and a pre-sized inverse-diagonal vector from the World allocator.

The current branch now gates that iterative sparse FEM ground-friction shape in
both:

- `World.BakedStepsDoNotGrowWorldBaseAllocatorForReservedEcsPaths`
- `World.BakedMultibodyAndDeformableStepsDoNotAllocateGlobalHeap`

`World.DeformableIterativeFemGroundFrictionBlockIsActive` verifies the subcase
actually exercises iterative projected-Newton solves and iterations rather than
falling back silently.

## Final Stop Handoff (2026-06-11, No Further Verification)

Maintainer stop request: stop implementation and verification immediately,
focus only on handoff, push that handoff, and stop. No lint, build, test,
benchmark, allocator probe, or CI verification is intentionally run for this
final docs-only handoff.

Use exactly one continuation branch:
`pr/hmm-phase45-follow-up-clean`, tracking
`origin/pr/hmm-phase45-follow-up-clean`. PR #2955 and PR #2956 are merged.
Other HMM follow-up branches are historical/no-resume targets unless a
maintainer explicitly redirects the work.

The pushed head of `pr/hmm-phase45-follow-up-clean` after this handoff is the
single branch a fresh Claude/Codex session should start from. Treat older
sections below as evidence history only. When they conflict with this section,
this section and `RESUME.md` are authoritative.

The current branch preserves the post-PR #2956 continuation work:

- `tests/unit/simulation/world/test_world.cpp` adds dynamic rigid IPC
  kinematic-conveyor and kinematic-turntable scenes and wires them into the
  baked World-base no-growth and global-heap no-allocation gates. These reuse
  existing rigid IPC behavior shapes but route them through the built-in
  `World` IPC solver after bake.
- `docs/dev_tasks/hierarchical_memory_manager/README.md` and `RESUME.md`
  record the branch, status, and remaining Phase 4/5 gaps for a fresh session.

Verification boundary for this continuation:

- This final handoff commit intentionally skips all validation after the latest
  stop request.
- Work later resumed on this same branch. `pixi run lint`, `pixi run build`,
  the existing `World.RigidIpcKinematicTurntableCarriesRestingBox` behavior
  regression, and the focused baked allocator gate pair passed after adding the
  turntable subcase:
  `World.BakedStepsDoNotGrowWorldBaseAllocatorForReservedEcsPaths` and
  `World.BakedDynamicRigidIpcStepsDoNotAllocateGlobalHeap`.
- `pixi run test-unit` passed all 161 tests for this checkpoint.
- The branch still has no open PR. A fresh agent should verify current state
  before adding more code.

Fresh-session entry point:

```bash
git fetch origin
git checkout pr/hmm-phase45-follow-up-clean
git pull --ff-only
git status -sb
git log --oneline --decorate -8
```

Then read `RESUME.md` and the remaining Phase 4/5 follow-up items in this
file. Continue evidence-first: reproduce a real allocator/no-heap gap before
editing. Do not add more work to PR #2956; it is already merged.

## Historical Handoff

Work resumed after the prior critical stop handoff on the same authoritative
branch, `pr/hmm-phase45-follow-up-clean`. The current slice adds baked
World-base no-growth and global-heap no-allocation coverage for a dynamic rigid
IPC mixed rigid/deformable surface-obstacle solve and dynamic rigid IPC
kinematic-conveyor and kinematic-turntable contact solves. The first scene
mirrors the existing
`RigidIpcContactStageUsesDeformableSurfaceObstacle` behavior coverage but runs
through the built-in `World` IPC solver after bake; the kinematic scenes mirror
the existing conveyor and turntable behavior shapes and cover kinematic-trace
storage, active dynamic contact, and linear plus angular lagged-friction
surface motion in the same baked loop.

No implementation scratch change was needed for these shapes: both
`World.BakedStepsDoNotGrowWorldBaseAllocatorForReservedEcsPaths` and
`World.BakedDynamicRigidIpcStepsDoNotAllocateGlobalHeap` pass with the new
mixed rigid/deformable, conveyor, and turntable IPC subcases. This closes
those specific documented coverage gaps; remaining rigid IPC follow-up should
start from newly measured larger mesh/contact-set, articulated-contact-stack,
or other shapes that expose real allocator traffic.

Verification note for this slice: the focused `test_world` gate pair passed
with the mixed rigid/deformable, conveyor, and turntable subcases. The current
continuation also reran `pixi run lint`, `pixi run build`, and
`pixi run test-unit`.

## Prior Critical Stop Handoff

Maintainer requested on 2026-06-11: stop implementation work and focus only on
handoff, with no further verification. This docs-only handoff must not be read
as a new validation point; the next agent should start fresh and verify before
making code changes.

Use exactly one branch for continuation:
`pr/hmm-phase45-follow-up-clean`, tracking
`origin/pr/hmm-phase45-follow-up-clean`. PR #2955 and PR #2956 are merged.
Other HMM follow-up branches are historical/no-resume targets unless a
maintainer explicitly says otherwise.

Latest validated source checkpoint before this handoff:
`e9b2014f3db` (`Prewarm rigid IPC stack solve scratch`). It was pushed and
validated with lint, build, focused rigid IPC world/barrier tests, and full
`pixi run test-unit`. The handoff-only docs commit after it intentionally skips
`pixi run lint`, build, tests, and benchmarks.

Current completed Phase 4/5 slice:

- Rigid IPC projected-Newton solve scratch now persists assembly and
  line-search buffers on the World allocator.
- `RigidIpcContactStage::prepare()` prewarms same-shape solve/result surface
  buffers, articulation rows, active constraint rows, and assembly scratch.
- The dynamic rigid IPC no-heap gate now covers contact-free dynamics, active
  static/dynamic mesh barrier, fixed-joint and revolute-joint IPC constraints,
  the two-box stack, and a mixed rigid/deformable surface-obstacle barrier
  solve, plus kinematic-conveyor and kinematic-turntable active contacts with
  lagged friction.
- The measured two-box stack gap was 1 global allocation / 96 bytes over four
  baked steps from free-list growth during objective assembly; that gap is now
  closed at the latest validated source checkpoint.

Fresh agent entry point:

```bash
git fetch origin
git checkout pr/hmm-phase45-follow-up-clean
git pull --ff-only
git status -sb
git log --oneline --decorate -8
```

Then read `RESUME.md` and the remaining Phase 4/5 follow-up items in this
file. Continue only after measuring a real remaining allocator/no-heap gap.
Do not add more work to PR #2956; it is already merged.

## Current Handoff

Stop state for fresh handoff: use exactly one branch,
`pr/hmm-phase45-follow-up-clean`, tracking
`origin/pr/hmm-phase45-follow-up-clean` and based on `origin/main` after PR
#2956 landed. Other similarly named HMM follow-up branches are historical and
should not be used by fresh sessions unless a maintainer explicitly redirects
the work.

The final 2026-06-11 handoff snapshot is the pushed branch head after the
critical "handoff only, no further verification" request. It includes the
active dynamic rigid IPC barrier cleanup plus these handoff notes, but it was
committed without any fresh lint/build/test run after the stop request. The
previous fully validated code checkpoint was `57cb751eef9` (`Avoid heap
allocation in dynamic rigid IPC no-contact steps`). The docs-only stop commit
`91c3d83dd35` incorrectly described the
`WorldStorage` allocator-root slice as unapplied. Current source inspection
shows the branch already contains the equivalent, broader persistent-World
root-routing work: `WorldStorage`, the private built-in step-pipeline cache,
built-in stage-owned scratch/cache objects, the lazy collision query cache, and
the optional replay controller object are all constructed through the World
free-list allocator. Treat `git log` on `pr/hmm-phase45-follow-up-clean` as the
source of truth for the exact pushed branch head, and resume only from a fresh
session after reading this task state.

Current continuation after the final stop request: the rigid IPC two-box stack
WIP was completed and the temporary diagnostic patch was removed. The measured
gap was `World.BakedDynamicRigidIpcStepsDoNotAllocateGlobalHeap` failing for
`dynamic rigid IPC two-box stack` with 1 global allocation / 96 bytes over the
counted baked steps. The allocation came from free-list growth during
projected-Newton objective assembly after a large aligned triplet-scratch
request. `RigidIpcProjectedNewtonSolveScratch` now owns persistent
assembly/line-search scratch constructed from the World allocator, and
`RigidIpcContactStage::prepare()` prewarms same-shape solve/result surface
buffers, articulation rows, and assembly scratch before counted steps.

The focused baked dynamic rigid IPC no-heap gate now also covers fixed-joint
and revolute-joint articulated IPC constraints plus the two-box stack. The
focused gate passed locally after the debug-only heap backtrace and free-list
growth probes were removed. `pixi run lint`, `pixi run build`, focused rigid
IPC world/barrier tests, and `pixi run test-unit` also pass for this slice.

Fresh-session agents should start with
`docs/dev_tasks/hierarchical_memory_manager/RESUME.md`, then verify the live
checkout with:

```bash
git checkout pr/hmm-phase45-follow-up-clean
git status -sb
git log --oneline --decorate -5
```

The latest allocator-root slice closes a compute-graph
scratch gap: allocator-aware `ComputeGraph` traversal used to allocate from the
global heap while checking cycles, rebuilding topological order, validating,
and scanning resource hazards. The focused regression failed before the fix
with 38 global allocations / 880 bytes during allocator-aware traversal and now
passes with zero global heap allocation. This is pipeline scratch cleanup, not
a new deformable production-scene coverage claim.

The current continuation extends dynamic rigid IPC no-heap coverage in three
bounded steps. First, the single supported dynamic body with no possible
contact/articulation pairs takes the exact diagonal inertial quadratic
minimizer instead of constructing the full projected-Newton system; that baked
gate failed before the slice with 156 global allocations / 7920 bytes over four
steps. Second, an active static/dynamic mesh barrier shape no longer rebuilds a
single-node internal solve graph per step, prewarms the reusable projected
Newton step/result row storage during `prepare()`, and reuses those buffers
through the counted solve; that shape failed before the slice with 157 global
allocations / 7968 bytes over four baked steps. Third, the projected-Newton
solve scratch now owns persistent assembly/line-search buffers so stack and
articulated-contact shapes can prewarm their objective assembly scratch during
`prepare()`; the two-box stack failed before this slice with 1 global
allocation / 96 bytes over four baked steps. The covered dynamic rigid IPC
shapes now pass with zero global heap allocation.

The previous proven Phase 4/5 slice keeps the semi-implicit external-force
multibody path inside both World-base no-growth and global-heap no-allocation
gates after bake. The body-Jacobian container now reuses
`MultibodyDynamicsScratch::bodyJacobian`, and the remaining pure
external-force global-heap allocations were removed by skipping split
semi-implicit contact/unified collision queries when no relevant collision
shapes exist. A follow-up query-pruning slice makes the rigid contact stage
treat empty `CollisionGeometry` components like no collision geometry when
deciding whether prepare/execute needs a contact query; this is a scoped
performance guard, not a new claimed heap-gap closure.

The last full local validation for the code checkpoint was:

- `git diff --check`
- `pixi run lint`
- `pixi run build`
- `pixi run test-unit`
- Focused `test_world` no-growth/no-heap gates covering the forced-slider,
  multibody/deformable, and boxed-LCP fallback shapes.

Additional checks completed before the 2026-06-11 stop-and-handoff request for
the rigid empty-geometry query-pruning slice:

- Focused `test_world` contact gates:
  `World.BakedRigidBodyContactStepsDoNotAllocateGlobalHeap`,
  `World.BakedBoxedLcpFallbackContactsDoNotGrowWorldBaseAllocator`, and
  `World.BakedBoxedLcpFallbackContactStepsDoNotAllocateGlobalHeap`.
- `git diff --check`
- `pixi run lint`
- `pixi run build`

No additional verification was run after the stop request; in particular,
`pixi run test-unit` was not re-run for the latest rigid empty-geometry slice.

Additional checks completed for the compute-graph traversal slice:

- Before the fix:
  `SimulationComputeGraph.AllocatorAwareTraversalAvoidsGlobalHeap` failed with
  38 global allocations / 880 bytes.
- After the fix:
  `build/default/cpp/Release/bin/test_compute_graph --gtest_filter='SimulationComputeGraph.AllocatorAwareTraversalAvoidsGlobalHeap' --gtest_color=no`
  passed.
- Full `build/default/cpp/Release/bin/test_compute_graph --gtest_color=no`
  passed.
- `git diff --check`
- `pixi run lint`
- `pixi run build`
- `pixi run test-unit` passed all 161 tests.

Additional checks completed for the dynamic rigid IPC dynamics-only slice:

- Before the fix:
  `World.BakedDynamicRigidIpcStepsDoNotAllocateGlobalHeap` failed with 156
  global allocations / 7920 bytes over four baked steps.
- After the fix:
  `build/default/cpp/Release/bin/test_world --gtest_filter='World.BakedDynamicRigidIpcStepsDoNotAllocateGlobalHeap:World.RigidIpcContactStageAdvancesMeshBodyFromRuntimeDynamics:World.RigidIpcContactStageAdvancesSphereBodyFromRuntimeDynamics' --gtest_color=no`
  passed.
- A broader focused IPC `test_world` filter also passed after preserving the
  one-body IPC diagnostic counters.
- `git diff --check` and `pixi run lint` passed before the final handoff-only
  doc edits.

Additional check completed for the persistent-World root-routing status
correction:

- `cmake --build build/default/cpp/Release --target test_world --parallel "$JOBS"
&& build/default/cpp/Release/bin/test_world --gtest_filter='World.WorldPersistentStorageUsesWorldFreeAllocator' --gtest_color=no`
  passed, confirming the current branch's existing `WorldStorage`,
  built-in-cache, collision-cache, replay-controller, and `World::clear()`
  allocator-root coverage.

Additional checks completed for the active dynamic rigid IPC barrier slice:

- Before the fix:
  `World.BakedDynamicRigidIpcStepsDoNotAllocateGlobalHeap` failed for the
  active static/dynamic mesh barrier subcase with 157 global allocations / 7968
  bytes over four baked steps.
- After the fix:
  `cmake --build build/default/cpp/Release --target test_world --parallel "$JOBS"
&& build/default/cpp/Release/bin/test_world --gtest_filter='World.BakedDynamicRigidIpcStepsDoNotAllocateGlobalHeap' --gtest_color=no`
  passed.
- A broader focused rigid IPC `test_world` filter passed:
  `World.BakedDynamicRigidIpcStepsDoNotAllocateGlobalHeap`,
  `World.RigidIpcContactStageSeparatesActivatedMeshBarrier`,
  `World.RigidIpcContactStageAdvancesMeshBodyFromRuntimeDynamics`,
  `World.RigidIpcContactStageAdvancesSphereBodyFromRuntimeDynamics`, and
  `World.RigidIpcContactStageTwoBoxStackSettlesWithoutPenetration`.
- `git diff --check`, `pixi run lint`, and `pixi run build` passed before the
  critical final handoff request.

Additional checks completed for the dynamic rigid IPC stack/articulation
scratch slice:

- Before the fix:
  `World.BakedDynamicRigidIpcStepsDoNotAllocateGlobalHeap` failed for the
  `dynamic rigid IPC two-box stack` subcase with 1 global allocation / 96 bytes
  over four baked steps.
- The temporary allocation trace identified free-list growth during
  `assembleRigidIpcObjectiveSystemWithScratch()` through
  `solveRigidIpcProjectedNewtonBarrierSystem()` after a large aligned
  triplet-scratch request.
- After the fix:
  `cmake --build build/default/cpp/Release --target test_world --parallel "$JOBS"
&& build/default/cpp/Release/bin/test_world --gtest_filter='World.BakedDynamicRigidIpcStepsDoNotAllocateGlobalHeap' --gtest_color=no`
  passed with the debug-only heap/free-list logging removed.
- A broader focused rigid IPC `test_world` filter passed:
  `World.BakedDynamicRigidIpcStepsDoNotAllocateGlobalHeap`,
  `World.RigidIpcContactStageSeparatesActivatedMeshBarrier`,
  `World.RigidIpcContactStageAdvancesMeshBodyFromRuntimeDynamics`,
  `World.RigidIpcContactStageAdvancesSphereBodyFromRuntimeDynamics`,
  `World.RigidIpcContactStageTwoBoxStackSettlesWithoutPenetration`,
  `World.RigidIpcContactStageProjectsFixedJointPose`, and
  `World.RigidIpcContactStageProjectsRevoluteJointHingeAxis`.
- Full `build/default/cpp/Release/bin/test_rigid_ipc_barrier --gtest_color=no`
  passed.
- `git diff --check`, `pixi run lint`, `pixi run build`, and
  `pixi run test-unit` passed. `pixi run test-unit` passed all 161 tests.

The final handoff edits after the critical stop request intentionally skipped
`pixi run lint`, build, and tests. A `pixi run test-unit` job was in progress
and was terminated to honor the stop request, so no full `pixi run test-unit`
result should be inferred for the active dynamic rigid IPC barrier slice after
the listed focused checks. Later continuation slices should list their own
verification here, as the dynamic rigid IPC slice does above.

Continue only with evidence-first Phase 4/5 work from the remaining follow-up
items below. Do not add more scenes or scratch-reuse commits to PR #2956; that
PR is already merged.

## Current Status

- [x] Phase 1: Experimental `World` owns a memory manager and exposes
      frame-scratch diagnostics.
- [ ] Phase 2: Allocator correctness and performance gates cover DART
      allocators against standard C++ allocators and foonathan/memory.
      Alignment-aware allocation is implemented; fixed-size pool comparison now
      has a DART `FixedPoolAllocator` path that beats foonathan/memory and
      `std::pmr` locally. The comparative benchmark binary now honors the
      checker-requested repetition count instead of forcing five repetitions.
      The strict checker now rejects high-CV rows before treating ratios as
      evidence. `StlAllocator` keeps allocator-backed STL storage
      alignment-aware, including fixed-pool-backed max-aligned values. Focused
      EnTT registry probes now cover foonathan/memory's array-capable pool
      baseline and the standard registry, including separate build/growth rows
      for bake-time registry storage allocation. The focused EnTT checker
      caches known component storage handles, uses free-list-backed
      world-lifetime DART storage for persistent no-growth churn, and uses
      a resettable frame-backed DART bake arena for one-shot registry
      build/growth storage construction. The foonathan build/growth row uses
      `memory_stack` marker/unwind storage, matching the same bulk-lifetime
      role instead of comparing against a persistent pool collection. The
      no-growth row reports post-prewarm allocator-call counters and fails if
      reserved churn asks the configured allocator for storage after prewarm;
      the build/growth row times the uninstrumented
      construction/destruction path. The
      strict checker prints DART benchmark counters
      alongside
      pass/fail ratios, so EnTT misses distinguish timing loss, allocator
      traffic, growth, and noisy benchmark evidence. It also requires the
      expected benchmark keys for the selected mode, so missing
      foonathan/memory coverage is a failure instead of a skipped comparison.
      The checker rejects high-CV rows unless the saved mean/stddev/repetition
      aggregates still show DART's normal-approximation 95% confidence interval
      strictly below the selected baseline's confidence interval.
      The default comparative matrix now also covers foonathan/memory static
      fixed-storage stacks, scoped temporary allocators, and two-iteration frame
      allocators against the DART frame allocator's 32-byte fast path and
      `std::pmr` monotonic baselines. It also requires raw heap/malloc/new
      allocator rows plus aligned, fallback, segregator, tracked, and deeply
      tracked foonathan adapter rows mapped to DART frame/pool HMM roles and
      standard allocator baselines.
      `FreeListAllocator` now has a fixed-capacity mode for
      deterministic bounded failure after preallocation, and `MemoryManager` /
      DART 7 `WorldOptions` can construct the World free-list hierarchy
      with a fixed-capacity policy. Fixed-capacity free-list arenas can also
      satisfy over-aligned pool chunks from reserved bytes without growing from
      the base allocator. The current allocator correctness slice hardens
      count/size overflow guards for `MemoryAllocator::allocateAs`,
      `FrameAllocator`, `FrameStlAllocator`, and `StlAllocator`, adds focused
      pool/free-list/fixed-pool/frame/STL allocator coverage for invalid sizes,
      reuse, bounded failure, diagnostics, debug misuse paths, and
      allocator-root isolation across independent `MemoryManager` and
      experimental `World` instances, keeps aligned debug allocations tied to
      the matching aligned deallocation contract, releases debugger-tracked
      leaks with their recorded size/alignment during debugger destruction, and
      keeps fixed-capacity free-list aligned-allocation diagnostics on
      user-requested bytes instead of internal header padding. `FrameStlAllocator` blocks are cache-line
      aligned without per-block cache coloring, so frame-backed STL pages used by
      allocator-aware EnTT storage keep the alignment benefit without avoidable
      arena padding, and the frame allocator now has cheaper no-overflow
      reset plus 32-byte and cache-line-aligned fast paths for the comparative
      frame/STL rows. The STL-vector adapter benchmark is batched reserve-only
      allocator-adapter work so it measures allocator throughput instead of
      identical vector element writes. `PoolAllocator` now has an explicit
      diagnostics policy; release `MemoryManager` pool allocation and the
      comparative DART pool rows use the non-diagnostic hot path while direct
      `PoolAllocator` construction keeps live/peak counters enabled by default.
      The stack, frame-bulk, fallback-stack, small-pool, STL-vector,
      iteration, tracked-stack, and deeply tracked pool comparative rows now
      batch repeated allocator cycles so the strict CV gate measures sustained
      allocator work. `FixedPoolAllocator` also uses a cache-friendly stride
      for medium power-of-two slots, which removes the fixed-pool cache-set
      conflict that previously let foonathan/memory win `BM_Pool/256/256`.
      `PoolAllocator` default-size requests now use a constexpr heap-index
      lookup table for the same rounded/skewed size classes, removing repeated
      size-class arithmetic from mixed-size allocation/deallocation hot paths.
      A 2026-06-06 CPU-affined foonathan-plus-standard-plus-EnTT matrix,
      merged with focused replacement rows for strict-CV stability, passed all
      94 DART-vs-baseline comparisons, including EnTT no-growth and
      build/growth rows. Phase 2 remains open for broader HMM production
      no-growth coverage and any future allocator baselines, but the current
      DART allocator implementations now beat every required standard C++ and
      foonathan/memory row in the comparative matrix with non-noisy aggregate
      evidence. A 2026-06-07 continuation added allocator overflow and
      `construct`/`destroy` hooks to the STL adapters, cache-line colored
      frame-backed STL storage, cheaper no-overflow frame resets, and a
      fixed-pool DART steady-state churn row. The current foonathan-only
      matrix, merged with focused strict-CV replacement rows for the loaded
      host, passes all 47 DART-vs-foonathan checks, including EnTT no-growth,
      EnTT build/growth, steady-state, stack, frame, raw, adapter, and tracked
      rows. A later 2026-06-07/08 continuation restored explicit STL
      construct/destroy hooks, added a stateless `DefaultStlAllocator`, and
      refined the checker to require confidence-separated evidence for any
      accepted high-CV rows; the merged current foonathan broad-plus-EnTT
      result in
      `.benchmark_results/allocator_foonathan_broad_entt_current_check.json`
      passes all 47 foonathan comparisons. Re-run the standard-baseline half
      before making a fresh post-policy-change 94-row claim. Later
      random-interleaved EnTT diagnostics showed the pool-backed no-growth row
      is still too close to foonathan/memory at small sizes to treat the
      sequential artifact as the final "beats every foonathan allocator" proof;
      keep EnTT steady-state optimization open and use random interleaving for
      follow-up allocator-policy evidence. A follow-up switched persistent EnTT
      no-growth storage to the world free-list arena, which is the better DART
      HMM match for reserved variable-size registry arrays; the merged focused
      result in
      `.benchmark_results/allocator_entt_freelist_nogrowth_frame_build_current_merged_check.json`
      passes all 12 EnTT no-growth/build comparisons against both
      foonathan/memory and standard baselines with strict CV checks. The same
      continuation fixed the remaining mixed-size pool misses with the
      heap-index table; the current merged broad-plus-focused result
      `.benchmark_results/allocator_comparative_lookup_table_mixed_entt_merged_check.json`
      passes all 94 foonathan/memory and standard comparisons, including
      `BM_MultiPool`, `BM_Realistic`, EnTT no-growth, and EnTT build/growth.
      A 2026-06-08 follow-up restored cache-line alignment for large
      allocator-backed STL storage pages, switched the EnTT build/growth DART
      row to the stateless default C-heap STL adapter, and switched the
      foonathan build/growth baseline to `memory_stack` marker/unwind bulk
      lifetime storage. The full foonathan matrix in
      `.benchmark_results/allocator_comparative_foonathan_sustained_entt_stack_cpu13_check.json`
      passes all 47 DART-vs-foonathan comparisons, including EnTT no-growth
      and build/growth. The focused EnTT run in
      `.benchmark_results/allocator_entt_sustained_default_stack_cpu13_check.json`
      passes all 12 EnTT comparisons against both foonathan/memory and standard
      baselines; the full standard-registry half remains a separate
      post-policy-change gap before making a fresh 94-row
      standard-plus-foonathan claim.
      A later continuation corrected the build/growth DART row back to
      `StlAllocator` over `FrameAllocator`, matching foonathan/memory's
      `memory_stack` marker/unwind lifetime. Focused CPU6 probes show this
      role-correct build/growth path can beat foonathan and standard rows, but
      the persistent no-growth timing proof remains open because current runs
      are dominated by unrelated host load and the no-growth row still reports
      zero allocator calls after prewarm. A subsequent follow-up added
      runner-side CPU prewarm immediately before affinity-pinned benchmark
      launch and cache-line coloring for large 64-byte-aligned
      `FreeListAllocator` array allocations, then added shape-aware coloring
      for large 64-byte-aligned `FrameAllocator` storage. The coloring is based
      on the EnTT/World registry access pattern: consecutive page-sized
      component arrays can otherwise start on identical cache sets when their
      allocation sizes are multiples of the cache-index period. Persistent
      free-list-backed registry storage uses eight 64-byte colors with a
      shifted initial phase, while one-shot frame-backed build storage stays
      dense for scalar arrays and uses cache-line alignment only for
      cache-line-sized or over-aligned value pages.
      Focused probes show the 512-entity no-growth row can close the foonathan
      gap with zero post-prewarm allocator calls; a stricter focused check then
      reduced the remaining evidence gap to the `BM_EnttRegistryBuild/512`
      frame-backed build/unwind row. Frame allocator coloring reduced that row
      from ratio 1.043 to roughly 1.007 in a focused probe; the remaining
      adapter overhead is addressed by using DART's frame-native
      `FrameStlAllocator` for the build/growth row and by routing scalar
      frame-backed STL arrays directly to the inline default frame fast path
      instead of cache-line-aligning every large `std::vector` payload.
- [ ] Phase 3: EnTT registry/component storage allocation is configurable from
      the World memory hierarchy and covered by no-growth ECS tests.
      Allocator-aware EnTT storage now has focused `StlAllocator` and
      `FrameStlAllocator` unit tests showing that reserved
      create/emplace/read/destroy churn makes no configured allocator calls or
      arena growth after the prewarm pass. The DART comparative no-growth EnTT
      row now uses free-list-backed world-lifetime storage for reserved
      variable-size registry arrays and fails if reserved churn asks the
      configured allocator for storage after prewarm. The
      benchmark hot path caches known component storage handles so the timing
      surface matches optimized World systems rather than repeated registry
      type lookup. The focused no-growth benchmark now uses free-list-backed
      persistent array storage rather than the small-object pool path. Separate
      EnTT build/growth rows measure the bake-time
      storage allocation phase instead of conflating that cost with the
      no-growth simulation loop. The comparative benchmark now discovers
      installed EnTT package metadata for these rows without invoking DART's
      FetchContent-backed dependency helper, and the checker reports a clear
      error if a requested result file is empty.
      Initial registry/component-storage wiring and `enterSimulationMode()`
      reservation/no-growth tests for current World-owned ECS storage and the
      first multibody/deformable private step-scratch components are
      implemented. Direct EnTT create/emplace/clear/destroy/reuse storage
      cycling is covered after explicit reserve. `World::clear()` now recreates
      the internal allocator-backed registry storage so ECS capacities and
      debug-tracked registry allocations reset at the rebuild boundary while
      preserving the World memory hierarchy. Contact-heavy variational
      ground-contact dual state and compliant contact-point scratch are now
      sized at `enterSimulationMode()` and covered by World-base allocator
      no-growth tests that step multiple variational contact sliders without
      registry capacity growth. Pure compliant variational contact now
      evaluates directly from baked ground-contact scratch and reusable
      contact-evaluation vectors, so it no longer constructs the
      augmented-Lagrangian solver's dual vector; AL contact still owns solver
      dual scratch only when a positive dual-update cadence requires it. The
      compliant contact hook no longer copies contact-point storage in the
      step loop, and the single-prismatic World-surface contact path now
      evaluates ground contact without constructing the general residual/contact
      work vectors.
      Broader solver scratch coverage remains open.
- [ ] Phase 4: Built-in simulation stages borrow world memory for transient
      buffers and avoid growth after simulation is baked. The default
      `WorldStepPipeline` now stores built-in non-owning stage pointers inline,
      removing its per-build `std::vector` allocation from the normal step path.
      Longer custom pipelines keep the previous arbitrary-stage behavior through
      an overflow path. The legacy graph-backed `RigidBodyIntegrationStage` now
      reuses stage-owned rigid-body entity and dependency-node scratch instead
      of allocating per execute. The batched SoA rigid-body integration stage
      now reuses stage-owned force, state, model, initial-state, and
      parent-before-child frame-order scratch, and runs its single SoA kernel
      directly instead of rebuilding a one-node compute graph every execute.
      Custom batched SoA stages can now borrow a `MemoryManager` so their
      force and frame-order scratch vectors reserve from the World allocator
      root.
      The default rigid-body velocity/contact stages and semi-implicit
      multibody velocity/contact path now reuse baked scratch for the covered
      rigid and articulated resting-contact scenes, and the semi-implicit
      external-force branch now reuses baked body-Jacobian scratch instead of
      building a step-local Jacobian vector. The
      variational multibody stage now owns cache-only reusable contact/constraint
      scratch, bakes loop-closure and hard AVBD point-joint constraint capacity,
      evaluates pure compliant ground contact from baked ground-contact scratch,
      reserves augmented-Lagrangian solver scratch only for positive dual-update
      cadence contact, and bakes those vectors before contact-heavy steps. Its
      baked tree topology/link-index/child-list scratch now also borrows the
      World free allocator. It now routes the single-prismatic compliant
      ground-contact stage path through a scalar scratch-free solver fast path.
      A stronger compliant-contact global-heap
      probe now covers the baked multi-slider World-surface path without
      step-loop heap allocations. The boxed-LCP
      unified constraint stage now reuses stage-owned assembly containers and
      unified problem storage, and its shared/cross-row assembly no longer
      allocates per-step row-direction, rigid/articulated row-end, or
      shared-body inertia lookup containers. The stage path also assembles
      per-multibody link-contact rows through reusable
      `MultibodyDynamicsScratch` instead of the public return-by-value
      assembler, and cross-multibody row completion reuses the same scratch
      for other-link point Jacobians and joint-space denominator work instead
      of allocating local lookup/context/Jacobian temporaries. The Dantzig
      boxed-LCP solver now has caller-owned reusable scratch, a matrix/vector
      overload that avoids `LcpProblem` copies for already assembled systems,
      and a same-shape no-heap regression; `UnifiedConstraintStage` owns and
      reuses unified solve scratch that carries the Dantzig scratch plus island
      remapping/sub-problem buffers, successful link impulse application
      buffers, normal-only fallback buffers, and fallback tangent accumulators.
      Same-shape no-heap coverage now includes unified island solves, scratch-
      backed link impulse application, and rigid plus cross-multibody fallback
      friction sweeps.
      The unified assembler now reuses same-shape link-block row storage
      without per-row Eigen matrix-vector temporaries; same-shape no-heap
      coverage now also includes mixed rigid plus borrowed-link unified
      assembly. The boxed-LCP stage borrows per-multibody contact problems from
      persistent `MultibodyDynamicsScratch` instead of copying them into
      staging containers first. The rigid-contact assembler now has an in-place
      scratch overload, so same-shape fallback steps reuse the stage-owned
      rigid contact problem instead of building a by-value temporary each step.
      `UnifiedConstraintStage::prepare()` now primes the initial boxed-LCP
      contact shape during `enterSimulationMode()`, moving the current World
      fallback scenes' first active solve allocation out of the step loop. The
      same bake path now also primes normal-only fallback scratch
      unconditionally, so a same-shape solve that switches from the full
      boxed-LCP to the rank-deficient fallback does not allocate fallback
      matrices, tangent accumulators, or joint-space impulse buffers in the
      counted step loop.
      Public multibody link-contact assembly now has reusable scratch storage
      that can be borrowed by the in-place unified assembler without same-shape
      heap growth. The no-scratch public boxed-LCP solve wrapper now moves the
      solved lambda vector out of its local scratch instead of allocating a
      second result copy, and callers that want a `UnifiedConstraintSolution`
      object can now pass caller-owned result storage alongside solve scratch
      for same-shape no-growth solves. Rigid IPC accepted and rejected
      writeback now reuses the stage-owned blocked/writeback/entity-order
      scratch prepared with the rigid body capacity instead of allocating local
      traversal vectors, and the rigid IPC resting-contact no-op predicate now
      reuses stage-owned per-body contact-power and stationary-flag scratch.
      The rigid IPC projected-Newton loop now reuses solve-local surface buffers
      across line-search and sufficient-decrease backtracking candidates, and
      repeated solve-internal barrier assembly and line-search calls reuse
      surface-pair/triplet scratch, including the lagged-friction barrier pass.
      The rigid IPC contact stage now calls the projected-Newton solve through a
      caller-owned result/scratch overload so per-solve surface candidate
      buffers persist in stage scratch across steps.
      `DeformableDynamicsStage` now owns reusable
      obstacle-list, deformable surface-snapshot, and rigid
      surface-CCD snapshot scratch, and `prepare()` primes per-body
      surface-contact candidate buffers plus inter-body/rigid surface-CCD sweep
      buffers for baked steps. Scripted deformable boundary processing now
      reuses per-body Dirichlet/Neumann count masks instead of allocating local
      per-node vectors each step. Default deformable projected-Newton friction
      now reuses per-body normal-force, normal-direction, and self-contact
      friction-contact buffers instead of allocating local `std::vector`
      scratch in the step loop, and static-ground box CCD footprint clipping now
      uses fixed-size stack storage instead of allocating a tiny footprint
      vector on every non-vertical sweep. Surface-contact candidate and sweep
      buffers now get topology-scaled bake-time reserve capacity, so the covered
      frictional self-contact patch, 5x5 two-layer grid, 7x7 two-layer large
      grid, 9x9 two-layer production grid, and 11x11 two-layer extended
      production grid reuse candidate and friction-contact storage through
      projected-Newton line-search CCD. The direct-sparse default solver now
      carries that same no-growth coverage through 13x13, 15x15, 17x17, and
      dense 13x19 rectangular two-layer production grids, and the matrix-free
      default solver now carries the 17x17 larger, 13x19 rectangular, 7x17 wide,
      and 17x7 tall production guards through its CG scratch path. A mixed
      two-body production gate now steps one direct-sparse rectangular grid and
      one matrix-free wide grid in the same baked default-solver loop. A mixed
      dense production gate now combines a notched, jittered direct-sparse 13x17
      grid with a matrix-free 13x19 dense rectangular grid, and a mixed
      late-active production gate now steps direct-sparse square and matrix-free
      rectangular self-contact grids whose active contacts enter during the
      counted baked steps. These cover
      per-body solver/contact scratch storage for independent deformable bodies
      with different topology shapes and linear-solver modes. A default
      projected-Newton FEM ground-friction block now covers multi-tetrahedron
      rest-shape, Hessian-block, and multi-node ground-friction storage in both
      baked no-growth guards; compact and production mixed default-solver
      storage gates now combine direct-sparse self-contact, matrix-free
      self-contact, and FEM ground-friction bodies in one baked World memory
      root. The default projected-Newton path now also has a baked
      sphere/box/capsule static obstacle barrier gate, covering those radial and
      oriented-obstacle Hessian paths separately from the ground height-field
      barrier and surface-CCD snapshot gates. A production static-obstacle
      friction patch now slides independent node patches near sphere, box, and
      capsule barriers in one default projected-Newton solve, covering the
      shared static-obstacle normal/friction scratch for both sparse and
      matrix-free projected-Newton paths separately from self-contact friction.
      A mixed production gate now steps sparse static-obstacle friction and
      matrix-free self-contact bodies in the same baked World root, covering
      simultaneous obstacle and self-contact scratch without shared-root growth;
      a complementary mixed gate pairs matrix-free static-obstacle friction
      with a direct-sparse irregular self-contact grid to cover the opposite
      solver pairing in one root. The iterative sparse FEM ground-friction path
      now uses DART-owned Jacobi-CG scratch instead of Eigen's local IC-CG
      temporaries, and the baked no-growth/no-heap gates cover that
      `useIterativeLinearSolver=true` path separately from the direct-sparse
      and matrix-free projected-Newton paths. A complementary contact-family
      production gate now combines that matrix-free obstacle/direct-irregular
      self-contact pairing with inter-body surface CCD in one baked root.
      The moving rigid-surface CCD path now has baked
      swept-box point-crossing gates for both free predicted motion and
      kinematic trace-backed motion, including a multi-kinematic traced-obstacle
      scene that reuses combined swept snapshot capacity across independent
      deformable bodies. `prepare()` primes the kinematic swept-box
      snapshot buffers before a current-frame trace exists, so stage-owned
      moving rigid snapshot storage and surface-sweep candidate scratch are
      covered separately from static rigid snapshots. Projected-Newton sparse
      assembly now reserves self-contact barrier block storage from baked
      candidate
      capacity instead of only the
      bake-active candidate count, so same-topology active set variation does
      not grow DART-owned barrier vectors. Motion-aware
      self-contact candidate buffers now reserve the swept late-activation
      envelope during bake, and both the direct sparse and matrix-free
      projected-Newton paths now cover square and rectangular late-activating
      self-contact without growing step-loop candidate storage. A notched,
      jittered 13x17 two-layer production self-contact mesh now exercises the
      same direct-sparse default solver path through topology-scaled scratch
      rather than square-grid assumptions, and the same irregular mesh now
      covers the matrix-free projected-Newton path. The matrix-free path also
      reuses solver-owned Hessian block plus CG vector scratch instead of
      allocating local solve temporaries. Per-body deformable solver vectors
      for inertial targets, trial/gradient/direction/candidate state,
      previous-step positions, external accelerations, and fixed/boundary
      masks now construct from the World free allocator.
      AVBD ground contact/friction rows and
      self-contact normal/friction rows now reuse row-inventory and
      self-contact adjacency storage, including previous friction warm-start
      rows, and `prepare()` bakes row/candidate capacity for the covered AVBD
      ground-contact and two-surface contact scenes. AVBD row-inventory and
      friction-projection warm-start lookup keep rows in descriptor order while
      indexing the previous frame through sorted reserved storage, avoiding
      both map allocation and quadratic previous-row scans. Rigid AVBD contact
      projection now reuses stage-owned snapshot, point-joint, row-counter,
      row-inventory, inertial-target, and solve-row scratch for covered active
      rigid contacts and no-contact fixed-joint rows; the snapshot-apply
      transform writeback helper now reuses snapshot-owned frame-dirty
      traversal scratch; the public
      return-by-value AVBD helpers remain allocation-boundary conveniences. The
      follow-up VBD path now also reuses stage-owned Chebyshev history vectors
      and bakes self-contact candidate sweep plus adjacency capacity for all
      enabled VBD surface bodies, not only AVBD self-contact row bodies. The
      current
      production boxed-LCP stage
      uses the in-place unified assembler and solve scratch; the public
      return-by-value unified problem and solution wrappers remain
      allocation-boundary API conveniences rather than step-loop hot paths.
- [ ] Phase 5: Add allocation/debug accounting gates for "no dynamic allocation
      during the step loop" on representative rigid, multibody, contact, and
      deformable scenes. World base-allocator no-growth guards now cover baked
      kinematic IPC rigid-body, multibody variational, deformable ECS,
      rigid-body resting-contact, semi-implicit external-force multibody,
      non-cross articulated resting-contact, and same-DOF cross-articulated
      link-contact paths after contact prewarm; a
      global heap guard covers the same baked kinematic IPC, rigid-body,
      multibody variational, compliant variational contact, deformable,
      rigid-body resting-contact, semi-implicit external-force multibody,
      non-cross articulated resting-contact, and same-DOF cross-articulated
      contact paths. Mixed/different-DOF, stacked, and coupled multi-row
      cross-contact boxed-LCP fallback scenes now have base-allocator
      no-growth gates and first baked-step global heap no-allocation gates.
      A focused same-shape guard also covers the prewarmed batched SoA
      rigid-body integration stage on a frame-coupled parent/child body pair,
      proving its batch arrays and frame-order scratch do not allocate after
      prewarm.
      Five-multibody, eight-multibody, 12-multibody, 16-multibody,
      24-multibody, and 32-multibody stacked contact sets extend the boxed-LCP
      fallback gate beyond the original small scenes, and a production
      multi-island mixed scene now covers independent articulated and rigid
      contact islands with 12+ initial contacts; the stress multi-island gate
      extends that shape to 30+ initial contacts across independent
      articulated and rigid islands. A mixed stress boxed-LCP scene now combines
      the 32-multibody stacked fallback and stress multi-island shape under one
      baked World root with 60+ initial contacts. Broader solver coverage,
      including default-solver deformable storage and any newly exposed
      production contact shapes, remains open before making a full
      zero-allocation claim.
      The
      global heap guard now also covers a baked deformable surface-snapshot
      scene with a static rigid surface-CCD obstacle and first-baked-step active
      VBD static rigid surface-CCD point crossing. Default projected-Newton
      deformable scratch now reuses its RHS, sparse Hessian assembly, PSD block
      batches, sparse-pattern cache, and solution storage for the covered
      mass-spring path; default static rigid surface-CCD point crossing and an
      active inter-body deformable surface-CCD crossing are also covered by
      baked no-growth guards. FEM rest-shape caches are primed during
      `enterSimulationMode()`, and a one-tetrahedron FEM projected-Newton path
      is covered by the same guard; motion-aware surface-contact candidate
      scratch now reserves the wider swept-AABB envelope needed by non-square
      self-contact grids while staying linear in topology size; broader
      projected-Newton self-contact barrier scratch is sized from bake-primed
      contact candidates and covered for the two-triangle no-friction
      self-contact path. The global heap guard now also covers a baked
      default-solver deformable ground-friction projected-Newton scene, a
      multi-tetrahedron FEM ground-friction block, plus a multi-triangle
      frictional self-contact patch, a 5x5 two-layer frictional self-contact
      grid, a 7x7 two-layer large grid, a 9x9 two-layer production grid, an
      11x11 two-layer extended production grid, a 13x13 two-layer dense
      production grid, a 15x15 extra-dense two-layer production grid,
      direct-sparse and matrix-free 17x17 larger two-layer production grids, a
      9x13 non-square two-layer production grid, direct-sparse and matrix-free
      13x19 dense rectangular production grids, direct-sparse and matrix-free
      7x17 wide non-square production grids, direct-sparse and matrix-free 17x7
      tall non-square production grids, a notched and jittered 13x17 irregular
      direct-sparse production grid, a notched and jittered 13x17 irregular
      matrix-free production grid, and an 11x11 late-active two-layer
      direct-sparse plus matrix-free grid that starts outside the self-contact
      barrier band and enters it during the counted baked steps, with the same
      late-active direct-sparse plus matrix-free coverage for a 9x13
      rectangular grid. The same base and global-heap guards now also include
      a mixed two-body production scene with independent direct-sparse
      rectangular and matrix-free wide self-contact grids, a mixed late-active
      production scene with independent direct-sparse square and matrix-free
      rectangular self-contact grids, a mixed dense production scene with
      notched direct-sparse and dense matrix-free grids, a production mixed
      default-solver storage scene with direct-sparse self-contact,
      matrix-free self-contact, and FEM ground-friction bodies, a mixed
      production FEM ground-friction scene with direct and matrix-free
      4x4x4-node blocks, plus a production rectangular inter-body deformable
      surface-CCD crossing, a
      barrier-only static sphere/box/capsule obstacle scene, and a production
      static sphere/box/capsule obstacle friction patch for sparse and
      matrix-free default projected Newton, plus a mixed static-obstacle and
      self-contact production scene and the complementary matrix-free
      static-obstacle plus direct irregular self-contact scene that exercise
      both scratch families under one World memory root. A further mixed
      default-solver contact-family production scene now combines direct static
      obstacle friction, matrix-free self-contact friction, and inter-body
      surface CCD under one baked World root; the complementary contact-family
      scene now flips the static-obstacle path to matrix-free and the
      self-contact path to direct irregular topology while keeping production
      inter-body surface CCD in the same baked root.
      The base and global-heap guards now also include default moving
      rigid-surface CCD swept-box crossings for free, single-kinematic, and
      multi-kinematic rigid obstacles, closing the previously static-only rigid
      surface snapshot coverage and the first-traced-kinematic allocation gap.
      The
      larger-grid guards also assert
      non-vacuous solver activity through public deformable diagnostics: active
      self-contact barriers, converged active contacts, and positive friction
      dissipation. The global heap and World-base no-growth guards now also
      cover the compact and production rectangular inter-body deformable
      surface-CCD crossings, active AVBD ground contact/friction rows, AVBD
      self-contact normal/friction rows including 5x9 and 9x13 rectangular grid
      row workloads, and an active rigid AVBD penetrating contact plus
      no-contact fixed-joint rows.
      Additional broader or differently shaped production-scale frictional
      deformable scenarios still need no-growth gates before making the full
      deformable claim.
      The follow-up branch adds an active VBD Chebyshev self-contact grid to
      both the World-base no-growth and global-heap no-allocation guards,
      proving the baked VBD Chebyshev/self-contact path does not allocate after
      prewarm for that covered shape.
- [ ] Phase 6: Add memory-layout profiler/debugger surfaces and GUI
      visualization. `MemoryAllocatorDebugger` now exposes structured live
      bytes, peak live bytes, and live allocation count; `MemoryManager` and
      DART 7 `World` diagnostics now surface direct free/pool allocator
      debug counters. `WorldMemoryDiagnostics` also reports aggregate and
      per-storage ECS registry layout counters without exposing EnTT types, and
      dartpy exposes the same read-only snapshot through
      `World.memory_diagnostics`. The standalone `dartsim` editor now has a
      tested read-only Memory panel action seam that surfaces frame scratch,
      allocator debug counters, and largest ECS storage capacities. Broader
      profiler overlays and workflow-specific visualizations remain future
      work.

## Goal

Move the DART 7 simulation stack toward zero dynamic memory allocation in
the simulation loop by giving each `World` one hierarchical allocator root and
routing persistent and per-frame data through that root. Long term, the memory
hierarchy should map cleanly to world objects/components so it can drive memory
debugging, profiling, optimization experiments, and ImGui visualization.

## Non-Goals For The First Slice

- Do not claim zero allocations for every experimental solver path yet.
- Do not replace Eigen's internal heap behavior in one pass.
- Do not expose raw EnTT storage, solver registries, or backend resource types
  in public memory diagnostics; surface only DART-owned counters and diagnostic
  IDs.

## Key Decisions

- Reuse `dart::common::MemoryManager` as the allocator hierarchy root instead
  of introducing a simulation-only allocator stack for the first slice, but do
  not assume the current allocator implementations are correct or fast enough.
- Follow the existing common-module rule: `World` owns the memory manager;
  stages and components borrow allocators from the world.
- Treat frame scratch as valid for the current simulation frame and reset it at
  the start of the next `World::step()` call.
- Treat `entt::registry` allocation as a first-class integration target. The
  ECS storage layer is a dominant owner of world/component memory, so future
  work must either instantiate the registry/storage with a DART allocator or
  provide an equivalent EnTT storage integration that preserves public API
  boundaries.
- The active EnTT version supports stateful allocator propagation through
  `entt::basic_registry`: the DART 7 World now constructs its internal
  registry and component storages with a `dart::common::StlAllocator` borrowing
  the World's active free allocator.
- Free-list allocations must preserve at least `std::max_align_t` alignment
  after every split. EnTT component storage surfaced this as an Eigen
  `FrameCache` alignment failure when a max-aligned component allocation
  followed an odd-sized allocation.
- Broad hot-loop adoption is blocked until allocator correctness tests and
  benchmarks prove DART's allocators beat both standard C++ allocators and every
  required foonathan/memory allocator baseline on DART-relevant workloads. A
  missing foonathan/memory baseline is incomplete evidence, not a pass. If DART
  cannot beat foonathan/memory for required features and workloads, prefer an
  explicit dependency decision over shipping a weaker in-house allocator.
- Use `FixedPoolAllocator` for fixed-size node/slot workloads and keep
  `PoolAllocator` focused on mixed size-classed small-object workloads. The
  comparative benchmark must not compare DART's generic size-classed pool
  against foonathan's fixed-size pool when a DART fixed-size allocator exists.
- Use fixed-capacity `FreeListAllocator` instances when the allocator budget was
  established during world creation or bake/build and runtime growth would
  violate the no-dynamic-allocation contract. The default free-list policy
  remains expandable for general heap-like use. Route the policy through
  `MemoryManager::Options` and `WorldOptions` rather than exposing EnTT or
  solver storage types.

## Required Allocator Evidence

- Correctness tests: alignment (including over-aligned Eigen-like types),
  zero-size behavior, reuse-after-free, double-free/leak debug paths, frame
  reset semantics, overflow/growth accounting, deterministic failure handling
  for bounded/static storage, and multi-world isolation.
- Benchmarks: allocation/deallocation latency, container workloads, per-frame
  arena reset, pool reuse under churn, mixed-size free-list fragmentation,
  multibody/contact/deformable step-loop scratch patterns, and no-growth
  baked-world repeated simulation loops.
- Comparison targets: `std::allocator`/`std::pmr` where applicable, current
  DART allocators, improved DART allocators, and foonathan/memory's relevant
  raw allocators, pools, static storage, temporary allocator, adapters, and
  tracking/debug wrappers.
- Completion gate: the required comparative benchmark/checker matrix must cover
  every foonathan/memory allocator family that maps to a DART allocator role
  used by the HMM plan, and every DART implementation row must beat the matching
  foonathan/memory row with non-noisy aggregate evidence or
  confidence-separated high-CV evidence. Rows that are missing, noisy without
  separated confidence intervals, or slower keep this dev task open.

## Required EnTT Integration Evidence

- Registry construction: prove whether the active EnTT version can use a
  stateful allocator through `entt::basic_registry` and component
  `basic_storage`, or document the exact adapter/storage work needed.
- Component storage: cover create/emplace/destroy/clear/reuse patterns for the
  current DART 7 components, including sparse-set growth and component
  array capacity behavior. Initial direct coverage now reserves entity,
  `RigidBodyTag`, `Transform`, `Velocity`, and `Force` storages and verifies
  repeated create/emplace/clear/re-emplace/destroy cycles do not grow World
  base-allocator traffic after reserve.
- Bake/build reserve path: reserve registry entities and component pools before
  simulation so repeated `World::step()` calls do not grow ECS storage.
- No-growth proof: after the reserve/prewarm path, representative
  create/emplace/read/destroy churn must make zero calls through the configured
  DART allocator or grow the configured arena; benchmark timing alone is not
  sufficient evidence.
- Boundary rule: keep EnTT allocator/storage types out of the promoted public
  facade; expose only DART-owned memory options and diagnostics.

## PR #2956 Phase 4/5 Wrap-Up

PR #2956 stops at the proven coverage below. Remaining Phase 4/5 work moves to
a follow-up branch so this PR can ship the current allocation gates without
adding more scene surface.

Current Phase 4 scratch-reuse coverage shipped by this PR includes:

- Inline built-in stage storage for the default `WorldStepPipeline`, legacy
  graph-backed rigid integration scratch, and batched SoA rigid integration
  scratch for force/state/model/initial-state/frame-order buffers.
- Rigid IPC stage scratch for accepted/rejected writeback, resting-contact
  no-op detection, projected-Newton surface buffers, barrier assembly, line
  search, equality change-of-variable indices, and lagged-friction passes.
- Unified/boxed-LCP scratch for in-place rigid and multibody assembly, Dantzig
  solve reuse, island remapping, link impulse application, normal-only
  fallback, tangent accumulators, multibody staging vectors, and same-shape
  fallback friction sweeps.
- Variational multibody scratch for baked loop-closure, hard AVBD point-joint,
  finite-stiffness AVBD point-joint compliant-loop constraints,
  velocity-actuator projection rows, compliant/augmented-Lagrangian ground
  contact, and the scalar single-prismatic compliant-contact fast path.
- Default projected-Newton deformable scratch for obstacle lists, static and
  moving rigid surface snapshots, surface-contact candidates, boundary masks,
  friction buffers, sparse/matrix-free solver storage, FEM blocks, static
  obstacle friction, self-contact, and inter-body surface CCD.
- AVBD scratch for ground contact/friction rows, self-contact adjacency and
  warm-start lookup, rigid AVBD contact snapshot/row-counter setup, contact
  projection solve rows, warm-start inventories, inertial targets, and
  no-contact fixed-joint rows.

Current Phase 5 no-growth/no-heap gates shipped by this PR include:

- Baked rigid, kinematic IPC, resting-contact, batched SoA rigid integration,
  multibody variational, compliant-contact, same-DOF cross-articulated, and
  mixed/different-DOF boxed-LCP fallback scenes.
- Production boxed-LCP contact sets up to 32 stacked multibodies, independent
  multi-island rigid/articulated contacts, stress multi-island contacts, and a
  mixed stress stack plus multi-island scene with 60+ initial contacts.
- Default-solver deformable guards for frictional self-contact patches,
  square/rectangular/irregular direct-sparse and matrix-free two-layer grids,
  late-active contact patterns, mixed direct/matrix-free roots, FEM
  ground-friction blocks, static sphere/box/capsule obstacle friction, moving
  rigid-surface CCD, and inter-body surface CCD.
- Mixed contact-family default-solver scenes that combine static-obstacle
  friction, self-contact friction, and production inter-body surface CCD under
  one baked World memory root for both direct-sparse and matrix-free pairings.
- AVBD ground-contact, self-contact, rectangular-row, rigid-contact, and
  no-contact fixed-joint guards with World-base no-growth and global-heap
  no-allocation coverage after bake.

Follow-up progress after PR #2956:

- The VBD Chebyshev self-contact path now borrows stage-owned Chebyshev history
  vectors, bakes self-contact sweep/adjacency capacity for all enabled
  VBD surface bodies, and has active-scene plus World-base/global-heap gates for
  a 5x9 two-layer self-contact grid.
- The VBD topology element lists for springs and tetrahedra now live in
  `DeformableVbdScratch` with the borrowed World allocator. The internal VBD
  kernels consume those read-only lists through `std::span`, so standalone
  one-shot callers keep ordinary vectors while baked World-stage steps avoid
  heap-owned spring/tet topology storage.
- Contact-heavy `WorldRegistry` rebuild coverage now reuses the existing
  compliant variational contact slider setup to bake/step solver-owned ECS
  scratch, clear the `World`, verify registry capacities release to zero while
  preserving the world free-list allocator wiring, rebuild the same contact
  shape, and re-step with identical baked storage capacities and no base
  allocator activity in the baked step loop.
- Scripted deformable Dirichlet/Neumann boundary processing now has a focused
  active-scene check plus World-base/global-heap baked-step allocation guards,
  proving the per-body boundary masks and external-acceleration scratch stay
  inside the baked deformable solver storage after prewarm.
- Variational multibody manifold Anderson acceleration now uses the baked
  `MultibodyVariationalScratch` component for its step/iterate history,
  per-joint tangent temporary, least-squares work matrices, and trial
  positions. The World-stage path pre-sizes this storage during
  `enterSimulationMode()`, while the public one-shot integration helper keeps a
  local fallback scratch. Focused coverage extends the existing loop-closure
  scratch bake test and keeps the manifold Anderson spherical/floating
  regression tests passing.
- Variational multibody articulated inverse-mass and exact recursive-Newton
  solves now share baked linear-solve scratch for articulated operators, spatial
  bias/twist buffers, joint projectors, factored joint blocks, right-hand sides,
  and result vectors. The World-stage bake path pre-sizes this storage with the
  multibody shape, the public helper keeps a local fallback, and focused
  coverage checks the baked scratch shape while preserving dense inverse-mass,
  long-chain exact-Newton, loop-closure, and manifold Anderson regressions.
- Variational multibody step state now reuses baked scratch for generalized
  position, velocity, applied force, next-position trial, bootstrap spatial
  velocities, and forced-DEL residual storage. The hot-loop residual evaluation
  writes into caller-owned storage instead of returning a fresh vector per
  root-find or line-search trial, and the initial-guess retraction reuses
  existing per-joint tangent storage. The stage also routes its initial-guess
  inverse-dynamics query through a reusable scratch-backed overload, so
  same-shape baked steps no longer use the public return-by-value dynamics
  helper or allocate a temporary zero-acceleration vector for that bias query.
  Loop-closure projection now also reuses baked Jacobian, inverse-mass
  transpose, constraint-mass, factorization, lambda, and correction storage
  plus projection row bounds/row partitions instead of allocating dense
  projection work matrices or temporary row lists on every projection
  iteration. The projection loop refreshes the step tree's configuration in
  place for each candidate instead of rebuilding the tree topology, index map,
  and child vectors from the registry on every projection iteration. The
  initial per-step variational tree build now also writes into baked
  `MultibodyVariationalScratch` storage, so same-shape steps reuse the tree
  topology vector, link-index map, per-link child lists, and link-frame
  subspace matrices instead of constructing fresh containers; the same-shape
  map nodes stay alive rather than being cleared/reallocated in the step loop.
  The follow-up branch now also constructs the baked tree scratch pimpl, link
  vector, per-link child lists, and link-index map from the World free
  allocator. The same follow-up line routes the nested inverse-dynamics scratch
  pimpl and dynamics-tree vector payloads through that allocator for baked
  variational stages. Baked variational ground-contact point scratch and
  augmented-Lagrangian solver dual scratch now also allocate their reusable
  vector storage from the World free allocator, as do contact-evaluation
  transform/Jacobian scratch vectors for the same baked contact scenes.
  Existing variational loop-closure baked storage now also routes the step
  constraint staging vector, step spatial-velocity list, articulated
  linear-solve vector lists, projection Jacobian/row-index lists, and Anderson
  history lists through the same World allocator root. The AL contact
  dual-update post-transform list now borrows that allocator too.
  Velocity-actuator projection now follows the same baked projection path:
  bake-time sizing counts actuator target rows, the projection loop writes those
  rows directly into the reusable residual/Jacobian, and per-joint projection
  retractions write through existing scratch instead of allocating a temporary
  constraint list or return-by-value retract result.
  The existing World baked-step global-heap gate now includes an active
  loop-closure chain, covering the variational tree/projection scratch path at
  World level without broadening production scene scope.
- The existing compact and production mixed default-deformable storage scenes
  now have World registry clear/rebuild gates. They prove direct-sparse
  self-contact, matrix-free self-contact, and FEM ground-friction solver
  storage can be baked, stepped without base-allocator growth, cleared back to
  zero registry capacity, and rebuilt with the same storage capacities under
  the world free-list allocator.
- The production mixed default contact-family scene and its complementary
  matrix-free/static-obstacle plus direct-irregular/self-contact pairing now
  have the same clear/rebuild coverage, adding static-obstacle friction,
  self-contact friction, and inter-body surface CCD solver storage to the
  rebuild-boundary proof across both default-solver storage pairings.
- Existing AVBD/VBD scenes now have the same `World::clear()` rebuild-boundary
  proof without adding new scene definitions: AVBD self-contact friction, VBD
  Chebyshev self-contact, and AVBD ground-friction storage bake, step without
  World-base allocator growth or ECS capacity changes, clear to zero registry
  capacity, and rebuild with the same storage shape.
- The existing variational loop-closure chain now has a matching clear/rebuild
  gate that asserts the baked `MultibodyVariationalScratch` tree, step,
  inverse-dynamics, linear-solve, projection, and Anderson storage dimensions,
  then rebuilds the same shape with identical registry capacities under the
  World allocator root.
- Existing boxed-LCP multibody contact scenes now have clear/rebuild coverage
  for semi-implicit multibody contact storage using stacked and multi-island
  fallback shapes. The gate bakes active contact rows, verifies same-shape
  steps do not grow the World base allocator or ECS capacities, clears the
  registry to zero capacity, and rebuilds with identical storage capacities.
- The existing kinematic IPC surface-CCD crossing scene now has a matching
  clear/rebuild gate for `KinematicBodyStepTrace` storage. It bakes the active
  kinematic trace path, verifies same-shape steps keep ECS capacities stable
  without World-base allocator growth, clears the registry to zero capacity,
  and rebuilds the same trace storage shape under the World free-list
  allocator.
- Existing rigid AVBD contact and fixed-joint scenes now cover the remaining
  rigid AVBD registry storage rebuild boundary. The gate proves
  `RigidAvbdContactConfig` and generated `AvbdRigidWorldPointJointConfig`
  storage can be baked, stepped without World-base allocator growth or ECS
  capacity changes, cleared to zero registry capacity, and rebuilt with the
  same storage capacities under the World free-list allocator.
- Replay recording and one-rigid-body replay restore now follow the World free
  allocator root for the replay controller, top-level frame buffer,
  `ReplayState::Frame` snapshot vectors, AVBD warm-start replay row snapshots
  captured by a World, rigid-body restore ordering, and transient-component
  cleanup scratch. Enabling replay recording for an empty World and for a
  one-rigid-body World, then restoring that rigid-body replay frame, now
  completes without global heap allocation. Dynamic `Eigen::VectorXd` and
  `std::string` payloads inside richer replay snapshots remain follow-up work.
- Persistent ignored-collision-pair storage now follows the same World free
  allocator root as the registry and differentiable-parameter storage. The
  public `setCollisionPairIgnored()` path no longer allocates the pair set node
  from the global heap, and `clearIgnoredCollisionPairs()` releases that storage
  back to the World allocator.
- The existing contact-heavy variational dual-state setup now has matching
  clear/rebuild coverage for `VariationalContactDualState` storage. The gate
  bakes six sliders with four persistent contact duals each, verifies
  same-shape steps keep dual vector capacity and ECS storage capacity stable
  without World-base allocator growth, clears the registry to zero capacity,
  and rebuilds the same dual-state shape. The dual vector payload now borrows
  the World free allocator when baked through `enterSimulationMode()` or first
  created by the variational World stage, pre-existing/default-constructed
  dual-state components are rebound to that allocator before sizing, and the
  existing binary state serialization path accepts allocator-aware trivial
  vectors.
- The sibling `VariationalContact` persistent contact-point configuration now
  uses allocator-aware point-index and local-position vectors. Public
  `Multibody::setGroundContact()`/`addGroundContactPoint()` construction uses
  the World free allocator, and the variational World stage rebinds
  loaded/pre-existing contact configs before building baked contact scratch.
- `MultibodyVariationalState` two-step history now uses allocator-aware
  transform and momentum vectors. Bake-time and lazy World-stage creation use
  the World free allocator, loaded/pre-existing state is rebound without losing
  history, and binary state serialization now handles allocator-aware SE(3)
  transform and 6D momentum lists.
- Finite-stiffness AVBD point-joint compliant-loop scratch now follows the same
  variational multibody route. The private World-stage component constructs its
  compliant constraint list, axis-row vectors, row-descriptor staging lists, and
  scalar-row inventories from the World free allocator; `enterSimulationMode()`
  pre-sizes and synchronizes the row inventories for baked point-joint shapes;
  and the compliant contact hook reads that baked scratch directly instead of
  copying constraints into a default-heap vector.
- Pure compliant variational ground contact now bypasses
  `VariationalGroundContactSolver` and its default-heap dual vector entirely.
  Baked `MultibodyVariationalScratch` still owns the normalized ground-contact
  points plus contact-evaluation force/forcing vectors, while the
  augmented-Lagrangian path keeps solver-owned dual scratch only for positive
  `dualUpdateCadence` configurations. Existing compliant-contact World gates now
  assert the no-solver pure-compliant path, and the contact-heavy dual-state gate
  still asserts solver dual scratch for AL contact.
- `MultibodyVariationalTreeScratch` is now allocator-aware. The World stage sets
  it to the World free allocator before bake/step tree construction, so the
  pimpl object, link vector, per-link child lists, and link-index map belong to
  the same hierarchy as the surrounding variational scratch; one-shot helpers
  keep the default allocator fallback. The existing loop-closure clear/rebuild
  gate now asserts the baked scratch uses the World allocator.
- The semi-implicit one-slider multibody path now has the same clear/rebuild
  proof for baked private multibody dynamics storage. The gate covers the
  all-storage capacity map created by `reserveMultibodyDynamicsRegistryStorage`
  for the active `MultibodyDynamicsScratch`/`PendingMultibodyVelocity` path,
  then verifies same-shape steps do not grow World-base allocator counts or ECS
  capacities, clears to zero registry capacity, and rebuilds the same storage
  shape.
- The semi-implicit `MultibodyDynamicsScratch` follow-up now constructs its
  DART-owned dynamics-tree, link-index, RNEA, link-contact, constrained-DOF,
  constrained-target, contact-row, and body-Jacobian containers from the World
  free allocator when the component is bake-created or first needed by a World
  stage. Its external-force body-Jacobian branch now fills the baked
  body-Jacobian scratch instead of constructing a default-allocated local
  Jacobian vector during `world.step()`, and a focused semi-implicit
  external-force multibody gate covers both World-base no-growth and
  global-heap no-allocation after bake. The remaining heap allocation was the
  unconditional contact query in the split semi-implicit contact/unified stages
  for a world with no collision shapes, not the body-Jacobian multiply itself.
  Eigen-owned matrices/vectors remain tracked as the solver-private storage
  limitation called out below.
- `MemoryAllocatorDebugger` now records the requested alignment alongside live
  byte counts for aligned allocations and rejects mismatched aligned
  deallocations without forwarding them to the wrapped allocator, closing one
  remaining debug-accounting misuse gap. Its destructor also releases any
  still-tracked leaked allocations with the recorded size/alignment after
  reporting the leak.
- `docs/design/hierarchical_allocator.md` now reflects the implemented
  experimental DART 7 `World` hierarchy instead of the original proposal: the
  durable design note describes `WorldOptions`, the World-owned
  `MemoryManager`, allocator lifetime roles, registry bake/rebuild boundaries,
  and the direct evidence expected before making broader zero-allocation
  claims.
- The opaque `WorldStorage` object, private built-in step-pipeline cache,
  built-in stage-owned scratch/cache objects, lazy collision query cache, and
  optional replay controller object use the same World free-list allocator as
  the EnTT registry and differentiable-parameter storage. The focused
  `WorldPersistentStorageUsesWorldFreeAllocator` test verifies initial
  construction, built-in stage scratch construction, lazy collision-cache
  construction, lazy replay-controller construction, and `World::clear()`
  rebuilds keep persistent World state under the World memory hierarchy while
  dropping cached collision query state at the rebuild boundary. Replay frame
  payload vectors and nested stage scratch payload vectors remain governed by
  the existing same-shape no-growth/no-heap gates, not by this allocator-root
  ownership check.
- The first nested stage-scratch payload route covers
  `RigidBodyVelocityStage` force-batch vectors. When that stage borrows the
  World `MemoryManager`, its entity, force, and torque reserve/growth traffic
  uses the World free allocator; a focused heap-counter test verifies first
  `prepare()` does not allocate from the global heap. Other nested stage
  scratch payloads remain evidence-first follow-up work.
- The legacy graph-backed `RigidBodyIntegrationStage` can now borrow a
  `MemoryManager` as well. Its stage-owned rigid-body entity and dependency-node
  scratch vectors reserve from the provided free allocator, and its transient
  `ComputeGraph` uses the same allocator root for owned graph storage. A
  focused parent/child custom-stage test verifies the persistent scratch vector
  reserves use and release the provided free allocator.
- The batched SoA `BatchedRigidBodyIntegrationStage` can now borrow a
  `MemoryManager` for custom stage use. Its force-batch entity/force/torque
  vectors and parent-before-child frame-order/visit-state vectors reserve from
  the provided free allocator while the existing same-shape heap guard still
  covers the prewarmed frame-coupled path.
- `WorldStepPipeline` overflow stage-pointer storage can now borrow a provided
  allocator. The built-in World pipeline cache uses the World free allocator
  for that spillover path, and a focused custom-pipeline test verifies inline
  stages stay allocation-free while the first overflow reserve is charged to
  the provided allocator and released with the pipeline.
- The next nested route covers `RigidBodyContactStage`'s sequential-impulse
  constraint vector. A focused compact contact prepare verifies the vector's
  first reserve increases the World free-list allocation count. The AVBD
  contact scratch bundle is covered by the follow-up bullets below.
- The AVBD contact scratch follow-up routes that broader bundle's stage-owned
  private contact snapshot vectors, row-counter scratch, solve scratch vectors,
  warm-start inventories, and point-joint input vector through the borrowed
  allocator. A focused fixed-joint prepare verifies those first reserves
  increase an isolated provided free-list allocation count and release when the
  custom stage is destroyed.
- The large-row AVBD contact follow-up now routes generated scalar-row
  descriptors, motor active-row pointer lists, and distance-spring active-row
  pointer lists through allocator-backed reusable scratch instead of local
  default-heap vectors. `RigidBodyContactStage::AvbdScratch` also constructs
  distance-spring row inventory with the World allocator, and thresholded
  point-joint fracture-index result storage borrows the solve scratch allocator.
  Focused large-row/fracture builder tests verify the provided scratch allocator
  is used, and the existing rigid AVBD plus baked World no-heap gates still pass
  without adding new production scenes.
- The rigid IPC follow-up routes the stage's top-level runtime-body,
  solver-body, surface, dynamics-term, projected-Newton result,
  kinematic-trace, writeback-order, and resting-contact scratch vectors through
  the borrowed World free allocator. A focused IPC prepare test verifies those
  top-level and nested surface mesh vector reserves increase the provided
  free-list allocation count and release when the custom stage is destroyed.
  The projected-Newton solve scratch now also has allocator-aware construction
  for its surface work vectors, and the stage passes the same allocator into
  that nested solver scratch; a focused detail-solver test verifies those
  reserves use and release the provided free allocator. The same detail-solver
  path now constructs projected-Newton result assembly body-offset, active
  constraint, and active friction-constraint vectors with that allocator,
  preserves the destination allocator across repeated result assignments, and
  routes solve-internal barrier-assembly plus line-search surface-pair,
  sweep-item, candidate-pair, triplet, articulation equality-row, and equality
  change-of-variable index scratch
  through the same borrowed allocator. After
  the post-#2956 main merge added rigid/deformable mixed-domain candidate
  diagnostics, the same stage scratch now reuses allocator-backed BDF2 history,
  articulation-input, mixed-domain surface payload, candidate, edge, and AABB
  scratch; the focused custom-stage prepare gate covers a mixed
  rigid/deformable surface shape with zero global-heap allocation.
- The deformable stage scratch follow-up routes the stage-owned static-ground
  barrier, sphere/box/capsule obstacle, deformable surface-snapshot, static
  rigid surface-CCD snapshot, and moving rigid surface-CCD snapshot vectors
  through the borrowed World free allocator. Each snapshot's nested position,
  topology, contact-mask, and edge payload vectors now use that allocator as
  well. A focused custom-stage test covers those top-level and nested payload
  vectors against an isolated provided free allocator and verifies release when
  the stage is destroyed.
- Deformable self-contact and inter-body surface-contact candidate storage now
  has allocator-aware `ContactCandidateSet` and sweep scratch constructors.
  The World-stage `DeformableContactSolverScratch` and `DeformableVbdScratch`
  components build those candidate/sweep buffers from the World free allocator
  when the component is first created, while standalone reusable builders keep
  default construction for one-shot callers. The same
  `DeformableContactSolverScratch` route now covers the per-body surface
  topology/contact-mask storage and inter-body surface-CCD edge, sweep-item,
  and sweep-link buffers primed during bake. Focused contact-candidate tests
  verify provided allocator reserve/release, and existing World-base/global-heap
  baked-step guards still pass without adding a new production scene.
- `DeformableVbdScratch` now also constructs AVBD scalar-row inventories,
  descriptor metadata vectors, static-contact feature ID buffers, and friction
  warm-start lookup vectors from the World free allocator. The AVBD solve row
  arrays now use the same allocator-backed scratch after the mass-spring,
  half-space contact, attachment, self-contact friction, and tet finite-stiffness
  row kernels accepted allocator-aware vector contracts. The follow-up branch
  also routes the static contact-plane buffer and AVBD attachment fixed-mask
  scratch through the World allocator by
  narrowing the mixed deformable block-descent contact-plane contract to a
  read-only span and making the AVBD mass-spring row fixed-mask argument
  allocator-agnostic. Chebyshev history scratch for the mixed VBD driver now
  follows the same route: the driver preserves caller-provided scratch
  allocators while standalone one-shot callers keep default local storage. The
  spring/tet topology element lists now use the same allocator-backed
  `DeformableVbdScratch` route after narrowing read-only VBD topology inputs to
  spans. The cached VBD coloring plus spring, tetrahedron, and self-contact
  incident-adjacency builders now preserve caller-provided allocators for their
  nested vectors, so World-owned VBD topology scratch no longer falls back to
  default heap storage for those cached structures.
- `DeformableSolverScratch` now constructs inertial targets, iterate, gradient,
  direction, candidate, previous-step, external-acceleration, and active
  fixed/Dirichlet/Neumann mask buffers from the World free allocator. The
  default projected-Newton and VBD/AVBD driver contracts accept those
  allocator-backed vectors through spans or allocator-agnostic vector templates,
  so baked World-stage steps no longer fall back to default heap storage for
  those per-body solver buffers.
- Default deformable projected-Newton assembly scratch now borrows that same
  World free allocator for sparse-pattern arrays, triplet assembly, PSD
  edge/tet/barrier block batches, and matrix-free block/diagonal storage. The
  FEM rest-shape cache and lagged friction normal/contact arrays now use the
  same allocator-backed scratch component. The iterative sparse solver path
  now reuses DART-owned residual, preconditioned-residual, direction,
  Hessian-direction, and inverse-diagonal vectors instead of Eigen IC-CG
  temporaries, closing the measured iterative FEM ground-friction heap gap.
  The existing deformable stage allocator test now expects projected-Newton
  vectors to reserve from the provided allocator, and the baked
  World-base/global-heap gates now include the iterative FEM ground-friction
  coverage in addition to the existing direct-sparse and matrix-free scenes.
- The kinematics cache follow-up routes `WorldKinematicsGraph`'s entity-node
  lookup vector through the World free allocator when the graph is constructed
  by the built-in kinematics stage. A focused stack-constructed graph test
  verifies that entity-node cache storage increases the World free-list live
  allocation count and releases it when the graph is destroyed.
- The `ComputeGraph` follow-up adds allocator-aware graph construction and
  routes owned `ComputeNode` objects, the node-name lookup table, the
  dependency-edge vector, and the topological-order cache through the supplied
  allocator. Read-only edge/order accessors now return span views so those
  allocator-backed containers stay private while existing range-iteration call
  sites remain source-compatible. A focused compute-graph test verifies node,
  lookup, edge, and order storage use the provided World free allocator and
  release it on graph destruction. The current continuation extends that
  allocator contract to traversal scratch: cycle detection, topological-order
  rebuild, validation, and no-hazard resource scans no longer allocate from the
  global heap for allocator-aware graphs.
- The unified constraint stage now constructs its private multibody entity,
  link-contact bucket, required-block marker, staged-contact, dynamics-scratch
  pointer, and staged-velocity vectors with the borrowed World free allocator.
  The follow-up also routes the stage-owned rigid contact problem, unified row
  owner/rigid-constraint/multibody-block containers, nested multibody block row
  storage, and boxed-LCP solve scratch traversal/fallback/tangent vectors
  through the same allocator root.
  A focused stacked boxed-LCP prepare test verifies those stage-owned reserves
  use and release a provided free-list allocator.
- The pure rigid `ContactSolverMethod::BoxedLcp` branch now keeps reusable
  stage-owned scratch for contact normals, body-column lookup, dense Delassus
  work matrices/vectors, and Dantzig solver buffers. `RigidBodyContactStage`
  prewarms that scratch during `prepare()`, and the baked rigid sphere-ground
  boxed-LCP gate now covers both World-base no-growth and global-heap
  no-allocation behavior.
- The rigid contact stage now skips contact-query prepare/execute work when the
  only rigid/link collision-geometry components are empty. A baked global-heap
  regression covers that loaded/pre-existing ECS-storage shape without changing
  the existing shape-backed rigid, link, AVBD, and boxed-LCP contact gates.
- The dynamic rigid IPC dynamics-only path bypasses the full projected-Newton
  assembly for the exact single supported dynamic body with no possible contact
  or articulation pairs. The active rigid IPC mesh-barrier path also executes
  the one-node projected-Newton solve directly instead of rebuilding an
  internal `ComputeGraph` each step, and `prepare()` prewarms reusable
  projected-Newton step/result row storage from the current surface topology.
  The latest stack/articulation slice makes the projected-Newton solve scratch
  own persistent assembly/line-search buffers from the World allocator, and the
  focused baked gate now covers the contact-free IPC update, active
  static/dynamic mesh barrier, fixed/revolute articulated constraints, and a
  two-box stack with zero global heap allocation after bake. The follow-up gate
  also covers a mixed rigid/deformable surface-obstacle barrier solve through
  both baked World-base no-growth and global-heap no-allocation guards. The
  current continuation adds a focused World-base no-growth gate with the same
  dynamic IPC scene matrix as the global-heap gate: solve graph, active
  barrier, fixed/revolute joints, two-box stack, mixed rigid/deformable surface
  obstacle, kinematic conveyor, and kinematic turntable. The three existing IPC
  no-growth rows moved out of the monolithic reserved-ECS no-growth test so the
  full dynamic rigid IPC allocation matrix is owned by one focused test.
  Broader rigid IPC contact shapes remain evidence-first follow-up work.

Remaining Phase 4/5 follow-up items for the next PR:

- Do not add more production scenes or scratch-reuse work to PR #2956; continue
  any remaining no-growth and scratch work on a new follow-up branch.
- Continue projected-Newton deformable scratch reuse only where profiling or a
  no-growth gate exposes a real allocation path, especially solver-private
  storage that still cannot borrow the World allocator directly
  (`Eigen::SparseMatrix`/`VectorXd` internals). The iterative sparse FEM
  ground-friction path is now covered; remaining work should target newly
  measured Eigen/storage allocations or differently shaped frictional
  self-contact, static-obstacle, or inter-body CCD mixes not represented by the
  current gates.
- Continue broader rigid IPC projected-Newton scratch reuse only from measured
  failing shapes beyond the current contact-free and active two-mesh-barrier
  gates, such as larger mesh contact sets or articulated contact-stack shapes
  that expose new per-step allocator growth. The dynamic rigid IPC two-box
  stack, mixed rigid/deformable surface-obstacle, kinematic-conveyor contact,
  and kinematic-turntable contact shapes are now covered by baked
  no-growth/no-heap gates, so the next rigid IPC expansion should start from a
  newly measured failing shape rather than reusing those cases as open work.
- Add any remaining default-solver deformable storage/no-heap gates for
  solver-private paths not exercised by the current direct-sparse,
  matrix-free, FEM, obstacle, surface-CCD, and compact/production
  clear/rebuild mixed/contact-family scenes.
- Expand production contact-set coverage only for newly exposed boxed-LCP or
  unified-assembly shapes that are not covered by the current stacked,
  multi-island, mixed-stress, and contact-family gates.
- Continue production `WorldRegistry` bake/build sizing guidance beyond the
  current compliant-contact allocator-aware config/dual-state/history
  clear/rebuild gate and the newly allocator-backed point-joint loop-scratch
  bake route, especially for differently shaped solver-owned ECS storages and
  rebuild boundaries.
- Re-run allocator comparative evidence when allocator, STL, or frame policy
  changes; keep the current foonathan/memory and standard-baseline evidence
  green instead of adding allocator-policy work to this PR.

## Immediate Next Steps

1. Continue promoting the benchmark-only EnTT storage policies toward
   production `WorldRegistry` bake/build guidance and integration: free-list
   backed world-lifetime storage for reserved no-growth arrays, and resettable
   rebuild-lifetime frame storage for bake/growth. Production wiring now resets
   registry storage on `World::clear()` rebuild boundaries, and the follow-up
   branch adds compliant-contact dual-state, compact, production, mixed and
   complementary contact-family default-deformable, AVBD/VBD, boxed-LCP
   multibody contact, semi-implicit multibody dynamics, kinematic IPC
   surface-CCD trace, rigid AVBD contact/joint, and variational loop-closure
   clear/rebuild no-growth gates for baked solver scratch plus baked
   variational Anderson, step-state, and
   linear-solve/inverse-dynamics/projection work storage. Broader bake/build
   sizing guidance for other solver-owned ECS storage shapes remains open; it
   must not use the per-step frame scratch allocator that resets inside
   `World::step()`.
2. Continue extending allocator correctness tests for remaining
   leak/debug-accounting gaps and production workload cases after the new
   count/size overflow, over-alignment, aligned debug-deallocation,
   debug-leak release, reuse, diagnostics, and bounded-failure coverage for
   `MemoryAllocator`, `MemoryAllocatorDebugger`, `FixedPoolAllocator`,
   `PoolAllocator`, `FreeListAllocator`, `FrameAllocator`,
   `FrameStlAllocator`, `StlAllocator`, `MemoryManager`, and experimental
   `World` allocator-root isolation.
3. Keep the strict allocator comparative checker green as allocator policy
   changes land, and extend it to any remaining foonathan/memory baselines that
   map to HMM allocator roles. The current foonathan-plus-standard-plus-EnTT
   matrix covers pools, stack/frame, static/temporary/iteration, raw
   heap/malloc/new, aligned/fallback/segregator adapters, tracked/deeply tracked
   wrappers, and EnTT no-growth/build rows; a CPU-affined full run plus focused
   replacement rows passed all 94 foonathan and standard comparisons on
   2026-06-06. After the latest frame/STL allocator and steady-state benchmark
   policy changes, a 2026-06-07 foonathan-only run plus focused strict-CV row
   replacements passed all 47 foonathan comparisons in
   `.benchmark_results/allocator_comparative_current_foonathan_entt_cpu16_merged_check.json`.
   After the latest STL allocator hook/default-adapter change, the merged
   foonathan broad-plus-EnTT input
   `.benchmark_results/allocator_foonathan_broad_entt_current_check.json`
   passes all 47 foonathan comparisons with either non-noisy rows or
   confidence-separated high-CV rows.
   Later random-interleaved EnTT diagnostics on the same branch found that the
   pool-backed no-growth row could miss foonathan/memory at 256/512 entities,
   while frame-backed and default-backed probes did not robustly close the gap.
   A follow-up switched the no-growth row to free-list-backed world-lifetime
   storage, matching reserved variable-size registry arrays, and passed all 12
   EnTT comparisons in
   `.benchmark_results/allocator_entt_freelist_nogrowth_frame_build_current_merged_check.json`.
   A later 2026-06-07/08 probe kept `StlAllocator` storage at natural alignment
   for non-overaligned values and added cache-friendly default `PoolAllocator`
   strides for medium power-of-two slots. This is scoped to allocator
   contracts: STL storage only requests natural alignment, while default pool
   requests carry no over-alignment promise and can use non-power-of-two size
   classes to reduce cache-set conflicts. The same continuation moved the EnTT
   build/growth DART row back to the resettable frame-backed bake arena; the
   CPU-pinned build/growth probe in
   `.benchmark_results/allocator_entt_build_frame_bake_current_cpuauto_probe.json`
   beat foonathan/memory clearly and beat the standard registry by median, but
   the 256/512 standard rows were not confidence-separated because the standard
   rows were slightly above the strict CV gate. Earlier no-growth probes,
   including
   `.benchmark_results/allocator_entt_nogrowth_pool_stride_current_cpu12_probe.json`
   and
   `.benchmark_results/allocator_entt_nogrowth_pool_stride_current_cpu12_warm_probe.json`,
   were rejected as noisy and are superseded by the free-list storage result.
   The default `PoolAllocator` heap-index lookup table then fixed the remaining
   mixed-size `BM_MultiPool` and `BM_Realistic` foonathan misses. The current
   merged broad-plus-focused result
   `.benchmark_results/allocator_comparative_lookup_table_mixed_entt_merged_check.json`
   passes all 94 foonathan/memory and standard comparisons. Keep this combined
   gate green after allocator or benchmark policy changes:

   A 2026-06-08 follow-up added runner-side CPU prewarm and cache-line coloring
   for large 64-byte-aligned `FreeListAllocator` storage allocations. Direct
   focused probes improved the weak EnTT no-growth 512 row; the current
   persistent policy keeps free-list registry storage on eight 64-byte colors.
   Later frame-bake experiments showed that coloring ordinary one-shot
   `FrameStlAllocator` scalar arrays can move wins between foonathan and
   standard rows without a stable theoretical advantage. The retained frame
   policy keeps scalar arrays on the dense 32-byte frame fast path and reserves
   cache-line alignment for cache-line-sized or over-aligned value pages. The
   best current focused checker evidence for this retained policy is the merged
   CPU12 run in
   `.benchmark_results/allocator_entt_frame_dense_retained_cpu12_reps9_merged.json`:
   all 12 EnTT no-growth/build comparisons pass against foonathan/memory and
   standard baselines, and the no-growth DART rows report zero post-prewarm
   allocator calls. The base run had two noisy baseline rows, so
   `.benchmark_results/allocator_entt_frame_dense_retained_replacements_cpu12_reps9.json`
   replaces only `BM_EnttRegistryBuild_Foonathan/512` and
   `BM_EnttRegistryBuild_Std/256` before the strict checker is re-run over the
   merged artifact.
   Follow-up free-list color stride experiments are intentionally discarded:
   they improved selected 256/512 rows but failed different foonathan or
   standard rows across CPUs, which is cache-layout overfitting rather than
   evidence for a general allocator policy.

   ```bash
   pixi run bm-allocator-comparative-check \
     --include-entt-registry --baseline foonathan --baseline std --verbose \
     --cpu-affinity auto
   ```

4. Repeat the focused EnTT allocator benchmark when registry allocation policy
   changes, then promote the cached-storage policy into production
   `WorldRegistry` bake/build guidance only after the production integration
   has matching no-growth tests. Keep using the registry-only checker for
   focused allocator-policy loops:

   ```bash
   pixi run bm-allocator-comparative-check --only-entt-registry \
     --baseline foonathan --baseline std
   ```

   The current benchmark policy uses cached component storage handles and the
   free-list-backed `StlAllocator` role used by production `WorldRegistry`
   storage for persistent no-growth churn. Build/growth is a separate one-shot
   storage-construction role: DART uses `StlAllocator` over a resettable
   `FrameAllocator` bake arena, and foonathan/memory uses `memory_stack` with
   marker/unwind bulk lifetime. This is benchmark evidence; production
   integration still needs matching no-growth tests and lifetime diagnostics for
   broader rebuild paths.

5. Extend bake-time registry/component storage reservation and no-growth
   allocation tests to remaining solver scratch step paths. The rigid-body,
   non-cross articulated, same-DOF sequential cross-articulated, and boxed-LCP
   mixed/different-DOF, stacked, coupled multi-row, larger stacked, extended,
   dense production, extra-dense production, and stress production stacked
   cross-articulated guards plus disconnected multi-island mixed
   rigid/articulated contact guards, including a mixed stress stack plus
   multi-island scene with 60+ initial contacts, now cover World base-allocator
   growth and first baked-step global heap allocation by priming unified
   constraint scratch at `enterSimulationMode()`. The current
   deformable friction guard
   scales the same topology-reserved candidate/friction scratch, including
   swept-AABB line-search CCD capacity, from patch, 5x5, 7x7, and 9x9 grids to
   active 11x11, 13x13, and 15x15 square grids, a matrix-free 17x17 square grid,
   plus direct-sparse and matrix-free 13x19, 9x13, 7x17, and 17x7 non-square
   two-layer grids.
   Late-active 11x11 square and 9x13 rectangular
   direct-sparse and matrix-free grids now cover dynamic contact-pattern cases
   without World-base or global-heap growth, and a mixed late-active
   production scene now covers independent direct-sparse square and matrix-free
   rectangular self-contact grids that activate during the same baked loop. A
   mixed dense production scene also covers a notched direct-sparse 13x17 grid
   and matrix-free 13x19 dense rectangular grid sharing one baked World memory
   root; a notched, jittered matrix-free 13x17 irregular grid now covers the CG
   scratch path on non-grid topology; compact and production mixed
   default-solver storage scenes now combine direct-sparse, matrix-free, and
   FEM ground-friction deformables under the same root; a mixed production FEM
   scene now combines direct and matrix-free 4x4x4-node ground-friction blocks
   under one baked root; a production rectangular inter-body deformable
   surface-CCD crossing now exercises
   inter-body sweep/candidate scratch beyond the tiny two-triangle crossing;
   and a production static-obstacle friction patch now covers shared
   sphere/box/capsule normal-force, normal-direction, and Hessian scratch under
   sparse and matrix-free no-growth guards. A mixed production scene now
   combines sparse static-obstacle friction and matrix-free self-contact bodies
   under one baked World root; the complementary matrix-free static-obstacle
   plus direct irregular self-contact scene covers the opposite solver pairing.
   A mixed default contact-family production scene now combines direct static
   obstacle friction, matrix-free self-contact friction, and inter-body surface
   CCD under the same baked World root, with a complementary contact-family
   scene covering matrix-free static-obstacle friction, direct irregular
   self-contact, and production inter-body surface CCD in one root.
   The AVBD self-contact row guard now also covers 5x9 and 9x13 rectangular
   grid row workloads with replay-backed activity assertions.
   Continue broadening boxed-LCP unified problem assembly only for newly
   exposed contact shapes, and keep moving any newly exposed deformable/contact
   candidate buffers to backed storage before making the full zero-allocation
   claim.
6. Start replacing per-step `std::vector`/`Eigen` temporaries in hot stages with
   world-frame or world-pool backed storage only after the allocator evidence
   gate proves the DART allocator path is better for that workload. The
   non-owning `WorldStepPipeline` stage list is already inline; focus next on
   solver-owned scratch and contact/deformable candidate buffers.
