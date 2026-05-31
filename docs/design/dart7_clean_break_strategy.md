# DART 7 Clean-Break Strategy

Status: accepted planning direction for maintainer review; implementation is
gated by the release checklist below.

## Decision

DART 7 is the clean-break major release for the DART 6 to DART 7 transition.
DART 7 does not serve as a long-lived compatibility bridge for the DART 6 API.
`release-6.16` remains the compatibility lane for the established API and for
Gazebo / gz-physics users that need it.

The experimental simulation stack is the public API target for the clean break,
but promotion is parity-gated. DART 7 should not ship with a promoted
experimental world until the core robotics workflows below have direct
evidence. DART 8 is reserved for a later post-DART-7 major release, not as the
active cleanup point for DART 6 compatibility debt.

Main no longer treats gz-physics source compatibility as a required design
constraint for the DART 7 API. The main-branch Gazebo workflow is a migration
canary unless a change is explicitly scoped as Gazebo migration work.

## Rationale

Keeping DART 7 as a bridge and DART 8 as the cleanup release creates a long
dual-maintenance window. Main would need to carry the DART 6 API, the classic
Skeleton-backed simulation path, and the new experimental world while the
robotics simulation landscape is moving quickly.

Parking gz-physics support on `release-6.16` removes the main reason to keep the
classic DART 6 surface on main. The compatibility surface is broad: classic
`World` ownership, `Skeleton` / `BodyNode` / `Joint` concepts, deprecated
collision and constraint APIs, and package-config behavior used by pinned
gz-physics builds. A broad compatibility shim inside DART 7 would recreate the
same maintenance burden under a different name.

The clean break is still evidence-gated. The experimental world must cover
basic robotics simulation before promotion. Research algorithms such as IPC,
VBD, differentiable simulation, and GPU backends can continue as opt-in
features; they do not all block the DART 7 release unless the promoted public
API depends on them.

## Release Topology

- **DART 6.16.x**: maintained compatibility line for DART 6 API users and
  Gazebo / gz-physics. Backport critical bug fixes, build fixes, security
  fixes, and gz-required compatibility fixes. Do not backport normal DART 7
  features by default.
- **DART 7.0**: clean-break release from main. Promotes the new simulation API
  only after the parity gates in this document pass. Removes DART 6 public API
  shims, legacy dartpy modules, classic `World`, and gz-only compatibility
  surfaces from the DART 7 public contract.
- **DART 8**: future post-DART-7 major release. It is not the active cleanup
  target for the DART 6 to DART 7 transition.

## DART 7 Release Gates

DART 7 must not ship until these gates have direct evidence:

1. **Experimental-world model loading**: URDF, SDF, MJCF, and SKEL load through
   a world-centric API into the new simulation stack with tests for tree
   topology, DOF count, transforms, mass/inertia, center of mass, collision
   geometry, and Python entry points.
2. **Rigid-body dynamics parity**: shared scenes match the classic DART 6 path
   within documented tolerances for gravity, open-chain articulated dynamics,
   state integration, energy or trajectory drift, and control inputs.
3. **Contacts and constraints parity**: contact response, friction, joint
   limits, motors, mimic/coupler behavior, and loop closures have focused tests
   and migration examples for supported workflows.
4. **Serialization and replay parity**: world topology, state, model assets, and
   record/replay round-trip with bounded error for reproducible research and
   the standalone `dartsim` workflow.
5. **Stable public API promotion**: promoted C++ and Python APIs hide ECS
   storage, component types, solver registries, backend details, and
   implementation escape hatches. Doxygen/user docs, stubs, migration snippets,
   and `check-api-boundaries` evidence are present.
6. **World-parity suite**: a dedicated suite loads reference scenes, steps the
   classic and new paths while both exist, and asserts published tolerances.
   The suite becomes the release proof for deleting the classic path.
7. **Release packaging**: version metadata, changelog, CMake package exports,
   wheels, README quick starts, package smoke checks, and artifact checks agree
   on the DART 7 clean-break contract. DART 7 packages must not accidentally
   satisfy DART 6/gz-physics package ranges.
8. **DART 6.16 support policy**: the maintained Gazebo branch/version matrix,
   backport scope, and sunset date or sunset trigger are published before DART 7
   deletes the legacy surface from main.

Sensors and rendering parity are not DART 7 release blockers unless a promoted
public API depends on them. They remain post-release follow-up work behind their
own documented gates.

## Implementation Sequence

1. **Policy alignment**: update release roadmap, plan dashboard, north-star,
   API-boundary, CI, release-management, README, and changelog docs so DART 7 is
   the clean break and DART 6.16 is the compatibility line.
2. **Gazebo lane split**: keep required gz-physics validation on
   `release-6.16`; keep main's gz-physics workflow as a manual migration
   canary. Coordinate any branch-protection changes with maintainers before
   relying on the demotion.
3. **DART 6.16 support packet**: audit fixes on main against `release-6.16` and
   backport only compatibility-critical patches needed for the old API/Gazebo
   line.
4. **Parity implementation**: prioritize experimental-world model loading,
   rigid dynamics, contacts/constraints, serialization, and the parity suite
   ahead of additional research solver breadth.
5. **Promotion and removal**: port in-repo examples, tutorials, tests,
   benchmarks, and Python stubs to the promoted API; then remove the classic
   `World`, DART 6 C++ API shims, legacy dartpy compatibility modules, and
   gz-only compatibility surfaces from main.

## Compatibility-Shim Policy

Do not build a broad `dart6-compat` shim into main. A future shim is acceptable
only if it is separate from the DART 7 core, inventory-driven by a concrete
downstream migration, limited to the calls that downstream uses, and sunset
dated. The default compatibility answer for gz-physics is the maintained
`release-6.16` line.

## Open Decisions

- Final DART 6.16 Gazebo support window or sunset trigger.
- Whether yanked `dartpy` 7.0.0 artifacts can be reused safely, or whether the
  first published clean-break package should use a later patch/pre-release
  number.
- Exact parity tolerances and scene corpus for the world-parity suite.
- Whether any future gz-physics migration adapter is owned by DART or by
  downstream Gazebo work.
