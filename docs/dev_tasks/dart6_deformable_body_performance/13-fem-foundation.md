# M2.1 — volumetric FEM foundation

First build increment of the Kim/Pollard half of the full-parity program
(`10-full-parity-execution-plan.md` §7/§11, after the M2.0 seam validation in
`11-fem-integration-seam.md`). Delivers the ABI-safe skeleton of a volumetric
FEM deformable subsystem that lives in a DART 6 world, verified by exact
numerical oracles. **No elasticity yet** — that is the next increment.

Branch `wp-db-fem-foundation`, based on `release-6.20`. Independent of #3382:
the integration seam and the SDF soft-shape fix (#3399) are both on
`release-6.20` already.

## Why this increment

DART has no finite-element, tetrahedral, volumetric, or reduced-deformation code
of any kind. Five parity rows (Fatman, starfish and its obstacle-escape row,
fish, worm at paper scale, CPU scaling) plus the whole Kim/Pollard lane sit
behind this subsystem, so it is the program's long pole. M2.0 proved the
integration seam; this increment builds the data model and the per-step pipeline
on top of it so the elastic-force work can be added to a working, tested body.

## Compatibility posture: internal-only staging

New public API added to a release branch is ABI-frozen once 6.20 ships, which is
a bad trade for a subsystem still under design. DART has a precedent for
avoiding that: the native-collision port (#3281, `135cc8f2076`, 2026-07-05)
staged **17** headers so that sources compiled into `libdart` while nothing was
installed and nothing joined a generated aggregate header.

Be clear about what that precedent actually shows. It was a **transitional**
state, not a settled posture: #3352 (`f343528708e`, 2026-07-08) added the
install rules **three days later**, and renamed all 17 headers in the same
commit. Native collision headers are installed today. So internal-only staging
buys design freedom now and carries a known exit cost later — expect to settle
naming and file layout before installing, or to pay a rename. That trade is
still the right one here, because the FEM data model will keep moving through
M2.2-M2.5, but it should be taken with the exit cost in view rather than
described as permanent.

M2.1 follows that posture:

- `dart/dynamics/fem/CMakeLists.txt` registers the sources and **installs no
  header**, so the release branch's installed API and ABI surface is unchanged.
- The headers are not added to the generated `dynamics.hpp` aggregate.
- They are also excluded from Doxygen (`CMakeLists.txt`, `DOXYGEN_EXCLUDE`);
  otherwise the published API reference would document classes that consumers
  cannot include. Drop that entry when the headers are installed.
- `DeformableBody` uses a pimpl (`std::unique_ptr<detail::DeformableBodyImpl>`,
  defined entirely in the `.cpp`) so the class is already installable without an
  ABI break when the design settles.
- No existing type is modified: no new member, virtual, or layout change to
  `BodyNode`, `SoftBodyNode`, `Skeleton`, `World`, or `ConstraintSolver`, and in
  particular none of `SoftBodyNode`'s two ABI intrusions (the
  `Skeleton::mSoftBodyNodes` registry member and `BodyNode::asSoftBodyNode()`).
- Attachment uses public API only: `DeformableBody::attachTo(WorldPtr)` calls
  `world->getConstraintSolver()->addConstraint(...)`. There is deliberately no
  `World::addDeformableBody()`, which would change `World`'s layout.

Note that the headers are not installed but their symbols **are** exported —
DART builds without hidden visibility. Nothing existing changes ABI; the honest
statement is "no existing ABI is altered", not "nothing is added".

`dart/dynamics/CMakeLists.txt` globs only `*.hpp`, `*.cpp` and `detail/*`,
non-recursively, so the new subdirectory needed both its own `CMakeLists.txt`
and an `add_subdirectory(fem)` line. Every new file must be listed there
explicitly or it silently will not build.

## Integration seam, and the early-out it inherits

`ConstraintSolver::updateConstraints()` calls `manualConstraint->update()`
unconditionally (`ConstraintSolver.cpp:1009`) before the `isActive()` LCP gate
(`:1011`). `DeformableBodyUpdateConstraint` therefore advances the body every
solve while reporting `isActive() == false`, so it never enters the LCP.
`prepareForSimulation()` calls `updateConstraints(false)` (`:857`), so manual
constraints are not updated during preparation and there is no double step on
simulation-mode entry.

**The seam borrows someone else's call site, and therefore inherits every
early-out on the way to it.** An earlier draft of this document claimed that
`World::step()`'s two `solve()` call sites are "unconditional within each". That
was wrong, and independent review demonstrated the consequence: `World::step()`
has a **third** path (`World.cpp:1403-1410`) that returns after advancing time
and frame without calling `solve()` at all, taken when automatic deactivation
decides the whole rigid scene is resting and no contacts remain. Deactivation is
**enabled by default** (`DeactivationOptions.hpp:58`). A deformable body in such
a world froze silently — position and velocity bit-identical for thousands of
steps — while simulated time kept advancing, which is indistinguishable from
being detached. The scene that triggers it is a box settling on a floor.

The body is not a `Skeleton`, never enters `mSkeletons`, and so cannot take part
in the resting decision or keep the world awake. `attachTo()` therefore disables
the world's deactivation, which keeps every step solving. The cost is scoped:
worlds without a deformable body are untouched, and a world that has one is by
definition not entirely at rest. `detach()` deliberately does not restore the
setting, because another deformable body may still be attached and relying on
it. This is a real limitation of hanging per-step work off the constraint
solver; a first-class step participant would be the better long-term answer, and
that requires revisiting the no-`World`-change posture.

Zero rigid-body overhead still holds for worlds with no deformable body: no
constraint is registered, so the manual-constraint loop iterates zero times.

## Known trade-offs when a body is attached

- The world's all-resting deactivation fast path is disabled (above). Be precise
  about what that costs: the body never perturbs the constraint solve, so gate
  13's bit-identical rigid trajectory holds — but only for scenes where nothing
  would have slept. In a scene that *would* sleep, rigid bodies now keep
  integrating instead of freezing, so their trajectories differ from the same
  scene without a deformable body. Sleeping is itself an approximation and the
  continued integration is the more faithful answer, but it is a behavior
  difference, not only a performance one, and it is the price of hanging
  per-step work off the constraint solver.
- A non-empty manual-constraint list disables the parallel constrained-group
  solve (`ConstraintSolver.cpp:2776`). Results are unchanged either way, and the
  default thread count is one, so nothing is affected today — but this belongs
  in the M2.3 performance work.
- `World::clone()` does not copy manual constraints, so a cloned world does not
  carry the deformable body.

## What landed

- `dart/dynamics/fem/TetMesh.{hpp,cpp}` — tetrahedral mesh with cached rest
  quantities (per-tet rest volume and inverse rest shape matrix), plus a box
  generator.
- `dart/dynamics/fem/DeformableBody.{hpp,cpp}` — `Material`, the pimpl body
  (node state, lumped masses, embedded surface, world attach/detach), and the
  internal update constraint.
- `tests/integration/test_FemDeformableBody.cpp` — 14 gates, registered with
  `dart_add_test("integration" test_FemDeformableBody)`.

## Math

1. **Rest quantities** per tetrahedron `(x0,x1,x2,x3)`:
   `Dm = [x1-x0 | x2-x0 | x3-x0]`, `restVolume = det(Dm)/6`, and the cached
   `Dm⁻¹` that the elastic work will use for deformation gradients
   `F = Ds · Dm⁻¹`. A non-positive rest volume is **rejected loudly**; silently
   reordering user nodes would mask genuinely inverted input. A tetrahedron
   whose volume is vanishing relative to its edge lengths is rejected too: it
   passes the sign check while making `Dm⁻¹` numerically unusable, which is
   harmless under gravity alone and ruinous once `F` is formed.
2. **Box generator**: Kuhn decomposition, six tetrahedra per grid cell, one per
   ordering of the three axes, so cells tile exactly and faces stay conforming.
   Three of the six come out negatively oriented and are corrected by swapping
   the last two nodes, which flips orientation without changing the region
   covered. The decomposition was verified numerically before being written in
   C++, and independent review confirmed conformity by counting shared faces:
   every interior face is shared by exactly two tetrahedra for uniform and
   non-uniform divisions alike.
3. **Mass lumping**: each tetrahedron contributes `density · restVolume / 4` to
   each of its four nodes, so summed node masses reproduce
   `density · totalRestVolume` exactly. A non-positive density is rejected,
   since it would defeat that guarantee from the other side and leave every node
   massless and the body permanently inert while still reporting itself healthy.
4. **Embedded surface**: each surface vertex is bound to its containing
   tetrahedron with barycentric weights solved through the cached `Dm⁻¹`. A
   vertex outside every tetrahedron binds to the one it violates least in
   barycentric terms. That measure is normalized per tetrahedron rather than
   being a distance, so for a vertex well outside the mesh the chosen
   tetrahedron is not necessarily the nearest and the extrapolation error is
   unbounded.
5. **Integration**: semi-implicit Euler in the same order the world integrates
   rigid bodies, with **mass-proportional** damping applied implicitly:
   `v <- (v + g·dt) / (1 + c·dt)`, then `x += v·dt`. Dividing the damping force
   by node mass instead would give every node its own decay factor, and lumped
   masses vary several-fold across an ordinary box mesh, so a uniformly
   translating body would damp faster at its corners than in its interior and
   tear itself apart with no elastic force present. Independent review measured
   that defect at 0.12 m of shape deviation on a 0.4x0.2x0.3 m box before the
   change. The mass-proportional implicit form preserves rigid translation
   exactly and is stable for any non-negative rate and time step.

## Acceptance gates

Fourteen gates assert against exact oracles rather than loose tolerances.

1. **Box tiling**: expected node and tetrahedron counts; every rest volume
   positive; every cached `Dm⁻¹` inverts its `Dm`; total rest volume equals the
   box volume.
2. **Invalid meshes rejected**: inverted, degenerate, out-of-range-index, and
   sliver tetrahedra all throw, while a merely thin element is accepted.
3. **Mass**: total and summed per-node lumped mass equal `density · volume`.
4. **Non-physical materials rejected**: zero or negative density, negative
   damping.
5. **Embedded surface, affine**: reproduces the input surface exactly at rest,
   and an affine motion of every node moves every embedded vertex by the same
   affine map.
6. **Embedded surface, containment** — this is the gate that constrains the
   search. Review demonstrated that gate 5 is invariant to *which* tetrahedron a
   vertex binds to: weights solved from any tetrahedron reproduce the vertex and
   stay affine covariant, and binding all 13 vertices to a tetrahedron
   containing only one of them still passed. Gate 6 asserts weights lie in
   `[0, 1]` for interior vertices, and applies a **non-affine** motion —
   displacing a single node — under which a vertex may move only by that node's
   own weight. That matters because M2.3 plans to replace the brute-force search
   with a spatial structure.
7. **Undamped free fall**: matches the closed-form semi-implicit Euler solution
   `v_n = n·g·dt`, `x_n = x₀ + g·dt²·n(n+1)/2`. A skipped or doubled step misses
   by nine orders of magnitude, so this pins the per-step contract.
8. **Runs in a world with no skeletons.** Recorded honestly: review showed this
   gate cannot fail, because `isAllRestingFastPathReady` returns false when
   there is no mobile skeleton, so the fast path is structurally unreachable
   here. It is kept as a smoke check, and gate 9 is what actually covers the
   fast path.
9. **Keeps integrating when the rigid scene would fall asleep** — a box falling
   onto a floor and settling. Asserts that attaching disabled deactivation and
   that the body still matches the closed-form solution through the transition.
   This is the regression gate for the blocker above; without the fix the body
   freezes completely.
10. **Damped decay**: `v_n = v₀·(1 + c·dt)⁻ⁿ` exactly, independent of node mass.
11. **Damping preserves shape and stays stable**: on a mesh whose node masses
    vary more than fourfold, with `c·dt = 0.5` (outside explicit stability), a
    uniformly moving body keeps every rest offset and one shared velocity.
12. **Determinism**: two identical runs give an identical component-weighted
    checksum, and that checksum differs from the rest state, so a body that
    never moved cannot pass.
13. **No rigid perturbation**: a rigid skeleton's 200-step trajectory is
    bit-identical with and without a deformable body attached.
14. **Attach/detach**: detach stops integration, re-attach resumes.

## Review evidence

Two independent role-separated reviews were run on the committed state, one on
physics and correctness and one on ABI and release posture. Both returned
ISSUES-FOUND, and between them found one blocker and four major issues that the
14-gate suite did not catch:

- the deactivation fast-path freeze (both reviewers, independently reproduced);
- mass-dependent damping deforming a uniformly translating body;
- the containment search having no effective test coverage;
- non-physical densities being accepted; and
- this document overstating the internal-only precedent.

All of the above are fixed or corrected here. The reviews confirmed the CMake
and ABI containment independently, one of them by staging a real `DESTDIR`
install and finding zero FEM files, and confirmed the box generator's conformity
by counting shared faces. Remaining review observations that are recorded rather
than fixed — the parallel-solve gating, `World::clone()`, and the `Fwd.hpp` /
`Export.hpp` / aggregate-header artifacts that installation will need — are
listed above and in the next-increment notes.

## Verification

```bash
pixi run lint
pixi run config
DART_DISABLE_COMPILER_CACHE=ON pixi run cmake --build build/default/cpp/Release \
  --target test_FemDeformableBody --parallel 8
./build/default/cpp/Release/tests/integration/test_FemDeformableBody
DART_DISABLE_COMPILER_CACHE=ON pixi run test
```

## Next increments

- **M2.2** elastic element forces (linear, then corotational/StVK), energy and
  stability gates, and the first deforming demo.
- **M2.3** reduced/modal coordinates and Kim/Pollard selective diagonalization,
  with the performance rows. The embedded-surface binding is a brute-force
  search over tetrahedra, fine for correctness but needing acceleration at paper
  scale (the Fatman row embeds 34,362 surface nodes in 8,619 elements); gate 6
  is what will guard that replacement.
- **M2.4** skeleton coupling: one-way prescribed drive first, then two-way. Note
  from M2.0 that two-way reaction cannot use a mid-solve `addExtForce` (external
  forces are cleared at the end of `World::step`); it must use LCP participation
  or a force applied before `step()`. Note also that the body performs its whole
  velocity-and-position update inside `solve()`, half a step out of phase with
  the skeletons, which is unobservable while decoupled and will matter here.
- **M2.5** collision, the four paper characters, demos, and the parity gates.
  `TetMesh` has no boundary-surface extraction yet, which collision will need.

Before the headers are installed, settle naming and add the `Fwd.hpp`,
`Export.hpp`, and `fem.hpp` aggregate that the native-collision precedent needed
at the same point, and run the suite under the project's ASan build.
