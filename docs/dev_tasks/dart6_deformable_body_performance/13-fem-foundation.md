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

The plan flagged a real risk: new public API added to a release branch is
ABI-frozen once 6.20 ships, which is a bad trade for a subsystem still under
design. DART already has the right precedent — the native-collision port
(#3281) staged eighteen headers **internal-only**: sources compile into
`libdart` through `dart_add_core_headers`/`dart_add_core_sources`, but nothing
is installed and nothing is added to the generated aggregate header.

M2.1 follows that posture exactly:

- `dart/dynamics/fem/CMakeLists.txt` registers the sources and **does not
  install any header**, so the release branch's installed API and ABI surface is
  unchanged.
- The headers are not added to the generated `dynamics.hpp` aggregate.
- `DeformableBody` still uses a pimpl (`std::unique_ptr<detail::DeformableBodyImpl>`,
  defined entirely in the `.cpp`) so the class is already installable without an
  ABI break when the design settles.
- No existing type is modified: no new member, virtual, or layout change to
  `BodyNode`, `SoftBodyNode`, `Skeleton`, `World`, or `ConstraintSolver`, and in
  particular none of `SoftBodyNode`'s two ABI intrusions (the
  `Skeleton::mSoftBodyNodes` registry member and `BodyNode::asSoftBodyNode()`).
- Attachment uses public API only: `DeformableBody::attachTo(WorldPtr)` calls
  `world->getConstraintSolver()->addConstraint(...)`. There is deliberately no
  `World::addDeformableBody()`, which would change `World`'s layout.

Note that `dart/dynamics/CMakeLists.txt` globs only `*.hpp`, `*.cpp` and
`detail/*`, non-recursively, so the new subdirectory needed both its own
`CMakeLists.txt` and an `add_subdirectory(fem)` line. Every new file must be
listed explicitly there or it silently will not build.

## Integration seam (re-confirmed on release-6.20)

`ConstraintSolver::updateConstraints()` calls `manualConstraint->update()`
unconditionally (`ConstraintSolver.cpp:1009`) before the `isActive()` LCP gate
(`:1011`). `DeformableBodyUpdateConstraint` therefore advances the body every
solve while reporting `isActive() == false`, so it never enters the LCP.

Two further facts were verified on this base:

- `prepareForSimulation()` calls `updateConstraints(false)` (`:857`), so manual
  constraints are not updated during preparation — no double-step on
  simulation-mode entry.
- `World::step()` has two `solve()` call sites (`World.cpp:1327`, `:1501`) on
  mutually exclusive paths, and `solve()` is unconditional within each. Rather
  than rely on that reading, the once-per-step property is **proved** by an exact
  discrete oracle in the tests.

Zero rigid-body overhead follows structurally: a world with no deformable body
registers no constraint, so the manual-constraint loop iterates zero times.

## What landed

- `dart/dynamics/fem/TetMesh.{hpp,cpp}` — tetrahedral mesh with cached rest
  quantities (per-tet rest volume and inverse rest shape matrix), plus a box
  generator.
- `dart/dynamics/fem/DeformableBody.{hpp,cpp}` — `Material`, the pimpl body
  (node state, lumped masses, embedded surface, world attach/detach), and the
  internal update constraint.
- `tests/integration/test_FemDeformableBody.cpp` — nine gates, registered with
  `dart_add_test("integration" test_FemDeformableBody)`.

## Math

1. **Rest quantities** per tetrahedron `(x0,x1,x2,x3)`:
   `Dm = [x1-x0 | x2-x0 | x3-x0]`, `restVolume = det(Dm)/6`, and the cached
   `Dm⁻¹` that the elastic work will use for deformation gradients
   `F = Ds · Dm⁻¹`. A non-positive rest volume is **rejected loudly**; silently
   reordering user nodes would mask genuinely inverted input.
2. **Box generator**: Kuhn decomposition, six tetrahedra per grid cell, one per
   ordering of the three axes, so cells tile exactly and faces stay conforming.
   Three of the six come out negatively oriented and are corrected by swapping
   the last two nodes. The decomposition was verified numerically before being
   written in C++: volumes are all positive and sum to the box volume with zero
   error for uniform and non-uniform divisions alike.
3. **Mass lumping**: each tetrahedron contributes `density · restVolume / 4` to
   each of its four nodes, so summed node masses reproduce
   `density · totalRestVolume` exactly.
4. **Embedded surface**: each surface vertex is bound to its containing
   tetrahedron with barycentric weights solved through the cached `Dm⁻¹`; a
   vertex outside every tetrahedron binds to the closest one and extrapolates.
   Current position is `Σ bᵢ · xᵢ(t)`.
5. **Integration**: semi-implicit Euler in the same order the world integrates
   rigid bodies — `f = m·g − c·v`, then `v += (f/m)·dt`, then `x += v·dt`.

## Acceptance gates — exact oracles, not loose tolerances

All nine assert to 1e-12 or to bit equality.

1. **Box tiling**: expected node and tetrahedron counts; every rest volume
   positive; every cached `Dm⁻¹` actually inverts its `Dm`; total rest volume
   equals the box volume.
2. **Invalid meshes rejected**: inverted, degenerate (repeated node), and
   out-of-range-index tetrahedra all throw; the positively oriented control is
   accepted.
3. **Mass**: total and summed per-node lumped mass equal `density · volume`.
4. **Embedded surface**: reproduces the input surface exactly at rest, and
   because barycentric weights sum to one, an affine motion `A·x + b` applied to
   every node moves every embedded vertex by exactly the same affine map. That
   single check exercises the containment search, the weights, and the
   interpolation together.
5. **Undamped free fall — the once-per-step oracle.** Semi-implicit Euler from
   rest under constant gravity has the closed form `v_n = n·g·dt` and
   `x_n = x₀ + g·dt²·n(n+1)/2`. Matching it to 1e-12 proves the body advanced
   exactly once per world step; a skipped or doubled step misses by orders of
   magnitude. This replaces any assumption about `World::step`'s two `solve()`
   paths and needs no test-only counter API. The test also asserts the body
   stays undeformed, as it must without elastic forces.
6. **Runs in a world with no skeletons** — guards against an empty-world fast
   path skipping the solve.
7. **Damped decay**: with gravity off, `v_n = v₀·(1 − c·dt/m)ⁿ` exactly.
8. **Determinism**: two identical runs give an identical state checksum.
9. **No rigid perturbation**: a rigid skeleton's 200-step trajectory is
   **bit-identical** with and without a deformable body attached to the same
   world, and detach stops integration while re-attach resumes it.

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
  with the performance rows.
- **M2.4** skeleton coupling: one-way prescribed drive first, then two-way. Note
  from M2.0 that two-way reaction cannot use a mid-solve `addExtForce` (external
  forces are cleared at `World.cpp:1378`); it must use LCP participation or a
  force applied before `step()`.
- **M2.5** collision, the four paper characters, demos, and the parity gates.

The embedded-surface binding is currently a brute-force search over
tetrahedra, which is fine for correctness but will need acceleration at paper
scale (the Fatman row embeds 34,362 surface nodes in 8,619 elements). That is an
M2.3 performance concern, not a correctness gap.
