# Resume: Native Collision Default

## Last Session Summary

Work resumed on `feature/new_coll` to make the native collision detector the
default replacement for FCL, Bullet, and ODE. This session moved core default
paths to `dart`, fixed native feature and performance parity gaps, preserved
gz-physics compatibility through the legacy `DARTCollisionDetector` facade,
made FCL/Bullet/ODE optional for native default builds, and reran full
validation successfully.

## Current Branch

`feature/new_coll` - local branch tracking `origin/feature/new_coll`.

## Immediate Next Step

Continue from `docs/dev_tasks/native_collision/README.md`: use the evidence
when drafting the PR description, then remove the working dev-task folder in
the completion PR.

## Context That Would Be Lost

- `DartCollisionDetector::getStaticType()` returns `dart`; `experimental` is
  kept as a factory alias for compatibility.
- `ConstraintSolver`, `WorldConfig`, and SKEL loading now prefer `dart`.
- Native box-box face contacts now emit a contact patch when `maxNumContacts`
  allows it; this fixed the default solver expanded-manifold regression.
- `pixi run -e gazebo test-gz` passed after the legacy uppercase compatibility
  header kept gz-physics ray intersection behavior unsupported while lowercase
  `DartCollisionDetector` retained native raycast support.
- Benchmarks show native now winning comparative primitive, narrow-phase,
  supported distance, raycast, batch-raycast, mesh-heavy, and the recorded
  mixed-primitive scenario cases, including dense 1000 after cached snapshot
  reuse.
- FCL, Bullet, and ODE can be disabled for a core `dart` build. Focused
  native/default C++ tests and `dartpy` also build and pass in that
  configuration.
- The last audited native feature gap was `VoxelGridShape`; native now adapts
  occupied OctoMap leaves into a compound of box cells and has unit plus
  integration coverage.
- `pixi run test-all` passes after the native voxel-grid work and GUI headless
  fixes. The fresh full run passed lint, build, Release C++ tests,
  simulation-experimental tests, Python tests, and docs.
- A fresh gz-physics run passed 65/65 tests and the plugin link check.
- Tests and benchmark comments now use native collision naming except for the
  explicit `"experimental"` compatibility alias.
- gz-physics compatibility and performance parity are explicit gates, not
  optional follow-up work.

## How to Resume

```bash
git checkout feature/new_coll
git status -sb
git log -5 --oneline --decorate
```

Then read:

- `docs/dev_tasks/native_collision/README.md`
- `docs/dev_tasks/native_collision/01-design.md`
- `docs/dev_tasks/native_collision/02-milestones.md`
- `docs/dev_tasks/native_collision/03-evidence-gates.md`

Use `pixi run ...` tasks for validation and update the evidence gates after
each meaningful test or benchmark run.
