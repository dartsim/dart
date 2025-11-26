# Continuous Collision Detection (Issue #426)

## Status (2025-03)

- Planar meshes no longer rely on the thin-box workaround: the FCL backend now uses an analytic halfspace, and `Issue426.FclThinBoxMeshModeUsesHalfspacePlane` guards a thin box resting on a plane even when the detector runs in `MESH` mode.
- DART still has no CCD plumbing. Collision detectors sample only end-of-step poses, `World::step()` uses a fixed step, and Bullet's CCD hooks are not wired up. Thin, fast bodies can still tunnel through the ground or each other.

## Reproduction

- Ground contact coverage: run `ctest -R Issue426` after a `pixi run config && pixi run build-tests` to exercise the thin-box-vs-plane regression.
- CCD gap: configure and build the tests, then run a short harness (C++/dartpy) that
  1. creates two free Skeletons with `BoxShape({0.1, 0.1, 1e-4})`,
  2. places them at `(-0.05, 0, 0)` and `(0.05, 0, 0)` with linear velocities `(10, 0, 0)` and `(-10, 0, 0)`,
  3. sets `world->setTimeStep(0.01)`, calls `world->step()`, and inspects `world->getLastCollisionResult().getNumContacts()`—it is `0` even though the boxes cross paths mid-step. Swapping to the Bullet detector yields the same outcome, confirming CCD is the missing piece.

## Next Steps

- Track previous transforms per `CollisionObject`/`CollisionGroup` and add a CCD flag to `CollisionOption` so detectors can run swept queries (FCL `continuousCollide`, Bullet convex casts).
- In `World::step()`, subdivide the time step at the earliest time of impact and feed TOI poses into `ConstraintSolver` instead of resolving after penetration.
- Add regressions for crossing thin plates and fast self-collision for each backend, and surface CCD toggles (including swept-sphere radius) through dartpy.
