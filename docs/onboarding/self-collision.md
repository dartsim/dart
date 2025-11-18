# Self-Collision in DART

This note explains how to enable self-collision robustly and how to ignore problematic link pairs (the root cause of historic issue [#743](https://github.com/dartsim/dart/issues/743)).

## Recommended workflow
- Enable self-collision per skeleton and keep the default adjacent-body filter enabled (parent/child pairs are ignored unless you explicitly enable adjacent checks).
- Before simulating, verify that the robot's initial pose is collision-free; deep interpenetrations at startup will trigger constraint infeasibility or NaNs.
- Ignore additional near-adjacent pairs (e.g., grandparent–grandchild links that nest inside each other) with `BodyNodeCollisionFilter`.

### Enable self-collision
```cpp
// C++
auto skel = loader.parseSkeleton("robot.urdf");
skel->enableSelfCollisionCheck();              // turn on self-collision
// skel->enableAdjacentBodyCheck();            // only if you truly want parent/child contacts
world->addSkeleton(skel);
```

```python
# Python
skel = dart.utils.DartLoader().parseSkeleton("robot.urdf")
skel.enableSelfCollisionCheck()
# skel.enableAdjacentBodyCheck()              # rarely needed; can create infeasible constraints
world.addSkeleton(skel)
```

### Check a pose for unintended self-contacts
Call collide once before running dynamics to identify overlapping links in the default configuration:
```cpp
auto solver = world->getConstraintSolver();
auto& option = solver->getCollisionOption();  // default filter is BodyNodeCollisionFilter
dart::collision::CollisionResult result;
solver->getCollisionGroup()->collide(option, &result);
// Inspect result.getContacts() to find link pairs that overlap before simulation starts.
```

### Ignore additional link pairs
If you find grandparent–grandchild (or other tightly nested) pairs that should always be ignored, add them to the blacklist:
```cpp
auto& option = world->getConstraintSolver()->getCollisionOption();
auto filter = std::dynamic_pointer_cast<dart::collision::BodyNodeCollisionFilter>(
    option.collisionFilter);
if (!filter) {
  filter = std::make_shared<dart::collision::BodyNodeCollisionFilter>();
  option.collisionFilter = filter;
}
filter->addBodyNodePairToBlackList(bodyA, bodyB);
```

```python
option = world.getConstraintSolver().getCollisionOption()
filter = option.collisionFilter
if filter is None:
    filter = dart.collision.BodyNodeCollisionFilter()
    option.collisionFilter = filter
filter.addBodyNodePairToBlackList(bodyA, bodyB)
```

## Validating fixes (CI-friendly)
- Unit/integration coverage already exercises self-collision filtering: `tests/integration/collision/test_SelfCollisionFiltering.cpp` and `tests/unit/collision/test_DistanceFilter.cpp`.
- For a focused check in CI, run `pixi run -e default test -R SelfCollisionFiltering` (no local build artifacts required).

## When problems persist
- Ensure geometry is not interpenetrating at load time; adjust joint default positions or shape sizes if needed.
- Prefer keeping adjacent-body collisions disabled unless your application truly requires them.
- If the constraint solver still reports infeasible contacts after ignoring intended pairs, look for modeling errors (zero-thickness meshes, overlapping collision shapes, missing inertial data).
