# Cylindrical Constraint Example

## Summary

- Goal: attach two free bodies at runtime while allowing slide and spin along a
  shared axis.
- Concepts/APIs: `dart::constraint::CylindricalJointConstraint`,
  `dart::constraint::ConstraintSolver`, `dart::simulation::World::step`.
- Expected output: initial and final radial axis error printed to the console.

For a GUI comparison of every DART 6.x runtime dynamic joint constraint, see
the `dynamic_joint_constraints` scene in the consolidated `dart-demos`
catalog (`pixi run demos`, then select "Dynamic Joint Constraints").

`CylindricalJointConstraint` sits between the existing runtime dynamic joint
constraints:

- `BallJointConstraint` keeps one anchor coincident and leaves all relative
  rotation free.
- `CylindricalJointConstraint` keeps the bodies on one axis and leaves
  translation along that axis plus rotation about that axis free.
- `WeldJointConstraint` removes all relative rigid-body DOFs.
- DART 7/main `RevoluteJointConstraint` keeps a fixed anchor and leaves only
  rotation about the shared axis free. DART 6.19 does not expose that dynamic
  constraint, so this example uses the 6.x runtime constraints directly.

## Build Instructions

From this directory:

    $ mkdir build
    $ cd build
    $ cmake ..
    $ make

## Execute Instructions

Launch the executable from the build directory above:

    $ ./cylindrical_constraint
