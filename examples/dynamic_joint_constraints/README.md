# Dynamic Joint Constraints Example

## Summary

- Goal: compare DART 6.x runtime dynamic joint constraints in one GUI scene.
- Concepts/APIs: `dart::constraint::BallJointConstraint`,
  `dart::constraint::CylindricalJointConstraint`,
  `dart::constraint::WeldJointConstraint`,
  `dart::constraint::ConstraintSolver`, `dart::gui::osg::ImGuiViewer`.
- Expected behavior: the three rows show progressively stricter runtime
  attachments.

The viewer shows the DART 6.x dynamic joint constraints side by side:

- `BallJointConstraint` pins one body point to a world anchor and leaves all
  relative rotation free.
- `CylindricalJointConstraint` pins a body to a world axis and leaves
  translation along the axis plus rotation about the axis free.
- `WeldJointConstraint` keeps the full body transform fixed.

The scene uses large in-view labels, color-coded lanes, and motion markers so
the constraints can be compared without relying only on the ImGui panel. Use
`--gui-scale <value>` if the control panel text is too small on a high-DPI or
remote display.

DART 7/main also has `RevoluteJointConstraint`, which keeps a fixed anchor and
leaves only rotation about the shared axis free. DART 6.19 does not expose that
dynamic constraint, so this example compares the maintained 6.x runtime
constraints directly.

## Build Instructions

From this directory:

    $ mkdir build
    $ cd build
    $ cmake ..
    $ make

## Execute Instructions

Launch the executable from the build directory above:

    $ ./dynamic_joint_constraints

Scale the ImGui control panel when needed:

    $ ./dynamic_joint_constraints --gui-scale 1.5
