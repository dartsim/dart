# DART Examples README

## Build Each Example

Copy the subdirectory to your workspace and follow the instruction of README.md
in the subdirectory.

## Build Examples as One Project

### Build Instructions

This project is dependent on DART. Please make sure a proper version of DART is
installed before building this project.

Copy this directory to your workspace (e.g., in Linux):

    $ cp -r examples /your/workspace/directory/dart_examples
    $ cd /your/workspace/directory/dart_examples

From the workspace directory:

    $ mkdir build
    $ cd build
    $ cmake ..
    $ make

### Execute Instructions

Launch the each executable from the build directory above (e.g.,):

    $ ./hello_world

Follow the instructions detailed in the console.

## Dynamic Joint Constraint Examples

The `dynamic_joint_constraints` GUI example demonstrates the DART 6.x runtime
dynamic joint constraints side by side. The `cylindrical_constraint` headless
example provides a focused convergence smoke test for slide-and-rotate
attachments with `dart::constraint::CylindricalJointConstraint`.

`BallJointConstraint` keeps one anchor coincident and leaves all relative
rotation free. `CylindricalJointConstraint` keeps the bodies on one axis and
leaves translation along that axis plus rotation about that axis free.
`WeldJointConstraint` removes all relative rigid-body DOFs. DART 7/main also
has `RevoluteJointConstraint`, which keeps a fixed anchor and leaves only
rotation about the shared axis free; DART 6.19 does not expose that dynamic
constraint, so this example compares the 6.x runtime constraints directly.
