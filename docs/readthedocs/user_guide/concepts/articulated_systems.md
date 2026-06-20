# Articulated systems

A robot arm, a leg, a pendulum chain — these are **articulated systems**: rigid
links connected by joints. In DART 7 you build one as a `Multibody`: a tree of
**links** joined by **joints**, owned by the world.

## Building a multibody

Add a multibody to the world, then add links. The first link is the root; every
later link names its `parent` and the `JointSpec` that connects it. A
`JointSpec` describes the joint's type, axis, and where it sits relative to the
parent:

```python
import dartpy as dart
import numpy as np

def translation(x, y, z):
    t = np.eye(4)
    t[:3, 3] = (x, y, z)
    return t

world = dart.World()
robot = world.add_multibody("arm")

# Root link.
base = robot.add_link("base")

# Upper arm on a revolute (hinge) shoulder, one metre out from the base.
upper = robot.add_link(
    "upper_arm",
    parent=base,
    joint=dart.JointSpec(
        name="shoulder",
        type=dart.JointType.REVOLUTE,
        axis=(0.0, 1.0, 0.0),
        transform_from_parent=translation(1.0, 0.0, 0.0),
    ),
)
upper.mass = 2.0
upper.inertia = ((0.10, 0.0, 0.0), (0.0, 0.20, 0.0), (0.0, 0.0, 0.30))

# Forearm on a 2-axis universal wrist.
fore = robot.add_link(
    "forearm",
    parent=upper,
    joint=dart.JointSpec(
        name="wrist",
        type=dart.JointType.UNIVERSAL,
        axis=(0.0, 0.0, 1.0),
        axis2=(0.0, 1.0, 0.0),
        transform_from_parent=translation(1.0, 0.0, 0.0),
    ),
)
fore.mass = 1.0
fore.inertia = ((0.05, 0.0, 0.0), (0.0, 0.05, 0.0), (0.0, 0.0, 0.05))

world.enter_simulation_mode()
```

This arm has three degrees of freedom: a 1-DOF revolute shoulder plus a 2-DOF
universal wrist. `robot.num_dofs` reports the total generalized-coordinate count
for the tree.

## Links

A **link** is a rigid body inside the multibody tree. It carries inertial
properties and acts as a frame you can query:

```python
upper.mass                      # link mass
upper.inertia                   # 3x3 inertia tensor
upper.translation               # world-frame position of the link
upper.parent_joint              # the joint connecting it to its parent
```

## Joints

A **joint** defines how a link can move relative to its parent. You reach a
joint through its child link's `parent_joint`, then read or write its
generalized state:

```python
shoulder = upper.parent_joint
shoulder.position               # generalized position(s)
shoulder.velocity               # generalized velocity(ies)
shoulder.damping_coefficient    # viscous damping
```

Common joint types include `REVOLUTE` (a hinge, 1 DOF), `UNIVERSAL` (2 DOF), and
others selected through `dart.JointType`. The `axis` (and `axis2` for two-axis
joints) sets the direction of motion, and `transform_from_parent` places the
joint frame relative to the parent link.

## Trees and closed loops

A `Multibody` is a **tree**: each link has exactly one parent joint, which keeps
articulated dynamics fast and well-conditioned. Mechanisms with closed loops —
four-bar linkages, parallel arms — are modeled by adding explicit **loop
closures** on top of the tree rather than by forcing a second parent. Loop
closure is an advanced, still-maturing area of the DART 7 API; for now, build the
tree and consult the
[simulation API design notes](https://github.com/dartsim/dart/blob/main/docs/design/simulation_cpp_api.md)
for the current state of closed-chain support.

## Next

Bodies now move and collide. The next section explains how DART finds and
resolves those collisions: {doc}`collisions & contacts
<../interaction/collisions_and_contacts>`.
