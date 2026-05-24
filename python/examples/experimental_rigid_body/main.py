"""Experimental rigid-body simulation MVP (DART 7 experimental World).

This example showcases the experimental ECS-based ``World`` as a first
rigid-body dynamics solver:

* articulated forward dynamics for several joint types (revolute, universal,
  prismatic) on a staged compute pipeline,
* generalized-coordinate dynamics accessors (mass matrix, Coriolis, gravity)
  and inverse dynamics,
* a floating base via a free joint with SO(3)/SE(3) manifold integration, and
* contact resolution (a link resting on static ground).

Run with the built dartpy on PYTHONPATH, e.g.::

    PYTHONPATH=build/default/cpp/Release/python python python/examples/experimental_rigid_body/main.py
"""

import numpy as np

import dartpy.simulation_experimental as sx


def _translation(x, y, z):
    transform = np.eye(4)
    transform[:3, 3] = (x, y, z)
    return transform


def articulated_dynamics_demo():
    """A 2-DOF arm: report the generalized dynamics terms and inverse dynamics."""
    print("== Articulated dynamics (revolute + universal arm) ==")
    world = sx.World()  # default gravity (0, 0, -9.81)
    robot = world.add_multibody("arm")

    base = robot.add_link("base")
    upper = robot.add_link(
        "upper_arm",
        parent=base,
        joint=sx.JointSpec(
            name="shoulder",
            type=sx.JointType.REVOLUTE,
            axis=(0.0, 1.0, 0.0),
            transform_from_parent=_translation(1.0, 0.0, 0.0),
        ),
    )
    upper.mass = 2.0
    upper.inertia = ((0.10, 0.0, 0.0), (0.0, 0.20, 0.0), (0.0, 0.0, 0.30))

    fore = robot.add_link(
        "forearm",
        parent=upper,
        joint=sx.JointSpec(
            name="wrist",
            type=sx.JointType.UNIVERSAL,
            axis=(0.0, 0.0, 1.0),
            axis2=(0.0, 1.0, 0.0),
            transform_from_parent=_translation(1.0, 0.0, 0.0),
        ),
    )
    fore.mass = 1.0
    fore.inertia = ((0.05, 0.0, 0.0), (0.0, 0.05, 0.0), (0.0, 0.0, 0.05))

    world.enter_simulation_mode()

    print(f"  degrees of freedom : {robot.num_dofs}")
    print(f"  mass matrix M(q)   :\n{np.array2string(robot.mass_matrix, prefix='    ')}")
    print(f"  gravity forces g(q): {robot.gravity_forces}")

    # Inverse dynamics round-trip: the torque that produces a target acceleration
    # reproduces that acceleration through forward dynamics.
    desired = np.array([0.5, -0.2, 0.3])
    tau = robot.compute_inverse_dynamics(desired)
    offset = 0
    for joint in robot.joints:
        joint.force = tau[offset : offset + joint.num_dofs].tolist()
        offset += joint.num_dofs
    world.step()
    achieved = np.concatenate([joint.acceleration for joint in robot.joints])
    print(f"  inverse-dynamics round trip: desired {desired}, achieved {achieved}")


def floating_base_demo():
    """A free-floating body free-falls under gravity (SE(3) integration)."""
    print("\n== Floating base (free joint) ==")
    world = sx.World()
    robot = world.add_multibody("floating")
    base = robot.add_link("base")
    body = robot.add_link(
        "body",
        parent=base,
        joint=sx.JointSpec(name="floating", type=sx.JointType.FLOATING),
    )
    body.mass = 1.5
    body.inertia = ((0.1, 0.0, 0.0), (0.0, 0.1, 0.0), (0.0, 0.0, 0.1))

    joint = body.parent_joint
    # Free-joint velocity is [linear; angular]: drift along +X while spinning.
    joint.velocity = [1.0, 0.0, 0.0, 0.0, 0.0, 2.0]

    dt = 0.01
    world.time_step = dt
    world.enter_simulation_mode()
    world.step(n=50)

    position = joint.position  # [translation; rotation vector]
    print(f"  after {0.5:.2f}s: translation {position[:3]}, rotation vec {position[3:]}")


def contact_demo():
    """A prismatic 'leg' with a sphere foot drops and rests on static ground."""
    print("\n== Contact (link resting on static ground) ==")
    world = sx.World()  # default gravity (0, 0, -9.81)
    robot = world.add_multibody("leg_robot")
    base = robot.add_link("base")
    leg = robot.add_link(
        "leg",
        parent=base,
        joint=sx.JointSpec(
            name="slider", type=sx.JointType.PRISMATIC, axis=(0.0, 0.0, 1.0)
        ),
    )
    leg.mass = 1.0
    leg.set_collision_shape(sx.CollisionShape.sphere(0.2))
    leg.parent_joint.position = [-0.25]  # start just above the ground

    ground = world.add_rigid_body("ground", position=(0.0, 0.0, -1.0))
    ground.is_static = True
    ground.set_collision_shape(sx.CollisionShape.box((5.0, 5.0, 0.5)))

    world.time_step = 0.002
    world.enter_simulation_mode()
    world.step(n=1500)

    rest_z = leg.transform[2, 3]
    print(f"  foot settles at z = {rest_z:.3f} (ground top at -0.5, radius 0.2)")


def main():
    articulated_dynamics_demo()
    floating_base_demo()
    contact_demo()


if __name__ == "__main__":
    main()
