from __future__ import annotations

import importlib

import numpy as np
import pytest

import dartpy as dart


def _simulation():
    try:
        module = importlib.import_module("dartpy")
    except ModuleNotFoundError as exc:
        raise AssertionError(
            "dartpy should be available with the DART 7 World stack"
        ) from exc
    if not hasattr(module, "build_multibody_from_skeleton"):
        raise AssertionError(
            "dartpy imported but did not expose " "build_multibody_from_skeleton"
        )
    return module


def _two_link_revolute_skeleton():
    """A revolute-revolute serial chain whose root joint sits at the origin and
    whose second joint is offset one link down (matches the C++ parity test)."""
    skeleton = dart.Skeleton("double_pendulum")

    joint1, body1 = skeleton.create_revolute_joint_and_body_node_pair()
    joint1.set_axis(np.array([0.0, 1.0, 0.0]))
    body1.set_inertia(dart.Inertia(2.0, [0.0, 0.0, -0.5], np.diag([0.05, 0.05, 0.02])))

    joint2, body2 = skeleton.create_revolute_joint_and_body_node_pair(body1)
    joint2.set_axis(np.array([0.0, 1.0, 0.0]))
    offset = np.identity(4)
    offset[2, 3] = -1.0
    joint2.set_transform_from_parent_body_node(offset)
    body2.set_inertia(dart.Inertia(1.0, [0.0, 0.0, -0.5], np.diag([0.03, 0.03, 0.01])))

    # The legacy skeleton and the DART 7 World share the same default
    # gravity (0, 0, -9.81), so no explicit gravity is set here.
    return skeleton


def test_build_multibody_from_skeleton_matches_legacy():
    sx = _simulation()
    skeleton = _two_link_revolute_skeleton()
    skeleton.set_positions([0.3, -0.5])
    skeleton.set_velocities([0.7, 0.2])

    legacy_mass = np.array(skeleton.get_mass_matrix())
    legacy_coriolis_gravity = np.array(skeleton.get_coriolis_and_gravity_forces())

    world = sx.World()
    world.gravity = [0.0, 0.0, -9.81]
    multibody = sx.build_multibody_from_skeleton(world, skeleton)

    # One extra link: the synthetic world base.
    assert multibody.num_links == skeleton.get_num_body_nodes() + 1
    assert multibody.num_joints == skeleton.get_num_body_nodes()
    assert multibody.num_dofs == skeleton.get_num_dofs() == 2

    # DART 6 parity on the bound dynamics accessors.
    assert np.allclose(np.array(multibody.mass_matrix), legacy_mass, atol=1e-9)
    assert np.allclose(
        np.array(multibody.coriolis_and_gravity_forces),
        legacy_coriolis_gravity,
        atol=1e-9,
    )


def test_build_multibody_options_and_copy_state():
    sx = _simulation()
    skeleton = _two_link_revolute_skeleton()
    skeleton.set_positions([0.4, 0.25])

    world = sx.World()
    options = sx.SkeletonToMultibodyOptions()
    options.name = "loaded_robot"
    options.base_link_name = "ground"
    options.copy_state = True
    multibody = sx.build_multibody_from_skeleton(world, skeleton, options)

    assert multibody.name == "loaded_robot"
    assert multibody.get_link("ground") is not None
    joint_positions = [joint.position[0] for joint in multibody.joints]
    assert np.allclose(joint_positions, [0.4, 0.25], atol=1e-12)

    # With copy_state disabled, the multibody stays at the zero configuration.
    world_zero = sx.World()
    options_zero = sx.SkeletonToMultibodyOptions()
    options_zero.copy_state = False
    multibody_zero = sx.build_multibody_from_skeleton(
        world_zero, skeleton, options_zero
    )
    zero_positions = [joint.position[0] for joint in multibody_zero.joints]
    assert np.allclose(zero_positions, [0.0, 0.0], atol=1e-12)


def test_build_multibody_steps_under_gravity():
    sx = _simulation()
    skeleton = _two_link_revolute_skeleton()
    # Start off the hanging equilibrium so gravity produces a non-zero torque.
    skeleton.set_positions([0.6, 0.3])

    world = sx.World()
    world.gravity = [0.0, 0.0, -9.81]
    world.time_step = 1e-3
    multibody = sx.build_multibody_from_skeleton(world, skeleton)

    world.step(50)
    velocities = [abs(joint.velocity[0]) for joint in multibody.joints]
    assert max(velocities) > 0.0


def test_build_multibody_from_multi_tree_skeleton():
    sx = _simulation()

    # A skeleton with two independent root bodies (two trees). Both attach
    # beneath the single synthetic base link.
    skeleton = dart.Skeleton("two_trees")
    for _ in range(2):
        joint, body = skeleton.create_revolute_joint_and_body_node_pair()
        joint.set_axis(np.array([0.0, 1.0, 0.0]))
        body.set_inertia(
            dart.Inertia(1.0, [0.0, 0.0, -0.5], np.diag([0.02, 0.02, 0.01]))
        )
    skeleton.set_positions([0.2, -0.3])
    skeleton.set_velocities([0.4, 0.1])

    legacy_mass = np.array(skeleton.get_mass_matrix())
    legacy_coriolis_gravity = np.array(skeleton.get_coriolis_and_gravity_forces())

    world = sx.World()
    world.gravity = [0.0, 0.0, -9.81]
    multibody = sx.build_multibody_from_skeleton(world, skeleton)

    assert multibody.num_links == skeleton.get_num_body_nodes() + 1
    assert multibody.num_dofs == 2
    assert np.allclose(np.array(multibody.mass_matrix), legacy_mass, atol=1e-9)
    assert np.allclose(
        np.array(multibody.coriolis_and_gravity_forces),
        legacy_coriolis_gravity,
        atol=1e-9,
    )


def test_build_multibody_ball_joint_matches_legacy():
    sx = _simulation()

    skeleton = dart.Skeleton("ball")
    joint, body = skeleton.create_ball_joint_and_body_node_pair()
    joint.set_name("ball")
    body.set_inertia(
        dart.Inertia(1.2, [0.05, -0.05, -0.3], np.diag([0.04, 0.03, 0.02]))
    )
    skeleton.set_positions([0.2, -0.3, 0.15])  # rotation vector
    skeleton.set_velocities([0.3, 0.1, -0.2])  # body angular velocity

    legacy_mass = np.array(skeleton.get_mass_matrix())
    legacy_coriolis_gravity = np.array(skeleton.get_coriolis_and_gravity_forces())

    world = sx.World()
    world.gravity = [0.0, 0.0, -9.81]
    multibody = sx.build_multibody_from_skeleton(world, skeleton)

    assert multibody.num_dofs == 3
    # The ball joint shares the legacy coordinate basis, so quantities match
    # element-wise.
    assert np.allclose(np.array(multibody.mass_matrix), legacy_mass, atol=1e-9)
    assert np.allclose(
        np.array(multibody.coriolis_and_gravity_forces),
        legacy_coriolis_gravity,
        atol=1e-9,
    )


def test_build_multibody_floating_base_steps():
    sx = _simulation()

    # A free joint models a floating base: the body should fall under gravity.
    skeleton = dart.Skeleton("floating")
    joint, body = skeleton.create_free_joint_and_body_node_pair()
    joint.set_name("free")
    body.set_inertia(dart.Inertia(2.0, [0.0, 0.0, 0.0], np.diag([0.05, 0.05, 0.05])))
    skeleton.set_positions([0.0, 0.0, 0.0, 0.0, 0.0, 1.0])  # [rotation; translation]

    world = sx.World()
    world.gravity = [0.0, 0.0, -9.81]
    world.time_step = 1e-3
    multibody = sx.build_multibody_from_skeleton(world, skeleton)

    assert multibody.num_dofs == 6
    assert multibody.num_links == skeleton.get_num_body_nodes() + 1

    free = multibody.joints[0]
    world.step(20)
    # The floating body accelerates downward under gravity (experimental free
    # velocity is [linear; angular]; the linear-z component grows negative).
    assert free.velocity[2] < -1e-3


def test_build_multibody_branching_tree_matches_legacy():
    sx = _simulation()

    # A branching parent: a root revolute body with two revolute children at
    # different parent-side offsets. Each child joint carries its own offset.
    skeleton = dart.Skeleton("branch")
    j0, b0 = skeleton.create_revolute_joint_and_body_node_pair()
    j0.set_axis(np.array([0.0, 1.0, 0.0]))
    b0.set_inertia(dart.Inertia(2.0, [0.0, 0.0, -0.2], np.diag([0.05, 0.05, 0.03])))

    offset1 = np.identity(4)
    offset1[0, 3] = 0.3
    j1, b1 = skeleton.create_revolute_joint_and_body_node_pair(b0)
    j1.set_axis(np.array([0.0, 1.0, 0.0]))
    j1.set_transform_from_parent_body_node(offset1)
    b1.set_inertia(dart.Inertia(1.0, [0.0, 0.0, -0.25], np.diag([0.02, 0.02, 0.01])))

    offset2 = np.identity(4)
    offset2[0, 3] = -0.3
    offset2[2, 3] = 0.1
    j2, b2 = skeleton.create_revolute_joint_and_body_node_pair(b0)
    j2.set_axis(np.array([1.0, 0.0, 0.0]))
    j2.set_transform_from_parent_body_node(offset2)
    b2.set_inertia(dart.Inertia(0.8, [0.0, 0.0, -0.2], np.diag([0.015, 0.015, 0.008])))

    skeleton.set_positions([0.4, -0.3, 0.2])
    skeleton.set_velocities([0.2, 0.5, -0.1])

    legacy_mass = np.array(skeleton.get_mass_matrix())
    legacy_coriolis_gravity = np.array(skeleton.get_coriolis_and_gravity_forces())

    world = sx.World()
    world.gravity = [0.0, 0.0, -9.81]
    multibody = sx.build_multibody_from_skeleton(world, skeleton)

    assert multibody.num_dofs == 3
    assert np.allclose(np.array(multibody.mass_matrix), legacy_mass, atol=1e-9)
    assert np.allclose(
        np.array(multibody.coriolis_and_gravity_forces),
        legacy_coriolis_gravity,
        atol=1e-9,
    )


def test_build_multiple_multibodies_from_skeletons():
    sx = _simulation()

    double_pendulum = _two_link_revolute_skeleton()

    slider = dart.Skeleton("slider")
    joint, body = slider.create_prismatic_joint_and_body_node_pair()
    joint.set_axis(np.array([0.0, 0.0, 1.0]))
    body.set_inertia(dart.Inertia(1.0, [0.0, 0.0, 0.0], np.diag([0.01, 0.01, 0.01])))

    world = sx.World()
    multibodies = [
        sx.build_multibody_from_skeleton(world, double_pendulum),
        sx.build_multibody_from_skeleton(world, slider),
    ]

    assert len(multibodies) == 2
    assert world.num_multibodies == 2
    assert {mb.name for mb in multibodies} == {"double_pendulum", "slider"}
    assert multibodies[0].num_dofs == 2
    assert multibodies[1].num_dofs == 1


def test_build_multibody_from_urdf_skeleton():
    sx = _simulation()

    parser = dart.io.UrdfParser()
    options = dart.io.UrdfParserOptions()
    options.m_default_root_joint_type = dart.io.UrdfParserRootJointType.Fixed
    parser.set_options(options)
    skeleton = parser.parse_skeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf")
    assert skeleton is not None

    world = sx.World()
    multibody = sx.build_multibody_from_skeleton(world, skeleton)

    assert multibody.num_dofs == skeleton.get_num_dofs()
    assert multibody.num_links == skeleton.get_num_body_nodes() + 1
    # A converted, fixed-base arm steps without error.
    world.step(1)


def test_build_multibody_rejects_unsupported_joint():
    sx = _simulation()

    # Euler joints are not among the supported joint types.
    skeleton = dart.Skeleton("euler")
    _, body = skeleton.create_euler_joint_and_body_node_pair()
    body.set_inertia(dart.Inertia(1.0, [0.0, 0.0, 0.0], np.diag([0.01, 0.01, 0.01])))

    world = sx.World()
    with pytest.raises(Exception, match="does not yet support"):
        sx.build_multibody_from_skeleton(world, skeleton)
