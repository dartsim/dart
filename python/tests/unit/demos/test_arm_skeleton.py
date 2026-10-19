"""Arm geometry, target controls, and ownership stay independent of the GUI host."""

import gc
import sys
from pathlib import Path
import weakref

import dartpy as dart
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from examples.demos.models.arm_skeleton import (
    ArmSkeleton,
    JointSpec,
    get_ssik_arm_specs,
    make_ssik_target_transform,
    ssik_target_controls,
)


def _chain():
    left = np.eye(4)
    left[:3, 3] = [0.4, 0.0, 0.1]
    right = np.eye(4)
    right[:3, 3] = [0.2, 0.0, 0.0]
    return [
        JointSpec(left, right, [0, 0, 3]),
        JointSpec(
            np.eye(4), np.eye(4), [0, 2, 0], prismatic=True
        ),
    ]


def _arm():
    arm = ArmSkeleton(dart.simulation.World())
    arm.build(_chain())
    return arm


def test_nineteen_arm_catalog_order():
    arms = get_ssik_arm_specs()
    assert len(arms) == 19
    assert (arms[0].label, arms[0].module) == (
        "Universal Robots UR5",
        "ur5_ik",
    )
    assert (arms[-1].label, arms[-1].module) == (
        "OpenArm right",
        "openarm_right_ik",
    )
    assert len({arm.label for arm in arms}) == 19
    assert len({arm.module for arm in arms}) == 19
    with pytest.raises(AttributeError):
        arms[0].label = "another arm"


def test_joint_specs_copy_inputs_and_return_independent_readonly_properties():
    left, right, axis = np.eye(4), np.eye(4), np.array([0.0, 0.0, 3.0])
    spec = JointSpec(left, right, axis)
    left[0, 3], right[1, 3], axis[0] = 1, 2, 3
    np.testing.assert_array_equal(spec.t_left, np.eye(4))
    np.testing.assert_array_equal(spec.t_right, np.eye(4))
    np.testing.assert_array_equal(spec.axis, [0, 0, 3])
    for name in ("t_left", "t_right", "axis"):
        value = getattr(spec, name)
        value[:] = 0
        assert np.any(getattr(spec, name))
        with pytest.raises(AttributeError):
            setattr(spec, name, value)
    with pytest.raises(AttributeError):
        spec.prismatic = True


def test_mixed_joints_compose_rotated_fixed_frames():
    chain = _chain()
    left, right = np.eye(4), np.eye(4)
    left[:3, :3] = dart.math.AngleAxis(0.3, [1, 0, 0]).rotation()
    left[:3, 3] = [0.3, 0.1, 0.05]
    right[:3, :3] = dart.math.AngleAxis(-0.2, [0, 1, 0]).rotation()
    right[:3, 3] = [0.1, 0.02, 0.04]
    chain[1] = JointSpec(left, right, [0, 2, 0], prismatic=True)
    arm = ArmSkeleton(dart.simulation.World())
    arm.build(chain)
    arm.show_solutions([[0.7, 0.25]])
    skeleton = arm.world.getSkeleton(0)
    assert [body.getName() for body in skeleton.getBodyNodes()] == ["base", "b0", "b1"]
    assert [joint.getType() for joint in skeleton.getJoints()] == [
        "WeldJoint", "RevoluteJoint", "PrismaticJoint"
    ]
    rotation, translation = np.eye(4), np.eye(4)
    rotation[:3, :3] = dart.math.AngleAxis(0.7, [0, 0, 1]).rotation()
    translation[:3, 3] = [0, 0.25, 0]
    expected = chain[0].t_left @ rotation @ chain[0].t_right @ left @ translation @ right
    np.testing.assert_allclose(
        skeleton.getBodyNode(2).getWorldTransform().matrix(), expected, atol=1e-14, rtol=0
    )


@pytest.mark.parametrize("direction", [[0, 0, 1], [0, 0, -1], [1e-7, 0, -1], [1, 2, 3]])
def test_link_cylinders_connect_joint_origins_including_opposite_axes(direction):
    left = np.eye(4)
    left[:3, 3] = np.asarray(direction) * 0.4
    arm = ArmSkeleton(dart.simulation.World())
    arm.build([JointSpec(left, np.eye(4), [0, 0, 1])])
    arm.show_solutions([])
    shapes = arm.world.getSkeleton(0).getBodyNode(0).getShapeNodes()
    assert [node.getShape().getType() for node in shapes] == ["SphereShape", "CylinderShape"]
    cylinder = shapes[1]
    transform = cylinder.getRelativeTransform()
    delta = left[:3, 3]
    np.testing.assert_allclose(transform.translation(), delta * 0.5, atol=1e-14, rtol=0)
    np.testing.assert_allclose(
        transform.rotation()[:, 2], delta / np.linalg.norm(delta), atol=2e-9, rtol=0
    )
    assert cylinder.getShape().getRadius() == 0.028
    assert cylinder.getShape().getHeight() == np.linalg.norm(delta)


def test_chain_fk_and_sixteen_ghosts():
    arm = _arm()
    configs = [[i * 0.03, i * 0.02] for i in range(20)]
    arm.show_solutions(configs, 3)
    world = arm.world
    assert world.getNumSkeletons() == 16
    for i in range(16):
        skeleton = world.getSkeleton(i)
        assert skeleton.getName() == f"ssik_arm_{i}"
        assert skeleton.getNumBodyNodes() == 3
        assert skeleton.getNumDofs() == 2
        np.testing.assert_array_equal(skeleton.getPositions(), configs[i])
        angle, distance = configs[i]
        expected = np.eye(4)
        cosine, sine = np.cos(angle), np.sin(angle)
        expected[:3, :3] = [[cosine, -sine, 0], [sine, cosine, 0], [0, 0, 1]]
        expected[:3, 3] = [
            0.4 + cosine * 0.2 - sine * distance,
            sine * 0.2 + cosine * distance,
            0.1,
        ]
        np.testing.assert_allclose(
            skeleton.getBodyNode(2).getWorldTransform().matrix(),
            expected,
            atol=1e-14,
        )
        color = [0.90, 0.20, 0.18, 1] if i == 3 else [0.85, 0.32, 0.30, 0.22]
        for body in skeleton.getBodyNodes():
            for shape in body.getShapeNodes():
                np.testing.assert_array_equal(shape.getVisualAspect().getRGBA(), color)
                assert shape.getCollisionAspect() is None
    arm.show_solutions([], -1)
    assert world.getNumSkeletons() == 1
    np.testing.assert_array_equal(world.getSkeleton(0).getPositions(), [0, 0])
    arm.build([])
    arm.show_solutions([])
    assert world.getNumSkeletons() == 0
    arm.show_solutions(configs)
    assert world.getNumSkeletons() == 0


def test_destroying_arm_removes_its_ghosts_from_the_world():
    world = dart.simulation.World()
    arm = ArmSkeleton(world)
    arm.build(_chain())
    arm.show_solutions([[0.1, 0.2], [0.3, 0.4]], 0)
    assert world.getNumSkeletons() == 2
    reference = weakref.ref(arm)
    del arm
    gc.collect()
    assert reference() is None
    assert world.getNumSkeletons() == 0


def test_rebuild_and_destruction_preserve_unrelated_skeletons_and_instance_names():
    world = dart.simulation.World()
    unrelated = dart.dynamics.Skeleton("unrelated")
    world.addSkeleton(unrelated)
    arm = ArmSkeleton(world)
    arm.build(_chain())
    arm.show_solutions([[0.1, 0.2], [0.3, 0.4]])
    assert world.getNumSkeletons() == 3
    arm.show_solutions([])
    np.testing.assert_array_equal(
        world.getSkeleton(1).getBodyNode(0).getShapeNode(0).getVisualAspect().getRGBA(),
        [0.55, 0.58, 0.62, 0.5],
    )
    arm.build(_chain())
    assert world.getNumSkeletons() == 1
    arm.show_solutions([])
    assert world.getSkeleton(1).getName() == "ssik_arm_2"
    del arm
    gc.collect()
    assert world.getNumSkeletons() == 1
    assert world.getSkeleton(0) is unrelated


def test_invalid_world_chain_and_solution_leave_existing_pose_unchanged():
    with pytest.raises(ValueError):
        ArmSkeleton(None)
    with pytest.raises(TypeError):
        ArmSkeleton("world")
    bad_transforms = []
    for index, value in (((0, 0), np.nan), ((3, 3), 2), ((0, 0), 2)):
        transform = np.eye(4)
        transform[index] = value
        bad_transforms.append(transform)
    for transform in bad_transforms:
        with pytest.raises(ValueError):
            JointSpec(transform, np.eye(4), [0, 0, 1])
        with pytest.raises(ValueError):
            JointSpec(np.eye(4), transform, [0, 0, 1])
    for axis in ([0, 0, 0], [np.inf, 0, 1], [0, np.nan, 1], [1e308, 0, 1]):
        with pytest.raises(ValueError):
            JointSpec(np.eye(4), np.eye(4), axis)
    with pytest.raises(TypeError):
        JointSpec(np.eye(3), np.eye(4), [0, 0, 1])
    arm = _arm()
    arm.show_solutions([[0.1, 0.2]])
    before = arm.world.getSkeleton(0).getPositions().copy()
    for configs, selected in (
        ([[0.1]], 0),
        ([[0.1, np.nan]], 0),
        ([[0.1, np.inf]], 0),
        ([[0.1, 0.2], [0.1]], 0),
        ([[0.1, 0.2]], -2),
        ([[0.1, 0.2]], 1),
        ([], 1),
    ):
        with pytest.raises(ValueError):
            arm.show_solutions(configs, selected)
        assert arm.world.getNumSkeletons() == 1
        np.testing.assert_array_equal(arm.world.getSkeleton(0).getPositions(), before)


def test_target_helpers_preserve_source_float_rounding_and_euler_branch():
    xyz = [0.123456789, -0.234567891, 0.345678912]
    transform = make_ssik_target_transform(xyz, [0, 0, -30])
    np.testing.assert_array_equal(transform.translation(), np.asarray(xyz, np.float32))
    read_xyz, read_rpy = ssik_target_controls(transform)
    np.testing.assert_array_equal(read_xyz, transform.translation())
    assert 0 <= read_rpy[2] <= 180
    reconstructed = make_ssik_target_transform(read_xyz, read_rpy)
    np.testing.assert_allclose(reconstructed.matrix(), transform.matrix(), atol=1e-7)
    for bad in ([np.nan, 0, 0], [0, np.inf, 0], [0, 0, 1e100]):
        with pytest.raises(ValueError):
            make_ssik_target_transform(bad, [0, 0, 0])
        with pytest.raises(ValueError):
            make_ssik_target_transform([0, 0, 0], bad)
    invalid = dart.math.Isometry3()
    invalid.set_translation([np.nan, 0, 0])
    with pytest.raises(ValueError):
        ssik_target_controls(invalid)


def test_arm_retains_world_and_collects_world_cycles():
    class World(dart.simulation.World):
        pass

    world = World()
    arm = ArmSkeleton(world)
    arm.build(_chain())
    arm.show_solutions([[0.1, 0.2]])
    reference = weakref.ref(world)
    del world
    gc.collect()
    assert reference() is arm.world
    world = arm.world
    world.arm = arm
    references = weakref.ref(world), weakref.ref(arm)
    del world, arm
    gc.collect()
    assert all(reference() is None for reference in references)
