import gc
import weakref

import dartpy as dart
import numpy as np
import pytest


def _chain():
    left = np.eye(4)
    left[:3, 3] = [0.4, 0.0, 0.1]
    right = np.eye(4)
    right[:3, 3] = [0.2, 0.0, 0.0]
    return [
        dart._demos.JointSpec(left, right, [0, 0, 3]),
        dart._demos.JointSpec(
            np.eye(4), np.eye(4), [0, 2, 0], prismatic=True
        ),
    ]


def _arm():
    arm = dart._demos.ArmSkeleton(dart.simulation.World())
    arm.build(_chain())
    return arm


def test_nineteen_arm_catalog_order():
    arms = dart._demos.get_ssik_arm_specs()
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


def test_native_chain_fk_and_sixteen_ghosts():
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
    arm = dart._demos.ArmSkeleton(world)
    arm.build(_chain())
    arm.show_solutions([[0.1, 0.2], [0.3, 0.4]], 0)
    assert world.getNumSkeletons() == 2
    reference = weakref.ref(arm)
    del arm
    gc.collect()
    assert reference() is None
    assert world.getNumSkeletons() == 0


def test_binding_rejects_invalid_world_chain_and_solution_without_mutation():
    with pytest.raises(ValueError):
        dart._demos.ArmSkeleton(None)
    with pytest.raises(TypeError):
        dart._demos.ArmSkeleton("world")
    bad_transforms = []
    for index, value in (((0, 0), np.nan), ((3, 3), 2), ((0, 0), 2)):
        transform = np.eye(4)
        transform[index] = value
        bad_transforms.append(transform)
    for transform in bad_transforms:
        with pytest.raises(ValueError):
            dart._demos.JointSpec(transform, np.eye(4), [0, 0, 1])
        with pytest.raises(ValueError):
            dart._demos.JointSpec(np.eye(4), transform, [0, 0, 1])
    for axis in ([0, 0, 0], [np.inf, 0, 1], [0, np.nan, 1], [1e308, 0, 1]):
        with pytest.raises(ValueError):
            dart._demos.JointSpec(np.eye(4), np.eye(4), axis)
    with pytest.raises(TypeError):
        dart._demos.JointSpec(np.eye(3), np.eye(4), [0, 0, 1])
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
    transform = dart._demos.make_ssik_target_transform(xyz, [0, 0, -30])
    np.testing.assert_array_equal(transform.translation(), np.asarray(xyz, np.float32))
    read_xyz, read_rpy = dart._demos.ssik_target_controls(transform)
    np.testing.assert_array_equal(read_xyz, transform.translation())
    assert 0 <= read_rpy[2] <= 180
    reconstructed = dart._demos.make_ssik_target_transform(read_xyz, read_rpy)
    np.testing.assert_allclose(reconstructed.matrix(), transform.matrix(), atol=1e-7)
    for bad in ([np.nan, 0, 0], [0, np.inf, 0], [0, 0, 1e100]):
        with pytest.raises(ValueError):
            dart._demos.make_ssik_target_transform(bad, [0, 0, 0])
        with pytest.raises(ValueError):
            dart._demos.make_ssik_target_transform([0, 0, 0], bad)
    invalid = dart.math.Isometry3()
    invalid.set_translation([np.nan, 0, 0])
    with pytest.raises(ValueError):
        dart._demos.ssik_target_controls(invalid)


def test_arm_retains_world_and_collects_world_cycles():
    class World(dart.simulation.World):
        pass

    world = World()
    arm = dart._demos.ArmSkeleton(world)
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
