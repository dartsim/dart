import platform

import dartpy as dart
import numpy as np
import pytest


def collision_groups_tester(cd):
    size = [1, 1, 1]
    pos1 = [0, 0, 0]
    pos2 = [0.5, 0, 0]

    simple_frame1 = dart.dynamics.SimpleFrame()
    simple_frame2 = dart.dynamics.SimpleFrame()

    sphere1 = dart.dynamics.SphereShape(1)
    sphere2 = dart.dynamics.SphereShape(1)

    simple_frame1.set_shape(sphere1)
    simple_frame2.set_shape(sphere2)

    group = cd.create_collision_group()
    group.add_shape_frame(simple_frame1)
    group.add_shape_frame(simple_frame2)
    assert group.get_num_shape_frames() == 2

    #
    #    ( s1,s2 )              collision!
    # ---+---|---+---+---+---+--->
    #   -1   0  +1  +2  +3  +4
    #
    assert group.collide()

    #
    #    (  s1   )   (  s2   )  no collision
    # ---+---|---+---+---+---+--->
    #   -1   0  +1  +2  +3  +4
    #
    simple_frame2.set_translation([3, 0, 0])
    assert not group.collide()

    option = dart.CollisionOption()
    result = dart.CollisionResult()

    group.collide(option, result)
    assert not result.is_collision()
    assert result.get_num_contacts() == 0

    option.enable_contact = True
    simple_frame2.set_translation([1.99, 0, 0])

    group.collide(option, result)
    assert result.is_collision()
    assert result.get_num_contacts() != 0

    # Repeat the same test with BodyNodes instead of SimpleFrames

    group.remove_all_shape_frames()
    assert group.get_num_shape_frames() == 0

    skel1 = dart.dynamics.Skeleton()
    skel2 = dart.dynamics.Skeleton()

    [joint1, body1] = skel1.create_free_joint_and_body_node_pair(None)
    [joint2, body2] = skel2.create_free_joint_and_body_node_pair(None)

    shape_node1 = body1.create_shape_node(sphere1)
    shape_node1.create_visual_aspect()
    shape_node1.create_collision_aspect()

    shape_node2 = body2.create_shape_node(sphere2)
    shape_node2.create_visual_aspect()
    shape_node2.create_collision_aspect()

    group.add_shape_frames_of(body1)
    group.add_shape_frames_of(body2)

    assert group.get_num_shape_frames() == 2

    assert group.collide()

    joint2.set_position(3, 3)
    assert not group.collide()

    # Repeat the same test with BodyNodes and two groups

    joint2.set_position(3, 0)

    group.remove_all_shape_frames()
    assert group.get_num_shape_frames() == 0
    group2 = cd.create_collision_group()

    group.add_shape_frames_of(body1)
    group2.add_shape_frames_of(body2)

    assert group.get_num_shape_frames() == 1
    assert group2.get_num_shape_frames() == 1

    assert group.collide(group2)

    joint2.set_position(3, 3)
    assert not group.collide(group2)


def test_collision_groups():
    cd = dart.FCLCollisionDetector()
    collision_groups_tester(cd)

    cd = dart.DARTCollisionDetector()
    collision_groups_tester(cd)

    if hasattr(dart.collision, "BulletCollisionDetector"):
        cd = dart.BulletCollisionDetector()
        collision_groups_tester(cd)

    if hasattr(dart.collision, "OdeCollisionDetector"):
        cd = dart.OdeCollisionDetector()
        collision_groups_tester(cd)


# TODO: Add more collision detectors
@pytest.mark.parametrize("cd", [dart.FCLCollisionDetector()])
def test_filter(cd):
    # Create two bodies skeleton. The two bodies are placed at the same position
    # with the same size shape so that they collide by default.
    skel = dart.dynamics.Skeleton()

    shape = dart.dynamics.BoxShape(np.ones(3))

    _, body0 = skel.create_revolute_joint_and_body_node_pair()
    shape_node0 = body0.create_shape_node(shape)
    shape_node0.create_visual_aspect()
    shape_node0.create_collision_aspect()

    _, body1 = skel.create_revolute_joint_and_body_node_pair(body0)
    shape_node1 = body1.create_shape_node(shape)
    shape_node1.create_visual_aspect()
    shape_node1.create_collision_aspect()

    # Create a world and add the created skeleton
    world = dart.simulation.World()
    world.add_skeleton(skel)

    # Set a new collision detector
    constraint_solver = world.get_constraint_solver()
    constraint_solver.set_collision_detector(cd)

    # Get the collision group from the constraint solver
    group = constraint_solver.get_collision_group()
    assert group.get_num_shape_frames() == 2

    # Create BodyNodeCollisionFilter
    option = constraint_solver.get_collision_option()
    body_node_filter = dart.BodyNodeCollisionFilter()
    option.collision_filter = body_node_filter

    skel.enable_self_collision_check()
    skel.enable_adjacent_body_check()
    assert skel.is_enabled_self_collision_check()
    assert skel.is_enabled_adjacent_body_check()
    assert group.collide()
    assert group.collide(option)

    skel.enable_self_collision_check()
    skel.disable_adjacent_body_check()
    assert skel.is_enabled_self_collision_check()
    assert not skel.is_enabled_adjacent_body_check()
    assert group.collide()
    assert not group.collide(option)

    skel.disable_self_collision_check()
    skel.enable_adjacent_body_check()
    assert not skel.is_enabled_self_collision_check()
    assert skel.is_enabled_adjacent_body_check()
    assert group.collide()
    assert not group.collide(option)

    skel.disable_self_collision_check()
    skel.disable_adjacent_body_check()
    assert not skel.is_enabled_self_collision_check()
    assert not skel.is_enabled_adjacent_body_check()
    assert group.collide()
    assert not group.collide(option)

    # Test collision body filtering
    skel.enable_self_collision_check()
    skel.enable_adjacent_body_check()
    body_node_filter.add_body_node_pair_to_black_list(body0, body1)
    assert not group.collide(option)
    body_node_filter.remove_body_node_pair_from_black_list(body0, body1)
    assert group.collide(option)
    body_node_filter.add_body_node_pair_to_black_list(body0, body1)
    assert not group.collide(option)
    body_node_filter.remove_all_body_node_pairs_from_black_list()
    assert group.collide(option)


def test_raycast():
    cd = dart.BulletCollisionDetector()

    simple_frame = dart.dynamics.SimpleFrame()
    sphere = dart.dynamics.SphereShape(1)
    simple_frame.set_shape(sphere)

    group = cd.create_collision_group()
    group.add_shape_frame(simple_frame)
    assert group.get_num_shape_frames() == 1

    option = dart.RaycastOption()
    option.m_enable_all_hits = False

    result = dart.RaycastResult()
    assert not result.has_hit()

    ray_hit = dart.RayHit()

    result.clear()
    simple_frame.set_translation(np.zeros(3))
    assert group.raycast([-2, 0, 0], [2, 0, 0], option, result)
    assert result.has_hit()
    assert len(result.m_ray_hits) == 1
    ray_hit = result.m_ray_hits[0]
    assert np.isclose(ray_hit.m_point, [-1, 0, 0]).all()
    assert np.isclose(ray_hit.m_normal, [-1, 0, 0]).all()
    assert ray_hit.m_fraction == pytest.approx(0.25)

    result.clear()
    simple_frame.set_translation(np.zeros(3))
    assert group.raycast([2, 0, 0], [-2, 0, 0], option, result)
    assert result.has_hit()
    assert len(result.m_ray_hits) == 1
    ray_hit = result.m_ray_hits[0]
    assert np.isclose(ray_hit.m_point, [1, 0, 0]).all()
    assert np.isclose(ray_hit.m_normal, [1, 0, 0]).all()
    assert ray_hit.m_fraction == pytest.approx(0.25)

    result.clear()
    simple_frame.set_translation([1, 0, 0])
    assert group.raycast([-2, 0, 0], [2, 0, 0], option, result)
    assert result.has_hit()
    assert len(result.m_ray_hits) == 1
    ray_hit = result.m_ray_hits[0]
    assert np.isclose(ray_hit.m_point, [0, 0, 0]).all()
    assert np.isclose(ray_hit.m_normal, [-1, 0, 0]).all()
    assert ray_hit.m_fraction == pytest.approx(0.5)


if __name__ == "__main__":
    pytest.main()
