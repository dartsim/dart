"""
Integration tests for Atlas robot inverse kinematics.

This test suite mirrors atlas_puppet example scenarios to ensure IK works
correctly for complex humanoid robots.
"""

import dartpy as dart
import numpy as np
import pytest


def create_simple_atlas():
    """Create a simplified Atlas robot for testing."""
    urdf = dart.utils.DartLoader()
    atlas = urdf.parseSkeleton("dart://sample/sdf/atlas/atlas_v3_no_head.urdf")
    return atlas


def setup_atlas_standing_pose(atlas):
    """Configure Atlas into a default standing pose."""
    # Right leg
    atlas.getDof("r_leg_hpy").setPosition(-45.0 * np.pi / 180.0)
    atlas.getDof("r_leg_kny").setPosition(90.0 * np.pi / 180.0)
    atlas.getDof("r_leg_aky").setPosition(-45.0 * np.pi / 180.0)

    # Left leg
    atlas.getDof("l_leg_hpy").setPosition(-45.0 * np.pi / 180.0)
    atlas.getDof("l_leg_kny").setPosition(90.0 * np.pi / 180.0)
    atlas.getDof("l_leg_aky").setPosition(-45.0 * np.pi / 180.0)

    # Prevent knees from bending backwards
    atlas.getDof("r_leg_kny").setPositionLowerLimit(10 * np.pi / 180.0)
    atlas.getDof("l_leg_kny").setPositionLowerLimit(10 * np.pi / 180.0)


def test_atlas_hand_ik_simple():
    """Test that hand IK can reach a simple target."""
    # Create Atlas
    atlas = create_simple_atlas()
    setup_atlas_standing_pose(atlas)

    # Get the left hand body node
    l_hand_bn = atlas.getBodyNode("l_hand")
    assert l_hand_bn is not None

    # Create end effector
    tf_hand = dart.math.Isometry3()
    tf_hand.set_translation([0.0, 0.12, 0.0])
    l_hand = l_hand_bn.createEndEffector("l_hand")
    l_hand.setDefaultRelativeTransform(tf_hand, True)

    # Create a simple frame target
    target = dart.dynamics.SimpleFrame(dart.dynamics.Frame.World(), "target")

    # Store initial hand position
    initial_pos = l_hand.getWorldTransform().translation()

    # Set target 10cm forward from initial position
    target_tf = l_hand.getWorldTransform()
    target_pos = initial_pos + np.array([0.1, 0.0, 0.0])
    target_tf.set_translation(target_pos)
    target.setTransform(target_tf)

    # Set up IK
    ik = l_hand.getIK(True)
    ik.setTarget(target)
    ik.setActive(True)

    # Use whole body IK
    ik.useWholeBody()

    # ✅ FIX: Set tight bounds so displacement produces error
    # With infinite bounds, the error is always zero (the bug)
    ik.getErrorMethod().setLinearBounds(
        np.array([-1e-8, -1e-8, -1e-8]), np.array([1e-8, 1e-8, 1e-8])
    )
    ik.getErrorMethod().setAngularBounds(
        np.array([-1e-8, -1e-8, -1e-8]), np.array([1e-8, 1e-8, 1e-8])
    )

    # Configure solver
    solver = ik.getSolver()
    solver.setNumMaxIterations(100)

    # Verify initial distance
    initial_distance = np.linalg.norm(
        l_hand.getWorldTransform().translation() - target_pos
    )
    print(f"Initial distance to target: {initial_distance:.4f}m")
    assert initial_distance > 0.05, "Target should be at least 5cm away"

    # Solve IK - Use the SKELETON's hierarchical IK, not the individual IK module
    print("Solving IK using skeleton's hierarchical IK...")
    skel_ik = atlas.getIK(True)  # Get skeleton's hierarchical IK
    success = skel_ik.solveAndApply(True)

    # Check final distance
    final_pos = l_hand.getWorldTransform().translation()
    final_distance = np.linalg.norm(final_pos - target_pos)
    print(f"Final distance to target: {final_distance:.4f}m")
    print(f"IK solve returned: {success}")

    # The solver should have reduced the distance significantly
    assert (
        final_distance < initial_distance
    ), f"IK should reduce distance (initial={initial_distance:.4f}, final={final_distance:.4f})"

    # For a simple 10cm movement, we should get very close
    assert (
        final_distance < 0.01
    ), f"IK should reach target within 1cm (actual distance: {final_distance:.4f}m)"


def test_atlas_foot_ik_constrained():
    """Test that foot IK respects ground constraints."""
    atlas = create_simple_atlas()
    setup_atlas_standing_pose(atlas)

    # Get foot body node
    l_foot_bn = atlas.getBodyNode("l_foot")
    assert l_foot_bn is not None

    # Create end effector
    tf_foot = dart.math.Isometry3()
    tf_foot.set_translation([0.186, 0.0, -0.08])
    l_foot = l_foot_bn.createEndEffector("l_foot")
    l_foot.setRelativeTransform(tf_foot)

    # Create target
    target = dart.dynamics.SimpleFrame(dart.dynamics.Frame.World(), "target")

    # Get current foot position
    initial_tf = l_foot.getWorldTransform()

    # Set target 5cm to the side (Y direction)
    target_tf = initial_tf
    target_pos = initial_tf.translation() + np.array([0.0, 0.05, 0.0])
    target_tf.set_translation(target_pos)
    target.setTransform(target_tf)

    # Set up IK with ground constraints
    ik = l_foot.getIK(True)
    ik.setTarget(target)
    ik.setActive(True)
    ik.useWholeBody()

    # Constrain Z (vertical) and roll/pitch
    ik.getErrorMethod().setLinearBounds(
        np.array([-np.inf, -np.inf, -1e-8]), np.array([np.inf, np.inf, 1e-8])
    )
    ik.getErrorMethod().setAngularBounds(
        np.array([-1e-8, -1e-8, -np.inf]), np.array([1e-8, 1e-8, np.inf])
    )

    # Solve
    solver = ik.getSolver()
    solver.setNumMaxIterations(100)

    initial_z = l_foot.getWorldTransform().translation()[2]
    success = ik.solveAndApply(True)
    final_z = l_foot.getWorldTransform().translation()[2]

    # Z coordinate should not change (ground constraint)
    assert (
        abs(final_z - initial_z) < 1e-6
    ), f"Foot Z should be constrained (initial={initial_z:.6f}, final={final_z:.6f})"


def test_atlas_hierarchical_ik():
    """Test hierarchical IK with multiple end effectors."""
    atlas = create_simple_atlas()
    setup_atlas_standing_pose(atlas)

    # Create end effectors for both hands
    l_hand_bn = atlas.getBodyNode("l_hand")
    r_hand_bn = atlas.getBodyNode("r_hand")

    # Left hand
    tf_hand = dart.math.Isometry3()
    tf_hand.set_translation([0.0, 0.12, 0.0])
    l_hand = l_hand_bn.createEndEffector("l_hand")
    l_hand.setDefaultRelativeTransform(tf_hand, True)

    # Right hand (mirror)
    tf_hand.set_translation([0.0, -0.12, 0.0])
    r_hand = r_hand_bn.createEndEffector("r_hand")
    r_hand.setDefaultRelativeTransform(tf_hand, True)

    # Create targets
    l_target = dart.dynamics.SimpleFrame(dart.dynamics.Frame.World(), "l_target")
    r_target = dart.dynamics.SimpleFrame(dart.dynamics.Frame.World(), "r_target")

    # Set targets 10cm forward
    l_initial_pos = l_hand.getWorldTransform().translation()
    r_initial_pos = r_hand.getWorldTransform().translation()

    l_target_tf = l_hand.getWorldTransform()
    l_target_tf.set_translation(l_initial_pos + np.array([0.1, 0.0, 0.0]))
    l_target.setTransform(l_target_tf)

    r_target_tf = r_hand.getWorldTransform()
    r_target_tf.set_translation(r_initial_pos + np.array([0.1, 0.0, 0.0]))
    r_target.setTransform(r_target_tf)

    # Set up IK for both hands
    l_ik = l_hand.getIK(True)
    l_ik.setTarget(l_target)
    l_ik.setActive(True)
    l_ik.useWholeBody()

    r_ik = r_hand.getIK(True)
    r_ik.setTarget(r_target)
    r_ik.setActive(True)
    r_ik.useWholeBody()

    # Use skeleton IK to solve both simultaneously
    skel_ik = atlas.getIK(True)

    # Solve
    success = skel_ik.solveAndApply(True)

    # Check both hands reached targets
    l_final_pos = l_hand.getWorldTransform().translation()
    r_final_pos = r_hand.getWorldTransform().translation()

    l_distance = np.linalg.norm(l_final_pos - l_target_tf.translation())
    r_distance = np.linalg.norm(r_final_pos - r_target_tf.translation())

    print(f"Left hand distance: {l_distance:.4f}m")
    print(f"Right hand distance: {r_distance:.4f}m")

    # Both should be close to targets
    assert (
        l_distance < 0.02
    ), f"Left hand should reach target (distance={l_distance:.4f}m)"
    assert (
        r_distance < 0.02
    ), f"Right hand should reach target (distance={r_distance:.4f}m)"


def test_ik_solver_properties():
    """Test that IK solver properties can be set and retrieved."""
    atlas = create_simple_atlas()
    l_hand_bn = atlas.getBodyNode("l_hand")
    l_hand = l_hand_bn.createEndEffector("l_hand")

    ik = l_hand.getIK(True)
    solver = ik.getSolver()

    # Test max iterations
    solver.setNumMaxIterations(200)
    assert solver.getNumMaxIterations() == 200

    # Test tolerance
    problem = ik.getProblem()
    assert problem is not None
    assert problem.getDimension() > 0


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
