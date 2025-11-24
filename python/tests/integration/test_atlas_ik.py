"""Integration tests for Atlas robot inverse kinematics."""

from pathlib import Path
import os
import sys


def _prepend_build_python():
    """Ensure the built dartpy extension is importable when running from source."""
    build_type = os.environ.get("BUILD_TYPE", "Release")
    pixi_env = os.environ.get("PIXI_ENVIRONMENT_NAME", "default")
    candidate = (
        Path(__file__).resolve().parents[3]
        / "build"
        / pixi_env
        / "cpp"
        / build_type
        / "python"
    )
    if candidate.exists():
        path_str = str(candidate)
        if path_str not in sys.path:
            sys.path.insert(0, path_str)


_prepend_build_python()

import dartpy as dart
import numpy as np
import pytest


def _dof(skeleton: dart.Skeleton, name: str):
  """Return DOF by name."""
  for dof in skeleton.get_dofs():
    if dof.get_name() == name:
      return dof
  raise KeyError(name)


def create_simple_atlas():
  """Create a simplified Atlas robot for testing."""
  urdf = dart.io.DartLoader()
  atlas = urdf.parse_skeleton("dart://sample/sdf/atlas/atlas_v3_no_head.urdf")
  return atlas


def setup_atlas_standing_pose(atlas):
  """Configure Atlas into a default standing pose."""
  # Right leg
  _dof(atlas, "r_leg_hpy").set_position(-45.0 * np.pi / 180.0)
  _dof(atlas, "r_leg_kny").set_position(90.0 * np.pi / 180.0)
  _dof(atlas, "r_leg_aky").set_position(-45.0 * np.pi / 180.0)

  # Left leg
  _dof(atlas, "l_leg_hpy").set_position(-45.0 * np.pi / 180.0)
  _dof(atlas, "l_leg_kny").set_position(90.0 * np.pi / 180.0)
  _dof(atlas, "l_leg_aky").set_position(-45.0 * np.pi / 180.0)

  # Prevent knees from bending backwards
  _dof(atlas, "r_leg_kny").set_position_lower_limit(10 * np.pi / 180.0)
  _dof(atlas, "l_leg_kny").set_position_lower_limit(10 * np.pi / 180.0)


def ensure_end_effector(atlas, body_name, ee_name=None):
    """Return an EndEffector for the given body, creating one if needed."""
    bn = atlas.get_body_node(body_name)
    assert bn is not None
    name = ee_name or body_name

    existing = atlas.get_end_effector(name)
    if existing is not None:
        return existing

    if bn.get_num_end_effectors() > 0:
        ee = bn.get_end_effector(0)
        if ee is not None:
            return ee

    return bn.create_end_effector(name)


def test_end_effector_creation_and_support():
    """End effectors can be created, configured, and retrieved from skeletons."""
    atlas = create_simple_atlas()
    bn = atlas.get_body_node("l_hand")
    ee_name = "custom_l_hand"

    effector = bn.create_end_effector(ee_name)
    assert effector is not None
    assert effector.get_name() == ee_name

    tf = dart.Isometry3()
    tf.set_translation([0.01, -0.02, 0.03])
    effector.set_default_relative_transform(tf, True)

    support = effector.create_support()
    support_points = [
        np.array([0.0, 0.0, 0.0]),
        np.array([0.05, 0.0, 0.0]),
        np.array([0.0, 0.05, 0.0]),
    ]
    support.set_geometry(support_points)

    stored = support.get_geometry()
    assert len(stored) == len(support_points)
    for expected, actual in zip(support_points, stored):
        assert np.allclose(np.asarray(actual).ravel(), expected.ravel())

    assert effector.has_support()
    retrieved = atlas.get_end_effector(ee_name)
    assert retrieved is effector

    effector.remove_support()
    assert not effector.has_support()
    recreated = effector.get_support(True)
    assert recreated is not None


def test_atlas_hand_ik_simple():
    """Test simple hand IK to reach forward."""
    atlas = create_simple_atlas()
    setup_atlas_standing_pose(atlas)

    # Get or create the left hand end effector
    l_hand = ensure_end_effector(atlas, "l_hand")

    # Set offset for palm center
    tf_hand = dart.Isometry3()
    tf_hand.set_translation([0.0, 0.12, 0.0])
    l_hand.set_default_relative_transform(tf_hand)

    # Create a simple frame target
    target = dart.SimpleFrame(dart.Frame.world(), "target")

    # Store initial hand position
    initial_pos = l_hand.get_world_transform().translation()

    # Set target 10cm forward from initial position
    target_tf = l_hand.get_world_transform()
    target_pos = initial_pos + np.array([0.1, 0.0, 0.0])
    target_tf.set_translation(target_pos)
    target.set_transform(target_tf)

    # Set up IK
    ik = l_hand.get_ik(True)
    ik.set_target(target)
    ik.set_active(True)

    # Use whole body IK
    ik.use_whole_body()

    # ✅ FIX: Set tight bounds so displacement produces error
    # With infinite bounds, the error is always zero (the bug)
    ik.get_error_method().set_linear_bounds(
        np.array([-1e-8, -1e-8, -1e-8]), np.array([1e-8, 1e-8, 1e-8])
    )
    ik.get_error_method().set_angular_bounds(
        np.array([-1e-8, -1e-8, -1e-8]), np.array([1e-8, 1e-8, 1e-8])
    )

    # Configure solver
    solver = ik.get_solver()
    solver.set_num_max_iterations(100)

    # Verify initial distance
    initial_distance = np.linalg.norm(
        l_hand.get_world_transform().translation() - target_pos
    )
    print(f"Initial distance to target: {initial_distance:.4f}m")
    assert initial_distance > 0.05, "Target should be at least 5cm away"

    # Solve IK - Use the SKELETON's hierarchical IK, not the individual IK module
    print("Solving IK using skeleton's hierarchical IK...")
    skel_ik = atlas.get_ik(True)  # Get skeleton's hierarchical IK
    success = skel_ik.solve_and_apply(True)

    # Check final distance
    final_pos = l_hand.get_world_transform().translation()
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

    # Get foot body node and its end effector
    l_foot_bn = atlas.get_body_node("l_foot")
    assert l_foot_bn is not None
    l_foot = ensure_end_effector(atlas, "l_foot")

    # Set offset
    tf_foot = dart.Isometry3()
    tf_foot.set_translation([0.186, 0.0, -0.08])
    l_foot.set_default_relative_transform(tf_foot)

    # Create target
    target = dart.SimpleFrame(dart.Frame.world(), "target")

    # Get current foot position
    initial_tf = l_foot.get_world_transform()

    # Set target 5cm to the side (Y direction)
    target_tf = initial_tf
    target_pos = initial_tf.translation() + np.array([0.0, 0.05, 0.0])
    target_tf.set_translation(target_pos)
    target.set_transform(target_tf)

    # Set up IK with ground constraints
    ik = l_foot.get_ik(True)
    ik.set_target(target)
    ik.set_active(True)
    ik.use_whole_body()

    # Constrain Z (vertical) and roll/pitch
    ik.get_error_method().set_linear_bounds(
        np.array([-np.inf, -np.inf, -1e-8]), np.array([np.inf, np.inf, 1e-8])
    )
    ik.get_error_method().set_angular_bounds(
        np.array([-1e-8, -1e-8, -np.inf]), np.array([1e-8, 1e-8, np.inf])
    )

    # Solve
    solver = ik.get_solver()
    solver.set_num_max_iterations(100)

    initial_z = l_foot.get_world_transform().translation()[2]
    success = ik.solve_and_apply(True)
    final_z = l_foot.get_world_transform().translation()[2]

    # Z coordinate should not change (ground constraint)
    assert (
        abs(final_z - initial_z) < 1e-6
    ), f"Foot Z should be constrained (initial={initial_z:.6f}, final={final_z:.6f})"


def test_atlas_hierarchical_ik():
    """Test hierarchical IK with multiple end effectors."""
    atlas = create_simple_atlas()
    setup_atlas_standing_pose(atlas)

    l_hand = ensure_end_effector(atlas, "l_hand")
    r_hand = ensure_end_effector(atlas, "r_hand")

    # Set offsets
    tf_hand_l = dart.Isometry3()
    tf_hand_l.set_translation([0.0, 0.12, 0.0])
    l_hand.set_default_relative_transform(tf_hand_l)

    tf_hand_r = dart.Isometry3()
    tf_hand_r.set_translation([0.0, -0.12, 0.0])
    r_hand.set_default_relative_transform(tf_hand_r)

    # Create targets
    l_target = dart.SimpleFrame(dart.Frame.world(), "l_target")
    r_target = dart.SimpleFrame(dart.Frame.world(), "r_target")

    # Set targets 10cm forward
    l_initial_pos = l_hand.get_world_transform().translation()
    r_initial_pos = r_hand.get_world_transform().translation()

    l_target_tf = l_hand.get_world_transform()
    l_target_tf.set_translation(l_initial_pos + np.array([0.1, 0.0, 0.0]))
    l_target.set_transform(l_target_tf)

    r_target_tf = r_hand.get_world_transform()
    r_target_tf.set_translation(r_initial_pos + np.array([0.1, 0.0, 0.0]))
    r_target.set_transform(r_target_tf)

    # Set up IK for both hands
    l_ik = l_hand.get_ik(True)
    l_ik.set_target(l_target)
    l_ik.set_active(True)
    l_ik.use_whole_body()

    r_ik = r_hand.get_ik(True)
    r_ik.set_target(r_target)
    r_ik.set_active(True)
    r_ik.use_whole_body()

    # Use skeleton IK to solve both simultaneously
    skel_ik = atlas.get_ik(True)

    # Solve
    success = skel_ik.solve_and_apply(True)

    # Check both hands reached targets
    l_final_pos = l_hand.get_world_transform().translation()
    r_final_pos = r_hand.get_world_transform().translation()

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
    l_hand = ensure_end_effector(atlas, "l_hand")

    ik = l_hand.get_ik(True)
    solver = ik.get_solver()

    # Test max iterations
    solver.set_num_max_iterations(200)
    assert solver.get_num_max_iterations() == 200

    # Test tolerance
    problem = ik.get_problem()
    assert problem is not None
    assert problem.get_dimension() > 0


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
