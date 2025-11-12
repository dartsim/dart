"""
Whole-Body Inverse Kinematics Tutorial

This tutorial demonstrates how to use whole-body inverse kinematics (IK)
to control a humanoid robot's posture. You will learn how to:

1. Load a complex humanoid robot (Atlas)
2. Create end effectors for hands and feet
3. Set up IK targets and constraints
4. Use whole-body IK to achieve desired poses
5. Understand the importance of proper error method bounds

The tutorial is interactive - you can drag and drop end effectors
to see the robot adjust its posture in real-time.
"""

import dartpy as dart
import numpy as np


class WholeBodyIKWorldNode(dart.gui.osg.RealTimeWorldNode):
    """Custom world node for handling IK updates."""

    def __init__(self, world, robot):
        super().__init__(world)
        self.robot = robot

    def customPreStep(self):
        """Called before each simulation step."""
        # The IK is solved automatically when targets are moved
        # via drag-and-drop interaction
        pass


def load_atlas_robot():
    """Load the Atlas humanoid robot and configure initial pose."""
    loader = dart.utils.DartLoader()
    atlas = loader.parseSkeleton("dart://sample/sdf/atlas/atlas_v3_no_head.urdf")

    if not atlas:
        raise RuntimeError("Failed to load Atlas robot!")

    # Set up initial standing pose
    atlas.getDof("r_leg_hpy").setPosition(-45.0 * np.pi / 180.0)
    atlas.getDof("r_leg_kny").setPosition(90.0 * np.pi / 180.0)
    atlas.getDof("r_leg_aky").setPosition(-45.0 * np.pi / 180.0)
    atlas.getDof("l_leg_hpy").setPosition(-45.0 * np.pi / 180.0)
    atlas.getDof("l_leg_kny").setPosition(90.0 * np.pi / 180.0)
    atlas.getDof("l_leg_aky").setPosition(-45.0 * np.pi / 180.0)

    # Prevent knees from bending backwards
    atlas.getDof("r_leg_kny").setPositionLowerLimit(10.0 * np.pi / 180.0)
    atlas.getDof("l_leg_kny").setPositionLowerLimit(10.0 * np.pi / 180.0)

    return atlas


def create_hand_end_effector(hand_body_node, name):
    """Create an end effector for a hand with appropriate offset."""
    # Create transformation for the end effector
    # The offset moves the end effector point to the palm center
    hand_offset = dart.math.Isometry3()
    y_offset = 0.12 if name == "l_hand" else -0.12
    hand_offset.set_translation([0.0, y_offset, 0.0])

    # Create the end effector
    hand = hand_body_node.createEndEffector(name)
    hand.setDefaultRelativeTransform(hand_offset, True)

    return hand


def setup_hand_ik(hand):
    """Configure inverse kinematics for a hand end effector."""
    # Get or create the IK module for this end effector
    ik = hand.getIK(True)

    # CRITICAL: Set tight bounds for the error method
    # This ensures that any displacement from the target produces a non-zero error,
    # which allows the optimizer to find the gradient direction.
    # With infinite bounds (the default), the error would be zero, causing IK to fail.
    ik.getErrorMethod().setLinearBounds(
        np.array([-1e-8, -1e-8, -1e-8]), np.array([1e-8, 1e-8, 1e-8])
    )
    ik.getErrorMethod().setAngularBounds(
        np.array([-1e-8, -1e-8, -1e-8]), np.array([1e-8, 1e-8, 1e-8])
    )

    # Use whole-body IK: allows all dependent DOFs to be used
    ik.useWholeBody()

    # Configure the solver
    solver = ik.getSolver()
    solver.setNumMaxIterations(100)
    solver.setTolerance(1e-4)

    # Activate the IK module
    ik.setActive(True)

    print(f"Set up IK for end effector: {hand.getName()}")
    print("  - Using whole-body IK (all dependent DOFs)")
    print("  - Error method bounds: tight (1e-8)")
    print("  - Max iterations: 100")


def main():
    """Main tutorial function."""
    # Load the Atlas humanoid robot
    atlas = load_atlas_robot()

    print("=" * 50)
    print("  Whole-Body IK Tutorial")
    print("=" * 50)
    print(f"Loaded robot: {atlas.getName()}")
    print(f"Number of DOFs: {atlas.getNumDofs()}")
    print()

    # Create end effectors for both hands
    left_hand_body = atlas.getBodyNode("l_hand")
    right_hand_body = atlas.getBodyNode("r_hand")

    if not left_hand_body or not right_hand_body:
        raise RuntimeError("Failed to find hand body nodes!")

    left_hand = create_hand_end_effector(left_hand_body, "l_hand")
    right_hand = create_hand_end_effector(right_hand_body, "r_hand")

    # Set up IK for both hands
    print("Setting up whole-body IK...")
    setup_hand_ik(left_hand)
    setup_hand_ik(right_hand)
    print()

    # Create the world and add the robot
    world = dart.simulation.World()
    world.setGravity([0.0, -9.81, 0.0])
    world.addSkeleton(atlas)

    # Create the world node
    world_node = WholeBodyIKWorldNode(world, atlas)

    # Create the viewer
    viewer = dart.gui.osg.Viewer()
    viewer.addWorldNode(world_node)

    # Enable drag-and-drop for the end effectors
    # This allows you to interactively move the hands
    viewer.enableDragAndDrop(left_hand)
    viewer.enableDragAndDrop(right_hand)

    # Print instructions
    print("=" * 50)
    print("  Interactive Controls")
    print("=" * 50)
    print("Left-click and drag the colored spheres to move the hands")
    print("The robot will automatically adjust its posture using whole-body IK")
    print()
    print("Press Space to pause/resume simulation")
    print("=" * 50)

    # Set up the viewer window
    viewer.setUpViewInWindow(0, 0, 1280, 960)

    # Set camera position
    viewer.setCameraHomePosition([3.0, 2.0, 2.0], [0.0, 0.5, 0.0], [0.0, 0.0, 1.0])

    # Run the application
    viewer.run()


if __name__ == "__main__":
    main()
