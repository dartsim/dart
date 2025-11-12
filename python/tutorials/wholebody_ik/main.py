"""
Whole-Body Inverse Kinematics Tutorial

This tutorial will teach you how to:
- Lesson 1: Load a humanoid robot and set up a standing pose
- Lesson 2: Create end effectors with proper offsets
- Lesson 3: Configure IK with proper error method bounds
- Lesson 4: Enable drag-and-drop interaction for IK targets

Follow the instructions in each lesson to complete the tutorial.
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
        pass


def load_atlas_robot():
    """Load the Atlas humanoid robot and configure initial pose.

    Lesson 1: Complete this function to load and configure the robot.
    """
    loader = dart.utils.DartLoader()
    atlas = loader.parseSkeleton("dart://sample/sdf/atlas/atlas_v3_no_head.urdf")

    if not atlas:
        raise RuntimeError("Failed to load Atlas robot!")

    # TODO: Set up initial standing pose by setting the following joint positions:
    #   - r_leg_hpy: -45 degrees
    #   - r_leg_kny: 90 degrees
    #   - r_leg_aky: -45 degrees
    #   - l_leg_hpy: -45 degrees
    #   - l_leg_kny: 90 degrees
    #   - l_leg_aky: -45 degrees
    # Hint: Use atlas.getDof("joint_name").setPosition(angle_in_radians)
    # Hint: Convert degrees to radians using np.pi / 180.0

    # TODO: Set knee joint lower limits to prevent backward bending (10 degrees)
    # Hint: Use atlas.getDof("r_leg_kny").setPositionLowerLimit(...)

    return atlas


def create_hand_end_effector(hand_body_node, name):
    """Create an end effector for a hand with appropriate offset.

    Lesson 2: Complete this function to create an end effector with proper offset.
    """
    # TODO: Create transformation for the end effector
    # The offset should move the end effector point to the palm center
    # Left hand offset: (0.0, 0.12, 0.0)
    # Right hand offset: (0.0, -0.12, 0.0)
    # Hint: Use dart.math.Isometry3() for the offset transformation
    # Hint: Use hand_offset.set_translation([x, y, z])

    hand = hand_body_node.createEndEffector(name)
    # TODO: Set the default relative transform with the offset
    # Hint: Use hand.setDefaultRelativeTransform(hand_offset, True)

    return hand


def setup_hand_ik(hand):
    """Configure inverse kinematics for a hand end effector.

    Lesson 3: Complete this function to configure IK with proper settings.
    """
    # Get or create the IK module for this end effector
    ik = hand.getIK(True)

    # TODO: Set tight bounds for the error method
    # This is CRITICAL! The error method only produces non-zero error when
    # displacement is outside the bounds. With infinite bounds, error is always zero!
    # Use bounds of [-1e-8, -1e-8, -1e-8] to [1e-8, 1e-8, 1e-8] for both linear and angular
    # Hint: Use ik.getErrorMethod().setLinearBounds(lower_array, upper_array)
    # Hint: Use ik.getErrorMethod().setAngularBounds(lower_array, upper_array)

    # TODO: Use whole-body IK to allow all dependent DOFs to be used
    # Hint: Use ik.useWholeBody()

    # TODO: Configure the solver parameters:
    #   - Max iterations: 100
    #   - Tolerance: 1e-4
    # Hint: solver = ik.getSolver()
    # Hint: solver.setNumMaxIterations(100)
    # Hint: solver.setTolerance(1e-4)

    # TODO: Activate the IK module
    # Hint: Use ik.setActive(True)

    print(f"Set up IK for end effector: {hand.getName()}")


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

    # Lesson 4: Enable drag-and-drop for interactive IK
    # TODO: Enable drag-and-drop for the left and right hand end effectors
    # This allows you to interactively move the hands and see the robot adjust
    # Hint: Use viewer.enableDragAndDrop(end_effector)

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
