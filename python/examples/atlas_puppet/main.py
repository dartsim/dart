# Copyright (c) 2011-2025, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the following "BSD-style" License:
#   Redistribution and use in source and binary forms, with or
#   without modification, are permitted provided that the following
#   conditions are met:
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
#   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
#   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
#   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
#   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
#   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
#   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#   POSSIBILITY OF SUCH DAMAGE.

import dartpy as dart
import numpy as np


DISPLAY_ELEVATION = 0.05


class AtlasKeyboardHandler(dart.gui.osg.GUIEventHandler):
    """Custom keyboard event handler for Atlas control."""

    def __init__(self, teleop_world):
        super().__init__()
        self.teleop_world = teleop_world
        print("✓ AtlasKeyboardHandler created")

    def handle(self, ea, aa):
        """Handle keyboard events."""
        event_type = ea.getEventType()

        # Get key
        key = ea.getKey()

        # Convert key code to character
        if 32 <= key <= 126:  # Printable ASCII range
            key_char = chr(key).upper()
        else:
            return False

        # Handle key press
        if event_type == dart.gui.osg.EventType.KEYDOWN:
            return self.teleop_world.handle_key_press(key_char)

        # Handle key release
        elif event_type == dart.gui.osg.EventType.KEYUP:
            return self.teleop_world.handle_key_release(key_char)

        return False


class TeleoperationWorld(dart.gui.osg.RealTimeWorldNode):
    """Custom world node with keyboard teleoperation and continuous IK solving."""

    def __init__(self, world, atlas, interactive_targets):
        super().__init__(world)
        self.atlas = atlas
        self.interactive_targets = interactive_targets
        self.iter = 0

        # Movement state
        self.move_components = {
            "W": False,
            "S": False,
            "A": False,
            "D": False,
            "Q": False,
            "E": False,
            "F": False,
            "Z": False,
        }

        # Get end effectors for foot support control
        self.l_hand = atlas.getBodyNode("l_hand").getEndEffector(0)
        self.r_hand = atlas.getBodyNode("r_hand").getEndEffector(0)
        self.l_foot = atlas.getBodyNode("l_foot").getEndEffector(0)
        self.r_foot = atlas.getBodyNode("r_foot").getEndEffector(0)

        # Store rest configuration
        self.rest_config = atlas.getPositions().copy()

        # Constraint bounds
        inf = np.inf
        self.default_linear_bounds = np.full(3, inf)
        self.default_angular_bounds = np.full(3, inf)

        self.constrained_linear_bounds = np.array([inf, inf, 1e-8])
        self.constrained_angular_bounds = np.array([1e-8, 1e-8, inf])

        # Track foot constraint state
        self.l_foot_constrained = True
        self.r_foot_constrained = True

        print("✓ TeleoperationWorld initialized with keyboard control support")

    def handle_key_press(self, key_char):
        """Handle keyboard press events."""
        # Movement keys
        if key_char in self.move_components:
            self.move_components[key_char] = True
            return True

        # Toggle end effector targets (1-4)
        elif key_char in ["1", "2", "3", "4"]:
            idx = int(key_char) - 1
            ee = [self.l_hand, self.r_hand, self.l_foot, self.r_foot][idx]
            ee_names = ["Left Hand", "Right Hand", "Left Foot", "Right Foot"]

            # Toggle IK active state
            ik = ee.getIK()
            is_active = ik.isActive()
            ik.setActive(not is_active)

            status = "OFF" if is_active else "ON"
            print(f"{ee_names[idx]} IK: {status}")
            return True

        # Toggle left foot support (X)
        elif key_char == "X":
            self.l_foot_constrained = not self.l_foot_constrained
            ik = self.l_foot.getIK()
            if self.l_foot_constrained:
                ik.getErrorMethod().setLinearBounds(
                    -self.constrained_linear_bounds, self.constrained_linear_bounds
                )
                ik.getErrorMethod().setAngularBounds(
                    -self.constrained_angular_bounds, self.constrained_angular_bounds
                )
                print("Left foot support: ON (constrained to ground)")
            else:
                ik.getErrorMethod().setLinearBounds(
                    -self.default_linear_bounds, self.default_linear_bounds
                )
                ik.getErrorMethod().setAngularBounds(
                    -self.default_angular_bounds, self.default_angular_bounds
                )
                print("Left foot support: OFF (unconstrained)")
            return True

        # Toggle right foot support (C)
        elif key_char == "C":
            self.r_foot_constrained = not self.r_foot_constrained
            ik = self.r_foot.getIK()
            if self.r_foot_constrained:
                ik.getErrorMethod().setLinearBounds(
                    -self.constrained_linear_bounds, self.constrained_linear_bounds
                )
                ik.getErrorMethod().setAngularBounds(
                    -self.constrained_angular_bounds, self.constrained_angular_bounds
                )
                print("Right foot support: ON (constrained to ground)")
            else:
                ik.getErrorMethod().setLinearBounds(
                    -self.default_linear_bounds, self.default_linear_bounds
                )
                ik.getErrorMethod().setAngularBounds(
                    -self.default_angular_bounds, self.default_angular_bounds
                )
                print("Right foot support: OFF (unconstrained)")
            return True

        # Optimize posture (R)
        elif key_char == "R":
            print("Optimizing posture...")
            for _ in range(10):
                self.atlas.getIK(True).solveAndApply(True)
            print("Posture optimized!")
            return True

        # Reset to relaxed posture (T)
        elif key_char == "T":
            print("Resetting to relaxed posture...")
            self.atlas.setPositions(self.rest_config)
            # Reset targets to current end effector positions
            self.interactive_targets[0].setTransform(self.l_hand.getTransform())
            self.interactive_targets[1].setTransform(self.r_hand.getTransform())
            self.interactive_targets[2].setTransform(self.l_foot.getTransform())
            self.interactive_targets[3].setTransform(self.r_foot.getTransform())
            print("Reset complete!")
            return True

        return False

    def handle_key_release(self, key_char):
        """Handle keyboard release events."""
        if key_char in self.move_components:
            self.move_components[key_char] = False
            return True
        return False

    def customPreRefresh(self):
        """Solve IK and handle teleoperation before rendering each frame."""
        # Handle keyboard movement
        any_movement = any(self.move_components.values())

        if any_movement:
            # Get current root transform
            free_joint = self.atlas.getJoint(0)
            old_tf = free_joint.getRelativeTransform()
            new_tf = dart.math.Isometry3.Identity()

            # Get forward and left directions from current orientation
            forward = old_tf.rotation()[:, 0]  # X axis
            left = old_tf.rotation()[:, 1]  # Y axis
            up = np.array([0, 0, 1])  # Z axis (world up)

            # Movement parameters
            linear_step = 0.01
            elevation_step = 0.2 * linear_step
            rotational_step = 2.0 * np.pi / 180.0

            # Apply movements
            translation = np.zeros(3)
            rotation = dart.math.Isometry3.Identity()

            if self.move_components["W"]:
                translation += linear_step * forward
            if self.move_components["S"]:
                translation -= linear_step * forward
            if self.move_components["A"]:
                translation += linear_step * left
            if self.move_components["D"]:
                translation -= linear_step * left
            if self.move_components["F"]:
                translation += elevation_step * up
            if self.move_components["Z"]:
                translation -= elevation_step * up

            if self.move_components["Q"]:
                rot_matrix = dart.math.eulerXYZToMatrix([0, 0, rotational_step])
                rotation.set_rotation(rot_matrix)
            if self.move_components["E"]:
                rot_matrix = dart.math.eulerXYZToMatrix([0, 0, -rotational_step])
                rotation.set_rotation(rot_matrix)

            # Build new transform
            new_tf.set_translation(translation)
            new_tf.set_rotation(rotation.rotation() @ old_tf.rotation())
            new_tf.set_translation(new_tf.translation() + old_tf.translation())

            # Apply to free joint
            positions = np.zeros(6)
            positions[0:3] = new_tf.translation()
            rot_vec = dart.math.matrixToEulerXYZ(new_tf.rotation())
            positions[3:6] = rot_vec
            free_joint.setPositions(positions)

        # Solve IK
        skel_ik = self.atlas.getIK(True)
        skel_ik.solveAndApply(True)


def create_ground():
    """Create a ground plane for the robot to stand on."""
    ground = dart.dynamics.Skeleton("ground")
    thickness = 0.01

    # Create weld joint and body node
    joint, bn = ground.createWeldJointAndBodyNodePair()

    # Set the joint transform
    tf = dart.math.Isometry3()
    tf.set_translation([0, 0, -thickness / 2.0])
    joint.setTransformFromParentBodyNode(tf)

    # Create ground visual
    ground_shape = dart.dynamics.BoxShape([10, 10, thickness])
    shape_node = bn.createShapeNode(ground_shape)
    visual = shape_node.createVisualAspect()
    visual.setColor([0.2, 0.2, 1.0, 1.0])
    shape_node.createCollisionAspect()
    shape_node.createDynamicsAspect()

    return ground


def create_atlas():
    """Load the Atlas robot model."""
    urdf = dart.utils.DartLoader()
    atlas = urdf.parseSkeleton("dart://sample/sdf/atlas/atlas_v3_no_head.urdf")

    # Add a box to the root body for visualization
    scale = 0.25
    box_shape = dart.dynamics.BoxShape([scale * 1.0, scale * 1.0, scale * 0.5])

    tf = dart.math.Isometry3()
    tf.set_translation([0.0, 0.0, 0.1])

    shape_node = atlas.getBodyNode(0).createShapeNode(box_shape)
    visual = shape_node.createVisualAspect()
    visual.setColor([0.0, 0.0, 0.0, 1.0])
    shape_node.setRelativeTransform(tf)

    return atlas


def setup_start_configuration(atlas):
    """Configure Atlas into a default standing pose."""
    # Right leg
    atlas.getDof("r_leg_hpy").setPosition(-45.0 * np.pi / 180.0)
    atlas.getDof("r_leg_kny").setPosition(90.0 * np.pi / 180.0)
    atlas.getDof("r_leg_aky").setPosition(-45.0 * np.pi / 180.0)

    # Left leg
    atlas.getDof("l_leg_hpy").setPosition(-45.0 * np.pi / 180.0)
    atlas.getDof("l_leg_kny").setPosition(90.0 * np.pi / 180.0)
    atlas.getDof("l_leg_aky").setPosition(-45.0 * np.pi / 180.0)

    # Right arm
    atlas.getDof("r_arm_shx").setPosition(65.0 * np.pi / 180.0)
    atlas.getDof("r_arm_ely").setPosition(90.0 * np.pi / 180.0)
    atlas.getDof("r_arm_elx").setPosition(-90.0 * np.pi / 180.0)
    atlas.getDof("r_arm_wry").setPosition(65.0 * np.pi / 180.0)

    # Left arm
    atlas.getDof("l_arm_shx").setPosition(-65.0 * np.pi / 180.0)
    atlas.getDof("l_arm_ely").setPosition(90.0 * np.pi / 180.0)
    atlas.getDof("l_arm_elx").setPosition(90.0 * np.pi / 180.0)
    atlas.getDof("l_arm_wry").setPosition(65.0 * np.pi / 180.0)

    # Prevent knees from bending backwards
    atlas.getDof("r_leg_kny").setPositionLowerLimit(10 * np.pi / 180.0)
    atlas.getDof("l_leg_kny").setPositionLowerLimit(10 * np.pi / 180.0)


def setup_end_effectors(atlas):
    """Set up end effectors for hands and feet with IK."""
    # Root joint weights - very small to encourage using arm joints
    rootjoint_weights = 0.01 * np.ones(6)

    # Tight bounds for position tracking (fix for infinite bounds bug)
    # Using tight bounds (~1e-8) forces the error method to produce non-zero error
    # when the end effector is far from target
    linear_bounds = np.full(3, 1e-8)
    angular_bounds = np.full(3, 1e-8)

    # ----- LEFT HAND -----
    tf_hand = dart.math.Isometry3()
    tf_hand.set_translation([0.0009, 0.1254, 0.012])
    rot_matrix = dart.math.eulerXYZToMatrix([0, 0, 90.0 * np.pi / 180.0])
    tf_hand.set_rotation(rot_matrix)

    l_hand = atlas.getBodyNode("l_hand").createEndEffector("l_hand")
    l_hand.setDefaultRelativeTransform(tf_hand, True)

    # Create interactive target with larger size for visibility
    lh_target = dart.gui.osg.InteractiveFrame(
        dart.dynamics.Frame.World(),
        "lh_target",
        dart.math.Isometry3.Identity(),
        0.25,  # size_scale - make it bigger for visibility
        3.0,  # thickness_scale
    )

    # Set up IK
    l_hand_ik = l_hand.getIK(True)
    l_hand_ik.setTarget(lh_target)
    l_hand_ik.useWholeBody()

    # Set gradient weights
    l_hand_ik.getGradientMethod().setComponentWeights(rootjoint_weights)

    # Set bounds
    l_hand_ik.getErrorMethod().setLinearBounds(-linear_bounds, linear_bounds)
    l_hand_ik.getErrorMethod().setAngularBounds(-angular_bounds, angular_bounds)

    # ----- RIGHT HAND -----
    # Mirror the left hand transform
    tf_hand.set_translation([-0.0009, -0.1254, 0.012])
    rot_matrix_inv = rot_matrix.T
    tf_hand.set_rotation(rot_matrix_inv)

    r_hand = atlas.getBodyNode("r_hand").createEndEffector("r_hand")
    r_hand.setDefaultRelativeTransform(tf_hand, True)

    rh_target = dart.gui.osg.InteractiveFrame(
        dart.dynamics.Frame.World(),
        "rh_target",
        dart.math.Isometry3.Identity(),
        0.25,
        3.0,
    )

    r_hand_ik = r_hand.getIK(True)
    r_hand_ik.setTarget(rh_target)
    r_hand_ik.useWholeBody()
    r_hand_ik.getGradientMethod().setComponentWeights(rootjoint_weights)
    r_hand_ik.getErrorMethod().setLinearBounds(-linear_bounds, linear_bounds)
    r_hand_ik.getErrorMethod().setAngularBounds(-angular_bounds, angular_bounds)

    # ----- FEET WITH SUPPORT -----
    # Define support geometry for feet
    support = []
    sup_pos_x = 0.10 - 0.186
    sup_neg_x = -0.03 - 0.186
    sup_pos_y = 0.03
    sup_neg_y = -0.03
    support.append(np.array([sup_neg_x, sup_neg_y, 0.0]))
    support.append(np.array([sup_pos_x, sup_neg_y, 0.0]))
    support.append(np.array([sup_pos_x, sup_pos_y, 0.0]))
    support.append(np.array([sup_neg_x, sup_pos_y, 0.0]))

    # Foot transform
    tf_foot = dart.math.Isometry3()
    tf_foot.set_translation([0.186, 0.0, -0.08])

    # Constrain feet to ground
    linear_bounds[2] = 1e-8  # Z constrained
    angular_bounds[0] = 1e-8  # Roll constrained
    angular_bounds[1] = 1e-8  # Pitch constrained

    # LEFT FOOT
    l_foot = atlas.getBodyNode("l_foot").createEndEffector("l_foot")
    l_foot.setRelativeTransform(tf_foot)

    lf_target = dart.gui.osg.InteractiveFrame(
        dart.dynamics.Frame.World(),
        "lf_target",
        dart.math.Isometry3.Identity(),
        0.25,
        3.0,
    )

    l_foot_ik = l_foot.getIK(True)
    l_foot_ik.setTarget(lf_target)
    l_foot_ik.useWholeBody()
    l_foot_ik.getErrorMethod().setLinearBounds(-linear_bounds, linear_bounds)
    l_foot_ik.getErrorMethod().setAngularBounds(-angular_bounds, angular_bounds)

    # Create support
    l_support = l_foot.createSupport()
    l_support.setGeometry(support)

    # RIGHT FOOT
    r_foot = atlas.getBodyNode("r_foot").createEndEffector("r_foot")
    r_foot.setRelativeTransform(tf_foot)

    rf_target = dart.gui.osg.InteractiveFrame(
        dart.dynamics.Frame.World(),
        "rf_target",
        dart.math.Isometry3.Identity(),
        0.25,
        3.0,
    )

    r_foot_ik = r_foot.getIK(True)
    r_foot_ik.setTarget(rf_target)
    r_foot_ik.useWholeBody()
    r_foot_ik.getErrorMethod().setLinearBounds(-linear_bounds, linear_bounds)
    r_foot_ik.getErrorMethod().setAngularBounds(-angular_bounds, angular_bounds)

    # Create support
    r_support = r_foot.createSupport()
    r_support.setGeometry(support)

    print(f"✓ Created {atlas.getNumEndEffectors()} end effectors")
    print(f"✓ Left hand:  IK active with interactive target")
    print(f"✓ Right hand: IK active with interactive target")
    print(f"✓ Left foot:  IK active with support polygon ({len(support)} points)")
    print(f"✓ Right foot: IK active with support polygon ({len(support)} points)")

    # Return targets for drag-and-drop enabling
    return [lh_target, rh_target, lf_target, rf_target]


def main():
    """Main function to run the Atlas IK puppet demonstration."""
    print("=" * 70)
    print("Atlas Puppet Example - Full Python Implementation")
    print("=" * 70)
    print()
    print("This Python implementation demonstrates:")
    print("  ✓ Interactive end effector control with IK")
    print("  ✓ Whole-body inverse kinematics")
    print("  ✓ Support polygons for feet")
    print("  ✓ Custom gradient weights for IK optimization")
    print("  ✓ Constraint bounds for ground contact")
    print()
    print("Features implemented with new Python bindings:")
    print("  ✓ EndEffector class bindings")
    print("  ✓ Support class bindings")
    print("  ✓ BodyNode::createEndEffector()")
    print("  ✓ IK target configuration")
    print()
    print("Controls:")
    print("  - Drag the interactive frames to move end effectors")
    print("  - The robot will solve IK to reach targets")
    print("=" * 70)
    print()

    # Create world
    world = dart.simulation.World()

    # Create and add Atlas
    atlas = create_atlas()
    world.addSkeleton(atlas)

    # Create and add ground
    ground = create_ground()
    world.addSkeleton(ground)

    # Set up Atlas in default standing configuration
    setup_start_configuration(atlas)

    # Set up end effectors with IK and get targets
    interactive_targets = setup_end_effectors(atlas)

    # CRITICAL: Add interactive frames to the world so they're visible!
    for target in interactive_targets:
        world.addSimpleFrame(target)
    print(f"✓ Added {len(interactive_targets)} interactive frames to world")

    # Get end effectors
    l_hand = atlas.getBodyNode("l_hand").getEndEffector(0)
    r_hand = atlas.getBodyNode("r_hand").getEndEffector(0)
    l_foot = atlas.getBodyNode("l_foot").getEndEffector(0)
    r_foot = atlas.getBodyNode("r_foot").getEndEffector(0)

    # CRITICAL: Move atlas to the ground so feet touch the ground
    # This is from the C++ code - adjusts root Z position
    height_change = -r_foot.getWorldTransform().translation()[2]
    atlas.getDof(5).setPosition(height_change)
    print(f"✓ Adjusted robot height by {height_change:.3f}m to place feet on ground")

    # NOW set target transforms to match current end effector positions
    # CRITICAL: Must use getWorldTransform() not getTransform() to get world coordinates
    interactive_targets[0].setTransform(l_hand.getWorldTransform())  # lh_target
    interactive_targets[1].setTransform(r_hand.getWorldTransform())  # rh_target
    interactive_targets[2].setTransform(l_foot.getWorldTransform())  # lf_target
    interactive_targets[3].setTransform(r_foot.getWorldTransform())  # rf_target

    print(f"✓ Positioned interactive targets at end effector locations")

    print()
    print("Starting viewer...")
    print()

    # Create custom world node with teleoperation and IK solving
    node = TeleoperationWorld(world, atlas, interactive_targets)

    # Create viewer
    viewer = dart.gui.osg.Viewer()
    viewer.allowSimulation(False)  # Kinematics only - IK is solved in customPreRefresh
    viewer.addWorldNode(node)

    # Enable drag-and-drop for all interactive frames
    print("Enabling drag-and-drop for interactive targets...")
    for target in interactive_targets:
        viewer.enableDragAndDrop(target)
    print(f"✓ Drag-and-drop enabled for {len(interactive_targets)} interactive frames")
    print()

    # Add custom instructions for atlas_puppet
    viewer.addInstructionText(
        "Alt + Click:   Try to translate a body without changing its orientation\n"
    )
    viewer.addInstructionText(
        "Ctrl + Click:  Try to rotate a body without changing its translation\n"
    )
    viewer.addInstructionText(
        "Shift + Click: Move a body using only its parent joint\n"
    )
    viewer.addInstructionText(
        "1 -> 4:        Toggle the interactive target of an EndEffector\n"
    )
    viewer.addInstructionText("W A S D:       Move the robot around the scene\n")
    viewer.addInstructionText(
        "Q E:           Rotate the robot counter-clockwise and clockwise\n"
    )
    viewer.addInstructionText(
        "F Z:           Shift the robot's elevation up and down\n"
    )
    viewer.addInstructionText(
        "X C:           Toggle support on the left and right foot\n"
    )
    viewer.addInstructionText("R:             Optimize the robot's posture\n")
    viewer.addInstructionText(
        "T:             Reset the robot to its relaxed posture\n\n"
    )
    viewer.addInstructionText(
        "  Because this uses iterative Jacobian methods, the solver can get finicky,\n"
    )
    viewer.addInstructionText(
        "  and the robot can get tangled up. Use 'R' and 'T' keys when the robot is\n"
    )
    viewer.addInstructionText("  in a messy configuration\n\n")
    viewer.addInstructionText(
        "  The green polygon is the support polygon of the robot, and the blue/red ball is\n"
    )
    viewer.addInstructionText(
        "  the robot's center of mass. The green ball is the centroid of the polygon.\n\n"
    )
    viewer.addInstructionText(
        "Note that this is purely kinematic. Physical simulation is not allowed in this app.\n"
    )

    # Print instructions to console
    print(viewer.getInstructions())
    print()

    # Register keyboard event handler
    print("Registering keyboard event handler...")
    keyboard_handler = AtlasKeyboardHandler(node)
    viewer.addEventHandler(keyboard_handler)
    print("✓ Keyboard handler registered - W/A/S/D/Q/E/F/Z/X/C/R/T keys enabled!")
    print()

    # Set up camera
    viewer.setUpViewInWindow(0, 0, 1280, 960)
    viewer.setCameraHomePosition(
        [5.34, 3.00, 2.41], [0.00, 0.00, 1.00], [-0.20, -0.08, 0.98]
    )

    # Note: Python keyboard event handling
    print("=" * 70)
    print("Python Implementation Note:")
    print("=" * 70)
    print("Keyboard controls (W/A/S/D/Q/E/F/Z/X/C/R/T/1-4) require a custom")
    print("OSG event handler which cannot be easily subclassed from Python.")
    print()
    print("Workaround: Use mouse drag-and-drop for full interactivity!")
    print("  - Alt + Click: Drag to move end effectors")
    print("  - Ctrl + Click: Rotate end effectors")
    print()
    print("For full keyboard control, use the C++ version:")
    print("  build/default/cpp/Release/bin/atlas_puppet")
    print("=" * 70)
    print("\nStarting viewer - Press Ctrl+C to exit or close the window...")
    viewer.run()


if __name__ == "__main__":
    main()
