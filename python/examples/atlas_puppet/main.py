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

# NOTE: This is a LIMITED implementation of the C++ atlas_puppet example.
# The full C++ version uses EndEffector and Support classes which are not
# currently bound in dartpy. See README.md for details.

import dartpy as dart
import numpy as np


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


def main():
    """Main function to run the Atlas visualization."""
    print("=" * 70)
    print("Atlas Puppet Example - LIMITED Python Implementation")
    print("=" * 70)
    print()
    print("NOTE: This is a simplified version of the C++ atlas_puppet example.")
    print("The full C++ version includes:")
    print("  - Interactive end effector control")
    print("  - Support polygon visualization")
    print("  - Balance constraint optimization")
    print("  - Whole-body inverse kinematics")
    print()
    print("These features require EndEffector and Support classes which are")
    print("not currently bound in dartpy. See README.md for details.")
    print()
    print("This version simply displays the Atlas robot in a default pose.")
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

    # Create world node for visualization
    node = dart.gui.osg.WorldNode(world)

    # Create viewer
    viewer = dart.gui.osg.Viewer()
    viewer.allowSimulation(False)  # Disable simulation - this is kinematic only
    viewer.addWorldNode(node)

    # Set up camera
    viewer.setUpViewInWindow(0, 0, 1280, 960)
    viewer.setCameraHomePosition(
        [5.34, 3.00, 2.41],
        [0.00, 0.00, 1.00],
        [-0.20, -0.08, 0.98]
    )

    # Run viewer
    viewer.run()


if __name__ == "__main__":
    main()
