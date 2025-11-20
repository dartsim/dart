"""
Whole-Body Inverse Kinematics Tutorial (Exercise Version)

This guided tutorial now covers:
- Lesson 1: Load Atlas and configure a standing pose
- Lesson 2: Create hand end effectors with offsets
- Lesson 3: Configure IK with tight error bounds
- Lesson 4: Enable drag-and-drop interaction in the viewer
- Lesson 5: Drive IK targets headlessly (no GUI)
- Lesson 6: Tune IK options for smoother trajectories
"""

import argparse
import numpy as np
import dartpy_nb as dart


class WholeBodyIKWorldNode(dart.gui.osg.RealTimeWorldNode):
    def __init__(self, world, robot):
        super().__init__(world)
        self.robot = robot

    def customPreStep(self):
        # IK updates happen when targets are moved by drag-and-drop
        pass


def parse_args():
    parser = argparse.ArgumentParser(description="Whole-body IK tutorial")
    parser.add_argument(
        "--mode",
        choices=["gui", "headless"],
        default="gui",
        help="Select interactive viewer or headless solver loop",
    )
    return parser.parse_args()


def load_atlas_robot():
    loader = dart.utils.DartLoader()
    atlas = loader.parseSkeleton("dart://sample/sdf/atlas/atlas_v3_no_head.urdf")

    if not atlas:
        raise RuntimeError("Failed to load Atlas robot!")

    # TODO(Lesson 1): Configure the standing pose and knee limits (same as before)

    return atlas


def create_hand_end_effector(hand_body_node, name):
    """Lesson 2: Create an end effector with palm offset."""
    # TODO: Create a dart.math.Isometry3(), set the translation offset, and
    # call setDefaultRelativeTransform on the created end effector.
    hand = hand_body_node.createEndEffector(name)
    return hand


def setup_hand_ik(hand):
    """Lesson 3: Configure IK settings."""
    ik = hand.getIK(True)

    # TODO: Set tight linear and angular bounds (±1e-8) on the error method.
    # TODO: Call useWholeBody() so all dependent DOFs contribute.
    # TODO: Configure solver max iterations and tolerance, then activate IK.

    return ik


def configure_smooth_motion(ik):
    """Lesson 6: Apply weights/clamps for smoother motion."""
    # TODO: Grab ik.getGradientMethod(), build a weights vector matching
    # len(ik.getDofs()), down-weight the first 6 DOFs (root), and set a
    # component-wise clamp to keep gradients bounded. Optional: if the
    # gradient method exposes setDampingCoefficient, increase it slightly.
    pass


def enable_gui_controls(world, atlas, left_hand, right_hand):
    node = WholeBodyIKWorldNode(world, atlas)
    viewer = dart.gui.osg.Viewer()
    viewer.addWorldNode(node)

    # TODO(Lesson 4): Call viewer.enableDragAndDrop on both hands to allow
    # interactive manipulation.

    viewer.setUpViewInWindow(0, 0, 1280, 960)
    viewer.setCameraHomePosition([3.0, 2.0, 2.0], [0.0, 0.5, 0.0], [0.0, 0.0, 1.0])
    viewer.run()


def run_headless_demo(world, atlas, left_hand, right_hand):
    """Lesson 5: Scripted IK loop without a GUI."""
    # TODO:
    #   * Create dart.dynamics.SimpleFrame targets for each hand and register
    #     them with the respective IK modules via setTarget().
    #   * Build a small trajectory (e.g., circular arc) by updating the target
    #     transforms each iteration.
    #   * Call atlas.getIK(True).solveAndApply(True) so both hands solve
    #     simultaneously, then world.step().
    #   * Print the resulting end-effector positions so users can verify the
    #     trajectory is followed.
    raise NotImplementedError("Lesson 5: implement run_headless_demo")


def build_world(atlas):
    world = dart.simulation.World()
    world.setGravity([0.0, -9.81, 0.0])
    world.addSkeleton(atlas)
    return world


def main():
    args = parse_args()
    atlas = load_atlas_robot()

    left_hand = create_hand_end_effector(atlas.getBodyNode("l_hand"), "l_hand")
    right_hand = create_hand_end_effector(atlas.getBodyNode("r_hand"), "r_hand")

    left_ik = setup_hand_ik(left_hand)
    right_ik = setup_hand_ik(right_hand)
    configure_smooth_motion(left_ik)
    configure_smooth_motion(right_ik)

    world = build_world(atlas)

    if args.mode == "gui":
        enable_gui_controls(world, atlas, left_hand, right_hand)
    else:
        run_headless_demo(world, atlas, left_hand, right_hand)


if __name__ == "__main__":
    main()
