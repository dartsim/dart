"""
Whole-Body Inverse Kinematics Tutorial

This tutorial demonstrates how to use whole-body inverse kinematics (IK)
to control a humanoid robot's posture. You will learn how to:

1. Load a complex humanoid robot (Atlas)
2. Create end effectors for hands and feet
3. Set up IK targets and constraints
4. Use whole-body IK to achieve desired poses
5. Understand the importance of proper error method bounds

The tutorial is interactive by default, and it now also exposes a headless
mode that repeatedly solves IK for scripted target poses.
"""

from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass
from typing import Iterable

import dartpy_nb as dart
import numpy as np


@dataclass
class HandTarget:
    frame: dart.dynamics.SimpleFrame
    rest_translation: np.ndarray


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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Whole-body IK tutorial with GUI or headless modes"
    )
    parser.add_argument(
        "--mode",
        choices=["gui", "headless"],
        default="gui",
        help="Select interactive viewer or headless scripted run",
    )
    parser.add_argument(
        "--headless-steps",
        type=int,
        default=120,
        help="Number of trajectory samples for headless mode",
    )
    parser.add_argument(
        "--trajectory-radius",
        type=float,
        default=0.08,
        help="Radius (meters) for hand trajectory in headless mode",
    )
    return parser.parse_args()


# snippet:py-load-atlas-start
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
# snippet:py-load-atlas-end


# snippet:py-create-hand-start
def create_hand_end_effector(hand_body_node, name):
    """Create an end effector for a hand with appropriate offset."""
    hand_offset = dart.math.Isometry3()
    y_offset = 0.12 if name == "l_hand" else -0.12
    hand_offset.set_translation([0.0, y_offset, 0.0])

    hand = hand_body_node.createEndEffector(name)
    hand.setDefaultRelativeTransform(hand_offset, True)

    return hand
# snippet:py-create-hand-end


# snippet:py-setup-ik-start
def setup_hand_ik(hand):
    """Configure inverse kinematics for a hand end effector."""
    ik = hand.getIK(True)

    ik.getErrorMethod().setLinearBounds(
        np.array([-1e-8, -1e-8, -1e-8]), np.array([1e-8, 1e-8, 1e-8])
    )
    ik.getErrorMethod().setAngularBounds(
        np.array([-1e-8, -1e-8, -1e-8]), np.array([1e-8, 1e-8, 1e-8])
    )

    ik.useWholeBody()

    solver = ik.getSolver()
    solver.setNumMaxIterations(100)
    solver.setTolerance(1e-4)

    ik.setActive(True)

    return ik
# snippet:py-setup-ik-end


# snippet:py-smooth-motion-start
def configure_smooth_motion(ik, root_weight: float = 0.25):
    """Bias IK solutions toward smooth trajectories."""
    grad = ik.getGradientMethod()
    dofs = ik.getDofs()
    if not dofs:
        return

    weights = np.ones(len(dofs))
    root_count = min(6, len(dofs))
    weights[:root_count] = root_weight
    grad.setComponentWeights(weights)
    grad.setComponentWiseClamp(0.15)

    if hasattr(grad, "setDampingCoefficient"):
        grad.setDampingCoefficient(2.0)
# snippet:py-smooth-motion-end


def create_hand_target(hand) -> HandTarget:
    target = dart.dynamics.SimpleFrame(
        dart.dynamics.Frame.World(),
        f"{hand.getName()}_target",
        dart.math.Isometry3.Identity(),
    )
    target.setTransform(hand.getWorldTransform())
    hand.getIK().setTarget(target)
    rest_translation = np.array(target.getTransform().translation()).reshape(3)
    return HandTarget(target, rest_translation)


def update_target_pose(target: HandTarget, offset: np.ndarray):
    tf = dart.math.Isometry3(target.frame.getTransform())
    tf.set_translation((target.rest_translation + offset).tolist())
    target.frame.setTransform(tf)


# snippet:py-gui-start
def run_gui_demo(world, atlas, left_hand, right_hand):
    node = WholeBodyIKWorldNode(world, atlas)
    viewer = dart.gui.osg.Viewer()
    viewer.addWorldNode(node)

    viewer.enableDragAndDrop(left_hand)
    viewer.enableDragAndDrop(right_hand)

    print("=" * 50)
    print("  Interactive Controls")
    print("=" * 50)
    print("Left-click and drag the colored spheres to move the hands")
    print("The robot will automatically adjust its posture using whole-body IK")
    print()
    print("Press Space to pause/resume simulation")
    print("Press R to reset the pose in the viewer toolbar")
    print("=" * 50)

    viewer.setUpViewInWindow(0, 0, 1280, 960)
    viewer.setCameraHomePosition([3.0, 2.0, 2.0], [0.0, 0.5, 0.0], [0.0, 0.0, 1.0])

    viewer.run()
# snippet:py-gui-end


# snippet:py-headless-start
def run_headless_demo(
    world,
    atlas,
    left_hand,
    right_hand,
    steps: int,
    radius: float,
):
    left_target = create_hand_target(left_hand)
    right_target = create_hand_target(right_hand)
    world.addSimpleFrame(left_target.frame)
    world.addSimpleFrame(right_target.frame)

    atlas_ik = atlas.getIK(True)
    atlas_ik.setActive(True)
    configure_smooth_motion(atlas_ik)
    solver = atlas_ik.getSolver()

    print("Running headless trajectory tracking...")
    for idx in range(steps):
        phase = (idx / steps) * 2.0 * math.pi
        left_offset = np.array([radius * math.cos(phase), 0.0, radius * math.sin(phase)])
        right_offset = np.array([
            0.5 * radius * math.cos(phase),
            0.5 * radius * math.sin(phase),
            0.0,
        ])

        update_target_pose(left_target, left_offset)
        update_target_pose(right_target, right_offset)

        start = time.perf_counter()
        success = atlas_ik.solveAndApply(True)
        solve_ms = (time.perf_counter() - start) * 1000.0
        world.step()

        joint_positions = atlas.getPositions().copy()
        l_target_pos = np.array(
            left_target.frame.getTransform().translation()
        ).reshape(3)
        r_target_pos = np.array(
            right_target.frame.getTransform().translation()
        ).reshape(3)
        l_pos = np.array(left_hand.getWorldTransform().translation()).reshape(3)
        r_pos = np.array(right_hand.getWorldTransform().translation()).reshape(3)
        left_err = np.linalg.norm(l_target_pos - l_pos)
        right_err = np.linalg.norm(r_target_pos - r_pos)
        iterations = (
            solver.getLastNumIterations()
            if hasattr(solver, "getLastNumIterations")
            else None
        )
        print(
            f"step {idx:03d} | success={success} | "
            f"q[0:6]={joint_positions[:6]} | "
            f"left_err={left_err:.4f}m right_err={right_err:.4f}m | "
            f"iter={iterations if iterations is not None else 'n/a'} "
            f"time={solve_ms:.2f}ms"
        )
        time.sleep(0.02)

    print("Headless run complete. Final joint configuration saved on the skeleton.")
# snippet:py-headless-end


def build_world(atlas):
    world = dart.simulation.World()
    world.setGravity([0.0, -9.81, 0.0])
    world.addSkeleton(atlas)
    return world


def main():
    args = parse_args()
    atlas = load_atlas_robot()

    print("=" * 50)
    print("  Whole-Body IK Tutorial")
    print("=" * 50)
    print(f"Loaded robot: {atlas.getName()}")
    print(f"Number of DOFs: {atlas.getNumDofs()}")
    print()

    left_hand = create_hand_end_effector(atlas.getBodyNode("l_hand"), "l_hand")
    right_hand = create_hand_end_effector(atlas.getBodyNode("r_hand"), "r_hand")

    lh_ik = setup_hand_ik(left_hand)
    rh_ik = setup_hand_ik(right_hand)
    configure_smooth_motion(lh_ik)
    configure_smooth_motion(rh_ik)

    world = build_world(atlas)

    if args.mode == "gui":
        run_gui_demo(world, atlas, left_hand, right_hand)
    else:
        run_headless_demo(
            world,
            atlas,
            left_hand,
            right_hand,
            steps=args.headless_steps,
            radius=args.trajectory_radius,
        )


if __name__ == "__main__":
    main()
