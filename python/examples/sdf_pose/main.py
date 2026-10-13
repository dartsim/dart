"""Inspect SDF soft geometry and inertia expressed in rotated local frames.

Capture with body_frames and inertia_boxes using the agent-capture factory
python.examples.sdf_pose.main:make_world, or run this example directly.
"""

import dartpy as dart


def make_world():
    world = dart.utils.SdfParser.readWorld("dart://sample/sdf/test/pose_frames.world")
    if world is None:
        raise RuntimeError("failed to load sdf/test/pose_frames.world")
    return world


def main():
    viewer = dart.gui.Viewer()
    viewer.addWorldNode(dart.gui.WorldNode(make_world()))
    viewer.setUpViewInWindow(0, 0, 960, 600)
    viewer.setCameraHome([0, -5, 3], [0, 0, 1], [0, 0, 1])
    viewer.run()


if __name__ == "__main__":
    main()
