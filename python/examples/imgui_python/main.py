"""
Python ImGui widget embedded in dartpy's ImGuiViewer.

The widget below lives inside the ImGui context created by DART and renders via
the same event loop.
"""

from __future__ import annotations

import sys
from typing import List

import dartpy as dart
import numpy as np

imgui = dart.gui.imgui


class PythonImGuiWidget(dart.gui.ImGuiWidget):
    def __init__(self, viewer: dart.gui.ImGuiViewer, world_node: dart.gui.RealTimeWorldNode):  # noqa: D401
        super().__init__()
        self._viewer = viewer
        self._node = world_node
        self._gravity = self._node.get_world().get_gravity().tolist()
        self._frames = 0

    def render(self) -> None:
        """Render a small control panel into the ImGui overlay."""
        self._frames += 1
        imgui.set_next_window_size(320, 180, imgui.FIRST_USE_EVER)
        imgui.set_next_window_bg_alpha(0.85)
        imgui.begin("Python Dear ImGui widget")

        if imgui.is_key_down(imgui.Key.Escape):
            self._viewer.close()

        simulating = self._viewer.is_simulating()
        if imgui.button("Pause" if simulating else "Play"):
            self._viewer.simulate(not simulating)
        imgui.same_line()
        if imgui.button("Step once"):
            # Single stepping is helpful when paused.
            self._viewer.simulate(False)
            self._node.get_world().step()

        imgui.separator()
        imgui.text(f"Sim time: {self._node.get_world().get_time():.3f} s")
        imgui.text(f"Frame rate: {imgui.get_io().framerate:.1f} fps")

        changed, self._gravity[1] = imgui.slider_float(
            "Gravity Y", self._gravity[1], -20.0, 5.0
        )
        if changed:
            self._node.get_world().set_gravity(
                np.array(self._gravity, dtype=float)
            )

        if imgui.button("Quit viewer"):
            self._viewer.close()

        imgui.separator()
        imgui.text(f"Frames rendered: {self._frames}")
        if self._frames > 900 and not self._viewer.is_closed():
            # Failsafe: auto-close after ~15s if nothing else happens
            self._viewer.close()

        imgui.end()


def make_world() -> dart.World:
    world = dart.World()
    world.set_gravity(np.array([0.0, -9.81, 0.0], dtype=float))

    ground = dart.Skeleton("ground")
    ground_joint, ground_body = ground.create_weld_joint_and_body_node_pair()
    ground_shape = dart.BoxShape([6.0, 0.2, 6.0])
    ground_node = ground_body.create_shape_node(ground_shape)
    ground_node.create_visual_aspect()
    ground_node.get_visual_aspect().set_rgba(
        np.array([0.75, 0.75, 0.75, 1.0], dtype=float)
    )
    ground_node.create_collision_aspect()
    ground_node.create_dynamics_aspect()
    ground_tf = dart.Isometry3()
    ground_tf.set_translation([0.0, -0.6, 0.0])
    ground_joint.set_transform_from_parent_body_node(ground_tf)
    world.add_skeleton(ground)

    skeleton = dart.Skeleton("box")
    joint, body = skeleton.create_free_joint_and_body_node_pair()
    shape = dart.BoxShape([0.4, 0.4, 0.4])
    shape_node = body.create_shape_node(shape)
    shape_node.create_visual_aspect()
    shape_node.get_visual_aspect().set_rgba(
        np.array([0.2, 0.6, 0.9, 1.0], dtype=float)
    )
    shape_node.create_collision_aspect()
    shape_node.create_dynamics_aspect()
    box_tf = dart.Isometry3()
    box_tf.set_translation([0.0, 0.6, 0.0])
    joint.set_transform(box_tf)
    world.add_skeleton(skeleton)

    return world


def main(argv: List[str] | None = None) -> int:
    _ = argv  # unused

    world = make_world()

    # RealTimeWorldNode drives the simulation loop.
    node = dart.gui.RealTimeWorldNode(world)

    viewer = dart.gui.ImGuiViewer(np.array([0.95, 0.95, 0.95, 1.0], dtype=float))
    # Keep ImGui callbacks on the main thread for Python.
    viewer.set_threading_model(dart.gui.Viewer.ThreadingModel.SingleThreaded)
    viewer.add_world_node(node)
    viewer.set_key_event_sets_done(
        dart.gui.GUIEventAdapter.KeySymbol.KEY_Escape.value
    )
    viewer.set_quit_event_sets_done(True)

    widget = PythonImGuiWidget(viewer, node)
    viewer.get_im_gui_handler().add_widget(widget, True)

    grid = dart.gui.GridVisual()
    grid.set_plane_type(dart.gui.GridVisual.PlaneType.ZX)
    grid.set_offset([0.0, -0.5, 0.0])
    grid.refresh()
    viewer.add_attachment(grid)

    viewer.set_clear_color(np.array([0.95, 0.95, 0.95, 1.0], dtype=float))
    viewer.set_up_view_in_window(50, 50, 800, 600)
    viewer.setup_default_lights()
    viewer.set_camera_home_position(
        [2.5, 1.5, 2.5],
        [0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    )
    viewer.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
