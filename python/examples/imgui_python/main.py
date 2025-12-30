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
        self._gravity = list(self._node.get_world().get_gravity())
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
        imgui.text(f"Sim time: {self._node.get_world().getTime():.3f} s")
        imgui.text(f"Frame rate: {imgui.get_io().framerate:.1f} fps")

        changed, self._gravity[1] = imgui.slider_float(
            "Gravity Y", self._gravity[1], -20.0, 5.0
        )
        if changed:
            self._node.get_world().set_gravity(self._gravity)

        if imgui.button("Quit viewer"):
            self._viewer.close()

        imgui.separator()
        imgui.text(f"Frames rendered: {self._frames}")
        if self._frames > 900 and not self._viewer.is_closed():
            # Failsafe: auto-close after ~15s if nothing else happens
            self._viewer.close()

        imgui.end()


def make_world() -> dart.simulation.World:
    world = dart.simulation.World()
    world.set_gravity([0.0, -9.81, 0.0])

    skeleton = dart.dynamics.Skeleton.create("box")
    _, body = skeleton.create_free_joint_and_body_node_pair()
    shape = dart.dynamics.BoxShape([0.4, 0.4, 0.4])
    body.create_shape_node(shape)
    world.add_skeleton(skeleton)

    return world


def main(argv: List[str] | None = None) -> int:
    _ = argv  # unused

    world = make_world()

    # RealTimeWorldNode drives the simulation loop.
    node = dart.gui.RealTimeWorldNode(world)

    viewer = dart.gui.ImGuiViewer(np.array([0.95, 0.95, 0.95, 1.0], dtype=float))
    viewer.add_world_node(node)

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
