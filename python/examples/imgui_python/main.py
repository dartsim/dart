"""
Python ImGui widget embedded in dartpy's ImGuiViewer.

Prerequisite: install the Python Dear ImGui binding (PyPI package `imgui`)
against a version compatible with the ImGui bundled or used by DART
(e.g., `pip install "imgui>=2.0.0"`). The widget below lives inside the
ImGui context created by DART and renders via the same event loop.
"""

from __future__ import annotations

import sys
from typing import List

import dartpy as dart

try:
    import imgui  # type: ignore
except ImportError as exc:  # pragma: no cover - optional dependency
    raise SystemExit(
        "Missing dependency: install the 'imgui' package (PyPI) to run this example, "
        "e.g. `pip install imgui>=2.0.0`."
    ) from exc


class PythonImGuiWidget(dart.gui.ImGuiWidget):
    def __init__(self, viewer: dart.gui.ImGuiViewer, world_node: dart.gui.RealTimeWorldNode):  # noqa: D401
        super().__init__()
        self._viewer = viewer
        self._node = world_node
        self._gravity = list(self._node.getWorld().getGravity())

    def render(self) -> None:
        """Render a small control panel into the ImGui overlay."""
        imgui.set_next_window_size(320, 180, imgui.FIRST_USE_EVER)
        imgui.set_next_window_bg_alpha(0.85)
        imgui.begin("Python Dear ImGui widget")

        simulating = self._viewer.isSimulating()
        if imgui.button("Pause" if simulating else "Play"):
            self._viewer.simulate(not simulating)
        imgui.same_line()
        if imgui.button("Step once"):
            # Single stepping is helpful when paused.
            self._viewer.simulate(False)
            self._node.getWorld().step()

        imgui.separator()
        imgui.text(f"Sim time: {self._node.getWorld().getTime():.3f} s")
        imgui.text(f"Frame rate: {imgui.get_io().framerate:.1f} fps")

        changed, self._gravity[1] = imgui.slider_float(
            "Gravity Y", self._gravity[1], -20.0, 5.0
        )
        if changed:
            self._node.getWorld().setGravity(self._gravity)

        imgui.end()


def make_world() -> dart.simulation.World:
    world = dart.simulation.World()
    world.setGravity([0.0, -9.81, 0.0])
    return world


def main(argv: List[str] | None = None) -> int:
    _ = argv  # unused

    world = make_world()

    # RealTimeWorldNode drives the simulation loop.
    node = dart.gui.RealTimeWorldNode(world)

    viewer = dart.gui.ImGuiViewer([0.95, 0.95, 0.95, 1.0])
    viewer.addWorldNode(node)

    widget = PythonImGuiWidget(viewer, node)
    viewer.getImGuiHandler().addWidget(widget, True)

    grid = dart.gui.GridVisual()
    grid.setPlaneType(dart.gui.GridVisual.PlaneType.XZ)
    grid.setOffset([0.0, -0.5, 0.0])
    viewer.addAttachment(grid)

    viewer.setUpViewInWindow(50, 50, 800, 600)
    viewer.setCameraHomePosition(
        [2.5, 1.5, 2.5],
        [0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    )
    viewer.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
