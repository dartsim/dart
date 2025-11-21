"""
Lightweight pure-Python stubs for dartpy_nb.gui.osg.

These are *not* feature-complete renderers. They exist so examples that rely on
`dart.gui.osg` can execute in headless environments until GUI bindings are
ported to nanobind.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List

import dartpy_nb as dart


class GUIEventAdapter:
    class EventType:
        KEYDOWN = "keydown"
        KEYUP = "keyup"


class GUIActionAdapter:
    pass


class GUIEventHandler:
    def handle(self, ea: GUIEventAdapter, aa: GUIActionAdapter) -> bool:  # pragma: no cover - stub
        return False


class DragAndDropHandle:
    def __init__(self, frame):
        self.frame = frame


class RealTimeWorldNode:
    def __init__(self, world):
        self.world = world

    def customPreRefresh(self):  # pragma: no cover - stub
        pass


class SupportPolygonVisual:
    def __init__(self, skeleton, elevation):
        self.skeleton = skeleton
        self.elevation = elevation


@dataclass
class _InstructionBook:
    lines: List[str]

    def add(self, text: str):
        self.lines.append(text)

    def dump(self) -> str:
        return "".join(self.lines)


class Viewer:
    def __init__(self):
        self._world_nodes: List[RealTimeWorldNode] = []
        self._handlers: List[GUIEventHandler] = []
        self._instructions = _InstructionBook([])
        self._drag = []
        self._simulation_allowed = True

    def allowSimulation(self, allow: bool):
        self._simulation_allowed = allow

    def addWorldNode(self, node: RealTimeWorldNode):
        self._world_nodes.append(node)

    def addAttachment(self, attachment):
        # purely for API compatibility
        return attachment

    def addInstructionText(self, text: str):
        self._instructions.add(text)

    def getInstructions(self) -> str:
        return self._instructions.dump()

    def addEventHandler(self, handler: GUIEventHandler):
        self._handlers.append(handler)

    def enableDragAndDrop(self, frame):
        handle = DragAndDropHandle(frame)
        self._drag.append(handle)
        return handle

    def disableDragAndDrop(self, handle: DragAndDropHandle):
        self._drag = [h for h in self._drag if h is not handle]

    def setUpViewInWindow(self, *args, **kwargs):
        # no-op in headless mode
        return None

    def setCameraHomePosition(self, *args, **kwargs):
        return None

    def run(self, steps: int = 1):
        # Call the custom pre-refresh hook a few times to let demos progress.
        for _ in range(steps):
            for node in list(self._world_nodes):
                node.customPreRefresh()
        print("[dartpy_nb.gui.osg] Headless viewer run completed.")


class InteractiveFrame(dart.dynamics.SimpleFrame):
    def __new__(cls, parent, name: str = "", tf=None, *_, **__):
        if tf is None:
            tf = dart.math.Isometry3.Identity()
        return super().__new__(cls, parent, name, tf)

    def __init__(self, parent, name: str = "", tf=None, *_, **__):
        if tf is None:
            tf = dart.math.Isometry3.Identity()
        super().__init__(parent, name, tf)
