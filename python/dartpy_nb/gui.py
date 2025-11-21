"""
Minimal headless GUI shims for dartpy_nb.

These stubs keep Python examples runnable without the legacy glut bindings.
They intentionally expose a small surface (Viewer, InteractiveFrame, RealTimeWorldNode,
SupportPolygonVisual, GUIEventAdapter/Handler) used by tutorials/examples.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List

import dartpy_nb as dart


class GUIEventAdapter:
    class EventType:
        KEYDOWN = "keydown"
        KEYUP = "keyup"

    def __init__(self, key: int = 0, event_type: str | None = None):
        self._key = key
        self._event_type = event_type or self.EventType.KEYDOWN

    def getEventType(self):
        return self._event_type

    def getKey(self):
        return self._key


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
    def __init__(self, *_):
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
        return None

    def setCameraHomePosition(self, *args, **kwargs):
        return None

    def run(self, steps: int = 1):
        for _ in range(steps):
            for node in list(self._world_nodes):
                node.customPreRefresh()
        print("[dartpy_nb.gui] Headless viewer run completed.")


class InteractiveFrame(dart.dynamics.SimpleFrame):
    def __new__(cls, parent, name: str = "", tf=None, *_, **__):
        if tf is None:
            tf = dart.math.Isometry3.Identity()
        return super().__new__(cls, parent, name, tf)

    def __init__(self, parent, name: str = "", tf=None, *_, **__):
        if tf is None:
            tf = dart.math.Isometry3.Identity()
        super().__init__(parent, name, tf)
