"""Ordered demo-scene catalog.

The vector order is the display order; categories appear in first-appearance
order. Add a scene by importing its ``SCENE`` constant and appending it here.
"""

from __future__ import annotations

from .runner import PythonDemoScene
from .scenes.boxes import SCENE as BOXES
from .scenes.hello_world import SCENE as HELLO_WORLD
from .scenes.kr5_arm import SCENE as KR5_ARM
from .scenes.sx_articulated import SCENE as SX_ARTICULATED
from .scenes.sx_contact import SCENE as SX_CONTACT
from .scenes.sx_floating_base import SCENE as SX_FLOATING_BASE


def make_demo_scenes() -> list[PythonDemoScene]:
    return [
        # Getting Started
        HELLO_WORLD,
        # Rigid Body
        BOXES,
        # Rigid Body (Experimental)
        SX_ARTICULATED,
        SX_FLOATING_BASE,
        SX_CONTACT,
        # Robots
        KR5_ARM,
    ]
