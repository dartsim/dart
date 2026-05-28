"""Ordered demo-scene catalog.

The vector order is the display order; categories appear in first-appearance
order. Add a scene by importing its ``SCENE`` constant and appending it here.
"""

from __future__ import annotations

from .runner import PythonDemoScene
from .scenes.arm_push_box import SCENE as ARM_PUSH_BOX
from .scenes.boxes import SCENE as BOXES
from .scenes.cartpole_gym_env import SCENE as CARTPOLE_GYM_ENV
from .scenes.cartpole_mpc import SCENE as CARTPOLE_MPC
from .scenes.hello_world import SCENE as HELLO_WORLD
from .scenes.kr5_arm import SCENE as KR5_ARM
from .scenes.legged_balance import SCENE as LEGGED_BALANCE
from .scenes.operational_space_control import SCENE as OSC
from .scenes.rigid_chain import SCENE as RIGID_CHAIN
from .scenes.sensor_descriptors import SCENE as SENSOR_DESCRIPTORS
from .scenes.sx_articulated import SCENE as SX_ARTICULATED
from .scenes.sx_contact import SCENE as SX_CONTACT
from .scenes.sx_floating_base import SCENE as SX_FLOATING_BASE


def make_demo_scenes() -> list[PythonDemoScene]:
    return [
        # Getting Started
        HELLO_WORLD,
        # Rigid Body
        BOXES,
        RIGID_CHAIN,
        # Rigid Body (Experimental)
        SX_ARTICULATED,
        SX_FLOATING_BASE,
        SX_CONTACT,
        # Robots
        KR5_ARM,
        # Control & IK
        OSC,
        # Control & Modern (PLAN-103 Phase 3)
        LEGGED_BALANCE,
        ARM_PUSH_BOX,
        CARTPOLE_GYM_ENV,
        CARTPOLE_MPC,
        SENSOR_DESCRIPTORS,
    ]
