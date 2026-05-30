"""Ordered demo-scene catalog.

The vector order is the display order; categories appear in first-appearance
order. Add a scene by importing its ``SCENE`` constant and appending it here.
"""

from __future__ import annotations

from .runner import PythonDemoScene
from .scenes.add_delete_skels import SCENE as ADD_DELETE_SKELS
from .scenes.arm_push_box import SCENE as ARM_PUSH_BOX
from .scenes.biped_stand import SCENE as BIPED_STAND
from .scenes.box_stacking import SCENE as BOX_STACKING
from .scenes.boxes import SCENE as BOXES
from .scenes.capsule_ground_contact import SCENE as CAPSULE_GROUND_CONTACT
from .scenes.cartpole_gym_env import SCENE as CARTPOLE_GYM_ENV
from .scenes.cartpole_mpc import SCENE as CARTPOLE_MPC
from .scenes.collision_sandbox import SCENE as COLLISION_SANDBOX
from .scenes.coupler_constraint import SCENE as COUPLER_CONSTRAINT
from .scenes.empty import SCENE as EMPTY
from .scenes.experimental_rigid_body_gui import SCENE as EXPERIMENTAL_RIGID_BODY_GUI
from .scenes.free_joint_cases import SCENE as FREE_JOINT_CASES
from .scenes.hardcoded_design import SCENE as HARDCODED_DESIGN
from .scenes.hello_world import SCENE as HELLO_WORLD
from .scenes.heightmap import SCENE as HEIGHTMAP
from .scenes.hybrid_dynamics import SCENE as HYBRID_DYNAMICS
from .scenes.kr5_arm import SCENE as KR5_ARM
from .scenes.lcp_physics import SCENE as LCP_PHYSICS
from .scenes.legged_balance import SCENE as LEGGED_BALANCE
from .scenes.mimic_pendulums import SCENE as MIMIC_PENDULUMS
from .scenes.mixed_chain import SCENE as MIXED_CHAIN
from .scenes.operational_space_control import SCENE as OSC
from .scenes.point_cloud import SCENE as POINT_CLOUD
from .scenes.polyhedron_visual import SCENE as POLYHEDRON_VISUAL
from .scenes.rigid_chain import SCENE as RIGID_CHAIN
from .scenes.rigid_cubes import SCENE as RIGID_CUBES
from .scenes.rigid_loop import SCENE as RIGID_LOOP
from .scenes.rigid_shapes import SCENE as RIGID_SHAPES
from .scenes.sensor_descriptors import SCENE as SENSOR_DESCRIPTORS
from .scenes.shapes import SCENE as SHAPES
from .scenes.simple_frames import SCENE as SIMPLE_FRAMES
from .scenes.soft_bodies import SCENE as SOFT_BODIES
from .scenes.sx_articulated import SCENE as SX_ARTICULATED
from .scenes.vehicle import SCENE as VEHICLE
from .scenes.sx_contact import SCENE as SX_CONTACT
from .scenes.sx_floating_base import SCENE as SX_FLOATING_BASE


def make_demo_scenes() -> list[PythonDemoScene]:
    return [
        # Getting Started
        HELLO_WORLD,
        EMPTY,
        # Visualization
        SHAPES,
        SIMPLE_FRAMES,
        POLYHEDRON_VISUAL,
        HEIGHTMAP,
        POINT_CLOUD,
        # Rigid Body
        BOXES,
        RIGID_CHAIN,
        RIGID_CUBES,
        RIGID_SHAPES,
        ADD_DELETE_SKELS,
        # Collision
        CAPSULE_GROUND_CONTACT,
        COLLISION_SANDBOX,
        # Constraints & Joints
        HARDCODED_DESIGN,
        BOX_STACKING,
        COUPLER_CONSTRAINT,
        MIMIC_PENDULUMS,
        RIGID_LOOP,
        FREE_JOINT_CASES,
        LCP_PHYSICS,
        # Soft Bodies
        MIXED_CHAIN,
        SOFT_BODIES,
        # Robots
        KR5_ARM,
        # Control & IK
        OSC,
        HYBRID_DYNAMICS,
        BIPED_STAND,
        VEHICLE,
        # Control & Modern (PLAN-103 Phase 3)
        LEGGED_BALANCE,
        ARM_PUSH_BOX,
        CARTPOLE_GYM_ENV,
        CARTPOLE_MPC,
        SENSOR_DESCRIPTORS,
        # Experimental physics solver (sx::World) — kept at the bottom so the
        # in-development surface doesn't push the canonical scenes off-screen.
        SX_ARTICULATED,
        SX_FLOATING_BASE,
        SX_CONTACT,
        EXPERIMENTAL_RIGID_BODY_GUI,
    ]
