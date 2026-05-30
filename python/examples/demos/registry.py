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
from .scenes.diff_cartpole_trajopt import SCENE as DIFF_CARTPOLE_TRAJOPT
from .scenes.diff_drone_liftoff import SCENE as DIFF_DRONE_LIFTOFF
from .scenes.diff_throw_to_target import SCENE as DIFF_THROW_TO_TARGET
from .scenes.drag_and_drop import SCENE as DRAG_AND_DROP
from .scenes.empty import SCENE as EMPTY
from .scenes.experimental_rigid_body_gui import SCENE as EXPERIMENTAL_RIGID_BODY_GUI
from .scenes.free_joint_cases import SCENE as FREE_JOINT_CASES
from .scenes.hardcoded_design import SCENE as HARDCODED_DESIGN
from .scenes.heightmap import SCENE as HEIGHTMAP
from .scenes.hello_world import SCENE as HELLO_WORLD
from .scenes.hybrid_dynamics import SCENE as HYBRID_DYNAMICS
from .scenes.ipc_deformable_capsule_rod import SCENE as IPC_DEFORMABLE_CAPSULE_ROD
from .scenes.ipc_deformable_drape import SCENE as IPC_DEFORMABLE_DRAPE
from .scenes.ipc_deformable_fcr_twist import SCENE as IPC_DEFORMABLE_FCR_TWIST
from .scenes.ipc_deformable_fem_bar import SCENE as IPC_DEFORMABLE_FEM_BAR
from .scenes.ipc_deformable_fem_box import SCENE as IPC_DEFORMABLE_FEM_BOX
from .scenes.ipc_deformable_fem_buckle import SCENE as IPC_DEFORMABLE_FEM_BUCKLE
from .scenes.ipc_deformable_fem_drop import SCENE as IPC_DEFORMABLE_FEM_DROP
from .scenes.ipc_deformable_fem_msh import SCENE as IPC_DEFORMABLE_FEM_MSH
from .scenes.ipc_deformable_fem_sphere import SCENE as IPC_DEFORMABLE_FEM_SPHERE
from .scenes.ipc_deformable_fem_twist import SCENE as IPC_DEFORMABLE_FEM_TWIST
from .scenes.ipc_deformable_friction_slide import SCENE as IPC_DEFORMABLE_FRICTION_SLIDE
from .scenes.ipc_deformable_net import SCENE as IPC_DEFORMABLE_NET
from .scenes.ipc_deformable_obj_cloth import SCENE as IPC_DEFORMABLE_OBJ_CLOTH
from .scenes.ipc_deformable_pt_particles import SCENE as IPC_DEFORMABLE_PT_PARTICLES
from .scenes.ipc_deformable_scripted_dirichlet import SCENE as IPC_DEFORMABLE_SCRIPTED
from .scenes.ipc_deformable_seg_strand import SCENE as IPC_DEFORMABLE_SEG_STRAND
from .scenes.ipc_deformable_trampoline import SCENE as IPC_DEFORMABLE_TRAMPOLINE
from .scenes.joint_constraints import SCENE as JOINT_CONSTRAINTS
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
from .scenes.sx_contact import SCENE as SX_CONTACT
from .scenes.sx_floating_base import SCENE as SX_FLOATING_BASE
from .scenes.sx_rigid_ipc import SCENE as SX_RIGID_IPC
from .scenes.sx_rigid_ipc_incline import SCENE as SX_RIGID_IPC_INCLINE
from .scenes.sx_rigid_ipc_pile import SCENE as SX_RIGID_IPC_PILE
from .scenes.sx_rigid_ipc_slide import SCENE as SX_RIGID_IPC_SLIDE
from .scenes.sx_rigid_ipc_tunnel import SCENE as SX_RIGID_IPC_TUNNEL
from .scenes.sx_variational_chain import SCENE as SX_VARIATIONAL_CHAIN
from .scenes.sx_variational_tumbler import SCENE as SX_VARIATIONAL_TUMBLER
from .scenes.vbd_beam import SCENE as VBD_BEAM
from .scenes.vbd_cloth import SCENE as VBD_CLOTH
from .scenes.vbd_net import SCENE as VBD_NET
from .scenes.vbd_obstacle_drape import SCENE as VBD_OBSTACLE_DRAPE
from .scenes.vbd_self_fold import SCENE as VBD_SELF_FOLD
from .scenes.vbd_tilted_strand import SCENE as VBD_TILTED_STRAND
from .scenes.vehicle import SCENE as VEHICLE


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
        DRAG_AND_DROP,
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
        JOINT_CONSTRAINTS,
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
        # Rigid IPC (PLAN-082) contact-dynamics showcase, grouped by capability:
        # a drop, friction (flat + inclined), a multi-body pile, then the
        # intersection-free (no-tunneling) guarantee. Only scenes that run in
        # real time are registered; heavier contact scenes (a triangulated
        # sphere, a tight box stack) are intentionally omitted until the rigid
        # IPC performance work lands, since the solver currently runs at only a
        # few frames per second for those (they are covered by C++ regressions).
        SX_RIGID_IPC,
        SX_RIGID_IPC_SLIDE,
        SX_RIGID_IPC_INCLINE,
        SX_RIGID_IPC_PILE,
        SX_RIGID_IPC_TUNNEL,
        SX_VARIATIONAL_CHAIN,
        SX_VARIATIONAL_TUMBLER,
        # Differentiable physics (sx::World + sx.diff). Reproduces the paper's
        # gradient-based experiments as browsable, animated scenes. Each scene
        # degrades gracefully to an un-optimized rollout when the differentiable
        # bindings (DART_BUILD_DIFF) are absent, so the diff-OFF cycle smoke
        # still builds and renders them.
        DIFF_THROW_TO_TARGET,
        DIFF_CARTPOLE_TRAJOPT,
        DIFF_DRONE_LIFTOFF,
        # Vertex Block Descent (VBD) deformable scenes: mass-spring cloth/net, a
        # tetrahedral cantilever beam, the TinyVBD reference tilted strand (the
        # stiff, high-mass-ratio CPU parity scene), a cloth draping over a sphere
        # obstacle, and a surface self-contact showcase. The larger multi-body
        # paper scenes (216 squishy balls, 10368 models, tearing cloth) still
        # need broad inter-body contact and remain deferred.
        VBD_CLOTH,
        VBD_NET,
        VBD_BEAM,
        VBD_TILTED_STRAND,
        VBD_OBSTACLE_DRAPE,
        VBD_SELF_FOLD,
        # IPC Deformable (sx) — its own dedicated category so the IPC
        # deformable-solver showcases group together rather than mixing into
        # the general experimental scenes above.
        IPC_DEFORMABLE_NET,
        IPC_DEFORMABLE_DRAPE,
        IPC_DEFORMABLE_TRAMPOLINE,
        IPC_DEFORMABLE_FRICTION_SLIDE,
        IPC_DEFORMABLE_FEM_BAR,
        IPC_DEFORMABLE_FEM_TWIST,
        IPC_DEFORMABLE_FCR_TWIST,
        IPC_DEFORMABLE_FEM_DROP,
        IPC_DEFORMABLE_FEM_SPHERE,
        IPC_DEFORMABLE_FEM_BOX,
        IPC_DEFORMABLE_FEM_BUCKLE,
        IPC_DEFORMABLE_FEM_MSH,
        IPC_DEFORMABLE_OBJ_CLOTH,
        IPC_DEFORMABLE_CAPSULE_ROD,
        IPC_DEFORMABLE_SEG_STRAND,
        IPC_DEFORMABLE_PT_PARTICLES,
        IPC_DEFORMABLE_SCRIPTED,
    ]
