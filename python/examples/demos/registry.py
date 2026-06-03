"""Ordered demo-scene catalog.

The vector order is the display order; categories appear in first-appearance
order. Add a scene by importing its ``SCENE`` constant and appending it here.
"""

from __future__ import annotations

from .runner import PythonDemoScene
from .scenes.articulated import SCENE as ARTICULATED
from .scenes.contact import SCENE as CONTACT
from .scenes.diff_cartpole_trajopt import SCENE as DIFF_CARTPOLE_TRAJOPT
from .scenes.diff_drone_liftoff import SCENE as DIFF_DRONE_LIFTOFF
from .scenes.diff_throw_to_target import SCENE as DIFF_THROW_TO_TARGET
from .scenes.floating_base import SCENE as FLOATING_BASE
from .scenes.ipc_deformable_capsule_rod import SCENE as IPC_DEFORMABLE_CAPSULE_ROD
from .scenes.ipc_deformable_cg_contact import SCENE as IPC_DEFORMABLE_CG_CONTACT
from .scenes.ipc_deformable_cg_solver import SCENE as IPC_DEFORMABLE_CG_SOLVER
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
from .scenes.ipc_deformable_plate_friction import SCENE as IPC_DEFORMABLE_PLATE_FRICTION
from .scenes.ipc_deformable_pt_particles import SCENE as IPC_DEFORMABLE_PT_PARTICLES
from .scenes.ipc_deformable_rod_friction import SCENE as IPC_DEFORMABLE_ROD_FRICTION
from .scenes.ipc_deformable_scripted_dirichlet import SCENE as IPC_DEFORMABLE_SCRIPTED
from .scenes.ipc_deformable_seg_strand import SCENE as IPC_DEFORMABLE_SEG_STRAND
from .scenes.ipc_deformable_trampoline import SCENE as IPC_DEFORMABLE_TRAMPOLINE
from .scenes.loop_closure import SCENE as LOOP_CLOSURE
from .scenes.planned import COLLISION_SANDBOX as PLANNED_COLLISION_SANDBOX
from .scenes.planned import INVERSE_KINEMATICS as PLANNED_INVERSE_KINEMATICS
from .scenes.planned import MOBILE_MANIPULATION as PLANNED_MOBILE_MANIPULATION
from .scenes.planned import OPERATIONAL_SPACE_CONTROL as PLANNED_OSC
from .scenes.planned import ROBOT_PUPPETS as PLANNED_ROBOT_PUPPETS
from .scenes.planned import SIMBICON_WALKING as PLANNED_SIMBICON_WALKING
from .scenes.rigid_body import SCENE as RIGID_BODY
from .scenes.rigid_fixed_joint import SCENE as RIGID_FIXED_JOINT
from .scenes.rigid_ipc import SCENE as RIGID_IPC
from .scenes.rigid_ipc_edge_drop import SCENE as RIGID_IPC_EDGE_DROP
from .scenes.rigid_ipc_incline import SCENE as RIGID_IPC_INCLINE
from .scenes.rigid_ipc_pile import SCENE as RIGID_IPC_PILE
from .scenes.rigid_ipc_slide import SCENE as RIGID_IPC_SLIDE
from .scenes.rigid_ipc_tunnel import SCENE as RIGID_IPC_TUNNEL
from .scenes.variational_chain import SCENE as VARIATIONAL_CHAIN
from .scenes.variational_contact import SCENE as VARIATIONAL_CONTACT
from .scenes.variational_tumbler import SCENE as VARIATIONAL_TUMBLER
from .scenes.vbd_beam import SCENE as VBD_BEAM
from .scenes.vbd_cloth import SCENE as VBD_CLOTH
from .scenes.vbd_net import SCENE as VBD_NET
from .scenes.vbd_obstacle_drape import SCENE as VBD_OBSTACLE_DRAPE
from .scenes.vbd_self_fold import SCENE as VBD_SELF_FOLD
from .scenes.vbd_tilted_strand import SCENE as VBD_TILTED_STRAND


def make_demo_scenes() -> list[PythonDemoScene]:
    return [
        # World rigid-body dynamics.
        ARTICULATED,
        FLOATING_BASE,
        CONTACT,
        RIGID_BODY,
        RIGID_FIXED_JOINT,
        # High-value DART 6 examples that should return as World-native demos.
        # These lightweight placeholders keep the roadmap visible without
        # keeping legacy DART 6 scene implementations in the catalog.
        PLANNED_INVERSE_KINEMATICS,
        PLANNED_SIMBICON_WALKING,
        PLANNED_OSC,
        PLANNED_ROBOT_PUPPETS,
        PLANNED_COLLISION_SANDBOX,
        PLANNED_MOBILE_MANIPULATION,
        # Rigid IPC (PLAN-082) contact-dynamics showcase, grouped by capability:
        # a drop, friction (flat + inclined), a multi-body pile, then the
        # intersection-free (no-tunneling) guarantee. Only scenes that run in
        # real time are registered; heavier contact scenes (a triangulated
        # sphere, a tight box stack) are intentionally omitted until the rigid
        # IPC performance work lands, since the solver currently runs at only a
        # few frames per second for those (they are covered by C++ regressions).
        RIGID_IPC,
        RIGID_IPC_SLIDE,
        RIGID_IPC_INCLINE,
        RIGID_IPC_EDGE_DROP,
        RIGID_IPC_PILE,
        RIGID_IPC_TUNNEL,
        VARIATIONAL_CHAIN,
        VARIATIONAL_TUMBLER,
        VARIATIONAL_CONTACT,
        LOOP_CLOSURE,
        # Differentiable physics. Reproduces the paper's
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
        # IPC Deformable -- its own dedicated category so the IPC
        # deformable-solver showcases group together.
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
        IPC_DEFORMABLE_CG_SOLVER,
        IPC_DEFORMABLE_CG_CONTACT,
        IPC_DEFORMABLE_OBJ_CLOTH,
        IPC_DEFORMABLE_CAPSULE_ROD,
        IPC_DEFORMABLE_SEG_STRAND,
        IPC_DEFORMABLE_PT_PARTICLES,
        IPC_DEFORMABLE_ROD_FRICTION,
        IPC_DEFORMABLE_PLATE_FRICTION,
        IPC_DEFORMABLE_SCRIPTED,
    ]
