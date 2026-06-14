"""Ordered demo-scene catalog.

The vector order is the display order; categories appear in first-appearance
order. Add a scene by importing its ``SCENE`` constant and appending it here.
"""

from __future__ import annotations

from .runner import PythonDemoScene
from .scenes.avbd_empty_baseline import SCENE as AVBD_EMPTY_BASELINE
from .scenes.avbd_demo2d_cards import SCENE as AVBD_DEMO2D_CARDS
from .scenes.avbd_demo2d_dynamic_friction import (
    SCENE as AVBD_DEMO2D_DYNAMIC_FRICTION,
)
from .scenes.avbd_demo2d_fracture import SCENE as AVBD_DEMO2D_FRACTURE
from .scenes.avbd_demo2d_ground import SCENE as AVBD_DEMO2D_GROUND
from .scenes.avbd_demo2d_heavy_rope import SCENE as AVBD_DEMO2D_HEAVY_ROPE
from .scenes.avbd_demo2d_hanging_rope import SCENE as AVBD_DEMO2D_HANGING_ROPE
from .scenes.avbd_demo2d_joint_grid import SCENE as AVBD_DEMO2D_JOINT_GRID
from .scenes.avbd_demo2d_motor import SCENE as AVBD_DEMO2D_MOTOR
from .scenes.avbd_demo2d_net import SCENE as AVBD_DEMO2D_NET
from .scenes.avbd_demo2d_pyramid import SCENE as AVBD_DEMO2D_PYRAMID
from .scenes.avbd_demo2d_rod import SCENE as AVBD_DEMO2D_ROD
from .scenes.avbd_demo2d_rope import SCENE as AVBD_DEMO2D_ROPE
from .scenes.avbd_demo2d_soft_body import SCENE as AVBD_DEMO2D_SOFT_BODY
from .scenes.avbd_demo2d_spring import SCENE as AVBD_DEMO2D_SPRING
from .scenes.avbd_demo2d_spring_ratio import SCENE as AVBD_DEMO2D_SPRING_RATIO
from .scenes.avbd_demo2d_static_friction import (
    SCENE as AVBD_DEMO2D_STATIC_FRICTION,
)
from .scenes.avbd_demo2d_stack import SCENE as AVBD_DEMO2D_STACK
from .scenes.avbd_demo2d_stack_ratio import SCENE as AVBD_DEMO2D_STACK_RATIO
from .scenes.avbd_demo3d_bridge import SCENE as AVBD_DEMO3D_BRIDGE
from .scenes.avbd_demo3d_breakable import SCENE as AVBD_DEMO3D_BREAKABLE
from .scenes.avbd_demo3d_dynamic_friction import (
    SCENE as AVBD_DEMO3D_DYNAMIC_FRICTION,
)
from .scenes.avbd_demo3d_ground import SCENE as AVBD_DEMO3D_GROUND
from .scenes.avbd_demo3d_heavy_rope import SCENE as AVBD_DEMO3D_HEAVY_ROPE
from .scenes.avbd_demo3d_pyramid import SCENE as AVBD_DEMO3D_PYRAMID
from .scenes.avbd_demo3d_rope import SCENE as AVBD_DEMO3D_ROPE
from .scenes.avbd_demo3d_spring import SCENE as AVBD_DEMO3D_SPRING
from .scenes.avbd_demo3d_spring_ratio import SCENE as AVBD_DEMO3D_SPRING_RATIO
from .scenes.avbd_demo3d_static_friction import SCENE as AVBD_DEMO3D_STATIC_FRICTION
from .scenes.avbd_demo3d_stack import SCENE as AVBD_DEMO3D_STACK
from .scenes.avbd_demo3d_stack_ratio import SCENE as AVBD_DEMO3D_STACK_RATIO
from .scenes.avbd_demo3d_soft_body import SCENE as AVBD_DEMO3D_SOFT_BODY
from .scenes.avbd_articulated_breakable_joint import (
    SCENE as AVBD_ARTICULATED_BREAKABLE_JOINT,
)
from .scenes.avbd_articulated_fixed_pair_breakable_joint import (
    SCENE as AVBD_ARTICULATED_FIXED_PAIR_BREAKABLE_JOINT,
)
from .scenes.avbd_articulated_high_ratio_chain import (
    PAPER_SCALE_SCENE as AVBD_PAPER_SCALE_HIGH_RATIO_CHAIN,
    SCENE as AVBD_ARTICULATED_HIGH_RATIO_CHAIN,
)
from .scenes.avbd_articulated_motor_breakable_joint import (
    SCENE as AVBD_ARTICULATED_MOTOR_BREAKABLE_JOINT,
)
from .scenes.avbd_articulated_prismatic_motor import (
    SCENE as AVBD_ARTICULATED_PRISMATIC_MOTOR,
)
from .scenes.avbd_articulated_prismatic_pair_motor_breakable_joint import (
    SCENE as AVBD_ARTICULATED_PRISMATIC_PAIR_MOTOR_BREAKABLE_JOINT,
)
from .scenes.avbd_articulated_prismatic_motor_breakable_joint import (
    SCENE as AVBD_ARTICULATED_PRISMATIC_MOTOR_BREAKABLE_JOINT,
)
from .scenes.avbd_articulated_revolute_motor import (
    SCENE as AVBD_ARTICULATED_REVOLUTE_MOTOR,
)
from .scenes.avbd_articulated_spherical_breakable_joint import (
    SCENE as AVBD_ARTICULATED_SPHERICAL_BREAKABLE_JOINT,
)
from .scenes.avbd_articulated_spherical_pair_breakable_joint import (
    SCENE as AVBD_ARTICULATED_SPHERICAL_PAIR_BREAKABLE_JOINT,
)
from .scenes.avbd_articulated_world_revolute_motor_breakable_joint import (
    SCENE as AVBD_ARTICULATED_WORLD_REVOLUTE_MOTOR_BREAKABLE_JOINT,
)
from .scenes.avbd_rigid_breakable_joint import SCENE as AVBD_RIGID_BREAKABLE_JOINT
from .scenes.avbd_rigid_fixed_joint_contact import (
    SCENE as AVBD_RIGID_FIXED_JOINT_CONTACT,
)
from .scenes.avbd_rigid_prismatic_motor import SCENE as AVBD_RIGID_PRISMATIC_MOTOR
from .scenes.avbd_rigid_revolute_motor import SCENE as AVBD_RIGID_REVOLUTE_MOTOR
from .scenes.avbd_rigid_spherical_breakable_joint import (
    SCENE as AVBD_RIGID_SPHERICAL_BREAKABLE_JOINT,
)
from .scenes.articulated import SCENE as ARTICULATED
from .scenes.atlas_simbicon import SCENE as ATLAS_SIMBICON
from .scenes.contact import SCENE as CONTACT
from .scenes.diff_cartpole_trajopt import SCENE as DIFF_CARTPOLE_TRAJOPT
from .scenes.diff_drone_liftoff import SCENE as DIFF_DRONE_LIFTOFF
from .scenes.diff_pre_contact_surrogate import SCENE as DIFF_PRE_CONTACT_SURROGATE
from .scenes.diff_throw_to_target import SCENE as DIFF_THROW_TO_TARGET
from .scenes.floating_base import SCENE as FLOATING_BASE
from .scenes.gui_fidelity_debug_visuals import SCENE as GUI_FIDELITY_DEBUG_VISUALS
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
from .scenes.plan083_unified_newton_barrier import PLAN083_SCENES
from .scenes.planned import INVERSE_KINEMATICS as PLANNED_INVERSE_KINEMATICS
from .scenes.planned import MOBILE_MANIPULATION as PLANNED_MOBILE_MANIPULATION
from .scenes.planned import OPERATIONAL_SPACE_CONTROL as PLANNED_OSC
from .scenes.planned import SIMBICON_WALKING as PLANNED_SIMBICON_WALKING
from .scenes.replay_scrubber import SCENE as REPLAY_SCRUBBER
from .scenes.rigid_body import SCENE as RIGID_BODY
from .scenes.rigid_body_modes import SCENE as RIGID_BODY_MODES
from .scenes.rigid_collision_casts import SCENE as RIGID_COLLISION_CASTS
from .scenes.rigid_collision_query_options import (
    SCENE as RIGID_COLLISION_QUERY_OPTIONS,
)
from .scenes.rigid_contact_scale_budget import SCENE as RIGID_CONTACT_SCALE_BUDGET
from .scenes.rigid_contact_inspector import SCENE as RIGID_CONTACT_INSPECTOR
from .scenes.rigid_contact_manipulation import SCENE as RIGID_CONTACT_MANIPULATION
from .scenes.rigid_contact_solver_compare import (
    SCENE as RIGID_CONTACT_SOLVER_COMPARE,
)
from .scenes.rigid_distance_spring import SCENE as RIGID_DISTANCE_SPRING
from .scenes.rigid_executor_equivalence import SCENE as RIGID_EXECUTOR_EQUIVALENCE
from .scenes.rigid_external_loads import SCENE as RIGID_EXTERNAL_LOADS
from .scenes.rigid_frame_hierarchy import SCENE as RIGID_FRAME_HIERARCHY
from .scenes.rigid_fixed_joint import SCENE as RIGID_FIXED_JOINT
from .scenes.rigid_friction_threshold import SCENE as RIGID_FRICTION_THRESHOLD
from .scenes.rigid_free_flight import SCENE as RIGID_FREE_FLIGHT
from .scenes.rigid_ipc import SCENE as RIGID_IPC
from .scenes.rigid_ipc_edge_drop import SCENE as RIGID_IPC_EDGE_DROP
from .scenes.rigid_ipc_incline import SCENE as RIGID_IPC_INCLINE
from .scenes.rigid_ipc_pile import SCENE as RIGID_IPC_PILE
from .scenes.rigid_ipc_slide import SCENE as RIGID_IPC_SLIDE
from .scenes.rigid_ipc_stack_packet import HEAVY_SCENE as RIGID_IPC_HEAVY_STACK_PACKET
from .scenes.rigid_ipc_stack_packet import SCENE as RIGID_IPC_STACK_PACKET
from .scenes.rigid_ipc_tunnel import SCENE as RIGID_IPC_TUNNEL
from .scenes.rigid_joint_breakage import SCENE as RIGID_JOINT_BREAKAGE
from .scenes.rigid_joint_motor_limits import SCENE as RIGID_JOINT_MOTOR_LIMITS
from .scenes.rigid_joint_passive_parameters import (
    SCENE as RIGID_JOINT_PASSIVE_PARAMETERS,
)
from .scenes.rigid_kinematic_driver import SCENE as RIGID_KINEMATIC_DRIVER
from .scenes.rigid_kinematic_normal_push import SCENE as RIGID_KINEMATIC_NORMAL_PUSH
from .scenes.rigid_link_center_of_mass import SCENE as RIGID_LINK_CENTER_OF_MASS
from .scenes.rigid_link_jacobian import SCENE as RIGID_LINK_JACOBIAN
from .scenes.rigid_link_point_loads import SCENE as RIGID_LINK_POINT_LOADS
from .scenes.rigid_limited_joints import SCENE as RIGID_LIMITED_JOINTS
from .scenes.rigid_loop_closure import SCENE as RIGID_LOOP_CLOSURE
from .scenes.rigid_material_mixing import SCENE as RIGID_MATERIAL_MIXING
from .scenes.rigid_multibody_dynamics_terms import (
    SCENE as RIGID_MULTIBODY_DYNAMICS_TERMS,
)
from .scenes.rigid_multibody_solver_family import (
    SCENE as RIGID_MULTIBODY_SOLVER_FAMILY,
)
from .scenes.rigid_restitution_ladder import SCENE as RIGID_RESTITUTION_LADDER
from .scenes.rigid_screw_joint_pitch import SCENE as RIGID_SCREW_JOINT_PITCH
from .scenes.rigid_solver_compare import SCENE as RIGID_SOLVER_COMPARE
from .scenes.rigid_spin_roll_coupling import SCENE as RIGID_SPIN_ROLL_COUPLING
from .scenes.rigid_stack_stability import SCENE as RIGID_STACK_STABILITY
from .scenes.rigid_step_diagnostics import SCENE as RIGID_STEP_DIAGNOSTICS
from .scenes.rigid_timestep_sensitivity import SCENE as RIGID_TIMESTEP_SENSITIVITY
from .scenes.robot_puppets import ATLAS_PUPPET
from .scenes.robot_puppets import G1_PUPPET
from .scenes.robot_puppets import HUBO_PUPPET
from .scenes.variational_chain import SCENE as VARIATIONAL_CHAIN
from .scenes.variational_contact import SCENE as VARIATIONAL_CONTACT
from .scenes.variational_endpoint_loop_closure import (
    SCENE as VARIATIONAL_ENDPOINT_LOOP_CLOSURE,
)
from .scenes.variational_tumbler import SCENE as VARIATIONAL_TUMBLER
from .scenes.vbd_beam import SCENE as VBD_BEAM
from .scenes.vbd_cloth import SCENE as VBD_CLOTH
from .scenes.vbd_net import SCENE as VBD_NET
from .scenes.vbd_obstacle_drape import SCENE as VBD_OBSTACLE_DRAPE
from .scenes.vbd_self_fold import SCENE as VBD_SELF_FOLD
from .scenes.vbd_tilted_strand import SCENE as VBD_TILTED_STRAND


def make_demo_scenes() -> list[PythonDemoScene]:
    return [
        # World rigid-body dynamics. The opening rigid rows form the
        # user-facing visual-verification workflow: baseline dynamics,
        # body-mode semantics, no-contact initial-state free flight, frame
        # hierarchy/local-vs-world transform checks, external force/torque
        # response, link point-load semantics, time-step/gravity sensitivity,
        # step diagnostics, contact-scale frame-budget checks, matched contact
        # material response, pair-material mixing,
        # contact inspection, collision query filtering, collision casts,
        # solver comparison, executor equivalence,
        # contact-solver policy, multibody-link contact response,
        # friction threshold, spin/roll coupling,
        # resting-stack stability, contact-rich manipulation, prescribed
        # kinematic drivers,
        # joint-constraint checks, break-force lifecycle, distance springs,
        # joint motor limits, passive joint
        # parameters, screw-joint pitch coupling, generalized multibody
        # dynamics terms, link center-of-mass offsets, link-origin Jacobian
        # mapping, multibody solver-family selection, and loop-closure checks.
        # Broader World examples follow the curated verifier block.
        RIGID_BODY,
        RIGID_BODY_MODES,
        RIGID_FREE_FLIGHT,
        RIGID_FRAME_HIERARCHY,
        RIGID_EXTERNAL_LOADS,
        RIGID_LINK_POINT_LOADS,
        RIGID_TIMESTEP_SENSITIVITY,
        RIGID_STEP_DIAGNOSTICS,
        RIGID_CONTACT_SCALE_BUDGET,
        RIGID_RESTITUTION_LADDER,
        RIGID_MATERIAL_MIXING,
        RIGID_CONTACT_INSPECTOR,
        RIGID_COLLISION_QUERY_OPTIONS,
        RIGID_COLLISION_CASTS,
        RIGID_SOLVER_COMPARE,
        RIGID_EXECUTOR_EQUIVALENCE,
        RIGID_CONTACT_SOLVER_COMPARE,
        CONTACT,
        RIGID_FRICTION_THRESHOLD,
        RIGID_SPIN_ROLL_COUPLING,
        RIGID_STACK_STABILITY,
        RIGID_CONTACT_MANIPULATION,
        RIGID_KINEMATIC_DRIVER,
        RIGID_KINEMATIC_NORMAL_PUSH,
        RIGID_FIXED_JOINT,
        RIGID_JOINT_BREAKAGE,
        RIGID_DISTANCE_SPRING,
        RIGID_LIMITED_JOINTS,
        RIGID_JOINT_MOTOR_LIMITS,
        RIGID_JOINT_PASSIVE_PARAMETERS,
        RIGID_SCREW_JOINT_PITCH,
        RIGID_MULTIBODY_DYNAMICS_TERMS,
        RIGID_LINK_CENTER_OF_MASS,
        RIGID_LINK_JACOBIAN,
        RIGID_MULTIBODY_SOLVER_FAMILY,
        RIGID_LOOP_CLOSURE,
        ARTICULATED,
        FLOATING_BASE,
        GUI_FIDELITY_DEBUG_VISUALS,
        # AVBD rigid constraints (PLAN-104) start with fixed-joint rows in
        # contact, bounded revolute/prismatic motors, and public breakable-joint
        # lifecycles on free rigid bodies and articulated link/world endpoints.
        # Broader source-demo and soft/rigid coupling coverage remain tracked by
        # the AVBD dev task.
        AVBD_EMPTY_BASELINE,
        AVBD_DEMO2D_GROUND,
        AVBD_DEMO2D_MOTOR,
        AVBD_DEMO2D_DYNAMIC_FRICTION,
        AVBD_DEMO2D_STATIC_FRICTION,
        AVBD_DEMO2D_PYRAMID,
        AVBD_DEMO2D_CARDS,
        AVBD_DEMO2D_STACK,
        AVBD_DEMO2D_STACK_RATIO,
        AVBD_DEMO2D_ROD,
        AVBD_DEMO2D_SOFT_BODY,
        AVBD_DEMO2D_JOINT_GRID,
        AVBD_DEMO2D_ROPE,
        AVBD_DEMO2D_HEAVY_ROPE,
        AVBD_DEMO2D_HANGING_ROPE,
        AVBD_DEMO2D_SPRING,
        AVBD_DEMO2D_SPRING_RATIO,
        AVBD_DEMO2D_NET,
        AVBD_DEMO2D_FRACTURE,
        AVBD_DEMO3D_GROUND,
        AVBD_DEMO3D_DYNAMIC_FRICTION,
        AVBD_DEMO3D_STATIC_FRICTION,
        AVBD_DEMO3D_PYRAMID,
        AVBD_DEMO3D_ROPE,
        AVBD_DEMO3D_HEAVY_ROPE,
        AVBD_DEMO3D_SPRING,
        AVBD_DEMO3D_SPRING_RATIO,
        AVBD_DEMO3D_STACK,
        AVBD_DEMO3D_STACK_RATIO,
        AVBD_DEMO3D_SOFT_BODY,
        AVBD_DEMO3D_BRIDGE,
        AVBD_DEMO3D_BREAKABLE,
        AVBD_RIGID_FIXED_JOINT_CONTACT,
        AVBD_RIGID_REVOLUTE_MOTOR,
        AVBD_RIGID_PRISMATIC_MOTOR,
        AVBD_ARTICULATED_REVOLUTE_MOTOR,
        AVBD_ARTICULATED_PRISMATIC_MOTOR,
        AVBD_ARTICULATED_MOTOR_BREAKABLE_JOINT,
        AVBD_ARTICULATED_PRISMATIC_PAIR_MOTOR_BREAKABLE_JOINT,
        AVBD_ARTICULATED_PRISMATIC_MOTOR_BREAKABLE_JOINT,
        AVBD_ARTICULATED_WORLD_REVOLUTE_MOTOR_BREAKABLE_JOINT,
        AVBD_ARTICULATED_HIGH_RATIO_CHAIN,
        AVBD_PAPER_SCALE_HIGH_RATIO_CHAIN,
        AVBD_RIGID_BREAKABLE_JOINT,
        AVBD_RIGID_SPHERICAL_BREAKABLE_JOINT,
        AVBD_ARTICULATED_BREAKABLE_JOINT,
        AVBD_ARTICULATED_FIXED_PAIR_BREAKABLE_JOINT,
        AVBD_ARTICULATED_SPHERICAL_BREAKABLE_JOINT,
        AVBD_ARTICULATED_SPHERICAL_PAIR_BREAKABLE_JOINT,
        # High-value DART 6 examples that should return as World-native demos.
        # These lightweight placeholders keep the roadmap visible without
        # keeping legacy DART 6 scene implementations in the catalog.
        PLANNED_INVERSE_KINEMATICS,
        ATLAS_PUPPET,
        HUBO_PUPPET,
        G1_PUPPET,
        ATLAS_SIMBICON,
        PLANNED_SIMBICON_WALKING,
        PLANNED_OSC,
        PLANNED_MOBILE_MANIPULATION,
        # Rigid IPC (PLAN-082) contact-dynamics showcase, grouped by capability:
        # a drop, friction (flat + inclined), a multi-body pile, then the
        # intersection-free (no-tunneling) guarantee. Only scenes that run in
        # real time are registered first; stack packets are explicitly
        # capture-first and benchmark-adjacent so users can record the heavier
        # stress cases without promoting them into the numbered live workflow.
        RIGID_IPC,
        RIGID_IPC_SLIDE,
        RIGID_IPC_INCLINE,
        RIGID_IPC_EDGE_DROP,
        RIGID_IPC_PILE,
        RIGID_IPC_TUNNEL,
        RIGID_IPC_STACK_PACKET,
        RIGID_IPC_HEAVY_STACK_PACKET,
        # PLAN-083 unified Newton-barrier CPU corpus placeholders. These are
        # launchable py-demo rows with explicit smoke/visual/benchmark commands
        # but intentionally remain planned until runtime mixed stepping exists.
        *PLAN083_SCENES,
        REPLAY_SCRUBBER,
        VARIATIONAL_CHAIN,
        VARIATIONAL_TUMBLER,
        VARIATIONAL_CONTACT,
        LOOP_CLOSURE,
        VARIATIONAL_ENDPOINT_LOOP_CLOSURE,
        # Differentiable physics. Reproduces the paper's
        # gradient-based experiments as browsable, animated scenes. Each scene
        # degrades gracefully to an un-optimized rollout when the differentiable
        # bindings (DART_BUILD_DIFF) are absent, so the diff-OFF cycle smoke
        # still builds and renders them.
        DIFF_THROW_TO_TARGET,
        DIFF_CARTPOLE_TRAJOPT,
        DIFF_DRONE_LIFTOFF,
        DIFF_PRE_CONTACT_SURROGATE,
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
