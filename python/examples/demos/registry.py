"""Python scene modules in demo catalog order."""

import dartpy as dart

from .scenes import (
    adaptive_soft_contact,
    add_delete_skels,
    atlas_puppet,
    atlas_simbicon,
    atlas_v5,
    biped_stand,
    box_stacking,
    boxes,
    contact_inverse_dynamics,
    drag_and_drop,
    dynamic_joint_constraints,
    empty,
    hardcoded_design,
    heightmap,
    hello_world,
    hybrid_dynamics,
    joint_constraints,
    mixed_chain,
    operational_space_control,
    point_cloud,
    rigid_chain,
    rigid_cubes,
    rigid_loop,
    rigid_shapes,
    simple_frames,
    simulation_event_handler,
    sleeping,
    soft_bodies,
    soft_cubes,
    soft_foot_simbicon,
    soft_open_chain,
    soft_worm,
    ssik_ik_gui,
    tinkertoy,
    unitree_g1,
    vehicle,
    wam_ikfast,
)


def make_demo_scenes():
    modules = [
        empty,
        hello_world,
        simple_frames,
        boxes,
        rigid_cubes,
        rigid_chain,
        add_delete_skels,
        simulation_event_handler,
        rigid_shapes,
        sleeping,
        hardcoded_design,
        rigid_loop,
        box_stacking,
        dynamic_joint_constraints,
        tinkertoy,
        joint_constraints,
        hybrid_dynamics,
        biped_stand,
        operational_space_control,
        contact_inverse_dynamics,
        wam_ikfast,
        atlas_puppet,
        atlas_simbicon,
        mixed_chain,
        adaptive_soft_contact,
        soft_worm,
        soft_foot_simbicon,
        soft_bodies,
        soft_cubes,
        soft_open_chain,
    ]
    if hasattr(dart._demos, "ArmSkeleton"):
        modules.insert(modules.index(wam_ikfast), ssik_ik_gui)
    if hasattr(dart.utils, "ModelResourceRetriever"):
        modules.extend([atlas_v5, unitree_g1])
    modules.extend([vehicle, heightmap])
    if hasattr(dart.dynamics, "VoxelGridShape"):
        modules.append(point_cloud)
    modules.append(drag_and_drop)
    return [scene for module in modules for scene in module.scenes()]
