"""Python scene modules in the native C++ catalog's order."""

import dartpy as dart

from .scenes import (
    adaptive_soft_contact,
    add_delete_skels,
    atlas_puppet,
    atlas_simbicon,
    atlas_v5,
    biped_stand,
    boxes,
    drag_and_drop,
    dynamic_joint_constraints,
    empty,
    hardcoded_design,
    hello_world,
    hybrid_dynamics,
    mixed_chain,
    operational_space_control,
    rigid_chain,
    rigid_cubes,
    rigid_loop,
    rigid_shapes,
    simple_frames,
    simulation_event_handler,
    soft_bodies,
    soft_cubes,
    soft_foot_simbicon,
    soft_open_chain,
    soft_worm,
    tinkertoy,
    unitree_g1,
    vehicle,
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
        hardcoded_design,
        rigid_loop,
        dynamic_joint_constraints,
        tinkertoy,
        hybrid_dynamics,
        biped_stand,
        operational_space_control,
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
    if hasattr(dart.utils, "ModelResourceRetriever"):
        modules.extend([atlas_v5, unitree_g1])
    modules.extend([vehicle, drag_and_drop])
    return [scene for module in modules for scene in module.scenes()]
