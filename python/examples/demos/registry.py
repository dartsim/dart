"""Python scene modules in the native C++ catalog's order."""

import dartpy as dart

from .scenes import (
    add_delete_skels,
    atlas_puppet,
    biped_stand,
    boxes,
    drag_and_drop,
    empty,
    hello_world,
    modern_humanoids,
    operational_space_control,
    rigid_chain,
    rigid_cubes,
    rigid_loop,
    rigid_shapes,
    simple_frames,
    simulation_event_handler,
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
        rigid_loop,
        biped_stand,
        operational_space_control,
        atlas_puppet,
    ]
    if hasattr(dart.utils, "ModelResourceRetriever"):
        modules.append(modern_humanoids)
    modules.append(drag_and_drop)
    return [scene for module in modules for scene in module.scenes()]
