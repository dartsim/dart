#!/usr/bin/env python3
"""Write validated AVBD avbd-demo3d Spring source-demo evidence packets."""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from write_avbd_demo2d_spring_packet import (  # noqa: E402
    SpringPacketSpec,
    main_for_spec,
)

SPRING3D_SOURCE_REFERENCE_ROW: dict[str, Any] = {
    "demo": "avbd-demo3d",
    "repository": "https://github.com/savant117/avbd-demo3d",
    "revision": "7701bd427d55",
    "dimension": 3,
    "scene_index": 7,
    "scene_name": "Spring",
    "scene_count": 14,
    "scene_builder": "sceneSpring",
    "solver_defaults": {
        "time_step": 1.0 / 60.0,
        "gravity_axis": "z",
        "gravity": -10.0,
        "iterations": 10,
    },
    "source_shapes": {
        "ground": {
            "count": 1,
            "size": (100.0, 100.0, 1.0),
            "density": 0.0,
            "friction": 0.5,
            "position": (0.0, 0.0, 0.0),
        },
        "anchor": {
            "count": 1,
            "size": (1.0, 1.0, 1.0),
            "density": 0.0,
            "friction": 0.5,
            "position": (0.0, 0.0, 14.0),
        },
        "block": {
            "count": 1,
            "size": (2.0, 2.0, 2.0),
            "density": 1.0,
            "friction": 0.5,
            "position": (0.0, 0.0, 8.0),
        },
    },
    "source_constraints": {
        "radial_distance_springs": {
            "count": 1,
            "parent_anchor": (0.0, 0.0, 0.0),
            "child_anchor": (0.0, 0.0, 0.0),
            "rest_length": 4.0,
            "stiffness": 100.0,
        },
    },
    "expected_counts": {
        "rigid_bodies": 3,
        "dynamic_bodies": 1,
        "static_bodies": 2,
        "joints": 0,
        "distance_springs": 1,
        "collision_shapes": 3,
    },
}

SPRING_RATIO3D_SOURCE_REFERENCE_ROW: dict[str, Any] = {
    "demo": "avbd-demo3d",
    "repository": "https://github.com/savant117/avbd-demo3d",
    "revision": "7701bd427d55",
    "dimension": 3,
    "scene_index": 8,
    "scene_name": "Spring Ratio",
    "scene_count": 14,
    "scene_builder": "sceneSpringsRatio",
    "solver_defaults": {
        "time_step": 1.0 / 60.0,
        "gravity_axis": "z",
        "gravity": -10.0,
        "iterations": 10,
    },
    "source_shapes": {
        "ground": {
            "count": 1,
            "size": (100.0, 100.0, 1.0),
            "density": 0.0,
            "friction": 0.5,
            "position": (0.0, 0.0, -10.0),
        },
        "links": {
            "count": 8,
            "size": (1.0, 0.75, 0.75),
            "dynamic_density": 1.0,
            "static_endpoint_density": 0.0,
            "friction": 0.5,
            "first_position": (-10.5, 0.0, 12.0),
            "x_spacing": 3.0,
            "static_endpoint_indices": (0, 7),
        },
    },
    "source_constraints": {
        "radial_distance_springs": {
            "count": 7,
            "parent_anchor": (0.5, 0.0, 0.0),
            "child_anchor": (-0.5, 0.0, 0.0),
            "rest_length": 3.0,
            "stiffness_pattern": (
                1.0e4,
                10.0,
                1.0e4,
                10.0,
                1.0e4,
                10.0,
                1.0e4,
            ),
            "high_stiffness_springs": 4,
            "low_stiffness_springs": 3,
        },
    },
    "expected_counts": {
        "rigid_bodies": 9,
        "dynamic_bodies": 6,
        "static_bodies": 3,
        "joints": 0,
        "distance_springs": 7,
        "collision_shapes": 9,
    },
}

SPRING3D_SPEC = SpringPacketSpec(
    output=Path(
        "docs/plans/104-vertex-block-descent-solver/" "avbd-demo3d-spring-packet.json"
    ),
    packet="avbd_demo3d_spring_source_demo",
    scene_id="avbd_demo3d_spring",
    benchmark_name="BM_AvbdDemo3dSpringStep",
    reference_scene_arg="spring",
    source_reference_row=SPRING3D_SOURCE_REFERENCE_ROW,
    benchmark_counts={
        "rigid_bodies": 3.0,
        "rigid_body_joints": 0.0,
        "distance_springs": 1.0,
        "collision_shapes": 3.0,
        "ignored_collision_pairs": 1.0,
        "source_scene_index": 7.0,
    },
    reference_counts={
        "rigid_bodies": 3,
        "dynamic_bodies": 1,
        "static_bodies": 2,
        "joints": 0,
        "distance_springs": 1,
        "collision_shapes": 3,
    },
    reference_floats={
        "min_friction": 0.5,
        "max_friction": 0.5,
        "min_spring_stiffness": 100.0,
        "max_spring_stiffness": 100.0,
        "spring_rest_length": 4.0,
    },
    remaining_corpus_gate="broader 3D spring-corpus coverage beyond this source row",
)

SPRING_RATIO3D_SPEC = SpringPacketSpec(
    output=Path(
        "docs/plans/104-vertex-block-descent-solver/"
        "avbd-demo3d-spring-ratio-packet.json"
    ),
    packet="avbd_demo3d_spring_ratio_source_demo",
    scene_id="avbd_demo3d_spring_ratio",
    benchmark_name="BM_AvbdDemo3dSpringRatioStep",
    reference_scene_arg="spring_ratio",
    source_reference_row=SPRING_RATIO3D_SOURCE_REFERENCE_ROW,
    benchmark_counts={
        "rigid_bodies": 9.0,
        "rigid_body_joints": 0.0,
        "distance_springs": 7.0,
        "collision_shapes": 9.0,
        "ignored_collision_pairs": 7.0,
        "source_scene_index": 8.0,
    },
    reference_counts={
        "rigid_bodies": 9,
        "dynamic_bodies": 6,
        "static_bodies": 3,
        "joints": 0,
        "distance_springs": 7,
        "collision_shapes": 9,
        "spring_link_count": 8,
        "high_stiffness_springs": 4,
        "low_stiffness_springs": 3,
    },
    reference_floats={
        "min_friction": 0.5,
        "max_friction": 0.5,
        "min_spring_stiffness": 10.0,
        "max_spring_stiffness": 1.0e4,
        "spring_rest_length": 3.0,
    },
    remaining_corpus_gate=(
        "broader 3D stiffness-ratio sweep coverage beyond this source row"
    ),
)


def main(argv: list[str]) -> int:
    return main_for_spec(argv, SPRING3D_SPEC)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
