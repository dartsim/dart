#!/usr/bin/env python3
"""Generate deterministic box-pile scenes for the DART-vs-MuJoCo harness.

Each named preset ("pile-120", "pile-900", "dyn-stir-120", "ffi-overhead") is
built from a single seeded ``common.GeneratedSceneSpec`` + a materialized
``common.GeneratedLayout``. The same layout can be emitted either as an MJCF
XML string (for ``mujoco_runner.py``) or as a JSON body list (for
``dart_runner.py``), so both engines simulate the physically identical scene.

Layout algorithm (box pile): boxes are stacked in horizontal "sheets" above
the container and dropped under gravity. The footprint is capped at a 10x10
grid of boxes; additional objects add more stacked sheets instead of growing
the footprint further, so denser scenes (PILE-900 vs PILE-120) produce a
taller, more contact-rich collapse rather than a wide, mostly non-interacting
monolayer. Per-box positions get a small seeded random jitter so the initial
lattice is not perfectly regular (a perfectly regular lattice can produce
degenerate, simultaneous multi-contact configurations that are not
representative of a real pile).
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import common
import numpy as np

# Boxes per row/column of a single stacked "sheet"; extra objects beyond
# GRID_CAP**2 add more sheets rather than widening the footprint.
GRID_CAP = 10
GRID_GAP = 0.02  # extra clearance between adjacent grid cells, in meters

PRESETS: dict[str, dict] = {
    "pile-120": {"n_objects": 120, "stirrer": False},
    "pile-900": {"n_objects": 900, "stirrer": False},
    "dyn-stir-120": {"n_objects": 120, "stirrer": True},
}


def _pile_layout(
    scene_id: str,
    seed: int,
    n_objects: int,
    with_stirrer: bool,
    box_half_extent: float = 0.1,
    mass: float = 1.0,
    friction: float = 0.8,
    spacing_jitter: float = 0.02,
    drop_height: float = 0.5,
    timestep: float = 0.001,
) -> common.GeneratedLayout:
    rng = np.random.default_rng(seed)
    edge = 2.0 * box_half_extent
    pitch = edge + GRID_GAP
    base_side = max(1, min(GRID_CAP, math.ceil(math.sqrt(n_objects))))
    layer_capacity = base_side * base_side
    n_layers = math.ceil(n_objects / layer_capacity)

    jitter = rng.uniform(-spacing_jitter, spacing_jitter, size=(n_objects, 3))

    boxes: list[common.BoxBody] = []
    for i in range(n_objects):
        layer = i // layer_capacity
        idx_in_layer = i % layer_capacity
        ix = idx_in_layer % base_side
        iy = idx_in_layer // base_side
        x = (ix - (base_side - 1) / 2.0) * pitch + jitter[i, 0]
        y = (iy - (base_side - 1) / 2.0) * pitch + jitter[i, 1]
        z = drop_height + box_half_extent + layer * pitch + jitter[i, 2]
        boxes.append(
            common.BoxBody(
                name=f"box{i:04d}",
                half_extent=box_half_extent,
                mass=mass,
                friction=friction,
                position=(x, y, z),
            )
        )

    footprint_half_extent = base_side * pitch / 2.0 + 0.1
    wall_height = max(0.3, n_layers * pitch * 1.5)
    container = common.ContainerSpec(
        half_extent_xy=footprint_half_extent,
        wall_height=wall_height,
        wall_thickness=0.02,
        floor_half_thickness=0.02,
    )

    stirrer = None
    if with_stirrer:
        stirrer = common.StirrerSpec(
            radius=footprint_half_extent,
            half_height=max(pitch, wall_height * 0.3),
            half_thickness=0.02,
            friction=0.5,
            angular_velocity_rad_s=1.5,
            pivot_xyz=(0.0, 0.0, drop_height * 0.5 + box_half_extent),
        )

    spec = common.GeneratedSceneSpec(
        scene_id=scene_id,
        seed=seed,
        n_objects=n_objects,
        box_half_extent=box_half_extent,
        mass=mass,
        friction=friction,
        spacing_jitter=spacing_jitter,
        drop_height=drop_height,
        timestep=timestep,
        gravity=(0.0, 0.0, -9.81),
        container=container,
        stirrer=stirrer,
    )
    return common.GeneratedLayout(spec=spec, boxes=boxes)


def _ffi_overhead_layout(seed: int) -> common.GeneratedLayout:
    """A single free box with no gravity, no container, and (by
    construction, since it is the only body) never any contact. This isolates
    each engine's fixed per-step Python/FFI overhead from its contact-solver
    cost, giving a lower bound for the "wins/loses" comparisons on the other
    scenes.
    """
    box = common.BoxBody(
        name="box0000",
        half_extent=0.1,
        mass=1.0,
        friction=0.8,
        position=(0.0, 0.0, 1.0),
    )
    spec = common.GeneratedSceneSpec(
        scene_id="ffi-overhead",
        seed=seed,
        n_objects=1,
        box_half_extent=0.1,
        mass=1.0,
        friction=0.8,
        spacing_jitter=0.0,
        drop_height=1.0,
        timestep=0.001,
        gravity=(0.0, 0.0, 0.0),
        container=None,
        stirrer=None,
    )
    return common.GeneratedLayout(spec=spec, boxes=[box])


def build_layout(
    scene_id: str, seed: int, n_objects: int | None = None
) -> common.GeneratedLayout:
    if scene_id == "ffi-overhead":
        return _ffi_overhead_layout(seed)
    if scene_id not in PRESETS:
        raise SystemExit(
            f"Unknown scene id {scene_id!r}; choices: "
            f"{sorted([*PRESETS, 'ffi-overhead'])}"
        )
    preset = PRESETS[scene_id]
    n = n_objects if n_objects is not None else preset["n_objects"]
    return _pile_layout(scene_id, seed, n, preset["stirrer"])


def to_mjcf_xml(layout: common.GeneratedLayout) -> str:
    """Emit an MJCF XML string equivalent to ``layout`` for MuJoCo.

    Box sizes are half-extents already (MuJoCo's native convention), so no
    unit conversion is needed here; ``dart_runner.py`` doubles these values
    when constructing DART's full-extent ``BoxShape``.
    """
    spec = layout.spec
    lines = [f'<mujoco model="{spec.scene_id}">']
    lines.append('  <compiler angle="radian" inertiafromgeom="true"/>')
    lines.append(
        f'  <option timestep="{spec.timestep}" '
        f'gravity="{spec.gravity[0]} {spec.gravity[1]} {spec.gravity[2]}" '
        'integrator="Euler"/>'
    )
    lines.append("  <worldbody>")

    if spec.container is not None:
        c = spec.container
        he = c.half_extent_xy
        wt = c.wall_thickness
        wh = c.wall_height
        ft = c.floor_half_thickness
        friction = spec.friction
        lines.append(
            f'    <geom name="floor" type="box" pos="0 0 {-ft}" '
            f'size="{he} {he} {ft}" friction="{friction} 0.005 0.0001"/>'
        )
        walls = [
            ("wall_px", he + wt, 0.0, wt, he + wt),
            ("wall_nx", -(he + wt), 0.0, wt, he + wt),
            ("wall_py", 0.0, he + wt, he + wt, wt),
            ("wall_ny", 0.0, -(he + wt), he + wt, wt),
        ]
        for name, x, y, sx, sy in walls:
            lines.append(
                f'    <geom name="{name}" type="box" pos="{x} {y} {wh / 2.0}" '
                f'size="{sx} {sy} {wh / 2.0}" friction="{friction} 0.005 0.0001"/>'
            )

    for box in layout.boxes:
        x, y, z = box.position
        lines.append(f'    <body name="{box.name}" pos="{x} {y} {z}">')
        lines.append("      <freejoint/>")
        lines.append(
            f'      <geom type="box" size="{box.half_extent} {box.half_extent} '
            f'{box.half_extent}" mass="{box.mass}" '
            f'friction="{box.friction} 0.005 0.0001"/>'
        )
        lines.append("    </body>")

    if spec.stirrer is not None:
        s = spec.stirrer
        px, py, pz = s.pivot_xyz
        lines.append(f'    <body name="stirrer" mocap="true" pos="{px} {py} {pz}">')
        lines.append(
            f'      <geom type="box" size="{s.radius} {s.half_thickness} '
            f'{s.half_height}" friction="{s.friction} 0.005 0.0001"/>'
        )
        lines.append("    </body>")

    lines.append("  </worldbody>")
    lines.append("</mujoco>")
    return "\n".join(lines) + "\n"


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scene",
        required=True,
        choices=sorted([*PRESETS, "ffi-overhead"]),
        help="Which preset to generate.",
    )
    parser.add_argument("--seed", type=int, default=0, help="RNG seed.")
    parser.add_argument(
        "--n-objects",
        type=int,
        default=None,
        help="Override the preset's object count.",
    )
    parser.add_argument("--out", required=True, type=Path, help="Output file path.")
    parser.add_argument(
        "--format",
        choices=("json", "mjcf"),
        default=None,
        help="Output format; defaults to 'mjcf' if --out ends in .xml, else 'json'.",
    )
    return parser.parse_args(argv)


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    layout = build_layout(args.scene, args.seed, args.n_objects)

    fmt = args.format
    if fmt is None:
        fmt = "mjcf" if args.out.suffix == ".xml" else "json"

    if fmt == "json":
        common.save_generated_layout(layout, args.out)
    else:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(to_mjcf_xml(layout), encoding="utf-8")

    print(args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
