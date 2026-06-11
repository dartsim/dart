"""Cycle smoke for the headless `dart-demos` Python runner (PLAN-103 Phase 1).

Asserts the registry has scenes, every scene can build and step a few frames
without crashing, and the runner's `--list` lists the catalog. The runner's
soft-fail path turns an unbuildable scene (e.g. a missing asset) into a logged
skip, so this test also passes when a robot URDF is unavailable; what we
guarantee is that the runner itself stays healthy.
"""

from __future__ import annotations

import importlib.util
import json
import pathlib
import re
import shlex
import signal
import sys
import time
from typing import Any

CaptureCommandSpec = tuple[str, int, int, int, bool]
RelatedEvidenceSpec = tuple[str, str, str, str, str]
CaptureFirstIpcSpec = tuple[str, str, str, str, str]

# Put python/ on sys.path so the demos package is importable.
_PYTHON_DIR = pathlib.Path(__file__).resolve().parents[2]
if str(_PYTHON_DIR) not in sys.path:
    sys.path.insert(0, str(_PYTHON_DIR))

import pytest

from examples.demos import make_demo_scenes, run  # noqa: E402
from examples.demos.runner import (  # noqa: E402
    CAPTURE_METRICS_INFO_KEY,
    RIGID_VISUAL_WORKFLOW_GUIDES,
    RIGID_VISUAL_WORKFLOW_LABELS,
    _RIGID_WORKFLOW_RELATED_EVIDENCE,
    _viewer_catalog_title,
)


def _capture_py_demo_module():
    root = pathlib.Path(__file__).resolve().parents[3]
    spec = importlib.util.spec_from_file_location(
        "capture_py_demo", root / "scripts" / "capture_py_demo.py"
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _read_rigid_visual_workflow_details() -> list[tuple[int, str, str, str]]:
    root = pathlib.Path(__file__).resolve().parents[3]
    sidecar = (
        root
        / "docs"
        / "plans"
        / "103-examples-strategy"
        / "rigid-body-visual-verification.md"
    )
    rows: list[tuple[int, str, str, str]] = []
    for line in sidecar.read_text(encoding="utf-8").splitlines():
        if not line.startswith("| "):
            continue
        cells = [cell.strip() for cell in line.split("|")]
        if len(cells) < 9 or not cells[1].isdigit():
            continue
        rows.append((int(cells[1]), cells[2].strip("`"), cells[3], cells[8]))
    return rows


def _read_rigid_visual_workflow_rows() -> list[tuple[int, str]]:
    return [
        (order, scene_id)
        for order, scene_id, _question, _scope in _read_rigid_visual_workflow_details()
    ]


def _read_rigid_visual_related_evidence_rows() -> list[RelatedEvidenceSpec]:
    root = pathlib.Path(__file__).resolve().parents[3]
    sidecar = (
        root
        / "docs"
        / "plans"
        / "103-examples-strategy"
        / "rigid-body-visual-verification.md"
    )
    rows: list[RelatedEvidenceSpec] = []
    in_section = False
    for line in sidecar.read_text(encoding="utf-8").splitlines():
        if line == "## Related Evidence Routes":
            in_section = True
            continue
        if in_section and line.startswith("## "):
            break
        if not in_section or not line.startswith("| `"):
            continue
        cells = [cell.strip() for cell in line.split("|")]
        if len(cells) < 7:
            continue
        rows.append(
            (
                cells[1].strip("`"),
                cells[2].strip("`"),
                cells[3],
                cells[4],
                cells[5],
            )
        )
    return rows


def _read_rigid_visual_capture_first_ipc_rows() -> list[CaptureFirstIpcSpec]:
    root = pathlib.Path(__file__).resolve().parents[3]
    sidecar = (
        root
        / "docs"
        / "plans"
        / "103-examples-strategy"
        / "rigid-body-visual-verification.md"
    )
    rows: list[CaptureFirstIpcSpec] = []
    in_section = False
    for line in sidecar.read_text(encoding="utf-8").splitlines():
        if line == "## Capture-First Rigid IPC Packets":
            in_section = True
            continue
        if in_section and line.startswith("## "):
            break
        if not in_section or not line.startswith("| `"):
            continue
        cells = [cell.strip() for cell in line.split("|")]
        if len(cells) < 7:
            continue
        rows.append(
            (
                cells[1].strip("`"),
                cells[2],
                cells[3],
                cells[4],
                cells[5],
            )
        )
    return rows


def _capture_spec_from_command(command: str) -> CaptureCommandSpec:
    tokens = shlex.split(command)

    def value_after(flag: str) -> str:
        assert flag in tokens
        index = tokens.index(flag)
        assert index + 1 < len(tokens)
        return tokens[index + 1]

    return (
        value_after("--scene"),
        int(value_after("--frames")),
        int(value_after("--width")),
        int(value_after("--height")),
        "--show-ui" in tokens,
    )


def _read_rigid_visual_table_capture_specs() -> list[CaptureCommandSpec]:
    root = pathlib.Path(__file__).resolve().parents[3]
    sidecar = (
        root
        / "docs"
        / "plans"
        / "103-examples-strategy"
        / "rigid-body-visual-verification.md"
    )
    rows: list[CaptureCommandSpec] = []
    for line in sidecar.read_text(encoding="utf-8").splitlines():
        if not line.startswith("| "):
            continue
        cells = [cell.strip() for cell in line.split("|")]
        if len(cells) < 8 or not cells[1].isdigit():
            continue
        command = cells[6]
        if command.startswith("`") and command.endswith("`"):
            command = command[1:-1]
        rows.append(_capture_spec_from_command(command))
    return rows


def _read_rigid_visual_readme_workflow_ids() -> list[str]:
    root = pathlib.Path(__file__).resolve().parents[3]
    readme = root / "python" / "examples" / "demos" / "README.md"
    rows: list[str] = []
    in_workflow_section = False
    for line in readme.read_text(encoding="utf-8").splitlines():
        if line.startswith("## "):
            if in_workflow_section:
                break
            in_workflow_section = (
                line == "## Rigid body visual verification workflow"
            )
            continue
        if not in_workflow_section or not line.startswith("| `"):
            continue
        cells = [cell.strip() for cell in line.split("|")]
        if len(cells) < 3:
            continue
        rows.append(cells[1].removeprefix("`").removesuffix("`"))
    return rows


def _read_capture_command_specs(
    path: pathlib.Path, marker: str
) -> list[CaptureCommandSpec]:
    commands: list[str] = []
    command_parts: list[str] = []
    in_capture_block = False
    seen_marker = False
    for line in path.read_text(encoding="utf-8").splitlines():
        if not seen_marker:
            seen_marker = line == marker
            continue
        if line.startswith("```"):
            if in_capture_block:
                break
            in_capture_block = True
            continue
        if not in_capture_block:
            continue
        stripped = line.strip()
        if not stripped:
            continue
        if stripped.endswith("\\"):
            command_parts.append(stripped[:-1].strip())
            continue
        command_parts.append(stripped)
        command = " ".join(command_parts)
        command_parts = []
        if "pixi run py-demo-capture" in command:
            commands.append(command)
    assert not command_parts
    return [_capture_spec_from_command(command) for command in commands]


def _gui_run_demos_available() -> bool:
    """True when the dartpy Filament viewer entry point is built.

    Non-GUI builds omit the viewer. Without the viewer the runner returns early,
    so tests that exercise the cycle/screenshot paths skip in that configuration.
    """

    try:
        import dartpy as dart
    except Exception:  # pragma: no cover - dartpy import failure
        return False
    return hasattr(dart, "gui") and hasattr(dart.gui, "run_demos")


def _make_box_scene_setup(
    dart: Any,
    scene_setup_type: type,
    *,
    name: str,
    frame_name: str,
    color: tuple[float, float, float] = (0.2, 0.7, 0.9),
    pre_step=None,
    visible: bool = True,
):
    import numpy as np

    scene = dart.gui.DescriptorRenderScene(dart.World(), name)
    scene.set_time_step(0.001)
    if visible:
        frame = dart.SimpleFrame(dart.gui.world_render_frame(), frame_name)
        frame.set_shape(dart.BoxShape(np.array([0.2, 0.2, 0.2])))
        frame.create_visual_aspect().set_color(list(color))
        scene.add_simple_frame(frame)
    return scene_setup_type(
        world=scene,
        pre_step=pre_step,
        renderable_provider=scene.renderable_provider,
    )


def _read_ppm(path: pathlib.Path) -> tuple[int, int, bytes]:
    data = path.read_bytes()
    parts = data.split(b"\n", 3)
    assert len(parts) == 4
    assert parts[0] == b"P6"
    width, height = (int(part) for part in parts[1].split())
    assert parts[2] == b"255"
    return width, height, parts[3][: width * height * 3]


def _mean_luminance(
    pixels: bytes,
    width: int,
    x0: int,
    x1: int,
    y0: int,
    y1: int,
) -> float:
    total = 0.0
    count = 0
    for y in range(y0, y1):
        row = y * width * 3
        for x in range(x0, x1):
            offset = row + x * 3
            red, green, blue = pixels[offset : offset + 3]
            total += 0.2126 * red + 0.7152 * green + 0.0722 * blue
            count += 1
    assert count > 0
    return total / count


def _simulation_has(*names: str) -> bool:
    try:
        import dartpy as sx
    except Exception:  # pragma: no cover - dartpy import failure
        return False
    return all(hasattr(sx, name) for name in names)


def _require_simulation_symbols(*names: str):
    try:
        import dartpy as sx
    except Exception as exc:  # pragma: no cover - dartpy import failure
        pytest.skip(f"dartpy unavailable: {exc}")
    missing = [name for name in names if not hasattr(sx, name)]
    if missing:
        formatted = ", ".join(f"dartpy.{name}" for name in missing)
        pytest.skip(f"{formatted} unavailable in this build")
    return sx


def _deformable_bindings_available() -> bool:
    """True when the experimental deformable bindings are compiled in.

    Some Python builds can omit the deformable types, so the solver-free
    grid-builder check skips there rather than erroring.
    """

    return _simulation_has("DeformableBodyOptions")


def test_registry_has_scenes() -> None:
    scenes = make_demo_scenes()
    assert len(scenes) >= 1
    # IDs are unique and non-empty; titles/categories/summaries are set.
    ids = [scene.id for scene in scenes]
    assert len(set(ids)) == len(ids)
    for scene in scenes:
        assert scene.id and scene.title and scene.category and scene.summary
        assert callable(scene.build)


def test_runner_cycle_returns_zero() -> None:
    if not _gui_run_demos_available():
        pytest.skip("dartpy.gui.run_demos unavailable (GUI not built)")
    if not _simulation_has("World"):
        pytest.skip("dartpy.World unavailable in this build")
    rc = run(["--cycle-scenes", "--frames", "2", "--headless"], make_demo_scenes())
    assert rc == 0


def test_runner_list_prints_catalog(capsys: pytest.CaptureFixture[str]) -> None:
    rc = run(["--list"], make_demo_scenes())
    assert rc == 0
    captured = capsys.readouterr().out
    for scene in make_demo_scenes():
        assert scene.id in captured
    assert "01/" not in captured
    assert "rigid_body                   World Rigid Body" in captured


def test_runner_unknown_scene_exits() -> None:
    with pytest.raises(SystemExit):
        run(["--scene", "definitely_not_a_scene"], make_demo_scenes())


def test_ipc_deformable_grid_builder_topology() -> None:
    """Pin the shared IPC deformable grid builder's mesh math (solver-free).

    For a ``columns x rows`` grid the builder emits ``columns*rows`` nodes,
    horizontal + vertical + two-diagonal-per-cell spring edges, and two
    triangles per cell.
    """

    if not _deformable_bindings_available():
        pytest.skip("dartpy.DeformableBodyOptions unavailable in this build")

    from examples.demos._ipc_deformable_bridge import build_grid_options

    columns, rows = 9, 5
    options = build_grid_options(
        columns,
        rows,
        position_fn=lambda col, row: (0.1 * col, 0.1 * row, 0.0),
        mass=0.05,
        edge_stiffness=50.0,
        damping=1.0,
        fixed_nodes=[0],
    )

    expected_edges = (
        (columns - 1) * rows  # horizontal
        + columns * (rows - 1)  # vertical
        + 2 * (columns - 1) * (rows - 1)  # two diagonals per cell
    )
    expected_triangles = 2 * (columns - 1) * (rows - 1)
    assert len(options.positions) == columns * rows
    assert len(options.masses) == columns * rows
    assert len(options.edges) == expected_edges
    assert len(options.surface_triangles) == expected_triangles
    # Rest lengths come from the initial layout (the 0.1 grid spacing).
    assert options.edges[0].rest_length == pytest.approx(0.1)
    # The first edge is the horizontal (col 0, row 0) -> (col 1, row 0).
    assert (options.edges[0].node_a, options.edges[0].node_b) == (0, 1)
    # Pin the first cell's two triangles (a,b,c) + (b,d,c) with
    # a=(0,0), b=(1,0), c=(0,1), d=(1,1) so a winding/index flip is caught.
    first = options.surface_triangles[0]
    second = options.surface_triangles[1]
    assert (first.node_a, first.node_b, first.node_c) == (0, 1, columns)
    assert (second.node_a, second.node_b, second.node_c) == (1, columns + 1, columns)


def test_ipc_deformable_fem_self_contact_activates_under_compression() -> None:
    """The FEM buckling showcase actually self-collides, intersection-free.

    Driving a slender FEM beam's pinned ends together makes it buckle and fold
    onto itself; the self-contact barrier must (a) activate and (b) hold the
    folding surface at a positive separation (no interpenetration), with the
    solve staying finite. Verified through the solver-diagnostics snapshot.
    """

    sx = _require_simulation_symbols(
        "DeformableBodyOptions",
        "DeformableDirichletBoundaryCondition",
        "DeformableTetrahedron",
        "World",
    )
    import numpy as np
    from examples.demos._ipc_deformable_bridge import build_fem_compression_bar

    options, _edges = build_fem_compression_bar(
        cells_x=24,
        cells_y=2,
        cells_z=2,
        cell_size=0.05,
        origin=(-0.6, -0.05, 0.5),
        youngs_modulus=2.0e4,
        compression_rate=0.12,
        compression_end_time=3.0,
    )

    world = sx.World()
    world.gravity = [0.0, 0.0, -3.0]
    world.time_step = 0.004
    body = world.add_deformable_body("fem_buckle", options)
    world.enter_simulation_mode()

    max_active = 0
    positive_separation_samples = 0
    for _ in range(260):
        world.step()
        diag = world.last_deformable_solver_diagnostics
        max_active = max(max_active, diag.self_contact_barrier_active_contacts)
        if diag.converged_active_contact_count > 0:
            # The barrier must hold the active contacts strictly apart.
            assert diag.min_active_contact_distance > 0.0
            positive_separation_samples += 1

    # The beam self-collided (the barrier activated) over the compression.
    assert max_active > 0
    assert positive_separation_samples > 0

    # The solve stayed finite throughout (the barrier never let it blow up).
    final = np.array([body.node_position(i) for i in range(body.node_count)])
    assert np.isfinite(final).all()


def test_ipc_deformable_obj_cloth_loads_and_drapes() -> None:
    """The .obj importer feeds a real deformable solve.

    Loading a bundled Wavefront ``.obj`` cloth, pinning one edge, and stepping
    under gravity must produce a draped sheet: the free edge sags well below the
    pinned edge, the solve stays finite, and nothing falls through the ground.
    """

    # The importer round-trips the bundled mesh into a usable surface.
    sx = _require_simulation_symbols("load_obj_triangle_mesh", "World")
    import numpy as np
    from examples.demos.scenes.ipc_deformable_obj_cloth import _CLOTH_PATH, build

    loaded = sx.load_obj_triangle_mesh(str(_CLOTH_PATH))
    assert len(loaded.positions) == 121
    assert len(loaded.surface_triangles) == 200

    setup = build()
    world = setup.info["sx_world"]
    body = world.get_deformable_body("obj_cloth")
    n = body.node_count

    for _ in range(300):
        world.step()

    z = np.array([body.node_position(i)[2] for i in range(n)])
    assert np.isfinite(z).all()
    # The free edge sagged well below the pinned (max-y) edge.
    assert float(z.max() - z.min()) > 0.1
    # Nothing tunneled through the ground top (z = -0.20).
    assert float(z.min()) > -0.21


def test_ipc_deformable_cloth_drapes_over_capsule_rod_without_penetrating() -> None:
    """The capsule (rod) obstacle barrier holds a draping cloth off its surface.

    A cloth dropped over a static horizontal capsule rod must drape over it (its
    halves hang down on either side), stay finite, and keep every node strictly
    outside the rod surface -- the codimensional-obstacle analogue of the
    sphere/box settle tests, verified by the analytic point-to-segment distance.
    """

    _require_simulation_symbols("load_obj_triangle_mesh", "World")

    import numpy as np

    from examples.demos.scenes import ipc_deformable_capsule_rod as scene

    setup = scene.build()
    world = setup.info["sx_world"]
    body = world.get_deformable_body("capsule_cloth")
    n = body.node_count

    for _ in range(260):
        world.step()

    points = np.array([body.node_position(i) for i in range(n)])
    assert np.isfinite(points).all()
    # The sheet draped over the rod (clear vertical spread).
    assert float(points[:, 2].max() - points[:, 2].min()) > 0.2

    # Every node stays strictly outside the capsule surface (no interpenetration).
    center = np.asarray(scene._ROD_CENTER, dtype=float)
    axis = np.array([0.0, 1.0, 0.0])  # body z -> world y (the scene's rotation)
    point_a = center + scene._ROD_HALF_HEIGHT * axis
    point_b = center - scene._ROD_HALF_HEIGHT * axis
    seg = point_b - point_a
    seg_len_sq = float(seg @ seg)
    min_surface = float("inf")
    for p in points:
        t = np.clip((p - point_a) @ seg / seg_len_sq, 0.0, 1.0)
        closest = point_a + t * seg
        min_surface = min(
            min_surface, float(np.linalg.norm(p - closest)) - scene._ROD_RADIUS
        )
    assert min_surface > 0.0


def test_ipc_deformable_rod_friction_decelerates_sliding_strip() -> None:
    """Coulomb friction against the capsule rod obstacle stops a sliding strip.

    A deformable strip shoved along the (barrier-only, CCD-free) capsule rod
    slides measurably less with friction than without, while staying on the rod.
    """

    _require_simulation_symbols(
        "CollisionShape",
        "DeformableBodyOptions",
        "DeformableEdge",
        "World",
    )

    import numpy as np

    from examples.demos.scenes import ipc_deformable_rod_friction as scene

    def slid_distance(mu: float) -> float:
        options, _edges = scene._build_strip()
        options.material.friction_coefficient = mu
        world = scene.sx.World()
        world.gravity = [0.0, 0.0, -9.81]
        world.time_step = 0.004
        rod = world.add_rigid_body(
            "rod", position=scene._ROD_CENTER, orientation=scene._ROD_ORIENTATION
        )
        rod.is_static = True
        rod.set_collision_shape(
            scene.sx.CollisionShape.capsule(scene._ROD_RADIUS, scene._ROD_HALF_HEIGHT)
        )
        rod.is_deformable_surface_ccd_obstacle = True
        body = world.add_deformable_body("strip", options)
        y0 = float(np.mean([body.node_position(i)[1] for i in range(body.node_count)]))
        for _ in range(200):
            world.step()
        positions = np.array([body.node_position(i) for i in range(body.node_count)])
        assert np.isfinite(positions).all()
        return float(np.mean(positions[:, 1]) - y0)

    frictionless = slid_distance(0.0)
    high_friction = slid_distance(0.8)
    assert frictionless > 0.5
    assert high_friction < 0.5 * frictionless


def test_ipc_deformable_barrier_only_box_friction_decelerates_strip() -> None:
    """Friction against a barrier-only box plate decelerates a sliding strip.

    A strip shoved across a barrier-only box (excluded from the surface CCD) slides
    far frictionless but is held back under friction, staying above the top face --
    the CCD-free path to box/sphere obstacle friction.
    """

    _require_simulation_symbols(
        "CollisionShape",
        "DeformableBodyOptions",
        "DeformableEdge",
        "World",
    )

    import numpy as np

    from examples.demos.scenes import ipc_deformable_plate_friction as scene

    def slid(mu: float) -> tuple[float, float]:
        options, _edges = scene._build_strip()
        options.material.friction_coefficient = mu
        world = scene.sx.World()
        world.gravity = [0.0, 0.0, -9.81]
        world.time_step = 0.004
        plate = world.add_rigid_body("plate", position=scene._PLATE_CENTER)
        plate.is_static = True
        plate.set_collision_shape(scene.sx.CollisionShape.box(scene._PLATE_HALF))
        plate.is_deformable_surface_ccd_obstacle = True
        plate.is_deformable_obstacle_barrier_only = True
        body = world.add_deformable_body("strip", options)
        x0 = float(np.mean([body.node_position(i)[0] for i in range(body.node_count)]))
        for _ in range(200):
            world.step()
        positions = np.array([body.node_position(i) for i in range(body.node_count)])
        assert np.isfinite(positions).all()
        return float(np.mean(positions[:, 0]) - x0), float(positions[:, 2].min())

    frictionless, free_z = slid(0.0)
    high_friction, mu_z = slid(0.8)
    assert frictionless > 0.5
    assert high_friction < 0.5 * frictionless
    # Both stay above the box top face (z = 0.5).
    assert free_z > 0.49
    assert mu_z > 0.5


def test_ipc_deformable_seg_and_pt_importers_feed_solves() -> None:
    """The .seg and .pt importers feed real deformable solves.

    A `.seg` strand pinned at one end must hang (its tip falls well below the
    pinned node), and a `.pt` particle cloud must fall under gravity and stack on
    the ground barrier without tunneling through it -- both staying finite.
    """

    _require_simulation_symbols(
        "load_point_set",
        "load_seg_line_mesh",
        "World",
    )

    import numpy as np
    from examples.demos.scenes.ipc_deformable_pt_particles import (
        build as build_particles,
    )
    from examples.demos.scenes.ipc_deformable_seg_strand import build as build_strand

    strand = build_strand()
    sworld = strand.info["sx_world"]
    sbody = sworld.get_deformable_body("seg_strand")
    sn = sbody.node_count
    for _ in range(200):
        sworld.step()
    sz = np.array([sbody.node_position(i)[2] for i in range(sn)])
    assert np.isfinite(sz).all()
    # The pinned node (0) stays put; the tip hangs well below it.
    assert float(sz[0] - sz.min()) > 0.1

    particles = build_particles()
    pworld = particles.info["sx_world"]
    pbody = pworld.get_deformable_body("pt_particles")
    pn = pbody.node_count
    z_start = min(pbody.node_position(i)[2] for i in range(pn))
    for _ in range(200):
        pworld.step()
    pz = np.array([pbody.node_position(i)[2] for i in range(pn)])
    assert np.isfinite(pz).all()
    assert float(pz.min()) < z_start - 0.05  # fell under gravity
    assert float(pz.min()) > -0.02  # caught by the ground barrier


def test_ipc_deformable_scenes_share_dedicated_category() -> None:
    """The IPC deformable scenes live in their own dedicated menu category.

    The IPC showcases must group under ``IPC Deformable`` so the viewer renders
    them together.
    """

    scenes = make_demo_scenes()
    by_id = {scene.id: scene for scene in scenes}

    expected_ipc = {
        "ipc_deformable_net",
        "ipc_deformable_drape",
        "ipc_deformable_trampoline",
        "ipc_deformable_friction_slide",
        "ipc_deformable_fem_bar",
        "ipc_deformable_fem_twist",
        "ipc_deformable_fcr_twist",
        "ipc_deformable_fem_drop",
        "ipc_deformable_fem_sphere",
        "ipc_deformable_fem_box",
        "ipc_deformable_fem_buckle",
        "ipc_deformable_fem_msh",
        "ipc_deformable_cg_solver",
        "ipc_deformable_cg_contact",
        "ipc_deformable_obj_cloth",
        "ipc_deformable_capsule_rod",
        "ipc_deformable_rod_friction",
        "ipc_deformable_plate_friction",
        "ipc_deformable_seg_strand",
        "ipc_deformable_pt_particles",
        "ipc_deformable_scripted_dirichlet",
    }
    assert expected_ipc <= set(by_id), "missing IPC deformable scenes"

    for scene_id in expected_ipc:
        assert by_id[scene_id].category == "IPC Deformable"

    # Every scene in the dedicated category is an IPC deformable scene.
    ipc_category = [s.id for s in scenes if s.category == "IPC Deformable"]
    assert set(ipc_category) == expected_ipc
    assert not any(s.category == "Experimental" for s in scenes)


def test_world_scenes_use_solver_focused_categories() -> None:
    scenes = make_demo_scenes()
    by_id = {scene.id: scene for scene in scenes}

    expected = {
        "World Rigid Body": {
            "articulated",
            "floating_base",
            "contact",
            "rigid_body",
            "rigid_body_modes",
            "rigid_free_flight",
            "rigid_frame_hierarchy",
            "rigid_external_loads",
            "rigid_link_point_loads",
            "rigid_timestep_sensitivity",
            "rigid_step_diagnostics",
            "rigid_contact_scale_budget",
            "rigid_contact_inspector",
            "rigid_collision_query_options",
            "rigid_collision_casts",
            "rigid_contact_manipulation",
            "rigid_contact_solver_compare",
            "rigid_restitution_ladder",
            "rigid_material_mixing",
            "rigid_solver_compare",
            "rigid_executor_equivalence",
            "rigid_friction_threshold",
            "rigid_spin_roll_coupling",
            "rigid_stack_stability",
            "rigid_fixed_joint",
            "rigid_joint_breakage",
            "rigid_limited_joints",
            "rigid_joint_motor_limits",
            "rigid_joint_passive_parameters",
            "rigid_screw_joint_pitch",
            "rigid_multibody_dynamics_terms",
            "rigid_link_jacobian",
            "rigid_multibody_solver_family",
            "rigid_loop_closure",
            "rigid_kinematic_driver",
            "rigid_link_center_of_mass",
        },
        "AVBD Rigid Constraints (sx)": {
            "avbd_rigid_fixed_joint_contact",
            "avbd_rigid_revolute_motor",
            "avbd_rigid_breakable_joint",
        },
        "Planned World Ports": {
            "planned_inverse_kinematics",
            "planned_simbicon_walking",
            "planned_operational_space_control",
            "g1_puppet",
            "planned_collision_sandbox",
            "planned_mobile_manipulation",
        },
        "Robot Models": {
            "atlas_puppet",
            "hubo_puppet",
        },
        "Control & IK": {
            "atlas_simbicon",
        },
        "Rigid IPC": {
            "rigid_ipc",
            "rigid_ipc_slide",
            "rigid_ipc_incline",
            "rigid_ipc_edge_drop",
            "rigid_ipc_pile",
            "rigid_ipc_tunnel",
            "rigid_ipc_stack_packet",
        },
        "PLAN-083 Mixed Corpus": {
            "plan083_lying_flat",
            "plan083_hanging_bridge",
            "plan083_umbrella",
            "plan083_candy",
        },
        "PLAN-083 Constraints Corpus": {
            "plan083_pulley_system",
            "plan083_nunchaku",
            "plan083_windmill",
        },
        "PLAN-083 Robot Corpus": {
            "plan083_terrain_vehicle",
            "plan083_ragdolls",
            "plan083_precession",
        },
        "PLAN-083 ABD Corpus": {
            "plan083_abd_complex_geometry",
            "plan083_abd_fem_coupling",
        },
        "Simulation Replay": {
            "replay_scrubber",
        },
        "Variational Integrators": {
            "variational_chain",
            "variational_tumbler",
            "variational_contact",
            "loop_closure",
        },
        "Vertex Block Descent": {
            "vbd_cloth",
            "vbd_net",
            "vbd_beam",
            "vbd_tilted_strand",
            "vbd_obstacle_drape",
            "vbd_self_fold",
        },
    }

    for category, scene_ids in expected.items():
        for scene_id in scene_ids:
            assert by_id[scene_id].category == category

    assert not any(scene.category == "Experimental" for scene in scenes)
    assert not any(scene.id.startswith("sx_") for scene in scenes)


def test_world_rigid_visual_verification_scenes_are_ordered() -> None:
    scenes = make_demo_scenes()
    scene_ids = [scene.id for scene in scenes]
    world_rigid_ids = [
        scene.id for scene in scenes if scene.category == "World Rigid Body"
    ]
    ordered_ids = [
        "rigid_body",
        "rigid_body_modes",
        "rigid_free_flight",
        "rigid_frame_hierarchy",
        "rigid_external_loads",
        "rigid_link_point_loads",
        "rigid_timestep_sensitivity",
        "rigid_step_diagnostics",
        "rigid_contact_scale_budget",
        "rigid_restitution_ladder",
        "rigid_material_mixing",
        "rigid_contact_inspector",
        "rigid_collision_query_options",
        "rigid_collision_casts",
        "rigid_solver_compare",
        "rigid_executor_equivalence",
        "rigid_contact_solver_compare",
        "contact",
        "rigid_friction_threshold",
        "rigid_spin_roll_coupling",
        "rigid_stack_stability",
        "rigid_contact_manipulation",
        "rigid_kinematic_driver",
        "rigid_fixed_joint",
        "rigid_joint_breakage",
        "rigid_limited_joints",
        "rigid_joint_motor_limits",
        "rigid_joint_passive_parameters",
        "rigid_screw_joint_pitch",
        "rigid_multibody_dynamics_terms",
        "rigid_link_center_of_mass",
        "rigid_link_jacobian",
        "rigid_multibody_solver_family",
        "rigid_loop_closure",
    ]

    assert world_rigid_ids[: len(ordered_ids)] == ordered_ids
    assert scene_ids[: len(ordered_ids)] == ordered_ids


def test_rigid_visual_verification_sidecar_matches_registry_order() -> None:
    scenes = make_demo_scenes()
    by_id = {scene.id: scene for scene in scenes}
    workflow_rows = _read_rigid_visual_workflow_rows()
    orders = [order for order, _scene_id in workflow_rows]
    workflow_ids = [scene_id for _order, scene_id in workflow_rows]

    assert orders == list(range(1, len(workflow_rows) + 1))
    assert len(workflow_ids) == len(set(workflow_ids))
    assert not [scene_id for scene_id in workflow_ids if scene_id not in by_id]

    world_rigid_ids = [
        scene.id for scene in scenes if scene.category == "World Rigid Body"
    ]
    assert workflow_ids == world_rigid_ids[: len(workflow_ids)]
    assert workflow_ids[-1] == "rigid_loop_closure"
    assert "rigid_ipc_tunnel" not in workflow_ids
    assert "rigid_ipc_stack_packet" not in workflow_ids
    assert by_id["rigid_ipc_tunnel"].category == "Rigid IPC"
    assert by_id["rigid_ipc_stack_packet"].category == "Rigid IPC"


def test_rigid_visual_workflow_viewer_titles_are_numbered() -> None:
    scenes = make_demo_scenes()
    by_id = {scene.id: scene for scene in scenes}
    workflow_rows = _read_rigid_visual_workflow_rows()
    count = len(workflow_rows)

    assert count == len(RIGID_VISUAL_WORKFLOW_LABELS)
    for order, scene_id in workflow_rows:
        title = _viewer_catalog_title(by_id[scene_id])
        assert title.startswith(f"{order:02d}/{count:02d} ")
        assert title.endswith(f": {by_id[scene_id].title}")

    assert _viewer_catalog_title(by_id["articulated"]) == by_id["articulated"].title
    assert (
        _viewer_catalog_title(by_id["rigid_ipc_tunnel"])
        == by_id["rigid_ipc_tunnel"].title
    )
    assert (
        _viewer_catalog_title(by_id["rigid_ipc_stack_packet"])
        == by_id["rigid_ipc_stack_packet"].title
    )


def test_rigid_visual_workflow_guidance_matches_sidecar() -> None:
    workflow_details = _read_rigid_visual_workflow_details()
    workflow_ids = [
        scene_id for _order, scene_id, _question, _scope in workflow_details
    ]

    assert set(RIGID_VISUAL_WORKFLOW_GUIDES) == set(workflow_ids)
    assert len(RIGID_VISUAL_WORKFLOW_GUIDES) == len(RIGID_VISUAL_WORKFLOW_LABELS)

    for index, (order, scene_id, question, scope) in enumerate(workflow_details):
        guide = RIGID_VISUAL_WORKFLOW_GUIDES[scene_id]
        assert guide.index == order
        assert guide.count == len(workflow_details)
        assert guide.question == question
        assert guide.try_first
        assert guide.inspect
        assert guide.healthy_signal
        assert guide.scope == scope
        assert guide.previous_scene_id == (
            workflow_ids[index - 1] if index > 0 else None
        )
        assert guide.next_scene_id == (
            workflow_ids[index + 1] if index + 1 < len(workflow_ids) else None
        )


def test_rigid_visual_workflow_related_evidence_routes_are_valid() -> None:
    scenes = make_demo_scenes()
    by_id = {scene.id: scene for scene in scenes}
    related_rows = _read_rigid_visual_related_evidence_rows()

    assert related_rows == [
        (
            "rigid_solver_compare",
            "rigid_ipc_tunnel",
            "Rigid IPC",
            "Related shelf: Rigid IPC / rigid_ipc_tunnel - focused no-tunneling view",
            "Focused IPC capability scene; not a broad solver comparison or general proof.",
        ),
        (
            "rigid_contact_solver_compare",
            "diff_drone_liftoff",
            "Differentiable",
            (
                "Related shelf: Differentiable / diff_drone_liftoff - "
                "contact-gradient route"
            ),
            "Analytic vs complementarity-aware clamping-contact optimization; not a solver row.",
        ),
    ]
    assert set(_RIGID_WORKFLOW_RELATED_EVIDENCE) == {
        source_scene_id
        for source_scene_id, _target_scene_id, _shelf, _label, _scope in related_rows
    }
    for source_scene_id, target_scene_id, shelf, label, scope in related_rows:
        assert source_scene_id in RIGID_VISUAL_WORKFLOW_GUIDES
        entries = _RIGID_WORKFLOW_RELATED_EVIDENCE[source_scene_id]
        assert len(entries) == 1
        entry = entries[0]
        assert entry.scene_id == target_scene_id
        assert entry.scene_id in by_id
        assert entry.scene_id not in RIGID_VISUAL_WORKFLOW_GUIDES
        assert by_id[entry.scene_id].category == shelf
        assert entry.shelf == shelf
        assert f"Related shelf: {entry.label}" == label
        assert entry.reason == scope


def test_rigid_visual_capture_first_ipc_packets_are_documented() -> None:
    scenes = make_demo_scenes()
    by_id = {scene.id: scene for scene in scenes}
    workflow_ids = {
        scene_id for _order, scene_id in _read_rigid_visual_workflow_rows()
    }
    rows = _read_rigid_visual_capture_first_ipc_rows()

    assert rows == [
        (
            "rigid_ipc_stack_packet",
            "Can a four-box IPC stack stay separated, ordered, and finite beyond the live demo budget?",
            "Friction, box count, frame-budget threshold, min clearance, contact count, top drift, height error, max speed, wall time, and benchmark pointer.",
            "pixi run py-demo-capture -- --scene rigid_ipc_stack_packet --frames 24 --width 960 --height 540 --show-ui",
            "Capture-first stress packet; not a numbered workflow row and not a solver-performance parity claim.",
        )
    ]
    for scene_id, question, signals, command, scope in rows:
        assert scene_id in by_id
        assert by_id[scene_id].category == "Rigid IPC"
        assert scene_id not in workflow_ids
        assert _viewer_catalog_title(by_id[scene_id]) == by_id[scene_id].title
        assert "live demo budget" in question
        assert "wall time" in signals
        assert "benchmark pointer" in signals
        assert "not a numbered workflow row" in scope

        scene, frames, width, height, show_ui = _capture_spec_from_command(command)
        assert (scene, frames, width, height, show_ui) == (
            scene_id,
            24,
            960,
            540,
            True,
        )


def test_rigid_ipc_stack_packet_reports_capture_first_metrics() -> None:
    import numpy as np

    sx = _require_simulation_symbols("RigidBodySolver")

    from examples.demos.scenes.rigid_ipc_stack_packet import SCENE, build

    assert SCENE.category == "Rigid IPC"
    assert SCENE.id == "rigid_ipc_stack_packet"

    setup = build()
    controller = setup.info["rigid_ipc_stack_packet_controller"]
    assert setup.info["rigid_ipc_stack_packet_capture_first"] is True
    assert setup.info["rigid_ipc_stack_packet_benchmark"] == "bm_rigid_ipc_solver"
    assert controller.world.rigid_body_solver == sx.RigidBodySolver.IPC

    for _ in range(2):
        setup.pre_step()

    metrics = controller._last_metrics
    capture_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert float(metrics["box_count"]) == pytest.approx(4.0)
    assert float(metrics["min_clearance"]) > -0.004
    assert float(metrics["max_speed"]) >= 0.0
    assert float(metrics["top_drift"]) >= 0.0
    assert float(metrics["step_ms"]) >= 0.0
    assert str(metrics["status"]) in {"capture-first", "settling", "standing"}
    assert capture_metrics["capture_first"] is True
    assert capture_metrics["benchmark"] == "bm_rigid_ipc_solver"
    assert capture_metrics["solver"] == "ipc"
    assert float(capture_metrics["box_count"]) == pytest.approx(4.0)
    assert float(capture_metrics["frame_budget_ms"]) > 0.0
    assert float(capture_metrics["world_time"]) > 0.0
    assert callable(setup.info["replay_capture_state"])
    assert callable(setup.info["replay_restore_state"])
    replay_snapshot = setup.info["replay_capture_state"]()
    controller.friction = 0.05
    controller.frame_budget_ms = 12.0
    controller._step_ms_history.clear()
    setup.info["replay_restore_state"](replay_snapshot)
    assert controller.friction == pytest.approx(
        replay_snapshot["controls"]["friction"]
    )
    assert controller.frame_budget_ms == pytest.approx(
        replay_snapshot["controls"]["frame_budget_ms"]
    )
    assert list(controller._step_ms_history) == pytest.approx(
        replay_snapshot["step_ms_history"]
    )
    assert controller._step_ms_history
    assert controller._clearance_history
    assert controller._speed_history
    assert controller._drift_history
    assert controller._contact_history
    assert np.isfinite(
        [
            float(metrics["contact_count"]),
            float(metrics["height_error"]),
            float(metrics["max_speed"]),
            float(metrics["min_clearance"]),
            float(metrics["step_ms"]),
            float(metrics["top_drift"]),
        ]
    ).all()


def test_rigid_visual_workflow_docs_use_current_navigator_count() -> None:
    root = pathlib.Path(__file__).resolve().parents[3]
    workflow_count = len(_read_rigid_visual_workflow_rows())
    documented_paths = [
        root
        / "docs"
        / "plans"
        / "103-examples-strategy"
        / "rigid-body-visual-verification.md",
        root / "python" / "examples" / "demos" / "README.md",
    ]

    for path in documented_paths:
        text = path.read_text(encoding="utf-8")
        assert f"01/{workflow_count:02d} Baseline: World Rigid Body" in text
        assert f"15/{workflow_count:02d} Solver family: Rigid Solver Compare" in text
        for match in re.finditer(r"\b\d{2}/(?P<count>\d{2})\b", text):
            assert int(match.group("count")) == workflow_count


def test_rigid_visual_verification_deferred_api_gaps_are_documented() -> None:
    sx = _require_simulation_symbols(
        "RigidBody",
        "World",
        "LoopClosure",
        "LoopClosureSpec",
        "Multibody",
    )

    root = pathlib.Path(__file__).resolve().parents[3]
    sidecar = (
        root
        / "docs"
        / "plans"
        / "103-examples-strategy"
        / "rigid-body-visual-verification.md"
    )
    dev_task = (
        root
        / "docs"
        / "dev_tasks"
        / "rigid_body_visual_verification"
        / "README.md"
    )
    text = "\n".join(
        [
            sidecar.read_text(encoding="utf-8"),
            dev_task.read_text(encoding="utf-8"),
        ]
    )

    rigid_body_attrs = set(dir(sx.RigidBody))
    world_attrs = set(dir(sx.World))
    loop_attrs = set(dir(sx.LoopClosure)) | set(dir(sx.LoopClosureSpec))

    assert {
        "apply_force",
        "apply_torque",
        "linear_momentum",
        "angular_momentum",
    } <= rigid_body_attrs
    assert "compute_impulse_response" in set(dir(sx.Multibody))

    rigid_impulse_attrs = {
        name
        for name in rigid_body_attrs
        if "impulse" in name.lower()
    }
    assert not rigid_impulse_attrs, (
        "Public RigidBody impulse API appeared; update the rigid-body visual "
        "workflow and revisit the direct-impulse row deferral."
    )
    assert "no public direct rigid-body impulse surface" in text

    activation_attrs = {
        name
        for name in (rigid_body_attrs | world_attrs)
        if any(
            token in name.lower()
            for token in ("sleep", "wake", "island", "activation", "deactivation")
        )
    }
    assert not activation_attrs, (
        "Public sleep/wake/island activation API appeared; update the rigid-body "
        "visual workflow and revisit the activation-state row deferral."
    )
    assert "no public sleep/wake or island activation surface" in text

    compliance_attrs = {
        name
        for name in loop_attrs
        if any(token in name.lower() for token in ("compliance", "stiffness", "damping"))
    }
    assert not compliance_attrs, (
        "Public loop-closure compliance API appeared; update the rigid-body "
        "visual workflow and revisit the compliance row deferral."
    )
    assert "no public loop-closure compliance surface" in text


def test_rigid_visual_verification_readme_matches_sidecar_order() -> None:
    sidecar_ids = [
        scene_id for _order, scene_id in _read_rigid_visual_workflow_rows()
    ]
    readme_ids = _read_rigid_visual_readme_workflow_ids()

    assert readme_ids == sidecar_ids


def test_rigid_visual_verification_capture_commands_match_workflow() -> None:
    root = pathlib.Path(__file__).resolve().parents[3]
    sidecar = (
        root
        / "docs"
        / "plans"
        / "103-examples-strategy"
        / "rigid-body-visual-verification.md"
    )
    readme = root / "python" / "examples" / "demos" / "README.md"
    workflow_ids = [
        scene_id for _order, scene_id in _read_rigid_visual_workflow_rows()
    ]
    table_specs = _read_rigid_visual_table_capture_specs()

    sidecar_capture_specs = _read_capture_command_specs(
        sidecar,
        "For a quick curated refresh:",
    )
    readme_capture_specs = _read_capture_command_specs(
        readme,
        "Capture the focused rigid verifier scenes with the docked UI visible:",
    )

    assert table_specs == sidecar_capture_specs
    assert readme_capture_specs == table_specs
    assert [scene_id for scene_id, *_rest in table_specs] == workflow_ids
    assert all(
        (width, height, show_ui) == (960, 540, True)
        for _scene_id, _frames, width, height, show_ui in table_specs
    )


def test_rigid_contact_inspector_reports_contact_manifolds() -> None:
    import numpy as np

    sx = _require_simulation_symbols("CollisionShapeType")

    from examples.demos.scenes.rigid_contact_inspector import build

    setup = build()
    controller = setup.info["rigid_contact_inspector_controller"]
    for _ in range(4):
        setup.pre_step()

    metrics = controller._last_metrics
    assert len(controller.lanes) >= 7
    assert {lane.key for lane in controller.lanes} >= {
        "sphere_box",
        "box_ground",
        "plane_sphere",
        "capsule_sphere",
        "cylinder_sphere",
        "mesh_sphere",
        "compound_sphere",
    }
    plane_lane = next(lane for lane in controller.lanes if lane.key == "plane_sphere")
    assert plane_lane.target.collision_shape.type == sx.CollisionShapeType.PLANE
    assert metrics["total_contact_count"] >= len(controller.lanes)
    assert metrics["selected_contact_count"] >= 1
    assert metrics["max_depth"] > 0.0
    assert metrics["selected_max_depth"] > 0.0
    assert metrics["first_depth"] > 0.0
    assert np.isfinite(metrics["first_point"]).all()
    assert np.isfinite(metrics["first_normal"]).all()
    assert np.linalg.norm(metrics["first_normal"]) == pytest.approx(1.0, abs=1e-6)
    assert np.isfinite(metrics["first_local_a"]).all()
    assert np.isfinite(metrics["first_local_b"]).all()
    assert tuple(metrics["first_shape_indices"]) >= (0, 0)
    assert controller._contact_count_history
    assert controller._max_depth_history
    assert controller._selected_depth_history

    for index, lane in enumerate(controller.lanes):
        controller.pair_index = index
        controller._record_metrics()
        lane_metrics = controller._last_metrics
        assert lane_metrics["selected_contact_count"] >= 1
        assert lane_metrics["selected_max_depth"] == pytest.approx(
            controller.penetration,
            abs=1.0e-6,
        )
        assert np.isfinite(lane_metrics["first_point"]).all()
        assert np.isfinite(lane_metrics["first_normal"]).all()
        assert np.linalg.norm(lane_metrics["first_normal"]) == pytest.approx(
            1.0,
            abs=1.0e-6,
        )
        assert np.isfinite(lane_metrics["first_local_a"]).all()
        assert np.isfinite(lane_metrics["first_local_b"]).all()

        shape_indices = tuple(lane_metrics["first_shape_indices"])
        assert shape_indices[0] >= 0
        assert shape_indices[1] >= 0
        if lane.key == "compound_sphere":
            assert 1 in shape_indices


def test_rigid_body_modes_compare_dynamic_static_kinematic_semantics() -> None:
    import numpy as np

    from examples.demos.scenes.rigid_body_modes import build

    setup = build()
    controller = setup.info["rigid_body_modes_controller"]
    for _ in range(72):
        assert setup.pre_step is not None
        setup.pre_step()

    metrics = controller._last_metrics
    assert set(metrics) == {"dynamic", "static", "kinematic"}
    assert len(controller.world.collide()) == 0

    dynamic = metrics["dynamic"]
    static = metrics["static"]
    kinematic = metrics["kinematic"]

    assert float(dynamic["is_static"]) == 0.0
    assert float(dynamic["is_kinematic"]) == 0.0
    assert float(dynamic["height_drop"]) > 0.15
    assert float(dynamic["displacement_x"]) > 0.12
    assert float(dynamic["force_norm"]) == pytest.approx(controller.force_magnitude)
    assert str(dynamic["status"]) == "integrated"

    assert float(static["is_static"]) == 1.0
    assert float(static["is_kinematic"]) == 0.0
    assert float(static["displacement"]) == pytest.approx(0.0, abs=1.0e-12)
    assert float(static["speed"]) == pytest.approx(0.0, abs=1.0e-12)
    assert float(static["force_norm"]) == pytest.approx(controller.force_magnitude)
    assert str(static["status"]) == "fixed"

    assert float(kinematic["is_static"]) == 0.0
    assert float(kinematic["is_kinematic"]) == 1.0
    assert float(kinematic["displacement_x"]) > 0.10
    assert float(kinematic["kinematic_error"]) < 1.0e-12
    assert str(kinematic["status"]) == "prescribed"

    assert controller._height_history["dynamic"]
    assert controller._static_drift_history
    assert controller._kinematic_error_history
    assert max(controller._static_drift_history) == pytest.approx(0.0, abs=1.0e-12)
    assert max(controller._kinematic_error_history) == pytest.approx(0.0, abs=1.0e-12)
    assert np.isfinite([float(value) for value in controller._step_ms_history]).all()


def test_rigid_body_baseline_reports_restartable_first_run_diagnostics() -> None:
    import numpy as np

    from examples.demos.scenes.rigid_body import build

    setup = build()
    controller = setup.info["rigid_body_controller"]
    controller.friction = 0.42
    controller.restitution = 0.31
    controller.reset(clear_replay=False)

    assert controller.world.rigid_body_solver == controller._solver()
    assert controller.ground.friction == pytest.approx(0.42)
    for body in controller.dynamic_bodies:
        assert body.friction == pytest.approx(0.42)
        assert body.restitution == pytest.approx(0.31)

    for _ in range(180):
        assert setup.pre_step is not None
        setup.pre_step()

    metrics = controller._last_metrics
    assert float(metrics["max_speed"]) > 0.0
    assert float(metrics["min_height"]) < 1.2
    assert float(metrics["energy"]) >= 0.0
    assert float(metrics["contact_count"]) > 0.0
    assert controller._speed_history
    assert controller._min_height_history
    assert controller._energy_history
    assert controller._contact_history
    assert np.isfinite([float(value) for value in controller._step_ms_history]).all()

    controller.reset(clear_replay=False)
    assert controller.world.time == pytest.approx(0.0)
    assert len(controller._speed_history) == 1
    reset_positions = [
        np.asarray(state.body.translation, dtype=float).reshape(3)
        for state in controller._initial_states
    ]
    initial_positions = [state.transform[:3, 3] for state in controller._initial_states]
    assert np.asarray(reset_positions).reshape(-1).tolist() == pytest.approx(
        np.asarray(initial_positions).reshape(-1).tolist()
    )


def test_rigid_external_loads_scale_force_and_torque_response() -> None:
    import numpy as np

    from examples.demos.scenes.rigid_external_loads import build

    setup = build()
    controller = setup.info["rigid_external_loads_controller"]
    for _ in range(40):
        assert setup.pre_step is not None
        setup.pre_step()

    metrics = controller._last_metrics
    assert set(metrics) == {
        "light_force",
        "heavy_force",
        "pulse_force",
        "static_load",
        "low_inertia_torque",
        "high_inertia_torque",
    }

    light = metrics["light_force"]
    heavy = metrics["heavy_force"]
    pulse = metrics["pulse_force"]
    static = metrics["static_load"]
    low = metrics["low_inertia_torque"]
    high = metrics["high_inertia_torque"]

    assert float(light["linear_accel_x"]) == pytest.approx(
        float(light["expected_linear_accel_x"]), abs=1.0e-12
    )
    assert float(heavy["linear_accel_x"]) == pytest.approx(
        float(heavy["expected_linear_accel_x"]), abs=1.0e-12
    )
    assert float(light["linear_accel_x"]) == pytest.approx(
        controller.force_magnitude, abs=1.0e-12
    )
    assert float(heavy["linear_accel_x"]) == pytest.approx(
        controller.force_magnitude / controller.mass_ratio, abs=1.0e-12
    )
    assert float(light["speed"]) > 3.0 * float(heavy["speed"])
    assert float(pulse["force_norm"]) == pytest.approx(0.0, abs=1.0e-12)
    assert float(pulse["speed"]) < 0.05 * float(light["speed"])
    assert str(pulse["status"]) == "pulse cleared"

    assert float(low["angular_accel_z"]) == pytest.approx(
        float(low["expected_angular_accel_z"]), abs=1.0e-12
    )
    assert float(high["angular_accel_z"]) == pytest.approx(
        float(high["expected_angular_accel_z"]), abs=1.0e-12
    )
    assert float(low["angular_speed"]) > 4.0 * float(high["angular_speed"])
    assert float(low["angular_accel_z"]) == pytest.approx(
        controller.torque_magnitude / float(low["inertia_z"]), abs=1.0e-12
    )
    assert float(high["angular_accel_z"]) == pytest.approx(
        controller.torque_magnitude / float(high["inertia_z"]), abs=1.0e-12
    )

    assert float(static["speed"]) == pytest.approx(0.0, abs=1.0e-12)
    assert float(static["angular_speed"]) == pytest.approx(0.0, abs=1.0e-12)
    assert float(static["force_norm"]) == pytest.approx(controller.force_magnitude)
    assert float(static["torque_norm"]) == pytest.approx(controller.torque_magnitude)
    assert float(static["drift"]) == pytest.approx(0.0, abs=1.0e-12)
    assert np.isfinite([float(value) for value in controller._step_ms_history]).all()


def test_rigid_free_flight_preserves_initial_state_diagnostics() -> None:
    import numpy as np

    from examples.demos.scenes.rigid_free_flight import build

    setup = build()
    controller = setup.info["rigid_free_flight_controller"]
    for _ in range(80):
        assert setup.pre_step is not None
        setup.pre_step()

    metrics = controller._last_metrics
    assert str(metrics["status"]) == "no contacts"
    assert all(len(world.collide()) == 0 for world in controller.worlds)
    assert float(metrics["time"]) == pytest.approx(80 * 0.004)

    assert float(metrics["drift_position_error"]) < 1.0e-12
    assert float(metrics["drift_momentum_drift"]) < 1.0e-12
    assert float(metrics["drift_speed"]) == pytest.approx(controller.launch_speed)

    assert float(metrics["arc_position_error"]) < 0.010
    assert float(metrics["arc_momentum_residual"]) < 1.0e-12
    assert abs(float(metrics["arc_energy_drift"])) < 0.05
    assert float(metrics["arc_height"]) == pytest.approx(
        float(controller.arc_body.translation[2])
    )

    assert float(metrics["spin_low_angular_speed"]) == pytest.approx(
        abs(controller.spin_speed)
    )
    assert float(metrics["spin_high_angular_speed"]) == pytest.approx(
        abs(controller.spin_speed)
    )
    assert float(metrics["spin_momentum_ratio"]) == pytest.approx(
        controller.inertia_ratio
    )
    assert float(metrics["spin_energy_ratio"]) == pytest.approx(controller.inertia_ratio)

    assert controller._drift_error_history
    assert controller._arc_error_history
    assert controller._arc_momentum_residual_history
    assert controller._arc_energy_drift_history
    assert controller._spin_momentum_ratio_history
    assert controller._spin_energy_ratio_history
    assert np.isfinite([float(value) for value in controller._step_ms_history]).all()


def test_rigid_frame_hierarchy_tracks_body_fixed_frame() -> None:
    import numpy as np

    from examples.demos.scenes.rigid_frame_hierarchy import build

    setup = build()
    controller = setup.info["rigid_frame_hierarchy_controller"]
    controller.local_offset_x = 0.41
    controller.local_offset_y = -0.16
    controller.local_yaw_deg = 48.0
    controller.body_yaw_speed = 1.2
    controller.path_radius = 0.32
    controller.reset(clear_replay=False)
    for _ in range(32):
        assert setup.pre_step is not None
        setup.pre_step()

    body_transform = np.asarray(controller.body.transform, dtype=float)
    sensor_transform = np.asarray(controller.sensor.transform, dtype=float)
    local_transform = np.asarray(controller.sensor.local_transform, dtype=float)
    reconstructed = body_transform @ local_transform
    relative = np.asarray(controller.sensor.relative_transform(controller.body), dtype=float)

    assert controller.sensor.parent_frame == controller.body
    assert sensor_transform[:3, 3].tolist() == pytest.approx(
        reconstructed[:3, 3].tolist(), abs=1.0e-12
    )
    assert sensor_transform[:3, :3].reshape(9).tolist() == pytest.approx(
        reconstructed[:3, :3].reshape(9).tolist(), abs=1.0e-12
    )
    assert relative[:3, 3].tolist() == pytest.approx(
        local_transform[:3, 3].tolist(), abs=1.0e-12
    )

    metrics = controller._last_metrics
    assert str(metrics["parent"]) == controller.body.name
    assert float(metrics["world_error"]) == pytest.approx(0.0, abs=1.0e-12)
    assert float(metrics["relative_error"]) == pytest.approx(0.0, abs=1.0e-12)
    assert float(metrics["orientation_error"]) == pytest.approx(0.0, abs=1.0e-12)
    assert controller._world_error_history
    assert controller._relative_error_history
    assert np.isfinite([float(value) for value in controller._step_ms_history]).all()


def test_rigid_timestep_sensitivity_orders_freefall_error_by_step_size() -> None:
    import numpy as np

    from examples.demos.scenes.rigid_timestep_sensitivity import build

    setup = build()
    controller = setup.info["rigid_timestep_sensitivity_controller"]
    for _ in range(40):
        assert setup.pre_step is not None
        setup.pre_step()

    metrics = controller._last_metrics
    assert set(metrics) == {"fine", "medium", "coarse"}
    assert all(str(metrics[key]["status"]) == "free fall" for key in metrics)
    assert all(float(metrics[key]["contact_count"]) == 0.0 for key in metrics)

    times = [float(metrics[key]["time"]) for key in ("fine", "medium", "coarse")]
    assert max(times) - min(times) < 1.0e-12
    assert times[0] == pytest.approx(controller.base_time_step * 4.0 * 40)

    errors = [
        float(metrics[key]["freefall_error"]) for key in ("fine", "medium", "coarse")
    ]
    assert np.isfinite(errors).all()
    assert errors[0] < errors[1] < errors[2]
    assert errors[2] > 3.5 * errors[0]
    assert controller._coarse_error_ratio[-1] == pytest.approx(errors[2] / errors[0])

    assert float(metrics["fine"]["dt"]) == pytest.approx(controller.base_time_step)
    assert float(metrics["medium"]["dt"]) == pytest.approx(
        controller.base_time_step * 2.0
    )
    assert float(metrics["coarse"]["dt"]) == pytest.approx(
        controller.base_time_step * 4.0
    )

    for _ in range(40):
        assert setup.pre_step is not None
        setup.pre_step()

    assert any(value is not None for value in controller._first_contact_time.values())
    for lane in controller.lanes:
        assert controller._height_history[lane.key]
        assert controller._freefall_error_history[lane.key]
        assert controller._clearance_history[lane.key]
        assert controller._step_ms_history[lane.key]


def test_rigid_step_diagnostics_reports_profile_and_memory_counters() -> None:
    import numpy as np

    _require_simulation_symbols("World", "WorldMemoryDiagnostics", "WorldStepProfile")

    from examples.demos.scenes.rigid_step_diagnostics import build

    setup = build()
    controller = setup.info["rigid_step_diagnostics_controller"]
    assert setup.pre_step is not None
    for _ in range(18):
        setup.pre_step()

    metrics = controller._last_metrics
    capture_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert set(metrics) == {"single", "contact", "stack"}
    assert capture_metrics["row"] == "rigid_step_diagnostics"
    assert capture_metrics["solver"] == controller._solver_label()
    assert capture_metrics["executor"] == controller._executor_label()
    assert capture_metrics["time_step_ms"] == pytest.approx(4.0)
    assert capture_metrics["world_time"] > 0.0
    assert set(capture_metrics["lanes"]) == set(metrics)
    assert float(metrics["single"]["body_count"]) == 1.0
    assert float(metrics["contact"]["body_count"]) == 1.0
    assert float(metrics["stack"]["body_count"]) == 4.0
    assert float(metrics["stack"]["entity_count"]) > float(
        metrics["contact"]["entity_count"]
    )
    assert float(metrics["contact"]["entity_count"]) >= float(
        metrics["single"]["entity_count"]
    )
    assert float(metrics["stack"]["component_count"]) > float(
        metrics["contact"]["component_count"]
    )
    assert float(metrics["stack"]["contact_count"]) >= float(
        metrics["contact"]["contact_count"]
    )

    for key, lane_metrics in metrics.items():
        capture_lane = capture_metrics["lanes"][key]
        assert capture_lane["status"] == lane_metrics["status"]
        assert float(capture_lane["body_count"]) == pytest.approx(
            float(lane_metrics["body_count"])
        )
        assert float(capture_lane["frame_scratch_capacity_kib"]) > 0.0
        assert np.isfinite(float(lane_metrics["max_speed"]))
        assert float(lane_metrics["frame_scratch_capacity_kib"]) > 0.0
        assert float(lane_metrics["frame_scratch_peak_kib"]) >= 0.0
        assert float(lane_metrics["frame_scratch_overflow_count"]) >= 0.0
        assert float(lane_metrics["frame_scratch_reset_count"]) >= 0.0
        assert float(lane_metrics["accelerated_stage_count"]) >= 0.0
        assert float(lane_metrics["accelerated_stage_count"]) <= float(
            lane_metrics["stage_count"]
        )
        assert bool(lane_metrics["accelerated_backend_active"]) is (
            float(lane_metrics["accelerated_stage_count"]) > 0.0
        )
        assert isinstance(lane_metrics["top_stage_accelerated_backend"], bool)
        assert controller._wall_ms_history[key]
        assert controller._stage_ms_history[key]
        assert controller._contact_history[key]
        assert controller._scratch_peak_history[key]
        assert controller._entity_history[key]

    profiled = [
        lane_metrics
        for lane_metrics in metrics.values()
        if not bool(lane_metrics["profile_empty"])
    ]
    if profiled:
        for lane_metrics in profiled:
            assert float(lane_metrics["stage_count"]) > 0.0
            assert float(lane_metrics["wall_ms"]) >= 0.0
            assert float(lane_metrics["stage_ms"]) >= 0.0
            assert str(lane_metrics["top_stage"]) != "none"
            assert str(lane_metrics["top_stage_domain"]) != "none"
            assert str(lane_metrics["top_stage_acceleration"]) != "none"


def test_rigid_contact_scale_budget_orders_contact_loads() -> None:
    import numpy as np

    _require_simulation_symbols("World", "WorldMemoryDiagnostics", "WorldStepProfile")

    from examples.demos.scenes.rigid_contact_scale_budget import build

    setup = build()
    controller = setup.info["rigid_contact_scale_budget_controller"]
    assert setup.pre_step is not None
    for _ in range(20):
        setup.pre_step()

    metrics = controller._last_metrics
    capture_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert set(metrics) == {"single", "medium", "dense"}
    assert capture_metrics["row"] == "rigid_contact_scale_budget"
    assert capture_metrics["solver"] == controller._solver_label()
    assert capture_metrics["executor"] == controller._executor_label()
    assert capture_metrics["budget_ms"] == pytest.approx(controller.budget_ms)
    assert capture_metrics["friction"] == pytest.approx(controller.friction)
    assert capture_metrics["time_step_ms"] == pytest.approx(4.0)
    assert capture_metrics["world_time"] > 0.0
    assert set(capture_metrics["lanes"]) == set(metrics)
    assert float(metrics["single"]["body_count"]) == pytest.approx(1.0)
    assert float(metrics["medium"]["body_count"]) == pytest.approx(4.0)
    assert float(metrics["dense"]["body_count"]) == pytest.approx(9.0)
    assert float(metrics["medium"]["entity_count"]) > float(
        metrics["single"]["entity_count"]
    )
    assert float(metrics["dense"]["entity_count"]) > float(
        metrics["medium"]["entity_count"]
    )
    assert float(metrics["medium"]["component_count"]) > float(
        metrics["single"]["component_count"]
    )
    assert float(metrics["dense"]["component_count"]) > float(
        metrics["medium"]["component_count"]
    )
    assert float(metrics["medium"]["contact_count"]) >= float(
        metrics["single"]["contact_count"]
    )
    assert float(metrics["dense"]["contact_count"]) >= float(
        metrics["medium"]["contact_count"]
    )
    assert float(metrics["dense"]["contact_count"]) >= 6.0
    assert float(metrics["dense"]["target_contacts"]) == pytest.approx(9.0)
    assert controller._dense_single_ratio

    for key, lane_metrics in metrics.items():
        capture_lane = capture_metrics["lanes"][key]
        assert capture_lane["status"] == lane_metrics["status"]
        assert float(capture_lane["body_count"]) == pytest.approx(
            float(lane_metrics["body_count"])
        )
        assert float(capture_lane["contact_count"]) == pytest.approx(
            float(lane_metrics["contact_count"])
        )
        assert float(capture_lane["budget_ms"]) == pytest.approx(controller.budget_ms)
        assert np.isfinite(float(lane_metrics["wall_per_contact_us"]))
        assert float(lane_metrics["wall_per_contact_us"]) >= 0.0
        assert float(lane_metrics["frame_scratch_capacity_kib"]) > 0.0
        assert float(lane_metrics["frame_scratch_peak_kib"]) >= 0.0
        assert float(lane_metrics["budget_ms"]) == pytest.approx(controller.budget_ms)
        assert str(lane_metrics["status"]) in {
            "within budget",
            "over budget",
            "profiling unavailable",
        }
        if not bool(lane_metrics["profile_empty"]):
            assert float(lane_metrics["stage_count"]) > 0.0
            assert float(lane_metrics["wall_ms"]) >= 0.0
            assert str(lane_metrics["top_stage"]) != "none"
        assert controller._wall_ms_history[key]
        assert controller._contact_history[key]
        assert controller._scratch_peak_history[key]
        assert controller._per_contact_history[key]
        assert controller._budget_over_history[key]


def test_rigid_collision_query_options_filter_body_kinds() -> None:
    from examples.demos.scenes.rigid_collision_query_options import build

    setup = build()
    controller = setup.info["rigid_collision_query_options_controller"]
    for _ in range(3):
        assert setup.pre_step is not None
        setup.pre_step()

    metrics = controller._last_metrics
    assert {lane.key for lane in controller.lanes} == {
        "rigid_rigid",
        "rigid_link",
        "same_links",
        "cross_links",
    }
    assert metrics["baseline_contact_count"] == 4
    assert metrics["active_contact_count"] == 4
    assert metrics["filtered_contact_count"] == 0
    expected_body_kinds = {
        "rigid_rigid": ("rigid", "rigid"),
        "rigid_link": ("rigid", "link"),
        "same_links": ("link", "link"),
        "cross_links": ("link", "link"),
    }
    for lane in controller.lanes:
        lane_metrics = metrics["lanes"][lane.key]
        assert lane_metrics["baseline_count"] == 1
        assert lane_metrics["active_count"] == 1
        assert lane_metrics["first_depth"] == pytest.approx(0.06, abs=1.0e-12)
        assert tuple(lane_metrics["first_shape_indices"]) == (0, 0)
        assert sorted(lane_metrics["first_body_kinds"]) == sorted(
            expected_body_kinds[lane.key]
        )
        assert tuple(lane_metrics["first_body_valid"]) == (True, True)
        assert len(lane_metrics["first_bodies"]) == 2
        for body in lane_metrics["first_bodies"]:
            kind = body["kind"]
            assert kind in {"rigid", "link"}
            assert body["valid"] is True
            assert body["rigid_cast"] is (kind == "rigid")
            assert body["link_cast"] is (kind == "link")
        assert " -> " in lane_metrics["first_body_pair"]

    controller.include_rigid_body_pairs = False
    controller._record_metrics()
    lanes = controller._last_metrics["lanes"]
    assert lanes["rigid_rigid"]["filtered"] is True
    assert lanes["rigid_link"]["active_count"] == 1
    assert lanes["same_links"]["active_count"] == 1
    assert lanes["cross_links"]["active_count"] == 1

    controller._apply_preset("all")
    controller.include_rigid_body_link_pairs = False
    controller._record_metrics()
    lanes = controller._last_metrics["lanes"]
    assert lanes["rigid_link"]["filtered"] is True
    assert lanes["rigid_rigid"]["active_count"] == 1
    assert lanes["same_links"]["active_count"] == 1
    assert lanes["cross_links"]["active_count"] == 1

    controller._apply_preset("all")
    controller.include_link_pairs = False
    controller._record_metrics()
    lanes = controller._last_metrics["lanes"]
    assert lanes["same_links"]["filtered"] is True
    assert lanes["cross_links"]["filtered"] is True
    assert lanes["rigid_rigid"]["active_count"] == 1
    assert lanes["rigid_link"]["active_count"] == 1

    controller._apply_preset("all")
    controller.include_same_multibody_link_pairs = False
    controller._record_metrics()
    lanes = controller._last_metrics["lanes"]
    assert lanes["same_links"]["filtered"] is True
    assert lanes["cross_links"]["active_count"] == 1
    assert lanes["rigid_rigid"]["active_count"] == 1
    assert lanes["rigid_link"]["active_count"] == 1


def test_rigid_collision_casts_report_nearest_all_and_swept_hits() -> None:
    import numpy as np

    from examples.demos.scenes.rigid_collision_casts import build

    setup = build()
    controller = setup.info["rigid_collision_casts_controller"]
    for _ in range(3):
        assert setup.pre_step is not None
        setup.pre_step()

    metrics = controller._last_metrics
    ray = metrics["ray"]
    sphere_cast = metrics["sphere_cast"]
    capsule_cast = metrics["capsule_cast"]
    assert int(ray["hit_count"]) == 2
    assert ray["first_target"] == "near_sensor_target"
    assert ray["all_targets"] == ["near_sensor_target", "far_sensor_target"]
    assert float(ray["all_fractions"][0]) < float(ray["all_fractions"][1])
    assert float(ray["first_fraction"]) == pytest.approx(0.25909090909090915)
    assert np.asarray(ray["first_point"], dtype=float).tolist() == pytest.approx(
        [-0.53, 0.0, 0.25]
    )
    assert np.asarray(ray["first_normal"], dtype=float).tolist() == pytest.approx(
        [-1.0, 0.0, 0.0]
    )

    assert int(sphere_cast["hit_count"]) == 2
    assert sphere_cast["first_target"] == "near_sensor_target"
    assert sphere_cast["all_targets"] == ["near_sensor_target", "far_sensor_target"]
    assert float(sphere_cast["all_toi"][0]) < float(sphere_cast["all_toi"][1])
    assert 0.0 < float(sphere_cast["first_toi"]) < 1.0
    assert float(sphere_cast["margin"]) > 0.0

    assert int(capsule_cast["hit_count"]) == 2
    assert capsule_cast["first_target"] == "near_sensor_target"
    assert capsule_cast["all_targets"] == ["near_sensor_target", "far_sensor_target"]
    assert float(capsule_cast["all_toi"][0]) < float(capsule_cast["all_toi"][1])
    assert 0.0 < float(capsule_cast["first_toi"]) < 1.0
    assert float(capsule_cast["margin"]) > 0.0
    assert np.isfinite(np.asarray(capsule_cast["first_point"], dtype=float)).all()
    assert np.isfinite(np.asarray(capsule_cast["first_normal"], dtype=float)).all()
    assert np.linalg.norm(np.asarray(capsule_cast["first_normal"], dtype=float)) > 0.0
    assert controller._ray_hit_history
    assert controller._sphere_cast_toi_history
    assert controller._sweep_margin_history
    assert controller._capsule_cast_toi_history
    assert controller._capsule_margin_history

    controller.enable_all_ray_hits = False
    controller._record_metrics()
    assert int(controller._last_metrics["ray"]["hit_count"]) == 1

    controller.ray_offset_y = 0.30
    controller._record_metrics()
    assert int(controller._last_metrics["ray"]["hit_count"]) == 0

    controller.sweep_radius = 0.12
    controller._record_metrics()
    assert int(controller._last_metrics["sphere_cast"]["hit_count"]) == 0
    assert float(controller._last_metrics["sphere_cast"]["margin"]) < 0.0

    controller.capsule_offset_y = 1.0
    controller._record_metrics()
    assert int(controller._last_metrics["capsule_cast"]["hit_count"]) == 0
    assert float(controller._last_metrics["capsule_cast"]["margin"]) < 0.0


def test_rigid_solver_compare_records_wall_response() -> None:
    import numpy as np

    from examples.demos.scenes.rigid_solver_compare import build

    setup = build()
    controller = setup.info["rigid_solver_compare_controller"]
    for _ in range(120):
        assert setup.pre_step is not None
        setup.pre_step()

    assert controller._delta_history
    assert max(controller._delta_history) > 1.0e-4
    timeline = setup.info["replay_timeline"]
    snapshot = controller.capture_replay_state()
    assert timeline["signal_label"] == "Position divergence"
    assert timeline["signal"](snapshot) == pytest.approx(controller._delta_history[-1])
    assert (
        timeline["markers"]({"clearance_history": {"case": [0.12, 0.049]}})
        == pytest.approx(1.0)
    )
    assert (
        timeline["markers"]({"clearance_history": {"case": [0.12, 0.09]}})
        == pytest.approx(0.0)
    )
    for case in controller.cases:
        metrics = controller._last_metrics[case.label]
        assert np.isfinite(float(metrics["speed"]))
        assert np.isfinite(float(metrics["clearance"]))
        assert np.isfinite(float(metrics["step_ms"]))
        assert controller._speed_history[case.label]
        assert controller._clearance_history[case.label]
        assert controller._step_ms_history[case.label]
        assert min(controller._clearance_history[case.label]) < 0.05


def test_rigid_restitution_ladder_orders_rebound_height() -> None:
    import numpy as np

    from examples.demos.scenes.rigid_restitution_ladder import build

    setup = build()
    controller = setup.info["rigid_restitution_ladder_controller"]
    for _ in range(220):
        assert setup.pre_step is not None
        setup.pre_step()

    dead = controller._last_metrics["dead"]
    middle = controller._last_metrics["middle"]
    high = controller._last_metrics["high"]
    assert controller._max_rebound_height["dead"] < 0.12
    assert controller._max_rebound_height["middle"] > 0.18
    assert controller._max_rebound_height["high"] > 2.0 * controller._max_rebound_height[
        "middle"
    ]
    assert controller._max_upward_velocity["high"] > controller._max_upward_velocity[
        "middle"
    ]
    for key, metrics in controller._last_metrics.items():
        assert controller._had_contact[key] is True
        assert max(controller._contact_history[key]) >= 1.0
        assert np.isfinite(float(metrics["height"]))
        assert np.isfinite(float(metrics["vertical_velocity"]))
        assert np.isfinite(float(metrics["kinetic_energy"]))
        assert np.isfinite(float(metrics["potential_energy"]))
        assert np.isfinite(float(metrics["total_energy"]))
        assert controller._height_history[key]
        assert controller._velocity_history[key]
        assert controller._energy_history[key]
    assert float(dead["restitution"]) == pytest.approx(0.0)
    assert float(middle["restitution"]) == pytest.approx(0.5)
    assert float(high["restitution"]) == pytest.approx(0.9)


def test_rigid_material_mixing_applies_pair_rules() -> None:
    import math
    import numpy as np

    from examples.demos.scenes.rigid_material_mixing import build

    setup = build()
    controller = setup.info["rigid_material_mixing_controller"]
    for _ in range(180):
        assert setup.pre_step is not None
        setup.pre_step()

    metrics = controller._last_metrics
    expected_restitution = controller.high_restitution
    expected_friction = math.sqrt(controller.low_friction * controller.high_friction)
    for key in (
        "bounce_body_high",
        "bounce_surface_high",
        "slide_body_high",
        "slide_surface_high",
    ):
        lane_metrics = metrics[key]
        assert float(lane_metrics["effective_restitution"]) == pytest.approx(
            expected_restitution
        )
        assert float(lane_metrics["effective_friction"]) == pytest.approx(
            expected_friction
        )
        assert max(controller._contact_history[key]) >= 1.0
        assert np.isfinite(float(lane_metrics["height"]))
        assert np.isfinite(float(lane_metrics["tangential_speed"]))
        assert np.isfinite(float(lane_metrics["vertical_velocity"]))

    assert controller._max_rebound_height["bounce_body_high"] > 0.18
    assert controller._max_rebound_height["bounce_surface_high"] > 0.18
    assert controller._max_rebound_height["bounce_body_high"] == pytest.approx(
        controller._max_rebound_height["bounce_surface_high"], abs=1.0e-9
    )
    assert metrics["slide_body_high"]["speed_loss"] > 0.5
    assert metrics["slide_surface_high"]["speed_loss"] > 0.5
    assert metrics["slide_body_high"]["speed_loss"] == pytest.approx(
        metrics["slide_surface_high"]["speed_loss"], abs=1.0e-9
    )


def test_rigid_contact_manipulation_pushes_target_toward_goal() -> None:
    import numpy as np

    from examples.demos.scenes.rigid_contact_manipulation import build

    setup = build()
    controller = setup.info["rigid_contact_manipulation_controller"]
    for _ in range(96):
        assert setup.pre_step is not None
        setup.pre_step()

    assert controller._divergence_history
    assert max(controller._divergence_history) > 1.0e-4
    for case in controller.cases:
        metrics = controller._last_metrics[case.label]
        assert float(metrics["target_travel"]) > 0.08
        assert float(metrics["goal_error"]) < 0.30
        assert float(metrics["lateral_drift"]) < 0.05
        assert np.isfinite(float(metrics["target_speed"]))
        assert np.isfinite(float(metrics["step_ms"]))
        assert controller._travel_history[case.label]
        assert controller._gap_history[case.label]
        assert controller._contact_history[case.label]
        assert (
            max(controller._contact_history[case.label]) > 0.0
            or min(controller._gap_history[case.label]) < 0.025
        )


def test_rigid_kinematic_driver_carries_box_with_ipc() -> None:
    import numpy as np

    from examples.demos.scenes.rigid_kinematic_driver import build

    setup = build()
    controller = setup.info["rigid_kinematic_driver_controller"]
    for _ in range(96):
        assert setup.pre_step is not None
        setup.pre_step()

    metrics = controller._last_metrics
    assert {case.key for case in controller.cases} == {
        "ipc_grip",
        "ipc_slip",
        "si_caveat",
    }

    grip = metrics["ipc_grip"]
    slip = metrics["ipc_slip"]
    caveat = metrics["si_caveat"]

    assert float(grip["driver_travel"]) > 0.10
    assert float(grip["box_travel"]) > 0.07
    assert float(grip["speed_ratio"]) > 0.55
    assert abs(float(grip["support_gap"])) < 0.020

    assert float(slip["driver_travel"]) > 0.10
    assert abs(float(slip["box_travel"])) < 0.035
    assert float(slip["slip"]) > 0.08
    assert abs(float(slip["support_gap"])) < 0.020

    assert abs(float(caveat["driver_travel"])) < 1.0e-6
    assert abs(float(caveat["box_travel"])) < 0.035
    assert "static-like" in str(caveat["status"])

    for case in controller.cases:
        assert controller._driver_history[case.key]
        assert controller._box_history[case.key]
        assert controller._slip_history[case.key]
        assert controller._ratio_history[case.key]
        assert np.isfinite(float(metrics[case.key]["step_ms"]))


def test_rigid_contact_solver_compare_records_coupled_contact_policy() -> None:
    import numpy as np

    from examples.demos.scenes.rigid_contact_solver_compare import build

    setup = build()
    controller = setup.info["rigid_contact_solver_compare_controller"]
    for _ in range(96):
        assert setup.pre_step is not None
        setup.pre_step()

    assert controller._divergence_history
    assert max(controller._divergence_history) > 1.0e-4
    timeline = setup.info["replay_timeline"]
    snapshot = controller.capture_replay_state()
    assert timeline["signal_label"] == "Pose divergence"
    assert timeline["signal"](snapshot) == pytest.approx(
        controller._divergence_history[-1]
    )
    assert (
        timeline["markers"](
            {
                "contact_history": {"case": [0.0, 2.0]},
                "depth_history": {"case": [0.0, 0.0]},
            }
        )
        == pytest.approx(1.0)
    )
    assert (
        timeline["markers"](
            {
                "contact_history": {"case": [0.0, 1.0]},
                "depth_history": {"case": [0.0, 0.001]},
            }
        )
        == pytest.approx(1.0)
    )
    assert (
        timeline["markers"](
            {
                "contact_history": {"case": [0.0, 1.0]},
                "depth_history": {"case": [0.0, 0.0]},
            }
        )
        == pytest.approx(0.0)
    )
    methods = [case.world.contact_solver_method for case in controller.cases]
    assert methods[0].name == "SEQUENTIAL_IMPULSE"
    assert methods[1].name == "BOXED_LCP"
    for case in controller.cases:
        metrics = controller._last_metrics[case.label]
        assert float(metrics["contact_count"]) >= 2.0
        assert float(metrics["max_depth"]) < 0.010
        assert abs(float(metrics["clearance"])) < 0.020
        assert np.isfinite(float(metrics["speed"]))
        assert np.isfinite(float(metrics["angular_speed"]))
        assert np.isfinite(float(metrics["energy"]))
        assert np.isfinite(float(metrics["step_ms"]))
        assert controller._contact_history[case.label]
        assert max(controller._contact_history[case.label]) >= 2.0
        assert controller._depth_history[case.label]


def test_rigid_link_contact_exercises_multibody_contact_response() -> None:
    from examples.demos.scenes.contact import build

    setup = build()
    controller = setup.info["rigid_link_contact_controller"]
    for _ in range(120):
        assert setup.pre_step is not None
        setup.pre_step()

    mid_metrics = dict(controller._last_metrics)
    assert max(controller._push_contact_history) >= 1.0

    for _ in range(530):
        setup.pre_step()

    metrics = controller._last_metrics
    assert float(metrics["drop_contact_count"]) >= 1.0
    assert float(metrics["drop_height"]) == pytest.approx(-0.30, abs=2.0e-2)
    assert float(metrics["drop_max_upward_velocity"]) > 0.50
    assert float(metrics["slide_contact_count"]) >= 1.0
    assert float(metrics["slide_speed"]) < 0.08
    assert float(metrics["slide_travel"]) > 0.02
    assert abs(float(metrics["slide_height_error"])) < 2.0e-2
    assert float(metrics["push_target_travel"]) > 0.05
    assert abs(float(metrics["push_target_speed"])) > 1.0e-3
    assert "link" in str(metrics["contact_body_kinds"])

    assert float(mid_metrics["push_striker_speed"]) > float(
        metrics["push_striker_speed"]
    )
    assert controller._drop_height_history
    assert controller._slide_speed_history
    assert controller._push_target_travel_history
    assert controller.ground.friction == pytest.approx(controller.ground_friction)
    assert controller.ground.restitution == pytest.approx(
        controller.ground_restitution
    )


def test_rigid_verifier_replay_snapshots_restore_controls() -> None:
    from examples.demos.scenes.contact import build as link_contact_build
    from examples.demos.scenes.rigid_body import build as rigid_body_build
    from examples.demos.scenes.rigid_body_modes import build as body_modes_build
    from examples.demos.scenes.rigid_collision_query_options import (
        build as query_options_build,
    )
    from examples.demos.scenes.rigid_collision_casts import build as casts_build
    from examples.demos.scenes.rigid_contact_inspector import build as contact_build
    from examples.demos.scenes.rigid_contact_manipulation import (
        build as manipulation_build,
    )
    from examples.demos.scenes.rigid_contact_scale_budget import (
        build as contact_scale_budget_build,
    )
    from examples.demos.scenes.rigid_contact_solver_compare import (
        build as contact_solver_build,
    )
    from examples.demos.scenes.rigid_executor_equivalence import (
        build as executor_build,
    )
    from examples.demos.scenes.rigid_external_loads import (
        build as external_loads_build,
    )
    from examples.demos.scenes.rigid_fixed_joint import build as fixed_joint_build
    from examples.demos.scenes.rigid_friction_threshold import build as friction_build
    from examples.demos.scenes.rigid_frame_hierarchy import build as frame_build
    from examples.demos.scenes.rigid_free_flight import build as free_flight_build
    from examples.demos.scenes.rigid_joint_motor_limits import (
        build as motor_limit_build,
    )
    from examples.demos.scenes.rigid_joint_passive_parameters import (
        build as passive_joint_build,
    )
    from examples.demos.scenes.rigid_kinematic_driver import build as kinematic_build
    from examples.demos.scenes.rigid_link_point_loads import (
        build as point_loads_build,
    )
    from examples.demos.scenes.rigid_link_center_of_mass import (
        build as link_com_build,
    )
    from examples.demos.scenes.rigid_link_jacobian import (
        build as link_jacobian_build,
    )
    from examples.demos.scenes.rigid_limited_joints import build as limited_joint_build
    from examples.demos.scenes.rigid_loop_closure import build as loop_closure_build
    from examples.demos.scenes.rigid_material_mixing import (
        build as material_mixing_build,
    )
    from examples.demos.scenes.rigid_multibody_dynamics_terms import (
        build as multibody_dynamics_terms_build,
    )
    from examples.demos.scenes.rigid_multibody_solver_family import (
        build as multibody_solver_family_build,
    )
    from examples.demos.scenes.rigid_restitution_ladder import (
        build as restitution_build,
    )
    from examples.demos.scenes.rigid_screw_joint_pitch import (
        build as screw_joint_build,
    )
    from examples.demos.scenes.rigid_solver_compare import build as solver_build
    from examples.demos.scenes.rigid_spin_roll_coupling import (
        build as spin_roll_build,
    )
    from examples.demos.scenes.rigid_stack_stability import build as stack_build
    from examples.demos.scenes.rigid_step_diagnostics import (
        build as step_diagnostics_build,
    )
    from examples.demos.scenes.rigid_timestep_sensitivity import (
        build as timestep_build,
    )

    contact = contact_build().info["rigid_contact_inspector_controller"]
    contact.pair_index = len(contact.lanes) - 1
    contact.penetration = 0.073
    contact_state = contact.capture_replay_state()
    contact.pair_index = 0
    contact.penetration = 0.01
    contact.restore_replay_state(contact_state)
    assert contact.pair_index == len(contact.lanes) - 1
    assert contact.penetration == pytest.approx(0.073)

    query_options = query_options_build().info[
        "rigid_collision_query_options_controller"
    ]
    query_options.include_rigid_body_pairs = False
    query_options.include_rigid_body_link_pairs = True
    query_options.include_link_pairs = True
    query_options.include_same_multibody_link_pairs = False
    query_options_state = query_options.capture_replay_state()
    query_options.include_rigid_body_pairs = True
    query_options.include_rigid_body_link_pairs = False
    query_options.include_link_pairs = False
    query_options.include_same_multibody_link_pairs = True
    query_options.restore_replay_state(query_options_state)
    assert query_options.include_rigid_body_pairs is False
    assert query_options.include_rigid_body_link_pairs is True
    assert query_options.include_link_pairs is True
    assert query_options.include_same_multibody_link_pairs is False

    casts = casts_build().info["rigid_collision_casts_controller"]
    casts.ray_offset_y = 0.21
    casts.enable_all_ray_hits = False
    casts.sweep_radius = 0.41
    casts.capsule_offset_y = 0.73
    casts.capsule_radius = 0.17
    casts.capsule_height = 1.05
    casts_state = casts.capture_replay_state()
    casts.ray_offset_y = 0.0
    casts.enable_all_ray_hits = True
    casts.sweep_radius = 0.12
    casts.capsule_offset_y = 0.35
    casts.capsule_radius = 0.08
    casts.capsule_height = 0.25
    casts.restore_replay_state(casts_state)
    assert casts.ray_offset_y == pytest.approx(0.21)
    assert casts.enable_all_ray_hits is False
    assert casts.sweep_radius == pytest.approx(0.41)
    assert casts.capsule_offset_y == pytest.approx(0.73)
    assert casts.capsule_radius == pytest.approx(0.17)
    assert casts.capsule_height == pytest.approx(1.05)

    baseline = rigid_body_build().info["rigid_body_controller"]
    baseline.solver_index = 1
    baseline.friction = 0.37
    baseline.restitution = 0.42
    baseline_state = baseline.capture_replay_state()
    baseline.solver_index = 0
    baseline.friction = 0.91
    baseline.restitution = 0.05
    baseline.restore_replay_state(baseline_state)
    assert baseline.solver_index == 1
    assert baseline.friction == pytest.approx(0.37)
    assert baseline.restitution == pytest.approx(0.42)
    assert baseline.world.rigid_body_solver == baseline._solver()
    for body in baseline.dynamic_bodies:
        assert body.friction == pytest.approx(0.37)
        assert body.restitution == pytest.approx(0.42)

    loads = external_loads_build().info["rigid_external_loads_controller"]
    loads.executor_index = len(loads._executors) - 1
    loads.force_magnitude = 6.0
    loads.torque_magnitude = 0.22
    loads.mass_ratio = 6.0
    loads.inertia_ratio = 7.0
    loads_state = loads.capture_replay_state()
    loads.executor_index = 0
    loads.force_magnitude = 1.0
    loads.torque_magnitude = 0.05
    loads.mass_ratio = 2.0
    loads.inertia_ratio = 3.0
    loads.restore_replay_state(loads_state)
    assert loads.executor_index == len(loads._executors) - 1
    assert loads.force_magnitude == pytest.approx(6.0)
    assert loads.torque_magnitude == pytest.approx(0.22)
    assert loads.mass_ratio == pytest.approx(6.0)
    assert loads.inertia_ratio == pytest.approx(7.0)
    assert loads._lane("heavy_force").body.mass == pytest.approx(6.0)
    low_inertia_z = float(loads._lane("low_inertia_torque").body.inertia[2, 2])
    high_inertia_z = float(loads._lane("high_inertia_torque").body.inertia[2, 2])
    assert high_inertia_z == pytest.approx(7.0 * low_inertia_z)
    assert loads._lane("light_force").body.force[0] == pytest.approx(6.0)
    assert loads._lane("low_inertia_torque").body.torque[2] == pytest.approx(0.22)

    point_loads = point_loads_build().info["rigid_link_point_loads_controller"]
    point_loads.executor_index = len(point_loads._executors) - 1
    point_loads.force_magnitude = 7.0
    point_loads.point_offset = 0.27
    point_loads.yaw_degrees = 55.0
    point_loads_state = point_loads.capture_replay_state()
    point_loads.executor_index = 0
    point_loads.force_magnitude = 2.0
    point_loads.point_offset = 0.04
    point_loads.yaw_degrees = 10.0
    point_loads.restore_replay_state(point_loads_state)
    assert point_loads.executor_index == len(point_loads._executors) - 1
    assert point_loads.force_magnitude == pytest.approx(7.0)
    assert point_loads.point_offset == pytest.approx(0.27)
    assert point_loads.yaw_degrees == pytest.approx(55.0)

    free_flight = free_flight_build().info["rigid_free_flight_controller"]
    free_flight.executor_index = len(free_flight._executors) - 1
    free_flight.launch_speed = 1.8
    free_flight.launch_angle_deg = 41.0
    free_flight.gravity_scale = 0.7
    free_flight.spin_speed = 4.5
    free_flight.inertia_ratio = 6.0
    free_flight_state = free_flight.capture_replay_state()
    free_flight.executor_index = 0
    free_flight.launch_speed = 0.4
    free_flight.launch_angle_deg = 5.0
    free_flight.gravity_scale = 0.2
    free_flight.spin_speed = 1.0
    free_flight.inertia_ratio = 2.0
    free_flight.restore_replay_state(free_flight_state)
    assert free_flight.executor_index == len(free_flight._executors) - 1
    assert free_flight.launch_speed == pytest.approx(1.8)
    assert free_flight.launch_angle_deg == pytest.approx(41.0)
    assert free_flight.gravity_scale == pytest.approx(0.7)
    assert free_flight.spin_speed == pytest.approx(4.5)
    assert free_flight.inertia_ratio == pytest.approx(6.0)
    assert free_flight.arc_world.gravity[2] == pytest.approx(-9.81 * 0.7)
    low_inertia_z = float(free_flight.spin_low_body.inertia[2, 2])
    high_inertia_z = float(free_flight.spin_high_body.inertia[2, 2])
    assert high_inertia_z == pytest.approx(6.0 * low_inertia_z)

    frame_hierarchy = frame_build().info["rigid_frame_hierarchy_controller"]
    frame_hierarchy.executor_index = len(frame_hierarchy._executors) - 1
    frame_hierarchy.body_yaw_speed = 1.4
    frame_hierarchy.path_radius = 0.33
    frame_hierarchy.local_offset_x = 0.42
    frame_hierarchy.local_offset_y = -0.14
    frame_hierarchy.local_yaw_deg = 62.0
    frame_state = frame_hierarchy.capture_replay_state()
    frame_hierarchy.executor_index = 0
    frame_hierarchy.body_yaw_speed = 0.2
    frame_hierarchy.path_radius = 0.08
    frame_hierarchy.local_offset_x = 0.05
    frame_hierarchy.local_offset_y = 0.09
    frame_hierarchy.local_yaw_deg = -15.0
    frame_hierarchy.restore_replay_state(frame_state)
    assert frame_hierarchy.executor_index == len(frame_hierarchy._executors) - 1
    assert frame_hierarchy.body_yaw_speed == pytest.approx(1.4)
    assert frame_hierarchy.path_radius == pytest.approx(0.33)
    assert frame_hierarchy.local_offset_x == pytest.approx(0.42)
    assert frame_hierarchy.local_offset_y == pytest.approx(-0.14)
    assert frame_hierarchy.local_yaw_deg == pytest.approx(62.0)

    body_modes = body_modes_build().info["rigid_body_modes_controller"]
    body_modes.solver_index = 1
    body_modes.executor_index = len(body_modes._executors) - 1
    body_modes.gravity_scale = 0.72
    body_modes.force_magnitude = 5.5
    body_modes.drive_speed = 0.61
    body_modes_state = body_modes.capture_replay_state()
    body_modes.solver_index = 0
    body_modes.executor_index = 0
    body_modes.gravity_scale = 0.2
    body_modes.force_magnitude = 1.0
    body_modes.drive_speed = 0.12
    body_modes.restore_replay_state(body_modes_state)
    assert body_modes.solver_index == 1
    assert body_modes.executor_index == len(body_modes._executors) - 1
    assert body_modes.gravity_scale == pytest.approx(0.72)
    assert body_modes.force_magnitude == pytest.approx(5.5)
    assert body_modes.drive_speed == pytest.approx(0.61)
    assert body_modes.world.rigid_body_solver == body_modes._solver()
    assert body_modes.world.gravity[2] == pytest.approx(-9.81 * 0.72)
    assert body_modes._lane("static").body.is_static
    assert body_modes._lane("kinematic").body.is_kinematic
    assert body_modes._lane("dynamic").body.force[0] == pytest.approx(5.5)

    timestep = timestep_build().info["rigid_timestep_sensitivity_controller"]
    timestep.solver_index = 1
    timestep.executor_index = len(timestep._executors) - 1
    timestep.base_time_step = 0.003
    timestep.gravity_scale = 0.55
    timestep_state = timestep.capture_replay_state()
    timestep.solver_index = 0
    timestep.executor_index = 0
    timestep.base_time_step = 0.001
    timestep.gravity_scale = 1.8
    timestep.restore_replay_state(timestep_state)
    assert timestep.solver_index == 1
    assert timestep.executor_index == len(timestep._executors) - 1
    assert timestep.base_time_step == pytest.approx(0.003)
    assert timestep.gravity_scale == pytest.approx(0.55)
    for lane in timestep.lanes:
        assert lane.world.time_step == pytest.approx(
            timestep.base_time_step * lane.multiplier
        )
        assert lane.world.gravity[2] == pytest.approx(-9.81 * timestep.gravity_scale)
        assert lane.world.rigid_body_solver == timestep._solver()

    step_diagnostics = step_diagnostics_build().info[
        "rigid_step_diagnostics_controller"
    ]
    step_diagnostics.solver_index = 1
    step_diagnostics.executor_index = len(step_diagnostics._executors) - 1
    step_state = step_diagnostics.capture_replay_state()
    step_diagnostics.solver_index = 0
    step_diagnostics.executor_index = 0
    step_diagnostics.restore_replay_state(step_state)
    assert step_diagnostics.solver_index == 1
    assert step_diagnostics.executor_index == len(step_diagnostics._executors) - 1
    for lane in step_diagnostics.lanes:
        assert lane.world.rigid_body_solver == step_diagnostics._solver()
        assert lane.world.time_step == pytest.approx(0.004)

    contact_scale = contact_scale_budget_build().info[
        "rigid_contact_scale_budget_controller"
    ]
    contact_scale.solver_index = 1
    contact_scale.executor_index = len(contact_scale._executors) - 1
    contact_scale.budget_ms = 2.75
    contact_scale.friction = 0.44
    contact_scale_state = contact_scale.capture_replay_state()
    contact_scale.solver_index = 0
    contact_scale.executor_index = 0
    contact_scale.budget_ms = 0.25
    contact_scale.friction = 0.91
    contact_scale.restore_replay_state(contact_scale_state)
    assert contact_scale.solver_index == 1
    assert contact_scale.executor_index == len(contact_scale._executors) - 1
    assert contact_scale.budget_ms == pytest.approx(2.75)
    assert contact_scale.friction == pytest.approx(0.44)
    for lane in contact_scale.lanes:
        assert lane.world.rigid_body_solver == contact_scale._solver()
        assert lane.ground.friction == pytest.approx(0.44)
        assert lane.world.time_step == pytest.approx(0.004)

    solver = solver_build().info["rigid_solver_compare_controller"]
    solver.executor_index = len(solver._executors) - 1
    solver.launch_speed = 7.25
    solver.friction = 0.23
    solver.restitution = 0.17
    solver_state = solver.capture_replay_state()
    solver.executor_index = 0
    solver.launch_speed = 1.0
    solver.friction = 0.90
    solver.restitution = 0.55
    solver.restore_replay_state(solver_state)
    assert solver.executor_index == len(solver._executors) - 1
    assert solver.launch_speed == pytest.approx(7.25)
    assert solver.friction == pytest.approx(0.23)
    assert solver.restitution == pytest.approx(0.17)
    for case in solver.cases:
        assert case.box.friction == pytest.approx(0.23)
        assert case.box.restitution == pytest.approx(0.17)

    restitution = restitution_build().info["rigid_restitution_ladder_controller"]
    restitution.solver_index = 1
    restitution.executor_index = len(restitution._executors) - 1
    restitution.launch_height = 0.91
    restitution.restitution_scale = 0.77
    restitution_state = restitution.capture_replay_state()
    restitution.solver_index = 0
    restitution.executor_index = 0
    restitution.launch_height = 0.35
    restitution.restitution_scale = 0.25
    restitution.restore_replay_state(restitution_state)
    assert restitution.solver_index == 1
    assert restitution.executor_index == len(restitution._executors) - 1
    assert restitution.launch_height == pytest.approx(0.91)
    assert restitution.restitution_scale == pytest.approx(0.77)
    assert restitution.world.rigid_body_solver == restitution._solver()
    for lane in restitution.lanes:
        assert lane.ball.restitution == pytest.approx(
            lane.preset * restitution.restitution_scale
        )

    material = material_mixing_build().info["rigid_material_mixing_controller"]
    material.executor_index = len(material._executors) - 1
    material.low_restitution = 0.08
    material.high_restitution = 0.91
    material.low_friction = 0.06
    material.high_friction = 1.05
    material.impact_speed = 1.7
    material.tangential_speed = 1.4
    material_state = material.capture_replay_state()
    material.executor_index = 0
    material.low_restitution = 0.01
    material.high_restitution = 0.50
    material.low_friction = 0.01
    material.high_friction = 0.45
    material.impact_speed = 0.4
    material.tangential_speed = 0.5
    material.restore_replay_state(material_state)
    assert material.executor_index == len(material._executors) - 1
    assert material.low_restitution == pytest.approx(0.08)
    assert material.high_restitution == pytest.approx(0.91)
    assert material.low_friction == pytest.approx(0.06)
    assert material.high_friction == pytest.approx(1.05)
    assert material.impact_speed == pytest.approx(1.7)
    assert material.tangential_speed == pytest.approx(1.4)
    for lane in material.lanes:
        assert lane.mover.restitution == pytest.approx(material._body_restitution(lane))
        assert lane.pad.friction == pytest.approx(material._surface_friction(lane))

    contact_solver = contact_solver_build().info[
        "rigid_contact_solver_compare_controller"
    ]
    contact_solver.executor_index = len(contact_solver._executors) - 1
    contact_solver.launch_speed = 0.93
    contact_solver.friction = 0.42
    contact_solver.restitution = 0.11
    contact_solver.initial_tilt_deg = 13.0
    contact_solver_state = contact_solver.capture_replay_state()
    contact_solver.executor_index = 0
    contact_solver.launch_speed = 0.1
    contact_solver.friction = 0.9
    contact_solver.restitution = 0.4
    contact_solver.initial_tilt_deg = 2.0
    contact_solver.restore_replay_state(contact_solver_state)
    assert contact_solver.executor_index == len(contact_solver._executors) - 1
    assert contact_solver.launch_speed == pytest.approx(0.93)
    assert contact_solver.friction == pytest.approx(0.42)
    assert contact_solver.restitution == pytest.approx(0.11)
    assert contact_solver.initial_tilt_deg == pytest.approx(13.0)
    for case in contact_solver.cases:
        assert case.ground.friction == pytest.approx(0.42)
        assert case.plank.restitution == pytest.approx(0.11)

    link_contact = link_contact_build().info["rigid_link_contact_controller"]
    link_contact.executor_index = len(link_contact._executors) - 1
    link_contact.ground_friction = 0.62
    link_contact.ground_restitution = 0.71
    link_contact.drop_height = 0.33
    link_contact.slide_speed = 1.34
    link_contact.push_speed = 1.22
    link_contact_state = link_contact.capture_replay_state()
    link_contact.executor_index = 0
    link_contact.ground_friction = 0.18
    link_contact.ground_restitution = 0.05
    link_contact.drop_height = 0.07
    link_contact.slide_speed = 0.25
    link_contact.push_speed = 0.30
    link_contact.restore_replay_state(link_contact_state)
    assert link_contact.executor_index == len(link_contact._executors) - 1
    assert link_contact.ground_friction == pytest.approx(0.62)
    assert link_contact.ground_restitution == pytest.approx(0.71)
    assert link_contact.drop_height == pytest.approx(0.33)
    assert link_contact.slide_speed == pytest.approx(1.34)
    assert link_contact.push_speed == pytest.approx(1.22)
    assert link_contact.ground.friction == pytest.approx(0.62)
    assert link_contact.ground.restitution == pytest.approx(0.71)

    manipulation = manipulation_build().info["rigid_contact_manipulation_controller"]
    manipulation.executor_index = len(manipulation._executors) - 1
    manipulation.launch_speed = 1.45
    manipulation.friction = 0.22
    manipulation.pusher_mass = 11.0
    manipulation_state = manipulation.capture_replay_state()
    manipulation.executor_index = 0
    manipulation.launch_speed = 0.4
    manipulation.friction = 0.8
    manipulation.pusher_mass = 2.0
    manipulation.restore_replay_state(manipulation_state)
    assert manipulation.executor_index == len(manipulation._executors) - 1
    assert manipulation.launch_speed == pytest.approx(1.45)
    assert manipulation.friction == pytest.approx(0.22)
    assert manipulation.pusher_mass == pytest.approx(11.0)
    for case in manipulation.cases:
        assert case.ground.friction == pytest.approx(0.22)
        assert case.pusher.mass == pytest.approx(11.0)

    kinematic = kinematic_build().info["rigid_kinematic_driver_controller"]
    kinematic.executor_index = len(kinematic._executors) - 1
    kinematic.drive_speed = 0.47
    kinematic.grip_friction = 0.56
    kinematic_state = kinematic.capture_replay_state()
    kinematic.executor_index = 0
    kinematic.drive_speed = 0.15
    kinematic.grip_friction = 0.91
    kinematic.restore_replay_state(kinematic_state)
    assert kinematic.executor_index == len(kinematic._executors) - 1
    assert kinematic.drive_speed == pytest.approx(0.47)
    assert kinematic.grip_friction == pytest.approx(0.56)
    for case in kinematic.cases:
        if case.key == "ipc_slip":
            assert case.platform.friction == pytest.approx(0.0)
            assert case.box.friction == pytest.approx(0.0)
        else:
            assert case.platform.friction == pytest.approx(0.56)
            assert case.box.friction == pytest.approx(0.56)

    executor = executor_build().info["rigid_executor_equivalence_controller"]
    executor.solver_index = 1
    executor.launch_speed = 0.72
    executor.friction = 0.34
    executor.restitution = 0.21
    executor_state = executor.capture_replay_state()
    executor.solver_index = 0
    executor.launch_speed = 0.1
    executor.friction = 0.9
    executor.restitution = 0.7
    executor.restore_replay_state(executor_state)
    assert executor.solver_index == 1
    assert executor.launch_speed == pytest.approx(0.72)
    assert executor.friction == pytest.approx(0.34)
    assert executor.restitution == pytest.approx(0.21)
    for case in executor.cases:
        assert case.world.rigid_body_solver == executor._solver()
        assert case.ground.friction == pytest.approx(0.34)

    friction = friction_build().info["rigid_friction_threshold_controller"]
    friction.executor_index = len(friction._executors) - 1
    friction.angle_deg = 27.0
    friction.controlled_mu = 0.63
    friction_state = friction.capture_replay_state()
    friction.executor_index = 0
    friction.angle_deg = 9.0
    friction.controlled_mu = 0.10
    friction.restore_replay_state(friction_state)
    assert friction.executor_index == len(friction._executors) - 1
    assert friction.angle_deg == pytest.approx(27.0)
    assert friction.controlled_mu == pytest.approx(0.63)
    controlled = next(lane for lane in friction.lanes if lane.key == "controlled")
    assert controlled.box.friction == pytest.approx(0.63)

    spin_roll = spin_roll_build().info["rigid_spin_roll_coupling_controller"]
    spin_roll.executor_index = len(spin_roll._executors) - 1
    spin_roll.friction = 0.47
    spin_roll.launch_speed = 1.35
    spin_roll.backspin_ratio = -0.85
    spin_roll_state = spin_roll.capture_replay_state()
    spin_roll.executor_index = 0
    spin_roll.friction = 0.12
    spin_roll.launch_speed = 0.45
    spin_roll.backspin_ratio = -0.2
    spin_roll.restore_replay_state(spin_roll_state)
    assert spin_roll.executor_index == len(spin_roll._executors) - 1
    assert spin_roll.friction == pytest.approx(0.47)
    assert spin_roll.launch_speed == pytest.approx(1.35)
    assert spin_roll.backspin_ratio == pytest.approx(-0.85)
    assert spin_roll.ground.friction == pytest.approx(0.47)
    for lane in spin_roll.lanes:
        if lane.key == "low_friction_slip":
            assert lane.body.friction == pytest.approx(0.0)
        else:
            assert lane.body.friction == pytest.approx(0.47)

    stack = stack_build().info["rigid_stack_stability_controller"]
    stack.executor_index = len(stack._executors) - 1
    stack.friction = 0.72
    stack.top_mass_ratio = 12.0
    stack_state = stack.capture_replay_state()
    stack.executor_index = 0
    stack.friction = 0.2
    stack.top_mass_ratio = 2.0
    stack.restore_replay_state(stack_state)
    assert stack.executor_index == len(stack._executors) - 1
    assert stack.friction == pytest.approx(0.72)
    assert stack.top_mass_ratio == pytest.approx(12.0)
    for case in stack.cases:
        assert case.ground.friction == pytest.approx(0.72)
        assert case.boxes[-1].mass == pytest.approx(12.0)

    fixed = fixed_joint_build().info["rigid_fixed_joint_controller"]
    fixed.perturbation = 0.31
    fixed_state = fixed.capture_replay_state()
    fixed.perturbation = 0.02
    fixed.restore_replay_state(fixed_state)
    assert fixed.perturbation == pytest.approx(0.31)

    limited = limited_joint_build().info["rigid_one_dof_joint_controller"]
    limited.perturbation = 0.29
    limited_state = limited.capture_replay_state()
    limited.perturbation = 0.01
    limited.restore_replay_state(limited_state)
    assert limited.perturbation == pytest.approx(0.29)

    motor_limit = motor_limit_build().info["rigid_joint_motor_limit_controller"]
    motor_limit.command_speed = 0.71
    motor_limit.velocity_limit = 0.24
    motor_limit.position_limit = 0.44
    motor_limit.force_command = 18.0
    motor_limit.effort_limit = 3.5
    motor_limit_state = motor_limit.capture_replay_state()
    motor_limit.command_speed = 0.10
    motor_limit.velocity_limit = 0.45
    motor_limit.position_limit = 0.18
    motor_limit.force_command = 4.0
    motor_limit.effort_limit = 7.5
    motor_limit.restore_replay_state(motor_limit_state)
    assert motor_limit.command_speed == pytest.approx(0.71)
    assert motor_limit.velocity_limit == pytest.approx(0.24)
    assert motor_limit.position_limit == pytest.approx(0.44)
    assert motor_limit.force_command == pytest.approx(18.0)
    assert motor_limit.effort_limit == pytest.approx(3.5)
    assert motor_limit.motor_joint.command_velocity.tolist() == pytest.approx([0.71])
    assert motor_limit.motor_joint.velocity_upper_limits.tolist() == pytest.approx(
        [0.24]
    )
    assert motor_limit.limited_force_joint.effort_upper_limits.tolist() == pytest.approx(
        [3.5]
    )

    passive_joint = passive_joint_build().info[
        "rigid_joint_passive_parameters_controller"
    ]
    passive_joint.executor_index = len(passive_joint._executors) - 1
    passive_joint.spring_stiffness = 21.0
    passive_joint.damping_coefficient = 4.5
    passive_joint.rest_position = 0.12
    passive_joint.coulomb_friction = 7.5
    passive_joint.slip_force = 11.0
    passive_joint.armature = 8.0
    passive_joint_state = passive_joint.capture_replay_state()
    passive_joint.executor_index = 0
    passive_joint.spring_stiffness = 5.0
    passive_joint.damping_coefficient = 0.5
    passive_joint.rest_position = -0.10
    passive_joint.coulomb_friction = 2.0
    passive_joint.slip_force = 3.0
    passive_joint.armature = 1.0
    passive_joint.restore_replay_state(passive_joint_state)
    assert passive_joint.executor_index == len(passive_joint._executors) - 1
    assert passive_joint.spring_stiffness == pytest.approx(21.0)
    assert passive_joint.damping_coefficient == pytest.approx(4.5)
    assert passive_joint.rest_position == pytest.approx(0.12)
    assert passive_joint.coulomb_friction == pytest.approx(7.5)
    assert passive_joint.slip_force == pytest.approx(11.0)
    assert passive_joint.armature == pytest.approx(8.0)
    damped = next(
        lane for lane in passive_joint.lanes if lane.key == "spring_damper"
    )
    stiction = next(lane for lane in passive_joint.lanes if lane.key == "stiction")
    armature = next(
        lane for lane in passive_joint.lanes if lane.key == "armature_heavy"
    )
    assert damped.joint.spring_stiffness.tolist() == pytest.approx([21.0])
    assert damped.joint.damping_coefficient.tolist() == pytest.approx([4.5])
    assert damped.joint.rest_position.tolist() == pytest.approx([0.12])
    assert stiction.joint.coulomb_friction.tolist() == pytest.approx([7.5])
    assert armature.joint.armature.tolist() == pytest.approx([8.0])

    screw_joint = screw_joint_build().info["rigid_screw_joint_pitch_controller"]
    screw_joint.executor_index = len(screw_joint._executors) - 1
    screw_joint.pitch_scale = 0.33
    screw_joint.gravity_scale = 0.75
    screw_joint.moving_mass = 3.2
    screw_joint.axial_inertia = 0.42
    screw_joint_state = screw_joint.capture_replay_state()
    screw_joint.executor_index = 0
    screw_joint.pitch_scale = 0.12
    screw_joint.gravity_scale = 1.20
    screw_joint.moving_mass = 1.0
    screw_joint.axial_inertia = 0.10
    screw_joint.restore_replay_state(screw_joint_state)
    assert screw_joint.executor_index == len(screw_joint._executors) - 1
    assert screw_joint.pitch_scale == pytest.approx(0.33)
    assert screw_joint.gravity_scale == pytest.approx(0.75)
    assert screw_joint.moving_mass == pytest.approx(3.2)
    assert screw_joint.axial_inertia == pytest.approx(0.42)
    for lane in screw_joint.lanes:
        assert lane.joint.pitch == pytest.approx(screw_joint._lane_pitch(lane))
        assert lane.nut.mass == pytest.approx(3.2)
        assert float(lane.nut.inertia[2, 2]) == pytest.approx(0.42)

    dynamics_terms = multibody_dynamics_terms_build().info[
        "rigid_multibody_dynamics_terms_controller"
    ]
    dynamics_terms.executor_index = len(dynamics_terms._executors) - 1
    dynamics_terms.target_acceleration = 3.4
    dynamics_terms.joint_impulse = 4.5
    dynamics_terms.heavy_distal_mass_scale = 6.0
    dynamics_terms.gravity_scale = 0.7
    dynamics_terms_state = dynamics_terms.capture_replay_state()
    dynamics_terms.executor_index = 0
    dynamics_terms.target_acceleration = 0.8
    dynamics_terms.joint_impulse = 1.0
    dynamics_terms.heavy_distal_mass_scale = 2.0
    dynamics_terms.gravity_scale = 0.1
    dynamics_terms.restore_replay_state(dynamics_terms_state)
    assert dynamics_terms.executor_index == len(dynamics_terms._executors) - 1
    assert dynamics_terms.target_acceleration == pytest.approx(3.4)
    assert dynamics_terms.joint_impulse == pytest.approx(4.5)
    assert dynamics_terms.heavy_distal_mass_scale == pytest.approx(6.0)
    assert dynamics_terms.gravity_scale == pytest.approx(0.7)
    assert dynamics_terms.world.gravity[2] == pytest.approx(-9.81 * 0.7)
    heavy_lane = next(
        lane for lane in dynamics_terms.lanes if lane.key == "heavy_distal"
    )
    assert heavy_lane.links[1].mass == pytest.approx(0.65 * 6.0)

    link_com = link_com_build().info["rigid_link_center_of_mass_controller"]
    link_com.executor_index = len(link_com._executors) - 1
    link_com.com_offset = 0.24
    link_com.gravity_scale = 0.65
    link_com.link_mass = 3.1
    link_com.inertia_scale = 5.5
    link_com_state = link_com.capture_replay_state()
    link_com.executor_index = 0
    link_com.com_offset = 0.05
    link_com.gravity_scale = 1.25
    link_com.link_mass = 0.8
    link_com.inertia_scale = 1.5
    link_com.restore_replay_state(link_com_state)
    assert link_com.executor_index == len(link_com._executors) - 1
    assert link_com.com_offset == pytest.approx(0.24)
    assert link_com.gravity_scale == pytest.approx(0.65)
    assert link_com.link_mass == pytest.approx(3.1)
    assert link_com.inertia_scale == pytest.approx(5.5)
    positive_com_lane = next(lane for lane in link_com.lanes if lane.key == "positive")
    high_inertia_com_lane = next(
        lane for lane in link_com.lanes if lane.key == "high_inertia"
    )
    assert positive_com_lane.link.center_of_mass.tolist() == pytest.approx(
        [0.24, 0.0, 0.0]
    )
    assert high_inertia_com_lane.link.mass == pytest.approx(3.1)
    assert float(high_inertia_com_lane.link.inertia[1, 1]) == pytest.approx(
        link_com._lane_inertia_y(high_inertia_com_lane)
    )

    link_jacobian = link_jacobian_build().info["rigid_link_jacobian_controller"]
    link_jacobian.motion_speed = 1.35
    link_jacobian.elbow_phase = -0.44
    link_jacobian.wrench_force = 2.4
    link_jacobian.wrench_angle_deg = -32.0
    link_jacobian.wrench_moment = 0.31
    link_jacobian_state = link_jacobian.capture_replay_state()
    link_jacobian.motion_speed = 0.20
    link_jacobian.elbow_phase = 0.10
    link_jacobian.wrench_force = 0.2
    link_jacobian.wrench_angle_deg = 50.0
    link_jacobian.wrench_moment = -0.10
    link_jacobian.restore_replay_state(link_jacobian_state)
    assert link_jacobian.motion_speed == pytest.approx(1.35)
    assert link_jacobian.elbow_phase == pytest.approx(-0.44)
    assert link_jacobian.wrench_force == pytest.approx(2.4)
    assert link_jacobian.wrench_angle_deg == pytest.approx(-32.0)
    assert link_jacobian.wrench_moment == pytest.approx(0.31)
    assert float(link_jacobian._last_metrics["force_norm"]) == pytest.approx(2.4)
    assert float(link_jacobian._last_metrics["moment_y"]) == pytest.approx(0.31)

    multibody_solver_family = multibody_solver_family_build().info[
        "rigid_multibody_solver_family_controller"
    ]
    multibody_solver_family.executor_index = (
        len(multibody_solver_family._executors) - 1
    )
    multibody_solver_family.gravity_scale = 0.61
    multibody_state = multibody_solver_family.capture_replay_state()
    multibody_solver_family.executor_index = 0
    multibody_solver_family.gravity_scale = 1.40
    multibody_solver_family.restore_replay_state(multibody_state)
    assert multibody_solver_family.executor_index == (
        len(multibody_solver_family._executors) - 1
    )
    assert multibody_solver_family.gravity_scale == pytest.approx(0.61)
    for case in multibody_solver_family.cases:
        assert case.world.gravity[2] == pytest.approx(-9.81 * 0.61)

    loop_closure = loop_closure_build().info["rigid_loop_closure_controller"]
    loop_closure.executor_index = len(loop_closure._executors) - 1
    loop_closure.gravity_scale = 0.62
    loop_closure_state = loop_closure.capture_replay_state()
    loop_closure.executor_index = 0
    loop_closure.gravity_scale = 1.30
    loop_closure.restore_replay_state(loop_closure_state)
    assert loop_closure.executor_index == len(loop_closure._executors) - 1
    assert loop_closure.gravity_scale == pytest.approx(0.62)
    for case in loop_closure.cases:
        assert case.world.gravity[2] == pytest.approx(-9.81 * 0.62)
        assert case.closure.dynamics == case.dynamics


def test_rigid_friction_threshold_separates_stick_and_slip_lanes() -> None:
    from examples.demos.scenes.rigid_friction_threshold import build

    setup = build()
    controller = setup.info["rigid_friction_threshold_controller"]
    for _ in range(240):
        setup.pre_step()

    metrics = controller._last_metrics
    assert metrics["below"]["distance"] > 0.5
    assert abs(metrics["above"]["distance"]) < 0.05
    assert metrics["below"]["speed"] > 0.5
    assert abs(metrics["above"]["speed"]) < 0.05


def test_rigid_spin_roll_coupling_converts_slip_to_roll() -> None:
    import numpy as np

    from examples.demos.scenes.rigid_spin_roll_coupling import build

    setup = build()
    controller = setup.info["rigid_spin_roll_coupling_controller"]
    for _ in range(96):
        assert setup.pre_step is not None
        setup.pre_step()

    metrics = controller._last_metrics
    assert set(metrics) == {
        "matched_roll",
        "slide_to_roll",
        "backspin_scrub",
        "low_friction_slip",
    }
    matched = metrics["matched_roll"]
    slide = metrics["slide_to_roll"]
    backspin = metrics["backspin_scrub"]
    low_friction = metrics["low_friction_slip"]

    assert controller._last_contact_count >= 4
    assert abs(float(matched["contact_slip"])) < 0.02
    assert float(matched["travel"]) > 0.30
    assert abs(float(slide["contact_slip"])) < 0.02
    assert float(slide["spin_delta"]) > 1.0
    assert float(slide["linear_speed"]) < float(low_friction["linear_speed"])
    assert float(backspin["spin_delta"]) > 1.0
    assert abs(float(backspin["contact_slip"])) < float(
        backspin["initial_contact_slip"]
    )
    assert float(low_friction["contact_slip"]) > 0.5
    assert float(low_friction["spin_delta"]) == pytest.approx(0.0, abs=1.0e-12)

    for lane in controller.lanes:
        assert controller._slip_history[lane.key]
        assert controller._roll_ratio_history[lane.key]
        assert controller._travel_history[lane.key]
        assert controller._energy_history[lane.key]
    assert np.isfinite([float(value) for value in controller._step_ms_history]).all()


def test_rigid_executor_equivalence_keeps_parallel_rollout_matched() -> None:
    from examples.demos.scenes.rigid_executor_equivalence import build

    setup = build()
    controller = setup.info["rigid_executor_equivalence_controller"]
    for _ in range(90):
        setup.pre_step()

    assert controller._position_divergence
    assert controller._velocity_divergence
    assert controller._contact_delta
    assert max(controller._position_divergence) < 1.0e-9
    assert max(controller._velocity_divergence) < 1.0e-9
    assert max(controller._contact_delta) == 0.0
    for metrics in controller._last_metrics.values():
        assert float(metrics["contact_count"]) >= 0.0
        assert float(metrics["step_ms"]) >= 0.0


def test_rigid_stack_stability_keeps_ipc_stack_ordered() -> None:
    import numpy as np

    from examples.demos.scenes.rigid_stack_stability import build

    setup = build()
    controller = setup.info["rigid_stack_stability_controller"]
    for _ in range(45):
        setup.pre_step()

    assert set(controller._last_metrics) == {"Sequential impulse", "IPC barrier"}
    for label, metrics in controller._last_metrics.items():
        for key in (
            "top_height",
            "height_error",
            "min_clearance",
            "top_drift",
            "max_speed",
            "step_ms",
        ):
            assert np.isfinite(float(metrics[key])), (label, key)
        assert float(metrics["top_height"]) > 0.0
        assert float(metrics["step_ms"]) >= 0.0
        assert metrics["status"] in {"overlapping", "collapsed", "stable", "settling"}

        for histories in (
            controller._speed_history,
            controller._drift_history,
            controller._clearance_history,
            controller._height_history,
            controller._step_ms_history,
        ):
            values = list(histories[label])
            assert values, label
            assert np.isfinite([float(value) for value in values]).all()

    assert controller._delta_history
    assert np.isfinite([float(value) for value in controller._delta_history]).all()

    metrics = controller._last_metrics["IPC barrier"]
    assert float(metrics["min_clearance"]) > -0.002
    assert float(metrics["top_drift"]) < 0.03
    assert float(metrics["max_speed"]) < 0.02


def test_rigid_fixed_joint_verifier_restores_captured_transform() -> None:
    from examples.demos.scenes.rigid_fixed_joint import build

    setup = build()
    controller = setup.info["rigid_fixed_joint_controller"]
    controller.perturb()
    for _ in range(80):
        assert setup.pre_step is not None
        setup.pre_step()

    metrics = controller._last_metrics
    assert float(metrics["translation_error"]) < 1.0e-6
    assert float(metrics["orientation_error"]) < 1.0e-6
    assert float(metrics["payload_speed"]) < 1.0e-6
    assert controller._translation_error_history
    assert controller._orientation_error_history


def test_rigid_joint_breakage_marks_and_resets_breakage() -> None:
    import numpy as np

    _require_simulation_symbols("World")

    from examples.demos.scenes.rigid_joint_breakage import SCENE, build

    assert SCENE.category == "World Rigid Body"
    assert SCENE.summary.startswith("AVBD-pinned")

    setup = build()
    sx_world = setup.info["sx_world"]
    joint = setup.info["joint"]
    base = setup.info["base"]
    payload = setup.info["payload"]
    break_force = float(setup.info["break_force"])

    assert sx_world.num_rigid_body_fixed_joints == 1
    assert joint.break_force == pytest.approx(break_force)
    assert not joint.is_broken
    assert [panel.title for panel in setup.panels] == ["Rigid Joint Breakage"]

    initial_base = np.asarray(base.translation, dtype=float).reshape(3)
    initial_payload = np.asarray(payload.translation, dtype=float).reshape(3)
    for _ in range(45):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    assert joint.is_broken
    base_translation = np.asarray(base.translation, dtype=float).reshape(3)
    payload_translation = np.asarray(payload.translation, dtype=float).reshape(3)
    assert np.linalg.norm(base_translation - initial_base) < 1.0e-9
    assert np.linalg.norm(payload_translation - initial_payload) > 1.0e-2

    setup.info["reset_breakage_lifecycle"]()
    assert not joint.is_broken
    reset_payload = np.asarray(payload.translation, dtype=float).reshape(3)
    assert np.linalg.norm(reset_payload - initial_payload) < 1.0e-9
    assert sx_world.time == pytest.approx(0.0)


def test_rigid_one_dof_joint_verifier_preserves_locked_directions() -> None:
    from examples.demos.scenes.rigid_limited_joints import build

    setup = build()
    controller = setup.info["rigid_one_dof_joint_controller"]
    controller.perturb()
    for _ in range(80):
        assert setup.pre_step is not None
        setup.pre_step()

    metrics = controller._last_metrics
    assert float(metrics["hinge_radius_error"]) < 1.0e-6
    assert float(metrics["hinge_z_error"]) < 1.0e-6
    assert float(metrics["slider_orthogonal_error"]) < 1.0e-6
    assert abs(float(metrics["hinge_yaw"])) > 0.1
    assert float(metrics["slider_axis_travel"]) > 0.65
    assert controller._hinge_radius_error_history
    assert controller._slider_orthogonal_error_history


def test_rigid_joint_motor_limits_clamp_commands_and_effort() -> None:
    import numpy as np

    sx = _require_simulation_symbols("World", "ActuatorType")

    from examples.demos.scenes.rigid_joint_motor_limits import build

    setup = build()
    controller = setup.info["rigid_joint_motor_limit_controller"]
    for _ in range(180):
        assert setup.pre_step is not None
        setup.pre_step()

    metrics = controller._last_metrics
    assert controller.motor_joint.actuator_type == sx.ActuatorType.VELOCITY
    assert float(metrics["motor_speed"]) == pytest.approx(
        float(metrics["motor_expected_speed"]), abs=1.0e-9
    )
    assert float(metrics["motor_speed_error"]) < 1.0e-9
    assert float(metrics["position_limit_angle"]) == pytest.approx(
        float(metrics["position_limit_upper"]), abs=1.0e-9
    )
    assert float(metrics["position_limit_error"]) < 1.0e-9
    assert float(metrics["position_limit_speed"]) == pytest.approx(0.0, abs=1.0e-9)

    limited_accel = max(controller._limited_acceleration_history)
    open_accel = max(controller._open_acceleration_history)
    assert limited_accel == pytest.approx(
        controller.effort_limit / 2.0,
        abs=1.0e-9,
    )
    assert open_accel == pytest.approx(controller.force_command / 2.0, abs=1.0e-9)
    assert open_accel > 3.0 * limited_accel
    assert float(metrics["force_position_gap"]) > 0.20
    assert controller._motor_speed_history
    assert controller._limit_angle_history
    assert controller._force_position_gap_history
    assert np.isfinite(float(metrics["world_time"]))


def test_rigid_joint_passive_parameters_order_passive_response() -> None:
    import numpy as np

    _require_simulation_symbols("World", "JointSpec")

    from examples.demos.scenes.rigid_joint_passive_parameters import build

    setup = build()
    controller = setup.info["rigid_joint_passive_parameters_controller"]
    for _ in range(140):
        assert setup.pre_step is not None
        setup.pre_step()

    metrics = controller._last_metrics
    assert set(metrics) == {
        "spring_only",
        "spring_damper",
        "stiction",
        "slip",
        "armature_reference",
        "armature_heavy",
    }

    spring = metrics["spring_only"]
    damped = metrics["spring_damper"]
    stiction = metrics["stiction"]
    slip = metrics["slip"]
    armature_reference = metrics["armature_reference"]
    armature_heavy = metrics["armature_heavy"]

    assert float(damped["energy"]) < 0.55 * float(spring["energy"])
    assert max(controller._energy_history["spring_damper"]) < max(
        controller._energy_history["spring_only"]
    )

    assert abs(float(stiction["position"])) < 1.0e-12
    assert abs(float(stiction["velocity"])) < 1.0e-12
    assert float(stiction["acceleration"]) == pytest.approx(0.0, abs=1.0e-12)

    assert float(slip["position"]) > 0.20
    assert float(slip["velocity"]) > 0.70
    assert float(slip["acceleration"]) == pytest.approx(
        float(slip["expected_acceleration"]), abs=1.0e-9
    )

    assert float(armature_reference["acceleration"]) == pytest.approx(
        controller.armature_force / 2.0, abs=1.0e-9
    )
    assert float(armature_heavy["acceleration"]) == pytest.approx(
        controller.armature_force / (2.0 + controller.armature), abs=1.0e-9
    )
    assert float(armature_reference["acceleration"]) > 3.0 * float(
        armature_heavy["acceleration"]
    )
    assert float(armature_reference["position"]) > 3.0 * float(
        armature_heavy["position"]
    )

    for lane in controller.lanes:
        lane_metrics = metrics[lane.key]
        assert float(lane_metrics["acceleration"]) == pytest.approx(
            float(lane_metrics["expected_acceleration"]), abs=1.0e-8
        )
        assert controller._position_history[lane.key]
        assert controller._speed_history[lane.key]
        assert controller._accel_history[lane.key]
        assert controller._energy_history[lane.key]
    assert np.isfinite([float(value) for value in controller._step_ms_history]).all()


def test_rigid_screw_joint_pitch_couples_rotation_and_translation() -> None:
    import numpy as np

    _require_simulation_symbols("World", "JointSpec", "JointType")

    from examples.demos.scenes.rigid_screw_joint_pitch import build

    setup = build()
    controller = setup.info["rigid_screw_joint_pitch_controller"]
    for _ in range(120):
        assert setup.pre_step is not None
        setup.pre_step()

    metrics = controller._last_metrics
    assert set(metrics) == {
        "zero_pitch",
        "fine_pitch",
        "coarse_pitch",
        "reverse_pitch",
    }

    zero = metrics["zero_pitch"]
    fine = metrics["fine_pitch"]
    coarse = metrics["coarse_pitch"]
    reverse = metrics["reverse_pitch"]

    assert float(zero["pitch"]) == pytest.approx(0.0)
    assert abs(float(zero["angle"])) < 1.0e-12
    assert abs(float(zero["axial_travel"])) < 1.0e-12
    assert float(zero["acceleration"]) == pytest.approx(0.0, abs=1.0e-12)

    assert float(fine["pitch"]) == pytest.approx(controller.pitch_scale)
    assert float(coarse["pitch"]) == pytest.approx(2.0 * controller.pitch_scale)
    assert float(reverse["pitch"]) == pytest.approx(-controller.pitch_scale)
    for lane in controller.lanes:
        lane_metrics = metrics[lane.key]
        assert float(lane_metrics["travel_per_radian"]) == pytest.approx(
            float(lane_metrics["pitch"]), abs=1.0e-9
        )
        assert float(lane_metrics["acceleration"]) == pytest.approx(
            float(lane_metrics["expected_acceleration"]), abs=1.0e-9
        )
        assert float(lane_metrics["actual_axial_acceleration"]) == pytest.approx(
            float(lane_metrics["expected_axial_acceleration"]), abs=1.0e-9
        )
        assert float(lane_metrics["mass_matrix"]) == pytest.approx(
            float(lane_metrics["effective_mass"]), abs=1.0e-12
        )
        assert controller._angle_history[lane.key]
        assert controller._travel_history[lane.key]
        assert controller._accel_error_history[lane.key]

    assert float(fine["angle"]) < -0.5
    assert float(coarse["angle"]) < -0.5
    assert float(reverse["angle"]) > 0.5
    assert float(fine["axial_travel"]) < -0.15
    assert float(coarse["axial_travel"]) < float(fine["axial_travel"])
    assert float(reverse["axial_travel"]) < -0.15
    assert np.isfinite([float(value) for value in controller._step_ms_history]).all()


def test_rigid_multibody_dynamics_terms_expose_generalized_terms() -> None:
    import numpy as np

    _require_simulation_symbols("World", "JointSpec", "JointType")

    from examples.demos.scenes.rigid_multibody_dynamics_terms import build

    setup = build()
    controller = setup.info["rigid_multibody_dynamics_terms_controller"]
    for _ in range(80):
        assert setup.pre_step is not None
        setup.pre_step()

    metrics = controller._last_metrics
    assert set(metrics) == {
        "single_hinge",
        "coupled_two_link",
        "heavy_distal",
    }

    single = metrics["single_hinge"]
    coupled = metrics["coupled_two_link"]
    heavy = metrics["heavy_distal"]

    assert float(single["dofs"]) == pytest.approx(1.0)
    assert float(coupled["dofs"]) == pytest.approx(2.0)
    assert float(heavy["dofs"]) == pytest.approx(2.0)
    assert float(single["condition"]) == pytest.approx(1.0)
    assert abs(float(coupled["coupling"])) > 1.0e-2
    assert abs(float(heavy["coupling"])) > abs(float(coupled["coupling"]))

    for lane_metrics in metrics.values():
        assert float(lane_metrics["identity_error"]) < 1.0e-10
        assert float(lane_metrics["inverse_dynamics_residual"]) < 1.0e-10
        assert float(lane_metrics["impulse_residual"]) < 1.0e-10
        assert float(lane_metrics["dynamics_residual"]) < 1.0e-9
        assert float(lane_metrics["acceleration_error"]) < 1.0e-9
        assert float(lane_metrics["tau_norm"]) > 0.0
        assert float(lane_metrics["response_norm"]) > 0.0
        assert np.isfinite(
            [
                float(lane_metrics["mass_diag0"]),
                float(lane_metrics["inverse_diag0"]),
                float(lane_metrics["gravity_norm"]),
                float(lane_metrics["condition"]),
            ]
        ).all()

    assert float(heavy["tau_norm"]) > float(coupled["tau_norm"])
    assert float(heavy["response_norm"]) < float(coupled["response_norm"])
    assert abs(float(coupled["response1"])) > 1.0
    assert abs(float(heavy["response1"])) > 1.0
    assert controller._residual_history["single_hinge"]
    assert controller._response_history["heavy_distal"]
    assert controller._tau_history["coupled_two_link"]
    assert controller._coupling_history
    assert np.isfinite([float(value) for value in controller._step_ms_history]).all()


def test_rigid_link_center_of_mass_offsets_gravity_torque() -> None:
    import numpy as np

    _require_simulation_symbols("World", "JointSpec", "JointType")

    from examples.demos.scenes.rigid_link_center_of_mass import build

    setup = build()
    controller = setup.info["rigid_link_center_of_mass_controller"]
    initial_metrics = controller._last_metrics
    assert set(initial_metrics) == {
        "centered",
        "positive",
        "negative",
        "high_inertia",
    }

    centered_initial = initial_metrics["centered"]
    positive_initial = initial_metrics["positive"]
    negative_initial = initial_metrics["negative"]
    high_initial = initial_metrics["high_inertia"]

    assert float(centered_initial["offset"]) == pytest.approx(0.0)
    assert float(centered_initial["gravity_torque"]) == pytest.approx(
        0.0, abs=1.0e-12
    )
    assert float(centered_initial["expected_acceleration"]) == pytest.approx(
        0.0, abs=1.0e-12
    )
    assert float(positive_initial["offset"]) == pytest.approx(controller.com_offset)
    assert float(negative_initial["offset"]) == pytest.approx(-controller.com_offset)
    assert float(positive_initial["gravity_force"]) == pytest.approx(
        -controller.link_mass * 9.81 * controller.gravity_scale * controller.com_offset
    )
    assert float(negative_initial["gravity_force"]) == pytest.approx(
        -float(positive_initial["gravity_force"])
    )
    assert float(positive_initial["mass_matrix"]) == pytest.approx(
        float(positive_initial["expected_mass_matrix"])
    )
    assert float(high_initial["mass_matrix"]) == pytest.approx(
        float(high_initial["expected_mass_matrix"])
    )
    assert float(high_initial["mass_matrix"]) > float(positive_initial["mass_matrix"])
    assert float(high_initial["expected_acceleration"]) < float(
        positive_initial["expected_acceleration"]
    )

    for _ in range(32):
        assert setup.pre_step is not None
        setup.pre_step()

    metrics = controller._last_metrics
    centered = metrics["centered"]
    positive = metrics["positive"]
    negative = metrics["negative"]
    high = metrics["high_inertia"]

    assert float(centered["angle"]) == pytest.approx(0.0, abs=1.0e-12)
    assert float(positive["angle"]) > 0.08
    assert float(negative["angle"]) < -0.08
    assert float(negative["angle"]) == pytest.approx(-float(positive["angle"]))
    assert float(positive["acceleration"]) > 0.0
    assert float(negative["acceleration"]) < 0.0
    assert float(negative["acceleration"]) == pytest.approx(
        -float(positive["acceleration"]), rel=1.0e-6
    )
    assert 0.0 < float(high["acceleration"]) < float(positive["acceleration"])
    for lane in controller.lanes:
        lane_metrics = metrics[lane.key]
        assert float(lane_metrics["mass_matrix"]) == pytest.approx(
            float(lane_metrics["expected_mass_matrix"])
        )
        assert float(lane_metrics["acceleration"]) == pytest.approx(
            float(lane_metrics["expected_acceleration"]), abs=1.0e-2
        )
        assert np.isfinite(
            [
                float(lane_metrics["gravity_torque"]),
                float(lane_metrics["com_world_x"]),
                float(lane_metrics["com_world_z"]),
                float(lane_metrics["energy"]),
            ]
        ).all()
        assert controller._angle_history[lane.key]
        assert controller._accel_history[lane.key]
        assert controller._torque_history[lane.key]
        assert controller._energy_history[lane.key]
    positive_lane = next(lane for lane in controller.lanes if lane.key == "positive")
    assert positive_lane.link.center_of_mass.tolist() == pytest.approx(
        [controller.com_offset, 0.0, 0.0]
    )
    assert np.isfinite([float(value) for value in controller._step_ms_history]).all()


def test_rigid_link_jacobian_maps_link_origin_twist_and_wrench() -> None:
    import numpy as np

    _require_simulation_symbols("World", "JointSpec", "JointType")

    from examples.demos.scenes.rigid_link_jacobian import build

    setup = build()
    controller = setup.info["rigid_link_jacobian_controller"]
    controller.motion_speed = 1.10
    controller.elbow_phase = -0.35
    controller.wrench_force = 1.90
    controller.wrench_angle_deg = 34.0
    controller.wrench_moment = 0.22
    controller.reset(clear_replay=True)
    for _ in range(64):
        assert setup.pre_step is not None
        setup.pre_step()

    metrics = controller._last_metrics
    assert float(metrics["rows"]) == pytest.approx(6.0)
    assert float(metrics["dofs"]) == pytest.approx(2.0)
    assert float(metrics["linear_speed"]) > 0.05
    assert float(metrics["angular_speed"]) > 0.05
    assert float(metrics["finite_difference_error"]) < 1.0e-5
    assert float(metrics["power_error"]) < 1.0e-10
    assert float(metrics["joint_power"]) == pytest.approx(
        float(metrics["wrench_power"]), abs=1.0e-10
    )
    assert abs(float(metrics["tau0"])) > 1.0e-3
    assert abs(float(metrics["tau1"])) > 1.0e-3
    assert float(metrics["world_body_gap"]) > 1.0e-3
    assert float(metrics["force_norm"]) == pytest.approx(1.90)
    assert float(metrics["moment_y"]) == pytest.approx(0.22)
    assert controller._speed_history
    assert controller._fd_error_history
    assert controller._power_error_history
    assert controller._tau0_history
    assert controller._tau1_history
    assert controller._world_body_gap_history
    assert np.isfinite([float(value) for value in controller._speed_history]).all()


def test_rigid_link_point_loads_show_lever_arm_and_frame_semantics() -> None:
    import numpy as np

    _require_simulation_symbols("World", "JointSpec")

    from examples.demos.scenes.rigid_link_point_loads import build

    setup = build()
    controller = setup.info["rigid_link_point_loads_controller"]
    assert setup.pre_step is not None
    setup.pre_step()

    metrics = controller._last_metrics
    assert set(metrics) == {
        "center",
        "offcenter",
        "pulse",
        "double",
        "world_frame",
        "local_frame",
    }

    center = metrics["center"]
    offcenter = metrics["offcenter"]
    pulse = metrics["pulse"]
    double = metrics["double"]
    world_frame = metrics["world_frame"]
    local_frame = metrics["local_frame"]

    assert float(center["world_accel_x"]) == pytest.approx(
        float(center["expected_world_accel_x"]), abs=1.0e-9
    )
    assert float(center["world_accel_y"]) == pytest.approx(0.0, abs=1.0e-12)
    assert float(center["yaw_accel"]) == pytest.approx(0.0, abs=1.0e-12)

    assert float(offcenter["world_accel_x"]) == pytest.approx(
        float(center["world_accel_x"]), abs=1.0e-9
    )
    assert float(offcenter["yaw_accel"]) == pytest.approx(
        float(offcenter["expected_yaw_accel"]), abs=1.0e-9
    )
    assert abs(float(offcenter["yaw_accel"])) > 1.0

    assert float(double["world_accel_x"]) == pytest.approx(
        2.0 * float(center["world_accel_x"]), abs=1.0e-9
    )
    assert float(double["applied_count"]) == pytest.approx(2.0)

    assert float(world_frame["world_accel_x"]) == pytest.approx(
        float(center["world_accel_x"]), abs=1.0e-9
    )
    assert float(world_frame["world_accel_y"]) == pytest.approx(0.0, abs=1.0e-9)
    assert abs(float(local_frame["world_accel_x"])) < 1.0e-8
    assert float(local_frame["world_accel_y"]) == pytest.approx(
        float(center["world_accel_x"]), abs=1.0e-9
    )

    assert float(pulse["world_accel_x"]) == pytest.approx(
        float(center["world_accel_x"]), abs=1.0e-9
    )
    setup.pre_step()
    pulse_after_clear = controller._last_metrics["pulse"]
    assert float(pulse_after_clear["applied_count"]) == pytest.approx(0.0)
    assert float(pulse_after_clear["expected_world_accel_x"]) == pytest.approx(0.0)
    assert float(pulse_after_clear["world_accel_x"]) == pytest.approx(0.0, abs=1.0e-6)

    assert controller._speed_history["center"]
    assert controller._yaw_rate_history["offcenter"]
    assert np.isfinite(float(local_frame["displacement_y"]))


def test_rigid_loop_closure_compares_closure_families() -> None:
    import numpy as np

    sx = _require_simulation_symbols(
        "World",
        "LoopClosureSpec",
        "LoopClosureFamily",
        "ClosureDynamicsPolicy",
        "MultibodyOptions",
    )

    from examples.demos.scenes.rigid_loop_closure import build

    setup = build()
    controller = setup.info["rigid_loop_closure_controller"]
    for _ in range(120):
        assert setup.pre_step is not None
        setup.pre_step()

    metrics = controller._last_metrics
    assert {case.family_label for case in controller.cases} == {
        "POINT",
        "DISTANCE",
        "RIGID",
    }
    assert {case.policy_label for case in controller.cases} == {
        "residual",
        "solved",
    }
    assert len(controller.cases) == 6
    for case in controller.cases:
        expected = (
            sx.ClosureDynamicsPolicy.SOLVE
            if case.policy_label == "solved"
            else sx.ClosureDynamicsPolicy.RESIDUAL_ONLY
        )
        assert case.closure.dynamics == expected

    expected_coordinates = {
        "POINT": 3.0,
        "DISTANCE": 1.0,
        "RIGID": 6.0,
    }
    for family_label, coordinate_count in expected_coordinates.items():
        residual_only = metrics[f"{family_label} residual"]
        solved = metrics[f"{family_label} solved"]
        assert float(residual_only["coordinates"]) == pytest.approx(coordinate_count)
        assert float(solved["coordinates"]) == pytest.approx(coordinate_count)
        assert float(residual_only["residual"]) > 0.75
        assert float(solved["residual"]) < 1.0e-8
        assert controller._residual_ratio_history[family_label][-1] > 1.0e8
        assert bool(solved["active"]) is True
        assert bool(solved["enabled"]) is True
        assert controller._residual_history[f"{family_label} residual"]
        assert controller._residual_history[f"{family_label} solved"]
        assert np.isfinite(float(solved["step_ms"]))

    point_solved = metrics["POINT solved"]
    distance_solved = metrics["DISTANCE solved"]
    rigid_solved = metrics["RIGID solved"]
    assert float(point_solved["tip_error"]) == pytest.approx(
        float(point_solved["residual"]), abs=1.0e-9
    )
    assert float(distance_solved["distance_error"]) == pytest.approx(
        float(distance_solved["residual"]), abs=1.0e-9
    )
    assert float(distance_solved["tip_error"]) > 0.1
    assert float(rigid_solved["tip_error"]) == pytest.approx(
        float(rigid_solved["residual"]), abs=1.0e-9
    )
    assert float(metrics["RIGID residual"]["orientation_error"]) > 0.1
    assert float(rigid_solved["orientation_error"]) < 1.0e-8


def test_rigid_multibody_solver_family_routes_solved_closures() -> None:
    import numpy as np

    sx = _require_simulation_symbols(
        "World",
        "LoopClosureSpec",
        "LoopClosureFamily",
        "ClosureDynamicsPolicy",
        "MultibodyOptions",
    )

    from examples.demos.scenes.rigid_multibody_solver_family import build

    setup = build()
    controller = setup.info["rigid_multibody_solver_family_controller"]
    for _ in range(120):
        assert setup.pre_step is not None
        setup.pre_step()

    metrics = controller._last_metrics
    assert set(metrics) == {
        "semi_residual",
        "variational_residual",
        "variational_solved",
    }
    assert {case.integration_family for case in controller.cases} == {
        "semi-implicit",
        "variational integrator",
    }

    semi = metrics["semi_residual"]
    variational_residual = metrics["variational_residual"]
    variational_solved = metrics["variational_solved"]

    assert controller.cases[0].closure.dynamics == sx.ClosureDynamicsPolicy.RESIDUAL_ONLY
    assert controller.cases[1].closure.dynamics == sx.ClosureDynamicsPolicy.RESIDUAL_ONLY
    assert controller.cases[2].closure.dynamics == sx.ClosureDynamicsPolicy.SOLVE
    assert float(semi["coordinates"]) == pytest.approx(3.0)
    assert float(variational_residual["coordinates"]) == pytest.approx(3.0)
    assert float(variational_solved["coordinates"]) == pytest.approx(3.0)
    assert float(semi["residual"]) > 1.0
    assert float(variational_residual["residual"]) > 1.0
    assert float(variational_solved["residual"]) < 1.0e-8
    assert float(semi["tip_error"]) == pytest.approx(
        float(semi["residual"]), abs=1.0e-9
    )
    assert float(variational_solved["tip_error"]) == pytest.approx(
        float(variational_solved["residual"]), abs=1.0e-9
    )
    assert controller._solve_ratio_history[-1] > 1.0e8
    assert bool(variational_solved["dynamic_solve"]) is True
    assert bool(semi["dynamic_solve"]) is False
    assert np.isfinite([float(value) for value in controller._step_ms_history["semi_residual"]]).all()


def test_avbd_fixed_joint_contact_demo_exercises_contact_path() -> None:
    import numpy as np

    _require_simulation_symbols("World")

    from examples.demos.scenes.avbd_rigid_fixed_joint_contact import build

    setup = build()
    sx_world = setup.info["sx_world"]
    base = setup.info["base"]
    payload = setup.info["payload"]
    connector = setup.info["connector"]

    assert sx_world.num_rigid_body_fixed_joints == 1
    assert len(sx_world.collide()) > 0
    assert not base.is_static

    initial_base = np.asarray(base.translation, dtype=float).reshape(3)
    initial_payload = np.asarray(payload.translation, dtype=float).reshape(3)

    for _ in range(20):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    assert len(sx_world.collide()) > 0
    base_translation = np.asarray(base.translation, dtype=float).reshape(3)
    payload_translation = np.asarray(payload.translation, dtype=float).reshape(3)
    offset = payload_translation - base_translation
    expected_offset = np.array([0.72, 0.0, -0.34])
    assert np.linalg.norm(offset - expected_offset) < 2.0e-2
    connector_transform = np.asarray(connector.get_transform().matrix())
    assert connector_transform[:3, 0] == pytest.approx(
        offset / np.linalg.norm(offset), abs=1.0e-6
    )
    assert np.linalg.norm(base_translation - initial_base) > 1.0e-3
    assert np.linalg.norm(payload_translation - initial_payload) > 1.0e-3


def test_avbd_revolute_motor_demo_drives_hinge() -> None:
    import numpy as np

    sx = _require_simulation_symbols("World", "ActuatorType")

    from examples.demos.scenes.avbd_rigid_revolute_motor import build

    setup = build()
    sx_world = setup.info["sx_world"]
    rotor = setup.info["rotor"]
    joint = setup.info["joint"]
    target_speed = float(setup.info["target_speed"])
    max_torque = float(setup.info["max_torque"])

    assert sx_world.num_rigid_body_joints == 1
    assert joint.actuator_type == sx.ActuatorType.VELOCITY
    assert np.asarray(joint.command_velocity, dtype=float).reshape(1)[
        0
    ] == pytest.approx(target_speed)
    assert np.asarray(joint.effort_lower_limits, dtype=float).reshape(1)[
        0
    ] == pytest.approx(-max_torque)
    assert np.asarray(joint.effort_upper_limits, dtype=float).reshape(1)[
        0
    ] == pytest.approx(max_torque)

    initial_speed = float(np.asarray(rotor.angular_velocity, dtype=float).reshape(3)[2])
    for _ in range(40):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    measured_speed = float(
        np.asarray(rotor.angular_velocity, dtype=float).reshape(3)[2]
    )
    assert measured_speed > initial_speed + 0.05
    assert measured_speed == pytest.approx(target_speed, abs=0.7)


def test_avbd_breakable_joint_demo_marks_joint_broken() -> None:
    import numpy as np

    _require_simulation_symbols("World")

    from examples.demos.scenes.avbd_rigid_breakable_joint import build

    setup = build()
    sx_world = setup.info["sx_world"]
    joint = setup.info["joint"]
    base = setup.info["base"]
    payload = setup.info["payload"]
    break_force = float(setup.info["break_force"])

    assert sx_world.num_rigid_body_fixed_joints == 1
    assert joint.break_force == pytest.approx(break_force)
    assert not joint.is_broken

    initial_base = np.asarray(base.translation, dtype=float).reshape(3)
    initial_payload = np.asarray(payload.translation, dtype=float).reshape(3)
    for _ in range(45):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    assert joint.is_broken
    base_translation = np.asarray(base.translation, dtype=float).reshape(3)
    payload_translation = np.asarray(payload.translation, dtype=float).reshape(3)
    assert np.linalg.norm(base_translation - initial_base) < 1.0e-9
    assert np.linalg.norm(payload_translation - initial_payload) > 1.0e-2


def test_runner_screenshot_writes_ppm(tmp_path: pathlib.Path) -> None:
    """`--screenshot` writes a real PPM via the dartpy.gui Filament viewer
    (PLAN-103 Phase 2 replacement for the old JSON state stub)."""

    if not _gui_run_demos_available():
        pytest.skip("dartpy.gui.run_demos unavailable (GUI not built)")

    scenes = make_demo_scenes()
    target = scenes[0]
    out = tmp_path / "snap.ppm"
    rc = run(
        [
            "--scene",
            target.id,
            "--frames",
            "1",
            "--headless",
            "--width",
            "160",
            "--height",
            "120",
            "--screenshot",
            str(out),
        ],
        scenes,
    )
    # Soft-fail (rc != 0) is acceptable when the scene's assets can't load
    # in the test environment; what matters is the runner didn't crash.
    assert rc in (0, 1)
    if rc == 0:
        data = out.read_bytes()
        # PPM "P6" header + non-empty pixel payload (>1KB) for any valid frame.
        assert data.startswith(b"P6"), f"not a PPM: {data[:8]!r}"
        assert len(data) > 1024, f"PPM too small: {len(data)} bytes"


def test_show_ui_uses_docked_workspace_regions(
    tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    if not _gui_run_demos_available():
        pytest.skip("dartpy.gui.run_demos unavailable (GUI not built)")

    import dartpy as dart
    from examples.demos.runner import PythonDemoScene, ScenePanel, SceneSetup

    if not getattr(dart.gui, "is_docking_available", lambda: False)():
        pytest.skip("GUI build does not include ImGui docking support")

    monkeypatch.setenv("LIBGL_ALWAYS_SOFTWARE", "1")
    monkeypatch.setenv("MESA_LOADER_DRIVER_OVERRIDE", "llvmpipe")

    def build_scene() -> SceneSetup:
        panel = ScenePanel(
            "Controls",
            lambda builder, _context: builder.text("dock smoke"),
        )
        setup = _make_box_scene_setup(
            dart, SceneSetup, name="docked_smoke", frame_name="box"
        )
        setup.panels.append(panel)
        return setup

    out = tmp_path / "docked.ppm"
    rc = run(
        [
            "--scene",
            "docked_smoke",
            "--frames",
            "4",
            "--headless",
            "--show-ui",
            "--width",
            "640",
            "--height",
            "360",
            "--screenshot",
            str(out),
        ],
        [
            PythonDemoScene(
                id="docked_smoke",
                title="Docked Smoke",
                category="Test",
                summary="Exercises the docked workspace.",
                build=build_scene,
            )
        ],
    )

    assert rc == 0
    width, height, pixels = _read_ppm(out)
    assert (width, height) == (640, 360)

    top = _mean_luminance(pixels, width, 0, width, 0, 70)
    left = _mean_luminance(pixels, width, 0, 155, 80, 320)
    right = _mean_luminance(pixels, width, 470, width, 80, 320)
    bottom = _mean_luminance(pixels, width, 0, width, 320, height)
    center = _mean_luminance(pixels, width, 180, 460, 100, 300)

    assert top < 70.0
    assert left < 80.0
    assert right < 80.0
    assert bottom < 70.0
    assert center > 90.0
    assert center > left + 35.0
    assert center > right + 35.0


def test_py_demo_capture_records_ui_force_drag_artifacts(
    tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    if not _gui_run_demos_available():
        pytest.skip("dartpy.gui.run_demos unavailable (GUI not built)")

    import dartpy as dart

    if not getattr(dart.gui, "is_docking_available", lambda: False)():
        pytest.skip("GUI build does not include ImGui docking support")
    _require_simulation_symbols("World", "RigidBodySolver", "CollisionShape")

    monkeypatch.setenv("LIBGL_ALWAYS_SOFTWARE", "1")
    monkeypatch.setenv("MESA_LOADER_DRIVER_OVERRIDE", "llvmpipe")
    capture_py_demo = _capture_py_demo_module()
    output = tmp_path / "capture"

    rc = capture_py_demo.main(
        [
            "--scene",
            "rigid_ipc_slide",
            "--force-drag-target",
            "ipc_slide_box_visual",
            "--force-drag-frame",
            "2",
            "--force-drag-frames",
            "5",
            "--show-ui",
            "--frames",
            "10",
            "--width",
            "640",
            "--height",
            "360",
            "--output-dir",
            str(output),
        ]
    )

    assert rc == 0
    manifest = json.loads((output / "manifest.json").read_text())
    assert manifest["ui_ready"]["required"] is True
    assert manifest["ui_ready"]["dropped_warmup_frames"] >= 0
    assert pathlib.Path(manifest["artifacts"]["screenshot"]).is_file()
    capture = manifest["capture"]
    assert capture["width"] == 640
    assert capture["height"] == 360
    assert capture["requested_frames"] == 10
    png_frames = sorted((output / "png_frames").glob("frame_*.png"))
    assert png_frames
    assert capture["converted_frames"] == len(png_frames)
    ppm_frames = sorted((output / "frames").glob("frame_*.ppm"))
    assert [frame.name for frame in ppm_frames] == [
        f"frame_{index:06d}.ppm" for index in range(1, len(ppm_frames) + 1)
    ]
    assert capture_py_demo.ppm_has_docked_workspace_regions(ppm_frames[0])
    evidence = manifest["visual_evidence"]
    screenshot_evidence = evidence["screenshot"]
    assert screenshot_evidence["width"] == 640
    assert screenshot_evidence["height"] == 360
    assert screenshot_evidence["nonzero_pixels"] > 0
    assert screenshot_evidence["unique_rgb_count"] > 1
    assert screenshot_evidence["rgb_channel_variance"] > 0.0
    assert screenshot_evidence["docked_workspace"] is True
    first_frame_evidence = evidence["first_frame"]
    assert first_frame_evidence["width"] == 640
    assert first_frame_evidence["height"] == 360
    assert first_frame_evidence["nonzero_pixels"] > 0
    assert first_frame_evidence["unique_rgb_count"] > 1
    assert first_frame_evidence["rgb_channel_variance"] > 0.0
    assert first_frame_evidence["docked_workspace"] is True

    events = [
        json.loads(line)
        for line in (output / "events.jsonl").read_text().splitlines()
        if line.strip()
    ]
    event_names = [event["event"] for event in events]
    assert "force_drag_started" in event_names
    assert "force_drag_updated" in event_names
    assert "force_drag_released" in event_names
    assert event_names[-1] == "artifacts_written"


def test_scripted_demo_switch_restores_previous_scene_on_factory_error(
    tmp_path: pathlib.Path,
) -> None:
    if not _gui_run_demos_available():
        pytest.skip("dartpy.gui.run_demos unavailable (GUI not built)")

    import dartpy as dart
    from examples.demos.runner import PythonDemoScene, SceneSetup

    def build_good() -> SceneSetup:
        return _make_box_scene_setup(
            dart, SceneSetup, name="good", frame_name="good_box"
        )

    def build_broken() -> SceneSetup:
        raise RuntimeError("intentional scripted switch failure")

    events = tmp_path / "events.jsonl"
    screenshot = tmp_path / "snap.ppm"
    rc = run(
        [
            "--scene",
            "good",
            "--headless",
            "--frames",
            "5",
            "--width",
            "160",
            "--height",
            "120",
            "--screenshot",
            str(screenshot),
            "--scripted-demo-switch",
            "2:broken",
            "--scripted-demo-event-log",
            str(events),
        ],
        [
            PythonDemoScene(
                id="good",
                title="Good",
                category="Test",
                summary="Builds successfully.",
                build=build_good,
            ),
            PythonDemoScene(
                id="broken",
                title="Broken",
                category="Test",
                summary="Throws during factory startup.",
                build=build_broken,
            ),
        ],
    )

    assert rc == 0
    payloads = [json.loads(line) for line in events.read_text().splitlines()]
    events_by_name = {payload["event"]: payload for payload in payloads}
    assert events_by_name["requested_demo_switch"]["active_scene"] == "good"
    assert events_by_name["requested_demo_switch"]["target_scene"] == "broken"
    assert events_by_name["restored_previous_demo"]["active_scene"] == "good"
    assert "factory threw" in events_by_name["restored_previous_demo"]["status"]
    assert events_by_name["script_finished_without_target"]["active_scene"] == "good"


def test_scripted_demo_switch_restores_previous_scene_on_startup_timeout(
    tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    if not _gui_run_demos_available():
        pytest.skip("dartpy.gui.run_demos unavailable (GUI not built)")

    import dartpy as dart
    from examples.demos.runner import PythonDemoScene, SceneSetup

    good_setup: SceneSetup | None = None

    def build_good() -> SceneSetup:
        nonlocal good_setup
        if good_setup is not None:
            return good_setup
        good_setup = _make_box_scene_setup(
            dart, SceneSetup, name="good", frame_name="good_box"
        )
        return good_setup

    def build_slow() -> SceneSetup:
        time.sleep(0.25)
        return _make_box_scene_setup(
            dart, SceneSetup, name="slow", frame_name="slow_box", visible=False
        )

    monkeypatch.setenv("DART_PY_DEMO_SCENE_BUILD_TIMEOUT_MS", "10")
    monkeypatch.setenv("DART_DEMO_SCENE_STARTUP_TIMEOUT_MS", "100")
    events = tmp_path / "events.jsonl"
    screenshot = tmp_path / "snap.ppm"
    rc = run(
        [
            "--scene",
            "good",
            "--headless",
            "--frames",
            "5",
            "--width",
            "160",
            "--height",
            "120",
            "--screenshot",
            str(screenshot),
            "--scripted-demo-switch",
            "2:slow",
            "--scripted-demo-event-log",
            str(events),
        ],
        [
            PythonDemoScene(
                id="good",
                title="Good",
                category="Test",
                summary="Builds successfully.",
                build=build_good,
            ),
            PythonDemoScene(
                id="slow",
                title="Slow",
                category="Test",
                summary="Returns after the startup budget.",
                build=build_slow,
            ),
        ],
    )

    assert rc == 0
    payloads = [json.loads(line) for line in events.read_text().splitlines()]
    events_by_name = {payload["event"]: payload for payload in payloads}
    assert events_by_name["requested_demo_switch"]["active_scene"] == "good"
    assert events_by_name["requested_demo_switch"]["target_scene"] == "slow"
    assert events_by_name["restored_previous_demo"]["active_scene"] == "good"
    restored_status = events_by_name["restored_previous_demo"]["status"]
    assert any(
        expected in restored_status
        for expected in (
            "Python demo scene 'slow' build exceeded",
            "factory startup exceeded budget",
        )
    )
    assert events_by_name["script_finished_without_target"]["active_scene"] == "good"


def test_scripted_demo_switch_restores_previous_scene_on_render_state_failure(
    tmp_path: pathlib.Path,
) -> None:
    if not _gui_run_demos_available():
        pytest.skip("dartpy.gui.run_demos unavailable (GUI not built)")

    import dartpy as dart
    from examples.demos.runner import PythonDemoScene, SceneSetup

    def build_good() -> SceneSetup:
        return _make_box_scene_setup(
            dart, SceneSetup, name="good", frame_name="good_box"
        )

    def build_empty() -> SceneSetup:
        return _make_box_scene_setup(
            dart, SceneSetup, name="empty", frame_name="empty_box", visible=False
        )

    events = tmp_path / "events.jsonl"
    screenshot = tmp_path / "snap.ppm"
    rc = run(
        [
            "--scene",
            "good",
            "--headless",
            "--frames",
            "5",
            "--width",
            "160",
            "--height",
            "120",
            "--screenshot",
            str(screenshot),
            "--scripted-demo-switch",
            "2:empty",
            "--scripted-demo-event-log",
            str(events),
        ],
        [
            PythonDemoScene(
                id="good",
                title="Good",
                category="Test",
                summary="Builds successfully.",
                build=build_good,
            ),
            PythonDemoScene(
                id="empty",
                title="Empty",
                category="Test",
                summary="Builds but has no visible render state.",
                build=build_empty,
            ),
        ],
    )

    assert rc == 0
    payloads = [json.loads(line) for line in events.read_text().splitlines()]
    events_by_name = {payload["event"]: payload for payload in payloads}
    assert events_by_name["requested_demo_switch"]["active_scene"] == "good"
    assert events_by_name["requested_demo_switch"]["target_scene"] == "empty"
    assert events_by_name["restored_previous_demo"]["active_scene"] == "good"
    assert (
        "render state creation failed"
        in events_by_name["restored_previous_demo"]["status"]
    )
    assert events_by_name["script_finished_without_target"]["active_scene"] == "good"


def test_scripted_demo_switch_restores_previous_scene_when_python_factory_stalls(
    tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    if (
        not hasattr(signal, "SIGALRM")
        or not hasattr(signal, "ITIMER_REAL")
        or not hasattr(signal, "getitimer")
        or not hasattr(signal, "setitimer")
    ):
        pytest.skip("SIGALRM scene build watchdog unavailable")
    if not _gui_run_demos_available():
        pytest.skip("dartpy.gui.run_demos unavailable (GUI not built)")

    import dartpy as dart
    from examples.demos.runner import PythonDemoScene, SceneSetup

    def build_good() -> SceneSetup:
        return _make_box_scene_setup(
            dart, SceneSetup, name="good", frame_name="good_box"
        )

    def build_stalled() -> SceneSetup:
        time.sleep(60.0)
        return _make_box_scene_setup(
            dart, SceneSetup, name="stalled", frame_name="stalled_box", visible=False
        )

    monkeypatch.delenv("DART_PY_DEMO_SCENE_BUILD_TIMEOUT_MS", raising=False)
    monkeypatch.setenv("DART_DEMO_SCENE_STARTUP_TIMEOUT_MS", "10")
    events = tmp_path / "events.jsonl"
    screenshot = tmp_path / "snap.ppm"
    started = time.monotonic()
    rc = run(
        [
            "--scene",
            "good",
            "--headless",
            "--frames",
            "5",
            "--width",
            "160",
            "--height",
            "120",
            "--screenshot",
            str(screenshot),
            "--scripted-demo-switch",
            "2:stalled",
            "--scripted-demo-event-log",
            str(events),
        ],
        [
            PythonDemoScene(
                id="good",
                title="Good",
                category="Test",
                summary="Builds successfully.",
                build=build_good,
            ),
            PythonDemoScene(
                id="stalled",
                title="Stalled",
                category="Test",
                summary="Does not return without a watchdog.",
                build=build_stalled,
            ),
        ],
    )
    elapsed = time.monotonic() - started

    assert rc == 0
    assert elapsed < 5.0
    payloads = [json.loads(line) for line in events.read_text().splitlines()]
    events_by_name = {payload["event"]: payload for payload in payloads}
    assert events_by_name["requested_demo_switch"]["active_scene"] == "good"
    assert events_by_name["requested_demo_switch"]["target_scene"] == "stalled"
    assert events_by_name["restored_previous_demo"]["active_scene"] == "good"
    assert (
        "Python demo scene 'stalled' build exceeded"
        in events_by_name["restored_previous_demo"]["status"]
    )
    assert events_by_name["script_finished_without_target"]["active_scene"] == "good"


def test_scripted_demo_switch_restores_previous_scene_when_python_pre_step_stalls(
    tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    if (
        not hasattr(signal, "SIGALRM")
        or not hasattr(signal, "ITIMER_REAL")
        or not hasattr(signal, "getitimer")
        or not hasattr(signal, "setitimer")
    ):
        pytest.skip("SIGALRM scene callback watchdog unavailable")
    if not _gui_run_demos_available():
        pytest.skip("dartpy.gui.run_demos unavailable (GUI not built)")

    import dartpy as dart
    from examples.demos.runner import PythonDemoScene, SceneSetup

    def build_good() -> SceneSetup:
        return _make_box_scene_setup(
            dart, SceneSetup, name="good", frame_name="good_box"
        )

    def build_stalled_step() -> SceneSetup:
        def pre_step() -> None:
            time.sleep(60.0)

        return _make_box_scene_setup(
            dart,
            SceneSetup,
            name="stalled_step",
            frame_name="stalled_box",
            color=(0.8, 0.3, 0.2),
            pre_step=pre_step,
        )

    monkeypatch.delenv("DART_PY_DEMO_SCENE_BUILD_TIMEOUT_MS", raising=False)
    monkeypatch.setenv("DART_DEMO_SCENE_STARTUP_TIMEOUT_MS", "10")
    events = tmp_path / "events.jsonl"
    screenshot = tmp_path / "snap.ppm"
    started = time.monotonic()
    rc = run(
        [
            "--scene",
            "good",
            "--headless",
            "--frames",
            "5",
            "--width",
            "160",
            "--height",
            "120",
            "--screenshot",
            str(screenshot),
            "--scripted-demo-switch",
            "2:stalled_step",
            "--scripted-demo-event-log",
            str(events),
        ],
        [
            PythonDemoScene(
                id="good",
                title="Good",
                category="Test",
                summary="Builds successfully.",
                build=build_good,
            ),
            PythonDemoScene(
                id="stalled_step",
                title="Stalled Step",
                category="Test",
                summary="Does not pre-step without a watchdog.",
                build=build_stalled_step,
            ),
        ],
    )
    elapsed = time.monotonic() - started

    assert rc == 0
    assert elapsed < 5.0
    payloads = [json.loads(line) for line in events.read_text().splitlines()]
    events_by_name = {payload["event"]: payload for payload in payloads}
    assert events_by_name["requested_demo_switch"]["active_scene"] == "good"
    assert events_by_name["requested_demo_switch"]["target_scene"] == "stalled_step"
    assert events_by_name["restored_previous_demo"]["active_scene"] == "good"
    assert (
        "Python demo scene 'stalled_step' pre_step exceeded"
        in events_by_name["restored_previous_demo"]["status"]
    )
    assert events_by_name["script_finished_without_target"]["active_scene"] == "good"


def test_scripted_demo_switch_restores_previous_scene_on_slow_first_frame(
    tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    if not _gui_run_demos_available():
        pytest.skip("dartpy.gui.run_demos unavailable (GUI not built)")

    import dartpy as dart
    from examples.demos.runner import PythonDemoScene, SceneSetup

    def build_good() -> SceneSetup:
        return _make_box_scene_setup(
            dart, SceneSetup, name="good", frame_name="good_box"
        )

    def build_slow_step() -> SceneSetup:
        def pre_step() -> None:
            time.sleep(0.25)

        return _make_box_scene_setup(
            dart,
            SceneSetup,
            name="slow_step",
            frame_name="slow_box",
            color=(0.8, 0.3, 0.2),
            pre_step=pre_step,
        )

    monkeypatch.setenv("DART_PY_DEMO_SCENE_BUILD_TIMEOUT_MS", "0")
    monkeypatch.setenv("DART_DEMO_SCENE_STARTUP_TIMEOUT_MS", "100")
    events = tmp_path / "events.jsonl"
    screenshot = tmp_path / "snap.ppm"
    rc = run(
        [
            "--scene",
            "good",
            "--headless",
            "--frames",
            "5",
            "--width",
            "160",
            "--height",
            "120",
            "--screenshot",
            str(screenshot),
            "--scripted-demo-switch",
            "2:slow_step",
            "--scripted-demo-event-log",
            str(events),
        ],
        [
            PythonDemoScene(
                id="good",
                title="Good",
                category="Test",
                summary="Builds successfully.",
                build=build_good,
            ),
            PythonDemoScene(
                id="slow_step",
                title="Slow Step",
                category="Test",
                summary="Returns after the first-frame budget.",
                build=build_slow_step,
            ),
        ],
    )

    assert rc == 0
    payloads = [json.loads(line) for line in events.read_text().splitlines()]
    events_by_name = {payload["event"]: payload for payload in payloads}
    assert events_by_name["requested_demo_switch"]["active_scene"] == "good"
    assert events_by_name["requested_demo_switch"]["target_scene"] == "slow_step"
    assert events_by_name["restored_previous_demo"]["active_scene"] == "good"
    assert (
        "first frame exceeded startup budget"
        in events_by_name["restored_previous_demo"]["status"]
    )
    assert events_by_name["script_finished_without_target"]["active_scene"] == "good"
