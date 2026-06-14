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
    REPLAY_TIMELINE_INFO_KEY,
    RIGID_VISUAL_WORKFLOW_CAPTURE_SPECS,
    RIGID_VISUAL_WORKFLOW_GUIDES,
    RIGID_VISUAL_WORKFLOW_LABELS,
    PythonDemoScene,
    _RIGID_WORKFLOW_RELATED_EVIDENCE,
    _viewer_catalog_title,
)


@pytest.fixture(autouse=True)
def _force_headless_gui_software_renderer(monkeypatch: pytest.MonkeyPatch) -> None:
    """Keep headless Filament screenshot tests off host GPU/display state."""

    monkeypatch.setenv("LIBGL_ALWAYS_SOFTWARE", "1")
    monkeypatch.setenv("MESA_LOADER_DRIVER_OVERRIDE", "llvmpipe")


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


def _read_rigid_visual_workflow_capture_metric_docs() -> dict[str, str]:
    root = pathlib.Path(__file__).resolve().parents[3]
    sidecar = (
        root
        / "docs"
        / "plans"
        / "103-examples-strategy"
        / "rigid-body-visual-verification.md"
    )
    rows: dict[str, str] = {}
    for line in sidecar.read_text(encoding="utf-8").splitlines():
        if not line.startswith("| "):
            continue
        cells = [cell.strip() for cell in line.split("|")]
        if len(cells) < 9 or not cells[1].isdigit():
            continue
        scene_id = cells[2].strip("`")
        rows[scene_id] = " ".join((cells[5], cells[7]))
    return rows


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


def _read_rigid_visual_readme_workflow_rows() -> list[tuple[str, str]]:
    root = pathlib.Path(__file__).resolve().parents[3]
    readme = root / "python" / "examples" / "demos" / "README.md"
    rows: list[tuple[str, str]] = []
    in_workflow_section = False
    for line in readme.read_text(encoding="utf-8").splitlines():
        if line.startswith("## "):
            if in_workflow_section:
                break
            in_workflow_section = (
                line == "## Rigid body visual verification workflow"
            )
            continue
        if not in_workflow_section or not line.startswith("| "):
            continue
        cells = [cell.strip() for cell in line.split("|")]
        if len(cells) < 4 or not re.fullmatch(r"\d{2}/\d{2}", cells[1]):
            continue
        rows.append((cells[1], cells[2].removeprefix("`").removesuffix("`")))
    return rows


def _read_rigid_visual_readme_workflow_ids() -> list[str]:
    return [
        scene_id
        for _order_label, scene_id in _read_rigid_visual_readme_workflow_rows()
    ]


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


def _require_simulation_experimental_symbols(*names: str):
    return _require_simulation_symbols(*names)


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
    rc = run(["--cycle-scenes", "--frames", "1", "--headless"], make_demo_scenes())
    assert rc == 0


def test_runner_cycle_frames_budget_applies_per_scene() -> None:
    if not _gui_run_demos_available():
        pytest.skip("dartpy.gui.run_demos unavailable (GUI not built)")
    if not _simulation_has("World"):
        pytest.skip("dartpy.World unavailable in this build")

    from examples.demos.scenes.rigid_body import SCENE as BASE_SCENE

    visited: list[str] = []

    def make_scene(scene_id: str) -> PythonDemoScene:
        def build():
            visited.append(scene_id)
            return BASE_SCENE.build()

        return PythonDemoScene(
            id=scene_id,
            title=f"Cycle {scene_id}",
            category="Cycle Test",
            summary="Small regression scene for the cycle harness.",
            build=build,
        )

    rc = run(
        ["--cycle-scenes", "--frames", "1", "--headless", "--hide-ui"],
        [make_scene("cycle_a"), make_scene("cycle_b")],
    )

    assert rc == 0
    assert visited == ["cycle_a", "cycle_b"]


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
            "rigid_distance_spring",
            "rigid_limited_joints",
            "rigid_joint_motor_limits",
            "rigid_joint_passive_parameters",
            "rigid_screw_joint_pitch",
            "rigid_multibody_dynamics_terms",
            "rigid_link_jacobian",
            "rigid_multibody_solver_family",
            "rigid_loop_closure",
            "rigid_kinematic_driver",
            "rigid_kinematic_normal_push",
            "rigid_link_center_of_mass",
        },
        "AVBD Rigid Constraints (sx)": {
            "avbd_empty_baseline",
            "avbd_demo2d_ground",
            "avbd_demo2d_motor",
            "avbd_demo2d_dynamic_friction",
            "avbd_demo2d_static_friction",
            "avbd_demo2d_pyramid",
            "avbd_demo2d_cards",
            "avbd_demo2d_stack",
            "avbd_demo2d_stack_ratio",
            "avbd_demo2d_rod",
            "avbd_demo2d_joint_grid",
            "avbd_demo2d_rope",
            "avbd_demo2d_heavy_rope",
            "avbd_demo2d_hanging_rope",
            "avbd_demo2d_spring",
            "avbd_demo2d_spring_ratio",
            "avbd_demo2d_net",
            "avbd_demo2d_fracture",
            "avbd_demo3d_ground",
            "avbd_demo3d_dynamic_friction",
            "avbd_demo3d_static_friction",
            "avbd_demo3d_pyramid",
            "avbd_demo3d_rope",
            "avbd_demo3d_heavy_rope",
            "avbd_demo3d_stack",
            "avbd_demo3d_stack_ratio",
            "avbd_demo3d_soft_body",
            "avbd_demo3d_bridge",
            "avbd_demo3d_breakable",
            "avbd_rigid_fixed_joint_contact",
            "avbd_rigid_revolute_motor",
            "avbd_articulated_revolute_motor",
            "avbd_articulated_prismatic_motor",
            "avbd_articulated_motor_breakable_joint",
            "avbd_articulated_prismatic_pair_motor_breakable_joint",
            "avbd_articulated_prismatic_motor_breakable_joint",
            "avbd_articulated_world_revolute_motor_breakable_joint",
            "avbd_articulated_high_ratio_chain",
            "avbd_rigid_breakable_joint",
            "avbd_rigid_spherical_breakable_joint",
            "avbd_articulated_breakable_joint",
            "avbd_articulated_fixed_pair_breakable_joint",
            "avbd_articulated_spherical_breakable_joint",
            "avbd_articulated_spherical_pair_breakable_joint",
        },
        "Planned World Ports": {
            "planned_inverse_kinematics",
            "planned_simbicon_walking",
            "planned_operational_space_control",
            "g1_puppet",
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
            "rigid_ipc_heavy_stack_packet",
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
        "rigid_kinematic_normal_push",
        "rigid_fixed_joint",
        "rigid_joint_breakage",
        "rigid_distance_spring",
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
    assert "rigid_ipc_edge_drop" not in workflow_ids
    assert "rigid_ipc_stack_packet" not in workflow_ids
    assert "rigid_ipc_heavy_stack_packet" not in workflow_ids
    assert by_id["rigid_ipc_tunnel"].category == "Rigid IPC"
    assert by_id["rigid_ipc_edge_drop"].category == "Rigid IPC"
    assert by_id["rigid_ipc_stack_packet"].category == "Rigid IPC"
    assert by_id["rigid_ipc_heavy_stack_packet"].category == "Rigid IPC"


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
    assert (
        _viewer_catalog_title(by_id["rigid_ipc_heavy_stack_packet"])
        == by_id["rigid_ipc_heavy_stack_packet"].title
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


def test_rigid_visual_replay_timeline_rows_publish_scene_metadata() -> None:
    _require_simulation_symbols("World")
    sidecar_docs = _read_rigid_visual_workflow_capture_metric_docs()
    documented_replay_rows = [
        scene_id
        for scene_id, evidence_text in sidecar_docs.items()
        if "Replay timeline coverage" in evidence_text
    ]
    scene_by_id = {scene.id: scene for scene in make_demo_scenes()}
    actual_replay_rows: list[str] = []

    for scene_id in sidecar_docs:
        setup = scene_by_id[scene_id].build()
        metadata = setup.info.get(REPLAY_TIMELINE_INFO_KEY)
        if not isinstance(metadata, dict):
            continue
        actual_replay_rows.append(scene_id)
        label = metadata.get("signal_label", metadata.get("label"))
        assert isinstance(label, str) and label.strip(), scene_id
        assert label != "Saved states", scene_id
        assert "signal" in metadata or "value" in metadata, scene_id

    assert documented_replay_rows
    assert documented_replay_rows == actual_replay_rows
    assert documented_replay_rows[-1] == "rigid_loop_closure"


def test_rigid_visual_workflow_related_evidence_routes_are_valid() -> None:
    scenes = make_demo_scenes()
    by_id = {scene.id: scene for scene in scenes}
    related_rows = _read_rigid_visual_related_evidence_rows()

    assert related_rows == [
        (
            "rigid_free_flight",
            "floating_base",
            "World Rigid Body",
            "Related shelf: World Rigid Body / floating_base - broader floating-joint row",
            (
                "Broader floating-joint SE(3) drift/spin example; use the "
                "numbered row for baseline rigid-body initial-state diagnostics."
            ),
        ),
        (
            "rigid_multibody_dynamics_terms",
            "articulated",
            "World Rigid Body",
            "Related shelf: World Rigid Body / articulated - broader two-link arm row",
            (
                "Broader two-link arm example; use the numbered row for mass, "
                "inverse-dynamics, and impulse-response diagnostics."
            ),
        ),
        (
            "rigid_solver_compare",
            "rigid_ipc_tunnel",
            "Rigid IPC",
            "Related shelf: Rigid IPC / rigid_ipc_tunnel - focused no-tunneling view",
            "Focused IPC capability scene; not a broad solver comparison or general proof.",
        ),
        (
            "rigid_solver_compare",
            "rigid_ipc_edge_drop",
            "Rigid IPC",
            (
                "Related shelf: Rigid IPC / rigid_ipc_edge_drop - "
                "degenerate edge-contact view"
            ),
            (
                "Focused IPC degenerate edge-contact capability scene; not a "
                "broad solver comparison or contact-manifold inspector."
            ),
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
        (
            "rigid_contact_solver_compare",
            "diff_pre_contact_surrogate",
            "Differentiable",
            (
                "Related shelf: Differentiable / diff_pre_contact_surrogate - "
                "pre-contact gradient route"
            ),
            (
                "Analytic vs pre-contact surrogate backward-only gradient for "
                "an approaching but not touching body; not a solver row."
            ),
        ),
        (
            "contact",
            "avbd_rigid_fixed_joint_contact",
            "AVBD Rigid Constraints (sx)",
            (
                "Related shelf: AVBD Rigid Constraints (sx) / "
                "avbd_rigid_fixed_joint_contact - fixed-joint contact route"
            ),
            (
                "Variational fixed-joint/contact capability scene; not a "
                "World contact-policy comparison."
            ),
        ),
        (
            "rigid_joint_breakage",
            "avbd_rigid_breakable_joint",
            "AVBD Rigid Constraints (sx)",
            (
                "Related shelf: AVBD Rigid Constraints (sx) / "
                "avbd_rigid_breakable_joint - free-rigid fixed break/reset"
            ),
            (
                "Dedicated free-rigid fixed break/reset row; not sequential-"
                "impulse or IPC parity evidence."
            ),
        ),
        (
            "rigid_joint_breakage",
            "avbd_rigid_spherical_breakable_joint",
            "AVBD Rigid Constraints (sx)",
            (
                "Related shelf: AVBD Rigid Constraints (sx) / "
                "avbd_rigid_spherical_breakable_joint - spherical anchor break/reset"
            ),
            (
                "Dedicated free-rigid spherical anchor break/reset row; "
                "orientation remains intentionally free."
            ),
        ),
        (
            "rigid_joint_motor_limits",
            "avbd_rigid_revolute_motor",
            "AVBD Rigid Constraints (sx)",
            (
                "Related shelf: AVBD Rigid Constraints (sx) / "
                "avbd_rigid_revolute_motor - free-rigid hinge motor"
            ),
            (
                "AVBD free-rigid revolute velocity motor; not a World "
                "multibody motor/limit comparison."
            ),
        ),
        (
            "rigid_joint_motor_limits",
            "avbd_rigid_prismatic_motor",
            "AVBD Rigid Constraints (sx)",
            (
                "Related shelf: AVBD Rigid Constraints (sx) / "
                "avbd_rigid_prismatic_motor - free-rigid slider motor"
            ),
            (
                "AVBD free-rigid prismatic velocity motor; not a World "
                "multibody motor/limit comparison."
            ),
        ),
    ]
    assert set(_RIGID_WORKFLOW_RELATED_EVIDENCE) == {
        source_scene_id
        for source_scene_id, _target_scene_id, _shelf, _label, _scope in related_rows
    }
    assert [
        (source_scene_id, entry.scene_id)
        for source_scene_id, entries in _RIGID_WORKFLOW_RELATED_EVIDENCE.items()
        for entry in entries
    ] == [
        (source_scene_id, target_scene_id)
        for source_scene_id, target_scene_id, _shelf, _label, _scope in related_rows
    ]
    for source_scene_id, target_scene_id, shelf, label, scope in related_rows:
        assert source_scene_id in RIGID_VISUAL_WORKFLOW_GUIDES
        entries = _RIGID_WORKFLOW_RELATED_EVIDENCE[source_scene_id]
        matching_entries = [
            entry for entry in entries if entry.scene_id == target_scene_id
        ]
        assert len(matching_entries) == 1
        entry = matching_entries[0]
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
        ),
        (
            "rigid_ipc_heavy_stack_packet",
            "How does a taller, top-heavy IPC stack behave beyond the live demo budget?",
            "Friction, box count, top mass, frame-budget threshold, min clearance, contact count, top drift, height error, max speed, wall time, and benchmark pointer.",
            "pixi run py-demo-capture -- --scene rigid_ipc_heavy_stack_packet --frames 12 --width 960 --height 540 --show-ui",
            "Taller capture-first stress packet; not a numbered workflow row and not a solver-performance parity claim.",
        ),
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
            12 if scene_id == "rigid_ipc_heavy_stack_packet" else 24,
            960,
            540,
            True,
        )


def test_rigid_visual_routes_publish_self_describing_capture_metrics() -> None:
    capture_py_demo = _capture_py_demo_module()
    scenes = make_demo_scenes()
    by_id = {scene.id: scene for scene in scenes}
    workflow_ids = [scene_id for _order, scene_id in _read_rigid_visual_workflow_rows()]
    related_rows = _read_rigid_visual_related_evidence_rows()
    capture_first_rows = _read_rigid_visual_capture_first_ipc_rows()

    for scene_id in workflow_ids:
        setup = by_id[scene_id].build()
        capture_metrics = setup.info.get(CAPTURE_METRICS_INFO_KEY)
        assert callable(capture_metrics), scene_id
        metrics = capture_metrics()
        assert metrics["row"] == scene_id
        assert len(metrics) > 1
        assert capture_py_demo._resolved_solver_identity_from_metrics(metrics), scene_id

    for source_scene_id, target_scene_id, shelf, _label, _scope in related_rows:
        setup = by_id[target_scene_id].build()
        capture_metrics = setup.info.get(CAPTURE_METRICS_INFO_KEY)
        assert callable(capture_metrics), target_scene_id
        metrics = capture_metrics()
        assert metrics["row"] == target_scene_id
        assert metrics["related_source_row"] == source_scene_id
        assert by_id[target_scene_id].category == shelf
        assert target_scene_id not in workflow_ids
        assert capture_py_demo._resolved_solver_identity_from_metrics(
            metrics
        ), target_scene_id

    for scene_id, _question, _signals, _command, _scope in capture_first_rows:
        setup = by_id[scene_id].build()
        capture_metrics = setup.info.get(CAPTURE_METRICS_INFO_KEY)
        assert callable(capture_metrics), scene_id
        metrics = capture_metrics()
        assert metrics["row"] == scene_id
        assert metrics["capture_first"] is True
        assert scene_id not in workflow_ids
        assert capture_py_demo._resolved_solver_identity_from_metrics(metrics), scene_id


def test_rigid_visual_workflow_capture_metric_docs_match_hooks() -> None:
    scenes = make_demo_scenes()
    by_id = {scene.id: scene for scene in scenes}
    workflow_ids = [scene_id for _order, scene_id in _read_rigid_visual_workflow_rows()]
    sidecar_docs = _read_rigid_visual_workflow_capture_metric_docs()

    assert set(sidecar_docs) == set(workflow_ids)
    for scene_id in workflow_ids:
        setup = by_id[scene_id].build()
        assert callable(setup.info.get(CAPTURE_METRICS_INFO_KEY)), scene_id
        assert "capture metrics" in sidecar_docs[scene_id].lower(), scene_id


def test_world_related_evidence_routes_report_capture_metrics() -> None:
    import numpy as np

    from examples.demos.scenes.articulated import (
        SCENE as ARTICULATED_SCENE,
        build as build_articulated,
    )
    from examples.demos.scenes.floating_base import (
        SCENE as FLOATING_BASE_SCENE,
        build as build_floating_base,
    )

    assert FLOATING_BASE_SCENE.category == "World Rigid Body"
    assert FLOATING_BASE_SCENE.id == "floating_base"
    floating_setup = build_floating_base()
    assert callable(floating_setup.info[CAPTURE_METRICS_INFO_KEY])
    assert floating_setup.pre_step is not None
    for _ in range(8):
        floating_setup.pre_step()
        floating_setup.world.step()

    floating_metrics = floating_setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert floating_metrics["row"] == "floating_base"
    assert floating_metrics["related_source_row"] == "rigid_free_flight"
    assert floating_metrics["solver"] == "floating_joint_se3"
    assert floating_metrics["scope"] == "broader_floating_joint_drift_spin"
    assert floating_metrics["dofs"] == pytest.approx(float(floating_setup.info["dofs"]))
    assert floating_metrics["time_step_ms"] == pytest.approx(10.0)
    assert floating_metrics["history"]["samples"] >= 9.0
    assert floating_metrics["linear_speed"] == pytest.approx(1.0, abs=0.01)
    assert floating_metrics["angular_speed"] == pytest.approx(2.0)
    assert floating_metrics["world_time"] > 0.0
    assert floating_metrics["controls"]["spin_command"] == pytest.approx(
        floating_metrics["spin_command"]
    )
    assert np.isfinite(
        [
            float(floating_metrics["body_x"]),
            float(floating_metrics["body_y"]),
            float(floating_metrics["body_z"]),
            float(floating_metrics["spin_command"]),
            float(floating_metrics["history"]["max_body_x"]),
            float(floating_metrics["history"]["min_body_x"]),
        ]
    ).all()

    assert ARTICULATED_SCENE.category == "World Rigid Body"
    assert ARTICULATED_SCENE.id == "articulated"
    articulated_setup = build_articulated()
    assert callable(articulated_setup.info[CAPTURE_METRICS_INFO_KEY])
    assert articulated_setup.pre_step is not None
    for _ in range(8):
        articulated_setup.pre_step()
        articulated_setup.world.step()

    articulated_metrics = articulated_setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert articulated_metrics["row"] == "articulated"
    assert (
        articulated_metrics["related_source_row"]
        == "rigid_multibody_dynamics_terms"
    )
    assert articulated_metrics["solver"] == "articulated_sx_world"
    assert articulated_metrics["scope"] == "broader_two_link_arm"
    assert articulated_metrics["dofs"] == pytest.approx(
        float(articulated_setup.info["dofs"])
    )
    assert articulated_metrics["link_count"] == pytest.approx(3.0)
    assert articulated_metrics["history"]["samples"] >= 9.0
    assert articulated_metrics["world_time"] > 0.0
    assert articulated_metrics["controls"]["shoulder_damping"] == pytest.approx(
        articulated_metrics["shoulder_damping"]
    )
    assert articulated_metrics["controls"]["wrist_damping"] == pytest.approx(
        articulated_metrics["wrist_damping"]
    )
    assert np.isfinite(
        [
            float(articulated_metrics["time_step_ms"]),
            float(articulated_metrics["shoulder_speed"]),
            float(articulated_metrics["wrist_speed"]),
            float(articulated_metrics["max_joint_speed"]),
            float(articulated_metrics["forearm_height"]),
            float(articulated_metrics["shoulder_damping"]),
            float(articulated_metrics["wrist_damping"]),
            float(articulated_metrics["shoulder_position"]),
            float(articulated_metrics["wrist_position_norm"]),
            float(articulated_metrics["history"]["max_joint_speed"]),
            float(articulated_metrics["history"]["min_forearm_height"]),
        ]
    ).all()


def test_rigid_ipc_shelf_scenes_report_capture_metrics() -> None:
    import numpy as np

    sx = _require_simulation_symbols("RigidBodySolver")

    from examples.demos.scenes import (
        rigid_ipc,
        rigid_ipc_incline,
        rigid_ipc_pile,
        rigid_ipc_slide,
    )

    scenes = [
        (
            rigid_ipc,
            "rigid_ipc",
            "basic_box_ground_barrier_settle",
            ("box_height", "box_speed", "clearance", "max_speed", "min_clearance"),
        ),
        (
            rigid_ipc_slide,
            "rigid_ipc_slide",
            "friction_braked_tangential_slide",
            ("box_x", "box_speed", "clearance", "max_travel", "min_speed"),
        ),
        (
            rigid_ipc_incline,
            "rigid_ipc_incline",
            "tilted_face_friction_slide",
            (
                "down_slope_speed",
                "down_slope_travel",
                "max_down_slope_speed",
                "max_down_slope_travel",
                "ramp_gap",
            ),
        ),
        (
            rigid_ipc_pile,
            "rigid_ipc_pile",
            "multi_box_barrier_pile",
            ("box_count", "max_history_speed", "max_span_x", "mean_height", "span_x"),
        ),
    ]

    for module, scene_id, scope, numeric_keys in scenes:
        assert module.SCENE.category == "Rigid IPC"
        assert module.SCENE.id == scene_id

        setup = module.build()
        world = setup.info["sx_world"]
        assert setup.info["rigid_body_solver"] == "ipc"
        assert world.rigid_body_solver == sx.RigidBodySolver.IPC
        assert setup.panels
        assert callable(setup.info[CAPTURE_METRICS_INFO_KEY])
        assert callable(setup.info["replay_sync"])
        assert setup.info["replay_live_step_is_stateless"] is True

        for _ in range(36):
            setup.pre_step()

        capture_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
        assert capture_metrics["row"] == scene_id
        assert capture_metrics["solver"] == "rigid_ipc"
        assert capture_metrics["scope"] == scope
        assert capture_metrics["status"] in {
            "barrier-held",
            "barrier-pile",
            "barrier-slide",
            "braked",
            "coasting",
            "falling",
            "held",
            "near-barrier",
            "released",
            "settled",
            "sliding",
        }
        assert float(capture_metrics["time_step_ms"]) == pytest.approx(5.0)
        assert float(capture_metrics["world_time"]) > 0.0
        assert float(capture_metrics["history_samples"]) >= 37.0
        assert float(capture_metrics["max_step_ms"]) >= 0.0
        assert np.isfinite(
            [
                float(capture_metrics["contact_count"]),
                float(capture_metrics["friction"]),
                float(capture_metrics["max_contact_count"]),
                float(capture_metrics["world_time"]),
                *(float(capture_metrics[key]) for key in numeric_keys),
            ]
        ).all()
        assert capture_metrics["controls"]["friction"] == pytest.approx(
            capture_metrics["friction"]
        )


def test_rigid_ipc_tunnel_reports_no_tunneling_metrics() -> None:
    import numpy as np

    sx = _require_simulation_symbols("RigidBodySolver")

    from examples.demos.scenes.rigid_ipc_tunnel import SCENE, build

    assert SCENE.category == "Rigid IPC"
    assert SCENE.id == "rigid_ipc_tunnel"

    setup = build()
    world = setup.info["sx_world"]
    assert setup.info["rigid_body_solver"] == "ipc"
    assert world.rigid_body_solver == sx.RigidBodySolver.IPC
    assert setup.info["rigid_ipc_tunnel_capture_first"] is False

    for _ in range(12):
        setup.pre_step()

    capture_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert capture_metrics["row"] == "rigid_ipc_tunnel"
    assert capture_metrics["related_source_row"] == "rigid_solver_compare"
    assert capture_metrics["solver"] == "rigid_ipc"
    assert capture_metrics["scope"] == "focused_no_tunneling_capability"
    assert capture_metrics["status"] in {
        "approaching",
        "barrier-active",
        "barrier-held",
    }
    assert float(capture_metrics["launch_speed"]) == pytest.approx(8.0)
    assert float(capture_metrics["time_step_ms"]) == pytest.approx(10.0)
    assert float(capture_metrics["world_time"]) > 0.0
    assert float(capture_metrics["history_samples"]) >= 13.0
    assert float(capture_metrics["min_tunnel_margin"]) > 0.0
    assert float(capture_metrics["max_wall_crossing"]) < 0.01
    assert float(capture_metrics["max_step_ms"]) >= 0.0
    assert np.isfinite(
        [
            float(capture_metrics["box_x"]),
            float(capture_metrics["box_speed"]),
            float(capture_metrics["box_vx"]),
            float(capture_metrics["clearance"]),
            float(capture_metrics["contact_count"]),
            float(capture_metrics["leading_face_x"]),
            float(capture_metrics["min_clearance"]),
            float(capture_metrics["min_tunnel_margin"]),
            float(capture_metrics["trailing_face_x"]),
            float(capture_metrics["tunnel_margin"]),
            float(capture_metrics["wall_left_face_x"]),
            float(capture_metrics["wall_right_face_x"]),
        ]
    ).all()


def test_rigid_ipc_edge_drop_reports_degenerate_contact_metrics() -> None:
    import numpy as np

    sx = _require_simulation_symbols("RigidBodySolver")

    from examples.demos.scenes.rigid_ipc_edge_drop import SCENE, build

    assert SCENE.category == "Rigid IPC"
    assert SCENE.id == "rigid_ipc_edge_drop"

    setup = build()
    world = setup.info["physics_world"]
    assert setup.info["sx_world"] is world
    assert setup.info["rigid_body_solver"] == "ipc"
    assert world.rigid_body_solver == sx.RigidBodySolver.IPC
    assert setup.panels

    for _ in range(24):
        setup.pre_step()

    capture_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert capture_metrics["row"] == "rigid_ipc_edge_drop"
    assert capture_metrics["related_source_row"] == "rigid_solver_compare"
    assert capture_metrics["solver"] == "rigid_ipc"
    assert capture_metrics["scope"] == "degenerate_edge_contact_capability"
    assert capture_metrics["status"] in {
        "airborne",
        "approaching",
        "barrier-held",
        "edge-barrier",
        "edge-contact",
        "barrier-contact",
        "settled",
    }
    assert float(capture_metrics["time_step_ms"]) == pytest.approx(5.0)
    assert float(capture_metrics["world_time"]) > 0.0
    assert float(capture_metrics["history_samples"]) >= 25.0
    assert float(capture_metrics["min_barrier_gap"]) <= 0.01
    assert float(capture_metrics["max_angular_speed"]) >= 0.05
    assert float(capture_metrics["max_tilt_deg"]) > 45.0
    assert np.isfinite(
        [
            float(capture_metrics["angular_speed"]),
            float(capture_metrics["clearance"]),
            float(capture_metrics["contact_count"]),
            float(capture_metrics["cube_z"]),
            float(capture_metrics["friction"]),
            float(capture_metrics["initial_angular_speed"]),
            float(capture_metrics["max_angular_speed"]),
            float(capture_metrics["max_contact_count"]),
            float(capture_metrics["max_step_ms"]),
            float(capture_metrics["max_tilt_deg"]),
            float(capture_metrics["min_barrier_gap"]),
            float(capture_metrics["min_clearance"]),
            float(capture_metrics["min_tilt_deg"]),
            float(capture_metrics["tilt_deg"]),
            float(capture_metrics["vertical_speed"]),
        ]
    ).all()


def test_diff_drone_liftoff_reports_contact_gradient_metrics() -> None:
    import numpy as np

    _require_simulation_symbols("World")

    from examples.demos.scenes.diff_drone_liftoff import SCENE, build

    assert SCENE.category == "Differentiable"
    assert SCENE.id == "diff_drone_liftoff"

    setup = build()
    assert callable(setup.info[CAPTURE_METRICS_INFO_KEY])

    for _ in range(min(4, int(setup.info["steps"]))):
        setup.pre_step()

    capture_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert capture_metrics["row"] == "diff_drone_liftoff"
    assert capture_metrics["category"] == "Differentiable"
    assert capture_metrics["related_source_row"] == "rigid_contact_solver_compare"
    assert capture_metrics["solver"] == "boxed_lcp_contact_gradient_modes"
    assert capture_metrics["scope"] == "rigid_contact_gradient_saddle_escape"
    assert capture_metrics["contact_solver_method"] == "BOXED_LCP"
    assert capture_metrics["gradient_modes"] == [
        "ANALYTIC",
        "COMPLEMENTARITY_AWARE",
    ]
    assert capture_metrics["optimized"] is setup.info["optimized"]
    assert capture_metrics["time_step_ms"] == pytest.approx(10.0)
    assert capture_metrics["horizon"] == pytest.approx(150.0)
    assert capture_metrics["max_iters"] == pytest.approx(400.0)
    assert capture_metrics["learning_rate"] == pytest.approx(4.0)
    assert capture_metrics["target_z"] == pytest.approx(1.5)
    assert capture_metrics["steps"] == pytest.approx(float(setup.info["steps"]))
    assert capture_metrics["aware_history_samples"] >= 1.0
    assert capture_metrics["naive_history_samples"] >= 1.0
    assert capture_metrics["history"]["aware_height"]["samples"] >= 1.0
    assert capture_metrics["history"]["aware_thrust_gradient"]["samples"] >= 1.0
    assert capture_metrics["modes"]["ANALYTIC"]["final_z"] == pytest.approx(
        capture_metrics["naive_final_z"]
    )
    assert capture_metrics["modes"]["COMPLEMENTARITY_AWARE"][
        "final_z"
    ] == pytest.approx(capture_metrics["aware_final_z"])
    assert np.isfinite(
        [
            float(capture_metrics["current_z"]),
            float(capture_metrics["naive_thrust"]),
            float(capture_metrics["naive_final_z"]),
            float(capture_metrics["naive_loss"]),
            float(capture_metrics["aware_thrust"]),
            float(capture_metrics["aware_final_z"]),
            float(capture_metrics["aware_loss"]),
            float(capture_metrics["height_gap"]),
            float(capture_metrics["aware_target_error"]),
            float(capture_metrics["naive_target_error"]),
            float(capture_metrics["target_error_gap"]),
            float(capture_metrics["thrust_gap"]),
            float(capture_metrics["aware_min_height"]),
            float(capture_metrics["aware_max_height"]),
            float(capture_metrics["aware_max_thrust"]),
            float(capture_metrics["aware_last_thrust_gradient"]),
            float(capture_metrics["aware_min_loss"]),
            float(capture_metrics["naive_min_loss"]),
        ]
    ).all()

    if setup.info["optimized"]:
        assert capture_metrics["status"] == "saddle_escape"
        assert capture_metrics["aware_final_z"] > capture_metrics["naive_final_z"]
        assert capture_metrics["height_gap"] > 0.0
        assert capture_metrics["target_error_gap"] > 0.0
        assert capture_metrics["aware_thrust"] >= capture_metrics["naive_thrust"]
    else:
        assert capture_metrics["status"] == "fallback"
        assert capture_metrics["height_gap"] == pytest.approx(0.0)
        assert capture_metrics["thrust_gap"] == pytest.approx(0.0)


def test_diff_pre_contact_surrogate_reports_pre_contact_metrics() -> None:
    import numpy as np

    _require_simulation_symbols("World")

    from examples.demos.scenes.diff_pre_contact_surrogate import SCENE, build

    assert SCENE.category == "Differentiable"
    assert SCENE.id == "diff_pre_contact_surrogate"

    setup = build()
    assert callable(setup.info[CAPTURE_METRICS_INFO_KEY])
    assert callable(setup.pre_step)
    setup.pre_step()

    capture_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert capture_metrics["row"] == "diff_pre_contact_surrogate"
    assert capture_metrics["category"] == "Differentiable"
    assert capture_metrics["related_source_row"] == "rigid_contact_solver_compare"
    assert capture_metrics["solver"] == "boxed_lcp_pre_contact_surrogate"
    assert capture_metrics["scope"] == "pre_contact_backward_only_surrogate"
    assert capture_metrics["contact_solver_method"] == "BOXED_LCP"
    assert capture_metrics["executor"] == "World.step default"
    assert capture_metrics["gradient_modes"] == [
        "ANALYTIC",
        "PRE_CONTACT_SURROGATE",
    ]
    assert capture_metrics["time_step_ms"] == pytest.approx(1.0)
    assert capture_metrics["sphere_radius"] == pytest.approx(0.5)
    assert capture_metrics["sphere_mass"] == pytest.approx(2.0)
    assert capture_metrics["initial_center_z"] == pytest.approx(1.0)
    assert capture_metrics["initial_clearance"] == pytest.approx(0.5)
    assert capture_metrics["initial_vz"] == pytest.approx(-0.5)
    assert capture_metrics["approach_speed"] == pytest.approx(0.5)
    assert capture_metrics["pre_step_contact_count"] == pytest.approx(0.0)
    assert capture_metrics["surrogate_pre_step_contact_count"] == pytest.approx(0.0)
    assert capture_metrics["analytic_next_z"] == pytest.approx(
        capture_metrics["surrogate_next_z"]
    )
    assert capture_metrics["modes"]["ANALYTIC"]["next_z"] == pytest.approx(
        capture_metrics["analytic_next_z"]
    )
    assert capture_metrics["modes"]["PRE_CONTACT_SURROGATE"][
        "next_z"
    ] == pytest.approx(capture_metrics["surrogate_next_z"])
    assert np.isfinite(
        [
            float(capture_metrics["post_step_clearance"]),
            float(capture_metrics["surrogate_post_step_clearance"]),
            float(capture_metrics["forward_state_max_abs_diff"]),
            float(capture_metrics["analytic_freefall_error"]),
            float(capture_metrics["surrogate_block_magnitude"]),
            float(capture_metrics["analytic_dvzprime_dvz"]),
            float(capture_metrics["surrogate_dvzprime_dvz"]),
            float(capture_metrics["surrogate_delta_dvzprime_dvz"]),
            float(capture_metrics["surrogate_delta_dzprime_dvz"]),
            float(capture_metrics["in_plane_sensitivity_error"]),
        ]
    ).all()

    if capture_metrics["differentiable_available"]:
        assert capture_metrics["status"] == "pre_contact_surrogate"
        assert capture_metrics["thresholds_pass"] is True
        assert capture_metrics["forward_state_max_abs_diff"] < 1e-12
        assert capture_metrics["analytic_freefall_error"] < 1e-9
        assert capture_metrics["surrogate_block_magnitude"] > 1e-3
        assert capture_metrics["surrogate_dvzprime_dvz"] < 0.5
        assert capture_metrics["in_plane_sensitivity_error"] < 1e-9
    else:
        assert capture_metrics["status"] == "fallback"
        assert capture_metrics["thresholds_pass"] is False
        assert capture_metrics["surrogate_block_magnitude"] == pytest.approx(0.0)


def test_rigid_ipc_stack_packet_reports_capture_first_metrics() -> None:
    import numpy as np

    sx = _require_simulation_symbols("RigidBodySolver")

    from examples.demos.scenes.rigid_ipc_stack_packet import (
        HEAVY_SCENE,
        SCENE,
        build,
        build_heavy,
    )

    cases = [
        (SCENE, build, "rigid_ipc_stack_packet", 4.0),
        (HEAVY_SCENE, build_heavy, "rigid_ipc_heavy_stack_packet", 6.0),
    ]
    for scene, build_scene, scene_id, box_count in cases:
        assert scene.category == "Rigid IPC"
        assert scene.id == scene_id

        setup = build_scene()
        controller = setup.info[f"{scene_id}_controller"]
        assert setup.info[f"{scene_id}_capture_first"] is True
        assert setup.info[f"{scene_id}_benchmark"] == "bm_rigid_ipc_solver"
        assert setup.info["rigid_ipc_stack_packet_variant"] == scene_id
        assert controller.world.rigid_body_solver == sx.RigidBodySolver.IPC

        for _ in range(2):
            setup.pre_step()

        metrics = controller._last_metrics
        capture_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
        assert float(metrics["box_count"]) == pytest.approx(box_count)
        assert float(metrics["min_clearance"]) > -0.004
        assert float(metrics["max_speed"]) >= 0.0
        assert float(metrics["top_drift"]) >= 0.0
        assert float(metrics["step_ms"]) >= 0.0
        assert str(metrics["status"]) in {"capture-first", "settling", "standing"}
        assert capture_metrics["row"] == scene_id
        assert capture_metrics["capture_first"] is True
        assert capture_metrics["benchmark"] == "bm_rigid_ipc_solver"
        assert capture_metrics["solver"] == "ipc"
        assert float(capture_metrics["box_count"]) == pytest.approx(box_count)
        assert float(capture_metrics["frame_budget_ms"]) > 0.0
        assert capture_metrics["controls"]["friction"] == pytest.approx(
            capture_metrics["friction"]
        )
        assert capture_metrics["controls"]["frame_budget_ms"] == pytest.approx(
            capture_metrics["frame_budget_ms"]
        )
        assert float(capture_metrics["top_mass"]) > 0.0
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
    text = sidecar.read_text(encoding="utf-8")
    normalized_text = re.sub(r"\s+", " ", text)

    rigid_body_attrs = set(dir(sx.RigidBody))
    world_attrs = set(dir(sx.World))
    loop_attrs = set(dir(sx.LoopClosure)) | set(dir(sx.LoopClosureSpec))

    assert {
        "apply_force",
        "apply_torque",
        "apply_linear_impulse",
        "apply_angular_impulse",
        "linear_momentum",
        "angular_momentum",
    } <= rigid_body_attrs
    assert "compute_impulse_response" in set(dir(sx.Multibody))

    assert "public direct rigid-body impulse surface" in normalized_text

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
    assert "no public sleep/wake or island activation surface" in normalized_text

    compliance_attrs = {
        name
        for name in loop_attrs
        if any(token in name.lower() for token in ("compliance", "stiffness", "damping"))
    }
    assert not compliance_attrs, (
        "Public loop-closure compliance API appeared; update the rigid-body "
        "visual workflow and revisit the compliance row deferral."
    )
    assert "no public loop-closure compliance surface" in normalized_text

    readme = root / "python" / "examples" / "demos" / "README.md"
    for path in (sidecar, readme):
        path_text = re.sub(r"\s+", " ", path.read_text(encoding="utf-8"))
        assert "direct rigid body impulse" in path_text
        assert "direct linear impulse" in path_text
        assert "direct angular impulse" in path_text
        assert "sleep wake" in path_text
        assert "island activation" in path_text
        assert "loop closure compliance" in path_text
        assert "deferred API caveat" in path_text


def test_rigid_visual_verification_readme_matches_sidecar_order() -> None:
    sidecar_rows = _read_rigid_visual_workflow_rows()
    sidecar_ids = [scene_id for _order, scene_id in sidecar_rows]
    readme_rows = _read_rigid_visual_readme_workflow_rows()
    readme_ids = [scene_id for _order_label, scene_id in readme_rows]

    assert readme_ids == sidecar_ids
    assert [order_label for order_label, _scene_id in readme_rows] == [
        f"{order:02d}/{len(sidecar_rows):02d}" for order, _scene_id in sidecar_rows
    ]


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
    assert list(RIGID_VISUAL_WORKFLOW_CAPTURE_SPECS) == table_specs
    for scene_id, frames, width, height, show_ui in table_specs:
        command = RIGID_VISUAL_WORKFLOW_GUIDES[scene_id].capture_command
        expected_command = (
            "pixi run py-demo-capture -- "
            f"--scene {scene_id} --frames {frames} --width {width} --height {height}"
        )
        if show_ui:
            expected_command = f"{expected_command} --show-ui"
        assert command == expected_command
        assert show_ui is True
    capture_py_demo = _capture_py_demo_module()
    assert list(capture_py_demo.rigid_workflow_capture_specs()) == table_specs
    assert all(
        (width, height, show_ui) == (960, 540, True)
        for _scene_id, _frames, width, height, show_ui in table_specs
    )


def test_rigid_visual_motion_capture_video_flags_are_documented() -> None:
    root = pathlib.Path(__file__).resolve().parents[3]
    helper = root / "scripts" / "capture_py_demo.py"
    helper_text = helper.read_text(encoding="utf-8")
    if "--video" not in helper_text or "--fps" not in helper_text:
        pytest.skip("py-demo-capture does not expose video flags in this checkout")

    expected_tokens = (
        "pixi run py-demo-capture -- --scene rigid_solver_compare"
        " --frames 72 --width 960 --height 540 --show-ui --video --fps 24"
    )
    for path in (
        root / "python" / "examples" / "demos" / "README.md",
        root
        / "docs"
        / "plans"
        / "103-examples-strategy"
        / "rigid-body-visual-verification.md",
    ):
        text = " ".join(
            line.strip().removesuffix("\\").strip()
            for line in path.read_text(encoding="utf-8").splitlines()
        )
        assert "--video" in text
        assert "--fps 24" in text
        assert expected_tokens in text


def test_rigid_visual_workflow_continue_on_failure_is_documented() -> None:
    root = pathlib.Path(__file__).resolve().parents[3]
    helper_text = (root / "scripts" / "capture_py_demo.py").read_text(
        encoding="utf-8"
    )
    assert "--continue-on-failure" in helper_text

    expected_tokens = (
        "pixi run py-demo-capture -- --rigid-workflow --include-related"
        " --include-ipc-shelf --include-packets --continue-on-failure"
        " --output-dir /tmp/dart_capture_rigid_workflow_resilient"
    )
    for path in (
        root / "python" / "examples" / "demos" / "README.md",
        root
        / "docs"
        / "plans"
        / "103-examples-strategy"
        / "rigid-body-visual-verification.md",
    ):
        text = " ".join(
            line.strip().removesuffix("\\").strip()
            for line in path.read_text(encoding="utf-8").splitlines()
        )
        assert "continue_on_failure=true" in text
        assert "failed_rows" in text
        assert "Failed Rows summary" in text
        assert "workflow row-range rerun" in text
        assert expected_tokens in text


def test_root_readme_source_checkout_points_to_py_demos_rigid_front_door() -> None:
    root = pathlib.Path(__file__).resolve().parents[3]
    text = (root / "README.md").read_text(encoding="utf-8")
    source_checkout = text.split("### Source checkout", maxsplit=1)[1].split(
        "## Documentation",
        maxsplit=1,
    )[0]

    python_smoke = "pixi run py-demos -- --scene rigid_body --headless --frames 1"
    capture_command = (
        "pixi run py-demo-capture -- --scene rigid_solver_compare"
        " --frames 24 --width 960 --height 540 --show-ui"
    )
    cxx_smoke = "pixi run demos -- --scene rigid_body --headless --frames 1"
    assert python_smoke in source_checkout
    assert "pixi run py-demos" in source_checkout
    assert "pixi run py-demos -- --scene rigid_solver_compare" in source_checkout
    assert capture_command in source_checkout
    assert "Rigid Workflow" in source_checkout
    assert (
        "python/examples/demos/README.md#rigid-body-visual-verification-workflow"
        in source_checkout
    )
    assert cxx_smoke in source_checkout
    assert source_checkout.index(python_smoke) < source_checkout.index(cxx_smoke)
    assert source_checkout.index(capture_command) < source_checkout.index(cxx_smoke)


def test_rigid_visual_related_evidence_capture_commands_are_documented() -> None:
    root = pathlib.Path(__file__).resolve().parents[3]
    sidecar = (
        root
        / "docs"
        / "plans"
        / "103-examples-strategy"
        / "rigid-body-visual-verification.md"
    )
    readme = root / "python" / "examples" / "demos" / "README.md"
    expected_specs = [
        ("floating_base", 72, 960, 540, True),
        ("articulated", 72, 960, 540, True),
        ("rigid_ipc_tunnel", 24, 960, 540, True),
        ("rigid_ipc_edge_drop", 72, 960, 540, True),
        ("diff_drone_liftoff", 96, 960, 540, True),
        ("diff_pre_contact_surrogate", 24, 960, 540, True),
        ("avbd_rigid_fixed_joint_contact", 72, 960, 540, True),
        ("avbd_rigid_breakable_joint", 72, 960, 540, True),
        ("avbd_rigid_spherical_breakable_joint", 72, 960, 540, True),
        ("avbd_rigid_revolute_motor", 72, 960, 540, True),
        ("avbd_rigid_prismatic_motor", 72, 960, 540, True),
    ]

    related_scene_ids = [
        target_scene_id
        for _source_scene_id, target_scene_id, _shelf, _label, _scope in (
            _read_rigid_visual_related_evidence_rows()
        )
    ]
    assert [scene_id for scene_id, *_rest in expected_specs] == related_scene_ids

    marker = "Capture every non-numbered related-evidence route with the docked UI visible:"
    sidecar_specs = _read_capture_command_specs(sidecar, marker)
    readme_specs = _read_capture_command_specs(readme, marker)

    assert sidecar_specs == expected_specs
    assert readme_specs == expected_specs
    capture_py_demo = _capture_py_demo_module()
    assert list(capture_py_demo.rigid_workflow_related_capture_specs()) == expected_specs


def test_rigid_visual_capture_first_packets_are_documented() -> None:
    root = pathlib.Path(__file__).resolve().parents[3]
    readme = root / "python" / "examples" / "demos" / "README.md"
    sidecar = (
        root
        / "docs"
        / "plans"
        / "103-examples-strategy"
        / "rigid-body-visual-verification.md"
    )
    expected_specs = [
        ("rigid_ipc_stack_packet", 24, 960, 540, True),
        ("rigid_ipc_heavy_stack_packet", 12, 960, 540, True),
    ]

    packet_rows = _read_rigid_visual_capture_first_ipc_rows()
    assert [
        _capture_spec_from_command(command)
        for _scene_id, _question, _signals, command, _scope in packet_rows
    ] == expected_specs

    marker = "Capture the heavier Rigid IPC stack packets when the question is what happens"
    readme_specs = _read_capture_command_specs(readme, marker)
    assert readme_specs == expected_specs

    capture_py_demo = _capture_py_demo_module()
    assert list(capture_py_demo.rigid_workflow_packet_capture_specs()) == expected_specs
    related_packet_start = (
        len(RIGID_VISUAL_WORKFLOW_CAPTURE_SPECS)
        + len(capture_py_demo.rigid_workflow_related_capture_specs())
        + 1
    )
    related_packet_end = related_packet_start + len(expected_specs) - 1
    full_packet_start = (
        len(RIGID_VISUAL_WORKFLOW_CAPTURE_SPECS)
        + len(capture_py_demo.rigid_workflow_related_capture_specs())
        + len(capture_py_demo.rigid_workflow_ipc_shelf_capture_specs())
        + 1
    )
    full_packet_end = full_packet_start + len(expected_specs) - 1

    row_range_marker = (
        "For targeted reruns after a failed or manually inspected row"
    )
    for path in (readme, sidecar):
        row_range_tail = path.read_text(encoding="utf-8").split(
            row_range_marker, 1
        )[1]
        row_range_intro, row_range_rest = row_range_tail.split("```bash", 1)
        row_range_block = row_range_rest.split("```", 1)[0]
        row_range_section = row_range_intro + row_range_block
        expected_range = (
            f"--workflow-start-row {related_packet_start} "
            f"--workflow-end-row {related_packet_end}"
        )
        assert expected_range in row_range_section
        row_range_words = " ".join(row_range_section.split())
        assert (
            f"rows {related_packet_start}-{related_packet_end} are the two "
            "capture-first stack packets"
        ) in row_range_words
        assert (
            f"packet rows become {full_packet_start}-{full_packet_end}"
            in row_range_words
        )


def test_rigid_visual_direct_ipc_shelf_captures_are_documented() -> None:
    root = pathlib.Path(__file__).resolve().parents[3]
    sidecar = (
        root
        / "docs"
        / "plans"
        / "103-examples-strategy"
        / "rigid-body-visual-verification.md"
    )
    readme = root / "python" / "examples" / "demos" / "README.md"
    expected_specs = [
        ("rigid_ipc", 72, 960, 540, True),
        ("rigid_ipc_slide", 72, 960, 540, True),
        ("rigid_ipc_incline", 72, 960, 540, True),
        ("rigid_ipc_pile", 72, 960, 540, True),
    ]
    marker = "Capture the direct Rigid IPC shelf routes with the docked UI visible:"
    sidecar_specs = _read_capture_command_specs(sidecar, marker)
    readme_specs = _read_capture_command_specs(readme, marker)

    assert sidecar_specs == expected_specs
    assert readme_specs == expected_specs
    capture_py_demo = _capture_py_demo_module()
    assert list(capture_py_demo.rigid_workflow_ipc_shelf_capture_specs()) == expected_specs


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
    assert controller._step_ms_history
    assert callable(setup.info[CAPTURE_METRICS_INFO_KEY])
    capture_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert capture_metrics["row"] == "rigid_contact_inspector"
    assert capture_metrics["solver"] == "collision_query"
    assert capture_metrics["contact_scope"] == "shape_pair_manifold_fields"
    assert capture_metrics["time_step_ms"] == pytest.approx(5.0)
    assert capture_metrics["world_time"] > 0.0
    assert capture_metrics["lane_count"] == len(controller.lanes)
    assert capture_metrics["selected_lane"]["key"] == controller._selected_lane().key
    assert capture_metrics["selected_lane"]["label"] == controller._selected_lane().label
    assert capture_metrics["controls"]["pair_index"] == controller.pair_index
    assert capture_metrics["controls"]["penetration"] == pytest.approx(
        controller.penetration
    )
    assert capture_metrics["total_contact_count"] == metrics["total_contact_count"]
    assert capture_metrics["selected_contact_count"] == metrics["selected_contact_count"]
    assert capture_metrics["max_depth"] == pytest.approx(metrics["max_depth"])
    assert capture_metrics["selected_max_depth"] == pytest.approx(
        metrics["selected_max_depth"]
    )
    assert capture_metrics["first_depth"] == pytest.approx(metrics["first_depth"])
    assert capture_metrics["first_pair"] == metrics["first_pair"]
    assert np.isfinite(capture_metrics["first_point"]).all()
    assert np.isfinite(capture_metrics["first_normal"]).all()
    assert np.linalg.norm(capture_metrics["first_normal"]) == pytest.approx(
        1.0,
        abs=1.0e-6,
    )
    assert np.isfinite(capture_metrics["first_local_a"]).all()
    assert np.isfinite(capture_metrics["first_local_b"]).all()
    assert capture_metrics["first_shape_indices"][0] >= 0
    assert capture_metrics["first_shape_indices"][1] >= 0
    assert capture_metrics["first_shape_index_a"] >= 0
    assert capture_metrics["first_shape_index_b"] >= 0
    assert capture_metrics["history"]["samples"] == pytest.approx(
        float(len(controller._contact_count_history))
    )
    assert capture_metrics["history"]["max_total_contact_count"] >= len(
        controller.lanes
    )
    assert capture_metrics["history"]["max_selected_depth"] == pytest.approx(
        controller.penetration
    )
    assert capture_metrics["metrics"]["first_shape_indices"] == list(
        metrics["first_shape_indices"]
    )

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
    assert callable(setup.info[CAPTURE_METRICS_INFO_KEY])
    capture_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert capture_metrics["row"] == "rigid_body_modes"
    assert capture_metrics["comparison_axis"] == "rigid_body_mode_semantics"
    assert capture_metrics["solver"] == "Sequential impulse"
    assert capture_metrics["solver_enum"] == controller._solver().name
    assert capture_metrics["executor"] == controller._executors[0][0]
    assert capture_metrics["held_fixed"] == {
        "solver": "Sequential impulse",
        "executor": controller._executors[0][0],
        "gravity_scale": pytest.approx(controller.gravity_scale),
        "force_magnitude": pytest.approx(controller.force_magnitude),
        "body_mass": pytest.approx(1.0),
        "time_step_ms": pytest.approx(capture_metrics["time_step_ms"]),
    }
    assert set(capture_metrics["lanes"]) == set(metrics)
    assert capture_metrics["dynamic_height"] == pytest.approx(dynamic["height"])
    assert capture_metrics["dynamic_displacement_x"] == pytest.approx(
        dynamic["displacement_x"]
    )
    assert capture_metrics["static_drift"] == pytest.approx(static["displacement"])
    assert capture_metrics["kinematic_error"] == pytest.approx(
        kinematic["kinematic_error"]
    )
    assert capture_metrics["history"]["samples"] == pytest.approx(
        float(len(controller._step_ms_history))
    )


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
    assert callable(setup.info[CAPTURE_METRICS_INFO_KEY])
    capture_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert capture_metrics["row"] == "rigid_body"
    assert capture_metrics["solver"] == controller._solver_label()
    assert capture_metrics["solver_enum"] == controller._solver().name
    assert capture_metrics["dynamic_body_count"] == pytest.approx(
        float(len(controller.dynamic_bodies))
    )
    assert capture_metrics["baseline_max_speed"] == pytest.approx(
        metrics["max_speed"]
    )
    assert capture_metrics["baseline_min_height"] == pytest.approx(
        metrics["min_height"]
    )
    assert capture_metrics["baseline_energy"] == pytest.approx(metrics["energy"])
    assert capture_metrics["baseline_scene_contact_count"] == pytest.approx(
        metrics["contact_count"]
    )
    assert capture_metrics["baseline_step_ms"] == pytest.approx(metrics["step_ms"])
    assert capture_metrics["controls"]["friction"] == pytest.approx(0.42)
    assert capture_metrics["controls"]["restitution"] == pytest.approx(0.31)
    assert capture_metrics["metrics"]["max_speed"] == pytest.approx(
        metrics["max_speed"]
    )
    assert capture_metrics["metrics"]["contact_count"] == pytest.approx(
        metrics["contact_count"]
    )
    assert capture_metrics["history"]["samples"] == pytest.approx(
        float(len(controller._speed_history))
    )
    assert capture_metrics["history"]["max_contacts"] >= 1.0

    controller.reset(clear_replay=False)
    assert controller.world.time == pytest.approx(0.0)
    assert len(controller._speed_history) == 1
    reset_capture_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert reset_capture_metrics["world_time"] == pytest.approx(0.0)
    assert reset_capture_metrics["history"]["samples"] == pytest.approx(1.0)
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
        "linear_impulse",
        "low_inertia_torque",
        "high_inertia_torque",
        "angular_impulse",
    }

    light = metrics["light_force"]
    heavy = metrics["heavy_force"]
    pulse = metrics["pulse_force"]
    static = metrics["static_load"]
    linear_impulse = metrics["linear_impulse"]
    low = metrics["low_inertia_torque"]
    high = metrics["high_inertia_torque"]
    angular_impulse = metrics["angular_impulse"]

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
    assert float(linear_impulse["linear_momentum_x"]) == pytest.approx(
        controller.linear_impulse_magnitude
    )
    assert float(linear_impulse["force_norm"]) == pytest.approx(0.0, abs=1.0e-12)
    assert str(linear_impulse["status"]) == "linear impulse applied"

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
    assert float(angular_impulse["angular_momentum_z"]) == pytest.approx(
        controller.angular_impulse_magnitude
    )
    assert float(angular_impulse["torque_norm"]) == pytest.approx(0.0, abs=1.0e-12)
    assert str(angular_impulse["status"]) == "angular impulse applied"

    assert float(static["speed"]) == pytest.approx(0.0, abs=1.0e-12)
    assert float(static["angular_speed"]) == pytest.approx(0.0, abs=1.0e-12)
    assert float(static["force_norm"]) == pytest.approx(controller.force_magnitude)
    assert float(static["torque_norm"]) == pytest.approx(controller.torque_magnitude)
    assert float(static["drift"]) == pytest.approx(0.0, abs=1.0e-12)
    assert np.isfinite([float(value) for value in controller._step_ms_history]).all()
    assert callable(setup.info[CAPTURE_METRICS_INFO_KEY])
    capture_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert capture_metrics["row"] == "rigid_external_loads"
    assert capture_metrics["solver"] == "sequential_impulse"
    assert capture_metrics["executor"] == controller._executors[0][0]
    assert set(capture_metrics["lanes"]) == set(metrics)
    assert capture_metrics["light_force_accel_x"] == pytest.approx(
        light["linear_accel_x"]
    )
    assert capture_metrics["heavy_force_accel_x"] == pytest.approx(
        heavy["linear_accel_x"]
    )
    assert capture_metrics["pulse_force_norm"] == pytest.approx(pulse["force_norm"])
    assert capture_metrics["static_drift"] == pytest.approx(static["drift"])
    assert capture_metrics["linear_impulse_momentum_x"] == pytest.approx(
        linear_impulse["linear_momentum_x"]
    )
    assert capture_metrics["linear_impulse_speed"] == pytest.approx(
        linear_impulse["speed"]
    )
    assert capture_metrics["low_inertia_angular_accel_z"] == pytest.approx(
        low["angular_accel_z"]
    )
    assert capture_metrics["angular_impulse_momentum_z"] == pytest.approx(
        angular_impulse["angular_momentum_z"]
    )
    assert capture_metrics["angular_impulse_speed"] == pytest.approx(
        angular_impulse["angular_speed"]
    )
    assert capture_metrics["controls"]["linear_impulse_magnitude"] == pytest.approx(
        controller.linear_impulse_magnitude
    )
    assert capture_metrics["controls"]["angular_impulse_magnitude"] == pytest.approx(
        controller.angular_impulse_magnitude
    )
    assert capture_metrics["history"]["samples"] == pytest.approx(
        float(len(controller._step_ms_history))
    )


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
    assert callable(setup.info[CAPTURE_METRICS_INFO_KEY])
    capture_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert capture_metrics["row"] == "rigid_free_flight"
    assert capture_metrics["solver"] == "sequential_impulse"
    assert capture_metrics["executor"] == controller._executors[0][0]
    assert capture_metrics["drift_position_error"] == pytest.approx(
        metrics["drift_position_error"]
    )
    assert capture_metrics["arc_position_error"] == pytest.approx(
        metrics["arc_position_error"]
    )
    assert capture_metrics["arc_momentum_residual"] == pytest.approx(
        metrics["arc_momentum_residual"]
    )
    assert capture_metrics["spin_momentum_ratio"] == pytest.approx(
        metrics["spin_momentum_ratio"]
    )
    assert capture_metrics["contact_count"] == pytest.approx(0.0)
    assert capture_metrics["history"]["samples"] == pytest.approx(
        float(len(controller._step_ms_history))
    )


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
    assert callable(setup.info[CAPTURE_METRICS_INFO_KEY])
    capture_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert capture_metrics["row"] == "rigid_frame_hierarchy"
    assert capture_metrics["solver"] == "world_frame_hierarchy"
    assert capture_metrics["executor"] == controller._executors[0][0]
    assert capture_metrics["parent"] == controller.body.name
    assert capture_metrics["world_error"] == pytest.approx(metrics["world_error"])
    assert capture_metrics["relative_error"] == pytest.approx(metrics["relative_error"])
    assert capture_metrics["orientation_error"] == pytest.approx(
        metrics["orientation_error"]
    )
    assert capture_metrics["sensor_x"] == pytest.approx(metrics["sensor_x"])
    assert capture_metrics["history"]["samples"] == pytest.approx(
        float(len(controller._step_ms_history))
    )


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
    assert callable(setup.info[CAPTURE_METRICS_INFO_KEY])
    capture_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert capture_metrics["row"] == "rigid_timestep_sensitivity"
    assert capture_metrics["comparison_axis"] == "time_step_multiplier"
    assert capture_metrics["solver"] == "Sequential impulse"
    assert capture_metrics["solver_enum"] == controller._solver().name
    assert capture_metrics["executor"] == controller._executors[0][0]
    assert capture_metrics["held_fixed"] == {
        "solver": "Sequential impulse",
        "solver_enum": controller._solver().name,
        "executor": controller._executors[0][0],
        "display_step_ms": pytest.approx(controller.base_time_step * 4.0 * 1000.0),
        "gravity_scale": pytest.approx(controller.gravity_scale),
    }
    assert set(capture_metrics["lanes"]) == set(metrics)
    assert capture_metrics["fine_freefall_error"] == pytest.approx(
        metrics["fine"]["freefall_error"]
    )
    assert capture_metrics["medium_freefall_error"] == pytest.approx(
        metrics["medium"]["freefall_error"]
    )
    assert capture_metrics["coarse_freefall_error"] == pytest.approx(
        metrics["coarse"]["freefall_error"]
    )
    assert capture_metrics["coarse_error_ratio"] == pytest.approx(
        controller._coarse_error_ratio[-1]
    )
    assert capture_metrics["history"]["samples"] == pytest.approx(
        float(len(controller._step_ms_history["fine"]))
    )


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
    assert capture_metrics["comparison_axis"] == "workload_shape"
    assert capture_metrics["solver"] == controller._solver_label()
    assert capture_metrics["executor"] == controller._executor_label()
    assert capture_metrics["held_fixed"] == {
        "solver": controller._solver_label(),
        "executor": controller._executor_label(),
        "time_step_ms": pytest.approx(4.0),
    }
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
    assert capture_metrics["comparison_axis"] == "contact_workload_size"
    assert capture_metrics["solver"] == controller._solver_label()
    assert capture_metrics["executor"] == controller._executor_label()
    assert capture_metrics["held_fixed"] == {
        "solver": controller._solver_label(),
        "executor": controller._executor_label(),
        "friction": pytest.approx(controller.friction),
        "time_step_ms": pytest.approx(4.0),
    }
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
    assert metrics["option_contact_count"] == 4
    assert metrics["active_contact_count"] == 4
    assert metrics["filtered_contact_count"] == 0
    assert metrics["option_filtered_contact_count"] == 0
    assert metrics["ignored_contact_count"] == 0
    assert metrics["ignored_pair_count"] == 0
    assert metrics["ignored_pair_key"] == "none"
    expected_body_kinds = {
        "rigid_rigid": ("rigid", "rigid"),
        "rigid_link": ("rigid", "link"),
        "same_links": ("link", "link"),
        "cross_links": ("link", "link"),
    }
    for lane in controller.lanes:
        lane_metrics = metrics["lanes"][lane.key]
        assert lane_metrics["baseline_count"] == 1
        assert lane_metrics["option_count"] == 1
        assert lane_metrics["active_count"] == 1
        assert lane_metrics["option_filtered"] is False
        assert lane_metrics["pair_ignored"] is False
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
    assert lanes["rigid_rigid"]["option_filtered"] is True
    assert lanes["rigid_rigid"]["pair_ignored"] is False
    assert lanes["rigid_link"]["active_count"] == 1
    assert lanes["same_links"]["active_count"] == 1
    assert lanes["cross_links"]["active_count"] == 1
    assert controller._last_metrics["option_filtered_contact_count"] == 1
    assert controller._last_metrics["ignored_contact_count"] == 0

    controller._apply_preset("all")
    controller.include_rigid_body_link_pairs = False
    controller._record_metrics()
    lanes = controller._last_metrics["lanes"]
    assert lanes["rigid_link"]["filtered"] is True
    assert lanes["rigid_link"]["option_filtered"] is True
    assert lanes["rigid_link"]["pair_ignored"] is False
    assert lanes["rigid_rigid"]["active_count"] == 1
    assert lanes["same_links"]["active_count"] == 1
    assert lanes["cross_links"]["active_count"] == 1

    controller._apply_preset("all")
    controller.include_link_pairs = False
    controller._record_metrics()
    lanes = controller._last_metrics["lanes"]
    assert lanes["same_links"]["filtered"] is True
    assert lanes["cross_links"]["filtered"] is True
    assert lanes["same_links"]["option_filtered"] is True
    assert lanes["cross_links"]["option_filtered"] is True
    assert lanes["rigid_rigid"]["active_count"] == 1
    assert lanes["rigid_link"]["active_count"] == 1

    controller._apply_preset("all")
    controller.include_same_multibody_link_pairs = False
    controller._record_metrics()
    lanes = controller._last_metrics["lanes"]
    assert lanes["same_links"]["filtered"] is True
    assert lanes["same_links"]["option_filtered"] is True
    assert lanes["cross_links"]["active_count"] == 1
    assert lanes["rigid_rigid"]["active_count"] == 1
    assert lanes["rigid_link"]["active_count"] == 1

    controller._apply_preset("ignore_rigid_link")
    metrics = controller._last_metrics
    lanes = metrics["lanes"]
    assert metrics["baseline_contact_count"] == 4
    assert metrics["option_contact_count"] == 4
    assert metrics["active_contact_count"] == 3
    assert metrics["option_filtered_contact_count"] == 0
    assert metrics["ignored_contact_count"] == 1
    assert metrics["ignored_pair_count"] == 1
    assert metrics["ignored_pair_key"] == "rigid_link"
    assert lanes["rigid_link"]["filtered"] is True
    assert lanes["rigid_link"]["option_filtered"] is False
    assert lanes["rigid_link"]["pair_ignored"] is True
    assert lanes["rigid_link"]["option_count"] == 1
    assert lanes["rigid_link"]["active_count"] == 0
    assert lanes["rigid_rigid"]["active_count"] == 1
    assert lanes["same_links"]["active_count"] == 1
    assert lanes["cross_links"]["active_count"] == 1
    assert controller.world.is_collision_pair_ignored(
        controller.lanes[1].body_a, controller.lanes[1].body_b
    )

    snapshot = setup.info["replay_capture_state"]()
    controller._apply_preset("clear_ignore")
    setup.info["replay_restore_state"](snapshot)
    assert controller.ignored_pair_key == "rigid_link"
    assert controller.world.num_ignored_collision_pairs == 1
    assert controller._ignored_count_history[-1] == pytest.approx(1.0)

    capture_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert capture_metrics["row"] == "rigid_collision_query_options"
    assert capture_metrics["solver"] == "collision_query"
    assert capture_metrics["ignored_pair_key"] == "rigid_link"
    assert capture_metrics["ignored_pair_count"] == 1
    assert capture_metrics["ignored_contact_count"] == 1
    assert capture_metrics["active_contact_count"] == 3
    assert capture_metrics["option_contact_count"] == 4
    assert capture_metrics["baseline_contact_count"] == 4
    assert set(capture_metrics["lanes"]) == {
        "rigid_rigid",
        "rigid_link",
        "same_links",
        "cross_links",
    }
    capture_lane = capture_metrics["lanes"]["rigid_link"]
    assert capture_lane["pair_ignored"] is True
    assert capture_lane["option_filtered"] is False
    assert capture_lane["option_count"] == 1
    assert capture_lane["active_count"] == 0


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

    assert callable(setup.info[CAPTURE_METRICS_INFO_KEY])
    capture_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert capture_metrics["row"] == "rigid_collision_casts"
    assert capture_metrics["query_scope"] == "raycast_sphere_cast_capsule_cast"
    assert capture_metrics["controls"]["enable_all_ray_hits"] is True
    assert capture_metrics["ray_hit_count"] == 2
    assert capture_metrics["ray_first_target"] == "near_sensor_target"
    assert capture_metrics["ray_first_fraction"] == pytest.approx(
        ray["first_fraction"]
    )
    assert capture_metrics["sphere_hit_count"] == 2
    assert capture_metrics["sphere_first_target"] == "near_sensor_target"
    assert capture_metrics["sphere_first_toi"] == pytest.approx(
        sphere_cast["first_toi"]
    )
    assert capture_metrics["capsule_hit_count"] == 2
    assert capture_metrics["capsule_first_target"] == "near_sensor_target"
    assert capture_metrics["capsule_first_toi"] == pytest.approx(
        capsule_cast["first_toi"]
    )
    assert capture_metrics["metrics"]["ray"]["all_targets"] == [
        "near_sensor_target",
        "far_sensor_target",
    ]
    assert capture_metrics["history"]["samples"] == pytest.approx(
        len(controller._ray_hit_history)
    )
    assert capture_metrics["history"]["max_ray_hit_count"] == pytest.approx(2.0)
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
    assert callable(setup.info[CAPTURE_METRICS_INFO_KEY])
    capture_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert capture_metrics["row"] == "rigid_solver_compare"
    assert capture_metrics["comparison_axis"] == "rigid_body_solver_family"
    assert capture_metrics["solver"] == "sequential_impulse_vs_ipc"
    assert capture_metrics["executor"] == controller._executors[0][0]
    assert capture_metrics["case_pair"] == [case.label for case in controller.cases]
    assert capture_metrics["solver_pair"] == [case.solver.name for case in controller.cases]
    assert capture_metrics["controls"]["executor_index"] == pytest.approx(
        controller.executor_index
    )
    assert set(capture_metrics["cases"]) == {case.label for case in controller.cases}
    assert (
        capture_metrics["cases"]["Sequential impulse"]["rigid_body_solver"]
        == "SEQUENTIAL_IMPULSE"
    )
    assert capture_metrics["cases"]["IPC barrier"]["rigid_body_solver"] == "IPC"
    assert capture_metrics["divergence"]["max_x"] == pytest.approx(
        max(controller._delta_history)
    )


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
    assert callable(setup.info[CAPTURE_METRICS_INFO_KEY])
    capture_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert capture_metrics["row"] == "rigid_restitution_ladder"
    assert capture_metrics["solver"] == "Sequential impulse"
    assert capture_metrics["solver_enum"] == controller._solver().name
    assert capture_metrics["executor"] == controller._executors[0][0]
    assert set(capture_metrics["lanes"]) == set(controller._last_metrics)
    assert capture_metrics["dead_rebound_height"] == pytest.approx(
        dead["max_rebound_height"]
    )
    assert capture_metrics["middle_rebound_height"] == pytest.approx(
        middle["max_rebound_height"]
    )
    assert capture_metrics["high_rebound_height"] == pytest.approx(
        high["max_rebound_height"]
    )
    assert capture_metrics["history"]["samples"] == pytest.approx(
        float(len(controller._step_ms_history))
    )


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
    assert callable(setup.info[CAPTURE_METRICS_INFO_KEY])
    capture_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert capture_metrics["row"] == "rigid_material_mixing"
    assert capture_metrics["solver"] == "sequential_impulse"
    assert capture_metrics["executor"] == controller._executors[0][0]
    assert set(capture_metrics["lanes"]) == set(metrics)
    assert capture_metrics["expected_restitution_rule"] == "max"
    assert capture_metrics["expected_friction_rule"] == "sqrt_product"
    assert capture_metrics["effective_restitution"] == pytest.approx(
        expected_restitution
    )
    assert capture_metrics["effective_friction"] == pytest.approx(expected_friction)
    assert capture_metrics["bounce_body_high_rebound"] == pytest.approx(
        metrics["bounce_body_high"]["max_rebound_height"]
    )
    assert capture_metrics["slide_body_high_speed_loss"] == pytest.approx(
        metrics["slide_body_high"]["speed_loss"]
    )
    assert capture_metrics["history"]["samples"] == pytest.approx(
        float(len(controller._step_ms_history))
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
        assert controller._drift_history[case.label]
        assert controller._goal_error_history[case.label]
        assert (
            max(controller._contact_history[case.label]) > 0.0
            or min(controller._gap_history[case.label]) < 0.025
        )

    assert callable(setup.info[CAPTURE_METRICS_INFO_KEY])
    capture_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert capture_metrics["row"] == "rigid_contact_manipulation"
    assert capture_metrics["comparison_axis"] == "rigid_pusher_contact_response"
    assert capture_metrics["solver"] == "sequential_impulse_vs_ipc"
    assert capture_metrics["executor"] == controller._executor_label()
    assert capture_metrics["held_fixed"]["executor"] == controller._executor_label()
    assert capture_metrics["held_fixed"]["target_mass"] == pytest.approx(1.0)
    assert capture_metrics["held_fixed"]["time_step_ms"] == pytest.approx(
        capture_metrics["time_step_ms"]
    )
    assert capture_metrics["controls"]["launch_speed"] == pytest.approx(
        controller.launch_speed
    )
    assert capture_metrics["controls"]["friction"] == pytest.approx(
        controller.friction
    )
    assert capture_metrics["controls"]["pusher_mass"] == pytest.approx(
        controller.pusher_mass
    )
    assert capture_metrics["case_pair"] == [case.label for case in controller.cases]
    assert capture_metrics["solver_pair"] == [
        case.solver.name for case in controller.cases
    ]
    assert capture_metrics["lane_order"] == [case.label for case in controller.cases]
    assert set(capture_metrics["cases"]) == {"Sequential impulse", "IPC barrier"}
    assert (
        capture_metrics["cases"]["Sequential impulse"]["rigid_body_solver"]
        == "SEQUENTIAL_IMPULSE"
    )
    assert capture_metrics["cases"]["IPC barrier"]["rigid_body_solver"] == "IPC"
    assert capture_metrics["sequential_impulse_status"] == str(
        controller._last_metrics["Sequential impulse"]["status"]
    )
    assert capture_metrics["ipc_status"] == str(
        controller._last_metrics["IPC barrier"]["status"]
    )
    assert capture_metrics["sequential_impulse_target_travel"] == pytest.approx(
        controller._last_metrics["Sequential impulse"]["target_travel"]
    )
    assert capture_metrics["ipc_target_travel"] == pytest.approx(
        controller._last_metrics["IPC barrier"]["target_travel"]
    )
    assert capture_metrics["ipc_gap"] == pytest.approx(
        controller._last_metrics["IPC barrier"]["gap"]
    )
    assert capture_metrics["sequential_impulse_max_contact_count"] == pytest.approx(
        max(controller._contact_history["Sequential impulse"])
    )
    assert capture_metrics["ipc_max_contact_count"] == pytest.approx(
        max(controller._contact_history["IPC barrier"])
    )
    assert capture_metrics["travel_divergence"] == pytest.approx(
        controller._divergence_history[-1]
    )
    timeline = setup.info["replay_timeline"]
    snapshot = controller.capture_replay_state()
    assert timeline["signal_label"] == "Travel divergence"
    assert timeline["signal"](snapshot) == pytest.approx(
        controller._divergence_history[-1]
    )
    assert timeline["markers"]({"divergence_history": [0.0, 0.012]}) == pytest.approx(
        1.0
    )
    assert (
        timeline["markers"](
            {
                "last_metrics": {
                    "case": {
                        "contact_count": 0.0,
                        "gap": 0.010,
                        "status": "approaching",
                        "target_travel": 0.02,
                    }
                }
            }
        )
        == pytest.approx(1.0)
    )
    assert (
        timeline["markers"]({"contact_history": {"case": [0.0, 1.0]}})
        == pytest.approx(1.0)
    )
    assert (
        timeline["markers"](
            {
                "divergence_history": [0.004],
                "last_metrics": {
                    "case": {
                        "contact_count": 0.0,
                        "gap": 0.080,
                        "status": "approaching",
                        "target_travel": 0.02,
                    }
                },
            }
        )
        == pytest.approx(0.0)
    )
    assert capture_metrics["history"]["samples"] == pytest.approx(
        len(controller._travel_history["Sequential impulse"])
    )
    assert capture_metrics["history"]["max_travel_divergence"] > 1.0e-4
    assert capture_metrics["history"]["ipc_max_target_travel"] > 0.08
    assert capture_metrics["history"]["sequential_impulse_min_gap"] < 0.025


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
        assert controller._gap_history[case.key]
        assert controller._contact_history[case.key]
        assert np.isfinite(float(metrics[case.key]["step_ms"]))

    assert callable(setup.info[CAPTURE_METRICS_INFO_KEY])
    capture_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert capture_metrics["row"] == "rigid_kinematic_driver"
    assert (
        capture_metrics["comparison_axis"]
        == "prescribed_tangential_contact_response"
    )
    assert capture_metrics["solver"] == "ipc_kinematic_driver_with_si_caveat"
    assert capture_metrics["executor"] == controller._executor_label()
    assert capture_metrics["held_fixed"]["executor"] == controller._executor_label()
    assert capture_metrics["held_fixed"]["kinematic_driver"] == "tangential support"
    assert capture_metrics["held_fixed"]["time_step_ms"] == pytest.approx(
        capture_metrics["time_step_ms"]
    )
    assert capture_metrics["controls"]["drive_speed"] == pytest.approx(
        controller.drive_speed
    )
    assert capture_metrics["controls"]["grip_friction"] == pytest.approx(
        controller.grip_friction
    )
    assert capture_metrics["case_pair"] == [case.label for case in controller.cases]
    assert capture_metrics["solver_pair"] == [
        case.solver.name for case in controller.cases
    ]
    assert capture_metrics["lane_order"] == [case.key for case in controller.cases]
    assert set(capture_metrics["lanes"]) == {"ipc_grip", "ipc_slip", "si_caveat"}
    assert capture_metrics["lanes"]["ipc_grip"]["rigid_body_solver"] == "IPC"
    assert capture_metrics["lanes"]["ipc_slip"]["friction_mode"] == "zero"
    assert (
        capture_metrics["lanes"]["si_caveat"]["rigid_body_solver"]
        == "SEQUENTIAL_IMPULSE"
    )
    assert capture_metrics["ipc_grip_box_travel"] == pytest.approx(
        grip["box_travel"]
    )
    assert capture_metrics["ipc_grip_min_abs_support_gap"] == pytest.approx(
        min(abs(value) for value in controller._gap_history["ipc_grip"])
    )
    assert capture_metrics["ipc_slip_slip"] == pytest.approx(slip["slip"])
    assert capture_metrics["si_caveat_driver_travel"] == pytest.approx(
        caveat["driver_travel"]
    )
    assert capture_metrics["history"]["samples"] == pytest.approx(
        len(controller._driver_history["ipc_grip"])
    )
    assert capture_metrics["history"]["ipc_grip_max_box_travel"] > 0.07
    assert capture_metrics["history"]["ipc_slip_max_slip"] > 0.08
    assert capture_metrics["history"]["si_caveat_max_abs_driver_travel"] < 1.0e-6
    timeline = setup.info["replay_timeline"]
    snapshot = controller.capture_replay_state()
    assert timeline["signal_label"] == "IPC grip box travel"
    assert timeline["signal"](snapshot) == pytest.approx(
        controller._box_history["ipc_grip"][-1]
    )
    assert (
        timeline["markers"]({"box_history": {"ipc_grip": [0.0, 0.041]}})
        == pytest.approx(1.0)
    )
    assert (
        timeline["markers"]({"slip_history": {"ipc_slip": [0.0, 0.081]}})
        == pytest.approx(1.0)
    )
    assert (
        timeline["markers"](
            {"last_metrics": {"ipc_grip": {"contact_count": 1.0}}}
        )
        == pytest.approx(1.0)
    )
    assert (
        timeline["markers"](
            {
                "last_metrics": {
                    "ipc_grip": {
                        "box_travel": 0.02,
                        "contact_count": 0.0,
                        "driver_travel": 0.05,
                        "status": "partial drag",
                    },
                    "si_caveat": {"status": "static-like caveat"},
                }
            }
        )
        == pytest.approx(1.0)
    )
    assert (
        timeline["markers"](
            {
                "driver_history": {"ipc_grip": [0.0, 0.05]},
                "last_metrics": {
                    "si_caveat": {"status": "static-like caveat"},
                },
            }
        )
        == pytest.approx(1.0)
    )
    assert (
        timeline["markers"](
            {
                "box_history": {"ipc_grip": [0.01]},
                "last_metrics": {
                    "ipc_grip": {
                        "box_travel": 0.01,
                        "contact_count": 0.0,
                        "driver_travel": 0.02,
                        "status": "partial drag",
                    },
                    "ipc_slip": {"slip": 0.02},
                    "si_caveat": {"status": "static-like caveat"},
                },
                "slip_history": {"ipc_slip": [0.02]},
            }
        )
        == pytest.approx(0.0)
    )


def test_rigid_kinematic_normal_push_exposes_normal_pusher_caveat() -> None:
    import numpy as np

    from examples.demos.scenes.rigid_kinematic_normal_push import build

    setup = build()
    controller = setup.info["rigid_kinematic_normal_push_controller"]
    for _ in range(96):
        assert setup.pre_step is not None
        setup.pre_step()

    metrics = controller._last_metrics
    assert {case.key for case in controller.cases} == {
        "ipc_normal",
        "ipc_heavy",
        "si_caveat",
    }

    ipc = metrics["ipc_normal"]
    heavy = metrics["ipc_heavy"]
    si = metrics["si_caveat"]

    assert float(ipc["driver_travel"]) > 0.15
    assert abs(float(ipc["target_travel"])) < 0.020
    assert float(ipc["max_depth"]) > 0.10
    assert float(ipc["contact_count"]) > 0.0
    assert ipc["status"] == "ipc penetration caveat"

    assert float(heavy["target_mass"]) > float(ipc["target_mass"])
    assert abs(float(heavy["target_travel"])) < 0.020
    assert float(heavy["max_depth"]) > 0.10
    assert heavy["status"] == "ipc penetration caveat"

    assert float(si["driver_travel"]) > 0.15
    assert float(si["target_travel"]) > 0.12
    assert abs(float(si["analytic_gap"])) < 0.010
    assert float(si["contact_count"]) > 0.0
    assert si["status"] == "pushed"
    assert float(si["target_travel"]) > float(ipc["target_travel"]) + 0.10

    capture_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert capture_metrics["row"] == "rigid_kinematic_normal_push"
    assert (
        capture_metrics["comparison_axis"]
        == "prescribed_normal_contact_response"
    )
    assert capture_metrics["solver"] == (
        "ipc_penetration_caveat_vs_sequential_impulse_push"
    )
    assert capture_metrics["held_fixed"]["executor"] == controller._executor_label()
    assert capture_metrics["held_fixed"]["kinematic_driver"] == "normal paddle"
    assert capture_metrics["held_fixed"]["time_step_ms"] == pytest.approx(
        capture_metrics["time_step_ms"]
    )
    assert capture_metrics["controls"]["push_speed"] == pytest.approx(
        controller.push_speed
    )
    assert capture_metrics["controls"]["target_mass"] == pytest.approx(
        controller.target_mass
    )
    assert capture_metrics["case_pair"] == [case.label for case in controller.cases]
    assert capture_metrics["solver_pair"] == [
        case.solver.name for case in controller.cases
    ]
    assert capture_metrics["lane_order"] == [case.key for case in controller.cases]
    assert set(capture_metrics["lanes"]) == set(metrics)
    assert capture_metrics["lanes"]["ipc_normal"]["metrics"]["status"] == (
        "ipc penetration caveat"
    )
    assert capture_metrics["ipc_normal_max_depth"] == pytest.approx(
        float(ipc["max_depth"])
    )
    assert capture_metrics["ipc_heavy_max_depth"] == pytest.approx(
        float(heavy["max_depth"])
    )
    assert capture_metrics["si_caveat_target_travel"] == pytest.approx(
        float(si["target_travel"])
    )
    assert capture_metrics["target_travel_divergence"] == pytest.approx(
        abs(float(si["target_travel"]) - float(ipc["target_travel"]))
    )
    assert np.isfinite(
        float(capture_metrics["lanes"]["si_caveat"]["metrics"]["step_ms"])
    )
    assert controller._target_history["ipc_normal"]
    assert controller._target_history["si_caveat"]
    assert controller._depth_history["ipc_normal"]
    timeline = setup.info["replay_timeline"]
    snapshot = controller.capture_replay_state()
    assert timeline["signal_label"] == "Target travel divergence"
    assert timeline["signal"](snapshot) == pytest.approx(
        abs(
            controller._target_history["si_caveat"][-1]
            - controller._target_history["ipc_normal"][-1]
        )
    )
    assert (
        timeline["markers"](
            {
                "target_history": {
                    "ipc_normal": [0.0, 0.01],
                    "si_caveat": [0.0, 0.071],
                }
            }
        )
        == pytest.approx(1.0)
    )
    assert (
        timeline["markers"]({"depth_history": {"ipc_normal": [0.0, 0.051]}})
        == pytest.approx(1.0)
    )
    assert (
        timeline["markers"]({"contact_history": {"si_caveat": [0.0, 1.0]}})
        == pytest.approx(1.0)
    )
    assert (
        timeline["markers"](
            {"last_metrics": {"ipc_normal": {"status": "ipc penetration caveat"}}}
        )
        == pytest.approx(1.0)
    )
    assert (
        timeline["markers"](
            {"last_metrics": {"si_caveat": {"target_travel": 0.081}}}
        )
        == pytest.approx(1.0)
    )
    assert (
        timeline["markers"](
            {
                "target_history": {
                    "ipc_normal": [0.01],
                    "si_caveat": [0.04],
                },
                "last_metrics": {
                    "ipc_normal": {
                        "contact_count": 0.0,
                        "max_depth": 0.02,
                        "status": "partial push",
                    },
                    "si_caveat": {
                        "contact_count": 0.0,
                        "status": "partial push",
                        "target_travel": 0.04,
                    },
                },
            }
        )
        == pytest.approx(0.0)
    )


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
    assert callable(setup.info[CAPTURE_METRICS_INFO_KEY])
    capture_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert capture_metrics["row"] == "rigid_contact_solver_compare"
    assert capture_metrics["comparison_axis"] == "contact_solver_method"
    assert capture_metrics["solver"] == "sequential_impulse_rigid_body"
    assert capture_metrics["contact_policy"] == "sequential_impulse_vs_boxed_lcp"
    assert capture_metrics["case_pair"] == [case.label for case in controller.cases]
    assert capture_metrics["contact_policy_pair"] == [
        case.method.name for case in controller.cases
    ]
    assert capture_metrics["controls"]["executor_index"] == pytest.approx(
        controller.executor_index
    )
    assert set(capture_metrics["cases"]) == {case.label for case in controller.cases}
    assert (
        capture_metrics["cases"]["Sequential impulse contacts"][
            "contact_solver_method"
        ]
        == "SEQUENTIAL_IMPULSE"
    )
    assert (
        capture_metrics["cases"]["Boxed LCP contacts"]["contact_solver_method"]
        == "BOXED_LCP"
    )
    assert capture_metrics["divergence"]["max_pose"] == pytest.approx(
        max(controller._divergence_history)
    )


def test_rigid_link_contact_exercises_multibody_contact_response() -> None:
    from examples.demos.scenes.contact import build

    setup = build()
    controller = setup.info["rigid_link_contact_controller"]
    for _ in range(120):
        assert setup.pre_step is not None
        setup.pre_step()

    mid_metrics = dict(controller._last_metrics)
    assert max(controller._push_contact_history) >= 1.0
    assert callable(setup.info[CAPTURE_METRICS_INFO_KEY])
    mid_capture_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert mid_capture_metrics["row"] == "contact"
    assert mid_capture_metrics["solver"] == "sequential_impulse_rigid_body"
    assert mid_capture_metrics["contact_scope"] == "multibody_link_contact"
    assert mid_capture_metrics["rigid_body_solver"] == "SEQUENTIAL_IMPULSE"
    assert set(mid_capture_metrics["lanes"]) == {"drop", "slide", "push"}
    assert mid_capture_metrics["lanes"]["push"][
        "target_travel"
    ] == pytest.approx(mid_metrics["push_target_travel"])
    assert mid_capture_metrics["history"]["push_max_contacts"] >= 1.0

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
    capture_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert capture_metrics["executor"] == controller._executor_label()
    assert capture_metrics["controls"]["executor_index"] == pytest.approx(
        float(controller.executor_index)
    )
    assert capture_metrics["controls"]["ground_friction"] == pytest.approx(
        controller.ground_friction
    )
    assert capture_metrics["controls"]["ground_restitution"] == pytest.approx(
        controller.ground_restitution
    )
    assert capture_metrics["lanes"]["drop"]["contact_count"] == pytest.approx(
        metrics["drop_contact_count"]
    )
    assert capture_metrics["lanes"]["slide"]["travel"] == pytest.approx(
        metrics["slide_travel"]
    )
    assert capture_metrics["lanes"]["push"]["target_travel"] == pytest.approx(
        metrics["push_target_travel"]
    )
    assert capture_metrics["metrics"]["contact_body_kinds"] == metrics[
        "contact_body_kinds"
    ]
    assert capture_metrics["history"]["samples"] == pytest.approx(
        len(controller._step_ms_history)
    )
    assert capture_metrics["history"]["drop_max_contacts"] >= 1.0
    assert capture_metrics["history"]["slide_max_contacts"] >= 1.0
    assert capture_metrics["history"]["push_max_target_travel"] > 0.05


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
    from examples.demos.scenes.rigid_distance_spring import (
        build as distance_spring_build,
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
    from examples.demos.scenes.rigid_kinematic_normal_push import (
        build as normal_push_build,
    )
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

    normal_push = normal_push_build().info["rigid_kinematic_normal_push_controller"]
    normal_push.executor_index = len(normal_push._executors) - 1
    normal_push.push_speed = 0.52
    normal_push.target_mass = 2.30
    normal_push_state = normal_push.capture_replay_state()
    normal_push.executor_index = 0
    normal_push.push_speed = 0.18
    normal_push.target_mass = 0.40
    normal_push.restore_replay_state(normal_push_state)
    assert normal_push.executor_index == len(normal_push._executors) - 1
    assert normal_push.push_speed == pytest.approx(0.52)
    assert normal_push.target_mass == pytest.approx(2.30)
    for case in normal_push.cases:
        expected_mass = 2.30 * case.target_mass_scale
        assert case.target.mass == pytest.approx(expected_mass)

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

    distance_spring = distance_spring_build().info[
        "rigid_distance_spring_controller"
    ]
    distance_spring.executor_index = len(distance_spring._executors) - 1
    distance_spring.initial_stretch = 0.41
    distance_spring.gravity_scale = 0.35
    distance_spring.rest_length = 0.58
    distance_spring.soft_stiffness = 63.0
    distance_spring.stiff_stiffness = 245.0
    distance_spring.offset_stiffness = 145.0
    distance_spring._apply_spring_parameters()
    distance_spring_state = distance_spring.capture_replay_state()
    distance_spring.executor_index = 0
    distance_spring.initial_stretch = 0.10
    distance_spring.gravity_scale = 0.0
    distance_spring.rest_length = 0.30
    distance_spring.soft_stiffness = 20.0
    distance_spring.stiff_stiffness = 90.0
    distance_spring.offset_stiffness = 50.0
    distance_spring.restore_replay_state(distance_spring_state)
    assert distance_spring.executor_index == len(distance_spring._executors) - 1
    assert distance_spring.initial_stretch == pytest.approx(0.41)
    assert distance_spring.gravity_scale == pytest.approx(0.35)
    assert distance_spring.rest_length == pytest.approx(0.58)
    assert distance_spring.soft_stiffness == pytest.approx(63.0)
    assert distance_spring.stiff_stiffness == pytest.approx(245.0)
    assert distance_spring.offset_stiffness == pytest.approx(145.0)
    assert distance_spring.world.gravity[2] == pytest.approx(-4.0 * 0.35)
    assert distance_spring.world.get_rigid_body_distance_spring_parameters(
        "soft_distance_spring"
    ) == pytest.approx((0.58, 63.0))
    assert distance_spring.world.get_rigid_body_distance_spring_parameters(
        "stiff_distance_spring"
    ) == pytest.approx((0.58, 245.0))
    assert distance_spring.world.get_rigid_body_distance_spring_parameters(
        "offset_distance_spring"
    ) == pytest.approx((0.58, 145.0))
    assert set(distance_spring._last_metrics) == {
        "free",
        "soft",
        "stiff",
        "offset",
    }

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
    passive_joint.hold_force = 4.5
    passive_joint.slip_force = 11.0
    passive_joint.armature_force = 9.5
    passive_joint.armature = 8.0
    passive_joint_state = passive_joint.capture_replay_state()
    passive_joint.executor_index = 0
    passive_joint.spring_stiffness = 5.0
    passive_joint.damping_coefficient = 0.5
    passive_joint.rest_position = -0.10
    passive_joint.coulomb_friction = 2.0
    passive_joint.hold_force = 1.0
    passive_joint.slip_force = 3.0
    passive_joint.armature_force = 2.0
    passive_joint.armature = 1.0
    passive_joint.restore_replay_state(passive_joint_state)
    assert passive_joint.executor_index == len(passive_joint._executors) - 1
    assert passive_joint.spring_stiffness == pytest.approx(21.0)
    assert passive_joint.damping_coefficient == pytest.approx(4.5)
    assert passive_joint.rest_position == pytest.approx(0.12)
    assert passive_joint.coulomb_friction == pytest.approx(7.5)
    assert passive_joint.hold_force == pytest.approx(4.5)
    assert passive_joint.slip_force == pytest.approx(11.0)
    assert passive_joint.armature_force == pytest.approx(9.5)
    assert passive_joint.armature == pytest.approx(8.0)
    damped = next(
        lane for lane in passive_joint.lanes if lane.key == "spring_damper"
    )
    stiction = next(lane for lane in passive_joint.lanes if lane.key == "stiction")
    armature_reference = next(
        lane for lane in passive_joint.lanes if lane.key == "armature_reference"
    )
    armature = next(
        lane for lane in passive_joint.lanes if lane.key == "armature_heavy"
    )
    assert damped.joint.spring_stiffness.tolist() == pytest.approx([21.0])
    assert damped.joint.damping_coefficient.tolist() == pytest.approx([4.5])
    assert damped.joint.rest_position.tolist() == pytest.approx([0.12])
    assert stiction.joint.coulomb_friction.tolist() == pytest.approx([7.5])
    assert stiction.joint.force.tolist() == pytest.approx([4.5])
    assert armature_reference.joint.force.tolist() == pytest.approx([9.5])
    assert armature.joint.force.tolist() == pytest.approx([9.5])
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

    assert callable(setup.info[CAPTURE_METRICS_INFO_KEY])
    capture_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert capture_metrics["row"] == "rigid_friction_threshold"
    assert capture_metrics["comparison_axis"] == "friction_threshold_lane"
    assert capture_metrics["solver"] == "ipc"
    assert capture_metrics["solver_enum"] == "IPC"
    assert capture_metrics["held_fixed"]["solver"] == "IPC"
    assert capture_metrics["held_fixed"]["executor"] == controller._executor_label()
    assert capture_metrics["controls"]["angle_deg"] == pytest.approx(
        controller.angle_deg
    )
    assert capture_metrics["controls"]["threshold_mu"] == pytest.approx(
        controller._threshold()
    )
    assert capture_metrics["below_distance"] == pytest.approx(
        metrics["below"]["distance"]
    )
    assert capture_metrics["below_speed"] == pytest.approx(metrics["below"]["speed"])
    assert capture_metrics["above_distance"] == pytest.approx(
        metrics["above"]["distance"]
    )
    assert capture_metrics["above_speed"] == pytest.approx(metrics["above"]["speed"])
    assert set(capture_metrics["lanes"]) == {"below", "controlled", "above"}
    assert capture_metrics["lanes"]["controlled"]["metrics"]["friction"] == pytest.approx(
        controller.controlled_mu
    )
    assert capture_metrics["history"]["samples"] == pytest.approx(
        len(controller._distance_history["below"])
    )
    assert capture_metrics["history"]["below_max_distance"] > 0.5
    assert capture_metrics["history"]["above_max_abs_distance"] < 0.05


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

    assert callable(setup.info[CAPTURE_METRICS_INFO_KEY])
    capture_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert capture_metrics["row"] == "rigid_spin_roll_coupling"
    assert capture_metrics["comparison_axis"] == "spin_roll_initial_condition"
    assert capture_metrics["solver"] == "sequential_impulse"
    assert capture_metrics["solver_enum"] == "SEQUENTIAL_IMPULSE"
    assert capture_metrics["held_fixed"]["solver"] == "Sequential impulse"
    assert capture_metrics["held_fixed"]["executor"] == controller._executor_label()
    assert capture_metrics["controls"]["friction"] == pytest.approx(
        controller.friction
    )
    assert capture_metrics["controls"]["launch_speed"] == pytest.approx(
        controller.launch_speed
    )
    assert capture_metrics["contact_count"] == pytest.approx(
        controller._last_contact_count
    )
    assert set(capture_metrics["lanes"]) == {
        "matched_roll",
        "slide_to_roll",
        "backspin_scrub",
        "low_friction_slip",
    }
    assert capture_metrics["matched_roll_contact_slip"] == pytest.approx(
        matched["contact_slip"]
    )
    assert capture_metrics["slide_to_roll_spin_delta"] == pytest.approx(
        slide["spin_delta"]
    )
    assert capture_metrics["backspin_scrub_spin_delta"] == pytest.approx(
        backspin["spin_delta"]
    )
    assert capture_metrics["low_friction_slip_contact_slip"] == pytest.approx(
        low_friction["contact_slip"]
    )
    assert capture_metrics["low_friction_slip_friction"] == pytest.approx(0.0)
    assert capture_metrics["history"]["samples"] == pytest.approx(
        len(controller._step_ms_history)
    )
    assert capture_metrics["history"]["low_friction_slip_max_slip"] > 0.5


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
    timeline = setup.info["replay_timeline"]
    snapshot = controller.capture_replay_state()
    assert timeline["signal_label"] == "Pose divergence"
    assert timeline["signal"](snapshot) == pytest.approx(
        controller._position_divergence[-1]
    )
    assert (
        timeline["markers"]({"position_divergence": [0.0], "contact_delta": [0.0]})
        == pytest.approx(0.0)
    )
    assert (
        timeline["markers"]({"position_divergence": [2.0e-8], "contact_delta": [0.0]})
        == pytest.approx(1.0)
    )
    assert (
        timeline["markers"](
            {"velocity_divergence": [2.0e-8], "contact_delta": [0.0]}
        )
        == pytest.approx(1.0)
    )
    assert (
        timeline["markers"]({"position_divergence": [0.0], "contact_delta": [1.0]})
        == pytest.approx(1.0)
    )
    for metrics in controller._last_metrics.values():
        assert float(metrics["contact_count"]) >= 0.0
        assert float(metrics["step_ms"]) >= 0.0

    assert callable(setup.info[CAPTURE_METRICS_INFO_KEY])
    capture_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert capture_metrics["row"] == "rigid_executor_equivalence"
    assert capture_metrics["comparison_axis"] == "executor"
    assert capture_metrics["same_solver"] is True
    assert capture_metrics["solver"] == "same_solver_executor_equivalence"
    assert capture_metrics["solver_enum"] == controller._solver().name
    assert capture_metrics["controls"]["solver_index"] == pytest.approx(
        controller.solver_index
    )
    assert capture_metrics["executor_pair"] == [case.label for case in controller.cases]
    assert capture_metrics["sequential_contact_count"] == pytest.approx(
        controller._last_metrics[controller.cases[0].label]["contact_count"]
    )
    assert capture_metrics["parallel_contact_count"] == pytest.approx(
        controller._last_metrics[controller.cases[1].label]["contact_count"]
    )
    assert capture_metrics["position_divergence"] == pytest.approx(
        controller._position_divergence[-1]
    )
    assert capture_metrics["velocity_divergence"] == pytest.approx(
        controller._velocity_divergence[-1]
    )
    assert capture_metrics["contact_delta"] == pytest.approx(
        controller._contact_delta[-1]
    )
    assert capture_metrics["history"]["samples"] == pytest.approx(
        len(controller._position_divergence)
    )
    assert capture_metrics["history"]["max_position_divergence"] < 1.0e-9


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
    timeline = setup.info["replay_timeline"]
    snapshot = controller.capture_replay_state()
    assert timeline["signal_label"] == "Top x divergence"
    assert timeline["signal"](snapshot) == pytest.approx(
        controller._delta_history[-1]
    )
    assert timeline["markers"]({"delta_history": [0.0, 0.011]}) == pytest.approx(
        1.0
    )
    assert (
        timeline["markers"](
            {
                "last_metrics": {
                    "case": {"min_clearance": 0.0005, "status": "settling"}
                }
            }
        )
        == pytest.approx(1.0)
    )
    assert (
        timeline["markers"](
            {"last_metrics": {"case": {"top_drift": 0.021, "status": "settling"}}}
        )
        == pytest.approx(1.0)
    )
    assert (
        timeline["markers"]({"last_metrics": {"case": {"status": "collapsed"}}})
        == pytest.approx(1.0)
    )
    assert (
        timeline["markers"](
            {
                "delta_history": [0.004],
                "last_metrics": {
                    "case": {
                        "min_clearance": 0.02,
                        "status": "stable",
                        "top_drift": 0.004,
                    }
                },
            }
        )
        == pytest.approx(0.0)
    )

    metrics = controller._last_metrics["IPC barrier"]
    assert float(metrics["min_clearance"]) > -0.002
    assert float(metrics["top_drift"]) < 0.03
    assert float(metrics["max_speed"]) < 0.02

    assert callable(setup.info[CAPTURE_METRICS_INFO_KEY])
    capture_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert capture_metrics["row"] == "rigid_stack_stability"
    assert capture_metrics["comparison_axis"] == "rigid_body_solver_family"
    assert capture_metrics["solver"] == "sequential_impulse_vs_ipc"
    assert capture_metrics["executor"] == controller._executors[0][0]
    assert capture_metrics["held_fixed"]["executor"] == controller._executor_label()
    assert capture_metrics["held_fixed"]["top_mass_ratio"] == pytest.approx(
        controller.top_mass_ratio
    )
    assert capture_metrics["held_fixed"]["time_step_ms"] == pytest.approx(
        capture_metrics["time_step_ms"]
    )
    assert capture_metrics["controls"]["friction"] == pytest.approx(
        controller.friction
    )
    assert capture_metrics["controls"]["top_mass_ratio"] == pytest.approx(
        controller.top_mass_ratio
    )
    assert capture_metrics["case_pair"] == [case.label for case in controller.cases]
    assert capture_metrics["solver_pair"] == [
        case.solver.name for case in controller.cases
    ]
    assert set(capture_metrics["cases"]) == {"Sequential impulse", "IPC barrier"}
    assert (
        capture_metrics["cases"]["Sequential impulse"]["rigid_body_solver"]
        == "SEQUENTIAL_IMPULSE"
    )
    assert capture_metrics["cases"]["IPC barrier"]["rigid_body_solver"] == "IPC"
    assert capture_metrics["sequential_impulse_max_speed"] == pytest.approx(
        controller._last_metrics["Sequential impulse"]["max_speed"]
    )
    assert capture_metrics["ipc_max_speed"] == pytest.approx(
        controller._last_metrics["IPC barrier"]["max_speed"]
    )
    assert capture_metrics["ipc_min_clearance"] == pytest.approx(
        controller._last_metrics["IPC barrier"]["min_clearance"]
    )
    assert capture_metrics["top_x_divergence"] == pytest.approx(
        controller._delta_history[-1]
    )
    assert capture_metrics["history"]["samples"] == pytest.approx(
        len(controller._speed_history["Sequential impulse"])
    )
    assert capture_metrics["history"]["ipc_max_speed"] < 0.02
    assert capture_metrics["history"]["ipc_min_clearance"] > -0.002
    assert capture_metrics["history"]["max_top_x_divergence"] >= 0.0


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
    assert callable(setup.info[CAPTURE_METRICS_INFO_KEY])
    capture_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert capture_metrics["row"] == "rigid_fixed_joint"
    assert capture_metrics["comparison_axis"] == "fixed_relative_transform_recovery"
    assert capture_metrics["solver"] == "sequential_rigid_joints"
    assert capture_metrics["constraint"] == "fixed_relative_transform"
    assert capture_metrics["joint_name"] == controller.fixed_joint.name
    assert capture_metrics["fixed_joint_count"] == pytest.approx(1.0)
    assert capture_metrics["held_fixed"] == {
        "base": "static",
        "captured_offset_m": pytest.approx(0.85),
        "gravity_z": pytest.approx(-9.81),
        "payload_mass": pytest.approx(1.0),
        "solver": "Sequential rigid joints",
        "time_step_ms": pytest.approx(capture_metrics["time_step_ms"]),
    }
    assert capture_metrics["fixed_joint_translation_error"] == pytest.approx(
        float(metrics["translation_error"])
    )
    assert capture_metrics["fixed_joint_orientation_error"] == pytest.approx(
        float(metrics["orientation_error"])
    )
    assert capture_metrics["fixed_joint_payload_speed"] == pytest.approx(
        float(metrics["payload_speed"])
    )
    assert capture_metrics["fixed_joint_payload_angular_speed"] == pytest.approx(
        float(metrics["payload_angular_speed"])
    )
    assert capture_metrics["metrics"]["translation_error"] == pytest.approx(
        float(metrics["translation_error"])
    )
    assert capture_metrics["metrics"]["orientation_error"] == pytest.approx(
        float(metrics["orientation_error"])
    )
    assert capture_metrics["history"]["samples"] > 1.0
    assert controller._translation_error_history
    assert controller._orientation_error_history
    timeline = setup.info["replay_timeline"]
    snapshot = controller.capture_replay_state()
    assert timeline["signal_label"] == "Fixed-joint offset error"
    assert timeline["signal"](snapshot) == pytest.approx(
        controller._translation_error_history[-1]
    )
    assert (
        timeline["markers"]({"translation_error_history": [0.0, 0.011]})
        == pytest.approx(1.0)
    )
    assert (
        timeline["markers"]({"orientation_error_history": [0.0, 0.021]})
        == pytest.approx(1.0)
    )
    assert (
        timeline["markers"]({"speed_history": [0.0, 0.051]})
        == pytest.approx(1.0)
    )
    assert (
        timeline["markers"]({"last_metrics": {"payload_angular_speed": 0.051}})
        == pytest.approx(1.0)
    )
    assert (
        timeline["markers"](
            {
                "translation_error_history": [0.005],
                "orientation_error_history": [0.005],
                "speed_history": [0.010],
                "angular_speed_history": [0.010],
                "last_metrics": {
                    "translation_error": 0.005,
                    "orientation_error": 0.005,
                    "payload_speed": 0.010,
                    "payload_angular_speed": 0.010,
                },
            }
        )
        == pytest.approx(0.0)
    )


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
    assert callable(setup.info[CAPTURE_METRICS_INFO_KEY])
    capture_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert capture_metrics["row"] == "rigid_joint_breakage"
    assert capture_metrics["comparison_axis"] == "fixed_break_force_lifecycle"
    assert capture_metrics["solver"] == "avbd_rigid_joints"
    assert capture_metrics["constraint"] == "fixed_break_force_lifecycle"
    assert capture_metrics["joint_name"] == joint.name
    assert capture_metrics["held_fixed"] == {
        "base": "static",
        "captured_offset_m": pytest.approx(0.62),
        "ground_friction": pytest.approx(0.6),
        "payload_mass": pytest.approx(1.0),
        "solver": "AVBD rigid joints",
        "time_step_ms": pytest.approx(capture_metrics["time_step_ms"]),
    }
    assert capture_metrics["breakage_payload_release_distance"] == pytest.approx(
        capture_metrics["metrics"]["payload_release_distance"]
    )
    assert capture_metrics["breakage_broken"] == pytest.approx(1.0)
    assert capture_metrics["breakage_captured_offset_error"] == pytest.approx(
        capture_metrics["metrics"]["captured_offset_error"]
    )
    assert capture_metrics["breakage_payload_speed"] == pytest.approx(
        capture_metrics["metrics"]["payload_speed"]
    )
    assert capture_metrics["breakage_status"] == "broken"
    assert capture_metrics["metrics"]["broken"] == pytest.approx(1.0)
    assert capture_metrics["controls"]["break_force"] == pytest.approx(break_force)
    assert capture_metrics["controls"]["break_force_log10"] == pytest.approx(-12.0)
    assert capture_metrics["controls"]["break_force_log10_range"] == [
        pytest.approx(-12.0),
        pytest.approx(12.0),
    ]
    assert capture_metrics["controls"]["reset_break_force"] == pytest.approx(
        float(setup.info["reset_break_force"])
    )
    assert capture_metrics["metrics"]["break_force_log10"] == pytest.approx(-12.0)
    assert capture_metrics["metrics"]["status"] == "broken"
    assert capture_metrics["metrics"]["payload_release_distance"] > 1.0e-2
    assert capture_metrics["history"]["saw_broken"] == pytest.approx(1.0)
    assert capture_metrics["history"]["max_payload_release_distance"] > 1.0e-2

    assert callable(setup.info["replay_capture_state"])
    assert callable(setup.info["replay_restore_state"])
    timeline = setup.info["replay_timeline"]
    snapshot = setup.info["replay_capture_state"]()
    assert timeline["signal_label"] == "Payload release distance"
    assert timeline["signal"](snapshot) == pytest.approx(
        capture_metrics["metrics"]["payload_release_distance"]
    )
    assert (
        timeline["markers"]({"broken_history": [0.0, 1.0]})
        == pytest.approx(1.0)
    )
    assert (
        timeline["markers"]({"release_history": [0.0, 0.011]})
        == pytest.approx(1.0)
    )
    assert (
        timeline["markers"](
            {
                "last_metrics": {
                    "status": "broken",
                    "payload_release_distance": 0.0,
                    "broken": 1.0,
                }
            }
        )
        == pytest.approx(1.0)
    )
    assert (
        timeline["markers"](
            {
                "release_history": [0.002],
                "broken_history": [0.0],
                "last_metrics": {
                    "status": "intact",
                    "payload_release_distance": 0.002,
                    "broken": 0.0,
                },
            }
        )
        == pytest.approx(0.0)
    )

    setup.info["reset_breakage_lifecycle"]()
    assert not joint.is_broken
    reset_payload = np.asarray(payload.translation, dtype=float).reshape(3)
    assert np.linalg.norm(reset_payload - initial_payload) < 1.0e-9
    assert sx_world.time == pytest.approx(0.0)
    reset_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert reset_metrics["metrics"]["status"] == "intact"
    assert reset_metrics["history"]["samples"] == pytest.approx(1.0)
    reset_snapshot = setup.info["replay_capture_state"]()
    assert timeline["signal"](reset_snapshot) == pytest.approx(0.0)
    assert timeline["markers"](reset_snapshot) == pytest.approx(0.0)

    setup.info["set_break_force_log10"](2.0)
    setup.info["reset_joint"]()
    assert joint.break_force == pytest.approx(100.0)
    threshold_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert threshold_metrics["controls"]["break_force"] == pytest.approx(100.0)
    assert threshold_metrics["controls"]["break_force_log10"] == pytest.approx(2.0)
    threshold_snapshot = setup.info["replay_capture_state"]()
    setup.info["set_break_force_log10"](-12.0)
    assert joint.break_force == pytest.approx(break_force)
    setup.info["replay_restore_state"](threshold_snapshot)
    assert joint.break_force == pytest.approx(100.0)
    restored_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert restored_metrics["controls"]["break_force_log10"] == pytest.approx(2.0)


def test_rigid_distance_spring_reduces_stretch_and_spins_offset_anchor() -> None:
    from examples.demos.scenes.rigid_distance_spring import build

    setup = build()
    controller = setup.info["rigid_distance_spring_controller"]
    assert callable(setup.info[CAPTURE_METRICS_INFO_KEY])
    for _ in range(60):
        assert setup.pre_step is not None
        setup.pre_step()

    metrics = controller._last_metrics
    assert set(metrics) == {"free", "soft", "stiff", "offset"}
    free_stretch = abs(float(metrics["free"]["stretch"]))
    assert free_stretch == pytest.approx(controller.initial_stretch)
    assert abs(float(metrics["soft"]["stretch"])) < free_stretch * 0.35
    assert abs(float(metrics["stiff"]["stretch"])) < free_stretch
    assert abs(float(metrics["offset"]["stretch"])) < free_stretch * 0.35
    assert float(metrics["offset"]["angular_speed"]) > 1.0
    assert metrics["offset"]["status"] == "off-center torque"

    capture_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert capture_metrics["row"] == "rigid_distance_spring"
    assert capture_metrics["comparison_axis"] == "distance_spring_response_family"
    assert capture_metrics["solver"] == "sequential_impulse_avbd_distance_spring"
    assert capture_metrics["executor"] == controller._executor_label()
    assert capture_metrics["held_fixed"] == {
        "executor": controller._executor_label(),
        "payload_mass": pytest.approx(1.0),
        "rest_length_m": pytest.approx(controller.rest_length),
        "solver": "Sequential impulse + AVBD distance springs",
        "time_step_ms": pytest.approx(capture_metrics["time_step_ms"]),
    }
    assert capture_metrics["controls"]["initial_stretch"] == pytest.approx(
        controller.initial_stretch
    )
    assert capture_metrics["controls"]["gravity_scale"] == pytest.approx(
        controller.gravity_scale
    )
    assert capture_metrics["controls"]["rest_length"] == pytest.approx(
        controller.rest_length
    )
    assert capture_metrics["controls"]["soft_stiffness"] == pytest.approx(
        controller.soft_stiffness
    )
    assert capture_metrics["controls"]["stiff_stiffness"] == pytest.approx(
        controller.stiff_stiffness
    )
    assert capture_metrics["controls"]["offset_stiffness"] == pytest.approx(
        controller.offset_stiffness
    )
    assert capture_metrics["lane_order"] == [lane.key for lane in controller.lanes]
    assert capture_metrics["spring_lanes"] == [
        lane.label for lane in controller.lanes
    ]
    assert set(capture_metrics["lanes"]) == set(metrics)
    assert capture_metrics["distance_spring_free_abs_stretch"] == pytest.approx(
        abs(float(metrics["free"]["stretch"]))
    )
    assert capture_metrics["distance_spring_soft_abs_stretch"] == pytest.approx(
        abs(float(metrics["soft"]["stretch"]))
    )
    assert capture_metrics["distance_spring_stiff_abs_stretch"] == pytest.approx(
        abs(float(metrics["stiff"]["stretch"]))
    )
    assert capture_metrics["distance_spring_offset_abs_stretch"] == pytest.approx(
        abs(float(metrics["offset"]["stretch"]))
    )
    assert capture_metrics["distance_spring_offset_angular_speed"] == pytest.approx(
        float(metrics["offset"]["angular_speed"])
    )
    assert capture_metrics["lanes"]["free"]["has_spring"] == pytest.approx(0.0)
    assert capture_metrics["lanes"]["soft"]["has_spring"] == pytest.approx(1.0)
    assert capture_metrics["history"]["samples"] == pytest.approx(
        len(controller._step_ms_history)
    )
    assert capture_metrics["history"]["max_sprung_abs_stretch"] >= abs(
        float(metrics["soft"]["stretch"])
    )
    assert capture_metrics["distance_spring_max_sprung_abs_stretch"] == pytest.approx(
        capture_metrics["history"]["max_sprung_abs_stretch"]
    )
    assert capture_metrics["history"]["max_offset_angular_speed"] > 1.0

    assert controller._length_history["soft"]
    assert controller._stretch_history["stiff"]
    assert controller._angular_speed_history["offset"]
    assert all(value >= 0.0 for value in controller._step_ms_history)
    timeline = setup.info["replay_timeline"]
    snapshot = controller.capture_replay_state()
    latest_sprung_stretch = max(
        abs(controller._stretch_history[key][-1])
        for key in ("soft", "stiff", "offset")
    )
    assert timeline["signal_label"] == "Max spring stretch"
    assert timeline["signal"](snapshot) == pytest.approx(latest_sprung_stretch)
    assert (
        timeline["markers"]({"stretch_history": {"soft": [0.0, 0.26]}})
        == pytest.approx(1.0)
    )
    assert (
        timeline["markers"]({"angular_speed_history": {"offset": [0.0, 1.1]}})
        == pytest.approx(1.0)
    )
    assert (
        timeline["markers"](
            {
                "stretch_history": {
                    "soft": [0.03],
                    "stiff": [0.04],
                    "offset": [0.02],
                },
                "angular_speed_history": {"offset": [0.2]},
                "last_metrics": {
                    "offset": {
                        "stretch": 0.02,
                        "angular_speed": 0.2,
                    }
                },
            }
        )
        == pytest.approx(0.0)
    )


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
    assert callable(setup.info[CAPTURE_METRICS_INFO_KEY])
    capture_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert capture_metrics["row"] == "rigid_limited_joints"
    assert capture_metrics["comparison_axis"] == "one_dof_joint_constraint_family"
    assert capture_metrics["solver"] == "sequential_rigid_joints"
    assert capture_metrics["constraint"] == "revolute_prismatic_one_dof"
    assert capture_metrics["held_fixed"] == {
        "base": "static",
        "joint_axes": "z-axis revolute and prismatic",
        "payload_mass": pytest.approx(1.0),
        "solver": "Sequential rigid joints",
        "time_step_ms": pytest.approx(capture_metrics["time_step_ms"]),
    }
    assert capture_metrics["controls"]["perturbation"] == pytest.approx(
        controller.perturbation
    )
    assert capture_metrics["joint_lanes"] == ["hinge", "slider"]
    assert capture_metrics["joint_count"] == pytest.approx(2.0)
    assert capture_metrics["joints"]["hinge"] == controller.hinge_joint.name
    assert capture_metrics["joints"]["slider"] == controller.slider_joint.name
    assert capture_metrics["metrics"]["hinge_radius_error"] == pytest.approx(
        float(metrics["hinge_radius_error"])
    )
    assert capture_metrics["metrics"]["slider_orthogonal_error"] == pytest.approx(
        float(metrics["slider_orthogonal_error"])
    )
    assert capture_metrics["one_dof_hinge_radius_error"] == pytest.approx(
        float(metrics["hinge_radius_error"])
    )
    assert capture_metrics["one_dof_hinge_z_error"] == pytest.approx(
        float(metrics["hinge_z_error"])
    )
    assert capture_metrics["one_dof_slider_orthogonal_error"] == pytest.approx(
        float(metrics["slider_orthogonal_error"])
    )
    assert capture_metrics["one_dof_hinge_yaw"] == pytest.approx(
        float(metrics["hinge_yaw"])
    )
    assert capture_metrics["one_dof_slider_axis_travel"] == pytest.approx(
        float(metrics["slider_axis_travel"])
    )
    assert capture_metrics["one_dof_hinge_angular_speed"] == pytest.approx(
        float(metrics["hinge_angular_speed"])
    )
    assert capture_metrics["one_dof_slider_axis_speed"] == pytest.approx(
        float(metrics["slider_axis_speed"])
    )
    assert capture_metrics["history"]["samples"] > 1.0
    assert capture_metrics["history"]["max_abs_hinge_yaw"] >= abs(
        float(metrics["hinge_yaw"])
    )
    assert capture_metrics["history"]["max_slider_axis_travel"] >= float(
        metrics["slider_axis_travel"]
    )
    assert controller._hinge_radius_error_history
    assert controller._slider_orthogonal_error_history
    timeline = setup.info["replay_timeline"]
    snapshot = controller.capture_replay_state()
    latest_locked_error = max(
        abs(controller._hinge_radius_error_history[-1]),
        abs(controller._hinge_z_error_history[-1]),
        abs(controller._slider_orthogonal_error_history[-1]),
    )
    assert timeline["signal_label"] == "Locked-axis error"
    assert timeline["signal"](snapshot) == pytest.approx(latest_locked_error)
    assert (
        timeline["markers"]({"hinge_radius_error_history": [0.0, 0.02]})
        == pytest.approx(1.0)
    )
    assert (
        timeline["markers"]({"hinge_yaw_history": [0.0, 0.12]})
        == pytest.approx(1.0)
    )
    assert (
        timeline["markers"]({"slider_axis_travel_history": [0.55, 0.62]})
        == pytest.approx(1.0)
    )
    assert (
        timeline["markers"](
            {
                "hinge_radius_error_history": [0.001],
                "hinge_z_error_history": [0.001],
                "slider_orthogonal_error_history": [0.001],
                "hinge_yaw_history": [0.02],
                "slider_axis_travel_history": [0.57],
                "last_metrics": {
                    "hinge_radius_error": 0.001,
                    "hinge_z_error": 0.001,
                    "slider_orthogonal_error": 0.001,
                    "hinge_yaw": 0.02,
                    "slider_axis_travel": 0.57,
                },
            }
        )
        == pytest.approx(0.0)
    )


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
    acceleration_gap = open_accel - limited_accel
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
    assert controller._step_ms_history
    assert np.isfinite(float(metrics["world_time"]))

    assert callable(setup.info[CAPTURE_METRICS_INFO_KEY])
    capture_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert capture_metrics["row"] == "rigid_joint_motor_limits"
    assert (
        capture_metrics["comparison_axis"]
        == "world_multibody_actuator_limit_family"
    )
    assert capture_metrics["solver"] == "world_multibody_joint_actuators"
    assert (
        capture_metrics["constraint"]
        == "velocity_motor_position_limit_effort_cap"
    )
    assert capture_metrics["held_fixed"] == {
        "solver": "World multibody joint actuators",
        "joint_axes": "x-axis prismatic rails and y-axis revolute stop",
        "carriage_mass": pytest.approx(2.0),
        "limit_link_mass": pytest.approx(1.0),
        "time_step_ms": pytest.approx(capture_metrics["time_step_ms"]),
    }
    assert capture_metrics["controls"]["command_speed"] == pytest.approx(
        controller.command_speed
    )
    assert capture_metrics["controls"]["velocity_limit"] == pytest.approx(
        controller.velocity_limit
    )
    assert capture_metrics["controls"]["position_limit"] == pytest.approx(
        controller.position_limit
    )
    assert capture_metrics["controls"]["force_command"] == pytest.approx(
        controller.force_command
    )
    assert capture_metrics["controls"]["effort_limit"] == pytest.approx(
        controller.effort_limit
    )
    assert capture_metrics["joint_lanes"] == [
        "velocity_motor",
        "position_limit",
        "effort_limited",
        "effort_reference",
    ]
    assert capture_metrics["joints"]["velocity_motor"] == controller.motor_joint.name
    assert capture_metrics["joints"]["position_limit"] == controller.limit_joint.name
    assert capture_metrics["joint_motor_speed"] == pytest.approx(
        float(metrics["motor_speed"])
    )
    assert capture_metrics["joint_motor_expected_speed"] == pytest.approx(
        float(metrics["motor_expected_speed"])
    )
    assert capture_metrics["joint_motor_speed_error"] == pytest.approx(
        float(metrics["motor_speed_error"])
    )
    assert capture_metrics["joint_motor_position_limit_angle"] == pytest.approx(
        float(metrics["position_limit_angle"])
    )
    assert capture_metrics["joint_motor_position_limit_error"] == pytest.approx(
        float(metrics["position_limit_error"])
    )
    assert capture_metrics["joint_motor_force_position_gap"] == pytest.approx(
        float(metrics["force_position_gap"])
    )
    assert capture_metrics["joint_motor_force_acceleration_gap"] == pytest.approx(
        float(metrics["force_acceleration_gap"])
    )
    assert capture_metrics["motor_speed"] == pytest.approx(float(metrics["motor_speed"]))
    assert capture_metrics["motor_expected_speed"] == pytest.approx(
        float(metrics["motor_expected_speed"])
    )
    assert capture_metrics["position_limit_error"] == pytest.approx(
        float(metrics["position_limit_error"])
    )
    assert capture_metrics["force_position_gap"] == pytest.approx(
        float(metrics["force_position_gap"])
    )
    assert capture_metrics["limited_force_acceleration"] == pytest.approx(
        float(metrics["limited_force_acceleration"])
    )
    assert capture_metrics["open_force_acceleration"] == pytest.approx(
        float(metrics["open_force_acceleration"])
    )
    assert capture_metrics["force_acceleration_gap"] == pytest.approx(
        float(metrics["force_acceleration_gap"])
    )
    assert capture_metrics["history"]["samples"] == pytest.approx(
        len(controller._motor_speed_history)
    )
    assert capture_metrics["history"]["max_open_force_acceleration"] == pytest.approx(
        open_accel
    )
    assert capture_metrics["history"]["max_force_acceleration_gap"] == pytest.approx(
        acceleration_gap
    )
    assert capture_metrics["history"]["max_force_position_gap"] > 0.20
    timeline = setup.info["replay_timeline"]
    snapshot = controller.capture_replay_state()
    assert timeline["signal_label"] == "Force travel gap"
    assert timeline["signal"](snapshot) == pytest.approx(
        controller._force_position_gap_history[-1]
    )
    assert (
        timeline["markers"](
            {
                "limited_acceleration_history": [0.0, 2.0],
                "open_acceleration_history": [0.0, 3.0],
                "force_position_gap_history": [0.0],
            }
        )
        == pytest.approx(1.0)
    )
    assert (
        timeline["markers"](
            {
                "controls": {"command_speed": 0.55, "velocity_limit": 0.30},
                "motor_speed_history": [0.0, 0.28],
            }
        )
        == pytest.approx(1.0)
    )
    assert (
        timeline["markers"](
            {
                "controls": {"position_limit": 0.35},
                "limit_angle_history": [0.0, 0.34],
            }
        )
        == pytest.approx(1.0)
    )
    assert (
        timeline["markers"](
            {
                "controls": {
                    "command_speed": 0.20,
                    "velocity_limit": 0.30,
                    "position_limit": 0.35,
                },
                "motor_speed_history": [0.10],
                "limit_angle_history": [0.20],
                "limit_error_history": [0.0],
                "limited_acceleration_history": [2.0],
                "open_acceleration_history": [2.1],
                "force_position_gap_history": [0.02],
                "last_metrics": {
                    "motor_speed": 0.10,
                    "position_limit_angle": 0.20,
                    "position_limit_error": 0.0,
                    "force_acceleration_gap": 0.10,
                    "force_position_gap": 0.02,
                },
            }
        )
        == pytest.approx(0.0)
    )


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

    assert callable(setup.info[CAPTURE_METRICS_INFO_KEY])
    capture_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert capture_metrics["row"] == "rigid_joint_passive_parameters"
    assert capture_metrics["comparison_axis"] == "passive_joint_parameter_family"
    assert capture_metrics["solver"] == "world_multibody_passive_joint_parameters"
    assert capture_metrics["scope"] == "contact_free_prismatic_lanes"
    assert capture_metrics["held_fixed"] == {
        "solver": "world_multibody_passive_joint_parameters",
        "joint_family": "prismatic",
        "gravity": "off",
        "contacts": "off",
        "link_mass": pytest.approx(2.0),
        "time_step_ms": pytest.approx(capture_metrics["time_step_ms"]),
    }
    assert (
        capture_metrics["executor"]
        == controller._executors[int(controller.executor_index)][0]
    )
    assert capture_metrics["controls"]["spring_stiffness"] == pytest.approx(
        controller.spring_stiffness
    )
    assert capture_metrics["controls"]["damping_coefficient"] == pytest.approx(
        controller.damping_coefficient
    )
    assert capture_metrics["controls"]["rest_position"] == pytest.approx(
        controller.rest_position
    )
    assert capture_metrics["controls"]["coulomb_friction"] == pytest.approx(
        controller.coulomb_friction
    )
    assert capture_metrics["controls"]["hold_force"] == pytest.approx(
        controller.hold_force
    )
    assert capture_metrics["controls"]["slip_force"] == pytest.approx(
        controller.slip_force
    )
    assert capture_metrics["controls"]["armature_force"] == pytest.approx(
        controller.armature_force
    )
    assert capture_metrics["controls"]["armature"] == pytest.approx(
        controller.armature
    )
    expected_lanes = [lane.key for lane in controller.lanes]
    assert capture_metrics["joint_lanes"] == expected_lanes
    assert capture_metrics["lane_order"] == expected_lanes
    assert capture_metrics["lane_count"] == pytest.approx(len(controller.lanes))
    assert set(capture_metrics["lanes"]) == {lane.key for lane in controller.lanes}
    assert capture_metrics["lanes"]["spring_only"]["kind"] == "spring"
    assert (
        capture_metrics["lanes"]["spring_only"]["joint"]
        == controller.lanes[0].joint.name
    )
    assert capture_metrics["lanes"]["slip"]["metrics"]["status"] == str(
        slip["status"]
    )
    assert capture_metrics["spring_only_position"] == pytest.approx(
        float(spring["position"])
    )
    assert capture_metrics["spring_energy"] == pytest.approx(float(spring["energy"]))
    assert capture_metrics["damped_energy"] == pytest.approx(float(damped["energy"]))
    assert capture_metrics["damped_energy_ratio"] == pytest.approx(
        float(damped["energy"]) / float(spring["energy"])
    )
    assert capture_metrics["passive_joint_spring_energy"] == pytest.approx(
        float(spring["energy"])
    )
    assert capture_metrics["passive_joint_damped_energy"] == pytest.approx(
        float(damped["energy"])
    )
    assert capture_metrics["passive_joint_damped_energy_ratio"] == pytest.approx(
        float(damped["energy"]) / float(spring["energy"])
    )
    assert capture_metrics["stiction_position"] == pytest.approx(
        float(stiction["position"])
    )
    assert capture_metrics["stiction_speed"] == pytest.approx(float(stiction["speed"]))
    assert capture_metrics["slip_position"] == pytest.approx(float(slip["position"]))
    assert capture_metrics["passive_joint_slip_speed"] == pytest.approx(
        float(slip["speed"])
    )
    assert capture_metrics["slip_acceleration"] == pytest.approx(
        float(slip["acceleration"])
    )
    assert capture_metrics["slip_acceleration_error"] == pytest.approx(
        float(slip["acceleration_error"])
    )
    assert capture_metrics["armature_reference_acceleration"] == pytest.approx(
        float(armature_reference["acceleration"])
    )
    assert capture_metrics["armature_heavy_acceleration"] == pytest.approx(
        float(armature_heavy["acceleration"])
    )
    assert capture_metrics["armature_acceleration_gap"] == pytest.approx(
        float(armature_reference["acceleration"])
        - float(armature_heavy["acceleration"])
    )
    assert capture_metrics["passive_joint_armature_acceleration_gap"] == pytest.approx(
        float(armature_reference["acceleration"])
        - float(armature_heavy["acceleration"])
    )
    assert capture_metrics["armature_position_gap"] == pytest.approx(
        float(armature_reference["position"]) - float(armature_heavy["position"])
    )
    assert capture_metrics["passive_joint_armature_position_gap"] == pytest.approx(
        float(armature_reference["position"]) - float(armature_heavy["position"])
    )
    assert capture_metrics["history"]["samples"] == pytest.approx(
        len(controller._step_ms_history)
    )
    assert capture_metrics["history"]["max_damped_energy"] == pytest.approx(
        max(controller._energy_history["spring_damper"])
    )
    assert capture_metrics["history"]["slip_max_speed"] == pytest.approx(
        max(controller._speed_history["slip"])
    )
    assert capture_metrics["history"]["armature_heavy_max_abs_acceleration"] == pytest.approx(
        max(abs(value) for value in controller._accel_history["armature_heavy"])
    )
    timeline = setup.info["replay_timeline"]
    snapshot = controller.capture_replay_state()
    latest_armature_gap = (
        controller._position_history["armature_reference"][-1]
        - controller._position_history["armature_heavy"][-1]
    )
    assert timeline["signal_label"] == "Armature position gap"
    assert timeline["signal"](snapshot) == pytest.approx(latest_armature_gap)
    assert (
        timeline["markers"](
            {
                "energy_history": {
                    "spring_only": [2.0],
                    "spring_damper": [1.7],
                }
            }
        )
        == pytest.approx(1.0)
    )
    assert (
        timeline["markers"](
            {
                "position_history": {
                    "stiction": [0.0],
                    "slip": [0.06],
                }
            }
        )
        == pytest.approx(1.0)
    )
    assert (
        timeline["markers"](
            {
                "position_history": {
                    "armature_reference": [0.10],
                    "armature_heavy": [0.03],
                }
            }
        )
        == pytest.approx(1.0)
    )
    assert (
        timeline["markers"](
            {
                "accel_history": {
                    "armature_reference": [0.75],
                    "armature_heavy": [0.10],
                }
            }
        )
        == pytest.approx(1.0)
    )
    assert (
        timeline["markers"](
            {
                "position_history": {
                    "stiction": [0.0],
                    "slip": [0.02],
                    "armature_reference": [0.04],
                    "armature_heavy": [0.02],
                },
                "energy_history": {
                    "spring_only": [2.0],
                    "spring_damper": [1.9],
                },
                "accel_history": {
                    "armature_reference": [0.30],
                    "armature_heavy": [0.10],
                },
                "last_metrics": {
                    "spring_only": {"energy": 2.0},
                    "spring_damper": {"energy": 1.9},
                    "stiction": {"position": 0.0},
                    "slip": {"position": 0.02},
                    "armature_reference": {
                        "position": 0.04,
                        "acceleration": 0.30,
                    },
                    "armature_heavy": {
                        "position": 0.02,
                        "acceleration": 0.10,
                    },
                },
            }
        )
        == pytest.approx(0.0)
    )


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

    assert callable(setup.info[CAPTURE_METRICS_INFO_KEY])
    capture_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert capture_metrics["row"] == "rigid_screw_joint_pitch"
    assert capture_metrics["comparison_axis"] == "screw_pitch_coupling_family"
    assert capture_metrics["solver"] == "world_multibody_screw_joint_pitch"
    assert capture_metrics["scope"] == "contact_free_screw_pitch_lanes"
    assert capture_metrics["executor"] == controller._executors[0][0]
    assert capture_metrics["held_fixed"] == {
        "solver": "world_multibody_screw_joint_pitch",
        "joint_family": "screw",
        "axis": "z",
        "contacts": "off",
        "moving_mass": pytest.approx(controller.moving_mass),
        "axial_inertia": pytest.approx(controller.axial_inertia),
        "time_step_ms": pytest.approx(capture_metrics["time_step_ms"]),
    }
    assert capture_metrics["controls"]["pitch_scale"] == pytest.approx(
        controller.pitch_scale
    )
    assert capture_metrics["controls"]["gravity_scale"] == pytest.approx(
        controller.gravity_scale
    )
    assert capture_metrics["controls"]["moving_mass"] == pytest.approx(
        controller.moving_mass
    )
    assert capture_metrics["controls"]["axial_inertia"] == pytest.approx(
        controller.axial_inertia
    )
    expected_lanes = [lane.key for lane in controller.lanes]
    assert capture_metrics["joint_lanes"] == expected_lanes
    assert capture_metrics["lane_order"] == expected_lanes
    assert capture_metrics["lane_count"] == pytest.approx(len(controller.lanes))
    assert set(capture_metrics["lanes"]) == {lane.key for lane in controller.lanes}
    assert capture_metrics["lanes"]["fine_pitch"]["joint"] == (
        controller.lanes[1].joint.name
    )
    assert capture_metrics["lanes"]["reverse_pitch"]["pitch_multiplier"] == (
        pytest.approx(-1.0)
    )
    assert capture_metrics["lanes"]["zero_pitch"]["metrics"]["status"] == str(
        zero["status"]
    )
    assert capture_metrics["zero_pitch"] == pytest.approx(float(zero["pitch"]))
    assert capture_metrics["zero_pitch_angle"] == pytest.approx(
        float(zero["angle"])
    )
    assert capture_metrics["screw_joint_zero_pitch_axial_travel"] == pytest.approx(
        float(zero["axial_travel"])
    )
    assert capture_metrics["fine_pitch"] == pytest.approx(float(fine["pitch"]))
    assert capture_metrics["coarse_pitch"] == pytest.approx(float(coarse["pitch"]))
    assert capture_metrics["reverse_pitch"] == pytest.approx(float(reverse["pitch"]))
    assert capture_metrics["screw_joint_fine_pitch"] == pytest.approx(
        float(fine["pitch"])
    )
    assert capture_metrics["screw_joint_coarse_pitch"] == pytest.approx(
        float(coarse["pitch"])
    )
    assert capture_metrics["screw_joint_reverse_pitch"] == pytest.approx(
        float(reverse["pitch"])
    )
    assert capture_metrics["coarse_to_fine_pitch_ratio"] == pytest.approx(2.0)
    assert capture_metrics["reverse_to_fine_pitch_ratio"] == pytest.approx(-1.0)
    assert capture_metrics["fine_angle"] == pytest.approx(float(fine["angle"]))
    assert capture_metrics["coarse_angle"] == pytest.approx(float(coarse["angle"]))
    assert capture_metrics["reverse_angle"] == pytest.approx(
        float(reverse["angle"])
    )
    assert capture_metrics["fine_axial_travel"] == pytest.approx(
        float(fine["axial_travel"])
    )
    assert capture_metrics["coarse_axial_travel"] == pytest.approx(
        float(coarse["axial_travel"])
    )
    assert capture_metrics["reverse_axial_travel"] == pytest.approx(
        float(reverse["axial_travel"])
    )
    assert capture_metrics["coarse_minus_fine_axial_travel"] == pytest.approx(
        float(coarse["axial_travel"]) - float(fine["axial_travel"])
    )
    assert capture_metrics["screw_joint_coarse_fine_travel_gap"] == pytest.approx(
        abs(float(coarse["axial_travel"]) - float(fine["axial_travel"]))
    )
    assert capture_metrics["screw_joint_reverse_angle"] == pytest.approx(
        float(reverse["angle"])
    )
    assert capture_metrics["fine_travel_per_radian"] == pytest.approx(
        float(fine["travel_per_radian"])
    )
    assert capture_metrics["coarse_travel_per_radian"] == pytest.approx(
        float(coarse["travel_per_radian"])
    )
    assert capture_metrics["reverse_travel_per_radian"] == pytest.approx(
        float(reverse["travel_per_radian"])
    )
    assert capture_metrics["fine_acceleration"] == pytest.approx(
        float(fine["acceleration"])
    )
    assert capture_metrics["fine_expected_acceleration"] == pytest.approx(
        float(fine["expected_acceleration"])
    )
    assert capture_metrics["fine_acceleration_error"] == pytest.approx(
        float(fine["acceleration_error"])
    )
    assert capture_metrics["screw_joint_fine_acceleration_error"] == pytest.approx(
        float(fine["acceleration_error"])
    )
    assert capture_metrics["coarse_actual_axial_acceleration"] == pytest.approx(
        float(coarse["actual_axial_acceleration"])
    )
    assert capture_metrics["coarse_expected_axial_acceleration"] == pytest.approx(
        float(coarse["expected_axial_acceleration"])
    )
    assert capture_metrics["reverse_effective_mass"] == pytest.approx(
        float(reverse["effective_mass"])
    )
    assert capture_metrics["reverse_mass_matrix"] == pytest.approx(
        float(reverse["mass_matrix"])
    )
    assert capture_metrics["history"]["samples"] == pytest.approx(
        len(controller._step_ms_history)
    )
    assert capture_metrics["history"]["fine_pitch_max_abs_angle"] == pytest.approx(
        max(abs(value) for value in controller._angle_history["fine_pitch"])
    )
    assert capture_metrics["history"][
        "coarse_pitch_max_abs_axial_travel"
    ] == pytest.approx(
        max(abs(value) for value in controller._travel_history["coarse_pitch"])
    )
    assert capture_metrics["history"][
        "reverse_pitch_max_abs_acceleration_error"
    ] == pytest.approx(
        max(
            abs(value)
            for value in list(controller._accel_error_history["reverse_pitch"])[1:]
        )
    )
    timeline = setup.info["replay_timeline"]
    snapshot = controller.capture_replay_state()
    latest_travel_gap = abs(
        controller._travel_history["coarse_pitch"][-1]
        - controller._travel_history["fine_pitch"][-1]
    )
    assert timeline["signal_label"] == "Coarse/fine travel gap"
    assert timeline["signal"](snapshot) == pytest.approx(latest_travel_gap)
    assert (
        timeline["markers"](
            {
                "travel_history": {
                    "fine_pitch": [-0.11],
                    "coarse_pitch": [-0.18],
                }
            }
        )
        == pytest.approx(1.0)
    )
    assert (
        timeline["markers"](
            {
                "travel_history": {
                    "zero_pitch": [0.0],
                    "fine_pitch": [-0.09],
                }
            }
        )
        == pytest.approx(1.0)
    )
    assert (
        timeline["markers"](
            {
                "angle_history": {
                    "fine_pitch": [-0.30],
                    "reverse_pitch": [0.30],
                }
            }
        )
        == pytest.approx(1.0)
    )
    assert (
        timeline["markers"](
            {
                "travel_history": {
                    "zero_pitch": [0.005],
                    "fine_pitch": [-0.04],
                    "coarse_pitch": [-0.06],
                },
                "angle_history": {
                    "fine_pitch": [-0.10],
                    "reverse_pitch": [0.10],
                },
                "last_metrics": {
                    "zero_pitch": {"axial_travel": 0.005},
                    "fine_pitch": {"axial_travel": -0.04, "angle": -0.10},
                    "coarse_pitch": {"axial_travel": -0.06},
                    "reverse_pitch": {"angle": 0.10},
                },
            }
        )
        == pytest.approx(0.0)
    )


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

    assert callable(setup.info[CAPTURE_METRICS_INFO_KEY])
    capture_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert capture_metrics["row"] == "rigid_multibody_dynamics_terms"
    assert capture_metrics["comparison_axis"] == "joint_space_dynamics_term_family"
    assert capture_metrics["solver"] == "world_multibody_dynamics_terms"
    assert capture_metrics["scope"] == "contact_free_joint_space_dynamics"
    assert capture_metrics["executor"] == controller._executors[0][0]
    assert capture_metrics["held_fixed"] == {
        "solver": "world_multibody_dynamics_terms",
        "contacts": "off",
        "base": "fixed",
        "joint_type": "revolute",
        "target_acceleration": pytest.approx(controller.target_acceleration),
        "joint_impulse": pytest.approx(controller.joint_impulse),
        "gravity_scale": pytest.approx(controller.gravity_scale),
        "link_length": pytest.approx(0.55),
        "time_step_ms": pytest.approx(capture_metrics["time_step_ms"]),
    }
    assert capture_metrics["time_step_ms"] == pytest.approx(3.0)
    assert capture_metrics["world_time"] > 0.0
    assert capture_metrics["target_acceleration"] == pytest.approx(
        controller.target_acceleration
    )
    assert capture_metrics["joint_impulse"] == pytest.approx(controller.joint_impulse)
    assert capture_metrics["heavy_distal_mass_scale"] == pytest.approx(
        controller.heavy_distal_mass_scale
    )
    assert capture_metrics["gravity_scale"] == pytest.approx(controller.gravity_scale)
    assert capture_metrics["controls"]["executor_index"] == pytest.approx(
        controller.executor_index
    )
    assert capture_metrics["controls"]["target_acceleration"] == pytest.approx(
        controller.target_acceleration
    )
    assert capture_metrics["controls"]["joint_impulse"] == pytest.approx(
        controller.joint_impulse
    )
    assert capture_metrics["controls"]["heavy_distal_mass_scale"] == pytest.approx(
        controller.heavy_distal_mass_scale
    )
    assert capture_metrics["controls"]["gravity_scale"] == pytest.approx(
        controller.gravity_scale
    )
    expected_lanes = [lane.key for lane in controller.lanes]
    assert capture_metrics["dynamics_lanes"] == expected_lanes
    assert capture_metrics["lane_order"] == expected_lanes
    assert capture_metrics["lane_count"] == pytest.approx(len(controller.lanes))
    assert set(capture_metrics["lanes"]) == {lane.key for lane in controller.lanes}

    for lane in controller.lanes:
        lane_capture = capture_metrics["lanes"][lane.key]
        lane_metrics = metrics[lane.key]
        assert lane_capture["label"] == lane.label
        assert lane_capture["dofs"] == pytest.approx(len(lane.joints))
        assert lane_capture["joints"] == [joint.name for joint in lane.joints]
        assert lane_capture["target_pattern"] == pytest.approx(
            list(lane.target_pattern)
        )
        assert lane_capture["impulse_pattern"] == pytest.approx(
            list(lane.impulse_pattern)
        )
        assert lane_capture["metrics"]["status"] == str(lane_metrics["status"])
        for metric_key, metric_value in lane_metrics.items():
            if isinstance(metric_value, str):
                continue
            assert lane_capture["metrics"][metric_key] == pytest.approx(
                float(metric_value)
            )
            assert capture_metrics[f"{lane.key}_{metric_key}"] == pytest.approx(
                float(metric_value)
            )

    assert capture_metrics["single_hinge_mass_diag0"] == pytest.approx(
        float(single["mass_diag0"])
    )
    assert capture_metrics["multibody_dynamics_single_mass_diag0"] == pytest.approx(
        float(single["mass_diag0"])
    )
    assert capture_metrics["single_hinge_inverse_diag0"] == pytest.approx(
        float(single["inverse_diag0"])
    )
    assert capture_metrics["single_hinge_dynamics_residual"] == pytest.approx(
        float(single["dynamics_residual"])
    )
    assert capture_metrics["coupled_two_link_coupling"] == pytest.approx(
        float(coupled["coupling"])
    )
    assert capture_metrics["multibody_dynamics_coupled_coupling"] == pytest.approx(
        float(coupled["coupling"])
    )
    assert capture_metrics["coupled_two_link_response1"] == pytest.approx(
        float(coupled["response1"])
    )
    assert capture_metrics["heavy_distal_coupling"] == pytest.approx(
        float(heavy["coupling"])
    )
    assert capture_metrics["heavy_distal_response1"] == pytest.approx(
        float(heavy["response1"])
    )
    assert capture_metrics["heavy_minus_coupled_tau_norm"] == pytest.approx(
        float(heavy["tau_norm"]) - float(coupled["tau_norm"])
    )
    assert capture_metrics["multibody_dynamics_heavy_tau_gap"] == pytest.approx(
        float(heavy["tau_norm"]) - float(coupled["tau_norm"])
    )
    assert capture_metrics["heavy_to_coupled_tau_norm_ratio"] == pytest.approx(
        float(heavy["tau_norm"]) / float(coupled["tau_norm"])
    )
    assert capture_metrics["heavy_to_coupled_response_norm_ratio"] == pytest.approx(
        float(heavy["response_norm"]) / float(coupled["response_norm"])
    )
    assert capture_metrics[
        "multibody_dynamics_coupled_heavy_response_gap"
    ] == pytest.approx(
        max(0.0, float(coupled["response_norm"]) - float(heavy["response_norm"]))
    )
    assert capture_metrics["multibody_dynamics_heavy_response_ratio"] == pytest.approx(
        float(heavy["response_norm"]) / float(coupled["response_norm"])
    )
    assert capture_metrics["coupled_response1_abs"] == pytest.approx(
        abs(float(coupled["response1"]))
    )
    assert capture_metrics["heavy_response1_abs"] == pytest.approx(
        abs(float(heavy["response1"]))
    )
    assert capture_metrics["max_dynamics_residual"] == pytest.approx(
        max(float(value["dynamics_residual"]) for value in metrics.values())
    )
    assert capture_metrics["max_inverse_dynamics_residual"] == pytest.approx(
        max(float(value["inverse_dynamics_residual"]) for value in metrics.values())
    )
    assert capture_metrics["multibody_dynamics_max_inverse_residual"] == pytest.approx(
        max(float(value["inverse_dynamics_residual"]) for value in metrics.values())
    )
    assert capture_metrics["max_impulse_residual"] == pytest.approx(
        max(float(value["impulse_residual"]) for value in metrics.values())
    )
    assert capture_metrics["multibody_dynamics_max_impulse_residual"] == pytest.approx(
        max(float(value["impulse_residual"]) for value in metrics.values())
    )
    assert capture_metrics["max_acceleration_error"] == pytest.approx(
        max(float(value["acceleration_error"]) for value in metrics.values())
    )
    assert capture_metrics["max_identity_error"] == pytest.approx(
        max(float(value["identity_error"]) for value in metrics.values())
    )
    assert capture_metrics["step_ms"] == pytest.approx(
        controller._step_ms_history[-1]
    )
    assert capture_metrics["history"]["samples"] == pytest.approx(
        len(controller._step_ms_history)
    )
    assert capture_metrics["history"]["max_step_ms"] == pytest.approx(
        max(controller._step_ms_history)
    )
    assert capture_metrics["history"]["max_abs_coupling"] == pytest.approx(
        max(abs(value) for value in controller._coupling_history)
    )
    for lane_key in metrics:
        assert capture_metrics["history"][
            f"{lane_key}_max_dynamics_residual"
        ] == pytest.approx(max(controller._residual_history[lane_key]))
        assert capture_metrics["history"][
            f"{lane_key}_max_response_norm"
        ] == pytest.approx(max(controller._response_history[lane_key]))
        assert capture_metrics["history"][f"{lane_key}_max_tau_norm"] == pytest.approx(
            max(controller._tau_history[lane_key])
        )
    timeline = setup.info["replay_timeline"]
    snapshot = controller.capture_replay_state()
    latest_response_gap = max(
        0.0,
        controller._response_history["coupled_two_link"][-1]
        - controller._response_history["heavy_distal"][-1],
    )
    assert timeline["signal_label"] == "Response norm gap"
    assert timeline["signal"](snapshot) == pytest.approx(latest_response_gap)
    assert (
        timeline["markers"](
            {
                "response_history": {
                    "coupled_two_link": [2.0],
                    "heavy_distal": [0.5],
                }
            }
        )
        == pytest.approx(1.0)
    )
    assert timeline["markers"]({"coupling_history": [0.12]}) == pytest.approx(
        1.0
    )
    assert (
        timeline["markers"](
            {
                "tau_history": {
                    "coupled_two_link": [3.0],
                    "heavy_distal": [5.5],
                }
            }
        )
        == pytest.approx(1.0)
    )
    assert (
        timeline["markers"](
            {
                "response_history": {
                    "coupled_two_link": [1.2],
                    "heavy_distal": [0.9],
                },
                "coupling_history": [0.05],
                "tau_history": {
                    "coupled_two_link": [3.0],
                    "heavy_distal": [4.0],
                },
                "last_metrics": {
                    "coupled_two_link": {
                        "response_norm": 1.2,
                        "coupling": 0.05,
                        "tau_norm": 3.0,
                    },
                    "heavy_distal": {
                        "response_norm": 0.9,
                        "tau_norm": 4.0,
                    },
                },
            }
        )
        == pytest.approx(0.0)
    )


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

    assert callable(setup.info[CAPTURE_METRICS_INFO_KEY])
    capture_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert capture_metrics["row"] == "rigid_link_center_of_mass"
    assert capture_metrics["comparison_axis"] == "link_center_of_mass_offset_family"
    assert capture_metrics["solver"] == "world_multibody_inertial_offsets"
    assert capture_metrics["scope"] == "contact_free_link_center_of_mass_offsets"
    assert capture_metrics["executor"] == controller._executors[0][0]
    assert capture_metrics["held_fixed"] == {
        "solver": "world_multibody_inertial_offsets",
        "contacts": "off",
        "joint_type": "revolute",
        "visual_geometry": "fixed",
        "link_mass": pytest.approx(controller.link_mass),
        "gravity_scale": pytest.approx(controller.gravity_scale),
        "time_step_ms": pytest.approx(capture_metrics["time_step_ms"]),
    }
    assert capture_metrics["time_step_ms"] == pytest.approx(3.0)
    assert capture_metrics["world_time"] > 0.0
    assert capture_metrics["com_offset"] == pytest.approx(controller.com_offset)
    assert capture_metrics["gravity_scale"] == pytest.approx(controller.gravity_scale)
    assert capture_metrics["link_mass"] == pytest.approx(controller.link_mass)
    assert capture_metrics["inertia_scale"] == pytest.approx(controller.inertia_scale)
    assert capture_metrics["controls"]["executor_index"] == pytest.approx(
        controller.executor_index
    )
    assert capture_metrics["controls"]["com_offset"] == pytest.approx(
        controller.com_offset
    )
    assert capture_metrics["controls"]["gravity_scale"] == pytest.approx(
        controller.gravity_scale
    )
    assert capture_metrics["controls"]["link_mass"] == pytest.approx(
        controller.link_mass
    )
    assert capture_metrics["controls"]["inertia_scale"] == pytest.approx(
        controller.inertia_scale
    )
    expected_lanes = [lane.key for lane in controller.lanes]
    assert capture_metrics["com_lanes"] == expected_lanes
    assert capture_metrics["lane_order"] == expected_lanes
    assert capture_metrics["lane_count"] == pytest.approx(len(controller.lanes))
    assert set(capture_metrics["lanes"]) == {lane.key for lane in controller.lanes}

    for lane in controller.lanes:
        lane_capture = capture_metrics["lanes"][lane.key]
        lane_metrics = metrics[lane.key]
        assert lane_capture["label"] == lane.label
        assert lane_capture["offset_multiplier"] == pytest.approx(
            lane.offset_multiplier
        )
        assert lane_capture["high_inertia"] is lane.high_inertia
        assert lane_capture["link"] == lane.link.name
        assert lane_capture["joint"] == lane.joint.name
        assert lane_capture["local_center_of_mass"] == pytest.approx(
            [float(lane_metrics["offset"]), 0.0, 0.0]
        )
        assert lane_capture["metrics"]["status"] == str(lane_metrics["status"])
        for metric_key, metric_value in lane_metrics.items():
            if isinstance(metric_value, str):
                continue
            assert lane_capture["metrics"][metric_key] == pytest.approx(
                float(metric_value)
            )
            assert capture_metrics[f"{lane.key}_{metric_key}"] == pytest.approx(
                float(metric_value)
            )

    assert capture_metrics["centered_gravity_torque"] == pytest.approx(
        float(centered["gravity_torque"])
    )
    assert capture_metrics["link_com_centered_gravity_torque"] == pytest.approx(
        float(centered["gravity_torque"])
    )
    assert capture_metrics["positive_offset"] == pytest.approx(
        float(positive["offset"])
    )
    assert capture_metrics["negative_offset"] == pytest.approx(
        float(negative["offset"])
    )
    assert capture_metrics["positive_gravity_torque"] == pytest.approx(
        float(positive["gravity_torque"])
    )
    assert capture_metrics["link_com_positive_gravity_torque"] == pytest.approx(
        float(positive["gravity_torque"])
    )
    assert capture_metrics["negative_gravity_torque"] == pytest.approx(
        float(negative["gravity_torque"])
    )
    assert capture_metrics["link_com_negative_gravity_torque"] == pytest.approx(
        float(negative["gravity_torque"])
    )
    assert capture_metrics["high_inertia_gravity_torque"] == pytest.approx(
        float(high["gravity_torque"])
    )
    assert capture_metrics["positive_mass_matrix"] == pytest.approx(
        float(positive["mass_matrix"])
    )
    assert capture_metrics["high_inertia_mass_matrix"] == pytest.approx(
        float(high["mass_matrix"])
    )
    assert capture_metrics["high_to_positive_mass_matrix_ratio"] == pytest.approx(
        float(high["mass_matrix"]) / float(positive["mass_matrix"])
    )
    assert capture_metrics["link_com_high_mass_matrix_ratio"] == pytest.approx(
        float(high["mass_matrix"]) / float(positive["mass_matrix"])
    )
    assert capture_metrics["positive_angle"] == pytest.approx(float(positive["angle"]))
    assert capture_metrics["negative_angle"] == pytest.approx(float(negative["angle"]))
    assert capture_metrics["positive_negative_angle_sum"] == pytest.approx(
        float(positive["angle"]) + float(negative["angle"])
    )
    assert capture_metrics["link_com_positive_negative_angle_sum"] == pytest.approx(
        float(positive["angle"]) + float(negative["angle"])
    )
    assert capture_metrics["positive_acceleration"] == pytest.approx(
        float(positive["acceleration"])
    )
    assert capture_metrics["positive_expected_acceleration"] == pytest.approx(
        float(positive["expected_acceleration"])
    )
    assert capture_metrics["negative_acceleration"] == pytest.approx(
        float(negative["acceleration"])
    )
    assert capture_metrics["high_inertia_acceleration"] == pytest.approx(
        float(high["acceleration"])
    )
    assert capture_metrics["negative_to_positive_acceleration_ratio"] == pytest.approx(
        float(negative["acceleration"]) / abs(float(positive["acceleration"]))
    )
    assert capture_metrics["positive_negative_acceleration_sum"] == pytest.approx(
        float(positive["acceleration"]) + float(negative["acceleration"])
    )
    assert capture_metrics["high_to_positive_acceleration_ratio"] == pytest.approx(
        float(high["acceleration"]) / float(positive["acceleration"])
    )
    assert capture_metrics["link_com_high_acceleration_ratio"] == pytest.approx(
        float(high["acceleration"]) / float(positive["acceleration"])
    )
    assert capture_metrics["positive_negative_torque_sum"] == pytest.approx(
        float(positive["gravity_torque"]) + float(negative["gravity_torque"])
    )
    assert capture_metrics["positive_com_world_x"] == pytest.approx(
        float(positive["com_world_x"])
    )
    assert capture_metrics["positive_com_world_z"] == pytest.approx(
        float(positive["com_world_z"])
    )
    assert capture_metrics["max_abs_acceleration_error"] == pytest.approx(
        max(abs(float(value["acceleration_error"])) for value in metrics.values())
    )
    assert capture_metrics["link_com_max_acceleration_error"] == pytest.approx(
        max(abs(float(value["acceleration_error"])) for value in metrics.values())
    )
    assert capture_metrics["step_ms"] == pytest.approx(
        controller._step_ms_history[-1]
    )
    assert capture_metrics["max_step_ms"] == pytest.approx(
        max(controller._step_ms_history)
    )
    assert capture_metrics["history"]["samples"] == pytest.approx(
        len(controller._step_ms_history)
    )
    assert capture_metrics["history"]["max_step_ms"] == pytest.approx(
        max(controller._step_ms_history)
    )
    for lane_key in metrics:
        accel_error_samples = list(controller._accel_error_history[lane_key])
        accel_error_samples = (
            accel_error_samples[1:]
            if len(accel_error_samples) > 1
            else accel_error_samples
        )
        assert capture_metrics["history"][
            f"{lane_key}_max_abs_angle"
        ] == pytest.approx(max(abs(value) for value in controller._angle_history[lane_key]))
        assert capture_metrics["history"][
            f"{lane_key}_max_abs_acceleration"
        ] == pytest.approx(
            max(abs(value) for value in controller._accel_history[lane_key])
        )
        assert capture_metrics["history"][
            f"{lane_key}_max_abs_acceleration_error"
        ] == pytest.approx(max(abs(value) for value in accel_error_samples))
        assert capture_metrics["history"][
            f"{lane_key}_max_abs_gravity_torque"
        ] == pytest.approx(max(abs(value) for value in controller._torque_history[lane_key]))
        assert capture_metrics["history"][
            f"{lane_key}_max_abs_energy"
        ] == pytest.approx(max(abs(value) for value in controller._energy_history[lane_key]))
    timeline = setup.info["replay_timeline"]
    snapshot = controller.capture_replay_state()
    latest_angle_spread = abs(
        controller._angle_history["positive"][-1]
        - controller._angle_history["negative"][-1]
    )
    assert timeline["signal_label"] == "Mirrored COM angle spread"
    assert timeline["signal"](snapshot) == pytest.approx(latest_angle_spread)
    assert (
        timeline["markers"](
            {
                "angle_history": {
                    "positive": [0.10],
                    "negative": [-0.09],
                }
            }
        )
        == pytest.approx(1.0)
    )
    assert (
        timeline["markers"](
            {
                "angle_history": {
                    "centered": [0.0],
                    "positive": [0.09],
                }
            }
        )
        == pytest.approx(1.0)
    )
    assert (
        timeline["markers"](
            {
                "angle_history": {
                    "positive": [0.18],
                    "high_inertia": [0.10],
                }
            }
        )
        == pytest.approx(1.0)
    )
    assert (
        timeline["markers"](
            {
                "angle_history": {
                    "centered": [0.002],
                    "positive": [0.04],
                    "negative": [-0.03],
                    "high_inertia": [0.035],
                },
                "last_metrics": {
                    "centered": {"angle": 0.002},
                    "positive": {"angle": 0.04},
                    "negative": {"angle": -0.03},
                    "high_inertia": {"angle": 0.035},
                },
            }
        )
        == pytest.approx(0.0)
    )


def test_rigid_link_jacobian_maps_link_origin_twist_and_wrench() -> None:
    import numpy as np

    _require_simulation_symbols("World", "JointSpec", "JointType")

    from examples.demos.scenes.rigid_link_jacobian import _BASE_ANCHOR, build

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
    timeline = setup.info["replay_timeline"]
    snapshot = controller.capture_replay_state()
    assert timeline["signal_label"] == "Link-origin speed"
    assert timeline["signal"](snapshot) == pytest.approx(controller._speed_history[-1])
    assert timeline["signal"]({"last_metrics": {"linear_speed": 0.42}}) == pytest.approx(
        0.42
    )
    assert timeline["markers"]({"speed_history": [0.76]}) == pytest.approx(1.0)
    assert timeline["markers"]({"tau0_history": [-0.51]}) == pytest.approx(1.0)
    assert timeline["markers"]({"tau1_history": [0.51]}) == pytest.approx(1.0)
    assert timeline["markers"]({"world_body_gap_history": [0.11]}) == pytest.approx(
        1.0
    )
    assert timeline["markers"]({"fd_error_history": [1.2e-6]}) == pytest.approx(
        1.0
    )
    assert timeline["markers"]({"power_error_history": [1.2e-9]}) == pytest.approx(
        1.0
    )
    assert (
        timeline["markers"](
            {
                "speed_history": [0.20],
                "tau0_history": [0.10],
                "tau1_history": [0.10],
                "world_body_gap_history": [0.04],
                "fd_error_history": [2.0e-7],
                "power_error_history": [2.0e-10],
            }
        )
        == pytest.approx(0.0)
    )

    assert callable(setup.info[CAPTURE_METRICS_INFO_KEY])
    capture_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert capture_metrics["row"] == "rigid_link_jacobian"
    assert capture_metrics["comparison_axis"] == "link_origin_jacobian_mapping_family"
    assert capture_metrics["solver"] == "world_multibody_link_jacobian"
    assert capture_metrics["scope"] == "contact_free_link_origin_jacobian_wrench_map"
    assert capture_metrics["held_fixed"] == {
        "solver": "world_multibody_link_jacobian",
        "contacts": "off",
        "gravity": "off",
        "joint_family": "two_revolute_links",
        "link_length": pytest.approx(0.55),
        "finite_difference_eps": pytest.approx(1.0e-6),
        "time_step_ms": pytest.approx(capture_metrics["time_step_ms"]),
    }
    assert capture_metrics["time_step_ms"] == pytest.approx(4.0)
    assert capture_metrics["world_time"] == pytest.approx(controller.world.time)
    assert capture_metrics["motion_speed"] == pytest.approx(controller.motion_speed)
    assert capture_metrics["elbow_phase"] == pytest.approx(controller.elbow_phase)
    assert capture_metrics["wrench_force"] == pytest.approx(controller.wrench_force)
    assert capture_metrics["wrench_angle_deg"] == pytest.approx(
        controller.wrench_angle_deg
    )
    assert capture_metrics["wrench_moment"] == pytest.approx(
        controller.wrench_moment
    )
    assert capture_metrics["controls"]["motion_speed"] == pytest.approx(
        controller.motion_speed
    )
    assert capture_metrics["controls"]["elbow_phase"] == pytest.approx(
        controller.elbow_phase
    )
    assert capture_metrics["controls"]["wrench_force"] == pytest.approx(
        controller.wrench_force
    )
    assert capture_metrics["controls"]["wrench_angle_deg"] == pytest.approx(
        controller.wrench_angle_deg
    )
    assert capture_metrics["controls"]["wrench_moment"] == pytest.approx(
        controller.wrench_moment
    )
    assert capture_metrics["jacobian_terms"] == [
        "world_jacobian_twist",
        "finite_difference_velocity",
        "jacobian_transpose_wrench",
        "world_body_jacobian_gap",
    ]
    assert capture_metrics["joint_names"] == [
        joint.name for joint in controller.joints
    ]
    assert capture_metrics["link"] == controller.links[-1].name
    assert capture_metrics["metrics"]["status"] == str(metrics["status"])
    for metric_key, metric_value in metrics.items():
        if isinstance(metric_value, str):
            continue
        assert capture_metrics["metrics"][metric_key] == pytest.approx(
            float(metric_value)
        )
        assert capture_metrics[metric_key] == pytest.approx(float(metric_value))
    assert capture_metrics["link_jacobian_linear_speed"] == pytest.approx(
        float(metrics["linear_speed"])
    )
    assert capture_metrics["link_jacobian_angular_speed"] == pytest.approx(
        float(metrics["angular_speed"])
    )
    assert capture_metrics["link_jacobian_world_body_gap"] == pytest.approx(
        float(metrics["world_body_gap"])
    )
    assert capture_metrics["link_jacobian_finite_difference_error"] == pytest.approx(
        float(metrics["finite_difference_error"])
    )
    assert capture_metrics["link_jacobian_tau0"] == pytest.approx(
        float(metrics["tau0"])
    )
    assert capture_metrics["link_jacobian_tau1"] == pytest.approx(
        float(metrics["tau1"])
    )
    assert capture_metrics["link_jacobian_power_error"] == pytest.approx(
        float(metrics["power_error"])
    )

    link_origin = _BASE_ANCHOR + controller._link_origin()
    assert capture_metrics["link_origin_world_x"] == pytest.approx(link_origin[0])
    assert capture_metrics["link_origin_world_y"] == pytest.approx(link_origin[1])
    assert capture_metrics["link_origin_world_z"] == pytest.approx(link_origin[2])
    assert capture_metrics["linear_velocity_x"] == pytest.approx(
        controller._last_velocity_vector[0]
    )
    assert capture_metrics["linear_velocity_y"] == pytest.approx(
        controller._last_velocity_vector[1]
    )
    assert capture_metrics["linear_velocity_z"] == pytest.approx(
        controller._last_velocity_vector[2]
    )
    assert capture_metrics["wrench_force_x"] == pytest.approx(
        controller._last_force_vector[0]
    )
    assert capture_metrics["wrench_force_y"] == pytest.approx(
        controller._last_force_vector[1]
    )
    assert capture_metrics["wrench_force_z"] == pytest.approx(
        controller._last_force_vector[2]
    )
    assert capture_metrics["history"]["samples"] == pytest.approx(
        len(controller._speed_history)
    )
    assert capture_metrics["history"]["max_linear_speed"] == pytest.approx(
        max(controller._speed_history)
    )
    assert capture_metrics["history"][
        "max_finite_difference_error"
    ] == pytest.approx(max(controller._fd_error_history))
    assert capture_metrics["history"]["max_power_error"] == pytest.approx(
        max(controller._power_error_history)
    )
    assert capture_metrics["history"]["max_abs_tau0"] == pytest.approx(
        max(abs(value) for value in controller._tau0_history)
    )
    assert capture_metrics["history"]["max_abs_tau1"] == pytest.approx(
        max(abs(value) for value in controller._tau1_history)
    )
    assert capture_metrics["history"]["max_world_body_gap"] == pytest.approx(
        max(controller._world_body_gap_history)
    )


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
    assert callable(setup.info[CAPTURE_METRICS_INFO_KEY])
    capture_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert capture_metrics["row"] == "rigid_link_point_loads"
    assert capture_metrics["solver"] == "sequential_impulse"
    assert capture_metrics["executor"] == controller._executors[0][0]
    assert set(capture_metrics["lanes"]) == set(controller._last_metrics)
    assert capture_metrics["center_world_accel_x"] == pytest.approx(
        controller._last_metrics["center"]["world_accel_x"]
    )
    assert capture_metrics["offcenter_yaw_accel"] == pytest.approx(
        controller._last_metrics["offcenter"]["yaw_accel"]
    )
    assert capture_metrics["pulse_applied_count"] == pytest.approx(
        pulse_after_clear["applied_count"]
    )
    assert capture_metrics["local_frame_world_accel_y"] == pytest.approx(
        controller._last_metrics["local_frame"]["world_accel_y"]
    )
    assert capture_metrics["history"]["samples"] == pytest.approx(
        float(len(controller._step_ms_history))
    )


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

    assert callable(setup.info[CAPTURE_METRICS_INFO_KEY])
    capture_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert capture_metrics["row"] == "rigid_loop_closure"
    assert capture_metrics["comparison_axis"] == "loop_closure_family_policy_selection"
    assert capture_metrics["solver"] == "variational_rigid_multibody_loop_closure"
    assert (
        capture_metrics["scope"]
        == "point_distance_rigid_closure_family_selection"
    )
    assert capture_metrics["held_fixed"] == {
        "solver": "variational_rigid_multibody_loop_closure",
        "contacts": "off",
        "integration_family": "variational integrator",
        "joint_family": "four_revolute_links",
        "chain_links": 4,
        "link_length": pytest.approx(0.56),
        "link_mass": pytest.approx(0.55),
        "initial_bend": pytest.approx(0.18),
        "gravity_scale": pytest.approx(controller.gravity_scale),
        "time_step_ms": pytest.approx(capture_metrics["time_step_ms"]),
    }
    assert capture_metrics["executor"] == controller._executors[
        controller.executor_index
    ][0]
    assert capture_metrics["time_step_ms"] == pytest.approx(5.0)
    assert capture_metrics["world_time"] == pytest.approx(
        controller.primary_world.time
    )
    assert capture_metrics["gravity_scale"] == pytest.approx(
        controller.gravity_scale
    )
    assert capture_metrics["controls"]["executor_index"] == pytest.approx(
        controller.executor_index
    )
    assert capture_metrics["controls"]["gravity_scale"] == pytest.approx(
        controller.gravity_scale
    )
    assert capture_metrics["family_order"] == ["POINT", "DISTANCE", "RIGID"]
    assert capture_metrics["policy_order"] == ["residual", "solved"]
    assert capture_metrics["closure_family_lanes"] == [
        "POINT",
        "DISTANCE",
        "RIGID",
    ]
    assert capture_metrics["closure_policy_lanes"] == ["residual", "solved"]
    assert capture_metrics["case_order"] == [case.key for case in controller.cases]
    assert capture_metrics["case_count"] == pytest.approx(len(controller.cases))
    assert set(capture_metrics["cases"]) == {case.key for case in controller.cases}
    assert set(capture_metrics["families"]) == {"POINT", "DISTANCE", "RIGID"}
    for case in controller.cases:
        case_payload = capture_metrics["cases"][case.key]
        assert case_payload["label"] == case.label
        assert case_payload["family"] == case.family_label
        assert case_payload["policy"] == case.policy_label
        assert case_payload["dynamic_solve"] is (
            case.dynamics == sx.ClosureDynamicsPolicy.SOLVE
        )
        assert case_payload["closure"] == case.closure.name
        assert case_payload["target_point"] == pytest.approx(case.target_point)
        assert case_payload["anchor_point"] == pytest.approx(case.anchor_point)
        assert case_payload["target_distance"] == pytest.approx(
            case.target_distance
        )
        for metric_key, metric_value in metrics[case.label].items():
            serialized = case_payload["metrics"][metric_key]
            if isinstance(metric_value, bool):
                assert serialized is bool(metric_value)
            elif isinstance(metric_value, str):
                assert serialized == metric_value
            else:
                assert serialized == pytest.approx(float(metric_value))
                assert capture_metrics[f"{case.key}_{metric_key}"] == pytest.approx(
                    float(metric_value)
                )

    for family_label in ("POINT", "DISTANCE", "RIGID"):
        family_payload = capture_metrics["families"][family_label]
        residual_metric = metrics[f"{family_label} residual"]
        solved_metric = metrics[f"{family_label} solved"]
        expected_ratio = float(residual_metric["residual"]) / max(
            float(solved_metric["residual"]),
            1.0e-12,
        )
        assert family_payload["residual_case"] == f"{family_label} residual"
        assert family_payload["solved_case"] == f"{family_label} solved"
        assert family_payload["residual"] == pytest.approx(
            float(residual_metric["residual"])
        )
        assert family_payload["solved_residual"] == pytest.approx(
            max(float(solved_metric["residual"]), 1.0e-12)
        )
        assert family_payload["residual_ratio"] == pytest.approx(expected_ratio)
        prefix = family_label.lower()
        assert capture_metrics[f"{prefix}_residual_ratio"] == pytest.approx(
            expected_ratio
        )
        assert capture_metrics["history"]["families"][family_label][
            "max_residual_ratio"
        ] == pytest.approx(max(controller._residual_ratio_history[family_label]))
    assert capture_metrics["loop_closure_point_residual_ratio"] == pytest.approx(
        capture_metrics["families"]["POINT"]["residual_ratio"]
    )
    assert capture_metrics["loop_closure_distance_residual_ratio"] == pytest.approx(
        capture_metrics["families"]["DISTANCE"]["residual_ratio"]
    )
    assert capture_metrics["loop_closure_rigid_residual_ratio"] == pytest.approx(
        capture_metrics["families"]["RIGID"]["residual_ratio"]
    )
    assert capture_metrics[
        "loop_closure_distance_solved_distance_error"
    ] == pytest.approx(float(metrics["DISTANCE solved"]["distance_error"]))
    assert capture_metrics["loop_closure_distance_solved_tip_error"] == pytest.approx(
        float(metrics["DISTANCE solved"]["tip_error"])
    )
    assert capture_metrics[
        "loop_closure_rigid_residual_orientation_error"
    ] == pytest.approx(float(metrics["RIGID residual"]["orientation_error"]))
    assert capture_metrics[
        "loop_closure_rigid_solved_orientation_error"
    ] == pytest.approx(float(metrics["RIGID solved"]["orientation_error"]))
    assert capture_metrics["loop_closure_max_step_ms"] == pytest.approx(
        capture_metrics["history"]["max_step_ms"]
    )

    assert capture_metrics["history"]["samples"] == pytest.approx(
        max(len(history) for history in controller._residual_ratio_history.values())
    )
    for case in controller.cases:
        history = capture_metrics["history"]["cases"][case.key]
        assert history["samples"] == pytest.approx(
            len(controller._residual_history[case.label])
        )
        assert history["max_residual"] == pytest.approx(
            max(controller._residual_history[case.label])
        )
        assert history["max_distance_error"] == pytest.approx(
            max(controller._distance_error_history[case.label])
        )
        assert history["max_orientation_error"] == pytest.approx(
            max(controller._orientation_error_history[case.label])
        )
        assert history["max_joint_speed"] == pytest.approx(
            max(controller._joint_speed_history[case.label])
        )
        assert history["max_step_ms"] == pytest.approx(
            max(controller._step_ms_history[case.label])
        )
    timeline = setup.info["replay_timeline"]
    snapshot = controller.capture_replay_state()
    latest_ratio = max(
        controller._residual_ratio_history[family_label][-1]
        for family_label in ("POINT", "DISTANCE", "RIGID")
    )
    assert timeline["signal_label"] == "Max closure residual ratio"
    assert timeline["signal"](snapshot) == pytest.approx(latest_ratio)
    assert timeline["signal"](
        {
            "point_residual_ratio": 14.0,
            "distance_residual_ratio": 22.0,
            "rigid_residual_ratio": 18.0,
        }
    ) == pytest.approx(22.0)
    assert timeline["signal"](
        {
            "last_metrics": {
                "POINT residual": {"residual": 0.90},
                "POINT solved": {"residual": 0.03},
                "DISTANCE residual": {"residual": 0.50},
                "DISTANCE solved": {"residual": 0.05},
                "RIGID residual": {"residual": 0.60},
                "RIGID solved": {"residual": 0.12},
            }
        }
    ) == pytest.approx(30.0)
    assert timeline["markers"](
        {"residual_ratio_history": {"POINT": [1.0e9]}}
    ) == pytest.approx(1.0)
    assert (
        timeline["markers"](
            {
                "residual_history": {
                    "POINT residual": [0.90],
                    "POINT solved": [1.0e-9],
                }
            }
        )
        == pytest.approx(1.0)
    )
    assert (
        timeline["markers"](
            {"orientation_error_history": {"RIGID residual": [0.20]}}
        )
        == pytest.approx(1.0)
    )
    assert (
        timeline["markers"](
            {
                "last_metrics": {
                    "DISTANCE solved": {
                        "distance_error": 1.0e-9,
                        "tip_error": 0.20,
                    }
                }
            }
        )
        == pytest.approx(1.0)
    )
    assert (
        timeline["markers"](
            {
                "residual_ratio_history": {
                    "POINT": [10.0],
                    "DISTANCE": [12.0],
                    "RIGID": [11.0],
                },
                "residual_history": {
                    "POINT residual": [0.20],
                    "POINT solved": [0.10],
                },
                "orientation_error_history": {"RIGID residual": [0.02]},
                "last_metrics": {
                    "DISTANCE solved": {
                        "distance_error": 0.02,
                        "tip_error": 0.04,
                    }
                },
            }
        )
        == pytest.approx(0.0)
    )


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
    assert np.isfinite(
        [float(value) for value in controller._step_ms_history["semi_residual"]]
    ).all()
    timeline = setup.info["replay_timeline"]
    snapshot = controller.capture_replay_state()
    assert timeline["signal_label"] == "Residual solve ratio"
    assert timeline["signal"](snapshot) == pytest.approx(
        controller._solve_ratio_history[-1]
    )
    assert timeline["signal"]({"residual_solve_ratio": 42.0}) == pytest.approx(42.0)
    assert timeline["signal"](
        {
            "last_metrics": {
                "semi_residual": {"residual": 0.35},
                "variational_residual": {"residual": 0.62},
                "variational_solved": {"residual": 0.02},
            }
        }
    ) == pytest.approx(31.0)
    assert timeline["markers"]({"solve_ratio_history": [1.0e9]}) == pytest.approx(
        1.0
    )
    assert (
        timeline["markers"](
            {
                "residual_history": {
                    "semi_residual": [0.10],
                    "variational_residual": [0.65],
                    "variational_solved": [0.02],
                }
            }
        )
        == pytest.approx(1.0)
    )
    assert (
        timeline["markers"](
            {
                "tip_error_history": {
                    "semi_residual": [0.30],
                    "variational_residual": [0.22],
                    "variational_solved": [1.0e-9],
                }
            }
        )
        == pytest.approx(1.0)
    )
    assert (
        timeline["markers"](
            {
                "last_metrics": {
                    "semi_residual": {"residual": 0.30, "tip_error": 0.30},
                    "variational_residual": {"residual": 0.20, "tip_error": 0.20},
                    "variational_solved": {
                        "residual": 1.0e-9,
                        "tip_error": 1.0e-9,
                    },
                }
            }
        )
        == pytest.approx(1.0)
    )
    assert (
        timeline["markers"](
            {
                "residual_history": {
                    "semi_residual": [0.12],
                    "variational_residual": [0.15],
                    "variational_solved": [0.10],
                },
                "solve_ratio_history": [10.0],
                "tip_error_history": {
                    "semi_residual": [0.12],
                    "variational_residual": [0.15],
                    "variational_solved": [0.10],
                },
            }
        )
        == pytest.approx(0.0)
    )

    assert callable(setup.info[CAPTURE_METRICS_INFO_KEY])
    capture_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert capture_metrics["row"] == "rigid_multibody_solver_family"
    assert (
        capture_metrics["comparison_axis"]
        == "multibody_integration_solve_policy_family"
    )
    assert capture_metrics["solver"] == "world_multibody_integration_family"
    assert capture_metrics["scope"] == "multibody_closure_solve_routing"
    assert capture_metrics["held_fixed"] == {
        "solver": "world_multibody_integration_family",
        "contacts": "off",
        "closure_family": "point",
        "joint_family": "three_revolute_links",
        "chain_links": 3,
        "link_length": pytest.approx(0.55),
        "link_mass": pytest.approx(0.55),
        "initial_bend": pytest.approx(0.28),
        "gravity_scale": pytest.approx(controller.gravity_scale),
        "time_step_ms": pytest.approx(capture_metrics["time_step_ms"]),
    }
    assert capture_metrics["executor"] == controller._executor_label()
    assert capture_metrics["time_step_ms"] == pytest.approx(5.0)
    assert capture_metrics["world_time"] == pytest.approx(
        controller.primary_world.time
    )
    assert capture_metrics["gravity_scale"] == pytest.approx(
        controller.gravity_scale
    )
    assert capture_metrics["controls"]["executor_index"] == pytest.approx(
        controller.executor_index
    )
    assert capture_metrics["controls"]["gravity_scale"] == pytest.approx(
        controller.gravity_scale
    )
    assert capture_metrics["case_order"] == [case.key for case in controller.cases]
    assert capture_metrics["solver_family_lanes"] == [
        case.key for case in controller.cases
    ]
    assert capture_metrics["case_count"] == pytest.approx(len(controller.cases))
    assert set(capture_metrics["cases"]) == set(metrics)
    for case in controller.cases:
        case_payload = capture_metrics["cases"][case.key]
        assert case_payload["label"] == case.label
        assert case_payload["integration_family"] == case.integration_family
        assert case_payload["dynamic_solve"] is (
            case.dynamics == sx.ClosureDynamicsPolicy.SOLVE
        )
        assert case_payload["closure"] == case.closure.name
        assert case_payload["target_tip"] == pytest.approx(case.target_tip)
        for metric_key, metric_value in metrics[case.key].items():
            serialized = case_payload["metrics"][metric_key]
            if isinstance(metric_value, bool):
                assert serialized is bool(metric_value)
            elif isinstance(metric_value, str):
                assert serialized == metric_value
            else:
                assert serialized == pytest.approx(float(metric_value))
                assert capture_metrics[f"{case.key}_{metric_key}"] == pytest.approx(
                    float(metric_value)
                )

    assert capture_metrics["residual_only_residual"] == pytest.approx(
        max(float(semi["residual"]), float(variational_residual["residual"]))
    )
    assert capture_metrics["solved_residual"] == pytest.approx(
        max(float(variational_solved["residual"]), 1.0e-12)
    )
    assert capture_metrics["residual_solve_ratio"] == pytest.approx(
        controller._solve_ratio_history[-1]
    )
    assert capture_metrics["multibody_solver_residual_only_residual"] == pytest.approx(
        max(float(semi["residual"]), float(variational_residual["residual"]))
    )
    assert capture_metrics["multibody_solver_solved_residual"] == pytest.approx(
        max(float(variational_solved["residual"]), 1.0e-12)
    )
    assert capture_metrics["multibody_solver_residual_solve_ratio"] == pytest.approx(
        controller._solve_ratio_history[-1]
    )
    assert capture_metrics["multibody_solver_semi_residual"] == pytest.approx(
        float(semi["residual"])
    )
    assert capture_metrics["multibody_solver_variational_residual"] == pytest.approx(
        float(variational_residual["residual"])
    )
    assert capture_metrics["multibody_solver_solved_tip_error"] == pytest.approx(
        float(variational_solved["tip_error"])
    )
    assert capture_metrics["multibody_solver_max_step_ms"] == pytest.approx(
        capture_metrics["history"]["max_step_ms"]
    )
    assert capture_metrics["history"]["samples"] == pytest.approx(
        len(controller._solve_ratio_history)
    )
    assert capture_metrics["history"]["max_residual_solve_ratio"] == pytest.approx(
        max(controller._solve_ratio_history)
    )
    for case in controller.cases:
        history = capture_metrics["history"]["cases"][case.key]
        assert history["samples"] == pytest.approx(
            len(controller._residual_history[case.key])
        )
        assert history["max_residual"] == pytest.approx(
            max(controller._residual_history[case.key])
        )
        assert history["max_tip_error"] == pytest.approx(
            max(controller._tip_error_history[case.key])
        )
        assert history["max_joint_speed"] == pytest.approx(
            max(controller._joint_speed_history[case.key])
        )
        assert history["max_step_ms"] == pytest.approx(
            max(controller._step_ms_history[case.key])
        )


def test_avbd_empty_baseline_demo_steps_empty_world() -> None:
    sx = _require_simulation_experimental_symbols("World", "MultibodyOptions")

    from examples.demos.scenes.avbd_empty_baseline import build

    setup = build()
    sx_world = setup.info["sx_world"]

    assert sx_world.num_rigid_bodies == 0
    assert sx_world.num_multibodies == 0
    assert sx_world.num_articulated_joints == 0
    assert sx_world.multibody_options.integration_family == "variational integrator"
    assert setup.info["source_demo_rows"] == (
        "avbd-demo2d empty",
        "avbd-demo3d empty",
    )
    assert sx_world.time_step == pytest.approx(1.0 / 60.0)
    assert sx_world.gravity.tolist() == pytest.approx([0.0, 0.0, -10.0])

    reference_rows = setup.info["source_demo_reference_rows"]
    assert [row["demo"] for row in reference_rows] == ["avbd-demo2d", "avbd-demo3d"]
    assert [row["revision"] for row in reference_rows] == [
        "74699a11f858",
        "7701bd427d55",
    ]
    assert [row["dimension"] for row in reference_rows] == [2, 3]
    assert [row["scene_count"] for row in reference_rows] == [19, 14]
    for row in reference_rows:
        assert row["scene_index"] == 0
        assert row["scene_name"] == "Empty"
        assert row["scene_builder"] == "sceneEmpty"
        assert row["scene_effect"] == "solver->clear()"
        assert row["expected_counts"] == {
            "rigid_bodies": 0,
            "joints": 0,
            "springs": 0,
            "motors": 0,
        }
        assert row["solver_defaults"]["time_step"] == pytest.approx(1.0 / 60.0)
        assert row["solver_defaults"]["gravity"] == pytest.approx(-10.0)
        assert row["solver_defaults"]["iterations"] == 10

    for _ in range(8):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    assert sx_world.time == pytest.approx(8.0 * sx_world.time_step)


def test_avbd_demo2d_motor_scene_matches_source_row() -> None:
    import numpy as np

    sx = _require_simulation_experimental_symbols("World", "ActuatorType")

    from examples.demos.scenes.avbd_demo2d_motor import build

    setup = build()
    sx_world = setup.info["sx_world"]
    bar = setup.info["bar"]
    joint = setup.info["joint"]
    source = setup.info["source_demo_reference"]

    assert setup.info["source_demo_row"] == "avbd-demo2d motor"
    assert source["demo"] == "avbd-demo2d"
    assert source["revision"] == "74699a11f858"
    assert source["scene_index"] == 17
    assert source["scene_name"] == "Motor"
    assert source["scene_builder"] == "sceneMotor"
    assert source["scene_count"] == 19
    assert source["expected_counts"] == {
        "rigid_bodies": 2,
        "joints": 1,
        "motors": 1,
    }
    assert source["source_constraints"]["motor"] == {
        "body_a": None,
        "body_b": "bar",
        "speed": 20.0,
        "max_torque": 50.0,
    }
    assert sx_world.num_rigid_bodies == 2
    assert sx_world.num_rigid_body_joints == 1
    assert sx_world.time_step == pytest.approx(1.0 / 60.0)
    assert sx_world.gravity.tolist() == pytest.approx([0.0, -10.0, 0.0])
    assert joint.actuator_type == sx.ActuatorType.VELOCITY
    assert np.asarray(joint.command_velocity, dtype=float).reshape(1)[0] == pytest.approx(
        20.0
    )
    assert np.asarray(joint.effort_lower_limits, dtype=float).reshape(1)[0] == pytest.approx(
        -50.0
    )
    assert np.asarray(joint.effort_upper_limits, dtype=float).reshape(1)[0] == pytest.approx(
        50.0
    )

    initial_translation = np.asarray(bar.translation, dtype=float).reshape(3)
    initial_speed = float(np.asarray(bar.angular_velocity, dtype=float).reshape(3)[2])
    for _ in range(24):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    final_translation = np.asarray(bar.translation, dtype=float).reshape(3)
    measured_speed = float(np.asarray(bar.angular_velocity, dtype=float).reshape(3)[2])
    assert sx_world.time == pytest.approx(24.0 * sx_world.time_step)
    assert final_translation == pytest.approx(initial_translation, abs=2.0e-5)
    assert measured_speed > initial_speed + 0.05


def test_avbd_demo2d_dynamic_friction_scene_matches_source_row() -> None:
    import numpy as np

    _require_simulation_experimental_symbols("World")

    from examples.demos.scenes.avbd_demo2d_dynamic_friction import build

    setup = build()
    sx_world = setup.info["sx_world"]
    ground = setup.info["ground"]
    boxes = setup.info["boxes"]
    source = setup.info["source_demo_reference"]

    assert setup.info["source_demo_row"] == "avbd-demo2d dynamic friction"
    assert source["demo"] == "avbd-demo2d"
    assert source["revision"] == "74699a11f858"
    assert source["scene_index"] == 2
    assert source["scene_name"] == "Dynamic Friction"
    assert source["scene_builder"] == "sceneDynamicFriction"
    assert source["scene_count"] == 19
    assert source["expected_counts"] == {
        "rigid_bodies": 12,
        "dynamic_bodies": 11,
        "static_bodies": 1,
        "joints": 0,
        "collision_shapes": 12,
    }
    assert source["source_shapes"]["ground"] == {
        "size": (100.0, 1.0),
        "density": 0.0,
        "friction": 0.5,
    }
    assert source["source_shapes"]["boxes"] == {
        "count": 11,
        "size": (1.0, 0.5),
        "density": 1.0,
        "friction_range": (5.0, 0.0),
        "first_position": (-30.0, 0.75, 0.0),
        "x_spacing": 2.0,
        "initial_velocity": (10.0, 0.0, 0.0),
    }
    assert sx_world.num_rigid_bodies == 12
    assert sx_world.num_rigid_body_fixed_joints == 0
    assert sx_world.time_step == pytest.approx(1.0 / 60.0)
    assert sx_world.gravity.tolist() == pytest.approx([0.0, -10.0, 0.0])
    assert len(ground.collision_shapes) == 1
    assert len(boxes) == 11
    assert sum(len(box.collision_shapes) for box in boxes) == 11
    assert [box.friction for box in boxes] == pytest.approx(
        [5.0 - float(index) / 10.0 * 5.0 for index in range(11)]
    )
    initial_positions = np.array(
        [np.asarray(box.translation, dtype=float).reshape(3) for box in boxes]
    )
    assert initial_positions[:, 0].tolist() == pytest.approx(
        [-30.0 + 2.0 * float(index) for index in range(11)]
    )
    assert initial_positions[:, 1].tolist() == pytest.approx([0.75] * 11)
    assert [
        np.asarray(box.linear_velocity, dtype=float).reshape(3)[0] for box in boxes
    ] == pytest.approx([10.0] * 11)
    assert len(sx_world.collide()) >= 11

    high_friction_box = boxes[0]
    zero_friction_box = boxes[-1]
    initial_high = np.asarray(high_friction_box.translation, dtype=float).reshape(3)
    initial_zero = np.asarray(zero_friction_box.translation, dtype=float).reshape(3)

    for _ in range(60):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    final_high = np.asarray(high_friction_box.translation, dtype=float).reshape(3)
    final_zero = np.asarray(zero_friction_box.translation, dtype=float).reshape(3)
    high_velocity = np.asarray(
        high_friction_box.linear_velocity, dtype=float
    ).reshape(3)
    zero_velocity = np.asarray(
        zero_friction_box.linear_velocity, dtype=float
    ).reshape(3)
    high_speed = float(np.linalg.norm(high_velocity[:2]))
    zero_speed = float(np.linalg.norm(zero_velocity[:2]))
    assert sx_world.time == pytest.approx(60.0 * sx_world.time_step)
    assert np.isfinite(final_high).all()
    assert np.isfinite(final_zero).all()
    assert final_high[0] > initial_high[0]
    assert final_zero[0] > initial_zero[0]
    assert final_high[0] < final_zero[0] - 4.0
    assert final_high[1] == pytest.approx(0.75, abs=1.0e-2)
    assert final_zero[1] == pytest.approx(0.75, abs=1.0e-2)
    assert high_speed < 1.0
    assert zero_speed > 9.0
    assert len(sx_world.collide()) >= 11


def test_avbd_demo2d_dynamic_friction_scene_max_friction_env(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    import numpy as np

    _require_simulation_experimental_symbols("World")

    from examples.demos.scenes.avbd_demo2d_dynamic_friction import build

    monkeypatch.setenv(
        "DART_AVBD_DEMO2D_DYNAMIC_FRICTION_MAX_FRICTION",
        "2.5",
    )
    setup = build()
    boxes = setup.info["boxes"]
    source = setup.info["source_demo_reference"]

    class _PanelBuilder:
        def __init__(self) -> None:
            self.plots: list[str] = []

        def text(self, value: str) -> None:
            pass

        def plot_lines(self, label: str, values: list[float]) -> None:
            self.plots.append(label)

        def separator(self) -> None:
            pass

        def checkbox(self, label: str, value: bool) -> tuple[bool, bool]:
            return False, value

        def slider(
            self, label: str, value: float, minimum: float, maximum: float
        ) -> tuple[bool, float]:
            return False, value

        def item_tooltip(self, text: str) -> None:
            pass

        def same_line(self) -> None:
            pass

        def button(self, label: str) -> bool:
            return False

    panel_builder = _PanelBuilder()
    setup.panels[0].build(panel_builder, object())

    assert setup.info["max_dynamic_box_friction"] == pytest.approx(2.5)
    assert setup.info["high_friction_speed_label"] == "Friction 2.5 speed"
    assert source["source_shapes"]["boxes"]["friction_range"] == (2.5, 0.0)
    assert source["parameters"]["max_dynamic_box_friction"] == pytest.approx(2.5)
    assert [box.friction for box in boxes] == pytest.approx(
        [2.5 - float(index) / 10.0 * 2.5 for index in range(11)]
    )
    assert np.asarray(boxes[0].linear_velocity, dtype=float).tolist() == pytest.approx(
        [10.0, 0.0, 0.0]
    )
    assert "Friction 2.5 speed" in panel_builder.plots
    assert "Friction 5 speed" not in panel_builder.plots


def test_avbd_demo2d_static_friction_scene_matches_source_row() -> None:
    import numpy as np

    _require_simulation_experimental_symbols("World")

    from examples.demos.scenes.avbd_demo2d_static_friction import build

    setup = build()
    sx_world = setup.info["sx_world"]
    ground = setup.info["ground"]
    boxes = setup.info["boxes"]
    tangent = np.asarray(setup.info["ramp_tangent"], dtype=float).reshape(3)
    source = setup.info["source_demo_reference"]

    ramp_angle = 3.14159 / 6.0
    expected_rotation = np.array(
        [
            [np.cos(ramp_angle), -np.sin(ramp_angle), 0.0],
            [np.sin(ramp_angle), np.cos(ramp_angle), 0.0],
            [0.0, 0.0, 1.0],
        ]
    )

    assert setup.info["source_demo_row"] == "avbd-demo2d static friction"
    assert source["demo"] == "avbd-demo2d"
    assert source["revision"] == "74699a11f858"
    assert source["scene_index"] == 3
    assert source["scene_name"] == "Static Friction"
    assert source["scene_builder"] == "sceneStaticFriction"
    assert source["scene_count"] == 19
    assert source["expected_counts"] == {
        "rigid_bodies": 12,
        "dynamic_bodies": 11,
        "static_bodies": 1,
        "joints": 0,
        "collision_shapes": 12,
    }
    assert source["source_shapes"]["ground"] == {
        "size": (100.0, 1.0),
        "density": 0.0,
        "friction": 1.0,
        "position": (0.0, 0.0, ramp_angle),
    }
    assert source["source_shapes"]["boxes"] == {
        "count": 11,
        "size": (5.0, 0.5),
        "density": 1.0,
        "friction": 1.0,
        "first_position": (0.0, 1.0, ramp_angle),
        "y_spacing": 1.0,
    }
    assert sx_world.num_rigid_bodies == 12
    assert sx_world.num_rigid_body_fixed_joints == 0
    assert sx_world.time_step == pytest.approx(1.0 / 60.0)
    assert sx_world.gravity.tolist() == pytest.approx([0.0, -10.0, 0.0])
    assert len(ground.collision_shapes) == 1
    assert len(boxes) == 11
    assert sum(len(box.collision_shapes) for box in boxes) == 11
    assert ground.friction == pytest.approx(1.0)
    assert [box.friction for box in boxes] == pytest.approx([1.0] * 11)
    assert np.asarray(ground.rotation, dtype=float).reshape(3, 3) == pytest.approx(
        expected_rotation
    )
    assert np.asarray(boxes[0].rotation, dtype=float).reshape(3, 3) == pytest.approx(
        expected_rotation
    )

    initial_positions = np.array(
        [np.asarray(box.translation, dtype=float).reshape(3) for box in boxes]
    )
    assert initial_positions[:, 0].tolist() == pytest.approx([0.0] * 11)
    assert initial_positions[:, 1].tolist() == pytest.approx(
        [float(index) + 1.0 for index in range(11)]
    )

    low_box = boxes[0]
    high_box = boxes[-1]
    initial_low = np.asarray(low_box.translation, dtype=float).reshape(3)
    initial_high = np.asarray(high_box.translation, dtype=float).reshape(3)

    for _ in range(60):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    final_low = np.asarray(low_box.translation, dtype=float).reshape(3)
    final_high = np.asarray(high_box.translation, dtype=float).reshape(3)
    low_slide = float(np.dot(final_low - initial_low, tangent))
    high_slide = float(np.dot(final_high - initial_high, tangent))
    low_speed = float(
        np.linalg.norm(np.asarray(low_box.linear_velocity, dtype=float).reshape(3))
    )
    high_speed = float(
        np.linalg.norm(np.asarray(high_box.linear_velocity, dtype=float).reshape(3))
    )
    assert sx_world.time == pytest.approx(60.0 * sx_world.time_step)
    assert np.isfinite(final_low).all()
    assert np.isfinite(final_high).all()
    assert high_slide < low_slide - 1.0
    assert high_speed > low_speed + 1.0
    assert len(sx_world.collide()) >= 11


def test_avbd_demo2d_pyramid_scene_matches_source_row() -> None:
    import numpy as np

    _require_simulation_experimental_symbols("World")

    from examples.demos.scenes.avbd_demo2d_pyramid import build

    setup = build()
    sx_world = setup.info["sx_world"]
    ground = setup.info["ground"]
    boxes = setup.info["boxes"]
    source = setup.info["source_demo_reference"]

    assert setup.info["source_demo_row"] == "avbd-demo2d pyramid"
    assert source["demo"] == "avbd-demo2d"
    assert source["revision"] == "74699a11f858"
    assert source["scene_index"] == 4
    assert source["scene_name"] == "Pyramid"
    assert source["scene_builder"] == "scenePyramid"
    assert source["scene_count"] == 19
    assert source["expected_counts"] == {
        "rigid_bodies": 211,
        "dynamic_bodies": 210,
        "static_bodies": 1,
        "joints": 0,
        "collision_shapes": 211,
    }
    assert source["source_shapes"]["ground"] == {
        "size": (100.0, 0.5),
        "density": 0.0,
        "friction": 0.5,
        "position": (0.0, -2.0, 0.0),
    }
    assert source["source_shapes"]["boxes"] == {
        "pyramid_size": 20,
        "count": 210,
        "size": (1.0, 0.5),
        "density": 1.0,
        "friction": 0.5,
        "x_spacing": 1.1,
        "row_x_offset": 0.5,
        "y_spacing": 0.85,
        "base_y": 0.0,
    }
    assert sx_world.num_rigid_bodies == 211
    assert sx_world.num_rigid_body_fixed_joints == 0
    assert sx_world.time_step == pytest.approx(1.0 / 60.0)
    assert sx_world.gravity.tolist() == pytest.approx([0.0, -10.0, 0.0])
    assert len(ground.collision_shapes) == 1
    assert len(boxes) == 210
    assert sum(len(box.collision_shapes) for box in boxes) == 210
    assert ground.friction == pytest.approx(0.5)
    assert [box.friction for box in boxes] == pytest.approx([0.5] * 210)

    initial_positions = np.array(
        [np.asarray(box.translation, dtype=float).reshape(3) for box in boxes]
    )
    assert initial_positions[0].tolist() == pytest.approx([-10.0, 0.0, 0.0])
    assert initial_positions[19].tolist() == pytest.approx([10.9, 0.0, 0.0])
    assert initial_positions[-1].tolist() == pytest.approx([-0.5, 16.15, 0.0])
    assert len(sx_world.collide()) == 0

    top_box = boxes[-1]
    initial_top = np.asarray(top_box.translation, dtype=float).reshape(3)
    for _ in range(60):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    final_top = np.asarray(top_box.translation, dtype=float).reshape(3)
    assert sx_world.time == pytest.approx(60.0 * sx_world.time_step)
    assert np.isfinite(final_top).all()
    assert final_top[1] < initial_top[1] - 4.0
    assert len(sx_world.collide()) >= 100


def test_avbd_demo2d_cards_scene_matches_source_row() -> None:
    import numpy as np

    _require_simulation_experimental_symbols("World")

    from examples.demos.scenes.avbd_demo2d_cards import build

    setup = build()
    sx_world = setup.info["sx_world"]
    ground = setup.info["ground"]
    cards = setup.info["cards"]
    card_specs = setup.info["card_specs"]
    source = setup.info["source_demo_reference"]

    assert setup.info["source_demo_row"] == "avbd-demo2d cards"
    assert source["demo"] == "avbd-demo2d"
    assert source["revision"] == "74699a11f858"
    assert source["scene_index"] == 5
    assert source["scene_name"] == "Cards"
    assert source["scene_builder"] == "sceneCards"
    assert source["scene_count"] == 19
    assert source["expected_counts"] == {
        "rigid_bodies": 41,
        "dynamic_bodies": 40,
        "static_bodies": 1,
        "joints": 0,
        "collision_shapes": 41,
    }
    assert source["source_shapes"]["ground"] == {
        "density": 0.0,
        "friction": 0.7,
        "position": (0.0, -2.0, 0.0),
        "size": (80.0, 4.0),
    }
    cards_shape = source["source_shapes"]["cards"]
    assert cards_shape["count"] == 40
    assert cards_shape["density"] == 1.0
    assert cards_shape["friction"] == 0.7
    assert cards_shape["horizontal_count"] == 10
    assert cards_shape["leaning_count"] == 30
    assert cards_shape["levels"] == 5
    assert cards_shape["size"] == (0.002, 0.4)
    assert cards_shape["angle_horizontal"] == pytest.approx(0.5 * 3.14159)
    assert cards_shape["angle_negative"] == pytest.approx(-25.0 * 3.14159 / 180.0)
    assert cards_shape["angle_positive"] == pytest.approx(25.0 * 3.14159 / 180.0)
    assert cards_shape["first_position"][:2] == pytest.approx((0.25, 0.36))
    assert cards_shape["first_position"][2] == pytest.approx(0.5 * 3.14159)
    assert cards_shape["last_position"][:2] == pytest.approx((0.875, 1.62))
    assert cards_shape["last_position"][2] == pytest.approx(25.0 * 3.14159 / 180.0)
    assert sx_world.num_rigid_bodies == 41
    assert sx_world.num_rigid_body_fixed_joints == 0
    assert sx_world.time_step == pytest.approx(1.0 / 60.0)
    assert sx_world.gravity.tolist() == pytest.approx([0.0, -10.0, 0.0])
    assert ground.is_static
    assert ground.friction == pytest.approx(0.7)
    assert len(ground.collision_shapes) == 1
    assert len(cards) == 40
    assert sum(len(card.collision_shapes) for card in cards) == 40
    assert [card.friction for card in cards] == pytest.approx([0.7] * 40)
    assert [spec["kind"] for spec in card_specs].count("horizontal") == 10
    assert [spec["kind"] for spec in card_specs].count("leaning_negative") == 15
    assert [spec["kind"] for spec in card_specs].count("leaning_positive") == 15

    initial_positions = np.array(
        [np.asarray(card.translation, dtype=float).reshape(3) for card in cards]
    )
    assert initial_positions[0].tolist() == pytest.approx([0.25, 0.36, 0.0])
    assert initial_positions[-1].tolist() == pytest.approx([0.875, 1.62, 0.0])

    for _ in range(12):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    final_positions = np.array(
        [np.asarray(card.translation, dtype=float).reshape(3) for card in cards]
    )
    assert sx_world.time == pytest.approx(12.0 * sx_world.time_step)
    assert np.isfinite(final_positions).all()
    assert len(sx_world.collide()) >= 1


def test_avbd_demo2d_ground_scene_matches_source_row() -> None:
    import numpy as np

    _require_simulation_experimental_symbols("World")

    from examples.demos.scenes.avbd_demo2d_ground import build

    setup = build()
    sx_world = setup.info["sx_world"]
    ground = setup.info["ground"]
    source = setup.info["source_demo_reference"]

    assert setup.info["source_demo_row"] == "avbd-demo2d ground"
    assert source["demo"] == "avbd-demo2d"
    assert source["revision"] == "74699a11f858"
    assert source["scene_index"] == 1
    assert source["scene_name"] == "Ground"
    assert source["scene_builder"] == "sceneGround"
    assert source["scene_count"] == 19
    assert source["expected_counts"] == {
        "rigid_bodies": 1,
        "dynamic_bodies": 0,
        "static_bodies": 1,
        "joints": 0,
        "collision_shapes": 1,
    }
    assert source["source_shapes"]["ground"] == {
        "size": (100.0, 1.0),
        "density": 0.0,
        "friction": 0.5,
        "position": (0.0, 0.0, 0.0),
    }
    assert sx_world.num_rigid_bodies == 1
    assert sx_world.num_rigid_body_fixed_joints == 0
    assert sx_world.time_step == pytest.approx(1.0 / 60.0)
    assert sx_world.gravity.tolist() == pytest.approx([0.0, -10.0, 0.0])
    assert ground.is_static
    assert ground.friction == pytest.approx(0.5)
    assert len(ground.collision_shapes) == 1
    initial_position = np.asarray(ground.translation, dtype=float).reshape(3)
    assert initial_position.tolist() == pytest.approx([0.0, 0.0, 0.0])
    assert len(sx_world.collide()) == 0

    for _ in range(12):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    final_position = np.asarray(ground.translation, dtype=float).reshape(3)
    assert sx_world.time == pytest.approx(12.0 * sx_world.time_step)
    assert final_position.tolist() == pytest.approx(initial_position.tolist())
    assert len(sx_world.collide()) == 0


def test_avbd_demo2d_stack_scene_matches_source_row() -> None:
    import numpy as np

    _require_simulation_experimental_symbols("World")

    from examples.demos.scenes.avbd_demo2d_stack import build

    setup = build()
    sx_world = setup.info["sx_world"]
    ground = setup.info["ground"]
    boxes = setup.info["boxes"]
    source = setup.info["source_demo_reference"]

    assert setup.info["source_demo_row"] == "avbd-demo2d stack"
    assert source["demo"] == "avbd-demo2d"
    assert source["revision"] == "74699a11f858"
    assert source["scene_index"] == 11
    assert source["scene_name"] == "Stack"
    assert source["scene_builder"] == "sceneStack"
    assert source["scene_count"] == 19
    assert source["expected_counts"] == {
        "rigid_bodies": 21,
        "dynamic_bodies": 20,
        "static_bodies": 1,
        "joints": 0,
        "collision_shapes": 21,
    }
    assert source["source_shapes"]["ground"] == {
        "density": 0.0,
        "friction": 0.5,
        "position": (0.0, 0.0, 0.0),
        "size": (100.0, 1.0),
    }
    assert source["source_shapes"]["boxes"] == {
        "base_y": 1.0,
        "count": 20,
        "density": 1.0,
        "friction": 0.5,
        "size": (1.0, 1.0),
        "y_spacing": 2.0,
    }
    assert sx_world.num_rigid_bodies == 21
    assert sx_world.num_rigid_body_fixed_joints == 0
    assert sx_world.time_step == pytest.approx(1.0 / 60.0)
    assert sx_world.gravity.tolist() == pytest.approx([0.0, -10.0, 0.0])
    assert len(ground.collision_shapes) == 1
    assert len(boxes) == 20
    assert sum(len(box.collision_shapes) for box in boxes) == 20
    assert [box.friction for box in boxes] == pytest.approx([0.5] * 20)

    initial_positions = np.array(
        [np.asarray(box.translation, dtype=float).reshape(3) for box in boxes]
    )
    assert initial_positions[:, 1].tolist() == pytest.approx(
        [float(index) * 2.0 + 1.0 for index in range(20)]
    )

    for _ in range(30):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    final_positions = np.array(
        [np.asarray(box.translation, dtype=float).reshape(3) for box in boxes]
    )
    assert sx_world.time == pytest.approx(30.0 * sx_world.time_step)
    assert np.isfinite(final_positions).all()
    assert final_positions[-1, 1] < initial_positions[-1, 1] - 0.5
    assert final_positions[:, 1].min() > 0.25
    assert len(sx_world.collide()) >= 8


def test_avbd_demo2d_stack_ratio_scene_matches_source_row() -> None:
    import numpy as np

    _require_simulation_experimental_symbols("World")

    from examples.demos.scenes.avbd_demo2d_stack_ratio import build

    setup = build()
    sx_world = setup.info["sx_world"]
    ground = setup.info["ground"]
    boxes = setup.info["boxes"]
    box_sizes = setup.info["box_sizes"]
    source = setup.info["source_demo_reference"]

    assert setup.info["source_demo_row"] == "avbd-demo2d stack ratio"
    assert source["demo"] == "avbd-demo2d"
    assert source["revision"] == "74699a11f858"
    assert source["scene_index"] == 12
    assert source["scene_name"] == "Stack Ratio"
    assert source["scene_builder"] == "sceneStackRatio"
    assert source["scene_count"] == 19
    assert source["expected_counts"] == {
        "rigid_bodies": 7,
        "dynamic_bodies": 6,
        "static_bodies": 1,
        "joints": 0,
        "collision_shapes": 7,
    }
    assert source["source_shapes"]["ground"] == {
        "density": 0.0,
        "friction": 0.5,
        "position": (0.0, 0.0, 0.0),
        "size": (100.0, 1.0),
    }
    assert source["source_shapes"]["boxes"] == {
        "centers_y": (1.0, 2.0, 5.0, 11.0, 23.0, 47.0),
        "count": 6,
        "density": 1.0,
        "friction": 0.5,
        "initial_size": 1.0,
        "size_multiplier": 2.0,
        "sizes": (
            (1.0, 1.0),
            (2.0, 2.0),
            (4.0, 4.0),
            (8.0, 8.0),
            (16.0, 16.0),
            (32.0, 32.0),
        ),
    }
    assert sx_world.num_rigid_bodies == 7
    assert sx_world.num_rigid_body_fixed_joints == 0
    assert sx_world.time_step == pytest.approx(1.0 / 60.0)
    assert sx_world.gravity.tolist() == pytest.approx([0.0, -10.0, 0.0])
    assert len(ground.collision_shapes) == 1
    assert len(boxes) == 6
    assert [len(box.collision_shapes) for box in boxes] == [1] * 6
    assert [box.friction for box in boxes] == pytest.approx([0.5] * 6)
    assert np.array([size.tolist() for size in box_sizes]) == pytest.approx(
        np.array(
            [
                [1.0, 1.0, 0.2],
                [2.0, 2.0, 0.2],
                [4.0, 4.0, 0.2],
                [8.0, 8.0, 0.2],
                [16.0, 16.0, 0.2],
                [32.0, 32.0, 0.2],
            ]
        )
    )

    initial_positions = np.array(
        [np.asarray(box.translation, dtype=float).reshape(3) for box in boxes]
    )
    assert initial_positions[:, 1].tolist() == pytest.approx(
        [1.0, 2.0, 5.0, 11.0, 23.0, 47.0]
    )

    for _ in range(30):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    final_positions = np.array(
        [np.asarray(box.translation, dtype=float).reshape(3) for box in boxes]
    )
    assert sx_world.time == pytest.approx(30.0 * sx_world.time_step)
    assert np.isfinite(final_positions).all()
    assert final_positions[-1, 1] < initial_positions[-1, 1] - 0.5
    assert final_positions[:, 1].min() > 0.25
    assert len(sx_world.collide()) >= 5


def test_avbd_demo2d_rod_scene_matches_source_row() -> None:
    import numpy as np

    sx = _require_simulation_experimental_symbols("World")

    from examples.demos.scenes.avbd_demo2d_rod import build

    setup = build()
    sx_world = setup.info["sx_world"]
    links = setup.info["links"]
    link_sizes = setup.info["link_sizes"]
    joints = setup.info["joints"]
    source = setup.info["source_demo_reference"]

    assert setup.info["source_demo_row"] == "avbd-demo2d rod"
    assert source["demo"] == "avbd-demo2d"
    assert source["revision"] == "74699a11f858"
    assert source["scene_index"] == 13
    assert source["scene_name"] == "Rod"
    assert source["scene_builder"] == "sceneRod"
    assert source["scene_count"] == 19
    assert source["expected_counts"] == {
        "rigid_bodies": 20,
        "dynamic_bodies": 19,
        "static_bodies": 1,
        "joints": 19,
        "fixed_joints": 19,
        "collision_shapes": 20,
    }
    assert source["source_shapes"]["rod_link"] == {
        "count": 20,
        "dynamic_density": 1.0,
        "first_position": (0.0, 10.0, 0.0),
        "friction": 0.5,
        "size": (1.0, 0.5),
        "static_anchor_density": 0.0,
        "x_spacing": 1.0,
    }
    assert source["source_constraints"]["all_axis_fixed_joints"] == {
        "angular_stiffness": "infinity",
        "child_anchor": (-0.5, 0.0),
        "count": 19,
        "fracture": "infinity",
        "linear_stiffness": "infinity",
        "parent_anchor": (0.5, 0.0),
    }
    assert sx_world.num_rigid_bodies == 20
    assert sx_world.num_rigid_body_joints == 19
    assert sx_world.num_rigid_body_fixed_joints == 19
    assert sx_world.time_step == pytest.approx(1.0 / 60.0)
    assert sx_world.gravity.tolist() == pytest.approx([0.0, -10.0, 0.0])
    assert len(links) == 20
    assert len(link_sizes) == 20
    assert len(joints) == 19
    assert all(joint.type == sx.JointType.FIXED for joint in joints)
    assert [joint.num_dofs for joint in joints] == [0] * 19
    assert [len(link.collision_shapes) for link in links] == [1] * 20
    assert [link.friction for link in links] == pytest.approx([0.5] * 20)
    assert links[0].is_static
    assert not any(link.is_static for link in links[1:])
    assert np.array([size.tolist() for size in link_sizes]) == pytest.approx(
        np.tile(np.array([[1.0, 0.5, 0.2]]), (20, 1))
    )

    initial_positions = np.array(
        [np.asarray(link.translation, dtype=float).reshape(3) for link in links]
    )
    assert initial_positions[:, 0].tolist() == pytest.approx(
        [float(index) for index in range(20)]
    )
    assert initial_positions[:, 1].tolist() == pytest.approx([10.0] * 20)

    def fixed_pose_errors() -> list[float]:
        errors = []
        captured_offset = np.array([1.0, 0.0, 0.0])
        for parent, child in zip(links, links[1:]):
            parent_translation = np.asarray(parent.translation, dtype=float).reshape(3)
            child_translation = np.asarray(child.translation, dtype=float).reshape(3)
            parent_rotation = np.asarray(parent.rotation, dtype=float).reshape(3, 3)
            child_rotation = np.asarray(child.rotation, dtype=float).reshape(3, 3)
            translation_error = np.linalg.norm(
                child_translation
                - parent_translation
                - parent_rotation @ captured_offset
            )
            rotation_error = 0.25 * np.linalg.norm(child_rotation - parent_rotation)
            errors.append(float(translation_error + rotation_error))
        return errors

    assert max(fixed_pose_errors()) < 1.0e-12
    for _ in range(20):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    final_positions = np.array(
        [np.asarray(link.translation, dtype=float).reshape(3) for link in links]
    )
    assert sx_world.time == pytest.approx(20.0 * sx_world.time_step)
    assert np.isfinite(final_positions).all()
    assert final_positions[-1, 1] == pytest.approx(initial_positions[-1, 1], abs=0.1)
    assert max(fixed_pose_errors()) < 0.1


def test_avbd_demo2d_joint_grid_scene_matches_source_row() -> None:
    import numpy as np

    sx = _require_simulation_experimental_symbols("World")

    from examples.demos.scenes.avbd_demo2d_joint_grid import build

    setup = build()
    sx_world = setup.info["sx_world"]
    cells = setup.info["cells"]
    grid = setup.info["cell_grid"]
    cell_size = setup.info["cell_size"]
    joints = setup.info["joints"]
    horizontal_joints = setup.info["horizontal_joints"]
    vertical_joints = setup.info["vertical_joints"]
    ignored_collision_pairs = setup.info["ignored_collision_pairs"]
    source = setup.info["source_demo_reference"]

    assert setup.info["source_demo_row"] == "avbd-demo2d joint grid"
    assert source["demo"] == "avbd-demo2d"
    assert source["revision"] == "74699a11f858"
    assert source["scene_index"] == 15
    assert source["scene_name"] == "Joint Grid"
    assert source["scene_builder"] == "sceneJointGrid"
    assert source["scene_count"] == 19
    assert source["expected_counts"] == {
        "rigid_bodies": 625,
        "dynamic_bodies": 623,
        "static_bodies": 2,
        "joints": 1200,
        "fixed_joints": 1200,
        "collision_shapes": 625,
    }
    assert source["source_shapes"]["grid_cell"] == {
        "count": 625,
        "dynamic_density": 1.0,
        "first_position": (0.0, 0.0, 0.0),
        "friction": 0.5,
        "height": 25,
        "size": (1.0, 1.0),
        "static_corner_density": 0.0,
        "static_corner_positions": ((0.0, 24.0, 0.0), (24.0, 24.0, 0.0)),
        "width": 25,
    }
    assert source["source_constraints"]["all_axis_fixed_joints"] == {
        "angular_stiffness": "infinity",
        "count": 1200,
        "fracture": "infinity",
        "horizontal_child_anchor": (-0.5, 0.0),
        "horizontal_count": 600,
        "horizontal_parent_anchor": (0.5, 0.0),
        "linear_stiffness": "infinity",
        "vertical_child_anchor": (0.0, -0.5),
        "vertical_count": 600,
        "vertical_parent_anchor": (0.0, 0.5),
    }
    assert source["source_collision_filters"] == {
        "diagonal_ignore_collision_pairs": 1152,
    }
    assert sx_world.num_rigid_bodies == 625
    assert sx_world.num_rigid_body_joints == 1200
    assert sx_world.num_rigid_body_fixed_joints == 1200
    assert sx_world.time_step == pytest.approx(1.0 / 60.0)
    assert sx_world.gravity.tolist() == pytest.approx([0.0, -10.0, 0.0])
    assert len(cells) == 625
    assert len(grid) == 25
    assert all(len(column) == 25 for column in grid)
    assert len(horizontal_joints) == 600
    assert len(vertical_joints) == 600
    assert len(joints) == 1200
    assert len(ignored_collision_pairs) == 1152
    assert sx_world.num_ignored_collision_pairs == 1152
    assert sx_world.is_collision_pair_ignored(grid[0][0], grid[1][1])
    assert sx_world.is_collision_pair_ignored(grid[1][0], grid[0][1])
    assert not sx_world.is_collision_pair_ignored(grid[0][0], grid[1][0])
    assert all(joint.type == sx.JointType.FIXED for joint in joints)
    assert [joint.num_dofs for joint in joints[:10]] == [0] * 10
    assert [len(cell.collision_shapes) for cell in cells] == [1] * 625
    assert [cell.friction for cell in cells[:10]] == pytest.approx([0.5] * 10)
    assert grid[0][24].is_static
    assert grid[24][24].is_static
    assert sum(1 for cell in cells if cell.is_static) == 2
    assert cell_size.tolist() == pytest.approx([1.0, 1.0, 0.2])

    initial_positions = np.array(
        [np.asarray(cell.translation, dtype=float).reshape(3) for cell in cells]
    )
    assert np.isfinite(initial_positions).all()
    assert np.asarray(grid[0][0].translation).tolist() == pytest.approx(
        [0.0, 0.0, 0.0]
    )
    assert np.asarray(grid[24][24].translation).tolist() == pytest.approx(
        [24.0, 24.0, 0.0]
    )

    def fixed_pose_errors() -> list[float]:
        errors = []
        for x in range(1, 25):
            for y in range(25):
                parent = grid[x - 1][y]
                child = grid[x][y]
                parent_translation = np.asarray(parent.translation, dtype=float).reshape(3)
                child_translation = np.asarray(child.translation, dtype=float).reshape(3)
                parent_rotation = np.asarray(parent.rotation, dtype=float).reshape(3, 3)
                child_rotation = np.asarray(child.rotation, dtype=float).reshape(3, 3)
                translation_error = np.linalg.norm(
                    child_translation
                    - parent_translation
                    - parent_rotation @ np.array([1.0, 0.0, 0.0])
                )
                rotation_error = 0.25 * np.linalg.norm(child_rotation - parent_rotation)
                errors.append(float(translation_error + rotation_error))
        for x in range(25):
            for y in range(1, 25):
                parent = grid[x][y - 1]
                child = grid[x][y]
                parent_translation = np.asarray(parent.translation, dtype=float).reshape(3)
                child_translation = np.asarray(child.translation, dtype=float).reshape(3)
                parent_rotation = np.asarray(parent.rotation, dtype=float).reshape(3, 3)
                child_rotation = np.asarray(child.rotation, dtype=float).reshape(3, 3)
                translation_error = np.linalg.norm(
                    child_translation
                    - parent_translation
                    - parent_rotation @ np.array([0.0, 1.0, 0.0])
                )
                rotation_error = 0.25 * np.linalg.norm(child_rotation - parent_rotation)
                errors.append(float(translation_error + rotation_error))
        return errors

    assert max(fixed_pose_errors()) < 1.0e-12
    for _ in range(3):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    final_positions = np.array(
        [np.asarray(cell.translation, dtype=float).reshape(3) for cell in cells]
    )
    assert sx_world.time == pytest.approx(3.0 * sx_world.time_step)
    assert np.isfinite(final_positions).all()
    assert np.asarray(grid[0][24].translation).tolist() == pytest.approx(
        [0.0, 24.0, 0.0]
    )
    assert np.asarray(grid[24][24].translation).tolist() == pytest.approx(
        [24.0, 24.0, 0.0]
    )
    assert max(fixed_pose_errors()) < 0.1


def test_avbd_demo2d_soft_body_scene_matches_source_row() -> None:
    import numpy as np

    sx = _require_simulation_experimental_symbols("World")

    from examples.demos.scenes.avbd_demo2d_soft_body import build

    setup = build()
    sx_world = setup.info["sx_world"]
    cells = setup.info["cells"]
    grids = setup.info["cell_grids"]
    cell_size = setup.info["cell_size"]
    joints = setup.info["joints"]
    horizontal_joints = setup.info["horizontal_joints"]
    vertical_joints = setup.info["vertical_joints"]
    ignored_collision_pairs = setup.info["ignored_collision_pairs"]
    source = setup.info["source_demo_reference"]

    assert setup.info["source_demo_row"] == "avbd-demo2d soft body"
    assert source["demo"] == "avbd-demo2d"
    assert source["revision"] == "74699a11f858"
    assert source["scene_index"] == 14
    assert source["scene_name"] == "Soft Body"
    assert source["scene_builder"] == "sceneSoftBody"
    assert source["scene_count"] == 19
    assert source["expected_counts"] == {
        "rigid_bodies": 151,
        "dynamic_bodies": 150,
        "static_bodies": 1,
        "joints": 260,
        "fixed_joints": 260,
        "finite_stiffness_fixed_joints": 260,
        "collision_shapes": 151,
    }
    assert source["source_shapes"]["ground"] == {
        "count": 1,
        "density": 0.0,
        "friction": 0.5,
        "position": (0.0, 0.0, 0.0),
        "size": (100.0, 0.5),
    }
    assert source["source_shapes"]["soft_cell"] == {
        "count": 150,
        "density": 1.0,
        "first_position": (0.0, 5.0, 0.0),
        "friction": 0.5,
        "height": 5,
        "size": (1.0, 1.0),
        "stack_y_spacing": 10.0,
        "stacks": 2,
        "width": 15,
    }
    assert source["source_constraints"]["finite_all_axis_fixed_joints"] == {
        "angular_stiffness": 100.0,
        "count": 260,
        "fracture": "infinity",
        "horizontal_child_anchor": (-0.5, 0.0),
        "horizontal_count": 140,
        "horizontal_parent_anchor": (0.5, 0.0),
        "linear_stiffness": 1000.0,
        "vertical_child_anchor": (0.0, -0.5),
        "vertical_count": 120,
        "vertical_parent_anchor": (0.0, 0.5),
    }
    assert source["source_collision_filters"] == {
        "diagonal_ignore_collision_pairs": 224,
    }
    assert sx_world.num_rigid_bodies == 151
    assert sx_world.num_rigid_body_joints == 260
    assert sx_world.num_rigid_body_fixed_joints == 260
    assert sx_world.time_step == pytest.approx(1.0 / 60.0)
    assert sx_world.gravity.tolist() == pytest.approx([0.0, -10.0, 0.0])
    assert len(cells) == 150
    assert len(grids) == 2
    assert all(len(grid) == 15 for grid in grids)
    assert all(len(column) == 5 for grid in grids for column in grid)
    assert len(horizontal_joints) == 140
    assert len(vertical_joints) == 120
    assert len(joints) == 260
    assert len(ignored_collision_pairs) == 224
    assert sx_world.num_ignored_collision_pairs == 224
    assert sx_world.is_collision_pair_ignored(grids[0][0][0], grids[0][1][1])
    assert sx_world.is_collision_pair_ignored(grids[0][1][0], grids[0][0][1])
    assert not sx_world.is_collision_pair_ignored(grids[0][0][0], grids[0][1][0])
    assert all(joint.type == sx.JointType.FIXED for joint in joints)
    assert [joint.num_dofs for joint in joints[:10]] == [0] * 10
    assert [joint.avbd_linear_stiffness for joint in joints[:10]] == pytest.approx(
        [1000.0] * 10
    )
    assert [joint.avbd_angular_stiffness for joint in joints[:10]] == pytest.approx(
        [100.0] * 10
    )
    assert [len(cell.collision_shapes) for cell in cells] == [1] * 150
    assert [cell.friction for cell in cells[:10]] == pytest.approx([0.5] * 10)
    assert sum(1 for cell in cells if cell.is_static) == 0
    assert setup.info["ground"].is_static
    assert cell_size.tolist() == pytest.approx([1.0, 1.0, 0.2])

    initial_positions = np.array(
        [np.asarray(cell.translation, dtype=float).reshape(3) for cell in cells]
    )
    assert np.isfinite(initial_positions).all()
    assert np.asarray(grids[0][0][0].translation).tolist() == pytest.approx(
        [0.0, 5.0, 0.0]
    )
    assert np.asarray(grids[1][14][4].translation).tolist() == pytest.approx(
        [14.0, 19.0, 0.0]
    )

    for _ in range(3):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    final_positions = np.array(
        [np.asarray(cell.translation, dtype=float).reshape(3) for cell in cells]
    )
    assert sx_world.time == pytest.approx(3.0 * sx_world.time_step)
    assert np.isfinite(final_positions).all()


def test_avbd_demo2d_hanging_rope_scene_matches_source_row() -> None:
    import numpy as np

    sx = _require_simulation_experimental_symbols("World")

    from examples.demos.scenes.avbd_demo2d_hanging_rope import build

    setup = build()
    sx_world = setup.info["sx_world"]
    links = setup.info["links"]
    link_sizes = setup.info["link_sizes"]
    joints = setup.info["joints"]
    source = setup.info["source_demo_reference"]

    assert setup.info["source_demo_row"] == "avbd-demo2d hanging rope"
    assert source["demo"] == "avbd-demo2d"
    assert source["revision"] == "74699a11f858"
    assert source["scene_index"] == 8
    assert source["scene_name"] == "Hanging Rope"
    assert source["scene_builder"] == "sceneHangingRope"
    assert source["scene_count"] == 19
    assert source["expected_counts"] == {
        "rigid_bodies": 50,
        "dynamic_bodies": 49,
        "static_bodies": 1,
        "joints": 49,
        "linear_point_joints": 49,
        "collision_shapes": 50,
    }
    assert source["source_shapes"]["rope_link"] == {
        "count": 49,
        "dynamic_density": 1.0,
        "first_position": (0.0, 10.0, 0.0),
        "friction": 0.5,
        "size": (0.5, 1.0),
        "static_anchor_density": 0.0,
        "y_spacing": -1.0,
    }
    assert source["source_shapes"]["heavy_block"] == {
        "count": 1,
        "density": 1.0,
        "friction": 0.5,
        "position": (0.0, -44.0, 0.0),
        "size": (10.0, 10.0),
    }
    assert source["source_constraints"]["linear_point_joints"] == {
        "angular_stiffness": 0.0,
        "count": 49,
        "fracture": "infinity",
        "heavy_child_anchor": (0.0, 5.0),
        "linear_stiffness": "infinity",
        "parent_anchor": (0.0, -0.5),
        "regular_child_anchor": (0.0, 0.5),
        "source_initial_last_endpoint_gap": 0.5,
    }
    assert sx_world.num_rigid_bodies == 50
    assert sx_world.num_rigid_body_joints == 49
    assert sx_world.num_rigid_body_fixed_joints == 0
    assert sx_world.time_step == pytest.approx(1.0 / 60.0)
    assert sx_world.gravity.tolist() == pytest.approx([0.0, -10.0, 0.0])
    assert len(links) == 50
    assert len(link_sizes) == 50
    assert len(joints) == 49
    assert all(joint.type == sx.JointType.SPHERICAL for joint in joints)
    assert [joint.num_dofs for joint in joints] == [3] * 49
    assert [joint.avbd_start_stiffness for joint in joints] == pytest.approx(
        [1.0e6] * 49
    )
    assert [len(link.collision_shapes) for link in links] == [1] * 50
    assert [link.friction for link in links] == pytest.approx([0.5] * 50)
    assert links[0].is_static
    assert not any(link.is_static for link in links[1:])

    initial_positions = np.array(
        [np.asarray(link.translation, dtype=float).reshape(3) for link in links]
    )
    assert initial_positions[:, 0].tolist() == pytest.approx([0.0] * 50)
    assert initial_positions[:-1, 1].tolist() == pytest.approx(
        [10.0 - float(index) for index in range(49)]
    )
    assert initial_positions[-1, 1] == pytest.approx(-44.0)

    parent_anchor = np.array([0.0, -0.5, 0.0])
    regular_child_anchor = np.array([0.0, 0.5, 0.0])
    heavy_child_anchor = np.array([0.0, 5.0, 0.0])

    def endpoint_errors() -> list[float]:
        errors = []
        for index, (parent, child) in enumerate(zip(links, links[1:])):
            child_anchor = heavy_child_anchor if index == 48 else regular_child_anchor
            parent_point = np.asarray(parent.translation, dtype=float).reshape(
                3
            ) + np.asarray(parent.rotation, dtype=float) @ parent_anchor
            child_point = np.asarray(child.translation, dtype=float).reshape(
                3
            ) + np.asarray(child.rotation, dtype=float) @ child_anchor
            errors.append(float(np.linalg.norm(parent_point - child_point)))
        return errors

    initial_endpoint_errors = endpoint_errors()
    assert max(initial_endpoint_errors[:-1]) < 1.0e-12
    assert initial_endpoint_errors[-1] == pytest.approx(0.5)
    step_count = 240
    for _ in range(step_count):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    final_positions = np.array(
        [np.asarray(link.translation, dtype=float).reshape(3) for link in links]
    )
    assert sx_world.time == pytest.approx(float(step_count) * sx_world.time_step)
    assert np.isfinite(final_positions).all()
    assert final_positions[-1, 1] > initial_positions[-1, 1] + 0.25
    assert max(endpoint_errors()) < 1.0


def test_avbd_demo2d_rope_scene_matches_source_row() -> None:
    import numpy as np

    sx = _require_simulation_experimental_symbols("World")

    from examples.demos.scenes.avbd_demo2d_rope import build

    setup = build()
    sx_world = setup.info["sx_world"]
    links = setup.info["links"]
    joints = setup.info["joints"]
    source = setup.info["source_demo_reference"]

    assert setup.info["source_demo_row"] == "avbd-demo2d rope"
    assert source["demo"] == "avbd-demo2d"
    assert source["revision"] == "74699a11f858"
    assert source["scene_index"] == 6
    assert source["scene_name"] == "Rope"
    assert source["scene_builder"] == "sceneRope"
    assert source["scene_count"] == 19
    assert source["expected_counts"] == {
        "rigid_bodies": 20,
        "dynamic_bodies": 19,
        "static_bodies": 1,
        "joints": 19,
        "linear_point_joints": 19,
        "collision_shapes": 20,
    }
    assert source["source_shapes"]["rope_link"] == {
        "count": 20,
        "dynamic_density": 1.0,
        "first_position": (0.0, 10.0, 0.0),
        "friction": 0.5,
        "size": (1.0, 0.5),
        "static_anchor_density": 0.0,
        "x_spacing": 1.0,
    }
    assert source["source_constraints"]["linear_point_joints"] == {
        "angular_stiffness": 0.0,
        "child_anchor": (-0.5, 0.0),
        "count": 19,
        "fracture": "infinity",
        "linear_stiffness": "infinity",
        "parent_anchor": (0.5, 0.0),
    }
    assert sx_world.num_rigid_bodies == 20
    assert sx_world.num_rigid_body_joints == 19
    assert sx_world.num_rigid_body_fixed_joints == 0
    assert sx_world.time_step == pytest.approx(1.0 / 60.0)
    assert sx_world.gravity.tolist() == pytest.approx([0.0, -10.0, 0.0])
    assert len(links) == 20
    assert len(joints) == 19
    assert all(joint.type == sx.JointType.SPHERICAL for joint in joints)
    assert [joint.num_dofs for joint in joints] == [3] * 19
    assert [len(link.collision_shapes) for link in links] == [1] * 20
    assert [link.friction for link in links] == pytest.approx([0.5] * 20)
    assert links[0].is_static
    assert not any(link.is_static for link in links[1:])

    initial_positions = np.array(
        [np.asarray(link.translation, dtype=float).reshape(3) for link in links]
    )
    assert initial_positions[:, 0].tolist() == pytest.approx(
        [float(index) for index in range(20)]
    )
    assert initial_positions[:, 1].tolist() == pytest.approx([10.0] * 20)

    parent_anchor = np.array([0.5, 0.0, 0.0])
    child_anchor = np.array([-0.5, 0.0, 0.0])

    def endpoint_errors() -> list[float]:
        errors = []
        for parent, child in zip(links, links[1:]):
            parent_point = np.asarray(parent.translation, dtype=float).reshape(
                3
            ) + np.asarray(parent.rotation, dtype=float) @ parent_anchor
            child_point = np.asarray(child.translation, dtype=float).reshape(
                3
            ) + np.asarray(child.rotation, dtype=float) @ child_anchor
            errors.append(float(np.linalg.norm(parent_point - child_point)))
        return errors

    assert max(endpoint_errors()) < 1.0e-12
    for _ in range(20):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    final_positions = np.array(
        [np.asarray(link.translation, dtype=float).reshape(3) for link in links]
    )
    assert sx_world.time == pytest.approx(20.0 * sx_world.time_step)
    assert np.isfinite(final_positions).all()
    assert final_positions[-1, 1] < initial_positions[-1, 1]
    assert max(endpoint_errors()) < 1.0


def test_avbd_demo2d_spring_scene_matches_source_row() -> None:
    import numpy as np

    sx = _require_simulation_experimental_symbols("World")

    from examples.demos.scenes.avbd_demo2d_spring import build

    setup = build()
    sx_world = setup.info["sx_world"]
    anchor = setup.info["anchor"]
    block = setup.info["block"]
    springs = setup.info["springs"]
    ignored_collision_pairs = setup.info["ignored_collision_pairs"]
    spring_length = setup.info["spring_length"]
    source = setup.info["source_demo_reference"]

    assert setup.info["source_demo_row"] == "avbd-demo2d spring"
    assert source["demo"] == "avbd-demo2d"
    assert source["revision"] == "74699a11f858"
    assert source["scene_index"] == 9
    assert source["scene_name"] == "Spring"
    assert source["scene_builder"] == "sceneSpring"
    assert source["scene_count"] == 19
    assert source["expected_counts"] == {
        "rigid_bodies": 2,
        "dynamic_bodies": 1,
        "static_bodies": 1,
        "joints": 0,
        "distance_springs": 1,
        "collision_shapes": 2,
    }
    assert source["source_shapes"]["anchor"] == {
        "density": 0.0,
        "friction": 0.5,
        "position": (0.0, 0.0, 0.0),
        "size": (1.0, 1.0),
    }
    assert source["source_shapes"]["block"] == {
        "density": 1.0,
        "friction": 0.5,
        "position": (0.0, -8.0, 0.0),
        "size": (4.0, 4.0),
    }
    assert source["source_constraints"]["radial_distance_springs"] == {
        "anchor": (0.0, 0.0),
        "block_anchor": (0.0, 0.0),
        "count": 1,
        "rest_length": 4.0,
        "stiffness": 100.0,
    }
    assert sx_world.num_rigid_bodies == 2
    assert sx_world.num_rigid_body_joints == 0
    assert sx_world.time_step == pytest.approx(1.0 / 60.0)
    assert sx_world.gravity.tolist() == pytest.approx([0.0, -10.0, 0.0])
    assert anchor.is_static
    assert not block.is_static
    assert [len(body.collision_shapes) for body in (anchor, block)] == [1, 1]
    assert [body.friction for body in (anchor, block)] == pytest.approx([0.5, 0.5])
    assert len(springs) == 1
    assert len(ignored_collision_pairs) == 1
    assert sx_world.num_ignored_collision_pairs == 1
    assert sx_world.is_collision_pair_ignored(anchor, block)
    assert springs[0]["rest_length"] == pytest.approx(4.0)
    assert springs[0]["stiffness"] == pytest.approx(100.0)

    initial_length = spring_length()
    initial_block_y = float(np.asarray(block.translation, dtype=float).reshape(3)[1])
    assert initial_length == pytest.approx(8.0)

    for _ in range(20):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    final_length = spring_length()
    final_block_y = float(np.asarray(block.translation, dtype=float).reshape(3)[1])
    assert sx_world.time == pytest.approx(20.0 * sx_world.time_step)
    assert np.isfinite(final_length)
    assert final_length < initial_length
    assert final_block_y > initial_block_y


def test_avbd_demo2d_spring_ratio_scene_matches_source_row() -> None:
    import numpy as np

    sx = _require_simulation_experimental_symbols("World")

    from examples.demos.scenes.avbd_demo2d_spring_ratio import build

    setup = build()
    sx_world = setup.info["sx_world"]
    links = setup.info["links"]
    springs = setup.info["springs"]
    ignored_collision_pairs = setup.info["ignored_collision_pairs"]
    spring_lengths = setup.info["spring_lengths"]
    source = setup.info["source_demo_reference"]

    assert setup.info["source_demo_row"] == "avbd-demo2d spring ratio"
    assert source["demo"] == "avbd-demo2d"
    assert source["revision"] == "74699a11f858"
    assert source["scene_index"] == 10
    assert source["scene_name"] == "Spring Ratio"
    assert source["scene_builder"] == "sceneSpringsRatio"
    assert source["scene_count"] == 19
    assert source["expected_counts"] == {
        "rigid_bodies": 8,
        "dynamic_bodies": 6,
        "static_bodies": 2,
        "joints": 0,
        "distance_springs": 7,
        "collision_shapes": 8,
    }
    assert source["source_shapes"]["spring_link"] == {
        "count": 8,
        "dynamic_density": 1.0,
        "first_position": (0.0, 10.0, 0.0),
        "friction": 0.5,
        "size": (1.0, 0.5),
        "static_endpoint_density": 0.0,
        "x_spacing": 4.0,
    }
    assert source["source_constraints"]["radial_distance_springs"] == {
        "child_anchor": (-0.5, 0.0),
        "count": 7,
        "high_stiffness": 1.0e6,
        "high_stiffness_count": 4,
        "low_stiffness": 1.0e3,
        "low_stiffness_count": 3,
        "parent_anchor": (0.5, 0.0),
        "rest_length": 0.1,
    }
    assert sx_world.num_rigid_bodies == 8
    assert sx_world.num_rigid_body_joints == 0
    assert sx_world.time_step == pytest.approx(1.0 / 60.0)
    assert sx_world.gravity.tolist() == pytest.approx([0.0, -10.0, 0.0])
    assert len(links) == 8
    assert len(springs) == 7
    assert len(ignored_collision_pairs) == 7
    assert sx_world.num_ignored_collision_pairs == 7
    assert all(
        sx_world.is_collision_pair_ignored(links[index], links[index + 1])
        for index in range(7)
    )
    assert not sx_world.is_collision_pair_ignored(links[0], links[2])
    assert [link.is_static for link in links] == [
        True,
        False,
        False,
        False,
        False,
        False,
        False,
        True,
    ]
    assert [spring["stiffness"] for spring in springs] == pytest.approx(
        [1.0e6, 1.0e3, 1.0e6, 1.0e3, 1.0e6, 1.0e3, 1.0e6]
    )
    assert [spring["rest_length"] for spring in springs] == pytest.approx([0.1] * 7)
    assert [len(link.collision_shapes) for link in links] == [1] * 8
    assert [link.friction for link in links] == pytest.approx([0.5] * 8)

    initial_positions = np.array(
        [np.asarray(link.translation, dtype=float).reshape(3) for link in links]
    )
    assert initial_positions[:, 0].tolist() == pytest.approx(
        [4.0 * float(index) for index in range(8)]
    )
    assert initial_positions[:, 1].tolist() == pytest.approx([10.0] * 8)
    assert spring_lengths() == pytest.approx([3.0] * 7)

    for _ in range(10):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    final_positions = np.array(
        [np.asarray(link.translation, dtype=float).reshape(3) for link in links]
    )
    assert sx_world.time == pytest.approx(10.0 * sx_world.time_step)
    assert np.isfinite(final_positions).all()
    assert links[0].translation.tolist() == pytest.approx(initial_positions[0])
    assert links[-1].translation.tolist() == pytest.approx(initial_positions[-1])
    assert all(np.isfinite(length) for length in spring_lengths())


def test_avbd_demo3d_spring_scene_matches_source_row() -> None:
    import numpy as np

    sx = _require_simulation_experimental_symbols("World")

    from examples.demos.scenes.avbd_demo3d_spring import build

    setup = build()
    sx_world = setup.info["sx_world"]
    ground = setup.info["ground"]
    anchor = setup.info["anchor"]
    block = setup.info["block"]
    springs = setup.info["springs"]
    ignored_collision_pairs = setup.info["ignored_collision_pairs"]
    spring_length = setup.info["spring_length"]
    source = setup.info["source_demo_reference"]

    assert setup.info["source_demo_row"] == "avbd-demo3d spring"
    assert source["demo"] == "avbd-demo3d"
    assert source["revision"] == "7701bd427d55"
    assert source["scene_index"] == 7
    assert source["scene_name"] == "Spring"
    assert source["scene_builder"] == "sceneSpring"
    assert source["scene_count"] == 14
    assert source["expected_counts"] == {
        "rigid_bodies": 3,
        "dynamic_bodies": 1,
        "static_bodies": 2,
        "joints": 0,
        "distance_springs": 1,
        "collision_shapes": 3,
    }
    assert source["source_shapes"]["ground"] == {
        "density": 0.0,
        "friction": 0.5,
        "position": (0.0, 0.0, 0.0),
        "size": (100.0, 100.0, 1.0),
    }
    assert source["source_shapes"]["anchor"] == {
        "density": 0.0,
        "friction": 0.5,
        "position": (0.0, 0.0, 14.0),
        "size": (1.0, 1.0, 1.0),
    }
    assert source["source_shapes"]["block"] == {
        "density": 1.0,
        "friction": 0.5,
        "position": (0.0, 0.0, 8.0),
        "size": (2.0, 2.0, 2.0),
    }
    assert source["source_constraints"]["radial_distance_springs"] == {
        "child_anchor": (0.0, 0.0, 0.0),
        "count": 1,
        "parent_anchor": (0.0, 0.0, 0.0),
        "rest_length": 4.0,
        "stiffness": 100.0,
    }
    assert sx_world.num_rigid_bodies == 3
    assert sx_world.num_rigid_body_joints == 0
    assert sx_world.time_step == pytest.approx(1.0 / 60.0)
    assert sx_world.gravity.tolist() == pytest.approx([0.0, 0.0, -10.0])
    assert ground.is_static
    assert anchor.is_static
    assert not block.is_static
    assert [len(body.collision_shapes) for body in (ground, anchor, block)] == [
        1,
        1,
        1,
    ]
    assert [body.friction for body in (ground, anchor, block)] == pytest.approx(
        [0.5, 0.5, 0.5]
    )
    assert len(springs) == 1
    assert len(ignored_collision_pairs) == 1
    assert sx_world.num_ignored_collision_pairs == 1
    assert sx_world.is_collision_pair_ignored(anchor, block)
    assert springs[0]["rest_length"] == pytest.approx(4.0)
    assert springs[0]["stiffness"] == pytest.approx(100.0)

    initial_length = spring_length()
    initial_block_z = float(np.asarray(block.translation, dtype=float).reshape(3)[2])
    assert initial_length == pytest.approx(6.0)

    for _ in range(20):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    final_length = spring_length()
    final_block_z = float(np.asarray(block.translation, dtype=float).reshape(3)[2])
    assert sx_world.time == pytest.approx(20.0 * sx_world.time_step)
    assert np.isfinite(final_length)
    assert final_length < initial_length
    assert final_block_z > initial_block_z


def test_avbd_demo3d_spring_ratio_scene_matches_source_row() -> None:
    import numpy as np

    sx = _require_simulation_experimental_symbols("World")

    from examples.demos.scenes.avbd_demo3d_spring_ratio import build

    setup = build()
    sx_world = setup.info["sx_world"]
    ground = setup.info["ground"]
    links = setup.info["links"]
    springs = setup.info["springs"]
    ignored_collision_pairs = setup.info["ignored_collision_pairs"]
    spring_lengths = setup.info["spring_lengths"]
    source = setup.info["source_demo_reference"]

    assert setup.info["source_demo_row"] == "avbd-demo3d spring ratio"
    assert source["demo"] == "avbd-demo3d"
    assert source["revision"] == "7701bd427d55"
    assert source["scene_index"] == 8
    assert source["scene_name"] == "Spring Ratio"
    assert source["scene_builder"] == "sceneSpringsRatio"
    assert source["scene_count"] == 14
    assert source["expected_counts"] == {
        "rigid_bodies": 9,
        "dynamic_bodies": 6,
        "static_bodies": 3,
        "joints": 0,
        "distance_springs": 7,
        "collision_shapes": 9,
    }
    assert source["source_shapes"]["ground"] == {
        "density": 0.0,
        "friction": 0.5,
        "position": (0.0, 0.0, -10.0),
        "size": (100.0, 100.0, 1.0),
    }
    assert source["source_shapes"]["spring_link"] == {
        "count": 8,
        "dynamic_density": 1.0,
        "friction": 0.5,
        "size": (1.0, 0.75, 0.75),
        "static_endpoint_density": 0.0,
        "x_spacing": 3.0,
        "x_start": -10.5,
        "z": 12.0,
    }
    assert source["source_constraints"]["radial_distance_springs"] == {
        "child_anchor": (-0.5, 0.0, 0.0),
        "count": 7,
        "high_stiffness": 1.0e4,
        "high_stiffness_count": 4,
        "low_stiffness": 10.0,
        "low_stiffness_count": 3,
        "parent_anchor": (0.5, 0.0, 0.0),
        "rest_length": 3.0,
    }
    assert sx_world.num_rigid_bodies == 9
    assert sx_world.num_rigid_body_joints == 0
    assert sx_world.time_step == pytest.approx(1.0 / 60.0)
    assert sx_world.gravity.tolist() == pytest.approx([0.0, 0.0, -10.0])
    assert ground.is_static
    assert len(links) == 8
    assert len(springs) == 7
    assert len(ignored_collision_pairs) == 7
    assert sx_world.num_ignored_collision_pairs == 7
    assert all(
        sx_world.is_collision_pair_ignored(links[index], links[index + 1])
        for index in range(7)
    )
    assert not sx_world.is_collision_pair_ignored(links[0], links[2])
    assert [link.is_static for link in links] == [
        True,
        False,
        False,
        False,
        False,
        False,
        False,
        True,
    ]
    assert [spring["stiffness"] for spring in springs] == pytest.approx(
        [1.0e4, 10.0, 1.0e4, 10.0, 1.0e4, 10.0, 1.0e4]
    )
    assert [spring["rest_length"] for spring in springs] == pytest.approx([3.0] * 7)
    assert [len(body.collision_shapes) for body in [ground, *links]] == [1] * 9
    assert [body.friction for body in [ground, *links]] == pytest.approx([0.5] * 9)

    initial_positions = np.array(
        [np.asarray(link.translation, dtype=float).reshape(3) for link in links]
    )
    assert initial_positions[:, 0].tolist() == pytest.approx(
        [-10.5 + 3.0 * float(index) for index in range(8)]
    )
    assert initial_positions[:, 2].tolist() == pytest.approx([12.0] * 8)
    assert spring_lengths() == pytest.approx([2.0] * 7)

    for _ in range(10):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    final_positions = np.array(
        [np.asarray(link.translation, dtype=float).reshape(3) for link in links]
    )
    assert sx_world.time == pytest.approx(10.0 * sx_world.time_step)
    assert np.isfinite(final_positions).all()
    assert links[0].translation.tolist() == pytest.approx(initial_positions[0])
    assert links[-1].translation.tolist() == pytest.approx(initial_positions[-1])
    assert all(np.isfinite(length) for length in spring_lengths())


def test_avbd_demo2d_heavy_rope_scene_matches_source_row() -> None:
    import numpy as np

    sx = _require_simulation_experimental_symbols("World")

    from examples.demos.scenes.avbd_demo2d_heavy_rope import build

    setup = build()
    sx_world = setup.info["sx_world"]
    links = setup.info["links"]
    link_sizes = setup.info["link_sizes"]
    joints = setup.info["joints"]
    source = setup.info["source_demo_reference"]

    assert setup.info["source_demo_row"] == "avbd-demo2d heavy rope"
    assert source["demo"] == "avbd-demo2d"
    assert source["revision"] == "74699a11f858"
    assert source["scene_index"] == 7
    assert source["scene_name"] == "Heavy Rope"
    assert source["scene_builder"] == "sceneHeavyRope"
    assert source["scene_count"] == 19
    assert source["expected_counts"] == {
        "rigid_bodies": 20,
        "dynamic_bodies": 19,
        "static_bodies": 1,
        "joints": 19,
        "linear_point_joints": 19,
        "collision_shapes": 20,
    }
    assert source["source_shapes"]["rope_link"] == {
        "count": 19,
        "dynamic_density": 1.0,
        "first_position": (0.0, 10.0, 0.0),
        "friction": 0.5,
        "size": (1.0, 0.5),
        "static_anchor_density": 0.0,
        "x_spacing": 1.0,
    }
    assert source["source_shapes"]["heavy_block"] == {
        "count": 1,
        "density": 1.0,
        "friction": 0.5,
        "position": (34.0, 10.0, 0.0),
        "size": (30.0, 30.0),
    }
    assert source["source_constraints"]["linear_point_joints"] == {
        "angular_stiffness": 0.0,
        "count": 19,
        "fracture": "infinity",
        "heavy_child_anchor": (-15.0, 0.0),
        "linear_stiffness": "infinity",
        "parent_anchor": (0.5, 0.0),
        "regular_child_anchor": (-0.5, 0.0),
        "source_initial_last_endpoint_gap": 0.5,
    }
    assert sx_world.num_rigid_bodies == 20
    assert sx_world.num_rigid_body_joints == 19
    assert sx_world.num_rigid_body_fixed_joints == 0
    assert sx_world.time_step == pytest.approx(1.0 / 60.0)
    assert sx_world.gravity.tolist() == pytest.approx([0.0, -10.0, 0.0])
    assert len(links) == 20
    assert len(link_sizes) == 20
    assert len(joints) == 19
    assert all(joint.type == sx.JointType.SPHERICAL for joint in joints)
    assert [joint.num_dofs for joint in joints] == [3] * 19
    assert [len(link.collision_shapes) for link in links] == [1] * 20
    assert [link.friction for link in links] == pytest.approx([0.5] * 20)
    assert links[0].is_static
    assert not any(link.is_static for link in links[1:])
    assert np.asarray(link_sizes[-1], dtype=float).tolist() == pytest.approx(
        [30.0, 30.0, 0.2]
    )

    initial_positions = np.array(
        [np.asarray(link.translation, dtype=float).reshape(3) for link in links]
    )
    assert initial_positions[:, 0].tolist() == pytest.approx(
        [float(index) for index in range(19)] + [34.0]
    )
    assert initial_positions[:, 1].tolist() == pytest.approx([10.0] * 20)

    parent_anchor = np.array([0.5, 0.0, 0.0])
    regular_child_anchor = np.array([-0.5, 0.0, 0.0])
    heavy_child_anchor = np.array([-15.0, 0.0, 0.0])

    def endpoint_errors() -> list[float]:
        errors = []
        for index, (parent, child) in enumerate(zip(links, links[1:])):
            child_anchor = heavy_child_anchor if index == 18 else regular_child_anchor
            parent_point = np.asarray(parent.translation, dtype=float).reshape(
                3
            ) + np.asarray(parent.rotation, dtype=float) @ parent_anchor
            child_point = np.asarray(child.translation, dtype=float).reshape(
                3
            ) + np.asarray(child.rotation, dtype=float) @ child_anchor
            errors.append(float(np.linalg.norm(parent_point - child_point)))
        return errors

    initial_endpoint_errors = endpoint_errors()
    assert initial_endpoint_errors[:-1] == pytest.approx([0.0] * 18)
    assert initial_endpoint_errors[-1] == pytest.approx(0.5)
    for _ in range(4):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    final_positions = np.array(
        [np.asarray(link.translation, dtype=float).reshape(3) for link in links]
    )
    assert sx_world.time == pytest.approx(4.0 * sx_world.time_step)
    assert np.isfinite(final_positions).all()
    assert final_positions[-1, 1] < initial_positions[-1, 1] - 0.01
    assert max(endpoint_errors()) < 0.1


def test_avbd_demo2d_net_scene_matches_source_row() -> None:
    import numpy as np

    sx = _require_simulation_experimental_symbols("World")

    from examples.demos.scenes.avbd_demo2d_net import build

    setup = build()
    sx_world = setup.info["sx_world"]
    ground = setup.info["ground"]
    links = setup.info["links"]
    joints = setup.info["joints"]
    falling_blocks = setup.info["falling_blocks"]
    source = setup.info["source_demo_reference"]

    assert setup.info["source_demo_row"] == "avbd-demo2d net"
    assert source["demo"] == "avbd-demo2d"
    assert source["revision"] == "74699a11f858"
    assert source["scene_index"] == 16
    assert source["scene_name"] == "Net"
    assert source["scene_builder"] == "sceneNet"
    assert source["scene_count"] == 19
    assert source["expected_counts"] == {
        "rigid_bodies": 91,
        "dynamic_bodies": 88,
        "static_bodies": 3,
        "joints": 39,
        "linear_point_joints": 39,
        "collision_shapes": 91,
    }
    assert source["source_shapes"]["ground"] == {
        "count": 1,
        "density": 0.0,
        "friction": 0.5,
        "position": (0.0, 0.0, 0.0),
        "size": (100.0, 0.5),
    }
    assert source["source_shapes"]["net_link"] == {
        "count": 40,
        "dynamic_density": 1.0,
        "first_position": (-20.0, 10.0, 0.0),
        "friction": 0.5,
        "size": (1.0, 0.5),
        "static_endpoint_density": 0.0,
        "x_spacing": 1.0,
    }
    assert source["source_shapes"]["falling_block"] == {
        "columns": 10,
        "count": 50,
        "density": 1.0,
        "first_position": (-5.0, 15.0, 0.0),
        "friction": 0.5,
        "rows": 5,
        "size": (1.0, 1.0),
        "x_spacing": 1.0,
        "y_spacing": 1.0,
    }
    assert source["source_constraints"]["linear_point_joints"] == {
        "angular_stiffness": 0.0,
        "child_anchor": (-0.5, 0.0),
        "count": 39,
        "fracture": "infinity",
        "linear_stiffness": "infinity",
        "parent_anchor": (0.5, 0.0),
    }
    assert sx_world.num_rigid_bodies == 91
    assert sx_world.num_rigid_body_joints == 39
    assert sx_world.num_rigid_body_fixed_joints == 0
    assert sx_world.time_step == pytest.approx(1.0 / 60.0)
    assert sx_world.gravity.tolist() == pytest.approx([0.0, -10.0, 0.0])
    assert ground.is_static
    assert len(links) == 40
    assert len(joints) == 39
    assert len(falling_blocks) == 50
    assert all(joint.type == sx.JointType.SPHERICAL for joint in joints)
    assert [joint.num_dofs for joint in joints] == [3] * 39
    assert [len(link.collision_shapes) for link in links] == [1] * 40
    assert [len(block.collision_shapes) for block in falling_blocks] == [1] * 50
    assert links[0].is_static
    assert links[-1].is_static
    assert not any(link.is_static for link in links[1:-1])
    assert not any(block.is_static for block in falling_blocks)

    link_positions = np.array(
        [np.asarray(link.translation, dtype=float).reshape(3) for link in links]
    )
    assert link_positions[:, 0].tolist() == pytest.approx(
        [float(index) - 20.0 for index in range(40)]
    )
    assert link_positions[:, 1].tolist() == pytest.approx([10.0] * 40)

    block_positions = np.array(
        [
            np.asarray(block.translation, dtype=float).reshape(3)
            for block in falling_blocks
        ]
    )
    assert block_positions[:, 0].tolist() == pytest.approx(
        [float(x) - 5.0 for x in range(10) for _ in range(5)]
    )
    assert block_positions[:, 1].tolist() == pytest.approx(
        [float(y) + 15.0 for _ in range(10) for y in range(5)]
    )

    parent_anchor = np.array([0.5, 0.0, 0.0])
    child_anchor = np.array([-0.5, 0.0, 0.0])

    def endpoint_errors() -> list[float]:
        errors = []
        for parent, child in zip(links, links[1:]):
            parent_point = np.asarray(parent.translation, dtype=float).reshape(
                3
            ) + np.asarray(parent.rotation, dtype=float) @ parent_anchor
            child_point = np.asarray(child.translation, dtype=float).reshape(
                3
            ) + np.asarray(child.rotation, dtype=float) @ child_anchor
            errors.append(float(np.linalg.norm(parent_point - child_point)))
        return errors

    assert max(endpoint_errors()) < 1.0e-12
    initial_lowest_block = float(block_positions[:, 1].min())
    for _ in range(4):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    final_link_positions = np.array(
        [np.asarray(link.translation, dtype=float).reshape(3) for link in links]
    )
    final_block_positions = np.array(
        [
            np.asarray(block.translation, dtype=float).reshape(3)
            for block in falling_blocks
        ]
    )
    assert sx_world.time == pytest.approx(4.0 * sx_world.time_step)
    assert np.isfinite(final_link_positions).all()
    assert np.isfinite(final_block_positions).all()
    assert float(final_block_positions[:, 1].min()) < initial_lowest_block


def test_avbd_demo2d_fracture_scene_matches_source_row() -> None:
    import numpy as np

    sx = _require_simulation_experimental_symbols("World")

    from examples.demos.scenes.avbd_demo2d_fracture import build

    setup = build()
    sx_world = setup.info["sx_world"]
    ground = setup.info["ground"]
    chain = setup.info["chain"]
    joints = setup.info["joints"]
    supports = setup.info["supports"]
    falling_blocks = setup.info["falling_blocks"]
    source = setup.info["source_demo_reference"]

    assert setup.info["source_demo_row"] == "avbd-demo2d fracture"
    assert source["demo"] == "avbd-demo2d"
    assert source["revision"] == "74699a11f858"
    assert source["scene_index"] == 18
    assert source["scene_name"] == "Fracture"
    assert source["scene_builder"] == "sceneFracture"
    assert source["scene_count"] == 19
    assert source["expected_counts"] == {
        "rigid_bodies": 29,
        "dynamic_bodies": 28,
        "static_bodies": 1,
        "joints": 10,
        "breakable_joints": 10,
        "collision_shapes": 29,
    }
    assert source["source_shapes"]["support"] == {
        "count": 2,
        "size": (1.0, 5.0),
        "density": 1.0,
        "friction": 0.5,
        "positions": ((-5.0, 2.5, 0.0), (5.0, 2.5, 0.0)),
    }
    assert source["source_shapes"]["falling_block"] == {
        "count": 15,
        "size": (2.0, 1.0),
        "density": 1.0,
        "friction": 0.5,
        "first_position": (0.0, 8.0, 0.0),
        "y_spacing": 2.0,
    }
    assert source["source_constraints"]["breakable_fixed_joints"] == {
        "count": 10,
        "anchor_a": (0.5, 0.0),
        "anchor_b": (-0.5, 0.0),
        "linear_stiffness": "infinity",
        "angular_stiffness": "infinity",
        "break_force": 500.0,
    }
    assert source["source_collision_filters"] == {
        "joint_connected_collision_pairs": 10,
    }
    assert sx_world.num_rigid_bodies == 29
    assert sx_world.num_rigid_body_joints == 10
    assert sx_world.num_rigid_body_fixed_joints == 10
    assert sx_world.num_ignored_collision_pairs == 0
    assert not sx_world.is_collision_pair_ignored(chain[0], chain[1])
    assert sx_world.time_step == pytest.approx(1.0 / 60.0)
    assert sx_world.gravity.tolist() == pytest.approx([0.0, -10.0, 0.0])
    assert len(chain) == 11
    assert len(joints) == 10
    assert len(supports) == 2
    assert len(falling_blocks) == 15
    assert all(joint.type == sx.JointType.FIXED for joint in joints)
    assert all(joint.break_force == pytest.approx(500.0) for joint in joints)
    assert ground.is_static
    assert not any(body.is_static for body in chain)
    assert not any(body.is_static for body in supports)
    assert not any(body.is_static for body in falling_blocks)
    assert len(ground.collision_shapes) == 1
    assert sum(len(body.collision_shapes) for body in chain) == 11
    assert sum(len(body.collision_shapes) for body in supports) == 2
    assert sum(len(body.collision_shapes) for body in falling_blocks) == 15
    assert [body.friction for body in chain] == pytest.approx([0.5] * 11)
    assert [body.friction for body in supports] == pytest.approx([0.5] * 2)
    assert [body.friction for body in falling_blocks] == pytest.approx([0.5] * 15)

    initial_chain_positions = np.array(
        [np.asarray(body.translation, dtype=float).reshape(3) for body in chain]
    )
    assert initial_chain_positions[:, 0].tolist() == pytest.approx(
        [float(index - 5) for index in range(11)]
    )
    assert initial_chain_positions[:, 1].tolist() == pytest.approx([6.0] * 11)
    initial_support_positions = np.array(
        [np.asarray(body.translation, dtype=float).reshape(3) for body in supports]
    )
    np.testing.assert_allclose(
        initial_support_positions[:, :2], np.array([[-5.0, 2.5], [5.0, 2.5]])
    )
    initial_falling_positions = np.array(
        [
            np.asarray(body.translation, dtype=float).reshape(3)
            for body in falling_blocks
        ]
    )
    assert initial_falling_positions[:, 0].tolist() == pytest.approx([0.0] * 15)
    assert initial_falling_positions[:, 1].tolist() == pytest.approx(
        [float(index) * 2.0 + 8.0 for index in range(15)]
    )

    for _ in range(6):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    final_chain = np.asarray(chain[5].translation, dtype=float).reshape(3)
    final_top_block = np.asarray(falling_blocks[-1].translation, dtype=float).reshape(
        3
    )
    assert sx_world.time == pytest.approx(6.0 * sx_world.time_step)
    assert np.isfinite(final_chain).all()
    assert np.isfinite(final_top_block).all()
    assert final_top_block[1] < initial_falling_positions[-1, 1]
    assert abs(final_chain[1] - initial_chain_positions[5, 1]) < 1.0


def test_avbd_demo3d_breakable_scene_matches_source_row() -> None:
    import numpy as np

    sx = _require_simulation_experimental_symbols("World")

    from examples.demos.scenes.avbd_demo3d_breakable import build

    setup = build()
    sx_world = setup.info["sx_world"]
    ground = setup.info["ground"]
    chain = setup.info["chain"]
    joints = setup.info["joints"]
    supports = setup.info["supports"]
    falling_blocks = setup.info["falling_blocks"]
    source = setup.info["source_demo_reference"]

    assert setup.info["source_demo_row"] == "avbd-demo3d breakable"
    assert source["demo"] == "avbd-demo3d"
    assert source["revision"] == "7701bd427d55"
    assert source["scene_index"] == 13
    assert source["scene_name"] == "Breakable"
    assert source["scene_builder"] == "sceneBreakable"
    assert source["scene_count"] == 14
    assert source["expected_counts"] == {
        "rigid_bodies": 19,
        "joints": 10,
        "breakable_joints": 10,
        "collision_shapes": 19,
    }
    assert source["source_constraints"]["breakable_fixed_joints"] == {
        "count": 10,
        "anchor_a": (0.5, 0.0, 0.0),
        "anchor_b": (-0.5, 0.0, 0.0),
        "linear_stiffness": "infinity",
        "angular_stiffness": "infinity",
        "break_force": 90.0,
    }
    assert sx_world.num_rigid_bodies == 19
    assert sx_world.num_rigid_body_fixed_joints == 10
    assert sx_world.time_step == pytest.approx(1.0 / 60.0)
    assert sx_world.gravity.tolist() == pytest.approx([0.0, 0.0, -10.0])
    assert len(chain) == 11
    assert len(joints) == 10
    assert len(supports) == 2
    assert len(falling_blocks) == 5
    assert all(joint.type == sx.JointType.FIXED for joint in joints)
    assert all(joint.break_force == pytest.approx(90.0) for joint in joints)
    assert len(ground.collision_shapes) == 1
    assert sum(len(body.collision_shapes) for body in chain) == 11
    assert sum(len(body.collision_shapes) for body in supports) == 2
    assert sum(len(body.collision_shapes) for body in falling_blocks) == 5

    initial_chain = np.asarray(chain[5].translation, dtype=float).reshape(3)
    initial_top_block = np.asarray(falling_blocks[-1].translation, dtype=float).reshape(3)
    for _ in range(6):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    final_chain = np.asarray(chain[5].translation, dtype=float).reshape(3)
    final_top_block = np.asarray(falling_blocks[-1].translation, dtype=float).reshape(3)
    assert sx_world.time == pytest.approx(6.0 * sx_world.time_step)
    assert np.isfinite(final_chain).all()
    assert np.isfinite(final_top_block).all()
    assert final_top_block[2] < initial_top_block[2]
    assert abs(final_chain[2] - initial_chain[2]) < 1.0


def _fixed_chain_anchor_errors(chain: tuple[Any, ...]) -> list[float]:
    import numpy as np

    parent_anchor = np.array([0.5, 0.0, 0.0])
    child_anchor = np.array([-0.5, 0.0, 0.0])
    errors = []
    for parent, child in zip(chain, chain[1:]):
        parent_point = np.asarray(parent.translation, dtype=float).reshape(
            3
        ) + np.asarray(parent.rotation, dtype=float) @ parent_anchor
        child_point = np.asarray(child.translation, dtype=float).reshape(
            3
        ) + np.asarray(child.rotation, dtype=float) @ child_anchor
        errors.append(float(np.linalg.norm(parent_point - child_point)))
    return errors


def _assert_source_fixed_joint_fracture_resets(setup: Any) -> None:
    joints = setup.info["joints"]
    chain = setup.info["chain"]

    assert len(joints) == 10
    assert max(_fixed_chain_anchor_errors(chain)) < 1.0e-12

    for _ in range(45):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    broken_count = sum(joint.is_broken for joint in joints)
    assert broken_count >= len(joints) // 2
    broken_anchor_error = max(_fixed_chain_anchor_errors(chain))
    assert broken_anchor_error > 1.0e-2

    for joint in joints:
        joint.break_force = 1.0e12
        if joint.is_broken:
            joint.reset_breakage()

    assert not any(joint.is_broken for joint in joints)

    min_reset_anchor_error = float("inf")
    for _ in range(6):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()
        min_reset_anchor_error = min(
            min_reset_anchor_error, max(_fixed_chain_anchor_errors(chain))
        )

    assert not any(joint.is_broken for joint in joints)
    assert min_reset_anchor_error < 0.5 * broken_anchor_error


def test_avbd_demo2d_fracture_scene_breaks_and_resets_source_joints() -> None:
    _require_simulation_experimental_symbols("World")

    from examples.demos.scenes.avbd_demo2d_fracture import build

    setup = build()
    assert setup.info["source_demo_row"] == "avbd-demo2d fracture"
    assert setup.info["break_force"] == pytest.approx(500.0)

    _assert_source_fixed_joint_fracture_resets(setup)


def test_avbd_demo3d_breakable_scene_breaks_and_resets_source_joints() -> None:
    _require_simulation_experimental_symbols("World")

    from examples.demos.scenes.avbd_demo3d_breakable import build

    setup = build()
    assert setup.info["source_demo_row"] == "avbd-demo3d breakable"
    assert setup.info["break_force"] == pytest.approx(90.0)

    _assert_source_fixed_joint_fracture_resets(setup)


def test_avbd_demo3d_ground_scene_matches_source_row() -> None:
    import numpy as np

    sx = _require_simulation_experimental_symbols("World")

    from examples.demos.scenes.avbd_demo3d_ground import build

    setup = build()
    sx_world = setup.info["sx_world"]
    ground = setup.info["ground"]
    box = setup.info["box"]
    source = setup.info["source_demo_reference"]

    assert setup.info["source_demo_row"] == "avbd-demo3d ground"
    assert source["demo"] == "avbd-demo3d"
    assert source["revision"] == "7701bd427d55"
    assert source["scene_index"] == 1
    assert source["scene_name"] == "Ground"
    assert source["scene_builder"] == "sceneGround"
    assert source["scene_count"] == 14
    assert source["expected_counts"] == {
        "rigid_bodies": 2,
        "joints": 0,
        "collision_shapes": 2,
    }
    assert sx_world.num_rigid_bodies == 2
    assert sx_world.num_rigid_body_fixed_joints == 0
    assert sx_world.time_step == pytest.approx(1.0 / 60.0)
    assert sx_world.gravity.tolist() == pytest.approx([0.0, 0.0, -10.0])
    assert len(ground.collision_shapes) == 1
    assert len(box.collision_shapes) == 1

    initial_box = np.asarray(box.translation, dtype=float).reshape(3)
    first_contact_frame = None
    for frame in range(1, 61):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()
        if sx_world.collide() and first_contact_frame is None:
            first_contact_frame = frame

    final_box = np.asarray(box.translation, dtype=float).reshape(3)
    contacts = sx_world.collide()
    assert sx_world.time == pytest.approx(60.0 * sx_world.time_step)
    assert np.isfinite(final_box).all()
    assert final_box[2] < initial_box[2]
    assert final_box[2] == pytest.approx(1.0, abs=5.0e-3)
    assert first_contact_frame is not None
    assert contacts
    for contact in contacts:
        assert {contact.body_a.name, contact.body_b.name} == {
            "avbd_demo3d_ground_box",
            "avbd_demo3d_ground_ground",
        }


def test_avbd_demo3d_dynamic_friction_scene_matches_source_row() -> None:
    import numpy as np

    _require_simulation_experimental_symbols("World")

    from examples.demos.scenes.avbd_demo3d_dynamic_friction import build

    setup = build()
    sx_world = setup.info["sx_world"]
    ground = setup.info["ground"]
    boxes = setup.info["boxes"]
    source = setup.info["source_demo_reference"]

    assert setup.info["source_demo_row"] == "avbd-demo3d dynamic friction"
    assert source["demo"] == "avbd-demo3d"
    assert source["revision"] == "7701bd427d55"
    assert source["scene_index"] == 2
    assert source["scene_name"] == "Dynamic Friction"
    assert source["scene_builder"] == "sceneDynamicFriction"
    assert source["scene_count"] == 14
    assert source["expected_counts"] == {
        "rigid_bodies": 12,
        "joints": 0,
        "collision_shapes": 12,
    }
    assert source["source_shapes"]["boxes"] == {
        "count": 11,
        "density": 1.0,
        "friction_range": (5.0, 0.0),
        "initial_velocity": (10.0, 0.0, 0.0),
        "size": (1.0, 1.0, 0.5),
    }
    assert sx_world.num_rigid_bodies == 12
    assert sx_world.num_rigid_body_fixed_joints == 0
    assert sx_world.time_step == pytest.approx(1.0 / 60.0)
    assert sx_world.gravity.tolist() == pytest.approx([0.0, 0.0, -10.0])
    assert len(ground.collision_shapes) == 1
    assert len(boxes) == 11
    assert sum(len(box.collision_shapes) for box in boxes) == 11
    assert [box.friction for box in boxes] == pytest.approx(
        [5.0 - float(index) / 10.0 * 5.0 for index in range(11)]
    )
    assert len(sx_world.collide()) >= 11

    high_friction_box = boxes[0]
    zero_friction_box = boxes[-1]
    initial_high = np.asarray(high_friction_box.translation, dtype=float).reshape(3)
    initial_zero = np.asarray(zero_friction_box.translation, dtype=float).reshape(3)

    for _ in range(60):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    final_high = np.asarray(high_friction_box.translation, dtype=float).reshape(3)
    final_zero = np.asarray(zero_friction_box.translation, dtype=float).reshape(3)
    high_speed = float(
        np.linalg.norm(
            np.asarray(high_friction_box.linear_velocity, dtype=float).reshape(3)[:2]
        )
    )
    zero_speed = float(
        np.linalg.norm(
            np.asarray(zero_friction_box.linear_velocity, dtype=float).reshape(3)[:2]
        )
    )
    assert sx_world.time == pytest.approx(60.0 * sx_world.time_step)
    assert np.isfinite(final_high).all()
    assert np.isfinite(final_zero).all()
    assert final_high[0] > initial_high[0]
    assert final_zero[0] > initial_zero[0]
    assert final_high[0] < final_zero[0] - 4.0
    assert final_high[2] == pytest.approx(0.75, abs=1.0e-2)
    assert final_zero[2] == pytest.approx(0.75, abs=1.0e-2)
    assert high_speed < 1.0
    assert zero_speed > 9.0
    assert len(sx_world.collide()) >= 11


def test_avbd_demo3d_static_friction_scene_matches_source_row() -> None:
    import numpy as np

    _require_simulation_experimental_symbols("World")

    from examples.demos.scenes.avbd_demo3d_static_friction import build

    setup = build()
    sx_world = setup.info["sx_world"]
    ground = setup.info["ground"]
    ramp = setup.info["ramp"]
    boxes = setup.info["boxes"]
    tangent = np.asarray(setup.info["ramp_tangent"], dtype=float).reshape(3)
    source = setup.info["source_demo_reference"]

    assert setup.info["source_demo_row"] == "avbd-demo3d static friction"
    assert source["demo"] == "avbd-demo3d"
    assert source["revision"] == "7701bd427d55"
    assert source["scene_index"] == 3
    assert source["scene_name"] == "Static Friction"
    assert source["scene_builder"] == "sceneStaticFriction"
    assert source["scene_count"] == 14
    assert source["expected_counts"] == {
        "rigid_bodies": 13,
        "joints": 0,
        "collision_shapes": 13,
    }
    assert source["source_shapes"]["ramp"] == {
        "angle_degrees": 30.0,
        "density": 0.0,
        "friction": 1.0,
        "size": (40.0, 24.0, 1.0),
    }
    assert source["source_shapes"]["boxes"] == {
        "count": 11,
        "density": 1.0,
        "friction_range": (0.25, 0.5),
        "size": (1.0, 1.0, 1.0),
    }
    assert sx_world.num_rigid_bodies == 13
    assert sx_world.num_rigid_body_fixed_joints == 0
    assert sx_world.time_step == pytest.approx(1.0 / 60.0)
    assert sx_world.gravity.tolist() == pytest.approx([0.0, 0.0, -10.0])
    assert len(ground.collision_shapes) == 1
    assert len(ramp.collision_shapes) == 1
    assert len(boxes) == 11
    assert sum(len(box.collision_shapes) for box in boxes) == 11
    assert [box.friction for box in boxes] == pytest.approx(
        [float(index) / 10.0 * 0.25 + 0.25 for index in range(11)]
    )
    assert len(sx_world.collide()) >= 11

    low_friction_box = boxes[0]
    high_friction_box = boxes[-1]
    initial_low = np.asarray(low_friction_box.translation, dtype=float).reshape(3)
    initial_high = np.asarray(high_friction_box.translation, dtype=float).reshape(3)

    for _ in range(60):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    final_low = np.asarray(low_friction_box.translation, dtype=float).reshape(3)
    final_high = np.asarray(high_friction_box.translation, dtype=float).reshape(3)
    low_slide = float(np.dot(final_low - initial_low, tangent))
    high_slide = float(np.dot(final_high - initial_high, tangent))
    low_speed = float(
        np.linalg.norm(
            np.asarray(low_friction_box.linear_velocity, dtype=float).reshape(3)
        )
    )
    high_speed = float(
        np.linalg.norm(
            np.asarray(high_friction_box.linear_velocity, dtype=float).reshape(3)
        )
    )
    assert sx_world.time == pytest.approx(60.0 * sx_world.time_step)
    assert np.isfinite(final_low).all()
    assert np.isfinite(final_high).all()
    assert low_slide > high_slide + 0.25
    assert low_speed > high_speed + 0.5
    assert len(sx_world.collide()) >= 11


def test_avbd_demo3d_pyramid_scene_matches_source_row() -> None:
    import numpy as np

    _require_simulation_experimental_symbols("World")

    from examples.demos.scenes.avbd_demo3d_pyramid import build

    setup = build()
    sx_world = setup.info["sx_world"]
    ground = setup.info["ground"]
    boxes = setup.info["boxes"]
    source = setup.info["source_demo_reference"]

    assert setup.info["source_demo_row"] == "avbd-demo3d pyramid"
    assert source["demo"] == "avbd-demo3d"
    assert source["revision"] == "7701bd427d55"
    assert source["scene_index"] == 4
    assert source["scene_name"] == "Pyramid"
    assert source["scene_builder"] == "scenePyramid"
    assert source["scene_count"] == 14
    assert source["expected_counts"] == {
        "rigid_bodies": 137,
        "joints": 0,
        "collision_shapes": 137,
    }
    assert source["source_shapes"]["ground"] == {
        "density": 0.0,
        "friction": 0.5,
        "position": (0.0, 0.0, -0.5),
        "size": (100.0, 100.0, 1.0),
    }
    assert source["source_shapes"]["boxes"] == {
        "base_z": 0.5,
        "count": 136,
        "density": 1.0,
        "friction": 0.5,
        "pyramid_size": 16,
        "row_x_offset": 0.5,
        "size": (1.0, 0.5, 0.5),
        "x_spacing": 1.01,
        "z_spacing": 0.85,
    }
    assert sx_world.num_rigid_bodies == 137
    assert sx_world.num_rigid_body_fixed_joints == 0
    assert sx_world.time_step == pytest.approx(1.0 / 60.0)
    assert sx_world.gravity.tolist() == pytest.approx([0.0, 0.0, -10.0])
    assert len(ground.collision_shapes) == 1
    assert len(boxes) == 136
    assert sum(len(box.collision_shapes) for box in boxes) == 136
    assert [box.friction for box in boxes] == pytest.approx([0.5] * 136)

    top_box = boxes[-1]
    initial_top = np.asarray(top_box.translation, dtype=float).reshape(3)

    for _ in range(30):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    final_top = np.asarray(top_box.translation, dtype=float).reshape(3)
    positions = np.array(
        [np.asarray(box.translation, dtype=float).reshape(3) for box in boxes]
    )
    assert sx_world.time == pytest.approx(30.0 * sx_world.time_step)
    assert np.isfinite(positions).all()
    assert final_top[2] < initial_top[2] - 0.5
    assert positions[:, 2].min() > -0.5
    assert len(sx_world.collide()) > 25


def test_avbd_demo3d_stack_scene_matches_source_row() -> None:
    import numpy as np

    _require_simulation_experimental_symbols("World")

    from examples.demos.scenes.avbd_demo3d_stack import build

    setup = build()
    sx_world = setup.info["sx_world"]
    ground = setup.info["ground"]
    boxes = setup.info["boxes"]
    source = setup.info["source_demo_reference"]

    assert setup.info["source_demo_row"] == "avbd-demo3d stack"
    assert source["demo"] == "avbd-demo3d"
    assert source["revision"] == "7701bd427d55"
    assert source["scene_index"] == 9
    assert source["scene_name"] == "Stack"
    assert source["scene_builder"] == "sceneStack"
    assert source["scene_count"] == 14
    assert source["expected_counts"] == {
        "rigid_bodies": 11,
        "joints": 0,
        "collision_shapes": 11,
    }
    assert source["source_shapes"]["ground"] == {
        "density": 0.0,
        "friction": 0.5,
        "position": (0.0, 0.0, 0.0),
        "size": (100.0, 100.0, 1.0),
    }
    assert source["source_shapes"]["boxes"] == {
        "base_z": 1.0,
        "count": 10,
        "density": 1.0,
        "friction": 0.5,
        "size": (1.0, 1.0, 1.0),
        "z_spacing": 1.5,
    }
    assert sx_world.num_rigid_bodies == 11
    assert sx_world.num_rigid_body_fixed_joints == 0
    assert sx_world.time_step == pytest.approx(1.0 / 60.0)
    assert sx_world.gravity.tolist() == pytest.approx([0.0, 0.0, -10.0])
    assert len(ground.collision_shapes) == 1
    assert len(boxes) == 10
    assert sum(len(box.collision_shapes) for box in boxes) == 10
    assert [box.friction for box in boxes] == pytest.approx([0.5] * 10)

    initial_positions = np.array(
        [np.asarray(box.translation, dtype=float).reshape(3) for box in boxes]
    )
    assert initial_positions[:, 2].tolist() == pytest.approx(
        [float(index) * 1.5 + 1.0 for index in range(10)]
    )

    for _ in range(30):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    final_positions = np.array(
        [np.asarray(box.translation, dtype=float).reshape(3) for box in boxes]
    )
    assert sx_world.time == pytest.approx(30.0 * sx_world.time_step)
    assert np.isfinite(final_positions).all()
    assert final_positions[-1, 2] < initial_positions[-1, 2] - 0.5
    assert final_positions[:, 2].min() > 0.25
    assert len(sx_world.collide()) > 8


def test_avbd_demo3d_rope_scene_matches_source_row() -> None:
    import numpy as np

    sx = _require_simulation_experimental_symbols("World")

    from examples.demos.scenes.avbd_demo3d_rope import build

    setup = build()
    sx_world = setup.info["sx_world"]
    ground = setup.info["ground"]
    links = setup.info["links"]
    joints = setup.info["joints"]
    source = setup.info["source_demo_reference"]

    assert setup.info["source_demo_row"] == "avbd-demo3d rope"
    assert source["demo"] == "avbd-demo3d"
    assert source["revision"] == "7701bd427d55"
    assert source["scene_index"] == 5
    assert source["scene_name"] == "Rope"
    assert source["scene_builder"] == "sceneRope"
    assert source["scene_count"] == 14
    assert source["expected_counts"] == {
        "rigid_bodies": 21,
        "dynamic_bodies": 19,
        "static_bodies": 2,
        "joints": 19,
        "linear_point_joints": 19,
        "collision_shapes": 21,
    }
    assert source["source_shapes"]["ground"] == {
        "density": 0.0,
        "friction": 0.5,
        "position": (0.0, 0.0, -20.0),
        "size": (100.0, 100.0, 1.0),
    }
    assert source["source_shapes"]["rope_link"] == {
        "count": 20,
        "dynamic_density": 1.0,
        "first_position": (0.0, 0.0, 10.0),
        "friction": 0.5,
        "size": (1.0, 0.5, 0.5),
        "static_anchor_density": 0.0,
        "x_spacing": 1.0,
    }
    assert source["source_constraints"]["linear_point_joints"] == {
        "angular_stiffness": 0.0,
        "child_anchor": (-0.5, 0.0, 0.0),
        "count": 19,
        "fracture": "infinity",
        "linear_stiffness": "infinity",
        "parent_anchor": (0.5, 0.0, 0.0),
    }
    assert sx_world.num_rigid_bodies == 21
    assert sx_world.num_rigid_body_joints == 19
    assert sx_world.num_rigid_body_fixed_joints == 0
    assert sx_world.time_step == pytest.approx(1.0 / 60.0)
    assert sx_world.gravity.tolist() == pytest.approx([0.0, 0.0, -10.0])
    assert len(ground.collision_shapes) == 1
    assert len(links) == 20
    assert len(joints) == 19
    assert all(joint.type == sx.JointType.SPHERICAL for joint in joints)
    assert [joint.num_dofs for joint in joints] == [3] * 19
    assert [link.friction for link in links] == pytest.approx([0.5] * 20)
    assert links[0].is_static
    assert not any(link.is_static for link in links[1:])

    initial_positions = np.array(
        [np.asarray(link.translation, dtype=float).reshape(3) for link in links]
    )
    assert initial_positions[:, 0].tolist() == pytest.approx(
        [float(index) for index in range(20)]
    )
    assert initial_positions[:, 2].tolist() == pytest.approx([10.0] * 20)

    parent_anchor = np.array([0.5, 0.0, 0.0])
    child_anchor = np.array([-0.5, 0.0, 0.0])

    def endpoint_errors() -> list[float]:
        errors = []
        for parent, child in zip(links, links[1:]):
            parent_point = np.asarray(parent.translation, dtype=float).reshape(
                3
            ) + np.asarray(parent.rotation, dtype=float) @ parent_anchor
            child_point = np.asarray(child.translation, dtype=float).reshape(
                3
            ) + np.asarray(child.rotation, dtype=float) @ child_anchor
            errors.append(float(np.linalg.norm(parent_point - child_point)))
        return errors

    assert max(endpoint_errors()) < 1.0e-12
    for _ in range(30):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    final_positions = np.array(
        [np.asarray(link.translation, dtype=float).reshape(3) for link in links]
    )
    assert sx_world.time == pytest.approx(30.0 * sx_world.time_step)
    assert np.isfinite(final_positions).all()
    assert final_positions[-1, 2] < initial_positions[-1, 2] - 0.05
    assert max(endpoint_errors()) < 0.35


def test_avbd_demo3d_heavy_rope_scene_matches_source_row() -> None:
    import numpy as np

    sx = _require_simulation_experimental_symbols("World")

    from examples.demos.scenes.avbd_demo3d_heavy_rope import build

    setup = build()
    sx_world = setup.info["sx_world"]
    ground = setup.info["ground"]
    links = setup.info["links"]
    link_sizes = setup.info["link_sizes"]
    joints = setup.info["joints"]
    source = setup.info["source_demo_reference"]

    assert setup.info["source_demo_row"] == "avbd-demo3d heavy rope"
    assert source["demo"] == "avbd-demo3d"
    assert source["revision"] == "7701bd427d55"
    assert source["scene_index"] == 6
    assert source["scene_name"] == "Heavy Rope"
    assert source["scene_builder"] == "sceneHeavyRope"
    assert source["scene_count"] == 14
    assert source["expected_counts"] == {
        "rigid_bodies": 21,
        "dynamic_bodies": 19,
        "static_bodies": 2,
        "joints": 19,
        "linear_point_joints": 19,
        "collision_shapes": 21,
    }
    assert source["source_shapes"]["ground"] == {
        "density": 0.0,
        "friction": 0.5,
        "position": (0.0, 0.0, -20.0),
        "size": (100.0, 100.0, 1.0),
    }
    assert source["source_shapes"]["rope_link"] == {
        "count": 19,
        "dynamic_density": 1.0,
        "first_position": (0.0, 0.0, 10.0),
        "friction": 0.5,
        "size": (1.0, 0.5, 0.5),
        "static_anchor_density": 0.0,
        "x_spacing": 1.0,
    }
    assert source["source_shapes"]["heavy_block"] == {
        "count": 1,
        "density": 1.0,
        "friction": 0.5,
        "position": (21.5, 0.0, 10.0),
        "size": (5.0, 5.0, 5.0),
    }
    assert source["source_constraints"]["linear_point_joints"] == {
        "angular_stiffness": 0.0,
        "count": 19,
        "fracture": "infinity",
        "heavy_child_anchor": (-2.5, 0.0, 0.0),
        "linear_stiffness": "infinity",
        "parent_anchor": (0.5, 0.0, 0.0),
        "regular_child_anchor": (-0.5, 0.0, 0.0),
        "source_initial_last_endpoint_gap": 0.5,
    }
    assert sx_world.num_rigid_bodies == 21
    assert sx_world.num_rigid_body_joints == 19
    assert sx_world.num_rigid_body_fixed_joints == 0
    assert sx_world.time_step == pytest.approx(1.0 / 60.0)
    assert sx_world.gravity.tolist() == pytest.approx([0.0, 0.0, -10.0])
    assert len(ground.collision_shapes) == 1
    assert len(links) == 20
    assert len(link_sizes) == 20
    assert len(joints) == 19
    assert all(joint.type == sx.JointType.SPHERICAL for joint in joints)
    assert [joint.num_dofs for joint in joints] == [3] * 19
    assert [link.friction for link in links] == pytest.approx([0.5] * 20)
    assert links[0].is_static
    assert not any(link.is_static for link in links[1:])
    assert np.asarray(link_sizes[-1], dtype=float).tolist() == pytest.approx(
        [5.0, 5.0, 5.0]
    )

    initial_positions = np.array(
        [np.asarray(link.translation, dtype=float).reshape(3) for link in links]
    )
    assert initial_positions[:, 0].tolist() == pytest.approx(
        [float(index) for index in range(19)] + [21.5]
    )
    assert initial_positions[:, 2].tolist() == pytest.approx([10.0] * 20)

    parent_anchor = np.array([0.5, 0.0, 0.0])
    regular_child_anchor = np.array([-0.5, 0.0, 0.0])
    heavy_child_anchor = np.array([-2.5, 0.0, 0.0])

    def endpoint_errors() -> list[float]:
        errors = []
        for index, (parent, child) in enumerate(zip(links, links[1:])):
            child_anchor = heavy_child_anchor if index == 18 else regular_child_anchor
            parent_point = np.asarray(parent.translation, dtype=float).reshape(
                3
            ) + np.asarray(parent.rotation, dtype=float) @ parent_anchor
            child_point = np.asarray(child.translation, dtype=float).reshape(
                3
            ) + np.asarray(child.rotation, dtype=float) @ child_anchor
            errors.append(float(np.linalg.norm(parent_point - child_point)))
        return errors

    initial_endpoint_errors = endpoint_errors()
    assert initial_endpoint_errors[:-1] == pytest.approx([0.0] * 18)
    assert initial_endpoint_errors[-1] == pytest.approx(0.5)
    for _ in range(30):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    final_positions = np.array(
        [np.asarray(link.translation, dtype=float).reshape(3) for link in links]
    )
    assert sx_world.time == pytest.approx(30.0 * sx_world.time_step)
    assert np.isfinite(final_positions).all()
    assert final_positions[-1, 2] < initial_positions[-1, 2] - 0.05
    assert max(endpoint_errors()) < 0.75


def test_avbd_demo3d_stack_ratio_scene_matches_source_row() -> None:
    import numpy as np

    _require_simulation_experimental_symbols("World")

    from examples.demos.scenes.avbd_demo3d_stack_ratio import build

    setup = build()
    sx_world = setup.info["sx_world"]
    ground = setup.info["ground"]
    boxes = setup.info["boxes"]
    box_sizes = setup.info["box_sizes"]
    source = setup.info["source_demo_reference"]

    assert setup.info["source_demo_row"] == "avbd-demo3d stack ratio"
    assert source["demo"] == "avbd-demo3d"
    assert source["revision"] == "7701bd427d55"
    assert source["scene_index"] == 10
    assert source["scene_name"] == "Stack Ratio"
    assert source["scene_builder"] == "sceneStackRatio"
    assert source["scene_count"] == 14
    assert source["expected_counts"] == {
        "rigid_bodies": 5,
        "joints": 0,
        "collision_shapes": 5,
    }
    assert source["source_shapes"]["ground"] == {
        "density": 0.0,
        "friction": 0.5,
        "position": (0.0, 0.0, 0.0),
        "size": (100.0, 100.0, 1.0),
    }
    assert source["source_shapes"]["boxes"] == {
        "centers_z": (1.0, 2.5, 5.5, 11.5),
        "count": 4,
        "density": 1.0,
        "friction": 0.5,
        "initial_size": 1.0,
        "size_multiplier": 2.0,
        "sizes": (
            (1.0, 1.0, 1.0),
            (2.0, 2.0, 2.0),
            (4.0, 4.0, 4.0),
            (8.0, 8.0, 8.0),
        ),
    }
    assert sx_world.num_rigid_bodies == 5
    assert sx_world.num_rigid_body_fixed_joints == 0
    assert sx_world.time_step == pytest.approx(1.0 / 60.0)
    assert sx_world.gravity.tolist() == pytest.approx([0.0, 0.0, -10.0])
    assert len(ground.collision_shapes) == 1
    assert len(boxes) == 4
    assert [box.friction for box in boxes] == pytest.approx([0.5] * 4)
    assert np.array([size.tolist() for size in box_sizes]) == pytest.approx(
        np.array(
            [
                [1.0, 1.0, 1.0],
                [2.0, 2.0, 2.0],
                [4.0, 4.0, 4.0],
                [8.0, 8.0, 8.0],
            ]
        )
    )

    initial_positions = np.array(
        [np.asarray(box.translation, dtype=float).reshape(3) for box in boxes]
    )
    assert initial_positions[:, 2].tolist() == pytest.approx([1.0, 2.5, 5.5, 11.5])

    for _ in range(30):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    final_positions = np.array(
        [np.asarray(box.translation, dtype=float).reshape(3) for box in boxes]
    )
    assert sx_world.time == pytest.approx(30.0 * sx_world.time_step)
    assert np.isfinite(final_positions).all()
    assert final_positions[-1, 2] < initial_positions[-1, 2] - 0.5
    assert final_positions[:, 2].min() > 0.25
    assert len(sx_world.collide()) > 12


def test_avbd_demo3d_soft_body_scene_matches_source_row() -> None:
    import numpy as np

    sx = _require_simulation_experimental_symbols("World")

    from examples.demos.scenes.avbd_demo3d_soft_body import build

    setup = build()
    sx_world = setup.info["sx_world"]
    ground = setup.info["ground"]
    cells = setup.info["cells"]
    grids = setup.info["cell_grids"]
    cell_size = setup.info["cell_size"]
    joints = setup.info["joints"]
    x_joints = setup.info["x_joints"]
    y_joints = setup.info["y_joints"]
    z_joints = setup.info["z_joints"]
    ignored_collision_pairs = setup.info["ignored_collision_pairs"]
    source = setup.info["source_demo_reference"]

    assert setup.info["source_demo_row"] == "avbd-demo3d soft body"
    assert source["demo"] == "avbd-demo3d"
    assert source["revision"] == "7701bd427d55"
    assert source["scene_index"] == 11
    assert source["scene_name"] == "Soft Body"
    assert source["scene_builder"] == "sceneSoftBody"
    assert source["scene_count"] == 14
    assert source["expected_counts"] == {
        "rigid_bodies": 193,
        "dynamic_bodies": 192,
        "static_bodies": 1,
        "joints": 432,
        "fixed_joints": 432,
        "finite_stiffness_fixed_joints": 432,
        "collision_shapes": 193,
    }
    assert source["source_shapes"]["ground"] == {
        "count": 1,
        "density": 0.0,
        "friction": 0.5,
        "position": (0.0, 0.0, 0.0),
        "size": (100.0, 100.0, 1.0),
    }
    assert source["source_shapes"]["soft_cell"] == {
        "base_z": 8.0,
        "count": 192,
        "density": 1.0,
        "depth": 4,
        "first_position": (-1.2, -1.2, 8.0),
        "friction": 0.5,
        "height": 4,
        "size": (0.8, 0.8, 0.8),
        "stack_gap": 2.0,
        "stacks": 3,
        "width": 4,
    }
    assert source["source_constraints"]["finite_all_axis_fixed_joints"] == {
        "angular_stiffness": 250.0,
        "count": 432,
        "fracture": "infinity",
        "linear_stiffness": 1000.0,
        "x_child_anchor": (-0.4, 0.0, 0.0),
        "x_count": 144,
        "x_parent_anchor": (0.4, 0.0, 0.0),
        "y_child_anchor": (0.0, -0.4, 0.0),
        "y_count": 144,
        "y_parent_anchor": (0.0, 0.4, 0.0),
        "z_child_anchor": (0.0, 0.0, -0.4),
        "z_count": 144,
        "z_parent_anchor": (0.0, 0.0, 0.4),
    }
    assert source["source_collision_filters"] == {
        "diagonal_ignore_collision_pairs": 648,
    }
    assert sx_world.num_rigid_bodies == 193
    assert sx_world.num_rigid_body_joints == 432
    assert sx_world.num_rigid_body_fixed_joints == 432
    assert sx_world.time_step == pytest.approx(1.0 / 60.0)
    assert sx_world.gravity.tolist() == pytest.approx([0.0, 0.0, -10.0])
    assert len(cells) == 192
    assert len(grids) == 3
    assert all(len(grid) == 4 for grid in grids)
    assert all(len(yz_plane) == 4 for grid in grids for yz_plane in grid)
    assert all(
        len(column) == 4 for grid in grids for yz_plane in grid for column in yz_plane
    )
    assert len(x_joints) == 144
    assert len(y_joints) == 144
    assert len(z_joints) == 144
    assert len(joints) == 432
    assert len(ignored_collision_pairs) == 648
    assert sx_world.num_ignored_collision_pairs == 648
    assert sx_world.is_collision_pair_ignored(grids[0][0][0][0], grids[0][1][0][1])
    assert sx_world.is_collision_pair_ignored(grids[0][1][0][0], grids[0][0][0][1])
    assert sx_world.is_collision_pair_ignored(grids[0][0][0][0], grids[0][0][1][1])
    assert sx_world.is_collision_pair_ignored(grids[0][0][0][0], grids[0][1][1][0])
    assert not sx_world.is_collision_pair_ignored(
        grids[0][0][0][0], grids[0][1][0][0]
    )
    assert all(joint.type == sx.JointType.FIXED for joint in joints)
    assert [joint.num_dofs for joint in joints[:10]] == [0] * 10
    assert [joint.avbd_linear_stiffness for joint in joints[:10]] == pytest.approx(
        [1000.0] * 10
    )
    assert [joint.avbd_angular_stiffness for joint in joints[:10]] == pytest.approx(
        [250.0] * 10
    )
    assert [len(cell.collision_shapes) for cell in cells] == [1] * 192
    assert [cell.friction for cell in cells[:10]] == pytest.approx([0.5] * 10)
    assert sum(1 for cell in cells if cell.is_static) == 0
    assert ground.is_static
    assert cell_size.tolist() == pytest.approx([0.8, 0.8, 0.8])

    initial_positions = np.array(
        [np.asarray(cell.translation, dtype=float).reshape(3) for cell in cells]
    )
    assert np.isfinite(initial_positions).all()
    assert np.asarray(grids[0][0][0][0].translation).tolist() == pytest.approx(
        [-1.2, -1.2, 8.0]
    )
    assert np.asarray(grids[2][3][3][3].translation).tolist() == pytest.approx(
        [1.2, 1.2, 20.8]
    )

    for _ in range(2):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    final_positions = np.array(
        [np.asarray(cell.translation, dtype=float).reshape(3) for cell in cells]
    )
    assert sx_world.time == pytest.approx(2.0 * sx_world.time_step)
    assert np.isfinite(final_positions).all()


def test_avbd_demo3d_bridge_scene_matches_source_row() -> None:
    import numpy as np

    sx = _require_simulation_experimental_symbols("World")

    from examples.demos.scenes.avbd_demo3d_bridge import build

    setup = build()
    sx_world = setup.info["sx_world"]
    ground = setup.info["ground"]
    planks = setup.info["planks"]
    load_boxes = setup.info["load_boxes"]
    joints = setup.info["joints"]
    source = setup.info["source_demo_reference"]

    assert setup.info["source_demo_row"] == "avbd-demo3d bridge"
    assert source["demo"] == "avbd-demo3d"
    assert source["revision"] == "7701bd427d55"
    assert source["scene_index"] == 12
    assert source["scene_name"] == "Bridge"
    assert source["scene_builder"] == "sceneBridge"
    assert source["scene_count"] == 14
    assert source["expected_counts"] == {
        "rigid_bodies": 91,
        "dynamic_bodies": 88,
        "static_bodies": 3,
        "joints": 78,
        "linear_point_joints": 78,
        "collision_shapes": 91,
    }
    assert source["source_shapes"]["ground"] == {
        "density": 0.0,
        "friction": 0.5,
        "position": (0.0, 0.0, 0.0),
        "size": (100.0, 100.0, 1.0),
    }
    assert source["source_shapes"]["bridge_plank"] == {
        "count": 40,
        "dynamic_density": 1.0,
        "first_position": (-20.0, 0.0, 10.0),
        "friction": 0.5,
        "size": (1.0, 4.0, 0.5),
        "static_endpoint_density": 0.0,
        "static_indices": (0, 39),
        "x_spacing": 1.0,
    }
    assert source["source_shapes"]["load_box"] == {
        "count": 50,
        "density": 1.0,
        "friction": 0.5,
        "size": (1.0, 1.0, 1.0),
        "x_range": (-5.0, 4.0),
        "z_range": (12.0, 16.0),
    }
    assert source["source_constraints"]["linear_point_joints"] == {
        "angular_stiffness": 0.0,
        "child_anchors": ((-0.5, 2.0, 0.0), (-0.5, -2.0, 0.0)),
        "count": 78,
        "fracture": "infinity",
        "linear_stiffness": "infinity",
        "pairs": 39,
        "parent_anchors": ((0.5, 2.0, 0.0), (0.5, -2.0, 0.0)),
    }
    assert sx_world.num_rigid_bodies == 91
    assert sx_world.num_rigid_body_joints == 78
    assert sx_world.num_rigid_body_fixed_joints == 0
    assert sx_world.time_step == pytest.approx(1.0 / 60.0)
    assert sx_world.gravity.tolist() == pytest.approx([0.0, 0.0, -10.0])
    assert len(ground.collision_shapes) == 1
    assert len(planks) == 40
    assert len(load_boxes) == 50
    assert len(joints) == 78
    assert all(joint.type == sx.JointType.SPHERICAL for joint in joints)
    assert [joint.num_dofs for joint in joints] == [3] * 78
    assert [plank.friction for plank in planks] == pytest.approx([0.5] * 40)
    assert planks[0].is_static
    assert planks[-1].is_static
    assert not any(plank.is_static for plank in planks[1:-1])

    initial_plank_positions = np.array(
        [np.asarray(plank.translation, dtype=float).reshape(3) for plank in planks]
    )
    assert initial_plank_positions[:, 0].tolist() == pytest.approx(
        [float(index) - 20.0 for index in range(40)]
    )
    assert initial_plank_positions[:, 2].tolist() == pytest.approx([10.0] * 40)
    initial_load_positions = np.array(
        [np.asarray(box.translation, dtype=float).reshape(3) for box in load_boxes]
    )
    assert initial_load_positions[:, 0].min() == pytest.approx(-5.0)
    assert initial_load_positions[:, 0].max() == pytest.approx(4.0)
    assert initial_load_positions[:, 2].min() == pytest.approx(12.0)
    assert initial_load_positions[:, 2].max() == pytest.approx(16.0)

    parent_anchors = (np.array([0.5, 2.0, 0.0]), np.array([0.5, -2.0, 0.0]))
    child_anchors = (np.array([-0.5, 2.0, 0.0]), np.array([-0.5, -2.0, 0.0]))

    def endpoint_errors() -> list[float]:
        errors = []
        for parent, child in zip(planks, planks[1:]):
            for parent_anchor, child_anchor in zip(parent_anchors, child_anchors):
                parent_point = np.asarray(parent.translation, dtype=float).reshape(
                    3
                ) + np.asarray(parent.rotation, dtype=float) @ parent_anchor
                child_point = np.asarray(child.translation, dtype=float).reshape(
                    3
                ) + np.asarray(child.rotation, dtype=float) @ child_anchor
                errors.append(float(np.linalg.norm(parent_point - child_point)))
        return errors

    assert max(endpoint_errors()) < 1.0e-12
    for _ in range(20):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    final_load_positions = np.array(
        [np.asarray(box.translation, dtype=float).reshape(3) for box in load_boxes]
    )
    assert sx_world.time == pytest.approx(20.0 * sx_world.time_step)
    assert np.isfinite(final_load_positions).all()
    assert final_load_positions[:, 2].mean() < initial_load_positions[:, 2].mean() - 0.1
    assert max(endpoint_errors()) < 1.0


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

    assert callable(setup.info[CAPTURE_METRICS_INFO_KEY])
    capture_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert capture_metrics["row"] == "avbd_rigid_fixed_joint_contact"
    assert capture_metrics["solver"] == "avbd_rigid_joints"
    assert capture_metrics["constraint"] == "fixed_joint_contact_path"
    assert capture_metrics["related_source_row"] == "contact"
    assert capture_metrics["fixed_joint_count"] == pytest.approx(1.0)
    assert capture_metrics["contact_count"] >= 1.0
    assert capture_metrics["captured_offset_error"] < 2.0e-2
    assert capture_metrics["metrics"]["payload_speed"] == pytest.approx(
        capture_metrics["payload_speed"]
    )
    assert capture_metrics["history"]["samples"] >= 1.0
    assert capture_metrics["history"]["max_contact_count"] >= 1.0
    assert np.isfinite(
        [
            float(capture_metrics["captured_offset_error"]),
            float(capture_metrics["payload_ground_clearance"]),
            float(capture_metrics["payload_speed"]),
            float(capture_metrics["base_x"]),
            float(capture_metrics["payload_x"]),
            float(capture_metrics["world_time"]),
        ]
    ).all()


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

    assert callable(setup.info[CAPTURE_METRICS_INFO_KEY])
    capture_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert capture_metrics["row"] == "avbd_rigid_revolute_motor"
    assert capture_metrics["solver"] == "avbd_rigid_joints"
    assert capture_metrics["actuator"] == "revolute_velocity_motor"
    assert capture_metrics["related_source_row"] == "rigid_joint_motor_limits"
    assert capture_metrics["target_speed"] == pytest.approx(target_speed)
    assert capture_metrics["max_torque"] == pytest.approx(max_torque)
    assert capture_metrics["measured_speed"] == pytest.approx(measured_speed)
    assert capture_metrics["abs_speed_error"] == pytest.approx(
        abs(target_speed - measured_speed)
    )
    assert capture_metrics["history"]["samples"] >= 1.0
    assert capture_metrics["history"]["max_measured_speed"] >= measured_speed
    assert np.isfinite(
        [
            float(capture_metrics["measured_speed"]),
            float(capture_metrics["speed_error"]),
            float(capture_metrics["abs_speed_error"]),
            float(capture_metrics["world_time"]),
        ]
    ).all()


def test_avbd_prismatic_motor_demo_drives_slider() -> None:
    import numpy as np

    sx = _require_simulation_symbols("World", "ActuatorType")

    from examples.demos.scenes.avbd_rigid_prismatic_motor import build

    setup = build()
    sx_world = setup.info["sx_world"]
    base = setup.info["base"]
    slider = setup.info["slider"]
    joint = setup.info["joint"]
    target_speed = float(setup.info["target_speed"])
    max_force = float(setup.info["max_force"])
    base_position = np.asarray(setup.info["base_position"], dtype=float).reshape(3)

    assert sx_world.num_rigid_body_joints == 1
    assert joint.actuator_type == sx.ActuatorType.VELOCITY
    assert np.asarray(joint.command_velocity, dtype=float).reshape(1)[
        0
    ] == pytest.approx(target_speed)
    assert np.asarray(joint.effort_lower_limits, dtype=float).reshape(1)[
        0
    ] == pytest.approx(-max_force)
    assert np.asarray(joint.effort_upper_limits, dtype=float).reshape(1)[
        0
    ] == pytest.approx(max_force)

    initial_position = np.asarray(slider.translation, dtype=float).reshape(3)
    for _ in range(40):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    final_position = np.asarray(slider.translation, dtype=float).reshape(3)
    measured_speed = float(np.asarray(slider.linear_velocity, dtype=float).reshape(3)[0])
    assert float(final_position[0]) > float(initial_position[0]) + 0.05
    assert measured_speed == pytest.approx(target_speed, abs=0.45)
    assert np.linalg.norm(final_position[1:] - base_position[1:]) < 1.0e-6
    assert np.linalg.norm(np.asarray(slider.rotation) - np.eye(3)) < 1.0e-6
    assert np.linalg.norm(np.asarray(base.translation) - base_position) < 1.0e-12

    assert callable(setup.info[CAPTURE_METRICS_INFO_KEY])
    capture_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert capture_metrics["row"] == "avbd_rigid_prismatic_motor"
    assert capture_metrics["solver"] == "avbd_rigid_joints"
    assert capture_metrics["actuator"] == "prismatic_velocity_motor"
    assert capture_metrics["related_source_row"] == "rigid_joint_motor_limits"
    assert capture_metrics["target_speed"] == pytest.approx(target_speed)
    assert capture_metrics["max_force"] == pytest.approx(max_force)
    assert capture_metrics["measured_speed"] == pytest.approx(measured_speed)
    assert capture_metrics["abs_speed_error"] == pytest.approx(
        abs(target_speed - measured_speed)
    )
    assert capture_metrics["axis_position"] == pytest.approx(
        float(final_position[0] - base_position[0]), abs=0.01
    )
    assert capture_metrics["orthogonal_drift"] < 1.0e-6
    assert capture_metrics["history"]["samples"] >= 1.0
    assert capture_metrics["history"]["max_axis_position"] >= float(
        final_position[0] - base_position[0]
    )
    assert np.isfinite(
        [
            float(capture_metrics["measured_speed"]),
            float(capture_metrics["speed_error"]),
            float(capture_metrics["abs_speed_error"]),
            float(capture_metrics["axis_position"]),
            float(capture_metrics["orthogonal_drift"]),
            float(capture_metrics["world_time"]),
        ]
    ).all()


def test_avbd_articulated_revolute_motor_demo_reverses_command() -> None:
    import numpy as np

    sx = _require_simulation_experimental_symbols("World", "ActuatorType")

    from examples.demos.scenes.avbd_articulated_revolute_motor import build

    setup = build()
    sx_world = setup.info["sx_world"]
    rotor = setup.info["rotor"]
    joint = setup.info["joint"]
    target_speed = float(setup.info["target_speed"])
    switch_time = float(setup.info["command_switch_time"])

    assert sx_world.num_articulated_joints == 1
    assert sx_world.multibody_options.integration_family == "variational integrator"
    assert joint.actuator_type == sx.ActuatorType.VELOCITY
    assert np.asarray(joint.command_velocity, dtype=float).reshape(1)[0] == pytest.approx(
        target_speed
    )

    def yaw() -> float:
        rotation = np.asarray(rotor.rotation, dtype=float).reshape(3, 3)
        return float(np.arctan2(rotation[1, 0], rotation[0, 0]))

    def step(n: int) -> None:
        for _ in range(n):
            assert setup.pre_step is not None
            setup.pre_step()
            setup.world.step()

    step(20)
    yaw_before_switch = yaw()
    assert sx_world.time < switch_time
    assert yaw_before_switch > 0.04

    step(50)
    reversed_command = np.asarray(joint.command_velocity, dtype=float).reshape(1)[0]
    assert sx_world.time > switch_time
    assert reversed_command == pytest.approx(-target_speed)
    assert yaw() < yaw_before_switch - 0.05
    assert np.linalg.norm(np.asarray(rotor.translation, dtype=float).reshape(3)) < 1.0e-6


def test_avbd_articulated_prismatic_motor_demo_reverses_command() -> None:
    import numpy as np

    sx = _require_simulation_experimental_symbols("World", "ActuatorType")

    from examples.demos.scenes.avbd_articulated_prismatic_motor import build

    setup = build()
    sx_world = setup.info["sx_world"]
    carriage = setup.info["carriage"]
    joint = setup.info["joint"]
    target_speed = float(setup.info["target_speed"])
    switch_time = float(setup.info["command_switch_time"])

    assert sx_world.num_articulated_joints == 1
    assert sx_world.multibody_options.integration_family == "variational integrator"
    assert joint.actuator_type == sx.ActuatorType.VELOCITY
    assert np.asarray(joint.command_velocity, dtype=float).reshape(1)[0] == pytest.approx(
        target_speed
    )

    def position() -> np.ndarray:
        return np.asarray(carriage.translation, dtype=float).reshape(3)

    def step(n: int) -> None:
        for _ in range(n):
            assert setup.pre_step is not None
            setup.pre_step()
            setup.world.step()

    step(20)
    x_before_switch = float(position()[0])
    assert sx_world.time < switch_time
    assert x_before_switch > 0.03

    step(50)
    reversed_command = np.asarray(joint.command_velocity, dtype=float).reshape(1)[0]
    final_position = position()
    assert sx_world.time > switch_time
    assert reversed_command == pytest.approx(-target_speed)
    assert float(final_position[0]) < x_before_switch - 0.03
    assert np.linalg.norm(final_position[1:]) < 1.0e-6
    assert np.linalg.norm(np.asarray(carriage.rotation) - np.eye(3)) < 1.0e-6


def test_avbd_articulated_motor_breakable_joint_demo_resets_motor_rows() -> None:
    import numpy as np

    sx = _require_simulation_experimental_symbols("World", "ActuatorType")

    from examples.demos.scenes.avbd_articulated_motor_breakable_joint import build

    setup = build()
    sx_world = setup.info["sx_world"]
    rotor = setup.info["rotor"]
    joint = setup.info["joint"]
    target_speed = float(setup.info["target_speed"])

    assert sx_world.num_articulated_joints == 1
    assert sx_world.multibody_options.integration_family == "variational integrator"
    assert joint.type == sx.JointType.REVOLUTE
    assert joint.actuator_type == sx.ActuatorType.VELOCITY
    assert joint.break_force == pytest.approx(float(setup.info["break_force"]))
    assert not joint.is_broken

    def translation() -> np.ndarray:
        return np.asarray(rotor.translation, dtype=float).reshape(3)

    def yaw() -> float:
        rotation = np.asarray(rotor.rotation, dtype=float).reshape(3, 3)
        return float(np.arctan2(rotation[1, 0], rotation[0, 0]))

    def step(n: int) -> None:
        for _ in range(n):
            assert setup.pre_step is not None
            setup.pre_step()
            setup.world.step()

    step(1)
    assert joint.is_broken
    assert yaw() == pytest.approx(target_speed * sx_world.time_step, abs=1.0e-6)

    step(30)
    assert np.linalg.norm(translation()) > 1.0e-3

    reset_joint = setup.info["reset_joint"]
    reset_joint(float(setup.info["reset_break_force"]))
    assert not joint.is_broken

    setup.info["set_target_speed"](-target_speed)
    reset_yaw = yaw()

    step(6)
    assert not joint.is_broken
    assert np.asarray(joint.command_velocity, dtype=float).reshape(1)[0] == pytest.approx(
        -target_speed
    )
    assert np.linalg.norm(translation()) < 1.0e-6
    assert yaw() < reset_yaw - 1.0e-3
    axis = np.asarray(rotor.rotation, dtype=float).reshape(3, 3) @ np.array(
        [0.0, 0.0, 1.0]
    )
    assert np.linalg.norm(axis - np.array([0.0, 0.0, 1.0])) < 1.0e-6

    setup.info["rearm_weak_joint"]()
    assert not joint.is_broken
    step(1)
    assert joint.is_broken
    assert np.asarray(joint.command_velocity, dtype=float).reshape(1)[0] == pytest.approx(
        -target_speed
    )


def test_avbd_articulated_prismatic_motor_breakable_joint_demo_resets_rows() -> None:
    import numpy as np

    sx = _require_simulation_experimental_symbols("World", "ActuatorType")

    from examples.demos.scenes.avbd_articulated_prismatic_motor_breakable_joint import (
        build,
    )

    setup = build()
    sx_world = setup.info["sx_world"]
    carriage = setup.info["carriage"]
    joint = setup.info["joint"]
    axis = np.asarray(setup.info["axis"], dtype=float).reshape(3)
    target_speed = float(setup.info["target_speed"])

    assert sx_world.num_articulated_joints == 1
    assert sx_world.multibody_options.integration_family == "variational integrator"
    assert joint.type == sx.JointType.PRISMATIC
    assert joint.num_dofs == 1
    assert joint.child_link == carriage
    with pytest.raises(Exception, match="parent endpoint"):
        _ = joint.parent_link
    assert joint.actuator_type == sx.ActuatorType.VELOCITY
    assert joint.break_force == pytest.approx(float(setup.info["break_force"]))
    assert not joint.is_broken

    def position() -> np.ndarray:
        return np.asarray(carriage.translation, dtype=float).reshape(3)

    def orthogonal_norm() -> float:
        p = position()
        return float(np.linalg.norm(p - float(p @ axis) * axis))

    def step(n: int) -> None:
        for _ in range(n):
            assert setup.pre_step is not None
            setup.pre_step()
            setup.world.step()

    step(1)
    assert joint.is_broken
    first_axis_position = float(position() @ axis)
    assert first_axis_position == pytest.approx(
        target_speed * sx_world.time_step, abs=1.0e-6
    )

    step(30)
    assert orthogonal_norm() > 1.0e-3

    reset_joint = setup.info["reset_joint"]
    reset_joint(float(setup.info["reset_break_force"]))
    assert not joint.is_broken

    setup.info["set_target_speed"](-target_speed)
    reset_axis_position = float(position() @ axis)

    step(6)
    assert not joint.is_broken
    assert np.asarray(joint.command_velocity, dtype=float).reshape(1)[0] == pytest.approx(
        -target_speed
    )
    assert orthogonal_norm() < 1.0e-6
    assert np.linalg.norm(np.asarray(carriage.rotation) - np.eye(3)) < 1.0e-6
    assert float(position() @ axis) < reset_axis_position - 1.0e-3

    setup.info["rearm_weak_joint"]()
    assert not joint.is_broken
    step(1)
    assert joint.is_broken
    assert np.asarray(joint.command_velocity, dtype=float).reshape(1)[0] == pytest.approx(
        -target_speed
    )


def test_avbd_articulated_prismatic_pair_motor_breakable_joint_demo_resets_rows() -> None:
    import numpy as np

    sx = _require_simulation_experimental_symbols("World", "ActuatorType")

    from examples.demos.scenes.avbd_articulated_prismatic_pair_motor_breakable_joint import (
        build,
    )

    setup = build()
    sx_world = setup.info["sx_world"]
    carriage = setup.info["carriage"]
    joint = setup.info["joint"]
    axis = np.asarray(setup.info["axis"], dtype=float).reshape(3)
    target_speed = float(setup.info["target_speed"])

    assert sx_world.num_articulated_joints == 1
    assert sx_world.multibody_options.integration_family == "variational integrator"
    assert joint.type == sx.JointType.PRISMATIC
    assert joint.actuator_type == sx.ActuatorType.VELOCITY
    assert joint.break_force == pytest.approx(float(setup.info["break_force"]))
    assert not joint.is_broken

    def position() -> np.ndarray:
        return np.asarray(carriage.translation, dtype=float).reshape(3)

    def orthogonal_norm() -> float:
        p = position()
        return float(np.linalg.norm(p - float(p @ axis) * axis))

    def step(n: int) -> None:
        for _ in range(n):
            assert setup.pre_step is not None
            setup.pre_step()
            setup.world.step()

    step(1)
    assert joint.is_broken
    first_axis_position = float(position() @ axis)
    assert first_axis_position == pytest.approx(
        target_speed * sx_world.time_step, abs=1.0e-6
    )

    step(30)
    assert orthogonal_norm() > 1.0e-3

    reset_joint = setup.info["reset_joint"]
    reset_joint(float(setup.info["reset_break_force"]))
    assert not joint.is_broken

    setup.info["set_target_speed"](-target_speed)
    reset_axis_position = float(position() @ axis)

    step(6)
    assert not joint.is_broken
    assert np.asarray(joint.command_velocity, dtype=float).reshape(1)[0] == pytest.approx(
        -target_speed
    )
    assert orthogonal_norm() < 1.0e-6
    assert np.linalg.norm(np.asarray(carriage.rotation) - np.eye(3)) < 1.0e-6
    assert float(position() @ axis) < reset_axis_position - 1.0e-3

    setup.info["rearm_weak_joint"]()
    assert not joint.is_broken
    step(1)
    assert joint.is_broken
    assert np.asarray(joint.command_velocity, dtype=float).reshape(1)[0] == pytest.approx(
        -target_speed
    )


def test_avbd_articulated_world_revolute_motor_breakable_joint_demo_resets_rows() -> None:
    import numpy as np

    sx = _require_simulation_experimental_symbols("World", "ActuatorType")

    from examples.demos.scenes.avbd_articulated_world_revolute_motor_breakable_joint import (
        build,
    )

    setup = build()
    sx_world = setup.info["sx_world"]
    rotor = setup.info["rotor"]
    joint = setup.info["joint"]
    target_speed = float(setup.info["target_speed"])

    assert sx_world.num_articulated_joints == 1
    assert sx_world.multibody_options.integration_family == "variational integrator"
    assert joint.type == sx.JointType.REVOLUTE
    assert joint.num_dofs == 1
    assert joint.child_link == rotor
    with pytest.raises(Exception, match="parent endpoint"):
        _ = joint.parent_link
    assert joint.actuator_type == sx.ActuatorType.VELOCITY
    assert joint.break_force == pytest.approx(float(setup.info["break_force"]))
    assert not joint.is_broken

    def translation() -> np.ndarray:
        return np.asarray(rotor.translation, dtype=float).reshape(3)

    def yaw() -> float:
        rotation = np.asarray(rotor.rotation, dtype=float).reshape(3, 3)
        return float(np.arctan2(rotation[1, 0], rotation[0, 0]))

    def step(n: int) -> None:
        for _ in range(n):
            assert setup.pre_step is not None
            setup.pre_step()
            setup.world.step()

    step(1)
    assert joint.is_broken
    assert yaw() == pytest.approx(target_speed * sx_world.time_step, abs=1.0e-6)

    step(30)
    assert np.linalg.norm(translation()) > 1.0e-3

    reset_joint = setup.info["reset_joint"]
    reset_joint(float(setup.info["reset_break_force"]))
    assert not joint.is_broken

    setup.info["set_target_speed"](-target_speed)
    reset_yaw = yaw()

    step(6)
    assert not joint.is_broken
    assert np.asarray(joint.command_velocity, dtype=float).reshape(1)[0] == pytest.approx(
        -target_speed
    )
    assert np.linalg.norm(translation()) < 1.0e-6
    assert yaw() < reset_yaw - 1.0e-3
    axis = np.asarray(rotor.rotation, dtype=float).reshape(3, 3) @ np.array(
        [0.0, 0.0, 1.0]
    )
    assert np.linalg.norm(axis - np.array([0.0, 0.0, 1.0])) < 1.0e-6

    setup.info["rearm_weak_joint"]()
    assert not joint.is_broken
    step(1)
    assert joint.is_broken
    assert np.asarray(joint.command_velocity, dtype=float).reshape(1)[0] == pytest.approx(
        -target_speed
    )


def test_avbd_articulated_high_ratio_chain_demo_swings_and_resets() -> None:
    import numpy as np

    sx = _require_simulation_experimental_symbols("World", "MultibodyOptions")

    from examples.demos.scenes.avbd_articulated_high_ratio_chain import build

    setup = build()
    sx_world = setup.info["sx_world"]
    multibody = setup.info["multibody"]
    links = setup.info["links"]
    joints = setup.info["joints"]
    tip_position = setup.info["tip_position"]
    max_abs_joint_angle = setup.info["max_abs_joint_angle"]
    reset_chain = setup.info["reset_chain"]
    replay_state = setup.info["replay_state"]
    initial_tip = np.asarray(setup.info["initial_tip"], dtype=float).reshape(3)

    assert sx_world.multibody_options.integration_family == "variational integrator"
    assert sx_world.num_articulated_joints == 0
    assert multibody.num_dofs == len(joints)
    assert len(links) == 5
    assert setup.info["mass_ratio"] == pytest.approx(200.0)
    assert all(joint.type == sx.JointType.REVOLUTE for joint in joints)

    replay_state["enabled"] = False
    for _ in range(80):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    swung_tip = np.asarray(tip_position(), dtype=float).reshape(3)
    assert np.all(np.isfinite(swung_tip))
    assert initial_tip[2] - swung_tip[2] > 0.2
    assert max_abs_joint_angle() > 0.1

    reset_chain()
    reset_tip = np.asarray(tip_position(), dtype=float).reshape(3)
    assert np.linalg.norm(reset_tip - initial_tip) < 1.0e-10
    assert max_abs_joint_angle() == pytest.approx(0.0)


def test_avbd_paper_scale_high_ratio_chain_demo_builds_and_resets() -> None:
    import numpy as np

    sx = _require_simulation_experimental_symbols("World", "MultibodyOptions")

    from examples.demos.scenes.avbd_articulated_high_ratio_chain import (
        build_paper_scale,
    )

    setup = build_paper_scale()
    sx_world = setup.info["sx_world"]
    multibody = setup.info["multibody"]
    links = setup.info["links"]
    joints = setup.info["joints"]
    tip_position = setup.info["tip_position"]
    reset_chain = setup.info["reset_chain"]
    initial_tip = np.asarray(setup.info["initial_tip"], dtype=float).reshape(3)

    assert sx_world.multibody_options.integration_family == "variational integrator"
    assert sx_world.multibody_options.variational_max_iterations == 200
    assert sx_world.multibody_options.variational_tolerance == pytest.approx(1.0e-9)
    assert multibody.num_dofs == len(joints)
    assert len(links) == 50
    assert setup.info["mass_ratio"] == pytest.approx(50000.0)
    assert setup.info["auto_reset_seconds"] == pytest.approx(0.16)
    assert all(joint.type == sx.JointType.REVOLUTE for joint in joints)
    assert initial_tip[0] == pytest.approx(22.5)

    for _ in range(4):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    stepped_tip = np.asarray(tip_position(), dtype=float).reshape(3)
    assert np.all(np.isfinite(stepped_tip))
    assert initial_tip[2] - stepped_tip[2] >= 0.0

    reset_chain()
    reset_tip = np.asarray(tip_position(), dtype=float).reshape(3)
    assert np.linalg.norm(reset_tip - initial_tip) < 1.0e-10


def test_avbd_breakable_joint_demo_marks_and_resets_joint() -> None:
    import numpy as np

    _require_simulation_symbols("World")

    from examples.demos.scenes.avbd_rigid_breakable_joint import build

    setup = build()
    sx_world = setup.info["sx_world"]
    joint = setup.info["joint"]
    base = setup.info["base"]
    payload = setup.info["payload"]
    captured_offset = np.asarray(setup.info["captured_offset"], dtype=float).reshape(3)
    captured_rotation = np.asarray(
        setup.info["captured_payload_rotation"], dtype=float
    )
    break_force = float(setup.info["break_force"])

    assert sx_world.num_rigid_body_joints == 1
    assert sx_world.num_rigid_body_fixed_joints == 1
    assert joint.parent_rigid_body == base
    assert joint.child_rigid_body == payload
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
    assert callable(setup.info[CAPTURE_METRICS_INFO_KEY])
    capture_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert capture_metrics["row"] == "avbd_rigid_breakable_joint"
    assert capture_metrics["related_source_row"] == "rigid_joint_breakage"
    assert capture_metrics["metrics"]["status"] == "broken"
    assert capture_metrics["history"]["saw_broken"] == pytest.approx(1.0)

    reset_joint = setup.info["reset_joint"]
    reset_joint(float(setup.info["reset_break_force"]))
    assert not joint.is_broken

    for _ in range(6):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    reset_base_translation = np.asarray(base.translation, dtype=float).reshape(3)
    reset_payload_translation = np.asarray(payload.translation, dtype=float).reshape(3)
    reset_rotation = np.asarray(payload.rotation, dtype=float)
    assert not joint.is_broken
    assert np.linalg.norm(reset_base_translation - initial_base) < 1.0e-9
    assert (
        np.linalg.norm((reset_payload_translation - reset_base_translation) - captured_offset)
        < 1.0e-3
    )
    assert np.linalg.norm(reset_rotation - captured_rotation) < 1.0e-3

    setup.info["rearm_weak_joint"]()
    assert not joint.is_broken
    for _ in range(45):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    assert joint.is_broken


def test_avbd_rigid_spherical_breakable_joint_demo_resets_anchor_only() -> None:
    import numpy as np

    sx = _require_simulation_experimental_symbols("World")

    from examples.demos.scenes.avbd_rigid_spherical_breakable_joint import build

    setup = build()
    sx_world = setup.info["sx_world"]
    joint = setup.info["joint"]
    base = setup.info["base"]
    payload = setup.info["payload"]
    captured_offset = np.asarray(setup.info["captured_offset"], dtype=float).reshape(3)
    captured_rotation = np.asarray(
        setup.info["captured_payload_rotation"], dtype=float
    )

    assert sx_world.num_rigid_body_joints == 1
    assert sx_world.num_rigid_body_fixed_joints == 0
    assert joint.type == sx.JointType.SPHERICAL
    assert joint.num_dofs == 3
    assert joint.parent_rigid_body == base
    assert joint.child_rigid_body == payload
    assert joint.break_force == pytest.approx(float(setup.info["break_force"]))
    assert not joint.is_broken

    for _ in range(45):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    assert joint.is_broken
    base_translation = np.asarray(base.translation, dtype=float).reshape(3)
    payload_translation = np.asarray(payload.translation, dtype=float).reshape(3)
    broken_rotation = np.asarray(payload.rotation, dtype=float)
    assert np.linalg.norm((payload_translation - base_translation) - captured_offset) > (
        1.0e-3
    )
    assert np.linalg.norm(broken_rotation - captured_rotation) > 1.0e-3

    assert callable(setup.info[CAPTURE_METRICS_INFO_KEY])
    capture_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert capture_metrics["row"] == "avbd_rigid_spherical_breakable_joint"
    assert capture_metrics["solver"] == "avbd_rigid_joints"
    assert (
        capture_metrics["constraint"]
        == "spherical_break_force_anchor_lifecycle"
    )
    assert capture_metrics["related_source_row"] == "rigid_joint_breakage"
    assert capture_metrics["status"] == "broken"
    assert capture_metrics["broken"] == pytest.approx(1.0)
    assert capture_metrics["anchor_offset_error"] > 1.0e-3
    assert capture_metrics["orientation_drift"] > 1.0e-3
    assert capture_metrics["history"]["saw_broken"] == pytest.approx(1.0)
    assert capture_metrics["history"]["max_anchor_offset_error"] >= float(
        capture_metrics["anchor_offset_error"]
    )
    assert np.isfinite(
        [
            float(capture_metrics["anchor_offset_error"]),
            float(capture_metrics["orientation_drift"]),
            float(capture_metrics["payload_speed"]),
            float(capture_metrics["payload_height"]),
            float(capture_metrics["world_time"]),
        ]
    ).all()

    reset_joint = setup.info["reset_joint"]
    reset_joint(float(setup.info["reset_break_force"]))
    assert not joint.is_broken

    for _ in range(6):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    reset_base_translation = np.asarray(base.translation, dtype=float).reshape(3)
    reset_payload_translation = np.asarray(payload.translation, dtype=float).reshape(3)
    reset_rotation = np.asarray(payload.rotation, dtype=float)
    assert not joint.is_broken
    assert (
        np.linalg.norm((reset_payload_translation - reset_base_translation) - captured_offset)
        < 1.0e-3
    )
    assert np.linalg.norm(reset_rotation - captured_rotation) > 1.0e-4

    setup.info["rearm_weak_joint"]()
    assert not joint.is_broken
    for _ in range(45):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    assert joint.is_broken


def test_avbd_articulated_breakable_joint_demo_marks_and_resets_joint() -> None:
    import numpy as np

    _require_simulation_experimental_symbols("World", "MultibodyOptions")

    from examples.demos.scenes.avbd_articulated_breakable_joint import build

    setup = build()
    sx_world = setup.info["sx_world"]
    joint = setup.info["joint"]
    payload = setup.info["payload"]
    captured_position = np.asarray(
        setup.info["captured_position"],
        dtype=float,
    ).reshape(3)

    assert sx_world.num_articulated_joints == 1
    assert sx_world.multibody_options.integration_family == "variational integrator"
    assert joint.break_force == pytest.approx(float(setup.info["break_force"]))
    assert not joint.is_broken
    with pytest.raises(Exception, match="parent endpoint"):
        _ = joint.parent_link

    for _ in range(45):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    assert joint.is_broken
    broken_position = np.asarray(payload.translation, dtype=float).reshape(3)
    assert np.linalg.norm(broken_position - captured_position) > 1.0e-3

    setup.info["reset_joint"](float(setup.info["reset_break_force"]))
    assert not joint.is_broken

    for _ in range(4):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    reset_position = np.asarray(payload.translation, dtype=float).reshape(3)
    assert not joint.is_broken
    assert np.linalg.norm(reset_position - captured_position) < 1.0e-5

    setup.info["rearm_weak_joint"]()
    assert not joint.is_broken
    for _ in range(45):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    assert joint.is_broken


def test_avbd_articulated_fixed_pair_breakable_joint_demo_resets_relative_pose() -> None:
    import numpy as np

    sx = _require_simulation_experimental_symbols("World", "MultibodyOptions")

    from examples.demos.scenes.avbd_articulated_fixed_pair_breakable_joint import (
        build,
    )

    setup = build()
    sx_world = setup.info["sx_world"]
    joint = setup.info["joint"]
    base = setup.info["base"]
    payload = setup.info["payload"]
    captured_relative = np.asarray(setup.info["captured_relative"], dtype=float)
    relative_transform = setup.info["relative_transform"]

    assert sx_world.num_articulated_joints == 1
    assert sx_world.multibody_options.integration_family == "variational integrator"
    assert joint.type == sx.JointType.FIXED
    assert joint.num_dofs == 0
    assert joint.parent_link == base
    assert joint.child_link == payload
    assert joint.break_force == pytest.approx(float(setup.info["break_force"]))
    assert not joint.is_broken

    for _ in range(45):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    assert joint.is_broken
    broken_relative = np.asarray(relative_transform(), dtype=float)
    assert np.linalg.norm(broken_relative[:3, 3] - captured_relative[:3, 3]) > 1.0e-3
    assert (
        np.linalg.norm(broken_relative[:3, :3] - captured_relative[:3, :3])
        > 1.0e-3
    )

    reset_joint = setup.info["reset_joint"]
    reset_joint(float(setup.info["reset_break_force"]))
    assert not joint.is_broken

    for _ in range(4):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    reset_relative = np.asarray(relative_transform(), dtype=float)
    assert not joint.is_broken
    assert np.linalg.norm(reset_relative[:3, 3] - captured_relative[:3, 3]) < 1.0e-5
    assert (
        np.linalg.norm(reset_relative[:3, :3] - captured_relative[:3, :3])
        < 1.0e-5
    )

    setup.info["rearm_weak_joint"]()
    assert not joint.is_broken
    for _ in range(45):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    assert joint.is_broken


def test_avbd_articulated_spherical_breakable_joint_demo_resets_anchor_only() -> None:
    import numpy as np

    sx = _require_simulation_experimental_symbols("World", "MultibodyOptions")

    from examples.demos.scenes.avbd_articulated_spherical_breakable_joint import (
        build,
    )

    setup = build()
    sx_world = setup.info["sx_world"]
    joint = setup.info["joint"]
    payload = setup.info["payload"]
    payload_anchor = setup.info["payload_anchor"]
    world_anchor = np.asarray(setup.info["world_anchor"], dtype=float).reshape(3)
    captured_rotation = np.asarray(
        setup.info["captured_rotation"],
        dtype=float,
    )

    assert sx_world.num_articulated_joints == 1
    assert sx_world.multibody_options.integration_family == "variational integrator"
    assert joint.type == sx.JointType.SPHERICAL
    assert joint.num_dofs == 3
    assert joint.break_force == pytest.approx(float(setup.info["break_force"]))
    assert not joint.is_broken
    with pytest.raises(Exception, match="parent endpoint"):
        _ = joint.parent_link

    for _ in range(45):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    assert joint.is_broken
    broken_anchor = np.asarray(payload_anchor(), dtype=float).reshape(3)
    broken_rotation = np.asarray(payload.rotation, dtype=float)
    assert np.linalg.norm(broken_anchor - world_anchor) > 1.0e-3
    assert np.linalg.norm(broken_rotation - captured_rotation) > 1.0e-3

    reset_joint = setup.info["reset_joint"]
    reset_joint(float(setup.info["reset_break_force"]))
    assert not joint.is_broken

    for _ in range(4):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    reset_anchor = np.asarray(payload_anchor(), dtype=float).reshape(3)
    reset_rotation = np.asarray(payload.rotation, dtype=float)
    assert not joint.is_broken
    assert np.linalg.norm(reset_anchor - world_anchor) < 1.0e-5
    assert np.linalg.norm(reset_rotation - captured_rotation) > 1.0e-4

    setup.info["rearm_weak_joint"]()
    assert not joint.is_broken
    for _ in range(45):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    assert joint.is_broken


def test_avbd_articulated_spherical_pair_breakable_joint_demo_resets_anchor_only() -> None:
    import numpy as np

    sx = _require_simulation_experimental_symbols("World", "MultibodyOptions")

    from examples.demos.scenes.avbd_articulated_spherical_pair_breakable_joint import (
        build,
    )

    setup = build()
    sx_world = setup.info["sx_world"]
    joint = setup.info["joint"]
    base = setup.info["base"]
    payload = setup.info["payload"]
    parent_anchor_position = setup.info["parent_anchor_position"]
    payload_anchor = setup.info["payload_anchor"]
    captured_rotation = np.asarray(
        setup.info["captured_rotation"],
        dtype=float,
    )

    assert sx_world.num_articulated_joints == 1
    assert sx_world.multibody_options.integration_family == "variational integrator"
    assert joint.type == sx.JointType.SPHERICAL
    assert joint.num_dofs == 3
    assert joint.parent_link == base
    assert joint.child_link == payload
    assert joint.break_force == pytest.approx(float(setup.info["break_force"]))
    assert not joint.is_broken

    for _ in range(45):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    assert joint.is_broken
    broken_anchor = np.asarray(payload_anchor(), dtype=float).reshape(3)
    broken_parent_anchor = np.asarray(parent_anchor_position(), dtype=float).reshape(3)
    broken_rotation = np.asarray(payload.rotation, dtype=float)
    assert np.linalg.norm(broken_anchor - broken_parent_anchor) > 1.0e-3
    assert np.linalg.norm(broken_rotation - captured_rotation) > 1.0e-3

    reset_joint = setup.info["reset_joint"]
    reset_joint(float(setup.info["reset_break_force"]))
    assert not joint.is_broken

    for _ in range(4):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    reset_anchor = np.asarray(payload_anchor(), dtype=float).reshape(3)
    reset_parent_anchor = np.asarray(parent_anchor_position(), dtype=float).reshape(3)
    reset_rotation = np.asarray(payload.rotation, dtype=float)
    assert not joint.is_broken
    assert np.linalg.norm(reset_anchor - reset_parent_anchor) < 1.0e-5
    assert np.linalg.norm(reset_rotation - captured_rotation) > 1.0e-4

    setup.info["rearm_weak_joint"]()
    assert not joint.is_broken
    for _ in range(45):
        assert setup.pre_step is not None
        setup.pre_step()
        setup.world.step()

    assert joint.is_broken


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


def test_py_demo_capture_records_numbered_rigid_workflow_metrics_artifacts(
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
    output = tmp_path / "numbered_workflow_capture"

    rc = capture_py_demo.main(
        [
            "--scene",
            "rigid_contact_scale_budget",
            "--show-ui",
            "--frames",
            "8",
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
    assert manifest["scene"] == "rigid_contact_scale_budget"
    assert manifest["ui_ready"]["required"] is True
    assert pathlib.Path(manifest["artifacts"]["screenshot"]).is_file()

    evidence = manifest["visual_evidence"]
    for image_key in ("screenshot", "first_frame"):
        image_evidence = evidence[image_key]
        assert image_evidence["width"] == 640
        assert image_evidence["height"] == 360
        assert image_evidence["nonzero_pixels"] > 0
        assert image_evidence["unique_rgb_count"] > 1
        assert image_evidence["docked_workspace"] is True

    scene_metrics = manifest["scene_metrics"]
    assert scene_metrics["event_count"] > 0
    assert pathlib.Path(manifest["artifacts"]["scene_metrics_events"]).is_file()
    latest = scene_metrics["latest"]
    assert latest["scene"] == "rigid_contact_scale_budget"
    metrics = latest["metrics"]
    assert metrics["row"] == "rigid_contact_scale_budget"
    assert set(metrics["lanes"]) == {"single", "medium", "dense"}
    for lane_key, lane_metrics in metrics["lanes"].items():
        assert lane_metrics["target_contacts"] >= 1.0, lane_key
        assert lane_metrics["body_count"] >= 1.0, lane_key
        assert lane_metrics["status"] in {
            "over budget",
            "profiling unavailable",
            "within budget",
        }


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
