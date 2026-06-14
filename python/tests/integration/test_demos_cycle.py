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
import signal
import sys
import time
from typing import Any

# Put python/ on sys.path so the demos package is importable.
_PYTHON_DIR = pathlib.Path(__file__).resolve().parents[2]
if str(_PYTHON_DIR) not in sys.path:
    sys.path.insert(0, str(_PYTHON_DIR))

import pytest

from examples.demos import make_demo_scenes, run  # noqa: E402


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
    rc = run(["--cycle-scenes", "--frames", "2", "--headless"], make_demo_scenes())
    assert rc == 0


def test_runner_list_prints_catalog(capsys: pytest.CaptureFixture[str]) -> None:
    rc = run(["--list"], make_demo_scenes())
    assert rc == 0
    captured = capsys.readouterr().out
    for scene in make_demo_scenes():
        assert scene.id in captured


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
            "rigid_limited_joints",
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
    png_frames = sorted((output / "png_frames").glob("frame_*.png"))
    assert png_frames
    ppm_frames = sorted((output / "frames").glob("frame_*.ppm"))
    assert [frame.name for frame in ppm_frames] == [
        f"frame_{index:06d}.ppm" for index in range(1, len(ppm_frames) + 1)
    ]
    assert capture_py_demo.ppm_has_docked_workspace_regions(ppm_frames[0])

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
