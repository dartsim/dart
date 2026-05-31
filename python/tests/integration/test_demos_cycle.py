"""Cycle smoke for the headless `dart-demos` Python runner (PLAN-103 Phase 1).

Asserts the registry has scenes, every scene can build and step a few frames
without crashing, and the runner's `--list` lists the catalog. The runner's
soft-fail path turns an unbuildable scene (e.g. a missing asset) into a logged
skip, so this test also passes when a robot URDF is unavailable; what we
guarantee is that the runner itself stays healthy.
"""

from __future__ import annotations

import json
import pathlib
import signal
import sys
import time

# Put python/ on sys.path so the demos package is importable.
_PYTHON_DIR = pathlib.Path(__file__).resolve().parents[2]
if str(_PYTHON_DIR) not in sys.path:
    sys.path.insert(0, str(_PYTHON_DIR))

import pytest

from examples.demos import make_demo_scenes, run  # noqa: E402


def _gui_run_demos_available() -> bool:
    """True when the dartpy Filament viewer entry point is built.

    Reduced builds disable the GUI (for example the macOS arm64 CI job builds
    with ``DART_BUILD_SIMULATION_EXPERIMENTAL=OFF``, and the GUI depends on the
    experimental target). Without the viewer the runner returns early, so tests
    that exercise the cycle/screenshot paths skip in that configuration.
    """

    try:
        import dartpy as dart
    except Exception:  # pragma: no cover - dartpy import failure
        return False
    return hasattr(dart, "gui") and hasattr(dart.gui, "run_demos")


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


def _simulation_experimental_has(*names: str) -> bool:
    try:
        import dartpy.simulation_experimental as sx
    except Exception:  # pragma: no cover - reduced build without the submodule
        return False
    return all(hasattr(sx, name) for name in names)


def _require_simulation_experimental_symbols(*names: str):
    try:
        import dartpy.simulation_experimental as sx
    except Exception as exc:  # pragma: no cover - reduced build without submodule
        pytest.skip(f"dartpy.simulation_experimental unavailable: {exc}")
    missing = [name for name in names if not hasattr(sx, name)]
    if missing:
        formatted = ", ".join(
            f"simulation_experimental.{name}" for name in missing
        )
        pytest.skip(f"{formatted} unavailable in this build")
    return sx


def _deformable_bindings_available() -> bool:
    """True when the experimental deformable bindings are compiled in.

    Builds with ``DART_BUILD_SIMULATION_EXPERIMENTAL=OFF`` ship a reduced
    ``dartpy.simulation_experimental`` without the deformable types, so the
    solver-free grid-builder check skips there rather than erroring.
    """

    return _simulation_experimental_has("DeformableBodyOptions")


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
        pytest.skip(
            "DeformableBodyOptions unavailable "
            "(experimental simulation disabled in this build)"
        )

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

    sx = _require_simulation_experimental_symbols(
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
    sx = _require_simulation_experimental_symbols("load_obj_triangle_mesh", "World")
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

    _require_simulation_experimental_symbols("load_obj_triangle_mesh", "World")

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

    _require_simulation_experimental_symbols(
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

    _require_simulation_experimental_symbols(
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

    _require_simulation_experimental_symbols(
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

    The five IPC showcases must group under ``IPC Deformable (sx)`` (not the
    general ``Experimental`` sx category), so the viewer renders them together.
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
        assert by_id[scene_id].category == "IPC Deformable (sx)"

    # Every scene in the dedicated category is an IPC deformable scene, and
    # none of them leaked back into the general Experimental category.
    ipc_category = [s.id for s in scenes if s.category == "IPC Deformable (sx)"]
    assert set(ipc_category) == expected_ipc
    experimental = [s.id for s in scenes if s.category == "Experimental"]
    assert not any(s.startswith("ipc_deformable_") for s in experimental)


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
    import numpy as np
    from examples.demos.runner import PythonDemoScene, ScenePanel, SceneSetup

    if not getattr(dart.gui, "is_docking_available", lambda: False)():
        pytest.skip("GUI build does not include ImGui docking support")

    monkeypatch.setenv("LIBGL_ALWAYS_SOFTWARE", "1")
    monkeypatch.setenv("MESA_LOADER_DRIVER_OVERRIDE", "llvmpipe")

    def build_scene() -> SceneSetup:
        world = dart.World("docked_smoke")
        world.set_time_step(0.001)
        frame = dart.SimpleFrame(dart.Frame.world(), "box")
        frame.set_shape(dart.BoxShape(np.array([0.2, 0.2, 0.2])))
        frame.create_visual_aspect().set_color([0.2, 0.7, 0.9])
        world.add_simple_frame(frame)
        panel = ScenePanel(
            "Controls",
            lambda builder, _context: builder.text("dock smoke"),
        )
        return SceneSetup(world=world, panels=[panel])

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


def test_scripted_demo_switch_restores_previous_scene_on_factory_error(
    tmp_path: pathlib.Path,
) -> None:
    if not _gui_run_demos_available():
        pytest.skip("dartpy.gui.run_demos unavailable (GUI not built)")

    import dartpy as dart
    import numpy as np
    from examples.demos.runner import PythonDemoScene, SceneSetup

    def build_good() -> SceneSetup:
        world = dart.World("good")
        world.set_time_step(0.001)
        frame = dart.SimpleFrame(dart.Frame.world(), "good_box")
        frame.set_shape(dart.BoxShape(np.array([0.2, 0.2, 0.2])))
        frame.create_visual_aspect().set_color([0.2, 0.7, 0.9])
        world.add_simple_frame(frame)
        return SceneSetup(world=world)

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
    import numpy as np
    from examples.demos.runner import PythonDemoScene, SceneSetup

    def build_good() -> SceneSetup:
        world = dart.World("good")
        world.set_time_step(0.001)
        frame = dart.SimpleFrame(dart.Frame.world(), "good_box")
        frame.set_shape(dart.BoxShape(np.array([0.2, 0.2, 0.2])))
        frame.create_visual_aspect().set_color([0.2, 0.7, 0.9])
        world.add_simple_frame(frame)
        return SceneSetup(world=world)

    def build_slow() -> SceneSetup:
        time.sleep(0.02)
        world = dart.World("slow")
        world.set_time_step(0.001)
        return SceneSetup(world=world)

    monkeypatch.setenv("DART_DEMO_SCENE_STARTUP_TIMEOUT_MS", "1")
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
    assert (
        "factory startup exceeded budget"
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
    import numpy as np
    from examples.demos.runner import PythonDemoScene, SceneSetup

    def build_good() -> SceneSetup:
        world = dart.World("good")
        world.set_time_step(0.001)
        frame = dart.SimpleFrame(dart.Frame.world(), "good_box")
        frame.set_shape(dart.BoxShape(np.array([0.2, 0.2, 0.2])))
        frame.create_visual_aspect().set_color([0.2, 0.7, 0.9])
        world.add_simple_frame(frame)
        return SceneSetup(world=world)

    def build_stalled() -> SceneSetup:
        time.sleep(60.0)
        world = dart.World("stalled")
        world.set_time_step(0.001)
        return SceneSetup(world=world)

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
