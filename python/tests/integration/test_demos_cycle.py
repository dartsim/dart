"""Cycle smoke for the headless `dart-demos` Python runner (PLAN-103 Phase 1).

Asserts the registry has scenes, every scene can build and step a few frames
without crashing, and the runner's `--list` lists the catalog. The runner's
soft-fail path turns an unbuildable scene (e.g. a missing asset) into a logged
skip, so this test also passes when a robot URDF is unavailable; what we
guarantee is that the runner itself stays healthy.
"""

from __future__ import annotations

import pathlib
import sys

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


def _deformable_bindings_available() -> bool:
    """True when the experimental deformable bindings are compiled in.

    Builds with ``DART_BUILD_SIMULATION_EXPERIMENTAL=OFF`` ship a reduced
    ``dartpy.simulation_experimental`` without the deformable types, so the
    solver-free grid-builder check skips there rather than erroring.
    """

    try:
        import dartpy.simulation_experimental as sx
    except Exception:  # pragma: no cover - reduced build without the submodule
        return False
    return hasattr(sx, "DeformableBodyOptions")


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
        "ipc_deformable_fem_drop",
        "ipc_deformable_fem_sphere",
        "ipc_deformable_fem_box",
        "ipc_deformable_fem_msh",
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
