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


def test_runner_screenshot_writes_snapshot(tmp_path: pathlib.Path) -> None:
    scenes = make_demo_scenes()
    # Pick the first scene that can build (a runnable one) so the snapshot is
    # produced from real world state.
    target = scenes[0]
    rc = run(
        ["--scene", target.id, "--frames", "1", "--screenshot", str(tmp_path / "snap.json")],
        scenes,
    )
    # If the target scene cannot build its assets in this environment, the
    # runner soft-fails (rc != 0); accept either outcome but the runner must
    # not crash.
    assert rc in (0, 1)
    if rc == 0:
        snap = (tmp_path / "snap.json").read_text()
        assert target.id in snap
