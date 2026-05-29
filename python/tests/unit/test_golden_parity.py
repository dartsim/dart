"""Python side of the dart-demos golden-set parity smoke (PLAN-103 Phase 2).

For every golden scene id, run the Python scene deterministically for
``DEFAULT_STEPS`` steps and assert its state matches the shared fixture in
``python/examples/demos/golden/<id>.json``. The same fixtures are asserted from
the C++ side (see ``tests/unit/gui/test_demos_golden_parity.cpp``), so drift in
either language fails its smoke without cross-process comparison.
"""

from __future__ import annotations

import pathlib
import sys

_PYTHON_DIR = pathlib.Path(__file__).resolve().parents[2]
if str(_PYTHON_DIR) not in sys.path:
    sys.path.insert(0, str(_PYTHON_DIR))

import pytest

from examples.demos.golden import (  # noqa: E402
    DEFAULT_STEPS,
    GOLDEN_SCENE_IDS,
    capture_state,
    compare_states,
    load_fixture,
)
from examples.demos.registry import make_demo_scenes  # noqa: E402
from examples.demos.runner import _step  # noqa: E402


@pytest.mark.parametrize("scene_id", GOLDEN_SCENE_IDS)
def test_golden_scene_matches_fixture(scene_id: str) -> None:
    scenes = {scene.id: scene for scene in make_demo_scenes()}
    assert scene_id in scenes, (
        f"golden scene '{scene_id}' is not in the demos registry"
    )
    scene = scenes[scene_id]
    setup = scene.build()
    _step(setup, DEFAULT_STEPS)
    actual = capture_state(setup)

    fixture = load_fixture(scene_id)
    assert fixture["steps"] == DEFAULT_STEPS
    expected = fixture["state"]

    errors = compare_states(actual, expected)
    assert not errors, "\n".join(errors)
