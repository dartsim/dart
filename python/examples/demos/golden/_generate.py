"""Generate / regenerate the golden-set fixtures.

Run from the repo root with dartpy on PYTHONPATH:

    PYTHONPATH=build/default/cpp/Release/python:python \\
        python -m examples.demos.golden._generate

Each fixture captures the Python scene's state after
``examples.demos.golden.helpers.DEFAULT_STEPS`` steps. Regenerating is a
deliberate, reviewed change.
"""

from __future__ import annotations

import json
import sys

from ..registry import make_demo_scenes
from ..runner import _step
from .helpers import (
    DEFAULT_STEPS,
    GOLDEN_SCENE_IDS,
    capture_state,
    fixture_path,
)


def regenerate_one(scene_id: str) -> None:
    scenes = {scene.id: scene for scene in make_demo_scenes()}
    if scene_id not in scenes:
        raise SystemExit(f"unknown golden scene id: {scene_id}")
    scene = scenes[scene_id]
    setup = scene.build()
    _step(setup, DEFAULT_STEPS)
    state = capture_state(setup)
    payload = {
        "scene_id": scene.id,
        "scene_title": scene.title,
        "steps": DEFAULT_STEPS,
        "state": state,
    }
    out = fixture_path(scene.id)
    out.write_text(json.dumps(payload, indent=2) + "\n")
    print(f"wrote {out.relative_to(out.parents[3])}")


def main(argv: list[str]) -> int:
    ids = argv[1:] or list(GOLDEN_SCENE_IDS)
    for scene_id in ids:
        regenerate_one(scene_id)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
