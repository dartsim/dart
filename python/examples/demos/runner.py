"""Headless scene-registry runner for the DART Python demos.

CLI mirrors C++ ``dart-demos`` (PLAN-102/103):

- ``--scene <id>``: run a single scene (default: the first scene).
- ``--cycle-scenes``: render a few frames of each scene then exit.
- ``--frames N``: per-scene step budget (default 60 single, 4 cycle).
- ``--screenshot <path>``: writes a JSON state snapshot at <path> for the
  active/last scene. A real PPM screenshot via ``dartpy.gui`` is Phase 2 work.
- ``--headless``: accepted as an explicit no-op; Python demos are always
  headless (there is no interactive ``dartpy`` viewer binding by design).
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Iterable

# Match the C++ host's per-scene budget in --cycle-scenes mode.
CYCLE_FRAMES_PER_SCENE = 4
SINGLE_SCENE_DEFAULT_FRAMES = 60


@dataclass
class SceneSetup:
    """The per-scene state returned by a scene's ``build()``.

    ``world`` is whatever the scene built (legacy ``dartpy.World`` or an
    ``sx.World``); both support ``.step()``. ``step`` is an optional custom
    callable taking the number of frames to advance; if omitted, the runner
    calls ``world.step()`` in a loop. ``info`` carries scene-specific data the
    runner or tests can inspect.
    """

    world: Any
    step: Callable[[int], None] | None = None
    info: dict[str, Any] = field(default_factory=dict)


@dataclass
class PythonDemoScene:
    """One entry in the demos catalog."""

    id: str
    title: str
    category: str
    summary: str
    build: Callable[[], SceneSetup]


def _step(setup: SceneSetup, frames: int) -> None:
    if setup.step is not None:
        setup.step(frames)
        return
    for _ in range(max(0, frames)):
        setup.world.step()


def _world_state_snapshot(world: Any) -> dict[str, Any]:
    """Best-effort JSON-serializable summary of a world's state (used by
    --screenshot in Phase 1; replaced by a real PPM in Phase 2 once the
    dartpy.gui headless screenshot path is wired)."""

    snapshot: dict[str, Any] = {}
    for attr, key in (("get_num_skeletons", "num_skeletons"),
                      ("num_rigid_bodies", "num_rigid_bodies"),
                      ("num_multibodies", "num_multibodies"),
                      ("get_time", "sim_time"),
                      ("time", "time")):
        getter = getattr(world, attr, None)
        if callable(getter):
            try:
                snapshot[key] = getter()
            except Exception:  # noqa: BLE001
                pass
    return snapshot


def _write_screenshot(setup: SceneSetup, scene: PythonDemoScene, path: str) -> None:
    """Write a deterministic state snapshot at ``path`` so the --screenshot
    contract works in Phase 1. Phase 2 replaces this with a non-blank PPM
    captured through dartpy.gui."""

    payload = {
        "scene_id": scene.id,
        "scene_title": scene.title,
        "phase1_note": (
            "Real PPM screenshot via dartpy.gui is Phase 2; this is a"
            " deterministic JSON state snapshot proving the --screenshot path."
        ),
        "world_state": _world_state_snapshot(setup.world),
    }
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(payload, indent=2, default=str))


def _index_for(scenes: list[PythonDemoScene], scene_id: str | None) -> int:
    if not scene_id:
        return 0
    for i, entry in enumerate(scenes):
        if entry.id == scene_id:
            return i
    available = ", ".join(entry.id for entry in scenes)
    raise SystemExit(
        f"unknown --scene '{scene_id}'. Known scenes: {available}"
    )


def _parse(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="dart-demos (python)",
        description="Run DART Python demo scenes headless.",
    )
    parser.add_argument("--scene", default=None,
                        help="Scene id to run (default: first registered)")
    parser.add_argument("--cycle-scenes", action="store_true",
                        help="Cycle through every scene and exit")
    parser.add_argument("--frames", type=int, default=None,
                        help="Per-scene step budget")
    parser.add_argument("--screenshot", default=None,
                        help="Write a Phase-1 JSON state snapshot at this path")
    parser.add_argument("--headless", action="store_true",
                        help="Accepted no-op (Python demos are always headless)")
    parser.add_argument("--list", action="store_true",
                        help="Print the scene catalog and exit")
    return parser.parse_args(argv)


def _print_catalog(scenes: list[PythonDemoScene]) -> None:
    last_category: str | None = None
    for entry in scenes:
        if entry.category != last_category:
            print(f"\n[{entry.category}]")
            last_category = entry.category
        print(f"  {entry.id:<28s} {entry.title} — {entry.summary}")


def _build_safe(scene: PythonDemoScene) -> SceneSetup | None:
    try:
        return scene.build()
    except Exception as error:  # noqa: BLE001
        print(
            f"demo scene '{scene.id}' failed to build: {type(error).__name__}: {error}",
            file=sys.stderr,
        )
        return None


def run(argv: Iterable[str], scenes: list[PythonDemoScene]) -> int:
    """Entry point. ``argv`` is the program's argv excluding argv[0]."""

    if not scenes:
        print("runner: no scenes registered", file=sys.stderr)
        return 1

    args = _parse(list(argv))
    if args.list:
        _print_catalog(scenes)
        return 0

    if args.cycle_scenes:
        per_scene = args.frames if args.frames is not None else CYCLE_FRAMES_PER_SCENE
        last_setup: SceneSetup | None = None
        last_scene: PythonDemoScene | None = None
        for scene in scenes:
            print(f"[cycle] {scene.id}: building ...")
            setup = _build_safe(scene)
            if setup is None:
                continue
            print(f"[cycle] {scene.id}: stepping {per_scene} frames")
            _step(setup, per_scene)
            last_setup, last_scene = setup, scene
        if args.screenshot and last_setup is not None and last_scene is not None:
            _write_screenshot(last_setup, last_scene, args.screenshot)
        return 0

    index = _index_for(scenes, args.scene)
    scene = scenes[index]
    frames = args.frames if args.frames is not None else SINGLE_SCENE_DEFAULT_FRAMES
    print(f"[demos] running '{scene.id}' for {frames} frames")
    setup = _build_safe(scene)
    if setup is None:
        return 1
    _step(setup, frames)
    if args.screenshot:
        _write_screenshot(setup, scene, args.screenshot)
        print(f"[demos] state snapshot -> {args.screenshot}")
    return 0


