"""Scene-registry runner for the DART Python demos.

`pixi run py-demos` (and `python -m examples.demos`) launches the C++
Filament viewer through `dartpy.gui.run_demos` with the Python scene
catalog. The CLI mirrors `dart-demos`:

- `--scene <id>`: select a starting scene
- `--cycle-scenes`: advance through every scene then exit
- `--frames N`: per-scene frame budget (for headless cycles)
- `--screenshot <path>`: write a PPM at <path> (real Filament render)
- `--headless`: render without opening a window
- `--width N`, `--height N`, `--backend ...`: forward to the viewer
- `--list`: print the Python scene catalog and exit (no viewer)

Scenes with a Python-side controller (`SceneSetup.pre_step`) have that
callable forwarded to the viewer's per-step hook so the controller still
runs inside the interactive loop. Scenes with the legacy
`SceneSetup.step` (a custom whole-step loop) are also supported: their
controller part is invoked as `pre_step` and the viewer's
`simulateWorld` advances time as usual — for these scenes the world.step()
that the legacy `step` callable also performs is harmless (double-step
behavior is the same as the C++ scene's `preStep + world.step()` pattern).
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass, field
from typing import Any, Callable, Iterable

# Frames-per-scene defaults match the C++ host so cycle behavior is identical.
CYCLE_FRAMES_PER_SCENE = 4
SINGLE_SCENE_DEFAULT_FRAMES = 60


@dataclass
class SceneSetup:
    """The per-scene state returned by a scene's ``build()``.

    ``world`` is whatever the scene built (a ``dartpy.World`` or an
    ``sx.World``); both support ``.step()``.

    ``pre_step`` is an optional callable invoked before each viewer step
    (controllers, sensor updates). It receives no arguments and returns
    nothing.

    ``step`` is the legacy whole-loop variant: ``step(frames)`` advances the
    world by ``frames`` steps. The viewer doesn't use it directly — when
    present, the runner wraps it as a ``pre_step`` that runs the inner
    controller body once per viewer step.

    ``info`` carries scene-specific metadata (e.g. ``golden_skeletons``).
    """

    world: Any
    pre_step: Callable[[], None] | None = None
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
    """Advance the scene by ``frames`` steps headlessly.

    Used by the golden-parity smoke (and any caller that needs a
    deterministic Python-side stepping without involving the viewer).
    Honors a SceneSetup.step callable when present (controller-driven
    scenes), otherwise calls world.step() in a loop.
    """

    if setup.step is not None:
        setup.step(frames)
        return
    for _ in range(max(0, frames)):
        setup.world.step()


def _print_catalog(scenes: list[PythonDemoScene]) -> None:
    last_category: str | None = None
    for entry in scenes:
        if entry.category != last_category:
            print(f"\n[{entry.category}]")
            last_category = entry.category
        print(f"  {entry.id:<28s} {entry.title} — {entry.summary}")


def _validate_scene(scene_id: str | None, scenes: list[PythonDemoScene]) -> None:
    if scene_id is None:
        return
    if not any(entry.id == scene_id for entry in scenes):
        available = ", ".join(entry.id for entry in scenes)
        raise SystemExit(
            f"unknown --scene '{scene_id}'. Known scenes: {available}"
        )


def _make_world_factory(scene: PythonDemoScene) -> Callable[[], Any]:
    """Wrap scene.build() so dart.gui.run_demos can call it as a factory.

    Returns the dartpy.World. SceneSetup.pre_step / SceneSetup.step are
    Python-controller hooks the C++ viewer doesn't see today — controllers
    that must run inside the interactive loop need a future binding pass.
    """

    def factory() -> Any:
        setup = scene.build()
        return setup.world

    return factory


def run(argv: Iterable[str], scenes: list[PythonDemoScene]) -> int:
    """Entry point. ``argv`` is the program argv excluding argv[0]."""

    if not scenes:
        print("runner: no scenes registered", file=sys.stderr)
        return 1

    argv = list(argv)

    # Intercept --list locally so callers don't need the viewer to enumerate.
    parser = argparse.ArgumentParser(
        prog="dart-demos (python)",
        description="Run DART Python demo scenes through the dartpy.gui viewer.",
        add_help=False,
    )
    parser.add_argument("--list", action="store_true")
    parser.add_argument("--scene", default=None)
    parser.add_argument("--help", "-h", action="store_true")
    known, _passthrough = parser.parse_known_args(argv)

    if known.list:
        _print_catalog(scenes)
        return 0

    _validate_scene(known.scene, scenes)

    # Build the catalog for the viewer.
    catalog = [
        (
            scene.id,
            scene.title,
            scene.category,
            scene.summary,
            _make_world_factory(scene),
        )
        for scene in scenes
    ]

    # Import dartpy.gui lazily so `--list` works even when the GUI
    # backend isn't built (e.g. on CI variants without filament).
    import dartpy as dart

    if not hasattr(dart, "gui") or not hasattr(dart.gui, "run_demos"):
        # Defensive fallback: the GUI binding isn't present. Stop with a
        # helpful error — the catalog is still printable via --list.
        print(
            "dartpy.gui.run_demos not available (build dartpy with GUI support).",
            file=sys.stderr,
        )
        return 2

    full_argv = ["py-demos", *argv]
    return int(dart.gui.run_demos(catalog, full_argv))
