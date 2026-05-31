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
runs inside the interactive loop. `SceneSetup.step` remains a headless
whole-step escape hatch for legacy parity tests; interactive demos should use
`pre_step` so the viewer owns `world.step()`.
"""

from __future__ import annotations

import argparse
import contextlib
import os
import signal
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Iterable

# Frames-per-scene defaults match the C++ host so cycle behavior is identical.
CYCLE_FRAMES_PER_SCENE = 4
SINGLE_SCENE_DEFAULT_FRAMES = 60
SCENE_BUILD_TIMEOUT_ENV = "DART_PY_DEMO_SCENE_BUILD_TIMEOUT_MS"
DEMO_SCENE_STARTUP_TIMEOUT_ENV = "DART_DEMO_SCENE_STARTUP_TIMEOUT_MS"
DEFAULT_SCENE_BUILD_TIMEOUT_MS = 5000.0


@dataclass
class ScenePanel:
    """A renderer-neutral custom panel owned by a Python demo scene.

    ``build(builder, context)`` is called once per UI frame. ``builder`` is the
    DART panel abstraction (text, buttons, sliders, plots, tables, etc.);
    ``context`` is a read-only snapshot of viewer state. Scene panels should
    capture scene-owned state explicitly instead of mutating ``context``.
    """

    title: str
    build: Callable[[Any, Any], None]
    dock_side: str = "right"
    initial_size: tuple[float, float] | None = (320.0, 440.0)
    initial_position: tuple[float, float] | None = None
    auto_resize: bool = False
    background_alpha: float | None = None
    horizontal_scrollbar: bool = False
    menu_bar: bool = False


@dataclass
class SceneSetup:
    """The per-scene state returned by a scene's ``build()``.

    ``world`` is whatever the scene built (a ``dartpy.World`` or an
    ``sx.World``); both support ``.step()``.

    ``pre_step`` is an optional callable invoked before each viewer step
    (controllers, sensor updates). It receives no arguments and returns
    nothing.

    ``step`` is the legacy whole-loop variant: ``step(frames)`` advances the
    world by ``frames`` steps in headless runner paths. The interactive viewer
    does not use it; interactive controllers should use ``pre_step``.

    ``force_drag`` is an optional callable invoked by the viewer's mouse
    "force-drag" while the user left-drags one of this scene's renderables that
    is not backed by a legacy BodyNode (e.g. the sx SimpleFrame mirrors). It
    receives a single event mapping with keys ``renderable_id`` (int),
    ``renderable_name`` (str), ``application_point`` (np.ndarray, world frame),
    ``force`` (np.ndarray, world frame), and ``active`` (bool), and returns
    nothing. The handler must (re)apply the one-shot force on every step while
    ``active`` is true and stop once an event with ``active`` false arrives.

    ``panels`` carries optional custom UI panels for scene-specific controls
    and diagnostics. Panels are rendered through DART's renderer-neutral
    ``PanelBuilder`` abstraction and dock on the right by default.

    ``info`` carries scene-specific metadata (e.g. ``golden_skeletons``).
    """

    world: Any
    pre_step: Callable[[], None] | None = None
    step: Callable[[int], None] | None = None
    force_drag: Callable[[dict[str, Any]], None] | None = None
    panels: list[ScenePanel] = field(default_factory=list)
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

    Honors (in order of precedence):
      1. SceneSetup.step (legacy whole-loop callable, owns world.step()
         itself — used by the OSC/rigid_loop golden-parity smokes).
      2. SceneSetup.pre_step (run the controller body, then world.step()
         here). Same pattern the interactive viewer uses, so the same
         scene runs in both surfaces without duplicate logic.
      3. Plain world.step() per frame.
    """

    if setup.step is not None:
        setup.step(frames)
        return
    pre = setup.pre_step
    for _ in range(max(0, frames)):
        if pre is not None:
            pre()
        setup.world.step()


def _print_catalog(scenes: list[PythonDemoScene]) -> None:
    last_category: str | None = None
    for entry in scenes:
        if entry.category != last_category:
            print(f"\n[{entry.category}]")
            last_category = entry.category
        print(f"  {entry.id:<28s} {entry.title} — {entry.summary}")


def _canonical_scene_id(scene_id: str) -> str:
    return scene_id.replace("-", "_")


def _validate_scene(scene_id: str | None, scenes: list[PythonDemoScene]) -> None:
    if scene_id is None:
        return
    canonical = _canonical_scene_id(scene_id)
    if not any(entry.id == canonical for entry in scenes):
        available = ", ".join(entry.id for entry in scenes)
        raise SystemExit(
            f"unknown --scene '{scene_id}'. Known scenes: {available}"
        )


def _scene_build_timeout_ms() -> float | None:
    value = os.environ.get(SCENE_BUILD_TIMEOUT_ENV)
    python_specific_override = value is not None and value != ""
    if value is None or value == "":
        value = os.environ.get(DEMO_SCENE_STARTUP_TIMEOUT_ENV)
    if value is None or value == "":
        return DEFAULT_SCENE_BUILD_TIMEOUT_MS

    try:
        timeout_ms = float(value)
    except ValueError:
        return DEFAULT_SCENE_BUILD_TIMEOUT_MS
    if timeout_ms <= 0.0:
        if python_specific_override:
            return None
        return DEFAULT_SCENE_BUILD_TIMEOUT_MS
    return timeout_ms


@contextlib.contextmanager
def _bounded_scene_callback(scene_id: str, callback_name: str):
    timeout_ms = _scene_build_timeout_ms()
    if (
        timeout_ms is None
        or threading.current_thread() is not threading.main_thread()
        or not hasattr(signal, "SIGALRM")
        or not hasattr(signal, "ITIMER_REAL")
        or not hasattr(signal, "getitimer")
        or not hasattr(signal, "setitimer")
    ):
        yield
        return

    previous_handler = signal.getsignal(signal.SIGALRM)
    previous_timer = signal.getitimer(signal.ITIMER_REAL)
    start = time.monotonic()

    def _handle_timeout(_signum: int, _frame: object) -> None:
        raise TimeoutError(
            f"Python demo scene '{scene_id}' {callback_name} exceeded "
            f"{timeout_ms:g} ms"
        )

    signal.signal(signal.SIGALRM, _handle_timeout)
    signal.setitimer(signal.ITIMER_REAL, timeout_ms / 1000.0)
    try:
        yield
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0.0)
        signal.signal(signal.SIGALRM, previous_handler)
        previous_delay, previous_interval = previous_timer
        if previous_delay > 0.0:
            elapsed = time.monotonic() - start
            signal.setitimer(
                signal.ITIMER_REAL,
                max(1.0e-6, previous_delay - elapsed),
                previous_interval,
            )


@contextlib.contextmanager
def _bounded_scene_build(scene_id: str):
    with _bounded_scene_callback(scene_id, "build"):
        yield


def _make_world_factory(scene: PythonDemoScene) -> Callable[[], Any]:
    """Wrap scene.build() so dart.gui.run_demos can call it as a factory.

    Returns one of:
      * a bare ``dartpy.World`` (physics-only), or
      * a ``(World, pre_step)`` tuple, or
      * a ``(World, pre_step, force_drag)`` tuple, or
      * a ``(World, pre_step, force_drag, panels)`` tuple.

    The viewer binding inspects the return value: a bare World is treated as
    physics-only; element 1 (if not ``None``) registers as the viewer's per-step
    hook; element 2 (if not ``None``) registers as the viewer's mouse
    force-drag handler; element 3 (if not ``None``) registers scene-specific
    panels. These are used by experimental-world scenes that own an sx::World
    and a render-mirror dart::simulation::World.
    """

    def factory() -> Any:
        with _bounded_scene_build(scene.id):
            setup = scene.build()
        pre_step = setup.pre_step
        if pre_step is not None:
            original_pre_step = pre_step

            def bounded_pre_step() -> None:
                with _bounded_scene_callback(scene.id, "pre_step"):
                    original_pre_step()

            pre_step = bounded_pre_step
        if setup.panels:
            return (setup.world, pre_step, setup.force_drag, setup.panels)
        if setup.force_drag is not None:
            return (setup.world, pre_step, setup.force_drag)
        if pre_step is not None:
            return (setup.world, pre_step)
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
