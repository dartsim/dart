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
- `--gpu` / `--no-gpu`: force GPU (CUDA) deformable solve on/off; the default
  ("auto") enables it when a CUDA device is available (e.g. under
  `pixi run -e cuda py-demos`). `DART_PY_DEMOS_GPU=on|off|auto` overrides the
  default. The GPU offload covers the deformable projected-Newton PSD
  projection and is a process-wide toggle (also exposed in an in-viewer panel).

Scenes with a Python-side controller (`SceneSetup.pre_step`) have that
callable forwarded to the viewer's per-step hook so the controller still
runs inside the interactive loop. `SceneSetup.step` remains a headless
whole-step escape hatch for legacy parity tests; interactive demos should use
`pre_step` so the viewer owns `world.step()`.
"""

from __future__ import annotations

import argparse
import contextlib
import copy
import os
import signal
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Iterable, Mapping

# Frames-per-scene defaults match the C++ host so cycle behavior is identical.
CYCLE_FRAMES_PER_SCENE = 4
SINGLE_SCENE_DEFAULT_FRAMES = 60
DEFAULT_INITIAL_SCENE_ID = "replay_scrubber"
SCENE_BUILD_TIMEOUT_ENV = "DART_PY_DEMO_SCENE_BUILD_TIMEOUT_MS"
DEMO_SCENE_STARTUP_TIMEOUT_ENV = "DART_DEMO_SCENE_STARTUP_TIMEOUT_MS"
DEFAULT_SCENE_BUILD_TIMEOUT_MS = 5000.0
DEFAULT_REPLAY_MAX_FRAMES = 900
REPLAY_WORLD_INFO_KEYS = ("sx_world", "physics_world")
_REPLAY_RATE_LABELS = (
    "1 frame/tick",
    "2 frames/tick",
    "4 frames/tick",
    "8 frames/tick",
)
_REPLAY_RATE_STEPS = (1, 2, 4, 8)


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

    ``world`` is the scene's headless stepping object. It may be ``None`` for
    descriptor-only scenes.

    ``pre_step`` is an optional callable invoked before each viewer step
    (controllers, sensor updates). It receives no arguments and returns
    nothing.

    ``step`` is the whole-loop variant: ``step(frames)`` advances the world by
    ``frames`` steps in headless runner paths. The interactive viewer does not
    use it; interactive controllers should use ``pre_step``.

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

    ``renderable_provider`` returns the current GUI descriptors for interactive
    rendering. When omitted, the runner also checks whether ``world`` provides a
    ``renderable_provider`` method.

    ``debug_provider`` returns renderer-neutral debug overlay geometry for the
    current frame. The provider returns ``dart.gui.DebugScene`` and is wired into
    the built-in viewer overlay instead of adding ad-hoc renderables.

    ``info`` carries scene-specific metadata for tests and panels.
    """

    world: Any | None = None
    pre_step: Callable[[], None] | None = None
    step: Callable[[int], None] | None = None
    force_drag: Callable[[dict[str, Any]], None] | None = None
    panels: list[ScenePanel] = field(default_factory=list)
    renderable_provider: Callable[[], list[Any]] | None = None
    debug_provider: Callable[[], Any] | None = None
    info: dict[str, Any] = field(default_factory=dict)


@dataclass
class PythonDemoScene:
    """One entry in the demos catalog."""

    id: str
    title: str
    category: str
    summary: str
    build: Callable[[], SceneSetup]


def _has_world_replay_api(world: Any) -> bool:
    return all(
        hasattr(world, name)
        for name in (
            "replay_recording_enabled",
            "replay_frame_count",
            "replay_cursor",
            "restore_replay_frame",
            "clear_replay_recording",
            "get_replay_frame_time",
            "get_replay_simulation_frame",
        )
    )


def _replay_world_from_setup(setup: SceneSetup) -> Any | None:
    for key in REPLAY_WORLD_INFO_KEYS:
        world = setup.info.get(key)
        if world is not None and _has_world_replay_api(world):
            return world
    if _has_world_replay_api(setup.world):
        return setup.world
    return None


def _sync_callback_from_pre_step(
    pre_step: Callable[[], None] | None, replay_world: Any
) -> Callable[[], None] | None:
    """Find a WorldRenderBridge.sync callback captured by a scene pre-step."""

    def sync_from_object(value: Any) -> Callable[[], None] | None:
        if (
            getattr(value, "_physics_world", None) is replay_world
            and hasattr(value, "sync")
        ):
            return getattr(value, "sync")
        return None

    if pre_step is None:
        return None

    bound_sync = sync_from_object(getattr(pre_step, "__self__", None))
    if bound_sync is not None:
        return bound_sync

    for cell in getattr(pre_step, "__closure__", ()) or ():
        try:
            value = cell.cell_contents
        except ValueError:
            continue
        cell_sync = sync_from_object(value)
        if cell_sync is not None:
            return cell_sync
    return None


def _is_bridge_pre_step(pre_step: Callable[[], None] | None, replay_world: Any) -> bool:
    if pre_step is None:
        return False
    owner = getattr(pre_step, "__self__", None)
    return (
        getattr(owner, "_physics_world", None) is replay_world
        and getattr(pre_step, "__name__", "") == "pre_step"
    )


def _replay_state_callbacks(
    setup: SceneSetup,
) -> tuple[Callable[[], Any] | None, Callable[[Any], None] | None]:
    capture = setup.info.get("replay_capture_state")
    restore = setup.info.get("replay_restore_state")
    if capture is None and restore is None:
        return None, None
    if callable(capture) and callable(restore):
        return capture, restore
    setup.info["shared_replay_skipped_reason"] = (
        "replay_capture_state and replay_restore_state must both be callable"
    )
    return None, None


def _timeline_sample_count(frame_count: int) -> int:
    return max(0, min(int(frame_count), 1024))


def _frame_for_timeline_sample(
    sample_index: int, sample_count: int, frame_count: int
) -> int:
    if sample_count <= 1 or frame_count <= 1:
        return 0
    normalized = float(sample_index) / float(sample_count - 1)
    return int(round(normalized * float(frame_count - 1)))


def _frame_mark_series(frame_count: int) -> list[float]:
    sample_count = _timeline_sample_count(frame_count)
    if sample_count <= 0:
        return []
    marker_interval = max(1, int(frame_count) // 12)
    values = [0.0] * sample_count
    for sample in range(sample_count):
        frame = _frame_for_timeline_sample(sample, sample_count, frame_count)
        if frame == 0 or frame + 1 == frame_count or frame % marker_interval == 0:
            values[sample] = 1.0
    return values


def _cursor_series(frame_count: int, frame: int) -> list[float]:
    sample_count = _timeline_sample_count(frame_count)
    if sample_count <= 0:
        return []
    clamped = max(0, min(int(frame), max(0, int(frame_count) - 1)))
    if frame_count <= 1:
        cursor_sample = 0
    else:
        cursor_sample = int(
            round(float(clamped) * float(sample_count - 1) / float(frame_count - 1))
        )
    values = [0.0] * sample_count
    values[max(0, min(cursor_sample, sample_count - 1))] = 1.0
    return values


class _ReplayController:
    """Shared replay recorder/scrubber for py-demos DART 7 worlds."""

    def __init__(
        self,
        world: Any,
        sync: Callable[[], None] | None,
        capture_state: Callable[[], Any] | None = None,
        restore_state: Callable[[Any], None] | None = None,
        *,
        max_frames: int = DEFAULT_REPLAY_MAX_FRAMES,
    ) -> None:
        self._world = world
        self._sync = sync or (lambda: None)
        self._capture_state = capture_state
        self._restore_state = restore_state
        self._scene_states: list[Any] = []
        self._max_frames = max(1, int(max_frames))
        self.save_replay = True
        self.replay_active = False
        self.playing = False
        self.loop = False
        self.rate_index = 0
        self.selected_frame = 0
        self.status = "recording"
        self._set_recording(True)
        self.selected_frame = self._current_frame()
        self._capture_scene_state()
        self._sync()

    @property
    def panel(self) -> ScenePanel:
        return ScenePanel(
            "Replay",
            self.build_panel,
            dock_side="bottom",
            initial_size=(960.0, 260.0),
            horizontal_scrollbar=True,
        )

    @property
    def frame_count(self) -> int:
        try:
            return int(self._world.replay_frame_count)
        except Exception:  # noqa: BLE001
            return 0

    def _set_recording(self, enabled: bool) -> None:
        try:
            self._world.replay_recording_enabled = bool(enabled)
        except Exception:  # noqa: BLE001
            self.save_replay = False

    def _recording_enabled(self) -> bool:
        try:
            return bool(self._world.replay_recording_enabled)
        except Exception:  # noqa: BLE001
            return False

    def _current_frame(self) -> int:
        try:
            cursor = self._world.replay_cursor
        except Exception:  # noqa: BLE001
            cursor = None
        if cursor is None:
            return max(0, min(self.selected_frame, max(0, self.frame_count - 1)))
        return max(0, min(int(cursor), max(0, self.frame_count - 1)))

    def _frame_time(self, frame: int) -> float:
        if self.frame_count <= 0:
            return 0.0
        try:
            return float(
                self._world.get_replay_frame_time(
                    max(0, min(int(frame), self.frame_count - 1))
                )
            )
        except Exception:  # noqa: BLE001
            return float(getattr(self._world, "time", 0.0))

    def _simulation_frame(self, frame: int) -> int:
        if self.frame_count <= 0:
            return 0
        try:
            return int(
                self._world.get_replay_simulation_frame(
                    max(0, min(int(frame), self.frame_count - 1))
                )
            )
        except Exception:  # noqa: BLE001
            return int(getattr(self._world, "frame", 0))

    def _pause_viewer(self, context: Any) -> None:
        if hasattr(context, "set_paused"):
            context.set_paused(True)

    def _resume_viewer(self, context: Any) -> None:
        if hasattr(context, "set_paused"):
            context.set_paused(False)

    def _copy_state(self, state: Any) -> Any:
        try:
            return copy.deepcopy(state)
        except Exception:  # noqa: BLE001
            return state

    def _capture_scene_state(self) -> None:
        if self._capture_state is None:
            return
        count = self.frame_count
        if count <= 0:
            self._scene_states.clear()
            return

        current = self._current_frame()
        if len(self._scene_states) > count:
            del self._scene_states[count:]
        while len(self._scene_states) < count:
            self._scene_states.append(None)
        try:
            self._scene_states[current] = self._copy_state(self._capture_state())
        except Exception:  # noqa: BLE001
            self.save_replay = False
            self.playing = False
            self._set_recording(False)
            self.status = "replay state capture failed"

    def _restore_scene_state(self, frame: int) -> None:
        if self._restore_state is None:
            return
        if frame < 0 or frame >= len(self._scene_states):
            return
        snapshot = self._scene_states[frame]
        if snapshot is None:
            return
        try:
            self._restore_state(self._copy_state(snapshot))
        except Exception:  # noqa: BLE001
            self.playing = False
            self.replay_active = False
            self.status = "replay state restore failed"

    def restore_frame(self, index: int) -> None:
        count = self.frame_count
        if count <= 0:
            self.replay_active = False
            self.playing = False
            self.selected_frame = 0
            return
        clamped = max(0, min(int(index), count - 1))
        self._world.restore_replay_frame(clamped)
        self._restore_scene_state(clamped)
        self.selected_frame = clamped
        self.replay_active = True
        self.status = "replay"
        self._sync()

    def on_live_step_complete(self) -> None:
        if not self.save_replay:
            self.status = "live"
            return
        self.selected_frame = self._current_frame()
        self._capture_scene_state()
        if self.frame_count >= self._max_frames:
            self._set_recording(False)
            self.save_replay = False
            self.status = f"saved {self.frame_count} frames"
        else:
            self.status = "recording"

    def pre_step(self, live_step: Callable[[], None] | None) -> None:
        if self.replay_active:
            self._advance_replay()
            return
        if live_step is not None:
            live_step()
        self.on_live_step_complete()

    def _advance_replay(self) -> None:
        count = self.frame_count
        if count <= 0:
            self.replay_active = False
            self.playing = False
            return
        if not self.playing:
            self.restore_frame(self.selected_frame)
            return
        rate = _REPLAY_RATE_STEPS[
            max(0, min(int(self.rate_index), len(_REPLAY_RATE_STEPS) - 1))
        ]
        next_frame = self.selected_frame + rate
        if next_frame >= count:
            if self.loop:
                next_frame = next_frame % count
            else:
                next_frame = count - 1
                self.playing = False
        self.restore_frame(next_frame)

    def _set_save_replay(self, enabled: bool) -> None:
        self.save_replay = bool(enabled)
        if self.save_replay:
            self.replay_active = False
            self.playing = False
            self._set_recording(True)
            self.selected_frame = self._current_frame()
            self._capture_scene_state()
            self.status = "recording"
        else:
            self._set_recording(False)
            self.status = "live"

    def _clear(self) -> None:
        self.playing = False
        self.replay_active = False
        self._world.clear_replay_recording()
        self.selected_frame = self._current_frame()
        self._capture_scene_state()
        self._sync()

    def _resume_live(self, context: Any) -> None:
        self.playing = False
        self.replay_active = False
        if self.save_replay and not self._recording_enabled():
            self._set_recording(True)
        self.status = "recording" if self.save_replay else "live"
        self._resume_viewer(context)

    def build_panel(self, builder: Any, context: Any) -> None:
        count = self.frame_count
        current = max(0, min(self.selected_frame, max(0, count - 1)))
        if not self.replay_active:
            current = self._current_frame()
            self.selected_frame = current
        status = "replaying" if self.replay_active else self.status
        if self.save_replay and self._recording_enabled() and not self.replay_active:
            status = "recording"
        builder.text(
            f"{status} | frames {count}/{self._max_frames} | "
            f"time {self._frame_time(current):.3f} s | "
            f"sim frame {self._simulation_frame(current)}"
        )

        changed, save = builder.checkbox("Save replay", bool(self.save_replay))
        builder.item_tooltip(
            "Record DART 7 World states while the demo runs; disabling it "
            "keeps existing replay frames but stops storing new states."
        )
        if changed:
            self._set_save_replay(bool(save))
            current = self.selected_frame
            count = self.frame_count

        builder.same_line()
        if builder.button("Clear replay"):
            self._clear()
            count = self.frame_count
            current = self.selected_frame
        builder.item_tooltip("Discard saved frames and restart from the current state.")
        builder.same_line()
        if builder.button("Resume live"):
            self._resume_live(context)
            current = self.selected_frame
        builder.item_tooltip("Leave replay mode and continue simulating from this state.")

        if count <= 0:
            builder.separator()
            builder.text("No replay frames saved yet.")
            return

        builder.separator()
        if builder.button("|<##replay_first"):
            self.playing = False
            self.restore_frame(0)
            self._pause_viewer(context)
        builder.item_tooltip("First replay frame.")
        builder.same_line()
        if builder.button("<##replay_previous"):
            self.playing = False
            self.restore_frame(current - 1)
            self._pause_viewer(context)
        builder.item_tooltip("Previous replay frame.")
        builder.same_line()
        if builder.button("Pause##replay_pause" if self.playing else "Play##replay_play"):
            self.replay_active = True
            self.playing = not self.playing
            if self.playing:
                self._resume_viewer(context)
            else:
                self._pause_viewer(context)
        builder.item_tooltip("Play or pause saved-state replay.")
        builder.same_line()
        if builder.button(">##replay_next"):
            self.playing = False
            self.restore_frame(current + 1)
            self._pause_viewer(context)
        builder.item_tooltip("Next replay frame.")
        builder.same_line()
        if builder.button(">|##replay_last"):
            self.playing = False
            self.restore_frame(count - 1)
            self._pause_viewer(context)
        builder.item_tooltip("Last replay frame.")
        builder.same_line()
        changed, loop = builder.checkbox("Loop replay", bool(self.loop))
        if changed:
            self.loop = bool(loop)
        builder.same_line()
        changed, rate_index = builder.select(
            "Rate", int(self.rate_index), list(_REPLAY_RATE_LABELS)
        )
        if changed:
            self.rate_index = max(
                0, min(int(rate_index), len(_REPLAY_RATE_STEPS) - 1)
            )

        current = max(0, min(self.selected_frame, count - 1))
        selected = float(current)
        changed, selected = builder.timeline(
            "Saved states##py_demo_replay_timeline",
            selected,
            0.0,
            float(max(0, count - 1)),
            value_track=[],
            marker_track=_frame_mark_series(count),
            cursor_track=_cursor_series(count, current),
            value_track_label="Saved states",
        )
        if changed:
            self.playing = False
            self.restore_frame(int(round(selected)))
            self._pause_viewer(context)

        builder.text(f"selected {self.selected_frame + 1}/{count}")


def _attach_replay_controls(scene: PythonDemoScene, setup: SceneSetup) -> SceneSetup:
    if setup.info.get("disable_shared_replay"):
        return setup

    replay_world = _replay_world_from_setup(setup)
    if replay_world is None:
        return setup

    capture_state, restore_state = _replay_state_callbacks(setup)
    if (
        setup.pre_step is not None
        and not _is_bridge_pre_step(setup.pre_step, replay_world)
        and capture_state is None
        and not setup.info.get("replay_live_step_is_stateless")
    ):
        setup.info["shared_replay_skipped_reason"] = (
            "custom pre_step needs replay_capture_state/replay_restore_state "
            "or replay_live_step_is_stateless"
        )
        return setup

    sync = setup.info.get("replay_sync")
    if sync is None:
        sync = _sync_callback_from_pre_step(setup.pre_step, replay_world)
    controller = _ReplayController(
        replay_world,
        sync,
        capture_state=capture_state,
        restore_state=restore_state,
    )
    live_pre_step = setup.pre_step

    def replay_pre_step() -> None:
        controller.pre_step(live_pre_step)

    setup.pre_step = replay_pre_step
    setup.panels = [*setup.panels, controller.panel]
    setup.info["replay_world"] = replay_world
    setup.info["replay_controller"] = controller
    setup.info["replay_panel_title"] = controller.panel.title
    setup.info["replay_scene_id"] = scene.id
    return setup


def _step(setup: SceneSetup, frames: int) -> None:
    """Advance the scene by ``frames`` steps headlessly.

    Honors (in order of precedence):
      1. SceneSetup.step (whole-loop callable, owns world.step() itself).
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
        if setup.world is not None and hasattr(setup.world, "step"):
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


def _make_world_factory(
    scene: PythonDemoScene, gpu_panel: ScenePanel | None = None
) -> Callable[[], Any]:
    """Wrap scene.build() so dart.gui.run_demos can call it as a factory.

    Returns a tuple consumed by the viewer binding:
      ``(pre_step, force_drag, panels, renderable_provider, debug_provider)``.
    """

    def factory() -> Any:
        with _bounded_scene_build(scene.id):
            setup = scene.build()
        setup = _attach_replay_controls(scene, setup)
        pre_step = setup.pre_step
        if pre_step is not None:
            original_pre_step = pre_step

            def bounded_pre_step() -> None:
                with _bounded_scene_callback(scene.id, "pre_step"):
                    original_pre_step()

            pre_step = bounded_pre_step
        panels = list(setup.panels)
        if gpu_panel is not None:
            # A runner-injected, scene-independent GPU compute toggle (only
            # present when CUDA is available); see _make_gpu_panel.
            panels.append(gpu_panel)
        renderable_provider = setup.renderable_provider
        if renderable_provider is None and setup.world is not None:
            renderable_provider = getattr(setup.world, "renderable_provider", None)
        return (
            pre_step,
            setup.force_drag,
            panels if panels else None,
            renderable_provider,
            setup.debug_provider,
        )

    return factory


def _gpu_preference(cli_pref: bool | None) -> str:
    """Resolve the GPU-compute preference: ``on`` / ``off`` / ``auto``.

    Precedence: an explicit ``--gpu`` / ``--no-gpu`` flag, then the
    ``DART_PY_DEMOS_GPU`` environment variable, else ``auto`` (enable when a
    CUDA device is available).
    """

    if cli_pref is not None:
        return "on" if cli_pref else "off"
    env = os.environ.get("DART_PY_DEMOS_GPU", "auto").strip().lower()
    if env in {"1", "on", "true", "yes", "gpu", "cuda"}:
        return "on"
    if env in {"0", "off", "false", "no", "cpu"}:
        return "off"
    return "auto"


def _strip_gpu_flags(argv: list[str]) -> list[str]:
    """Drop ``--gpu`` / ``--no-gpu`` before forwarding argv to the C++ viewer.

    The viewer's argument parser does not know these runner-local flags, so they
    must not reach it.
    """

    return [arg for arg in argv if arg not in {"--gpu", "--no-gpu"}]


def _default_initial_scene_args(
    argv: list[str], scene_arg: str | None, environ: Mapping[str, str]
) -> list[str]:
    """Forward a showcase default without changing the catalog order."""

    if scene_arg is not None:
        return []
    if environ.get("DART_DEMOS_SCENE"):
        return []
    if "--cycle-scenes" in argv:
        return []
    return ["--scene", DEFAULT_INITIAL_SCENE_ID]


def _make_gpu_panel(sx: Any) -> ScenePanel:
    """A small in-viewer panel that toggles GPU (CUDA) deformable solve.

    The PSD-projection backend is process-wide, so this single toggle affects
    every deformable scene. Only injected when CUDA is available.
    """

    def build(builder: Any, _context: Any) -> None:
        builder.text("GPU compute (CUDA)")
        builder.separator()
        enabled = bool(sx.is_accelerated_deformable_solve_enabled())
        changed, new_enabled = builder.checkbox("GPU deformable solve", enabled)
        if changed:
            sx.set_accelerated_deformable_solve(bool(new_enabled))
        builder.text("Offloads the deformable projected-Newton")
        builder.text("PSD projection to the GPU (CUDA).")
        builder.text("Process-wide: affects every deformable scene.")

    return ScenePanel("GPU", build, dock_side="right", initial_size=(300.0, 150.0))


def _configure_gpu_compute(dart: Any, cli_pref: bool | None) -> ScenePanel | None:
    """Apply the resolved GPU preference and return an in-app toggle panel.

    Returns a ``ScenePanel`` to inject into every scene when CUDA is available,
    otherwise ``None`` (the default/CPU build is left untouched). Prints a
    one-line status so headless and interactive runs both report the mode.
    """

    # In DART 7 the ECS simulation API is flat on the dartpy module, so the GPU
    # deformable-solve controls live directly on it when the World stack is built.
    sx = dart
    if not hasattr(sx, "set_accelerated_deformable_solve"):
        return None

    available = bool(sx.is_accelerated_deformable_solve_available())
    preference = _gpu_preference(cli_pref)
    if preference == "off":
        enabled = bool(sx.set_accelerated_deformable_solve(False))
    elif preference == "on":
        enabled = bool(sx.set_accelerated_deformable_solve(True))
        if not enabled:
            print(
                "py-demos: GPU requested but CUDA is unavailable; using CPU.",
                file=sys.stderr,
            )
    else:  # auto
        enabled = bool(sx.set_accelerated_deformable_solve(available))

    if available:
        print(
            f"py-demos: GPU deformable solve (CUDA) "
            f"{'ON' if enabled else 'off'} [{preference}]; "
            f"toggle in the GPU panel or with --gpu/--no-gpu."
        )
        return _make_gpu_panel(sx)

    print(
        "py-demos: CUDA not available in this build; " "deformable solve runs on CPU."
    )
    return None


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
    # GPU (CUDA) deformable-solve toggle. Default (unset) is "auto": enable when
    # a CUDA device is available (e.g. under `pixi run -e cuda py-demos`),
    # otherwise run on CPU. DART_PY_DEMOS_GPU overrides the default.
    parser.add_argument("--gpu", dest="gpu", action="store_true", default=None)
    parser.add_argument("--no-gpu", dest="gpu", action="store_false")
    known, _passthrough = parser.parse_known_args(argv)

    if known.list:
        _print_catalog(scenes)
        return 0

    _validate_scene(known.scene, scenes)
    default_initial_scene_args = _default_initial_scene_args(
        argv, known.scene, os.environ
    )
    if default_initial_scene_args:
        _validate_scene(DEFAULT_INITIAL_SCENE_ID, scenes)

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

    # Resolve the GPU-compute preference before scenes build/step. Returns an
    # in-viewer toggle panel when CUDA is available, else None (CPU build).
    gpu_panel = _configure_gpu_compute(dart, known.gpu)

    # Build the catalog for the viewer.
    catalog = [
        (
            scene.id,
            scene.title,
            scene.category,
            scene.summary,
            _make_world_factory(scene, gpu_panel),
        )
        for scene in scenes
    ]

    # The viewer's argument parser does not know the runner-local GPU flags.
    full_argv = [
        "py-demos",
        *default_initial_scene_args,
        *_strip_gpu_flags(argv),
    ]
    return int(dart.gui.run_demos(catalog, full_argv))
