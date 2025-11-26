from __future__ import annotations

import argparse
import logging
import os
import time
from datetime import datetime
from pathlib import Path

from imgui_bundle import imgui, immapp

from demo_hub.core import Recorder, build_default_registry
from demo_hub.gui.primitives import Segment2D
from demo_hub.gui.viewport import draw_topdown

logger = logging.getLogger(__name__)


class GuiState:
    def __init__(self, registry, scene_id: str, dt: float) -> None:
        self.registry = registry
        self.scene_ids = registry.scene_ids
        self.dt = dt
        self.selected_idx = self.scene_ids.index(scene_id)
        self.recorder = Recorder()
        self.recording = False
        self.record_path: Path | None = None
        self.last_time = time.perf_counter()
        self.accumulator = 0.0
        self.step_count = 0
        self.paused = False
        self.step_once = False
        self.scene = registry.create(scene_id)
        self.scene.setup(self.dt)

    def switch_scene(self, idx: int) -> None:
        self.selected_idx = idx
        scene_id = self.scene_ids[idx]
        self.scene = self.registry.create(scene_id)
        self.scene.setup(self.dt)
        self.accumulator = 0.0
        self.step_count = 0
        self.paused = False
        self.step_once = False
        self.last_time = time.perf_counter()
        if self.recording:
            self.recorder.stop()
        self.recording = False
        self.record_path = None


def _simulate(state: GuiState) -> None:
    now = time.perf_counter()
    frame_time = now - state.last_time
    state.last_time = now

    if not state.paused:
        state.accumulator += frame_time
    if state.step_once:
        state.accumulator += state.dt
        state.step_once = False

    while state.accumulator >= state.dt:
        state.scene.update(state.dt)
        state.step_count += 1
        if state.recording:
            state.recorder.log(state.step_count, state.scene.export_state() or {})
        state.accumulator -= state.dt


def _render(state: GuiState) -> None:  # pragma: no cover - manual UI
    io = imgui.get_io()
    io.config_flags |= imgui.ConfigFlags_.docking_enable
    imgui.dock_space_over_viewport()

    _simulate(state)

    imgui.begin("Demo Hub")

    changed, selected_idx = imgui.combo("Scene", state.selected_idx, state.scene_ids)
    if changed:
        state.switch_scene(selected_idx)

    imgui.text(f"dt: {state.dt:.4f}s")
    if imgui.button("Play" if state.paused else "Pause"):
        state.paused = not state.paused
    imgui.same_line()
    if imgui.button("Step"):
        state.step_once = True
        state.paused = True
    imgui.same_line()
    if imgui.button("Reset"):
        state.scene.reset()
        state.accumulator = 0.0
        state.step_count = 0

    if imgui.button("Start/Stop Recording"):
        if state.recording:
            state.recorder.stop()
            state.recording = False
        else:
            state.record_path = Path(
                f"demo_recording_{state.scene.metadata.scene_id}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jsonl"
            )
            state.recorder.start(
                state.record_path,
                {"scene": state.scene.metadata.scene_id, "dt": state.dt},
            )
            state.recording = True
    if state.recording and state.record_path:
        imgui.same_line()
        imgui.text(f"REC -> {state.record_path.name}")

    state_payload = state.scene.export_state() or {}
    if state_payload:
        imgui.separator()
        imgui.text("State")
        for key, value in state_payload.items():
            imgui.text(f"{key}: {value}")

    imgui.end()

    segments = [Segment2D(seg[0], seg[1], seg[2]) for seg in state.scene.debug_draw_2d()]
    draw_topdown("Viewport (XZ top-down)", segments)


def main(argv: list[str] | None = None) -> None:
    if os.name != "nt" and not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
        raise SystemExit("GUI requires a display (set DISPLAY/WAYLAND_DISPLAY or run inside a desktop session).")

    registry = build_default_registry()

    parser = argparse.ArgumentParser(description="ImGui shell for demo_hub scenes")
    parser.add_argument("--scene", default="hello_world", choices=registry.scene_ids, help="Scene id to start with")
    parser.add_argument("--dt", type=float, default=1.0 / 240.0, help="Simulation timestep")
    args = parser.parse_args(argv)

    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    logger.info("Launching GUI shell (scene=%s)", args.scene)
    state = GuiState(registry, args.scene, args.dt)

    params = immapp.RunnerParams()
    params.app_window_params.window_title = "DART Demo Hub (Python)"
    params.app_window_params.window_geometry.size = [1280, 720]
    params.callbacks.show_gui = lambda: _render(state)
    immapp.run(runner_params=params)


if __name__ == "__main__":  # pragma: no cover - manual UI
    main()
