from __future__ import annotations

import argparse
import logging
import time
from datetime import datetime
from pathlib import Path
from typing import Iterable

from demo_hub.core import Recorder, build_default_registry
from demo_hub.gui.primitives import Segment2D
from demo_hub.gui.viewport import draw_topdown

logger = logging.getLogger(__name__)


def has_gui_deps() -> bool:
    try:
        import imgui  # noqa: F401
        import glfw  # noqa: F401
        from OpenGL import GL  # noqa: F401
    except ImportError:  # pragma: no cover - exercised manually
        return False
    return True


def _init_window(width: int, height: int):
    import glfw

    if not glfw.init():
        raise RuntimeError("Failed to initialize GLFW")
    glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)
    glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 3)
    glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)
    window = glfw.create_window(width, height, "DART Demo Hub (Python)", None, None)
    if not window:
        glfw.terminate()
        raise RuntimeError("Failed to create GLFW window")
    glfw.make_context_current(window)
    glfw.swap_interval(1)
    return window


def _render_loop(window, registry, scene_id: str, dt: float) -> None:  # pragma: no cover - manual UI
    import imgui
    import glfw
    from imgui.integrations.glfw import GlfwRenderer
    from OpenGL import GL

    imgui.create_context()
    impl = GlfwRenderer(window, attach_callbacks=False)

    scene = registry.create(scene_id)
    scene.setup(dt)
    paused = False
    step_once = False
    accumulator = 0.0
    last_time = time.perf_counter()
    step_count = 0
    scene_ids = registry.scene_ids
    selected_idx = scene_ids.index(scene_id)

    recorder = Recorder()
    recording = False
    record_path: Path | None = None

    while not glfw.window_should_close(window):
        glfw.poll_events()
        impl.process_inputs()

        now = time.perf_counter()
        frame_time = now - last_time
        last_time = now

        if not paused:
            accumulator += frame_time
        if step_once:
            accumulator += dt
            step_once = False

        while accumulator >= dt:
            scene.update(dt)
            step_count += 1
            if recording:
                recorder.log(step_count, scene.export_state() or {})
            accumulator -= dt

        imgui.new_frame()
        imgui.begin("Demo Hub")

        changed, selected_idx = imgui.combo("Scene", selected_idx, scene_ids)
        if changed:
            selected_id = scene_ids[selected_idx]
            scene = registry.create(selected_id)
            scene.setup(dt)
            paused = False
            accumulator = 0.0
            step_count = 0

        imgui.text(f"dt: {dt:.4f}s")
        if imgui.button("Play/Pause"):
            paused = not paused
        imgui.same_line()
        if imgui.button("Step"):
            step_once = True
            paused = True
        imgui.same_line()
        if imgui.button("Reset"):
            scene.reset()
            accumulator = 0.0
            step_count = 0

        if imgui.button("Start/Stop Recording"):
            if recording:
                recorder.stop()
                recording = False
            else:
                record_path = Path(
                    f"demo_recording_{scene.metadata.scene_id}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jsonl"
                )
                recorder.start(
                    record_path,
                    {"scene": scene.metadata.scene_id, "dt": dt},
                )
                recording = True
        if recording and record_path:
            imgui.same_line()
            imgui.text(f"REC -> {record_path.name}")

        state = scene.export_state() or {}
        if state:
            imgui.separator()
            imgui.text("State")
            for key, value in state.items():
                imgui.text(f"{key}: {value}")

        imgui.end()

        # Simple top-down debug view
        segments = [Segment2D(seg[0], seg[1], seg[2]) for seg in scene.debug_draw_2d()]
        draw_topdown("Viewport (XZ top-down)", segments)

        GL.glViewport(0, 0, int(glfw.get_framebuffer_size(window)[0]), int(glfw.get_framebuffer_size(window)[1]))
        GL.glClearColor(0.1, 0.1, 0.1, 1)
        GL.glClear(GL.GL_COLOR_BUFFER_BIT)
        imgui.render()
        impl.render(imgui.get_draw_data())
        glfw.swap_buffers(window)

    if recording:
        recorder.stop()
    impl.shutdown()
    glfw.terminate()


def main(argv: list[str] | None = None) -> None:
    if not has_gui_deps():
        raise SystemExit(
            "Missing GUI dependencies. Ensure imgui[glfw], glfw, and PyOpenGL are installed in the pixi environment."
        )
    registry = build_default_registry()

    parser = argparse.ArgumentParser(description="Minimal ImGui shell for demo_hub scenes")
    parser.add_argument("--scene", default="hello_world", choices=registry.scene_ids, help="Scene id to start with")
    parser.add_argument("--dt", type=float, default=1.0 / 240.0, help="Simulation timestep")
    args = parser.parse_args(argv)

    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    window = _init_window(1280, 720)
    logger.info("Launching GUI shell (scene=%s)", args.scene)
    _render_loop(window, registry, args.scene, args.dt)


if __name__ == "__main__":  # pragma: no cover - manual UI
    main()
