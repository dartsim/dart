"""Full-catalog no-crash smoke for the DART Python demos.

This is the M0 "battle test" for ``pixi run py-demos``: it builds *every*
registered scene, steps it headlessly for a few frames, and exercises its
renderable/debug providers, reporting per-scene pass / fail / timeout / crash.
The GUI cycle smoke in ``python/tests/integration/test_demos_cycle.py`` only
covers the first three scenes; this covers the whole catalog.

No display, window, or GUI backend is required: scenes are built and stepped
through the same headless contract the runner uses (``SceneSetup`` /
``_step``), without invoking ``dartpy.gui.run_demos``.

Each scene runs in its **own subprocess** so a hard crash (C++ segfault) or an
infinite loop in one scene cannot hide the rest of the catalog.

Usage::

    # whole catalog (orchestrator mode)
    PYTHONPATH=build/cuda/cpp/Release-docking/python:python \
        .pixi/envs/cuda/bin/python scripts/py_demos_smoke.py

    # one scene, in-process (worker mode; used internally, handy for triage)
    ... scripts/py_demos_smoke.py --scene rigid_body --frames 5

Exit code is non-zero if any scene fails.
"""

from __future__ import annotations

import argparse
import ctypes
import ctypes.util
import importlib.util
import json
import os
import pathlib
import struct
import subprocess
import sys
import tempfile
import time
import zlib
from dataclasses import asdict, dataclass


# --------------------------------------------------------------------------- #
# Worker mode: build + step a single scene in-process. Raises on any failure.
# --------------------------------------------------------------------------- #
def _smoke_one_scene(scene_id: str, frames: int, gpu_pref: bool | None) -> int:
    """Build, step, and exercise providers for one scene. Returns steps taken."""

    import dartpy as dart  # noqa: F401  (import side effects; configures backend)
    from examples.demos._smoke_support import exercise_panels
    from examples.demos.registry import make_demo_scenes
    from examples.demos.runner import _configure_gpu_compute, _step

    # Match the real runner: resolve the process-wide GPU compute preference
    # (auto-enables under CUDA) before any scene builds or steps.
    _configure_gpu_compute(dart, gpu_pref)

    scenes = {scene.id: scene for scene in make_demo_scenes()}
    scene = scenes.get(scene_id)
    if scene is None:
        raise SystemExit(f"unknown scene id: {scene_id!r}")

    setup = scene.build()
    _step(setup, frames)

    # Exercise the interactive-render providers (these run every UI frame in the
    # viewer, so a crash here is a real, user-visible defect).
    provider = setup.renderable_provider
    if provider is None and setup.world is not None:
        provider = getattr(setup.world, "renderable_provider", None)
    if callable(provider):
        provider()
    if callable(setup.debug_provider):
        setup.debug_provider()

    # Exercise scene-owned panels (a live-viewer crash source) with a faithful
    # fake builder/context, without opening the real viewer.
    exercise_panels(setup)

    return frames


# --------------------------------------------------------------------------- #
# Orchestrator mode: enumerate scenes, run each in an isolated subprocess.
# --------------------------------------------------------------------------- #
@dataclass
class SceneResult:
    scene_id: str
    status: str  # ok | fail | timeout | crash
    seconds: float
    detail: str = ""


def _list_scene_ids() -> list[str]:
    from examples.demos.registry import make_demo_scenes

    return [scene.id for scene in make_demo_scenes()]


def _select_scene_ids(scene_ids: list[str], only: list[str] | None) -> list[str]:
    """Return the catalog subset to smoke, preserving registry order."""

    if not scene_ids:
        raise ValueError("demo registry is empty")
    if not only:
        return scene_ids

    known = set(scene_ids)
    requested = set(only)
    missing = sorted(requested - known)
    if missing:
        formatted = ", ".join(missing)
        raise ValueError(f"unknown scene id(s) for --only: {formatted}")

    selected = [scene_id for scene_id in scene_ids if scene_id in requested]
    if not selected:
        raise ValueError("--only selected no scenes")
    return selected


def _run_worker(
    scene_id: str, frames: int, gpu_pref: bool | None, timeout: float
) -> SceneResult:
    cmd = [
        sys.executable,
        os.path.abspath(__file__),
        "--scene",
        scene_id,
        "--frames",
        str(frames),
    ]
    if gpu_pref is True:
        cmd.append("--gpu")
    elif gpu_pref is False:
        cmd.append("--no-gpu")

    start = time.monotonic()
    try:
        proc = subprocess.run(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=timeout,
            text=True,
        )
    except subprocess.TimeoutExpired:
        return SceneResult(
            scene_id, "timeout", time.monotonic() - start, f"exceeded {timeout:.0f}s"
        )
    elapsed = time.monotonic() - start

    if proc.returncode == 0:
        return SceneResult(scene_id, "ok", elapsed)

    # Negative return code => killed by signal (segfault/abort) => crash.
    status = "crash" if proc.returncode < 0 else "fail"
    tail = "\n".join((proc.stdout or "").strip().splitlines()[-25:])
    detail = f"exit={proc.returncode}\n{tail}"
    return SceneResult(scene_id, status, elapsed, detail)


def _apply_stable_linux_render_env() -> None:
    if not sys.platform.startswith("linux"):
        return
    os.environ.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")
    os.environ.setdefault("MESA_LOADER_DRIVER_OVERRIDE", "llvmpipe")
    egl_vendor = pathlib.Path("/usr/share/glvnd/egl_vendor.d/50_mesa.json")
    if egl_vendor.is_file():
        os.environ.setdefault("__EGL_VENDOR_LIBRARY_FILENAMES", str(egl_vendor))


def _can_open_linux_display() -> bool:
    if not sys.platform.startswith("linux"):
        return True
    display = os.environ.get("DISPLAY")
    if not display:
        return False
    lib_name = ctypes.util.find_library("X11")
    if not lib_name:
        return False
    try:
        lib_x11 = ctypes.CDLL(lib_name)
        lib_x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
        lib_x11.XOpenDisplay.restype = ctypes.c_void_p
        lib_x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
        handle = lib_x11.XOpenDisplay(display.encode())
        if not handle:
            return False
        lib_x11.XCloseDisplay(handle)
        return True
    except Exception:  # noqa: BLE001
        return False


def _ppm_is_nonblank(path: os.PathLike[str] | str) -> bool:
    data = pathlib.Path(path).read_bytes()
    tokens: list[bytes] = []
    index = 0

    while len(tokens) < 4:
        while index < len(data):
            byte = data[index]
            if byte == ord("#"):
                newline = data.find(b"\n", index)
                if newline < 0:
                    return False
                index = newline + 1
                continue
            if byte not in b" \t\r\n":
                break
            index += 1
        if index >= len(data):
            return False

        start = index
        while index < len(data) and data[index] not in b" \t\r\n":
            index += 1
        tokens.append(data[start:index])

    if tokens[0] != b"P6" or tokens[3] != b"255":
        return False
    try:
        width = int(tokens[1])
        height = int(tokens[2])
    except ValueError:
        return False

    if data[index : index + 2] == b"\r\n":
        pixel_start = index + 2
    elif index < len(data) and data[index] in b" \t\r\n":
        pixel_start = index + 1
    else:
        pixel_start = index

    if width <= 0 or height <= 0:
        return False

    expected = width * height * 3
    pixels = data[pixel_start : pixel_start + expected]
    if len(pixels) < expected:
        return False

    first = pixels[0]
    return any(byte != first for byte in pixels[1:])


def _load_headless_analyzer():
    path = (
        pathlib.Path(__file__).resolve().parents[1]
        / "dart"
        / "gui"
        / "detail"
        / "testing"
        / "analyze_headless_smoke.py"
    )
    spec = importlib.util.spec_from_file_location("dart_headless_smoke_analyzer", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"failed to load {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _write_rgba_ppm(path: pathlib.Path, image: object) -> None:
    width = int(getattr(image, "width"))
    height = int(getattr(image, "height"))
    channels = int(getattr(image, "channels"))
    if channels != 4:
        raise RuntimeError(f"expected RGBA image, got {channels} channels")
    pixels = memoryview(image)
    rgba = bytes(pixels)
    rgb = bytearray(width * height * 3)
    for src in range(0, len(rgba), 4):
        dst = src // 4 * 3
        rgb[dst : dst + 3] = rgba[src : src + 3]
    path.write_bytes(f"P6\n{width} {height}\n255\n".encode() + bytes(rgb))


def _require_png_decode(data: bytes, expected_width: int, expected_height: int) -> None:
    if not data.startswith(b"\x89PNG\r\n\x1a\n"):
        raise RuntimeError("png_bytes() did not return a PNG signature")
    offset = 8
    width = height = None
    idat = bytearray()
    while offset < len(data):
        if offset + 12 > len(data):
            raise RuntimeError("png_bytes() ended inside a PNG chunk")
        length = struct.unpack(">I", data[offset : offset + 4])[0]
        chunk = data[offset + 4 : offset + 8]
        payload_start = offset + 8
        payload_end = payload_start + length
        if payload_end + 4 > len(data):
            raise RuntimeError("png_bytes() contains a truncated PNG chunk")
        payload = data[payload_start:payload_end]
        offset = payload_end + 4
        if chunk == b"IHDR":
            width, height = struct.unpack(">II", payload[:8])
        elif chunk == b"IDAT":
            idat.extend(payload)
        elif chunk == b"IEND":
            break
    if (width, height) != (expected_width, expected_height):
        raise RuntimeError(
            f"png_bytes() dimensions {(width, height)} != "
            f"{(expected_width, expected_height)}"
        )
    raw = zlib.decompress(bytes(idat))
    expected = expected_height * (1 + expected_width * 4)
    if len(raw) != expected:
        raise RuntimeError(
            f"png_bytes() decompressed to {len(raw)} bytes, expected {expected}"
        )


def _run_offscreen_render_api_smoke(width: int, height: int) -> SceneResult:
    start = time.monotonic()
    if not _can_open_linux_display():
        return SceneResult(
            "dart_gui_render_api",
            "fail",
            time.monotonic() - start,
            "Filament OpenGL headless rendering requires a usable DISPLAY/Xvfb",
        )
    try:
        import dartpy as dart

        world = dart.World()
        ground = world.add_rigid_body(
            "offscreen_smoke_ground", position=(0.0, 0.0, -0.05)
        )
        ground.is_static = True
        ground.set_collision_shape(dart.CollisionShape.box((1.4, 1.4, 0.05)))
        box = world.add_rigid_body("offscreen_smoke_box", position=(0.0, 0.0, 0.35))
        box.set_collision_shape(dart.CollisionShape.box((0.22, 0.22, 0.22)))

        image = dart.gui.render(world, size=(width, height))
        with tempfile.NamedTemporaryFile(
            prefix="dart_gui_render_api_", suffix=".ppm", delete=False
        ) as tmp:
            ppm_path = pathlib.Path(tmp.name)
        try:
            _write_rgba_ppm(ppm_path, image)
            # PLAN-012's gate is a non-blank Filament-backed image. The strict
            # shadow/lighting contrast heuristic is calibrated for full-fidelity
            # demo scenes; a minimal one-box offscreen render under the default
            # lightweight passes is legitimately flat, so assert non-blank here.
            _load_headless_analyzer().analyze_basic(ppm_path, width, height)
            _require_png_decode(image.png_bytes(), width, height)
        finally:
            try:
                ppm_path.unlink()
            except FileNotFoundError:
                # Another cleanup path may already have removed the temporary file.
                pass
    except Exception as exc:  # noqa: BLE001
        return SceneResult(
            "dart_gui_render_api", "fail", time.monotonic() - start, str(exc)
        )
    return SceneResult("dart_gui_render_api", "ok", time.monotonic() - start)


def _run_render_worker(
    scene_id: str,
    frames: int,
    gpu_pref: bool | None,
    timeout: float,
    width: int,
    height: int,
) -> SceneResult:
    with tempfile.NamedTemporaryFile(
        prefix=f"dart_py_demo_{scene_id}_", suffix=".ppm", delete=False
    ) as tmp:
        screenshot_path = tmp.name

    cmd = [
        sys.executable,
        "-m",
        "examples.demos",
        "--scene",
        scene_id,
        "--headless",
        "--screenshot",
        screenshot_path,
        "--frames",
        str(frames),
        "--width",
        str(width),
        "--height",
        str(height),
    ]
    if gpu_pref is True:
        cmd.append("--gpu")
    elif gpu_pref is False:
        cmd.append("--no-gpu")

    start = time.monotonic()
    try:
        proc = subprocess.run(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=timeout,
            text=True,
        )
    except subprocess.TimeoutExpired:
        try:
            os.unlink(screenshot_path)
        except FileNotFoundError:
            # The worker may exit before writing the screenshot; cleanup is best-effort.
            pass
        return SceneResult(
            scene_id, "timeout", time.monotonic() - start, f"exceeded {timeout:.0f}s"
        )
    elapsed = time.monotonic() - start

    try:
        if proc.returncode == 0:
            if not os.path.isfile(screenshot_path):
                return SceneResult(
                    scene_id, "fail", elapsed, "render completed without a screenshot"
                )
            if not _ppm_is_nonblank(screenshot_path):
                return SceneResult(
                    scene_id,
                    "fail",
                    elapsed,
                    "render screenshot is missing, invalid, or blank",
                )
            return SceneResult(scene_id, "ok", elapsed)

        status = "crash" if proc.returncode < 0 else "fail"
        tail = "\n".join((proc.stdout or "").strip().splitlines()[-25:])
        detail = f"exit={proc.returncode}\n{tail}"
        return SceneResult(scene_id, status, elapsed, detail)
    finally:
        try:
            os.unlink(screenshot_path)
        except FileNotFoundError:
            # The screenshot may already be gone after a timeout or failed render.
            pass


def _orchestrate(args: argparse.Namespace) -> int:
    try:
        scene_ids = _select_scene_ids(_list_scene_ids(), args.only)
    except ValueError as exc:
        print(f"py-demos smoke: {exc}", file=sys.stderr)
        return 2

    if args.render:
        _apply_stable_linux_render_env()
        print(
            f"py-demos smoke: {len(scene_ids)} scenes, {args.frames} frames each, "
            f"timeout {args.timeout:.0f}s, mode=render, "
            f"resolution={args.width}x{args.height}, gpu={args.gpu_pref_label}"
        )
    else:
        print(
            f"py-demos smoke: {len(scene_ids)} scenes, {args.frames} frames each, "
            f"timeout {args.timeout:.0f}s, gpu={args.gpu_pref_label}"
        )
    print("-" * 72)

    results: list[SceneResult] = []
    total = len(scene_ids) + (1 if args.render else 0)
    if args.render:
        res = _run_offscreen_render_api_smoke(args.width, args.height)
        results.append(res)
        mark = {"ok": "ok  ", "fail": "FAIL", "timeout": "TIME", "crash": "CRSH"}[
            res.status
        ]
        print(f"[{1:3d}/{total}] {mark} {res.scene_id:<44s} {res.seconds:6.2f}s")
        if res.status != "ok":
            for line in res.detail.splitlines():
                print(f"           | {line}")

    start_index = 2 if args.render else 1
    for i, scene_id in enumerate(scene_ids, start_index):
        if args.render:
            res = _run_render_worker(
                scene_id, args.frames, args.gpu, args.timeout, args.width, args.height
            )
        else:
            res = _run_worker(scene_id, args.frames, args.gpu, args.timeout)
        results.append(res)
        mark = {"ok": "ok  ", "fail": "FAIL", "timeout": "TIME", "crash": "CRSH"}[
            res.status
        ]
        print(f"[{i:3d}/{total}] {mark} {scene_id:<44s} {res.seconds:6.2f}s")
        if res.status != "ok":
            for line in res.detail.splitlines():
                print(f"           | {line}")

    failures = [r for r in results if r.status != "ok"]
    print("-" * 72)
    print(
        f"PASS {len(results) - len(failures)}/{len(results)}   " f"FAIL {len(failures)}"
    )
    if failures:
        print("Failing scenes:")
        for r in failures:
            print(f"  {r.status:<8s} {r.scene_id}")

    if args.json_out:
        with open(args.json_out, "w", encoding="utf-8") as fh:
            json.dump([asdict(r) for r in results], fh, indent=2)
        print(f"wrote {args.json_out}")

    return 1 if failures else 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scene", default=None, help="worker mode: smoke one scene in-process"
    )
    parser.add_argument(
        "--frames",
        type=int,
        default=None,
        help="steps/frames per scene (default 3, render default 2)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=None,
        help="per-scene subprocess timeout seconds (default 60, render default 120)",
    )
    parser.add_argument(
        "--only",
        nargs="*",
        default=None,
        help="restrict the catalog to these scene ids",
    )
    parser.add_argument(
        "--json-out", default="", help="write per-scene results as JSON to this path"
    )
    parser.add_argument("--gpu", dest="gpu", action="store_true", default=None)
    parser.add_argument("--no-gpu", dest="gpu", action="store_false")
    parser.add_argument("--render", action="store_true", help="use the real viewer")
    parser.add_argument("--width", type=int, default=480, help="render width")
    parser.add_argument("--height", type=int, default=320, help="render height")
    args = parser.parse_args(argv)
    if args.frames is None:
        args.frames = 2 if args.render else 3
    if args.timeout is None:
        args.timeout = 120.0 if args.render else 60.0
    args.gpu_pref_label = {True: "on", False: "off", None: "auto"}[args.gpu]

    if args.scene is not None:
        steps = _smoke_one_scene(args.scene, args.frames, args.gpu)
        print(f"SMOKE_OK {args.scene} steps={steps}")
        return 0

    return _orchestrate(args)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
