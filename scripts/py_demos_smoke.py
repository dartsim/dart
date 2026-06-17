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
import json
import os
import subprocess
import sys
import time
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


def _orchestrate(args: argparse.Namespace) -> int:
    try:
        scene_ids = _select_scene_ids(_list_scene_ids(), args.only)
    except ValueError as exc:
        print(f"py-demos smoke: {exc}", file=sys.stderr)
        return 2

    print(
        f"py-demos smoke: {len(scene_ids)} scenes, {args.frames} frames each, "
        f"timeout {args.timeout:.0f}s, gpu={args.gpu_pref_label}"
    )
    print("-" * 72)

    results: list[SceneResult] = []
    for i, scene_id in enumerate(scene_ids, 1):
        res = _run_worker(scene_id, args.frames, args.gpu, args.timeout)
        results.append(res)
        mark = {"ok": "ok  ", "fail": "FAIL", "timeout": "TIME", "crash": "CRSH"}[
            res.status
        ]
        print(f"[{i:3d}/{len(scene_ids)}] {mark} {scene_id:<44s} {res.seconds:6.2f}s")
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
        "--frames", type=int, default=3, help="steps per scene (default 3)"
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=60.0,
        help="per-scene subprocess timeout seconds (default 60)",
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
    args = parser.parse_args(argv)
    args.gpu_pref_label = {True: "on", False: "off", None: "auto"}[args.gpu]

    if args.scene is not None:
        steps = _smoke_one_scene(args.scene, args.frames, args.gpu)
        print(f"SMOKE_OK {args.scene} steps={steps}")
        return 0

    return _orchestrate(args)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
