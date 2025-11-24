from __future__ import annotations

import argparse
import logging
from typing import Iterable

from demo_hub.core import Scene, build_default_registry

logger = logging.getLogger(__name__)


def _list_scenes(scenes: Iterable) -> None:
    for meta in scenes:
        tags = f" [{' '.join(meta.tags)}]" if meta.tags else ""
        print(f"{meta.scene_id}: {meta.label}{tags}\n  {meta.summary}\n")


def _run_scene(scene: Scene, steps: int, dt: float, log_interval: int) -> None:
    scene.setup(dt)
    for step in range(steps):
        scene.update(dt)
        if log_interval > 0 and (step + 1) % log_interval == 0:
            state = scene.export_state()
            if state:
                logger.info("Step %d: %s", step + 1, state)


def main(argv: list[str] | None = None) -> None:
    registry = build_default_registry()

    parser = argparse.ArgumentParser(description="Python demo hub (headless skeleton)")
    parser.add_argument("--list", action="store_true", help="List available scenes")
    parser.add_argument(
        "--scene",
        default="hello_world",
        choices=registry.scene_ids,
        help="Scene id to run",
    )
    parser.add_argument("--steps", type=int, default=240, help="Steps to simulate")
    parser.add_argument(
        "--dt",
        type=float,
        default=1.0 / 240.0,
        help="Timestep passed to the scene",
    )
    parser.add_argument(
        "--log-interval",
        type=int,
        default=60,
        help="Log every N steps (0 to disable)",
    )
    args = parser.parse_args(argv)

    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")

    if args.list:
        _list_scenes(registry.scenes)
        return

    scene = registry.create(args.scene)
    logger.info("Running scene '%s' for %d steps (dt=%s)", args.scene, args.steps, args.dt)
    _run_scene(scene, args.steps, args.dt, args.log_interval)
    logger.info("Done")


if __name__ == "__main__":
    main()

