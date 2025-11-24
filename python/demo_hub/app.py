from __future__ import annotations

import argparse
import logging
from pathlib import Path
from typing import Iterable

from demo_hub.core import Recorder, Scene, build_default_registry

logger = logging.getLogger(__name__)


def _list_scenes(scenes: Iterable) -> None:
    for meta in scenes:
        tags = f" [{' '.join(meta.tags)}]" if meta.tags else ""
        print(f"{meta.scene_id}: {meta.label}{tags}\n  {meta.summary}\n")


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
    parser.add_argument(
        "--record",
        type=Path,
        help="Optional JSONL file to record scene state (export_state output per step).",
    )
    parser.add_argument(
        "--record-interval",
        type=int,
        default=1,
        help="Record every N steps (only when --record is set).",
    )
    args = parser.parse_args(argv)

    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")

    if args.list:
        _list_scenes(registry.scenes)
        return

    scene = registry.create(args.scene)
    scene.setup(args.dt)
    recorder: Recorder | None = None
    if args.record:
        recorder = Recorder()
        recorder.start(
            args.record,
            {"scene": args.scene, "dt": args.dt, "steps": args.steps},
        )
        logger.info("Recording to %s", args.record)
    logger.info("Running scene '%s' for %d steps (dt=%s)", args.scene, args.steps, args.dt)
    try:
        for step in range(args.steps):
            scene.update(args.dt)
            if args.log_interval > 0 and (step + 1) % args.log_interval == 0:
                state = scene.export_state()
                if state:
                    logger.info("Step %d: %s", step + 1, state)
            if recorder and (step % args.record_interval == 0):
                recorder.log(step, scene.export_state() or {})
    finally:
        if recorder:
            recorder.stop()
    logger.info("Done")


if __name__ == "__main__":
    main()
