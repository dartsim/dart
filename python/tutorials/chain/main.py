"""Compatibility entrypoint for the multi-pendulum tutorial."""

from pathlib import Path
import runpy

if __name__ == "__main__":
    runpy.run_path(
        str(
            Path(__file__).resolve().parents[1] / "multi_pendulum" / Path(__file__).name
        ),
        run_name="__main__",
    )
