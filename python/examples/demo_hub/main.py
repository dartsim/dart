from pathlib import Path
import sys

# Ensure the demo_hub package (under python/) is importable when running as an example.
REPO_ROOT = Path(__file__).resolve().parents[3]
REPO_PYTHON = REPO_ROOT / "python"
if str(REPO_PYTHON) not in sys.path:
    sys.path.insert(0, str(REPO_PYTHON))

import argparse

from demo_hub.app import main as headless_main


def dispatch(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description="DART demo hub",
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--gui", dest="mode", action="store_const", const="gui", help="Launch the GUI (default)")
    mode.add_argument(
        "--headless", dest="mode", action="store_const", const="headless", help="Run headless CLI (no GUI)"
    )
    parser.set_defaults(mode="gui")
    args, remaining = parser.parse_known_args(argv)

    if "--help" in (argv or []):
        if args.mode == "gui":
            from demo_hub.gui.app import main as gui_main

            gui_main(["--help"])
        else:
            headless_main(["--help"])
        return

    if args.mode == "gui":
        from demo_hub.gui.app import main as gui_main
        from demo_hub.gui.app import has_gui_deps

        if not has_gui_deps():
            print(
                "GUI dependencies are missing; falling back to headless mode. "
                "Install imgui[glfw], glfw, and PyOpenGL in the pixi environment to enable the GUI."
            )
            headless_main(remaining)
            return

        gui_main(remaining)
    else:
        headless_main(remaining)


if __name__ == "__main__":
    dispatch()
