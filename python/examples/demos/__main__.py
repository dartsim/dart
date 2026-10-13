"""Entry point for ``python -m examples.demos``."""

import sys

import dartpy as dart


def main():
    if not hasattr(dart, "viewer"):
        print(
            "This build lacks the demo app host; rebuild with the bundled docking "
            "ImGui (-DDART_USE_SYSTEM_IMGUI=OFF, as passed by pixi run config).",
            file=sys.stderr,
        )
        return 1

    from .registry import make_demo_scenes

    return dart.viewer.run(make_demo_scenes(), sys.argv)


if __name__ == "__main__":
    sys.exit(main())
