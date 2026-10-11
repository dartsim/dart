"""Show a DART world or capture it without a display using the neutral GUI.

Run with ``pixi run py-ex filament``. For display-free capture, run this script
with ``--capture frame.png`` and the local dartpy extension on PYTHONPATH.
"""

import argparse
from pathlib import Path

import dartpy as dart


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture", type=Path, help="Write a PNG and exit")
    parser.add_argument("--width", type=int)
    parser.add_argument("--height", type=int)
    args = parser.parse_args()
    width = 640 if args.width is None else args.width
    height = 480 if args.height is None else args.height

    world = dart.simulation.World("Filament example")
    box = dart.dynamics.SimpleFrame(dart.dynamics.Frame.World(), "box")
    box.setShape(dart.dynamics.BoxShape([1.0, 1.0, 1.0]))
    box.getVisualAspect(True).setColor([0.8, 0.15, 0.1])
    world.addSimpleFrame(box)

    viewer = dart.gui.Viewer()
    viewer.addWorldNode(dart.gui.WorldNode(world))
    viewer.setCameraHome([2.0, -3.0, 2.0], [0.0, 0.0, 0.0], [0.0, 0.0, 1.0])
    if args.capture:
        viewer.setUpViewForCapture(width, height)
        for _ in range(5):
            viewer.frame()
        viewer.captureScreen(str(args.capture))
        viewer.frame()
        if not args.capture.is_file() or args.capture.stat().st_size == 0:
            raise RuntimeError(f"Capture was not written: {args.capture}")
        print(f"Captured {args.capture} with {viewer.getRenderBackend()}")
    else:
        if args.width is None and args.height is None:
            viewer.setUpViewInWindow()
        else:
            viewer.setUpViewInWindow(0, 0, width, height)
        viewer.run()


if __name__ == "__main__":
    main()
