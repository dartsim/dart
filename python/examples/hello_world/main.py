import argparse

import dartpy as dart


def _build_world():
    world = dart.simulation.World()
    urdf_parser = dart.utils.UrdfParser()
    robot = urdf_parser.parseSkeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf")
    ground = urdf_parser.parseSkeleton("dart://sample/urdf/KR5/ground.urdf")
    world.addSkeleton(robot)
    world.addSkeleton(ground)
    world.setGravity([0, -9.81, 0])
    return world, robot


def _run_headless(world, robot, steps, log_every):
    print("Robot {} is loaded".format(robot.getName()))
    for _ in range(steps):
        if log_every > 0 and world.getSimFrames() % log_every == 0:
            print(
                "[{}] joint position: {}".format(
                    world.getSimFrames(), robot.getPositions()
                )
            )
        world.step()


def _run_gui(world):
    node = dart.gui.RealTimeWorldNode(world)

    viewer = dart.gui.Viewer([1.0, 1.0, 1.0, 1.0])
    viewer.addWorldNode(node)

    grid = dart.gui.GridVisual()
    grid.setPlaneType(dart.gui.GridVisual.PlaneType.ZX)
    grid.setOffset([0, -0.55, 0])
    viewer.addAttachment(grid)

    viewer.setUpViewInWindow(0, 0, 640, 480)
    viewer.setCameraHomePosition([2.0, 1.0, 2.0], [0.0, 0.0, 0.0], [-0.24, 0.94, -0.25])
    viewer.run()


def _parse_args():
    parser = argparse.ArgumentParser(description="Run the DART hello world example.")
    parser.add_argument(
        "--gui",
        action="store_true",
        help="Launch the GUI viewer instead of running headless steps.",
    )
    parser.add_argument(
        "--steps",
        type=int,
        default=100,
        help="Number of headless simulation steps to run.",
    )
    parser.add_argument(
        "--log-every",
        type=int,
        default=10,
        help="Log joint positions every N steps (headless only).",
    )
    return parser.parse_args()


def main():
    args = _parse_args()
    world, robot = _build_world()
    if args.gui:
        print("Robot {} is loaded".format(robot.getName()))
        _run_gui(world)
    else:
        _run_headless(world, robot, args.steps, args.log_every)


if __name__ == "__main__":
    main()
