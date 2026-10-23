import dartpy as dart


def main():
    world = dart.simulation.World()

    urdfParser = dart.utils.DartLoader()
    arm = urdfParser.parseSkeleton("dart://sample/urdf/six_axis_arm/six_axis_arm.urdf")
    ground = urdfParser.parseSkeleton("dart://sample/urdf/six_axis_arm/ground.urdf")
    world.addSkeleton(arm)
    world.addSkeleton(ground)
    print("Robot {} is loaded".format(arm.getName()))

    for i in range(100):
        if i % 10 == 0:
            print(
                "[{}] joint position: {}".format(
                    world.getSimFrames(), arm.getPositions()
                )
            )
        world.step()


if __name__ == "__main__":
    main()
