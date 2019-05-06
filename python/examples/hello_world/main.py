import dartpy as dart


def main():
    world = dart.simulation.World.create()

    urdfParser = dart.utils.DartLoader()
    kr5 = urdfParser.parseSkeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf")
    ground = urdfParser.parseSkeleton("dart://sample/urdf/KR5/ground.urdf")
    world.addSkeleton(kr5)
    world.addSkeleton(ground)
    print('Robot {} is loaded'.format(kr5.getName()))

    for i in range(100):
        if i % 10 is 0:
            print('[{}] joint position: {}'.format(world.getSimFrames(), kr5.getPositions()))
        world.step()


if __name__ == "__main__":
    main()
