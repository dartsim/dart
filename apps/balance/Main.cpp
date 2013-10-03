#include "MyWindow.h"

#include "utils/Paths.h"
#include "dynamics/Skeleton.h"
#include "simulation/World.h"
#include "utils/SkelParser.h"

using namespace dart;
using namespace dynamics;
using namespace simulation;

int main(int argc, char* argv[])
{
    // load a skeleton file
    // create and initialize the world
    dart::simulation::World* myWorld
            = utils::SkelParser::readSkelFile(DART_DATA_PATH"skel/fullbody1.skel");
    assert(myWorld != NULL);

    Eigen::Vector3d gravity(0.0, -9.81, 0.0);
    myWorld->setGravity(gravity);

    std::vector<int> genCoordIds;
    genCoordIds.push_back(1);
    genCoordIds.push_back(6); // left hip
    genCoordIds.push_back(9); // left knee
    genCoordIds.push_back(10); // left ankle
    genCoordIds.push_back(13); // right hip
    genCoordIds.push_back(16); // right knee
    genCoordIds.push_back(17); // right ankle
    genCoordIds.push_back(21); // lower back
    Eigen::VectorXd initConfig(8);
    initConfig << -0.1, 0.2, -0.5, 0.3, 0.2, -0.5, 0.3, -0.1;
    myWorld->getSkeleton(1)->setConfig(genCoordIds, initConfig);

    // create controller
    Controller* myController = new Controller(myWorld->getSkeleton(1),
                                              myWorld->getConstraintHandler(),
                                              myWorld->getTimeStep());

    // create a window and link it to the world
    MyWindow window;
    window.setWorld(myWorld);
    window.setController(myController);

    std::cout << "space bar: simulation on/off" << std::endl;
    std::cout << "'p': playback/stop" << std::endl;
    std::cout << "'[' and ']': play one frame backward and forward" << std::endl;
    std::cout << "'v': visualization on/off" << std::endl;
    std::cout << "'1'--'4': programmed interaction" << std::endl;

    glutInit(&argc, argv);
    window.initWindow(640, 480, "Balance");
    glutMainLoop();

    return 0;
}

