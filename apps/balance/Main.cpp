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

    Eigen::VectorXd initPose = myWorld->getSkeleton(1)->get_q();
    initPose[1] = -0.1;
    initPose[6] = 0.2; // left hip
    initPose[9] = -0.5; // left knee
    initPose[10] = 0.3; // left ankle
    initPose[13] = 0.2; // right hip
    initPose[16] = -0.5; // right knee
    initPose[17] = 0.3; // right ankle
    initPose[21] = -0.1; // lower back
    myWorld->getSkeleton(1)->setConfig(initPose, true);

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

