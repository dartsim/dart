#include "utils/Paths.h"
#include "math/Helpers.h"
#include "dynamics/Skeleton.h"
#include "simulation/World.h"
#include "utils/SkelParser.h"

#include "MyWindow.h"

using namespace dart;
using namespace math;
using namespace dynamics;
using namespace simulation;

int main(int argc, char* argv[])
{
    // load a skeleton file
    // create and initialize the world
    dart::simulation::World* myWorld
            = utils::SkelParser::readSkelFile(DART_DATA_PATH"/skel/chain.skel");
    assert(myWorld != NULL);
    
    // create and initialize the world
    Eigen::Vector3d gravity(0.0, -9.81, 0.0);
    myWorld->setGravity(gravity);
    myWorld->setTimeStep(1.0/2000);

    int dof =  myWorld->getSkeleton(0)->getNumGenCoords();
    Eigen::VectorXd initPose(dof);
    for (int i = 0; i < dof; i++)
        initPose[i] = random(-0.5, 0.5);
    myWorld->getSkeleton(0)->setConfig(initPose);

    // create a window and link it to the world
    MyWindow window;
    window.setWorld(myWorld);
  
    glutInit(&argc, argv);
    window.initWindow(640, 480, "Forward Simulation");
    glutMainLoop();

    return 0;
}
