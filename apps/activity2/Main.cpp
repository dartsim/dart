#include "dynamics/SkeletonDynamics.h"
#include "kinematics/FileInfoSkel.hpp"
#include "utils/Paths.h"
#include "utils/UtilsMath.h"
#include "simulation/World.h"

#include "MyWindow.h"
#include "Controller.h"

using namespace kinematics;
using namespace dynamics;
using namespace simulation;
using namespace utils;

int main(int argc, char* argv[])
{
    // load a skeleton file
    FileInfoSkel<SkeletonDynamics> model;
    model.loadFile(DART_DATA_PATH"/skel/nunchuck.skel", SKEL);
    FileInfoSkel<SkeletonDynamics> ground;
    ground.loadFile(DART_DATA_PATH"/skel/ground1.skel", SKEL);
    FileInfoSkel<SkeletonDynamics> box;
    box.loadFile(DART_DATA_PATH"/skel/cube1.skel", SKEL);
    
    // create and initialize the world
    World *myWorld = new World();
    Vector3d gravity(0.0, -9.81, 0.0);
    myWorld->setGravity(gravity);

    ((SkeletonDynamics*)ground.getSkel())->setImmobileState(true);
    myWorld->addSkeleton((SkeletonDynamics*)ground.getSkel());
    VectorXd initPose = myWorld->getSkeleton(0)->getPose();
    initPose[1] = -0.46;
    myWorld->getSkeleton(0)->setPose(initPose);

    myWorld->addSkeleton((SkeletonDynamics*)box.getSkel());
    initPose = myWorld->getSkeleton(1)->getPose();
    initPose[0] = 0.1;
    myWorld->getSkeleton(1)->setPose(initPose);

    myWorld->addSkeleton((SkeletonDynamics*)model.getSkel());
    int nDof = myWorld->getSkeleton(2)->getNumDofs();
    initPose.resize(nDof);
    for (int i = 0; i < nDof; i++)
        initPose[i] = random(-0.5, 0.5);
    myWorld->getSkeleton(2)->setPose(initPose);

    myWorld->setTimeStep(1.0/2000);

    // create a window and link it to the world
    MyWindow window;
    window.setWorld(myWorld);

    // create controller
    Controller *myController = new Controller(myWorld->getSkeleton(2));
    window.setController(myController);
  
    glutInit(&argc, argv);
    window.initWindow(640, 480, "Forward Simulation");
    glutMainLoop();

    return 0;
}
