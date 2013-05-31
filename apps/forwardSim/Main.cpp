#include "dynamics/SkeletonDynamics.h"
#include "kinematics/FileInfoSkel.hpp"
#include "utils/Paths.h"
#include "math/UtilsMath.h"
#include "simulation/World.h"

#include "MyWindow.h"

using namespace kinematics;
using namespace dynamics;
using namespace simulation;
using namespace dart_math;

int main(int argc, char* argv[])
{
    // load a skeleton file
    FileInfoSkel<SkeletonDynamics> model;
    model.loadFile(DART_DATA_PATH"/skel/Chain.skel", SKEL);
    
    // create and initialize the world
    World *myWorld = new World();
    Vector3d gravity(0.0, -9.81, 0.0);
    myWorld->setGravity(gravity);
    myWorld->setTimeStep(1.0/2000);

    myWorld->addSkeleton((SkeletonDynamics*)model.getSkel());
    int nDof =  myWorld->getSkeleton(0)->getNumDofs();
    VectorXd initPose(nDof);
    for (int i = 0; i < nDof; i++)
        initPose[i] = random(-0.5, 0.5);
    myWorld->getSkeleton(0)->setPose(initPose);

    // create a window and link it to the world
    MyWindow window;
    window.setWorld(myWorld);
  
    glutInit(&argc, argv);
    window.initWindow(640, 480, "Forward Simulation");
    glutMainLoop();

    return 0;
}
