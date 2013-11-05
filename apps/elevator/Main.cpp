#include "dynamics/SkeletonDynamics.h"
#include "dynamics/ConstraintDynamics.h"
#include "kinematics/FileInfoSkel.hpp"
#include "utils/Paths.h"
#include "math/UtilsMath.h"
#include "MyWorld.h"
#include "collision/dart/DARTCollisionDetector.h"

#include "MyWindow.h"

using namespace kinematics;
using namespace dynamics;
using namespace simulation;
using namespace dart_math;

int main(int argc, char* argv[])
{
    // Load skeleton files
    FileInfoSkel<SkeletonDynamics> model1, model2, model3, model4;
    model1.loadFile(DART_DATA_PATH"/skel/fullbody2.skel", SKEL);
    model2.loadFile(DART_DATA_PATH"/skel/elevator.skel", SKEL);
    model3.loadFile(DART_DATA_PATH"/skel/plane.skel", SKEL);
    model4.loadFile(DART_DATA_PATH"/skel/roof.skel", SKEL);

    // Initialize human pose
    Eigen::VectorXd pose = model1.getSkel()->getPose();
    // standing initial pose
    pose[1] = 0.92;
    // lying down initial pose
    //    pose[1] = 0.1;
    //    pose[3] = 1.57;
    model1.getSkel()->setPose(pose);

    // Set ground to be immobile object
    model3.getSkel()->setImmobileState(true);

    // Create and initialize the world
    MyWorld *myWorld = new MyWorld();
    Vector3d gravity(0.0, -9.81, 0.0);
    myWorld->setGravity(gravity);
    myWorld->setTimeStep(1.0/2000);
    myWorld->getCollisionHandle()->setCollisionChecker(new collision::DARTCollisionDetector());

    myWorld->addSkeleton((SkeletonDynamics*)model1.getSkel());
    myWorld->addSkeleton((SkeletonDynamics*)model2.getSkel());
    myWorld->addSkeleton((SkeletonDynamics*)model3.getSkel());
    myWorld->addSkeleton((SkeletonDynamics*)model4.getSkel());

    // Create controller
    Controller *myController = new Controller(myWorld->getSkeleton(0), myWorld->getCollisionHandle(), myWorld->getTimeStep());
    myWorld->setController(myController);


    // Create a window and link it to the world
    MyWindow window;
    window.setWorld(myWorld);
  
    glutInit(&argc, argv);
    window.initWindow(640, 480, "This is going to hurt!");
    glutMainLoop();

    return 0;
}
