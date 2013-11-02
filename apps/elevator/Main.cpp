#include "dynamics/SkeletonDynamics.h"
#include "dynamics/ConstraintDynamics.h"
#include "kinematics/FileInfoSkel.hpp"
#include "utils/Paths.h"
#include "math/UtilsMath.h"
#include "simulation/World.h"
#include "robotics/parser/dart_parser/DartLoader.h"

#include "MyWindow.h"

using namespace kinematics;
using namespace dynamics;
using namespace simulation;
using namespace dart_math;

int main(int argc, char* argv[])
{
    // load skeleton files
    FileInfoSkel<SkeletonDynamics> model1, model2, model3;
    model1.loadFile(DART_DATA_PATH"/skel/fullbody1.skel", SKEL);
    model2.loadFile(DART_DATA_PATH"/skel/elevator.skel", SKEL);
    model3.loadFile(DART_DATA_PATH"/skel/plane.skel", SKEL);


    // set ground position
    Eigen::VectorXd pose = model3.getSkel()->getPose();
    pose[1] = -10.0;
    model3.getSkel()->setPose(pose);
    model3.getSkel()->setImmobileState(true);

    // set human position
    pose = model1.getSkel()->getPose();
    pose[1] = -0.3;
    model1.getSkel()->setPose(pose);
    
    // create and initialize the world
    World *myWorld = new World();
    Vector3d gravity(0.0, -9.81, 0.0);
    myWorld->setGravity(gravity);
    myWorld->setTimeStep(1.0/2000);
    myWorld->getCollisionHandle()->getCollisionChecker()->setNumMaxContacts(3);

    myWorld->addSkeleton((SkeletonDynamics*)model1.getSkel());
    myWorld->addSkeleton((SkeletonDynamics*)model2.getSkel());
    myWorld->addSkeleton((SkeletonDynamics*)model3.getSkel());

    // create controller
    Controller *myController = new Controller(myWorld->getSkeleton(0),
                                              myWorld->getTimeStep());

    // create a window and link it to the world and the controller
    MyWindow window;
    window.setWorld(myWorld);
    window.setController(myController);
  
    glutInit(&argc, argv);
    window.initWindow(640, 480, "This is going to hurt!");
    glutMainLoop();

    return 0;
}
