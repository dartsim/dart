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

    // load a skeleton file
    FileInfoSkel<SkeletonDynamics> model1, model2;
    model1.loadFile(DART_DATA_PATH"/skel/arm.skel", SKEL);
    model2.loadFile(DART_DATA_PATH"/skel/sphere.skel", SKEL);

    DartLoader dl;
    SkeletonDynamics* hoop;
    string urdfFileName(DART_DATA_PATH"urdf/hoop.urdf");
    hoop = dl.parseSkeleton(urdfFileName);
    
    // set initial position for the ball
    Eigen::VectorXd pose = model2.getSkel()->getPose();
    pose[0] = 0.45;
    pose[1] = 0.1;
    pose[2] = 0.03;
    model2.getSkel()->setPose(pose);

    pose = hoop->getPose();
    pose[0] = 0.8;
    hoop->setPose(pose);
    hoop->setImmobileState(true);

    // create and initialize the world
    World *myWorld = new World();
    Vector3d gravity(0.0, -9.81, 0.0);
    myWorld->setGravity(gravity);
    myWorld->setTimeStep(1.0/2000);
    myWorld->getCollisionHandle()->getCollisionChecker()->setNumMaxContacts(3);

    myWorld->addSkeleton((SkeletonDynamics*)model1.getSkel());
    myWorld->addSkeleton((SkeletonDynamics*)model2.getSkel());
    myWorld->addSkeleton(hoop);

    // create controller
    Controller *myController = new Controller(myWorld->getSkeleton(0),
                                              myWorld->getTimeStep());

    // create a window and link it to the world and the controller
    MyWindow window;
    window.setWorld(myWorld);
    window.setController(myController);
  
    glutInit(&argc, argv);
    window.initWindow(640, 480, "Freethrow!");
    glutMainLoop();

    return 0;
}
