#include "utils/Paths.h"
#include "math/Helpers.h"
#include "dynamics/Skeleton.h"
#include "dynamics/BodyNode.h"
#include "dynamics/Joint.h"
#include "simulation/World.h"
#include "utils/SkelParser.h"
#include "constraint/ConstraintDynamics.h"
#include "constraint/BallJointConstraint.h"

#include "MyWindow.h"

using namespace dart;
using namespace math;
using namespace dynamics;
using namespace simulation;
using namespace constraint;

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
    initPose.setZero();
    initPose[20] = 3.14159 * 0.4;
    initPose[23] = 3.14159 * 0.4;
    initPose[26] = 3.14159 * 0.4;
    initPose[29] = 3.14159 * 0.4;
    myWorld->getSkeleton(0)->setConfig(initPose);

    // create a ball joint constraint
    BodyNode *bd1 = myWorld->getSkeleton(0)->getBodyNode("link 6");
    BodyNode *bd2 = myWorld->getSkeleton(0)->getBodyNode("link 10");
    Eigen::Vector3d offset1 = bd1->getParentJoint()->getTransformFromChildBodyNode().translation();
    Eigen::Vector3d offset2(0.0, -0.025, 0.0);
    BallJointConstraint *cl = new BallJointConstraint(bd1, bd2, offset1, offset2);
    myWorld->getConstraintHandler()->addConstraint(cl);

    // create a window and link it to the world
    MyWindow window;
    window.setWorld(myWorld);
  
    glutInit(&argc, argv);
    window.initWindow(640, 480, "Closed Loop");
    glutMainLoop();

    return 0;
}
