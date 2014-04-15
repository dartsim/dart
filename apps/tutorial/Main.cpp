#include "dynamics/SkeletonDynamics.h"
#include "dynamics/ConstraintDynamics.h"
#include "collision/dart/DARTCollisionDetector.h"
#include "kinematics/FileInfoSkel.hpp"
#include "kinematics/BodyNode.cpp"
#include "simulation/World.h"
#include "utils/Paths.h"
#include "MyWindow.h"
#include "PDController.h"
#include "SPDController.h"
#include "JTController.h"
#include "JumpController.h"
#include "SwingController.h"

using namespace kinematics;
using namespace dynamics;
using namespace simulation;

int main(int argc, char* argv[])
{
    // Load skeleton files
    FileInfoSkel<SkeletonDynamics> model1, model2;
    model1.loadFile(DART_DATA_PATH"/skel/fullbody1.skel", SKEL);
    model2.loadFile(DART_DATA_PATH"/skel/ground1.skel", SKEL);

    // Initialize skeletons
    // uncomment this for swing controller
    
    // Eigen::VectorXd pose1 = model1.getSkel()->getPose();
    // pose1[27] = -2.5;
    // pose1[28] = 0.75;
    // pose1[29] = 0.3;
    // pose1[33] = 2.5;
    // pose1[34] = -0.75;
    // pose1[35] = 0.3;
    // model1.getSkel()->setPose(pose1);
    

    Eigen::VectorXd pose2 = model2.getSkel()->getPose();
    pose2[1] = -0.95;
    model2.getSkel()->setPose(pose2);

    // Set ground to be immobile object
    model2.getSkel()->setImmobileState(true);

    // Create and initialize the world
    World *myWorld = new World();
    Vector3d gravity(0.0, -9.8, 0.0);
    myWorld->setGravity(gravity);
    myWorld->setTimeStep(1.0/2000);
    myWorld->getCollisionHandle()->setCollisionChecker(new collision::DARTCollisionDetector());

    myWorld->addSkeleton((SkeletonDynamics*)model1.getSkel());
    myWorld->addSkeleton((SkeletonDynamics*)model2.getSkel());

    // Create a controller
    //Controller* myController = new Controller(myWorld->getSkeleton(0));
    //PDController* myController = new PDController(myWorld->getSkeleton(0));

    //SPDController* myController = new SPDController(myWorld->getSkeleton(0), myWorld->getTimeStep());

    //JTController* myController = new JTController(myWorld->getSkeleton(0), myWorld->getSkeleton(0)->getNode("fullbody1_h_heel_left"));

    JumpController* myController = new JumpController(myWorld->getSkeleton(0), myWorld->getTimeStep());

    //SwingController* myController = new SwingController(myWorld->getSkeleton(0), myWorld->getCollisionHandle(), myWorld->getTimeStep());

    // Create a window
    MyWindow window;
    window.setWorld(myWorld);
    window.setController(myController);
  
    glutInit(&argc, argv);
    window.initWindow(640, 480, "My First Controller");
    std::cout << "space bar: simulation on/off" << std::endl;
    std::cout << "'p': playback/stop" << std::endl;
    std::cout << "'[' and ']': play one frame backward and forward" << std::endl;
    std::cout << "'v': contact visualization on/off" << std::endl;
    std::cout << "'1': push the character" << std::endl;
    std::cout << "Left click: rotate camera" << std::endl;
    std::cout << "Right click: pan camera" << std::endl;
    std::cout << "Shift + Left click: zoom camera" << std::endl;
    
    glutMainLoop();

    return 0;
}
