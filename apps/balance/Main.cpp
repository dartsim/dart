#include "simulation/World.h"
#include "dynamics/SkeletonDynamics.h"
#include "kinematics/FileInfoSkel.hpp"
#include "utils/Paths.h"

#include "MyWindow.h"

using namespace Eigen;
using namespace std;
using namespace kinematics;
using namespace dynamics;
using namespace simulation;

int main(int argc, char* argv[])
{
    // load a skeleton file
    FileInfoSkel<SkeletonDynamics> model, model2;
    model.loadFile(DART_DATA_PATH"/skel/ground1.skel", SKEL);
    model2.loadFile(DART_DATA_PATH"/skel/fullbody1.skel", SKEL);
    // create and initialize the world
    World *myWorld = new World();
    Vector3d gravity(0.0, -9.81, 0.0);
    myWorld->setGravity(gravity);

    ((SkeletonDynamics*)model.getSkel())->setImmobileState(true);
    myWorld->addSkeleton((SkeletonDynamics*)model.getSkel());
    myWorld->addSkeleton((SkeletonDynamics*)model2.getSkel());

    VectorXd initPose = myWorld->getSkeleton(0)->get_q();
    initPose[1] = -0.92;
    myWorld->getSkeleton(0)->setPose(initPose);

    initPose = myWorld->getSkeleton(1)->get_q();
    initPose[1] = -0.1;
    initPose[6] = 0.2; // left hip
    initPose[9] = -0.5; // left knee
    initPose[10] = 0.3; // left ankle
    initPose[13] = 0.2; // right hip
    initPose[16] = -0.5; // right knee
    initPose[17] = 0.3; // right ankle
    initPose[21] = -0.1; // lower back
    myWorld->getSkeleton(1)->setPose(initPose);
    
    // create controller
    Controller *myController = new Controller(myWorld->getSkeleton(1),
                                              myWorld->getCollisionHandle(),
                                              myWorld->getTimeStep());

    // create a window and link it to the world
    MyWindow window;
    window.setWorld(myWorld);
    window.setController(myController);


    cout << "space bar: simulation on/off" << endl;
    cout << "'p': playback/stop" << endl;
    cout << "'[' and ']': play one frame backward and forward" << endl;
    cout << "'v': visualization on/off" << endl;
    cout << "'1'--'4': programmed interaction" << endl;
       
    glutInit(&argc, argv);
    window.initWindow(640, 480, "Balance");
    glutMainLoop();

    return 0;
}

