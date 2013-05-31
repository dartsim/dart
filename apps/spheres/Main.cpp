#include "MyWindow.h"
#include "kinematics/FileInfoSkel.hpp"
#include "utils/Paths.h"

using namespace kinematics;
using namespace dynamics;
using namespace simulation;
using namespace std;
using namespace Eigen;

int main(int argc, char* argv[])
{
    // load a skeleton file
    FileInfoSkel<SkeletonDynamics> model, model2, model3, model4, model5;
    model.loadFile(DART_DATA_PATH"/skel/ground1.skel", SKEL);
    model2.loadFile(DART_DATA_PATH"/skel/sphere.skel", SKEL);
    model3.loadFile(DART_DATA_PATH"/skel/sphere1.skel", SKEL);
    model4.loadFile(DART_DATA_PATH"/skel/sphere1.skel", SKEL);
    model5.loadFile(DART_DATA_PATH"/skel/sphere1.skel", SKEL);
    
    // create and initialize the world
    World *myWorld = new World();
    Vector3d gravity(0.0, -9.81, 0.0);
    myWorld->setGravity(gravity);

    ((SkeletonDynamics*)model.getSkel())->setImmobileState(true);
    myWorld->addSkeleton((SkeletonDynamics*)model.getSkel());
    myWorld->addSkeleton((SkeletonDynamics*)model2.getSkel());
    myWorld->addSkeleton((SkeletonDynamics*)model3.getSkel());
    myWorld->addSkeleton((SkeletonDynamics*)model4.getSkel());
    myWorld->addSkeleton((SkeletonDynamics*)model5.getSkel());

    VectorXd initPose = myWorld->getSkeleton(0)->get_q();
    initPose[1] = -0.35;
    myWorld->getSkeleton(0)->setPose(initPose);

    initPose = myWorld->getSkeleton(1)->get_q();
    initPose[0] = 0.00;            // x-axis
    initPose[1] = -0.35 + 0.1;    // y-axis
    myWorld->getSkeleton(1)->setPose(initPose);

    initPose = myWorld->getSkeleton(2)->get_q();
    initPose[0] = 0.30;            // x-axis
    initPose[1] = -0.35 + 0.1;    // y-axis
    myWorld->getSkeleton(2)->setPose(initPose);

    initPose = myWorld->getSkeleton(3)->get_q();
    initPose[0] = 0.60;                 // x-axis
    initPose[1] = -0.35 + 0.1 + 0.08;  // y-axis
    myWorld->getSkeleton(3)->setPose(initPose);

    initPose = myWorld->getSkeleton(4)->get_q();
    initPose[0] = -0.30;                 // x-axis
    initPose[1] = -0.35 + 0.1 + 0.08;  // y-axis
    myWorld->getSkeleton(4)->setPose(initPose);

    // create a window and link it to the world
    MyWindow window;
    window.setWorld(myWorld);
  
    cout << "space bar: simulation on/off" << endl;
    cout << "'p': playback/stop" << endl;
    cout << "'[' and ']': play one frame backward and forward" << endl;
    cout << "'v': visualization on/off" << endl;
    cout << "'1'--'4': programmed interaction" << endl;

    glutInit(&argc, argv);
    window.initWindow(640, 480, "Spheres");
    glutMainLoop();

    return 0;
}
