#include <iostream>

#include "dart/dart.h"
#include "MyWindow.h"


int main(int argc, char* argv[]) 
{
    // create and initialize the world
    dart::simulation::WorldPtr mWorld = dart::utils::SkelParser::readWorld(DART_DATA_PATH"skel/ground.skel");
    assert(myWorld != nullptr);

    Eigen::Vector3d gravity(0.0, -9.81, 0.0);
    mWorld->setGravity(gravity);

    // create a window and link it to the world
    MyWindow window(mWorld);

    glutInit(&argc, argv);
    window.initWindow(640, 480, "Balance");
    glutMainLoop();

    return 0;
}

