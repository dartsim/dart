#include <iostream>
#include "MyWindow.h"
#include "utils/Paths.h"
#include "utils/SkelParser.h"

using namespace dart;

int main(int argc, char* argv[])
{
    // create and initialize the world
    dart::simulation::World *myWorld
            = utils::SkelParser::readSkelFile(
                  DART_DATA_PATH"/skel/cubes.skel");
    assert(myWorld != NULL);
    Eigen::Vector3d gravity(0.0, -9.81, 0.0);
    myWorld->setGravity(gravity);

    // create a window and link it to the world
    MyWindow window;
    window.setWorld(myWorld);
  
    std::cout << "space bar: simulation on/off" << std::endl;
    std::cout << "'p': playback/stop" << std::endl;
    std::cout << "'[' and ']': play one frame backward and forward" << std::endl;
    std::cout << "'v': visualization on/off" << std::endl;
    std::cout << "'1'--'4': programmed interaction" << std::endl;
    std::cout << "'q': spawn a random cube" << std::endl;
    std::cout << "'w': delete a spawned cube" << std::endl;

    glutInit(&argc, argv);
    window.initWindow(640, 480, "Boxes");
    glutMainLoop();

    return 0;
}
