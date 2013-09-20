#include "utils/Paths.h"
#include "utils/sdf/SdfParser.h"
#include "MyWindow.h"

int main(int argc, char* argv[])
{
    // load a world
    dart::simulation::World* myWorld = dart::utils::SdfParser::readSdfFile(
                DART_DATA_PATH"/sdf/double_pendulum_with_base.world");
    assert(myWorld != NULL);

    // create a window and link it to the world
    MyWindow window;
    window.setWorld(myWorld);

    std::cout << "space bar: simulation on/off" << std::endl;
    std::cout << "'p': playback/stop" << std::endl;
    std::cout << "'[' and ']': play one frame backward and forward" << std::endl;
    std::cout << "'v': visualization on/off" << std::endl;
    std::cout << "'1'--'4': programmed interaction" << std::endl;

    glutInit(&argc, argv);
    window.initWindow(640, 480, "SDFormat Loader");
    glutMainLoop();

    return 0;
}
