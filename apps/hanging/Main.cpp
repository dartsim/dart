#include "MyWindow.h"

#include <iostream>

#include "utils/Paths.h"
#include "utils/SkelParser.h"

using namespace std;
using namespace dart;

int main(int argc, char* argv[])
{
    dart::simulation::World* myWorld
            = utils::SkelParser::readSkelFile(DART_DATA_PATH"skel/fullbody1.skel");
    assert(myWorld != NULL);

    // create a window and link it to the world
    MyWindow window(myWorld);

    cout << "space bar: simulation on/off" << endl;
    cout << "'p': playback/stop" << endl;
    cout << "'[' and ']': play one frame backward and forward" << endl;
    cout << "'v': visualization on/off" << endl;
    cout << "'1''--'4': programmed interaction" << endl;
       
    glutInit(&argc, argv);
    window.initWindow(640, 480, "Balance");
    glutMainLoop();

    return 0;
}
