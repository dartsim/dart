#include "MyWindow.h"

#include "utils/Paths.h"
#include "dynamics/Skeleton.h"
#include "simulation/World.h"
#include "utils/SkelParser.h"

using namespace Eigen;
using namespace std;

using namespace dart;
using namespace dynamics;

int main(int argc, char* argv[])
{
    dart::simulation::World* myWorld
            = dart::utils::readSkelFile(DART_DATA_PATH"skel/fullbody1.skel");
    assert(myWorld != NULL);

    MyWindow window(myWorld->getSkeleton(0), myWorld->getSkeleton(1));
    //window.setWorld(myWorld);

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
