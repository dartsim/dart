#include "MyWindow.h"
#include "dynamics/SkeletonDynamics.h"
#include "kinematics/FileInfoSkel.hpp"
#include "utils/Paths.h"

using namespace std;
using namespace kinematics;
using namespace dynamics;

int main(int argc, char* argv[])
{
    FileInfoSkel<SkeletonDynamics> model, model2;
    model.loadFile(DART_DATA_PATH"/skel/ground1.skel", SKEL);
    model2.loadFile(DART_DATA_PATH"/skel/fullbody1.skel", SKEL);

    MyWindow window((SkeletonDynamics*)model.getSkel(), (SkeletonDynamics*)model2.getSkel(), NULL);

    cout << "space bar: simulation on/off" << endl;
    cout << "'p': playback/stop" << endl;
    cout << "'[' and ']': play one frame backward and forward" << endl;
    cout << "'v': visualization on/off" << endl;
    cout << "'1' and '2': programmed interaction" << endl;
       
    glutInit(&argc, argv);
    window.initWindow(640, 480, "Balance");
    glutMainLoop();

    return 0;
}
