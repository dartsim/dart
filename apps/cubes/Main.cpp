#include "MyWindow.h"
#include "kinematics/FileInfoSkel.hpp"
#include "utils/Paths.h"

using namespace kinematics;
using namespace dynamics;
using namespace std;

int main(int argc, char* argv[])
{
    FileInfoSkel<SkeletonDynamics> model, model2, model3, model4;
    model.loadFile(DART_DATA_PATH"/skel/ground1.skel", SKEL);
    model2.loadFile(DART_DATA_PATH"/skel/cube2.skel", SKEL);
    model3.loadFile(DART_DATA_PATH"/skel/cube1.skel", SKEL);
    model4.loadFile(DART_DATA_PATH"/skel/cube1.skel", SKEL);

    MyWindow window((SkeletonDynamics*)model.getSkel(), (SkeletonDynamics*)model2.getSkel(), (SkeletonDynamics*)model3.getSkel(), (SkeletonDynamics*)model4.getSkel(), NULL);

    cout << "space bar: simulation on/off" << endl;
    cout << "'p': playback/stop" << endl;
    cout << "'[' and ']': play one frame backward and forward" << endl;
    cout << "'v': visualization on/off" << endl;
    cout << "'1' and '2': programmed interaction" << endl;
    
   
    glutInit(&argc, argv);
    window.initWindow(640, 480, "Cubes");
    glutMainLoop();

    return 0;
}
