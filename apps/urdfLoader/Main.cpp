#include "MyWindow.h"
#include "kinematics/FileInfoSkel.hpp"
#include "robotics/parser/dart_parser/DartLoader.h"
#include "utils/Paths.h"

using namespace kinematics;
using namespace dynamics;
using namespace std;

int main(int argc, char* argv[])
{
    FileInfoSkel<SkeletonDynamics> model;
    model.loadFile(DART_DATA_PATH"skel/ground1.skel", SKEL);
    DartLoader urdfLoader;
    urdfLoader.mPath = DART_DATA_PATH"urdf/atlas/";
    SkeletonDynamics* model2 = urdfLoader.parseSkeleton(DART_DATA_PATH"urdf/atlas/atlas_no_head.urdf");

    MyWindow window((SkeletonDynamics*)model.getSkel(), model2, NULL);

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
