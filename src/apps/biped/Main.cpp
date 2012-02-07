#include "MyWindow.h"
#include "dynamics/BodyNodeDynamics.h"
#include "dynamics/SkeletonDynamics.h"
#include "kinematics/FileInfoSkel.hpp"
#include "utils/Paths.h"



using namespace std;
using namespace Eigen;
using namespace kinematics;
using namespace dynamics;

int main(int argc, char* argv[])
{
    FileInfoSkel<SkeletonDynamics> model, model2, model3, model4;
    model.loadFile(DART_DATA_PATH"/skel/ground1.skel", kinematics::SKEL);
    model2.loadFile(DART_DATA_PATH"/skel/fullbody.skel", kinematics::SKEL);

    MyWindow window((SkeletonDynamics*)model.getSkel(), (SkeletonDynamics*)model2.getSkel(), (SkeletonDynamics*)model3.getSkel(), (SkeletonDynamics*)model4.getSkel(), NULL);

    cout << "space bar: simulation on/off" << endl;
    cout << "'s': simulate one step" << endl;
    cout << "'p': playback/stop" << endl;
    cout << "'[' and ']': play one frame backward and forward" << endl;
    cout << "'v': visualization on/off" << endl;
    cout << "'1' and '2': programmed interaction" << endl;
    
   
    glutInit(&argc, argv);
    window.initWindow(640, 480, "Cubes");
    glutMainLoop();

    return 0;
}
