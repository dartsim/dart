#include "MyWindow.h"
#include "kinematics/FileInfoSkel.hpp"
#include "kinematics/BodyNode.h"
#include "kinematics/Shape.h"
#include "utils/Paths.h"
#include <fstream>
#include <iostream>

using namespace kinematics;
using namespace dynamics;
using namespace Eigen;
using namespace std;


int main(int argc, char* argv[])
{  
    FileInfoSkel<SkeletonDynamics> model;
    model.loadFile(DART_DATA_PATH"/skel/mobilManipulator.skel", SKEL);

    MyWindow window((SkeletonDynamics*)model.getSkel(), NULL);
   
    glutInit(&argc, argv);
    window.initWindow(640, 480, "Hybrid");
    glutMainLoop();

    return 0;
}
