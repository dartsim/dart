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
    FileInfoSkel<SkeletonDynamics> model, model2, model3;
    model.loadFile(DART_DATA_PATH"/skel/ground1.skel", SKEL);
    model2.loadFile(DART_DATA_PATH"/skel/manipulator.skel", SKEL);
    model3.loadFile(DART_DATA_PATH"/skel/sphere.skel", SKEL);

    Vector3d red(1.0, 0.0, 0.0);
    Vector3d gray(0.9, 0.9, 0.9);
    model.getSkel()->getNode(0)->getShape()->setColor(gray);
    model3.getSkel()->getNode(0)->getShape()->setColor(red);

    MyWindow window((SkeletonDynamics*)model.getSkel(), (SkeletonDynamics*)model2.getSkel(), (SkeletonDynamics*)model3.getSkel(), NULL);
   
    glutInit(&argc, argv);
    window.initWindow(640, 480, "Manipulator");
    glutMainLoop();

    return 0;
}
