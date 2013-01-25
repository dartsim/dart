#include "dynamics/SkeletonDynamics.h"
#include "kinematics/FileInfoSkel.hpp"
#include "utils/Paths.h"

#include "MyWindow.h"

using namespace kinematics;
using namespace dynamics;

int main(int argc, char* argv[])
{
    FileInfoSkel<SkeletonDynamics> model;
    model.loadFile(DART_DATA_PATH"/skel/Chain.skel", SKEL);

    MyWindow window((SkeletonDynamics*)model.getSkel());
    
    glutInit(&argc, argv);
    window.initWindow(640, 480, "Forward Simulation");
    glutMainLoop();

    return 0;
}
