#include "MyWindow.h"
#include "dynamics/BodyNodeDynamics.h"
#include "dynamics/SkeletonDynamics.h"

#include "model3d/FileInfoSkel.hpp"

#include "utils/Paths.h"
#include "collision/collision.h"




using namespace std;
using namespace Eigen;
using namespace model3d;
using namespace dynamics;

int main(int argc, char* argv[])
{
    FileInfoSkel<SkeletonDynamics> model;
    model.loadFile(GROUNDZERO_DATA_PATH"/skel/Chain.skel", model3d::SKEL);

    MyWindow window((SkeletonDynamics*)model.getSkel());
    
    glutInit(&argc, argv);
    window.initWindow(640,480,"expForwardDyn");
    glutMainLoop();

    return 0;
}
