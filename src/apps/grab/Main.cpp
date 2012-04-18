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
    FileInfoSkel<SkeletonDynamics> model, model2, model3;
    model.loadFile(DART_DATA_PATH"/skel/ground1.skel", kinematics::SKEL);
    model2.loadFile(DART_DATA_PATH"/skel/simpleHand.skel", kinematics::SKEL);
    model3.loadFile(DART_DATA_PATH"/skel/cube1.skel", kinematics::SKEL);

    MyWindow window((SkeletonDynamics*)model.getSkel(), (SkeletonDynamics*)model2.getSkel(), (SkeletonDynamics*)model3.getSkel(), NULL);
   
    glutInit(&argc, argv);
    window.initWindow(640, 480, "Grab");
    glutMainLoop();

    return 0;
}
