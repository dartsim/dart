#include "MyWindow.h"
#include "kinematics/FileInfoSkel.hpp"
#include "utils/Paths.h"

using namespace kinematics;
using namespace dynamics;

int main(int argc, char* argv[])
{
    const char* modelfile;
    const char* doffile;
    if(argc!=3){
        modelfile = DART_DATA_PATH"skel/fixedHand.skel";
        doffile = DART_DATA_PATH"dof/fixedHand.dof";
    }else{
        modelfile = argv[1];
        doffile = argv[2];
    }

    FileInfoSkel<SkeletonDynamics> model;
    model.loadFile(modelfile, SKEL);
    FileInfoDof *motion = new FileInfoDof(model.getSkel());
    motion->loadFile(doffile);

    MyWindow window(motion, (SkeletonDynamics*)model.getSkel(), NULL);    
   
    glutInit(&argc, argv);
    window.initWindow(640, 480, "PD Control");
    glutMainLoop();

    return 0;
}
