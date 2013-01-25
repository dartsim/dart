#include "MyWindow.h"
#include "kinematics/FileInfoSkel.hpp"
#include "utils/Paths.h"

using namespace kinematics;
using namespace dynamics;
using namespace std;

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

    MyWindow window(motion, (SkeletonDynamics*)model.getSkel());
   
    cout << "space bar: simulation on/off" << endl;
    cout << "'p': playback/stop" << endl;
    cout << "'[' and ']': play one frame backward and forward" << endl;

    glutInit(&argc, argv);
    window.initWindow(640, 480, "Open Loop Control");
    glutMainLoop();

    return 0;
}
