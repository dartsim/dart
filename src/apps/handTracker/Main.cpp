#include "kinematics/FileInfoSkel.hpp"
#include "kinematics/FileInfoDof.h"
#include "dynamics/SkeletonDynamics.h"
#include "utils/Paths.h"
#include "MyWindow.h"

using namespace std;
using namespace kinematics;
using namespace dynamics;

int main(int argc, char* argv[])
{
    const char* modelfile;
    const char* doffile;
    if(argc!=3){
        modelfile = GROUNDZERO_DATA_PATH"skel/fixedHand.skel";
        //		doffile = GROUNDZERO_DATA_PATH"dof/RHand.dof";
    }else{
        modelfile = argv[1];
        doffile = argv[2];
    }
    
    cout << "This app demonstrates how to use PD controllers to track a hand pose from a random initial pose" << endl;
    cout << "Press q: add an upward force to the palm" << endl;
    cout << "Press a: add a downward force to the middle finger" << endl;
    FileInfoSkel<SkeletonDynamics> model;
    model.loadFile(modelfile, kinematics::SKEL);

    //    FileInfoDof motion(model.getSkel());
    //    motion.loadFile(doffile);

    MyWindow window((SkeletonDynamics*)model.getSkel());

    glutInit(&argc, argv);
    window.initWindow(640, 480, modelfile);
    glutMainLoop();

    return 0;
}
