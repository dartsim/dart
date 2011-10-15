#include "kinematics/Skeleton.h"
#include "kinematics/FileInfoSkel.hpp"
#include "kinematics/FileInfoDof.h"

#include "MyWindow.h"

#include <iostream>

using namespace std;
using namespace kinematics;
#include "utils/Paths.h"

int main(int argc, char* argv[])
{
    const char* modelfile;
    const char* doffile;
    if(argc!=3){
		modelfile = DART_DATA_PATH"skel/YutingEuler.skel";
		doffile = DART_DATA_PATH"dof/RHand.dof";
    }else{
        modelfile = argv[1];
        doffile = argv[2];
    }
	
    FileInfoSkel<Skeleton> model;
    model.loadFile(modelfile);
    // model.getSkel();

    FileInfoDof motion(model.getSkel());
    motion.loadFile(doffile);

    MyWindow window(motion);
    window.computeMax();

    glutInit(&argc, argv);
    window.initWindow(640, 480, modelfile);
    glutMainLoop();

    return 0;
}
