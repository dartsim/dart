#include "dynamics/SkeletonDynamics.h"
#include "model3d/FileInfoSkel.hpp"
#include "model3d/FileInfoDof.h"

#include "MyWindow.h"

#include <iostream>

using namespace std;
#include "utils/Paths.h"

int main(int argc, char* argv[])
{
    const char* modelfile;
    const char* doffile;
    if(argc!=3){
		modelfile = GROUNDZERO_DATA_PATH"skel/YutingEuler.skel";
		doffile = GROUNDZERO_DATA_PATH"dof/RHand.dof";
    }else{
        modelfile = argv[1];
        doffile = argv[2];
    }
	
    model3d::FileInfoSkel<dynamics::SkeletonDynamics> model;
    model.loadFile(modelfile, model3d::SKEL);
    // model.getSkel();

    model3d::FileInfoDof motion(model.getSkel());
    motion.loadFile(doffile);

    MyWindow window(motion);
    window.computeMax();

    glutInit(&argc, argv);
    window.initWindow(640, 480, modelfile);
    glutMainLoop();

    return 0;
}
