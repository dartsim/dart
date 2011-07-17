#include "model3d/FileInfoModel.h"
#include "model3d/FileInfoDof.h"

#include "MyWindow.h"

#include <iostream>

using namespace std;
using namespace model3d;
#include "utils/LoadOpengl.h"

int main(int argc, char* argv[])
{
    const char* modelfile;
    const char* doffile;
    if(argc!=3){
		modelfile = "YutingEuler.skel";
		doffile = "RHand.dof";
    }else{
        modelfile = argv[1];
        doffile = argv[2];
    }
	
    FileInfoModel model;
    model.loadFile(modelfile,FileInfoModel::SKEL);

    FileInfoDof motion(model.getModel());
    motion.loadFile(doffile);

    MyWindow window(motion);
    window.computeMax();

    glutInit(&argc, argv);
    window.initWindow(640, 480, modelfile);
    glutMainLoop();

    return 0;
}
