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
<<<<<<< HEAD
    if(argc!=3){
		modelfile = DART_DATA_PATH"skel/YutingEuler.skel";
		doffile = DART_DATA_PATH"dof/RHand.dof";
=======
    const char* doffile1;
    if(argc!=3){
        //		modelfile = DART_DATA_PATH"skel/YutingEuler.skel";
        //		modelfile = DART_DATA_PATH"skel/Nick01.vsk";
        modelfile = DART_DATA_PATH"skel/fullbody1.skel";
        //                modelfile = DART_DATA_PATH"skel/fixedHand.skel";
        
        //doffile = DART_DATA_PATH"dof/RHand.dof";
                		doffile = DART_DATA_PATH"dof/result01_vertical.dof";
                //		doffile = DART_DATA_PATH"dof/simMotion.dof";
                //		doffile = DART_DATA_PATH"dof/fixedHand.dof";
                //		doffile1 = DART_DATA_PATH"dof/simMotion1.dof";
>>>>>>> karen
    }else{
        modelfile = argv[1];
        doffile = argv[2];
    }
	
    FileInfoSkel<Skeleton> model;
    model.loadFile(modelfile);
    // model.getSkel();

    FileInfoDof motion(model.getSkel());
    motion.loadFile(doffile);
<<<<<<< HEAD

    MyWindow window(motion);
=======
    //    FileInfoDof motion1(model.getSkel());
    //motion1.loadFile(doffile1);

    MyWindow window(motion);
    //    MyWindow window(motion, motion1);
>>>>>>> karen
    window.computeMax();

    glutInit(&argc, argv);
    window.initWindow(640, 480, modelfile);
    glutMainLoop();

    return 0;
}
