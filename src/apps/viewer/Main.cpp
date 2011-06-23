#include "model3d/FileInfoModel.h"
#include "model3d/FileInfoDof.h"

#include "MyWindow.h"

#include <iostream>

using namespace std;
using namespace model3d;

int main(int argc, char* argv[])
{
    if(argc!=3){
        //cout<<"viewer skel_file dof_file"<<endl;
        //exit(0);
		argv = new char*[3];
		argv[1] = "YutingEuler.skel";
		argv[2] = "RHand.dof";
    }
	
    FileInfoModel model;
    model.loadFile(argv[1],FileInfoModel::SKEL);

    FileInfoDof motion(model.getModel());
    motion.loadFile(argv[2]);

    MyWindow window(motion);
    window.computeMax();

    glutInit(&argc, argv);
    window.initWindow(640, 480, argv[1]);
    glutMainLoop();

    return 0;
}
