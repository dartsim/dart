#include "model3d/FileInfoModel.h"
#include "model3d/FileInfoDof.h"
#include "MyWindow.h"
#include "utils/LoadOpengl.h"
#include <iostream>
using namespace std;

using namespace model3d;

int main(int argc, char* argv[])
{
    if(argc!=3){
        cout<<"viewer skel_file dof_file"<<endl;
        exit(0);
    }
	
    FileInfoModel model;
    model.loadFile(argv[1],FileInfoModel::SKEL);

    FileInfoDof motion(model.getSkel());
    motion.loadFile(argv[2]);


    MyWindow window(motion);
    window.computeMax();

    glutInit(&argc, argv);
    window.initWindow(640, 480, argv[1]);
    glutMainLoop();

    return 0;
}
