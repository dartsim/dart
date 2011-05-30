#include "model3d/fileinfo_model.h"
#include "model3d/fileinfo_dof.h"
#include "MyWindow.h"
#include <GLUT/glut.h>

using namespace model3d;

Eigen::Vector3d gravity(0,0,0);

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
