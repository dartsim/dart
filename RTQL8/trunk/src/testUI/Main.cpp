#include "MyWin3D.h"
#include "MyWin2D.h"
#include "fileinfo_model.h"

using namespace model3d;

Vector3d gravity(0,0,0);

int main(int argc, char* argv[])
{
	FileInfoModel modelFile;
	modelFile.loadFile("Yuting.vsk",FileInfoModel::VSK);

	MyWin3D win3d(modelFile.getSkel());
	MyWin2D win2d;

	glutInit(&argc, argv);
	win2d.initWindow(640,480,"2D Window test");
	win3d.initWindow(640,480,"3D Window test");
	glutMainLoop();
	return 0;
}
