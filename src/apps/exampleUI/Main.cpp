#include "MyWin3D.h"
#include "MyWin2D.h"

int main(int argc, char* argv[])
{
	MyWin3D win3d;
	MyWin2D win2d;

	glutInit(&argc, argv);
	win2d.initWindow(640,480,"2D Window test");
	win3d.initWindow(640,480,"3D Window test");
	glutMainLoop();
	return 0;
}
