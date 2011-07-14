#include "MyWin3D.h"
#include "MyWin2D.h"
#include <iostream>

using namespace std;

int main(int argc, char* argv[])
{
	MyWin3D win3d;
	MyWin2D win2d;
        /*

        double coffee1 = 638.85;
        double coffee2 = 495.24;
        double coffee3 = 640.14;
        double coffee4 = 496.53;
        double coffee5 = 579.6;
        double lunch1 = 2511.6;
        double lunch2 = 2511.6;
        double dinner = 7521.92;
        double reception = 2301.26;
        double deposit = 445.72 + 574.97 + 2260.44 + 576.13 + 2260.44 + 521.64 + 446.53 + 6769.73 + 2071.13;
        double balance = 495.24 + 638.85 + 2511.60 + 640.14 + 2511.6 + 579.6 + 496.53 + 7521.92 + 2301.26;

        cout << "deposit = " << deposit << endl;
        cout << "balance = " << balance << endl;*/
	glutInit(&argc, argv);
	win2d.initWindow(640,480,"2D Window test");
	win3d.initWindow(640,480,"3D Window test");
	glutMainLoop();

	return 0;
}
