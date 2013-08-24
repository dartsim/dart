#include "MyWindow.h"
#include <iostream>

using namespace std;

int main(int argc, char* argv[])
{	
    MyWindow window;
    MyWorld *world = new MyWorld(3);
    window.setWorld(world);

    glutInit(&argc, argv);
    window.initWindow(640, 480, "Particles");
    glutMainLoop();

    return 0;
}
