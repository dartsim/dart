#include "MyWindow.h"
#include <iostream>

int main(int argc, char* argv[])
{
    // Create a window for rendering and UI
    MyWindow window;

    // Create a grid with 64 by 64 cells
    MyWorld *world = new MyWorld(64, 0.1, 0.0, 0.0);

    // Link the world to the window and start running the glut event loop
    window.setWorld(world);
    glutInit(&argc, argv);
    window.initWindow(640, 640, "Fluid");

    std::cout << "Left click+drag: add density" << std::endl;
    std::cout << "Right click+drag: add velocity" << std::endl;
    std::cout << "'v': visualization density/velocity" << std::endl;
    std::cout << "space bar: simulation on/off" << std::endl;

    glutMainLoop();
    return 0;
}
