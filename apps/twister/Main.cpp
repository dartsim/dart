#include "MyWindow.h"
#include "MyWorld.h"
#include <iostream>

int main(int argc, char* argv[]) 
{
    // Create a world
    MyWorld *world = new MyWorld();

    // Link the world to the window and start running the glut event loop
    // Create a window for rendering and UI
    glutInit(&argc, argv);
    MyWindow window;
    window.initWindow(640, 480, "Twister");
    window.setWorld(world);

    std::cout << "Alt + Left click: grab a marker" << std::endl;
    std::cout << "Alt + Right click: remove a constraint" << std::endl;
    std::cout << "'v': visualization on/off" << std::endl;
    std::cout << "Left click: rotate camera" << std::endl;
    std::cout << "Right click: pan camera" << std::endl;
    std::cout << "Shift + Left click: zoom camera" << std::endl;

    glutMainLoop();

    return 0;
}
