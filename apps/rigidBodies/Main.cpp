#include "MyWindow.h"

int main(int argc, char* argv[])
{
    // Create a window for rendering and UI
    MyWindow window;

    // Create a world
    MyWorld *world = new MyWorld();

    // Link the world to the window and start running the glut event loop
    window.setWorld(world);
    glutInit(&argc, argv);
    window.initWindow(640, 480, "Rigid Body Smoothie");
    glutMainLoop();

    return 0;
}
