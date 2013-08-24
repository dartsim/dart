#include "MyWindow.h"

int main(int argc, char* argv[])
{
    // Create a window for rendering and UI
    MyWindow window;

    // Create a world for particle simulation
    MyWorld *world = new MyWorld(3);

    // Link the world to the window and start running
    window.setWorld(world);
    glutInit(&argc, argv);
    window.initWindow(640, 480, "Particles");
    glutMainLoop();

    return 0;
}
