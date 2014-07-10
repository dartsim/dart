#include <iostream>
#include "MyWindow.h"
#include "MyWorld.h"

int main(int argc, char* argv[])
{
  // Create a window for rendering and UI
  MyWindow window;

  // Create a world with one particle
  MyWorld *world = new MyWorld(1);

  // Link the world to the window and start running the glut event loop
  std::cout << "space bar: simulation on/off" << std::endl;
  window.setWorld(world);
  glutInit(&argc, argv);
  window.initWindow(640, 480, "Tinkertoy");
  glutMainLoop();

  return 0;
}
