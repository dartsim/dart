#include <iostream>
#include "MyWindow.h"
#include "MyWorld.h"

int main(int argc, char* argv[])
{
  // Create a window for rendering and UI
  MyWindow window;

  // Create a world with 3 particles
  MyWorld *world = new MyWorld(3);

  // Link the world to the window and start running the glut event loop
  window.setWorld(world);
    
  std::cout << "space bar: simulation on/off" << std::endl;
  glutInit(&argc, argv);
  window.initWindow(640, 480, "Galileo's experiment"); 
  int a = 0;
  glutMainLoop();

  return 0;
}
