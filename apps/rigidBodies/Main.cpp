#include "dart/simulation/World.h"
#include "build/dart/utils/Paths.h"
#include "dart/utils/SkelParser.h"
#include "MyWindow.h"
#include <iostream>

int main(int argc, char* argv[])
{
  // Create a window for rendering and UI
  MyWindow window;

  // Create a world
  MyWorld *world = new MyWorld();
  dart::simulation::World* pinataWorld
    = dart::utils::SkelParser::readWorld(
                                         DART_DATA_PATH"skel/pinata.skel");
  world->setDartWorld(pinataWorld);
  world->initializePinata();

  // Link the world to the window and start running the glut event loop
  window.setWorld(world);
  glutInit(&argc, argv);
  window.initWindow(640, 640, "Rigid Body Pinata");
  std::cout << "space bar: simulation on/off" << std::endl;
  std::cout << "'p': playback/stop" << std::endl;
  std::cout << "'[' and ']': play one frame backward and forward" << std::endl;
  std::cout << "'1'-'4': Hit the pinata from different directions" << std::endl;
  std::cout << "'v': visualization on/off" << std::endl;
  std::cout << "Left click: rotate camera" << std::endl;
  std::cout << "Right click: pan camera" << std::endl;
  std::cout << "Shift + Left click: zoom camera" << std::endl;
    
  glutMainLoop();

  return 0;
}
