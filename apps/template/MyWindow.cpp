#include "MyWindow.h"

MyWindow::MyWindow(dart::simulation::WorldPtr world)
{
    setWorld(world);    
}

MyWindow::~MyWindow() 
{
    
}

void MyWindow::drawSkels()
{
    // Make sure lighting is turned on and that polygons get filled in
    glEnable(GL_LIGHTING);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    
    dart::gui::SimWindow::drawSkels();
}
