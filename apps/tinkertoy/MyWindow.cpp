#include <iostream>
#include "MyWindow.h"
#include "MyWorld.h"
#include "Particle.h"
#include "dart/gui/GLFuncs.h"

using namespace Eigen;

MyWindow::MyWindow() : SimWindow() {
  mBackground[0] = 1.0;
  mBackground[1] = 1.0;
  mBackground[2] = 1.0;
  mBackground[3] = 1.0;
		
  mPlaying = false;
            
  mPersp = 30.f;
  mTrans[1] = 0.f;
  mTrans[2] = -500;
  mFrame = 0;
  mDisplayTimeout = 5;
}

MyWindow::~MyWindow() {
}

void MyWindow::displayTimer(int _val) {
  mWorld->simulate();
  glutPostRedisplay();
  mFrame++;
  if(mPlaying)
    glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
}

void MyWindow::draw() {
  // Draw particles
  for (int i = 0; i < mWorld->getNumParticles(); i++)
    mWorld->getParticle(i)->draw(mRI);

  glDisable(GL_LIGHTING);
  mRI->setPenColor(Vector4d(0.3, 0.3, 0.3, 1.0));
  mRI->pushMatrix();
  // Draw a circle
  glBegin(GL_LINE_LOOP);
  double rad = 3.14 / 180.0;
  double radius = 0.2;
  for (int i = 0; i < 360; i++) {
    double angle = i * rad;
    glVertex3d(radius * cos(angle), radius * sin(angle), 0.0);
  }
  glEnd();
  // Draw a line
  glBegin(GL_LINES);
  Vector3d p1 = mWorld->getParticle(0)->mPosition;
  Vector3d p2 = mWorld->getParticle(1)->mPosition;
  glVertex3f(p1[0], p1[1], p1[2]);
  glVertex3f(p2[0], p2[1], p2[2]);
  glEnd();
  mRI->popMatrix();
  glEnable(GL_LIGHTING);

  // Display the frame count in 2D text
  char buff[64];
  sprintf(buff,"%d",mFrame);
  std::string frame(buff);
  glDisable(GL_LIGHTING);
  glColor3f(0.0,0.0,0.0);
  dart::gui::drawStringOnScreen(0.02f,0.02f,frame);
  glEnable(GL_LIGHTING);
}

void MyWindow::keyboard(unsigned char key, int x, int y) {
  switch(key){
  case ' ': // Use space key to play or stop the motion
    mPlaying = !mPlaying;
    if(mPlaying)
      glutTimerFunc( mDisplayTimeout, refreshTimer, 0);
    break;
  default:
    Win3D::keyboard(key,x,y);
  }
  glutPostRedisplay();
}

void MyWindow::click(int button, int state, int x, int y) {
  mMouseDown = !mMouseDown;
  if(mMouseDown){
    if (button == GLUT_LEFT_BUTTON)
      std::cout << "Left Click" << std::endl;
    else if (button == GLUT_RIGHT_BUTTON || button == GLUT_MIDDLE_BUTTON)
      std::cout << "RIGHT Click" << std::endl;
        
    mMouseX = x;
    mMouseY = y;
  }
  glutPostRedisplay();
}

void MyWindow::drag(int x, int y) {
  double deltaX = x - mMouseX;
  double deltaY = y - mMouseY;

  mMouseX = x;
  mMouseY = y;
  std::cout << "Drag by (" << deltaX << ", " << deltaY << ")" << std::endl;

  glutPostRedisplay();
}
