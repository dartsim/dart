#include "MyWindow.h"
#include "Particle.h"
#include "MyWorld.h"
#include "dart/gui/GLFuncs.h"

using namespace Eigen;

MyWindow::MyWindow() : SimWindow() {
  mBackground[0] = 1.0;
  mBackground[1] = 1.0;
  mBackground[2] = 1.0;
  mBackground[3] = 1.0;
  
  mPlaying = false;
  
  mPersp = 30.f;
  mTrans[1] = -20000.f;
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

  //Draw a checker board background
  glDisable(GL_LIGHTING);
  bool flip = true;
  double wall = -5.0;
  for(int i = -20; i < 20; i++){
    for(int j = -20; j < 25; j++){
      if(flip == true){
        glColor4d(0.7, 0.7, 0.7, 1.0);
        flip = false; 
      }else{
        glColor4d(0.8, 0.8, 0.8, 1.0);
        flip = true;
      }
      glBegin(GL_QUADS);
      glNormal3d(0, 0, 1);
      glVertex3d(i, j, wall);
      glVertex3d(i, j + 1, wall);
      glVertex3d(i + 1, j + 1, wall);
      glVertex3d(i + 1, j, wall);
      glEnd();
    }
  }
  glEnable(GL_LIGHTING);

  // Pan the camera based on the average height of all particles
  double average_height = 0.0;
  for (int i = 0; i < mWorld->getNumParticles(); i++)
    average_height += mWorld->getParticle(i)->mPosition[1];
  average_height /= mWorld->getNumParticles();
  mTrans[1] = average_height * -1000.0;

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
