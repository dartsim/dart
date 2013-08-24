#include "MyWindow.h"
#include "yui/GLFuncs.h"
#include "Particle.h"

using namespace Eigen;

void MyWindow::displayTimer(int _val)
{
    mWorld->simulate();
    glutPostRedisplay();
    mFrame++;
    if(mPlaying)
        glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
}

void MyWindow::draw()
{
    // draw particles
    for (int i = 0; i < mWorld->getNumParticles(); i++)
        mWorld->getParticle(i)->draw(mRI);

    // draw background
  //draw ground
    glDisable(GL_LIGHTING);
    bool flip = true;
    double groundZero = -5.0;
    for(int i = -20; i < 20; i++){
        for(int j = -20; j < 25; j++){
            if(flip == true){
                glColor4d(0.42, 0.42, 0.42, 1.0);
                flip = false; 
            }else{
                glColor4d(0.5, 0.5, 0.5, 1.0);
                flip = true;
            }
            glBegin(GL_QUADS);
            glNormal3d(0, 0, 1);
            glVertex3d(i, j, groundZero);
            glVertex3d(i, j + 1, groundZero);
            glVertex3d(i + 1, j + 1, groundZero);
            glVertex3d(i + 1, j, groundZero);
            glEnd();
        }
    }
    glEnable(GL_LIGHTING);

    // auto-pan camera
    double average_height = 0.0;
    for (int i = 0; i < mWorld->getNumParticles(); i++)
        average_height += mWorld->getParticle(i)->mPosition[1];
    average_height /= mWorld->getNumParticles();
    mTrans[1] = average_height * -1000.0;

    // display the frame count in 2D text
    char buff[64];
    sprintf(buff,"%d",mFrame);
    std::string frame(buff);
    glDisable(GL_LIGHTING);
    glColor3f(0.0,0.0,0.0);
    yui::drawStringOnScreen(0.02f,0.02f,frame);
    glEnable(GL_LIGHTING);
}

void MyWindow::keyboard(unsigned char key, int x, int y)
{
    switch(key){
    case ' ': // use space key to play or stop the motion
        mPlaying = !mPlaying;
        if(mPlaying)
            glutTimerFunc( mDisplayTimeout, refreshTimer, 0);
        break;
    default:
        Win3D::keyboard(key,x,y);
    }
    glutPostRedisplay();
}
