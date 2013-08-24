#include "MyWindow.h"
#include "yui/GLFuncs.h"
#include <cstdio>

using namespace std;
using namespace Eigen;

void MyWindow::displayTimer(int _val)
{
    mFrame = 0;
    glutPostRedisplay();
    if(mPlaying)	
        glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
}

void MyWindow::draw()
{
    for (int i = 0; i < mWorld->getNumParticles(); i++)
        mWorld->getParticle(i)->draw(mRI);

    // display the frame count in 2D text
    char buff[64];
    sprintf(buff,"%d",mFrame);
    string frame(buff);
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
