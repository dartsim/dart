#include "MyWindow.h"
#include "yui/GLFuncs.h"
#include "RigidBody.h"
#include "kinematics/Skeleton.h"

using namespace Eigen;

void MyWindow::displayTimer(int _val) {
    mWorld->simulate();
    glutPostRedisplay();
    mFrame++;
    if(mPlaying)
        glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
}

void MyWindow::draw() {
    // Draw rigid bodies
    for (int i = 0; i < mWorld->getNumRigidBodies(); i++)
        mWorld->getRigidBody(i)->draw(mRI);
    
    // Draw the blender
    mWorld->getBlender()->draw(mRI);
    // Draw the blade
    mRI->pushMatrix();
    mRI->rotate(Vector3d(0, 1, 0), mWorld->getBladeAngle());
    mWorld->getBlade()->draw(mRI);
    mRI->popMatrix();

    // Display the frame count in 2D text
    char buff[64];
    sprintf(buff,"%d",mFrame);
    std::string frame(buff);
    glDisable(GL_LIGHTING);
    glColor3f(0.0,0.0,0.0);
    yui::drawStringOnScreen(0.02f,0.02f,frame);
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
