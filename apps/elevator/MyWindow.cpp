#include "MyWindow.h"
#include "simulation/World.h"
#include "dynamics/ConstraintDynamics.h"
#include <iostream>

using namespace Eigen;

void MyWindow::timeStepping()
{
    //    mController->computeTorques(mWorld->getSkeleton(0)->get_q(), mWorld->getSkeleton(0)->get_dq(), mWorld->getCollisionHandle()->getTotalConstraintForce(0));
    //    mWorld->getSkeleton(0)->setInternalForces(mController->getTorques());
    mWorld->step();
}

void MyWindow::drawSkels()
{
    // moving camera
    mTrans[1] =  1000 + mWorld->getSkeleton(1)->getWorldCOM()[1] * -1000;
    if (mTrans[1] > 9000)
        mTrans[1] = 9000;
    glEnable(GL_LIGHTING);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    mWorld->getSkeleton(0)->draw(mRI);
    Vector4d color;
    color << 0.9, 0.9, 0.9, 1.0;
    mWorld->getSkeleton(2)->draw(mRI, color, false);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    mWorld->getSkeleton(1)->draw(mRI);
}

void MyWindow::keyboard(unsigned char key, int x, int y)
{
    switch(key){
    case ' ': // use space key to play or stop the motion
        mSimulating = !mSimulating;
        if(mSimulating) {
            mPlay = false;
            glutTimerFunc( mDisplayTimeout, refreshTimer, 0);
        }
        break;
    case 'p': // playBack
        mPlay = !mPlay;
        if (mPlay) {
            mSimulating = false;
            glutTimerFunc( mDisplayTimeout, refreshTimer, 0);
        }
        break;
    case '[': // step backward
        if (!mSimulating) {
            mPlayFrame--;
            if(mPlayFrame < 0)
                mPlayFrame = 0;
            glutPostRedisplay();
        }
        break;
    case ']': // step forwardward
        if (!mSimulating) {
            mPlayFrame++;
            if(mPlayFrame >= mBakedStates.size())
                mPlayFrame = 0;
            glutPostRedisplay();
        }
        break;
    case 'v': // show or hide markers
        mShowMarkers = !mShowMarkers;
        break;
    default:
        Win3D::keyboard(key,x,y);

    }
    glutPostRedisplay();
}
