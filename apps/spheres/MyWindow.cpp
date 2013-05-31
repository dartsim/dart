#include "MyWindow.h"
#include "simulation/World.h"
#include "dynamics/BodyNodeDynamics.h"

using namespace Eigen;
using namespace dynamics;


void MyWindow::timeStepping()
{
    static_cast<BodyNodeDynamics*>(mWorld->getSkeleton(1)->getNode(0))->addExtForce(Vector3d(0.0, 0.0, 0.0), mForce);
    mWorld->step();
    mForce /= 2.0;
}

void MyWindow::drawSkels()
{
    glEnable(GL_LIGHTING);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    Vector4d color;
    color << 0.95, 0.95, 0.95, 1.0;
    mWorld->getSkeleton(0)->draw(mRI, color, false);
    color << 0.8, 0.3, 0.3, 1.0;
    mWorld->getSkeleton(1)->draw(mRI, color, false);
    color << 0.3, 0.8, 0.3, 1.0;
    mWorld->getSkeleton(2)->draw(mRI, color, false);
//    color << 0.8, 0.8, 0.4, 1.0;
//    mWorld->getSkeleton(3)->draw(mRI, color, false);
//    color << 0.8, 0.5, 0.3, 1.0;
//    mWorld->getSkeleton(4)->draw(mRI, color, false);
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
    case '1': // upper right force
        mForce[0] = -500;
        break;
    case '2': // upper right force
        mForce[0] = 500;
        break;
    case '3': // upper right force
        mForce[2] = -500;
        break;
    case '4': // upper right force
        mForce[2] = 500;
        break;
    default:
        Win3D::keyboard(key,x,y);

    }
    glutPostRedisplay();
}

