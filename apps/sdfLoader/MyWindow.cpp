#include "MyWindow.h"
#include "simulation/World.h"
#include "dynamics/Skeleton.h"

using namespace dart;
using namespace dynamics;

MyWindow::MyWindow()
    : SimWindow()
{
    mForce = Eigen::Vector3d::Zero();
}

void MyWindow::timeStepping()
{
    //static_cast<BodyNode*>(mWorld->getSkeleton(1)->getNode(0))->addExtForce(Vector3d(0.0, 0.0, 0.0), mForce);
    mWorld->step();
    //mForce /= 2.0;
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
