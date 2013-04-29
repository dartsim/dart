#include "MyWindow.h"
#include "simulation/World.h"
#include "dynamics/BodyNodeDynamics.h"

using namespace Eigen;
using namespace dynamics;

void MyWindow::timeStepping()
{
    VectorXd damping = computeDamping();    
    // add control force
    mController->computeTorques(mWorld->getSkeleton(0)->getPose(), mWorld->getSkeleton(0)->getPoseVelocity());
    mWorld->getSkeleton(0)->setInternalForces(damping + mController->getTorques());
    // add push force
    static_cast<BodyNodeDynamics*>(mWorld->getSkeleton(0)->getNode(8))->addExtForce(Vector3d(0.0, 0.0, 0.0), mForce);

    mWorld->step();
    mForce /= 2.0;
}

VectorXd MyWindow::computeDamping()
{
    int nDof = mWorld->getSkeleton(0)->getNumDofs();
    VectorXd damping = VectorXd::Zero(nDof);
    // add damping to each joint; twist-dof has smaller damping
    damping = -0.01 * mWorld->getSkeleton(0)->getPoseVelocity();
    for (int i = 0; i < nDof; i++)
        if (i % 3 == 1)
            damping[i] *= 0.1;
    return damping;
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
    default:
        Win3D::keyboard(key,x,y);
    }
    glutPostRedisplay();
}
