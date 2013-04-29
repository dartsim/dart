#include "MyWindow.h"
#include "simulation/World.h"
#include "dynamics/BodyNodeDynamics.h"
#include "yui/GLFuncs.h"

using namespace Eigen;
using namespace dynamics;
using namespace utils;
using namespace yui;


void MyWindow::timeStepping()
{
    VectorXd damping = computeDamping();
    // add control force
    mController->computeTorques(mWorld->getSkeleton(2)->getPose(), mWorld->getSkeleton(2)->getPoseVelocity());
    mWorld->getSkeleton(2)->setInternalForces(damping + mController->getTorques());
    // add push force
    static_cast<BodyNodeDynamics*>(mWorld->getSkeleton(2)->getNode(2))->addExtForce(Vector3d(0.0, 0.0, 0.0), mForce);

    mWorld->step();
    mForce /= 2.0;
}

VectorXd MyWindow::computeDamping()
{
    int nDof = mWorld->getSkeleton(2)->getNumDofs();
    VectorXd damping = VectorXd::Zero(nDof);
    // add damping to each joint; twist-dof has smaller damping
    damping = -0.01 * mWorld->getSkeleton(2)->getPoseVelocity();
    for (int i = 0; i < nDof; i++)
        if (i % 3 == 1)
            damping[i] *= 0.1;
    return damping;
}

void MyWindow::drawSkels()
{
    for (unsigned int i = 0; i < mWorld->getNumSkeletons(); i++)
        mWorld->getSkeleton(i)->draw(mRI);

    // draw arrow
    if (mForce.norm() > 1.0) {
        Vector3d poa = xformHom(mWorld->getSkeleton(2)->getNode(2)->getWorldTransform(), Vector3d(0.0, 0.0, 0.0));
        Vector3d start = poa - mForce / 10.0;
        double len = mForce.norm() / 10.0;
        drawArrow3D(start, mForce, len, 0.025, 0.05);
    }
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
        mForce[0] = -200;
        break;
    case '2': // upper right force
        mForce[0] = 200;
        break;
    case '3': // upper right force
        mForce[2] = -200;
        break;
    case '4': // upper right force
        mForce[2] = 200;
        break;
    default:
        Win3D::keyboard(key,x,y);
    }
    glutPostRedisplay();
}
