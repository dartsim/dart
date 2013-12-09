#include "MyWindow.h"

#include "math/Helpers.h"
#include "dynamics/BodyNode.h"
#include "dynamics/Skeleton.h"
#include "constraint/ConstraintDynamics.h"
#include "simulation/World.h"
#include "yui/GLFuncs.h"

using namespace dart;
using namespace math;
using namespace dynamics;
using namespace yui;

void MyWindow::timeStepping()
{
    mWorld->getSkeleton(1)->getBodyNode("h_spine")->addExtForce(Eigen::Vector3d(0.0, 0.0, 0.0), mForce);

    mController->setConstrForces(mWorld->getConstraintHandler()->getTotalConstraintForce(1));
    mController->computeTorques(mWorld->getSkeleton(1)->get_q(), mWorld->getSkeleton(1)->get_dq());
    mWorld->getSkeleton(1)->setInternalForceVector(mController->getTorques());

    mWorld->step();

    // for perturbation test
    mImpulseDuration--;
    if (mImpulseDuration <= 0) {
        mImpulseDuration = 0;
        mForce.setZero();
    }    
}

void MyWindow::drawSkels()
{
    for (unsigned int i = 0; i < mWorld->getNumSkeletons(); i++)
        mWorld->getSkeleton(i)->draw(mRI);

    // draw arrow
    if (mImpulseDuration > 0) {
        Eigen::Vector3d poa = mWorld->getSkeleton(1)->getBodyNode("h_spine")->getWorldTransform() * Eigen::Vector3d(0.0, 0.0, 0.0);
        Eigen::Vector3d start = poa - mForce / 10.0;
        double len = mForce.norm() / 10.0;
        drawArrow3D(start, mForce, len, 0.05, 0.1);
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
    case '1':
        mForce[0] = 20;
        mImpulseDuration = 100.0;
        std::cout << "push forward" << std::endl;
        break;
    case '2':
        mForce[0] = -10;
        mImpulseDuration = 100.0;
        std::cout << "push backward" << std::endl;
        break;
    case '3':
        mForce[2] = 50;
        mImpulseDuration = 100.0;
        std::cout << "push right" << std::endl;
        break;
    case '4':
        mForce[2] = -50;
        mImpulseDuration = 100.0;
        std::cout << "push left" << std::endl;
        break;
    default:
        Win3D::keyboard(key,x,y);

    }
    glutPostRedisplay();
}

