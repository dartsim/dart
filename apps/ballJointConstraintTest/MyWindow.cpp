#include "MyWindow.h"

#include "dynamics/Skeleton.h"
#include "dynamics/BodyNode.h"
#include "simulation/World.h"
#include "constraint/ConstraintDynamics.h"
#include "constraint/BallJointConstraint.h"

using namespace dart::dynamics;
using namespace dart::constraint;

void MyWindow::timeStepping()
{
    //    Eigen::VectorXd damping = computeDamping();
    //mWorld->getSkeleton(0)->setInternalForces(damping);
    mWorld->step();
}

Eigen::VectorXd MyWindow::computeDamping()
{
    int nDof = mWorld->getSkeleton(0)->getNumGenCoords();
    Eigen::VectorXd damping = Eigen::VectorXd::Zero(nDof);
    // add damping to each joint; twist-dof has smaller damping
    damping = -0.01 * mWorld->getSkeleton(0)->get_dq();
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

    case 'h':
        if (mHeadConstraint) {
            mWorld->getConstraintHandler()->deleteConstraint(mHeadConstraint);
            mHeadConstraint = NULL;
        } else {
            mHeadConstraint = addHeadConstraint();
        }
        break;

    case 't':
        if (mTailConstraint) {
            mWorld->getConstraintHandler()->deleteConstraint(mTailConstraint);
            mTailConstraint = NULL;
        } else {
            mTailConstraint = addTailConstraint();
        }
        break;
    default:
        Win3D::keyboard(key,x,y);

    }
    glutPostRedisplay();
}

Constraint* MyWindow::addHeadConstraint() {
    BodyNode *bd = mWorld->getSkeleton(0)->getBodyNode("link 1");
    Eigen::Vector3d offset(0.0, 0.025, 0.0);
    Eigen::Vector3d target = bd->getWorldTransform() * offset;
    BallJointConstraint *bj = new BallJointConstraint(bd, offset, target);
    mWorld->getConstraintHandler()->addConstraint(bj);
    return bj;
}

Constraint* MyWindow::addTailConstraint() {
    BodyNode *bd = mWorld->getSkeleton(0)->getBodyNode("link 10");
    Eigen::Vector3d offset(0.0, -0.025, 0.0);
    Eigen::Vector3d target = bd->getWorldTransform() * offset;
    BallJointConstraint *bj = new BallJointConstraint(bd, offset, target);
    mWorld->getConstraintHandler()->addConstraint(bj);
    return bj;
}
