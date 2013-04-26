#include <cstdio>

#include "MyWindow.h"
#include "dynamics/SkeletonDynamics.h"
#include "dynamics/ClosedLoopConstraint.h"
#include "dynamics/ConstraintDynamics.h"
#include "dynamics/BodyNodeDynamics.h"
#include "utils/UtilsMath.h"
#include "utils/Timer.h"
#include "yui/GLFuncs.h"

using namespace utils;
using namespace dynamics;

void MyWindow::initDyn()
{
    // set random initial conditions
    mDofs.resize(mSkels[0]->getNumDofs());
    mDofVels.resize(mSkels[0]->getNumDofs());
    mDofs[20] = 3.14159 * 0.4;
    mDofs[23] = 3.14159 * 0.4;
    mDofs[26] = 3.14159 * 0.4;
    mDofs[29] = 3.14159 * 0.4;
    // initial dynamic skeleton
    mSkels[0]->initDynamics();
    mSkels[0]->setPose(mDofs, false, false); // set flags to skip transformation and first-derivatives updates
    mSkels[0]->computeDynamics(mGravity, mDofVels, false);

    // initialize closed loop constraint
    mConstraintHandle = new ConstraintDynamics(mSkels, mTimeStep);
    BodyNodeDynamics *bd1 = (BodyNodeDynamics*)mSkels[0]->getNode("Chain_node5");
    BodyNodeDynamics *bd2 = (BodyNodeDynamics*)mSkels[0]->getNode("Chain_node9");
    Vector3d offset1(0, 0, 0);
    Vector3d offset2 = bd2->getLocalCOM();
    offset2[1] *= 2;
    ClosedLoopConstraint *cl = new ClosedLoopConstraint(bd1, bd2, offset1, offset2, 0, 0);
    mConstraintHandle->addConstraint(cl);
}

VectorXd MyWindow::getState() {
    VectorXd state(mDofs.size() + mDofVels.size());
    state.head(mDofs.size()) = mDofs;
    state.tail(mDofVels.size()) = mDofVels;
    return state;
}

VectorXd MyWindow::evalDeriv() {
    mSkels[0]->setPose(mDofs, false, true);
    mSkels[0]->computeDynamics(mGravity, mDofVels, true); // update equations of motion; set flag to use recursive computation
    VectorXd deriv(mDofs.size() + mDofVels.size());
    mConstraintHandle->computeConstraintForces();
    VectorXd qddot = mSkels[0]->getInvMassMatrix() * (-mSkels[0]->getCombinedVector() + mSkels[0]->getInternalForces() + mConstraintHandle->getTotalConstraintForce(0));

    mSkels[0]->clampRotation(mDofs, mDofVels);
    deriv.head(mDofs.size()) = mDofVels + mTimeStep * qddot;
    deriv.tail(mDofVels.size()) = qddot;
    return deriv;
}

void MyWindow::setState(const VectorXd& newState) {
    mDofVels = newState.tail(mDofVels.size());
    mDofs = newState.head(mDofs.size());
}

void MyWindow::displayTimer(int _val)
{
    //    static Timer tSim("Simulation");
    int numIter = mDisplayTimeout / (1000 * mTimeStep); // refresh screen every "numIter"time steps
    for (int i = 0; i < numIter; i++) {
        //        tSim.startTimer();
#ifdef DAMPING
        VectorXd damping = computeDamping();
        mSkels[0]->setInternalForces(damping);
#endif
        mIntegrator.integrate(this, mTimeStep);
        //        tSim.stopTimer();
        //        tSim.printScreen();
        mFrame++;
    }
    glutPostRedisplay();
    if (mRunning)	
        glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
}

void MyWindow::draw()
{
    mSkels[0]->draw(mRI);
    
    // display the frame count in 2D text
    char buff[64];
    sprintf(buff, "%d", mFrame);
    string frame(buff);
    glDisable(GL_LIGHTING);
    glColor3f(0.0, 0.0, 0.0);
    yui::drawStringOnScreen(0.02f, 0.02f, frame);
    glEnable(GL_LIGHTING);
}

void MyWindow::keyboard(unsigned char key, int x, int y)
{
    switch(key){
    case ' ': // use space key to play or stop the motion
        mRunning = !mRunning;
        if(mRunning)
            glutTimerFunc( mDisplayTimeout, refreshTimer, 0);
        break;
    case 'r': // reset the motion to the first frame
        mFrame = 0;
        break;
    default:
        Win3D::keyboard(key,x,y);
    }
    glutPostRedisplay();
}

#ifdef DAMPING
VectorXd MyWindow::computeDamping()
{
    VectorXd damping = VectorXd::Zero(mDofVels.size());
    // add damping to each joint; twist-dof has smaller damping
    damping = -0.01 * mDofVels;
    for (int i = 0; i < damping.size(); i++)
        if (i % 3 == 1)
            damping[i] *= 0.1;
    return damping;
}
#endif
