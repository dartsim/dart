#include "MyWindow.h"
#include "dynamics/SkeletonDynamics.h"
#include "utils/UtilsMath.h"
#include "utils/Timer.h"
#include "yui/GLFuncs.h"

using namespace utils;

void MyWindow::initDyn()
{
    // set random initial conditions
    mDofs.resize(mModel->getNumDofs());
    mDofVels.resize(mModel->getNumDofs());
    for(unsigned int i = 0; i < mModel->getNumDofs(); i++){
        mDofs[i] = random(-0.5, 0.5);
        mDofVels[i] = random(-0.1, 0.1);
    }
    // initial dynamic skeleton
    mModel->initDynamics();
    mModel->setPose(mDofs, false, false); // set flags to skip transformation and first-derivatives updates
}

VectorXd MyWindow::getState() {
    VectorXd state(mDofs.size() + mDofVels.size());
    state.head(mDofs.size()) = mDofs;
    state.tail(mDofVels.size()) = mDofVels;
    return state;
}

VectorXd MyWindow::evalDeriv() {
    mModel->setPose(mDofs, false, false);
    mModel->computeDynamics(mGravity, mDofVels, true); // update equations of motion; set flag to use recursive computation
    VectorXd deriv(mDofs.size() + mDofVels.size());        
    VectorXd qddot = mModel->getInvMassMatrix() * (-mModel->getCombinedVector() + mModel->getInternalForces());
    mModel->clampRotation(mDofs, mDofVels);
    deriv.head(mDofs.size()) = mDofVels + mTimeStep * qddot;
    deriv.tail(mDofVels.size()) = qddot;
    return deriv;
}

void MyWindow::setState(VectorXd newState) {
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
        mModel->setInternalForces(damping);
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
    mModel->draw(mRI);
    
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
