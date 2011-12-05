#include "MyWindow.h"
#include "dynamics/SkeletonDynamics.h"
#include "dynamics/JointLimitDynamics.h"
#include "dynamics/BodyNodeDynamics.h"
#include "utils/UtilsMath.h"
#include "utils/Timer.h"
#include "yui/GLFuncs.h"
#include <cstdio>

using namespace std;
using namespace Eigen;
using namespace kinematics;
using namespace utils;
using namespace integration;
using namespace dynamics;

void MyWindow::initDyn()
{
    // set random initial conditions
    mDofs.resize(mModel->getNumDofs());
    mDofVels.resize(mModel->getNumDofs());
    for(unsigned int i=0; i<mModel->getNumDofs(); i++){
        mDofs[i] = utils::random(-0.6,0.6);
        mDofVels[i] = utils::random(-3.0,3.0);
    }
    mModel->setPose(mDofs,false,false);
    mModel->computeDynamics(mGravity, mDofVels, false);
    mJointLimitConstr = new JointLimitDynamics(mModel, mTimeStep);
}

VectorXd MyWindow::getState() {
    VectorXd state(mDofs.size() + mDofVels.size());
    state.head(mDofs.size()) = mDofs;
    state.tail(mDofVels.size()) = mDofVels;
    return state;
}

VectorXd MyWindow::evalDeriv() {
    VectorXd deriv(mDofs.size() + mDofVels.size());
    VectorXd qddot = mModel->getMassMatrix().fullPivHouseholderQr().solve(-mModel->getCombinedVector() + mModel->getExternalForces() + mJointLimitConstr->getConstraintForce());
    mModel->clampRotation(mDofs, mDofVels);
    deriv.tail(mDofVels.size()) = qddot; // set qddot (accelerations)
    deriv.head(mDofs.size()) = (mDofVels + (qddot * mTimeStep)); // set new velocities
    return deriv;
}

void MyWindow::setState(VectorXd newState) {
    mDofVels = newState.tail(mDofVels.size());
    mDofs = newState.head(mDofs.size());
}

void MyWindow::setPose() {
    mModel->setPose(mDofs,false,false);
    mModel->computeDynamics(mGravity, mDofVels, true);
    mJointLimitConstr->applyJointLimitTorques();
}

void MyWindow::displayTimer(int _val)
{
    int numIter = mDisplayTimeout / (mTimeStep*1000);
    for(int i = 0; i < numIter; i++){
        setPose();
        mIntegrator.integrate(this, mTimeStep);
    }

    mFrame += numIter;   
    glutPostRedisplay();
    if(mRunning)
        glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
}

void MyWindow::draw()
{
    mModel->draw(mRI);
    if(mShowMarker) mModel->drawMarkers(mRI);
    
    // display the frame count in 2D text
    char buff[64];
    sprintf(buff,"%d",mFrame);
    string frame(buff);
    glDisable(GL_LIGHTING);
    glColor3f(0.0,0.0,0.0);
    yui::drawStringOnScreen(0.02f,0.02f,frame);
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
    case 'h': // show or hide markers
        mShowMarker = !mShowMarker;
        break;
    case 's': // simulate one frame
        setPose();
        mIntegrator.integrate(this, mTimeStep);
        mFrame++;   
        glutPostRedisplay();
        break;
    case 'l': // right force
        mForce[0] = 300.0;
        static_cast<BodyNodeDynamics*>(mModel->getNode(1))->addExtForce(Vector3d(0.2, 0.0, 0), mForce);
        mForce[0] = 0.0;
        cout << "push" << endl;
        break;
    case 'k': // left force
        mForce[0] = -300.0;
        static_cast<BodyNodeDynamics*>(mModel->getNode(1))->addExtForce(Vector3d(0.2, 0.0, 0), mForce);
        mForce[0] = 0.0;
        cout << "push" << endl;
        break;
    default:
        Win3D::keyboard(key,x,y);
    }
    glutPostRedisplay();
}


