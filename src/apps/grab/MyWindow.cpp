#include "MyWindow.h"
#include "dynamics/BodyNodeDynamics.h"
#include "dynamics/ContactDynamics.h"
#include "kinematics/Dof.h"
#include "utils/UtilsMath.h"
#include "utils/Timer.h"
#include "yui/GLFuncs.h"
#include <cstdio>
#include "kinematics/BodyNode.h"

using namespace Eigen;
using namespace kinematics;
using namespace utils;
using namespace integration;
using namespace dynamics;

void MyWindow::initDyn()
{

    mDofs.resize(mSkels.size());
    mDofVels.resize(mSkels.size());

    for (unsigned int i = 0; i < mSkels.size(); i++) {
        mDofs[i].resize(mSkels[i]->getNumDofs());
        mDofVels[i].resize(mSkels[i]->getNumDofs());
        mDofs[i].setZero();
        mDofVels[i].setZero();
    }

    mDofs[0][1] = -0.35;
    mDofs[1][0] = 0.1;
    mDofs[1][1] = -0.324;
    mDofs[2][1] = -0.2;
    mDofs[2][6] = -0.35;
    mDofs[2][8] = -0.5;
    mDofs[2][9] = -0.35;
    mDofs[2][10] = -0.38;

    for (unsigned int i = 0; i < mSkels.size(); i++) {
        mSkels[i]->setPose(mDofs[i], false, false);
        mSkels[i]->computeDynamics(mGravity, mDofVels[i], false);
    }
    mSkels[0]->setKinematicState(true);
    mSkels[2]->setKinematicState(true);

    mCollisionHandle = new dynamics::ContactDynamics(mSkels, mTimeStep);
}

VectorXd MyWindow::getState() {
    VectorXd state(mIndices.back() * 2);    
    for (unsigned int i = 0; i < mSkels.size(); i++) {
        int start = mIndices[i] * 2;
        int size = mDofs[i].size();
        state.segment(start, size) = mDofs[i];
        state.segment(start + size, size) = mDofVels[i];
    }
    return state;
}

VectorXd MyWindow::evalDeriv() {
    VectorXd deriv = VectorXd::Zero(mIndices.back() * 2);    
    for (unsigned int i = 0; i < mSkels.size(); i++) {
        if (mSkels[i]->getKinematicState())
            continue;
        int start = mIndices[i] * 2;
        int size = mDofs[i].size();
        VectorXd qddot = mSkels[i]->getMassMatrix().fullPivHouseholderQr().solve(-mSkels[i]->getCombinedVector() + mSkels[i]->getExternalForces() + mCollisionHandle->getConstraintForce(i)); 
        mSkels[i]->clampRotation(mDofs[i], mDofVels[i]);
        deriv.segment(start, size) = mDofVels[i] + (qddot * mTimeStep); // set velocities
        deriv.segment(start + size, size) = qddot; // set qddot (accelerations)
    }        
    return deriv;
}

void MyWindow::setState(VectorXd newState) {
    for (unsigned int i = 0; i < mSkels.size(); i++) {
        int start = mIndices[i] * 2;
        int size = mDofs[i].size();
        mDofs[i] = newState.segment(start, size);
        mDofVels[i] = newState.segment(start + size, size);
    }
}

void MyWindow::setPose() {
    for (unsigned int i = 0; i < mSkels.size(); i++) {
        if (mSkels[i]->getKinematicState()) {
            mSkels[i]->setPose(mDofs[i], true, false);
        } else {
            mSkels[i]->setPose(mDofs[i], false, false);
            mSkels[i]->computeDynamics(mGravity, mDofVels[i], true);    
        }
    }
    mCollisionHandle->applyContactForces();
}

void MyWindow::displayTimer(int _val)
{
    if (mPlayBack) {        
        if (mCurrFrame < mBakedStates.size()) {
            for (unsigned int i = 0; i < mSkels.size(); i++) {
                int start = mIndices[i];
                int size = mDofs[i].size();
                mSkels[i]->setPose(mBakedStates[mCurrFrame].segment(start, size), false, false);
            }
            mCurrFrame++;
            glutPostRedisplay();
        }else{
            mCurrFrame = 0;
        }        
        glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
        return;
    }

    //    static Timer tSim("Simulation");
    int numIter = mDisplayTimeout / (mTimeStep*1000);
    for (int i = 0; i < numIter; i++) {
        //        tSim.startTimer();
        static_cast<BodyNodeDynamics*>(mSkels[1]->getNode(0))->addExtForce(Vector3d(0.0, -0.1, 0.0), mForce);
        setPose();
        mIntegrator.integrate(this, mTimeStep);
        //        tSim.stopTimer();
    }
    //    tSim.printScreen();

    bake();
    mForce = Vector3d::Zero();
    mFrame += numIter;

    if(mFrame > numIter * 25 && mFrame < numIter * 40) {    
        mDofs[2][0] += 0.004;
        mDofs[2][1] -= 0.005;
    }
    if (mFrame == numIter * 40) {        
        mSkels[1]->setKinematicState(true);
        mCollisionHandle->reset();
        mDofs[2][9] -= 0.4;
        mDofs[2][10] -= 0.3;
        mDofs[2][11] -= 0.4;
    }
    if(mFrame > numIter * 40 && mFrame < numIter * 90) {    
        mDofs[1][0] -= 0.01;
        mDofs[1][1] += 0.008;
        mDofs[2][0] -= 0.01;
        mDofs[2][1] += 0.008;
    }
    if(mFrame > numIter * 90 && mFrame < numIter * 118) {    
        mDofs[1][0] += 0.015;
        mDofs[1][1] -= 0.008;
        mDofs[1][4] -= 0.009;
        mDofs[2][0] += 0.015;
        mDofs[2][1] -= 0.008;
        mDofs[2][4] -= 0.009;
    }
    if (mFrame == numIter * 118) {
        mDofVels[1][0] = 0.015 / (numIter * mTimeStep);
        mDofVels[1][1] = -0.008 / (numIter * mTimeStep);
        mDofVels[1][4] = -0.009 / (numIter * mTimeStep);
        mSkels[1]->setKinematicState(false);
        mCollisionHandle->reset();
        mDofs[2][9] += 0.4;
        mDofs[2][10] += 0.3;
        mDofs[2][11] += 0.4;
    }

    glutPostRedisplay();
   
    if (mRunning)
        glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
}

void MyWindow::draw()
{
    glDisable(GL_LIGHTING);

    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    for (unsigned int i = 0; i < mSkels.size(); i++)
        mSkels[i]->draw(mRI);

    if (!mPlayBack) {
        glBegin(GL_LINES);
        VectorXd f = mCollisionHandle->getConstraintForce(1);
        for (int k = 0; k < mCollisionHandle->getCollisionChecker()->getNumContact(); k++) {
            Vector3d  v = mCollisionHandle->getCollisionChecker()->getContact(k).point;
            Vector3d n = mCollisionHandle->getCollisionChecker()->getContact(k).normal;

            glVertex3f(v[0], v[1], v[2]);
            glVertex3f(v[0] + n[0], v[1] + n[1], v[2] + n[2]);
        }
        glEnd();

        mRI->setPenColor(Vector3d(0.2, 0.2, 0.8));
        for (int k = 0; k < mCollisionHandle->getCollisionChecker()->getNumContact(); k++) {
            Vector3d  v = mCollisionHandle->getCollisionChecker()->getContact(k).point;
            mRI->pushMatrix();
            glTranslated(v[0], v[1], v[2]);
            mRI->drawEllipsoid(Vector3d(0.02, 0.02, 0.02));
            mRI->popMatrix();
        }
    }
        
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
        if(mRunning){
            mPlayBack = false;
            glutTimerFunc( mDisplayTimeout, refreshTimer, 0);
        }
        break;
    case 'r': // reset the motion to the first frame
        mFrame = 0;
        break;
    case 'h': // show or hide markers
        mShowMarker = !mShowMarker;
        break;
    case 's': // simulate one frame
        mForce = Vector3d::Zero();
        setPose();
        mIntegrator.integrate(this, mTimeStep);
        mFrame++;   
        glutPostRedisplay();
        break;
    case 'w': // upper right force
        //mForce[0] = 300.0;        
        mForce[0] = 0.4;
        mForce[1] = 0.8;
        cout << "push up and right" << endl;
        break;
    case 'q': // upper right force
        //mForce[0] = 300.0;        
        mForce[0] = -0.4;
        cout << "push left" << endl;
        break;
    case 'p': // playBack
        mPlayBack = !mPlayBack;
        if(mPlayBack){
            mRunning = false;
            glutTimerFunc( mDisplayTimeout, refreshTimer, 0);
        }
        break;
    default:
        Win3D::keyboard(key,x,y);
    }
    glutPostRedisplay();
}

void MyWindow::bake()
{
    VectorXd state(mIndices.back());
    for (unsigned int i = 0; i < mSkels.size(); i++)
        state.segment(mIndices[i], mDofs[i].size()) = mDofs[i];        
    mBakedStates.push_back(state);
}
