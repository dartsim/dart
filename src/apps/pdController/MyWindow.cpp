#include "MyWindow.h"
#include "dynamics/BodyNodeDynamics.h"
#include "kinematics/Dof.h"
#include "utils/UtilsMath.h"
#include "utils/Timer.h"
#include "yui/GLFuncs.h"
#include "Controller.h"
#include <stdio.h>

using namespace dynamics;
using namespace utils;

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

    mDofs[0] = mMotion->getPoseAtFrame(0);

    for (unsigned int i = 0; i < mSkels.size(); i++) {
        mSkels[i]->initDynamics();
        mSkels[i]->setPose(mDofs[i], false, false);
        // compute dynamics here because computation of control force at first iteration needs to access mass matrix
        mSkels[i]->computeDynamics(mGravity, mDofVels[i], false);
    }
    
    // init controller
    int nDof = mSkels[0]->getNumDofs();
    mController = new Controller(mMotion, mSkels[0], mTimeStep);
    for (int i = 0; i < nDof; i++)
        mController->setDesiredDof(i, mController->getSkel()->getDof(i)->getValue());
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
    for (unsigned int i = 0; i < mSkels.size(); i++) {
        if (mSkels[i]->getImmobileState()) {
            mSkels[i]->setPose(mDofs[i], false, false);
        } else {
            mSkels[i]->setPose(mDofs[i], false, false);
            mSkels[i]->computeDynamics(mGravity, mDofVels[i], true);
        }
    }
    VectorXd deriv = VectorXd::Zero(mIndices.back() * 2);    
    for (unsigned int i = 0; i < mSkels.size(); i++) {
        // skip immobile objects in forward simulation
        if (mSkels[i]->getImmobileState())
            continue;
        int start = mIndices[i] * 2;
        int size = mDofs[i].size();
        VectorXd qddot = mSkels[i]->getInvMassMatrix() * (-mSkels[i]->getCombinedVector() + mSkels[i]->getExternalForces() + mSkels[i]->getInternalForces());
        mSkels[i]->clampRotation(mDofs[i], mDofVels[i]);
        deriv.segment(start, size) = mDofVels[i] + (qddot * mTimeStep); // set velocities
        deriv.segment(start + size, size) = qddot; // set qddot (accelerations)
    }
    return deriv;
}

void MyWindow::setState(const VectorXd &newState) {
    for (unsigned int i = 0; i < mSkels.size(); i++) {
        int start = mIndices[i] * 2;
        int size = mDofs[i].size();
        mDofs[i] = newState.segment(start, size);
        mDofVels[i] = newState.segment(start + size, size);
    }
}

void MyWindow::displayTimer(int _val)
{
    int numIter = mDisplayTimeout / (mTimeStep * 1000);
    if (mPlay) {
        mPlayFrame += 16;
        if (mPlayFrame >= mBakedStates.size())
            mPlayFrame = 0;
        glutPostRedisplay();
        glutTimerFunc(mDisplayTimeout, refreshTimer, _val);      
    }else if (mSim) {
        //        static Timer tSim("Simulation");
        for (int i = 0; i < numIter; i++) {
            //            tSim.startTimer();
            static_cast<BodyNodeDynamics*>(mSkels[0]->getNode("fixedHand_indexDIP"))->addExtForce(Vector3d(0.02, 0.0, 0), mForce);
            mController->computeTorques(mDofs[0], mDofVels[0]);
            mSkels[0]->setInternalForces(mController->getTorques());
            mIntegrator.integrate(this, mTimeStep);
            //            tSim.stopTimer();
            //tSim.printScreen();
            bake();
            mSimFrame++;
        }
        mForce.setZero();

        glutPostRedisplay();
        glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
    }
}

void MyWindow::draw()
{
    glDisable(GL_LIGHTING);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    if (!mSim) {
        if (mPlayFrame < mBakedStates.size()) {
            for (unsigned int i = 0; i < mSkels.size(); i++) {
                int start = mIndices[i];
                int size = mDofs[i].size();
                mSkels[i]->setPose(mBakedStates[mPlayFrame].segment(start, size), false, false);
            }
        }
    }
    
    for (unsigned int i = 0; i < mSkels.size(); i++)
        mSkels[i]->draw(mRI);
            
    // display the frame count in 2D text
    char buff[64];
    if (!mSim) 
        sprintf(buff, "%d", mPlayFrame);
    else
        sprintf(buff, "%d", mSimFrame);
    string frame(buff);
    glColor3f(0.0,0.0,0.0);
    yui::drawStringOnScreen(0.02f,0.02f,frame);
    glEnable(GL_LIGHTING);
}

void MyWindow::keyboard(unsigned char key, int x, int y)
{
    switch(key){
    case ' ': // use space key to play or stop the motion
        mSim = !mSim;
        if (mSim) {
            mPlay = false;
            glutTimerFunc( mDisplayTimeout, refreshTimer, 0);
        }
        break;
    case '1':
        mForce[1] = 50;
        cout << "push" << endl;
        break;
    case '2':
        mForce[1] = -50;
        cout << "push" << endl;
        break;
    case 'p': // playBack
        mPlay = !mPlay;
        if (mPlay) {
            mSim = false;
            glutTimerFunc( mDisplayTimeout, refreshTimer, 0);
        }
        break;
    case '[': // step backward
        if (!mSim) {
            mPlayFrame--;
            if(mPlayFrame < 0)
                mPlayFrame = 0;
            glutPostRedisplay();
        }
        break;
    case ']': // step forwardward
        if (!mSim) {
            mPlayFrame++;
            if(mPlayFrame >= mBakedStates.size())
                mPlayFrame = 0;
            glutPostRedisplay();
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
