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
        mDofs[i][1] = 0;
    }

    mDofs[0][2] = -2;
    mDofs[1][2] = -1;
    mDofs[2][2] = 0;
    mDofs[3][2] = 1;
    mDofs[4][2] = 2;

    for (unsigned int i = 0; i < mSkels.size(); i++) {
        // mSkels[i]->initDynamics();
        mSkels[i]->setPose(mDofs[i], false, false);
        // mSkels[i]->computeDynamics(mGravity, mDofVels[i], false);
    }
    // mSkels[0]->setImmobileState(true);

    // mCollisionHandle = new dynamics::ContactDynamics(mSkels, mTimeStep);
    mSkeletonCollision = new collision_checking::SkeletonCollision();
    for (unsigned int i = 0; i < mSkels.size(); i++) {
        for (unsigned int j = 0; j < mSkels[i]->getNumNodes(); j++) {
            mSkeletonCollision->addCollisionSkeletonNode(mSkels[i]->getNode(j), false);
        }
    }
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
    setPose();
    VectorXd deriv = VectorXd::Zero(mIndices.back() * 2);    
    // for (unsigned int i = 0; i < mSkels.size(); i++) {
    //     if (mSkels[i]->getImmobileState())
    //         continue;
    //     int start = mIndices[i] * 2;
    //     int size = mDofs[i].size();
    //     VectorXd qddot = mSkels[i]->getMassMatrix().fullPivHouseholderQr().solve(-mSkels[i]->getCombinedVector() + mSkels[i]->getExternalForces() + mCollisionHandle->getConstraintForce(i));
    //     mSkels[i]->clampRotation(mDofs[i], mDofVels[i]);
    //     deriv.segment(start, size) = mDofVels[i] + (qddot * mTimeStep); // set velocities
    //     deriv.segment(start + size, size) = qddot; // set qddot (accelerations)
    // }        
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
        if (mSkels[i]->getImmobileState()) {
            mSkels[i]->setPose(mDofs[i], true, false);
        } else {
            mSkels[i]->setPose(mDofs[i], true, true);
            mSkels[i]->computeDynamics(mGravity, mDofVels[i], true);
        }
    }
    // mCollisionHandle->applyContactForces();
}

void MyWindow::displayTimer(int _val)
{

    // I think that this is where the main loop goes. This is ticked
    // using the glutTimerFunc at the end, which (through a callback
    // and a handler) calls this function again after mDisplayTimeout
    // milliseconds. I think.

    mDofs[0][2] += 0.02;
    mDofs[1][2] += 0.01;
    mDofs[2][2] += 0.00;
    mDofs[3][2] -= 0.01;
    mDofs[4][2] -= 0.02;
    for (unsigned int i = 0; i < mSkels.size(); i++) {
        mSkels[i]->setPose(mDofs[i], false, false);
    }

    mSkeletonCollision->clearAllContacts();
    mSkeletonCollision->checkCollision();

    std::cout << "Collision checking ran! Found " << mSkeletonCollision->getNumContact() << " collisions." << std::endl;
    
    glutPostRedisplay();
    glutTimerFunc(0, refreshTimer, _val + 1);
}

void MyWindow::draw()
{
    glDisable(GL_LIGHTING);

    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    for (unsigned int i = 0; i < mSkels.size(); i++) {
        int start = mIndices[i];
        int size = mDofs[i].size();
        // mSkels[i]->setPose(mBakedStates[mPlayFrame].segment(start, size), false, false);
    }

    for (unsigned int i = 0; i < mSkels.size(); i++)
        mSkels[i]->draw(mRI);
        
    // // display the frame count in 2D text
    // char buff[64];
    // if (!mSim) 
    //     sprintf(buff, "%d", mPlayFrame);
    // else
    //     sprintf(buff, "%d", mSimFrame);
    // string frame(buff);
    // glDisable(GL_LIGHTING);
    // glColor3f(0.0,0.0,0.0);
    // yui::drawStringOnScreen(0.02f,0.02f,frame);
    // glEnable(GL_LIGHTING);
}

void MyWindow::keyboard(unsigned char key, int x, int y)
{
    switch(key){
    case ' ': // use space key to play or stop the motion
        glutTimerFunc(0, refreshTimer, 0);
        break;
    case 's': // simulate one frame
        // if (!mPlay) {
        //     mForce = Vector3d::Zero();
        //     setPose();
        //     mIntegrator.integrate(this, mTimeStep);
        //     mSimFrame++;
        //     bake();
        //     glutPostRedisplay();
        // }
        break;
    case '1': // upper right force
        break;
    case '2': // upper right force
        break;
    case 'p': // playBack
        break;
    case '[': // step backward
        break;
    case ']': // step forwardward
        // if (!mSim) {
        //     mPlayFrame++;
        //     if(mPlayFrame >= mBakedStates.size())
        //         mPlayFrame = 0;
        //     glutPostRedisplay();
        // }
        break;
    case 'v': // show or hide markers
        break;
    default:
        Win3D::keyboard(key,x,y);
    }
    glutPostRedisplay();
}
