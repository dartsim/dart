#include "MyWindow.h"

#include <stdio.h>

#include "common/Timer.h"
#include "math/Helpers.h"
#include "collision/CollisionDetector.h"
#include "constraint/ConstraintDynamics.h"
#include "constraint/PointConstraint.h"
#include "dynamics/BodyNode.h"
#include "dynamics/GenCoord.h"
#include "yui/GLFuncs.h"

using namespace std;
using namespace Eigen;

using namespace dart;
using namespace math;
using namespace dynamics;
using namespace constraint;
using namespace yui;



void MyWindow::initDyn()
{
    mDofs.resize(mSkels.size());
    mDofVels.resize(mSkels.size());

    for (unsigned int i = 0; i < mSkels.size(); i++) {
        mDofs[i].resize(mSkels[i]->getDOF());
        mDofVels[i].resize(mSkels[i]->getDOF());
        mDofs[i].setZero();
        mDofVels[i].setZero();
    }
    mDofs[0][1] = -2.9; // ground level
    // default standing pose
    mDofs[1][1] = -0.1;
    mDofs[1][6] = 0.2; // left hip
    mDofs[1][9] = -0.5; // left knee
    mDofs[1][10] = 0.3; // left ankle
    mDofs[1][13] = 0.2; // right hip
    mDofs[1][16] = -0.5; // right knee
    mDofs[1][17] = 0.3; // right ankle
    mDofs[1][21] = -0.1; // lower back
    mDofs[1][28] = 0.5; // left shoulder
    mDofs[1][34] = -0.5; // right shoulder
    
    for (unsigned int i = 0; i < mSkels.size(); i++) {
        mSkels[i]->initDynamics();
        mSkels[i]->setPose(mDofs[i], false, false);
        // compute dynamics here because computation of control force at first iteration needs to access mass matrix
        mSkels[i]->set_dq(mDofVels[i]);
        mSkels[i]->computeEquationsOfMotionID(mGravity);
    }
    mSkels[0]->setImmobileState(true);

    mController = new Controller(mSkels[1], mConstraintHandle, mTimeStep);
     
    for (int i = 0; i < mSkels[1]->getDOF(); i++)
        mController->setDesiredDof(i, mController->getSkel()->getDof(i)->get_q());

    // initialize constraint on the hand
    mConstraintHandle = new ConstraintDynamics(mSkels, mTimeStep);
    BodyNode *bd = mSkels[1]->findBodyNode("fullbody1_h_hand_left");
    PointConstraint *point1 = new PointConstraint(bd, bd->getLocalCOM(), bd->getWorldCOM(), 1);
    mConstraintHandle->addConstraint(point1);
    bd = mSkels[1]->findBodyNode("fullbody1_h_hand_right");
    PointConstraint *point2 = new PointConstraint(bd, bd->getLocalCOM(), bd->getWorldCOM(), 1);
    mConstraintHandle->addConstraint(point2);
}

void MyWindow::displayTimer(int _val)
{
    int numIter = mDisplayTimeout / (mTimeStep * 1000);
    if (mPlay) {
        mPlayFrame += 30;
        if (mPlayFrame >= mBakedStates.size())
            mPlayFrame = 0;
        glutPostRedisplay();
        glutTimerFunc(mDisplayTimeout, refreshTimer, _val);        
    }else if (mSimulating) {
        //        static Timer tSim("Simulation");
        for (int i = 0; i < numIter; i++) {
            mSkels[1]->findBodyNode("fullbody1_root")->addExtForce(Vector3d(0.0, 0.0, 0.0), mForce);
            //  tSim.startTimer();
            mController->computeTorques(mDofs[1], mDofVels[1]);
            mSkels[1]->setInternalForces(mController->getTorques());
            mIntegrator.integrate(this, mTimeStep);
            //tSim.stopTimer();
            //tSim.printScreen();
            bake();
            mSimFrame++;
        }
        // for perturbation test
        mImpulseDuration--;
        if (mImpulseDuration <= 0) {
            mImpulseDuration = 0;
            mForce.setZero();
        }
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
                mSkels[i]->setPose(mBakedStates[mPlayFrame].segment(start, size), true, false);
            }
            Vector3d com = mSkels[1]->getWorldCOM();
            mRI->setPenColor(Vector3d(0.8, 0.8, 0.2));
            mRI->pushMatrix();
            glTranslated(com[0], com[1], com[2]);
            mRI->drawEllipsoid(Vector3d(0.05, 0.05, 0.05));
            mRI->popMatrix();

            if (mShowMarkers) {
                int sumDofs = mIndices[mSkels.size()]; 
                int nContact = (mBakedStates[mPlayFrame].size() - sumDofs) / 6;
                for (int i = 0; i < nContact; i++) {
                    Vector3d v = mBakedStates[mPlayFrame].segment(sumDofs + i * 6, 3);
                    Vector3d f = mBakedStates[mPlayFrame].segment(sumDofs + i * 6 + 3, 3) / 100.0;
                    mRI->setPenColor(Vector3d(0.2, 0.2, 0.8));
                    glBegin(GL_LINES);
                    glVertex3f(v[0], v[1], v[2]);
                    glVertex3f(v[0] + f[0], v[1] + f[1], v[2] + f[2]);
                    glEnd();
                    mRI->pushMatrix();
                    glTranslated(v[0], v[1], v[2]);
                    mRI->drawEllipsoid(Vector3d(0.02, 0.02, 0.02));
                    mRI->popMatrix();
                }
            }
        }
    }else{
        if (mShowMarkers) {
            for (int k = 0; k < mConstraintHandle->getCollisionChecker()->getNumContacts(); k++) {
                Vector3d  v = mConstraintHandle->getCollisionChecker()->getContact(k).point;
                Vector3d n = mConstraintHandle->getCollisionChecker()->getContact(k).normal / 10.0;
                Vector3d f = mConstraintHandle->getCollisionChecker()->getContact(k).force / 100.0;

                mRI->setPenColor(Vector3d(0.2, 0.2, 0.8));
                glBegin(GL_LINES);
                glVertex3f(v[0], v[1], v[2]);
                glVertex3f(v[0] + f[0], v[1] + f[1], v[2] + f[2]);
                glEnd();
                mRI->pushMatrix();
                glTranslated(v[0], v[1], v[2]);
                mRI->drawEllipsoid(Vector3d(0.02, 0.02, 0.02));
                mRI->popMatrix();
            }
        }
    }

    // draw handholds
    mRI->setPenColor(Vector3d(0.2, 0.2, 0.2));
    mRI->pushMatrix();
    glTranslated(0.0, -0.06, -0.52);
    mRI->drawEllipsoid(Vector3d(0.1, 0.1, 0.1));
    mRI->popMatrix();
    mRI->setPenColor(Vector3d(0.2, 0.2, 0.2));
    mRI->pushMatrix();
    glTranslated(0.0, -0.06, 0.52);
    mRI->drawEllipsoid(Vector3d(0.1, 0.1, 0.1));
    mRI->popMatrix();

    // draw arrow
    if (mImpulseDuration > 0) {
        Vector3d poa = xformHom(mSkels[1]->findBodyNode("fullbody1_root")->getWorldTransform(), Vector3d(0.0, 0.0, 0.0));
        Vector3d start = poa - mForce / 10.0;
        double len = mForce.norm() / 10.0;
        drawArrow3D(start, mForce, len, 0.05, 0.1);
    }

    for (unsigned int i = 0; i < mSkels.size(); i++)
        mSkels[i]->draw(mRI);
        
    // display the frame count in 2D text
    char buff[64];
    if (!mSim) 
        sprintf(buff, "%d", mPlayFrame);
    else{
        sprintf(buff, "%d", mSimFrame);
    }
    string frame(buff);
    glDisable(GL_LIGHTING);
    glColor3f(0.0,0.0,0.0);
    drawStringOnScreen(0.02f,0.02f,frame);
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
    case 's': // simulate one frame
        if (!mPlay) {
            mForce = Vector3d::Zero();
            for (unsigned int i = 0; i < mSkels.size(); i++) {
                if (mSkels[i]->getImmobileState()) {
                    mSkels[i]->setPose(mDofs[i], true, false);
                } else {
                    mSkels[i]->setPose(mDofs[i], false, true);
                    mSkels[i]->set_dq(mDofVels[i]);
                    mSkels[i]->computeEquationsOfMotionID(mGravity);
                }
            }
            mConstraintHandle->computeConstraintForces();
            mController->setConstrForces(mConstraintHandle->getTotalConstraintForce(1));
            mIntegrator.integrate(this, mTimeStep);
            mSimFrame++;
            bake();
            glutPostRedisplay();
        }
        break;
    case '1':
        mForce[0] = 20;
        mImpulseDuration = 10.0;
        cout << "push forward" << endl;
        break;
    case '2':
        mForce[0] = -10;
        mImpulseDuration = 10.0;
        cout << "push backward" << endl;
        break;
    case '3':
        mForce[2] = 50;
        mImpulseDuration = 10.0;
        cout << "push right" << endl;
        break;
    case '4':
        mForce[2] = -50;
        mImpulseDuration = 10.0;
        cout << "push left" << endl;
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
    case 'v': // show or hide markers
        mShowMarkers = !mShowMarkers;
        break;

    default:
        Win3D::keyboard(key,x,y);
    }
    glutPostRedisplay();
}
