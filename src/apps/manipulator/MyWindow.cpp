#include "MyWindow.h"
#include "dynamics/BodyNodeDynamics.h"
#include "dynamics/ContactDynamics.h"
#include "kinematics/Dof.h"
#include "collision/CollisionSkeleton.h"
#include "utils/UtilsMath.h"
#include "utils/Timer.h"
#include "yui/GLFuncs.h"
#include "Controller.h"

using namespace dynamics;
using namespace utils;

static double initPose[] = {0.0, 3.14, 0.0, 0.0, -0.868398, -0.256271, -0.794892, -0.129097, 0.147894, -0.690374, -0.605116, -0.148172, -0.243184, -0.220659, -0.4107145, -0.0341097, -0.64186, -0.040947, -0.3300072, 0.014561, -0.570674, -0.0651653, -0.369658, -0.140586, -0.643183, -0.0800537};

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

    // initial pose for hand
    for (int i = 0; i < mSkels[1]->getNumDofs(); i++)
        mDofs[1][i] = initPose[i];
    // initial position of the box
    mDofs[2][0] = 0.17;
    mDofs[2][1] = mDofs[0][1] + 0.25;

    for (unsigned int i = 0; i < mSkels.size(); i++) {
        mSkels[i]->initDynamics();
        mSkels[i]->setPose(mDofs[i], false, false);
         // compute dynamics here because computation of control force at first iteration needs to access mass matrix
        mSkels[i]->computeDynamics(mGravity, mDofVels[i], false);
   }

    // set the ground to be an immobile object; it will still participate in collision
    mSkels[0]->setImmobileState(true);
    // create a collision handler
    mCollisionHandle = new dynamics::ContactDynamics(mSkels, mTimeStep);

    // create controller
    mController = new Controller(mSkels[1], mTimeStep);

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
    for (unsigned int i = 0; i < mSkels.size(); i++) {
        // skip immobile objects in forward simulation
        if (mSkels[i]->getImmobileState())
            continue;
        int start = mIndices[i] * 2;
        int size = mDofs[i].size();
        VectorXd qddot = mSkels[i]->getInvMassMatrix() * (-mSkels[i]->getCombinedVector() + mSkels[i]->getExternalForces() + mSkels[i]->getInternalForces() + mCollisionHandle->getConstraintForce(i));
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
        if (mSkels[i]->getImmobileState()) {
            // need to update node transformation for collision
            mSkels[i]->setPose(mDofs[i], true, false);
        } else {
            // need to update first derivatives for collision
            mSkels[i]->setPose(mDofs[i], false, true);
            mSkels[i]->computeDynamics(mGravity, mDofVels[i], true);
        }
    }
    // compute contact forces
    mCollisionHandle->applyContactForces();
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
            if (mSimFrame > 1000 && mSimFrame < 1033) {
                cout << "push on the ball" << endl;
                mForce[0] = 60;
            } else {
                mForce[0] = 0.0;
            }
            //            tSim.startTimer();
            static_cast<BodyNodeDynamics*>(mSkels[2]->getNode("sphere_root"))->addExtForce(Vector3d(0.0, 0.0, 0), mForce);        
            mController->computeTorques(mDofs[1], mDofVels[1], mDofVels[2]);
            mSkels[1]->setInternalForces(mController->getTorques());
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
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    if (!mSim) {
        if (mPlayFrame < mBakedStates.size()) {
            for (unsigned int i = 0; i < mSkels.size(); i++) {
                int start = mIndices[i];
                int size = mDofs[i].size();
                mSkels[i]->setPose(mBakedStates[mPlayFrame].segment(start, size), false, false);
            }
            if (mShowMarkers) {
                int sumDofs = mIndices[mSkels.size()]; 
                int nContact = (mBakedStates[mPlayFrame].size() - sumDofs) / 6;
                for (int i = 0; i < nContact; i++) {
                    Vector3d v = mBakedStates[mPlayFrame].segment(sumDofs + i * 6, 3);
                    Vector3d f = mBakedStates[mPlayFrame].segment(sumDofs + i * 6 + 3, 3) / 10.0;
                    glBegin(GL_LINES);
                    glVertex3f(v[0], v[1], v[2]);
                    glVertex3f(v[0] + f[0], v[1] + f[1], v[2] + f[2]);
                    glEnd();
                    mRI->setPenColor(Vector3d(0.2, 0.8, 0.2));
                    mRI->pushMatrix();
                    glTranslated(v[0], v[1], v[2]);
                    mRI->drawEllipsoid(Vector3d(0.01, 0.01, 0.01));
                    mRI->popMatrix();
                }
            }
        }
    }else{
        if (mShowMarkers) {
            for (int k = 0; k < mCollisionHandle->getCollisionChecker()->getNumContact(); k++) {
                Vector3d  v = mCollisionHandle->getCollisionChecker()->getContact(k).point;
                Vector3d n = mCollisionHandle->getCollisionChecker()->getContact(k).normal / 10.0;
                Vector3d f = mCollisionHandle->getCollisionChecker()->getContact(k).force / 100.0;

                mRI->setPenColor(Vector3d(0.2, 0.2, 0.8));
                glBegin(GL_LINES);
                glVertex3f(v[0], v[1], v[2]);
                glVertex3f(v[0] + f[0], v[1] + f[1], v[2] + f[2]);
                glEnd();
                mRI->setPenColor(Vector3d(0.2, 0.8, 0.2));
                mRI->pushMatrix();
                glTranslated(v[0], v[1], v[2]);
                mRI->drawEllipsoid(Vector3d(0.01, 0.01, 0.01));
                mRI->popMatrix();
            }
        }
    }

    
    for (unsigned int i = 0; i < mSkels.size(); i++)
        mSkels[i]->draw(mRI);
            
    // display the frame count in 2D text
    glDisable(GL_LIGHTING);
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
    case 's': // simulate one frame
        if (!mPlay) {
            mForce = Vector3d::Zero();
            setPose();
            mIntegrator.integrate(this, mTimeStep);
            mSimFrame++;
            bake();
            glutPostRedisplay();
        }
        break;
    case '1':
        mForce[2] = 40;
        cout << "push" << endl;
        break;
    case '2':
        mForce[2] = -40;
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
    case 'v': // show or hide markers
        mShowMarkers = !mShowMarkers;
        break;
 
    default:
        Win3D::keyboard(key,x,y);
    }
    glutPostRedisplay();
}

void MyWindow::bake()
{
    int nContact = mCollisionHandle->getCollisionChecker()->getNumContact();
    VectorXd state(mIndices.back() + 6 * nContact);
    for (unsigned int i = 0; i < mSkels.size(); i++)
        state.segment(mIndices[i], mDofs[i].size()) = mDofs[i];
    for (int i = 0; i < nContact; i++) {
        int begin = mIndices.back() + i * 6;
        state.segment(begin, 3) = mCollisionHandle->getCollisionChecker()->getContact(i).point;
        state.segment(begin + 3, 3) = mCollisionHandle->getCollisionChecker()->getContact(i).force;
    }
    mBakedStates.push_back(state);
}
