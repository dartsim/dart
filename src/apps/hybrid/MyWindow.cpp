#include "MyWindow.h"
#include "dynamics/BodyNodeDynamics.h"
#include "kinematics/Dof.h"
#include "utils/UtilsMath.h"
#include "utils/Timer.h"
#include "yui/GLFuncs.h"
#include "Controller.h"

using namespace dynamics;
using namespace utils;

static double initPose[] = {0.0, 0.0, 0.0, 1.57, 3.14, 0.0, 0.0, -0.868398, -0.256271, -0.794892, -0.129097, 0.147894, -0.690374, -0.605116, -0.148172, -0.243184, -0.220659, -0.4107145, -0.0341097, -0.64186, -0.040947, -0.3300072, 0.014561, -0.570674, -0.0651653, -0.369658, -0.140586, -0.643183, -0.0800537};

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
    for (int i = 0; i < mSkels[0]->getNumDofs(); i++)
        mDofs[0][i] = initPose[i];

    for (unsigned int i = 0; i < mSkels.size(); i++) {
        mSkels[i]->initDynamics();
        mSkels[i]->setPose(mDofs[i], false, false);
        // compute dynamics here because computation of control force at first iteration needs to access mass matrix
        mSkels[i]->computeDynamics(mGravity, mDofVels[i], false);
    }
    
    // init controller
    mController = new Controller(mSkels[0]);

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
        // The root follows the desired acceleration and the rest of the body follows dynamic equations. The root acceleration will impact the rest of the body correctly. Collision or other external forces will alter the root acceleration 
   setPose();
    VectorXd deriv = VectorXd::Zero(mIndices.back() * 2);    
    for (unsigned int i = 0; i < mSkels.size(); i++) {
        // skip immobile objects in forward simulation
        if (mSkels[i]->getImmobileState())
            continue;
        int start = mIndices[i] * 2;
        int size = mDofs[i].size();
        Vector3d desiredAccel(0.0, 0.0, 0.0);
        if (mSimFrame > 500 && mSimFrame < 800)
            desiredAccel[0] = 5.0;
        else if (mSimFrame > 800 && mSimFrame < 1100)
            desiredAccel[0] = -7.0;
        else if (mSimFrame > 1100 && mSimFrame < 1400)
            desiredAccel[0] = 6.0;
        else if (mSimFrame > 1400 && mSimFrame < 1700)
            desiredAccel[0] = -5.0;
        else if (mSimFrame > 1700 && mSimFrame < 2000)
            desiredAccel[0] = 5.0;
        else if (mSimFrame > 2000 && mSimFrame < 2300)
            desiredAccel[0] = -5.0;       

        int nDof = mSkels[i]->getNumDofs();
        MatrixXd J(3, nDof);
        J.setZero();
        J(0, 0) = 1.0;
        J(1, 1) = 1.0;
        J(2, 2) = 1.0;
 
        MatrixXd A = mSkels[i]->getMassMatrix();
        A.block(0, 0, nDof, 3) = J.transpose();
        VectorXd b = -mSkels[i]->getCombinedVector() + mSkels[i]->getInternalForces() - mSkels[i]->getMassMatrix().block(0, 0, nDof, 3) * desiredAccel;
        VectorXd x = A.inverse() * b;
        VectorXd qddot = x;
        qddot.head(3) = desiredAccel;
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
            mSkels[i]->setPose(mDofs[i], false, false);
        } else {
            mSkels[i]->setPose(mDofs[i], false, false);
            mSkels[i]->computeDynamics(mGravity, mDofVels[i], true);
        }
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
            cout << "iter " << mSimFrame << endl;
            //            tSim.startTimer();
            //          static_cast<BodyNodeDynamics*>(mSkels[0]->getNode("manipulator_indexDIP"))->addExtForce(Vector3d(0.02, 0.0, 0), mForce);
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
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

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
