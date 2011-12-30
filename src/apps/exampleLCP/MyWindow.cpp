#include "MyWindow.h"
#include "dynamics/SkeletonDynamics.h"
#include "dynamics/BodyNodeDynamics.h"
#include "dynamics/ContactDynamics.h"
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
    // set random initial conditions
    mDofs.resize(mSkels[0]->getNumDofs());
    mDofVels.resize(mSkels[0]->getNumDofs());
    mDofs2.resize(mSkels[1]->getNumDofs());
    mDofVels2.resize(mSkels[1]->getNumDofs());
    for(unsigned int i = 0; i < mSkels[0]->getNumDofs(); i++){
        mDofs[i] = utils::random(-0.5,0.5);
        mDofVels[i] = utils::random(-0.1,0.1);
    }
    for(unsigned int i = 0; i < mSkels[1]->getNumDofs(); i++){
        mDofs2[i] = 0.0;
        mDofVels2[i] = 0.0;
    }
    mDofVels2[0] = 0.3;
    mDofVels2[2] = 0.15;
    //    mDofs2[1] = -0.1;

    mSkels[0]->setPose(mDofs,false,false);
    mSkels[1]->setPose(mDofs2,false,false);
    mSkels[0]->computeDynamics(mGravity, mDofVels, false);
    mSkels[1]->computeDynamics(mGravity, mDofVels2, false);
    
    mCollisionHandle = new dynamics::ContactDynamics(mSkels, mTimeStep);
}

VectorXd MyWindow::getState() {
    int size1 = mDofs.size();
    int size2 = mDofVels.size();
    int size3 = mDofs2.size();
    int size4 = mDofVels2.size();
    VectorXd state(size1 + size2 + size3 + size4);
    state.head(size1) = mDofs;
    state.segment(size1, size2) = mDofVels;
    state.segment(size1 + size2, size3) = mDofs2;
    state.tail(size4) = mDofVels2;
    return state;
}

VectorXd MyWindow::evalDeriv() {
    int size1 = mDofs.size();
    int size2 = mDofVels.size();
    int size3 = mDofs2.size();
    int size4 = mDofVels2.size();
    VectorXd deriv(size1 + size2 + size3 + size4);
    VectorXd qddot = mSkels[0]->getMassMatrix().fullPivHouseholderQr().solve(-mSkels[0]->getCombinedVector() + mSkels[0]->getExternalForces() + mCollisionHandle->getConstraintForce(0)); 
    mSkels[0]->clampRotation(mDofs, mDofVels);
    deriv.segment(size1, size2) = qddot; // set qddot (accelerations)
    deriv.head(size1) = mDofVels + (qddot * mTimeStep); // set velocities
    VectorXd qddot2 = mSkels[1]->getMassMatrix().fullPivHouseholderQr().solve(-mSkels[1]->getCombinedVector() + mSkels[1]->getExternalForces() + mCollisionHandle->getConstraintForce(1));
    mSkels[1]->clampRotation(mDofs2, mDofVels2);
    deriv.tail(size4) = qddot2; // set qddot (accelerations)
    deriv.segment(size1 + size2, size3) = mDofVels2 + (qddot2 * mTimeStep); // set velocities
        
    return deriv;
}

void MyWindow::setState(VectorXd newState) {
    int size1 = mDofs.size();
    int size2 = mDofVels.size();
    int size3 = mDofs2.size();
    int size4 = mDofVels2.size();
    mDofs = newState.head(size1);
    mDofVels = newState.segment(size1, size2);
    mDofs2 = newState.segment(size1 + size2, size3);
    mDofVels2 = newState.tail(size4);
}

void MyWindow::setPose() {
    mSkels[0]->setPose(mDofs,false,false);
    mSkels[0]->computeDynamics(mGravity, mDofVels, true);
    mSkels[1]->setPose(mDofs2,false,false);
    mSkels[1]->computeDynamics(mGravity, mDofVels2, true);
    mCollisionHandle->applyContactForces();
}

void MyWindow::displayTimer(int _val)
{
    if (mPlayBack) {        
        if (mCurrFrame < mBakedStates.size()) {
            int nDof1 = mDofs.size();
            int nDof2 = mDofs2.size();
            mSkels[0]->setPose(mBakedStates[mCurrFrame].head(nDof1), false, false);
            mSkels[1]->setPose(mBakedStates[mCurrFrame].tail(nDof2), false, false);
            mCurrFrame++;
            glutPostRedisplay();
        }else{
            mCurrFrame = 0;
        }        
        glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
        return;
    }

    static Timer tSim("Simulation");
    int numIter = mDisplayTimeout / (mTimeStep*1000);
    for (int i = 0; i < numIter; i++) {
        tSim.startTimer();
        static_cast<BodyNodeDynamics*>(mSkels[1]->getNode(0))->addExtForce(Vector3d(0.1, 0.1, 0.1), mForce);
        setPose();
        mIntegrator.integrate(this, mTimeStep);
        tSim.stopTimer();
    }
    tSim.printScreen();

    bake();
    mForce = Vector3d::Zero();
    mFrame += numIter;   
    glutPostRedisplay();
   
    if (mRunning)
        glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
}

void MyWindow::draw()
{
    glDisable(GL_LIGHTING);

    //collision and draw
    MatrixXd worldTrans(4, 4);

    bool bCollide = false;
    MatrixXd matCOM;
    matCOM.setIdentity(4, 4);
    
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    mSkels[0]->draw(mRI);
    mSkels[1]->draw(mRI);

    
    glBegin(GL_LINES);
    VectorXd f = mCollisionHandle->getConstraintForce(1);
    for (int k = 0; k < mCollisionHandle->getCollisionChecker()->getNumContact(); k++) {
        Vector3d  v = mCollisionHandle->getCollisionChecker()->getContact(k).point;
        Vector3d n = mCollisionHandle->getCollisionChecker()->getContact(k).normal;
        //        glVertex3f(v[0], v[1], v[2]);
        //        glVertex3f(v[0]+n[0], v[1]+n[1], v[2]+n[2]);

        glVertex3f(v[0], v[1], v[2]);
        glVertex3f(v[0] + n[0], v[1] + n[1], v[2] + n[2]);

        //cout << "contact " << k << endl;
        //cout << v << endl;
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
        mForce[1] = 0.4;
        cout << "push" << endl;
        break;
    case 'q': // upper right force
        //mForce[0] = 300.0;        
        mForce[0] = -0.4;
        cout << "push" << endl;
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
    int nDof1 = mDofs.size();
    int nDof2 = mDofs2.size();
    VectorXd state(nDof1 + nDof2);
    for (int i = 0; i < nDof1; i++)
        state[i] = mDofs[i];
    for (int i = 0; i < nDof2; i++)
        state[nDof1 + i] = mDofs2[i];
        
    mBakedStates.push_back(state);
}
