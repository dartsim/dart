#include "MyWindow.h"
#include "dynamics/SkeletonDynamics.h"
#include "dynamics/ContactDynamics.h"
#include "utils/UtilsMath.h"
#include "utils/Timer.h"
#include "yui/GLFuncs.h"
#include <cstdio>
#include "yui/GLFuncs.h"
#include "kinematics/BodyNode.h"
#include "kinematics/Shape.h"

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
    for (unsigned int i=0; i<mModel->getNumDofs(); i++) {
        mDofs[i] = utils::random(-0.5,0.5);
        mDofVels[i] = utils::random(-0.1,0.1);
    }
    mModel->setPose(mDofs,false,false);
    mModel->computeDynamics(mGravity, mDofVels, false);
    
    VectorXd zeroVec(VectorXd::Zero(3));
    mModel2->computeDynamics(mGravity, zeroVec, false);
    mModel2->setPose(zeroVec, false, false);
    
    //init collision mesh
    //    mContactCheck.addCollisionSkeletonNode(mModel->getRoot(), true);
    //    mContactCheck.addCollisionSkeletonNode(mModel2->getRoot(), true);

    mCollisionHandle = new dynamics::ContactDynamics(mSkels, mTimeStep);

}

VectorXd MyWindow::getState() {
    VectorXd state(mDofs.size() + mDofVels.size());
    state.head(mDofs.size()) = mDofs;
    state.tail(mDofVels.size()) = mDofVels;
    return state;
}

VectorXd MyWindow::evalDeriv() {
    VectorXd deriv(mDofs.size() + mDofVels.size());
    VectorXd qddot = mModel->getMassMatrix().fullPivHouseholderQr().solve(-mModel->getCombinedVector() + mModel->getExternalForces() + mCollisionHandle->getConstraintForce(0)); 
    mModel->clampRotation(mDofs, mDofVels);
    deriv.tail(mDofVels.size()) = qddot; // set qddot (accelerations)
    deriv.head(mDofs.size()) = (mDofVels + (qddot * mTimeStep)); // set velocities
    return deriv;
}

void MyWindow::setState(VectorXd newState) {
    mDofVels = newState.tail(mDofVels.size());
    mDofs = newState.head(mDofs.size());
}

void MyWindow::setPose() {
    mModel->setPose(mDofs,false,false);
    mModel->computeDynamics(mGravity, mDofVels, true);
    mCollisionHandle->applyContactForces();
}

void MyWindow::displayTimer(int _val)
{
    if (mPlayBack) {        
        if (mCurrFrame < mBakedStates.size()) {
            mModel->setPose(mBakedStates[mCurrFrame], false, false);
            mCurrFrame++;
            glutPostRedisplay();
        }else{
            mCurrFrame = 0;
        }        
        glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
        return;
    }

    int numIter = mDisplayTimeout / (mTimeStep*1000);
    for (int i = 0; i < numIter; i++) {
        setPose();
        mIntegrator.integrate(this, mTimeStep);
    }
    bake();
    //    mContactCheck.checkCollision();

    mFrame += numIter;   
    glutPostRedisplay();
    if(mRunning)	
        glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
}

void MyWindow::draw()
{
    glDisable(GL_LIGHTING);

    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    mModel->draw(mRI);
    mModel2->draw(mRI);
    if (mRunning) {
        glBegin(GL_LINES);
        for (int k = 0; k < mCollisionHandle->getCollisionChecker()->getNumContact(); k++) {
            Vector3d  v = mCollisionHandle->getCollisionChecker()->getContact(k).point;
            Vector3d n = mCollisionHandle->getCollisionChecker()->getContact(k).normal;
            glVertex3f(v[0], v[1], v[2]);
            /* printf("%f %f %f\n", v[0], v[1], v[2]);*/
            glVertex3f(v[0]+n[0], v[1]+n[1], v[2]+n[2]);
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
    VectorXd state(mDofs.size());
    for (int i = 0; i < mDofs.size(); i++)
        state[i] = mDofs[i];        
    mBakedStates.push_back(state);
}
