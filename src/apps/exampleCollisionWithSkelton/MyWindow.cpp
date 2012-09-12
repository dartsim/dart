#include "MyWindow.h"
#include "dynamics/SkeletonDynamics.h"
#include "dynamics/ContactDynamics.h"
#include "dynamics/BodyNodeDynamics.h"
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
    mDofs2.resize(mModel2->getNumDofs());
    mDofVels2.resize(mModel2->getNumDofs());

    for (unsigned int i = 0; i < mModel->getNumDofs(); i++) {
        mDofs[i] = utils::random(-0.7,0.7);
        mDofVels[i] = utils::random(-0.15,0.15);
    }
    mModel->initDynamics();
    mModel->setPose(mDofs,false,false);
    mModel->computeDynamics(mGravity, mDofVels, false);
    
    for(unsigned int i = 0; i < mModel2->getNumDofs(); i++){
        mDofs2[i] = 0.0;
        mDofVels2[i] = 0.0;
    }
    mModel2->initDynamics();
    mModel2->setPose(mDofs2, false, false);
    mModel2->computeDynamics(mGravity, mDofVels2, false);
    
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
    setPose();
    int size1 = mDofs.size();
    int size2 = mDofVels.size();
    int size3 = mDofs2.size();
    int size4 = mDofVels2.size();
    VectorXd deriv(size1 + size2 + size3 + size4);

    VectorXd qddot = mModel->getMassMatrix().fullPivHouseholderQr().solve(-mModel->getCombinedVector() + mModel->getExternalForces() + mCollisionHandle->getConstraintForce(0)); 
    mModel->clampRotation(mDofs, mDofVels);
    deriv.segment(size1, size2) = qddot; // set qddot (accelerations)
    deriv.head(size1) = mDofVels + (qddot * mTimeStep); // set velocities
    VectorXd qddot2 = mModel2->getMassMatrix().fullPivHouseholderQr().solve(-mModel2->getCombinedVector() + mModel2->getExternalForces() + mCollisionHandle->getConstraintForce(1));
    mModel2->clampRotation(mDofs2, mDofVels2);
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
    mModel->setPose(mDofs, true, true);
    mModel->computeDynamics(mGravity, mDofVels, true);
    mModel2->setPose(mDofs2, true, true);
    mModel2->computeDynamics(mGravity, mDofVels2, true);
    mCollisionHandle->applyContactForces();
}

void MyWindow::retrieveBakedState(int frame)
{
    int nDof1 = mDofs.size();
    int nDof2 = mDofs2.size();
    mModel->setPose(mBakedStates[frame].head(nDof1), false, false);
    mModel2->setPose(mBakedStates[frame].tail(nDof2), false, false);
}

void MyWindow::displayTimer(int _val)
{
    switch(mPlayState)
    {
    case PLAYBACK:
        if (mPlayFrame >= mBakedStates.size()) {
            mPlayFrame = 0;
        }
        retrieveBakedState(mPlayFrame);
        mPlayFrame++;
        glutPostRedisplay();
        glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
        break;
    case RECORD:
        if (mScreenshotScheduled) { // Wait for every frame to be drawn and captured
            glutPostRedisplay();
            glutTimerFunc(mDisplayTimeout + 1000.0, refreshTimer, _val);
        }
        else if (mMovieFrame >= mBakedStates.size()) {
            mPlayState = PAUSED;
        }
        else {
            retrieveBakedState(mMovieFrame);
            mMovieFrame++;
            mScreenshotScheduled = true;
            glutPostRedisplay();
            glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
        }
        break;
    case SIMULATE:
        setPose();
        int numIter = mDisplayTimeout / (mTimeStep*1000);
        for (int i = 0; i < numIter; i++) {
            mIntegrator.integrate(this, mTimeStep);
        }
        mSimFrame += numIter;   
        glutPostRedisplay();
        bake();
        glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
        break;
    }
}

void MyWindow::draw()
{
    glDisable(GL_LIGHTING);

    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    mModel->draw(mRI);
    mModel2->draw(mRI);

    if(mPlayState == SIMULATE){
        glBegin(GL_LINES);    
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

    if (mScreenshotScheduled)
    {
        mScreenshotScheduled = false;
        screenshot();
    }
    
    // display the frame count, the playback frame, and the movie length in 2D text
    glDisable(GL_LIGHTING);
    glColor3f(0.0,0.0,0.0);
    char buff[128];
    string frame;

    switch(mPlayState)
    {
    case PAUSED:
        sprintf(buff," ");
        break;
    case PLAYBACK:
        sprintf(buff,"Playing");
        break;
    case SIMULATE:
        sprintf(buff,"Simulating");
        break;
    case RECORD:
        sprintf(buff,"Saving a Movie");
        break;
    }
    frame = string(buff);
    yui::drawStringOnScreen(0.02f,0.17f,frame);

    sprintf(buff,"Sim Frame: %d", mSimFrame);
    frame = string(buff);
    yui::drawStringOnScreen(0.02f,0.12f,frame);

    sprintf(buff,"Play Frame: %d", mPlayFrame);
    frame = string(buff);
    yui::drawStringOnScreen(0.02f,0.07f,frame);

    sprintf(buff,"Movie Frame: %d", mMovieFrame);
    frame = string(buff);
    yui::drawStringOnScreen(0.02f,0.02f,frame);

    glEnable(GL_LIGHTING);
}

void MyWindow::keyboard(unsigned char key, int x, int y)
{
    switch(key){
    case ' ': // pause or unpause whatever's running; if space is the
              // first thing that's pressed, simulate
        if (mPlayState == PAUSED)
        {
            if (mPlayStateLast != PAUSED)
                mPlayState = mPlayStateLast;
            else
                mPlayState = SIMULATE;
            glutTimerFunc(mDisplayTimeout, refreshTimer, 0);
        }
        else
        {
            mPlayStateLast = mPlayState;
            mPlayState = PAUSED;
        }
        break;
    case 's': // switch to simulation mode
        if (mPlayState == SIMULATE) {
            mPlayState = PAUSED;
        }
        else {
            if (mPlayState == PAUSED)
                glutTimerFunc(mDisplayTimeout, refreshTimer, 0);
            mPlayState = SIMULATE;
            mPlayStateLast = SIMULATE;
        }
        break;
    case 'm': // switch to movie-saving mode
        if (mPlayState == RECORD) {
            mPlayState = PAUSED;
        }
        else {
            if (mPlayState == PAUSED)
                glutTimerFunc( mDisplayTimeout, refreshTimer, 0);
            mPlayState = RECORD;
            mPlayStateLast = RECORD;
        }
        break;
    case 'p': // switch to playback mode
        if (mPlayState == PLAYBACK) {
            mPlayState = PAUSED;
        }
        else {
            if (mPlayState == PAUSED)
                glutTimerFunc(mDisplayTimeout, refreshTimer, 0);
            mPlayState = PLAYBACK;
            mPlayStateLast = PLAYBACK;
        }
        break;

    case '[':
        if (mPlayState == PLAYBACK || mPlayStateLast == PLAYBACK)
        {
            mPlayFrame -= 1;
            if (mPlayFrame < 0) mPlayFrame = mBakedStates.size()-1;
            retrieveBakedState(mPlayFrame);
            glutPostRedisplay();
        }
        break;
    case ']':
        if (mPlayState == PLAYBACK || mPlayStateLast == PLAYBACK)
        {
            mPlayFrame += 1;
            if (mPlayFrame >= mBakedStates.size()) mPlayFrame = 0;
            retrieveBakedState(mPlayFrame);
            glutPostRedisplay();
        }
        break;
    case 'r': // set playback to the first frame
        mPlayFrame = 0;
        retrieveBakedState(mPlayFrame);
        glutPostRedisplay();
        break;
    case 't': // set playback motion to the newest simulated frame
        mPlayFrame = mBakedStates.size()-1;
        retrieveBakedState(mPlayFrame);
        glutPostRedisplay();
        break;
    case 'h': // show or hide markers
        mShowMarker = !mShowMarker;
        break;
    case 'n': // save a single screeNshot
        screenshot();
        break;
    case 'l': // right force
        mForce[0] = 300.0;
        static_cast<BodyNodeDynamics*>(mModel->getNode(0))->addExtForce(Vector3d(0.0, 0.1, 0), mForce);
        mForce[0] = 0.0;
        cout << "right" << endl;
        break;
    case 'k': // left force
        mForce[0] = -300.0;
        static_cast<BodyNodeDynamics*>(mModel->getNode(0))->addExtForce(Vector3d(0.0, 0.1, 0), mForce);
        mForce[0] = 0.0;
        cout << "left" << endl;
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
