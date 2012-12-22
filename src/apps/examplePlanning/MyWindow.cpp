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
    int numDofs = 0;
    for(int i = 0; i < mWorld->getNumSkeletons(); i++) {
        numDofs += mWorld->getSkeleton(i)->getNumDofs();
    }

    mDofs.resize(mWorld->getNumSkeletons());
    mDofVels.resize(mWorld->getNumSkeletons());

    for (unsigned int i = 0; i < mDofs.size(); i++) {
        mDofs[i].resize(mWorld->getSkeleton(i)->getNumDofs());
        mDofVels[i].resize(mWorld->getSkeleton(i)->getNumDofs());
        mWorld->getSkeleton(i)->getPose(mDofs[i]);
        mDofVels[i] = mWorld->getSkeleton(i)->getQDotVector();
        if(!mWorld->getSkeleton(i)->getImmobileState()) {
            mWorld->getSkeleton(i)->computeDynamics(mGravity, mDofVels[i], false);
        }
    }
}

VectorXd MyWindow::getState() {
    VectorXd state(mIndices.back() * 2);
    for (int i = 0; i < mWorld->getNumSkeletons(); i++) {
        int start = mIndices[i] * 2;
        int size = mDofs[i].size();
        state.segment(start, size) = mDofs[i];
        state.segment(start + size, size) = mDofVels[i];
    }
    return state;
}

VectorXd MyWindow::evalDeriv() {
    // compute dynamic equations
    for (int i = 0; i < mWorld->getNumSkeletons(); i++) {
        if (mWorld->getSkeleton(i)->getImmobileState()) {
            // need to update node transformation for collision
            mWorld->getSkeleton(i)->setPose(mDofs[i], true, false);
        } else {
            // need to update first derivatives for collision
            mWorld->getSkeleton(i)->setPose(mDofs[i], false, true);
            mWorld->getSkeleton(i)->computeDynamics(mGravity, mDofVels[i], true);
        }
    }
    // compute contact forces
    mWorld->mCollisionHandle->applyContactForces();

    // compute derivatives for integration
    VectorXd deriv = VectorXd::Zero(mIndices.back() * 2);
    for (int i = 0; i < mWorld->getNumSkeletons(); i++) {
        // skip immobile objects in forward simulation
        if (mWorld->getSkeleton(i)->getImmobileState())
            continue;
        int start = mIndices[i] * 2;
        int size = mDofs[i].size();
        VectorXd qddot = mWorld->getSkeleton(i)->getMassMatrix().fullPivHouseholderQr().solve(-mWorld->getSkeleton(i)->getCombinedVector() + mWorld->getSkeleton(i)->getExternalForces() + mWorld->mCollisionHandle->getConstraintForce(i));
        mWorld->getSkeleton(i)->clampRotation(mDofs[i], mDofVels[i]);
        deriv.segment(start, size) = mDofVels[i] + (qddot * mTimeStep); // set velocities
        deriv.segment(start + size, size) = qddot; // set qddot (accelerations)
    }
    return deriv;
}

void MyWindow::setState(VectorXd newState) {
    for (int i = 0; i < mWorld->getNumSkeletons(); i++) {
        int start = mIndices[i] * 2;
        int size = mDofs[i].size();
        mDofs[i] = newState.segment(start, size);
        mDofVels[i] = newState.segment(start + size, size);
    }
}

void MyWindow::retrieveBakedState(int frame)
{
    for (int i = 0; i < mWorld->getNumSkeletons(); i++) {
        int start = mIndices[i] * 2;
        int size = mDofs[i].size();
        mWorld->getSkeleton(i)->setPose(mBakedStates[frame].segment(start, size), false, false);
    }
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
    for(int i = 0; i < mWorld->getNumSkeletons(); i++) {
        mWorld->getSkeleton(i)->draw(mRI);
    }

    if(mPlayState == SIMULATE){
        glBegin(GL_LINES);    
        for (int k = 0; k < mWorld->mCollisionHandle->getCollisionChecker()->getNumContact(); k++) {
            Vector3d  v = mWorld->mCollisionHandle->getCollisionChecker()->getContact(k).point;
            Vector3d n = mWorld->mCollisionHandle->getCollisionChecker()->getContact(k).normal;
            glVertex3f(v[0], v[1], v[2]);
            glVertex3f(v[0] + n[0], v[1] + n[1], v[2] + n[2]);
        }
        glEnd();

        mRI->setPenColor(Vector3d(0.2, 0.2, 0.8));
        for (int k = 0; k < mWorld->mCollisionHandle->getCollisionChecker()->getNumContact(); k++) {
            Vector3d  v = mWorld->mCollisionHandle->getCollisionChecker()->getContact(k).point;
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

    default:
        Win3D::keyboard(key,x,y);
    }
    glutPostRedisplay();
}

void MyWindow::bake()
{
    VectorXd state(mIndices.back());
    for(int i = 0; i < mWorld->getNumSkeletons(); i++) {
        state.segment(mIndices[i], mDofs[i].size()) = mDofs[i];
    }
    mBakedStates.push_back(state);
}
