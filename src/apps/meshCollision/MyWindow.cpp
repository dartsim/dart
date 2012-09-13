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
    mDofs[1][3] = -0.425;
//    mDofs[2][1] = mDofs[1][1] + 0.025 + 0.025;
//
//    // Old cube
//    mDofs[3][0] = 0.05 -0.06;
//    mDofs[3][1] = mDofs[2][1] + 0.025 + 0.025 + 0.03;
    
     // Our Mesh
//    mDofs[4][0] = 0.05;
    //mDofs[4][1] = mDofs[2][1] + 0.025 + 0.025 + 0.03;

     //rotate about z
    /*
    mDofs[0][1] = -0.35;
    mDofs[1][1] = -0.35;
    mDofs[2][1] = -0.35;
    mDofs[1][0] = mDofs[0][0] + 0.025 + 0.025;
    mDofs[2][0] = mDofs[1][0] + 0.025 + 0.025;
    mDofs[0][5] = 1.57;
    mDofs[1][5] = 1.57;
    mDofs[2][5] = 1.57;
    */
    /*
    mDofs[0][1] = -0.35;
    mDofs[1][0] = 0.06;
    mDofs[1][1] = -0.2;
    mDofs[2][0] = 0.02;
    mDofs[2][2] = 0.04;
    mDofs[3][0] = 0.09;
    mDofs[3][1] = 0.08;
    */
    for (unsigned int i = 0; i < mSkels.size(); i++) {
        mSkels[i]->initDynamics();
        mSkels[i]->setPose(mDofs[i], false, false);
        mSkels[i]->computeDynamics(mGravity, mDofVels[i], false);
    }
    mSkels[0]->setImmobileState(true);

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
    setPose();
    VectorXd deriv = VectorXd::Zero(mIndices.back() * 2);    
    for (unsigned int i = 0; i < mSkels.size(); i++) {
        if (mSkels[i]->getImmobileState())
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
        if (mSkels[i]->getImmobileState()) {
            mSkels[i]->setPose(mDofs[i], true, false);
        } else {
            mSkels[i]->setPose(mDofs[i], true, true);
            mSkels[i]->computeDynamics(mGravity, mDofVels[i], true);
        }
    }
    mCollisionHandle->applyContactForces();
}

void MyWindow::retrieveBakedState(int frame)
{
    for (unsigned int i = 0; i < mSkels.size(); i++) {
        int start = mIndices[i];
        int size = mDofs[i].size();
        mSkels[i]->setPose(mBakedStates[frame].segment(start, size), false, false);
    }
}

void MyWindow::displayTimer(int _val)
{
    switch(mPlayState)
    {
    case PLAYBACK:
        if (mPlayFrame >= 0 && mPlayFrame >= mBakedStates.size() - mDisplayFrequency) {
            mPlayFrame = -mDisplayFrequency;
        }
        mPlayFrame += mDisplayFrequency;
        retrieveBakedState(mPlayFrame);
        glutPostRedisplay();
        glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
        break;
    case RECORD:
        if (mScreenshotScheduled) { // Wait for every frame to be drawn and captured
            glutPostRedisplay();
            glutTimerFunc(mDisplayTimeout + 1000.0, refreshTimer, _val);
        }
        else if (mMovieFrame >= 0 && mMovieFrame >= mBakedStates.size() - mDisplayFrequency) {
            mPlayState = PAUSED;
        }
        else {
            mMovieFrame += mDisplayFrequency;
            retrieveBakedState(mMovieFrame);
            mScreenshotScheduled = true;
            glutPostRedisplay();
            glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
        }
        break;
    case SIMULATE:
        int numIter = mDisplayTimeout / (mTimeStep * 1000);
        setPose();              // retrieve current simulation state
        for (int i = 0; i < numIter; i++) {
            static_cast<BodyNodeDynamics*>(mSkels[1]->getNode(0))->
                addExtForce(Vector3d(0.0, 0.0, 0.0), mForce);
            mIntegrator.integrate(this, mTimeStep);
            mSimFrame++;
            bake();
        }
        mImpulseDuration--;
        if (mImpulseDuration <= 0) {
            mImpulseDuration = 0;
            mForce.setZero();
        }
        glutPostRedisplay();
        glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
        break;
    }
}

void MyWindow::drawContact(Vector3d vertex, Vector3d force, Vector3d penColor, Vector3d ellipsoidColor)
{
    glBegin(GL_LINES);
    glVertex3f(vertex[0], vertex[1], vertex[2]);
    glVertex3f(vertex[0] + force[0], vertex[1] + force[1], vertex[2] + force[2]);
    glEnd();
    mRI->setPenColor(penColor);
    mRI->pushMatrix();
    glTranslated(vertex[0], vertex[1], vertex[2]);
    mRI->drawEllipsoid(ellipsoidColor);
    mRI->popMatrix();
}

void MyWindow::drawText()
{
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

void MyWindow::draw()
{
    glDisable(GL_LIGHTING);

    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    // draw contact points and associated force vectors
    if (mShowMarkers) {
        Vector3d penColor;
        Vector3d ellipsoidColor;

        playstate_enum disp = mPlayState;
        if (mPlayState == PAUSED) disp = mPlayStateLast;
        // std::cout << "disp: " << disp << std::endl;;


        if (disp == SIMULATE) {
            penColor = Vector3d(0.2, 0.2, 0.8);
            ellipsoidColor = Vector3d(0.02, 0.02, 0.02);
            for (int k = 0; k < mCollisionHandle->getCollisionChecker()->getNumContact(); k++) {
                drawContact(mCollisionHandle->getCollisionChecker()->getContact(k).point, 
                            mCollisionHandle->getCollisionChecker()->getContact(k).force / 10.0,
                            penColor,
                            ellipsoidColor);
            }
        }
        else { // disp == PLAYBACK || disp == RECORD
            assert(disp == RECORD || disp == PLAYBACK);
            int frame;
            penColor = Vector3d(0.8, 0.2, 0.2);
            ellipsoidColor = Vector3d(0.02, 0.02, 0.02);
            if (disp == RECORD) frame = mMovieFrame;
            if (disp == PLAYBACK) frame = mPlayFrame;
            if (frame < 0) frame = 0;
            int sumDofs = mIndices[mSkels.size()]; 
            int nContact = (mBakedStates[frame].size() - sumDofs) / 6;
            for (int k = 0; k < nContact; k++) {
                drawContact(mBakedStates[frame].segment(sumDofs + k * 6, 3),
                            mBakedStates[frame].segment(sumDofs + k * 6 + 3, 3) / 10.0,
                            penColor,
                            ellipsoidColor);
            }
        }
    }

    // draw the actual skeletons
    for (unsigned int i = 0; i < mSkels.size(); i++)
        mSkels[i]->draw(mRI);

    // take screenshots before putting up text
    if (mScreenshotScheduled)
    {
        mScreenshotScheduled = false;
        screenshot();
    }
    
    drawText();
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
    case 's': // simulation mode
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


    case 'n': // save a single screeNshot
        screenshot();
        break;
    case 'f': // simulation mode, pause, and simulate one frame
        mPlayState = PAUSED;
        mPlayStateLast = SIMULATE;
        
        setPose();
        mForce = Vector3d::Zero();
        mIntegrator.integrate(this, mTimeStep);
        mSimFrame++;
        bake();
        glutPostRedisplay();
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
    case 'v': // show or hide markers
        mShowMarkers = !mShowMarkers;
        break;

    case '1': // upper right force
        mForce[0] = 40;
        mImpulseDuration = 1.0;
        cout << "push up and right" << endl;
        break;
    case '2': // upper right force
        mForce[0] = -40;
        mImpulseDuration = 1.0;
        cout << "push left" << endl;
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
        //state.segment(begin + 3, 3) = mCollisionHandle->getCollisionChecker()->getContact(i).force;
        state.segment(begin + 3, 3) = mCollisionHandle->getCollisionChecker()->getContact(i).normal;
    }
    mBakedStates.push_back(state);
}
