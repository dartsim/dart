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

void MyWindow::retrieveBakedState(int frame)
{
    if (frame >= mBakedStates.size()) {
        std::cout << "tried to retrieve an invalid baked state" << std::endl;
    }
    else{
        for (unsigned int i = 0; i < mSkels.size(); i++) {
            int start = mIndices[i];
            int size = mDofs[i].size();
            mSkels[i]->setPose(mBakedStates[frame].segment(start, size), false, false);
        }
    }
}

void MyWindow::displayTimer(int _val)
{
    switch(mPlayState)
    {
    case PLAYBACK:
        if (mPlayFrame >= 0 && mPlayFrame >= mBakedStates.size() - mDisplayFrequency)
            mPlayFrame = -mDisplayFrequency;
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
            // compute and apply mForce
            if (mSimFrame > 1000 && mSimFrame < 1033) {
                cout << "push on the ball" << endl;
                mForce[0] = 60;
            } else {
                mForce[0] = 0.0;
            }
            static_cast<BodyNodeDynamics*>(mSkels[2]->getNode("sphere_root"))
                ->addExtForce(Vector3d(0.0, 0.0, 0), mForce);        
            // compute and apply pd controller value
            mController->computeTorques(mDofs[1], mDofVels[1], mDofVels[2]);
            mSkels[1]->setInternalForces(mController->getTorques());
            mIntegrator.integrate(this, mTimeStep);
            bake();
            mSimFrame++;
        }
        glutPostRedisplay();
        glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
        break;
    }
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

void MyWindow::drawContact(Vector3d vertex, Vector3d force, Vector3d penColor1, Vector3d penColor2, Vector3d ellipsoidColor)
{
    mRI->setPenColor(penColor1);
    glBegin(GL_LINES);
    glVertex3f(vertex[0], vertex[1], vertex[2]);
    glVertex3f(vertex[0] + force[0], vertex[1] + force[1], vertex[2] + force[2]);
    glEnd();
    mRI->setPenColor(penColor2);
    mRI->pushMatrix();
    glTranslated(vertex[0], vertex[1], vertex[2]);
    mRI->drawEllipsoid(ellipsoidColor);
    mRI->popMatrix();
}

void MyWindow::draw()
{
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    // draw contact points and associated force vectors
    if (mShowMarkers) {
        Vector3d penColor1;
        Vector3d penColor2;
        Vector3d ellipsoidColor;

        playstate_enum disp = mPlayState;
        if (mPlayState == PAUSED) disp = mPlayStateLast;

        if (disp == SIMULATE) {
            penColor1 = Vector3d(0.2, 0.2, 0.8);
            penColor2 = Vector3d(0.2, 0.8, 0.2);
            ellipsoidColor = Vector3d(0.01, 0.01, 0.01);
            for (int k = 0; k < mCollisionHandle->getCollisionChecker()->getNumContact(); k++) {
                drawContact(mCollisionHandle->getCollisionChecker()->getContact(k).point, 
                            mCollisionHandle->getCollisionChecker()->getContact(k).force / 100.0,
                            penColor1,
                            penColor2,
                            ellipsoidColor);
            }
        }
        else { // disp == PLAYBACK || disp == RECORD
            int frame;
            penColor1 = Vector3d(0.2, 0.2, 0.8);
            penColor2 = Vector3d(0.2, 0.8, 0.2);
            ellipsoidColor = Vector3d(0.01, 0.01, 0.01);
            if (disp == RECORD) frame = mMovieFrame;
            if (disp == PLAYBACK) frame = mPlayFrame;
            if (frame < 0) frame = 0;
            int sumDofs = mIndices[mSkels.size()]; 
            int nContact = (mBakedStates[frame].size() - sumDofs) / 6;
            for (int k = 0; k < nContact; k++) {
                drawContact(mBakedStates[frame].segment(sumDofs + k * 6, 3),
                            mBakedStates[frame].segment(sumDofs + k * 6 + 3, 3) / 10.0,
                            penColor1,
                            penColor2,
                            ellipsoidColor);
            }
        }
    }
    
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
            mPlayState = mPlayStateLast;
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
        cout << "push up and right" << endl;
        break;
    case '2': // upper right force
        mForce[0] = -40;
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
        state.segment(begin + 3, 3) = mCollisionHandle->getCollisionChecker()->getContact(i).force;
    }
    mBakedStates.push_back(state);
}
