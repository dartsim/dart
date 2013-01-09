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
#include "planning/PathPlanner.h"
#include "planning/Controller.h"
#include "planning/Trajectory.h"
#include "robotics/Robot.h"
#include "kinematics/Dof.h"

using namespace Eigen;
using namespace kinematics;
using namespace utils;
using namespace integration;
using namespace dynamics;
using namespace std;
using namespace planning;

MyWindow::MyWindow(): Win3D() {
    DartLoader dl;
    mWorld = dl.parseWorld(DART_DATA_PATH"/scenes/hubo_world.urdf");
        
    // Add ground plane
    robotics::Object* ground = new robotics::Object();
    ground->addDefaultRootNode();
    dynamics::BodyNodeDynamics* node = new dynamics::BodyNodeDynamics();
    node->setShape(new kinematics::ShapeCube(Eigen::Vector3d(10.0, 10.0, 0.0001), 1.0));
    kinematics::Joint* joint = new kinematics::Joint(ground->getRoot(), node);
    ground->addNode(node);
    ground->initSkel();
    ground->update();
    ground->setImmobileState(true);
    mWorld->addObject(ground);
    mWorld->rebuildCollision();

    mBackground[0] = 1.0;
    mBackground[1] = 1.0;
    mBackground[2] = 1.0;
    mBackground[3] = 1.0;

    mPlayState = PAUSED;
    mSimFrame = 0;
    mPlayFrame = 0;
    mMovieFrame = 0;
    mTime = 0.0;

    mShowMarker = false;

    mTrans[2] = -2000.f;
    mEye = Eigen::Vector3d(2.0, -2.0, 2.0);
    mUp = Eigen::Vector3d(0.0, 0.0, 1.0);
    
    mGravity = Eigen::Vector3d(0.0, 0.0, -9.8);
    mTimeStep = 1.0/1000.0;

    vector<int> trajectoryDofs(7);
    string trajectoryNodes[] = {"Body_RSP", "Body_RSR", "Body_RSY", "Body_REP", "Body_RWY", "rightUJoint", "rightPalmDummy"}; 
    for(int i = 0; i < 7; i++) {
        trajectoryDofs[i] = mWorld->getRobot(0)->getNode(trajectoryNodes[i].c_str())->getDof(0)->getSkelIndex();
    }

    vector<int> actuatedDofs(mWorld->getRobot(0)->getNumDofs() - 6);
    for(unsigned int i = 0; i < actuatedDofs.size(); i++) {
        actuatedDofs[i] = i + 6;
    }

    // Deactivate collision checking between the feet and the ground during planning
    mWorld->mCollisionHandle->getCollisionChecker()->deactivatePair(mWorld->getRobot(0)->getNode("leftFoot"), ground->getNode(1));
    mWorld->mCollisionHandle->getCollisionChecker()->deactivatePair(mWorld->getRobot(0)->getNode("rightFoot"), ground->getNode(1));

    mController = new planning::Controller(mWorld->getSkeleton(0), actuatedDofs);
    PathPlanner<> pathPlanner(*mWorld);
    VectorXd goal(7);
    goal << 0.0, -M_PI / 2.0, 0.0, -M_PI / 2.0, 0.0, 0.0, 0.0;
    list<VectorXd> path;
    if(!pathPlanner.planPath(0, trajectoryDofs, Eigen::VectorXd::Zero(7), goal, path)) {
        cout << "Path planner could not find a path" << endl;
    }
    else {
        const VectorXd maxVelocity = 0.3 * VectorXd::Ones(7);
        const VectorXd maxAcceleration = 0.3 * VectorXd::Ones(7);
        Trajectory* trajectory = new Trajectory(path, maxVelocity, maxAcceleration);
        mController->setTrajectory(trajectory, 0.1, trajectoryDofs);
    }

    mWorld->mCollisionHandle->getCollisionChecker()->activatePair(mWorld->getRobot(0)->getNode("leftFoot"), ground->getNode(1));
    mWorld->mCollisionHandle->getCollisionChecker()->activatePair(mWorld->getRobot(0)->getNode("rightFoot"), ground->getNode(1));

    initDyn();

    std::cout << 
        "\nKeybindings:\n" <<
        "\n" <<
        "s: start or continue simulating.\n" <<
        "\n" <<
        "p: start or continue playback.\n" <<
        "r, t: move to start or end of playback.\n" <<
        "[, ]: step through playback by one frame.\n" <<
        "\n" <<
        "m: start or continue movie recording.\n" <<
        "\n" <<
        "space: pause/unpause whatever is happening.\n" <<
        "\n" <<
        "q, escape: quit.\n" <<
        std::endl;
}


void MyWindow::initDyn()
{
    int sumNDofs = 0;
    mIndices.push_back(sumNDofs);
    for (unsigned int i = 0; i < mWorld->getNumSkeletons(); i++) {
        int nDofs = mWorld->getSkeleton(i)->getNumDofs();
        sumNDofs += nDofs;
        mIndices.push_back(sumNDofs);
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
        VectorXd qddot = mWorld->getSkeleton(i)->getMassMatrix().fullPivHouseholderQr().solve(
            - mWorld->getSkeleton(i)->getCombinedVector() + mWorld->getSkeleton(i)->getExternalForces()
            + mWorld->mCollisionHandle->getConstraintForce(i) + mWorld->getSkeleton(i)->getInternalForces());

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
        int start = mIndices[i];
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
        int numIter = (mDisplayTimeout / 1000.0) / mTimeStep;
        for (int i = 0; i < numIter; i++) {
            mWorld->getSkeleton(0)->setInternalForces(mController->getTorques(mDofs[0], mDofVels[0], mTime));
            mIntegrator.integrate(this, mTimeStep);
            mTime += mTimeStep;
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
