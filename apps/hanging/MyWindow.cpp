#include "MyWindow.h"

#include <stdio.h>

#include "common/Timer.h"
#include "math/Helpers.h"
#include "collision/CollisionDetector.h"
#include "constraint/ConstraintDynamics.h"
#include "constraint/PointConstraint.h"
#include "dynamics/BodyNode.h"
#include "dynamics/GenCoord.h"
#include "dynamics/Skeleton.h"
#include "yui/GLFuncs.h"

using namespace std;
using namespace Eigen;

using namespace dart;
using namespace math;
using namespace dynamics;
using namespace constraint;
using namespace yui;

MyWindow::MyWindow(simulation::World* _world)
    : SimWindow()
{
    assert(_world);
    mWorld = _world;

    mForce = Eigen::Vector3d::Zero();
    mImpulseDuration = 0;

    Eigen::Vector3d gravity(0.0, -9.81, 0.0);
    mWorld->setGravity(gravity);

    //
    std::vector<int> genCoordIds0;
    genCoordIds0.push_back(4);

    // default standing pose
    std::vector<int> genCoordIds1;
    genCoordIds1.push_back(1);
    genCoordIds1.push_back(6); // left hip
    genCoordIds1.push_back(9); // left knee
    genCoordIds1.push_back(10); // left ankle
    genCoordIds1.push_back(13); // right hip
    genCoordIds1.push_back(16); // right knee
    genCoordIds1.push_back(17); // right ankle
    genCoordIds1.push_back(21); // lower back
    genCoordIds1.push_back(28); // left shoulder
    genCoordIds1.push_back(34); // right shoulder

    Eigen::VectorXd initConfig0(1);
    initConfig0 << -2.9;
    Eigen::VectorXd initConfig1(10);
    initConfig1 << -0.1, 0.2, -0.5, 0.3, 0.2, -0.5, 0.3, -0.1, 0.5, -0.5;

    mWorld->getSkeleton(0)->setConfig(genCoordIds0, initConfig0);
    mWorld->getSkeleton(1)->setConfig(genCoordIds1, initConfig1);

    // create controller
    mController = new Controller(mWorld->getSkeleton(1),
                                 mWorld->getConstraintHandler(),
                                 mWorld->getTimeStep());

    for (int i = 0; i < mWorld->getSkeleton(1)->getNumGenCoords(); i++)
        mController->setDesiredDof(i, mController->getSkeleton()->getGenCoord(i)->get_q());

    // initialize constraint on the hand
    BodyNode* bd = mWorld->getSkeleton(1)->getBodyNode("h_hand_left");
    PointConstraint* point1 = new PointConstraint(bd, bd->getLocalCOM(), bd->getWorldTransform().translation() + bd->getWorldCOM(), 1);
    //mWorld->getConstraintHandler()->addConstraint(point1);
    bd = mWorld->getSkeleton(1)->getBodyNode("h_hand_right");
    PointConstraint* point2 = new PointConstraint(bd, bd->getLocalCOM(), bd->getWorldCOM(), 1);
    //mWorld->getConstraintHandler()->addConstraint(point2);
}

MyWindow::~MyWindow()
{
}

void MyWindow::timeStepping(int _val)
{
    int numIter = mDisplayTimeout / (mWorld->getTimeStep() * 1000);
    if (mPlay)
    {
        mPlayFrame += 30;
        if (mPlayFrame >= mBakedStates.size())
            mPlayFrame = 0;
        glutPostRedisplay();
        glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
    }
    else if (mSimulating)
    {
        mWorld->step();

        // for perturbation test
        mImpulseDuration--;
        if (mImpulseDuration <= 0) {
            mImpulseDuration = 0;
            mForce.setZero();
        }
//        glutPostRedisplay();
//        glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
    }
}

void MyWindow::drawSkels()
{
    for (unsigned int i = 0; i < mWorld->getNumSkeletons(); i++)
        mWorld->getSkeleton(i)->draw(mRI);

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
        Vector3d poa = mWorld->getSkeleton(1)->getBodyNode("fullbody1_root")->getWorldTransform() * Vector3d(0.0, 0.0, 0.0);
        Vector3d start = poa - mForce / 10.0;
        double len = mForce.norm() / 10.0;
        drawArrow3D(start, mForce, len, 0.05, 0.1);
    }
}

void MyWindow::keyboard(unsigned char key, int x, int y)
{
    switch(key){
    case ' ': // use space key to play or stop the motion
        mSimulating = !mSimulating;
        if (mSimulating) {
            mPlay = false;
            glutTimerFunc( mDisplayTimeout, refreshTimer, 0);
        }
        break;
    case 's': // simulate one frame
        if (!mPlay) {
            mForce = Vector3d::Zero();
            mWorld->step();
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
            mSimulating = false;
            glutTimerFunc( mDisplayTimeout, refreshTimer, 0);
        }
        break;
    case '[': // step backward
        if (!mSimulating) {
            mPlayFrame--;
            if(mPlayFrame < 0)
                mPlayFrame = 0;
            glutPostRedisplay();
        }
        break;
    case ']': // step forwardward
        if (!mSimulating) {
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
