#include "MyWindow.h"
#include "dynamics/BodyNodeDynamics.h"
#include "yui/GLFuncs.h"
#include <iostream>
#include "Controller.h"

using namespace Eigen;
using namespace dynamics;
using namespace yui;

void MyWindow::timeStepping()
{
    // Add push force
    static_cast<BodyNodeDynamics*>(mWorld->getSkeleton(0)->getNode("fullbody1_h_shin_left"))->addExtForce(Vector3d(0.0, 0.0, 0.0), mForce);

    // Compute and apply control force
    mController->computeTorques(mWorld->getSkeleton(0)->getPose(), mWorld->getSkeleton(0)->getPoseVelocity());
    mWorld->getSkeleton(0)->setInternalForces(mController->getTorques());

    // Integrate one step forward
    mWorld->step();

    // Apply push force for a period of time
    mImpulseDuration--;
    if (mImpulseDuration <= 0) {
        mImpulseDuration = 0;
        mForce.setZero();
    }    
}

void MyWindow::drawSkels()
{
    glEnable(GL_LIGHTING);
    // Draw the human
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    mWorld->getSkeleton(0)->draw(mRI);

    // Draw the ground
    Vector4d color;
    color << 0.7, 0.7, 0.7, 1.0;
    mWorld->getSkeleton(1)->draw(mRI, color, false);

    // Draw a bar
    mRI->setPenColor(Vector3d(0.8, 0.2, 0.2));
    mRI->pushMatrix();
    glTranslated(0.5, 0.95, 0.0);
    mRI->drawCube(Vector3d(0.04, 0.04, 2.0));
    mRI->popMatrix();
}


void MyWindow::keyboard(unsigned char key, int x, int y)
{
    switch(key){
    case ' ': // Use space key to play or stop the motion
        mSimulating = !mSimulating;
        if(mSimulating) {
            mPlay = false;
            glutTimerFunc( mDisplayTimeout, refreshTimer, 0);
        }
        break;
    case 'p': // PlayBack
        mPlay = !mPlay;
        if (mPlay) {
            mSimulating = false;
            glutTimerFunc( mDisplayTimeout, refreshTimer, 0);
        }
        break;
    case '[': // Step backward
        if (!mSimulating) {
            mPlayFrame--;
            if(mPlayFrame < 0)
                mPlayFrame = 0;
            glutPostRedisplay();
        }
        break;
    case ']': // Step forwardward
        if (!mSimulating) {
            mPlayFrame++;
            if(mPlayFrame >= mBakedStates.size())
                mPlayFrame = 0;
            glutPostRedisplay();
        }
        break;
    case 'v': // Show or hide contact info
        mShowMarkers = !mShowMarkers;
        break;
    case '1': // Push force
        mForce[0] = 50;
        mImpulseDuration = 100.0;
        break;
    case 'o': // Print out current pose
        cout << mWorld->getSkeleton(0)->getNode("fullbody1_h_hand_left")->getWorldCOM() << endl;
        cout << mWorld->getSkeleton(0)->getNode("fullbody1_h_hand_right")->getWorldCOM() << endl;
        break;
    default:
        Win3D::keyboard(key,x,y);

    }
    glutPostRedisplay();
}
