#include "MyWindow.h"
#include "MyWorld.h"
#include "dynamics/ConstraintDynamics.h"
#include "dynamics/BodyNodeDynamics.h"
#include "yui/GLFuncs.h"
#include "math/UtilsMath.h"
#include <iostream>

using namespace Eigen;
using namespace std;
using namespace dynamics;
using namespace dart_math;
using namespace yui;

void MyWindow::timeStepping()
{
    // Add jet propulsion force
    static_cast<BodyNodeDynamics*>(mWorld->getSkeleton(0)->getNode("fullbody2_h_spine"))->addExtForce(Vector3d(0.0, 0.0, 0.0), mForce);
    // Add control force from your controller
    ((MyWorld*)mWorld)->getController()->computeTorques(mWorld->getSkeleton(0)->get_q(), mWorld->getSkeleton(0)->get_dq());
    mWorld->getSkeleton(0)->setInternalForces(((MyWorld*)mWorld)->getController()->getTorques());
    
    // Compute joint forces for the wall
    ((MyWorld*)mWorld)->controlWalls();

    // Integrate one step forward
    mWorld->step();

    // Measure accumulated impact
    ((MyWorld*)mWorld)->computeImpact();
    ((MyWorld*)mWorld)->computeJointStress();

    mImpulseDuration--;
    if (mImpulseDuration <= 0) {
        mImpulseDuration = 0;
        mForce.setZero();
    }
}

void MyWindow::drawSkels()
{
    glEnable(GL_LIGHTING);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    mWorld->getSkeleton(0)->draw(mRI);

    // Draw a checker board background
    glDisable(GL_LIGHTING);
    bool flip = true;
    double wall = -2.0;
    for(int i = -20; i < 20; i++){
        for(int j = -12; j < 25; j++){
            if(flip == true){
                glColor4d(0.7, 0.7, 0.7, 1.0);
                flip = false; 
            }else{
                glColor4d(0.8, 0.8, 0.8, 1.0);
                flip = true;
            }
            glBegin(GL_QUADS);
            glNormal3d(0, 0, 1);
            glVertex3d(i, j, wall);
            glVertex3d(i, j + 1, wall);
            glVertex3d(i + 1, j + 1, wall);
            glVertex3d(i + 1, j, wall);
            glEnd();
        }
    }

    // Set and Draw the ground
    VectorXd pose = mWorld->getSkeleton(2)->getPose();
    pose[1] = ((MyWorld*)mWorld)->getGroundHeight();
    mWorld->getSkeleton(2)->setPose(pose);
    Vector4d color;
    color << 0.5, 0.5, 0.5, 1.0;
    mWorld->getSkeleton(2)->draw(mRI, color, false);

    // Draw the elevator
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    color << 0.8, 0.3, 0.3, 1.0;
    mWorld->getSkeleton(1)->draw(mRI, color, false);
    mWorld->getSkeleton(3)->draw(mRI, color, false);

    // Moving camera
    if (mSimulating && mSimulating) {
        mTrans[1] =  mWorld->getSkeleton(1)->getWorldCOM()[1] * -1000;
    }
    // Display the accumulated impact in 2D text
    glDisable(GL_LIGHTING);
    char buff[64];
    sprintf(buff, "Imapct %5.0f", ((MyWorld*)mWorld)->getImpact());
    string frame(buff);
    glColor3f(0.0, 0.0, 0.0);
    drawStringOnScreen(0.75f, 0.02f, frame);
    glEnable(GL_LIGHTING);
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
    case 'l': // Add a propulsion force
        mForce[1] = 500;
        mImpulseDuration = 1000.0;
        break;
    case '=': // Make the wall sturdier
        ((MyWorld*)mWorld)->setWallMaterial(((MyWorld*)mWorld)->getWallMaterial() + 10000);
        break;
    case '-': // Make the wall flimsier (Don't over do it. Wall material coefficient can't be negative)
        ((MyWorld*)mWorld)->setWallMaterial(((MyWorld*)mWorld)->getWallMaterial() - 10000);
        break;
    case 'r': // reset the camera
        mTrans[1] = mWorld->getSkeleton(2)->getWorldCOM()[1] * -1000;
        break;
    default:
        Win3D::keyboard(key,x,y);

    }
    glutPostRedisplay();
}
