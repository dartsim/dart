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
    static_cast<BodyNodeDynamics*>(mWorld->getSkeleton(0)->getNode("fullbody2_h_spine"))->addExtForce(Vector3d(0.0, 0.0, 0.0), mForce);
    ((MyWorld*)mWorld)->getController()->computeTorques(mWorld->getSkeleton(0)->get_q(), mWorld->getSkeleton(0)->get_dq());
    mWorld->getSkeleton(0)->setInternalForces(((MyWorld*)mWorld)->getController()->getTorques());
    ((MyWorld*)mWorld)->controlWalls();
    mWorld->step();

    ((MyWorld*)mWorld)->computeImpact();
    // for perturbation test
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

    // moving camera
    if (mSimulating) {
        mTrans[1] =  mWorld->getSkeleton(1)->getWorldCOM()[1] * -1000;
        if (mTrans[1] > 9000)
            mTrans[1] = 9000;
    }
    // display the frame count in 2D text
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
    case ' ': // use space key to play or stop the motion
        mSimulating = !mSimulating;
        if(mSimulating) {
            mPlay = false;
            glutTimerFunc( mDisplayTimeout, refreshTimer, 0);
        }
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
    case 'l': // lift force
        mForce[1] = 500;
        mImpulseDuration = 1000.0;
        break;
    case '=': // show or hide markers
        ((MyWorld*)mWorld)->setWallMaterial(((MyWorld*)mWorld)->getWallMaterial() + 10000);
        break;
    case '-': // show or hide markers
        ((MyWorld*)mWorld)->setWallMaterial(((MyWorld*)mWorld)->getWallMaterial() - 10000);
        break;
    default:
        Win3D::keyboard(key,x,y);

    }
    glutPostRedisplay();
}
