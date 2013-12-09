#include "MyWindow.h"
#include "simulation/World.h"
#include "dynamics/ConstraintDynamics.h"
#include "math/UtilsMath.h"
#include "yui/GLFuncs.h"
#include <iostream>

using namespace Eigen;
using namespace std;
using namespace dart_math;
using namespace yui;


void MyWindow::timeStepping()
{
    mController->computeTorques(mWorld->getSkeleton(0)->get_q(), mWorld->getSkeleton(0)->get_dq(), mWorld->getCollisionHandle()->getTotalConstraintForce(0));
    mWorld->getSkeleton(0)->setInternalForces(mController->getTorques());
    mWorld->step();

    if(!mSwish) {
        Vector3d diff = mWorld->getSkeleton(1)->getWorldCOM() - mWorld->getSkeleton(2)->getWorldCOM();
        if (diff[1] < 0.0 && diff[1] > -0.01 && diff[0] * diff[0] + diff[2] * diff[2] < 0.01) { 
            mSwish = true;
            mScore += 2;
        }
    }
}

void MyWindow::drawSkels()
{
    glEnable(GL_LIGHTING);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    mWorld->getSkeleton(0)->draw(mRI);
    Vector4d color;
    color << 0.9, 0.2, 0.2, 1.0;
    mWorld->getSkeleton(1)->draw(mRI, color, false);
    mWorld->getSkeleton(2)->draw(mRI);

    
    if (mSwish) {
        // Display text
        glDisable(GL_LIGHTING);
        char buff[64];
        sprintf(buff, "Swish!!!");
        string frame(buff);
        glColor3f(0.0, 0.0, 0.0);
        drawStringOnScreen(0.85f, 0.02f, frame);
        glEnable(GL_LIGHTING);
    }
    glDisable(GL_LIGHTING);
    char buff[64];
    sprintf(buff, "%d", mScore);
    string frame(buff);
    glColor3f(0.0, 0.0, 0.0);
    drawStringOnScreen(0.93f, 0.95f, frame);
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
    case 'r': // reset arm, ball, and hoop (with a new location)
        resetScene();
    default:
        Win3D::keyboard(key,x,y);

    }
    glutPostRedisplay();
}

void MyWindow::resetScene() {
    // Reset the arm
    int nDof = mWorld->getSkeleton(0)->getNumDofs();    
    VectorXd initArmPose = VectorXd::Zero(nDof);
    mWorld->getSkeleton(0)->set_dq(initArmPose);
    initArmPose[2] = 3.14;
    mWorld->getSkeleton(0)->setPose(initArmPose);

    // Reset the ball
    nDof = mWorld->getSkeleton(1)->getNumDofs();    
    VectorXd initBallPose = VectorXd::Zero(nDof);
    mWorld->getSkeleton(1)->set_dq(initBallPose);
    initBallPose << 0.45, 0.1, 0.03;
    mWorld->getSkeleton(1)->setPose(initBallPose);

    // Move the hoop
    double x = random(-2, 2);
    double y = random(0, 2);
    double z = random(-2, 2);
    nDof = mWorld->getSkeleton(2)->getNumDofs();    
    VectorXd initHoopPose = VectorXd::Zero(nDof);
    initHoopPose << x, y, z;
    mWorld->getSkeleton(2)->setPose(initHoopPose);

    //    initBallPose << x, y + 1.0, z + 0.09;
    //mWorld->getSkeleton(1)->setPose(initBallPose);

    mController->initializeTargetPose();
    if (!mSwish)
        mScore -= 1;
    mSwish = false;
}
