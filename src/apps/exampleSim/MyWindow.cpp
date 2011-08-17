#include "MyWindow.h"
#include "dynamics/SkeletonDynamics.h"
#include "utils/UtilsMath.h"
#include "yui/GLFuncs.h"
#include <cstdio>

using namespace std;
using namespace Eigen;
using namespace model3d;

void MyWindow::initDyn()
{
    // set random initial conditions
    mDofs.resize(mModel->getNumDofs());
    mDofVels.resize(mModel->getNumDofs());
    for(unsigned int i=0; i<mModel->getNumDofs(); i++){
        mDofs[i] = utils::random(-0.1,0.1);
        mDofVels[i] = utils::random(-0.1,0.1);
    }
    mModel->setPose(mDofs,false,false);
}

void MyWindow::displayTimer(int _val)
{
    int numIter = mDisplayTimeout / (mTimeStep*1000);
    for(int i=0; i<numIter; i++){
        mModel->setPose(mDofs,false,false);
        mModel->computeDynamics(mGravity, mDofVels, true);
        VectorXd qddot = -mModel->mM.fullPivHouseholderQr().solve(mModel->mCg); 
        mModel->clampRotation(mDofs,mDofVels);
        mDofVels += qddot*mTimeStep;
        mDofs += mDofVels*mTimeStep;
    }

    mFrame += numIter;   
    glutPostRedisplay();
    if(mRunning)	
        glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
}

void MyWindow::draw()
{
    mModel->draw(mRI);
    if(mShowMarker) mModel->drawHandles(mRI);
    
    // display the frame count in 2D text
    char buff[64];
    sprintf(buff,"%d",mFrame);
    string frame(buff);
    glDisable(GL_LIGHTING);
    glColor3f(0.0,0.0,0.0);
    yui::drawStringOnScreen(0.02f,0.02f,frame);
    glEnable(GL_LIGHTING);
}

void MyWindow::keyboard(unsigned char key, int x, int y)
{
    switch(key){
    case ' ': // use space key to play or stop the motion
        mRunning = !mRunning;
        if(mRunning)
            glutTimerFunc( mDisplayTimeout, refreshTimer, 0);
        break;
    case 'r': // reset the motion to the first frame
        mFrame = 0;
        break;
    case 'h': // show or hide markers
        mShowMarker = !mShowMarker;
        break;
    default:
        Win3D::keyboard(key,x,y);
    }
    glutPostRedisplay();
}


