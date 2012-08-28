#include "MyWindow.h"
#include "dynamics/BodyNodeDynamics.h"
#include "kinematics/Dof.h"
#include "utils/UtilsMath.h"
#include "utils/Timer.h"
#include "yui/GLFuncs.h"
#include "Analyzer.h"

using namespace dynamics;

void MyWindow::initDyn()
{
    mTimeStep = 1.0 / mMotion->getFPS();

    mSkel->initDynamics();
    mSkel->setPose(mMotion->getPoseAtFrame(0), false, false);
    
    mAnalyzer = new Analyzer(mMotion, mSkel, mGravity);
}

void MyWindow::displayTimer(int _val)
{
    if (mPlay) {
        mPlayFrame++;
        if (mPlayFrame >= mMotion->getNumFrames())
            mPlayFrame = 0;
        glutPostRedisplay();
        glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
    }
}

void MyWindow::draw()
{
    glDisable(GL_LIGHTING);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    if (mPlayFrame < mMotion->getNumFrames())
        mSkel->setPose(mMotion->getPoseAtFrame(mPlayFrame), false, false);
    
    mSkel->draw(mRI);
            
    // display the frame count in 2D text
    char buff[64];
    sprintf(buff, "%d", mPlayFrame);
    string frame(buff);
    glColor3f(0.0,0.0,0.0);
    yui::drawStringOnScreen(0.02f,0.02f,frame);
    glEnable(GL_LIGHTING);
}

void MyWindow::keyboard(unsigned char key, int x, int y)
{
    switch(key){
    case ' ': // use space key to play or stop the motion
        mAnalyzer->analyze();
        cout << "See output.txt" << endl;
        break;
    case 'p': // playBack
        mPlay = !mPlay;
        if (mPlay)
            glutTimerFunc( mDisplayTimeout, refreshTimer, 0);
        
        break;
    case '[': // step backward
        mPlayFrame--;
        if(mPlayFrame < 0)
            mPlayFrame = 0;
        glutPostRedisplay();
        
        break;
    case ']': // step forwardward
        mPlayFrame++;
        if(mPlayFrame >= mMotion->getNumFrames())
            mPlayFrame = 0;
        glutPostRedisplay();
        
        break;
    default:
        Win3D::keyboard(key,x,y);
    }
    glutPostRedisplay();
}
