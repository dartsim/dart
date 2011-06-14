#include "MyWindow.h"
#include "YUI/glFuncs.h"
// #include <GLUT/glut.h>
#include "utils/LoadOpengl.h"
#include "model3d/Skeleton.h"
#include <cstdio>

using namespace std;
using namespace Eigen;
using namespace model3d;

void MyWindow::displayTimer(int val)
{
    mFrame++;
    if(mFrame == mMaxFrame){
        mFrame = 0;
        mPlaying = false;
    }
    glutPostRedisplay();
    if(mPlaying)	
        glutTimerFunc(mDisplayTimeout, refreshTimer, val);
}

void MyWindow::computeMax()
{
    mMaxFrame = mMotion.getNumFrames();
}

void MyWindow::draw()
{
    Skeleton* model = mMotion.getModel();
    int nFrames = mMotion.getNumFrames();
    int fr = mFrame;
    if(fr>=nFrames) fr = nFrames-1;
    model->setPose(mMotion.mDofs.at(fr));
    model->draw(&mRenderer, Vector4d(0.0,0.0,0.0,1.0),false);
    if(bMarker)
        model->drawHandles(&mRenderer, Vector4d(1.0,0.0,0.0,1.0),false);

    if(bShowProgress) drawProgressBar(fr,mMaxFrame);

    char buff[64];
    sprintf(buff,"%d/%d",mFrame,mMaxFrame);
    string frame(buff);
    glDisable(GL_LIGHTING);
    glColor3f(0.0,0.0,0.0);
    drawStringOnScreen(0.02f,0.02f,frame);
    glEnable(GL_LIGHTING);
}

void MyWindow::keyboard(unsigned char key, int x, int y)
{
    switch(key){
    case ' ':
        mPlaying = !mPlaying;
        if(mPlaying)
            glutTimerFunc( mDisplayTimeout, refreshTimer, 0);
        break;
    case 'r':
        mFrame = 0;
        break;
    case 'a':
        if(mFrame<mMaxFrame-1)
            mFrame ++;
        break;
    case 'd':
        if(mFrame>0)
            mFrame --;
        break;
    case 'm':
        bMarker = !bMarker;
        break;
    default:
        Win3D::keyboard(key,x,y);
    }
    glutPostRedisplay();
}

void MyWindow::move(int x, int y)
{
    double Xport = (double)x/mWinWidth;
    double Yport = (double)(mWinHeight-y)/mWinHeight;

    bool oldShow = bShowProgress;

    if(!mRotate && !mTranslate && !mZooming){
        if(Xport>=0.15 && Xport<=0.85 && Yport>=0.02 && Yport<=0.08)
            bShowProgress = true;
        else if(bShowProgress) bShowProgress = false;
    }

    if( oldShow != bShowProgress)
        glutPostRedisplay();
}
