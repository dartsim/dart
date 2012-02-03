#include "MyWindow.h"
#include "dynamics/SkeletonDynamics.h"
#include "utils/UtilsMath.h"
#include "utils/Timer.h"
#include "yui/GLFuncs.h"
#include <cstdio>

using namespace std;
using namespace Eigen;
using namespace kinematics;
using namespace utils;
using namespace integration;

//Constructor
MyWindow::MyWindow(dynamics::SkeletonDynamics* _m)
	: Win3D(), mModel(_m)
{
	//Background Color
	mBackground[0] = 1.0;
	mBackground[1] = 1.0;
	mBackground[2] = 1.0;
	mBackground[3] = 1.0;

	//Camera settings
	mPersp = 45.0f;
	mTrans[1] = 300.0f;

	//Environment Settings
	mGravity = Vector3d(0.0,-9.8,0.0);
	mTimeStep = 1.0/1000.0;
	initDyn();
}

void MyWindow::initDyn()
{
	// initialize degrees of freedom
	mDofs.resize(mModel->getNumDofs());
	mDofVels.resize(mDofs.size());
	for(int i=0; i<mModel->getNumDofs(); i++)
	{
		mDofs[i] = i/10.0 - 0.5;
		mDofVels[i] = 0.1;
	}
	mModel->setPose(mDofs,false,false);
}

VectorXd MyWindow::getState() {
    VectorXd state(mDofs.size() + mDofVels.size());
    state.head(mDofs.size()) = mDofs;
    state.tail(mDofVels.size()) = mDofVels;
    return state;
}

VectorXd MyWindow::evalDeriv() {
    VectorXd deriv(mDofs.size() + mDofVels.size());
    VectorXd qddot = -mModel->getMassMatrix().fullPivHouseholderQr().solve( mModel->getCombinedVector() ); 
    mModel->clampRotation(mDofs, mDofVels);
    deriv.tail(mDofVels.size()) = qddot; // set qddot (accelerations)
    deriv.head(mDofs.size()) = (mDofVels + (qddot * mTimeStep)); // set new velocities
    return deriv;
}

void MyWindow::setState(VectorXd newState) {
    mDofVels = newState.tail(mDofVels.size());
    mDofs = newState.head(mDofs.size());
}

void MyWindow::setPose() {
    mModel->setPose(mDofs,false,false);
    mModel->computeDynamics(mGravity, mDofVels, true);
}

void MyWindow::displayTimer(int _val)
{
    static Timer tSim("Super Cool Simulation");
    int numIter = mDisplayTimeout / (mTimeStep*1000);
    for(int i=0; i<numIter; i++){
        tSim.startTimer();
        setPose();
        mIntegrator.integrate(this, mTimeStep);
        tSim.stopTimer();
    }

    mFrame += numIter;  
	glutPostRedisplay();
	glutTimerFunc(mDisplayTimeout, refreshTimer, 0);
}

void MyWindow::draw()
{
	mModel->draw(mRI);

	//display frame count
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
    case 'r':
        mFrame = 0;
        break;
    default:
        Win3D::keyboard(key,x,y);
    }
    glutPostRedisplay();
}


