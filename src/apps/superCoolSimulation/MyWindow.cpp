#include "MyWindow.h"
#include "dynamics/BodyNodeDynamics.h"
#include "dynamics/ContactDynamics.h"
#include "kinematics/Dof.h"
#include "utils/UtilsMath.h"
#include "utils/Timer.h"
#include "yui/GLFuncs.h"
#include <cstdio>
#include "kinematics/BodyNode.h"

using namespace std;
using namespace Eigen;
using namespace kinematics;
using namespace utils;
using namespace integration;
using namespace dynamics;

//Constructor
MyWindow::MyWindow(dynamics::SkeletonDynamics* _m,dynamics::SkeletonDynamics* _m2)
	: Win3D(), mModel(_m), mModel2(_m2)
{
	//Background Color
	mBackground[0] = 1.0;
	mBackground[1] = 1.0;
	mBackground[2] = 1.0;
	mBackground[3] = 1.0;
	mFrame = 0;
	//Camera settings
	mPersp = 45.0f;
	mTrans[2] = -20.0f;
	mZoom = 0.5f;
	mSim = false;
	mPlay = false;

	//setup skels vector
	mSkels.resize(0);
	mSkels.push_back(_m);
	mSkels.push_back(_m2);

	//Environment Settings
	mGravity = Vector3d(0.0,-9.8,0.0);
	mTimeStep = 1.0/1000.0;
	initDyn();
}

void MyWindow::initDyn()
{
	// initialize degrees of freedom
	mDofs.resize(2);
	mDofVels.resize(2);

	// Initialize JellyMan
	mDofs[0].resize(mModel->getNumDofs());
	mDofVels[0].resize(mDofs[0].size());
	mDofs[0].setZero();
	mDofVels[0].setZero();

	// translate JellyMan
	mDofs[0][1] = 0.4;
	mDofs[0][7] = 0.5;
	mDofs[0][8] = 0.5;
	mDofs[0][9] = 0.5;
	mDofs[0][15] = 0.6;
	mDofs[0][16] = -0.32;
	mDofs[0][17] = 0.815;
	mDofs[0][18] = 0.74;
	mDofs[0][26] = -0.32;
	mDofs[0][27] = 0.815;
	mDofs[0][28] = 0.74;

	// Initialize Ground
	mDofs[1].resize(mModel2->getNumDofs());
	mDofVels[1].resize(mDofs[0].size());
	mDofs[1].setZero();
	mDofVels[1].setZero();

	// translate Ground
	mDofs[1][1] = -0.3;

	// Set Poses and Other Variables
	mModel->setPose(mDofs[0],false,false);
	mModel->computeDynamics(mGravity,mDofVels[0],false);

	mModel2->setPose(mDofs[1],false,false);
	mModel2->setKinematicState(true);

	//Initialize collision handling 
	mCollisionHandle = new dynamics::ContactDynamics(mSkels,mTimeStep);
	
}

VectorXd MyWindow::getState() {
	int stateSize = mDofs[0].size() + mDofs[1].size() + 
		mDofVels[0].size() + mDofVels[1].size();
    VectorXd state(stateSize);

	int offset = 0;
	state.head(mDofs[0].size()) = mDofs[0];
	
	offset += mDofs[0].size();
	state.segment(offset,mDofVels[0].size()) = mDofVels[0];

	offset += mDofVels[0].size();
	state.segment(offset,mDofs[1].size()) = mDofs[1];

	offset += mDofs[1].size();
	state.segment(offset,mDofVels[1].size()) = mDofVels[1];

    return state;
}

VectorXd MyWindow::evalDeriv() {
	int stateSize = mDofs[0].size() + mDofs[1].size() + 
		mDofVels[0].size() + mDofVels[1].size();

    VectorXd deriv = VectorXd::Zero(stateSize);

    VectorXd qddot = mModel->getMassMatrix().fullPivHouseholderQr().solve( -mModel->getCombinedVector() 
		+ mModel->getExternalForces()
		+ mCollisionHandle->getConstraintForce(0) ); 
    mModel->clampRotation(mDofs[0], mDofVels[0]);
	deriv.segment(mDofs[0].size(), mDofVels[0].size()) = qddot; // set qddot (accelerations)
    deriv.head(mDofs[0].size()) = (mDofVels[0] + (qddot * mTimeStep)); // set new velocities
    return deriv;
}

void MyWindow::setState(VectorXd newState) {
	int offset = 0;
	mDofs[0] = newState.head(mDofs[0].size());
	
	offset += mDofs[0].size();
	mDofVels[0] = newState.segment(offset,mDofVels[0].size());

	offset += mDofVels[0].size();
	mDofs[1] = newState.segment(offset,mDofs[1].size());

	offset += mDofs[1].size();
	mDofVels[1] = newState.segment(offset,mDofVels[1].size());

}

void MyWindow::setPose() {
    mModel->setPose(mDofs[0],true,true);
    mModel->computeDynamics(mGravity, mDofVels[0], true);

	mModel2->setPose(mDofs[1],true,false);
	
	mCollisionHandle->applyContactForces();
}

void MyWindow::displayTimer(int _val)
{
	if (mSim)
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
		glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
	}
}

void MyWindow::draw()
{
	mModel->draw(mRI);
	mModel2->draw(mRI);
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
	case 's': // simulate one frame
		setPose();
		mIntegrator.integrate(this, mTimeStep);
		mFrame++;
		break;
    case ' ': // use space key to play or stop the motion
        mSim = !mSim;
        if (mSim) {
			mPlay = false;
            glutTimerFunc( mDisplayTimeout, refreshTimer, 0);
        }
        break;	
	case 'p': // playBack
        mPlay = !mPlay;
        if (mPlay) {
            mSim = false;
            glutTimerFunc( mDisplayTimeout, refreshTimer, 0);
        }
        break;
	default:
        Win3D::keyboard(key,x,y);
    }
    glutPostRedisplay();
}


