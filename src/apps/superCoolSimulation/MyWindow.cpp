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
	mSimFrame = 0;
	mPlayFrame = 0;

	//Camera settings
	mPersp = 45.0f;
	mTrans[2] = -20.0f;
	mZoom = 0.7f;
	mSim = false;
	mPlay = false;
	mDrop = false;

	//External Forces
	mForce = Eigen::Vector3d::Zero();
	mForce2 = Eigen::Vector3d::Zero();
	
	//setup skels vector
	mSkels.resize(0);
	mSkels.push_back(_m);
	mSkels.push_back(_m2);

	//Environment Settings
	mGravity = /*Eigen::Vector3d::Zero();/**/ Vector3d(0.0,-9.8,0.0);
	mTimeStep = 1.0/1000.0;

	int sumNDofs = 0;
    mIndices.push_back(sumNDofs);
    for (unsigned int i = 0; i < mSkels.size(); i++) {
        int nDofs = mSkels[i]->getNumDofs();
        sumNDofs += nDofs;
        mIndices.push_back(sumNDofs);
    }

	//Controller
	mController = new Controller(mModel->getNumDofs());

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

	//Set spring forces 
	for (unsigned int i = 0; i < mModel->getNumDofs(); i++) {
        mController->setDesiredDof(i, mDofs[0][i]);
    }

	// Initialize Ground
	mDofs[1].resize(mModel2->getNumDofs());
	mDofVels[1].resize(mDofs[0].size());
	mDofs[1].setZero();
	mDofVels[1].setZero();

	// translate Ground
	mDofs[1][1] = -0.3;

  // Init Dynamics
  mSkels[0]->initDynamics();
  mSkels[1]->initDynamics();

	// Set Poses and Other Variables
	mSkels[0]->setPose(mDofs[0],false,false);
	mSkels[0]->computeDynamics(mGravity,mDofVels[0],false);

	mSkels[1]->setPose(mDofs[1],false,false);
	mSkels[1]->computeDynamics(mGravity,mDofVels[1],false);
	mSkels[1]->setKinematicState(true);

	if (mModel!=mSkels[0] || mModel2!=mSkels[1])
	{
		fprintf(stderr, "Models do not match mSkels.\nm1=%x mSkels[0]=%x.\nm2=%x mSkels[1]=%x.\n",
			mModel, mSkels[0], mModel2, mSkels[1]);
	}


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
	VectorXd qddot;
    if(!mDrop){//includes torque only if controller is active
		qddot = mSkels[0]->getMassMatrix().fullPivHouseholderQr().solve( -mSkels[0]->getCombinedVector() 
		+ mSkels[0]->getExternalForces() 
		+ mCollisionHandle->getConstraintForce(0))  
		+ mController->getTorques();
	}
	else{
		qddot = mSkels[0]->getMassMatrix().fullPivHouseholderQr().solve( -mSkels[0]->getCombinedVector() 
		+ mSkels[0]->getExternalForces() 
		+ mCollisionHandle->getConstraintForce(0));
	}
    mSkels[0]->clampRotation(mDofs[0], mDofVels[0]);
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
    mSkels[0]->setPose(mDofs[0],true,true);
    mSkels[0]->computeDynamics(mGravity, mDofVels[0], true);

	mSkels[1]->setPose(mDofs[1],true,false);
	
	mCollisionHandle->applyContactForces();
}

void MyWindow::displayTimer(int _val)
{
	int numIter = mDisplayTimeout / (mTimeStep*1000);
	if (mSim)
	{
		static Timer tSim("Super Cool Simulation");
		for(int i=0; i<numIter; i++){
			tSim.startTimer();
			if(!mDrop){mController->computeTorques(mDofs[0], mDofVels[0]);}
			static_cast<BodyNodeDynamics*>(mSkels[0]->getNode(4))->addExtForce(Vector3d(0.0, -0.1, 0.0), mForce);
			static_cast<BodyNodeDynamics*>(mSkels[0]->getNode(7))->addExtForce(Vector3d(0.0, 0.1, 0.0), mForce2);
			setPose();
			mIntegrator.integrate(this, mTimeStep);
			tSim.stopTimer();
			bake();
		}
		mSimFrame += numIter;  
		mForce = Vector3d::Zero();
		mForce2 = Vector3d::Zero();
		glutPostRedisplay();
		glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
	}
	else if(mPlay){
		mPlayFrame = mPlayFrame + 8;
		if (mPlayFrame >= mBakedStates.size())
			mPlayFrame = 0;
		glutPostRedisplay();
		glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
	}
}

void MyWindow::draw()
{

    glDisable(GL_LIGHTING);

    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	if(!mSim) {//During playback, displays baked states
        if (mPlayFrame < mBakedStates.size()) {
            for (unsigned int i = 0; i < mSkels.size(); i++) {
                int start = mIndices[i];
                int size = mDofs[i].size();
                mSkels[i]->setPose(mBakedStates[mPlayFrame].segment(start, size), false, false);
            }
		}
	}
	mSkels[0]->draw(mRI);
	mSkels[1]->draw(mRI);
	if(mSim){//display markers when simulating only
		for (int k = 0; k < mCollisionHandle->getCollisionChecker()->getNumContact(); k++) {
			Vector3d  v = mCollisionHandle->getCollisionChecker()->getContact(k).point;
			Vector3d n = mCollisionHandle->getCollisionChecker()->getContact(k).normal / 10.0;
		
			glBegin(GL_LINES);
			glVertex3f(v[0], v[1], v[2]);
			glVertex3f(v[0] + n[0], v[1] + n[1], v[2] + n[2]);
			glEnd();
			mRI->setPenColor(Vector3d(0.2, 0.2, 0.8));
			mRI->pushMatrix();
			glTranslated(v[0], v[1], v[2]);
			mRI->drawEllipsoid(Vector3d(0.02, 0.02, 0.02));
			mRI->popMatrix();
		}
	}
	
	//display frame count
	char buff[64];
    if(!mSim)
		sprintf(buff,"%d",mPlayFrame);
	else
		sprintf(buff,"%d",mSimFrame);
    string frame(buff);
    glDisable(GL_LIGHTING);
    glColor3f(0.0,0.0,0.0);
    yui::drawStringOnScreen(0.02f,0.02f,frame);
    glEnable(GL_LIGHTING);
}

void MyWindow::keyboard(unsigned char key, int x, int y)
{
	switch(key){
	case 'd': // drops the jellyfishMan
        mDrop = !mDrop;
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
	case '1': // shin force
        mForce[0] = 0.8;
        cout << "force on shin" << endl;
        break;
	case '2': // arm force
        mForce2[0] = -0.8;
        cout << "force on arm" << endl;
        break;
	default:
        Win3D::keyboard(key,x,y);
    }
    glutPostRedisplay();
}


void MyWindow::bake()
{
    int nContact = mCollisionHandle->getCollisionChecker()->getNumContact();
    VectorXd state(mIndices.back() + 6 * nContact);
    for (unsigned int i = 0; i < mSkels.size(); i++)
        state.segment(mIndices[i], mDofs[i].size()) = mDofs[i];
    for (int i = 0; i < nContact; i++) {
        int begin = mIndices.back() + i * 6;
        state.segment(begin, 3) = mCollisionHandle->getCollisionChecker()->getContact(i).point;
        state.segment(begin + 3, 3) = mCollisionHandle->getCollisionChecker()->getContact(i).normal;
    }

    mBakedStates.push_back(state);
}
