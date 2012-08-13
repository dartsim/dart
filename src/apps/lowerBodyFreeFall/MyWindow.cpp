/**
  * @file MyWindow.cpp
  * @author A. Huaman and C. Erdogan
  * @date Sunday, August 13th, 2012
  */

#include "MyWindow.h"
#include "dynamics/BodyNodeDynamics.h"
#include "dynamics/ContactDynamics.h"
#include "kinematics/Dof.h"
#include "utils/UtilsMath.h"
#include "utils/Timer.h"
#include "yui/GLFuncs.h"
#include <cstdio>
#include "kinematics/BodyNode.h"

using namespace Eigen;
using namespace kinematics;
using namespace utils;
using namespace integration;
using namespace dynamics;

/**
  * @function initDyn
  */
void MyWindow::initDyn() {
	dDOF = 0.1;
	mDofs.resize(mSkels.size());
	mDofVels.resize(mSkels.size());

	for(unsigned int i = 0; i < mSkels.size(); i++) {
		mDofs[i].resize(mSkels[i]->getNumDofs());
		mDofVels[i].resize(mSkels[i]->getNumDofs());
		mDofs[i].setZero();
		mDofVels[i].setZero();
	}

	// Initialize DOFs
  
   //-- Initialize Ground Location
   mDofs[0][1] = -0.35;
   //-- Initialize LowerBodyLocation
   mDofs[1][0] = 0.025;
   mDofs[1][1] = 1.0;
   mDofs[1][2] = 0.035;

   mDofs[1][3] = 0.3;
   mDofs[1][4] = 0.3;
   mDofs[1][5] = 0.3;
   mDofs[1][6] = 0.3;
   mDofs[1][7] = 0.3;
   mDofs[1][8] = 0.3;

	for(unsigned int i = 0; i < mSkels.size(); i++) {
		mSkels[i]->initDynamics();
		mSkels[i]->setPose(mDofs[i], false, false);
		mSkels[i]->computeDynamics(mGravity, mDofVels[i], false);
	}
  //-- Set Ground to be immobile
	mSkels[0]->setImmobileState(true);

  // Set Joint limit dynamics for the lowerBody (Skel[1])
  mJointLimitConstr = new JointLimitDynamics( mSkels[1], mTimeStep );

  // Set collision handling for all the skels (Ground and LowerBody)
	mCollisionHandle = new dynamics::ContactDynamics(mSkels, mTimeStep);
}

/**
  * @function getState
  */
VectorXd MyWindow::getState() {
	VectorXd state(mIndices.back() * 2);
	for(unsigned int i = 0; i < mSkels.size(); i++) {
		int start = mIndices[i] * 2;
		int size = mDofs[i].size();
		state.segment(start, size) = mDofs[i];
		state.segment(start + size, size) = mDofVels[i];
	}
	return state;
}

/**
  * @function evalDeriv
  */
VectorXd MyWindow::evalDeriv() {
	setPose();
	VectorXd deriv = VectorXd::Zero(mIndices.back() * 2);
	for(unsigned int i = 0; i < mSkels.size(); i++) {
		if(mSkels[i]->getImmobileState()) continue;
		int start = mIndices[i] * 2;
		int size = mDofs[i].size();
		VectorXd qddot = mSkels[i]->getMassMatrix().fullPivHouseholderQr().solve(
				-mSkels[i]->getCombinedVector() + mSkels[i]->getExternalForces() + mCollisionHandle->getConstraintForce(i) + mJointLimitConstr->getConstraintForce() );
		mSkels[i]->clampRotation(mDofs[i], mDofVels[i]);
		deriv.segment(start, size) = mDofVels[i] + (qddot * mTimeStep);  // set velocities
		deriv.segment(start + size, size) = qddot;  // set qddot (accelerations)
	}
	return deriv;
}

/**
  * @function setState
  */
void MyWindow::setState(VectorXd newState) {
	for(unsigned int i = 0; i < mSkels.size(); i++) {
		int start = mIndices[i] * 2;
		int size = mDofs[i].size();
		mDofs[i] = newState.segment(start, size);
		mDofVels[i] = newState.segment(start + size, size);
	}
}

/**
  * @function setPose
  */
void MyWindow::setPose() {
	for(unsigned int i = 0; i < mSkels.size(); i++) {
		if(mSkels[i]->getImmobileState()) {
			mSkels[i]->setPose(mDofs[i], true, false);
		}
		else {
			mSkels[i]->setPose(mDofs[i], true, true);
			mSkels[i]->computeDynamics(mGravity, mDofVels[i], true);
       mJointLimitConstr->applyJointLimitTorques();
		}
	}
	mCollisionHandle->applyContactForces();
}

/**
  * @function displayTimer
  */
void MyWindow::displayTimer(int _val) {
	int numIter = mDisplayTimeout / (mTimeStep * 1000);
	numIter = 15;
	if(mPlay) {
		mPlayFrame += 16;
		if(mPlayFrame >= mBakedStates.size()) mPlayFrame = 0;
		glutPostRedisplay();
		glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
	}
	else if(mSim) {
		for(int i = 0; i < numIter; i++) {
            cout << " ** iter = " << i + mSimFrame << endl;
     // Add a force to lowerBody - mSkels[1] (in this case zero force)
			static_cast <BodyNodeDynamics*>(mSkels[1]->getNode(0))->addExtForce(Vector3d(0.0, 0.0, 0.0), mForce);
			mIntegrator.integrate(this, mTimeStep);
			bake();
		}

		mImpulseDuration--;
		if(mImpulseDuration <= 0) {
			mImpulseDuration = 0;
			mForce.setZero();
		}

		mSimFrame += numIter;

		glutPostRedisplay();
		glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
	}
}

/**
  * @function draw
  */
void MyWindow::draw() {
	glDisable (GL_LIGHTING);

	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	if(!mSim) {
		if(mPlayFrame < mBakedStates.size()) {
			for(unsigned int i = 0; i < mSkels.size(); i++) {
				int start = mIndices[i];
				int size = mDofs[i].size();
				mSkels[i]->setPose(mBakedStates[mPlayFrame].segment(start, size), false, false);
			}
			if(mShowMarkers) {
				int sumDofs = mIndices[mSkels.size()];
				int nContact = (mBakedStates[mPlayFrame].size() - sumDofs) / 6;
				for(int i = 0; i < nContact; i++) {
					Vector3d v = mBakedStates[mPlayFrame].segment(sumDofs + i * 6, 3);
					//            Vector3d n = mBakedStates[mPlayFrame].segment(sumDofs + i * 6 + 3, 3) / 10.0;
					Vector3d f = mBakedStates[mPlayFrame].segment(sumDofs + i * 6 + 3, 3) / 10.0;
					glBegin (GL_LINES);
					glVertex3f(v[0], v[1], v[2]);
					glVertex3f(v[0] + f[0], v[1] + f[1], v[2] + f[2]);
					glEnd();
					mRI->setPenColor(Vector3d(0.8, 0.2, 0.2));
					mRI->pushMatrix();
					glTranslated(v[0], v[1], v[2]);
					mRI->drawEllipsoid(Vector3d(0.02, 0.02, 0.02));
					mRI->popMatrix();
				}
			}
		}
	}
	else {
		if(mShowMarkers) {
			for(int k = 0; k < mCollisionHandle->getCollisionChecker()->getNumContact(); k++) {
				//          if (mCollisionHandle->getCollisionChecker()->getContact(k).bdID1 == mSelectedNode) {
				Vector3d v = mCollisionHandle->getCollisionChecker()->getContact(k).point;
				Vector3d n = mCollisionHandle->getCollisionChecker()->getContact(k).normal / 10.0;
				//                    Vector3d f = mCollisionHandle->getCartesianForce(1, k) / 10.0;
				Vector3d f = mCollisionHandle->getCollisionChecker()->getContact(k).force / 10.0;
				glBegin (GL_LINES);
				glVertex3f(v[0], v[1], v[2]);
				glVertex3f(v[0] + f[0], v[1] + f[1], v[2] + f[2]);

				glEnd();
				mRI->setPenColor(Vector3d(0.2, 0.2, 0.8));
				mRI->pushMatrix();
				glTranslated(v[0], v[1], v[2]);
				mRI->drawEllipsoid(Vector3d(0.02, 0.02, 0.02));
				mRI->popMatrix();
				//}
			}
		}
	}

	for(unsigned int i = 0; i < mSkels.size(); i++)
		mSkels[i]->draw(mRI);

	// display the frame count in 2D text
	char buff[64];
	if(!mSim) sprintf(buff, "%d", mPlayFrame);
	else sprintf(buff, "%d", mSimFrame);
	string frame(buff);
	glDisable(GL_LIGHTING);
	glColor3f(0.0, 0.0, 0.0);
	yui::drawStringOnScreen(0.02f, 0.02f, frame);
	glEnable(GL_LIGHTING);
}

/**
  * @function keyboard
  */
void MyWindow::keyboard(unsigned char key, int x, int y) {
 
//  int mask = glutGetModifiers();
  static bool inverse = false;
	switch(key) {
		case '-':
			inverse = true;
			break;
		case ' ':  // use space key to play or stop the motion
			mSim = !mSim;
			if(mSim) {
				mPlay = false;
				glutTimerFunc(mDisplayTimeout, refreshTimer, 0);
			}
			break;
		case 's':  // simulate one frame
			if(!mPlay) {
				mForce = Vector3d::Zero();
				setPose();
				mIntegrator.integrate(this, mTimeStep);
				mSimFrame++;
				bake();
				glutPostRedisplay();
			}
			break;
			/*
			 case '1': // upper right force
			 mForce[0] = 40;
			 mImpulseDuration = 1.0;
			 cout << "push up and right" << endl;
			 break;
			 case '2': // upper right force
			 mForce[0] = -40;
			 mImpulseDuration = 1.0;
			 cout << "push left" << endl;
			 break;
			 */
		case 'p':  // playBack
			mPlay = !mPlay;
			if(mPlay) {
				mSim = false;
				glutTimerFunc(mDisplayTimeout, refreshTimer, 0);
			}
			break;
		case '[':  // step backward
			if(!mSim) {
				mPlayFrame--;
				if(mPlayFrame < 0) mPlayFrame = 0;
				glutPostRedisplay();
			}
			break;
		case ']':  // step forwardward
			if(!mSim) {
				mPlayFrame++;
				if(mPlayFrame >= mBakedStates.size()) mPlayFrame = 0;
				glutPostRedisplay();
			}
			break;
		case 'v':  // show or hide markers
			mShowMarkers = !mShowMarkers;
			break;

			// TO ADD DOF
		case '1': {  // Set first joint left leg 
			Eigen::VectorXd pose;
			mSkels[1]->getPose(pose);
			// Increase DOF value (DOF 0)
			pose(3) = pose(3) + (inverse ? -dDOF : dDOF);
			mSkels[1]->setPose(pose);
			std::cout << "Updated pose DOF 0: " << pose.transpose() << std::endl;
			glutPostRedisplay();
			if(inverse) inverse = false;
		}
			break;

		case '2': {  // Set second joint left leg
			Eigen::VectorXd pose;
			mSkels[1]->getPose(pose);
			// Increase DOF value (DOF 1)
			pose(4) = pose(4) + (inverse ? -dDOF : dDOF);
			mSkels[1]->setPose(pose);
			std::cout << "Updated pose DOF 1: " << pose.transpose() << std::endl;
			glutPostRedisplay();
			if(inverse) inverse = false;
		}
			break;

		case '3': {  // Set third joint left leg
			Eigen::VectorXd pose;
			mSkels[1]->getPose(pose);
			// Increase DOF value (DOF 2)
			pose(5) = pose(5) + (inverse ? -dDOF : dDOF);
			mSkels[1]->setPose(pose);
			std::cout << "Updated pose DOF 2: " << pose.transpose() << std::endl;
			glutPostRedisplay();
			if(inverse) inverse = false;
		} break;

		case '4': {  // Set fourth joint left leg
			Eigen::VectorXd pose;
			mSkels[1]->getPose(pose);
			// Increase DOF value (DOF 3)
			pose(6) = pose(6) + (inverse ? -dDOF : dDOF);
			mSkels[1]->setPose(pose);
			std::cout << "Updated pose DOF 3: " << pose.transpose() << std::endl;
			glutPostRedisplay();
			if(inverse) inverse = false;
		} break;

		case '5': {  // Set fifth joint left leg
			Eigen::VectorXd pose;
			mSkels[1]->getPose(pose);
			// Increase DOF value (DOF 4)
			pose(7) = pose(7) + (inverse ? -dDOF : dDOF);
			mSkels[1]->setPose(pose);
			std::cout << "Updated pose DOF 4: " << pose.transpose() << std::endl;
			glutPostRedisplay();
			if(inverse) inverse = false;
		} break;

		case '6': {  // Set sixth joint left leg
			Eigen::VectorXd pose;
			mSkels[1]->getPose(pose);
			// Increase DOF value (DOF 5)
			pose(8) = pose(8) + (inverse ? -dDOF : dDOF);
			mSkels[1]->setPose(pose);
			std::cout << "Updated pose DOF 5: " << pose.transpose() << std::endl;
			glutPostRedisplay();
			if(inverse) inverse = false;
		} break;

		default:
			Win3D::keyboard(key, x, y);
	}
	glutPostRedisplay();
}

/**
  * @function bake
  */
void MyWindow::bake() {
	int nContact = mCollisionHandle->getCollisionChecker()->getNumContact();
	VectorXd state(mIndices.back() + 6 * nContact);
	for(unsigned int i = 0; i < mSkels.size(); i++)
		state.segment(mIndices[i], mDofs[i].size()) = mDofs[i];
	for(int i = 0; i < nContact; i++) {
		int begin = mIndices.back() + i * 6;
		state.segment(begin, 3) = mCollisionHandle->getCollisionChecker()->getContact(i).point;
		//state.segment(begin + 3, 3) = mCollisionHandle->getCollisionChecker()->getContact(i).force;
		state.segment(begin + 3, 3) = mCollisionHandle->getCollisionChecker()->getContact(i).normal;
	}
	mBakedStates.push_back(state);
}
