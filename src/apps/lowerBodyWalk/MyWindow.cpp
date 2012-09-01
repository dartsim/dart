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

	// Allocate the positions and velocities for the dofs of the skeletons
	mDofs.resize(mSkels.size());
	mDofVels.resize(mSkels.size());
	for(unsigned int i = 0; i < mSkels.size(); i++) {
		mDofs[i].resize(mSkels[i]->getNumDofs());
		mDofVels[i].resize(mSkels[i]->getNumDofs());
		printf("Setting dofs: %d\n", mSkels[i]->getNumDofs());
		mDofs[i].setZero();
		mDofVels[i].setZero();
	}

	// Get the position of the skeleton in the beginnning of the motion and initialize the DOFs
	assert((mMotion != NULL) && "The motion is not loaded yet!");
	assert((mMotion->getNumFrames() > 0) && "No frames!");

//	Eigen::VectorXd initialPose (18);
//	double f [] = {0.0250066, 0.475798, 0.0351254, 0.000967281, -2.13853e-06, 9.92338e-05};
//	for(size_t i = 0; i < 6; i++)
//		initialPose(i) = f[i];
//	Eigen::VectorXd temp = mMotion->getPoseAtFrame(0);
//	for(size_t i = 0; i < 12; i++)
//		initialPose(i+6) = temp(i);
//	mDofs[1] = initialPose;
	mDofs[1] = mMotion->getPoseAtFrame(0);

	// Set the position of the ground
	mDofs[0][1] = -0.35;

	// Initiate the dynamics of the skeletons given the first pose
	for(unsigned int i = 0; i < mSkels.size(); i++) {
		mSkels[i]->initDynamics();
		mSkels[i]->setPose(mDofs[i], false, false);
		mSkels[i]->computeDynamics(mGravity, mDofVels[i], false);
	}

	// Set Ground to be immobile
	mSkels[0]->setImmobileState(true);

	// Initialize the collision detector & handler
	mCollisionHandle = new dynamics::ContactDynamics(mSkels, mTimeStep);

	// Initialize the controller
	int nDof = mSkels[1]->getNumDofs();
	mController = new Controller(mMotion, mSkels[1], mTimeStep);
	for(int i = 0; i < nDof; i++)
		mController->setDesiredDof(i, mController->getSkel()->getDof(i)->getValue());
//	mController = new Controller(mMotion, mSkels[1], mTimeStep);
//	for(int i = 6; i < mSkels[1]->getNumDofs(); i++)
//		mController->setDesiredDof(i - 6, mController->getSkel()->getDof(i)->getValue());

// Initialize some constants
	dDOF = 0.1;

	rightFoot = mSkels[1]->getNode("RAR");
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

	static const bool debug = false;

	setPose();
	VectorXd deriv = VectorXd::Zero(mIndices.back() * 2);
	for(unsigned int i = 0; i < mSkels.size(); i++) {
		if(mSkels[i]->getImmobileState()) continue;
		int start = mIndices[i] * 2;
		int size = mDofs[i].size();
		VectorXd qddot = mSkels[i]->getMassMatrix().fullPivHouseholderQr().solve(
				-mSkels[i]->getCombinedVector() + mSkels[i]->getExternalForces() + mSkels[i]->getInternalForces()
						+ mCollisionHandle->getConstraintForce(i));

//		VectorXd temp = mSkels[i]->getInternalForces();
//		VectorXd internalForces (16);
//		for(size_t j = 0; j < 6; j++) internalForces(j) = 0;
//		for(size_t j = 6; j < 18; j++) internalForces(j) = temp(j-6);
//		VectorXd qddot = mSkels[i]->getMassMatrix().fullPivHouseholderQr().solve(
//				-mSkels[i]->getCombinedVector() + mSkels[i]->getExternalForces() + internalForces
//						+ mCollisionHandle->getConstraintForce(i));

//		cout << "\n\n========================================================" << endl;
//		cout << "qddot: " << qddot.transpose() << endl;

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
		}
	}
	mCollisionHandle->applyContactForces();
}

/**
 * @function displayTimer
 */
void MyWindow::displayTimer(int _val) {
	int numIter = mDisplayTimeout / (mTimeStep * 1000);
	numIter = 5;
	if(mPlay) {
		mPlayFrame += 16;
		if(mPlayFrame >= mBakedStates.size()) mPlayFrame = 0;
		glutPostRedisplay();
		glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
    takeScreenshot();
	}
	else if(mSim) {
		for(int i = 0; i < numIter; i++) {

			// cout << "###iter = " << i + mSimFrame << endl;

			// Add a force to lowerBody - mSkels[1] (in this case zero force)
			static_cast <BodyNodeDynamics*>(mSkels[1]->getNode(0))->addExtForce(Vector3d(0.0, 0.0, 0.0), mForce);

			// Add the controller forces
			Eigen::VectorXd xdot(6);
			xdot << 0.1, -0.1, 0.0, 0.0, 0.0, 0.0;		// lean one side
			bool onlyRightLeg = false;
			if(mSimFrame > 450) {
				xdot << 0.1, -0.1, 0.1, -0.2, 0.1, 0.0;
				onlyRightLeg = true;
			}

			bool moveCondition1 = (mSimFrame > 20) && (mSimFrame < 200);
			bool moveCondition2 = (mSimFrame > 420) && (mSimFrame < 600);
			bool moveCondition3 = (mSimFrame > 820) && (mSimFrame < 1000);
			bool moveCondition4 = (mSimFrame > 1220) && (mSimFrame < 1400);
//			if(moveCondition1 || moveCondition2 || moveCondition3 || moveCondition4) {
			if(mSimFrame>150) {

				Eigen::MatrixXd rightJacobianLin = rightFoot->getJacobianLinear();
				Eigen::MatrixXd rightJacobianAng = rightFoot->getJacobianAngular();
				cout << "rightJacobianLin: \n" << rightJacobianLin << endl;
				cout << "rightJacobianAng: \n" << rightJacobianAng << endl;

				size_t temp = 6;
				Eigen::MatrixXd rightJacobian(6, temp);
				rightJacobian.block(0, 0, 3, temp) = rightJacobianLin.block(0, 0, 3, temp);
				rightJacobian.block(3, 0, 3, temp) = rightJacobianAng.block(0, 0, 3, temp);
				Eigen::MatrixXd rightJacobianTrans = rightJacobian.transpose();
				Eigen::MatrixXd rightJacobianInv = rightJacobianTrans * (rightJacobian * rightJacobianTrans).inverse();
				Eigen::VectorXd rightDofs = rightJacobianInv * xdot;
				size_t f = 18;
				Eigen::VectorXd dofs = Eigen::VectorXd::Zero(f, 1);
				for(size_t dof_idx = f - 6; dof_idx < f; dof_idx++) {
					dofs(dof_idx) = rightDofs(dof_idx - (f - 6));
					if(!onlyRightLeg) dofs(dof_idx - 6) = rightDofs(dof_idx - (f - 6));
				}
				cout << "\ndofs:: " << dofs.transpose() << endl << endl;
				//Eigen::VectorXd dofs = Eigen::VectorXd::Zero(12,1);
//				dofs(6) = 1.0;
				//printf("changing..\n");
				mController->computeTorques(mDofs[1], mDofVels[1], &dofs);
			}
			else
				mController->computeTorques(mDofs[1], mDofVels[1]);

			mSkels[1]->setInternalForces(mController->getTorques());

//			Eigen::VectorXd dofs (12);
//			for(size_t j = 0; j < 12; j++) dofs(j) = mDofs[1][j+6];
//			Eigen::VectorXd dofVels (12);
//			for(size_t j = 0; j < 12; j++) dofVels(j) = mDofVels[1][j+6];
//			mController->computeTorques(dofs, dofVels);
//
//			Eigen::VectorXd temp = mController->getTorques();
//			Eigen::VectorXd torques (18);
//			for(size_t j = 0; j < 6; j++) torques(j) = 0;
//			for(size_t j = 6; j < 18; j++) torques(j) = temp(j-6);
//			mSkels[1]->setInternalForces(torques);

			// Integrate stuff
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

bool MyWindow::takeScreenshot() {
	printf("Screenshot entered \n");
	static int count = 0;
	char fileBase[32] = "frames/";
	char fileName[64];
	// png
	sprintf(fileName, "%.4d.png", count++);
	int tw = glutGet(GLUT_WINDOW_WIDTH);
	int th = glutGet(GLUT_WINDOW_HEIGHT);
	bool antiAlias = true;
	if(yui::screenShot(FIF_PNG, tw, th, fileName, antiAlias)) {
		cout << fileName << " generated\n";
		return true;
	}
	//sprintf(fileName, "%s%.4d.tga", fileBase, count++);
	//screenShot(tw, th, fileName, antiAlias);
	return false;
}

/**
 * @function draw
 */
void MyWindow::draw() {
	glDisable (GL_LIGHTING);

	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	// Print the entire vector
//	Eigen::VectorXd pose;
//	mSkels[1]->getPose(pose);
//	cout << pose.transpose() << endl;

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
//				if(nContact > 0) exit(0);
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
			int nContacts = mCollisionHandle->getCollisionChecker()->getNumContact();
//			if(nContacts > 0) exit(0);
			for(int k = 0; k < nContacts; k++) {
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
		}
			break;

		case '4': {  // Set fourth joint left leg
			Eigen::VectorXd pose;
			mSkels[1]->getPose(pose);
			// Increase DOF value (DOF 3)
			pose(6) = pose(6) + (inverse ? -dDOF : dDOF);
			mSkels[1]->setPose(pose);
			std::cout << "Updated pose DOF 3: " << pose.transpose() << std::endl;
			glutPostRedisplay();
			if(inverse) inverse = false;
		}
			break;

		case '5': {  // Set fifth joint left leg
			Eigen::VectorXd pose;
			mSkels[1]->getPose(pose);
			// Increase DOF value (DOF 4)
			pose(7) = pose(7) + (inverse ? -dDOF : dDOF);
			mSkels[1]->setPose(pose);
			std::cout << "Updated pose DOF 4: " << pose.transpose() << std::endl;
			glutPostRedisplay();
			if(inverse) inverse = false;
		}
			break;

		case '6': {  // Set sixth joint left leg
			Eigen::VectorXd pose;
			mSkels[1]->getPose(pose);
			// Increase DOF value (DOF 5)
			pose(8) = pose(8) + (inverse ? -dDOF : dDOF);
			mSkels[1]->setPose(pose);
			std::cout << "Updated pose DOF 5: " << pose.transpose() << std::endl;
			glutPostRedisplay();
			if(inverse) inverse = false;
		}
			break;

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
