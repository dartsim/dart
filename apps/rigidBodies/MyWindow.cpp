#include "MyWindow.h"
#include "RigidBody.h"
#include "CollisionInterface.h"
#include "dart/gui/GLFuncs.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/constraint/ConstraintSolver.h"
#include <cstdio>
#include <iostream>

using namespace Eigen;

void MyWindow::displayTimer(int _val) {
  if (mPlaying) {
    mPlayFrame += mDisplayTimeout;
    if (mPlayFrame >= mBakedStates.size())
      mPlayFrame = 0;
  }else if (mSimulating) {
    int numIter = mDisplayTimeout / (mWorld->getTimeStep() * 1000);
    if (numIter < 1.0)
      numIter = 1;
    for (int i = 0; i < numIter; i++) {
      mWorld->simulate();
      bake(); // Store the simulated state for playback
    }
  }
  glutPostRedisplay();
  glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
}

void MyWindow::draw() {
  if (!mSimulating) { // Playback mode
    if (mPlayFrame < mBakedStates.size()) {
      // Load pinata state
      int pinataSize = mWorld->getPinataWorld()->getSkeleton(0)->getNumDofs();
      mWorld->getPinataWorld()->getSkeleton(0)->setPositions(mBakedStates[mPlayFrame].segment(0, pinataSize));
      mWorld->getPinataWorld()->getSkeleton(0)->computeForwardKinematics(true, false, false);
      // Load rigid body states
      int numRigids = mWorld->getNumRigidBodies();
      if (numRigids < 1)
        return;
      int configSize = mWorld->getRigidBody(0)->getConfigSize();
      for (int i = 0; i < numRigids; i++) { 
        mWorld->getRigidBody(i)->mPosition = mBakedStates[mPlayFrame].segment(pinataSize + i * configSize, 3);
        for (int j = 0; j < configSize - 3; j++)
          *(mWorld->getRigidBody(i)->mOrientation.data() + j) = mBakedStates[mPlayFrame][pinataSize + i * configSize + 3 + j];
      }      
      if (mShowMarkers) {
        // Load and draw contacts
        int sumDofs = pinataSize + mWorld->getNumRigidBodies() * configSize;
        int nContact = (mBakedStates[mPlayFrame].size() - sumDofs) / 6;
        for (int i = 0; i < nContact; i++) {
          Vector3d v = mBakedStates[mPlayFrame].segment(sumDofs + i * 6, 3);
          Vector3d n = mBakedStates[mPlayFrame].segment(sumDofs + i * 6 + 3, 3) / 4.0;
          glBegin(GL_LINES);
          glVertex3f(v[0], v[1], v[2]);
          glVertex3f(v[0] + n[0], v[1] + n[1], v[2] + n[2]);
          glEnd();
          mRI->setPenColor(Vector3d(0.2, 0.2, 0.8));
          mRI->pushMatrix();
          mRI->translate(v);
          mRI->drawEllipsoid(Vector3d(0.02, 0.02, 0.02));
          mRI->popMatrix();
        }
      }
    }
  }else{ // Simulation mode
    if (mShowMarkers) {
      for (int k = 0; k < mWorld->getCollisionDetector()->getNumContacts(); k++) {
        Vector3d  v = mWorld->getCollisionDetector()->getContact(k).point;
        Vector3d n = mWorld->getCollisionDetector()->getContact(k).normal / 4.0;
        glBegin(GL_LINES);
        glVertex3f(v[0], v[1], v[2]);
        glVertex3f(v[0] + n[0], v[1] + n[1], v[2] + n[2]);
        glEnd();
        mRI->setPenColor(Vector3d(0.2, 0.2, 0.8));
        mRI->pushMatrix();
        mRI->translate(v);
        mRI->drawEllipsoid(Vector3d(0.02, 0.02, 0.02));
        mRI->popMatrix();
      }
    }
  }

  // Draw rigid bodies
  glEnable(GL_LIGHTING);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  for (int i = 0; i < mWorld->getNumRigidBodies(); i++)
    mWorld->getRigidBody(i)->draw(mRI);

  // Draw the pinata
  glDisable(GL_LIGHTING);
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  mWorld->getPinataWorld()->getSkeleton(0)->draw(mRI);

  // Display the frame count in 2D text
  char buff[64];
  if (!mSimulating)
    sprintf(buff, "%d", mPlayFrame);
  else
    sprintf(buff, "%d", mWorld->getSimFrames());
  std::string frame(buff);
  glDisable(GL_LIGHTING);
  glColor3f(0.0,0.0,0.0);
  dart::gui::drawStringOnScreen(0.02f, 0.02f, frame);
  glEnable(GL_LIGHTING);
}


void MyWindow::keyboard(unsigned char key, int x, int y) 
{
  switch(key) {        
  case ' ': // Use space key to play or stop the motion
    mSimulating = !mSimulating;
    if(mSimulating) {
      mPlaying = false;
      glutTimerFunc( mDisplayTimeout, refreshTimer, 0);
    }
    break;
        
  case 'p': // PlayBack
    mPlaying = !mPlaying;
    if (mPlaying) {
      mSimulating = false;
      glutTimerFunc( mDisplayTimeout, refreshTimer, 0);
    }
    break;
  case '[': // Step backward
    if (!mSimulating) {
      mPlayFrame--;
      if(mPlayFrame < 0)
        mPlayFrame = 0;
      glutPostRedisplay();
    }
    break;
  case ']': // Step forwardward
    if (!mSimulating) {
      mPlayFrame++;
      if(mPlayFrame >= mBakedStates.size())
        mPlayFrame = 0;
      glutPostRedisplay();
    }
    break;

  case 'v': // Show or hide contacts
    mShowMarkers = !mShowMarkers;
    break;
    
  case '1': // Hit the pinata from the left
    mWorld->setExtForce(0, 500.0);
    break;

  case '2': // Hit the pinata from the right
    mWorld->setExtForce(0, -500.0);
    break;

  case '3': // Hit the pinata from the front
    mWorld->setExtForce(2, -500.0);
    break;

  case '4': // Hit the pinata from the back
    mWorld->setExtForce(2, 500.0);
    break;

  // case 'b': // Break the pinata
  //   mWorld->getPinataWorld()->getConstraintSolver()->removeAllConstraints();
    
  default:
    Win3D::keyboard(key,x,y);
  }
  glutPostRedisplay();
}


void MyWindow::bake()
{
  int nRigidBody = mWorld->getNumRigidBodies();
  if (nRigidBody < 1)
    return;
  int configSize = mWorld->getRigidBody(0)->getConfigSize();
  int nContact = mWorld->getCollisionDetector()->getNumContacts();
  int pinataSize = mWorld->getPinataWorld()->getSkeleton(0)->getNumDofs();
  VectorXd state(pinataSize + configSize * nRigidBody + 6 * nContact);

  // Record pinata state
  for (int i = 0; i < pinataSize; i++) 
    state(i) = mWorld->getPinataWorld()->getSkeleton(0)->getPosition(i);
  // Record rigid body states
  for (int i = 0; i < nRigidBody; i++) {
    state.segment(pinataSize + i * configSize, 3) = mWorld->getRigidBody(i)->mPosition;
    for (int j = 0; j < configSize - 3; j++)
      state(pinataSize + i * configSize + 3 + j) = *(mWorld->getRigidBody(i)->mOrientation.data() + j);
  }
  // Record contact info 
  for (int i = 0; i < nContact; i++) {
    int begin = pinataSize + configSize * nRigidBody + i * 6;
    state.segment(begin, 3) = mWorld->getCollisionDetector()->getContact(i).point;
    state.segment(begin + 3, 3) = mWorld->getCollisionDetector()->getContact(i).normal;
  }  
  mBakedStates.push_back(state);
}
