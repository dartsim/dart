#include "MyWorld.h"
#include "RigidBody.h"
#include "CollisionInterface.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/EllipsoidShape.h"
#include "dart/dynamics/BoxShape.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Joint.h"
#include "dart/constraint/ConstraintSolver.h"
#include "dart/constraint/WeldJointConstraint.h"

using namespace Eigen;

MyWorld::MyWorld() {
  mFrame = 0;
  mTimeStep = 0.001;
  mGravity = Vector3d(0.0, -9.8, 0.0);
  mForce.setZero();
  // Create a collision detector
  mCollisionDetector = new CollisionInterface();

  // Create and intialize two default rigid bodies (You can add more rigid bodies if you want) 
  RigidBody *rb1 = new RigidBody(dart::dynamics::Shape::BOX, Vector3d(0.05, 0.05, 0.05));
  mCollisionDetector->addRigidBody(rb1, "box"); // Put rb1 in collision detector
  rb1->mPosition[0] = -0.3;
  rb1->mPosition[1] = -0.5;
  rb1->mAngMomentum = Vector3d(0.0, 0.1, 0.0);
  mRigidBodies.push_back(rb1);
    
  RigidBody *rb2 = new RigidBody(dart::dynamics::Shape::ELLIPSOID, Vector3d(0.06, 0.06, 0.06));
  mCollisionDetector->addRigidBody(rb2, "ellipse"); // Put rb2 in collision detector
  rb2->mPosition[0] = 0.3;
  rb2->mPosition[1] = -0.5;
  rb1->mAngMomentum = Vector3d(0.1, 0.0, 0.0);
  rb2->mColor = Vector4d(0.2, 0.8, 0.2, 1.0); // Blue
  mRigidBodies.push_back(rb2);
}

void MyWorld::initializePinata() {
  // Add pinata to the collison detector
  mCollisionDetector->addSkeleton(mPinataWorld->getSkeleton(0));
  int nJoints = mPinataWorld->getSkeleton(0)->getNumBodyNodes();
  for (int i = 0; i < nJoints; i++) {
    int nDofs = mPinataWorld->getSkeleton(0)->getJoint(i)->getNumDofs();
    for (int j = 0; j < nDofs; j++)
      mPinataWorld->getSkeleton(0)->getJoint(i)->setDampingCoefficient(j, 1.0);
  }

  // Weld two seems to make a box
  dart::dynamics::BodyNode* top = mPinataWorld->getSkeleton(0)->getBodyNode("top");
  dart::dynamics::BodyNode* front = mPinataWorld->getSkeleton(0)->getBodyNode("front");
  dart::dynamics::BodyNode* back = mPinataWorld->getSkeleton(0)->getBodyNode("back");
  dart::constraint::WeldJointConstraint *joint1 = new dart::constraint::WeldJointConstraint(top, front);    
  dart::constraint::WeldJointConstraint *joint2 = new dart::constraint::WeldJointConstraint(top, back);    
  mPinataWorld->getConstraintSolver()->addConstraint(joint1);
  mPinataWorld->getConstraintSolver()->addConstraint(joint2);
}

MyWorld::~MyWorld() {
  for (int i = 0; i < mRigidBodies.size(); i++)
    delete mRigidBodies[i];
  mRigidBodies.clear();
  if (mCollisionDetector)
    delete mCollisionDetector;
  if (mPinataWorld)
    delete mPinataWorld;
}

void MyWorld::simulate() {
  mFrame++;
  // TODO: Replace the following code with your simulation
  for (int i = 0; i < mRigidBodies.size(); i++) {
    mRigidBodies[i]->mPosition += mTimeStep * (mRigidBodies[i]->mLinMomentum / mRigidBodies[i]->mMass);
    mRigidBodies[i]->mLinMomentum += mTimeStep * (mRigidBodies[i]->mMass * mGravity);
  }
  // Apply external force to the pinata
  mPinataWorld->getSkeleton(0)->getBodyNode("bottom")->addExtForce(mForce);
  mForce.setZero();
  // Simulate Pinata using DART
  mPinataWorld->step();
  // Run collision detector
  mCollisionDetector->checkCollision();

  // TODO: implement a collision handler
  collisionHandling();

  // Break the pinata if it has enough momentum
  if (mPinataWorld->getSkeleton(0)->getWorldCOMVelocity().norm() > 0.4)
    mPinataWorld->getConstraintSolver()->removeAllConstraints();
}

void MyWorld::collisionHandling() {
}
