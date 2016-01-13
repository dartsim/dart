/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <mxgrey@gatech.edu>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "dart/dart.h"

const double default_domino_height = 0.3;
const double default_domino_width = 0.4 * default_domino_height;
const double default_domino_depth = default_domino_width / 5.0;

const double default_distance = default_domino_height / 2.0;
const double default_angle = 20.0 * M_PI / 180.0;

const double default_domino_density = 2.6e3; // kg/m^3
const double default_domino_mass =
    default_domino_density
    * default_domino_height
    * default_domino_width
    * default_domino_depth;

const double default_push_force = 8.0;  // N
const int default_force_duration = 200; // # iterations
const int default_push_duration = 1000;  // # iterations

const double default_endeffector_offset = 0.05;

using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::math;

class Controller
{
public:

  Controller(const SkeletonPtr& manipulator, const SkeletonPtr& domino)
    : mManipulator(manipulator)
  {
    // Grab the current joint angles to use them as desired angles
    // Lesson 2b
    mQDesired = mManipulator->getPositions();

    // Set up the information needed for an operational space controller
    // Lesson 3a
    mEndEffector = mManipulator->getBodyNode(mManipulator->getNumBodyNodes()-1);
    mOffset = default_endeffector_offset * Eigen::Vector3d::UnitX();

    mTarget = std::make_shared<SimpleFrame>(Frame::World(), "target");
    Eigen::Isometry3d target_offset(Eigen::Isometry3d::Identity());
    target_offset.translation() = 
        default_domino_height / 2.0 * Eigen::Vector3d::UnitZ();

    target_offset.linear() = 
        mEndEffector->getTransform(domino->getBodyNode(0)).linear();

    mTarget->setTransform(target_offset, domino->getBodyNode(0));


    // Set PD control gains
    mKpPD = 200.0;
    mKdPD =  20.0;

    // Set operational space control gains
    mKpOS = 5.0;
    mKdOS = 0.01;
  }

  /// Compute a stable PD controller that also compensates for gravity and
  /// Coriolis forces
  void setPDForces()
  {
    // Write a stable PD controller
    // Lesson 2c
    Eigen::VectorXd q = mManipulator->getPositions();
    Eigen::VectorXd dq = mManipulator->getVelocities();

    //for stability consideration
    q+=dq*mManipulator->getTimeStep();

    Eigen::VectorXd q_err = mQDesired - q;
    Eigen::VectorXd dq_err = -dq;

    const Eigen::MatrixXd& M = mManipulator->getMassMatrix();

    

    

    // Compensate for gravity and Coriolis forces
    // Lesson 2d
    const Eigen::VectorXd& Cg = mManipulator->getCoriolisAndGravityForces();

    mForces = M * (mKpPD*q_err + mKdPD * dq_err) + Cg;
    mManipulator->setForces(mForces);
  }

  /// Compute an operational space controller to push on the first domino
  void setOperationalSpaceForces()
  {
    // Lesson 3b
    const Eigen::MatrixXd& M = mManipulator->getMassMatrix();
    Jacobian J = mEndEffector->getWorldJacobian(mOffset);

    Eigen::MatrixXd pinv_J = J.transpose() * (J* J.transpose() + 0.0025 * Eigen::Matrix6d::Identity()).inverse();

    Jacobian dJ = mEndEffector->getJacobianClassicDeriv(mOffset);

    Eigen::MatrixXd pinv_dJ = dJ.transpose() * (dJ * dJ.transpose() + 0.0025 * Eigen::Matrix6d::Identity()).inverse();

    Eigen::Vector6d e;
    e.tail<3>() = mTarget->getWorldTransform().translation() - mEndEffector->getWorldTransform() * mOffset;

    Eigen::AngleAxisd aa(mTarget->getTransform(mEndEffector).linear());
    e.head<3>() = aa.angle() * aa.axis();

    Eigen::Vector6d de = -mEndEffector->getSpatialVelocity(
          mOffset, mTarget.get(), Frame::World());

    const Eigen::VectorXd& Cg = mManipulator->getCoriolisAndGravityForces();

    Eigen::Matrix6d Kp = mKpOS * Eigen::Matrix6d::Identity();

    size_t  dofs = mManipulator->getNumDofs();
    Eigen::MatrixXd Kd = mKdOS * Eigen::MatrixXd::Identity(dofs, dofs);

    Eigen::Vector6d fDesired = Eigen::Vector6d::Zero();
    fDesired[3] = default_push_force;
    Eigen::VectorXd f = J.transpose() * fDesired;

    Eigen::VectorXd dq = mManipulator->getVelocities();
    mForces = M * (pinv_J * Kp * de + pinv_dJ * Kp * e) - Kd* dq + Kd * pinv_J *Kp*e + Cg+f;

    mManipulator->setForces(mForces);
  } 


protected:

  /// The manipulator Skeleton that we will be controlling
  SkeletonPtr mManipulator;

  /// The target pose for the controller
  SimpleFramePtr mTarget;

  /// End effector for the manipulator
  BodyNodePtr mEndEffector;

  /// Desired joint positions when not applying the operational space controller
  Eigen::VectorXd mQDesired;

  /// The offset of the end effector from the body origin of the last BodyNode
  /// in the manipulator
  Eigen::Vector3d mOffset;

  /// Control gains for the proportional error terms in the PD controller
  double mKpPD;

  /// Control gains for the derivative error terms in the PD controller
  double mKdPD;

  /// Control gains for the proportional error terms in the operational
  /// space controller
  double mKpOS;

  /// Control gains for the derivative error terms in the operational space
  /// controller
  double mKdOS;

  /// Joint forces for the manipulator (output of the Controller)
  Eigen::VectorXd mForces;
};

class MyWindow : public dart::gui::SimWindow
{
public:

  MyWindow(const WorldPtr& world)
    : mTotalAngle(0.0),
      mHasEverRun(false),
      mForceCountDown(0),
      mPushCountDown(0)
  {
    setWorld(world);
    mFirstDomino = world->getSkeleton("domino");
    mFloor = world->getSkeleton("floor");

    mController = std::unique_ptr<Controller>(
          new Controller(world->getSkeleton("manipulator"), mFirstDomino));
  }

  // Attempt to create a new domino. If the new domino would be in collision
  // with anything (other than the floor), then discard it.
  void attemptToCreateDomino(double angle)
  {
    // Create a new domino
    // Lesson 1a
    SkeletonPtr newDomino = mFirstDomino->clone();
    newDomino->setName("domino #" + std::to_string(mDominoes.size() +1));
    const SkeletonPtr& lastDomino = mDominoes.size() > 0 ? mDominoes.back() : mFirstDomino;

    Eigen::Vector3d dx = default_distance * Eigen::Vector3d(cos(mTotalAngle),sin(mTotalAngle),0.0);

    Eigen::Vector6d x = lastDomino->getPositions();
    x.tail<3>() +=dx;

    x[2] = mTotalAngle + angle;

    newDomino->setPositions(x);

    mWorld->addSkeleton(newDomino);

    // Look through the collisions to see if any dominoes are penetrating
    // something
    // Lesson 1b
    dart::collision::CollisionDetector* detector =
        mWorld->getConstraintSolver()->getCollisionDetector();
    detector->detectCollision(true, true);

    bool dominoCollision = false;
    size_t collisionCount = detector->getNumContacts();
    for(size_t i=0;i<collisionCount;++i)
    {
      const dart::collision::Contact& contact = detector->getContact(i);
      if(contact.bodyNode1.lock()->getSkeleton() != mFloor && 
          contact.bodyNode2.lock()->getSkeleton() != mFloor)
      {
        dominoCollision = true;
        break;
      }
    }
    if (dominoCollision)
    {
      mWorld->removeSkeleton(newDomino);
    }
    else
    {
      mAngles.push_back(angle);
      mDominoes.push_back(newDomino);
      mTotalAngle += angle;
    }

  }

  // Delete the last domino that was added to the scene. (Do not delete the
  // original domino)
  void deleteLastDomino()
  {
    // Lesson 1c
    if(mDominoes.size() > 0)
    {
      SkeletonPtr lastDomino = mDominoes.back();
      mDominoes.pop_back();
      mWorld->removeSkeleton(lastDomino);
      mTotalAngle -=mAngles.back();
      mAngles.pop_back();
    }
  }

  void keyboard(unsigned char key, int x, int y) override
  {
    if(!mHasEverRun)
    {
      switch(key)
      {
        case 'q':
          attemptToCreateDomino( default_angle);
          break;
        case 'w':
          attemptToCreateDomino(0.0);
          break;
        case 'e':
          attemptToCreateDomino(-default_angle);
          break;
        case 'd':
          deleteLastDomino();
          break;
        case ' ':
          mHasEverRun = true;
          break;
      }
    }
    else
    {
      switch(key)
      {
        case 'f':
          mForceCountDown = default_force_duration;
          break;

        case 'r':
          mPushCountDown = default_push_duration;
          break;
      }
    }

    SimWindow::keyboard(key, x, y);
  }

  void timeStepping() override
  {
    // If the user has pressed the 'f' key, apply a force to the first domino in
    // order to push it over
    if(mForceCountDown > 0)
    {
      // Lesson 1d
      Eigen::Vector3d force = default_push_force * Eigen::Vector3d::UnitX();
      Eigen::Vector3d location = 
          default_domino_height /2.0* Eigen::Vector3d::UnitZ();
      mFirstDomino->getBodyNode(0)->addExtForce(force, location);

      --mForceCountDown;
    }

    // Run the controller for the manipulator
    if(mPushCountDown > 0)
    {
      mController->setOperationalSpaceForces();
      --mPushCountDown;
    }
    else
    {
      mController->setPDForces();
    }

    SimWindow::timeStepping();
  }

protected:

  /// Base domino. Used to clone new dominoes.
  SkeletonPtr mFirstDomino;

  /// Floor of the scene
  SkeletonPtr mFloor;

  /// History of the dominoes that have been created
  std::vector<SkeletonPtr> mDominoes;

  /// History of the angles that the user has specified
  std::vector<double> mAngles;

  /// Sum of all angles so far
  double mTotalAngle;

  /// Set to true the first time spacebar is pressed
  bool mHasEverRun;

  /// The first domino will be pushed by a disembodied force while the value of
  /// this is greater than zero
  int mForceCountDown;

  /// The manipulator will attempt to push on the first domino while the value
  /// of this is greater than zero
  int mPushCountDown;

  std::unique_ptr<Controller> mController;

};

SkeletonPtr createDomino()
{
  // Create a Skeleton with the name "domino"
  SkeletonPtr domino = Skeleton::create("domino");

  // Create a body for the domino
  BodyNodePtr body =
      domino->createJointAndBodyNodePair<FreeJoint>(nullptr).second;

  // Create a shape for the domino
  std::shared_ptr<BoxShape> box(
        new BoxShape(Eigen::Vector3d(default_domino_depth,
                                     default_domino_width,
                                     default_domino_height)));
  body->addVisualizationShape(box);
  body->addCollisionShape(box);

  // Set up inertia for the domino
  dart::dynamics::Inertia inertia;
  inertia.setMass(default_domino_mass);
  inertia.setMoment(box->computeInertia(default_domino_mass));
  body->setInertia(inertia);

  domino->getDof("Joint_pos_z")->setPosition(default_domino_height / 2.0);

  return domino;
}

SkeletonPtr createFloor()
{
  SkeletonPtr floor = Skeleton::create("floor");

  // Give the floor a body
  BodyNodePtr body =
      floor->createJointAndBodyNodePair<WeldJoint>(nullptr).second;

  // Give the body a shape
  double floor_width = 10.0;
  double floor_height = 0.01;
  std::shared_ptr<BoxShape> box(
        new BoxShape(Eigen::Vector3d(floor_width, floor_width, floor_height)));
  box->setColor(dart::Color::Black());

  body->addVisualizationShape(box);
  body->addCollisionShape(box);

  // Put the body into position
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = Eigen::Vector3d(0.0, 0.0, -floor_height / 2.0);
  body->getParentJoint()->setTransformFromParentBodyNode(tf);

  return floor;
}

SkeletonPtr createManipulator()
{
  // Lesson 2a
  dart::utils::DartLoader loader;
  SkeletonPtr manipulator = 
      loader.parseSkeleton(DART_DATA_PATH"/my_urdf/wam_7dof_wam_bhand.urdf");
  manipulator->setName("manipulator");
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(-0.65, 0.0, 0.0);
  manipulator->getJoint(0)->setTransformFromParentBodyNode(tf);

  manipulator->getDof(1)->setPosition(140.0 * M_PI / 180.0);
  manipulator->getDof(2)->setPosition(-140.0 * M_PI / 180.0);
  return manipulator;

  //return Skeleton::create("manipulator");
}

int main(int argc, char* argv[])
{
  SkeletonPtr domino = createDomino();
  SkeletonPtr floor = createFloor();
  SkeletonPtr manipulator = createManipulator();

  WorldPtr world = std::make_shared<World>();
  world->addSkeleton(domino);
  world->addSkeleton(floor);
  world->addSkeleton(manipulator);

  MyWindow window(world);

  std::cout << "Before simulation has started, you can create new dominoes:" << std::endl;
  std::cout << "'w': Create new domino angled forward" << std::endl;
  std::cout << "'q': Create new domino angled to the left" << std::endl;
  std::cout << "'e': Create new domino angled to the right" << std::endl;
  std::cout << "'d': Delete the last domino that was created" << std::endl;
  std::cout << std::endl;
  std::cout << "spacebar: Begin simulation (you can no longer create or remove dominoes)" << std::endl;
  std::cout << "'p': replay simulation" << std::endl;
  std::cout << "'f': Push the first domino with a disembodies force so that it falls over" << std::endl;
  std::cout << "'r': Push the first domino with the manipulator so that it falls over" << std::endl;
  std::cout << "'v': Turn contact force visualization on/off" << std::endl;

  glutInit(&argc, argv);
  window.initWindow(640, 480, "Dominoes");
  glutMainLoop();
}
