/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
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

#include <dart/gui/all.hpp>

#include <dart/utils/urdf/All.hpp>

#include <dart/all.hpp>
#include <dart/io/read.hpp>

const double default_domino_height = 0.3;
const double default_domino_width = 0.4 * default_domino_height;
const double default_domino_depth = default_domino_width / 5.0;

const double default_distance = default_domino_height / 2.0;
const double default_angle = dart::math::toRadian(20.0);

const double default_domino_density = 2.6e3; // kg/m^3
const double default_domino_mass
    = default_domino_density * default_domino_height * default_domino_width
      * default_domino_depth;

const double default_push_force = 8.0;  // N
const int default_force_duration = 200; // # iterations
const int default_push_duration = 1000; // # iterations

const double default_endeffector_offset = 0.05;

using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::gui;
using namespace dart::gui;
using namespace dart::math;

class Controller
{
public:
  Controller(const SkeletonPtr& manipulator, const SkeletonPtr& domino)
    : mManipulator(manipulator)
  {
    // Grab the current joint angles to use them as desired angles
    // snippet:cpp-dominoes-lesson2b-desired-positions-start
    mQDesired = mManipulator->getPositions();
    // snippet:cpp-dominoes-lesson2b-desired-positions-end

    // Grab the last body in the manipulator, and use it as an end effector
    // snippet:cpp-dominoes-lesson3a-end-effector-start
    mEndEffector
        = mManipulator->getBodyNode(mManipulator->getNumBodyNodes() - 1);
    // snippet:cpp-dominoes-lesson3a-end-effector-end

    // Compute the body frame offset for the end effector
    // snippet:cpp-dominoes-lesson3a-offset-start
    mOffset = default_endeffector_offset * Eigen::Vector3d::UnitX();
    // snippet:cpp-dominoes-lesson3a-offset-end

    // Create a target reference frame
    // snippet:cpp-dominoes-lesson3a-target-frame-start
    mTarget = std::make_shared<SimpleFrame>(Frame::World(), "target");
    // snippet:cpp-dominoes-lesson3a-target-frame-end

    // Create a transform from the center of the domino to the top of the domino
    // snippet:cpp-dominoes-lesson3a-target-offset-start
    Eigen::Isometry3d target_offset(Eigen::Isometry3d::Identity());
    target_offset.translation()
        = default_domino_height / 2.0 * Eigen::Vector3d::UnitZ();
    // snippet:cpp-dominoes-lesson3a-target-offset-end

    // Rotate the transform so that it matches the orientation of the end
    // effector
    // snippet:cpp-dominoes-lesson3a-target-rotation-start
    target_offset.linear()
        = mEndEffector->getTransform(domino->getBodyNode(0)).linear();
    // snippet:cpp-dominoes-lesson3a-target-rotation-end

    // Place the mTarget SimpleFrame at the top of the domino
    // snippet:cpp-dominoes-lesson3a-target-set-start
    mTarget->setTransform(target_offset, domino->getBodyNode(0));
    // snippet:cpp-dominoes-lesson3a-target-set-end

    // Set PD control gains
    mKpPD = 200.0;
    mKdPD = 20.0;

    // Set operational space control gains
    mKpOS = 5.0;
    mKdOS = 0.01;
  }

  /// Compute a stable PD controller that also compensates for gravity and
  /// Coriolis forces
  void setPDForces()
  {
    if (nullptr == mManipulator) {
      return;
    }

    // Compute the joint position error
    // snippet:cpp-dominoes-lesson2c-state-start
    Eigen::VectorXd q = mManipulator->getPositions();
    Eigen::VectorXd dq = mManipulator->getVelocities();
    // snippet:cpp-dominoes-lesson2c-state-end
    // snippet:cpp-dominoes-lesson2c-integrate-start
    q += dq * mManipulator->getTimeStep();
    // snippet:cpp-dominoes-lesson2c-integrate-end

    // snippet:cpp-dominoes-lesson2c-q-error-start
    Eigen::VectorXd q_err = mQDesired - q;
    // snippet:cpp-dominoes-lesson2c-q-error-end

    // Compute the joint velocity error
    // snippet:cpp-dominoes-lesson2c-dq-error-start
    Eigen::VectorXd dq_err = -dq;
    // snippet:cpp-dominoes-lesson2c-dq-error-end

    // Compute the joint forces needed to compensate for Coriolis forces and
    // gravity
    // snippet:cpp-dominoes-lesson2d-cg-start
    const Eigen::VectorXd& Cg = mManipulator->getCoriolisAndGravityForces();
    // snippet:cpp-dominoes-lesson2d-cg-end

    // Compute the desired joint forces
    // snippet:cpp-dominoes-lesson2c-mass-start
    const Eigen::MatrixXd& M = mManipulator->getMassMatrix();
    // snippet:cpp-dominoes-lesson2c-mass-end
    // snippet:cpp-dominoes-lesson2c-force-law-start
    mForces = M * (mKpPD * q_err + mKdPD * dq_err) + Cg;
    // snippet:cpp-dominoes-lesson2c-force-law-end

    // snippet:cpp-dominoes-lesson2c-apply-start
    mManipulator->setForces(mForces);
    // snippet:cpp-dominoes-lesson2c-apply-end
  }

  /// Compute an operational space controller to push on the first domino
  void setOperationalSpaceForces()
  {
    if (nullptr == mManipulator) {
      return;
    }

    // snippet:cpp-dominoes-lesson3b-mass-start
    const Eigen::MatrixXd& M = mManipulator->getMassMatrix();
    // snippet:cpp-dominoes-lesson3b-mass-end

    // Compute the Jacobian
    // snippet:cpp-dominoes-lesson3b-jacobian-start
    Jacobian J = mEndEffector->getWorldJacobian(mOffset);
    // Compute the pseudo-inverse of the Jacobian
    Eigen::MatrixXd pinv_J
        = J.transpose()
          * (J * J.transpose() + 0.0025 * Eigen::Matrix6d::Identity())
                .inverse();
    // snippet:cpp-dominoes-lesson3b-jacobian-end

    // Compute the Jacobian time derivative
    // snippet:cpp-dominoes-lesson3b-jacobian-deriv-start
    Jacobian dJ = mEndEffector->getJacobianClassicDeriv(mOffset);
    // Compute the pseudo-inverse of the Jacobian time derivative
    Eigen::MatrixXd pinv_dJ
        = dJ.transpose()
          * (dJ * dJ.transpose() + 0.0025 * Eigen::Matrix6d::Identity())
                .inverse();
    // snippet:cpp-dominoes-lesson3b-jacobian-deriv-end

    // Compute the linear error
    // snippet:cpp-dominoes-lesson3b-linear-error-start
    Eigen::Vector6d e;
    e.tail<3>() = mTarget->getWorldTransform().translation()
                  - mEndEffector->getWorldTransform() * mOffset;
    // snippet:cpp-dominoes-lesson3b-linear-error-end

    // Compute the angular error
    // snippet:cpp-dominoes-lesson3b-angular-error-start
    Eigen::AngleAxisd aa(mTarget->getTransform(mEndEffector).linear());
    e.head<3>() = aa.angle() * aa.axis();
    // snippet:cpp-dominoes-lesson3b-angular-error-end

    // Compute the time derivative of the error
    // snippet:cpp-dominoes-lesson3b-error-derivative-start
    Eigen::Vector6d de = -mEndEffector->getSpatialVelocity(
        mOffset, mTarget.get(), Frame::World());
    // snippet:cpp-dominoes-lesson3b-error-derivative-end

    // Compute the forces needed to compensate for Coriolis forces and gravity
    // snippet:cpp-dominoes-lesson3b-cg-start
    const Eigen::VectorXd& Cg = mManipulator->getCoriolisAndGravityForces();
    // snippet:cpp-dominoes-lesson3b-cg-end

    // Turn the control gains into matrix form
    // snippet:cpp-dominoes-lesson3b-gains-kp-start
    Eigen::Matrix6d Kp = mKpOS * Eigen::Matrix6d::Identity();
    // snippet:cpp-dominoes-lesson3b-gains-kp-end

    // snippet:cpp-dominoes-lesson3b-gains-kd-start
    std::size_t dofs = mManipulator->getNumDofs();
    Eigen::MatrixXd Kd = mKdOS * Eigen::MatrixXd::Identity(dofs, dofs);
    // snippet:cpp-dominoes-lesson3b-gains-kd-end

    // Compute the joint forces needed to exert the desired workspace force
    // snippet:cpp-dominoes-lesson3b-feedforward-start
    Eigen::Vector6d fDesired = Eigen::Vector6d::Zero();
    fDesired[3] = default_push_force;
    Eigen::VectorXd f = J.transpose() * fDesired;
    // snippet:cpp-dominoes-lesson3b-feedforward-end

    // Compute the control forces
    // snippet:cpp-dominoes-lesson3b-control-law-start
    Eigen::VectorXd dq = mManipulator->getVelocities();
    mForces = M * (pinv_J * Kp * de + pinv_dJ * Kp * e) - Kd * dq
              + Kd * pinv_J * Kp * e + Cg + f;
    // snippet:cpp-dominoes-lesson3b-control-law-end

    // snippet:cpp-dominoes-lesson3b-apply-start
    mManipulator->setForces(mForces);
    // snippet:cpp-dominoes-lesson3b-apply-end
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

class DominoEventHandler : public ::osgGA::GUIEventHandler
{
public:
  DominoEventHandler(const WorldPtr& world, Controller* controller)
    : mWorld(world),
      mController(controller),
      mTotalAngle(0.0),
      mHasEverRun(false),
      mForceCountDown(0),
      mPushCountDown(0)
  {
    mFirstDomino = world->getSkeleton("domino");
    mFloor = world->getSkeleton("floor");
  }

  bool handle(
      const ::osgGA::GUIEventAdapter& ea, ::osgGA::GUIActionAdapter&) override
  {
    if (ea.getEventType() == ::osgGA::GUIEventAdapter::KEYDOWN) {
      if (!mHasEverRun) {
        switch (ea.getKey()) {
          case 'q':
            attemptToCreateDomino(default_angle);
            return true;
          case 'w':
            attemptToCreateDomino(0.0);
            return true;
          case 'e':
            attemptToCreateDomino(-default_angle);
            return true;
          case 'd':
            deleteLastDomino();
            return true;
          case ' ':
            mHasEverRun = true;
            return true;
          default:
            return false;
        }
      } else {
        switch (ea.getKey()) {
          case 'f':
            mForceCountDown = default_force_duration;
            return true;
          case 'r':
            mPushCountDown = default_push_duration;
            return true;
          default:
            return false;
        }
      }
    }
    return false;
  }

  void update()
  {
    // If the user has pressed the 'f' key, apply a force to the first domino in
    // order to push it over
    if (mForceCountDown > 0) {
      // snippet:cpp-dominoes-lesson1d-force-start
      Eigen::Vector3d force = default_push_force * Eigen::Vector3d::UnitX();
      Eigen::Vector3d location
          = default_domino_height / 2.0 * Eigen::Vector3d::UnitZ();
      mFirstDomino->getBodyNode(0)->addExtForce(force, location);
      // snippet:cpp-dominoes-lesson1d-force-end

      --mForceCountDown;
    }

    // Run the controller for the manipulator
    if (mPushCountDown > 0) {
      mController->setOperationalSpaceForces();
      --mPushCountDown;
    } else {
      mController->setPDForces();
    }
  }

  // Attempt to create a new domino. If the new domino would be in collision
  // with anything (other than the floor), then discard it.
  void attemptToCreateDomino(double angle)
  {
    // Create a new domino
    // snippet:cpp-dominoes-lesson1a-clone-start
    SkeletonPtr newDomino = mFirstDomino->cloneSkeleton();
    // snippet:cpp-dominoes-lesson1a-clone-end
    // snippet:cpp-dominoes-lesson1a-name-start
    newDomino->setName("domino #" + std::to_string(mDominoes.size() + 1));
    // snippet:cpp-dominoes-lesson1a-name-end

    // snippet:cpp-dominoes-lesson1a-last-start
    const SkeletonPtr& lastDomino
        = mDominoes.size() > 0 ? mDominoes.back() : mFirstDomino;
    // snippet:cpp-dominoes-lesson1a-last-end

    // Compute the position for the new domino
    // snippet:cpp-dominoes-lesson1a-offset-start
    Eigen::Vector3d dx
        = default_distance
          * Eigen::Vector3d(cos(mTotalAngle), sin(mTotalAngle), 0.0);
    // snippet:cpp-dominoes-lesson1a-offset-end

    // snippet:cpp-dominoes-lesson1a-copy-start
    Eigen::Vector6d x = lastDomino->getPositions();
    // snippet:cpp-dominoes-lesson1a-copy-end
    // snippet:cpp-dominoes-lesson1a-translate-start
    x.tail<3>() += dx;
    // snippet:cpp-dominoes-lesson1a-translate-end

    // Adjust the angle for the new domino
    // snippet:cpp-dominoes-lesson1a-angle-start
    x[2] = mTotalAngle + angle;
    // snippet:cpp-dominoes-lesson1a-angle-end

    // snippet:cpp-dominoes-lesson1a-set-positions-start
    newDomino->setPositions(x);
    // snippet:cpp-dominoes-lesson1a-set-positions-end

    // Check if the new domino collides with anything in the world.
    // Get the collision frames of all things in the world
    // snippet:cpp-dominoes-lesson1b-collision-check-start
    auto collisionGroup = mWorld->getConstraintSolver()->getCollisionGroup();

    // Create a new collision group which only contains the new domino
    auto collisionEngine = mWorld->getCollisionDetector();
    auto newGroup = collisionEngine->createCollisionGroup(newDomino.get());

    // Remove the floor from all things in the world, because the floor
    // will always collide with the new domino.
    collisionGroup->removeShapeFramesOf(mFloor.get());

    // Now check if the new domino collides with all the remaining things in
    // the world.
    bool dominoCollision = collisionGroup->collide(newGroup.get());

    // Put the floor back to all things in the world, otherwise the dominos
    // will fall to neverland once the simulation starts.
    collisionGroup->addShapeFramesOf(mFloor.get());

    // snippet:cpp-dominoes-lesson1b-collision-check-end
    // snippet:cpp-dominoes-lesson1b-result-start
    if (!dominoCollision) {
      mWorld->addSkeleton(newDomino);
      // Record the latest domino addition
      mAngles.push_back(angle);
      mDominoes.push_back(newDomino);
      mTotalAngle += angle;
    } else {
      std::cout << "The new domino would penetrate something. I will not add"
                << std::endl;
      std::cout << "it to the world. Remove some dominos with 'd' and try again"
                << std::endl;
    }
    // snippet:cpp-dominoes-lesson1b-result-end
  }

  // Delete the last domino that was added to the scene. (Do not delete the
  // original domino)
  void deleteLastDomino()
  {
    // snippet:cpp-dominoes-lesson1c-delete-start
    if (mDominoes.size() > 0) {
      SkeletonPtr lastDomino = mDominoes.back();
      mDominoes.pop_back();
      mWorld->removeSkeleton(lastDomino);

      mTotalAngle -= mAngles.back();
      mAngles.pop_back();
    }
    // snippet:cpp-dominoes-lesson1c-delete-end
  }

protected:
  WorldPtr mWorld;
  Controller* mController;

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
};

class CustomWorldNode : public RealTimeWorldNode
{
public:
  CustomWorldNode(const WorldPtr& world, DominoEventHandler* handler)
    : RealTimeWorldNode(world), mHandler(handler)
  {
  }

  void customPreStep() override
  {
    mHandler->update();
  }

protected:
  DominoEventHandler* mHandler;
};

SkeletonPtr createDomino()
{
  // Create a Skeleton with the name "domino"
  SkeletonPtr domino = Skeleton::create("domino");

  // Create a body for the domino
  BodyNodePtr body
      = domino->createJointAndBodyNodePair<FreeJoint>(nullptr).second;

  // Create a shape for the domino
  std::shared_ptr<BoxShape> box(new BoxShape(
      Eigen::Vector3d(
          default_domino_depth, default_domino_width, default_domino_height)));
  body->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(box);

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
  BodyNodePtr body
      = floor->createJointAndBodyNodePair<WeldJoint>(nullptr).second;

  // Give the body a shape
  double floor_width = 10.0;
  double floor_height = 0.01;
  std::shared_ptr<BoxShape> box(
      new BoxShape(Eigen::Vector3d(floor_width, floor_width, floor_height)));
  auto shapeNode = body->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(box);
  shapeNode->getVisualAspect()->setColor(dart::Color::Black());

  // Put the body into position
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = Eigen::Vector3d(0.0, 0.0, -floor_height / 2.0);
  body->getParentJoint()->setTransformFromParentBodyNode(tf);

  return floor;
}

SkeletonPtr createManipulator()
{
  // Load the Skeleton from a file
  // snippet:cpp-dominoes-lesson2a-loader-start
  dart::io::ReadOptions options;
  // snippet:cpp-dominoes-lesson2a-loader-end
  // snippet:cpp-dominoes-lesson2a-parse-start
  SkeletonPtr manipulator = dart::io::readSkeleton(
      "dart://sample/urdf/KR5/KR5 sixx R650.urdf", options);
  // snippet:cpp-dominoes-lesson2a-parse-end
  // snippet:cpp-dominoes-lesson2a-name-start
  manipulator->setName("manipulator");
  // snippet:cpp-dominoes-lesson2a-name-end

  // Position its base in a reasonable way
  // snippet:cpp-dominoes-lesson2a-base-start
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(-0.65, 0.0, 0.0);
  manipulator->getJoint(0)->setTransformFromParentBodyNode(tf);
  // snippet:cpp-dominoes-lesson2a-base-end

  // Get it into a useful configuration
  // snippet:cpp-dominoes-lesson2a-configuration-start
  manipulator->getDof(1)->setPosition(toRadian(140.0));
  manipulator->getDof(2)->setPosition(toRadian(-140.0));
  // snippet:cpp-dominoes-lesson2a-configuration-end

  // snippet:cpp-dominoes-lesson2a-return-start
  return manipulator;
  // snippet:cpp-dominoes-lesson2a-return-end
}

int main()
{
  SkeletonPtr domino = createDomino();
  SkeletonPtr floor = createFloor();
  SkeletonPtr manipulator = createManipulator();

  WorldPtr world = std::make_shared<World>();
  world->addSkeleton(domino);
  world->addSkeleton(floor);
  world->addSkeleton(manipulator);

  // Create controller and event handler
  auto controller = std::make_unique<Controller>(manipulator, domino);
  auto handler = new DominoEventHandler(world, controller.get());

  // Create a WorldNode and wrap it around the world
  ::osg::ref_ptr<CustomWorldNode> node = new CustomWorldNode(world, handler);

  // Create a Viewer and set it up with the WorldNode
  auto viewer = Viewer();
  viewer.addWorldNode(node);
  viewer.addEventHandler(handler);

  // Print instructions
  viewer.addInstructionText(
      "Before simulation has started, you can create new dominoes:\n");
  viewer.addInstructionText("'w': Create new domino angled forward\n");
  viewer.addInstructionText("'q': Create new domino angled to the left\n");
  viewer.addInstructionText("'e': Create new domino angled to the right\n");
  viewer.addInstructionText("'d': Delete the last domino that was created\n");
  viewer.addInstructionText("\n");
  viewer.addInstructionText(
      "spacebar: Begin simulation (you can no longer create or remove "
      "dominoes)\n");
  viewer.addInstructionText("'p': replay simulation\n");
  viewer.addInstructionText(
      "'f': Push the first domino with a disembodied force so that it falls "
      "over\n");
  viewer.addInstructionText(
      "'r': Push the first domino with the manipulator so that it falls "
      "over\n");
  viewer.addInstructionText("'v': Turn contact force visualization on/off\n");
  std::cout << viewer.getInstructions() << std::endl;

  // Set up the window to be 640x480
  viewer.setUpViewInWindow(0, 0, 640, 480);

  // Adjust the viewpoint of the Viewer
  viewer.getCameraManipulator()->setHomePosition(
      ::osg::Vec3(2.0f, 1.0f, 2.0f),
      ::osg::Vec3(0.0f, 0.0f, 0.0f),
      ::osg::Vec3(0.0f, 0.0f, 1.0f));

  // We need to re-dirty the CameraManipulator by passing it into the viewer
  // again, so that the viewer knows to update its HomePosition setting
  viewer.setCameraManipulator(viewer.getCameraManipulator());

  // Begin running the application loop
  viewer.run();

  return 0;
}
