/*
 * Copyright (c) 2011-2019, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include <dart/dart.hpp>
#include <dart/gui/gui.hpp>

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

const double defaultmEndEffectormOffset = 0.05;

using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::math;

class Controller
{
public:

  Controller(const SkeletonPtr& manipulator, const SkeletonPtr& /*domino*/)
    : mManipulator(manipulator)
  {
    // Grab the current joint angles to use them as desired angles
    // Lesson 2b

    // Set up the information needed for an operational space controller
    // Lesson 3a

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

    // Compensate for gravity and Coriolis forces
    // Lesson 2d
  }

  /// Compute an operational space controller to push on the first domino
  void setOperationalSpaceForces()
  {
    // Lesson 3b
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

class MyWindow : public dart::gui::glut::SimWindow
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

    mController = dart::common::make_unique<Controller>(
        world->getSkeleton("manipulator"), mFirstDomino);
  }

  // Attempt to create a new domino. If the new domino would be in collision
  // with anything (other than the floor), then discard it.
  void attemptToCreateDomino(double /*angle*/)
  {
    // Create a new domino
    // Lesson 1a

    // Look through the collisions to see if any dominoes are penetrating
    // something
    // Lesson 1b
  }

  // Delete the last domino that was added to the scene. (Do not delete the
  // original domino)
  void deleteLastDomino()
  {
    // Lesson 1c
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
  BodyNodePtr body =
      floor->createJointAndBodyNodePair<WeldJoint>(nullptr).second;

  // Give the body a shape
  double floor_width = 10.0;
  double floor_height = 0.01;
  std::shared_ptr<BoxShape> box(
        new BoxShape(Eigen::Vector3d(floor_width, floor_width, floor_height)));
  auto shapeNode
      = body->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(box);
  shapeNode->getVisualAspect()->setColor(dart::Color::Black());

  // Put the body into position
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = Eigen::Vector3d(0.0, 0.0, -floor_height / 2.0);
  body->getParentJoint()->setTransformFromParentBodyNode(tf);

  return floor;
}

SkeletonPtr createManipulator()
{
  // Lesson 2a
  return Skeleton::create("manipulator");
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
