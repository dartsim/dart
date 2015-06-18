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
const double default_domino_width = 0.4*default_domino_height;
const double default_domino_depth = default_domino_width/5.0;

const double default_distance = default_domino_height/2.0;
const double default_angle = 20.0*M_PI/180.0;

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
    : _manipulator(manipulator)
  {
    // Grab the current joint angles to use them as desired angles
    _qDesired = _manipulator->getPositions();

    // Grab the last body in the manipulator, and use it as an end effector
    _endEffector = _manipulator->getBodyNode(_manipulator->getNumBodyNodes()-1);

    // Compute the body frame offset for the end effector
    _offset = default_endeffector_offset * Eigen::Vector3d::UnitX();

    // Create a transform from the center of the domino to the top of the domino
    Eigen::Isometry3d target_offset(Eigen::Isometry3d::Identity());
    target_offset.translation() =
        default_domino_height/2.0 * Eigen::Vector3d::UnitZ();

    // Rotate the transform so that it matches the orientation of the end
    // effector
    target_offset.linear() =
        _endEffector->getTransform(domino->getBodyNode(0)).linear();

    // Place the _target SimpleFrame at the top of the domino
    _target = std::make_shared<SimpleFrame>(Frame::World(), "target");
    _target->setTransform(target_offset, domino->getBodyNode(0));

    // Set PD control gains
    _Kp_PD = 200.0;
    _Kd_PD =  20.0;

    // Set operational space control gains
    _Kp_OS = 5.0;
    _Kd_OS = 0.01;
  }

  /// Compute a stable PD controller that also compensates for gravity and
  /// Coriolis forces
  void setPDForces()
  {
    if(nullptr == _manipulator)
      return;

    // Compute the joint forces needed to compensate for Coriolis forces and
    // gravity
    const Eigen::VectorXd& Cg = _manipulator->getCoriolisAndGravityForces();

    // Compute the joint position error
    Eigen::VectorXd q = _manipulator->getPositions();
    Eigen::VectorXd dq = _manipulator->getVelocities();
    q += dq * _manipulator->getTimeStep();

    Eigen::VectorXd q_err = _qDesired - q;

    // Compute the joint velocity error
    Eigen::VectorXd dq_err = -dq;

    // Compute the desired joint forces
    const Eigen::MatrixXd& M = _manipulator->getMassMatrix();
    _forces = M*(_Kp_PD * q_err + _Kd_PD * dq_err) + Cg;

    _manipulator->setForces(_forces);
  }

  /// Compute an operational space controller to push on the first domino
  void setOperationalSpaceForces()
  {
    if(nullptr == _manipulator)
      return;

    const Eigen::MatrixXd& M = _manipulator->getMassMatrix();

    // Compute the Jacobian
    Jacobian J = _endEffector->getWorldJacobian(_offset);
    // Compute the pseudo-inverse of the Jacobian
    Eigen::MatrixXd pinv_J = J.transpose() * ( J * J.transpose()
                              + 0.0025*Eigen::Matrix6d::Identity() ).inverse();

    // Compute the Jacobian time derivative
    Jacobian dJ = _endEffector->getJacobianClassicDeriv(_offset);
    // Comptue the pseudo-inverse of the Jacobian time derivative
    Eigen::MatrixXd pinv_dJ = dJ.transpose() * ( dJ * dJ.transpose()
                              + 0.0025*Eigen::Matrix6d::Identity() ).inverse();

    // Compute the linear error
    Eigen::Vector6d e;
    e.tail<3>() = _target->getWorldTransform().translation()
                - _endEffector->getWorldTransform()*_offset;

    // Compute the angular error
    Eigen::AngleAxisd aa(_target->getTransform(_endEffector).linear());
    e.head<3>() = aa.angle() * aa.axis();

    // Compute the time derivative of the error
    Eigen::Vector6d de = - _endEffector->getSpatialVelocity(
          _offset, _target.get(), Frame::World());

    // Compute the forces needed to compensate for Coriolis forces and gravity
    const Eigen::VectorXd& Cg = _manipulator->getCoriolisAndGravityForces();

    // Turn the control gains into matrix form
    Eigen::Matrix6d Kp = _Kp_OS*Eigen::Matrix6d::Identity();

    size_t dofs = _manipulator->getNumDofs();
    Eigen::MatrixXd Kd = _Kd_OS * Eigen::MatrixXd::Identity(dofs, dofs);

    // Compute the joint forces needed to exert the desired workspace force
    Eigen::Vector6d fDesired = Eigen::Vector6d::Zero();
    fDesired[3] = default_push_force;
    Eigen::VectorXd f = J.transpose() * fDesired;

    // Compute the control forces
    Eigen::VectorXd dq = _manipulator->getVelocities();
    _forces = M * (pinv_J*Kp*de + pinv_dJ*Kp*e)
              - Kd*dq + Kd*pinv_J*Kp*e + Cg + f;

    _manipulator->setForces(_forces);
  }

protected:

  /// The manipulator Skeleton that we will be controlling
  SkeletonPtr _manipulator;

  /// The target pose for the controller
  SimpleFramePtr _target;

  /// End effector for the manipulator
  BodyNodePtr _endEffector;

  /// Desired joint positions when not applying the operational space controller
  Eigen::VectorXd _qDesired;

  /// The offset of the end effector from the body origin of the last BodyNode
  /// in the manipulator
  Eigen::Vector3d _offset;

  /// Control gains for the proportional error terms in the PD controller
  double _Kp_PD;

  /// Control gains for the derivative error terms in the PD controller
  double _Kd_PD;

  /// Control gains for the proportional error terms in the operational
  /// space controller
  double _Kp_OS;

  /// Control gains for the derivative error terms in the operational space
  /// controller
  double _Kd_OS;

  /// Joint forces for the manipulator (output of the Controller)
  Eigen::VectorXd _forces;
};

class MyWindow : public dart::gui::SimWindow
{
public:

  MyWindow(const WorldPtr& world)
    : _totalAngle(0.0),
      _hasEverRun(false),
      _forceCountDown(0),
      _pushCountDown(0)
  {
    setWorld(world);
    _firstDomino = world->getSkeleton("domino");
    _floor = world->getSkeleton("floor");

    _controller = std::unique_ptr<Controller>(
          new Controller(world->getSkeleton("manipulator"), _firstDomino));
  }

  // Attempt to create a new domino. If the new domino would be in collision
  // with anything (other than the floor), then discard it.
  void attemptToCreateDomino(double angle)
  {
    const SkeletonPtr& lastDomino = _dominoes.size()>0?
          _dominoes.back() : _firstDomino;

    // Compute the position for the new domino
    Eigen::Vector3d dx = default_distance*Eigen::Vector3d(
          cos(_totalAngle), sin(_totalAngle), 0.0);

    Eigen::Vector6d x = lastDomino->getPositions();
    x.tail<3>() += dx;

    // Adjust the angle for the new domino
    x[2] = _totalAngle + angle;

    // Create the new domino
    SkeletonPtr newDomino = _firstDomino->clone();
    newDomino->setName("domino #" + std::to_string(_dominoes.size()+1));
    newDomino->setPositions(x);

    mWorld->addSkeleton(newDomino);

    // Compute collisions
    dart::collision::CollisionDetector* detector =
        mWorld->getConstraintSolver()->getCollisionDetector();
    detector->detectCollision(true, true);

    // Look through the collisions to see if any dominoes are penetrating
    // something
    bool dominoCollision = false;
    size_t collisionCount = detector->getNumContacts();
    for(size_t i=0; i < collisionCount; ++i)
    {
      // If neither of the colliding BodyNodes belongs to the floor, then we
      // know the new domino is in contact with something it shouldn't be
      const dart::collision::Contact& contact = detector->getContact(i);
      if(   contact.bodyNode1.lock()->getSkeleton() != _floor
         && contact.bodyNode2.lock()->getSkeleton() != _floor)
      {
        dominoCollision = true;
        break;
      }
    }

    if(dominoCollision)
    {
      // Remove the new domino, because it is penetrating an existing one
      mWorld->removeSkeleton(newDomino);
    }
    else
    {
      // Record the latest domino addition
      _angles.push_back(angle);
      _dominoes.push_back(newDomino);
      _totalAngle += angle;
    }
  }

  // Delete the last domino that was added to the scene. (Do not delete the
  // original domino)
  void deleteLastDomino()
  {
    if(_dominoes.size() > 0)
    {
      SkeletonPtr lastDomino = _dominoes.back();
      _dominoes.pop_back();
      mWorld->removeSkeleton(lastDomino);

      _totalAngle -= _angles.back();
      _angles.pop_back();
    }
  }

  void keyboard(unsigned char key, int x, int y) override
  {
    if(!_hasEverRun)
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
          _hasEverRun = true;
          break;
      }
    }
    else
    {
      switch(key)
      {
        case 'f':
          _forceCountDown = default_force_duration;
          break;

        case 'r':
          _pushCountDown = default_push_duration;
          break;
      }
    }

    SimWindow::keyboard(key, x, y);
  }

  void timeStepping() override
  {
    // If the user has pressed the 'f' key, apply a force to the first domino in
    // order to push it over
    if(_forceCountDown > 0)
    {
      _firstDomino->getBodyNode(0)->addExtForce(
            default_push_force*Eigen::Vector3d::UnitX(),
            default_domino_height/2.0*Eigen::Vector3d::UnitZ());

      --_forceCountDown;
    }

    // Run the controller for the manipulator
    if(_pushCountDown > 0)
    {
      _controller->setOperationalSpaceForces();
      --_pushCountDown;
    }
    else
    {
      _controller->setPDForces();
    }

    SimWindow::timeStepping();
  }

protected:

  /// Base domino. Used to clone new dominoes.
  SkeletonPtr _firstDomino;

  /// Floor of the scene
  SkeletonPtr _floor;

  /// History of the dominoes that have been created
  std::vector<SkeletonPtr> _dominoes;

  /// History of the angles that the user has specified
  std::vector<double> _angles;

  /// Sum of all angles so far
  double _totalAngle;

  /// Set to true the first time spacebar is pressed
  bool _hasEverRun;

  /// The first domino will be pushed by a disembodied force while the value of
  /// this is greater than zero
  int _forceCountDown;

  /// The manipulator will attempt to push on the first domino while the value
  /// of this is greater than zero
  int _pushCountDown;

  std::unique_ptr<Controller> _controller;

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

  domino->getDof("Joint_pos_z")->setPosition(default_domino_height/2.0);

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
  tf.translation() = Eigen::Vector3d(0.0, 0.0, -floor_height/2.0);
  body->getParentJoint()->setTransformFromParentBodyNode(tf);

  return floor;
}

SkeletonPtr createManipulator()
{
  // Load the Skeleton from a file
  dart::utils::DartLoader loader;
  SkeletonPtr manipulator =
      loader.parseSkeleton(DART_DATA_PATH"urdf/KR5/KR5 sixx R650.urdf");
  manipulator->setName("manipulator");

  // Position its base in a reasonable way
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(-0.65, 0.0, 0.0);
  manipulator->getJoint(0)->setTransformFromParentBodyNode(tf);

  // Get it into a useful configuration
  manipulator->getDof(1)->setPosition( 140.0*M_PI/180.0);
  manipulator->getDof(2)->setPosition(-140.0*M_PI/180.0);

  return manipulator;
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
  std::cout << "'f': Push the first domino with a disembodies force so that it falls over" << std::endl;
  std::cout << "'r': Push the first domino with the manipulator so that it falls over" << std::endl;
  std::cout << "'v': Turn contact force visualization on/off" << std::endl;

  glutInit(&argc, argv);
  window.initWindow(640, 480, "Dominoes");
  glutMainLoop();
}
