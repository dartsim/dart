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

constexpr double default_height = 1.0; // m
constexpr double default_width = 0.2;  // m
constexpr double default_depth = 0.2;  // m

constexpr double default_torque = 15.0; // N-m
constexpr int default_countdown = 200; // Number of timesteps for applying force

using namespace dart::dynamics;
using namespace dart::simulation;

class MyWindow : public dart::gui::SimWindow
{
public:

  /// Constructor
  MyWindow(WorldPtr world)
    : _weldConstraint(nullptr),
      _positiveSign(true)
  {
    setWorld(world);

    // Find the Skeleton named "pendulum" within the World
    _pendulum = world->getSkeleton("pendulum");

    // Make sure that the pendulum was found in the World
    assert(_pendulum != nullptr);

    _forceCountDown.resize(_pendulum->getNumDofs(), 0);
  }

  /// Handle keyboard input
  void keyboard(unsigned char key, int x, int y) override
  {
    switch(key)
    {
      case '1':
        applyForce(0);
        break;
      case '2':
        applyForce(1);
        break;
      case '3':
        applyForce(2);
        break;
      case '4':
        applyForce(3);
        break;
      case '5':
        applyForce(4);
        break;
      case '6':
        applyForce(5);
        break;
      case '7':
        applyForce(6);
        break;
      case '8':
        applyForce(7);
        break;
      case '9':
        applyForce(8);
        break;
      case '0':
        applyForce(9);
        break;
      case '-':
        _positiveSign = !_positiveSign;
        break;
      case 'r':
      {
        if(_weldConstraint)
          removeConstraint();
        else
          addConstraint();
        break;
      }
    }

    SimWindow::keyboard(key, x, y);
  }

  void timeStepping() override
  {
    for(size_t i=0; i<_forceCountDown.size(); ++i)
    {
      const ShapePtr& shape =
          _pendulum->getBodyNode(i)->getVisualizationShape(0);
      DegreeOfFreedom* dof = _pendulum->getDof(i);

      if(_forceCountDown[i] > 0)
      {
        dof->setForce(_positiveSign? default_torque : -default_torque);
        shape->setColor(dart::Color::Red());
        --_forceCountDown[i];
      }
      else
      {
        dof->setForce(0.0);
        shape->setColor(dart::Color::Blue());
      }
    }

    SimWindow::timeStepping();
  }

  void applyForce(size_t index)
  {
    if(index < _forceCountDown.size())
      _forceCountDown[index] = default_countdown;
  }

  /// Add a constraint to turn the bottom of the pendulum into a triangle
  void addConstraint()
  {
    // Get the last body in the pendulum
    BodyNode* tip  = _pendulum->getBodyNode(_pendulum->getNumBodyNodes()-1);

    // Weld the last link to the world
    _weldConstraint = new dart::constraint::WeldJointConstraint(tip);
    mWorld->getConstraintSolver()->addConstraint(_weldConstraint);
  }

  /// Remove any existing constraint, allowing the pendulum to flail freely
  void removeConstraint()
  {
    mWorld->getConstraintSolver()->removeConstraint(_weldConstraint);
    delete _weldConstraint;
    _weldConstraint = nullptr;
  }

protected:

  /// The pendulum that we will be perturbing
  SkeletonPtr _pendulum;

  /// Pointer to the weld constraint that we will be turning on and off
  dart::constraint::WeldJointConstraint* _weldConstraint;

  /// Number of iterations before clearing a force entry
  std::vector<int> _forceCountDown;

  /// Whether a force should be applied in the positive or negative direction
  bool _positiveSign;
};

BodyNode* addBody(const SkeletonPtr& pendulum,
             BodyNode* parent, const std::string& name)
{
  // Set up the properties for the Joint
  RevoluteJoint::Properties properties;
  properties.mName = name+"_joint";
  properties.mAxis = Eigen::Vector3d::UnitY();
  if(parent)
  {
    properties.mT_ParentBodyToJoint.translation() =
        Eigen::Vector3d(0, 0, default_height);
  }

  // Create a new BodyNode, attached to its parent by a RevoluteJoint
  BodyNodePtr bn = pendulum->createJointAndBodyNodePair<RevoluteJoint>(
        parent, properties, BodyNode::Properties(name)).second;

  // Create a BoxShape to be used for both visualization and collision checking
  std::shared_ptr<BoxShape> box(new BoxShape(
      Eigen::Vector3d(default_width, default_depth, default_height)));
  box->setColor(dart::Color::Blue());

  // Set the location of the Box
  Eigen::Isometry3d box_tf(Eigen::Isometry3d::Identity());
  Eigen::Vector3d center = Eigen::Vector3d(0, 0, default_height/2.0);
  box_tf.translation() = center;
  box->setLocalTransform(box_tf);

  bn->addVisualizationShape(box);
  bn->addCollisionShape(box);

  // Move the center of mass to the center of the object
  bn->setLocalCOM(center);

  return bn;
}

int main(int argc, char* argv[])
{
  // Create an empty Skeleton with the name "pendulum"
  SkeletonPtr pendulum = Skeleton::create("pendulum");

  // Add each body to the last BodyNode in the pendulum
  BodyNode* bn = addBody(pendulum, nullptr, "body1");
  bn = addBody(pendulum, bn, "body2");
  bn = addBody(pendulum, bn, "body3");
  bn = addBody(pendulum, bn, "body4");
  bn = addBody(pendulum, bn, "body5");

  // Set the initial position of the first DegreeOfFreedom so that the pendulum
  // starts to swing right away
  pendulum->getDof(0)->setPosition(120*M_PI/180.0);

  // Create a world and add the pendulum to the world
  WorldPtr world(new World);
  world->addSkeleton(pendulum);

  // Create a window for rendering the world and handling user input
  MyWindow window(world);

  // Print instructions
  std::cout << "space bar: simulation on/off" << std::endl;
  std::cout << "'1' -> '9': apply torque to a pendulum body" << std::endl;
  std::cout << "'r': add/remove constraint on the end of the chain" << std::endl;

  // Initialize glut, initialize the window, and begin the glut event loop
  glutInit(&argc, argv);
  window.initWindow(640, 480, "Compound Pendulum Tutorial");
  glutMainLoop();
}
