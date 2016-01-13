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

const double default_height = 1.0; // m
const double default_width = 0.2;  // m
const double default_depth = 0.2;  // m

const double default_torque = 15.0; // N-m
const double default_force =  15.0; // N
const int default_countdown = 200;  // Number of timesteps for applying force

const double default_rest_position = 0.0;
const double delta_rest_position = 10.0 * M_PI / 180.0;

const double default_stiffness = 0.0;
const double delta_stiffness = 10;

const double default_damping = 5.0;
const double delta_damping = 1.0;

using namespace dart::dynamics;
using namespace dart::simulation;

class MyWindow : public dart::gui::SimWindow
{
public:

  /// Constructor
  MyWindow(WorldPtr world)
    : mBallConstraint(nullptr),
      mPositiveSign(true),
      mBodyForce(false)
  {
    setWorld(world);

    // Find the Skeleton named "pendulum" within the World
    mPendulum = world->getSkeleton("pendulum");

    // Make sure that the pendulum was found in the World
    assert(mPendulum != nullptr);

    mForceCountDown.resize(mPendulum->getNumDofs(), 0);

    ArrowShape::Properties arrow_properties;
    arrow_properties.mRadius = 0.05;
    mArrow = std::shared_ptr<ArrowShape>(new ArrowShape(
             Eigen::Vector3d(-default_height, 0.0, default_height / 2.0),
             Eigen::Vector3d(-default_width / 2.0, 0.0, default_height / 2.0),
             arrow_properties, dart::Color::Orange(1.0)));
  }

  void changeDirection()
  {
    mPositiveSign = !mPositiveSign;
    if(mPositiveSign)
    {
      mArrow->setPositions(
            Eigen::Vector3d(-default_height, 0.0, default_height / 2.0),
            Eigen::Vector3d(-default_width / 2.0, 0.0, default_height / 2.0));
    }
    else
    {
      mArrow->setPositions(
            Eigen::Vector3d(default_height, 0.0, default_height / 2.0),
            Eigen::Vector3d(default_width / 2.0, 0.0, default_height / 2.0));
    }
  }

  void applyForce(size_t index)
  {
    if(index < mForceCountDown.size())
      mForceCountDown[index] = default_countdown;
  }

  void changeRestPosition(double delta)
  {
    // Lesson 2a
  	for (size_t i=0; i<mPendulum->getNumDofs();++i)
  	{
  		DegreeOfFreedom* dof = mPendulum->getDof(i);
  		double q0 = dof->getRestPosition()+delta;

  		if (std::abs(q0)>90.0 *M_PI/180.0)
  			q0 = (q0 > 0)? (90.0 *M_PI/180.0):-(90.0 *M_PI/180.0);

  		dof->setRestPosition(q0);
  	}
  	mPendulum->getDof(0)->setRestPosition(0.0);
  	mPendulum->getDof(2)->setRestPosition(0.0);
  }

  void changeStiffness(double delta)
  {
    // Lesson 2b
    for (size_t i = 0; i<mPendulum->getNumDofs();++i)
    {
    	DegreeOfFreedom* dof = mPendulum->getDof(i);
    	double stiffness0 = dof->getSpringStiffness() + delta;

    	if (stiffness0<0)
    	{
    		stiffness0 = 0.0;
    	}
    	dof->setSpringStiffness(stiffness0);
    }
  }

  void changeDamping(double delta)
  {
    // Lesson 2c
    for (size_t i=0; i<mPendulum->getNumDofs(); ++i)
    {
    	DegreeOfFreedom* dof = mPendulum->getDof(i);
    	double damping0 = dof->getDampingCoefficient() + delta;

    	if (damping0<0.0)
    	{
    		damping0 = 0.0;
    	}
    	dof->setDampingCoefficient(damping0);
    }
  }

  /// Add a constraint to attach the final link to the world
  void addConstraint()
  {
    // Lesson 3
    BodyNode* tip = mPendulum->getBodyNode(mPendulum->getNumBodyNodes() - 1);
    Eigen::Vector3d location = tip->getTransform() * Eigen::Vector3d(0.0,0.0,default_height);

    mBallConstraint = new dart::constraint::BallJointConstraint(tip, location);
    mWorld->getConstraintSolver()->addConstraint(mBallConstraint);
  }

  /// Remove any existing constraint, allowing the pendulum to flail freely
  void removeConstraint()
  {
    // Lesson 3
    mWorld->getConstraintSolver()->removeConstraint(mBallConstraint);
    delete mBallConstraint;
    mBallConstraint = nullptr;
  }

  /// Handle keyboard input
  void keyboard(unsigned char key, int x, int y) override
  {
    switch(key)
    {
      case '-':
        changeDirection();
        break;

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

      case 'q':
        changeRestPosition(delta_rest_position);
        break;
      case 'a':
        changeRestPosition(-delta_rest_position);
        break;

      case 'w':
        changeStiffness(delta_stiffness);
        break;
      case 's':
        changeStiffness(-delta_stiffness);
        break;

      case 'e':
        changeDamping(delta_damping);
        break;
      case 'd':
        changeDamping(-delta_damping);
        break;

      case 'r':
      {
        if(mBallConstraint)
          removeConstraint();
        else
          addConstraint();
        break;
      }

      case 'f':
        mBodyForce = !mBodyForce;
        break;

      default:
        SimWindow::keyboard(key, x, y);
    }
  }

  void timeStepping() override
  {
    // Reset all the shapes to be Blue
    // Lesson 1a
    for(size_t i = 0; i<mPendulum->getNumBodyNodes();++i)
    {
    	BodyNode* bn = mPendulum->getBodyNode(i);
    	for(size_t j = 0; j < 2; ++j)
    	{
    		const ShapePtr& shape = bn->getVisualizationShape(j);

    		shape->setColor(dart::Color::Blue());
    	}
    	if(bn->getNumVisualizationShapes() == 3)
    	{
    		bn->removeVisualizationShape(mArrow);
    	}
    }

    if(!mBodyForce)
    {
      // Apply joint torques based on user input, and color the Joint shape red
      for(size_t i = 0; i < mPendulum->getNumDofs(); ++i)
      {
        if(mForceCountDown[i] > 0)
        {
          
          // Lesson 1b
          DegreeOfFreedom* dof = mPendulum->getDof(i);
          dof->setForce(mPositiveSign? default_torque:-default_torque);          
          BodyNode* bn = dof->getChildBodyNode();
          const ShapePtr& shape = bn->getVisualizationShape(0);
          shape->setColor(dart::Color::Red());

          --mForceCountDown[i];
        }
      }
    }
    else
    {
      // Apply body forces based on user input, and color the body shape red
      for(size_t i = 0; i < mPendulum->getNumBodyNodes(); ++i)
      {
        if(mForceCountDown[i] > 0)
        {
          // Lesson 1c
          BodyNode* bn = mPendulum->getBodyNode(i);

          Eigen::Vector3d force = default_force * Eigen::Vector3d::UnitX();
          Eigen::Vector3d location(-default_width / 2.0, 0.0, default_height / 2.0);

          if (!mPositiveSign)
          {
          	force = -force;
          	location[0]=-location[0];
          }

          bn->addExtForce(force, location, true, true);

          const ShapePtr& shape = bn->getVisualizationShape(1);
          shape->setColor(dart::Color::Red());

          bn->addVisualizationShape(mArrow);

          --mForceCountDown[i];
        }
      }
    }

    // Step the simulation forward
    SimWindow::timeStepping();
  }

protected:

  /// An arrow shape that we will use to visualize applied forces
  std::shared_ptr<ArrowShape> mArrow;

  /// The pendulum that we will be perturbing
  SkeletonPtr mPendulum;

  /// Pointer to the ball constraint that we will be turning on and off
  dart::constraint::BallJointConstraint* mBallConstraint;

  /// Number of iterations before clearing a force entry
  std::vector<int> mForceCountDown;

  /// Whether a force should be applied in the positive or negative direction
  bool mPositiveSign;

  /// True if 1-9 should be used to apply a body force. Otherwise, 1-9 will be
  /// used to apply a joint torque.
  bool mBodyForce;
};

void setGeometry(const BodyNodePtr& bn)
{
  // Create a BoxShape to be used for both visualization and collision checking
  std::shared_ptr<BoxShape> box(new BoxShape(
      Eigen::Vector3d(default_width, default_depth, default_height)));
  box->setColor(dart::Color::Blue());

  // Set the location of the Box
  Eigen::Isometry3d box_tf(Eigen::Isometry3d::Identity());
  Eigen::Vector3d center = Eigen::Vector3d(0, 0, default_height / 2.0);
  box_tf.translation() = center;
  box->setLocalTransform(box_tf);

  // Add it as a visualization and collision shape
  bn->addVisualizationShape(box);
  bn->addCollisionShape(box);

  // Move the center of mass to the center of the object
  bn->setLocalCOM(center);
}

BodyNode* makeRootBody(const SkeletonPtr& pendulum, const std::string& name)
{
  BallJoint::Properties properties;
  properties.mName = name + "_joint";
  properties.mRestPositions = Eigen::Vector3d::Constant(default_rest_position);
  properties.mSpringStiffnesses = Eigen::Vector3d::Constant(default_stiffness);
  properties.mDampingCoefficients = Eigen::Vector3d::Constant(default_damping);

  BodyNodePtr bn = pendulum->createJointAndBodyNodePair<BallJoint>(
        nullptr, properties, BodyNode::Properties(name)).second;

  // Make a shape for the Joint
  const double& R = default_width;
  std::shared_ptr<EllipsoidShape> ball(
        new EllipsoidShape(sqrt(2) * Eigen::Vector3d(R, R, R)));
  ball->setColor(dart::Color::Blue());
  bn->addVisualizationShape(ball);

  // Set the geometry of the Body
  setGeometry(bn);

  return bn;
}

BodyNode* addBody(const SkeletonPtr& pendulum, BodyNode* parent,
                  const std::string& name)
{
  // Set up the properties for the Joint
  RevoluteJoint::Properties properties;
  properties.mName = name + "_joint";
  properties.mAxis = Eigen::Vector3d::UnitY();
  properties.mT_ParentBodyToJoint.translation() =
      Eigen::Vector3d(0, 0, default_height);
  properties.mRestPosition = default_rest_position;
  properties.mSpringStiffness = default_stiffness;
  properties.mDampingCoefficient = default_damping;

  // Create a new BodyNode, attached to its parent by a RevoluteJoint
  BodyNodePtr bn = pendulum->createJointAndBodyNodePair<RevoluteJoint>(
        parent, properties, BodyNode::Properties(name)).second;

  // Make a shape for the Joint
  const double R = default_width / 2.0;
  const double h = default_depth;
  std::shared_ptr<CylinderShape> cyl(
        new CylinderShape(R, h));
  cyl->setColor(dart::Color::Blue());

  // Line up the cylinder with the Joint axis
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.linear() = dart::math::eulerXYZToMatrix(
        Eigen::Vector3d(90.0 * M_PI / 180.0, 0, 0));
  cyl->setLocalTransform(tf);

  bn->addVisualizationShape(cyl);

  // Set the geometry of the Body
  setGeometry(bn);

  return bn;
}

int main(int argc, char* argv[])
{
  // Create an empty Skeleton with the name "pendulum"
  SkeletonPtr pendulum = Skeleton::create("pendulum");

  // Add each body to the last BodyNode in the pendulum
  BodyNode* bn = makeRootBody(pendulum, "body1");
  bn = addBody(pendulum, bn, "body2");
  bn = addBody(pendulum, bn, "body3");
  bn = addBody(pendulum, bn, "body4");
  bn = addBody(pendulum, bn, "body5");

  // Set the initial position of the first DegreeOfFreedom so that the pendulum
  // starts to swing right away
  pendulum->getDof(1)->setPosition(120 * M_PI / 180.0);

  // Create a world and add the pendulum to the world
  WorldPtr world(new World);
  world->addSkeleton(pendulum);

  // Create a window for rendering the world and handling user input
  MyWindow window(world);

  // Print instructions
  std::cout << "space bar: simulation on/off" << std::endl;
  std::cout << "'p': replay simulation" << std::endl;
  std::cout << "'1' -> '9': apply torque to a pendulum body" << std::endl;
  std::cout << "'-': Change sign of applied joint torques" << std::endl;
  std::cout << "'q': Increase joint rest positions" << std::endl;
  std::cout << "'a': Decrease joint rest positions" << std::endl;
  std::cout << "'w': Increase joint spring stiffness" << std::endl;
  std::cout << "'s': Decrease joint spring stiffness" << std::endl;
  std::cout << "'e': Increase joint damping" << std::endl;
  std::cout << "'d': Decrease joint damping" << std::endl;
  std::cout << "'r': add/remove constraint on the end of the chain" << std::endl;
  std::cout << "'f': switch between applying joint torques and body forces" << std::endl;

  // Initialize glut, initialize the window, and begin the glut event loop
  glutInit(&argc, argv);
  window.initWindow(640, 480, "Multi-Pendulum Tutorial");
  glutMainLoop();
}
