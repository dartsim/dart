/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include <dart/gui/osg/osg.hpp>

#include <dart/dart.hpp>

const double default_height = 1.0; // m
const double default_width = 0.2;  // m
const double default_depth = 0.2;  // m

const double default_torque = 15.0; // N-m
const double default_force = 15.0;  // N
const int default_countdown = 200;  // Number of timesteps for applying force

const double default_rest_position = 0.0;
const double delta_rest_position = dart::math::toRadian(10.0);

const double default_stiffness = 0.0;
const double delta_stiffness = 10;

const double default_damping = 5.0;
const double delta_damping = 1.0;

using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::gui::osg;

class Controller
{
public:
  /// Constructor
  Controller(const SkeletonPtr& pendulum, WorldPtr world)
    : mPendulum(pendulum), mWorld(world), mBallConstraint(nullptr), mPositiveSign(true), mBodyForce(false)
  {
    // Make sure that the pendulum was found in the World
    assert(mPendulum != nullptr);

    mForceCountDown.resize(mPendulum->getNumDofs(), 0);

    ArrowShape::Properties arrow_properties;
    arrow_properties.mRadius = 0.05;
    mArrow = std::shared_ptr<ArrowShape>(new ArrowShape(
        Eigen::Vector3d(-default_height, 0.0, default_height / 2.0),
        Eigen::Vector3d(-default_width / 2.0, 0.0, default_height / 2.0),
        arrow_properties,
        dart::Color::Orange(1.0)));
  }

  void changeDirection()
  {
    mPositiveSign = !mPositiveSign;
    if (mPositiveSign) {
      mArrow->setPositions(
          Eigen::Vector3d(-default_height, 0.0, default_height / 2.0),
          Eigen::Vector3d(-default_width / 2.0, 0.0, default_height / 2.0));
    } else {
      mArrow->setPositions(
          Eigen::Vector3d(default_height, 0.0, default_height / 2.0),
          Eigen::Vector3d(default_width / 2.0, 0.0, default_height / 2.0));
    }
  }

  void applyForce(std::size_t index)
  {
    if (index < mForceCountDown.size())
      mForceCountDown[index] = default_countdown;
  }

  void changeRestPosition(double delta)
  {
    for (std::size_t i = 0; i < mPendulum->getNumDofs(); ++i) {
      DegreeOfFreedom* dof = mPendulum->getDof(i);
      double q0 = dof->getRestPosition() + delta;

      // The system becomes numerically unstable when the rest position exceeds
      // 90 degrees
      if (std::abs(q0) > dart::math::toRadian(90.0))
        q0 = (q0 > 0) ? dart::math::toRadian(90.0)
                      : dart::math::toRadian(-90.0);

      dof->setRestPosition(q0);
    }

    // Only curl up along one axis in the BallJoint
    mPendulum->getDof(0)->setRestPosition(0.0);
    mPendulum->getDof(2)->setRestPosition(0.0);
  }

  void changeStiffness(double delta)
  {
    for (std::size_t i = 0; i < mPendulum->getNumDofs(); ++i) {
      DegreeOfFreedom* dof = mPendulum->getDof(i);
      double stiffness = dof->getSpringStiffness() + delta;
      if (stiffness < 0.0)
        stiffness = 0.0;
      dof->setSpringStiffness(stiffness);
    }
  }

  void changeDamping(double delta)
  {
    for (std::size_t i = 0; i < mPendulum->getNumDofs(); ++i) {
      DegreeOfFreedom* dof = mPendulum->getDof(i);
      double damping = dof->getDampingCoefficient() + delta;
      if (damping < 0.0)
        damping = 0.0;
      dof->setDampingCoefficient(damping);
    }
  }

  /// Add a constraint to attach the final link to the world
  void addConstraint()
  {
    // Get the last body in the pendulum
    BodyNode* tip = mPendulum->getBodyNode(mPendulum->getNumBodyNodes() - 1);

    // Attach the last link to the world
    Eigen::Vector3d location
        = tip->getTransform() * Eigen::Vector3d(0.0, 0.0, default_height);
    mBallConstraint = std::make_shared<dart::constraint::BallJointConstraint>(
        tip, location);
    mWorld->getConstraintSolver()->addConstraint(mBallConstraint);
  }

  /// Remove any existing constraint, allowing the pendulum to flail freely
  void removeConstraint()
  {
    mWorld->getConstraintSolver()->removeConstraint(mBallConstraint);
    mBallConstraint = nullptr;
  }

  bool hasConstraint() const
  {
    return mBallConstraint != nullptr;
  }

  void toggleBodyForce()
  {
    mBodyForce = !mBodyForce;
  }

  void update()
  {
    // Reset all the shapes to be Blue
    for (std::size_t i = 0; i < mPendulum->getNumBodyNodes(); ++i) {
      BodyNode* bn = mPendulum->getBodyNode(i);

      // If we have three visualization shapes, that means the arrow is
      // attached. We should remove it in case this body is no longer
      // experiencing a force
      if (bn->getNumShapeNodesWith<VisualAspect>() == 3u) {
        assert(bn->getShapeNodeWith<VisualAspect>(2)->getShape() == mArrow);
        bn->getShapeNodeWith<VisualAspect>(2)->remove();
      }

      bn->setColor(dart::Color::Blue());
    }

    if (!mBodyForce) {
      // Apply joint torques based on user input, and color the Joint shape red
      for (std::size_t i = 0; i < mPendulum->getNumDofs(); ++i) {
        if (mForceCountDown[i] > 0) {
          DegreeOfFreedom* dof = mPendulum->getDof(i);
          dof->setForce(mPositiveSign ? default_torque : -default_torque);

          BodyNode* bn = dof->getChildBodyNode();
          bn->getShapeNodeWith<VisualAspect>(0)->getVisualAspect()->setColor(
              dart::Color::Red());

          --mForceCountDown[i];
        }
      }
    } else {
      // Apply body forces based on user input, and color the body shape red
      for (std::size_t i = 0; i < mPendulum->getNumBodyNodes(); ++i) {
        if (mForceCountDown[i] > 0) {
          BodyNode* bn = mPendulum->getBodyNode(i);

          Eigen::Vector3d force = default_force * Eigen::Vector3d::UnitX();
          Eigen::Vector3d location(
              -default_width / 2.0, 0.0, default_height / 2.0);
          if (!mPositiveSign) {
            force = -force;
            location[0] = -location[0];
          }
          bn->addExtForce(force, location, true, true);
          bn->getShapeNodeWith<VisualAspect>(1)->getVisualAspect()->setColor(
              dart::Color::Red());
          bn->createShapeNodeWith<VisualAspect>(mArrow);

          --mForceCountDown[i];
        }
      }
    }
  }

protected:
  /// An arrow shape that we will use to visualize applied forces
  std::shared_ptr<ArrowShape> mArrow;

  /// The pendulum that we will be perturbing
  SkeletonPtr mPendulum;

  /// World pointer needed for constraints
  WorldPtr mWorld;

  /// Pointer to the ball constraint that we will be turning on and off
  dart::constraint::BallJointConstraintPtr mBallConstraint;

  /// Number of iterations before clearing a force entry
  std::vector<int> mForceCountDown;

  /// Whether a force should be applied in the positive or negative direction
  bool mPositiveSign;

  /// True if 1-9 should be used to apply a body force. Otherwise, 1-9 will be
  /// used to apply a joint torque.
  bool mBodyForce;
};

class PendulumEventHandler : public ::osgGA::GUIEventHandler
{
public:
  PendulumEventHandler(const WorldPtr& world, Controller* controller)
    : mWorld(world), mController(controller)
  {
  }

  bool handle(
      const ::osgGA::GUIEventAdapter& ea, ::osgGA::GUIActionAdapter&) override
  {
    if (ea.getEventType() == ::osgGA::GUIEventAdapter::KEYDOWN) {
      switch (ea.getKey()) {
        case '-':
          mController->changeDirection();
          return true;
        case '1':
          mController->applyForce(0);
          return true;
        case '2':
          mController->applyForce(1);
          return true;
        case '3':
          mController->applyForce(2);
          return true;
        case '4':
          mController->applyForce(3);
          return true;
        case '5':
          mController->applyForce(4);
          return true;
        case '6':
          mController->applyForce(5);
          return true;
        case '7':
          mController->applyForce(6);
          return true;
        case '8':
          mController->applyForce(7);
          return true;
        case '9':
          mController->applyForce(8);
          return true;
        case '0':
          mController->applyForce(9);
          return true;
        case 'q':
          mController->changeRestPosition(delta_rest_position);
          return true;
        case 'a':
          mController->changeRestPosition(-delta_rest_position);
          return true;
        case 'w':
          mController->changeStiffness(delta_stiffness);
          return true;
        case 's':
          mController->changeStiffness(-delta_stiffness);
          return true;
        case 'e':
          mController->changeDamping(delta_damping);
          return true;
        case 'd':
          mController->changeDamping(-delta_damping);
          return true;
        case 'r':
          if (mController->hasConstraint())
            mController->removeConstraint();
          else
            mController->addConstraint();
          return true;
        case 'f':
          mController->toggleBodyForce();
          return true;
        default:
          return false;
      }
    }
    return false;
  }

protected:
  WorldPtr mWorld;
  Controller* mController;
};

class CustomWorldNode : public RealTimeWorldNode
{
public:
  CustomWorldNode(const WorldPtr& world, Controller* controller)
    : RealTimeWorldNode(world), mController(controller)
  {
  }

  void customPreStep() override
  {
    mController->update();
  }

protected:
  Controller* mController;
};

void setGeometry(const BodyNodePtr& bn)
{
  // Create a BoxShape to be used for both visualization and collision checking
  std::shared_ptr<BoxShape> box(new BoxShape(
      Eigen::Vector3d(default_width, default_depth, default_height)));

  // Create a shape node for visualization and collision checking
  auto shapeNode
      = bn->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(
          box);
  shapeNode->getVisualAspect()->setColor(dart::Color::Blue());

  // Set the location of the shape node
  Eigen::Isometry3d box_tf(Eigen::Isometry3d::Identity());
  Eigen::Vector3d center = Eigen::Vector3d(0, 0, default_height / 2.0);
  box_tf.translation() = center;
  shapeNode->setRelativeTransform(box_tf);

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

  auto bodyProp = BodyNode::Properties(BodyNode::AspectProperties(name));
  BodyNodePtr bn = pendulum
                       ->createJointAndBodyNodePair<BallJoint>(
                           nullptr, properties, bodyProp)
                       .second;

  // Make a shape for the Joint
  const double& R = default_width;
  std::shared_ptr<EllipsoidShape> ball(
      new EllipsoidShape(sqrt(2) * Eigen::Vector3d(R, R, R)));
  auto shapeNode = bn->createShapeNodeWith<VisualAspect>(ball);
  shapeNode->getVisualAspect()->setColor(dart::Color::Blue());

  // Set the geometry of the Body
  setGeometry(bn);

  return bn;
}

BodyNode* addBody(
    const SkeletonPtr& pendulum, BodyNode* parent, const std::string& name)
{
  // Set up the properties for the Joint
  RevoluteJoint::Properties properties;
  properties.mName = name + "_joint";
  properties.mAxis = Eigen::Vector3d::UnitY();
  properties.mT_ParentBodyToJoint.translation()
      = Eigen::Vector3d(0, 0, default_height);
  properties.mRestPositions[0] = default_rest_position;
  properties.mSpringStiffnesses[0] = default_stiffness;
  properties.mDampingCoefficients[0] = default_damping;

  // Create a new BodyNode, attached to its parent by a RevoluteJoint
  BodyNodePtr bn = pendulum
                       ->createJointAndBodyNodePair<RevoluteJoint>(
                           parent, properties, BodyNode::AspectProperties(name))
                       .second;

  // Make a shape for the Joint
  const double R = default_width / 2.0;
  const double h = default_depth;
  std::shared_ptr<CylinderShape> cyl(new CylinderShape(R, h));

  // Line up the cylinder with the Joint axis
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.linear() = dart::math::eulerXYZToMatrix(
      Eigen::Vector3d(dart::math::toRadian(90.0), 0, 0));

  auto shapeNode = bn->createShapeNodeWith<VisualAspect>(cyl);
  shapeNode->getVisualAspect()->setColor(dart::Color::Blue());
  shapeNode->setRelativeTransform(tf);

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
  pendulum->getDof(1)->setPosition(dart::math::toRadian(120.0));

  // Create a world and add the pendulum to the world
  WorldPtr world = World::create();
  world->addSkeleton(pendulum);

  // Create controller and event handler
  auto controller = std::make_unique<Controller>(pendulum, world);
  auto handler = new PendulumEventHandler(world, controller.get());

  // Create a WorldNode and wrap it around the world
  ::osg::ref_ptr<CustomWorldNode> node = new CustomWorldNode(world, controller.get());

  // Create a Viewer and set it up with the WorldNode
  auto viewer = Viewer();
  viewer.addWorldNode(node);
  viewer.addEventHandler(handler);

  // Print instructions
  viewer.addInstructionText("space bar: simulation on/off");
  viewer.addInstructionText("'p': replay simulation");
  viewer.addInstructionText("'1' -> '9': apply torque to a pendulum body");
  viewer.addInstructionText("'-': Change sign of applied joint torques");
  viewer.addInstructionText("'q': Increase joint rest positions");
  viewer.addInstructionText("'a': Decrease joint rest positions");
  viewer.addInstructionText("'w': Increase joint spring stiffness");
  viewer.addInstructionText("'s': Decrease joint spring stiffness");
  viewer.addInstructionText("'e': Increase joint damping");
  viewer.addInstructionText("'d': Decrease joint damping");
  viewer.addInstructionText("'r': add/remove constraint on the end of the chain");
  viewer.addInstructionText("'f': switch between applying joint torques and body forces");
  std::cout << viewer.getInstructions() << std::endl;

  // Set up the window to be 640x480
  viewer.setUpViewInWindow(0, 0, 640, 480);

  // Adjust the viewpoint of the Viewer
  viewer.getCameraManipulator()->setHomePosition(
      ::osg::Vec3(5.0f, 3.0f, 3.0f),
      ::osg::Vec3(0.0f, 0.0f, 1.0f),
      ::osg::Vec3(0.0f, 0.0f, 1.0f));

  // We need to re-dirty the CameraManipulator by passing it into the viewer
  // again, so that the viewer knows to update its HomePosition setting
  viewer.setCameraManipulator(viewer.getCameraManipulator());

  // Begin running the application loop
  viewer.run();

  return 0;
}
