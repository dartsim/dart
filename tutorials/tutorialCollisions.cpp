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

const double default_shape_density = 1000; // kg/m^3
const double default_shape_height  = 0.1;  // m
const double default_shape_width   = 0.03; // m

const double default_start_height = 0.4;  // m

const double minimum_start_v = 1.0; // m/s
const double maximum_start_v = 4.0; // m/s
const double default_start_v = 2;   // m/s

const double minimum_launch_angle = 20.0*M_PI/180.0; // rad
const double maximum_launch_angle = 60.0*M_PI/180.0; // rad
const double default_launch_angle = 45.0*M_PI/180.0; // rad

const double maximum_start_w = 6*M_PI; // rad/s
const double default_start_w = 3*M_PI;  // rad/s

const double default_spring_stiffness = 100;
const double default_damping_coefficient = 10;

const double default_ground_width = 2;
const double default_wall_thickness = 0.1;
const double default_wall_height = 1;
const double default_spawn_range = 0.9*default_ground_width/2;

using namespace dart::dynamics;
using namespace dart::simulation;

void setupRing(const SkeletonPtr& chain)
{
  for(size_t i=6; i < chain->getNumDofs(); ++i)
  {
    DegreeOfFreedom* dof = chain->getDof(i);
    dof->setSpringStiffness(default_spring_stiffness);
    dof->setDampingCoefficient(default_damping_coefficient);
  }

  size_t numEdges = chain->getNumBodyNodes();
  double angle = 2*M_PI/numEdges;

  for(size_t i=1; i < chain->getNumJoints(); ++i)
  {
    Joint* joint = chain->getJoint(i);
    Eigen::Vector3d restPos = BallJoint::convertToPositions(Eigen::Matrix3d(
          Eigen::AngleAxisd(angle, Eigen::Vector3d(0, 1, 0))));

    for(size_t j=0; j<3; ++j)
      joint->setRestPosition(j, restPos[j]);
  }
}

class MyWindow : public dart::gui::SimWindow
{
public:

  MyWindow(const WorldPtr& world, const SkeletonPtr& rigidChain,
           const SkeletonPtr& softChain, const SkeletonPtr& hybridChain)
    : mRandomize(true),
      mRD(),
      mMT(mRD()),
      mDistribution(-1.0, std::nextafter(1.0, 2.0)),
      mOriginalRigidChain(rigidChain),
      mOriginalSoftChain(softChain),
      mOriginalHybridChain(hybridChain),
      mSkelCount(0)
  {
    setWorld(world);
  }

  void keyboard(unsigned char key, int x, int y) override
  {
    switch(key)
    {
      case '1':
        addChain(mOriginalRigidChain->clone());
        break;

      case '2':
        addRing(mOriginalRigidChain->clone());
        break;

      case '3':
        addChain(mOriginalSoftChain->clone());
        break;

      case 'd':
        if(mWorld->getNumSkeletons() > 2)
          removeSkeleton(mWorld->getSkeleton(2));
        break;

      case 'r':
        mRandomize = !mRandomize;
        std::cout << "Randomization: " << (mRandomize? "on" : "off")
                  << std::endl;
        break;

      default:
        SimWindow::keyboard(key, x, y);
    }
  }

  void displayTimer(int _val) override
  {
    // We remove playback and baking, because we want to be able to add and
    // remove objects during runtime
    int numIter = mDisplayTimeout / (mWorld->getTimeStep() * 1000);
    if (mSimulating)
    {
      for (int i = 0; i < numIter; i++)
        timeStepping();
    }
    glutPostRedisplay();
    glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
  }

protected:

  bool addChain(const SkeletonPtr& chain)
  {
    // Set the starting position for the chain
    Eigen::Vector6d positions(Eigen::Vector6d::Zero());

    if(mRandomize)
      positions[4] = default_spawn_range*mDistribution(mMT);

    positions[5] = default_start_height;
    chain->getJoint(0)->setPositions(positions);

    // Wrap up the revolute joints of the chain
    for(size_t i=6; i < chain->getNumDofs(); ++i)
    {
      DegreeOfFreedom* dof = chain->getDof(i);
      dof->setPosition(dof->getRestPosition());
    }

    // Add the chain to the world
    chain->setName(chain->getName()+std::to_string(mSkelCount++));
    mWorld->addSkeleton(chain);

    // Compute collisions
    dart::collision::CollisionDetector* detector =
        mWorld->getConstraintSolver()->getCollisionDetector();
    detector->detectCollision(true, true);

    // Look through the collisions to see if the new ring would start in
    // collision with something
    bool collision = false;
    size_t collisionCount = detector->getNumContacts();
    for(size_t i = 0; i < collisionCount; ++i)
    {
      const dart::collision::Contact& contact = detector->getContact(i);
      if(contact.bodyNode1.lock()->getSkeleton() == chain
         || contact.bodyNode2.lock()->getSkeleton() == chain)
      {
        collision = true;
        break;
      }
    }

    // Refuse to add the chain if it is in collision
    if(collision)
    {
      mWorld->removeSkeleton(chain);
      std::cout << "The new object spawned in a collision. "
                << "It will not be added to the world." << std::endl;
      return false;
    }

    // Create reference frames for setting the initial velocity
    Eigen::Isometry3d centerTf(Eigen::Isometry3d::Identity());
    centerTf.translation() = chain->getCOM();
    SimpleFrame center(Frame::World(), "center", centerTf);

    SimpleFrame ref(&center, "root");
    ref.setRelativeTransform(chain->getBodyNode(0)->getTransform(&center));

    // Set the velocities of the reference frames so that we can easily give the
    // Skeleton the linear and angular velocities that we want
    double angle = default_launch_angle;
    double speed = default_start_v;
    double angular_speed = default_start_w;
    if(mRandomize)
    {
      angle = (mDistribution(mMT) + 1.0)/2.0 *
          (maximum_launch_angle - minimum_launch_angle) + minimum_launch_angle;

      speed = (mDistribution(mMT) + 1.0)/2.0 *
          (maximum_start_v - minimum_start_v) + minimum_start_v;

      angular_speed = mDistribution(mMT) * maximum_start_w;
    }

    Eigen::Vector3d v = speed * Eigen::Vector3d(cos(angle), 0.0, sin(angle));
    Eigen::Vector3d w = angular_speed * Eigen::Vector3d::UnitY();
    center.setClassicDerivatives(v, w);

    // Use the reference frames to set the velocity of the Skeleton's root
    chain->getJoint(0)->setVelocities(ref.getSpatialVelocity());

    return true;
  }

  void addRing(const SkeletonPtr& ring)
  {
    setupRing(ring);

    if(!addChain(ring))
      return;

    // Create a closed loop to turn the chain into a ring
    BodyNode* head = ring->getBodyNode(0);
    BodyNode* tail = ring->getBodyNode(ring->getNumBodyNodes()-1);

    Eigen::Vector3d offset = Eigen::Vector3d(0, 0, -default_shape_height / 2.0);
    auto constraint = new dart::constraint::BallJointConstraint(
          head, tail, head->getWorldTransform() * offset);

    mWorld->getConstraintSolver()->addConstraint(constraint);
    mJointConstraints.push_back(constraint);
  }

  void removeSkeleton(const SkeletonPtr& skel)
  {
    for(size_t i=0; i<mJointConstraints.size(); ++i)
    {
      dart::constraint::JointConstraint* constraint = mJointConstraints[i];
      if(constraint->getBodyNode1()->getSkeleton() == skel
         || constraint->getBodyNode2()->getSkeleton() == skel)
      {
        mWorld->getConstraintSolver()->removeConstraint(constraint);
        mJointConstraints.erase(mJointConstraints.begin()+i);
        delete constraint;
        break; // There should only be one constraint per skeleton
      }
    }

    mWorld->removeSkeleton(skel);
  }

  // std library objects that allow us to generate high-quality random numbers
  bool mRandomize;
  std::random_device mRD;
  std::mt19937 mMT;
  std::uniform_real_distribution<double> mDistribution;

  std::vector<dart::constraint::JointConstraint*> mJointConstraints;

  SkeletonPtr mOriginalRigidChain;

  SkeletonPtr mOriginalSoftChain;

  SkeletonPtr mOriginalHybridChain;

  /// Keep track of how many Skeletons we spawn to ensure we can give them all
  /// unique names
  size_t mSkelCount;

};

template<class JointType>
BodyNode* addRigidShape(const SkeletonPtr& chain, const std::string& name,
                        Shape::ShapeType type, BodyNode* parent = nullptr)
{
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  Eigen::Vector3d center = Eigen::Vector3d(0, 0, default_shape_height / 2.0);
  tf.translation() = center;

  typename JointType::Properties properties;
  properties.mName = name+"_joint";
  if(parent)
  {
    properties.mT_ParentBodyToJoint = tf;
    properties.mT_ChildBodyToJoint = tf.inverse();
  }

  BodyNode* bn = chain->createJointAndBodyNodePair<JointType>(
        parent, properties, BodyNode::Properties(name)).second;

  std::shared_ptr<Shape> shape;
  if(Shape::BOX == type)
  {
    shape = std::make_shared<BoxShape>(Eigen::Vector3d(
                                         default_shape_width,
                                         default_shape_width,
                                         default_shape_height));
  }
  else if(Shape::CYLINDER == type)
  {
    shape = std::make_shared<CylinderShape>(default_shape_width/2,
                                            default_shape_height);
  }

  bn->addVisualizationShape(shape);
  bn->addCollisionShape(shape);

  Inertia box_inertia;
  double mass = default_shape_density * shape->getVolume();
  box_inertia.setMass(mass);
  box_inertia.setMoment(shape->computeInertia(mass));
  bn->setInertia(box_inertia);

  return bn;
}

enum SoftShapeType {
  SOFT_BOX = 0,
  SOFT_CYLINDER
};

template<class JointType>
BodyNode* addSoftShape(const SkeletonPtr& chain, const std::string& name,
                       SoftShapeType type, BodyNode* parent = nullptr)
{
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  Eigen::Vector3d center = Eigen::Vector3d(0, 0, default_shape_height / 2.0);
  tf.translation() = center;

  typename JointType::Properties joint_properties;
  joint_properties.mName = name+"_joint";
  if(parent)
  {
    joint_properties.mT_ParentBodyToJoint = tf;
    joint_properties.mT_ChildBodyToJoint = tf.inverse();
  }

  double soft_shape_width = 2*default_shape_width;
  SoftBodyNode::UniqueProperties soft_properties;
  if(SOFT_BOX == type)
  {
    soft_properties = SoftBodyNodeHelper::makeBoxProperties(Eigen::Vector3d(
        soft_shape_width, soft_shape_width, default_shape_height),
        Eigen::Isometry3d::Identity(), Eigen::Vector3i(4,4,4),
        default_shape_density*default_shape_height*pow(soft_shape_width,2));
  }
  else if(SOFT_CYLINDER == type)
  {
    soft_properties = SoftBodyNodeHelper::makeCylinderProperties(
          soft_shape_width/2.0, default_shape_height, 8, 3, 2,
          default_shape_density * default_shape_height
          * pow(M_PI*soft_shape_width/2.0, 2));
  }
  soft_properties.mKv = 100;
  soft_properties.mKe = 0;
  soft_properties.mDampCoeff = 5;

  SoftBodyNode* bn = chain->createJointAndBodyNodePair<JointType, SoftBodyNode>(
        parent, joint_properties, SoftBodyNode::Properties(
          BodyNode::Properties(), soft_properties)).second;

  bn->getVisualizationShape(0)->setColor(dart::Color::Red());


  return bn;
}

SkeletonPtr createRigidChain()
{
  SkeletonPtr chain = Skeleton::create("rigid_chain");

  BodyNode* bn = addRigidShape<FreeJoint>(chain, "rigid box 1", Shape::BOX);
  bn = addRigidShape<BallJoint>(chain, "rigid cyl 2", Shape::CYLINDER, bn);
  bn = addRigidShape<BallJoint>(chain, "rigid box 3", Shape::BOX, bn);
  bn = addRigidShape<BallJoint>(chain, "rigid cyl 4", Shape::CYLINDER, bn);
  bn = addRigidShape<BallJoint>(chain, "rigid box 5", Shape::BOX, bn);
  bn = addRigidShape<BallJoint>(chain, "rigid cyl 6", Shape::CYLINDER, bn);

  return chain;
}

SkeletonPtr createSoftChain()
{
  SkeletonPtr chain = Skeleton::create("soft_chain");

  BodyNode* bn = addSoftShape<FreeJoint>(chain, "soft box 1", SOFT_BOX);
//  bn = addSoftShape<BallJoint>(chain, "soft cyl 2", SOFT_CYLINDER, bn);
//  bn = addSoftShape<BallJoint>(chain, "soft box 3", SOFT_BOX, bn);

  return chain;
}

SkeletonPtr createHybridChain()
{
  return Skeleton::create("hybrid_chain");
}

SkeletonPtr createGround()
{
  SkeletonPtr ground = Skeleton::create("ground");

  BodyNode* bn = ground->createJointAndBodyNodePair<WeldJoint>().second;

  std::shared_ptr<BoxShape> shape = std::make_shared<BoxShape>(
        Eigen::Vector3d(default_ground_width, default_ground_width,
                        default_wall_thickness));
  shape->setColor(Eigen::Vector3d(0,0,0));

  bn->addCollisionShape(shape);
  bn->addVisualizationShape(shape);

  return ground;
}

SkeletonPtr createWall()
{
  SkeletonPtr wall = Skeleton::create("wall");

  BodyNode* bn = wall->createJointAndBodyNodePair<WeldJoint>().second;

  std::shared_ptr<BoxShape> shape = std::make_shared<BoxShape>(
        Eigen::Vector3d(default_wall_thickness, default_ground_width,
                        default_wall_height));
  shape->setColor(Eigen::Vector3d(0,0,0));

  bn->addCollisionShape(shape);
  bn->addVisualizationShape(shape);

  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = Eigen::Vector3d(
        (default_ground_width + default_wall_thickness)/2.0, 0.0,
        (default_wall_height  - default_wall_thickness)/2.0);
  bn->getParentJoint()->setTransformFromParentBodyNode(tf);

  return wall;
}

int main(int argc, char* argv[])
{
  SkeletonPtr chain = createRigidChain();
  Eigen::Vector6d positions(Eigen::Vector6d::Zero());
  positions.tail<3>() = Eigen::Vector3d(0,0,3);
  chain->getJoint(0)->setPositions(positions);

  WorldPtr world = std::make_shared<World>();
  world->addSkeleton(createGround());
  world->addSkeleton(createWall());

  MyWindow window(world, createRigidChain(), createSoftChain(),
                  createHybridChain());

  std::cout << "space bar: simulation on/off" << std::endl;
  std::cout << "'1': spawn a rigid chain" << std::endl;
  std::cout << "'2': spawn a rigid ring" << std::endl;
  std::cout << "'3': spawn a soft body" << std::endl;

  glutInit(&argc, argv);
  window.initWindow(640, 480, "Collisions");
  glutMainLoop();
}
