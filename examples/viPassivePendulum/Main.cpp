/*
 * Copyright (c) 2011-2016, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Karen Liu <karenliu@cc.gatech.edu>,
 *            Jeongseok Lee <jslee02@gmail.com>
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

#include "dart/dart.hpp"
#include "MyWindow.hpp"

using namespace Eigen;
using namespace dart;
using namespace common;
using namespace math;
using namespace dynamics;

/// Function headers
enum TypeOfDOF
{
  DOF_X, DOF_Y, DOF_Z, DOF_ROLL, DOF_PITCH, DOF_YAW, BALL
};

//==============================================================================
std::pair<Joint*, BodyNode*> add1DofJoint(SkeletonPtr skel,
    BodyNode* parent, const BodyNode::Properties& node,
    const std::string& name, double val, double min, double max, int type)
{
  GenericJoint<R1Space>::Properties properties(name);
  properties.mPositionLowerLimits[0] = min;
  properties.mPositionUpperLimits[0] = max;
  std::pair<Joint*, BodyNode*> newComponent;
  if(DOF_X == type)
    newComponent = skel->createJointAndBodyNodePair<PrismaticJoint>(parent,
      PrismaticJoint::Properties(properties, Vector3d(1.0, 0.0, 0.0)), node);
  else if(DOF_Y == type)
    newComponent = skel->createJointAndBodyNodePair<PrismaticJoint>(parent,
      PrismaticJoint::Properties(properties, Vector3d(0.0, 1.0, 0.0)), node);
  else if(DOF_Z == type)
    newComponent = skel->createJointAndBodyNodePair<PrismaticJoint>(parent,
      PrismaticJoint::Properties(properties, Vector3d(0.0, 0.0, 1.0)), node);
  else if(DOF_YAW == type)
    newComponent = skel->createJointAndBodyNodePair<RevoluteJoint>(parent,
      RevoluteJoint::Properties(properties, Vector3d(0.0, 0.0, 1.0)), node);
  else if(DOF_PITCH == type)
    newComponent = skel->createJointAndBodyNodePair<RevoluteJoint>(parent,
      RevoluteJoint::Properties(properties, Vector3d(0.0, 1.0, 0.0)), node);
  else if(DOF_ROLL == type)
    newComponent = skel->createJointAndBodyNodePair<RevoluteJoint>(parent,
      RevoluteJoint::Properties(properties, Vector3d(1.0, 0.0, 0.0)), node);

  newComponent.first->setPosition(0, val);
  return newComponent;
}

//==============================================================================
/// Add an end-effector to the last link of the given robot
void addEndEffector(SkeletonPtr robot, BodyNode* parent_node, Vector3d dim)
{
  // Create the end-effector node with a random dimension
  BodyNode::Properties node(BodyNode::AspectProperties("ee"));
  std::shared_ptr<Shape> shape(new BoxShape(Vector3d(0.2, 0.2, 0.2)));

  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translate(Eigen::Vector3d(0.0, 0.0, dim(2)));
  Joint::Properties joint("eeJoint", T);

  auto pair = robot->createJointAndBodyNodePair<WeldJoint>(
        parent_node, joint, node);
  auto bodyNode = pair.second;
  bodyNode->createShapeNodeWith<
      VisualAspect, CollisionAspect, DynamicsAspect>(shape);
}

//==============================================================================
/// Creates a N link manipulator with the given dimensions where each joint is
/// the specified type
SkeletonPtr createNLinkRobot(int _n,
                             Vector3d dim,
                             TypeOfDOF type,
                             bool finished = false)
{
  assert(_n > 0);

  SkeletonPtr robot = Skeleton::create();
  robot->disableSelfCollisionCheck();

  // Create the first link, the joint with the ground and its shape
  BodyNode::Properties node(BodyNode::AspectProperties("link1"));
//  node.mInertia.setLocalCOM(Vector3d(0.0, 0.0, dim(2)/2.0));
  std::shared_ptr<Shape> shape(new BoxShape(dim));

  std::pair<Joint*, BodyNode*> pair1;
  if (BALL == type)
  {
    BallJoint::Base::Properties properties(std::string("joint1"));
    pair1 = robot->createJointAndBodyNodePair<BallJoint>(
          nullptr, BallJoint::Properties(properties), node);
  }
  else
  {
    pair1 = add1DofJoint(robot, nullptr, node, "joint1", 0.0,
                         -constantsd::pi(), constantsd::pi(), type);
  }

  Joint* joint = pair1.first;
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  // T.translation() = Eigen::Vector3d(0.0, 0.0, -0.5*dim(2));
  // joint->setTransformFromParentBodyNode(T);
  T.translation() = Eigen::Vector3d(0.0, 0.0, 0.5*dim(2));
  joint->setTransformFromChildBodyNode(T);
  //joint->setDampingCoefficient(0, 0.01);

  auto current_node = pair1.second;
  current_node->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(
        shape);

  BodyNode* parent_node = current_node;

  // Create links iteratively
  for (int i = 1; i < _n; ++i)
  {
    std::ostringstream ssLink;
    std::ostringstream ssJoint;
    ssLink << "link" << i;
    ssJoint << "joint" << i;

    node = BodyNode::Properties(BodyNode::AspectProperties(ssLink.str()));
//    node.mInertia.setLocalCOM(Vector3d(0.0, 0.0, dim(2)/2.0));
    shape = std::shared_ptr<Shape>(new BoxShape(dim));

    std::pair<Joint*, BodyNode*> newPair;

    if (BALL == type)
    {
      BallJoint::Base::Properties properties(ssJoint.str());
      newPair = robot->createJointAndBodyNodePair<BallJoint>(
            parent_node,
            BallJoint::Properties(properties),
            node);
    }
    else
    {
      newPair = add1DofJoint(robot, parent_node, node, ssJoint.str(), 0.0,
                             -constantsd::pi(), constantsd::pi(), type);
    }

    Joint* joint = newPair.first;
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation() = Eigen::Vector3d(0.0, 0.0, -0.5*dim(2));
    joint->setTransformFromParentBodyNode(T);
    T.translation() = Eigen::Vector3d(0.0, 0.0, 0.5*dim(2));
    joint->setTransformFromChildBodyNode(T);
    //joint->setDampingCoefficient(0, 0.01);

    auto current_node = newPair.second;
    current_node->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(
          shape);

    parent_node = current_node;
  }

  // If finished, initialize the skeleton
  if(finished)
    addEndEffector(robot, parent_node, dim);

  return robot;
}

//==============================================================================
int main(int argc, char* argv[])
{
  // create and initialize the world
  auto world = std::make_shared<simulation::WorldVariationalIntegrator>();
  assert(world != nullptr);

  // create and initialize the world
  Eigen::Vector3d gravity(0.0, -9.81, 0.0);
  //world->setGravity(Eigen::Vector3d::Zero());
  world->setGravity(gravity);
  world->setTimeStep(1.0/1000);

  const auto numLinks = 10;
  const auto l = 1.0;
  auto skel = createNLinkRobot(numLinks, Vector3d(0.3, 0.3, l), DOF_ROLL);
//  auto skel = createNLinkRobot(numLinks, Vector3d(0.3, 0.3, l), BALL);
  world->addSkeleton(skel);

  const auto posLower = math::constantsd::pi() * -0.1;
  const auto posUpper = math::constantsd::pi() *  0.1;
  const auto velLower = math::constantsd::pi() * -0.1;
  const auto velUpper = math::constantsd::pi() *  0.1;
  for (auto i = 0u; i < numLinks; ++i)
  {
    auto joint = skel->getJoint(i);

    const auto pos = math::random(posLower, posUpper);
    const auto vel = math::random(velLower, velUpper);

    for (auto j = 0u; j < joint->getNumDofs(); ++j)
    {
      joint->setPosition(j, pos);
      joint->setVelocity(j, vel);
    }
  }

  world->prestep();

//  while (world->getTime() < 60.0)
//  {
//    std::cout << "time: " << world->getTime() << std::endl;
//    world->dm_step();
//  }

//  std::cout << "E: " << world->getSkeleton(0)->getTotalEnergy() << std::endl;

  // create a window and link it to the world
  MyWindow window;
  window.setWorld(world);

  glutInit(&argc, argv);
  window.initWindow(640, 480, "Forward Simulation");
  glutMainLoop();

  return 0;
}
