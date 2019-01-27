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

#include <iostream>
#include <gtest/gtest.h>

#include "dart/config.hpp"
#include "dart/math/Helpers.hpp"
#include "TestHelpers.hpp"

using namespace Eigen;
using namespace dart;
using namespace dart::dynamics;

SkeletonPtr createFreeFloatingTwoLinkRobot(Vector3d dim1,
                                           Vector3d dim2, TypeOfDOF type2,
                                           bool finished = true)
{
  SkeletonPtr robot = Skeleton::create();

  // Create the first link
  BodyNode::Properties node(BodyNode::AspectProperties("link1"));
  node.mInertia.setLocalCOM(Vector3d(0.0, 0.0, dim1(2)/2.0));
  std::shared_ptr<Shape> shape(new BoxShape(dim1));

  BodyNode* parent_node = robot->createJointAndBodyNodePair<FreeJoint>(nullptr,
    GenericJoint<SE3Space>::Properties(std::string("joint1")), node).second;
  parent_node->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(
        shape);

  // Create the second link
  node = BodyNode::Properties(BodyNode::AspectProperties("link2"));
  node.mInertia.setLocalCOM(Vector3d(0.0, 0.0, dim2(2)/2.0));
  shape = std::shared_ptr<Shape>(new BoxShape(dim2));

  std::pair<Joint*, BodyNode*> pair2 = add1DofJoint(
        robot, parent_node, node, "joint2", 0.0, -constantsd::pi(), constantsd::pi(), type2);
  pair2.second->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(
        shape);

  Joint* joint = pair2.first;
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translate(Eigen::Vector3d(0.0, 0.0, dim1(2)));
  joint->setTransformFromParentBodyNode(T);

  parent_node = pair2.second;

  // If finished, initialize the skeleton
  if(finished)
    addEndEffector(robot, parent_node, dim2);

  return robot;
}

#if HAVE_NLOPT
//==============================================================================
//TEST(InverseKinematics, FittingTransformation)
//{
//  const double TOLERANCE = 1e-6;
//#if BUILD_TYPE_RELEASE
//  const std::size_t numRandomTests = 100;
//#else
//  const std::size_t numRandomTests = 10;
//#endif

//  // Create two link robot
//  const double l1 = 1.5;
//  const double l2 = 1.0;
//  SkeletonPtr robot = createFreeFloatingTwoLinkRobot(
//                      Vector3d(0.3, 0.3, l1),
//                      Vector3d(0.3, 0.3, l2), DOF_ROLL);
//  std::size_t dof = robot->getNumDofs();
//  VectorXd oldConfig = robot->getPositions();

//  BodyNode* body1 = robot->getBodyNode(0);
//  BodyNode* body2 = robot->getBodyNode(1);

////  Joint* joint1 = body1->getParentJoint();
//  Joint* joint2 = body2->getParentJoint();

//  //------------------------- Free joint test ----------------------------------
//  // The parent joint of body1 is free joint so body1 should be able to
//  // transform to arbitrary tramsformation.
//  for (std::size_t i = 0; i < numRandomTests; ++i)
//  {
//    // Get desiredT2 by transforming body1 to arbitrary transformation
//    Isometry3d desiredT1 = math::expMap(Vector6d::Random());
//    body1->fitWorldTransform(desiredT1);

//    // Check
//    Isometry3d newT1 = body1->getWorldTransform();
//    EXPECT_NEAR(math::logMap(newT1.inverse() * desiredT1).norm(),
//                0.0, TOLERANCE);

//    // Set to initial configuration
//    robot->setConfigs(oldConfig, true, false, false);
//  }

//  //----------------------- Revolute joint test ---------------------------------
//  // The parent joint of body2 is revolute joint so body2 can rotate along the
//  // axis of the revolute joint.
//  for (std::size_t i = 0; i < numRandomTests; ++i)
//  {
//    // Store the original transformation and joint angle
//    Isometry3d oldT2 = body2->getWorldTransform();
//    VectorXd oldQ2 = joint2->getPositions();

//    // Get desiredT2 by rotating the revolute joint with random angle
//    joint2->setConfigs(VectorXd::Random(1), true, false, false);
//    Isometry3d desiredT2 = body2->getWorldTransform();

//    // Transform body2 to the original transofrmation and check if it is done
//    // well
//    joint2->setConfigs(oldQ2, true, false, false);
//    EXPECT_NEAR(
//          math::logMap(oldT2.inverse() * body2->getWorldTransform()).norm(),
//          0.0, TOLERANCE);

//    // Try to find optimal joint angle
//    body2->fitWorldTransform(desiredT2);

//    // Check
//    Isometry3d newT2 = body2->getWorldTransform();
//    EXPECT_NEAR(math::logMap(newT2.inverse() * desiredT2).norm(),
//                0.0, TOLERANCE);
//  }

//  //---------------- Revolute joint test with joint limit ----------------------
//  for (std::size_t i = 0; i < numRandomTests; ++i)
//  {
//    // Set joint limit
//    joint2->setPositionLowerLimit(0, DART_RADIAN *  0.0);
//    joint2->setPositionUpperLimit(0, DART_RADIAN * 15.0);

//    // Store the original transformation and joint angle
//    Isometry3d oldT2 = body2->getWorldTransform();
//    VectorXd oldQ2 = joint2->getPositions();

//    // Get desiredT2 by rotating the revolute joint with random angle out of
//    // the joint limit range
//    joint2->setPosition(0, math::random(DART_RADIAN * 15.5, constantsd::pi()));
//    robot->setConfigs(robot->getPositions(), true, false, false);
//    Isometry3d desiredT2 = body2->getWorldTransform();

//    // Transform body2 to the original transofrmation and check if it is done
//    // well
//    joint2->setConfigs(oldQ2, true, false, false);
//    EXPECT_NEAR(
//          math::logMap(oldT2.inverse() * body2->getWorldTransform()).norm(),
//          0.0, TOLERANCE);

//    // Try to find optimal joint angle without joint limit constraint
//    body2->fitWorldTransform(desiredT2, BodyNode::IKP_PARENT_JOINT, false);

//    // Check if the optimal body2 transformation is reached to the desired one
//    Isometry3d newT2 = body2->getWorldTransform();
//    EXPECT_NEAR(math::logMap(newT2.inverse() * desiredT2).norm(),
//                0.0, TOLERANCE);

//    // Try to find optimal joint angle with joint limit constraint
//    body2->fitWorldTransform(desiredT2, BodyNode::IKP_PARENT_JOINT, true);

//    // Check if the optimal joint anlge is in the range
//    double newQ2 = joint2->getPosition(0);
//    EXPECT_GE(newQ2, DART_RADIAN *  0.0);
//    EXPECT_LE(newQ2, DART_RADIAN * 15.0);
//  }
//}

//==============================================================================
//TEST(InverseKinematics, FittingVelocity)
//{
//  const double TOLERANCE = 1e-4;
//#if BUILD_TYPE_RELEASE
//  const std::size_t numRandomTests = 100;
//#else
//  const std::size_t numRandomTests = 10;
//#endif

//  // Create two link robot
//  const double l1 = 1.5;
//  const double l2 = 1.0;
//  SkeletonPtr robot = createFreeFloatingTwoLinkRobot(
//                      Vector3d(0.3, 0.3, l1),
//                      Vector3d(0.3, 0.3, l2), DOF_ROLL);

//  BodyNode* body1 = robot->getBodyNode(0);
////  BodyNode* body2 = robot->getBodyNode(1);

//  Joint* joint1 = body1->getParentJoint();
////  Joint* joint2 = body2->getParentJoint();

//  //------------------------- Free joint test ----------------------------------
//  // The parent joint of body1 is free joint so body1 should be able to
//  // transform to arbitrary tramsformation.
//  for (std::size_t i = 0; i < numRandomTests; ++i)
//  {
//    // Test for linear velocity
//    Vector3d desiredVel = Vector3d::Random();
//    body1->fitWorldLinearVel(desiredVel);
//    Vector3d fittedLinVel = body1->getWorldLinearVelocity();
//    Vector3d fittedAngVel = body1->getWorldAngularVelocity();
//    Vector3d diff = fittedLinVel - desiredVel;
//    EXPECT_NEAR(diff.dot(diff), 0.0, TOLERANCE);
//    EXPECT_NEAR(fittedAngVel.dot(fittedAngVel), 0.0, TOLERANCE);
//    joint1->setGenVels(Vector6d::Zero(), true, true);
//    robot->setState(robot->getState(), true, true, false);

//    // Test for angular velocity
//    desiredVel = Vector3d::Random();
//    body1->fitWorldAngularVel(desiredVel);
//    fittedLinVel = body1->getWorldLinearVelocity();
//    fittedAngVel = body1->getWorldAngularVelocity();
//    diff = fittedAngVel - desiredVel;
//    EXPECT_NEAR(fittedLinVel.dot(fittedLinVel), 0.0, TOLERANCE);
//    EXPECT_NEAR(diff.dot(diff), 0.0, TOLERANCE);
//    joint1->setGenVels(Vector6d::Zero(), true, true);
//    robot->setState(robot->getState(), true, true, false);
//  }
//}
#endif
