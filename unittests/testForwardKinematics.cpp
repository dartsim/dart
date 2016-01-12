/*
 * Copyright (c) 2013-2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Can Erdogan <cerdogan3@gatech.edu>
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

#include <iostream>
#include <gtest/gtest.h>
#include "TestHelpers.h"

#include "kido/utils/urdf/KidoLoader.h"

std::vector<size_t> twoLinkIndices;

//==============================================================================
TEST(FORWARD_KINEMATICS, YAW_ROLL)
{
  // Checks forward kinematics for two DoF arm manipulators.
  // NOTE: The following is the reference frame description of the world
  //       frame. The x-axis is into the page, z-axis is to the top of the
  //       page and the y-axis is to the left. At the zero angle, the links
  //       are parallel to the z-axis and face the +x-axis.

  // Create the world
  const double l1 = 1.5, l2 = 1.0;
  SkeletonPtr robot = createTwoLinkRobot(Vector3d(0.3, 0.3, l1), DOF_YAW,
                                         Vector3d(0.3, 0.3, l2), DOF_ROLL);

  // Set the test cases with the joint values and the expected end-effector
  // positions
  const size_t numTests = 2;
  double temp = sqrt(0.5*l2*l2);
  Vector2d joints [numTests] = { Vector2d( KIDO_PI/4.0,  KIDO_PI/2.0),
                                 Vector2d(-KIDO_PI/4.0, -KIDO_PI/4.0) };
  Vector3d expectedPos [numTests] = { Vector3d(temp, -temp, l1),
                                      Vector3d(temp / sqrt(2.0),
                                      temp / sqrt(2.0), l1+temp) };

  // Check each case by setting the joint values and obtaining the end-effector
  // position
  for (size_t i = 0; i < numTests; i++)
  {
    robot->setPositions(Eigen::VectorXd(joints[i]));
    Vector3d actual
        = robot->getBodyNode("ee")->getTransform().translation();
    bool equality = equals(actual, expectedPos[i], 1e-3);
    EXPECT_TRUE(equality);
    if(!equality)
    {
      std::cout << "Joint values: " << joints[i].transpose() << std::endl;
      std::cout << "Actual pos: " << actual.transpose() << std::endl;
      std::cout << "Expected pos: " <<  expectedPos[i].transpose() << std::endl;
    }
  }
}

//==============================================================================
// TODO: Use link lengths in expectations explicitly
TEST(FORWARD_KINEMATICS, TWO_ROLLS)
{
  // Checks forward kinematics for two DoF arm manipulators.
  // NOTE: The following is the reference frame description of the world
  //       frame. The x-axis is into the page, z-axis is to the top of the
  //       page and the y-axis is to the left. At the zero angle, the links
  //       are parallel to the z-axis and face the +x-axis.

  // Create the world
  const double link1 = 1.5, link2 = 1.0;
  SkeletonPtr robot = createTwoLinkRobot(Vector3d(0.3, 0.3, link1), DOF_ROLL,
                                         Vector3d(0.3, 0.3, link2), DOF_ROLL);

  // Set the test cases with the joint values and the expected end-effector
  // positions
  const size_t numTests = 2;
  Vector2d joints [numTests] = { Vector2d(0.0, KIDO_PI/2.0),
                                 Vector2d(3*KIDO_PI/4.0,
                                 -KIDO_PI/4.0)};
  Vector3d expectedPos [numTests] = { Vector3d(0.0, -1.0, 1.5),
                                      Vector3d(0.0, -2.06, -1.06) };

  // Check each case by setting the joint values and obtaining the end-effector
  // position
  for (size_t i = 0; i < numTests; i++)
  {
    robot->setPositions(joints[i]);
    Vector3d actual
        = robot->getBodyNode("ee")->getTransform().translation();
    bool equality = equals(actual, expectedPos[i], 1e-3);
    EXPECT_TRUE(equality);
    if(!equality)
    {
      std::cout << "Joint values: " << joints[i].transpose() << std::endl;
      std::cout << "Actual pos: " << actual.transpose() << std::endl;
      std::cout << "Expected pos: " <<  expectedPos[i].transpose() << std::endl;
    }
  }
}

//==============================================================================
Eigen::MatrixXd finiteDifferenceJacobian(
    const SkeletonPtr& skeleton,
    const Eigen::VectorXd& q,
    const std::vector<size_t>& active_indices)
{
  Eigen::MatrixXd J(3, q.size());
  for(int i=0; i < q.size(); ++i)
  {
    const double dq = 1e-4;

    Eigen::VectorXd q_up = q;
    Eigen::VectorXd q_down = q;

    q_up[i] += 0.5*dq;
    q_down[i] -= 0.5*dq;

    BodyNode* last_bn = skeleton->getBodyNode(skeleton->getNumBodyNodes()-1);

    skeleton->setPositions(active_indices, q_up);
    Eigen::Vector3d x_up = last_bn->getTransform().translation();

    skeleton->setPositions(active_indices, q_down);
    Eigen::Vector3d x_down = last_bn->getTransform().translation();

    skeleton->setPositions(active_indices, q);
    J.col(i) = last_bn->getWorldTransform().linear().transpose() * (x_up - x_down) / dq;
  }

  return J;
}

//==============================================================================
Eigen::MatrixXd standardJacobian(
    const SkeletonPtr& skeleton,
    const Eigen::VectorXd& q,
    const std::vector<size_t>& active_indices)
{
  skeleton->setPositions(active_indices, q);
  BodyNode* last_bn = skeleton->getBodyNode(skeleton->getNumBodyNodes()-1);

  Eigen::MatrixXd J = skeleton->getJacobian(last_bn).bottomRows<3>();

  Eigen::MatrixXd reduced_J(3, q.size());
  for(int i=0; i < q.size(); ++i)
    reduced_J.col(i) = J.col(active_indices[i]);

  return reduced_J;
}

//==============================================================================
TEST(FORWARD_KINEMATICS, JACOBIAN_PARTIAL_CHANGE)
{
  // This is a regression test for issue #499
  const double tolerance = 1e-8;

  kido::utils::KidoLoader loader;
  SkeletonPtr skeleton1 =
      loader.parseSkeleton(KIDO_DATA_PATH"urdf/KR5/KR5 sixx R650.urdf");

  SkeletonPtr skeleton2 = skeleton1->clone();

  std::vector<size_t> active_indices;
  for(size_t i=0; i < 3; ++i)
    active_indices.push_back(i);

  Eigen::VectorXd q = Eigen::VectorXd::Random(active_indices.size());
  Eigen::MatrixXd fd_J = finiteDifferenceJacobian(skeleton1, q, active_indices);
  Eigen::MatrixXd J = standardJacobian(skeleton2, q, active_indices);

  EXPECT_TRUE((fd_J - J).norm() < tolerance);

  q = Eigen::VectorXd::Random(active_indices.size());
  fd_J = finiteDifferenceJacobian(skeleton1, q, active_indices);
  J = standardJacobian(skeleton2, q, active_indices);

  EXPECT_TRUE((fd_J - J).norm() < tolerance);
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
