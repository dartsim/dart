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
 *   ANY WAY OUT OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "helpers/gtest_utils.hpp"

#include "dart/all.hpp"
#include "dart/config.hpp"
#include "dart/math/constants.hpp"
#include "dart/math/helpers.hpp"

#include <dart/io/read.hpp>

#include <gtest/gtest.h>

#include <iostream>

using namespace Eigen;
using namespace dart;
using namespace dart::dynamics;
using namespace dart::test;

//==============================================================================
TEST(AtlasIK, HandReachesTarget)
{
  // Load Atlas
  SkeletonPtr atlas
      = dart::io::readSkeleton("dart://sample/sdf/atlas/atlas_v3_no_head.urdf");
  ASSERT_NE(atlas, nullptr);

  // Set standing pose
  atlas->getDof("r_leg_hpy")->setPosition(-45.0 * math::pi / 180.0);
  atlas->getDof("r_leg_kny")->setPosition(90.0 * math::pi / 180.0);
  atlas->getDof("r_leg_aky")->setPosition(-45.0 * math::pi / 180.0);
  atlas->getDof("l_leg_hpy")->setPosition(-45.0 * math::pi / 180.0);
  atlas->getDof("l_leg_kny")->setPosition(90.0 * math::pi / 180.0);
  atlas->getDof("l_leg_aky")->setPosition(-45.0 * math::pi / 180.0);

  // Create left hand end effector
  BodyNode* l_hand_bn = atlas->getBodyNode("l_hand");
  ASSERT_NE(l_hand_bn, nullptr);

  Eigen::Isometry3d tf_hand = Eigen::Isometry3d::Identity();
  tf_hand.translation() = Eigen::Vector3d(0.0, 0.12, 0.0);

  EndEffector* l_hand = l_hand_bn->createEndEffector("l_hand");
  l_hand->setDefaultRelativeTransform(tf_hand, true);

  // Store initial hand position
  Eigen::Vector3d initial_pos = l_hand->getWorldTransform().translation();

  // Create target 10cm forward
  auto target = SimpleFrame::createShared(Frame::World(), "target");
  Eigen::Isometry3d target_tf = l_hand->getWorldTransform();
  Eigen::Vector3d target_pos = initial_pos + Eigen::Vector3d(0.1, 0.0, 0.0);
  target_tf.translation() = target_pos;
  target->setTransform(target_tf);

  // Set up IK
  auto ik = l_hand->getIK(true);
  ik->setTarget(target);
  ik->setActive(true);
  ik->useWholeBody();

  // ✅ CORRECT: Set tight bounds so displacement produces error
  ik->getErrorMethod().setBounds(
      Eigen::Vector6d::Constant(-1e-8), Eigen::Vector6d::Constant(1e-8));

  // Configure solver
  ik->getSolver()->setNumMaxIterations(100);

  // Verify initial distance
  double initial_distance
      = (l_hand->getWorldTransform().translation() - target_pos).norm();
  std::cout << "Initial distance: " << initial_distance << "m" << std::endl;
  EXPECT_GT(initial_distance, 0.05);

  // Solve using skeleton's hierarchical IK
  auto skel_ik = atlas->getIK(true);
  bool success = skel_ik->solveAndApply(true);

  // Check final distance
  Eigen::Vector3d final_pos = l_hand->getWorldTransform().translation();
  double final_distance = (final_pos - target_pos).norm();
  std::cout << "Final distance: " << final_distance << "m" << std::endl;
  std::cout << "IK solve returned: " << success << std::endl;

  // The solver should have reduced the distance significantly
  EXPECT_LT(final_distance, initial_distance);

  // For a simple 10cm movement, we should get very close
  EXPECT_LT(final_distance, 0.01)
      << "IK should reach target within 1cm (actual: " << final_distance
      << "m)";
}

//==============================================================================
TEST(AtlasIK, TightBoundsProduceNonZeroError)
{
  // This test shows the FIX: tight bounds produce non-zero error

  SkeletonPtr atlas
      = dart::io::readSkeleton("dart://sample/sdf/atlas/atlas_v3_no_head.urdf");
  ASSERT_NE(atlas, nullptr);

  BodyNode* l_hand_bn = atlas->getBodyNode("l_hand");
  EndEffector* l_hand = l_hand_bn->createEndEffector("l_hand");

  Eigen::Vector3d initial_pos = l_hand->getWorldTransform().translation();

  auto target = SimpleFrame::createShared(Frame::World(), "target");
  Eigen::Isometry3d target_tf = l_hand->getWorldTransform();
  target_tf.translation()
      = initial_pos + Eigen::Vector3d(0.1, 0.0, 0.0); // 10cm away
  target->setTransform(target_tf);

  auto ik = l_hand->getIK(true);
  ik->setTarget(target);
  ik->setActive(true);
  ik->useWholeBody(); // Uses all dependent DOFs

  // Enforce tight bounds so the error term reflects the displacement.
  ik->getErrorMethod().setBounds(
      Eigen::Vector6d::Constant(-1e-8), Eigen::Vector6d::Constant(1e-8));

  // Compute error manually using the correct DOFs
  const auto dofs = ik->getDofs();
  Eigen::VectorXd q(dofs.size());
  for (std::size_t i = 0; i < dofs.size(); ++i)
    q[i] = atlas->getDof(dofs[i])->getPosition();

  ik->getErrorMethod().clearCache();
  const Eigen::Vector6d& error = ik->getErrorMethod().evalError(q);

  std::cout << "Error with tight bounds: " << error.transpose() << std::endl;
  std::cout << "Error norm: " << error.norm() << std::endl;

  EXPECT_GT(error.norm(), 0.01)
      << "With tight bounds, error should be non-zero when target is far";
}
