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
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "../../helpers/GTestUtils.hpp"
#include "dart/constraint/balance_constraint.hpp"
#include "dart/dynamics/body_node.hpp"
#include "dart/dynamics/end_effector.hpp"
#include "dart/dynamics/free_joint.hpp"
#include "dart/dynamics/hierarchical_ik.hpp"
#include "dart/dynamics/prismatic_joint.hpp"
#include "dart/dynamics/skeleton.hpp"
#include "dart/math/geometry.hpp"

#include <gtest/gtest.h>

#include <array>
#include <memory>

using namespace dart;
using namespace dart::constraint;
using namespace dart::dynamics;

namespace {

struct BalanceRig
{
  SkeletonPtr skeleton;
  std::shared_ptr<WholeBodyIK> ik;
  std::size_t comDofIndex;
};

math::SupportGeometry makeFootGeometry()
{
  math::SupportGeometry geometry;
  geometry.emplace_back(Eigen::Vector3d(0.10, 0.05, 0.0));
  geometry.emplace_back(Eigen::Vector3d(0.10, -0.05, 0.0));
  geometry.emplace_back(Eigen::Vector3d(-0.10, -0.05, 0.0));
  geometry.emplace_back(Eigen::Vector3d(-0.10, 0.05, 0.0));
  return geometry;
}

EndEffector* addSupportEndEffector(
    BodyNode* parent,
    const std::string& name,
    const Eigen::Vector3d& translation,
    const math::SupportGeometry& geometry)
{
  auto* ee = parent->createEndEffector(name);
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = translation;
  ee->setDefaultRelativeTransform(tf);
  ee->resetRelativeTransform();

  auto* support = ee->getSupport(true);
  support->setGeometry(geometry);
  support->setActive(true);
  return ee;
}

BalanceRig makeBalanceRig()
{
  auto skeleton = Skeleton::create("balancer");
  skeleton->setGravity(Eigen::Vector3d(0, 0, -9.81));

  auto [rootJoint, rootBody]
      = skeleton->createJointAndBodyNodePair<FreeJoint>();
  rootBody->setMass(10.0);

  const auto geometry = makeFootGeometry();
  addSupportEndEffector(
      rootBody, "left_foot", Eigen::Vector3d(0.8, 0.15, 0.0), geometry);
  addSupportEndEffector(
      rootBody, "right_foot", Eigen::Vector3d(1.0, -0.15, 0.0), geometry);

  PrismaticJoint::Properties sliderProps;
  sliderProps.mAxis = Eigen::Vector3d::UnitX();
  auto [sliderJoint, comBody]
      = skeleton->createJointAndBodyNodePair<PrismaticJoint>(
          rootBody, sliderProps, BodyNode::Properties());
  comBody->setMass(20.0);

  auto ik = WholeBodyIK::create(skeleton);
  return {skeleton, ik, sliderJoint->getDof(0)->getIndexInSkeleton()};
}

Eigen::VectorXd getPositions(const SkeletonPtr& skeleton)
{
  Eigen::VectorXd q = skeleton->getPositions();
  q.setZero();
  return q;
}

void setComOffset(const BalanceRig& rig, double x)
{
  Eigen::VectorXd q = rig.skeleton->getPositions();
  q[rig.comDofIndex] = x;
  rig.skeleton->setPositions(q);
}

} // namespace

//==============================================================================
TEST(BalanceConstraintTests, CentroidErrorDropsInsidePolygon)
{
  auto rig = makeBalanceRig();
  auto constraint = BalanceConstraint(
      rig.ik,
      BalanceConstraint::SHIFT_SUPPORT,
      BalanceConstraint::FROM_CENTROID);

  auto q = getPositions(rig.skeleton);
  rig.skeleton->setPositions(q);
  const double outsideError = constraint.eval(q);

  ASSERT_GT(outsideError, 0.0);
  EXPECT_DOUBLE_EQ(outsideError, constraint.eval(q));

  setComOffset(rig, 1.5);
  const double insideError = constraint.eval(rig.skeleton->getPositions());
  EXPECT_NEAR(insideError, 0.0, 1e-12);
}

//==============================================================================
TEST(BalanceConstraintTests, EdgeErrorMatchesClosestPointDistance)
{
  auto rig = makeBalanceRig();
  BalanceConstraint constraint(
      rig.ik, BalanceConstraint::SHIFT_SUPPORT, BalanceConstraint::FROM_EDGE);

  auto q = getPositions(rig.skeleton);
  rig.skeleton->setPositions(q);

  const auto& axes = rig.skeleton->getSupportAxes();
  const auto& polygon = rig.skeleton->getSupportPolygon();
  Eigen::Vector3d com = rig.skeleton->getCOM();
  Eigen::Vector2d projected(com.dot(axes.first), com.dot(axes.second));

  std::size_t idxA = 0, idxB = 0;
  Eigen::Vector2d closest = math::computeClosestPointOnSupportPolygon(
      idxA, idxB, projected, polygon);

  Eigen::Vector2d diff2d = projected - closest;
  Eigen::Vector3d expected = axes.first * diff2d[0] + axes.second * diff2d[1];

  const double filterError = constraint.eval(q);
  EXPECT_NEAR(expected.norm(), filterError, 1e-12);
}

//==============================================================================
TEST(BalanceConstraintTests, OptimizeBalanceHonorsTolerance)
{
  auto rig = makeBalanceRig();
  BalanceConstraint constraint(
      rig.ik,
      BalanceConstraint::SHIFT_SUPPORT,
      BalanceConstraint::OPTIMIZE_BALANCE);

  setComOffset(rig, 1.2);
  const auto q = rig.skeleton->getPositions();

  constraint.setOptimizationTolerance(1e-6);
  const double smallToleranceError = constraint.eval(q);
  EXPECT_GT(smallToleranceError, 0.0);

  constraint.setOptimizationTolerance(1.0);
  const double relaxedError = constraint.eval(q);
  EXPECT_NEAR(relaxedError, 0.0, 1e-12);
}

//==============================================================================
TEST(BalanceConstraintTests, ShiftComProducesGradient)
{
  auto rig = makeBalanceRig();
  BalanceConstraint constraint(
      rig.ik, BalanceConstraint::SHIFT_COM, BalanceConstraint::FROM_CENTROID);

  auto q = getPositions(rig.skeleton);
  rig.skeleton->setPositions(q);

  Eigen::VectorXd gradient(rig.skeleton->getNumDofs());
  Eigen::Map<Eigen::VectorXd> gradMap(gradient.data(), gradient.size());
  constraint.evalGradient(q, gradMap);

  EXPECT_GT(gradMap.norm(), 0.0);
}
