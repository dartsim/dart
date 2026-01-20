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

#include <dart/simulation/experimental/dynamics/articulated_body.hpp>

#include <gtest/gtest.h>

namespace dse = dart::simulation::experimental;
namespace dynamics = dse::dynamics;

TEST(ABAWorkspace, ResizeAllocatesCorrectLinkCount)
{
  dynamics::ABAWorkspace workspace;
  std::vector<std::size_t> jointDOFs = {0, 1, 1, 3};

  workspace.resize(4, jointDOFs);

  EXPECT_EQ(workspace.getNumLinks(), 4u);
}

TEST(ABAWorkspace, ResizeAllocatesCorrectJointDOFs)
{
  dynamics::ABAWorkspace workspace;
  std::vector<std::size_t> jointDOFs = {0, 1, 2, 6};

  workspace.resize(4, jointDOFs);

  EXPECT_EQ(workspace.getJointData(0).projectedInertiaInverse.rows(), 0);
  EXPECT_EQ(workspace.getJointData(1).projectedInertiaInverse.rows(), 1);
  EXPECT_EQ(workspace.getJointData(2).projectedInertiaInverse.rows(), 2);
  EXPECT_EQ(workspace.getJointData(3).projectedInertiaInverse.rows(), 6);
}

TEST(ABAWorkspace, ResetClearsLinkData)
{
  dynamics::ABAWorkspace workspace;
  std::vector<std::size_t> jointDOFs = {1, 1};
  workspace.resize(2, jointDOFs);

  auto& linkData = workspace.getLinkData(0);
  linkData.articulatedInertia.setOnes();
  linkData.biasForce.setOnes();
  linkData.spatialVelocity.setOnes();

  workspace.reset();

  EXPECT_TRUE(workspace.getLinkData(0).articulatedInertia.isApprox(
      dynamics::SpatialInertia::Zero()));
  EXPECT_TRUE(workspace.getLinkData(0).biasForce.isApprox(
      dynamics::SpatialForce::Zero()));
  EXPECT_TRUE(workspace.getLinkData(0).spatialVelocity.isApprox(
      dynamics::SpatialVelocity::Zero()));
}

TEST(ABAWorkspace, ResetClearsJointData)
{
  dynamics::ABAWorkspace workspace;
  std::vector<std::size_t> jointDOFs = {2};
  workspace.resize(1, jointDOFs);

  auto& jointData = workspace.getJointData(0);
  jointData.projectedInertiaInverse.setOnes();
  jointData.totalForce.setOnes();

  workspace.reset();

  EXPECT_TRUE(workspace.getJointData(0).projectedInertiaInverse.isZero());
  EXPECT_TRUE(workspace.getJointData(0).totalForce.isZero());
}

TEST(ABAWorkspace, GetLinkDataReturnsModifiableReference)
{
  dynamics::ABAWorkspace workspace;
  std::vector<std::size_t> jointDOFs = {1};
  workspace.resize(1, jointDOFs);

  workspace.getLinkData(0).spatialVelocity.setOnes();

  EXPECT_TRUE(workspace.getLinkData(0).spatialVelocity.isApprox(
      dynamics::SpatialVelocity::Ones()));
}

TEST(ABAWorkspace, GetJointDataReturnsModifiableReference)
{
  dynamics::ABAWorkspace workspace;
  std::vector<std::size_t> jointDOFs = {3};
  workspace.resize(1, jointDOFs);

  workspace.getJointData(0).totalForce.setOnes();

  EXPECT_TRUE(
      workspace.getJointData(0).totalForce.isApprox(Eigen::Vector3d::Ones()));
}

TEST(LinkABAData, DefaultInitializesToZero)
{
  dynamics::LinkABAData data;

  EXPECT_TRUE(
      data.articulatedInertia.isApprox(dynamics::SpatialInertia::Zero()));
  EXPECT_TRUE(data.biasForce.isApprox(dynamics::SpatialForce::Zero()));
  EXPECT_TRUE(data.spatialVelocity.isApprox(dynamics::SpatialVelocity::Zero()));
  EXPECT_TRUE(
      data.spatialAcceleration.isApprox(dynamics::SpatialAcceleration::Zero()));
  EXPECT_TRUE(
      data.partialAcceleration.isApprox(dynamics::SpatialAcceleration::Zero()));
  EXPECT_TRUE(data.transmittedForce.isApprox(dynamics::SpatialForce::Zero()));
}

TEST(JointABAData, ResizeSetsCorrectDimensions)
{
  dynamics::JointABAData data;

  data.resize(3);

  EXPECT_EQ(data.projectedInertiaInverse.rows(), 3);
  EXPECT_EQ(data.projectedInertiaInverse.cols(), 3);
  EXPECT_EQ(data.totalForce.rows(), 3);
}

TEST(JointABAData, ResizeInitializesToZero)
{
  dynamics::JointABAData data;

  data.resize(2);

  EXPECT_TRUE(data.projectedInertiaInverse.isZero());
  EXPECT_TRUE(data.totalForce.isZero());
}
