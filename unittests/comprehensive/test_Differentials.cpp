/*
 * Copyright (c) 2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#include <gtest/gtest.h>
#include "dart/dart.hpp"
#include "TestHelpers.hpp"

using namespace dart;

using JacobianMatrix = Eigen::Matrix<double, 6, Eigen::Dynamic>;

void setRandomState(const dynamics::SkeletonPtr& skel);

SkeletonPtr createRandomSkeleton();

Eigen::VectorXd computeNumericalLagrangianWrtQ(
    const dynamics::SkeletonPtr& skel,
    double delta = 1e-12);

Eigen::VectorXd computeNumericalLagrangianWrtDQ(
    const dynamics::SkeletonPtr& skel,
    double delta = 1e-12);

// Compute gradients of spatial velocities of bodies with respect to the joint
// positions using finite difference method
Eigen::aligned_vector<Eigen::Vector6d>
computeNumericalBodyVelocityGradientsWrtQ(
    const dynamics::SkeletonPtr& skel,
    std::size_t withRespectTo,
    double delta = 1e-12);

// Compute gradients of spatial velocities of bodies with respect to the joint
// velocities using finite difference method
Eigen::aligned_vector<Eigen::Vector6d>
computeNumericalBodyVelocityGradientsWrtDQ(
    const dynamics::SkeletonPtr& skel,
    std::size_t withRespectTo,
    double delta = 1e-12);

Eigen::aligned_vector<Eigen::Vector6d>
computeNumericalBodyVelocityHessiansWrtQQ(
    const dynamics::SkeletonPtr& skel,
    std::size_t withRespectTo1,
    std::size_t withRespectTo2,
    double delta = 1e-12);

Eigen::aligned_vector<Eigen::Vector6d>
computeNumericalBodyVelocityHessiansWrtQDQ(
    const dynamics::SkeletonPtr& skel,
    std::size_t withRespectTo1,
    std::size_t withRespectTo2,
    double delta = 1e-12);

Eigen::aligned_vector<Eigen::Vector6d>
computeNumericalBodyVelocityHessiansDQQ(
    const dynamics::SkeletonPtr& skel,
    std::size_t withRespectTo1,
    std::size_t withRespectTo2,
    double delta = 1e-12);

Eigen::aligned_vector<Eigen::Vector6d>
computeNumericalBodyVelocityHessiansWrtDQDQ(
    const dynamics::SkeletonPtr& skel,
    std::size_t withRespectTo1,
    std::size_t withRespectTo2,
    double delta = 1e-12);

//==============================================================================
TEST(Differentials, BodyVelocityGradients)
{
  auto skel = createRandomSkeleton();

  auto* skelDiff = skel->createAspect<SkeletonDifferential>();
  skelDiff->updateBodyVelocityGradients();

  Eigen::aligned_vector<Eigen::Vector6d> bodyVelocityGradients_q_fd;
  Eigen::aligned_vector<Eigen::Vector6d> bodyVelocityGradients_dq_fd;

  for (auto indexWrt = 0u; indexWrt < skel->getNumDofs(); ++indexWrt)
  {
    bodyVelocityGradients_q_fd
        = computeNumericalBodyVelocityGradientsWrtQ(skel, indexWrt, 1e-12);
    bodyVelocityGradients_dq_fd
        = computeNumericalBodyVelocityGradientsWrtDQ(skel, indexWrt, 1e-12);

    for (auto i = 0u; i < skel->getNumDofs(); ++i)
    {
      const auto* bodyNodeDiff
          = skel->getBodyNode(i)->get<BodyNodeDifferential>();

      JacobianMatrix bodyVelocityGradient_q_analytical
          = bodyNodeDiff->getBodyVelocityGradientWrtQ(indexWrt);
      JacobianMatrix bodyVelocityGradient_dq_analytical
          = bodyNodeDiff->getBodyVelocityGradientWrtDQ(indexWrt);

      EXPECT_TRUE(bodyVelocityGradients_q_fd[i].isApprox(
                    bodyVelocityGradient_q_analytical, 1e-1));
      EXPECT_TRUE(bodyVelocityGradients_dq_fd[i].isApprox(
                    bodyVelocityGradient_dq_analytical, 1e-1));
    }
  }
}

//==============================================================================
TEST(Differentials, BodyVelocityHessians)
{
  // TODO(JS): not implemented yet
}

//==============================================================================
TEST(Differentials, LagragianGradients)
{
  auto skel = createNLinkPendulum(5);
  setRandomState(skel);

  auto* skelDiff = skel->createAspect<SkeletonDifferential>();
  EXPECT_TRUE(skelDiff != nullptr);

  auto* skelComposite = skelDiff->getComposite();
  EXPECT_TRUE(skelComposite != nullptr);

  //auto lagGradWrtQ = skelDiff->computeLagrangianGradientWrtPositions();
  //auto lagGradWrtQNumerical = computeNumericalLagrangianWrtQ(skel);
  //EXPECT_TRUE(lagGradWrtQ.isApprox(lagGradWrtQNumerical));
  // TODO(JS): not implemented yet

  //auto lagGradWrtDQ = skelDiff->computeLagrangianGradientWrtVelocities();
  //auto lagGradWrtDQNumerical = computeNumericalLagrangianWrtDQ(skel);
  //EXPECT_TRUE(lagGradWrtDQ.isApprox(lagGradWrtDQNumerical));
  // TODO(JS): not implemented yet
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

//==============================================================================
//
//                              Implementations
//
//==============================================================================

//==============================================================================
void setRandomState(const dynamics::SkeletonPtr& skel)
{
  const auto pi = math::constantsd::pi();
  const auto numDofs = skel->getNumDofs();
  const auto posLower = pi * -0.5;
  const auto posUpper = pi *  0.5;
  const auto velLower = pi * -0.5;
  const auto velUpper = pi *  0.5;

  for (auto i = 0u; i < numDofs; ++i)
  {
    auto dof = skel->getDof(i);

    const auto pos = math::random(posLower, posUpper);
    const auto vel = math::random(velLower, velUpper);

    dof->setPosition(pos);
    dof->setVelocity(vel);
  }
}

//==============================================================================
SkeletonPtr createRandomSkeleton()
{
  const auto numLinks = 25u;
  const auto l = 1.5;
  auto skel = createNLinkRobot(numLinks, Eigen::Vector3d(0.3, 0.3, l), DOF_ROLL);
  setRandomState(skel);

  return skel;
}

//==============================================================================
Eigen::VectorXd computeNumericalLagrangianWrtQ(
    const dynamics::SkeletonPtr& skel, double delta)
{
  const auto numDofs = skel->getNumDofs();
  Eigen::VectorXd grad = Eigen::VectorXd::Zero(numDofs);

  const Eigen::VectorXd oldPositions = skel->getPositions();

  const auto lag = skel->computeLagrangian();

  for (auto i = 0u; i < numDofs; ++i)
  {
    skel->setPositions(oldPositions);
    skel->setPosition(i, oldPositions[i] + delta);
    grad[i] = (skel->computeLagrangian() - lag) / delta;
  }

  skel->setPositions(oldPositions);

  return grad;
}

//==============================================================================
Eigen::VectorXd computeNumericalLagrangianWrtDQ(
    const dynamics::SkeletonPtr& skel, double delta)
{
  const auto numDofs = skel->getNumDofs();
  Eigen::VectorXd grad = Eigen::VectorXd::Zero(numDofs);

  const Eigen::VectorXd oldVelocities = skel->getVelocities();

  const auto lag = skel->computeLagrangian();

  for (auto i = 0u; i < numDofs; ++i)
  {
    skel->setVelocities(oldVelocities);
    skel->setVelocity(i, oldVelocities[i] + delta);
    grad[i] = (skel->computeLagrangian() - lag) / delta;
  }

  skel->setVelocities(oldVelocities);

  return grad;
}

//==============================================================================
Eigen::aligned_vector<Eigen::Vector6d>
computeNumericalBodyVelocityGradientsWrtQ(
    const dynamics::SkeletonPtr& skel,
    std::size_t withRespectTo,
    double delta)
{
  const auto numBodies = skel->getNumBodyNodes();
  Eigen::aligned_vector<Eigen::Vector6d> grads(numBodies);

  const Eigen::VectorXd oldPositions = skel->getPositions();

  Eigen::aligned_vector<Eigen::Vector6d> bodyVelocities(numBodies);
  for (auto i = 0u; i < skel->getNumBodyNodes(); ++i)
    bodyVelocities[i] = skel->getBodyNode(i)->getSpatialVelocity();

  Eigen::VectorXd q = skel->getPositions();
  q[withRespectTo] += delta;
  skel->setPositions(q);

  for (auto i = 0u; i < numBodies; ++i)
  {
    grads[i]
        = (skel->getBodyNode(i)->getSpatialVelocity() - bodyVelocities[i])
        / delta;
  }

  skel->setPositions(oldPositions);

  return grads;
}

//==============================================================================
Eigen::aligned_vector<Eigen::Vector6d>
computeNumericalBodyVelocityGradientsWrtDQ(
    const dynamics::SkeletonPtr& skel,
    std::size_t withRespectTo,
    double delta)
{
  const auto numBodies = skel->getNumBodyNodes();
  Eigen::aligned_vector<Eigen::Vector6d> grads(numBodies);

  const Eigen::VectorXd oldVelocities = skel->getVelocities();

  Eigen::aligned_vector<Eigen::Vector6d> bodyVelocities(numBodies);
  for (auto i = 0u; i < skel->getNumBodyNodes(); ++i)
    bodyVelocities[i] = skel->getBodyNode(i)->getSpatialVelocity();

  Eigen::VectorXd dq = skel->getVelocities();
  dq[withRespectTo] += delta;
  skel->setVelocities(dq);

  for (auto i = 0u; i < numBodies; ++i)
  {
    grads[i]
        = (skel->getBodyNode(i)->getSpatialVelocity() - bodyVelocities[i])
        / delta;
  }

  skel->setVelocities(oldVelocities);

  return grads;
}

//==============================================================================
Eigen::aligned_vector<Eigen::Vector6d>
computeNumericalBodyVelocityHessiansWrtQQ(
    const dynamics::SkeletonPtr& skel,
    std::size_t withRespectTo1,
    std::size_t withRespectTo2,
    double delta)
{
  const auto numBodies = skel->getNumBodyNodes();
  Eigen::aligned_vector<Eigen::Vector6d> hessians(numBodies);

  const Eigen::VectorXd oldPositions = skel->getPositions();

  auto* skelDiff = skel->getOrCreateAspect<SkeletonDifferential>();
  skelDiff->updateBodyVelocityGradients();

  Eigen::aligned_vector<Eigen::Vector6d> bodyVelocityGradient1A(numBodies);
  Eigen::aligned_vector<Eigen::Vector6d> bodyVelocityGradient2A(numBodies);

  for (auto i = 0u; i < numBodies; ++i)
  {
    bodyVelocityGradient1A[i]
        = skelDiff->getBodyVelocityGradientWrtQ(withRespectTo1);
    bodyVelocityGradient2A[i]
        = skelDiff->getBodyVelocityGradientWrtQ(withRespectTo2);
  }

  Eigen::aligned_vector<Eigen::Vector6d> bodyVelocityGradient1B(numBodies);
  Eigen::aligned_vector<Eigen::Vector6d> bodyVelocityGradient2B(numBodies);

  Eigen::VectorXd positions1B = oldPositions;
  Eigen::VectorXd positions2B = oldPositions;

  positions1B[withRespectTo1] += delta;
  positions2B[withRespectTo2] += delta;

  skel->setPositions(positions1B);
  skelDiff->updateBodyVelocityGradients();
  for (auto i = 0u; i < numBodies; ++i)
  {
    bodyVelocityGradient1B[i]
        = skelDiff->getBodyVelocityGradientWrtQ(withRespectTo1);
  }

  skel->setPositions(positions2B);
  skelDiff->updateBodyVelocityGradients();
  for (auto i = 0u; i < numBodies; ++i)
  {
    bodyVelocityGradient2B[i]
        = skelDiff->getBodyVelocityGradientWrtQ(withRespectTo2);
  }

  for (auto i = 0u; i < numBodies; ++i)
  {
    hessians[i]
        = (bodyVelocityGradient1B[i] - bodyVelocityGradient1A[i]) / (2.0*delta)
        + (bodyVelocityGradient2B[i] - bodyVelocityGradient2A[i]) / (2.0*delta);
  }

  skel->setPositions(oldPositions);

  return hessians;
}

//==============================================================================
Eigen::aligned_vector<Eigen::Vector6d>
computeNumericalBodyVelocityHessiansWrtQDQ(
    const dynamics::SkeletonPtr& skel,
    std::size_t withRespectTo1,
    std::size_t withRespectTo2,
    double delta)
{
  const auto numBodies = skel->getNumBodyNodes();
  Eigen::aligned_vector<Eigen::Vector6d> hessians(numBodies);

  const Eigen::VectorXd oldPositions = skel->getPositions();
  const Eigen::VectorXd oldVelocities = skel->getVelocities();

  auto* skelDiff = skel->getOrCreateAspect<SkeletonDifferential>();
  skelDiff->updateBodyVelocityGradients();

  Eigen::aligned_vector<Eigen::Vector6d> bodyVelocityGradient1A(numBodies);
  Eigen::aligned_vector<Eigen::Vector6d> bodyVelocityGradient2A(numBodies);

  for (auto i = 0u; i < numBodies; ++i)
  {
    bodyVelocityGradient1A[i]
        = skelDiff->getBodyVelocityGradientWrtQ(withRespectTo1);
    bodyVelocityGradient2A[i]
        = skelDiff->getBodyVelocityGradientWrtDQ(withRespectTo2);
  }

  Eigen::aligned_vector<Eigen::Vector6d> bodyVelocityGradient1B(numBodies);
  Eigen::aligned_vector<Eigen::Vector6d> bodyVelocityGradient2B(numBodies);

  Eigen::VectorXd positions1B = oldPositions;
  Eigen::VectorXd velocities2B = oldVelocities;

  positions1B[withRespectTo1] += delta;
  velocities2B[withRespectTo2] += delta;

  skel->setPositions(oldPositions);
  skel->setVelocities(velocities2B);
  skelDiff->updateBodyVelocityGradients();
  for (auto i = 0u; i < numBodies; ++i)
  {
    bodyVelocityGradient1B[i]
        = skelDiff->getBodyVelocityGradientWrtQ(withRespectTo1);
  }

  skel->setPositions(positions1B);
  skel->setVelocities(oldVelocities);
  skelDiff->updateBodyVelocityGradients();
  for (auto i = 0u; i < numBodies; ++i)
  {
    bodyVelocityGradient2B[i]
        = skelDiff->getBodyVelocityGradientWrtDQ(withRespectTo2);
  }

  for (auto i = 0u; i < numBodies; ++i)
  {
    hessians[i]
        = (bodyVelocityGradient1B[i] - bodyVelocityGradient1A[i]) / (2.0*delta)
        + (bodyVelocityGradient2B[i] - bodyVelocityGradient2A[i]) / (2.0*delta);
  }

  skel->setPositions(oldPositions);
  skel->setVelocities(oldVelocities);

  return hessians;
}

//==============================================================================
Eigen::aligned_vector<Eigen::Vector6d>
computeNumericalBodyVelocityHessiansDQQ(
    const dynamics::SkeletonPtr& skel,
    std::size_t withRespectTo1,
    std::size_t withRespectTo2,
    double delta)
{
  return computeNumericalBodyVelocityHessiansWrtQDQ(
        skel, withRespectTo2, withRespectTo1, delta);
}

//==============================================================================
Eigen::aligned_vector<Eigen::Vector6d>
computeNumericalBodyVelocityHessiansWrtDQDQ(
    const dynamics::SkeletonPtr& skel,
    std::size_t withRespectTo1,
    std::size_t withRespectTo2,
    double delta)
{
  const Eigen::VectorXd oldVelocities = skel->getVelocities();

  const auto numDofs = skel->getNumDofs();
  Eigen::aligned_vector<Eigen::Vector6d> hessians(numDofs);

  auto* skelDiff = skel->getOrCreateAspect<SkeletonDifferential>();
  skelDiff->updateBodyVelocityGradients();

  Eigen::aligned_vector<Eigen::Vector6d> bodyVelocityGradient1A(numDofs);
  Eigen::aligned_vector<Eigen::Vector6d> bodyVelocityGradient2A(numDofs);

  for (auto i = 0u; i < numDofs; ++i)
  {
    bodyVelocityGradient1A[i]
        = skelDiff->getBodyVelocityGradientWrtQ(withRespectTo1);
    bodyVelocityGradient2A[i]
        = skelDiff->getBodyVelocityGradientWrtQ(withRespectTo2);
  }

  Eigen::aligned_vector<Eigen::Vector6d> bodyVelocityGradient1B(numDofs);
  Eigen::aligned_vector<Eigen::Vector6d> bodyVelocityGradient2B(numDofs);

  Eigen::VectorXd velocities1B = oldVelocities;
  Eigen::VectorXd velocities2B = oldVelocities;

  velocities1B[withRespectTo1] += delta;
  velocities2B[withRespectTo2] += delta;

  skel->setVelocities(velocities1B);
  skelDiff->updateBodyVelocityGradients();
  for (auto i = 0u; i < numDofs; ++i)
  {
    bodyVelocityGradient1B[i]
        = skelDiff->getBodyVelocityGradientWrtDQ(withRespectTo1);
  }

  skel->setVelocities(velocities2B);
  skelDiff->updateBodyVelocityGradients();
  for (auto i = 0u; i < numDofs; ++i)
  {
    bodyVelocityGradient2B[i]
        = skelDiff->getBodyVelocityGradientWrtDQ(withRespectTo2);
  }

  for (auto i = 0u; i < numDofs; ++i)
  {
    hessians[i]
        = (bodyVelocityGradient1B[i] - bodyVelocityGradient1A[i]) / (2.0*delta)
        + (bodyVelocityGradient2B[i] - bodyVelocityGradient2A[i]) / (2.0*delta);
  }

  skel->setVelocities(oldVelocities);

  return hessians;
}
