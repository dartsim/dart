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

Eigen::VectorXd computeNumericalLagrangianGradientWrtPositions(
    const dynamics::SkeletonPtr& skel,
    double delta = 1e-12);

Eigen::VectorXd computeNumericalLagrangianGradientWrtVelocities(
    const dynamics::SkeletonPtr& skel,
    double delta = 1e-12);

Eigen::MatrixXd computeNumericalLagrangianHessianWrtPositions(
    const dynamics::SkeletonPtr& skel,
    double delta = 1e-6);

Eigen::MatrixXd computeNumericalLagrangianHessianWrtPositionsVelocities(
    const dynamics::SkeletonPtr& skel,
    double delta = 1e-6);

Eigen::MatrixXd computeNumericalLagrangianHessianWrtVelocities(
    const dynamics::SkeletonPtr& skel,
    double delta = 1e-6);

// Compute gradients of spatial velocities of bodies with respect to the joint
// positions using finite difference method
Eigen::aligned_vector<Eigen::Vector6d>
computeNumericalSpatialVelocityDerivativesWrtPositions(
    const dynamics::SkeletonPtr& skel,
    std::size_t withRespectTo,
    double delta = 1e-12);

// Compute gradients of spatial velocities of bodies with respect to the joint
// velocities using finite difference method
Eigen::aligned_vector<Eigen::Vector6d>
computeNumericalSpatialVelocityDerivativesWrtVelocities(
    const dynamics::SkeletonPtr& skel,
    std::size_t withRespectTo,
    double delta = 1e-12);

Eigen::aligned_vector<Eigen::Vector6d>
computeNumericalBodyVelocityHessiansWrtPositionsQ(
    const dynamics::SkeletonPtr& skel,
    std::size_t withRespectTo1,
    std::size_t withRespectTo2,
    double delta = 1e-12);

Eigen::aligned_vector<Eigen::Vector6d>
computeNumericalBodyVelocityHessiansWrtPositionsDQ(
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
computeNumericalBodyVelocityHessiansWrtVelocitiesDQ(
    const dynamics::SkeletonPtr& skel,
    std::size_t withRespectTo1,
    std::size_t withRespectTo2,
    double delta = 1e-12);

//==============================================================================
TEST(Derivatives, SpatialVelocityDerivatives)
{
  auto skel = createRandomSkeleton();

  skel->createAspect<SkeletonDerivatives>();

  Eigen::aligned_vector<Eigen::Vector6d> bodyVelocityGradients_q_fd;
  Eigen::aligned_vector<Eigen::Vector6d> bodyVelocityGradients_dq_fd;

  for (auto indexWrt = 0u; indexWrt < skel->getNumDofs(); ++indexWrt)
  {
    bodyVelocityGradients_q_fd
        = computeNumericalSpatialVelocityDerivativesWrtPositions(skel, indexWrt, 1e-12);
    bodyVelocityGradients_dq_fd
        = computeNumericalSpatialVelocityDerivativesWrtVelocities(skel, indexWrt, 1e-12);

    for (auto i = 0u; i < skel->getNumDofs(); ++i)
    {
      const auto* bodyNodeDiff
          = skel->getBodyNode(i)->get<BodyNodeDerivatives>();

      JacobianMatrix bodyVelocityGradient_q_analytical
          = bodyNodeDiff->getSpatialVelocityDerivativeWrtPositions(indexWrt);
      JacobianMatrix bodyVelocityGradient_dq_analytical
          = bodyNodeDiff->getSpatialVelocityDerivativeWrtVelocities(indexWrt);

      EXPECT_TRUE(bodyVelocityGradients_q_fd[i].isApprox(
                    bodyVelocityGradient_q_analytical, 1e-1));
      EXPECT_TRUE(bodyVelocityGradients_dq_fd[i].isApprox(
                    bodyVelocityGradient_dq_analytical, 1e-1));
    }
  }
}

//==============================================================================
TEST(Derivatives, BodyVelocityHessians)
{
  // TODO(JS): not implemented yet
}

//==============================================================================
TEST(Derivatives, LagragianDerivatives)
{
  auto skel = createNLinkPendulum(5);
  skel->setGravity(Eigen::Vector3d::Zero());
  setRandomState(skel);

  auto* skelDeriv = skel->createAspect<SkeletonDerivatives>();
  EXPECT_TRUE(skelDeriv != nullptr);

  auto* skelComposite = skelDeriv->getComposite();
  EXPECT_TRUE(skelComposite != nullptr);

  auto lagGradWrtPositions = skelDeriv->computeLagrangianGradientWrtPositions();
  auto lagGradWrtPositionsNumerical = computeNumericalLagrangianGradientWrtPositions(skel);
  EXPECT_TRUE(lagGradWrtPositions.isApprox(lagGradWrtPositionsNumerical, 1e-2));

  auto lagGradWrtVelocities = skelDeriv->computeLagrangianGradientWrtVelocities();
  auto lagGradWrtVelocitiesNumerical = computeNumericalLagrangianGradientWrtVelocities(skel);
  EXPECT_TRUE(lagGradWrtVelocities.isApprox(lagGradWrtVelocitiesNumerical, 1e-2));
}

//==============================================================================
TEST(Derivatives, LagragianSecondDerivatives)
{
  auto skel = createNLinkPendulum(2);
  skel->setGravity(Eigen::Vector3d::Zero());
  setRandomState(skel);

  auto* skelDeriv = skel->createAspect<SkeletonSecondDerivatives>();
  EXPECT_TRUE(skelDeriv != nullptr);

  auto* skelComposite = skelDeriv->getComposite();
  EXPECT_TRUE(skelComposite != nullptr);

  auto lagrangianHessianWrtPositions = skelDeriv->computeLagrangianHessianWrtPositions();
  auto lagrangianHessianWrtPositionsNumerical = computeNumericalLagrangianHessianWrtPositions(skel);
  EXPECT_TRUE(lagrangianHessianWrtPositions.isApprox(lagrangianHessianWrtPositionsNumerical, 1e-2));

  auto lagrangianHessianWrtPositionsVelocities = skelDeriv->computeLagrangianHessianWrtPositionsVelocities();
  auto lagrangianHessianWrtPositionsVelocitiesNumerical = computeNumericalLagrangianHessianWrtPositionsVelocities(skel);
  EXPECT_TRUE(lagrangianHessianWrtPositionsVelocities.isApprox(lagrangianHessianWrtPositionsVelocitiesNumerical, 1e-2));

  auto lagrangianHessianWrtVelocities = skelDeriv->computeLagrangianHessianWrtVelocities();
  auto lagrangianHessianWrtVelocitiesNumerical = computeNumericalLagrangianHessianWrtVelocities(skel);
  EXPECT_TRUE(lagrangianHessianWrtVelocities.isApprox(lagrangianHessianWrtVelocitiesNumerical, 1e-2));
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
Eigen::VectorXd computeNumericalLagrangianGradientWrtPositions(
    const dynamics::SkeletonPtr& skel, double delta)
{
  const auto numDofs = skel->getNumDofs();
  Eigen::VectorXd grad = Eigen::VectorXd::Zero(numDofs);

  const Eigen::VectorXd oldPositions = skel->getPositions();

  const auto L = skel->computeLagrangian();

  for (auto i = 0u; i < numDofs; ++i)
  {
    skel->setPosition(i, oldPositions[i] + delta);
    grad[i] = (skel->computeLagrangian() - L) / delta;
    skel->setPosition(i, oldPositions[i]);
  }

  skel->setPositions(oldPositions);

  return grad;
}

//==============================================================================
Eigen::VectorXd computeNumericalLagrangianGradientWrtVelocities(
    const dynamics::SkeletonPtr& skel, double delta)
{
  const auto numDofs = skel->getNumDofs();
  Eigen::VectorXd grad = Eigen::VectorXd::Zero(numDofs);

  const Eigen::VectorXd oldVelocities = skel->getVelocities();

  const auto lag = skel->computeLagrangian();

  for (auto i = 0u; i < numDofs; ++i)
  {
    skel->setVelocity(i, oldVelocities[i] + delta);
    grad[i] = (skel->computeLagrangian() - lag) / delta;
    skel->setVelocity(i, oldVelocities[i]);
  }

  skel->setVelocities(oldVelocities);

  return grad;
}

//==============================================================================
Eigen::MatrixXd computeNumericalLagrangianHessianWrtPositions(
    const dynamics::SkeletonPtr& skel, double delta)
{
  const auto numDofs = skel->getNumDofs();
  Eigen::MatrixXd hessian = Eigen::MatrixXd::Zero(numDofs, numDofs);

  const Eigen::VectorXd oldPositions = skel->getPositions();

  for (auto i = 0u; i < numDofs; ++i)
  {
    Eigen::VectorXd grad1
        = computeNumericalLagrangianGradientWrtPositions(skel, delta);
    skel->setPosition(i, oldPositions[i] + delta);
    Eigen::VectorXd grad2
        = computeNumericalLagrangianGradientWrtPositions(skel, delta);
    skel->setPosition(i, oldPositions[i]);
    hessian.col(i) = (grad2 - grad1) / delta;
  }

  skel->setPositions(oldPositions);

  return hessian;
}

//==============================================================================
Eigen::MatrixXd computeNumericalLagrangianHessianWrtPositionsVelocities(
    const dynamics::SkeletonPtr& skel, double delta)
{
  const auto numDofs = skel->getNumDofs();
  Eigen::MatrixXd hessian = Eigen::MatrixXd::Zero(numDofs, numDofs);

  const Eigen::VectorXd oldPositions = skel->getPositions();
  const Eigen::VectorXd oldVelocities = skel->getVelocities();

  const auto L = skel->computeLagrangian();

  for (auto i = 0u; i < numDofs; ++i)
  {
    Eigen::VectorXd iPos = oldPositions;
    iPos[i] += delta;
    skel->setPositions(iPos);
    const auto Li = skel->computeLagrangian();
    skel->setPositions(oldPositions);

    for (auto j = 0u; j < numDofs; ++j)
    {
      Eigen::VectorXd jVel = oldVelocities;
      jVel[j] += delta;
      skel->setVelocities(jVel);
      const auto Lj = skel->computeLagrangian();
      skel->setVelocities(oldVelocities);

//      iPos = oldPositions;
//      iPos[i] += delta;
//      jVel = oldVelocities;
//      jVel[j] += delta;
      skel->setPositions(iPos);
      skel->setVelocities(jVel);
      const auto Lij = skel->computeLagrangian();
      skel->setPositions(oldPositions);
      skel->setVelocities(oldVelocities);

      hessian(i, j) = (Lij - Li - Lj + L) / (delta*delta);
    }
  }

  skel->setPositions(oldPositions);
  skel->setVelocities(oldVelocities);

  return hessian;
}

//==============================================================================
Eigen::MatrixXd computeNumericalLagrangianHessianWrtVelocities(
    const dynamics::SkeletonPtr& skel, double delta)
{
  const auto numDofs = skel->getNumDofs();
  Eigen::MatrixXd hessian = Eigen::MatrixXd::Zero(numDofs, numDofs);

  const Eigen::VectorXd oldVelocities = skel->getVelocities();

  for (auto i = 0u; i < numDofs; ++i)
  {
    Eigen::VectorXd grad1
        = computeNumericalLagrangianGradientWrtVelocities(skel, delta);
    skel->setVelocity(i, oldVelocities[i] + delta);
    Eigen::VectorXd grad2
        = computeNumericalLagrangianGradientWrtVelocities(skel, delta);
    skel->setVelocity(i, oldVelocities[i]);
    hessian.col(i) = (grad2 - grad1) / delta;
  }

  skel->setVelocities(oldVelocities);

  return hessian;
}

//==============================================================================
Eigen::aligned_vector<Eigen::Vector6d>
computeNumericalSpatialVelocityDerivativesWrtPositions(
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
computeNumericalSpatialVelocityDerivativesWrtVelocities(
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
computeNumericalBodyVelocityHessiansWrtPositionsQ(
    const dynamics::SkeletonPtr& skel,
    std::size_t withRespectTo1,
    std::size_t withRespectTo2,
    double delta)
{
  const auto numBodies = skel->getNumBodyNodes();
  Eigen::aligned_vector<Eigen::Vector6d> hessians(numBodies);

  const Eigen::VectorXd oldPositions = skel->getPositions();

  auto* skelDeriv = skel->getOrCreateAspect<SkeletonDerivatives>();

  Eigen::aligned_vector<Eigen::Vector6d> bodyVelocityGradient1A(numBodies);
  Eigen::aligned_vector<Eigen::Vector6d> bodyVelocityGradient2A(numBodies);

  for (auto i = 0u; i < numBodies; ++i)
  {
    bodyVelocityGradient1A[i]
        = skelDeriv->getSpatialVelocityDerivativeWrtPositions(withRespectTo1);
    bodyVelocityGradient2A[i]
        = skelDeriv->getSpatialVelocityDerivativeWrtPositions(withRespectTo2);
  }

  Eigen::aligned_vector<Eigen::Vector6d> bodyVelocityGradient1B(numBodies);
  Eigen::aligned_vector<Eigen::Vector6d> bodyVelocityGradient2B(numBodies);

  Eigen::VectorXd positions1B = oldPositions;
  Eigen::VectorXd positions2B = oldPositions;

  positions1B[withRespectTo1] += delta;
  positions2B[withRespectTo2] += delta;

  skel->setPositions(positions1B);
  for (auto i = 0u; i < numBodies; ++i)
  {
    bodyVelocityGradient1B[i]
        = skelDeriv->getSpatialVelocityDerivativeWrtPositions(withRespectTo1);
  }

  skel->setPositions(positions2B);
  for (auto i = 0u; i < numBodies; ++i)
  {
    bodyVelocityGradient2B[i]
        = skelDeriv->getSpatialVelocityDerivativeWrtPositions(withRespectTo2);
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
computeNumericalBodyVelocityHessiansWrtPositionsDQ(
    const dynamics::SkeletonPtr& skel,
    std::size_t withRespectTo1,
    std::size_t withRespectTo2,
    double delta)
{
  const auto numBodies = skel->getNumBodyNodes();
  Eigen::aligned_vector<Eigen::Vector6d> hessians(numBodies);

  const Eigen::VectorXd oldPositions = skel->getPositions();
  const Eigen::VectorXd oldVelocities = skel->getVelocities();

  auto* skelDeriv = skel->getOrCreateAspect<SkeletonDerivatives>();

  Eigen::aligned_vector<Eigen::Vector6d> bodyVelocityGradient1A(numBodies);
  Eigen::aligned_vector<Eigen::Vector6d> bodyVelocityGradient2A(numBodies);

  for (auto i = 0u; i < numBodies; ++i)
  {
    bodyVelocityGradient1A[i]
        = skelDeriv->getSpatialVelocityDerivativeWrtPositions(withRespectTo1);
    bodyVelocityGradient2A[i]
        = skelDeriv->getSpatialVelocityDerivativeWrtVelocities(withRespectTo2);
  }

  Eigen::aligned_vector<Eigen::Vector6d> bodyVelocityGradient1B(numBodies);
  Eigen::aligned_vector<Eigen::Vector6d> bodyVelocityGradient2B(numBodies);

  Eigen::VectorXd positions1B = oldPositions;
  Eigen::VectorXd velocities2B = oldVelocities;

  positions1B[withRespectTo1] += delta;
  velocities2B[withRespectTo2] += delta;

  skel->setPositions(oldPositions);
  skel->setVelocities(velocities2B);
  for (auto i = 0u; i < numBodies; ++i)
  {
    bodyVelocityGradient1B[i]
        = skelDeriv->getSpatialVelocityDerivativeWrtPositions(withRespectTo1);
  }

  skel->setPositions(positions1B);
  skel->setVelocities(oldVelocities);
  for (auto i = 0u; i < numBodies; ++i)
  {
    bodyVelocityGradient2B[i]
        = skelDeriv->getSpatialVelocityDerivativeWrtVelocities(withRespectTo2);
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
  return computeNumericalBodyVelocityHessiansWrtPositionsDQ(
        skel, withRespectTo2, withRespectTo1, delta);
}

//==============================================================================
Eigen::aligned_vector<Eigen::Vector6d>
computeNumericalBodyVelocityHessiansWrtVelocitiesDQ(
    const dynamics::SkeletonPtr& skel,
    std::size_t withRespectTo1,
    std::size_t withRespectTo2,
    double delta)
{
  const Eigen::VectorXd oldVelocities = skel->getVelocities();

  const auto numDofs = skel->getNumDofs();
  Eigen::aligned_vector<Eigen::Vector6d> hessians(numDofs);

  auto* skelDeriv = skel->getOrCreateAspect<SkeletonDerivatives>();

  Eigen::aligned_vector<Eigen::Vector6d> bodyVelocityGradient1A(numDofs);
  Eigen::aligned_vector<Eigen::Vector6d> bodyVelocityGradient2A(numDofs);

  for (auto i = 0u; i < numDofs; ++i)
  {
    bodyVelocityGradient1A[i]
        = skelDeriv->getSpatialVelocityDerivativeWrtPositions(withRespectTo1);
    bodyVelocityGradient2A[i]
        = skelDeriv->getSpatialVelocityDerivativeWrtPositions(withRespectTo2);
  }

  Eigen::aligned_vector<Eigen::Vector6d> bodyVelocityGradient1B(numDofs);
  Eigen::aligned_vector<Eigen::Vector6d> bodyVelocityGradient2B(numDofs);

  Eigen::VectorXd velocities1B = oldVelocities;
  Eigen::VectorXd velocities2B = oldVelocities;

  velocities1B[withRespectTo1] += delta;
  velocities2B[withRespectTo2] += delta;

  skel->setVelocities(velocities1B);
  for (auto i = 0u; i < numDofs; ++i)
  {
    bodyVelocityGradient1B[i]
        = skelDeriv->getSpatialVelocityDerivativeWrtVelocities(withRespectTo1);
  }

  skel->setVelocities(velocities2B);
  for (auto i = 0u; i < numDofs; ++i)
  {
    bodyVelocityGradient2B[i]
        = skelDeriv->getSpatialVelocityDerivativeWrtVelocities(withRespectTo2);
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
