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

#include "dart/dynamics/SkeletonViNewtonDrnea.hpp"

#include "dart/dynamics/DegreeOfFreedom.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/BodyNodeVariationalIntegrator.hpp"
#include "dart/dynamics/JointViRiqnDrnea.hpp"

namespace dart {
namespace dynamics {

namespace detail {

//==============================================================================
SkeletonViNewtonDrneaState::SkeletonViNewtonDrneaState()
{
  // Do nothing
}

} // namespace detail

//==============================================================================
SkeletonViNewtonDrnea::SkeletonViNewtonDrnea(
    const StateData& state)
{
  mState = state;
}

//==============================================================================
void SkeletonViNewtonDrnea::initialize()
{
  auto* skel = mComposite;

  for (auto* bodyNode : skel->getBodyNodes())
  {
    auto* aspect = bodyNode->get<BodyNodeViRiqnDrnea>();
    assert(aspect);
    aspect->initialize(skel->getTimeStep());
  }
}

//==============================================================================
void SkeletonViNewtonDrnea::setTolerance(double tol)
{
  mTolerance = tol;
}

//==============================================================================
double SkeletonViNewtonDrnea::getTolerance() const
{
  return mTolerance;
}

//==============================================================================
void SkeletonViNewtonDrnea::setMaxIternation(std::size_t iter)
{
  mMaxIteration = iter;
}

//==============================================================================
std::size_t SkeletonViNewtonDrnea::getMaxIteration() const
{
  return mMaxIteration;
}

//==============================================================================
SkeletonViNewtonDrnea::TerminalCondition
SkeletonViNewtonDrnea::integrate()
{
  auto* skel = mComposite;

  TerminalCondition cond = Invalid;

  // Skip immobile or 0-dof skeleton
  if (!skel->isMobile() || skel->getNumDofs() == 0u)
    return StaticSkeleton;

  const auto squaredTolerance = mTolerance * mTolerance;
  const auto dt = skel->getTimeStep();

  // Initial guess
  skel->computeForwardDynamics();
  const Eigen::VectorXd ddq = skel->getAccelerations();
  const Eigen::VectorXd qCurr = skel->getPositions();
  const Eigen::VectorXd qPrev = getPrevPositions();

  //  Eigen::VectorXd qNext = qCurr;
  Eigen::VectorXd qNext = skel->getPositionDifferences(
        ddq*dt*dt + skel->getPositionDifferences(qCurr, qPrev), -qCurr);

  cond = MaximumIteration;
  for (auto i = 0u; i < mMaxIteration; ++i)
  {
    const Eigen::VectorXd error = evaluateDel(qNext);
    auto squaredNorm = error.squaredNorm();

    if (squaredNorm <= squaredTolerance)
    {
      cond = Tolerance;
      break;
    }

    skel->setJointConstraintImpulses(-dt * error);
    skel->computeImpulseForwardDynamics();
    const Eigen::VectorXd delV = skel->getVelocityChanges();
    qNext = qNext + delV;
    // TODO: generalize for non Eucleadian joint spaces as well
  }

  stepForward(qNext);

  return cond;
}

//==============================================================================
void SkeletonViNewtonDrnea::setComposite(
    common::Composite* newComposite)
{
  Base::setComposite(newComposite);

  auto* skel = mComposite;
//  const auto numDofs = skel->getNumDofs();

  assert(skel);

//  mState.mDM_GradientKineticEnergy_q.resize(numDofs);

  for (auto* bodyNode : skel->getBodyNodes())
  {
    auto* aspect = bodyNode->getOrCreateAspect<BodyNodeViRiqnDrnea>();
    aspect->initialize(skel->getTimeStep());
  }
}

//==============================================================================
void SkeletonViNewtonDrnea::loseComposite(common::Composite* oldComposite)
{
  Base::loseComposite(oldComposite);
}

//==============================================================================
void SkeletonViNewtonDrnea::setPrevPositions(
    const Eigen::VectorXd& prevPositions)
{
  auto* skel = mComposite;
  assert(skel->getNumDofs() == static_cast<std::size_t>(prevPositions.size()));

  auto index = 0u;
  for (auto* bodyNode : skel->getBodyNodes())
  {
    auto* aspect = bodyNode->get<BodyNodeViRiqnDrnea>();
    assert(aspect);
    auto* joint = bodyNode->getParentJoint();
    const auto numDofs = joint->getNumDofs();

    aspect->getJointVi()->setPrevPositions(
          prevPositions.segment(index, numDofs));

    index += numDofs;
  }
}

//==============================================================================
Eigen::VectorXd SkeletonViNewtonDrnea::getPrevPositions() const
{
  auto* skel = mComposite;
  const auto numDofs = skel->getNumDofs();

  Eigen::VectorXd positions;
  positions.resize(numDofs);

  auto index = 0u;
  for (auto* bodyNode : skel->getBodyNodes())
  {
    auto* bodyNodeVi = bodyNode->get<BodyNodeViRiqnDrnea>();
    assert(bodyNodeVi);
    auto* jointVi = bodyNodeVi->getJointVi();
    const auto numJointDofs = bodyNode->getParentJoint()->getNumDofs();

    positions.segment(index, numJointDofs) = jointVi->getPrevPositions();

    index += numJointDofs;
  }

  return positions;
}

//==============================================================================
void SkeletonViNewtonDrnea::setNextPositions(
    const Eigen::VectorXd& nextPositions)
{
  auto* skel = mComposite;
  assert(skel->getNumDofs() == static_cast<std::size_t>(nextPositions.size()));

  auto index = 0u;
  for (auto* bodyNode : skel->getBodyNodes())
  {
    auto* aspect = bodyNode->get<BodyNodeViRiqnDrnea>();
    assert(aspect);
    auto* joint = bodyNode->getParentJoint();
    const auto numDofs = joint->getNumDofs();

    aspect->getJointVi()->setNextPositions(
          nextPositions.segment(index, numDofs));

    index += numDofs;
  }
}

//==============================================================================
Eigen::VectorXd SkeletonViNewtonDrnea::evaluateDel(
    const Eigen::VectorXd& nextPositions)
{
  // Implementation of Algorithm 2 of "A linear-time variational integrator for
  // multibody systems" (WAFR 2016).

  auto* skel = mComposite;
  const auto timeStep = skel->getTimeStep();
  const Eigen::Vector3d& gravity = skel->getGravity();

  setNextPositions(nextPositions);

  // Forward recursion: line 1 to 5 of Algorithm 2
  for (auto* bodyNode : skel->getBodyNodes())
  {
    auto* bodyNodeVi = bodyNode->get<BodyNodeViRiqnDrnea>();
    assert(bodyNodeVi);

    bodyNodeVi->updateNextTransform();
    bodyNodeVi->updateNextVelocity(timeStep);
  }

  // Backward recursion: line 6 to 9 of Algorithm 2
  for (auto it = skel->getBodyNodes().rbegin();
       it != skel->getBodyNodes().rend(); ++it)
  {
    auto* bodyNode = *it;
    auto* bodyNodeVi = bodyNode->get<BodyNodeViRiqnDrnea>();

    bodyNodeVi->evaluateDel(gravity, timeStep);
  }

  return getError();
}

//==============================================================================
Eigen::MatrixXd SkeletonViNewtonDrnea::evaluateDelDeriv(
    const Eigen::VectorXd& /*nextPositions*/)
{
  // Implementation of Algorithm 4 of "A linear-time variational integrator for
  // multibody systems" (WAFR 2016).

  auto* skel = mComposite;
  const auto numDofs = skel->getNumDofs();

  Eigen::MatrixXd J(numDofs, numDofs);

  const auto timeStep = skel->getTimeStep();
//  const Eigen::Vector3d& gravity = skel->getGravity();

  //setNextPositions(nextPositions);

  for (auto i = 0u; i < numDofs; ++i)
  {

    // Forward recursion: line 1 to 5 of Algorithm 4
    for (auto* bodyNode : skel->getBodyNodes())
    {
      auto* bodyNodeVi = bodyNode->get<BodyNodeViRiqnDrnea>();
      assert(bodyNodeVi);

      bodyNodeVi->updateNextTransformDeriv();
      bodyNodeVi->updateNextVelocityDeriv(timeStep);
    }

    //  // Backward recursion: line 6 to 9 of Algorithm 2
    //  for (auto it = skel->getBodyNodes().rbegin();
    //       it != skel->getBodyNodes().rend(); ++it)
    //  {
    //    auto* bodyNode = *it;
    //    auto* bodyNodeVi = bodyNode->get<BodyNodeViRiqnDrnea>();

    //    bodyNodeVi->evaluateDel(gravity, timeStep);
    //  }

  }

  return J;
}

//==============================================================================
Eigen::VectorXd SkeletonViNewtonDrnea::getError() const
{
  auto* skel = mComposite;
  const auto numDofs = skel->getNumDofs();

  Eigen::VectorXd fdel(numDofs);

  auto index = 0u;
  for (auto* bodyNode : skel->getBodyNodes())
  {
    auto* bodyNodeVi = bodyNode->get<BodyNodeViRiqnDrnea>();
    assert(bodyNodeVi);
    auto* jointVi = bodyNodeVi->getJointVi();
    const auto numJointDofs = bodyNode->getParentJoint()->getNumDofs();

    fdel.segment(index, numJointDofs) = jointVi->getError();

    index += numJointDofs;
  }

  return fdel;
}

//==============================================================================
void SkeletonViNewtonDrnea::stepForward(
    const Eigen::VectorXd& nextPositions)
{
  auto* skel = mComposite;
  const auto timeStep = skel->getTimeStep();

  // Update previous/current positions and velocities
  //setVelocities( (qNext - getPositions()) / getTimeStep() );
  skel->setVelocities(
      skel->getPositionDifferences(nextPositions, skel->getPositions())
      / timeStep);
  // TODO(JS): the displacement of geometric joints (e.g., BallJoint and
  // FreeJoint) should be calculated on the geometric space rather than
  // Euclidean space.
  setPrevPositions(skel->getPositions());
  skel->setPositions(nextPositions);
  // q, dq should be updated to get proper prediction from the continuous
  // forward dynamics algorithm.
  // TODO(JS): improve the performance here

  // Update previous spatial velocity and momentum of the bodies
  for (auto* bodyNode : skel->getBodyNodes())
  {
    auto* bodyNodeVi = bodyNode->get<BodyNodeViRiqnDrnea>();
    assert(bodyNodeVi);

    bodyNodeVi->mState.mPreAverageVelocity
        = bodyNodeVi->mState.mPostAverageVelocity;
    bodyNodeVi->mState.mPrevMomentum = bodyNodeVi->mState.mPostMomentum;
  }
}

} // namespace dynamics
} // namespace dart
