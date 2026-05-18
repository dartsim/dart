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

#ifndef DART_DYNAMICS_DETAIL_SKELETONDYNAMICSVIEW_HPP_
#define DART_DYNAMICS_DETAIL_SKELETONDYNAMICSVIEW_HPP_

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/skeleton.hpp>

#include <span>

namespace dart {
namespace dynamics {
namespace detail {

/// Adapter that exposes the existing Skeleton/BodyNode recursive pass kernels
/// through a compact model interface for articulated dynamics algorithms.
class SkeletonDynamicsView
{
public:
  explicit SkeletonDynamicsView(Skeleton& skeleton) noexcept
    : mSkeleton(skeleton),
      mBodyNodes(
          skeleton.mSkelCache.mBodyNodes.data(),
          skeleton.mSkelCache.mBodyNodes.size())
  {
    // Do nothing.
  }

  std::size_t getNumDofs() const noexcept
  {
    return mSkeleton.getNumDofs();
  }

  std::size_t getNumBodyNodes() const noexcept
  {
    return mBodyNodes.size();
  }

  BodyNode& getBodyNode(std::size_t index) const
  {
    return *mBodyNodes[index];
  }

  const Eigen::Vector3d& getGravity() const noexcept
  {
    return mSkeleton.getGravity();
  }

  double getTimeStep() const noexcept
  {
    return mSkeleton.getTimeStep();
  }

  void updateBiasForce(
      BodyNode& bodyNode, const Eigen::Vector3d& gravity, double timeStep) const
  {
    bodyNode.updateBiasForce(gravity, timeStep);
  }

  void updateAccelerationFD(BodyNode& bodyNode) const
  {
    bodyNode.updateAccelerationFD();
  }

  void updateTransmittedForceFD(BodyNode& bodyNode) const
  {
    bodyNode.updateTransmittedForceFD();
  }

  void updateJointForceFD(
      BodyNode& bodyNode,
      double timeStep,
      bool withDampingForces,
      bool withSpringForces) const
  {
    bodyNode.updateJointForceFD(timeStep, withDampingForces, withSpringForces);
  }

  void updateTransmittedForceID(
      BodyNode& bodyNode,
      const Eigen::Vector3d& gravity,
      bool withExternalForces) const
  {
    bodyNode.updateTransmittedForceID(gravity, withExternalForces);
  }

  void updateJointForceID(
      BodyNode& bodyNode,
      double timeStep,
      bool withDampingForces,
      bool withSpringForces) const
  {
    bodyNode.updateJointForceID(timeStep, withDampingForces, withSpringForces);
  }

private:
  Skeleton& mSkeleton;
  std::span<BodyNode* const> mBodyNodes;
};

} // namespace detail
} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_DETAIL_SKELETONDYNAMICSVIEW_HPP_
