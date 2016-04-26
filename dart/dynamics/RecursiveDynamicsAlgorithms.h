/*
 * Copyright (c) 2016, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
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

#ifndef DART_DYNAMICS_RECURSIVEDYNAMICSALGORITHMS_H_
#define DART_DYNAMICS_RECURSIVEDYNAMICSALGORITHMS_H_

#include <Eigen/Dense>
#include "dart/common/AspectWithVersion.h"
#include "dart/dynamics/detail/RecursiveDynamicsAlgorithms.h"

namespace dart {
namespace dynamics {

class BodyNode;

//==============================================================================
class HybridDynamicsForBodyNode :
    public common::AspectWithState<
        HybridDynamicsForBodyNode,
        detail::HybridDynamicsStateForBodyNode,
        BodyNode>
{
public:

  using Base = common::AspectWithState<
      HybridDynamicsForBodyNode,
      detail::HybridDynamicsStateForBodyNode,
      BodyNode>;

  /// Constructor
  HybridDynamicsForBodyNode(common::Composite* composite,
                            const StateData& state = StateData());

  /// Copy this Aspect is not safe
  HybridDynamicsForBodyNode(const HybridDynamicsForBodyNode&) = delete;

  DART_COMMON_SET_GET_ASPECT_STATE(Eigen::Vector6d, BiasForce)

  /// Update bias force associated with the articulated body inertia for forward
  /// dynamics.
  /// \param[in] gravity Vector of gravitational acceleration
  /// \param[in] timeStep Rquired for implicit joint stiffness and damping.
  virtual void updateBiasForce(const Eigen::Vector3d& gravity, double timeStep);

  /// Update spatial body acceleration with the partial spatial body
  /// acceleration for inverse dynamics.
//  void updateAccelerationID();

  /// Update spatial body acceleration for forward dynamics.
  void updateAccelerationFD();

  /// Update spatical body velocity change for impluse-based forward dynamics.
  void updateVelocityChangeFD();

  /// Update spatial body force for inverse dynamics.
  ///
  /// The spatial body force is transmitted to this BodyNode from the parent
  /// body through the connecting joint. It is expressed in this BodyNode's
  /// frame.
  void updateTransmittedForceID(const Eigen::Vector3d& gravity,
                                bool withExternalForces = false);

  /// Update spatial body force for forward dynamics.
  ///
  /// The spatial body force is transmitted to this BodyNode from the parent
  /// body through the connecting joint. It is expressed in this BodyNode's
  /// frame.
  void updateTransmittedForceFD();

  /// Update spatial body force for impulse-based forward dynamics.
  ///
  /// The spatial body impulse is transmitted to this BodyNode from the parent
  /// body through the connecting joint. It is expressed in this BodyNode's
  /// frame.
  void updateTransmittedImpulse();
  // TODO: Rename to updateTransmittedImpulseFD if impulse-based inverse
  // dynamics is implemented.

  /// Update the joint force for inverse dynamics.
  void updateJointForceID(double timeStep,
                          bool withDampingForces,
                          bool withSpringForces);

  /// Update the joint force for forward dynamics.
  void updateJointForceFD(double timeStep,
                          bool withDampingForces,
                          bool withSpringForces);

  /// Update the joint impulse for forward dynamics.
  void updateJointImpulseFD();

  /// Update constrained terms due to the constraint impulses for foward
  /// dynamics.
  void updateConstrainedTerms(double timeStep);

};

//==============================================================================
class HybridDynamicsForSoftBodyNode : public HybridDynamicsForBodyNode
{
public:

  /// Constructor
  HybridDynamicsForSoftBodyNode(common::Composite* composite,
                            const StateData& state = StateData());

  /// Copy this Aspect is not safe
  HybridDynamicsForSoftBodyNode(const HybridDynamicsForBodyNode&) = delete;

  // Documentation inherited
  void updateBiasForce(
      const Eigen::Vector3d& gravity, double timeStep) override;

};

} // namespace dynamics
} // namespace dart

#include "dart/dynamics/detail/RecursiveDynamicsAlgorithms.h"

#endif // DART_DYNAMICS_RECURSIVEDYNAMICSALGORITHMS_H_
