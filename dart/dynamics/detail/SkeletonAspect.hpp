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

#ifndef DART_DYNAMICS_DETAIL_SKELETONASPECT_HPP_
#define DART_DYNAMICS_DETAIL_SKELETONASPECT_HPP_

#include "dart/common/Composite.hpp"
#include "dart/common/ProxyAspect.hpp"
#include "dart/common/EmbeddedAspect.hpp"
#include "dart/common/RequiresAspect.hpp"
#include <Eigen/Core>

namespace dart {
namespace dynamics {

class Skeleton;

namespace detail {

//==============================================================================
/// The Properties of this Skeleton which are independent of the components
/// within the Skeleton, such as its BodyNodes and Joints. This does not
/// include any Properties of the Skeleton's Aspects.
struct SkeletonAspectProperties
{
  /// Name of the Skeleton
  std::string mName;

  /// If the skeleton is not mobile, its dynamic effect is equivalent
  /// to having infinite mass. If the configuration of an immobile skeleton is
  /// manually changed, the collision results might not be correct.
  bool mIsMobile;

  /// Gravity vector.
  Eigen::Vector3d mGravity;

  /// Time step for implicit joint damping force.
  double mTimeStep;

  /// True if self collision check is enabled. Use mEnabledAdjacentBodyCheck
  /// to disable collision checks between adjacent bodies.
  bool mEnabledSelfCollisionCheck;

  /// True if self collision check is enabled, including adjacent bodies.
  /// Note: If mEnabledSelfCollisionCheck is false, then this value will be
  /// ignored.
  bool mEnabledAdjacentBodyCheck;

  /// Default constructor
  SkeletonAspectProperties(
      const std::string& _name = "Skeleton",
      bool _isMobile = true,
      const Eigen::Vector3d& _gravity = Eigen::Vector3d(0.0, 0.0, -9.81),
      double _timeStep = 0.001,
      bool _enabledSelfCollisionCheck = false,
      bool _enableAdjacentBodyCheck = false);

  virtual ~SkeletonAspectProperties() = default;
};

//==============================================================================
using BodyNodeStateVector = std::vector<common::Composite::State>;
using BodyNodePropertiesVector = std::vector<common::Composite::Properties>;
using JointStateVector = std::vector<common::Composite::State>;
using JointPropertiesVector = std::vector<common::Composite::Properties>;

//==============================================================================
void setAllBodyNodeStates(Skeleton* skel, const BodyNodeStateVector& states);

//==============================================================================
BodyNodeStateVector getAllBodyNodeStates(const Skeleton* skel);

//==============================================================================
void setAllBodyNodeProperties(
    Skeleton* skel, const BodyNodePropertiesVector& properties);

//==============================================================================
BodyNodePropertiesVector getAllBodyNodeProperties(const Skeleton* skel);

//==============================================================================
using BodyNodeVectorProxyAspectState = common::ProxyCloneable<
    common::Aspect::State, Skeleton, BodyNodeStateVector,
    &setAllBodyNodeStates, &getAllBodyNodeStates>;

//==============================================================================
using BodyNodeVectorProxyAspectProperties = common::ProxyCloneable<
    common::Aspect::Properties, Skeleton, BodyNodePropertiesVector,
    &setAllBodyNodeProperties, &getAllBodyNodeProperties>;

//==============================================================================
using BodyNodeVectorProxyAspect = common::ProxyStateAndPropertiesAspect<Skeleton,
    BodyNodeVectorProxyAspectState, BodyNodeVectorProxyAspectProperties>;

//==============================================================================
void setAllJointStates(Skeleton* skel, const JointStateVector& states);

//==============================================================================
JointStateVector getAllJointStates(const Skeleton* skel);

//==============================================================================
void setAllJointProperties(
    Skeleton* skel, const JointPropertiesVector& properties);

//==============================================================================
JointPropertiesVector getAllJointProperties(const Skeleton* skel);

//==============================================================================
using JointVectorProxyAspectState = common::ProxyCloneable<
    common::Aspect::State, Skeleton, JointStateVector,
    &setAllJointStates, &getAllJointStates>;

//==============================================================================
using JointVectorProxyAspectProperties = common::ProxyCloneable<
    common::Aspect::Properties, Skeleton, JointPropertiesVector,
    &setAllJointProperties, &getAllJointProperties>;

//==============================================================================
using JointVectorProxyAspect = common::ProxyStateAndPropertiesAspect<Skeleton,
    JointVectorProxyAspectState, JointVectorProxyAspectProperties>;

//==============================================================================
using SkeletonProxyAspects = common::RequiresAspect<
    BodyNodeVectorProxyAspect, JointVectorProxyAspect>;

//==============================================================================
using SkeletonAspectBase = common::EmbedPropertiesOnTopOf<
    Skeleton, SkeletonAspectProperties, SkeletonProxyAspects>;

} // namespace detail
} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_DETAIL_SKELETONASPECT_HPP_
