/*
 * Copyright (c) 2016, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <mxgrey@gatech.edu>
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

#ifndef DART_DYNAMICS_DETAIL_SKELETONASPECT_H_
#define DART_DYNAMICS_DETAIL_SKELETONASPECT_H_

#include "dart/common/Composite.h"
#include "dart/common/ProxyAspect.h"
#include "dart/common/EmbeddedAspect.h"
#include "dart/common/RequiresAspect.h"
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
      bool _enableAdjacentBodyCheck = false,
      size_t _version = 0);
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

#endif // DART_DYNAMICS_DETAIL_SKELETONASPECT_H_
