/*
 * Copyright (c) 2011-2019, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#ifndef DART_DYNAMICS_DETAIL_BODYNODEASPECT_HPP_
#define DART_DYNAMICS_DETAIL_BODYNODEASPECT_HPP_

#include "dart/dynamics/Entity.hpp"
#include "dart/dynamics/Inertia.hpp"
#include "dart/dynamics/Node.hpp"
#include "dart/common/ProxyAspect.hpp"
#include "dart/common/EmbeddedAspect.hpp"

namespace dart {
namespace dynamics {

class Skeleton;

const double DART_DEFAULT_FRICTION_COEFF = 1.0;
const double DART_DEFAULT_RESTITUTION_COEFF = 0.0;

namespace detail {

//==============================================================================
struct BodyNodeState
{
  /// External spatial force
  Eigen::Vector6d mFext;

  BodyNodeState(const Eigen::Vector6d& Fext = Eigen::Vector6d::Zero());

  virtual ~BodyNodeState() = default;

  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//==============================================================================
struct BodyNodeAspectProperties
{
  /// Name of the Entity
  std::string mName;

  /// Inertia information for the BodyNode
  Inertia mInertia;

  /// Indicates whether this node is collidable;
  bool mIsCollidable;

  /// Coefficient of friction
  double mFrictionCoeff;

  /// Coefficient of restitution
  double mRestitutionCoeff;

  /// Gravity will be applied if true
  bool mGravityMode;

  /// Constructor
  BodyNodeAspectProperties(
      const std::string& name = "BodyNode",
      const Inertia& _inertia = Inertia(),
      bool _isCollidable = true,
      double _frictionCoeff = DART_DEFAULT_FRICTION_COEFF,
      double _restitutionCoeff = DART_DEFAULT_RESTITUTION_COEFF,
      bool _gravityMode = true);

  virtual ~BodyNodeAspectProperties() = default;
};

//==============================================================================
using NodeTypeStateVector = common::CloneableVector< std::unique_ptr<Node::State> >;
using NodeStateMap = std::map< std::type_index, std::unique_ptr<NodeTypeStateVector> >;
using AllNodeStates = common::CloneableMap<NodeStateMap>;

//==============================================================================
using NodeTypePropertiesVector = common::CloneableVector< std::unique_ptr<Node::Properties> >;
using NodePropertiesMap = std::map< std::type_index, std::unique_ptr<NodeTypePropertiesVector> >;
using AllNodeProperties = common::CloneableMap<NodePropertiesMap>;

//==============================================================================
void setAllNodeStates(BodyNode* bodyNode, const AllNodeStates& states);

//==============================================================================
AllNodeStates getAllNodeStates(const BodyNode* bodyNode);

//==============================================================================
void setAllNodeProperties(
    BodyNode* bodyNode, const AllNodeProperties& properties);

//==============================================================================
AllNodeProperties getAllNodeProperties(const BodyNode* bodyNode);

//==============================================================================
using NodeVectorProxyAspectState = common::ProxyCloneable<
    common::Aspect::State, BodyNode, AllNodeStates,
    &setAllNodeStates, &getAllNodeStates>;

//==============================================================================
using NodeVectorProxyAspectProperties = common::ProxyCloneable<
    common::Aspect::Properties, BodyNode, AllNodeProperties,
    &setAllNodeProperties, &getAllNodeProperties>;

//==============================================================================
using NodeVectorProxyAspect = common::ProxyStateAndPropertiesAspect<BodyNode,
    NodeVectorProxyAspectState, NodeVectorProxyAspectProperties>;

//==============================================================================
using BodyNodeCompositeBase = common::EmbedStateAndPropertiesOnTopOf<
    BodyNode, BodyNodeState, BodyNodeAspectProperties,
    common::RequiresAspect<NodeVectorProxyAspect> >;

} // namespace detail
} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_DETAIL_BODYNODEASPECT_HPP_
