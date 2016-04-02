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

#ifndef DART_DYNAMICS_DETAIL_BODYNODEPROPERTIES_H_
#define DART_DYNAMICS_DETAIL_BODYNODEPROPERTIES_H_

#include "dart/dynamics/Entity.h"
#include "dart/dynamics/Inertia.h"
#include "dart/dynamics/Node.h"
#include "dart/dynamics/Marker.h"

namespace dart {
namespace dynamics {

const double DART_DEFAULT_FRICTION_COEFF = 1.0;
const double DART_DEFAULT_RESTITUTION_COEFF = 0.0;

namespace detail {

//==============================================================================
struct BodyNodeState
{
  /// Whether the node is currently in collision with another node.
  bool mIsColliding;

  /// External spatial force
  Eigen::Vector6d mFext;

  BodyNodeState(bool isColliding = false,
                const Eigen::Vector6d& Fext = Eigen::Vector6d::Zero());
};

//==============================================================================
struct BodyNodeUniqueProperties
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

  /// Properties of the Markers belonging to this BodyNode
  std::vector<Marker::Properties> mMarkerProperties;

  /// Constructor
  BodyNodeUniqueProperties(
      const std::string& name = "BodyNode",
      const Inertia& _inertia = Inertia(),
      bool _isCollidable = true,
      double _frictionCoeff = DART_DEFAULT_FRICTION_COEFF,
      double _restitutionCoeff = DART_DEFAULT_RESTITUTION_COEFF,
      bool _gravityMode = true);

  virtual ~BodyNodeUniqueProperties() = default;

  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using BodyNodeProperties = BodyNodeUniqueProperties;

//==============================================================================
struct BodyNodeExtendedProperties : BodyNodeProperties
{
  using NodePropertiesVector = common::CloneableVector< std::unique_ptr<Node::Properties> >;
  using NodePropertiesMap = std::map< std::type_index, std::unique_ptr<NodePropertiesVector> >;
  using NodeProperties = common::CloneableMap<NodePropertiesMap>;
  using CompositeProperties = common::Composite::Properties;

  /// Composed constructor
  BodyNodeExtendedProperties(const BodyNodeProperties& standardProperties = BodyNodeProperties(),
      const NodeProperties& nodeProperties = NodeProperties(),
      const CompositeProperties& compositeProperties = CompositeProperties());

  /// Composed move constructor
  BodyNodeExtendedProperties(
      BodyNodeProperties&& standardProperties,
      NodeProperties&& nodeProperties,
      CompositeProperties&& aspectProperties);

  /// Properties of all the Nodes attached to this BodyNode
  NodeProperties mNodeProperties;

  /// Properties of all the Aspects attached to this BodyNode
  CompositeProperties mCompositeProperties;
};

} // namespace detail
} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_DETAIL_BODYNODEPROPERTIES_H_
