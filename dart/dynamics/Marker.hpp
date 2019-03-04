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

#ifndef DART_DYNAMICS_MARKER_HPP_
#define DART_DYNAMICS_MARKER_HPP_

#include <Eigen/Dense>
#include "dart/common/Deprecated.hpp"
#include "dart/dynamics/detail/MarkerAspect.hpp"
#include "dart/dynamics/FixedJacobianNode.hpp"

namespace dart {
namespace dynamics {

class BodyNode;

class Marker final :
    public common::EmbedPropertiesOnTopOf<
      Marker, detail::MarkerProperties,
      FixedJacobianNode>
{
public:

  using ConstraintType = detail::MarkerProperties::ConstraintType;
  static constexpr ConstraintType NO = detail::MarkerProperties::NO;
  static constexpr ConstraintType HARD = detail::MarkerProperties::HARD;
  static constexpr ConstraintType SOFT = detail::MarkerProperties::SOFT;

  using BasicProperties = common::Composite::MakeProperties<
      NameAspect,
      FixedFrame,
      Marker>;

  using Properties = common::Composite::Properties;

  /// Destructor
  virtual ~Marker() = default;

  /// Set the AspectProperties of this Marker
  void setAspectProperties(const AspectProperties& properties);

  /// Get the BodyNode this Marker belongs to
  ///
  /// Deprecated: Use getBodyNodePtr() instead
  DART_DEPRECATED(6.0)
  BodyNode* getBodyNode();

  /// Get the (const) BodyNode this Marker belongs to
  ///
  /// Deprecated: Use getBodyNodePtr() instead
  DART_DEPRECATED(6.0)
  const BodyNode* getBodyNode() const;

  /// Get position of this marker in the parent body node coordinates
  Eigen::Vector3d getLocalPosition() const;

  /// Set position of this marker in the parent body node coordinates
  void setLocalPosition(const Eigen::Vector3d& offset);

  /// Get position in the world coordinates
  Eigen::Vector3d getWorldPosition() const;

  /// Get global unique ID
  int getID() const;

  /// Set constraint type. which will be useful for inverse kinematics
  void setConstraintType(ConstraintType type);

  /// Get constraint type. which will be useful for inverse kinematics
  ConstraintType getConstraintType() const;

  /// Set the color of this Marker
  void setColor(const Eigen::Vector4d& color);

  /// Return color of this Marker
  const Eigen::Vector4d& getColor() const;

  friend class BodyNode;

protected:

  /// Constructor used by BodyNode
  Marker(BodyNode* parent, const BasicProperties& properties);

  // Documentation inherited
  Node* cloneNode(BodyNode* parent) const override;

private:
  /// Unique ID of this marker globally.
  int mID;

  /// Counts the number of markers globally.
  static int msMarkerCount;

};

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_MARKER_HPP_
