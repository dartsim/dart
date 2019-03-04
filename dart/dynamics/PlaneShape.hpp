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

#ifndef DART_DYNAMICS_PLANESHAPE_HPP_
#define DART_DYNAMICS_PLANESHAPE_HPP_

#include "dart/dynamics/Shape.hpp"

namespace dart {
namespace dynamics {

/// PlaneShape represents infinite plane has normal and offset as properties.
class PlaneShape : public Shape
{
public:
  /// Constructor
  PlaneShape(const Eigen::Vector3d& _normal, double _offset);

  /// Constructor
  PlaneShape(const Eigen::Vector3d& _normal, const Eigen::Vector3d& _point);

  // Documentation inherited.
  const std::string& getType() const override;

  /// Returns shape type for this class
  static const std::string& getStaticType();

  // Documentation inherited.
  Eigen::Matrix3d computeInertia(double mass) const override;

  /// Set plane normal
  void setNormal(const Eigen::Vector3d& _normal);

  /// Get plane normal
  const Eigen::Vector3d& getNormal() const;

  /// Set plane offset
  void setOffset(double _offset);

  /// Get plane offset
  double getOffset() const;

  /// Set plane normal and offset
  void setNormalAndOffset(const Eigen::Vector3d& _normal, double _offset);

  /// Set plane normal and point
  void setNormalAndPoint(const Eigen::Vector3d& _normal,
                         const Eigen::Vector3d& _point);

  /// Compute distance between the plane and the given point
  double computeDistance(const Eigen::Vector3d& _point) const;

  /// Compute signed distance between the plane and the given point
  double computeSignedDistance(const Eigen::Vector3d& _point) const;

private:
  // Documentation inherited.
  void updateBoundingBox() const override;

  // Documentation inherited.
  void updateVolume() const override;

  /// Plane normal
  Eigen::Vector3d mNormal;

  /// Plane offset
  double mOffset;
};

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_PLANESHAPE_HPP_
