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

#ifndef DART_DYNAMICS_CAPSULESHAPE_HPP_
#define DART_DYNAMICS_CAPSULESHAPE_HPP_

#include "dart/dynamics/Shape.hpp"

namespace dart {
namespace dynamics {

/// CapsuleShape represents a three-dimensional geometric shape consisting of a
/// cylinder with hemispherical ends.
class CapsuleShape : public Shape
{
public:

  /// Constructor.
  /// \param[in] radius Radius of the capsule.
  /// \param[in] height Height of the cylindrical part.
  CapsuleShape(double radius, double height);

  // Documentation inherited.
  const std::string& getType() const override;

  /// Get shape type string for this shape.
  static const std::string& getStaticType();

  /// Get the radius of the capsule.
  double getRadius() const;

  /// Set the radius of the capsule.
  void setRadius(double radius);

  /// Get the height of the cylindrical part.
  double getHeight() const;

  /// Set the height of the cylindrical part.
  void setHeight(double height);

  /// Compute volume from given properties.
  /// \param[in] radius Radius of the capsule.
  /// \param[in] height Height of the cylindrical part.
  static double computeVolume(double radius, double height);

  /// Compute moments of inertia of a capsule
  /// \param[in] radius Radius of the capsule.
  /// \param[in] height Height of the cylindrical part.
  static Eigen::Matrix3d computeInertia(
      double radius, double height, double mass);

  // Documentation inherited.
  Eigen::Matrix3d computeInertia(double mass) const override;

protected:
  // Documentation inherited.
  void updateBoundingBox() const override;

  // Documentation inherited.
  void updateVolume() const override;

private:
  /// Radius of the capsule.
  double mRadius;

  /// Height of the cylindrical part.
  double mHeight;

};

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_CAPSULESHAPE_HPP_
