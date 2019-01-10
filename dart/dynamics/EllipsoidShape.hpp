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

#ifndef DART_DYNAMICS_ELLIPSOIDSHAPE_HPP_
#define DART_DYNAMICS_ELLIPSOIDSHAPE_HPP_

#include "dart/dynamics/Shape.hpp"

namespace dart {
namespace dynamics {

class EllipsoidShape : public Shape
{
public:
  /// \brief Constructor.
  explicit EllipsoidShape(const Eigen::Vector3d& diameters);
  // TODO(JS): In order to follow the commonly used convention, change the
  // constructor to take radii instead of diameters in DART 7.

  /// \brief Destructor.
  virtual ~EllipsoidShape();

  // Documentation inherited.
  const std::string& getType() const override;

  /// Returns shape type for this class
  static const std::string& getStaticType();

  /// \brief Set diameters of this ellipsoid.
  /// \deprecated Deprecated in 6.2. Please use setDiameters() instead.
  DART_DEPRECATED(6.2)
  void setSize(const Eigen::Vector3d& diameters);

  /// \brief Get diameters of this ellipsoid.
  /// \deprecated Deprecated in 6.2. Please use getDiameters() instead.
  DART_DEPRECATED(6.2)
  const Eigen::Vector3d& getSize() const;

  /// \brief Set diameters of this ellipsoid.
  void setDiameters(const Eigen::Vector3d& diameters);

  /// \brief Get diameters of this ellipsoid.
  const Eigen::Vector3d& getDiameters() const;

  /// Set radii of this ellipsoid.
  void setRadii(const Eigen::Vector3d& radii);

  /// Get radii of this ellipsoid.
  const Eigen::Vector3d getRadii() const;

  /// \brief Compute volume from given properties
  static double computeVolume(const Eigen::Vector3d& diameters);
  // TODO(JS): In order to follow the commonly used convention, change to take
  // radii instead of diameters in DART 7.

  /// \brief Compute moments of inertia of a ellipsoid
  static Eigen::Matrix3d computeInertia(
      const Eigen::Vector3d& diameters, double mass);
  // TODO(JS): In order to follow the commonly used convention, change to take
  // radii instead of diameters in DART 7.

  // Documentation inherited.
  Eigen::Matrix3d computeInertia(double mass) const override;

  /// \brief True if all the radii are exactly eqaul.
  bool isSphere(void) const;

protected:
  // Documentation inherited.
  void updateBoundingBox() const override;

  // Documentation inherited.
  void updateVolume() const override;

private:
  /// \brief Diameters of this ellipsoid
  Eigen::Vector3d mDiameters;
};

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_ELLIPSOIDSHAPE_HPP_
