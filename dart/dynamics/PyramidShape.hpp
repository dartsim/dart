/*
 * Copyright (c) 2011-2022, The DART development contributors
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

#ifndef DART_DYNAMICS_PYRAMIDSHAPE_HPP_
#define DART_DYNAMICS_PYRAMIDSHAPE_HPP_

#include "dart/dynamics/Shape.hpp"

namespace dart {
namespace dynamics {

/// PyramidShape represents a polyhedron formed by connecting a rectangular base
/// and a point (apex) where each base edge and apex form a triangle.
///
/// The origin of the reference frame is at the center of a rectangular that
/// intersect the pyramid and is parallel to the base. The line from the origin
/// to the apex is aligned with the Z-axis while the lateral and the
/// longitudinal lengths of the base are aligned with the X-axis and Y-axis,
/// respectively.
class PyramidShape : public Shape
{
public:
  /// Constructor.
  ///
  /// \param[in] baseWidth Lateral length (along X-axis) of the rectangular
  /// base.
  /// \param[in] baseDepth Longitudinal length (along Y-axis) of the rectangular
  /// base.
  /// \param[in] height Perpendicular height from the base to the apex.
  PyramidShape(double baseWidth, double baseDepth, double height);

  // Documentation inherited.
  const std::string& getType() const override;

  /// Returns shape type string for this shape.
  static const std::string& getStaticType();

  /// Returns the lateral length (algon X-axis) of the base.
  double getBaseWidth() const;

  /// Sets the lateral length (algon X-axis) of the base.
  void setBaseWidth(double width);

  /// Returns the longitudinal length (algon Y-axis) of the base.
  double getBaseDepth() const;

  /// Sets the longitudinal length (algon Y-axis) of the base.
  void setBaseDepth(double depth);

  /// Returns the perpendicular height from the base to the apex.
  double getHeight() const;

  /// Returns the Perpendicular height from the base to the apex.
  void setHeight(double height);

  /// Computes the volume given properties.
  ///
  /// \param[in] baseWidth Lateral length (along X-axis) of the rectangular
  /// base.
  /// \param[in] baseDepth Longitudinal length (along Y-axis) of the rectangular
  /// base.
  /// \param[in] height Perpendicular height from the base to the apex.
  static double computeVolume(
      double baseWidth, double baseDepth, double height);

  // Documentation inherited.
  Eigen::Matrix3d computeInertia(double mass) const override;

  // Documentation inherited.
  ShapePtr clone() const override;

protected:
  // Documentation inherited.
  void updateBoundingBox() const override;

  // Documentation inherited.
  void updateVolume() const override;

private:
  /// Lateral length (algon X-axis) of the base.
  double mBaseWidth;

  /// Longitudinal length (algon Y-axis) of the base.
  double mBaseDepth;

  /// Perpendicular height from the base to the apex.
  double mHeight;
};

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_PYRAMIDSHAPE_HPP_
