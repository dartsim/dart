/*
 * Copyright (c) 2011-2023, The DART development contributors
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

#ifndef DART_GUI_TRACKBALL_HPP_
#define DART_GUI_TRACKBALL_HPP_

#include <dart/gui/Fwd.hpp>

namespace dart {
namespace gui {

/// \brief
class Trackball
{
public:
  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// \brief Default constructor
  Trackball();

  /// \brief Constructor
  Trackball(const math::Vector2d& _center, double _radius);

  /// \brief Set the starting position to the project of (x,y) on the trackball
  void startBall(double _x, double _y);

  /// \brief Update the current rotation to rotate from mStartPos to the
  ///        projection of (x,y) on trackball, then update mStartPos
  void updateBall(double _x, double _y);

  /// \brief Apply the current rotation to openGL env
  void applyGLRotation();

  /// \brief Draw the trackball on screen
  void draw(int _winWidth, int _winHeight);

  /// \brief
  void setTrackball(const math::Vector2d& _center, const double _radius);

  /// \brief
  void setCenter(const math::Vector2d& _center);

  /// \brief
  void setRadius(const double _radius);

  /// \brief
  void setQuaternion(const math::Quaterniond& _q);

  /// \brief
  math::Quaterniond getCurrQuat() const;

  /// \brief
  math::Matrix3d getRotationMatrix() const;

  /// \brief
  math::Vector2d getCenter() const;

  /// \brief
  double getRadius() const;

private:
  /// \brief Project screen coordinate (x,y) to the trackball
  math::Vector3d mouseOnSphere(double _mouseX, double _mouseY) const;

  /// \brief Compute the quaternion that rotates from vector "from" to vector
  ///        "to"
  math::Quaterniond quatFromVectors(
      const math::Vector3d& _from, const math::Vector3d& _to) const;

  /// \brief
  math::Vector2d mCenter;

  /// \brief
  double mRadius;

  /// \brief
  math::Vector3d mStartPos;

  /// \brief
  math::Quaterniond mCurrQuat;
};

} // namespace gui
} // namespace dart

#endif // DART_GUI_TRACKBALL_HPP_
