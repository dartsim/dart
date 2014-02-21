/*
 * Copyright (c) 2011-2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sumit Jain <sumit@cc.gatech.edu>
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

#ifndef DART_GUI_TRACKBALL_H_
#define DART_GUI_TRACKBALL_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace dart {
namespace gui {

/// \brief
class Trackball {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// \brief Default constructor
  Trackball();

  /// \brief Constructor
  Trackball(const Eigen::Vector2d& _center, double _radius);

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
  void setTrackball(const Eigen::Vector2d& _center, const double _radius);

  /// \brief
  void setCenter(const Eigen::Vector2d& _center);

  /// \brief
  void setRadius(const double _radius);

  /// \brief
  void setQuaternion(const Eigen::Quaterniond& _q);

  /// \brief
  Eigen::Quaterniond getCurrQuat() const;

  /// \brief
  Eigen::Matrix3d getRotationMatrix() const;

  /// \brief
  Eigen::Vector2d getCenter() const;

  /// \brief
  double getRadius() const;

private:
  /// \brief Project screen coordinate (x,y) to the trackball
  Eigen::Vector3d mouseOnSphere(double _mouseX, double _mouseY) const;

  /// \brief Compute the quaternion that rotates from vector "from" to vector
  ///        "to"
  Eigen::Quaterniond quatFromVectors(const Eigen::Vector3d& _from,
                                     const Eigen::Vector3d& _to) const;

  /// \brief
  Eigen::Vector2d mCenter;

  /// \brief
  double mRadius;

  /// \brief
  Eigen::Vector3d mStartPos;

  /// \brief
  Eigen::Quaterniond mCurrQuat;
};

}  // namespace gui
}  // namespace dart

#endif  // DART_GUI_TRACKBALL_H_
