/*
 * Copyright (c) 2013-2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
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
  // TODO(JS): Not implemented yet
  void draw(
      renderer::RenderInterface* _ri = nullptr,
      const Eigen::Vector4d& _col = Eigen::Vector4d::Ones()) const override;

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
  void updateVolume() override;

  /// Plane normal
  Eigen::Vector3d mNormal;

  /// Plane offset
  double mOffset;

public:
  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_PLANESHAPE_HPP_
