/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
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

#ifndef DART_DYNAMICS_PLANESHAPE_H_
#define DART_DYNAMICS_PLANESHAPE_H_

#include "dart/dynamics/Shape.h"

namespace dart {
namespace dynamics {

class PlaneShape : public Shape {
public:
  /// @brief Constructor.
  PlaneShape(const Eigen::Vector3d& _normal, const Eigen::Vector3d& _point);

  // Documentation inherited.
  // TODO(JS): Not implemented yet
  void draw(renderer::RenderInterface* _ri = NULL,
            const Eigen::Vector4d& _col = Eigen::Vector4d::Ones(),
            bool _default = true) const;

  // Documentation inherited.
  virtual Eigen::Matrix3d computeInertia(double _mass) const;

  /// @brief
  void setNormal(const Eigen::Vector3d& _normal);

  /// @brief
  const Eigen::Vector3d& getNormal() const;

  /// @brief
  void setPoint(const Eigen::Vector3d& _point);

  /// @brief
  const Eigen::Vector3d& getPoint() const;

private:
  // Documentation inherited.
  void computeVolume();

  /// @brief
  Eigen::Vector3d mNormal;

  /// @brief
  Eigen::Vector3d mPoint;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_PLANESHAPE_H_
