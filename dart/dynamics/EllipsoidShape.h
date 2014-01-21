/*
 * Copyright (c) 2011-2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>,
 *            Jeongseok Lee <jslee02@gmail.com>
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

#ifndef DART_DYNAMICS_ELLIPSOIDSHAPE_H_
#define DART_DYNAMICS_ELLIPSOIDSHAPE_H_

#include "dart/dynamics/Shape.h"

namespace dart {
namespace dynamics {

class EllipsoidShape : public Shape {
public:
  /// \brief Constructor.
  explicit EllipsoidShape(const Eigen::Vector3d& _size);

  /// \brief Destructor.
  virtual ~EllipsoidShape();

  /// \brief Set size of this box.
  void setSize(const Eigen::Vector3d& _size);

  /// \brief Get size of this box.
  const Eigen::Vector3d& getSize() const;

  // Documentation inherited.
  void draw(renderer::RenderInterface* _ri = NULL,
            const Eigen::Vector4d& _col = Eigen::Vector4d::Ones(),
            bool _useDefaultColor = true) const;

  // Documentation inherited.
  virtual Eigen::Matrix3d computeInertia(double _mass) const;

  /// \brief True if (mDim[0] == mDim[1] == mDim[2]).
  bool isSphere(void) const;

protected:
  // Documentation inherited.
  void computeVolume();

private:
  /// \brief Size of this ellipsoid
  Eigen::Vector3d mSize;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_ELLIPSOIDSHAPE_H_
