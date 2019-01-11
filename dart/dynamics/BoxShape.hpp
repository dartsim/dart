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

#ifndef DART_DYNAMICS_BOXSHAPE_HPP_
#define DART_DYNAMICS_BOXSHAPE_HPP_

#include "dart/dynamics/Shape.hpp"

namespace dart {
namespace dynamics {

class BoxShape : public Shape
{
public:
  /// \brief Constructor.
  explicit BoxShape(const Eigen::Vector3d& _size);

  /// \brief Destructor.
  virtual ~BoxShape();

  // Documentation inherited.
  const std::string& getType() const override;

  /// Returns shape type for this class
  static const std::string& getStaticType();

  /// \brief Set size of this box.
  void setSize(const Eigen::Vector3d& _size);

  /// \brief Get size of this box.
  const Eigen::Vector3d& getSize() const;

  /// \brief Compute volume from given properties
  static double computeVolume(const Eigen::Vector3d& size);

  /// \brief Compute moments of inertia of a box
  static Eigen::Matrix3d computeInertia(const Eigen::Vector3d& size,
                                        double mass);

  // Documentation inherited.
  Eigen::Matrix3d computeInertia(double mass) const override;

protected:
  // Documentation inherited.
  void updateBoundingBox() const override;

  // Documentation inherited.
  void updateVolume() const override;

private:
  /// \brief Side lengths of the box
  Eigen::Vector3d mSize;
};

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_BOXSHAPE_HPP_
