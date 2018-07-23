/*
 * Copyright (c) 2011-2018, The DART development contributors
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

#ifndef DART_GUI_GLFW_ENTITY_HPP_
#define DART_GUI_GLFW_ENTITY_HPP_

#include <memory>
#include <Eigen/Dense>

namespace dart {
namespace gui {
namespace glfw {

template <typename S>
class Frame
{
public:
  using Isometry3 = Eigen::Transform<S, 3, Eigen::Isometry>;
  using Vector3 = Eigen::Matrix<S, 3, 1>;

  /// Constructor
  Frame(const Isometry3& tf = Isometry3::Identity());

  /// Destructor
  virtual ~Frame() = default;

  /// Sets transform
  void setTransform(const Isometry3& tf);

  /// Returns transform
  const Isometry3& getTransform() const;

  /// Returns translation
  Vector3 getTranslation() const;

  /// Translate the object in world-space
  void translateWorld(const Vector3& v);
  // TODO(JS): Use Frame* instead

  /// Translate the object in local-space
  void translateLocal(const Vector3& v);
  // TODO(JS): Use Frame* instead

  /// Rotate the object in world-space
  void rotateWorld(const Vector3& axis, float angle);
  // TODO(JS): Use Frame* instead

  /// Rotate the object in local-space
  void rotateLocal(const Vector3& axis, float angle);
  // TODO(JS): Use Frame* instead

  /// Rotate around a world-space point
  void rotateAroundWorldPoint(
      const Vector3& axis, float angle, const Vector3& worldPoint);

  /// Rotate around a local-space point
  void rotateAroundLocalPoint(
      const Vector3& axis, float angle, const Vector3& localPoint);

protected:
  using Translation3 = Eigen::Translation<S, 3>;
  using AngleAxis = Eigen::AngleAxis<S>;

  Isometry3 mTransform;
};
// TODO: make this class pure virtual class
// TODO: seperate Entity into Node and Drawable

using Entityd = Frame<double>;
using Entityf = Frame<float>;

} // namespace glfw
} // namespace gui
} // namespace dart

#include "dart/gui/glfw/detail/Entity-impl.hpp"

#endif // DART_GUI_GLFW_ENTITY_HPP_
