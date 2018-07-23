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

#ifndef DART_GUI_GLFW_CAMERA_HPP_
#define DART_GUI_GLFW_CAMERA_HPP_

#include <Eigen/Dense>
#include "dart/gui/glfw/Entity.hpp"

namespace dart {
namespace gui {
namespace glfw {

template <typename S>
class Camera : public Frame<S>
{
public:
  using Matrix4 = Eigen::Matrix<S, 4, 4>;
  using Vector3 = typename Frame<S>::Vector3;

  /// Constructor
  Camera();

  /// Destructor
  ~Camera() override;

  void setDimensions(std::size_t width, std::size_t height);

  S getSceneRadius() const;

  void setSceneRadius(S radius);

  void setFieldOfView(S fov);

  void setZoom(S fraction);

  void translate(S dx, S dy, const Vector3& worldPoint);

  void setClippingPlanes(S near, S far);

  S getNearClippingPlane() const;

  S getFarClippingPlane() const;

  std::size_t getWidth() const;

  std::size_t getHeight() const;

  const Matrix4& getProjectionMatrix() const;

protected:
  void updateProjectionMatrix();

  S mFieldOfView;

  S mSceneRadius;

  S mNearPlane;

  S mFarPlane;

  std::size_t mWidth;

  std::size_t mHeight;

  Matrix4 mProjectionMatrix;
};

} // namespace glfw
} // namespace gui
} // namespace dart

#include "dart/gui/glfw/detail/Camera-impl.hpp"

#endif // DART_GUI_GLFW_CAMERA_HPP_
