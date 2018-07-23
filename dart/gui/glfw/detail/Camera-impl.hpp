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

#include "dart/gui/glfw/Camera.hpp"

#include "dart/math/Constants.hpp"

namespace dart {
namespace gui {
namespace glfw {

//==============================================================================
template <typename S>
Camera<S>::Camera() : Frame()
{
  // Set default values
  mFieldOfView = 45.0f;
  mSceneRadius = 1.0f;
  mNearPlane = 0.1f;
  mFarPlane = 10.0f;
  mWidth = 1;
  mHeight = 1;

  // Update the projection matrix
  updateProjectionMatrix();

  mProjectionMatrix.setZero();
}

//==============================================================================
template <typename S>
Camera<S>::~Camera()
{
  // Do nothing
}

//==============================================================================
template <typename S>
void Camera<S>::setDimensions(std::size_t width, std::size_t height)
{
  mWidth = width;
  mHeight = height;
  updateProjectionMatrix();
}

//==============================================================================
template <typename S>
S Camera<S>::getSceneRadius() const
{
  return mSceneRadius;
}

//==============================================================================
template <typename S>
void Camera<S>::setSceneRadius(S radius)
{
  mSceneRadius = radius;
  setClippingPlanes(0.01 * radius, 10.0 * radius);
}

//==============================================================================
template <typename S>
void Camera<S>::setFieldOfView(S fov)
{
  mFieldOfView = fov;
  updateProjectionMatrix();
}

//==============================================================================
template <typename S>
void Camera<S>::setZoom(S fraction)
{
  const Vector3 zoomVector(0, 0, mSceneRadius * fraction);
  translateLocal(zoomVector);
}

//==============================================================================
template <typename S>
void Camera<S>::translate(S dx, S dy, const Vector3& worldPoint)
{
  const Vector3 pointCamera = mTransform.inverse() * worldPoint;

  const S z = -pointCamera[2];

  const S aspect = S(mWidth) / S(mHeight);
  const S top = mNearPlane * tan(mFieldOfView * math::constantsf::pi() / 360.0);
  const S right = top * aspect;

  translateLocal(
      Vector3(
          2.0 * dx * right / mNearPlane * z,
          -2.0 * dy * top / mNearPlane * z,
          0.0));
}

//==============================================================================
template <typename S>
void Camera<S>::setClippingPlanes(S near, S far)
{
  mNearPlane = near;
  mFarPlane = far;
  updateProjectionMatrix();
}

//==============================================================================
template <typename S>
S Camera<S>::getNearClippingPlane() const
{
  return mNearPlane;
}

//==============================================================================
template <typename S>
S Camera<S>::getFarClippingPlane() const
{
  return mFarPlane;
}

//==============================================================================
template <typename S>
std::size_t Camera<S>::getWidth() const
{
  return mWidth;
}

//==============================================================================
template <typename S>
std::size_t Camera<S>::getHeight() const
{
  return mHeight;
}

//==============================================================================
template <typename S>
auto Camera<S>::getProjectionMatrix() const -> const Matrix4&
{
  return mProjectionMatrix;
}

//==============================================================================
template <typename S>
void Camera<S>::updateProjectionMatrix()
{
  const auto pi = math::constants<S>::pi();
  const auto aspect = mWidth / mHeight;

  const auto top = mNearPlane * std::tan((mFieldOfView / 2.0) * (pi / 180.0));
  const S bottom = -top;
  const S left = bottom * aspect;
  const S right = top * aspect;

  const S fx = 2.0 * mNearPlane / (right - left);
  const S fy = 2.0 * mNearPlane / (top - bottom);
  const S fz = -(mFarPlane + mNearPlane) / (mFarPlane - mNearPlane);
  const S fw = -2.0 * mFarPlane * mNearPlane / (mFarPlane - mNearPlane);

  mProjectionMatrix(0, 0) = fx;
  mProjectionMatrix(1, 1) = fy;
  mProjectionMatrix(2, 2) = fz;
  mProjectionMatrix(2, 3) = fw;
  mProjectionMatrix(3, 2) = -1;
}

} // namespace glfw
} // namespace gui
} // namespace dart
