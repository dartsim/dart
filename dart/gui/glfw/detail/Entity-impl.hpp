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

#include "dart/gui/glfw/Entity.hpp"

namespace dart {
namespace gui {
namespace glfw {

//==============================================================================
template <typename S>
Frame<S>::Frame(const Isometry3& tf) : mTransform(tf)
{
  // Do nothing
}

//==============================================================================
template <typename S>
auto Frame<S>::getTransform() const -> const Isometry3&
{
  return mTransform;
}

//==============================================================================
template <typename S>
auto Frame<S>::getTranslation() const -> Vector3
{
  return mTransform.translation();
}

//==============================================================================
template <typename S>
void Frame<S>::translateWorld(const Vector3& v)
{
  mTransform = Translation3(v) * mTransform;
}

//==============================================================================
template <typename S>
void Frame<S>::translateLocal(const Vector3& v)
{
  mTransform = mTransform * Translation3(v);
}

//==============================================================================
template <typename S>
void Frame<S>::rotateWorld(const Vector3& axis, float angle)
{
  mTransform = AngleAxis(angle, axis) * mTransform;
}

//==============================================================================
template <typename S>
void Frame<S>::rotateLocal(const Vector3& axis, float angle)
{
  mTransform = mTransform * AngleAxis(angle, axis);
}

//==============================================================================
template <typename S>
void Frame<S>::rotateAroundWorldPoint(
    const Vector3& axis, float angle, const Vector3& worldPoint)
{
  mTransform = Translation3(worldPoint) * AngleAxis(angle, axis)
               * Translation3(-worldPoint) * mTransform;
}

//==============================================================================
template <typename S>
void Frame<S>::rotateAroundLocalPoint(
    const Vector3& axis, float angle, const Vector3& worldPoint)
{
  // Convert the world point into the local coordinate system
  Vector3 localPoint = mTransform.inverse() * worldPoint;

  mTransform = mTransform * Translation3(localPoint) * AngleAxis(angle, axis)
               * Translation3(-localPoint);
}

//==============================================================================
template <typename S>
void Frame<S>::setTransform(const Isometry3& tf)
{
  mTransform = tf;
}

} // namespace glfw
} // namespace gui
} // namespace dart
