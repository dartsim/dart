/*
 * Copyright (c) 2011-2022, The DART development contributors
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

#ifndef DART_GUI_OSG_DETAIL_UTILS_IMPL_HPP_
#define DART_GUI_OSG_DETAIL_UTILS_IMPL_HPP_

#include "dart/gui/osg/Utils.hpp"

namespace dart::gui::osg {

//==============================================================================
template <typename T>
constexpr T getAlphaThreshold()
{
  if constexpr (std::is_same_v<T, float>)
  {
    return 1e-6;
  }
  else if constexpr (std::is_same_v<T, double>)
  {
    return 1e-9;
  }
  else
  {
    return 1e-9;
  }
}

//==============================================================================
template <typename Scalar>
::osg::Matrix eigToOsgMatrix(
    const Eigen::Transform<Scalar, 3, Eigen::Isometry>& tf)
{
  return ::osg::Matrix(
      tf(0, 0),
      tf(1, 0),
      tf(2, 0),
      tf(3, 0),
      tf(0, 1),
      tf(1, 1),
      tf(2, 1),
      tf(3, 1),
      tf(0, 2),
      tf(1, 2),
      tf(2, 2),
      tf(3, 2),
      tf(0, 3),
      tf(1, 3),
      tf(2, 3),
      tf(3, 3));
}

//==============================================================================
template <typename Derived>
::osg::Matrix eigToOsgMatrix(const Eigen::DenseBase<Derived>& M)
{
  return ::osg::Matrix(
      M(0, 0),
      M(1, 0),
      M(2, 0),
      M(3, 0),
      M(0, 1),
      M(1, 1),
      M(2, 1),
      M(3, 1),
      M(0, 2),
      M(1, 2),
      M(2, 2),
      M(3, 2),
      M(0, 3),
      M(1, 3),
      M(2, 3),
      M(3, 3));
}

//==============================================================================
template <typename Derived>
::osg::Vec3f eigToOsgVec3f(const Eigen::MatrixBase<Derived>& vec)
{
  return ::osg::Vec3f(vec[0], vec[1], vec[2]);
}

//==============================================================================
template <typename Derived>
::osg::Vec3d eigToOsgVec3d(const Eigen::MatrixBase<Derived>& vec)
{
  return ::osg::Vec3d(vec[0], vec[1], vec[2]);
}

//==============================================================================
template <typename Derived>
typename std::conditional<
    std::is_same<typename Derived::Scalar, float>::value,
    ::osg::Vec3f,
    ::osg::Vec3d>::type
eigToOsgVec3(const Eigen::MatrixBase<Derived>& vec)
{
  using Vec3 = typename std::conditional<
      std::is_same<typename Derived::Scalar, float>::value,
      ::osg::Vec3f,
      ::osg::Vec3d>::type;

  return Vec3(vec[0], vec[1], vec[2]);
}

//==============================================================================
template <typename Derived>
::osg::Vec4f eigToOsgVec4f(const Eigen::MatrixBase<Derived>& vec)
{
  return ::osg::Vec4f(vec[0], vec[1], vec[2], vec[3]);
}

//==============================================================================
template <typename Derived>
::osg::Vec4d eigToOsgVec4d(const Eigen::MatrixBase<Derived>& vec)
{
  return ::osg::Vec4d(vec[0], vec[1], vec[2], vec[3]);
}

//==============================================================================
template <typename Derived>
std::conditional<
    std::is_same<typename Derived::Scalar, float>::value,
    ::osg::Vec4f,
    ::osg::Vec4d>
eigToOsgVec4(const Eigen::MatrixBase<Derived>& vec)
{
  return std::conditional<
      std::is_same<typename Derived::Scalar, float>::value,
      ::osg::Vec4f,
      ::osg::Vec4d>(vec[0], vec[1], vec[2], vec[3]);
}

} // namespace dart::gui::osg

#endif // DART_GUI_OSG_DETAIL_UTILS_IMPL_HPP_
