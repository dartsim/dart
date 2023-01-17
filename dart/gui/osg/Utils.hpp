/*
 * Copyright (c) 2011-2023, The DART development contributors
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

#ifndef DART_GUI_OSG_UTILS_HPP_
#define DART_GUI_OSG_UTILS_HPP_

#include <dart/gui/Fwd.hpp>

#include <osg/Camera>
#include <osg/Matrix>

namespace dart::gui::osg {

/// Returns the alpha threshold for demining if the object is a transparent
/// object or not
template <typename T = double>
constexpr T getAlphaThreshold();

/// Converts math::Isometry to osg::Matrix
template <typename Scalar>
::osg::Matrix eigToOsgMatrix(const math::Isometry3<Scalar>& tf);

/// Converts math::DenseBase to osg::Matrix
template <typename Derived>
::osg::Matrix eigToOsgMatrix(const math::DenseBase<Derived>& M);

/// Converts math::MatrixBase to osg::Vec3f
template <typename Derived>
::osg::Vec3f eigToOsgVec3f(const math::MatrixBase<Derived>& vec);

/// Converts math::MatrixBase to osg::Vec3d
template <typename Derived>
::osg::Vec3d eigToOsgVec3d(const math::MatrixBase<Derived>& vec);

/// Converts math::MatrixBase to osg::Vec3f or osg::Vec3d based on the scalar
/// type
template <typename Derived>
typename std::conditional<
    std::is_same<typename Derived::Scalar, float>::value,
    ::osg::Vec3f,
    ::osg::Vec3d>::type
eigToOsgVec3(const math::MatrixBase<Derived>& vec);

/// Converts osg::Vec3f to math::Vector3f
math::Vector3f osgToEigVec3(const ::osg::Vec3f& vec);

/// Converts osg::Vec3d to math::Vector3d
math::Vector3d osgToEigVec3(const ::osg::Vec3d& vec);

/// Converts math::MatrixBase to osg::Vec4f
template <typename Derived>
::osg::Vec4f eigToOsgVec4f(const math::MatrixBase<Derived>& vec);

/// Converts math::MatrixBase to osg::Vec4d
template <typename Derived>
::osg::Vec4d eigToOsgVec4d(const math::MatrixBase<Derived>& vec);

/// Converts math::MatrixBase to osg::Vec4f or osg::Vec4d based on the scalar
/// type
template <typename Derived>
std::conditional<
    std::is_same<typename Derived::Scalar, float>::value,
    ::osg::Vec4f,
    ::osg::Vec4d>
eigToOsgVec4(const math::MatrixBase<Derived>& vec);

/// Converts osg::Vec4f to math::Vector4f
math::Vector4f osgToEigVec4(const ::osg::Vec4f& vec);

/// Converts osg::Vec4d to math::Vector4d
math::Vector4d osgToEigVec4(const ::osg::Vec4d& vec);

/// Create a Render-To-Texture (RTT) camera.
::osg::Camera* createRttCamera(
    ::osg::Camera::BufferComponent buffer,
    ::osg::Texture* tex,
    bool isAbsolute = false);

/// Creates a head-up display (HUD) camera that renders on the top after the
/// main scene is drawn, which is generally used for heads-up display
::osg::Camera* createHudCamera(
    double left = 0, double right = 1, double bottom = 0, double top = 1);

/// Creates a osg::Geode of quad shape
::osg::Geode* createScreenQuad(float width, float height, float scale = 1.0f);

} // namespace dart::gui::osg

#include <dart/gui/osg/detail/Utils-impl.hpp>

#endif // DART_GUI_OSG_UTILS_HPP_
