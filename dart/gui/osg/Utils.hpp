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

#ifndef DART_GUI_OSG_IO_HPP_
#define DART_GUI_OSG_IO_HPP_

#include <Eigen/Geometry>

#include <osg/Matrix>

//==============================================================================
template<typename Scalar>
::osg::Matrix eigToOsgMatrix(const Eigen::Transform<Scalar,3,Eigen::Isometry>& tf)
{
  return ::osg::Matrix(
        tf(0, 0), tf(1, 0), tf(2, 0), tf(3, 0),
        tf(0, 1), tf(1, 1), tf(2, 1), tf(3, 1),
        tf(0, 2), tf(1, 2), tf(2, 2), tf(3, 2),
        tf(0, 3), tf(1, 3), tf(2, 3), tf(3, 3));
}

//==============================================================================
template<typename Derived>
::osg::Matrix eigToOsgMatrix(const Eigen::DenseBase<Derived>& M)
{
  return ::osg::Matrix(
        M(0, 0), M(1, 0), M(2, 0), M(3, 0),
        M(0, 1), M(1, 1), M(2, 1), M(3, 1),
        M(0, 2), M(1, 2), M(2, 2), M(3, 2),
        M(0, 3), M(1, 3), M(2, 3), M(3, 3));
}

//==============================================================================
template<typename Derived>
::osg::Vec3d eigToOsgVec3(const Eigen::MatrixBase<Derived>& vec)
{
  return ::osg::Vec3d(vec[0], vec[1], vec[2]);
}

//==============================================================================
inline Eigen::Vector3d osgToEigVec3(const ::osg::Vec3d& vec)
{
  return Eigen::Vector3d(vec[0], vec[1], vec[2]);
}

//==============================================================================
template<typename Derived>
::osg::Vec4d eigToOsgVec4(const Eigen::MatrixBase<Derived>& vec)
{
  return ::osg::Vec4d(vec[0], vec[1], vec[2], vec[3]);
}

//==============================================================================
inline Eigen::Vector4d osgToEigVec4(const ::osg::Vec4d& vec)
{
  return Eigen::Vector4d(vec[0], vec[1], vec[2], vec[3]);
}

#endif // DART_GUI_OSG_IO_HPP_
