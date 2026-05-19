/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
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

#include "dart/gui/vsg/conversions.hpp"

namespace dart::gui::vsg {

::vsg::dmat4 toVsg(const Eigen::Isometry3d& transform)
{
  const Eigen::Matrix4d m = transform.matrix();
  return ::vsg::dmat4(
      m(0, 0),
      m(1, 0),
      m(2, 0),
      m(3, 0),
      m(0, 1),
      m(1, 1),
      m(2, 1),
      m(3, 1),
      m(0, 2),
      m(1, 2),
      m(2, 2),
      m(3, 2),
      m(0, 3),
      m(1, 3),
      m(2, 3),
      m(3, 3));
}

::vsg::dvec3 toVsg(const Eigen::Vector3d& vec)
{
  return ::vsg::dvec3(vec.x(), vec.y(), vec.z());
}

::vsg::dvec4 toVsg(const Eigen::Vector4d& vec)
{
  return ::vsg::dvec4(vec.x(), vec.y(), vec.z(), vec.w());
}

Eigen::Isometry3d toEigen(const ::vsg::dmat4& mat)
{
  Eigen::Matrix4d m;
  m << mat(0, 0), mat(0, 1), mat(0, 2), mat(0, 3), mat(1, 0), mat(1, 1),
      mat(1, 2), mat(1, 3), mat(2, 0), mat(2, 1), mat(2, 2), mat(2, 3),
      mat(3, 0), mat(3, 1), mat(3, 2), mat(3, 3);
  Eigen::Isometry3d transform;
  transform.matrix() = m;
  return transform;
}

Eigen::Vector3d toEigen(const ::vsg::dvec3& vec)
{
  return Eigen::Vector3d(vec.x, vec.y, vec.z);
}

::vsg::ref_ptr<::vsg::MatrixTransform> createTransform(
    const Eigen::Isometry3d& tf)
{
  auto transform = ::vsg::MatrixTransform::create();
  transform->matrix = toVsg(tf);
  return transform;
}

} // namespace dart::gui::vsg
