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

#ifndef DART_GUI_FILAMENT_TYPES_HPP_
#define DART_GUI_FILAMENT_TYPES_HPP_

#include <Eigen/Dense>

#include <math/mat3.h>
#include <math/mat4.h>
#include <math/quat.h>
#include <math/vec3.h>

namespace dart {
namespace gui {
namespace flmt {

class FilamentTypes
{
public:
  static Eigen::Vector3d convertVector3d(const ::math::double3& vec3)
  {
    return Eigen::Map<const Eigen::Vector3d>(vec3.v, 3);
  }

  static Eigen::Matrix3d convertMatrix3d(const ::math::mat3& mat3)
  {
    Eigen::Matrix3d ret;
    ret << mat3(0, 0), mat3(1, 0), mat3(2, 0), mat3(0, 1), mat3(1, 1),
        mat3(2, 1), mat3(0, 2), mat3(1, 2), mat3(2, 2);

    return ret;
  }

  static Eigen::Isometry3d convertIsometry3d(const ::math::mat4& mat4)
  {
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    tf.linear() = convertMatrix3d(mat4.upperLeft());
    tf.translation() = Eigen::Vector3d(mat4(0, 3), mat4(1, 3), mat4(2, 3));

    return tf;
  }

  static ::math::mat4f convertIsometry3d(const Eigen::Isometry3d& mat4)
  {
    ::math::mat4f tf(
        mat4(0, 0),
        mat4(0, 1),
        mat4(0, 2),
        mat4(0, 3),
        mat4(1, 0),
        mat4(1, 1),
        mat4(1, 2),
        mat4(1, 3),
        mat4(2, 0),
        mat4(2, 1),
        mat4(2, 2),
        mat4(2, 3),
        0,
        0,
        0,
        1);

    return tf;
  }
};

} // namespace flmt
} // namespace gui
} // namespace dart

#endif // DART_GUI_FILAMENT_TYPES_HPP_
