/*
 * Copyright (c) 2015-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2015-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#ifndef DART_COLLISION_FCL_FCLTTYPES_HPP_
#define DART_COLLISION_FCL_FCLTTYPES_HPP_

#include <Eigen/Dense>
#include <fcl/math/vec_3f.h>
#include <fcl/math/matrix_3f.h>
#include <fcl/math/transform.h>

#define FCL_VERSION_AT_LEAST(x,y,z) \
  (FCL_MAJOR_VERSION > x || (FCL_MAJOR_VERSION >= x && \
  (FCL_MINOR_VERSION > y || (FCL_MINOR_VERSION >= y && \
  FCL_PATCH_VERSION >= z))))

#define FCL_MAJOR_MINOR_VERSION_AT_MOST(x,y) \
  (FCL_MAJOR_VERSION < x || (FCL_MAJOR_VERSION <= x && \
  (FCL_MINOR_VERSION < y || (FCL_MINOR_VERSION <= y))))

#if FCL_VERSION_AT_LEAST(0,5,0)
#include <memory>
template <class T> using fcl_shared_ptr = std::shared_ptr<T>;
template <class T> using fcl_weak_ptr = std::weak_ptr<T>;
#else
template <class T> using fcl_shared_ptr = boost::shared_ptr<T>;
template <class T> using fcl_weak_ptr = boost::weak_ptr<T>;
#endif

namespace dart {
namespace collision {

class FCLTypes
{
public:
  /// Convert FCL vector3 type to Eigen vector3 type
  static Eigen::Vector3d convertVector3(const fcl::Vec3f& _vec);

  /// Convert Eigen vector3 type to FCL vector3 type
  static fcl::Vec3f convertVector3(const Eigen::Vector3d& _vec);

  /// Convert FCL matrix3x3 type to Eigen matrix3x3 type
  static fcl::Matrix3f convertMatrix3x3(const Eigen::Matrix3d& _R);

  /// Convert FCL transformation type to Eigen transformation type
  static fcl::Transform3f convertTransform(const Eigen::Isometry3d& _T);
};

}  // namespace collision
}  // namespace dart

#endif  // DART_COLLISION_FCL_FCLTTYPES_HPP_
