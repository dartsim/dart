/*
 * Copyright (c) 2011-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2011-2017, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016-2017, Personal Robotics Lab, Carnegie Mellon University
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

#ifndef DART_MATH_MATHTYPES_HPP_
#define DART_MATH_MATHTYPES_HPP_

#include <map>
#include <vector>

#include <Eigen/Dense>

#include "dart/common/Deprecated.hpp"
#include "dart/common/Memory.hpp"

//------------------------------------------------------------------------------
// Types
//------------------------------------------------------------------------------
namespace Eigen {

typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 6, 6> Matrix6d;

inline Vector6d compose(const Eigen::Vector3d& _angular,
                        const Eigen::Vector3d& _linear)
{
  Vector6d composition;
  composition << _angular, _linear;
  return composition;
}

typedef std::vector<Eigen::Vector3d> EIGEN_V_VEC3D;
typedef std::vector<std::vector<Eigen::Vector3d > > EIGEN_VV_VEC3D;

} // namespace Eigen

namespace dart {

#if EIGEN_VERSION_AT_LEAST(3,2,1) && EIGEN_VERSION_AT_MOST(3,2,8)

template <typename _Tp>
using aligned_vector = std::vector<_Tp,
    dart::common::detail::aligned_allocator_cpp11<_Tp>>;

template <typename _Key, typename _Tp, typename _Compare = std::less<_Key>>
using aligned_map = std::map<_Key, _Tp, _Compare,
    dart::common::detail::aligned_allocator_cpp11<std::pair<const _Key, _Tp>>>;

#else

template <typename _Tp>
using aligned_vector = std::vector<_Tp, Eigen::aligned_allocator<_Tp>>;

template <typename _Key, typename _Tp, typename _Compare = std::less<_Key>>
using aligned_map = std::map<_Key, _Tp, _Compare,
    Eigen::aligned_allocator<std::pair<const _Key, _Tp>>>;

#endif

template <typename _Tp, typename... _Args>
DART_DEPRECATED(6.2)
std::shared_ptr<_Tp> make_aligned_shared(_Args&&... __args)
{
  return ::dart::common::make_aligned_shared<_Tp, _Args...>(
        std::forward<_Args>(__args)...);
}

namespace math {

typedef Eigen::Matrix6d Inertia;
typedef Eigen::Matrix<double, 3, Eigen::Dynamic> LinearJacobian;
typedef Eigen::Matrix<double, 3, Eigen::Dynamic> AngularJacobian;
typedef Eigen::Matrix<double, 6, Eigen::Dynamic> Jacobian;

}  // namespace math
}  // namespace dart

#endif  // DART_MATH_MATHTYPES_HPP_
