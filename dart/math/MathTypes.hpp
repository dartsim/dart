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

#ifndef DART_MATH_MATHTYPES_HPP_
#define DART_MATH_MATHTYPES_HPP_

#include <map>
#include <vector>

#include <Eigen/Dense>

//------------------------------------------------------------------------------
// Types
//------------------------------------------------------------------------------
namespace Eigen {

using Vector6d = Matrix<double, 6, 1>;
using Matrix6d = Matrix<double, 6, 6>;

inline Vector6d compose(const Eigen::Vector3d& _angular,
                        const Eigen::Vector3d& _linear)
{
  Vector6d composition;
  composition << _angular, _linear;
  return composition;
}

} // namespace Eigen

namespace dart {
namespace math {

using Inertia = Eigen::Matrix6d;
using LinearJacobian = Eigen::Matrix<double, 3, Eigen::Dynamic>;
using AngularJacobian = Eigen::Matrix<double, 3, Eigen::Dynamic>;
using Jacobian = Eigen::Matrix<double, 6, Eigen::Dynamic>;

}  // namespace math
}  // namespace dart

#endif  // DART_MATH_MATHTYPES_HPP_
