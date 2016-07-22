/*
 * Copyright (c) 2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Humanoid Lab, Georgia Tech Research Corporation
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

#ifndef DART_MATH_CONFIGURATIONSPACE_HPP_
#define DART_MATH_CONFIGURATIONSPACE_HPP_

#include <Eigen/Dense>
#include "dart/math/MathTypes.hpp"
#include "dart/math/Geometry.hpp"

namespace dart {
namespace math {

//==============================================================================
template <std::size_t Dimension>
struct RealVectorSpace
{
  enum : std::size_t { NumDofs = Dimension };
  enum : int { NumDofsEigen = Dimension };

  using TangentSpace = RealVectorSpace<NumDofs>;

  using Point          = Eigen::Matrix<double, NumDofs, 1>;
  using EuclideanPoint = Eigen::Matrix<double, NumDofs, 1>;
  using Vector         = Eigen::Matrix<double, NumDofs, 1>;
  using Matrix         = Eigen::Matrix<double, NumDofs, NumDofs>;
  using JacobianMatrix = Eigen::Matrix<double, 6, NumDofs>;
};

using NullSpace = RealVectorSpace<0u>;
using R1Space = RealVectorSpace<1u>;
using R2Space = RealVectorSpace<2u>;
using R3Space = RealVectorSpace<3u>;

//==============================================================================
struct SO3Space
{
  enum : std::size_t { NumDofs = 3u };
  enum : int { NumDofsEigen = 3 };

  using TangentSpace = RealVectorSpace<NumDofs>;

  using Point          = Eigen::Matrix3d;
  using EuclideanPoint = Eigen::Vector3d;
  using Vector         = Eigen::Vector3d;
  using Matrix         = Eigen::Matrix3d;
  using JacobianMatrix = Eigen::Matrix<double, 6, NumDofs>;
};

//==============================================================================
struct SE3Space
{
  enum : std::size_t { NumDofs = 6u };
  enum : int { NumDofsEigen = 6 };

  using TangentSpace = RealVectorSpace<NumDofs>;

  using Point          = Eigen::Isometry3d;
  using EuclideanPoint = Eigen::Vector6d;
  using Vector         = Eigen::Vector6d;
  using Matrix         = Eigen::Matrix6d;
  using JacobianMatrix = Eigen::Matrix6d;
};

struct MapsToManifoldPoint {};


//==============================================================================
template <typename SpaceT>
typename SpaceT::Matrix inverse(const typename SpaceT::Matrix& mat);

//==============================================================================
template <typename SpaceT>
typename SpaceT::EuclideanPoint
toEuclideanPoint(const typename SpaceT::Point& point);

//==============================================================================
template <typename SpaceT>
typename SpaceT::Point
toManifoldPoint(const typename SpaceT::EuclideanPoint& point);

//==============================================================================
template <typename SpaceT>
typename SpaceT::Point integratePosition(
    const typename SpaceT::Point& pos,
    const typename SpaceT::Vector& vel,
    double dt);

//==============================================================================
template <typename SpaceT>
typename SpaceT::Vector integrateVelocity(
    const typename SpaceT::Vector& vel,
    const typename SpaceT::Vector& acc,
    double dt);

} // namespace math
} // namespace dart

#include "dart/math/detail/ConfigurationSpace.hpp"

#endif // DART_MATH_CONFIGURATIONSPACE_HPP_
