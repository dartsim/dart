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

#ifndef DART_MATH_DETAIL_CONFIGURATIONSPACE_H_
#define DART_MATH_DETAIL_CONFIGURATIONSPACE_H_

#include "dart/math/ConfigurationSpace.hpp"

#include "dart/math/Geometry.hpp"

namespace dart {
namespace math {

namespace detail {

template<bool> struct Range;

//==============================================================================
template <typename MatrixType, int Size, typename Enable = Range<true>>
struct inverseImpl
{
  static void run(const MatrixType& matrix, MatrixType& result)
  {
    result = matrix.ldlt().solve(MatrixType::Identity());
  }
};

//==============================================================================
template <typename MatrixType, int Size>
struct inverseImpl<MatrixType, Size, Range<(0 <= Size && Size <= 4)>>
{
  static void run(const MatrixType& matrix, MatrixType& result)
  {
    result = matrix.inverse();
  }
};

//==============================================================================
template <typename SpaceT>
struct toEuclideanPointImpl
{
  static typename SpaceT::EuclideanPoint run(
      const typename SpaceT::Point& point)
  {
    return point;
  }
};

//==============================================================================
template <>
struct toEuclideanPointImpl<SO3Space>
{
  static typename SO3Space::EuclideanPoint run(
      const typename SO3Space::Point& point)
  {
    return math::logMap(point);
  }
};

//==============================================================================
template <>
struct toEuclideanPointImpl<SE3Space>
{
  static typename SE3Space::EuclideanPoint run(
      const typename SE3Space::Point& point)
  {
    Eigen::Vector6d x;

    x.head<3>() = math::logMap(point.linear());
    x.tail<3>() = point.translation();

    return x;
  }
};

//==============================================================================
template <typename SpaceT>
struct toManifoldPointImpl
{
  static typename SpaceT::Point run(
      const typename SpaceT::EuclideanPoint& point)
  {
    return point;
  }
};

//==============================================================================
template <>
struct toManifoldPointImpl<SO3Space>
{
  static typename SO3Space::Point run(
      const typename SO3Space::EuclideanPoint& point)
  {
    return math::expMapRot(point);
  }
};

//==============================================================================
template <>
struct toManifoldPointImpl<SE3Space>
{
  static typename SE3Space::Point run(
      const typename SE3Space::EuclideanPoint& point)
  {
    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());

    tf.linear() = math::expMapRot(point.head<3>());
    tf.translation() = point.tail<3>();

    return tf;
  }
};

//==============================================================================
template <typename SpaceT>
struct integratePositionImpl
{
  static typename SpaceT::Point run(
      const typename SpaceT::Point& pos,
      const typename SpaceT::Vector& vel,
      double dt)
  {
    return pos + dt * vel;
  }
};

//==============================================================================
template <>
struct integratePositionImpl<SO3Space>
{
  static typename SO3Space::Point run(
      const typename SO3Space::Point& pos,
      const typename SO3Space::Vector& vel,
      double dt)
  {
    return pos * toManifoldPoint<SO3Space>(vel * dt);
  }
};

//==============================================================================
template <>
struct integratePositionImpl<SE3Space>
{
  static typename SE3Space::Point run(
      const typename SE3Space::Point& pos,
      const typename SE3Space::Vector& vel,
      double dt)
  {
    return pos * toManifoldPoint<SE3Space>(vel * dt);
  }
};

} // namespace detail



//==============================================================================
template <typename SpaceT>
typename SpaceT::Matrix inverse(const typename SpaceT::Matrix& mat)
{
  typename SpaceT::Matrix res;

  detail::inverseImpl<typename SpaceT::Matrix, SpaceT::NumDofsEigen>::run(
      mat, res);

  return res;
}

//==============================================================================
template <typename SpaceT>
typename SpaceT::EuclideanPoint
toEuclideanPoint(const typename SpaceT::Point& point)
{
  return detail::toEuclideanPointImpl<SpaceT>::run(point);
}

//==============================================================================
template <typename SpaceT>
typename SpaceT::Point
toManifoldPoint(const typename SpaceT::EuclideanPoint& point)
{
  return detail::toManifoldPointImpl<SpaceT>::run(point);
}

//==============================================================================
template <typename SpaceT>
typename SpaceT::Point integratePosition(
    const typename SpaceT::Point& pos,
    const typename SpaceT::Vector& vel,
    double dt)
{
  return detail::integratePositionImpl<SpaceT>::run(pos, vel, dt);
}

//==============================================================================
template <typename SpaceT>
typename SpaceT::Vector integrateVelocity(
    const typename SpaceT::Vector& vel,
    const typename SpaceT::Vector& acc,
    double dt)
{
  return vel + acc * dt;
}

} // namespace math
} // namespace dart

#endif // DART_MATH_DETAIL_CONFIGURATIONSPACE_H_
