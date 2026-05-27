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
 *     copyright notice and this list of conditions in the documentation
 *     and/or other materials provided with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <limits>

#include <cmath>

namespace dart::simulation::experimental::detail::deformable_contact {

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector9d = Eigen::Matrix<double, 9, 1>;
using Vector12d = Eigen::Matrix<double, 12, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix9d = Eigen::Matrix<double, 9, 9>;
using Matrix12d = Eigen::Matrix<double, 12, 12>;

enum class PointEdgeFeature
{
  PointEdgeStart,
  PointEdgeEnd,
  PointEdgeInterior,
};

enum class PointTriangleFeature
{
  VertexA,
  VertexB,
  VertexC,
  EdgeAB,
  EdgeBC,
  EdgeCA,
  Face,
  Degenerate,
};

enum class EdgeEdgeFeature
{
  EdgeAStartEdgeBStart,
  EdgeAStartEdgeBEnd,
  EdgeAEndEdgeBStart,
  EdgeAEndEdgeBEnd,
  EdgeAInteriorEdgeBStart,
  EdgeAInteriorEdgeBEnd,
  EdgeAStartEdgeBInterior,
  EdgeAEndEdgeBInterior,
  EdgeAInteriorEdgeBInterior,
};

struct PointEdgeDistance
{
  double squaredDistance = 0.0;
  double edgeCoordinate = 0.0;
  Eigen::Vector3d closestPoint = Eigen::Vector3d::Zero();
  PointEdgeFeature feature = PointEdgeFeature::PointEdgeInterior;
};

struct PointTriangleDistance
{
  double squaredDistance = 0.0;
  Eigen::Vector3d barycentric = Eigen::Vector3d::Zero();
  Eigen::Vector3d closestPoint = Eigen::Vector3d::Zero();
  PointTriangleFeature feature = PointTriangleFeature::Face;
};

struct EdgeEdgeDistance
{
  double squaredDistance = 0.0;
  double edgeACoordinate = 0.0;
  double edgeBCoordinate = 0.0;
  Eigen::Vector3d closestPointOnA = Eigen::Vector3d::Zero();
  Eigen::Vector3d closestPointOnB = Eigen::Vector3d::Zero();
  EdgeEdgeFeature feature = EdgeEdgeFeature::EdgeAInteriorEdgeBInterior;
};

namespace detail {

constexpr double kRelativeEpsilon
    = 64.0 * std::numeric_limits<double>::epsilon();
constexpr double kFiniteDifferenceStep = 1e-6;
constexpr double kMollifierThresholdScale = 1e-3;

//==============================================================================
inline bool isDegenerateSegment(const double squaredLength)
{
  return squaredLength <= std::numeric_limits<double>::min();
}

//==============================================================================
inline bool isDegenerateTriangle(
    const double normalSquaredNorm,
    const double edgeASquaredNorm,
    const double edgeBSquaredNorm)
{
  const double scale = std::max(
      edgeASquaredNorm * edgeBSquaredNorm, std::numeric_limits<double>::min());
  return normalSquaredNorm <= kRelativeEpsilon * scale;
}

//==============================================================================
inline bool isParallelEdges(
    const double denominator,
    const double edgeASquaredNorm,
    const double edgeBSquaredNorm)
{
  const double scale = std::max(
      edgeASquaredNorm * edgeBSquaredNorm, std::numeric_limits<double>::min());
  return denominator <= kRelativeEpsilon * scale;
}

//==============================================================================
inline Matrix12d finiteDifferenceHessian(
    const Vector12d& x, const auto& gradientFunction)
{
  Matrix12d hessian = Matrix12d::Zero();

  for (int axis = 0; axis < 12; ++axis) {
    Vector12d forward = x;
    Vector12d backward = x;
    forward[axis] += kFiniteDifferenceStep;
    backward[axis] -= kFiniteDifferenceStep;
    hessian.col(axis) = (gradientFunction(forward) - gradientFunction(backward))
                        / (2.0 * kFiniteDifferenceStep);
  }

  return 0.5 * (hessian + hessian.transpose());
}

//==============================================================================
inline Vector12d stack4(
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c,
    const Eigen::Vector3d& d)
{
  Vector12d x;
  x.segment<3>(0) = a;
  x.segment<3>(3) = b;
  x.segment<3>(6) = c;
  x.segment<3>(9) = d;
  return x;
}

//==============================================================================
inline Eigen::Vector3d segment(const Vector12d& x, const int block)
{
  return x.segment<3>(3 * block);
}

//==============================================================================
inline Matrix12d crossSquaredNormHessianFromEdges(
    const Eigen::Vector3d& u, const Eigen::Vector3d& v)
{
  const double u2 = u.squaredNorm();
  const double v2 = v.squaredNorm();
  const double uv = u.dot(v);

  const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
  const Eigen::Matrix3d hUU = 2.0 * v2 * identity - 2.0 * v * v.transpose();
  const Eigen::Matrix3d hVV = 2.0 * u2 * identity - 2.0 * u * u.transpose();
  const Eigen::Matrix3d hUV
      = 4.0 * u * v.transpose() - 2.0 * v * u.transpose() - 2.0 * uv * identity;
  const Eigen::Matrix3d hVU = hUV.transpose();

  Matrix12d hessian = Matrix12d::Zero();
  const std::array<double, 4> signU = {-1.0, 1.0, 0.0, 0.0};
  const std::array<double, 4> signV = {0.0, 0.0, -1.0, 1.0};

  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      Eigen::Matrix3d block = Eigen::Matrix3d::Zero();
      block += signU[row] * signU[col] * hUU;
      block += signU[row] * signV[col] * hUV;
      block += signV[row] * signU[col] * hVU;
      block += signV[row] * signV[col] * hVV;
      hessian.block<3, 3>(3 * row, 3 * col) = block;
    }
  }

  return 0.5 * (hessian + hessian.transpose());
}

} // namespace detail

//==============================================================================
inline double pointPointSquaredDistance(
    const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
  return (a - b).squaredNorm();
}

//==============================================================================
inline Vector6d pointPointSquaredDistanceGradient(
    const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
  Vector6d gradient;
  gradient.segment<3>(0) = 2.0 * (a - b);
  gradient.segment<3>(3) = -gradient.segment<3>(0);
  return gradient;
}

//==============================================================================
inline Matrix6d pointPointSquaredDistanceHessian()
{
  Matrix6d hessian = Matrix6d::Zero();
  hessian.diagonal().setConstant(2.0);
  hessian.block<3, 3>(0, 3) = -2.0 * Eigen::Matrix3d::Identity();
  hessian.block<3, 3>(3, 0) = -2.0 * Eigen::Matrix3d::Identity();
  return hessian;
}

//==============================================================================
inline PointEdgeDistance pointEdgeSquaredDistance(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b)
{
  PointEdgeDistance result;

  const Eigen::Vector3d edge = b - a;
  const double edgeSquaredNorm = edge.squaredNorm();
  if (detail::isDegenerateSegment(edgeSquaredNorm)) {
    result.edgeCoordinate = 0.0;
    result.closestPoint = a;
    result.squaredDistance = (p - a).squaredNorm();
    result.feature = PointEdgeFeature::PointEdgeStart;
    return result;
  }

  const double unclampedT = edge.dot(p - a) / edgeSquaredNorm;
  result.edgeCoordinate = std::clamp(unclampedT, 0.0, 1.0);
  result.closestPoint = a + result.edgeCoordinate * edge;
  result.squaredDistance = (p - result.closestPoint).squaredNorm();
  if (result.edgeCoordinate <= 0.0) {
    result.feature = PointEdgeFeature::PointEdgeStart;
  } else if (result.edgeCoordinate >= 1.0) {
    result.feature = PointEdgeFeature::PointEdgeEnd;
  } else {
    result.feature = PointEdgeFeature::PointEdgeInterior;
  }
  return result;
}

//==============================================================================
inline Vector9d pointEdgeSquaredDistanceGradient(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b)
{
  const auto distance = pointEdgeSquaredDistance(p, a, b);
  const Eigen::Vector3d residual = p - distance.closestPoint;
  const double t = distance.edgeCoordinate;

  Vector9d gradient = Vector9d::Zero();
  gradient.segment<3>(0) = 2.0 * residual;
  gradient.segment<3>(3) = -2.0 * (1.0 - t) * residual;
  gradient.segment<3>(6) = -2.0 * t * residual;
  return gradient;
}

//==============================================================================
inline Matrix9d pointEdgeSquaredDistanceHessian(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b)
{
  const Vector12d x = detail::stack4(p, a, b, Eigen::Vector3d::Zero());
  const auto hessian12
      = detail::finiteDifferenceHessian(x, [](const Vector12d& value) {
          Vector12d gradient = Vector12d::Zero();
          gradient.head<9>() = pointEdgeSquaredDistanceGradient(
              detail::segment(value, 0),
              detail::segment(value, 1),
              detail::segment(value, 2));
          return gradient;
        });

  return hessian12.topLeftCorner<9, 9>();
}

//==============================================================================
inline PointTriangleDistance pointTriangleSquaredDistance(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c)
{
  PointTriangleDistance result;

  const Eigen::Vector3d ab = b - a;
  const Eigen::Vector3d ac = c - a;
  const Eigen::Vector3d normal = ab.cross(ac);
  if (detail::isDegenerateTriangle(
          normal.squaredNorm(), ab.squaredNorm(), ac.squaredNorm())) {
    const auto abDistance = pointEdgeSquaredDistance(p, a, b);
    const auto bcDistance = pointEdgeSquaredDistance(p, b, c);
    const auto caDistance = pointEdgeSquaredDistance(p, c, a);

    result.feature = PointTriangleFeature::Degenerate;
    result.squaredDistance = abDistance.squaredDistance;
    result.closestPoint = abDistance.closestPoint;
    result.barycentric = Eigen::Vector3d(
        1.0 - abDistance.edgeCoordinate, abDistance.edgeCoordinate, 0.0);

    if (bcDistance.squaredDistance < result.squaredDistance) {
      result.squaredDistance = bcDistance.squaredDistance;
      result.closestPoint = bcDistance.closestPoint;
      result.barycentric = Eigen::Vector3d(
          0.0, 1.0 - bcDistance.edgeCoordinate, bcDistance.edgeCoordinate);
    }
    if (caDistance.squaredDistance < result.squaredDistance) {
      result.squaredDistance = caDistance.squaredDistance;
      result.closestPoint = caDistance.closestPoint;
      result.barycentric = Eigen::Vector3d(
          caDistance.edgeCoordinate, 0.0, 1.0 - caDistance.edgeCoordinate);
    }
    return result;
  }

  const Eigen::Vector3d ap = p - a;
  const double d1 = ab.dot(ap);
  const double d2 = ac.dot(ap);
  if (d1 <= 0.0 && d2 <= 0.0) {
    result.barycentric = Eigen::Vector3d(1.0, 0.0, 0.0);
    result.closestPoint = a;
    result.feature = PointTriangleFeature::VertexA;
  } else {
    const Eigen::Vector3d bp = p - b;
    const double d3 = ab.dot(bp);
    const double d4 = ac.dot(bp);
    if (d3 >= 0.0 && d4 <= d3) {
      result.barycentric = Eigen::Vector3d(0.0, 1.0, 0.0);
      result.closestPoint = b;
      result.feature = PointTriangleFeature::VertexB;
    } else {
      const double vc = d1 * d4 - d3 * d2;
      if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
        const double v = d1 / (d1 - d3);
        result.barycentric = Eigen::Vector3d(1.0 - v, v, 0.0);
        result.closestPoint = a + v * ab;
        result.feature = PointTriangleFeature::EdgeAB;
      } else {
        const Eigen::Vector3d cp = p - c;
        const double d5 = ab.dot(cp);
        const double d6 = ac.dot(cp);
        if (d6 >= 0.0 && d5 <= d6) {
          result.barycentric = Eigen::Vector3d(0.0, 0.0, 1.0);
          result.closestPoint = c;
          result.feature = PointTriangleFeature::VertexC;
        } else {
          const double vb = d5 * d2 - d1 * d6;
          if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
            const double w = d2 / (d2 - d6);
            result.barycentric = Eigen::Vector3d(1.0 - w, 0.0, w);
            result.closestPoint = a + w * ac;
            result.feature = PointTriangleFeature::EdgeCA;
          } else {
            const double va = d3 * d6 - d5 * d4;
            if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
              const double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
              result.barycentric = Eigen::Vector3d(0.0, 1.0 - w, w);
              result.closestPoint = b + w * (c - b);
              result.feature = PointTriangleFeature::EdgeBC;
            } else {
              const double denominator = 1.0 / (va + vb + vc);
              const double v = vb * denominator;
              const double w = vc * denominator;
              result.barycentric = Eigen::Vector3d(1.0 - v - w, v, w);
              result.closestPoint = a + ab * v + ac * w;
              result.feature = PointTriangleFeature::Face;
            }
          }
        }
      }
    }
  }

  result.squaredDistance = (p - result.closestPoint).squaredNorm();
  return result;
}

//==============================================================================
inline Vector12d pointTriangleSquaredDistanceGradient(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c)
{
  const auto distance = pointTriangleSquaredDistance(p, a, b, c);
  const Eigen::Vector3d residual = p - distance.closestPoint;

  Vector12d gradient = Vector12d::Zero();
  gradient.segment<3>(0) = 2.0 * residual;
  gradient.segment<3>(3) = -2.0 * distance.barycentric[0] * residual;
  gradient.segment<3>(6) = -2.0 * distance.barycentric[1] * residual;
  gradient.segment<3>(9) = -2.0 * distance.barycentric[2] * residual;
  return gradient;
}

//==============================================================================
inline Matrix12d pointTriangleSquaredDistanceHessian(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c)
{
  const Vector12d x = detail::stack4(p, a, b, c);
  return detail::finiteDifferenceHessian(x, [](const Vector12d& value) {
    return pointTriangleSquaredDistanceGradient(
        detail::segment(value, 0),
        detail::segment(value, 1),
        detail::segment(value, 2),
        detail::segment(value, 3));
  });
}

//==============================================================================
inline EdgeEdgeDistance edgeEdgeSquaredDistance(
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c,
    const Eigen::Vector3d& d)
{
  EdgeEdgeDistance result;

  const Eigen::Vector3d edgeA = b - a;
  const Eigen::Vector3d edgeB = d - c;
  const Eigen::Vector3d r = a - c;
  const double edgeASquaredNorm = edgeA.squaredNorm();
  const double edgeBSquaredNorm = edgeB.squaredNorm();
  const double edgeBDotR = edgeB.dot(r);

  double s = 0.0;
  double t = 0.0;

  if (detail::isDegenerateSegment(edgeASquaredNorm)
      && detail::isDegenerateSegment(edgeBSquaredNorm)) {
    s = 0.0;
    t = 0.0;
  } else if (detail::isDegenerateSegment(edgeASquaredNorm)) {
    s = 0.0;
    t = std::clamp(edgeBDotR / edgeBSquaredNorm, 0.0, 1.0);
  } else {
    const double edgeADotR = edgeA.dot(r);
    if (detail::isDegenerateSegment(edgeBSquaredNorm)) {
      t = 0.0;
      s = std::clamp(-edgeADotR / edgeASquaredNorm, 0.0, 1.0);
    } else {
      const double edgeADotB = edgeA.dot(edgeB);
      const double denominator
          = edgeASquaredNorm * edgeBSquaredNorm - edgeADotB * edgeADotB;
      if (!detail::isParallelEdges(
              denominator, edgeASquaredNorm, edgeBSquaredNorm)) {
        s = std::clamp(
            (edgeADotB * edgeBDotR - edgeADotR * edgeBSquaredNorm)
                / denominator,
            0.0,
            1.0);
      } else {
        s = 0.0;
      }

      t = (edgeADotB * s + edgeBDotR) / edgeBSquaredNorm;
      if (t < 0.0) {
        t = 0.0;
        s = std::clamp(-edgeADotR / edgeASquaredNorm, 0.0, 1.0);
      } else if (t > 1.0) {
        t = 1.0;
        s = std::clamp((edgeADotB - edgeADotR) / edgeASquaredNorm, 0.0, 1.0);
      }
    }
  }

  result.edgeACoordinate = s;
  result.edgeBCoordinate = t;
  result.closestPointOnA = a + s * edgeA;
  result.closestPointOnB = c + t * edgeB;
  result.squaredDistance
      = (result.closestPointOnA - result.closestPointOnB).squaredNorm();

  const bool aStart = s <= 0.0;
  const bool aEnd = s >= 1.0;
  const bool bStart = t <= 0.0;
  const bool bEnd = t >= 1.0;
  if (aStart && bStart) {
    result.feature = EdgeEdgeFeature::EdgeAStartEdgeBStart;
  } else if (aStart && bEnd) {
    result.feature = EdgeEdgeFeature::EdgeAStartEdgeBEnd;
  } else if (aEnd && bStart) {
    result.feature = EdgeEdgeFeature::EdgeAEndEdgeBStart;
  } else if (aEnd && bEnd) {
    result.feature = EdgeEdgeFeature::EdgeAEndEdgeBEnd;
  } else if (bStart) {
    result.feature = EdgeEdgeFeature::EdgeAInteriorEdgeBStart;
  } else if (bEnd) {
    result.feature = EdgeEdgeFeature::EdgeAInteriorEdgeBEnd;
  } else if (aStart) {
    result.feature = EdgeEdgeFeature::EdgeAStartEdgeBInterior;
  } else if (aEnd) {
    result.feature = EdgeEdgeFeature::EdgeAEndEdgeBInterior;
  } else {
    result.feature = EdgeEdgeFeature::EdgeAInteriorEdgeBInterior;
  }

  return result;
}

//==============================================================================
inline Vector12d edgeEdgeSquaredDistanceGradient(
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c,
    const Eigen::Vector3d& d)
{
  const auto distance = edgeEdgeSquaredDistance(a, b, c, d);
  const Eigen::Vector3d residual
      = distance.closestPointOnA - distance.closestPointOnB;
  const double s = distance.edgeACoordinate;
  const double t = distance.edgeBCoordinate;

  Vector12d gradient = Vector12d::Zero();
  gradient.segment<3>(0) = 2.0 * (1.0 - s) * residual;
  gradient.segment<3>(3) = 2.0 * s * residual;
  gradient.segment<3>(6) = -2.0 * (1.0 - t) * residual;
  gradient.segment<3>(9) = -2.0 * t * residual;
  return gradient;
}

//==============================================================================
inline Matrix12d edgeEdgeSquaredDistanceHessian(
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c,
    const Eigen::Vector3d& d)
{
  const Vector12d x = detail::stack4(a, b, c, d);
  return detail::finiteDifferenceHessian(x, [](const Vector12d& value) {
    return edgeEdgeSquaredDistanceGradient(
        detail::segment(value, 0),
        detail::segment(value, 1),
        detail::segment(value, 2),
        detail::segment(value, 3));
  });
}

//==============================================================================
inline double edgeEdgeCrossSquaredNorm(
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c,
    const Eigen::Vector3d& d)
{
  return (b - a).cross(d - c).squaredNorm();
}

//==============================================================================
inline Vector12d edgeEdgeCrossSquaredNormGradient(
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c,
    const Eigen::Vector3d& d)
{
  const Eigen::Vector3d u = b - a;
  const Eigen::Vector3d v = d - c;
  const Eigen::Vector3d gradientU = 2.0 * (v.squaredNorm() * u - u.dot(v) * v);
  const Eigen::Vector3d gradientV = 2.0 * (u.squaredNorm() * v - u.dot(v) * u);

  Vector12d gradient = Vector12d::Zero();
  gradient.segment<3>(0) = -gradientU;
  gradient.segment<3>(3) = gradientU;
  gradient.segment<3>(6) = -gradientV;
  gradient.segment<3>(9) = gradientV;
  return gradient;
}

//==============================================================================
inline Matrix12d edgeEdgeCrossSquaredNormHessian(
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c,
    const Eigen::Vector3d& d)
{
  return detail::crossSquaredNormHessianFromEdges(b - a, d - c);
}

//==============================================================================
inline double edgeEdgeMollifierThreshold(
    const Eigen::Vector3d& restA,
    const Eigen::Vector3d& restB,
    const Eigen::Vector3d& restC,
    const Eigen::Vector3d& restD)
{
  return detail::kMollifierThresholdScale * (restB - restA).squaredNorm()
         * (restD - restC).squaredNorm();
}

//==============================================================================
inline double edgeEdgeMollifier(
    const double crossSquaredNorm, const double threshold)
{
  if (threshold <= 0.0 || crossSquaredNorm >= threshold) {
    return 1.0;
  }

  const double ratio = crossSquaredNorm / threshold;
  return (-ratio + 2.0) * ratio;
}

//==============================================================================
inline double edgeEdgeMollifierGradient(
    const double crossSquaredNorm, const double threshold)
{
  if (threshold <= 0.0 || crossSquaredNorm >= threshold) {
    return 0.0;
  }

  return 2.0 * (1.0 - crossSquaredNorm / threshold) / threshold;
}

//==============================================================================
inline double edgeEdgeMollifierHessian(
    const double crossSquaredNorm, const double threshold)
{
  if (threshold <= 0.0 || crossSquaredNorm >= threshold) {
    return 0.0;
  }

  return -2.0 / (threshold * threshold);
}

//==============================================================================
inline double edgeEdgeMollifier(
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c,
    const Eigen::Vector3d& d,
    const double threshold)
{
  return edgeEdgeMollifier(edgeEdgeCrossSquaredNorm(a, b, c, d), threshold);
}

//==============================================================================
inline Vector12d edgeEdgeMollifierGradient(
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c,
    const Eigen::Vector3d& d,
    const double threshold)
{
  const double crossSquaredNorm = edgeEdgeCrossSquaredNorm(a, b, c, d);
  return edgeEdgeMollifierGradient(crossSquaredNorm, threshold)
         * edgeEdgeCrossSquaredNormGradient(a, b, c, d);
}

//==============================================================================
inline Matrix12d edgeEdgeMollifierHessian(
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c,
    const Eigen::Vector3d& d,
    const double threshold)
{
  const double crossSquaredNorm = edgeEdgeCrossSquaredNorm(a, b, c, d);
  const Vector12d crossGradient = edgeEdgeCrossSquaredNormGradient(a, b, c, d);

  return edgeEdgeMollifierGradient(crossSquaredNorm, threshold)
             * edgeEdgeCrossSquaredNormHessian(a, b, c, d)
         + edgeEdgeMollifierHessian(crossSquaredNorm, threshold)
               * (crossGradient * crossGradient.transpose());
}

} // namespace dart::simulation::experimental::detail::deformable_contact
