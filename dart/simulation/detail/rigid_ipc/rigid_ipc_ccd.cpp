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

#include <dart/simulation/detail/rigid_ipc/rigid_ipc_ccd.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <limits>
#include <vector>

#include <cmath>

namespace dart::simulation::detail {

namespace {

constexpr double kEpsilon = 1e-12;
constexpr double kConservativeScale = 0.1;
constexpr double kRigidIpcLengthTolerance = 1e-4;
constexpr int kIntervalRootMaxIterations = 10000;

struct ParameterInterval
{
  double lower{0.0};
  double upper{0.0};
};

using ParameterBox = std::array<ParameterInterval, 3>;

[[nodiscard]] double intervalWidth(const ParameterInterval& interval)
{
  return interval.upper - interval.lower;
}

[[nodiscard]] double intervalMidpoint(const ParameterInterval& interval)
{
  return 0.5 * (interval.lower + interval.upper);
}

[[nodiscard]] double parameterValue(
    const ParameterBox& box, const int index, const int cornerMask)
{
  return (cornerMask & (1 << index)) != 0 ? box[index].upper : box[index].lower;
}

[[nodiscard]] bool zeroInResidualRange(
    const Eigen::Vector3d& lower,
    const Eigen::Vector3d& upper,
    const double tolerance)
{
  for (Eigen::Index i = 0; i < lower.size(); ++i) {
    if (lower[i] > tolerance || upper[i] < -tolerance) {
      return false;
    }
  }
  return true;
}

template <typename ResidualFn>
[[nodiscard]] bool residualBoxMayContainZero(
    const ParameterBox& box,
    const int dimension,
    const ResidualFn& residual,
    const double tolerance)
{
  Eigen::Vector3d lower
      = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d upper
      = Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());

  const int cornerCount = 1 << dimension;
  for (int corner = 0; corner < cornerCount; ++corner) {
    const Eigen::Vector3d value = residual(
        parameterValue(box, 0, corner),
        dimension > 1 ? parameterValue(box, 1, corner) : 0.0,
        dimension > 2 ? parameterValue(box, 2, corner) : 0.0);
    lower = lower.cwiseMin(value);
    upper = upper.cwiseMax(value);
  }

  const Eigen::Vector3d midpointValue = residual(
      intervalMidpoint(box[0]),
      dimension > 1 ? intervalMidpoint(box[1]) : 0.0,
      dimension > 2 ? intervalMidpoint(box[2]) : 0.0);
  lower = lower.cwiseMin(midpointValue);
  upper = upper.cwiseMax(midpointValue);

  return zeroInResidualRange(lower, upper, tolerance);
}

[[nodiscard]] bool parameterWidthsWithinTolerance(
    const ParameterBox& box,
    const std::array<double, 3>& tolerances,
    const int dimension)
{
  for (int i = 0; i < dimension; ++i) {
    if (intervalWidth(box[i]) > tolerances[i]) {
      return false;
    }
  }
  return true;
}

[[nodiscard]] int largestRelativeWidthDimension(
    const ParameterBox& box,
    const std::array<double, 3>& tolerances,
    const int dimension)
{
  int splitDimension = 0;
  double bestRatio = -1.0;
  for (int i = 0; i < dimension; ++i) {
    const double tolerance = std::max(tolerances[i], kEpsilon);
    const double ratio = intervalWidth(box[i]) / tolerance;
    if (ratio > bestRatio) {
      bestRatio = ratio;
      splitDimension = i;
    }
  }
  return splitDimension;
}

[[nodiscard]] bool defaultParameterDomainIsValid(const ParameterBox&, const int)
{
  return true;
}

[[nodiscard]] bool faceVertexParameterDomainIsValid(
    const ParameterBox& box, const int)
{
  return box[1].lower + box[2].lower <= 1.0 + kEpsilon;
}

[[nodiscard]] bool faceVertexParameterCenterIsValid(
    const ParameterBox& box, const double tolerance)
{
  return rigidIpcFaceVertexDomainContains(
      intervalMidpoint(box[1]), intervalMidpoint(box[2]), tolerance);
}

[[nodiscard]] double residualToleranceForOption(
    const collision::native::CcdOption& option)
{
  return std::max({option.tolerance, option.minSeparation, kEpsilon});
}

template <typename ResidualFn, typename DomainFn, typename AcceptFn>
bool parameterBoxSubdivisionCcd(
    const int dimension,
    const ParameterBox& initialBox,
    const std::array<double, 3>& tolerances,
    const collision::native::CcdOption& option,
    const bool initiallyTouching,
    const ResidualFn& residual,
    const DomainFn& domainIsValid,
    const AcceptFn& acceptBox,
    collision::native::CcdPrimitiveResult& result)
{
  result.clear();

  const double residualTolerance = residualToleranceForOption(option);
  const int maxIterations
      = std::max(kIntervalRootMaxIterations, option.maxIterations);
  std::vector<ParameterBox> boxes;
  boxes.reserve(256);
  boxes.push_back(initialBox);

  ParameterBox earliestRoot{};
  bool foundRoot = false;
  int iterations = 0;

  while (!boxes.empty() && iterations < maxIterations) {
    ++iterations;
    ParameterBox box = boxes.back();
    boxes.pop_back();

    if (foundRoot && box[0].lower >= earliestRoot[0].lower) {
      continue;
    }
    if (!domainIsValid(box, dimension)) {
      continue;
    }
    if (!residualBoxMayContainZero(
            box, dimension, residual, residualTolerance)) {
      continue;
    }

    if (parameterWidthsWithinTolerance(box, tolerances, dimension)) {
      if (acceptBox(
              box, std::max(residualTolerance, kRigidIpcLengthTolerance))) {
        earliestRoot = box;
        foundRoot = true;
      }
      continue;
    }

    const int splitDimension
        = largestRelativeWidthDimension(box, tolerances, dimension);
    const double midpoint = intervalMidpoint(box[splitDimension]);

    ParameterBox upperHalf = box;
    upperHalf[splitDimension].lower = midpoint;
    ParameterBox lowerHalf = box;
    lowerHalf[splitDimension].upper = midpoint;

    boxes.push_back(upperHalf);
    boxes.push_back(lowerHalf);
  }

  if (!foundRoot) {
    return false;
  }

  result.hit = true;
  result.timeOfImpact = std::clamp(earliestRoot[0].lower, 0.0, 1.0);
  if (result.timeOfImpact == 0.0 && !initiallyTouching) {
    // A terminal root box can start at t=0 while enclosing a positive root.
    // Preserve true initial contacts, but do not surface a zero-time hit for a
    // separated start state.
    result.timeOfImpact = std::nextafter(0.0, 1.0);
  }
  return true;
}

[[nodiscard]] double edgeParameterTolerance(
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const collision::native::CcdOption& option)
{
  const double edgeLength = (b - a).norm();
  return std::max(
      option.tolerance,
      kRigidIpcLengthTolerance / std::max(edgeLength, kEpsilon));
}

[[nodiscard]] Eigen::Vector3d closestPointOnTriangle(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c)
{
  const Eigen::Vector3d ab = b - a;
  const Eigen::Vector3d ac = c - a;
  const Eigen::Vector3d ap = p - a;

  const double d1 = ab.dot(ap);
  const double d2 = ac.dot(ap);
  if (d1 <= 0.0 && d2 <= 0.0) {
    return a;
  }

  const Eigen::Vector3d bp = p - b;
  const double d3 = ab.dot(bp);
  const double d4 = ac.dot(bp);
  if (d3 >= 0.0 && d4 <= d3) {
    return b;
  }

  const double vc = d1 * d4 - d3 * d2;
  if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
    return a + (d1 / (d1 - d3)) * ab;
  }

  const Eigen::Vector3d cp = p - c;
  const double d5 = ab.dot(cp);
  const double d6 = ac.dot(cp);
  if (d6 >= 0.0 && d5 <= d6) {
    return c;
  }

  const double vb = d5 * d2 - d1 * d6;
  if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
    return a + (d2 / (d2 - d6)) * ac;
  }

  const double va = d3 * d6 - d5 * d4;
  if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
    return b + ((d4 - d3) / ((d4 - d3) + (d5 - d6))) * (c - b);
  }

  const double denom = 1.0 / (va + vb + vc);
  const double v = vb * denom;
  const double w = vc * denom;
  return a + ab * v + ac * w;
}

[[nodiscard]] double distancePointTriangle(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c)
{
  return (p - closestPointOnTriangle(p, a, b, c)).norm();
}

[[nodiscard]] double distancePointSegment(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b)
{
  const Eigen::Vector3d ab = b - a;
  const double abLengthSquared = ab.squaredNorm();
  if (abLengthSquared <= kEpsilon) {
    return (p - a).norm();
  }

  const double t = std::clamp((p - a).dot(ab) / abLengthSquared, 0.0, 1.0);
  return (p - (a + t * ab)).norm();
}

[[nodiscard]] double distanceSegmentSegment(
    const Eigen::Vector3d& p1,
    const Eigen::Vector3d& q1,
    const Eigen::Vector3d& p2,
    const Eigen::Vector3d& q2)
{
  const Eigen::Vector3d d1 = q1 - p1;
  const Eigen::Vector3d d2 = q2 - p2;
  const Eigen::Vector3d r = p1 - p2;
  const double a = d1.dot(d1);
  const double e = d2.dot(d2);
  const double f = d2.dot(r);

  double s = 0.0;
  double t = 0.0;

  if (a <= kEpsilon && e <= kEpsilon) {
    s = 0.0;
    t = 0.0;
  } else if (a <= kEpsilon) {
    s = 0.0;
    t = std::clamp(f / e, 0.0, 1.0);
  } else {
    const double c = d1.dot(r);
    if (e <= kEpsilon) {
      t = 0.0;
      s = std::clamp(-c / a, 0.0, 1.0);
    } else {
      const double b = d1.dot(d2);
      const double denom = a * e - b * b;
      if (denom > kEpsilon) {
        s = std::clamp((b * f - c * e) / denom, 0.0, 1.0);
      } else {
        s = 0.0;
      }

      t = (b * s + f) / e;
      if (t < 0.0) {
        t = 0.0;
        s = std::clamp(-c / a, 0.0, 1.0);
      } else if (t > 1.0) {
        t = 1.0;
        s = std::clamp((b - c) / a, 0.0, 1.0);
      }
    }
  }

  return ((p1 + d1 * s) - (p2 + d2 * t)).norm();
}

[[nodiscard]] double maxPointTrajectorySpeedBound(
    const std::array<Eigen::Vector3d, 3>& localPoints,
    const RigidIpcPose& poseStart,
    const RigidIpcPose& poseEnd)
{
  double speed = 0.0;
  for (const Eigen::Vector3d& localPoint : localPoints) {
    speed = std::max(
        speed,
        rigidIpcPointTrajectorySpeedBound(localPoint, poseStart, poseEnd));
  }
  return speed;
}

template <typename DistFn>
bool curvedAccdAdvance(
    const double speedBound,
    const collision::native::CcdOption& option,
    DistFn&& distAt,
    collision::native::CcdPrimitiveResult& result)
{
  result.clear();

  const double minSeparation = std::max(0.0, option.minSeparation);
  double distance = distAt(0.0);

  if (distance - minSeparation <= 0.0) {
    result.hit = true;
    result.status = collision::native::CcdPrimitiveStatus::Hit;
    result.timeOfImpact = 0.0;
    return true;
  }

  if (speedBound <= kEpsilon) {
    result.status = collision::native::CcdPrimitiveStatus::Miss;
    return false;
  }

  const double convergeAbs = std::max(option.tolerance, kEpsilon);
  const int maxIter = std::max(1, option.maxIterations);
  const double stepScale
      = option.advancement == collision::native::CcdAdvancement::Conservative
            ? 1.0 - kConservativeScale
            : 1.0;

  double time = 0.0;
  for (int iter = 0; iter < maxIter; ++iter) {
    const double clearance = distance - minSeparation;
    if (clearance <= convergeAbs) {
      result.hit = true;
      result.status = collision::native::CcdPrimitiveStatus::Hit;
      result.timeOfImpact = time;
      return true;
    }

    const double safeAdvance = clearance / speedBound;
    if (time + safeAdvance >= 1.0) {
      if (distAt(1.0) - minSeparation <= convergeAbs) {
        result.hit = true;
        result.status = collision::native::CcdPrimitiveStatus::Hit;
        result.timeOfImpact = 1.0;
        return true;
      }
      result.status = collision::native::CcdPrimitiveStatus::Miss;
      return false;
    }

    time += stepScale * safeAdvance;
    distance = distAt(time);
  }

  // The budget is exhausted without proving a hit or a clean miss, but every
  // conservative advance above was a provably contact-free sub-step, so `time`
  // is a valid lower bound on the true time of impact: the primitives are
  // guaranteed separated over [0, time]. Report it so callers can take a
  // bounded (positive) safe step instead of a fully blocked zero step -- the
  // result is still flagged Indeterminate, so callers that require a definitive
  // hit/miss are unaffected.
  result.timeOfImpact = std::clamp(time, 0.0, 1.0);
  result.status = collision::native::CcdPrimitiveStatus::Indeterminate;
  return false;
}

} // namespace

Eigen::Matrix3d rigidIpcRotationVectorToMatrix(const Eigen::Vector3d& rotation)
{
  const double angle = rotation.norm();
  if (angle == 0.0) {
    return Eigen::Matrix3d::Identity();
  }

  return Eigen::AngleAxisd(angle, rotation / angle).toRotationMatrix();
}

double rigidIpcPointTrajectorySpeedBound(
    const Eigen::Vector3d& localPoint,
    const RigidIpcPose& start,
    const RigidIpcPose& end)
{
  const double translationSpeed = (end.position - start.position).norm();
  const double angularSpeed = (end.rotation - start.rotation).norm();
  return translationSpeed + angularSpeed * localPoint.norm();
}

RigidIpcPose interpolateRigidIpcPose(
    const RigidIpcPose& start, const RigidIpcPose& end, const double time)
{
  return {
      start.position + time * (end.position - start.position),
      start.rotation + time * (end.rotation - start.rotation)};
}

Eigen::Vector3d transformRigidIpcPoint(
    const Eigen::Vector3d& localPoint, const RigidIpcPose& pose)
{
  return rigidIpcRotationVectorToMatrix(pose.rotation) * localPoint
         + pose.position;
}

Eigen::Vector3d transformRigidIpcPoint(
    const Eigen::Vector3d& localPoint,
    const RigidIpcPose& start,
    const RigidIpcPose& end,
    const double time)
{
  return transformRigidIpcPoint(
      localPoint, interpolateRigidIpcPose(start, end, time));
}

Eigen::Vector3d rigidIpcPointEdgeResidual(
    const Eigen::Vector3d& point,
    const RigidIpcPose& pointPoseStart,
    const RigidIpcPose& pointPoseEnd,
    const Eigen::Vector3d& edgeA,
    const Eigen::Vector3d& edgeB,
    const RigidIpcPose& edgePoseStart,
    const RigidIpcPose& edgePoseEnd,
    const double time,
    const double alpha)
{
  const Eigen::Vector3d pointWorld
      = transformRigidIpcPoint(point, pointPoseStart, pointPoseEnd, time);
  const Eigen::Vector3d edgeAWorld
      = transformRigidIpcPoint(edgeA, edgePoseStart, edgePoseEnd, time);
  const Eigen::Vector3d edgeBWorld
      = transformRigidIpcPoint(edgeB, edgePoseStart, edgePoseEnd, time);
  return pointWorld - (edgeAWorld + alpha * (edgeBWorld - edgeAWorld));
}

Eigen::Vector3d rigidIpcEdgeEdgeResidual(
    const Eigen::Vector3d& edgeA0,
    const Eigen::Vector3d& edgeA1,
    const RigidIpcPose& edgeAPoseStart,
    const RigidIpcPose& edgeAPoseEnd,
    const Eigen::Vector3d& edgeB0,
    const Eigen::Vector3d& edgeB1,
    const RigidIpcPose& edgeBPoseStart,
    const RigidIpcPose& edgeBPoseEnd,
    const double time,
    const double alpha,
    const double beta)
{
  const Eigen::Vector3d edgeA0World
      = transformRigidIpcPoint(edgeA0, edgeAPoseStart, edgeAPoseEnd, time);
  const Eigen::Vector3d edgeA1World
      = transformRigidIpcPoint(edgeA1, edgeAPoseStart, edgeAPoseEnd, time);
  const Eigen::Vector3d edgeB0World
      = transformRigidIpcPoint(edgeB0, edgeBPoseStart, edgeBPoseEnd, time);
  const Eigen::Vector3d edgeB1World
      = transformRigidIpcPoint(edgeB1, edgeBPoseStart, edgeBPoseEnd, time);
  const Eigen::Vector3d pointA
      = edgeA0World + alpha * (edgeA1World - edgeA0World);
  const Eigen::Vector3d pointB
      = edgeB0World + beta * (edgeB1World - edgeB0World);
  return pointA - pointB;
}

Eigen::Vector3d rigidIpcPointTriangleResidual(
    const Eigen::Vector3d& point,
    const RigidIpcPose& pointPoseStart,
    const RigidIpcPose& pointPoseEnd,
    const Eigen::Vector3d& triangleA,
    const Eigen::Vector3d& triangleB,
    const Eigen::Vector3d& triangleC,
    const RigidIpcPose& trianglePoseStart,
    const RigidIpcPose& trianglePoseEnd,
    const double time,
    const double u,
    const double v)
{
  const Eigen::Vector3d pointWorld
      = transformRigidIpcPoint(point, pointPoseStart, pointPoseEnd, time);
  const Eigen::Vector3d triangleAWorld = transformRigidIpcPoint(
      triangleA, trianglePoseStart, trianglePoseEnd, time);
  const Eigen::Vector3d triangleBWorld = transformRigidIpcPoint(
      triangleB, trianglePoseStart, trianglePoseEnd, time);
  const Eigen::Vector3d triangleCWorld = transformRigidIpcPoint(
      triangleC, trianglePoseStart, trianglePoseEnd, time);
  return pointWorld
         - (triangleAWorld + u * (triangleBWorld - triangleAWorld)
            + v * (triangleCWorld - triangleAWorld));
}

bool rigidIpcFaceVertexDomainContains(
    const double u, const double v, const double tolerance)
{
  const double domainTolerance = std::max(0.0, tolerance);
  return u >= -domainTolerance && v >= -domainTolerance
         && u + v <= 1.0 + domainTolerance;
}

bool rigidIpcPointEdgeIntervalCcd(
    const Eigen::Vector3d& point,
    const RigidIpcPose& pointPoseStart,
    const RigidIpcPose& pointPoseEnd,
    const Eigen::Vector3d& edgeA,
    const Eigen::Vector3d& edgeB,
    const RigidIpcPose& edgePoseStart,
    const RigidIpcPose& edgePoseEnd,
    const collision::native::CcdOption& option,
    collision::native::CcdPrimitiveResult& result)
{
  const ParameterBox initialBox{
      ParameterInterval{0.0, 1.0},
      ParameterInterval{0.0, 1.0},
      ParameterInterval{}};
  const std::array<double, 3> tolerances{
      std::max(option.tolerance, kEpsilon),
      edgeParameterTolerance(edgeA, edgeB, option),
      1.0};
  const auto residual
      = [&](const double time, const double alpha, const double) {
          return rigidIpcPointEdgeResidual(
              point,
              pointPoseStart,
              pointPoseEnd,
              edgeA,
              edgeB,
              edgePoseStart,
              edgePoseEnd,
              time,
              alpha);
        };
  const auto acceptBox = [&](const ParameterBox& box, const double tolerance) {
    return residual(intervalMidpoint(box[0]), intervalMidpoint(box[1]), 0.0)
               .norm()
           <= tolerance;
  };
  const bool initiallyTouching
      = distancePointSegment(
            transformRigidIpcPoint(point, pointPoseStart),
            transformRigidIpcPoint(edgeA, edgePoseStart),
            transformRigidIpcPoint(edgeB, edgePoseStart))
        <= residualToleranceForOption(option);
  return parameterBoxSubdivisionCcd(
      2,
      initialBox,
      tolerances,
      option,
      initiallyTouching,
      residual,
      defaultParameterDomainIsValid,
      acceptBox,
      result);
}

bool rigidIpcEdgeEdgeIntervalCcd(
    const Eigen::Vector3d& edgeA0,
    const Eigen::Vector3d& edgeA1,
    const RigidIpcPose& edgeAPoseStart,
    const RigidIpcPose& edgeAPoseEnd,
    const Eigen::Vector3d& edgeB0,
    const Eigen::Vector3d& edgeB1,
    const RigidIpcPose& edgeBPoseStart,
    const RigidIpcPose& edgeBPoseEnd,
    const collision::native::CcdOption& option,
    collision::native::CcdPrimitiveResult& result)
{
  const ParameterBox initialBox{
      ParameterInterval{0.0, 1.0},
      ParameterInterval{0.0, 1.0},
      ParameterInterval{0.0, 1.0}};
  const std::array<double, 3> tolerances{
      std::max(option.tolerance, kEpsilon),
      edgeParameterTolerance(edgeA0, edgeA1, option),
      edgeParameterTolerance(edgeB0, edgeB1, option)};
  const auto residual
      = [&](const double time, const double alpha, const double beta) {
          return rigidIpcEdgeEdgeResidual(
              edgeA0,
              edgeA1,
              edgeAPoseStart,
              edgeAPoseEnd,
              edgeB0,
              edgeB1,
              edgeBPoseStart,
              edgeBPoseEnd,
              time,
              alpha,
              beta);
        };
  const auto acceptBox = [&](const ParameterBox& box, const double tolerance) {
    return residual(
               intervalMidpoint(box[0]),
               intervalMidpoint(box[1]),
               intervalMidpoint(box[2]))
               .norm()
           <= tolerance;
  };
  const bool initiallyTouching
      = distanceSegmentSegment(
            transformRigidIpcPoint(edgeA0, edgeAPoseStart),
            transformRigidIpcPoint(edgeA1, edgeAPoseStart),
            transformRigidIpcPoint(edgeB0, edgeBPoseStart),
            transformRigidIpcPoint(edgeB1, edgeBPoseStart))
        <= residualToleranceForOption(option);
  return parameterBoxSubdivisionCcd(
      3,
      initialBox,
      tolerances,
      option,
      initiallyTouching,
      residual,
      defaultParameterDomainIsValid,
      acceptBox,
      result);
}

bool rigidIpcPointTriangleIntervalCcd(
    const Eigen::Vector3d& point,
    const RigidIpcPose& pointPoseStart,
    const RigidIpcPose& pointPoseEnd,
    const Eigen::Vector3d& triangleA,
    const Eigen::Vector3d& triangleB,
    const Eigen::Vector3d& triangleC,
    const RigidIpcPose& trianglePoseStart,
    const RigidIpcPose& trianglePoseEnd,
    const collision::native::CcdOption& option,
    collision::native::CcdPrimitiveResult& result)
{
  const ParameterBox initialBox{
      ParameterInterval{0.0, 1.0},
      ParameterInterval{0.0, 1.0},
      ParameterInterval{0.0, 1.0}};
  const std::array<double, 3> tolerances{
      std::max(option.tolerance, kEpsilon),
      edgeParameterTolerance(triangleA, triangleB, option),
      edgeParameterTolerance(triangleA, triangleC, option)};
  const auto residual = [&](const double time, const double u, const double v) {
    return rigidIpcPointTriangleResidual(
        point,
        pointPoseStart,
        pointPoseEnd,
        triangleA,
        triangleB,
        triangleC,
        trianglePoseStart,
        trianglePoseEnd,
        time,
        u,
        v);
  };
  const auto acceptBox = [&](const ParameterBox& box, const double tolerance) {
    return faceVertexParameterCenterIsValid(box, tolerance)
           && residual(
                  intervalMidpoint(box[0]),
                  intervalMidpoint(box[1]),
                  intervalMidpoint(box[2]))
                      .norm()
                  <= tolerance;
  };
  const bool initiallyTouching
      = distancePointTriangle(
            transformRigidIpcPoint(point, pointPoseStart),
            transformRigidIpcPoint(triangleA, trianglePoseStart),
            transformRigidIpcPoint(triangleB, trianglePoseStart),
            transformRigidIpcPoint(triangleC, trianglePoseStart))
        <= residualToleranceForOption(option);
  return parameterBoxSubdivisionCcd(
      3,
      initialBox,
      tolerances,
      option,
      initiallyTouching,
      residual,
      faceVertexParameterDomainIsValid,
      acceptBox,
      result);
}

bool rigidIpcPointTriangleCcd(
    const Eigen::Vector3d& point,
    const RigidIpcPose& pointPoseStart,
    const RigidIpcPose& pointPoseEnd,
    const Eigen::Vector3d& triangleA,
    const Eigen::Vector3d& triangleB,
    const Eigen::Vector3d& triangleC,
    const RigidIpcPose& trianglePoseStart,
    const RigidIpcPose& trianglePoseEnd,
    const collision::native::CcdOption& option,
    collision::native::CcdPrimitiveResult& result)
{
  const double speedBound
      = rigidIpcPointTrajectorySpeedBound(point, pointPoseStart, pointPoseEnd)
        + maxPointTrajectorySpeedBound(
            {triangleA, triangleB, triangleC},
            trianglePoseStart,
            trianglePoseEnd);

  const auto distanceAt = [&](const double time) {
    return distancePointTriangle(
        transformRigidIpcPoint(point, pointPoseStart, pointPoseEnd, time),
        transformRigidIpcPoint(
            triangleA, trianglePoseStart, trianglePoseEnd, time),
        transformRigidIpcPoint(
            triangleB, trianglePoseStart, trianglePoseEnd, time),
        transformRigidIpcPoint(
            triangleC, trianglePoseStart, trianglePoseEnd, time));
  };

  return curvedAccdAdvance(speedBound, option, distanceAt, result);
}

bool rigidIpcPointPointCcd(
    const Eigen::Vector3d& pointA,
    const RigidIpcPose& pointAPoseStart,
    const RigidIpcPose& pointAPoseEnd,
    const Eigen::Vector3d& pointB,
    const RigidIpcPose& pointBPoseStart,
    const RigidIpcPose& pointBPoseEnd,
    const collision::native::CcdOption& option,
    collision::native::CcdPrimitiveResult& result)
{
  const double speedBound = rigidIpcPointTrajectorySpeedBound(
                                pointA, pointAPoseStart, pointAPoseEnd)
                            + rigidIpcPointTrajectorySpeedBound(
                                pointB, pointBPoseStart, pointBPoseEnd);

  const auto distanceAt = [&](const double time) {
    return (transformRigidIpcPoint(pointA, pointAPoseStart, pointAPoseEnd, time)
            - transformRigidIpcPoint(
                pointB, pointBPoseStart, pointBPoseEnd, time))
        .norm();
  };

  return curvedAccdAdvance(speedBound, option, distanceAt, result);
}

bool rigidIpcPointEdgeCcd(
    const Eigen::Vector3d& point,
    const RigidIpcPose& pointPoseStart,
    const RigidIpcPose& pointPoseEnd,
    const Eigen::Vector3d& edgeA,
    const Eigen::Vector3d& edgeB,
    const RigidIpcPose& edgePoseStart,
    const RigidIpcPose& edgePoseEnd,
    const collision::native::CcdOption& option,
    collision::native::CcdPrimitiveResult& result)
{
  const double speedBound
      = rigidIpcPointTrajectorySpeedBound(point, pointPoseStart, pointPoseEnd)
        + maxPointTrajectorySpeedBound(
            {edgeA, edgeB, Eigen::Vector3d::Zero()},
            edgePoseStart,
            edgePoseEnd);

  const auto distanceAt = [&](const double time) {
    return distancePointSegment(
        transformRigidIpcPoint(point, pointPoseStart, pointPoseEnd, time),
        transformRigidIpcPoint(edgeA, edgePoseStart, edgePoseEnd, time),
        transformRigidIpcPoint(edgeB, edgePoseStart, edgePoseEnd, time));
  };

  return curvedAccdAdvance(speedBound, option, distanceAt, result);
}

bool rigidIpcEdgeEdgeCcd(
    const Eigen::Vector3d& edgeA0,
    const Eigen::Vector3d& edgeA1,
    const RigidIpcPose& edgeAPoseStart,
    const RigidIpcPose& edgeAPoseEnd,
    const Eigen::Vector3d& edgeB0,
    const Eigen::Vector3d& edgeB1,
    const RigidIpcPose& edgeBPoseStart,
    const RigidIpcPose& edgeBPoseEnd,
    const collision::native::CcdOption& option,
    collision::native::CcdPrimitiveResult& result)
{
  const double speedBound = maxPointTrajectorySpeedBound(
                                {edgeA0, edgeA1, Eigen::Vector3d::Zero()},
                                edgeAPoseStart,
                                edgeAPoseEnd)
                            + maxPointTrajectorySpeedBound(
                                {edgeB0, edgeB1, Eigen::Vector3d::Zero()},
                                edgeBPoseStart,
                                edgeBPoseEnd);

  const auto distanceAt = [&](const double time) {
    return distanceSegmentSegment(
        transformRigidIpcPoint(edgeA0, edgeAPoseStart, edgeAPoseEnd, time),
        transformRigidIpcPoint(edgeA1, edgeAPoseStart, edgeAPoseEnd, time),
        transformRigidIpcPoint(edgeB0, edgeBPoseStart, edgeBPoseEnd, time),
        transformRigidIpcPoint(edgeB1, edgeBPoseStart, edgeBPoseEnd, time));
  };

  return curvedAccdAdvance(speedBound, option, distanceAt, result);
}

} // namespace dart::simulation::detail
