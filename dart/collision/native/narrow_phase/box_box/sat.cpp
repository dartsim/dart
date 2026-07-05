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

#include <dart/collision/native/narrow_phase/box_box/sat.hpp>

#include <algorithm>
#include <array>

#include <cmath>

namespace dart::collision::native::box_box {

namespace {

constexpr double kAxisLengthEpsilon = 1e-10;
constexpr double kCrossAxisDegenerateLengthSq = 1e-12;
constexpr double kSeparationEpsilon = 1e-12;
constexpr double kTieAbsoluteTolerance = 1e-6;
constexpr double kTieRelativeTolerance = 2e-2;

[[nodiscard]] bool shouldReplaceBest(
    double overlap, SatAxisType axisType, int axisIndex, const SatResult& best)
{
  if (best.axisIndex < 0) {
    return true;
  }

  const double tolerance = std::max(
      kTieAbsoluteTolerance,
      kTieRelativeTolerance
          * std::min(std::abs(overlap), std::abs(best.penetration)));

  if (overlap < best.penetration - tolerance) {
    return true;
  }

  if (std::abs(overlap - best.penetration) > tolerance) {
    return false;
  }

  if (axisType == SatAxisType::Face && best.axisType == SatAxisType::Edge) {
    return true;
  }

  return axisType == best.axisType && axisIndex < best.axisIndex;
}

[[nodiscard]] bool testAxis(
    const Eigen::Vector3d& axis,
    const BoxData& box1,
    const BoxData& box2,
    const Eigen::Vector3d& centerDiff,
    int axisIndex,
    SatAxisType axisType,
    int referenceBox,
    int referenceAxis,
    SatResult& best)
{
  const double axisLengthSq = axis.squaredNorm();
  if (axisLengthSq < kAxisLengthEpsilon * kAxisLengthEpsilon) {
    return true;
  }

  const Eigen::Vector3d normalizedAxis = axis / std::sqrt(axisLengthSq);
  const double projection1 = projectBox(box1, normalizedAxis);
  const double projection2 = projectBox(box2, normalizedAxis);
  const double distance = std::abs(centerDiff.dot(normalizedAxis));
  double overlap = projection1 + projection2 - distance;

  if (overlap < -kSeparationEpsilon) {
    return false;
  }

  overlap = std::max(0.0, overlap);
  if (shouldReplaceBest(overlap, axisType, axisIndex, best)) {
    best.penetration = overlap;
    best.normal = normalizedAxis;
    best.axisIndex = axisIndex;
    best.axisType = axisType;
    best.referenceBox = referenceBox;
    best.referenceAxis = referenceAxis;
  }

  return true;
}

} // namespace

double projectBox(const BoxData& box, const Eigen::Vector3d& axis)
{
  return box.halfExtents.x() * std::abs(axis.dot(box.rotation.col(0)))
         + box.halfExtents.y() * std::abs(axis.dot(box.rotation.col(1)))
         + box.halfExtents.z() * std::abs(axis.dot(box.rotation.col(2)));
}

bool computeBoxBoxSat(
    const BoxData& box1, const BoxData& box2, SatResult& result)
{
  result = SatResult();

  const Eigen::Vector3d centerDiff = box2.center - box1.center;
  const std::array<Eigen::Vector3d, 3> axes1
      = {box1.rotation.col(0), box1.rotation.col(1), box1.rotation.col(2)};
  const std::array<Eigen::Vector3d, 3> axes2
      = {box2.rotation.col(0), box2.rotation.col(1), box2.rotation.col(2)};

  int axisIndex = 0;
  for (int i = 0; i < 3; ++i) {
    if (!testAxis(
            axes1[i],
            box1,
            box2,
            centerDiff,
            axisIndex++,
            SatAxisType::Face,
            0,
            i,
            result)) {
      return false;
    }
  }

  for (int i = 0; i < 3; ++i) {
    if (!testAxis(
            axes2[i],
            box1,
            box2,
            centerDiff,
            axisIndex++,
            SatAxisType::Face,
            1,
            i,
            result)) {
      return false;
    }
  }

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      const Eigen::Vector3d crossAxis = axes1[i].cross(axes2[j]);
      if (crossAxis.squaredNorm() < kCrossAxisDegenerateLengthSq) {
        ++axisIndex;
        continue;
      }

      if (!testAxis(
              crossAxis.normalized(),
              box1,
              box2,
              centerDiff,
              axisIndex++,
              SatAxisType::Edge,
              -1,
              -1,
              result)) {
        return false;
      }
    }
  }

  if (result.axisIndex < 0) {
    return false;
  }

  if (centerDiff.dot(result.normal) > 0.0) {
    result.normal = -result.normal;
  }

  return true;
}

} // namespace dart::collision::native::box_box
