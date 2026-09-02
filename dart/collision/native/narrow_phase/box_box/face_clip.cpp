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

#include <dart/collision/native/narrow_phase/box_box/face_clip.hpp>

#include <algorithm>
#include <array>
#include <stdexcept>

#include <cmath>

namespace dart::collision::native::box_box {

namespace {

constexpr double kClipTolerance = 1e-10;
constexpr double kSurfaceAxisEpsilon = 1e-8;

struct FaceBasis
{
  int axis = 0;
  double sign = 1.0;
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Vector3d tangent1 = Eigen::Vector3d::UnitX();
  Eigen::Vector3d tangent2 = Eigen::Vector3d::UnitY();
  double halfExtent1 = 0.0;
  double halfExtent2 = 0.0;
};

struct FixedPolygon
{
  static constexpr std::size_t kCapacity = FixedContactCandidates::kCapacity;

  void pushBack(const Eigen::Vector3d& point)
  {
    if (count >= values.size()) {
      throw std::overflow_error(
          "Box-box face clipping exceeded its geometric vertex bound");
    }
    values[count++] = point;
  }

  [[nodiscard]] bool empty() const noexcept
  {
    return count == 0u;
  }

  [[nodiscard]] const Eigen::Vector3d& back() const noexcept
  {
    return values[count - 1u];
  }

  [[nodiscard]] const Eigen::Vector3d* begin() const noexcept
  {
    return values.data();
  }

  [[nodiscard]] const Eigen::Vector3d* end() const noexcept
  {
    return values.data() + count;
  }

  std::array<Eigen::Vector3d, kCapacity> values{};
  std::size_t count = 0u;
};

[[nodiscard]] FaceBasis makeFaceBasis(
    const BoxData& box, int axis, const Eigen::Vector3d& outwardNormal)
{
  FaceBasis face;
  face.axis = axis;
  face.sign = (outwardNormal.dot(box.rotation.col(axis)) >= 0.0) ? 1.0 : -1.0;
  face.normal = face.sign * box.rotation.col(axis);
  face.center = box.center + face.normal * box.halfExtents[axis];

  const int tangentAxis1 = (axis + 1) % 3;
  const int tangentAxis2 = (axis + 2) % 3;
  face.tangent1 = box.rotation.col(tangentAxis1);
  face.tangent2 = box.rotation.col(tangentAxis2);
  face.halfExtent1 = box.halfExtents[tangentAxis1];
  face.halfExtent2 = box.halfExtents[tangentAxis2];
  return face;
}

[[nodiscard]] FaceBasis makeBestFaceBasis(
    const BoxData& box, const Eigen::Vector3d& outwardNormal)
{
  int axis = 0;
  double alignment = -1.0;
  for (int i = 0; i < 3; ++i) {
    const double candidate = std::abs(outwardNormal.dot(box.rotation.col(i)));
    if (candidate > alignment) {
      alignment = candidate;
      axis = i;
    }
  }

  return makeFaceBasis(box, axis, outwardNormal);
}

[[nodiscard]] FixedPolygon makeFaceVertices(const FaceBasis& face)
{
  FixedPolygon polygon;
  polygon.pushBack(
      face.center - face.halfExtent1 * face.tangent1
      - face.halfExtent2 * face.tangent2);
  polygon.pushBack(
      face.center + face.halfExtent1 * face.tangent1
      - face.halfExtent2 * face.tangent2);
  polygon.pushBack(
      face.center + face.halfExtent1 * face.tangent1
      + face.halfExtent2 * face.tangent2);
  polygon.pushBack(
      face.center - face.halfExtent1 * face.tangent1
      + face.halfExtent2 * face.tangent2);
  return polygon;
}

template <typename SignedDistance>
void clipPolygon(FixedPolygon& polygon, SignedDistance&& signedDistance)
{
  if (polygon.empty()) {
    return;
  }

  FixedPolygon clipped;

  Eigen::Vector3d previous = polygon.back();
  double previousDistance = signedDistance(previous);
  bool previousInside = previousDistance >= -kClipTolerance;

  for (const auto& current : polygon) {
    const double currentDistance = signedDistance(current);
    const bool currentInside = currentDistance >= -kClipTolerance;

    if (currentInside != previousInside) {
      const double denominator = previousDistance - currentDistance;
      if (std::abs(denominator) > kClipTolerance) {
        const double t = previousDistance / denominator;
        clipped.pushBack(previous + t * (current - previous));
      }
    }

    if (currentInside) {
      clipped.pushBack(current);
    }

    previous = current;
    previousDistance = currentDistance;
    previousInside = currentInside;
  }

  polygon = clipped;
}

void clipToReferenceFace(FixedPolygon& polygon, const FaceBasis& reference)
{
  clipPolygon(polygon, [&](const Eigen::Vector3d& point) {
    const double coordinate
        = (point - reference.center).dot(reference.tangent1);
    return reference.halfExtent1 - coordinate;
  });
  clipPolygon(polygon, [&](const Eigen::Vector3d& point) {
    const double coordinate
        = (point - reference.center).dot(reference.tangent1);
    return reference.halfExtent1 + coordinate;
  });
  clipPolygon(polygon, [&](const Eigen::Vector3d& point) {
    const double coordinate
        = (point - reference.center).dot(reference.tangent2);
    return reference.halfExtent2 - coordinate;
  });
  clipPolygon(polygon, [&](const Eigen::Vector3d& point) {
    const double coordinate
        = (point - reference.center).dot(reference.tangent2);
    return reference.halfExtent2 + coordinate;
  });
  clipPolygon(polygon, [&](const Eigen::Vector3d& point) {
    const double signedDistance
        = reference.normal.dot(point - reference.center);
    return -signedDistance;
  });
}

[[nodiscard]] Eigen::Vector3d surfacePointNear(
    const BoxData& box,
    const Eigen::Vector3d& direction,
    const Eigen::Vector3d& nearPoint)
{
  Eigen::Vector3d local = box.rotation.transpose() * (nearPoint - box.center);
  const Eigen::Vector3d localDirection = box.rotation.transpose() * direction;

  for (int i = 0; i < 3; ++i) {
    if (std::abs(localDirection[i]) > kSurfaceAxisEpsilon) {
      local[i] = (localDirection[i] > 0.0) ? box.halfExtents[i]
                                           : -box.halfExtents[i];
    } else {
      local[i] = std::clamp(local[i], -box.halfExtents[i], box.halfExtents[i]);
    }
  }

  return box.center + box.rotation * local;
}

[[nodiscard]] ContactCandidate computeSupportContactCandidate(
    const BoxData& box1, const BoxData& box2, const SatResult& sat)
{
  Eigen::Vector3d forwardPoint1
      = surfacePointNear(box1, -sat.normal, box2.center);
  Eigen::Vector3d forwardPoint2
      = surfacePointNear(box2, sat.normal, forwardPoint1);
  forwardPoint1 = surfacePointNear(box1, -sat.normal, forwardPoint2);

  Eigen::Vector3d reversePoint2
      = surfacePointNear(box2, sat.normal, box1.center);
  Eigen::Vector3d reversePoint1
      = surfacePointNear(box1, -sat.normal, reversePoint2);
  reversePoint2 = surfacePointNear(box2, sat.normal, reversePoint1);

  return {
      0.25 * (forwardPoint1 + forwardPoint2 + reversePoint1 + reversePoint2),
      sat.penetration};
}

[[nodiscard]] FixedContactCandidates computeFaceContactCandidates(
    const BoxData& box1, const BoxData& box2, const SatResult& sat)
{
  if (sat.referenceBox < 0 || sat.referenceAxis < 0) {
    return {};
  }

  const bool referenceIsBox1 = sat.referenceBox == 0;
  const BoxData& reference = referenceIsBox1 ? box1 : box2;
  const BoxData& incident = referenceIsBox1 ? box2 : box1;
  const Eigen::Vector3d referenceOutwardNormal
      = referenceIsBox1 ? -sat.normal : sat.normal;

  const FaceBasis referenceFace
      = makeFaceBasis(reference, sat.referenceAxis, referenceOutwardNormal);
  const FaceBasis incidentFace
      = makeBestFaceBasis(incident, -referenceFace.normal);

  FixedPolygon polygon = makeFaceVertices(incidentFace);
  clipToReferenceFace(polygon, referenceFace);

  FixedContactCandidates candidates;
  for (const auto& point : polygon) {
    const double signedDistance
        = referenceFace.normal.dot(point - referenceFace.center);
    if (-signedDistance + kClipTolerance < 0.0) {
      continue;
    }

    if (candidates.count >= candidates.values.size()) {
      throw std::overflow_error(
          "Box-box contact candidates exceeded their geometric bound");
    }
    candidates.values[candidates.count++]
        = {point - 0.5 * signedDistance * referenceFace.normal,
           std::max(0.0, sat.penetration)};
  }

  return candidates;
}

} // namespace

FixedContactCandidates computeBoxBoxContactCandidatesFixed(
    const BoxData& box1, const BoxData& box2, const SatResult& sat)
{
  if (sat.axisType == SatAxisType::Face) {
    return computeFaceContactCandidates(box1, box2, sat);
  }

  FixedContactCandidates candidates;
  candidates.values[0] = computeSupportContactCandidate(box1, box2, sat);
  candidates.count = 1u;
  return candidates;
}

std::vector<ContactCandidate> computeBoxBoxContactCandidates(
    const BoxData& box1, const BoxData& box2, const SatResult& sat)
{
  const auto fixed = computeBoxBoxContactCandidatesFixed(box1, box2, sat);
  return {fixed.begin(), fixed.end()};
}

} // namespace dart::collision::native::box_box
