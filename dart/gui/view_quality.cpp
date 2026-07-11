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

#include <dart/gui/interaction.hpp>
#include <dart/gui/view_quality.hpp>
#include <dart/gui/viewer.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <optional>
#include <unordered_set>
#include <vector>

#include <cmath>
#include <cstddef>

namespace dart::gui {
namespace {

// Subject screen-area fractions outside this band flag too-far / too-close.
constexpr double kMinSubjectFraction = 0.015;
constexpr double kMaxSubjectFraction = 0.75;
// A subject with corners past the viewport while its center is visible is
// cropped; below this on-screen corner fraction the view is discarded.
constexpr double kMinCornerCoverage = 0.999;
// Fraction of focus-sample rays that may hit another body before the view
// counts as occluded.
constexpr double kMaxOcclusionFraction = 0.35;
// Pairwise screen-box IoU above this for depth-separated bodies flags an
// ambiguous (axis-aligned, stacked) viewpoint.
constexpr double kMaxAmbiguityIoU = 0.55;

/// The eight world-space corners of a descriptor's local AABB, or an empty
/// vector when it exposes no local bounds.
std::vector<Eigen::Vector3d> descriptorWorldCorners(
    const RenderableDescriptor& descriptor)
{
  std::vector<Eigen::Vector3d> corners;
  if (!descriptor.geometry.hasLocalBounds) {
    return corners;
  }
  const Eigen::Vector3d& low = descriptor.geometry.localBoundsMin;
  const Eigen::Vector3d& high = descriptor.geometry.localBoundsMax;
  corners.reserve(8u);
  for (const double x : {low.x(), high.x()}) {
    for (const double y : {low.y(), high.y()}) {
      for (const double z : {low.z(), high.z()}) {
        corners.push_back(descriptor.worldTransform * Eigen::Vector3d(x, y, z));
      }
    }
  }
  return corners;
}

struct ScreenBox
{
  double minX = 0.0;
  double minY = 0.0;
  double maxX = 0.0;
  double maxY = 0.0;
  bool valid = false;
};

/// Axis-aligned screen bounds over the in-front (depth > 0) projected points.
/// Points behind the camera project to unbounded coordinates and are ignored,
/// mirroring the Python reference.
ScreenBox screenBox(const std::vector<Eigen::Vector3d>& projected)
{
  ScreenBox box;
  for (const Eigen::Vector3d& point : projected) {
    if (point.z() <= 0.0) {
      continue;
    }
    if (!box.valid) {
      box.minX = box.maxX = point.x();
      box.minY = box.maxY = point.y();
      box.valid = true;
    } else {
      box.minX = std::min(box.minX, point.x());
      box.minY = std::min(box.minY, point.y());
      box.maxX = std::max(box.maxX, point.x());
      box.maxY = std::max(box.maxY, point.y());
    }
  }
  return box;
}

double boxIoU(const ScreenBox& a, const ScreenBox& b)
{
  const double intersectionWidth
      = std::min(a.maxX, b.maxX) - std::max(a.minX, b.minX);
  const double intersectionHeight
      = std::min(a.maxY, b.maxY) - std::max(a.minY, b.minY);
  if (intersectionWidth <= 0.0 || intersectionHeight <= 0.0) {
    return 0.0;
  }
  const double intersection = intersectionWidth * intersectionHeight;
  const double areaA = (a.maxX - a.minX) * (a.maxY - a.minY);
  const double areaB = (b.maxX - b.minX) * (b.maxY - b.minY);
  const double unionArea = areaA + areaB - intersection;
  return unionArea > 0.0 ? intersection / unionArea : 0.0;
}

double median(std::vector<double> values)
{
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const std::size_t count = values.size();
  if (count % 2u == 1u) {
    return values[count / 2u];
  }
  return 0.5 * (values[count / 2u - 1u] + values[count / 2u]);
}

double occlusionFraction(
    const std::vector<RenderableDescriptor>& descriptors,
    const std::unordered_set<RenderableId>& focusIds,
    bool assessAll,
    const std::vector<Eigen::Vector3d>& samples,
    const Eigen::Vector3d& eye)
{
  int blocked = 0;
  int total = 0;
  for (const Eigen::Vector3d& sample : samples) {
    const Eigen::Vector3d offset = sample - eye;
    const double distance = offset.norm();
    if (distance <= 1e-9) {
      continue;
    }
    PickRay ray;
    ray.origin = eye;
    ray.direction = offset / distance;
    const std::optional<PickHit> hit = pickNearestRenderable(descriptors, ray);
    ++total;
    if (!hit.has_value()) {
      continue;
    }
    // A hit meaningfully closer than the sample that belongs to another
    // renderable blocks the line of sight to the focus subject.
    const bool hitIsFocus = assessAll || focusIds.count(hit->id) > 0u;
    if (!hitIsFocus && hit->distance < distance - 1e-6) {
      ++blocked;
    }
  }
  return total > 0 ? static_cast<double>(blocked) / static_cast<double>(total)
                   : 0.0;
}

double scoreReport(const ViewQualityReport& report)
{
  // Deterministic, monotone score in [0, 1]: coverage and line-of-sight
  // dominate; framing prefers a mid-band subject size; ambiguity discounts.
  const double framingMid
      = std::sqrt(kMinSubjectFraction * kMaxSubjectFraction);
  double framing = 0.0;
  if (report.subjectFraction > 0.0) {
    framing
        = 1.0
          - std::min(
              1.0,
              std::abs(std::log(report.subjectFraction / framingMid)) / 4.0);
  }
  const double visibility = 1.0 - report.occlusionFraction;
  const double clarity = 1.0 - report.ambiguityIoU;
  return std::max(
      0.0,
      report.cornerCoverage * visibility
          * (0.5 + 0.35 * framing + 0.15 * clarity));
}

} // namespace

ViewQualityReport assessView(
    const std::vector<RenderableDescriptor>& descriptors,
    const OrbitCamera& camera,
    int width,
    int height,
    const std::vector<RenderableId>& focusIds,
    const ProjectionOptions& options)
{
  ViewQualityReport report;

  const std::unordered_set<RenderableId> focusSet(
      focusIds.begin(), focusIds.end());
  const bool assessAll = focusSet.empty();
  const auto isFocus = [&](const RenderableDescriptor& descriptor) {
    return assessAll || focusSet.count(descriptor.id) > 0u;
  };

  std::vector<Eigen::Vector3d> focusCorners;
  for (const RenderableDescriptor& descriptor : descriptors) {
    if (!isFocus(descriptor)) {
      continue;
    }
    const std::vector<Eigen::Vector3d> corners
        = descriptorWorldCorners(descriptor);
    focusCorners.insert(focusCorners.end(), corners.begin(), corners.end());
  }
  if (focusCorners.empty()) {
    report.issues.emplace_back("no-bounded-focus");
    return report;
  }

  const int safeWidth = std::max(1, width);
  const int safeHeight = std::max(1, height);
  const double viewportWidth = static_cast<double>(safeWidth);
  const double viewportHeight = static_cast<double>(safeHeight);

  std::vector<Eigen::Vector3d> projected;
  projected.reserve(focusCorners.size());
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (const Eigen::Vector3d& corner : focusCorners) {
    projected.push_back(
        projectToPixels(camera, safeWidth, safeHeight, corner, options));
    centroid += corner;
  }
  centroid /= static_cast<double>(focusCorners.size());

  int onScreen = 0;
  for (const Eigen::Vector3d& point : projected) {
    if (point.z() > 0.0 && point.x() >= 0.0 && point.x() <= viewportWidth
        && point.y() >= 0.0 && point.y() <= viewportHeight) {
      ++onScreen;
    }
  }
  report.cornerCoverage
      = static_cast<double>(onScreen) / static_cast<double>(projected.size());

  const Eigen::Vector3d centerProjected
      = projectToPixels(camera, safeWidth, safeHeight, centroid, options);
  report.centerVisible = centerProjected.z() > 0.0 && centerProjected.x() >= 0.0
                         && centerProjected.x() <= viewportWidth
                         && centerProjected.y() >= 0.0
                         && centerProjected.y() <= viewportHeight;

  const ScreenBox box = screenBox(projected);
  if (box.valid) {
    const double clippedWidth = std::max(
        0.0, std::min(box.maxX, viewportWidth) - std::max(box.minX, 0.0));
    const double clippedHeight = std::max(
        0.0, std::min(box.maxY, viewportHeight) - std::max(box.minY, 0.0));
    report.subjectFraction
        = (clippedWidth * clippedHeight) / (viewportWidth * viewportHeight);
  }

  const OrbitCameraBasis basis = makeOrbitCameraBasis(camera);
  std::vector<Eigen::Vector3d> samples = focusCorners;
  samples.push_back(centroid);
  report.occlusionFraction
      = occlusionFraction(descriptors, focusSet, assessAll, samples, basis.eye);

  // Ambiguity: distinct bodies whose screen boxes pile up while separated in
  // depth suggest a degenerate view axis (e.g. a stack seen from above).
  struct AmbiguityBox
  {
    ScreenBox box;
    double depth = 0.0;
    double extent = 0.0;
  };
  std::vector<AmbiguityBox> boxes;
  for (const RenderableDescriptor& descriptor : descriptors) {
    const std::vector<Eigen::Vector3d> corners
        = descriptorWorldCorners(descriptor);
    if (corners.empty()) {
      continue;
    }
    std::vector<Eigen::Vector3d> descriptorProjected;
    descriptorProjected.reserve(corners.size());
    std::vector<double> depths;
    depths.reserve(corners.size());
    Eigen::Vector3d boundsMin = corners.front();
    Eigen::Vector3d boundsMax = corners.front();
    bool allInFront = true;
    for (const Eigen::Vector3d& corner : corners) {
      const Eigen::Vector3d point
          = projectToPixels(camera, safeWidth, safeHeight, corner, options);
      descriptorProjected.push_back(point);
      depths.push_back(point.z());
      if (point.z() <= 0.0) {
        // Bodies straddling the camera plane project to unbounded boxes and
        // signed depths; they cannot be judged for overlap, so skip them.
        allInFront = false;
      }
      boundsMin = boundsMin.cwiseMin(corner);
      boundsMax = boundsMax.cwiseMax(corner);
    }
    if (!allInFront) {
      continue;
    }
    const ScreenBox descriptorBox = screenBox(descriptorProjected);
    if (!descriptorBox.valid) {
      continue;
    }
    // Only overlap that is actually visible can make a view ambiguous.
    ScreenBox clipped;
    clipped.minX = std::max(descriptorBox.minX, 0.0);
    clipped.minY = std::max(descriptorBox.minY, 0.0);
    clipped.maxX = std::min(descriptorBox.maxX, viewportWidth);
    clipped.maxY = std::min(descriptorBox.maxY, viewportHeight);
    clipped.valid = true;
    if (clipped.maxX <= clipped.minX || clipped.maxY <= clipped.minY) {
      continue;
    }
    AmbiguityBox ambiguity;
    ambiguity.box = clipped;
    ambiguity.depth = median(depths);
    ambiguity.extent = (boundsMax - boundsMin).norm();
    boxes.push_back(ambiguity);
  }
  double worstIoU = 0.0;
  for (std::size_t i = 0; i < boxes.size(); ++i) {
    for (std::size_t j = i + 1u; j < boxes.size(); ++j) {
      const double depthGap = std::abs(boxes[i].depth - boxes[j].depth);
      const double minExtent
          = std::max(std::min(boxes[i].extent, boxes[j].extent), 1e-9);
      if (depthGap < 0.5 * minExtent) {
        continue;
      }
      worstIoU = std::max(worstIoU, boxIoU(boxes[i].box, boxes[j].box));
    }
  }
  report.ambiguityIoU = worstIoU;

  if (report.cornerCoverage < kMinCornerCoverage) {
    report.issues.emplace_back(report.centerVisible ? "cropped" : "off-frame");
  }
  if (report.subjectFraction < kMinSubjectFraction) {
    report.issues.emplace_back("too-far");
  } else if (report.subjectFraction > kMaxSubjectFraction) {
    report.issues.emplace_back("too-close");
  }
  if (report.occlusionFraction > kMaxOcclusionFraction) {
    report.issues.emplace_back("occluded");
  }
  if (report.ambiguityIoU > kMaxAmbiguityIoU) {
    report.issues.emplace_back("ambiguous");
  }

  report.score = scoreReport(report);
  return report;
}

} // namespace dart::gui
