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

#ifndef DART_MATH_DETAIL_MASONRYARCHGEOMETRY_HPP_
#define DART_MATH_DETAIL_MASONRYARCHGEOMETRY_HPP_

#include <dart/common/Macros.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <limits>
#include <vector>

#include <cmath>
#include <cstddef>

namespace dart {
namespace math {
namespace detail {

// Arch geometry adapted from ipc-sim/rigid-ipc (MIT), commit 23b6ba6
// (tools/meshes/large_arch.py), the masonry-arch scene generator credited by
// the SCA 2026 FBF paper (Rigid-IPC's own cited scene generator; the paper's
// code repository is unavailable, see
// docs/dev_tasks/fbf_exact_coulomb_friction/ for provenance). The formula
// itself is the real St. Louis Gateway Arch's documented "weighted catenary"
// shape, rescaled to centimeters. See fixtures/3D/friction/arch/*.json in the
// source repository for the byte-identical reference scenes this reproduces.

/// Parameters of the author-faithful weighted-catenary masonry-arch
/// generator. Defaults reproduce the source repository's arch-25-stones.json
/// / arch-101-stones.json scenes (only the stone count `nsegs` differs
/// between them).
struct MasonryArchGeneratorParams
{
  /// Maximum centroid height, in centimeters (source variable: `fc`).
  double crownHeight = 60.0;

  /// Base cross-sectional area, in square centimeters (source: `Qb`).
  double baseArea = 100.0;

  /// Crown cross-sectional area, in square centimeters (source: `Qt`).
  double crownArea = 49.0;

  /// Half-width of the centroid curve at the base, in centimeters
  /// (source: `L`).
  double halfWidth = 30.0;
};

/// Box-approximation geometry for a single masonry-arch stone.
///
/// Already converted to DART's meter units and axis convention
/// (DART_x = ripc_x, DART_y = ripc_z [depth], DART_z = ripc_y [height]), and
/// centered at the world origin (the caller may rigidly transform the whole
/// arch, e.g. to place it at a scene-specific location).
struct MasonryArchStoneBoxGeometry
{
  /// World-frame pose of the stone's centroid and box-local axes.
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  /// Full box extents (not half-extents), in meters: (along-arch chord
  /// length, extrusion depth, in-plane radial thickness). The source
  /// generator gives every stone a tapering *square* cross-section, so the
  /// depth and radial-thickness components are always equal.
  Eigen::Vector3d size = Eigen::Vector3d::Zero();
};

namespace masonry_arch_detail {

/// Closed-form weighted-catenary centroid curve and its derivatives.
struct CatenaryModel
{
  double A = 0.0;
  double C = 0.0;
  double fc = 0.0;
  double Qb = 0.0;
  double Qt = 0.0;
  double L = 0.0;

  double height(double x) const
  {
    return -A * (std::cosh(C * x / L) - 1.0) + fc;
  }

  double slope(double x) const
  {
    return -A * std::sinh(C * x / L) * C / L;
  }

  double arcLengthIntegrand(double x) const
  {
    const double coshTerm = std::cosh(C * x / L);
    return std::sqrt(
        1.0 + A * A * (coshTerm * coshTerm - 1.0) * C * C / (L * L));
  }
};

/// Composite Simpson's rule, matching the source repository's
/// dependency-free reference re-implementation (verify_arch.py) so this
/// port's numerics are independently checkable against it.
inline double simpsonIntegral(
    const CatenaryModel& model, double a, double b, int n = 2000)
{
  if (n % 2 == 1)
    ++n;
  const double h = (b - a) / static_cast<double>(n);
  double s = model.arcLengthIntegrand(a) + model.arcLengthIntegrand(b);
  for (int i = 1; i < n; ++i) {
    const double x = a + static_cast<double>(i) * h;
    s += model.arcLengthIntegrand(x) * (i % 2 == 0 ? 2.0 : 4.0);
  }
  return s * h / 3.0;
}

/// Finds the next equal-arc-length stone boundary `x1 > x0`, i.e. the root
/// of `arcLength(x0, x1) - targetSegmentLength == 0`. `arcLengthIntegrand`
/// is positive everywhere, so the residual is monotonically increasing in
/// `x1` and a bisection search is well posed given `hi` large enough.
inline double bisectNextBoundary(
    const CatenaryModel& model,
    double x0,
    double targetSegmentLength,
    double hi,
    double tol = 1e-10,
    int maxIterations = 200)
{
  const auto residual = [&](double x) {
    return simpsonIntegral(model, x0, x) - targetSegmentLength;
  };

  double lo = x0;
  double flo = residual(lo);
  for (int i = 0; i < maxIterations; ++i) {
    const double mid = 0.5 * (lo + hi);
    const double fm = residual(mid);
    if (std::abs(fm) < tol)
      return mid;
    if ((fm > 0.0) == (flo > 0.0)) {
      lo = mid;
      flo = fm;
    } else {
      hi = mid;
    }
  }
  return 0.5 * (lo + hi);
}

/// The four in-plane (ripc x, ripc y) corners of one stone's wedge
/// cross-section, before z-extrusion. `width` is the stone's constant
/// (non-tapering) cross-section side length, in centimeters.
struct StoneCrossSectionCorners
{
  Eigen::Vector2d inner0 = Eigen::Vector2d::Zero();
  Eigen::Vector2d outer0 = Eigen::Vector2d::Zero();
  Eigen::Vector2d inner1 = Eigen::Vector2d::Zero();
  Eigen::Vector2d outer1 = Eigen::Vector2d::Zero();
  double width = 0.0;
};

} // namespace masonry_arch_detail

/// Generates box-approximation geometry for every stone of an
/// `nsegs`-stone weighted-catenary masonry arch, following the closed-form
/// generator in ipc-sim/rigid-ipc's tools/meshes/large_arch.py. Stones are
/// numbered `0 .. nsegs - 1` from one springer (base) end to the other.
///
/// This is a *box* approximation of the source's true 8-vertex hexahedral
/// wedge stones (tilted, non-parallel end faces): each stone's box is fit
/// to the wedge's two end-face centroids (chord length, position,
/// tangent-following orientation) and the wedge's constant square
/// cross-section (width == depth). DART's native collision detector
/// (`collision::DARTCollisionDetector`, the detector these paper fixtures
/// use) has no general convex-mesh/wedge-wedge contact-manifold support, so
/// this is the fidelity tier DART's collision pipeline can back reliably;
/// see docs/dev_tasks/fbf_exact_coulomb_friction/ for the fidelity-tier
/// evidence and the literal-wedge-mesh tier that was evaluated and set
/// aside.
///
/// Deliberately DROPS the alternating +/-0.1cm-per-stone "gap hack" present
/// in the source generator (a barrier-method IPC artifact -- it exists to
/// give IPC's log-barrier a strictly non-zero initial gap, which has no
/// equivalent purpose for DART's constraint solver and would only distort
/// the geometry, especially for the 101-stone arch where the hack's
/// magnitude exceeds a stone's own width). KEEPS the base-flattening step
/// that levels the two springer stones flush to the ground (physically
/// necessary for the arch to rest stably).
///
/// All returned stones are meant to be fully dynamic rigid bodies; per the
/// source scene files, the only fixed body is a ground plane (the caller is
/// expected to add one), not any of the arch stones themselves.
inline std::vector<MasonryArchStoneBoxGeometry> generateMasonryArchStoneBoxes(
    std::size_t nsegs,
    const MasonryArchGeneratorParams& params = MasonryArchGeneratorParams())
{
  using masonry_arch_detail::bisectNextBoundary;
  using masonry_arch_detail::CatenaryModel;
  using masonry_arch_detail::simpsonIntegral;
  using masonry_arch_detail::StoneCrossSectionCorners;

  DART_ASSERT(nsegs > 0u);

  CatenaryModel model;
  model.fc = params.crownHeight;
  model.Qb = params.baseArea;
  model.Qt = params.crownArea;
  model.L = params.halfWidth;
  model.A = model.fc / (model.Qb / model.Qt - 1.0);
  model.C = std::acosh(model.Qb / model.Qt);

  const double arcLength = simpsonIntegral(model, -model.L, model.L);
  const double targetSegmentLength = arcLength / static_cast<double>(nsegs);
  const double sqrtQb = std::sqrt(model.Qb);
  const double sqrtQt = std::sqrt(model.Qt);

  std::vector<StoneCrossSectionCorners> stones;
  stones.reserve(nsegs);

  double x0 = -model.L;
  while (x0 < model.L * 0.999) {
    const double x1
        = bisectNextBoundary(model, x0, targetSegmentLength, model.L * 1.0001);
    const double y0 = model.height(x0);
    const double y1 = model.height(x1);
    const double dydx0 = model.slope(x0);
    const double dydx1 = model.slope(x1);

    Eigen::Vector2d v0(-dydx0, 1.0);
    Eigen::Vector2d v1(-dydx1, 1.0);
    v0.normalize();
    v1.normalize();

    double a0 = std::clamp(y0 / model.fc, 0.0, 1.0);
    double a1 = std::clamp(y1 / model.fc, 0.0, 1.0);
    double w0 = sqrtQb + a0 * (sqrtQt - sqrtQb);
    double w1 = sqrtQb + a1 * (sqrtQt - sqrtQb);
    // The source generator holds each individual stone's cross-section
    // constant (no internal taper): the crown-ward face reuses the
    // base-ward face's width.
    if (x0 < 0.0)
      w1 = w0;
    else
      w0 = w1;

    StoneCrossSectionCorners corners;
    corners.width = w0; // == w1 after the constant-width step above.
    const Eigen::Vector2d p0(x0, y0);
    const Eigen::Vector2d p1(x1, y1);
    corners.inner0 = p0 - 0.5 * w0 * v0;
    corners.outer0 = p0 + 0.5 * w0 * v0;
    corners.inner1 = p1 - 0.5 * w1 * v1;
    corners.outer1 = p1 + 0.5 * w1 * v1;
    stones.push_back(corners);

    x0 = x1;
  }

  DART_ASSERT(stones.size() == nsegs);

  // Base flattening: level the two springer stones' outer (ground-facing)
  // corner flush with their inner corner's height, at the end of the stone
  // that meets the ground.
  if (!stones.empty()) {
    auto& first = stones.front();
    const double firstSlope = (first.outer0.y() - first.outer1.y())
                              / (first.outer0.x() - first.outer1.x());
    const double firstTargetY = first.inner0.y();
    const double firstTargetX
        = (firstTargetY - first.outer1.y()) / firstSlope + first.outer1.x();
    first.outer0 = Eigen::Vector2d(firstTargetX, firstTargetY);

    auto& last = stones.back();
    const double lastSlope = (last.outer1.y() - last.outer0.y())
                             / (last.outer1.x() - last.outer0.x());
    const double lastTargetY = last.inner1.y();
    const double lastTargetX
        = (lastTargetY - last.outer0.y()) / lastSlope + last.outer0.x();
    last.outer1 = Eigen::Vector2d(lastTargetX, lastTargetY);
  }

  // Shift the whole arch so its lowest corner sits 0.1 cm above the local
  // ground datum, matching the source generator's own normalization.
  double minY = std::numeric_limits<double>::infinity();
  for (const auto& s : stones) {
    minY = std::min(
        {minY, s.inner0.y(), s.outer0.y(), s.inner1.y(), s.outer1.y()});
  }
  const double heightShift = 0.1 - minY;

  constexpr double kCentimetersToMeters = 0.01;
  std::vector<MasonryArchStoneBoxGeometry> result;
  result.reserve(stones.size());
  for (auto& s : stones) {
    s.inner0.y() += heightShift;
    s.outer0.y() += heightShift;
    s.inner1.y() += heightShift;
    s.outer1.y() += heightShift;

    const Eigen::Vector2d faceCenter0 = 0.5 * (s.inner0 + s.outer0);
    const Eigen::Vector2d faceCenter1 = 0.5 * (s.inner1 + s.outer1);
    const Eigen::Vector2d centroid2D = 0.5 * (faceCenter0 + faceCenter1);
    const Eigen::Vector2d chord = faceCenter1 - faceCenter0;
    const double chordLength = chord.norm();
    const Eigen::Vector2d xLocal2D = chord / chordLength;
    // 90-degree in-plane rotation of xLocal2D. This makes
    // col(0).cross(col(1)) == col(2) below exactly, i.e. a proper
    // right-handed orthonormal frame.
    const Eigen::Vector2d zLocal2D(-xLocal2D.y(), xLocal2D.x());

    MasonryArchStoneBoxGeometry stoneGeometry;
    stoneGeometry.transform.linear().col(0)
        = Eigen::Vector3d(xLocal2D.x(), 0.0, xLocal2D.y());
    // The extrusion (depth) axis never rotates out of the arch plane.
    stoneGeometry.transform.linear().col(1) = Eigen::Vector3d::UnitY();
    stoneGeometry.transform.linear().col(2)
        = Eigen::Vector3d(zLocal2D.x(), 0.0, zLocal2D.y());
    stoneGeometry.transform.translation()
        = kCentimetersToMeters
          * Eigen::Vector3d(centroid2D.x(), 0.0, centroid2D.y());
    stoneGeometry.size
        = kCentimetersToMeters * Eigen::Vector3d(chordLength, s.width, s.width);
    result.push_back(stoneGeometry);
  }

  return result;
}

} // namespace detail
} // namespace math
} // namespace dart

#endif // DART_MATH_DETAIL_MASONRYARCHGEOMETRY_HPP_
