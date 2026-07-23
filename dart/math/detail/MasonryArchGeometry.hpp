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
#include <array>
#include <limits>
#include <utility>
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
// shape, rescaled to centimeters. This is a source-derived C++ port, not a
// byte-identical reproduction: it uses dependency-free Simpson integration
// and bisection in place of the source's SciPy integration and root solve, and
// it converts the resulting coordinates to meters and DART's axis convention.

/// Parameters of the source-derived weighted-catenary masonry-arch generator.
/// Defaults match the published Rigid-IPC generator parameters used for its
/// arch-25-stones.json / arch-101-stones.json scenes (only `nsegs` differs).
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

/// Whether the literal wedge generator keeps Rigid-IPC's small per-stone
/// offsets. The source uses these offsets to give its IPC log barrier a
/// strictly positive initial gap. They are not collision contacts in a
/// complementarity-based rigid-contact model.
enum class MasonryArchBarrierGapPolicy
{
  /// Keep the source generator's offsets exactly before meter/axis conversion.
  IncludeSourceOffsets,

  /// Omit the offsets so adjacent wedge end faces share their nominal plane.
  OmitSourceOffsets,
};

/// Literal eight-vertex hexahedral geometry for one source-derived arch stone.
///
/// Vertices are expressed in meters in the arch's world frame using DART's
/// axis convention (DART_x = source_x, DART_y = source_z/depth,
/// DART_z = source_y/height). Their ordering is the source OBJ generator's
/// p000, p001, p010, p011, p100, p101, p110, p111 order after axis conversion.
/// `centroid` and `volume` are the exact prism centroid and volume computed
/// from the extruded quadrilateral, rather than bounding-box approximations.
/// `momentPerUnitMass` is the exact centroidal inertia tensor of the uniform
/// prism divided by its mass. A caller using density `rho` should therefore
/// set mass to `rho * volume` and moment to
/// `rho * volume * momentPerUnitMass`; it must not use
/// ConvexMeshShape::computeInertia(), which is based on the shape's AABB.
struct MasonryArchStoneWedgeGeometry
{
  std::array<Eigen::Vector3d, 8> vertices{};
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  Eigen::Vector3d min = Eigen::Vector3d::Zero();
  Eigen::Vector3d max = Eigen::Vector3d::Zero();
  double volume = 0.0;
  Eigen::Matrix3d momentPerUnitMass = Eigen::Matrix3d::Zero();
};

/// Source OBJ triangle indices with winding reversed after axis conversion so
/// every face points outward in DART coordinates. The two triangles on each
/// quad are kept separate to match ConvexMeshShape's representation.
inline const std::array<std::array<std::size_t, 3>, 12>&
getMasonryArchStoneWedgeTriangles()
{
  static const std::array<std::array<std::size_t, 3>, 12> triangles = {{
      {{0u, 3u, 2u}},
      {{3u, 0u, 1u}},
      {{4u, 7u, 5u}},
      {{7u, 4u, 6u}},
      {{0u, 5u, 1u}},
      {{5u, 0u, 4u}},
      {{2u, 7u, 6u}},
      {{7u, 2u, 3u}},
      {{0u, 6u, 4u}},
      {{6u, 0u, 2u}},
      {{1u, 7u, 3u}},
      {{7u, 1u, 5u}},
  }};
  return triangles;
}

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

inline std::vector<StoneCrossSectionCorners> generateStoneCrossSections(
    std::size_t nsegs,
    const MasonryArchGeneratorParams& params,
    MasonryArchBarrierGapPolicy gapPolicy)
{
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

    if (gapPolicy == MasonryArchBarrierGapPolicy::IncludeSourceOffsets) {
      // This is the source's literal per-stone offset, applied before
      // springer flattening. Despite its upstream "gap" comment, it is a
      // deterministic arch-shaped translation, not an alternating offset.
      const double midpoint = static_cast<double>(nsegs / 2u);
      const double centeredIndex
          = static_cast<double>(stones.size()) - midpoint;
      const Eigen::Vector2d sourceOffset(
          centeredIndex * 0.1, midpoint * 0.1 - std::abs(centeredIndex * 0.1));
      corners.inner0 += sourceOffset;
      corners.outer0 += sourceOffset;
      corners.inner1 += sourceOffset;
      corners.outer1 += sourceOffset;
    }

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
  for (const auto& stone : stones) {
    minY = std::min(
        {minY,
         stone.inner0.y(),
         stone.outer0.y(),
         stone.inner1.y(),
         stone.outer1.y()});
  }
  const double heightShift = 0.1 - minY;
  for (auto& stone : stones) {
    stone.inner0.y() += heightShift;
    stone.outer0.y() += heightShift;
    stone.inner1.y() += heightShift;
    stone.outer1.y() += heightShift;
  }

  return stones;
}

struct CrossSectionMassProperties
{
  double area = 0.0;
  Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
  Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
};

inline CrossSectionMassProperties computeCrossSectionMassProperties(
    const StoneCrossSectionCorners& stone)
{
  // Counter-clockwise boundary of the extruded quadrilateral in source x/y.
  const std::array<Eigen::Vector2d, 4> polygon
      = {{stone.inner0, stone.inner1, stone.outer1, stone.outer0}};
  double twiceSignedArea = 0.0;
  Eigen::Vector2d weightedCentroid = Eigen::Vector2d::Zero();
  double integralX2Numerator = 0.0;
  double integralY2Numerator = 0.0;
  double integralXyNumerator = 0.0;
  for (std::size_t i = 0u; i < polygon.size(); ++i) {
    const Eigen::Vector2d& current = polygon[i];
    const Eigen::Vector2d& next = polygon[(i + 1u) % polygon.size()];
    const double cross = current.x() * next.y() - next.x() * current.y();
    twiceSignedArea += cross;
    weightedCentroid += (current + next) * cross;
    integralX2Numerator += (current.x() * current.x() + current.x() * next.x()
                            + next.x() * next.x())
                           * cross;
    integralY2Numerator += (current.y() * current.y() + current.y() * next.y()
                            + next.y() * next.y())
                           * cross;
    integralXyNumerator
        += (2.0 * current.x() * current.y() + current.x() * next.y()
            + next.x() * current.y() + 2.0 * next.x() * next.y())
           * cross;
  }

  const double signedArea = 0.5 * twiceSignedArea;
  const Eigen::Vector2d centroid = weightedCentroid / (3.0 * twiceSignedArea);
  const double meanX2 = integralX2Numerator / (12.0 * signedArea);
  const double meanY2 = integralY2Numerator / (12.0 * signedArea);
  const double meanXy = integralXyNumerator / (24.0 * signedArea);

  CrossSectionMassProperties properties;
  properties.area = std::abs(signedArea);
  properties.centroid = centroid;
  properties.covariance(0, 0) = meanX2 - centroid.x() * centroid.x();
  properties.covariance(1, 1) = meanY2 - centroid.y() * centroid.y();
  properties.covariance(0, 1) = properties.covariance(1, 0)
      = meanXy - centroid.x() * centroid.y();
  return properties;
}

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
/// Deliberately DROPS the 0.1cm-per-stone "gap hack" present in the source
/// generator (a barrier-method IPC artifact -- it exists to
/// give IPC's log-barrier a strictly non-zero initial gap, which has no
/// equivalent purpose for DART's constraint solver and would only distort
/// the geometry, especially for the 101-stone arch where the hack's
/// magnitude exceeds a stone's own width). KEEPS the base-flattening step
/// that levels the two springer stones flush to the ground (physically
/// necessary for the arch to rest stably).
///
/// Geometry generation does not encode body mobility. The credited raw
/// Rigid-IPC scenes leave every stone dynamic, while the paper's 25-stone
/// experiment pins both springers; callers must select the intended contract.
inline std::vector<MasonryArchStoneBoxGeometry> generateMasonryArchStoneBoxes(
    std::size_t nsegs,
    const MasonryArchGeneratorParams& params = MasonryArchGeneratorParams())
{
  const auto stones = masonry_arch_detail::generateStoneCrossSections(
      nsegs, params, MasonryArchBarrierGapPolicy::OmitSourceOffsets);

  constexpr double kCentimetersToMeters = 0.01;
  std::vector<MasonryArchStoneBoxGeometry> result;
  result.reserve(stones.size());
  for (const auto& s : stones) {
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

/// Generates the source-derived eight-vertex wedge for every arch stone.
///
/// The default includes Rigid-IPC's per-stone log-barrier gap offsets, as the
/// source generator does. Pass `OmitSourceOffsets` for a collision-only probe
/// in which adjacent nominal end faces meet. Both modes retain the source's
/// springer flattening and whole-arch 0.1 cm height normalization.
///
/// This is not byte-identical to the source OBJ files; see the file-level
/// provenance note above. It does preserve the source vertex ordering,
/// formulas, units after conversion, and axis mapping.
///
/// `endFaceExpansionMeters` is zero for both source and nominal-contact
/// geometry. A positive value is an explicit sensitivity parameter that
/// lengthens each stone by that amount along its centroid chord (half at each
/// end face). It is intended only for collision-backend audits of shallow,
/// physically bounded interface closure; it is not a Rigid-IPC or paper
/// parameter and must be reported whenever nonzero.
inline std::vector<MasonryArchStoneWedgeGeometry>
generateMasonryArchStoneWedges(
    std::size_t nsegs,
    const MasonryArchGeneratorParams& params = MasonryArchGeneratorParams(),
    MasonryArchBarrierGapPolicy gapPolicy
    = MasonryArchBarrierGapPolicy::IncludeSourceOffsets,
    double endFaceExpansionMeters = 0.0)
{
  DART_ASSERT(std::isfinite(endFaceExpansionMeters));
  DART_ASSERT(endFaceExpansionMeters >= 0.0);
  auto stones = masonry_arch_detail::generateStoneCrossSections(
      nsegs, params, gapPolicy);

  constexpr double kCentimetersToMeters = 0.01;
  constexpr double kCubicCentimetersToCubicMeters = 1e-6;
  if (endFaceExpansionMeters > 0.0) {
    const double halfExpansionCentimeters
        = 0.5 * endFaceExpansionMeters / kCentimetersToMeters;
    for (auto& stone : stones) {
      const Eigen::Vector2d faceCenter0 = 0.5 * (stone.inner0 + stone.outer0);
      const Eigen::Vector2d faceCenter1 = 0.5 * (stone.inner1 + stone.outer1);
      const Eigen::Vector2d chordDirection
          = (faceCenter1 - faceCenter0).normalized();
      const Eigen::Vector2d expansion
          = halfExpansionCentimeters * chordDirection;
      stone.inner0 -= expansion;
      stone.outer0 -= expansion;
      stone.inner1 += expansion;
      stone.outer1 += expansion;
    }
  }

  std::vector<MasonryArchStoneWedgeGeometry> result;
  result.reserve(stones.size());
  for (const auto& stone : stones) {
    const double halfDepth = 0.5 * stone.width;
    MasonryArchStoneWedgeGeometry geometry;
    geometry.vertices = {{
        kCentimetersToMeters
            * Eigen::Vector3d(stone.inner0.x(), -halfDepth, stone.inner0.y()),
        kCentimetersToMeters
            * Eigen::Vector3d(stone.inner0.x(), halfDepth, stone.inner0.y()),
        kCentimetersToMeters
            * Eigen::Vector3d(stone.inner1.x(), -halfDepth, stone.inner1.y()),
        kCentimetersToMeters
            * Eigen::Vector3d(stone.inner1.x(), halfDepth, stone.inner1.y()),
        kCentimetersToMeters
            * Eigen::Vector3d(stone.outer0.x(), -halfDepth, stone.outer0.y()),
        kCentimetersToMeters
            * Eigen::Vector3d(stone.outer0.x(), halfDepth, stone.outer0.y()),
        kCentimetersToMeters
            * Eigen::Vector3d(stone.outer1.x(), -halfDepth, stone.outer1.y()),
        kCentimetersToMeters
            * Eigen::Vector3d(stone.outer1.x(), halfDepth, stone.outer1.y()),
    }};

    const auto crossSection
        = masonry_arch_detail::computeCrossSectionMassProperties(stone);
    geometry.centroid
        = kCentimetersToMeters
          * Eigen::Vector3d(
              crossSection.centroid.x(), 0.0, crossSection.centroid.y());
    geometry.volume
        = crossSection.area * stone.width * kCubicCentimetersToCubicMeters;

    constexpr double kSquareCentimetersToSquareMeters = 1e-4;
    const double varianceX
        = crossSection.covariance(0, 0) * kSquareCentimetersToSquareMeters;
    const double varianceZ
        = crossSection.covariance(1, 1) * kSquareCentimetersToSquareMeters;
    const double covarianceXz
        = crossSection.covariance(0, 1) * kSquareCentimetersToSquareMeters;
    const double depthMeters = stone.width * kCentimetersToMeters;
    const double varianceY = depthMeters * depthMeters / 12.0;
    geometry.momentPerUnitMass(0, 0) = varianceY + varianceZ;
    geometry.momentPerUnitMass(1, 1) = varianceX + varianceZ;
    geometry.momentPerUnitMass(2, 2) = varianceX + varianceY;
    geometry.momentPerUnitMass(0, 2) = geometry.momentPerUnitMass(2, 0)
        = -covarianceXz;

    geometry.min
        = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
    geometry.max = -geometry.min;
    for (const auto& vertex : geometry.vertices) {
      geometry.min = geometry.min.cwiseMin(vertex);
      geometry.max = geometry.max.cwiseMax(vertex);
    }

    result.push_back(geometry);
  }

  return result;
}

} // namespace detail
} // namespace math
} // namespace dart

#endif // DART_MATH_DETAIL_MASONRYARCHGEOMETRY_HPP_
