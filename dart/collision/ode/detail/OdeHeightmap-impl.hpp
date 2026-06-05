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

#include "dart/common/Macros.hpp"

#include <dart/collision/ode/detail/OdeHeightmap.hpp>

#include <dart/dynamics/HeightmapShape.hpp>

#include <algorithm>
#include <limits>
#include <type_traits>

#include <cmath>

namespace dart {
namespace collision {
namespace detail {

#define HF_THICKNESS 0.05

//==============================================================================
// A HeightmapShape scale/heights can be finite (so they pass the shape-level
// validation guard) yet make the values DART hands to ODE non-finite: e.g.
// (width-1)*scale.x() overflows to +/-Inf for a large field, or
// maxHeight*scale.z overflows ODE's vertical AABB. ODE then derives a
// non-finite sample spacing / AABB, collapses its per-cell index range, and
// aborts via dIASSERT((nMinX < nMaxX) && (nMinZ < nMaxZ)) in
// dCollideHeightfield. We intervene ONLY on an actual overflow (a non-finite
// value), clamping to a finite bound well below sqrt(DBL_MAX); finite values
// (even large but valid terrains) are passed through unchanged.
// See https://github.com/gazebosim/gz-physics/issues/847
constexpr double kOdeHeightfieldMaxExtent = 1.0e12;

// Replaces a non-finite extent (the result of an overflowed product) with a
// finite fallback so ODE's heightfield index math stays valid; finite inputs
// are returned unchanged. Finiteness is validated AFTER the dReal cast: when
// ODE is built single precision (dReal == float), a finite double larger than
// FLT_MAX overflows to +/-Inf on the cast and would still abort.
template <typename S>
dReal finiteOdeExtentOr(S value, const char* what)
{
  const dReal r = static_cast<dReal>(value);
  if (std::isfinite(r))
    return r;

  const double v = static_cast<double>(value);
  const dReal fallback
      = static_cast<dReal>(std::copysign(kOdeHeightfieldMaxExtent, v));
  dtwarn << "[OdeHeightmap] Non-finite " << what
         << " (likely from an extreme HeightmapShape scale); clamping to "
         << fallback << " to avoid an ODE heightfield collision abort.\n";
  return fallback;
}

//==============================================================================
// creates the ODE height field. Only enabled if the height data type is float.
template <class S>
void setOdeHeightfieldDetails(
    const dHeightfieldDataID odeHeightfieldId,
    const S* heights,
    const std::size_t& width,
    const std::size_t& height,
    const Eigen::Matrix<S, 3, 1>& scale,
    typename std::enable_if<std::is_same<float, S>::value>::type* = 0)
{
  DART_ASSERT(width >= 2);
  DART_ASSERT(height >= 2);
  if ((width < 2) || (height < 2)) {
    dtwarn << "Cannot create height field of dimensions " << width << "x"
           << height << ", needs to be at least 2" << std::endl;
    return;
  }
  dGeomHeightfieldDataBuildSingle(
      odeHeightfieldId,
      heights,
      0,
      finiteOdeExtentOr((width - 1) * scale.x(), "width extent"),
      finiteOdeExtentOr((height - 1) * scale.y(), "depth extent"),
      width,                                          // width (sampling size)
      height,                                         // height (sampling size)
      finiteOdeExtentOr(scale.z(), "vertical scale"), // clamped by the ctor too
      0.0,                                            // vertical offset
      HF_THICKNESS, // vertical thickness for closing the mesh
      0);           // wrap mode
}

//==============================================================================
// creates the ODE height field. Only enabled if the height data type is double.
template <class S>
void setOdeHeightfieldDetails(
    const dHeightfieldDataID odeHeightfieldId,
    const S* heights,
    const std::size_t& width,
    const std::size_t& height,
    const Eigen::Matrix<S, 3, 1>& scale,
    typename std::enable_if<std::is_same<double, S>::value>::type* = 0)
{
  DART_ASSERT(width >= 2);
  DART_ASSERT(height >= 2);
  if ((width < 2) || (height < 2)) {
    dtwarn << "Cannot create height field of dimensions " << width << "x"
           << height << ", needs to be at least 2" << std::endl;
    return;
  }

  dGeomHeightfieldDataBuildDouble(
      odeHeightfieldId,
      heights,
      0,
      finiteOdeExtentOr((width - 1) * scale.x(), "width extent"),
      finiteOdeExtentOr((height - 1) * scale.y(), "depth extent"),
      width,                                          // width (sampling size)
      height,                                         // height (sampling size)
      finiteOdeExtentOr(scale.z(), "vertical scale"), // clamped by the ctor too
      0.0,                                            // vertical offset
      HF_THICKNESS, // vertical thickness for closing the mesh
      0);           // wrap mode
}

//==============================================================================
template <typename S>
OdeHeightmap<S>::OdeHeightmap(
    const OdeCollisionObject* parent,
    const dynamics::HeightmapShape<S>* heightMap)
  : OdeGeom(parent)
{
  DART_ASSERT(heightMap);

  // get the heightmap parameters
  Eigen::Matrix<S, 3, 1> scale = heightMap->getScale();
  const auto& heights = heightMap->getHeightField();
  const double minHeight = static_cast<double>(heightMap->getMinHeight());
  const double maxHeight = static_cast<double>(heightMap->getMaxHeight());

  // ODE derives the heightfield's vertical AABB from the (unscaled) height
  // bounds multiplied by the vertical scale, while its height callback returns
  // sample * scale.z. If scale.z * |height bound| overflows to non-finite, the
  // AABB collapses and dCollideHeightfield aborts. Clamp the vertical scale
  // ONLY on an actual overflow (not on large-but-finite values) so the AABB
  // stays consistent with the heights ODE will sample.
  // See https://github.com/gazebosim/gz-physics/issues/847
  const double maxAbsBound
      = std::max(std::max(std::abs(minHeight), std::abs(maxHeight)), 1.0);
  // Check the product in dReal: ODE forms the vertical AABB as
  // bound * scale.z in dReal, so two values that are each finite (even in a
  // single-precision build where dReal == float) can still overflow when
  // multiplied. A double-only check would miss e.g. scale.z == 1e20 with a
  // 1e20 bound.
  const dReal scaledBound
      = static_cast<dReal>(maxAbsBound) * static_cast<dReal>(scale.z());
  if (!std::isfinite(scaledBound)) {
    const double safeZ = std::copysign(
        kOdeHeightfieldMaxExtent / maxAbsBound, static_cast<double>(scale.z()));
    dtwarn << "[OdeHeightmap] Vertical scale (" << scale.z()
           << ") combined with the height bound (" << maxAbsBound
           << ") overflows ODE's heightfield AABB; clamping vertical scale to "
           << safeZ << ".\n";
    scale.z() = static_cast<S>(safeZ);
  }

  // Create the ODE heightfield
  mOdeHeightfieldId = dGeomHeightfieldDataCreate();

  // specify height field details (the width/depth extents are clamped only if
  // they overflow; the vertical scale was clamped above if needed)
  setOdeHeightfieldDetails(
      mOdeHeightfieldId,
      heights.data(),
      heightMap->getWidth(),
      heightMap->getDepth(),
      scale);

  // Restrict the bounds of the AABB to improve efficiency.
  //
  // ODE applies the (now finite) vertical scale/offset when computing the AABB,
  // so we pass the unscaled bounds here. Passing the real bounds keeps the AABB
  // consistent with the heights ODE samples from the field.
  dGeomHeightfieldDataSetBounds(
      mOdeHeightfieldId,
      finiteOdeExtentOr(minHeight, "min height"),
      finiteOdeExtentOr(maxHeight, "max height"));

  // create the height field
  mGeomId = dCreateHeightfield(0, mOdeHeightfieldId, 1);

  // rotate it so that z axis is up.
  // remember the transform as a permanent relative transform by assigning
  // it to the geometry.
  dQuaternion q;
  q[0] = sqrt(0.5);
  q[1] = sqrt(0.5);
  q[2] = 0;
  q[3] = 0;
  dGeomSetQuaternion(mGeomId, q);

  // Center the ODE heightfield so its visual and collision representations
  // share the same origin.
  dGeomSetPosition(mGeomId, 0.0, 0.0, 0.0);
}

//==============================================================================
template <typename S>
OdeHeightmap<S>::~OdeHeightmap()
{
  dGeomHeightfieldDataDestroy(mOdeHeightfieldId);
  dGeomDestroy(mGeomId);
}

} // namespace detail
} // namespace collision
} // namespace dart
