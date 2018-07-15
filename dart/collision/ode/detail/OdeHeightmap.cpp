/*
 * Copyright (c) 2011-2018, The DART development contributors
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

#include "dart/collision/ode/detail/OdeHeightmap.hpp"

#include <type_traits>
#include "dart/dynamics/HeightmapShape.hpp"

namespace dart {
namespace collision {
namespace detail {

#define HF_THICKNESS 0.05

//==============================================================================
// creates the ODE height field. Only enabled if the height data type is float.
template <class HeightType>
void setOdeHeightfieldDetails(
    const dHeightfieldDataID odeHeightfieldId,
    const HeightType* heights,
    const std::size_t& width,
    const std::size_t& height,
    const Eigen::Vector3d& scale,
    typename std::enable_if<std::is_same<float, HeightType>::value>::type* = 0)
{
  assert(width >= 2);
  assert(height >= 2);
  if ((width < 2) || (height < 2))
  {
    dtwarn << "Cannot create height field of dimensions " << width << "x"
           << height << ", needs to be at least 2" << std::endl;
    return;
  }
  dGeomHeightfieldDataBuildSingle(
      odeHeightfieldId,
      heights,
      0,
      (width - 1) * scale.x(),  // width (in meters)
      (height - 1) * scale.y(), // height (in meters)
      width,                    // width (sampling size)
      height,                   // height (sampling size)
      scale.z(),                // vertical scaling
      0.0,                      // vertical offset
      HF_THICKNESS,             // vertical thickness for closing the mesh
      0);                       // wrap mode
}

//==============================================================================
// creates the ODE height field. Only enabled if the height data type is double.
template <class HeightType>
void setOdeHeightfieldDetails(
    const dHeightfieldDataID odeHeightfieldId,
    const HeightType* heights,
    const std::size_t& width,
    const std::size_t& height,
    const Eigen::Vector3d& scale,
    typename std::enable_if<std::is_same<double, HeightType>::value>::type* = 0)
{
  assert(width >= 2);
  assert(height >= 2);
  if ((width < 2) || (height < 2))
  {
    dtwarn << "Cannot create height field of dimensions " << width << "x"
           << height << ", needs to be at least 2" << std::endl;
    return;
  }

  dGeomHeightfieldDataBuildDouble(
      odeHeightfieldId,
      heights,
      0,
      (width - 1) * scale.x(),  // width (in meters)
      (height - 1) * scale.y(), // height (in meters)
      width,                    // width (sampling size)
      height,                   // height (sampling size)
      scale.z(),                // vertical scaling
      0.0,                      // vertical offset
      HF_THICKNESS,             // vertical thickness for closing the mesh
      0);                       // wrap mode
}

//==============================================================================
OdeHeightmap::OdeHeightmap(
    const OdeCollisionObject* parent,
    const dynamics::HeightmapShape* heightMap)
  : OdeGeom(parent)
{
  using dynamics::HeightmapShape;
  assert(heightMap);

  // get the heightmap parameters
  const Eigen::Vector3d& scale = heightMap->getScale();
  const HeightmapShape::HeightField& heights = heightMap->getHeightField();

  // Create the ODE heightfield
  mOdeHeightfieldId = dGeomHeightfieldDataCreate();

  static_assert(
      std::is_same<HeightmapShape::HeightType, float>::value
          || std::is_same<HeightmapShape::HeightType, double>::value,
      "Height field needs to be double or float");

  // specify height field details
  setOdeHeightfieldDetails(
      mOdeHeightfieldId,
      heights.data(),
      heightMap->getWidth(),
      heightMap->getDepth(),
      scale);

  // Restrict the bounds of the AABB to improve efficiency
  dGeomHeightfieldDataSetBounds(
      mOdeHeightfieldId, heightMap->getMinHeight(), heightMap->getMaxHeight());

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

  // TODO Take this out as soon as testing is finished, getting the
  // AABB is only needed for the debug print.
  dReal aabb[6];
  dGeomGetAABB(mGeomId, aabb);
  dtdbg << "ODE Heightfield AABB: min = {" << aabb[0] << ", " << aabb[2] << ", "
        << aabb[4] << "} max = {" << aabb[1] << ", " << aabb[3] << ", "
        << aabb[5] << "}" << std::endl;
}

//==============================================================================
OdeHeightmap::~OdeHeightmap()
{
  dGeomHeightfieldDataDestroy(mOdeHeightfieldId);
  dGeomDestroy(mGeomId);
}

} // namespace detail
} // namespace collision
} // namespace dart
