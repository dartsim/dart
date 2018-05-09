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

#include "dart/dynamics/HeightmapShape.hpp"
#include <type_traits>

namespace dart {
namespace collision {
namespace detail {

#define HF_THICKNESS 0.05

//==============================================================================
template<class HeightType>
void setOdeHeightfieldDetails(const dHeightfieldDataID odeHeightfieldID,
                         const std::vector<HeightType>& heights, 
                         const size_t& width,
                         const size_t& height,
                         const Eigen::Vector3d& scale,
                         typename std::enable_if<std::is_same<float, HeightType>::value>::type* = 0) 
{
  assert(width >= 2);
  assert(height >= 2);
  if ((width < 2) || (height < 2))
  {
    dtwarn << "Cannot create height field of dimensions " << width
           << "x" << height << ", needs to be at least 2" << std::endl;
    return;
  }
  dGeomHeightfieldDataBuildSingle(
      odeHeightfieldID,
      &(heights[0]),
      0,
      (width-1) * scale.x(),   // width (in meters)
      (height-1) * scale.y(),  // height (in meters)
      width,               // width (sampling size)
      height,              // height (sampling size)
      scale.z(),           // vertical (z-axis) scaling 
      0.0,                 // vertical (z-axis) offset
      HF_THICKNESS,        // vertical thickness for closing the height map mesh
      0);                  // wrap mode
}

//==============================================================================
template<class HeightType>
void setOdeHeightfieldDetails(const dHeightfieldDataID odeHeightfieldID,
                         const std::vector<HeightType>& heights, 
                         const size_t& width,
                         const size_t& height,
                         const Eigen::Vector3d& scale,
                         typename std::enable_if<std::is_same<double, HeightType>::value>::type* = 0) 
{
  dGeomHeightfieldDataBuildDouble(
      odeHeightfieldID,
      &(heights[0]),
      0,
      width * scale.x(),   // width (in meters)
      height * scale.y(),  // height (in meters)
      width,               // width (sampling size)
      height,              // height (sampling size)
      scale.z(),           // vertical (z-axis) scaling 
      0.0,                 // vertical (z-axis) offset
      HF_THICKNESS,        // vertical thickness for closing the height map mesh
      0);                  // wrap mode
}



//==============================================================================
OdeHeightmap::OdeHeightmap(const OdeCollisionObject* parent,
                           const dart::dynamics::HeightmapShape* heightMap)
  : OdeGeom(parent)
{
  using dart::dynamics::HeightmapShape;
  assert(heightMap);

  // get the heightmap parameters
  const Eigen::Vector3d& scale = heightMap->getScale();
  const std::vector<HeightmapShape::HeightType>& heights =
    heightMap->getHeightField();

  // Create the ODE heightfield
  odeHeightfieldID = dGeomHeightfieldDataCreate();
  
  static_assert(std::is_same<HeightmapShape::HeightType, float>::value ||
                std::is_same<HeightmapShape::HeightType, double>::value,
                "Height field needs to be double or float");

  // specify height field details
  setOdeHeightfieldDetails(odeHeightfieldID, heights, heightMap->getWidth(),
                      heightMap->getHeight(), scale);

  // Restrict the bounds of the AABB to improve efficiency
  dGeomHeightfieldDataSetBounds(odeHeightfieldID, heightMap->getMinHeight(),
      heightMap->getMaxHeight());

  // create the height field
  mGeomId = dCreateHeightfield(0, odeHeightfieldID, 1);

  // rotate it so that z axis is up 
  dQuaternion q;
  q[0] = sqrt(0.5);
  q[1] = sqrt(0.5);
  q[2] = 0;
  q[3] = 0;
  dGeomSetQuaternion(mGeomId, q);

  dReal aabb[6];
  dGeomGetAABB(mGeomId, aabb);
  dtdbg << "Heightfield AABB: min = {"
    << aabb[0] << ", " << aabb[2] << ", " << aabb[4]<< "} max = {"
    << aabb[1] << ", " << aabb[3] << ", " << aabb[5] << "}" << std::endl;
}

//==============================================================================
OdeHeightmap::~OdeHeightmap()
{
  dGeomHeightfieldDataDestroy(odeHeightfieldID);
  dGeomDestroy(mGeomId);
}

} // namespace detail
} // namespace collision
} // namespace dart
