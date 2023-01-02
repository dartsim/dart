/*
 * Copyright (c) 2011-2022, The DART development contributors
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

#include "dart/dynamics/ode/detail/OdePlane.hpp"

#include "dart/dynamics/PlaneShape.hpp"

namespace dart {
namespace collision {
namespace detail {

//==============================================================================
OdePlane::OdePlane(
    const OdeCollisionObject* parent,
    const Eigen::Vector3d& normal,
    double offset)
  : OdeGeom(parent)
{
  mGeomId = dCreatePlane(0, normal.x(), normal.y(), normal.z(), offset);
}

//==============================================================================
OdePlane::~OdePlane()
{
  dGeomDestroy(mGeomId);
}

//==============================================================================
void OdePlane::updateEngineData()
{
  const Eigen::Isometry3d& tf = mParentCollisionObject->getTransform();
  const Eigen::Vector3d pos = tf.translation();
  const Eigen::Matrix3d rot = tf.linear();

  auto plane = static_cast<const dynamics::PlaneShape*>(
      mParentCollisionObject->getShape().get());
  const Eigen::Vector3d& normal = plane->getNormal();
  const double offset = plane->getOffset();

  const Eigen::Vector3d& normal2 = rot * normal;
  const double offset2 = offset + pos.dot(normal2);

  dGeomPlaneSetParams(mGeomId, normal2.x(), normal2.y(), normal2.z(), offset2);
}

//==============================================================================
bool OdePlane::isPlaceable() const
{
  return false;
}

} // namespace detail
} // namespace collision
} // namespace dart
