/*
 * Copyright (c) 2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#include "dart/collision/dart/CollideSphereBox.hpp"

#include "dart/math/Constants.hpp"

#define DART_COLLISION_EPS  1E-6

namespace dart {
namespace collision {

//==============================================================================
void collideSphereBox(
    const dynamics::SphereShape& shapeA, const Eigen::Isometry3d& tfA,
    const dynamics::BoxShape& shapeB, const Eigen::Isometry3d& tfB,
    NarrowPhaseCallback* callback)
{
  Eigen::Vector3d size = 0.5 * shapeB.getSize();
  bool inside_box = true;

  // clipping a center of the sphere to a boundary of the box
  Eigen::Vector3d c0 = tfA.translation();
  Eigen::Vector3d p = tfB.inverse() * c0;

  if (p[0] < -size[0]) { p[0] = -size[0]; inside_box = false; }
  if (p[0] >  size[0]) { p[0] =  size[0]; inside_box = false; }

  if (p[1] < -size[1]) { p[1] = -size[1]; inside_box = false; }
  if (p[1] >  size[1]) { p[1] =  size[1]; inside_box = false; }

  if (p[2] < -size[2]) { p[2] = -size[2]; inside_box = false; }
  if (p[2] >  size[2]) { p[2] =  size[2]; inside_box = false; }


  Eigen::Vector3d normal(0.0, 0.0, 0.0);
  double penetration;

  if ( inside_box )
  {
    // find nearest side from the sphere center
    double min = size[0] - std::abs(p[0]);
    double tmin = size[1] - std::abs(p[1]);
    int idx = 0;

    if ( tmin < min )
    {
      min = tmin;
      idx = 1;
    }
    tmin = size[2] - std::abs(p[2]);
    if ( tmin < min )
    {
      min = tmin;
      idx = 2;
    }

    normal[idx] = (p[idx] > 0.0 ? 1.0 : -1.0);
    normal = tfB.linear() * normal;
    penetration = min + shapeA.getRadius();

    callback->notifyContact(&shapeA, &shapeB, c0, normal, penetration);
    return;
  }


  Eigen::Vector3d contactpt = tfB * p;
  normal = c0 - contactpt;
  double mag = normal.norm();
  penetration = shapeA.getRadius() - mag;

  if (penetration < 0.0)
  {
    return;
  }

  if (mag > DART_COLLISION_EPS)
  {
    normal *= (1.0/mag);

    callback->notifyContact(&shapeA, &shapeB, contactpt, normal, penetration);
  }
  else
  {
    double min = size[0] - std::abs(p[0]);
    double tmin = size[1] - std::abs(p[1]);
    int idx = 0;

    if ( tmin < min )
    {
      min = tmin;
      idx = 1;
    }
    tmin = size[2] - std::abs(p[2]);
    if ( tmin < min )
    {
      min = tmin;
      idx = 2;
    }
    normal.setZero();
    normal[idx] = (p[idx] > 0.0 ? 1.0 : -1.0);
    normal = tfB.linear() * normal;

    callback->notifyContact(&shapeA, &shapeB, contactpt, normal, penetration);
  }
}

//==============================================================================
void	collideBoxSphere(const dynamics::BoxShape& shapeA,
                       const Eigen::Isometry3d& tfA,
                       const dynamics::SphereShape& shapeB,
                       const Eigen::Isometry3d& tfB,
                       NarrowPhaseCallback* callback)
{
  Eigen::Vector3d halfSize = 0.5 * shapeA.getSize();
  bool inside_box = true;

  // clipping a center of the sphere to a boundary of the box
  //Vec3 c0(&tfA[9]);
  Eigen::Vector3d c0 = tfB.translation();
  Eigen::Vector3d p = tfA.inverse() * c0;

  if (p[0] < -halfSize[0]) { p[0] = -halfSize[0]; inside_box = false; }
  if (p[0] >  halfSize[0]) { p[0] =  halfSize[0]; inside_box = false; }

  if (p[1] < -halfSize[1]) { p[1] = -halfSize[1]; inside_box = false; }
  if (p[1] >  halfSize[1]) { p[1] =  halfSize[1]; inside_box = false; }

  if (p[2] < -halfSize[2]) { p[2] = -halfSize[2]; inside_box = false; }
  if (p[2] >  halfSize[2]) { p[2] =  halfSize[2]; inside_box = false; }


  Eigen::Vector3d normal(0.0, 0.0, 0.0);
  double penetration;

  if ( inside_box )
  {
    // find nearest side from the sphere center
    double min = halfSize[0] - std::abs(p[0]);
    double tmin = halfSize[1] - std::abs(p[1]);
    int idx = 0;

    if ( tmin < min )
    {
      min = tmin;
      idx = 1;
    }
    tmin = halfSize[2] - std::abs(p[2]);
    if ( tmin < min )
    {
      min = tmin;
      idx = 2;
    }

    //normal[idx] = (p[idx] > 0.0 ? 1.0 : -1.0);
    normal[idx] = (p[idx] > 0.0 ? -1.0 : 1.0);
    normal = tfA.linear() * normal;
    penetration = min + shapeB.getRadius();

    callback->notifyContact(&shapeA, &shapeB, c0, normal, penetration);
    return;
  }

  Eigen::Vector3d contactpt = tfA * p;
  //normal = c0 - contactpt;
  normal = contactpt - c0;
  double mag = normal.norm();
  penetration = shapeB.getRadius() - mag;

  if (penetration < 0.0)
  {
    return;
  }

  if (mag > DART_COLLISION_EPS)
  {
    normal *= (1.0/mag);

    callback->notifyContact(&shapeA, &shapeB, contactpt, normal, penetration);
  }
  else
  {
    double min = halfSize[0] - std::abs(p[0]);
    double tmin = halfSize[1] - std::abs(p[1]);
    int idx = 0;

    if ( tmin < min )
    {
      min = tmin;
      idx = 1;
    }
    tmin = halfSize[2] - std::abs(p[2]);
    if ( tmin < min )
    {
      min = tmin;
      idx = 2;
    }
    normal.setZero();
    //normal[idx] = (p[idx] > 0.0 ? 1.0 : -1.0);
    normal[idx] = (p[idx] > 0.0 ? -1.0 : 1.0);
    normal = tfA.linear() * normal;

    callback->notifyContact(&shapeA, &shapeB, contactpt, normal, penetration);
  }
}

} // namespace collision
} // namespace dart
