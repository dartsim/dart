/*
 * Copyright (c) 2014-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2014-2016, Humanoid Lab, Georgia Tech Research Corporation
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

// Must be included before any Bullet headers.
#include "dart/config.hpp"

#include "dart/collision/bullet/BulletTypes.hpp"

namespace dart {
namespace collision {

//==============================================================================
Eigen::Vector3d convertVector3(const btVector3& _vec)
{
  return Eigen::Vector3d(_vec.x(), _vec.y(), _vec.z());
}

//==============================================================================
btVector3 convertVector3(const Eigen::Vector3d& _vec)
{
  return btVector3(_vec.x(), _vec.y(), _vec.z());
}

//==============================================================================
btMatrix3x3 convertMatrix3x3(const Eigen::Matrix3d& _R)
{
  return btMatrix3x3(_R(0, 0), _R(0, 1), _R(0, 2),
                     _R(1, 0), _R(1, 1), _R(1, 2),
                     _R(2, 0), _R(2, 1), _R(2, 2));
}

//==============================================================================
btTransform convertTransform(const Eigen::Isometry3d& _T)
{
  btTransform trans;
  trans.setOrigin(convertVector3(_T.translation()));
  trans.setBasis(convertMatrix3x3(_T.linear()));
  return trans;
}

}  // namespace collision
}  // namespace dart
