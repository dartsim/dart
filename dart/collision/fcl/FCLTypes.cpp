/*
 * Copyright (c) 2011-2017, The DART development contributors
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

#include "dart/collision/fcl/FCLTypes.hpp"

namespace dart {
namespace collision {

#if FCL_VERSION_AT_LEAST(0,6,0)
//==============================================================================
Eigen::Vector3d FCLTypes::convertVector3(const dart::collision::fcl::Vector3& _vec)
{
  return _vec;
}

//==============================================================================
dart::collision::fcl::Matrix3 FCLTypes::convertMatrix3x3(const Eigen::Matrix3d& _R)
{
  return _R;
}

//==============================================================================
dart::collision::fcl::Transform3 FCLTypes::convertTransform(const Eigen::Isometry3d& _T)
{
  return _T;
}

#else
//==============================================================================
Eigen::Vector3d FCLTypes::convertVector3(const dart::collision::fcl::Vector3& _vec)
{
  return Eigen::Vector3d(_vec[0], _vec[1], _vec[2]);
}

//==============================================================================
dart::collision::fcl::Vector3 FCLTypes::convertVector3(const Eigen::Vector3d& _vec)
{
  return dart::collision::fcl::Vector3(_vec[0], _vec[1], _vec[2]);
}

//==============================================================================
dart::collision::fcl::Matrix3 FCLTypes::convertMatrix3x3(const Eigen::Matrix3d& _R)
{
  return dart::collision::fcl::Matrix3(_R(0, 0), _R(0, 1), _R(0, 2),
                            _R(1, 0), _R(1, 1), _R(1, 2),
                            _R(2, 0), _R(2, 1), _R(2, 2));
}

//==============================================================================
dart::collision::fcl::Transform3 FCLTypes::convertTransform(const Eigen::Isometry3d& _T)
{
  dart::collision::fcl::Transform3 trans;

  trans.setTranslation(convertVector3(_T.translation()));
  trans.setRotation(convertMatrix3x3(_T.linear()));

  return trans;
}
#endif
}  // namespace collision
}  // namespace dart
