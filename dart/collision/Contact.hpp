/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#ifndef DART_COLLISION_CONTACT_HPP_
#define DART_COLLISION_CONTACT_HPP_

#include <dart/collision/SmartPointer.hpp>

#include <dart/dynamics/SmartPointer.hpp>

#include <Eigen/Dense>

namespace dart {
namespace collision {

/// Contact information
struct Contact
{
  /// Default constructor
  Contact();

  /// Contact point w.r.t. the world frame
  Eigen::Vector3d point;

  /// Contact normal vector from bodyNode2 to bodyNode1 w.r.t. the world frame
  Eigen::Vector3d normal;

  /// Contact force acting on bodyNode1 w.r.t. the world frame
  ///
  /// The contact force acting on bodyNode2 is -force, which is the opposite
  /// direction of the force.
  Eigen::Vector3d force;

  /// First colliding collision object
  CollisionObject* collisionObject1;

  /// Second colliding collision object
  CollisionObject* collisionObject2;

  /// Penetration depth
  double penetrationDepth;

  // TODO(JS): triID1 will be deprecated when we don't use fcl_mesh
  /// \brief
  int triID1;

  // TODO(JS): triID2 will be deprecated when we don't use fcl_mesh
  /// \brief
  int triID2;

  // TODO(JS): userData is an experimental variable.
  /// \brief User data.
  void* userData;

  /// Returns the epsilon to be used for determination of zero-length normal.
  constexpr static double getNormalEpsilon();

  /// Returns the squired epsilon to be used for determination of zero-length
  /// normal.
  constexpr static double getNormalEpsilonSquared();

  /// Returns true if the length of a normal is less than the epsilon.
  static bool isZeroNormal(const Eigen::Vector3d& normal);

  /// Returns !isZeroNormal().
  static bool isNonZeroNormal(const Eigen::Vector3d& normal);
};

} // namespace collision
} // namespace dart

#include <dart/collision/detail/Contact-impl.hpp>

#endif // DART_COLLISION_CONTACT_HPP_
