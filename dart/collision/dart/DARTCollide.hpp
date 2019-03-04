/*
 * Copyright (c) 2011-2019, The DART development contributors
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

#ifndef DART_COLLISION_DART_DARTCOLLIDE_HPP_
#define DART_COLLISION_DART_DARTCOLLIDE_HPP_

#include <vector>
#include <Eigen/Dense>
#include "dart/collision/CollisionDetector.hpp"

namespace dart {
namespace collision {

int collide(CollisionObject* o1, CollisionObject* o2,
            CollisionResult& result);

int collideBoxBox(CollisionObject* o1, CollisionObject* o2,
                  const Eigen::Vector3d& size0, const Eigen::Isometry3d& T0,
                  const Eigen::Vector3d& size1, const Eigen::Isometry3d& T1,
                  CollisionResult& result);

int collideBoxSphere(CollisionObject* o1, CollisionObject* o2,
                     const Eigen::Vector3d& size0, const Eigen::Isometry3d& T0,
                     const double& r1, const Eigen::Isometry3d& T1,
                     CollisionResult& result);

int collideSphereBox(CollisionObject* o1, CollisionObject* o2,
                     const double& r0, const Eigen::Isometry3d& T0,
                     const Eigen::Vector3d& size1, const Eigen::Isometry3d& T1,
                     CollisionResult& result);

int collideSphereSphere(CollisionObject* o1, CollisionObject* o2,
                        const double& r0, const Eigen::Isometry3d& c0,
                        const double& r1, const Eigen::Isometry3d& c1,
                        CollisionResult& result);

int collideCylinderSphere(
    CollisionObject* o1, CollisionObject* o2,
    const double& cyl_rad, const double& half_height,
    const Eigen::Isometry3d& T0,
    const double& sphere_rad, const Eigen::Isometry3d& T1,
    CollisionResult& result);

int collideCylinderPlane(
    CollisionObject* o1, CollisionObject* o2,
    const double& cyl_rad, const double& half_height,
    const Eigen::Isometry3d& T0,
    const Eigen::Vector3d& plane_normal, const Eigen::Isometry3d& T1,
    CollisionResult& result);

}  // namespace collision
}  // namespace dart

#endif  // DART_COLLISION_DART_DARTCOLLIDE_HPP_
