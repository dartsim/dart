/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
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

#ifndef DART_COLLISION_DART_DARTCOLLIDE_H_
#define DART_COLLISION_DART_DARTCOLLIDE_H_

#include <vector>

#include <Eigen/Dense>

#include "dart/collision/CollisionDetector.h"

namespace dart {
namespace dynamics {
class Shape;
}  // namespace dynamics
}  // namespace dart

namespace dart {
namespace collision {

int collide(const dynamics::Shape* _shape0, const Eigen::Isometry3d& _T0,
            const dynamics::Shape* _shape1, const Eigen::Isometry3d& _T1,
            std::vector<Contact>* _result);

int collideBoxBox(const Eigen::Vector3d& size0, const Eigen::Isometry3d& T0,
                  const Eigen::Vector3d& size1, const Eigen::Isometry3d& T1,
                  std::vector<Contact>* result);

int collideBoxSphere(const Eigen::Vector3d& size0, const Eigen::Isometry3d& T0,
                     const double& r1, const Eigen::Isometry3d& T1,
                     std::vector<Contact>* result);

int collideSphereBox(const double& r0, const Eigen::Isometry3d& T0,
                     const Eigen::Vector3d& size1, const Eigen::Isometry3d& T1,
                     std::vector<Contact>* result);

int collideSphereSphere(const double& _r0, const Eigen::Isometry3d& c0,
                        const double& _r1, const Eigen::Isometry3d& c1,
                        std::vector<Contact>* result);

int collideCylinderSphere(
    const double& cyl_rad, const double& half_height,
    const Eigen::Isometry3d& T0,
    const double& sphere_rad, const Eigen::Isometry3d& T1,
    std::vector<Contact>* result);

int collideCylinderPlane(
    const double& cyl_rad, const double& half_height,
    const Eigen::Isometry3d& T0,
    const Eigen::Vector3d& plane_normal, const Eigen::Isometry3d& T1,
    std::vector<Contact>* result);

}  // namespace collision
}  // namespace dart

#endif  // DART_COLLISION_DART_DARTCOLLIDE_H_
