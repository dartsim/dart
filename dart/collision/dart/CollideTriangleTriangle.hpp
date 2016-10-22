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

#ifndef DART_COLLISION_DART_COLLIDETRIANGLETRIANGLE_HPP_
#define DART_COLLISION_DART_COLLIDETRIANGLETRIANGLE_HPP_

#include <Eigen/Dense>

#include "dart/collision/dart/NarrowPhaseAlgorithms.hpp"

namespace dart {
namespace collision {

namespace v1 {

int collideTriangleTriangle(
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& a2,
    const Eigen::Vector3d& a3,
    const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2,
    const Eigen::Vector3d& b3,
    Eigen::Vector3d& contact1,
    Eigen::Vector3d& contact2,
    Eigen::Vector3d& contact3);

} // namespace v1

namespace v2 {

int collideTriangleTriangle(
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& a2,
    const Eigen::Vector3d& a3,
    const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2,
    const Eigen::Vector3d& b3,
    Eigen::Vector3d& contact1,
    Eigen::Vector3d& contact2,
    Eigen::Vector3d& contact3);

} // namespace v2

int test2dPointInTriangle(
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2,
    const Eigen::Vector3d& b3,
    Eigen::Vector3d& contact1) {}

int test2dLineSegmentTriangle(
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& a2,
    const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2,
    const Eigen::Vector3d& b3,
    Eigen::Vector3d& contact1,
    Eigen::Vector3d& contact2) {}

inline int case1(
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2,
    const Eigen::Vector3d& b3,
    Eigen::Vector3d& contact1);

inline int case2(
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& a2,
    const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2,
    const Eigen::Vector3d& b3,
    Eigen::Vector3d& contact1,
    Eigen::Vector3d& contact2);

inline int case3(
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& a2,
    const Eigen::Vector3d& a3,
    const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2,
    const Eigen::Vector3d& b3,
    Eigen::Vector3d& contact1,
    Eigen::Vector3d& contact2);

inline int case4(
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& a2,
    const Eigen::Vector3d& a3,
    const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2,
    const Eigen::Vector3d& b3,
    Eigen::Vector3d& contact1,
    Eigen::Vector3d& contact2);

int probablyCase2(
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& a2,
    const Eigen::Vector3d& a3,
    const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2,
    const Eigen::Vector3d& b3,
    Eigen::Vector3d& contact1,
    Eigen::Vector3d& contact2,
    double db1, double db2, double db3);

int probablyCase3(
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& a2,
    const Eigen::Vector3d& a3,
    const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2,
    const Eigen::Vector3d& b3,
    Eigen::Vector3d& contact1,
    Eigen::Vector3d& contact2,
    double db1, double db2, double db3);

int probablyCase4(
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& a2,
    const Eigen::Vector3d& a3,
    const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2,
    const Eigen::Vector3d& b3,
    Eigen::Vector3d& contact1,
    Eigen::Vector3d& contact2,
    double db1, double db2, double db3);

int case1And(const Eigen::Vector3d& a1,
    const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2,
    const Eigen::Vector3d& b3,
    const Eigen::Vector3d& n1,
    double db1, double db2, double db3,
    Eigen::Vector3d& contact1);

int case2And(
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& a2,
    const Eigen::Vector3d& a3,
    const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2,
    const Eigen::Vector3d& b3,
    Eigen::Vector3d& contact1,
    Eigen::Vector3d& contact2,
    double db1, double db2, double db3) {}

int case3And(
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& a2,
    const Eigen::Vector3d& a3,
    const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2,
    const Eigen::Vector3d& b3,
    Eigen::Vector3d& contact1,
    Eigen::Vector3d& contact2,
    double db1, double db2, double db3) {}

int case4And(
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& a2,
    const Eigen::Vector3d& a3,
    const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2,
    const Eigen::Vector3d& b3,
    Eigen::Vector3d& contact1,
    Eigen::Vector3d& contact2,
    double db1, double db2, double db3) {}

int case1AndCase1(
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& b1,
    Eigen::Vector3d& contact1);

int case1AndCase2(
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2,
    Eigen::Vector3d& contact1);

int case1AndCase3(
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2,
    const Eigen::Vector3d& b3,
    const Eigen::Vector3d& n1,
    double db2,
    Eigen::Vector3d& contact1);

int case1AndCase4(
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2,
    const Eigen::Vector3d& b3,
    const Eigen::Vector3d& n1,
    double db1,
    Eigen::Vector3d& contact1);

int case2AndCase2(
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& a2,
    const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2,
    Eigen::Vector3d& contact1,
    Eigen::Vector3d& contact2);

int case2AndCase3(
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& a2,
    const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2,
    const Eigen::Vector3d& b3,
    const Eigen::Vector3d& n1,
    double db2,
    Eigen::Vector3d& contact1,
    Eigen::Vector3d& contact2);

int case2AndCase4(
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& a2,
    const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2,
    const Eigen::Vector3d& b3,
    const Eigen::Vector3d& n1,
    double db1,
    Eigen::Vector3d& contact1,
    Eigen::Vector3d& contact2);

int case3AndCase3(
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& a2,
    const Eigen::Vector3d& a3,
    const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2,
    const Eigen::Vector3d& b3,
    const Eigen::Vector3d& n1,
    const Eigen::Vector3d& n2,
    double da2,
    double db2,
    Eigen::Vector3d& contact1,
    Eigen::Vector3d& contact2);

int case3AndCase4(const Eigen::Vector3d& a1,
    const Eigen::Vector3d& a2,
    const Eigen::Vector3d& a3,
    const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2,
    const Eigen::Vector3d& b3,
    const Eigen::Vector3d& n1,
    const Eigen::Vector3d& n2,
    double da2,
    double db1,
    Eigen::Vector3d& contact1,
    Eigen::Vector3d& contact2);

int case4AndCase4(
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& a2,
    const Eigen::Vector3d& a3,
    const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2,
    const Eigen::Vector3d& b3,
    const Eigen::Vector3d& n1,
    const Eigen::Vector3d& n2,
    double da2,
    double db2,
    Eigen::Vector3d& contact1,
    Eigen::Vector3d& contact2);

inline namespace stable {
  using v1::collideTriangleTriangle;
} // namespace stable

}  // namespace collision
}  // namespace dart

#endif  // DART_COLLISION_DART_COLLIDETRIANGLETRIANGLE_HPP_
