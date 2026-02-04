/*
 * Copyright (c) 2011, The DART development contributors
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

#include <dart/collision/collision_result.hpp>
#include <dart/collision/fcl/fcl_collision_detector.hpp>

#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/frame.hpp>
#include <dart/dynamics/simple_frame.hpp>

#include <Eigen/Core>
#include <cmath>
#include <gtest/gtest.h>

using dart::collision::FCLCollisionDetector;

// Reproduces the face-to-face mesh box contact from DART issue #860.
// The detector is intentionally configured to use FCL's contact computation,
// and the test asserts that DART still returns sensible contact points on the
// touching face.
TEST(CollisionRegression, MeshMeshContactPointsStayOnContactPlane)
{
  auto detector = FCLCollisionDetector::create();
  detector->setPrimitiveShapeType(FCLCollisionDetector::MESH);
  detector->setContactPointComputationMethod(FCLCollisionDetector::FCL);

  auto shape = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d::Ones());

  auto bottom = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World());
  auto top = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World());

  bottom->setShape(shape);
  top->setShape(shape);

  bottom->setTranslation(Eigen::Vector3d::Zero());
  top->setTranslation(Eigen::Vector3d(0.0, 0.0, 1.0));

  auto group = detector->createCollisionGroup(bottom.get(), top.get());

  dart::collision::CollisionOption option;
  option.enableContact = true;
  option.maxNumContacts = 20u;

  dart::collision::CollisionResult result;
  const bool collided = group->collide(option, &result);
  ASSERT_TRUE(collided);
  ASSERT_GT(result.getNumContacts(), 0u);

  const double half = 0.5;
  const double tol = 1e-3;

  for (std::size_t i = 0; i < result.getNumContacts(); ++i) {
    const auto& contact = result.getContact(i);

    EXPECT_NEAR(contact.point[2], half, tol) << "Contact index: " << i;
    EXPECT_LE(contact.point.head<2>().cwiseAbs().maxCoeff(), half + tol);

    EXPECT_NEAR(std::abs(contact.normal[2]), 1.0, 1e-6);
    EXPECT_LT(std::abs(contact.normal[0]) + std::abs(contact.normal[1]), 1e-3);
  }
}
