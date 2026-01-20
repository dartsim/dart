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

#include <dart/simulation/world.hpp>

#include <dart/collision/collision_object.hpp>
#include <dart/collision/collision_result.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <dart/io/read.hpp>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <utility>
#include <vector>

#include <cmath>

using namespace dart;
using namespace collision;
using namespace dynamics;
using namespace simulation;

namespace {

bool isContactApproxEqual(
    const Contact& lhs, const Contact& rhs, double tol = 1e-12)
{
  if (!lhs.point.isApprox(rhs.point, tol))
    return false;
  if (!lhs.normal.isApprox(rhs.normal, tol))
    return false;
  if (!lhs.force.isApprox(rhs.force, tol))
    return false;
  if (std::abs(lhs.penetrationDepth - rhs.penetrationDepth) > tol)
    return false;

  const auto* lhsShape1
      = lhs.collisionObject1 ? lhs.collisionObject1->getShapeFrame() : nullptr;
  const auto* lhsShape2
      = lhs.collisionObject2 ? lhs.collisionObject2->getShapeFrame() : nullptr;
  const auto* rhsShape1
      = rhs.collisionObject1 ? rhs.collisionObject1->getShapeFrame() : nullptr;
  const auto* rhsShape2
      = rhs.collisionObject2 ? rhs.collisionObject2->getShapeFrame() : nullptr;

  const auto lhsPair = std::make_pair(
      lhsShape1 ? lhsShape1->getName() : std::string(),
      lhsShape2 ? lhsShape2->getName() : std::string());
  const auto rhsPair = std::make_pair(
      rhsShape1 ? rhsShape1->getName() : std::string(),
      rhsShape2 ? rhsShape2->getName() : std::string());

  return lhsPair == rhsPair;
}

std::vector<std::size_t> getJointIndices(const SkeletonPtr& skel)
{
  const std::array<const char*, 6> names = {{
      "j_scapula_left",
      "j_scapula_right",
      "j_forearm_left",
      "j_forearm_right",
      "j_shin_left",
      "j_shin_right",
  }};
  std::vector<std::size_t> indices;
  indices.reserve(names.size());
  for (const auto& name : names) {
    const auto* joint = skel->getJoint(name);
    if (joint)
      indices.push_back(joint->getIndexInSkeleton(0));
  }
  return indices;
}

void applyVelocityCommands(
    const WorldPtr& world,
    const SkeletonPtr& skel,
    const std::vector<std::size_t>& indices)
{
  const double t = world->getTime();

  skel->setCommand(indices[0], std::sin(t * 4.0));
  skel->setCommand(indices[1], -std::sin(t * 4.0));
  skel->setCommand(indices[2], 0.8 * std::sin(t * 4.0));
  skel->setCommand(indices[3], 0.8 * std::sin(t * 4.0));
  skel->setCommand(indices[4], 0.1 * std::sin(t * 2.0));
  skel->setCommand(indices[5], 0.1 * std::sin(t * 2.0));
}

} // namespace

// Verifies that cloned worlds run deterministically when given identical
// velocity commands, covering the original #410 report.
TEST(Simulation, DISABLED_ClonedWorldsStayDeterministic)
{
  auto baseWorld = dart::io::readWorld("dart://sample/skel/fullbody1.skel");
  ASSERT_NE(baseWorld, nullptr);
  baseWorld->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));
  baseWorld->setTimeStep(1.0 / 240.0);

  auto skel = baseWorld->getSkeleton(1);
  ASSERT_NE(skel, nullptr);
  const auto indices = getJointIndices(skel);
  ASSERT_EQ(indices.size(), 6u);

  const int worldCount = 3;
  std::vector<WorldPtr> worlds;
  worlds.reserve(worldCount);
  worlds.push_back(baseWorld);
  for (int i = 1; i < worldCount; ++i) {
    auto clone = baseWorld->clone();
    ASSERT_NE(clone, nullptr);
    worlds.push_back(clone);
  }

  const double tol = 1e-12;
  const int steps = 240;
  for (int i = 0; i < steps; ++i) {
    for (auto& world : worlds) {
      auto worldSkel = world->getSkeleton(1);
      ASSERT_NE(worldSkel, nullptr);
      applyVelocityCommands(world, worldSkel, indices);
      world->step();
    }

    const auto refWorld = worlds.front();
    const auto refSkel = refWorld->getSkeleton(1);
    const auto refPos = refSkel->getPositions();
    const auto refVel = refSkel->getVelocities();
    const auto& refContacts = refWorld->getLastCollisionResult().getContacts();

    for (std::size_t w = 1; w < worlds.size(); ++w) {
      const auto otherSkel = worlds[w]->getSkeleton(1);
      EXPECT_TRUE(refPos.isApprox(otherSkel->getPositions(), tol));
      EXPECT_TRUE(refVel.isApprox(otherSkel->getVelocities(), tol));
      EXPECT_DOUBLE_EQ(refWorld->getTime(), worlds[w]->getTime());

      const auto& contacts = worlds[w]->getLastCollisionResult().getContacts();
      ASSERT_EQ(refContacts.size(), contacts.size());
      for (std::size_t c = 0; c < contacts.size(); ++c) {
        EXPECT_TRUE(isContactApproxEqual(refContacts[c], contacts[c], tol))
            << "Contact mismatch at index " << c << " for world " << w;
      }
    }
  }
}

// Ensure contact points produced under a large time step still report
// meaningful forces instead of zero-force contacts.
TEST(Simulation, DISABLED_ContactsReportNonZeroForceWithLargeTimeStep)
{
  auto world = World::create("issue410_contacts");
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world->setTimeStep(1.0 / 30.0);

  // Ground with thickness to avoid tunneling at the large time step.
  auto ground = Skeleton::create("ground");
  auto groundPair = ground->createJointAndBodyNodePair<WeldJoint>(nullptr);
  auto groundShape
      = std::make_shared<BoxShape>(Eigen::Vector3d(10.0, 10.0, 0.2));
  groundPair.second->createShapeNodeWith<CollisionAspect, DynamicsAspect>(
      groundShape);
  Eigen::Isometry3d groundTf = Eigen::Isometry3d::Identity();
  groundTf.translation() = Eigen::Vector3d(0.0, 0.0, -0.1);
  groundPair.first->setTransformFromParentBodyNode(groundTf);
  world->addSkeleton(ground);

  // Falling box.
  auto box = Skeleton::create("box");
  auto boxPair = box->createJointAndBodyNodePair<FreeJoint>(
      nullptr, FreeJoint::Properties(), BodyNode::Properties());
  boxPair.second->setName("box_link");
  auto boxShape = std::make_shared<BoxShape>(Eigen::Vector3d::Constant(0.3));
  boxPair.second->createShapeNodeWith<CollisionAspect, DynamicsAspect>(
      boxShape);
  boxPair.second->setMass(1.0);
  Eigen::Isometry3d boxTf = Eigen::Isometry3d::Identity();
  boxTf.translation() = Eigen::Vector3d(0.0, 0.0, 0.6);
  FreeJoint::setTransformOf(boxPair.first, boxTf);
  world->addSkeleton(box);

  bool sawContact = false;
  std::size_t maxContacts = 0u;
  const double minForce = 1e-8;

  const int steps = 300;
  for (int i = 0; i < steps; ++i) {
    world->step();
    const auto& contacts = world->getLastCollisionResult().getContacts();
    if (contacts.empty())
      continue;

    sawContact = true;
    maxContacts = std::max<std::size_t>(maxContacts, contacts.size());
    for (std::size_t c = 0; c < contacts.size(); ++c) {
      EXPECT_GT(contacts[c].force.norm(), minForce)
          << "Zero-force contact detected at step " << i << " contact " << c;
    }
  }

  EXPECT_TRUE(sawContact);
  EXPECT_GE(maxContacts, 2u);
}
