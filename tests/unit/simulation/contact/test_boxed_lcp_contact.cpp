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

// Boxed-LCP rigid-body contact parity test (PLAN-080 WS4 / friction
// continuation). The opt-in BoxedLcp contact path must reproduce the
// SequentialImpulse path's resting behavior for a frictionless (normal-only)
// rigid-body scene: a body dropped onto a static ground comes to rest at the
// same height with a near-zero normal velocity and no deep penetration. A
// direct equal-mass head-on collision additionally checks momentum conservation
// against the closed-form result. Coulomb friction is exercised by a sliding
// box that decelerates consistently with the SequentialImpulse path and a small
// tangential push that static friction holds in place.

#include <dart/simulation/body/collision_shape.hpp>
#include <dart/simulation/body/rigid_body.hpp>
#include <dart/simulation/body/rigid_body_options.hpp>
#include <dart/simulation/common/exceptions.hpp>
#include <dart/simulation/comps/joint.hpp>
#include <dart/simulation/comps/name.hpp>
#include <dart/simulation/comps/rigid_body.hpp>
#include <dart/simulation/detail/deformable_vbd/rigid_world_contact.hpp>
#include <dart/simulation/detail/entity_conversion.hpp>
#include <dart/simulation/detail/world_registry_access.hpp>
#include <dart/simulation/multibody/joint.hpp>
#include <dart/simulation/multibody/link.hpp>
#include <dart/simulation/world.hpp>
#include <dart/simulation/world_options.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <optional>
#include <sstream>
#include <utility>
#include <vector>

#include <cmath>

namespace sx = dart::simulation;
namespace dvbd = dart::simulation::detail::deformable_vbd;

namespace {

//==============================================================================
// Build a frictionless sphere-on-static-ground drop scene with the requested
// contact solver method. Ground top face is at z = 0; the sphere (radius 0.5)
// starts above it so it should settle with its center near z = 0.5.
std::unique_ptr<sx::World> buildDropScene(
    sx::ContactSolverMethod method, double sphereHeight = 1.0)
{
  sx::WorldOptions options;
  options.timeStep = 0.005;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  options.contactSolverMethod = method;
  auto world = std::make_unique<sx::World>(options);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world->addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(5.0, 5.0, 0.5)));
  ground.setFriction(0.0);

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.position = Eigen::Vector3d(0.0, 0.0, sphereHeight);
  auto sphere = world->addRigidBody("sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
  sphere.setFriction(0.0);

  return world;
}

//==============================================================================
sx::Contact swapEndpointOrder(const sx::Contact& contact)
{
  sx::Contact swapped{
      contact.bodyB,
      contact.bodyA,
      contact.point,
      -contact.normal,
      contact.depth};
  swapped.shapeIndexA = contact.shapeIndexB;
  swapped.shapeIndexB = contact.shapeIndexA;
  swapped.localPointA = contact.localPointB;
  swapped.localPointB = contact.localPointA;
  return swapped;
}

//==============================================================================
void expectSphereSideRowsIgnoreContactOrder(
    const sx::CollisionShape& primitiveShape,
    std::uint64_t expectedFeatureLocalIndex)
{
  sx::World world;

  auto primitive = world.addRigidBody("primitive");
  primitive.setCollisionShape(primitiveShape);

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.position = Eigen::Vector3d(0.45, -0.10, -0.20);
  auto sphere = world.addRigidBody("sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.25));

  std::vector<sx::Contact> contacts;
  for (const Eigen::Vector3d& center :
       {Eigen::Vector3d(0.45, -0.10, -0.20),
        Eigen::Vector3d(0.45, 0.10, 0.20)}) {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = center;
    sphere.setTransform(pose);
    const std::vector<sx::Contact> emittedContacts = world.collide();
    ASSERT_FALSE(emittedContacts.empty());
    contacts.insert(
        contacts.end(), emittedContacts.begin(), emittedContacts.end());
  }
  ASSERT_GE(contacts.size(), 2u);

  const auto buildSnapshot = [&](std::vector<sx::Contact> orderedContacts) {
    return dvbd::buildAvbdRigidWorldContactSnapshot(
        dart::simulation::detail::registryOf(world),
        orderedContacts,
        dvbd::AvbdRigidWorldContactOptions{});
  };

  const dvbd::AvbdRigidWorldContactSnapshot forward = buildSnapshot(contacts);
  std::vector<sx::Contact> reversedContacts = contacts;
  std::reverse(reversedContacts.begin(), reversedContacts.end());
  const dvbd::AvbdRigidWorldContactSnapshot reversed
      = buildSnapshot(reversedContacts);

  ASSERT_EQ(forward.contacts.size(), contacts.size());
  ASSERT_EQ(reversed.contacts.size(), contacts.size());

  using RowKey
      = std::pair<dvbd::AvbdContactEndpointId, dvbd::AvbdContactEndpointId>;

  const auto rowKey = [](const dvbd::AvbdRigidContactManifoldPoint& contact) {
    return dvbd::canonicalizeAvbdContactEndpoints(
        contact.endpointA, contact.endpointB);
  };
  const RowKey expectedRowKey = rowKey(forward.contacts.front());
  const auto rowForPoint
      = [&](const Eigen::Vector3d& point) -> std::optional<std::uint32_t> {
    for (const dvbd::AvbdRigidContactManifoldPoint& contact :
         reversed.contacts) {
      if (rowKey(contact) == expectedRowKey
          && (contact.point - point).norm() <= 1e-10) {
        return contact.row;
      }
    }
    return std::nullopt;
  };

  const std::uint64_t primitiveObject = dvbd::avbdRigidWorldContactObjectId(
      sx::detail::toRegistryEntity(primitive.getEntity()));
  std::vector<std::uint32_t> rows;
  rows.reserve(forward.contacts.size());
  for (const dvbd::AvbdRigidContactManifoldPoint& contact : forward.contacts) {
    EXPECT_EQ(rowKey(contact), expectedRowKey);

    const dvbd::AvbdContactEndpointId primitiveEndpoint
        = contact.endpointA.object == primitiveObject ? contact.endpointA
                                                      : contact.endpointB;
    ASSERT_EQ(primitiveEndpoint.object, primitiveObject);
    EXPECT_EQ(
        dvbd::avbdContactFeatureKind(primitiveEndpoint.feature),
        dvbd::AvbdContactFeatureKind::Face);
    EXPECT_EQ(
        dvbd::avbdContactFeatureLocalIndex(primitiveEndpoint.feature),
        expectedFeatureLocalIndex);

    const std::optional<std::uint32_t> reversedRow = rowForPoint(contact.point);
    ASSERT_TRUE(reversedRow.has_value());
    EXPECT_EQ(contact.row, *reversedRow);
    rows.push_back(contact.row);
  }

  std::sort(rows.begin(), rows.end());
  rows.erase(std::unique(rows.begin(), rows.end()), rows.end());
  EXPECT_GE(rows.size(), 2u);
  for (std::size_t row = 0; row < rows.size(); ++row) {
    EXPECT_EQ(rows[row], row);
  }
}

//==============================================================================
void expectCylinderRimRowsIgnoreContactOrder(
    double sphereCenterZ, std::uint64_t cylinderFeatureCode)
{
  sx::World world;

  auto cylinder = world.addRigidBody("cylinder");
  cylinder.setCollisionShape(
      sx::CollisionShape::makeCylinder(/*radius=*/0.25, /*halfHeight=*/0.5));

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.position = Eigen::Vector3d(0.42, -0.07, sphereCenterZ);
  auto sphere = world.addRigidBody("sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.25));

  std::vector<sx::Contact> contacts;
  for (const Eigen::Vector3d& center :
       {Eigen::Vector3d(0.42, -0.07, sphereCenterZ),
        Eigen::Vector3d(0.42, 0.07, sphereCenterZ)}) {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = center;
    sphere.setTransform(pose);
    const std::vector<sx::Contact> emittedContacts = world.collide();
    ASSERT_FALSE(emittedContacts.empty());
    contacts.insert(
        contacts.end(), emittedContacts.begin(), emittedContacts.end());
  }
  ASSERT_GE(contacts.size(), 2u);

  const auto buildSnapshot = [&](std::vector<sx::Contact> orderedContacts) {
    return dvbd::buildAvbdRigidWorldContactSnapshot(
        dart::simulation::detail::registryOf(world),
        orderedContacts,
        dvbd::AvbdRigidWorldContactOptions{});
  };

  const dvbd::AvbdRigidWorldContactSnapshot forward = buildSnapshot(contacts);
  std::vector<sx::Contact> reversedContacts = contacts;
  std::reverse(reversedContacts.begin(), reversedContacts.end());
  const dvbd::AvbdRigidWorldContactSnapshot reversed
      = buildSnapshot(reversedContacts);

  ASSERT_EQ(forward.contacts.size(), contacts.size());
  ASSERT_EQ(reversed.contacts.size(), contacts.size());

  using RowKey
      = std::pair<dvbd::AvbdContactEndpointId, dvbd::AvbdContactEndpointId>;

  const auto rowKey = [](const dvbd::AvbdRigidContactManifoldPoint& contact) {
    return dvbd::canonicalizeAvbdContactEndpoints(
        contact.endpointA, contact.endpointB);
  };
  const RowKey expectedRowKey = rowKey(forward.contacts.front());
  const auto rowForPoint
      = [&](const Eigen::Vector3d& point) -> std::optional<std::uint32_t> {
    for (const dvbd::AvbdRigidContactManifoldPoint& contact :
         reversed.contacts) {
      if (rowKey(contact) == expectedRowKey
          && (contact.point - point).norm() <= 1e-10) {
        return contact.row;
      }
    }
    return std::nullopt;
  };

  const std::uint64_t cylinderObject = dvbd::avbdRigidWorldContactObjectId(
      sx::detail::toRegistryEntity(cylinder.getEntity()));
  const std::uint64_t expectedFeatureLocalIndex
      = dvbd::packAvbdCylinderContactFeatureId(0, cylinderFeatureCode);
  std::vector<std::uint32_t> rows;
  rows.reserve(forward.contacts.size());
  for (const dvbd::AvbdRigidContactManifoldPoint& contact : forward.contacts) {
    EXPECT_EQ(rowKey(contact), expectedRowKey);

    const dvbd::AvbdContactEndpointId cylinderEndpoint
        = contact.endpointA.object == cylinderObject ? contact.endpointA
                                                     : contact.endpointB;
    ASSERT_EQ(cylinderEndpoint.object, cylinderObject);
    EXPECT_EQ(
        dvbd::avbdContactFeatureKind(cylinderEndpoint.feature),
        dvbd::AvbdContactFeatureKind::Edge);
    EXPECT_EQ(
        dvbd::avbdContactFeatureLocalIndex(cylinderEndpoint.feature),
        expectedFeatureLocalIndex);

    const std::optional<std::uint32_t> reversedRow = rowForPoint(contact.point);
    ASSERT_TRUE(reversedRow.has_value());
    EXPECT_EQ(contact.row, *reversedRow);
    rows.push_back(contact.row);
  }

  std::sort(rows.begin(), rows.end());
  rows.erase(std::unique(rows.begin(), rows.end()), rows.end());
  EXPECT_GE(rows.size(), 2u);
  for (std::size_t row = 0; row < rows.size(); ++row) {
    EXPECT_EQ(rows[row], row);
  }
}

//==============================================================================
void expectPrimitiveCapRowsIgnoreContactOrder(
    const sx::CollisionShape& primitiveShape,
    const Eigen::Vector3d& planeNormal,
    double primitiveCenterZ,
    std::uint64_t expectedFeatureLocalIndex)
{
  sx::World world;

  sx::RigidBodyOptions planeOptions;
  planeOptions.isStatic = true;
  auto plane = world.addRigidBody("plane", planeOptions);
  plane.setCollisionShape(
      sx::CollisionShape::makePlane(planeNormal.normalized(), 0.0));

  sx::RigidBodyOptions primitiveOptions;
  primitiveOptions.position = Eigen::Vector3d(-0.25, 0.0, primitiveCenterZ);
  auto primitive = world.addRigidBody("primitive", primitiveOptions);
  primitive.setCollisionShape(primitiveShape);

  std::vector<sx::Contact> contacts;
  for (const Eigen::Vector3d& center :
       {Eigen::Vector3d(-0.25, 0.0, primitiveCenterZ),
        Eigen::Vector3d(0.25, 0.0, primitiveCenterZ)}) {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = center;
    primitive.setTransform(pose);
    const std::vector<sx::Contact> emittedContacts = world.collide();
    ASSERT_FALSE(emittedContacts.empty());
    contacts.insert(
        contacts.end(), emittedContacts.begin(), emittedContacts.end());
  }
  ASSERT_GE(contacts.size(), 2u);

  const auto buildSnapshot = [&](std::vector<sx::Contact> orderedContacts) {
    return dvbd::buildAvbdRigidWorldContactSnapshot(
        dart::simulation::detail::registryOf(world),
        orderedContacts,
        dvbd::AvbdRigidWorldContactOptions{});
  };

  const dvbd::AvbdRigidWorldContactSnapshot forward = buildSnapshot(contacts);
  std::vector<sx::Contact> reversedContacts = contacts;
  std::reverse(reversedContacts.begin(), reversedContacts.end());
  const dvbd::AvbdRigidWorldContactSnapshot reversed
      = buildSnapshot(reversedContacts);

  ASSERT_EQ(forward.contacts.size(), contacts.size());
  ASSERT_EQ(reversed.contacts.size(), contacts.size());

  using RowKey
      = std::pair<dvbd::AvbdContactEndpointId, dvbd::AvbdContactEndpointId>;

  const auto rowKey = [](const dvbd::AvbdRigidContactManifoldPoint& contact) {
    return dvbd::canonicalizeAvbdContactEndpoints(
        contact.endpointA, contact.endpointB);
  };
  const RowKey expectedRowKey = rowKey(forward.contacts.front());
  const auto rowForPoint
      = [&](const Eigen::Vector3d& point) -> std::optional<std::uint32_t> {
    for (const dvbd::AvbdRigidContactManifoldPoint& contact :
         reversed.contacts) {
      if (rowKey(contact) == expectedRowKey
          && (contact.point - point).norm() <= 1e-10) {
        return contact.row;
      }
    }
    return std::nullopt;
  };

  const std::uint64_t primitiveObject = dvbd::avbdRigidWorldContactObjectId(
      sx::detail::toRegistryEntity(primitive.getEntity()));
  std::vector<std::uint32_t> rows;
  rows.reserve(forward.contacts.size());
  for (const dvbd::AvbdRigidContactManifoldPoint& contact : forward.contacts) {
    EXPECT_EQ(rowKey(contact), expectedRowKey);

    const dvbd::AvbdContactEndpointId primitiveEndpoint
        = contact.endpointA.object == primitiveObject ? contact.endpointA
                                                      : contact.endpointB;
    ASSERT_EQ(primitiveEndpoint.object, primitiveObject);
    EXPECT_EQ(
        dvbd::avbdContactFeatureKind(primitiveEndpoint.feature),
        dvbd::AvbdContactFeatureKind::Face);
    EXPECT_EQ(
        dvbd::avbdContactFeatureLocalIndex(primitiveEndpoint.feature),
        expectedFeatureLocalIndex);

    const std::optional<std::uint32_t> reversedRow = rowForPoint(contact.point);
    ASSERT_TRUE(reversedRow.has_value());
    EXPECT_EQ(contact.row, *reversedRow);
    rows.push_back(contact.row);
  }

  std::sort(rows.begin(), rows.end());
  rows.erase(std::unique(rows.begin(), rows.end()), rows.end());
  EXPECT_GE(rows.size(), 2u);
  for (std::size_t row = 0; row < rows.size(); ++row) {
    EXPECT_EQ(rows[row], row);
  }
}

//==============================================================================
void expectSpherePrimitiveRowsPersistAcrossSmallPose(
    const sx::CollisionShape& primitiveShape,
    const std::vector<Eigen::Vector3d>& referenceSphereCenters,
    const std::vector<Eigen::Vector3d>& nudgedSphereCenters,
    dvbd::AvbdContactFeatureKind expectedFeatureKind,
    std::uint64_t expectedFeatureLocalIndex)
{
  ASSERT_GE(referenceSphereCenters.size(), 2u);
  ASSERT_EQ(referenceSphereCenters.size(), nudgedSphereCenters.size());

  sx::World world;

  auto primitive = world.addRigidBody("primitive");
  primitive.setCollisionShape(primitiveShape);
  primitive.setFriction(0.64);

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.position = referenceSphereCenters.front();
  auto sphere = world.addRigidBody("sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.25));
  sphere.setFriction(0.25);

  const auto buildSnapshot
      = [&](const std::vector<Eigen::Vector3d>& centers,
            dvbd::AvbdRigidWorldContactSnapshot& snapshot) {
          std::vector<sx::Contact> contacts;
          for (const Eigen::Vector3d& center : centers) {
            Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
            pose.translation() = center;
            sphere.setTransform(pose);
            const std::vector<sx::Contact> emittedContacts = world.collide();
            ASSERT_FALSE(emittedContacts.empty());
            contacts.insert(
                contacts.end(), emittedContacts.begin(), emittedContacts.end());
          }
          ASSERT_GE(contacts.size(), centers.size());

          snapshot = dvbd::buildAvbdRigidWorldContactSnapshot(
              dart::simulation::detail::registryOf(world),
              contacts,
              dvbd::AvbdRigidWorldContactOptions{});
          ASSERT_EQ(snapshot.contacts.size(), contacts.size());
        };

  dvbd::AvbdRigidWorldContactSnapshot reference;
  dvbd::AvbdRigidWorldContactSnapshot nudged;
  buildSnapshot(referenceSphereCenters, reference);
  buildSnapshot(nudgedSphereCenters, nudged);
  ASSERT_EQ(reference.contacts.size(), nudged.contacts.size());

  using RowKey
      = std::pair<dvbd::AvbdContactEndpointId, dvbd::AvbdContactEndpointId>;
  struct ObservedRow
  {
    RowKey key;
    std::uint32_t row = 0u;
    double frictionCoefficient = 0.0;
  };

  const auto rowKey = [](const dvbd::AvbdRigidContactManifoldPoint& contact) {
    return dvbd::canonicalizeAvbdContactEndpoints(
        contact.endpointA, contact.endpointB);
  };
  const std::uint64_t primitiveObject = dvbd::avbdRigidWorldContactObjectId(
      sx::detail::toRegistryEntity(primitive.getEntity()));
  const auto collectRows
      = [&](const dvbd::AvbdRigidWorldContactSnapshot& snapshot) {
          std::vector<ObservedRow> rows;
          rows.reserve(snapshot.contacts.size());
          for (const dvbd::AvbdRigidContactManifoldPoint& contact :
               snapshot.contacts) {
            const dvbd::AvbdContactEndpointId primitiveEndpoint
                = contact.endpointA.object == primitiveObject
                      ? contact.endpointA
                      : contact.endpointB;
            EXPECT_EQ(primitiveEndpoint.object, primitiveObject);
            if (primitiveEndpoint.object != primitiveObject) {
              continue;
            }
            EXPECT_EQ(
                dvbd::avbdContactFeatureKind(primitiveEndpoint.feature),
                expectedFeatureKind);
            EXPECT_EQ(
                dvbd::avbdContactFeatureLocalIndex(primitiveEndpoint.feature),
                expectedFeatureLocalIndex);
            EXPECT_NEAR(contact.frictionCoefficient, 0.4, 1e-12);
            rows.push_back(
                ObservedRow{
                    rowKey(contact), contact.row, contact.frictionCoefficient});
          }
          return rows;
        };
  const auto observedAtRow
      = [](const std::vector<ObservedRow>& rows,
           const RowKey& key,
           std::uint32_t row) -> std::optional<ObservedRow> {
    for (const ObservedRow& observed : rows) {
      if (observed.key == key && observed.row == row) {
        return observed;
      }
    }
    return std::nullopt;
  };

  const std::vector<ObservedRow> referenceRows = collectRows(reference);
  const std::vector<ObservedRow> nudgedRows = collectRows(nudged);
  ASSERT_EQ(referenceRows.size(), reference.contacts.size());
  ASSERT_EQ(nudgedRows.size(), nudged.contacts.size());

  std::vector<std::uint32_t> rows;
  rows.reserve(referenceRows.size());
  for (const ObservedRow& referenceRow : referenceRows) {
    const std::optional<ObservedRow> nudgedRow
        = observedAtRow(nudgedRows, referenceRow.key, referenceRow.row);
    ASSERT_TRUE(nudgedRow.has_value());
    EXPECT_NEAR(
        referenceRow.frictionCoefficient,
        nudgedRow->frictionCoefficient,
        1e-12);
    rows.push_back(referenceRow.row);
  }

  std::sort(rows.begin(), rows.end());
  rows.erase(std::unique(rows.begin(), rows.end()), rows.end());
  EXPECT_GE(rows.size(), 2u);
  for (std::size_t row = 0; row < rows.size(); ++row) {
    EXPECT_EQ(rows[row], row);
  }

  dvbd::AvbdScalarRowInventory normalInventory;
  dvbd::AvbdScalarRowInventory frictionInventory;
  std::vector<dvbd::AvbdRigidBodyPointPairRow> normalRows;
  std::vector<dvbd::AvbdRigidBodyPointPairFrictionRows> frictionRows;
  dvbd::AvbdRowWarmStartOptions warmStart;
  warmStart.alpha = 1.0;
  warmStart.gamma = 1.0;

  dvbd::buildAvbdRigidContactManifoldRows(
      reference.states,
      reference.contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      warmStart);

  ASSERT_EQ(normalInventory.size(), reference.contacts.size());
  ASSERT_EQ(frictionInventory.size(), 2u * reference.contacts.size());
  ASSERT_EQ(frictionRows.size(), reference.contacts.size());

  const auto keyFingerprint = [](const dvbd::AvbdScalarRowKey& key) {
    return static_cast<double>(
        key.objectA % 17u + 2u * (key.objectB % 19u) + 3u * (key.featureA % 23u)
        + 5u * (key.featureB % 29u));
  };
  const auto expectedNormalLambda = [&](const dvbd::AvbdScalarRowKey& key) {
    return 19.0 + static_cast<double>(key.row) + 0.01 * keyFingerprint(key);
  };
  const auto expectedFrictionLambda = [&](const dvbd::AvbdScalarRowKey& key) {
    return 5.0 + 0.5 * static_cast<double>(key.row)
           + 0.125 * static_cast<double>(key.axis) + 0.01 * keyFingerprint(key);
  };

  for (dvbd::AvbdScalarRowRecord& record : normalInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::ContactNormal);
    record.state.lambda = expectedNormalLambda(record.descriptor.key);
  }
  for (dvbd::AvbdScalarRowRecord& record : frictionInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::FrictionTangent);
    record.state.lambda = expectedFrictionLambda(record.descriptor.key);
  }

  struct PreviousFrictionPair
  {
    dvbd::AvbdScalarRowKey key;
    Eigen::Vector3d worldDual = Eigen::Vector3d::Zero();
  };
  const auto sameFrictionPair = [](const dvbd::AvbdScalarRowKey& lhs,
                                   const dvbd::AvbdScalarRowKey& rhs) {
    return lhs.role == rhs.role && lhs.objectA == rhs.objectA
           && lhs.objectB == rhs.objectB && lhs.featureA == rhs.featureA
           && lhs.featureB == rhs.featureB && lhs.row == rhs.row;
  };
  std::vector<PreviousFrictionPair> previousFrictionPairs;
  for (const dvbd::AvbdScalarRowRecord& record : frictionInventory.records()) {
    auto pairIt = std::find_if(
        previousFrictionPairs.begin(),
        previousFrictionPairs.end(),
        [&](const PreviousFrictionPair& pair) {
          return sameFrictionPair(pair.key, record.descriptor.key);
        });
    if (pairIt == previousFrictionPairs.end()) {
      previousFrictionPairs.push_back(
          PreviousFrictionPair{
              record.descriptor.key, record.state.lambda * record.direction});
    } else {
      pairIt->worldDual += record.state.lambda * record.direction;
    }
  }
  const auto previousFrictionPairForKey = [&](const dvbd::AvbdScalarRowKey& key)
      -> std::optional<PreviousFrictionPair> {
    for (const PreviousFrictionPair& pair : previousFrictionPairs) {
      if (sameFrictionPair(pair.key, key)) {
        return pair;
      }
    }
    return std::nullopt;
  };

  dvbd::buildAvbdRigidContactManifoldRows(
      nudged.states,
      nudged.contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      warmStart);

  ASSERT_EQ(normalInventory.size(), nudged.contacts.size());
  ASSERT_EQ(frictionInventory.size(), 2u * nudged.contacts.size());
  ASSERT_EQ(frictionRows.size(), nudged.contacts.size());

  for (const dvbd::AvbdScalarRowRecord& record : normalInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::ContactNormal);
    EXPECT_NEAR(
        record.state.lambda,
        expectedNormalLambda(record.descriptor.key),
        1e-12);
  }
  for (const dvbd::AvbdScalarRowRecord& record : frictionInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::FrictionTangent);
    const double forceLimit = 0.4 * expectedNormalLambda(record.descriptor.key);
    EXPECT_NEAR(record.descriptor.bounds.lower, -forceLimit, 1e-12);
    EXPECT_NEAR(record.descriptor.bounds.upper, forceLimit, 1e-12);
    const std::optional<PreviousFrictionPair> previousPair
        = previousFrictionPairForKey(record.descriptor.key);
    ASSERT_TRUE(previousPair.has_value());
    EXPECT_NEAR(
        record.state.lambda,
        previousPair->worldDual.dot(record.direction),
        1e-12);
  }
  for (std::size_t i = 0; i < frictionRows.size(); ++i) {
    EXPECT_NEAR(
        frictionRows[i].first.state.lambda,
        frictionInventory[2u * i].state.lambda,
        1e-12);
    EXPECT_NEAR(
        frictionRows[i].second.state.lambda,
        frictionInventory[2u * i + 1u].state.lambda,
        1e-12);
  }
}

//==============================================================================
void expectPlanePrimitiveRowsPersistAcrossSmallPose(
    const sx::CollisionShape& primitiveShape,
    const Eigen::Vector3d& planeNormal,
    double primitiveCenterZ,
    std::uint64_t expectedFeatureLocalIndex)
{
  sx::World world;

  sx::RigidBodyOptions planeOptions;
  planeOptions.isStatic = true;
  auto plane = world.addRigidBody("plane", planeOptions);
  plane.setCollisionShape(
      sx::CollisionShape::makePlane(planeNormal.normalized(), 0.0));
  plane.setFriction(0.64);

  sx::RigidBodyOptions primitiveOptions;
  primitiveOptions.position = Eigen::Vector3d(-0.25, 0.0, primitiveCenterZ);
  auto primitive = world.addRigidBody("primitive", primitiveOptions);
  primitive.setCollisionShape(primitiveShape);
  primitive.setFriction(0.25);

  const std::vector<Eigen::Vector3d> referenceCenters{
      Eigen::Vector3d(-0.25, 0.0, primitiveCenterZ),
      Eigen::Vector3d(0.25, 0.0, primitiveCenterZ)};
  const std::vector<Eigen::Vector3d> nudgedCenters{
      Eigen::Vector3d(-0.2475, 0.0025, primitiveCenterZ),
      Eigen::Vector3d(0.2475, -0.0025, primitiveCenterZ)};

  const auto buildSnapshot
      = [&](const std::vector<Eigen::Vector3d>& centers,
            dvbd::AvbdRigidWorldContactSnapshot& snapshot) {
          std::vector<sx::Contact> contacts;
          for (const Eigen::Vector3d& center : centers) {
            Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
            pose.translation() = center;
            primitive.setTransform(pose);
            const std::vector<sx::Contact> emittedContacts = world.collide();
            ASSERT_FALSE(emittedContacts.empty());
            contacts.insert(
                contacts.end(), emittedContacts.begin(), emittedContacts.end());
          }
          ASSERT_GE(contacts.size(), centers.size());

          snapshot = dvbd::buildAvbdRigidWorldContactSnapshot(
              dart::simulation::detail::registryOf(world),
              contacts,
              dvbd::AvbdRigidWorldContactOptions{});
          ASSERT_EQ(snapshot.contacts.size(), contacts.size());
        };

  dvbd::AvbdRigidWorldContactSnapshot reference;
  dvbd::AvbdRigidWorldContactSnapshot nudged;
  buildSnapshot(referenceCenters, reference);
  buildSnapshot(nudgedCenters, nudged);
  ASSERT_EQ(reference.contacts.size(), nudged.contacts.size());

  using RowKey
      = std::pair<dvbd::AvbdContactEndpointId, dvbd::AvbdContactEndpointId>;
  struct ObservedRow
  {
    RowKey key;
    std::uint32_t row = 0u;
    double frictionCoefficient = 0.0;
  };

  const auto rowKey = [](const dvbd::AvbdRigidContactManifoldPoint& contact) {
    return dvbd::canonicalizeAvbdContactEndpoints(
        contact.endpointA, contact.endpointB);
  };
  const std::uint64_t primitiveObject = dvbd::avbdRigidWorldContactObjectId(
      sx::detail::toRegistryEntity(primitive.getEntity()));
  const auto collectRows
      = [&](const dvbd::AvbdRigidWorldContactSnapshot& snapshot) {
          std::vector<ObservedRow> rows;
          rows.reserve(snapshot.contacts.size());
          for (const dvbd::AvbdRigidContactManifoldPoint& contact :
               snapshot.contacts) {
            const dvbd::AvbdContactEndpointId primitiveEndpoint
                = contact.endpointA.object == primitiveObject
                      ? contact.endpointA
                      : contact.endpointB;
            EXPECT_EQ(primitiveEndpoint.object, primitiveObject);
            if (primitiveEndpoint.object != primitiveObject) {
              continue;
            }
            EXPECT_EQ(
                dvbd::avbdContactFeatureKind(primitiveEndpoint.feature),
                dvbd::AvbdContactFeatureKind::Face);
            EXPECT_EQ(
                dvbd::avbdContactFeatureLocalIndex(primitiveEndpoint.feature),
                expectedFeatureLocalIndex);
            EXPECT_NEAR(contact.frictionCoefficient, 0.4, 1e-12);
            rows.push_back(
                ObservedRow{
                    rowKey(contact), contact.row, contact.frictionCoefficient});
          }
          return rows;
        };
  const auto observedAtRow
      = [](const std::vector<ObservedRow>& rows,
           const RowKey& key,
           std::uint32_t row) -> std::optional<ObservedRow> {
    for (const ObservedRow& observed : rows) {
      if (observed.key == key && observed.row == row) {
        return observed;
      }
    }
    return std::nullopt;
  };

  const std::vector<ObservedRow> referenceRows = collectRows(reference);
  const std::vector<ObservedRow> nudgedRows = collectRows(nudged);
  ASSERT_EQ(referenceRows.size(), reference.contacts.size());
  ASSERT_EQ(nudgedRows.size(), nudged.contacts.size());

  std::vector<std::uint32_t> rows;
  rows.reserve(referenceRows.size());
  for (const ObservedRow& referenceRow : referenceRows) {
    const std::optional<ObservedRow> nudgedRow
        = observedAtRow(nudgedRows, referenceRow.key, referenceRow.row);
    ASSERT_TRUE(nudgedRow.has_value());
    EXPECT_NEAR(
        referenceRow.frictionCoefficient,
        nudgedRow->frictionCoefficient,
        1e-12);
    rows.push_back(referenceRow.row);
  }

  std::sort(rows.begin(), rows.end());
  rows.erase(std::unique(rows.begin(), rows.end()), rows.end());
  EXPECT_GE(rows.size(), 2u);
  for (std::size_t row = 0; row < rows.size(); ++row) {
    EXPECT_EQ(rows[row], row);
  }

  dvbd::AvbdScalarRowInventory normalInventory;
  dvbd::AvbdScalarRowInventory frictionInventory;
  std::vector<dvbd::AvbdRigidBodyPointPairRow> normalRows;
  std::vector<dvbd::AvbdRigidBodyPointPairFrictionRows> frictionRows;
  dvbd::AvbdRowWarmStartOptions warmStart;
  warmStart.alpha = 1.0;
  warmStart.gamma = 1.0;

  dvbd::buildAvbdRigidContactManifoldRows(
      reference.states,
      reference.contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      warmStart);

  ASSERT_EQ(normalInventory.size(), reference.contacts.size());
  ASSERT_EQ(frictionInventory.size(), 2u * reference.contacts.size());
  ASSERT_EQ(frictionRows.size(), reference.contacts.size());

  const auto keyFingerprint = [](const dvbd::AvbdScalarRowKey& key) {
    return static_cast<double>(
        key.objectA % 17u + 2u * (key.objectB % 19u) + 3u * (key.featureA % 23u)
        + 5u * (key.featureB % 29u));
  };
  const auto expectedNormalLambda = [&](const dvbd::AvbdScalarRowKey& key) {
    return 21.0 + static_cast<double>(key.row) + 0.01 * keyFingerprint(key);
  };
  const auto expectedFrictionLambda = [&](const dvbd::AvbdScalarRowKey& key) {
    return 6.0 + 0.5 * static_cast<double>(key.row)
           + 0.125 * static_cast<double>(key.axis) + 0.01 * keyFingerprint(key);
  };

  for (dvbd::AvbdScalarRowRecord& record : normalInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::ContactNormal);
    record.state.lambda = expectedNormalLambda(record.descriptor.key);
  }
  for (dvbd::AvbdScalarRowRecord& record : frictionInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::FrictionTangent);
    record.state.lambda = expectedFrictionLambda(record.descriptor.key);
  }

  dvbd::buildAvbdRigidContactManifoldRows(
      nudged.states,
      nudged.contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      warmStart);

  ASSERT_EQ(normalInventory.size(), nudged.contacts.size());
  ASSERT_EQ(frictionInventory.size(), 2u * nudged.contacts.size());
  ASSERT_EQ(frictionRows.size(), nudged.contacts.size());

  for (const dvbd::AvbdScalarRowRecord& record : normalInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::ContactNormal);
    EXPECT_NEAR(
        record.state.lambda,
        expectedNormalLambda(record.descriptor.key),
        1e-12);
  }
  for (const dvbd::AvbdScalarRowRecord& record : frictionInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::FrictionTangent);
    const double forceLimit = 0.4 * expectedNormalLambda(record.descriptor.key);
    EXPECT_NEAR(record.descriptor.bounds.lower, -forceLimit, 1e-12);
    EXPECT_NEAR(record.descriptor.bounds.upper, forceLimit, 1e-12);
    EXPECT_NEAR(
        record.state.lambda,
        expectedFrictionLambda(record.descriptor.key),
        1e-12);
  }
  for (std::size_t i = 0; i < frictionRows.size(); ++i) {
    EXPECT_NEAR(
        frictionRows[i].first.state.lambda,
        frictionInventory[2u * i].state.lambda,
        1e-12);
    EXPECT_NEAR(
        frictionRows[i].second.state.lambda,
        frictionInventory[2u * i + 1u].state.lambda,
        1e-12);
  }
}

//==============================================================================
struct PrimitiveRowKey
{
  dvbd::AvbdContactEndpointId endpointA;
  dvbd::AvbdContactEndpointId endpointB;
};

bool primitiveRowKeysEqual(
    const PrimitiveRowKey& lhs, const PrimitiveRowKey& rhs)
{
  return lhs.endpointA == rhs.endpointA && lhs.endpointB == rhs.endpointB;
}

struct PrimitiveObservedRow
{
  PrimitiveRowKey key;
  std::uint32_t row = 0u;
  Eigen::Vector3d point = Eigen::Vector3d::Zero();
  double depth = 0.0;
  double frictionCoefficient = 0.0;
};

//==============================================================================
PrimitiveRowKey primitiveRowKey(
    const dvbd::AvbdRigidContactManifoldPoint& contact)
{
  const auto endpoints = dvbd::canonicalizeAvbdContactEndpoints(
      contact.endpointA, contact.endpointB);
  return PrimitiveRowKey{endpoints.first, endpoints.second};
}

//==============================================================================
std::vector<PrimitiveObservedRow> collectPrimitiveRows(
    const dvbd::AvbdRigidWorldContactSnapshot& snapshot,
    std::uint64_t primitiveObject,
    dvbd::AvbdContactFeatureKind expectedFeatureKind,
    std::uint64_t expectedFeatureLocalIndex)
{
  std::vector<PrimitiveObservedRow> rows;
  rows.reserve(snapshot.contacts.size());
  for (const dvbd::AvbdRigidContactManifoldPoint& contact : snapshot.contacts) {
    const dvbd::AvbdContactEndpointId primitiveEndpoint
        = contact.endpointA.object == primitiveObject ? contact.endpointA
                                                      : contact.endpointB;
    EXPECT_EQ(primitiveEndpoint.object, primitiveObject);
    if (primitiveEndpoint.object != primitiveObject) {
      continue;
    }
    EXPECT_EQ(
        dvbd::avbdContactFeatureKind(primitiveEndpoint.feature),
        expectedFeatureKind);
    EXPECT_EQ(
        dvbd::avbdContactFeatureLocalIndex(primitiveEndpoint.feature),
        expectedFeatureLocalIndex);
    EXPECT_NEAR(contact.frictionCoefficient, 0.4, 1e-12);
    rows.push_back(
        PrimitiveObservedRow{
            primitiveRowKey(contact),
            contact.row,
            contact.point,
            contact.depth,
            contact.frictionCoefficient});
  }
  return rows;
}

//==============================================================================
std::optional<PrimitiveObservedRow> observedPrimitiveRowAtPoint(
    const std::vector<PrimitiveObservedRow>& rows,
    const PrimitiveRowKey& key,
    const Eigen::Vector3d& point,
    double depth)
{
  for (const PrimitiveObservedRow& observed : rows) {
    if (primitiveRowKeysEqual(observed.key, key)
        && (observed.point - point).norm() <= 1e-10
        && std::abs(observed.depth - depth) <= 1e-10) {
      return observed;
    }
  }
  return std::nullopt;
}

//==============================================================================
void expectPrimitiveEndpointRowsMatch(
    const dvbd::AvbdRigidWorldContactSnapshot& forward,
    const dvbd::AvbdRigidWorldContactSnapshot& swapped,
    std::uint64_t primitiveObject,
    dvbd::AvbdContactFeatureKind expectedFeatureKind,
    std::uint64_t expectedFeatureLocalIndex)
{
  ASSERT_EQ(forward.contacts.size(), swapped.contacts.size());
  ASSERT_GE(forward.contacts.size(), 2u);

  const std::vector<PrimitiveObservedRow> forwardRows = collectPrimitiveRows(
      forward, primitiveObject, expectedFeatureKind, expectedFeatureLocalIndex);
  const std::vector<PrimitiveObservedRow> swappedRows = collectPrimitiveRows(
      swapped, primitiveObject, expectedFeatureKind, expectedFeatureLocalIndex);
  ASSERT_EQ(forwardRows.size(), forward.contacts.size());
  ASSERT_EQ(swappedRows.size(), swapped.contacts.size());

  std::vector<std::pair<PrimitiveRowKey, std::vector<std::uint32_t>>>
      groupedRows;
  for (const PrimitiveObservedRow& forwardRow : forwardRows) {
    const std::optional<PrimitiveObservedRow> swappedRow
        = observedPrimitiveRowAtPoint(
            swappedRows, forwardRow.key, forwardRow.point, forwardRow.depth);
    ASSERT_TRUE(swappedRow.has_value());
    EXPECT_EQ(forwardRow.row, swappedRow->row);
    EXPECT_NEAR(
        forwardRow.frictionCoefficient, swappedRow->frictionCoefficient, 1e-12);

    auto groupIt = std::find_if(
        groupedRows.begin(),
        groupedRows.end(),
        [&](const std::pair<PrimitiveRowKey, std::vector<std::uint32_t>>&
                group) {
          return primitiveRowKeysEqual(group.first, forwardRow.key);
        });
    if (groupIt == groupedRows.end()) {
      groupedRows.push_back({forwardRow.key, {forwardRow.row}});
    } else {
      groupIt->second.push_back(forwardRow.row);
    }
  }

  std::size_t sameFeatureGroups = 0u;
  for (auto& group : groupedRows) {
    std::vector<std::uint32_t>& rows = group.second;
    std::sort(rows.begin(), rows.end());
    rows.erase(std::unique(rows.begin(), rows.end()), rows.end());
    if (rows.size() >= 2u) {
      ++sameFeatureGroups;
      for (std::size_t row = 0; row < rows.size(); ++row) {
        EXPECT_EQ(rows[row], row);
      }
    }
  }
  EXPECT_GE(sameFeatureGroups, 1u);
}

//==============================================================================
double primitiveKeyFingerprint(const dvbd::AvbdScalarRowKey& key)
{
  return static_cast<double>(
      key.objectA % 17u + 2u * (key.objectB % 19u) + 3u * (key.featureA % 23u)
      + 5u * (key.featureB % 29u));
}

//==============================================================================
double primitiveExpectedNormalLambda(const dvbd::AvbdScalarRowKey& key)
{
  return 23.0 + static_cast<double>(key.row)
         + 0.01 * primitiveKeyFingerprint(key);
}

//==============================================================================
double primitiveExpectedFrictionLambda(const dvbd::AvbdScalarRowKey& key)
{
  return 7.0 + 0.5 * static_cast<double>(key.row)
         + 0.125 * static_cast<double>(key.axis)
         + 0.01 * primitiveKeyFingerprint(key);
}

struct PrimitivePreviousFrictionPair
{
  dvbd::AvbdScalarRowKey key;
  Eigen::Vector3d worldDual = Eigen::Vector3d::Zero();
};

//==============================================================================
bool samePrimitiveFrictionPair(
    const dvbd::AvbdScalarRowKey& lhs, const dvbd::AvbdScalarRowKey& rhs)
{
  return lhs.role == rhs.role && lhs.objectA == rhs.objectA
         && lhs.objectB == rhs.objectB && lhs.featureA == rhs.featureA
         && lhs.featureB == rhs.featureB && lhs.row == rhs.row;
}

//==============================================================================
std::vector<PrimitivePreviousFrictionPair>
collectPrimitivePreviousFrictionPairs(
    const dvbd::AvbdScalarRowInventory& frictionInventory)
{
  std::vector<PrimitivePreviousFrictionPair> previousFrictionPairs;
  for (const dvbd::AvbdScalarRowRecord& record : frictionInventory.records()) {
    auto pairIt = std::find_if(
        previousFrictionPairs.begin(),
        previousFrictionPairs.end(),
        [&](const PrimitivePreviousFrictionPair& pair) {
          return samePrimitiveFrictionPair(pair.key, record.descriptor.key);
        });
    if (pairIt == previousFrictionPairs.end()) {
      previousFrictionPairs.push_back(
          PrimitivePreviousFrictionPair{
              record.descriptor.key, record.state.lambda * record.direction});
    } else {
      pairIt->worldDual += record.state.lambda * record.direction;
    }
  }
  return previousFrictionPairs;
}

//==============================================================================
std::optional<PrimitivePreviousFrictionPair> findPrimitivePreviousFrictionPair(
    const std::vector<PrimitivePreviousFrictionPair>& previousFrictionPairs,
    const dvbd::AvbdScalarRowKey& key)
{
  for (const PrimitivePreviousFrictionPair& pair : previousFrictionPairs) {
    if (samePrimitiveFrictionPair(pair.key, key)) {
      return pair;
    }
  }
  return std::nullopt;
}

//==============================================================================
void expectPrimitiveEndpointWarmStarts(
    const dvbd::AvbdRigidWorldContactSnapshot& forward,
    const dvbd::AvbdRigidWorldContactSnapshot& swapped)
{
  dvbd::AvbdScalarRowInventory normalInventory;
  dvbd::AvbdScalarRowInventory frictionInventory;
  std::vector<dvbd::AvbdRigidBodyPointPairRow> normalRows;
  std::vector<dvbd::AvbdRigidBodyPointPairFrictionRows> frictionRows;
  dvbd::AvbdRowWarmStartOptions warmStart;
  warmStart.alpha = 1.0;
  warmStart.gamma = 1.0;

  dvbd::buildAvbdRigidContactManifoldRows(
      forward.states,
      forward.contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      warmStart);

  ASSERT_EQ(normalInventory.size(), forward.contacts.size());
  ASSERT_EQ(frictionInventory.size(), 2u * forward.contacts.size());
  ASSERT_EQ(frictionRows.size(), forward.contacts.size());

  for (dvbd::AvbdScalarRowRecord& record : normalInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::ContactNormal);
    record.state.lambda = primitiveExpectedNormalLambda(record.descriptor.key);
  }
  for (dvbd::AvbdScalarRowRecord& record : frictionInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::FrictionTangent);
    record.state.lambda
        = primitiveExpectedFrictionLambda(record.descriptor.key);
  }

  const std::vector<PrimitivePreviousFrictionPair> previousFrictionPairs
      = collectPrimitivePreviousFrictionPairs(frictionInventory);

  dvbd::buildAvbdRigidContactManifoldRows(
      swapped.states,
      swapped.contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      warmStart);

  ASSERT_EQ(normalInventory.size(), swapped.contacts.size());
  ASSERT_EQ(frictionInventory.size(), 2u * swapped.contacts.size());
  ASSERT_EQ(frictionRows.size(), swapped.contacts.size());

  for (const dvbd::AvbdScalarRowRecord& record : normalInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::ContactNormal);
    EXPECT_NEAR(
        record.state.lambda,
        primitiveExpectedNormalLambda(record.descriptor.key),
        1e-12);
  }
  for (const dvbd::AvbdScalarRowRecord& record : frictionInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::FrictionTangent);
    const double forceLimit
        = 0.4 * primitiveExpectedNormalLambda(record.descriptor.key);
    EXPECT_NEAR(record.descriptor.bounds.lower, -forceLimit, 1e-12);
    EXPECT_NEAR(record.descriptor.bounds.upper, forceLimit, 1e-12);
    const std::optional<PrimitivePreviousFrictionPair> previousPair
        = findPrimitivePreviousFrictionPair(
            previousFrictionPairs, record.descriptor.key);
    ASSERT_TRUE(previousPair.has_value());
    EXPECT_NEAR(
        record.state.lambda,
        previousPair->worldDual.dot(record.direction),
        1e-12);
  }
  for (std::size_t i = 0; i < frictionRows.size(); ++i) {
    EXPECT_NEAR(
        frictionRows[i].first.state.lambda,
        frictionInventory[2u * i].state.lambda,
        1e-12);
    EXPECT_NEAR(
        frictionRows[i].second.state.lambda,
        frictionInventory[2u * i + 1u].state.lambda,
        1e-12);
  }
}

//==============================================================================
void expectPrimitiveEndpointOrderRowsAndWarmStarts(
    const dvbd::AvbdRigidWorldContactSnapshot& forward,
    const dvbd::AvbdRigidWorldContactSnapshot& swapped,
    std::uint64_t primitiveObject,
    dvbd::AvbdContactFeatureKind expectedFeatureKind,
    std::uint64_t expectedFeatureLocalIndex)
{
  expectPrimitiveEndpointRowsMatch(
      forward,
      swapped,
      primitiveObject,
      expectedFeatureKind,
      expectedFeatureLocalIndex);
  expectPrimitiveEndpointWarmStarts(forward, swapped);
}

//==============================================================================
void expectSpherePrimitiveRowsIgnoreEndpointOrder(
    const sx::CollisionShape& primitiveShape,
    const std::vector<Eigen::Vector3d>& sphereCenters,
    dvbd::AvbdContactFeatureKind expectedFeatureKind,
    std::uint64_t expectedFeatureLocalIndex)
{
  ASSERT_GE(sphereCenters.size(), 2u);

  sx::World world;

  auto primitive = world.addRigidBody("primitive");
  primitive.setCollisionShape(primitiveShape);
  primitive.setFriction(0.64);

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.position = sphereCenters.front();
  auto sphere = world.addRigidBody("sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.25));
  sphere.setFriction(0.25);

  std::vector<sx::Contact> contacts;
  for (const Eigen::Vector3d& center : sphereCenters) {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = center;
    sphere.setTransform(pose);
    const std::vector<sx::Contact> emittedContacts = world.collide();
    ASSERT_FALSE(emittedContacts.empty());
    contacts.insert(
        contacts.end(), emittedContacts.begin(), emittedContacts.end());
  }
  ASSERT_GE(contacts.size(), sphereCenters.size());

  std::vector<sx::Contact> swappedContacts;
  swappedContacts.reserve(contacts.size());
  for (const sx::Contact& contact : contacts) {
    swappedContacts.push_back(swapEndpointOrder(contact));
  }

  const auto buildSnapshot = [&](const std::vector<sx::Contact>& rawContacts) {
    return dvbd::buildAvbdRigidWorldContactSnapshot(
        dart::simulation::detail::registryOf(world),
        rawContacts,
        dvbd::AvbdRigidWorldContactOptions{});
  };
  const dvbd::AvbdRigidWorldContactSnapshot forward = buildSnapshot(contacts);
  const dvbd::AvbdRigidWorldContactSnapshot swapped
      = buildSnapshot(swappedContacts);

  expectPrimitiveEndpointOrderRowsAndWarmStarts(
      forward,
      swapped,
      dvbd::avbdRigidWorldContactObjectId(
          sx::detail::toRegistryEntity(primitive.getEntity())),
      expectedFeatureKind,
      expectedFeatureLocalIndex);
}

//==============================================================================
void expectPlanePrimitiveRowsIgnoreEndpointOrder(
    const sx::CollisionShape& primitiveShape,
    const Eigen::Vector3d& planeNormal,
    double primitiveCenterZ,
    std::uint64_t expectedFeatureLocalIndex)
{
  sx::World world;

  sx::RigidBodyOptions planeOptions;
  planeOptions.isStatic = true;
  auto plane = world.addRigidBody("plane", planeOptions);
  plane.setCollisionShape(
      sx::CollisionShape::makePlane(planeNormal.normalized(), 0.0));
  plane.setFriction(0.64);

  sx::RigidBodyOptions primitiveOptions;
  primitiveOptions.position = Eigen::Vector3d(-0.25, 0.0, primitiveCenterZ);
  auto primitive = world.addRigidBody("primitive", primitiveOptions);
  primitive.setCollisionShape(primitiveShape);
  primitive.setFriction(0.25);

  std::vector<sx::Contact> contacts;
  for (const Eigen::Vector3d& center :
       {Eigen::Vector3d(-0.25, 0.0, primitiveCenterZ),
        Eigen::Vector3d(0.25, 0.0, primitiveCenterZ)}) {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = center;
    primitive.setTransform(pose);
    const std::vector<sx::Contact> emittedContacts = world.collide();
    ASSERT_FALSE(emittedContacts.empty());
    contacts.insert(
        contacts.end(), emittedContacts.begin(), emittedContacts.end());
  }
  ASSERT_GE(contacts.size(), 2u);

  std::vector<sx::Contact> swappedContacts;
  swappedContacts.reserve(contacts.size());
  for (const sx::Contact& contact : contacts) {
    swappedContacts.push_back(swapEndpointOrder(contact));
  }

  const auto buildSnapshot = [&](const std::vector<sx::Contact>& rawContacts) {
    return dvbd::buildAvbdRigidWorldContactSnapshot(
        dart::simulation::detail::registryOf(world),
        rawContacts,
        dvbd::AvbdRigidWorldContactOptions{});
  };
  const dvbd::AvbdRigidWorldContactSnapshot forward = buildSnapshot(contacts);
  const dvbd::AvbdRigidWorldContactSnapshot swapped
      = buildSnapshot(swappedContacts);

  expectPrimitiveEndpointOrderRowsAndWarmStarts(
      forward,
      swapped,
      dvbd::avbdRigidWorldContactObjectId(
          sx::detail::toRegistryEntity(primitive.getEntity())),
      dvbd::AvbdContactFeatureKind::Face,
      expectedFeatureLocalIndex);
}

//==============================================================================
void expectMeshFeatureRowsIgnoreContactOrder(
    const std::vector<Eigen::Vector3d>& sphereCenters,
    dvbd::AvbdContactFeatureKind expectedFeatureKind,
    std::uint64_t expectedFeatureLocalIndex)
{
  ASSERT_GE(sphereCenters.size(), 2u);

  sx::World world;

  sx::RigidBodyOptions meshOptions;
  meshOptions.isStatic = true;
  auto mesh = world.addRigidBody("mesh", meshOptions);
  mesh.setCollisionShape(
      sx::CollisionShape::makeMesh(
          {Eigen::Vector3d(-1.0, -1.0, 0.0),
           Eigen::Vector3d(1.0, -1.0, 0.0),
           Eigen::Vector3d(0.0, 1.0, 0.0)},
          {Eigen::Vector3i(0, 1, 2)}));

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.position = sphereCenters.front();
  auto sphere = world.addRigidBody("sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.25));

  std::vector<sx::Contact> contacts;
  for (const Eigen::Vector3d& center : sphereCenters) {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = center;
    sphere.setTransform(pose);
    const std::vector<sx::Contact> emittedContacts = world.collide();
    ASSERT_FALSE(emittedContacts.empty());
    contacts.insert(
        contacts.end(), emittedContacts.begin(), emittedContacts.end());
  }
  ASSERT_GE(contacts.size(), 2u);

  const auto buildSnapshot = [&](std::vector<sx::Contact> orderedContacts) {
    return dvbd::buildAvbdRigidWorldContactSnapshot(
        dart::simulation::detail::registryOf(world),
        orderedContacts,
        dvbd::AvbdRigidWorldContactOptions{});
  };

  const dvbd::AvbdRigidWorldContactSnapshot forward = buildSnapshot(contacts);
  std::vector<sx::Contact> reversedContacts = contacts;
  std::reverse(reversedContacts.begin(), reversedContacts.end());
  const dvbd::AvbdRigidWorldContactSnapshot reversed
      = buildSnapshot(reversedContacts);

  ASSERT_EQ(forward.contacts.size(), contacts.size());
  ASSERT_EQ(reversed.contacts.size(), contacts.size());

  using RowKey
      = std::pair<dvbd::AvbdContactEndpointId, dvbd::AvbdContactEndpointId>;

  const auto rowKey = [](const dvbd::AvbdRigidContactManifoldPoint& contact) {
    return dvbd::canonicalizeAvbdContactEndpoints(
        contact.endpointA, contact.endpointB);
  };
  const RowKey expectedRowKey = rowKey(forward.contacts.front());
  const auto rowForPoint
      = [&](const Eigen::Vector3d& point) -> std::optional<std::uint32_t> {
    for (const dvbd::AvbdRigidContactManifoldPoint& contact :
         reversed.contacts) {
      if (rowKey(contact) == expectedRowKey
          && (contact.point - point).norm() <= 1e-10) {
        return contact.row;
      }
    }
    return std::nullopt;
  };

  const std::uint64_t meshObject = dvbd::avbdRigidWorldContactObjectId(
      sx::detail::toRegistryEntity(mesh.getEntity()));
  std::vector<std::uint32_t> rows;
  rows.reserve(forward.contacts.size());
  for (const dvbd::AvbdRigidContactManifoldPoint& contact : forward.contacts) {
    EXPECT_EQ(rowKey(contact), expectedRowKey);

    const dvbd::AvbdContactEndpointId meshEndpoint
        = contact.endpointA.object == meshObject ? contact.endpointA
                                                 : contact.endpointB;
    ASSERT_EQ(meshEndpoint.object, meshObject);
    EXPECT_EQ(
        dvbd::avbdContactFeatureKind(meshEndpoint.feature),
        expectedFeatureKind);
    EXPECT_EQ(
        dvbd::avbdContactFeatureLocalIndex(meshEndpoint.feature),
        expectedFeatureLocalIndex);

    const std::optional<std::uint32_t> reversedRow = rowForPoint(contact.point);
    ASSERT_TRUE(reversedRow.has_value());
    EXPECT_EQ(contact.row, *reversedRow);
    rows.push_back(contact.row);
  }

  std::sort(rows.begin(), rows.end());
  rows.erase(std::unique(rows.begin(), rows.end()), rows.end());
  EXPECT_GE(rows.size(), 2u);
  for (std::size_t row = 0; row < rows.size(); ++row) {
    EXPECT_EQ(rows[row], row);
  }
}

//==============================================================================
void expectMeshFeatureRowsIgnoreEndpointOrder(
    const std::vector<Eigen::Vector3d>& sphereCenters,
    dvbd::AvbdContactFeatureKind expectedFeatureKind,
    std::uint64_t expectedFeatureLocalIndex)
{
  ASSERT_GE(sphereCenters.size(), 2u);

  sx::World world;

  sx::RigidBodyOptions meshOptions;
  meshOptions.isStatic = true;
  auto mesh = world.addRigidBody("mesh", meshOptions);
  mesh.setCollisionShape(
      sx::CollisionShape::makeMesh(
          {Eigen::Vector3d(-1.0, -1.0, 0.0),
           Eigen::Vector3d(1.0, -1.0, 0.0),
           Eigen::Vector3d(0.0, 1.0, 0.0)},
          {Eigen::Vector3i(0, 1, 2)}));
  mesh.setFriction(0.64);

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.position = sphereCenters.front();
  auto sphere = world.addRigidBody("sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.25));
  sphere.setFriction(0.25);

  std::vector<sx::Contact> contacts;
  for (const Eigen::Vector3d& center : sphereCenters) {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = center;
    sphere.setTransform(pose);
    const std::vector<sx::Contact> emittedContacts = world.collide();
    ASSERT_FALSE(emittedContacts.empty());
    contacts.insert(
        contacts.end(), emittedContacts.begin(), emittedContacts.end());
  }
  ASSERT_GE(contacts.size(), sphereCenters.size());

  std::vector<sx::Contact> swappedContacts;
  swappedContacts.reserve(contacts.size());
  for (const sx::Contact& contact : contacts) {
    swappedContacts.push_back(swapEndpointOrder(contact));
  }

  const auto buildSnapshot = [&](const std::vector<sx::Contact>& rawContacts) {
    return dvbd::buildAvbdRigidWorldContactSnapshot(
        dart::simulation::detail::registryOf(world),
        rawContacts,
        dvbd::AvbdRigidWorldContactOptions{});
  };
  const dvbd::AvbdRigidWorldContactSnapshot forward = buildSnapshot(contacts);
  const dvbd::AvbdRigidWorldContactSnapshot swapped
      = buildSnapshot(swappedContacts);

  expectPrimitiveEndpointOrderRowsAndWarmStarts(
      forward,
      swapped,
      dvbd::avbdRigidWorldContactObjectId(
          sx::detail::toRegistryEntity(mesh.getEntity())),
      expectedFeatureKind,
      expectedFeatureLocalIndex);
}

//==============================================================================
void expectMeshFeatureRowsPersistAcrossSmallPose(
    const std::vector<Eigen::Vector3d>& referenceCenters,
    const std::vector<Eigen::Vector3d>& nudgedCenters,
    dvbd::AvbdContactFeatureKind expectedFeatureKind,
    std::uint64_t expectedFeatureLocalIndex)
{
  ASSERT_GE(referenceCenters.size(), 2u);
  ASSERT_EQ(referenceCenters.size(), nudgedCenters.size());

  sx::World world;

  sx::RigidBodyOptions meshOptions;
  meshOptions.isStatic = true;
  auto mesh = world.addRigidBody("mesh", meshOptions);
  mesh.setCollisionShape(
      sx::CollisionShape::makeMesh(
          {Eigen::Vector3d(-1.0, -1.0, 0.0),
           Eigen::Vector3d(1.0, -1.0, 0.0),
           Eigen::Vector3d(0.0, 1.0, 0.0)},
          {Eigen::Vector3i(0, 1, 2)}));
  mesh.setFriction(0.64);

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.position = referenceCenters.front();
  auto sphere = world.addRigidBody("sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.25));
  sphere.setFriction(0.25);

  const auto buildSnapshot
      = [&](const std::vector<Eigen::Vector3d>& centers,
            dvbd::AvbdRigidWorldContactSnapshot& snapshot) {
          std::vector<sx::Contact> contacts;
          for (const Eigen::Vector3d& center : centers) {
            Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
            pose.translation() = center;
            sphere.setTransform(pose);
            const std::vector<sx::Contact> emittedContacts = world.collide();
            ASSERT_FALSE(emittedContacts.empty());
            contacts.insert(
                contacts.end(), emittedContacts.begin(), emittedContacts.end());
          }
          ASSERT_GE(contacts.size(), centers.size());

          snapshot = dvbd::buildAvbdRigidWorldContactSnapshot(
              dart::simulation::detail::registryOf(world),
              contacts,
              dvbd::AvbdRigidWorldContactOptions{});
          ASSERT_EQ(snapshot.contacts.size(), contacts.size());
        };

  dvbd::AvbdRigidWorldContactSnapshot reference;
  dvbd::AvbdRigidWorldContactSnapshot nudged;
  buildSnapshot(referenceCenters, reference);
  buildSnapshot(nudgedCenters, nudged);
  ASSERT_EQ(reference.contacts.size(), nudged.contacts.size());

  using RowKey
      = std::pair<dvbd::AvbdContactEndpointId, dvbd::AvbdContactEndpointId>;
  struct ObservedRow
  {
    RowKey key;
    std::uint32_t row = 0u;
    double frictionCoefficient = 0.0;
  };

  const auto rowKey = [](const dvbd::AvbdRigidContactManifoldPoint& contact) {
    return dvbd::canonicalizeAvbdContactEndpoints(
        contact.endpointA, contact.endpointB);
  };
  const std::uint64_t meshObject = dvbd::avbdRigidWorldContactObjectId(
      sx::detail::toRegistryEntity(mesh.getEntity()));
  const auto collectRows
      = [&](const dvbd::AvbdRigidWorldContactSnapshot& snapshot) {
          std::vector<ObservedRow> rows;
          rows.reserve(snapshot.contacts.size());
          for (const dvbd::AvbdRigidContactManifoldPoint& contact :
               snapshot.contacts) {
            const dvbd::AvbdContactEndpointId meshEndpoint
                = contact.endpointA.object == meshObject ? contact.endpointA
                                                         : contact.endpointB;
            EXPECT_EQ(meshEndpoint.object, meshObject);
            if (meshEndpoint.object != meshObject) {
              continue;
            }
            EXPECT_EQ(
                dvbd::avbdContactFeatureKind(meshEndpoint.feature),
                expectedFeatureKind);
            EXPECT_EQ(
                dvbd::avbdContactFeatureLocalIndex(meshEndpoint.feature),
                expectedFeatureLocalIndex);
            EXPECT_NEAR(contact.frictionCoefficient, 0.4, 1e-12);
            rows.push_back(
                ObservedRow{
                    rowKey(contact), contact.row, contact.frictionCoefficient});
          }
          return rows;
        };
  const auto observedAtRow
      = [](const std::vector<ObservedRow>& rows,
           const RowKey& key,
           std::uint32_t row) -> std::optional<ObservedRow> {
    for (const ObservedRow& observed : rows) {
      if (observed.key == key && observed.row == row) {
        return observed;
      }
    }
    return std::nullopt;
  };

  const std::vector<ObservedRow> referenceRows = collectRows(reference);
  const std::vector<ObservedRow> nudgedRows = collectRows(nudged);
  ASSERT_EQ(referenceRows.size(), reference.contacts.size());
  ASSERT_EQ(nudgedRows.size(), nudged.contacts.size());

  std::vector<std::uint32_t> rows;
  rows.reserve(referenceRows.size());
  for (const ObservedRow& referenceRow : referenceRows) {
    const std::optional<ObservedRow> nudgedRow
        = observedAtRow(nudgedRows, referenceRow.key, referenceRow.row);
    ASSERT_TRUE(nudgedRow.has_value());
    EXPECT_NEAR(
        referenceRow.frictionCoefficient,
        nudgedRow->frictionCoefficient,
        1e-12);
    rows.push_back(referenceRow.row);
  }

  std::sort(rows.begin(), rows.end());
  rows.erase(std::unique(rows.begin(), rows.end()), rows.end());
  EXPECT_GE(rows.size(), 2u);
  for (std::size_t row = 0; row < rows.size(); ++row) {
    EXPECT_EQ(rows[row], row);
  }

  dvbd::AvbdScalarRowInventory normalInventory;
  dvbd::AvbdScalarRowInventory frictionInventory;
  std::vector<dvbd::AvbdRigidBodyPointPairRow> normalRows;
  std::vector<dvbd::AvbdRigidBodyPointPairFrictionRows> frictionRows;
  dvbd::AvbdRowWarmStartOptions warmStart;
  warmStart.alpha = 1.0;
  warmStart.gamma = 1.0;

  dvbd::buildAvbdRigidContactManifoldRows(
      reference.states,
      reference.contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      warmStart);

  ASSERT_EQ(normalInventory.size(), reference.contacts.size());
  ASSERT_EQ(frictionInventory.size(), 2u * reference.contacts.size());
  ASSERT_EQ(frictionRows.size(), reference.contacts.size());

  const auto keyFingerprint = [](const dvbd::AvbdScalarRowKey& key) {
    return static_cast<double>(
        key.objectA % 17u + 2u * (key.objectB % 19u) + 3u * (key.featureA % 23u)
        + 5u * (key.featureB % 29u));
  };
  const auto expectedNormalLambda = [&](const dvbd::AvbdScalarRowKey& key) {
    return 18.0 + static_cast<double>(key.row) + 0.01 * keyFingerprint(key);
  };
  const auto expectedFrictionLambda = [&](const dvbd::AvbdScalarRowKey& key) {
    return 4.0 + 0.5 * static_cast<double>(key.row)
           + 0.125 * static_cast<double>(key.axis) + 0.01 * keyFingerprint(key);
  };

  for (dvbd::AvbdScalarRowRecord& record : normalInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::ContactNormal);
    record.state.lambda = expectedNormalLambda(record.descriptor.key);
  }
  for (dvbd::AvbdScalarRowRecord& record : frictionInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::FrictionTangent);
    record.state.lambda = expectedFrictionLambda(record.descriptor.key);
  }

  struct PreviousFrictionPair
  {
    dvbd::AvbdScalarRowKey key;
    Eigen::Vector3d worldDual = Eigen::Vector3d::Zero();
  };
  const auto sameFrictionPair = [](const dvbd::AvbdScalarRowKey& lhs,
                                   const dvbd::AvbdScalarRowKey& rhs) {
    return lhs.role == rhs.role && lhs.objectA == rhs.objectA
           && lhs.objectB == rhs.objectB && lhs.featureA == rhs.featureA
           && lhs.featureB == rhs.featureB && lhs.row == rhs.row;
  };
  std::vector<PreviousFrictionPair> previousFrictionPairs;
  for (const dvbd::AvbdScalarRowRecord& record : frictionInventory.records()) {
    auto pairIt = std::find_if(
        previousFrictionPairs.begin(),
        previousFrictionPairs.end(),
        [&](const PreviousFrictionPair& pair) {
          return sameFrictionPair(pair.key, record.descriptor.key);
        });
    if (pairIt == previousFrictionPairs.end()) {
      previousFrictionPairs.push_back(
          PreviousFrictionPair{
              record.descriptor.key, record.state.lambda * record.direction});
    } else {
      pairIt->worldDual += record.state.lambda * record.direction;
    }
  }
  const auto previousFrictionPairForKey = [&](const dvbd::AvbdScalarRowKey& key)
      -> std::optional<PreviousFrictionPair> {
    for (const PreviousFrictionPair& pair : previousFrictionPairs) {
      if (sameFrictionPair(pair.key, key)) {
        return pair;
      }
    }
    return std::nullopt;
  };

  dvbd::buildAvbdRigidContactManifoldRows(
      nudged.states,
      nudged.contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      warmStart);

  ASSERT_EQ(normalInventory.size(), nudged.contacts.size());
  ASSERT_EQ(frictionInventory.size(), 2u * nudged.contacts.size());
  ASSERT_EQ(frictionRows.size(), nudged.contacts.size());

  for (const dvbd::AvbdScalarRowRecord& record : normalInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::ContactNormal);
    EXPECT_NEAR(
        record.state.lambda,
        expectedNormalLambda(record.descriptor.key),
        1e-12);
  }
  for (const dvbd::AvbdScalarRowRecord& record : frictionInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::FrictionTangent);
    const double forceLimit = 0.4 * expectedNormalLambda(record.descriptor.key);
    EXPECT_NEAR(record.descriptor.bounds.lower, -forceLimit, 1e-12);
    EXPECT_NEAR(record.descriptor.bounds.upper, forceLimit, 1e-12);
    const std::optional<PreviousFrictionPair> previousPair
        = previousFrictionPairForKey(record.descriptor.key);
    ASSERT_TRUE(previousPair.has_value());
    EXPECT_NEAR(
        record.state.lambda,
        previousPair->worldDual.dot(record.direction),
        1e-12);
  }
  for (std::size_t i = 0; i < frictionRows.size(); ++i) {
    EXPECT_NEAR(
        frictionRows[i].first.state.lambda,
        frictionInventory[2u * i].state.lambda,
        1e-12);
    EXPECT_NEAR(
        frictionRows[i].second.state.lambda,
        frictionInventory[2u * i + 1u].state.lambda,
        1e-12);
  }
}

} // namespace

//==============================================================================
// The contact-solver selector is reflected by the World and defaults to
// SequentialImpulse, independent of the differentiable flag.
TEST(BoxedLcpContact, MethodSelectorReflectsConstruction)
{
  sx::World defaultWorld;
  EXPECT_EQ(
      defaultWorld.getContactSolverMethod(),
      sx::ContactSolverMethod::SequentialImpulse);

  sx::WorldOptions options;
  options.contactSolverMethod = sx::ContactSolverMethod::BoxedLcp;
  sx::World lcpWorld(options);
  EXPECT_EQ(
      lcpWorld.getContactSolverMethod(), sx::ContactSolverMethod::BoxedLcp);
  EXPECT_FALSE(lcpWorld.isDifferentiable());
}

//==============================================================================
// Compound-shape contacts must key AVBD warm-start rows from the contacted
// shape, not from the body's primary shape.
TEST(AvbdContact, CompoundShapeFeatureKeysUseContactedShapeIndex)
{
  sx::World world;

  auto compound = world.addRigidBody("compound");
  compound.addCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.1, 0.1, 0.1)));
  sx::CollisionShape secondary
      = sx::CollisionShape::makeBox(Eigen::Vector3d(0.25, 0.2, 0.2));
  secondary.localTransform.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);
  compound.addCollisionShape(secondary);

  sx::RigidBodyOptions targetOptions;
  targetOptions.position = Eigen::Vector3d(1.35, 0.0, 0.0);
  auto target = world.addRigidBody("target", targetOptions);
  target.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.25, 0.2, 0.2)));

  const std::vector<sx::Contact> contacts = world.collide();
  ASSERT_FALSE(contacts.empty());
  const sx::Contact& contact = contacts.front();
  const bool compoundIsA
      = sx::detail::toRegistryEntity(contact.bodyA.getEntity())
        == sx::detail::toRegistryEntity(compound.getEntity());
  ASSERT_TRUE(
      compoundIsA
      || sx::detail::toRegistryEntity(contact.bodyB.getEntity())
             == sx::detail::toRegistryEntity(compound.getEntity()));
  const std::size_t compoundShapeIndex
      = compoundIsA ? contact.shapeIndexA : contact.shapeIndexB;
  const Eigen::Vector3d compoundLocalPoint
      = compoundIsA ? contact.localPointA : contact.localPointB;
  ASSERT_EQ(compoundShapeIndex, 1u);
  ASSERT_TRUE(compoundLocalPoint.allFinite());

  const dvbd::AvbdRigidWorldContactSnapshot snapshot
      = dvbd::buildAvbdRigidWorldContactSnapshot(
          dart::simulation::detail::registryOf(world),
          contacts,
          dvbd::AvbdRigidWorldContactOptions{});
  ASSERT_FALSE(snapshot.contacts.empty());
  const auto& manifold = snapshot.contacts.front();
  const dvbd::AvbdContactEndpointId compoundEndpoint
      = compoundIsA ? manifold.endpointA : manifold.endpointB;

  const std::uint64_t featureCode = dvbd::avbdBoxContactFeatureCode(
      compoundLocalPoint, secondary.halfExtents);
  // Each shape occupies a disjoint feature-id block (shapeIndex * stride) so
  // distinct shapes never alias, even across shape types.
  const std::uint64_t expectedFeature = dvbd::packAvbdContactFeatureId(
      dvbd::avbdBoxContactFeatureKind(featureCode),
      compoundShapeIndex * dvbd::kAvbdRigidWorldShapeFeatureStride
          + dvbd::packAvbdBoxContactFeatureId(0, featureCode));
  EXPECT_EQ(compoundEndpoint.feature, expectedFeature);
}

//==============================================================================
// Primitive feature keys should use the shape-local points emitted by the
// narrow phase for the contacted compound shape, not a body-local fallback.
TEST(AvbdContact, PrimitiveFeatureKeysUseNarrowPhaseShapeLocalPoints)
{
  sx::World world;

  auto compound = world.addRigidBody("compound");
  const sx::CollisionShape cylinder
      = sx::CollisionShape::makeCylinder(/*radius=*/0.25, /*halfHeight=*/1.0);
  compound.addCollisionShape(cylinder);
  sx::CollisionShape capsule
      = sx::CollisionShape::makeCapsule(/*radius=*/0.25, /*halfHeight=*/1.0);
  capsule.localTransform.translation() = Eigen::Vector3d(0.0, 1.0, 0.0);
  compound.addCollisionShape(capsule);
  sx::CollisionShape sphere = sx::CollisionShape::makeSphere(0.25);
  sphere.localTransform.translation() = Eigen::Vector3d(0.0, 2.0, 0.0);
  compound.addCollisionShape(sphere);

  sx::RigidBodyOptions cylinderTargetOptions;
  cylinderTargetOptions.position = Eigen::Vector3d(0.45, 0.0, 0.0);
  auto cylinderTarget
      = world.addRigidBody("cylinder_target", cylinderTargetOptions);
  cylinderTarget.setCollisionShape(sx::CollisionShape::makeSphere(0.25));

  sx::RigidBodyOptions capsuleTargetOptions;
  capsuleTargetOptions.position = Eigen::Vector3d(0.0, 1.45, 0.0);
  auto capsuleTarget
      = world.addRigidBody("capsule_target", capsuleTargetOptions);
  capsuleTarget.setCollisionShape(sx::CollisionShape::makeSphere(0.25));

  sx::RigidBodyOptions sphereTargetOptions;
  sphereTargetOptions.position = Eigen::Vector3d(0.0, 2.45, 0.0);
  auto sphereTarget = world.addRigidBody("sphere_target", sphereTargetOptions);
  sphereTarget.setCollisionShape(sx::CollisionShape::makeSphere(0.25));

  const std::vector<sx::Contact> contacts = world.collide();
  ASSERT_FALSE(contacts.empty());

  const dvbd::AvbdRigidWorldContactSnapshot snapshot
      = dvbd::buildAvbdRigidWorldContactSnapshot(
          dart::simulation::detail::registryOf(world),
          contacts,
          dvbd::AvbdRigidWorldContactOptions{});
  ASSERT_EQ(snapshot.contacts.size(), contacts.size());

  const entt::entity compoundEntity
      = sx::detail::toRegistryEntity(compound.getEntity());
  bool sawCylinderFeature = false;
  bool sawCapsuleFeature = false;
  bool sawSphereFeature = false;
  for (std::size_t i = 0; i < contacts.size(); ++i) {
    const sx::Contact& contact = contacts[i];
    const bool compoundIsA
        = sx::detail::toRegistryEntity(contact.bodyA.getEntity())
          == compoundEntity;
    const bool compoundIsB
        = sx::detail::toRegistryEntity(contact.bodyB.getEntity())
          == compoundEntity;
    ASSERT_TRUE(compoundIsA || compoundIsB);

    const std::size_t shapeIndex
        = compoundIsA ? contact.shapeIndexA : contact.shapeIndexB;
    const Eigen::Vector3d shapeLocalPoint
        = compoundIsA ? contact.localPointA : contact.localPointB;
    ASSERT_TRUE(shapeLocalPoint.allFinite());

    const auto& manifoldPoint = snapshot.contacts[i];
    const dvbd::AvbdContactEndpointId compoundEndpoint
        = compoundIsA ? manifoldPoint.endpointA : manifoldPoint.endpointB;

    if (shapeIndex == 0u) {
      const std::uint64_t featureCode = dvbd::avbdCylinderContactFeatureCode(
          shapeLocalPoint, cylinder.radius, cylinder.halfExtents.z());
      const std::uint64_t expectedFeature = dvbd::packAvbdContactFeatureId(
          dvbd::avbdCylinderContactFeatureKind(featureCode),
          dvbd::packAvbdCylinderContactFeatureId(0, featureCode));
      EXPECT_EQ(compoundEndpoint.feature, expectedFeature);
      sawCylinderFeature = true;
    } else if (shapeIndex == 1u) {
      const std::uint64_t featureCode = dvbd::avbdCapsuleContactFeatureCode(
          shapeLocalPoint, capsule.halfExtents.z());
      const std::uint64_t expectedFeature = dvbd::packAvbdContactFeatureId(
          dvbd::avbdCapsuleContactFeatureKind(featureCode),
          dvbd::kAvbdRigidWorldShapeFeatureStride
              + dvbd::packAvbdCapsuleContactFeatureId(0, featureCode));
      EXPECT_EQ(compoundEndpoint.feature, expectedFeature);
      sawCapsuleFeature = true;
    } else if (shapeIndex == 2u) {
      const std::uint64_t expectedFeature = dvbd::packAvbdContactFeatureId(
          dvbd::AvbdContactFeatureKind::Body,
          2u * dvbd::kAvbdRigidWorldShapeFeatureStride);
      EXPECT_EQ(compoundEndpoint.feature, expectedFeature);
      sawSphereFeature = true;
    } else {
      ADD_FAILURE() << "Unexpected compound shape index: " << shapeIndex;
    }
  }

  EXPECT_TRUE(sawCylinderFeature);
  EXPECT_TRUE(sawCapsuleFeature);
  EXPECT_TRUE(sawSphereFeature);
}

//==============================================================================
// Plane and triangle-mesh feature keys should also be backed by the actual
// narrow-phase shape-local point carried by World::collide().
TEST(AvbdContact, PlaneAndMeshFeatureKeysUseNarrowPhaseShapeLocalPoints)
{
  {
    sx::World world;

    auto plane = world.addRigidBody("plane");
    plane.setCollisionShape(
        sx::CollisionShape::makePlane(Eigen::Vector3d::UnitZ(), 0.0));

    sx::RigidBodyOptions sphereOptions;
    sphereOptions.position = Eigen::Vector3d(0.0, 0.0, 0.2);
    auto sphere = world.addRigidBody("sphere", sphereOptions);
    sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.25));

    const std::vector<sx::Contact> contacts = world.collide();
    ASSERT_FALSE(contacts.empty());

    const dvbd::AvbdRigidWorldContactSnapshot snapshot
        = dvbd::buildAvbdRigidWorldContactSnapshot(
            dart::simulation::detail::registryOf(world),
            contacts,
            dvbd::AvbdRigidWorldContactOptions{});
    ASSERT_EQ(snapshot.contacts.size(), contacts.size());

    const entt::entity planeEntity
        = sx::detail::toRegistryEntity(plane.getEntity());
    bool sawPlaneFeature = false;
    for (std::size_t i = 0; i < contacts.size(); ++i) {
      const sx::Contact& contact = contacts[i];
      const bool planeIsA
          = sx::detail::toRegistryEntity(contact.bodyA.getEntity())
            == planeEntity;
      const bool planeIsB
          = sx::detail::toRegistryEntity(contact.bodyB.getEntity())
            == planeEntity;
      ASSERT_TRUE(planeIsA || planeIsB);

      const std::size_t shapeIndex
          = planeIsA ? contact.shapeIndexA : contact.shapeIndexB;
      const Eigen::Vector3d shapeLocalPoint
          = planeIsA ? contact.localPointA : contact.localPointB;
      ASSERT_EQ(shapeIndex, 0u);
      ASSERT_TRUE(shapeLocalPoint.allFinite());

      const auto& manifoldPoint = snapshot.contacts[i];
      const dvbd::AvbdContactEndpointId planeEndpoint
          = planeIsA ? manifoldPoint.endpointA : manifoldPoint.endpointB;
      const std::uint64_t expectedFeature = dvbd::packAvbdContactFeatureId(
          dvbd::AvbdContactFeatureKind::Face,
          dvbd::kAvbdRigidWorldPlaneContactFeatureIdOffset);
      EXPECT_EQ(planeEndpoint.feature, expectedFeature);
      sawPlaneFeature = true;
    }
    EXPECT_TRUE(sawPlaneFeature);
  }

  {
    sx::World world;

    auto mesh = world.addRigidBody("mesh");
    mesh.setCollisionShape(
        sx::CollisionShape::makeMesh(
            {Eigen::Vector3d(-1.0, -1.0, 0.0),
             Eigen::Vector3d(1.0, -1.0, 0.0),
             Eigen::Vector3d(0.0, 1.0, 0.0)},
            {Eigen::Vector3i(0, 1, 2)}));

    sx::RigidBodyOptions sphereOptions;
    sphereOptions.position = Eigen::Vector3d(0.0, 0.0, 0.2);
    auto sphere = world.addRigidBody("sphere", sphereOptions);
    sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.25));

    const std::vector<sx::Contact> contacts = world.collide();
    ASSERT_FALSE(contacts.empty());

    const dvbd::AvbdRigidWorldContactSnapshot snapshot
        = dvbd::buildAvbdRigidWorldContactSnapshot(
            dart::simulation::detail::registryOf(world),
            contacts,
            dvbd::AvbdRigidWorldContactOptions{});
    ASSERT_EQ(snapshot.contacts.size(), contacts.size());

    const entt::entity meshEntity
        = sx::detail::toRegistryEntity(mesh.getEntity());
    bool sawMeshFeature = false;
    for (std::size_t i = 0; i < contacts.size(); ++i) {
      const sx::Contact& contact = contacts[i];
      const bool meshIsA
          = sx::detail::toRegistryEntity(contact.bodyA.getEntity())
            == meshEntity;
      const bool meshIsB
          = sx::detail::toRegistryEntity(contact.bodyB.getEntity())
            == meshEntity;
      ASSERT_TRUE(meshIsA || meshIsB);

      const std::size_t shapeIndex
          = meshIsA ? contact.shapeIndexA : contact.shapeIndexB;
      const Eigen::Vector3d shapeLocalPoint
          = meshIsA ? contact.localPointA : contact.localPointB;
      ASSERT_EQ(shapeIndex, 0u);
      ASSERT_TRUE(shapeLocalPoint.allFinite());

      const auto& manifoldPoint = snapshot.contacts[i];
      const dvbd::AvbdContactEndpointId meshEndpoint
          = meshIsA ? manifoldPoint.endpointA : manifoldPoint.endpointB;
      const std::uint64_t expectedFeature = dvbd::packAvbdContactFeatureId(
          dvbd::AvbdContactFeatureKind::Face,
          dvbd::kAvbdRigidWorldMeshContactFeatureIdOffset);
      EXPECT_EQ(meshEndpoint.feature, expectedFeature);
      sawMeshFeature = true;
    }
    EXPECT_TRUE(sawMeshFeature);
  }

  {
    sx::World world;

    auto mesh = world.addRigidBody("mesh");
    mesh.setCollisionShape(
        sx::CollisionShape::makeMesh(
            {Eigen::Vector3d(-1.0, -1.0, 0.0),
             Eigen::Vector3d(1.0, -1.0, 0.0),
             Eigen::Vector3d(0.0, 1.0, 0.0)},
            {Eigen::Vector3i(0, 1, 2)}));

    sx::RigidBodyOptions sphereOptions;
    sphereOptions.position = Eigen::Vector3d(-1.10, -1.10, 0.10);
    auto sphere = world.addRigidBody("sphere", sphereOptions);
    sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.25));

    const std::vector<sx::Contact> contacts = world.collide();
    ASSERT_FALSE(contacts.empty());

    const dvbd::AvbdRigidWorldContactSnapshot snapshot
        = dvbd::buildAvbdRigidWorldContactSnapshot(
            dart::simulation::detail::registryOf(world),
            contacts,
            dvbd::AvbdRigidWorldContactOptions{});
    ASSERT_EQ(snapshot.contacts.size(), contacts.size());

    const entt::entity meshEntity
        = sx::detail::toRegistryEntity(mesh.getEntity());
    bool sawMeshVertexFeature = false;
    for (std::size_t i = 0; i < contacts.size(); ++i) {
      const sx::Contact& contact = contacts[i];
      const bool meshIsA
          = sx::detail::toRegistryEntity(contact.bodyA.getEntity())
            == meshEntity;
      const bool meshIsB
          = sx::detail::toRegistryEntity(contact.bodyB.getEntity())
            == meshEntity;
      ASSERT_TRUE(meshIsA || meshIsB);

      const std::size_t shapeIndex
          = meshIsA ? contact.shapeIndexA : contact.shapeIndexB;
      const Eigen::Vector3d shapeLocalPoint
          = meshIsA ? contact.localPointA : contact.localPointB;
      ASSERT_EQ(shapeIndex, 0u);
      ASSERT_TRUE(shapeLocalPoint.allFinite());
      EXPECT_NEAR(shapeLocalPoint.x(), -1.0, 1e-10);
      EXPECT_NEAR(shapeLocalPoint.y(), -1.0, 1e-10);
      EXPECT_NEAR(shapeLocalPoint.z(), 0.0, 1e-10);

      const auto& manifoldPoint = snapshot.contacts[i];
      const dvbd::AvbdContactEndpointId meshEndpoint
          = meshIsA ? manifoldPoint.endpointA : manifoldPoint.endpointB;
      const std::uint64_t expectedFeature = dvbd::packAvbdContactFeatureId(
          dvbd::AvbdContactFeatureKind::Vertex,
          dvbd::kAvbdRigidWorldMeshContactFeatureIdOffset);
      EXPECT_EQ(meshEndpoint.feature, expectedFeature);
      sawMeshVertexFeature = true;
    }
    EXPECT_TRUE(sawMeshVertexFeature);
  }
}

//==============================================================================
// Actual narrow-phase contact points must keep AVBD row ordinals attached to
// their canonical locations even when the input contact order changes.
TEST(AvbdContact, WorldCollideSameFeatureRowsIgnoreContactOrder)
{
  sx::World world;

  sx::RigidBodyOptions planeOptions;
  planeOptions.isStatic = true;
  auto plane = world.addRigidBody("plane", planeOptions);
  plane.setCollisionShape(
      sx::CollisionShape::makePlane(Eigen::Vector3d::UnitZ(), 0.0));

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.position = Eigen::Vector3d(-0.25, 0.0, 0.20);
  auto sphere = world.addRigidBody("sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.25));

  std::vector<sx::Contact> contacts;
  for (const Eigen::Vector3d& center :
       {Eigen::Vector3d(-0.25, 0.0, 0.20), Eigen::Vector3d(0.25, 0.0, 0.20)}) {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = center;
    sphere.setTransform(pose);
    const std::vector<sx::Contact> emittedContacts = world.collide();
    ASSERT_EQ(emittedContacts.size(), 1u);
    contacts.push_back(emittedContacts.front());
  }
  ASSERT_EQ(contacts.size(), 2u);

  const auto buildSnapshot = [&](std::vector<sx::Contact> orderedContacts) {
    return dvbd::buildAvbdRigidWorldContactSnapshot(
        dart::simulation::detail::registryOf(world),
        orderedContacts,
        dvbd::AvbdRigidWorldContactOptions{});
  };

  const dvbd::AvbdRigidWorldContactSnapshot forward = buildSnapshot(contacts);
  std::vector<sx::Contact> reversedContacts = contacts;
  std::reverse(reversedContacts.begin(), reversedContacts.end());
  const dvbd::AvbdRigidWorldContactSnapshot reversed
      = buildSnapshot(reversedContacts);

  ASSERT_EQ(forward.contacts.size(), contacts.size());
  ASSERT_EQ(reversed.contacts.size(), contacts.size());

  const auto rowKey = [](const dvbd::AvbdRigidContactManifoldPoint& contact) {
    return dvbd::canonicalizeAvbdContactEndpoints(
        contact.endpointA, contact.endpointB);
  };
  const auto expectedRowKey = rowKey(forward.contacts.front());
  const auto rowForPoint
      = [](const dvbd::AvbdRigidWorldContactSnapshot& snapshot,
           const Eigen::Vector3d& point) -> std::optional<std::uint32_t> {
    for (const dvbd::AvbdRigidContactManifoldPoint& contact :
         snapshot.contacts) {
      if ((contact.point - point).norm() <= 1e-10) {
        return contact.row;
      }
    }
    return std::nullopt;
  };

  std::vector<std::uint32_t> rows;
  rows.reserve(forward.contacts.size());
  for (const dvbd::AvbdRigidContactManifoldPoint& contact : forward.contacts) {
    EXPECT_EQ(rowKey(contact), expectedRowKey);
    const std::optional<std::uint32_t> reversedRow
        = rowForPoint(reversed, contact.point);
    ASSERT_TRUE(reversedRow.has_value());
    EXPECT_EQ(contact.row, *reversedRow);
    rows.push_back(contact.row);
  }

  std::sort(rows.begin(), rows.end());
  rows.erase(std::unique(rows.begin(), rows.end()), rows.end());
  EXPECT_GE(rows.size(), 2u);
}

//==============================================================================
// Friction tangent rows share the same endpoint-pair and same-feature row keys
// as the corresponding normal rows. A live sphere/plane replay verifies that
// contact-order changes preserve warm-started tangent duals by row key.
TEST(AvbdContact, WorldCollideFrictionRowsIgnoreContactOrder)
{
  sx::World world;

  sx::RigidBodyOptions planeOptions;
  planeOptions.isStatic = true;
  auto plane = world.addRigidBody("plane", planeOptions);
  plane.setCollisionShape(
      sx::CollisionShape::makePlane(Eigen::Vector3d::UnitZ(), 0.0));
  plane.setFriction(0.64);

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.position = Eigen::Vector3d(-0.25, 0.0, 0.20);
  auto sphere = world.addRigidBody("sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.25));
  sphere.setFriction(0.25);

  std::vector<sx::Contact> contacts;
  for (const Eigen::Vector3d& center :
       {Eigen::Vector3d(-0.25, 0.0, 0.20), Eigen::Vector3d(0.25, 0.0, 0.20)}) {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = center;
    sphere.setTransform(pose);
    const std::vector<sx::Contact> emittedContacts = world.collide();
    ASSERT_EQ(emittedContacts.size(), 1u);
    contacts.push_back(emittedContacts.front());
  }
  ASSERT_EQ(contacts.size(), 2u);

  const auto buildSnapshot = [&](std::vector<sx::Contact> orderedContacts) {
    return dvbd::buildAvbdRigidWorldContactSnapshot(
        dart::simulation::detail::registryOf(world),
        orderedContacts,
        dvbd::AvbdRigidWorldContactOptions{});
  };

  const dvbd::AvbdRigidWorldContactSnapshot forward = buildSnapshot(contacts);
  std::vector<sx::Contact> reversedContacts = contacts;
  std::reverse(reversedContacts.begin(), reversedContacts.end());
  const dvbd::AvbdRigidWorldContactSnapshot reversed
      = buildSnapshot(reversedContacts);

  ASSERT_EQ(forward.contacts.size(), contacts.size());
  ASSERT_EQ(reversed.contacts.size(), contacts.size());

  dvbd::AvbdScalarRowInventory normalInventory;
  dvbd::AvbdScalarRowInventory frictionInventory;
  std::vector<dvbd::AvbdRigidBodyPointPairRow> normalRows;
  std::vector<dvbd::AvbdRigidBodyPointPairFrictionRows> frictionRows;
  dvbd::AvbdRowWarmStartOptions warmStart;
  warmStart.alpha = 1.0;
  warmStart.gamma = 1.0;

  dvbd::buildAvbdRigidContactManifoldRows(
      forward.states,
      forward.contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      warmStart);

  ASSERT_EQ(normalInventory.size(), contacts.size());
  ASSERT_EQ(frictionInventory.size(), 2u * contacts.size());
  ASSERT_EQ(frictionRows.size(), contacts.size());

  const auto expectedNormalLambda = [](const dvbd::AvbdScalarRowKey& key) {
    return 20.0 + static_cast<double>(key.row);
  };
  const auto expectedFrictionLambda = [](const dvbd::AvbdScalarRowKey& key) {
    return 1.0 + 0.5 * static_cast<double>(key.row)
           + 0.125 * static_cast<double>(key.axis);
  };

  for (dvbd::AvbdScalarRowRecord& record : normalInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::ContactNormal);
    record.state.lambda = expectedNormalLambda(record.descriptor.key);
  }
  for (dvbd::AvbdScalarRowRecord& record : frictionInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::FrictionTangent);
    record.state.lambda = expectedFrictionLambda(record.descriptor.key);
  }

  dvbd::buildAvbdRigidContactManifoldRows(
      reversed.states,
      reversed.contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      warmStart);

  ASSERT_EQ(normalInventory.size(), contacts.size());
  ASSERT_EQ(frictionInventory.size(), 2u * contacts.size());
  ASSERT_EQ(frictionRows.size(), contacts.size());

  for (const dvbd::AvbdScalarRowRecord& record : normalInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::ContactNormal);
    EXPECT_DOUBLE_EQ(
        record.state.lambda, expectedNormalLambda(record.descriptor.key));
  }
  for (const dvbd::AvbdScalarRowRecord& record : frictionInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::FrictionTangent);
    const double forceLimit = 0.4 * expectedNormalLambda(record.descriptor.key);
    EXPECT_NEAR(record.descriptor.bounds.lower, -forceLimit, 1e-12);
    EXPECT_NEAR(record.descriptor.bounds.upper, forceLimit, 1e-12);
    EXPECT_DOUBLE_EQ(
        record.state.lambda, expectedFrictionLambda(record.descriptor.key));
  }
  for (std::size_t i = 0; i < frictionRows.size(); ++i) {
    EXPECT_DOUBLE_EQ(
        frictionRows[i].first.state.lambda,
        frictionInventory[2u * i].state.lambda);
    EXPECT_DOUBLE_EQ(
        frictionRows[i].second.state.lambda,
        frictionInventory[2u * i + 1u].state.lambda);
  }
}

//==============================================================================
// A live normal change should project persisted friction duals into the new
// tangent basis instead of treating the old tangent components as basis-local
// scalars.
TEST(AvbdContact, WorldCollideFrictionRowsProjectAcrossChangingPlaneNormal)
{
  sx::World world;

  sx::RigidBodyOptions planeOptions;
  planeOptions.isStatic = true;
  auto plane = world.addRigidBody("plane", planeOptions);
  plane.setCollisionShape(
      sx::CollisionShape::makePlane(Eigen::Vector3d::UnitZ(), 0.0));
  plane.setFriction(0.64);

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.position = Eigen::Vector3d(0.0, 0.0, 0.20);
  auto sphere = world.addRigidBody("sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.25));
  sphere.setFriction(0.25);

  const auto buildSnapshot
      = [&](const Eigen::Vector3d& planeNormal,
            dvbd::AvbdRigidWorldContactSnapshot& snapshot) {
          const Eigen::Vector3d normal = planeNormal.normalized();
          plane.setCollisionShape(sx::CollisionShape::makePlane(normal, 0.0));

          Eigen::Isometry3d spherePose = Eigen::Isometry3d::Identity();
          spherePose.translation() = 0.20 * normal;
          sphere.setTransform(spherePose);

          const std::vector<sx::Contact> contacts = world.collide();
          ASSERT_EQ(contacts.size(), 1u);

          snapshot = dvbd::buildAvbdRigidWorldContactSnapshot(
              dart::simulation::detail::registryOf(world),
              contacts,
              dvbd::AvbdRigidWorldContactOptions{});
          ASSERT_EQ(snapshot.contacts.size(), contacts.size());
        };

  dvbd::AvbdRigidWorldContactSnapshot reference;
  dvbd::AvbdRigidWorldContactSnapshot tilted;
  buildSnapshot(Eigen::Vector3d::UnitZ(), reference);
  buildSnapshot(Eigen::Vector3d(0.20, 0.10, 1.0), tilted);

  ASSERT_EQ(reference.contacts.size(), 1u);
  ASSERT_EQ(tilted.contacts.size(), 1u);
  EXPECT_LT(
      reference.contacts.front().normalFromAtoB.normalized().dot(
          tilted.contacts.front().normalFromAtoB.normalized()),
      0.99);

  const auto referenceKey = dvbd::canonicalizeAvbdContactEndpoints(
      reference.contacts.front().endpointA,
      reference.contacts.front().endpointB);
  const auto tiltedKey = dvbd::canonicalizeAvbdContactEndpoints(
      tilted.contacts.front().endpointA, tilted.contacts.front().endpointB);
  EXPECT_EQ(referenceKey, tiltedKey);
  EXPECT_EQ(reference.contacts.front().row, tilted.contacts.front().row);
  EXPECT_NEAR(reference.contacts.front().frictionCoefficient, 0.4, 1e-12);
  EXPECT_NEAR(tilted.contacts.front().frictionCoefficient, 0.4, 1e-12);

  dvbd::AvbdScalarRowInventory normalInventory;
  dvbd::AvbdScalarRowInventory frictionInventory;
  std::vector<dvbd::AvbdRigidBodyPointPairRow> normalRows;
  std::vector<dvbd::AvbdRigidBodyPointPairFrictionRows> frictionRows;
  dvbd::AvbdRowWarmStartOptions warmStart;
  warmStart.alpha = 1.0;
  warmStart.gamma = 1.0;

  dvbd::buildAvbdRigidContactManifoldRows(
      reference.states,
      reference.contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      warmStart);

  ASSERT_EQ(normalInventory.size(), 1u);
  ASSERT_EQ(frictionInventory.size(), 2u);
  ASSERT_EQ(frictionRows.size(), 1u);

  normalInventory[0].state.lambda = 12.0;
  frictionInventory[0].state.lambda = 2.0;
  frictionInventory[1].state.lambda = -1.25;
  const Eigen::Vector3d previousFirstDirection = frictionInventory[0].direction;
  const Eigen::Vector3d previousSecondDirection
      = frictionInventory[1].direction;
  const Eigen::Vector3d previousWorldDual
      = frictionInventory[0].state.lambda * previousFirstDirection
        + frictionInventory[1].state.lambda * previousSecondDirection;

  dvbd::buildAvbdRigidContactManifoldRows(
      tilted.states,
      tilted.contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      warmStart);

  ASSERT_EQ(normalInventory.size(), 1u);
  ASSERT_EQ(frictionInventory.size(), 2u);
  ASSERT_EQ(frictionRows.size(), 1u);
  EXPECT_NEAR(normalInventory[0].state.lambda, 12.0, 1e-12);

  for (const dvbd::AvbdScalarRowRecord& record : frictionInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::FrictionTangent);
    EXPECT_NEAR(record.descriptor.bounds.lower, -4.8, 1e-12);
    EXPECT_NEAR(record.descriptor.bounds.upper, 4.8, 1e-12);
    EXPECT_NEAR(
        record.state.lambda, previousWorldDual.dot(record.direction), 1e-12);
  }
  EXPECT_NEAR(
      frictionRows[0].first.state.lambda,
      frictionInventory[0].state.lambda,
      1e-12);
  EXPECT_NEAR(
      frictionRows[0].second.state.lambda,
      frictionInventory[1].state.lambda,
      1e-12);
  EXPECT_GT(
      std::max(
          std::abs(frictionInventory[0].state.lambda - 2.0),
          std::abs(frictionInventory[1].state.lambda + 1.25)),
      1e-3);
}

//==============================================================================
// Mesh face contacts use triangle-derived endpoint features, so replaying live
// sphere/mesh contacts on the same face checks same-feature row ordering beyond
// primitive half-spaces.
TEST(AvbdContact, WorldCollideMeshFaceRowsIgnoreContactOrder)
{
  expectMeshFeatureRowsIgnoreContactOrder(
      {Eigen::Vector3d(-0.25, 0.0, 0.20), Eigen::Vector3d(0.25, 0.0, 0.20)},
      dvbd::AvbdContactFeatureKind::Face,
      dvbd::kAvbdRigidWorldMeshContactFeatureIdOffset);
}

//==============================================================================
// Mesh face rows should also keep their row identity and tangent duals when the
// raw contact endpoints arrive swapped.
TEST(AvbdContact, WorldCollideMeshFaceRowsIgnoreEndpointOrder)
{
  expectMeshFeatureRowsIgnoreEndpointOrder(
      {Eigen::Vector3d(-0.25, 0.0, 0.20), Eigen::Vector3d(0.25, 0.0, 0.20)},
      dvbd::AvbdContactFeatureKind::Face,
      dvbd::kAvbdRigidWorldMeshContactFeatureIdOffset);
}

//==============================================================================
// Mesh edge contacts use packed vertex-pair endpoint features. Replaying live
// sphere/mesh contacts along the same triangle edge checks same-feature row
// ordering beyond mesh-face interior contacts.
TEST(AvbdContact, WorldCollideMeshEdgeRowsIgnoreContactOrder)
{
  const std::uint64_t edgeLocalIndex = 1u;
  expectMeshFeatureRowsIgnoreContactOrder(
      {Eigen::Vector3d(-0.25, -1.0, 0.20), Eigen::Vector3d(0.25, -1.0, 0.20)},
      dvbd::AvbdContactFeatureKind::Edge,
      dvbd::kAvbdRigidWorldMeshContactFeatureIdOffset + edgeLocalIndex);
}

//==============================================================================
// Mesh edge rows should also preserve their warm-started rows across endpoint
// swaps from the collision backend.
TEST(AvbdContact, WorldCollideMeshEdgeRowsIgnoreEndpointOrder)
{
  const std::uint64_t edgeLocalIndex = 1u;
  expectMeshFeatureRowsIgnoreEndpointOrder(
      {Eigen::Vector3d(-0.25, -1.0, 0.20), Eigen::Vector3d(0.25, -1.0, 0.20)},
      dvbd::AvbdContactFeatureKind::Edge,
      dvbd::kAvbdRigidWorldMeshContactFeatureIdOffset + edgeLocalIndex);
}

//==============================================================================
// Mesh face rows must persist across small pose changes that keep the contact
// points on the same triangle face.
TEST(AvbdContact, WorldCollideMeshFaceRowsPersistAcrossSmallPose)
{
  expectMeshFeatureRowsPersistAcrossSmallPose(
      {Eigen::Vector3d(-0.25, 0.0, 0.20), Eigen::Vector3d(0.25, 0.0, 0.20)},
      {Eigen::Vector3d(-0.2475, 0.0025, 0.20),
       Eigen::Vector3d(0.2475, -0.0025, 0.20)},
      dvbd::AvbdContactFeatureKind::Face,
      dvbd::kAvbdRigidWorldMeshContactFeatureIdOffset);
}

//==============================================================================
// Mesh edge rows must persist across small pose changes that keep the contact
// points on the same triangle edge.
TEST(AvbdContact, WorldCollideMeshEdgeRowsPersistAcrossSmallPose)
{
  const std::uint64_t edgeLocalIndex = 1u;
  expectMeshFeatureRowsPersistAcrossSmallPose(
      {Eigen::Vector3d(-0.25, -1.0, 0.20), Eigen::Vector3d(0.25, -1.0, 0.20)},
      {Eigen::Vector3d(-0.2475, -1.005, 0.20),
       Eigen::Vector3d(0.2475, -1.005, 0.20)},
      dvbd::AvbdContactFeatureKind::Edge,
      dvbd::kAvbdRigidWorldMeshContactFeatureIdOffset + edgeLocalIndex);
}

//==============================================================================
// Mesh vertex contacts use packed vertex endpoint features. Replaying live
// sphere/mesh contacts at each triangle vertex closes contact-order warm-start
// coverage for mesh vertex endpoint features.
TEST(AvbdContact, WorldCollideMeshVertexRowsIgnoreContactOrder)
{
  sx::World world;

  sx::RigidBodyOptions meshOptions;
  meshOptions.isStatic = true;
  auto mesh = world.addRigidBody("mesh", meshOptions);
  mesh.setCollisionShape(
      sx::CollisionShape::makeMesh(
          {Eigen::Vector3d(-1.0, -1.0, 0.0),
           Eigen::Vector3d(1.0, -1.0, 0.0),
           Eigen::Vector3d(0.0, 1.0, 0.0)},
          {Eigen::Vector3i(0, 1, 2)}));
  mesh.setFriction(0.64);

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.position = Eigen::Vector3d(-1.10, -1.10, 0.10);
  auto sphere = world.addRigidBody("sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.25));
  sphere.setFriction(0.25);

  const std::vector<Eigen::Vector3d> sphereCenters{
      Eigen::Vector3d(-1.10, -1.10, 0.10),
      Eigen::Vector3d(1.10, -1.10, 0.10),
      Eigen::Vector3d(0.0, 1.15, 0.10)};

  std::vector<sx::Contact> contacts;
  for (const Eigen::Vector3d& center : sphereCenters) {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = center;
    sphere.setTransform(pose);
    const std::vector<sx::Contact> emittedContacts = world.collide();
    ASSERT_FALSE(emittedContacts.empty());
    contacts.insert(
        contacts.end(), emittedContacts.begin(), emittedContacts.end());
  }
  ASSERT_GE(contacts.size(), sphereCenters.size());

  const auto buildSnapshot = [&](std::vector<sx::Contact> orderedContacts) {
    return dvbd::buildAvbdRigidWorldContactSnapshot(
        dart::simulation::detail::registryOf(world),
        orderedContacts,
        dvbd::AvbdRigidWorldContactOptions{});
  };

  const dvbd::AvbdRigidWorldContactSnapshot forward = buildSnapshot(contacts);
  std::vector<sx::Contact> reversedContacts = contacts;
  std::reverse(reversedContacts.begin(), reversedContacts.end());
  const dvbd::AvbdRigidWorldContactSnapshot reversed
      = buildSnapshot(reversedContacts);

  ASSERT_EQ(forward.contacts.size(), contacts.size());
  ASSERT_EQ(reversed.contacts.size(), contacts.size());

  using RowKey
      = std::pair<dvbd::AvbdContactEndpointId, dvbd::AvbdContactEndpointId>;

  const auto rowKey = [](const dvbd::AvbdRigidContactManifoldPoint& contact) {
    return dvbd::canonicalizeAvbdContactEndpoints(
        contact.endpointA, contact.endpointB);
  };
  const auto reversedContactForKey
      = [&](const RowKey& key) -> const dvbd::AvbdRigidContactManifoldPoint* {
    for (const dvbd::AvbdRigidContactManifoldPoint& contact :
         reversed.contacts) {
      if (rowKey(contact) == key) {
        return &contact;
      }
    }
    return nullptr;
  };

  const std::uint64_t meshObject = dvbd::avbdRigidWorldContactObjectId(
      sx::detail::toRegistryEntity(mesh.getEntity()));
  std::vector<std::uint64_t> meshFeatureLocalIndices;
  meshFeatureLocalIndices.reserve(forward.contacts.size());
  for (const dvbd::AvbdRigidContactManifoldPoint& contact : forward.contacts) {
    const dvbd::AvbdContactEndpointId meshEndpoint
        = contact.endpointA.object == meshObject ? contact.endpointA
                                                 : contact.endpointB;
    ASSERT_EQ(meshEndpoint.object, meshObject);
    EXPECT_EQ(
        dvbd::avbdContactFeatureKind(meshEndpoint.feature),
        dvbd::AvbdContactFeatureKind::Vertex);
    meshFeatureLocalIndices.push_back(
        dvbd::avbdContactFeatureLocalIndex(meshEndpoint.feature));
    EXPECT_NEAR(contact.frictionCoefficient, 0.4, 1e-12);

    const dvbd::AvbdRigidContactManifoldPoint* reversedContact
        = reversedContactForKey(rowKey(contact));
    ASSERT_NE(reversedContact, nullptr);
    EXPECT_EQ(contact.row, reversedContact->row);
    EXPECT_NEAR(
        contact.frictionCoefficient,
        reversedContact->frictionCoefficient,
        1e-12);
  }

  std::sort(meshFeatureLocalIndices.begin(), meshFeatureLocalIndices.end());
  meshFeatureLocalIndices.erase(
      std::unique(
          meshFeatureLocalIndices.begin(), meshFeatureLocalIndices.end()),
      meshFeatureLocalIndices.end());
  ASSERT_EQ(meshFeatureLocalIndices.size(), 3u);
  for (std::size_t vertex = 0; vertex < 3u; ++vertex) {
    EXPECT_EQ(
        meshFeatureLocalIndices[vertex],
        dvbd::kAvbdRigidWorldMeshContactFeatureIdOffset + vertex);
  }

  dvbd::AvbdScalarRowInventory normalInventory;
  dvbd::AvbdScalarRowInventory frictionInventory;
  std::vector<dvbd::AvbdRigidBodyPointPairRow> normalRows;
  std::vector<dvbd::AvbdRigidBodyPointPairFrictionRows> frictionRows;
  dvbd::AvbdRowWarmStartOptions warmStart;
  warmStart.alpha = 1.0;
  warmStart.gamma = 1.0;

  dvbd::buildAvbdRigidContactManifoldRows(
      forward.states,
      forward.contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      warmStart);

  ASSERT_EQ(normalInventory.size(), forward.contacts.size());
  ASSERT_EQ(frictionInventory.size(), 2u * forward.contacts.size());
  ASSERT_EQ(frictionRows.size(), forward.contacts.size());

  const auto keyFingerprint = [](const dvbd::AvbdScalarRowKey& key) {
    return static_cast<double>(
        key.objectA % 17u + 2u * (key.objectB % 19u) + 3u * (key.featureA % 23u)
        + 5u * (key.featureB % 29u));
  };
  const auto expectedNormalLambda = [&](const dvbd::AvbdScalarRowKey& key) {
    return 15.0 + static_cast<double>(key.row) + 0.01 * keyFingerprint(key);
  };
  const auto expectedFrictionLambda = [&](const dvbd::AvbdScalarRowKey& key) {
    return 2.0 + 0.5 * static_cast<double>(key.row)
           + 0.125 * static_cast<double>(key.axis) + 0.01 * keyFingerprint(key);
  };

  for (dvbd::AvbdScalarRowRecord& record : normalInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::ContactNormal);
    record.state.lambda = expectedNormalLambda(record.descriptor.key);
  }
  for (dvbd::AvbdScalarRowRecord& record : frictionInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::FrictionTangent);
    record.state.lambda = expectedFrictionLambda(record.descriptor.key);
  }

  dvbd::buildAvbdRigidContactManifoldRows(
      reversed.states,
      reversed.contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      warmStart);

  ASSERT_EQ(normalInventory.size(), reversed.contacts.size());
  ASSERT_EQ(frictionInventory.size(), 2u * reversed.contacts.size());
  ASSERT_EQ(frictionRows.size(), reversed.contacts.size());

  for (const dvbd::AvbdScalarRowRecord& record : normalInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::ContactNormal);
    EXPECT_NEAR(
        record.state.lambda,
        expectedNormalLambda(record.descriptor.key),
        1e-12);
  }
  for (const dvbd::AvbdScalarRowRecord& record : frictionInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::FrictionTangent);
    const double forceLimit = 0.4 * expectedNormalLambda(record.descriptor.key);
    EXPECT_NEAR(record.descriptor.bounds.lower, -forceLimit, 1e-12);
    EXPECT_NEAR(record.descriptor.bounds.upper, forceLimit, 1e-12);
    EXPECT_NEAR(
        record.state.lambda,
        expectedFrictionLambda(record.descriptor.key),
        1e-12);
  }
  for (std::size_t i = 0; i < frictionRows.size(); ++i) {
    EXPECT_NEAR(
        frictionRows[i].first.state.lambda,
        frictionInventory[2u * i].state.lambda,
        1e-12);
    EXPECT_NEAR(
        frictionRows[i].second.state.lambda,
        frictionInventory[2u * i + 1u].state.lambda,
        1e-12);
  }
}

//==============================================================================
// Mesh vertex endpoint features should also keep their row keys and warm starts
// when the raw sphere/mesh endpoint order is swapped.
TEST(AvbdContact, WorldCollideMeshVertexRowsIgnoreEndpointOrder)
{
  expectMeshFeatureRowsIgnoreEndpointOrder(
      {Eigen::Vector3d(-1.10, -1.10, 0.10),
       Eigen::Vector3d(-1.12, -1.08, 0.10)},
      dvbd::AvbdContactFeatureKind::Vertex,
      dvbd::kAvbdRigidWorldMeshContactFeatureIdOffset);
  expectMeshFeatureRowsIgnoreEndpointOrder(
      {Eigen::Vector3d(1.10, -1.10, 0.10), Eigen::Vector3d(1.12, -1.08, 0.10)},
      dvbd::AvbdContactFeatureKind::Vertex,
      dvbd::kAvbdRigidWorldMeshContactFeatureIdOffset + 1u);
  expectMeshFeatureRowsIgnoreEndpointOrder(
      {Eigen::Vector3d(0.0, 1.15, 0.10), Eigen::Vector3d(0.02, 1.14, 0.10)},
      dvbd::AvbdContactFeatureKind::Vertex,
      dvbd::kAvbdRigidWorldMeshContactFeatureIdOffset + 2u);
}

//==============================================================================
// Mesh vertex endpoint features must also keep their AVBD rows across small
// pose changes that preserve the active vertex feature at each contact.
TEST(AvbdContact, WorldCollideMeshVertexRowsPersistAcrossSmallPose)
{
  sx::World world;

  sx::RigidBodyOptions meshOptions;
  meshOptions.isStatic = true;
  auto mesh = world.addRigidBody("mesh", meshOptions);
  mesh.setCollisionShape(
      sx::CollisionShape::makeMesh(
          {Eigen::Vector3d(-1.0, -1.0, 0.0),
           Eigen::Vector3d(1.0, -1.0, 0.0),
           Eigen::Vector3d(0.0, 1.0, 0.0)},
          {Eigen::Vector3i(0, 1, 2)}));
  mesh.setFriction(0.64);

  const std::vector<Eigen::Vector3d> referenceCenters{
      Eigen::Vector3d(-1.10, -1.10, 0.10),
      Eigen::Vector3d(1.10, -1.10, 0.10),
      Eigen::Vector3d(0.0, 1.15, 0.10)};
  const std::vector<Eigen::Vector3d> nudgedCenters{
      Eigen::Vector3d(-1.105, -1.095, 0.10),
      Eigen::Vector3d(1.105, -1.095, 0.10),
      Eigen::Vector3d(0.005, 1.145, 0.10)};

  std::vector<sx::RigidBody> spheres;
  spheres.reserve(referenceCenters.size());
  for (std::size_t i = 0; i < referenceCenters.size(); ++i) {
    sx::RigidBodyOptions sphereOptions;
    sphereOptions.position = referenceCenters[i];
    std::ostringstream name;
    name << "sphere_" << i;
    auto sphere = world.addRigidBody(name.str(), sphereOptions);
    sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.25));
    sphere.setFriction(0.25);
    spheres.push_back(sphere);
  }

  const auto buildSnapshot
      = [&](const std::vector<Eigen::Vector3d>& centers,
            dvbd::AvbdRigidWorldContactSnapshot& snapshot) {
          ASSERT_EQ(centers.size(), spheres.size());
          for (std::size_t i = 0; i < centers.size(); ++i) {
            Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
            pose.translation() = centers[i];
            spheres[i].setTransform(pose);
          }

          const std::vector<sx::Contact> contacts = world.collide();
          ASSERT_GE(contacts.size(), centers.size());

          snapshot = dvbd::buildAvbdRigidWorldContactSnapshot(
              dart::simulation::detail::registryOf(world),
              contacts,
              dvbd::AvbdRigidWorldContactOptions{});
          ASSERT_EQ(snapshot.contacts.size(), contacts.size());
        };

  dvbd::AvbdRigidWorldContactSnapshot reference;
  dvbd::AvbdRigidWorldContactSnapshot nudged;
  buildSnapshot(referenceCenters, reference);
  buildSnapshot(nudgedCenters, nudged);
  ASSERT_EQ(reference.contacts.size(), nudged.contacts.size());

  using RowKey
      = std::pair<dvbd::AvbdContactEndpointId, dvbd::AvbdContactEndpointId>;
  struct ObservedRow
  {
    RowKey key;
    std::uint32_t row = 0u;
    std::uint64_t meshFeatureLocalIndex = 0u;
    double frictionCoefficient = 0.0;
  };

  const auto rowKey = [](const dvbd::AvbdRigidContactManifoldPoint& contact) {
    return dvbd::canonicalizeAvbdContactEndpoints(
        contact.endpointA, contact.endpointB);
  };

  const std::uint64_t meshObject = dvbd::avbdRigidWorldContactObjectId(
      sx::detail::toRegistryEntity(mesh.getEntity()));
  const auto collectRows
      = [&](const dvbd::AvbdRigidWorldContactSnapshot& snapshot) {
          std::vector<ObservedRow> rows;
          rows.reserve(snapshot.contacts.size());
          for (const dvbd::AvbdRigidContactManifoldPoint& contact :
               snapshot.contacts) {
            const dvbd::AvbdContactEndpointId meshEndpoint
                = contact.endpointA.object == meshObject ? contact.endpointA
                                                         : contact.endpointB;
            EXPECT_EQ(meshEndpoint.object, meshObject);
            if (meshEndpoint.object != meshObject) {
              continue;
            }
            EXPECT_EQ(
                dvbd::avbdContactFeatureKind(meshEndpoint.feature),
                dvbd::AvbdContactFeatureKind::Vertex);
            EXPECT_NEAR(contact.frictionCoefficient, 0.4, 1e-12);
            rows.push_back(
                ObservedRow{
                    rowKey(contact),
                    contact.row,
                    dvbd::avbdContactFeatureLocalIndex(meshEndpoint.feature),
                    contact.frictionCoefficient});
          }
          return rows;
        };
  const auto observedAtRow
      = [](const std::vector<ObservedRow>& rows,
           const RowKey& key,
           std::uint32_t row) -> std::optional<ObservedRow> {
    for (const ObservedRow& observed : rows) {
      if (observed.key == key && observed.row == row) {
        return observed;
      }
    }
    return std::nullopt;
  };

  const std::vector<ObservedRow> referenceRows = collectRows(reference);
  const std::vector<ObservedRow> nudgedRows = collectRows(nudged);
  ASSERT_EQ(referenceRows.size(), nudgedRows.size());

  std::vector<std::uint64_t> meshFeatureLocalIndices;
  meshFeatureLocalIndices.reserve(referenceRows.size());
  for (const ObservedRow& referenceRow : referenceRows) {
    const std::optional<ObservedRow> nudgedRow
        = observedAtRow(nudgedRows, referenceRow.key, referenceRow.row);
    ASSERT_TRUE(nudgedRow.has_value());
    EXPECT_EQ(
        referenceRow.meshFeatureLocalIndex, nudgedRow->meshFeatureLocalIndex);
    EXPECT_NEAR(
        referenceRow.frictionCoefficient,
        nudgedRow->frictionCoefficient,
        1e-12);
    meshFeatureLocalIndices.push_back(referenceRow.meshFeatureLocalIndex);
  }

  std::sort(meshFeatureLocalIndices.begin(), meshFeatureLocalIndices.end());
  meshFeatureLocalIndices.erase(
      std::unique(
          meshFeatureLocalIndices.begin(), meshFeatureLocalIndices.end()),
      meshFeatureLocalIndices.end());
  ASSERT_EQ(meshFeatureLocalIndices.size(), 3u);
  for (std::size_t vertex = 0; vertex < 3u; ++vertex) {
    EXPECT_EQ(
        meshFeatureLocalIndices[vertex],
        dvbd::kAvbdRigidWorldMeshContactFeatureIdOffset + vertex);
  }

  dvbd::AvbdScalarRowInventory normalInventory;
  dvbd::AvbdScalarRowInventory frictionInventory;
  std::vector<dvbd::AvbdRigidBodyPointPairRow> normalRows;
  std::vector<dvbd::AvbdRigidBodyPointPairFrictionRows> frictionRows;
  dvbd::AvbdRowWarmStartOptions warmStart;
  warmStart.alpha = 1.0;
  warmStart.gamma = 1.0;

  dvbd::buildAvbdRigidContactManifoldRows(
      reference.states,
      reference.contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      warmStart);

  ASSERT_EQ(normalInventory.size(), reference.contacts.size());
  ASSERT_EQ(frictionInventory.size(), 2u * reference.contacts.size());
  ASSERT_EQ(frictionRows.size(), reference.contacts.size());

  const auto keyFingerprint = [](const dvbd::AvbdScalarRowKey& key) {
    return static_cast<double>(
        key.objectA % 17u + 2u * (key.objectB % 19u) + 3u * (key.featureA % 23u)
        + 5u * (key.featureB % 29u));
  };
  const auto expectedNormalLambda = [&](const dvbd::AvbdScalarRowKey& key) {
    return 17.0 + static_cast<double>(key.row) + 0.01 * keyFingerprint(key);
  };
  const auto expectedFrictionLambda = [&](const dvbd::AvbdScalarRowKey& key) {
    return 3.0 + 0.5 * static_cast<double>(key.row)
           + 0.125 * static_cast<double>(key.axis) + 0.01 * keyFingerprint(key);
  };

  for (dvbd::AvbdScalarRowRecord& record : normalInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::ContactNormal);
    record.state.lambda = expectedNormalLambda(record.descriptor.key);
  }
  for (dvbd::AvbdScalarRowRecord& record : frictionInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::FrictionTangent);
    record.state.lambda = expectedFrictionLambda(record.descriptor.key);
  }

  struct PreviousFrictionPair
  {
    dvbd::AvbdScalarRowKey key;
    Eigen::Vector3d worldDual = Eigen::Vector3d::Zero();
  };
  const auto sameFrictionPair = [](const dvbd::AvbdScalarRowKey& lhs,
                                   const dvbd::AvbdScalarRowKey& rhs) {
    return lhs.role == rhs.role && lhs.objectA == rhs.objectA
           && lhs.objectB == rhs.objectB && lhs.featureA == rhs.featureA
           && lhs.featureB == rhs.featureB && lhs.row == rhs.row;
  };
  std::vector<PreviousFrictionPair> previousFrictionPairs;
  for (const dvbd::AvbdScalarRowRecord& record : frictionInventory.records()) {
    auto pairIt = std::find_if(
        previousFrictionPairs.begin(),
        previousFrictionPairs.end(),
        [&](const PreviousFrictionPair& pair) {
          return sameFrictionPair(pair.key, record.descriptor.key);
        });
    if (pairIt == previousFrictionPairs.end()) {
      previousFrictionPairs.push_back(
          PreviousFrictionPair{
              record.descriptor.key, record.state.lambda * record.direction});
    } else {
      pairIt->worldDual += record.state.lambda * record.direction;
    }
  }
  const auto previousFrictionPairForKey = [&](const dvbd::AvbdScalarRowKey& key)
      -> std::optional<PreviousFrictionPair> {
    for (const PreviousFrictionPair& pair : previousFrictionPairs) {
      if (sameFrictionPair(pair.key, key)) {
        return pair;
      }
    }
    return std::nullopt;
  };

  dvbd::buildAvbdRigidContactManifoldRows(
      nudged.states,
      nudged.contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      warmStart);

  ASSERT_EQ(normalInventory.size(), nudged.contacts.size());
  ASSERT_EQ(frictionInventory.size(), 2u * nudged.contacts.size());
  ASSERT_EQ(frictionRows.size(), nudged.contacts.size());

  for (const dvbd::AvbdScalarRowRecord& record : normalInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::ContactNormal);
    EXPECT_NEAR(
        record.state.lambda,
        expectedNormalLambda(record.descriptor.key),
        1e-12);
  }
  for (const dvbd::AvbdScalarRowRecord& record : frictionInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::FrictionTangent);
    const double forceLimit = 0.4 * expectedNormalLambda(record.descriptor.key);
    EXPECT_NEAR(record.descriptor.bounds.lower, -forceLimit, 1e-12);
    EXPECT_NEAR(record.descriptor.bounds.upper, forceLimit, 1e-12);
    const std::optional<PreviousFrictionPair> previousPair
        = previousFrictionPairForKey(record.descriptor.key);
    ASSERT_TRUE(previousPair.has_value());
    EXPECT_NEAR(
        record.state.lambda,
        previousPair->worldDual.dot(record.direction),
        1e-12);
  }
  for (std::size_t i = 0; i < frictionRows.size(); ++i) {
    EXPECT_NEAR(
        frictionRows[i].first.state.lambda,
        frictionInventory[2u * i].state.lambda,
        1e-12);
    EXPECT_NEAR(
        frictionRows[i].second.state.lambda,
        frictionInventory[2u * i + 1u].state.lambda,
        1e-12);
  }
}

//==============================================================================
// The same actual narrow-phase contacts must keep their canonical AVBD row
// ordinals when a collision backend reports the two bodies in the opposite
// endpoint order.
TEST(AvbdContact, WorldCollideSameFeatureRowsIgnoreEndpointOrder)
{
  sx::World world;

  sx::RigidBodyOptions planeOptions;
  planeOptions.isStatic = true;
  auto plane = world.addRigidBody("plane", planeOptions);
  plane.setCollisionShape(
      sx::CollisionShape::makePlane(Eigen::Vector3d::UnitZ(), 0.0));

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.position = Eigen::Vector3d(-0.25, 0.0, 0.20);
  auto sphere = world.addRigidBody("sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.25));

  std::vector<sx::Contact> contacts;
  for (const Eigen::Vector3d& center :
       {Eigen::Vector3d(-0.25, 0.0, 0.20), Eigen::Vector3d(0.25, 0.0, 0.20)}) {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = center;
    sphere.setTransform(pose);
    const std::vector<sx::Contact> emittedContacts = world.collide();
    ASSERT_EQ(emittedContacts.size(), 1u);
    contacts.push_back(emittedContacts.front());
  }
  ASSERT_EQ(contacts.size(), 2u);

  std::vector<sx::Contact> swappedContacts;
  swappedContacts.reserve(contacts.size());
  for (const sx::Contact& contact : contacts) {
    swappedContacts.push_back(swapEndpointOrder(contact));
  }

  const auto buildSnapshot = [&](std::vector<sx::Contact> orderedContacts) {
    return dvbd::buildAvbdRigidWorldContactSnapshot(
        dart::simulation::detail::registryOf(world),
        orderedContacts,
        dvbd::AvbdRigidWorldContactOptions{});
  };

  const dvbd::AvbdRigidWorldContactSnapshot forward = buildSnapshot(contacts);
  const dvbd::AvbdRigidWorldContactSnapshot swapped
      = buildSnapshot(swappedContacts);

  ASSERT_EQ(forward.contacts.size(), contacts.size());
  ASSERT_EQ(swapped.contacts.size(), contacts.size());

  using RowKey
      = std::pair<dvbd::AvbdContactEndpointId, dvbd::AvbdContactEndpointId>;
  struct ObservedRow
  {
    RowKey key;
    std::uint32_t row = 0u;
  };

  const auto rowKey = [](const dvbd::AvbdRigidContactManifoldPoint& contact) {
    return dvbd::canonicalizeAvbdContactEndpoints(
        contact.endpointA, contact.endpointB);
  };
  const auto observedAtPoint
      = [&](const dvbd::AvbdRigidWorldContactSnapshot& snapshot,
            const Eigen::Vector3d& point) -> std::optional<ObservedRow> {
    for (const dvbd::AvbdRigidContactManifoldPoint& contact :
         snapshot.contacts) {
      if ((contact.point - point).norm() <= 1e-10) {
        return ObservedRow{rowKey(contact), contact.row};
      }
    }
    return std::nullopt;
  };

  std::vector<std::uint32_t> rows;
  rows.reserve(forward.contacts.size());
  for (const dvbd::AvbdRigidContactManifoldPoint& contact : forward.contacts) {
    const std::optional<ObservedRow> swappedObservation
        = observedAtPoint(swapped, contact.point);
    ASSERT_TRUE(swappedObservation.has_value());
    EXPECT_EQ(rowKey(contact), swappedObservation->key);
    EXPECT_EQ(contact.row, swappedObservation->row);
    rows.push_back(contact.row);
  }

  std::sort(rows.begin(), rows.end());
  rows.erase(std::unique(rows.begin(), rows.end()), rows.end());
  EXPECT_GE(rows.size(), 2u);
}

//==============================================================================
// Curved primitive contacts should get the same deterministic same-feature row
// ordering as sphere/plane and box manifolds. Replaying a live cylinder cap
// against a plane at multiple positions gives multiple rows for the same
// canonical face-pair without relying on synthetic contact points.
TEST(AvbdContact, WorldCollideCylinderCapRowsIgnoreContactOrder)
{
  const sx::CollisionShape cylinderShape
      = sx::CollisionShape::makeCylinder(/*radius=*/0.25, /*halfHeight=*/0.5);
  expectPrimitiveCapRowsIgnoreContactOrder(
      cylinderShape,
      Eigen::Vector3d::UnitZ(),
      0.45,
      dvbd::packAvbdCylinderContactFeatureId(0, 2u));
  expectPrimitiveCapRowsIgnoreContactOrder(
      cylinderShape,
      -Eigen::Vector3d::UnitZ(),
      -0.45,
      dvbd::packAvbdCylinderContactFeatureId(0, 1u));
}

//==============================================================================
// Cylinder cap rows should keep their feature-coded keys and warm-started duals
// across small same-cap pose changes.
TEST(AvbdContact, WorldCollideCylinderCapRowsPersistAcrossSmallPose)
{
  const sx::CollisionShape cylinderShape
      = sx::CollisionShape::makeCylinder(/*radius=*/0.25, /*halfHeight=*/0.5);
  expectPlanePrimitiveRowsPersistAcrossSmallPose(
      cylinderShape,
      Eigen::Vector3d::UnitZ(),
      0.45,
      dvbd::packAvbdCylinderContactFeatureId(0, 2u));
  expectPlanePrimitiveRowsPersistAcrossSmallPose(
      cylinderShape,
      -Eigen::Vector3d::UnitZ(),
      -0.45,
      dvbd::packAvbdCylinderContactFeatureId(0, 1u));
}

//==============================================================================
// Cylinder cap rows should also preserve row identity and warm-started duals
// when the raw contact endpoints arrive swapped.
TEST(AvbdContact, WorldCollideCylinderCapRowsIgnoreEndpointOrder)
{
  const sx::CollisionShape cylinderShape
      = sx::CollisionShape::makeCylinder(/*radius=*/0.25, /*halfHeight=*/0.5);
  expectPlanePrimitiveRowsIgnoreEndpointOrder(
      cylinderShape,
      Eigen::Vector3d::UnitZ(),
      0.45,
      dvbd::packAvbdCylinderContactFeatureId(0, 2u));
  expectPlanePrimitiveRowsIgnoreEndpointOrder(
      cylinderShape,
      -Eigen::Vector3d::UnitZ(),
      -0.45,
      dvbd::packAvbdCylinderContactFeatureId(0, 1u));
}

//==============================================================================
// Cylinder side contacts should also get stable same-feature row ordering when
// replaying real sphere-side contacts in reverse order.
TEST(AvbdContact, WorldCollideCylinderSideRowsIgnoreContactOrder)
{
  expectSphereSideRowsIgnoreContactOrder(
      sx::CollisionShape::makeCylinder(/*radius=*/0.25, /*halfHeight=*/0.5),
      dvbd::packAvbdCylinderContactFeatureId(0, 0u));
}

//==============================================================================
// Cylinder side rows should also persist across small same-side sphere pose
// changes that rotate the local tangent basis.
TEST(AvbdContact, WorldCollideCylinderSideRowsPersistAcrossSmallPose)
{
  expectSpherePrimitiveRowsPersistAcrossSmallPose(
      sx::CollisionShape::makeCylinder(/*radius=*/0.25, /*halfHeight=*/0.5),
      {Eigen::Vector3d(0.45, -0.10, -0.20), Eigen::Vector3d(0.45, 0.10, 0.20)},
      {Eigen::Vector3d(0.4525, -0.0975, -0.1975),
       Eigen::Vector3d(0.4525, 0.0975, 0.1975)},
      dvbd::AvbdContactFeatureKind::Face,
      dvbd::packAvbdCylinderContactFeatureId(0, 0u));
}

//==============================================================================
// Cylinder side rows should also survive endpoint-swapped raw contacts.
TEST(AvbdContact, WorldCollideCylinderSideRowsIgnoreEndpointOrder)
{
  expectSpherePrimitiveRowsIgnoreEndpointOrder(
      sx::CollisionShape::makeCylinder(/*radius=*/0.25, /*halfHeight=*/0.5),
      {Eigen::Vector3d(0.45, -0.10, -0.20), Eigen::Vector3d(0.45, 0.10, 0.20)},
      dvbd::AvbdContactFeatureKind::Face,
      dvbd::packAvbdCylinderContactFeatureId(0, 0u));
}

//==============================================================================
// Cylinder rim contacts are edge features rather than face patches. Live sphere
// contacts on the same top or bottom rim should still get deterministic row
// ordering.
TEST(AvbdContact, WorldCollideCylinderRimRowsIgnoreContactOrder)
{
  expectCylinderRimRowsIgnoreContactOrder(0.66, 3u);
  expectCylinderRimRowsIgnoreContactOrder(-0.66, 4u);
}

//==============================================================================
// Cylinder rim rows should also persist across small same-rim pose changes.
TEST(AvbdContact, WorldCollideCylinderRimRowsPersistAcrossSmallPose)
{
  expectSpherePrimitiveRowsPersistAcrossSmallPose(
      sx::CollisionShape::makeCylinder(/*radius=*/0.25, /*halfHeight=*/0.5),
      {Eigen::Vector3d(0.42, -0.07, 0.66), Eigen::Vector3d(0.42, 0.07, 0.66)},
      {Eigen::Vector3d(0.4225, -0.0675, 0.66),
       Eigen::Vector3d(0.4225, 0.0675, 0.66)},
      dvbd::AvbdContactFeatureKind::Edge,
      dvbd::packAvbdCylinderContactFeatureId(0, 3u));
  expectSpherePrimitiveRowsPersistAcrossSmallPose(
      sx::CollisionShape::makeCylinder(/*radius=*/0.25, /*halfHeight=*/0.5),
      {Eigen::Vector3d(0.42, -0.07, -0.66), Eigen::Vector3d(0.42, 0.07, -0.66)},
      {Eigen::Vector3d(0.4225, -0.0675, -0.66),
       Eigen::Vector3d(0.4225, 0.0675, -0.66)},
      dvbd::AvbdContactFeatureKind::Edge,
      dvbd::packAvbdCylinderContactFeatureId(0, 4u));
}

//==============================================================================
// Cylinder rim rows are edge-keyed and should also be endpoint-order stable.
TEST(AvbdContact, WorldCollideCylinderRimRowsIgnoreEndpointOrder)
{
  expectSpherePrimitiveRowsIgnoreEndpointOrder(
      sx::CollisionShape::makeCylinder(/*radius=*/0.25, /*halfHeight=*/0.5),
      {Eigen::Vector3d(0.42, -0.07, 0.66), Eigen::Vector3d(0.42, 0.07, 0.66)},
      dvbd::AvbdContactFeatureKind::Edge,
      dvbd::packAvbdCylinderContactFeatureId(0, 3u));
  expectSpherePrimitiveRowsIgnoreEndpointOrder(
      sx::CollisionShape::makeCylinder(/*radius=*/0.25, /*halfHeight=*/0.5),
      {Eigen::Vector3d(0.42, -0.07, -0.66), Eigen::Vector3d(0.42, 0.07, -0.66)},
      dvbd::AvbdContactFeatureKind::Edge,
      dvbd::packAvbdCylinderContactFeatureId(0, 4u));
}

//==============================================================================
// Capsule caps are curved primitive patches like cylinder caps but use a
// different feature encoder. Replaying a live capsule cap against a plane
// verifies same-feature row ordering on that patch stays independent of the
// emitted contact order.
TEST(AvbdContact, WorldCollideCapsuleCapRowsIgnoreContactOrder)
{
  const sx::CollisionShape capsuleShape
      = sx::CollisionShape::makeCapsule(/*radius=*/0.25, /*halfHeight=*/0.5);
  expectPrimitiveCapRowsIgnoreContactOrder(
      capsuleShape,
      Eigen::Vector3d::UnitZ(),
      0.7,
      dvbd::packAvbdCapsuleContactFeatureId(0, 2u));
  expectPrimitiveCapRowsIgnoreContactOrder(
      capsuleShape,
      -Eigen::Vector3d::UnitZ(),
      -0.7,
      dvbd::packAvbdCapsuleContactFeatureId(0, 1u));
}

//==============================================================================
// Capsule cap rows should keep their capsule feature namespace and warm starts
// across small same-cap pose changes.
TEST(AvbdContact, WorldCollideCapsuleCapRowsPersistAcrossSmallPose)
{
  const sx::CollisionShape capsuleShape
      = sx::CollisionShape::makeCapsule(/*radius=*/0.25, /*halfHeight=*/0.5);
  expectPlanePrimitiveRowsPersistAcrossSmallPose(
      capsuleShape,
      Eigen::Vector3d::UnitZ(),
      0.7,
      dvbd::packAvbdCapsuleContactFeatureId(0, 2u));
  expectPlanePrimitiveRowsPersistAcrossSmallPose(
      capsuleShape,
      -Eigen::Vector3d::UnitZ(),
      -0.7,
      dvbd::packAvbdCapsuleContactFeatureId(0, 1u));
}

//==============================================================================
// Capsule cap rows should also survive endpoint-swapped raw contacts.
TEST(AvbdContact, WorldCollideCapsuleCapRowsIgnoreEndpointOrder)
{
  const sx::CollisionShape capsuleShape
      = sx::CollisionShape::makeCapsule(/*radius=*/0.25, /*halfHeight=*/0.5);
  expectPlanePrimitiveRowsIgnoreEndpointOrder(
      capsuleShape,
      Eigen::Vector3d::UnitZ(),
      0.7,
      dvbd::packAvbdCapsuleContactFeatureId(0, 2u));
  expectPlanePrimitiveRowsIgnoreEndpointOrder(
      capsuleShape,
      -Eigen::Vector3d::UnitZ(),
      -0.7,
      dvbd::packAvbdCapsuleContactFeatureId(0, 1u));
}

//==============================================================================
// Capsule side contacts use a separate feature code from the spherical caps.
// Replaying live side contacts in reverse order guards that patch as well.
TEST(AvbdContact, WorldCollideCapsuleSideRowsIgnoreContactOrder)
{
  expectSphereSideRowsIgnoreContactOrder(
      sx::CollisionShape::makeCapsule(/*radius=*/0.25, /*halfHeight=*/0.5),
      dvbd::packAvbdCapsuleContactFeatureId(0, 0u));
}

//==============================================================================
// Capsule side rows should persist across small same-side sphere pose changes.
TEST(AvbdContact, WorldCollideCapsuleSideRowsPersistAcrossSmallPose)
{
  expectSpherePrimitiveRowsPersistAcrossSmallPose(
      sx::CollisionShape::makeCapsule(/*radius=*/0.25, /*halfHeight=*/0.5),
      {Eigen::Vector3d(0.45, -0.10, -0.20), Eigen::Vector3d(0.45, 0.10, 0.20)},
      {Eigen::Vector3d(0.4525, -0.0975, -0.1975),
       Eigen::Vector3d(0.4525, 0.0975, 0.1975)},
      dvbd::AvbdContactFeatureKind::Face,
      dvbd::packAvbdCapsuleContactFeatureId(0, 0u));
}

//==============================================================================
// Capsule side rows should also preserve warm starts across endpoint swaps.
TEST(AvbdContact, WorldCollideCapsuleSideRowsIgnoreEndpointOrder)
{
  expectSpherePrimitiveRowsIgnoreEndpointOrder(
      sx::CollisionShape::makeCapsule(/*radius=*/0.25, /*halfHeight=*/0.5),
      {Eigen::Vector3d(0.45, -0.10, -0.20), Eigen::Vector3d(0.45, 0.10, 0.20)},
      dvbd::AvbdContactFeatureKind::Face,
      dvbd::packAvbdCapsuleContactFeatureId(0, 0u));
}

//==============================================================================
// A live box-box narrow-phase manifold emits multiple points on the same box
// features. Reversing the emitted contacts must not change which contact point
// owns each AVBD same-feature row ordinal.
TEST(AvbdContact, WorldCollideLiveManifoldSameFeatureRowsIgnoreContactOrder)
{
  sx::World world;

  auto boxA = world.addRigidBody("box_a");
  boxA.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.5, 0.5, 0.5)));

  sx::RigidBodyOptions boxBOptions;
  boxBOptions.position = Eigen::Vector3d(0.8, 0.0, 0.0);
  auto boxB = world.addRigidBody("box_b", boxBOptions);
  boxB.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.5, 0.5, 0.5)));

  const std::vector<sx::Contact> contacts = world.collide();
  ASSERT_GE(contacts.size(), 4u);

  const auto buildSnapshot = [&](std::vector<sx::Contact> orderedContacts) {
    return dvbd::buildAvbdRigidWorldContactSnapshot(
        dart::simulation::detail::registryOf(world),
        orderedContacts,
        dvbd::AvbdRigidWorldContactOptions{});
  };

  const dvbd::AvbdRigidWorldContactSnapshot forward = buildSnapshot(contacts);
  std::vector<sx::Contact> reversedContacts = contacts;
  std::reverse(reversedContacts.begin(), reversedContacts.end());
  const dvbd::AvbdRigidWorldContactSnapshot reversed
      = buildSnapshot(reversedContacts);

  ASSERT_EQ(forward.contacts.size(), contacts.size());
  ASSERT_EQ(reversed.contacts.size(), contacts.size());

  using RowKey
      = std::pair<dvbd::AvbdContactEndpointId, dvbd::AvbdContactEndpointId>;

  const auto rowKey = [](const dvbd::AvbdRigidContactManifoldPoint& contact) {
    return dvbd::canonicalizeAvbdContactEndpoints(
        contact.endpointA, contact.endpointB);
  };
  const auto rowForPoint
      = [&](const Eigen::Vector3d& point,
            const RowKey& key) -> std::optional<std::uint32_t> {
    for (const dvbd::AvbdRigidContactManifoldPoint& contact :
         reversed.contacts) {
      if (rowKey(contact) == key && (contact.point - point).norm() <= 1e-10) {
        return contact.row;
      }
    }
    return std::nullopt;
  };

  std::vector<std::pair<RowKey, std::vector<std::uint32_t>>> groupedRows;
  for (const dvbd::AvbdRigidContactManifoldPoint& contact : forward.contacts) {
    EXPECT_EQ(
        dvbd::avbdContactFeatureKind(contact.endpointA.feature),
        dvbd::AvbdContactFeatureKind::Face);
    EXPECT_EQ(
        dvbd::avbdContactFeatureKind(contact.endpointB.feature),
        dvbd::AvbdContactFeatureKind::Face);

    const RowKey key = rowKey(contact);
    const std::optional<std::uint32_t> reversedRow
        = rowForPoint(contact.point, key);
    ASSERT_TRUE(reversedRow.has_value());
    EXPECT_EQ(contact.row, *reversedRow);

    auto groupIt = std::find_if(
        groupedRows.begin(),
        groupedRows.end(),
        [&](const std::pair<RowKey, std::vector<std::uint32_t>>& group) {
          return group.first == key;
        });
    if (groupIt == groupedRows.end()) {
      groupedRows.push_back({key, {contact.row}});
    } else {
      groupIt->second.push_back(contact.row);
    }
  }

  std::size_t sameFeatureGroups = 0u;
  for (auto& group : groupedRows) {
    std::vector<std::uint32_t>& rows = group.second;
    std::sort(rows.begin(), rows.end());
    rows.erase(std::unique(rows.begin(), rows.end()), rows.end());
    if (rows.size() >= 2u) {
      ++sameFeatureGroups;
      for (std::size_t row = 0; row < rows.size(); ++row) {
        EXPECT_EQ(rows[row], row);
      }
    }
  }

  EXPECT_GE(sameFeatureGroups, 2u);
}

//==============================================================================
// A live box-box narrow-phase manifold must also keep its AVBD same-feature row
// ordinals when the collision backend reports each contact with endpoints
// swapped.
TEST(AvbdContact, WorldCollideLiveManifoldSameFeatureRowsIgnoreEndpointOrder)
{
  sx::World world;

  auto boxA = world.addRigidBody("box_a");
  boxA.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.5, 0.5, 0.5)));

  sx::RigidBodyOptions boxBOptions;
  boxBOptions.position = Eigen::Vector3d(0.8, 0.0, 0.0);
  auto boxB = world.addRigidBody("box_b", boxBOptions);
  boxB.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.5, 0.5, 0.5)));

  const std::vector<sx::Contact> contacts = world.collide();
  ASSERT_GE(contacts.size(), 4u);

  std::vector<sx::Contact> swappedContacts;
  swappedContacts.reserve(contacts.size());
  for (const sx::Contact& contact : contacts) {
    swappedContacts.push_back(swapEndpointOrder(contact));
  }

  const auto buildSnapshot = [&](std::vector<sx::Contact> orderedContacts) {
    return dvbd::buildAvbdRigidWorldContactSnapshot(
        dart::simulation::detail::registryOf(world),
        orderedContacts,
        dvbd::AvbdRigidWorldContactOptions{});
  };

  const dvbd::AvbdRigidWorldContactSnapshot forward = buildSnapshot(contacts);
  const dvbd::AvbdRigidWorldContactSnapshot swapped
      = buildSnapshot(swappedContacts);

  ASSERT_EQ(forward.contacts.size(), contacts.size());
  ASSERT_EQ(swapped.contacts.size(), contacts.size());

  using RowKey
      = std::pair<dvbd::AvbdContactEndpointId, dvbd::AvbdContactEndpointId>;

  const auto rowKey = [](const dvbd::AvbdRigidContactManifoldPoint& contact) {
    return dvbd::canonicalizeAvbdContactEndpoints(
        contact.endpointA, contact.endpointB);
  };
  const auto rowForPoint
      = [&](const Eigen::Vector3d& point,
            const RowKey& key) -> std::optional<std::uint32_t> {
    for (const dvbd::AvbdRigidContactManifoldPoint& contact :
         swapped.contacts) {
      if (rowKey(contact) == key && (contact.point - point).norm() <= 1e-10) {
        return contact.row;
      }
    }
    return std::nullopt;
  };

  std::vector<std::pair<RowKey, std::vector<std::uint32_t>>> groupedRows;
  for (const dvbd::AvbdRigidContactManifoldPoint& contact : forward.contacts) {
    EXPECT_EQ(
        dvbd::avbdContactFeatureKind(contact.endpointA.feature),
        dvbd::AvbdContactFeatureKind::Face);
    EXPECT_EQ(
        dvbd::avbdContactFeatureKind(contact.endpointB.feature),
        dvbd::AvbdContactFeatureKind::Face);

    const RowKey key = rowKey(contact);
    const std::optional<std::uint32_t> swappedRow
        = rowForPoint(contact.point, key);
    ASSERT_TRUE(swappedRow.has_value());
    EXPECT_EQ(contact.row, *swappedRow);

    auto groupIt = std::find_if(
        groupedRows.begin(),
        groupedRows.end(),
        [&](const std::pair<RowKey, std::vector<std::uint32_t>>& group) {
          return group.first == key;
        });
    if (groupIt == groupedRows.end()) {
      groupedRows.push_back({key, {contact.row}});
    } else {
      groupIt->second.push_back(contact.row);
    }
  }

  std::size_t sameFeatureGroups = 0u;
  for (auto& group : groupedRows) {
    std::vector<std::uint32_t>& rows = group.second;
    std::sort(rows.begin(), rows.end());
    rows.erase(std::unique(rows.begin(), rows.end()), rows.end());
    if (rows.size() >= 2u) {
      ++sameFeatureGroups;
      for (std::size_t row = 0; row < rows.size(); ++row) {
        EXPECT_EQ(rows[row], row);
      }
    }
  }

  EXPECT_GE(sameFeatureGroups, 2u);
}

//==============================================================================
// Friction tangent rows on a live box-box manifold must also preserve their
// warm-started physical tangent dual when all raw narrow-phase contacts arrive
// with the endpoint order swapped.
TEST(AvbdContact, WorldCollideLiveManifoldFrictionRowsIgnoreEndpointOrder)
{
  sx::World world;

  auto boxA = world.addRigidBody("box_a");
  boxA.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.5, 0.5, 0.5)));
  boxA.setFriction(0.81);

  sx::RigidBodyOptions boxBOptions;
  boxBOptions.position = Eigen::Vector3d(0.8, 0.0, 0.0);
  auto boxB = world.addRigidBody("box_b", boxBOptions);
  boxB.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.5, 0.5, 0.5)));
  boxB.setFriction(0.25);

  const std::vector<sx::Contact> contacts = world.collide();
  ASSERT_GE(contacts.size(), 4u);

  std::vector<sx::Contact> swappedContacts;
  swappedContacts.reserve(contacts.size());
  for (const sx::Contact& contact : contacts) {
    swappedContacts.push_back(swapEndpointOrder(contact));
  }

  const auto buildSnapshot = [&](std::vector<sx::Contact> orderedContacts) {
    return dvbd::buildAvbdRigidWorldContactSnapshot(
        dart::simulation::detail::registryOf(world),
        orderedContacts,
        dvbd::AvbdRigidWorldContactOptions{});
  };

  const dvbd::AvbdRigidWorldContactSnapshot forward = buildSnapshot(contacts);
  const dvbd::AvbdRigidWorldContactSnapshot swapped
      = buildSnapshot(swappedContacts);

  ASSERT_EQ(forward.contacts.size(), contacts.size());
  ASSERT_EQ(swapped.contacts.size(), contacts.size());

  using RowKey
      = std::pair<dvbd::AvbdContactEndpointId, dvbd::AvbdContactEndpointId>;

  const auto rowKey = [](const dvbd::AvbdRigidContactManifoldPoint& contact) {
    return dvbd::canonicalizeAvbdContactEndpoints(
        contact.endpointA, contact.endpointB);
  };

  std::vector<std::pair<RowKey, std::vector<std::uint32_t>>> groupedRows;
  for (const dvbd::AvbdRigidContactManifoldPoint& contact : forward.contacts) {
    EXPECT_EQ(
        dvbd::avbdContactFeatureKind(contact.endpointA.feature),
        dvbd::AvbdContactFeatureKind::Face);
    EXPECT_EQ(
        dvbd::avbdContactFeatureKind(contact.endpointB.feature),
        dvbd::AvbdContactFeatureKind::Face);
    EXPECT_NEAR(contact.frictionCoefficient, 0.45, 1e-12);

    const RowKey key = rowKey(contact);
    auto groupIt = std::find_if(
        groupedRows.begin(),
        groupedRows.end(),
        [&](const std::pair<RowKey, std::vector<std::uint32_t>>& group) {
          return group.first == key;
        });
    if (groupIt == groupedRows.end()) {
      groupedRows.push_back({key, {contact.row}});
    } else {
      groupIt->second.push_back(contact.row);
    }
  }

  std::size_t sameFeatureGroups = 0u;
  for (auto& group : groupedRows) {
    std::vector<std::uint32_t>& rows = group.second;
    std::sort(rows.begin(), rows.end());
    rows.erase(std::unique(rows.begin(), rows.end()), rows.end());
    if (rows.size() >= 2u) {
      ++sameFeatureGroups;
      for (std::size_t row = 0; row < rows.size(); ++row) {
        EXPECT_EQ(rows[row], row);
      }
    }
  }
  EXPECT_GE(sameFeatureGroups, 2u);

  dvbd::AvbdScalarRowInventory normalInventory;
  dvbd::AvbdScalarRowInventory frictionInventory;
  std::vector<dvbd::AvbdRigidBodyPointPairRow> normalRows;
  std::vector<dvbd::AvbdRigidBodyPointPairFrictionRows> frictionRows;
  dvbd::AvbdRowWarmStartOptions warmStart;
  warmStart.alpha = 1.0;
  warmStart.gamma = 1.0;

  dvbd::buildAvbdRigidContactManifoldRows(
      forward.states,
      forward.contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      warmStart);

  ASSERT_EQ(normalInventory.size(), forward.contacts.size());
  ASSERT_EQ(frictionInventory.size(), 2u * forward.contacts.size());
  ASSERT_EQ(frictionRows.size(), forward.contacts.size());

  const auto keyFingerprint = [](const dvbd::AvbdScalarRowKey& key) {
    return static_cast<double>(
        key.objectA % 17u + 2u * (key.objectB % 19u) + 3u * (key.featureA % 23u)
        + 5u * (key.featureB % 29u));
  };
  const auto expectedNormalLambda = [&](const dvbd::AvbdScalarRowKey& key) {
    return 25.0 + static_cast<double>(key.row) + 0.01 * keyFingerprint(key);
  };
  const auto expectedFrictionLambda = [&](const dvbd::AvbdScalarRowKey& key) {
    return 1.5 + 0.5 * static_cast<double>(key.row)
           + 0.125 * static_cast<double>(key.axis) + 0.01 * keyFingerprint(key);
  };

  for (dvbd::AvbdScalarRowRecord& record : normalInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::ContactNormal);
    record.state.lambda = expectedNormalLambda(record.descriptor.key);
  }
  for (dvbd::AvbdScalarRowRecord& record : frictionInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::FrictionTangent);
    record.state.lambda = expectedFrictionLambda(record.descriptor.key);
  }

  struct PreviousFrictionRow
  {
    dvbd::AvbdScalarRowKey key;
    double lambda = 0.0;
    Eigen::Vector3d direction = Eigen::Vector3d::Zero();
  };
  std::vector<PreviousFrictionRow> previousFrictionRows;
  previousFrictionRows.reserve(frictionInventory.size());
  for (const dvbd::AvbdScalarRowRecord& record : frictionInventory.records()) {
    previousFrictionRows.push_back(
        PreviousFrictionRow{
            record.descriptor.key, record.state.lambda, record.direction});
  }

  const auto previousFrictionRowForKey
      = [&](const dvbd::AvbdScalarRowKey& key) -> const PreviousFrictionRow* {
    for (const PreviousFrictionRow& previous : previousFrictionRows) {
      if (previous.key == key) {
        return &previous;
      }
    }
    return nullptr;
  };

  dvbd::buildAvbdRigidContactManifoldRows(
      swapped.states,
      swapped.contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      warmStart);

  ASSERT_EQ(normalInventory.size(), swapped.contacts.size());
  ASSERT_EQ(frictionInventory.size(), 2u * swapped.contacts.size());
  ASSERT_EQ(frictionRows.size(), swapped.contacts.size());

  for (const dvbd::AvbdScalarRowRecord& record : normalInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::ContactNormal);
    EXPECT_NEAR(
        record.state.lambda,
        expectedNormalLambda(record.descriptor.key),
        1e-12);
  }
  for (std::size_t i = 0; i < frictionRows.size(); ++i) {
    const dvbd::AvbdScalarRowRecord& firstRecord = frictionInventory[2u * i];
    const dvbd::AvbdScalarRowRecord& secondRecord
        = frictionInventory[2u * i + 1u];
    EXPECT_EQ(
        firstRecord.descriptor.key.role,
        dvbd::AvbdScalarRowRole::FrictionTangent);
    EXPECT_EQ(
        secondRecord.descriptor.key.role,
        dvbd::AvbdScalarRowRole::FrictionTangent);

    const double forceLimit
        = 0.45 * expectedNormalLambda(firstRecord.descriptor.key);
    EXPECT_NEAR(firstRecord.descriptor.bounds.lower, -forceLimit, 1e-12);
    EXPECT_NEAR(firstRecord.descriptor.bounds.upper, forceLimit, 1e-12);
    EXPECT_NEAR(secondRecord.descriptor.bounds.lower, -forceLimit, 1e-12);
    EXPECT_NEAR(secondRecord.descriptor.bounds.upper, forceLimit, 1e-12);

    const PreviousFrictionRow* previousFirst
        = previousFrictionRowForKey(firstRecord.descriptor.key);
    const PreviousFrictionRow* previousSecond
        = previousFrictionRowForKey(secondRecord.descriptor.key);
    ASSERT_NE(previousFirst, nullptr);
    ASSERT_NE(previousSecond, nullptr);

    const Eigen::Vector3d worldDual
        = previousFirst->lambda * previousFirst->direction
          + previousSecond->lambda * previousSecond->direction;
    EXPECT_NEAR(
        firstRecord.state.lambda, worldDual.dot(firstRecord.direction), 1e-12);
    EXPECT_NEAR(
        secondRecord.state.lambda,
        worldDual.dot(secondRecord.direction),
        1e-12);
    EXPECT_NEAR(
        frictionRows[i].first.state.lambda, firstRecord.state.lambda, 1e-12);
    EXPECT_NEAR(
        frictionRows[i].second.state.lambda, secondRecord.state.lambda, 1e-12);
  }
}

//==============================================================================
// A small live pose change that preserves the contacting box features should
// keep each same-feature AVBD row ordinal attached to the same in-face local
// contact location, even though the raw world-space contact points move.
TEST(AvbdContact, WorldCollideLiveManifoldSameFeatureRowsPersistAcrossSmallPose)
{
  sx::World world;

  auto boxA = world.addRigidBody("box_a");
  boxA.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.5, 0.5, 0.5)));

  sx::RigidBodyOptions boxBOptions;
  boxBOptions.position = Eigen::Vector3d(0.8, 0.0, 0.0);
  auto boxB = world.addRigidBody("box_b", boxBOptions);
  boxB.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.5, 0.5, 0.5)));

  const auto buildSnapshot =
      [&](double boxBPosition, dvbd::AvbdRigidWorldContactSnapshot& snapshot) {
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.translation() = Eigen::Vector3d(boxBPosition, 0.0, 0.0);
        boxB.setTransform(pose);

        const std::vector<sx::Contact> contacts = world.collide();
        ASSERT_GE(contacts.size(), 4u);

        snapshot = dvbd::buildAvbdRigidWorldContactSnapshot(
            dart::simulation::detail::registryOf(world),
            contacts,
            dvbd::AvbdRigidWorldContactOptions{});
        ASSERT_EQ(snapshot.contacts.size(), contacts.size());
      };

  dvbd::AvbdRigidWorldContactSnapshot reference;
  dvbd::AvbdRigidWorldContactSnapshot perturbed;
  buildSnapshot(0.8, reference);
  buildSnapshot(0.8025, perturbed);

  using RowKey
      = std::pair<dvbd::AvbdContactEndpointId, dvbd::AvbdContactEndpointId>;
  struct ObservedRow
  {
    RowKey key;
    std::uint32_t row = 0u;
    Eigen::Vector3d canonicalLocalPoint = Eigen::Vector3d::Zero();
  };

  const auto rowKey = [](const dvbd::AvbdRigidContactManifoldPoint& contact) {
    return dvbd::canonicalizeAvbdContactEndpoints(
        contact.endpointA, contact.endpointB);
  };
  const auto collectRows
      = [&](const dvbd::AvbdRigidWorldContactSnapshot& snapshot) {
          std::vector<ObservedRow> rows;
          rows.reserve(snapshot.contacts.size());
          for (const dvbd::AvbdRigidContactManifoldPoint& contact :
               snapshot.contacts) {
            EXPECT_EQ(
                dvbd::avbdContactFeatureKind(contact.endpointA.feature),
                dvbd::AvbdContactFeatureKind::Face);
            EXPECT_EQ(
                dvbd::avbdContactFeatureKind(contact.endpointB.feature),
                dvbd::AvbdContactFeatureKind::Face);
            rows.push_back(
                ObservedRow{
                    rowKey(contact),
                    contact.row,
                    dvbd::detail::avbdRigidWorldContactCanonicalLocalPoint(
                        snapshot, contact)});
          }
          return rows;
        };
  const auto observedAtRow
      = [](const std::vector<ObservedRow>& rows,
           const RowKey& key,
           std::uint32_t row) -> std::optional<ObservedRow> {
    for (const ObservedRow& observed : rows) {
      if (observed.key == key && observed.row == row) {
        return observed;
      }
    }
    return std::nullopt;
  };

  const std::vector<ObservedRow> referenceRows = collectRows(reference);
  const std::vector<ObservedRow> perturbedRows = collectRows(perturbed);
  ASSERT_EQ(referenceRows.size(), perturbedRows.size());

  std::vector<std::pair<RowKey, std::vector<std::uint32_t>>> groupedRows;
  for (const ObservedRow& referenceRow : referenceRows) {
    const std::optional<ObservedRow> perturbedRow
        = observedAtRow(perturbedRows, referenceRow.key, referenceRow.row);
    ASSERT_TRUE(perturbedRow.has_value());
    EXPECT_NEAR(
        referenceRow.canonicalLocalPoint.y(),
        perturbedRow->canonicalLocalPoint.y(),
        1e-8);
    EXPECT_NEAR(
        referenceRow.canonicalLocalPoint.z(),
        perturbedRow->canonicalLocalPoint.z(),
        1e-8);

    auto groupIt = std::find_if(
        groupedRows.begin(),
        groupedRows.end(),
        [&](const std::pair<RowKey, std::vector<std::uint32_t>>& group) {
          return group.first == referenceRow.key;
        });
    if (groupIt == groupedRows.end()) {
      groupedRows.push_back({referenceRow.key, {referenceRow.row}});
    } else {
      groupIt->second.push_back(referenceRow.row);
    }
  }

  std::size_t sameFeatureGroups = 0u;
  for (auto& group : groupedRows) {
    std::vector<std::uint32_t>& rows = group.second;
    std::sort(rows.begin(), rows.end());
    rows.erase(std::unique(rows.begin(), rows.end()), rows.end());
    if (rows.size() >= 2u) {
      ++sameFeatureGroups;
      for (std::size_t row = 0; row < rows.size(); ++row) {
        EXPECT_EQ(rows[row], row);
      }
    }
  }

  EXPECT_GE(sameFeatureGroups, 2u);
}

//==============================================================================
// Friction tangent rows on a live box-box manifold must warm start by the same
// endpoint-pair/feature/row keys as their paired normal rows across small pose
// changes that preserve the contacting box faces.
TEST(AvbdContact, WorldCollideLiveManifoldFrictionRowsPersistAcrossSmallPose)
{
  sx::World world;

  auto boxA = world.addRigidBody("box_a");
  boxA.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.5, 0.5, 0.5)));
  boxA.setFriction(0.81);

  sx::RigidBodyOptions boxBOptions;
  boxBOptions.position = Eigen::Vector3d(0.8, 0.0, 0.0);
  auto boxB = world.addRigidBody("box_b", boxBOptions);
  boxB.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.5, 0.5, 0.5)));
  boxB.setFriction(0.25);

  const auto buildSnapshot =
      [&](double boxBPosition, dvbd::AvbdRigidWorldContactSnapshot& snapshot) {
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.translation() = Eigen::Vector3d(boxBPosition, 0.0, 0.0);
        boxB.setTransform(pose);

        const std::vector<sx::Contact> contacts = world.collide();
        ASSERT_GE(contacts.size(), 4u);

        snapshot = dvbd::buildAvbdRigidWorldContactSnapshot(
            dart::simulation::detail::registryOf(world),
            contacts,
            dvbd::AvbdRigidWorldContactOptions{});
        ASSERT_EQ(snapshot.contacts.size(), contacts.size());
      };

  dvbd::AvbdRigidWorldContactSnapshot reference;
  dvbd::AvbdRigidWorldContactSnapshot perturbed;
  buildSnapshot(0.8, reference);
  buildSnapshot(0.8025, perturbed);
  ASSERT_EQ(reference.contacts.size(), perturbed.contacts.size());

  using RowKey
      = std::pair<dvbd::AvbdContactEndpointId, dvbd::AvbdContactEndpointId>;

  const auto rowKey = [](const dvbd::AvbdRigidContactManifoldPoint& contact) {
    return dvbd::canonicalizeAvbdContactEndpoints(
        contact.endpointA, contact.endpointB);
  };

  std::vector<std::pair<RowKey, std::vector<std::uint32_t>>> groupedRows;
  for (const dvbd::AvbdRigidContactManifoldPoint& contact :
       reference.contacts) {
    EXPECT_EQ(
        dvbd::avbdContactFeatureKind(contact.endpointA.feature),
        dvbd::AvbdContactFeatureKind::Face);
    EXPECT_EQ(
        dvbd::avbdContactFeatureKind(contact.endpointB.feature),
        dvbd::AvbdContactFeatureKind::Face);
    EXPECT_NEAR(contact.frictionCoefficient, 0.45, 1e-12);

    const RowKey key = rowKey(contact);
    auto groupIt = std::find_if(
        groupedRows.begin(),
        groupedRows.end(),
        [&](const std::pair<RowKey, std::vector<std::uint32_t>>& group) {
          return group.first == key;
        });
    if (groupIt == groupedRows.end()) {
      groupedRows.push_back({key, {contact.row}});
    } else {
      groupIt->second.push_back(contact.row);
    }
  }

  std::size_t sameFeatureGroups = 0u;
  for (auto& group : groupedRows) {
    std::vector<std::uint32_t>& rows = group.second;
    std::sort(rows.begin(), rows.end());
    rows.erase(std::unique(rows.begin(), rows.end()), rows.end());
    if (rows.size() >= 2u) {
      ++sameFeatureGroups;
      for (std::size_t row = 0; row < rows.size(); ++row) {
        EXPECT_EQ(rows[row], row);
      }
    }
  }
  EXPECT_GE(sameFeatureGroups, 2u);

  dvbd::AvbdScalarRowInventory normalInventory;
  dvbd::AvbdScalarRowInventory frictionInventory;
  std::vector<dvbd::AvbdRigidBodyPointPairRow> normalRows;
  std::vector<dvbd::AvbdRigidBodyPointPairFrictionRows> frictionRows;
  dvbd::AvbdRowWarmStartOptions warmStart;
  warmStart.alpha = 1.0;
  warmStart.gamma = 1.0;

  dvbd::buildAvbdRigidContactManifoldRows(
      reference.states,
      reference.contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      warmStart);

  ASSERT_EQ(normalInventory.size(), reference.contacts.size());
  ASSERT_EQ(frictionInventory.size(), 2u * reference.contacts.size());
  ASSERT_EQ(frictionRows.size(), reference.contacts.size());

  const auto keyFingerprint = [](const dvbd::AvbdScalarRowKey& key) {
    return static_cast<double>(
        key.objectA % 17u + 2u * (key.objectB % 19u) + 3u * (key.featureA % 23u)
        + 5u * (key.featureB % 29u));
  };
  const auto expectedNormalLambda = [&](const dvbd::AvbdScalarRowKey& key) {
    return 20.0 + static_cast<double>(key.row) + 0.01 * keyFingerprint(key);
  };
  const auto expectedFrictionLambda = [&](const dvbd::AvbdScalarRowKey& key) {
    return 1.0 + 0.5 * static_cast<double>(key.row)
           + 0.125 * static_cast<double>(key.axis) + 0.01 * keyFingerprint(key);
  };

  for (dvbd::AvbdScalarRowRecord& record : normalInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::ContactNormal);
    record.state.lambda = expectedNormalLambda(record.descriptor.key);
  }
  for (dvbd::AvbdScalarRowRecord& record : frictionInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::FrictionTangent);
    record.state.lambda = expectedFrictionLambda(record.descriptor.key);
  }

  dvbd::buildAvbdRigidContactManifoldRows(
      perturbed.states,
      perturbed.contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      warmStart);

  ASSERT_EQ(normalInventory.size(), perturbed.contacts.size());
  ASSERT_EQ(frictionInventory.size(), 2u * perturbed.contacts.size());
  ASSERT_EQ(frictionRows.size(), perturbed.contacts.size());

  for (const dvbd::AvbdScalarRowRecord& record : normalInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::ContactNormal);
    EXPECT_NEAR(
        record.state.lambda,
        expectedNormalLambda(record.descriptor.key),
        1e-12);
  }
  for (const dvbd::AvbdScalarRowRecord& record : frictionInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::FrictionTangent);
    const double forceLimit
        = 0.45 * expectedNormalLambda(record.descriptor.key);
    EXPECT_NEAR(record.descriptor.bounds.lower, -forceLimit, 1e-12);
    EXPECT_NEAR(record.descriptor.bounds.upper, forceLimit, 1e-12);
    EXPECT_NEAR(
        record.state.lambda,
        expectedFrictionLambda(record.descriptor.key),
        1e-12);
  }
  for (std::size_t i = 0; i < frictionRows.size(); ++i) {
    EXPECT_NEAR(
        frictionRows[i].first.state.lambda,
        frictionInventory[2u * i].state.lambda,
        1e-12);
    EXPECT_NEAR(
        frictionRows[i].second.state.lambda,
        frictionInventory[2u * i + 1u].state.lambda,
        1e-12);
  }
}

//==============================================================================
// A small live pose change in a stacked contact scene should preserve AVBD row
// identity across more than one simultaneous canonical endpoint pair.
TEST(AvbdContact, WorldCollideStackedManifoldsPersistRowsAcrossSmallPose)
{
  sx::World world;

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(2.0, 2.0, 0.5)));

  sx::RigidBodyOptions lowerOptions;
  lowerOptions.position = Eigen::Vector3d(0.0, 0.0, 0.45);
  auto lower = world.addRigidBody("lower", lowerOptions);
  lower.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.5, 0.5, 0.5)));

  sx::RigidBodyOptions upperOptions;
  upperOptions.position = Eigen::Vector3d(0.0, 0.0, 1.35);
  auto upper = world.addRigidBody("upper", upperOptions);
  upper.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.5, 0.5, 0.5)));

  const auto buildSnapshot
      = [&](double upperHeight, dvbd::AvbdRigidWorldContactSnapshot& snapshot) {
          Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
          pose.translation() = Eigen::Vector3d(0.0, 0.0, upperHeight);
          upper.setTransform(pose);

          const std::vector<sx::Contact> contacts = world.collide();
          ASSERT_GE(contacts.size(), 8u);

          snapshot = dvbd::buildAvbdRigidWorldContactSnapshot(
              dart::simulation::detail::registryOf(world),
              contacts,
              dvbd::AvbdRigidWorldContactOptions{});
          ASSERT_EQ(snapshot.contacts.size(), contacts.size());
        };

  dvbd::AvbdRigidWorldContactSnapshot reference;
  dvbd::AvbdRigidWorldContactSnapshot perturbed;
  buildSnapshot(1.35, reference);
  buildSnapshot(1.3525, perturbed);

  using RowKey
      = std::pair<dvbd::AvbdContactEndpointId, dvbd::AvbdContactEndpointId>;
  struct ObservedRow
  {
    RowKey key;
    std::uint32_t row = 0u;
    Eigen::Vector3d canonicalLocalPoint = Eigen::Vector3d::Zero();
  };

  const auto rowKey = [](const dvbd::AvbdRigidContactManifoldPoint& contact) {
    return dvbd::canonicalizeAvbdContactEndpoints(
        contact.endpointA, contact.endpointB);
  };
  const auto collectRows
      = [&](const dvbd::AvbdRigidWorldContactSnapshot& snapshot) {
          std::vector<ObservedRow> rows;
          rows.reserve(snapshot.contacts.size());
          for (const dvbd::AvbdRigidContactManifoldPoint& contact :
               snapshot.contacts) {
            EXPECT_EQ(
                dvbd::avbdContactFeatureKind(contact.endpointA.feature),
                dvbd::AvbdContactFeatureKind::Face);
            EXPECT_EQ(
                dvbd::avbdContactFeatureKind(contact.endpointB.feature),
                dvbd::AvbdContactFeatureKind::Face);
            rows.push_back(
                ObservedRow{
                    rowKey(contact),
                    contact.row,
                    dvbd::detail::avbdRigidWorldContactCanonicalLocalPoint(
                        snapshot, contact)});
          }
          return rows;
        };
  const auto observedAtRow
      = [](const std::vector<ObservedRow>& rows,
           const RowKey& key,
           std::uint32_t row) -> std::optional<ObservedRow> {
    for (const ObservedRow& observed : rows) {
      if (observed.key == key && observed.row == row) {
        return observed;
      }
    }
    return std::nullopt;
  };

  const std::vector<ObservedRow> referenceRows = collectRows(reference);
  const std::vector<ObservedRow> perturbedRows = collectRows(perturbed);
  ASSERT_EQ(referenceRows.size(), perturbedRows.size());

  std::vector<std::pair<RowKey, std::vector<std::uint32_t>>> groupedRows;
  for (const ObservedRow& referenceRow : referenceRows) {
    const std::optional<ObservedRow> perturbedRow
        = observedAtRow(perturbedRows, referenceRow.key, referenceRow.row);
    ASSERT_TRUE(perturbedRow.has_value());
    EXPECT_NEAR(
        referenceRow.canonicalLocalPoint.x(),
        perturbedRow->canonicalLocalPoint.x(),
        1e-8);
    EXPECT_NEAR(
        referenceRow.canonicalLocalPoint.y(),
        perturbedRow->canonicalLocalPoint.y(),
        1e-8);

    auto groupIt = std::find_if(
        groupedRows.begin(),
        groupedRows.end(),
        [&](const std::pair<RowKey, std::vector<std::uint32_t>>& group) {
          return group.first == referenceRow.key;
        });
    if (groupIt == groupedRows.end()) {
      groupedRows.push_back({referenceRow.key, {referenceRow.row}});
    } else {
      groupIt->second.push_back(referenceRow.row);
    }
  }

  std::size_t sameFeatureGroups = 0u;
  for (auto& group : groupedRows) {
    std::vector<std::uint32_t>& rows = group.second;
    std::sort(rows.begin(), rows.end());
    rows.erase(std::unique(rows.begin(), rows.end()), rows.end());
    if (rows.size() >= 2u) {
      ++sameFeatureGroups;
      for (std::size_t row = 0; row < rows.size(); ++row) {
        EXPECT_EQ(rows[row], row);
      }
    }
  }

  EXPECT_GE(sameFeatureGroups, 2u);
}

//==============================================================================
// A stacked live contact scene should preserve friction warm-start state across
// more than one simultaneous canonical endpoint pair. This guards the first
// pile-like static/dynamic and dynamic/dynamic manifold combination.
TEST(
    AvbdContact, WorldCollideStackedManifoldsFrictionRowsPersistAcrossSmallPose)
{
  sx::World world;

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(2.0, 2.0, 0.5)));
  ground.setFriction(0.64);

  sx::RigidBodyOptions lowerOptions;
  lowerOptions.position = Eigen::Vector3d(0.0, 0.0, 0.45);
  auto lower = world.addRigidBody("lower", lowerOptions);
  lower.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.5, 0.5, 0.5)));
  lower.setFriction(0.25);

  sx::RigidBodyOptions upperOptions;
  upperOptions.position = Eigen::Vector3d(0.0, 0.0, 1.35);
  auto upper = world.addRigidBody("upper", upperOptions);
  upper.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.5, 0.5, 0.5)));
  upper.setFriction(0.36);

  const auto buildSnapshot
      = [&](double upperHeight, dvbd::AvbdRigidWorldContactSnapshot& snapshot) {
          Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
          pose.translation() = Eigen::Vector3d(0.0, 0.0, upperHeight);
          upper.setTransform(pose);

          const std::vector<sx::Contact> contacts = world.collide();
          ASSERT_GE(contacts.size(), 8u);

          snapshot = dvbd::buildAvbdRigidWorldContactSnapshot(
              dart::simulation::detail::registryOf(world),
              contacts,
              dvbd::AvbdRigidWorldContactOptions{});
          ASSERT_EQ(snapshot.contacts.size(), contacts.size());
        };

  dvbd::AvbdRigidWorldContactSnapshot reference;
  dvbd::AvbdRigidWorldContactSnapshot perturbed;
  buildSnapshot(1.35, reference);
  buildSnapshot(1.3525, perturbed);
  ASSERT_EQ(reference.contacts.size(), perturbed.contacts.size());

  using RowKey
      = std::pair<dvbd::AvbdContactEndpointId, dvbd::AvbdContactEndpointId>;

  const auto rowKey = [](const dvbd::AvbdRigidContactManifoldPoint& contact) {
    return dvbd::canonicalizeAvbdContactEndpoints(
        contact.endpointA, contact.endpointB);
  };

  std::vector<std::pair<RowKey, std::vector<std::uint32_t>>> groupedRows;
  std::vector<double> frictionCoefficients;
  for (const dvbd::AvbdRigidContactManifoldPoint& contact :
       reference.contacts) {
    EXPECT_EQ(
        dvbd::avbdContactFeatureKind(contact.endpointA.feature),
        dvbd::AvbdContactFeatureKind::Face);
    EXPECT_EQ(
        dvbd::avbdContactFeatureKind(contact.endpointB.feature),
        dvbd::AvbdContactFeatureKind::Face);
    frictionCoefficients.push_back(contact.frictionCoefficient);

    const RowKey key = rowKey(contact);
    auto groupIt = std::find_if(
        groupedRows.begin(),
        groupedRows.end(),
        [&](const std::pair<RowKey, std::vector<std::uint32_t>>& group) {
          return group.first == key;
        });
    if (groupIt == groupedRows.end()) {
      groupedRows.push_back({key, {contact.row}});
    } else {
      groupIt->second.push_back(contact.row);
    }
  }

  std::sort(frictionCoefficients.begin(), frictionCoefficients.end());
  frictionCoefficients.erase(
      std::unique(
          frictionCoefficients.begin(),
          frictionCoefficients.end(),
          [](double lhs, double rhs) { return std::abs(lhs - rhs) <= 1e-12; }),
      frictionCoefficients.end());
  EXPECT_GE(frictionCoefficients.size(), 2u);

  std::size_t sameFeatureGroups = 0u;
  for (auto& group : groupedRows) {
    std::vector<std::uint32_t>& rows = group.second;
    std::sort(rows.begin(), rows.end());
    rows.erase(std::unique(rows.begin(), rows.end()), rows.end());
    if (rows.size() >= 2u) {
      ++sameFeatureGroups;
      for (std::size_t row = 0; row < rows.size(); ++row) {
        EXPECT_EQ(rows[row], row);
      }
    }
  }
  EXPECT_GE(sameFeatureGroups, 2u);

  dvbd::AvbdScalarRowInventory normalInventory;
  dvbd::AvbdScalarRowInventory frictionInventory;
  std::vector<dvbd::AvbdRigidBodyPointPairRow> normalRows;
  std::vector<dvbd::AvbdRigidBodyPointPairFrictionRows> frictionRows;
  dvbd::AvbdRowWarmStartOptions warmStart;
  warmStart.alpha = 1.0;
  warmStart.gamma = 1.0;

  dvbd::buildAvbdRigidContactManifoldRows(
      reference.states,
      reference.contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      warmStart);

  ASSERT_EQ(normalInventory.size(), reference.contacts.size());
  ASSERT_EQ(frictionInventory.size(), 2u * reference.contacts.size());
  ASSERT_EQ(frictionRows.size(), reference.contacts.size());

  const auto keyFingerprint = [](const dvbd::AvbdScalarRowKey& key) {
    return static_cast<double>(
        key.objectA % 17u + 2u * (key.objectB % 19u) + 3u * (key.featureA % 23u)
        + 5u * (key.featureB % 29u));
  };
  const auto expectedNormalLambda = [&](const dvbd::AvbdScalarRowKey& key) {
    return 30.0 + static_cast<double>(key.row) + 0.01 * keyFingerprint(key);
  };
  const auto expectedFrictionLambda = [&](const dvbd::AvbdScalarRowKey& key) {
    return 2.0 + 0.5 * static_cast<double>(key.row)
           + 0.125 * static_cast<double>(key.axis) + 0.01 * keyFingerprint(key);
  };

  for (dvbd::AvbdScalarRowRecord& record : normalInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::ContactNormal);
    record.state.lambda = expectedNormalLambda(record.descriptor.key);
  }
  for (dvbd::AvbdScalarRowRecord& record : frictionInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::FrictionTangent);
    record.state.lambda = expectedFrictionLambda(record.descriptor.key);
  }

  dvbd::buildAvbdRigidContactManifoldRows(
      perturbed.states,
      perturbed.contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      warmStart);

  ASSERT_EQ(normalInventory.size(), perturbed.contacts.size());
  ASSERT_EQ(frictionInventory.size(), 2u * perturbed.contacts.size());
  ASSERT_EQ(frictionRows.size(), perturbed.contacts.size());

  const auto frictionCoefficientForKey
      = [&](const dvbd::AvbdScalarRowKey& key) -> std::optional<double> {
    for (const dvbd::AvbdRigidContactManifoldPoint& contact :
         perturbed.contacts) {
      const dvbd::AvbdScalarRowKey contactKey
          = dvbd::makeAvbdEndpointPairRowKey(
              dvbd::AvbdScalarRowRole::FrictionTangent,
              contact.endpointA,
              contact.endpointB,
              contact.row,
              key.axis);
      if (contactKey == key) {
        return contact.frictionCoefficient;
      }
    }
    return std::nullopt;
  };

  for (const dvbd::AvbdScalarRowRecord& record : normalInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::ContactNormal);
    EXPECT_NEAR(
        record.state.lambda,
        expectedNormalLambda(record.descriptor.key),
        1e-12);
  }
  for (const dvbd::AvbdScalarRowRecord& record : frictionInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::FrictionTangent);
    const std::optional<double> frictionCoefficient
        = frictionCoefficientForKey(record.descriptor.key);
    ASSERT_TRUE(frictionCoefficient.has_value());
    const double forceLimit
        = *frictionCoefficient * expectedNormalLambda(record.descriptor.key);
    EXPECT_NEAR(record.descriptor.bounds.lower, -forceLimit, 1e-12);
    EXPECT_NEAR(record.descriptor.bounds.upper, forceLimit, 1e-12);
    EXPECT_NEAR(
        record.state.lambda,
        expectedFrictionLambda(record.descriptor.key),
        1e-12);
  }
  for (std::size_t i = 0; i < frictionRows.size(); ++i) {
    EXPECT_NEAR(
        frictionRows[i].first.state.lambda,
        frictionInventory[2u * i].state.lambda,
        1e-12);
    EXPECT_NEAR(
        frictionRows[i].second.state.lambda,
        frictionInventory[2u * i + 1u].state.lambda,
        1e-12);
  }
}

//==============================================================================
// Contact-order robustness should also hold across the first stacked
// static/dynamic and dynamic/dynamic manifold combination.
TEST(AvbdContact, WorldCollideStackedManifoldsFrictionRowsIgnoreContactOrder)
{
  sx::World world;

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(2.0, 2.0, 0.5)));
  ground.setFriction(0.64);

  sx::RigidBodyOptions lowerOptions;
  lowerOptions.position = Eigen::Vector3d(0.0, 0.0, 0.45);
  auto lower = world.addRigidBody("lower", lowerOptions);
  lower.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.5, 0.5, 0.5)));
  lower.setFriction(0.25);

  sx::RigidBodyOptions upperOptions;
  upperOptions.position = Eigen::Vector3d(0.0, 0.0, 1.35);
  auto upper = world.addRigidBody("upper", upperOptions);
  upper.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.5, 0.5, 0.5)));
  upper.setFriction(0.36);

  const std::vector<sx::Contact> contacts = world.collide();
  ASSERT_GE(contacts.size(), 8u);

  std::vector<sx::Contact> reversedContacts = contacts;
  std::reverse(reversedContacts.begin(), reversedContacts.end());

  const auto buildSnapshot = [&](std::vector<sx::Contact> orderedContacts) {
    return dvbd::buildAvbdRigidWorldContactSnapshot(
        dart::simulation::detail::registryOf(world),
        orderedContacts,
        dvbd::AvbdRigidWorldContactOptions{});
  };

  const dvbd::AvbdRigidWorldContactSnapshot forward = buildSnapshot(contacts);
  const dvbd::AvbdRigidWorldContactSnapshot reversed
      = buildSnapshot(reversedContacts);

  ASSERT_EQ(forward.contacts.size(), contacts.size());
  ASSERT_EQ(reversed.contacts.size(), contacts.size());

  using RowKey
      = std::pair<dvbd::AvbdContactEndpointId, dvbd::AvbdContactEndpointId>;

  const auto rowKey = [](const dvbd::AvbdRigidContactManifoldPoint& contact) {
    return dvbd::canonicalizeAvbdContactEndpoints(
        contact.endpointA, contact.endpointB);
  };
  const auto matchingReversedContact
      = [&](const dvbd::AvbdRigidContactManifoldPoint& reference)
      -> const dvbd::AvbdRigidContactManifoldPoint* {
    const RowKey key = rowKey(reference);
    for (const dvbd::AvbdRigidContactManifoldPoint& contact :
         reversed.contacts) {
      if (rowKey(contact) == key
          && (contact.point - reference.point).norm() <= 1e-10) {
        return &contact;
      }
    }
    return nullptr;
  };

  std::vector<std::pair<RowKey, std::vector<std::uint32_t>>> groupedRows;
  std::vector<double> frictionCoefficients;
  for (const dvbd::AvbdRigidContactManifoldPoint& contact : forward.contacts) {
    EXPECT_EQ(
        dvbd::avbdContactFeatureKind(contact.endpointA.feature),
        dvbd::AvbdContactFeatureKind::Face);
    EXPECT_EQ(
        dvbd::avbdContactFeatureKind(contact.endpointB.feature),
        dvbd::AvbdContactFeatureKind::Face);

    const dvbd::AvbdRigidContactManifoldPoint* reversedContact
        = matchingReversedContact(contact);
    ASSERT_NE(reversedContact, nullptr);
    EXPECT_EQ(contact.row, reversedContact->row);
    EXPECT_NEAR(
        contact.frictionCoefficient,
        reversedContact->frictionCoefficient,
        1e-12);
    frictionCoefficients.push_back(contact.frictionCoefficient);

    const RowKey key = rowKey(contact);
    auto groupIt = std::find_if(
        groupedRows.begin(),
        groupedRows.end(),
        [&](const std::pair<RowKey, std::vector<std::uint32_t>>& group) {
          return group.first == key;
        });
    if (groupIt == groupedRows.end()) {
      groupedRows.push_back({key, {contact.row}});
    } else {
      groupIt->second.push_back(contact.row);
    }
  }

  std::sort(frictionCoefficients.begin(), frictionCoefficients.end());
  frictionCoefficients.erase(
      std::unique(
          frictionCoefficients.begin(),
          frictionCoefficients.end(),
          [](double lhs, double rhs) { return std::abs(lhs - rhs) <= 1e-12; }),
      frictionCoefficients.end());
  EXPECT_GE(frictionCoefficients.size(), 2u);

  std::size_t sameFeatureGroups = 0u;
  for (auto& group : groupedRows) {
    std::vector<std::uint32_t>& rows = group.second;
    std::sort(rows.begin(), rows.end());
    rows.erase(std::unique(rows.begin(), rows.end()), rows.end());
    if (rows.size() >= 2u) {
      ++sameFeatureGroups;
      for (std::size_t row = 0; row < rows.size(); ++row) {
        EXPECT_EQ(rows[row], row);
      }
    }
  }
  EXPECT_GE(sameFeatureGroups, 2u);

  dvbd::AvbdScalarRowInventory normalInventory;
  dvbd::AvbdScalarRowInventory frictionInventory;
  std::vector<dvbd::AvbdRigidBodyPointPairRow> normalRows;
  std::vector<dvbd::AvbdRigidBodyPointPairFrictionRows> frictionRows;
  dvbd::AvbdRowWarmStartOptions warmStart;
  warmStart.alpha = 1.0;
  warmStart.gamma = 1.0;

  dvbd::buildAvbdRigidContactManifoldRows(
      forward.states,
      forward.contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      warmStart);

  ASSERT_EQ(normalInventory.size(), forward.contacts.size());
  ASSERT_EQ(frictionInventory.size(), 2u * forward.contacts.size());
  ASSERT_EQ(frictionRows.size(), forward.contacts.size());

  const auto keyFingerprint = [](const dvbd::AvbdScalarRowKey& key) {
    return static_cast<double>(
        key.objectA % 17u + 2u * (key.objectB % 19u) + 3u * (key.featureA % 23u)
        + 5u * (key.featureB % 29u));
  };
  const auto expectedNormalLambda = [&](const dvbd::AvbdScalarRowKey& key) {
    return 32.0 + static_cast<double>(key.row) + 0.01 * keyFingerprint(key);
  };
  const auto expectedFrictionLambda = [&](const dvbd::AvbdScalarRowKey& key) {
    return 2.25 + 0.5 * static_cast<double>(key.row)
           + 0.125 * static_cast<double>(key.axis) + 0.01 * keyFingerprint(key);
  };

  for (dvbd::AvbdScalarRowRecord& record : normalInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::ContactNormal);
    record.state.lambda = expectedNormalLambda(record.descriptor.key);
  }
  for (dvbd::AvbdScalarRowRecord& record : frictionInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::FrictionTangent);
    record.state.lambda = expectedFrictionLambda(record.descriptor.key);
  }

  const auto frictionCoefficientForKey
      = [&](const dvbd::AvbdScalarRowKey& key) -> std::optional<double> {
    for (const dvbd::AvbdRigidContactManifoldPoint& contact :
         reversed.contacts) {
      const dvbd::AvbdScalarRowKey contactKey
          = dvbd::makeAvbdEndpointPairRowKey(
              dvbd::AvbdScalarRowRole::FrictionTangent,
              contact.endpointA,
              contact.endpointB,
              contact.row,
              key.axis);
      if (contactKey == key) {
        return contact.frictionCoefficient;
      }
    }
    return std::nullopt;
  };

  dvbd::buildAvbdRigidContactManifoldRows(
      reversed.states,
      reversed.contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      warmStart);

  ASSERT_EQ(normalInventory.size(), reversed.contacts.size());
  ASSERT_EQ(frictionInventory.size(), 2u * reversed.contacts.size());
  ASSERT_EQ(frictionRows.size(), reversed.contacts.size());

  for (const dvbd::AvbdScalarRowRecord& record : normalInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::ContactNormal);
    EXPECT_NEAR(
        record.state.lambda,
        expectedNormalLambda(record.descriptor.key),
        1e-12);
  }
  for (const dvbd::AvbdScalarRowRecord& record : frictionInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::FrictionTangent);
    const std::optional<double> frictionCoefficient
        = frictionCoefficientForKey(record.descriptor.key);
    ASSERT_TRUE(frictionCoefficient.has_value());
    const double forceLimit
        = *frictionCoefficient * expectedNormalLambda(record.descriptor.key);
    EXPECT_NEAR(record.descriptor.bounds.lower, -forceLimit, 1e-12);
    EXPECT_NEAR(record.descriptor.bounds.upper, forceLimit, 1e-12);
    EXPECT_NEAR(
        record.state.lambda,
        expectedFrictionLambda(record.descriptor.key),
        1e-12);
  }
  for (std::size_t i = 0; i < frictionRows.size(); ++i) {
    EXPECT_NEAR(
        frictionRows[i].first.state.lambda,
        frictionInventory[2u * i].state.lambda,
        1e-12);
    EXPECT_NEAR(
        frictionRows[i].second.state.lambda,
        frictionInventory[2u * i + 1u].state.lambda,
        1e-12);
  }
}

//==============================================================================
// A wider live pile should preserve friction warm-start state across several
// simultaneous canonical endpoint pairs, including two independent lower
// supports and one top body spanning both dynamic supports.
TEST(AvbdContact, WorldCollideBoxPileFrictionRowsPersistAcrossSmallPose)
{
  sx::World world;

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(2.5, 2.0, 0.5)));
  ground.setFriction(0.64);

  sx::RigidBodyOptions leftLowerOptions;
  leftLowerOptions.position = Eigen::Vector3d(-0.45, 0.0, 0.38);
  auto leftLower = world.addRigidBody("left_lower", leftLowerOptions);
  leftLower.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.4, 0.4, 0.4)));
  leftLower.setFriction(0.25);

  sx::RigidBodyOptions rightLowerOptions;
  rightLowerOptions.position = Eigen::Vector3d(0.45, 0.0, 0.38);
  auto rightLower = world.addRigidBody("right_lower", rightLowerOptions);
  rightLower.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.4, 0.4, 0.4)));
  rightLower.setFriction(0.36);

  sx::RigidBodyOptions topOptions;
  topOptions.position = Eigen::Vector3d(0.0, 0.0, 1.10);
  auto top = world.addRigidBody("top", topOptions);
  top.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.7, 0.4, 0.35)));
  top.setFriction(0.49);

  const auto buildSnapshot
      = [&](double topHeight, dvbd::AvbdRigidWorldContactSnapshot& snapshot) {
          Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
          pose.translation() = Eigen::Vector3d(0.0, 0.0, topHeight);
          top.setTransform(pose);

          const std::vector<sx::Contact> contacts = world.collide();
          ASSERT_GE(contacts.size(), 12u);

          snapshot = dvbd::buildAvbdRigidWorldContactSnapshot(
              dart::simulation::detail::registryOf(world),
              contacts,
              dvbd::AvbdRigidWorldContactOptions{});
          ASSERT_EQ(snapshot.contacts.size(), contacts.size());
        };

  dvbd::AvbdRigidWorldContactSnapshot reference;
  dvbd::AvbdRigidWorldContactSnapshot perturbed;
  buildSnapshot(1.10, reference);
  buildSnapshot(1.1025, perturbed);
  ASSERT_EQ(reference.contacts.size(), perturbed.contacts.size());

  using RowKey
      = std::pair<dvbd::AvbdContactEndpointId, dvbd::AvbdContactEndpointId>;
  struct ObservedRow
  {
    RowKey key;
    std::uint32_t row = 0u;
    Eigen::Vector3d canonicalLocalPoint = Eigen::Vector3d::Zero();
    double frictionCoefficient = 0.0;
  };

  const auto rowKey = [](const dvbd::AvbdRigidContactManifoldPoint& contact) {
    return dvbd::canonicalizeAvbdContactEndpoints(
        contact.endpointA, contact.endpointB);
  };
  const auto collectRows
      = [&](const dvbd::AvbdRigidWorldContactSnapshot& snapshot) {
          std::vector<ObservedRow> rows;
          rows.reserve(snapshot.contacts.size());
          for (const dvbd::AvbdRigidContactManifoldPoint& contact :
               snapshot.contacts) {
            EXPECT_EQ(
                dvbd::avbdContactFeatureKind(contact.endpointA.feature),
                dvbd::AvbdContactFeatureKind::Face);
            EXPECT_EQ(
                dvbd::avbdContactFeatureKind(contact.endpointB.feature),
                dvbd::AvbdContactFeatureKind::Face);
            rows.push_back(
                ObservedRow{
                    rowKey(contact),
                    contact.row,
                    dvbd::detail::avbdRigidWorldContactCanonicalLocalPoint(
                        snapshot, contact),
                    contact.frictionCoefficient});
          }
          return rows;
        };
  const auto observedAtRow
      = [](const std::vector<ObservedRow>& rows,
           const RowKey& key,
           std::uint32_t row) -> std::optional<ObservedRow> {
    for (const ObservedRow& observed : rows) {
      if (observed.key == key && observed.row == row) {
        return observed;
      }
    }
    return std::nullopt;
  };

  const std::vector<ObservedRow> referenceRows = collectRows(reference);
  const std::vector<ObservedRow> perturbedRows = collectRows(perturbed);
  ASSERT_EQ(referenceRows.size(), perturbedRows.size());

  std::vector<std::pair<RowKey, std::vector<std::uint32_t>>> groupedRows;
  std::vector<double> frictionCoefficients;
  for (const ObservedRow& referenceRow : referenceRows) {
    const std::optional<ObservedRow> perturbedRow
        = observedAtRow(perturbedRows, referenceRow.key, referenceRow.row);
    ASSERT_TRUE(perturbedRow.has_value());
    EXPECT_NEAR(
        referenceRow.canonicalLocalPoint.x(),
        perturbedRow->canonicalLocalPoint.x(),
        1e-8);
    EXPECT_NEAR(
        referenceRow.canonicalLocalPoint.y(),
        perturbedRow->canonicalLocalPoint.y(),
        1e-8);
    EXPECT_NEAR(
        referenceRow.frictionCoefficient,
        perturbedRow->frictionCoefficient,
        1e-12);
    frictionCoefficients.push_back(referenceRow.frictionCoefficient);

    auto groupIt = std::find_if(
        groupedRows.begin(),
        groupedRows.end(),
        [&](const std::pair<RowKey, std::vector<std::uint32_t>>& group) {
          return group.first == referenceRow.key;
        });
    if (groupIt == groupedRows.end()) {
      groupedRows.push_back({referenceRow.key, {referenceRow.row}});
    } else {
      groupIt->second.push_back(referenceRow.row);
    }
  }

  std::sort(frictionCoefficients.begin(), frictionCoefficients.end());
  frictionCoefficients.erase(
      std::unique(
          frictionCoefficients.begin(),
          frictionCoefficients.end(),
          [](double lhs, double rhs) { return std::abs(lhs - rhs) <= 1e-12; }),
      frictionCoefficients.end());
  EXPECT_GE(frictionCoefficients.size(), 3u);

  std::size_t sameFeatureGroups = 0u;
  for (auto& group : groupedRows) {
    std::vector<std::uint32_t>& rows = group.second;
    std::sort(rows.begin(), rows.end());
    rows.erase(std::unique(rows.begin(), rows.end()), rows.end());
    if (rows.size() >= 2u) {
      ++sameFeatureGroups;
      for (std::size_t row = 0; row < rows.size(); ++row) {
        EXPECT_EQ(rows[row], row);
      }
    }
  }
  EXPECT_GE(sameFeatureGroups, 4u);

  dvbd::AvbdScalarRowInventory normalInventory;
  dvbd::AvbdScalarRowInventory frictionInventory;
  std::vector<dvbd::AvbdRigidBodyPointPairRow> normalRows;
  std::vector<dvbd::AvbdRigidBodyPointPairFrictionRows> frictionRows;
  dvbd::AvbdRowWarmStartOptions warmStart;
  warmStart.alpha = 1.0;
  warmStart.gamma = 1.0;

  dvbd::buildAvbdRigidContactManifoldRows(
      reference.states,
      reference.contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      warmStart);

  ASSERT_EQ(normalInventory.size(), reference.contacts.size());
  ASSERT_EQ(frictionInventory.size(), 2u * reference.contacts.size());
  ASSERT_EQ(frictionRows.size(), reference.contacts.size());

  const auto keyFingerprint = [](const dvbd::AvbdScalarRowKey& key) {
    return static_cast<double>(
        key.objectA % 17u + 2u * (key.objectB % 19u) + 3u * (key.featureA % 23u)
        + 5u * (key.featureB % 29u));
  };
  const auto expectedNormalLambda = [&](const dvbd::AvbdScalarRowKey& key) {
    return 40.0 + static_cast<double>(key.row) + 0.01 * keyFingerprint(key);
  };
  const auto expectedFrictionLambda = [&](const dvbd::AvbdScalarRowKey& key) {
    return 3.0 + 0.5 * static_cast<double>(key.row)
           + 0.125 * static_cast<double>(key.axis) + 0.01 * keyFingerprint(key);
  };

  for (dvbd::AvbdScalarRowRecord& record : normalInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::ContactNormal);
    record.state.lambda = expectedNormalLambda(record.descriptor.key);
  }
  for (dvbd::AvbdScalarRowRecord& record : frictionInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::FrictionTangent);
    record.state.lambda = expectedFrictionLambda(record.descriptor.key);
  }

  dvbd::buildAvbdRigidContactManifoldRows(
      perturbed.states,
      perturbed.contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      warmStart);

  ASSERT_EQ(normalInventory.size(), perturbed.contacts.size());
  ASSERT_EQ(frictionInventory.size(), 2u * perturbed.contacts.size());
  ASSERT_EQ(frictionRows.size(), perturbed.contacts.size());

  const auto frictionCoefficientForKey
      = [&](const dvbd::AvbdScalarRowKey& key) -> std::optional<double> {
    for (const dvbd::AvbdRigidContactManifoldPoint& contact :
         perturbed.contacts) {
      const dvbd::AvbdScalarRowKey contactKey
          = dvbd::makeAvbdEndpointPairRowKey(
              dvbd::AvbdScalarRowRole::FrictionTangent,
              contact.endpointA,
              contact.endpointB,
              contact.row,
              key.axis);
      if (contactKey == key) {
        return contact.frictionCoefficient;
      }
    }
    return std::nullopt;
  };

  for (const dvbd::AvbdScalarRowRecord& record : normalInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::ContactNormal);
    EXPECT_NEAR(
        record.state.lambda,
        expectedNormalLambda(record.descriptor.key),
        1e-12);
  }
  for (const dvbd::AvbdScalarRowRecord& record : frictionInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::FrictionTangent);
    const std::optional<double> frictionCoefficient
        = frictionCoefficientForKey(record.descriptor.key);
    ASSERT_TRUE(frictionCoefficient.has_value());
    const double forceLimit
        = *frictionCoefficient * expectedNormalLambda(record.descriptor.key);
    EXPECT_NEAR(record.descriptor.bounds.lower, -forceLimit, 1e-12);
    EXPECT_NEAR(record.descriptor.bounds.upper, forceLimit, 1e-12);
    EXPECT_NEAR(
        record.state.lambda,
        expectedFrictionLambda(record.descriptor.key),
        1e-12);
  }
  for (std::size_t i = 0; i < frictionRows.size(); ++i) {
    EXPECT_NEAR(
        frictionRows[i].first.state.lambda,
        frictionInventory[2u * i].state.lambda,
        1e-12);
    EXPECT_NEAR(
        frictionRows[i].second.state.lambda,
        frictionInventory[2u * i + 1u].state.lambda,
        1e-12);
  }
}

//==============================================================================
// A multi-top pile should preserve friction warm-start state across independent
// upper/lower dynamic contact pairs and independent lower/ground supports in
// the same live scene.
TEST(AvbdContact, WorldCollideMultiTopBoxPileFrictionRowsPersistAcrossSmallPose)
{
  sx::World world;

  constexpr double lowerXOffset = 0.55;
  constexpr double lowerHalfZ = 0.4;
  constexpr double upperHalfZ = 0.35;

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(3.0, 2.0, 0.5)));
  ground.setFriction(0.64);

  const auto addLower = [&](std::string_view name,
                            const double x,
                            const double friction) {
    sx::RigidBodyOptions options;
    options.position = Eigen::Vector3d(x, 0.0, 0.38);
    auto body = world.addRigidBody(name, options);
    body.setCollisionShape(
        sx::CollisionShape::makeBox(Eigen::Vector3d(0.45, 0.45, lowerHalfZ)));
    body.setFriction(friction);
    return body;
  };
  auto leftLower = addLower("left_lower", -lowerXOffset, 0.25);
  auto rightLower = addLower("right_lower", lowerXOffset, 0.36);

  const auto addUpper = [&](std::string_view name,
                            const double x,
                            const double friction) {
    sx::RigidBodyOptions options;
    options.position = Eigen::Vector3d(x, 0.0, 1.10);
    auto body = world.addRigidBody(name, options);
    body.setCollisionShape(
        sx::CollisionShape::makeBox(Eigen::Vector3d(0.35, 0.35, upperHalfZ)));
    body.setFriction(friction);
    return body;
  };
  auto leftUpper = addUpper("left_upper", -lowerXOffset, 0.49);
  auto rightUpper = addUpper("right_upper", lowerXOffset, 0.16);

  const auto setUpperPose
      = [](sx::RigidBody& body, const double x, const double height) {
          Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
          pose.translation() = Eigen::Vector3d(x, 0.0, height);
          body.setTransform(pose);
        };

  const auto buildSnapshot
      = [&](double upperHeight, dvbd::AvbdRigidWorldContactSnapshot& snapshot) {
          setUpperPose(leftUpper, -lowerXOffset, upperHeight);
          setUpperPose(rightUpper, lowerXOffset, upperHeight);

          const std::vector<sx::Contact> contacts = world.collide();
          ASSERT_GE(contacts.size(), 12u);

          snapshot = dvbd::buildAvbdRigidWorldContactSnapshot(
              dart::simulation::detail::registryOf(world),
              contacts,
              dvbd::AvbdRigidWorldContactOptions{});
          ASSERT_EQ(snapshot.contacts.size(), contacts.size());
        };

  dvbd::AvbdRigidWorldContactSnapshot reference;
  dvbd::AvbdRigidWorldContactSnapshot perturbed;
  buildSnapshot(1.10, reference);
  buildSnapshot(1.1025, perturbed);
  ASSERT_EQ(reference.contacts.size(), perturbed.contacts.size());

  using RowKey
      = std::pair<dvbd::AvbdContactEndpointId, dvbd::AvbdContactEndpointId>;
  struct ObservedRow
  {
    RowKey key;
    std::uint32_t row = 0u;
    Eigen::Vector3d canonicalLocalPoint = Eigen::Vector3d::Zero();
    double frictionCoefficient = 0.0;
  };

  const auto rowKey = [](const dvbd::AvbdRigidContactManifoldPoint& contact) {
    return dvbd::canonicalizeAvbdContactEndpoints(
        contact.endpointA, contact.endpointB);
  };
  const auto collectRows
      = [&](const dvbd::AvbdRigidWorldContactSnapshot& snapshot) {
          std::vector<ObservedRow> rows;
          rows.reserve(snapshot.contacts.size());
          for (const dvbd::AvbdRigidContactManifoldPoint& contact :
               snapshot.contacts) {
            EXPECT_EQ(
                dvbd::avbdContactFeatureKind(contact.endpointA.feature),
                dvbd::AvbdContactFeatureKind::Face);
            EXPECT_EQ(
                dvbd::avbdContactFeatureKind(contact.endpointB.feature),
                dvbd::AvbdContactFeatureKind::Face);
            rows.push_back(
                ObservedRow{
                    rowKey(contact),
                    contact.row,
                    dvbd::detail::avbdRigidWorldContactCanonicalLocalPoint(
                        snapshot, contact),
                    contact.frictionCoefficient});
          }
          return rows;
        };
  const auto observedAtRow
      = [](const std::vector<ObservedRow>& rows,
           const RowKey& key,
           std::uint32_t row) -> std::optional<ObservedRow> {
    for (const ObservedRow& observed : rows) {
      if (observed.key == key && observed.row == row) {
        return observed;
      }
    }
    return std::nullopt;
  };

  const std::vector<ObservedRow> referenceRows = collectRows(reference);
  const std::vector<ObservedRow> perturbedRows = collectRows(perturbed);
  ASSERT_EQ(referenceRows.size(), perturbedRows.size());

  std::vector<std::pair<RowKey, std::vector<std::uint32_t>>> groupedRows;
  std::vector<double> frictionCoefficients;
  for (const ObservedRow& referenceRow : referenceRows) {
    const std::optional<ObservedRow> perturbedRow
        = observedAtRow(perturbedRows, referenceRow.key, referenceRow.row);
    ASSERT_TRUE(perturbedRow.has_value());
    EXPECT_NEAR(
        referenceRow.canonicalLocalPoint.x(),
        perturbedRow->canonicalLocalPoint.x(),
        1e-8);
    EXPECT_NEAR(
        referenceRow.canonicalLocalPoint.y(),
        perturbedRow->canonicalLocalPoint.y(),
        1e-8);
    EXPECT_NEAR(
        referenceRow.frictionCoefficient,
        perturbedRow->frictionCoefficient,
        1e-12);
    frictionCoefficients.push_back(referenceRow.frictionCoefficient);

    auto groupIt = std::find_if(
        groupedRows.begin(),
        groupedRows.end(),
        [&](const std::pair<RowKey, std::vector<std::uint32_t>>& group) {
          return group.first == referenceRow.key;
        });
    if (groupIt == groupedRows.end()) {
      groupedRows.push_back({referenceRow.key, {referenceRow.row}});
    } else {
      groupIt->second.push_back(referenceRow.row);
    }
  }

  std::sort(frictionCoefficients.begin(), frictionCoefficients.end());
  frictionCoefficients.erase(
      std::unique(
          frictionCoefficients.begin(),
          frictionCoefficients.end(),
          [](double lhs, double rhs) { return std::abs(lhs - rhs) <= 1e-12; }),
      frictionCoefficients.end());
  EXPECT_GE(frictionCoefficients.size(), 4u);

  std::size_t sameFeatureGroups = 0u;
  for (auto& group : groupedRows) {
    std::vector<std::uint32_t>& rows = group.second;
    std::sort(rows.begin(), rows.end());
    rows.erase(std::unique(rows.begin(), rows.end()), rows.end());
    if (rows.size() >= 2u) {
      ++sameFeatureGroups;
      for (std::size_t row = 0; row < rows.size(); ++row) {
        EXPECT_EQ(rows[row], row);
      }
    }
  }
  EXPECT_GE(sameFeatureGroups, 4u);

  dvbd::AvbdScalarRowInventory normalInventory;
  dvbd::AvbdScalarRowInventory frictionInventory;
  std::vector<dvbd::AvbdRigidBodyPointPairRow> normalRows;
  std::vector<dvbd::AvbdRigidBodyPointPairFrictionRows> frictionRows;
  dvbd::AvbdRowWarmStartOptions warmStart;
  warmStart.alpha = 1.0;
  warmStart.gamma = 1.0;

  dvbd::buildAvbdRigidContactManifoldRows(
      reference.states,
      reference.contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      warmStart);

  ASSERT_EQ(normalInventory.size(), reference.contacts.size());
  ASSERT_EQ(frictionInventory.size(), 2u * reference.contacts.size());
  ASSERT_EQ(frictionRows.size(), reference.contacts.size());

  const auto keyFingerprint = [](const dvbd::AvbdScalarRowKey& key) {
    return static_cast<double>(
        key.objectA % 17u + 2u * (key.objectB % 19u) + 3u * (key.featureA % 23u)
        + 5u * (key.featureB % 29u));
  };
  const auto expectedNormalLambda = [&](const dvbd::AvbdScalarRowKey& key) {
    return 55.0 + static_cast<double>(key.row) + 0.01 * keyFingerprint(key);
  };
  const auto expectedFrictionLambda = [&](const dvbd::AvbdScalarRowKey& key) {
    return 5.0 + 0.5 * static_cast<double>(key.row)
           + 0.125 * static_cast<double>(key.axis) + 0.01 * keyFingerprint(key);
  };

  for (dvbd::AvbdScalarRowRecord& record : normalInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::ContactNormal);
    record.state.lambda = expectedNormalLambda(record.descriptor.key);
  }
  for (dvbd::AvbdScalarRowRecord& record : frictionInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::FrictionTangent);
    record.state.lambda = expectedFrictionLambda(record.descriptor.key);
  }

  dvbd::buildAvbdRigidContactManifoldRows(
      perturbed.states,
      perturbed.contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      warmStart);

  ASSERT_EQ(normalInventory.size(), perturbed.contacts.size());
  ASSERT_EQ(frictionInventory.size(), 2u * perturbed.contacts.size());
  ASSERT_EQ(frictionRows.size(), perturbed.contacts.size());

  const auto frictionCoefficientForKey
      = [&](const dvbd::AvbdScalarRowKey& key) -> std::optional<double> {
    for (const dvbd::AvbdRigidContactManifoldPoint& contact :
         perturbed.contacts) {
      const dvbd::AvbdScalarRowKey contactKey
          = dvbd::makeAvbdEndpointPairRowKey(
              dvbd::AvbdScalarRowRole::FrictionTangent,
              contact.endpointA,
              contact.endpointB,
              contact.row,
              key.axis);
      if (contactKey == key) {
        return contact.frictionCoefficient;
      }
    }
    return std::nullopt;
  };

  for (const dvbd::AvbdScalarRowRecord& record : normalInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::ContactNormal);
    EXPECT_NEAR(
        record.state.lambda,
        expectedNormalLambda(record.descriptor.key),
        1e-12);
  }
  for (const dvbd::AvbdScalarRowRecord& record : frictionInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::FrictionTangent);
    const std::optional<double> frictionCoefficient
        = frictionCoefficientForKey(record.descriptor.key);
    ASSERT_TRUE(frictionCoefficient.has_value());
    const double forceLimit
        = *frictionCoefficient * expectedNormalLambda(record.descriptor.key);
    EXPECT_NEAR(record.descriptor.bounds.lower, -forceLimit, 1e-12);
    EXPECT_NEAR(record.descriptor.bounds.upper, forceLimit, 1e-12);
    EXPECT_NEAR(
        record.state.lambda,
        expectedFrictionLambda(record.descriptor.key),
        1e-12);
  }
  for (std::size_t i = 0; i < frictionRows.size(); ++i) {
    EXPECT_NEAR(
        frictionRows[i].first.state.lambda,
        frictionInventory[2u * i].state.lambda,
        1e-12);
    EXPECT_NEAR(
        frictionRows[i].second.state.lambda,
        frictionInventory[2u * i + 1u].state.lambda,
        1e-12);
  }
}

//==============================================================================
// Contact-order replay should also hold for the multi-top pile where the live
// contact set contains independent upper/lower dynamic pairs and lower/ground
// supports.
TEST(AvbdContact, WorldCollideMultiTopBoxPileFrictionRowsIgnoreContactOrder)
{
  sx::World world;

  constexpr double lowerXOffset = 0.55;
  constexpr double lowerHalfZ = 0.4;
  constexpr double upperHalfZ = 0.35;

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(3.0, 2.0, 0.5)));
  ground.setFriction(0.64);

  const auto addLower = [&](std::string_view name,
                            const double x,
                            const double friction) {
    sx::RigidBodyOptions options;
    options.position = Eigen::Vector3d(x, 0.0, 0.38);
    auto body = world.addRigidBody(name, options);
    body.setCollisionShape(
        sx::CollisionShape::makeBox(Eigen::Vector3d(0.45, 0.45, lowerHalfZ)));
    body.setFriction(friction);
    return body;
  };
  auto leftLower = addLower("left_lower", -lowerXOffset, 0.25);
  auto rightLower = addLower("right_lower", lowerXOffset, 0.36);

  const auto addUpper = [&](std::string_view name,
                            const double x,
                            const double friction) {
    sx::RigidBodyOptions options;
    options.position = Eigen::Vector3d(x, 0.0, 1.10);
    auto body = world.addRigidBody(name, options);
    body.setCollisionShape(
        sx::CollisionShape::makeBox(Eigen::Vector3d(0.35, 0.35, upperHalfZ)));
    body.setFriction(friction);
    return body;
  };
  auto leftUpper = addUpper("left_upper", -lowerXOffset, 0.49);
  auto rightUpper = addUpper("right_upper", lowerXOffset, 0.16);

  const std::vector<sx::Contact> contacts = world.collide();
  ASSERT_GE(contacts.size(), 12u);

  std::vector<sx::Contact> reversedContacts = contacts;
  std::reverse(reversedContacts.begin(), reversedContacts.end());

  const auto buildSnapshot = [&](std::vector<sx::Contact> orderedContacts) {
    return dvbd::buildAvbdRigidWorldContactSnapshot(
        dart::simulation::detail::registryOf(world),
        orderedContacts,
        dvbd::AvbdRigidWorldContactOptions{});
  };

  const dvbd::AvbdRigidWorldContactSnapshot forward = buildSnapshot(contacts);
  const dvbd::AvbdRigidWorldContactSnapshot reversed
      = buildSnapshot(reversedContacts);

  ASSERT_EQ(forward.contacts.size(), contacts.size());
  ASSERT_EQ(reversed.contacts.size(), contacts.size());

  using RowKey
      = std::pair<dvbd::AvbdContactEndpointId, dvbd::AvbdContactEndpointId>;

  const auto rowKey = [](const dvbd::AvbdRigidContactManifoldPoint& contact) {
    return dvbd::canonicalizeAvbdContactEndpoints(
        contact.endpointA, contact.endpointB);
  };
  const auto matchingReversedContact
      = [&](const dvbd::AvbdRigidContactManifoldPoint& reference)
      -> const dvbd::AvbdRigidContactManifoldPoint* {
    const RowKey key = rowKey(reference);
    for (const dvbd::AvbdRigidContactManifoldPoint& contact :
         reversed.contacts) {
      if (rowKey(contact) == key
          && (contact.point - reference.point).norm() <= 1e-10) {
        return &contact;
      }
    }
    return nullptr;
  };

  std::vector<std::pair<RowKey, std::vector<std::uint32_t>>> groupedRows;
  std::vector<double> frictionCoefficients;
  for (const dvbd::AvbdRigidContactManifoldPoint& contact : forward.contacts) {
    EXPECT_EQ(
        dvbd::avbdContactFeatureKind(contact.endpointA.feature),
        dvbd::AvbdContactFeatureKind::Face);
    EXPECT_EQ(
        dvbd::avbdContactFeatureKind(contact.endpointB.feature),
        dvbd::AvbdContactFeatureKind::Face);

    const dvbd::AvbdRigidContactManifoldPoint* reversedContact
        = matchingReversedContact(contact);
    ASSERT_NE(reversedContact, nullptr);
    EXPECT_EQ(contact.row, reversedContact->row);
    EXPECT_NEAR(
        contact.frictionCoefficient,
        reversedContact->frictionCoefficient,
        1e-12);
    frictionCoefficients.push_back(contact.frictionCoefficient);

    const RowKey key = rowKey(contact);
    auto groupIt = std::find_if(
        groupedRows.begin(),
        groupedRows.end(),
        [&](const std::pair<RowKey, std::vector<std::uint32_t>>& group) {
          return group.first == key;
        });
    if (groupIt == groupedRows.end()) {
      groupedRows.push_back({key, {contact.row}});
    } else {
      groupIt->second.push_back(contact.row);
    }
  }

  std::sort(frictionCoefficients.begin(), frictionCoefficients.end());
  frictionCoefficients.erase(
      std::unique(
          frictionCoefficients.begin(),
          frictionCoefficients.end(),
          [](double lhs, double rhs) { return std::abs(lhs - rhs) <= 1e-12; }),
      frictionCoefficients.end());
  EXPECT_GE(frictionCoefficients.size(), 4u);

  std::size_t sameFeatureGroups = 0u;
  for (auto& group : groupedRows) {
    std::vector<std::uint32_t>& rows = group.second;
    std::sort(rows.begin(), rows.end());
    rows.erase(std::unique(rows.begin(), rows.end()), rows.end());
    if (rows.size() >= 2u) {
      ++sameFeatureGroups;
      for (std::size_t row = 0; row < rows.size(); ++row) {
        EXPECT_EQ(rows[row], row);
      }
    }
  }
  EXPECT_GE(sameFeatureGroups, 4u);

  dvbd::AvbdScalarRowInventory normalInventory;
  dvbd::AvbdScalarRowInventory frictionInventory;
  std::vector<dvbd::AvbdRigidBodyPointPairRow> normalRows;
  std::vector<dvbd::AvbdRigidBodyPointPairFrictionRows> frictionRows;
  dvbd::AvbdRowWarmStartOptions warmStart;
  warmStart.alpha = 1.0;
  warmStart.gamma = 1.0;

  dvbd::buildAvbdRigidContactManifoldRows(
      forward.states,
      forward.contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      warmStart);

  ASSERT_EQ(normalInventory.size(), forward.contacts.size());
  ASSERT_EQ(frictionInventory.size(), 2u * forward.contacts.size());
  ASSERT_EQ(frictionRows.size(), forward.contacts.size());

  const auto keyFingerprint = [](const dvbd::AvbdScalarRowKey& key) {
    return static_cast<double>(
        key.objectA % 17u + 2u * (key.objectB % 19u) + 3u * (key.featureA % 23u)
        + 5u * (key.featureB % 29u));
  };
  const auto expectedNormalLambda = [&](const dvbd::AvbdScalarRowKey& key) {
    return 65.0 + static_cast<double>(key.row) + 0.01 * keyFingerprint(key);
  };
  const auto expectedFrictionLambda = [&](const dvbd::AvbdScalarRowKey& key) {
    return 6.0 + 0.5 * static_cast<double>(key.row)
           + 0.125 * static_cast<double>(key.axis) + 0.01 * keyFingerprint(key);
  };

  for (dvbd::AvbdScalarRowRecord& record : normalInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::ContactNormal);
    record.state.lambda = expectedNormalLambda(record.descriptor.key);
  }
  for (dvbd::AvbdScalarRowRecord& record : frictionInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::FrictionTangent);
    record.state.lambda = expectedFrictionLambda(record.descriptor.key);
  }

  const auto frictionCoefficientForKey
      = [&](const dvbd::AvbdScalarRowKey& key) -> std::optional<double> {
    for (const dvbd::AvbdRigidContactManifoldPoint& contact :
         reversed.contacts) {
      const dvbd::AvbdScalarRowKey contactKey
          = dvbd::makeAvbdEndpointPairRowKey(
              dvbd::AvbdScalarRowRole::FrictionTangent,
              contact.endpointA,
              contact.endpointB,
              contact.row,
              key.axis);
      if (contactKey == key) {
        return contact.frictionCoefficient;
      }
    }
    return std::nullopt;
  };

  dvbd::buildAvbdRigidContactManifoldRows(
      reversed.states,
      reversed.contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      warmStart);

  ASSERT_EQ(normalInventory.size(), reversed.contacts.size());
  ASSERT_EQ(frictionInventory.size(), 2u * reversed.contacts.size());
  ASSERT_EQ(frictionRows.size(), reversed.contacts.size());

  for (const dvbd::AvbdScalarRowRecord& record : normalInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::ContactNormal);
    EXPECT_NEAR(
        record.state.lambda,
        expectedNormalLambda(record.descriptor.key),
        1e-12);
  }
  for (const dvbd::AvbdScalarRowRecord& record : frictionInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::FrictionTangent);
    const std::optional<double> frictionCoefficient
        = frictionCoefficientForKey(record.descriptor.key);
    ASSERT_TRUE(frictionCoefficient.has_value());
    const double forceLimit
        = *frictionCoefficient * expectedNormalLambda(record.descriptor.key);
    EXPECT_NEAR(record.descriptor.bounds.lower, -forceLimit, 1e-12);
    EXPECT_NEAR(record.descriptor.bounds.upper, forceLimit, 1e-12);
    EXPECT_NEAR(
        record.state.lambda,
        expectedFrictionLambda(record.descriptor.key),
        1e-12);
  }
  for (std::size_t i = 0; i < frictionRows.size(); ++i) {
    EXPECT_NEAR(
        frictionRows[i].first.state.lambda,
        frictionInventory[2u * i].state.lambda,
        1e-12);
    EXPECT_NEAR(
        frictionRows[i].second.state.lambda,
        frictionInventory[2u * i + 1u].state.lambda,
        1e-12);
  }
}

//==============================================================================
// Endpoint-order robustness should also hold for the multi-top pile: the
// canonical endpoint keys must preserve per-pair Coulomb bounds while
// projecting persisted paired tangent duals into the swapped physical tangent
// basis.
TEST(AvbdContact, WorldCollideMultiTopBoxPileFrictionRowsIgnoreEndpointOrder)
{
  sx::World world;

  constexpr double lowerXOffset = 0.55;
  constexpr double lowerHalfZ = 0.4;
  constexpr double upperHalfZ = 0.35;

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(3.0, 2.0, 0.5)));
  ground.setFriction(0.64);

  const auto addLower = [&](std::string_view name,
                            const double x,
                            const double friction) {
    sx::RigidBodyOptions options;
    options.position = Eigen::Vector3d(x, 0.0, 0.38);
    auto body = world.addRigidBody(name, options);
    body.setCollisionShape(
        sx::CollisionShape::makeBox(Eigen::Vector3d(0.45, 0.45, lowerHalfZ)));
    body.setFriction(friction);
    return body;
  };
  auto leftLower = addLower("left_lower", -lowerXOffset, 0.25);
  auto rightLower = addLower("right_lower", lowerXOffset, 0.36);

  const auto addUpper = [&](std::string_view name,
                            const double x,
                            const double friction) {
    sx::RigidBodyOptions options;
    options.position = Eigen::Vector3d(x, 0.0, 1.10);
    auto body = world.addRigidBody(name, options);
    body.setCollisionShape(
        sx::CollisionShape::makeBox(Eigen::Vector3d(0.35, 0.35, upperHalfZ)));
    body.setFriction(friction);
    return body;
  };
  auto leftUpper = addUpper("left_upper", -lowerXOffset, 0.49);
  auto rightUpper = addUpper("right_upper", lowerXOffset, 0.16);

  const std::vector<sx::Contact> contacts = world.collide();
  ASSERT_GE(contacts.size(), 12u);

  std::vector<sx::Contact> swappedContacts;
  swappedContacts.reserve(contacts.size());
  for (const sx::Contact& contact : contacts) {
    swappedContacts.push_back(swapEndpointOrder(contact));
  }

  const auto buildSnapshot = [&](std::vector<sx::Contact> orderedContacts) {
    return dvbd::buildAvbdRigidWorldContactSnapshot(
        dart::simulation::detail::registryOf(world),
        orderedContacts,
        dvbd::AvbdRigidWorldContactOptions{});
  };

  const dvbd::AvbdRigidWorldContactSnapshot forward = buildSnapshot(contacts);
  const dvbd::AvbdRigidWorldContactSnapshot swapped
      = buildSnapshot(swappedContacts);

  ASSERT_EQ(forward.contacts.size(), contacts.size());
  ASSERT_EQ(swapped.contacts.size(), contacts.size());

  using RowKey
      = std::pair<dvbd::AvbdContactEndpointId, dvbd::AvbdContactEndpointId>;

  const auto rowKey = [](const dvbd::AvbdRigidContactManifoldPoint& contact) {
    return dvbd::canonicalizeAvbdContactEndpoints(
        contact.endpointA, contact.endpointB);
  };

  std::vector<std::pair<RowKey, std::vector<std::uint32_t>>> groupedRows;
  std::vector<double> frictionCoefficients;
  for (const dvbd::AvbdRigidContactManifoldPoint& contact : forward.contacts) {
    EXPECT_EQ(
        dvbd::avbdContactFeatureKind(contact.endpointA.feature),
        dvbd::AvbdContactFeatureKind::Face);
    EXPECT_EQ(
        dvbd::avbdContactFeatureKind(contact.endpointB.feature),
        dvbd::AvbdContactFeatureKind::Face);
    frictionCoefficients.push_back(contact.frictionCoefficient);

    const RowKey key = rowKey(contact);
    auto groupIt = std::find_if(
        groupedRows.begin(),
        groupedRows.end(),
        [&](const std::pair<RowKey, std::vector<std::uint32_t>>& group) {
          return group.first == key;
        });
    if (groupIt == groupedRows.end()) {
      groupedRows.push_back({key, {contact.row}});
    } else {
      groupIt->second.push_back(contact.row);
    }
  }

  std::sort(frictionCoefficients.begin(), frictionCoefficients.end());
  frictionCoefficients.erase(
      std::unique(
          frictionCoefficients.begin(),
          frictionCoefficients.end(),
          [](double lhs, double rhs) { return std::abs(lhs - rhs) <= 1e-12; }),
      frictionCoefficients.end());
  EXPECT_GE(frictionCoefficients.size(), 4u);

  std::size_t sameFeatureGroups = 0u;
  for (auto& group : groupedRows) {
    std::vector<std::uint32_t>& rows = group.second;
    std::sort(rows.begin(), rows.end());
    rows.erase(std::unique(rows.begin(), rows.end()), rows.end());
    if (rows.size() >= 2u) {
      ++sameFeatureGroups;
      for (std::size_t row = 0; row < rows.size(); ++row) {
        EXPECT_EQ(rows[row], row);
      }
    }
  }
  EXPECT_GE(sameFeatureGroups, 4u);

  dvbd::AvbdScalarRowInventory normalInventory;
  dvbd::AvbdScalarRowInventory frictionInventory;
  std::vector<dvbd::AvbdRigidBodyPointPairRow> normalRows;
  std::vector<dvbd::AvbdRigidBodyPointPairFrictionRows> frictionRows;
  dvbd::AvbdRowWarmStartOptions warmStart;
  warmStart.alpha = 1.0;
  warmStart.gamma = 1.0;

  dvbd::buildAvbdRigidContactManifoldRows(
      forward.states,
      forward.contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      warmStart);

  ASSERT_EQ(normalInventory.size(), forward.contacts.size());
  ASSERT_EQ(frictionInventory.size(), 2u * forward.contacts.size());
  ASSERT_EQ(frictionRows.size(), forward.contacts.size());

  const auto keyFingerprint = [](const dvbd::AvbdScalarRowKey& key) {
    return static_cast<double>(
        key.objectA % 17u + 2u * (key.objectB % 19u) + 3u * (key.featureA % 23u)
        + 5u * (key.featureB % 29u));
  };
  const auto expectedNormalLambda = [&](const dvbd::AvbdScalarRowKey& key) {
    return 75.0 + static_cast<double>(key.row) + 0.01 * keyFingerprint(key);
  };
  const auto expectedFrictionLambda = [&](const dvbd::AvbdScalarRowKey& key) {
    return 7.0 + 0.5 * static_cast<double>(key.row)
           + 0.125 * static_cast<double>(key.axis) + 0.01 * keyFingerprint(key);
  };

  for (dvbd::AvbdScalarRowRecord& record : normalInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::ContactNormal);
    record.state.lambda = expectedNormalLambda(record.descriptor.key);
  }
  for (dvbd::AvbdScalarRowRecord& record : frictionInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::FrictionTangent);
    record.state.lambda = expectedFrictionLambda(record.descriptor.key);
  }

  struct PreviousFrictionRow
  {
    dvbd::AvbdScalarRowKey key;
    double lambda = 0.0;
    Eigen::Vector3d direction = Eigen::Vector3d::Zero();
  };
  std::vector<PreviousFrictionRow> previousFrictionRows;
  previousFrictionRows.reserve(frictionInventory.size());
  for (const dvbd::AvbdScalarRowRecord& record : frictionInventory.records()) {
    previousFrictionRows.push_back(
        PreviousFrictionRow{
            record.descriptor.key, record.state.lambda, record.direction});
  }

  const auto previousFrictionRowForKey
      = [&](const dvbd::AvbdScalarRowKey& key) -> const PreviousFrictionRow* {
    for (const PreviousFrictionRow& previous : previousFrictionRows) {
      if (previous.key == key) {
        return &previous;
      }
    }
    return nullptr;
  };

  const auto frictionCoefficientForKey
      = [&](const dvbd::AvbdScalarRowKey& key) -> std::optional<double> {
    for (const dvbd::AvbdRigidContactManifoldPoint& contact :
         swapped.contacts) {
      const dvbd::AvbdScalarRowKey contactKey
          = dvbd::makeAvbdEndpointPairRowKey(
              dvbd::AvbdScalarRowRole::FrictionTangent,
              contact.endpointA,
              contact.endpointB,
              contact.row,
              key.axis);
      if (contactKey == key) {
        return contact.frictionCoefficient;
      }
    }
    return std::nullopt;
  };

  dvbd::buildAvbdRigidContactManifoldRows(
      swapped.states,
      swapped.contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      warmStart);

  ASSERT_EQ(normalInventory.size(), swapped.contacts.size());
  ASSERT_EQ(frictionInventory.size(), 2u * swapped.contacts.size());
  ASSERT_EQ(frictionRows.size(), swapped.contacts.size());

  for (const dvbd::AvbdScalarRowRecord& record : normalInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::ContactNormal);
    EXPECT_NEAR(
        record.state.lambda,
        expectedNormalLambda(record.descriptor.key),
        1e-12);
  }
  for (std::size_t i = 0; i < frictionRows.size(); ++i) {
    const dvbd::AvbdScalarRowRecord& firstRecord = frictionInventory[2u * i];
    const dvbd::AvbdScalarRowRecord& secondRecord
        = frictionInventory[2u * i + 1u];
    EXPECT_EQ(
        firstRecord.descriptor.key.role,
        dvbd::AvbdScalarRowRole::FrictionTangent);
    EXPECT_EQ(
        secondRecord.descriptor.key.role,
        dvbd::AvbdScalarRowRole::FrictionTangent);

    const std::optional<double> frictionCoefficient
        = frictionCoefficientForKey(firstRecord.descriptor.key);
    ASSERT_TRUE(frictionCoefficient.has_value());
    const double forceLimit
        = *frictionCoefficient
          * expectedNormalLambda(firstRecord.descriptor.key);
    EXPECT_NEAR(firstRecord.descriptor.bounds.lower, -forceLimit, 1e-12);
    EXPECT_NEAR(firstRecord.descriptor.bounds.upper, forceLimit, 1e-12);
    EXPECT_NEAR(secondRecord.descriptor.bounds.lower, -forceLimit, 1e-12);
    EXPECT_NEAR(secondRecord.descriptor.bounds.upper, forceLimit, 1e-12);

    const PreviousFrictionRow* previousFirst
        = previousFrictionRowForKey(firstRecord.descriptor.key);
    const PreviousFrictionRow* previousSecond
        = previousFrictionRowForKey(secondRecord.descriptor.key);
    ASSERT_NE(previousFirst, nullptr);
    ASSERT_NE(previousSecond, nullptr);

    const Eigen::Vector3d worldDual
        = previousFirst->lambda * previousFirst->direction
          + previousSecond->lambda * previousSecond->direction;
    EXPECT_NEAR(
        firstRecord.state.lambda, worldDual.dot(firstRecord.direction), 1e-12);
    EXPECT_NEAR(
        secondRecord.state.lambda,
        worldDual.dot(secondRecord.direction),
        1e-12);
    EXPECT_NEAR(
        frictionRows[i].first.state.lambda, firstRecord.state.lambda, 1e-12);
    EXPECT_NEAR(
        frictionRows[i].second.state.lambda, secondRecord.state.lambda, 1e-12);
  }
}

//==============================================================================
// Contact-order replay should also hold for the wider pile with several
// simultaneous endpoint pairs and distinct per-pair friction bounds.
TEST(AvbdContact, WorldCollideBoxPileFrictionRowsIgnoreContactOrder)
{
  sx::World world;

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(2.5, 2.0, 0.5)));
  ground.setFriction(0.64);

  sx::RigidBodyOptions leftLowerOptions;
  leftLowerOptions.position = Eigen::Vector3d(-0.45, 0.0, 0.38);
  auto leftLower = world.addRigidBody("left_lower", leftLowerOptions);
  leftLower.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.4, 0.4, 0.4)));
  leftLower.setFriction(0.25);

  sx::RigidBodyOptions rightLowerOptions;
  rightLowerOptions.position = Eigen::Vector3d(0.45, 0.0, 0.38);
  auto rightLower = world.addRigidBody("right_lower", rightLowerOptions);
  rightLower.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.4, 0.4, 0.4)));
  rightLower.setFriction(0.36);

  sx::RigidBodyOptions topOptions;
  topOptions.position = Eigen::Vector3d(0.0, 0.0, 1.10);
  auto top = world.addRigidBody("top", topOptions);
  top.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.7, 0.4, 0.35)));
  top.setFriction(0.49);

  const std::vector<sx::Contact> contacts = world.collide();
  ASSERT_GE(contacts.size(), 12u);

  std::vector<sx::Contact> reversedContacts = contacts;
  std::reverse(reversedContacts.begin(), reversedContacts.end());

  const auto buildSnapshot = [&](std::vector<sx::Contact> orderedContacts) {
    return dvbd::buildAvbdRigidWorldContactSnapshot(
        dart::simulation::detail::registryOf(world),
        orderedContacts,
        dvbd::AvbdRigidWorldContactOptions{});
  };

  const dvbd::AvbdRigidWorldContactSnapshot forward = buildSnapshot(contacts);
  const dvbd::AvbdRigidWorldContactSnapshot reversed
      = buildSnapshot(reversedContacts);

  ASSERT_EQ(forward.contacts.size(), contacts.size());
  ASSERT_EQ(reversed.contacts.size(), contacts.size());

  using RowKey
      = std::pair<dvbd::AvbdContactEndpointId, dvbd::AvbdContactEndpointId>;

  const auto rowKey = [](const dvbd::AvbdRigidContactManifoldPoint& contact) {
    return dvbd::canonicalizeAvbdContactEndpoints(
        contact.endpointA, contact.endpointB);
  };
  const auto matchingReversedContact
      = [&](const dvbd::AvbdRigidContactManifoldPoint& reference)
      -> const dvbd::AvbdRigidContactManifoldPoint* {
    const RowKey key = rowKey(reference);
    for (const dvbd::AvbdRigidContactManifoldPoint& contact :
         reversed.contacts) {
      if (rowKey(contact) == key
          && (contact.point - reference.point).norm() <= 1e-10) {
        return &contact;
      }
    }
    return nullptr;
  };

  std::vector<std::pair<RowKey, std::vector<std::uint32_t>>> groupedRows;
  std::vector<double> frictionCoefficients;
  for (const dvbd::AvbdRigidContactManifoldPoint& contact : forward.contacts) {
    EXPECT_EQ(
        dvbd::avbdContactFeatureKind(contact.endpointA.feature),
        dvbd::AvbdContactFeatureKind::Face);
    EXPECT_EQ(
        dvbd::avbdContactFeatureKind(contact.endpointB.feature),
        dvbd::AvbdContactFeatureKind::Face);

    const dvbd::AvbdRigidContactManifoldPoint* reversedContact
        = matchingReversedContact(contact);
    ASSERT_NE(reversedContact, nullptr);
    EXPECT_EQ(contact.row, reversedContact->row);
    EXPECT_NEAR(
        contact.frictionCoefficient,
        reversedContact->frictionCoefficient,
        1e-12);
    frictionCoefficients.push_back(contact.frictionCoefficient);

    const RowKey key = rowKey(contact);
    auto groupIt = std::find_if(
        groupedRows.begin(),
        groupedRows.end(),
        [&](const std::pair<RowKey, std::vector<std::uint32_t>>& group) {
          return group.first == key;
        });
    if (groupIt == groupedRows.end()) {
      groupedRows.push_back({key, {contact.row}});
    } else {
      groupIt->second.push_back(contact.row);
    }
  }

  std::sort(frictionCoefficients.begin(), frictionCoefficients.end());
  frictionCoefficients.erase(
      std::unique(
          frictionCoefficients.begin(),
          frictionCoefficients.end(),
          [](double lhs, double rhs) { return std::abs(lhs - rhs) <= 1e-12; }),
      frictionCoefficients.end());
  EXPECT_GE(frictionCoefficients.size(), 3u);

  std::size_t sameFeatureGroups = 0u;
  for (auto& group : groupedRows) {
    std::vector<std::uint32_t>& rows = group.second;
    std::sort(rows.begin(), rows.end());
    rows.erase(std::unique(rows.begin(), rows.end()), rows.end());
    if (rows.size() >= 2u) {
      ++sameFeatureGroups;
      for (std::size_t row = 0; row < rows.size(); ++row) {
        EXPECT_EQ(rows[row], row);
      }
    }
  }
  EXPECT_GE(sameFeatureGroups, 4u);

  dvbd::AvbdScalarRowInventory normalInventory;
  dvbd::AvbdScalarRowInventory frictionInventory;
  std::vector<dvbd::AvbdRigidBodyPointPairRow> normalRows;
  std::vector<dvbd::AvbdRigidBodyPointPairFrictionRows> frictionRows;
  dvbd::AvbdRowWarmStartOptions warmStart;
  warmStart.alpha = 1.0;
  warmStart.gamma = 1.0;

  dvbd::buildAvbdRigidContactManifoldRows(
      forward.states,
      forward.contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      warmStart);

  ASSERT_EQ(normalInventory.size(), forward.contacts.size());
  ASSERT_EQ(frictionInventory.size(), 2u * forward.contacts.size());
  ASSERT_EQ(frictionRows.size(), forward.contacts.size());

  const auto keyFingerprint = [](const dvbd::AvbdScalarRowKey& key) {
    return static_cast<double>(
        key.objectA % 17u + 2u * (key.objectB % 19u) + 3u * (key.featureA % 23u)
        + 5u * (key.featureB % 29u));
  };
  const auto expectedNormalLambda = [&](const dvbd::AvbdScalarRowKey& key) {
    return 50.0 + static_cast<double>(key.row) + 0.01 * keyFingerprint(key);
  };
  const auto expectedFrictionLambda = [&](const dvbd::AvbdScalarRowKey& key) {
    return 4.0 + 0.5 * static_cast<double>(key.row)
           + 0.125 * static_cast<double>(key.axis) + 0.01 * keyFingerprint(key);
  };

  for (dvbd::AvbdScalarRowRecord& record : normalInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::ContactNormal);
    record.state.lambda = expectedNormalLambda(record.descriptor.key);
  }
  for (dvbd::AvbdScalarRowRecord& record : frictionInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::FrictionTangent);
    record.state.lambda = expectedFrictionLambda(record.descriptor.key);
  }

  const auto frictionCoefficientForKey
      = [&](const dvbd::AvbdScalarRowKey& key) -> std::optional<double> {
    for (const dvbd::AvbdRigidContactManifoldPoint& contact :
         reversed.contacts) {
      const dvbd::AvbdScalarRowKey contactKey
          = dvbd::makeAvbdEndpointPairRowKey(
              dvbd::AvbdScalarRowRole::FrictionTangent,
              contact.endpointA,
              contact.endpointB,
              contact.row,
              key.axis);
      if (contactKey == key) {
        return contact.frictionCoefficient;
      }
    }
    return std::nullopt;
  };

  dvbd::buildAvbdRigidContactManifoldRows(
      reversed.states,
      reversed.contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      warmStart);

  ASSERT_EQ(normalInventory.size(), reversed.contacts.size());
  ASSERT_EQ(frictionInventory.size(), 2u * reversed.contacts.size());
  ASSERT_EQ(frictionRows.size(), reversed.contacts.size());

  for (const dvbd::AvbdScalarRowRecord& record : normalInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::ContactNormal);
    EXPECT_NEAR(
        record.state.lambda,
        expectedNormalLambda(record.descriptor.key),
        1e-12);
  }
  for (const dvbd::AvbdScalarRowRecord& record : frictionInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::FrictionTangent);
    const std::optional<double> frictionCoefficient
        = frictionCoefficientForKey(record.descriptor.key);
    ASSERT_TRUE(frictionCoefficient.has_value());
    const double forceLimit
        = *frictionCoefficient * expectedNormalLambda(record.descriptor.key);
    EXPECT_NEAR(record.descriptor.bounds.lower, -forceLimit, 1e-12);
    EXPECT_NEAR(record.descriptor.bounds.upper, forceLimit, 1e-12);
    EXPECT_NEAR(
        record.state.lambda,
        expectedFrictionLambda(record.descriptor.key),
        1e-12);
  }
  for (std::size_t i = 0; i < frictionRows.size(); ++i) {
    EXPECT_NEAR(
        frictionRows[i].first.state.lambda,
        frictionInventory[2u * i].state.lambda,
        1e-12);
    EXPECT_NEAR(
        frictionRows[i].second.state.lambda,
        frictionInventory[2u * i + 1u].state.lambda,
        1e-12);
  }
}

//==============================================================================
// Endpoint-order robustness should also hold for a wider pile with more than
// two simultaneous endpoint pairs and distinct per-pair friction bounds.
TEST(AvbdContact, WorldCollideBoxPileFrictionRowsIgnoreEndpointOrder)
{
  sx::World world;

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(2.5, 2.0, 0.5)));
  ground.setFriction(0.64);

  sx::RigidBodyOptions leftLowerOptions;
  leftLowerOptions.position = Eigen::Vector3d(-0.45, 0.0, 0.38);
  auto leftLower = world.addRigidBody("left_lower", leftLowerOptions);
  leftLower.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.4, 0.4, 0.4)));
  leftLower.setFriction(0.25);

  sx::RigidBodyOptions rightLowerOptions;
  rightLowerOptions.position = Eigen::Vector3d(0.45, 0.0, 0.38);
  auto rightLower = world.addRigidBody("right_lower", rightLowerOptions);
  rightLower.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.4, 0.4, 0.4)));
  rightLower.setFriction(0.36);

  sx::RigidBodyOptions topOptions;
  topOptions.position = Eigen::Vector3d(0.0, 0.0, 1.10);
  auto top = world.addRigidBody("top", topOptions);
  top.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.7, 0.4, 0.35)));
  top.setFriction(0.49);

  const std::vector<sx::Contact> contacts = world.collide();
  ASSERT_GE(contacts.size(), 12u);

  std::vector<sx::Contact> swappedContacts;
  swappedContacts.reserve(contacts.size());
  for (const sx::Contact& contact : contacts) {
    swappedContacts.push_back(swapEndpointOrder(contact));
  }

  const auto buildSnapshot = [&](std::vector<sx::Contact> orderedContacts) {
    return dvbd::buildAvbdRigidWorldContactSnapshot(
        dart::simulation::detail::registryOf(world),
        orderedContacts,
        dvbd::AvbdRigidWorldContactOptions{});
  };

  const dvbd::AvbdRigidWorldContactSnapshot forward = buildSnapshot(contacts);
  const dvbd::AvbdRigidWorldContactSnapshot swapped
      = buildSnapshot(swappedContacts);

  ASSERT_EQ(forward.contacts.size(), contacts.size());
  ASSERT_EQ(swapped.contacts.size(), contacts.size());

  using RowKey
      = std::pair<dvbd::AvbdContactEndpointId, dvbd::AvbdContactEndpointId>;

  const auto rowKey = [](const dvbd::AvbdRigidContactManifoldPoint& contact) {
    return dvbd::canonicalizeAvbdContactEndpoints(
        contact.endpointA, contact.endpointB);
  };

  std::vector<std::pair<RowKey, std::vector<std::uint32_t>>> groupedRows;
  std::vector<double> frictionCoefficients;
  for (const dvbd::AvbdRigidContactManifoldPoint& contact : forward.contacts) {
    EXPECT_EQ(
        dvbd::avbdContactFeatureKind(contact.endpointA.feature),
        dvbd::AvbdContactFeatureKind::Face);
    EXPECT_EQ(
        dvbd::avbdContactFeatureKind(contact.endpointB.feature),
        dvbd::AvbdContactFeatureKind::Face);
    frictionCoefficients.push_back(contact.frictionCoefficient);

    const RowKey key = rowKey(contact);
    auto groupIt = std::find_if(
        groupedRows.begin(),
        groupedRows.end(),
        [&](const std::pair<RowKey, std::vector<std::uint32_t>>& group) {
          return group.first == key;
        });
    if (groupIt == groupedRows.end()) {
      groupedRows.push_back({key, {contact.row}});
    } else {
      groupIt->second.push_back(contact.row);
    }
  }

  std::sort(frictionCoefficients.begin(), frictionCoefficients.end());
  frictionCoefficients.erase(
      std::unique(
          frictionCoefficients.begin(),
          frictionCoefficients.end(),
          [](double lhs, double rhs) { return std::abs(lhs - rhs) <= 1e-12; }),
      frictionCoefficients.end());
  EXPECT_GE(frictionCoefficients.size(), 3u);

  std::size_t sameFeatureGroups = 0u;
  for (auto& group : groupedRows) {
    std::vector<std::uint32_t>& rows = group.second;
    std::sort(rows.begin(), rows.end());
    rows.erase(std::unique(rows.begin(), rows.end()), rows.end());
    if (rows.size() >= 2u) {
      ++sameFeatureGroups;
      for (std::size_t row = 0; row < rows.size(); ++row) {
        EXPECT_EQ(rows[row], row);
      }
    }
  }
  EXPECT_GE(sameFeatureGroups, 4u);

  dvbd::AvbdScalarRowInventory normalInventory;
  dvbd::AvbdScalarRowInventory frictionInventory;
  std::vector<dvbd::AvbdRigidBodyPointPairRow> normalRows;
  std::vector<dvbd::AvbdRigidBodyPointPairFrictionRows> frictionRows;
  dvbd::AvbdRowWarmStartOptions warmStart;
  warmStart.alpha = 1.0;
  warmStart.gamma = 1.0;

  dvbd::buildAvbdRigidContactManifoldRows(
      forward.states,
      forward.contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      warmStart);

  ASSERT_EQ(normalInventory.size(), forward.contacts.size());
  ASSERT_EQ(frictionInventory.size(), 2u * forward.contacts.size());
  ASSERT_EQ(frictionRows.size(), forward.contacts.size());

  const auto keyFingerprint = [](const dvbd::AvbdScalarRowKey& key) {
    return static_cast<double>(
        key.objectA % 17u + 2u * (key.objectB % 19u) + 3u * (key.featureA % 23u)
        + 5u * (key.featureB % 29u));
  };
  const auto expectedNormalLambda = [&](const dvbd::AvbdScalarRowKey& key) {
    return 45.0 + static_cast<double>(key.row) + 0.01 * keyFingerprint(key);
  };
  const auto expectedFrictionLambda = [&](const dvbd::AvbdScalarRowKey& key) {
    return 3.5 + 0.5 * static_cast<double>(key.row)
           + 0.125 * static_cast<double>(key.axis) + 0.01 * keyFingerprint(key);
  };

  for (dvbd::AvbdScalarRowRecord& record : normalInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::ContactNormal);
    record.state.lambda = expectedNormalLambda(record.descriptor.key);
  }
  for (dvbd::AvbdScalarRowRecord& record : frictionInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::FrictionTangent);
    record.state.lambda = expectedFrictionLambda(record.descriptor.key);
  }

  struct PreviousFrictionRow
  {
    dvbd::AvbdScalarRowKey key;
    double lambda = 0.0;
    Eigen::Vector3d direction = Eigen::Vector3d::Zero();
  };
  std::vector<PreviousFrictionRow> previousFrictionRows;
  previousFrictionRows.reserve(frictionInventory.size());
  for (const dvbd::AvbdScalarRowRecord& record : frictionInventory.records()) {
    previousFrictionRows.push_back(
        PreviousFrictionRow{
            record.descriptor.key, record.state.lambda, record.direction});
  }

  const auto previousFrictionRowForKey
      = [&](const dvbd::AvbdScalarRowKey& key) -> const PreviousFrictionRow* {
    for (const PreviousFrictionRow& previous : previousFrictionRows) {
      if (previous.key == key) {
        return &previous;
      }
    }
    return nullptr;
  };

  const auto frictionCoefficientForKey
      = [&](const dvbd::AvbdScalarRowKey& key) -> std::optional<double> {
    for (const dvbd::AvbdRigidContactManifoldPoint& contact :
         swapped.contacts) {
      const dvbd::AvbdScalarRowKey contactKey
          = dvbd::makeAvbdEndpointPairRowKey(
              dvbd::AvbdScalarRowRole::FrictionTangent,
              contact.endpointA,
              contact.endpointB,
              contact.row,
              key.axis);
      if (contactKey == key) {
        return contact.frictionCoefficient;
      }
    }
    return std::nullopt;
  };

  dvbd::buildAvbdRigidContactManifoldRows(
      swapped.states,
      swapped.contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      warmStart);

  ASSERT_EQ(normalInventory.size(), swapped.contacts.size());
  ASSERT_EQ(frictionInventory.size(), 2u * swapped.contacts.size());
  ASSERT_EQ(frictionRows.size(), swapped.contacts.size());

  for (const dvbd::AvbdScalarRowRecord& record : normalInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::ContactNormal);
    EXPECT_NEAR(
        record.state.lambda,
        expectedNormalLambda(record.descriptor.key),
        1e-12);
  }
  for (std::size_t i = 0; i < frictionRows.size(); ++i) {
    const dvbd::AvbdScalarRowRecord& firstRecord = frictionInventory[2u * i];
    const dvbd::AvbdScalarRowRecord& secondRecord
        = frictionInventory[2u * i + 1u];
    EXPECT_EQ(
        firstRecord.descriptor.key.role,
        dvbd::AvbdScalarRowRole::FrictionTangent);
    EXPECT_EQ(
        secondRecord.descriptor.key.role,
        dvbd::AvbdScalarRowRole::FrictionTangent);

    const std::optional<double> frictionCoefficient
        = frictionCoefficientForKey(firstRecord.descriptor.key);
    ASSERT_TRUE(frictionCoefficient.has_value());
    const double forceLimit
        = *frictionCoefficient
          * expectedNormalLambda(firstRecord.descriptor.key);
    EXPECT_NEAR(firstRecord.descriptor.bounds.lower, -forceLimit, 1e-12);
    EXPECT_NEAR(firstRecord.descriptor.bounds.upper, forceLimit, 1e-12);
    EXPECT_NEAR(secondRecord.descriptor.bounds.lower, -forceLimit, 1e-12);
    EXPECT_NEAR(secondRecord.descriptor.bounds.upper, forceLimit, 1e-12);

    const PreviousFrictionRow* previousFirst
        = previousFrictionRowForKey(firstRecord.descriptor.key);
    const PreviousFrictionRow* previousSecond
        = previousFrictionRowForKey(secondRecord.descriptor.key);
    ASSERT_NE(previousFirst, nullptr);
    ASSERT_NE(previousSecond, nullptr);

    const Eigen::Vector3d worldDual
        = previousFirst->lambda * previousFirst->direction
          + previousSecond->lambda * previousSecond->direction;
    EXPECT_NEAR(
        firstRecord.state.lambda, worldDual.dot(firstRecord.direction), 1e-12);
    EXPECT_NEAR(
        secondRecord.state.lambda,
        worldDual.dot(secondRecord.direction),
        1e-12);
    EXPECT_NEAR(
        frictionRows[i].first.state.lambda, firstRecord.state.lambda, 1e-12);
    EXPECT_NEAR(
        frictionRows[i].second.state.lambda, secondRecord.state.lambda, 1e-12);
  }
}

//==============================================================================
// Endpoint-order robustness for friction rows also needs coverage across more
// than one simultaneous canonical endpoint pair, matching the first stacked
// static/dynamic and dynamic/dynamic manifold combination.
TEST(AvbdContact, WorldCollideStackedManifoldsFrictionRowsIgnoreEndpointOrder)
{
  sx::World world;

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(2.0, 2.0, 0.5)));
  ground.setFriction(0.64);

  sx::RigidBodyOptions lowerOptions;
  lowerOptions.position = Eigen::Vector3d(0.0, 0.0, 0.45);
  auto lower = world.addRigidBody("lower", lowerOptions);
  lower.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.5, 0.5, 0.5)));
  lower.setFriction(0.25);

  sx::RigidBodyOptions upperOptions;
  upperOptions.position = Eigen::Vector3d(0.0, 0.0, 1.35);
  auto upper = world.addRigidBody("upper", upperOptions);
  upper.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.5, 0.5, 0.5)));
  upper.setFriction(0.36);

  const std::vector<sx::Contact> contacts = world.collide();
  ASSERT_GE(contacts.size(), 8u);

  std::vector<sx::Contact> swappedContacts;
  swappedContacts.reserve(contacts.size());
  for (const sx::Contact& contact : contacts) {
    swappedContacts.push_back(swapEndpointOrder(contact));
  }

  const auto buildSnapshot = [&](std::vector<sx::Contact> orderedContacts) {
    return dvbd::buildAvbdRigidWorldContactSnapshot(
        dart::simulation::detail::registryOf(world),
        orderedContacts,
        dvbd::AvbdRigidWorldContactOptions{});
  };

  const dvbd::AvbdRigidWorldContactSnapshot forward = buildSnapshot(contacts);
  const dvbd::AvbdRigidWorldContactSnapshot swapped
      = buildSnapshot(swappedContacts);

  ASSERT_EQ(forward.contacts.size(), contacts.size());
  ASSERT_EQ(swapped.contacts.size(), contacts.size());

  using RowKey
      = std::pair<dvbd::AvbdContactEndpointId, dvbd::AvbdContactEndpointId>;

  const auto rowKey = [](const dvbd::AvbdRigidContactManifoldPoint& contact) {
    return dvbd::canonicalizeAvbdContactEndpoints(
        contact.endpointA, contact.endpointB);
  };

  std::vector<std::pair<RowKey, std::vector<std::uint32_t>>> groupedRows;
  std::vector<double> frictionCoefficients;
  for (const dvbd::AvbdRigidContactManifoldPoint& contact : forward.contacts) {
    EXPECT_EQ(
        dvbd::avbdContactFeatureKind(contact.endpointA.feature),
        dvbd::AvbdContactFeatureKind::Face);
    EXPECT_EQ(
        dvbd::avbdContactFeatureKind(contact.endpointB.feature),
        dvbd::AvbdContactFeatureKind::Face);
    frictionCoefficients.push_back(contact.frictionCoefficient);

    const RowKey key = rowKey(contact);
    auto groupIt = std::find_if(
        groupedRows.begin(),
        groupedRows.end(),
        [&](const std::pair<RowKey, std::vector<std::uint32_t>>& group) {
          return group.first == key;
        });
    if (groupIt == groupedRows.end()) {
      groupedRows.push_back({key, {contact.row}});
    } else {
      groupIt->second.push_back(contact.row);
    }
  }

  std::sort(frictionCoefficients.begin(), frictionCoefficients.end());
  frictionCoefficients.erase(
      std::unique(
          frictionCoefficients.begin(),
          frictionCoefficients.end(),
          [](double lhs, double rhs) { return std::abs(lhs - rhs) <= 1e-12; }),
      frictionCoefficients.end());
  EXPECT_GE(frictionCoefficients.size(), 2u);

  std::size_t sameFeatureGroups = 0u;
  for (auto& group : groupedRows) {
    std::vector<std::uint32_t>& rows = group.second;
    std::sort(rows.begin(), rows.end());
    rows.erase(std::unique(rows.begin(), rows.end()), rows.end());
    if (rows.size() >= 2u) {
      ++sameFeatureGroups;
      for (std::size_t row = 0; row < rows.size(); ++row) {
        EXPECT_EQ(rows[row], row);
      }
    }
  }
  EXPECT_GE(sameFeatureGroups, 2u);

  dvbd::AvbdScalarRowInventory normalInventory;
  dvbd::AvbdScalarRowInventory frictionInventory;
  std::vector<dvbd::AvbdRigidBodyPointPairRow> normalRows;
  std::vector<dvbd::AvbdRigidBodyPointPairFrictionRows> frictionRows;
  dvbd::AvbdRowWarmStartOptions warmStart;
  warmStart.alpha = 1.0;
  warmStart.gamma = 1.0;

  dvbd::buildAvbdRigidContactManifoldRows(
      forward.states,
      forward.contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      warmStart);

  ASSERT_EQ(normalInventory.size(), forward.contacts.size());
  ASSERT_EQ(frictionInventory.size(), 2u * forward.contacts.size());
  ASSERT_EQ(frictionRows.size(), forward.contacts.size());

  const auto keyFingerprint = [](const dvbd::AvbdScalarRowKey& key) {
    return static_cast<double>(
        key.objectA % 17u + 2u * (key.objectB % 19u) + 3u * (key.featureA % 23u)
        + 5u * (key.featureB % 29u));
  };
  const auto expectedNormalLambda = [&](const dvbd::AvbdScalarRowKey& key) {
    return 35.0 + static_cast<double>(key.row) + 0.01 * keyFingerprint(key);
  };
  const auto expectedFrictionLambda = [&](const dvbd::AvbdScalarRowKey& key) {
    return 2.5 + 0.5 * static_cast<double>(key.row)
           + 0.125 * static_cast<double>(key.axis) + 0.01 * keyFingerprint(key);
  };

  for (dvbd::AvbdScalarRowRecord& record : normalInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::ContactNormal);
    record.state.lambda = expectedNormalLambda(record.descriptor.key);
  }
  for (dvbd::AvbdScalarRowRecord& record : frictionInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::FrictionTangent);
    record.state.lambda = expectedFrictionLambda(record.descriptor.key);
  }

  struct PreviousFrictionRow
  {
    dvbd::AvbdScalarRowKey key;
    double lambda = 0.0;
    Eigen::Vector3d direction = Eigen::Vector3d::Zero();
  };
  std::vector<PreviousFrictionRow> previousFrictionRows;
  previousFrictionRows.reserve(frictionInventory.size());
  for (const dvbd::AvbdScalarRowRecord& record : frictionInventory.records()) {
    previousFrictionRows.push_back(
        PreviousFrictionRow{
            record.descriptor.key, record.state.lambda, record.direction});
  }

  const auto previousFrictionRowForKey
      = [&](const dvbd::AvbdScalarRowKey& key) -> const PreviousFrictionRow* {
    for (const PreviousFrictionRow& previous : previousFrictionRows) {
      if (previous.key == key) {
        return &previous;
      }
    }
    return nullptr;
  };

  const auto frictionCoefficientForKey
      = [&](const dvbd::AvbdScalarRowKey& key) -> std::optional<double> {
    for (const dvbd::AvbdRigidContactManifoldPoint& contact :
         swapped.contacts) {
      const dvbd::AvbdScalarRowKey contactKey
          = dvbd::makeAvbdEndpointPairRowKey(
              dvbd::AvbdScalarRowRole::FrictionTangent,
              contact.endpointA,
              contact.endpointB,
              contact.row,
              key.axis);
      if (contactKey == key) {
        return contact.frictionCoefficient;
      }
    }
    return std::nullopt;
  };

  dvbd::buildAvbdRigidContactManifoldRows(
      swapped.states,
      swapped.contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      warmStart);

  ASSERT_EQ(normalInventory.size(), swapped.contacts.size());
  ASSERT_EQ(frictionInventory.size(), 2u * swapped.contacts.size());
  ASSERT_EQ(frictionRows.size(), swapped.contacts.size());

  for (const dvbd::AvbdScalarRowRecord& record : normalInventory.records()) {
    EXPECT_EQ(
        record.descriptor.key.role, dvbd::AvbdScalarRowRole::ContactNormal);
    EXPECT_NEAR(
        record.state.lambda,
        expectedNormalLambda(record.descriptor.key),
        1e-12);
  }
  for (std::size_t i = 0; i < frictionRows.size(); ++i) {
    const dvbd::AvbdScalarRowRecord& firstRecord = frictionInventory[2u * i];
    const dvbd::AvbdScalarRowRecord& secondRecord
        = frictionInventory[2u * i + 1u];
    EXPECT_EQ(
        firstRecord.descriptor.key.role,
        dvbd::AvbdScalarRowRole::FrictionTangent);
    EXPECT_EQ(
        secondRecord.descriptor.key.role,
        dvbd::AvbdScalarRowRole::FrictionTangent);

    const std::optional<double> frictionCoefficient
        = frictionCoefficientForKey(firstRecord.descriptor.key);
    ASSERT_TRUE(frictionCoefficient.has_value());
    const double forceLimit
        = *frictionCoefficient
          * expectedNormalLambda(firstRecord.descriptor.key);
    EXPECT_NEAR(firstRecord.descriptor.bounds.lower, -forceLimit, 1e-12);
    EXPECT_NEAR(firstRecord.descriptor.bounds.upper, forceLimit, 1e-12);
    EXPECT_NEAR(secondRecord.descriptor.bounds.lower, -forceLimit, 1e-12);
    EXPECT_NEAR(secondRecord.descriptor.bounds.upper, forceLimit, 1e-12);

    const PreviousFrictionRow* previousFirst
        = previousFrictionRowForKey(firstRecord.descriptor.key);
    const PreviousFrictionRow* previousSecond
        = previousFrictionRowForKey(secondRecord.descriptor.key);
    ASSERT_NE(previousFirst, nullptr);
    ASSERT_NE(previousSecond, nullptr);

    const Eigen::Vector3d worldDual
        = previousFirst->lambda * previousFirst->direction
          + previousSecond->lambda * previousSecond->direction;
    EXPECT_NEAR(
        firstRecord.state.lambda, worldDual.dot(firstRecord.direction), 1e-12);
    EXPECT_NEAR(
        secondRecord.state.lambda,
        worldDual.dot(secondRecord.direction),
        1e-12);
    EXPECT_NEAR(
        frictionRows[i].first.state.lambda, firstRecord.state.lambda, 1e-12);
    EXPECT_NEAR(
        frictionRows[i].second.state.lambda, secondRecord.state.lambda, 1e-12);
  }
}

//==============================================================================
// The private AVBD contact path is an internal forward-solve opt-in. Its first
// World slice projects a supported penetrating rigid-body contact into the
// velocity that the existing position stage then applies.
TEST(AvbdContact, PenetratingRigidBodyProjectsVelocity)
{
  auto avbd = buildDropScene(sx::ContactSolverMethod::SequentialImpulse, 0.49);
  avbd->setGravity(Eigen::Vector3d::Zero());
  auto sphere = avbd->getRigidBody("sphere");
  ASSERT_TRUE(sphere.has_value());
  dart::simulation::detail::registryOf(*avbd)
      .emplace_or_replace<sx::comps::RigidAvbdContactConfig>(
          dart::simulation::detail::toRegistryEntity(sphere->getEntity()));
  avbd->enterSimulationMode();

  avbd->step();

  EXPECT_GT(sphere->getTranslation().z(), 0.498);
  EXPECT_LT(sphere->getTranslation().z(), 0.505);
  EXPECT_GT(sphere->getLinearVelocity().z(), 0.0);
}

//==============================================================================
// Fixed-joint rows are still private AVBD detail, but the contact-stage opt-in
// should append them to the same rigid World projection when they link rigid
// body entities.
TEST(AvbdContact, FixedJointRowsParticipateInProjection)
{
  auto avbd = buildDropScene(sx::ContactSolverMethod::SequentialImpulse, 0.49);
  avbd->setGravity(Eigen::Vector3d::Zero());

  auto ground = avbd->getRigidBody("ground");
  auto sphere = avbd->getRigidBody("sphere");
  ASSERT_TRUE(ground.has_value());
  ASSERT_TRUE(sphere.has_value());

  Eigen::Isometry3d spherePose = Eigen::Isometry3d::Identity();
  spherePose.translation() = Eigen::Vector3d(0.25, 0.0, 0.49);
  sphere->setTransform(spherePose);

  auto& registry = dart::simulation::detail::registryOf(*avbd);
  const auto& toReg = dart::simulation::detail::toRegistryEntity;
  registry.emplace_or_replace<sx::comps::RigidAvbdContactConfig>(
      toReg(sphere->getEntity()));

  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::Joint>(jointEntity);
  joint.type = sx::comps::JointType::Fixed;
  joint.parentLink = toReg(ground->getEntity());
  joint.childLink = toReg(sphere->getEntity());

  auto& config
      = registry.emplace<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  config.localAnchorA = Eigen::Vector3d(0.0, 0.0, 1.0);
  config.localAnchorB = Eigen::Vector3d::Zero();
  config.startStiffness = 1e5;
  config.maxStiffness = 1e6;

  avbd->enterSimulationMode();
  avbd->step();

  EXPECT_LT(std::abs(sphere->getTranslation().x()), 0.05);
  EXPECT_LT(sphere->getLinearVelocity().x(), 0.0);
  EXPECT_GT(sphere->getTranslation().z(), 0.498);
}

//==============================================================================
// Private fixed-joint rows should also project rigid bodies when no contact
// rows are present, so the World path does not depend on contact activation.
TEST(AvbdContact, FixedJointRowsProjectWithoutContacts)
{
  sx::WorldOptions options;
  options.timeStep = 0.005;
  options.gravity = Eigen::Vector3d::Zero();
  sx::World world(options);

  sx::RigidBodyOptions baseOptions;
  baseOptions.isStatic = true;
  auto base = world.addRigidBody("base", baseOptions);

  sx::RigidBodyOptions linkOptions;
  linkOptions.position = Eigen::Vector3d::UnitX();
  auto link = world.addRigidBody("link", linkOptions);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::Joint>(jointEntity);
  joint.type = sx::comps::JointType::Fixed;
  joint.parentLink
      = dart::simulation::detail::toRegistryEntity(base.getEntity());
  joint.childLink
      = dart::simulation::detail::toRegistryEntity(link.getEntity());

  auto& config
      = registry.emplace<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  config.startStiffness = 1e5;
  config.maxStiffness = 1e6;

  world.enterSimulationMode();
  world.step();

  EXPECT_LT(std::abs(link.getTranslation().x()), 0.05);
  EXPECT_LT(link.getLinearVelocity().x(), 0.0);
  EXPECT_TRUE(base.getTranslation().isApprox(Eigen::Vector3d::Zero()));
}

//==============================================================================
// Missing private fixed-joint configs should be initialized from the
// design-time pose on opt-in rigid bodies when simulation mode starts, not
// re-baselined from later drift.
TEST(AvbdContact, FixedJointPoseBridgeCapturesSimulationEntryPose)
{
  sx::WorldOptions options;
  options.timeStep = 0.005;
  options.gravity = Eigen::Vector3d::Zero();
  sx::World world(options);

  sx::RigidBodyOptions baseOptions;
  baseOptions.isStatic = true;
  auto base = world.addRigidBody("base", baseOptions);

  sx::RigidBodyOptions linkOptions;
  linkOptions.position = Eigen::Vector3d::UnitX();
  auto link = world.addRigidBody("link", linkOptions);

  auto& registry = dart::simulation::detail::registryOf(world);
  registry.emplace_or_replace<sx::comps::RigidAvbdContactConfig>(
      sx::detail::toRegistryEntity(link.getEntity()));

  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::Joint>(jointEntity);
  joint.type = sx::comps::JointType::Fixed;
  joint.parentLink = sx::detail::toRegistryEntity(base.getEntity());
  joint.childLink = sx::detail::toRegistryEntity(link.getEntity());

  ASSERT_FALSE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  world.enterSimulationMode();

  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_NEAR(
      (config.localAnchorA - Eigen::Vector3d::UnitX()).norm(), 0.0, 1e-12);
  EXPECT_NEAR(config.localAnchorB.norm(), 0.0, 1e-12);

  Eigen::Isometry3d driftedPose = Eigen::Isometry3d::Identity();
  driftedPose.translation() = Eigen::Vector3d(1.25, 0.0, 0.0);
  link.setTransform(driftedPose);

  world.step();

  EXPECT_LT(std::abs(link.getTranslation().x() - 1.0), 0.05);
  EXPECT_LT(link.getLinearVelocity().x(), 0.0);
  EXPECT_TRUE(base.getTranslation().isApprox(Eigen::Vector3d::Zero()));
}

//==============================================================================
// Private rigid-body revolute ECS joints should configure the same point-joint
// primitive as fixed joints, but leave the configured hinge axis free.
TEST(AvbdContact, RevoluteRigidBodyJointPoseBridgeCapturesAxisConfig)
{
  sx::WorldOptions options;
  options.timeStep = 0.005;
  options.gravity = Eigen::Vector3d::Zero();
  sx::World world(options);

  sx::RigidBodyOptions baseOptions;
  baseOptions.isStatic = true;
  auto base = world.addRigidBody("base", baseOptions);

  sx::RigidBodyOptions linkOptions;
  linkOptions.position = Eigen::Vector3d::UnitX();
  auto link = world.addRigidBody("link", linkOptions);

  auto& registry = dart::simulation::detail::registryOf(world);
  auto& avbdConfig
      = registry.emplace_or_replace<sx::comps::RigidAvbdContactConfig>(
          sx::detail::toRegistryEntity(link.getEntity()));
  avbdConfig.startStiffness = 123.0;
  avbdConfig.maxStiffness = 456.0;

  const Eigen::Vector3d hingeAxis = Eigen::Vector3d(1.0, 2.0, 3.0).normalized();
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::Joint>(jointEntity);
  joint.type = sx::comps::JointType::Revolute;
  joint.parentLink = sx::detail::toRegistryEntity(base.getEntity());
  joint.childLink = sx::detail::toRegistryEntity(link.getEntity());
  joint.axis = hingeAxis;

  ASSERT_FALSE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  world.enterSimulationMode();

  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_NEAR(
      (config.localAnchorA - Eigen::Vector3d::UnitX()).norm(), 0.0, 1e-12);
  EXPECT_NEAR(config.localAnchorB.norm(), 0.0, 1e-12);
  EXPECT_EQ(config.linearAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_EQ(config.angularAxisMask, dvbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_NEAR(config.angularAxes.col(0).dot(hingeAxis), 0.0, 1e-12);
  EXPECT_NEAR(config.angularAxes.col(1).dot(hingeAxis), 0.0, 1e-12);
  EXPECT_NEAR(std::abs(config.angularAxes.col(2).dot(hingeAxis)), 1.0, 1e-12);
  EXPECT_EQ(config.startStiffness, 123.0);
  EXPECT_EQ(config.maxStiffness, 456.0);

  const auto inputs = dvbd::extractAvbdRigidWorldPointJointInputs(registry);
  ASSERT_EQ(inputs.size(), 1u);
  EXPECT_EQ(inputs[0].linearAxisMask, config.linearAxisMask);
  EXPECT_EQ(inputs[0].angularAxisMask, config.angularAxisMask);
  EXPECT_NEAR((inputs[0].angularAxes - config.angularAxes).norm(), 0.0, 1e-12);
}

//==============================================================================
// Private rigid-body prismatic ECS joints should leave the configured
// translation axis free while constraining all angular axes.
TEST(AvbdContact, PrismaticRigidBodyJointPoseBridgeCapturesAxisConfig)
{
  sx::WorldOptions options;
  options.timeStep = 0.005;
  options.gravity = Eigen::Vector3d::Zero();
  sx::World world(options);

  sx::RigidBodyOptions baseOptions;
  baseOptions.isStatic = true;
  auto base = world.addRigidBody("base", baseOptions);

  sx::RigidBodyOptions linkOptions;
  linkOptions.position = Eigen::Vector3d::UnitX();
  auto link = world.addRigidBody("link", linkOptions);

  auto& registry = dart::simulation::detail::registryOf(world);
  const Eigen::Vector3d translationAxis
      = Eigen::Vector3d(-0.5, 0.25, 1.0).normalized();
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::Joint>(jointEntity);
  joint.type = sx::comps::JointType::Prismatic;
  joint.parentLink = sx::detail::toRegistryEntity(base.getEntity());
  joint.childLink = sx::detail::toRegistryEntity(link.getEntity());
  joint.axis = translationAxis;

  ASSERT_FALSE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  world.enterSimulationMode();

  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_EQ(config.linearAxisMask, dvbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_EQ(config.angularAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_NEAR(config.linearAxes.col(0).dot(translationAxis), 0.0, 1e-12);
  EXPECT_NEAR(config.linearAxes.col(1).dot(translationAxis), 0.0, 1e-12);
  EXPECT_NEAR(
      std::abs(config.linearAxes.col(2).dot(translationAxis)), 1.0, 1e-12);

  const auto inputs = dvbd::extractAvbdRigidWorldPointJointInputs(registry);
  ASSERT_EQ(inputs.size(), 1u);
  EXPECT_EQ(inputs[0].linearAxisMask, config.linearAxisMask);
  EXPECT_EQ(inputs[0].angularAxisMask, config.angularAxisMask);
  EXPECT_NEAR((inputs[0].linearAxes - config.linearAxes).norm(), 0.0, 1e-12);
}

//==============================================================================
// The public facade should create the fixed-joint row config without exposing
// ECS or AVBD detail components to users.
TEST(AvbdContact, PublicRigidBodyFixedJointProjectsFromCapturedPose)
{
  sx::WorldOptions options;
  options.timeStep = 0.005;
  options.gravity = Eigen::Vector3d::Zero();
  sx::World world(options);

  sx::RigidBodyOptions baseOptions;
  baseOptions.isStatic = true;
  auto base = world.addRigidBody("base", baseOptions);

  sx::RigidBodyOptions linkOptions;
  linkOptions.position = Eigen::Vector3d::UnitX();
  auto link = world.addRigidBody("link", linkOptions);

  auto joint = world.addRigidBodyFixedJoint("base_to_link", base, link);
  EXPECT_EQ(joint.getName(), "base_to_link");
  EXPECT_EQ(joint.getType(), sx::JointType::Fixed);
  EXPECT_EQ(joint.getDOFCount(), 0u);
  EXPECT_EQ(joint.getParentRigidBody().getName(), "base");
  EXPECT_EQ(joint.getChildRigidBody().getName(), "link");
  EXPECT_TRUE(world.hasRigidBodyFixedJoint("base_to_link"));
  EXPECT_FALSE(world.hasRigidBodyFixedJoint("missing"));
  EXPECT_EQ(world.getRigidBodyFixedJointCount(), 1u);
  auto foundJoint = world.getRigidBodyFixedJoint("base_to_link");
  ASSERT_TRUE(foundJoint.has_value());
  EXPECT_EQ(foundJoint->getParentRigidBody().getName(), "base");
  EXPECT_EQ(foundJoint->getChildRigidBody().getName(), "link");
  EXPECT_FALSE(world.getRigidBodyFixedJoint("missing").has_value());
  const auto fixedJoints = world.getRigidBodyFixedJoints();
  ASSERT_EQ(fixedJoints.size(), 1u);
  EXPECT_EQ(fixedJoints.front().getName(), "base_to_link");
  EXPECT_THROW(
      {
        auto parentLink = joint.getParentLink();
        (void)parentLink;
      },
      sx::InvalidArgumentException);
  EXPECT_THROW(
      {
        auto childLink = joint.getChildLink();
        (void)childLink;
      },
      sx::InvalidArgumentException);
  EXPECT_THROW(
      world.addRigidBodyFixedJoint("base_to_link", base, link),
      sx::InvalidArgumentException);

  Eigen::Isometry3d driftedPose = Eigen::Isometry3d::Identity();
  driftedPose.translation() = Eigen::Vector3d(1.25, 0.0, 0.0);
  link.setTransform(driftedPose);

  world.enterSimulationMode();
  world.step();

  EXPECT_LT(std::abs(link.getTranslation().x() - 1.0), 0.05);
  EXPECT_LT(link.getLinearVelocity().x(), 0.0);
  EXPECT_THROW(
      world.addRigidBodyFixedJoint("late_joint", base, link),
      sx::InvalidOperationException);
}

//==============================================================================
// The public one-DOF rigid-body facade should expose a revolute joint that
// projects the captured anchor while leaving the hinge axis free.
TEST(AvbdContact, PublicRigidBodyRevoluteJointProjectsAnchor)
{
  sx::WorldOptions options;
  options.timeStep = 0.005;
  options.gravity = Eigen::Vector3d::Zero();
  sx::World world(options);

  sx::RigidBodyOptions baseOptions;
  baseOptions.isStatic = true;
  auto base = world.addRigidBody("base", baseOptions);

  sx::RigidBodyOptions linkOptions;
  linkOptions.position = Eigen::Vector3d::UnitX();
  auto link = world.addRigidBody("link", linkOptions);

  auto joint = world.addRigidBodyRevoluteJoint(
      "base_to_link_hinge", base, link, Eigen::Vector3d::UnitZ());
  EXPECT_EQ(joint.getType(), sx::JointType::Revolute);
  EXPECT_EQ(joint.getDOFCount(), 1u);
  EXPECT_EQ(joint.getParentRigidBody().getName(), "base");
  EXPECT_EQ(joint.getChildRigidBody().getName(), "link");
  EXPECT_TRUE(world.hasRigidBodyJoint("base_to_link_hinge"));
  EXPECT_FALSE(world.hasRigidBodyFixedJoint("base_to_link_hinge"));
  EXPECT_EQ(world.getRigidBodyJointCount(), 1u);
  EXPECT_EQ(world.getRigidBodyFixedJointCount(), 0u);
  auto foundJoint = world.getRigidBodyJoint("base_to_link_hinge");
  ASSERT_TRUE(foundJoint.has_value());
  EXPECT_EQ(foundJoint->getType(), sx::JointType::Revolute);
  const auto rigidBodyJoints = world.getRigidBodyJoints();
  ASSERT_EQ(rigidBodyJoints.size(), 1u);
  EXPECT_EQ(rigidBodyJoints.front().getName(), "base_to_link_hinge");

  Eigen::Isometry3d driftedPose = Eigen::Isometry3d::Identity();
  driftedPose.translation() = Eigen::Vector3d(1.25, 0.25, 0.0);
  driftedPose.linear()
      = Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  link.setTransform(driftedPose);

  world.enterSimulationMode();
  world.step();

  EXPECT_LT(std::abs(link.getTranslation().x() - 1.0), 0.05);
  EXPECT_LT(std::abs(link.getTranslation().y()), 0.05);
  EXPECT_TRUE(base.getTranslation().isApprox(Eigen::Vector3d::Zero()));
}

//==============================================================================
// The public prismatic facade should project drift orthogonal to the slide axis
// while leaving translation along that axis unconstrained.
TEST(AvbdContact, PublicRigidBodyPrismaticJointProjectsOrthogonalDrift)
{
  sx::WorldOptions options;
  options.timeStep = 0.005;
  options.gravity = Eigen::Vector3d::Zero();
  sx::World world(options);

  sx::RigidBodyOptions baseOptions;
  baseOptions.isStatic = true;
  auto base = world.addRigidBody("base", baseOptions);

  sx::RigidBodyOptions linkOptions;
  linkOptions.position = Eigen::Vector3d::UnitZ();
  auto link = world.addRigidBody("link", linkOptions);

  auto joint = world.addRigidBodyPrismaticJoint(
      "base_to_link_slider", base, link, Eigen::Vector3d::UnitZ());
  EXPECT_EQ(joint.getType(), sx::JointType::Prismatic);
  EXPECT_EQ(joint.getDOFCount(), 1u);
  EXPECT_EQ(world.getRigidBodyJointCount(), 1u);
  EXPECT_FALSE(world.hasRigidBodyFixedJoint("base_to_link_slider"));

  Eigen::Isometry3d driftedPose = Eigen::Isometry3d::Identity();
  driftedPose.translation() = Eigen::Vector3d(0.25, 0.0, 1.5);
  link.setTransform(driftedPose);

  world.enterSimulationMode();
  world.step();

  EXPECT_LT(std::abs(link.getTranslation().x()), 0.05);
  EXPECT_NEAR(link.getTranslation().z(), 1.5, 0.05);
  EXPECT_TRUE(base.getTranslation().isApprox(Eigen::Vector3d::Zero()));
}

//==============================================================================
// Saving a public fixed joint before simulation should preserve the AVBD row
// config and the joint-relative anchors captured at creation time.
TEST(AvbdContact, PublicRigidBodyFixedJointSurvivesSaveLoad)
{
  sx::WorldOptions options;
  options.timeStep = 0.005;
  options.gravity = Eigen::Vector3d::Zero();
  sx::World world(options);

  sx::RigidBodyOptions baseOptions;
  baseOptions.isStatic = true;
  auto base = world.addRigidBody("base", baseOptions);

  sx::RigidBodyOptions linkOptions;
  linkOptions.position = Eigen::Vector3d::UnitX();
  auto link = world.addRigidBody("link", linkOptions);
  (void)world.addRigidBodyFixedJoint("base_to_link", base, link);

  Eigen::Isometry3d designPose = Eigen::Isometry3d::Identity();
  designPose.translation() = Eigen::Vector3d(1.25, 0.0, 0.0);
  link.setTransform(designPose);

  std::stringstream data;
  world.saveBinary(data);

  sx::World restored;
  restored.loadBinary(data);

  auto restoredBase = restored.getRigidBody("base");
  auto restoredLink = restored.getRigidBody("link");
  ASSERT_TRUE(restoredBase.has_value());
  ASSERT_TRUE(restoredLink.has_value());

  auto& registry = dart::simulation::detail::registryOf(restored);
  EXPECT_EQ(restored.getRigidBodyFixedJointCount(), 1u);
  EXPECT_TRUE(restored.hasRigidBodyFixedJoint("base_to_link"));
  auto restoredJoint = restored.getRigidBodyFixedJoint("base_to_link");
  ASSERT_TRUE(restoredJoint.has_value());
  const entt::entity jointEntity
      = sx::detail::toRegistryEntity(restoredJoint->getEntity());
  EXPECT_EQ(restoredJoint->getParentRigidBody().getName(), "base");
  EXPECT_EQ(restoredJoint->getChildRigidBody().getName(), "link");
  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  const auto& loadedConfig
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_NEAR(
      (loadedConfig.localAnchorA - Eigen::Vector3d::UnitX()).norm(),
      0.0,
      1e-12);
  EXPECT_NEAR(loadedConfig.localAnchorB.norm(), 0.0, 1e-12);

  restored.enterSimulationMode();

  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_NEAR(
      (config.localAnchorA - Eigen::Vector3d::UnitX()).norm(), 0.0, 1e-12);
  EXPECT_NEAR(config.localAnchorB.norm(), 0.0, 1e-12);

  restored.step();

  EXPECT_LT(std::abs(restoredLink->getTranslation().x() - 1.0), 0.05);
  EXPECT_LT(restoredLink->getLinearVelocity().x(), 0.0);
  EXPECT_TRUE(restoredBase->getTranslation().isApprox(Eigen::Vector3d::Zero()));
}

//==============================================================================
TEST(AvbdContact, PublicRigidBodyJointBreakStateSurvivesSaveLoad)
{
  sx::World world;

  sx::RigidBodyOptions baseOptions;
  baseOptions.isStatic = true;
  auto base = world.addRigidBody("base", baseOptions);

  sx::RigidBodyOptions linkOptions;
  linkOptions.position = Eigen::Vector3d::UnitX();
  auto link = world.addRigidBody("link", linkOptions);

  auto joint = world.addRigidBodyFixedJoint("base_to_link", base, link);
  joint.setBreakForce(7.5);

  auto& registry = dart::simulation::detail::registryOf(world);
  registry
      .get<sx::comps::Joint>(sx::detail::toRegistryEntity(joint.getEntity()))
      .broken = true;

  std::stringstream data;
  world.saveBinary(data);

  sx::World restored;
  restored.loadBinary(data);

  auto restoredJoint = restored.getRigidBodyFixedJoint("base_to_link");
  ASSERT_TRUE(restoredJoint.has_value());
  EXPECT_DOUBLE_EQ(restoredJoint->getBreakForce(), 7.5);
  EXPECT_TRUE(restoredJoint->isBroken());

  restoredJoint->resetBreakage();
  EXPECT_FALSE(restoredJoint->isBroken());
}

//==============================================================================
// Saving after simulation mode should also preserve the public fixed joint. A
// loaded simulation-mode world cannot re-enter simulation mode, so loadBinary()
// must rebuild the private AVBD row config directly.
TEST(AvbdContact, PublicRigidBodyFixedJointSurvivesSimulationModeSaveLoad)
{
  sx::WorldOptions options;
  options.timeStep = 0.005;
  options.gravity = Eigen::Vector3d::Zero();
  sx::World world(options);

  sx::RigidBodyOptions baseOptions;
  baseOptions.isStatic = true;
  auto base = world.addRigidBody("base", baseOptions);

  sx::RigidBodyOptions linkOptions;
  linkOptions.position = Eigen::Vector3d::UnitX();
  auto link = world.addRigidBody("link", linkOptions);
  (void)world.addRigidBodyFixedJoint("base_to_link", base, link);

  world.enterSimulationMode();

  std::stringstream data;
  world.saveBinary(data);

  sx::World restored;
  restored.loadBinary(data);
  ASSERT_TRUE(restored.isSimulationMode());

  auto restoredBase = restored.getRigidBody("base");
  auto restoredLink = restored.getRigidBody("link");
  ASSERT_TRUE(restoredBase.has_value());
  ASSERT_TRUE(restoredLink.has_value());

  auto& registry = dart::simulation::detail::registryOf(restored);
  EXPECT_EQ(restored.getRigidBodyFixedJointCount(), 1u);
  EXPECT_TRUE(restored.hasRigidBodyFixedJoint("base_to_link"));
  auto restoredJoint = restored.getRigidBodyFixedJoint("base_to_link");
  ASSERT_TRUE(restoredJoint.has_value());
  const entt::entity jointEntity
      = sx::detail::toRegistryEntity(restoredJoint->getEntity());
  EXPECT_EQ(restoredJoint->getParentRigidBody().getName(), "base");
  EXPECT_EQ(restoredJoint->getChildRigidBody().getName(), "link");
  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_NEAR(
      (config.localAnchorA - Eigen::Vector3d::UnitX()).norm(), 0.0, 1e-12);
  EXPECT_NEAR(config.localAnchorB.norm(), 0.0, 1e-12);

  Eigen::Isometry3d driftedPose = Eigen::Isometry3d::Identity();
  driftedPose.translation() = Eigen::Vector3d(1.25, 0.0, 0.0);
  restoredLink->setTransform(driftedPose);

  restored.step();

  EXPECT_LT(std::abs(restoredLink->getTranslation().x() - 1.0), 0.05);
  EXPECT_LT(restoredLink->getLinearVelocity().x(), 0.0);
  EXPECT_TRUE(restoredBase->getTranslation().isApprox(Eigen::Vector3d::Zero()));
}

//==============================================================================
// Public fixed joints should remain active when ordinary, non-AVBD-opted
// contacts involve the fixed body. The contact still falls through to the
// selected default contact solver while the fixed-joint rows project
// independently.
TEST(AvbdContact, PublicFixedJointProjectsWithDefaultContactOnFixedBody)
{
  sx::WorldOptions options;
  options.timeStep = 0.005;
  options.gravity = Eigen::Vector3d::Zero();
  sx::World world(options);

  sx::RigidBodyOptions baseOptions;
  baseOptions.isStatic = true;
  auto base = world.addRigidBody("base", baseOptions);

  sx::RigidBodyOptions linkOptions;
  linkOptions.position = Eigen::Vector3d(1.0, 0.0, 0.49);
  auto link = world.addRigidBody("link", linkOptions);
  link.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.5, 0.5, 0.5)));

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(1.0, 0.0, -0.5);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(2.0, 2.0, 0.5)));

  (void)world.addRigidBodyFixedJoint("base_to_link", base, link);
  world.enterSimulationMode();

  auto& registry = dart::simulation::detail::registryOf(world);
  EXPECT_FALSE(registry.all_of<sx::comps::RigidAvbdContactConfig>(
      sx::detail::toRegistryEntity(link.getEntity())));
  EXPECT_FALSE(registry.all_of<sx::comps::RigidAvbdContactConfig>(
      sx::detail::toRegistryEntity(ground.getEntity())));
  ASSERT_FALSE(world.collide().empty());

  Eigen::Isometry3d driftedPose = link.getTransform();
  driftedPose.translation().x() = 1.25;
  link.setTransform(driftedPose);
  const double driftBeforeStep = std::abs(link.getTranslation().x() - 1.0);
  ASSERT_FALSE(world.collide().empty());

  world.step();

  EXPECT_LT(std::abs(link.getTranslation().x() - 1.0), driftBeforeStep);
  EXPECT_LT(link.getLinearVelocity().x(), 0.0);
  EXPECT_TRUE(base.getTranslation().isApprox(Eigen::Vector3d::Zero()));
}

//==============================================================================
// The no-contact fixed-joint path should also route angular rows through the
// private AVBD projection instead of only correcting point-anchor drift.
TEST(AvbdContact, FixedJointAngularRowsProjectWithoutContacts)
{
  sx::WorldOptions options;
  options.timeStep = 0.005;
  options.gravity = Eigen::Vector3d::Zero();
  sx::World world(options);

  sx::RigidBodyOptions baseOptions;
  baseOptions.isStatic = true;
  auto base = world.addRigidBody("base", baseOptions);

  sx::RigidBodyOptions linkOptions;
  linkOptions.orientation = Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ());
  auto link = world.addRigidBody("link", linkOptions);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::Joint>(jointEntity);
  joint.type = sx::comps::JointType::Fixed;
  joint.parentLink
      = dart::simulation::detail::toRegistryEntity(base.getEntity());
  joint.childLink
      = dart::simulation::detail::toRegistryEntity(link.getEntity());

  auto& config
      = registry.emplace<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  config.startStiffness = 1e5;
  config.maxStiffness = 1e6;

  world.enterSimulationMode();
  world.step();

  const Eigen::AngleAxisd residual(link.getTransform().linear());
  EXPECT_LT(std::abs(residual.angle()), 0.05);
  EXPECT_LT(link.getAngularVelocity().z(), 0.0);
  EXPECT_TRUE(
      base.getTransform().linear().isApprox(Eigen::Matrix3d::Identity()));
}

//==============================================================================
// Private fixed-joint rows should continue to project when an unrelated contact
// falls back to the ordinary rigid contact solver.
TEST(AvbdContact, FixedJointRowsProjectWithFallbackContacts)
{
  auto world = buildDropScene(sx::ContactSolverMethod::SequentialImpulse, 0.49);
  world->setGravity(Eigen::Vector3d::Zero());

  auto sphere = world->getRigidBody("sphere");
  ASSERT_TRUE(sphere.has_value());

  sx::RigidBodyOptions baseOptions;
  baseOptions.isStatic = true;
  baseOptions.position = Eigen::Vector3d(10.0, 0.0, 0.0);
  auto base = world->addRigidBody("joint_base", baseOptions);

  sx::RigidBodyOptions linkOptions;
  linkOptions.position = Eigen::Vector3d(11.0, 0.0, 0.0);
  auto link = world->addRigidBody("joint_link", linkOptions);

  auto& registry = dart::simulation::detail::registryOf(*world);
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::Joint>(jointEntity);
  joint.type = sx::comps::JointType::Fixed;
  joint.parentLink
      = dart::simulation::detail::toRegistryEntity(base.getEntity());
  joint.childLink
      = dart::simulation::detail::toRegistryEntity(link.getEntity());

  auto& config
      = registry.emplace<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  config.startStiffness = 1e5;
  config.maxStiffness = 1e6;

  world->enterSimulationMode();
  world->step();

  EXPECT_LT(std::abs(link.getTranslation().x() - 10.0), 0.05);
  EXPECT_LT(link.getLinearVelocity().x(), 0.0);
  EXPECT_GT(sphere->getTranslation().z(), 0.49);
}

//==============================================================================
// A body dropped onto a static ground rests in the same place under both
// solver paths (non-penetration, normal velocity -> 0, same resting height).
TEST(BoxedLcpContact, RestingHeightMatchesSequentialImpulse)
{
  auto reference = buildDropScene(sx::ContactSolverMethod::SequentialImpulse);
  auto lcp = buildDropScene(sx::ContactSolverMethod::BoxedLcp);

  reference->enterSimulationMode();
  lcp->enterSimulationMode();
  reference->step(1000);
  lcp->step(1000);

  auto referenceSphere = reference->getRigidBody("sphere");
  auto lcpSphere = lcp->getRigidBody("sphere");
  ASSERT_TRUE(referenceSphere.has_value());
  ASSERT_TRUE(lcpSphere.has_value());

  const double referenceZ = referenceSphere->getTranslation().z();
  const double lcpZ = lcpSphere->getTranslation().z();

  // Both rest near z = 0.5 (sphere radius on a ground top at z = 0).
  EXPECT_NEAR(lcpZ, 0.5, 2e-2);
  // The LCP path matches the sequential-impulse resting height within a tight
  // tolerance.
  EXPECT_NEAR(lcpZ, referenceZ, 1e-6);

  // Non-penetration at rest: the sphere bottom stays at or above the ground.
  EXPECT_GT(lcpZ, 0.5 - 1e-3);

  // Normal velocity has essentially stopped.
  EXPECT_LT(std::abs(lcpSphere->getLinearVelocity().z()), 0.1);

  // The static ground did not move under the LCP path.
  EXPECT_TRUE(lcp->getRigidBody("ground")->getTranslation().isApprox(
      Eigen::Vector3d(0.0, 0.0, -0.5)));
}

//==============================================================================
// A single resting drop step: with the body already touching the ground at
// near-zero velocity, the LCP normal impulse cancels the incoming gravity
// velocity exactly as the sequential path does.
TEST(BoxedLcpContact, SingleStepNormalVelocityMatches)
{
  auto reference = buildDropScene(sx::ContactSolverMethod::SequentialImpulse);
  auto lcp = buildDropScene(sx::ContactSolverMethod::BoxedLcp);

  // Place both spheres just touching the ground (center at z = 0.5) so the
  // first step is a normal-only contact resolution.
  for (auto* world : {reference.get(), lcp.get()}) {
    auto sphere = world->getRigidBody("sphere");
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(0.0, 0.0, 0.5);
    sphere->setTransform(pose);
    world->enterSimulationMode();
  }

  reference->step();
  lcp->step();

  const double referenceVz
      = reference->getRigidBody("sphere")->getLinearVelocity().z();
  const double lcpVz = lcp->getRigidBody("sphere")->getLinearVelocity().z();

  // The contact removes the downward approach; residual normal velocity is
  // near zero and matches the sequential path closely.
  EXPECT_NEAR(lcpVz, referenceVz, 1e-9);
  EXPECT_LT(std::abs(lcpVz), 1e-6);
}

//==============================================================================
// Momentum/known-physics check: two equal-mass spheres in a head-on, fully
// inelastic (zero restitution), frictionless collision both come to rest, so
// total linear momentum (zero) is conserved -- identical under both paths.
TEST(BoxedLcpContact, EqualMassHeadOnConservesMomentum)
{
  const auto run = [](sx::ContactSolverMethod method) {
    sx::WorldOptions options;
    options.timeStep = 0.001;
    options.gravity = Eigen::Vector3d::Zero();
    options.contactSolverMethod = method;
    sx::World world(options);

    sx::RigidBodyOptions optionsA;
    optionsA.position = Eigen::Vector3d(-0.45, 0.0, 0.0);
    optionsA.linearVelocity = Eigen::Vector3d(1.0, 0.0, 0.0);
    auto bodyA = world.addRigidBody("a", optionsA);
    bodyA.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
    bodyA.setFriction(0.0);

    sx::RigidBodyOptions optionsB;
    optionsB.position = Eigen::Vector3d(0.45, 0.0, 0.0);
    optionsB.linearVelocity = Eigen::Vector3d(-1.0, 0.0, 0.0);
    auto bodyB = world.addRigidBody("b", optionsB);
    bodyB.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
    bodyB.setFriction(0.0);

    world.step();
    return std::make_pair(
        bodyA.getLinearVelocity().x(), bodyB.getLinearVelocity().x());
  };

  const auto reference = run(sx::ContactSolverMethod::SequentialImpulse);
  const auto lcp = run(sx::ContactSolverMethod::BoxedLcp);

  // Closed form: equal masses, head-on, fully inelastic -> both at rest.
  EXPECT_NEAR(lcp.first, 0.0, 1e-9);
  EXPECT_NEAR(lcp.second, 0.0, 1e-9);

  // Momentum conserved (zero before and after).
  EXPECT_NEAR(lcp.first + lcp.second, 0.0, 1e-12);

  // Parity with the sequential-impulse path.
  EXPECT_NEAR(lcp.first, reference.first, 1e-9);
  EXPECT_NEAR(lcp.second, reference.second, 1e-9);
}

namespace {

//==============================================================================
// Build a box-on-static-box-ground scene with the requested contact solver and
// Coulomb friction coefficient. Ground top face is at z = 0; the box (half
// extents 0.5) starts resting on it (center at z = 0.5). The box is given an
// initial horizontal velocity so the tangential rows are exercised.
std::unique_ptr<sx::World> buildFrictionScene(
    sx::ContactSolverMethod method,
    double friction,
    const Eigen::Vector3d& initialVelocity)
{
  sx::WorldOptions options;
  options.timeStep = 0.005;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  options.contactSolverMethod = method;
  auto world = std::make_unique<sx::World>(options);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world->addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(20.0, 20.0, 0.5)));
  ground.setFriction(friction);

  sx::RigidBodyOptions boxOptions;
  boxOptions.position = Eigen::Vector3d(0.0, 0.0, 0.5);
  boxOptions.linearVelocity = initialVelocity;
  auto box = world->addRigidBody("box", boxOptions);
  box.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.5, 0.5, 0.5)));
  box.setFriction(friction);

  return world;
}

} // namespace

//==============================================================================
// Kinetic-friction parity: a box sliding horizontally on a frictional ground
// decelerates, and the BoxedLcp path's slowing behavior tracks the
// SequentialImpulse path within a documented tolerance. Friction mu = 0.5.
TEST(BoxedLcpContact, SlidingBoxDeceleratesLikeSequentialImpulse)
{
  constexpr double kFriction = 0.5;
  const Eigen::Vector3d push(1.0, 0.0, 0.0);
  auto reference = buildFrictionScene(
      sx::ContactSolverMethod::SequentialImpulse, kFriction, push);
  auto lcp
      = buildFrictionScene(sx::ContactSolverMethod::BoxedLcp, kFriction, push);

  reference->enterSimulationMode();
  lcp->enterSimulationMode();
  reference->step(200);
  lcp->step(200);

  const double referenceVx
      = reference->getRigidBody("box")->getLinearVelocity().x();
  const double lcpVx = lcp->getRigidBody("box")->getLinearVelocity().x();

  // Friction removed forward speed in both paths (the box slowed markedly).
  EXPECT_LT(lcpVx, 0.9);
  EXPECT_GE(lcpVx, -1e-6); // friction does not reverse the slide

  // The BoxedLcp slowing tracks the SequentialImpulse path. The two solvers use
  // different inner iteration schemes (pivoting boxed LCP vs Gauss-Seidel box
  // friction), so the documented parity tolerance is loose (absolute 0.15 m/s
  // on the residual forward speed).
  EXPECT_NEAR(lcpVx, referenceVx, 0.15);

  // The box stayed on the ground (no spurious vertical drift) under the LCP
  // path.
  EXPECT_NEAR(lcp->getRigidBody("box")->getTranslation().z(), 0.5, 1e-2);
}

//==============================================================================
// Static-friction hold: a small tangential push well inside the friction cone
// is fully resisted, so the box stays essentially in place under the BoxedLcp
// path. Friction mu = 0.8 with a tiny initial velocity.
TEST(BoxedLcpContact, StaticFrictionHoldsSmallPush)
{
  constexpr double kFriction = 0.8;
  // A small tangential velocity that one friction impulse can fully cancel.
  const Eigen::Vector3d smallPush(0.02, 0.0, 0.0);
  auto lcp = buildFrictionScene(
      sx::ContactSolverMethod::BoxedLcp, kFriction, smallPush);

  lcp->enterSimulationMode();

  const double startX = lcp->getRigidBody("box")->getTranslation().x();
  lcp->step(200);
  const double endX = lcp->getRigidBody("box")->getTranslation().x();
  const double endVx = lcp->getRigidBody("box")->getLinearVelocity().x();

  // Static friction held the box: tangential velocity is driven to ~zero and
  // the box barely moved over the whole run.
  EXPECT_LT(std::abs(endVx), 1e-3);
  EXPECT_LT(std::abs(endX - startX), 5e-3);
}
