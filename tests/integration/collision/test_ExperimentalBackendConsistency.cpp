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

#include <dart/config.hpp>

#include <dart/collision/CollisionResult.hpp>
#include <dart/collision/DistanceResult.hpp>
#include <dart/collision/experimental/narrow_phase/box_box.hpp>
#include <dart/collision/experimental/narrow_phase/capsule_box.hpp>
#include <dart/collision/experimental/narrow_phase/capsule_capsule.hpp>
#include <dart/collision/experimental/narrow_phase/capsule_sphere.hpp>
#include <dart/collision/experimental/narrow_phase/distance.hpp>
#include <dart/collision/experimental/narrow_phase/sphere_box.hpp>
#include <dart/collision/experimental/narrow_phase/sphere_sphere.hpp>
#include <dart/collision/experimental/shapes/shape.hpp>
#include <dart/collision/experimental/types.hpp>
#include <dart/collision/fcl/FCLCollisionDetector.hpp>

#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/CapsuleShape.hpp>
#include <dart/dynamics/SphereShape.hpp>

#if DART_HAVE_BULLET
  #include <dart/collision/bullet/BulletCollisionDetector.hpp>
#endif

#if DART_HAVE_ODE
  #include <dart/collision/ode/OdeCollisionDetector.hpp>
#endif

#include "tests/benchmark/collision/fixtures/edge_cases.hpp"
#include "tests/benchmark/collision/fixtures/shape_factories.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include <cmath>

namespace {

using dart::benchmark::collision::EdgeCase;
using dart::benchmark::collision::PairKind;

struct CaseSpec
{
  PairKind pair;
  EdgeCase edge;
  double scale;
};

struct Outcome
{
  bool colliding = false;
  bool hasDistance = false;
  double distance = std::numeric_limits<double>::infinity();
  double depth = 0.0;
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
  bool hasNormal = false;
};

struct BackendCapabilities
{
  bool supportsDistance = true;
  bool supportsCapsule = true;
};

struct PairContext
{
  dart::dynamics::SkeletonPtr skel1;
  dart::dynamics::SkeletonPtr skel2;
  dart::collision::CollisionGroupPtr group;
};

PairContext MakePairContext(
    const std::shared_ptr<dart::collision::CollisionDetector>& detector,
    const std::shared_ptr<dart::dynamics::Shape>& shape1,
    const Eigen::Isometry3d& tf1,
    const std::shared_ptr<dart::dynamics::Shape>& shape2,
    const Eigen::Isometry3d& tf2)
{
  PairContext ctx;
  ctx.skel1
      = dart::benchmark::collision::CreateSingleShapeSkeleton(shape1, tf1);
  ctx.skel2
      = dart::benchmark::collision::CreateSingleShapeSkeleton(shape2, tf2);
  ctx.group = detector->createCollisionGroup();
  dart::benchmark::collision::AddSkeletonToGroup(ctx.group.get(), ctx.skel1);
  dart::benchmark::collision::AddSkeletonToGroup(ctx.group.get(), ctx.skel2);
  return ctx;
}

double ContactEpsilon(double scale)
{
  return std::max(1e-5, scale * 1e-2);
}

double DistanceTolerance(double scale)
{
  return std::max(1e-5, scale * 1e-2);
}

double DepthTolerance(double scale)
{
  return std::max(1e-5, scale * 2e-2);
}

double NormalAlignment(
    const Eigen::Vector3d& normal1, const Eigen::Vector3d& normal2)
{
  const double n1 = normal1.norm();
  const double n2 = normal2.norm();
  if (n1 < 1e-9 || n2 < 1e-9) {
    return 0.0;
  }
  return std::abs(normal1.dot(normal2) / (n1 * n2));
}

BackendCapabilities CapabilitiesFor(std::string_view type)
{
  BackendCapabilities caps;
  if (type == "ode") {
    caps.supportsDistance = false;
    caps.supportsCapsule = false;
  }
  return caps;
}

bool PairUsesCapsule(PairKind pair)
{
  return pair == PairKind::kCapsuleCapsule || pair == PairKind::kCapsuleSphere
         || pair == PairKind::kCapsuleBox;
}

bool ShouldCompareDepth(const CaseSpec& spec)
{
  if (spec.edge != EdgeCase::kDeepPenetration) {
    return false;
  }

  if (spec.pair == PairKind::kSphereBox || spec.pair == PairKind::kCapsuleBox) {
    return false;
  }

  return true;
}

std::vector<CaseSpec> BuildCases()
{
  std::vector<CaseSpec> cases;
  const std::vector<EdgeCase> baseEdges
      = {EdgeCase::kTouching, EdgeCase::kDeepPenetration, EdgeCase::kGrazing};
  const std::vector<EdgeCase> boxEdges
      = {EdgeCase::kTouching,
         EdgeCase::kDeepPenetration,
         EdgeCase::kGrazing,
         EdgeCase::kThinFeature};

  for (double scale : dart::benchmark::collision::kScaleSweep) {
    for (EdgeCase edge : baseEdges) {
      cases.push_back({PairKind::kSphereSphere, edge, scale});
      cases.push_back({PairKind::kCapsuleCapsule, edge, scale});
      cases.push_back({PairKind::kCapsuleSphere, edge, scale});
    }
    for (EdgeCase edge : boxEdges) {
      cases.push_back({PairKind::kBoxBox, edge, scale});
      cases.push_back({PairKind::kSphereBox, edge, scale});
      cases.push_back({PairKind::kCapsuleBox, edge, scale});
    }
  }

  return cases;
}

Outcome EvaluateExperimental(const CaseSpec& spec)
{
  using namespace dart::collision::experimental;
  using dart::benchmark::collision::MakeBoxBoxTransforms;
  using dart::benchmark::collision::MakeBoxSpec;
  using dart::benchmark::collision::MakeCapsuleBoxTransforms;
  using dart::benchmark::collision::MakeCapsuleCapsuleTransforms;
  using dart::benchmark::collision::MakeCapsuleSpec;
  using dart::benchmark::collision::MakeCapsuleSphereTransforms;
  using dart::benchmark::collision::MakeSphereBoxTransforms;
  using dart::benchmark::collision::MakeSphereSpec;
  using dart::benchmark::collision::MakeSphereSphereTransforms;

  const auto sphereSpec = MakeSphereSpec(spec.scale);
  const auto capsuleSpec = MakeCapsuleSpec(spec.scale);
  const auto boxSpec
      = MakeBoxSpec(spec.scale, spec.edge == EdgeCase::kThinFeature);

  DistanceResult distResult;
  DistanceOption distOption = DistanceOption::unlimited();
  distOption.enableNearestPoints = true;

  CollisionResult collResult;
  CollisionOption collOption = CollisionOption::fullContacts();

  Outcome outcome;
  outcome.hasDistance = true;
  bool hit = false;

  switch (spec.pair) {
    case PairKind::kSphereSphere: {
      SphereShape s1(sphereSpec.radius);
      SphereShape s2(sphereSpec.radius);
      const auto tfs = MakeSphereSphereTransforms(
          sphereSpec.radius, sphereSpec.radius, spec.edge);
      distResult.clear();
      outcome.distance = distanceSphereSphere(
          s1, tfs.tf1, s2, tfs.tf2, distResult, distOption);
      collResult.clear();
      hit = collideSpheres(s1, tfs.tf1, s2, tfs.tf2, collResult, collOption);
      break;
    }
    case PairKind::kBoxBox: {
      BoxShape b1(boxSpec.halfExtents);
      BoxShape b2(boxSpec.halfExtents);
      const auto tfs = MakeBoxBoxTransforms(
          boxSpec.halfExtents, boxSpec.halfExtents, spec.edge);
      distResult.clear();
      outcome.distance
          = distanceBoxBox(b1, tfs.tf1, b2, tfs.tf2, distResult, distOption);
      collResult.clear();
      hit = collideBoxes(b1, tfs.tf1, b2, tfs.tf2, collResult, collOption);
      break;
    }
    case PairKind::kCapsuleCapsule: {
      CapsuleShape c1(capsuleSpec.radius, capsuleSpec.height);
      CapsuleShape c2(capsuleSpec.radius, capsuleSpec.height);
      const auto tfs = MakeCapsuleCapsuleTransforms(
          capsuleSpec.radius, capsuleSpec.radius, spec.edge);
      distResult.clear();
      outcome.distance = distanceCapsuleCapsule(
          c1, tfs.tf1, c2, tfs.tf2, distResult, distOption);
      collResult.clear();
      hit = collideCapsules(c1, tfs.tf1, c2, tfs.tf2, collResult, collOption);
      break;
    }
    case PairKind::kSphereBox: {
      SphereShape sphere(sphereSpec.radius);
      BoxShape box(boxSpec.halfExtents);
      const auto tfs = MakeSphereBoxTransforms(
          sphereSpec.radius, boxSpec.halfExtents, spec.edge);
      distResult.clear();
      outcome.distance = distanceSphereBox(
          sphere, tfs.tf1, box, tfs.tf2, distResult, distOption);
      collResult.clear();
      hit = collideSphereBox(
          sphere, tfs.tf1, box, tfs.tf2, collResult, collOption);
      break;
    }
    case PairKind::kCapsuleSphere: {
      CapsuleShape capsule(capsuleSpec.radius, capsuleSpec.height);
      SphereShape sphere(sphereSpec.radius);
      const auto tfs = MakeCapsuleSphereTransforms(
          capsuleSpec.radius, sphereSpec.radius, spec.edge);
      distResult.clear();
      outcome.distance = distanceCapsuleSphere(
          capsule, tfs.tf1, sphere, tfs.tf2, distResult, distOption);
      collResult.clear();
      hit = collideCapsuleSphere(
          capsule, tfs.tf1, sphere, tfs.tf2, collResult, collOption);
      break;
    }
    case PairKind::kCapsuleBox: {
      CapsuleShape capsule(capsuleSpec.radius, capsuleSpec.height);
      BoxShape box(boxSpec.halfExtents);
      const auto tfs = MakeCapsuleBoxTransforms(
          capsuleSpec.radius, boxSpec.halfExtents, spec.edge);
      distResult.clear();
      outcome.distance = distanceCapsuleBox(
          capsule, tfs.tf1, box, tfs.tf2, distResult, distOption);
      collResult.clear();
      hit = collideCapsuleBox(
          capsule, tfs.tf1, box, tfs.tf2, collResult, collOption);
      break;
    }
  }

  outcome.colliding = hit || collResult.isCollision();
  if (outcome.colliding && collResult.numContacts() > 0) {
    const auto& contact = collResult.getContact(0);
    outcome.depth = contact.depth;
    outcome.normal = contact.normal;
    outcome.hasNormal = contact.depth > 0.0;
  }

  return outcome;
}

Outcome EvaluateReference(
    const std::shared_ptr<dart::collision::CollisionDetector>& detector,
    const CaseSpec& spec,
    const BackendCapabilities& caps)
{
  using dart::benchmark::collision::MakeBoxBoxTransforms;
  using dart::benchmark::collision::MakeBoxSpec;
  using dart::benchmark::collision::MakeCapsuleBoxTransforms;
  using dart::benchmark::collision::MakeCapsuleCapsuleTransforms;
  using dart::benchmark::collision::MakeCapsuleSpec;
  using dart::benchmark::collision::MakeCapsuleSphereTransforms;
  using dart::benchmark::collision::MakeSphereBoxTransforms;
  using dart::benchmark::collision::MakeSphereSpec;
  using dart::benchmark::collision::MakeSphereSphereTransforms;

  const auto sphereSpec = MakeSphereSpec(spec.scale);
  const auto capsuleSpec = MakeCapsuleSpec(spec.scale);
  const auto boxSpec
      = MakeBoxSpec(spec.scale, spec.edge == EdgeCase::kThinFeature);

  std::shared_ptr<dart::dynamics::Shape> shape1;
  std::shared_ptr<dart::dynamics::Shape> shape2;
  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();

  switch (spec.pair) {
    case PairKind::kSphereSphere: {
      shape1 = std::make_shared<dart::dynamics::SphereShape>(sphereSpec.radius);
      shape2 = std::make_shared<dart::dynamics::SphereShape>(sphereSpec.radius);
      const auto tfs = MakeSphereSphereTransforms(
          sphereSpec.radius, sphereSpec.radius, spec.edge);
      tf1 = tfs.tf1;
      tf2 = tfs.tf2;
      break;
    }
    case PairKind::kBoxBox: {
      shape1 = std::make_shared<dart::dynamics::BoxShape>(boxSpec.size);
      shape2 = std::make_shared<dart::dynamics::BoxShape>(boxSpec.size);
      const auto tfs = MakeBoxBoxTransforms(
          boxSpec.halfExtents, boxSpec.halfExtents, spec.edge);
      tf1 = tfs.tf1;
      tf2 = tfs.tf2;
      break;
    }
    case PairKind::kCapsuleCapsule: {
      shape1 = std::make_shared<dart::dynamics::CapsuleShape>(
          capsuleSpec.radius, capsuleSpec.height);
      shape2 = std::make_shared<dart::dynamics::CapsuleShape>(
          capsuleSpec.radius, capsuleSpec.height);
      const auto tfs = MakeCapsuleCapsuleTransforms(
          capsuleSpec.radius, capsuleSpec.radius, spec.edge);
      tf1 = tfs.tf1;
      tf2 = tfs.tf2;
      break;
    }
    case PairKind::kSphereBox: {
      shape1 = std::make_shared<dart::dynamics::SphereShape>(sphereSpec.radius);
      shape2 = std::make_shared<dart::dynamics::BoxShape>(boxSpec.size);
      const auto tfs = MakeSphereBoxTransforms(
          sphereSpec.radius, boxSpec.halfExtents, spec.edge);
      tf1 = tfs.tf1;
      tf2 = tfs.tf2;
      break;
    }
    case PairKind::kCapsuleSphere: {
      shape1 = std::make_shared<dart::dynamics::CapsuleShape>(
          capsuleSpec.radius, capsuleSpec.height);
      shape2 = std::make_shared<dart::dynamics::SphereShape>(sphereSpec.radius);
      const auto tfs = MakeCapsuleSphereTransforms(
          capsuleSpec.radius, sphereSpec.radius, spec.edge);
      tf1 = tfs.tf1;
      tf2 = tfs.tf2;
      break;
    }
    case PairKind::kCapsuleBox: {
      shape1 = std::make_shared<dart::dynamics::CapsuleShape>(
          capsuleSpec.radius, capsuleSpec.height);
      shape2 = std::make_shared<dart::dynamics::BoxShape>(boxSpec.size);
      const auto tfs = MakeCapsuleBoxTransforms(
          capsuleSpec.radius, boxSpec.halfExtents, spec.edge);
      tf1 = tfs.tf1;
      tf2 = tfs.tf2;
      break;
    }
  }

  auto ctx = MakePairContext(detector, shape1, tf1, shape2, tf2);

  auto collisionOption = dart::benchmark::collision::MakeCollisionOption(4);
  dart::collision::CollisionResult collResult;
  Outcome outcome;
  outcome.colliding
      = detector->collide(ctx.group.get(), collisionOption, &collResult);
  if (outcome.colliding && collResult.getNumContacts() > 0) {
    const auto& contact = collResult.getContact(0);
    outcome.depth = contact.penetrationDepth;
    outcome.normal = contact.normal;
    outcome.hasNormal = contact.penetrationDepth > 0.0;
  }

  auto distanceOption = dart::benchmark::collision::MakeDistanceOption();
  dart::collision::DistanceResult distResult;
  if (caps.supportsDistance) {
    outcome.distance
        = detector->distance(ctx.group.get(), distanceOption, &distResult);
    outcome.hasDistance = true;
  }

  return outcome;
}

void ExpectConsistent(
    const Outcome& experimental, const Outcome& reference, const CaseSpec& spec)
{
  const double contactEps = ContactEpsilon(spec.scale);
  const bool expContact
      = experimental.colliding
        || (experimental.hasDistance && experimental.distance <= contactEps);
  const bool refContact
      = reference.colliding
        || (reference.hasDistance && reference.distance <= contactEps);

  if (spec.edge == EdgeCase::kDeepPenetration) {
    EXPECT_TRUE(expContact);
    EXPECT_TRUE(refContact);
  } else if (expContact != refContact) {
    if (!expContact && experimental.hasDistance) {
      EXPECT_LE(std::abs(experimental.distance), contactEps);
    }
    if (!refContact && reference.hasDistance) {
      EXPECT_LE(std::abs(reference.distance), contactEps);
    }
  } else {
    EXPECT_EQ(expContact, refContact);
  }

  if (!expContact && !refContact) {
    if (experimental.hasDistance && reference.hasDistance) {
      EXPECT_NEAR(
          reference.distance,
          experimental.distance,
          DistanceTolerance(spec.scale));
    }
    return;
  }

  if (ShouldCompareDepth(spec) && experimental.depth > 0.0
      && reference.depth > 0.0) {
    EXPECT_NEAR(
        reference.depth, experimental.depth, DepthTolerance(spec.scale));
    if (experimental.hasNormal && reference.hasNormal) {
      EXPECT_GE(NormalAlignment(experimental.normal, reference.normal), 0.98);
    }
  }
}

} // namespace

TEST(ExperimentalCollision, CrossBackendConsistency)
{
  struct Backend
  {
    std::string name;
    std::shared_ptr<dart::collision::CollisionDetector> detector;
    BackendCapabilities caps;
  };

  std::vector<Backend> backends;
  {
    auto detector = dart::collision::FCLCollisionDetector::create();
    backends.push_back(
        {"FCL", detector, CapabilitiesFor(detector->getTypeView())});
  }
#if DART_HAVE_BULLET
  {
    auto detector = dart::collision::BulletCollisionDetector::create();
    backends.push_back(
        {"Bullet", detector, CapabilitiesFor(detector->getTypeView())});
  }
#endif
#if DART_HAVE_ODE
  {
    auto detector = dart::collision::OdeCollisionDetector::create();
    backends.push_back(
        {"ODE", detector, CapabilitiesFor(detector->getTypeView())});
  }
#endif

  if (backends.empty()) {
    GTEST_SKIP() << "No reference backends available for cross-check.";
  }

  const auto cases = BuildCases();
  for (const auto& spec : cases) {
    const Outcome experimental = EvaluateExperimental(spec);

    for (const auto& backend : backends) {
      if (!backend.caps.supportsCapsule && PairUsesCapsule(spec.pair)) {
        continue;
      }

      SCOPED_TRACE(
          ::testing::Message()
          << "Backend=" << backend.name
          << " Pair=" << dart::benchmark::collision::PairKindName(spec.pair)
          << " Edge=" << dart::benchmark::collision::EdgeCaseName(spec.edge)
          << " Scale=" << spec.scale);

      const Outcome reference
          = EvaluateReference(backend.detector, spec, backend.caps);
      ExpectConsistent(experimental, reference, spec);
    }
  }
}
