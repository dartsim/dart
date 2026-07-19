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
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "../../examples/demos/scenes/FbfAuthorCardHouseSpec.hpp"
#include "../../examples/demos/scenes/FbfAuthorTurntableSpec.hpp"
#include "dart/collision/dart/DARTCollisionDetector.hpp"
#include "dart/collision/native/NativeCollisionDetector.hpp"
#include "dart/constraint/ExactCoulombFbfConstraintSolver.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/CylinderShape.hpp"
#include "dart/dynamics/FreeJoint.hpp"
#include "dart/dynamics/PlaneShape.hpp"
#include "dart/dynamics/ShapeFrame.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/SphereShape.hpp"
#include "dart/math/detail/MasonryArchGeometry.hpp"
#include "dart/simulation/DeactivationOptions.hpp"
#include "dart/simulation/World.hpp"

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include <cmath>
#include <cstdlib>

using namespace dart;

namespace {

constexpr double kInclineTan = 0.5;
constexpr double kInclineDt = 1.0 / 60.0;
constexpr double kInclineDuration = 2.0;
constexpr double kGravity = 9.81;
constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kBackspinRadius = 0.25;
constexpr double kBackspinLinearVelocity = 4.0;
constexpr double kBackspinAngularVelocity = -200.0;
constexpr double kBackspinFriction = 0.5;
constexpr double kBackspinDuration = 4.0;
constexpr double kTurntableInitialRadius = 1.0;
constexpr double kTurntableDuration = 4.0;
constexpr double kTurntableRampDuration = 1.0;
constexpr double kPainleveDuration = 2.5;
constexpr double kPainleveInitialPitch = 0.08;
constexpr double kPainleveInitialVelocity = 3.6;
constexpr double kCardHouseFriction = 0.8;
constexpr double kCardHouseDuration = 1.0;
constexpr double kCardHouseAngle = 0.23;
constexpr double kCardHouseHeight = 1.0;
constexpr double kCardHouseWidth = 0.45;
constexpr double kCardHouseThickness = 0.03;
// Source-informed reconstruction: Newton 1.3 defaults rigid shapes to
// 1000 kg/m^3. The unavailable paper scene may override this value.
constexpr double kCardHouseDensity = 1000.0;
constexpr double kCardHouseInitialPenetration = 0.003;
// Reconstruction choice: adjacent horizontal one-meter cards meet with the
// same nominal 3 mm overlap used at the other interfaces. The paper does not
// publish this spacing.
constexpr double kCardHouseFrameSpacing
    = kCardHouseHeight - kCardHouseInitialPenetration;
constexpr double kCardHouseStepSizeScale = 10.0;
constexpr double kCardHouseOuterRelaxation = 1.5;
constexpr std::size_t kCardHouseSettleProjectileSmokeContacts = 16u;
constexpr std::size_t kCardHouseProjectileCount = 4u;
// Official paper imagery shows cube projectiles. The 0.11 m edge length is a
// reconstruction that preserves the former sphere diameter; the author size,
// mass, and launch speed are not published.
constexpr double kCardHouseProjectileEdgeLength = 0.11;
constexpr double kCardHouseProjectileMass
    = kCardHouseDensity * kCardHouseProjectileEdgeLength
      * kCardHouseProjectileEdgeLength * kCardHouseProjectileEdgeLength;
constexpr double kCardHouseProjectileSpeed = 4.0;
constexpr double kCardHouseProjectileDropHeight = 4.45;
constexpr double kSmallFixtureStepSizeScale = 2.0;
constexpr std::size_t kCardHouseFourLevelCount = 4u;
constexpr std::size_t kCardHouseTenLevelCount = 10u;
// Full natural manifold for the repaired 26-card reconstruction: 512/4 caps
// observe 96 contacts in the initial configuration. This is a measured DART
// reconstruction count, not the paper timing row's 214-contact contract.
constexpr std::size_t kCardHouseReducedMaxContacts = 512u;
constexpr std::size_t kCardHouseReducedMaxContactsPerPair = 4u;
// The paper's contact-rich arch experiments explicitly use mu=0.8. The
// credited Rigid-IPC geometry scene uses 0.5, but that source value is not the
// paper experiment's contact contract.
constexpr double kArchFriction = 0.8;
// Rigid-IPC's default body density ("plastic", src/io/read_rb_scene.cpp:68);
// neither arch scene specifies a per-body density.
constexpr double kArchDensity = 1000.0;
constexpr std::size_t kArchStoneCount = 25u;
constexpr std::size_t kArch101StoneCount = 101u;
constexpr std::size_t kArchReducedMaxContacts = 48u;
constexpr std::size_t kArchReducedMaxContactsPerPair = 2u;
constexpr std::size_t kArch101ReducedMaxContacts = 38u;
constexpr std::size_t kArch101ReducedMaxContactsPerPair = 2u;
// Full natural manifold for both arches at per_pair 4 on the author-faithful
// Rigid-IPC catenary geometry: the 25-stone arch observes 52 actual contacts
// and the 101-stone arch observes 204 (the earlier 96/512 numbers belonged to
// the deliberately-overlapping approximate scaffold this geometry replaced).
constexpr std::size_t kArchFullManifoldMaxContacts = 512u;
constexpr std::size_t kArchFullManifoldMaxContactsPerPair = 4u;
constexpr int kArchMaxOuterIterations = 120000;
// The wedged quasi-static arches converge much faster with the same
// over-relaxed accepted corrections already used by the reduced card-house
// rung; measured at 96 contacts / 30000 outers the failed residual improves
// from 8.87e-6 at relaxation 1.0 to 3.75e-6 at 1.5.
constexpr double kArchOuterRelaxation = 1.5;
constexpr double kArchStepSizeScale = 10.0;
// Official paper imagery shows a horizontal row of about twelve small cubes
// dropping over the crown. These numerical values remain reconstructed.
constexpr std::size_t kArchProjectileCount = 12u;
constexpr double kArchProjectileEdgeLength = 0.035;
constexpr double kArchProjectileMass = kArchDensity * kArchProjectileEdgeLength
                                       * kArchProjectileEdgeLength
                                       * kArchProjectileEdgeLength;
constexpr double kArchProjectileSpeed = 3.0;
constexpr double kArchProjectileSpacing = 0.045;
constexpr double kArchProjectileDropHeight = 0.95;
constexpr const char* kFbfPaperStressTestEnv
    = "DART_RUN_FBF_PAPER_STRESS_TESTS";

bool isFbfPaperStressTestValueEnabled(const char* value)
{
  return value != nullptr && std::string(value) == "1";
}

bool areFbfPaperStressTestsEnabled()
{
  return isFbfPaperStressTestValueEnabled(std::getenv(kFbfPaperStressTestEnv));
}

struct ExactResidualTrace
{
  double maxResidual = std::numeric_limits<double>::quiet_NaN();
  std::size_t sampledSteps = 0u;
  std::size_t failedSteps = 0u;
  std::size_t nonFiniteResiduals = 0u;
};

constexpr std::size_t computeCardHouseCardCount(std::size_t levelCount)
{
  return levelCount * (3u * levelCount + 1u) / 2u;
}

struct InclineRunResult
{
  double displacement = std::numeric_limits<double>::quiet_NaN();
  double speed = std::numeric_limits<double>::quiet_NaN();
  double maxPenetration = std::numeric_limits<double>::quiet_NaN();
  double residual = std::numeric_limits<double>::quiet_NaN();
  double primalResidual = std::numeric_limits<double>::quiet_NaN();
  double dualResidual = std::numeric_limits<double>::quiet_NaN();
  double complementarityResidual = std::numeric_limits<double>::quiet_NaN();
  ExactResidualTrace residualTrace;
  std::size_t trajectorySteps = 0u;
  std::size_t contactSteps = 0u;
  std::size_t lastContacts = 0u;
  std::size_t exactSolves = 0u;
  std::size_t boxedFallbacks = 0u;
  std::size_t exactFailures = 0u;
  std::size_t projectedGradientRetries = 0u;
  int fbfStatus = -1;
  int fbfIterations = 0;
  int maxFbfIterations = 0;
  std::size_t totalFbfIterations = 0u;
  bool projectedGradientRetryUsed = false;
  bool exactSuccess = false;
};

struct BackspinRunResult
{
  double linearVelocity = std::numeric_limits<double>::quiet_NaN();
  double angularVelocity = std::numeric_limits<double>::quiet_NaN();
  double slipVelocity = std::numeric_limits<double>::quiet_NaN();
  double height = std::numeric_limits<double>::quiet_NaN();
  double residual = std::numeric_limits<double>::quiet_NaN();
  double primalResidual = std::numeric_limits<double>::quiet_NaN();
  double dualResidual = std::numeric_limits<double>::quiet_NaN();
  double complementarityResidual = std::numeric_limits<double>::quiet_NaN();
  ExactResidualTrace residualTrace;
  std::size_t lastContacts = 0u;
  std::size_t exactSolves = 0u;
  std::size_t boxedFallbacks = 0u;
  std::size_t exactFailures = 0u;
  std::size_t projectedGradientRetries = 0u;
  int fbfStatus = -1;
  int fbfIterations = 0;
  int maxFbfIterations = 0;
  std::size_t totalFbfIterations = 0u;
  bool projectedGradientRetryUsed = false;
  bool exactSuccess = false;
};

struct PainleveRunResult
{
  double travel = std::numeric_limits<double>::quiet_NaN();
  double tumbleTravel = std::numeric_limits<double>::quiet_NaN();
  double tumbleTime = std::numeric_limits<double>::quiet_NaN();
  double speed = std::numeric_limits<double>::quiet_NaN();
  double uprightness = std::numeric_limits<double>::quiet_NaN();
  double height = std::numeric_limits<double>::quiet_NaN();
  double residual = std::numeric_limits<double>::quiet_NaN();
  double primalResidual = std::numeric_limits<double>::quiet_NaN();
  double dualResidual = std::numeric_limits<double>::quiet_NaN();
  double complementarityResidual = std::numeric_limits<double>::quiet_NaN();
  ExactResidualTrace residualTrace;
  std::size_t lastContacts = 0u;
  std::size_t exactSolves = 0u;
  std::size_t boxedFallbacks = 0u;
  std::size_t exactFailures = 0u;
  std::size_t projectedGradientRetries = 0u;
  int fbfStatus = -1;
  int fbfIterations = 0;
  int maxFbfIterations = 0;
  std::size_t totalFbfIterations = 0u;
  bool projectedGradientRetryUsed = false;
  bool exactSuccess = false;
};

struct TurntableRunResult
{
  double radius = std::numeric_limits<double>::quiet_NaN();
  double maxRadius = 0.0;
  double radialVelocity = std::numeric_limits<double>::quiet_NaN();
  double tangentialVelocity = std::numeric_limits<double>::quiet_NaN();
  double tangentialCoRotationError = std::numeric_limits<double>::quiet_NaN();
  double height = std::numeric_limits<double>::quiet_NaN();
  double minFullySupportedHeight = std::numeric_limits<double>::infinity();
  double residual = std::numeric_limits<double>::quiet_NaN();
  double primalResidual = std::numeric_limits<double>::quiet_NaN();
  double dualResidual = std::numeric_limits<double>::quiet_NaN();
  double complementarityResidual = std::numeric_limits<double>::quiet_NaN();
  ExactResidualTrace residualTrace;
  std::size_t lastContacts = 0u;
  std::size_t exactSolves = 0u;
  std::size_t boxedFallbacks = 0u;
  std::size_t exactFailures = 0u;
  std::size_t projectedGradientRetries = 0u;
  int fbfStatus = -1;
  int fbfIterations = 0;
  int maxFbfIterations = 0;
  std::size_t totalFbfIterations = 0u;
  bool finiteState = true;
  bool sourceExitObserved = false;
  bool fellThroughSupport = false;
  bool projectedGradientRetryUsed = false;
  bool exactSuccess = false;
};

struct CardHouseRunResult
{
  std::size_t cardCount = 0u;
  std::size_t projectileCount = 0u;
  double minCardAxisUp = std::numeric_limits<double>::quiet_NaN();
  double minCenterHeight = std::numeric_limits<double>::quiet_NaN();
  double maxHorizontalTravel = std::numeric_limits<double>::quiet_NaN();
  double maxProjectileSpeed = 0.0;
  double residual = std::numeric_limits<double>::quiet_NaN();
  double primalResidual = std::numeric_limits<double>::quiet_NaN();
  double dualResidual = std::numeric_limits<double>::quiet_NaN();
  double complementarityResidual = std::numeric_limits<double>::quiet_NaN();
  double failedResidual = std::numeric_limits<double>::quiet_NaN();
  double failedPrimalResidual = std::numeric_limits<double>::quiet_NaN();
  double failedDualResidual = std::numeric_limits<double>::quiet_NaN();
  double failedComplementarityResidual
      = std::numeric_limits<double>::quiet_NaN();
  double elapsedMs = std::numeric_limits<double>::quiet_NaN();
  ExactResidualTrace residualTrace;
  std::size_t lastContacts = 0u;
  std::size_t exactSolves = 0u;
  std::size_t boxedFallbacks = 0u;
  std::size_t exactFailures = 0u;
  std::size_t projectedGradientRetries = 0u;
  int fbfStatus = -1;
  int fbfIterations = 0;
  int maxFbfIterations = 0;
  std::size_t totalFbfIterations = 0u;
  int failedFbfStatus = -1;
  int failedFbfIterations = 0;
  bool projectedGradientRetryUsed = false;
  bool exactSuccess = false;
  bool finiteState = false;
};

bool isCardHouseSkeletonName(const std::string& name);
bool isCardHouseProjectileSkeletonName(const std::string& name);
bool isMasonryArchStoneName(const std::string& name);
bool isMasonryArchProjectileSkeletonName(const std::string& name);

dynamics::SkeletonPtr createInclinePlane(double frictionCoeff)
{
  const double theta = std::atan(kInclineTan);
  const Eigen::Vector3d normal(-std::sin(theta), 0.0, std::cos(theta));

  auto skeleton = dynamics::Skeleton::create("incline_plane");
  auto body
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>().second;
  auto* shapeNode = body->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(
      std::make_shared<dynamics::PlaneShape>(normal, 0.0));
  shapeNode->getDynamicsAspect()->setFrictionCoeff(frictionCoeff);
  skeleton->setMobile(false);
  return skeleton;
}

dynamics::SkeletonPtr createHorizontalPlane(double frictionCoeff)
{
  auto skeleton = dynamics::Skeleton::create("ground_plane");
  auto body
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>().second;
  auto* shapeNode = body->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(
      std::make_shared<dynamics::PlaneShape>(Eigen::Vector3d::UnitZ(), 0.0));
  shapeNode->getDynamicsAspect()->setFrictionCoeff(frictionCoeff);
  skeleton->setMobile(false);
  return skeleton;
}

dynamics::SkeletonPtr createTurntableSupport(double frictionCoeff)
{
  constexpr double kThickness = 0.1;
  auto skeleton = dynamics::Skeleton::create("turntable");
  dynamics::GenericJoint<math::SE3Space>::Properties jointProperties(
      std::string("turntable_joint"));
  dynamics::BodyNode::Properties bodyProperties(
      dynamics::BodyNode::AspectProperties("turntable_body"));
  bodyProperties.mInertia.setMass(1.0);

  auto pair = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>(
      nullptr, jointProperties, bodyProperties);
  auto* joint = pair.first;
  auto* body = pair.second;
  auto* shapeNode = body->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(std::make_shared<dynamics::BoxShape>(
      Eigen::Vector3d(4.0, 4.0, kThickness)));
  shapeNode->getDynamicsAspect()->setFrictionCoeff(frictionCoeff);

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation().z() = -0.5 * kThickness;
  joint->setPositions(dynamics::FreeJoint::convertToPositions(transform));
  skeleton->setMobile(false);
  return skeleton;
}

dynamics::SkeletonPtr createInclineCube(double frictionCoeff)
{
  const double theta = std::atan(kInclineTan);
  const Eigen::Vector3d normal(-std::sin(theta), 0.0, std::cos(theta));
  const Eigen::Vector3d size = Eigen::Vector3d::Ones();
  constexpr double kMass = 1.0;
  constexpr double kInitialPenetration = 0.01;

  auto skeleton = dynamics::Skeleton::create("incline_cube");
  dynamics::GenericJoint<math::SE3Space>::Properties jointProperties(
      std::string("incline_cube_joint"));
  dynamics::BodyNode::Properties bodyProperties(
      dynamics::BodyNode::AspectProperties("incline_cube_body"));
  bodyProperties.mInertia.setMass(kMass);
  bodyProperties.mInertia.setMoment(
      dynamics::BoxShape::computeInertia(size, kMass));

  auto pair = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>(
      nullptr, jointProperties, bodyProperties);
  auto* joint = pair.first;
  auto* body = pair.second;

  auto* shapeNode = body->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(std::make_shared<dynamics::BoxShape>(size));
  shapeNode->getDynamicsAspect()->setFrictionCoeff(frictionCoeff);

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear()
      = Eigen::AngleAxisd(-theta, Eigen::Vector3d::UnitY()).toRotationMatrix();
  transform.translation() = normal * (0.5 * size.z() - kInitialPenetration);
  joint->setPositions(dynamics::FreeJoint::convertToPositions(transform));
  return skeleton;
}

dynamics::SkeletonPtr createPainleveBox(double frictionCoeff)
{
  const Eigen::Vector3d size(0.6, 0.6, 1.0);
  constexpr double kMass = 1.0;
  constexpr double kInitialPenetration = 0.005;

  auto skeleton = dynamics::Skeleton::create("painleve_box");
  dynamics::GenericJoint<math::SE3Space>::Properties jointProperties(
      std::string("painleve_box_joint"));
  dynamics::BodyNode::Properties bodyProperties(
      dynamics::BodyNode::AspectProperties("painleve_box_body"));
  bodyProperties.mInertia.setMass(kMass);
  bodyProperties.mInertia.setMoment(
      dynamics::BoxShape::computeInertia(size, kMass));

  auto pair = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>(
      nullptr, jointProperties, bodyProperties);
  auto* joint = pair.first;
  auto* body = pair.second;
  auto* shapeNode = body->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(std::make_shared<dynamics::BoxShape>(size));
  shapeNode->getDynamicsAspect()->setFrictionCoeff(frictionCoeff);

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear()
      = Eigen::AngleAxisd(kPainleveInitialPitch, Eigen::Vector3d::UnitY())
            .toRotationMatrix();
  const double verticalHalfExtent
      = 0.5
        * (std::abs(transform.linear()(2, 0)) * size.x()
           + std::abs(transform.linear()(2, 1)) * size.y()
           + std::abs(transform.linear()(2, 2)) * size.z());
  transform.translation().z() = verticalHalfExtent - kInitialPenetration;
  joint->setPositions(dynamics::FreeJoint::convertToPositions(transform));
  joint->setLinearVelocity(Eigen::Vector3d(kPainleveInitialVelocity, 0.0, 0.0));
  return skeleton;
}

dynamics::SkeletonPtr createTurntableRider(double frictionCoeff)
{
  const Eigen::Vector3d size = Eigen::Vector3d::Constant(0.25);
  constexpr double kMass = 1.0;
  constexpr double kInitialPenetration = 0.005;

  auto skeleton = dynamics::Skeleton::create("turntable_rider");
  dynamics::GenericJoint<math::SE3Space>::Properties jointProperties(
      std::string("turntable_rider_joint"));
  dynamics::BodyNode::Properties bodyProperties(
      dynamics::BodyNode::AspectProperties("turntable_rider_body"));
  bodyProperties.mInertia.setMass(kMass);
  bodyProperties.mInertia.setMoment(
      dynamics::BoxShape::computeInertia(size, kMass));

  auto pair = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>(
      nullptr, jointProperties, bodyProperties);
  auto* joint = pair.first;
  auto* body = pair.second;
  auto* shapeNode = body->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(std::make_shared<dynamics::BoxShape>(size));
  shapeNode->getDynamicsAspect()->setFrictionCoeff(frictionCoeff);

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(
      kTurntableInitialRadius, 0.0, 0.5 * size.z() - kInitialPenetration);
  joint->setPositions(dynamics::FreeJoint::convertToPositions(transform));
  return skeleton;
}

dynamics::SkeletonPtr createAuthorTurntableSupport(double frictionCoeff)
{
  auto skeleton = dynamics::Skeleton::create("turntable");
  dynamics::GenericJoint<math::SE3Space>::Properties jointProperties(
      std::string("turntable_joint"));
  dynamics::BodyNode::Properties bodyProperties(
      dynamics::BodyNode::AspectProperties("turntable_body"));
  bodyProperties.mInertia.setMass(1.0);

  auto pair = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>(
      nullptr, jointProperties, bodyProperties);
  auto* joint = pair.first;
  auto* body = pair.second;
  auto* shapeNode = body->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(std::make_shared<dynamics::CylinderShape>(
      fbf_author_turntable::kSupportRadius,
      2.0 * fbf_author_turntable::kSupportHalfHeight));
  shapeNode->getDynamicsAspect()->setFrictionCoeff(frictionCoeff);

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation().z() = fbf_author_turntable::kSupportHalfHeight;
  joint->setPositions(dynamics::FreeJoint::convertToPositions(transform));
  skeleton->setMobile(false);
  return skeleton;
}

dynamics::SkeletonPtr createAuthorTurntableRider(double frictionCoeff)
{
  constexpr double kSide = 2.0 * fbf_author_turntable::kRiderHalfSize;
  constexpr double kMass
      = fbf_author_turntable::kRiderDensity * kSide * kSide * kSide;
  const Eigen::Vector3d size = Eigen::Vector3d::Constant(kSide);

  auto skeleton = dynamics::Skeleton::create("turntable_rider");
  dynamics::GenericJoint<math::SE3Space>::Properties jointProperties(
      std::string("turntable_rider_joint"));
  dynamics::BodyNode::Properties bodyProperties(
      dynamics::BodyNode::AspectProperties("turntable_rider_body"));
  bodyProperties.mInertia.setMass(kMass);
  bodyProperties.mInertia.setMoment(
      dynamics::BoxShape::computeInertia(size, kMass));

  auto pair = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>(
      nullptr, jointProperties, bodyProperties);
  auto* joint = pair.first;
  auto* body = pair.second;
  auto* shapeNode = body->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(std::make_shared<dynamics::BoxShape>(size));
  shapeNode->getDynamicsAspect()->setFrictionCoeff(frictionCoeff);

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(
      fbf_author_turntable::kInitialRadius,
      0.0,
      2.0 * fbf_author_turntable::kSupportHalfHeight
          + fbf_author_turntable::kRiderHalfSize
          + fbf_author_turntable::kGeometricGap
          + fbf_author_turntable::kDropHeight);
  joint->setPositions(dynamics::FreeJoint::convertToPositions(transform));
  return skeleton;
}

dynamics::SkeletonPtr createAuthorCardHouseBox(
    const std::string& name,
    const Eigen::Vector3d& size,
    const Eigen::Isometry3d& transform,
    double density,
    bool mobile)
{
  const double mass = density * size.x() * size.y() * size.z();
  auto skeleton = dynamics::Skeleton::create(name);
  dynamics::GenericJoint<math::SE3Space>::Properties jointProperties(
      name + "_joint");
  dynamics::BodyNode::Properties bodyProperties(
      dynamics::BodyNode::AspectProperties(name + "_body"));
  bodyProperties.mInertia.setMass(mass);
  bodyProperties.mInertia.setMoment(
      dynamics::BoxShape::computeInertia(size, mass));

  auto pair = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>(
      nullptr, jointProperties, bodyProperties);
  auto* joint = pair.first;
  auto* body = pair.second;
  auto* shapeNode = body->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(std::make_shared<dynamics::BoxShape>(size));
  shapeNode->getDynamicsAspect()->setFrictionCoeff(
      fbf_author_card_house::kFriction);
  joint->setPositions(dynamics::FreeJoint::convertToPositions(transform));
  skeleton->setMobile(mobile);
  return skeleton;
}

dynamics::SkeletonPtr createCardPlate(
    const std::string& name,
    const Eigen::Isometry3d& transform,
    double frictionCoeff)
{
  const Eigen::Vector3d size(
      kCardHouseThickness, kCardHouseWidth, kCardHouseHeight);
  const double mass = kCardHouseDensity * size.x() * size.y() * size.z();

  auto skeleton = dynamics::Skeleton::create(name);
  dynamics::GenericJoint<math::SE3Space>::Properties jointProperties(
      name + "_joint");
  dynamics::BodyNode::Properties bodyProperties(
      dynamics::BodyNode::AspectProperties(name + "_body"));
  bodyProperties.mInertia.setMass(mass);
  bodyProperties.mInertia.setMoment(
      dynamics::BoxShape::computeInertia(size, mass));

  auto pair = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>(
      nullptr, jointProperties, bodyProperties);
  auto* joint = pair.first;
  auto* body = pair.second;
  auto* shapeNode = body->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(std::make_shared<dynamics::BoxShape>(size));
  shapeNode->getDynamicsAspect()->setFrictionCoeff(frictionCoeff);

  joint->setPositions(dynamics::FreeJoint::convertToPositions(transform));
  return skeleton;
}

dynamics::SkeletonPtr createCardHouseProjectile(std::size_t index)
{
  auto skeleton
      = dynamics::Skeleton::create("fbf_projectile_" + std::to_string(index));
  dynamics::GenericJoint<math::SE3Space>::Properties jointProperties(
      skeleton->getName() + "_joint");
  dynamics::BodyNode::Properties bodyProperties(
      dynamics::BodyNode::AspectProperties(skeleton->getName() + "_body"));
  bodyProperties.mInertia.setMass(kCardHouseProjectileMass);
  bodyProperties.mInertia.setMoment(dynamics::BoxShape::computeInertia(
      Eigen::Vector3d::Constant(kCardHouseProjectileEdgeLength),
      kCardHouseProjectileMass));

  auto pair = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>(
      nullptr, jointProperties, bodyProperties);
  auto* joint = pair.first;
  auto* body = pair.second;
  auto* shapeNode = body->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(std::make_shared<dynamics::BoxShape>(
      Eigen::Vector3d::Constant(kCardHouseProjectileEdgeLength)));
  shapeNode->getDynamicsAspect()->setFrictionCoeff(kCardHouseFriction);

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(
      (static_cast<double>(index) - 1.5) * 0.15,
      0.0,
      kCardHouseProjectileDropHeight);
  joint->setPositions(dynamics::FreeJoint::convertToPositions(transform));
  joint->setLinearVelocity(
      Eigen::Vector3d(0.0, 0.0, -kCardHouseProjectileSpeed));
  return skeleton;
}

dynamics::SkeletonPtr createMasonryArchStone(
    std::size_t index,
    const math::detail::MasonryArchStoneBoxGeometry& geometry)
{
  const double mass = kArchDensity * geometry.size.x() * geometry.size.y()
                      * geometry.size.z();

  auto skeleton = dynamics::Skeleton::create(
      "masonry_arch_stone_" + std::to_string(index));
  dynamics::GenericJoint<math::SE3Space>::Properties jointProperties(
      skeleton->getName() + "_joint");
  dynamics::BodyNode::Properties bodyProperties(
      dynamics::BodyNode::AspectProperties(skeleton->getName() + "_body"));
  bodyProperties.mInertia.setMass(mass);
  bodyProperties.mInertia.setMoment(
      dynamics::BoxShape::computeInertia(geometry.size, mass));

  auto pair = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>(
      nullptr, jointProperties, bodyProperties);
  auto* joint = pair.first;
  auto* body = pair.second;
  auto* shapeNode = body->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(
      std::make_shared<dynamics::BoxShape>(geometry.size));
  shapeNode->getDynamicsAspect()->setFrictionCoeff(kArchFriction);
  joint->setPositions(
      dynamics::FreeJoint::convertToPositions(geometry.transform));
  // Mobility is assigned by the paper-scene builder below; this helper only
  // creates the source-derived geometry, mass, and contact properties.
  return skeleton;
}

dynamics::SkeletonPtr createMasonryArchProjectile(std::size_t index)
{
  auto skeleton = dynamics::Skeleton::create(
      "masonry_arch_projectile_" + std::to_string(index));
  dynamics::GenericJoint<math::SE3Space>::Properties jointProperties(
      skeleton->getName() + "_joint");
  dynamics::BodyNode::Properties bodyProperties(
      dynamics::BodyNode::AspectProperties(skeleton->getName() + "_body"));
  bodyProperties.mInertia.setMass(kArchProjectileMass);
  bodyProperties.mInertia.setMoment(dynamics::BoxShape::computeInertia(
      Eigen::Vector3d::Constant(kArchProjectileEdgeLength),
      kArchProjectileMass));

  auto pair = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>(
      nullptr, jointProperties, bodyProperties);
  auto* joint = pair.first;
  auto* body = pair.second;
  auto* shapeNode = body->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(std::make_shared<dynamics::BoxShape>(
      Eigen::Vector3d::Constant(kArchProjectileEdgeLength)));
  shapeNode->getDynamicsAspect()->setFrictionCoeff(kArchFriction);

  // Aim at the crown (topmost, narrowest) stone, just outside its depth
  // face, regardless of stone count.
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(
      (static_cast<double>(index)
       - 0.5 * static_cast<double>(kArchProjectileCount - 1u))
          * kArchProjectileSpacing,
      0.0,
      kArchProjectileDropHeight);
  joint->setPositions(dynamics::FreeJoint::convertToPositions(transform));
  joint->setLinearVelocity(Eigen::Vector3d(0.0, 0.0, -kArchProjectileSpeed));
  return skeleton;
}

double computeCardHouseVerticalHalfExtent(const Eigen::Matrix3d& rotation)
{
  const Eigen::Vector3d size(
      kCardHouseThickness, kCardHouseWidth, kCardHouseHeight);
  return 0.5
         * (std::abs(rotation(2, 0)) * size.x()
            + std::abs(rotation(2, 1)) * size.y()
            + std::abs(rotation(2, 2)) * size.z());
}

Eigen::Isometry3d createCardHouseAFrameTransform(
    double centerX, double baseZ, bool leftCard)
{
  const double angle = leftCard ? kCardHouseAngle : -kCardHouseAngle;

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear()
      = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY()).toRotationMatrix();

  const double horizontalHalfExtent
      = 0.5
        * (kCardHouseHeight * std::sin(kCardHouseAngle)
           + kCardHouseThickness * std::cos(kCardHouseAngle));
  const double centerOffset
      = horizontalHalfExtent - 0.5 * kCardHouseInitialPenetration;
  transform.translation().x()
      = centerX + (leftCard ? -centerOffset : centerOffset);
  transform.translation().y() = 0.0;
  const double verticalHalfExtent
      = computeCardHouseVerticalHalfExtent(transform.linear());
  transform.translation().z()
      = baseZ + verticalHalfExtent - kCardHouseInitialPenetration;
  return transform;
}

Eigen::Isometry3d createCardHouseAFrameTransform(bool leftCard)
{
  return createCardHouseAFrameTransform(0.0, 0.0, leftCard);
}

Eigen::Isometry3d createCardHouseHorizontalSupportTransform(
    double centerX, double baseZ)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = Eigen::AngleAxisd(0.5 * kPi, Eigen::Vector3d::UnitY())
                           .toRotationMatrix();
  transform.translation().x() = centerX;
  transform.translation().y() = 0.0;
  transform.translation().z()
      = baseZ + 0.5 * kCardHouseThickness - kCardHouseInitialPenetration;
  return transform;
}

constraint::ExactCoulombFbfConstraintSolverOptions
makePaperFixtureSolverOptions()
{
  constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.maxOuterIterations = 500;
  options.tolerance = 1e-6;
  options.innerMaxSweeps = 120;
  options.innerLocalIterations = 32;
  options.stepSizeScale = kSmallFixtureStepSizeScale;
  return options;
}

constraint::ExactCoulombFbfConstraintSolverOptions
makeCardHousePrecursorSolverOptions()
{
  auto options = makePaperFixtureSolverOptions();
  options.maxOuterIterations = 5000;
  options.tolerance = 1e-6;
  options.innerMaxSweeps = 240;
  options.innerLocalIterations = 64;
  options.stepSizeScale = 1.0;
  options.enableWarmStart = false;
  return options;
}

constraint::ExactCoulombFbfConstraintSolverOptions
makeCardHouseFourLevelProbeSolverOptions()
{
  auto options = makeCardHousePrecursorSolverOptions();
  options.maxOuterIterations = 30000;
  options.innerMaxSweeps = 120;
  options.innerLocalIterations = 32;
  options.projectedGradientMaxIterations = 200;
  options.stepSizeScale = kCardHouseStepSizeScale;
  options.outerRelaxation = kCardHouseOuterRelaxation;
  return options;
}

constraint::ExactCoulombFbfConstraintSolverOptions
makeMasonryArchSolverOptions()
{
  auto options = makePaperFixtureSolverOptions();
  options.maxOuterIterations = kArchMaxOuterIterations;
  options.stepSizeScale = kArchStepSizeScale;
  options.outerRelaxation = kArchOuterRelaxation;
  options.enableWarmStart = false;
  return options;
}

std::shared_ptr<collision::NativeCollisionDetector>
createFbfPaperNativeCollisionDetector(
    collision::NativeCollisionDetector::ContactManifoldMode manifoldMode
    = collision::NativeCollisionDetector::ContactManifoldMode::Compact)
{
  auto detector = collision::NativeCollisionDetector::create();
  detector->setContactManifoldMode(manifoldMode);
  return detector;
}

std::shared_ptr<simulation::World> createInclineWorld(
    double frictionCoeff, bool exactCoulomb)
{
  auto world = simulation::World::create("exact_coulomb_incline");
  world->setTimeStep(kInclineDt);
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -kGravity));
  world->setNumSimulationThreads(1u);

  simulation::DeactivationOptions deactivation;
  deactivation.mEnabled = false;
  world->setDeactivationOptions(deactivation);

  if (exactCoulomb) {
    auto solver = std::make_unique<constraint::ExactCoulombFbfConstraintSolver>(
        makePaperFixtureSolverOptions());
    solver->setCollisionDetector(createFbfPaperNativeCollisionDetector(
        collision::NativeCollisionDetector::ContactManifoldMode::
            FourPointPlanar));
    solver->setNumSimulationThreads(1u);
    world->setConstraintSolver(std::move(solver));
  } else {
    auto* solver = world->getConstraintSolver();
    solver->setCollisionDetector(createFbfPaperNativeCollisionDetector(
        collision::NativeCollisionDetector::ContactManifoldMode::
            FourPointPlanar));
    solver->setNumSimulationThreads(1u);
  }

  auto& collisionOption = world->getConstraintSolver()->getCollisionOption();
  collisionOption.maxNumContacts = 4u;
  collisionOption.maxNumContactsPerPair = 4u;

  world->addSkeleton(createInclinePlane(frictionCoeff));
  world->addSkeleton(createInclineCube(frictionCoeff));
  return world;
}

std::shared_ptr<simulation::World> createPainleveWorld(
    double frictionCoeff, bool exactCoulomb)
{
  auto world = simulation::World::create("exact_coulomb_painleve");
  world->setTimeStep(kInclineDt);
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -kGravity));
  world->setNumSimulationThreads(1u);

  simulation::DeactivationOptions deactivation;
  deactivation.mEnabled = false;
  world->setDeactivationOptions(deactivation);

  if (exactCoulomb) {
    auto solver = std::make_unique<constraint::ExactCoulombFbfConstraintSolver>(
        makePaperFixtureSolverOptions());
    solver->setCollisionDetector(collision::DARTCollisionDetector::create());
    solver->setNumSimulationThreads(1u);
    world->setConstraintSolver(std::move(solver));
  } else {
    auto* solver = world->getConstraintSolver();
    solver->setCollisionDetector(collision::DARTCollisionDetector::create());
    solver->setNumSimulationThreads(1u);
  }

  auto& collisionOption = world->getConstraintSolver()->getCollisionOption();
  collisionOption.maxNumContacts = 4u;
  collisionOption.maxNumContactsPerPair = 4u;

  world->addSkeleton(createHorizontalPlane(frictionCoeff));
  world->addSkeleton(createPainleveBox(frictionCoeff));
  return world;
}

std::shared_ptr<simulation::World> createTurntableWorld(
    double frictionCoeff, bool exactCoulomb)
{
  auto world = simulation::World::create("exact_coulomb_turntable");
  world->setTimeStep(kInclineDt);
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -kGravity));
  world->setNumSimulationThreads(1u);

  simulation::DeactivationOptions deactivation;
  deactivation.mEnabled = false;
  world->setDeactivationOptions(deactivation);

  if (exactCoulomb) {
    auto solver = std::make_unique<constraint::ExactCoulombFbfConstraintSolver>(
        makePaperFixtureSolverOptions());
    solver->setCollisionDetector(createFbfPaperNativeCollisionDetector(
        collision::NativeCollisionDetector::ContactManifoldMode::
            FourPointPlanar));
    solver->setNumSimulationThreads(1u);
    world->setConstraintSolver(std::move(solver));
  } else {
    auto* solver = world->getConstraintSolver();
    solver->setCollisionDetector(createFbfPaperNativeCollisionDetector(
        collision::NativeCollisionDetector::ContactManifoldMode::
            FourPointPlanar));
    solver->setNumSimulationThreads(1u);
  }

  auto& collisionOption = world->getConstraintSolver()->getCollisionOption();
  collisionOption.maxNumContacts = 4u;
  collisionOption.maxNumContactsPerPair = 4u;

  world->addSkeleton(createTurntableSupport(frictionCoeff));
  world->addSkeleton(createTurntableRider(frictionCoeff));
  return world;
}

std::shared_ptr<simulation::World> createAuthorTurntableWorld(
    double frictionCoeff, bool exactCoulomb)
{
  auto world = simulation::World::create("exact_coulomb_author_turntable");
  world->setTimeStep(kInclineDt);
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -kGravity));
  world->setNumSimulationThreads(1u);

  simulation::DeactivationOptions deactivation;
  deactivation.mEnabled = false;
  world->setDeactivationOptions(deactivation);

  if (exactCoulomb) {
    auto solver = std::make_unique<constraint::ExactCoulombFbfConstraintSolver>(
        fbf_author_turntable::dartBestSolverOptions());
    solver->setCollisionDetector(createFbfPaperNativeCollisionDetector(
        collision::NativeCollisionDetector::ContactManifoldMode::
            FourPointPlanar));
    solver->setNumSimulationThreads(1u);
    world->setConstraintSolver(std::move(solver));
  } else {
    auto* solver = world->getConstraintSolver();
    solver->setCollisionDetector(createFbfPaperNativeCollisionDetector(
        collision::NativeCollisionDetector::ContactManifoldMode::
            FourPointPlanar));
    solver->setNumSimulationThreads(1u);
  }

  auto& collisionOption = world->getConstraintSolver()->getCollisionOption();
  collisionOption.maxNumContacts = fbf_author_turntable::kMaxContacts;
  collisionOption.maxNumContactsPerPair
      = fbf_author_turntable::kMaxContactsPerPair;

  world->addSkeleton(createAuthorTurntableSupport(frictionCoeff));
  world->addSkeleton(createAuthorTurntableRider(frictionCoeff));
  return world;
}

std::shared_ptr<simulation::World> createAuthorCardHouseConstructionWorld()
{
  auto world = simulation::World::create(fbf_author_card_house::kDemoSceneId);
  world->setTimeStep(fbf_author_card_house::kRuntimeTimeStep);
  world->setGravity(
      Eigen::Vector3d(0.0, 0.0, -fbf_author_card_house::kGravity));
  world->setNumSimulationThreads(1u);

  simulation::DeactivationOptions deactivation;
  deactivation.mEnabled = false;
  world->setDeactivationOptions(deactivation);

  auto solver = std::make_unique<constraint::ExactCoulombFbfConstraintSolver>(
      fbf_author_card_house::dartConstructionSolverOptions());
  solver->setCollisionDetector(createFbfPaperNativeCollisionDetector(
      collision::NativeCollisionDetector::ContactManifoldMode::
          FourPointPlanar));
  solver->setNumSimulationThreads(1u);
  world->setConstraintSolver(std::move(solver));
  auto& collisionOption = world->getConstraintSolver()->getCollisionOption();
  collisionOption.maxNumContacts = fbf_author_card_house::kSourceMaxContacts;
  collisionOption.maxNumContactsPerPair = 4u;

  world->addSkeleton(createHorizontalPlane(fbf_author_card_house::kFriction));
  for (const auto& spec : fbf_author_card_house::makeCardSpecs(
           fbf_author_card_house::kDefaultLevelCount)) {
    world->addSkeleton(createAuthorCardHouseBox(
        spec.name,
        spec.size,
        spec.transform,
        fbf_author_card_house::kCardDensity,
        true));
  }
  for (const auto& spec : fbf_author_card_house::makeCubeSpecs()) {
    world->addSkeleton(createAuthorCardHouseBox(
        spec.name,
        spec.size,
        spec.transform,
        fbf_author_card_house::kCubeDensity,
        false));
  }
  return world;
}

std::shared_ptr<simulation::World> createCardHousePrecursorWorld(
    bool exactCoulomb)
{
  auto world = simulation::World::create("exact_coulomb_card_house_precursor");
  world->setTimeStep(kInclineDt);
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -kGravity));
  world->setNumSimulationThreads(1u);

  simulation::DeactivationOptions deactivation;
  deactivation.mEnabled = false;
  world->setDeactivationOptions(deactivation);

  if (exactCoulomb) {
    auto solver = std::make_unique<constraint::ExactCoulombFbfConstraintSolver>(
        makeCardHousePrecursorSolverOptions());
    solver->setCollisionDetector(collision::DARTCollisionDetector::create());
    solver->setNumSimulationThreads(1u);
    world->setConstraintSolver(std::move(solver));
  } else {
    auto* solver = world->getConstraintSolver();
    solver->setCollisionDetector(collision::DARTCollisionDetector::create());
    solver->setNumSimulationThreads(1u);
  }

  auto& collisionOption = world->getConstraintSolver()->getCollisionOption();
  collisionOption.maxNumContacts = 32u;
  collisionOption.maxNumContactsPerPair = 8u;

  world->addSkeleton(createHorizontalPlane(kCardHouseFriction));
  world->addSkeleton(createCardPlate(
      "card_house_left",
      createCardHouseAFrameTransform(true),
      kCardHouseFriction));
  world->addSkeleton(createCardPlate(
      "card_house_right",
      createCardHouseAFrameTransform(false),
      kCardHouseFriction));
  return world;
}

std::shared_ptr<simulation::World> createCardHouseWorld(
    const std::string& name,
    std::size_t levelCount,
    bool exactCoulomb,
    const constraint::ExactCoulombFbfConstraintSolverOptions& exactOptions
    = makeCardHousePrecursorSolverOptions(),
    std::size_t maxContacts = 256u,
    std::size_t maxContactsPerPair = 8u)
{
  auto world = simulation::World::create(name);
  world->setTimeStep(kInclineDt);
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -kGravity));
  world->setNumSimulationThreads(1u);

  simulation::DeactivationOptions deactivation;
  deactivation.mEnabled = false;
  world->setDeactivationOptions(deactivation);

  if (exactCoulomb) {
    auto solver = std::make_unique<constraint::ExactCoulombFbfConstraintSolver>(
        exactOptions);
    solver->setCollisionDetector(collision::DARTCollisionDetector::create());
    solver->setNumSimulationThreads(1u);
    world->setConstraintSolver(std::move(solver));
  } else {
    auto* solver = world->getConstraintSolver();
    solver->setCollisionDetector(collision::DARTCollisionDetector::create());
    solver->setNumSimulationThreads(1u);
  }

  auto& collisionOption = world->getConstraintSolver()->getCollisionOption();
  collisionOption.maxNumContacts = maxContacts;
  collisionOption.maxNumContactsPerPair = maxContactsPerPair;

  world->addSkeleton(createHorizontalPlane(kCardHouseFriction));

  const Eigen::Matrix3d aFrameRotation
      = Eigen::AngleAxisd(kCardHouseAngle, Eigen::Vector3d::UnitY())
            .toRotationMatrix();
  const double aFrameHeight
      = 2.0 * computeCardHouseVerticalHalfExtent(aFrameRotation);
  double baseZ = 0.0;

  for (std::size_t level = 0u; level < levelCount; ++level) {
    const std::size_t frameCount = levelCount - level;
    for (std::size_t frame = 0u; frame < frameCount; ++frame) {
      const double centerX
          = (static_cast<double>(frame) - 0.5 * (frameCount - 1))
            * kCardHouseFrameSpacing;
      world->addSkeleton(createCardPlate(
          "card_house_l" + std::to_string(level) + "_f" + std::to_string(frame)
              + "_left",
          createCardHouseAFrameTransform(centerX, baseZ, true),
          kCardHouseFriction));
      world->addSkeleton(createCardPlate(
          "card_house_l" + std::to_string(level) + "_f" + std::to_string(frame)
              + "_right",
          createCardHouseAFrameTransform(centerX, baseZ, false),
          kCardHouseFriction));
    }

    if (level + 1u == levelCount) {
      break;
    }

    const double supportBaseZ
        = baseZ + aFrameHeight - kCardHouseInitialPenetration;
    for (std::size_t support = 0u; support + 1u < frameCount; ++support) {
      const double centerX
          = (static_cast<double>(support) + 0.5 - 0.5 * (frameCount - 1))
            * kCardHouseFrameSpacing;
      world->addSkeleton(createCardPlate(
          "card_house_l" + std::to_string(level) + "_support"
              + std::to_string(support),
          createCardHouseHorizontalSupportTransform(centerX, supportBaseZ),
          kCardHouseFriction));
    }

    baseZ = supportBaseZ + kCardHouseThickness - kCardHouseInitialPenetration;
  }

  return world;
}

std::shared_ptr<simulation::World> createCardHouseFourLevelWorld(
    bool exactCoulomb,
    const constraint::ExactCoulombFbfConstraintSolverOptions& exactOptions
    = makeCardHousePrecursorSolverOptions(),
    std::size_t maxContacts = 256u,
    std::size_t maxContactsPerPair = 8u)
{
  return createCardHouseWorld(
      "exact_coulomb_card_house_4_level",
      kCardHouseFourLevelCount,
      exactCoulomb,
      exactOptions,
      maxContacts,
      maxContactsPerPair);
}

std::shared_ptr<simulation::World> createCardHouseTenLevelWorld(
    bool exactCoulomb,
    const constraint::ExactCoulombFbfConstraintSolverOptions& exactOptions
    = makeCardHousePrecursorSolverOptions(),
    std::size_t maxContacts = 512u,
    std::size_t maxContactsPerPair = 8u)
{
  return createCardHouseWorld(
      "exact_coulomb_card_house_10_level",
      kCardHouseTenLevelCount,
      exactCoulomb,
      exactOptions,
      maxContacts,
      maxContactsPerPair);
}

std::shared_ptr<simulation::World> createMasonryArchWorld(
    const std::string& name,
    std::size_t stoneCount,
    bool exactCoulomb,
    std::size_t maxContacts,
    std::size_t maxContactsPerPair)
{
  auto world = simulation::World::create(name);
  world->setTimeStep(kInclineDt);
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -kGravity));
  world->setNumSimulationThreads(1u);

  simulation::DeactivationOptions deactivation;
  deactivation.mEnabled = false;
  world->setDeactivationOptions(deactivation);

  if (exactCoulomb) {
    auto solver = std::make_unique<constraint::ExactCoulombFbfConstraintSolver>(
        makeMasonryArchSolverOptions());
    solver->setCollisionDetector(collision::DARTCollisionDetector::create());
    solver->setNumSimulationThreads(1u);
    world->setConstraintSolver(std::move(solver));
  } else {
    auto* solver = world->getConstraintSolver();
    solver->setCollisionDetector(collision::DARTCollisionDetector::create());
    solver->setNumSimulationThreads(1u);
  }

  auto& collisionOption = world->getConstraintSolver()->getCollisionOption();
  collisionOption.maxNumContacts = maxContacts;
  collisionOption.maxNumContactsPerPair = maxContactsPerPair;

  // The paper explicitly pins both springers in the 25-stone scene. Figure 8
  // calls the 101-stone experiment the same setup, so the same endpoint
  // condition is used there. The credited raw Rigid-IPC scenes instead leave
  // every stone dynamic; that source distinction is retained in provenance.
  world->addSkeleton(createHorizontalPlane(kArchFriction));

  const auto stoneGeometry
      = math::detail::generateMasonryArchStoneBoxes(stoneCount);
  for (std::size_t i = 0u; i < stoneCount; ++i) {
    auto stone = createMasonryArchStone(i, stoneGeometry[i]);
    if (i == 0u || i + 1u == stoneCount)
      stone->setMobile(false);
    world->addSkeleton(stone);
  }

  return world;
}

std::shared_ptr<simulation::World> createMasonryArch25World(
    bool exactCoulomb,
    std::size_t maxContacts = kArchReducedMaxContacts,
    std::size_t maxContactsPerPair = kArchReducedMaxContactsPerPair)
{
  return createMasonryArchWorld(
      "exact_coulomb_masonry_arch_25",
      kArchStoneCount,
      exactCoulomb,
      maxContacts,
      maxContactsPerPair);
}

std::shared_ptr<simulation::World> createMasonryArch101World(
    bool exactCoulomb,
    std::size_t maxContacts = kArch101ReducedMaxContacts,
    std::size_t maxContactsPerPair = kArch101ReducedMaxContactsPerPair)
{
  return createMasonryArchWorld(
      "exact_coulomb_masonry_arch_101",
      kArch101StoneCount,
      exactCoulomb,
      maxContacts,
      maxContactsPerPair);
}

std::shared_ptr<simulation::World> createBackspinWorld(bool exactCoulomb)
{
  auto world = simulation::World::create("exact_coulomb_backspin");
  world->setTimeStep(kInclineDt);
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -kGravity));
  world->setNumSimulationThreads(1u);

  simulation::DeactivationOptions deactivation;
  deactivation.mEnabled = false;
  world->setDeactivationOptions(deactivation);

  if (exactCoulomb) {
    auto solver = std::make_unique<constraint::ExactCoulombFbfConstraintSolver>(
        makePaperFixtureSolverOptions());
    solver->setCollisionDetector(collision::DARTCollisionDetector::create());
    solver->setNumSimulationThreads(1u);
    world->setConstraintSolver(std::move(solver));
  } else {
    auto* solver = world->getConstraintSolver();
    solver->setCollisionDetector(collision::DARTCollisionDetector::create());
    solver->setNumSimulationThreads(1u);
  }

  auto& collisionOption = world->getConstraintSolver()->getCollisionOption();
  collisionOption.maxNumContacts = 1u;
  collisionOption.maxNumContactsPerPair = 1u;

  world->addSkeleton(createHorizontalPlane(kBackspinFriction));

  constexpr double kMass = 1.0;
  constexpr double kInitialPenetration = 0.005;
  auto sphere = dynamics::Skeleton::create("backspin_sphere");
  dynamics::GenericJoint<math::SE3Space>::Properties jointProperties(
      std::string("backspin_sphere_joint"));
  dynamics::BodyNode::Properties bodyProperties(
      dynamics::BodyNode::AspectProperties("backspin_sphere_body"));
  bodyProperties.mInertia.setMass(kMass);
  bodyProperties.mInertia.setMoment(
      dynamics::SphereShape::computeInertia(kBackspinRadius, kMass));

  auto pair = sphere->createJointAndBodyNodePair<dynamics::FreeJoint>(
      nullptr, jointProperties, bodyProperties);
  auto* joint = pair.first;
  auto* body = pair.second;
  auto* shapeNode = body->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(
      std::make_shared<dynamics::SphereShape>(kBackspinRadius));
  shapeNode->getDynamicsAspect()->setFrictionCoeff(kBackspinFriction);

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation().z() = kBackspinRadius - kInitialPenetration;
  joint->setPositions(dynamics::FreeJoint::convertToPositions(transform));
  joint->setLinearVelocity(Eigen::Vector3d(kBackspinLinearVelocity, 0.0, 0.0));
  joint->setAngularVelocity(
      Eigen::Vector3d(0.0, kBackspinAngularVelocity, 0.0));

  world->addSkeleton(sphere);
  return world;
}

const constraint::ExactCoulombFbfConstraintSolver* getExactCoulombSolver(
    const std::shared_ptr<simulation::World>& world)
{
  const auto* solver
      = dynamic_cast<const constraint::ExactCoulombFbfConstraintSolver*>(
          world->getConstraintSolver());
  EXPECT_NE(nullptr, solver);
  return solver;
}

void sampleExactResidualTrace(
    const constraint::ExactCoulombFbfConstraintSolver* solver,
    std::size_t& previousExactSolves,
    ExactResidualTrace& trace)
{
  if (solver == nullptr)
    return;

  const std::size_t exactSolves = solver->getNumExactCoulombSolves();
  if (exactSolves == previousExactSolves)
    return;

  previousExactSolves = exactSolves;
  ++trace.sampledSteps;

  if (solver->getLastExactCoulombStatus()
          != constraint::ExactCoulombFbfConstraintSolverStatus::Success
      || solver->getLastExactCoulombFbfStatus()
             != math::detail::ExactCoulombFbfStatus::Success) {
    ++trace.failedSteps;
  }

  const double residual = solver->getLastExactCoulombResidual();
  if (std::isfinite(residual)) {
    if (std::isfinite(trace.maxResidual))
      trace.maxResidual = std::max(trace.maxResidual, residual);
    else
      trace.maxResidual = residual;
  } else {
    ++trace.nonFiniteResiduals;
  }
}

template <typename Result>
void copyExactCoulombDiagnostics(
    const constraint::ExactCoulombFbfConstraintSolver* solver,
    const ExactResidualTrace& trace,
    Result& result)
{
  if (solver == nullptr)
    return;

  result.residual = solver->getLastExactCoulombResidual();
  const auto& residual = solver->getLastExactCoulombResidualDetails();
  result.primalResidual = residual.primalFeasibility;
  result.dualResidual = residual.dualFeasibility;
  result.complementarityResidual = residual.complementarity;
  result.residualTrace = trace;
  result.exactSolves = solver->getNumExactCoulombSolves();
  result.boxedFallbacks = solver->getNumBoxedLcpFallbacks();
  result.exactFailures = solver->getNumExactCoulombFailures();
  result.projectedGradientRetries
      = solver->getNumExactCoulombProjectedGradientRetries();
  result.fbfStatus = static_cast<int>(solver->getLastExactCoulombFbfStatus());
  result.fbfIterations = solver->getLastExactCoulombIterations();
  result.maxFbfIterations = solver->getMaxExactCoulombIterations();
  result.totalFbfIterations = solver->getTotalExactCoulombIterations();
  result.projectedGradientRetryUsed
      = solver->getLastExactCoulombProjectedGradientRetryUsed();
  result.exactSuccess
      = solver->getLastExactCoulombStatus()
        == constraint::ExactCoulombFbfConstraintSolverStatus::Success;
}

void copyCardHouseFailureDiagnostics(
    const constraint::ExactCoulombFbfConstraintSolver* solver,
    CardHouseRunResult& result)
{
  if (solver == nullptr)
    return;

  result.failedResidual = solver->getLastFailedExactCoulombResidual();
  const auto& residual = solver->getLastFailedExactCoulombResidualDetails();
  result.failedPrimalResidual = residual.primalFeasibility;
  result.failedDualResidual = residual.dualFeasibility;
  result.failedComplementarityResidual = residual.complementarity;
  result.failedFbfStatus
      = static_cast<int>(solver->getLastFailedExactCoulombFbfStatus());
  result.failedFbfIterations = solver->getLastFailedExactCoulombIterations();
}

void expectConvergedResidualTrace(
    const ExactResidualTrace& trace, double tolerance)
{
  EXPECT_GT(trace.sampledSteps, 0u);
  EXPECT_EQ(trace.failedSteps, 0u);
  EXPECT_EQ(trace.nonFiniteResiduals, 0u);
  EXPECT_TRUE(std::isfinite(trace.maxResidual));
  EXPECT_LE(trace.maxResidual, tolerance);
}

InclineRunResult runInclineCube(double frictionCoeff, bool exactCoulomb)
{
  auto world = createInclineWorld(frictionCoeff, exactCoulomb);
  const auto cube = world->getSkeleton("incline_cube");
  const auto* body = cube->getBodyNode(0);
  const Eigen::Vector3d initialPosition
      = body->getWorldTransform().translation();

  const double theta = std::atan(kInclineTan);
  const Eigen::Vector3d downhill(-std::cos(theta), 0.0, -std::sin(theta));
  const int steps = static_cast<int>(std::round(kInclineDuration / kInclineDt));
  const auto* exactSolver
      = exactCoulomb ? getExactCoulombSolver(world) : nullptr;
  ExactResidualTrace residualTrace;
  std::size_t previousExactSolves = 0u;
  std::size_t contactSteps = 0u;
  double maxPenetration = 0.0;
  bool finitePenetration = true;

  for (int i = 0; i < steps; ++i) {
    world->step();
    sampleExactResidualTrace(exactSolver, previousExactSolves, residualTrace);
    const auto& collisionResult
        = world->getConstraintSolver()->getLastCollisionResult();
    if (collisionResult.getNumContacts() > 0u)
      ++contactSteps;
    for (std::size_t contactIndex = 0u;
         contactIndex < collisionResult.getNumContacts();
         ++contactIndex) {
      const double penetration
          = collisionResult.getContact(contactIndex).penetrationDepth;
      if (!std::isfinite(penetration)) {
        finitePenetration = false;
      } else {
        maxPenetration = std::max(maxPenetration, penetration);
      }
    }
  }

  InclineRunResult result;
  const Eigen::Vector3d finalPosition = body->getWorldTransform().translation();
  const Eigen::Vector3d linearVelocity = body->getLinearVelocity();
  result.displacement = (finalPosition - initialPosition).dot(downhill);
  result.speed = linearVelocity.dot(downhill);
  result.maxPenetration = finitePenetration
                              ? maxPenetration
                              : std::numeric_limits<double>::quiet_NaN();
  result.trajectorySteps = static_cast<std::size_t>(steps);
  result.contactSteps = contactSteps;
  result.lastContacts
      = world->getConstraintSolver()->getLastCollisionResult().getNumContacts();

  if (exactCoulomb)
    copyExactCoulombDiagnostics(exactSolver, residualTrace, result);

  return result;
}

BackspinRunResult runBackspinSphere(bool exactCoulomb)
{
  auto world = createBackspinWorld(exactCoulomb);
  const auto sphere = world->getSkeleton("backspin_sphere");
  const auto* body = sphere->getBodyNode(0);
  const int steps
      = static_cast<int>(std::round(kBackspinDuration / kInclineDt));
  const auto* exactSolver
      = exactCoulomb ? getExactCoulombSolver(world) : nullptr;
  ExactResidualTrace residualTrace;
  std::size_t previousExactSolves = 0u;

  for (int i = 0; i < steps; ++i) {
    world->step();
    sampleExactResidualTrace(exactSolver, previousExactSolves, residualTrace);
  }

  BackspinRunResult result;
  result.linearVelocity = body->getLinearVelocity().x();
  result.angularVelocity = body->getAngularVelocity().y();
  result.slipVelocity
      = result.linearVelocity - kBackspinRadius * result.angularVelocity;
  result.height = body->getWorldTransform().translation().z();
  result.lastContacts
      = world->getConstraintSolver()->getLastCollisionResult().getNumContacts();

  if (exactCoulomb)
    copyExactCoulombDiagnostics(exactSolver, residualTrace, result);

  return result;
}

PainleveRunResult runPainleveBox(double frictionCoeff, bool exactCoulomb)
{
  auto world = createPainleveWorld(frictionCoeff, exactCoulomb);
  const auto box = world->getSkeleton("painleve_box");
  const auto* body = box->getBodyNode(0);
  const Eigen::Vector3d initialPosition
      = body->getWorldTransform().translation();
  const int steps
      = static_cast<int>(std::round(kPainleveDuration / kInclineDt));
  const auto* exactSolver
      = exactCoulomb ? getExactCoulombSolver(world) : nullptr;
  ExactResidualTrace residualTrace;
  std::size_t previousExactSolves = 0u;

  PainleveRunResult result;
  for (int i = 0; i < steps; ++i) {
    world->step();
    sampleExactResidualTrace(exactSolver, previousExactSolves, residualTrace);
    if (!std::isfinite(result.tumbleTravel)) {
      const Eigen::Isometry3d transform = body->getWorldTransform();
      const double uprightness = transform.linear().col(2).normalized().dot(
          Eigen::Vector3d::UnitZ());
      const double height = transform.translation().z();
      if (uprightness < 0.55 || height < 0.35) {
        result.tumbleTravel = transform.translation().x() - initialPosition.x();
        result.tumbleTime = static_cast<double>(i + 1) * kInclineDt;
      }
    }
  }

  const Eigen::Isometry3d finalTransform = body->getWorldTransform();
  const Eigen::Vector3d finalPosition = finalTransform.translation();
  result.travel = finalPosition.x() - initialPosition.x();
  result.speed = body->getLinearVelocity().x();
  result.uprightness = finalTransform.linear().col(2).normalized().dot(
      Eigen::Vector3d::UnitZ());
  result.height = finalPosition.z();
  result.lastContacts
      = world->getConstraintSolver()->getLastCollisionResult().getNumContacts();

  if (exactCoulomb)
    copyExactCoulombDiagnostics(exactSolver, residualTrace, result);

  return result;
}

TurntableRunResult runTurntable(
    double frictionCoeff, double angularVelocity, bool exactCoulomb)
{
  auto world = createTurntableWorld(frictionCoeff, exactCoulomb);
  const auto turntable = world->getSkeleton("turntable");
  auto* turntableJoint
      = static_cast<dynamics::FreeJoint*>(turntable->getJoint(0));
  const auto rider = world->getSkeleton("turntable_rider");
  const auto* body = rider->getBodyNode(0);
  const int steps
      = static_cast<int>(std::round(kTurntableDuration / kInclineDt));
  const auto* exactSolver
      = exactCoulomb ? getExactCoulombSolver(world) : nullptr;
  ExactResidualTrace residualTrace;
  std::size_t previousExactSolves = 0u;

  for (int i = 0; i < steps; ++i) {
    const double time = static_cast<double>(i) * kInclineDt;
    const double ramp = std::min(time / kTurntableRampDuration, 1.0);
    turntableJoint->setAngularVelocity(
        Eigen::Vector3d(0.0, 0.0, ramp * angularVelocity));
    world->step();
    sampleExactResidualTrace(exactSolver, previousExactSolves, residualTrace);
  }

  TurntableRunResult result;
  const Eigen::Vector3d position = body->getWorldTransform().translation();
  const Eigen::Vector3d velocity = body->getLinearVelocity();
  const Eigen::Vector2d radial(position.x(), position.y());
  result.radius = radial.norm();
  result.height = position.z();
  if (result.radius > 1e-12) {
    const Eigen::Vector2d radialDirection = radial.normalized();
    const Eigen::Vector2d tangentialDirection(
        -radialDirection.y(), radialDirection.x());
    result.radialVelocity = velocity.head<2>().dot(radialDirection);
    result.tangentialVelocity = velocity.head<2>().dot(tangentialDirection);
    result.tangentialCoRotationError
        = std::abs(result.tangentialVelocity - angularVelocity * result.radius);
  } else {
    result.radialVelocity = 0.0;
    result.tangentialVelocity = 0.0;
    result.tangentialCoRotationError = 0.0;
  }
  result.lastContacts
      = world->getConstraintSolver()->getLastCollisionResult().getNumContacts();

  if (exactCoulomb)
    copyExactCoulombDiagnostics(exactSolver, residualTrace, result);

  return result;
}

TurntableRunResult runAuthorTurntable(
    double frictionCoeff, double angularVelocity, bool exactCoulomb)
{
  auto world = createAuthorTurntableWorld(frictionCoeff, exactCoulomb);
  const auto turntable = world->getSkeleton("turntable");
  auto* turntableJoint
      = static_cast<dynamics::FreeJoint*>(turntable->getJoint(0));
  const auto rider = world->getSkeleton("turntable_rider");
  const auto* body = rider->getBodyNode(0);
  const int steps = static_cast<int>(
      std::round(fbf_author_turntable::kDuration / kInclineDt));
  const auto* exactSolver
      = exactCoulomb ? getExactCoulombSolver(world) : nullptr;
  ExactResidualTrace residualTrace;
  std::size_t previousExactSolves = 0u;
  TurntableRunResult result;

  for (int i = 0; i < steps; ++i) {
    const double time = static_cast<double>(i) * kInclineDt;
    Eigen::Isometry3d supportTransform = Eigen::Isometry3d::Identity();
    supportTransform.linear()
        = Eigen::AngleAxisd(
              fbf_author_turntable::integratedYaw(time, angularVelocity),
              Eigen::Vector3d::UnitZ())
              .toRotationMatrix();
    supportTransform.translation().z()
        = fbf_author_turntable::kSupportHalfHeight;
    turntableJoint->setPositions(
        dynamics::FreeJoint::convertToPositions(supportTransform));
    turntableJoint->setAngularVelocity(Eigen::Vector3d(
        0.0,
        0.0,
        fbf_author_turntable::angularVelocity(time, angularVelocity)));
    world->step();
    sampleExactResidualTrace(exactSolver, previousExactSolves, residualTrace);

    const Eigen::Vector3d position = body->getWorldTransform().translation();
    const double radius = position.head<2>().norm();
    result.finiteState = result.finiteState && position.allFinite()
                         && body->getLinearVelocity().allFinite()
                         && body->getAngularVelocity().allFinite();
    result.maxRadius = std::max(result.maxRadius, radius);

    const double tableTop = 2.0 * fbf_author_turntable::kSupportHalfHeight;
    result.sourceExitObserved
        = result.sourceExitObserved
          || radius > fbf_author_turntable::kSupportRadius
                          + fbf_author_turntable::kRiderHalfSize
          || position.z() < tableTop - 0.5;

    const double fullySupportedRadius = fbf_author_turntable::kSupportRadius
                                        - fbf_author_turntable::kRiderHalfSize;
    if (radius <= fullySupportedRadius) {
      result.minFullySupportedHeight
          = std::min(result.minFullySupportedHeight, position.z());
      result.fellThroughSupport
          = result.fellThroughSupport
            || position.z() < tableTop - fbf_author_turntable::kRiderHalfSize;
    }
  }

  const Eigen::Vector3d position = body->getWorldTransform().translation();
  const Eigen::Vector3d velocity = body->getLinearVelocity();
  const Eigen::Vector2d radial(position.x(), position.y());
  result.radius = radial.norm();
  result.height = position.z();
  if (result.radius > 1e-12) {
    const Eigen::Vector2d radialDirection = radial.normalized();
    const Eigen::Vector2d tangentialDirection(
        -radialDirection.y(), radialDirection.x());
    result.radialVelocity = velocity.head<2>().dot(radialDirection);
    result.tangentialVelocity = velocity.head<2>().dot(tangentialDirection);
    result.tangentialCoRotationError
        = std::abs(result.tangentialVelocity - angularVelocity * result.radius);
  } else {
    result.radialVelocity = 0.0;
    result.tangentialVelocity = 0.0;
    result.tangentialCoRotationError = 0.0;
  }
  result.lastContacts
      = world->getConstraintSolver()->getLastCollisionResult().getNumContacts();

  if (exactCoulomb)
    copyExactCoulombDiagnostics(exactSolver, residualTrace, result);

  return result;
}

CardHouseRunResult runCardHousePrecursor(bool exactCoulomb)
{
  auto world = createCardHousePrecursorWorld(exactCoulomb);
  const int steps
      = static_cast<int>(std::round(kCardHouseDuration / kInclineDt));
  const auto* exactSolver
      = exactCoulomb ? getExactCoulombSolver(world) : nullptr;
  ExactResidualTrace residualTrace;
  std::size_t previousExactSolves = 0u;

  for (int i = 0; i < steps; ++i) {
    world->step();
    sampleExactResidualTrace(exactSolver, previousExactSolves, residualTrace);
  }

  CardHouseRunResult result;
  result.minCardAxisUp = std::numeric_limits<double>::infinity();
  result.minCenterHeight = std::numeric_limits<double>::infinity();
  result.maxHorizontalTravel = 0.0;
  result.finiteState = true;

  for (const auto* name : {"card_house_left", "card_house_right"}) {
    const auto card = world->getSkeleton(name);
    const auto* body = card->getBodyNode(0);
    const Eigen::Isometry3d transform = body->getWorldTransform();
    const Eigen::Vector3d position = transform.translation();
    result.finiteState = result.finiteState && position.allFinite()
                         && body->getLinearVelocity().allFinite()
                         && body->getAngularVelocity().allFinite();
    result.minCardAxisUp = std::min(
        result.minCardAxisUp,
        transform.linear().col(2).normalized().dot(Eigen::Vector3d::UnitZ()));
    result.minCenterHeight = std::min(result.minCenterHeight, position.z());
    result.maxHorizontalTravel
        = std::max(result.maxHorizontalTravel, std::abs(position.x()));
  }

  result.lastContacts
      = world->getConstraintSolver()->getLastCollisionResult().getNumContacts();

  if (exactCoulomb) {
    copyExactCoulombDiagnostics(exactSolver, residualTrace, result);
    copyCardHouseFailureDiagnostics(exactSolver, result);
  }

  return result;
}

void launchCardHouseProjectiles(const std::shared_ptr<simulation::World>& world)
{
  if (world->getSkeleton("fbf_projectile_0") != nullptr)
    return;

  for (std::size_t i = 0u; i < kCardHouseProjectileCount; ++i)
    world->addSkeleton(createCardHouseProjectile(i));
}

void launchMasonryArchProjectile(
    const std::shared_ptr<simulation::World>& world)
{
  if (world->getSkeleton("masonry_arch_projectile_0") != nullptr)
    return;

  for (std::size_t i = 0u; i < kArchProjectileCount; ++i)
    world->addSkeleton(createMasonryArchProjectile(i));
}

CardHouseRunResult runCardHouseFourLevelOneStep(
    bool exactCoulomb, std::size_t maxContacts, std::size_t maxContactsPerPair)
{
  auto world = createCardHouseFourLevelWorld(
      exactCoulomb,
      makeCardHouseFourLevelProbeSolverOptions(),
      maxContacts,
      maxContactsPerPair);
  const auto* exactSolver
      = exactCoulomb ? getExactCoulombSolver(world) : nullptr;
  ExactResidualTrace residualTrace;
  std::size_t previousExactSolves = 0u;

  CardHouseRunResult result;
  result.finiteState = true;
  result.minCardAxisUp = std::numeric_limits<double>::infinity();
  result.minCenterHeight = std::numeric_limits<double>::infinity();
  result.maxHorizontalTravel = 0.0;

  for (std::size_t i = 0u; i < world->getNumSkeletons(); ++i) {
    const auto skeleton = world->getSkeleton(i);
    if (skeleton != nullptr && isCardHouseSkeletonName(skeleton->getName())) {
      ++result.cardCount;
    }
  }

  const auto start = std::chrono::steady_clock::now();
  world->step();
  const auto stop = std::chrono::steady_clock::now();
  result.elapsedMs
      = std::chrono::duration<double, std::milli>(stop - start).count();
  sampleExactResidualTrace(exactSolver, previousExactSolves, residualTrace);

  for (std::size_t i = 0u; i < world->getNumSkeletons(); ++i) {
    const auto skeleton = world->getSkeleton(i);
    if (skeleton == nullptr || !isCardHouseSkeletonName(skeleton->getName())) {
      continue;
    }

    const auto* body = skeleton->getBodyNode(0);
    const Eigen::Isometry3d transform = body->getWorldTransform();
    const Eigen::Vector3d position = transform.translation();
    result.finiteState = result.finiteState && position.allFinite()
                         && body->getLinearVelocity().allFinite()
                         && body->getAngularVelocity().allFinite();
    result.minCardAxisUp = std::min(
        result.minCardAxisUp,
        transform.linear().col(2).normalized().dot(Eigen::Vector3d::UnitZ()));
    result.minCenterHeight = std::min(result.minCenterHeight, position.z());
    result.maxHorizontalTravel
        = std::max(result.maxHorizontalTravel, position.head<2>().norm());
  }

  result.lastContacts
      = world->getConstraintSolver()->getLastCollisionResult().getNumContacts();

  if (exactCoulomb) {
    copyExactCoulombDiagnostics(exactSolver, residualTrace, result);
    copyCardHouseFailureDiagnostics(exactSolver, result);
  }

  return result;
}

CardHouseRunResult runCardHouseFourLevelSettleProjectileSmoke()
{
  auto world = createCardHouseFourLevelWorld(
      true,
      makeCardHouseFourLevelProbeSolverOptions(),
      kCardHouseSettleProjectileSmokeContacts,
      kCardHouseReducedMaxContactsPerPair);
  const auto* exactSolver = getExactCoulombSolver(world);
  ExactResidualTrace residualTrace;
  std::size_t previousExactSolves = 0u;

  CardHouseRunResult result;
  result.finiteState = true;
  result.minCardAxisUp = std::numeric_limits<double>::infinity();
  result.minCenterHeight = std::numeric_limits<double>::infinity();
  result.maxHorizontalTravel = 0.0;

  const auto start = std::chrono::steady_clock::now();
  world->step();
  sampleExactResidualTrace(exactSolver, previousExactSolves, residualTrace);
  launchCardHouseProjectiles(world);
  world->step();
  sampleExactResidualTrace(exactSolver, previousExactSolves, residualTrace);
  const auto stop = std::chrono::steady_clock::now();
  result.elapsedMs
      = std::chrono::duration<double, std::milli>(stop - start).count();

  for (std::size_t i = 0u; i < world->getNumSkeletons(); ++i) {
    const auto skeleton = world->getSkeleton(i);
    if (skeleton == nullptr)
      continue;

    const auto* body = skeleton->getBodyNode(0);
    const Eigen::Isometry3d transform = body->getWorldTransform();
    const Eigen::Vector3d position = transform.translation();
    result.finiteState = result.finiteState && position.allFinite()
                         && body->getLinearVelocity().allFinite()
                         && body->getAngularVelocity().allFinite();

    if (isCardHouseSkeletonName(skeleton->getName())) {
      ++result.cardCount;
      result.minCardAxisUp = std::min(
          result.minCardAxisUp,
          transform.linear().col(2).normalized().dot(Eigen::Vector3d::UnitZ()));
      result.minCenterHeight = std::min(result.minCenterHeight, position.z());
      result.maxHorizontalTravel
          = std::max(result.maxHorizontalTravel, position.head<2>().norm());
    } else if (isCardHouseProjectileSkeletonName(skeleton->getName())) {
      ++result.projectileCount;
      result.maxProjectileSpeed = std::max(
          result.maxProjectileSpeed, body->getLinearVelocity().norm());
    }
  }

  result.lastContacts
      = world->getConstraintSolver()->getLastCollisionResult().getNumContacts();

  copyExactCoulombDiagnostics(exactSolver, residualTrace, result);
  copyCardHouseFailureDiagnostics(exactSolver, result);
  return result;
}

CardHouseRunResult runMasonryArchOneStep(
    std::size_t stoneCount,
    bool exactCoulomb,
    std::size_t maxContacts,
    std::size_t maxContactsPerPair)
{
  auto world = createMasonryArchWorld(
      "exact_coulomb_masonry_arch_" + std::to_string(stoneCount),
      stoneCount,
      exactCoulomb,
      maxContacts,
      maxContactsPerPair);
  const auto* exactSolver
      = exactCoulomb ? getExactCoulombSolver(world) : nullptr;
  ExactResidualTrace residualTrace;
  std::size_t previousExactSolves = 0u;

  CardHouseRunResult result;
  result.finiteState = true;
  result.minCardAxisUp = std::numeric_limits<double>::infinity();
  result.minCenterHeight = std::numeric_limits<double>::infinity();

  for (std::size_t i = 0u; i < world->getNumSkeletons(); ++i) {
    const auto skeleton = world->getSkeleton(i);
    if (skeleton != nullptr && isMasonryArchStoneName(skeleton->getName())) {
      ++result.cardCount;
    }
  }

  const auto start = std::chrono::steady_clock::now();
  world->step();
  const auto stop = std::chrono::steady_clock::now();
  result.elapsedMs
      = std::chrono::duration<double, std::milli>(stop - start).count();
  sampleExactResidualTrace(exactSolver, previousExactSolves, residualTrace);

  for (std::size_t i = 0u; i < world->getNumSkeletons(); ++i) {
    const auto skeleton = world->getSkeleton(i);
    if (skeleton == nullptr || !isMasonryArchStoneName(skeleton->getName())) {
      continue;
    }

    const auto* body = skeleton->getBodyNode(0);
    const Eigen::Isometry3d transform = body->getWorldTransform();
    const Eigen::Vector3d position = transform.translation();
    result.finiteState = result.finiteState && position.allFinite()
                         && body->getLinearVelocity().allFinite()
                         && body->getAngularVelocity().allFinite();
    result.minCardAxisUp = std::min(
        result.minCardAxisUp,
        transform.linear().col(2).normalized().dot(Eigen::Vector3d::UnitZ()));
    result.minCenterHeight = std::min(result.minCenterHeight, position.z());
    result.maxHorizontalTravel
        = std::max(result.maxHorizontalTravel, position.head<2>().norm());
  }

  result.lastContacts
      = world->getConstraintSolver()->getLastCollisionResult().getNumContacts();

  if (exactCoulomb) {
    copyExactCoulombDiagnostics(exactSolver, residualTrace, result);
    copyCardHouseFailureDiagnostics(exactSolver, result);
  }

  return result;
}

CardHouseRunResult runMasonryArch25OneStep(
    bool exactCoulomb,
    std::size_t maxContacts = kArchReducedMaxContacts,
    std::size_t maxContactsPerPair = kArchReducedMaxContactsPerPair)
{
  return runMasonryArchOneStep(
      kArchStoneCount, exactCoulomb, maxContacts, maxContactsPerPair);
}

CardHouseRunResult runMasonryArch25ProjectileScaffold()
{
  auto world = createMasonryArch25World(
      true, kArchReducedMaxContacts, kArchReducedMaxContactsPerPair);
  const auto* exactSolver = getExactCoulombSolver(world);
  ExactResidualTrace residualTrace;
  std::size_t previousExactSolves = 0u;

  CardHouseRunResult result;
  result.finiteState = true;
  result.minCardAxisUp = std::numeric_limits<double>::infinity();
  result.minCenterHeight = std::numeric_limits<double>::infinity();

  const auto start = std::chrono::steady_clock::now();
  world->step();
  sampleExactResidualTrace(exactSolver, previousExactSolves, residualTrace);
  launchMasonryArchProjectile(world);
  world->step();
  sampleExactResidualTrace(exactSolver, previousExactSolves, residualTrace);
  const auto stop = std::chrono::steady_clock::now();
  result.elapsedMs
      = std::chrono::duration<double, std::milli>(stop - start).count();

  for (std::size_t i = 0u; i < world->getNumSkeletons(); ++i) {
    const auto skeleton = world->getSkeleton(i);
    if (skeleton == nullptr)
      continue;

    const auto* body = skeleton->getBodyNode(0);
    const Eigen::Isometry3d transform = body->getWorldTransform();
    const Eigen::Vector3d position = transform.translation();
    result.finiteState = result.finiteState && position.allFinite()
                         && body->getLinearVelocity().allFinite()
                         && body->getAngularVelocity().allFinite();

    if (isMasonryArchStoneName(skeleton->getName())) {
      ++result.cardCount;
      result.minCardAxisUp = std::min(
          result.minCardAxisUp,
          transform.linear().col(2).normalized().dot(Eigen::Vector3d::UnitZ()));
      result.minCenterHeight = std::min(result.minCenterHeight, position.z());
      result.maxHorizontalTravel
          = std::max(result.maxHorizontalTravel, position.head<2>().norm());
    } else if (isMasonryArchProjectileSkeletonName(skeleton->getName())) {
      ++result.projectileCount;
      result.maxProjectileSpeed = std::max(
          result.maxProjectileSpeed, body->getLinearVelocity().norm());
    }
  }

  result.lastContacts
      = world->getConstraintSolver()->getLastCollisionResult().getNumContacts();
  copyExactCoulombDiagnostics(exactSolver, residualTrace, result);
  copyCardHouseFailureDiagnostics(exactSolver, result);
  return result;
}

CardHouseRunResult runMasonryArch101OneStep(
    bool exactCoulomb,
    std::size_t maxContacts = kArch101ReducedMaxContacts,
    std::size_t maxContactsPerPair = kArch101ReducedMaxContactsPerPair)
{
  return runMasonryArchOneStep(
      kArch101StoneCount, exactCoulomb, maxContacts, maxContactsPerPair);
}

double analyticalSlidingDisplacement(double frictionCoeff)
{
  const double theta = std::atan(kInclineTan);
  const double acceleration
      = kGravity * (std::sin(theta) - frictionCoeff * std::cos(theta));
  return 0.5 * acceleration * kInclineDuration * kInclineDuration;
}

std::pair<double, double> analyticalBackspinTerminalState()
{
  constexpr double kSphereInertiaOverMassRadiusSquared = 2.0 / 5.0;
  const double impulseOverMass
      = (kBackspinRadius * kBackspinAngularVelocity - kBackspinLinearVelocity)
        / (1.0 + 1.0 / kSphereInertiaOverMassRadiusSquared);
  const double linearVelocity = kBackspinLinearVelocity + impulseOverMass;
  const double angularVelocity
      = kBackspinAngularVelocity
        - impulseOverMass
              / (kSphereInertiaOverMassRadiusSquared * kBackspinRadius);
  return {linearVelocity, angularVelocity};
}

bool isTurntableCaptured(const TurntableRunResult& result)
{
  return result.radius < 1.25 && result.height > 0.05
         && result.lastContacts > 0u;
}

bool isTurntableEjected(const TurntableRunResult& result)
{
  return result.radius > 1.75 || result.height < 0.0;
}

bool isAuthorTurntableCaptured(const TurntableRunResult& result)
{
  const double tableTop = 2.0 * fbf_author_turntable::kSupportHalfHeight;
  return result.finiteState && !result.fellThroughSupport
         && result.radius < 1.25 && result.height > tableTop
         && result.lastContacts > 0u;
}

bool hasAuthorTurntableSourceExit(const TurntableRunResult& result)
{
  // The public author runner marks either radial departure or a 0.5 m
  // vertical drop as "left table" for metric bookkeeping. Keep that source
  // condition explicit, but do not let vertical tunneling count as the paper's
  // ejected outcome in DART.
  return result.sourceExitObserved;
}

bool isAuthorTurntableEjected(const TurntableRunResult& result)
{
  return result.finiteState && !result.fellThroughSupport
         && result.radius > fbf_author_turntable::kSupportRadius
                                + fbf_author_turntable::kRiderHalfSize
         && result.lastContacts == 0u;
}

bool isPainleveUpright(const PainleveRunResult& result)
{
  return result.uprightness > 0.85 && result.height > 0.35;
}

bool isPainleveTumbled(const PainleveRunResult& result)
{
  return result.uprightness < 0.55 || result.height < 0.35;
}

bool isCardHouseSkeletonName(const std::string& name)
{
  return name.rfind("card_house_", 0u) == 0u;
}

bool isCardHouseProjectileSkeletonName(const std::string& name)
{
  return name.rfind("fbf_projectile_", 0u) == 0u;
}

bool isMasonryArchStoneName(const std::string& name)
{
  return name.rfind("masonry_arch_stone_", 0u) == 0u;
}

bool isMasonryArchProjectileSkeletonName(const std::string& name)
{
  return name.rfind("masonry_arch_projectile_", 0u) == 0u;
}

} // namespace

//==============================================================================
TEST(ExactCoulombFbfPaperFixtures, StressFixtureOptInRequiresExplicitOne)
{
  EXPECT_FALSE(isFbfPaperStressTestValueEnabled(nullptr));
  EXPECT_FALSE(isFbfPaperStressTestValueEnabled(""));
  EXPECT_FALSE(isFbfPaperStressTestValueEnabled("0"));
  EXPECT_FALSE(isFbfPaperStressTestValueEnabled("true"));
  EXPECT_TRUE(isFbfPaperStressTestValueEnabled("1"));
}

//==============================================================================
TEST(
    ExactCoulombFbfPaperFixtures,
    SelectsScenarioSpecificCollisionFrontendAndManifoldMode)
{
  using ManifoldMode = collision::NativeCollisionDetector::ContactManifoldMode;
  const auto expectNativeMode
      = [](const std::shared_ptr<simulation::World>& world,
           ManifoldMode expected) {
          const auto detector
              = std::dynamic_pointer_cast<collision::NativeCollisionDetector>(
                  world->getConstraintSolver()->getCollisionDetector());
          ASSERT_NE(nullptr, detector);
          EXPECT_EQ(expected, detector->getContactManifoldMode());
        };
  const auto expectDartFrontend
      = [](const std::shared_ptr<simulation::World>& world) {
          const auto detector
              = std::dynamic_pointer_cast<collision::DARTCollisionDetector>(
                  world->getConstraintSolver()->getCollisionDetector());
          ASSERT_NE(nullptr, detector);
        };

  for (const bool exactCoulomb : {false, true}) {
    expectNativeMode(
        createInclineWorld(0.5, exactCoulomb), ManifoldMode::FourPointPlanar);
    expectNativeMode(
        createTurntableWorld(0.5, exactCoulomb), ManifoldMode::FourPointPlanar);
  }

  expectDartFrontend(createPainleveWorld(0.5, true));
  expectDartFrontend(createBackspinWorld(true));
  expectDartFrontend(createCardHouseFourLevelWorld(true));
  expectDartFrontend(createMasonryArch25World(true));
}

//==============================================================================
TEST(ExactCoulombFbfPaperFixtures, ProjectileGeometryMatchesPublishedVisuals)
{
  const Eigen::Vector3d cardSize(
      kCardHouseThickness, kCardHouseWidth, kCardHouseHeight);
  const auto card = createCardPlate(
      "density_probe_card", Eigen::Isometry3d::Identity(), kCardHouseFriction);
  ASSERT_NE(card, nullptr);
  const auto* cardBody = card->getBodyNode(0);
  ASSERT_NE(cardBody, nullptr);
  const double cardVolume = cardSize.x() * cardSize.y() * cardSize.z();
  EXPECT_NEAR(
      cardBody->getInertia().getMass() / cardVolume, kCardHouseDensity, 1e-9);
  EXPECT_TRUE(cardBody->getInertia().getMoment().isApprox(
      dynamics::BoxShape::computeInertia(
          cardSize, kCardHouseDensity * cardVolume),
      1e-12));

  double previousCardX = -std::numeric_limits<double>::infinity();
  for (std::size_t i = 0u; i < kCardHouseProjectileCount; ++i) {
    const auto projectile = createCardHouseProjectile(i);
    ASSERT_NE(projectile, nullptr);
    const auto* body = projectile->getBodyNode(0);
    ASSERT_NE(body, nullptr);
    const auto* shapeNode = body->getShapeNode(0);
    ASSERT_NE(shapeNode, nullptr);
    const auto shape = shapeNode->getShape();
    ASSERT_NE(shape, nullptr);
    EXPECT_TRUE(shape->is<dynamics::BoxShape>());
    const auto box = std::static_pointer_cast<const dynamics::BoxShape>(shape);
    EXPECT_TRUE(box->getSize().isApprox(
        Eigen::Vector3d::Constant(kCardHouseProjectileEdgeLength), 1e-12));
    EXPECT_NEAR(body->getInertia().getMass(), kCardHouseProjectileMass, 1e-12);
    EXPECT_NEAR(
        body->getInertia().getMass()
            / std::pow(kCardHouseProjectileEdgeLength, 3),
        kCardHouseDensity,
        1e-9);
    const Eigen::Vector3d position = body->getWorldTransform().translation();
    EXPECT_GT(position.x(), previousCardX);
    EXPECT_NEAR(position.y(), 0.0, 1e-12);
    EXPECT_NEAR(position.z(), kCardHouseProjectileDropHeight, 1e-12);
    EXPECT_NEAR(body->getLinearVelocity().x(), 0.0, 1e-12);
    EXPECT_NEAR(body->getLinearVelocity().y(), 0.0, 1e-12);
    EXPECT_LT(body->getLinearVelocity().z(), 0.0);
    previousCardX = position.x();
  }

  double previousArchX = -std::numeric_limits<double>::infinity();
  for (std::size_t i = 0u; i < kArchProjectileCount; ++i) {
    const auto projectile = createMasonryArchProjectile(i);
    ASSERT_NE(projectile, nullptr);
    const auto* body = projectile->getBodyNode(0);
    ASSERT_NE(body, nullptr);
    const auto* shapeNode = body->getShapeNode(0);
    ASSERT_NE(shapeNode, nullptr);
    const auto shape = shapeNode->getShape();
    ASSERT_NE(shape, nullptr);
    EXPECT_TRUE(shape->is<dynamics::BoxShape>());
    const auto box = std::static_pointer_cast<const dynamics::BoxShape>(shape);
    EXPECT_TRUE(box->getSize().isApprox(
        Eigen::Vector3d::Constant(kArchProjectileEdgeLength), 1e-12));
    EXPECT_NEAR(body->getInertia().getMass(), kArchProjectileMass, 1e-12);
    EXPECT_NEAR(
        body->getInertia().getMass() / std::pow(kArchProjectileEdgeLength, 3),
        kArchDensity,
        1e-9);
    const Eigen::Vector3d position = body->getWorldTransform().translation();
    EXPECT_GT(position.x(), previousArchX);
    EXPECT_NEAR(position.y(), 0.0, 1e-12);
    EXPECT_NEAR(position.z(), kArchProjectileDropHeight, 1e-12);
    EXPECT_NEAR(body->getLinearVelocity().x(), 0.0, 1e-12);
    EXPECT_NEAR(body->getLinearVelocity().y(), 0.0, 1e-12);
    EXPECT_LT(body->getLinearVelocity().z(), 0.0);
    previousArchX = position.x();
  }
}

//==============================================================================
TEST(ExactCoulombFbfPaperFixtures, InclineCubeThresholdStickSlide)
{
  const auto exactStick = runInclineCube(0.5, true);
  const auto exactSlide = runInclineCube(0.4, true);
  const auto boxedStick = runInclineCube(0.5, false);
  const auto boxedSlide = runInclineCube(0.4, false);

  EXPECT_TRUE(exactStick.exactSuccess);
  EXPECT_TRUE(exactSlide.exactSuccess);
  EXPECT_EQ(exactStick.boxedFallbacks, 0u);
  EXPECT_EQ(exactSlide.boxedFallbacks, 0u);
  EXPECT_GT(exactStick.exactSolves, 0u);
  EXPECT_GT(exactSlide.exactSolves, 0u);
  EXPECT_GT(exactStick.lastContacts, 0u);
  EXPECT_GT(exactSlide.lastContacts, 0u);
  EXPECT_EQ(exactStick.contactSteps, exactStick.trajectorySteps);
  EXPECT_EQ(exactSlide.contactSteps, exactSlide.trajectorySteps);
  EXPECT_TRUE(std::isfinite(exactStick.maxPenetration));
  EXPECT_TRUE(std::isfinite(exactSlide.maxPenetration));
  EXPECT_LE(exactStick.maxPenetration, 0.02);
  EXPECT_LE(exactSlide.maxPenetration, 0.02);
  EXPECT_TRUE(std::isfinite(exactStick.residual));
  EXPECT_TRUE(std::isfinite(exactSlide.residual));
  EXPECT_LE(exactStick.residual, 1e-6);
  EXPECT_LE(exactSlide.residual, 1e-6);
  expectConvergedResidualTrace(exactStick.residualTrace, 1e-6);
  expectConvergedResidualTrace(exactSlide.residualTrace, 1e-6);

  EXPECT_TRUE(std::isfinite(boxedStick.displacement));
  EXPECT_TRUE(std::isfinite(boxedSlide.displacement));
  EXPECT_GT(boxedStick.lastContacts, 0u);
  EXPECT_GT(boxedSlide.lastContacts, 0u);

  EXPECT_NEAR(exactStick.displacement, 0.0, 2e-2)
      << "speed=" << exactStick.speed;
  EXPECT_NEAR(exactSlide.displacement, analyticalSlidingDisplacement(0.4), 0.2)
      << "speed=" << exactSlide.speed;
  EXPECT_GT(exactSlide.displacement, exactStick.displacement + 0.5);
  EXPECT_GT(boxedSlide.displacement, boxedStick.displacement + 0.5);
}

//==============================================================================
TEST(ExactCoulombFbfPaperFixtures, BackspinSphereReachesAnalyticalRollingState)
{
  const auto exact = runBackspinSphere(true);
  const auto boxed = runBackspinSphere(false);
  const auto [expectedLinearVelocity, expectedAngularVelocity]
      = analyticalBackspinTerminalState();

  EXPECT_TRUE(exact.exactSuccess);
  EXPECT_EQ(exact.boxedFallbacks, 0u);
  EXPECT_GT(exact.exactSolves, 0u);
  EXPECT_GT(exact.lastContacts, 0u);
  EXPECT_TRUE(std::isfinite(exact.residual));
  EXPECT_LE(exact.residual, 1e-6);
  expectConvergedResidualTrace(exact.residualTrace, 1e-6);

  EXPECT_NEAR(exact.linearVelocity, expectedLinearVelocity, 0.5);
  EXPECT_NEAR(exact.angularVelocity, expectedAngularVelocity, 2.0);
  EXPECT_NEAR(exact.slipVelocity, 0.0, 0.5);
  EXPECT_NEAR(exact.height, kBackspinRadius, 3e-2);

  EXPECT_TRUE(std::isfinite(boxed.linearVelocity));
  EXPECT_TRUE(std::isfinite(boxed.angularVelocity));
  EXPECT_GT(boxed.lastContacts, 0u);
}

//==============================================================================
TEST(ExactCoulombFbfPaperFixtures, PainleveProxySlideTumbleThreshold)
{
  const auto exactSlide = runPainleveBox(0.5, true);
  const auto exactTumble = runPainleveBox(0.55, true);
  const auto boxedSlide = runPainleveBox(0.5, false);
  const auto boxedTumble = runPainleveBox(0.55, false);

  EXPECT_TRUE(exactSlide.exactSuccess);
  EXPECT_TRUE(exactTumble.exactSuccess);
  EXPECT_EQ(exactSlide.boxedFallbacks, 0u);
  EXPECT_EQ(exactTumble.boxedFallbacks, 0u);
  EXPECT_GT(exactSlide.exactSolves, 0u);
  EXPECT_GT(exactTumble.exactSolves, 0u);
  EXPECT_TRUE(std::isfinite(exactSlide.residual));
  EXPECT_TRUE(std::isfinite(exactTumble.residual));
  EXPECT_LE(exactSlide.residual, 1e-6);
  EXPECT_LE(exactTumble.residual, 1e-6);
  expectConvergedResidualTrace(exactSlide.residualTrace, 1e-6);
  expectConvergedResidualTrace(exactTumble.residualTrace, 1e-6);

  EXPECT_TRUE(isPainleveUpright(exactSlide))
      << "travel=" << exactSlide.travel << " speed=" << exactSlide.speed
      << " uprightness=" << exactSlide.uprightness
      << " height=" << exactSlide.height
      << " contacts=" << exactSlide.lastContacts;
  EXPECT_FALSE(std::isfinite(exactSlide.tumbleTravel));
  EXPECT_TRUE(isPainleveTumbled(exactTumble))
      << "travel=" << exactTumble.travel << " speed=" << exactTumble.speed
      << " uprightness=" << exactTumble.uprightness
      << " height=" << exactTumble.height
      << " contacts=" << exactTumble.lastContacts;
  EXPECT_TRUE(std::isfinite(exactTumble.tumbleTravel));
  EXPECT_LT(exactTumble.tumbleTravel, exactSlide.travel)
      << "tumbleTime=" << exactTumble.tumbleTime
      << " finalTravel=" << exactTumble.travel;

  EXPECT_TRUE(std::isfinite(boxedSlide.travel));
  EXPECT_TRUE(std::isfinite(boxedTumble.travel));
  EXPECT_TRUE(std::isfinite(boxedSlide.uprightness));
  EXPECT_TRUE(std::isfinite(boxedTumble.uprightness));
}

//==============================================================================
TEST(ExactCoulombFbfPaperFixtures, TurntableCaptureEjectionGrid)
{
  struct Cell
  {
    double frictionCoeff;
    double angularVelocity;
    bool captured;
  };

  constexpr std::array<Cell, 4> cells{{
      {0.2, 2.0, false},
      {0.2, 5.0, false},
      {0.5, 2.0, true},
      {0.5, 5.0, false},
  }};

  for (const auto& cell : cells) {
    SCOPED_TRACE(
        "mu=" + std::to_string(cell.frictionCoeff)
        + " omega=" + std::to_string(cell.angularVelocity));
    const auto exact
        = runTurntable(cell.frictionCoeff, cell.angularVelocity, true);
    const auto boxed
        = runTurntable(cell.frictionCoeff, cell.angularVelocity, false);

    EXPECT_EQ(exact.boxedFallbacks, 0u);
    EXPECT_GT(exact.exactSolves, 0u);
    EXPECT_TRUE(std::isfinite(exact.residual));
    EXPECT_LE(exact.residual, 1e-6);
    expectConvergedResidualTrace(exact.residualTrace, 1e-6);

    if (cell.captured) {
      EXPECT_TRUE(exact.exactSuccess);
      EXPECT_TRUE(isTurntableCaptured(exact))
          << "radius=" << exact.radius
          << " radialVelocity=" << exact.radialVelocity
          << " height=" << exact.height << " contacts=" << exact.lastContacts;
      EXPECT_TRUE(std::isfinite(exact.radialVelocity));
      EXPECT_TRUE(std::isfinite(exact.tangentialCoRotationError));
      EXPECT_LE(std::abs(exact.radialVelocity), 0.2);
      EXPECT_LE(exact.tangentialCoRotationError, 0.05)
          << "tangentialVelocity=" << exact.tangentialVelocity
          << " expected=" << cell.angularVelocity * exact.radius;
    } else {
      EXPECT_TRUE(isTurntableEjected(exact))
          << "radius=" << exact.radius
          << " radialVelocity=" << exact.radialVelocity
          << " height=" << exact.height << " contacts=" << exact.lastContacts;
    }

    EXPECT_TRUE(std::isfinite(boxed.radius));
    EXPECT_TRUE(std::isfinite(boxed.height));
  }
}

//==============================================================================
TEST(ExactCoulombFbfPaperFixtures, AuthorCardHouseSourceConstructionIsPinned)
{
  using namespace fbf_author_card_house;

  EXPECT_STREQ(kAuthorCommit, "b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0");
  EXPECT_STREQ(kAuthorTree, "ffcdafb61adeda2239c8366d054b548b50d26685e");
  EXPECT_STREQ(
      kAuthorCardHouseBlob, "35f33651bc9674a259071ac723e47755504152db");
  EXPECT_STREQ(
      kAuthorCardHouseRunSha256,
      "18c58c85eaad865aeef480b46e880a52088f266b79c90226f624637221ee36f8");
  EXPECT_STREQ(
      kAuthorConfigSha256,
      "88f3f9ffd758eccce8496f7897192587a05907109e313c7a86bcf8f9de8cc248");
  EXPECT_STREQ(
      kAuthorSolverSha256,
      "8ec32aa20bf8d6c1173ed6c7f3735e2926fbb4b5059ee2236e26ad27eb22f941");

  EXPECT_EQ(leaningCardCount(kDefaultLevelCount), 30u);
  EXPECT_EQ(bridgeCardCount(kDefaultLevelCount), 10u);
  EXPECT_EQ(cardCount(kDefaultLevelCount), 40u);
  const auto cards = makeCardSpecs(kDefaultLevelCount);
  ASSERT_EQ(cards.size(), 40u);
  EXPECT_EQ(
      std::count_if(
          cards.begin(),
          cards.end(),
          [](const auto& spec) { return spec.kind == CardKind::Bridge; }),
      10);
  EXPECT_EQ(
      std::count_if(
          cards.begin(),
          cards.end(),
          [](const auto& spec) { return spec.kind != CardKind::Bridge; }),
      30);

  const auto findCard = [&cards](const std::string& name) -> const CardSpec& {
    const auto found
        = std::find_if(cards.begin(), cards.end(), [&name](const auto& spec) {
            return spec.name == name;
          });
    if (found == cards.end())
      throw std::runtime_error("missing test card " + name);
    return *found;
  };
  const auto& firstLeft = findCard("tent_left_L0_T0");
  EXPECT_TRUE(firstLeft.size.isApprox(Eigen::Vector3d(0.04, 1.25, 2.5)));
  EXPECT_NEAR(firstLeft.transform.translation().x(), -4.95, 1e-14);
  EXPECT_NEAR(firstLeft.transform.translation().z(), 0.5 * kTentHeight, 1e-14);
  EXPECT_TRUE(
      firstLeft.transform.linear().isApprox(rotationAboutY(25.0), 1e-15));
  const auto& firstRight = findCard("tent_right_L0_T0");
  EXPECT_NEAR(firstRight.transform.translation().x(), -3.85, 1e-14);
  EXPECT_TRUE(
      firstRight.transform.linear().isApprox(rotationAboutY(-25.0), 1e-15));
  const auto& firstBridge = findCard("bridge_L1_T0");
  EXPECT_TRUE(firstBridge.size.isApprox(Eigen::Vector3d(2.5, 1.25, 0.04)));
  EXPECT_NEAR(firstBridge.transform.translation().x(), -3.3, 1e-14);
  EXPECT_NEAR(firstBridge.transform.translation().z(), kTentHeight, 1e-14);
  EXPECT_TRUE(
      firstBridge.transform.linear().isApprox(rotationAboutY(-1.0), 1e-15));
  EXPECT_DOUBLE_EQ(
      kCardDensity * 0.04 * 1.25 * 2.5, fbf_author_card_house::kCardMass);

  const auto cubes = makeCubeSpecs(kDefaultLevelCount);
  ASSERT_EQ(cubes.size(), 4u);
  constexpr std::array<double, 4> expectedX{{-1.8, -0.6, 0.6, 1.8}};
  for (std::size_t index = 0u; index < cubes.size(); ++index) {
    EXPECT_TRUE(cubes[index].size.isApprox(Eigen::Vector3d::Constant(0.8)));
    EXPECT_NEAR(
        cubes[index].transform.translation().x(), expectedX[index], 1e-14);
    EXPECT_NEAR(
        cubes[index].transform.translation().z(), kCubeInitialHeight, 1e-14);
  }
  EXPECT_DOUBLE_EQ(kCubeDensity * 0.8 * 0.8 * 0.8, kCubeMass);
  EXPECT_DOUBLE_EQ(kDisplayTimeStep, 1.0 / 60.0);
  EXPECT_EQ(kSubstepsPerFrame, 4u);
  EXPECT_DOUBLE_EQ(kRuntimeTimeStep, 1.0 / 240.0);
  EXPECT_EQ(kReleaseFrame, 400u);
  EXPECT_EQ(kReleaseSubstep, 1600u);
  EXPECT_EQ(kTotalFrames, 800u);
  EXPECT_EQ(kTotalSubsteps, 3200u);
  EXPECT_FALSE(kTrajectoryEquivalence);
  EXPECT_FALSE(kSolverEquivalence);
  EXPECT_FALSE(kPhysicalOutcomeEquivalence);
  EXPECT_FALSE(kFig06Parity);
  EXPECT_FALSE(kVideo06Parity);
  EXPECT_FALSE(kTimingComparability);
}

//==============================================================================
TEST(ExactCoulombFbfPaperFixtures, AuthorCardHouseContractIsConstructionOnly)
{
  using namespace fbf_author_card_house;
  const auto world = createAuthorCardHouseConstructionWorld();
  EXPECT_DOUBLE_EQ(world->getTime(), 0.0);

  const auto contract = inspectConfigurationContract(
      world, kDefaultLevelCount, "integration_fixture", "not_applicable");
  EXPECT_EQ(contract.levelCount, 5u);
  EXPECT_DOUBLE_EQ(contract.timeStep, kRuntimeTimeStep);
  EXPECT_TRUE(contract.gravity.isApprox(Eigen::Vector3d(0.0, 0.0, -9.81)));
  EXPECT_EQ(contract.simulationThreads, 1u);
  EXPECT_FALSE(contract.deactivationEnabled);
  EXPECT_EQ(contract.collisionDetector, "native");
  EXPECT_EQ(contract.contactManifold, "four_point_planar");
  EXPECT_EQ(contract.maxContacts, kSourceMaxContacts);
  EXPECT_EQ(contract.maxContactsPerPair, 4u);
  EXPECT_FALSE(contract.splitImpulseEnabled);
  EXPECT_EQ(
      contract.solverOptions.maxOuterIterations, kSourceMaxOuterIterations);
  EXPECT_DOUBLE_EQ(contract.solverOptions.tolerance, kSourceOuterTolerance);
  EXPECT_EQ(
      contract.solverOptions.innerMaxSweeps, kSourceInnerGaussSeidelSweeps);
  EXPECT_DOUBLE_EQ(
      contract.solverOptions.innerTolerance, kSourceInnerTolerance);
  EXPECT_TRUE(contract.solverOptions.enableWarmStart);
  EXPECT_TRUE(contract.solverOptions.runFixedInnerSweeps);
  EXPECT_FALSE(contract.solverOptions.fallbackToBoxedLcp);
  EXPECT_FALSE(contract.solverOptions.enableProjectedGradientRetry);
  EXPECT_FALSE(contract.solverOptions.enableDenseResidualPolish);
  EXPECT_EQ(contract.groundShape, "plane");
  EXPECT_DOUBLE_EQ(contract.groundFriction, kFriction);
  EXPECT_FALSE(contract.groundMobile);

  const auto expectedCards = makeCardSpecs(kDefaultLevelCount);
  ASSERT_EQ(contract.cards.size(), expectedCards.size());
  for (std::size_t index = 0u; index < expectedCards.size(); ++index) {
    const auto& actual = contract.cards[index];
    const auto& expected = expectedCards[index];
    EXPECT_EQ(actual.name, expected.name);
    EXPECT_TRUE(actual.size.isApprox(expected.size, 1e-15));
    EXPECT_TRUE(
        actual.pose.matrix().isApprox(expected.transform.matrix(), 1e-14));
    EXPECT_DOUBLE_EQ(actual.mass, kCardMass);
    EXPECT_DOUBLE_EQ(actual.friction, kFriction);
    EXPECT_TRUE(actual.mobile);
    EXPECT_TRUE(actual.linearVelocity.isZero(0.0));
    EXPECT_TRUE(actual.angularVelocity.isZero(0.0));
  }

  const auto expectedCubes = makeCubeSpecs(kDefaultLevelCount);
  ASSERT_EQ(contract.cubes.size(), expectedCubes.size());
  for (std::size_t index = 0u; index < expectedCubes.size(); ++index) {
    const auto& actual = contract.cubes[index];
    const auto& expected = expectedCubes[index];
    EXPECT_EQ(actual.name, expected.name);
    EXPECT_TRUE(actual.size.isApprox(expected.size, 1e-15));
    EXPECT_TRUE(
        actual.pose.matrix().isApprox(expected.transform.matrix(), 1e-14));
    EXPECT_DOUBLE_EQ(actual.mass, kCubeMass);
    EXPECT_DOUBLE_EQ(actual.friction, kFriction);
    EXPECT_FALSE(actual.mobile);
    EXPECT_TRUE(actual.linearVelocity.isZero(0.0));
    EXPECT_TRUE(actual.angularVelocity.isZero(0.0));
  }

  const std::string json = configurationContractJson(contract);
  EXPECT_NE(
      json.find("\"schema_version\":\"dart.fbf_author_card_house_"
                "configuration_contract/v1\""),
      std::string::npos);
  EXPECT_NE(json.find("\"dynamics_executed\":false"), std::string::npos);
  EXPECT_NE(json.find("\"configuration_port_valid\":true"), std::string::npos);
  EXPECT_NE(json.find("\"trajectory_valid\":false"), std::string::npos);
  EXPECT_NE(json.find("\"solver_valid\":false"), std::string::npos);
  EXPECT_NE(json.find("\"physical_outcome_valid\":false"), std::string::npos);
  EXPECT_NE(json.find("\"trajectory_equivalence\":false"), std::string::npos);
  EXPECT_NE(json.find("\"solver_equivalence\":false"), std::string::npos);
  EXPECT_NE(
      json.find("\"physical_outcome_equivalence\":false"), std::string::npos);
  EXPECT_NE(json.find("\"fig06_parity\":false"), std::string::npos);
  EXPECT_NE(json.find("\"video06_parity\":false"), std::string::npos);
  EXPECT_NE(json.find("\"timing_comparability\":false"), std::string::npos);
  EXPECT_NE(json.find("\"paper_timing_valid\":false"), std::string::npos);
  EXPECT_NE(
      json.find("\"source_contact_gap_semantics_implemented\":false"),
      std::string::npos);
  EXPECT_NE(
      json.find("\"source_solver_backend_semantics_implemented\":false"),
      std::string::npos);
  EXPECT_DOUBLE_EQ(world->getTime(), 0.0);
}

//==============================================================================
TEST(ExactCoulombFbfPaperFixtures, AuthorTurntableSourceContractIsPinned)
{
  const auto& scenario = fbf_author_turntable::kScenarios[0];
  constexpr double friction = fbf_author_turntable::kScenarios[0].friction;
  constexpr double cubeSide = 2.0 * fbf_author_turntable::kRiderHalfSize;
  constexpr double cubeMass
      = fbf_author_turntable::kRiderDensity * cubeSide * cubeSide * cubeSide;
  auto world = createAuthorTurntableWorld(friction, true);

  const auto contract = fbf_author_turntable::inspectPhysicsContract(
      world, scenario, "dart_best", "integration_fixture", "not_applicable");
  EXPECT_EQ(contract.scenario, &scenario);
  EXPECT_EQ(contract.collisionDetector, "native");
  EXPECT_EQ(contract.contactManifold, "four_point_planar");
  EXPECT_EQ(contract.maxContacts, fbf_author_turntable::kMaxContacts);
  EXPECT_EQ(
      contract.maxContactsPerPair, fbf_author_turntable::kMaxContactsPerPair);
  EXPECT_EQ(
      contract.maxOuterIterations, fbf_author_turntable::kMaxOuterIterations);
  EXPECT_DOUBLE_EQ(contract.tolerance, fbf_author_turntable::kTolerance);

  EXPECT_NEAR(world->getTimeStep(), 1.0 / 60.0, 1e-15);
  const auto detector
      = std::dynamic_pointer_cast<collision::NativeCollisionDetector>(
          world->getConstraintSolver()->getCollisionDetector());
  ASSERT_NE(detector, nullptr);
  EXPECT_EQ(
      detector->getContactManifoldMode(),
      collision::NativeCollisionDetector::ContactManifoldMode::FourPointPlanar);

  const auto support = world->getSkeleton("turntable");
  ASSERT_NE(support, nullptr);
  EXPECT_FALSE(support->isMobile());
  const auto* supportBody = support->getBodyNode(0);
  ASSERT_NE(supportBody, nullptr);
  const auto* supportNode
      = supportBody->getShapeNodeWith<dynamics::CollisionAspect>(0u);
  ASSERT_NE(supportNode, nullptr);
  const auto supportShape = supportNode->getShape();
  ASSERT_NE(supportShape, nullptr);
  ASSERT_TRUE(supportShape->is<dynamics::CylinderShape>());
  const auto cylinder
      = std::static_pointer_cast<const dynamics::CylinderShape>(supportShape);
  EXPECT_NEAR(
      cylinder->getRadius(), fbf_author_turntable::kSupportRadius, 1e-15);
  EXPECT_NEAR(
      cylinder->getHeight(),
      2.0 * fbf_author_turntable::kSupportHalfHeight,
      1e-15);
  EXPECT_NEAR(
      supportBody->getWorldTransform().translation().z(),
      fbf_author_turntable::kSupportHalfHeight,
      1e-15);
  EXPECT_NEAR(
      supportNode->getDynamicsAspect()->getFrictionCoeff(), friction, 1e-15);

  const auto rider = world->getSkeleton("turntable_rider");
  ASSERT_NE(rider, nullptr);
  const auto* riderBody = rider->getBodyNode(0);
  ASSERT_NE(riderBody, nullptr);
  const auto* riderNode
      = riderBody->getShapeNodeWith<dynamics::CollisionAspect>(0u);
  ASSERT_NE(riderNode, nullptr);
  const auto riderShape = riderNode->getShape();
  ASSERT_NE(riderShape, nullptr);
  ASSERT_TRUE(riderShape->is<dynamics::BoxShape>());
  const auto box
      = std::static_pointer_cast<const dynamics::BoxShape>(riderShape);
  EXPECT_TRUE(
      box->getSize().isApprox(Eigen::Vector3d::Constant(cubeSide), 1e-15));
  EXPECT_NEAR(riderBody->getInertia().getMass(), cubeMass, 1e-12);
  EXPECT_NEAR(cubeMass, 13.5, 1e-12);
  EXPECT_NEAR(
      riderNode->getDynamicsAspect()->getFrictionCoeff(), friction, 1e-15);
  const Eigen::Vector3d position = riderBody->getWorldTransform().translation();
  EXPECT_NEAR(position.x(), 1.0, 1e-15);
  EXPECT_NEAR(position.y(), 0.0, 1e-15);
  EXPECT_NEAR(position.z(), 0.455, 1e-15);
}

//==============================================================================
TEST(ExactCoulombFbfPaperFixtures, AuthorTurntableScheduleIsContinuous)
{
  constexpr double omega = 5.0;
  constexpr double epsilon = 1e-9;
  const double settle = fbf_author_turntable::kSettleDuration;
  const double rampEnd = settle + fbf_author_turntable::kRampDuration;

  EXPECT_DOUBLE_EQ(fbf_author_turntable::angularVelocity(settle, omega), 0.0);
  EXPECT_DOUBLE_EQ(fbf_author_turntable::integratedYaw(settle, omega), 0.0);
  EXPECT_NEAR(
      fbf_author_turntable::angularVelocity(settle + epsilon, omega),
      0.0,
      1e-14);
  EXPECT_NEAR(
      fbf_author_turntable::integratedYaw(settle + epsilon, omega), 0.0, 1e-20);

  EXPECT_NEAR(
      fbf_author_turntable::angularVelocity(rampEnd - epsilon, omega),
      omega,
      1e-14);
  EXPECT_DOUBLE_EQ(
      fbf_author_turntable::angularVelocity(rampEnd, omega), omega);
  const double expectedRampYaw
      = 0.5 * omega * fbf_author_turntable::kRampDuration;
  EXPECT_NEAR(
      fbf_author_turntable::integratedYaw(rampEnd - epsilon, omega),
      expectedRampYaw,
      omega * epsilon * 1.01);
  EXPECT_DOUBLE_EQ(
      fbf_author_turntable::integratedYaw(rampEnd, omega), expectedRampYaw);
  EXPECT_DOUBLE_EQ(
      fbf_author_turntable::integratedYaw(rampEnd + 0.25, omega),
      expectedRampYaw + 0.25 * omega);
}

//==============================================================================
TEST(ExactCoulombFbfPaperFixtures, AuthorTurntableCaptureEjectionGrid)
{
  struct Cell
  {
    double frictionCoeff;
    double angularVelocity;
    bool captured;
  };

  constexpr std::array<Cell, 4> cells{{
      {0.2, 2.0, false},
      {0.2, 5.0, false},
      {0.5, 2.0, true},
      {0.5, 5.0, false},
  }};

  for (const auto& cell : cells) {
    SCOPED_TRACE(
        "author b3f3c5c mu=" + std::to_string(cell.frictionCoeff)
        + " omega=" + std::to_string(cell.angularVelocity));
    const auto exact
        = runAuthorTurntable(cell.frictionCoeff, cell.angularVelocity, true);

    EXPECT_TRUE(exact.exactSuccess);
    EXPECT_EQ(exact.boxedFallbacks, 0u);
    EXPECT_EQ(exact.exactFailures, 0u);
    EXPECT_GT(exact.exactSolves, 0u);
    EXPECT_TRUE(std::isfinite(exact.residual));
    EXPECT_LE(exact.residual, 1e-6);
    expectConvergedResidualTrace(exact.residualTrace, 1e-6);
    EXPECT_TRUE(exact.finiteState);
    EXPECT_FALSE(exact.fellThroughSupport)
        << "minimum fully-supported height=" << exact.minFullySupportedHeight
        << " final radius=" << exact.radius << " final height=" << exact.height;
    EXPECT_TRUE(std::isfinite(exact.minFullySupportedHeight));
    EXPECT_EQ(hasAuthorTurntableSourceExit(exact), !cell.captured);

    if (cell.captured) {
      EXPECT_TRUE(isAuthorTurntableCaptured(exact))
          << "radius=" << exact.radius
          << " radialVelocity=" << exact.radialVelocity
          << " height=" << exact.height << " contacts=" << exact.lastContacts
          << " maxRadius=" << exact.maxRadius;
    } else {
      EXPECT_TRUE(isAuthorTurntableEjected(exact))
          << "radius=" << exact.radius
          << " radialVelocity=" << exact.radialVelocity
          << " height=" << exact.height << " contacts=" << exact.lastContacts
          << " maxRadius=" << exact.maxRadius
          << " fellThroughSupport=" << exact.fellThroughSupport;
    }
  }
}

//==============================================================================
TEST(
    ExactCoulombFbfPaperFixtures,
    AuthorTurntableVerticalExitDoesNotCountAsEjection)
{
  TurntableRunResult fellThrough;
  fellThrough.radius = 1.0;
  fellThrough.height = -1.0;
  fellThrough.lastContacts = 0u;
  fellThrough.sourceExitObserved = true;
  fellThrough.fellThroughSupport = true;
  EXPECT_TRUE(hasAuthorTurntableSourceExit(fellThrough));
  EXPECT_FALSE(isAuthorTurntableEjected(fellThrough));

  auto radialExit = fellThrough;
  radialExit.radius = fbf_author_turntable::kSupportRadius
                      + fbf_author_turntable::kRiderHalfSize + 0.01;
  radialExit.fellThroughSupport = false;
  EXPECT_TRUE(isAuthorTurntableEjected(radialExit));

  radialExit.lastContacts = 1u;
  EXPECT_FALSE(isAuthorTurntableEjected(radialExit));
}

//==============================================================================
TEST(
    ExactCoulombFbfPaperFixtures,
    TurntableContactLossAloneDoesNotEstablishEjection)
{
  TurntableRunResult contactLost;
  contactLost.radius = 1.0;
  contactLost.height = 0.1;
  contactLost.lastContacts = 0u;
  EXPECT_FALSE(isTurntableEjected(contactLost));

  auto outward = contactLost;
  outward.radius = 1.76;
  EXPECT_TRUE(isTurntableEjected(outward));

  auto fallen = contactLost;
  fallen.height = -0.01;
  EXPECT_TRUE(isTurntableEjected(fallen));
}

//==============================================================================
TEST(ExactCoulombFbfPaperFixtures, CardHouseAFramePrecursorStands)
{
  const auto exact = runCardHousePrecursor(true);
  const auto boxed = runCardHousePrecursor(false);

  EXPECT_TRUE(exact.exactSuccess);
  EXPECT_EQ(exact.boxedFallbacks, 0u);
  EXPECT_GT(exact.exactSolves, 0u);
  EXPECT_TRUE(std::isfinite(exact.residual));
  EXPECT_LE(exact.residual, 1e-6)
      << "primal=" << exact.primalResidual << " dual=" << exact.dualResidual
      << " complementarity=" << exact.complementarityResidual
      << " projectedGradientRetries=" << exact.projectedGradientRetries
      << " projectedGradientRetryUsed=" << exact.projectedGradientRetryUsed
      << " fbfStatus=" << exact.fbfStatus
      << " fbfIterations=" << exact.fbfIterations;
  expectConvergedResidualTrace(exact.residualTrace, 1e-6);

  EXPECT_TRUE(exact.finiteState);
  EXPECT_GT(exact.lastContacts, 2u);
  EXPECT_GT(exact.minCardAxisUp, 0.85)
      << "minCenterHeight=" << exact.minCenterHeight
      << " maxHorizontalTravel=" << exact.maxHorizontalTravel
      << " contacts=" << exact.lastContacts;
  EXPECT_GT(exact.minCenterHeight, 0.25);
  EXPECT_LT(exact.maxHorizontalTravel, 0.45);

  EXPECT_TRUE(boxed.finiteState);
  EXPECT_GT(boxed.lastContacts, 2u);
}

TEST(
    ExactCoulombFbfPaperFixtures,
    CardHouseReconstructionUsesNominalInterfaceOverlap)
{
  const Eigen::Vector3d size(
      kCardHouseThickness, kCardHouseWidth, kCardHouseHeight);
  const Eigen::Isometry3d left = createCardHouseAFrameTransform(0.0, 0.0, true);
  const Eigen::Isometry3d right
      = createCardHouseAFrameTransform(0.0, 0.0, false);

  const auto horizontalHalfExtent = [&size](const Eigen::Matrix3d& rotation) {
    return 0.5
           * (std::abs(rotation(0, 0)) * size.x()
              + std::abs(rotation(0, 1)) * size.y()
              + std::abs(rotation(0, 2)) * size.z());
  };
  const double apexOverlap
      = horizontalHalfExtent(left.linear())
        + horizontalHalfExtent(right.linear())
        - std::abs(right.translation().x() - left.translation().x());
  EXPECT_NEAR(apexOverlap, kCardHouseInitialPenetration, 1e-12);

  const double leftGroundOverlap
      = computeCardHouseVerticalHalfExtent(left.linear())
        - left.translation().z();
  EXPECT_NEAR(leftGroundOverlap, kCardHouseInitialPenetration, 1e-12);

  // Horizontal cards have their one-meter local height along world X. Two
  // adjacent supports therefore meet at the same nominal overlap.
  EXPECT_NEAR(
      kCardHouseHeight - kCardHouseFrameSpacing,
      kCardHouseInitialPenetration,
      1e-12);
}

TEST(ExactCoulombFbfPaperFixtures, CardHouseFourLevelSceneBuilds)
{
  auto exactWorld = createCardHouseFourLevelWorld(true);
  const auto* exactSolver = getExactCoulombSolver(exactWorld);

  std::size_t cardCount = 0u;
  for (std::size_t i = 0u; i < exactWorld->getNumSkeletons(); ++i) {
    const auto skeleton = exactWorld->getSkeleton(i);
    if (skeleton != nullptr && isCardHouseSkeletonName(skeleton->getName())) {
      ++cardCount;
    }
  }

  EXPECT_NE(nullptr, exactSolver);
  EXPECT_EQ(cardCount, computeCardHouseCardCount(kCardHouseFourLevelCount));
  EXPECT_EQ(
      exactWorld->getNumSkeletons(),
      computeCardHouseCardCount(kCardHouseFourLevelCount) + 1u);
  EXPECT_EQ(
      exactWorld->getConstraintSolver()->getCollisionOption().maxNumContacts,
      256u);
  EXPECT_EQ(
      exactWorld->getConstraintSolver()
          ->getCollisionOption()
          .maxNumContactsPerPair,
      8u);
}

TEST(ExactCoulombFbfPaperFixtures, CardHouseTenLevelSceneBuilds)
{
  auto exactWorld = createCardHouseTenLevelWorld(true);
  const auto* exactSolver = getExactCoulombSolver(exactWorld);

  std::size_t cardCount = 0u;
  for (std::size_t i = 0u; i < exactWorld->getNumSkeletons(); ++i) {
    const auto skeleton = exactWorld->getSkeleton(i);
    if (skeleton != nullptr && isCardHouseSkeletonName(skeleton->getName())) {
      ++cardCount;
    }
  }

  EXPECT_NE(nullptr, exactSolver);
  EXPECT_EQ(cardCount, computeCardHouseCardCount(kCardHouseTenLevelCount));
  EXPECT_EQ(
      exactWorld->getNumSkeletons(),
      computeCardHouseCardCount(kCardHouseTenLevelCount) + 1u);
  EXPECT_EQ(
      exactWorld->getConstraintSolver()->getCollisionOption().maxNumContacts,
      512u);
  EXPECT_EQ(
      exactWorld->getConstraintSolver()
          ->getCollisionOption()
          .maxNumContactsPerPair,
      8u);
}

TEST(ExactCoulombFbfPaperFixtures, CardHouseFourLevelOneStepReducedContactProbe)
{
  if (!areFbfPaperStressTestsEnabled()) {
    GTEST_SKIP() << "Scientific stress fixture is opt-in; set "
                 << kFbfPaperStressTestEnv << "=1 to run all stress fixtures.";
  }

  const auto exact = runCardHouseFourLevelOneStep(
      true, kCardHouseReducedMaxContacts, kCardHouseReducedMaxContactsPerPair);

  RecordProperty(
      "max_contacts", static_cast<int>(kCardHouseReducedMaxContacts));
  RecordProperty(
      "max_contacts_per_pair",
      static_cast<int>(kCardHouseReducedMaxContactsPerPair));
  RecordProperty("contacts", static_cast<int>(exact.lastContacts));
  RecordProperty("elapsed_ms", exact.elapsedMs);
  RecordProperty("step_size_scale", kCardHouseStepSizeScale);
  RecordProperty("outer_relaxation", kCardHouseOuterRelaxation);
  RecordProperty("residual", exact.residual);
  RecordProperty("exact_solves", static_cast<int>(exact.exactSolves));
  RecordProperty("exact_failures", static_cast<int>(exact.exactFailures));
  RecordProperty("fbf_iterations", exact.fbfIterations);
  RecordProperty("max_fbf_iterations", exact.maxFbfIterations);
  RecordProperty(
      "total_fbf_iterations", static_cast<int>(exact.totalFbfIterations));
  RecordProperty("last_failure_residual", exact.failedResidual);
  RecordProperty("last_failure_primal", exact.failedPrimalResidual);
  RecordProperty("last_failure_dual", exact.failedDualResidual);
  RecordProperty(
      "last_failure_complementarity", exact.failedComplementarityResidual);
  RecordProperty("last_failure_fbf_status", exact.failedFbfStatus);
  RecordProperty("last_failure_fbf_iterations", exact.failedFbfIterations);

  EXPECT_EQ(
      exact.cardCount, computeCardHouseCardCount(kCardHouseFourLevelCount));
  EXPECT_TRUE(exact.finiteState);
  EXPECT_GT(exact.lastContacts, 0u);
  EXPECT_TRUE(std::isfinite(exact.elapsedMs));
  EXPECT_LT(exact.elapsedMs, 60000.0);
  EXPECT_TRUE(exact.exactSuccess)
      << "elapsedMs=" << exact.elapsedMs << " contacts=" << exact.lastContacts
      << " residual=" << exact.residual << " primal=" << exact.primalResidual
      << " dual=" << exact.dualResidual
      << " complementarity=" << exact.complementarityResidual
      << " fallbacks=" << exact.boxedFallbacks
      << " exactFailures=" << exact.exactFailures
      << " projectedGradientRetries=" << exact.projectedGradientRetries
      << " fbfStatus=" << exact.fbfStatus
      << " fbfIterations=" << exact.fbfIterations
      << " maxFbfIterations=" << exact.maxFbfIterations
      << " totalFbfIterations=" << exact.totalFbfIterations
      << " failedResidual=" << exact.failedResidual
      << " failedDual=" << exact.failedDualResidual
      << " failedFbfStatus=" << exact.failedFbfStatus
      << " failedFbfIterations=" << exact.failedFbfIterations;
  EXPECT_EQ(exact.boxedFallbacks, 0u);
  EXPECT_EQ(exact.exactFailures, 0u);
  EXPECT_GT(exact.exactSolves, 0u);
  EXPECT_TRUE(std::isfinite(exact.residual));
  EXPECT_LE(exact.residual, 1e-6);
  expectConvergedResidualTrace(exact.residualTrace, 1e-6);
}

TEST(
    ExactCoulombFbfPaperFixtures,
    CardHouseFourLevelSettleProjectileScaffoldRuns)
{
  const auto exact = runCardHouseFourLevelSettleProjectileSmoke();

  RecordProperty(
      "max_contacts",
      static_cast<int>(kCardHouseSettleProjectileSmokeContacts));
  RecordProperty(
      "max_contacts_per_pair",
      static_cast<int>(kCardHouseReducedMaxContactsPerPair));
  RecordProperty("steps", 2);
  RecordProperty("settle_steps", 1);
  RecordProperty("projectiles", static_cast<int>(kCardHouseProjectileCount));
  RecordProperty("contacts", static_cast<int>(exact.lastContacts));
  RecordProperty("elapsed_ms", exact.elapsedMs);
  RecordProperty("residual", exact.residual);
  RecordProperty("exact_solves", static_cast<int>(exact.exactSolves));
  RecordProperty("exact_failures", static_cast<int>(exact.exactFailures));
  RecordProperty("fallbacks", static_cast<int>(exact.boxedFallbacks));
  RecordProperty("fbf_iterations", exact.fbfIterations);
  RecordProperty("max_fbf_iterations", exact.maxFbfIterations);
  RecordProperty(
      "total_fbf_iterations", static_cast<int>(exact.totalFbfIterations));
  RecordProperty("max_projectile_speed", exact.maxProjectileSpeed);

  EXPECT_EQ(
      exact.cardCount, computeCardHouseCardCount(kCardHouseFourLevelCount));
  EXPECT_EQ(exact.projectileCount, kCardHouseProjectileCount);
  EXPECT_TRUE(exact.finiteState);
  EXPECT_GT(exact.lastContacts, 0u);
  EXPECT_TRUE(std::isfinite(exact.elapsedMs));
  EXPECT_LT(exact.elapsedMs, 60000.0);
  EXPECT_GT(exact.maxProjectileSpeed, 0.0);
  EXPECT_TRUE(exact.exactSuccess)
      << "elapsedMs=" << exact.elapsedMs << " contacts=" << exact.lastContacts
      << " residual=" << exact.residual << " primal=" << exact.primalResidual
      << " dual=" << exact.dualResidual
      << " complementarity=" << exact.complementarityResidual
      << " fallbacks=" << exact.boxedFallbacks
      << " exactFailures=" << exact.exactFailures
      << " projectedGradientRetries=" << exact.projectedGradientRetries
      << " fbfStatus=" << exact.fbfStatus
      << " fbfIterations=" << exact.fbfIterations
      << " maxFbfIterations=" << exact.maxFbfIterations
      << " totalFbfIterations=" << exact.totalFbfIterations;
  EXPECT_EQ(exact.boxedFallbacks, 0u);
  EXPECT_EQ(exact.exactFailures, 0u);
  EXPECT_GT(exact.exactSolves, 0u);
  EXPECT_TRUE(std::isfinite(exact.residual));
  EXPECT_LE(exact.residual, 1e-6);
  expectConvergedResidualTrace(exact.residualTrace, 1e-6);
}

TEST(
    ExactCoulombFbfPaperFixtures,
    CardHouseFourLevelFullManifoldSplitImpulseSettleProbe)
{
  if (!areFbfPaperStressTestsEnabled()) {
    GTEST_SKIP() << "Scientific stress fixture is opt-in; set "
                 << kFbfPaperStressTestEnv << "=1 to run all stress fixtures.";
  }

  // Multi-step full-natural-manifold settle regression. DART's ERP
  // position-correction bias in the velocity-phase right-hand side makes
  // settle steps 2+ unsolvable at the paper tolerance (measured fallbacks at
  // residuals 2.9e-6 through 1.5e-4 even warm-started with a 120000-outer
  // budget); separating position recovery with split impulse matches the
  // paper's formulation, and every settle step then solves exactly with zero
  // fallbacks, roughly 30x faster.
  auto options = makeCardHouseFourLevelProbeSolverOptions();
  options.enableWarmStart = true;
  // Later settle steps densify and couple harder than the one-step probe;
  // use the same outer budget as the long-run trace scenario.
  options.maxOuterIterations = 120000;
  auto world = createCardHouseFourLevelWorld(
      true,
      options,
      kCardHouseReducedMaxContacts,
      kCardHouseReducedMaxContactsPerPair);
  // Must be enabled after the solver is installed: World::setConstraintSolver
  // copies the previous solver's split-impulse flag into the new solver.
  world->getConstraintSolver()->setSplitImpulseEnabled(true);
  auto* solver = dynamic_cast<constraint::ExactCoulombFbfConstraintSolver*>(
      world->getConstraintSolver());
  ASSERT_NE(solver, nullptr);

  constexpr std::size_t kSettleSteps = 10u;
  const auto startTime = std::chrono::steady_clock::now();
  double maxResidual = 0.0;
  std::size_t maxContactsSeen = 0u;
  for (std::size_t step = 0u; step < kSettleSteps; ++step) {
    world->step();
    ASSERT_EQ(
        solver->getLastExactCoulombStatus(),
        constraint::ExactCoulombFbfConstraintSolverStatus::Success)
        << "settle step " << step
        << " residual=" << solver->getLastExactCoulombResidual()
        << " fallbacks=" << solver->getNumBoxedLcpFallbacks();
    const double residual = solver->getLastExactCoulombResidual();
    ASSERT_TRUE(std::isfinite(residual)) << "settle step " << step;
    maxResidual = (std::max)(maxResidual, residual);
    maxContactsSeen = (std::max)(
        maxContactsSeen,
        world->getConstraintSolver()
            ->getLastCollisionResult()
            .getNumContacts());
  }
  const double elapsedMs = std::chrono::duration<double, std::milli>(
                               std::chrono::steady_clock::now() - startTime)
                               .count();

  RecordProperty(
      "max_contacts", static_cast<int>(kCardHouseReducedMaxContacts));
  RecordProperty(
      "max_contacts_per_pair",
      static_cast<int>(kCardHouseReducedMaxContactsPerPair));
  RecordProperty("settle_steps", static_cast<int>(kSettleSteps));
  RecordProperty("max_contacts_seen", static_cast<int>(maxContactsSeen));
  RecordProperty("elapsed_ms", elapsedMs);
  RecordProperty("max_residual", maxResidual);
  RecordProperty(
      "exact_solves", static_cast<int>(solver->getNumExactCoulombSolves()));
  RecordProperty(
      "warm_starts", static_cast<int>(solver->getNumExactCoulombWarmStarts()));
  RecordProperty(
      "fallbacks", static_cast<int>(solver->getNumBoxedLcpFallbacks()));

  EXPECT_EQ(solver->getNumBoxedLcpFallbacks(), 0u);
  EXPECT_EQ(solver->getNumExactCoulombFailures(), 0u);
  EXPECT_GE(solver->getNumExactCoulombSolves(), kSettleSteps);
  EXPECT_GE(solver->getNumExactCoulombWarmStarts(), 1u);
  EXPECT_LE(maxResidual, 1e-6);
  // The full natural manifold must actually engage while settling.
  EXPECT_GT(maxContactsSeen, 100u);
  EXPECT_LT(elapsedMs, 120000.0);

  for (std::size_t i = 0u; i < world->getNumSkeletons(); ++i) {
    const auto skeleton = world->getSkeleton(i);
    ASSERT_NE(skeleton, nullptr);
    EXPECT_TRUE(skeleton->getPositions().allFinite());
    EXPECT_TRUE(skeleton->getVelocities().allFinite());
  }
}

TEST(ExactCoulombFbfPaperFixtures, MasonryArch25SceneBuilds)
{
  auto exactWorld = createMasonryArch25World(true);
  const auto* exactSolver = getExactCoulombSolver(exactWorld);

  std::size_t archStones = 0u;
  std::size_t dynamicStones = 0u;
  for (std::size_t i = 0u; i < exactWorld->getNumSkeletons(); ++i) {
    const auto skeleton = exactWorld->getSkeleton(i);
    if (skeleton != nullptr && isMasonryArchStoneName(skeleton->getName())) {
      ++archStones;
      if (skeleton->isMobile())
        ++dynamicStones;
    }
  }

  EXPECT_NE(nullptr, exactSolver);
  EXPECT_EQ(archStones, kArchStoneCount);
  // Paper Fig. 7 pins the two springers; the other 23 stones are dynamic.
  EXPECT_EQ(dynamicStones, kArchStoneCount - 2u);
  EXPECT_EQ(
      exactWorld->getConstraintSolver()->getCollisionOption().maxNumContacts,
      kArchReducedMaxContacts);
  EXPECT_EQ(
      exactWorld->getConstraintSolver()
          ->getCollisionOption()
          .maxNumContactsPerPair,
      kArchReducedMaxContactsPerPair);
}

TEST(ExactCoulombFbfPaperFixtures, MasonryArch25OneStepReducedContactProbe)
{
  const auto exact = runMasonryArch25OneStep(true);

  RecordProperty("max_contacts", static_cast<int>(kArchReducedMaxContacts));
  RecordProperty(
      "max_contacts_per_pair",
      static_cast<int>(kArchReducedMaxContactsPerPair));
  RecordProperty("contacts", static_cast<int>(exact.lastContacts));
  RecordProperty("elapsed_ms", exact.elapsedMs);
  RecordProperty("residual", exact.residual);
  RecordProperty("exact_solves", static_cast<int>(exact.exactSolves));
  RecordProperty("exact_failures", static_cast<int>(exact.exactFailures));
  RecordProperty("fallbacks", static_cast<int>(exact.boxedFallbacks));
  RecordProperty("fbf_iterations", exact.fbfIterations);
  RecordProperty("max_fbf_iterations", exact.maxFbfIterations);
  RecordProperty(
      "total_fbf_iterations", static_cast<int>(exact.totalFbfIterations));
  RecordProperty("last_failure_residual", exact.failedResidual);
  RecordProperty("last_failure_dual", exact.failedDualResidual);
  RecordProperty("last_failure_fbf_status", exact.failedFbfStatus);
  RecordProperty("last_failure_fbf_iterations", exact.failedFbfIterations);

  EXPECT_EQ(exact.cardCount, kArchStoneCount);
  EXPECT_TRUE(exact.finiteState);
  EXPECT_GT(exact.lastContacts, 0u);
  EXPECT_TRUE(std::isfinite(exact.elapsedMs));
  EXPECT_LT(exact.elapsedMs, 60000.0);
  EXPECT_TRUE(exact.exactSuccess)
      << "elapsedMs=" << exact.elapsedMs << " contacts=" << exact.lastContacts
      << " residual=" << exact.residual << " primal=" << exact.primalResidual
      << " dual=" << exact.dualResidual
      << " complementarity=" << exact.complementarityResidual
      << " fallbacks=" << exact.boxedFallbacks
      << " exactFailures=" << exact.exactFailures
      << " failedResidual=" << exact.failedResidual
      << " failedDual=" << exact.failedDualResidual
      << " failedFbfStatus=" << exact.failedFbfStatus
      << " failedFbfIterations=" << exact.failedFbfIterations;
  EXPECT_EQ(exact.boxedFallbacks, 0u);
  EXPECT_EQ(exact.exactFailures, 0u);
  EXPECT_GT(exact.exactSolves, 0u);
  EXPECT_TRUE(std::isfinite(exact.residual));
  EXPECT_LE(exact.residual, 1e-6);
  expectConvergedResidualTrace(exact.residualTrace, 1e-6);
}

TEST(ExactCoulombFbfPaperFixtures, MasonryArch25ProjectileScaffoldRuns)
{
  const auto exact = runMasonryArch25ProjectileScaffold();

  RecordProperty("max_contacts", static_cast<int>(kArchReducedMaxContacts));
  RecordProperty(
      "max_contacts_per_pair",
      static_cast<int>(kArchReducedMaxContactsPerPair));
  RecordProperty("contacts", static_cast<int>(exact.lastContacts));
  RecordProperty("elapsed_ms", exact.elapsedMs);
  RecordProperty("residual", exact.residual);
  RecordProperty("projectiles", static_cast<int>(exact.projectileCount));
  RecordProperty("max_projectile_speed", exact.maxProjectileSpeed);
  RecordProperty("exact_solves", static_cast<int>(exact.exactSolves));
  RecordProperty("exact_failures", static_cast<int>(exact.exactFailures));
  RecordProperty("fallbacks", static_cast<int>(exact.boxedFallbacks));
  RecordProperty("fbf_iterations", exact.fbfIterations);
  RecordProperty("max_fbf_iterations", exact.maxFbfIterations);
  RecordProperty(
      "total_fbf_iterations", static_cast<int>(exact.totalFbfIterations));

  EXPECT_EQ(exact.cardCount, kArchStoneCount);
  EXPECT_EQ(exact.projectileCount, kArchProjectileCount);
  EXPECT_TRUE(exact.finiteState);
  EXPECT_GT(exact.maxProjectileSpeed, 0.0);
  EXPECT_GT(exact.lastContacts, 0u);
  EXPECT_TRUE(std::isfinite(exact.elapsedMs));
  EXPECT_LT(exact.elapsedMs, 60000.0);
  EXPECT_TRUE(exact.exactSuccess)
      << "elapsedMs=" << exact.elapsedMs << " contacts=" << exact.lastContacts
      << " residual=" << exact.residual << " primal=" << exact.primalResidual
      << " dual=" << exact.dualResidual
      << " complementarity=" << exact.complementarityResidual
      << " fallbacks=" << exact.boxedFallbacks
      << " exactFailures=" << exact.exactFailures
      << " failedResidual=" << exact.failedResidual
      << " failedDual=" << exact.failedDualResidual
      << " failedFbfStatus=" << exact.failedFbfStatus
      << " failedFbfIterations=" << exact.failedFbfIterations;
  EXPECT_EQ(exact.boxedFallbacks, 0u);
  EXPECT_EQ(exact.exactFailures, 0u);
  EXPECT_GT(exact.exactSolves, 0u);
  EXPECT_TRUE(std::isfinite(exact.residual));
  EXPECT_LE(exact.residual, 1e-6);
  expectConvergedResidualTrace(exact.residualTrace, 1e-6);
}

TEST(ExactCoulombFbfPaperFixtures, MasonryArch101SceneBuilds)
{
  auto exactWorld = createMasonryArch101World(true);
  const auto* exactSolver = getExactCoulombSolver(exactWorld);

  std::size_t archStones = 0u;
  std::size_t dynamicStones = 0u;
  for (std::size_t i = 0u; i < exactWorld->getNumSkeletons(); ++i) {
    const auto skeleton = exactWorld->getSkeleton(i);
    if (skeleton != nullptr && isMasonryArchStoneName(skeleton->getName())) {
      ++archStones;
      if (skeleton->isMobile())
        ++dynamicStones;
    }
  }

  EXPECT_NE(nullptr, exactSolver);
  EXPECT_EQ(archStones, kArch101StoneCount);
  // Fig. 8 describes the 101-stone case as the same setup, so its two
  // springers are pinned and its other 99 stones are dynamic.
  EXPECT_EQ(dynamicStones, kArch101StoneCount - 2u);
  EXPECT_EQ(
      exactWorld->getConstraintSolver()->getCollisionOption().maxNumContacts,
      kArch101ReducedMaxContacts);
  EXPECT_EQ(
      exactWorld->getConstraintSolver()
          ->getCollisionOption()
          .maxNumContactsPerPair,
      kArch101ReducedMaxContactsPerPair);
}

TEST(ExactCoulombFbfPaperFixtures, MasonryArch101OneStepReducedContactProbe)
{
  const auto exact = runMasonryArch101OneStep(true);

  RecordProperty("max_contacts", static_cast<int>(kArch101ReducedMaxContacts));
  RecordProperty(
      "max_contacts_per_pair",
      static_cast<int>(kArch101ReducedMaxContactsPerPair));
  RecordProperty("contacts", static_cast<int>(exact.lastContacts));
  RecordProperty("elapsed_ms", exact.elapsedMs);
  RecordProperty("residual", exact.residual);
  RecordProperty("exact_solves", static_cast<int>(exact.exactSolves));
  RecordProperty("exact_failures", static_cast<int>(exact.exactFailures));
  RecordProperty("fallbacks", static_cast<int>(exact.boxedFallbacks));
  RecordProperty("fbf_iterations", exact.fbfIterations);
  RecordProperty("max_fbf_iterations", exact.maxFbfIterations);
  RecordProperty(
      "total_fbf_iterations", static_cast<int>(exact.totalFbfIterations));
  RecordProperty("last_failure_residual", exact.failedResidual);
  RecordProperty("last_failure_dual", exact.failedDualResidual);
  RecordProperty("last_failure_fbf_status", exact.failedFbfStatus);
  RecordProperty("last_failure_fbf_iterations", exact.failedFbfIterations);

  EXPECT_EQ(exact.cardCount, kArch101StoneCount);
  EXPECT_TRUE(exact.finiteState);
  EXPECT_GT(exact.lastContacts, 0u);
  EXPECT_TRUE(std::isfinite(exact.elapsedMs));
  EXPECT_LT(exact.elapsedMs, 60000.0);
  EXPECT_TRUE(exact.exactSuccess)
      << "elapsedMs=" << exact.elapsedMs << " contacts=" << exact.lastContacts
      << " residual=" << exact.residual << " primal=" << exact.primalResidual
      << " dual=" << exact.dualResidual
      << " complementarity=" << exact.complementarityResidual
      << " fallbacks=" << exact.boxedFallbacks
      << " exactFailures=" << exact.exactFailures
      << " failedResidual=" << exact.failedResidual
      << " failedDual=" << exact.failedDualResidual
      << " failedFbfStatus=" << exact.failedFbfStatus
      << " failedFbfIterations=" << exact.failedFbfIterations;
  EXPECT_EQ(exact.boxedFallbacks, 0u);
  EXPECT_EQ(exact.exactFailures, 0u);
  EXPECT_GT(exact.exactSolves, 0u);
  EXPECT_TRUE(std::isfinite(exact.residual));
  EXPECT_LE(exact.residual, 1e-6);
  expectConvergedResidualTrace(exact.residualTrace, 1e-6);
}

// Full natural manifold probe: 512/4 caps observe the arch's full 96-contact
// manifold (per_pair 8 observes the same 96, so the manifold is saturated).
// Clean at outer_relaxation=1.5, step_size_scale=10, 120000 outer budget.
TEST(ExactCoulombFbfPaperFixtures, MasonryArch25FullManifoldOneStepProbe)
{
  const auto exact = runMasonryArch25OneStep(
      true, kArchFullManifoldMaxContacts, kArchFullManifoldMaxContactsPerPair);

  RecordProperty(
      "max_contacts", static_cast<int>(kArchFullManifoldMaxContacts));
  RecordProperty(
      "max_contacts_per_pair",
      static_cast<int>(kArchFullManifoldMaxContactsPerPair));
  RecordProperty("contacts", static_cast<int>(exact.lastContacts));
  RecordProperty("elapsed_ms", exact.elapsedMs);
  RecordProperty("residual", exact.residual);
  RecordProperty("exact_solves", static_cast<int>(exact.exactSolves));
  RecordProperty("exact_failures", static_cast<int>(exact.exactFailures));
  RecordProperty("fallbacks", static_cast<int>(exact.boxedFallbacks));
  RecordProperty("fbf_iterations", exact.fbfIterations);
  RecordProperty("max_fbf_iterations", exact.maxFbfIterations);
  RecordProperty(
      "total_fbf_iterations", static_cast<int>(exact.totalFbfIterations));
  RecordProperty("last_failure_residual", exact.failedResidual);
  RecordProperty("last_failure_dual", exact.failedDualResidual);
  RecordProperty("last_failure_fbf_status", exact.failedFbfStatus);
  RecordProperty("last_failure_fbf_iterations", exact.failedFbfIterations);

  EXPECT_EQ(exact.cardCount, kArchStoneCount);
  EXPECT_TRUE(exact.finiteState);
  EXPECT_GT(exact.lastContacts, 0u);
  EXPECT_TRUE(std::isfinite(exact.elapsedMs));
  // The full natural manifold (96 contacts) takes materially longer than the
  // reduced-cap rung (measured ~63-74 s across machines), so this probe uses
  // a wider bound than the reduced one-step probes' 60 s sanity check.
  EXPECT_LT(exact.elapsedMs, 120000.0);
  EXPECT_TRUE(exact.exactSuccess)
      << "elapsedMs=" << exact.elapsedMs << " contacts=" << exact.lastContacts
      << " residual=" << exact.residual << " primal=" << exact.primalResidual
      << " dual=" << exact.dualResidual
      << " complementarity=" << exact.complementarityResidual
      << " fallbacks=" << exact.boxedFallbacks
      << " exactFailures=" << exact.exactFailures
      << " failedResidual=" << exact.failedResidual
      << " failedDual=" << exact.failedDualResidual
      << " failedFbfStatus=" << exact.failedFbfStatus
      << " failedFbfIterations=" << exact.failedFbfIterations;
  EXPECT_EQ(exact.boxedFallbacks, 0u);
  EXPECT_EQ(exact.exactFailures, 0u);
  EXPECT_GT(exact.exactSolves, 0u);
  EXPECT_TRUE(std::isfinite(exact.residual));
  EXPECT_LE(exact.residual, 1e-6);
  expectConvergedResidualTrace(exact.residualTrace, 1e-6);
}

// Full natural manifold probe: 512/4 caps observe the full 512-contact
// manifold for the 101-stone arch. Clean at outer_relaxation=1.5,
// step_size_scale=10, 120000 outer budget.
TEST(ExactCoulombFbfPaperFixtures, MasonryArch101FullManifoldOneStepProbe)
{
  if (!areFbfPaperStressTestsEnabled()) {
    GTEST_SKIP() << "Scientific stress fixture is opt-in; set "
                 << kFbfPaperStressTestEnv << "=1 to run all stress fixtures.";
  }

  const auto exact = runMasonryArch101OneStep(
      true, kArchFullManifoldMaxContacts, kArchFullManifoldMaxContactsPerPair);

  RecordProperty(
      "max_contacts", static_cast<int>(kArchFullManifoldMaxContacts));
  RecordProperty(
      "max_contacts_per_pair",
      static_cast<int>(kArchFullManifoldMaxContactsPerPair));
  RecordProperty("contacts", static_cast<int>(exact.lastContacts));
  RecordProperty("elapsed_ms", exact.elapsedMs);
  RecordProperty("residual", exact.residual);
  RecordProperty("exact_solves", static_cast<int>(exact.exactSolves));
  RecordProperty("exact_failures", static_cast<int>(exact.exactFailures));
  RecordProperty("fallbacks", static_cast<int>(exact.boxedFallbacks));
  RecordProperty("fbf_iterations", exact.fbfIterations);
  RecordProperty("max_fbf_iterations", exact.maxFbfIterations);
  RecordProperty(
      "total_fbf_iterations", static_cast<int>(exact.totalFbfIterations));
  RecordProperty("last_failure_residual", exact.failedResidual);
  RecordProperty("last_failure_dual", exact.failedDualResidual);
  RecordProperty("last_failure_fbf_status", exact.failedFbfStatus);
  RecordProperty("last_failure_fbf_iterations", exact.failedFbfIterations);

  EXPECT_EQ(exact.cardCount, kArch101StoneCount);
  EXPECT_TRUE(exact.finiteState);
  EXPECT_GT(exact.lastContacts, 0u);
  EXPECT_TRUE(std::isfinite(exact.elapsedMs));
  // The full 512-contact manifold takes materially longer than the reduced
  // rung (measured ~37-53 s across machines), so this probe uses a wider
  // bound than the reduced one-step probes' 60 s sanity check.
  EXPECT_LT(exact.elapsedMs, 120000.0);
  EXPECT_TRUE(exact.exactSuccess)
      << "elapsedMs=" << exact.elapsedMs << " contacts=" << exact.lastContacts
      << " residual=" << exact.residual << " primal=" << exact.primalResidual
      << " dual=" << exact.dualResidual
      << " complementarity=" << exact.complementarityResidual
      << " fallbacks=" << exact.boxedFallbacks
      << " exactFailures=" << exact.exactFailures
      << " failedResidual=" << exact.failedResidual
      << " failedDual=" << exact.failedDualResidual
      << " failedFbfStatus=" << exact.failedFbfStatus
      << " failedFbfIterations=" << exact.failedFbfIterations;
  EXPECT_EQ(exact.boxedFallbacks, 0u);
  EXPECT_EQ(exact.exactFailures, 0u);
  EXPECT_GT(exact.exactSolves, 0u);
  EXPECT_TRUE(std::isfinite(exact.residual));
  EXPECT_LE(exact.residual, 1e-6);
  expectConvergedResidualTrace(exact.residualTrace, 1e-6);
}
