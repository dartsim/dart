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

#include <dart/simulation/DeactivationOptions.hpp>
#include <dart/simulation/World.hpp>

#include <dart/constraint/ExactCoulombFbfConstraintSolver.hpp>

#include <dart/collision/dart/DARTCollisionDetector.hpp>

#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/PlaneShape.hpp>
#include <dart/dynamics/ShapeFrame.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/SphereShape.hpp>

#include <dart/math/detail/MasonryArchGeometry.hpp>

#include <Eigen/Geometry>
#include <benchmark/benchmark.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>

#include <cmath>
#include <cstddef>

namespace {

constexpr double kDt = 1.0 / 60.0;
constexpr double kGravity = 9.81;
constexpr double kInclineTan = 0.5;
constexpr double kBackspinRadius = 0.25;
constexpr double kBackspinLinearVelocity = 4.0;
constexpr double kBackspinAngularVelocity = -200.0;
constexpr double kBackspinFriction = 0.5;
constexpr double kTurntableInitialRadius = 1.0;
constexpr double kTurntableRampDuration = 1.0;
constexpr double kPainleveInitialPitch = 0.08;
constexpr double kPainleveInitialVelocity = 3.6;
constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kCardHouseFriction = 0.8;
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
constexpr std::size_t kCardHouseSettleProjectileSettleSteps = 1u;
constexpr std::size_t kCardHouseSettleProjectileSteps = 2u;
constexpr std::size_t kCardHouseProjectileCount = 4u;
// Official paper imagery shows cube projectiles. The edge length preserves the
// former sphere diameter but remains reconstructed: the author dimensions,
// mass, and launch speed are unpublished.
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
constexpr std::size_t kCardHouseTenLevelConstructionMaxContacts = 512u;
constexpr std::size_t kCardHouseTenLevelConstructionMaxContactsPerPair = 8u;
// The paper's contact-rich arch experiments explicitly use mu=0.8. The
// credited Rigid-IPC geometry scene uses 0.5, but that source value is not the
// paper experiment's contact contract.
constexpr double kArchFriction = 0.8;
// Rigid-IPC's default body density ("plastic", src/io/read_rb_scene.cpp:68).
constexpr double kArchDensity = 1000.0;
constexpr std::size_t kArchStoneCount = 25u;
constexpr std::size_t kArch101StoneCount = 101u;
constexpr std::size_t kArchReducedMaxContacts = 48u;
constexpr std::size_t kArchReducedMaxContactsPerPair = 2u;
constexpr std::size_t kArch101ReducedMaxContacts = 38u;
constexpr std::size_t kArch101ReducedMaxContactsPerPair = 2u;
// Full natural manifold for both arches at per_pair 4: the 25-stone arch
// observes 96 actual contacts (96 also at per_pair 8, i.e. the manifold is
// saturated), and the 101-stone arch observes the full 512-contact cap.
constexpr std::size_t kArchFullManifoldMaxContacts = 512u;
constexpr std::size_t kArchFullManifoldMaxContactsPerPair = 4u;
constexpr int kArchMaxOuterIterations = 120000;
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

enum class Scenario
{
  InclineStick,
  InclineSlide,
  Backspin,
  TurntableLowSlow,
  TurntableLowFast,
  TurntableHighSlow,
  TurntableHighFast,
  PainleveSlide,
  PainleveTumble,
  CardHouseFourLevelReduced,
  CardHouseFourLevelSettleProjectileReduced,
  CardHouseTenLevelConstruction,
  MasonryArch25Reduced,
  MasonryArch25ProjectileReduced,
  MasonryArch101Reduced,
  MasonryArch25FullManifold,
  MasonryArch101FullManifold,
};

enum class SolverMode
{
  BoxedLcp,
  ExactFbf,
};

struct ExactTrace
{
  double maxResidual = std::numeric_limits<double>::quiet_NaN();
  std::size_t samples = 0u;
  std::size_t failedSteps = 0u;
  std::size_t nonFiniteResiduals = 0u;
};

struct RunMetrics
{
  double checksum = 0.0;
  double maxResidual = std::numeric_limits<double>::quiet_NaN();
  std::size_t contacts = 0u;
  std::size_t exactSolves = 0u;
  std::size_t boxedFallbacks = 0u;
  std::size_t exactFailures = 0u;
  std::size_t exactWarmStarts = 0u;
  int maxFbfIterations = 0;
  std::size_t totalFbfIterations = 0u;
  std::size_t residualSamples = 0u;
  std::size_t residualFailures = 0u;
  std::size_t nonFiniteResiduals = 0u;
};

constexpr std::size_t computeCardHouseCardCount(std::size_t levelCount)
{
  return levelCount * (3u * levelCount + 1u) / 2u;
}

const char* scenarioName(Scenario scenario)
{
  switch (scenario) {
    case Scenario::InclineStick:
      return "incline_mu_0_5";
    case Scenario::InclineSlide:
      return "incline_mu_0_4";
    case Scenario::Backspin:
      return "backspin";
    case Scenario::TurntableLowSlow:
      return "turntable_mu_0_2_omega_2";
    case Scenario::TurntableLowFast:
      return "turntable_mu_0_2_omega_5";
    case Scenario::TurntableHighSlow:
      return "turntable_mu_0_5_omega_2";
    case Scenario::TurntableHighFast:
      return "turntable_mu_0_5_omega_5";
    case Scenario::PainleveSlide:
      return "painleve_mu_0_5";
    case Scenario::PainleveTumble:
      return "painleve_mu_0_55";
    case Scenario::CardHouseFourLevelReduced:
      return "card_house_26_reduced_contact";
    case Scenario::CardHouseFourLevelSettleProjectileReduced:
      return "card_house_26_settle_projectile_reduced_contact";
    case Scenario::CardHouseTenLevelConstruction:
      return "card_house_10_construction";
    case Scenario::MasonryArch25Reduced:
      return "masonry_arch_25_reduced_contact";
    case Scenario::MasonryArch25ProjectileReduced:
      return "masonry_arch_25_projectile_reduced_contact";
    case Scenario::MasonryArch101Reduced:
      return "masonry_arch_101_reduced_contact";
    case Scenario::MasonryArch25FullManifold:
      return "masonry_arch_25_full_manifold";
    case Scenario::MasonryArch101FullManifold:
      return "masonry_arch_101_full_manifold";
  }

  return "unknown";
}

const char* solverName(SolverMode mode)
{
  switch (mode) {
    case SolverMode::BoxedLcp:
      return "boxed_lcp";
    case SolverMode::ExactFbf:
      return "exact_fbf";
  }

  return "unknown";
}

double scenarioFriction(Scenario scenario)
{
  switch (scenario) {
    case Scenario::InclineStick:
      return 0.5;
    case Scenario::InclineSlide:
      return 0.4;
    case Scenario::TurntableLowSlow:
    case Scenario::TurntableLowFast:
      return 0.2;
    case Scenario::TurntableHighSlow:
    case Scenario::TurntableHighFast:
    case Scenario::Backspin:
    case Scenario::PainleveSlide:
      return 0.5;
    case Scenario::PainleveTumble:
      return 0.55;
    case Scenario::CardHouseFourLevelReduced:
    case Scenario::CardHouseFourLevelSettleProjectileReduced:
    case Scenario::CardHouseTenLevelConstruction:
      return kCardHouseFriction;
    case Scenario::MasonryArch25Reduced:
    case Scenario::MasonryArch25ProjectileReduced:
    case Scenario::MasonryArch101Reduced:
    case Scenario::MasonryArch25FullManifold:
    case Scenario::MasonryArch101FullManifold:
      return kArchFriction;
  }

  return 0.5;
}

double turntableAngularVelocity(Scenario scenario)
{
  switch (scenario) {
    case Scenario::TurntableLowSlow:
    case Scenario::TurntableHighSlow:
      return 2.0;
    case Scenario::TurntableLowFast:
    case Scenario::TurntableHighFast:
      return 5.0;
    default:
      return 0.0;
  }
}

std::size_t scenarioSteps(Scenario scenario)
{
  switch (scenario) {
    case Scenario::InclineStick:
    case Scenario::InclineSlide:
      return static_cast<std::size_t>(std::round(2.0 / kDt));
    case Scenario::Backspin:
    case Scenario::TurntableLowSlow:
    case Scenario::TurntableLowFast:
    case Scenario::TurntableHighSlow:
    case Scenario::TurntableHighFast:
      return static_cast<std::size_t>(std::round(4.0 / kDt));
    case Scenario::PainleveSlide:
    case Scenario::PainleveTumble:
      return static_cast<std::size_t>(std::round(2.5 / kDt));
    case Scenario::CardHouseFourLevelReduced:
    case Scenario::CardHouseTenLevelConstruction:
    case Scenario::MasonryArch25Reduced:
    case Scenario::MasonryArch25ProjectileReduced:
    case Scenario::MasonryArch101Reduced:
    case Scenario::MasonryArch25FullManifold:
    case Scenario::MasonryArch101FullManifold:
      return 1u;
    case Scenario::CardHouseFourLevelSettleProjectileReduced:
      return kCardHouseSettleProjectileSteps;
  }

  return 0u;
}

bool isTurntableScenario(Scenario scenario)
{
  return scenario == Scenario::TurntableLowSlow
         || scenario == Scenario::TurntableLowFast
         || scenario == Scenario::TurntableHighSlow
         || scenario == Scenario::TurntableHighFast;
}

dart::constraint::ExactCoulombFbfConstraintSolverOptions makeFbfOptions()
{
  dart::constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.maxOuterIterations = 500;
  options.tolerance = 1e-6;
  options.innerMaxSweeps = 120;
  options.innerLocalIterations = 32;
  options.stepSizeScale = kSmallFixtureStepSizeScale;
  return options;
}

dart::constraint::ExactCoulombFbfConstraintSolverOptions makeCardFbfOptions()
{
  auto options = makeFbfOptions();
  options.maxOuterIterations = 30000;
  options.innerMaxSweeps = 120;
  options.innerLocalIterations = 32;
  options.projectedGradientMaxIterations = 200;
  options.enableWarmStart = false;
  options.stepSizeScale = kCardHouseStepSizeScale;
  options.outerRelaxation = kCardHouseOuterRelaxation;
  return options;
}

dart::constraint::ExactCoulombFbfConstraintSolverOptions makeArchFbfOptions()
{
  auto options = makeFbfOptions();
  options.maxOuterIterations = kArchMaxOuterIterations;
  options.stepSizeScale = kArchStepSizeScale;
  options.outerRelaxation = kArchOuterRelaxation;
  options.enableWarmStart = false;
  return options;
}

void configureWorldSolver(
    const std::shared_ptr<dart::simulation::World>& world,
    SolverMode solverMode,
    std::size_t maxContacts,
    std::size_t maxContactsPerPair,
    bool cardBudget = false,
    bool archBudget = false)
{
  world->setTimeStep(kDt);
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -kGravity));
  world->setNumSimulationThreads(1u);

  dart::simulation::DeactivationOptions deactivation;
  deactivation.mEnabled = false;
  world->setDeactivationOptions(deactivation);

  if (solverMode == SolverMode::ExactFbf) {
    auto solver
        = std::make_unique<dart::constraint::ExactCoulombFbfConstraintSolver>(
            archBudget   ? makeArchFbfOptions()
            : cardBudget ? makeCardFbfOptions()
                         : makeFbfOptions());
    solver->setCollisionDetector(
        dart::collision::DARTCollisionDetector::create());
    solver->setNumSimulationThreads(1u);
    world->setConstraintSolver(std::move(solver));
  } else {
    auto* solver = world->getConstraintSolver();
    solver->setCollisionDetector(
        dart::collision::DARTCollisionDetector::create());
    solver->setNumSimulationThreads(1u);
  }

  auto& collisionOption = world->getConstraintSolver()->getCollisionOption();
  collisionOption.maxNumContacts = maxContacts;
  collisionOption.maxNumContactsPerPair = maxContactsPerPair;
}

dart::dynamics::SkeletonPtr createHorizontalPlane(double frictionCoeff)
{
  auto skeleton = dart::dynamics::Skeleton::create("ground_plane");
  auto body = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>()
                  .second;
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::PlaneShape>(
          Eigen::Vector3d::UnitZ(), 0.0));
  shapeNode->getDynamicsAspect()->setFrictionCoeff(frictionCoeff);
  skeleton->setMobile(false);
  return skeleton;
}

dart::dynamics::SkeletonPtr createInclinePlane(double frictionCoeff)
{
  const double theta = std::atan(kInclineTan);
  const Eigen::Vector3d normal(-std::sin(theta), 0.0, std::cos(theta));

  auto skeleton = dart::dynamics::Skeleton::create("incline_plane");
  auto body = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>()
                  .second;
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::PlaneShape>(normal, 0.0));
  shapeNode->getDynamicsAspect()->setFrictionCoeff(frictionCoeff);
  skeleton->setMobile(false);
  return skeleton;
}

dart::dynamics::SkeletonPtr createInclineCube(double frictionCoeff)
{
  const double theta = std::atan(kInclineTan);
  const Eigen::Vector3d normal(-std::sin(theta), 0.0, std::cos(theta));
  const Eigen::Vector3d size = Eigen::Vector3d::Ones();
  constexpr double kMass = 1.0;
  constexpr double kInitialPenetration = 0.01;

  auto skeleton = dart::dynamics::Skeleton::create("incline_cube");
  dart::dynamics::GenericJoint<dart::math::SE3Space>::Properties
      jointProperties(std::string("incline_cube_joint"));
  dart::dynamics::BodyNode::Properties bodyProperties(
      dart::dynamics::BodyNode::AspectProperties("incline_cube_body"));
  bodyProperties.mInertia.setMass(kMass);
  bodyProperties.mInertia.setMoment(
      dart::dynamics::BoxShape::computeInertia(size, kMass));

  auto pair = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
      nullptr, jointProperties, bodyProperties);
  auto* joint = pair.first;
  auto* body = pair.second;
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::BoxShape>(size));
  shapeNode->getDynamicsAspect()->setFrictionCoeff(frictionCoeff);

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear()
      = Eigen::AngleAxisd(-theta, Eigen::Vector3d::UnitY()).toRotationMatrix();
  transform.translation() = normal * (0.5 * size.z() - kInitialPenetration);
  joint->setPositions(dart::dynamics::FreeJoint::convertToPositions(transform));
  return skeleton;
}

dart::dynamics::SkeletonPtr createBackspinSphere()
{
  constexpr double kMass = 1.0;
  constexpr double kInitialPenetration = 0.005;

  auto skeleton = dart::dynamics::Skeleton::create("backspin_sphere");
  dart::dynamics::GenericJoint<dart::math::SE3Space>::Properties
      jointProperties(std::string("backspin_sphere_joint"));
  dart::dynamics::BodyNode::Properties bodyProperties(
      dart::dynamics::BodyNode::AspectProperties("backspin_sphere_body"));
  bodyProperties.mInertia.setMass(kMass);
  bodyProperties.mInertia.setMoment(
      dart::dynamics::SphereShape::computeInertia(kBackspinRadius, kMass));

  auto pair = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
      nullptr, jointProperties, bodyProperties);
  auto* joint = pair.first;
  auto* body = pair.second;
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::SphereShape>(kBackspinRadius));
  shapeNode->getDynamicsAspect()->setFrictionCoeff(kBackspinFriction);

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation().z() = kBackspinRadius - kInitialPenetration;
  joint->setPositions(dart::dynamics::FreeJoint::convertToPositions(transform));
  joint->setLinearVelocity(Eigen::Vector3d(kBackspinLinearVelocity, 0.0, 0.0));
  joint->setAngularVelocity(
      Eigen::Vector3d(0.0, kBackspinAngularVelocity, 0.0));
  return skeleton;
}

dart::dynamics::SkeletonPtr createTurntableSupport(double frictionCoeff)
{
  constexpr double kThickness = 0.1;

  auto skeleton = dart::dynamics::Skeleton::create("turntable");
  dart::dynamics::GenericJoint<dart::math::SE3Space>::Properties
      jointProperties(std::string("turntable_joint"));
  dart::dynamics::BodyNode::Properties bodyProperties(
      dart::dynamics::BodyNode::AspectProperties("turntable_body"));
  bodyProperties.mInertia.setMass(1.0);

  auto pair = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
      nullptr, jointProperties, bodyProperties);
  auto* joint = pair.first;
  auto* body = pair.second;
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d(4.0, 4.0, kThickness)));
  shapeNode->getDynamicsAspect()->setFrictionCoeff(frictionCoeff);

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation().z() = -0.5 * kThickness;
  joint->setPositions(dart::dynamics::FreeJoint::convertToPositions(transform));
  skeleton->setMobile(false);
  return skeleton;
}

dart::dynamics::SkeletonPtr createTurntableRider(double frictionCoeff)
{
  const Eigen::Vector3d size = Eigen::Vector3d::Constant(0.25);
  constexpr double kMass = 1.0;
  constexpr double kInitialPenetration = 0.005;

  auto skeleton = dart::dynamics::Skeleton::create("turntable_rider");
  dart::dynamics::GenericJoint<dart::math::SE3Space>::Properties
      jointProperties(std::string("turntable_rider_joint"));
  dart::dynamics::BodyNode::Properties bodyProperties(
      dart::dynamics::BodyNode::AspectProperties("turntable_rider_body"));
  bodyProperties.mInertia.setMass(kMass);
  bodyProperties.mInertia.setMoment(
      dart::dynamics::BoxShape::computeInertia(size, kMass));

  auto pair = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
      nullptr, jointProperties, bodyProperties);
  auto* joint = pair.first;
  auto* body = pair.second;
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::BoxShape>(size));
  shapeNode->getDynamicsAspect()->setFrictionCoeff(frictionCoeff);

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(
      kTurntableInitialRadius, 0.0, 0.5 * size.z() - kInitialPenetration);
  joint->setPositions(dart::dynamics::FreeJoint::convertToPositions(transform));
  return skeleton;
}

dart::dynamics::SkeletonPtr createPainleveBox(double frictionCoeff)
{
  const Eigen::Vector3d size(0.6, 0.6, 1.0);
  constexpr double kMass = 1.0;
  constexpr double kInitialPenetration = 0.005;

  auto skeleton = dart::dynamics::Skeleton::create("painleve_box");
  dart::dynamics::GenericJoint<dart::math::SE3Space>::Properties
      jointProperties(std::string("painleve_box_joint"));
  dart::dynamics::BodyNode::Properties bodyProperties(
      dart::dynamics::BodyNode::AspectProperties("painleve_box_body"));
  bodyProperties.mInertia.setMass(kMass);
  bodyProperties.mInertia.setMoment(
      dart::dynamics::BoxShape::computeInertia(size, kMass));

  auto pair = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
      nullptr, jointProperties, bodyProperties);
  auto* joint = pair.first;
  auto* body = pair.second;
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::BoxShape>(size));
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
  joint->setPositions(dart::dynamics::FreeJoint::convertToPositions(transform));
  joint->setLinearVelocity(Eigen::Vector3d(kPainleveInitialVelocity, 0.0, 0.0));
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
  transform.translation().z()
      = baseZ + computeCardHouseVerticalHalfExtent(transform.linear())
        - kCardHouseInitialPenetration;
  return transform;
}

Eigen::Isometry3d createCardHouseHorizontalSupportTransform(
    double centerX, double baseZ)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = Eigen::AngleAxisd(0.5 * kPi, Eigen::Vector3d::UnitY())
                           .toRotationMatrix();
  transform.translation().x() = centerX;
  transform.translation().z()
      = baseZ + 0.5 * kCardHouseThickness - kCardHouseInitialPenetration;
  return transform;
}

dart::dynamics::SkeletonPtr createCardPlate(
    const std::string& name, const Eigen::Isometry3d& transform)
{
  const Eigen::Vector3d size(
      kCardHouseThickness, kCardHouseWidth, kCardHouseHeight);
  const double mass = kCardHouseDensity * size.x() * size.y() * size.z();

  auto skeleton = dart::dynamics::Skeleton::create(name);
  dart::dynamics::GenericJoint<dart::math::SE3Space>::Properties
      jointProperties(name + "_joint");
  dart::dynamics::BodyNode::Properties bodyProperties(
      dart::dynamics::BodyNode::AspectProperties(name + "_body"));
  bodyProperties.mInertia.setMass(mass);
  bodyProperties.mInertia.setMoment(
      dart::dynamics::BoxShape::computeInertia(size, mass));

  auto pair = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
      nullptr, jointProperties, bodyProperties);
  auto* joint = pair.first;
  auto* body = pair.second;
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::BoxShape>(size));
  shapeNode->getDynamicsAspect()->setFrictionCoeff(kCardHouseFriction);
  joint->setPositions(dart::dynamics::FreeJoint::convertToPositions(transform));
  return skeleton;
}

dart::dynamics::SkeletonPtr createCardHouseProjectile(std::size_t index)
{
  auto skeleton = dart::dynamics::Skeleton::create(
      "fbf_projectile_" + std::to_string(index));
  dart::dynamics::GenericJoint<dart::math::SE3Space>::Properties
      jointProperties(skeleton->getName() + "_joint");
  dart::dynamics::BodyNode::Properties bodyProperties(
      dart::dynamics::BodyNode::AspectProperties(
          skeleton->getName() + "_body"));
  bodyProperties.mInertia.setMass(kCardHouseProjectileMass);
  bodyProperties.mInertia.setMoment(dart::dynamics::BoxShape::computeInertia(
      Eigen::Vector3d::Constant(kCardHouseProjectileEdgeLength),
      kCardHouseProjectileMass));

  auto pair = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
      nullptr, jointProperties, bodyProperties);
  auto* joint = pair.first;
  auto* body = pair.second;
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d::Constant(kCardHouseProjectileEdgeLength)));
  shapeNode->getDynamicsAspect()->setFrictionCoeff(kCardHouseFriction);

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(
      (static_cast<double>(index) - 1.5) * 0.15,
      0.0,
      kCardHouseProjectileDropHeight);
  joint->setPositions(dart::dynamics::FreeJoint::convertToPositions(transform));
  joint->setLinearVelocity(
      Eigen::Vector3d(0.0, 0.0, -kCardHouseProjectileSpeed));
  return skeleton;
}

dart::dynamics::SkeletonPtr createMasonryArchStone(
    std::size_t index,
    const dart::math::detail::MasonryArchStoneBoxGeometry& geometry)
{
  const double mass = kArchDensity * geometry.size.x() * geometry.size.y()
                      * geometry.size.z();

  auto skeleton = dart::dynamics::Skeleton::create(
      "masonry_arch_stone_" + std::to_string(index));
  dart::dynamics::GenericJoint<dart::math::SE3Space>::Properties
      jointProperties(skeleton->getName() + "_joint");
  dart::dynamics::BodyNode::Properties bodyProperties(
      dart::dynamics::BodyNode::AspectProperties(
          skeleton->getName() + "_body"));
  bodyProperties.mInertia.setMass(mass);
  bodyProperties.mInertia.setMoment(
      dart::dynamics::BoxShape::computeInertia(geometry.size, mass));

  auto pair = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
      nullptr, jointProperties, bodyProperties);
  auto* joint = pair.first;
  auto* body = pair.second;
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::BoxShape>(geometry.size));
  shapeNode->getDynamicsAspect()->setFrictionCoeff(kArchFriction);
  joint->setPositions(
      dart::dynamics::FreeJoint::convertToPositions(geometry.transform));
  // Mobility is assigned by the paper-scene builder below.
  return skeleton;
}

dart::dynamics::SkeletonPtr createMasonryArchProjectile(std::size_t index)
{
  auto skeleton = dart::dynamics::Skeleton::create(
      "masonry_arch_projectile_" + std::to_string(index));
  dart::dynamics::GenericJoint<dart::math::SE3Space>::Properties
      jointProperties(skeleton->getName() + "_joint");
  dart::dynamics::BodyNode::Properties bodyProperties(
      dart::dynamics::BodyNode::AspectProperties(
          skeleton->getName() + "_body"));
  bodyProperties.mInertia.setMass(kArchProjectileMass);
  bodyProperties.mInertia.setMoment(dart::dynamics::BoxShape::computeInertia(
      Eigen::Vector3d::Constant(kArchProjectileEdgeLength),
      kArchProjectileMass));

  auto pair = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
      nullptr, jointProperties, bodyProperties);
  auto* joint = pair.first;
  auto* body = pair.second;
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::BoxShape>(
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
  joint->setPositions(dart::dynamics::FreeJoint::convertToPositions(transform));
  joint->setLinearVelocity(Eigen::Vector3d(0.0, 0.0, -kArchProjectileSpeed));
  return skeleton;
}

void addMasonryArch(
    const std::shared_ptr<dart::simulation::World>& world,
    std::size_t stoneCount)
{
  // Paper Fig. 7 pins the 25-stone springers. Fig. 8 calls the 101-stone case
  // the same setup, so both endpoints are pinned at either scale.
  world->addSkeleton(createHorizontalPlane(kArchFriction));

  const auto stoneGeometry
      = dart::math::detail::generateMasonryArchStoneBoxes(stoneCount);
  for (std::size_t i = 0u; i < stoneCount; ++i) {
    auto stone = createMasonryArchStone(i, stoneGeometry[i]);
    if (i == 0u || i + 1u == stoneCount)
      stone->setMobile(false);
    world->addSkeleton(stone);
  }
}

void addCardHouse(
    const std::shared_ptr<dart::simulation::World>& world,
    std::size_t levelCount)
{
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
          createCardHouseAFrameTransform(centerX, baseZ, true)));
      world->addSkeleton(createCardPlate(
          "card_house_l" + std::to_string(level) + "_f" + std::to_string(frame)
              + "_right",
          createCardHouseAFrameTransform(centerX, baseZ, false)));
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
          createCardHouseHorizontalSupportTransform(centerX, supportBaseZ)));
    }

    baseZ = supportBaseZ + kCardHouseThickness - kCardHouseInitialPenetration;
  }
}

std::shared_ptr<dart::simulation::World> createBenchmarkWorld(
    Scenario scenario, SolverMode solverMode)
{
  const double frictionCoeff = scenarioFriction(scenario);
  const std::size_t maxContacts
      = scenario == Scenario::Backspin ? 1u
        : scenario == Scenario::CardHouseFourLevelReduced
                || scenario
                       == Scenario::CardHouseFourLevelSettleProjectileReduced
            ? kCardHouseReducedMaxContacts
        : scenario == Scenario::CardHouseTenLevelConstruction
            ? kCardHouseTenLevelConstructionMaxContacts
        : scenario == Scenario::MasonryArch25Reduced ? kArchReducedMaxContacts
        : scenario == Scenario::MasonryArch25ProjectileReduced
            ? kArchReducedMaxContacts
        : scenario == Scenario::MasonryArch101Reduced
            ? kArch101ReducedMaxContacts
        : scenario == Scenario::MasonryArch25FullManifold
                || scenario == Scenario::MasonryArch101FullManifold
            ? kArchFullManifoldMaxContacts
            : 4u;
  const std::size_t maxContactsPerPair
      = scenario == Scenario::CardHouseFourLevelReduced
                || scenario
                       == Scenario::CardHouseFourLevelSettleProjectileReduced
            ? kCardHouseReducedMaxContactsPerPair
        : scenario == Scenario::CardHouseTenLevelConstruction
            ? kCardHouseTenLevelConstructionMaxContactsPerPair
        : scenario == Scenario::MasonryArch25Reduced
            ? kArchReducedMaxContactsPerPair
        : scenario == Scenario::MasonryArch25ProjectileReduced
            ? kArchReducedMaxContactsPerPair
        : scenario == Scenario::MasonryArch101Reduced
            ? kArch101ReducedMaxContactsPerPair
        : scenario == Scenario::MasonryArch25FullManifold
                || scenario == Scenario::MasonryArch101FullManifold
            ? kArchFullManifoldMaxContactsPerPair
            : maxContacts;
  const bool cardBudget
      = scenario == Scenario::CardHouseFourLevelReduced
        || scenario == Scenario::CardHouseFourLevelSettleProjectileReduced;
  const bool archBudget
      = scenario == Scenario::MasonryArch25Reduced
        || scenario == Scenario::MasonryArch25ProjectileReduced
        || scenario == Scenario::MasonryArch101Reduced
        || scenario == Scenario::MasonryArch25FullManifold
        || scenario == Scenario::MasonryArch101FullManifold;

  auto world = dart::simulation::World::create(
      std::string("exact_coulomb_benchmark_") + scenarioName(scenario));
  configureWorldSolver(
      world,
      solverMode,
      maxContacts,
      maxContactsPerPair,
      cardBudget,
      archBudget);

  switch (scenario) {
    case Scenario::InclineStick:
    case Scenario::InclineSlide:
      world->addSkeleton(createInclinePlane(frictionCoeff));
      world->addSkeleton(createInclineCube(frictionCoeff));
      break;
    case Scenario::Backspin:
      world->addSkeleton(createHorizontalPlane(kBackspinFriction));
      world->addSkeleton(createBackspinSphere());
      break;
    case Scenario::TurntableLowSlow:
    case Scenario::TurntableLowFast:
    case Scenario::TurntableHighSlow:
    case Scenario::TurntableHighFast:
      world->addSkeleton(createTurntableSupport(frictionCoeff));
      world->addSkeleton(createTurntableRider(frictionCoeff));
      break;
    case Scenario::PainleveSlide:
    case Scenario::PainleveTumble:
      world->addSkeleton(createHorizontalPlane(frictionCoeff));
      world->addSkeleton(createPainleveBox(frictionCoeff));
      break;
    case Scenario::CardHouseFourLevelReduced:
    case Scenario::CardHouseFourLevelSettleProjectileReduced:
      addCardHouse(world, kCardHouseFourLevelCount);
      break;
    case Scenario::CardHouseTenLevelConstruction:
      addCardHouse(world, kCardHouseTenLevelCount);
      break;
    case Scenario::MasonryArch25Reduced:
      addMasonryArch(world, kArchStoneCount);
      break;
    case Scenario::MasonryArch25ProjectileReduced:
      addMasonryArch(world, kArchStoneCount);
      for (std::size_t i = 0u; i < kArchProjectileCount; ++i)
        world->addSkeleton(createMasonryArchProjectile(i));
      break;
    case Scenario::MasonryArch101Reduced:
      addMasonryArch(world, kArch101StoneCount);
      break;
    case Scenario::MasonryArch25FullManifold:
      addMasonryArch(world, kArchStoneCount);
      break;
    case Scenario::MasonryArch101FullManifold:
      addMasonryArch(world, kArch101StoneCount);
      break;
  }

  return world;
}

const dart::constraint::ExactCoulombFbfConstraintSolver* getExactSolver(
    const std::shared_ptr<dart::simulation::World>& world)
{
  return dynamic_cast<const dart::constraint::ExactCoulombFbfConstraintSolver*>(
      world->getConstraintSolver());
}

void launchCardHouseProjectiles(
    const std::shared_ptr<dart::simulation::World>& world)
{
  if (world->getSkeleton("fbf_projectile_0") != nullptr)
    return;

  for (std::size_t i = 0u; i < kCardHouseProjectileCount; ++i)
    world->addSkeleton(createCardHouseProjectile(i));
}

void sampleExactTrace(
    const dart::constraint::ExactCoulombFbfConstraintSolver* solver,
    std::size_t& previousExactSolves,
    ExactTrace& trace)
{
  if (solver == nullptr)
    return;

  const std::size_t exactSolves = solver->getNumExactCoulombSolves();
  if (exactSolves == previousExactSolves)
    return;

  previousExactSolves = exactSolves;
  ++trace.samples;

  if (solver->getLastExactCoulombStatus()
          != dart::constraint::ExactCoulombFbfConstraintSolverStatus::Success
      || solver->getLastExactCoulombFbfStatus()
             != dart::math::detail::ExactCoulombFbfStatus::Success) {
    ++trace.failedSteps;
  }

  const double residual = solver->getLastExactCoulombResidual();
  if (std::isfinite(residual)) {
    trace.maxResidual = std::isfinite(trace.maxResidual)
                            ? std::max(trace.maxResidual, residual)
                            : residual;
  } else {
    ++trace.nonFiniteResiduals;
  }
}

void applyScenarioControl(
    const std::shared_ptr<dart::simulation::World>& world,
    Scenario scenario,
    std::size_t step)
{
  if (scenario == Scenario::CardHouseFourLevelSettleProjectileReduced
      && step == kCardHouseSettleProjectileSettleSteps) {
    launchCardHouseProjectiles(world);
  }

  if (!isTurntableScenario(scenario))
    return;

  auto turntable = world->getSkeleton("turntable");
  if (!turntable)
    return;

  auto* joint = static_cast<dart::dynamics::FreeJoint*>(turntable->getJoint(0));
  const double time = static_cast<double>(step) * kDt;
  const double ramp = std::min(time / kTurntableRampDuration, 1.0);
  joint->setAngularVelocity(
      Eigen::Vector3d(0.0, 0.0, ramp * turntableAngularVelocity(scenario)));
}

double computeChecksum(const std::shared_ptr<dart::simulation::World>& world)
{
  double checksum = world->getTime();
  for (std::size_t i = 0; i < world->getNumSkeletons(); ++i) {
    const auto skeleton = world->getSkeleton(i);
    if (!skeleton)
      continue;

    checksum += skeleton->getPositions().sum();
    checksum += 0.1 * skeleton->getVelocities().sum();
  }
  return checksum;
}

bool isValidExactBenchmarkRun(const RunMetrics& metrics)
{
  return metrics.exactSolves > 0u && metrics.boxedFallbacks == 0u
         && metrics.exactFailures == 0u && metrics.residualSamples > 0u
         && metrics.residualFailures == 0u && metrics.nonFiniteResiduals == 0u
         && std::isfinite(metrics.maxResidual) && metrics.maxResidual <= 1e-6;
}

void BM_PaperFixtureStepTime(
    benchmark::State& state, Scenario scenario, SolverMode solverMode)
{
  RunMetrics lastMetrics;
  const std::size_t steps = scenarioSteps(scenario);

  for (auto _ : state) {
    state.PauseTiming();
    auto world = createBenchmarkWorld(scenario, solverMode);
    const auto* exactSolver
        = solverMode == SolverMode::ExactFbf ? getExactSolver(world) : nullptr;
    ExactTrace trace;
    std::size_t previousExactSolves = 0u;
    state.ResumeTiming();

    for (std::size_t step = 0; step < steps; ++step) {
      applyScenarioControl(world, scenario, step);
      world->step();
      sampleExactTrace(exactSolver, previousExactSolves, trace);
    }

    state.PauseTiming();
    lastMetrics.checksum = computeChecksum(world);
    lastMetrics.contacts = world->getConstraintSolver()
                               ->getLastCollisionResult()
                               .getNumContacts();
    lastMetrics.maxResidual = trace.maxResidual;
    lastMetrics.residualSamples = trace.samples;
    lastMetrics.residualFailures = trace.failedSteps;
    lastMetrics.nonFiniteResiduals = trace.nonFiniteResiduals;
    if (exactSolver != nullptr) {
      lastMetrics.exactSolves = exactSolver->getNumExactCoulombSolves();
      lastMetrics.boxedFallbacks = exactSolver->getNumBoxedLcpFallbacks();
      lastMetrics.exactFailures = exactSolver->getNumExactCoulombFailures();
      lastMetrics.exactWarmStarts = exactSolver->getNumExactCoulombWarmStarts();
      lastMetrics.maxFbfIterations
          = exactSolver->getMaxExactCoulombIterations();
      lastMetrics.totalFbfIterations
          = exactSolver->getTotalExactCoulombIterations();
    }
    benchmark::DoNotOptimize(lastMetrics.checksum);
    world.reset();
    state.ResumeTiming();
  }

  if (solverMode == SolverMode::ExactFbf
      && !isValidExactBenchmarkRun(lastMetrics)) {
    state.SkipWithError("exact FBF paper benchmark did not converge cleanly");
    return;
  }

  state.SetItemsProcessed(
      static_cast<std::int64_t>(state.iterations() * steps));
  state.SetLabel(
      std::string("scenario=") + scenarioName(scenario) + " solver="
      + solverName(solverMode) + " steps=" + std::to_string(steps));
  state.counters["sim_s/s"] = benchmark::Counter(
      static_cast<double>(state.iterations()) * static_cast<double>(steps)
          * kDt,
      benchmark::Counter::kIsRate);
  state.counters["contacts"] = static_cast<double>(lastMetrics.contacts);
  if (scenario == Scenario::CardHouseFourLevelReduced
      || scenario == Scenario::CardHouseFourLevelSettleProjectileReduced) {
    state.counters["cards"] = static_cast<double>(
        computeCardHouseCardCount(kCardHouseFourLevelCount));
  } else if (scenario == Scenario::CardHouseTenLevelConstruction) {
    state.counters["cards"] = static_cast<double>(
        computeCardHouseCardCount(kCardHouseTenLevelCount));
  } else if (scenario == Scenario::MasonryArch25ProjectileReduced) {
    state.counters["projectiles"] = static_cast<double>(kArchProjectileCount);
  }
  state.counters["steps"] = static_cast<double>(steps);
  state.counters["exact_solves"] = static_cast<double>(lastMetrics.exactSolves);
  state.counters["exact_failures"]
      = static_cast<double>(lastMetrics.exactFailures);
  state.counters["fallbacks"] = static_cast<double>(lastMetrics.boxedFallbacks);
  state.counters["warm_starts"]
      = static_cast<double>(lastMetrics.exactWarmStarts);
  state.counters["max_fbf_iterations"]
      = static_cast<double>(lastMetrics.maxFbfIterations);
  state.counters["total_fbf_iterations"]
      = static_cast<double>(lastMetrics.totalFbfIterations);
  if (solverMode == SolverMode::ExactFbf) {
    state.counters["step_size_scale"]
        = scenario == Scenario::CardHouseFourLevelReduced
                  || scenario
                         == Scenario::CardHouseFourLevelSettleProjectileReduced
              ? kCardHouseStepSizeScale
          : (scenario == Scenario::MasonryArch25Reduced
             || scenario == Scenario::MasonryArch25ProjectileReduced
             || scenario == Scenario::MasonryArch101Reduced
             || scenario == Scenario::MasonryArch25FullManifold
             || scenario == Scenario::MasonryArch101FullManifold)
              ? kArchStepSizeScale
              : kSmallFixtureStepSizeScale;
    state.counters["outer_relaxation"]
        = scenario == Scenario::CardHouseFourLevelReduced
                  || scenario
                         == Scenario::CardHouseFourLevelSettleProjectileReduced
              ? kCardHouseOuterRelaxation
          : (scenario == Scenario::MasonryArch25Reduced
             || scenario == Scenario::MasonryArch25ProjectileReduced
             || scenario == Scenario::MasonryArch101Reduced
             || scenario == Scenario::MasonryArch25FullManifold
             || scenario == Scenario::MasonryArch101FullManifold)
              ? kArchOuterRelaxation
              : 1.0;
  }
  if (std::isfinite(lastMetrics.maxResidual))
    state.counters["max_residual"] = lastMetrics.maxResidual;
}

} // namespace

BENCHMARK_CAPTURE(
    BM_PaperFixtureStepTime,
    incline_mu_0_5_boxed_lcp,
    Scenario::InclineStick,
    SolverMode::BoxedLcp)
    ->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(
    BM_PaperFixtureStepTime,
    incline_mu_0_5_exact_fbf,
    Scenario::InclineStick,
    SolverMode::ExactFbf)
    ->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(
    BM_PaperFixtureStepTime,
    incline_mu_0_4_boxed_lcp,
    Scenario::InclineSlide,
    SolverMode::BoxedLcp)
    ->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(
    BM_PaperFixtureStepTime,
    incline_mu_0_4_exact_fbf,
    Scenario::InclineSlide,
    SolverMode::ExactFbf)
    ->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(
    BM_PaperFixtureStepTime,
    backspin_boxed_lcp,
    Scenario::Backspin,
    SolverMode::BoxedLcp)
    ->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(
    BM_PaperFixtureStepTime,
    backspin_exact_fbf,
    Scenario::Backspin,
    SolverMode::ExactFbf)
    ->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(
    BM_PaperFixtureStepTime,
    turntable_mu_0_2_omega_2_boxed_lcp,
    Scenario::TurntableLowSlow,
    SolverMode::BoxedLcp)
    ->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(
    BM_PaperFixtureStepTime,
    turntable_mu_0_2_omega_2_exact_fbf,
    Scenario::TurntableLowSlow,
    SolverMode::ExactFbf)
    ->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(
    BM_PaperFixtureStepTime,
    turntable_mu_0_2_omega_5_boxed_lcp,
    Scenario::TurntableLowFast,
    SolverMode::BoxedLcp)
    ->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(
    BM_PaperFixtureStepTime,
    turntable_mu_0_2_omega_5_exact_fbf,
    Scenario::TurntableLowFast,
    SolverMode::ExactFbf)
    ->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(
    BM_PaperFixtureStepTime,
    turntable_mu_0_5_omega_2_boxed_lcp,
    Scenario::TurntableHighSlow,
    SolverMode::BoxedLcp)
    ->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(
    BM_PaperFixtureStepTime,
    turntable_mu_0_5_omega_2_exact_fbf,
    Scenario::TurntableHighSlow,
    SolverMode::ExactFbf)
    ->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(
    BM_PaperFixtureStepTime,
    turntable_mu_0_5_omega_5_boxed_lcp,
    Scenario::TurntableHighFast,
    SolverMode::BoxedLcp)
    ->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(
    BM_PaperFixtureStepTime,
    turntable_mu_0_5_omega_5_exact_fbf,
    Scenario::TurntableHighFast,
    SolverMode::ExactFbf)
    ->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(
    BM_PaperFixtureStepTime,
    painleve_mu_0_5_boxed_lcp,
    Scenario::PainleveSlide,
    SolverMode::BoxedLcp)
    ->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(
    BM_PaperFixtureStepTime,
    painleve_mu_0_5_exact_fbf,
    Scenario::PainleveSlide,
    SolverMode::ExactFbf)
    ->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(
    BM_PaperFixtureStepTime,
    painleve_mu_0_55_boxed_lcp,
    Scenario::PainleveTumble,
    SolverMode::BoxedLcp)
    ->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(
    BM_PaperFixtureStepTime,
    painleve_mu_0_55_exact_fbf,
    Scenario::PainleveTumble,
    SolverMode::ExactFbf)
    ->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(
    BM_PaperFixtureStepTime,
    card_house_26_reduced_contact_boxed_lcp,
    Scenario::CardHouseFourLevelReduced,
    SolverMode::BoxedLcp)
    ->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(
    BM_PaperFixtureStepTime,
    card_house_26_reduced_contact_exact_fbf,
    Scenario::CardHouseFourLevelReduced,
    SolverMode::ExactFbf)
    ->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(
    BM_PaperFixtureStepTime,
    card_house_26_settle_projectile_reduced_contact_boxed_lcp,
    Scenario::CardHouseFourLevelSettleProjectileReduced,
    SolverMode::BoxedLcp)
    ->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(
    BM_PaperFixtureStepTime,
    card_house_26_settle_projectile_reduced_contact_exact_fbf,
    Scenario::CardHouseFourLevelSettleProjectileReduced,
    SolverMode::ExactFbf)
    ->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(
    BM_PaperFixtureStepTime,
    card_house_10_construction_boxed_lcp,
    Scenario::CardHouseTenLevelConstruction,
    SolverMode::BoxedLcp)
    ->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(
    BM_PaperFixtureStepTime,
    masonry_arch_25_reduced_contact_boxed_lcp,
    Scenario::MasonryArch25Reduced,
    SolverMode::BoxedLcp)
    ->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(
    BM_PaperFixtureStepTime,
    masonry_arch_25_reduced_contact_exact_fbf,
    Scenario::MasonryArch25Reduced,
    SolverMode::ExactFbf)
    ->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(
    BM_PaperFixtureStepTime,
    masonry_arch_25_projectile_reduced_contact_boxed_lcp,
    Scenario::MasonryArch25ProjectileReduced,
    SolverMode::BoxedLcp)
    ->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(
    BM_PaperFixtureStepTime,
    masonry_arch_25_projectile_reduced_contact_exact_fbf,
    Scenario::MasonryArch25ProjectileReduced,
    SolverMode::ExactFbf)
    ->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(
    BM_PaperFixtureStepTime,
    masonry_arch_101_reduced_contact_boxed_lcp,
    Scenario::MasonryArch101Reduced,
    SolverMode::BoxedLcp)
    ->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(
    BM_PaperFixtureStepTime,
    masonry_arch_101_reduced_contact_exact_fbf,
    Scenario::MasonryArch101Reduced,
    SolverMode::ExactFbf)
    ->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(
    BM_PaperFixtureStepTime,
    masonry_arch_25_full_manifold_boxed_lcp,
    Scenario::MasonryArch25FullManifold,
    SolverMode::BoxedLcp)
    ->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(
    BM_PaperFixtureStepTime,
    masonry_arch_25_full_manifold_exact_fbf,
    Scenario::MasonryArch25FullManifold,
    SolverMode::ExactFbf)
    ->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(
    BM_PaperFixtureStepTime,
    masonry_arch_101_full_manifold_boxed_lcp,
    Scenario::MasonryArch101FullManifold,
    SolverMode::BoxedLcp)
    ->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(
    BM_PaperFixtureStepTime,
    masonry_arch_101_full_manifold_exact_fbf,
    Scenario::MasonryArch101FullManifold,
    SolverMode::ExactFbf)
    ->Unit(benchmark::kMillisecond);
