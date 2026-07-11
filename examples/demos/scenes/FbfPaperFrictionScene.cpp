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

// Visual counterparts for the currently implemented headless fixtures from
// Song, Fan, Ascher, and Pai, "A Splitting Architecture for Exact Reduced
// Coulomb Friction" (SCA 2026). These scenes are inspection and comparison
// aids; the authoritative parity gates remain the unit/integration tests,
// benchmark, and CSV trace exporter.

#include "Scenes.hpp"

#include <dart/gui/osg/osg.hpp>

#include <dart/simulation/DeactivationOptions.hpp>

#include <dart/constraint/ExactCoulombFbfConstraintSolver.hpp>

#include <dart/collision/dart/DARTCollisionDetector.hpp>

#include <dart/math/detail/MasonryArchGeometry.hpp>

#include <dart/dart.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <memory>
#include <string>

#include <cmath>

namespace dart_demos {

namespace {

using dart::dynamics::BodyNode;
using dart::dynamics::BoxShape;
using dart::dynamics::CollisionAspect;
using dart::dynamics::DynamicsAspect;
using dart::dynamics::FreeJoint;
using dart::dynamics::Skeleton;
using dart::dynamics::SkeletonPtr;
using dart::dynamics::SphereShape;
using dart::dynamics::VisualAspect;
using dart::simulation::WorldPtr;

constexpr double kDt = 1.0 / 60.0;
constexpr double kGravity = 9.81;
constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kInclineTan = 0.5;
constexpr double kBackspinRadius = 0.25;
constexpr double kBackspinLinearVelocity = 4.0;
constexpr double kBackspinAngularVelocity = -200.0;
constexpr double kBackspinFriction = 0.5;
constexpr double kPainleveInitialPitch = 0.08;
constexpr double kPainleveInitialVelocity = 3.6;
constexpr double kCardHouseFriction = 0.8;
constexpr double kCardHouseAngle = 0.23;
constexpr double kCardHouseHeight = 1.0;
constexpr double kCardHouseWidth = 0.45;
constexpr double kCardHouseThickness = 0.03;
constexpr double kCardHouseInitialPenetration = 0.003;
constexpr double kCardHouseFrameSpacing = 0.55;
constexpr double kCardHouseStepSizeScale = 10.0;
constexpr double kCardHouseOuterRelaxation = 1.5;
constexpr std::size_t kCardHouseProjectileCount = 4u;
constexpr double kCardHouseProjectileRadius = 0.055;
constexpr double kCardHouseProjectileMass = 0.02;
constexpr double kCardHouseProjectileSpeed = 4.0;
constexpr double kSmallFixtureStepSizeScale = 2.0;
constexpr std::size_t kCardHouseFourLevelCount = 4u;
constexpr std::size_t kCardHouseTenLevelCount = 10u;
// Full natural manifold for the 26-card one-step scaffold: 512/4 caps
// observe 108 actual contacts and solve clean (~2.5 s/step, zero fallbacks)
// with the card-house step-size-scale/outer-relaxation options below.
constexpr std::size_t kCardHouseReducedMaxContacts = 512u;
constexpr std::size_t kCardHouseReducedMaxContactsPerPair = 4u;
constexpr std::size_t kCardHouseTenLevelConstructionMaxContacts = 512u;
constexpr std::size_t kCardHouseTenLevelConstructionMaxContactsPerPair = 8u;
// Source-faithful Rigid-IPC arch parameters: uniform friction 0.5 and the
// repository's default density (see dart/math/detail/MasonryArchGeometry.hpp
// for the shared weighted-catenary generator and provenance).
constexpr double kArchFriction = 0.5;
constexpr double kArchDensity = 1000.0;
constexpr std::size_t kArchStoneCount = 25u;
constexpr std::size_t kArch101StoneCount = 101u;
// Crown (topmost) centroid height and cross-section width from the
// weighted-catenary generator (fc = 60 cm, sqrt(Qt) = 7 cm); used to aim
// the projectile at the crown regardless of stone count.
constexpr double kArchCrownHeight = 0.60;
constexpr double kArchCrownWidth = 0.07;
constexpr std::size_t kArchReducedMaxContacts = 48u;
constexpr std::size_t kArchReducedMaxContactsPerPair = 2u;
constexpr std::size_t kArch101ReducedMaxContacts = 38u;
constexpr std::size_t kArch101ReducedMaxContactsPerPair = 2u;
constexpr int kArchMaxOuterIterations = 120000;
constexpr double kArchOuterRelaxation = 1.5;
constexpr double kArchStepSizeScale = 10.0;
constexpr double kArchProjectileRadius = 0.08;
constexpr double kArchProjectileMass = 0.05;
constexpr double kArchProjectileSpeed = 3.0;

enum class SolverMode
{
  ExactFbf,
  BoxedLcp,
};

struct FbfPaperState
{
  SolverMode solverMode = SolverMode::ExactFbf;
  double turntableAngularVelocity = 2.0;
  bool cardHouseProjectilesLaunched = false;
  bool masonryArchProjectileLaunched = false;
};

//==============================================================================
const char* solverModeLabel(SolverMode mode)
{
  return mode == SolverMode::ExactFbf ? "Exact FBF" : "Boxed LCP";
}

//==============================================================================
const char* fbfStatusLabel(dart::math::detail::ExactCoulombFbfStatus status)
{
  using dart::math::detail::ExactCoulombFbfStatus;
  switch (status) {
    case ExactCoulombFbfStatus::Success:
      return "success";
    case ExactCoulombFbfStatus::MaxIterations:
      return "max_iterations";
    case ExactCoulombFbfStatus::InvalidInput:
      return "invalid_input";
    case ExactCoulombFbfStatus::InnerSolverFailed:
      return "inner_failed";
    case ExactCoulombFbfStatus::StepSizeUnderflow:
      return "step_underflow";
  }
  return "unknown";
}

//==============================================================================
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

//==============================================================================
dart::constraint::ExactCoulombFbfConstraintSolverOptions makeCardFbfOptions()
{
  auto options = makeFbfOptions();
  options.maxOuterIterations = 30000;
  options.enableWarmStart = false;
  options.stepSizeScale = kCardHouseStepSizeScale;
  options.outerRelaxation = kCardHouseOuterRelaxation;
  return options;
}

//==============================================================================
dart::constraint::ExactCoulombFbfConstraintSolverOptions makeArchFbfOptions()
{
  auto options = makeFbfOptions();
  options.maxOuterIterations = kArchMaxOuterIterations;
  options.stepSizeScale = kArchStepSizeScale;
  options.outerRelaxation = kArchOuterRelaxation;
  options.enableWarmStart = false;
  return options;
}

//==============================================================================
void configureWorldBase(const WorldPtr& world)
{
  world->setTimeStep(kDt);
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -kGravity));
  world->setNumSimulationThreads(1u);

  dart::simulation::DeactivationOptions deactivation;
  deactivation.mEnabled = false;
  world->setDeactivationOptions(deactivation);
}

//==============================================================================
void configureSolver(
    const WorldPtr& world,
    SolverMode mode,
    std::size_t maxContacts,
    std::size_t maxContactsPerPair,
    bool cardBudget = false,
    bool archBudget = false)
{
  if (mode == SolverMode::ExactFbf) {
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
    auto solver
        = std::make_unique<dart::constraint::BoxedLcpConstraintSolver>();
    solver->setCollisionDetector(
        dart::collision::DARTCollisionDetector::create());
    solver->setNumSimulationThreads(1u);
    world->setConstraintSolver(std::move(solver));
  }

  auto& option = world->getConstraintSolver()->getCollisionOption();
  option.maxNumContacts = maxContacts;
  option.maxNumContactsPerPair = maxContactsPerPair;
}

//==============================================================================
void setShapeInertia(BodyNode* body, const dart::dynamics::ShapePtr& shape)
{
  constexpr double density = 1000.0;
  const double mass = density * shape->getVolume();
  dart::dynamics::Inertia inertia;
  inertia.setMass(mass);
  inertia.setMoment(shape->computeInertia(mass));
  body->setInertia(inertia);
}

//==============================================================================
SkeletonPtr createStaticBox(
    const std::string& name,
    const Eigen::Vector3d& size,
    const Eigen::Isometry3d& transform,
    double friction,
    const Eigen::Vector4d& color)
{
  auto skeleton = Skeleton::create(name);
  auto* joint = skeleton->createJointAndBodyNodePair<FreeJoint>(nullptr).first;
  auto* body = skeleton->getBodyNode(0);
  auto shape = std::make_shared<BoxShape>(size);
  auto* node = body->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(shape);
  node->getVisualAspect()->setRGBA(color);
  node->getDynamicsAspect()->setFrictionCoeff(friction);
  joint->setPositions(FreeJoint::convertToPositions(transform));
  skeleton->setMobile(false);
  return skeleton;
}

//==============================================================================
SkeletonPtr createDynamicBox(
    const std::string& name,
    const Eigen::Vector3d& size,
    const Eigen::Isometry3d& transform,
    double friction,
    const Eigen::Vector4d& color,
    const Eigen::Vector3d& linearVelocity = Eigen::Vector3d::Zero())
{
  auto skeleton = Skeleton::create(name);
  auto* joint = skeleton->createJointAndBodyNodePair<FreeJoint>(nullptr).first;
  auto* body = skeleton->getBodyNode(0);
  auto shape = std::make_shared<BoxShape>(size);
  auto* node = body->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(shape);
  node->getVisualAspect()->setRGBA(color);
  node->getDynamicsAspect()->setFrictionCoeff(friction);
  setShapeInertia(body, shape);
  joint->setPositions(FreeJoint::convertToPositions(transform));
  joint->setLinearVelocity(linearVelocity);
  return skeleton;
}

//==============================================================================
SkeletonPtr createDynamicSphere(
    const std::string& name,
    double radius,
    double mass,
    const Eigen::Isometry3d& transform,
    double friction,
    const Eigen::Vector4d& color,
    const Eigen::Vector3d& linearVelocity)
{
  auto skeleton = Skeleton::create(name);
  auto* joint = skeleton->createJointAndBodyNodePair<FreeJoint>(nullptr).first;
  auto* body = skeleton->getBodyNode(0);
  auto shape = std::make_shared<SphereShape>(radius);
  auto* node = body->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(shape);
  node->getVisualAspect()->setRGBA(color);
  node->getDynamicsAspect()->setFrictionCoeff(friction);

  dart::dynamics::Inertia inertia;
  inertia.setMass(mass);
  inertia.setMoment(SphereShape::computeInertia(radius, mass));
  body->setInertia(inertia);

  joint->setPositions(FreeJoint::convertToPositions(transform));
  joint->setLinearVelocity(linearVelocity);
  return skeleton;
}

//==============================================================================
SkeletonPtr createGround(double friction)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation().z() = -0.05;
  return createStaticBox(
      "ground",
      Eigen::Vector3d(8.0, 8.0, 0.1),
      transform,
      friction,
      Eigen::Vector4d(0.45, 0.47, 0.50, 1.0));
}

//==============================================================================
SkeletonPtr createBackspinSphere()
{
  auto skeleton = Skeleton::create("backspin_sphere");
  auto* joint = skeleton->createJointAndBodyNodePair<FreeJoint>(nullptr).first;
  auto* body = skeleton->getBodyNode(0);
  auto shape = std::make_shared<SphereShape>(kBackspinRadius);
  auto* node = body->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(shape);
  node->getVisualAspect()->setRGBA(Eigen::Vector4d(0.92, 0.52, 0.12, 1.0));
  node->getDynamicsAspect()->setFrictionCoeff(kBackspinFriction);
  setShapeInertia(body, shape);

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation().z() = kBackspinRadius - 0.005;
  joint->setPositions(FreeJoint::convertToPositions(transform));
  joint->setLinearVelocity(Eigen::Vector3d(kBackspinLinearVelocity, 0.0, 0.0));
  joint->setAngularVelocity(
      Eigen::Vector3d(0.0, kBackspinAngularVelocity, 0.0));
  return skeleton;
}

//==============================================================================
WorldPtr createInclineWorld(SolverMode mode)
{
  auto world = dart::simulation::World::create("fbf_paper_incline");
  configureWorldBase(world);
  configureSolver(world, mode, 16u, 4u);

  const double theta = std::atan(kInclineTan);
  const Eigen::Matrix3d rotation
      = Eigen::AngleAxisd(-theta, Eigen::Vector3d::UnitY()).toRotationMatrix();
  const Eigen::Vector3d normal = rotation * Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d boardSize(6.0, 1.8, 0.1);
  const Eigen::Vector3d cubeSize = Eigen::Vector3d::Ones();

  for (const auto& cell :
       {std::pair<double, double>{0.5, -1.15},
        std::pair<double, double>{0.4, 1.15}}) {
    const double friction = cell.first;
    const double y = cell.second;
    Eigen::Isometry3d board = Eigen::Isometry3d::Identity();
    board.linear() = rotation;
    board.translation() = Eigen::Vector3d(0.0, y, -0.05);
    world->addSkeleton(createStaticBox(
        friction == 0.5 ? "incline_mu_0_5" : "incline_mu_0_4",
        boardSize,
        board,
        friction,
        friction == 0.5 ? Eigen::Vector4d(0.33, 0.50, 0.70, 1.0)
                        : Eigen::Vector4d(0.55, 0.45, 0.24, 1.0)));

    Eigen::Isometry3d cube = Eigen::Isometry3d::Identity();
    cube.linear() = rotation;
    cube.translation() = board.translation()
                         + normal * (0.5 * cubeSize.z() + 0.05 - 0.01)
                         + rotation * Eigen::Vector3d(1.6, 0.0, 0.0);
    world->addSkeleton(createDynamicBox(
        friction == 0.5 ? "cube_stick_mu_0_5" : "cube_slide_mu_0_4",
        cubeSize,
        cube,
        friction,
        friction == 0.5 ? Eigen::Vector4d(0.15, 0.78, 0.56, 1.0)
                        : Eigen::Vector4d(0.88, 0.37, 0.22, 1.0)));
  }

  return world;
}

//==============================================================================
WorldPtr createBackspinWorld(SolverMode mode)
{
  auto world = dart::simulation::World::create("fbf_paper_backspin");
  configureWorldBase(world);
  configureSolver(world, mode, 1u, 1u);
  world->addSkeleton(createGround(kBackspinFriction));
  world->addSkeleton(createBackspinSphere());
  return world;
}

//==============================================================================
WorldPtr createTurntableWorld(
    SolverMode mode, const std::shared_ptr<FbfPaperState>& state)
{
  auto world = dart::simulation::World::create("fbf_paper_turntable");
  configureWorldBase(world);
  configureSolver(world, mode, 4u, 4u);

  Eigen::Isometry3d turntable = Eigen::Isometry3d::Identity();
  turntable.translation().z() = -0.05;
  world->addSkeleton(createStaticBox(
      "turntable",
      Eigen::Vector3d(4.0, 4.0, 0.1),
      turntable,
      0.5,
      Eigen::Vector4d(0.28, 0.30, 0.34, 1.0)));

  Eigen::Isometry3d rider = Eigen::Isometry3d::Identity();
  rider.translation() = Eigen::Vector3d(1.0, 0.0, 0.125 - 0.005);
  world->addSkeleton(createDynamicBox(
      "turntable_rider",
      Eigen::Vector3d::Constant(0.25),
      rider,
      0.5,
      Eigen::Vector4d(0.20, 0.63, 0.91, 1.0)));

  state->turntableAngularVelocity = 2.0;
  return world;
}

//==============================================================================
WorldPtr createPainleveWorld(SolverMode mode)
{
  auto world = dart::simulation::World::create("fbf_paper_painleve_proxy");
  configureWorldBase(world);
  configureSolver(world, mode, 4u, 4u);
  world->addSkeleton(createGround(0.5));

  const Eigen::Vector3d size(0.6, 0.6, 1.0);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear()
      = Eigen::AngleAxisd(kPainleveInitialPitch, Eigen::Vector3d::UnitY())
            .toRotationMatrix();
  const double verticalHalfExtent
      = 0.5
        * (std::abs(transform.linear()(2, 0)) * size.x()
           + std::abs(transform.linear()(2, 1)) * size.y()
           + std::abs(transform.linear()(2, 2)) * size.z());
  transform.translation().z() = verticalHalfExtent - 0.005;
  world->addSkeleton(createDynamicBox(
      "painleve_proxy_box",
      size,
      transform,
      0.5,
      Eigen::Vector4d(0.90, 0.58, 0.12, 1.0),
      Eigen::Vector3d(kPainleveInitialVelocity, 0.0, 0.0)));
  return world;
}

//==============================================================================
double computeCardVerticalHalfExtent(const Eigen::Matrix3d& rotation)
{
  const Eigen::Vector3d size(
      kCardHouseThickness, kCardHouseWidth, kCardHouseHeight);
  return 0.5
         * (std::abs(rotation(2, 0)) * size.x()
            + std::abs(rotation(2, 1)) * size.y()
            + std::abs(rotation(2, 2)) * size.z());
}

//==============================================================================
Eigen::Isometry3d createCardAFrameTransform(
    double centerX, double baseZ, bool leftCard)
{
  const double angle = leftCard ? kCardHouseAngle : -kCardHouseAngle;
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear()
      = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY()).toRotationMatrix();
  const double halfSpan = 0.5 * kCardHouseHeight * std::sin(kCardHouseAngle);
  transform.translation().x()
      = centerX + (leftCard ? -0.96 * halfSpan : 0.96 * halfSpan);
  transform.translation().y() = 0.0;
  transform.translation().z()
      = baseZ + computeCardVerticalHalfExtent(transform.linear())
        - kCardHouseInitialPenetration;
  return transform;
}

//==============================================================================
Eigen::Isometry3d createCardAFrameTransform(bool leftCard)
{
  return createCardAFrameTransform(0.0, 0.0, leftCard);
}

//==============================================================================
Eigen::Isometry3d createCardHorizontalSupportTransform(
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

//==============================================================================
SkeletonPtr createCard(
    const std::string& name,
    const Eigen::Isometry3d& transform,
    bool leftCard,
    bool mobile)
{
  const Eigen::Vector3d size(
      kCardHouseThickness, kCardHouseWidth, kCardHouseHeight);
  const Eigen::Vector4d color = leftCard
                                    ? Eigen::Vector4d(0.83, 0.33, 0.28, 1.0)
                                    : Eigen::Vector4d(0.24, 0.58, 0.82, 1.0);
  if (!mobile) {
    return createStaticBox(name, size, transform, kCardHouseFriction, color);
  }

  constexpr double kCardMass = 0.05;
  auto card
      = createDynamicBox(name, size, transform, kCardHouseFriction, color);
  auto* body = card->getBodyNode(0);
  dart::dynamics::Inertia inertia;
  inertia.setMass(kCardMass);
  inertia.setMoment(BoxShape::computeInertia(size, kCardMass));
  body->setInertia(inertia);
  return card;
}

//==============================================================================
SkeletonPtr createAFrameCard(const std::string& name, bool leftCard)
{
  return createCard(name, createCardAFrameTransform(leftCard), leftCard, true);
}

//==============================================================================
WorldPtr createCardAFrameWorld(SolverMode mode)
{
  auto world = dart::simulation::World::create("fbf_paper_card_aframe");
  configureWorldBase(world);
  configureSolver(world, mode, 32u, 8u, true);
  world->addSkeleton(createGround(kCardHouseFriction));
  world->addSkeleton(createAFrameCard("left_card", true));
  world->addSkeleton(createAFrameCard("right_card", false));
  return world;
}

//==============================================================================
WorldPtr createCardHouseWorld(
    const std::string& name,
    std::size_t levelCount,
    SolverMode mode,
    std::size_t maxContacts,
    std::size_t maxContactsPerPair,
    bool cardBudget,
    bool mobileCards)
{
  auto world = dart::simulation::World::create(name);
  configureWorldBase(world);
  configureSolver(world, mode, maxContacts, maxContactsPerPair, cardBudget);
  world->addSkeleton(createGround(kCardHouseFriction));

  const Eigen::Matrix3d aFrameRotation
      = Eigen::AngleAxisd(kCardHouseAngle, Eigen::Vector3d::UnitY())
            .toRotationMatrix();
  const double aFrameHeight
      = 2.0 * computeCardVerticalHalfExtent(aFrameRotation);
  double baseZ = 0.0;

  for (std::size_t level = 0u; level < levelCount; ++level) {
    const std::size_t frameCount = levelCount - level;
    for (std::size_t frame = 0u; frame < frameCount; ++frame) {
      const double centerX
          = (static_cast<double>(frame) - 0.5 * (frameCount - 1))
            * kCardHouseFrameSpacing;
      world->addSkeleton(createCard(
          "card_house_l" + std::to_string(level) + "_f" + std::to_string(frame)
              + "_left",
          createCardAFrameTransform(centerX, baseZ, true),
          true,
          mobileCards));
      world->addSkeleton(createCard(
          "card_house_l" + std::to_string(level) + "_f" + std::to_string(frame)
              + "_right",
          createCardAFrameTransform(centerX, baseZ, false),
          false,
          mobileCards));
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
      world->addSkeleton(createCard(
          "card_house_l" + std::to_string(level) + "_support"
              + std::to_string(support),
          createCardHorizontalSupportTransform(centerX, supportBaseZ),
          true,
          mobileCards));
    }

    baseZ = supportBaseZ + kCardHouseThickness - kCardHouseInitialPenetration;
  }

  return world;
}

WorldPtr createCardHouseFourLevelReducedWorld(SolverMode mode)
{
  return createCardHouseWorld(
      "fbf_paper_card_house_26",
      kCardHouseFourLevelCount,
      mode,
      kCardHouseReducedMaxContacts,
      kCardHouseReducedMaxContactsPerPair,
      true,
      true);
}

//==============================================================================
SkeletonPtr createCardHouseProjectile(std::size_t index)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(
      -1.35,
      (static_cast<double>(index) - 1.5) * 0.15,
      0.35 + 0.28 * static_cast<double>(index));
  return createDynamicSphere(
      "fbf_projectile_" + std::to_string(index),
      kCardHouseProjectileRadius,
      kCardHouseProjectileMass,
      transform,
      kCardHouseFriction,
      Eigen::Vector4d(0.95, 0.74, 0.18, 1.0),
      Eigen::Vector3d(kCardHouseProjectileSpeed, 0.0, 0.0));
}

//==============================================================================
std::size_t countCardHouseProjectiles(const WorldPtr& world)
{
  std::size_t count = 0u;
  for (std::size_t i = 0u; i < world->getNumSkeletons(); ++i) {
    const auto skeleton = world->getSkeleton(i);
    if (skeleton != nullptr
        && skeleton->getName().rfind("fbf_projectile_", 0u) == 0u) {
      ++count;
    }
  }
  return count;
}

//==============================================================================
void launchCardHouseProjectiles(
    const WorldPtr& world, const std::shared_ptr<FbfPaperState>& state)
{
  if (countCardHouseProjectiles(world) > 0u) {
    state->cardHouseProjectilesLaunched = true;
    return;
  }

  for (std::size_t i = 0u; i < kCardHouseProjectileCount; ++i)
    world->addSkeleton(createCardHouseProjectile(i));
  state->cardHouseProjectilesLaunched = true;
}

WorldPtr createCardHouseTenLevelConstructionWorld(SolverMode mode)
{
  return createCardHouseWorld(
      "fbf_paper_card_house_10",
      kCardHouseTenLevelCount,
      mode,
      kCardHouseTenLevelConstructionMaxContacts,
      kCardHouseTenLevelConstructionMaxContactsPerPair,
      false,
      false);
}

//==============================================================================
SkeletonPtr createMasonryArchStone(
    std::size_t index,
    const dart::math::detail::MasonryArchStoneBoxGeometry& geometry)
{
  const double mass = kArchDensity * geometry.size.x() * geometry.size.y()
                      * geometry.size.z();
  auto stone = createDynamicBox(
      "masonry_arch_stone_" + std::to_string(index),
      geometry.size,
      geometry.transform,
      kArchFriction,
      Eigen::Vector4d(0.62, 0.58, 0.52, 1.0));
  auto* body = stone->getBodyNode(0);
  dart::dynamics::Inertia inertia;
  inertia.setMass(mass);
  inertia.setMoment(BoxShape::computeInertia(geometry.size, mass));
  body->setInertia(inertia);
  return stone;
}

//==============================================================================
WorldPtr createMasonryArchReducedWorld(
    SolverMode mode,
    const std::string& name,
    std::size_t stoneCount,
    std::size_t maxContacts,
    std::size_t maxContactsPerPair)
{
  auto world = dart::simulation::World::create(name);
  configureWorldBase(world);
  configureSolver(world, mode, maxContacts, maxContactsPerPair, false, true);

  // Author boundary condition from the Rigid-IPC arch scenes: only the
  // ground is fixed; every stone (including both springers) is dynamic.
  world->addSkeleton(createGround(kArchFriction));
  const auto stoneGeometry
      = dart::math::detail::generateMasonryArchStoneBoxes(stoneCount);
  for (std::size_t i = 0u; i < stoneCount; ++i) {
    world->addSkeleton(createMasonryArchStone(i, stoneGeometry[i]));
  }

  return world;
}

WorldPtr createMasonryArch25ReducedWorld(SolverMode mode)
{
  return createMasonryArchReducedWorld(
      mode,
      "fbf_paper_masonry_arch_25",
      kArchStoneCount,
      kArchReducedMaxContacts,
      kArchReducedMaxContactsPerPair);
}

WorldPtr createMasonryArch101ReducedWorld(SolverMode mode)
{
  return createMasonryArchReducedWorld(
      mode,
      "fbf_paper_masonry_arch_101",
      kArch101StoneCount,
      kArch101ReducedMaxContacts,
      kArch101ReducedMaxContactsPerPair);
}

//==============================================================================
SkeletonPtr createMasonryArchProjectile()
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(
      0.0,
      -0.5 * kArchCrownWidth - kArchProjectileRadius + 0.005,
      kArchCrownHeight);
  return createDynamicSphere(
      "masonry_arch_projectile",
      kArchProjectileRadius,
      kArchProjectileMass,
      transform,
      kArchFriction,
      Eigen::Vector4d(0.96, 0.67, 0.18, 1.0),
      Eigen::Vector3d(0.0, kArchProjectileSpeed, 0.0));
}

//==============================================================================
void launchMasonryArchProjectile(
    const WorldPtr& world, const std::shared_ptr<FbfPaperState>& state)
{
  if (world->getSkeleton("masonry_arch_projectile") == nullptr)
    world->addSkeleton(createMasonryArchProjectile());

  state->masonryArchProjectileLaunched = true;
}

//==============================================================================
void renderSolverControls(
    const WorldPtr& world,
    const std::shared_ptr<FbfPaperState>& state,
    std::size_t maxContacts,
    std::size_t maxContactsPerPair,
    bool cardBudget = false,
    bool archBudget = false,
    bool exactFbfAvailable = true)
{
  ImGui::TextDisabled("Solver");
  int solverIndex = exactFbfAvailable
                        ? (state->solverMode == SolverMode::ExactFbf ? 0 : 1)
                        : 0;
  bool clicked = false;
  if (exactFbfAvailable) {
    clicked |= ImGui::RadioButton("Exact FBF", &solverIndex, 0);
    ImGui::SameLine();
    clicked |= ImGui::RadioButton("Boxed LCP", &solverIndex, 1);
  } else {
    ImGui::TextUnformatted(
        "Exact FBF: unavailable for this construction-only scene");
    clicked |= ImGui::RadioButton("Boxed LCP", &solverIndex, 0);
  }
  if (clicked) {
    const SolverMode newMode
        = exactFbfAvailable
              ? (solverIndex == 0 ? SolverMode::ExactFbf : SolverMode::BoxedLcp)
              : SolverMode::BoxedLcp;
    if (newMode != state->solverMode) {
      state->solverMode = newMode;
      configureSolver(
          world,
          state->solverMode,
          maxContacts,
          maxContactsPerPair,
          cardBudget,
          archBudget);
    }
  }

  ImGui::Text("Mode: %s", solverModeLabel(state->solverMode));
  ImGui::Text("Time: %.3f", world->getTime());
  const auto contacts
      = world->getConstraintSolver()->getLastCollisionResult().getNumContacts();
  ImGui::Text("Last contacts: %zu", contacts);
  ImGui::Separator();
  ImGui::TextDisabled("Diagnostics");

  const auto* exactSolver
      = dynamic_cast<const dart::constraint::ExactCoulombFbfConstraintSolver*>(
          world->getConstraintSolver());
  if (exactSolver == nullptr) {
    ImGui::TextUnformatted("Exact diagnostics: boxed-LCP mode");
    return;
  }

  if (exactSolver->getNumExactCoulombSolves() == 0u
      && exactSolver->getNumExactCoulombFailures() == 0u
      && exactSolver->getNumBoxedLcpFallbacks() == 0u) {
    ImGui::TextUnformatted("Exact diagnostics: not run yet");
    return;
  }

  ImGui::Text(
      "FBF status: %s",
      fbfStatusLabel(exactSolver->getLastExactCoulombFbfStatus()));
  ImGui::Text("Residual: %.3e", exactSolver->getLastExactCoulombResidual());
  ImGui::Text(
      "Iterations: last %d  max %d",
      exactSolver->getLastExactCoulombIterations(),
      exactSolver->getMaxExactCoulombIterations());
  ImGui::Text(
      "Total iterations: %zu", exactSolver->getTotalExactCoulombIterations());
  ImGui::Text(
      "Gamma: %.3e  safe %.3e",
      exactSolver->getLastExactCoulombStepSize(),
      exactSolver->getLastExactCoulombSafeStepSize());
  ImGui::Text(
      "Gamma scale: %.2f", exactSolver->getExactCoulombOptions().stepSizeScale);
  ImGui::Text(
      "Outer relaxation: %.2f",
      exactSolver->getExactCoulombOptions().outerRelaxation);
  ImGui::Text(
      "Shrink: %d  coupling: %.3e",
      exactSolver->getLastExactCoulombShrinkIterations(),
      exactSolver->getLastExactCoulombCouplingVariationRatio());
  ImGui::Text(
      "Solves: %zu  Failures: %zu  Fallbacks: %zu",
      exactSolver->getNumExactCoulombSolves(),
      exactSolver->getNumExactCoulombFailures(),
      exactSolver->getNumBoxedLcpFallbacks());
  if (exactSolver->getNumExactCoulombFailures() > 0u) {
    ImGui::Text(
        "Last failure: %s  iter %d",
        fbfStatusLabel(exactSolver->getLastFailedExactCoulombFbfStatus()),
        exactSolver->getLastFailedExactCoulombIterations());
    ImGui::Text(
        "Failure residual: %.3e",
        exactSolver->getLastFailedExactCoulombResidual());
    ImGui::Text(
        "Failure gamma: %.3e  shrink %d",
        exactSolver->getLastFailedExactCoulombStepSize(),
        exactSolver->getLastFailedExactCoulombShrinkIterations());
  }
  ImGui::Text(
      "Warm starts: %zu  PG retries: %zu",
      exactSolver->getNumExactCoulombWarmStarts(),
      exactSolver->getNumExactCoulombProjectedGradientRetries());
}

//==============================================================================
void renderCardHousePhaseControls(
    const WorldPtr& world, const std::shared_ptr<FbfPaperState>& state)
{
  ImGui::Separator();
  ImGui::TextDisabled("Phase scaffold");
  const std::size_t projectileCount = countCardHouseProjectiles(world);
  state->cardHouseProjectilesLaunched = projectileCount > 0u;
  ImGui::Text(
      "Projectiles: %zu/%zu", projectileCount, kCardHouseProjectileCount);
  ImGui::Text("Trace phase: one settle step, then four incoming projectiles");
  if (ImGui::Button("Launch 4 projectiles")) {
    launchCardHouseProjectiles(world, state);
  }
  ImGui::TextDisabled("Use Reset to rebuild the unlaunched scaffold");
}

//==============================================================================
void renderMasonryArchProjectileControls(
    const WorldPtr& world, const std::shared_ptr<FbfPaperState>& state)
{
  ImGui::Separator();
  ImGui::TextDisabled("Projectile scaffold");
  state->masonryArchProjectileLaunched
      = world->getSkeleton("masonry_arch_projectile") != nullptr;
  ImGui::Text(
      "Projectile: %s",
      state->masonryArchProjectileLaunched ? "launched" : "not launched");
  ImGui::TextUnformatted("Reduced sphere aimed through the crown stone.");
  if (ImGui::Button("Launch projectile")) {
    launchMasonryArchProjectile(world, state);
  }
  ImGui::TextDisabled("Use Reset to rebuild the unlaunched scaffold");
}

//==============================================================================
DemoScene makeFbfPaperScene(
    const std::string& id,
    const std::string& title,
    const std::string& summary,
    const CameraHome& camera,
    const std::function<WorldPtr(const std::shared_ptr<FbfPaperState>&)>&
        makeWorld,
    std::size_t maxContacts,
    std::size_t maxContactsPerPair,
    bool cardBudget = false,
    bool archBudget = false,
    const char* overview = nullptr,
    const char* expected = nullptr,
    const char* coverage = nullptr,
    bool exactFbfAvailable = true,
    SolverMode initialSolverMode = SolverMode::ExactFbf,
    const std::function<
        void(const WorldPtr&, const std::shared_ptr<FbfPaperState>&)>&
        renderExtraPanel
    = nullptr,
    const std::function<void(
        DemoSceneSetup&,
        const WorldPtr&,
        const std::shared_ptr<FbfPaperState>&)>& configureExtraSetup
    = nullptr)
{
  DemoScene scene;
  scene.id = id;
  scene.title = title;
  scene.category = "Research";
  scene.summary = summary;
  scene.scenePanelDocumentation = ScenePanelDocumentation{
      overview == nullptr ? "" : overview,
      expected == nullptr ? "" : expected,
      coverage == nullptr ? "" : coverage};

  scene.factory = [=] {
    auto state = std::make_shared<FbfPaperState>();
    state->solverMode
        = exactFbfAvailable ? initialSolverMode : SolverMode::BoxedLcp;
    auto world = makeWorld(state);

    DemoSceneSetup setup;
    setup.world = world;
    setup.cameraHome = camera;
    setup.renderPanel = [world,
                         state,
                         maxContacts,
                         maxContactsPerPair,
                         cardBudget,
                         archBudget,
                         exactFbfAvailable,
                         renderExtraPanel] {
      renderSolverControls(
          world,
          state,
          maxContacts,
          maxContactsPerPair,
          cardBudget,
          archBudget,
          exactFbfAvailable);
      if (renderExtraPanel)
        renderExtraPanel(world, state);
    };
    if (exactFbfAvailable) {
      setup.keyActions.push_back(KeyAction{
          'e',
          "Toggle exact/boxed",
          [world,
           state,
           maxContacts,
           maxContactsPerPair,
           cardBudget,
           archBudget] {
            state->solverMode = state->solverMode == SolverMode::ExactFbf
                                    ? SolverMode::BoxedLcp
                                    : SolverMode::ExactFbf;
            configureSolver(
                world,
                state->solverMode,
                maxContacts,
                maxContactsPerPair,
                cardBudget,
                archBudget);
          }});
    }
    if (configureExtraSetup)
      configureExtraSetup(setup, world, state);
    return setup;
  };
  return scene;
}

} // namespace

//==============================================================================
DemoScene makeFbfPaperInclineScene()
{
  return makeFbfPaperScene(
      "fbf_paper_incline",
      "FBF Paper: Incline",
      "Side-by-side incline stick/slide threshold fixture from the exact "
      "Coulomb friction paper.",
      CameraHome{
          ::osg::Vec3d(5.0, -6.5, 3.2),
          ::osg::Vec3d(0.0, 0.0, 0.3),
          ::osg::Vec3d(0.0, 0.0, 1.0)},
      [](const auto& state) { return createInclineWorld(state->solverMode); },
      16u,
      4u,
      false,
      false,
      "Cube-on-incline threshold fixture from the exact reduced Coulomb "
      "friction paper. The slope has tan(theta)=0.5.",
      "With the exact-FBF solver, the high-friction cube should remain near "
      "the threshold while the lower-friction comparison slides down the "
      "incline in the headless regression.",
      "This GUI is a visual counterpart to the implemented state/residual "
      "regression. Full paper snapshot sweeps and external baselines remain "
      "outside this scene.");
}

//==============================================================================
DemoScene makeFbfPaperBackspinScene()
{
  return makeFbfPaperScene(
      "fbf_paper_backspin",
      "FBF Paper: Backspin",
      "Backspin sphere fixture with exact-FBF and boxed-LCP diagnostics.",
      CameraHome{
          ::osg::Vec3d(4.0, -4.5, 2.1),
          ::osg::Vec3d(0.0, 0.0, 0.25),
          ::osg::Vec3d(0.0, 0.0, 1.0)},
      [](const auto& state) { return createBackspinWorld(state->solverMode); },
      1u,
      1u,
      false,
      false,
      "Backspin sphere fixture from the paper: a sphere starts with forward "
      "linear velocity and strong reverse spin on a rough plane.",
      "The exact-FBF run should reverse direction and approach the analytical "
      "rolling state checked by the headless test.",
      "This scene exposes the implemented single-contact fixture and solver "
      "diagnostics. Paper trajectory plots and external comparisons are still "
      "reported as missing.");
}

//==============================================================================
DemoScene makeFbfPaperTurntableScene()
{
  const char* overview
      = "Rotating turntable capture/ejection fixture from the paper's "
        "parameter grid.";
  const char* expected
      = "Low friction should eject the rider. With higher friction and the "
        "slower angular speed, the exact-FBF headless regression classifies "
        "the rider as captured.";
  const char* coverage
      = "The GUI exposes the small implemented grid with a live angular "
        "speed control. Full radial trajectory and snapshot parity remain "
        "missing.";

  DemoScene scene;
  scene.id = "fbf_paper_turntable";
  scene.title = "FBF Paper: Turntable";
  scene.category = "Research";
  scene.summary
      = "Rotating turntable capture/ejection fixture with live angular-speed "
        "control.";
  scene.scenePanelDocumentation
      = ScenePanelDocumentation{overview, expected, coverage};

  scene.factory = [] {
    auto state = std::make_shared<FbfPaperState>();
    state->turntableAngularVelocity = 2.0;
    auto world = createTurntableWorld(state->solverMode, state);

    DemoSceneSetup setup;
    setup.world = world;
    setup.cameraHome = CameraHome{
        ::osg::Vec3d(4.2, -5.0, 3.0),
        ::osg::Vec3d(0.0, 0.0, 0.15),
        ::osg::Vec3d(0.0, 0.0, 1.0)};
    setup.preStep = [world, state] {
      const auto turntable = world->getSkeleton("turntable");
      auto* joint = dynamic_cast<FreeJoint*>(turntable->getJoint(0));
      if (joint != nullptr) {
        const double ramp = std::min(world->getTime(), 1.0);
        joint->setAngularVelocity(
            Eigen::Vector3d(0.0, 0.0, ramp * state->turntableAngularVelocity));
      }
    };
    setup.renderPanel = [world, state] {
      renderSolverControls(world, state, 4u, 4u, false, false, true);
      double omega = state->turntableAngularVelocity;
      const double minOmega = 0.0;
      const double maxOmega = 5.0;
      if (ImGui::SliderScalar(
              "omega",
              ImGuiDataType_Double,
              &omega,
              &minOmega,
              &maxOmega,
              "%.2f",
              ImGuiSliderFlags_AlwaysClamp)) {
        state->turntableAngularVelocity = std::clamp(omega, 0.0, 5.0);
      }
    };
    setup.keyActions.push_back(KeyAction{
        'e', "Toggle exact/boxed", [world, state] {
          state->solverMode = state->solverMode == SolverMode::ExactFbf
                                  ? SolverMode::BoxedLcp
                                  : SolverMode::ExactFbf;
          configureSolver(world, state->solverMode, 4u, 4u);
        }});
    return setup;
  };
  return scene;
}

//==============================================================================
DemoScene makeFbfPaperPainleveScene()
{
  return makeFbfPaperScene(
      "fbf_paper_painleve",
      "FBF Paper: Painleve Proxy",
      "DART-side Painleve-style proxy pending the authors' exact scene files.",
      CameraHome{
          ::osg::Vec3d(4.5, -4.0, 2.4),
          ::osg::Vec3d(0.8, 0.0, 0.35),
          ::osg::Vec3d(0.0, 0.0, 1.0)},
      [](const auto& state) { return createPainleveWorld(state->solverMode); },
      4u,
      4u,
      false,
      false,
      "Painleve-style sliding box proxy. The paper gives qualitative outcomes "
      "but the authors' exact scene files are not available here.",
      "The implemented proxy checks an upright slide/rest case and a higher "
      "friction tumble case in headless tests.",
      "This scene is explicitly not author-scene parity. Replace or "
      "corroborate it if the paper implementation or scene parameters become "
      "available.");
}

//==============================================================================
DemoScene makeFbfPaperCardAFrameScene()
{
  return makeFbfPaperScene(
      "fbf_paper_card_aframe",
      "FBF Paper: Card A-Frame",
      "Two-card A-frame precursor for the paper's house-of-cards scene.",
      CameraHome{
          ::osg::Vec3d(2.3, -3.2, 1.8),
          ::osg::Vec3d(0.0, 0.0, 0.45),
          ::osg::Vec3d(0.0, 0.0, 1.0)},
      [](const auto& state) {
        return createCardAFrameWorld(state->solverMode);
      },
      32u,
      8u,
      true,
      false,
      "Two-card A-frame precursor for the paper's house-of-cards fixture.",
      "The exact-FBF headless test should keep the A-frame standing at the "
      "paper residual tolerance without boxed-LCP fallback.",
      "This is a contact-rich precursor only. It does not replace the full "
      "26-card settle/projectile scene.");
}

//==============================================================================
DemoScene makeFbfPaperCardHouse26Scene()
{
  return makeFbfPaperScene(
      "fbf_paper_card_house_26",
      "FBF Paper: Card House 26",
      "Full natural manifold (108 contacts at the current base) dynamic "
      "26-card scaffold for the paper's four-level house-of-cards scene.",
      CameraHome{
          ::osg::Vec3d(2.8, -4.5, 2.4),
          ::osg::Vec3d(0.0, 0.0, 0.9),
          ::osg::Vec3d(0.0, 0.0, 1.0)},
      [](const auto& state) {
        return createCardHouseFourLevelReducedWorld(state->solverMode);
      },
      kCardHouseReducedMaxContacts,
      kCardHouseReducedMaxContactsPerPair,
      true,
      false,
      "Full natural manifold (108 contacts at the current base) dynamic "
      "26-card scaffold for the paper's four-level house-of-cards scene.",
      "The current exact-FBF smoke runs one bounded step at the full "
      "natural manifold (512-contact cap, 4 contacts per pair, 108 actual "
      "contacts) and should report zero boxed-LCP fallback in roughly 2.5 s "
      "per step. A separate phase scaffold can launch four incoming "
      "projectiles after the first settle step; the matching CSV trace "
      "records initial, settle, and projectile rows.",
      "This is not Fig. 6 parity. The 6.7 s no-creep settle, impact "
      "outcomes, timing, and dynamic snapshots are still missing.",
      true,
      SolverMode::ExactFbf,
      renderCardHousePhaseControls,
      [](DemoSceneSetup& setup,
         const WorldPtr& world,
         const std::shared_ptr<FbfPaperState>& state) {
        setup.keyActions.push_back(
            KeyAction{'p', "Launch 4 projectiles", [world, state] {
                        launchCardHouseProjectiles(world, state);
                      }});
      });
}

//==============================================================================
DemoScene makeFbfPaperCardHouse10Scene()
{
  return makeFbfPaperScene(
      "fbf_paper_card_house_10",
      "FBF Paper: Card House 10",
      "Construction-only 10-level card-house scaffold for the paper's GPU "
      "comparison scene.",
      CameraHome{
          ::osg::Vec3d(5.0, -7.5, 5.0),
          ::osg::Vec3d(0.0, 0.0, 2.4),
          ::osg::Vec3d(0.0, 0.0, 1.0)},
      [](const auto& state) {
        return createCardHouseTenLevelConstructionWorld(state->solverMode);
      },
      kCardHouseTenLevelConstructionMaxContacts,
      kCardHouseTenLevelConstructionMaxContactsPerPair,
      false,
      false,
      "Construction-only 10-level card-house scaffold with 155 cards. The "
      "cards "
      "are static in this GUI scene so the shape can be inspected without "
      "triggering an unsupported contact-rich exact-FBF solve.",
      "The expected result is a stable visual scaffold and boxed-LCP "
      "diagnostics "
      "only. Exact-FBF dynamics are intentionally unavailable for this scene.",
      "This is not the paper's dynamic 10-level benchmark. Exact-FBF outcome, "
      "residual traces, external baselines, timing, and snapshots remain "
      "missing.",
      false,
      SolverMode::BoxedLcp);
}

//==============================================================================
DemoScene makeFbfPaperMasonryArch25Scene()
{
  return makeFbfPaperScene(
      "fbf_paper_masonry_arch_25",
      "FBF Paper: Masonry Arch 25",
      "Reduced-contact 25-stone masonry-arch scaffold for the paper's arch "
      "scene.",
      CameraHome{
          ::osg::Vec3d(3.0, -4.0, 2.2),
          ::osg::Vec3d(0.0, 0.0, 0.7),
          ::osg::Vec3d(0.0, 0.0, 1.0)},
      [](const auto& state) {
        return createMasonryArch25ReducedWorld(state->solverMode);
      },
      kArchReducedMaxContacts,
      kArchReducedMaxContactsPerPair,
      false,
      true,
      "Author-faithful 25-stone masonry arch using the Rigid-IPC "
      "weighted-catenary generator (MIT, commit 23b6ba6): tapering voussoir "
      "stones, uniform friction 0.5, and the source boundary condition where "
      "only the ground is fixed and all 25 stones are dynamic.",
      "The current exact-FBF smoke runs one bounded step with a 48-contact, "
      "two-contacts-per-pair cap and should report zero boxed-LCP fallback. "
      "The Scene tab can launch a reduced projectile aimed through the crown "
      "so the missing Fig. 7 impact phase is visible in the GUI scaffold.",
      "This is not yet full Fig. 7 parity. Full-natural-manifold headless "
      "evidence exists (52 contacts on this geometry, clean at 1e-6 "
      "residual); this GUI uses the reduced 48-contact cap for interactive "
      "frame rates. Long-run post-impact outcome, timing, and paper-matched "
      "snapshots remain missing.",
      true,
      SolverMode::ExactFbf,
      renderMasonryArchProjectileControls,
      [](DemoSceneSetup& setup,
         const WorldPtr& world,
         const std::shared_ptr<FbfPaperState>& state) {
        setup.keyActions.push_back(
            KeyAction{'p', "Launch projectile", [world, state] {
                        launchMasonryArchProjectile(world, state);
                      }});
      });
}

//==============================================================================
DemoScene makeFbfPaperMasonryArch101Scene()
{
  return makeFbfPaperScene(
      "fbf_paper_masonry_arch_101",
      "FBF Paper: Masonry Arch 101",
      "Reduced-contact 101-stone masonry-arch scaffold for the paper's large "
      "arch scene.",
      CameraHome{
          ::osg::Vec3d(3.0, -4.0, 2.2),
          ::osg::Vec3d(0.0, 0.0, 0.7),
          ::osg::Vec3d(0.0, 0.0, 1.0)},
      [](const auto& state) {
        return createMasonryArch101ReducedWorld(state->solverMode);
      },
      kArch101ReducedMaxContacts,
      kArch101ReducedMaxContactsPerPair,
      false,
      true,
      "Author-faithful 101-stone masonry arch using the Rigid-IPC "
      "weighted-catenary generator (MIT, commit 23b6ba6): tapering voussoir "
      "stones, uniform friction 0.5, and the source boundary condition where "
      "only the ground is fixed and all 101 stones are dynamic.",
      "The current exact-FBF smoke runs one bounded step with a 38-contact, "
      "two-contacts-per-pair cap and should report zero boxed-LCP fallback.",
      "This is not yet full Fig. 8 parity. Full-natural-manifold headless "
      "evidence exists (204 contacts on this geometry, clean at 1e-6 "
      "residual); this GUI uses the reduced 38-contact cap for interactive "
      "frame rates. Long-run balance outcome, timing, traces, and snapshots "
      "remain missing.");
}

} // namespace dart_demos
