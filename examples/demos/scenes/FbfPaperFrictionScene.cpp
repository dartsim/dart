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

#include "FbfAuthorCardHouseSpec.hpp"
#include "FbfAuthorMasonryArchDartAdapter.hpp"
#include "FbfAuthorTurntableSpec.hpp"
#include "FbfLiteralMasonryArchSpec.hpp"
#include "Scenes.hpp"

#include <dart/gui/osg/osg.hpp>

#include <dart/utils/DartResourceRetriever.hpp>

#include <dart/simulation/DeactivationOptions.hpp>

#include <dart/constraint/ExactCoulombFbfConstraintSolver.hpp>

#include <dart/collision/dart/DARTCollisionDetector.hpp>
#include <dart/collision/native/NativeCollisionDetector.hpp>

#include <dart/math/detail/MasonryArchGeometry.hpp>

#include <dart/dart.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <cmath>

namespace dart_demos {

namespace {

using dart::dynamics::BodyNode;
using dart::dynamics::BoxShape;
using dart::dynamics::CollisionAspect;
using dart::dynamics::CylinderShape;
using dart::dynamics::DynamicsAspect;
using dart::dynamics::FreeJoint;
using dart::dynamics::MeshShape;
using dart::dynamics::PlaneShape;
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
constexpr std::size_t kCardHouseProjectileCount = 4u;
// The official paper teaser and Fig. 6 show cube projectiles. Their dimensions
// are unpublished; retain the former 0.11 m diameter as a reconstructed cube
// edge length so this source-backed shape correction does not masquerade as
// an author parameter.
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
// The existing boxed-LCP construction probe reaches this 512-contact cap.
// Therefore it is a bounded visual budget, not a measured natural-manifold
// count. Both the static inspection and dynamic adapter expose that limit.
constexpr std::size_t kCardHouseTenLevelMaxContacts = 512u;
constexpr std::size_t kCardHouseTenLevelMaxContactsPerPair = 8u;
// The paper's contact-rich arch experiments explicitly use mu=0.8. The
// credited Rigid-IPC geometry scene uses 0.5, but that source value is not the
// paper experiment's contact contract.
constexpr double kArchFriction = 0.8;
// Source-informed reconstruction: Rigid-IPC's default density is 1000 kg/m^3.
// The unavailable paper scene may override this value.
constexpr double kArchDensity = 1000.0;
constexpr std::size_t kArchStoneCount = 25u;
constexpr std::size_t kArch101StoneCount = 101u;
constexpr std::size_t kArchReducedMaxContacts = 48u;
constexpr std::size_t kArchReducedMaxContactsPerPair = 2u;
constexpr std::size_t kArch101ReducedMaxContacts = 38u;
constexpr std::size_t kArch101ReducedMaxContactsPerPair = 2u;
constexpr int kArchMaxOuterIterations = 120000;
constexpr double kArchOuterRelaxation = 1.5;
constexpr double kArchStepSizeScale = 10.0;
// The official teaser/video show a horizontal row of about twelve small cube
// projectiles dropping onto the crown. Count, edge, mass, spacing, and drop
// state remain reconstructions because the paper does not publish them.
constexpr std::size_t kArchProjectileCount = 12u;
constexpr double kArchProjectileEdgeLength = 0.035;
constexpr double kArchProjectileMass = kArchDensity * kArchProjectileEdgeLength
                                       * kArchProjectileEdgeLength
                                       * kArchProjectileEdgeLength;
constexpr double kArchProjectileSpeed = 3.0;
constexpr double kArchProjectileSpacing = 0.045;
constexpr double kArchProjectileDropHeight = 0.95;

enum class SolverMode
{
  ExactFbf,
  BoxedLcp,
};

using SolverInstaller = std::function<void(const WorldPtr&, SolverMode)>;

//==============================================================================
fbf_literal_masonry_arch::SolverLane literalMasonryArchSolverLane(
    SolverMode mode)
{
  return mode == SolverMode::ExactFbf
             ? fbf_literal_masonry_arch::SolverLane::ExactFbf
             : fbf_literal_masonry_arch::SolverLane::BoxedLcp;
}

//==============================================================================
fbf_author_masonry_arch_adapter::SolverLane authorMasonryArchSolverLane(
    SolverMode mode)
{
  return mode == SolverMode::ExactFbf
             ? fbf_author_masonry_arch_adapter::SolverLane::ExactFbf
             : fbf_author_masonry_arch_adapter::SolverLane::BoxedLcp;
}

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
std::shared_ptr<dart::collision::CollisionDetector>
createFbfPaperCollisionDetector(
    dart::collision::NativeCollisionDetector::ContactManifoldMode manifoldMode
    = dart::collision::NativeCollisionDetector::ContactManifoldMode::Compact)
{
  if (manifoldMode
      == dart::collision::NativeCollisionDetector::ContactManifoldMode::
          Compact) {
    return dart::collision::DARTCollisionDetector::create();
  }

  auto detector = dart::collision::NativeCollisionDetector::create();
  detector->setContactManifoldMode(manifoldMode);
  return detector;
}

//==============================================================================
void configureSolver(
    const WorldPtr& world,
    SolverMode mode,
    std::size_t maxContacts,
    std::size_t maxContactsPerPair,
    bool cardBudget = false,
    bool archBudget = false,
    dart::collision::NativeCollisionDetector::ContactManifoldMode manifoldMode
    = dart::collision::NativeCollisionDetector::ContactManifoldMode::Compact)
{
  if (mode == SolverMode::ExactFbf) {
    auto solver
        = std::make_unique<dart::constraint::ExactCoulombFbfConstraintSolver>(
            archBudget   ? makeArchFbfOptions()
            : cardBudget ? makeCardFbfOptions()
                         : makeFbfOptions());
    solver->setCollisionDetector(createFbfPaperCollisionDetector(manifoldMode));
    solver->setNumSimulationThreads(1u);
    world->setConstraintSolver(std::move(solver));
  } else {
    auto solver
        = std::make_unique<dart::constraint::BoxedLcpConstraintSolver>();
    solver->setCollisionDetector(createFbfPaperCollisionDetector(manifoldMode));
    solver->setNumSimulationThreads(1u);
    world->setConstraintSolver(std::move(solver));
  }

  auto& option = world->getConstraintSolver()->getCollisionOption();
  option.maxNumContacts = maxContacts;
  option.maxNumContactsPerPair = maxContactsPerPair;
}

//==============================================================================
dart::collision::NativeCollisionDetector::ContactManifoldMode
getConfiguredContactManifoldMode(const WorldPtr& world)
{
  const auto detector
      = std::dynamic_pointer_cast<dart::collision::NativeCollisionDetector>(
          world->getConstraintSolver()->getCollisionDetector());
  return detector != nullptr ? detector->getContactManifoldMode()
                             : dart::collision::NativeCollisionDetector::
                                 ContactManifoldMode::Compact;
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
void addBackspinCheckerTexture(BodyNode* body)
{
  const dart::common::Uri meshUri(
      "dart://sample/obj/fbf_backspin_checker_sphere.obj");
  const auto retriever = dart::utils::DartResourceRetriever::create();

  DART_SUPPRESS_DEPRECATED_BEGIN
  const aiScene* mesh = MeshShape::loadMesh(meshUri, retriever);
  DART_SUPPRESS_DEPRECATED_END
  if (!mesh)
    throw std::runtime_error("failed to load the backspin checker texture");

  // The physical SphereShape owns collision, mass, inertia, and friction.
  // This UV mesh is deliberately VisualAspect-only and uses DART's existing
  // MeshShape renderer to bind the OBJ material's checker texture.
  DART_SUPPRESS_DEPRECATED_BEGIN
  auto checker = std::make_shared<MeshShape>(
      Eigen::Vector3d::Constant(kBackspinRadius), mesh, meshUri, retriever);
  DART_SUPPRESS_DEPRECATED_END
  auto* checkerNode = body->createShapeNodeWith<VisualAspect>(checker);
  checkerNode->getVisualAspect()->setRGBA(Eigen::Vector4d::Ones());
}

//==============================================================================
SkeletonPtr createBackspinSphere()
{
  auto skeleton = Skeleton::create("backspin_sphere");
  auto* joint = skeleton->createJointAndBodyNodePair<FreeJoint>(nullptr).first;
  auto* body = skeleton->getBodyNode(0);
  auto shape = std::make_shared<SphereShape>(kBackspinRadius);
  auto* node
      = body->createShapeNodeWith<CollisionAspect, DynamicsAspect>(shape);
  node->getDynamicsAspect()->setFrictionCoeff(kBackspinFriction);
  setShapeInertia(body, shape);
  addBackspinCheckerTexture(body);

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
  configureSolver(
      world,
      mode,
      16u,
      4u,
      false,
      false,
      dart::collision::NativeCollisionDetector::ContactManifoldMode::
          FourPointPlanar);

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
    const std::string& name,
    SolverMode mode,
    const std::shared_ptr<FbfPaperState>& state,
    double friction,
    double angularVelocity)
{
  auto world = dart::simulation::World::create(name);
  configureWorldBase(world);
  configureSolver(
      world,
      mode,
      4u,
      4u,
      false,
      false,
      dart::collision::NativeCollisionDetector::ContactManifoldMode::
          FourPointPlanar);

  Eigen::Isometry3d turntable = Eigen::Isometry3d::Identity();
  turntable.translation().z() = -0.05;
  world->addSkeleton(createStaticBox(
      "turntable",
      Eigen::Vector3d(4.0, 4.0, 0.1),
      turntable,
      friction,
      Eigen::Vector4d(0.28, 0.30, 0.34, 1.0)));

  Eigen::Isometry3d rider = Eigen::Isometry3d::Identity();
  rider.translation() = Eigen::Vector3d(1.0, 0.0, 0.125 - 0.005);
  world->addSkeleton(createDynamicBox(
      "turntable_rider",
      Eigen::Vector3d::Constant(0.25),
      rider,
      friction,
      Eigen::Vector4d(0.20, 0.63, 0.91, 1.0)));

  state->turntableAngularVelocity = angularVelocity;
  return world;
}

//==============================================================================
SkeletonPtr createAuthorTurntableSupport(double friction)
{
  auto skeleton = Skeleton::create("turntable");
  auto* joint = skeleton->createJointAndBodyNodePair<FreeJoint>(nullptr).first;
  auto* body = skeleton->getBodyNode(0);
  auto collisionShape = std::make_shared<CylinderShape>(
      fbf_author_turntable::kSupportRadius,
      2.0 * fbf_author_turntable::kSupportHalfHeight);
  auto* node = body->createShapeNodeWith<CollisionAspect, DynamicsAspect>(
      collisionShape);
  node->getDynamicsAspect()->setFrictionCoeff(friction);

  const dart::common::Uri meshUri(
      "dart://sample/obj/fbf_author_turntable_disc.obj");
  const auto retriever = dart::utils::DartResourceRetriever::create();
  DART_SUPPRESS_DEPRECATED_BEGIN
  const aiScene* mesh = MeshShape::loadMesh(meshUri, retriever);
  DART_SUPPRESS_DEPRECATED_END
  if (!mesh)
    throw std::runtime_error("failed to load the segmented turntable visual");

  // The source-pinned CylinderShape remains the sole collision/dynamics
  // geometry. This paper-style sector mesh is VisualAspect-only: its coral
  // registration wedge and alternating gray sectors expose the analytic yaw
  // without changing mass, inertia, contacts, or friction.
  DART_SUPPRESS_DEPRECATED_BEGIN
  auto visualShape = std::make_shared<MeshShape>(
      Eigen::Vector3d(
          fbf_author_turntable::kSupportRadius,
          fbf_author_turntable::kSupportRadius,
          2.0 * fbf_author_turntable::kSupportHalfHeight),
      mesh,
      meshUri,
      retriever);
  DART_SUPPRESS_DEPRECATED_END
  auto* visualNode = body->createShapeNodeWith<VisualAspect>(visualShape);
  visualNode->getVisualAspect()->setRGBA(Eigen::Vector4d::Ones());

  // Author code places the kinematic cylinder center at z=half_height. DART's
  // rigid body, exact solver, and renderer remain implementation-local.
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation().z() = fbf_author_turntable::kSupportHalfHeight;
  joint->setPositions(FreeJoint::convertToPositions(transform));
  skeleton->setMobile(false);
  return skeleton;
}

//==============================================================================
SkeletonPtr createAuthorTurntableRider(double friction)
{
  constexpr double kSide = 2.0 * fbf_author_turntable::kRiderHalfSize;
  constexpr double kMass
      = fbf_author_turntable::kRiderDensity * kSide * kSide * kSide;
  const Eigen::Vector3d size = Eigen::Vector3d::Constant(kSide);

  auto skeleton = Skeleton::create("turntable_rider");
  auto* joint = skeleton->createJointAndBodyNodePair<FreeJoint>(nullptr).first;
  auto* body = skeleton->getBodyNode(0);
  auto shape = std::make_shared<BoxShape>(size);
  auto* node = body->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(shape);
  node->getVisualAspect()->setRGBA(Eigen::Vector4d(0.20, 0.63, 0.91, 1.0));
  node->getDynamicsAspect()->setFrictionCoeff(friction);

  dart::dynamics::Inertia inertia;
  inertia.setMass(kMass);
  inertia.setMoment(BoxShape::computeInertia(size, kMass));
  body->setInertia(inertia);

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(
      fbf_author_turntable::kInitialRadius,
      0.0,
      2.0 * fbf_author_turntable::kSupportHalfHeight
          + fbf_author_turntable::kRiderHalfSize
          + fbf_author_turntable::kGeometricGap
          + fbf_author_turntable::kDropHeight);
  joint->setPositions(FreeJoint::convertToPositions(transform));
  return skeleton;
}

//==============================================================================
WorldPtr createAuthorTurntableWorld(
    const std::string& name,
    SolverMode mode,
    const std::shared_ptr<FbfPaperState>& state,
    double friction,
    double angularVelocity)
{
  auto world = dart::simulation::World::create(name);
  configureWorldBase(world);
  configureSolver(
      world,
      mode,
      4u,
      4u,
      false,
      false,
      dart::collision::NativeCollisionDetector::ContactManifoldMode::
          FourPointPlanar);
  auto* exactSolver
      = dynamic_cast<dart::constraint::ExactCoulombFbfConstraintSolver*>(
          world->getConstraintSolver());
  if (exactSolver == nullptr)
    throw std::runtime_error("author turntable requires exact FBF");
  exactSolver->setExactCoulombOptions(
      fbf_author_turntable::dartBestSolverOptions());
  world->addSkeleton(createAuthorTurntableSupport(friction));
  world->addSkeleton(createAuthorTurntableRider(friction));
  state->turntableAngularVelocity = angularVelocity;
  return world;
}

//==============================================================================
void setAuthorTurntableSupportState(
    const WorldPtr& world, double time, double targetAngularVelocity)
{
  const auto turntable = world->getSkeleton("turntable");
  auto* joint = dynamic_cast<FreeJoint*>(turntable->getJoint(0));
  if (joint == nullptr)
    throw std::runtime_error("author turntable support has no FreeJoint");

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear()
      = Eigen::AngleAxisd(
            fbf_author_turntable::integratedYaw(time, targetAngularVelocity),
            Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
  transform.translation().z() = fbf_author_turntable::kSupportHalfHeight;
  joint->setPositions(FreeJoint::convertToPositions(transform));
  joint->setAngularVelocity(Eigen::Vector3d(
      0.0,
      0.0,
      fbf_author_turntable::angularVelocity(time, targetAngularVelocity)));
}

//==============================================================================
Eigen::Vector4d authorCardHouseColor(
    const fbf_author_card_house::CardSpec& spec)
{
  const bool alternate = (spec.level + spec.tent) % 2u != 0u;
  switch (spec.kind) {
    case fbf_author_card_house::CardKind::Bridge:
      return alternate ? Eigen::Vector4d(0.68, 0.57, 0.46, 1.0)
                       : Eigen::Vector4d(0.79, 0.69, 0.55, 1.0);
    case fbf_author_card_house::CardKind::LeanLeft:
      return alternate ? Eigen::Vector4d(0.76, 0.40, 0.37, 1.0)
                       : Eigen::Vector4d(0.88, 0.58, 0.48, 1.0);
    case fbf_author_card_house::CardKind::LeanRight:
      return alternate ? Eigen::Vector4d(0.38, 0.56, 0.66, 1.0)
                       : Eigen::Vector4d(0.50, 0.68, 0.72, 1.0);
  }
  return Eigen::Vector4d(0.7, 0.7, 0.7, 1.0);
}

//==============================================================================
SkeletonPtr createAuthorCardHouseGround()
{
  auto skeleton = Skeleton::create("ground_plane");
  const auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>(nullptr);
  auto* joint = pair.first;
  auto* body = pair.second;
  auto* collisionNode
      = body->createShapeNodeWith<CollisionAspect, DynamicsAspect>(
          std::make_shared<PlaneShape>(Eigen::Vector3d::UnitZ(), 0.0));
  body->setGravityMode(true);
  fbf_author_card_house::configureDartFreeJoint(joint);
  fbf_author_card_house::configureDartContactMaterial(
      collisionNode->getDynamicsAspect());

  // Newton's infinite ground has no source renderer. This finite slab is a
  // VisualAspect-only DART presentation aid for the construction still.
  auto* visualNode = body->createShapeNodeWith<VisualAspect>(
      std::make_shared<BoxShape>(Eigen::Vector3d(13.0, 4.0, 0.04)));
  visualNode->setRelativeTranslation(Eigen::Vector3d(0.0, 0.0, -0.02));
  visualNode->getVisualAspect()->setRGBA(
      Eigen::Vector4d(0.34, 0.35, 0.36, 1.0));
  skeleton->setMobile(false);
  return skeleton;
}

//==============================================================================
SkeletonPtr createAuthorCardHouseBox(
    const std::string& name,
    const Eigen::Vector3d& size,
    const Eigen::Isometry3d& transform,
    double density,
    const Eigen::Vector4d& color,
    bool mobile)
{
  auto skeleton = createDynamicBox(
      name, size, transform, fbf_author_card_house::kFriction, color);
  auto* body = skeleton->getBodyNode(0u);
  auto* joint = dynamic_cast<FreeJoint*>(skeleton->getJoint(0u));
  const double mass = density * size.x() * size.y() * size.z();
  dart::dynamics::Inertia inertia;
  inertia.setMass(mass);
  inertia.setMoment(BoxShape::computeInertia(size, mass));
  body->setInertia(inertia);
  body->setGravityMode(true);
  fbf_author_card_house::configureDartFreeJoint(joint);
  auto* collisionNode = body->getShapeNodeWith<CollisionAspect>(0u);
  if (collisionNode == nullptr)
    throw std::runtime_error("author card house collision shape is missing");
  fbf_author_card_house::configureDartContactMaterial(
      collisionNode->getDynamicsAspect());
  skeleton->setMobile(mobile);
  return skeleton;
}

//==============================================================================
void installAuthorCardHouseSolver(
    const WorldPtr& world,
    SolverMode mode,
    bool postCorrectionProjectionEnabled = true)
{
  configureSolver(
      world,
      mode,
      fbf_author_card_house::kSourceMaxContacts,
      fbf_author_card_house::kDartMaxContactsPerPair,
      false,
      false,
      dart::collision::NativeCollisionDetector::ContactManifoldMode::
          FourPointPlanar);
  auto* solver = world->getConstraintSolver();
  fbf_author_card_house::configureDartCollisionGeneration(solver);
  if (mode == SolverMode::ExactFbf) {
    auto* exactSolver
        = dynamic_cast<dart::constraint::ExactCoulombFbfConstraintSolver*>(
            solver);
    if (exactSolver == nullptr)
      throw std::runtime_error("author card house requires exact FBF");
    exactSolver->setExactCoulombOptions(
        fbf_author_card_house::dartConstructionSolverOptions());
    exactSolver->setExactCoulombCrossStepPolicyOptions(
        fbf_author_card_house::dartConstructionCrossStepPolicyOptions());
    exactSolver->setExactCoulombPostCorrectionProjectionEnabled(
        postCorrectionProjectionEnabled);
    exactSolver->setExactCoulombColoredBlockGaussSeidelEnabled(false);
    exactSolver
        ->setExactCoulombColoredBlockGaussSeidelParticipantAffinityEnabled(
            false);
  } else {
    fbf_author_card_house::configureDartBoxedBaseline(
        dynamic_cast<dart::constraint::BoxedLcpConstraintSolver*>(solver));
  }
}

//==============================================================================
WorldPtr createAuthorCardHouseWorld(
    const std::string& name,
    std::size_t levelCount,
    SolverMode mode,
    bool postCorrectionProjectionEnabled = true)
{
  auto world = dart::simulation::World::create(name);
  configureWorldBase(world);
  world->setTimeStep(fbf_author_card_house::kRuntimeTimeStep);
  installAuthorCardHouseSolver(world, mode, postCorrectionProjectionEnabled);

  world->addSkeleton(createAuthorCardHouseGround());
  for (const auto& spec : fbf_author_card_house::makeCardSpecs(levelCount)) {
    world->addSkeleton(createAuthorCardHouseBox(
        spec.name,
        spec.size,
        spec.transform,
        fbf_author_card_house::kCardDensity,
        authorCardHouseColor(spec),
        true));
  }

  const std::array<Eigen::Vector4d, fbf_author_card_house::kCubeCount> colors{
      {Eigen::Vector4d(0.72, 0.48, 0.42, 1.0),
       Eigen::Vector4d(0.49, 0.61, 0.67, 1.0),
       Eigen::Vector4d(0.70, 0.63, 0.42, 1.0),
       Eigen::Vector4d(0.54, 0.49, 0.64, 1.0)}};
  for (const auto& spec : fbf_author_card_house::makeCubeSpecs(levelCount)) {
    world->addSkeleton(createAuthorCardHouseBox(
        spec.name,
        spec.size,
        spec.transform,
        fbf_author_card_house::kCubeDensity,
        colors[spec.index],
        false));
  }
  return world;
}

//==============================================================================
WorldPtr createPainleveWorld(
    const std::string& name, SolverMode mode, double friction)
{
  auto world = dart::simulation::World::create(name);
  configureWorldBase(world);
  configureSolver(world, mode, 4u, 4u);
  world->addSkeleton(createGround(friction));

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
      friction,
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
  const double horizontalHalfExtent
      = 0.5
        * (kCardHouseHeight * std::sin(kCardHouseAngle)
           + kCardHouseThickness * std::cos(kCardHouseAngle));
  const double centerOffset
      = horizontalHalfExtent - 0.5 * kCardHouseInitialPenetration;
  transform.translation().x()
      = centerX + (leftCard ? -centerOffset : centerOffset);
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

  const double cardMass = kCardHouseDensity * size.x() * size.y() * size.z();
  auto card
      = createDynamicBox(name, size, transform, kCardHouseFriction, color);
  auto* body = card->getBodyNode(0);
  dart::dynamics::Inertia inertia;
  inertia.setMass(cardMass);
  inertia.setMoment(BoxShape::computeInertia(size, cardMass));
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
      (static_cast<double>(index) - 1.5) * 0.15,
      0.0,
      kCardHouseProjectileDropHeight);
  const std::array<Eigen::Vector4d, kCardHouseProjectileCount> colors
      = {Eigen::Vector4d(0.90, 0.48, 0.49, 1.0),
         Eigen::Vector4d(0.80, 0.66, 0.55, 1.0),
         Eigen::Vector4d(0.91, 0.62, 0.27, 1.0),
         Eigen::Vector4d(0.84, 0.55, 0.66, 1.0)};
  auto projectile = createDynamicBox(
      "fbf_projectile_" + std::to_string(index),
      Eigen::Vector3d::Constant(kCardHouseProjectileEdgeLength),
      transform,
      kCardHouseFriction,
      colors[index],
      Eigen::Vector3d(0.0, 0.0, -kCardHouseProjectileSpeed));
  auto* body = projectile->getBodyNode(0);
  dart::dynamics::Inertia inertia;
  inertia.setMass(kCardHouseProjectileMass);
  inertia.setMoment(BoxShape::computeInertia(
      Eigen::Vector3d::Constant(kCardHouseProjectileEdgeLength),
      kCardHouseProjectileMass));
  body->setInertia(inertia);
  return projectile;
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
      kCardHouseTenLevelMaxContacts,
      kCardHouseTenLevelMaxContactsPerPair,
      false,
      false);
}

//==============================================================================
WorldPtr createCardHouseTenLevelDynamicWorld(SolverMode mode)
{
  return createCardHouseWorld(
      "fbf_paper_card_house_10_dynamic",
      kCardHouseTenLevelCount,
      mode,
      kCardHouseTenLevelMaxContacts,
      kCardHouseTenLevelMaxContactsPerPair,
      true,
      true);
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

  // The paper explicitly pins both springers in the 25-stone scene. Figure 8
  // calls the 101-stone experiment the same setup, so the same endpoint
  // condition is used there. The credited raw Rigid-IPC scenes instead leave
  // every stone dynamic; that source distinction is retained in provenance.
  world->addSkeleton(createGround(kArchFriction));
  const auto stoneGeometry
      = dart::math::detail::generateMasonryArchStoneBoxes(stoneCount);
  for (std::size_t i = 0u; i < stoneCount; ++i) {
    auto stone = createMasonryArchStone(i, stoneGeometry[i]);
    if (i == 0u || i + 1u == stoneCount)
      stone->setMobile(false);
    world->addSkeleton(stone);
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
SkeletonPtr createMasonryArchProjectile(std::size_t index)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(
      (static_cast<double>(index)
       - 0.5 * static_cast<double>(kArchProjectileCount - 1u))
          * kArchProjectileSpacing,
      0.0,
      kArchProjectileDropHeight);
  auto projectile = createDynamicBox(
      "masonry_arch_projectile_" + std::to_string(index),
      Eigen::Vector3d::Constant(kArchProjectileEdgeLength),
      transform,
      kArchFriction,
      Eigen::Vector4d(0.90, 0.48, 0.55, 1.0),
      Eigen::Vector3d(0.0, 0.0, -kArchProjectileSpeed));
  auto* body = projectile->getBodyNode(0);
  dart::dynamics::Inertia inertia;
  inertia.setMass(kArchProjectileMass);
  inertia.setMoment(BoxShape::computeInertia(
      Eigen::Vector3d::Constant(kArchProjectileEdgeLength),
      kArchProjectileMass));
  body->setInertia(inertia);
  return projectile;
}

//==============================================================================
void launchMasonryArchProjectile(
    const WorldPtr& world, const std::shared_ptr<FbfPaperState>& state)
{
  if (world->getSkeleton("masonry_arch_projectile_0") == nullptr) {
    for (std::size_t i = 0u; i < kArchProjectileCount; ++i)
      world->addSkeleton(createMasonryArchProjectile(i));
  }

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
    bool exactFbfAvailable = true,
    bool solverSelectionEnabled = true,
    const SolverInstaller& solverInstaller = nullptr)
{
  ImGui::TextDisabled("Solver");
  int solverIndex = exactFbfAvailable
                        ? (state->solverMode == SolverMode::ExactFbf ? 0 : 1)
                        : 0;
  bool clicked = false;
  if (exactFbfAvailable && solverSelectionEnabled) {
    clicked |= ImGui::RadioButton("Exact FBF", &solverIndex, 0);
    ImGui::SameLine();
    clicked |= ImGui::RadioButton("Boxed LCP", &solverIndex, 1);
  } else if (exactFbfAvailable) {
    ImGui::TextUnformatted("Exact FBF (fixed by evidence contract)");
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
      if (solverInstaller) {
        solverInstaller(world, state->solverMode);
      } else {
        configureSolver(
            world,
            state->solverMode,
            maxContacts,
            maxContactsPerPair,
            cardBudget,
            archBudget,
            getConfiguredContactManifoldMode(world));
      }
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
  ImGui::Text("Trace phase: settle, then four vertically dropped cubes");
  if (ImGui::Button("Drop 4 projectile cubes")) {
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
      = world->getSkeleton("masonry_arch_projectile_0") != nullptr;
  ImGui::Text(
      "Projectile row (%zu cubes): %s",
      kArchProjectileCount,
      state->masonryArchProjectileLaunched ? "launched" : "not launched");
  ImGui::TextUnformatted("Reconstructed vertical drop over the crown.");
  if (ImGui::Button("Drop projectile row")) {
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
    = nullptr,
    const SolverInstaller& solverInstaller = nullptr)
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
                         renderExtraPanel,
                         solverInstaller] {
      renderSolverControls(
          world,
          state,
          maxContacts,
          maxContactsPerPair,
          cardBudget,
          archBudget,
          exactFbfAvailable,
          true,
          solverInstaller);
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
           archBudget,
           solverInstaller] {
            state->solverMode = state->solverMode == SolverMode::ExactFbf
                                    ? SolverMode::BoxedLcp
                                    : SolverMode::ExactFbf;
            if (solverInstaller) {
              solverInstaller(world, state->solverMode);
            } else {
              configureSolver(
                  world,
                  state->solverMode,
                  maxContacts,
                  maxContactsPerPair,
                  cardBudget,
                  archBudget,
                  getConfiguredContactManifoldMode(world));
            }
          }});
    }
    if (configureExtraSetup)
      configureExtraSetup(setup, world, state);
    return setup;
  };
  return scene;
}

//==============================================================================
DemoScene makeFbfPaperTurntableParameterizedScene(
    const std::string& id,
    const std::string& title,
    double friction,
    double angularVelocity,
    bool angularVelocityEditable,
    const std::string& summary,
    const std::string& overview,
    const std::string& expected,
    const std::string& coverage)
{
  DemoScene scene;
  scene.id = id;
  scene.title = title;
  scene.category = "Research";
  scene.summary = summary;
  scene.scenePanelDocumentation
      = ScenePanelDocumentation{overview, expected, coverage};

  scene.factory = [=] {
    auto state = std::make_shared<FbfPaperState>();
    auto world = createTurntableWorld(
        id, state->solverMode, state, friction, angularVelocity);

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
    setup.renderPanel = [world, state, friction, angularVelocityEditable] {
      renderSolverControls(world, state, 4u, 4u, false, false, true);
      ImGui::Separator();
      ImGui::TextDisabled("Fixture parameters");
      ImGui::Text("Friction coefficient: %.2f", friction);
      if (!angularVelocityEditable) {
        ImGui::Text(
            "Angular speed: %.2f rad/s (fixed)",
            state->turntableAngularVelocity);
        return;
      }

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
          configureSolver(
              world,
              state->solverMode,
              4u,
              4u,
              false,
              false,
              getConfiguredContactManifoldMode(world));
        }});
    return setup;
  };
  return scene;
}

//==============================================================================
DemoScene makeFbfAuthorTurntableParameterizedScene(
    const std::string& title,
    const fbf_author_turntable::ScenarioSpec& scenario)
{
  DemoScene scene;
  scene.id = scenario.demoScene;
  scene.title = title;
  scene.category = "Research";
  scene.summary
      = "Source-pinned author turntable cell with fixed friction and speed.";
  scene.scenePanelDocumentation = ScenePanelDocumentation{
      "Numerical scene and control schedule pinned to public author commit "
      "b3f3c5c: a radius-2 m, height-0.1 m cylinder; a 0.3 m cube at "
      "radius 1 m with density 500 kg/m^3, 5 mm gap, and 0.2 m drop; then "
      "0.5 s stationary settle and a 0.5 s smoothstep speed ramp.",
      "Over the six-second source horizon this cell is expected to be "
          + std::string(
              std::string(scenario.expectedOutcome) == "ejected"
                  ? "ejected"
                  : "retained on the support through 6 s")
          + ".",
      "The geometry, mass, placement, timing, friction, and commanded angular "
      "speed follow the public float32 Warp/Newton source. DART remains a "
      "float64 reimplementation with its own exact solver, Native "
      "FourPointPlanar contact frontend, camera, materials, and renderer."};

  scene.factory = [scenario] {
    auto state = std::make_shared<FbfPaperState>();
    auto world = createAuthorTurntableWorld(
        scenario.demoScene,
        state->solverMode,
        state,
        scenario.friction,
        scenario.angularVelocity);

    DemoSceneSetup setup;
    setup.world = world;
    setup.physicsContractProvider = [world, scenario] {
      return fbf_author_turntable::physicsContractJson(
          fbf_author_turntable::inspectPhysicsContract(
              world,
              scenario,
              "dart_best",
              "dart_demos",
              DART_FBF_AUTHOR_TURNTABLE_IMPLEMENTATION_SHA256));
    };
    setup.cameraHome = CameraHome{
        ::osg::Vec3d(6.3, -7.4, 4.5),
        ::osg::Vec3d(0.0, 0.0, 0.15),
        ::osg::Vec3d(0.0, 0.0, 1.0)};
    setup.preStep = [world, state] {
      setAuthorTurntableSupportState(
          world, world->getTime(), state->turntableAngularVelocity);
    };
    setup.postStep = [world, state] {
      // The support is deliberately immobile, so DART does not integrate its
      // FreeJoint pose. Reapply the analytic yaw at the completed time to keep
      // renderer orientation synchronized with the contact surface velocity.
      setAuthorTurntableSupportState(
          world, world->getTime(), state->turntableAngularVelocity);
    };
    setup.renderPanel = [world, state, scenario] {
      renderSolverControls(world, state, 4u, 4u, false, false, true, false);
      ImGui::Separator();
      ImGui::TextDisabled("Pinned author parameters (b3f3c5c)");
      ImGui::Text("Friction coefficient: %.2f", scenario.friction);
      ImGui::Text(
          "Angular speed: %.2f rad/s (fixed)", state->turntableAngularVelocity);
      ImGui::Text("Schedule: settle .5 s, smoothstep ramp .5 s, run 6 s");
    };
    return setup;
  };
  return scene;
}

//==============================================================================
DemoScene makeFbfAuthorCardHouseParameterizedScene(std::size_t levelCount)
{
  DemoScene scene;
  scene.id = fbf_author_card_house::kDemoSceneId;
  scene.title = "FBF Author Card House: 5-Level Construction";
  scene.category = "Research";
  scene.summary
      = "Source-pinned public-author 40-card construction still with four "
        "suspended cubes.";
  scene.scenePanelDocumentation = ScenePanelDocumentation{
      "Configuration pinned to public author commit b3f3c5c: five levels, "
      "30 leaning cards, 10 bridges, and four 0.8 m cubes. Card dimensions, "
      "density, mass, friction, poses, and the source release schedule are "
      "queryable from the scene contract.",
      "The expected result is a legible time-zero construction still. The "
      "cards preserve the author's mobile state, the four cubes remain "
      "immobile, and the construction evidence executes zero simulation "
      "steps and no release event.",
      "This source-configuration port proves neither trajectory nor solver "
      "equivalence, physical outcome, Fig. 6 or video parity, nor timing "
      "comparability. The alternating paper colors, finite ground slab, "
      "camera, and renderer are DART-only presentation choices."};

  scene.factory = [levelCount] {
    auto world = createAuthorCardHouseWorld(
        fbf_author_card_house::kDemoSceneId, levelCount, SolverMode::ExactFbf);

    DemoSceneSetup setup;
    setup.world = world;
    setup.physicsContractProvider = [world, levelCount] {
      return fbf_author_card_house::configurationContractJson(
          fbf_author_card_house::inspectConfigurationContract(
              world,
              levelCount,
              "dart_demos",
              DART_FBF_AUTHOR_CARD_HOUSE_IMPLEMENTATION_SHA256));
    };
    setup.cameraHome = CameraHome{
        ::osg::Vec3d(26.8, -35.6, 18.6),
        ::osg::Vec3d(0.0, 0.0, 5.5),
        ::osg::Vec3d(0.0, 0.0, 1.0)};
    setup.renderPanel = [levelCount] {
      ImGui::TextDisabled("Pinned author construction (b3f3c5c)");
      ImGui::Text("Levels: %zu", levelCount);
      ImGui::Text(
          "Cards: %zu leaning + %zu bridges",
          fbf_author_card_house::leaningCardCount(levelCount),
          fbf_author_card_house::bridgeCardCount(levelCount));
      ImGui::Text("Suspended cubes: %zu", fbf_author_card_house::kCubeCount);
      ImGui::Text("Friction: %.2f", fbf_author_card_house::kFriction);
      ImGui::Separator();
      ImGui::TextDisabled("Construction only: time-zero inspection");
      ImGui::TextWrapped(
          "Cards are mobile; cubes remain suspended. No release, trajectory, "
          "outcome, timing, or paper-frame claim.");
    };
    return setup;
  };
  return scene;
}

//==============================================================================
void renderFbfAuthorCardHouseDynamicsControls(const WorldPtr& world)
{
  using namespace fbf_author_card_house;
  ImGui::Separator();
  ImGui::TextDisabled("Pinned current-source schedule (b3f3c5c)");
  ImGui::Text("Four levels: 20 leaning + 6 bridge cards");
  ImGui::Text("Suspended cubes: 4 x edge 0.8 m, mass 256 kg");
  ImGui::Text("Runtime dt: 1/240 s; evidence horizon: 2,400 steps");
  ImGui::Text("Runner: p after step 1,600; interactive p/button: immediate");
  const bool released = cubesReleased(world, kFigureLevelCount);
  ImGui::Text("Cube state: %s", released ? "released" : "kinematic");
  if (!released && ImGui::Button("Release 4 source-configured cubes"))
    releaseCubes(world, kFigureLevelCount);
  ImGui::TextWrapped(
      "This is a current-source-parameterized DART adapter. Historical paper "
      "invocation, source collision/contact-gap/backend/float32 semantics, "
      "trajectory, outcome, timing, and Fig. 6 parity remain unclaimed.");
}

//==============================================================================
DemoScene makeFbfAuthorCardHouse4ImpactCurrentSourceParameterizedScene()
{
  using namespace fbf_author_card_house;
  return makeFbfPaperScene(
      kDynamicsDemoSceneId,
      "FBF Author Card House 4: Impact (Current Source)",
      "Public-author four-level 26-card configuration and 600-frame cube "
      "release schedule executed through DART exact/boxed dynamics adapters.",
      CameraHome{
          ::osg::Vec3d(22.0, -30.0, 16.0),
          ::osg::Vec3d(0.0, 0.0, 4.7),
          ::osg::Vec3d(0.0, 0.0, 1.0)},
      [](const auto& state) {
        return createAuthorCardHouseWorld(
            kDynamicsDemoSceneId, kFigureLevelCount, state->solverMode, false);
      },
      kSourceMaxContacts,
      kDartMaxContactsPerPair,
      false,
      false,
      "Configuration pinned to public author commit b3f3c5c and selected "
      "through supported run.py arguments: four levels, 20 leaning cards, "
      "six bridges, four initially kinematic 0.8 m cubes, mu=.8, and 600 "
      "display frames. The evidence runner uses four DART substeps per frame "
      "at dt=1/240 s and invokes immediate action `p` after completed step "
      "1,600 to release the existing cubes from rest.",
      "The expected evidence is a deterministic, finite DART exact/boxed "
      "comparison spanning the pre-release construction and subsequent cube "
      "impact. Exact media is promotable only if every constrained step passes "
      "the strict residual, cap, failure, and fallback gates and the physical "
      "outcome oracle is satisfied.",
      "This is a DART dynamics adapter for a source-supported "
      "parameterization, "
      "not proof of the historical paper invocation. Native FourPointPlanar "
      "collision, DART contact-gap behavior, float64 arithmetic, exact-FBF "
      "options, boxed LCP, camera, materials, and rendering differ from the "
      "author runtime. Trajectory/outcome equivalence, Fig. 6/video parity, "
      "and timing comparability remain unproven.",
      true,
      SolverMode::ExactFbf,
      [](const WorldPtr& world, const std::shared_ptr<FbfPaperState>&) {
        renderFbfAuthorCardHouseDynamicsControls(world);
      },
      [](DemoSceneSetup& setup,
         const WorldPtr& world,
         const std::shared_ptr<FbfPaperState>&) {
        setup.physicsContractProvider = [world] {
          return dynamicsAdapterContractJson(inspectDynamicsAdapterContract(
              world,
              kFigureLevelCount,
              DART_FBF_AUTHOR_CARD_HOUSE_IMPLEMENTATION_SHA256));
        };
        setup.keyActions.push_back(KeyAction{
            kReleaseActionKey, "Release 4 source-configured cubes", [world] {
              releaseCubes(world, kFigureLevelCount);
            }});
        setup.onActivate = [](DemoHostContext& context) {
          auto scopedErp
              = std::make_shared<ScopedContactErrorReductionParameter>();
          context.addTeardown([scopedErp]() mutable { scopedErp.reset(); });
        };
      },
      [](const WorldPtr& world, SolverMode mode) {
        installAuthorCardHouseSolver(world, mode, false);
      });
}

//==============================================================================
void renderFbfAuthorMasonryArchControls(const WorldPtr& world)
{
  ImGui::Separator();
  ImGui::TextDisabled("Pinned author schedule (b3f3c5c)");
  const bool released = fbf_author_masonry_arch_adapter::cubesReleased(world);
  ImGui::Text("Raw-scale wedges: 25 (23 mobile)");
  ImGui::Text("Suspended cubes: 3 x edge 3, mass 54000");
  ImGui::Text("Runtime dt: 1/240 s");
  ImGui::Text("Runner: p after step 1600; interactive p/button: immediate");
  ImGui::Text("Cube state: %s", released ? "released" : "kinematic");
  if (!released && ImGui::Button("Release 3 source-configured cubes"))
    fbf_author_masonry_arch_adapter::releaseCubes(world);
  ImGui::TextWrapped(
      "This DART adapter ports the pinned numeric geometry, masses, friction, "
      "initial state, and release schedule. Collision, contact-gap, "
      "solver-backend, float32, trajectory, outcome, timing, and Fig. 7 "
      "parity remain unclaimed.");
}

//==============================================================================
DemoScene makeFbfAuthorMasonryArch25CrownImpactCurrentSourceParameterizedScene()
{
  using namespace fbf_author_masonry_arch_adapter;
  return makeFbfPaperScene(
      kDemoSceneId,
      "FBF Author Masonry Arch 25: Crown Impact (Current Source)",
      "Public-author raw-scale 25-wedge configuration and 500-frame release "
      "schedule executed through DART exact/boxed dynamics adapters.",
      CameraHome{
          ::osg::Vec3d(135.0, -195.0, 110.0),
          ::osg::Vec3d(0.0, 0.0, 28.0),
          ::osg::Vec3d(0.0, 0.0, 1.0)},
      [](const auto& state) {
        return createWorld(authorMasonryArchSolverLane(state->solverMode));
      },
      fbf_author_masonry_arch::kSourceMaxContacts,
      kDartMaxContactsPerPair,
      false,
      false,
      "Configuration pinned to public author commit b3f3c5c: 25 quantized "
      "literal OBJ wedges at raw numeric scale, fixed springers, mu=.8, and "
      "three initially kinematic cubes above the crown. The evidence schedule "
      "runs 500 display frames as 2,000 DART substeps at dt=1/240 s and "
      "asks the immediate `p` action to release the existing cubes after "
      "completed step 1,600; interactive `p`/button use releases them at once.",
      "The expected evidence is a deterministic, finite DART exact/boxed "
      "comparison spanning the pre-release construction and subsequent crown "
      "impact. Exact media is promotable only if every constrained step passes "
      "the strict residual, cap, failure, and fallback gates.",
      "This is a DART dynamics adapter for a source-pinned configuration and "
      "release schedule. Native FourPointPlanar collision, split impulse, "
      "float64 arithmetic, exact-FBF options, boxed LCP, camera, materials, "
      "and rendering are DART choices. Source trajectory/outcome equivalence, "
      "the paper's 100-contact timing row, Fig. 7/video parity, and timing "
      "comparability remain unproven.",
      true,
      SolverMode::ExactFbf,
      [](const WorldPtr& world, const std::shared_ptr<FbfPaperState>&) {
        renderFbfAuthorMasonryArchControls(world);
      },
      [](DemoSceneSetup& setup,
         const WorldPtr& world,
         const std::shared_ptr<FbfPaperState>&) {
        setup.physicsContractProvider = [world] {
          return adapterContractJson(
              inspectAdapterContract(world),
              DART_FBF_AUTHOR_MASONRY_ARCH_SPEC_SHA256,
              DART_FBF_AUTHOR_MASONRY_ARCH_ADAPTER_SHA256,
              DART_FBF_AUTHOR_MASONRY_ARCH_IMPLEMENTATION_SHA256);
        };
        setup.keyActions.push_back(KeyAction{
            kReleaseActionKey, "Release 3 source-configured cubes", [world] {
              releaseCubes(world);
            }});
        setup.onActivate = [](DemoHostContext& context) {
          auto scopedErp
              = std::make_shared<ScopedContactErrorReductionParameter>();
          context.addTeardown([scopedErp]() mutable { scopedErp.reset(); });
        };
      },
      [](const WorldPtr& world, SolverMode mode) {
        installSolver(
            world,
            authorMasonryArchSolverLane(mode),
            world->getNumSimulationThreads());
      });
}

//==============================================================================
DemoScene makeFbfPaperPainleveParameterizedScene(
    const std::string& id,
    const std::string& title,
    double friction,
    const std::string& summary,
    const std::string& overview,
    const std::string& expected)
{
  return makeFbfPaperScene(
      id,
      title,
      summary,
      CameraHome{
          ::osg::Vec3d(4.5, -4.0, 2.4),
          ::osg::Vec3d(0.8, 0.0, 0.35),
          ::osg::Vec3d(0.0, 0.0, 1.0)},
      [id, friction](const auto& state) {
        return createPainleveWorld(id, state->solverMode, friction);
      },
      4u,
      4u,
      false,
      false,
      overview.c_str(),
      expected.c_str(),
      "This scene is an outcome-oriented DART proxy, not author-scene parity. "
      "Its geometry, mass, initial state, camera, and 2.5-second observation "
      "window are local reconstruction choices.",
      true,
      SolverMode::ExactFbf,
      [friction](const WorldPtr&, const std::shared_ptr<FbfPaperState>&) {
        ImGui::Separator();
        ImGui::TextDisabled("Fixture parameters");
        ImGui::Text("Friction coefficient: %.2f (fixed)", friction);
      });
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
      // Keep the view nearly perpendicular to the Y spin axis. This makes the
      // renderer-applied checker cells read as quadrilaterals and keeps the
      // full advance/reversal trajectory in frame.
      CameraHome{
          ::osg::Vec3d(-0.5, -1.25, 5.5),
          ::osg::Vec3d(-0.5, 0.0, 0.2),
          ::osg::Vec3d(0.0, 1.0, 0.0)},
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
  return makeFbfPaperTurntableParameterizedScene(
      "fbf_paper_turntable",
      "FBF Paper: Turntable (mu=.5, omega=2)",
      0.5,
      2.0,
      true,
      "Generic turntable proxy initialized at mu=.5 and omega=2 rad/s, with "
      "a live angular-speed control.",
      "Rotating turntable capture/ejection proxy initialized at friction "
      "mu=.5 and angular speed omega=2 rad/s. Angular speed is editable in "
      "the Scene panel; use the fixed parameter scenes for evidence capture.",
      "At the initial mu=.5, omega=2 setting, the exact-FBF headless "
      "regression classifies the rider as captured over four seconds.",
      "The square support, one-second speed ramp, cube properties, camera, "
      "and four-second horizon are DART reconstruction choices. This generic "
      "interactive scene is not a stable parameter-cell capture target.");
}

//==============================================================================
DemoScene makeFbfPaperTurntableMu02Omega2Scene()
{
  return makeFbfPaperTurntableParameterizedScene(
      "fbf_paper_turntable_mu_0_2_omega_2",
      "FBF Paper: Turntable mu=.2, omega=2",
      0.2,
      2.0,
      false,
      "Fixed turntable proxy at mu=.2 and omega=2 rad/s for reproducible "
      "headless capture.",
      "Fixed parameter cell from the paper's turntable grid: the support and "
      "rider both use mu=.2, and the support ramps to omega=2 rad/s in one "
      "second.",
      "The exact-FBF headless regression classifies the rider as ejected over "
      "the four-second observation window.",
      "The support is square rather than the paper video's segmented disk; "
      "the ramp, cube properties, camera, and observation horizon are local "
      "reconstruction choices. This scene fixes parameters but is not asset "
      "or snapshot parity.");
}

//==============================================================================
DemoScene makeFbfPaperTurntableMu02Omega5Scene()
{
  return makeFbfPaperTurntableParameterizedScene(
      "fbf_paper_turntable_mu_0_2_omega_5",
      "FBF Paper: Turntable mu=.2, omega=5",
      0.2,
      5.0,
      false,
      "Fixed turntable proxy at mu=.2 and omega=5 rad/s for reproducible "
      "headless capture.",
      "Fixed parameter cell from the paper's turntable grid: the support and "
      "rider both use mu=.2, and the support ramps to omega=5 rad/s in one "
      "second.",
      "The exact-FBF headless regression classifies the rider as ejected over "
      "the four-second observation window.",
      "The support is square rather than the paper video's segmented disk; "
      "the ramp, cube properties, camera, and observation horizon are local "
      "reconstruction choices. This scene fixes parameters but is not asset "
      "or snapshot parity.");
}

//==============================================================================
DemoScene makeFbfPaperTurntableMu05Omega5Scene()
{
  return makeFbfPaperTurntableParameterizedScene(
      "fbf_paper_turntable_mu_0_5_omega_5",
      "FBF Paper: Turntable mu=.5, omega=5",
      0.5,
      5.0,
      false,
      "Fixed turntable proxy at mu=.5 and omega=5 rad/s for reproducible "
      "headless capture.",
      "Fixed parameter cell from the paper's turntable grid: the support and "
      "rider both use mu=.5, and the support ramps to omega=5 rad/s in one "
      "second.",
      "The exact-FBF headless regression classifies the rider as ejected over "
      "the four-second observation window.",
      "The support is square rather than the paper video's segmented disk; "
      "the ramp, cube properties, camera, and observation horizon are local "
      "reconstruction choices. This scene fixes parameters but is not asset "
      "or snapshot parity.");
}

//==============================================================================
DemoScene makeFbfAuthorTurntableMu02Omega2Scene()
{
  return makeFbfAuthorTurntableParameterizedScene(
      "FBF Author Turntable: mu=.2, omega=2",
      fbf_author_turntable::kScenarios[0]);
}

//==============================================================================
DemoScene makeFbfAuthorTurntableMu02Omega5Scene()
{
  return makeFbfAuthorTurntableParameterizedScene(
      "FBF Author Turntable: mu=.2, omega=5",
      fbf_author_turntable::kScenarios[1]);
}

//==============================================================================
DemoScene makeFbfAuthorTurntableMu05Omega2Scene()
{
  return makeFbfAuthorTurntableParameterizedScene(
      "FBF Author Turntable: mu=.5, omega=2",
      fbf_author_turntable::kScenarios[2]);
}

//==============================================================================
DemoScene makeFbfAuthorTurntableMu05Omega5Scene()
{
  return makeFbfAuthorTurntableParameterizedScene(
      "FBF Author Turntable: mu=.5, omega=5",
      fbf_author_turntable::kScenarios[3]);
}

//==============================================================================
std::string fbfAuthorTurntablePhysicsContractJson(const std::string& sceneId)
{
  const auto* scenario = fbf_author_turntable::findByDemoScene(sceneId);
  if (scenario == nullptr) {
    throw std::invalid_argument(
        "unknown FBF author-turntable scene id '" + sceneId + "'");
  }

  auto state = std::make_shared<FbfPaperState>();
  const auto world = createAuthorTurntableWorld(
      scenario->demoScene,
      SolverMode::ExactFbf,
      state,
      scenario->friction,
      scenario->angularVelocity);
  return fbf_author_turntable::physicsContractJson(
      fbf_author_turntable::inspectPhysicsContract(
          world,
          *scenario,
          "dart_best",
          "dart_demos",
          DART_FBF_AUTHOR_TURNTABLE_IMPLEMENTATION_SHA256));
}

//==============================================================================
DemoScene makeFbfAuthorCardHouseScene()
{
  return makeFbfAuthorCardHouseParameterizedScene(
      fbf_author_card_house::kDefaultLevelCount);
}

//==============================================================================
DemoScene makeFbfAuthorCardHouse4ImpactCurrentSourceScene()
{
  return makeFbfAuthorCardHouse4ImpactCurrentSourceParameterizedScene();
}

//==============================================================================
DemoScene makeFbfAuthorMasonryArch25CrownImpactCurrentSourceScene()
{
  return makeFbfAuthorMasonryArch25CrownImpactCurrentSourceParameterizedScene();
}

//==============================================================================
std::string fbfAuthorCardHouseConfigurationContractJson(
    const std::string& sceneId)
{
  if (sceneId != fbf_author_card_house::kDemoSceneId) {
    throw std::invalid_argument(
        "unknown FBF author-card-house scene id '" + sceneId + "'");
  }

  const auto world = createAuthorCardHouseWorld(
      sceneId, fbf_author_card_house::kDefaultLevelCount, SolverMode::ExactFbf);
  return fbf_author_card_house::configurationContractJson(
      fbf_author_card_house::inspectConfigurationContract(
          world,
          fbf_author_card_house::kDefaultLevelCount,
          "dart_demos",
          DART_FBF_AUTHOR_CARD_HOUSE_IMPLEMENTATION_SHA256));
}

//==============================================================================
DemoScene makeFbfPaperPainleveScene()
{
  return makeFbfPaperPainleveParameterizedScene(
      "fbf_paper_painleve",
      "FBF Paper: Painleve Proxy mu=.5",
      0.5,
      "DART-side Painleve-style proxy pending the authors' exact scene files.",
      "Fixed mu=.5 Painleve-style sliding-box proxy. Both the ground and box "
      "use friction coefficient 0.5; the authors' exact scene assets and "
      "numerical parameters are unavailable.",
      "At mu=.5, the exact-FBF proxy should slide, slow, and remain upright "
      "over the 2.5-second headless observation window.");
}

//==============================================================================
DemoScene makeFbfPaperPainleveMu055Scene()
{
  return makeFbfPaperPainleveParameterizedScene(
      "fbf_paper_painleve_mu_0_55",
      "FBF Paper: Painleve Proxy mu=.55",
      0.55,
      "Fixed mu=.55 DART-side Painleve-style proxy for reproducible headless "
      "capture.",
      "Fixed mu=.55 Painleve-style sliding-box proxy. Both the ground and box "
      "use friction coefficient 0.55; the authors' exact scene assets and "
      "numerical parameters are unavailable.",
      "At mu=.55, the exact-FBF proxy should travel less than the mu=.5 cell "
      "and then tumble within the 2.5-second headless observation window.");
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
      "Full natural manifold (96 initial contacts in this reconstruction) "
      "dynamic "
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
      "Full natural manifold (96 initial contacts in this reconstruction) "
      "dynamic "
      "26-card scaffold for the paper's four-level house-of-cards scene.",
      "The current exact-FBF smoke runs one bounded step at the full "
      "natural manifold (512-contact cap, 4 contacts per pair, 96 initial "
      "contacts). A separate phase scaffold can drop four cube "
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
            KeyAction{'p', "Drop 4 projectile cubes", [world, state] {
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
      kCardHouseTenLevelMaxContacts,
      kCardHouseTenLevelMaxContactsPerPair,
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
DemoScene makeFbfPaperCardHouse10DynamicScene()
{
  return makeFbfPaperScene(
      "fbf_paper_card_house_10_dynamic",
      "FBF Paper: Card House 10 (Dynamic, Capped)",
      "Dynamic exact-FBF adapter for the reconstructed 155-card, ten-level "
      "house, using an explicitly saturated 512-contact visual budget.",
      CameraHome{
          ::osg::Vec3d(5.0, -7.5, 5.0),
          ::osg::Vec3d(0.0, 0.0, 2.4),
          ::osg::Vec3d(0.0, 0.0, 1.0)},
      [](const auto& state) {
        return createCardHouseTenLevelDynamicWorld(state->solverMode);
      },
      kCardHouseTenLevelMaxContacts,
      kCardHouseTenLevelMaxContactsPerPair,
      true,
      false,
      "Dynamic adapter for the reconstructed triangular ten-level card house: "
      "155 mobile cards, mu=.8, dt=1/60 s, and exact FBF by default. Collision "
      "output is capped at 512 contacts and eight contacts per pair.",
      "This scene exposes actual dynamics and solver diagnostics, but makes no "
      "standing-outcome claim. A visual run is acceptable only when its timed "
      "sidecar reports exact-FBF solves with zero exact failures and zero "
      "boxed-LCP fallbacks for every sampled post-step frame.",
      "The prior boxed-LCP construction probe reached exactly 512 contacts, so "
      "this budget is known to saturate and is not the natural manifold. The "
      "authors' full asset recipe and duration are unavailable; full-manifold "
      "stability, paper timing, realtime performance, and external-solver "
      "parity remain unproven.");
}

//==============================================================================
DemoScene makeFbfPaperMasonryArch25LiteralStandingScene()
{
  return makeFbfPaperScene(
      "fbf_paper_masonry_arch_25_literal_standing",
      "FBF Paper: Masonry Arch 25 (Literal Standing)",
      "Literal-wedge 25-stone standing reconstruction with the shared strict "
      "trace and capture contract.",
      CameraHome{
          ::osg::Vec3d(3.0, -4.0, 2.2),
          ::osg::Vec3d(0.0, 0.0, 0.7),
          ::osg::Vec3d(0.0, 0.0, 1.0)},
      [](const auto& state) {
        return fbf_literal_masonry_arch::createWorld(
            literalMasonryArchSolverLane(state->solverMode),
            fbf_literal_masonry_arch::VisualMode::DemoPalette,
            1u);
      },
      fbf_literal_masonry_arch::kMaxContacts,
      fbf_literal_masonry_arch::kMaxContactsPerPair,
      false,
      false,
      "Literal 25-voussoir reconstruction shared with the text oracle and "
      "off-screen capture: exact convex wedges and uniform-prism inertia, "
      "mu=.8, dt=1/60 s, pinned springers, Native FourPointPlanar contacts, "
      "split impulse, and process-scoped contact ERP=0.",
      "The no-projectile arch should remain standing through the declared "
      "observation window in both exact-FBF and existing boxed-LCP lanes. "
      "Every new exact evidence run fails fast on fallback, solver failure, "
      "an accepted iteration cap, or residual above 1e-6.",
      "This closes a literal standing reconstruction only. It does not add "
      "the paper video's crown impact, prove its unpublished 100-contact "
      "timing scene, reproduce author-code traces, or establish full Fig. 7 "
      "parity.",
      true,
      SolverMode::ExactFbf,
      nullptr,
      [](DemoSceneSetup& setup,
         const WorldPtr& world,
         const std::shared_ptr<FbfPaperState>&) {
        setup.physicsContractProvider = [world] {
          return fbf_literal_masonry_arch::physicsContractJson(
              fbf_literal_masonry_arch::inspectPhysicsContract(world),
              DART_FBF_LITERAL_MASONRY_ARCH_SPEC_SHA256,
              DART_FBF_LITERAL_MASONRY_ARCH_IMPLEMENTATION_SHA256,
              DART_FBF_LITERAL_MASONRY_ARCH_GEOMETRY_SHA256,
              DART_FBF_EXACT_SOLVER_OPTIONS_SHA256);
        };
        setup.onActivate = [](DemoHostContext& context) {
          auto scopedErp = std::make_shared<
              fbf_literal_masonry_arch::ScopedContactErrorReductionParameter>();
          context.addTeardown([scopedErp]() mutable { scopedErp.reset(); });
        };
      },
      [](const WorldPtr& world, SolverMode mode) {
        fbf_literal_masonry_arch::installSolver(
            world,
            literalMasonryArchSolverLane(mode),
            world->getNumSimulationThreads());
      });
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
      "DART reconstruction of the 25-stone masonry arch using source-derived "
      "Rigid-IPC weighted-catenary placement (MIT, commit 23b6ba6), an "
      "oriented-box approximation of each tapered voussoir, uniform friction "
      "0.8, both endpoint stones pinned, and all 23 interior stones dynamic.",
      "The current exact-FBF smoke runs one bounded step with a 48-contact, "
      "two-contacts-per-pair cap and should report zero boxed-LCP fallback. "
      "The Scene tab can drop a reconstructed row of twelve small cubes over "
      "the crown, matching the source video's visible projectile shape and "
      "direction while keeping unpublished numerical values explicit.",
      "This is not literal-wedge or full Fig. 7 parity. The oriented-box "
      "reconstruction has measured 96-contact natural-manifold evidence, "
      "while this GUI uses the reduced 48-contact cap for interactive frame "
      "rates. The separate collision audit owns the literal wedges and exact "
      "uniform-prism inertia; long-run outcome, timing, and source-matched "
      "snapshots remain missing.",
      true,
      SolverMode::ExactFbf,
      renderMasonryArchProjectileControls,
      [](DemoSceneSetup& setup,
         const WorldPtr& world,
         const std::shared_ptr<FbfPaperState>& state) {
        setup.keyActions.push_back(
            KeyAction{'p', "Drop projectile row", [world, state] {
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
      "DART reconstruction of the 101-stone masonry arch using source-derived "
      "Rigid-IPC weighted-catenary placement (MIT, commit 23b6ba6), an "
      "oriented-box approximation of each tapered voussoir, uniform friction "
      "0.8, and both endpoint stones pinned by inference from Fig. 8's "
      "same-setup caption.",
      "The current exact-FBF smoke runs one bounded step with a 38-contact, "
      "two-contacts-per-pair cap and should report zero boxed-LCP fallback.",
      "This is not literal-wedge or full Fig. 8 parity. The oriented-box "
      "reconstruction reaches the 512-contact audit cap, so its natural "
      "manifold count is not established; this GUI uses the reduced "
      "38-contact cap for interactive frame rates. The separate collision "
      "audit owns literal wedges and exact inertia. Long-run balance, timing, "
      "traces, and source-matched snapshots remain missing.");
}

} // namespace dart_demos
