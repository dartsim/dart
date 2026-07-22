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

#ifndef DART_EXAMPLES_DEMOS_SCENES_FBFAUTHORINCLINESPEC_HPP_
#define DART_EXAMPLES_DEMOS_SCENES_FBFAUTHORINCLINESPEC_HPP_

#include <dart/simulation/DeactivationOptions.hpp>
#include <dart/simulation/World.hpp>

#include <dart/constraint/BoxedLcpConstraintSolver.hpp>
#include <dart/constraint/ExactCoulombFbfConstraintSolver.hpp>

#include <dart/collision/native/NativeCollisionDetector.hpp>

#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/Inertia.hpp>
#include <dart/dynamics/ShapeNode.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <cmath>
#include <cstddef>

#ifndef DART_FBF_AUTHOR_INCLINE_SPEC_SHA256
  #error "FBF author-incline consumers must bind the shared spec source hash"
#endif
#ifndef DART_FBF_EXACT_SOLVER_OPTIONS_SHA256
  #error "FBF author-incline consumers must bind the exact-solver options hash"
#endif

namespace fbf_author_incline {

inline constexpr const char* kSceneId
    = "fbf_author_incline_sweep_current_source";
inline constexpr const char* kContractSchema
    = "dart.fbf_author_incline_sweep_dynamics_adapter/v1";
inline constexpr const char* kContractKind
    = "current_source_configuration_dynamics_adapter";
inline constexpr const char* kAuthorRepository
    = "https://github.com/matthcsong/fbf-sca-2026";
inline constexpr const char* kAuthorCommit
    = "b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0";
inline constexpr const char* kAuthorTree
    = "ffcdafb61adeda2239c8366d054b548b50d26685";
inline constexpr const char* kAuthorRunPath
    = "paper_examples/cube-on-incline/run.py";
inline constexpr const char* kAuthorRunBlob
    = "63cfc28dca1f6c65ce4a27dbfa239cba154580c6";
inline constexpr const char* kAuthorRunSha256
    = "881d486f25d85f9ae197bf4164e110b6bc39775c498433f27ec4407ca09ebf82";
inline constexpr double kSourceDefaultFriction = 0.5;
inline constexpr const char* kMuGridProvenanceKind
    = "operator_selected_figure_1_grid";
inline constexpr const char* kMuGridSelectionInterface = "supported_--mu_cli";
inline constexpr const char* kSpecSourceSha256
    = DART_FBF_AUTHOR_INCLINE_SPEC_SHA256;
inline constexpr const char* kExactSolverOptionsSha256
    = DART_FBF_EXACT_SOLVER_OPTIONS_SHA256;

inline constexpr double kGravity = 9.81;
inline constexpr double kInclineTangent = 0.5;
inline constexpr double kInclineAngleRadians
    = 0.463647609000806116214256231461214402;
inline constexpr double kCubeHalfExtent = 0.5;
inline constexpr double kCubeDensity = 1000.0;
// The source places the cube center HALF+GAP from the plane, where GAP=.001.
inline constexpr double kInitialGeometricSeparation = 0.001;
inline constexpr double kInitialCenterNormalDistance
    = kCubeHalfExtent + kInitialGeometricSeparation;
// This is a separate source collision-shape setting. DART does not implement
// Newton/Warp's shape-gap semantics in this adapter.
inline constexpr double kSourceCubeShapeGap = 0.01;
inline constexpr double kPlaneHalfExtentX = 5.0;
inline constexpr double kPlaneHalfExtentY = 1.5;
inline constexpr double kPlaneHalfExtentZ = 0.05;
inline constexpr double kTimeStep = 1.0 / 60.0;
inline constexpr double kDuration = 2.0;
inline constexpr std::size_t kTotalSteps = 120u;

// The source runs each mu independently. The DART demo translates otherwise
// identical cells only along Y so all seven can be inspected simultaneously.
inline constexpr double kLaneSpacing = 3.4;
inline constexpr std::size_t kCellCount = 7u;
inline constexpr std::size_t kMaxContacts = 4u * kCellCount;
inline constexpr std::size_t kMaxContactsPerPair = 4u;

// These are DART adapter settings, not claims about the source FBF backend.
inline constexpr int kDartMaxOuterIterations = 500;
inline constexpr double kDartTolerance = 1.0e-6;
inline constexpr int kDartInnerMaxSweeps = 120;
inline constexpr int kDartInnerLocalIterations = 32;
inline constexpr double kDartStepSizeScale = 2.0;
inline constexpr bool kDartFallbackToBoxedLcpEnabled = false;
inline constexpr const char* kDartSolverPolicy
    = "existing_small_fixture_exact_fbf_adapter";

enum class SolverLane
{
  ExactFbf,
  BoxedLcp,
};

struct CellSpec
{
  std::size_t index;
  double friction;
  const char* statePrefix;
  const char* planeSkeletonName;
  const char* planeBodyName;
  const char* cubeSkeletonName;
  const char* cubeBodyName;
  double laneTranslationY;
};

inline constexpr std::array<CellSpec, kCellCount> kCells{{
    {0u,
     0.3,
     "mu_0_3",
     "author_incline_plane_mu_0_3",
     "author_incline_plane_mu_0_3_body",
     "author_incline_cube_mu_0_3",
     "author_incline_cube_mu_0_3_body",
     -3.0 * kLaneSpacing},
    {1u,
     0.4,
     "mu_0_4",
     "author_incline_plane_mu_0_4",
     "author_incline_plane_mu_0_4_body",
     "author_incline_cube_mu_0_4",
     "author_incline_cube_mu_0_4_body",
     -2.0 * kLaneSpacing},
    {2u,
     0.45,
     "mu_0_45",
     "author_incline_plane_mu_0_45",
     "author_incline_plane_mu_0_45_body",
     "author_incline_cube_mu_0_45",
     "author_incline_cube_mu_0_45_body",
     -kLaneSpacing},
    {3u,
     0.5,
     "mu_0_5",
     "author_incline_plane_mu_0_5",
     "author_incline_plane_mu_0_5_body",
     "author_incline_cube_mu_0_5",
     "author_incline_cube_mu_0_5_body",
     0.0},
    {4u,
     0.55,
     "mu_0_55",
     "author_incline_plane_mu_0_55",
     "author_incline_plane_mu_0_55_body",
     "author_incline_cube_mu_0_55",
     "author_incline_cube_mu_0_55_body",
     kLaneSpacing},
    {5u,
     0.6,
     "mu_0_6",
     "author_incline_plane_mu_0_6",
     "author_incline_plane_mu_0_6_body",
     "author_incline_cube_mu_0_6",
     "author_incline_cube_mu_0_6_body",
     2.0 * kLaneSpacing},
    {6u,
     0.8,
     "mu_0_8",
     "author_incline_plane_mu_0_8",
     "author_incline_plane_mu_0_8_body",
     "author_incline_cube_mu_0_8",
     "author_incline_cube_mu_0_8_body",
     3.0 * kLaneSpacing},
}};

//==============================================================================
inline const CellSpec* findCellByCubeSkeletonName(const std::string& name)
{
  for (const auto& cell : kCells) {
    if (name == cell.cubeSkeletonName)
      return &cell;
  }
  return nullptr;
}

//==============================================================================
inline Eigen::Vector3d planeSize()
{
  return Eigen::Vector3d(
      2.0 * kPlaneHalfExtentX,
      2.0 * kPlaneHalfExtentY,
      2.0 * kPlaneHalfExtentZ);
}

//==============================================================================
inline Eigen::Vector3d cubeSize()
{
  return Eigen::Vector3d::Constant(2.0 * kCubeHalfExtent);
}

//==============================================================================
inline constexpr double cubeMass()
{
  return kCubeDensity * 8.0 * kCubeHalfExtent * kCubeHalfExtent
         * kCubeHalfExtent;
}

//==============================================================================
inline Eigen::Matrix3d cubeMoment()
{
  const double diagonal = cubeMass() / 6.0;
  return Eigen::Matrix3d::Identity() * diagonal;
}

//==============================================================================
inline Eigen::Matrix3d sourceRotation()
{
  return Eigen::AngleAxisd(kInclineAngleRadians, Eigen::Vector3d::UnitY())
      .toRotationMatrix();
}

//==============================================================================
inline Eigen::Vector3d slopeTangent()
{
  return Eigen::Vector3d(
      std::cos(kInclineAngleRadians), 0.0, -std::sin(kInclineAngleRadians));
}

//==============================================================================
inline Eigen::Vector3d slopeNormal()
{
  return Eigen::Vector3d(
      std::sin(kInclineAngleRadians), 0.0, std::cos(kInclineAngleRadians));
}

//==============================================================================
inline Eigen::Isometry3d planePose(const CellSpec& cell)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = sourceRotation();
  pose.translation() = Eigen::Vector3d(
      -std::sin(kInclineAngleRadians) * kPlaneHalfExtentZ,
      cell.laneTranslationY,
      -std::cos(kInclineAngleRadians) * kPlaneHalfExtentZ);
  return pose;
}

//==============================================================================
inline Eigen::Isometry3d cubePose(const CellSpec& cell)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = sourceRotation();
  pose.translation() = Eigen::Vector3d(
      std::sin(kInclineAngleRadians) * kInitialCenterNormalDistance,
      cell.laneTranslationY,
      std::cos(kInclineAngleRadians) * kInitialCenterNormalDistance);
  return pose;
}

//==============================================================================
inline dart::constraint::ExactCoulombFbfConstraintSolverOptions
makeExactSolverOptions()
{
  dart::constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.maxOuterIterations = kDartMaxOuterIterations;
  options.tolerance = kDartTolerance;
  options.innerMaxSweeps = kDartInnerMaxSweeps;
  options.innerLocalIterations = kDartInnerLocalIterations;
  options.stepSizeScale = kDartStepSizeScale;
  options.fallbackToBoxedLcp = kDartFallbackToBoxedLcpEnabled;
  return options;
}

//==============================================================================
inline std::shared_ptr<dart::collision::NativeCollisionDetector>
createCollisionDetector()
{
  auto detector = dart::collision::NativeCollisionDetector::create();
  detector->setContactManifoldMode(dart::collision::NativeCollisionDetector::
                                       ContactManifoldMode::FourPointPlanar);
  return detector;
}

//==============================================================================
inline void installSolver(
    const std::shared_ptr<dart::simulation::World>& world, SolverLane lane)
{
  if (!world)
    throw std::invalid_argument("cannot install author incline solver on null");

  const auto detector = createCollisionDetector();
  if (lane == SolverLane::ExactFbf) {
    auto solver
        = std::make_unique<dart::constraint::ExactCoulombFbfConstraintSolver>(
            makeExactSolverOptions());
    solver->setCollisionDetector(detector);
    solver->setNumSimulationThreads(1u);
    world->setConstraintSolver(std::move(solver));
  } else {
    auto solver
        = std::make_unique<dart::constraint::BoxedLcpConstraintSolver>();
    solver->setCollisionDetector(detector);
    solver->setNumSimulationThreads(1u);
    world->setConstraintSolver(std::move(solver));
  }

  auto* installed = world->getConstraintSolver();
  installed->setSplitImpulseEnabled(false);
  auto& collisionOption = installed->getCollisionOption();
  collisionOption.maxNumContacts = kMaxContacts;
  collisionOption.maxNumContactsPerPair = kMaxContactsPerPair;
}

//==============================================================================
inline dart::dynamics::SkeletonPtr createPlane(const CellSpec& cell)
{
  auto skeleton = dart::dynamics::Skeleton::create(cell.planeSkeletonName);
  auto pair = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
  auto* joint = pair.first;
  auto* body = pair.second;
  body->setName(cell.planeBodyName);
  auto* node = body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::BoxShape>(planeSize()));
  node->getVisualAspect()->setRGBA(Eigen::Vector4d(0.55, 0.55, 0.55, 1.0));
  node->getDynamicsAspect()->setFrictionCoeff(cell.friction);
  joint->setPositions(
      dart::dynamics::FreeJoint::convertToPositions(planePose(cell)));
  skeleton->setMobile(false);
  return skeleton;
}

//==============================================================================
inline dart::dynamics::SkeletonPtr createCube(const CellSpec& cell)
{
  auto skeleton = dart::dynamics::Skeleton::create(cell.cubeSkeletonName);
  auto pair = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
  auto* joint = pair.first;
  auto* body = pair.second;
  body->setName(cell.cubeBodyName);
  auto* node = body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::BoxShape>(cubeSize()));
  node->getVisualAspect()->setRGBA(Eigen::Vector4d(0.75, 0.75, 0.75, 1.0));
  node->getDynamicsAspect()->setFrictionCoeff(cell.friction);

  dart::dynamics::Inertia inertia;
  inertia.setMass(cubeMass());
  inertia.setMoment(cubeMoment());
  body->setInertia(inertia);
  joint->setPositions(
      dart::dynamics::FreeJoint::convertToPositions(cubePose(cell)));
  return skeleton;
}

//==============================================================================
inline std::shared_ptr<dart::simulation::World> createWorld(SolverLane lane)
{
  auto world = dart::simulation::World::create(kSceneId);
  world->setTimeStep(kTimeStep);
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -kGravity));
  world->setNumSimulationThreads(1u);

  dart::simulation::DeactivationOptions deactivation;
  deactivation.mEnabled = false;
  world->setDeactivationOptions(deactivation);

  installSolver(world, lane);
  for (const auto& cell : kCells) {
    world->addSkeleton(createPlane(cell));
    world->addSkeleton(createCube(cell));
  }
  return world;
}

//==============================================================================
inline std::vector<std::string> sceneStateFieldNames()
{
  std::vector<std::string> names{"world_time_seconds"};
  constexpr std::array<const char*, 19> kSuffixes{{
      "mu",
      "position_x_m",
      "position_y_m",
      "position_z_m",
      "orientation_w",
      "orientation_x",
      "orientation_y",
      "orientation_z",
      "linear_velocity_x_m_s",
      "linear_velocity_y_m_s",
      "linear_velocity_z_m_s",
      "angular_velocity_x_rad_s",
      "angular_velocity_y_rad_s",
      "angular_velocity_z_rad_s",
      "tangential_displacement_m",
      "tangential_velocity_m_s",
      "normal_distance_m",
      "normal_velocity_m_s",
      "contact_count",
  }};
  names.reserve(1u + kCellCount * kSuffixes.size());
  for (const auto& cell : kCells) {
    for (const char* suffix : kSuffixes)
      names.emplace_back(std::string(cell.statePrefix) + '_' + suffix);
  }
  return names;
}

struct ObservedCell
{
  CellSpec declared;
  std::string planeSkeletonName;
  std::string planeBodyName;
  bool planeMobile = true;
  Eigen::Vector3d planeSize = Eigen::Vector3d::Zero();
  Eigen::Isometry3d planePose = Eigen::Isometry3d::Identity();
  double planeFriction = 0.0;
  std::string cubeSkeletonName;
  std::string cubeBodyName;
  bool cubeMobile = false;
  Eigen::Vector3d cubeSize = Eigen::Vector3d::Zero();
  Eigen::Isometry3d cubePose = Eigen::Isometry3d::Identity();
  Eigen::Vector3d cubeLinearVelocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d cubeAngularVelocity = Eigen::Vector3d::Zero();
  double cubeFriction = 0.0;
  double cubeMass = 0.0;
  Eigen::Matrix3d cubeMoment = Eigen::Matrix3d::Zero();
};

struct PhysicsContract
{
  std::string implementationSourceSha256;
  std::string solverLane;
  double timeStep = 0.0;
  Eigen::Vector3d gravity = Eigen::Vector3d::Zero();
  std::size_t simulationThreads = 0u;
  bool deactivationEnabled = true;
  std::string collisionDetector;
  std::string contactManifold;
  std::size_t maxContacts = 0u;
  std::size_t maxContactsPerPair = 0u;
  bool splitImpulseEnabled = false;
  bool exactOptionsAvailable = false;
  int maxOuterIterations = 0;
  double tolerance = 0.0;
  int innerMaxSweeps = 0;
  int innerLocalIterations = 0;
  double stepSizeScale = 0.0;
  bool fallbackToBoxedLcpEnabled = false;
  std::vector<ObservedCell> cells;
};

//==============================================================================
inline void requireNear(double actual, double expected, const std::string& what)
{
  const double scale = std::max({1.0, std::abs(actual), std::abs(expected)});
  if (!std::isfinite(actual) || std::abs(actual - expected) > 1e-11 * scale) {
    std::ostringstream message;
    message << std::setprecision(std::numeric_limits<double>::max_digits10)
            << "author incline mismatch: " << what << " actual=" << actual
            << " expected=" << expected;
    throw std::runtime_error(message.str());
  }
}

//==============================================================================
inline PhysicsContract inspectPhysicsContract(
    const std::shared_ptr<dart::simulation::World>& world,
    const std::string& implementationSourceSha256)
{
  if (!world || !world->getConstraintSolver())
    throw std::runtime_error("author incline adapter has no solver");
  if (world->getName() != kSceneId)
    throw std::runtime_error("author incline adapter has the wrong world");
  if (implementationSourceSha256.size() != 64u)
    throw std::runtime_error("author incline implementation hash is invalid");

  PhysicsContract contract;
  contract.implementationSourceSha256 = implementationSourceSha256;
  contract.timeStep = world->getTimeStep();
  contract.gravity = world->getGravity();
  contract.simulationThreads = world->getNumSimulationThreads();
  contract.deactivationEnabled = world->getDeactivationOptions().mEnabled;

  auto* exact
      = dynamic_cast<dart::constraint::ExactCoulombFbfConstraintSolver*>(
          world->getConstraintSolver());
  auto* boxed = dynamic_cast<dart::constraint::BoxedLcpConstraintSolver*>(
      world->getConstraintSolver());
  if (exact) {
    contract.solverLane = "exact_fbf";
    contract.exactOptionsAvailable = true;
    const auto& options = exact->getExactCoulombOptions();
    contract.maxOuterIterations = options.maxOuterIterations;
    contract.tolerance = options.tolerance;
    contract.innerMaxSweeps = options.innerMaxSweeps;
    contract.innerLocalIterations = options.innerLocalIterations;
    contract.stepSizeScale = options.stepSizeScale;
    contract.fallbackToBoxedLcpEnabled = options.fallbackToBoxedLcp;
    const bool fallbackPolicyMatches
        = contract.fallbackToBoxedLcpEnabled == kDartFallbackToBoxedLcpEnabled;
    if (contract.maxOuterIterations != kDartMaxOuterIterations
        || contract.innerMaxSweeps != kDartInnerMaxSweeps
        || contract.innerLocalIterations != kDartInnerLocalIterations
        || !fallbackPolicyMatches) {
      throw std::runtime_error("author incline mismatch: exact solver policy");
    }
    requireNear(contract.tolerance, kDartTolerance, "exact tolerance");
    requireNear(
        contract.stepSizeScale, kDartStepSizeScale, "exact step-size scale");
  } else if (boxed) {
    contract.solverLane = "boxed_lcp";
  } else {
    throw std::runtime_error("author incline adapter solver is unsupported");
  }

  const auto detector
      = std::dynamic_pointer_cast<dart::collision::NativeCollisionDetector>(
          world->getConstraintSolver()->getCollisionDetector());
  if (!detector)
    throw std::runtime_error(
        "author incline adapter requires Native collision");
  contract.collisionDetector = detector->getType();
  contract.contactManifold
      = detector->getContactManifoldMode()
                == dart::collision::NativeCollisionDetector::
                    ContactManifoldMode::FourPointPlanar
            ? "four_point_planar"
            : "compact";
  const auto& collisionOption
      = world->getConstraintSolver()->getCollisionOption();
  contract.maxContacts = collisionOption.maxNumContacts;
  contract.maxContactsPerPair = collisionOption.maxNumContactsPerPair;
  contract.splitImpulseEnabled
      = world->getConstraintSolver()->isSplitImpulseEnabled();

  requireNear(contract.timeStep, kTimeStep, "time step");
  if (!contract.gravity.isApprox(Eigen::Vector3d(0.0, 0.0, -kGravity), 1e-12))
    throw std::runtime_error("author incline mismatch: gravity");
  if (contract.simulationThreads != 1u || contract.deactivationEnabled)
    throw std::runtime_error("author incline mismatch: world policy");
  if (contract.contactManifold != "four_point_planar")
    throw std::runtime_error("author incline mismatch: contact manifold");
  if (contract.maxContacts != kMaxContacts
      || contract.maxContactsPerPair != kMaxContactsPerPair) {
    throw std::runtime_error("author incline mismatch: contact caps");
  }
  if (world->getNumSkeletons() != 2u * kCellCount)
    throw std::runtime_error("author incline mismatch: skeleton inventory");

  contract.cells.reserve(kCellCount);
  for (const auto& cell : kCells) {
    const auto plane = world->getSkeleton(cell.planeSkeletonName);
    const auto cube = world->getSkeleton(cell.cubeSkeletonName);
    if (!plane || !cube || plane->getNumBodyNodes() != 1u
        || cube->getNumBodyNodes() != 1u) {
      throw std::runtime_error(
          std::string("author incline bodies are missing: ")
          + cell.statePrefix);
    }
    const auto* planeBody = plane->getBodyNode(0u);
    const auto* cubeBody = cube->getBodyNode(0u);
    const auto* planeNode
        = planeBody->getShapeNodeWith<dart::dynamics::CollisionAspect>(0u);
    const auto* cubeNode
        = cubeBody->getShapeNodeWith<dart::dynamics::CollisionAspect>(0u);
    if (!planeNode || !cubeNode || !planeNode->getDynamicsAspect()
        || !cubeNode->getDynamicsAspect()) {
      throw std::runtime_error("author incline collision shapes are missing");
    }
    const auto planeBox
        = std::dynamic_pointer_cast<const dart::dynamics::BoxShape>(
            planeNode->getShape());
    const auto dynamicBox
        = std::dynamic_pointer_cast<const dart::dynamics::BoxShape>(
            cubeNode->getShape());
    if (!planeBox || !dynamicBox)
      throw std::runtime_error("author incline shapes are not boxes");

    ObservedCell observed{};
    observed.declared = cell;
    observed.planeSkeletonName = plane->getName();
    observed.planeBodyName = planeBody->getName();
    observed.planeMobile = plane->isMobile();
    observed.planeSize = planeBox->getSize();
    observed.planePose = planeBody->getWorldTransform();
    observed.planeFriction = planeNode->getDynamicsAspect()->getFrictionCoeff();
    observed.cubeSkeletonName = cube->getName();
    observed.cubeBodyName = cubeBody->getName();
    observed.cubeMobile = cube->isMobile();
    observed.cubeSize = dynamicBox->getSize();
    observed.cubePose = cubeBody->getWorldTransform();
    observed.cubeLinearVelocity = cubeBody->getLinearVelocity();
    observed.cubeAngularVelocity = cubeBody->getAngularVelocity();
    observed.cubeFriction = cubeNode->getDynamicsAspect()->getFrictionCoeff();
    observed.cubeMass = cubeBody->getInertia().getMass();
    observed.cubeMoment = cubeBody->getInertia().getMoment();

    if (observed.planeSkeletonName != cell.planeSkeletonName
        || observed.planeBodyName != cell.planeBodyName
        || observed.cubeSkeletonName != cell.cubeSkeletonName
        || observed.cubeBodyName != cell.cubeBodyName) {
      throw std::runtime_error("author incline mismatch: body naming");
    }
    if (observed.planeMobile || !observed.planeSize.isApprox(planeSize(), 1e-12)
        || !observed.planePose.matrix().isApprox(
            fbf_author_incline::planePose(cell).matrix(), 1e-12)) {
      throw std::runtime_error("author incline mismatch: plane geometry");
    }
    requireNear(observed.planeFriction, cell.friction, "plane friction");
    if (!observed.cubeMobile || !observed.cubeSize.isApprox(cubeSize(), 1e-12)
        || !observed.cubePose.matrix().isApprox(
            fbf_author_incline::cubePose(cell).matrix(), 1e-12)
        || !observed.cubeLinearVelocity.isZero(1e-12)
        || !observed.cubeAngularVelocity.isZero(1e-12)) {
      throw std::runtime_error("author incline mismatch: cube initial state");
    }
    requireNear(observed.cubeFriction, cell.friction, "cube friction");
    requireNear(observed.cubeMass, cubeMass(), "cube mass");
    if (!observed.cubeMoment.isApprox(cubeMoment(), 1e-12))
      throw std::runtime_error("author incline mismatch: cube moment");
    contract.cells.push_back(std::move(observed));
  }
  return contract;
}

//==============================================================================
inline void writeJsonString(std::ostream& out, const std::string& value)
{
  out << '"';
  for (const char c : value) {
    if (c == '"' || c == '\\')
      out << '\\';
    out << c;
  }
  out << '"';
}

//==============================================================================
inline void writeJsonVector(std::ostream& out, const Eigen::Vector3d& value)
{
  out << '[' << value.x() << ',' << value.y() << ',' << value.z() << ']';
}

//==============================================================================
inline void writeJsonMatrix(std::ostream& out, const Eigen::Matrix3d& value)
{
  out << '[';
  for (int row = 0; row < 3; ++row) {
    if (row != 0)
      out << ',';
    out << '[';
    for (int column = 0; column < 3; ++column) {
      if (column != 0)
        out << ',';
      out << value(row, column);
    }
    out << ']';
  }
  out << ']';
}

//==============================================================================
inline void writeJsonPose(std::ostream& out, const Eigen::Isometry3d& value)
{
  out << "{\"translation\":";
  writeJsonVector(out, value.translation());
  out << ",\"rotation\":";
  writeJsonMatrix(out, value.linear());
  out << '}';
}

//==============================================================================
inline std::string physicsContractJson(const PhysicsContract& contract)
{
  if (contract.cells.size() != kCellCount)
    throw std::runtime_error("author incline contract cell inventory changed");

  std::ostringstream out;
  out << std::setprecision(std::numeric_limits<double>::max_digits10);
  out << "{\"schema_version\":";
  writeJsonString(out, kContractSchema);
  out << ",\"kind\":";
  writeJsonString(out, kContractKind);
  out << ",\"source_binding\":{\"repository\":";
  writeJsonString(out, kAuthorRepository);
  out << ",\"commit\":";
  writeJsonString(out, kAuthorCommit);
  out << ",\"tree\":";
  writeJsonString(out, kAuthorTree);
  out << ",\"runner_path\":";
  writeJsonString(out, kAuthorRunPath);
  out << ",\"runner_blob\":";
  writeJsonString(out, kAuthorRunBlob);
  out << ",\"runner_sha256\":";
  writeJsonString(out, kAuthorRunSha256);
  out << ",\"configuration_spec_sha256\":";
  writeJsonString(out, kSpecSourceSha256);
  out << ",\"exact_solver_options_sha256\":";
  writeJsonString(out, kExactSolverOptionsSha256);
  out << ",\"demo_implementation_sha256\":";
  writeJsonString(out, contract.implementationSourceSha256);
  out << "},\"source_configuration\":{"
         "\"mu_values\":[0.3,0.4,0.45,0.5,0.55,0.6,0.8],"
         "\"mu_grid_provenance\":{\"kind\":";
  writeJsonString(out, kMuGridProvenanceKind);
  out << ",\"source_default_mu\":" << kSourceDefaultFriction
      << ",\"selection_interface\":";
  writeJsonString(out, kMuGridSelectionInterface);
  out << ",\"source_runs_each_mu_independently\":true},"
         "\"tan_theta\":0.5,\"theta_rad\":"
      << kInclineAngleRadians << ",\"gravity_m_s2\":" << kGravity
      << ",\"plane_half_extents_m\":[5,1.5,0.05],"
         "\"plane_full_dimensions_m\":[10,3,0.1],"
         "\"cube_half_extent_m\":0.5,\"cube_full_dimensions_m\":[1,1,1],"
         "\"cube_density_kg_m3\":1000,\"cube_mass_kg\":"
      << cubeMass()
      << ",\"initial_geometric_separation_m\":0.001,"
         "\"initial_center_normal_distance_m\":0.501,"
         "\"source_cube_shape_gap_m\":0.01,"
         "\"time_step_seconds\":"
      << kTimeStep << ",\"duration_seconds\":" << kDuration
      << ",\"total_steps\":" << kTotalSteps
      << "},\"dart_adapter\":{\"scene_id\":";
  writeJsonString(out, kSceneId);
  out << ",\"world\":{\"time_step_seconds\":" << contract.timeStep
      << ",\"gravity_m_s2\":";
  writeJsonVector(out, contract.gravity);
  out << ",\"simulation_threads\":" << contract.simulationThreads
      << ",\"deactivation_enabled\":"
      << (contract.deactivationEnabled ? "true" : "false")
      << "},\"layout\":{\"kind\":\"simultaneous_y_translated_lanes\","
         "\"lane_spacing_m\":"
      << kLaneSpacing
      << ",\"source_cells_run_independently\":true,"
         "\"translation_axis\":\"y\",\"geometry_otherwise_unchanged\":true},"
         "\"collision\":{\"detector\":";
  writeJsonString(out, contract.collisionDetector);
  out << ",\"contact_manifold\":";
  writeJsonString(out, contract.contactManifold);
  out << ",\"max_contacts\":" << contract.maxContacts
      << ",\"max_contacts_per_pair\":" << contract.maxContactsPerPair
      << "},\"solver\":{\"lane\":";
  writeJsonString(out, contract.solverLane);
  out << ",\"configuration_policy\":";
  writeJsonString(out, kDartSolverPolicy);
  out << ",\"split_impulse_enabled\":"
      << (contract.splitImpulseEnabled ? "true" : "false")
      << ",\"exact_options\":";
  if (!contract.exactOptionsAvailable) {
    out << "null";
  } else {
    out << "{\"max_outer_iterations\":" << contract.maxOuterIterations
        << ",\"tolerance\":" << contract.tolerance
        << ",\"inner_max_sweeps\":" << contract.innerMaxSweeps
        << ",\"inner_local_iterations\":" << contract.innerLocalIterations
        << ",\"step_size_scale\":" << contract.stepSizeScale
        << ",\"fallback_to_boxed_lcp_enabled\":"
        << (contract.fallbackToBoxedLcpEnabled ? "true" : "false") << '}';
  }
  out << "},\"cells\":[";
  for (std::size_t index = 0u; index < contract.cells.size(); ++index) {
    if (index != 0u)
      out << ',';
    const auto& cell = contract.cells[index];
    out << "{\"index\":" << cell.declared.index
        << ",\"mu\":" << cell.declared.friction << ",\"state_prefix\":";
    writeJsonString(out, cell.declared.statePrefix);
    out << ",\"lane_translation_y_m\":" << cell.declared.laneTranslationY
        << ",\"plane\":{\"skeleton_name\":";
    writeJsonString(out, cell.planeSkeletonName);
    out << ",\"body_name\":";
    writeJsonString(out, cell.planeBodyName);
    out << ",\"mobile\":" << (cell.planeMobile ? "true" : "false")
        << ",\"size_m\":";
    writeJsonVector(out, cell.planeSize);
    out << ",\"initial_pose\":";
    writeJsonPose(out, cell.planePose);
    out << ",\"friction\":" << cell.planeFriction
        << "},\"cube\":{\"skeleton_name\":";
    writeJsonString(out, cell.cubeSkeletonName);
    out << ",\"body_name\":";
    writeJsonString(out, cell.cubeBodyName);
    out << ",\"mobile\":" << (cell.cubeMobile ? "true" : "false")
        << ",\"size_m\":";
    writeJsonVector(out, cell.cubeSize);
    out << ",\"initial_pose\":";
    writeJsonPose(out, cell.cubePose);
    out << ",\"initial_linear_velocity_m_s\":";
    writeJsonVector(out, cell.cubeLinearVelocity);
    out << ",\"initial_angular_velocity_rad_s\":";
    writeJsonVector(out, cell.cubeAngularVelocity);
    out << ",\"friction\":" << cell.cubeFriction
        << ",\"mass_kg\":" << cell.cubeMass << ",\"moment_kg_m2\":";
    writeJsonMatrix(out, cell.cubeMoment);
    out << "}}";
  }
  out << "]},\"adapter_boundaries\":{"
         "\"source_initial_geometric_separation_represented\":true,"
         "\"source_shape_gap_semantics_implemented\":false,"
         "\"source_collision_backend_implemented\":false,"
         "\"source_solver_backend_semantics_implemented\":false,"
         "\"source_float32_semantics_implemented\":false,"
         "\"dart_native_four_point_planar_is_adapter_choice\":true,"
         "\"seven_cells_run_simultaneously_in_source\":false},"
         "\"claim_boundary\":{"
         "\"current_source_geometry_clock_mu_grid_port\":true,"
         "\"historical_paper_invocation_known\":false,"
         "\"trajectory_valid\":false,\"physical_outcome_valid\":false,"
         "\"trajectory_equivalence\":false,\"solver_equivalence\":false,"
         "\"physical_outcome_equivalence\":false,\"video_parity\":false,"
         "\"timing_comparability\":false,\"paper_parity\":false}}";
  return out.str();
}

//==============================================================================
inline std::vector<std::pair<std::string, double>> sceneStateFields(
    const std::shared_ptr<dart::simulation::World>& world)
{
  if (!world || world->getName() != kSceneId)
    throw std::runtime_error("author incline scene state has the wrong world");

  const double worldTime = world->getTime();
  if (!std::isfinite(worldTime))
    throw std::runtime_error("author incline scene time is non-finite");
  std::vector<std::pair<std::string, double>> fields;
  fields.reserve(1u + 19u * kCellCount);
  fields.emplace_back("world_time_seconds", worldTime);
  for (const auto& cell : kCells) {
    const auto cube = world->getSkeleton(cell.cubeSkeletonName);
    if (!cube || cube->getNumBodyNodes() != 1u
        || cube->getBodyNode(0u)->getName() != cell.cubeBodyName) {
      throw std::runtime_error(
          std::string("author incline scene state has no cube: ")
          + cell.statePrefix);
    }
    const auto* body = cube->getBodyNode(0u);
    const Eigen::Isometry3d transform = body->getWorldTransform();
    const Eigen::Vector3d position = transform.translation();
    const Eigen::Vector3d velocity = body->getLinearVelocity();
    const Eigen::Vector3d angularVelocity = body->getAngularVelocity();
    Eigen::Quaterniond orientation(transform.linear());
    orientation.normalize();
    if (orientation.w() < 0.0)
      orientation.coeffs() *= -1.0;
    const Eigen::Vector3d displacement
        = position - cubePose(cell).translation();
    const double tangentialDisplacement = displacement.dot(slopeTangent());
    const double tangentialVelocity = velocity.dot(slopeTangent());
    const double normalDistance = position.dot(slopeNormal());
    const double normalVelocity = velocity.dot(slopeNormal());
    std::size_t contactCount = 0u;
    for (const auto& contact : world->getLastCollisionResult().getContacts()) {
      if (contact.getBodyNodePtr1().get() == body
          || contact.getBodyNodePtr2().get() == body) {
        ++contactCount;
      }
    }
    if (!transform.matrix().allFinite() || !velocity.allFinite()
        || !angularVelocity.allFinite() || !orientation.coeffs().allFinite()
        || !std::isfinite(tangentialDisplacement)
        || !std::isfinite(tangentialVelocity) || !std::isfinite(normalDistance)
        || !std::isfinite(normalVelocity)) {
      throw std::runtime_error("author incline scene state is non-finite");
    }

    const std::string prefix(cell.statePrefix);
    fields.emplace_back(prefix + "_mu", cell.friction);
    fields.emplace_back(prefix + "_position_x_m", position.x());
    fields.emplace_back(prefix + "_position_y_m", position.y());
    fields.emplace_back(prefix + "_position_z_m", position.z());
    fields.emplace_back(prefix + "_orientation_w", orientation.w());
    fields.emplace_back(prefix + "_orientation_x", orientation.x());
    fields.emplace_back(prefix + "_orientation_y", orientation.y());
    fields.emplace_back(prefix + "_orientation_z", orientation.z());
    fields.emplace_back(prefix + "_linear_velocity_x_m_s", velocity.x());
    fields.emplace_back(prefix + "_linear_velocity_y_m_s", velocity.y());
    fields.emplace_back(prefix + "_linear_velocity_z_m_s", velocity.z());
    fields.emplace_back(
        prefix + "_angular_velocity_x_rad_s", angularVelocity.x());
    fields.emplace_back(
        prefix + "_angular_velocity_y_rad_s", angularVelocity.y());
    fields.emplace_back(
        prefix + "_angular_velocity_z_rad_s", angularVelocity.z());
    fields.emplace_back(
        prefix + "_tangential_displacement_m", tangentialDisplacement);
    fields.emplace_back(
        prefix + "_tangential_velocity_m_s", tangentialVelocity);
    fields.emplace_back(prefix + "_normal_distance_m", normalDistance);
    fields.emplace_back(prefix + "_normal_velocity_m_s", normalVelocity);
    fields.emplace_back(prefix + "_contact_count", contactCount);
  }
  return fields;
}

} // namespace fbf_author_incline

#endif // DART_EXAMPLES_DEMOS_SCENES_FBFAUTHORINCLINESPEC_HPP_
