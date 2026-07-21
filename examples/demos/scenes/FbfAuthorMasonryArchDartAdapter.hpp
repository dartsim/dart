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
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
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

#ifndef DART_EXAMPLES_DEMOS_SCENES_FBFAUTHORMASONRYARCHDARTADAPTER_HPP_
#define DART_EXAMPLES_DEMOS_SCENES_FBFAUTHORMASONRYARCHDARTADAPTER_HPP_

#include "FbfAuthorMasonryArchSpec.hpp"

#include <dart/simulation/DeactivationOptions.hpp>
#include <dart/simulation/World.hpp>

#include <dart/constraint/BoxedLcpConstraintSolver.hpp>
#include <dart/constraint/ContactConstraint.hpp>
#include <dart/constraint/ExactCoulombFbfConstraintSolver.hpp>

#include <dart/collision/native/NativeCollisionDetector.hpp>

#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/ConvexMeshShape.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/PlaneShape.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace fbf_author_masonry_arch_adapter {

inline constexpr const char* kContractSchema
    = "dart.fbf_author_masonry_arch_crown_impact_dart_adapter/v1";
inline constexpr const char* kContractKind
    = "source_configuration_dynamics_adapter";
inline constexpr const char* kDemoSceneId
    = "fbf_author_masonry_arch_25_crown_impact_current_source";
inline constexpr std::size_t kEvidenceFrameCount = 500u;
inline constexpr std::size_t kEvidenceTotalSubsteps
    = kEvidenceFrameCount * fbf_author_masonry_arch::kSubstepsPerFrame;
inline constexpr std::size_t kReleaseActionCompletedStep
    = fbf_author_masonry_arch::kReleaseSubstep;
inline constexpr int kReleaseActionKey = 'p';
inline constexpr std::size_t kDartMaxContactsPerPair = 8u;
inline constexpr double kDartContactErrorReductionParameter = 0.0;

// These are deliberately DART adapter choices, not translations of the
// author's Warp/Newton solver. The source solver values remain recorded and
// hash-bound through FbfAuthorMasonryArchSpec.hpp; the observed DART policy is
// serialized lane-specifically in the adapter contract.
inline constexpr int kDartMaxOuterIterations = 5000;
inline constexpr double kDartResidualTolerance = 1e-6;
inline constexpr double kDartStepSizeScale = 35.0;
inline constexpr double kDartOuterRelaxation = 1.1;
inline constexpr int kDartInnerMaxSweeps = 30;
inline constexpr int kDartInnerLocalIterations = 1;

enum class SolverLane
{
  ExactFbf,
  BoxedLcp,
};

//==============================================================================
inline const char* solverLaneLabel(SolverLane lane)
{
  switch (lane) {
    case SolverLane::ExactFbf:
      return "exact_fbf";
    case SolverLane::BoxedLcp:
      return "boxed_lcp";
  }
  throw std::invalid_argument("author masonry-arch solver lane is invalid");
}

/// Process-global ERP override scoped to the active adapter scene.
class ScopedContactErrorReductionParameter
{
public:
  ScopedContactErrorReductionParameter()
    : mPreviousValue(
        dart::constraint::ContactConstraint::getErrorReductionParameter())
  {
    dart::constraint::ContactConstraint::setErrorReductionParameter(
        kDartContactErrorReductionParameter);
  }

  ~ScopedContactErrorReductionParameter()
  {
    dart::constraint::ContactConstraint::setErrorReductionParameter(
        mPreviousValue);
  }

  ScopedContactErrorReductionParameter(
      const ScopedContactErrorReductionParameter&)
      = delete;
  ScopedContactErrorReductionParameter& operator=(
      const ScopedContactErrorReductionParameter&)
      = delete;

private:
  double mPreviousValue;
};

//==============================================================================
inline dart::constraint::ExactCoulombFbfConstraintSolverOptions
makeExactOptions()
{
  dart::constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.fallbackToBoxedLcp = false;
  options.includeConstraintRegularization = false;
  options.useMatrixFreeDelassusOperator = false;
  options.useContactRowDelassusOperator = true;
  options.assembleDenseContactRowSnapshot = false;
  options.enableWarmStart = true;
  options.enableStepSizePersistence = false;
  options.stepSizeRecoveryGrowthFactor = 1.05;
  options.warmStartMatchDistance = 0.025;
  options.seedNormalImpulseFromDiagonal = false;
  options.useMatrixFreeDelassusSeed = false;
  options.enableProjectedGradientRetry = false;
  options.enableDenseResidualPolish = false;
  options.maxOuterIterations = kDartMaxOuterIterations;
  options.acceptOuterMaxIterations = true;
  options.tolerance = kDartResidualTolerance;
  options.initialStepSize = std::numeric_limits<double>::quiet_NaN();
  options.capInitialStepSizeAtSafeBound = true;
  options.stepSizeScale = kDartStepSizeScale;
  options.outerRelaxation = kDartOuterRelaxation;
  options.couplingVariationTolerance = 0.9;
  options.shrinkFactor = 0.7;
  options.maxStepShrinkIterations = 20;
  options.enableAdaptiveStepSize = true;
  options.spectralIterations = 10;
  options.innerMaxSweeps = kDartInnerMaxSweeps;
  options.innerLocalSolver = dart::constraint::ExactCoulombFbfLocalBlockSolver::
      ExactMetricProjection;
  options.runFixedInnerSweeps = true;
  options.acceptInnerMaxIterations = true;
  options.innerLocalIterations = kDartInnerLocalIterations;
  options.innerTolerance = 1e-10;
  options.innerLocalTolerance = 1e-12;
  options.innerDiagonalRegularization = 0.0;
  options.projectedGradientMaxIterations = 400;
  options.projectedGradientTolerance = 1e-12;
  options.denseResidualPolishIterations = 8;
  options.denseResidualPolishLineSearchIterations = 8;
  options.denseResidualPolishRegularization = 1e-9;
  options.maxResidualHistorySamples = 0;
  options.maxResidualHistoryRecords = 0;
  return options;
}

//==============================================================================
inline dart::constraint::ExactCoulombFbfCrossStepPolicyOptions
makeExactCrossStepPolicyOptions()
{
  dart::constraint::ExactCoulombFbfCrossStepPolicyOptions options;
  options.warmStartMatchMode = dart::constraint::
      ExactCoulombFbfWarmStartMatchMode::EitherBodyLocalFeature;
  options.warmStartNormalCosine = 0.9;
  options.useStrictWarmStartMatchDistance = false;
  options.warmStartMaxAge = -1;
  options.persistentStepSizeSafeBoundScale = 1.0;
  options.minimumStepSize = std::numeric_limits<double>::quiet_NaN();
  options.maximumStepSize = std::numeric_limits<double>::quiet_NaN();
  options.warmStartResidualThreshold = std::numeric_limits<double>::quiet_NaN();
  options.warmStartStepSizeCap = std::numeric_limits<double>::quiet_NaN();
  options.persistUncappedStepSizeOnWarmStartCap = false;
  options.requireResidualImprovementForUnconvergedCacheSave = false;
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
    const dart::simulation::WorldPtr& world,
    SolverLane lane,
    std::size_t simulationThreads = 1u)
{
  if (!world)
    throw std::invalid_argument("cannot install author arch solver on null");

  const auto detector = createCollisionDetector();
  if (lane == SolverLane::ExactFbf) {
    auto solver
        = std::make_unique<dart::constraint::ExactCoulombFbfConstraintSolver>(
            makeExactOptions());
    solver->setCollisionDetector(detector);
    solver->setNumSimulationThreads(simulationThreads);
    world->setConstraintSolver(std::move(solver));

    auto* installed
        = static_cast<dart::constraint::ExactCoulombFbfConstraintSolver*>(
            world->getConstraintSolver());
    installed->setExactCoulombCrossStepPolicyOptions(
        makeExactCrossStepPolicyOptions());
    installed->setExactCoulombColoredBlockGaussSeidelEnabled(true);
    installed->setExactCoulombColoredBlockGaussSeidelParticipantAffinityEnabled(
        true);
  } else {
    auto solver
        = std::make_unique<dart::constraint::BoxedLcpConstraintSolver>();
    solver->setCollisionDetector(detector);
    solver->setNumSimulationThreads(simulationThreads);
    world->setConstraintSolver(std::move(solver));
  }

  auto* installed = world->getConstraintSolver();
  installed->setSplitImpulseEnabled(true);
  auto& collisionOption = installed->getCollisionOption();
  collisionOption.maxNumContacts = fbf_author_masonry_arch::kSourceMaxContacts;
  collisionOption.maxNumContactsPerPair = kDartMaxContactsPerPair;
}

//==============================================================================
inline void configureContactDynamics(dart::dynamics::DynamicsAspect* dynamics)
{
  if (!dynamics)
    throw std::invalid_argument("author masonry-arch dynamics is null");
  dynamics->setFrictionCoeff(fbf_author_masonry_arch::kFriction);
  dynamics->setRestitutionCoeff(0.0);
  dynamics->setPrimarySlipCompliance(-1.0);
  dynamics->setSecondarySlipCompliance(-1.0);
  dynamics->setFirstFrictionDirection(Eigen::Vector3d::Zero());
  dynamics->setFirstFrictionDirectionFrame(nullptr);
}

//==============================================================================
inline std::shared_ptr<dart::dynamics::ConvexMeshShape> createStoneShape(
    const fbf_author_masonry_arch::StoneSpec& spec)
{
  dart::dynamics::ConvexMeshShape::Vertices vertices;
  vertices.reserve(spec.localVertices.size());
  for (const auto& vertex : spec.localVertices)
    vertices.push_back(vertex);

  dart::dynamics::ConvexMeshShape::Triangles triangles;
  triangles.reserve(fbf_author_masonry_arch::triangles().size());
  for (const auto& triangle : fbf_author_masonry_arch::triangles()) {
    triangles.emplace_back(
        static_cast<Eigen::Index>(triangle[0]),
        static_cast<Eigen::Index>(triangle[1]),
        static_cast<Eigen::Index>(triangle[2]));
  }
  return std::make_shared<dart::dynamics::ConvexMeshShape>(vertices, triangles);
}

//==============================================================================
inline Eigen::Vector4d stoneColor(std::size_t index)
{
  if (index == 0u || index + 1u == fbf_author_masonry_arch::kStoneCount)
    return Eigen::Vector4d(0.35, 0.23, 0.16, 1.0);
  return index % 2u == 0u ? Eigen::Vector4d(0.82, 0.54, 0.30, 1.0)
                          : Eigen::Vector4d(0.70, 0.39, 0.23, 1.0);
}

//==============================================================================
inline dart::dynamics::SkeletonPtr createGround()
{
  auto skeleton = dart::dynamics::Skeleton::create("ground_plane");
  auto* body = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>()
                   .second;
  auto* collision = body->createShapeNodeWith<
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::PlaneShape>(
          Eigen::Vector3d::UnitZ(), 0.0));
  configureContactDynamics(collision->getDynamicsAspect());

  // The author source uses an infinite plane without renderer metadata. This
  // slab is VisualAspect-only and therefore excluded from the adapter claim.
  auto* visual = body->createShapeNodeWith<dart::dynamics::VisualAspect>(
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d(100.0, 35.0, 0.4)));
  visual->setRelativeTranslation(Eigen::Vector3d(0.0, 0.0, -0.2));
  visual->getVisualAspect()->setRGBA(Eigen::Vector4d(0.34, 0.37, 0.42, 1.0));
  skeleton->setMobile(false);
  return skeleton;
}

//==============================================================================
inline dart::dynamics::SkeletonPtr createStone(
    const fbf_author_masonry_arch::StoneSpec& spec)
{
  auto skeleton = dart::dynamics::Skeleton::create(spec.name);
  auto pair = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
  auto* joint = pair.first;
  auto* body = pair.second;
  auto shape = createStoneShape(spec);
  auto* node = body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(shape);
  node->getVisualAspect()->setRGBA(stoneColor(spec.index));
  configureContactDynamics(node->getDynamicsAspect());

  dart::dynamics::Inertia inertia;
  inertia.setMass(spec.mass);
  inertia.setLocalCOM(spec.localCenterOfMass);
  inertia.setMoment(spec.moment);
  body->setInertia(inertia);
  joint->setPositions(
      dart::dynamics::FreeJoint::convertToPositions(spec.transform));
  skeleton->setMobile(spec.mobile);
  return skeleton;
}

//==============================================================================
inline dart::dynamics::SkeletonPtr createCube(
    const fbf_author_masonry_arch::CubeSpec& spec)
{
  auto skeleton = dart::dynamics::Skeleton::create(spec.name);
  auto pair = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
  auto* joint = pair.first;
  auto* body = pair.second;
  auto shape = std::make_shared<dart::dynamics::BoxShape>(spec.size);
  auto* node = body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(shape);
  const std::array<Eigen::Vector4d, fbf_author_masonry_arch::kCubeCount> colors{
      {Eigen::Vector4d(0.90, 0.48, 0.49, 1.0),
       Eigen::Vector4d(0.91, 0.62, 0.27, 1.0),
       Eigen::Vector4d(0.49, 0.61, 0.67, 1.0)}};
  node->getVisualAspect()->setRGBA(colors[spec.index]);
  configureContactDynamics(node->getDynamicsAspect());

  dart::dynamics::Inertia inertia;
  inertia.setMass(spec.mass);
  inertia.setMoment(spec.moment);
  body->setInertia(inertia);
  joint->setPositions(
      dart::dynamics::FreeJoint::convertToPositions(spec.transform));
  skeleton->setMobile(spec.initiallyMobile);
  return skeleton;
}

//==============================================================================
inline dart::simulation::WorldPtr createWorld(
    SolverLane lane, std::size_t simulationThreads = 1u)
{
  auto world = dart::simulation::World::create(kDemoSceneId);
  world->setTimeStep(fbf_author_masonry_arch::kRuntimeTimeStep);
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world->setNumSimulationThreads(simulationThreads);

  dart::simulation::DeactivationOptions deactivation;
  deactivation.mEnabled = false;
  world->setDeactivationOptions(deactivation);

  installSolver(world, lane, simulationThreads);
  world->addSkeleton(createGround());
  for (const auto& stone : fbf_author_masonry_arch::makeStoneSpecs())
    world->addSkeleton(createStone(stone));
  for (const auto& cube : fbf_author_masonry_arch::makeCubeSpecs())
    world->addSkeleton(createCube(cube));
  return world;
}

//==============================================================================
inline bool cubesReleased(const dart::simulation::WorldPtr& world)
{
  if (!world)
    throw std::invalid_argument("author masonry-arch world is null");

  std::size_t mobile = 0u;
  for (const auto& spec : fbf_author_masonry_arch::makeCubeSpecs()) {
    const auto cube = world->getSkeleton(spec.name);
    if (!cube)
      throw std::runtime_error(
          "author masonry-arch cube is missing: " + spec.name);
    mobile += cube->isMobile() ? 1u : 0u;
  }
  if (mobile != 0u && mobile != fbf_author_masonry_arch::kCubeCount)
    throw std::runtime_error("author masonry-arch cubes have mixed mobility");
  return mobile == fbf_author_masonry_arch::kCubeCount;
}

//==============================================================================
inline void releaseCubes(const dart::simulation::WorldPtr& world)
{
  if (cubesReleased(world))
    return;
  for (const auto& spec : fbf_author_masonry_arch::makeCubeSpecs())
    world->getSkeleton(spec.name)->setMobile(true);
}

struct AdapterContract
{
  SolverLane solverLane = SolverLane::BoxedLcp;
  double timeStep = 0.0;
  Eigen::Vector3d gravity = Eigen::Vector3d::Zero();
  std::size_t simulationThreads = 0u;
  bool deactivationEnabled = true;
  std::size_t maxContacts = 0u;
  std::size_t maxContactsPerPair = 0u;
  bool nativeFourPointPlanar = false;
  bool splitImpulseEnabled = false;
  bool exactColoredBlockGaussSeidelEnabled = false;
  bool exactParticipantAffinityEnabled = false;
  std::optional<dart::constraint::ExactCoulombFbfConstraintSolverOptions>
      exactOptions;
  std::optional<dart::constraint::ExactCoulombFbfCrossStepPolicyOptions>
      exactCrossStepOptions;
  double observedContactErrorReductionParameter = 0.0;
  bool cubesAreReleased = false;
  std::size_t stoneCount = 0u;
  std::size_t mobileStoneCount = 0u;
  std::size_t cubeCount = 0u;
};

//==============================================================================
inline void requireNear(double actual, double expected, const std::string& what)
{
  const double scale = std::max({1.0, std::abs(actual), std::abs(expected)});
  if (!std::isfinite(actual) || std::abs(actual - expected) > 1e-11 * scale)
    throw std::runtime_error("author masonry-arch mismatch: " + what);
}

//==============================================================================
inline AdapterContract inspectAdapterContract(
    const dart::simulation::WorldPtr& world)
{
  if (!world || !world->getConstraintSolver())
    throw std::runtime_error("author masonry-arch adapter has no solver");

  AdapterContract contract;
  contract.timeStep = world->getTimeStep();
  contract.gravity = world->getGravity();
  contract.simulationThreads = world->getNumSimulationThreads();
  contract.deactivationEnabled = world->getDeactivationOptions().mEnabled;
  contract.cubesAreReleased = cubesReleased(world);

  auto* exact
      = dynamic_cast<dart::constraint::ExactCoulombFbfConstraintSolver*>(
          world->getConstraintSolver());
  auto* boxed = dynamic_cast<dart::constraint::BoxedLcpConstraintSolver*>(
      world->getConstraintSolver());
  if (exact) {
    contract.solverLane = SolverLane::ExactFbf;
    contract.exactOptions = exact->getExactCoulombOptions();
    contract.exactCrossStepOptions
        = exact->getExactCoulombCrossStepPolicyOptions();
    contract.exactColoredBlockGaussSeidelEnabled
        = exact->getExactCoulombColoredBlockGaussSeidelEnabled();
    contract.exactParticipantAffinityEnabled
        = exact
              ->getExactCoulombColoredBlockGaussSeidelParticipantAffinityEnabled();
  } else if (boxed) {
    contract.solverLane = SolverLane::BoxedLcp;
  } else {
    throw std::runtime_error(
        "author masonry-arch adapter solver is unsupported");
  }

  const auto detector
      = std::dynamic_pointer_cast<dart::collision::NativeCollisionDetector>(
          world->getConstraintSolver()->getCollisionDetector());
  if (!detector)
    throw std::runtime_error(
        "author masonry-arch adapter requires Native collision");
  contract.nativeFourPointPlanar = detector->getContactManifoldMode()
                                   == dart::collision::NativeCollisionDetector::
                                       ContactManifoldMode::FourPointPlanar;
  const auto& collisionOption
      = world->getConstraintSolver()->getCollisionOption();
  contract.maxContacts = collisionOption.maxNumContacts;
  contract.maxContactsPerPair = collisionOption.maxNumContactsPerPair;
  contract.splitImpulseEnabled
      = world->getConstraintSolver()->isSplitImpulseEnabled();
  contract.observedContactErrorReductionParameter
      = dart::constraint::ContactConstraint::getErrorReductionParameter();

  const auto ground = world->getSkeleton("ground_plane");
  if (!ground || ground->isMobile() || ground->getNumBodyNodes() != 1u)
    throw std::runtime_error("author masonry-arch ground is invalid");
  const auto* groundCollision
      = ground->getBodyNode(0u)
            ->getShapeNodeWith<dart::dynamics::CollisionAspect>(0u);
  if (!groundCollision || !groundCollision->getDynamicsAspect()
      || !std::dynamic_pointer_cast<const dart::dynamics::PlaneShape>(
          groundCollision->getShape()))
    throw std::runtime_error("author masonry-arch ground is not a plane");
  requireNear(
      groundCollision->getDynamicsAspect()->getFrictionCoeff(),
      fbf_author_masonry_arch::kFriction,
      "ground friction");

  const auto stoneSpecs = fbf_author_masonry_arch::makeStoneSpecs();
  for (const auto& spec : stoneSpecs) {
    const auto stone = world->getSkeleton(spec.name);
    if (!stone || stone->getNumBodyNodes() != 1u)
      throw std::runtime_error(
          "author masonry-arch stone is missing: " + spec.name);
    const auto* body = stone->getBodyNode(0u);
    const auto* node
        = body->getShapeNodeWith<dart::dynamics::CollisionAspect>(0u);
    const auto shape
        = node ? std::dynamic_pointer_cast<
              const dart::dynamics::ConvexMeshShape>(node->getShape())
               : nullptr;
    if (!shape || !node->getDynamicsAspect())
      throw std::runtime_error("author masonry-arch stone shape is invalid");
    if (shape->getMesh()->getVertices().size() != spec.localVertices.size()
        || shape->getMesh()->getTriangles().size()
               != fbf_author_masonry_arch::triangles().size())
      throw std::runtime_error("author masonry-arch stone mesh changed");
    for (std::size_t index = 0u; index < spec.localVertices.size(); ++index) {
      if (!shape->getMesh()->getVertices()[index].isApprox(
              spec.localVertices[index], 1e-12))
        throw std::runtime_error("author masonry-arch stone vertices changed");
    }
    if (stone->isMobile() != spec.mobile)
      throw std::runtime_error("author masonry-arch stone mobility changed");
    requireNear(body->getInertia().getMass(), spec.mass, "stone mass");
    if (!body->getInertia().getLocalCOM().isApprox(
            spec.localCenterOfMass, 1e-11)
        || !body->getInertia().getMoment().isApprox(spec.moment, 1e-10)
        || !body->getWorldTransform().isApprox(spec.transform, 1e-11))
      throw std::runtime_error("author masonry-arch stone state changed");
    requireNear(
        node->getDynamicsAspect()->getFrictionCoeff(),
        spec.friction,
        "stone friction");
    ++contract.stoneCount;
    contract.mobileStoneCount += stone->isMobile() ? 1u : 0u;
  }

  const auto cubeSpecs = fbf_author_masonry_arch::makeCubeSpecs();
  for (const auto& spec : cubeSpecs) {
    const auto cube = world->getSkeleton(spec.name);
    if (!cube || cube->getNumBodyNodes() != 1u)
      throw std::runtime_error(
          "author masonry-arch cube is missing: " + spec.name);
    const auto* body = cube->getBodyNode(0u);
    const auto* node
        = body->getShapeNodeWith<dart::dynamics::CollisionAspect>(0u);
    const auto shape
        = node ? std::dynamic_pointer_cast<const dart::dynamics::BoxShape>(
              node->getShape())
               : nullptr;
    if (!shape || !node->getDynamicsAspect()
        || !shape->getSize().isApprox(spec.size, 0.0))
      throw std::runtime_error("author masonry-arch cube shape changed");
    requireNear(body->getInertia().getMass(), spec.mass, "cube mass");
    if (!body->getInertia().getMoment().isApprox(spec.moment, 1e-12)
        || !body->getWorldTransform().isApprox(spec.transform, 1e-12)
        || !body->getLinearVelocity().isZero(0.0)
        || !body->getAngularVelocity().isZero(0.0))
      throw std::runtime_error("author masonry-arch cube state changed");
    requireNear(
        node->getDynamicsAspect()->getFrictionCoeff(),
        spec.friction,
        "cube friction");
    ++contract.cubeCount;
  }

  if (world->getNumSkeletons() != 1u + contract.stoneCount + contract.cubeCount)
    throw std::runtime_error("author masonry-arch skeleton inventory changed");
  return contract;
}

//==============================================================================
inline void writeJsonString(std::ostream& out, const std::string& value)
{
  out << '"';
  for (const char character : value) {
    if (character == '"' || character == '\\')
      out << '\\';
    out << character;
  }
  out << '"';
}

//==============================================================================
inline void writeJsonVector(std::ostream& out, const Eigen::Vector3d& value)
{
  out << '[' << value.x() << ',' << value.y() << ',' << value.z() << ']';
}

//==============================================================================
inline void writeJsonFiniteOrNull(std::ostream& out, double value)
{
  if (std::isfinite(value)) {
    out << value;
  } else if (std::isnan(value)) {
    out << "null";
  } else {
    throw std::invalid_argument(
        "author masonry-arch contract contains an infinite option");
  }
}

//==============================================================================
inline const char* localBlockSolverLabel(
    dart::constraint::ExactCoulombFbfLocalBlockSolver solver)
{
  using Solver = dart::constraint::ExactCoulombFbfLocalBlockSolver;
  switch (solver) {
    case Solver::InverseEuclideanProjection:
      return "inverse_euclidean_projection";
    case Solver::ExactMetricProjection:
      return "exact_metric_projection";
    case Solver::ProjectedGradient:
      return "projected_gradient";
  }
  throw std::invalid_argument(
      "author masonry-arch local block solver is invalid");
}

//==============================================================================
inline const char* warmStartMatchModeLabel(
    dart::constraint::ExactCoulombFbfWarmStartMatchMode mode)
{
  using Mode = dart::constraint::ExactCoulombFbfWarmStartMatchMode;
  switch (mode) {
    case Mode::EitherBodyLocalFeature:
      return "either_body_local_feature";
    case Mode::OrderedBodyBLocalFeature:
      return "ordered_body_b_local_feature";
  }
  throw std::invalid_argument(
      "author masonry-arch warm-start match mode is invalid");
}

//==============================================================================
inline std::string adapterContractJson(
    const AdapterContract& contract,
    const std::string& specSourceSha256,
    const std::string& adapterSourceSha256,
    const std::string& implementationSourceSha256)
{
  if (specSourceSha256.empty() || adapterSourceSha256.empty()
      || implementationSourceSha256.empty())
    throw std::invalid_argument("author masonry-arch source hashes are empty");

  std::ostringstream out;
  out << std::setprecision(std::numeric_limits<double>::max_digits10);
  out << "{\"schema_version\":";
  writeJsonString(out, kContractSchema);
  out << ",\"kind\":";
  writeJsonString(out, kContractKind);
  out << ",\"source_binding\":{\"repository\":";
  writeJsonString(out, fbf_author_masonry_arch::kAuthorRepository);
  out << ",\"commit\":";
  writeJsonString(out, fbf_author_masonry_arch::kAuthorCommit);
  out << ",\"run_py_sha256\":";
  writeJsonString(out, fbf_author_masonry_arch::kAuthorRunSha256);
  out << ",\"mesh_tree_sha256\":";
  writeJsonString(out, fbf_author_masonry_arch::kAuthorMeshTreeSha256);
  out << ",\"configuration_spec_sha256\":";
  writeJsonString(out, specSourceSha256);
  out << ",\"dart_adapter_sha256\":";
  writeJsonString(out, adapterSourceSha256);
  out << ",\"demo_implementation_sha256\":";
  writeJsonString(out, implementationSourceSha256);
  out << "},\"source_configuration\":{\"coordinate_scale\":"
      << fbf_author_masonry_arch::kScale
      << ",\"coordinate_units\":\"author_raw_numeric_values\""
         ",\"stones\":"
      << fbf_author_masonry_arch::kStoneCount
      << ",\"fixed_springers\":" << fbf_author_masonry_arch::kFixedSpringerCount
      << ",\"cubes\":" << fbf_author_masonry_arch::kCubeCount
      << ",\"cube_edge\":" << fbf_author_masonry_arch::kCubeEdgeLength
      << ",\"cube_mass\":" << fbf_author_masonry_arch::kCubeMass
      << ",\"friction\":" << fbf_author_masonry_arch::kFriction
      << ",\"contact_gap\":" << fbf_author_masonry_arch::kContactGap
      << ",\"shape_stiffness\":" << fbf_author_masonry_arch::kShapeStiffness
      << ",\"shape_damping\":" << fbf_author_masonry_arch::kShapeDamping
      << ",\"display_time_step_seconds\":"
      << fbf_author_masonry_arch::kDisplayTimeStep << ",\"substeps_per_frame\":"
      << fbf_author_masonry_arch::kSubstepsPerFrame
      << ",\"runtime_time_step_seconds\":"
      << fbf_author_masonry_arch::kRuntimeTimeStep
      << ",\"release_frame\":" << fbf_author_masonry_arch::kDropFrame
      << ",\"release_substep\":" << fbf_author_masonry_arch::kReleaseSubstep
      << ",\"evidence_frames\":" << kEvidenceFrameCount
      << ",\"evidence_substeps\":" << kEvidenceTotalSubsteps
      << "},\"dart_adapter\":{\"scene_id\":";
  writeJsonString(out, kDemoSceneId);
  out << ",\"world\":{\"time_step_seconds\":" << contract.timeStep
      << ",\"gravity_coordinate_units_per_s2\":";
  writeJsonVector(out, contract.gravity);
  out << ",\"simulation_threads\":" << contract.simulationThreads
      << ",\"deactivation_enabled\":"
      << (contract.deactivationEnabled ? "true" : "false")
      << "},\"collision\":{\"detector\":\"native\""
         ",\"contact_manifold\":\"four_point_planar\""
         ",\"observed_four_point_planar\":"
      << (contract.nativeFourPointPlanar ? "true" : "false")
      << ",\"max_contacts\":" << contract.maxContacts
      << ",\"max_contacts_per_pair\":" << contract.maxContactsPerPair
      << "},\"solver\":{\"lane\":";
  writeJsonString(out, solverLaneLabel(contract.solverLane));
  out << ",\"split_impulse_enabled\":"
      << (contract.splitImpulseEnabled ? "true" : "false")
      << ",\"colored_block_gauss_seidel_enabled\":"
      << (contract.exactColoredBlockGaussSeidelEnabled ? "true" : "false")
      << ",\"participant_affinity_enabled\":"
      << (contract.exactParticipantAffinityEnabled ? "true" : "false")
      << ",\"exact_options\":";
  if (!contract.exactOptions) {
    out << "null";
  } else {
    const auto& options = *contract.exactOptions;
    out << "{\"fallback_to_boxed_lcp_enabled\":"
        << (options.fallbackToBoxedLcp ? "true" : "false")
        << ",\"constraint_regularization_enabled\":"
        << (options.includeConstraintRegularization ? "true" : "false")
        << ",\"matrix_free_operator_enabled\":"
        << (options.useMatrixFreeDelassusOperator ? "true" : "false")
        << ",\"contact_row_operator_enabled\":"
        << (options.useContactRowDelassusOperator ? "true" : "false")
        << ",\"dense_contact_row_snapshot_enabled\":"
        << (options.assembleDenseContactRowSnapshot ? "true" : "false")
        << ",\"warm_start_enabled\":"
        << (options.enableWarmStart ? "true" : "false")
        << ",\"step_size_persistence_enabled\":"
        << (options.enableStepSizePersistence ? "true" : "false")
        << ",\"step_size_recovery_growth_factor\":"
        << options.stepSizeRecoveryGrowthFactor
        << ",\"warm_start_match_distance\":" << options.warmStartMatchDistance
        << ",\"diagonal_seed_enabled\":"
        << (options.seedNormalImpulseFromDiagonal ? "true" : "false")
        << ",\"matrix_free_seed_enabled\":"
        << (options.useMatrixFreeDelassusSeed ? "true" : "false")
        << ",\"projected_gradient_retry_enabled\":"
        << (options.enableProjectedGradientRetry ? "true" : "false")
        << ",\"dense_residual_polish_enabled\":"
        << (options.enableDenseResidualPolish ? "true" : "false")
        << ",\"max_outer_iterations\":" << options.maxOuterIterations
        << ",\"accept_outer_max_iterations\":"
        << (options.acceptOuterMaxIterations ? "true" : "false")
        << ",\"tolerance\":" << options.tolerance << ",\"initial_step_size\":";
    writeJsonFiniteOrNull(out, options.initialStepSize);
    out << ",\"cap_initial_step_size_at_safe_bound\":"
        << (options.capInitialStepSizeAtSafeBound ? "true" : "false")
        << ",\"step_size_scale\":" << options.stepSizeScale
        << ",\"outer_relaxation\":" << options.outerRelaxation
        << ",\"coupling_variation_tolerance\":"
        << options.couplingVariationTolerance
        << ",\"shrink_factor\":" << options.shrinkFactor
        << ",\"max_step_shrink_iterations\":" << options.maxStepShrinkIterations
        << ",\"adaptive_step_size_enabled\":"
        << (options.enableAdaptiveStepSize ? "true" : "false")
        << ",\"spectral_iterations\":" << options.spectralIterations
        << ",\"inner_max_sweeps\":" << options.innerMaxSweeps
        << ",\"inner_local_solver\":";
    writeJsonString(out, localBlockSolverLabel(options.innerLocalSolver));
    out << ",\"run_fixed_inner_sweeps\":"
        << (options.runFixedInnerSweeps ? "true" : "false")
        << ",\"accept_inner_max_iterations\":"
        << (options.acceptInnerMaxIterations ? "true" : "false")
        << ",\"inner_local_iterations\":" << options.innerLocalIterations
        << ",\"inner_tolerance\":" << options.innerTolerance
        << ",\"inner_local_tolerance\":" << options.innerLocalTolerance
        << ",\"inner_diagonal_regularization\":"
        << options.innerDiagonalRegularization
        << ",\"projected_gradient_max_iterations\":"
        << options.projectedGradientMaxIterations
        << ",\"projected_gradient_tolerance\":"
        << options.projectedGradientTolerance
        << ",\"dense_residual_polish_iterations\":"
        << options.denseResidualPolishIterations
        << ",\"dense_residual_polish_line_search_iterations\":"
        << options.denseResidualPolishLineSearchIterations
        << ",\"dense_residual_polish_regularization\":"
        << options.denseResidualPolishRegularization
        << ",\"max_residual_history_samples\":"
        << options.maxResidualHistorySamples
        << ",\"max_residual_history_records\":"
        << options.maxResidualHistoryRecords << '}';
  }
  out << ",\"cross_step_options\":";
  if (!contract.exactCrossStepOptions) {
    out << "null";
  } else {
    const auto& options = *contract.exactCrossStepOptions;
    out << "{\"warm_start_match_mode\":";
    writeJsonString(out, warmStartMatchModeLabel(options.warmStartMatchMode));
    out << ",\"warm_start_normal_cosine\":" << options.warmStartNormalCosine
        << ",\"strict_warm_start_match_distance\":"
        << (options.useStrictWarmStartMatchDistance ? "true" : "false")
        << ",\"warm_start_max_age\":" << options.warmStartMaxAge
        << ",\"persistent_step_size_safe_bound_scale\":"
        << options.persistentStepSizeSafeBoundScale
        << ",\"minimum_step_size\":";
    writeJsonFiniteOrNull(out, options.minimumStepSize);
    out << ",\"maximum_step_size\":";
    writeJsonFiniteOrNull(out, options.maximumStepSize);
    out << ",\"warm_start_residual_threshold\":";
    writeJsonFiniteOrNull(out, options.warmStartResidualThreshold);
    out << ",\"warm_start_step_size_cap\":";
    writeJsonFiniteOrNull(out, options.warmStartStepSizeCap);
    out << ",\"persist_uncapped_step_size_on_warm_start_cap\":"
        << (options.persistUncappedStepSizeOnWarmStartCap ? "true" : "false")
        << ",\"require_residual_improvement_for_unconverged_cache_save\":"
        << (options.requireResidualImprovementForUnconvergedCacheSave ? "true"
                                                                      : "false")
        << '}';
  }
  out << "},\"process_state\":{\"observed_contact_erp\":"
      << contract.observedContactErrorReductionParameter
      << "},\"inventory\":{\"stones\":" << contract.stoneCount
      << ",\"mobile_stones\":" << contract.mobileStoneCount
      << ",\"cubes\":" << contract.cubeCount << ",\"cubes_released\":"
      << (contract.cubesAreReleased ? "true" : "false")
      << "},\"schedule\":{\"evidence_runner_action_completed_step\":"
      << kReleaseActionCompletedStep << ",\"release_action_key\":";
  writeJsonString(out, std::string(1u, static_cast<char>(kReleaseActionKey)));
  out << ",\"interactive_action_semantics\":";
  writeJsonString(out, "immediate_on_invocation");
  out << "}},\"claim_boundary\":{"
         "\"source_numeric_geometry_mass_friction_and_initial_state_ported_to_"
         "dart\":true,"
         "\"source_release_action_ported_to_dart\":true,"
         "\"source_release_schedule_declared_for_evidence_runner\":true,"
         "\"interactive_demo_auto_releases_at_source_step\":false,"
         "\"source_collision_semantics_equivalent\":false,"
         "\"source_contact_gap_semantics_equivalent\":false,"
         "\"source_solver_backend_equivalent\":false,"
         "\"source_float32_semantics_equivalent\":false,"
         "\"trajectory_equivalent\":false,"
         "\"physical_outcome_equivalent\":false,"
         "\"fig07_parity\":false,"
         "\"video07_parity\":false,"
         "\"timing_comparable\":false,"
         "\"paper_parity\":false}}";
  return out.str();
}

} // namespace fbf_author_masonry_arch_adapter

#endif // DART_EXAMPLES_DEMOS_SCENES_FBFAUTHORMASONRYARCHDARTADAPTER_HPP_
