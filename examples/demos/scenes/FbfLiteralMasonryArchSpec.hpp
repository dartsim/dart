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

#ifndef DART_EXAMPLES_DEMOS_SCENES_FBFLITERALMASONRYARCHSPEC_HPP_
#define DART_EXAMPLES_DEMOS_SCENES_FBFLITERALMASONRYARCHSPEC_HPP_

#include <dart/simulation/DeactivationOptions.hpp>
#include <dart/simulation/World.hpp>

#include <dart/constraint/BoxedLcpConstraintSolver.hpp>
#include <dart/constraint/ContactConstraint.hpp>
#include <dart/constraint/ExactCoulombFbfConstraintSolver.hpp>

#include <dart/collision/native/NativeCollisionDetector.hpp>

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/ConvexMeshShape.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/PlaneShape.hpp>
#include <dart/dynamics/ShapeNode.hpp>
#include <dart/dynamics/Skeleton.hpp>

#include <dart/math/detail/MasonryArchGeometry.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace fbf_literal_masonry_arch {

// This is the current-source DART literal-wedge reconstruction shared by the
// trace and visual-capture paths. It is not the author's mesh asset or a paper
// parity claim. In particular, the desired zero ERP remains an inspectable
// contract value; callers that own a whole process may apply it explicitly.

inline constexpr const char* kWorldName = "fbf_literal_masonry_arch_25";
inline constexpr double kTimeStep = 1.0 / 60.0;
inline constexpr double kGravity = 9.81;
inline constexpr double kDensity = 1000.0;
inline constexpr double kFriction = 0.8;
inline constexpr std::size_t kStoneCount = 25u;
inline constexpr std::size_t kMobileStoneCount = 23u;
inline constexpr std::size_t kPinnedSpringerCount = 2u;
inline constexpr std::size_t kMaxContacts = 400u;
inline constexpr std::size_t kMaxContactsPerPair = 8u;
inline constexpr double kEndFaceExpansion = 1e-6;
inline constexpr double kDownwardShift = 0.001001;
inline constexpr double kDesiredContactErrorReductionParameter = 0.0;
inline constexpr auto kBarrierGapPolicy
    = dart::math::detail::MasonryArchBarrierGapPolicy::OmitSourceOffsets;

inline constexpr int kMaxOuterIterations = 5000;
inline constexpr double kResidualTolerance = 1e-6;
inline constexpr int kInnerMaxSweeps = 30;
inline constexpr int kInnerLocalIterations = 1;
inline constexpr double kStepSizeScale = 35.0;
inline constexpr double kOuterRelaxation = 1.1;
inline constexpr const char* kExpectedPhysicalGeometryFingerprint
    = "1ff65f2a99ec96d1";

enum class SolverLane
{
  ExactFbf,
  BoxedLcp,
};

enum class VisualMode
{
  None,
  DemoPalette,
};

/// Owns the process-global contact ERP required by the literal arch contract.
///
/// Keep this token alive for the complete interval in which a literal arch
/// world may be stepped. Destruction restores the value observed at
/// construction. World creation remains mutation-free so callers must make
/// this process-wide lifetime explicit.
class ScopedContactErrorReductionParameter
{
public:
  ScopedContactErrorReductionParameter()
    : mPreviousValue(
        dart::constraint::ContactConstraint::getErrorReductionParameter())
  {
    dart::constraint::ContactConstraint::setErrorReductionParameter(
        kDesiredContactErrorReductionParameter);
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
inline const char* solverLaneLabel(SolverLane lane)
{
  switch (lane) {
    case SolverLane::ExactFbf:
      return "exact_fbf";
    case SolverLane::BoxedLcp:
      return "boxed_lcp";
  }
  throw std::invalid_argument("literal masonry arch solver lane is invalid");
}

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
  options.maxOuterIterations = kMaxOuterIterations;
  options.acceptOuterMaxIterations = true;
  options.tolerance = kResidualTolerance;
  options.initialStepSize = std::numeric_limits<double>::quiet_NaN();
  options.capInitialStepSizeAtSafeBound = true;
  options.stepSizeScale = kStepSizeScale;
  options.outerRelaxation = kOuterRelaxation;
  options.couplingVariationTolerance = 0.9;
  options.shrinkFactor = 0.7;
  options.maxStepShrinkIterations = 20;
  options.enableAdaptiveStepSize = true;
  options.spectralIterations = 10;
  options.innerMaxSweeps = kInnerMaxSweeps;
  options.innerLocalSolver = dart::constraint::ExactCoulombFbfLocalBlockSolver::
      ExactMetricProjection;
  options.runFixedInnerSweeps = true;
  options.acceptInnerMaxIterations = true;
  options.innerLocalIterations = kInnerLocalIterations;
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
inline std::vector<dart::math::detail::MasonryArchStoneWedgeGeometry>
makeStoneGeometries()
{
  auto geometries = dart::math::detail::generateMasonryArchStoneWedges(
      kStoneCount, {}, kBarrierGapPolicy, kEndFaceExpansion);
  if (geometries.size() != kStoneCount) {
    throw std::runtime_error(
        "literal masonry arch generator did not produce 25 stones");
  }
  return geometries;
}

//==============================================================================
inline std::shared_ptr<dart::dynamics::ConvexMeshShape> createWedgeShape(
    const dart::math::detail::MasonryArchStoneWedgeGeometry& geometry)
{
  dart::dynamics::ConvexMeshShape::Vertices vertices;
  vertices.reserve(geometry.vertices.size());
  for (const auto& vertex : geometry.vertices)
    vertices.push_back(vertex - geometry.centroid);

  dart::dynamics::ConvexMeshShape::Triangles triangles;
  const auto& sourceTriangles
      = dart::math::detail::getMasonryArchStoneWedgeTriangles();
  triangles.reserve(sourceTriangles.size());
  for (const auto& triangle : sourceTriangles) {
    triangles.emplace_back(
        static_cast<Eigen::Index>(triangle[0]),
        static_cast<Eigen::Index>(triangle[1]),
        static_cast<Eigen::Index>(triangle[2]));
  }
  return std::make_shared<dart::dynamics::ConvexMeshShape>(vertices, triangles);
}

//==============================================================================
inline void configureContactDynamics(dart::dynamics::DynamicsAspect* dynamics)
{
  if (!dynamics)
    throw std::invalid_argument("literal masonry arch dynamics is null");
  dynamics->setFrictionCoeff(kFriction);
  dynamics->setRestitutionCoeff(0.0);
  dynamics->setPrimarySlipCompliance(-1.0);
  dynamics->setSecondarySlipCompliance(-1.0);
  dynamics->setFirstFrictionDirection(Eigen::Vector3d::Zero());
  dynamics->setFirstFrictionDirectionFrame(nullptr);
}

//==============================================================================
inline Eigen::Vector4d stoneColor(std::size_t index)
{
  if (index == 0u || index + 1u == kStoneCount)
    return Eigen::Vector4d(0.38, 0.24, 0.16, 1.0);
  if (index % 2u == 0u)
    return Eigen::Vector4d(0.82, 0.54, 0.30, 1.0);
  return Eigen::Vector4d(0.72, 0.40, 0.22, 1.0);
}

//==============================================================================
inline dart::dynamics::SkeletonPtr createGround(VisualMode visualMode)
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

  if (visualMode == VisualMode::DemoPalette) {
    auto* visual = body->createShapeNodeWith<dart::dynamics::VisualAspect>(
        std::make_shared<dart::dynamics::BoxShape>(
            Eigen::Vector3d(0.9, 0.35, 0.012)));
    visual->setRelativeTranslation(Eigen::Vector3d(0.0, 0.0, -0.006));
    visual->getVisualAspect()->setRGBA(Eigen::Vector4d(0.34, 0.37, 0.42, 1.0));
  }

  skeleton->setMobile(false);
  return skeleton;
}

//==============================================================================
inline dart::dynamics::SkeletonPtr createStone(
    std::size_t index,
    const dart::math::detail::MasonryArchStoneWedgeGeometry& geometry,
    VisualMode visualMode)
{
  if (index >= kStoneCount)
    throw std::out_of_range("literal masonry arch stone index out of range");

  const std::string name = "masonry_arch_stone_" + std::to_string(index);
  auto skeleton = dart::dynamics::Skeleton::create(name);
  dart::dynamics::GenericJoint<dart::math::SE3Space>::Properties
      jointProperties(name + "_joint");
  dart::dynamics::BodyNode::Properties bodyProperties(
      dart::dynamics::BodyNode::AspectProperties(name + "_body"));
  auto pair = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
      nullptr, jointProperties, bodyProperties);
  auto* joint = pair.first;
  auto* body = pair.second;

  const auto shape = createWedgeShape(geometry);
  dart::dynamics::ShapeNode* shapeNode = nullptr;
  if (visualMode == VisualMode::DemoPalette) {
    shapeNode = body->createShapeNodeWith<
        dart::dynamics::VisualAspect,
        dart::dynamics::CollisionAspect,
        dart::dynamics::DynamicsAspect>(shape);
    shapeNode->getVisualAspect()->setRGBA(stoneColor(index));
  } else {
    shapeNode = body->createShapeNodeWith<
        dart::dynamics::CollisionAspect,
        dart::dynamics::DynamicsAspect>(shape);
  }
  shapeNode->setName(name + "_shape");
  configureContactDynamics(shapeNode->getDynamicsAspect());

  const double mass = kDensity * geometry.volume;
  dart::dynamics::Inertia inertia;
  inertia.setMass(mass);
  inertia.setMoment(mass * geometry.momentPerUnitMass);
  body->setInertia(inertia);

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation()
      = geometry.centroid - kDownwardShift * Eigen::Vector3d::UnitZ();
  joint->setPositions(dart::dynamics::FreeJoint::convertToPositions(transform));

  if (index == 0u || index + 1u == kStoneCount)
    skeleton->setMobile(false);
  return skeleton;
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
    std::size_t simulationThreads)
{
  if (!world)
    throw std::invalid_argument("cannot install literal arch solver on null");

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
  collisionOption.maxNumContacts = kMaxContacts;
  collisionOption.maxNumContactsPerPair = kMaxContactsPerPair;
}

//==============================================================================
inline dart::simulation::WorldPtr createWorld(
    SolverLane lane,
    VisualMode visualMode = VisualMode::None,
    std::size_t simulationThreads = 1u)
{
  auto world = dart::simulation::World::create(kWorldName);
  world->setTimeStep(kTimeStep);
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -kGravity));
  world->setNumSimulationThreads(simulationThreads);

  dart::simulation::DeactivationOptions deactivation;
  deactivation.mEnabled = false;
  world->setDeactivationOptions(deactivation);

  installSolver(world, lane, simulationThreads);
  world->addSkeleton(createGround(visualMode));
  const auto geometries = makeStoneGeometries();
  for (std::size_t index = 0u; index < geometries.size(); ++index)
    world->addSkeleton(createStone(index, geometries[index], visualMode));
  return world;
}

struct StonePhysicsContract
{
  std::string name;
  bool mobile = true;
  double friction = 0.0;
  double primaryFriction = 0.0;
  double secondaryFriction = 0.0;
  double restitution = 0.0;
  double primarySlipCompliance = 0.0;
  double secondarySlipCompliance = 0.0;
  Eigen::Vector3d firstFrictionDirection = Eigen::Vector3d::Zero();
  bool usesDefaultFrictionDirectionFrame = false;
  double mass = 0.0;
  Eigen::Vector3d localCenterOfMass = Eigen::Vector3d::Zero();
  Eigen::Matrix3d moment = Eigen::Matrix3d::Zero();
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  Eigen::Vector3d linearVelocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d angularVelocity = Eigen::Vector3d::Zero();
  std::size_t collisionShapeCount = 0u;
  Eigen::Isometry3d collisionShapeRelativeTransform
      = Eigen::Isometry3d::Identity();
  dart::dynamics::ConvexMeshShape::Vertices collisionMeshVertices;
  dart::dynamics::ConvexMeshShape::Triangles collisionMeshTriangles;
  bool hasVisualAspect = false;
  Eigen::Vector4d color = Eigen::Vector4d::Zero();
};

struct DeclaredPhysicsSpec
{
  // These values identify the construction recipe. They are declarations, not
  // values inferred from an arbitrary World by inspectPhysicsContract().
  double density = kDensity;
  double friction = kFriction;
  dart::math::detail::MasonryArchBarrierGapPolicy barrierGapPolicy
      = kBarrierGapPolicy;
  double endFaceExpansion = kEndFaceExpansion;
  double downwardShift = kDownwardShift;
  double contactErrorReductionParameter
      = kDesiredContactErrorReductionParameter;
};

struct PhysicsContract
{
  SolverLane solverLane = SolverLane::BoxedLcp;
  VisualMode visualMode = VisualMode::None;
  std::string worldName;
  double timeStep = 0.0;
  Eigen::Vector3d gravity = Eigen::Vector3d::Zero();
  std::size_t simulationThreads = 0u;
  bool deactivationEnabled = true;
  std::size_t worldSkeletonCount = 0u;
  std::size_t stoneCount = 0u;
  std::size_t mobileStoneCount = 0u;
  std::size_t pinnedStoneCount = 0u;
  DeclaredPhysicsSpec declaredSpec;
  bool groundMobile = true;
  bool groundIsPlane = false;
  double groundFriction = 0.0;
  double groundPrimaryFriction = 0.0;
  double groundSecondaryFriction = 0.0;
  double groundRestitution = 0.0;
  double groundPrimarySlipCompliance = 0.0;
  double groundSecondarySlipCompliance = 0.0;
  Eigen::Vector3d groundFirstFrictionDirection = Eigen::Vector3d::Zero();
  bool groundUsesDefaultFrictionDirectionFrame = false;
  Eigen::Vector3d groundLocalCenterOfMass = Eigen::Vector3d::Zero();
  Eigen::Isometry3d groundBodyTransform = Eigen::Isometry3d::Identity();
  Eigen::Vector3d groundLinearVelocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d groundAngularVelocity = Eigen::Vector3d::Zero();
  std::size_t groundCollisionShapeCount = 0u;
  Eigen::Isometry3d groundCollisionShapeRelativeTransform
      = Eigen::Isometry3d::Identity();
  Eigen::Vector3d groundPlaneNormal = Eigen::Vector3d::Zero();
  double groundPlaneOffset = 0.0;
  bool groundHasVisualFloor = false;
  Eigen::Vector3d visualFloorSize = Eigen::Vector3d::Zero();
  Eigen::Vector3d visualFloorTranslation = Eigen::Vector3d::Zero();
  std::size_t maxContacts = 0u;
  std::size_t maxContactsPerPair = 0u;
  bool nativeCollision = false;
  dart::collision::NativeCollisionDetector::ContactManifoldMode manifoldMode
      = dart::collision::NativeCollisionDetector::ContactManifoldMode::Compact;
  bool splitImpulseEnabled = false;
  bool exactColoredBlockGaussSeidelEnabled = false;
  bool exactParticipantAffinityEnabled = false;
  std::optional<dart::constraint::ExactCoulombFbfConstraintSolverOptions>
      exactOptions;
  std::optional<dart::constraint::ExactCoulombFbfCrossStepPolicyOptions>
      exactCrossStepOptions;
  double observedProcessContactErrorReductionParameter = 0.0;
  std::vector<StonePhysicsContract> stones;
};

//==============================================================================
inline PhysicsContract inspectPhysicsContract(
    const dart::simulation::WorldPtr& world)
{
  if (!world || !world->getConstraintSolver())
    throw std::runtime_error("literal masonry arch contract has no solver");

  PhysicsContract contract;
  contract.worldName = world->getName();
  contract.timeStep = world->getTimeStep();
  contract.gravity = world->getGravity();
  contract.simulationThreads = world->getNumSimulationThreads();
  contract.deactivationEnabled = world->getDeactivationOptions().mEnabled;
  contract.worldSkeletonCount = world->getNumSkeletons();

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
    throw std::runtime_error("literal masonry arch has unsupported solver");
  }

  const auto detector
      = std::dynamic_pointer_cast<dart::collision::NativeCollisionDetector>(
          world->getConstraintSolver()->getCollisionDetector());
  if (!detector) {
    throw std::runtime_error(
        "literal masonry arch requires Native collision detector");
  }
  contract.nativeCollision = true;
  contract.manifoldMode = detector->getContactManifoldMode();
  const auto& collisionOption
      = world->getConstraintSolver()->getCollisionOption();
  contract.maxContacts = collisionOption.maxNumContacts;
  contract.maxContactsPerPair = collisionOption.maxNumContactsPerPair;
  contract.splitImpulseEnabled
      = world->getConstraintSolver()->isSplitImpulseEnabled();
  contract.observedProcessContactErrorReductionParameter
      = dart::constraint::ContactConstraint::getErrorReductionParameter();

  const auto ground = world->getSkeleton("ground_plane");
  if (!ground || ground->getNumBodyNodes() != 1u)
    throw std::runtime_error("literal masonry arch ground is missing");
  const auto* groundBody = ground->getBodyNode(0u);
  const auto* groundCollision
      = groundBody->getShapeNodeWith<dart::dynamics::CollisionAspect>(0u);
  if (!groundCollision || !groundCollision->getDynamicsAspect())
    throw std::runtime_error(
        "literal masonry arch ground collision is missing");
  const auto groundPlane
      = std::dynamic_pointer_cast<const dart::dynamics::PlaneShape>(
          groundCollision->getShape());
  contract.groundMobile = ground->isMobile();
  contract.groundIsPlane = groundPlane != nullptr;
  const auto* groundDynamics = groundCollision->getDynamicsAspect();
  contract.groundFriction = groundDynamics->getFrictionCoeff();
  contract.groundPrimaryFriction = groundDynamics->getPrimaryFrictionCoeff();
  contract.groundSecondaryFriction
      = groundDynamics->getSecondaryFrictionCoeff();
  contract.groundRestitution = groundDynamics->getRestitutionCoeff();
  contract.groundPrimarySlipCompliance
      = groundDynamics->getPrimarySlipCompliance();
  contract.groundSecondarySlipCompliance
      = groundDynamics->getSecondarySlipCompliance();
  contract.groundFirstFrictionDirection
      = groundDynamics->getFirstFrictionDirection();
  contract.groundUsesDefaultFrictionDirectionFrame
      = groundDynamics->getFirstFrictionDirectionFrame() == nullptr;
  contract.groundLocalCenterOfMass = groundBody->getInertia().getLocalCOM();
  contract.groundBodyTransform = groundBody->getWorldTransform();
  contract.groundLinearVelocity = groundBody->getLinearVelocity();
  contract.groundAngularVelocity = groundBody->getAngularVelocity();
  contract.groundCollisionShapeCount
      = groundBody->getNumShapeNodesWith<dart::dynamics::CollisionAspect>();
  contract.groundCollisionShapeRelativeTransform
      = groundCollision->getRelativeTransform();
  if (groundPlane) {
    contract.groundPlaneNormal = groundPlane->getNormal();
    contract.groundPlaneOffset = groundPlane->getOffset();
  }
  const auto* floorVisual
      = groundBody->getShapeNodeWith<dart::dynamics::VisualAspect>(0u);
  contract.groundHasVisualFloor = floorVisual != nullptr;
  if (floorVisual) {
    const auto floorBox
        = std::dynamic_pointer_cast<const dart::dynamics::BoxShape>(
            floorVisual->getShape());
    if (!floorBox)
      throw std::runtime_error(
          "literal masonry arch visual floor is not a box");
    contract.visualFloorSize = floorBox->getSize();
    contract.visualFloorTranslation = floorVisual->getRelativeTranslation();
  }

  contract.stones.reserve(kStoneCount);
  for (std::size_t index = 0u; index < kStoneCount; ++index) {
    const std::string name = "masonry_arch_stone_" + std::to_string(index);
    const auto stone = world->getSkeleton(name);
    if (!stone || stone->getNumBodyNodes() != 1u)
      throw std::runtime_error(
          "literal masonry arch stone is missing: " + name);
    const auto* body = stone->getBodyNode(0u);
    const auto* shapeNode
        = body->getShapeNodeWith<dart::dynamics::CollisionAspect>(0u);
    if (!shapeNode || !shapeNode->getDynamicsAspect()) {
      throw std::runtime_error(
          "literal masonry arch stone collision is missing: " + name);
    }
    const auto mesh
        = std::dynamic_pointer_cast<const dart::dynamics::ConvexMeshShape>(
            shapeNode->getShape());
    if (!mesh)
      throw std::runtime_error("literal masonry arch stone is not convex mesh");

    StonePhysicsContract stoneContract;
    stoneContract.name = name;
    stoneContract.mobile = stone->isMobile();
    const auto* dynamics = shapeNode->getDynamicsAspect();
    stoneContract.friction = dynamics->getFrictionCoeff();
    stoneContract.primaryFriction = dynamics->getPrimaryFrictionCoeff();
    stoneContract.secondaryFriction = dynamics->getSecondaryFrictionCoeff();
    stoneContract.restitution = dynamics->getRestitutionCoeff();
    stoneContract.primarySlipCompliance = dynamics->getPrimarySlipCompliance();
    stoneContract.secondarySlipCompliance
        = dynamics->getSecondarySlipCompliance();
    stoneContract.firstFrictionDirection
        = dynamics->getFirstFrictionDirection();
    stoneContract.usesDefaultFrictionDirectionFrame
        = dynamics->getFirstFrictionDirectionFrame() == nullptr;
    stoneContract.mass = body->getInertia().getMass();
    stoneContract.localCenterOfMass = body->getInertia().getLocalCOM();
    stoneContract.moment = body->getInertia().getMoment();
    stoneContract.transform = body->getWorldTransform();
    stoneContract.linearVelocity = body->getLinearVelocity();
    stoneContract.angularVelocity = body->getAngularVelocity();
    stoneContract.collisionShapeCount
        = body->getNumShapeNodesWith<dart::dynamics::CollisionAspect>();
    stoneContract.collisionShapeRelativeTransform
        = shapeNode->getRelativeTransform();
    stoneContract.collisionMeshVertices = mesh->getMesh()->getVertices();
    stoneContract.collisionMeshTriangles = mesh->getMesh()->getTriangles();
    stoneContract.hasVisualAspect = shapeNode->getVisualAspect() != nullptr;
    if (stoneContract.hasVisualAspect)
      stoneContract.color = shapeNode->getVisualAspect()->getRGBA();
    if (stoneContract.mobile)
      ++contract.mobileStoneCount;
    else
      ++contract.pinnedStoneCount;
    contract.stones.push_back(std::move(stoneContract));
  }
  contract.stoneCount = contract.stones.size();

  const bool allStonesVisual
      = contract.stoneCount > 0u
        && contract.stoneCount
               == static_cast<std::size_t>(std::count_if(
                   contract.stones.begin(),
                   contract.stones.end(),
                   [](const auto& stone) { return stone.hasVisualAspect; }));
  const bool noStonesVisual = std::none_of(
      contract.stones.begin(), contract.stones.end(), [](const auto& stone) {
        return stone.hasVisualAspect;
      });
  if (contract.groundHasVisualFloor && allStonesVisual) {
    contract.visualMode = VisualMode::DemoPalette;
  } else if (!contract.groundHasVisualFloor && noStonesVisual) {
    contract.visualMode = VisualMode::None;
  } else {
    throw std::runtime_error(
        "literal masonry arch visual mode is inconsistent");
  }
  return contract;
}

//==============================================================================
inline void writeContractJsonString(std::ostream& out, const std::string& value)
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
inline void writeContractJsonVector(
    std::ostream& out, const Eigen::Vector3d& value)
{
  out << '[' << value.x() << ',' << value.y() << ',' << value.z() << ']';
}

//==============================================================================
inline void writeContractJsonMatrix(
    std::ostream& out, const Eigen::Matrix3d& value)
{
  out << '[';
  for (Eigen::Index row = 0; row < 3; ++row) {
    if (row > 0)
      out << ',';
    out << '[';
    for (Eigen::Index column = 0; column < 3; ++column) {
      if (column > 0)
        out << ',';
      out << value(row, column);
    }
    out << ']';
  }
  out << ']';
}

//==============================================================================
inline void writeContractJsonPose(
    std::ostream& out, const Eigen::Isometry3d& value)
{
  out << "{\"translation\":";
  writeContractJsonVector(out, value.translation());
  out << ",\"rotation\":";
  writeContractJsonMatrix(out, value.linear());
  out << '}';
}

//==============================================================================
inline const char* manifoldModeLabel(
    dart::collision::NativeCollisionDetector::ContactManifoldMode mode)
{
  using Mode = dart::collision::NativeCollisionDetector::ContactManifoldMode;
  switch (mode) {
    case Mode::Compact:
      return "compact";
    case Mode::FourPointPlanar:
      return "four_point_planar";
  }
  throw std::invalid_argument(
      "literal masonry arch contact manifold mode is invalid");
}

//==============================================================================
inline const char* barrierGapPolicyLabel(
    dart::math::detail::MasonryArchBarrierGapPolicy policy)
{
  using Policy = dart::math::detail::MasonryArchBarrierGapPolicy;
  switch (policy) {
    case Policy::IncludeSourceOffsets:
      return "include_source_offsets";
    case Policy::OmitSourceOffsets:
      return "omit_source_offsets";
  }
  throw std::invalid_argument(
      "literal masonry arch barrier gap policy is invalid");
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
      "literal masonry arch local block solver is invalid");
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
      "literal masonry arch warm-start match mode is invalid");
}

//==============================================================================
inline void writeContractJsonFiniteOrNull(std::ostream& out, double value)
{
  if (std::isfinite(value)) {
    out << value;
  } else if (std::isnan(value)) {
    out << "null";
  } else {
    throw std::invalid_argument(
        "literal masonry arch contract contains an infinite option");
  }
}

/// Stable checksum over the observed non-visual ground and stone state.
///
/// The byte contract is deliberately small and language-independent: unsigned
/// integers are eight-byte little-endian values, finite doubles are quantized
/// to signed integers at 1e-10 resolution and stored as eight-byte
/// little-endian values, strings are length-prefixed UTF-8, and booleans are
/// one byte. The Python evidence validator recomputes this value from the
/// sidecar and compares it with a pinned current-source oracle.
class PhysicalGeometryFingerprint
{
public:
  void appendBoolean(bool value)
  {
    appendByte(value ? 1u : 0u);
  }

  void appendUnsigned(std::uint64_t value)
  {
    for (std::size_t index = 0u; index < sizeof(value); ++index) {
      appendByte(static_cast<std::uint8_t>((value >> (8u * index)) & 0xffu));
    }
  }

  void appendDouble(double value)
  {
    if (!std::isfinite(value)) {
      throw std::invalid_argument(
          "literal masonry arch geometry contains a nonfinite value");
    }
    // The generator uses libm. Quantizing to 1e-10 keeps the fingerprint
    // stable across supported standard libraries while remaining tighter than
    // the published vertex checks and far below any meaningful scene change.
    constexpr double kScale = 1e10;
    const double scaled = value * kScale;
    if (scaled < static_cast<double>(std::numeric_limits<std::int64_t>::min())
        || scaled > static_cast<double>(
               std::numeric_limits<std::int64_t>::max())) {
      throw std::invalid_argument(
          "literal masonry arch geometry exceeds fingerprint range");
    }
    const auto quantized = static_cast<std::int64_t>(std::llround(scaled));
    appendUnsigned(static_cast<std::uint64_t>(quantized));
  }

  void appendString(const std::string& value)
  {
    appendUnsigned(static_cast<std::uint64_t>(value.size()));
    for (const unsigned char character : value)
      appendByte(character);
  }

  std::string value() const
  {
    std::ostringstream out;
    out << std::hex << std::setfill('0') << std::setw(16) << mValue;
    return out.str();
  }

private:
  void appendByte(std::uint8_t value)
  {
    mValue ^= value;
    mValue *= 1099511628211ull;
  }

  std::uint64_t mValue = 14695981039346656037ull;
};

//==============================================================================
inline void appendFingerprintVector(
    PhysicalGeometryFingerprint& fingerprint, const Eigen::Vector3d& value)
{
  for (Eigen::Index index = 0; index < 3; ++index)
    fingerprint.appendDouble(value[index]);
}

//==============================================================================
inline void appendFingerprintMatrix(
    PhysicalGeometryFingerprint& fingerprint, const Eigen::Matrix3d& value)
{
  for (Eigen::Index row = 0; row < 3; ++row) {
    for (Eigen::Index column = 0; column < 3; ++column)
      fingerprint.appendDouble(value(row, column));
  }
}

//==============================================================================
inline void appendFingerprintPose(
    PhysicalGeometryFingerprint& fingerprint, const Eigen::Isometry3d& value)
{
  appendFingerprintVector(fingerprint, value.translation());
  appendFingerprintMatrix(fingerprint, value.linear());
}

//==============================================================================
inline std::string physicalGeometryFingerprint(const PhysicsContract& contract)
{
  PhysicalGeometryFingerprint fingerprint;
  fingerprint.appendString(
      "dart.fbf_literal_masonry_arch_physical_geometry/"
      "fnv1a64_q1e-10_le_v1");
  fingerprint.appendUnsigned(contract.worldSkeletonCount);
  fingerprint.appendBoolean(contract.groundMobile);
  fingerprint.appendBoolean(contract.groundIsPlane);
  fingerprint.appendDouble(contract.groundFriction);
  fingerprint.appendDouble(contract.groundPrimaryFriction);
  fingerprint.appendDouble(contract.groundSecondaryFriction);
  fingerprint.appendDouble(contract.groundRestitution);
  fingerprint.appendDouble(contract.groundPrimarySlipCompliance);
  fingerprint.appendDouble(contract.groundSecondarySlipCompliance);
  appendFingerprintVector(fingerprint, contract.groundFirstFrictionDirection);
  fingerprint.appendBoolean(contract.groundUsesDefaultFrictionDirectionFrame);
  appendFingerprintVector(fingerprint, contract.groundLocalCenterOfMass);
  appendFingerprintPose(fingerprint, contract.groundBodyTransform);
  appendFingerprintVector(fingerprint, contract.groundLinearVelocity);
  appendFingerprintVector(fingerprint, contract.groundAngularVelocity);
  fingerprint.appendUnsigned(contract.groundCollisionShapeCount);
  appendFingerprintPose(
      fingerprint, contract.groundCollisionShapeRelativeTransform);
  appendFingerprintVector(fingerprint, contract.groundPlaneNormal);
  fingerprint.appendDouble(contract.groundPlaneOffset);
  fingerprint.appendUnsigned(contract.stones.size());
  for (const auto& stone : contract.stones) {
    fingerprint.appendString(stone.name);
    fingerprint.appendBoolean(stone.mobile);
    fingerprint.appendDouble(stone.friction);
    fingerprint.appendDouble(stone.primaryFriction);
    fingerprint.appendDouble(stone.secondaryFriction);
    fingerprint.appendDouble(stone.restitution);
    fingerprint.appendDouble(stone.primarySlipCompliance);
    fingerprint.appendDouble(stone.secondarySlipCompliance);
    appendFingerprintVector(fingerprint, stone.firstFrictionDirection);
    fingerprint.appendBoolean(stone.usesDefaultFrictionDirectionFrame);
    fingerprint.appendDouble(stone.mass);
    appendFingerprintVector(fingerprint, stone.localCenterOfMass);
    appendFingerprintMatrix(fingerprint, stone.moment);
    appendFingerprintPose(fingerprint, stone.transform);
    appendFingerprintVector(fingerprint, stone.linearVelocity);
    appendFingerprintVector(fingerprint, stone.angularVelocity);
    fingerprint.appendUnsigned(stone.collisionShapeCount);
    appendFingerprintPose(fingerprint, stone.collisionShapeRelativeTransform);
    fingerprint.appendUnsigned(stone.collisionMeshVertices.size());
    for (const auto& vertex : stone.collisionMeshVertices)
      appendFingerprintVector(fingerprint, vertex);
    fingerprint.appendUnsigned(stone.collisionMeshTriangles.size());
    for (const auto& triangle : stone.collisionMeshTriangles) {
      for (Eigen::Index index = 0; index < 3; ++index) {
        fingerprint.appendUnsigned(static_cast<std::uint64_t>(triangle[index]));
      }
    }
  }
  return fingerprint.value();
}

//==============================================================================
inline std::string physicsContractJson(
    const PhysicsContract& contract,
    const std::string& specSourceSha256,
    const std::string& implementationSourceSha256,
    const std::string& geometrySourceSha256,
    const std::string& solverOptionsSourceSha256)
{
  if (specSourceSha256.empty() || implementationSourceSha256.empty()
      || geometrySourceSha256.empty() || solverOptionsSourceSha256.empty()) {
    throw std::invalid_argument(
        "literal masonry arch source hashes must be nonempty");
  }

  std::ostringstream out;
  out << std::setprecision(std::numeric_limits<double>::max_digits10);
  const std::string geometryFingerprint = physicalGeometryFingerprint(contract);
  if (geometryFingerprint != kExpectedPhysicalGeometryFingerprint) {
    throw std::runtime_error(
        "literal masonry arch observed physical geometry changed");
  }
  out << "{\"schema_version\":"
         "\"dart.fbf_literal_masonry_arch_physics_contract/v1\","
         "\"kind\":\"physics_control\",\"source_binding\":{"
         "\"spec_sha256\":";
  writeContractJsonString(out, specSourceSha256);
  out << ",\"implementation_sha256\":";
  writeContractJsonString(out, implementationSourceSha256);
  out << ",\"geometry_sha256\":";
  writeContractJsonString(out, geometrySourceSha256);
  out << ",\"solver_options_sha256\":";
  writeContractJsonString(out, solverOptionsSourceSha256);
  out << "},\"world\":{\"name\":";
  writeContractJsonString(out, contract.worldName);
  out << ",\"time_step_seconds\":" << contract.timeStep << ",\"gravity_m_s2\":";
  writeContractJsonVector(out, contract.gravity);
  out << ",\"simulation_threads\":" << contract.simulationThreads
      << ",\"deactivation_enabled\":"
      << (contract.deactivationEnabled ? "true" : "false")
      << ",\"skeleton_count\":" << contract.worldSkeletonCount
      << "},\"declared_spec\":{\"density_kg_m3\":"
      << contract.declaredSpec.density
      << ",\"friction\":" << contract.declaredSpec.friction
      << ",\"barrier_gap_policy\":";
  writeContractJsonString(
      out, barrierGapPolicyLabel(contract.declaredSpec.barrierGapPolicy));
  out << ",\"end_face_expansion_m\":" << contract.declaredSpec.endFaceExpansion
      << ",\"downward_shift_m\":" << contract.declaredSpec.downwardShift
      << ",\"contact_erp\":"
      << contract.declaredSpec.contactErrorReductionParameter
      << "},\"collision\":{\"detector\":\"native\","
         "\"contact_manifold\":";
  writeContractJsonString(out, manifoldModeLabel(contract.manifoldMode));
  out << ",\"native_detector_observed\":"
      << (contract.nativeCollision ? "true" : "false")
      << ",\"max_contacts\":" << contract.maxContacts
      << ",\"max_contacts_per_pair\":" << contract.maxContactsPerPair
      << "},\"solver\":{\"lane\":";
  writeContractJsonString(out, solverLaneLabel(contract.solverLane));
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
    writeContractJsonFiniteOrNull(out, options.initialStepSize);
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
    writeContractJsonString(
        out, localBlockSolverLabel(options.innerLocalSolver));
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
    writeContractJsonString(
        out, warmStartMatchModeLabel(options.warmStartMatchMode));
    out << ",\"warm_start_normal_cosine\":" << options.warmStartNormalCosine
        << ",\"strict_warm_start_match_distance\":"
        << (options.useStrictWarmStartMatchDistance ? "true" : "false")
        << ",\"warm_start_max_age\":" << options.warmStartMaxAge
        << ",\"persistent_step_size_safe_bound_scale\":"
        << options.persistentStepSizeSafeBoundScale
        << ",\"minimum_step_size\":";
    writeContractJsonFiniteOrNull(out, options.minimumStepSize);
    out << ",\"maximum_step_size\":";
    writeContractJsonFiniteOrNull(out, options.maximumStepSize);
    out << ",\"warm_start_residual_threshold\":";
    writeContractJsonFiniteOrNull(out, options.warmStartResidualThreshold);
    out << ",\"warm_start_step_size_cap\":";
    writeContractJsonFiniteOrNull(out, options.warmStartStepSizeCap);
    out << ",\"persist_uncapped_step_size_on_warm_start_cap\":"
        << (options.persistUncappedStepSizeOnWarmStartCap ? "true" : "false")
        << ",\"require_residual_improvement_for_unconverged_cache_save\":"
        << (options.requireResidualImprovementForUnconvergedCacheSave ? "true"
                                                                      : "false")
        << '}';
  }
  out << "},\"physical_geometry_fingerprint\":{"
         "\"algorithm\":\"fnv1a64_q1e-10_le_v1\",\"value\":";
  writeContractJsonString(out, geometryFingerprint);
  out << "},\"process_state\":{\"observed_contact_erp\":"
      << contract.observedProcessContactErrorReductionParameter
      << "},\"ground\":{\"mobile\":"
      << (contract.groundMobile ? "true" : "false")
      << ",\"plane_shape_observed\":"
      << (contract.groundIsPlane ? "true" : "false")
      << ",\"friction\":" << contract.groundFriction
      << ",\"primary_friction\":" << contract.groundPrimaryFriction
      << ",\"secondary_friction\":" << contract.groundSecondaryFriction
      << ",\"restitution\":" << contract.groundRestitution
      << ",\"primary_slip_compliance\":" << contract.groundPrimarySlipCompliance
      << ",\"secondary_slip_compliance\":"
      << contract.groundSecondarySlipCompliance
      << ",\"first_friction_direction\":";
  writeContractJsonVector(out, contract.groundFirstFrictionDirection);
  out << ",\"default_friction_direction_frame\":"
      << (contract.groundUsesDefaultFrictionDirectionFrame ? "true" : "false")
      << ",\"local_center_of_mass\":";
  writeContractJsonVector(out, contract.groundLocalCenterOfMass);
  out << ",\"body_pose\":";
  writeContractJsonPose(out, contract.groundBodyTransform);
  out << ",\"linear_velocity\":";
  writeContractJsonVector(out, contract.groundLinearVelocity);
  out << ",\"angular_velocity\":";
  writeContractJsonVector(out, contract.groundAngularVelocity);
  out << ",\"collision_shape_count\":" << contract.groundCollisionShapeCount
      << ",\"collision_shape_local_pose\":";
  writeContractJsonPose(out, contract.groundCollisionShapeRelativeTransform);
  out << ",\"plane_normal\":";
  writeContractJsonVector(out, contract.groundPlaneNormal);
  out << ",\"plane_offset\":" << contract.groundPlaneOffset << "},\"stones\":[";
  for (std::size_t index = 0u; index < contract.stones.size(); ++index) {
    if (index > 0u)
      out << ',';
    const auto& stone = contract.stones[index];
    out << "{\"name\":";
    writeContractJsonString(out, stone.name);
    out << ",\"mobile\":" << (stone.mobile ? "true" : "false")
        << ",\"friction\":" << stone.friction
        << ",\"primary_friction\":" << stone.primaryFriction
        << ",\"secondary_friction\":" << stone.secondaryFriction
        << ",\"restitution\":" << stone.restitution
        << ",\"primary_slip_compliance\":" << stone.primarySlipCompliance
        << ",\"secondary_slip_compliance\":" << stone.secondarySlipCompliance
        << ",\"first_friction_direction\":";
    writeContractJsonVector(out, stone.firstFrictionDirection);
    out << ",\"default_friction_direction_frame\":"
        << (stone.usesDefaultFrictionDirectionFrame ? "true" : "false")
        << ",\"mass_kg\":" << stone.mass << ",\"local_center_of_mass\":";
    writeContractJsonVector(out, stone.localCenterOfMass);
    out << ",\"moment_kg_m2\":";
    writeContractJsonMatrix(out, stone.moment);
    out << ",\"body_pose\":";
    writeContractJsonPose(out, stone.transform);
    out << ",\"linear_velocity\":";
    writeContractJsonVector(out, stone.linearVelocity);
    out << ",\"angular_velocity\":";
    writeContractJsonVector(out, stone.angularVelocity);
    out << ",\"collision_shape_count\":" << stone.collisionShapeCount
        << ",\"collision_shape_local_pose\":";
    writeContractJsonPose(out, stone.collisionShapeRelativeTransform);
    out << ",\"vertices\":[";
    for (std::size_t vertex = 0u; vertex < stone.collisionMeshVertices.size();
         ++vertex) {
      if (vertex > 0u)
        out << ',';
      writeContractJsonVector(out, stone.collisionMeshVertices[vertex]);
    }
    out << "],\"triangles\":[";
    for (std::size_t triangle = 0u;
         triangle < stone.collisionMeshTriangles.size();
         ++triangle) {
      if (triangle > 0u)
        out << ',';
      const auto& indices = stone.collisionMeshTriangles[triangle];
      out << '[' << indices[0] << ',' << indices[1] << ',' << indices[2] << ']';
    }
    out << "]}";
  }
  out << "]}";
  return out.str();
}

} // namespace fbf_literal_masonry_arch

#endif // DART_EXAMPLES_DEMOS_SCENES_FBFLITERALMASONRYARCHSPEC_HPP_
