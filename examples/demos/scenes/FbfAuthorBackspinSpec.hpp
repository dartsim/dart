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

#ifndef DART_EXAMPLES_DEMOS_SCENES_FBFAUTHORBACKSPINSPEC_HPP_
#define DART_EXAMPLES_DEMOS_SCENES_FBFAUTHORBACKSPINSPEC_HPP_

#include <dart/simulation/DeactivationOptions.hpp>
#include <dart/simulation/World.hpp>

#include <dart/constraint/BoxedLcpConstraintSolver.hpp>
#include <dart/constraint/ContactConstraint.hpp>
#include <dart/constraint/ExactCoulombFbfConstraintSolver.hpp>

#include <dart/collision/native/NativeCollisionDetector.hpp>

#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/Inertia.hpp>
#include <dart/dynamics/ShapeNode.hpp>
#include <dart/dynamics/SphereShape.hpp>

#include <Eigen/Geometry>

#include <algorithm>
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

#ifndef DART_FBF_AUTHOR_BACKSPIN_SPEC_SHA256
  #error "FBF author-backspin consumers must bind the shared spec source hash"
#endif
#ifndef DART_FBF_EXACT_SOLVER_OPTIONS_SHA256
  #error "FBF author-backspin consumers must bind the exact-solver options hash"
#endif

namespace fbf_author_backspin {

inline constexpr const char* kSceneId = "fbf_author_backspin_current_source";
inline constexpr const char* kContractSchema
    = "dart.fbf_author_backspin_dynamics_adapter/v1";
inline constexpr const char* kSceneStateSchema
    = "dart.fbf_author_backspin_scene_state/v1";
inline constexpr const char* kContractKind
    = "current_source_configuration_dynamics_adapter";
inline constexpr const char* kAuthorRepository
    = "https://github.com/matthcsong/fbf-sca-2026";
inline constexpr const char* kAuthorCommit
    = "b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0";
inline constexpr const char* kAuthorTree
    = "ffcdafb61adeda2239c8366d054b548b50d26685";
inline constexpr const char* kAuthorRunPath
    = "paper_examples/backspin-ball/run.py";
inline constexpr const char* kAuthorRunBlob
    = "82d8916233df8db1cacc5915699dc53a5d08ea17";
inline constexpr const char* kAuthorRunSha256
    = "c9174e88bf18dbe050d72568639de50b8477f2bc57ea9558637087da4268409a";
inline constexpr const char* kAuthorSourceRunId = "20260722T185956Z";
inline constexpr const char* kAuthorSweepResultsSha256
    = "830e0db3b81406ab3ac305383f811daa45680b4bd2a8a94f198250b8b4cbb2ca";
inline constexpr const char* kAuthorMetadataJsonSha256
    = "0d919084b7ef8dcc42e9c89c7b7a3c9256e50c4eac664b6b861c407b5ed9eb84";
inline constexpr const char* kAuthorResultJsonSha256
    = "29a63de1277ec96c6e4f23c4650634c28c1a783a56995a5138c2d3bd15066adc";
inline constexpr const char* kAuthorTrajectoryNpzSha256
    = "77a28f87962bef54132b98fa1f37d9e82807d33b8e5c3bb154e814bb02462133";
inline constexpr const char* kAuthorHistoryJsonSha256
    = "2d0d6fc73923d227e748002785ee11543f8aa292bf7926a03c21a9b4a7f7a3c9";
inline constexpr const char* kSpecSourceSha256
    = DART_FBF_AUTHOR_BACKSPIN_SPEC_SHA256;
inline constexpr const char* kExactSolverOptionsSha256
    = DART_FBF_EXACT_SOLVER_OPTIONS_SHA256;

inline constexpr double kGravity = 9.81;
inline constexpr double kRadius = 0.25;
inline constexpr double kMass = 1.0;
inline constexpr double kDensity
    = kMass
      / (4.0 / 3.0 * 3.14159265358979323846 * kRadius * kRadius * kRadius);
inline constexpr double kFriction = 0.5;
inline constexpr double kInitialLinearVelocity = 4.0;
inline constexpr double kInitialAngularVelocity = -200.0;
inline constexpr double kInitialGeometricSeparation = 0.001;
inline constexpr double kInitialCenterHeight
    = kRadius + kInitialGeometricSeparation;
inline constexpr double kTimeStep = 1.0 / 60.0;
inline constexpr double kDuration = 4.0;
inline constexpr std::size_t kTotalSteps = 240u;
inline constexpr double kGroundHalfExtentX = 15.0;
inline constexpr double kGroundHalfExtentY = 0.5;
inline constexpr double kGroundHalfExtentZ = 0.05;
inline constexpr double kSourceShapeGap = 0.001;
inline constexpr double kSourceShapeStiffness = 1.0e4;
inline constexpr double kSourceShapeDamping = 1.0e3;
inline constexpr std::size_t kSourceMaxContacts = 4096u;

// These settings are a declared DART adapter to the pinned source CLI. They
// do not claim Warp/Newton kernel, float32, gap, or compliance equivalence.
inline constexpr std::size_t kDartMaxContacts = 4u;
inline constexpr std::size_t kDartMaxContactsPerPair = 4u;
inline constexpr int kDartMaxOuterIterations = 200;
inline constexpr double kDartTolerance = 1.0e-6;
inline constexpr int kDartInnerMaxSweeps = 10;
inline constexpr int kDartInnerLocalIterations = 200;
inline constexpr double kDartInnerTolerance = 1.0e-6;
inline constexpr double kDartInnerLocalTolerance = 1.0e-6;
inline constexpr double kDartStepSizeScale = 10.0;
inline constexpr bool kDartFallbackToBoxedLcpEnabled = false;
inline constexpr bool kDartSourceInnerInitializationEnabled = true;
inline constexpr bool kDartColoredBlockGaussSeidelEnabled = false;
inline constexpr bool kDartParticipantAffinityEnabled = false;
inline constexpr double kDartContactErrorReductionParameter = 0.0;
inline constexpr double kDartRestitution = 0.0;
inline constexpr double kDartEdgeContactTolerance = 0.05;
inline constexpr double kDartMaxOffAxisMagnitude = 1.0e-9;
inline constexpr std::size_t kDartMaxFirstSupportedStep = 2u;
inline constexpr std::size_t kDartRequiredRollingTailSamples = 5u;
inline constexpr double kSourceWarmStartMatchRadius = 0.02;
inline constexpr double kSourceWarmStartNormalCosine = 0.9;
inline constexpr int kSourceWarmStartMaxAge = 3;
inline constexpr double kSourceWarmStartGammaCap = 1.0e4;
inline constexpr double kSourceMinimumStepSize = 1.0e-6;
inline constexpr double kSourceMaximumStepSize = 1.0e6;
inline constexpr double kSourceWarmStartResidualThreshold = 1.0e-4;
inline constexpr double kSourceArmijoShrink = 0.7;
inline constexpr const char* kDartSolverPolicy
    = "source_cli_parameterized_strict_dart_adapter";

inline constexpr double kSourceReferenceFinalLinearVelocity
    = -11.428571701049805;
inline constexpr double kSourceReferenceFinalAngularVelocity
    = -45.71428680419922;
inline constexpr double kSourceReferenceFinalSlipVelocity = 0.0;
inline constexpr double kSourceReferenceFinalHeight = -1.3714094161987305;

enum class SolverLane
{
  ExactFbf,
  BoxedLcp,
};

/// Process-global ERP override scoped to this active source adapter.
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
inline Eigen::Vector3d groundSize()
{
  return Eigen::Vector3d(
      2.0 * kGroundHalfExtentX,
      2.0 * kGroundHalfExtentY,
      2.0 * kGroundHalfExtentZ);
}

//==============================================================================
inline Eigen::Matrix3d sphereMoment()
{
  return dart::dynamics::SphereShape::computeInertia(kRadius, kMass);
}

//==============================================================================
inline dart::constraint::ExactCoulombFbfConstraintSolverOptions
makeExactSolverOptions()
{
  dart::constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.maxOuterIterations = kDartMaxOuterIterations;
  options.acceptOuterMaxIterations = false;
  options.tolerance = kDartTolerance;
  options.innerMaxSweeps = kDartInnerMaxSweeps;
  options.innerLocalSolver = dart::constraint::ExactCoulombFbfLocalBlockSolver::
      ExactMetricProjection;
  options.runFixedInnerSweeps = false;
  options.acceptInnerMaxIterations = true;
  options.innerLocalIterations = kDartInnerLocalIterations;
  options.innerTolerance = kDartInnerTolerance;
  options.innerLocalTolerance = kDartInnerLocalTolerance;
  options.stepSizeScale = kDartStepSizeScale;
  options.fallbackToBoxedLcp = kDartFallbackToBoxedLcpEnabled;
  options.includeConstraintRegularization = false;
  options.useMatrixFreeDelassusOperator = false;
  options.useContactRowDelassusOperator = true;
  options.assembleDenseContactRowSnapshot = true;
  options.enableWarmStart = true;
  options.enableStepSizePersistence = true;
  options.stepSizeRecoveryGrowthFactor = 1.0 / kSourceArmijoShrink;
  options.warmStartMatchDistance = kSourceWarmStartMatchRadius;
  options.seedNormalImpulseFromDiagonal = true;
  options.useMatrixFreeDelassusSeed = false;
  options.enableProjectedGradientRetry = false;
  options.enableDenseResidualPolish = false;
  options.initialStepSize = std::numeric_limits<double>::quiet_NaN();
  options.capInitialStepSizeAtSafeBound = true;
  options.outerRelaxation = 1.0;
  options.couplingVariationTolerance = 0.9;
  options.shrinkFactor = 0.7;
  options.maxStepShrinkIterations = 8;
  options.enableAdaptiveStepSize = true;
  options.spectralIterations = 10;
  options.innerDiagonalRegularization = 0.0;
  options.projectedGradientMaxIterations = kDartInnerLocalIterations;
  options.projectedGradientTolerance = kDartInnerTolerance;
  options.denseResidualPolishIterations = 8;
  options.denseResidualPolishLineSearchIterations = 8;
  options.denseResidualPolishRegularization = 1.0e-9;
  options.maxResidualHistorySamples = 0;
  options.maxResidualHistoryRecords = 0;
  return options;
}

//==============================================================================
inline dart::constraint::ExactCoulombFbfCrossStepPolicyOptions
makeCrossStepPolicyOptions()
{
  dart::constraint::ExactCoulombFbfCrossStepPolicyOptions options;
  options.warmStartMatchMode = dart::constraint::
      ExactCoulombFbfWarmStartMatchMode::OrderedBodyBLocalFeature;
  options.warmStartNormalCosine = kSourceWarmStartNormalCosine;
  options.useStrictWarmStartMatchDistance = true;
  options.warmStartMaxAge = kSourceWarmStartMaxAge;
  options.persistentStepSizeSafeBoundScale = kDartStepSizeScale;
  options.minimumStepSize = kSourceMinimumStepSize;
  options.maximumStepSize = kSourceMaximumStepSize;
  options.warmStartResidualThreshold = kSourceWarmStartResidualThreshold;
  options.warmStartStepSizeCap = kSourceWarmStartGammaCap;
  options.persistUncappedStepSizeOnWarmStartCap = true;
  options.requireResidualImprovementForUnconvergedCacheSave = true;
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
    throw std::invalid_argument(
        "cannot install author backspin solver on null");

  const auto detector = createCollisionDetector();
  if (lane == SolverLane::ExactFbf) {
    auto solver
        = std::make_unique<dart::constraint::ExactCoulombFbfConstraintSolver>(
            makeExactSolverOptions());
    solver->setCollisionDetector(detector);
    solver->setNumSimulationThreads(1u);
    world->setConstraintSolver(std::move(solver));
    // World::setConstraintSolver() imports default-off sidecar policies from
    // the replaced solver. Apply this source CLI choice after installation.
    auto* installedExact
        = dynamic_cast<dart::constraint::ExactCoulombFbfConstraintSolver*>(
            world->getConstraintSolver());
    installedExact->setExactCoulombCrossStepPolicyOptions(
        makeCrossStepPolicyOptions());
    installedExact->setExactCoulombPostCorrectionProjectionEnabled(false);
    installedExact->setExactCoulombSourceInnerInitializationEnabled(
        kDartSourceInnerInitializationEnabled);
    installedExact->setExactCoulombColoredBlockGaussSeidelEnabled(
        kDartColoredBlockGaussSeidelEnabled);
    installedExact
        ->setExactCoulombColoredBlockGaussSeidelParticipantAffinityEnabled(
            kDartParticipantAffinityEnabled);
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
  collisionOption.maxNumContacts = kDartMaxContacts;
  collisionOption.maxNumContactsPerPair = kDartMaxContactsPerPair;
}

//==============================================================================
inline dart::dynamics::SkeletonPtr createGround()
{
  auto skeleton = dart::dynamics::Skeleton::create("backspin_author_ground");
  auto pair = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
  auto* joint = pair.first;
  auto* body = pair.second;
  body->setName("backspin_author_ground_body");
  auto* node = body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::BoxShape>(groundSize()));
  node->getVisualAspect()->setRGBA(Eigen::Vector4d(0.42, 0.44, 0.47, 1.0));
  auto* dynamics = node->getDynamicsAspect();
  dynamics->setPrimaryFrictionCoeff(kFriction);
  dynamics->setSecondaryFrictionCoeff(kFriction);
  dynamics->setRestitutionCoeff(kDartRestitution);
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation().z() = -kGroundHalfExtentZ;
  joint->setPositions(dart::dynamics::FreeJoint::convertToPositions(pose));
  skeleton->setMobile(false);
  return skeleton;
}

//==============================================================================
inline dart::dynamics::SkeletonPtr createSphere()
{
  auto skeleton = dart::dynamics::Skeleton::create("backspin_author_ball");
  dart::dynamics::FreeJoint::Properties jointProperties;
  jointProperties.mName = "backspin_author_ball_joint";
  dart::dynamics::BodyNode::Properties bodyProperties;
  bodyProperties.mName = "backspin_author_ball_body";
  bodyProperties.mInertia.setMass(kMass);
  bodyProperties.mInertia.setMoment(sphereMoment());
  auto pair = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
      nullptr, jointProperties, bodyProperties);
  auto* joint = pair.first;
  auto* body = pair.second;
  auto* node = body->createShapeNodeWith<
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::SphereShape>(kRadius));
  auto* dynamics = node->getDynamicsAspect();
  dynamics->setPrimaryFrictionCoeff(kFriction);
  dynamics->setSecondaryFrictionCoeff(kFriction);
  dynamics->setRestitutionCoeff(kDartRestitution);

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation().z() = kInitialCenterHeight;
  joint->setPositions(dart::dynamics::FreeJoint::convertToPositions(pose));
  joint->setLinearVelocity(Eigen::Vector3d(kInitialLinearVelocity, 0.0, 0.0));
  joint->setAngularVelocity(Eigen::Vector3d(0.0, kInitialAngularVelocity, 0.0));
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
  world->addSkeleton(createGround());
  world->addSkeleton(createSphere());
  return world;
}

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
  dart::constraint::ExactCoulombFbfConstraintSolverOptions exactOptions;
  dart::constraint::ExactCoulombFbfCrossStepPolicyOptions crossStepOptions;
  int maxOuterIterations = 0;
  double tolerance = 0.0;
  int innerMaxSweeps = 0;
  int innerLocalIterations = 0;
  double stepSizeScale = 0.0;
  bool sourceContinuationEnabled = false;
  bool postCorrectionProjectionEnabled = false;
  bool sourceInnerInitializationEnabled = false;
  bool coloredBlockGaussSeidelEnabled = false;
  bool participantAffinityEnabled = false;
  bool fallbackToBoxedLcpEnabled = false;
  double observedContactErrorReductionParameter = 0.0;
  bool groundMobile = true;
  Eigen::Vector3d observedGroundSize = Eigen::Vector3d::Zero();
  Eigen::Isometry3d groundPose = Eigen::Isometry3d::Identity();
  double groundPrimaryFriction = 0.0;
  double groundSecondaryFriction = 0.0;
  double groundRestitution = 0.0;
  bool sphereMobile = false;
  double observedSphereRadius = 0.0;
  Eigen::Isometry3d spherePose = Eigen::Isometry3d::Identity();
  Eigen::Vector3d sphereLinearVelocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d sphereAngularVelocity = Eigen::Vector3d::Zero();
  double spherePrimaryFriction = 0.0;
  double sphereSecondaryFriction = 0.0;
  double sphereRestitution = 0.0;
  double observedSphereMass = 0.0;
  Eigen::Matrix3d observedSphereMoment = Eigen::Matrix3d::Zero();
};

//==============================================================================
inline void requireNear(double actual, double expected, const std::string& what)
{
  const double scale = std::max({1.0, std::abs(actual), std::abs(expected)});
  if (!std::isfinite(actual) || std::abs(actual - expected) > 1e-11 * scale) {
    std::ostringstream message;
    message << std::setprecision(std::numeric_limits<double>::max_digits10)
            << "author backspin mismatch: " << what << " actual=" << actual
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
    throw std::runtime_error("author backspin adapter has no solver");
  if (world->getName() != kSceneId)
    throw std::runtime_error("author backspin adapter has the wrong world");
  if (implementationSourceSha256.size() != 64u)
    throw std::runtime_error("author backspin implementation hash is invalid");

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
    contract.exactOptions = options;
    contract.crossStepOptions = exact->getExactCoulombCrossStepPolicyOptions();
    contract.maxOuterIterations = options.maxOuterIterations;
    contract.tolerance = options.tolerance;
    contract.innerMaxSweeps = options.innerMaxSweeps;
    contract.innerLocalIterations = options.innerLocalIterations;
    contract.stepSizeScale = options.stepSizeScale;
    contract.sourceContinuationEnabled
        = exact->getExactCoulombSourceContinuationOptions().enabled;
    contract.postCorrectionProjectionEnabled
        = exact->getExactCoulombPostCorrectionProjectionEnabled();
    contract.sourceInnerInitializationEnabled
        = exact->getExactCoulombSourceInnerInitializationEnabled();
    contract.coloredBlockGaussSeidelEnabled
        = exact->getExactCoulombColoredBlockGaussSeidelEnabled();
    contract.participantAffinityEnabled
        = exact
              ->getExactCoulombColoredBlockGaussSeidelParticipantAffinityEnabled();
    contract.fallbackToBoxedLcpEnabled = options.fallbackToBoxedLcp;
    const auto expectedOptions = makeExactSolverOptions();
    const auto expectedCrossStep = makeCrossStepPolicyOptions();
    if (options.fallbackToBoxedLcp != expectedOptions.fallbackToBoxedLcp
        || options.includeConstraintRegularization
               != expectedOptions.includeConstraintRegularization
        || options.useMatrixFreeDelassusOperator
               != expectedOptions.useMatrixFreeDelassusOperator
        || options.useContactRowDelassusOperator
               != expectedOptions.useContactRowDelassusOperator
        || options.assembleDenseContactRowSnapshot
               != expectedOptions.assembleDenseContactRowSnapshot
        || options.enableWarmStart != expectedOptions.enableWarmStart
        || options.enableStepSizePersistence
               != expectedOptions.enableStepSizePersistence
        || options.seedNormalImpulseFromDiagonal
               != expectedOptions.seedNormalImpulseFromDiagonal
        || options.useMatrixFreeDelassusSeed
               != expectedOptions.useMatrixFreeDelassusSeed
        || options.enableProjectedGradientRetry
               != expectedOptions.enableProjectedGradientRetry
        || options.enableDenseResidualPolish
               != expectedOptions.enableDenseResidualPolish
        || options.maxOuterIterations != expectedOptions.maxOuterIterations
        || options.acceptOuterMaxIterations
               != expectedOptions.acceptOuterMaxIterations
        || !std::isnan(options.initialStepSize)
        || options.capInitialStepSizeAtSafeBound
               != expectedOptions.capInitialStepSizeAtSafeBound
        || options.maxStepShrinkIterations
               != expectedOptions.maxStepShrinkIterations
        || options.enableAdaptiveStepSize
               != expectedOptions.enableAdaptiveStepSize
        || options.spectralIterations != expectedOptions.spectralIterations
        || options.innerMaxSweeps != expectedOptions.innerMaxSweeps
        || options.innerLocalSolver != expectedOptions.innerLocalSolver
        || options.runFixedInnerSweeps != expectedOptions.runFixedInnerSweeps
        || options.acceptInnerMaxIterations
               != expectedOptions.acceptInnerMaxIterations
        || options.innerLocalIterations != expectedOptions.innerLocalIterations
        || options.projectedGradientMaxIterations
               != expectedOptions.projectedGradientMaxIterations
        || options.denseResidualPolishIterations
               != expectedOptions.denseResidualPolishIterations
        || options.denseResidualPolishLineSearchIterations
               != expectedOptions.denseResidualPolishLineSearchIterations
        || options.maxResidualHistorySamples
               != expectedOptions.maxResidualHistorySamples
        || options.maxResidualHistoryRecords
               != expectedOptions.maxResidualHistoryRecords
        || contract.sourceContinuationEnabled
        || contract.postCorrectionProjectionEnabled
        || contract.sourceInnerInitializationEnabled
               != kDartSourceInnerInitializationEnabled
        || contract.coloredBlockGaussSeidelEnabled
               != kDartColoredBlockGaussSeidelEnabled
        || contract.participantAffinityEnabled
               != kDartParticipantAffinityEnabled
        || contract.fallbackToBoxedLcpEnabled != kDartFallbackToBoxedLcpEnabled
        || contract.crossStepOptions.warmStartMatchMode
               != expectedCrossStep.warmStartMatchMode
        || contract.crossStepOptions.useStrictWarmStartMatchDistance
               != expectedCrossStep.useStrictWarmStartMatchDistance
        || contract.crossStepOptions.warmStartMaxAge
               != expectedCrossStep.warmStartMaxAge
        || contract.crossStepOptions.persistUncappedStepSizeOnWarmStartCap
               != expectedCrossStep.persistUncappedStepSizeOnWarmStartCap
        || contract.crossStepOptions
                   .requireResidualImprovementForUnconvergedCacheSave
               != expectedCrossStep
                      .requireResidualImprovementForUnconvergedCacheSave) {
      std::ostringstream message;
      message << "author backspin mismatch: exact solver policy";
      throw std::runtime_error(message.str());
    }
    requireNear(contract.tolerance, kDartTolerance, "exact tolerance");
    requireNear(
        contract.stepSizeScale, kDartStepSizeScale, "exact step-size scale");
    requireNear(
        options.stepSizeRecoveryGrowthFactor,
        expectedOptions.stepSizeRecoveryGrowthFactor,
        "step-size recovery growth");
    requireNear(
        options.warmStartMatchDistance,
        expectedOptions.warmStartMatchDistance,
        "warm-start match distance");
    requireNear(
        options.outerRelaxation,
        expectedOptions.outerRelaxation,
        "outer relaxation");
    requireNear(
        options.couplingVariationTolerance,
        expectedOptions.couplingVariationTolerance,
        "coupling variation tolerance");
    requireNear(
        options.shrinkFactor, expectedOptions.shrinkFactor, "shrink factor");
    requireNear(
        options.innerTolerance,
        expectedOptions.innerTolerance,
        "inner tolerance");
    requireNear(
        options.innerLocalTolerance,
        expectedOptions.innerLocalTolerance,
        "inner local tolerance");
    requireNear(
        options.innerDiagonalRegularization,
        expectedOptions.innerDiagonalRegularization,
        "inner diagonal regularization");
    requireNear(
        options.projectedGradientTolerance,
        expectedOptions.projectedGradientTolerance,
        "projected-gradient tolerance");
    requireNear(
        options.denseResidualPolishRegularization,
        expectedOptions.denseResidualPolishRegularization,
        "dense-polish regularization");
    requireNear(
        contract.crossStepOptions.warmStartNormalCosine,
        expectedCrossStep.warmStartNormalCosine,
        "cross-step normal cosine");
    requireNear(
        contract.crossStepOptions.persistentStepSizeSafeBoundScale,
        expectedCrossStep.persistentStepSizeSafeBoundScale,
        "cross-step safe-bound scale");
    requireNear(
        contract.crossStepOptions.minimumStepSize,
        expectedCrossStep.minimumStepSize,
        "cross-step minimum step size");
    requireNear(
        contract.crossStepOptions.maximumStepSize,
        expectedCrossStep.maximumStepSize,
        "cross-step maximum step size");
    requireNear(
        contract.crossStepOptions.warmStartResidualThreshold,
        expectedCrossStep.warmStartResidualThreshold,
        "cross-step residual threshold");
    requireNear(
        contract.crossStepOptions.warmStartStepSizeCap,
        expectedCrossStep.warmStartStepSizeCap,
        "cross-step step-size cap");
  } else if (boxed) {
    contract.solverLane = "boxed_lcp";
  } else {
    throw std::runtime_error("author backspin adapter solver is unsupported");
  }

  const auto detector
      = std::dynamic_pointer_cast<dart::collision::NativeCollisionDetector>(
          world->getConstraintSolver()->getCollisionDetector());
  if (!detector)
    throw std::runtime_error(
        "author backspin adapter requires Native collision");
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
  contract.observedContactErrorReductionParameter
      = dart::constraint::ContactConstraint::getErrorReductionParameter();

  const auto ground = world->getSkeleton("backspin_author_ground");
  const auto sphere = world->getSkeleton("backspin_author_ball");
  if (!ground || !sphere || ground->getNumBodyNodes() != 1u
      || sphere->getNumBodyNodes() != 1u) {
    throw std::runtime_error("author backspin adapter bodies are missing");
  }
  const auto* groundBody = ground->getBodyNode(0u);
  const auto* sphereBody = sphere->getBodyNode(0u);
  const auto* groundNode
      = groundBody->getShapeNodeWith<dart::dynamics::CollisionAspect>(0u);
  const auto* sphereNode
      = sphereBody->getShapeNodeWith<dart::dynamics::CollisionAspect>(0u);
  if (!groundNode || !sphereNode || !groundNode->getDynamicsAspect()
      || !sphereNode->getDynamicsAspect()) {
    throw std::runtime_error("author backspin collision shapes are missing");
  }
  const auto groundBox
      = std::dynamic_pointer_cast<const dart::dynamics::BoxShape>(
          groundNode->getShape());
  const auto dynamicSphere
      = std::dynamic_pointer_cast<const dart::dynamics::SphereShape>(
          sphereNode->getShape());
  if (!groundBox || !dynamicSphere)
    throw std::runtime_error("author backspin adapter shapes are invalid");

  contract.groundMobile = ground->isMobile();
  contract.observedGroundSize = groundBox->getSize();
  contract.groundPose = groundBody->getWorldTransform();
  contract.groundPrimaryFriction
      = groundNode->getDynamicsAspect()->getPrimaryFrictionCoeff();
  contract.groundSecondaryFriction
      = groundNode->getDynamicsAspect()->getSecondaryFrictionCoeff();
  contract.groundRestitution
      = groundNode->getDynamicsAspect()->getRestitutionCoeff();
  contract.sphereMobile = sphere->isMobile();
  contract.observedSphereRadius = dynamicSphere->getRadius();
  contract.spherePose = sphereBody->getWorldTransform();
  contract.sphereLinearVelocity = sphereBody->getLinearVelocity();
  contract.sphereAngularVelocity = sphereBody->getAngularVelocity();
  contract.spherePrimaryFriction
      = sphereNode->getDynamicsAspect()->getPrimaryFrictionCoeff();
  contract.sphereSecondaryFriction
      = sphereNode->getDynamicsAspect()->getSecondaryFrictionCoeff();
  contract.sphereRestitution
      = sphereNode->getDynamicsAspect()->getRestitutionCoeff();
  contract.observedSphereMass = sphereBody->getInertia().getMass();
  contract.observedSphereMoment = sphereBody->getInertia().getMoment();

  requireNear(contract.timeStep, kTimeStep, "time step");
  if (!contract.gravity.isApprox(Eigen::Vector3d(0.0, 0.0, -kGravity), 1e-12))
    throw std::runtime_error("author backspin mismatch: gravity");
  if (contract.simulationThreads != 1u || contract.deactivationEnabled)
    throw std::runtime_error("author backspin mismatch: world policy");
  requireNear(
      contract.observedContactErrorReductionParameter,
      kDartContactErrorReductionParameter,
      "contact ERP");
  if (contract.splitImpulseEnabled)
    throw std::runtime_error("author backspin mismatch: split impulse");
  if (contract.contactManifold != "four_point_planar")
    throw std::runtime_error("author backspin mismatch: contact manifold");
  if (contract.maxContacts != kDartMaxContacts
      || contract.maxContactsPerPair != kDartMaxContactsPerPair) {
    throw std::runtime_error("author backspin mismatch: contact caps");
  }
  if (contract.groundMobile
      || !contract.observedGroundSize.isApprox(groundSize(), 1e-12)
      || !contract.groundPose.linear().isApprox(
          Eigen::Matrix3d::Identity(), 1e-12)
      || !contract.groundPose.translation().isApprox(
          Eigen::Vector3d(0.0, 0.0, -kGroundHalfExtentZ), 1e-12)) {
    throw std::runtime_error("author backspin mismatch: ground geometry");
  }
  requireNear(
      contract.groundPrimaryFriction, kFriction, "ground primary friction");
  requireNear(
      contract.groundSecondaryFriction, kFriction, "ground secondary friction");
  requireNear(
      contract.groundRestitution, kDartRestitution, "ground restitution");
  if (!contract.sphereMobile
      || !contract.spherePose.linear().isApprox(
          Eigen::Matrix3d::Identity(), 1e-12)
      || !contract.spherePose.translation().isApprox(
          Eigen::Vector3d(0.0, 0.0, kInitialCenterHeight), 1e-12)
      || !contract.sphereLinearVelocity.isApprox(
          Eigen::Vector3d(kInitialLinearVelocity, 0.0, 0.0), 1e-12)
      || !contract.sphereAngularVelocity.isApprox(
          Eigen::Vector3d(0.0, kInitialAngularVelocity, 0.0), 1e-12)) {
    throw std::runtime_error("author backspin mismatch: sphere initial state");
  }
  requireNear(contract.observedSphereRadius, kRadius, "sphere radius");
  requireNear(
      contract.spherePrimaryFriction, kFriction, "sphere primary friction");
  requireNear(
      contract.sphereSecondaryFriction, kFriction, "sphere secondary friction");
  requireNear(
      contract.sphereRestitution, kDartRestitution, "sphere restitution");
  requireNear(contract.observedSphereMass, kMass, "sphere mass");
  if (!contract.observedSphereMoment.isApprox(sphereMoment(), 1e-12))
    throw std::runtime_error("author backspin mismatch: sphere moment");
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
  out << ",\"source_run_id\":";
  writeJsonString(out, kAuthorSourceRunId);
  out << ",\"sweep_results_sha256\":";
  writeJsonString(out, kAuthorSweepResultsSha256);
  out << ",\"metadata_json_sha256\":";
  writeJsonString(out, kAuthorMetadataJsonSha256);
  out << ",\"result_json_sha256\":";
  writeJsonString(out, kAuthorResultJsonSha256);
  out << ",\"trajectory_npz_sha256\":";
  writeJsonString(out, kAuthorTrajectoryNpzSha256);
  out << ",\"history_json_sha256\":";
  writeJsonString(out, kAuthorHistoryJsonSha256);
  out << ",\"configuration_spec_sha256\":";
  writeJsonString(out, kSpecSourceSha256);
  out << ",\"exact_solver_options_sha256\":";
  writeJsonString(out, kExactSolverOptionsSha256);
  out << ",\"demo_implementation_sha256\":";
  writeJsonString(out, contract.implementationSourceSha256);
  out << "},\"source_configuration\":{\"gravity_m_s2\":" << kGravity
      << ",\"radius_m\":" << kRadius << ",\"mass_kg\":" << kMass
      << ",\"density_kg_m3\":" << kDensity << ",\"friction\":" << kFriction
      << ",\"initial_center_m\":[0,0," << kInitialCenterHeight
      << "],\"initial_linear_velocity_m_s\":[" << kInitialLinearVelocity
      << ",0,0],\"initial_angular_velocity_rad_s\":[0,"
      << kInitialAngularVelocity << ",0],\"time_step_seconds\":" << kTimeStep
      << ",\"duration_seconds\":" << kDuration
      << ",\"total_steps\":" << kTotalSteps << ",\"ground_half_extents_m\":["
      << kGroundHalfExtentX << ',' << kGroundHalfExtentY << ','
      << kGroundHalfExtentZ << "],\"shape_gap_m\":" << kSourceShapeGap
      << ",\"shape_stiffness\":" << kSourceShapeStiffness
      << ",\"shape_damping\":" << kSourceShapeDamping
      << ",\"max_contacts\":" << kSourceMaxContacts
      << "},\"source_reference_projection\":{\"terminal_vx_m_s\":"
      << kSourceReferenceFinalLinearVelocity
      << ",\"terminal_wy_rad_s\":" << kSourceReferenceFinalAngularVelocity
      << ",\"terminal_slip_m_s\":" << kSourceReferenceFinalSlipVelocity
      << ",\"terminal_z_m\":" << kSourceReferenceFinalHeight
      << ",\"terminal_supported\":false},\"dart_adapter\":{\"scene_id\":";
  writeJsonString(out, kSceneId);
  out << ",\"scene_state_schema\":";
  writeJsonString(out, kSceneStateSchema);
  out << ",\"world\":{\"time_step_seconds\":" << contract.timeStep
      << ",\"gravity_m_s2\":";
  writeJsonVector(out, contract.gravity);
  out << ",\"simulation_threads\":" << contract.simulationThreads
      << ",\"deactivation_enabled\":"
      << (contract.deactivationEnabled ? "true" : "false")
      << "},\"collision\":{\"detector\":";
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
    const auto& options = contract.exactOptions;
    out << "{\"constraint_regularization_enabled\":"
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
        << ",\"tolerance\":" << options.tolerance
        << ",\"initial_step_size\":null"
        << ",\"cap_initial_step_size_at_safe_bound\":"
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
        << ",\"inner_local_solver\":\"exact_metric_projection\""
        << ",\"run_fixed_inner_sweeps\":"
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
        << options.maxResidualHistoryRecords
        << ",\"source_inner_initialization_enabled\":"
        << (contract.sourceInnerInitializationEnabled ? "true" : "false")
        << ",\"colored_block_gauss_seidel_enabled\":"
        << (contract.coloredBlockGaussSeidelEnabled ? "true" : "false")
        << ",\"participant_affinity_enabled\":"
        << (contract.participantAffinityEnabled ? "true" : "false")
        << ",\"source_continuation_enabled\":"
        << (contract.sourceContinuationEnabled ? "true" : "false")
        << ",\"post_correction_projection_enabled\":"
        << (contract.postCorrectionProjectionEnabled ? "true" : "false")
        << ",\"fallback_to_boxed_lcp_enabled\":"
        << (contract.fallbackToBoxedLcpEnabled ? "true" : "false") << '}';
  }
  out << ",\"cross_step_options\":";
  if (!contract.exactOptionsAvailable) {
    out << "null";
  } else {
    const auto& options = contract.crossStepOptions;
    out << "{\"warm_start_match_mode\":\"ordered_body_b_local_feature\""
        << ",\"warm_start_normal_cosine\":" << options.warmStartNormalCosine
        << ",\"strict_warm_start_match_distance\":"
        << (options.useStrictWarmStartMatchDistance ? "true" : "false")
        << ",\"warm_start_max_age\":" << options.warmStartMaxAge
        << ",\"persistent_step_size_safe_bound_scale\":"
        << options.persistentStepSizeSafeBoundScale
        << ",\"minimum_step_size\":" << options.minimumStepSize
        << ",\"maximum_step_size\":" << options.maximumStepSize
        << ",\"warm_start_residual_threshold\":"
        << options.warmStartResidualThreshold
        << ",\"warm_start_step_size_cap\":" << options.warmStartStepSizeCap
        << ",\"persist_uncapped_step_size_on_warm_start_cap\":"
        << (options.persistUncappedStepSizeOnWarmStartCap ? "true" : "false")
        << ",\"require_residual_improvement_for_unconverged_cache_save\":"
        << (options.requireResidualImprovementForUnconvergedCacheSave ? "true"
                                                                      : "false")
        << '}';
  }
  out << "},\"process_state\":{\"observed_contact_erp\":"
      << contract.observedContactErrorReductionParameter
      << "},\"ground\":{\"mobile\":"
      << (contract.groundMobile ? "true" : "false") << ",\"size_m\":";
  writeJsonVector(out, contract.observedGroundSize);
  out << ",\"initial_pose\":";
  writeJsonPose(out, contract.groundPose);
  out << ",\"primary_friction\":" << contract.groundPrimaryFriction
      << ",\"secondary_friction\":" << contract.groundSecondaryFriction
      << ",\"restitution\":" << contract.groundRestitution
      << "},\"sphere\":{\"mobile\":"
      << (contract.sphereMobile ? "true" : "false")
      << ",\"radius_m\":" << contract.observedSphereRadius
      << ",\"initial_pose\":";
  writeJsonPose(out, contract.spherePose);
  out << ",\"initial_linear_velocity_m_s\":";
  writeJsonVector(out, contract.sphereLinearVelocity);
  out << ",\"initial_angular_velocity_rad_s\":";
  writeJsonVector(out, contract.sphereAngularVelocity);
  out << ",\"primary_friction\":" << contract.spherePrimaryFriction
      << ",\"secondary_friction\":" << contract.sphereSecondaryFriction
      << ",\"restitution\":" << contract.sphereRestitution
      << ",\"mass_kg\":" << contract.observedSphereMass << ",\"moment_kg_m2\":";
  writeJsonMatrix(out, contract.observedSphereMoment);
  out << "}},\"adapter_boundaries\":{"
         "\"source_initial_geometric_separation_represented\":true,"
         "\"source_shape_gap_semantics_implemented\":false,"
         "\"source_shape_stiffness_semantics_implemented\":false,"
         "\"source_shape_damping_semantics_implemented\":false,"
         "\"source_collision_backend_implemented\":false,"
         "\"source_solver_backend_semantics_implemented\":false,"
         "\"source_float32_semantics_implemented\":false,"
         "\"checker_texture_visual_only\":true,"
         "\"support_observation\":\"two_body_collision_result_contact_count_"
         "proxy\"},\"claim_boundary\":{"
         "\"current_source_configuration_port\":true,"
         "\"terminal_outcome_slice_candidate\":true,"
         "\"historical_paper_invocation_known\":false,"
         "\"full_source_trajectory_equivalence\":false,"
         "\"solver_equivalence\":false,"
         "\"renderer_equivalence\":false,"
         "\"timing_comparability\":false,"
         "\"fig03_parity\":false,\"paper_parity\":false}}";
  return out.str();
}

//==============================================================================
inline std::vector<std::pair<std::string, double>> sceneStateFields(
    const std::shared_ptr<dart::simulation::World>& world)
{
  if (!world || world->getName() != kSceneId)
    throw std::runtime_error("author backspin scene state has the wrong world");
  const auto sphere = world->getSkeleton("backspin_author_ball");
  if (!sphere || sphere->getNumBodyNodes() != 1u)
    throw std::runtime_error("author backspin scene state has no sphere");
  const auto* body = sphere->getBodyNode(0u);
  const Eigen::Isometry3d transform = body->getWorldTransform();
  const Eigen::Vector3d linearVelocity = body->getLinearVelocity();
  const Eigen::Vector3d angularVelocity = body->getAngularVelocity();
  Eigen::Quaterniond orientation(transform.linear());
  orientation.normalize();
  const double slipVelocity
      = linearVelocity.x() - kRadius * angularVelocity.y();
  const double worldTime = world->getTime();
  const std::size_t contactCount
      = world->getConstraintSolver()->getLastCollisionResult().getNumContacts();
  if (!transform.matrix().allFinite() || !linearVelocity.allFinite()
      || !angularVelocity.allFinite() || !orientation.coeffs().allFinite()
      || !std::isfinite(slipVelocity) || !std::isfinite(worldTime)) {
    throw std::runtime_error("author backspin scene state is non-finite");
  }
  return {
      {"world_time_seconds", worldTime},
      {"position_x_m", transform.translation().x()},
      {"position_y_m", transform.translation().y()},
      {"position_z_m", transform.translation().z()},
      {"linear_velocity_x_m_s", linearVelocity.x()},
      {"linear_velocity_y_m_s", linearVelocity.y()},
      {"linear_velocity_z_m_s", linearVelocity.z()},
      {"angular_velocity_x_rad_s", angularVelocity.x()},
      {"angular_velocity_y_rad_s", angularVelocity.y()},
      {"angular_velocity_z_rad_s", angularVelocity.z()},
      {"orientation_w", orientation.w()},
      {"orientation_x", orientation.x()},
      {"orientation_y", orientation.y()},
      {"orientation_z", orientation.z()},
      {"slip_velocity_m_s", slipVelocity},
      {"contact_count", static_cast<double>(contactCount)},
      {"supported", contactCount > 0u ? 1.0 : 0.0},
  };
}

} // namespace fbf_author_backspin

#endif // DART_EXAMPLES_DEMOS_SCENES_FBFAUTHORBACKSPINSPEC_HPP_
