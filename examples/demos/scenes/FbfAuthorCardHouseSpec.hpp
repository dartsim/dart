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

#ifndef DART_EXAMPLES_DEMOS_SCENES_FBFAUTHORCARDHOUSESPEC_HPP_
#define DART_EXAMPLES_DEMOS_SCENES_FBFAUTHORCARDHOUSESPEC_HPP_

#include <dart/simulation/World.hpp>

#include <dart/constraint/BoxedLcpConstraintSolver.hpp>
#include <dart/constraint/ContactConstraint.hpp>
#include <dart/constraint/DantzigBoxedLcpSolver.hpp>
#include <dart/constraint/ExactCoulombFbfConstraintSolver.hpp>
#include <dart/constraint/PgsBoxedLcpSolver.hpp>

#include <dart/collision/CollisionFilter.hpp>
#include <dart/collision/native/NativeCollisionDetector.hpp>

#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/PlaneShape.hpp>
#include <dart/dynamics/ShapeNode.hpp>

#include <Eigen/Geometry>

#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <typeinfo>
#include <vector>

#include <cmath>
#include <cstddef>

#ifndef DART_FBF_AUTHOR_CARD_HOUSE_SPEC_SHA256
  #error "FBF author-card-house consumers must bind the shared spec source hash"
#endif

namespace fbf_author_card_house {

inline constexpr const char* kContractSchema
    = "dart.fbf_author_card_house_configuration_contract/v1";
inline constexpr const char* kContractKind = "configuration_only";
inline constexpr const char* kDynamicsContractSchema
    = "dart.fbf_author_card_house_dynamics_adapter/v1";
inline constexpr const char* kDynamicsContractKind
    = "source_configuration_dynamics_adapter";
inline constexpr const char* kDemoSceneId
    = "fbf_author_card_house_5_construction";
inline constexpr const char* kDynamicsDemoSceneId
    = "fbf_author_card_house_4_impact_current_source";
inline constexpr const char* kSourceContinuationDynamicsDemoSceneId
    = "fbf_author_card_house_4_impact_source_continuation_current_source";
inline constexpr const char* kTenLevelDynamicsDemoSceneId
    = "fbf_author_card_house_10_impact_current_source";
inline constexpr const char* kAuthorRepository
    = "https://github.com/matthcsong/fbf-sca-2026";
inline constexpr const char* kAuthorCommit
    = "b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0";
inline constexpr const char* kAuthorTree
    = "ffcdafb61adeda2239c8366d054b548b50d26685e";
inline constexpr const char* kAuthorCardHouseBlob
    = "35f33651bc9674a259071ac723e47755504152db";
inline constexpr const char* kAuthorCardHouseRunSha256
    = "18c58c85eaad865aeef480b46e880a52088f266b79c90226f624637221ee36f8";
inline constexpr const char* kAuthorConfigSha256
    = "88f3f9ffd758eccce8496f7897192587a05907109e313c7a86bcf8f9de8cc248";
inline constexpr const char* kAuthorSolverSha256
    = "8ec32aa20bf8d6c1173ed6c7f3735e2926fbb4b5059ee2236e26ad27eb22f941";
inline constexpr const char* kSpecSourceSha256
    = DART_FBF_AUTHOR_CARD_HOUSE_SPEC_SHA256;

inline constexpr double kPi = 3.141592653589793238462643383279502884;
inline constexpr double kGravity = 9.81;
inline constexpr double kFriction = 0.8;
inline constexpr double kContactGap = 0.005;
inline constexpr double kSourceGroundContactGap = 0.1;
inline constexpr double kSourceShapeStiffness = 1.0e4;
inline constexpr double kSourceShapeDamping = 1.0e3;
inline constexpr double kSourceGroundShapeStiffness = 2.5e3;
inline constexpr double kSourceGroundShapeDamping = 1.0e2;
inline constexpr std::size_t kDefaultLevelCount = 5u;
inline constexpr std::size_t kFigureLevelCount = 4u;
inline constexpr std::size_t kTenLevelCount = 10u;
inline constexpr std::size_t kCubeCount = 4u;
inline constexpr double kLeanFromHorizontalDegrees = 65.0;
inline constexpr double kLeanFromVerticalDegrees = 25.0;
inline constexpr double kBridgeAngleDegrees = -1.0;
inline constexpr double kCardHalfLength = 1.25;
inline constexpr double kCardHalfDepth = 0.625;
inline constexpr double kCardHalfThickness = 0.02;
inline constexpr double kCardDensity = 200.0;
inline constexpr double kCardMass = 25.0;
inline constexpr double kTentHalfGap = 0.55;
inline constexpr double kTentWidth = 2.2;
// Evaluated from the author's exact formula:
//   (tan(65 deg) * 2.2 + tan(3 deg) * 2.2) / 2.
inline constexpr double kTentHeight = 2.41660616977186;
inline constexpr double kCubeHalfSize = 0.4;
inline constexpr double kCubeDensity = 500.0;
inline constexpr double kCubeMass = 256.0;
inline constexpr double kDropHeight = 1.0;
inline constexpr double kCubeInitialHeight = 13.0830308488593;
inline constexpr double kDisplayTimeStep = 1.0 / 60.0;
inline constexpr std::size_t kSubstepsPerFrame = 4u;
inline constexpr double kRuntimeTimeStep = 1.0 / 240.0;
inline constexpr std::size_t kReleaseFrame = 400u;
inline constexpr std::size_t kReleaseSubstep = 1600u;
inline constexpr std::size_t kTotalFrames = 800u;
inline constexpr std::size_t kTotalSubsteps = 3200u;
// Source-supported CLI selection matching the published four-level/26-card
// example and its 10-second video horizon. The public source defaults remain
// five levels and 800 frames; this selected invocation changes only arguments
// exposed by run.py.
inline constexpr std::size_t kFigureEvidenceFrames = 600u;
inline constexpr std::size_t kFigureEvidenceSubsteps
    = kFigureEvidenceFrames * kSubstepsPerFrame;
inline constexpr char kReleaseActionKey = 'p';
inline constexpr std::size_t kDartMaxContactsPerPair = 4u;
inline constexpr double kDartContactErrorReductionParameter = 0.0;
inline constexpr double kDartRestitution = 0.0;
inline constexpr double kDartSlipCompliance = -1.0;

/// A current-source-supported card-house dynamics invocation represented in
/// DART. This descriptor keeps scene identity, schedule, continuation policy,
/// and the Native contact-gap adapter choice bound together.
struct DynamicsScenario
{
  const char* demoSceneId;
  std::size_t levelCount;
  std::size_t selectedFrames;
  bool sourceContinuation;
  bool sourceContactGapValuesRepresented;

  constexpr std::size_t selectedSubsteps() const
  {
    return selectedFrames * kSubstepsPerFrame;
  }
};

inline constexpr DynamicsScenario kFourLevelImpactScenario{
    kDynamicsDemoSceneId,
    kFigureLevelCount,
    kFigureEvidenceFrames,
    false,
    false};
inline constexpr DynamicsScenario kFourLevelSourceContinuationScenario{
    kSourceContinuationDynamicsDemoSceneId,
    kFigureLevelCount,
    kFigureEvidenceFrames,
    true,
    false};
inline constexpr DynamicsScenario kTenLevelImpactScenario{
    kTenLevelDynamicsDemoSceneId, kTenLevelCount, kTotalFrames, false, true};

inline constexpr std::size_t kSourceMaxContacts = 4096u;
inline constexpr int kSourceMaxOuterIterations = 200;
inline constexpr double kSourceOuterTolerance = 1e-6;
inline constexpr int kSourceResidualCheckInterval = 5;
inline constexpr const char* kSourceInnerSolver = "block_gs";
inline constexpr int kSourceInnerGaussSeidelSweeps = 10;
inline constexpr int kSourceInnerMaxIterations = 200;
inline constexpr double kSourceInnerTolerance = 1e-6;
inline constexpr double kSourceGammaC = 5.0;
inline constexpr double kSourceGammaMax = 1e6;
inline constexpr double kSourceArmijoRhoHigh = 0.9;
inline constexpr double kSourceArmijoShrink = 0.7;
inline constexpr int kSourceArmijoMaxBacktracks = 8;
inline constexpr int kSourcePlateauPatience = 5;
inline constexpr double kSourcePlateauRelativeTolerance = 0.01;
inline constexpr double kSourceCouplingVariationSkipThreshold = 1e-10;
inline constexpr int kSourceContinuationResidualHistorySamples = 64;
inline constexpr int kSourceContinuationResidualHistoryRecords
    = static_cast<int>(kSourceMaxContacts);
inline constexpr const char* kSourceContinuationPolicy = "source_continuation";
inline constexpr const char* kSourcePlateauMetric = "natural";
inline constexpr const char* kSourceInitialConvergenceResidual
    = "natural_map_unscaled";
inline constexpr const char* kSourceSampledTerminationResidual
    = "coulomb_rel_dimensionless";
inline constexpr const char* kSourceStrictConvergenceComparison = "<";
inline constexpr const char* kSourceSmallChangeArmijoAction = "accept";
inline constexpr const char* kSourceShrinkCapAction = "shrink_cap";
inline constexpr double kSourceWarmStartMatchRadius = 0.02;
inline constexpr double kSourceWarmStartNormalCosine = 0.9;
inline constexpr int kSourceWarmStartMaxAge = 3;
inline constexpr double kSourceWarmStartGammaCap = 1e4;
inline constexpr double kSourcePersistentStepSizeSafeBoundScale = 10.0;
inline constexpr double kSourceMinimumStepSize = 1e-6;
inline constexpr double kSourceMaximumStepSize = 1e6;
inline constexpr double kSourceWarmStartResidualThreshold = 1e-4;
inline constexpr double kSourceBaumgarteErp = 0.0;
inline constexpr const char* kSourceTerminationResidual = "coulomb_rel";
inline constexpr double kSourceTerminationTolerance = 1e-6;

inline constexpr bool kTrajectoryEquivalence = false;
inline constexpr bool kSolverEquivalence = false;
inline constexpr bool kPhysicalOutcomeEquivalence = false;
inline constexpr bool kFig06Parity = false;
inline constexpr bool kVideo06Parity = false;
inline constexpr bool kTimingComparability = false;

/// Process-global ERP override scoped to the active dynamics adapter scene.
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
constexpr std::size_t leaningCardCount(std::size_t levelCount)
{
  return levelCount * (levelCount + 1u);
}

//==============================================================================
constexpr std::size_t bridgeCardCount(std::size_t levelCount)
{
  return levelCount == 0u ? 0u : levelCount * (levelCount - 1u) / 2u;
}

//==============================================================================
constexpr std::size_t cardCount(std::size_t levelCount)
{
  return leaningCardCount(levelCount) + bridgeCardCount(levelCount);
}

enum class CardKind
{
  Bridge,
  LeanLeft,
  LeanRight,
};

//==============================================================================
inline const char* cardKindLabel(CardKind kind)
{
  switch (kind) {
    case CardKind::Bridge:
      return "bridge";
    case CardKind::LeanLeft:
      return "tent_left";
    case CardKind::LeanRight:
      return "tent_right";
  }
  return "unknown";
}

struct CardSpec
{
  std::string name;
  CardKind kind = CardKind::Bridge;
  std::size_t level = 0u;
  std::size_t tent = 0u;
  Eigen::Vector3d size = Eigen::Vector3d::Zero();
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
};

struct CubeSpec
{
  std::string name;
  std::size_t index = 0u;
  Eigen::Vector3d size = Eigen::Vector3d::Zero();
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
};

//==============================================================================
inline double degreesToRadians(double degrees)
{
  return degrees * kPi / 180.0;
}

//==============================================================================
inline Eigen::Matrix3d rotationAboutY(double angleDegrees)
{
  return Eigen::AngleAxisd(
             degreesToRadians(angleDegrees), Eigen::Vector3d::UnitY())
      .toRotationMatrix();
}

//==============================================================================
inline std::vector<CardSpec> makeCardSpecs(std::size_t levelCount)
{
  if (levelCount == 0u)
    throw std::invalid_argument(
        "author card house requires at least one level");

  std::vector<CardSpec> specs;
  specs.reserve(cardCount(levelCount));
  for (std::size_t level = 0u; level < levelCount; ++level) {
    const std::size_t tentCount = levelCount - level;
    for (std::size_t tent = 0u; tent < tentCount; ++tent) {
      const double xOffset = (static_cast<double>(tent)
                              - static_cast<double>(tentCount) / 2.0 + 0.5)
                             * kTentWidth;

      if (level != 0u) {
        CardSpec bridge;
        bridge.name
            = "bridge_L" + std::to_string(level) + "_T" + std::to_string(tent);
        bridge.kind = CardKind::Bridge;
        bridge.level = level;
        bridge.tent = tent;
        bridge.size = Eigen::Vector3d(
            2.0 * kCardHalfLength,
            2.0 * kCardHalfDepth,
            2.0 * kCardHalfThickness);
        bridge.transform.linear() = rotationAboutY(kBridgeAngleDegrees);
        bridge.transform.translation()
            = Eigen::Vector3d(xOffset, 0.0, level * kTentHeight);
        specs.push_back(bridge);
      }

      CardSpec left;
      left.name
          = "tent_left_L" + std::to_string(level) + "_T" + std::to_string(tent);
      left.kind = CardKind::LeanLeft;
      left.level = level;
      left.tent = tent;
      left.size = Eigen::Vector3d(
          2.0 * kCardHalfThickness,
          2.0 * kCardHalfDepth,
          2.0 * kCardHalfLength);
      left.transform.linear() = rotationAboutY(kLeanFromVerticalDegrees);
      left.transform.translation() = Eigen::Vector3d(
          xOffset - kTentHalfGap, 0.0, (level + 0.5) * kTentHeight);
      specs.push_back(left);

      CardSpec right = left;
      right.name = "tent_right_L" + std::to_string(level) + "_T"
                   + std::to_string(tent);
      right.kind = CardKind::LeanRight;
      right.transform.linear() = rotationAboutY(-kLeanFromVerticalDegrees);
      right.transform.translation().x() = xOffset + kTentHalfGap;
      specs.push_back(right);
    }
  }
  return specs;
}

//==============================================================================
inline std::vector<CubeSpec> makeCubeSpecs(
    std::size_t levelCount = kDefaultLevelCount)
{
  if (levelCount == 0u)
    throw std::invalid_argument(
        "author card house requires at least one level");

  std::vector<CubeSpec> specs;
  specs.reserve(kCubeCount);
  const double topZ = levelCount * kTentHeight + kDropHeight;
  const double spacing = 3.0 * kCubeHalfSize;
  for (std::size_t index = 0u; index < kCubeCount; ++index) {
    CubeSpec cube;
    cube.name = "cube_" + std::to_string(index);
    cube.index = index;
    cube.size = Eigen::Vector3d::Constant(2.0 * kCubeHalfSize);
    cube.transform.translation() = Eigen::Vector3d(
        (static_cast<double>(index) - 0.5 * (kCubeCount - 1u)) * spacing,
        0.0,
        topZ);
    specs.push_back(cube);
  }
  return specs;
}

//==============================================================================
inline dart::constraint::ExactCoulombFbfConstraintSolverOptions
dartConstructionSolverOptions()
{
  dart::constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.fallbackToBoxedLcp = false;
  options.includeConstraintRegularization = false;
  options.useMatrixFreeDelassusOperator = false;
  options.useContactRowDelassusOperator = true;
  options.assembleDenseContactRowSnapshot = false;
  options.enableWarmStart = true;
  options.enableStepSizePersistence = true;
  options.stepSizeRecoveryGrowthFactor = 1.0 / kSourceArmijoShrink;
  options.warmStartMatchDistance = kSourceWarmStartMatchRadius;
  options.seedNormalImpulseFromDiagonal = true;
  options.useMatrixFreeDelassusSeed = true;
  options.enableProjectedGradientRetry = false;
  options.enableDenseResidualPolish = false;
  options.maxOuterIterations = kSourceMaxOuterIterations;
  options.acceptOuterMaxIterations = false;
  options.tolerance = kSourceOuterTolerance;
  options.initialStepSize = std::numeric_limits<double>::quiet_NaN();
  options.capInitialStepSizeAtSafeBound = true;
  // DART's conservative spectral factor is 0.5. A scale of ten represents
  // the author's gamma_c=5 configuration, subject to DART's own line search.
  options.stepSizeScale = 2.0 * kSourceGammaC;
  options.outerRelaxation = 1.0;
  options.couplingVariationTolerance = kSourceArmijoRhoHigh;
  options.shrinkFactor = kSourceArmijoShrink;
  options.maxStepShrinkIterations = kSourceArmijoMaxBacktracks;
  options.enableAdaptiveStepSize = true;
  options.spectralIterations = 10;
  options.innerMaxSweeps = kSourceInnerGaussSeidelSweeps;
  options.innerLocalSolver = dart::constraint::ExactCoulombFbfLocalBlockSolver::
      ExactMetricProjection;
  options.runFixedInnerSweeps = true;
  options.acceptInnerMaxIterations = true;
  options.innerLocalIterations = 8;
  options.innerTolerance = kSourceInnerTolerance;
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
inline dart::constraint::ExactCoulombFbfConstraintSolverOptions
dartSourceContinuationSolverOptions()
{
  auto options = dartConstructionSolverOptions();
  options.maxResidualHistorySamples = kSourceContinuationResidualHistorySamples;
  options.maxResidualHistoryRecords = kSourceContinuationResidualHistoryRecords;
  return options;
}

//==============================================================================
inline dart::constraint::ExactCoulombFbfSourceContinuationOptions
dartSourceContinuationOptions()
{
  dart::constraint::ExactCoulombFbfSourceContinuationOptions options;
  options.enabled = true;
  options.residualCheckInterval = kSourceResidualCheckInterval;
  options.plateauPatience = kSourcePlateauPatience;
  options.plateauRelativeTolerance = kSourcePlateauRelativeTolerance;
  options.stepSizeBacktrackLimit = kSourceArmijoMaxBacktracks;
  options.couplingVariationSkipThreshold
      = kSourceCouplingVariationSkipThreshold;
  return options;
}

//==============================================================================
inline dart::constraint::ExactCoulombFbfCrossStepPolicyOptions
dartConstructionCrossStepPolicyOptions()
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
inline dart::constraint::ExactCoulombFbfCrossStepPolicyOptions
dartSourceContinuationCrossStepPolicyOptions()
{
  dart::constraint::ExactCoulombFbfCrossStepPolicyOptions options;
  options.warmStartMatchMode = dart::constraint::
      ExactCoulombFbfWarmStartMatchMode::OrderedBodyBLocalFeature;
  options.warmStartNormalCosine = kSourceWarmStartNormalCosine;
  options.useStrictWarmStartMatchDistance = true;
  options.warmStartMaxAge = kSourceWarmStartMaxAge;
  options.persistentStepSizeSafeBoundScale
      = kSourcePersistentStepSizeSafeBoundScale;
  options.minimumStepSize = kSourceMinimumStepSize;
  options.maximumStepSize = kSourceMaximumStepSize;
  options.warmStartResidualThreshold = kSourceWarmStartResidualThreshold;
  options.warmStartStepSizeCap = kSourceWarmStartGammaCap;
  options.persistUncappedStepSizeOnWarmStartCap = true;
  options.requireResidualImprovementForUnconvergedCacheSave = true;
  return options;
}

//==============================================================================
inline void configureDartContactMaterial(
    dart::dynamics::DynamicsAspect* dynamics)
{
  if (dynamics == nullptr)
    throw std::invalid_argument("author card-house contact material is null");

  dynamics->setFrictionCoeff(kFriction);
  dynamics->setRestitutionCoeff(kDartRestitution);
  dynamics->setPrimarySlipCompliance(kDartSlipCompliance);
  dynamics->setSecondarySlipCompliance(kDartSlipCompliance);
  dynamics->setFirstFrictionDirection(Eigen::Vector3d::Zero());
  dynamics->setFirstFrictionDirectionFrame(nullptr);
}

//==============================================================================
inline void configureDartFreeJoint(dart::dynamics::FreeJoint* joint)
{
  if (joint == nullptr || joint->getNumDofs() != 6u)
    throw std::invalid_argument("author card-house free joint is invalid");
  for (std::size_t index = 0u; index < joint->getNumDofs(); ++index) {
    joint->setDampingCoefficient(index, 0.0);
    joint->setCoulombFriction(index, 0.0);
  }
}

//==============================================================================
inline dart::constraint::BoxedLcpConstraintSolver::
    MatrixFreeContactSolverOptions
    dartBoxedMatrixFreeContactSolverOptions()
{
  dart::constraint::BoxedLcpConstraintSolver::MatrixFreeContactSolverOptions
      options;
  options.mEnabled = false;
  options.mMinRows = 193u;
  options.mMaxIterations = 30;
  options.mSor = 0.9;
  options.mDeltaTolerance = 1e-6;
  options.mRelativeDeltaTolerance = 1e-3;
  options.mEpsilonForDivision = 1e-9;
  return options;
}

//==============================================================================
inline void configureDartBoxedBaseline(
    dart::constraint::BoxedLcpConstraintSolver* solver)
{
  if (solver == nullptr)
    throw std::invalid_argument("author card-house boxed solver is null");

  solver->setBoxedLcpSolver(
      std::make_shared<dart::constraint::DantzigBoxedLcpSolver>());
  solver->setSecondaryBoxedLcpSolver(
      std::make_shared<dart::constraint::PgsBoxedLcpSolver>());
  solver->setMatrixFreeContactSolverOptions(
      dartBoxedMatrixFreeContactSolverOptions());
}

//==============================================================================
inline void configureDartCollisionGeneration(
    dart::constraint::ConstraintSolver* solver,
    bool allowNegativePenetrationDepthContacts = false)
{
  if (solver == nullptr)
    throw std::invalid_argument("author card-house constraint solver is null");

  auto& options = solver->getCollisionOption();
  options.enableContact = true;
  options.allowNegativePenetrationDepthContacts
      = allowNegativePenetrationDepthContacts;
  options.collisionFilter
      = std::make_shared<dart::collision::BodyNodeCollisionFilter>();
}

//==============================================================================
inline std::size_t configureDartContactGapValues(
    const std::shared_ptr<dart::simulation::World>& world, bool enabled)
{
  if (!world || world->getConstraintSolver() == nullptr)
    throw std::invalid_argument("author card-house contact-gap world is null");

  auto* solver = world->getConstraintSolver();
  auto detector
      = std::dynamic_pointer_cast<dart::collision::NativeCollisionDetector>(
          solver->getCollisionDetector());
  if (!detector)
    throw std::runtime_error(
        "author card-house contact gaps require Native collision");

  detector->clearContactGaps();
  solver->getCollisionOption().allowNegativePenetrationDepthContacts = enabled;

  std::size_t collisionShapeCount = 0u;
  for (std::size_t skeletonIndex = 0u; skeletonIndex < world->getNumSkeletons();
       ++skeletonIndex) {
    const auto skeleton = world->getSkeleton(skeletonIndex);
    if (!skeleton)
      continue;
    for (std::size_t bodyIndex = 0u; bodyIndex < skeleton->getNumBodyNodes();
         ++bodyIndex) {
      auto* body = skeleton->getBodyNode(bodyIndex);
      const std::size_t shapeCount
          = body->getNumShapeNodesWith<dart::dynamics::CollisionAspect>();
      for (std::size_t shapeIndex = 0u; shapeIndex < shapeCount; ++shapeIndex) {
        auto* shape = body->getShapeNodeWith<dart::dynamics::CollisionAspect>(
            shapeIndex);
        if (shape == nullptr)
          throw std::runtime_error(
              "author card-house collision ShapeFrame is missing");
        if (enabled) {
          detector->setContactGap(
              shape,
              skeleton->getName() == "ground_plane" ? kSourceGroundContactGap
                                                    : kContactGap);
        }
        ++collisionShapeCount;
      }
    }
  }
  return collisionShapeCount;
}

struct BodyContract
{
  std::string name;
  std::string role;
  Eigen::Vector3d size = Eigen::Vector3d::Zero();
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  double friction = 0.0;
  double mass = 0.0;
  Eigen::Vector3d localCenterOfMass = Eigen::Vector3d::Zero();
  Eigen::Matrix3d moment = Eigen::Matrix3d::Zero();
  std::size_t collisionShapeCount = 0u;
  Eigen::Isometry3d collisionShapeRelativeTransform
      = Eigen::Isometry3d::Identity();
  bool collidable = false;
  double primaryFriction = 0.0;
  double secondaryFriction = 0.0;
  double restitution = 0.0;
  double primarySlipCompliance = 0.0;
  double secondarySlipCompliance = 0.0;
  Eigen::Vector3d firstFrictionDirection = Eigen::Vector3d::Zero();
  bool usesDefaultFrictionDirectionFrame = false;
  bool gravityMode = true;
  bool mobile = true;
  Eigen::Vector3d linearVelocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d angularVelocity = Eigen::Vector3d::Zero();
};

struct ConfigurationContract
{
  std::string binaryRole;
  std::string binarySourceSha256;
  std::size_t levelCount = 0u;
  double timeStep = 0.0;
  Eigen::Vector3d gravity = Eigen::Vector3d::Zero();
  std::size_t simulationThreads = 0u;
  bool deactivationEnabled = true;
  std::string collisionDetector;
  std::string contactManifold;
  std::size_t maxContacts = 0u;
  std::size_t maxContactsPerPair = 0u;
  bool splitImpulseEnabled = false;
  dart::constraint::ExactCoulombFbfConstraintSolverOptions solverOptions;
  std::string groundShape;
  double groundFriction = 0.0;
  bool groundMobile = true;
  std::vector<BodyContract> cards;
  std::vector<BodyContract> cubes;
};

struct DynamicsAdapterContract
{
  const DynamicsScenario* scenario = nullptr;
  bool sourceContinuationScene = false;
  std::size_t levelCount = 0u;
  double timeStep = 0.0;
  double worldTime = 0.0;
  Eigen::Vector3d gravity = Eigen::Vector3d::Zero();
  std::size_t simulationThreads = 0u;
  bool deactivationEnabled = true;
  std::string collisionDetector;
  std::string contactManifold;
  std::size_t maxContacts = 0u;
  std::size_t maxContactsPerPair = 0u;
  bool contactGenerationEnabled = false;
  bool negativePenetrationDepthContactsAllowed = false;
  bool defaultEmptyBodyNodeCollisionFilter = false;
  double groundContactGap = 0.0;
  double dynamicShapeContactGap = 0.0;
  std::size_t collisionShapeFrameCount = 0u;
  std::size_t collisionShapeFramesWithContactGap = 0u;
  std::size_t groundShapeFramesWithContactGap = 0u;
  std::size_t dynamicShapeFramesWithContactGap = 0u;
  std::string solverLane;
  bool splitImpulseEnabled = false;
  double observedContactErrorReductionParameter = 0.0;
  bool exactOptionsAvailable = false;
  dart::constraint::ExactCoulombFbfConstraintSolverOptions exactOptions;
  dart::constraint::ExactCoulombFbfCrossStepPolicyOptions exactCrossStepOptions;
  dart::constraint::ExactCoulombFbfSourceContinuationOptions
      exactSourceContinuationOptions;
  bool exactSourceContinuationActive = false;
  bool exactPostCorrectionProjectionEnabled = true;
  bool exactSourceInnerInitializationRequested = false;
  bool exactSourceInnerInitializationActive = false;
  bool exactColoredBlockGaussSeidelEnabled = false;
  bool exactParticipantAffinityEnabled = false;
  bool boxedOptionsAvailable = false;
  std::string boxedPrimarySolver;
  std::string boxedSecondarySolver;
  dart::constraint::BoxedLcpConstraintSolver::MatrixFreeContactSolverOptions
      boxedMatrixFreeOptions;
  double primaryFriction = 0.0;
  double secondaryFriction = 0.0;
  double restitution = 0.0;
  double primarySlipCompliance = 0.0;
  double secondarySlipCompliance = 0.0;
  Eigen::Vector3d firstFrictionDirection = Eigen::Vector3d::Zero();
  bool usesDefaultFrictionDirectionFrame = false;
  std::size_t cardCount = 0u;
  std::size_t cubeCount = 0u;
  std::size_t releasedCubeCount = 0u;
  bool finiteState = false;
  std::string binarySourceSha256;
};

//==============================================================================
inline BodyContract inspectBox(
    const std::shared_ptr<dart::simulation::World>& world,
    const std::string& name,
    const std::string& role)
{
  const auto skeleton = world->getSkeleton(name);
  if (!skeleton || skeleton->getNumBodyNodes() != 1u)
    throw std::runtime_error("author card-house body is missing: " + name);

  const auto* body = skeleton->getBodyNode(0u);
  const auto* joint
      = dynamic_cast<const dart::dynamics::FreeJoint*>(skeleton->getJoint(0u));
  if (joint == nullptr || joint->getNumDofs() != 6u)
    throw std::runtime_error("author card-house body is not free: " + name);
  for (std::size_t index = 0u; index < joint->getNumDofs(); ++index) {
    if (joint->getDampingCoefficient(index) != 0.0
        || joint->getCoulombFriction(index) != 0.0) {
      throw std::runtime_error(
          "author card-house joint material changed: " + name);
    }
  }
  const std::size_t collisionShapeCount
      = body->getNumShapeNodesWith<dart::dynamics::CollisionAspect>();
  if (collisionShapeCount != 1u)
    throw std::runtime_error(
        "author card-house collision shape count changed: " + name);
  const auto* node
      = body->getShapeNodeWith<dart::dynamics::CollisionAspect>(0u);
  if (node == nullptr || node->getDynamicsAspect() == nullptr)
    throw std::runtime_error(
        "author card-house collision shape is missing: " + name);

  const auto shape = std::dynamic_pointer_cast<const dart::dynamics::BoxShape>(
      node->getShape());
  if (!shape)
    throw std::runtime_error("author card-house body is not a box: " + name);

  BodyContract contract;
  contract.name = name;
  contract.role = role;
  contract.size = shape->getSize();
  contract.pose = body->getWorldTransform();
  contract.friction = node->getDynamicsAspect()->getFrictionCoeff();
  contract.mass = body->getInertia().getMass();
  contract.localCenterOfMass = body->getInertia().getLocalCOM();
  contract.moment = body->getInertia().getMoment();
  contract.collisionShapeCount = collisionShapeCount;
  contract.collisionShapeRelativeTransform = node->getRelativeTransform();
  contract.collidable = node->getCollisionAspect()->isCollidable();
  contract.primaryFriction
      = node->getDynamicsAspect()->getPrimaryFrictionCoeff();
  contract.secondaryFriction
      = node->getDynamicsAspect()->getSecondaryFrictionCoeff();
  contract.restitution = node->getDynamicsAspect()->getRestitutionCoeff();
  contract.primarySlipCompliance
      = node->getDynamicsAspect()->getPrimarySlipCompliance();
  contract.secondarySlipCompliance
      = node->getDynamicsAspect()->getSecondarySlipCompliance();
  contract.firstFrictionDirection
      = node->getDynamicsAspect()->getFirstFrictionDirection();
  contract.usesDefaultFrictionDirectionFrame
      = node->getDynamicsAspect()->getFirstFrictionDirectionFrame() == nullptr;
  contract.gravityMode = body->getGravityMode();
  contract.mobile = skeleton->isMobile();
  contract.linearVelocity = body->getLinearVelocity();
  contract.angularVelocity = body->getAngularVelocity();
  return contract;
}

//==============================================================================
inline ConfigurationContract inspectConfigurationContract(
    const std::shared_ptr<dart::simulation::World>& world,
    std::size_t levelCount,
    const std::string& binaryRole,
    const std::string& binarySourceSha256)
{
  if (!world)
    throw std::runtime_error("author card-house contract has no world");

  auto* solver
      = dynamic_cast<dart::constraint::ExactCoulombFbfConstraintSolver*>(
          world->getConstraintSolver());
  if (solver == nullptr)
    throw std::runtime_error("author card-house contract requires exact FBF");

  const auto detector = std::dynamic_pointer_cast<
      const dart::collision::NativeCollisionDetector>(
      solver->getCollisionDetector());
  if (!detector)
    throw std::runtime_error(
        "author card-house contract requires Native collision");

  const auto ground = world->getSkeleton("ground_plane");
  if (!ground || ground->getNumBodyNodes() != 1u)
    throw std::runtime_error("author card-house ground plane is missing");
  const auto* groundNode
      = ground->getBodyNode(0u)
            ->getShapeNodeWith<dart::dynamics::CollisionAspect>(0u);
  if (groundNode == nullptr || groundNode->getDynamicsAspect() == nullptr
      || !std::dynamic_pointer_cast<const dart::dynamics::PlaneShape>(
          groundNode->getShape())) {
    throw std::runtime_error("author card-house ground is not a plane");
  }

  const auto cardSpecs = makeCardSpecs(levelCount);
  const auto cubeSpecs = makeCubeSpecs(levelCount);
  if (world->getNumSkeletons() != 1u + cardSpecs.size() + cubeSpecs.size()) {
    throw std::runtime_error(
        "author card-house contract has an unexpected skeleton count");
  }

  ConfigurationContract contract;
  contract.binaryRole = binaryRole;
  contract.binarySourceSha256 = binarySourceSha256;
  contract.levelCount = levelCount;
  contract.timeStep = world->getTimeStep();
  contract.gravity = world->getGravity();
  contract.simulationThreads = world->getNumSimulationThreads();
  contract.deactivationEnabled = world->getDeactivationOptions().mEnabled;
  contract.collisionDetector = detector->getType();
  contract.contactManifold
      = detector->getContactManifoldMode()
                == dart::collision::NativeCollisionDetector::
                    ContactManifoldMode::FourPointPlanar
            ? "four_point_planar"
            : "compact";
  const auto& collisionOption = solver->getCollisionOption();
  contract.maxContacts = collisionOption.maxNumContacts;
  contract.maxContactsPerPair = collisionOption.maxNumContactsPerPair;
  contract.splitImpulseEnabled = solver->isSplitImpulseEnabled();
  contract.solverOptions = solver->getExactCoulombOptions();
  contract.groundShape = "plane";
  contract.groundFriction = groundNode->getDynamicsAspect()->getFrictionCoeff();
  contract.groundMobile = ground->isMobile();
  contract.cards.reserve(cardSpecs.size());
  for (const auto& spec : cardSpecs)
    contract.cards.push_back(
        inspectBox(world, spec.name, cardKindLabel(spec.kind)));
  contract.cubes.reserve(cubeSpecs.size());
  for (const auto& spec : cubeSpecs)
    contract.cubes.push_back(inspectBox(world, spec.name, "cube"));
  return contract;
}

//==============================================================================
inline std::size_t releasedCubeCount(
    const std::shared_ptr<dart::simulation::World>& world,
    std::size_t levelCount)
{
  if (!world)
    throw std::runtime_error("author card-house release has no world");

  std::size_t released = 0u;
  for (const auto& spec : makeCubeSpecs(levelCount)) {
    const auto cube = world->getSkeleton(spec.name);
    if (!cube || cube->getNumBodyNodes() != 1u)
      throw std::runtime_error(
          "author card-house release cube is missing: " + spec.name);
    if (cube->isMobile())
      ++released;
  }
  return released;
}

//==============================================================================
inline bool cubesReleased(
    const std::shared_ptr<dart::simulation::World>& world,
    std::size_t levelCount)
{
  return releasedCubeCount(world, levelCount) == kCubeCount;
}

//==============================================================================
inline void releaseCubes(
    const std::shared_ptr<dart::simulation::World>& world,
    std::size_t levelCount)
{
  if (!world)
    throw std::runtime_error("author card-house release has no world");

  for (const auto& spec : makeCubeSpecs(levelCount)) {
    const auto cube = world->getSkeleton(spec.name);
    if (!cube || cube->getNumBodyNodes() != 1u)
      throw std::runtime_error(
          "author card-house release cube is missing: " + spec.name);
    cube->setMobile(true);
  }
}

//==============================================================================
inline DynamicsAdapterContract inspectDynamicsAdapterContract(
    const std::shared_ptr<dart::simulation::World>& world,
    const DynamicsScenario& scenario,
    const std::string& binarySourceSha256,
    bool sourceInnerInitializationRequested = false)
{
  if (!world)
    throw std::runtime_error(
        "author card-house dynamics contract has no world");

  const auto* solver = world->getConstraintSolver();
  if (solver == nullptr)
    throw std::runtime_error("author card-house dynamics solver is missing");
  const auto* exact
      = dynamic_cast<const dart::constraint::ExactCoulombFbfConstraintSolver*>(
          solver);
  const auto* boxed
      = dynamic_cast<const dart::constraint::BoxedLcpConstraintSolver*>(solver);
  if (exact == nullptr && boxed == nullptr)
    throw std::runtime_error(
        "author card-house dynamics solver is unsupported");

  const auto detector = std::dynamic_pointer_cast<
      const dart::collision::NativeCollisionDetector>(
      solver->getCollisionDetector());
  if (!detector)
    throw std::runtime_error(
        "author card-house dynamics contract requires Native collision");

  const auto cardSpecs = makeCardSpecs(scenario.levelCount);
  const auto cubeSpecs = makeCubeSpecs(scenario.levelCount);
  if (world->getNumSkeletons() != 1u + cardSpecs.size() + cubeSpecs.size()) {
    throw std::runtime_error(
        "author card-house dynamics contract has an unexpected skeleton "
        "count");
  }

  const auto validateContactMaterial = [](const auto* dynamics,
                                          const std::string& label) {
    if (dynamics == nullptr
        || std::abs(dynamics->getPrimaryFrictionCoeff() - kFriction) > 1e-12
        || std::abs(dynamics->getSecondaryFrictionCoeff() - kFriction) > 1e-12
        || std::abs(dynamics->getRestitutionCoeff() - kDartRestitution) > 1e-12
        || std::abs(dynamics->getPrimarySlipCompliance() - kDartSlipCompliance)
               > 1e-12
        || std::abs(
               dynamics->getSecondarySlipCompliance() - kDartSlipCompliance)
               > 1e-12
        || !dynamics->getFirstFrictionDirection().isZero(0.0)
        || dynamics->getFirstFrictionDirectionFrame() != nullptr) {
      throw std::runtime_error(
          "author card-house contact material changed: " + label);
    }
  };

  const auto ground = world->getSkeleton("ground_plane");
  if (!ground || ground->isMobile() || ground->getNumBodyNodes() != 1u)
    throw std::runtime_error("author card-house dynamics ground is invalid");
  const auto* groundJoint
      = dynamic_cast<const dart::dynamics::FreeJoint*>(ground->getJoint(0u));
  const auto* groundBody = ground->getBodyNode(0u);
  if (groundJoint == nullptr || groundJoint->getNumDofs() != 6u
      || groundBody->getNumShapeNodesWith<dart::dynamics::CollisionAspect>()
             != 1u
      || !groundBody->getGravityMode()
      || !groundBody->getWorldTransform().matrix().isApprox(
          Eigen::Isometry3d::Identity().matrix(), 0.0)
      || !groundBody->getLinearVelocity().isZero(0.0)
      || !groundBody->getAngularVelocity().isZero(0.0)) {
    throw std::runtime_error("author card-house dynamics ground body changed");
  }
  for (std::size_t index = 0u; index < groundJoint->getNumDofs(); ++index) {
    if (groundJoint->getDampingCoefficient(index) != 0.0
        || groundJoint->getCoulombFriction(index) != 0.0) {
      throw std::runtime_error(
          "author card-house dynamics ground joint changed");
    }
  }
  const auto* groundNode
      = groundBody->getShapeNodeWith<dart::dynamics::CollisionAspect>(0u);
  const auto groundPlane
      = groundNode == nullptr
            ? nullptr
            : std::dynamic_pointer_cast<const dart::dynamics::PlaneShape>(
                groundNode->getShape());
  if (groundNode == nullptr || groundNode->getDynamicsAspect() == nullptr
      || groundNode->getCollisionAspect() == nullptr
      || !groundNode->getCollisionAspect()->isCollidable() || !groundPlane
      || !groundNode->getRelativeTransform().matrix().isApprox(
          Eigen::Isometry3d::Identity().matrix(), 0.0)
      || !groundPlane->getNormal().isApprox(Eigen::Vector3d::UnitZ(), 0.0)
      || groundPlane->getOffset() != 0.0) {
    throw std::runtime_error(
        "author card-house dynamics ground contract changed");
  }
  validateContactMaterial(groundNode->getDynamicsAspect(), "ground_plane");

  const bool initialState = std::abs(world->getTime()) <= 1e-15;
  const auto validateBody = [initialState](
                                const BodyContract& actual,
                                const Eigen::Vector3d& expectedSize,
                                const Eigen::Isometry3d& expectedPose,
                                double expectedMass,
                                bool expectedMobile,
                                const std::string& label) {
    if (!actual.size.isApprox(expectedSize, 1e-15)
        || std::abs(actual.mass - expectedMass) > 1e-10
        || !actual.localCenterOfMass.isZero(1e-15)
        || !actual.moment.isApprox(
            dart::dynamics::BoxShape::computeInertia(
                expectedSize, expectedMass),
            1e-12)
        || actual.collisionShapeCount != 1u
        || !actual.collisionShapeRelativeTransform.matrix().isApprox(
            Eigen::Isometry3d::Identity().matrix(), 0.0)
        || !actual.collidable
        || std::abs(actual.primaryFriction - kFriction) > 1e-12
        || std::abs(actual.secondaryFriction - kFriction) > 1e-12
        || std::abs(actual.restitution - kDartRestitution) > 1e-12
        || std::abs(actual.primarySlipCompliance - kDartSlipCompliance) > 1e-12
        || std::abs(actual.secondarySlipCompliance - kDartSlipCompliance)
               > 1e-12
        || !actual.firstFrictionDirection.isZero(0.0)
        || !actual.usesDefaultFrictionDirectionFrame || !actual.gravityMode
        || actual.mobile != expectedMobile) {
      throw std::runtime_error(
          "author card-house dynamics physical contract changed: " + label);
    }
    if (initialState
        && (!actual.pose.matrix().isApprox(expectedPose.matrix(), 1e-14)
            || !actual.linearVelocity.isZero(0.0)
            || !actual.angularVelocity.isZero(0.0))) {
      throw std::runtime_error(
          "author card-house dynamics initial state changed: " + label);
    }
  };

  for (const auto& spec : cardSpecs) {
    validateBody(
        inspectBox(world, spec.name, cardKindLabel(spec.kind)),
        spec.size,
        spec.transform,
        kCardMass,
        true,
        spec.name);
  }
  const std::size_t released = releasedCubeCount(world, scenario.levelCount);
  if (released != 0u && released != kCubeCount) {
    throw std::runtime_error(
        "author card-house dynamics cube release is only partially applied");
  }
  if (initialState && released != 0u) {
    throw std::runtime_error(
        "author card-house dynamics cubes released before initial contract");
  }
  for (const auto& spec : cubeSpecs) {
    validateBody(
        inspectBox(world, spec.name, "cube"),
        spec.size,
        spec.transform,
        kCubeMass,
        released == kCubeCount,
        spec.name);
  }

  bool finiteState = true;
  const auto inspectFiniteState
      = [&finiteState](
            const std::shared_ptr<dart::dynamics::Skeleton>& skeleton) {
          if (!skeleton || skeleton->getNumBodyNodes() != 1u) {
            finiteState = false;
            return;
          }
          const auto* body = skeleton->getBodyNode(0u);
          finiteState = finiteState
                        && body->getWorldTransform().matrix().allFinite()
                        && body->getLinearVelocity().allFinite()
                        && body->getAngularVelocity().allFinite();
        };
  for (const auto& spec : cardSpecs)
    inspectFiniteState(world->getSkeleton(spec.name));
  for (const auto& spec : cubeSpecs)
    inspectFiniteState(world->getSkeleton(spec.name));

  DynamicsAdapterContract contract;
  contract.scenario = &scenario;
  contract.sourceContinuationScene = scenario.sourceContinuation;
  contract.levelCount = scenario.levelCount;
  contract.timeStep = world->getTimeStep();
  contract.worldTime = world->getTime();
  contract.gravity = world->getGravity();
  contract.simulationThreads = world->getNumSimulationThreads();
  contract.deactivationEnabled = world->getDeactivationOptions().mEnabled;
  contract.collisionDetector = detector->getType();
  contract.contactManifold
      = detector->getContactManifoldMode()
                == dart::collision::NativeCollisionDetector::
                    ContactManifoldMode::FourPointPlanar
            ? "four_point_planar"
            : "compact";
  const auto& collisionOption = solver->getCollisionOption();
  const auto bodyNodeFilter = std::dynamic_pointer_cast<
      const dart::collision::BodyNodeCollisionFilter>(
      collisionOption.collisionFilter);
  const auto* const collisionFilter = collisionOption.collisionFilter.get();
  const dart::collision::BodyNodeCollisionFilter emptyBodyNodeFilter;
  if (!collisionOption.enableContact
      || collisionOption.allowNegativePenetrationDepthContacts
             != scenario.sourceContactGapValuesRepresented
      || !bodyNodeFilter || collisionFilter == nullptr
      || typeid(*collisionFilter)
             != typeid(dart::collision::BodyNodeCollisionFilter)
      || bodyNodeFilter->getRevision() != emptyBodyNodeFilter.getRevision()) {
    throw std::runtime_error(
        "author card-house collision generation policy changed");
  }

  const double expectedGroundContactGap
      = scenario.sourceContactGapValuesRepresented ? kSourceGroundContactGap
                                                   : 0.0;
  const double expectedDynamicShapeContactGap
      = scenario.sourceContactGapValuesRepresented ? kContactGap : 0.0;
  std::size_t collisionShapeFrameCount = 0u;
  std::size_t collisionShapeFramesWithContactGap = 0u;
  std::size_t groundShapeFramesWithContactGap = 0u;
  std::size_t dynamicShapeFramesWithContactGap = 0u;
  for (std::size_t skeletonIndex = 0u; skeletonIndex < world->getNumSkeletons();
       ++skeletonIndex) {
    const auto skeleton = world->getSkeleton(skeletonIndex);
    if (!skeleton)
      throw std::runtime_error(
          "author card-house collision-gap skeleton is missing");
    const bool groundShape = skeleton->getName() == "ground_plane";
    for (std::size_t bodyIndex = 0u; bodyIndex < skeleton->getNumBodyNodes();
         ++bodyIndex) {
      const auto* body = skeleton->getBodyNode(bodyIndex);
      const std::size_t shapeCount
          = body->getNumShapeNodesWith<dart::dynamics::CollisionAspect>();
      for (std::size_t shapeIndex = 0u; shapeIndex < shapeCount; ++shapeIndex) {
        const auto* shape
            = body->getShapeNodeWith<dart::dynamics::CollisionAspect>(
                shapeIndex);
        if (shape == nullptr)
          throw std::runtime_error(
              "author card-house collision-gap ShapeFrame is missing");
        const double expectedContactGap = groundShape
                                              ? expectedGroundContactGap
                                              : expectedDynamicShapeContactGap;
        const double configuredGap = detector->getContactGap(shape);
        if (std::abs(configuredGap - expectedContactGap) > 1e-15) {
          throw std::runtime_error(
              "author card-house contact-gap configuration changed");
        }
        ++collisionShapeFrameCount;
        if (configuredGap > 0.0) {
          ++collisionShapeFramesWithContactGap;
          if (groundShape)
            ++groundShapeFramesWithContactGap;
          else
            ++dynamicShapeFramesWithContactGap;
        }
      }
    }
  }
  contract.maxContacts = collisionOption.maxNumContacts;
  contract.maxContactsPerPair = collisionOption.maxNumContactsPerPair;
  contract.contactGenerationEnabled = collisionOption.enableContact;
  contract.negativePenetrationDepthContactsAllowed
      = collisionOption.allowNegativePenetrationDepthContacts;
  contract.defaultEmptyBodyNodeCollisionFilter = true;
  contract.groundContactGap = expectedGroundContactGap;
  contract.dynamicShapeContactGap = expectedDynamicShapeContactGap;
  contract.collisionShapeFrameCount = collisionShapeFrameCount;
  contract.collisionShapeFramesWithContactGap
      = collisionShapeFramesWithContactGap;
  contract.groundShapeFramesWithContactGap = groundShapeFramesWithContactGap;
  contract.dynamicShapeFramesWithContactGap = dynamicShapeFramesWithContactGap;
  contract.solverLane = exact != nullptr ? "exact_fbf" : "boxed_lcp";
  contract.splitImpulseEnabled = solver->isSplitImpulseEnabled();
  contract.observedContactErrorReductionParameter
      = dart::constraint::ContactConstraint::getErrorReductionParameter();
  contract.exactOptionsAvailable = exact != nullptr;
  contract.exactSourceInnerInitializationRequested
      = sourceInnerInitializationRequested;
  if (exact != nullptr) {
    contract.exactOptions = exact->getExactCoulombOptions();
    contract.exactCrossStepOptions
        = exact->getExactCoulombCrossStepPolicyOptions();
    contract.exactSourceContinuationOptions
        = exact->getExactCoulombSourceContinuationOptions();
    contract.exactSourceContinuationActive
        = exact->getLastExactCoulombSourceContinuationActive();
    contract.exactPostCorrectionProjectionEnabled
        = exact->getExactCoulombPostCorrectionProjectionEnabled();
    contract.exactSourceInnerInitializationActive
        = exact->getExactCoulombSourceInnerInitializationEnabled();
    contract.exactColoredBlockGaussSeidelEnabled
        = exact->getExactCoulombColoredBlockGaussSeidelEnabled();
    contract.exactParticipantAffinityEnabled
        = exact
              ->getExactCoulombColoredBlockGaussSeidelParticipantAffinityEnabled();
  } else {
    contract.boxedOptionsAvailable = true;
    const auto primary = boxed->getBoxedLcpSolver();
    const auto secondary = boxed->getSecondaryBoxedLcpSolver();
    if (!primary || !secondary)
      throw std::runtime_error(
          "author card-house boxed baseline is incomplete");
    contract.boxedPrimarySolver = primary->getType();
    contract.boxedSecondarySolver = secondary->getType();
    contract.boxedMatrixFreeOptions
        = boxed->getMatrixFreeContactSolverOptions();
  }
  const auto* groundDynamics = groundNode->getDynamicsAspect();
  contract.primaryFriction = groundDynamics->getPrimaryFrictionCoeff();
  contract.secondaryFriction = groundDynamics->getSecondaryFrictionCoeff();
  contract.restitution = groundDynamics->getRestitutionCoeff();
  contract.primarySlipCompliance = groundDynamics->getPrimarySlipCompliance();
  contract.secondarySlipCompliance
      = groundDynamics->getSecondarySlipCompliance();
  contract.firstFrictionDirection = groundDynamics->getFirstFrictionDirection();
  contract.usesDefaultFrictionDirectionFrame
      = groundDynamics->getFirstFrictionDirectionFrame() == nullptr;
  contract.cardCount = cardSpecs.size();
  contract.cubeCount = cubeSpecs.size();
  contract.releasedCubeCount = released;
  contract.finiteState = finiteState;
  contract.binarySourceSha256 = binarySourceSha256;
  return contract;
}

//==============================================================================
inline DynamicsAdapterContract inspectDynamicsAdapterContract(
    const std::shared_ptr<dart::simulation::World>& world,
    std::size_t levelCount,
    const std::string& binarySourceSha256,
    bool sourceInnerInitializationRequested = false,
    bool sourceContinuationScene = false)
{
  const auto& scenario = sourceContinuationScene
                             ? kFourLevelSourceContinuationScenario
                             : kFourLevelImpactScenario;
  if (levelCount != scenario.levelCount) {
    throw std::invalid_argument(
        "author card-house dynamics scenario must be selected explicitly");
  }
  return inspectDynamicsAdapterContract(
      world, scenario, binarySourceSha256, sourceInnerInitializationRequested);
}

//==============================================================================
inline void writeJsonString(std::ostream& out, const std::string& value)
{
  out << '"';
  for (const char c : value) {
    switch (c) {
      case '"':
        out << "\\\"";
        break;
      case '\\':
        out << "\\\\";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\r':
        out << "\\r";
        break;
      case '\t':
        out << "\\t";
        break;
      default:
        out << c;
        break;
    }
  }
  out << '"';
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
        "author card-house contract contains an infinite option");
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
      "author card-house local block solver is invalid");
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
      "author card-house warm-start match mode is invalid");
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
inline void writeJsonBodies(
    std::ostream& out, const std::vector<BodyContract>& bodies)
{
  out << '[';
  for (std::size_t index = 0u; index < bodies.size(); ++index) {
    if (index != 0u)
      out << ',';
    const auto& body = bodies[index];
    out << "{\"name\":";
    writeJsonString(out, body.name);
    out << ",\"role\":";
    writeJsonString(out, body.role);
    out << ",\"size_m\":";
    writeJsonVector(out, body.size);
    out << ",\"mass_kg\":" << body.mass << ",\"friction\":" << body.friction
        << ",\"mobile\":" << (body.mobile ? "true" : "false")
        << ",\"initial_pose\":";
    writeJsonPose(out, body.pose);
    out << ",\"initial_linear_velocity_m_s\":";
    writeJsonVector(out, body.linearVelocity);
    out << ",\"initial_angular_velocity_rad_s\":";
    writeJsonVector(out, body.angularVelocity);
    out << '}';
  }
  out << ']';
}

//==============================================================================
inline std::string configurationContractJson(
    const ConfigurationContract& contract)
{
  std::ostringstream out;
  out << std::setprecision(std::numeric_limits<double>::max_digits10);
  out << "{\"schema_version\":";
  writeJsonString(out, kContractSchema);
  out << ",\"kind\":";
  writeJsonString(out, kContractKind);
  out << ",\"author_source\":{\"repository\":";
  writeJsonString(out, kAuthorRepository);
  out << ",\"commit\":";
  writeJsonString(out, kAuthorCommit);
  out << ",\"tree\":";
  writeJsonString(out, kAuthorTree);
  out << ",\"card_house_run_blob\":";
  writeJsonString(out, kAuthorCardHouseBlob);
  out << ",\"card_house_run_py_sha256\":";
  writeJsonString(out, kAuthorCardHouseRunSha256);
  out << ",\"fbf_config_py_sha256\":";
  writeJsonString(out, kAuthorConfigSha256);
  out << ",\"solver_fbf_py_sha256\":";
  writeJsonString(out, kAuthorSolverSha256);
  out << "},\"configuration_spec_source_sha256\":";
  writeJsonString(out, kSpecSourceSha256);
  out << ",\"binary_binding\":{\"role\":";
  writeJsonString(out, contract.binaryRole);
  out << ",\"implementation_source_sha256\":";
  writeJsonString(out, contract.binarySourceSha256);
  out << "},\"source_configuration\":{\"scene\":{\"demo_scene_id\":";
  writeJsonString(out, kDemoSceneId);
  out << ",\"levels\":" << contract.levelCount
      << ",\"leaning_cards\":" << leaningCardCount(contract.levelCount)
      << ",\"bridges\":" << bridgeCardCount(contract.levelCount)
      << ",\"cards\":" << cardCount(contract.levelCount)
      << ",\"cubes\":" << kCubeCount << "},\"cards\":{\"lean_size_m\":["
      << 2.0 * kCardHalfThickness << ',' << 2.0 * kCardHalfDepth << ','
      << 2.0 * kCardHalfLength << "],\"bridge_size_m\":["
      << 2.0 * kCardHalfLength << ',' << 2.0 * kCardHalfDepth << ','
      << 2.0 * kCardHalfThickness << "],\"density_kg_m3\":" << kCardDensity
      << ",\"mass_kg\":" << kCardMass
      << ",\"lean_from_horizontal_degrees\":" << kLeanFromHorizontalDegrees
      << ",\"lean_from_vertical_degrees\":" << kLeanFromVerticalDegrees
      << ",\"bridge_angle_degrees\":" << kBridgeAngleDegrees
      << ",\"tent_half_gap_m\":" << kTentHalfGap
      << ",\"tent_width_m\":" << kTentWidth
      << ",\"tent_height_m\":" << kTentHeight
      << ",\"source_mobile\":true},\"cubes\":{\"size_m\":["
      << 2.0 * kCubeHalfSize << ',' << 2.0 * kCubeHalfSize << ','
      << 2.0 * kCubeHalfSize << "],\"density_kg_m3\":" << kCubeDensity
      << ",\"mass_kg\":" << kCubeMass << ",\"initial_height_m\":"
      << contract.levelCount * kTentHeight + kDropHeight
      << ",\"source_initially_kinematic\":true}"
      << ",\"contact\":{\"friction\":" << kFriction
      << ",\"gap_m\":" << kContactGap
      << "},\"schedule\":{\"display_time_step_seconds\":" << kDisplayTimeStep
      << ",\"substeps_per_frame\":" << kSubstepsPerFrame
      << ",\"runtime_time_step_seconds\":" << kRuntimeTimeStep
      << ",\"release_frame\":" << kReleaseFrame
      << ",\"release_substep\":" << kReleaseSubstep
      << ",\"total_frames\":" << kTotalFrames
      << ",\"total_substeps\":" << kTotalSubsteps
      << "},\"solver\":{\"type\":\"fbf_exact_coulomb\""
      << ",\"max_contacts\":" << kSourceMaxContacts
      << ",\"max_outer\":" << kSourceMaxOuterIterations
      << ",\"outer_tol\":" << kSourceOuterTolerance
      << ",\"residual_check_interval\":" << kSourceResidualCheckInterval
      << ",\"inner_solver\":";
  writeJsonString(out, kSourceInnerSolver);
  out << ",\"inner_gs_sweeps\":" << kSourceInnerGaussSeidelSweeps
      << ",\"inner_max_iter\":" << kSourceInnerMaxIterations
      << ",\"inner_tol\":" << kSourceInnerTolerance
      << ",\"gamma\":null,\"adaptive_gamma\":true,\"gamma_c\":" << kSourceGammaC
      << ",\"gamma_max\":" << kSourceGammaMax
      << ",\"armijo_rho_high\":" << kSourceArmijoRhoHigh
      << ",\"armijo_shrink\":" << kSourceArmijoShrink
      << ",\"armijo_max_backtracks\":" << kSourceArmijoMaxBacktracks
      << ",\"plateau_patience\":" << kSourcePlateauPatience
      << ",\"plateau_rtol\":" << kSourcePlateauRelativeTolerance
      << ",\"warm_start\":true,\"warm_start_match_radius\":"
      << kSourceWarmStartMatchRadius
      << ",\"warm_start_normal_cosine\":" << kSourceWarmStartNormalCosine
      << ",\"warm_start_max_age\":" << kSourceWarmStartMaxAge
      << ",\"warm_start_gamma_cap\":" << kSourceWarmStartGammaCap
      << ",\"baumgarte_erp\":" << kSourceBaumgarteErp
      << ",\"project_after_correction\":false"
      << ",\"termination_residual\":";
  writeJsonString(out, kSourceTerminationResidual);
  out << ",\"termination_tol\":" << kSourceTerminationTolerance
      << "}},\"dart_observation\":{\"world\":{\"time_step_seconds\":"
      << contract.timeStep << ",\"gravity_m_s2\":";
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
      << "},\"solver_adapter\":{\"type\":\"exact_fbf\""
      << ",\"split_impulse_enabled\":"
      << (contract.splitImpulseEnabled ? "true" : "false")
      << ",\"max_outer_iterations\":"
      << contract.solverOptions.maxOuterIterations
      << ",\"tolerance\":" << contract.solverOptions.tolerance
      << ",\"inner_max_sweeps\":" << contract.solverOptions.innerMaxSweeps
      << ",\"inner_tolerance\":" << contract.solverOptions.innerTolerance
      << ",\"step_size_scale\":" << contract.solverOptions.stepSizeScale
      << ",\"warm_start_enabled\":"
      << (contract.solverOptions.enableWarmStart ? "true" : "false")
      << ",\"fallback_to_boxed_lcp_enabled\":"
      << (contract.solverOptions.fallbackToBoxedLcp ? "true" : "false")
      << ",\"projected_gradient_retry_enabled\":"
      << (contract.solverOptions.enableProjectedGradientRetry ? "true"
                                                              : "false")
      << ",\"dense_residual_polish_enabled\":"
      << (contract.solverOptions.enableDenseResidualPolish ? "true" : "false")
      << "},\"ground\":{\"shape\":";
  writeJsonString(out, contract.groundShape);
  out << ",\"friction\":" << contract.groundFriction
      << ",\"mobile\":" << (contract.groundMobile ? "true" : "false")
      << "},\"cards\":";
  writeJsonBodies(out, contract.cards);
  out << ",\"cubes\":";
  writeJsonBodies(out, contract.cubes);
  out << "},\"adapter_boundaries\":{"
      << "\"source_contact_gap_recorded_m\":" << kContactGap
      << ",\"source_contact_gap_semantics_implemented\":false"
      << ",\"source_solver_backend_semantics_implemented\":false}"
      << ",\"claim_boundary\":{\"construction_only\":true"
      << ",\"configuration_port_valid\":true"
      << ",\"source_geometry_and_initial_pose_port\":true"
      << ",\"dart_cards_mobile\":true"
      << ",\"dart_cubes_initially_immobile\":true"
      << ",\"dynamics_executed\":false"
      << ",\"trajectory_valid\":false"
      << ",\"solver_valid\":false"
      << ",\"physical_outcome_valid\":false"
      << ",\"trajectory_equivalence\":"
      << (kTrajectoryEquivalence ? "true" : "false")
      << ",\"solver_equivalence\":" << (kSolverEquivalence ? "true" : "false")
      << ",\"physical_outcome_equivalence\":"
      << (kPhysicalOutcomeEquivalence ? "true" : "false")
      << ",\"fig06_parity\":" << (kFig06Parity ? "true" : "false")
      << ",\"video06_parity\":" << (kVideo06Parity ? "true" : "false")
      << ",\"timing_comparability\":"
      << (kTimingComparability ? "true" : "false")
      << ",\"paper_timing_valid\":false"
      << ",\"renderer_colors_source_parity\":false}"
      << ",\"visual_style\":{\"renderer_only\":true"
      << ",\"description\":\"restrained alternating paper colors for layer "
         "legibility\"}}";
  return out.str();
}

//==============================================================================
inline std::string dynamicsAdapterContractJson(
    const DynamicsAdapterContract& contract)
{
  if (contract.binarySourceSha256.empty())
    throw std::invalid_argument(
        "author card-house dynamics binary source hash is empty");
  if (contract.scenario == nullptr)
    throw std::invalid_argument(
        "author card-house dynamics scenario is missing");
  const auto& scenario = *contract.scenario;
  if (contract.levelCount != scenario.levelCount
      || contract.sourceContinuationScene != scenario.sourceContinuation) {
    throw std::invalid_argument(
        "author card-house dynamics scenario contract is inconsistent");
  }
  const std::size_t expectedDynamicShapeFrames
      = cardCount(scenario.levelCount) + kCubeCount;
  const std::size_t expectedCollisionShapeFrames
      = 1u + expectedDynamicShapeFrames;
  const std::size_t expectedGappedShapeFrames
      = scenario.sourceContactGapValuesRepresented
            ? expectedCollisionShapeFrames
            : 0u;
  const std::size_t expectedGappedGroundShapeFrames
      = scenario.sourceContactGapValuesRepresented ? 1u : 0u;
  const std::size_t expectedGappedDynamicShapeFrames
      = scenario.sourceContactGapValuesRepresented ? expectedDynamicShapeFrames
                                                   : 0u;
  const double expectedGroundContactGap
      = scenario.sourceContactGapValuesRepresented ? kSourceGroundContactGap
                                                   : 0.0;
  const double expectedDynamicShapeContactGap
      = scenario.sourceContactGapValuesRepresented ? kContactGap : 0.0;
  if (contract.collisionShapeFrameCount != expectedCollisionShapeFrames
      || contract.collisionShapeFramesWithContactGap
             != expectedGappedShapeFrames
      || contract.groundShapeFramesWithContactGap
             != expectedGappedGroundShapeFrames
      || contract.dynamicShapeFramesWithContactGap
             != expectedGappedDynamicShapeFrames
      || std::abs(contract.groundContactGap - expectedGroundContactGap) > 1e-15
      || std::abs(
             contract.dynamicShapeContactGap - expectedDynamicShapeContactGap)
             > 1e-15
      || contract.negativePenetrationDepthContactsAllowed
             != scenario.sourceContactGapValuesRepresented) {
    throw std::invalid_argument(
        "author card-house dynamics contact-gap contract is inconsistent");
  }
  if ((contract.solverLane == "exact_fbf") != contract.exactOptionsAvailable) {
    throw std::invalid_argument(
        "author card-house dynamics solver contract is inconsistent");
  }
  if (contract.solverLane != "exact_fbf"
      && contract.solverLane != "boxed_lcp") {
    throw std::invalid_argument(
        "author card-house dynamics solver lane is invalid");
  }
  if ((contract.solverLane == "boxed_lcp") != contract.boxedOptionsAvailable
      || contract.exactOptionsAvailable == contract.boxedOptionsAvailable) {
    throw std::invalid_argument(
        "author card-house dynamics baseline contract is inconsistent");
  }
  if (contract.exactOptionsAvailable
      && contract.exactSourceContinuationOptions.enabled
             != contract.sourceContinuationScene) {
    throw std::invalid_argument(
        "author card-house source-continuation request is inconsistent");
  }
  if (!contract.sourceContinuationScene
      && contract.exactSourceContinuationActive) {
    throw std::invalid_argument(
        "author card-house source continuation is unexpectedly active");
  }

  std::ostringstream out;
  out << std::setprecision(std::numeric_limits<double>::max_digits10);
  out << "{\"schema_version\":";
  writeJsonString(out, kDynamicsContractSchema);
  out << ",\"kind\":";
  writeJsonString(out, kDynamicsContractKind);
  out << ",\"source_binding\":{\"repository\":";
  writeJsonString(out, kAuthorRepository);
  out << ",\"commit\":";
  writeJsonString(out, kAuthorCommit);
  out << ",\"tree\":";
  writeJsonString(out, kAuthorTree);
  out << ",\"card_house_run_blob\":";
  writeJsonString(out, kAuthorCardHouseBlob);
  out << ",\"card_house_run_py_sha256\":";
  writeJsonString(out, kAuthorCardHouseRunSha256);
  out << ",\"fbf_config_py_sha256\":";
  writeJsonString(out, kAuthorConfigSha256);
  out << ",\"solver_fbf_py_sha256\":";
  writeJsonString(out, kAuthorSolverSha256);
  out << ",\"configuration_spec_sha256\":";
  writeJsonString(out, kSpecSourceSha256);
  out << ",\"demo_implementation_sha256\":";
  writeJsonString(out, contract.binarySourceSha256);
  out << "},\"source_defaults\":{\"levels\":" << kDefaultLevelCount
      << ",\"frames\":" << kTotalFrames << ",\"drop_frame\":" << kReleaseFrame
      << ",\"num_cubes\":" << kCubeCount << ",\"mu\":" << kFriction
      << ",\"cube_half_size_m\":" << kCubeHalfSize
      << ",\"cube_density_kg_m3\":" << kCubeDensity
      << ",\"drop_height_m\":" << kDropHeight
      << "},\"selected_source_invocation\":{"
         "\"provenance\":\"source_supported_cli_parameterization\""
         ",\"historical_paper_invocation_known\":false"
         ",\"arguments\":{\"solvers\":[\"fbf\"],\"levels\":"
      << scenario.levelCount << ",\"frames\":" << scenario.selectedFrames
      << ",\"drop_frame\":" << kReleaseFrame << ",\"num_cubes\":" << kCubeCount
      << ",\"mu\":" << kFriction << ",\"cube_half_size_m\":" << kCubeHalfSize
      << ",\"cube_density_kg_m3\":" << kCubeDensity
      << ",\"drop_height_m\":" << kDropHeight
      << ",\"device\":\"cpu\",\"profile\":true,\"usd\":true}}"
         ",\"source_configuration\":{\"cards\":{\"count\":"
      << cardCount(contract.levelCount)
      << ",\"leaning_count\":" << leaningCardCount(contract.levelCount)
      << ",\"bridge_count\":" << bridgeCardCount(contract.levelCount)
      << ",\"lean_size_m\":[" << 2.0 * kCardHalfThickness << ','
      << 2.0 * kCardHalfDepth << ',' << 2.0 * kCardHalfLength
      << "],\"bridge_size_m\":[" << 2.0 * kCardHalfLength << ','
      << 2.0 * kCardHalfDepth << ',' << 2.0 * kCardHalfThickness
      << "],\"density_kg_m3\":" << kCardDensity << ",\"mass_kg\":" << kCardMass
      << ",\"lean_from_vertical_degrees\":" << kLeanFromVerticalDegrees
      << ",\"bridge_angle_degrees\":" << kBridgeAngleDegrees
      << ",\"tent_half_gap_m\":" << kTentHalfGap
      << ",\"tent_width_m\":" << kTentWidth
      << ",\"tent_height_m\":" << kTentHeight
      << "},\"cubes\":{\"count\":" << kCubeCount
      << ",\"edge_m\":" << 2.0 * kCubeHalfSize
      << ",\"density_kg_m3\":" << kCubeDensity << ",\"mass_kg\":" << kCubeMass
      << ",\"initial_height_m\":"
      << contract.levelCount * kTentHeight + kDropHeight
      << ",\"initially_kinematic\":true,\"initial_velocity_m_s\":[0,0,0]}"
         ",\"contact\":{\"friction\":"
      << kFriction;
  if (scenario.sourceContactGapValuesRepresented) {
    out << ",\"dynamic_shape_contact\":{\"gap_m\":" << kContactGap
        << ",\"shape_stiffness\":" << kSourceShapeStiffness
        << ",\"shape_damping\":" << kSourceShapeDamping
        << "},\"ground_contact\":{\"gap_m\":" << kSourceGroundContactGap
        << ",\"shape_stiffness\":" << kSourceGroundShapeStiffness
        << ",\"shape_damping\":" << kSourceGroundShapeDamping << '}';
  } else {
    out << ",\"gap_m\":" << kContactGap
        << ",\"shape_stiffness\":" << kSourceShapeStiffness
        << ",\"shape_damping\":" << kSourceShapeDamping;
  }
  out << "},\"schedule\":{\"display_time_step_seconds\":" << kDisplayTimeStep
      << ",\"substeps_per_frame\":" << kSubstepsPerFrame
      << ",\"runtime_time_step_seconds\":" << kRuntimeTimeStep
      << ",\"release_frame\":" << kReleaseFrame
      << ",\"release_substep\":" << kReleaseSubstep
      << ",\"total_frames\":" << scenario.selectedFrames
      << ",\"total_substeps\":" << scenario.selectedSubsteps()
      << "},\"solver\":{\"type\":\"fbf_exact_coulomb\""
         ",\"max_contacts\":"
      << kSourceMaxContacts << ",\"max_outer\":" << kSourceMaxOuterIterations
      << ",\"outer_tol\":" << kSourceOuterTolerance
      << ",\"residual_check_interval\":" << kSourceResidualCheckInterval
      << ",\"inner_solver\":";
  writeJsonString(out, kSourceInnerSolver);
  out << ",\"inner_gs_sweeps\":" << kSourceInnerGaussSeidelSweeps
      << ",\"inner_max_iter\":" << kSourceInnerMaxIterations
      << ",\"inner_tol\":" << kSourceInnerTolerance
      << ",\"adaptive_gamma\":true,\"gamma_c\":" << kSourceGammaC
      << ",\"gamma_max\":" << kSourceGammaMax
      << ",\"armijo_rho_high\":" << kSourceArmijoRhoHigh
      << ",\"armijo_shrink\":" << kSourceArmijoShrink
      << ",\"armijo_max_backtracks\":" << kSourceArmijoMaxBacktracks
      << ",\"warm_start\":true,\"project_after_correction\":false"
      << ",\"restart_inner_from_current_outer_reaction\":true"
         ",\"project_inner_initial_reaction\":false"
      << ",\"baumgarte_erp\":" << kSourceBaumgarteErp
      << ",\"termination_residual\":";
  writeJsonString(out, kSourceTerminationResidual);
  out << ",\"termination_tol\":" << kSourceTerminationTolerance
      << "}},\"dart_adapter\":{\"scene_id\":";
  writeJsonString(out, scenario.demoSceneId);
  out << ",\"world\":{\"time_step_seconds\":" << contract.timeStep
      << ",\"current_time_seconds\":" << contract.worldTime
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
      << ",\"enable_contact\":"
      << (contract.contactGenerationEnabled ? "true" : "false")
      << ",\"allow_negative_penetration_depth_contacts\":"
      << (contract.negativePenetrationDepthContactsAllowed ? "true" : "false")
      << ",\"default_empty_body_node_filter\":"
      << (contract.defaultEmptyBodyNodeCollisionFilter ? "true" : "false");
  if (scenario.sourceContactGapValuesRepresented) {
    out << ",\"ground_contact_gap_m\":" << contract.groundContactGap
        << ",\"dynamic_shape_contact_gap_m\":"
        << contract.dynamicShapeContactGap
        << ",\"collision_shape_frames\":" << contract.collisionShapeFrameCount
        << ",\"collision_shape_frames_with_contact_gap\":"
        << contract.collisionShapeFramesWithContactGap
        << ",\"ground_shape_frames_with_contact_gap\":"
        << contract.groundShapeFramesWithContactGap
        << ",\"dynamic_shape_frames_with_contact_gap\":"
        << contract.dynamicShapeFramesWithContactGap
        << ",\"speculative_contact_velocity_allowance\":";
    writeJsonString(out, "physical_separation_over_time_step");
  }
  out << "},\"contact_material\":{\"primary_friction\":"
      << contract.primaryFriction
      << ",\"secondary_friction\":" << contract.secondaryFriction
      << ",\"restitution\":" << contract.restitution
      << ",\"primary_slip_compliance\":" << contract.primarySlipCompliance
      << ",\"secondary_slip_compliance\":" << contract.secondarySlipCompliance
      << ",\"first_friction_direction\":";
  writeJsonVector(out, contract.firstFrictionDirection);
  out << ",\"uses_default_friction_direction_frame\":"
      << (contract.usesDefaultFrictionDirectionFrame ? "true" : "false")
      << "},\"process_state\":{\"observed_contact_erp\":"
      << contract.observedContactErrorReductionParameter
      << "},\"solver\":{\"lane\":";
  writeJsonString(out, contract.solverLane);
  out << ",\"split_impulse_enabled\":"
      << (contract.splitImpulseEnabled ? "true" : "false")
      << ",\"source_inner_initialization_requested\":"
      << (contract.exactSourceInnerInitializationRequested ? "true" : "false")
      << ",\"source_inner_initialization_active\":"
      << (contract.exactSourceInnerInitializationActive ? "true" : "false")
      << ",\"colored_block_gauss_seidel_enabled\":"
      << (contract.exactColoredBlockGaussSeidelEnabled ? "true" : "false")
      << ",\"participant_affinity_enabled\":"
      << (contract.exactParticipantAffinityEnabled ? "true" : "false");
  if (contract.sourceContinuationScene) {
    const auto& options = contract.exactSourceContinuationOptions;
    out << ",\"source_continuation\":{\"policy\":";
    writeJsonString(out, kSourceContinuationPolicy);
    out << ",\"options_available\":"
        << (contract.exactOptionsAvailable ? "true" : "false")
        << ",\"requested\":"
        << (contract.exactOptionsAvailable && options.enabled ? "true"
                                                              : "false")
        << ",\"last_active\":"
        << (contract.exactSourceContinuationActive ? "true" : "false")
        << ",\"numeric_settings\":{\"residual_check_interval\":"
        << (contract.exactOptionsAvailable ? options.residualCheckInterval
                                           : kSourceResidualCheckInterval)
        << ",\"plateau_patience\":"
        << (contract.exactOptionsAvailable ? options.plateauPatience
                                           : kSourcePlateauPatience)
        << ",\"plateau_relative_tolerance\":"
        << (contract.exactOptionsAvailable ? options.plateauRelativeTolerance
                                           : kSourcePlateauRelativeTolerance)
        << ",\"step_size_backtrack_limit\":"
        << (contract.exactOptionsAvailable ? options.stepSizeBacktrackLimit
                                           : kSourceArmijoMaxBacktracks)
        << ",\"coupling_variation_skip_threshold\":"
        << (contract.exactOptionsAvailable
                ? options.couplingVariationSkipThreshold
                : kSourceCouplingVariationSkipThreshold)
        << "},\"fixed_semantics\":{\"strict_convergence_comparison\":";
    writeJsonString(out, kSourceStrictConvergenceComparison);
    out << ",\"iteration_zero_residual\":";
    writeJsonString(out, kSourceInitialConvergenceResidual);
    out << ",\"sampled_termination_residual\":";
    writeJsonString(out, kSourceSampledTerminationResidual);
    out << ",\"plateau_metric\":";
    writeJsonString(out, kSourcePlateauMetric);
    out << ",\"small_change_armijo_action\":";
    writeJsonString(out, kSourceSmallChangeArmijoAction);
    out << ",\"line_search_cap_action\":";
    writeJsonString(out, kSourceShrinkCapAction);
    out << "}}";
  }
  out << ",\"exact_options\":";
  if (!contract.exactOptionsAvailable) {
    out << "null";
  } else {
    const auto& options = contract.exactOptions;
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
        << ",\"project_after_correction\":"
        << (contract.exactPostCorrectionProjectionEnabled ? "true" : "false")
        << ",\"restart_inner_from_current_outer_reaction\":"
        << (contract.exactSourceInnerInitializationActive ? "true" : "false")
        << ",\"project_inner_initial_reaction\":"
        << (contract.exactSourceInnerInitializationActive ? "false" : "true")
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
  if (!contract.exactOptionsAvailable) {
    out << "null";
  } else {
    const auto& options = contract.exactCrossStepOptions;
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
  out << ",\"boxed_baseline\":";
  if (!contract.boxedOptionsAvailable) {
    out << "null";
  } else {
    const auto& options = contract.boxedMatrixFreeOptions;
    out << "{\"primary_solver\":";
    writeJsonString(out, contract.boxedPrimarySolver);
    out << ",\"secondary_solver\":";
    writeJsonString(out, contract.boxedSecondarySolver);
    out << ",\"matrix_free_options\":{\"enabled\":"
        << (options.mEnabled ? "true" : "false")
        << ",\"min_rows\":" << options.mMinRows
        << ",\"max_iterations\":" << options.mMaxIterations
        << ",\"sor\":" << options.mSor
        << ",\"delta_tolerance\":" << options.mDeltaTolerance
        << ",\"relative_delta_tolerance\":" << options.mRelativeDeltaTolerance
        << ",\"epsilon_for_division\":" << options.mEpsilonForDivision << "}}";
  }
  out << "},\"inventory\":{\"cards\":" << contract.cardCount
      << ",\"cubes\":" << contract.cubeCount
      << ",\"released_cubes\":" << contract.releasedCubeCount
      << ",\"finite_state\":" << (contract.finiteState ? "true" : "false")
      << "},\"schedule\":{\"evidence_total_substeps\":"
      << scenario.selectedSubsteps()
      << ",\"evidence_runner_action_completed_step\":" << kReleaseSubstep
      << ",\"release_action_key\":";
  writeJsonString(out, std::string(1u, kReleaseActionKey));
  out << ",\"interactive_action_semantics\":";
  writeJsonString(out, "immediate_on_invocation");
  out << "}},\"adapter_boundaries\":{";
  if (scenario.sourceContactGapValuesRepresented) {
    out << "\"source_dynamic_shape_contact_gap_recorded_m\":" << kContactGap
        << ",\"source_ground_contact_gap_recorded_m\":"
        << kSourceGroundContactGap
        << ",\"source_contact_gap_semantics_implemented\":false"
        << ",\"source_contact_gap_values_represented\":true"
        << ",\"source_separation_over_dt_term_represented\":true";
  } else {
    out << "\"source_contact_gap_recorded_m\":" << kContactGap
        << ",\"source_contact_gap_semantics_implemented\":false";
  }
  out << ",\"source_shape_stiffness_semantics_implemented\":false"
         ",\"source_shape_damping_semantics_implemented\":false"
         ",\"source_collision_backend_implemented\":false"
         ",\"source_solver_backend_semantics_implemented\":false"
         ",\"source_float32_semantics_implemented\":false"
         ",\"dart_native_four_point_planar_is_adapter_choice\":true}"
         ",\"claim_boundary\":{"
         "\"current_source_parameterized_configuration_port\":true"
         ",\"source_release_action_ported_to_dart\":true"
         ",\"source_release_schedule_declared_for_evidence_runner\":true"
         ",\"interactive_demo_auto_releases_at_source_step\":false"
         ",\"historical_paper_invocation_known\":false";
  if (scenario.sourceContactGapValuesRepresented)
    out << ",\"historical_tables_6_7_invocation_known\":false";
  out << ",\"trajectory_valid\":false"
         ",\"physical_outcome_valid\":false"
         ",\"trajectory_equivalence\":false"
         ",\"solver_equivalence\":false"
         ",\"physical_outcome_equivalence\":false"
         ",\"fig06_parity\":false"
         ",\"video06_parity\":false"
         ",\"timing_comparability\":false"
         ",\"paper_parity\":false}}";
  return out.str();
}

} // namespace fbf_author_card_house

#endif // DART_EXAMPLES_DEMOS_SCENES_FBFAUTHORCARDHOUSESPEC_HPP_
