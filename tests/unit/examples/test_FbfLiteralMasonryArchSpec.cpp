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

#include "../../../examples/demos/scenes/FbfLiteralMasonryArchSpec.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <string>
#include <type_traits>

#include <cmath>
#include <cstddef>

namespace {

using namespace fbf_literal_masonry_arch;

//==============================================================================
void expectExactOptions(
    const dart::constraint::ExactCoulombFbfConstraintSolverOptions& options)
{
  EXPECT_FALSE(options.fallbackToBoxedLcp);
  EXPECT_FALSE(options.includeConstraintRegularization);
  EXPECT_FALSE(options.useMatrixFreeDelassusOperator);
  EXPECT_TRUE(options.useContactRowDelassusOperator);
  EXPECT_FALSE(options.assembleDenseContactRowSnapshot);
  EXPECT_TRUE(options.enableWarmStart);
  EXPECT_FALSE(options.enableStepSizePersistence);
  EXPECT_DOUBLE_EQ(options.stepSizeRecoveryGrowthFactor, 1.05);
  EXPECT_DOUBLE_EQ(options.warmStartMatchDistance, 0.025);
  EXPECT_FALSE(options.seedNormalImpulseFromDiagonal);
  EXPECT_FALSE(options.useMatrixFreeDelassusSeed);
  EXPECT_FALSE(options.enableProjectedGradientRetry);
  EXPECT_FALSE(options.enableDenseResidualPolish);
  EXPECT_EQ(options.maxOuterIterations, 5000);
  EXPECT_TRUE(options.acceptOuterMaxIterations);
  EXPECT_DOUBLE_EQ(options.tolerance, 1e-6);
  EXPECT_TRUE(std::isnan(options.initialStepSize));
  EXPECT_TRUE(options.capInitialStepSizeAtSafeBound);
  EXPECT_DOUBLE_EQ(options.stepSizeScale, 35.0);
  EXPECT_DOUBLE_EQ(options.outerRelaxation, 1.1);
  EXPECT_DOUBLE_EQ(options.couplingVariationTolerance, 0.9);
  EXPECT_DOUBLE_EQ(options.shrinkFactor, 0.7);
  EXPECT_EQ(options.maxStepShrinkIterations, 20);
  EXPECT_TRUE(options.enableAdaptiveStepSize);
  EXPECT_EQ(options.spectralIterations, 10);
  EXPECT_EQ(options.innerMaxSweeps, 30);
  EXPECT_EQ(
      options.innerLocalSolver,
      dart::constraint::ExactCoulombFbfLocalBlockSolver::ExactMetricProjection);
  EXPECT_TRUE(options.runFixedInnerSweeps);
  EXPECT_TRUE(options.acceptInnerMaxIterations);
  EXPECT_EQ(options.innerLocalIterations, 1);
  EXPECT_DOUBLE_EQ(options.innerTolerance, 1e-10);
  EXPECT_DOUBLE_EQ(options.innerLocalTolerance, 1e-12);
  EXPECT_DOUBLE_EQ(options.innerDiagonalRegularization, 0.0);
  EXPECT_EQ(options.projectedGradientMaxIterations, 400);
  EXPECT_DOUBLE_EQ(options.projectedGradientTolerance, 1e-12);
  EXPECT_EQ(options.denseResidualPolishIterations, 8);
  EXPECT_EQ(options.denseResidualPolishLineSearchIterations, 8);
  EXPECT_DOUBLE_EQ(options.denseResidualPolishRegularization, 1e-9);
  EXPECT_EQ(options.maxResidualHistorySamples, 0);
  EXPECT_EQ(options.maxResidualHistoryRecords, 0);
}

//==============================================================================
void expectExactCrossStepOptions(
    const dart::constraint::ExactCoulombFbfCrossStepPolicyOptions& options)
{
  EXPECT_EQ(
      options.warmStartMatchMode,
      dart::constraint::ExactCoulombFbfWarmStartMatchMode::
          EitherBodyLocalFeature);
  EXPECT_DOUBLE_EQ(options.warmStartNormalCosine, 0.9);
  EXPECT_FALSE(options.useStrictWarmStartMatchDistance);
  EXPECT_EQ(options.warmStartMaxAge, -1);
  EXPECT_DOUBLE_EQ(options.persistentStepSizeSafeBoundScale, 1.0);
  EXPECT_TRUE(std::isnan(options.minimumStepSize));
  EXPECT_TRUE(std::isnan(options.maximumStepSize));
  EXPECT_TRUE(std::isnan(options.warmStartResidualThreshold));
  EXPECT_TRUE(std::isnan(options.warmStartStepSizeCap));
  EXPECT_FALSE(options.persistUncappedStepSizeOnWarmStartCap);
  EXPECT_FALSE(options.requireResidualImprovementForUnconvergedCacheSave);
}

//==============================================================================
void expectSameNonSolverPhysics(
    const PhysicsContract& exact, const PhysicsContract& boxed)
{
  EXPECT_EQ(exact.visualMode, boxed.visualMode);
  EXPECT_EQ(exact.worldName, boxed.worldName);
  EXPECT_DOUBLE_EQ(exact.timeStep, boxed.timeStep);
  EXPECT_TRUE(exact.gravity.isApprox(boxed.gravity, 0.0));
  EXPECT_EQ(exact.simulationThreads, boxed.simulationThreads);
  EXPECT_EQ(exact.deactivationEnabled, boxed.deactivationEnabled);
  EXPECT_EQ(exact.worldSkeletonCount, boxed.worldSkeletonCount);
  EXPECT_EQ(exact.stoneCount, boxed.stoneCount);
  EXPECT_EQ(exact.mobileStoneCount, boxed.mobileStoneCount);
  EXPECT_EQ(exact.pinnedStoneCount, boxed.pinnedStoneCount);
  EXPECT_DOUBLE_EQ(exact.declaredSpec.density, boxed.declaredSpec.density);
  EXPECT_DOUBLE_EQ(exact.declaredSpec.friction, boxed.declaredSpec.friction);
  EXPECT_EQ(
      exact.declaredSpec.barrierGapPolicy, boxed.declaredSpec.barrierGapPolicy);
  EXPECT_DOUBLE_EQ(
      exact.declaredSpec.endFaceExpansion, boxed.declaredSpec.endFaceExpansion);
  EXPECT_DOUBLE_EQ(
      exact.declaredSpec.downwardShift, boxed.declaredSpec.downwardShift);
  EXPECT_DOUBLE_EQ(
      exact.declaredSpec.contactErrorReductionParameter,
      boxed.declaredSpec.contactErrorReductionParameter);
  EXPECT_EQ(exact.groundMobile, boxed.groundMobile);
  EXPECT_EQ(exact.groundIsPlane, boxed.groundIsPlane);
  EXPECT_DOUBLE_EQ(exact.groundFriction, boxed.groundFriction);
  EXPECT_DOUBLE_EQ(exact.groundPrimaryFriction, boxed.groundPrimaryFriction);
  EXPECT_DOUBLE_EQ(
      exact.groundSecondaryFriction, boxed.groundSecondaryFriction);
  EXPECT_DOUBLE_EQ(exact.groundRestitution, boxed.groundRestitution);
  EXPECT_DOUBLE_EQ(
      exact.groundPrimarySlipCompliance, boxed.groundPrimarySlipCompliance);
  EXPECT_DOUBLE_EQ(
      exact.groundSecondarySlipCompliance, boxed.groundSecondarySlipCompliance);
  EXPECT_TRUE(exact.groundFirstFrictionDirection.isApprox(
      boxed.groundFirstFrictionDirection, 0.0));
  EXPECT_EQ(
      exact.groundUsesDefaultFrictionDirectionFrame,
      boxed.groundUsesDefaultFrictionDirectionFrame);
  EXPECT_TRUE(exact.groundLocalCenterOfMass.isApprox(
      boxed.groundLocalCenterOfMass, 0.0));
  EXPECT_TRUE(exact.groundBodyTransform.matrix().isApprox(
      boxed.groundBodyTransform.matrix(), 0.0));
  EXPECT_TRUE(
      exact.groundLinearVelocity.isApprox(boxed.groundLinearVelocity, 0.0));
  EXPECT_TRUE(
      exact.groundAngularVelocity.isApprox(boxed.groundAngularVelocity, 0.0));
  EXPECT_EQ(exact.groundCollisionShapeCount, boxed.groundCollisionShapeCount);
  EXPECT_TRUE(exact.groundCollisionShapeRelativeTransform.matrix().isApprox(
      boxed.groundCollisionShapeRelativeTransform.matrix(), 0.0));
  EXPECT_TRUE(exact.groundPlaneNormal.isApprox(boxed.groundPlaneNormal, 0.0));
  EXPECT_DOUBLE_EQ(exact.groundPlaneOffset, boxed.groundPlaneOffset);
  EXPECT_EQ(exact.groundHasVisualFloor, boxed.groundHasVisualFloor);
  EXPECT_TRUE(exact.visualFloorSize.isApprox(boxed.visualFloorSize, 0.0));
  EXPECT_TRUE(
      exact.visualFloorTranslation.isApprox(boxed.visualFloorTranslation, 0.0));
  EXPECT_EQ(exact.maxContacts, boxed.maxContacts);
  EXPECT_EQ(exact.maxContactsPerPair, boxed.maxContactsPerPair);
  EXPECT_EQ(exact.nativeCollision, boxed.nativeCollision);
  EXPECT_EQ(exact.manifoldMode, boxed.manifoldMode);
  EXPECT_EQ(exact.splitImpulseEnabled, boxed.splitImpulseEnabled);
  ASSERT_EQ(exact.stones.size(), boxed.stones.size());
  for (std::size_t index = 0u; index < exact.stones.size(); ++index) {
    const auto& exactStone = exact.stones[index];
    const auto& boxedStone = boxed.stones[index];
    EXPECT_EQ(exactStone.name, boxedStone.name);
    EXPECT_EQ(exactStone.mobile, boxedStone.mobile);
    EXPECT_DOUBLE_EQ(exactStone.friction, boxedStone.friction);
    EXPECT_DOUBLE_EQ(exactStone.primaryFriction, boxedStone.primaryFriction);
    EXPECT_DOUBLE_EQ(
        exactStone.secondaryFriction, boxedStone.secondaryFriction);
    EXPECT_DOUBLE_EQ(exactStone.restitution, boxedStone.restitution);
    EXPECT_DOUBLE_EQ(
        exactStone.primarySlipCompliance, boxedStone.primarySlipCompliance);
    EXPECT_DOUBLE_EQ(
        exactStone.secondarySlipCompliance, boxedStone.secondarySlipCompliance);
    EXPECT_TRUE(exactStone.firstFrictionDirection.isApprox(
        boxedStone.firstFrictionDirection, 0.0));
    EXPECT_EQ(
        exactStone.usesDefaultFrictionDirectionFrame,
        boxedStone.usesDefaultFrictionDirectionFrame);
    EXPECT_DOUBLE_EQ(exactStone.mass, boxedStone.mass);
    EXPECT_TRUE(exactStone.localCenterOfMass.isApprox(
        boxedStone.localCenterOfMass, 0.0));
    EXPECT_TRUE(exactStone.moment.isApprox(boxedStone.moment, 0.0));
    EXPECT_TRUE(exactStone.transform.matrix().isApprox(
        boxedStone.transform.matrix(), 0.0));
    EXPECT_TRUE(
        exactStone.linearVelocity.isApprox(boxedStone.linearVelocity, 0.0));
    EXPECT_TRUE(
        exactStone.angularVelocity.isApprox(boxedStone.angularVelocity, 0.0));
    EXPECT_EQ(exactStone.collisionShapeCount, boxedStone.collisionShapeCount);
    EXPECT_TRUE(exactStone.collisionShapeRelativeTransform.matrix().isApprox(
        boxedStone.collisionShapeRelativeTransform.matrix(), 0.0));
    ASSERT_EQ(
        exactStone.collisionMeshVertices.size(),
        boxedStone.collisionMeshVertices.size());
    for (std::size_t vertex = 0u;
         vertex < exactStone.collisionMeshVertices.size();
         ++vertex) {
      EXPECT_TRUE(exactStone.collisionMeshVertices[vertex].isApprox(
          boxedStone.collisionMeshVertices[vertex], 0.0));
    }
    ASSERT_EQ(
        exactStone.collisionMeshTriangles.size(),
        boxedStone.collisionMeshTriangles.size());
    for (std::size_t triangle = 0u;
         triangle < exactStone.collisionMeshTriangles.size();
         ++triangle) {
      EXPECT_TRUE((exactStone.collisionMeshTriangles[triangle].array()
                   == boxedStone.collisionMeshTriangles[triangle].array())
                      .all());
    }
    EXPECT_EQ(exactStone.hasVisualAspect, boxedStone.hasVisualAspect);
    EXPECT_TRUE(exactStone.color.isApprox(boxedStone.color, 0.0));
  }
}

//==============================================================================
TEST(FbfLiteralMasonryArchSpec, ConstantsAndExactOptionsArePinned)
{
  EXPECT_EQ(kStoneCount, 25u);
  EXPECT_EQ(kMobileStoneCount, 23u);
  EXPECT_EQ(kPinnedSpringerCount, 2u);
  EXPECT_DOUBLE_EQ(kTimeStep, 1.0 / 60.0);
  EXPECT_DOUBLE_EQ(kGravity, 9.81);
  EXPECT_DOUBLE_EQ(kDensity, 1000.0);
  EXPECT_DOUBLE_EQ(kFriction, 0.8);
  EXPECT_EQ(
      kBarrierGapPolicy,
      dart::math::detail::MasonryArchBarrierGapPolicy::OmitSourceOffsets);
  EXPECT_DOUBLE_EQ(kEndFaceExpansion, 1e-6);
  EXPECT_DOUBLE_EQ(kDownwardShift, 0.001001);
  EXPECT_EQ(kMaxContacts, 400u);
  EXPECT_EQ(kMaxContactsPerPair, 8u);
  EXPECT_DOUBLE_EQ(kDesiredContactErrorReductionParameter, 0.0);
  EXPECT_STREQ(kExpectedPhysicalGeometryFingerprint, "1ff65f2a99ec96d1");
  expectExactOptions(makeExactOptions());
  expectExactCrossStepOptions(makeExactCrossStepPolicyOptions());
}

//==============================================================================
TEST(FbfLiteralMasonryArchSpec, ScopedContactErpRestoresProcessValue)
{
  static_assert(
      !std::is_copy_constructible<ScopedContactErrorReductionParameter>::value);
  static_assert(
      !std::is_copy_assignable<ScopedContactErrorReductionParameter>::value);

  const double processValue
      = dart::constraint::ContactConstraint::getErrorReductionParameter();
  constexpr double kSentinelErp = 0.37;
  dart::constraint::ContactConstraint::setErrorReductionParameter(kSentinelErp);
  {
    ScopedContactErrorReductionParameter scopedErp;
    EXPECT_DOUBLE_EQ(
        dart::constraint::ContactConstraint::getErrorReductionParameter(),
        kDesiredContactErrorReductionParameter);
  }
  EXPECT_DOUBLE_EQ(
      dart::constraint::ContactConstraint::getErrorReductionParameter(),
      kSentinelErp);
  dart::constraint::ContactConstraint::setErrorReductionParameter(processValue);
}

//==============================================================================
TEST(FbfLiteralMasonryArchSpec, ExactWorldUsesLiteralConstruction)
{
  const double initialErp
      = dart::constraint::ContactConstraint::getErrorReductionParameter();
  const auto world = createWorld(SolverLane::ExactFbf, VisualMode::None, 1u);
  EXPECT_DOUBLE_EQ(
      dart::constraint::ContactConstraint::getErrorReductionParameter(),
      initialErp);

  const PhysicsContract contract = inspectPhysicsContract(world);
  EXPECT_EQ(contract.solverLane, SolverLane::ExactFbf);
  EXPECT_EQ(contract.visualMode, VisualMode::None);
  EXPECT_EQ(contract.worldName, kWorldName);
  EXPECT_DOUBLE_EQ(contract.timeStep, kTimeStep);
  EXPECT_TRUE(
      contract.gravity.isApprox(Eigen::Vector3d(0.0, 0.0, -kGravity), 0.0));
  EXPECT_EQ(contract.simulationThreads, 1u);
  EXPECT_FALSE(contract.deactivationEnabled);
  EXPECT_EQ(contract.worldSkeletonCount, kStoneCount + 1u);
  EXPECT_EQ(contract.stoneCount, kStoneCount);
  EXPECT_EQ(contract.mobileStoneCount, kMobileStoneCount);
  EXPECT_EQ(contract.pinnedStoneCount, kPinnedSpringerCount);
  EXPECT_DOUBLE_EQ(contract.declaredSpec.density, kDensity);
  EXPECT_DOUBLE_EQ(contract.declaredSpec.friction, kFriction);
  EXPECT_EQ(contract.declaredSpec.barrierGapPolicy, kBarrierGapPolicy);
  EXPECT_DOUBLE_EQ(contract.declaredSpec.endFaceExpansion, kEndFaceExpansion);
  EXPECT_DOUBLE_EQ(contract.declaredSpec.downwardShift, kDownwardShift);
  EXPECT_DOUBLE_EQ(
      contract.declaredSpec.contactErrorReductionParameter,
      kDesiredContactErrorReductionParameter);
  EXPECT_FALSE(contract.groundMobile);
  EXPECT_TRUE(contract.groundIsPlane);
  EXPECT_DOUBLE_EQ(contract.groundFriction, kFriction);
  EXPECT_DOUBLE_EQ(contract.groundPrimaryFriction, kFriction);
  EXPECT_DOUBLE_EQ(contract.groundSecondaryFriction, kFriction);
  EXPECT_DOUBLE_EQ(contract.groundRestitution, 0.0);
  EXPECT_DOUBLE_EQ(contract.groundPrimarySlipCompliance, -1.0);
  EXPECT_DOUBLE_EQ(contract.groundSecondarySlipCompliance, -1.0);
  EXPECT_TRUE(contract.groundFirstFrictionDirection.isZero(0.0));
  EXPECT_TRUE(contract.groundUsesDefaultFrictionDirectionFrame);
  EXPECT_TRUE(contract.groundLocalCenterOfMass.isZero(0.0));
  EXPECT_TRUE(contract.groundBodyTransform.matrix().isIdentity(0.0));
  EXPECT_TRUE(contract.groundLinearVelocity.isZero(0.0));
  EXPECT_TRUE(contract.groundAngularVelocity.isZero(0.0));
  EXPECT_EQ(contract.groundCollisionShapeCount, 1u);
  EXPECT_TRUE(
      contract.groundCollisionShapeRelativeTransform.matrix().isIdentity(0.0));
  EXPECT_TRUE(
      contract.groundPlaneNormal.isApprox(Eigen::Vector3d::UnitZ(), 0.0));
  EXPECT_DOUBLE_EQ(contract.groundPlaneOffset, 0.0);
  EXPECT_FALSE(contract.groundHasVisualFloor);
  EXPECT_EQ(contract.maxContacts, kMaxContacts);
  EXPECT_EQ(contract.maxContactsPerPair, kMaxContactsPerPair);
  EXPECT_TRUE(contract.nativeCollision);
  EXPECT_EQ(
      contract.manifoldMode,
      dart::collision::NativeCollisionDetector::ContactManifoldMode::
          FourPointPlanar);
  EXPECT_TRUE(contract.splitImpulseEnabled);
  EXPECT_TRUE(contract.exactColoredBlockGaussSeidelEnabled);
  EXPECT_TRUE(contract.exactParticipantAffinityEnabled);
  ASSERT_TRUE(contract.exactOptions.has_value());
  expectExactOptions(*contract.exactOptions);
  ASSERT_TRUE(contract.exactCrossStepOptions.has_value());
  expectExactCrossStepOptions(*contract.exactCrossStepOptions);
  EXPECT_DOUBLE_EQ(
      contract.observedProcessContactErrorReductionParameter, initialErp);
  EXPECT_EQ(world->getNumSkeletons(), kStoneCount + 1u);
  const auto ground = world->getSkeleton("ground_plane");
  ASSERT_NE(ground, nullptr);
  const auto* groundShapeNode
      = ground->getBodyNode(0u)
            ->getShapeNodeWith<dart::dynamics::CollisionAspect>(0u);
  ASSERT_NE(groundShapeNode, nullptr);
  const auto groundPlane
      = std::dynamic_pointer_cast<const dart::dynamics::PlaneShape>(
          groundShapeNode->getShape());
  ASSERT_NE(groundPlane, nullptr);
  EXPECT_TRUE(groundPlane->getNormal().isApprox(Eigen::Vector3d::UnitZ(), 0.0));
  EXPECT_DOUBLE_EQ(groundPlane->getOffset(), 0.0);

  const auto geometries = makeStoneGeometries();
  ASSERT_EQ(contract.stones.size(), geometries.size());
  const auto& triangles
      = dart::math::detail::getMasonryArchStoneWedgeTriangles();
  for (std::size_t index = 0u; index < kStoneCount; ++index) {
    const auto skeleton
        = world->getSkeleton("masonry_arch_stone_" + std::to_string(index));
    ASSERT_NE(skeleton, nullptr);
    EXPECT_EQ(skeleton->isMobile(), index != 0u && index + 1u != kStoneCount);
    ASSERT_EQ(skeleton->getNumBodyNodes(), 1u);
    const auto* body = skeleton->getBodyNode(0u);
    const auto* shapeNode
        = body->getShapeNodeWith<dart::dynamics::CollisionAspect>(0u);
    ASSERT_NE(shapeNode, nullptr);
    EXPECT_EQ(body->getNumShapeNodesWith<dart::dynamics::VisualAspect>(), 0u);
    const auto& stoneContract = contract.stones[index];
    EXPECT_EQ(stoneContract.collisionShapeCount, 1u);
    EXPECT_DOUBLE_EQ(stoneContract.primaryFriction, kFriction);
    EXPECT_DOUBLE_EQ(stoneContract.secondaryFriction, kFriction);
    EXPECT_DOUBLE_EQ(stoneContract.restitution, 0.0);
    EXPECT_DOUBLE_EQ(stoneContract.primarySlipCompliance, -1.0);
    EXPECT_DOUBLE_EQ(stoneContract.secondarySlipCompliance, -1.0);
    EXPECT_TRUE(stoneContract.firstFrictionDirection.isZero(0.0));
    EXPECT_TRUE(stoneContract.usesDefaultFrictionDirectionFrame);
    EXPECT_TRUE(stoneContract.localCenterOfMass.isZero(0.0));
    EXPECT_TRUE(stoneContract.linearVelocity.isZero(0.0));
    EXPECT_TRUE(stoneContract.angularVelocity.isZero(0.0));
    EXPECT_TRUE(
        stoneContract.collisionShapeRelativeTransform.matrix().isIdentity(0.0));
    const auto mesh
        = std::dynamic_pointer_cast<const dart::dynamics::ConvexMeshShape>(
            shapeNode->getShape());
    ASSERT_NE(mesh, nullptr);
    ASSERT_EQ(stoneContract.collisionMeshVertices.size(), 8u);
    ASSERT_EQ(stoneContract.collisionMeshTriangles.size(), triangles.size());
    for (std::size_t vertex = 0u; vertex < 8u; ++vertex) {
      EXPECT_TRUE(stoneContract.collisionMeshVertices[vertex].isApprox(
          geometries[index].vertices[vertex] - geometries[index].centroid,
          1e-15));
    }
    for (std::size_t triangle = 0u; triangle < triangles.size(); ++triangle) {
      EXPECT_EQ(
          stoneContract.collisionMeshTriangles[triangle][0],
          static_cast<Eigen::Index>(triangles[triangle][0]));
      EXPECT_EQ(
          stoneContract.collisionMeshTriangles[triangle][1],
          static_cast<Eigen::Index>(triangles[triangle][1]));
      EXPECT_EQ(
          stoneContract.collisionMeshTriangles[triangle][2],
          static_cast<Eigen::Index>(triangles[triangle][2]));
    }

    const double expectedMass = kDensity * geometries[index].volume;
    EXPECT_DOUBLE_EQ(body->getInertia().getMass(), expectedMass);
    EXPECT_TRUE(body->getInertia().getMoment().isApprox(
        expectedMass * geometries[index].momentPerUnitMass, 0.0));
    EXPECT_TRUE(body->getWorldTransform().translation().isApprox(
        geometries[index].centroid - kDownwardShift * Eigen::Vector3d::UnitZ(),
        1e-15));
    EXPECT_TRUE(body->getWorldTransform().linear().isIdentity(0.0));
    EXPECT_DOUBLE_EQ(
        shapeNode->getDynamicsAspect()->getFrictionCoeff(), kFriction);
  }
}

//==============================================================================
TEST(FbfLiteralMasonryArchSpec, DemoPaletteIsVisualOnly)
{
  const auto noneWorld
      = createWorld(SolverLane::ExactFbf, VisualMode::None, 1u);
  const auto paletteWorld
      = createWorld(SolverLane::ExactFbf, VisualMode::DemoPalette, 1u);
  const auto noneContract = inspectPhysicsContract(noneWorld);
  const auto paletteContract = inspectPhysicsContract(paletteWorld);

  EXPECT_EQ(paletteContract.visualMode, VisualMode::DemoPalette);
  EXPECT_TRUE(paletteContract.groundHasVisualFloor);
  EXPECT_TRUE(paletteContract.visualFloorSize.isApprox(
      Eigen::Vector3d(0.9, 0.35, 0.012), 0.0));
  EXPECT_TRUE(paletteContract.visualFloorTranslation.isApprox(
      Eigen::Vector3d(0.0, 0.0, -0.006), 0.0));
  const auto paletteGround = paletteWorld->getSkeleton("ground_plane");
  ASSERT_NE(paletteGround, nullptr);
  const auto* floorVisual
      = paletteGround->getBodyNode(0u)
            ->getShapeNodeWith<dart::dynamics::VisualAspect>(0u);
  ASSERT_NE(floorVisual, nullptr);
  EXPECT_EQ(floorVisual->getCollisionAspect(), nullptr);
  EXPECT_EQ(floorVisual->getDynamicsAspect(), nullptr);
  ASSERT_EQ(paletteContract.stones.size(), kStoneCount);
  for (std::size_t index = 0u; index < kStoneCount; ++index) {
    EXPECT_TRUE(paletteContract.stones[index].hasVisualAspect);
    EXPECT_TRUE(
        paletteContract.stones[index].color.isApprox(stoneColor(index), 0.0));
  }

  PhysicsContract paletteWithoutVisuals = paletteContract;
  paletteWithoutVisuals.visualMode = VisualMode::None;
  paletteWithoutVisuals.groundHasVisualFloor = false;
  paletteWithoutVisuals.visualFloorSize.setZero();
  paletteWithoutVisuals.visualFloorTranslation.setZero();
  for (auto& stone : paletteWithoutVisuals.stones) {
    stone.hasVisualAspect = false;
    stone.color.setZero();
  }
  expectSameNonSolverPhysics(noneContract, paletteWithoutVisuals);
}

//==============================================================================
TEST(FbfLiteralMasonryArchSpec, SolverLanesShareNonSolverPhysics)
{
  const double initialErp
      = dart::constraint::ContactConstraint::getErrorReductionParameter();
  const auto exactWorld
      = createWorld(SolverLane::ExactFbf, VisualMode::None, 1u);
  const auto boxedWorld
      = createWorld(SolverLane::BoxedLcp, VisualMode::None, 1u);
  EXPECT_DOUBLE_EQ(
      dart::constraint::ContactConstraint::getErrorReductionParameter(),
      initialErp);

  const auto exact = inspectPhysicsContract(exactWorld);
  const auto boxed = inspectPhysicsContract(boxedWorld);
  EXPECT_EQ(exact.solverLane, SolverLane::ExactFbf);
  EXPECT_EQ(boxed.solverLane, SolverLane::BoxedLcp);
  EXPECT_TRUE(exact.exactOptions.has_value());
  EXPECT_FALSE(boxed.exactOptions.has_value());
  EXPECT_TRUE(exact.exactCrossStepOptions.has_value());
  EXPECT_FALSE(boxed.exactCrossStepOptions.has_value());
  EXPECT_TRUE(exact.exactColoredBlockGaussSeidelEnabled);
  EXPECT_TRUE(exact.exactParticipantAffinityEnabled);
  EXPECT_FALSE(boxed.exactColoredBlockGaussSeidelEnabled);
  EXPECT_FALSE(boxed.exactParticipantAffinityEnabled);
  EXPECT_TRUE(exact.splitImpulseEnabled);
  EXPECT_TRUE(boxed.splitImpulseEnabled);
  expectSameNonSolverPhysics(exact, boxed);
}

//==============================================================================
TEST(FbfLiteralMasonryArchSpec, PhysicsContractJsonPinsPhysicalState)
{
  ScopedContactErrorReductionParameter scopedErp;
  const auto exactNone = inspectPhysicsContract(
      createWorld(SolverLane::ExactFbf, VisualMode::None, 1u));
  const auto exactPalette = inspectPhysicsContract(
      createWorld(SolverLane::ExactFbf, VisualMode::DemoPalette, 1u));
  const auto boxed = inspectPhysicsContract(
      createWorld(SolverLane::BoxedLcp, VisualMode::None, 1u));
  EXPECT_EQ(
      physicalGeometryFingerprint(exactNone),
      kExpectedPhysicalGeometryFingerprint);
  EXPECT_EQ(
      physicalGeometryFingerprint(exactPalette),
      kExpectedPhysicalGeometryFingerprint);
  EXPECT_EQ(
      physicalGeometryFingerprint(boxed), kExpectedPhysicalGeometryFingerprint);

  const std::string exactJson = physicsContractJson(
      exactNone,
      "spec-sha",
      "implementation-sha",
      "geometry-sha",
      "solver-options-sha");
  EXPECT_EQ(
      exactJson,
      physicsContractJson(
          exactPalette,
          "spec-sha",
          "implementation-sha",
          "geometry-sha",
          "solver-options-sha"));
  EXPECT_NE(
      exactJson.find("\"schema_version\":\"dart.fbf_literal_masonry_arch_"
                     "physics_contract/v1\""),
      std::string::npos);
  EXPECT_NE(
      exactJson.find("\"source_binding\":{\"spec_sha256\":\"spec-sha\","
                     "\"implementation_sha256\":\"implementation-sha\","
                     "\"geometry_sha256\":\"geometry-sha\","
                     "\"solver_options_sha256\":\"solver-options-sha\"}"),
      std::string::npos);
  EXPECT_NE(
      exactJson.find("\"barrier_gap_policy\":\"omit_source_offsets\""),
      std::string::npos);
  EXPECT_NE(exactJson.find("\"observed_contact_erp\":0"), std::string::npos);
  EXPECT_NE(exactJson.find("\"lane\":\"exact_fbf\""), std::string::npos);
  EXPECT_NE(exactJson.find("\"exact_options\":{"), std::string::npos);
  EXPECT_NE(exactJson.find("\"cross_step_options\":{"), std::string::npos);
  EXPECT_NE(
      exactJson.find("\"physical_geometry_fingerprint\":{"), std::string::npos);
  EXPECT_EQ(exactJson.find("visual"), std::string::npos);

  const std::string boxedJson = physicsContractJson(
      boxed,
      "spec-sha",
      "implementation-sha",
      "geometry-sha",
      "solver-options-sha");
  EXPECT_NE(boxedJson.find("\"lane\":\"boxed_lcp\""), std::string::npos);
  EXPECT_NE(boxedJson.find("\"exact_options\":null"), std::string::npos);
  EXPECT_NE(boxedJson.find("\"cross_step_options\":null"), std::string::npos);
  EXPECT_THROW(
      physicsContractJson(
          exactNone,
          "",
          "implementation-sha",
          "geometry-sha",
          "solver-options-sha"),
      std::invalid_argument);
  EXPECT_THROW(
      physicsContractJson(
          exactNone, "spec-sha", "", "geometry-sha", "solver-options-sha"),
      std::invalid_argument);
}

} // namespace
